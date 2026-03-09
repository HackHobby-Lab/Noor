/**
 * usb_manager.c
 * USB CDC + MSC composite device implementation
 *
 * SD card ownership model:
 *   - sd_card_init()               : hardware init only (SPI + card, no FAT)
 *   - usb_manager_init()           : initial FAT mount via tinyusb_msc_storage_mount()
 *   - premount callback (USB in)   : audio_stop_immediate(), free path lists
 *   - [TinyUSB internally]         : tinyusb_msc_storage_unmount() — FAT unmounted,
 *                                    SDSPI device handle preserved for raw MSC I/O
 *   - mount_changed (USB out)      : FAT remounted, notify audio task to rescan
 */

#include "usb_manager.h"
#include "sd_card.h"
#include "audio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "tusb_msc_storage.h"

static const char *TAG = "USB_MGR";

static volatile bool  s_msc_active      = false;
static volatile bool  s_boot_audio_done = false;
static TaskHandle_t   s_audio_task      = NULL;

/* ------------------------------------------------------------------ */
/*  USB Device Descriptor                                               */
/*  bDeviceClass = TUSB_CLASS_MISC (0xEF) with IAD is required for    */
/*  composite CDC+MSC — using TUSB_CLASS_CDC breaks MSC on Windows.   */
/* ------------------------------------------------------------------ */
static const tusb_desc_device_t s_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,   /* Espressif VID */
    .idProduct          = 0x4001,   /* AL-NOOR CDC+MSC composite */
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

/* ------------------------------------------------------------------ */
/*  Interface numbering                                                 */
/*  ITF 0,1 = CDC ACM (control + data)                                 */
/*  ITF 2   = MSC                                                       */
/* ------------------------------------------------------------------ */
#define ITF_NUM_CDC         0
#define ITF_NUM_CDC_DATA    1
#define ITF_NUM_MSC         2
#define ITF_NUM_TOTAL       3

#define EPNUM_CDC_NOTIF     0x81
#define EPNUM_CDC_OUT       0x02
#define EPNUM_CDC_IN        0x82
#define EPNUM_MSC_OUT       0x03
#define EPNUM_MSC_IN        0x83

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t s_config_desc[] = {
    /* Config descriptor: 1 configuration, ITF_NUM_TOTAL interfaces,
     * string idx 0, total length, remote-wakeup, 100mA              */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    /* CDC: control ITF, data ITF, string 4, notif EP/size, out/in EP, EP size */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    /* MSC: ITF, string 5, out EP, in EP, EP size */
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 5, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
};

static const char *s_string_desc[] = {
    "\x09\x04",           /* 0: English (0x0409) */
    "Al-Noor",            /* 1: Manufacturer */
    "AL-NOOR Player",     /* 2: Product */
    "ALNOOR-001",         /* 3: Serial number */
    "AL-NOOR CDC",        /* 4: CDC interface name */
    "AL-NOOR MSC",        /* 5: MSC interface name */
};

/* ------------------------------------------------------------------ */
/*  MSC Callbacks                                                       */
/* ------------------------------------------------------------------ */

/**
 * Called BEFORE TinyUSB changes FAT mount state.
 * is_mounted = true  → FAT is currently mounted locally, about to be given to USB host
 * is_mounted = false → FAT is currently unmounted (USB host had it), about to remount locally
 */
static void _premount_cb(tinyusb_msc_event_t *event)
{
    if (event->mount_changed_data.is_mounted) {
        /* On the very first USB connect, wait until the audio task signals
         * that boot audio (welcome.wav + rotate.wav) has finished, or up
         * to 15s maximum.  Subsequent connects proceed immediately.      */
        static bool s_first_connect = true;
        if (s_first_connect) {
            s_first_connect = false;
            TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(15000);
            while (!s_boot_audio_done && xTaskGetTickCount() < deadline) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        /* Local VFS is about to be unmounted so USB host can take the SD card.
         * Stop audio and free stale path lists before FAT unmounts. */
        ESP_LOGI(TAG, "MSC: USB host connecting, stopping audio");
        audio_stop_immediate();
        vTaskDelay(pdMS_TO_TICKS(120));  /* Wait for audio I/O to stop */
        sd_free_folders();
        sd_free_subfolders();
        sd_free_wavs();
    }
    /* If !is_mounted: FAT about to remount locally after USB eject — nothing to do here */
}

/**
 * Called AFTER TinyUSB changes FAT mount state.
 * is_mounted = false → FAT is now unmounted (SD card given to USB host)
 * is_mounted = true  → FAT is now remounted (SD card returned to local VFS)
 */
static void _mount_changed_cb(tinyusb_msc_event_t *event)
{
    if (!event->mount_changed_data.is_mounted) {
        /* FAT unmounted — USB host is now reading/writing the SD card */
        s_msc_active = true;
        ESP_LOGI(TAG, "MSC: SD card handed to USB host");
    } else {
        /* FAT remounted — USB host released the SD card */
        s_msc_active = false;
        ESP_LOGI(TAG, "MSC: SD card returned to local VFS, notifying audio task");
        if (s_audio_task) {
            xTaskNotify(s_audio_task, NOTIFY_USB_MSC_CHANGED_BIT, eSetBits);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void usb_manager_init(TaskHandle_t audio_task)
{
    s_audio_task = audio_task;

    sdmmc_card_t *card = sd_get_card_handle();
    if (!card) {
        ESP_LOGE(TAG, "SD card not initialized — cannot start USB MSC");
        return;
    }

    /* 1. Register MSC storage backend with the already-initialized card.
     *    The card handle's host interface (SDSPI) is used for raw sector
     *    I/O during USB host access. TinyUSB MSC storage manages all
     *    FAT mount/unmount from this point on.                          */
    esp_vfs_fat_mount_config_t msc_mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024,
    };
    tinyusb_msc_sdmmc_config_t msc_cfg = {
        .card                      = card,
        .callback_mount_changed    = _mount_changed_cb,
        .callback_premount_changed = _premount_cb,
        .mount_config              = msc_mount_cfg,
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_sdmmc(&msc_cfg));

    /* 2. Initial FAT mount — TinyUSB MSC storage takes ownership.
     *    All subsequent mount/unmount events (USB connect/disconnect)
     *    are handled automatically by the MSC storage driver.          */
    ESP_ERROR_CHECK(tinyusb_msc_storage_mount("/sdcard"));
    ESP_LOGI(TAG, "SD card mounted at /sdcard (via TinyUSB MSC storage)");

    /* 3. Install TinyUSB driver — starts the internal tusb_device_task */
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor        = &s_device_desc,
        .string_descriptor        = s_string_desc,
        .string_descriptor_count  = sizeof(s_string_desc) / sizeof(s_string_desc[0]),
        .external_phy             = false,
        .configuration_descriptor = s_config_desc,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    /* 4. Initialize CDC ACM interface 0 (the serial debug COM port) */
    const tinyusb_config_cdcacm_t cdc_cfg = {
        .usb_dev                    = TINYUSB_USBDEV_0,
        .cdc_port                   = TINYUSB_CDC_ACM_0,
        .callback_rx                = NULL,
        .callback_rx_wanted_char    = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&cdc_cfg));

    /* 5. Redirect ESP_LOG output to USB CDC.
     *    When no USB host is connected, CDC writes are silently dropped
     *    (non-blocking). UART0 console remains available in parallel
     *    via CONFIG_ESP_CONSOLE_UART_DEFAULT=y.                        */
    ESP_ERROR_CHECK(esp_tusb_init_console(TINYUSB_CDC_ACM_0));

    ESP_LOGI(TAG, "USB CDC+MSC composite initialized");
}

bool usb_manager_msc_active(void)
{
    return s_msc_active;
}

void usb_manager_set_boot_audio_done(void)
{
    s_boot_audio_done = true;
}

