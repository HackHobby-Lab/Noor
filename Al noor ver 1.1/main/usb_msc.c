/**
 * usb_msc.c
 * USB Mass Storage Class (MSC) implementation using TinyUSB
 *
 * Changes vs previous version:
 * ----------------------------
 * 1. Removed the private sd_access_mutex — it only protected USB MSC
 *    callbacks against each other but did nothing to prevent contention
 *    with audio.c fread() calls hitting the FAT layer at the same time.
 * 2. All tud_msc_read10_cb / tud_msc_write10_cb calls now acquire the
 *    shared mutex from sd_get_access_mutex() (owned by sd_card.c).
 *    This serialises every sector-level SD access system-wide:
 *      - USB host reads/writes
 *      - audio.c fread() during playback
 *      - sd_card.c opendir/readdir/stat during folder scans
 * 3. usb_msc_init() no longer creates its own mutex.
 * 4. #include "sd_card.h" added for sd_get_access_mutex().
 */

#include "usb_msc.h"
#include "sd_card.h"         /* sd_get_access_mutex() */
#include "esp_log.h"
#include "tinyusb.h"
#include "class/msc/msc_device.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static const char *TAG = "USB_MSC";

/* USB connection state tracking */
static volatile bool usb_connected = false;
static volatile bool usb_mounted   = false;
static sdmmc_card_t *sd_card       = NULL;

/* -------------------------------------------------------------------------
 * TinyUSB Device Callbacks
 * ---------------------------------------------------------------------- */

void tud_mount_cb(void) {
    usb_connected = true;
    ESP_LOGI(TAG, "USB CONNECTED - Device detected by host");
}

void tud_umount_cb(void) {
    usb_connected = false;
    usb_mounted   = false;
    ESP_LOGI(TAG, "USB DISCONNECTED from host");
}

void tud_suspend_cb(bool remote_wakeup_en) {
    ESP_LOGD(TAG, "USB suspended");
}

void tud_resume_cb(void) {
    ESP_LOGD(TAG, "USB resumed");
}

/* -------------------------------------------------------------------------
 * TinyUSB MSC Callbacks
 * ---------------------------------------------------------------------- */

void tud_msc_inquiry_cb(uint8_t lun,
                        uint8_t vendor_id[8],
                        uint8_t product_id[16],
                        uint8_t product_rev[4])
{
    const char vid[] = "Espressif";
    const char pid[] = "Noor Player";
    const char rev[] = "1.0";

    memcpy(vendor_id,   vid, sizeof(vid) - 1);
    memcpy(product_id,  pid, sizeof(pid) - 1);
    memcpy(product_rev, rev, sizeof(rev) - 1);
}

bool tud_msc_start_stop_cb(uint8_t lun,
                            uint8_t power_condition,
                            bool start,
                            bool load_eject)
{
    return true;
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    return (sd_card != NULL);
}

void tud_msc_capacity_cb(uint8_t lun,
                          uint32_t *block_count,
                          uint16_t *block_size)
{
    if (sd_card) {
        *block_count = sd_card->csd.capacity;
        *block_size  = 512;
        ESP_LOGD(TAG, "USB MSC Capacity: %lu blocks", *block_count);
    } else {
        *block_count = 0;
        *block_size  = 512;
    }
}

int32_t tud_msc_read10_cb(uint8_t lun,
                           uint32_t lba,
                           uint32_t offset,
                           void    *buffer,
                           uint32_t bufsize)
{
    if (!sd_card || !buffer || bufsize == 0) return 0;

    uint32_t num_sectors = (bufsize + 511) / 512;

    if (lba >= sd_card->csd.capacity) return 0;
    if (lba + num_sectors > sd_card->csd.capacity)
        num_sectors = sd_card->csd.capacity - lba;

    /* Use the shared SD mutex — same lock held by audio.c fread() and
     * sd_card.c scan functions so all FAT access is fully serialised. */
    SemaphoreHandle_t m = sd_get_access_mutex();
    if (m && xSemaphoreTake(m, pdMS_TO_TICKS(200)) == pdTRUE) {
        esp_err_t ret = sdmmc_read_sectors(sd_card, buffer, lba, num_sectors);
        xSemaphoreGive(m);

        if (ret == ESP_OK) {
            return (int32_t)(num_sectors * 512);
        } else {
            ESP_LOGW(TAG, "SD read error at LBA %" PRIu32 ": %s",
                     lba, esp_err_to_name(ret));
            return 0;
        }
    }

    ESP_LOGW(TAG, "SD mutex timeout on read (LBA %" PRIu32 ")", lba);
    return 0;
}

int32_t tud_msc_write10_cb(uint8_t  lun,
                            uint32_t lba,
                            uint32_t offset,
                            uint8_t *buffer,
                            uint32_t bufsize)
{
    if (!sd_card || !buffer || bufsize == 0) return 0;

    uint32_t num_sectors = (bufsize + 511) / 512;

    if (lba >= sd_card->csd.capacity) return 0;
    if (lba + num_sectors > sd_card->csd.capacity)
        num_sectors = sd_card->csd.capacity - lba;

    SemaphoreHandle_t m = sd_get_access_mutex();
    if (m && xSemaphoreTake(m, pdMS_TO_TICKS(200)) == pdTRUE) {
        esp_err_t ret = sdmmc_write_sectors(sd_card, buffer, lba, num_sectors);
        xSemaphoreGive(m);

        if (ret == ESP_OK) {
            return (int32_t)(num_sectors * 512);
        } else {
            ESP_LOGW(TAG, "SD write error at LBA %" PRIu32 ": %s",
                     lba, esp_err_to_name(ret));
            return 0;
        }
    }

    ESP_LOGW(TAG, "SD mutex timeout on write (LBA %" PRIu32 ")", lba);
    return 0;
}

int32_t tud_msc_scsi_cb(uint8_t lun,
                         uint8_t const scsi_cmd[16],
                         void   *buffer,
                         uint16_t bufsize)
{
    return -1; /* Unsupported command */
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void usb_msc_set_sd_card(sdmmc_card_t *card) {
    sd_card = card;
    if (card) {
        ESP_LOGI(TAG, "SD card registered with USB MSC");
        ESP_LOGI(TAG, "- Capacity: %llu bytes",
                 (unsigned long long)card->csd.capacity * 512);
        ESP_LOGI(TAG, "- Sectors: %" PRIu32, (uint32_t)card->csd.capacity);
    }
}

bool usb_msc_init(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing USB MSC (Mass Storage Class)");
    ESP_LOGI(TAG, "========================================");

    /* NOTE: No private mutex created here any more.
     * The shared mutex in sd_card.c (created by sd_card_init) is used
     * by tud_msc_read10_cb / tud_msc_write10_cb instead.
     * usb_msc_init() is called BEFORE sd_card_init(), so the mutex
     * will not exist yet at this point — that is fine because no SD
     * access happens until the host actually sends a READ/WRITE command,
     * which only occurs after sd_card_init() has completed and the
     * card handle has been registered via usb_msc_set_sd_card(). */

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor       = NULL,
        .string_descriptor       = NULL,
        .string_descriptor_count = 0,
        .external_phy            = false,
        .self_powered            = false,
        .vbus_monitor_io         = -1
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB driver install failed: 0x%x", ret);
        return false;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "USB MSC initialized successfully!");
    ESP_LOGI(TAG, "- Connect second USB-C cable to ESP32S3");
    ESP_LOGI(TAG, "- SD card will appear as removable USB storage");
    ESP_LOGI(TAG, "- Audio playback continues on first USB (UART)");
    ESP_LOGI(TAG, "========================================");

    return true;
}

bool usb_msc_is_connected(void) {
    return usb_connected;
}

bool usb_msc_is_mounted(void) {
    return usb_mounted;
}

const char* usb_msc_get_status(void) {
    if (!usb_connected) return "USB Disconnected - Connect second USB cable";
    if (usb_mounted)    return "USB MSC Mounted - SD card accessible";
    return "USB Connected - SD card mounting";
}
