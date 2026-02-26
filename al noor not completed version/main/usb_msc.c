/**
 * usb_msc.c
 * USB Mass Storage Class (MSC) using TinyUSB
 *
 * Updated behavior:
 * - Not initialized at boot.
 * - Started on-demand via usb_msc_start(card) (e.g., HOME long-press).
 * - Intended to run until hardware reset.
 *
 * IMPORTANT:
 * - When MSC is active, do NOT access SD card filesystem from the firmware.
 */

#include "usb_msc.h"
#include "esp_log.h"
#include "esp_err.h"

#include "tinyusb.h"
#include "class/msc/msc_device.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "USB_MSC";

static volatile bool s_usb_connected = false;
static volatile bool s_usb_active = false;
static sdmmc_card_t *s_sd_card = NULL;
static SemaphoreHandle_t s_sd_mutex = NULL;

// ---------------- TinyUSB device callbacks ----------------

void tud_mount_cb(void)
{
    s_usb_connected = true;
    ESP_LOGI(TAG, "USB CONNECTED (MSC enumerated)");
}

void tud_umount_cb(void)
{
    s_usb_connected = false;
    ESP_LOGI(TAG, "USB DISCONNECTED");
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
}

void tud_resume_cb(void)
{
}

// ---------------- TinyUSB MSC callbacks ----------------

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    const char vid[] = "Espressif";
    const char pid[] = "NoorPlayer";
    const char rev[] = "1.0";

    memset(vendor_id, ' ', 8);
    memset(product_id, ' ', 16);
    memset(product_rev, ' ', 4);

    memcpy(vendor_id, vid, sizeof(vid) - 1);
    memcpy(product_id, pid, sizeof(pid) - 1);
    memcpy(product_rev, rev, sizeof(rev) - 1);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return (s_sd_card != NULL);
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun;
    (void)power_condition;
    (void)start;
    (void)load_eject;
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_size = 512;

    if (!s_sd_card) {
        *block_count = 0;
        return;
    }

    // sdmmc_card_t::csd.capacity is in 512-byte sectors for SDSPI in ESP-IDF
    *block_count = s_sd_card->csd.capacity;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    (void)offset;

    if (!s_sd_card || !buffer || bufsize == 0) {
        return 0;
    }

    uint32_t sectors = (bufsize + 511) / 512;

    if (lba >= s_sd_card->csd.capacity) {
        return 0;
    }
    if (lba + sectors > s_sd_card->csd.capacity) {
        sectors = s_sd_card->csd.capacity - lba;
    }

    if (s_sd_mutex && xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        esp_err_t ret = sdmmc_read_sectors(s_sd_card, buffer, lba, sectors);
        xSemaphoreGive(s_sd_mutex);
        if (ret == ESP_OK) {
            return (int32_t)(sectors * 512);
        }
        ESP_LOGW(TAG, "SD read error LBA=%lu: %s", (unsigned long)lba, esp_err_to_name(ret));
    }

    return 0;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    (void)offset;

    if (!s_sd_card || !buffer || bufsize == 0) {
        return 0;
    }

    uint32_t sectors = (bufsize + 511) / 512;

    if (lba >= s_sd_card->csd.capacity) {
        return 0;
    }
    if (lba + sectors > s_sd_card->csd.capacity) {
        sectors = s_sd_card->csd.capacity - lba;
    }

    if (s_sd_mutex && xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(250)) == pdTRUE) {
        esp_err_t ret = sdmmc_write_sectors(s_sd_card, buffer, lba, sectors);
        xSemaphoreGive(s_sd_mutex);
        if (ret == ESP_OK) {
            return (int32_t)(sectors * 512);
        }
        ESP_LOGW(TAG, "SD write error LBA=%lu: %s", (unsigned long)lba, esp_err_to_name(ret));
    }

    return 0;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    (void)lun;
    (void)scsi_cmd;
    (void)buffer;
    (void)bufsize;
    return -1;
}

// ---------------- Public API ----------------

bool usb_msc_start(sdmmc_card_t *card)
{
    if (s_usb_active) {
        ESP_LOGW(TAG, "USB MSC already active");
        return true;
    }

    if (!card) {
        ESP_LOGE(TAG, "usb_msc_start(): SD card handle is NULL");
        return false;
    }

    s_sd_card = card;

    if (!s_sd_mutex) {
        s_sd_mutex = xSemaphoreCreateMutex();
        if (!s_sd_mutex) {
            ESP_LOGE(TAG, "Failed to create SD mutex");
            return false;
        }
    }

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .string_descriptor_count = 0,
        .external_phy = false,
        .self_powered = false,
        .vbus_monitor_io = -1,
    };

    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_driver_install failed: %s", esp_err_to_name(ret));
        return false;
    }

    s_usb_active = true;
    ESP_LOGI(TAG, "USB MSC started: SD is now exposed to host");
    return true;
}

bool usb_msc_is_active(void)
{
    return s_usb_active;
}

bool usb_msc_is_connected(void)
{
    return s_usb_connected;
}

const char* usb_msc_get_status(void)
{
    if (!s_usb_active) return "MSC inactive";
    if (!s_usb_connected) return "MSC active (waiting host)";
    return "MSC active (host connected)";
}
