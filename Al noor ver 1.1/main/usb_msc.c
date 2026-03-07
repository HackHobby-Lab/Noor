/**
 * usb_msc.c
 * USB Mass Storage Class (MSC) implementation using TinyUSB
 * Exposes SD card as USB storage device while maintaining UART operation
 */

#include "usb_msc.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "class/msc/msc_device.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static const char *TAG = "USB_MSC";

/* USB connection state tracking */
static volatile bool usb_connected = false;
static volatile bool usb_mounted = false;
static sdmmc_card_t *sd_card = NULL;
static SemaphoreHandle_t sd_access_mutex = NULL;

/**
 * TinyUSB Device Callbacks
 */

void tud_mount_cb(void) {
    usb_connected = true;
    ESP_LOGI(TAG, "USB CONNECTED - Device detected by host");
}

void tud_umount_cb(void) {
    usb_connected = false;
    usb_mounted = false;
    ESP_LOGI(TAG, "USB DISCONNECTED from host");
}

void tud_suspend_cb(bool remote_wakeup_en) {
    ESP_LOGD(TAG, "USB suspended");
}

void tud_resume_cb(void) {
    ESP_LOGD(TAG, "USB resumed");
}

/**
 * TinyUSB MSC Callbacks - Stub implementations
 * MSC functionality is handled by the ESP TinyUSB wrapper
 */

// Invoked when received SCSI_CMD_INQUIRY
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    const char vid[] = "Espressif";
    const char pid[] = "Noor Player";
    const char rev[] = "1.0";
    
    memcpy(vendor_id, vid, sizeof(vid)-1);
    memcpy(product_id, pid, sizeof(pid)-1);
    memcpy(product_rev, rev, sizeof(rev)-1);
}

// Invoked when received Start Stop Unit command
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    return true;
}

// Invoked when received Test Unit Ready command
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    return true;
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    if (sd_card) {
        // Report actual SD card capacity in 512-byte sectors
        *block_count = sd_card->csd.capacity;
        *block_size = 512;
        ESP_LOGD(TAG, "USB MSC Capacity Query: %lu blocks of %u bytes", *block_count, *block_size);
    } else {
        // No SD card - report 0 capacity
        *block_count = 0;
        *block_size = 512;
    }
}

// Invoked when received SCSI READ10 command
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    if (!sd_card || !buffer || bufsize == 0) {
        return 0;  // No data to read
    }
    
    // Calculate number of sectors (512 bytes each)
    uint32_t num_sectors = (bufsize + 511) / 512;
    
    // Protect against reading past end of card
    if (lba >= sd_card->csd.capacity) {
        return 0;
    }
    
    if (lba + num_sectors > sd_card->csd.capacity) {
        num_sectors = sd_card->csd.capacity - lba;
    }
    
    // Try to acquire mutex with short timeout to avoid blocking USB task
    if (sd_access_mutex && xSemaphoreTake(sd_access_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_err_t ret = sdmmc_read_sectors(sd_card, buffer, lba, num_sectors);
        xSemaphoreGive(sd_access_mutex);
        
        if (ret == ESP_OK) {
            return num_sectors * 512;
        } else {
            ESP_LOGW(TAG, "SD read error at LBA %lu: %s", lba, esp_err_to_name(ret));
            return 0;
        }
    } else {
        // Couldn't get mutex - return partial or no data
        return 0;
    }
}

// Invoked when received SCSI WRITE10 command
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    if (!sd_card || !buffer || bufsize == 0) {
        return 0;  // No data to write
    }
    
    // Calculate number of sectors (512 bytes each)
    uint32_t num_sectors = (bufsize + 511) / 512;
    
    // Protect against writing past end of card
    if (lba >= sd_card->csd.capacity) {
        return 0;
    }
    
    if (lba + num_sectors > sd_card->csd.capacity) {
        num_sectors = sd_card->csd.capacity - lba;
    }
    
    // Try to acquire mutex with short timeout to avoid blocking USB task
    if (sd_access_mutex && xSemaphoreTake(sd_access_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_err_t ret = sdmmc_write_sectors(sd_card, buffer, lba, num_sectors);
        xSemaphoreGive(sd_access_mutex);
        
        if (ret == ESP_OK) {
            return num_sectors * 512;
        } else {
            ESP_LOGW(TAG, "SD write error at LBA %lu: %s", lba, esp_err_to_name(ret));
            return 0;
        }
    } else {
        // Couldn't get mutex - return no write
        return 0;
    }
}

// Invoked when received an SCSI command not in built-in list
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    return -1; // Unsupported command
}

/**
 * Set SD card reference for USB MSC access
 */
void usb_msc_set_sd_card(sdmmc_card_t *card) {
    sd_card = card;
    if (card) {
        ESP_LOGI(TAG, "SD card registered with USB MSC");
        ESP_LOGI(TAG, "- Capacity: %llu bytes", (unsigned long long)card->csd.capacity * 512);
        ESP_LOGI(TAG, "- Sectors: %" PRIu32, (uint32_t)card->csd.capacity);
    }
}

bool usb_msc_init(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initializing USB MSC (Mass Storage Class)");
    ESP_LOGI(TAG, "========================================");
    
    // Create mutex for SD card access synchronization
    if (!sd_access_mutex) {
        sd_access_mutex = xSemaphoreCreateMutex();
        if (!sd_access_mutex) {
            ESP_LOGE(TAG, "Failed to create SD access mutex");
            return false;
        }
    }
    
    // Initialize TinyUSB device driver with MSC support
    // tusb_msc_storage component initializes MCB automatically
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .string_descriptor_count = 0,
        .external_phy = false,
        .self_powered = false,
        .vbus_monitor_io = -1
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
    if (!usb_connected) {
        return "USB Disconnected - Connect second USB cable";
    }
    if (usb_mounted) {
        return "USB MSC Mounted - SD card accessible";
    }
    return "USB Connected - SD card mounting";
}
