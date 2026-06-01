#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <inttypes.h>
#include "SD_CARD_OTA.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"

static const char *TAG = "SD_OTA";

#define OTA_FILE_PATH "/sdcard/firmware.bin"

void check_sd_ota_update(void)
{
    ESP_LOGI(TAG, "Checking for OTA firmware update...");

    FILE *fw_file = fopen(OTA_FILE_PATH, "rb");
    if (fw_file == NULL) {
        ESP_LOGI(TAG, "No OTA file found on SD card.");
        return;
    }

    fseek(fw_file, 0, SEEK_END);
    long fw_size_signed = ftell(fw_file);
    fseek(fw_file, 0, SEEK_SET);

    if (fw_size_signed < 0) {
        ESP_LOGE(TAG, "ftell failed");
        fclose(fw_file);
        return;
    }

    size_t fw_size = (size_t)fw_size_signed;

    /* Get next OTA partition */
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
    if (!ota_partition) {
        ESP_LOGE(TAG, "No OTA partition available!");
        fclose(fw_file);
        return;
    }

    if (fw_size > ota_partition->size) {
        ESP_LOGE(TAG, "Firmware size (%zu) exceeds OTA partition (%" PRIu32 ")",
                 fw_size, ota_partition->size);
        fclose(fw_file);
        return;
    }

    ESP_LOGI(TAG, "Starting OTA update to partition '%s' (%zu bytes)",
             ota_partition->label, fw_size);

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(ota_partition, fw_size, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        fclose(fw_file);
        return;
    }

    uint8_t buffer[1024];
    size_t read_bytes;
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), fw_file)) > 0) {
        err = esp_ota_write(ota_handle, buffer, read_bytes);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_end(ota_handle);
            fclose(fw_file);
            return;
        }
    }

    fclose(fw_file);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "OTA update complete! Rebooting...");
    remove(OTA_FILE_PATH);   /* delete firmware.bin so we don't re-flash on next boot */
    esp_restart();
}
