/**
 * SD_CARD_OTA.h
 * SD card firmware update (OTA) interface
 *
 * Place a file named "firmware.bin" in the root of the SD card.
 * Call check_sd_ota_update() once after sd_card_init() succeeds.
 * If the file is found it will be flashed to the next OTA partition
 * and the device will reboot.  The file is deleted first so the
 * device does not re-flash itself on every subsequent boot.
 */

#ifndef SD_CARD_OTA_H
#define SD_CARD_OTA_H

/**
 * Check for /sdcard/firmware.bin and apply OTA update if found.
 * Does NOT return if an update is applied (calls esp_restart()).
 * Returns silently if no update file is present.
 */
void check_sd_ota_update(void);

#endif /* SD_CARD_OTA_H */
