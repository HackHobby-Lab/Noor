/**
 * sd_card.h
 * SD Card operations interface
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>
#include "sdmmc_cmd.h"
#include "esp_err.h"
#include "config.h"   /* MAX_FOLDERS, MAX_WAV_FILES, PIN_NUM_* */

/* Initialise and mount the SD card. Returns true on success. */
bool sd_card_init(void);

/* Scan top-level folders under path into internal list */
void sd_scan_folders(const char *path);

/* Scan subfolders inside a prophet folder */
void sd_scan_subfolders(const char *folder_path);

/* Scan WAV files inside a folder */
void sd_scan_wav_files(const char *folder_path);

/* Counts */
int sd_get_folder_count(void);
int sd_get_subfolder_count(void);
int sd_get_wav_count(void);

/* Path accessors — return NULL if index out of range */
const char* sd_get_folder_path(int index);
const char* sd_get_subfolder_path(int index);
const char* sd_get_wav_path(int index);

/* Free internal lists */
void sd_free_folders(void);
void sd_free_subfolders(void);
void sd_free_wavs(void);

/* Raw card handle for USB MSC */
sdmmc_card_t* sd_get_card_handle(void);

/* OTA update from /sdcard/update.bin */
esp_err_t sdcard_ota_update(void);

#endif /* SD_CARD_H */