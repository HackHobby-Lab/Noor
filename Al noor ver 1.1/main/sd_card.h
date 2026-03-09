/**
 * sd_card.h
 * SD Card operations header
 *
 * Changes vs previous version:
 * ----------------------------
 * 1. Added sd_get_access_mutex() — returns the shared FreeRTOS mutex
 *    that serialises ALL SD card access across audio.c, usb_msc.c, and
 *    the scan functions in sd_card.c itself.
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* -------------------------------------------------------------------------
 * Shared SD access mutex
 *
 * Created in sd_card_init().  Every module that performs SD card I/O
 * (fopen, fread, fclose, opendir, readdir, sdmmc_read_sectors, etc.)
 * must hold this mutex for the duration of each individual operation.
 *
 * Pattern:
 *   SemaphoreHandle_t m = sd_get_access_mutex();
 *   if (m) xSemaphoreTake(m, portMAX_DELAY);
 *   // ... SD operation ...
 *   if (m) xSemaphoreGive(m);
 *
 * Returns NULL if sd_card_init() has not been called yet or failed.
 * ---------------------------------------------------------------------- */
SemaphoreHandle_t sd_get_access_mutex(void);

/* -------------------------------------------------------------------------
 * Initialisation
 * ---------------------------------------------------------------------- */
bool sd_card_init(void);

/* -------------------------------------------------------------------------
 * Folder / subfolder / WAV scanning
 * ---------------------------------------------------------------------- */
void sd_scan_folders(const char *path);
void sd_scan_subfolders(const char *folder_path);
void sd_scan_wav_files(const char *folder_path);

/* -------------------------------------------------------------------------
 * Free cached lists
 * ---------------------------------------------------------------------- */
void sd_free_folders(void);
void sd_free_subfolders(void);
void sd_free_wavs(void);

/* -------------------------------------------------------------------------
 * Count getters
 * ---------------------------------------------------------------------- */
int sd_get_folder_count(void);
int sd_get_subfolder_count(void);
int sd_get_wav_count(void);

/* -------------------------------------------------------------------------
 * Path getters — returned pointers are valid until the next free/scan call
 * ---------------------------------------------------------------------- */
const char* sd_get_folder_path(int index);
const char* sd_get_subfolder_path(int index);
const char* sd_get_wav_path(int index);

/* -------------------------------------------------------------------------
 * Raw card handle — used by usb_msc.c to register the card with TinyUSB
 * ---------------------------------------------------------------------- */
sdmmc_card_t* sd_get_card_handle(void);

#endif /* SD_CARD_H */
