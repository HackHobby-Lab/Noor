/**
 * sd_card.h
 * SD Card operations interface
 */

#pragma once

#include <stdbool.h>
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "config.h"

/* -----------------------------------------------------------------------
 * Hardware init
 * ----------------------------------------------------------------------- */

/**
 * Initialise SPI bus + SD card hardware.
 * Does NOT mount FAT — that is done by usb_manager_init() via
 * tinyusb_msc_storage_mount("/sdcard").
 */
bool sd_card_init(void);

/** Return the raw card handle (used by usb_manager to register with MSC). */
sdmmc_card_t *sd_get_card_handle(void);

/**
 * Return the shared SD access mutex.
 *
 * This mutex serialises ALL FAT-layer access system-wide:
 *   - audio.c  fread() during playback
 *   - sd_card.c opendir/readdir/stat during folder scans
 *   - usb_msc  tud_msc_read10_cb / tud_msc_write10_cb sector I/O
 *
 * The mutex is created inside sd_card_init() and lives for the
 * lifetime of the application.  Returns NULL before sd_card_init()
 * has been called (callers must guard for NULL).
 */
SemaphoreHandle_t sd_get_access_mutex(void);

/* -----------------------------------------------------------------------
 * Folder scanning
 * ----------------------------------------------------------------------- */
void        sd_scan_folders(const char *path);
int         sd_get_folder_count(void);
const char *sd_get_folder_path(int index);
void        sd_free_folders(void);

/* -----------------------------------------------------------------------
 * Subfolder scanning
 * ----------------------------------------------------------------------- */
void        sd_scan_subfolders(const char *folder_path);
int         sd_get_subfolder_count(void);
const char *sd_get_subfolder_path(int index);
void        sd_free_subfolders(void);

/* -----------------------------------------------------------------------
 * WAV file scanning
 * ----------------------------------------------------------------------- */
void        sd_scan_wav_files(const char *folder_path);
int         sd_get_wav_count(void);
const char *sd_get_wav_path(int index);
void        sd_free_wavs(void);
