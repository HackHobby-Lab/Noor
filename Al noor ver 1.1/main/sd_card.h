/**
 * sd_card.h
 * SD Card operations for Noor Audio Player
 * 
 * This module handles:
 * - SD card mounting via SPI
 * - Scanning folders and WAV files
 * - Managing folder/file lists
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>
#include "sdmmc_cmd.h"

/* Pin definitions for SD card SPI interface */
#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

/* Maximum number of folders and files we can track */
#define MAX_FOLDERS   32
#define MAX_WAV_FILES 64

/**
 * Initialize SD card and mount it at /sdcard
 * 
 * @return true if successful, false if failed
 */
bool sd_card_init(void);

/**
 * Scan the root directory for folders
 * This will populate the internal folder list
 * 
 * @param path Root path to scan (typically "/sdcard")
 */
void sd_scan_folders(const char *path);

/**
 * Scan a specific folder for WAV files
 * This will populate the internal WAV file list
 * 
 * @param folder_path Path to the folder to scan
 */
void sd_scan_wav_files(const char *folder_path);

/**
 * Get the number of folders found
 * 
 * @return Number of folders in the list
 */
int sd_get_folder_count(void);

/**
 * Scan a folder for subfolders
 * This will populate the internal subfolder list
 * 
 * @param folder_path Path to the folder to scan for subfolders
 */
void sd_scan_subfolders(const char *folder_path);

/**
 * Get the number of subfolders found
 * 
 * @return Number of subfolders in the list
 */
int sd_get_subfolder_count(void);

/**
 * Get the path of a specific subfolder by index
 * 
 * @param index Subfolder index (0 to subfolder_count-1)
 * @return Full path string, or NULL if invalid index
 */
const char* sd_get_subfolder_path(int index);

/**
 * Free all allocated memory for subfolder list
 * Call this when leaving subfolder view
 */
void sd_free_subfolders(void);

/**
 * Get the number of WAV files found in current folder
 * 
 * @return Number of WAV files in the list
 */
int sd_get_wav_count(void);

/**
 * Get the path of a specific folder by index
 * 
 * @param index Folder index (0 to folder_count-1)
 * @return Full path string, or NULL if invalid index
 */
const char* sd_get_folder_path(int index);

/**
 * Get the path of a specific WAV file by index
 * 
 * @param index WAV file index (0 to wav_count-1)
 * @return Full path string, or NULL if invalid index
 */
const char* sd_get_wav_path(int index);

/**
 * Free all allocated memory for folder list
 * Call this when leaving folder view
 */
void sd_free_folders(void);

/**
 * Free all allocated memory for WAV file list
 * Call this when leaving file view
 */
void sd_free_wavs(void);

/**
 * Get the SD card handle (for advanced operations)
 * 
 * @return Pointer to SD card structure
 */
sdmmc_card_t* sd_get_card_handle(void);

#endif // SD_CARD_H