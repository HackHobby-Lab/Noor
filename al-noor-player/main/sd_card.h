/**
 * sd_card.h
 * SD Card operations for Noor Audio Player
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>
#include "sdmmc_cmd.h"

#define PIN_NUM_MISO  13
#define PIN_NUM_MOSI  11
#define PIN_NUM_CLK   12
#define PIN_NUM_CS    10

#define MAX_FOLDERS    32
#define MAX_WAV_FILES  64

bool        sd_card_init(void);

void        sd_scan_folders(const char *path);
int         sd_get_folder_count(void);
const char *sd_get_folder_path(int index);
void        sd_free_folders(void);

void        sd_scan_subfolders(const char *folder_path);
int         sd_get_subfolder_count(void);
const char *sd_get_subfolder_path(int index);
void        sd_free_subfolders(void);

void        sd_scan_wav_files(const char *folder_path);
int         sd_get_wav_count(void);
const char *sd_get_wav_path(int index);
void        sd_free_wavs(void);

sdmmc_card_t *sd_get_card_handle(void);

#endif // SD_CARD_H
