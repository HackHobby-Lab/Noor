/**
 * sd_card.c
 * SD Card operations implementation
 *
 * Changes vs previous version:
 * ----------------------------
 * 1. Added sd_mutex (SemaphoreHandle_t) created in sd_card_init().
 * 2. sd_get_access_mutex() exposes the mutex to audio.c and usb_msc.c
 *    so ALL SD card access is serialised through one shared lock.
 * 3. sd_mutex is taken/given around every fopen-level VFS call that
 *    originates from this module (scan functions use opendir/readdir
 *    which go through the same FAT layer as fread, so they are also
 *    protected).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <inttypes.h>
#include "config.h"
#include "sd_card.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_system.h"
#include "esp_ota_ops.h"

static const char *TAG = "SD_CARD";

/* ------------------------------------------------------------------ */
/* Shared SD access mutex                                              */
/* All callers (audio.c fread, usb_msc.c sdmmc_read/write_sectors)   */
/* must hold this mutex while accessing the card.                     */
/* ------------------------------------------------------------------ */
static SemaphoreHandle_t sd_mutex = NULL;

SemaphoreHandle_t sd_get_access_mutex(void) {
    return sd_mutex;
}

/* Internal storage for folder and file lists */
static char *folder_list[MAX_FOLDERS];
static int num_folders = 0;

static char *subfolder_list[MAX_FOLDERS];
static int num_subfolders = 0;

static char *wav_list[MAX_WAV_FILES];
static int num_wavs = 0;

/* SD card handle */
static sdmmc_card_t *sd_card = NULL;

/* Helper function to create full path from directory and filename */
static char* make_full_path(const char *dir, const char *name) {
    size_t dir_len = strlen(dir);
    size_t name_len = strlen(name);
    size_t total_len = dir_len + 1 + name_len + 1;

    char *full_path = malloc(total_len);
    if (!full_path) {
        ESP_LOGE(TAG, "Failed to allocate memory for path");
        return NULL;
    }

    memcpy(full_path, dir, dir_len);
    full_path[dir_len] = '/';
    memcpy(full_path + dir_len + 1, name, name_len);
    full_path[total_len - 1] = '\0';

    return full_path;
}

/* Check if entry is a directory */
static bool is_directory(const char *parent_path, struct dirent *entry) {
    if (entry->d_type == DT_DIR) {
        return true;
    }
    if (entry->d_type == DT_UNKNOWN) {
        char *full_path = make_full_path(parent_path, entry->d_name);
        if (!full_path) return false;
        struct stat sb;
        bool result = (stat(full_path, &sb) == 0 && S_ISDIR(sb.st_mode));
        free(full_path);
        return result;
    }
    return false;
}

/* Check if entry is a regular file */
static bool is_regular_file(const char *parent_path, struct dirent *entry) {
    if (entry->d_type == DT_REG) {
        return true;
    }
    if (entry->d_type == DT_UNKNOWN) {
        char *full_path = make_full_path(parent_path, entry->d_name);
        if (!full_path) return false;
        struct stat sb;
        bool result = (stat(full_path, &sb) == 0 && S_ISREG(sb.st_mode));
        free(full_path);
        return result;
    }
    return false;
}

/*
 * Numerical-aware WAV path comparator for qsort.
 *
 * Compares by leading alpha prefix (case-insensitive) then by the
 * numeric suffix parsed as an integer, so:
 *   m1 < m2 < m3 ... < m9 < m10
 *   TT1 < TT2 ... < TT9 < TT10 ... < TT20
 *
 * This corrects the FAT32 alphabetical order (which gives TT10 before TT2).
 */
static int compare_wav_numerical(const void *a, const void *b) {
    const char *pa = *(const char *const *)a;
    const char *pb = *(const char *const *)b;

    /* Use filename only (portion after last '/') */
    const char *fa = strrchr(pa, '/'); fa = fa ? fa + 1 : pa;
    const char *fb = strrchr(pb, '/'); fb = fb ? fb + 1 : pb;

    /* Walk matching alpha prefix characters (case-insensitive) */
    size_t i = 0;
    while (fa[i] && fb[i] &&
           isalpha((unsigned char)fa[i]) &&
           isalpha((unsigned char)fb[i])) {
        int d = toupper((unsigned char)fa[i]) - toupper((unsigned char)fb[i]);
        if (d != 0) return d;
        i++;
    }

    /* If both are now at a digit, compare the numeric part as integers */
    if (isdigit((unsigned char)fa[i]) && isdigit((unsigned char)fb[i])) {
        int na = atoi(fa + i);
        int nb = atoi(fb + i);
        if (na != nb) return na - nb;
    }

    /* Fallback: full case-insensitive lexicographic comparison */
    return strcasecmp(fa, fb);
}

/* Check if filename ends with .wav (case-insensitive) */
static bool is_wav_file(const char *filename) {
    size_t len = strlen(filename);
    if (len <= 4) return false;
    const char *ext = filename + (len - 4);
    return (strcasecmp(ext, ".wav") == 0);
}

/*
 * Check if a WAV file is a navigable story file (not a quiz support file).
 * Quiz support files always contain an underscore (e.g. sa1_1.wav, crra2_3.wav).
 * Story files never contain underscores (a1.wav, ib10.wav, m3.wav, T5.wav …).
 * Note: TAWID lesson files are T1.wav..T20.wav (single T prefix).
 *       TAWID announcement files (TT1.wav..TT20.wav) live in announcements/.
 */
static bool is_story_wav_file(const char *filename) {
    if (!is_wav_file(filename)) return false;
    if (strchr(filename, '_') != NULL) return false;  /* quiz support file */
    return true;
}

bool sd_card_init(void) {
    ESP_LOGI(TAG, "Initializing SD card...");
    ESP_LOGI(TAG, "SPI Pins - CS=%d, MOSI=%d, MISO=%d, CLK=%d",
             PIN_NUM_CS, PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK);

    /* Create the shared SD mutex before any card access */
    sd_mutex = xSemaphoreCreateMutex();
    if (!sd_mutex) {
        ESP_LOGE(TAG, "Failed to create SD access mutex");
        return false;
    }
    ESP_LOGI(TAG, "SD access mutex created");

    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };

    ESP_LOGI(TAG, "Initializing SPI bus...");
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "SPI bus initialized successfully");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    ESP_LOGI(TAG, "Mounting SD card to /sdcard...");
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SD card mounted successfully!");
    if (sd_card) {
        ESP_LOGI(TAG, "  - Capacity: %" PRIu64 " bytes",
                 ((uint64_t)sd_card->csd.capacity) * sd_card->csd.sector_size);
        ESP_LOGI(TAG, "  - Sectors: %" PRIu32, (uint32_t)sd_card->csd.capacity);
        ESP_LOGI(TAG, "  - Sector Size: %d bytes", sd_card->csd.sector_size);
    }

    return true;
}

/* ------------------------------------------------------------------ */
/* Free helpers                                                        */
/* ------------------------------------------------------------------ */

void sd_free_folders(void) {
    for (int i = 0; i < num_folders; i++) {
        if (folder_list[i]) { free(folder_list[i]); folder_list[i] = NULL; }
    }
    num_folders = 0;
}

void sd_free_wavs(void) {
    for (int i = 0; i < num_wavs; i++) {
        if (wav_list[i]) { free(wav_list[i]); wav_list[i] = NULL; }
    }
    num_wavs = 0;
}

void sd_free_subfolders(void) {
    for (int i = 0; i < num_subfolders; i++) {
        if (subfolder_list[i]) { free(subfolder_list[i]); subfolder_list[i] = NULL; }
    }
    num_subfolders = 0;
}

/* ------------------------------------------------------------------ */
/* Scan functions — mutex held for the entire directory traversal     */
/* because opendir/readdir/stat all touch FAT structures on the card. */
/* ------------------------------------------------------------------ */

void sd_scan_folders(const char *path) {
    ESP_LOGI(TAG, "Scanning folders in: %s", path);
    sd_free_folders();

    if (sd_mutex) xSemaphoreTake(sd_mutex, portMAX_DELAY);

    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", path);
        if (sd_mutex) xSemaphoreGive(sd_mutex);
        return;
    }

    struct dirent *entry;
    int found = 0;

    while ((entry = readdir(dir)) != NULL && found < MAX_FOLDERS) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;

        if (is_directory(path, entry)) {
            char *full_path = make_full_path(path, entry->d_name);
            if (!full_path) break;
            folder_list[found] = full_path;
            ESP_LOGI(TAG, "  [%d] %s", found, full_path);
            found++;
        }
    }

    closedir(dir);
    if (sd_mutex) xSemaphoreGive(sd_mutex);

    num_folders = found;
    ESP_LOGI(TAG, "Found %d folders", num_folders);
}

void sd_scan_wav_files(const char *folder_path) {
    ESP_LOGI(TAG, "Scanning WAV files in: %s", folder_path);
    sd_free_wavs();

    if (sd_mutex) xSemaphoreTake(sd_mutex, portMAX_DELAY);

    DIR *dir = opendir(folder_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open folder: %s", folder_path);
        if (sd_mutex) xSemaphoreGive(sd_mutex);
        return;
    }

    struct dirent *entry;
    int found = 0;

    while ((entry = readdir(dir)) != NULL && found < MAX_WAV_FILES) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;

        if (!is_regular_file(folder_path, entry)) continue;

        if (is_story_wav_file(entry->d_name)) {
            char *full_path = make_full_path(folder_path, entry->d_name);
            if (!full_path) break;
            wav_list[found] = full_path;
            ESP_LOGI(TAG, "  [%d] %s", found, full_path);
            found++;
        }
    }

    closedir(dir);
    if (sd_mutex) xSemaphoreGive(sd_mutex);

    /* Sort numerically: m1<m2<...<m10, TT1<TT2<...<TT20.
     * FAT32 readdir() returns files in alphabetical order which puts
     * TT10 before TT2 — qsort with compare_wav_numerical fixes this. */
    if (found > 1)
        qsort(wav_list, (size_t)found, sizeof(char *), compare_wav_numerical);

    num_wavs = found;
    ESP_LOGI(TAG, "Found %d WAV files", num_wavs);
}

void sd_scan_subfolders(const char *folder_path) {
    sd_free_subfolders();

    if (sd_mutex) xSemaphoreTake(sd_mutex, portMAX_DELAY);

    DIR *dir = opendir(folder_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open folder: %s", folder_path);
        if (sd_mutex) xSemaphoreGive(sd_mutex);
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && num_subfolders < MAX_FOLDERS) {
        if (entry->d_name[0] == '.') continue;

        if (is_directory(folder_path, entry)) {
            char *full_path = make_full_path(folder_path, entry->d_name);
            if (full_path) {
                subfolder_list[num_subfolders] = full_path;
                num_subfolders++;
                ESP_LOGI(TAG, "Found subfolder [%d]: %s", num_subfolders - 1, entry->d_name);
            }
        }
    }

    closedir(dir);
    if (sd_mutex) xSemaphoreGive(sd_mutex);

    ESP_LOGI(TAG, "Total subfolders found: %d", num_subfolders);
}

/* ------------------------------------------------------------------ */
/* Public getters                                                      */
/* ------------------------------------------------------------------ */

int sd_get_folder_count(void)    { return num_folders; }
int sd_get_subfolder_count(void) { return num_subfolders; }
int sd_get_wav_count(void)       { return num_wavs; }

const char* sd_get_folder_path(int index) {
    if (index < 0 || index >= num_folders) return NULL;
    return folder_list[index];
}

const char* sd_get_subfolder_path(int index) {
    if (index < 0 || index >= num_subfolders) return NULL;
    return subfolder_list[index];
}

const char* sd_get_wav_path(int index) {
    if (index < 0 || index >= num_wavs) return NULL;
    return wav_list[index];
}

sdmmc_card_t* sd_get_card_handle(void) {
    return sd_card;
}

/* ------------------------------------------------------------------ */
/* OTA via SD card (update.bin)                                       */
/* ------------------------------------------------------------------ */

static const char *OTA_TAG = "SD_OTA";

esp_err_t sdcard_ota_update(void) {
    ESP_LOGI(OTA_TAG, "Checking SD card for update.bin...");

    if (!sd_card) {
        ESP_LOGE(OTA_TAG, "SD card not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    if (sd_mutex) xSemaphoreTake(sd_mutex, portMAX_DELAY);
    FILE *f = fopen("/sdcard/update.bin", "rb");
    if (sd_mutex) xSemaphoreGive(sd_mutex);

    if (!f) {
        ESP_LOGI(OTA_TAG, "No update.bin found on SD card.");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(OTA_TAG, "update.bin found, starting OTA...");

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(OTA_TAG, "No OTA partition available!");
        fclose(f);
        return ESP_FAIL;
    }

    esp_ota_handle_t update_handle;
    esp_err_t ret = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(OTA_TAG, "esp_ota_begin failed: %s", esp_err_to_name(ret));
        fclose(f);
        return ret;
    }

    uint8_t buf[1024];
    size_t read_bytes;
    while (1) {
        if (sd_mutex) xSemaphoreTake(sd_mutex, portMAX_DELAY);
        read_bytes = fread(buf, 1, sizeof(buf), f);
        if (sd_mutex) xSemaphoreGive(sd_mutex);

        if (read_bytes == 0) break;

        ret = esp_ota_write(update_handle, buf, read_bytes);
        if (ret != ESP_OK) {
            ESP_LOGE(OTA_TAG, "esp_ota_write failed: %s", esp_err_to_name(ret));
            fclose(f);
            esp_ota_abort(update_handle);
            return ret;
        }
    }

    fclose(f);

    ret = esp_ota_end(update_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(OTA_TAG, "esp_ota_end failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ota_set_boot_partition(update_partition);
    if (ret != ESP_OK) {
        ESP_LOGE(OTA_TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(OTA_TAG, "OTA update complete! Rebooting...");
    remove("/sdcard/update.bin");
    esp_restart();

    return ESP_OK; /* never reached */
}
