/**
 * sd_card.c
 * SD Card operations implementation
 */

#include "sd_card.h"
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"

static const char *TAG = "SD_CARD";

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
    size_t total_len = dir_len + 1 + name_len + 1; // dir + '/' + name + '\0'
    
    char *full_path = malloc(total_len);
    if (!full_path) {
        ESP_LOGE(TAG, "Failed to allocate memory for path");
        return NULL;
    }
    
    // Build the path: dir/name
    memcpy(full_path, dir, dir_len);
    full_path[dir_len] = '/';
    memcpy(full_path + dir_len + 1, name, name_len);
    full_path[total_len - 1] = '\0';
    
    return full_path;
}

/* Check if entry is a directory */
static bool is_directory(const char *parent_path, struct dirent *entry) {
    // First try d_type (if available)
    if (entry->d_type == DT_DIR) {
        return true;
    }
    
    // If d_type is unknown, use stat()
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
    // First try d_type (if available)
    if (entry->d_type == DT_REG) {
        return true;
    }
    
    // If d_type is unknown, use stat()
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

/* Check if filename ends with .wav (case-insensitive) */
static bool is_wav_file(const char *filename) {
    size_t len = strlen(filename);
    if (len <= 4) return false;
    
    const char *ext = filename + (len - 4);
    return (strcasecmp(ext, ".wav") == 0);
}

bool sd_card_init(void) {
    ESP_LOGI(TAG, "Initializing SD card...");
    ESP_LOGI(TAG, "SPI Pins - CS=%d, MOSI=%d, MISO=%d, CLK=%d", 
             PIN_NUM_CS, PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK);
    
    // Configure SPI bus
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
    
    // Configure SD card slot
    ESP_LOGI(TAG, "Configuring SD card slot...");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = SPI2_HOST;
    
    // Mount configuration
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    // Mount the SD card
    ESP_LOGI(TAG, "Mounting SD card to /sdcard...");
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  - SD card not inserted");
        ESP_LOGE(TAG, "  - SD card pins incorrect");
        ESP_LOGE(TAG, "  - SPI bus conflicts");
        return false;
    }
    
    // Print card info
    ESP_LOGI(TAG, "SD card mounted successfully!");
    if (sd_card) {
        ESP_LOGI(TAG, "Card Info:");
        ESP_LOGI(TAG, "  - Capacity: %llu bytes", ((uint64_t)sd_card->csd.capacity) * sd_card->csd.sector_size);
        ESP_LOGI(TAG, "  - Sectors: %lu", sd_card->csd.capacity);
        ESP_LOGI(TAG, "  - Sector Size: %d bytes", sd_card->csd.sector_size);
    }
    
    return true;
}

void sd_scan_folders(const char *path) {
    ESP_LOGI(TAG, "Scanning folders in: %s", path);
    
    // Clear previous folder list
    sd_free_folders();
    
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", path);
        return;
    }
    
    struct dirent *entry;
    int found = 0;
    
    while ((entry = readdir(dir)) != NULL && found < MAX_FOLDERS) {
        // Skip . and ..
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        
        // Check if it's a directory
        if (is_directory(path, entry)) {
            char *full_path = make_full_path(path, entry->d_name);
            if (!full_path) break;
            
            folder_list[found] = full_path;
            ESP_LOGI(TAG, "  [%d] %s", found, full_path);
            found++;
        }
    }
    
    closedir(dir);
    num_folders = found;
    ESP_LOGI(TAG, "Found %d folders", num_folders);
}

void sd_scan_wav_files(const char *folder_path) {
    ESP_LOGI(TAG, "Scanning WAV files in: %s", folder_path);
    
    // Clear previous WAV list
    sd_free_wavs();
    
    DIR *dir = opendir(folder_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open folder: %s", folder_path);
        return;
    }
    
    struct dirent *entry;
    int found = 0;
    
    while ((entry = readdir(dir)) != NULL && found < MAX_WAV_FILES) {
        // Skip . and ..
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        
        ESP_LOGI(TAG, "  Checking: %s", entry->d_name);
        
        // Check if it's a regular file
        if (!is_regular_file(folder_path, entry)) {
            ESP_LOGW(TAG, "    Not a regular file: %s", entry->d_name);
            continue;
        }
        
        // Check if it's a WAV file
        if (is_wav_file(entry->d_name)) {
            char *full_path = make_full_path(folder_path, entry->d_name);
            if (!full_path) break;
            
            wav_list[found] = full_path;
            ESP_LOGI(TAG, "  [%d] %s", found, full_path);
            found++;
        } else {
            ESP_LOGW(TAG, "    Not a WAV file: %s", entry->d_name);
        }
    }
    
    closedir(dir);
    num_wavs = found;
    ESP_LOGI(TAG, "Found %d WAV files", num_wavs);
}

int sd_get_folder_count(void) {
    return num_folders;
}

void sd_scan_subfolders(const char *folder_path) {
    // Free previous subfolder list
    sd_free_subfolders();
    
    DIR *dir = opendir(folder_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open folder: %s", folder_path);
        return;
    }
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && num_subfolders < MAX_FOLDERS) {
        // Skip hidden files and current/parent directory entries
        if (entry->d_name[0] == '.') {
            continue;
        }
        
        // Check if it's a directory
        if (is_directory(folder_path, entry)) {
            // Create full path
            char *full_path = make_full_path(folder_path, entry->d_name);
            if (full_path) {
                subfolder_list[num_subfolders] = full_path;
                num_subfolders++;
                ESP_LOGI(TAG, "Found subfolder [%d]: %s", num_subfolders - 1, entry->d_name);
            }
        }
    }
    
    closedir(dir);
    ESP_LOGI(TAG, "Total subfolders found: %d", num_subfolders);
}

int sd_get_subfolder_count(void) {
    return num_subfolders;
}

const char* sd_get_subfolder_path(int index) {
    if (index < 0 || index >= num_subfolders) {
        return NULL;
    }
    return subfolder_list[index];
}

void sd_free_subfolders(void) {
    for (int i = 0; i < num_subfolders; i++) {
        if (subfolder_list[i]) {
            free(subfolder_list[i]);
            subfolder_list[i] = NULL;
        }
    }
    num_subfolders = 0;
    ESP_LOGI(TAG, "Subfolder list freed");
}

int sd_get_wav_count(void) {
    return num_wavs;
}

const char* sd_get_folder_path(int index) {
    if (index < 0 || index >= num_folders) {
        return NULL;
    }
    return folder_list[index];
}

const char* sd_get_wav_path(int index) {
    if (index < 0 || index >= num_wavs) {
        return NULL;
    }
    return wav_list[index];
}

void sd_free_folders(void) {
    for (int i = 0; i < num_folders; i++) {
        if (folder_list[i]) {
            free(folder_list[i]);
            folder_list[i] = NULL;
        }
    }
    num_folders = 0;
}

void sd_free_wavs(void) {
    for (int i = 0; i < num_wavs; i++) {
        if (wav_list[i]) {
            free(wav_list[i]);
            wav_list[i] = NULL;
        }
    }
    num_wavs = 0;
}

sdmmc_card_t* sd_get_card_handle(void) {
    return sd_card;
}