/**
 * sd_card.c
 * SD Card operations implementation
 */

#include "sd_card.h"
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

static const char *TAG = "SD_CARD";

static char *folder_list[MAX_FOLDERS];
static int   num_folders = 0;

static char *subfolder_list[MAX_FOLDERS];
static int   num_subfolders = 0;

static char *wav_list[MAX_WAV_FILES];
static int   num_wavs = 0;

static sdmmc_card_t *sd_card = NULL;

/* ---------------------------------------------------------- */
static char *make_full_path(const char *dir, const char *name) {
    size_t len = strlen(dir) + 1 + strlen(name) + 1;
    char  *p   = malloc(len);
    if (!p) return NULL;
    snprintf(p, len, "%s/%s", dir, name);
    return p;
}

static bool is_directory(const char *parent, struct dirent *entry) {
    if (entry->d_type == DT_DIR) return true;
    if (entry->d_type == DT_UNKNOWN) {
        char *fp = make_full_path(parent, entry->d_name);
        if (!fp) return false;
        struct stat sb;
        bool r = (stat(fp, &sb) == 0 && S_ISDIR(sb.st_mode));
        free(fp);
        return r;
    }
    return false;
}

static bool is_regular_file(const char *parent, struct dirent *entry) {
    if (entry->d_type == DT_REG) return true;
    if (entry->d_type == DT_UNKNOWN) {
        char *fp = make_full_path(parent, entry->d_name);
        if (!fp) return false;
        struct stat sb;
        bool r = (stat(fp, &sb) == 0 && S_ISREG(sb.st_mode));
        free(fp);
        return r;
    }
    return false;
}

static bool is_wav_file(const char *filename) {
    size_t len = strlen(filename);
    if (len <= 4) return false;
    return strcasecmp(filename + len - 4, ".wav") == 0;
}

/* ---------------------------------------------------------- */
bool sd_card_init(void) {
    ESP_LOGI(TAG, "Initializing SD card hardware...");
    ESP_LOGI(TAG, "Pins: CS=%d MOSI=%d MISO=%d CLK=%d",
             PIN_NUM_CS, PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK);

    spi_bus_config_t bus = {
        .mosi_io_num     = PIN_NUM_MOSI,
        .miso_io_num     = PIN_NUM_MISO,
        .sclk_io_num     = PIN_NUM_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* Initialize SDSPI host driver */
    ret = sdspi_host_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means already initialized - that is fine */
        ESP_LOGE(TAG, "SDSPI host init failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI2_HOST);
        return false;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 1000;  /* 1MHz - very conservative, maximally compatible */

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs  = PIN_NUM_CS;
    slot.host_id  = SPI2_HOST;

    /* Add the SD card device to the SPI bus */
    sdspi_dev_handle_t sdspi_handle;
    ret = sdspi_host_init_device(&slot, &sdspi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SDSPI device init failed: %s", esp_err_to_name(ret));
        sdspi_host_deinit();
        spi_bus_free(SPI2_HOST);
        return false;
    }
    host.slot = sdspi_handle;  /* slot field holds the device handle */

    /* Allocate and initialize the card struct (no FAT mount yet -
     * TinyUSB MSC storage will own all FAT mount/unmount operations) */
    sd_card = malloc(sizeof(sdmmc_card_t));
    if (!sd_card) {
        ESP_LOGE(TAG, "OOM: cannot allocate sdmmc_card_t");
        sdspi_host_remove_device(sdspi_handle);
        sdspi_host_deinit();
        spi_bus_free(SPI2_HOST);
        return false;
    }

    /* Try card init up to 3 times */
    for (int attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "Card init attempt %d/3...", attempt);
        ret = sdmmc_card_init(&host, sd_card);
        if (ret == ESP_OK) break;
        ESP_LOGW(TAG, "Attempt %d failed: %s", attempt, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card init failed: %s", esp_err_to_name(ret));
        free(sd_card);
        sd_card = NULL;
        sdspi_host_remove_device(sdspi_handle);
        sdspi_host_deinit();
        spi_bus_free(SPI2_HOST);
        return false;
    }

    ESP_LOGI(TAG, "SD card hardware initialized (FAT will be mounted by USB manager)");
    return true;
}

/* ---------------------------------------------------------- */
void sd_scan_folders(const char *path) {
    sd_free_folders();
    DIR *dir = opendir(path);
    if (!dir) { ESP_LOGE(TAG, "Cannot open: %s", path); return; }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && num_folders < MAX_FOLDERS) {
        if (entry->d_name[0] == '.') continue;
        if (is_directory(path, entry)) {
            char *fp = make_full_path(path, entry->d_name);
            if (fp) { folder_list[num_folders++] = fp; ESP_LOGI(TAG, "Folder: %s", fp); }
        }
    }
    closedir(dir);
    ESP_LOGI(TAG, "Found %d folders", num_folders);
}

int         sd_get_folder_count(void)      { return num_folders; }
const char *sd_get_folder_path(int index)  { return (index >= 0 && index < num_folders) ? folder_list[index] : NULL; }

void sd_free_folders(void) {
    for (int i = 0; i < num_folders; i++) { free(folder_list[i]); folder_list[i] = NULL; }
    num_folders = 0;
}

/* ---------------------------------------------------------- */
void sd_scan_subfolders(const char *folder_path) {
    sd_free_subfolders();
    DIR *dir = opendir(folder_path);
    if (!dir) { ESP_LOGE(TAG, "Cannot open: %s", folder_path); return; }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && num_subfolders < MAX_FOLDERS) {
        if (entry->d_name[0] == '.') continue;
        if (is_directory(folder_path, entry)) {
            char *fp = make_full_path(folder_path, entry->d_name);
            if (fp) { subfolder_list[num_subfolders++] = fp; }
        }
    }
    closedir(dir);
    ESP_LOGI(TAG, "Found %d subfolders", num_subfolders);
}

int         sd_get_subfolder_count(void)      { return num_subfolders; }
const char *sd_get_subfolder_path(int index)  { return (index >= 0 && index < num_subfolders) ? subfolder_list[index] : NULL; }

void sd_free_subfolders(void) {
    for (int i = 0; i < num_subfolders; i++) { free(subfolder_list[i]); subfolder_list[i] = NULL; }
    num_subfolders = 0;
}

/* ---------------------------------------------------------- */
void sd_scan_wav_files(const char *folder_path) {
    sd_free_wavs();
    DIR *dir = opendir(folder_path);
    if (!dir) { ESP_LOGE(TAG, "Cannot open: %s", folder_path); return; }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && num_wavs < MAX_WAV_FILES) {
        if (entry->d_name[0] == '.') continue;
        if (is_regular_file(folder_path, entry) && is_wav_file(entry->d_name)) {
            char *fp = make_full_path(folder_path, entry->d_name);
            if (fp) { wav_list[num_wavs++] = fp; ESP_LOGI(TAG, "WAV: %s", fp); }
        }
    }
    closedir(dir);
    ESP_LOGI(TAG, "Found %d WAV files", num_wavs);
}

int         sd_get_wav_count(void)      { return num_wavs; }
const char *sd_get_wav_path(int index)  { return (index >= 0 && index < num_wavs) ? wav_list[index] : NULL; }

void sd_free_wavs(void) {
    for (int i = 0; i < num_wavs; i++) { free(wav_list[i]); wav_list[i] = NULL; }
    num_wavs = 0;
}

sdmmc_card_t *sd_get_card_handle(void) { return sd_card; }
