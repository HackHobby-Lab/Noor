// main.c
// NAV_PLAYER: SD WAV player with MAX98357 (I2S) + Buttons + Rotary Encoder (ISR-driven)
// Behavior: HOME -> FOLDER_VIEW -> FILE_VIEW -> PLAY (Play/Pause required to start)
// I2S (MAX98357): BCLK=GPIO18, LRCLK/WS=GPIO17, DIN=GPIO16, GAIN -> GND
// SD (SPI): CS=GPIO10, MOSI=GPIO11, SCK=GPIO12, MISO=GPIO13
// Buttons: PlayPause=GPIO14, Home=GPIO15, Vol+ = GPIO4, Vol- = GPIO5
// Rotary encoder: CLK=GPIO1 (rising-edge), DT=GPIO2 (read level), SW(push)=GPIO19 (rising-edge)

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"

static const char *TAG = "NAV_PLAYER_ISR";

/* ---------- USER PIN DEFINES ---------- */
#define I2S_BCK_PIN   18
#define I2S_WS_PIN    17
#define I2S_DO_PIN    16

#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

#define BTN_PLAY_PAUSE_PIN 14
#define BTN_HOME_PIN       15
#define BTN_VOL_UP_PIN     4
#define BTN_VOL_DOWN_PIN   5

#define ENC_CLK_PIN  1
#define ENC_DT_PIN   2
#define ENC_SW_PIN   19

/* ---------- SETTINGS ---------- */
#define DEBOUNCE_MS 50
#define ENC_STEP_DEBOUNCE_MS 60   // step debounce
#define MAX_WAV_FILES 64
#define MAX_FOLDERS 32
#define ENC_QUEUE_LEN 16

/* ---------- navigation state ---------- */
typedef enum { NAV_HOME = 0, NAV_FOLDER_VIEW, NAV_FILE_VIEW } nav_state_t;
static volatile nav_state_t nav_state = NAV_HOME;

/* ---------- playback & control globals ---------- */
static volatile bool g_pause = false;         // pause flag when playing
static volatile bool g_playing = false;       // true when actively playing
static volatile bool g_stop_and_rewind = false;
static volatile int g_volume_percent = 100;   // 0-200%

/* ---------- lists ---------- */
static char *folder_list[MAX_FOLDERS];
static int num_folders = 0;
static int selected_folder = 0;

static char *wav_list[MAX_WAV_FILES];
static volatile int num_tracks = 0;
static volatile int current_track = 0;        // selection index in file view
static volatile int playing_track = -1;       // -1 none
static volatile bool track_change_request = false;

/* ---------- sd mount ---------- */
esp_vfs_fat_mount_config_t mount_cfg = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
};
sdmmc_card_t *sdcard = NULL;

/* ---------- helpers ---------- */
static inline int16_t clip16(int32_t s) {
    if (s > 32767) return 32767;
    if (s < -32768) return -32768;
    return (int16_t)s;
}

static char *make_path(const char *dir, const char *name) {
    size_t need = strlen(dir) + 1 + strlen(name) + 1;
    char *p = malloc(need);
    if (!p) return NULL;
    sprintf(p, "%s/%s", dir, name);
    return p;
}

static void free_folder_list(void) {
    for (int i = 0; i < num_folders; ++i) { free(folder_list[i]); folder_list[i] = NULL; }
    num_folders = 0; selected_folder = 0;
}
static void free_wav_list(void) {
    for (int i = 0; i < num_tracks; ++i) { free(wav_list[i]); wav_list[i] = NULL; }
    num_tracks = 0; current_track = 0;
}

/* ---------- robust scan for folders in /sdcard ---------- */
static void scan_root_folders(const char *path) {
    free_folder_list();
    DIR *d = opendir(path);
    if (!d) { ESP_LOGE(TAG, "Failed to open %s", path); return; }

    struct dirent *entry;
    int found = 0;
    while ((entry = readdir(d)) != NULL && found < MAX_FOLDERS) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;

        bool is_dir = false;
        if (entry->d_type == DT_DIR) is_dir = true;
        else if (entry->d_type == DT_UNKNOWN) {
            char *full = make_path(path, entry->d_name);
            if (!full) break;
            struct stat sb;
            if (stat(full, &sb) == 0 && S_ISDIR(sb.st_mode)) is_dir = true;
            free(full);
        }

        if (is_dir) {
            char *full = make_path(path, entry->d_name);
            if (!full) break;
            folder_list[found++] = full;
            ESP_LOGI(TAG, "Found folder [%d]: %s", found - 1, full);
        }
    }
    closedir(d);
    num_folders = found;
    ESP_LOGI(TAG, "Folders found: %d", num_folders);
}

/* ---------- robust scan for WAVs in folder ---------- */
static void scan_wavs_in_folder(const char *path) {
    free_wav_list();
    DIR *d = opendir(path);
    if (!d) { ESP_LOGE(TAG, "Failed to open folder %s", path); return; }

    struct dirent *entry;
    int found = 0;
    while ((entry = readdir(d)) != NULL && found < MAX_WAV_FILES) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;

        bool is_file = false;
        if (entry->d_type == DT_REG) is_file = true;
        else if (entry->d_type == DT_UNKNOWN) {
            char *full = make_path(path, entry->d_name);
            if (!full) break;
            struct stat sb;
            if (stat(full, &sb) == 0 && S_ISREG(sb.st_mode)) is_file = true;
            free(full);
        }
        if (!is_file) continue;

        const char *name = entry->d_name;
        size_t n = strlen(name);
        if (n <= 4) continue;
        const char *ext = name + (n - 4);
        if (!strcasecmp(ext, ".wav")) {
            char *full = make_path(path, name);
            if (!full) break;
            wav_list[found++] = full;
            ESP_LOGI(TAG, "Found WAV [%d]: %s", found - 1, full);
        }
    }
    closedir(d);
    num_tracks = found;
    ESP_LOGI(TAG, "WAV files found: %d in %s", num_tracks, path);
}

/* ---------- SD init ---------- */
static bool init_sd(void) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };
    esp_err_t r = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (r != ESP_OK) { ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(r)); return false; }
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = PIN_NUM_CS;
    slot_cfg.host_id = SPI2_HOST;
    r = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_cfg, &mount_cfg, &sdcard);
    if (r != ESP_OK) { ESP_LOGE(TAG, "Failed to mount SD: %s", esp_err_to_name(r)); return false; }
    sdmmc_card_print_info(stdout, sdcard);
    ESP_LOGI(TAG, "SD mounted at /sdcard");
    return true;
}

/* ---------- inputs init ---------- */
static void init_inputs(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE
    };
    // buttons
    io_conf.pin_bit_mask = (1ULL<<BTN_PLAY_PAUSE_PIN) | (1ULL<<BTN_HOME_PIN) | (1ULL<<BTN_VOL_UP_PIN) | (1ULL<<BTN_VOL_DOWN_PIN);
    gpio_config(&io_conf);
    // encoder pins - inputs (we will attach interrupts for CLK and SW)
    io_conf.pin_bit_mask = (1ULL<<ENC_CLK_PIN) | (1ULL<<ENC_DT_PIN) | (1ULL<<ENC_SW_PIN);
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Inputs configured (buttons + encoder)");
}

/* ---------- button structs & read ---------- */
typedef struct { gpio_num_t gpio; const char *name; uint32_t last_time; bool last_state; } btn_t;
static btn_t btn_play = { BTN_PLAY_PAUSE_PIN, "Play/Pause", 0, false };
static btn_t btn_home = { BTN_HOME_PIN, "Home", 0, false };
static btn_t btn_volp = { BTN_VOL_UP_PIN, "Vol+", 0, false };
static btn_t btn_volm = { BTN_VOL_DOWN_PIN, "Vol-", 0, false };

static bool read_button_press(btn_t *b) {
    bool lvl = gpio_get_level(b->gpio);
    uint32_t t = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (lvl && !b->last_state) {
        if (t - b->last_time > DEBOUNCE_MS) {
            b->last_time = t;
            b->last_state = lvl;
            return true;
        }
    }
    b->last_state = lvl;
    return false;
}

/* ---------- encoder ISR-driven implementation ---------- */
typedef enum { ENC_EVT_CLK = 1, ENC_EVT_SW = 2 } enc_evt_type_t;
typedef struct {
    enc_evt_type_t type;
    uint8_t dt_level; // valid for CLK events
} enc_evt_t;

static QueueHandle_t enc_queue = NULL;

// ISR handlers (must be IRAM safe)
static void IRAM_ATTR gpio_isr_clk_handler(void *arg) {
    // read DT level quickly and post event
    uint32_t dt_level = gpio_get_level(ENC_DT_PIN);
    enc_evt_t ev = { .type = ENC_EVT_CLK, .dt_level = (uint8_t)dt_level };
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(enc_queue, &ev, &hp);
    if (hp == pdTRUE) portYIELD_FROM_ISR();
}

static void IRAM_ATTR gpio_isr_sw_handler(void *arg) {
    enc_evt_t ev = { .type = ENC_EVT_SW, .dt_level = 0 };
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(enc_queue, &ev, &hp);
    if (hp == pdTRUE) portYIELD_FROM_ISR();
}

/* Encoder processing task: consumes events from queue, debounces, updates selection */
static void encoder_task(void *arg) {
    ESP_LOGI(TAG, "encoder_task (ISR-driven) started");
    enc_evt_t ev;
    uint32_t last_step_time = 0;
    uint32_t last_sw_time = 0;

    while (1) {
        if (xQueueReceive(enc_queue, &ev, portMAX_DELAY) == pdTRUE) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (ev.type == ENC_EVT_CLK) {
                // step debounce
                if (now - last_step_time < ENC_STEP_DEBOUNCE_MS) continue;
                last_step_time = now;
                // update selection only (no auto-play)
                if (nav_state == NAV_HOME || nav_state == NAV_FOLDER_VIEW) {
                    if (num_folders > 0) {
                        if (ev.dt_level == 0) selected_folder = (selected_folder + 1) % num_folders;
                        else selected_folder = (selected_folder - 1 + num_folders) % num_folders;
                        ESP_LOGI(TAG, "Folder selected: %d -> %s", selected_folder, folder_list[selected_folder]);
                    }
                } else if (nav_state == NAV_FILE_VIEW) {
                    if (num_tracks > 0) {
                        if (ev.dt_level == 0) current_track = (current_track + 1) % num_tracks;
                        else current_track = (current_track - 1 + num_tracks) % num_tracks;
                        ESP_LOGI(TAG, "File selected (only): %d -> %s", current_track, wav_list[current_track]);
                    }
                }
            } else if (ev.type == ENC_EVT_SW) {
                // SW debounce (simple)
                if (now - last_sw_time < DEBOUNCE_MS) continue;
                last_sw_time = now;
                // emulate Play/Pause press
                if (nav_state == NAV_FILE_VIEW) {
                    if (!g_playing) {
                        if (num_tracks > 0) {
                            playing_track = current_track;
                            g_playing = true;
                            g_pause = false;
                            track_change_request = true;
                            ESP_LOGI(TAG, "Encoder SW: start playing selected %d", playing_track);
                        }
                    } else {
                        if (playing_track == current_track) {
                            g_pause = !g_pause;
                            ESP_LOGI(TAG, "Encoder SW: toggle pause -> %s", g_pause ? "PAUSED" : "PLAYING");
                        } else {
                            playing_track = current_track;
                            g_pause = false;
                            track_change_request = true;
                            ESP_LOGI(TAG, "Encoder SW: switch playing to %d", playing_track);
                        }
                    }
                } else if (nav_state == NAV_FOLDER_VIEW) {
                    if (num_folders > 0) {
                        const char *fp = folder_list[selected_folder];
                        scan_wavs_in_folder(fp);
                        nav_state = NAV_FILE_VIEW;
                        current_track = 0;
                        playing_track = -1;
                        g_playing = false; g_pause = false;
                        ESP_LOGI(TAG, "Entered folder via encoder SW: %s", fp);
                    }
                } else if (nav_state == NAV_HOME) {
                    if (num_folders > 0) { nav_state = NAV_FOLDER_VIEW; ESP_LOGI(TAG, "HOME -> FOLDER_VIEW via encoder SW"); }
                }
            }
        }
    }
}

/* ---------- WAV header parse (simple 44-byte) ---------- */
typedef struct {
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint16_t channels;
    uint32_t data_size;
    uint32_t data_offset;
} wav_info_t;

static bool parse_wav_header(FILE *f, wav_info_t *info) {
    uint8_t header[44];
    if (fread(header, 1, 44, f) != 44) { ESP_LOGE(TAG, "Failed to read WAV header"); return false; }
    if (memcmp(header, "RIFF", 4) != 0 || memcmp(header + 8, "WAVE", 4) != 0) { ESP_LOGE(TAG, "Not a RIFF/WAVE file"); return false; }
    info->channels = *((uint16_t *)(header + 22));
    info->sample_rate = *((uint32_t *)(header + 24));
    info->bits_per_sample = *((uint16_t *)(header + 34));
    info->data_size = *((uint32_t *)(header + 40));
    info->data_offset = 44;
    ESP_LOGI(TAG, "WAV: ch=%d sr=%d bits=%d data=%u", info->channels, info->sample_rate, info->bits_per_sample, info->data_size);
    return true;
}

/* ---------- audio task ---------- */
static void audio_task(void *arg) {
    while (1) {
        if (!g_playing) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        if (nav_state != NAV_FILE_VIEW) { g_playing = false; playing_track = -1; vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        if (num_tracks == 0) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        int idx = playing_track;
        if (idx < 0 || idx >= num_tracks) { ESP_LOGW(TAG, "Invalid playing_track %d", idx); g_playing = false; playing_track = -1; vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        char *filename = wav_list[idx];
        if (!filename) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        FILE *f = fopen(filename, "rb");
        if (!f) { ESP_LOGE(TAG, "Failed to open %s", filename); vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        ESP_LOGI(TAG, "Playing %d: %s", idx, filename);

        wav_info_t winfo;
        if (!parse_wav_header(f, &winfo)) { fclose(f); vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        if (winfo.bits_per_sample != 16) { ESP_LOGE(TAG, "Only 16-bit PCM supported"); fclose(f); vTaskDelay(pdMS_TO_TICKS(500)); continue; }

        i2s_config_t i2s_cfg = {
            .mode = I2S_MODE_MASTER | I2S_MODE_TX,
            .sample_rate = (int)winfo.sample_rate,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = (winfo.channels == 1) ? I2S_CHANNEL_FMT_ONLY_LEFT : I2S_CHANNEL_FMT_RIGHT_LEFT,
            .communication_format = I2S_COMM_FORMAT_I2S,
            .intr_alloc_flags = 0,
            .dma_buf_count = 4,
            .dma_buf_len = 1024,
            .use_apll = false,
            .tx_desc_auto_clear = true
        };
        i2s_pin_config_t pin_cfg = { .bck_io_num = I2S_BCK_PIN, .ws_io_num = I2S_WS_PIN, .data_out_num = I2S_DO_PIN, .data_in_num = I2S_PIN_NO_CHANGE };

        esp_err_t r = i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
        if (r != ESP_OK) { ESP_LOGE(TAG, "i2s_driver_install failed: %s", esp_err_to_name(r)); fclose(f); vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        r = i2s_set_pin(I2S_NUM_0, &pin_cfg);
        if (r != ESP_OK) { ESP_LOGE(TAG, "i2s_set_pin failed: %s", esp_err_to_name(r)); i2s_driver_uninstall(I2S_NUM_0); fclose(f); vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        r = i2s_set_clk(I2S_NUM_0, winfo.sample_rate, I2S_BITS_PER_SAMPLE_16BIT, winfo.channels);
        if (r != ESP_OK) ESP_LOGW(TAG, "i2s_set_clk: %s", esp_err_to_name(r));

        fseek(f, winfo.data_offset, SEEK_SET);
        const size_t chunk_bytes = 1024 * (winfo.bits_per_sample / 8) * winfo.channels;
        uint8_t *read_buf = malloc(chunk_bytes);
        if (!read_buf) { ESP_LOGE(TAG, "No mem"); fclose(f); i2s_driver_uninstall(I2S_NUM_0); vTaskDelay(pdMS_TO_TICKS(500)); continue; }

        while (g_playing) {
            if (track_change_request) { track_change_request = false; ESP_LOGI(TAG, "Track change requested -> %d", playing_track); break; }
            if (g_stop_and_rewind) { g_stop_and_rewind = false; fseek(f, winfo.data_offset, SEEK_SET); ESP_LOGI(TAG, "Stop & rewind"); }
            if (g_pause) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

            size_t bytes_read = fread(read_buf, 1, chunk_bytes, f);
            if (bytes_read == 0) { fseek(f, winfo.data_offset, SEEK_SET); continue; }

            if (g_volume_percent != 100) {
                int16_t *samps = (int16_t *)read_buf;
                size_t ns = bytes_read / 2;
                for (size_t i = 0; i < ns; ++i) {
                    int32_t scaled = ((int32_t)samps[i] * g_volume_percent) / 100;
                    samps[i] = clip16(scaled);
                }
            }

            size_t bw = 0;
            esp_err_t res = i2s_write(I2S_NUM_0, read_buf, bytes_read, &bw, pdMS_TO_TICKS(1000));
            if (res != ESP_OK) ESP_LOGW(TAG, "i2s_write: %s", esp_err_to_name(res));
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        free(read_buf);
        fclose(f);
        i2s_driver_uninstall(I2S_NUM_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------- app_main ---------- */
void app_main(void) {
    ESP_LOGI(TAG, "=== NAV_PLAYER starting (HOME) ===");

    init_inputs();

    if (!init_sd()) {
        ESP_LOGE(TAG, "SD init failed - check wiring and card");
    } else {
        scan_root_folders("/sdcard");
        // pick default folder: prefer "01" -> "audios" -> first folder
        if (num_folders > 0) {
            int found_idx = -1;
            for (int i = 0; i < num_folders; ++i) {
                const char *p = strrchr(folder_list[i], '/');
                const char *name = p ? p + 1 : folder_list[i];
                if (strcasecmp(name, "01") == 0) { found_idx = i; break; }
                if (found_idx < 0 && strcasecmp(name, "audios") == 0) found_idx = i;
            }
            if (found_idx >= 0) selected_folder = found_idx;
            else selected_folder = 0;
            ESP_LOGI(TAG, "Default folder selected: index=%d -> %s", selected_folder, folder_list[selected_folder]);
        } else {
            ESP_LOGW(TAG, "No folders found at /sdcard");
        }
    }

    // create encoder event queue
    enc_queue = xQueueCreate(ENC_QUEUE_LEN, sizeof(enc_evt_t));
    if (!enc_queue) {
        ESP_LOGE(TAG, "Failed to create encoder queue");
    } else {
        // install ISR service and attach handlers
        gpio_install_isr_service(0);
        gpio_set_intr_type(ENC_CLK_PIN, GPIO_INTR_POSEDGE);
        gpio_isr_handler_add(ENC_CLK_PIN, gpio_isr_clk_handler, NULL);
        gpio_set_intr_type(ENC_SW_PIN, GPIO_INTR_POSEDGE);
        gpio_isr_handler_add(ENC_SW_PIN, gpio_isr_sw_handler, NULL);
        ESP_LOGI(TAG, "Encoder ISRs installed (CLK rising, SW rising)");
    }

    // create tasks: audio and encoder
    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(encoder_task, "encoder_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);

    // main loop: handle buttons & navigation
    while (1) {
        if (read_button_press(&btn_play)) {
            ESP_LOGI(TAG, "Play/Pause pressed (nav=%d)", nav_state);
            if (nav_state == NAV_HOME) {
                if (num_folders > 0) {
                    nav_state = NAV_FOLDER_VIEW;
                    ESP_LOGI(TAG, "HOME -> FOLDER_VIEW (selected=%d)", selected_folder);
                } else ESP_LOGI(TAG, "No folders to enter");
            } else if (nav_state == NAV_FOLDER_VIEW) {
                if (num_folders > 0) {
                    const char *fp = folder_list[selected_folder];
                    scan_wavs_in_folder(fp);
                    nav_state = NAV_FILE_VIEW;
                    current_track = 0;
                    playing_track = -1;
                    g_playing = false;
                    g_pause = false;
                    ESP_LOGI(TAG, "Entered folder %s (files=%d)", fp, num_tracks);
                }
            } else if (nav_state == NAV_FILE_VIEW) {
                if (!g_playing) {
                    if (num_tracks > 0) {
                        playing_track = current_track;
                        g_playing = true;
                        g_pause = false;
                        track_change_request = true;
                        ESP_LOGI(TAG, "Start playing selected file %d", playing_track);
                    } else ESP_LOGI(TAG, "No tracks to play");
                } else {
                    if (playing_track == current_track) {
                        g_pause = !g_pause;
                        ESP_LOGI(TAG, "Toggle pause -> %s", g_pause ? "PAUSED" : "PLAYING");
                    } else {
                        playing_track = current_track;
                        g_pause = false;
                        track_change_request = true;
                        ESP_LOGI(TAG, "Switch playing to selected file %d", playing_track);
                    }
                }
            }
        }

        if (read_button_press(&btn_home)) {
            ESP_LOGI(TAG, "Home pressed (nav=%d)", nav_state);
            if (nav_state == NAV_FILE_VIEW) {
                if (g_playing) { g_playing = false; g_pause = false; g_stop_and_rewind = true; }
                free_wav_list();
                nav_state = NAV_FOLDER_VIEW;
                ESP_LOGI(TAG, "FILE_VIEW -> FOLDER_VIEW");
            } else if (nav_state == NAV_FOLDER_VIEW) {
                nav_state = NAV_HOME;
                ESP_LOGI(TAG, "FOLDER_VIEW -> HOME");
            } else ESP_LOGI(TAG, "Already at HOME");
        }

        if (read_button_press(&btn_volp)) { g_volume_percent += 10; if (g_volume_percent > 200) g_volume_percent = 200; ESP_LOGI(TAG, "Volume + -> %d%%", g_volume_percent); }
        if (read_button_press(&btn_volm)) { g_volume_percent -= 10; if (g_volume_percent < 0) g_volume_percent = 0; ESP_LOGI(TAG, "Volume - -> %d%%", g_volume_percent); }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
