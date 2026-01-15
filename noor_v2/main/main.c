// main.c
// NAV_PLAYER: Home/stories announcements + interruptible playback
// - Plays welcome.wav then home.wav on boot
// - When at HOME, selecting stories folder auto-plays stories.wav
// - Entering stories folder: announcements for S1..S5 (story1.wav..story5.wav) play on selection
// - Announcements interrupt normal playback; any interaction interrupts announcements
// Pins: I2S BCLK=18 WS=17 DIN=16
// SD SPI: CS=10 MOSI=11 SCK=12 MISO=13
// Buttons: PlayPause=14 Home=15 Vol+=4 Vol-=5 (active HIGH, pull-down)
// Encoder: CLK=1 DT=2 SW=19 (ISR-driven)

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctype.h>              // for isdigit()
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

static const char *TAG = "NAV_PLAYER_ANN";

// ---------- Pins ----------
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
#define ENC_SW_PIN   21
// ---------- Settings ----------
#define DEBOUNCE_MS 50
#define ENC_STEP_DEBOUNCE_MS 60
#define MAX_WAV_FILES 64
#define MAX_FOLDERS 32
#define ENC_QUEUE_LEN 16
#define ANNOUNCE_PATH_MAX 256

// ---------- Navigation ----------
typedef enum { NAV_HOME=0, NAV_FOLDER_VIEW, NAV_FILE_VIEW } nav_state_t;
static volatile nav_state_t nav_state = NAV_HOME;

// ---------- Controls ----------
static volatile bool g_pause = false;
static volatile bool g_playing = false;
static volatile bool g_stop_and_rewind = false;
static volatile int g_volume_percent = 100;

// ---------- Lists ----------
static char *folder_list[MAX_FOLDERS];
static int num_folders = 0;
static int selected_folder = 0;

static char *wav_list[MAX_WAV_FILES];
static volatile int num_tracks = 0;
static volatile int current_track = 0;    // selection index inside folder
static volatile int playing_track = -1;   // which file is actively playing
static volatile bool track_change_request = false;

// ---------- SD mount ----------
esp_vfs_fat_mount_config_t mount_cfg = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
};
sdmmc_card_t *sdcard = NULL;

// ---------- Announcement request ----------
static volatile bool announce_request = false;
static char announce_path[ANNOUNCE_PATH_MAX];

// ---------- Encoder queue ----------
typedef enum { ENC_EVT_CLK = 1, ENC_EVT_SW = 2 } enc_evt_type_t;
typedef struct { enc_evt_type_t type; uint8_t dt_level; } enc_evt_t;
static QueueHandle_t enc_queue = NULL;

// ---------- Utility helpers ----------
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

// ---------- File scanning (robust) ----------
static void scan_root_folders(const char *path) {
    free_folder_list();
    DIR *d = opendir(path);
    if (!d) { ESP_LOGE(TAG, "Failed to open %s", path); return; }
    struct dirent *entry;
    int found = 0;
    while ((entry = readdir(d)) != NULL && found < MAX_FOLDERS) {
        if (strcmp(entry->d_name, ".")==0 || strcmp(entry->d_name, "..")==0) continue;
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
            ESP_LOGI(TAG, "Found folder [%d]: %s", found-1, full);
        }
    }
    closedir(d);
    num_folders = found;
    ESP_LOGI(TAG, "Folders found: %d", num_folders);
}

static void scan_wavs_in_folder(const char *path) {
    free_wav_list();
    DIR *d = opendir(path);
    if (!d) { ESP_LOGE(TAG, "Failed to open folder %s", path); return; }
    struct dirent *entry;
    int found = 0;
    while ((entry = readdir(d)) != NULL && found < MAX_WAV_FILES) {
        if (strcmp(entry->d_name, ".")==0 || strcmp(entry->d_name, "..")==0) continue;
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
            ESP_LOGI(TAG, "Found WAV [%d]: %s", found-1, full);
        }
    }
    closedir(d);
    num_tracks = found;
    ESP_LOGI(TAG, "WAV files found: %d in %s", num_tracks, path);
}

// ---------- SD init ----------
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

// ---------- GPIO init ----------
static void init_inputs(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE
    };
    io_conf.pin_bit_mask = (1ULL<<BTN_PLAY_PAUSE_PIN) | (1ULL<<BTN_HOME_PIN) | (1ULL<<BTN_VOL_UP_PIN) | (1ULL<<BTN_VOL_DOWN_PIN);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL<<ENC_CLK_PIN) | (1ULL<<ENC_DT_PIN) | (1ULL<<ENC_SW_PIN);
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Inputs configured (buttons + encoder)");
}

// ---------- Buttons ----------
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

// ---------- WAV header parsing ----------
typedef struct {
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint16_t channels;
    uint32_t data_size;
    uint32_t data_offset;
} wav_info_t;

static bool parse_wav_header(FILE *f, wav_info_t *info) {
    uint8_t header[44];
    if (fread(header,1,44,f) != 44) return false;
    if (memcmp(header,"RIFF",4)!=0 || memcmp(header+8,"WAVE",4)!=0) return false;
    info->channels = *((uint16_t*)(header+22));
    info->sample_rate = *((uint32_t*)(header+24));
    info->bits_per_sample = *((uint16_t*)(header+34));
    info->data_size = *((uint32_t*)(header+40));
    info->data_offset = 44;
    return true;
}

// ---------- Stream file with interruption checks (used by audio_task for announcements and playback) ----------
static bool stream_file_interruptible(const char *fullpath, volatile bool *stop_flag_ptr) {
    if (!fullpath) return false;
    FILE *f = fopen(fullpath, "rb");
    if (!f) { ESP_LOGW(TAG, "stream_file: not found: %s", fullpath); return false; }
    wav_info_t winfo;
    if (!parse_wav_header(f, &winfo)) { ESP_LOGE(TAG, "Invalid WAV header: %s", fullpath); fclose(f); return false; }
    if (winfo.bits_per_sample != 16) { ESP_LOGE(TAG, "Only 16-bit PCM supported (ann/play): %s", fullpath); fclose(f); return false; }

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
    if (r != ESP_OK) { ESP_LOGE(TAG, "i2s_driver_install failed: %s", esp_err_to_name(r)); fclose(f); return false; }
    r = i2s_set_pin(I2S_NUM_0, &pin_cfg);
    if (r != ESP_OK) { ESP_LOGE(TAG, "i2s_set_pin failed: %s", esp_err_to_name(r)); i2s_driver_uninstall(I2S_NUM_0); fclose(f); return false; }
    r = i2s_set_clk(I2S_NUM_0, winfo.sample_rate, I2S_BITS_PER_SAMPLE_16BIT, winfo.channels);
    if (r != ESP_OK) ESP_LOGW(TAG, "i2s_set_clk: %s", esp_err_to_name(r));

    fseek(f, winfo.data_offset, SEEK_SET);
    const size_t chunk_bytes = 1024 * (winfo.bits_per_sample / 8) * winfo.channels;
    uint8_t *buf = malloc(chunk_bytes);
    if (!buf) { ESP_LOGE(TAG, "No mem"); fclose(f); i2s_driver_uninstall(I2S_NUM_0); return false; }

    while (1) {
        if (stop_flag_ptr && *stop_flag_ptr) {
            ESP_LOGI(TAG, "stream interrupted: %s", fullpath);
            break;
        }
        size_t bytes_read = fread(buf, 1, chunk_bytes, f);
        if (bytes_read == 0) break;
        // apply volume scaling
        if (g_volume_percent != 100) {
            int16_t *samps = (int16_t*)buf;
            size_t ns = bytes_read / 2;
            for (size_t i=0;i<ns;++i) {
                int32_t scaled = ((int32_t)samps[i] * g_volume_percent) / 100;
                samps[i] = clip16(scaled);
            }
        }
        size_t written = 0;
        esp_err_t res = i2s_write(I2S_NUM_0, buf, bytes_read, &written, pdMS_TO_TICKS(1000));
        if (res != ESP_OK) ESP_LOGW(TAG, "i2s_write: %s", esp_err_to_name(res));
        // yield so other tasks can operate and so we can catch interrupts
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(buf);
    fclose(f);
    i2s_driver_uninstall(I2S_NUM_0);
    return true;
}

// ---------- Announcement setter (thread-safe-ish) ----------
static void request_announcement(const char *path) {
    if (!path) return;
    // copy path into announce_path
    strncpy(announce_path, path, sizeof(announce_path)-1);
    announce_path[sizeof(announce_path)-1] = '\0';
    announce_request = true;
}

// ---------- Encoder ISRs (IRAM safe) ----------
static void IRAM_ATTR gpio_isr_clk_handler(void *arg) {
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

// ---------- Encoder processing task ----------
static void encoder_task(void *arg) {
    ESP_LOGI(TAG, "encoder_task started");
    enc_evt_t ev;
    uint32_t last_step_time = 0;
    uint32_t last_sw_time = 0;

    while (1) {
        if (xQueueReceive(enc_queue, &ev, portMAX_DELAY) == pdTRUE) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (ev.type == ENC_EVT_CLK) {
                if (now - last_step_time < ENC_STEP_DEBOUNCE_MS) continue;
                last_step_time = now;
                if (nav_state == NAV_HOME || nav_state == NAV_FOLDER_VIEW) {
                    if (num_folders > 0) {
                        if (ev.dt_level == 0) selected_folder = (selected_folder + 1) % num_folders;
                        else selected_folder = (selected_folder - 1 + num_folders) % num_folders;
                        ESP_LOGI(TAG, "Folder selected: %d -> %s", selected_folder, folder_list[selected_folder]);
                        // if folder is stories/01, announce it
                        const char *fp = folder_list[selected_folder];
                        const char *name = strrchr(fp, '/'); name = name ? name+1 : fp;
                        if (strcasecmp(name, "01")==0 || strcasecmp(name, "stories")==0) {
                            // try root /sdcard/stories.wav then folder/stories.wav
                            if (access("/sdcard/stories.wav", F_OK) == 0) request_announcement("/sdcard/stories.wav");
                            else {
                                char *in_folder = make_path(fp, "stories.wav");
                                if (in_folder) { if (access(in_folder, F_OK)==0) request_announcement(in_folder); free(in_folder); }
                            }
                        }
                    }
                } else if (nav_state == NAV_FILE_VIEW) {
                    if (num_tracks > 0) {
                        if (ev.dt_level == 0) current_track = (current_track + 1) % num_tracks;
                        else current_track = (current_track - 1 + num_tracks) % num_tracks;
                        ESP_LOGI(TAG, "File selected (only): %d -> %s", current_track, wav_list[current_track]);
                        // announcement: if filename matches S<number>.wav -> play story<number>.wav
                        const char *fullname = wav_list[current_track];
                        const char *b = strrchr(fullname, '/'); b = b ? b+1 : fullname;
                        if ((b[0]=='S' || b[0]=='s') && isdigit((unsigned char)b[1])) {
                            int n = b[1] - '0';
                            char candidate_root[ANNOUNCE_PATH_MAX];
                            snprintf(candidate_root, sizeof(candidate_root), "/sdcard/story%d.wav", n);
                            if (access(candidate_root, F_OK) == 0) request_announcement(candidate_root);
                            else {
                                // try in the same folder where the S#.wav resides
                                const char *slash = strrchr(fullname, '/');
                                if (slash) {
                                    size_t folder_len = slash - fullname;
                                    if (folder_len < ANNOUNCE_PATH_MAX) {
                                        char folder_dir[ANNOUNCE_PATH_MAX];
                                        memcpy(folder_dir, fullname, folder_len);
                                        folder_dir[folder_len] = '\0';
                                        char storyname[32];
                                        snprintf(storyname, sizeof(storyname), "story%d.wav", n);
                                        char *candidate2 = make_path(folder_dir, storyname);
                                        if (candidate2) { if (access(candidate2, F_OK)==0) request_announcement(candidate2); free(candidate2); }
                                    }
                                }
                            }
                        }
                    }
                }
            } else if (ev.type == ENC_EVT_SW) {
                if (now - last_sw_time < DEBOUNCE_MS) continue;
                last_sw_time = now;
                // behave like Play/Pause press
                if (nav_state == NAV_FILE_VIEW) {
                    if (!g_playing) {
                        if (num_tracks > 0) {
                            playing_track = current_track; g_playing = true; g_pause = false; track_change_request = true;
                            ESP_LOGI(TAG, "Encoder SW: start playing selected %d", playing_track);
                        }
                    } else {
                        if (playing_track == current_track) { g_pause = !g_pause; ESP_LOGI(TAG, "Encoder SW: toggle pause -> %s", g_pause ? "PAUSED":"PLAYING"); }
                        else { playing_track = current_track; g_pause = false; track_change_request = true; ESP_LOGI(TAG, "Encoder SW: switch to %d", playing_track); }
                    }
                } else if (nav_state == NAV_FOLDER_VIEW) {
                    if (num_folders > 0) {
                        const char *folder_path = folder_list[selected_folder];
                        // play stories.wav if folder is stories/01 before entering (same behavior)
                        const char *name = strrchr(folder_path, '/'); name = name ? name+1 : folder_path;
                        if (strcasecmp(name,"01")==0 || strcasecmp(name,"stories")==0) {
                            if (access("/sdcard/stories.wav", F_OK)==0) request_announcement("/sdcard/stories.wav");
                            else {
                                char *in_folder = make_path(folder_path, "stories.wav");
                                if (in_folder) { if (access(in_folder,F_OK)==0) request_announcement(in_folder); free(in_folder); }
                            }
                        }
                        scan_wavs_in_folder(folder_path);
                        nav_state = NAV_FILE_VIEW; current_track = 0; playing_track = -1; g_playing = false; g_pause = false;
                        ESP_LOGI(TAG, "Entered folder via encoder SW: %s (files=%d)", folder_path, num_tracks);
                        // announce current file (S1) immediately if S1 exists -> so trigger announcement for current selection
                        if (num_tracks > 0) {
                            const char *b = strrchr(wav_list[current_track], '/'); b = b ? b+1 : wav_list[current_track];
                            if ((b[0]=='S' || b[0]=='s') && isdigit((unsigned char)b[1])) {
                                int n = b[1]-'0';
                                char candidate_root[ANNOUNCE_PATH_MAX];
                                snprintf(candidate_root, sizeof(candidate_root), "/sdcard/story%d.wav", n);
                                if (access(candidate_root,F_OK)==0) request_announcement(candidate_root);
                                else {
                                    // try in-folder storyN.wav
                                    const char *slash = strrchr(wav_list[current_track], '/');
                                    if (slash) {
                                        size_t folder_len = slash - wav_list[current_track];
                                        if (folder_len < ANNOUNCE_PATH_MAX) {
                                            char folder_dir[ANNOUNCE_PATH_MAX];
                                            memcpy(folder_dir, wav_list[current_track], folder_len);
                                            folder_dir[folder_len] = '\0';
                                            char storyname[32];
                                            snprintf(storyname, sizeof(storyname), "story%d.wav", n);
                                            char *candidate_in = make_path(folder_dir, storyname);
                                            if (candidate_in) { if (access(candidate_in,F_OK)==0) request_announcement(candidate_in); free(candidate_in); }
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else if (nav_state == NAV_HOME) {
                    if (num_folders > 0) { nav_state = NAV_FOLDER_VIEW; ESP_LOGI(TAG, "HOME -> FOLDER_VIEW via encoder SW"); }
                }
            }
        }
    }
}

// ---------- Audio task (handles announcements and normal playback, interruptible) ----------
static void audio_task(void *arg) {
    ESP_LOGI(TAG, "audio_task started");
    while (1) {
        // Handle announcement request first (high priority)
        if (announce_request) {
            // copy announce path and clear flag
            char local_ann[ANNOUNCE_PATH_MAX];
            strncpy(local_ann, announce_path, sizeof(local_ann)-1);
            local_ann[sizeof(local_ann)-1] = '\0';
            announce_request = false;
            ESP_LOGI(TAG, "Announcement requested: %s", local_ann);

            // Interrupt normal playback if any
            track_change_request = false;
            g_playing = false; // stopping any playing track; user interaction interrupts playback
            // Play announcement with stop_flag pointer = &announce_request (so new announcements interrupt this)
            stream_file_interruptible(local_ann, (volatile bool *)&announce_request);
            // finished one announcement, continue loop to check for new announcements or playback
            continue;
        }

        // Normal playback if requested
        if (g_playing) {
            if (nav_state != NAV_FILE_VIEW) {
                // only play when inside folder
                g_playing = false; playing_track = -1;
                vTaskDelay(pdMS_TO_TICKS(200)); continue;
            }
            if (num_tracks == 0) { g_playing = false; vTaskDelay(pdMS_TO_TICKS(200)); continue; }
            int idx = playing_track;
            if (idx < 0 || idx >= num_tracks) { ESP_LOGW(TAG, "Invalid playing_track %d", idx); g_playing=false; playing_track=-1; vTaskDelay(pdMS_TO_TICKS(200)); continue; }
            char *file_to_play = wav_list[idx];
            if (!file_to_play) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
            ESP_LOGI(TAG, "Start playing track %d: %s", idx, file_to_play);

            // While streaming, monitor announce_request; if it becomes true, we must stop immediately
            stream_file_interruptible(file_to_play, (volatile bool *)&announce_request);

            // After streaming (either finished or interrupted), clear playing flags if interrupted by announce
            if (announce_request) { // if announcement now queued, playback was interrupted
                g_playing = false; playing_track = -1;
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------- Encoder ISR setup & app main ----------
void app_main(void) {
    ESP_LOGI(TAG, "=== NAV_PLAYER (announcements) starting ===");

    init_inputs();

    if (!init_sd()) {
        ESP_LOGE(TAG, "SD init failed - check wiring/card");
    } else {
        scan_root_folders("/sdcard");
        // default selection: prefer "01" -> "audios" -> first
        if (num_folders > 0) {
            int found_idx = -1;
            for (int i=0;i<num_folders;++i) {
                const char *p = strrchr(folder_list[i], '/'); const char *name = p ? p+1 : folder_list[i];
                if (strcasecmp(name,"01")==0) { found_idx = i; break; }
                if (found_idx < 0 && strcasecmp(name,"audios")==0) found_idx = i;
            }
            selected_folder = (found_idx >= 0) ? found_idx : 0;
            ESP_LOGI(TAG, "Default folder selected: index=%d -> %s", selected_folder, folder_list[selected_folder]);
        } else ESP_LOGW(TAG, "No folders found at /sdcard");

        // Play welcome + home on boot if present
        if (access("/sdcard/welcome.wav", F_OK) == 0) stream_file_interruptible("/sdcard/welcome.wav", NULL);
        if (access("/sdcard/home.wav", F_OK) == 0) stream_file_interruptible("/sdcard/home.wav", NULL);
        // After boot greetings, we are in HOME (nav_state default) — no need to request_announcement here because we just played home.wav.
    }

    // create encoder queue & ISRs
    enc_queue = xQueueCreate(ENC_QUEUE_LEN, sizeof(enc_evt_t));
    if (!enc_queue) ESP_LOGE(TAG, "Failed to create encoder queue");
    else {
        gpio_install_isr_service(0);
        gpio_set_intr_type(ENC_CLK_PIN, GPIO_INTR_POSEDGE);
        gpio_isr_handler_add(ENC_CLK_PIN, gpio_isr_clk_handler, NULL);
        gpio_set_intr_type(ENC_SW_PIN, GPIO_INTR_POSEDGE);
        gpio_isr_handler_add(ENC_SW_PIN, gpio_isr_sw_handler, NULL);
        ESP_LOGI(TAG, "Encoder ISRs installed");
    }

    // create tasks
    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 5, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(encoder_task, "encoder_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);

    // main loop: handle buttons & navigation, plus play announcements when entering HOME or entering folder
    while (1) {
        if (read_button_press(&btn_play)) {
            ESP_LOGI(TAG, "Play/Pause pressed (nav=%d)", nav_state);
            if (nav_state == NAV_HOME) {
                if (num_folders > 0) { nav_state = NAV_FOLDER_VIEW; ESP_LOGI(TAG, "HOME -> FOLDER_VIEW (selected=%d)", selected_folder); }
                else ESP_LOGI(TAG, "No folders to enter");
            } else if (nav_state == NAV_FOLDER_VIEW) {
                if (num_folders > 0) {
                    const char *folder_path = folder_list[selected_folder];
                    // If folder is "01" or "stories", play stories.wav (root or folder) before entering
                    const char *name = strrchr(folder_path, '/'); name = name ? name+1 : folder_path;
                    if (strcasecmp(name,"01")==0 || strcasecmp(name,"stories")==0) {
                        if (access("/sdcard/stories.wav", F_OK) == 0) request_announcement("/sdcard/stories.wav");
                        else {
                            char *in_folder = make_path(folder_path, "stories.wav");
                            if (in_folder) { if (access(in_folder,F_OK)==0) request_announcement(in_folder); free(in_folder); }
                        }
                    }
                    scan_wavs_in_folder(folder_path);
                    nav_state = NAV_FILE_VIEW;
                    current_track = 0; playing_track = -1; g_playing = false; g_pause = false;
                    ESP_LOGI(TAG, "Entered folder %s (files=%d)", folder_path, num_tracks);
                    // announce current selection inside folder (S1) if pattern matches
                    if (num_tracks > 0) {
                        const char *b = strrchr(wav_list[current_track], '/'); b = b ? b+1 : wav_list[current_track];
                        if ((b[0]=='S' || b[0]=='s') && isdigit((unsigned char)b[1])) {
                            int n = b[1]-'0';
                            char candidate_root[ANNOUNCE_PATH_MAX];
                            snprintf(candidate_root, sizeof(candidate_root), "/sdcard/story%d.wav", n);
                            if (access(candidate_root, F_OK)==0) request_announcement(candidate_root);
                            else {
                                // try in-folder storyN.wav
                                const char *slash = strrchr(wav_list[current_track], '/');
                                if (slash) {
                                    size_t folder_len = slash - wav_list[current_track];
                                    if (folder_len < ANNOUNCE_PATH_MAX) {
                                        char folder_dir[ANNOUNCE_PATH_MAX];
                                        memcpy(folder_dir, wav_list[current_track], folder_len);
                                        folder_dir[folder_len] = '\0';
                                        char storyname[32];
                                        snprintf(storyname, sizeof(storyname), "story%d.wav", n);
                                        char *cand = make_path(folder_dir, storyname);
                                        if (cand) { if (access(cand,F_OK)==0) request_announcement(cand); free(cand); }
                                    }
                                }
                            }
                        }
                    }
                }
            } else if (nav_state == NAV_FILE_VIEW) {
                if (!g_playing) {
                    if (num_tracks > 0) {
                        playing_track = current_track; g_playing = true; g_pause = false; track_change_request = true;
                        ESP_LOGI(TAG, "Start playing selected file %d", playing_track);
                    } else ESP_LOGI(TAG, "No tracks to play");
                } else {
                    if (playing_track == current_track) { g_pause = !g_pause; ESP_LOGI(TAG, "Toggle pause -> %s", g_pause ? "PAUSED":"PLAYING"); }
                    else { playing_track = current_track; g_pause = false; track_change_request = true; ESP_LOGI(TAG, "Switch playing to selected file %d", playing_track); }
                }
            }
        }

        if (read_button_press(&btn_home)) {
            ESP_LOGI(TAG, "Home pressed (nav=%d)", nav_state);
            if (nav_state == NAV_FILE_VIEW) {
                if (g_playing) { g_playing = false; g_pause = false; g_stop_and_rewind = true; }
                free_wav_list(); nav_state = NAV_FOLDER_VIEW; ESP_LOGI(TAG, "FILE_VIEW -> FOLDER_VIEW");
            } else if (nav_state == NAV_FOLDER_VIEW) {
                nav_state = NAV_HOME;
                ESP_LOGI(TAG, "FOLDER_VIEW -> HOME");
                // play home.wav when we get to HOME (announcement)
                if (access("/sdcard/home.wav", F_OK) == 0) request_announcement("/sdcard/home.wav");
            } else ESP_LOGI(TAG, "Already at HOME");
        }

        if (read_button_press(&btn_volp)) { g_volume_percent += 10; if (g_volume_percent>200) g_volume_percent=200; ESP_LOGI(TAG, "Vol+ -> %d%%", g_volume_percent); }
        if (read_button_press(&btn_volm)) { g_volume_percent -= 10; if (g_volume_percent<0) g_volume_percent=0; ESP_LOGI(TAG, "Vol- -> %d%%", g_volume_percent); }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
