/**
 * audio.c — volume changes without announcing (main.c handles announcements
 * via NOTIFY_VOLUME_BIT so audio_task owns the I2S driver exclusively).
 *
 * PATCHED to eliminate per-file heap churn.
 *
 * Original design installed the I2S driver and malloc'd 2 x 1024-byte
 * playback buffers on EVERY call to audio_play_file(), then uninstalled
 * the driver and freed the buffers on exit.  During navigation the device
 * plays many short announcements back-to-back (folder name, story
 * announcement, volume cues, etc.), and the constant install / malloc /
 * free / uninstall cycle:
 *   - Fragments the internal heap (esp_alloc retries can fail or succeed
 *     after long delays under fragmentation pressure).
 *   - Touches the I2S DMA descriptor pool, which on ESP-IDF 4.x has had
 *     known leak / late-free issues across rapid install/uninstall cycles.
 * Either failure mode manifests as a watchdog reset or an aborted task,
 * which the user experiences as "device restarts and plays welcome again"
 * or "loses navigation position" mid-browse.
 *
 * Changes vs original:
 *   1. I2S driver is installed ONCE in audio_init() and never uninstalled.
 *      Per-file work is reduced to i2s_set_clk() to switch sample rate.
 *   2. Playback buffers are static (file-scope), not malloc'd.
 *      Sized for the worst case the firmware actually emits:
 *        - 16-bit PCM
 *        - up to 2 channels (stereo input downmixed to mono out)
 *        - chunk_frames = 512
 *      => input buffer 2 * 512 * 2 = 2048 bytes
 *      => output buffer 512 * 2 = 1024 bytes
 *   3. audio_init() now does the heavy lifting; audio_play_file() is fast.
 *
 * No API changes.  audio_play_file() signature, return semantics, stop
 * flag, and pause flag behaviour are all preserved.  Sample-rate switching
 * between files still works because i2s_set_clk() is called at the top of
 * audio_play_file().
 */

#include "audio.h"
#include "announcements.h"
#include "sd_card.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_log.h"

static const char *TAG = "AUDIO";

static volatile int current_volume   = VOLUME_DEFAULT;
static bool         s_i2s_installed  = false;

/* -------------------------------------------------------------------------
 * Static playback buffers — sized for the worst case the firmware emits.
 *   chunk_frames = 512
 *   input  : 16-bit PCM * 2 channels * 512 frames = 2048 bytes
 *   output : 16-bit mono           * 512 frames = 1024 bytes
 * Allocating these statically eliminates per-file malloc/free pressure.
 * ---------------------------------------------------------------------- */
#define AUDIO_CHUNK_FRAMES   512
#define AUDIO_INPUT_BYTES    (AUDIO_CHUNK_FRAMES * 2 * sizeof(int16_t))
#define AUDIO_OUTPUT_BYTES   (AUDIO_CHUNK_FRAMES *     sizeof(int16_t))

static uint8_t s_input_buffer [AUDIO_INPUT_BYTES];
static uint8_t s_output_buffer[AUDIO_OUTPUT_BYTES];

/* -------------------------------------------------------------------------
 * SD mutex helpers — bounded timeout
 * ---------------------------------------------------------------------- */
#define SD_MUTEX_TIMEOUT_MS  50

static inline bool sd_mutex_take(void) {
    SemaphoreHandle_t m = sd_get_access_mutex();
    if (!m) return true;
    return (xSemaphoreTake(m, pdMS_TO_TICKS(SD_MUTEX_TIMEOUT_MS)) == pdTRUE);
}

static inline void sd_mutex_give(void) {
    SemaphoreHandle_t m = sd_get_access_mutex();
    if (m) xSemaphoreGive(m);
}

static inline int16_t clip_sample(int32_t s) {
    if (s >  32767) return  32767;
    if (s < -32768) return -32768;
    return (int16_t)s;
}

/* -------------------------------------------------------------------------
 * Install the I2S driver once.  Called from audio_init() only.
 * Sample rate is set per-file later via i2s_set_clk().
 * ---------------------------------------------------------------------- */
static bool i2s_install_once(void) {
    if (s_i2s_installed) return true;

    i2s_config_t i2s_config = {
        .mode                 = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate          = 44100,                          /* default; overridden per file */
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags     = 0,
        .dma_buf_count        = 8,
        .dma_buf_len          = 1024,
        .use_apll             = false,
        .tx_desc_auto_clear   = true
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num   = I2S_BCK_PIN,
        .ws_io_num    = I2S_WS_PIN,
        .data_out_num = I2S_DO_PIN,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };

    esp_err_t ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S install failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S set pin failed: %s", esp_err_to_name(ret));
        i2s_driver_uninstall(I2S_NUM_0);
        return false;
    }

    s_i2s_installed = true;
    ESP_LOGI(TAG, "I2S driver installed once (BCLK=%d WS=%d DOUT=%d)",
             I2S_BCK_PIN, I2S_WS_PIN, I2S_DO_PIN);
    return true;
}

/* -------------------------------------------------------------------------
 * Public API — init
 * ---------------------------------------------------------------------- */
void audio_init(void) {
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Audio System Initialization");
    ESP_LOGI(TAG, "Default Volume: %d%% (%d/100)", VOLUME_DEFAULT, VOLUME_DEFAULT);
    ESP_LOGI(TAG, "I2S Pins: BCLK=%d, WS=%d, DIN=%d", I2S_BCLK_PIN, I2S_WS_PIN, I2S_DIN_PIN);
    ESP_LOGI(TAG, "Static buffers: input=%d B, output=%d B",
             (int)AUDIO_INPUT_BYTES, (int)AUDIO_OUTPUT_BYTES);
    ESP_LOGI(TAG, "================================");

    current_volume = VOLUME_DEFAULT;

    /* Install I2S once and leave it installed for the lifetime of the
     * application.  Per-file work in audio_play_file() then only sets
     * the sample-rate clock. */
    if (!i2s_install_once()) {
        ESP_LOGE(TAG, "audio_init: I2S install failed -- playback will not work");
    }
}

/* -------------------------------------------------------------------------
 * Volume API (unchanged)
 * ---------------------------------------------------------------------- */
void audio_set_volume(int percent) {
    if (percent < VOLUME_MIN) percent = VOLUME_MIN;
    if (percent > VOLUME_MAX) percent = VOLUME_MAX;
    current_volume = percent;
    ESP_LOGI(TAG, "Volume set to %d%%", current_volume);
    announce_volume_percent(current_volume);
}

int audio_get_volume(void) { return current_volume; }

void audio_set_volume_silent(int percent) {
    if (percent < VOLUME_MIN) percent = VOLUME_MIN;
    if (percent > VOLUME_MAX) percent = VOLUME_MAX;
    current_volume = percent;
    ESP_LOGI(TAG, "Volume set (silent) to %d%%", current_volume);
}

void audio_volume_up(void) {
    int v = current_volume + 10;
    if (v > VOLUME_MAX) v = VOLUME_MAX;
    current_volume = v;
    ESP_LOGI(TAG, "Volume UP -> %d%%", current_volume);
    announce_volume_percent(current_volume);
}

void audio_volume_down(void) {
    int v = current_volume - 10;
    if (v < VOLUME_MIN) v = VOLUME_MIN;
    current_volume = v;
    ESP_LOGI(TAG, "Volume DOWN -> %d%%", current_volume);
    announce_volume_percent(current_volume);
}

void audio_volume_up_no_announce(void) {
    int v = current_volume + 10;
    if (v > VOLUME_MAX) v = VOLUME_MAX;
    current_volume = v;
    ESP_LOGI(TAG, "Volume UP (silent) -> %d%%", current_volume);
}

void audio_volume_down_no_announce(void) {
    int v = current_volume - 10;
    if (v < VOLUME_MIN) v = VOLUME_MIN;
    current_volume = v;
    ESP_LOGI(TAG, "Volume DOWN (silent) -> %d%%", current_volume);
}

/* -------------------------------------------------------------------------
 * WAV header parser (unchanged)
 * ---------------------------------------------------------------------- */
bool audio_parse_wav_header(const char *filepath, audio_wav_info_t *info) {
    if (!filepath || !info) return false;

    if (!sd_mutex_take()) {
        ESP_LOGW(TAG, "SD mutex timeout reading WAV header: %s", filepath);
        return false;
    }
    FILE *f = fopen(filepath, "rb");
    if (!f) { sd_mutex_give(); ESP_LOGE(TAG, "Failed to open: %s", filepath); return false; }

    uint8_t header[44];
    size_t got = fread(header, 1, 44, f);
    fclose(f);
    sd_mutex_give();

    if (got != 44) { ESP_LOGE(TAG, "Short WAV header (%zu bytes)", got); return false; }

    if (memcmp(header, "RIFF", 4) != 0 || memcmp(header + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Invalid WAV format: %s", filepath);
        return false;
    }

    info->channels        = *((uint16_t *)(header + 22));
    info->sample_rate     = *((uint32_t *)(header + 24));
    info->bits_per_sample = *((uint16_t *)(header + 34));
    info->data_size       = *((uint32_t *)(header + 40));
    info->data_offset     = 44;

    ESP_LOGI(TAG, "WAV Info: %lu Hz, %d-bit, %d ch, %lu bytes",
             (unsigned long)info->sample_rate, info->bits_per_sample,
             info->channels,                   (unsigned long)info->data_size);
    return true;
}

/* -------------------------------------------------------------------------
 * Playback — patched: no install/uninstall, no malloc/free
 * ---------------------------------------------------------------------- */
bool audio_play_file(const char *filepath,
                     volatile bool *stop_flag,
                     volatile bool *pause_flag) {
    if (!filepath) { ESP_LOGE(TAG, "NULL filepath"); return false; }

    ESP_LOGI(TAG, "Playing: %s", filepath);

    /* Defensive: if audio_init() was never called or its I2S install
     * failed, retry now so playback still has a chance to work. */
    if (!s_i2s_installed && !i2s_install_once()) {
        ESP_LOGE(TAG, "I2S not installed -- cannot play");
        return false;
    }

    audio_wav_info_t wav_info;
    if (!audio_parse_wav_header(filepath, &wav_info)) return false;

    if (wav_info.bits_per_sample != 16) {
        ESP_LOGE(TAG, "Only 16-bit PCM supported (got %d-bit)", wav_info.bits_per_sample);
        return false;
    }
    if (wav_info.channels < 1 || wav_info.channels > 2) {
        ESP_LOGE(TAG, "Unsupported channels: %d", wav_info.channels);
        return false;
    }

    ESP_LOGI(TAG, "Format: %lu Hz, %d-bit, %d channel(s)",
             (unsigned long)wav_info.sample_rate,
             wav_info.bits_per_sample, wav_info.channels);

    /* Switch sample rate for this file.  Cheap compared to a full
     * driver reinstall. */
    esp_err_t ret = i2s_set_clk(I2S_NUM_0, wav_info.sample_rate,
                                I2S_BITS_PER_SAMPLE_16BIT, 1);
    if (ret != ESP_OK) ESP_LOGW(TAG, "I2S clk warn: %s", esp_err_to_name(ret));

    /* Open the file. */
    if (!sd_mutex_take()) { ESP_LOGW(TAG, "SD mutex timeout opening: %s", filepath); return false; }
    FILE *f = fopen(filepath, "rb");
    if (!f) { sd_mutex_give(); ESP_LOGW(TAG, "File not found: %s", filepath); return false; }
    fseek(f, wav_info.data_offset, SEEK_SET);
    sd_mutex_give();

    const size_t in_frame_bytes = (wav_info.bits_per_sample / 8) * wav_info.channels;
    const size_t in_chunk_bytes = in_frame_bytes * AUDIO_CHUNK_FRAMES;
    /* Sanity: ensure the static buffer can hold a full chunk for this WAV. */
    if (in_chunk_bytes > AUDIO_INPUT_BYTES) {
        ESP_LOGE(TAG, "Chunk too large for static buffer (%zu > %zu)",
                 in_chunk_bytes, (size_t)AUDIO_INPUT_BYTES);
        if (sd_mutex_take()) { fclose(f); sd_mutex_give(); } else fclose(f);
        return false;
    }

    ESP_LOGI(TAG, "Starting playback loop...");

    while (1) {
        if (stop_flag  && *stop_flag)  { ESP_LOGI(TAG, "Playback stopped by flag"); break; }
        if (pause_flag && *pause_flag) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        size_t bytes_read = 0;
        if (sd_mutex_take()) {
            bytes_read = fread(s_input_buffer, 1, in_chunk_bytes, f);
            sd_mutex_give();
        } else {
            ESP_LOGD(TAG, "SD mutex timeout -- skipping chunk");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (bytes_read == 0) { ESP_LOGI(TAG, "End of file reached"); break; }

        size_t  frames_read = bytes_read / in_frame_bytes;
        int     vol         = current_volume;
        if (vol < VOLUME_MIN) vol = VOLUME_MIN;
        if (vol > VOLUME_MAX) vol = VOLUME_MAX;

        int16_t *samples_in  = (int16_t *)s_input_buffer;
        int16_t *samples_out = (int16_t *)s_output_buffer;

        if (wav_info.channels == 1) {
            for (size_t i = 0; i < frames_read; i++)
                samples_out[i] = clip_sample(((int32_t)samples_in[i] * vol) / 100);
        } else {
            for (size_t i = 0; i < frames_read; i++) {
                int32_t mixed = ((int32_t)samples_in[i*2] +
                                 (int32_t)samples_in[i*2 + 1]) / 2;
                samples_out[i] = clip_sample((mixed * vol) / 100);
            }
        }

        size_t output_bytes  = frames_read * sizeof(int16_t);
        size_t bytes_written = 0;
        ret = i2s_write(I2S_NUM_0, s_output_buffer, output_bytes,
                        &bytes_written, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) ESP_LOGW(TAG, "I2S write error: %s", esp_err_to_name(ret));

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (sd_mutex_take()) { fclose(f); sd_mutex_give(); } else fclose(f);

    /* Driver stays installed; no uninstall, no free.
     * Zero the DMA tail so the speaker amp does not latch on the
     * last sample written. */
    size_t silence_written = 0;
    static const int16_t silence[64] = {0};
    i2s_write(I2S_NUM_0, silence, sizeof(silence),
              &silence_written, pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "Playback finished");
    return true;
}
