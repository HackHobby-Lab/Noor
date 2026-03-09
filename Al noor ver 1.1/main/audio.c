/**
 * audio.c
 * Audio playback implementation
 *
 * Changes vs previous version:
 * ----------------------------
 * 1. SD_LOCK() now uses a BOUNDED timeout (SD_MUTEX_TIMEOUT_MS = 50ms)
 *    instead of portMAX_DELAY.
 *
 *    The crash:
 *      Guru Meditation Error: Core 0 panic'ed (Interrupt wdt timeout on CPU0)
 *      i2s_write at i2s_legacy.c:1713
 *      audio_play_file at audio.c:290
 *
 *    Cause: audio_task held the SD mutex inside fread() with portMAX_DELAY.
 *    While blocked, the I2S DMA interrupt fired. i2s_write() internally uses
 *    a FreeRTOS queue whose spinlock contended with the same critical section
 *    that the SD mutex path was holding. CPU0 stayed in an uninterruptible
 *    state until the watchdog (5s) expired.
 *
 *    Fix: replace portMAX_DELAY with pdMS_TO_TICKS(50). If the USB MSC
 *    callback holds the mutex, we wait at most 50ms — safely inside the WDT
 *    window. On timeout we skip the chunk (tiny glitch) rather than crashing.
 *
 * 2. vTaskDelay(1) added after every i2s_write() to yield CPU0 back to the
 *    scheduler. This feeds the IDLE/WDT task and prevents the audio loop
 *    from monopolising the core between DMA interrupts.
 *
 * 3. I2S DMA buffer count raised from 4 to 8 to reduce DMA interrupt
 *    pressure and give more headroom between fread() and i2s_write().
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

static volatile int current_volume  = VOLUME_DEFAULT;
static bool         s_i2s_installed = false;

/* -------------------------------------------------------------------------
 * SD mutex helpers — bounded timeout, never portMAX_DELAY
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

/* -------------------------------------------------------------------------
 * Utility
 * ---------------------------------------------------------------------- */
static inline int16_t clip_sample(int32_t s) {
    if (s >  32767) return  32767;
    if (s < -32768) return -32768;
    return (int16_t)s;
}

/* -------------------------------------------------------------------------
 * Public API — volume
 * ---------------------------------------------------------------------- */
void audio_init(void) {
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Audio System Initialization");
    ESP_LOGI(TAG, "Default Volume: %d%% (%d/100)", VOLUME_DEFAULT, VOLUME_DEFAULT);
    ESP_LOGI(TAG, "I2S Pins: BCLK=%d, WS=%d, DIN=%d", I2S_BCLK_PIN, I2S_WS_PIN, I2S_DIN_PIN);
    ESP_LOGI(TAG, "================================");
    current_volume = VOLUME_DEFAULT;
    ESP_LOGI(TAG, "Audio system initialized (volume=%d%%)", current_volume);
}

void audio_set_volume(int percent) {
    if (percent < VOLUME_MIN) percent = VOLUME_MIN;
    if (percent > VOLUME_MAX) percent = VOLUME_MAX;
    current_volume = percent;
    ESP_LOGI(TAG, "Volume set to %d%%", current_volume);
    announce_volume_percent(current_volume);
}

int audio_get_volume(void) {
    return current_volume;
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

/* -------------------------------------------------------------------------
 * WAV header parser (mutex-bounded)
 * ---------------------------------------------------------------------- */
bool audio_parse_wav_header(const char *filepath, audio_wav_info_t *info) {
    if (!filepath || !info) return false;

    if (!sd_mutex_take()) {
        ESP_LOGW(TAG, "SD mutex timeout reading WAV header: %s", filepath);
        return false;
    }
    FILE *f = fopen(filepath, "rb");
    if (!f) {
        sd_mutex_give();
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        return false;
    }

    uint8_t header[44];
    size_t got = fread(header, 1, 44, f);
    fclose(f);
    sd_mutex_give();

    if (got != 44) {
        ESP_LOGE(TAG, "Failed to read WAV header (%zu bytes)", got);
        return false;
    }

    if (memcmp(header, "RIFF", 4) != 0 || memcmp(header + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Invalid WAV file format");
        return false;
    }

    info->channels        = *((uint16_t *)(header + 22));
    info->sample_rate     = *((uint32_t *)(header + 24));
    info->bits_per_sample = *((uint16_t *)(header + 34));
    info->data_size       = *((uint32_t *)(header + 40));
    info->data_offset     = 44;

    ESP_LOGI(TAG, "WAV Info: %lu Hz, %d-bit, %d ch, %lu bytes",
             info->sample_rate, info->bits_per_sample,
             info->channels, info->data_size);
    return true;
}

/* -------------------------------------------------------------------------
 * Playback
 * ---------------------------------------------------------------------- */
bool audio_play_file(const char *filepath,
                     volatile bool *stop_flag,
                     volatile bool *pause_flag) {
    if (!filepath) {
        ESP_LOGE(TAG, "NULL filepath");
        return false;
    }

    ESP_LOGI(TAG, "Playing: %s", filepath);

    audio_wav_info_t wav_info;
    if (!audio_parse_wav_header(filepath, &wav_info)) {
        ESP_LOGE(TAG, "Failed to parse WAV header: %s", filepath);
        return false;
    }

    if (wav_info.bits_per_sample != 16) {
        ESP_LOGE(TAG, "Only 16-bit PCM supported (got %d-bit)", wav_info.bits_per_sample);
        return false;
    }

    if (wav_info.channels < 1 || wav_info.channels > 2) {
        ESP_LOGE(TAG, "Unsupported channel count: %d", wav_info.channels);
        return false;
    }

    ESP_LOGI(TAG, "Format: %lu Hz, %d-bit, %d channel(s)",
             wav_info.sample_rate, wav_info.bits_per_sample, wav_info.channels);

    /* Open file for playback */
    if (!sd_mutex_take()) {
        ESP_LOGW(TAG, "SD mutex timeout opening: %s", filepath);
        return false;
    }
    FILE *f = fopen(filepath, "rb");
    if (!f) {
        sd_mutex_give();
        ESP_LOGW(TAG, "File not found: %s", filepath);
        return false;
    }
    fseek(f, wav_info.data_offset, SEEK_SET);
    sd_mutex_give();

    /* Install I2S driver */
    if (s_i2s_installed) {
        i2s_driver_uninstall(I2S_NUM_0);
        s_i2s_installed = false;
    }

    i2s_config_t i2s_config = {
        .mode                 = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate          = (int)wav_info.sample_rate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags     = 0,
        .dma_buf_count        = 8,     /* raised from 4 — reduces DMA interrupt pressure */
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
        ESP_LOGE(TAG, "I2S driver install failed: %s", esp_err_to_name(ret));
        if (sd_mutex_take()) { fclose(f); sd_mutex_give(); } else { fclose(f); }
        return false;
    }
    s_i2s_installed = true;

    ret = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S set pin failed: %s", esp_err_to_name(ret));
        i2s_driver_uninstall(I2S_NUM_0);
        s_i2s_installed = false;
        if (sd_mutex_take()) { fclose(f); sd_mutex_give(); } else { fclose(f); }
        return false;
    }

    ret = i2s_set_clk(I2S_NUM_0, wav_info.sample_rate, I2S_BITS_PER_SAMPLE_16BIT, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2S set clk warning: %s", esp_err_to_name(ret));
    }

    /* Buffers */
    const size_t chunk_frames   = 512;
    const size_t in_frame_bytes = (wav_info.bits_per_sample / 8) * wav_info.channels;
    const size_t in_chunk_bytes = in_frame_bytes * chunk_frames;

    uint8_t *input_buffer  = malloc(in_chunk_bytes);
    uint8_t *output_buffer = malloc(chunk_frames * sizeof(int16_t));

    if (!input_buffer || !output_buffer) {
        ESP_LOGE(TAG, "Failed to allocate audio buffers");
        free(input_buffer);
        free(output_buffer);
        if (sd_mutex_take()) { fclose(f); sd_mutex_give(); } else { fclose(f); }
        i2s_driver_uninstall(I2S_NUM_0);
        s_i2s_installed = false;
        return false;
    }

    ESP_LOGI(TAG, "Starting playback loop...");

    /* ---- Playback loop ---- */
    while (1) {
        if (stop_flag && *stop_flag) {
            ESP_LOGI(TAG, "Playback stopped by flag");
            break;
        }

        if (pause_flag && *pause_flag) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Bounded fread — never block longer than SD_MUTEX_TIMEOUT_MS.
         * If USB MSC holds the mutex: skip chunk (tiny glitch) rather
         * than blocking until the interrupt watchdog fires. */
        size_t bytes_read = 0;
        if (sd_mutex_take()) {
            bytes_read = fread(input_buffer, 1, in_chunk_bytes, f);
            sd_mutex_give();
        } else {
            ESP_LOGD(TAG, "SD mutex timeout during playback, skipping chunk");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (bytes_read == 0) {
            ESP_LOGI(TAG, "End of file reached");
            break;
        }

        size_t frames_read = bytes_read / in_frame_bytes;

        int vol = current_volume;
        if (vol < VOLUME_MIN) vol = VOLUME_MIN;
        if (vol > VOLUME_MAX) vol = VOLUME_MAX;

        int16_t *samples_in  = (int16_t *)input_buffer;
        int16_t *samples_out = (int16_t *)output_buffer;

        if (wav_info.channels == 1) {
            for (size_t i = 0; i < frames_read; i++) {
                samples_out[i] = clip_sample(((int32_t)samples_in[i] * vol) / 100);
            }
        } else {
            for (size_t i = 0; i < frames_read; i++) {
                int32_t mixed = ((int32_t)samples_in[i * 2] +
                                 (int32_t)samples_in[i * 2 + 1]) / 2;
                samples_out[i] = clip_sample((mixed * vol) / 100);
            }
        }

        size_t output_bytes  = frames_read * sizeof(int16_t);
        size_t bytes_written = 0;
        ret = i2s_write(I2S_NUM_0, output_buffer, output_bytes,
                        &bytes_written, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2S write error: %s", esp_err_to_name(ret));
        }

        /* Yield after every chunk — feeds WDT IDLE task and lets USB MSC,
         * headphone detect, and encoder tasks get CPU time. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* Cleanup */
    free(input_buffer);
    free(output_buffer);

    if (sd_mutex_take()) { fclose(f); sd_mutex_give(); }
    else                  { fclose(f); }

    i2s_driver_uninstall(I2S_NUM_0);
    s_i2s_installed = false;

    ESP_LOGI(TAG, "Playback finished");
    return true;
}
