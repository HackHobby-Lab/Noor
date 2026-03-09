/**
 * audio.c
 * Audio playback implementation
 */

#include "audio.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_log.h"

static const char *TAG = "AUDIO";

/* Current volume level */
static volatile int current_volume = VOLUME_DEFAULT;

/* Stop flag registered by main.c — allows USB manager to halt playback */
static volatile bool *s_stop_flag_ptr = NULL;

void audio_register_stop_flag(volatile bool *flag) { s_stop_flag_ptr = flag; }
void audio_stop_immediate(void) { if (s_stop_flag_ptr) *s_stop_flag_ptr = true; }

/* Helper: Clip audio sample to 16-bit range */
static inline int16_t clip_sample(int32_t sample) {
    if (sample > 32767)  return 32767;
    if (sample < -32768) return -32768;
    return (int16_t)sample;
}

void audio_init(void) {
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Audio System Initialization");
    ESP_LOGI(TAG, "Default Volume: %d%%", VOLUME_DEFAULT);
    ESP_LOGI(TAG, "I2S Pins: BCLK=%d, WS=%d, DO=%d", I2S_BCK_PIN, I2S_WS_PIN, I2S_DO_PIN);
    ESP_LOGI(TAG, "================================");
}

void audio_set_volume(int percent) {
    if (percent < VOLUME_MIN) percent = VOLUME_MIN;
    if (percent > VOLUME_MAX) percent = VOLUME_MAX;
    current_volume = percent;
    ESP_LOGI(TAG, "Volume set to %d%%", current_volume);
}

int audio_get_volume(void) {
    return current_volume;
}

void audio_volume_up(void) {
    int new_vol = current_volume + 10;
    if (new_vol > VOLUME_MAX) new_vol = VOLUME_MAX;
    current_volume = new_vol;
    ESP_LOGI(TAG, "Volume UP -> %d%%", current_volume);
}

void audio_volume_down(void) {
    int new_vol = current_volume - 10;
    if (new_vol < VOLUME_MIN) new_vol = VOLUME_MIN;
    current_volume = new_vol;
    ESP_LOGI(TAG, "Volume DOWN -> %d%%", current_volume);
}

bool audio_parse_wav_header(const char *filepath, audio_wav_info_t *info) {
    if (!filepath || !info) return false;

    FILE *f = fopen(filepath, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open: %s", filepath);
        return false;
    }

    uint8_t header[44];
    if (fread(header, 1, 44, f) != 44) {
        fclose(f);
        return false;
    }

    if (memcmp(header, "RIFF", 4) != 0 || memcmp(header + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Invalid WAV format");
        fclose(f);
        return false;
    }

    info->channels       = *((uint16_t *)(header + 22));
    info->sample_rate    = *((uint32_t *)(header + 24));
    info->bits_per_sample = *((uint16_t *)(header + 34));
    info->data_size      = *((uint32_t *)(header + 40));
    info->data_offset    = 44;

    fclose(f);
    return true;
}

bool audio_play_file(const char *filepath,
                     volatile bool *stop_flag,
                     volatile bool *pause_flag) {
    if (!filepath) return false;

    ESP_LOGI(TAG, "Playing: %s", filepath);

    FILE *f = fopen(filepath, "rb");
    if (!f) {
        ESP_LOGW(TAG, "File not found: %s", filepath);
        return false;
    }

    /* Parse WAV header */
    uint8_t header[44];
    if (fread(header, 1, 44, f) != 44) {
        fclose(f);
        return false;
    }

    if (memcmp(header, "RIFF", 4) != 0 || memcmp(header + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Invalid WAV format: %s", filepath);
        fclose(f);
        return false;
    }

    audio_wav_info_t wav;
    wav.channels        = *((uint16_t *)(header + 22));
    wav.sample_rate     = *((uint32_t *)(header + 24));
    wav.bits_per_sample = *((uint16_t *)(header + 34));
    wav.data_size       = *((uint32_t *)(header + 40));
    wav.data_offset     = 44;

    if (wav.bits_per_sample != 16) {
        ESP_LOGE(TAG, "Only 16-bit PCM supported");
        fclose(f);
        return false;
    }

    ESP_LOGI(TAG, "WAV: %lu Hz, %d ch, %d-bit",
             wav.sample_rate, wav.channels, wav.bits_per_sample);

    /* Configure I2S */
    i2s_config_t i2s_config = {
        .mode                 = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate          = (int)wav.sample_rate,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags     = 0,
        .dma_buf_count        = 4,
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
        fclose(f);
        return false;
    }

    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, wav.sample_rate, I2S_BITS_PER_SAMPLE_16BIT, 1);

    fseek(f, wav.data_offset, SEEK_SET);

    const size_t chunk_frames   = 512;
    const size_t in_frame_bytes = (wav.bits_per_sample / 8) * wav.channels;
    const size_t in_chunk_bytes = in_frame_bytes * chunk_frames;

    uint8_t  *input_buffer  = malloc(in_chunk_bytes);
    uint8_t  *output_buffer = malloc(chunk_frames * sizeof(int16_t));

    if (!input_buffer || !output_buffer) {
        ESP_LOGE(TAG, "Buffer alloc failed");
        free(input_buffer);
        free(output_buffer);
        fclose(f);
        i2s_driver_uninstall(I2S_NUM_0);
        return false;
    }

    /* Playback loop */
    while (1) {
        if (stop_flag && *stop_flag) break;

        if (pause_flag && *pause_flag) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        size_t bytes_read = fread(input_buffer, 1, in_chunk_bytes, f);
        if (bytes_read == 0) break;

        size_t frames_read = bytes_read / in_frame_bytes;
        int    vol         = current_volume;

        int16_t *in  = (int16_t *)input_buffer;
        int16_t *out = (int16_t *)output_buffer;

        if (wav.channels == 1) {
            for (size_t i = 0; i < frames_read; i++) {
                out[i] = clip_sample(((int32_t)in[i] * vol) / 100);
            }
        } else {
            /* Stereo -> mono (left channel) */
            for (size_t i = 0; i < frames_read; i++) {
                out[i] = clip_sample(((int32_t)in[i * 2] * vol) / 100);
            }
        }

        size_t bytes_written = 0;
        i2s_write(I2S_NUM_0, output_buffer, frames_read * sizeof(int16_t),
                  &bytes_written, pdMS_TO_TICKS(1000));

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(input_buffer);
    free(output_buffer);
    fclose(f);
    i2s_driver_uninstall(I2S_NUM_0);

    ESP_LOGI(TAG, "Playback finished: %s", filepath);
    return true;
}
