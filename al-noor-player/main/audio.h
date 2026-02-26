/**
 * audio.h
 * Audio playback system for Noor Audio Player
 *
 * Handles:
 * - WAV file parsing (16-bit PCM only)
 * - I2S audio output (mono, MAX98357)
 * - Volume control (0-100%, steps of 10)
 * - Pause/resume/stop functionality
 */

#ifndef AUDIO_H
#define AUDIO_H

#include "config.h"
#include <stdbool.h>
#include <stdint.h>

/* I2S Pin definitions for MAX98357 */
#define I2S_BCK_PIN  18   // Bit clock
#define I2S_WS_PIN   17   // Word select (LRCLK)
#define I2S_DO_PIN   16   // Data output

/* Volume limits are defined in config.h:
 * VOLUME_MIN     = 0
 * VOLUME_MAX     = 100   (maps to v1.wav..v10.wav)
 * VOLUME_DEFAULT = 60
 */

/**
 * WAV file information structure
 */
typedef struct {
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint16_t channels;
    uint32_t data_size;
    uint32_t data_offset;
} audio_wav_info_t;

void audio_init(void);

/**
 * Register the main stop flag so the USB manager can halt playback
 * from a different task context without needing main.c's internals.
 * Call once from app_main() immediately after audio_init().
 */
void audio_register_stop_flag(volatile bool *flag);

/**
 * Immediately signal audio playback to stop.
 * Safe to call from any task (USB manager premount callback).
 */
void audio_stop_immediate(void);

bool audio_play_file(const char *filepath,
                     volatile bool *stop_flag,
                     volatile bool *pause_flag);

void audio_set_volume(int percent);
int  audio_get_volume(void);
void audio_volume_up(void);
void audio_volume_down(void);

bool audio_parse_wav_header(const char *filepath, audio_wav_info_t *info);

#endif // AUDIO_H
