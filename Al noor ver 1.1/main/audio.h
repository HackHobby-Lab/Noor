/**
 * audio.h
 * Audio playback system for Noor Audio Player
 * 
 * This module handles:
 * - WAV file parsing (16-bit PCM only)
 * - I2S audio output (mono, MAX98357)
 * - Volume control with clipping
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

/* Volume limits */
#define VOLUME_MIN      0
#define VOLUME_MAX      200
#define VOLUME_DEFAULT  60   // Conservative safe default

/**
 * WAV file information structure
 */
typedef struct {
    uint32_t sample_rate;       // Sample rate in Hz (e.g., 44100)
    uint16_t bits_per_sample;   // Bit depth (only 16 supported)
    uint16_t channels;          // 1 = mono, 2 = stereo
    uint32_t data_size;         // Size of audio data in bytes
    uint32_t data_offset;       // Offset to audio data in file
} audio_wav_info_t;

/**
 * Initialize audio system
 * Call this once at startup
 */
void audio_init(void);

/**
 * Play a WAV file with interrupt capability
 * 
 * @param filepath Full path to WAV file (e.g., "/sdcard/01/story1.wav")
 * @param stop_flag Pointer to flag - set to true to stop playback
 * @param pause_flag Pointer to flag - set to true to pause, false to resume
 * @return true if played successfully, false on error
 */
bool audio_play_file(const char *filepath, 
                     volatile bool *stop_flag, 
                     volatile bool *pause_flag);

/**
 * Set volume level
 * 
 * @param percent Volume percentage (0-200)
 *                0 = mute
 *                100 = original volume
 *                200 = 2x amplification (may clip)
 */
void audio_set_volume(int percent);

/**
 * Get current volume level
 * 
 * @return Current volume percentage
 */
int audio_get_volume(void);

/**
 * Increase volume by 10%
 */
void audio_volume_up(void);

/**
 * Decrease volume by 10%
 */
void audio_volume_down(void);

/**
 * Parse WAV file header
 * 
 * @param filepath Path to WAV file
 * @param info Pointer to structure to fill with WAV information
 * @return true if valid WAV, false otherwise
 */
bool audio_parse_wav_header(const char *filepath, audio_wav_info_t *info);

#endif // AUDIO_H