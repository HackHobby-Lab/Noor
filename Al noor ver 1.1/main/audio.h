/**
 * audio.h
 * Audio playback interface
 *
 * All pin definitions and volume limits come from config.h.
 * Do NOT redefine them here.
 */

#pragma once

#include "config.h"   /* VOLUME_MIN, VOLUME_MAX, VOLUME_DEFAULT, I2S pins */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -----------------------------------------------------------------------
 * WAV file information structure
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t channels;
    uint32_t sample_rate;
    uint16_t bits_per_sample;
    uint32_t data_size;
    uint32_t data_offset;
} audio_wav_info_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */
void  audio_init(void);
void  audio_set_volume(int percent);
int   audio_get_volume(void);
void  audio_volume_up(void);
void  audio_volume_down(void);
bool  audio_parse_wav_header(const char *filepath, audio_wav_info_t *info);
bool  audio_play_file(const char *filepath,
                      volatile bool *stop_flag,
                      volatile bool *pause_flag);
