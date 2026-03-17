/**
 * announcements.h
 * Voice announcement system interface
 *
 * ANNOUNCE_PATH_MAX and NOTIFY_ANNOUNCE_BIT are defined in config.h.
 * Do NOT redefine them here.
 */

#pragma once

#include "config.h"   /* ANNOUNCE_PATH_MAX, NOTIFY_ANNOUNCE_BIT */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -----------------------------------------------------------------------
 * Lifecycle
 *
 * Call announcements_init() AFTER sd_card_init() succeeds so the
 * announcements directory (/sdcard/ANNOUN~1) is already mounted.
 * ----------------------------------------------------------------------- */
void announcements_init(void);

/* -----------------------------------------------------------------------
 * Pending-announcement queue (single slot)
 * ----------------------------------------------------------------------- */
void announce_request(const char *filepath,
                      TaskHandle_t audio_task_handle,
                      volatile bool *stop_flag);

bool announce_get_pending(char *buffer, size_t buffer_size);
void announce_clear_pending(void);

/* -----------------------------------------------------------------------
 * File lookup helpers
 * ----------------------------------------------------------------------- */
const char *announce_get_folder_file(const char *folder_name);
bool        announce_get_story_file(const char *filename,
                                    char *buffer, size_t buffer_size);
const char *announce_get_entering_folder_file(const char *folder_name);

/* -----------------------------------------------------------------------
 * Playback helpers
 * ----------------------------------------------------------------------- */
bool announce_check_and_play(const char *root_path,
                              const char *folder_path,
                              const char *filename,
                              TaskHandle_t audio_task_handle,
                              volatile bool *stop_flag);

/**
 * Play boot greetings (WELCOME.WAV / welcome.wave then ROTATE.WAV).
 * root_path is kept for backward compatibility but may be NULL.
 * Call AFTER sd_card_init() and announcements_init().
 */
void announce_play_boot_greetings(const char *root_path);

/**
 * Play an announcement file by filename only, synchronously.
 * Returns true if the file was found and played.
 */
bool announce_play_direct(const char *filename);

/**
 * Look up and play the track announcement for wav_filename synchronously.
 * Call immediately before starting story playback.
 * Example: announce_play_track("a1.wav") plays A1ANN.WAV.
 */
void announce_play_track(const char *wav_filename);

void announce_volume_percent(int volume_percent);

void announce_volume(int volume_percent);