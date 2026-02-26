/**
 * announcements.h
 * Voice announcement system for Noor Audio Player
 *
 * SD Card root files needed:
 *   HOME.WAV     - played whenever user returns to home position
 *   WELCOME.WAV  - played on device boot
 *   STORIES.WAV  - played when STORIES folder is highlighted
 *   v1.wav       - volume 10%
 *   v2.wav       - volume 20%
 *   ...
 *   v10.wav      - volume 100%
 *
 * TAWID folder must contain:
 *   TA.WAV       - played when TAWID folder is highlighted
 */

#ifndef ANNOUNCEMENTS_H
#define ANNOUNCEMENTS_H

#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ANNOUNCE_PATH_MAX   512
#define NOTIFY_ANNOUNCE_BIT (1u << 31)

void        announcements_init(void);

void        announce_request(const char *filepath,
                             TaskHandle_t audio_task_handle,
                             volatile bool *stop_flag);

bool        announce_get_pending(char *buffer, size_t buffer_size);
void        announce_clear_pending(void);

bool        announce_check_and_play(const char *root_path,
                                    const char *folder_path,
                                    const char *filename,
                                    TaskHandle_t audio_task_handle,
                                    volatile bool *stop_flag);

/* Returns WAV filename for a folder name, or NULL if not known.
 * NOTE: TAWID returns NULL intentionally - handled separately in main.c
 *       using TA.WAV from inside the TAWID folder. */
const char *announce_get_folder_file(const char *folder_name);

bool        announce_get_story_file(const char *filename,
                                    char *buffer, size_t buffer_size);

const char *announce_get_entering_folder_file(const char *folder_name);

/* Plays WELCOME.WAV then ROTATE.WAV on boot.
 * Pass &g_stop_flag so USB MSC premount can cleanly stop playback
 * before FAT is unmounted. */
void        announce_play_boot_greetings(const char *root_path,
                                         volatile bool *stop_flag);

/* Maps volume 0-100% to v1.wav..v10.wav
 * e.g. 60% -> "v6.wav", 100% -> "v10.wav", 0% -> "v1.wav" */
bool        announce_get_volume_file(int volume_percent,
                                     char *buffer, size_t buffer_size);

#endif // ANNOUNCEMENTS_H
