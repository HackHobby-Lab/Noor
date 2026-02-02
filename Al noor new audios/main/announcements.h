/**
 * announcements.h
 * Voice announcement system for Noor Audio Player
 * 
 * This module handles:
 * - Playing announcement files (welcome.wav, home.wav, stories.wav, etc.)
 * - Automatic announcement detection based on folder/file names
 * - Interrupting current playback for announcements
 */

#ifndef ANNOUNCEMENTS_H
#define ANNOUNCEMENTS_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Maximum path length for announcement files */
#define ANNOUNCE_PATH_MAX  512

/* Notification bit for audio task */
#define NOTIFY_ANNOUNCE_BIT  (1u << 31)

/**
 * Initialize announcements system
 */
void announcements_init(void);

/**
 * Request an announcement to be played
 * This will interrupt any current playback
 * 
 * @param filepath Full path to announcement WAV file
 * @param audio_task_handle Handle to audio task for notification
 * @param stop_flag Pointer to stop flag to interrupt playback
 */
void announce_request(const char *filepath, 
                     TaskHandle_t audio_task_handle,
                     volatile bool *stop_flag);

/**
 * Get the current announcement path
 * (Used by audio task to know what to play)
 * 
 * @param buffer Buffer to copy path into
 * @param buffer_size Size of buffer
 * @return true if announcement is pending, false otherwise
 */
bool announce_get_pending(char *buffer, size_t buffer_size);

/**
 * Clear announcement flag after playing
 */
void announce_clear_pending(void);

/**
 * Check if announcement file exists and request it
 * Searches in root first, then in specified folder
 * 
 * @param root_path Root directory (e.g., "/sdcard")
 * @param folder_path Folder to search if not found in root (can be NULL)
 * @param filename Announcement filename (e.g., "stories.wav")
 * @param audio_task_handle Handle to audio task
 * @param stop_flag Pointer to stop flag
 * @return true if announcement was found and requested
 */
bool announce_check_and_play(const char *root_path,
                             const char *folder_path,
                             const char *filename,
                             TaskHandle_t audio_task_handle,
                             volatile bool *stop_flag);

/**
 * Get announcement filename for a folder name
 * E.g., folder "01" or "stories" -> "stories.wav"
 * 
 * @param folder_name Name of folder (last component of path)
 * @return Announcement filename, or NULL if no match
 */
const char* announce_get_folder_file(const char *folder_name);

/**
 * Get announcement filename for a story file
 * E.g., file "S1xyz.wav" -> "story1.wav"
 * 
 * @param filename Name of story file
 * @param buffer Buffer to store result
 * @param buffer_size Size of buffer
 * @return true if pattern matched and announcement exists
 */
bool announce_get_story_file(const char *filename, char *buffer, size_t buffer_size);

/**
 * Get announcement for entering a subfolder
 * E.g., "Hazrat Muhammed" -> "inside_prophet_Muhammad_stories_folder.wav"
 * 
 * @param folder_name Name of subfolder
 * @return Announcement filename, or NULL if no match
 */
const char* announce_get_entering_folder_file(const char *folder_name);

/**
 * Play boot greetings (welcome.wav, home.wav)
 * 
 * @param root_path Root directory (e.g., "/sdcard")
 */
void announce_play_boot_greetings(const char *root_path);

#endif // ANNOUNCEMENTS_H