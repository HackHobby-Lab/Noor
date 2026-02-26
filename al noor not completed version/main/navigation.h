/**
 * navigation.h
 * Navigation state machine for Noor Audio Player
 * 
 * This module handles:
 * - Three-level navigation (HOME → FOLDER_VIEW → FILE_VIEW)
 * - Folder and file selection
 * - State transitions
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>

/**
 * Navigation states
 */
typedef enum {
    NAV_STATE_HOME = 0,        // Home screen (Level 1)
    NAV_STATE_FOLDER_VIEW,     // Browsing main folders (Level 2) - e.g., "Stories of Prophets"
    NAV_STATE_SUBFOLDER_VIEW,  // Browsing subfolders (Level 3) - e.g., "Hazrat Muhammed"
    NAV_STATE_FILE_VIEW        // Browsing files in a subfolder (Level 4)
} nav_state_t;

/**
 * Initialize navigation system
 */
void nav_init(void);

/**
 * Get current navigation state
 * 
 * @return Current state
 */
nav_state_t nav_get_state(void);

/**
 * Set navigation state
 * 
 * @param state New state to set
 */
void nav_set_state(nav_state_t state);

/**
 * Get currently selected folder index
 * 
 * @return Folder index (0-based)
 */
int nav_get_selected_folder(void);

/**
 * Set selected folder index
 * 
 * @param index Folder index
 */
void nav_set_selected_folder(int index);

/**
 * Get currently selected file/track index
 * 
 * @return File index (0-based)
 */
int nav_get_selected_track(void);

/**
 * Set selected track index
 * 
 * @param index Track index
 */
void nav_set_selected_track(int index);

/**
 * Get currently selected subfolder index
 * 
 * @return Subfolder index (0-based)
 */
int nav_get_selected_subfolder(void);

/**
 * Set selected subfolder index
 * 
 * @param index Subfolder index
 */
void nav_set_selected_subfolder(int index);

/**
 * Move to next folder (with wrap-around)
 * 
 * @param total_folders Total number of folders available
 * @return New selected folder index
 */
int nav_next_folder(int total_folders);

/**
 * Move to previous folder (with wrap-around)
 * 
 * @param total_folders Total number of folders available
 * @return New selected folder index
 */
int nav_prev_folder(int total_folders);

/**
 * Move to next track (with wrap-around)
 * 
 * @param total_tracks Total number of tracks available
 * @return New selected track index
 */
int nav_next_track(int total_tracks);

/**
 * Move to previous track (with wrap-around)
 * 
 * @param total_tracks Total number of tracks available
 * @return New selected track index
 */
int nav_prev_track(int total_tracks);

/**
 * Move to next subfolder (with wrap-around)
 * 
 * @param total_subfolders Total number of subfolders available
 * @return New selected subfolder index
 */
int nav_next_subfolder(int total_subfolders);

/**
 * Move to previous subfolder (with wrap-around)
 * 
 * @param total_subfolders Total number of subfolders available
 * @return New selected subfolder index
 */
int nav_prev_subfolder(int total_subfolders);

/**
 * Navigate back (FILE_VIEW → SUBFOLDER_VIEW → FOLDER_VIEW → HOME)
 * 
 * @return true if navigation changed, false if already at HOME
 */
bool nav_go_back(void);

/**
 * Navigate forward (HOME → FOLDER_VIEW → SUBFOLDER_VIEW → FILE_VIEW)
 * Requires external logic to scan folders/files
 * 
 * @return true if navigation changed, false if can't go forward
 */
bool nav_go_forward(void);

/**
 * Check if we're in playback mode (FILE_VIEW with playing flag)
 */
bool nav_is_in_playback_mode(void);

#endif // NAVIGATION_H