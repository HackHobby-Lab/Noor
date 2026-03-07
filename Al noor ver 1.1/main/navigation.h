/**
 * navigation.h
 * Navigation state machine interface
 */

#pragma once

#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Navigation states
 * ----------------------------------------------------------------------- */
typedef enum {
    NAV_STATE_HOME           = 0,  /* Top level — no folder selected */
    NAV_STATE_FOLDER_VIEW    = 1,  /* Browsing top-level folders */
    NAV_STATE_SUBFOLDER_VIEW = 2,  /* Browsing subfolders of selected folder */
    NAV_STATE_FILE_VIEW      = 3,  /* Browsing / playing files */
} nav_state_t;

/* -----------------------------------------------------------------------
 * Lifecycle
 * ----------------------------------------------------------------------- */
void         nav_init(void);

/* -----------------------------------------------------------------------
 * State access
 * ----------------------------------------------------------------------- */
nav_state_t  nav_get_state(void);
void         nav_set_state(nav_state_t state);
bool         nav_is_in_playback_mode(void);

/* -----------------------------------------------------------------------
 * Folder selection
 * ----------------------------------------------------------------------- */
int  nav_get_selected_folder(void);
void nav_set_selected_folder(int index);
int  nav_next_folder(int total_folders);
int  nav_prev_folder(int total_folders);

/* -----------------------------------------------------------------------
 * Subfolder selection
 * ----------------------------------------------------------------------- */
int  nav_get_selected_subfolder(void);
void nav_set_selected_subfolder(int index);
int  nav_next_subfolder(int total_subfolders);
int  nav_prev_subfolder(int total_subfolders);

/* -----------------------------------------------------------------------
 * Track selection
 * ----------------------------------------------------------------------- */
int  nav_get_selected_track(void);
void nav_set_selected_track(int index);
int  nav_next_track(int total_tracks);
int  nav_prev_track(int total_tracks);

/* -----------------------------------------------------------------------
 * State transitions
 * ----------------------------------------------------------------------- */

/** Standard forward: HOME->FOLDER->SUBFOLDER->FILE */
bool nav_go_forward(void);

/** Standard back: FILE->SUBFOLDER->FOLDER->HOME */
bool nav_go_back(void);

/**
 * Direct forward: FOLDER_VIEW -> FILE_VIEW  (skip SUBFOLDER_VIEW).
 * Use this when the selected folder has no subfolders (e.g. TAWID).
 */
bool nav_go_to_files_direct(void);

/**
 * Direct back: FILE_VIEW -> FOLDER_VIEW  (skip SUBFOLDER_VIEW).
 * Pair with nav_go_to_files_direct().
 */
bool nav_go_back_from_files_direct(void);
