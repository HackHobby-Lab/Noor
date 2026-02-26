/**
 * navigation.h
 * Navigation state machine for Noor Audio Player
 *
 * Levels:
 *   HOME -> FOLDER_VIEW -> SUBFOLDER_VIEW -> FILE_VIEW
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>

typedef enum {
    NAV_STATE_HOME = 0,
    NAV_STATE_FOLDER_VIEW,
    NAV_STATE_SUBFOLDER_VIEW,
    NAV_STATE_FILE_VIEW,
    NAV_STATE_QUIZ           /* Quiz active after story finishes */
} nav_state_t;

void        nav_init(void);
nav_state_t nav_get_state(void);
void        nav_set_state(nav_state_t state);

int  nav_get_selected_folder(void);
void nav_set_selected_folder(int index);

int  nav_get_selected_subfolder(void);
void nav_set_selected_subfolder(int index);

int  nav_get_selected_track(void);
void nav_set_selected_track(int index);

int  nav_next_folder(int total);
int  nav_prev_folder(int total);
int  nav_next_subfolder(int total);
int  nav_prev_subfolder(int total);
int  nav_next_track(int total);
int  nav_prev_track(int total);

bool nav_go_forward(void);
bool nav_go_back(void);

#endif // NAVIGATION_H
