/**
 * navigation.c
 * Navigation state machine implementation
 */

#include "navigation.h"
#include "esp_log.h"

static const char *TAG = "NAV";

static nav_state_t current_state      = NAV_STATE_HOME;
static int         selected_folder    = 0;
static int         selected_subfolder = 0;
static int         selected_track     = 0;

void nav_init(void) {
    current_state      = NAV_STATE_HOME;
    selected_folder    = 0;
    selected_subfolder = 0;
    selected_track     = 0;
    ESP_LOGI(TAG, "Navigation initialized at HOME");
}

nav_state_t nav_get_state(void)              { return current_state; }
void        nav_set_state(nav_state_t state) { current_state = state; }

int  nav_get_selected_folder(void)    { return selected_folder; }
void nav_set_selected_folder(int i)   { if (i >= 0) selected_folder = i; }

int  nav_get_selected_subfolder(void) { return selected_subfolder; }
void nav_set_selected_subfolder(int i){ if (i >= 0) selected_subfolder = i; }

int  nav_get_selected_track(void)     { return selected_track; }
void nav_set_selected_track(int i)    { if (i >= 0) selected_track = i; }

int nav_next_folder(int total) {
    if (total <= 0) return 0;
    selected_folder = (selected_folder + 1) % total;
    return selected_folder;
}

int nav_prev_folder(int total) {
    if (total <= 0) return 0;
    selected_folder = (selected_folder - 1 + total) % total;
    return selected_folder;
}

int nav_next_subfolder(int total) {
    if (total <= 0) return 0;
    selected_subfolder = (selected_subfolder + 1) % total;
    return selected_subfolder;
}

int nav_prev_subfolder(int total) {
    if (total <= 0) return 0;
    selected_subfolder = (selected_subfolder - 1 + total) % total;
    return selected_subfolder;
}

int nav_next_track(int total) {
    if (total <= 0) return 0;
    selected_track = (selected_track + 1) % total;
    return selected_track;
}

int nav_prev_track(int total) {
    if (total <= 0) return 0;
    selected_track = (selected_track - 1 + total) % total;
    return selected_track;
}

bool nav_go_forward(void) {
    switch (current_state) {
        case NAV_STATE_HOME:
            current_state = NAV_STATE_FOLDER_VIEW;
            ESP_LOGI(TAG, "HOME -> FOLDER_VIEW");
            return true;
        case NAV_STATE_FOLDER_VIEW:
            current_state      = NAV_STATE_SUBFOLDER_VIEW;
            selected_subfolder = 0;
            ESP_LOGI(TAG, "FOLDER_VIEW -> SUBFOLDER_VIEW");
            return true;
        case NAV_STATE_SUBFOLDER_VIEW:
            current_state  = NAV_STATE_FILE_VIEW;
            selected_track = 0;
            ESP_LOGI(TAG, "SUBFOLDER_VIEW -> FILE_VIEW");
            return true;
        default:
            return false;
    }
}

bool nav_go_back(void) {
    switch (current_state) {
        case NAV_STATE_FILE_VIEW:
            current_state = NAV_STATE_SUBFOLDER_VIEW;
            ESP_LOGI(TAG, "FILE_VIEW -> SUBFOLDER_VIEW");
            return true;
        case NAV_STATE_SUBFOLDER_VIEW:
            current_state = NAV_STATE_FOLDER_VIEW;
            ESP_LOGI(TAG, "SUBFOLDER_VIEW -> FOLDER_VIEW");
            return true;
        case NAV_STATE_FOLDER_VIEW:
            current_state = NAV_STATE_HOME;
            ESP_LOGI(TAG, "FOLDER_VIEW -> HOME");
            return true;
        default:
            return false;
    }
}
