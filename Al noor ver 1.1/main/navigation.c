/**
 * navigation.c
 * Navigation state machine implementation
 *
 * State flow:
 *   HOME -> FOLDER_VIEW -> SUBFOLDER_VIEW -> FILE_VIEW
 *
 * For folders that contain files directly (no subfolders, e.g. TAWID),
 * pressing Play in FOLDER_VIEW transitions straight to FILE_VIEW
 * via nav_go_to_files_direct().  The caller detects this by checking
 * sd_get_subfolder_count() == 0 after scanning.
 */

#include "navigation.h"
#include "esp_log.h"

static const char *TAG = "NAVIGATION";

/* Current navigation state */
static nav_state_t current_state = NAV_STATE_HOME;

/* Selection indices */
static int selected_folder    = 0;
static int selected_subfolder = 0;
static int selected_track     = 0;

void nav_init(void) {
    current_state     = NAV_STATE_HOME;
    selected_folder   = 0;
    selected_subfolder = 0;
    selected_track    = 0;
    ESP_LOGI(TAG, "Navigation initialized at HOME");
}

nav_state_t nav_get_state(void) {
    return current_state;
}

void nav_set_state(nav_state_t state) {
    if (current_state != state) {
        ESP_LOGI(TAG, "State change: %d -> %d", current_state, state);
        current_state = state;
    }
}

int nav_get_selected_folder(void) { return selected_folder; }

void nav_set_selected_folder(int index) {
    if (index >= 0) {
        selected_folder = index;
        ESP_LOGI(TAG, "Selected folder: %d", selected_folder);
    }
}

int nav_get_selected_track(void) { return selected_track; }

void nav_set_selected_track(int index) {
    if (index >= 0) {
        selected_track = index;
        ESP_LOGI(TAG, "Selected track: %d", selected_track);
    }
}

/* -----------------------------------------------------------------------
 * Folder navigation (circular)
 * ----------------------------------------------------------------------- */

int nav_next_folder(int total_folders) {
    if (total_folders <= 0) return 0;
    selected_folder = (selected_folder + 1) % total_folders;
    ESP_LOGI(TAG, "Next folder: %d (of %d)", selected_folder, total_folders);
    return selected_folder;
}

int nav_prev_folder(int total_folders) {
    if (total_folders <= 0) return 0;
    selected_folder = (selected_folder - 1 + total_folders) % total_folders;
    ESP_LOGI(TAG, "Previous folder: %d (of %d)", selected_folder, total_folders);
    return selected_folder;
}

/* -----------------------------------------------------------------------
 * Track navigation (circular)
 * ----------------------------------------------------------------------- */

int nav_next_track(int total_tracks) {
    if (total_tracks <= 0) return 0;
    selected_track = (selected_track + 1) % total_tracks;
    ESP_LOGI(TAG, "Next track: %d (of %d)", selected_track, total_tracks);
    return selected_track;
}

int nav_prev_track(int total_tracks) {
    if (total_tracks <= 0) return 0;
    selected_track = (selected_track - 1 + total_tracks) % total_tracks;
    ESP_LOGI(TAG, "Previous track: %d (of %d)", selected_track, total_tracks);
    return selected_track;
}

/* -----------------------------------------------------------------------
 * Subfolder navigation (circular)
 * ----------------------------------------------------------------------- */

int nav_get_selected_subfolder(void) { return selected_subfolder; }

void nav_set_selected_subfolder(int index) {
    if (index >= 0) {
        selected_subfolder = index;
        ESP_LOGI(TAG, "Selected subfolder: %d", selected_subfolder);
    }
}

int nav_next_subfolder(int total_subfolders) {
    if (total_subfolders <= 0) return 0;
    selected_subfolder = (selected_subfolder + 1) % total_subfolders;
    ESP_LOGI(TAG, "Next subfolder: %d (of %d)", selected_subfolder, total_subfolders);
    return selected_subfolder;
}

int nav_prev_subfolder(int total_subfolders) {
    if (total_subfolders <= 0) return 0;
    selected_subfolder = (selected_subfolder - 1 + total_subfolders) % total_subfolders;
    ESP_LOGI(TAG, "Previous subfolder: %d (of %d)", selected_subfolder, total_subfolders);
    return selected_subfolder;
}

/* -----------------------------------------------------------------------
 * State transitions
 * ----------------------------------------------------------------------- */

bool nav_go_back(void) {
    switch (current_state) {
        case NAV_STATE_FILE_VIEW:
            current_state = NAV_STATE_SUBFOLDER_VIEW;
            ESP_LOGI(TAG, "Back: FILE_VIEW -> SUBFOLDER_VIEW");
            return true;

        case NAV_STATE_SUBFOLDER_VIEW:
            current_state = NAV_STATE_FOLDER_VIEW;
            ESP_LOGI(TAG, "Back: SUBFOLDER_VIEW -> FOLDER_VIEW");
            return true;

        case NAV_STATE_FOLDER_VIEW:
            current_state = NAV_STATE_HOME;
            ESP_LOGI(TAG, "Back: FOLDER_VIEW -> HOME");
            return true;

        case NAV_STATE_HOME:
            ESP_LOGI(TAG, "Already at HOME");
            return false;

        default:
            return false;
    }
}

bool nav_go_forward(void) {
    switch (current_state) {
        case NAV_STATE_HOME:
            current_state = NAV_STATE_FOLDER_VIEW;
            ESP_LOGI(TAG, "Forward: HOME -> FOLDER_VIEW");
            return true;

        case NAV_STATE_FOLDER_VIEW:
            current_state  = NAV_STATE_SUBFOLDER_VIEW;
            selected_subfolder = 0;
            ESP_LOGI(TAG, "Forward: FOLDER_VIEW -> SUBFOLDER_VIEW");
            return true;

        case NAV_STATE_SUBFOLDER_VIEW:
            current_state  = NAV_STATE_FILE_VIEW;
            selected_track = 0;
            ESP_LOGI(TAG, "Forward: SUBFOLDER_VIEW -> FILE_VIEW");
            return true;

        case NAV_STATE_FILE_VIEW:
            ESP_LOGI(TAG, "Already at FILE_VIEW");
            return false;

        default:
            return false;
    }
}

/**
 * nav_go_to_files_direct
 *
 * Used when a top-level folder contains WAV files directly (no subfolders),
 * e.g. TAWID.  Jumps from FOLDER_VIEW straight to FILE_VIEW,
 * skipping SUBFOLDER_VIEW.
 */
bool nav_go_to_files_direct(void) {
    if (current_state != NAV_STATE_FOLDER_VIEW) {
        ESP_LOGW(TAG, "nav_go_to_files_direct: not in FOLDER_VIEW (state=%d)", current_state);
        return false;
    }
    current_state  = NAV_STATE_FILE_VIEW;
    selected_track = 0;
    ESP_LOGI(TAG, "Forward (direct): FOLDER_VIEW -> FILE_VIEW (no subfolders)");
    return true;
}

/**
 * nav_go_back_from_files_direct
 *
 * Reverse of nav_go_to_files_direct: FILE_VIEW -> FOLDER_VIEW,
 * skipping SUBFOLDER_VIEW.
 */
bool nav_go_back_from_files_direct(void) {
    if (current_state != NAV_STATE_FILE_VIEW) return false;
    current_state = NAV_STATE_FOLDER_VIEW;
    ESP_LOGI(TAG, "Back (direct): FILE_VIEW -> FOLDER_VIEW");
    return true;
}

bool nav_is_in_playback_mode(void) {
    return (current_state == NAV_STATE_FILE_VIEW);
}
