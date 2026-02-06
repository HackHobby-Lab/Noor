/**
 * announcements.c
 * Voice announcement system implementation
 */

#include "announcements.h"
#include "audio.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <unistd.h>
#include "esp_log.h"

static const char *TAG = "ANNOUNCE";

/* Pending announcement path */
static char pending_announce_path[ANNOUNCE_PATH_MAX] = {0};
static volatile bool has_pending_announce = false;

void announcements_init(void) {
    memset(pending_announce_path, 0, sizeof(pending_announce_path));
    has_pending_announce = false;
    ESP_LOGI(TAG, "Announcements system initialized");
}

void announce_request(const char *filepath,
                     TaskHandle_t audio_task_handle,
                     volatile bool *stop_flag) {
    if (!filepath) {
        ESP_LOGW(TAG, "NULL filepath for announcement");
        return;
    }
    
    // Copy path to pending buffer
    strncpy(pending_announce_path, filepath, sizeof(pending_announce_path) - 1);
    pending_announce_path[sizeof(pending_announce_path) - 1] = '\0';
    has_pending_announce = true;
    
    // Set stop flag to interrupt current playback
    if (stop_flag) {
        *stop_flag = true;
    }
    
    // Notify audio task
    if (audio_task_handle) {
        xTaskNotify(audio_task_handle, NOTIFY_ANNOUNCE_BIT, eSetBits);
    }
    
    ESP_LOGI(TAG, "Announcement requested: %s", filepath);
}

bool announce_get_pending(char *buffer, size_t buffer_size) {
    if (!has_pending_announce) {
        return false;
    }
    
    if (buffer && buffer_size > 0) {
        strncpy(buffer, pending_announce_path, buffer_size - 1);
        buffer[buffer_size - 1] = '\0';
    }
    
    return true;
}

void announce_clear_pending(void) {
    has_pending_announce = false;
    memset(pending_announce_path, 0, sizeof(pending_announce_path));
}

/* Helper: Build full path */
static char* build_path(const char *dir, const char *filename) {
    if (!dir || !filename) return NULL;
    
    size_t dir_len = strlen(dir);
    size_t name_len = strlen(filename);
    size_t total = dir_len + 1 + name_len + 1;
    
    char *path = malloc(total);
    if (!path) return NULL;
    
    snprintf(path, total, "%s/%s", dir, filename);
    return path;
}

bool announce_check_and_play(const char *root_path,
                             const char *folder_path,
                             const char *filename,
                             TaskHandle_t audio_task_handle,
                             volatile bool *stop_flag) {
    if (!root_path || !filename) return false;
    
    // Try root directory first
    char *root_file = build_path(root_path, filename);
    if (root_file) {
        if (access(root_file, F_OK) == 0) {
            // Found in root
            announce_request(root_file, audio_task_handle, stop_flag);
            free(root_file);
            return true;
        }
        free(root_file);
    }
    
    // Try folder if provided
    if (folder_path) {
        char *folder_file = build_path(folder_path, filename);
        if (folder_file) {
            if (access(folder_file, F_OK) == 0) {
                // Found in folder
                announce_request(folder_file, audio_task_handle, stop_flag);
                free(folder_file);
                return true;
            }
            free(folder_file);
        }
    }
    
    ESP_LOGD(TAG, "Announcement file not found: %s", filename);
    return false;
}

const char* announce_get_folder_file(const char *folder_name) {
    if (!folder_name) return NULL;
    
    // Main folder: STORIES
    if (strcasecmp(folder_name, "STORIES") == 0) {
        return "STORIES.WAV";
    }
    
    // Prophet subfolders (uppercase names)
    if (strcasecmp(folder_name, "ADAM") == 0) {
        return "ADAM.WAV";
    }
    if (strcasecmp(folder_name, "DAOUD") == 0) {
        return "DAOUD.WAV";
    }
    if (strcasecmp(folder_name, "IBRAHIM") == 0) {
        return "IBRAHIM.WAV";
    }
    if (strcasecmp(folder_name, "ISMAEL") == 0 ||
        strcasecmp(folder_name, "ISMAEEL") == 0) {
        return "ISMAEEL.WAV";
    }
    if (strcasecmp(folder_name, "ISSA") == 0) {
        return "ISSA.WAV";
    }
    if (strcasecmp(folder_name, "MOUSSA") == 0) {
        return "MOUSSA.WAV";
    }
    if (strcasecmp(folder_name, "MUHAMMAD") == 0) {
        return "MUHAMMAD.WAV";
    }
    if (strcasecmp(folder_name, "NUH") == 0) {
        return "NUH.WAV";
    }
    if (strcasecmp(folder_name, "YAQOOB") == 0 ||
        strcasecmp(folder_name, "YA'QOOB") == 0) {
        return "YAQOOB.WAV";
    }
    if (strcasecmp(folder_name, "YOUSUF") == 0 ||
        strcasecmp(folder_name, "YOUSSOUF") == 0) {
        return "YOUSUF.WAV";
    }
    
    return NULL;
}

bool announce_get_story_file(const char *filename, char *buffer, size_t buffer_size) {
    if (!filename || !buffer || buffer_size < 32) return false;
    
    // Muhammad stories: m1.wav -> M1ANN.WAV, m2.wav -> M2ANN.WAV, etc.
    if ((filename[0] == 'm' || filename[0] == 'M') && 
        isdigit((unsigned char)filename[1])) {
        int story_num = filename[1] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "M%dANN.WAV", story_num);
            return true;
        }
    }
    
    // Old pattern for compatibility: S1xyz.wav -> story1.wav
    if ((filename[0] == 'S' || filename[0] == 's') && 
        isdigit((unsigned char)filename[1])) {
        int story_num = filename[1] - '0';
        snprintf(buffer, buffer_size, "story%d.wav", story_num);
        return true;
    }
    
    return false;
}

void announce_play_boot_greetings(const char *root_path) {
    if (!root_path) return;
    
    ESP_LOGI(TAG, "Playing boot greetings...");
    
    // Play WELCOME.WAV
    char *welcome_path = build_path(root_path, "WELCOME.WAV");
    if (welcome_path) {
        if (access(welcome_path, F_OK) == 0) {
            ESP_LOGI(TAG, "Playing WELCOME.WAV");
            audio_play_file(welcome_path, NULL, NULL);
        }
        free(welcome_path);
    }
    
    // Play ROTATE.WAV (home audio)
    char *home_path = build_path(root_path, "ROTATE.WAV");
    if (home_path) {
        if (access(home_path, F_OK) == 0) {
            ESP_LOGI(TAG, "Playing ROTATE.WAV (home audio)");
            audio_play_file(home_path, NULL, NULL);
        }
        free(home_path);
    }
}

const char* announce_get_entering_folder_file(const char *folder_name) {
    if (!folder_name) return NULL;
    
    // Entering Muhammad's stories folder
    if (strcasecmp(folder_name, "MUHAMMAD") == 0) {
        return "INSMUH.WAV";
    }
    
    // Add more "entering folder" announcements here as needed
    // For example:
    // if (strcasecmp(folder_name, "ADAM") == 0) {
    //     return "INSADAM.WAV";
    // }
    
    return NULL;
}