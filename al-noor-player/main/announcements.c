/**
 * announcements.c
 * Voice announcement system implementation
 *
 * Audio file mapping:
 *   HOME.WAV        -> user is at home position
 *   WELCOME.WAV     -> boot greeting
 *   STORIES.WAV     -> STORIES folder highlighted
 *   TA.WAV          -> TAWID folder highlighted (lives INSIDE TAWID folder)
 *   v1.wav..v10.wav -> volume 10%..100% (all in /sdcard root)
 *   ADAM.WAV etc.   -> prophet folder names (in /sdcard root)
 *
 * Track announcements (generic: filename uppercase + ANN.wav, all in /sdcard root):
 *   a1.wav -> A1ANN.wav  (ADAM)      d1.wav -> D1ANN.wav  (DAOUD)
 *   ib1.wav -> IB1ANN.wav (IBRAHIM)  is1.wav -> IS1ANN.wav (ISMAEEL)
 *   i1.wav -> I1ANN.wav  (ISSA)      ms1.wav -> MS1ANN.wav (MOUSSA)
 *   m1.wav -> M1ANN.wav  (MUHAMMAD)  n1.wav -> N1ANN.wav  (NUH)
 *   yq1.wav -> YQ1ANN.wav (YAQOOB)   y1.wav -> Y1ANN.wav  (YOUSUF)
 */

#include "announcements.h"
#include "audio.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <unistd.h>
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "ANNOUNCE";

static char          pending_announce_path[ANNOUNCE_PATH_MAX] = {0};
static volatile bool has_pending_announce = false;

/* ---------------------------------------------------------- */
/*  Init                                                       */
/* ---------------------------------------------------------- */
void announcements_init(void) {
    memset(pending_announce_path, 0, sizeof(pending_announce_path));
    has_pending_announce = false;
    ESP_LOGI(TAG, "Announcements system initialized");
}

/* ---------------------------------------------------------- */
/*  Internal helper: build full path                          */
/* ---------------------------------------------------------- */
static char *build_path(const char *dir, const char *filename) {
    if (!dir || !filename) return NULL;
    size_t len = strlen(dir) + 1 + strlen(filename) + 1;
    char  *p   = malloc(len);
    if (!p) return NULL;
    snprintf(p, len, "%s/%s", dir, filename);
    return p;
}

/* ---------------------------------------------------------- */
/*  Request & pending management                              */
/* ---------------------------------------------------------- */
void announce_request(const char *filepath,
                      TaskHandle_t audio_task_handle,
                      volatile bool *stop_flag) {
    if (!filepath) return;

    strncpy(pending_announce_path, filepath, sizeof(pending_announce_path) - 1);
    pending_announce_path[sizeof(pending_announce_path) - 1] = '\0';
    has_pending_announce = true;

    if (stop_flag)          *stop_flag = true;
    if (audio_task_handle)  xTaskNotify(audio_task_handle, NOTIFY_ANNOUNCE_BIT, eSetBits);

    ESP_LOGI(TAG, "Announcement requested: %s", filepath);
}

bool announce_get_pending(char *buffer, size_t buffer_size) {
    if (!has_pending_announce) return false;
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

/* ---------------------------------------------------------- */
/*  Check file exists and play it                             */
/* ---------------------------------------------------------- */
bool announce_check_and_play(const char *root_path,
                              const char *folder_path,
                              const char *filename,
                              TaskHandle_t audio_task_handle,
                              volatile bool *stop_flag) {
    if (!root_path || !filename) return false;

    /* Try /sdcard/announcements first */
    char *ann_file = build_path("/sdcard/announcements", filename);
    if (ann_file) {
        if (access(ann_file, F_OK) == 0) {
            announce_request(ann_file, audio_task_handle, stop_flag);
            free(ann_file);
            return true;
        }
        free(ann_file);
    }

    /* Try root */
    char *root_file = build_path(root_path, filename);
    if (root_file) {
        if (access(root_file, F_OK) == 0) {
            announce_request(root_file, audio_task_handle, stop_flag);
            free(root_file);
            return true;
        }
        free(root_file);
    }

    /* Try folder if given */
    if (folder_path) {
        char *folder_file = build_path(folder_path, filename);
        if (folder_file) {
            if (access(folder_file, F_OK) == 0) {
                announce_request(folder_file, audio_task_handle, stop_flag);
                free(folder_file);
                return true;
            }
            free(folder_file);
        }
    }

    ESP_LOGD(TAG, "Announce file not found: %s", filename);
    return false;
}

/* ---------------------------------------------------------- */
/*  Folder name -> WAV file mapping                           */
/* ---------------------------------------------------------- */
const char *announce_get_folder_file(const char *folder_name) {
    if (!folder_name) return NULL;

    /* Main folders */
    /* "StoriesofProphets" folder -> STORIES.wav */
    if (strcasecmp(folder_name, "StoriesofProphets") == 0 ||
        strcasecmp(folder_name, "STORIES")           == 0 ||
        strcasecmp(folder_name, "StoriesOfProphets") == 0) {
        return "STORIES.wav";
    }

    /* TAWID -> handled separately in main.c
     * TA.wav lives in /sdcard root (not inside folder) */
    if (strcasecmp(folder_name, "TAWID") == 0) return NULL;

    /* Prophet subfolders - match your exact folder names */
    if (strcasecmp(folder_name, "ADAM")    == 0) return "ADAM.wav";
    if (strcasecmp(folder_name, "DOUD")    == 0 ||
        strcasecmp(folder_name, "DAOUD")   == 0) return "DAOUD.wav";
    if (strcasecmp(folder_name, "IBRAHIM") == 0) return "IBRAHIM.wav";
    if (strcasecmp(folder_name, "ISMAEEL") == 0 ||
        strcasecmp(folder_name, "ISMAEL")  == 0) return "ISMAEEL.wav";
    if (strcasecmp(folder_name, "ISSA")    == 0) return "ISSA.wav";
    if (strcasecmp(folder_name, "MOUSSA")  == 0) return "MOUSSA.wav";
    if (strcasecmp(folder_name, "MUHAMMAD")== 0) return "MUHAMMAD.wav";
    if (strcasecmp(folder_name, "NUH")     == 0) return "NUH.wav";
    if (strcasecmp(folder_name, "YAQOOB")  == 0) return "YAQOOB.wav";
    if (strcasecmp(folder_name, "YOSUF")   == 0 ||
        strcasecmp(folder_name, "YOUSUF")  == 0) return "YOUSUF.wav";

    return NULL;
}

/* ---------------------------------------------------------- */
/*  Story file -> announcement file                           */
/*  Generic: strip .wav, uppercase, append ANN.wav            */
/*  e.g. a3.wav -> A3ANN.wav, ib5.wav -> IB5ANN.wav          */
/*       m1.wav -> M1ANN.wav (backward compatible)            */
/* ---------------------------------------------------------- */
bool announce_get_story_file(const char *filename,
                              char *buffer, size_t buffer_size) {
    if (!filename || !buffer || buffer_size < 16) return false;

    /* Find the .wav extension */
    const char *dot = strrchr(filename, '.');
    if (!dot) return false;

    size_t base_len = dot - filename;
    if (base_len == 0 || base_len + 7 >= buffer_size) return false;  /* "ANN.wav" = 7 chars */

    /* Copy base name as uppercase, then append ANN.wav */
    for (size_t i = 0; i < base_len; i++)
        buffer[i] = toupper((unsigned char)filename[i]);
    memcpy(buffer + base_len, "ANN.wav", 7);
    buffer[base_len + 7] = '\0';

    return true;
}

/* ---------------------------------------------------------- */
/*  "Entering folder" announcement                            */
/* ---------------------------------------------------------- */
const char *announce_get_entering_folder_file(const char *folder_name) {
    if (!folder_name) return NULL;
    /* When entering MUHAMMAD subfolder -> play INSMUH.wav from root */
    if (strcasecmp(folder_name, "MUHAMMAD") == 0) return "INSMUH.wav";
    return NULL;
}

/* ---------------------------------------------------------- */
/*  Boot greetings                                            */
/* ---------------------------------------------------------- */
void announce_play_boot_greetings(const char *root_path,
                                   volatile bool *stop_flag) {
    if (!root_path) return;
    const char *ann_dir = "/sdcard/announcements";

    ESP_LOGI(TAG, "Playing boot greetings...");

    /* 1. welcome.wav - boot greeting */
    char *welcome = build_path(ann_dir, "welcome.wav");
    if (!welcome || access(welcome, F_OK) != 0) {
        free(welcome);
        welcome = build_path(root_path, "welcome.wav");
    }
    if (welcome) {
        if (access(welcome, F_OK) == 0) {
            ESP_LOGI(TAG, "Playing %s", welcome);
            audio_play_file(welcome, stop_flag, NULL);
        } else {
            ESP_LOGW(TAG, "welcome.wav not found");
        }
        free(welcome);
    }

    /* 2. ROTATE.wav - home position indicator */
    char *home = build_path(ann_dir, "ROTATE.wav");
    if (!home || access(home, F_OK) != 0) {
        free(home);
        home = build_path(root_path, "ROTATE.wav");
    }
    if (home) {
        if (access(home, F_OK) == 0) {
            ESP_LOGI(TAG, "Playing %s", home);
            audio_play_file(home, stop_flag, NULL);
        } else {
            ESP_LOGW(TAG, "ROTATE.wav not found");
        }
        free(home);
    }
}

/* ---------------------------------------------------------- */
/*  Volume -> v1.wav .. v10.wav                               */
/* ---------------------------------------------------------- */
bool announce_get_volume_file(int volume_percent,
                               char *buffer, size_t buffer_size) {
    if (!buffer || buffer_size < 16) return false;

    if (volume_percent < 0)   volume_percent = 0;
    if (volume_percent > 100) volume_percent = 100;

    /* 0%      -> v1.wav (minimum)
     * 1-10%   -> v1.wav
     * 11-20%  -> v2.wav  ...  91-100% -> v10.wav */
    int index = (volume_percent == 0) ? 1 : (volume_percent + 9) / 10;
    if (index < 1)  index = 1;
    if (index > 10) index = 10;

    snprintf(buffer, buffer_size, "v%d.wav", index);
    ESP_LOGI(TAG, "Volume %d%% -> %s", volume_percent, buffer);
    return true;
}
