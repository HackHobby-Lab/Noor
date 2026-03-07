/**
 * announcements.c
 * Voice announcement system implementation
 *
 * SD card layout expected:
 *   /sdcard/ANNOUN~1/   (FAT32 truncation of "announcements")
 *       WELCOME.WAV  (or welcome.wave — both tried)
 *       ROTATE.WAV
 *       STORIES.WAV
 *       ADAM.WAV, DAOUD.WAV, IBRAHIM.WAV, ISMAEEL.WAV, ISSA.WAV,
 *       MOUSSA.WAV, MUHAMMAD.WAV, NUH.WAV, YAQOOB.WAV, YOUSUF.WAV
 *       INSMUH.WAV          (entering Muhammad folder)
 *       A1ANN.WAV … A7ANN.WAV
 *       D1ANN.WAV … D7ANN.WAV
 *       IB1ANN.WAV … IB7ANN.WAV
 *       IS1ANN.WAV … IS7ANN.WAV
 *       I1ANN.WAV  … I7ANN.WAV
 *       M1ANN.WAV  … M7ANN.WAV
 *       MS1ANN.WAV … MS7ANN.WAV
 *       N1ANN.WAV  … N7ANN.WAV
 *       Y1ANN.WAV  … Y7ANN.WAV
 *       YQ1ANN.WAV … YQ7ANN.WAV
 *       v1.wav … v10.wav    (volume feedback)
 *       correct.wav, b1.wav, b2.wav, b3.wav
 *       TA.wav, TT1.wav, TT2.wav, TT3.wav
 */

#include "config.h"        /* ANNOUNCE_PATH_MAX, NOTIFY_ANNOUNCE_BIT, VOLUME_MAX … */
#include "announcements.h"
#include "audio.h"
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include "esp_log.h"

static const char *TAG = "ANNOUNCE";

/*
 * FAT32 shortens long directory names.  "announcements" becomes "ANNOUN~1"
 * on most FAT32 implementations.  We store both candidates and probe at
 * runtime so the code works regardless of the host OS that formatted the card.
 */
#define ANNOUNCE_DIR_SHORTNAME  "ANNOUN~1"
#define ANNOUNCE_DIR_LONGNAME   "announcements"

/* Resolved path to the announcements directory (set once at init) */
static char announce_root[128] = {0};
static bool announce_root_found = false;

/* Pending announcement path */
static char pending_announce_path[ANNOUNCE_PATH_MAX] = {0};
static volatile bool has_pending_announce = false;

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static char *build_path(const char *dir, const char *filename) {
    if (!dir || !filename) return NULL;
    size_t dir_len  = strlen(dir);
    size_t name_len = strlen(filename);
    size_t total    = dir_len + 1 + name_len + 1;
    char  *path     = malloc(total);
    if (!path) return NULL;
    snprintf(path, total, "%s/%s", dir, filename);
    return path;
}

/**
 * Try to play a file from the announcements folder.
 * Returns true if the file exists and playback was started.
 */
static bool play_announce_file(const char *filename) {
    if (!announce_root_found || !filename) return false;

    char *path = build_path(announce_root, filename);
    if (!path) return false;

    bool exists = (access(path, F_OK) == 0);
    if (exists) {
        ESP_LOGI(TAG, "Playing announcement: %s", path);
        audio_play_file(path, NULL, NULL);
    } else {
        ESP_LOGD(TAG, "Announcement file not found: %s", path);
    }
    free(path);
    return exists;
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

void announcements_init(void) {
    memset(pending_announce_path, 0, sizeof(pending_announce_path));
    has_pending_announce  = false;
    announce_root_found   = false;

    /*
     * Probe for the announcements directory.
     * Try short FAT32 name first, then long name.
     */
    const char *candidates[] = { ANNOUNCE_DIR_SHORTNAME, ANNOUNCE_DIR_LONGNAME, NULL };
    for (int i = 0; candidates[i]; i++) {
        snprintf(announce_root, sizeof(announce_root), "/sdcard/%s", candidates[i]);
        if (access(announce_root, F_OK) == 0) {
            announce_root_found = true;
            ESP_LOGI(TAG, "Announcements directory found: %s", announce_root);
            break;
        }
    }

    if (!announce_root_found) {
        ESP_LOGW(TAG, "Announcements directory not found — tried %s and %s",
                 ANNOUNCE_DIR_SHORTNAME, ANNOUNCE_DIR_LONGNAME);
        announce_root[0] = '\0';
    }

    ESP_LOGI(TAG, "Announcements system initialized");
}

void announce_request(const char *filepath,
                      TaskHandle_t audio_task_handle,
                      volatile bool *stop_flag) {
    if (!filepath) {
        ESP_LOGW(TAG, "NULL filepath for announcement");
        return;
    }

    strncpy(pending_announce_path, filepath, sizeof(pending_announce_path) - 1);
    pending_announce_path[sizeof(pending_announce_path) - 1] = '\0';
    has_pending_announce = true;

    if (stop_flag) *stop_flag = true;

    if (audio_task_handle)
        xTaskNotify(audio_task_handle, NOTIFY_ANNOUNCE_BIT, eSetBits);

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

bool announce_check_and_play(const char *root_path,
                              const char *folder_path,
                              const char *filename,
                              TaskHandle_t audio_task_handle,
                              volatile bool *stop_flag) {
    if (!filename) return false;

    /* Try root_path first (if provided) */
    if (root_path) {
        char *p = build_path(root_path, filename);
        if (p) {
            if (access(p, F_OK) == 0) {
                announce_request(p, audio_task_handle, stop_flag);
                free(p);
                return true;
            }
            free(p);
        }
    }

    /* Try resolved announcements root */
    if (announce_root_found) {
        char *p = build_path(announce_root, filename);
        if (p) {
            if (access(p, F_OK) == 0) {
                announce_request(p, audio_task_handle, stop_flag);
                free(p);
                return true;
            }
            free(p);
        }
    }

    /* Try caller-supplied folder_path */
    if (folder_path) {
        char *p = build_path(folder_path, filename);
        if (p) {
            if (access(p, F_OK) == 0) {
                announce_request(p, audio_task_handle, stop_flag);
                free(p);
                return true;
            }
            free(p);
        }
    }

    ESP_LOGD(TAG, "Announcement file not found: %s", filename);
    return false;
}

/* -----------------------------------------------------------------------
 * Folder / subfolder name announcements
 * ----------------------------------------------------------------------- */

const char *announce_get_folder_file(const char *folder_name) {
    if (!folder_name) return NULL;

    /* Top-level folder */
    if (strcasecmp(folder_name, "STORIES") == 0)    return "STORIES.WAV";

    /* Prophet subfolders */
    if (strcasecmp(folder_name, "ADAM")     == 0)   return "ADAM.WAV";
    if (strcasecmp(folder_name, "DAOUD")    == 0)   return "DAOUD.WAV";
    if (strcasecmp(folder_name, "IBRAHIM")  == 0)   return "IBRAHIM.WAV";
    if (strcasecmp(folder_name, "ISMAEEL")  == 0 ||
        strcasecmp(folder_name, "ISMAEL")   == 0)   return "ISMAEEL.WAV";
    if (strcasecmp(folder_name, "ISSA")     == 0)   return "ISSA.WAV";
    if (strcasecmp(folder_name, "MOUSSA")   == 0)   return "MOUSSA.WAV";
    if (strcasecmp(folder_name, "MUHAMMAD") == 0)   return "MUHAMMAD.WAV";
    if (strcasecmp(folder_name, "NUH")      == 0)   return "NUH.WAV";
    if (strcasecmp(folder_name, "YAQOOB")   == 0 ||
        strcasecmp(folder_name, "YA'QOOB")  == 0)   return "YAQOOB.WAV";
    if (strcasecmp(folder_name, "YOUSUF")   == 0 ||
        strcasecmp(folder_name, "YOUSSOUF") == 0)   return "YOUSUF.WAV";

    return NULL;
}

/**
 * announce_get_story_file
 *
 * Maps a story WAV filename (from the SD card subfolder) to its
 * announcement counterpart in the announcements folder.
 *
 * Naming rules (case-insensitive input):
 *   a1.wav   -> A1ANN.WAV   (Adam)
 *   d1.wav   -> D1ANN.WAV   (Daoud)
 *   ib1.wav  -> IB1ANN.WAV  (Ibrahim)
 *   is1.wav  -> IS1ANN.WAV  (Ismaeel)
 *   i1.wav   -> I1ANN.WAV   (Issa)
 *   m1.wav   -> M1ANN.WAV   (Muhammad)
 *   ms1.wav  -> MS1ANN.WAV  (Moussa)
 *   n1.wav   -> N1ANN.WAV   (Nuh)
 *   yq1.wav  -> YQ1ANN.WAV  (Yaqoob)
 *   y1.wav   -> Y1ANN.WAV   (Yousuf)
 *   crrm1.wav / sm1.wav / wrm1.wav -> no track announcement
 */
bool announce_get_story_file(const char *filename, char *buffer, size_t buffer_size) {
    if (!filename || !buffer || buffer_size < 32) return false;

    /* Work with an uppercase copy for easy matching */
    char upper[32];
    size_t fn_len = strlen(filename);
    if (fn_len >= sizeof(upper)) return false;
    for (size_t i = 0; i <= fn_len; i++)
        upper[i] = (char)toupper((unsigned char)filename[i]);

    int story_num = 0;

    /* ---- Two-letter prefixes (check BEFORE single-letter) ---- */

    /* IB = Ibrahim */
    if (upper[0] == 'I' && upper[1] == 'B' && isdigit((unsigned char)upper[2])) {
        story_num = upper[2] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "IB%dANN.WAV", story_num);
            return true;
        }
    }

    /* IS = Ismaeel */
    if (upper[0] == 'I' && upper[1] == 'S' && isdigit((unsigned char)upper[2])) {
        story_num = upper[2] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "IS%dANN.WAV", story_num);
            return true;
        }
    }

    /* MS = Moussa */
    if (upper[0] == 'M' && upper[1] == 'S' && isdigit((unsigned char)upper[2])) {
        story_num = upper[2] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "MS%dANN.WAV", story_num);
            return true;
        }
    }

    /* YQ = Yaqoob */
    if (upper[0] == 'Y' && upper[1] == 'Q' && isdigit((unsigned char)upper[2])) {
        story_num = upper[2] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "YQ%dANN.WAV", story_num);
            return true;
        }
    }

    /* ---- Single-letter prefixes ---- */

    /* A = Adam */
    if (upper[0] == 'A' && isdigit((unsigned char)upper[1])) {
        story_num = upper[1] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "A%dANN.WAV", story_num);
            return true;
        }
    }

    /* D = Daoud */
    if (upper[0] == 'D' && isdigit((unsigned char)upper[1])) {
        story_num = upper[1] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "D%dANN.WAV", story_num);
            return true;
        }
    }

    /* I = Issa (must come AFTER IB/IS checks above) */
    if (upper[0] == 'I' && isdigit((unsigned char)upper[1])) {
        story_num = upper[1] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "I%dANN.WAV", story_num);
            return true;
        }
    }

    /* M = Muhammad (must come AFTER MS check above) */
    if (upper[0] == 'M' && isdigit((unsigned char)upper[1])) {
        story_num = upper[1] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "M%dANN.WAV", story_num);
            return true;
        }
    }

    /* N = Nuh */
    if (upper[0] == 'N' && isdigit((unsigned char)upper[1])) {
        story_num = upper[1] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "N%dANN.WAV", story_num);
            return true;
        }
    }

    /* Y = Yousuf (must come AFTER YQ check above) */
    if (upper[0] == 'Y' && isdigit((unsigned char)upper[1])) {
        story_num = upper[1] - '0';
        if (story_num >= 1 && story_num <= 9) {
            snprintf(buffer, buffer_size, "Y%dANN.WAV", story_num);
            return true;
        }
    }

    /* Special Muhammad files with no track announcement */
    if (strcasecmp(filename, "CRRM1.WAV") == 0 ||
        strcasecmp(filename, "SM1.WAV")   == 0 ||
        strcasecmp(filename, "WRM1.WAV")  == 0) {
        ESP_LOGD(TAG, "Special Muhammad file — no track announcement: %s", filename);
        return false;
    }

    ESP_LOGD(TAG, "No announcement mapping for: %s", filename);
    return false;
}

/* -----------------------------------------------------------------------
 * Boot greetings
 * ----------------------------------------------------------------------- */

void announce_play_boot_greetings(const char *root_path) {
    /* root_path is kept for compatibility but we prefer announce_root */
    (void)root_path;

    if (!announce_root_found) {
        ESP_LOGW(TAG, "Cannot play boot greetings — announce root not found");
        return;
    }

    ESP_LOGI(TAG, "Playing boot greetings from: %s", announce_root);

    /* WELCOME.WAV (the file on disk is welcome.wave — try both) */
    if (!play_announce_file("WELCOME.WAV")) {
        play_announce_file("welcome.wave");
    }

    /* ROTATE.WAV — "rotate to navigate" instruction */
    play_announce_file("ROTATE.WAV");
}

/* -----------------------------------------------------------------------
 * "Entering folder" announcements
 * ----------------------------------------------------------------------- */

const char *announce_get_entering_folder_file(const char *folder_name) {
    if (!folder_name) return NULL;

    if (strcasecmp(folder_name, "MUHAMMAD") == 0) return "INSMUH.WAV";

    /* Add more here as needed, e.g.:
     * if (strcasecmp(folder_name, "ADAM") == 0) return "INSADAM.WAV";
     */
    return NULL;
}

/* -----------------------------------------------------------------------
 * Helper: directly play an announcement by filename (no task notification)
 * ----------------------------------------------------------------------- */

bool announce_play_direct(const char *filename) {
    return play_announce_file(filename);
}

/* -----------------------------------------------------------------------
 * Helper: play track announcement before playing a story track
 * ----------------------------------------------------------------------- */

void announce_play_track(const char *wav_filename) {
    if (!wav_filename) return;

    char ann_file[32];
    if (announce_get_story_file(wav_filename, ann_file, sizeof(ann_file))) {
        play_announce_file(ann_file);
    }
}
