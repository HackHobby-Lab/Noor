/**
 * main.c
 * Noor Audio Player - Main Application (4-Level Navigation)
 *
 * Navigation Levels:
 * 1. HOME          - Initial screen
 * 2. FOLDER_VIEW   - Browse main folders  (STORIES, TAWID …)
 * 3. SUBFOLDER_VIEW- Browse prophet folders (ADAM, DAOUD …)
 * 4. FILE_VIEW     - Browse and play audio files (a1.wav, m2.wav …)
 *
 * Fixes in this version:
 * ----------------------
 *
 * FIX A — Single press to enter STORIES (HOME → SUBFOLDER_VIEW directly)
 *   Root cause: Play/Encoder press from HOME only advanced to FOLDER_VIEW
 *   and announced the folder name, requiring a second press to enter.
 *   Fix: when pressing Play or Encoder from HOME (or FOLDER_VIEW), we now
 *   scan subfolders immediately and go all the way to SUBFOLDER_VIEW in
 *   one press, announcing the first subfolder name (e.g. ADAM.WAV).
 *   If the folder has no subfolders (e.g. TAWID), we go straight to FILE_VIEW.
 *
 * FIX B — Folder-name .wav plays when entering subfolder (MUHAMMAD.WAV first,
 *   then INSMUH.WAV for Muhammad only)
 *   Root cause: NOTIFY_ENTER_SUBFOLDER_BIT handler only played the
 *   "entering" announcement (INSMUH.WAV) without first playing the folder
 *   name (MUHAMMAD.WAV).  Other folders have no "entering" file, so they
 *   got silence on entry.
 *   Fix: handler now plays folder-name .wav (e.g. MUHAMMAD.WAV) first,
 *   then plays the entering-folder file if one exists (INSMUH.WAV for
 *   Muhammad), then scans files and announces track 0.
 *
 * FIX C — Volume behavior: silent during playback, announced when idle
 *   If a story/audio is currently playing (g_playing == true):
 *     → volume is adjusted silently with no announcement and no interruption.
 *   If nothing is playing:
 *     → volume is adjusted AND vN.wav is announced via audio_task.
 *     → a second vol press while vN.wav is playing cancels it immediately
 *       via g_volume_local_stop.
 *   Result: story playback is never interrupted by volume changes.
 *
 * FIX 1 — Stale settle index (g_enc_settle_index removed)
 * FIX 2 — Entering-folder race (NOTIFY_ENTER_SUBFOLDER_BIT)
 *
 * Notification bits:
 *   bit 31  NOTIFY_ANNOUNCE_BIT        — announcement queued
 *   bit 30  NOTIFY_SETTLE_BIT          — encoder settle timer fired
 *   bit 29  NOTIFY_ENTER_SUBFOLDER_BIT — enter subfolder: play intro + transition
 *   bit 28  NOTIFY_QUIZ_BIT            — start quiz for current story
 *   bit 27  NOTIFY_QUIZ_RESULT_BIT     — play quiz result then end quiz
 *   bit 26  NOTIFY_BATTERY_ALERT_BIT   — play pending battery alert
 *   bit 25  NOTIFY_VOLUME_BIT          — play volume announcement (only when not playing)
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "sd_card.h"
#include "audio.h"
#include "buttons.h"
#include "encoder.h"
#include "navigation.h"
#include "announcements.h"
#include "headphone_detect.h"
#include "usb_msc.h"
#include "SD_CARD_OTA.h"
#include "battery.h"
#include "quiz.h"

static const char *TAG = "MAIN";

/* -------------------------------------------------------------------------
 * Global playback state
 * ---------------------------------------------------------------------- */
static volatile bool g_playing       = false;
static volatile bool g_pause         = false;
static volatile int  g_playing_track = -1;
static volatile bool g_stop_flag     = false;

static bool g_in_flat_folder = false;

static TaskHandle_t audio_task_handle = NULL;

static button_t btn_play;
static button_t btn_home;
static button_t btn_vol_up;
static button_t btn_vol_down;

/* -------------------------------------------------------------------------
 * Quiz state
 * ---------------------------------------------------------------------- */
static volatile bool          g_quiz_active          = false;
static char                   g_quiz_story_path[256] = {0};
static volatile quiz_result_t g_quiz_pending_result  = QUIZ_RESULT_WRONG;

/* -------------------------------------------------------------------------
 * Volume state
 * FIX C: volume is handled inside audio_task via NOTIFY_VOLUME_BIT,
 * but ONLY when g_playing == false (nothing currently playing).
 * g_volume_local_stop is set TRUE by every vol press to cancel any
 * currently-playing vN.wav announcement instantly.
 * ---------------------------------------------------------------------- */
static volatile bool g_volume_local_stop = false;

/* -------------------------------------------------------------------------
 * Notification bits
 * ---------------------------------------------------------------------- */
#define NOTIFY_ENTER_SUBFOLDER_BIT  (1UL << 29)
#define NOTIFY_QUIZ_BIT             (1UL << 28)
#define NOTIFY_QUIZ_RESULT_BIT      (1UL << 27)
#define NOTIFY_BATTERY_ALERT_BIT    (1UL << 26)
#define NOTIFY_VOLUME_BIT           (1UL << 25)

static char g_pending_subfolder_path[256] = {0};

/* -------------------------------------------------------------------------
 * Encoder settle timer
 * ---------------------------------------------------------------------- */
static TimerHandle_t g_enc_settle_timer = NULL;

/* -------------------------------------------------------------------------
 * Path helpers
 * ---------------------------------------------------------------------- */
static const char *get_folder_name(const char *path) {
    if (!path) return NULL;
    const char *name = strrchr(path, '/');
    return name ? name + 1 : path;
}
static const char *get_filename(const char *path) { return get_folder_name(path); }

static char *get_folder_dir(const char *filepath) {
    if (!filepath) return NULL;
    const char *last = strrchr(filepath, '/');
    if (!last) return NULL;
    size_t len = (size_t)(last - filepath);
    char *dir = malloc(len + 1);
    if (!dir) return NULL;
    memcpy(dir, filepath, len);
    dir[len] = '\0';
    return dir;
}

static bool folder_is_hidden(const char *folder_name) {
    if (!folder_name) return true;

    /* Firmware-internal directories */
    if (strncasecmp(folder_name, "ANNOUN", 6) == 0) return true;  /* announcements/ANNOUN~1 */

    /* Windows/macOS/Linux system folders created automatically when the
     * SD card is used on a PC. Without this they appear as navigable
     * folders in the device UI. */
    if (folder_name[0] == '$')                         return true;  /* $RECYCLE.BIN, $SysReset */
    if (folder_name[0] == '.')                         return true;  /* .Trash, .fseventsd, .Spotlight */
    if (strncasecmp(folder_name, "SYSTEM", 6) == 0)   return true;  /* System Volume Information */

    return false;
}

/* -------------------------------------------------------------------------
 * Helper: play an announcement blocking inline (drain pending queue).
 * Used inside audio_task where we need sequential playback.
 * ---------------------------------------------------------------------- */
static void _play_inline(const char *path) {
    if (!path || !path[0]) return;
    g_stop_flag = false;
    ESP_LOGI(TAG, "Playing: %s", path);
    audio_play_file(path, &g_stop_flag, NULL);
}

static void _announce_and_drain(const char *filename) {
    if (!filename) return;
    announce_check_and_play(NULL, NULL, filename, audio_task_handle, &g_stop_flag);
    char ap[ANNOUNCE_PATH_MAX];
    if (announce_get_pending(ap, sizeof(ap))) {
        _play_inline(ap);
        announce_clear_pending();
    }
}

/* -------------------------------------------------------------------------
 * Track playback
 * ---------------------------------------------------------------------- */
static volatile int  g_next_track         = -1;
static volatile bool g_next_track_pending = false;

static void start_track_playback(int track_index) {
    if (track_index < 0 || track_index >= sd_get_wav_count()) return;

    const char *filepath    = sd_get_wav_path(track_index);
    if (!filepath) return;

    const char *filename    = get_filename(filepath);
    char       *folder_dir  = get_folder_dir(filepath);
    const char *folder_name = folder_dir ? get_folder_name(folder_dir) : NULL;

    char ann_file[32] = {0};
    bool has_ann = false;

    if (folder_name && strcasecmp(folder_name, "TAWID") == 0) {
        /* Lesson files are T1.wav .. T20.wav (single-T prefix).
         * Announcement files are TT1.WAV .. TT20.WAV (double-T, in announcements/).
         * Extract N from filename: "T5.wav" -> atoi("5.wav") = 5.
         * Do NOT use track_index+1 — FAT32 alpha sort means T10 before T2. */
        int tt_num = (filename &&
                      (filename[0]=='T' || filename[0]=='t') &&
                      isdigit((unsigned char)filename[1]))
                     ? atoi(filename + 1) : (track_index + 1);
        snprintf(ann_file, sizeof(ann_file), "TT%d.WAV", tt_num);
        has_ann = true;
    } else {
        has_ann = announce_get_story_file(filename, ann_file, sizeof(ann_file));
    }

    if (folder_dir) free(folder_dir);

    g_stop_flag          = true;
    g_next_track         = track_index;
    g_next_track_pending = true;
    g_playing_track      = track_index;
    g_playing            = true;
    g_pause              = false;

    if (has_ann) {
        bool announced = announce_check_and_play(NULL, NULL, ann_file,
                                                  audio_task_handle, &g_stop_flag);
        if (!announced) {
            /* Announcement file missing — play track directly so it never
             * silently gets stuck (g_next_track_pending is already set). */
            ESP_LOGW(TAG, "Announcement not found for %s — playing directly", ann_file);
            g_stop_flag = false;
            xTaskNotify(audio_task_handle, (uint32_t)(track_index + 1),
                        eSetValueWithOverwrite);
        }
    } else {
        g_stop_flag = false;
        xTaskNotify(audio_task_handle, (uint32_t)(track_index + 1),
                    eSetValueWithOverwrite);
    }
}

/* -------------------------------------------------------------------------
 * FIX A helper: enter a top-level folder (scan subfolders or files directly)
 * and announce the first item.  Called from both play-button and encoder-button
 * handlers when in FOLDER_VIEW, so the logic is in one place.
 * ---------------------------------------------------------------------- */
static void enter_selected_folder(void) {
    int sel               = nav_get_selected_folder();
    const char *fpath     = sd_get_folder_path(sel);
    const char *fname     = get_folder_name(fpath);

    if (!fpath || folder_is_hidden(fname)) return;

    sd_scan_subfolders(fpath);
    int num_sub = sd_get_subfolder_count();

    if (num_sub > 0) {
        /* Has subfolders → go to SUBFOLDER_VIEW, announce first subfolder */
        nav_go_forward();           /* FOLDER_VIEW → SUBFOLDER_VIEW */
        g_in_flat_folder = false;
        nav_set_selected_subfolder(0);

        const char *sp  = sd_get_subfolder_path(0);
        const char *sn  = get_folder_name(sp);
        const char *ann = announce_get_folder_file(sn);
        if (ann)
            announce_check_and_play(NULL, NULL, ann, audio_task_handle, &g_stop_flag);

        ESP_LOGI(TAG, "Entered %s → SUBFOLDER_VIEW (%d sub)", fname, num_sub);

    } else {
        /* Flat folder (e.g. TAWID) → go straight to FILE_VIEW */
        sd_scan_wav_files(fpath);
        int num_wavs = sd_get_wav_count();
        if (num_wavs > 0) {
            nav_go_to_files_direct();
            g_in_flat_folder = true;
            nav_set_selected_track(0);

            if (strcasecmp(fname, "TAWID") == 0)
                announce_check_and_play(NULL, fpath, "TT1.WAV",
                                        audio_task_handle, &g_stop_flag);
            else
                announce_play_track(get_filename(sd_get_wav_path(0)));

            ESP_LOGI(TAG, "Flat folder %s → FILE_VIEW (%d tracks)", fname, num_wavs);
        } else {
            ESP_LOGW(TAG, "No files in %s", fpath);
        }
    }
}

/* -------------------------------------------------------------------------
 * Announce for current nav selection (FIX 1 — reads nav live)
 * ---------------------------------------------------------------------- */
static void play_announce_for_current_selection(void) {
    nav_state_t state = nav_get_state();

    if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
        int index         = nav_get_selected_folder();
        const char *fpath = sd_get_folder_path(index);
        const char *fname = get_folder_name(fpath);
        if (!fname) return;
        if (strcasecmp(fname, "TAWID") == 0)
            announce_check_and_play(NULL, fpath, "TA.wav",
                                    audio_task_handle, &g_stop_flag);
        else {
            const char *ann = announce_get_folder_file(fname);
            if (ann) announce_check_and_play(NULL, NULL, ann,
                                             audio_task_handle, &g_stop_flag);
        }

    } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
        int index         = nav_get_selected_subfolder();
        const char *spath = sd_get_subfolder_path(index);
        const char *sname = get_folder_name(spath);
        const char *ann   = announce_get_folder_file(sname);
        if (ann) announce_check_and_play(NULL, NULL, ann,
                                         audio_task_handle, &g_stop_flag);

    } else if (state == NAV_STATE_FILE_VIEW) {
        int index             = nav_get_selected_track();
        const char *filepath  = sd_get_wav_path(index);
        const char *filename  = get_filename(filepath);
        char       *folder_dir  = get_folder_dir(filepath);
        const char *folder_name = folder_dir ? get_folder_name(folder_dir) : NULL;

        if (folder_name && strcasecmp(folder_name, "TAWID") == 0) {
            char preview[32];
            /* Lesson files are T1.wav..T20.wav. Announcement files are TT1.WAV..TT20.WAV.
             * Extract N from filename: "T5.wav" -> atoi(filename+1) = 5. */
            int tt_num = (filename &&
                          (filename[0]=='T' || filename[0]=='t') &&
                          isdigit((unsigned char)filename[1]))
                         ? atoi(filename + 1) : (index + 1);
            snprintf(preview, sizeof(preview), "TT%d.WAV", tt_num);
            announce_check_and_play(NULL, folder_dir, preview,
                                    audio_task_handle, &g_stop_flag);
        } else {
            char story_file[32];
            if (announce_get_story_file(filename, story_file, sizeof(story_file)))
                announce_check_and_play(NULL, NULL, story_file,
                                        audio_task_handle, &g_stop_flag);
        }
        if (folder_dir) free(folder_dir);
    }
}

/* -------------------------------------------------------------------------
 * Quiz submit
 * ---------------------------------------------------------------------- */
static void quiz_do_submit(void) {
    if (!g_quiz_active) return;
    g_quiz_pending_result = quiz_submit();
    g_stop_flag = true;
    xTaskNotify(audio_task_handle, NOTIFY_QUIZ_RESULT_BIT, eSetBits);
}

/* -------------------------------------------------------------------------
 * FIX C: Volume button handler
 *
 * Behaviour:
 *   - Always adjusts the hardware volume register immediately (no I2S,
 *     no interruption).
 *   - If a story / audio is currently playing (g_playing == true):
 *       → volume changes silently. No announcement. Story is NOT stopped.
 *   - If nothing is playing:
 *       → sets g_volume_local_stop so any in-progress vN.wav is cancelled.
 *       → stops the encoder settle timer to prevent a stale nav announcement
 *         racing with the volume announcement in audio_task.
 *       → sends NOTIFY_VOLUME_BIT to audio_task to play vN.wav.
 * ---------------------------------------------------------------------- */
static void handle_volume_change(bool up) {
    if (up) audio_volume_up_no_announce();
    else    audio_volume_down_no_announce();

    if (g_playing) {
        /* Audio is playing — just change the hardware volume, stay silent. */
        ESP_LOGI(TAG, "Volume changed while playing — silent adjust only");
        return;
    }

    /* Nothing is playing — announce the new volume level via audio_task. */
    g_volume_local_stop = true;   /* interrupt any in-progress vN.wav */

    /* Stop settle timer — prevents stale NOTIFY_SETTLE_BIT racing vol */
    if (g_enc_settle_timer) xTimerStop(g_enc_settle_timer, 0);

    xTaskNotify(audio_task_handle, NOTIFY_VOLUME_BIT, eSetBits);
}

/* -------------------------------------------------------------------------
 * Audio task — single owner of the I2S driver
 * ---------------------------------------------------------------------- */
static void audio_task(void *arg) {
    ESP_LOGI(TAG, "Audio task started");

    while (1) {
        uint32_t nv = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &nv, portMAX_DELAY);

        /* ----------------------------------------------------------------
         * NOTIFY_BATTERY_ALERT_BIT — always first, highest priority.
         * Uses its own local stop flag inside battery_play_pending_alert().
         * ---------------------------------------------------------------- */
        if (nv & NOTIFY_BATTERY_ALERT_BIT) {
            ESP_LOGW(TAG, "Battery alert notification received");
            battery_play_pending_alert();
            nv &= ~NOTIFY_BATTERY_ALERT_BIT;
            if (!nv) continue;
        }

        /* ----------------------------------------------------------------
         * NOTIFY_VOLUME_BIT — FIX C
         *
         * Only reached when g_playing was false at the moment the volume
         * button was pressed (handle_volume_change guards this).
         *
         * Play vN.wav using g_volume_local_stop as the stop flag so that
         * a second vol press cancels the in-progress announcement instantly.
         * g_stop_flag is NOT touched here — it is not needed since no story
         * is playing.
         *
         * Path resolution: probe known announcement directories in priority
         * order to avoid calling announce_check_and_play() which would queue
         * NOTIFY_ANNOUNCE_BIT and cause a re-entrant dispatch issue.
         * ---------------------------------------------------------------- */
        if (nv & NOTIFY_VOLUME_BIT) {
            int vol  = audio_get_volume();
            int step = (vol + 9) / 10;
            if (step < 1)  step = 1;
            if (step > 10) step = 10;

            char vol_file[16];
            snprintf(vol_file, sizeof(vol_file), "v%d.wav", step);
            ESP_LOGI(TAG, "Volume announcement: %s (vol=%d%%)", vol_file, vol);

            /* Probe announcement directories in priority order */
            static const char *s_ann_dirs[] = {
                "/sdcard/ANNOUN~1",
                "/sdcard/announcements",
                "/sdcard",
                NULL
            };
            char vpath[128] = {0};
            for (int d = 0; s_ann_dirs[d] != NULL; d++) {
                char candidate[128];
                snprintf(candidate, sizeof(candidate), "%s/%s",
                         s_ann_dirs[d], vol_file);
                FILE *tf = fopen(candidate, "rb");
                if (tf) {
                    fclose(tf);
                    strncpy(vpath, candidate, sizeof(vpath) - 1);
                    break;
                }
            }

            if (vpath[0]) {
                g_volume_local_stop = false;   /* arm — next vol press sets true */
                ESP_LOGI(TAG, "Playing volume wav: %s", vpath);
                audio_play_file(vpath, &g_volume_local_stop, NULL);
            } else {
                ESP_LOGW(TAG, "Volume wav not found: %s", vol_file);
            }

            /* g_stop_flag is intentionally NOT cleared here — no story was
             * playing so there is nothing to resume or unblock. */

            nv &= ~NOTIFY_VOLUME_BIT;
            if (!nv) continue;
        }

        /* ---- FIX 1: Encoder settle ---- */
        if (nv & NOTIFY_SETTLE_BIT) {
            if (g_quiz_active) {
                quiz_announce_option(quiz_get_selected(),
                                     audio_task_handle, &g_stop_flag);
            } else {
                g_stop_flag = false;
                play_announce_for_current_selection();
            }
            nv &= ~NOTIFY_SETTLE_BIT;
            if (!nv) continue;
        }

        /* ---- FIX 2 + FIX B: Enter subfolder ---- */
        if (nv & NOTIFY_ENTER_SUBFOLDER_BIT) {
            const char *spath = g_pending_subfolder_path;
            const char *sname = get_folder_name(spath);

            if (!spath || spath[0] == '\0') {
                nv &= ~NOTIFY_ENTER_SUBFOLDER_BIT;
                goto handle_remaining;
            }

            /*
             * FIX B Step 1: Play the folder-name announcement first.
             * e.g. MUHAMMAD.WAV, ADAM.WAV, etc.
             * This is what the user hears as "entering confirmation".
             */
            g_stop_flag = false;
            const char *folder_ann = announce_get_folder_file(sname);
            if (folder_ann) {
                _announce_and_drain(folder_ann);
            }

            /* Check for abort between step 1 and step 2 */
            if (g_stop_flag) {
                nv &= ~NOTIFY_ENTER_SUBFOLDER_BIT;
                goto handle_remaining;
            }

            /*
             * FIX B Step 2: Play the "entering" announcement if one exists.
             * Currently only MUHAMMAD has this: INSMUH.WAV.
             */
            const char *entering = announce_get_entering_folder_file(sname);
            if (entering) {
                announce_check_and_play(NULL, spath, entering,
                                        audio_task_handle, &g_stop_flag);
                char ap[ANNOUNCE_PATH_MAX];
                if (announce_get_pending(ap, sizeof(ap))) {
                    g_stop_flag = false;
                    audio_play_file(ap, &g_stop_flag, NULL);
                    announce_clear_pending();
                }
            }

            if (g_stop_flag) {
                nv &= ~NOTIFY_ENTER_SUBFOLDER_BIT;
                goto handle_remaining;
            }

            /* Step 3: Scan files and transition to FILE_VIEW */
            sd_scan_wav_files(spath);
            int num_wavs = sd_get_wav_count();
            if (num_wavs > 0) {
                nav_go_forward();   /* SUBFOLDER_VIEW → FILE_VIEW */
                g_in_flat_folder = false;
                nav_set_selected_track(0);
                ESP_LOGI(TAG, "Entered subfolder %s (files=%d)", sname, num_wavs);

                /* Step 4: Announce track 0 */
                const char *fp0 = sd_get_wav_path(0);
                const char *fn0 = get_filename(fp0);
                char story[32];
                if (announce_get_story_file(fn0, story, sizeof(story))) {
                    _announce_and_drain(story);
                }
            } else {
                ESP_LOGW(TAG, "No WAV files in %s", spath);
            }

            nv &= ~NOTIFY_ENTER_SUBFOLDER_BIT;
            if (!nv) continue;
        }

        /* ---- Quiz start ---- */
        if (nv & NOTIFY_QUIZ_BIT) {
            ESP_LOGI(TAG, "Quiz starting: %s", g_quiz_story_path);
            if (quiz_begin(g_quiz_story_path)) {
                nav_set_state(NAV_STATE_QUIZ);
                g_quiz_active = true;
                g_playing     = false;

                quiz_play_question(&g_stop_flag);
                if (!g_stop_flag) quiz_play_options_intro(&g_stop_flag);

                /*
                 * Abort check: use g_quiz_active, NOT g_stop_flag.
                 *
                 * g_stop_flag=true here can mean TWO things:
                 *   a) Home button was pressed → home handler sets g_quiz_active=false
                 *      AND calls quiz_end() already.  We just clean up state.
                 *   b) User rotated encoder during intro → quiz_announce_option()
                 *      calls announce_request() which sets g_stop_flag=true, but
                 *      g_quiz_active is still true — quiz should continue.
                 *
                 * So: if g_quiz_active is false → genuine abort (home pressed).
                 *     if g_quiz_active is true  → user interacted early, continue.
                 */
                if (!g_quiz_active) {
                    /* Home was pressed during intro — already cleaned up by home handler */
                    ESP_LOGI(TAG, "Quiz aborted during intro (home pressed)");
                    nav_set_state(NAV_STATE_FILE_VIEW);
                } else {
                    /* Completed normally OR user interacted early — announce current option */
                    g_stop_flag = false;
                    ESP_LOGI(TAG, "Quiz ready — awaiting user selection");
                    quiz_announce_option(quiz_get_selected(), audio_task_handle, &g_stop_flag);
                }
            }
            nv &= ~NOTIFY_QUIZ_BIT;
            if (!nv) continue;
        }

        /* ---- Quiz result ---- */
        if (nv & NOTIFY_QUIZ_RESULT_BIT) {
            quiz_result_t result = g_quiz_pending_result;
            ESP_LOGI(TAG, "Quiz result: %s (quiz %d/%d)",
                     result == QUIZ_RESULT_CORRECT ? "CORRECT" : "WRONG",
                     quiz_get_current_num(), quiz_get_total_count());

            /*
             * Clear g_stop_flag before playing the result audio.
             * quiz_do_submit() sets g_stop_flag=true to stop any in-progress
             * option announcement — that is NOT an abort signal.
             * We will re-check g_stop_flag after quiz_play_result to detect
             * a genuine home-button press DURING result playback.
             */
            g_stop_flag = false;
            quiz_play_result(result, &g_stop_flag);

            if (g_stop_flag && !g_quiz_active) {
                /* Home button was pressed during result playback — already
                 * cleaned up by the home handler. Just tidy remaining state. */
                ESP_LOGI(TAG, "Quiz aborted during result (home pressed)");
                g_playing       = false;
                g_playing_track = -1;

            } else if (result == QUIZ_RESULT_WRONG) {
                /* Wrong answer — re-randomise and replay the SAME quiz */
                ESP_LOGI(TAG, "Wrong — retrying quiz %d", quiz_get_current_num());
                quiz_randomise();
                g_stop_flag = false;
                quiz_play_question(&g_stop_flag);
                if (!g_stop_flag) quiz_play_options_intro(&g_stop_flag);
                if (!g_quiz_active) {
                    /* Home pressed during retry */
                    g_playing       = false;
                    g_playing_track = -1;
                } else {
                    g_stop_flag = false;
                    quiz_announce_option(quiz_get_selected(), audio_task_handle, &g_stop_flag);
                }

            } else if (!quiz_is_last()) {
                /* Correct but more quizzes remain — advance to next quiz */
                quiz_advance();
                ESP_LOGI(TAG, "Correct — advancing to quiz %d", quiz_get_current_num());
                g_stop_flag = false;
                quiz_play_question(&g_stop_flag);
                if (!g_stop_flag) quiz_play_options_intro(&g_stop_flag);
                if (!g_quiz_active) {
                    /* Home pressed during next quiz intro */
                    g_playing       = false;
                    g_playing_track = -1;
                } else {
                    g_stop_flag = false;
                    quiz_announce_option(quiz_get_selected(), audio_task_handle, &g_stop_flag);
                }

            } else {
                /* Correct on the final quiz — all done, back to FILE_VIEW */
                ESP_LOGI(TAG, "All %d quizzes completed! Returning to file view.",
                         quiz_get_total_count());
                g_quiz_active   = false;
                quiz_end();
                nav_set_state(NAV_STATE_FILE_VIEW);
                g_playing       = false;
                g_playing_track = -1;
            }

            nv &= ~NOTIFY_QUIZ_RESULT_BIT;
            if (!nv) continue;
        }

        handle_remaining:

        /* ---- Announcement ---- */
        if (nv & NOTIFY_ANNOUNCE_BIT) {
            char ap[ANNOUNCE_PATH_MAX];
            if (announce_get_pending(ap, sizeof(ap))) {
                g_stop_flag = false;
                ESP_LOGI(TAG, "Playing announcement: %s", ap);
                audio_play_file(ap, &g_stop_flag, NULL);
                announce_clear_pending();
            }

            /*
             * Always play the pending track if one is queued, even if
             * g_stop_flag is true.  g_stop_flag=true here means the
             * announcement was cut short by the user pressing play early —
             * NOT that playback should be cancelled.  We reset g_stop_flag
             * before starting the track so it plays from the beginning.
             */
            if (g_next_track_pending) {
                int ti               = g_next_track;
                g_next_track_pending = false;
                g_next_track         = -1;
                g_stop_flag          = false;   /* clear — announcement stop ≠ abort */

                if (ti >= 0 && ti < sd_get_wav_count()) {
                    const char *fp = sd_get_wav_path(ti);
                    if (fp) {
                        g_stop_flag     = false;
                        g_playing_track = ti;
                        g_playing       = true;
                        g_pause         = false;
                        audio_play_file(fp, &g_stop_flag, &g_pause);

                        if (!g_stop_flag && quiz_exists_for_story(fp)) {
                            strncpy(g_quiz_story_path, fp,
                                    sizeof(g_quiz_story_path) - 1);
                            g_playing       = false;
                            g_playing_track = -1;
                            xTaskNotify(audio_task_handle,
                                        NOTIFY_QUIZ_BIT, eSetBits);
                            continue;
                        }
                    }
                }
                g_playing       = false;
                g_playing_track = -1;
            } else {
                g_next_track_pending = false;
                g_next_track         = -1;
            }
            continue;
        }

        /* ---- Direct track index notify ---- */
        if (nv != 0) {
            int ti               = (int)nv - 1;
            g_next_track_pending = false;
            g_next_track         = -1;

            if (ti < 0 || ti >= sd_get_wav_count()) {
                g_playing = false; g_playing_track = -1; continue;
            }

            const char *fp = sd_get_wav_path(ti);
            if (!fp) { g_playing = false; g_playing_track = -1; continue; }

            g_stop_flag     = false;
            g_playing_track = ti;
            g_playing       = true;
            g_pause         = false;

            audio_play_file(fp, &g_stop_flag, &g_pause);

            if (!g_stop_flag && quiz_exists_for_story(fp)) {
                strncpy(g_quiz_story_path, fp, sizeof(g_quiz_story_path) - 1);
                g_playing       = false;
                g_playing_track = -1;
                xTaskNotify(audio_task_handle, NOTIFY_QUIZ_BIT, eSetBits);
                continue;
            }

            g_playing       = false;
            g_playing_track = -1;
        }
    }
}

/* -------------------------------------------------------------------------
 * Settle timer callback (FIX 1)
 * ---------------------------------------------------------------------- */
static void enc_settle_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    g_stop_flag = true;
    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(audio_task_handle, NOTIFY_SETTLE_BIT, eSetBits, &hp);
    portYIELD_FROM_ISR(hp);
}

/* -------------------------------------------------------------------------
 * Request subfolder entry (FIX 2)
 * ---------------------------------------------------------------------- */
static void request_enter_subfolder(const char *spath)
{
    if (!spath || !spath[0]) return;
    strncpy(g_pending_subfolder_path, spath,
            sizeof(g_pending_subfolder_path) - 1);
    g_pending_subfolder_path[sizeof(g_pending_subfolder_path) - 1] = '\0';
    g_stop_flag = true;
    xTaskNotify(audio_task_handle, NOTIFY_ENTER_SUBFOLDER_BIT, eSetBits);
}

/* -------------------------------------------------------------------------
 * Encoder callback
 * ---------------------------------------------------------------------- */
static void encoder_callback(encoder_event_t event) {
    nav_state_t state = nav_get_state();

    if (event.type == ENC_EVENT_ROTATE) {

        if (g_quiz_active) {
            quiz_next_option();
            quiz_announce_option(quiz_get_selected(),
                                 audio_task_handle, &g_stop_flag);
            return;
        }

        g_stop_flag = true;

        if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
            int total = sd_get_folder_count();
            if (total <= 0) return;
            do {
                if (event.direction == ENC_DIR_CW) nav_next_folder(total);
                else                               nav_prev_folder(total);
            } while (folder_is_hidden(
                get_folder_name(sd_get_folder_path(nav_get_selected_folder()))));
            xTimerReset(g_enc_settle_timer, 0);

        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            int total = sd_get_subfolder_count();
            if (total <= 0) return;
            if (event.direction == ENC_DIR_CW) nav_next_subfolder(total);
            else                               nav_prev_subfolder(total);
            xTimerReset(g_enc_settle_timer, 0);

        } else if (state == NAV_STATE_FILE_VIEW) {
            int total = sd_get_wav_count();
            if (total <= 0) return;
            if (event.direction == ENC_DIR_CW) nav_next_track(total);
            else                               nav_prev_track(total);
            xTimerReset(g_enc_settle_timer, 0);
        }

    } else if (event.type == ENC_EVENT_BUTTON) {

        if (g_quiz_active) { quiz_do_submit(); return; }

        /* Cancel pending settle timer to prevent NOTIFY_SETTLE_BIT
         * from racing with the navigation action below. */
        if (g_enc_settle_timer) xTimerStop(g_enc_settle_timer, 0);

        if (state == NAV_STATE_HOME) {
            /*
             * FIX A: HOME → enter folder in one press.
             * Skip the intermediate FOLDER_VIEW announcement step.
             */
            if (sd_get_folder_count() <= 0) return;
            nav_go_forward();   /* HOME → FOLDER_VIEW */
            int total = sd_get_folder_count();
            while (folder_is_hidden(
                get_folder_name(sd_get_folder_path(nav_get_selected_folder()))))
                nav_next_folder(total);
            enter_selected_folder();

        } else if (state == NAV_STATE_FOLDER_VIEW) {
            /* FIX A: FOLDER_VIEW → enter immediately */
            enter_selected_folder();

        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            const char *spath = sd_get_subfolder_path(nav_get_selected_subfolder());
            if (spath) request_enter_subfolder(spath);

        } else if (state == NAV_STATE_FILE_VIEW) {
            int track = nav_get_selected_track();
            if (track < 0 || track >= sd_get_wav_count()) {
                nav_set_selected_track(0); track = 0;
            }
            if (!g_playing)                    start_track_playback(track);
            else if (g_playing_track != track) {
                g_stop_flag = true;
                start_track_playback(track);
            }
        }
    }
}

/* -------------------------------------------------------------------------
 * app_main
 * ---------------------------------------------------------------------- */
void app_main(void) {
    ESP_LOGI(TAG, "=== Noor Audio Player (4-Level Navigation + Quiz) ===");

    audio_init();
    buttons_init();
    nav_init();
    quiz_init();

    button_init_struct(&btn_play,     BTN_PLAY_PIN);
    button_init_struct(&btn_home,     BTN_HOME_PIN);
    button_init_struct(&btn_vol_up,   BTN_VOLUP_PIN);
    button_init_struct(&btn_vol_down, BTN_VOLDN_PIN);

    if (!usb_msc_init())
        ESP_LOGW(TAG, "USB MSC init failed (non-fatal)");
    else
        ESP_LOGI(TAG, "USB MSC enabled");

    bool sd_ok = sd_card_init();
    if (!sd_ok) {
        ESP_LOGE(TAG, "SD card init FAILED");
    } else {
        usb_msc_set_sd_card(sd_get_card_handle());
        check_sd_ota_update();
    }

    announcements_init();

    if (sd_ok) {
        announce_play_boot_greetings(NULL);
        sd_scan_folders("/sdcard");

        int num_folders    = sd_get_folder_count();
        int default_folder = -1;
        for (int i = 0; i < num_folders; i++) {
            const char *fn = get_folder_name(sd_get_folder_path(i));
            if (strcasecmp(fn, "STORIES") == 0) { default_folder = i; break; }
        }
        if (default_folder < 0) {
            for (int i = 0; i < num_folders; i++) {
                if (!folder_is_hidden(get_folder_name(sd_get_folder_path(i))))
                    { default_folder = i; break; }
            }
        }
        if (default_folder >= 0) nav_set_selected_folder(default_folder);
    }

    if (headphone_detect_init()) headphone_detect_start_task();

    g_enc_settle_timer = xTimerCreate(
        "enc_settle", pdMS_TO_TICKS(ENC_SETTLE_MS),
        pdFALSE, NULL, enc_settle_timer_cb);
    configASSERT(g_enc_settle_timer != NULL);

    if (encoder_init()) encoder_start_task(encoder_callback);

    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL,
                            PRIORITY_AUDIO, &audio_task_handle, tskNO_AFFINITY);

    battery_init();
    battery_set_audio_task(audio_task_handle, NOTIFY_BATTERY_ALERT_BIT);
    battery_set_playing_flag(&g_playing);

    ESP_LOGI(TAG, "Init complete. Main loop running.");

    /* =========================================================================
     * Main loop
     * ====================================================================== */
    while (1) {
        nav_state_t state = nav_get_state();

        /* ---- PLAY / PAUSE ---- */
        if (button_is_pressed(&btn_play)) {

            if (g_quiz_active) { quiz_do_submit(); goto next_button; }

            /* Cancel any pending settle timer so its NOTIFY_SETTLE_BIT
             * announcement does not race with our navigation action below. */
            if (g_enc_settle_timer) xTimerStop(g_enc_settle_timer, 0);

            if (state != NAV_STATE_FILE_VIEW || !g_playing) g_stop_flag = true;

            if (state == NAV_STATE_HOME) {
                /* FIX A: one press from HOME enters the folder */
                if (sd_get_folder_count() <= 0) goto next_button;
                nav_go_forward();   /* HOME → FOLDER_VIEW */
                int total = sd_get_folder_count();
                while (folder_is_hidden(
                    get_folder_name(sd_get_folder_path(nav_get_selected_folder()))))
                    nav_next_folder(total);
                enter_selected_folder();

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                /* FIX A: already in FOLDER_VIEW → enter immediately */
                enter_selected_folder();

            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                const char *spath = sd_get_subfolder_path(nav_get_selected_subfolder());
                if (!spath) goto next_button;
                request_enter_subfolder(spath);

            } else if (state == NAV_STATE_FILE_VIEW) {
                int track = nav_get_selected_track();
                if (track < 0 || track >= sd_get_wav_count()) {
                    nav_set_selected_track(0); track = 0;
                }
                if (!g_playing)                    start_track_playback(track);
                else if (g_playing_track == track) g_pause = !g_pause;
                else { g_stop_flag = true; start_track_playback(track); }
            }
        }
        next_button:;

        /* ---- HOME / BACK ---- */
        if (button_is_pressed(&btn_home)) {

            if (g_quiz_active) {
                g_stop_flag = true; g_quiz_active = false;
                quiz_end(); nav_set_state(NAV_STATE_FILE_VIEW);
                g_playing = false; g_playing_track = -1;
                goto home_done;
            }

            g_stop_flag = true;

            if (state == NAV_STATE_FILE_VIEW) {
                g_playing = false; g_pause = false;
                sd_free_wavs();
                if (g_in_flat_folder) {
                    nav_go_back_from_files_direct(); g_in_flat_folder = false;
                    int sel        = nav_get_selected_folder();
                    const char *fn = get_folder_name(sd_get_folder_path(sel));
                    if (strcasecmp(fn, "TAWID") == 0)
                        announce_check_and_play(NULL, sd_get_folder_path(sel),
                                                "TA.wav",
                                                audio_task_handle, &g_stop_flag);
                    else {
                        const char *ann = announce_get_folder_file(fn);
                        if (ann) announce_check_and_play(NULL, NULL, ann,
                                                         audio_task_handle,
                                                         &g_stop_flag);
                    }
                } else {
                    nav_go_back();  /* FILE_VIEW → SUBFOLDER_VIEW */
                    int sel        = nav_get_selected_subfolder();
                    const char *sn = get_folder_name(sd_get_subfolder_path(sel));
                    const char *ann = announce_get_folder_file(sn);
                    if (ann) announce_check_and_play(NULL, NULL, ann,
                                                     audio_task_handle,
                                                     &g_stop_flag);
                }

            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                sd_free_subfolders(); nav_go_back();    /* → FOLDER_VIEW */
                int sel        = nav_get_selected_folder();
                const char *fn = get_folder_name(sd_get_folder_path(sel));
                const char *ann = announce_get_folder_file(fn);
                if (ann) announce_check_and_play(NULL, NULL, ann,
                                                 audio_task_handle, &g_stop_flag);

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                nav_go_back();      /* → HOME */
                announce_play_direct("ROTATE.WAV");
            }
        }
        home_done:;

        /* ----------------------------------------------------------------
         * VOLUME buttons — FIX C
         *
         * If g_playing == true  → hardware volume adjusted silently.
         *                          Story/audio is NOT interrupted at all.
         * If g_playing == false → hardware volume adjusted AND vN.wav
         *                          announcement is played via audio_task.
         *                          A second press cancels in-progress vN.wav.
         * ---------------------------------------------------------------- */
        if (button_is_pressed(&btn_vol_up))   handle_volume_change(true);
        if (button_is_pressed(&btn_vol_down))  handle_volume_change(false);

        /* ---- Periodic USB MSC log ---- */
        static uint32_t last_usb_log = 0;
        uint32_t now = xTaskGetTickCount();
        if ((now - last_usb_log) > pdMS_TO_TICKS(5000)) {
            last_usb_log = now;
            if (usb_msc_is_connected())
                ESP_LOGI(TAG, "USB: %s", usb_msc_get_status());
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}