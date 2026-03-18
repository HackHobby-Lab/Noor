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
 * FIX B — Folder-name .wav plays when entering subfolder
 * FIX C — Volume behavior: silent during playback, announced when idle
 * FIX 1 — Stale settle index (g_enc_settle_index removed)
 * FIX 2 — Entering-folder race (NOTIFY_ENTER_SUBFOLDER_BIT)
 * FIX D — Quiz option announce: scroll cancels current, plays new immediately
 *
 * FIX E — Lock encoder input during question + options intro playback
 * -----------------------------------------------------------------------
 * Problem: encoder_callback was accepting rotate events at any time once
 * g_quiz_active was true — including while quiz_play_question() and
 * quiz_play_options_intro() were still playing inside audio_task.
 *
 * From the logs:
 *   I (164364) QUIZ: Selected option: B  ← scroll DURING question audio
 *   I (167494) QUIZ: Selected option: A  ← scroll again during question
 *   I (167694) AUDIO: End of file reached ← question finishes naturally
 *   I (168104) QUIZ: Options intro starts ← intro replays from scratch
 *
 * The encoder queued announces during the question, those announces were
 * in the pending slot when the intro started, causing confusion about
 * which option was selected.
 *
 * Fix: add g_quiz_input_locked (volatile bool).
 *   - Set TRUE by audio_task immediately before quiz_play_question().
 *   - Set FALSE by audio_task immediately after quiz_play_options_intro()
 *     completes (or is interrupted by home).
 *   - encoder_callback checks this flag: if true, rotate events during
 *     quiz are silently ignored. Button (submit) is also ignored while
 *     locked since there is nothing to submit yet.
 *   - Home button is NOT blocked — it must always be able to abort.
 *
 * This applies to all three call sites:
 *   - Initial quiz start (NOTIFY_QUIZ_BIT)
 *   - Wrong answer retry (NOTIFY_QUIZ_RESULT_BIT → WRONG branch)
 *   - Correct advance (NOTIFY_QUIZ_RESULT_BIT → advance branch)
 *
 * Notification bits:
 *   bit 31  NOTIFY_ANNOUNCE_BIT        — announcement queued
 *   bit 30  NOTIFY_SETTLE_BIT          — encoder settle timer fired
 *   bit 29  NOTIFY_ENTER_SUBFOLDER_BIT — enter subfolder
 *   bit 28  NOTIFY_QUIZ_BIT            — start quiz for current story
 *   bit 27  NOTIFY_QUIZ_RESULT_BIT     — play quiz result then end quiz
 *   bit 26  NOTIFY_BATTERY_ALERT_BIT   — play pending battery alert
 *   bit 25  NOTIFY_VOLUME_BIT          — play volume announcement
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
#include <ctype.h>

static const char *TAG = "MAIN";

/* -------------------------------------------------------------------------
 * Global playback state
 * ---------------------------------------------------------------------- */
static volatile bool g_playing       = false;
static volatile bool g_pause         = false;
static volatile int  g_playing_track = -1;
static volatile bool g_stop_flag     = false;

/* -------------------------------------------------------------------------
 * Quiz flags:
 *
 *   g_quiz_stop_current  — stop currently-playing option audio.
 *                          Set by encoder_callback, cleared by audio_task.
 *
 *   g_quiz_announce_stop — stop intro/question audio.
 *                          Set by encoder (home only), submit, home button;
 *                          cleared by audio_task before question/intro.
 *
 *   g_quiz_input_locked  — TRUE while question + intro are playing.
 *                          Encoder rotate and submit are ignored when set.
 *                          Set by audio_task before quiz_play_question().
 *                          Cleared by audio_task after quiz_play_options_intro()
 *                          returns (whether completed or home-aborted).
 *                          Home button ignores this flag (always works).
 * ---------------------------------------------------------------------- */
static volatile bool g_quiz_stop_current  = false;
static volatile bool g_quiz_announce_stop = false;
static volatile bool g_quiz_input_locked  = false;  /* FIX E */

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
    if (strncasecmp(folder_name, "ANNOUN", 6) == 0) return true;
    if (folder_name[0] == '$')                       return true;
    if (folder_name[0] == '.')                       return true;
    if (strncasecmp(folder_name, "SYSTEM", 6) == 0) return true;
    return false;
}

/* -------------------------------------------------------------------------
 * Helper: play an announcement blocking inline (drain pending queue).
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
 * FIX A helper: enter a top-level folder
 * ---------------------------------------------------------------------- */
static void enter_selected_folder(void) {
    int sel               = nav_get_selected_folder();
    const char *fpath     = sd_get_folder_path(sel);
    const char *fname     = get_folder_name(fpath);

    if (!fpath || folder_is_hidden(fname)) return;

    sd_scan_subfolders(fpath);
    int num_sub = sd_get_subfolder_count();

    if (num_sub > 0) {
        nav_go_forward();
        g_in_flat_folder = false;
        nav_set_selected_subfolder(0);

        const char *sp  = sd_get_subfolder_path(0);
        const char *sn  = get_folder_name(sp);
        const char *ann = announce_get_folder_file(sn);
        if (ann)
            announce_check_and_play(NULL, NULL, ann, audio_task_handle, &g_stop_flag);

        ESP_LOGI(TAG, "Entered %s → SUBFOLDER_VIEW (%d sub)", fname, num_sub);

    } else {
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
 * Announce for current nav selection (FIX 1)
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
    if (g_quiz_input_locked) return;   /* FIX E: ignore submit during intro */
    g_quiz_pending_result = quiz_submit();
    g_quiz_stop_current   = true;
    g_quiz_announce_stop  = true;
    g_stop_flag           = true;
    xTaskNotify(audio_task_handle, NOTIFY_QUIZ_RESULT_BIT, eSetBits);
}

/* -------------------------------------------------------------------------
 * FIX C: Volume button handler
 * ---------------------------------------------------------------------- */
static void handle_volume_change(bool up) {
    if (up) audio_volume_up_no_announce();
    else    audio_volume_down_no_announce();

    if (g_playing) {
        ESP_LOGI(TAG, "Volume changed while playing — silent adjust only");
        return;
    }

    g_volume_local_stop = true;
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
         * NOTIFY_BATTERY_ALERT_BIT — highest priority.
         * ---------------------------------------------------------------- */
        if (nv & NOTIFY_BATTERY_ALERT_BIT) {
            ESP_LOGW(TAG, "Battery alert notification received");

            if (g_quiz_active && (nv & NOTIFY_ANNOUNCE_BIT)) {
                char ap[ANNOUNCE_PATH_MAX];
                if (announce_get_pending(ap, sizeof(ap))) {
                    announce_clear_pending();
                    g_quiz_stop_current = false;
                    ESP_LOGI(TAG, "Playing announcement: %s", ap);
                    audio_play_file(ap, &g_quiz_stop_current, NULL);
                }
                nv &= ~NOTIFY_ANNOUNCE_BIT;
            }

            battery_play_pending_alert();
            nv &= ~NOTIFY_BATTERY_ALERT_BIT;
            if (!nv) continue;
        }

        // /* ----------------------------------------------------------------
        //  * NOTIFY_BATTERY_ALERT_BIT — always first, highest priority.
        //  * ---------------------------------------------------------------- */
        // if (nv & NOTIFY_BATTERY_ALERT_BIT) {
        //     ESP_LOGW(TAG, "Battery alert notification received");
        //     battery_play_pending_alert();
        //     nv &= ~NOTIFY_BATTERY_ALERT_BIT;
        //     if (!nv) continue;
        // }

        /* ----------------------------------------------------------------
         * NOTIFY_VOLUME_BIT — FIX C
         * ---------------------------------------------------------------- */
        if (nv & NOTIFY_VOLUME_BIT) {
            int vol  = audio_get_volume();
            int step = (vol + 9) / 10;
            if (step < 1)  step = 1;
            if (step > 10) step = 10;

            char vol_file[16];
            snprintf(vol_file, sizeof(vol_file), "v%d.wav", step);
            ESP_LOGI(TAG, "Volume announcement: %s (vol=%d%%)", vol_file, vol);

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
                g_volume_local_stop = false;
                ESP_LOGI(TAG, "Playing volume wav: %s", vpath);
                audio_play_file(vpath, &g_volume_local_stop, NULL);
            } else {
                ESP_LOGW(TAG, "Volume wav not found: %s", vol_file);
            }

            nv &= ~NOTIFY_VOLUME_BIT;
            if (!nv) continue;
        }

        /* ---- FIX 1: Encoder settle ---- */
        if (nv & NOTIFY_SETTLE_BIT) {
            if (g_quiz_active) {
                g_quiz_stop_current = false;
                quiz_announce_option(quiz_get_selected(),
                                     audio_task_handle, &g_quiz_stop_current);
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

            g_stop_flag = false;
            const char *folder_ann = announce_get_folder_file(sname);
            if (folder_ann) {
                _announce_and_drain(folder_ann);
            }

            if (g_stop_flag) {
                nv &= ~NOTIFY_ENTER_SUBFOLDER_BIT;
                goto handle_remaining;
            }

            const char *entering = announce_get_entering_folder_file(sname);
            if (entering) {
                announce_check_and_play(NULL, spath, entering,
                                        audio_task_handle, &g_stop_flag);
                char ap[ANNOUNCE_PATH_MAX];
                if (announce_get_pending(ap, sizeof(ap))) {
                    announce_clear_pending();
                    g_stop_flag = false;
                    audio_play_file(ap, &g_stop_flag, NULL);
                }
            }

            if (g_stop_flag) {
                nv &= ~NOTIFY_ENTER_SUBFOLDER_BIT;
                goto handle_remaining;
            }

            sd_scan_wav_files(spath);
            int num_wavs = sd_get_wav_count();
            if (num_wavs > 0) {
                nav_go_forward();
                g_in_flat_folder = false;
                nav_set_selected_track(0);
                ESP_LOGI(TAG, "Entered subfolder %s (files=%d)", sname, num_wavs);

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

                /*
                 * FIX E: Lock encoder input for the entire question+intro
                 * sequence. Unlocked after intro completes or home aborts.
                 */
                g_quiz_input_locked = true;
                ESP_LOGI(TAG, "Quiz input locked (question+intro playing)");

                g_stop_flag = false;
                quiz_play_question(&g_stop_flag);

                g_quiz_announce_stop = false;
                if (!g_stop_flag) quiz_play_options_intro(&g_quiz_announce_stop);

                /* Unlock regardless of how we got here */
                g_quiz_input_locked = false;
                ESP_LOGI(TAG, "Quiz input unlocked — awaiting selection");

                /* Also drain any stale announces that arrived while locked */
                announce_clear_pending();

                if (!g_quiz_active) {
                    ESP_LOGI(TAG, "Quiz aborted during intro (home pressed)");
                    nav_set_state(NAV_STATE_FILE_VIEW);
                } else {
                    g_quiz_stop_current = false;
                    quiz_announce_option(quiz_get_selected(),
                                         audio_task_handle, &g_quiz_stop_current);
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

            g_stop_flag = false;
            quiz_play_result(result, &g_stop_flag);

            if (g_stop_flag && !g_quiz_active) {
                ESP_LOGI(TAG, "Quiz aborted during result (home pressed)");
                g_quiz_input_locked = false;
                g_playing           = false;
                g_playing_track     = -1;

            } else if (result == QUIZ_RESULT_WRONG) {
                ESP_LOGI(TAG, "Wrong — retrying quiz %d", quiz_get_current_num());
                quiz_randomise();

                /* FIX E: lock during retry question+intro */
                g_quiz_input_locked = true;
                ESP_LOGI(TAG, "Quiz input locked (retry question+intro)");

                g_stop_flag = false;
                quiz_play_question(&g_stop_flag);
                g_quiz_announce_stop = false;
                if (!g_stop_flag) quiz_play_options_intro(&g_quiz_announce_stop);

                g_quiz_input_locked = false;
                ESP_LOGI(TAG, "Quiz input unlocked — awaiting selection");
                announce_clear_pending();

                if (!g_quiz_active) {
                    g_playing       = false;
                    g_playing_track = -1;
                } else {
                    g_quiz_stop_current = false;
                    quiz_announce_option(quiz_get_selected(),
                                         audio_task_handle, &g_quiz_stop_current);
                }

            } else if (!quiz_is_last()) {
                quiz_advance();
                ESP_LOGI(TAG, "Correct — advancing to quiz %d", quiz_get_current_num());

                /* FIX E: lock during advance question+intro */
                g_quiz_input_locked = true;
                ESP_LOGI(TAG, "Quiz input locked (advance question+intro)");

                g_stop_flag = false;
                quiz_play_question(&g_stop_flag);
                g_quiz_announce_stop = false;
                if (!g_stop_flag) quiz_play_options_intro(&g_quiz_announce_stop);

                g_quiz_input_locked = false;
                ESP_LOGI(TAG, "Quiz input unlocked — awaiting selection");
                announce_clear_pending();

                if (!g_quiz_active) {
                    g_playing       = false;
                    g_playing_track = -1;
                } else {
                    g_quiz_stop_current = false;
                    quiz_announce_option(quiz_get_selected(),
                                         audio_task_handle, &g_quiz_stop_current);
                }

            } else {
                ESP_LOGI(TAG, "All %d quizzes completed! Returning to file view.",
                         quiz_get_total_count());
                g_quiz_input_locked = false;
                g_quiz_active       = false;
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
            if (g_quiz_active) {
                /*
                 * Quiz option announce loop.
                 * announce_clear_pending() called BEFORE audio_play_file so
                 * the slot is free for the next encoder event while playing.
                 * Loop after return to drain any announce that arrived during
                 * playback without needing another notification round-trip.
                 */
                char ap[ANNOUNCE_PATH_MAX];
                while (announce_get_pending(ap, sizeof(ap))) {
                    announce_clear_pending();
                    g_quiz_stop_current = false;
                    ESP_LOGI(TAG, "Playing announcement: %s", ap);
                    audio_play_file(ap, &g_quiz_stop_current, NULL);
                }

            } else {
                char ap[ANNOUNCE_PATH_MAX];
                if (announce_get_pending(ap, sizeof(ap))) {
                    announce_clear_pending();
                    g_stop_flag = false;
                    ESP_LOGI(TAG, "Playing announcement: %s", ap);
                    audio_play_file(ap, &g_stop_flag, NULL);
                }
            }

            if (g_next_track_pending) {
                int ti               = g_next_track;
                g_next_track_pending = false;
                g_next_track         = -1;
                g_stop_flag          = false;

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
            /*
             * FIX E: ignore rotate while question + intro are playing.
             * The user must wait until the intro finishes before selecting.
             */
            if (g_quiz_input_locked) return;

            g_quiz_stop_current  = true;
            g_quiz_announce_stop = true;
            quiz_next_option();
            quiz_announce_option(quiz_get_selected(),
                                 audio_task_handle, &g_quiz_stop_current);
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

        if (g_quiz_active) {
            quiz_do_submit();   /* quiz_do_submit() checks g_quiz_input_locked */
            return;
        }

        if (g_enc_settle_timer) xTimerStop(g_enc_settle_timer, 0);

        if (state == NAV_STATE_HOME) {
            if (sd_get_folder_count() <= 0) return;
            nav_go_forward();
            int total = sd_get_folder_count();
            while (folder_is_hidden(
                get_folder_name(sd_get_folder_path(nav_get_selected_folder()))))
                nav_next_folder(total);
            enter_selected_folder();

        } else if (state == NAV_STATE_FOLDER_VIEW) {
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
    battery_set_quiz_flag(&g_quiz_active);

    ESP_LOGI(TAG, "Init complete. Main loop running.");

    while (1) {
        nav_state_t state = nav_get_state();

        /* ---- PLAY / PAUSE ---- */
        if (button_is_pressed(&btn_play)) {

            if (g_quiz_active) { quiz_do_submit(); goto next_button; }

            if (g_enc_settle_timer) xTimerStop(g_enc_settle_timer, 0);

            if (state != NAV_STATE_FILE_VIEW || !g_playing) g_stop_flag = true;

            if (state == NAV_STATE_HOME) {
                if (sd_get_folder_count() <= 0) goto next_button;
                nav_go_forward();
                int total = sd_get_folder_count();
                while (folder_is_hidden(
                    get_folder_name(sd_get_folder_path(nav_get_selected_folder()))))
                    nav_next_folder(total);
                enter_selected_folder();

            } else if (state == NAV_STATE_FOLDER_VIEW) {
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
                /* Home always works — it ignores g_quiz_input_locked */
                g_quiz_input_locked  = false;
                g_quiz_stop_current  = true;
                g_quiz_announce_stop = true;
                g_stop_flag          = true;
                g_quiz_active        = false;
                quiz_end();
                nav_set_state(NAV_STATE_FILE_VIEW);
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
                    nav_go_back();
                    int sel        = nav_get_selected_subfolder();
                    const char *sn = get_folder_name(sd_get_subfolder_path(sel));
                    const char *ann = announce_get_folder_file(sn);
                    if (ann) announce_check_and_play(NULL, NULL, ann,
                                                     audio_task_handle,
                                                     &g_stop_flag);
                }

            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                sd_free_subfolders(); nav_go_back();
                int sel        = nav_get_selected_folder();
                const char *fn = get_folder_name(sd_get_folder_path(sel));
                const char *ann = announce_get_folder_file(fn);
                if (ann) announce_check_and_play(NULL, NULL, ann,
                                                 audio_task_handle, &g_stop_flag);

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                nav_go_back();
                announce_play_direct("ROTATE.WAV");
            }
        }
        home_done:;

        if (button_is_pressed(&btn_vol_up))   handle_volume_change(true);
        if (button_is_pressed(&btn_vol_down))  handle_volume_change(false);

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
