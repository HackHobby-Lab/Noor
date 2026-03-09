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
 * Changes vs previous version:
 * ----------------------------
 *
 * FIX 1 — Stale settle index announces wrong track/folder
 *   Root cause: g_enc_settle_index and g_enc_settle_state were captured
 *   when the settle timer was ARMED (at encoder rotation time).  If the
 *   encoder moved again before the 250ms window elapsed, those values were
 *   stale by the time the timer fired.
 *
 *   Fix: removed g_enc_settle_index and g_enc_settle_state entirely.
 *   The settle timer callback just sends NOTIFY_SETTLE_BIT.  audio_task
 *   reads nav_get_state() / nav_get_selected_*() LIVE at the moment it
 *   handles the notification.  Since the timer only fires after 250ms of
 *   no rotation, the nav state at fire-time is always the final position.
 *
 * FIX 2 — Entering-folder announcement (INSMUH.WAV etc.) skipped
 *   Root cause: in the SUBFOLDER_VIEW play handler, the "entering folder"
 *   announce was queued to audio_task via announce_check_and_play(), but
 *   the main loop immediately continued and:
 *     - sd_scan_wav_files() ran while INSMUH was still playing
 *     - nav_go_forward() entered FILE_VIEW
 *     - announce_play_track(track0) was queued, overwriting INSMUH
 *   All three steps happened within the same 10ms main-loop tick.
 *
 *   Fix: the entering-folder announcement + FILE_VIEW transition is now
 *   handled entirely inside audio_task via a new NOTIFY_ENTER_SUBFOLDER_BIT
 *   notification.  The main loop just sets the subfolder path and wakes
 *   audio_task; audio_task plays the intro then scans files, transitions
 *   state, and announces track 0 — all sequentially with no races.
 *
 *   The same pattern (announce_check_and_play then immediately calling
 *   sd_scan + nav_go_forward) existed in both the encoder button handler
 *   and the Play button handler for SUBFOLDER_VIEW — both fixed.
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Our modules */
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

static const char *TAG = "MAIN";

/* -------------------------------------------------------------------------
 * Global playback state
 * ---------------------------------------------------------------------- */
static volatile bool g_playing       = false;
static volatile bool g_pause         = false;
static volatile int  g_playing_track = -1;
static volatile bool g_stop_flag     = false;

static bool g_in_flat_folder = false;

/* Task handle for audio task */
static TaskHandle_t audio_task_handle = NULL;

/* Button structures */
static button_t btn_play;
static button_t btn_home;
static button_t btn_vol_up;
static button_t btn_vol_down;

/* -------------------------------------------------------------------------
 * Notification bits
 *
 * NOTIFY_ANNOUNCE_BIT      (bit 31) — announcement pending in announce queue
 * NOTIFY_SETTLE_BIT        (bit 30) — encoder settle timer fired
 * NOTIFY_ENTER_SUBFOLDER   (bit 29) — enter subfolder: play intro + transition
 *
 * Bits 0-28: track index + 1 for direct track playback (value != 0)
 * ---------------------------------------------------------------------- */
#define NOTIFY_ENTER_SUBFOLDER_BIT  (1UL << 29)

/* -------------------------------------------------------------------------
 * Pending subfolder path for NOTIFY_ENTER_SUBFOLDER_BIT.
 * Written by main loop / encoder callback, read by audio_task.
 * Protected by the fact that main loop and audio_task never write it
 * concurrently — we set g_stop_flag first, which makes audio_task finish
 * its current operation before reading this.
 * ---------------------------------------------------------------------- */
static char g_pending_subfolder_path[256] = {0};

/* -------------------------------------------------------------------------
 * Encoder settle timer
 *
 * FIX 1: g_enc_settle_index / g_enc_settle_state REMOVED.
 * The timer just sends NOTIFY_SETTLE_BIT; audio_task reads nav state live.
 * ---------------------------------------------------------------------- */
static TimerHandle_t g_enc_settle_timer = NULL;

/* -------------------------------------------------------------------------
 * Helper: last path component
 * ---------------------------------------------------------------------- */
static const char *get_folder_name(const char *path) {
    if (!path) return NULL;
    const char *name = strrchr(path, '/');
    return name ? name + 1 : path;
}

static const char *get_filename(const char *path) {
    return get_folder_name(path);
}

static char *get_folder_dir(const char *filepath) {
    if (!filepath) return NULL;
    const char *last_slash = strrchr(filepath, '/');
    if (!last_slash) return NULL;
    size_t len = (size_t)(last_slash - filepath);
    char *folder = malloc(len + 1);
    if (!folder) return NULL;
    memcpy(folder, filepath, len);
    folder[len] = '\0';
    return folder;
}

static bool folder_is_hidden(const char *folder_name) {
    if (!folder_name) return true;
    if (strncasecmp(folder_name, "SYSTEM", 6) == 0) return true;
    if (strncasecmp(folder_name, "ANNOUN", 6) == 0) return true;
    return false;
}

/* -------------------------------------------------------------------------
 * Next track queued after announcement
 * ---------------------------------------------------------------------- */
static volatile int  g_next_track         = -1;
static volatile bool g_next_track_pending = false;

static void start_track_playback(int track_index) {
    if (track_index < 0 || track_index >= sd_get_wav_count()) {
        ESP_LOGW(TAG, "start_track_playback: invalid index %d", track_index);
        return;
    }

    const char *filepath  = sd_get_wav_path(track_index);
    if (!filepath) return;

    const char *filename   = get_filename(filepath);
    char       *folder_dir = get_folder_dir(filepath);
    const char *folder_name = folder_dir ? get_folder_name(folder_dir) : NULL;

    char ann_file[32] = {0};
    bool has_ann = false;

    if (folder_name && strcasecmp(folder_name, "TAWID") == 0) {
        snprintf(ann_file, sizeof(ann_file), "TT%d.WAV", track_index + 1);
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
        announce_check_and_play(NULL, NULL, ann_file, audio_task_handle, &g_stop_flag);
    } else {
        g_stop_flag = false;
        xTaskNotify(audio_task_handle, (uint32_t)(track_index + 1),
                    eSetValueWithOverwrite);
    }

    ESP_LOGI(TAG, "Playing track %d: %s", track_index, filepath);
}

/* -------------------------------------------------------------------------
 * play_announce_for_index — called from audio_task, reads nav state live.
 *
 * FIX 1: no index/state parameters from a captured snapshot.
 * We read the navigation state fresh here, which is always correct because
 * the settle timer only fires after 250ms of no encoder movement.
 * ---------------------------------------------------------------------- */
static void play_announce_for_current_selection(void)
{
    nav_state_t state = nav_get_state();

    if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
        int index = nav_get_selected_folder();
        const char *fpath = sd_get_folder_path(index);
        const char *fname = get_folder_name(fpath);
        if (!fname) return;
        if (strcasecmp(fname, "TAWID") == 0) {
            announce_check_and_play(NULL, fpath, "TA.wav",
                                    audio_task_handle, &g_stop_flag);
        } else {
            const char *ann = announce_get_folder_file(fname);
            if (ann)
                announce_check_and_play(NULL, NULL, ann,
                                        audio_task_handle, &g_stop_flag);
        }

    } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
        int index = nav_get_selected_subfolder();
        const char *spath = sd_get_subfolder_path(index);
        const char *sname = get_folder_name(spath);
        const char *ann   = announce_get_folder_file(sname);
        if (ann)
            announce_check_and_play(NULL, NULL, ann,
                                    audio_task_handle, &g_stop_flag);

    } else if (state == NAV_STATE_FILE_VIEW) {
        int index = nav_get_selected_track();
        const char *filepath    = sd_get_wav_path(index);
        const char *filename    = get_filename(filepath);
        char       *folder_dir  = get_folder_dir(filepath);
        const char *folder_name = folder_dir ? get_folder_name(folder_dir) : NULL;

        if (folder_name && strcasecmp(folder_name, "TAWID") == 0) {
            char preview[32];
            snprintf(preview, sizeof(preview), "TT%d.WAV", index + 1);
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
 * Audio task — single owner of the I2S driver
 *
 * FIX 1: NOTIFY_SETTLE_BIT handler calls play_announce_for_current_selection()
 *        with no captured state — reads live nav position.
 *
 * FIX 2: NOTIFY_ENTER_SUBFOLDER_BIT handler plays intro, scans files,
 *        transitions to FILE_VIEW, and announces track 0 — all sequentially.
 * ---------------------------------------------------------------------- */
static void audio_task(void *arg) {
    ESP_LOGI(TAG, "Audio task started");

    while (1) {
        uint32_t notification_value = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, portMAX_DELAY);

        /* ---- FIX 1: Encoder settle — read nav state live ---- */
        if (notification_value & NOTIFY_SETTLE_BIT) {
            g_stop_flag = false;
            play_announce_for_current_selection();
            continue;
        }

        /* ---- FIX 2: Enter subfolder — play intro then transition ---- */
        if (notification_value & NOTIFY_ENTER_SUBFOLDER_BIT) {
            /* g_pending_subfolder_path was set by caller before sending this bit */
            const char *spath = g_pending_subfolder_path;
            const char *sname = get_folder_name(spath);

            if (!spath || spath[0] == '\0') {
                ESP_LOGW(TAG, "ENTER_SUBFOLDER: empty path, ignoring");
                continue;
            }

            /* Step 1: Play the "entering folder" announcement (e.g. INSMUH.WAV).
             * This runs to completion before anything else happens. */
            g_stop_flag = false;
            const char *entering = announce_get_entering_folder_file(sname);
            if (entering) {
                ESP_LOGI(TAG, "Playing announcement: %s/%s", spath, entering);
                char ann_path[512];
                snprintf(ann_path, sizeof(ann_path), "%s/%s", spath, entering);
                /* Use the announce root path instead of spath if it exists there */
                announce_check_and_play(NULL, spath, entering,
                                        audio_task_handle, &g_stop_flag);
                /* Re-enter wait: announce_check_and_play sends NOTIFY_ANNOUNCE_BIT
                 * back to this same task, so we need to process it now inline.
                 * Actually announce_check_and_play queues the path and sends the
                 * bit — we must process the ANNOUNCE notification before continuing.
                 * The simplest approach: play it directly here. */
            }

            /* Step 2: Process the pending NOTIFY_ANNOUNCE_BIT that
             * announce_check_and_play just queued — drain it now. */
            {
                char announce_path[ANNOUNCE_PATH_MAX];
                if (announce_get_pending(announce_path, sizeof(announce_path))) {
                    g_stop_flag = false;
                    ESP_LOGI(TAG, "Playing announcement: %s", announce_path);
                    audio_play_file(announce_path, &g_stop_flag, NULL);
                    announce_clear_pending();
                }
            }

            /* Step 3: Check for stop — if home was pressed during intro, abort */
            if (g_stop_flag) {
                ESP_LOGI(TAG, "Subfolder entry aborted during intro");
                continue;
            }

            /* Step 4: Scan files and transition to FILE_VIEW */
            sd_scan_wav_files(spath);
            int num_wavs = sd_get_wav_count();

            if (num_wavs > 0) {
                nav_go_forward(); /* SUBFOLDER_VIEW -> FILE_VIEW */
                g_in_flat_folder = false;
                nav_set_selected_track(0);

                ESP_LOGI(TAG, "Entered subfolder (files=%d)", num_wavs);

                /* Step 5: Announce track 0 */
                const char *fp0      = sd_get_wav_path(0);
                const char *fn0      = get_filename(fp0);
                char        story[32];
                if (announce_get_story_file(fn0, story, sizeof(story))) {
                    ESP_LOGI(TAG, "Playing announcement: track 0 = %s", story);
                    /* Queue via pending mechanism */
                    announce_check_and_play(NULL, NULL, story,
                                            audio_task_handle, &g_stop_flag);
                    /* Drain the queued announcement inline */
                    char ann2[ANNOUNCE_PATH_MAX];
                    if (announce_get_pending(ann2, sizeof(ann2))) {
                        g_stop_flag = false;
                        audio_play_file(ann2, &g_stop_flag, NULL);
                        announce_clear_pending();
                    }
                }
            } else {
                ESP_LOGW(TAG, "No files in %s", spath);
            }
            continue;
        }

        /* ---- Announcement request ---- */
        if (notification_value & NOTIFY_ANNOUNCE_BIT) {
            char announce_path[ANNOUNCE_PATH_MAX];
            if (announce_get_pending(announce_path, sizeof(announce_path))) {
                g_stop_flag = false;
                ESP_LOGI(TAG, "Playing announcement: %s", announce_path);
                audio_play_file(announce_path, &g_stop_flag, NULL);
                announce_clear_pending();
            }

            if (g_next_track_pending && !g_stop_flag) {
                int track_index      = g_next_track;
                g_next_track_pending = false;
                g_next_track         = -1;

                if (track_index >= 0 && track_index < sd_get_wav_count()) {
                    const char *filepath = sd_get_wav_path(track_index);
                    if (filepath) {
                        g_stop_flag     = false;
                        g_playing_track = track_index;
                        g_playing       = true;
                        g_pause         = false;
                        ESP_LOGI(TAG, "Playing track %d: %s", track_index, filepath);
                        audio_play_file(filepath, &g_stop_flag, &g_pause);
                        if (!g_stop_flag) ESP_LOGI(TAG, "Track finished normally");
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

        /* ---- Regular track playback (direct notify) ---- */
        if (notification_value != 0) {
            int track_index = (int)notification_value - 1;

            g_next_track_pending = false;
            g_next_track         = -1;

            if (track_index < 0 || track_index >= sd_get_wav_count()) {
                ESP_LOGW(TAG, "Invalid track index: %d", track_index);
                g_playing       = false;
                g_playing_track = -1;
                continue;
            }

            const char *filepath = sd_get_wav_path(track_index);
            if (!filepath) {
                g_playing       = false;
                g_playing_track = -1;
                continue;
            }

            g_stop_flag     = false;
            g_playing_track = track_index;
            g_playing       = true;
            g_pause         = false;

            audio_play_file(filepath, &g_stop_flag, &g_pause);

            if (!g_stop_flag) ESP_LOGI(TAG, "Track finished normally");

            g_playing       = false;
            g_playing_track = -1;
        }
    }
}

/* -------------------------------------------------------------------------
 * Settle timer callback
 *
 * FIX 1: No state/index captured. Just sends NOTIFY_SETTLE_BIT.
 * audio_task reads nav position live when it handles the bit.
 * ---------------------------------------------------------------------- */
static void enc_settle_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    g_stop_flag = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(audio_task_handle, NOTIFY_SETTLE_BIT,
                       eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* -------------------------------------------------------------------------
 * Helper: request subfolder entry via audio_task (FIX 2)
 *
 * Instead of calling announce + sd_scan + nav_go_forward in the main loop
 * (which races), we package the subfolder path and hand off to audio_task
 * via NOTIFY_ENTER_SUBFOLDER_BIT.  audio_task plays the intro, then scans
 * and transitions — all sequentially.
 * ---------------------------------------------------------------------- */
static void request_enter_subfolder(const char *spath)
{
    if (!spath || spath[0] == '\0') return;

    /* Copy path before sending notification */
    strncpy(g_pending_subfolder_path, spath, sizeof(g_pending_subfolder_path) - 1);
    g_pending_subfolder_path[sizeof(g_pending_subfolder_path) - 1] = '\0';

    /* Stop current audio and wake audio_task */
    g_stop_flag = true;
    xTaskNotify(audio_task_handle, NOTIFY_ENTER_SUBFOLDER_BIT, eSetBits);
}

/* -------------------------------------------------------------------------
 * Encoder callback — rotation & button press
 * ---------------------------------------------------------------------- */
static void encoder_callback(encoder_event_t event) {
    nav_state_t state = nav_get_state();

    if (event.type == ENC_EVENT_ROTATE) {

        g_stop_flag = true;

        if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
            int total = sd_get_folder_count();
            if (total <= 0) return;

            do {
                if (event.direction == ENC_DIR_CW)
                    nav_next_folder(total);
                else
                    nav_prev_folder(total);
            } while (folder_is_hidden(
                         get_folder_name(
                             sd_get_folder_path(nav_get_selected_folder()))));

            int selected = nav_get_selected_folder();
            ESP_LOGI(TAG, "Encoder: Main folder %d selected (%s)",
                     selected,
                     get_folder_name(sd_get_folder_path(selected)));

            /* FIX 1: just re-arm timer — no index/state capture */
            xTimerReset(g_enc_settle_timer, 0);

        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            int total = sd_get_subfolder_count();
            if (total <= 0) return;

            if (event.direction == ENC_DIR_CW)
                nav_next_subfolder(total);
            else
                nav_prev_subfolder(total);

            int selected = nav_get_selected_subfolder();
            ESP_LOGI(TAG, "Encoder: Subfolder %d selected (%s)",
                     selected,
                     get_folder_name(sd_get_subfolder_path(selected)));

            /* FIX 1: just re-arm timer */
            xTimerReset(g_enc_settle_timer, 0);

        } else if (state == NAV_STATE_FILE_VIEW) {
            int total = sd_get_wav_count();
            if (total <= 0) return;

            if (event.direction == ENC_DIR_CW)
                nav_next_track(total);
            else
                nav_prev_track(total);

            int selected = nav_get_selected_track();
            ESP_LOGI(TAG, "Encoder: Track %d selected (%s)",
                     selected, get_filename(sd_get_wav_path(selected)));

            /* FIX 1: just re-arm timer */
            xTimerReset(g_enc_settle_timer, 0);
        }

    } else if (event.type == ENC_EVENT_BUTTON) {

        if (state == NAV_STATE_HOME) {
            if (sd_get_folder_count() > 0) {
                nav_go_forward();

                int total = sd_get_folder_count();
                while (folder_is_hidden(
                           get_folder_name(
                               sd_get_folder_path(nav_get_selected_folder())))) {
                    nav_next_folder(total);
                }

                int selected      = nav_get_selected_folder();
                const char *fpath = sd_get_folder_path(selected);
                const char *fname = get_folder_name(fpath);
                const char *ann   = announce_get_folder_file(fname);
                if (ann)
                    announce_check_and_play(NULL, NULL, ann,
                                            audio_task_handle, &g_stop_flag);

                ESP_LOGI(TAG, "Encoder button: HOME -> FOLDER_VIEW");
            }

        } else if (state == NAV_STATE_FOLDER_VIEW) {
            int selected      = nav_get_selected_folder();
            const char *fpath = sd_get_folder_path(selected);
            const char *fname = get_folder_name(fpath);

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
                    announce_check_and_play(NULL, NULL, ann,
                                            audio_task_handle, &g_stop_flag);

                ESP_LOGI(TAG, "Encoder button: FOLDER_VIEW -> SUBFOLDER_VIEW (%d sub)", num_sub);

            } else {
                sd_scan_wav_files(fpath);
                int num_wavs = sd_get_wav_count();

                if (num_wavs > 0) {
                    nav_go_to_files_direct();
                    g_in_flat_folder = true;

                    if (strcasecmp(fname, "TAWID") == 0) {
                        announce_check_and_play(NULL, fpath, "TT1.WAV",
                                                audio_task_handle, &g_stop_flag);
                    } else {
                        const char *fp0 = sd_get_wav_path(0);
                        announce_play_track(get_filename(fp0));
                    }

                    ESP_LOGI(TAG, "Encoder button: Flat folder FILE_VIEW (%d tracks)", num_wavs);
                } else {
                    ESP_LOGW(TAG, "No files in %s", fpath);
                }
            }

        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            /* FIX 2: hand off to audio_task — no racing announce+scan here */
            int selected      = nav_get_selected_subfolder();
            const char *spath = sd_get_subfolder_path(selected);

            if (!spath) return;

            ESP_LOGI(TAG, "Encoder button: SUBFOLDER_VIEW -> FILE_VIEW (pending)");
            request_enter_subfolder(spath);

        } else if (state == NAV_STATE_FILE_VIEW) {
            int track = nav_get_selected_track();

            if (track < 0 || track >= sd_get_wav_count()) {
                nav_set_selected_track(0);
                track = 0;
            }

            if (!g_playing) {
                start_track_playback(track);
                ESP_LOGI(TAG, "Encoder button: Start playback track %d", track);
            } else if (g_playing_track != track) {
                g_stop_flag = true;
                start_track_playback(track);
                ESP_LOGI(TAG, "Encoder button: Switch to track %d", track);
            }
        }
    }
}

/* -------------------------------------------------------------------------
 * Main application entry point
 * ---------------------------------------------------------------------- */
void app_main(void) {
    ESP_LOGI(TAG, "=== Noor Audio Player (4-Level Navigation) ===");
    ESP_LOGI(TAG, "Initializing modules...");

    audio_init();
    buttons_init();
    nav_init();

    button_init_struct(&btn_play,     BTN_PLAY_PIN);
    button_init_struct(&btn_home,     BTN_HOME_PIN);
    button_init_struct(&btn_vol_up,   BTN_VOLUP_PIN);
    button_init_struct(&btn_vol_down, BTN_VOLDN_PIN);

    ESP_LOGI(TAG, "Attempting USB MSC initialization...");
    if (!usb_msc_init()) {
        ESP_LOGW(TAG, "USB MSC initialization failed (non-fatal)");
    } else {
        ESP_LOGI(TAG, "USB MSC enabled");
    }

    ESP_LOGI(TAG, "Attempting SD card initialization...");
    bool sd_ok = sd_card_init();

    if (!sd_ok) {
        ESP_LOGE(TAG, "=== CRITICAL: SD card init FAILED ===");
        ESP_LOGE(TAG, "Check pins: CS=%d MOSI=%d MISO=%d CLK=%d",
                 PIN_NUM_CS, PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK);
    } else {
        usb_msc_set_sd_card(sd_get_card_handle());
        ESP_LOGI(TAG, "SD card registered with USB MSC");
        check_sd_ota_update();
    }

    announcements_init();

    if (sd_ok) {
        announce_play_boot_greetings(NULL);
        sd_scan_folders("/sdcard");

        int num_folders = sd_get_folder_count();
        ESP_LOGI(TAG, "Found %d top-level folders", num_folders);

        int default_folder = -1;
        for (int i = 0; i < num_folders; i++) {
            const char *fn = get_folder_name(sd_get_folder_path(i));
            if (strcasecmp(fn, "STORIES") == 0) {
                default_folder = i;
                break;
            }
        }

        if (default_folder < 0) {
            for (int i = 0; i < num_folders; i++) {
                const char *fn = get_folder_name(sd_get_folder_path(i));
                if (!folder_is_hidden(fn)) {
                    default_folder = i;
                    break;
                }
            }
        }

        if (default_folder >= 0) {
            nav_set_selected_folder(default_folder);
            ESP_LOGI(TAG, "Default folder index: %d (%s)",
                     default_folder,
                     get_folder_name(sd_get_folder_path(default_folder)));
        }
    }

    if (headphone_detect_init()) {
        headphone_detect_start_task();
    }

    /* Settle timer — one-shot, re-armed on every rotation */
    g_enc_settle_timer = xTimerCreate(
        "enc_settle",
        pdMS_TO_TICKS(ENC_SETTLE_MS),
        pdFALSE,
        NULL,
        enc_settle_timer_cb
    );
    configASSERT(g_enc_settle_timer != NULL);

    if (encoder_init()) {
        encoder_start_task(encoder_callback);
    }

    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL,
                            PRIORITY_AUDIO, &audio_task_handle, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Initialization complete. Entering main loop...");

    /* =========================================================================
     * Main loop
     * ====================================================================== */
    while (1) {
        nav_state_t state = nav_get_state();

        /* ----------------------------------------------------------------
         * PLAY / PAUSE button
         * -------------------------------------------------------------- */
        if (button_is_pressed(&btn_play)) {
            if (state != NAV_STATE_FILE_VIEW || !g_playing) {
                g_stop_flag = true;
            }

            ESP_LOGI(TAG, "Play button pressed (state=%d)", state);

            if (state == NAV_STATE_HOME) {
                if (sd_get_folder_count() > 0) {
                    nav_go_forward();

                    int total = sd_get_folder_count();
                    while (folder_is_hidden(
                               get_folder_name(
                                   sd_get_folder_path(nav_get_selected_folder())))) {
                        nav_next_folder(total);
                    }

                    int sel       = nav_get_selected_folder();
                    const char *fn = get_folder_name(sd_get_folder_path(sel));
                    const char *ann = announce_get_folder_file(fn);
                    if (ann)
                        announce_check_and_play(NULL, NULL, ann,
                                                audio_task_handle, &g_stop_flag);

                    ESP_LOGI(TAG, "HOME -> FOLDER_VIEW");
                }

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                int selected      = nav_get_selected_folder();
                const char *fpath = sd_get_folder_path(selected);
                const char *fname = get_folder_name(fpath);

                if (!fpath || folder_is_hidden(fname)) goto next_button;

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
                        announce_check_and_play(NULL, NULL, ann,
                                                audio_task_handle, &g_stop_flag);

                    ESP_LOGI(TAG, "Entered folder (subfolders=%d)", num_sub);

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

                        ESP_LOGI(TAG, "Flat folder FILE_VIEW (%d tracks)", num_wavs);
                    } else {
                        ESP_LOGW(TAG, "No files in %s", fpath);
                    }
                }

            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                /* FIX 2: same as encoder button — hand off to audio_task */
                int selected      = nav_get_selected_subfolder();
                const char *spath = sd_get_subfolder_path(selected);

                if (!spath) goto next_button;

                ESP_LOGI(TAG, "Play button: SUBFOLDER_VIEW -> FILE_VIEW (pending)");
                request_enter_subfolder(spath);

            } else if (state == NAV_STATE_FILE_VIEW) {
                int track = nav_get_selected_track();

                if (track < 0 || track >= sd_get_wav_count()) {
                    nav_set_selected_track(0);
                    track = 0;
                }

                if (!g_playing) {
                    start_track_playback(track);
                    ESP_LOGI(TAG, "Play: Start track %d", track);

                } else if (g_playing_track == track) {
                    g_pause = !g_pause;
                    ESP_LOGI(TAG, "Play: %s", g_pause ? "Paused" : "Resumed");

                } else {
                    g_stop_flag = true;
                    start_track_playback(track);
                    ESP_LOGI(TAG, "Play: Switch to track %d", track);
                }
            }
        }

        next_button:;

        /* ----------------------------------------------------------------
         * HOME / BACK button
         * -------------------------------------------------------------- */
        if (button_is_pressed(&btn_home)) {
            g_stop_flag = true;
            ESP_LOGI(TAG, "Home button pressed (state=%d)", state);

            if (state == NAV_STATE_FILE_VIEW) {
                if (g_playing) {
                    g_playing = false;
                    g_pause   = false;
                }
                sd_free_wavs();

                if (g_in_flat_folder) {
                    nav_go_back_from_files_direct();
                    g_in_flat_folder = false;

                    int sel       = nav_get_selected_folder();
                    const char *fn = get_folder_name(sd_get_folder_path(sel));
                    if (strcasecmp(fn, "TAWID") == 0)
                        announce_check_and_play(NULL, sd_get_folder_path(sel), "TA.wav",
                                                audio_task_handle, &g_stop_flag);
                    else {
                        const char *ann = announce_get_folder_file(fn);
                        if (ann)
                            announce_check_and_play(NULL, NULL, ann,
                                                    audio_task_handle, &g_stop_flag);
                    }

                    ESP_LOGI(TAG, "Back (flat): FILE_VIEW -> FOLDER_VIEW");

                } else {
                    nav_go_back();

                    int sel        = nav_get_selected_subfolder();
                    const char *sp = sd_get_subfolder_path(sel);
                    const char *sn = get_folder_name(sp);
                    const char *ann = announce_get_folder_file(sn);
                    if (ann)
                        announce_check_and_play(NULL, NULL, ann,
                                                audio_task_handle, &g_stop_flag);

                    ESP_LOGI(TAG, "FILE_VIEW -> SUBFOLDER_VIEW");
                }

            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                sd_free_subfolders();
                nav_go_back();

                int sel       = nav_get_selected_folder();
                const char *fn = get_folder_name(sd_get_folder_path(sel));
                const char *ann = announce_get_folder_file(fn);
                if (ann)
                    announce_check_and_play(NULL, NULL, ann,
                                            audio_task_handle, &g_stop_flag);

                ESP_LOGI(TAG, "SUBFOLDER_VIEW -> FOLDER_VIEW");

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                nav_go_back();
                announce_play_direct("ROTATE.WAV");
                ESP_LOGI(TAG, "FOLDER_VIEW -> HOME");
            }
        }

        /* ----------------------------------------------------------------
         * VOLUME buttons
         * -------------------------------------------------------------- */
        if (button_is_pressed(&btn_vol_up)) {
            audio_volume_up();
        }

        if (button_is_pressed(&btn_vol_down)) {
            audio_volume_down();
        }

        /* ----------------------------------------------------------------
         * Periodic USB MSC status log
         * -------------------------------------------------------------- */
        static uint32_t last_usb_log = 0;
        uint32_t now = xTaskGetTickCount();
        if ((now - last_usb_log) > pdMS_TO_TICKS(5000)) {
            last_usb_log = now;
            if (usb_msc_is_connected()) {
                ESP_LOGI(TAG, "USB Status: %s", usb_msc_get_status());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
