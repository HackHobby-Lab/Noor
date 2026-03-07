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
 * Fixes applied vs previous version
 * -----------------------------------
 * 1. announcements_init() now called AFTER sd_card_init() so the
 *    /sdcard/ANNOUN~1 directory is already mounted when probed.
 * 2. announce_play_boot_greetings() called after announcements_init().
 * 3. SYSTEM~1 and ANNOUN~1 folders are hidden from user navigation.
 * 4. TAWID (flat folder – files with no subfolders) uses
 *    nav_go_to_files_direct() / nav_go_back_from_files_direct().
 * 5. Track-announcement played before every story track.
 * 6. VOLUME_MAX enforced at 100 (in config.h / audio.h).
 * 7. "Invalid track index" guard added.
 * 8. announce_check_and_play() now searches the resolved ANNOUN~1
 *    directory automatically, so callers pass NULL for root_path.
 * 9. Back from flat-folder FILE_VIEW goes to FOLDER_VIEW, not
 *    SUBFOLDER_VIEW.
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

/*
 * Set to true when we entered FILE_VIEW directly from FOLDER_VIEW
 * (flat folder like TAWID that has no subfolders).
 * Controls which "back" transition is used.
 */
static bool g_in_flat_folder = false;

/* Task handle for audio task */
static TaskHandle_t audio_task_handle = NULL;

/* Button structures */
static button_t btn_play;
static button_t btn_home;
static button_t btn_vol_up;
static button_t btn_vol_down;

/* -------------------------------------------------------------------------
 * Encoder settle-timer
 *
 * Both problems — missed announcements on fast spin and bounce-back to the
 * same folder — are caused by the same root issue: encoder events arrive
 * faster than the announcement system can react.
 *
 * Fix: instead of playing the announcement on every rotation event, we
 * record the last-rotation timestamp and the target index.  A lightweight
 * FreeRTOS timer fires ENC_SETTLE_MS after the LAST rotation event and
 * plays the announcement for whatever index is current at that point.
 *
 * This means:
 *  - Fast spinning: only the final folder gets announced (no missed/skipped).
 *  - Bounce: a mechanical ghost-pulse arrives within ENC_SETTLE_MS of the
 *    real click, so the timer is just re-armed and no extra announcement fires.
 *
 * ENC_SETTLE_MS is defined in config.h (default 180 ms).
 * ---------------------------------------------------------------------- */
static TimerHandle_t  g_enc_settle_timer  = NULL;
static volatile int   g_enc_settle_index  = -1;   /* folder/subfolder/track index to announce */
static volatile int   g_enc_settle_state  = -1;   /* nav_state_t at last rotation            */

/* -------------------------------------------------------------------------
 * Helper: last path component ("folder name" or "filename")
 * ---------------------------------------------------------------------- */
static const char *get_folder_name(const char *path) {
    if (!path) return NULL;
    const char *name = strrchr(path, '/');
    return name ? name + 1 : path;
}

static const char *get_filename(const char *path) {
    return get_folder_name(path);   /* same logic */
}

/* Return heap-allocated directory part of a full file path.
 * Caller must free(). */
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

/* -------------------------------------------------------------------------
 * Helper: should this top-level folder be hidden from the user?
 * SYSTEM~1 is Windows metadata; ANNOUN~1 is internal announcement files.
 * ---------------------------------------------------------------------- */
static bool folder_is_hidden(const char *folder_name) {
    if (!folder_name) return true;
    if (strncasecmp(folder_name, "SYSTEM",  6) == 0) return true;
    if (strncasecmp(folder_name, "ANNOUN",  6) == 0) return true;
    return false;
}

/* -------------------------------------------------------------------------
 * Helper: play track announcement then start playback via audio_task.
 *
 * ALL I2S access goes through audio_task — we never call audio_play_file()
 * directly from the main loop or encoder callback.  This eliminates the
 * "I2S port is in use" race condition entirely.
 *
 * Sequence:
 *   1. Set stop_flag so any running playback stops cleanly.
 *   2. Store the announcement path in the pending slot.
 *   3. Send NOTIFY_ANNOUNCE_BIT — audio_task plays the announcement.
 *   4. After the announcement finishes, audio_task checks g_next_track
 *      and starts the story automatically.
 * ---------------------------------------------------------------------- */

/* Next track queued to play after the current announcement finishes */
static volatile int  g_next_track      = -1;
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

    /* Build announcement filename */
    char ann_file[32] = {0};
    bool has_ann = false;

    if (folder_name && strcasecmp(folder_name, "TAWID") == 0) {
        snprintf(ann_file, sizeof(ann_file), "TT%d.WAV", track_index + 1);
        has_ann = true;
    } else {
        has_ann = announce_get_story_file(filename, ann_file, sizeof(ann_file));
    }

    if (folder_dir) free(folder_dir);

    /* Stop any current audio */
    g_stop_flag = true;

    /* Queue the story track so audio_task starts it after announcement */
    g_next_track         = track_index;
    g_next_track_pending = true;
    g_playing_track      = track_index;
    g_playing            = true;
    g_pause              = false;

    if (has_ann) {
        /* Route announcement through audio_task */
        announce_check_and_play(NULL, NULL, ann_file, audio_task_handle, &g_stop_flag);
    } else {
        /* No announcement — kick story directly */
        g_stop_flag = false;
        xTaskNotify(audio_task_handle, (uint32_t)(track_index + 1),
                    eSetValueWithOverwrite);
    }

    ESP_LOGI(TAG, "Playing track %d: %s", track_index, filepath);
}

/* -------------------------------------------------------------------------
 * play_announce_for_index — called from audio_task (full stack, safe for I/O)
 * ---------------------------------------------------------------------- */
static void play_announce_for_index(int state, int index)
{
    if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
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
        const char *spath = sd_get_subfolder_path(index);
        const char *sname = get_folder_name(spath);
        const char *ann   = announce_get_folder_file(sname);
        if (ann)
            announce_check_and_play(NULL, NULL, ann,
                                    audio_task_handle, &g_stop_flag);
    } else if (state == NAV_STATE_FILE_VIEW) {
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
 * Audio task – single owner of the I2S driver
 * ---------------------------------------------------------------------- */
static void audio_task(void *arg) {
    ESP_LOGI(TAG, "Audio task started");

    while (1) {
        uint32_t notification_value = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, portMAX_DELAY);

        /* ---- Encoder settle: announce final selection after spinning ---- */
        if (notification_value & NOTIFY_SETTLE_BIT) {
            int state = g_enc_settle_state;
            int index = g_enc_settle_index;
            g_enc_settle_state = -1;
            g_enc_settle_index = -1;
            if (state >= 0 && index >= 0) {
                g_stop_flag = false;
                play_announce_for_index(state, index);
            }
            continue;  /* never fall through to track-playback handler */
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

            /* If a story track is queued to follow this announcement, play it now.
             * Both the announcement and the story run sequentially in this same
             * task, so the I2S driver is never double-installed. */
            if (g_next_track_pending && !g_stop_flag) {
                int track_index = g_next_track;
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
                /* Announcement was interrupted or no track queued */
                g_next_track_pending = false;
                g_next_track         = -1;
                if (!g_playing) {
                    /* Make sure state is clean */
                }
            }
            continue;
        }

        /* ---- Regular track playback (direct notify, no announcement) ---- */
        if (notification_value != 0) {
            int track_index = (int)notification_value - 1;

            /* Also drain any pending next-track flag to avoid double-play */
            g_next_track_pending = false;
            g_next_track         = -1;

            if (track_index < 0 || track_index >= sd_get_wav_count()) {
                ESP_LOGW(TAG, "Invalid track index: %d", track_index);
                g_playing = false;
                g_playing_track = -1;
                continue;
            }

            const char *filepath = sd_get_wav_path(track_index);
            if (!filepath) {
                g_playing = false;
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
 * Settle timer callback — fires ENC_SETTLE_MS after the last encoder step.
 * Plays the announcement for whatever index is current at that moment.
 * Runs in the FreeRTOS timer daemon task (keep it short — no blocking I/O).
 * ---------------------------------------------------------------------- */
static void enc_settle_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    /* Called from Timer Service task — stack is only ~2 KB.
     * Do NOT call announce_check_and_play() or any file I/O here.
     * Just set the stop flag and wake the audio_task which has a full stack. */
    g_stop_flag = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(audio_task_handle, NOTIFY_SETTLE_BIT,
                       eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* -------------------------------------------------------------------------
 * Encoder callback – rotation & button press
 * ---------------------------------------------------------------------- */
static void encoder_callback(encoder_event_t event) {
    nav_state_t state = nav_get_state();

    /* ================================================================
     * ROTATION
     * ============================================================== */
    if (event.type == ENC_EVENT_ROTATE) {

        /* Stop any currently-playing audio immediately */
        g_stop_flag = true;

        /* ---- HOME or FOLDER_VIEW: cycle main folders ---- */
        if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
            int total = sd_get_folder_count();
            if (total <= 0) return;

            /* Step, skip hidden folders */
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

            /* Arm settle timer — announcement fires after last rotation */
            g_enc_settle_state = (int)state;
            g_enc_settle_index = selected;
            xTimerReset(g_enc_settle_timer, 0);

        /* ---- SUBFOLDER_VIEW: cycle prophet subfolders ---- */
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

            /* Arm settle timer */
            g_enc_settle_state = (int)state;
            g_enc_settle_index = selected;
            xTimerReset(g_enc_settle_timer, 0);

        /* ---- FILE_VIEW: cycle tracks ---- */
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

            /* Arm settle timer */
            g_enc_settle_state = (int)state;
            g_enc_settle_index = selected;
            xTimerReset(g_enc_settle_timer, 0);
        }

    /* ================================================================
     * BUTTON PRESS
     * ============================================================== */
    } else if (event.type == ENC_EVENT_BUTTON) {

        /* ---- HOME: enter FOLDER_VIEW ---- */
        if (state == NAV_STATE_HOME) {
            if (sd_get_folder_count() > 0) {
                nav_go_forward(); /* HOME -> FOLDER_VIEW */

                /* Skip hidden folders */
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

        /* ---- FOLDER_VIEW: enter selected folder ---- */
        } else if (state == NAV_STATE_FOLDER_VIEW) {
            int selected          = nav_get_selected_folder();
            const char *fpath     = sd_get_folder_path(selected);
            const char *fname     = get_folder_name(fpath);

            if (!fpath || folder_is_hidden(fname)) return;

            sd_scan_subfolders(fpath);
            int num_sub = sd_get_subfolder_count();

            if (num_sub > 0) {
                /* Normal folder with subfolders (STORIES) */
                nav_go_forward(); /* FOLDER_VIEW -> SUBFOLDER_VIEW */
                g_in_flat_folder = false;
                nav_set_selected_subfolder(0);

                const char *sp   = sd_get_subfolder_path(0);
                const char *sn   = get_folder_name(sp);
                const char *ann  = announce_get_folder_file(sn);
                if (ann)
                    announce_check_and_play(NULL, NULL, ann,
                                            audio_task_handle, &g_stop_flag);

                ESP_LOGI(TAG, "Encoder button: FOLDER_VIEW -> SUBFOLDER_VIEW (%d sub)", num_sub);

            } else {
                /* Flat folder: no subfolders — scan WAV files directly (TAWID) */
                sd_scan_wav_files(fpath);
                int num_wavs = sd_get_wav_count();

                if (num_wavs > 0) {
                    nav_go_to_files_direct(); /* FOLDER_VIEW -> FILE_VIEW */
                    g_in_flat_folder = true;

                    /* Announce first track */
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

        /* ---- SUBFOLDER_VIEW: enter selected subfolder ---- */
        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            int selected          = nav_get_selected_subfolder();
            const char *spath     = sd_get_subfolder_path(selected);
            const char *sname     = get_folder_name(spath);

            if (!spath) return;

            /* "Entering folder" announcement (e.g. INSMUH.WAV for MUHAMMAD) */
            const char *entering = announce_get_entering_folder_file(sname);
            if (entering)
                announce_check_and_play(NULL, spath, entering,
                                        audio_task_handle, &g_stop_flag);

            sd_scan_wav_files(spath);
            int num_wavs = sd_get_wav_count();

            if (num_wavs > 0) {
                nav_go_forward(); /* SUBFOLDER_VIEW -> FILE_VIEW */
                g_in_flat_folder = false;
                nav_set_selected_track(0);

                /* Announce first track */
                const char *fp0  = sd_get_wav_path(0);
                announce_play_track(get_filename(fp0));

                ESP_LOGI(TAG, "Encoder button: SUBFOLDER_VIEW -> FILE_VIEW (%d tracks)", num_wavs);
            } else {
                ESP_LOGW(TAG, "No files in %s", spath);
            }

        /* ---- FILE_VIEW: start / switch playback ---- */
        } else if (state == NAV_STATE_FILE_VIEW) {
            int track = nav_get_selected_track();

            /* Guard against stale/invalid index */
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
            /* If same track already playing: do nothing (use Play button to pause) */
        }
    }
}

/* -------------------------------------------------------------------------
 * Main application entry point
 * ---------------------------------------------------------------------- */
void app_main(void) {
    ESP_LOGI(TAG, "=== Noor Audio Player (4-Level Navigation) ===");
    ESP_LOGI(TAG, "Initializing modules...");

    /* ---- Core init (no SD dependency) ---- */
    audio_init();
    buttons_init();
    nav_init();

    button_init_struct(&btn_play,     BTN_PLAY_PIN);
    button_init_struct(&btn_home,     BTN_HOME_PIN);
    button_init_struct(&btn_vol_up,   BTN_VOLUP_PIN);
    button_init_struct(&btn_vol_down, BTN_VOLDN_PIN);

    /* ---- USB MSC init (before SD so it can register the card later) ---- */
    ESP_LOGI(TAG, "Attempting USB MSC initialization...");
    if (!usb_msc_init()) {
        ESP_LOGW(TAG, "USB MSC initialization failed (non-fatal)");
    } else {
        ESP_LOGI(TAG, "USB MSC enabled");
    }

    /* ---- SD card init ---- */
    ESP_LOGI(TAG, "Attempting SD card initialization...");
    bool sd_ok = sd_card_init();

    if (!sd_ok) {
        ESP_LOGE(TAG, "=== CRITICAL: SD card init FAILED ===");
        ESP_LOGE(TAG, "Check pins: CS=%d MOSI=%d MISO=%d CLK=%d",
                 PIN_NUM_CS, PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK);
    } else {
        /* Register SD card with USB MSC */
        usb_msc_set_sd_card(sd_get_card_handle());
        ESP_LOGI(TAG, "SD card registered with USB MSC");

        /* Check for firmware.bin on SD card — flashes and reboots if found */
        check_sd_ota_update();
    }

    /* ---- announcements_init() MUST come after SD card mounts ---- */
    announcements_init();   /* probes /sdcard/ANNOUN~1 or /sdcard/announcements */

    if (sd_ok) {
        /* Play boot greetings now that announce_root is resolved */
        announce_play_boot_greetings(NULL);

        /* Scan top-level folders */
        sd_scan_folders("/sdcard");

        int num_folders = sd_get_folder_count();
        ESP_LOGI(TAG, "Found %d top-level folders", num_folders);

        /* Default to STORIES folder if present */
        int default_folder = -1;
        for (int i = 0; i < num_folders; i++) {
            const char *fn = get_folder_name(sd_get_folder_path(i));
            if (strcasecmp(fn, "STORIES") == 0) {
                default_folder = i;
                break;
            }
        }

        /* If STORIES not found, pick first non-hidden folder */
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

    /* ---- Headphone detection ---- */
    if (headphone_detect_init()) {
        headphone_detect_start_task();
    }

    /* ---- Encoder settle timer ----
     * One-shot timer: fires ENC_SETTLE_MS after the last encoder step.
     * Re-armed on every rotation event so it only fires when the knob
     * has been idle for ENC_SETTLE_MS.  This fixes both:
     *   (a) missed announcements when spinning fast — only the final
     *       selected item gets announced.
     *   (b) bounce-back to same folder — mechanical ghost pulses that
     *       arrive within ENC_SETTLE_MS are collapsed into one event.
     */
    g_enc_settle_timer = xTimerCreate(
        "enc_settle",
        pdMS_TO_TICKS(ENC_SETTLE_MS),
        pdFALSE,               /* one-shot */
        NULL,
        enc_settle_timer_cb
    );
    configASSERT(g_enc_settle_timer != NULL);

    /* ---- Encoder ---- */
    if (encoder_init()) {
        encoder_start_task(encoder_callback);
    }

    /* ---- Audio task ---- */
    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL,
                            PRIORITY_AUDIO, &audio_task_handle, tskNO_AFFINITY);

    ESP_LOGI(TAG, "Initialization complete. Entering main loop...");

    /* =========================================================================
     * Main loop — handle dedicated button presses
     * ====================================================================== */
    while (1) {
        nav_state_t state = nav_get_state();

        /* ----------------------------------------------------------------
         * PLAY / PAUSE button
         * -------------------------------------------------------------- */
        if (button_is_pressed(&btn_play)) {
            /* In FILE_VIEW while playing: don't interrupt (just pause/resume) */
            if (state != NAV_STATE_FILE_VIEW || !g_playing) {
                g_stop_flag = true;
            }

            ESP_LOGI(TAG, "Play button pressed (state=%d)", state);

            if (state == NAV_STATE_HOME) {
                /* Same as encoder button at HOME */
                if (sd_get_folder_count() > 0) {
                    nav_go_forward(); /* HOME -> FOLDER_VIEW */

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
                    nav_go_forward(); /* FOLDER_VIEW -> SUBFOLDER_VIEW */
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
                    /* Flat folder (TAWID) */
                    sd_scan_wav_files(fpath);
                    int num_wavs = sd_get_wav_count();

                    if (num_wavs > 0) {
                        nav_go_to_files_direct(); /* FOLDER_VIEW -> FILE_VIEW */
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
                int selected      = nav_get_selected_subfolder();
                const char *spath = sd_get_subfolder_path(selected);
                const char *sname = get_folder_name(spath);

                if (!spath) goto next_button;

                const char *entering = announce_get_entering_folder_file(sname);
                if (entering)
                    announce_check_and_play(NULL, spath, entering,
                                            audio_task_handle, &g_stop_flag);

                sd_scan_wav_files(spath);
                int num_wavs = sd_get_wav_count();

                if (num_wavs > 0) {
                    nav_go_forward(); /* SUBFOLDER_VIEW -> FILE_VIEW */
                    g_in_flat_folder = false;
                    nav_set_selected_track(0);

                    announce_play_track(get_filename(sd_get_wav_path(0)));

                    ESP_LOGI(TAG, "Entered subfolder (files=%d)", num_wavs);
                }

            } else if (state == NAV_STATE_FILE_VIEW) {
                int track = nav_get_selected_track();

                /* Guard against invalid index */
                if (track < 0 || track >= sd_get_wav_count()) {
                    nav_set_selected_track(0);
                    track = 0;
                }

                if (!g_playing) {
                    start_track_playback(track);
                    ESP_LOGI(TAG, "Play: Start track %d", track);

                } else if (g_playing_track == track) {
                    /* Toggle pause on same track */
                    g_pause = !g_pause;
                    ESP_LOGI(TAG, "Play: %s", g_pause ? "Paused" : "Resumed");

                } else {
                    /* Switch to different track */
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
                    /* Came from FOLDER_VIEW directly — go straight back */
                    nav_go_back_from_files_direct(); /* FILE_VIEW -> FOLDER_VIEW */
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
                    nav_go_back(); /* FILE_VIEW -> SUBFOLDER_VIEW */

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
                nav_go_back(); /* SUBFOLDER_VIEW -> FOLDER_VIEW */

                int sel       = nav_get_selected_folder();
                const char *fn = get_folder_name(sd_get_folder_path(sel));
                const char *ann = announce_get_folder_file(fn);
                if (ann)
                    announce_check_and_play(NULL, NULL, ann,
                                            audio_task_handle, &g_stop_flag);

                ESP_LOGI(TAG, "SUBFOLDER_VIEW -> FOLDER_VIEW");

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                nav_go_back(); /* FOLDER_VIEW -> HOME */

                /* Play ROTATE.WAV as "home" audio cue */
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
         * Periodic USB MSC status log (every 5 s)
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
