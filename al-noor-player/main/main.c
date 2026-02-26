/**
 * main.c
 * Noor Audio Player - Main Application
 *
 * Navigation: HOME -> FOLDER_VIEW -> SUBFOLDER_VIEW -> FILE_VIEW -> QUIZ
 *
 * Quiz Flow:
 *   m1.wav ends naturally -> sm1.wav (question) -> opt1+opt2 intro
 *   Encoder rotate -> navigate options
 *   Encoder btn / Play btn -> submit answer
 *   Home btn -> exit quiz
 *
 * Volume: NEVER interrupts playing audio. Volume wav plays AFTER current audio.
 * Home: home.wav plays whenever returning to HOME state from anywhere.
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <dirent.h>
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
#include "quiz.h"
#include "usb_manager.h"
#include "tusb_msc_storage.h"
#include "battery.h"

static const char *TAG = "MAIN";

/* ------------------------------------------------------------------ */
/*  Playback state                                                      */
/* ------------------------------------------------------------------ */
static volatile bool g_playing       = false;
static volatile bool g_pause         = false;
static volatile int  g_playing_track = -1;
static volatile bool g_stop_flag     = false;

/* Last story path — for quiz after natural end */
static char g_last_story_path[512] = {0};

/* Quiz state */
static volatile bool g_quiz_active       = false;
static volatile bool g_quiz_options_done = false; /* options intro finished, ready for input */
static volatile bool g_quiz_submitted    = false;

static TaskHandle_t audio_task_handle = NULL;

#define NOTIFY_QUIZ_OPTS_BIT   (1u << 30)
#define NOTIFY_HOME_AUDIO_BIT  (1u << 28)  /* play welcome.wav + rotate.wav */

/* ------------------------------------------------------------------ */
/*  Buttons                                                             */
/* ------------------------------------------------------------------ */
static button_t btn_play;
static button_t btn_home;
static button_t btn_vol_up;
static button_t btn_vol_down;

/* ------------------------------------------------------------------ */
/*  Path helpers                                                        */
/* ------------------------------------------------------------------ */
static const char *get_folder_name(const char *path) {
    if (!path) return NULL;
    const char *n = strrchr(path, '/');
    return n ? n + 1 : path;
}
static const char *get_filename(const char *path) {
    if (!path) return NULL;
    const char *n = strrchr(path, '/');
    return n ? n + 1 : path;
}
static char *get_folder_dir(const char *filepath) {
    if (!filepath) return NULL;
    const char *last = strrchr(filepath, '/');
    if (!last) return NULL;
    size_t len = last - filepath;
    char  *d   = malloc(len + 1);
    if (!d) return NULL;
    memcpy(d, filepath, len);
    d[len] = '\0';
    return d;
}

/* ------------------------------------------------------------------ */
/*  play_home - play welcome.wav + rotate.wav via audio task           */
/* ------------------------------------------------------------------ */
static void play_home(void) {
    g_stop_flag = true;
    vTaskDelay(pdMS_TO_TICKS(50));
    g_stop_flag = false;
    xTaskNotify(audio_task_handle, NOTIFY_HOME_AUDIO_BIT, eSetBits);
}

/* ------------------------------------------------------------------ */
/*  Announce helpers                                                    */
/* ------------------------------------------------------------------ */
static void announce_folder(const char *folder_path, const char *folder_name) {
    if (!folder_name) return;
    const char *wav = announce_get_folder_file(folder_name);
    if (wav) {
        announce_check_and_play("/sdcard", NULL, wav,
                                audio_task_handle, &g_stop_flag);
    } else if (strcasecmp(folder_name, "TAWID") == 0) {
        announce_check_and_play("/sdcard", (char*)folder_path, "TA.wav",
                                audio_task_handle, &g_stop_flag);
    }
}

/* Volume announce - does NOT set g_stop_flag, does NOT interrupt story */
static void announce_volume_only(void) {
    char vol_file[16];
    if (!announce_get_volume_file(audio_get_volume(), vol_file, sizeof(vol_file))) return;
    /* Only play volume announcement if nothing important is playing */
    if (!g_playing && !g_quiz_active) {
        announce_check_and_play("/sdcard", NULL, vol_file,
                                audio_task_handle, &g_stop_flag);
    }
    /* If story is playing - just change volume silently, no interruption */
}

/* ------------------------------------------------------------------ */
/*  Quiz helpers                                                        */
/* ------------------------------------------------------------------ */
static void try_start_quiz(const char *story_path) {
    if (!story_path || story_path[0] == '\0') return;
    if (!quiz_exists_for_story(story_path)) {
        ESP_LOGI(TAG, "No quiz for: %s", get_filename(story_path));
        return;
    }
    if (!quiz_begin(story_path)) {
        ESP_LOGE(TAG, "quiz_begin failed");
        return;
    }

    ESP_LOGI(TAG, "=== Quiz START: %s ===", get_filename(story_path));
    g_quiz_active       = true;
    g_quiz_options_done = false;
    g_quiz_submitted    = false;
    nav_set_state(NAV_STATE_QUIZ);

    /* Play question - audio_task will detect end and play options */
    g_stop_flag = false;
    quiz_play_question(audio_task_handle, &g_stop_flag);
}

static void quiz_do_submit(void) {
    if (!g_quiz_active || g_quiz_submitted) return;
    g_quiz_submitted = true;

    g_stop_flag = true;
    vTaskDelay(pdMS_TO_TICKS(50));

    quiz_result_t result = quiz_submit();
    ESP_LOGI(TAG, "Quiz result: %s",
             result == QUIZ_RESULT_CORRECT ? "CORRECT" : "WRONG");
    g_stop_flag = false;
    quiz_play_result(result, audio_task_handle, &g_stop_flag);
}

static void quiz_do_exit(void) {
    g_stop_flag         = true;
    g_quiz_active       = false;
    g_quiz_submitted    = false;
    g_quiz_options_done = false;
    quiz_end();
    nav_set_state(NAV_STATE_FILE_VIEW);
    ESP_LOGI(TAG, "Quiz exited -> FILE_VIEW");

    /* Announce current track position */
    vTaskDelay(pdMS_TO_TICKS(100));
    g_stop_flag = false;
    int         sel   = nav_get_selected_track();
    const char *fp    = sd_get_wav_path(sel);
    char       *fdir  = get_folder_dir(fp);
    char        story[32];
    if (announce_get_story_file(get_filename(fp), story, sizeof(story)))
        announce_check_and_play("/sdcard", fdir, story,
                                audio_task_handle, &g_stop_flag);
    if (fdir) free(fdir);
}

/* ------------------------------------------------------------------ */
/*  AUDIO TASK                                                          */
/*                                                                      */
/*  Notification values:                                                */
/*    NOTIFY_ANNOUNCE_BIT set  -> play announcement                     */
/*    1..N                     -> play track N-1                        */
/*    NOTIFY_QUIZ_OPTS_BIT     -> play options intro (after question)   */
/* ------------------------------------------------------------------ */
static void audio_task(void *arg) {
    ESP_LOGI(TAG, "Audio task started");

    while (1) {
        uint32_t notif = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notif, portMAX_DELAY);

        /* --- USB MSC state change (SD card returned from USB host) --- */
        if (notif & NOTIFY_USB_MSC_CHANGED_BIT) {
            /* Give TinyUSB time to finish the FAT remount before scanning */
            vTaskDelay(pdMS_TO_TICKS(800));
            /* If remount didn't take, try once more explicitly */
            {
                DIR *_d = opendir("/sdcard");
                if (!_d) {
                    ESP_LOGW(TAG, "SD not ready, retrying mount...");
                    tinyusb_msc_storage_mount("/sdcard");
                    vTaskDelay(pdMS_TO_TICKS(500));
                } else {
                    closedir(_d);
                }
            }
            sd_scan_folders("/sdcard");
            nav_set_state(NAV_STATE_HOME);
            nav_set_selected_folder(0);
            nav_set_selected_subfolder(0);
            nav_set_selected_track(0);
            g_playing       = false;
            g_pause         = false;
            g_stop_flag     = false;
            g_quiz_active   = false;
            quiz_end();
            announce_play_boot_greetings("/sdcard", &g_stop_flag);
            continue;
        }

        /* --- Home audio: welcome.wav + rotate.wav --- */
        if (notif & NOTIFY_HOME_AUDIO_BIT) {
            if (usb_manager_msc_active()) { continue; }
            g_stop_flag = false;
            announce_play_boot_greetings("/sdcard", &g_stop_flag);
            usb_manager_set_boot_audio_done();  /* unblock first USB connect */
            continue;
        }

        /* --- Quiz options intro trigger --- */
        if (notif & NOTIFY_QUIZ_OPTS_BIT) {
            if (g_quiz_active && !g_quiz_options_done) {
                ESP_LOGI(TAG, "Playing options intro");
                vTaskDelay(pdMS_TO_TICKS(400));
                g_stop_flag = false;
                quiz_play_options_intro(audio_task_handle, &g_stop_flag);
                if (!g_stop_flag) {
                    g_quiz_options_done = true;
                    ESP_LOGI(TAG, "Options intro done - waiting for user input");
                }
            }
            continue;
        }

        /* --- Announcement --- */
        if (notif & NOTIFY_ANNOUNCE_BIT) {
            if (usb_manager_msc_active()) { announce_clear_pending(); continue; }
            char path[512];
            if (announce_get_pending(path, sizeof(path))) {
                bool was_question = (g_quiz_active && !g_quiz_options_done);

                g_stop_flag = false;
                ESP_LOGI(TAG, "Announce: %s", get_filename(path));
                audio_play_file(path, &g_stop_flag, NULL);
                announce_clear_pending();

                /* If this was the quiz question and it ended naturally */
                if (was_question && !g_stop_flag && g_quiz_active) {
                    ESP_LOGI(TAG, "Question finished -> triggering options");
                    xTaskNotify(audio_task_handle,
                                NOTIFY_QUIZ_OPTS_BIT, eSetBits);
                }
            }
            continue;
        }

        /* --- Regular story playback --- */
        if (notif != 0) {
            if (usb_manager_msc_active()) continue;  /* SD not available */
            int idx = (int)notif - 1;
            if (idx < 0 || idx >= sd_get_wav_count()) continue;

            const char *fp = sd_get_wav_path(idx);
            if (!fp) continue;

            strncpy(g_last_story_path, fp, sizeof(g_last_story_path) - 1);
            g_last_story_path[sizeof(g_last_story_path) - 1] = '\0';

            g_stop_flag     = false;
            g_playing_track = idx;
            g_playing       = true;
            g_pause         = false;

            ESP_LOGI(TAG, "Playing: %s", get_filename(fp));
            audio_play_file(fp, &g_stop_flag, &g_pause);

            g_playing       = false;
            g_playing_track = -1;

            /* Only start quiz if ended naturally (not interrupted) */
            if (!g_stop_flag && !g_quiz_active) {
                ESP_LOGI(TAG, "Story finished naturally -> quiz check");
                try_start_quiz(g_last_story_path);
            } else {
                ESP_LOGI(TAG, "Story stopped (stop_flag=%d)", g_stop_flag);
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  ENCODER CALLBACK                                                    */
/* ------------------------------------------------------------------ */
static void encoder_callback(encoder_event_t event) {
    nav_state_t state = nav_get_state();

    /* === QUIZ === */
    if (state == NAV_STATE_QUIZ) {
        if (event.type == ENC_EVENT_ROTATE) {
            if (!g_quiz_options_done || g_quiz_submitted) return;
            if (event.direction == ENC_DIR_CW) quiz_next_option();
            else                                quiz_prev_option();
            g_stop_flag = true;
            vTaskDelay(pdMS_TO_TICKS(20));
            g_stop_flag = false;
            quiz_announce_option(quiz_get_selected(),
                                 audio_task_handle, &g_stop_flag);
        } else if (event.type == ENC_EVENT_BUTTON) {
            if (g_quiz_options_done && !g_quiz_submitted)
                quiz_do_submit();
        }
        return;
    }

    /* === ROTATION - always stop current audio === */
    if (event.type == ENC_EVENT_ROTATE) {
        g_stop_flag = true;

        if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
            if (state == NAV_STATE_HOME) nav_set_state(NAV_STATE_FOLDER_VIEW);
            int total = sd_get_folder_count();
            if (total > 0) {
                if (event.direction == ENC_DIR_CW) nav_next_folder(total);
                else                                nav_prev_folder(total);
                vTaskDelay(pdMS_TO_TICKS(30));
                g_stop_flag = false;
                int         sel = nav_get_selected_folder();
                const char *fp  = sd_get_folder_path(sel);
                announce_folder(fp, get_folder_name(fp));
            }

        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            int total = sd_get_subfolder_count();
            if (total > 0) {
                if (event.direction == ENC_DIR_CW) nav_next_subfolder(total);
                else                                nav_prev_subfolder(total);
                vTaskDelay(pdMS_TO_TICKS(30));
                g_stop_flag = false;
                int         sel = nav_get_selected_subfolder();
                const char *fp  = sd_get_subfolder_path(sel);
                announce_folder(fp, get_folder_name(fp));
            }

        } else if (state == NAV_STATE_FILE_VIEW) {
            int total = sd_get_wav_count();
            if (total > 0) {
                if (event.direction == ENC_DIR_CW) nav_next_track(total);
                else                                nav_prev_track(total);
                int         sel   = nav_get_selected_track();
                const char *fp    = sd_get_wav_path(sel);
                char       *fdir  = get_folder_dir(fp);
                const char *fdn   = fdir ? get_folder_name(fdir) : NULL;
                /* Stop current audio, then reset flag before announcing */
                vTaskDelay(pdMS_TO_TICKS(30));
                g_stop_flag = false;
                if (fdn && strcasecmp(fdn, "TAWID") == 0) {
                    char preview[32];
                    snprintf(preview, sizeof(preview), "TT%d.wav", sel + 1);
                    announce_check_and_play("/sdcard", fdir, preview,
                                           audio_task_handle, &g_stop_flag);
                } else {
                    char story[32];
                    if (announce_get_story_file(get_filename(fp), story, sizeof(story)))
                        announce_check_and_play("/sdcard", NULL, story,
                                               audio_task_handle, &g_stop_flag);
                }
                if (fdir) free(fdir);
            }
        }

    /* === ENCODER BUTTON === */
    } else if (event.type == ENC_EVENT_BUTTON) {

        if (state == NAV_STATE_HOME) {
            if (sd_get_folder_count() > 0) {
                g_stop_flag = true;
                nav_go_forward(); /* HOME -> FOLDER_VIEW */
                int         sel = nav_get_selected_folder();
                const char *fp  = sd_get_folder_path(sel);
                announce_folder(fp, get_folder_name(fp));
            }

        } else if (state == NAV_STATE_FOLDER_VIEW) {
            g_stop_flag = true;
            int         sel = nav_get_selected_folder();
            const char *fp  = sd_get_folder_path(sel);
            if (!fp) return;
            sd_scan_subfolders(fp);
            if (sd_get_subfolder_count() > 0) {
                nav_go_forward(); /* FOLDER_VIEW -> SUBFOLDER_VIEW */
                int         si  = nav_get_selected_subfolder();
                const char *sfp = sd_get_subfolder_path(si);
                announce_folder(sfp, get_folder_name(sfp));
            } else {
                sd_scan_wav_files(fp);
                if (sd_get_wav_count() > 0) {
                    nav_go_forward(); nav_go_forward(); /* -> FILE_VIEW */
                    const char *wfp  = sd_get_wav_path(0);
                    char       *wdir = get_folder_dir(wfp);
                    const char *wdn  = wdir ? get_folder_name(wdir) : NULL;
                    if (wdn && strcasecmp(wdn, "TAWID") == 0)
                        announce_check_and_play("/sdcard", wdir, "TA.wav",
                                               audio_task_handle, &g_stop_flag);
                    else {
                        char story[32];
                        if (announce_get_story_file(get_filename(wfp), story, sizeof(story)))
                            announce_check_and_play("/sdcard", wdir, story,
                                                   audio_task_handle, &g_stop_flag);
                    }
                    if (wdir) free(wdir);
                }
            }

        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            g_stop_flag = true;
            int         sel = nav_get_selected_subfolder();
            const char *sfp = sd_get_subfolder_path(sel);
            if (!sfp) return;
            const char *en = announce_get_entering_folder_file(get_folder_name(sfp));
            if (en) announce_check_and_play("/sdcard", sfp, en,
                                           audio_task_handle, &g_stop_flag);
            sd_scan_wav_files(sfp);
            if (sd_get_wav_count() > 0) {
                nav_go_forward(); /* SUBFOLDER_VIEW -> FILE_VIEW */
                const char *wfp  = sd_get_wav_path(0);
                char       *wdir = get_folder_dir(wfp);
                char        story[32];
                if (announce_get_story_file(get_filename(wfp), story, sizeof(story)))
                    announce_check_and_play("/sdcard", wdir, story,
                                           audio_task_handle, &g_stop_flag);
                if (wdir) free(wdir);
            }

        } else if (state == NAV_STATE_FILE_VIEW) {
            int track = nav_get_selected_track();
            if (!g_playing) {
                g_stop_flag = true;
                vTaskDelay(pdMS_TO_TICKS(30));
                xTaskNotify(audio_task_handle, (uint32_t)(track + 1),
                            eSetValueWithOverwrite);
                g_playing = true; g_pause = false; g_playing_track = track;
            } else {
                if (g_playing_track == track) {
                    g_pause = !g_pause; /* pause/resume same track */
                } else {
                    g_stop_flag = true;
                    vTaskDelay(pdMS_TO_TICKS(30));
                    xTaskNotify(audio_task_handle, (uint32_t)(track + 1),
                                eSetValueWithOverwrite);
                    g_playing_track = track; g_pause = false;
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  app_main                                                            */
/* ------------------------------------------------------------------ */
void app_main(void) {
    ESP_LOGI(TAG, "=== Noor Audio Player Booting ===");

    battery_init();
    audio_init();
    audio_register_stop_flag(&g_stop_flag);  /* Allow USB manager to halt audio */
    buttons_init();
    nav_init();
    announcements_init();
    quiz_init();
    headphone_detect_init();

    button_init_struct(&btn_play,     BTN_PLAY_PIN);
    button_init_struct(&btn_home,     BTN_HOME_PIN);
    button_init_struct(&btn_vol_up,   BTN_VOLUP_PIN);
    button_init_struct(&btn_vol_down, BTN_VOLDN_PIN);

    /* Create audio task FIRST — usb_manager_init() needs the task handle */
    xTaskCreatePinnedToCore(audio_task, "audio", 10240, NULL,
                            PRIORITY_AUDIO, &audio_task_handle, tskNO_AFFINITY);

    battery_set_audio_task(audio_task_handle, &g_stop_flag);

    if (!sd_card_init()) {
        ESP_LOGE(TAG, "SD card FAILED - check wiring");
    } else {
        /* usb_manager_init() mounts /sdcard via tinyusb_msc_storage_mount()
         * and starts the TinyUSB CDC+MSC composite device.              */
        usb_manager_init(audio_task_handle);

        sd_scan_folders("/sdcard");
        if (sd_get_folder_count() > 0) {
            int def = 0;
            for (int i = 0; i < sd_get_folder_count(); i++) {
                const char *n = get_folder_name(sd_get_folder_path(i));
                if (n && (strcasecmp(n, "STORIES") == 0 ||
                          strcasecmp(n, "StoriesofProphets") == 0)) {
                    def = i; break;
                }
            }
            nav_set_selected_folder(def);
        }
        /* Notify audio task to play welcome.wav + rotate.wav.
         * _premount_cb waits 5s on the first USB connect so boot audio
         * finishes before the SD card is handed to the PC.              */
        g_stop_flag = false;
        xTaskNotify(audio_task_handle, NOTIFY_HOME_AUDIO_BIT, eSetBits);
    }

    if (encoder_init()) encoder_start_task(encoder_callback);
    headphone_detect_start_task();
    ESP_LOGI(TAG, "Init complete.");

    /* ---------------------------------------------------------------- */
    /*  Main loop                                                         */
    /* ---------------------------------------------------------------- */
    while (1) {
        nav_state_t state = nav_get_state();

        /* === PLAY BUTTON === */
        if (button_is_pressed(&btn_play)) {

            if (state == NAV_STATE_QUIZ) {
                if (g_quiz_options_done && !g_quiz_submitted)
                    quiz_do_submit();

            } else if (state == NAV_STATE_FILE_VIEW) {
                int track = nav_get_selected_track();
                if (!g_playing) {
                    g_stop_flag = true;
                    vTaskDelay(pdMS_TO_TICKS(30));
                    xTaskNotify(audio_task_handle, (uint32_t)(track + 1),
                                eSetValueWithOverwrite);
                    g_playing = true; g_pause = false; g_playing_track = track;
                } else {
                    if (g_playing_track == track) {
                        g_pause = !g_pause; /* pause/resume */
                    } else {
                        g_stop_flag = true;
                        vTaskDelay(pdMS_TO_TICKS(30));
                        xTaskNotify(audio_task_handle, (uint32_t)(track + 1),
                                    eSetValueWithOverwrite);
                        g_playing_track = track; g_pause = false;
                    }
                }
            } else if (state == NAV_STATE_HOME) {
                if (sd_get_folder_count() > 0) {
                    g_stop_flag = true;
                    nav_go_forward();
                    int sel = nav_get_selected_folder();
                    const char *fp = sd_get_folder_path(sel);
                    announce_folder(fp, get_folder_name(fp));
                }

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                g_stop_flag = true;
                int         sel = nav_get_selected_folder();
                const char *fp  = sd_get_folder_path(sel);
                if (fp) {
                    sd_scan_subfolders(fp);
                    if (sd_get_subfolder_count() > 0) {
                        nav_go_forward();
                        int         si  = nav_get_selected_subfolder();
                        const char *sfp = sd_get_subfolder_path(si);
                        announce_folder(sfp, get_folder_name(sfp));
                    } else {
                        sd_scan_wav_files(fp);
                        if (sd_get_wav_count() > 0) {
                            nav_go_forward(); nav_go_forward();
                            const char *wfp  = sd_get_wav_path(0);
                            char       *wdir = get_folder_dir(wfp);
                            const char *wdn  = wdir ? get_folder_name(wdir) : NULL;
                            if (wdn && strcasecmp(wdn, "TAWID") == 0)
                                announce_check_and_play("/sdcard", wdir, "TA.wav",
                                                       audio_task_handle, &g_stop_flag);
                            else {
                                char story[32];
                                if (announce_get_story_file(get_filename(wfp), story, sizeof(story)))
                                    announce_check_and_play("/sdcard", wdir, story,
                                                           audio_task_handle, &g_stop_flag);
                            }
                            if (wdir) free(wdir);
                        }
                    }
                }

            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                g_stop_flag = true;
                int         sel = nav_get_selected_subfolder();
                const char *sfp = sd_get_subfolder_path(sel);
                if (sfp) {
                    const char *en = announce_get_entering_folder_file(get_folder_name(sfp));
                    if (en) announce_check_and_play("/sdcard", sfp, en,
                                                   audio_task_handle, &g_stop_flag);
                    sd_scan_wav_files(sfp);
                    if (sd_get_wav_count() > 0) {
                        nav_go_forward();
                        const char *wfp  = sd_get_wav_path(0);
                        char       *wdir = get_folder_dir(wfp);
                        char        story[32];
                        if (announce_get_story_file(get_filename(wfp), story, sizeof(story)))
                            announce_check_and_play("/sdcard", wdir, story,
                                                   audio_task_handle, &g_stop_flag);
                        if (wdir) free(wdir);
                    }
                }
            }
        }

        /* === HOME BUTTON === */
        if (button_is_pressed(&btn_home)) {
            g_stop_flag = true;
            vTaskDelay(pdMS_TO_TICKS(50));

            if (state == NAV_STATE_QUIZ) {
                quiz_do_exit();

            } else if (state == NAV_STATE_FILE_VIEW) {
                g_playing = false; g_pause = false;
                sd_free_wavs();
                nav_go_back(); /* FILE_VIEW -> SUBFOLDER_VIEW */
                int         si  = nav_get_selected_subfolder();
                const char *sfp = sd_get_subfolder_path(si);
                if (sfp) {
                    g_stop_flag = false;
                    announce_folder(sfp, get_folder_name(sfp));
                }

            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                sd_free_subfolders();
                nav_go_back(); /* SUBFOLDER_VIEW -> FOLDER_VIEW */
                int         fi = nav_get_selected_folder();
                const char *fp = sd_get_folder_path(fi);
                if (fp) {
                    g_stop_flag = false;
                    announce_folder(fp, get_folder_name(fp));
                }

            } else if (state == NAV_STATE_FOLDER_VIEW) {
                nav_go_back(); /* FOLDER_VIEW -> HOME */
                g_stop_flag = false;
                play_home();

            } else if (state == NAV_STATE_HOME) {
                /* Already home - just play home sound */
                g_stop_flag = false;
                play_home();
            }
        }

        /* === VOLUME BUTTONS - NEVER interrupt playing story === */
        if (button_is_pressed(&btn_vol_up)) {
            audio_volume_up();
            announce_volume_only();
        }
        if (button_is_pressed(&btn_vol_down)) {
            audio_volume_down();
            announce_volume_only();
        }

        /* === FIRMWARE UPDATE MODE (Home + VolUp held for 3 seconds) ===
         * Reboots into USB ROM download mode for flashing via esptool.py.
         * button_read_raw() reads raw GPIO level (no debounce edge detect)
         * so both buttons can be detected as simultaneously held.         */
        {
            static uint32_t s_fw_hold_start = 0;
            if (button_read_raw(BTN_HOME_PIN) && button_read_raw(BTN_VOLUP_PIN)) {
                uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                if (s_fw_hold_start == 0) s_fw_hold_start = now;
                if ((now - s_fw_hold_start) >= 3000) {
                    ESP_LOGW(TAG, "Home+VolUp held 3s -> entering USB download mode");
                    g_stop_flag = true;
                    vTaskDelay(pdMS_TO_TICKS(200));
                    usb_manager_reboot_to_download();  /* does not return */
                }
            } else {
                s_fw_hold_start = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
