/**
 * main.c
 * Noor Audio Player - Main Application (4-Level Navigation)
 * 
 * Navigation Levels:
 * 1. HOME - Initial screen
 * 2. FOLDER_VIEW - Browse main folders (e.g., "Stories of Prophets")
 * 3. SUBFOLDER_VIEW - Browse prophet folders (e.g., "Hazrat Muhammed")
 * 4. FILE_VIEW - Browse and play audio files (e.g., m1.wav, m2.wav)
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Our modules
#include "config.h"
#include "sd_card.h"
#include "audio.h"
#include "buttons.h"
#include "encoder.h"
#include "navigation.h"
#include "announcements.h"
#include "headphone_detect.h"

static const char *TAG = "MAIN";

/* Global state */
static volatile bool g_playing = false;
static volatile bool g_pause = false;
static volatile int g_playing_track = -1;
static volatile bool g_stop_flag = false;

/* Task handle for audio task */
static TaskHandle_t audio_task_handle = NULL;

/* Button structures */
static button_t btn_play;
static button_t btn_home;
static button_t btn_vol_up;
static button_t btn_vol_down;

/* Helper: Get folder name from path */
static const char* get_folder_name(const char *path) {
    if (!path) return NULL;
    const char *name = strrchr(path, '/');
    return name ? name + 1 : path;
}

/* Helper: Get filename from path */
static const char* get_filename(const char *path) {
    if (!path) return NULL;
    const char *name = strrchr(path, '/');
    return name ? name + 1 : path;
}

/* Helper: Get folder directory from full file path */
static char* get_folder_dir(const char *filepath) {
    if (!filepath) return NULL;
    
    const char *last_slash = strrchr(filepath, '/');
    if (!last_slash) return NULL;
    
    size_t len = last_slash - filepath;
    char *folder = malloc(len + 1);
    if (!folder) return NULL;
    
    memcpy(folder, filepath, len);
    folder[len] = '\0';
    return folder;
}

/* Audio task - handles playback with interruption support */
static void audio_task(void *arg) {
    ESP_LOGI(TAG, "Audio task started");
    
    while (1) {
        uint32_t notification_value = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, portMAX_DELAY);
        
        // Check if this is an announcement
        if (notification_value & NOTIFY_ANNOUNCE_BIT) {
            char announce_path[ANNOUNCE_PATH_MAX];
            if (announce_get_pending(announce_path, sizeof(announce_path))) {
                g_stop_flag = false;
                ESP_LOGI(TAG, "Playing announcement: %s", announce_path);
                audio_play_file(announce_path, &g_stop_flag, NULL);
                announce_clear_pending();
            }
            continue;
        }
        
        // Regular playback
        if (notification_value != 0) {
            int track_index = (int)notification_value - 1;
            
            if (track_index < 0 || track_index >= sd_get_wav_count()) {
                ESP_LOGW(TAG, "Invalid track index: %d", track_index);
                continue;
            }
            
            const char *filepath = sd_get_wav_path(track_index);
            if (!filepath) continue;
            
            g_stop_flag = false;
            g_playing_track = track_index;
            g_playing = true;
            g_pause = false;
            
            ESP_LOGI(TAG, "Playing track %d: %s", track_index, filepath);
            audio_play_file(filepath, &g_stop_flag, &g_pause);
            
            if (!g_stop_flag) {
                ESP_LOGI(TAG, "Track finished normally");
            }
            
            g_playing = false;
            g_playing_track = -1;
        }
    }
}

/* Encoder callback - called when encoder rotates or button pressed */
static void encoder_callback(encoder_event_t event) {
    nav_state_t state = nav_get_state();
    
    if (event.type == ENC_EVENT_ROTATE) {
        // ALWAYS INTERRUPT AUDIO WHEN ENCODER ROTATES
        g_stop_flag = true;
        
        // Handle rotation based on current state
        
        if (state == NAV_STATE_HOME || state == NAV_STATE_FOLDER_VIEW) {
            // Level 1/2: Rotate through main folders (e.g., "Stories of Prophets")
            int total_folders = sd_get_folder_count();
            if (total_folders > 0) {
                if (event.direction == ENC_DIR_CW) {
                    nav_next_folder(total_folders);
                } else {
                    nav_prev_folder(total_folders);
                }
                
                // Announce folder name
                int selected = nav_get_selected_folder();
                const char *folder_path = sd_get_folder_path(selected);
                const char *folder_name = get_folder_name(folder_path);
                const char *announce_file = announce_get_folder_file(folder_name);
                
                if (announce_file) {
                    announce_check_and_play("/sdcard", NULL, announce_file,
                                          audio_task_handle, &g_stop_flag);
                }
                
                ESP_LOGI(TAG, "Encoder: Main folder %d selected (%s)", selected, folder_name);
            }
            
        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            // Level 3: Rotate through prophet subfolders (e.g., "Hazrat Muhammed")
            int total_subfolders = sd_get_subfolder_count();
            if (total_subfolders > 0) {
                if (event.direction == ENC_DIR_CW) {
                    nav_next_subfolder(total_subfolders);
                } else {
                    nav_prev_subfolder(total_subfolders);
                }
                
                // Announce prophet name
                int selected = nav_get_selected_subfolder();
                const char *subfolder_path = sd_get_subfolder_path(selected);
                const char *subfolder_name = get_folder_name(subfolder_path);
                const char *announce_file = announce_get_folder_file(subfolder_name);
                
                if (announce_file) {
                    announce_check_and_play("/sdcard", NULL, announce_file,
                                          audio_task_handle, &g_stop_flag);
                }
                
                ESP_LOGI(TAG, "Encoder: Subfolder %d selected (%s)", selected, subfolder_name);
            }
            
        } else if (state == NAV_STATE_FILE_VIEW) {
            // Level 4: Rotate through story files (e.g., m1.wav, m2.wav)
            int total_tracks = sd_get_wav_count();
            if (total_tracks > 0) {
                if (event.direction == ENC_DIR_CW) {
                    nav_next_track(total_tracks);
                } else {
                    nav_prev_track(total_tracks);
                }
                
                // Announce story number
                int selected = nav_get_selected_track();
                const char *filepath = sd_get_wav_path(selected);
                const char *filename = get_filename(filepath);
                
                char story_file[32];
                if (announce_get_story_file(filename, story_file, sizeof(story_file))) {
                    char *folder_dir = get_folder_dir(filepath);
                    announce_check_and_play("/sdcard", folder_dir, story_file,
                                          audio_task_handle, &g_stop_flag);
                    free(folder_dir);
                }
                
                ESP_LOGI(TAG, "Encoder: Track %d selected (%s)", selected, filename);
            }
        }
        
    } else if (event.type == ENC_EVENT_BUTTON) {
        // Handle button press based on current state
        
        if (state == NAV_STATE_HOME) {
            // Go to FOLDER_VIEW
            if (sd_get_folder_count() > 0) {
                nav_go_forward();
                
                // Announce first folder
                int selected = nav_get_selected_folder();
                const char *folder_path = sd_get_folder_path(selected);
                const char *folder_name = get_folder_name(folder_path);
                const char *announce_file = announce_get_folder_file(folder_name);
                
                if (announce_file) {
                    announce_check_and_play("/sdcard", NULL, announce_file,
                                          audio_task_handle, &g_stop_flag);
                }
                
                ESP_LOGI(TAG, "Encoder button: HOME -> FOLDER_VIEW");
            }
            
        } else if (state == NAV_STATE_FOLDER_VIEW) {
            // Enter selected main folder and scan for subfolders
            int selected = nav_get_selected_folder();
            const char *folder_path = sd_get_folder_path(selected);
            
            if (folder_path) {
                // Scan for subfolders (prophet folders)
                sd_scan_subfolders(folder_path);
                
                if (sd_get_subfolder_count() > 0) {
                    nav_go_forward(); // FOLDER_VIEW -> SUBFOLDER_VIEW
                    
                    // Announce first subfolder (prophet)
                    int subfolder_idx = nav_get_selected_subfolder();
                    const char *subfolder_path = sd_get_subfolder_path(subfolder_idx);
                    const char *subfolder_name = get_folder_name(subfolder_path);
                    const char *announce_file = announce_get_folder_file(subfolder_name);
                    
                    if (announce_file) {
                        announce_check_and_play("/sdcard", NULL, announce_file,
                                              audio_task_handle, &g_stop_flag);
                    }
                    
                    ESP_LOGI(TAG, "Encoder button: Entered folder (subfolders=%d)", 
                             sd_get_subfolder_count());
                } else {
                    ESP_LOGW(TAG, "No subfolders found in %s", folder_path);
                }
            }
            
        } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
            // Enter selected subfolder and scan for files
            int selected = nav_get_selected_subfolder();
            const char *subfolder_path = sd_get_subfolder_path(selected);
            
            if (subfolder_path) {
                const char *subfolder_name = get_folder_name(subfolder_path);
                
                // Play "entering folder" announcement
                const char *entering_announce = announce_get_entering_folder_file(subfolder_name);
                if (entering_announce) {
                    announce_check_and_play("/sdcard", subfolder_path, entering_announce,
                                          audio_task_handle, &g_stop_flag);
                }
                
                // Scan for WAV files
                sd_scan_wav_files(subfolder_path);
                
                if (sd_get_wav_count() > 0) {
                    nav_go_forward(); // SUBFOLDER_VIEW -> FILE_VIEW
                    
                    // Announce first story
                    const char *filepath = sd_get_wav_path(0);
                    const char *filename = get_filename(filepath);
                    
                    char story_file[32];
                    if (announce_get_story_file(filename, story_file, sizeof(story_file))) {
                        char *folder_dir = get_folder_dir(filepath);
                        announce_check_and_play("/sdcard", folder_dir, story_file,
                                              audio_task_handle, &g_stop_flag);
                        free(folder_dir);
                    }
                    
                    ESP_LOGI(TAG, "Encoder button: Entered subfolder (files=%d)", 
                             sd_get_wav_count());
                } else {
                    ESP_LOGW(TAG, "No audio files found in %s", subfolder_path);
                }
            }
            
        } else if (state == NAV_STATE_FILE_VIEW) {
            // Encoder button in FILE_VIEW: Start playback of selected story
            if (!g_playing) {
                // Start playback of selected track
                int track = nav_get_selected_track();
                if (track >= 0 && track < sd_get_wav_count()) {
                    g_playing_track = track;
                    g_playing = true;
                    g_pause = false;
                    g_stop_flag = true;
                    xTaskNotify(audio_task_handle, (uint32_t)(track + 1), eSetValueWithOverwrite);
                    ESP_LOGI(TAG, "Encoder button: Start playback track %d", track);
                }
            } else {
                // If already playing, switch to selected track
                int track = nav_get_selected_track();
                if (g_playing_track != track) {
                    g_playing_track = track;
                    g_pause = false;
                    g_stop_flag = true;
                    xTaskNotify(audio_task_handle, (uint32_t)(track + 1), eSetValueWithOverwrite);
                    ESP_LOGI(TAG, "Encoder button: Switch to track %d", track);
                }
                // If same track is playing, do nothing (use Play button to pause)
            }
        }
    }
}

/* Main application */
void app_main(void) {
    ESP_LOGI(TAG, "=== Noor Audio Player (4-Level Navigation) ===");
    
    // Initialize all modules
    ESP_LOGI(TAG, "Initializing modules...");
    
    audio_init();
    buttons_init();
    nav_init();
    announcements_init();
    headphone_detect_init();
    
    // Initialize button structures
    button_init_struct(&btn_play, BTN_PLAY_PIN);
    button_init_struct(&btn_home, BTN_HOME_PIN);
    button_init_struct(&btn_vol_up, BTN_VOLUP_PIN);
    button_init_struct(&btn_vol_down, BTN_VOLDN_PIN);
    
    // Initialize SD card
    if (!sd_card_init()) {
        ESP_LOGE(TAG, "SD card initialization failed!");
    } else {
        // Scan main folders
        sd_scan_folders("/sdcard");
        
        if (sd_get_folder_count() > 0) {
            ESP_LOGI(TAG, "Found %d main folders", sd_get_folder_count());
            
            // Try to find "STORIES" folder as default
            int default_folder = 0;
            for (int i = 0; i < sd_get_folder_count(); i++) {
                const char *path = sd_get_folder_path(i);
                const char *name = get_folder_name(path);
                if (strcasecmp(name, "STORIES") == 0) {
                    default_folder = i;
                    break;
                }
            }
            nav_set_selected_folder(default_folder);
            ESP_LOGI(TAG, "Default folder: %d", default_folder);
        }
        
        // Play boot greetings
        announce_play_boot_greetings("/sdcard");
    }
    
    // Initialize encoder
    if (encoder_init()) {
        encoder_start_task(encoder_callback);
    }
    
    // Start audio task
    xTaskCreatePinnedToCore(audio_task, "audio", 8192, NULL, PRIORITY_AUDIO, 
                           &audio_task_handle, tskNO_AFFINITY);
    
    // Start headphone detection (if enabled)
    headphone_detect_start_task();
    
    ESP_LOGI(TAG, "Initialization complete. Entering main loop...");
    
    /* Main loop - handle button presses */
    while (1) {
        nav_state_t state = nav_get_state();
        
        // Play/Pause button
        if (button_is_pressed(&btn_play)) {
            // ALWAYS INTERRUPT AUDIO IMMEDIATELY (except when actually playing in FILE_VIEW)
            if (state != NAV_STATE_FILE_VIEW || !g_playing) {
                g_stop_flag = true;
            }
            
            ESP_LOGI(TAG, "Play button pressed (state=%d)", state);
            
            if (state == NAV_STATE_HOME) {
                // Same as encoder button at home
                if (sd_get_folder_count() > 0) {
                    nav_go_forward();
                    
                    int selected = nav_get_selected_folder();
                    const char *folder_path = sd_get_folder_path(selected);
                    const char *folder_name = get_folder_name(folder_path);
                    const char *announce_file = announce_get_folder_file(folder_name);
                    
                    if (announce_file) {
                        announce_check_and_play("/sdcard", NULL, announce_file,
                                              audio_task_handle, &g_stop_flag);
                    }
                    
                    ESP_LOGI(TAG, "HOME -> FOLDER_VIEW");
                }
                
            } else if (state == NAV_STATE_FOLDER_VIEW) {
                // Enter selected folder
                int selected = nav_get_selected_folder();
                const char *folder_path = sd_get_folder_path(selected);
                
                if (folder_path) {
                    sd_scan_subfolders(folder_path);
                    
                    if (sd_get_subfolder_count() > 0) {
                        nav_go_forward();
                        
                        int subfolder_idx = nav_get_selected_subfolder();
                        const char *subfolder_path = sd_get_subfolder_path(subfolder_idx);
                        const char *subfolder_name = get_folder_name(subfolder_path);
                        const char *announce_file = announce_get_folder_file(subfolder_name);
                        
                        if (announce_file) {
                            announce_check_and_play("/sdcard", NULL, announce_file,
                                                  audio_task_handle, &g_stop_flag);
                        }
                        
                        ESP_LOGI(TAG, "Entered folder (subfolders=%d)", sd_get_subfolder_count());
                    }
                }
                
            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                // Enter subfolder
                int selected = nav_get_selected_subfolder();
                const char *subfolder_path = sd_get_subfolder_path(selected);
                
                if (subfolder_path) {
                    const char *subfolder_name = get_folder_name(subfolder_path);
                    
                    const char *entering_announce = announce_get_entering_folder_file(subfolder_name);
                    if (entering_announce) {
                        announce_check_and_play("/sdcard", subfolder_path, entering_announce,
                                              audio_task_handle, &g_stop_flag);
                    }
                    
                    sd_scan_wav_files(subfolder_path);
                    
                    if (sd_get_wav_count() > 0) {
                        nav_go_forward();
                        
                        const char *filepath = sd_get_wav_path(0);
                        const char *filename = get_filename(filepath);
                        
                        char story_file[32];
                        if (announce_get_story_file(filename, story_file, sizeof(story_file))) {
                            char *folder_dir = get_folder_dir(filepath);
                            announce_check_and_play("/sdcard", folder_dir, story_file,
                                                  audio_task_handle, &g_stop_flag);
                            free(folder_dir);
                        }
                        
                        ESP_LOGI(TAG, "Entered subfolder (files=%d)", sd_get_wav_count());
                    }
                }
                
            } else if (state == NAV_STATE_FILE_VIEW) {
                // Play or pause
                if (!g_playing) {
                    int track = nav_get_selected_track();
                    if (track >= 0 && track < sd_get_wav_count()) {
                        g_playing_track = track;
                        g_playing = true;
                        g_pause = false;
                        g_stop_flag = true;
                        xTaskNotify(audio_task_handle, (uint32_t)(track + 1), eSetValueWithOverwrite);
                        ESP_LOGI(TAG, "Play: Start track %d", track);
                    }
                } else {
                    if (g_playing_track == nav_get_selected_track()) {
                        g_pause = !g_pause;
                        ESP_LOGI(TAG, "Play: Toggle pause");
                    } else {
                        int track = nav_get_selected_track();
                        g_playing_track = track;
                        g_pause = false;
                        g_stop_flag = true;
                        xTaskNotify(audio_task_handle, (uint32_t)(track + 1), eSetValueWithOverwrite);
                        ESP_LOGI(TAG, "Play: Switch to track %d", track);
                    }
                }
            }
        }
        
        // Home button - navigate back through levels
        if (button_is_pressed(&btn_home)) {
            // ALWAYS INTERRUPT AUDIO IMMEDIATELY
            g_stop_flag = true;
            
            ESP_LOGI(TAG, "Home button pressed (state=%d)", state);
            
            if (state == NAV_STATE_FILE_VIEW) {
                // Stop playback
                if (g_playing) {
                    g_playing = false;
                    g_pause = false;
                    g_stop_flag = true;
                }
                
                // Free WAV list and go back to subfolder view
                sd_free_wavs();
                nav_go_back(); // FILE_VIEW -> SUBFOLDER_VIEW
                
                // Announce current subfolder
                int subfolder_idx = nav_get_selected_subfolder();
                const char *subfolder_path = sd_get_subfolder_path(subfolder_idx);
                const char *subfolder_name = get_folder_name(subfolder_path);
                const char *announce_file = announce_get_folder_file(subfolder_name);
                
                if (announce_file) {
                    announce_check_and_play("/sdcard", NULL, announce_file,
                                          audio_task_handle, &g_stop_flag);
                }
                
                ESP_LOGI(TAG, "FILE_VIEW -> SUBFOLDER_VIEW");
                
            } else if (state == NAV_STATE_SUBFOLDER_VIEW) {
                // Free subfolder list and go back to folder view
                sd_free_subfolders();
                nav_go_back(); // SUBFOLDER_VIEW -> FOLDER_VIEW
                
                // Announce current main folder
                int folder_idx = nav_get_selected_folder();
                const char *folder_path = sd_get_folder_path(folder_idx);
                const char *folder_name = get_folder_name(folder_path);
                const char *announce_file = announce_get_folder_file(folder_name);
                
                if (announce_file) {
                    announce_check_and_play("/sdcard", NULL, announce_file,
                                          audio_task_handle, &g_stop_flag);
                }
                
                ESP_LOGI(TAG, "SUBFOLDER_VIEW -> FOLDER_VIEW");
                
            } else if (state == NAV_STATE_FOLDER_VIEW) {
                // Go back to home
                nav_go_back(); // FOLDER_VIEW -> HOME
                
                // Play home audio (ROTATE.WAV)
                announce_check_and_play("/sdcard", NULL, "ROTATE.WAV",
                                      audio_task_handle, &g_stop_flag);
                
                ESP_LOGI(TAG, "FOLDER_VIEW -> HOME");
            }
        }
        
        // Volume Up button
        if (button_is_pressed(&btn_vol_up)) {
            audio_volume_up();
        }
        
        // Volume Down button
        if (button_is_pressed(&btn_vol_down)) {
            audio_volume_down();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}