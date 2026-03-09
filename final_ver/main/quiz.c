/**
 * quiz.c
 * Quiz system
 *
 * SD structure (all files in same folder as stories):
 *   /sdcard/STORIES/MUHAMMAD/M1.WAV    <- story
 *   /sdcard/STORIES/MUHAMMAD/sm1.wav   <- question audio
 *   /sdcard/STORIES/MUHAMMAD/A.wav     <- option A audio
 *   /sdcard/STORIES/MUHAMMAD/B.wav     <- option B audio
 *   /sdcard/STORIES/MUHAMMAD/wrm1.wav  <- wrong answer audio
 *   /sdcard/STORIES/MUHAMMAD/crrm1.wav <- correct answer audio
 *
 * Root-level:
 *   /sdcard/correct.wav  <- "correct!" feedback
 *   /sdcard/wrong.wav    <- "wrong!" feedback
 *
 * Answer mapping: A = wrong, B = correct
 * (Change QUIZ_CORRECT_OPT below to flip this)
 */

#include "quiz.h"
#include "audio.h"
#include "announcements.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>

static const char *TAG = "QUIZ";

#define STORY_DIR_MAX  256
#define PATH_BUF       512

static char story_dir[STORY_DIR_MAX] = {0};
static int  story_num  = 0;
static quiz_option_t selected_opt = QUIZ_OPT_A;

/* ------------------------------------------------------------------ */
/*  quiz_story_number: M1.WAV / m1.wav -> 1 (case insensitive)        */
/* ------------------------------------------------------------------ */
int quiz_story_number(const char *filename) {
    if (!filename) return 0;
    if (toupper((unsigned char)filename[0]) != 'M') return 0;
    if (filename[1] < '1' || filename[1] > '9') return 0;
    const char *dot = strrchr(filename, '.');
    if (!dot || strcasecmp(dot, ".wav") != 0) return 0;
    return filename[1] - '0';
}

/* ------------------------------------------------------------------ */
/*  quiz_exists_for_story                                               */
/* ------------------------------------------------------------------ */
bool quiz_exists_for_story(const char *story_filepath) {
    if (!story_filepath) return false;

    const char *fname = strrchr(story_filepath, '/');
    fname = fname ? fname + 1 : story_filepath;

    int n = quiz_story_number(fname);
    if (n == 0) return false;

    const char *last = strrchr(story_filepath, '/');
    if (!last) return false;
    size_t dlen = (size_t)(last - story_filepath);
    if (dlen >= STORY_DIR_MAX) return false;

    char dir[STORY_DIR_MAX];
    memcpy(dir, story_filepath, dlen);
    dir[dlen] = '\0';

    char qpath[PATH_BUF];
    snprintf(qpath, sizeof(qpath), "%s/sm%d.wav", dir, n);
    bool exists = (access(qpath, F_OK) == 0);
    ESP_LOGI(TAG, "Quiz check m%d: %s -> %s", n, qpath, exists ? "FOUND" : "not found");
    return exists;
}

/* ------------------------------------------------------------------ */
/*  quiz_begin                                                          */
/* ------------------------------------------------------------------ */
bool quiz_begin(const char *story_filepath) {
    if (!story_filepath) return false;

    const char *fname = strrchr(story_filepath, '/');
    fname = fname ? fname + 1 : story_filepath;

    story_num = quiz_story_number(fname);
    if (story_num == 0) return false;

    const char *last = strrchr(story_filepath, '/');
    if (!last) return false;
    size_t dlen = (size_t)(last - story_filepath);
    if (dlen >= STORY_DIR_MAX) return false;

    memcpy(story_dir, story_filepath, dlen);
    story_dir[dlen] = '\0';

    selected_opt = QUIZ_OPT_A;  /* Start on A */
    ESP_LOGI(TAG, "Quiz begin: story=%d dir=%s", story_num, story_dir);
    return true;
}

/* ------------------------------------------------------------------ */
/*  quiz_play_question: play sm<N>.wav                                  */
/* ------------------------------------------------------------------ */
void quiz_play_question(TaskHandle_t th, volatile bool *sf) {
    char path[PATH_BUF];
    snprintf(path, sizeof(path), "%s/sm%d.wav", story_dir, story_num);
    ESP_LOGI(TAG, "Question: %s", path);
    if (access(path, F_OK) == 0)
        announce_request(path, th, sf);
    else
        ESP_LOGE(TAG, "Question file missing: %s", path);
}

/* ------------------------------------------------------------------ */
/*  quiz_play_options_intro                                             */
/*  After question: play A.wav then B.wav so user knows the options    */
/* ------------------------------------------------------------------ */
static void play_sync(const char *path, TaskHandle_t th, volatile bool *sf) {
    if (!path || access(path, F_OK) != 0) {
        ESP_LOGW(TAG, "File missing: %s", path ? path : "null");
        return;
    }
    announce_request(path, th, sf);
    /* Wait until announce system picks it up and plays it */
    vTaskDelay(pdMS_TO_TICKS(150));
    char buf[512];
    int  wait = 0;
    while (announce_get_pending(buf, sizeof(buf)) && wait < 100) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait++;
    }
    vTaskDelay(pdMS_TO_TICKS(400));
}

void quiz_play_options_intro(TaskHandle_t th, volatile bool *sf) {
    /* crrm<N>.wav = correct option (A), wrm<N>.wav = wrong option (B) */
    char crr_path[PATH_BUF], wrm_path[PATH_BUF];
    snprintf(crr_path, sizeof(crr_path), "%s/crrm%d.wav", story_dir, story_num);
    snprintf(wrm_path, sizeof(wrm_path), "%s/wrm%d.wav",  story_dir, story_num);
    ESP_LOGI(TAG, "Options intro: %s then %s", crr_path, wrm_path);

    /* Direct audio_play_file - bypass announce_request to avoid notification loop */
    if (access(crr_path, F_OK) == 0) {
        *sf = false;
        ESP_LOGI(TAG, "Playing crrm%d.wav (option A)", story_num);
        audio_play_file(crr_path, sf, NULL);
        vTaskDelay(pdMS_TO_TICKS(400));
    } else {
        ESP_LOGE(TAG, "crrm%d.wav missing: %s", story_num, crr_path);
    }

    if (!(*sf) && access(wrm_path, F_OK) == 0) {
        *sf = false;
        ESP_LOGI(TAG, "Playing wrm%d.wav (option B)", story_num);
        audio_play_file(wrm_path, sf, NULL);
        vTaskDelay(pdMS_TO_TICKS(300));
    } else if (!(*sf)) {
        ESP_LOGE(TAG, "wrm%d.wav missing: %s", story_num, wrm_path);
    }
}

/* ------------------------------------------------------------------ */
/*  quiz_announce_option: play A.wav or B.wav on encoder rotate        */
/* ------------------------------------------------------------------ */
void quiz_announce_option(quiz_option_t opt, TaskHandle_t th, volatile bool *sf) {
    char path[PATH_BUF];
    /* QUIZ_OPT_A = crrm (correct option), QUIZ_OPT_B = wrm (wrong option) */
    if (opt == QUIZ_OPT_A)
        snprintf(path, sizeof(path), "%s/crrm%d.wav", story_dir, story_num);
    else
        snprintf(path, sizeof(path), "%s/wrm%d.wav",  story_dir, story_num);

    ESP_LOGI(TAG, "Option %s: %s", opt == QUIZ_OPT_A ? "A(crrm)" : "B(wrm)", path);
    if (access(path, F_OK) == 0)
        announce_request(path, th, sf);
    else
        ESP_LOGW(TAG, "Option file missing: %s", path);
}

/* ------------------------------------------------------------------ */
/*  Selection navigation                                                */
/* ------------------------------------------------------------------ */
quiz_option_t quiz_get_selected(void) { return selected_opt; }

void quiz_next_option(void) {
    selected_opt = (selected_opt == QUIZ_OPT_A) ? QUIZ_OPT_B : QUIZ_OPT_A;
    ESP_LOGI(TAG, "Selected: %s", selected_opt == QUIZ_OPT_A ? "A" : "B");
}

void quiz_prev_option(void) { quiz_next_option(); }  /* only 2 options = same */

/* ------------------------------------------------------------------ */
/*  quiz_submit                                                         */
/* ------------------------------------------------------------------ */
quiz_result_t quiz_submit(void) {
    /* crrm = correct option (QUIZ_OPT_A), wrm = wrong option (QUIZ_OPT_B) */
    quiz_result_t result = (selected_opt == QUIZ_OPT_A)
                           ? QUIZ_RESULT_CORRECT
                           : QUIZ_RESULT_WRONG;
    ESP_LOGI(TAG, "Submit: selected=%s -> %s",
             selected_opt == QUIZ_OPT_A ? "crrm(A)" : "wrm(B)",
             result == QUIZ_RESULT_CORRECT ? "CORRECT" : "WRONG");
    return result;
}

/* ------------------------------------------------------------------ */
/*  quiz_play_result                                                    */
/* ------------------------------------------------------------------ */
void quiz_play_result(quiz_result_t result, TaskHandle_t th, volatile bool *sf) {
    const char *filename = (result == QUIZ_RESULT_CORRECT)
                           ? "correct.wav" : "wrong.wav";
    ESP_LOGI(TAG, "Result: %s -> looking for %s",
             result == QUIZ_RESULT_CORRECT ? "CORRECT" : "WRONG", filename);

    /* Try /sdcard/announcements/ first, then /sdcard/ root */
    char path[PATH_BUF];
    snprintf(path, sizeof(path), "/sdcard/announcements/%s", filename);
    if (access(path, F_OK) == 0) {
        ESP_LOGI(TAG, "Found: %s", path);
        announce_request(path, th, sf);
        return;
    }
    snprintf(path, sizeof(path), "/sdcard/%s", filename);
    if (access(path, F_OK) == 0) {
        ESP_LOGI(TAG, "Found: %s", path);
        announce_request(path, th, sf);
        return;
    }
    ESP_LOGW(TAG, "Result file missing: %s (checked announcements/ and root)", filename);
}

/* ------------------------------------------------------------------ */
/*  quiz_init / quiz_end                                                */
/* ------------------------------------------------------------------ */
void quiz_init(void) {
    memset(story_dir, 0, sizeof(story_dir));
    story_num    = 0;
    selected_opt = QUIZ_OPT_A;
    ESP_LOGI(TAG, "Quiz system initialized");
}

void quiz_end(void) {
    memset(story_dir, 0, sizeof(story_dir));
    story_num    = 0;
    selected_opt = QUIZ_OPT_A;
    ESP_LOGI(TAG, "Quiz ended");
}
