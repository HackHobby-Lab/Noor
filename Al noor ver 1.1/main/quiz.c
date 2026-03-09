/**
 * quiz.c
 * Quiz system for Noor Audio Player
 *
 * Integration notes for main2.c firmware:
 * ----------------------------------------
 * All quiz audio functions are called FROM WITHIN audio_task.
 * They must use audio_play_file() directly — never announce_request() —
 * because announce_request() sends NOTIFY_ANNOUNCE_BIT back to the same
 * task that is currently executing, causing a re-entrant notify that would
 * only be processed after the current handler returns (i.e. too late, or
 * out of order).
 *
 * Call sequence from audio_task:
 *   1. story ends naturally (g_stop_flag == false)
 *      -> main.c try_start_quiz() -> quiz_begin() + quiz_play_question()
 *         quiz_play_question() calls announce_request() which is fine here
 *         because it wakes audio_task for the NEXT iteration.
 *   2. NOTIFY_ANNOUNCE_BIT fires for the question file
 *      -> audio_task plays it via audio_play_file()
 *      -> was_question==true && !g_stop_flag -> sends NOTIFY_QUIZ_OPTS_BIT
 *   3. NOTIFY_QUIZ_OPTS_BIT fires
 *      -> audio_task calls quiz_play_options_intro() — BLOCKING, direct I2S
 *      -> sets g_quiz_options_done = true
 *   4. User rotates encoder
 *      -> encoder_callback calls quiz_announce_option()
 *         quiz_announce_option() calls announce_request() -> NOTIFY_ANNOUNCE_BIT
 *      -> audio_task plays the option file
 *   5. User presses play/encoder button
 *      -> main.c quiz_do_submit() -> quiz_submit() + quiz_play_result()
 *         quiz_play_result() calls announce_request() -> NOTIFY_ANNOUNCE_BIT
 *      -> audio_task plays result file
 *
 * SD card layout (all files in same folder as the story):
 *   /sdcard/STORIES/MUHAMMAD/M1.WAV     <- story
 *   /sdcard/STORIES/MUHAMMAD/sm1.wav    <- question audio
 *   /sdcard/STORIES/MUHAMMAD/crrm1.wav  <- correct answer audio (option A)
 *   /sdcard/STORIES/MUHAMMAD/wrm1.wav   <- wrong answer audio  (option B)
 *
 * Answer mapping: QUIZ_OPT_A = correct (crrm), QUIZ_OPT_B = wrong (wrm)
 *
 * Result audio search order (quiz_play_result):
 *   1. story_dir/crrm<N>.wav or story_dir/wrm<N>.wav (story-specific)
 *   2. /sdcard/ANNOUN~1/correct.wav or wrong.wav
 *   3. /sdcard/correct.wav or wrong.wav
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
/*  quiz_story_number: M1.WAV / m1.wav -> 1  (case insensitive)       */
/*  Returns 0 if not a quiz story filename.                            */
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
/*  Checks whether sm<N>.wav exists in the same folder as the story.   */
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

    selected_opt = QUIZ_OPT_A;
    ESP_LOGI(TAG, "Quiz begin: story=%d dir=%s", story_num, story_dir);
    return true;
}

/* ------------------------------------------------------------------ */
/*  quiz_play_question                                                  */
/*  Queues sm<N>.wav via announce_request so audio_task plays it in    */
/*  its NEXT notification cycle (called from main loop context, not    */
/*  from inside audio_task).                                            */
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
/*  Plays crrm<N>.wav (option A) then wrm<N>.wav (option B).           */
/*  BLOCKING — uses audio_play_file() directly.                        */
/*  Called from INSIDE audio_task (NOTIFY_QUIZ_OPTS_BIT handler).      */
/* ------------------------------------------------------------------ */
void quiz_play_options_intro(TaskHandle_t th, volatile bool *sf) {
    char crr_path[PATH_BUF], wrm_path[PATH_BUF];
    snprintf(crr_path, sizeof(crr_path), "%s/crrm%d.wav", story_dir, story_num);
    snprintf(wrm_path, sizeof(wrm_path), "%s/wrm%d.wav",  story_dir, story_num);

    ESP_LOGI(TAG, "Options intro: %s then %s", crr_path, wrm_path);

    /* Play option A (correct explanation) */
    if (access(crr_path, F_OK) == 0) {
        *sf = false;
        ESP_LOGI(TAG, "Playing crrm%d.wav (option A)", story_num);
        audio_play_file(crr_path, sf, NULL);
        vTaskDelay(pdMS_TO_TICKS(400));
    } else {
        ESP_LOGE(TAG, "crrm%d.wav missing: %s", story_num, crr_path);
    }

    /* Play option B (wrong explanation) — only if not interrupted */
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
/*  quiz_announce_option                                                */
/*  Called from encoder_callback (NOT from audio_task).                */
/*  Uses announce_request() to queue the file — audio_task will play   */
/*  it on the next NOTIFY_ANNOUNCE_BIT.                                 */
/* ------------------------------------------------------------------ */
void quiz_announce_option(quiz_option_t opt, TaskHandle_t th, volatile bool *sf) {
    char path[PATH_BUF];

    /* QUIZ_OPT_A = correct option (crrm), QUIZ_OPT_B = wrong option (wrm) */
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
    ESP_LOGI(TAG, "Selected: %s", selected_opt == QUIZ_OPT_A ? "A(crrm)" : "B(wrm)");
}

void quiz_prev_option(void) { quiz_next_option(); }  /* only 2 options */

/* ------------------------------------------------------------------ */
/*  quiz_submit                                                         */
/* ------------------------------------------------------------------ */
quiz_result_t quiz_submit(void) {
    quiz_result_t result = (selected_opt == QUIZ_OPT_A)
                           ? QUIZ_RESULT_CORRECT
                           : QUIZ_RESULT_WRONG;
    ESP_LOGI(TAG, "Submit: selected=%s -> %s",
             selected_opt == QUIZ_OPT_A ? "A(crrm)" : "B(wrm)",
             result == QUIZ_RESULT_CORRECT ? "CORRECT" : "WRONG");
    return result;
}

/* ------------------------------------------------------------------ */
/*  quiz_play_result                                                    */
/*  Queues the result audio via announce_request.                      */
/*  Called from main.c quiz_do_submit() (NOT from audio_task).         */
/*                                                                      */
/*  Search order:                                                       */
/*   1. story_dir/crrm<N>.wav (correct) or story_dir/wrm<N>.wav (wrong)*/
/*      These are the story-specific full explanations.                 */
/*   2. /sdcard/ANNOUN~1/correct.wav or wrong.wav                      */
/*   3. /sdcard/correct.wav or wrong.wav                               */
/* ------------------------------------------------------------------ */
void quiz_play_result(quiz_result_t result, TaskHandle_t th, volatile bool *sf) {
    bool correct = (result == QUIZ_RESULT_CORRECT);
    ESP_LOGI(TAG, "Result: %s", correct ? "CORRECT" : "WRONG");

    /* 1. Story-specific explanation (same files used in options intro) */
    char story_path[PATH_BUF];
    if (correct)
        snprintf(story_path, sizeof(story_path), "%s/crrm%d.wav", story_dir, story_num);
    else
        snprintf(story_path, sizeof(story_path), "%s/wrm%d.wav",  story_dir, story_num);

    if (access(story_path, F_OK) == 0) {
        ESP_LOGI(TAG, "Result file (story-specific): %s", story_path);
        announce_request(story_path, th, sf);
        return;
    }

    /* 2 & 3. Generic correct.wav / wrong.wav — search via announcement system */
    const char *generic = correct ? "correct.wav" : "wrong.wav";
    if (announce_check_and_play(NULL, NULL, generic, th, sf)) {
        ESP_LOGI(TAG, "Result file (generic): %s", generic);
        return;
    }

    /* Fallback: try /sdcard root directly */
    char root_path[PATH_BUF];
    snprintf(root_path, sizeof(root_path), "/sdcard/%s", generic);
    if (access(root_path, F_OK) == 0) {
        ESP_LOGI(TAG, "Result file (root): %s", root_path);
        announce_request(root_path, th, sf);
        return;
    }

    ESP_LOGW(TAG, "No result audio found for %s", generic);
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
