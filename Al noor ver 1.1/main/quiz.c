/**
 * quiz.c
 * Quiz system for Noor Audio Player
 *
 * Integration notes for main.c firmware:
 * ----------------------------------------
 * All quiz audio functions are called FROM WITHIN audio_task.
 * They must use audio_play_file() directly — never announce_request() —
 * because announce_request() sends NOTIFY_ANNOUNCE_BIT back to the same
 * task that is currently executing, causing a re-entrant notify that would
 * only be processed after the current handler returns (i.e. too late, or
 * out of order).
 *
 * Call sequence from audio_task (NOTIFY_QUIZ_BIT handler):
 *   1. story ends naturally (g_stop_flag == false, was_quiz_story == true)
 *      -> audio_task sends NOTIFY_QUIZ_BIT to itself
 *   2. NOTIFY_QUIZ_BIT fires
 *      -> audio_task calls quiz_begin() to set up state
 *      -> audio_task calls quiz_play_question()  — BLOCKING, direct I2S
 *      -> audio_task calls quiz_play_options_intro() — BLOCKING, direct I2S
 *         Options intro plays crrm<N>.wav and wrm<N>.wav in RANDOM ORDER
 *         so the correct answer isn't always "first option heard".
 *         g_quiz_option_order indicates which was played first.
 *      -> sets g_quiz_active = true, awaits encoder input
 *   3. User rotates encoder
 *      -> encoder_callback calls quiz_announce_option()
 *         quiz_announce_option() calls announce_request() -> NOTIFY_ANNOUNCE_BIT
 *      -> audio_task plays the option file (crrm or wrm for selected slot)
 *   4. User presses play/encoder button
 *      -> main.c quiz_do_submit() -> quiz_submit() + quiz_play_result()
 *         quiz_play_result() plays correct.wav or wrong.wav directly (blocking)
 *         then calls quiz_end()
 *
 * Randomisation:
 *   On each quiz, a coin-flip (esp_random()) decides whether option-slot A
 *   shows crrm (correct) or wrm (wrong) first.  The mapping is stored in
 *   g_quiz_crr_is_A so submit logic always knows which slot is correct.
 *
 * SD card layout (all files in same folder as the story):
 *   /sdcard/STORIES/MUHAMMAD/m1.wav     <- story
 *   /sdcard/STORIES/MUHAMMAD/sm1.wav    <- question audio
 *   /sdcard/STORIES/MUHAMMAD/crrm1.wav  <- correct answer audio
 *   /sdcard/STORIES/MUHAMMAD/wrm1.wav   <- wrong answer audio
 *   (same pattern for m2..m7 / sm2..sm7 / crrm2..crrm7 / wrm2..wrm7)
 *
 * Answer mapping:
 *   g_quiz_crr_is_A == true  -> QUIZ_OPT_A = correct (crrm), QUIZ_OPT_B = wrong (wrm)
 *   g_quiz_crr_is_A == false -> QUIZ_OPT_A = wrong  (wrm),  QUIZ_OPT_B = correct (crrm)
 *
 * Result audio search order (quiz_play_result):
 *   1. /sdcard/ANNOUN~1/correct.wav or wrong.wav  (via announce_check_and_play)
 *   2. /sdcard/correct.wav or wrong.wav            (root fallback)
 */

#include "quiz.h"
#include "audio.h"
#include "announcements.h"
#include "esp_log.h"
#include "esp_random.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>

static const char *TAG = "QUIZ";

#define STORY_DIR_MAX  256
#define PATH_BUF       512

/* ---- State ---- */
static char story_dir[STORY_DIR_MAX] = {0};
static int  story_num  = 0;

/*
 * Option slot mapping.
 * g_quiz_crr_is_A == true  : slot A = crrm (correct), slot B = wrm (wrong)
 * g_quiz_crr_is_A == false : slot A = wrm  (wrong),   slot B = crrm (correct)
 * Randomised each time quiz_begin() is called.
 */
static bool          g_quiz_crr_is_A = true;
static quiz_option_t selected_opt    = QUIZ_OPT_A;

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/** Return the path for the correct-answer file (crrm<N>.wav). */
static void _crr_path(char *buf, size_t sz) {
    snprintf(buf, sz, "%s/crrm%d.wav", story_dir, story_num);
}

/** Return the path for the wrong-answer file (wrm<N>.wav). */
static void _wrm_path(char *buf, size_t sz) {
    snprintf(buf, sz, "%s/wrm%d.wav", story_dir, story_num);
}

/** Path for whichever file is mapped to slot A (depends on randomisation). */
static void _opt_a_path(char *buf, size_t sz) {
    if (g_quiz_crr_is_A) _crr_path(buf, sz);
    else                  _wrm_path(buf, sz);
}

/** Path for whichever file is mapped to slot B. */
static void _opt_b_path(char *buf, size_t sz) {
    if (g_quiz_crr_is_A) _wrm_path(buf, sz);
    else                  _crr_path(buf, sz);
}

/* ------------------------------------------------------------------ */
/*  quiz_story_number                                                   */
/*  m1.wav / M1.WAV -> 1   (case insensitive)                          */
/*  Returns 0 if not a quiz story filename.                             */
/* ------------------------------------------------------------------ */
int quiz_story_number(const char *filename) {
    if (!filename) return 0;
    if (toupper((unsigned char)filename[0]) != 'M') return 0;
    /* digit 1-7 in position 1 */
    if (filename[1] < '1' || filename[1] > '7') return 0;
    const char *dot = strrchr(filename, '.');
    if (!dot || strcasecmp(dot, ".wav") != 0) return 0;
    /* make sure it's exactly "M<digit>.wav" — not "MS1.wav" etc. */
    /* position 2 must be the dot or checked via dot pointer */
    if ((dot - filename) != 2) return 0;
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
/*  Sets up quiz state for the given story filepath.                    */
/*  Randomises which option slot (A/B) maps to correct/wrong.          */
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

    /* Randomise option order: 50/50 whether slot A is correct or wrong */
    g_quiz_crr_is_A = (esp_random() & 1) ? true : false;
    selected_opt    = QUIZ_OPT_A;

    ESP_LOGI(TAG, "Quiz begin: story=%d dir=%s  slot_A=%s",
             story_num, story_dir,
             g_quiz_crr_is_A ? "CORRECT(crrm)" : "WRONG(wrm)");
    return true;
}

/* ------------------------------------------------------------------ */
/*  quiz_play_question                                                  */
/*  Plays sm<N>.wav DIRECTLY (blocking) from inside audio_task.        */
/*  Called from the NOTIFY_QUIZ_BIT handler — never from main loop.    */
/* ------------------------------------------------------------------ */
void quiz_play_question(volatile bool *sf) {
    char path[PATH_BUF];
    snprintf(path, sizeof(path), "%s/sm%d.wav", story_dir, story_num);
    ESP_LOGI(TAG, "Playing question: %s", path);
    if (access(path, F_OK) == 0) {
        *sf = false;
        audio_play_file(path, sf, NULL);
        vTaskDelay(pdMS_TO_TICKS(400));
    } else {
        ESP_LOGE(TAG, "Question file missing: %s", path);
    }
}

/* ------------------------------------------------------------------ */
/*  quiz_play_options_intro                                             */
/*  Plays option A then option B (order randomised by quiz_begin).     */
/*  BLOCKING — uses audio_play_file() directly.                        */
/*  Called from INSIDE audio_task (NOTIFY_QUIZ_BIT handler).           */
/* ------------------------------------------------------------------ */
void quiz_play_options_intro(volatile bool *sf) {
    char path_a[PATH_BUF], path_b[PATH_BUF];
    _opt_a_path(path_a, sizeof(path_a));
    _opt_b_path(path_b, sizeof(path_b));

    ESP_LOGI(TAG, "Options intro: A=%s  B=%s", path_a, path_b);

    /* Play slot A */
    if (access(path_a, F_OK) == 0) {
        *sf = false;
        ESP_LOGI(TAG, "Playing option A: %s", path_a);
        audio_play_file(path_a, sf, NULL);
        vTaskDelay(pdMS_TO_TICKS(400));
    } else {
        ESP_LOGE(TAG, "Option A file missing: %s", path_a);
    }

    /* Play slot B — only if not interrupted */
    if (!(*sf) && access(path_b, F_OK) == 0) {
        *sf = false;
        ESP_LOGI(TAG, "Playing option B: %s", path_b);
        audio_play_file(path_b, sf, NULL);
        vTaskDelay(pdMS_TO_TICKS(300));
    } else if (!(*sf)) {
        ESP_LOGE(TAG, "Option B file missing: %s", path_b);
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

    if (opt == QUIZ_OPT_A)
        _opt_a_path(path, sizeof(path));
    else
        _opt_b_path(path, sizeof(path));

    ESP_LOGI(TAG, "Announce option %s: %s",
             opt == QUIZ_OPT_A ? "A" : "B", path);

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
    ESP_LOGI(TAG, "Selected option: %s (%s)",
             selected_opt == QUIZ_OPT_A ? "A" : "B",
             (selected_opt == QUIZ_OPT_A) == g_quiz_crr_is_A ? "correct" : "wrong");
}

void quiz_prev_option(void) { quiz_next_option(); }  /* only 2 options */

/* ------------------------------------------------------------------ */
/*  quiz_submit                                                         */
/*  Returns CORRECT if the selected slot maps to crrm, else WRONG.     */
/* ------------------------------------------------------------------ */
quiz_result_t quiz_submit(void) {
    bool selected_correct = (selected_opt == QUIZ_OPT_A) ? g_quiz_crr_is_A : !g_quiz_crr_is_A;
    quiz_result_t result = selected_correct ? QUIZ_RESULT_CORRECT : QUIZ_RESULT_WRONG;

    ESP_LOGI(TAG, "Submit: slot=%s  crr_is_A=%d -> %s",
             selected_opt == QUIZ_OPT_A ? "A" : "B",
             g_quiz_crr_is_A,
             result == QUIZ_RESULT_CORRECT ? "CORRECT" : "WRONG");
    return result;
}

/* ------------------------------------------------------------------ */
/*  quiz_play_result                                                    */
/*  Plays correct.wav or wrong.wav DIRECTLY (blocking).                */
/*  Called from INSIDE audio_task after quiz_submit().                 */
/*                                                                      */
/*  Search order:                                                       */
/*   1. /sdcard/ANNOUN~1/correct.wav or wrong.wav  (announcements dir) */
/*   2. /sdcard/correct.wav or wrong.wav           (SD root)            */
/* ------------------------------------------------------------------ */
void quiz_play_result(quiz_result_t result, volatile bool *sf) {
    bool correct = (result == QUIZ_RESULT_CORRECT);
    const char *generic = correct ? "correct.wav" : "wrong.wav";

    ESP_LOGI(TAG, "Playing result: %s -> %s",
             correct ? "CORRECT" : "WRONG", generic);

    /* announce_check_and_play searches the announcements dir automatically.
     * Since we are inside audio_task and need blocking playback, we resolve
     * the path ourselves and call audio_play_file() directly. */

    /* Try announcements dir via announce_get_pending trick:
     * We'll use announce_check_and_play with a dummy task to resolve path,
     * then drain the pending queue and play directly. */

    /* Simpler: try known paths in order */
    const char *search_dirs[] = {
        "/sdcard/ANNOUN~1",
        "/sdcard/announcements",
        "/sdcard",
        NULL
    };

    for (int i = 0; search_dirs[i]; i++) {
        char path[PATH_BUF];
        snprintf(path, sizeof(path), "%s/%s", search_dirs[i], generic);
        if (access(path, F_OK) == 0) {
            ESP_LOGI(TAG, "Result audio: %s", path);
            *sf = false;
            audio_play_file(path, sf, NULL);
            return;
        }
    }

    ESP_LOGW(TAG, "No result audio found for: %s", generic);
}

/* ------------------------------------------------------------------ */
/*  quiz_init / quiz_end                                                */
/* ------------------------------------------------------------------ */
void quiz_init(void) {
    memset(story_dir, 0, sizeof(story_dir));
    story_num       = 0;
    selected_opt    = QUIZ_OPT_A;
    g_quiz_crr_is_A = true;
    ESP_LOGI(TAG, "Quiz system initialized");
}

void quiz_end(void) {
    memset(story_dir, 0, sizeof(story_dir));
    story_num       = 0;
    selected_opt    = QUIZ_OPT_A;
    g_quiz_crr_is_A = true;
    ESP_LOGI(TAG, "Quiz ended");
}

/* ------------------------------------------------------------------ */
/*  quiz_is_crr_slot_a                                                  */
/*  Returns true when slot A is the correct answer this round.          */
/*  Used by display/LED code if needed.                                 */
/* ------------------------------------------------------------------ */
bool quiz_is_crr_slot_a(void) {
    return g_quiz_crr_is_A;
}
