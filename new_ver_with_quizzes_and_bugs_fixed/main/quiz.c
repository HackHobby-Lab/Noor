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
 *         Options intro plays crr{prefix}{N}_{Q}.wav and wr{prefix}{N}_{Q}.wav
 *         in RANDOM ORDER so the correct answer isn't always "first option heard".
 *      -> sets g_quiz_active = true, awaits encoder input
 *   3. User rotates encoder
 *      -> encoder_callback calls quiz_announce_option()
 *         quiz_announce_option() calls announce_request() -> NOTIFY_ANNOUNCE_BIT
 *      -> audio_task plays the option file (crr or wr for selected slot)
 *   4. User presses play/encoder button
 *      -> main.c quiz_do_submit() -> quiz_submit() + quiz_play_result()
 *         quiz_play_result() plays correct.wav or wrong.wav directly (blocking)
 *         then calls quiz_end()
 *
 * Randomisation:
 *   On each quiz, a coin-flip (esp_random()) decides whether option-slot A
 *   shows crr (correct) or wr (wrong) first.  The mapping is stored in
 *   g_quiz_crr_is_A so submit logic always knows which slot is correct.
 *
 * SD card layout — applies to ALL prophet subfolders AND the TAWID folder.
 * File naming convention: s{prefix}{N}_{Q}.wav / crr{prefix}{N}_{Q}.wav / wr{prefix}{N}_{Q}.wav
 * where {prefix} matches the story filename prefix, {N}=story number (1-10 for prophets,
 * 1-20 for TAWID), {Q}=quiz number (1-3 for prophets, 1-5 for TAWID).
 *
 * Examples:
 *   /sdcard/STORIES/ADAM/a1.wav        <- story
 *   /sdcard/STORIES/ADAM/sa1_1.wav     <- quiz 1 question
 *   /sdcard/STORIES/ADAM/crra1_1.wav   <- quiz 1 correct answer
 *   /sdcard/STORIES/ADAM/wra1_1.wav    <- quiz 1 wrong answer
 *   ... sa1_2/crra1_2/wra1_2 ... sa1_3/crra1_3/wra1_3  (quizzes 2 & 3)
 *
 *   /sdcard/STORIES/MUHAMMAD/m1.wav      <- story
 *   /sdcard/STORIES/MUHAMMAD/sm1_1.wav   <- quiz 1 question
 *   /sdcard/STORIES/MUHAMMAD/crrm1_1.wav <- quiz 1 correct answer
 *   /sdcard/STORIES/MUHAMMAD/wrm1_1.wav  <- quiz 1 wrong answer
 *   ... sm1_2/crrm1_2/wrm1_2 ... sm1_3/crrm1_3/wrm1_3 (quizzes 2 & 3)
 *
 * Prophet prefix table:
 *   Adam=a  Daoud=d  Ibrahim=ib  Ismaeel=is  Issa=i
 *   Moussa=ms  Muhammad=m  Nuh=n  Yaqoob=yq  Yousuf=y
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
static char story_prefix[8]         = {0};  /* e.g. "a","d","ib","is","i","ms","m","n","yq","y","t" (TAWID=t) */
static int  story_num  = 0;
static int  quiz_index = 1;   /* current quiz round: 1 .. quiz_count */

/*
 * quiz_count is set at runtime by quiz_begin():
 *   3 for all prophet story folders (ADAM, DAOUD, … YOUSUF)
 *   5 for the TAWID folder (20 lessons × 5 quizzes each)
 */
static int  quiz_count = 3;

/*
 * Option slot mapping.
 * g_quiz_crr_is_A == true  : slot A = crr (correct), slot B = wr (wrong)
 * g_quiz_crr_is_A == false : slot A = wr  (wrong),   slot B = crr (correct)
 * Randomised each time quiz_begin() is called.
 */
static bool          g_quiz_crr_is_A = true;
static quiz_option_t selected_opt    = QUIZ_OPT_A;

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/**
 * parse_story_filename — extracts the alphabetic prefix and story number
 * from a story filename (e.g. "ib3.wav" -> prefix="ib", num=3).
 *
 * Rules:
 *   - Must end with .wav (case-insensitive).
 *   - Must NOT contain an underscore (those are quiz support files).
 *   - Prefix = leading alphabetic characters (lowercased).
 *   - Num    = digits between prefix and '.', range 1-20 (1-10 for prophets, 1-20 for TAWID).
 *
 * Returns true on success, false if filename doesn't match.
 */
static bool parse_story_filename(const char *filename,
                                  char *prefix, size_t prefix_sz,
                                  int  *num_out)
{
    if (!filename || !prefix || !num_out) return false;

    /* Reject quiz support files that contain an underscore */
    if (strchr(filename, '_')) return false;

    const char *dot = strrchr(filename, '.');
    if (!dot || strcasecmp(dot, ".wav") != 0) return false;

    /* Find where the alphabetic prefix ends (first digit) */
    size_t i = 0;
    while (filename[i] && !isdigit((unsigned char)filename[i]) && filename[i] != '.') i++;

    /* Must have at least one letter prefix and at least one digit */
    if (i == 0 || !isdigit((unsigned char)filename[i])) return false;
    if (i >= prefix_sz) return false;

    /* Copy and lowercase the prefix */
    for (size_t j = 0; j < i; j++)
        prefix[j] = (char)tolower((unsigned char)filename[j]);
    prefix[i] = '\0';

    /* Parse the numeric part between prefix end and dot */
    ptrdiff_t dot_pos  = dot - filename;
    ptrdiff_t num_start = (ptrdiff_t)i;
    size_t    num_len   = (size_t)(dot_pos - num_start);
    if (num_len == 0 || num_len > 3) return false;

    char numstr[4] = {0};
    memcpy(numstr, filename + num_start, num_len);
    int n = atoi(numstr);
    if (n < 1 || n > 20) return false;  /* 1-10 for prophets, 1-20 for TAWID */

    *num_out = n;
    return true;
}

/** Return the path for the correct-answer file (crr{prefix}{N}_{Q}.wav). */
static void _crr_path(char *buf, size_t sz) {
    snprintf(buf, sz, "%s/crr%s%d_%d.wav", story_dir, story_prefix, story_num, quiz_index);
}

/** Return the path for the wrong-answer file (wr{prefix}{N}_{Q}.wav). */
static void _wrm_path(char *buf, size_t sz) {
    snprintf(buf, sz, "%s/wr%s%d_%d.wav", story_dir, story_prefix, story_num, quiz_index);
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
/*  Returns the story/lesson number N from any story filename.         */
/*  Works for prophets (1-10): a1.wav->1, ib3.wav->3, yq10.wav->10.   */
/*  Works for TAWID (1-20):    T5.wav->5, T20.wav->20.                 */
/*  Returns 0 if the filename is not a valid story file.               */
/* ------------------------------------------------------------------ */
int quiz_story_number(const char *filename) {
    char prefix[8];
    int  num = 0;
    if (!parse_story_filename(filename, prefix, sizeof(prefix), &num)) return 0;
    return num;
}

/* ------------------------------------------------------------------ */
/*  quiz_exists_for_story                                               */
/*  Checks whether s{prefix}{N}_1.wav exists alongside the story.      */
/*  Works for all prophets (Adam, Daoud, Ibrahim, ... Yousuf).         */
/* ------------------------------------------------------------------ */
bool quiz_exists_for_story(const char *story_filepath) {
    if (!story_filepath) return false;

    const char *fname = strrchr(story_filepath, '/');
    fname = fname ? fname + 1 : story_filepath;

    char prefix[8];
    int  n = 0;
    if (!parse_story_filename(fname, prefix, sizeof(prefix), &n)) return false;

    const char *last = strrchr(story_filepath, '/');
    if (!last) return false;
    size_t dlen = (size_t)(last - story_filepath);
    if (dlen >= STORY_DIR_MAX) return false;

    char dir[STORY_DIR_MAX];
    memcpy(dir, story_filepath, dlen);
    dir[dlen] = '\0';

    char qpath[PATH_BUF];
    snprintf(qpath, sizeof(qpath), "%s/s%s%d_1.wav", dir, prefix, n);
    bool exists = (access(qpath, F_OK) == 0);
    ESP_LOGI(TAG, "Quiz check %s%d: %s -> %s", prefix, n, qpath,
             exists ? "FOUND" : "not found");
    return exists;
}

/* ------------------------------------------------------------------ */
/*  quiz_begin                                                          */
/*  Parses story filepath, stores prefix+num+dir, resets to quiz 1.    */
/*  Works for every prophet folder AND the TAWID folder.               */
/*  Sets quiz_count = 5 for TAWID, quiz_count = 3 for all others.      */
/* ------------------------------------------------------------------ */
bool quiz_begin(const char *story_filepath) {
    if (!story_filepath) return false;

    const char *fname = strrchr(story_filepath, '/');
    fname = fname ? fname + 1 : story_filepath;

    char prefix[8];
    int  num = 0;
    if (!parse_story_filename(fname, prefix, sizeof(prefix), &num)) return false;

    const char *last = strrchr(story_filepath, '/');
    if (!last) return false;
    size_t dlen = (size_t)(last - story_filepath);
    if (dlen >= STORY_DIR_MAX) return false;

    memcpy(story_dir, story_filepath, dlen);
    story_dir[dlen] = '\0';
    strncpy(story_prefix, prefix, sizeof(story_prefix) - 1);
    story_prefix[sizeof(story_prefix) - 1] = '\0';
    story_num  = num;
    quiz_index = 1;

    /* Detect TAWID folder — use 5 quizzes per lesson, 3 for prophet stories */
    const char *folder_name = strrchr(story_dir, '/');
    folder_name = folder_name ? folder_name + 1 : story_dir;
    quiz_count = (strcasecmp(folder_name, "TAWID") == 0) ? 5 : 3;

    /* Randomise option order */
    g_quiz_crr_is_A = (esp_random() & 1) ? true : false;
    selected_opt    = QUIZ_OPT_A;

    ESP_LOGI(TAG, "Quiz begin: prefix=%s story=%d dir=%s  quiz=%d/%d  slot_A=%s",
             story_prefix, story_num, story_dir, quiz_index, quiz_count,
             g_quiz_crr_is_A ? "CORRECT(crr)" : "WRONG(wr)");
    return true;
}

/* ------------------------------------------------------------------ */
/*  quiz_randomise                                                      */
/*  Re-randomises the option slot order for a retry of the same quiz.  */
/*  Resets selected_opt to A so the user starts fresh.                 */
/* ------------------------------------------------------------------ */
void quiz_randomise(void) {
    g_quiz_crr_is_A = (esp_random() & 1) ? true : false;
    selected_opt    = QUIZ_OPT_A;
    ESP_LOGI(TAG, "Quiz randomised: story=%d quiz=%d  slot_A=%s",
             story_num, quiz_index,
             g_quiz_crr_is_A ? "CORRECT(crrm)" : "WRONG(wrm)");
}

/* ------------------------------------------------------------------ */
/*  quiz_advance                                                        */
/*  Moves to the next quiz round and re-randomises option order.       */
/*  Call only when quiz_is_last() is false.                            */
/* ------------------------------------------------------------------ */
void quiz_advance(void) {
    if (quiz_index < quiz_count) quiz_index++;
    g_quiz_crr_is_A = (esp_random() & 1) ? true : false;
    selected_opt    = QUIZ_OPT_A;
    ESP_LOGI(TAG, "Quiz advanced: story=%d -> quiz=%d/%d  slot_A=%s",
             story_num, quiz_index, quiz_count,
             g_quiz_crr_is_A ? "CORRECT(crrm)" : "WRONG(wrm)");
}

/* ------------------------------------------------------------------ */
/*  quiz_is_last                                                        */
/*  Returns true when the current quiz is the final one.               */
/*  Final = quiz 3 for prophet stories, quiz 5 for TAWID.              */
/* ------------------------------------------------------------------ */
bool quiz_is_last(void) {
    return (quiz_index >= quiz_count);
}

/* ------------------------------------------------------------------ */
/*  quiz_get_current_num                                                */
/*  Returns the current quiz index (1 .. quiz_count).                  */
/* ------------------------------------------------------------------ */
int quiz_get_current_num(void) {
    return quiz_index;
}

/* ------------------------------------------------------------------ */
/*  quiz_get_total_count                                                */
/*  Returns the total number of quizzes for this session:              */
/*    3 for prophet story folders, 5 for TAWID.                        */
/* ------------------------------------------------------------------ */
int quiz_get_total_count(void) {
    return quiz_count;
}

/* ------------------------------------------------------------------ */
/*  Plays sm<N>.wav DIRECTLY (blocking) from inside audio_task.        */
/*  Called from the NOTIFY_QUIZ_BIT handler — never from main loop.    */
/* ------------------------------------------------------------------ */
void quiz_play_question(volatile bool *sf) {
    char path[PATH_BUF];
    snprintf(path, sizeof(path), "%s/s%s%d_%d.wav",
             story_dir, story_prefix, story_num, quiz_index);
    ESP_LOGI(TAG, "Playing question (prefix=%s story=%d quiz=%d): %s",
             story_prefix, story_num, quiz_index, path);
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
    memset(story_dir,    0, sizeof(story_dir));
    memset(story_prefix, 0, sizeof(story_prefix));
    story_num       = 0;
    quiz_index      = 1;
    quiz_count      = 3;
    selected_opt    = QUIZ_OPT_A;
    g_quiz_crr_is_A = true;
    ESP_LOGI(TAG, "Quiz system initialized");
}

void quiz_end(void) {
    memset(story_dir,    0, sizeof(story_dir));
    memset(story_prefix, 0, sizeof(story_prefix));
    story_num       = 0;
    quiz_index      = 1;
    quiz_count      = 3;
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
