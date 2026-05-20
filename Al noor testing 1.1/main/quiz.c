/**
 * quiz.c
 * Quiz system for Noor Audio Player
 *
 * PATCH (this revision):
 * ----------------------
 * 1. parse_story_filename: lifted N>20 cap to N>999.  Client can now have
 *    any number of lessons and any number of stories per prophet without
 *    recompiling firmware.
 * 2. quiz_begin: hardcoded "3 quizzes for prophets / 5 for TAWID" rule
 *    REMOVED.  Quiz count is now detected dynamically by probing for
 *    s{prefix}{N}_1.wav, _2.wav, _3.wav ... on the SD card.  Whatever
 *    number of quiz files the user puts there is what plays.
 * 3. Quiz folder gating ("only TEACHINGS") is handled in main.c at the
 *    quiz_exists_for_story call site, NOT here, because quiz.c has no
 *    knowledge of folder semantics — main.c knows which folder is active.
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
 * SD card layout — applies ONLY to the TEACHINGS folder.
 * Quizzes for stories in STORIES/...  or INVOCATIONS/...  are gated OFF
 * by main.c via is_quiz_eligible_folder() before this code is invoked.
 *
 * File naming convention (inside TEACHINGS/):
 *   s{prefix}{N}_{Q}.wav    quiz question
 *   crr{prefix}{N}_{Q}.wav  correct answer
 *   wr{prefix}{N}_{Q}.wav   wrong answer
 *
 * {prefix} matches the lesson filename prefix (e.g. "t" for T1.wav).
 * {N}      = lesson number (1-999, no hardcoded cap).
 * {Q}      = quiz number — count is DYNAMIC, detected at runtime by
 *            probing _1, _2, _3, ... up to _99 on the SD card.  Client
 *            can put any number of quiz files per lesson.
 *
 * Examples (inside /sdcard/TEACHINGS/):
 *   T1.wav           lesson 1 audio
 *   st1_1.wav        lesson 1, quiz 1 question
 *   crrt1_1.wav      lesson 1, quiz 1 correct answer
 *   wrt1_1.wav       lesson 1, quiz 1 wrong answer
 *   st1_2.wav ...    add as many quizzes per lesson as desired
 *
 * Prophet prefix table (used by the parse logic only; quizzes won't fire
 * for these folders, but the function still tolerates the filenames):
 *   Adam=a  Daoud=d  Ibrahim=ib  Ismaeel=is  Issa=i
 *   Moussa=ms  Muhammad=m  Nuh=n  Yaqoob=yq  Yousuf=y
 *
 * Answer mapping:
 *   g_quiz_crr_is_A == true  -> QUIZ_OPT_A = correct, QUIZ_OPT_B = wrong
 *   g_quiz_crr_is_A == false -> QUIZ_OPT_A = wrong,   QUIZ_OPT_B = correct
 *
 * Result audio search order (quiz_play_result):
 *   1. /sdcard/ANNOUN~1/correct.wav or wrong.wav  (via announce_check_and_play)
 *   2. /sdcard/correct.wav or wrong.wav            (root fallback)
 *
 * *** FIX 1 — stop-flag ownership (quiz_play_question / quiz_play_options_intro) ***
 *
 * ORIGINAL code reset *sf = false before every audio_play_file() call inside
 * these two functions.  This caused encoder-driven cancellation to be silently
 * ignored.  Fix: removed ALL *sf = false assignments from quiz_play_question()
 * and quiz_play_options_intro().  The caller (audio_task) is responsible for
 * clearing the flag before calling these functions.
 *
 * *** FIX 2 — FAT 8.3 filename mangling for long "crr" prefixed filenames ***
 *
 * The FAT filesystem used on the SD card enforces 8.3 filenames (8 chars base,
 * 3 chars extension).  Long filenames are stored in hidden LFN (Long File Name)
 * directory entries alongside a mangled 8.3 alias.
 *
 * The ESP-IDF FATFS/SPI driver exposes LFN support via CONFIG_FATFS_LFN_HEAP or
 * CONFIG_FATFS_LFN_STACK.  When LFN is DISABLED, access() and fopen() can only
 * find files by their 8.3 alias, not by the long name.  This caused the bug:
 *
 *   "wris10_1.wav"  (12 chars total, 8-char base) — fits in 8.3 → found  ✅
 *   "crris10_1.wav" (13 chars total, 9-char base) — OVER 8.3 limit → mangled
 *                   → stored as e.g. "CRRIS1~1.WAV" → access("crris10_1.wav") = -1 ❌
 *
 * Similarly affected prefixes (base name > 8 chars):
 *   crris{N}_{Q}.wav  (prefix "is", crr+is = 5 chars + digits + _Q = often 9+)
 *   crrms{N}_{Q}.wav  (prefix "ms")
 *   crryq{N}_{Q}.wav  (prefix "yq")
 *   crrm{N}_{Q}.wav   (prefix "m", 2-digit story number pushes it over)
 *
 * Fix: fat_access() — a drop-in replacement for access() that:
 *   1. Tries the path as-is (works when LFN is enabled or name is short enough).
 *   2. If that fails, scans the parent directory with opendir()/readdir() and
 *      compares filenames case-insensitively.  This matches the LFN entry
 *      regardless of what 8.3 alias the FAT driver generated.
 *
 * fat_access() is used everywhere in this file instead of access().
 *
 * To permanently fix this at the filesystem level, enable LFN support in
 * sdkconfig:
 *   CONFIG_FATFS_LFN_HEAP=y   (or CONFIG_FATFS_LFN_STACK=y)
 *   CONFIG_FATFS_MAX_LFN=255
 * With LFN enabled, the original access() calls would work and fat_access()
 * degrades gracefully to a single access() call (the fast path succeeds).
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
#include <dirent.h>   /* opendir / readdir — needed by fat_access() */

static const char *TAG = "QUIZ";

#define STORY_DIR_MAX  256
#define PATH_BUF       512

/* ---- State ---- */
static char story_dir[STORY_DIR_MAX] = {0};
static char story_prefix[8]         = {0};
static int  story_num  = 0;
static int  quiz_index = 1;

static int  quiz_count = 3;

static bool          g_quiz_crr_is_A = true;
static quiz_option_t selected_opt    = QUIZ_OPT_A;

/* ------------------------------------------------------------------ */
/*  fat_access()                                                        */
/*                                                                      */
/*  Drop-in replacement for access(path, F_OK) that works even when    */
/*  the FAT filesystem has mangled the 8.3 alias for a long filename.  */
/*                                                                      */
/*  Strategy:                                                           */
/*   1. Fast path  — try access() directly.  This succeeds whenever    */
/*      LFN support is enabled in sdkconfig, or the filename already   */
/*      fits within 8.3.                                                */
/*   2. Slow path  — split path into dir + filename, then scan the     */
/*      directory with readdir() comparing names case-insensitively.   */
/*      readdir() returns the LFN (long name) when available, so this  */
/*      finds the file regardless of what 8.3 alias the FAT driver     */
/*      chose.                                                          */
/*                                                                      */
/*  Returns: 0 if found, -1 if not found.                              */
/*                                                                      */
/*  If found via slow path, resolved_out (if non-NULL and buf_sz > 0)  */
/*  is filled with the exact path that can be passed to fopen().       */
/*  If found via fast path, resolved_out is a copy of the input path.  */
/* ------------------------------------------------------------------ */
static int fat_access(const char *path, char *resolved_out, size_t buf_sz)
{
    if (!path) return -1;

    /* --- Fast path: try as-is --- */
    if (access(path, F_OK) == 0) {
        if (resolved_out && buf_sz > 0)
            snprintf(resolved_out, buf_sz, "%s", path);
        return 0;
    }

    /* --- Slow path: directory scan --- */

    /* Split into directory and filename components */
    const char *last_sep = strrchr(path, '/');
    if (!last_sep) return -1;   /* no directory component, give up */

    /* Directory portion */
    size_t dir_len = (size_t)(last_sep - path);
    if (dir_len == 0 || dir_len >= PATH_BUF) return -1;

    char dir_buf[PATH_BUF];
    memcpy(dir_buf, path, dir_len);
    dir_buf[dir_len] = '\0';

    /* Filename portion (the part after the last '/') */
    const char *want_name = last_sep + 1;
    if (*want_name == '\0') return -1;

    DIR *d = opendir(dir_buf);
    if (!d) return -1;

    struct dirent *entry;
    bool found = false;
    while ((entry = readdir(d)) != NULL) {
        /* Case-insensitive comparison — FAT is case-insensitive */
        if (strcasecmp(entry->d_name, want_name) == 0) {
            found = true;
            if (resolved_out && buf_sz > 0)
                snprintf(resolved_out, buf_sz, "%s/%s", dir_buf, entry->d_name);
            break;
        }
    }
    closedir(d);

    return found ? 0 : -1;
}

/* Convenience wrapper: check existence only (no resolved path needed) */
static inline bool fat_exists(const char *path)
{
    return (fat_access(path, NULL, 0) == 0);
}

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static bool parse_story_filename(const char *filename,
                                  char *prefix, size_t prefix_sz,
                                  int  *num_out)
{
    if (!filename || !prefix || !num_out) return false;

    if (strchr(filename, '_')) return false;

    const char *dot = strrchr(filename, '.');
    if (!dot || strcasecmp(dot, ".wav") != 0) return false;

    size_t i = 0;
    while (filename[i] && !isdigit((unsigned char)filename[i]) && filename[i] != '.') i++;

    if (i == 0 || !isdigit((unsigned char)filename[i])) return false;
    if (i >= prefix_sz) return false;

    for (size_t j = 0; j < i; j++)
        prefix[j] = (char)tolower((unsigned char)filename[j]);
    prefix[i] = '\0';

    ptrdiff_t dot_pos  = dot - filename;
    ptrdiff_t num_start = (ptrdiff_t)i;
    size_t    num_len   = (size_t)(dot_pos - num_start);
    if (num_len == 0 || num_len > 3) return false;

    char numstr[4] = {0};
    memcpy(numstr, filename + num_start, num_len);
    int n = atoi(numstr);
    /* PATCH: lifted from n>20 to n>999 so any number of lessons/stories
     * works without a firmware recompile. */
    if (n < 1 || n > 999) return false;

    *num_out = n;
    return true;
}

static void _crr_path(char *buf, size_t sz) {
    snprintf(buf, sz, "%s/crr%s%d_%d.wav", story_dir, story_prefix, story_num, quiz_index);
}

static void _wrm_path(char *buf, size_t sz) {
    snprintf(buf, sz, "%s/wr%s%d_%d.wav", story_dir, story_prefix, story_num, quiz_index);
}

static void _opt_a_path(char *buf, size_t sz) {
    if (g_quiz_crr_is_A) _crr_path(buf, sz);
    else                  _wrm_path(buf, sz);
}

static void _opt_b_path(char *buf, size_t sz) {
    if (g_quiz_crr_is_A) _wrm_path(buf, sz);
    else                  _crr_path(buf, sz);
}

/* ------------------------------------------------------------------ */
/*  quiz_story_number                                                   */
/* ------------------------------------------------------------------ */
int quiz_story_number(const char *filename) {
    char prefix[8];
    int  num = 0;
    if (!parse_story_filename(filename, prefix, sizeof(prefix), &num)) return 0;
    return num;
}

/* ------------------------------------------------------------------ */
/*  quiz_exists_for_story                                               */
/*                                                                      */
/*  NEW DESIGN (client confirmed):                                      */
/*  The question is now part of the lesson file (T{N}.wav).            */
/*  Quiz exists if BOTH the correct-answer (crrt) AND wrong-answer (wr) */
/*  files exist for quiz #1.  No separate question file (st...) needed. */
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

    /* New trigger: both crrt and wrt files must exist for quiz #1. */
    char qpath_crr[PATH_BUF];
    char qpath_wr[PATH_BUF];
    snprintf(qpath_crr, sizeof(qpath_crr), "%s/crr%s%d_1.wav", dir, prefix, n);
    snprintf(qpath_wr,  sizeof(qpath_wr),  "%s/wr%s%d_1.wav",  dir, prefix, n);
    bool exists = fat_exists(qpath_crr) && fat_exists(qpath_wr);
    ESP_LOGI(TAG, "Quiz check %s%d: crr=%s wr=%s -> %s",
             prefix, n, qpath_crr, qpath_wr, exists ? "FOUND" : "not found");
    return exists;
}

/* ------------------------------------------------------------------ */
/*  quiz_begin                                                          */
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

    const char *folder_name = strrchr(story_dir, '/');
    folder_name = folder_name ? folder_name + 1 : story_dir;

    /* NEW DESIGN: quiz_count is derived dynamically by counting actual
     * answer-pair files on the SD card.  Probe crr{prefix}{N}_1.wav AND
     * wr{prefix}{N}_1.wav, then _2, _3, ... until either is missing.
     * Both files must exist for a quiz to count.  This lets the client
     * put 1, 3, 5, or 20 quizzes per lesson without firmware changes.
     *
     * Note: this function is only called when the calling code has already
     * decided quizzes are allowed for this folder (see main.c quiz gating
     * to TEACHINGS only). */
    quiz_count = 0;
    for (int q = 1; q <= 99; q++) {
        char probe_crr[PATH_BUF];
        char probe_wr[PATH_BUF];
        snprintf(probe_crr, sizeof(probe_crr), "%s/crr%s%d_%d.wav",
                 story_dir, story_prefix, story_num, q);
        snprintf(probe_wr,  sizeof(probe_wr),  "%s/wr%s%d_%d.wav",
                 story_dir, story_prefix, story_num, q);
        if (!fat_exists(probe_crr) || !fat_exists(probe_wr)) break;
        quiz_count = q;
    }
    if (quiz_count == 0) {
        ESP_LOGW(TAG, "quiz_begin: no quiz file pairs found for %s%d in %s",
                 story_prefix, story_num, story_dir);
        return false;
    }
    ESP_LOGI(TAG, "Detected %d quiz pair(s) on card for %s%d",
             quiz_count, story_prefix, story_num);

    g_quiz_crr_is_A = (esp_random() & 1) ? true : false;
    selected_opt    = QUIZ_OPT_A;

    ESP_LOGI(TAG, "Quiz begin: prefix=%s story=%d dir=%s  quiz=%d/%d  slot_A=%s  folder=%s",
             story_prefix, story_num, story_dir, quiz_index, quiz_count,
             g_quiz_crr_is_A ? "CORRECT(crr)" : "WRONG(wr)", folder_name);
    return true;
}

/* ------------------------------------------------------------------ */
/*  quiz_randomise                                                      */
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
/* ------------------------------------------------------------------ */
bool quiz_is_last(void) {
    return (quiz_index >= quiz_count);
}

/* ------------------------------------------------------------------ */
/*  quiz_get_current_num                                                */
/* ------------------------------------------------------------------ */
int quiz_get_current_num(void) {
    return quiz_index;
}

/* ------------------------------------------------------------------ */
/*  quiz_get_total_count                                                */
/* ------------------------------------------------------------------ */
int quiz_get_total_count(void) {
    return quiz_count;
}

/* ------------------------------------------------------------------ */
/*  quiz_play_question                                                  */
/*                                                                      */
/*  NEW DESIGN (client confirmed):                                      */
/*  The question is normally part of the lesson file (T{N}.wav itself), */
/*  so there is no separate question audio to play.  This function       */
/*  remains as a no-op in the new flow — it only plays s{...}.wav if    */
/*  the SD card happens to provide one (backward compatibility).         */
/*                                                                      */
/*  FIX 1: removed *sf = false.                                         */
/*  FIX 2: uses fat_access() instead of access() so long filenames     */
/*  that got mangled to 8.3 aliases on the FAT SD card are found.      */
/* ------------------------------------------------------------------ */
void quiz_play_question(volatile bool *sf) {
    char path[PATH_BUF];
    char resolved[PATH_BUF];
    snprintf(path, sizeof(path), "%s/s%s%d_%d.wav",
             story_dir, story_prefix, story_num, quiz_index);
    if (fat_access(path, resolved, sizeof(resolved)) == 0) {
        ESP_LOGI(TAG, "Playing question (legacy st file): %s", resolved);
        audio_play_file(resolved, sf, NULL);
        vTaskDelay(pdMS_TO_TICKS(400));
    } else {
        /* New design: question is part of the lesson.  No separate file
         * to play — just continue to the options intro. */
        ESP_LOGI(TAG, "No separate question file (%s) — question was in lesson",
                 path);
    }
}

/* ------------------------------------------------------------------ */
/*  quiz_play_options_intro                                             */
/*                                                                      */
/*  FIX 1: removed all *sf = false assignments.                         */
/*  FIX 2: uses fat_access() with resolved path for both slot A and B  */
/*  so that long filenames with "crr" prefix (e.g. crris10_1.wav,      */
/*  which exceed 8 chars) are resolved to their actual FAT entry        */
/*  instead of failing with ENOENT.                                     */
/*                                                                      */
/*  Root cause of the original bug:                                     */
/*   "crris10_1.wav" has a 9-char base name (crris10_1), which exceeds  */
/*   the FAT 8.3 limit of 8 chars.  The FAT driver stores it under a   */
/*   mangled alias like "CRRIS1~1.WAV".  Without LFN support enabled   */
/*   in sdkconfig, access("crris10_1.wav") returns -1 even though the  */
/*   file is physically present on the card.  fat_access() works around */
/*   this by falling back to a case-insensitive readdir() scan which    */
/*   returns the LFN entry (the long name) when it is available.        */
/* ------------------------------------------------------------------ */
void quiz_play_options_intro(volatile bool *sf) {
    char path_a[PATH_BUF], path_b[PATH_BUF];
    char resolved_a[PATH_BUF], resolved_b[PATH_BUF];
    _opt_a_path(path_a, sizeof(path_a));
    _opt_b_path(path_b, sizeof(path_b));

    ESP_LOGI(TAG, "Options intro: A=%s  B=%s", path_a, path_b);

    /* Play slot A — only if not already interrupted */
    if (!(*sf)) {
        if (fat_access(path_a, resolved_a, sizeof(resolved_a)) == 0) {
            ESP_LOGI(TAG, "Playing option A: %s", resolved_a);
            audio_play_file(resolved_a, sf, NULL);
            vTaskDelay(pdMS_TO_TICKS(400));
        } else {
            ESP_LOGE(TAG, "Option A file missing: %s", path_a);
        }
    }

    /* Play slot B — only if not interrupted */
    if (!(*sf)) {
        if (fat_access(path_b, resolved_b, sizeof(resolved_b)) == 0) {
            ESP_LOGI(TAG, "Playing option B: %s", resolved_b);
            audio_play_file(resolved_b, sf, NULL);
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            ESP_LOGE(TAG, "Option B file missing: %s", path_b);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  quiz_announce_option                                                */
/*  Called from encoder_callback (NOT from audio_task).                */
/*  Uses announce_request() to queue the file — audio_task will play   */
/*  it on the next NOTIFY_ANNOUNCE_BIT.                                 */
/*                                                                      */
/*  FIX 2: uses fat_access() so the resolved (actual on-disk) path is  */
/*  passed to announce_request() rather than the logical long name that */
/*  FAT may not find directly.                                          */
/* ------------------------------------------------------------------ */
void quiz_announce_option(quiz_option_t opt, TaskHandle_t th, volatile bool *sf) {
    char path[PATH_BUF];
    char resolved[PATH_BUF];

    if (opt == QUIZ_OPT_A)
        _opt_a_path(path, sizeof(path));
    else
        _opt_b_path(path, sizeof(path));

    ESP_LOGI(TAG, "Announce option %s: %s",
             opt == QUIZ_OPT_A ? "A" : "B", path);

    if (fat_access(path, resolved, sizeof(resolved)) == 0)
        announce_request(resolved, th, sf);
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

void quiz_prev_option(void) { quiz_next_option(); }

/* ------------------------------------------------------------------ */
/*  quiz_submit                                                         */
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
/*                                                                      */
/*  NOTE: this function intentionally keeps *sf = false before          */
/*  audio_play_file.  quiz_do_submit() sets g_stop_flag=true to stop    */
/*  the option announcement — that stop signal must NOT prevent the     */
/*  result audio from playing.  Resetting sf here is correct.          */
/*                                                                      */
/*  FIX 2: uses fat_access() for the result audio lookup.              */
/* ------------------------------------------------------------------ */
void quiz_play_result(quiz_result_t result, volatile bool *sf) {
    bool correct = (result == QUIZ_RESULT_CORRECT);

    /* NEW DESIGN (client confirmed):
     *   Correct → generic /announcements/correct.wav  (e.g. "Bravo!")
     *   Wrong   → per-quiz explanation in TEACHINGS folder:
     *             expt{N}_{Q}.wav  (e.g. "No, we pray to Allah because…")
     *             If the per-quiz explanation file is missing on the SD card,
     *             fall back to the generic /announcements/wrong.wav so the
     *             child still gets feedback.
     */

    /* WRONG case: try the per-quiz explanation first, in the lesson's own dir. */
    if (!correct) {
        char expt_path[PATH_BUF];
        char expt_resolved[PATH_BUF];
        snprintf(expt_path, sizeof(expt_path), "%s/exp%s%d_%d.wav",
                 story_dir, story_prefix, story_num, quiz_index);
        if (fat_access(expt_path, expt_resolved, sizeof(expt_resolved)) == 0) {
            ESP_LOGI(TAG, "Playing per-quiz explanation: %s", expt_resolved);
            *sf = false;
            audio_play_file(expt_resolved, sf, NULL);
            return;
        }
        ESP_LOGI(TAG, "No per-quiz explanation (%s), falling back to generic wrong.wav",
                 expt_path);
    }

    /* Generic fallback (always used for correct, and for wrong when expt missing). */
    const char *generic = correct ? "correct.wav" : "wrong.wav";

    ESP_LOGI(TAG, "Playing result: %s -> %s",
             correct ? "CORRECT" : "WRONG", generic);

    const char *search_dirs[] = {
        "/sdcard/ANNOUN~1",
        "/sdcard/announcements",
        "/sdcard",
        NULL
    };

    for (int i = 0; search_dirs[i]; i++) {
        char path[PATH_BUF];
        char resolved[PATH_BUF];
        snprintf(path, sizeof(path), "%s/%s", search_dirs[i], generic);
        if (fat_access(path, resolved, sizeof(resolved)) == 0) {
            ESP_LOGI(TAG, "Result audio: %s", resolved);
            *sf = false;   /* intentional — stop signal from submit ≠ abort result */
            audio_play_file(resolved, sf, NULL);
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
/* ------------------------------------------------------------------ */
bool quiz_is_crr_slot_a(void) {
    return g_quiz_crr_is_A;
}