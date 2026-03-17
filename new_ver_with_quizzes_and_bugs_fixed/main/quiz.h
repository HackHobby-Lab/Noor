/**
 * quiz.h
 * Quiz system for Noor Audio Player
 *
 * Supports ALL prophet subfolders (Adam, Daoud, Ibrahim, Ismaeel, Issa,
 * Moussa, Muhammad, Nuh, Yaqoob, Yousuf) — stories 1-10 each, 3 quizzes per story.
 * Also supports the TAWID folder — lessons T1-T20, 5 quizzes per lesson.
 *
 * Each story quiz (Q = 1, 2, 3):
 *   s{prefix}{N}_{Q}.wav   — question audio
 *   crr{prefix}{N}_{Q}.wav — correct answer explanation
 *   wr{prefix}{N}_{Q}.wav  — wrong answer explanation
 *
 * Each TAWID quiz (Q = 1 … 5):
 *   st{N}_{Q}.wav    — question audio  (e.g. st1_1.wav … st20_5.wav)
 *   crrt{N}_{Q}.wav  — correct answer  (e.g. crrt1_1.wav … crrt20_5.wav)
 *   wrt{N}_{Q}.wav   — wrong answer    (e.g. wrt1_1.wav … wrt20_5.wav)
 *   (prefix is "t" because parse_story_filename("T5.wav") → prefix="t")
 *
 * Prefix table:
 *   Adam=a  Daoud=d  Ibrahim=ib  Ismaeel=is  Issa=i
 *   Moussa=ms  Muhammad=m  Nuh=n  Yaqoob=yq  Yousuf=y
 *
 * Flow per story/lesson:
 *   Audio ends → Quiz 1
 *     Wrong  → replay Quiz 1 (repeat until correct)
 *     Correct → Quiz 2 → … → Quiz N (3 for stories, 5 for TAWID)
 *         Correct on last → return to FILE_VIEW
 *
 * Option order is randomised each round (quiz_begin / quiz_randomise):
 *   slot A may be crr or wr depending on esp_random() coin-flip.
 *   quiz_submit() uses the internal mapping to determine correctness.
 */

#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Option slots (A = first presented, B = second) */
typedef enum {
    QUIZ_OPT_A = 0,
    QUIZ_OPT_B = 1,
} quiz_option_t;

typedef enum {
    QUIZ_RESULT_CORRECT = 0,
    QUIZ_RESULT_WRONG   = 1,
} quiz_result_t;

/**
 * quiz_story_number — extracts N from any story filename (1-10).
 * Works for all prophets: a3.wav->3, ib10.wav->10, yq1.wav->1, etc.
 * Returns 0 if the filename is not a valid story file.
 */
int  quiz_story_number(const char *filename);

/**
 * quiz_exists_for_story — returns true if s{prefix}{N}_1.wav exists alongside the story.
 * Works for all prophet folders.
 */
bool quiz_exists_for_story(const char *story_filepath);

/**
 * quiz_begin — initialises quiz state at quiz 1 and randomises option order.
 * Must be called once when a story ends before starting quizzes.
 */
bool quiz_begin(const char *story_filepath);

/**
 * quiz_randomise — re-randomises the option slot order for a retry of the same quiz.
 * Call when the user answers WRONG to shuffle options before replaying.
 */
void quiz_randomise(void);

/**
 * quiz_advance — moves to the next quiz (quiz_index++), re-randomises option order.
 * Call when the user answers CORRECT and quiz_is_last() is false.
 */
void quiz_advance(void);

/**
 * quiz_is_last — returns true when the current quiz is the final one.
 * Final = quiz 3 for prophet story folders, quiz 5 for TAWID.
 */
bool quiz_is_last(void);

/**
 * quiz_get_current_num — returns the current quiz index (1 .. quiz_count).
 */
int quiz_get_current_num(void);

/**
 * quiz_get_total_count — returns total quizzes for this session:
 *   3 for prophet story folders, 5 for TAWID.
 */
int quiz_get_total_count(void);

/**
 * quiz_play_question — plays s{prefix}{N}_{Q}.wav BLOCKING (direct audio_play_file).
 * Call ONLY from inside audio_task.
 */
void quiz_play_question(volatile bool *sf);

/**
 * quiz_play_options_intro — plays option A then option B BLOCKING.
 * Order is randomised by quiz_begin() / quiz_randomise() / quiz_advance().
 * Call ONLY from inside audio_task.
 */
void quiz_play_options_intro(volatile bool *sf);

/**
 * quiz_announce_option — queues crr{prefix} or wr{prefix} for the given slot via announce_request.
 * Safe to call from encoder_callback or main loop (outside audio_task).
 */
void quiz_announce_option(quiz_option_t opt, TaskHandle_t th, volatile bool *sf);

/**
 * quiz_get_selected / quiz_next_option / quiz_prev_option — slot navigation.
 */
quiz_option_t quiz_get_selected(void);
void          quiz_next_option(void);
void          quiz_prev_option(void);

/**
 * quiz_submit — returns CORRECT or WRONG based on current selection.
 */
quiz_result_t quiz_submit(void);

/**
 * quiz_play_result — plays correct.wav or wrong.wav BLOCKING (direct audio_play_file).
 * Searches /sdcard/ANNOUN~1/ then /sdcard/.
 * Call ONLY from inside audio_task.
 */
void quiz_play_result(quiz_result_t result, volatile bool *sf);

/**
 * quiz_is_crr_slot_a — true if slot A is the correct answer this round.
 */
bool quiz_is_crr_slot_a(void);

void quiz_init(void);
void quiz_end(void);

#ifdef __cplusplus
}
#endif
