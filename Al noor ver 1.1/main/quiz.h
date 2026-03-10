/**
 * quiz.h
 * Quiz system for Noor Audio Player
 *
 * Supports Muhammad stories m1.wav – m7.wav.
 * Each story has:
 *   sm<N>.wav   — question audio
 *   crrm<N>.wav — correct answer explanation
 *   wrm<N>.wav  — wrong answer explanation
 *
 * Option order is randomised each quiz (quiz_begin):
 *   slot A may be crrm or wrm depending on esp_random() coin-flip.
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
 * quiz_story_number — extracts N from m<N>.wav (1-7), returns 0 if not a quiz story.
 */
int  quiz_story_number(const char *filename);

/**
 * quiz_exists_for_story — returns true if sm<N>.wav exists alongside the story.
 */
bool quiz_exists_for_story(const char *story_filepath);

/**
 * quiz_begin — initialises quiz state and randomises option order.
 * Must be called once per quiz before any play/announce functions.
 */
bool quiz_begin(const char *story_filepath);

/**
 * quiz_play_question — plays sm<N>.wav BLOCKING (direct audio_play_file).
 * Call ONLY from inside audio_task.
 */
void quiz_play_question(volatile bool *sf);

/**
 * quiz_play_options_intro — plays option A then option B BLOCKING.
 * Order is randomised by quiz_begin().
 * Call ONLY from inside audio_task.
 */
void quiz_play_options_intro(volatile bool *sf);

/**
 * quiz_announce_option — queues crrm or wrm for the given slot via announce_request.
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
