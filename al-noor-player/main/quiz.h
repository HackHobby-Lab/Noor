/**
 * quiz.h
 * Quiz system for Noor Audio Player
 *
 * Flow:
 *   Story (M1.WAV) ends naturally
 *   -> quiz_exists_for_story() checks for sm1.wav in same folder
 *   -> quiz_begin() sets up paths
 *   -> quiz_play_question() plays sm1.wav
 *   -> question ends naturally -> quiz_play_options_intro() plays A.wav then B.wav
 *   -> user rotates encoder -> quiz_next_option() + quiz_announce_option() plays A/B
 *   -> user presses play or encoder button -> quiz_submit() -> quiz_play_result()
 *   -> Home button -> quiz_do_exit() -> back to FILE_VIEW
 *
 * Files needed in story folder (e.g. /sdcard/STORIES/MUHAMMAD/):
 *   sm1.wav    - question audio for story 1
 *   A.wav      - option A audio
 *   B.wav      - option B audio
 *   wrm1.wav   - wrong answer explanation for story 1
 *   crrm1.wav  - correct answer explanation for story 1
 *
 * Files needed in SD root (/sdcard/):
 *   correct.wav - generic "correct!" sound
 *   wrong.wav   - generic "wrong!" sound
 */

#ifndef QUIZ_H
#define QUIZ_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    QUIZ_OPT_A = 1,
    QUIZ_OPT_B = 2
} quiz_option_t;

typedef enum {
    QUIZ_RESULT_CORRECT = 0,
    QUIZ_RESULT_WRONG   = 1
} quiz_result_t;

/** Initialize quiz system (call once at startup) */
void quiz_init(void);

/** Reset quiz state (call when quiz ends) */
void quiz_end(void);

/**
 * Extract story number from filename
 * M1.WAV / m1.wav -> 1   (case insensitive)
 * Returns 0 if not a quiz story
 */
int quiz_story_number(const char *filename);

/**
 * Check if sm<N>.wav exists in the same folder as the story
 * Call after story finishes naturally
 */
bool quiz_exists_for_story(const char *story_filepath);

/**
 * Initialize quiz for given story filepath
 * Must call before any other quiz functions
 */
bool quiz_begin(const char *story_filepath);

/** Play question audio (sm<N>.wav) via announce system */
void quiz_play_question(TaskHandle_t th, volatile bool *sf);

/**
 * Play options intro: A.wav then B.wav
 * Call after question audio finishes
 * This is BLOCKING - waits for both files to finish
 */
void quiz_play_options_intro(TaskHandle_t th, volatile bool *sf);

/**
 * Announce the currently selected option (A.wav or B.wav)
 * Call on each encoder rotation
 */
void quiz_announce_option(quiz_option_t opt, TaskHandle_t th, volatile bool *sf);

/** Get currently selected option */
quiz_option_t quiz_get_selected(void);

/** Move to next option (A->B or B->A) */
void quiz_next_option(void);

/** Move to prev option (same as next since only 2 options) */
void quiz_prev_option(void);

/**
 * Submit selected answer
 * Returns QUIZ_RESULT_CORRECT or QUIZ_RESULT_WRONG
 */
quiz_result_t quiz_submit(void);

/**
 * Play result audio:
 *   wrm<N>.wav or crrm<N>.wav (story-specific)
 *   then correct.wav or wrong.wav (generic feedback)
 */
void quiz_play_result(quiz_result_t result, TaskHandle_t th, volatile bool *sf);

#endif /* QUIZ_H */
