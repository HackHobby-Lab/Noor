/**
 * battery.h
 * Li-ion battery percentage monitor with low-battery audio alerts
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise ADC and start the battery monitor task. */
void battery_init(void);

/**
 * battery_set_audio_task
 * Register the audio_task handle and the dedicated notification bit
 * audio_task should handle for battery alerts.
 *
 * @task        : audio_task handle (from xTaskCreatePinnedToCore)
 * @notify_bit  : a unique FreeRTOS notification bit, e.g. (1UL << 26)
 *                Must NOT overlap with NOTIFY_ANNOUNCE_BIT, NOTIFY_SETTLE_BIT,
 *                NOTIFY_ENTER_SUBFOLDER_BIT, NOTIFY_QUIZ_BIT, or NOTIFY_QUIZ_RESULT_BIT.
 */
void battery_set_audio_task(TaskHandle_t task, uint32_t notify_bit);

/**
 * battery_set_playing_flag
 * Pass a pointer to main's g_playing flag.
 * When true, alerts are DEFERRED (not dropped) until playback ends.
 */
void battery_set_playing_flag(volatile bool *playing_flag);

/**
 * battery_play_pending_alert
 * Called by audio_task when NOTIFY_BATTERY_ALERT_BIT fires.
 * Plays the queued alert WAV with a local stop flag — never killed by g_stop_flag.
 */
void battery_play_pending_alert(void);

/** Current battery percentage (0–100), or -1 if not yet measured. */
int battery_get_percent(void);

/** Current battery voltage in millivolts. */
int battery_get_mv(void);

#ifdef __cplusplus
}
#endif
