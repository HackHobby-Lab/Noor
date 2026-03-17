/**
 * headphone_detect.h
 * Headphone detection interface
 *
 * HEADPHONE_DETECT_PIN and SPEAKER_ENABLE_PIN are defined in config.h.
 * Do NOT redefine them here.
 */

#pragma once

#include "config.h"   /* HEADPHONE_DETECT_PIN, SPEAKER_ENABLE_PIN */
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/**
 * Initialize the headphone detection GPIO(s).
 * Returns true if the feature is enabled (HEADPHONE_DETECT_PIN >= 0)
 * and GPIO configuration succeeded.
 * Returns false if the feature is disabled or init failed.
 */
bool headphone_detect_init(void);

/**
 * Start the background monitoring task.
 * Call only after headphone_detect_init() returned true.
 * Returns true on success.
 */
bool headphone_detect_start_task(void);

/** Returns true if the detection feature is active. */
bool headphone_detect_is_enabled(void);

/** Returns true if headphones are currently detected as connected. */
bool headphone_detect_is_connected(void);
