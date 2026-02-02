/**
 * headphone_detect.h
 * Headphone detection with automatic speaker muting
 * 
 * This module handles:
 * - Detecting headphone insertion/removal
 * - Automatically muting speaker when headphones connected
 * - Optional feature (disabled by default)
 */

#ifndef HEADPHONE_DETECT_H
#define HEADPHONE_DETECT_H
#include "config.h"
#include <stdbool.h>

/* Optional GPIO pins (set to -1 to disable) */
#define HEADPHONE_DETECT_PIN  -1   // Jack detect pin (LOW = headphone inserted)
#define SPEAKER_ENABLE_PIN    -1   // Speaker enable pin (HIGH = speaker on)

/**
 * Initialize headphone detection system
 * 
 * @return true if enabled and initialized, false if disabled
 * 
 * Note: This function checks if pins are configured (not -1).
 * If disabled, it does nothing and returns false.
 */
bool headphone_detect_init(void);

/**
 * Start headphone detection monitoring task
 * Monitors detect pin and controls speaker enable pin
 * 
 * @return true if task started, false if feature disabled
 */
bool headphone_detect_start_task(void);

/**
 * Check if headphone detection is enabled
 * 
 * @return true if feature is enabled
 */
bool headphone_detect_is_enabled(void);

/**
 * Get current headphone state
 * 
 * @return true if headphones are connected, false otherwise
 */
bool headphone_detect_is_connected(void);

#endif // HEADPHONE_DETECT_H