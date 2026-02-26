/**
 * headphone_detect.h
 * Headphone detection with automatic volume adjustment
 */

#ifndef HEADPHONE_DETECT_H
#define HEADPHONE_DETECT_H

#include "config.h"
#include <stdbool.h>

/* HEADPHONE_DETECT_PIN 48 and SPEAKER_ENABLE_PIN -1 defined in config.h */

bool headphone_detect_init(void);
bool headphone_detect_start_task(void);
bool headphone_detect_is_enabled(void);
bool headphone_detect_is_connected(void);

#endif // HEADPHONE_DETECT_H
