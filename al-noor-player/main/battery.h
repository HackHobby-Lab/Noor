#ifndef BATTERY_H
#define BATTERY_H

/**
 * battery.h
 * Li-ion battery percentage monitor via ADC
 *
 * Circuit: Battery+ → 10K → GPIO6 → 22K → GND
 *
 * Low battery audio alerts (files on SD card root):
 *   b1.wav → plays once when battery drops to 40%
 *   b2.wav → plays once when battery drops to 20%
 *   b3.wav → plays once when battery drops to 5%
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void battery_init(void);

/**
 * Set the audio task handle so battery alerts can play WAV files.
 * Call this AFTER the audio task is created.
 */
void battery_set_audio_task(TaskHandle_t task, volatile bool *stop_flag);

int  battery_get_percent(void);
int  battery_get_mv(void);

#endif /* BATTERY_H */