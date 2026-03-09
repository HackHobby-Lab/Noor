/**
 * encoder.h
 * Rotary encoder handling for Noor Audio Player
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/* ENC_CLK_PIN 1, ENC_DT_PIN 2, ENC_SW_PIN 21 - defined in config.h */

#define ENC_DEBOUNCE_MS  60
#define ENC_QUEUE_LEN    16

typedef enum {
    ENC_EVENT_ROTATE = 1,
    ENC_EVENT_BUTTON = 2
} encoder_event_type_t;

typedef enum {
    ENC_DIR_CW  = 0,
    ENC_DIR_CCW = 1
} encoder_direction_t;

typedef struct {
    encoder_event_type_t type;
    encoder_direction_t  direction;
} encoder_event_t;

typedef void (*encoder_callback_t)(encoder_event_t event);

bool         encoder_init(void);
void         encoder_start_task(encoder_callback_t callback);
QueueHandle_t encoder_get_queue(void);

#endif // ENCODER_H
