/**
 * encoder.h
 * Rotary encoder handling for Noor Audio Player
 * 
 * This module handles:
 * - Rotary encoder rotation detection (CW/CCW)
 * - Encoder button press detection
 * - Interrupt-driven operation with queue
 */

#ifndef ENCODER_H
#define ENCODER_H
#include "config.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/* Encoder GPIO pins */
#define ENC_CLK_PIN  1    // A phase (clock)
#define ENC_DT_PIN   2    // B phase (data)
#define ENC_SW_PIN   21   // Push button switch

/* Encoder debounce time */
#define ENC_DEBOUNCE_MS  60

/* Queue length for encoder events */
#define ENC_QUEUE_LEN  16

/**
 * Encoder event types
 */
typedef enum {
    ENC_EVENT_ROTATE = 1,   // Rotation detected
    ENC_EVENT_BUTTON = 2    // Button press detected
} encoder_event_type_t;

/**
 * Encoder rotation direction
 */
typedef enum {
    ENC_DIR_CW = 0,   // Clockwise
    ENC_DIR_CCW = 1   // Counter-clockwise
} encoder_direction_t;

/**
 * Encoder event structure
 */
typedef struct {
    encoder_event_type_t type;      // Event type
    encoder_direction_t direction;  // Direction (only for ROTATE events)
} encoder_event_t;

/**
 * Encoder callback function type
 * 
 * @param event Encoder event that occurred
 */
typedef void (*encoder_callback_t)(encoder_event_t event);

/**
 * Initialize encoder GPIOs and interrupts
 * 
 * @return true if successful, false on error
 */
bool encoder_init(void);

/**
 * Start encoder task
 * This task processes encoder events from the ISR queue
 * 
 * @param callback Function to call when encoder event occurs
 */
void encoder_start_task(encoder_callback_t callback);

/**
 * Get encoder event queue handle
 * (For advanced users who want to process events manually)
 * 
 * @return Queue handle
 */
QueueHandle_t encoder_get_queue(void);

#endif // ENCODER_H