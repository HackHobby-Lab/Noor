/**
 * buttons.h
 * Button handling with debouncing for Noor Audio Player
 * 
 * This module handles:
 * - Button GPIO initialization
 * - Debounced button press detection
 * - Support for active-HIGH buttons with internal pull-down
 */

#ifndef BUTTONS_H
#define BUTTONS_H
#include "config.h"
#include <stdbool.h>
#include "driver/gpio.h"

/* Button GPIO pins (active HIGH with internal pull-down) */
#define BTN_PLAY_PIN      14
#define BTN_HOME_PIN      15
#define BTN_VOL_UP_PIN    4
#define BTN_VOL_DOWN_PIN  5

/* Debounce time in milliseconds */
#define BUTTON_DEBOUNCE_MS  50

/**
 * Button structure for state tracking
 */
typedef struct {
    gpio_num_t gpio;        // GPIO pin number
    uint32_t last_time;     // Last state change time (ms)
    bool last_state;        // Last button state (true = pressed)
} button_t;

/**
 * Initialize all button GPIOs
 * Configures pins with internal pull-down resistors
 */
void buttons_init(void);

/**
 * Initialize a single button structure
 * 
 * @param button Pointer to button structure
 * @param gpio_pin GPIO pin number for this button
 */
void button_init_struct(button_t *button, gpio_num_t gpio_pin);

/**
 * Check if button is pressed (debounced)
 * 
 * @param button Pointer to button structure
 * @return true if valid press detected, false otherwise
 * 
 * Note: This function implements debouncing and edge detection.
 * It returns true only once per press, on the rising edge.
 */
bool button_is_pressed(button_t *button);

/**
 * Get the raw state of a button (no debouncing)
 * 
 * @param gpio_pin GPIO pin number
 * @return true if button is currently pressed, false otherwise
 */
bool button_read_raw(gpio_num_t gpio_pin);

#endif // BUTTONS_H