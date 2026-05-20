/**
 * buttons.h
 * Button handling with debouncing for Noor Audio Player
 *
 * PATCHED FOR ACTIVE-LOW WIRING (pull-up enabled internally; pressed = LOW).
 * No API changes vs original.
 */

#ifndef BUTTONS_H
#define BUTTONS_H

#include "config.h"
#include <stdbool.h>
#include "driver/gpio.h"

/* Button GPIO pins (active-LOW with internal pull-up) */
#define BTN_PLAY_PIN      14
#define BTN_HOME_PIN      15
#define BTN_VOL_UP_PIN     4
#define BTN_VOL_DOWN_PIN   5

/* Debounce time in milliseconds */
#define BUTTON_DEBOUNCE_MS  50

/**
 * Button structure for state tracking.
 * last_state stores the raw GPIO level on the previous read
 * (HIGH = idle, LOW = pressed).
 */
typedef struct {
    gpio_num_t gpio;
    uint32_t   last_time;   /* ms tick at last accepted press           */
    bool       last_state;  /* raw GPIO level on previous read (true=HIGH) */
} button_t;

void buttons_init(void);
void button_init_struct(button_t *button, gpio_num_t gpio_pin);
bool button_is_pressed(button_t *button);
bool button_read_raw(gpio_num_t gpio_pin);

#endif /* BUTTONS_H */
