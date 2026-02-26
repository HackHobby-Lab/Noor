/**
 * buttons.h
 * Button handling with debouncing for Noor Audio Player
 */

#ifndef BUTTONS_H
#define BUTTONS_H

#include "config.h"
#include <stdbool.h>
#include "driver/gpio.h"

/* GPIO pins - defined in config.h */
/* BTN_PLAY_PIN    14
   BTN_HOME_PIN    15
   BTN_VOLUP_PIN    4
   BTN_VOLDN_PIN    5  */

typedef struct {
    gpio_num_t gpio;
    uint32_t   last_time;
    bool       last_state;
} button_t;

void buttons_init(void);
void button_init_struct(button_t *button, gpio_num_t gpio_pin);
bool button_is_pressed(button_t *button);
bool button_read_raw(gpio_num_t gpio_pin);

#endif // BUTTONS_H
