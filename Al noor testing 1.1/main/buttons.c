/**
 * buttons.c
 * Button handling implementation
 *
 * PATCHED FOR ACTIVE-LOW WIRING
 * -----------------------------
 * The PCB ships with all four buttons wired to GND through a momentary
 * switch (one terminal = GPIO, other terminal = GND).  Original firmware
 * was configured for the opposite (active-HIGH, button to 3.3V), which is
 * why no presses were detected.
 *
 * Changes vs original:
 *   1. gpio_config: PULLUP_ENABLE + PULLDOWN_DISABLE
 *      Idle level = HIGH (pulled up internally), pressed level = LOW.
 *   2. button_is_pressed: detect FALLING edge (HIGH -> LOW) instead of
 *      rising edge.
 *
 * Behaviour change for the rest of the firmware: NONE.
 * button_is_pressed() still returns true once per press, on the press edge.
 */

#include "buttons.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BUTTONS";

void buttons_init(void) {
    ESP_LOGI(TAG, "Initializing buttons (active-LOW, pull-up)...");

    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,     /* idle = HIGH (pulled up) */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  /* button to GND on press  */
        .pin_bit_mask = ((1ULL << BTN_PLAY_PIN)    |
                         (1ULL << BTN_HOME_PIN)    |
                         (1ULL << BTN_VOL_UP_PIN)  |
                         (1ULL << BTN_VOL_DOWN_PIN))
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Buttons initialized: Play=%d, Home=%d, Vol+=%d, Vol-=%d",
             BTN_PLAY_PIN, BTN_HOME_PIN, BTN_VOL_UP_PIN, BTN_VOL_DOWN_PIN);
}

void button_init_struct(button_t *button, gpio_num_t gpio_pin) {
    if (!button) return;

    button->gpio       = gpio_pin;
    button->last_time  = 0;
    /* last_state semantics: track raw GPIO level, not "pressed-ness".
     * Initialise to HIGH because that is the idle level with pull-up. */
    button->last_state = true;
}

bool button_is_pressed(button_t *button) {
    if (!button) return false;

    /* Read raw GPIO level: HIGH = idle, LOW = pressed (active-LOW). */
    bool current_level = gpio_get_level(button->gpio);

    /* Current time in ms (tick count is monotonic, wraparound-safe by
     * unsigned subtraction). */
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /* Falling edge: line was HIGH, now LOW => press just happened. */
    if (!current_level && button->last_state) {
        if (current_time - button->last_time > BUTTON_DEBOUNCE_MS) {
            button->last_time  = current_time;
            button->last_state = current_level;   /* now LOW */
            return true;
        }
    }

    button->last_state = current_level;
    return false;
}

bool button_read_raw(gpio_num_t gpio_pin) {
    /* For active-LOW: pressed when level == 0.  Return true when pressed
     * so callers see consistent semantics regardless of wiring. */
    return gpio_get_level(gpio_pin) == 0;
}
