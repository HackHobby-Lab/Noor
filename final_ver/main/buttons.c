/**
 * buttons.c
 * Button handling implementation
 */

#include "buttons.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BUTTONS";

void buttons_init(void) {
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pin_bit_mask = ((1ULL << BTN_PLAY_PIN)  |
                         (1ULL << BTN_HOME_PIN)  |
                         (1ULL << BTN_VOLUP_PIN) |
                         (1ULL << BTN_VOLDN_PIN))
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Buttons init: Play=%d Home=%d Vol+=%d Vol-=%d",
             BTN_PLAY_PIN, BTN_HOME_PIN, BTN_VOLUP_PIN, BTN_VOLDN_PIN);
}

void button_init_struct(button_t *button, gpio_num_t gpio_pin) {
    if (!button) return;
    button->gpio       = gpio_pin;
    button->last_time  = 0;
    button->last_state = false;
}

bool button_is_pressed(button_t *button) {
    if (!button) return false;

    bool     current_level = gpio_get_level(button->gpio);
    uint32_t current_time  = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (current_level && !button->last_state) {
        if (current_time - button->last_time > BUTTON_DEBOUNCE_MS) {
            button->last_time  = current_time;
            button->last_state = current_level;
            return true;
        }
    }

    button->last_state = current_level;
    return false;
}

bool button_read_raw(gpio_num_t gpio_pin) {
    return gpio_get_level(gpio_pin);
}
