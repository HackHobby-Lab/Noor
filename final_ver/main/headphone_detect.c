/**
 * headphone_detect.c
 * Headphone detection with proper debounce and volume lock
 *
 * FIXES applied:
 * 1. GPIO48 debounce: pin must stay stable 300ms before acting
 * 2. Volume lock: HP task ignores volume for 3s after user presses vol button
 */

#include "headphone_detect.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "audio.h"

static const char *TAG = "HP_DETECT";

static bool          is_enabled           = false;
static volatile bool headphones_connected = false;

/* Tick of last user-triggered volume change.
 * HP task will not override volume for HP_VOLUME_LOCK_MS after this. */
static volatile TickType_t g_user_volume_tick = 0;

#define HP_VOLUME_LOCK_MS   3000   /* ignore HP volume change for 3s after user press */
#define HP_DEBOUNCE_MS       300   /* pin must be stable this long before we act */
#define HP_POLL_MS            50   /* task poll interval */

/* Call this from main.c whenever user presses vol up/down button */
void headphone_detect_notify_user_volume(void) {
    g_user_volume_tick = xTaskGetTickCount();
}

/* ---------------------------------------------------------- */
static void headphone_monitor_task(void *arg) {
    bool     confirmed   = gpio_get_level(HEADPHONE_DETECT_PIN);
    bool     candidate   = confirmed;
    TickType_t stable_at = xTaskGetTickCount();

    headphones_connected = confirmed;
    ESP_LOGI(TAG, "HP monitor started. GPIO%d=%d", HEADPHONE_DETECT_PIN, confirmed);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HP_POLL_MS));

        bool reading = gpio_get_level(HEADPHONE_DETECT_PIN);

        if (reading != candidate) {
            /* New candidate state - restart stability timer */
            candidate  = reading;
            stable_at  = xTaskGetTickCount();
            continue;
        }

        /* How long has candidate been stable? */
        uint32_t stable_ms = (xTaskGetTickCount() - stable_at) * portTICK_PERIOD_MS;
        if (stable_ms < HP_DEBOUNCE_MS) continue;   /* not stable enough yet */

        /* Already confirmed this state - no change */
        if (candidate == confirmed) continue;

        /* ----- State truly changed ----- */
        confirmed            = candidate;
        headphones_connected = confirmed;
        ESP_LOGI(TAG, "HP confirmed: %s", confirmed ? "INSERTED" : "REMOVED");

        /* Check volume lock - do NOT override if user just changed volume */
        uint32_t ms_since_user =
            (xTaskGetTickCount() - g_user_volume_tick) * portTICK_PERIOD_MS;

        if (ms_since_user < HP_VOLUME_LOCK_MS) {
            ESP_LOGI(TAG, "HP: skipping volume (user changed %lums ago)", ms_since_user);
            continue;
        }

        /* Safe to set volume based on jack state */
        if (confirmed) {
            if (SPEAKER_ENABLE_PIN >= 0) gpio_set_level(SPEAKER_ENABLE_PIN, 0);
            audio_set_volume(30);
            ESP_LOGI(TAG, "Headphones IN -> Vol 30%%");
        } else {
            if (SPEAKER_ENABLE_PIN >= 0) gpio_set_level(SPEAKER_ENABLE_PIN, 1);
            audio_set_volume(70);
            ESP_LOGI(TAG, "Headphones OUT -> Vol 70%%");
        }
    }
}

/* ---------------------------------------------------------- */
bool headphone_detect_init(void) {
    if (HEADPHONE_DETECT_PIN < 0) {
        ESP_LOGI(TAG, "HP detect disabled (pin not configured)");
        return false;
    }

    /* Guard: function returns early above if pin < 0, but compiler still
     * evaluates the shift expression. Use max() to keep value non-negative. */
    gpio_config_t detect_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << (HEADPHONE_DETECT_PIN >= 0 ? HEADPHONE_DETECT_PIN : 0))
    };
    if (gpio_config(&detect_conf) != ESP_OK) { is_enabled = false; return false; }

    if (SPEAKER_ENABLE_PIN >= 0) {
        gpio_config_t spk_conf = {
            .intr_type    = GPIO_INTR_DISABLE,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pin_bit_mask = (1ULL << (SPEAKER_ENABLE_PIN >= 0 ? SPEAKER_ENABLE_PIN : 0))
        };
        gpio_config(&spk_conf);
        gpio_set_level(SPEAKER_ENABLE_PIN, 1);
    }

    bool initial = gpio_get_level(HEADPHONE_DETECT_PIN);
    headphones_connected = initial;

    if (initial) {
        audio_set_volume(30);
        ESP_LOGI(TAG, "HP at startup -> Vol 30%%");
    } else {
        audio_set_volume(70);
        ESP_LOGI(TAG, "No HP at startup -> Vol 70%%");
    }

    is_enabled = true;
    ESP_LOGI(TAG, "HP detect init OK (pin=%d)", HEADPHONE_DETECT_PIN);
    return true;
}

bool headphone_detect_start_task(void) {
    if (!is_enabled) return false;
    xTaskCreatePinnedToCore(headphone_monitor_task, "hp_detect", 2048,
                            NULL, PRIORITY_HEADPHONE, NULL, tskNO_AFFINITY);
    return true;
}

bool headphone_detect_is_enabled(void)   { return is_enabled; }
bool headphone_detect_is_connected(void) { return headphones_connected; }
