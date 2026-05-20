/**
 * headphone_detect.c
 * Headphone jack detection for Noor Audio Player
 *
 * PATCHED to harden against transient-induced reboots when the jack is
 * removed.
 *
 * Changes vs original:
 *   1. Monitor task stack increased from 2048 -> 4096 bytes.
 *      ESP_LOGI from inside the polling loop occasionally pushed the
 *      original 2048-byte stack close to its limit, especially when a
 *      burst of jack transitions arrives during unplug arcing.
 *   2. Two-sample debounce replaced with a 3-of-3 stable read window.
 *      A jack switch contact opening can chatter for tens of ms;
 *      the original logic accepted the second sample as truth.  The
 *      new logic requires three consecutive matching samples before
 *      committing to a state change, which filters arcing transients
 *      at the cost of ~600 ms total response time (still imperceptible).
 *   3. Volume changes still go through audio_set_volume_silent() —
 *      no I2S touched here, no announcement queued.  This keeps the
 *      hp_detect task fully decoupled from audio_task.
 */

#include "headphone_detect.h"
#include "audio.h"          /* audio_set_volume_silent() */
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HP_DETECT";

/* Stack size for the polling task — raised from 2048 in original code. */
#define HP_DETECT_TASK_STACK    4096

/* Stable-read window: how many consecutive matching samples are needed
 * before a state change is committed.  3 samples * HP_DETECT_POLL_MS gives
 * the total settle time (with HP_DETECT_POLL_MS = 200 ms => 600 ms). */
#define HP_DETECT_STABLE_SAMPLES  3

/* -------------------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------------- */
static bool s_enabled   = false;   /* true once init succeeded         */
static bool s_connected = false;   /* last committed headphone state   */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

/** Read the raw GPIO level and return true if headphones are inserted.
 *  Hardware: jack detect switch pulls GPIO48 LOW when plug is inserted. */
static inline bool read_jack_inserted(void)
{
    return gpio_get_level(HEADPHONE_DETECT_PIN) == 0;
}

/** Apply the correct volume for the given headphone state. */
static void apply_state(bool inserted, bool log_it)
{
    if (inserted) {
        audio_set_volume_silent(VOLUME_HEADPHONE);
#if SPEAKER_ENABLE_PIN >= 0
        gpio_set_level(SPEAKER_ENABLE_PIN, 0);   /* mute speaker amp */
#endif
        if (log_it)
            ESP_LOGI(TAG, "Headphones inserted -> vol %d%%", VOLUME_HEADPHONE);
    } else {
        audio_set_volume_silent(VOLUME_SPEAKER);
#if SPEAKER_ENABLE_PIN >= 0
        gpio_set_level(SPEAKER_ENABLE_PIN, 1);   /* enable speaker amp */
#endif
        if (log_it)
            ESP_LOGI(TAG, "Headphones removed  -> vol %d%%", VOLUME_SPEAKER);
    }
}

/* -------------------------------------------------------------------------
 * Monitoring task (3-of-3 stable read debounce)
 * ---------------------------------------------------------------------- */
static void headphone_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "Headphone monitoring task started (stable=%d samples, poll=%d ms)",
             HP_DETECT_STABLE_SAMPLES, HP_DETECT_POLL_MS);

    /* Track the candidate state and how many consecutive samples have
     * agreed with it.  When candidate != s_connected and stable_count
     * reaches HP_DETECT_STABLE_SAMPLES, commit the change. */
    bool    candidate    = s_connected;
    uint8_t stable_count = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HP_DETECT_POLL_MS));

        bool raw = read_jack_inserted();

        if (raw == candidate) {
            if (stable_count < HP_DETECT_STABLE_SAMPLES)
                stable_count++;
        } else {
            candidate    = raw;
            stable_count = 1;   /* first sample of a new candidate */
        }

        if (stable_count >= HP_DETECT_STABLE_SAMPLES &&
            candidate    != s_connected) {

            s_connected = candidate;
            apply_state(s_connected, true);

            ESP_LOGI(TAG, "Jack state committed: GPIO%d=%d (%s)",
                     HEADPHONE_DETECT_PIN,
                     (int)raw,
                     raw ? "inserted" : "removed");
        }
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

bool headphone_detect_init(void)
{
    if (HEADPHONE_DETECT_PIN < 0) {
        ESP_LOGW(TAG, "Headphone detection disabled (pin=-1)");
        return false;
    }

    ESP_LOGI(TAG, "Initializing headphone detection...");

    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << HEADPHONE_DETECT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed for pin %d: %s",
                 HEADPHONE_DETECT_PIN, esp_err_to_name(ret));
        return false;
    }

    if (SPEAKER_ENABLE_PIN >= 0) {
        unsigned spk_pin = (unsigned)SPEAKER_ENABLE_PIN;
        gpio_config_t spk_cfg = {
            .pin_bit_mask = (1ULL << spk_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&spk_cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Speaker enable pin %d config failed: %s",
                     SPEAKER_ENABLE_PIN, esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Speaker enable pin: GPIO%d", SPEAKER_ENABLE_PIN);
        }
    }

    s_connected = read_jack_inserted();

    ESP_LOGI(TAG, "Initial GPIO%d state: %d (%s)",
             HEADPHONE_DETECT_PIN,
             gpio_get_level(HEADPHONE_DETECT_PIN),
             s_connected ? "headphones" : "speaker");

    apply_state(s_connected, false);

    ESP_LOGI(TAG, "Headphone detection initialized (detect=%d, enable=%d)",
             HEADPHONE_DETECT_PIN, SPEAKER_ENABLE_PIN);

    s_enabled = true;
    return true;
}

bool headphone_detect_start_task(void)
{
    if (!s_enabled) {
        ESP_LOGW(TAG, "Cannot start task -- not initialized");
        return false;
    }

    BaseType_t result = xTaskCreate(
        headphone_monitor_task,
        "hp_detect",
        HP_DETECT_TASK_STACK,
        NULL,
        PRIORITY_HEADPHONE,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create headphone monitor task");
        return false;
    }

    ESP_LOGI(TAG, "Headphone monitoring task started (stack=%d)",
             HP_DETECT_TASK_STACK);
    return true;
}

bool headphone_detect_is_enabled(void)   { return s_enabled; }
bool headphone_detect_is_connected(void) { return s_connected; }
