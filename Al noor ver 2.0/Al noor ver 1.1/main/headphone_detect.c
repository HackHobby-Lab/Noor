/**
 * headphone_detect.c
 * Headphone jack detection for Noor Audio Player
 *
 * Hardware (GPIO48):
 *   HIGH = headphone/AUX plug inserted  → lower volume, mute speaker
 *   LOW  = no plug (speaker mode)       → restore speaker volume
 *
 * When SPEAKER_ENABLE_PIN >= 0:
 *   The pin is driven HIGH to enable the speaker amplifier and LOW to
 *   disable it (mute speaker when headphones are plugged in).
 *   If SPEAKER_ENABLE_PIN == -1 the pin is simply not used.
 *
 * Volume behaviour:
 *   Headphones inserted  → VOLUME_HEADPHONE  (default 30%)
 *   Speaker (no phones)  → VOLUME_SPEAKER    (default 60%)
 *
 * All constants come from config.h — do NOT redefine them here.
 *
 * FIX CRASH: apply_state() now calls audio_set_volume_silent() instead of
 * audio_set_volume(). The old audio_set_volume() called announce_volume_percent()
 * which internally called i2s_write() from the hp_detect task context. This
 * collided with audio_task owning the I2S mutex and caused:
 *   assert failed: xTaskPriorityDisinherit tasks.c:5147
 * audio_set_volume_silent() only writes the volume integer — no I2S access,
 * no announcement, safe to call from any task context.
 */

#include "headphone_detect.h"
#include "audio.h"          /* audio_set_volume_silent() */
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HP_DETECT";

/* -------------------------------------------------------------------------
 * Module state
 * ---------------------------------------------------------------------- */
static bool s_enabled     = false;   /* true once init succeeded          */
static bool s_connected   = false;   /* last known headphone state        */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

/** Read the raw GPIO level and return true if headphones are inserted.
 *  Hardware: jack detect switch pulls GPIO48 LOW when plug is inserted.
 *  GPIO48 floats HIGH (pulled up) when nothing is connected. */
static inline bool read_jack_inserted(void)
{
    /* GPIO48 LOW = jack inserted (plug grounds the detect pin) */
    return gpio_get_level(HEADPHONE_DETECT_PIN) == 0;
}

/** Apply the correct volume and (if wired) speaker-enable GPIO for the
 *  given headphone state.  Logs the transition.
 *
 *  FIX CRASH: Uses audio_set_volume_silent() instead of audio_set_volume().
 *  audio_set_volume() called announce_volume_percent() → i2s_write() from
 *  the hp_detect task, which crashed with xTaskPriorityDisinherit assert
 *  because audio_task already owned the I2S driver mutex.
 *  audio_set_volume_silent() is safe from any task — it only sets the
 *  volume integer with no I2S or announcement side-effects. */
static void apply_state(bool inserted, bool log_it)
{
    if (inserted) {
        /* FIX CRASH: was audio_set_volume(VOLUME_HEADPHONE) — caused assert crash */
        audio_set_volume_silent(VOLUME_HEADPHONE);
#if SPEAKER_ENABLE_PIN >= 0
        gpio_set_level(SPEAKER_ENABLE_PIN, 0);   /* disable speaker amp */
#endif
        if (log_it)
            ESP_LOGI(TAG, "Headphones inserted → vol %d%%", VOLUME_HEADPHONE);
    } else {
        /* FIX CRASH: was audio_set_volume(VOLUME_SPEAKER) — caused assert crash */
        audio_set_volume_silent(VOLUME_SPEAKER);
#if SPEAKER_ENABLE_PIN >= 0
        gpio_set_level(SPEAKER_ENABLE_PIN, 1);   /* enable speaker amp */
#endif
        if (log_it)
            ESP_LOGI(TAG, "Headphones removed  → vol %d%%", VOLUME_SPEAKER);
    }
}

/* -------------------------------------------------------------------------
 * Monitoring task
 *
 * Polls GPIO48 every HP_DETECT_POLL_MS ms.  Uses a simple software
 * debounce: two consecutive identical readings are required before a
 * state change is accepted.
 * ---------------------------------------------------------------------- */
static void headphone_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "Headphone monitoring task started");

    /* Two-sample debounce state */
    bool prev_raw  = s_connected;
    bool candidate = s_connected;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HP_DETECT_POLL_MS));

        bool raw = read_jack_inserted();

        if (raw == prev_raw && raw != s_connected) {
            /* Two consistent readings that differ from current state */
            s_connected = raw;
            apply_state(s_connected, true);

            ESP_LOGI(TAG, "Jack state changed: GPIO%d=%d (%s)",
                     HEADPHONE_DETECT_PIN,
                     (int)raw,
                     raw ? "inserted" : "removed");
        }

        prev_raw  = candidate;
        candidate = raw;
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

    /* ---- Configure detect pin ---- */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << HEADPHONE_DETECT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,    /* pull-up so floating = LOW = speaker */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed for pin %d: %s",
                 HEADPHONE_DETECT_PIN, esp_err_to_name(ret));
        return false;
    }

    /* ---- Configure speaker-enable pin (if wired) ---- */
    /* SPEAKER_ENABLE_PIN == -1 means "not connected" — skip entirely */
    if (SPEAKER_ENABLE_PIN >= 0) {
        /* Use a local variable so the compiler doesn't evaluate the shift
         * for the -1 case and emit a shift-count-overflow warning. */
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
            /* Non-fatal — continue without speaker control */
        } else {
            ESP_LOGI(TAG, "Speaker enable pin: GPIO%d", SPEAKER_ENABLE_PIN);
        }
    }

    /* ---- Read initial state ---- */
    s_connected = read_jack_inserted();

    ESP_LOGI(TAG, "Initial GPIO%d state: %d (%s)",
             HEADPHONE_DETECT_PIN,
             gpio_get_level(HEADPHONE_DETECT_PIN),
             s_connected ? "headphones" : "speaker");

    /* Apply initial volume and speaker state
     * FIX CRASH: apply_state() now uses audio_set_volume_silent() internally */
    if (s_connected) {
        ESP_LOGI(TAG, "Headphones detected at startup (GPIO%d=LOW) -> Vol %d%%",
                 HEADPHONE_DETECT_PIN, VOLUME_HEADPHONE);
    } else {
        ESP_LOGI(TAG, "No headphones at startup (GPIO%d=HIGH) -> Vol %d%%",
                 HEADPHONE_DETECT_PIN, VOLUME_SPEAKER);
    }
    apply_state(s_connected, false);

    ESP_LOGI(TAG, "Headphone detection initialized (detect=%d, enable=%d)",
             HEADPHONE_DETECT_PIN, SPEAKER_ENABLE_PIN);

    s_enabled = true;
    return true;
}

bool headphone_detect_start_task(void)
{
    if (!s_enabled) {
        ESP_LOGW(TAG, "Cannot start task — not initialized");
        return false;
    }

    BaseType_t result = xTaskCreate(
        headphone_monitor_task,
        "hp_detect",
        2048,
        NULL,
        PRIORITY_HEADPHONE,
        NULL
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create headphone monitor task");
        return false;
    }

    ESP_LOGI(TAG, "Headphone monitoring task started");
    return true;
}

bool headphone_detect_is_enabled(void)
{
    return s_enabled;
}

bool headphone_detect_is_connected(void)
{
    return s_connected;
}
