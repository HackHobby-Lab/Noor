/**
 * encoder.c
 * Rotary encoder implementation
 *
 * Changes vs previous version:
 * ----------------------------
 * 1. ISR-side timestamp: every event now carries the tick count at the
 *    moment the GPIO fired, not the moment the task processes it.
 *    Previously, multiple queued events could all pass the debounce check
 *    because they were all processed "now" — after the debounce window
 *    had already elapsed for all of them.  Timestamping in the ISR means
 *    the task sees the real arrival time of each pulse.
 *
 * 2. ENC_DEBOUNCE_MS raised to 200 in config.h (see config.h change).
 *    Logs show ghost bounces arriving up to 170ms after the real click,
 *    so 200ms debounce catches all observed cases with 30ms margin.
 *
 * 3. ENC_SETTLE_MS raised to 250 in config.h so the settle timer window
 *    always outlasts the new debounce window.
 *
 * 4. Button debounce uses the same ISR-side timestamp fix.
 *
 * 5. Queue length raised from 16 to 32 to ensure no events are dropped
 *    during fast spinning even with a full stack of ISR timestamps.
 */

#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ENCODER";

/* -------------------------------------------------------------------------
 * Internal ISR event — now carries a timestamp taken AT interrupt time.
 * ---------------------------------------------------------------------- */
typedef struct {
    encoder_event_type_t type;
    uint8_t  dt_level;          /* DT pin level at the moment CLK fired  */
    uint32_t isr_tick;          /* xTaskGetTickCountFromISR() at fire time */
} encoder_isr_event_t;

/* Queue length: raised to 32 so fast spinning never overflows */
#define ENC_ISR_QUEUE_LEN   32

static QueueHandle_t      isr_queue    = NULL;
static encoder_callback_t user_callback = NULL;

/* -------------------------------------------------------------------------
 * ISR: CLK pin — rotation detected
 * ---------------------------------------------------------------------- */
static void IRAM_ATTR encoder_isr_clk(void *arg)
{
    encoder_isr_event_t event = {
        .type     = ENC_EVENT_ROTATE,
        .dt_level = (uint8_t)gpio_get_level(ENC_DT_PIN),
        .isr_tick = xTaskGetTickCountFromISR()   /* ← timestamp at IRQ time */
    };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(isr_queue, &event, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* -------------------------------------------------------------------------
 * ISR: SW pin — button press detected
 * ---------------------------------------------------------------------- */
static void IRAM_ATTR encoder_isr_sw(void *arg)
{
    encoder_isr_event_t event = {
        .type     = ENC_EVENT_BUTTON,
        .dt_level = 0,
        .isr_tick = xTaskGetTickCountFromISR()   /* ← timestamp at IRQ time */
    };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(isr_queue, &event, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* -------------------------------------------------------------------------
 * Encoder processing task
 *
 * Key change: debounce is evaluated against event.isr_tick (when the GPIO
 * actually fired) rather than xTaskGetTickCount() (when the task woke up).
 * This closes the window where multiple stale queued events all appear
 * "fresh" because they are processed together after the debounce interval
 * has elapsed.
 * ---------------------------------------------------------------------- */
static void encoder_task(void *arg)
{
    ESP_LOGI(TAG, "Encoder task started");

    encoder_isr_event_t isr_event;
    uint32_t last_rotation_tick = 0;
    uint32_t last_button_tick   = 0;

    /* Convert ms thresholds to ticks once */
    const uint32_t rot_debounce_ticks = pdMS_TO_TICKS(ENC_DEBOUNCE_MS);
    const uint32_t btn_debounce_ticks = pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS);

    while (1) {
        if (xQueueReceive(isr_queue, &isr_event, portMAX_DELAY) != pdTRUE)
            continue;

        uint32_t event_tick = isr_event.isr_tick;   /* when GPIO actually fired */

        if (isr_event.type == ENC_EVENT_ROTATE) {

            /* Debounce: reject pulses that fired within the window of the
             * previous accepted pulse.  Both timestamps are from the ISR
             * so they reflect real hardware timing, not task scheduling. */
            uint32_t elapsed = event_tick - last_rotation_tick;
            if (elapsed < rot_debounce_ticks) {
                ESP_LOGD(TAG, "Rotation debounced (elapsed %" PRIu32 " ticks, need %" PRIu32 ")",
                         elapsed, rot_debounce_ticks);
                continue;
            }
            last_rotation_tick = event_tick;

            encoder_event_t ev;
            ev.type      = ENC_EVENT_ROTATE;
            ev.direction = (isr_event.dt_level == 0) ? ENC_DIR_CW : ENC_DIR_CCW;

            if (user_callback) user_callback(ev);

        } else if (isr_event.type == ENC_EVENT_BUTTON) {

            uint32_t elapsed = event_tick - last_button_tick;
            if (elapsed < btn_debounce_ticks) {
                ESP_LOGD(TAG, "Button debounced (elapsed %" PRIu32 " ticks, need %" PRIu32 ")",
                         elapsed, btn_debounce_ticks);
                continue;
            }
            last_button_tick = event_tick;

            encoder_event_t ev;
            ev.type      = ENC_EVENT_BUTTON;
            ev.direction = ENC_DIR_CW;  /* direction unused for button */

            if (user_callback) user_callback(ev);
        }
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

bool encoder_init(void)
{
    ESP_LOGI(TAG, "Initializing encoder...");

    /* Configure all three pins as inputs with pull-up */
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = ((1ULL << ENC_CLK_PIN) |
                         (1ULL << ENC_DT_PIN)  |
                         (1ULL << ENC_SW_PIN))
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* Create ISR event queue — larger than before to survive fast spinning */
    isr_queue = xQueueCreate(ENC_ISR_QUEUE_LEN, sizeof(encoder_isr_event_t));
    if (!isr_queue) {
        ESP_LOGE(TAG, "Failed to create encoder queue");
        return false;
    }

    /* Install GPIO ISR service (ESP_ERR_INVALID_STATE = already installed, OK) */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* CLK: rising edge = one detent step */
    gpio_set_intr_type(ENC_CLK_PIN, GPIO_INTR_POSEDGE);
    ret = gpio_isr_handler_add(ENC_CLK_PIN, encoder_isr_clk, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for CLK: %s", esp_err_to_name(ret));
        return false;
    }

    /* SW: rising edge = button release (active-low, pull-up) */
    gpio_set_intr_type(ENC_SW_PIN, GPIO_INTR_POSEDGE);
    ret = gpio_isr_handler_add(ENC_SW_PIN, encoder_isr_sw, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for SW: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Encoder initialized: CLK=%d, DT=%d, SW=%d  debounce=%dms settle=%dms",
             ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN,
             ENC_DEBOUNCE_MS, ENC_SETTLE_MS);

    return true;
}

void encoder_start_task(encoder_callback_t callback)
{
    user_callback = callback;

    xTaskCreatePinnedToCore(
        encoder_task,
        "encoder_task",
        4096,
        NULL,
        PRIORITY_ENCODER,
        NULL,
        tskNO_AFFINITY
    );

    ESP_LOGI(TAG, "Encoder task started");
}

QueueHandle_t encoder_get_queue(void)
{
    return isr_queue;
}
