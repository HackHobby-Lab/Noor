/**
 * encoder.c
 * Rotary encoder implementation
 *
 * PATCH (this revision):
 * ----------------------
 * Replaced single-edge "POSEDGE on CLK + read DT" approach with a proper
 * quadrature state-machine decoder.
 *
 * Why this was needed
 * -------------------
 * Cheap mechanical rotary encoders bounce — the internal contacts can
 * make/break the CLK or DT line multiple times for a single physical
 * click.  The old logic would see each bounce as a separate POSEDGE,
 * and even with a 200 ms time-based debounce, some bounces leaked
 * through, causing:
 *   - "skip" (one click moved 2 or 3 items)
 *   - "stuck loop around 2-3 items" (direction kept reversing)
 *   - "repeated announcement" (settle timer fired twice for one click)
 *
 * How the new decoder works
 * -------------------------
 * The encoder has 4 possible states formed by (CLK, DT): 00, 01, 10, 11.
 * One physical detent moves through 4 of these states in order.  We
 * watch every transition and use a 16-entry lookup table indexed by
 * (old_state << 2 | new_state) to decide: +1, -1, or 0 (invalid/bounce).
 *
 * We accumulate these and only emit ONE event when the accumulator
 * reaches +/-4 — which corresponds to exactly one full detent click.
 * Bounces show up as 0 in the table or as +/-1 oscillations that never
 * reach 4, so they self-cancel.  Result: 1 physical click = exactly 1
 * event, regardless of contact bouncing.
 *
 * Both pins now use ANYEDGE interrupts so we see every transition.
 * The button (SW pin) is unchanged.
 */

#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ENCODER";

/* -------------------------------------------------------------------------
 * Internal ISR event
 * ---------------------------------------------------------------------- */
typedef struct {
    encoder_event_type_t type;
    encoder_direction_t  dir;       /* used only for ROTATE */
    uint32_t             isr_tick;
} encoder_isr_event_t;

#define ENC_ISR_QUEUE_LEN   32

static QueueHandle_t      isr_queue     = NULL;
static encoder_callback_t user_callback = NULL;

/* -------------------------------------------------------------------------
 * Quadrature state-machine state — accessed only from ISR + decoder
 * ---------------------------------------------------------------------- */

/*
 * Lookup table for one quadrature transition.
 *
 * Index = (old_state << 2) | new_state, where state = (CLK<<1) | DT.
 *
 *   Entry +1 = CW step
 *   Entry -1 = CCW step
 *   Entry  0 = invalid transition (noise / bounce)
 *
 * This is the standard "1 increment per quadrature edge" decoder.
 * 4 valid edges = 1 full detent.
 */
static const int8_t qdec_table[16] = {
    /* old->new   00  01  10  11   */
    /* 00 */       0, +1, -1,  0,
    /* 01 */      -1,  0,  0, +1,
    /* 10 */      +1,  0,  0, -1,
    /* 11 */       0, -1, +1,  0
};

static volatile uint8_t  q_last_state = 0;   /* (CLK<<1)|DT  at last ISR */
static volatile int8_t   q_accum     = 0;   /* +/- accumulator           */

/* -------------------------------------------------------------------------
 * ISR: CLK or DT changed — quadrature update
 * ---------------------------------------------------------------------- */
static void IRAM_ATTR encoder_isr_quad(void *arg)
{
    uint8_t clk = (uint8_t)gpio_get_level(ENC_CLK_PIN);
    uint8_t dt  = (uint8_t)gpio_get_level(ENC_DT_PIN);
    uint8_t new_state = (uint8_t)((clk << 1) | dt);

    /* Same as last sample (could happen on glitches) — ignore */
    if (new_state == q_last_state) return;

    uint8_t idx = (uint8_t)((q_last_state << 2) | new_state);
    int8_t  inc = qdec_table[idx & 0x0F];
    q_last_state = new_state;

    if (inc == 0) return;       /* invalid transition / bounce */

    q_accum = (int8_t)(q_accum + inc);

    /* One full detent reached?  Emit ONE event.
     *
     * ENCODER_INVERT_DIRECTION (from config.h) lets the integrator flip
     * CW <-> CCW with a single rebuild if the encoder's physical wiring
     * makes clockwise rotation produce -4 in this table instead of +4.
     * The check is done with a compile-time #if so there is zero runtime
     * cost. */
    if (q_accum >= 4) {
        q_accum = 0;
        encoder_isr_event_t event = {
            .type     = ENC_EVENT_ROTATE,
#if ENCODER_INVERT_DIRECTION
            .dir      = ENC_DIR_CCW,
#else
            .dir      = ENC_DIR_CW,
#endif
            .isr_tick = xTaskGetTickCountFromISR()
        };
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(isr_queue, &event, &woken);
        if (woken) portYIELD_FROM_ISR();
    } else if (q_accum <= -4) {
        q_accum = 0;
        encoder_isr_event_t event = {
            .type     = ENC_EVENT_ROTATE,
#if ENCODER_INVERT_DIRECTION
            .dir      = ENC_DIR_CW,
#else
            .dir      = ENC_DIR_CCW,
#endif
            .isr_tick = xTaskGetTickCountFromISR()
        };
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(isr_queue, &event, &woken);
        if (woken) portYIELD_FROM_ISR();
    }
}

/* -------------------------------------------------------------------------
 * ISR: SW pin — encoder button press (unchanged)
 * ---------------------------------------------------------------------- */
static void IRAM_ATTR encoder_isr_sw(void *arg)
{
    encoder_isr_event_t event = {
        .type     = ENC_EVENT_BUTTON,
        .dir      = ENC_DIR_CW,
        .isr_tick = xTaskGetTickCountFromISR()
    };
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(isr_queue, &event, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* -------------------------------------------------------------------------
 * Encoder processing task
 *
 * The decoder emits at most one event per physical detent, so heavy
 * software debouncing is no longer required.  We still apply a small
 * button debounce because the SW switch is a plain mechanical contact.
 * ---------------------------------------------------------------------- */
static void encoder_task(void *arg)
{
    ESP_LOGI(TAG, "Encoder task started (quadrature decoder, debounce=%dms btn)",
             BUTTON_DEBOUNCE_MS);

    encoder_isr_event_t isr_event;
    uint32_t last_button_tick = 0;
    const uint32_t btn_debounce_ticks = pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS);

    while (1) {
        if (xQueueReceive(isr_queue, &isr_event, portMAX_DELAY) != pdTRUE)
            continue;

        if (isr_event.type == ENC_EVENT_ROTATE) {

            /* Decoder already filtered bounces.  Pass straight through. */
            encoder_event_t ev;
            ev.type      = ENC_EVENT_ROTATE;
            ev.direction = isr_event.dir;
            if (user_callback) user_callback(ev);

        } else if (isr_event.type == ENC_EVENT_BUTTON) {

            uint32_t elapsed = isr_event.isr_tick - last_button_tick;
            if (elapsed < btn_debounce_ticks) continue;
            last_button_tick = isr_event.isr_tick;

            encoder_event_t ev;
            ev.type      = ENC_EVENT_BUTTON;
            ev.direction = ENC_DIR_CW;  /* unused for button */
            if (user_callback) user_callback(ev);
        }
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

bool encoder_init(void)
{
    ESP_LOGI(TAG, "Initializing encoder (quadrature decoder)...");

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

    isr_queue = xQueueCreate(ENC_ISR_QUEUE_LEN, sizeof(encoder_isr_event_t));
    if (!isr_queue) {
        ESP_LOGE(TAG, "Failed to create encoder queue");
        return false;
    }

    /* Prime quadrature state with the current pin reading so the first
     * transition after boot doesn't generate a phantom step. */
    {
        uint8_t clk = (uint8_t)gpio_get_level(ENC_CLK_PIN);
        uint8_t dt  = (uint8_t)gpio_get_level(ENC_DT_PIN);
        q_last_state = (uint8_t)((clk << 1) | dt);
        q_accum      = 0;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* CLK and DT: ANY EDGE — the quadrature decoder sees every transition. */
    gpio_set_intr_type(ENC_CLK_PIN, GPIO_INTR_ANYEDGE);
    ret = gpio_isr_handler_add(ENC_CLK_PIN, encoder_isr_quad, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for CLK: %s", esp_err_to_name(ret));
        return false;
    }

    gpio_set_intr_type(ENC_DT_PIN, GPIO_INTR_ANYEDGE);
    ret = gpio_isr_handler_add(ENC_DT_PIN, encoder_isr_quad, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for DT: %s", esp_err_to_name(ret));
        return false;
    }

    /* SW: rising edge (active-low, pull-up) = release */
    gpio_set_intr_type(ENC_SW_PIN, GPIO_INTR_POSEDGE);
    ret = gpio_isr_handler_add(ENC_SW_PIN, encoder_isr_sw, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for SW: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Encoder initialized: CLK=%d, DT=%d, SW=%d  (quadrature)",
             ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN);

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
