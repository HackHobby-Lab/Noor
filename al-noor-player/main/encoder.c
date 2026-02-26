/**
 * encoder.c
 * Rotary encoder implementation
 */

#include "encoder.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "ENCODER";

typedef struct {
    encoder_event_type_t type;
    uint8_t              dt_level;
} encoder_isr_event_t;

static QueueHandle_t      isr_queue    = NULL;
static encoder_callback_t user_callback = NULL;

static void IRAM_ATTR encoder_isr_clk(void *arg) {
    encoder_isr_event_t event = {
        .type     = ENC_EVENT_ROTATE,
        .dt_level = (uint8_t)gpio_get_level(ENC_DT_PIN)
    };
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(isr_queue, &event, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR encoder_isr_sw(void *arg) {
    encoder_isr_event_t event = { .type = ENC_EVENT_BUTTON, .dt_level = 0 };
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(isr_queue, &event, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static void encoder_task(void *arg) {
    encoder_isr_event_t isr_event;
    uint32_t last_rotate_time = 0;
    uint32_t last_button_time = 0;

    while (1) {
        if (xQueueReceive(isr_queue, &isr_event, portMAX_DELAY) != pdTRUE) continue;

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (isr_event.type == ENC_EVENT_ROTATE) {
            if (now - last_rotate_time < ENC_DEBOUNCE_MS) continue;
            last_rotate_time = now;

            encoder_event_t event = {
                .type      = ENC_EVENT_ROTATE,
                .direction = (isr_event.dt_level == 0) ? ENC_DIR_CW : ENC_DIR_CCW
            };
            if (user_callback) user_callback(event);

        } else if (isr_event.type == ENC_EVENT_BUTTON) {
            if (now - last_button_time < BUTTON_DEBOUNCE_MS) continue;
            last_button_time = now;

            encoder_event_t event = { .type = ENC_EVENT_BUTTON, .direction = ENC_DIR_CW };
            if (user_callback) user_callback(event);
        }
    }
}

bool encoder_init(void) {
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

    isr_queue = xQueueCreate(ENC_QUEUE_LEN, sizeof(encoder_isr_event_t));
    if (!isr_queue) return false;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return false;

    gpio_set_intr_type(ENC_CLK_PIN, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(ENC_CLK_PIN, encoder_isr_clk, NULL);

    gpio_set_intr_type(ENC_SW_PIN, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(ENC_SW_PIN, encoder_isr_sw, NULL);

    ESP_LOGI(TAG, "Encoder init: CLK=%d DT=%d SW=%d",
             ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN);
    return true;
}

void encoder_start_task(encoder_callback_t callback) {
    user_callback = callback;
    xTaskCreatePinnedToCore(encoder_task, "encoder_task", 4096,
                            NULL, PRIORITY_ENCODER, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "Encoder task started");
}

QueueHandle_t encoder_get_queue(void) {
    return isr_queue;
}
