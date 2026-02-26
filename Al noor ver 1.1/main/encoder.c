/**
 * encoder.c
 * Rotary encoder implementation
 */

#include "encoder.h"
#include "freertos/task.h"
#include "esp_log.h"

 static const char *TAG = "ENCODER";

 /* Internal queue for ISR events */
 typedef struct {
	 encoder_event_type_t type;
	 uint8_t dt_level; // For rotation events
} encoder_isr_event_t;

static QueueHandle_t isr_queue = NULL;
static encoder_callback_t user_callback = NULL;

/* ISR Handler for CLK pin (rotation detection) */
static void IRAM_ATTR encoder_isr_clk(void *arg) {
    uint32_t dt_level = gpio_get_level(ENC_DT_PIN);
    encoder_isr_event_t event = {
        .type = ENC_EVENT_ROTATE,
        .dt_level = (uint8_t)dt_level
    };
    
    BaseType_t high_priority_task_woken = pdFALSE;
    xQueueSendFromISR(isr_queue, &event, &high_priority_task_woken);
    
    if (high_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* ISR Handler for SW pin (button press detection) */
static void IRAM_ATTR encoder_isr_sw(void *arg) {
    encoder_isr_event_t event = {
        .type = ENC_EVENT_BUTTON,
        .dt_level = 0
    };
    
    BaseType_t high_priority_task_woken = pdFALSE;
    xQueueSendFromISR(isr_queue, &event, &high_priority_task_woken);
    
    if (high_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* Encoder processing task */
static void encoder_task(void *arg) {
    ESP_LOGI(TAG, "Encoder task started");
    
    encoder_isr_event_t isr_event;
    uint32_t last_rotation_time = 0;
    uint32_t last_button_time = 0;
    
    while (1) {
        // Wait for event from ISR
        if (xQueueReceive(isr_queue, &isr_event, portMAX_DELAY) == pdTRUE) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            if (isr_event.type == ENC_EVENT_ROTATE) {
                // Debounce rotation
                if (now - last_rotation_time < ENC_DEBOUNCE_MS) {
                    continue;
                }
                last_rotation_time = now;
                
                // Determine direction based on DT level when CLK triggered
                encoder_event_t event;
                event.type = ENC_EVENT_ROTATE;
                
                if (isr_event.dt_level == 0) {
                    event.direction = ENC_DIR_CW;   // Clockwise
                } else {
                    event.direction = ENC_DIR_CCW;  // Counter-clockwise
                }
                
                // Call user callback
                if (user_callback) {
                    user_callback(event);
                }
                
            } else if (isr_event.type == ENC_EVENT_BUTTON) {
                // Debounce button
                if (now - last_button_time < BUTTON_DEBOUNCE_MS) {
                    continue;
                }
                last_button_time = now;
                
                // Call user callback
                if (user_callback) {
                    encoder_event_t event;
                    event.type = ENC_EVENT_BUTTON;
                    event.direction = ENC_DIR_CW; // Don't care for button events
                    user_callback(event);
                }
            }
        }
    }
}

bool encoder_init(void) {
    ESP_LOGI(TAG, "Initializing encoder...");
    
    // Configure encoder pins as inputs with pull-up
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = ((1ULL << ENC_CLK_PIN) | 
                        (1ULL << ENC_DT_PIN) | 
                        (1ULL << ENC_SW_PIN))
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Create ISR event queue
    isr_queue = xQueueCreate(ENC_QUEUE_LEN, sizeof(encoder_isr_event_t));
    if (!isr_queue) {
        ESP_LOGE(TAG, "Failed to create encoder queue");
        return false;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means already installed (OK)
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Setup interrupts for CLK pin (rising edge = rotation step)
    gpio_set_intr_type(ENC_CLK_PIN, GPIO_INTR_POSEDGE);
    ret = gpio_isr_handler_add(ENC_CLK_PIN, encoder_isr_clk, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for CLK: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Setup interrupts for SW pin (rising edge = button press)
    gpio_set_intr_type(ENC_SW_PIN, GPIO_INTR_POSEDGE);
    ret = gpio_isr_handler_add(ENC_SW_PIN, encoder_isr_sw, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR for SW: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(TAG, "Encoder initialized: CLK=%d, DT=%d, SW=%d",
             ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN);
    
    return true;
}

void encoder_start_task(encoder_callback_t callback) {
    user_callback = callback;
    
    xTaskCreatePinnedToCore(
        encoder_task,
        "encoder_task",
        4096,
        NULL,
        3,  // Priority
        NULL,
        tskNO_AFFINITY
    );
    
    ESP_LOGI(TAG, "Encoder task started");
}

QueueHandle_t encoder_get_queue(void) {
    return isr_queue;
}