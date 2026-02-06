/**
 * headphone_detect.c
 * Headphone detection implementation
 */

#include "headphone_detect.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "audio.h"

static const char *TAG = "HP_DETECT";

static bool is_enabled = false;
static volatile bool headphones_connected = false;

/* Headphone monitoring task */
static void headphone_monitor_task(void *arg) {
    ESP_LOGI(TAG, "Headphone monitoring task started");
    
    bool last_state = gpio_get_level(HEADPHONE_DETECT_PIN);
    ESP_LOGI(TAG, "Initial GPIO48 state: %d (HIGH=1/jack-inserted, LOW=0/no-jack)", last_state);
    
    while (1) {
        bool current_state = gpio_get_level(HEADPHONE_DETECT_PIN);
        
        if (current_state != last_state) {
            last_state = current_state;
            ESP_LOGI(TAG, "GPIO48 change detected: %d", current_state);
            
            // Your jack: HIGH = headphone inserted, LOW = no headphone (non-standard)
            if (current_state) {
                // Headphone inserted -> disable speaker (if available)
                if (SPEAKER_ENABLE_PIN >= 0) gpio_set_level(SPEAKER_ENABLE_PIN, 0);
                headphones_connected = true;
                ESP_LOGI(TAG, "Headphones detected -> Speaker DISABLED, Vol 30%");
                audio_set_volume(30);
            } else {
                // No headphone -> enable speaker (if available)
                if (SPEAKER_ENABLE_PIN >= 0) gpio_set_level(SPEAKER_ENABLE_PIN, 1);
                headphones_connected = false;
                ESP_LOGI(TAG, "Headphones removed -> Speaker ENABLED, Vol 70%");
                audio_set_volume(70);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool headphone_detect_init(void) {
    // Check if feature is enabled - only detect pin is required
    if (HEADPHONE_DETECT_PIN < 0) {
        ESP_LOGI(TAG, "Headphone detection disabled (detect pin not configured)");
        is_enabled = false;
        return false;
    }
    
    ESP_LOGI(TAG, "Initializing headphone detection...");
    
    // Configure detect pin as input with pull-up
    gpio_config_t detect_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << HEADPHONE_DETECT_PIN)
    };

    esp_err_t ret = gpio_config(&detect_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure detect pin: %s", esp_err_to_name(ret));
        is_enabled = false;
        return false;
    }

    // If SPEAKER_ENABLE_PIN is configured, set it up as output; otherwise skip
    if (SPEAKER_ENABLE_PIN >= 0) {
        gpio_config_t enable_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pin_bit_mask = (1ULL << SPEAKER_ENABLE_PIN)
        };

        ret = gpio_config(&enable_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure enable pin: %s", esp_err_to_name(ret));
            is_enabled = false;
            return false;
        }

        // Initialize speaker enable to HIGH (speaker on by default)
        gpio_set_level(SPEAKER_ENABLE_PIN, 1);
    }

    // Read initial GPIO48 state and set volume accordingly
    bool initial_state = gpio_get_level(HEADPHONE_DETECT_PIN);
    if (initial_state) {
        // HIGH = headphone inserted (opposite of typical jack wiring)
        headphones_connected = true;
        audio_set_volume(30);
        ESP_LOGI(TAG, "Headphones detected at startup (GPIO48=HIGH) -> Vol 30%");
    } else {
        // LOW = no headphone
        headphones_connected = false;
        audio_set_volume(70);
        ESP_LOGI(TAG, "No headphones at startup (GPIO48=HIGH) -> Vol 70%");
    }
    
    is_enabled = true;
    ESP_LOGI(TAG, "Headphone detection initialized (detect=%d, enable=%d)",
             HEADPHONE_DETECT_PIN, SPEAKER_ENABLE_PIN);
    
    return true;
}

bool headphone_detect_start_task(void) {
    if (!is_enabled) {
        ESP_LOGD(TAG, "Cannot start task - feature disabled");
        return false;
    }
    
    xTaskCreatePinnedToCore(
        headphone_monitor_task,
        "hp_detect",
        2048,
        NULL,
        4,  // Priority
        NULL,
        tskNO_AFFINITY
    );
    
    ESP_LOGI(TAG, "Headphone monitoring task started");
    return true;
}

bool headphone_detect_is_enabled(void) {
    return is_enabled;
}

bool headphone_detect_is_connected(void) {
    return headphones_connected;
}