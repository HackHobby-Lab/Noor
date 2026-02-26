/**
 * config.h
 * Central configuration file for Noor Audio Player
 *
 * CRITICAL: FreeRTOS.h MUST be included before any FreeRTOS components!
 */

#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// CRITICAL: FreeRTOS.h MUST BE FIRST
// ========================================
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// ========================================
// Standard C Libraries
// ========================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

// ========================================
// ESP-IDF System Headers
// ========================================
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

// ========================================
// ESP-IDF Driver Headers
// ========================================
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"

// ========================================
// ESP-IDF VFS and SD Card
// ========================================
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

// ========================================
// PIN DEFINITIONS
// ========================================
#define I2S_BCLK_PIN       18
#define I2S_WS_PIN         17
#define I2S_DIN_PIN        16

#define SD_CS_PIN          10
#define SD_MOSI_PIN        11
#define SD_SCK_PIN         12
#define SD_MISO_PIN        13

#define BTN_PLAY_PIN       14
#define BTN_HOME_PIN       15
#define BTN_VOLUP_PIN      4
#define BTN_VOLDN_PIN      5

#define ENC_CLK_PIN        1
#define ENC_DT_PIN         2
#define ENC_SW_PIN         21

// Headphone detection
// DISABLED: GPIO48 floating (not wired) causes random 30/70% volume flicker.
// Re-enable: change -1 back to 48 once jack detect wire is physically connected.
#define HEADPHONE_DETECT_PIN   48
#define SPEAKER_ENABLE_PIN     -1

// ========================================
// TIMING CONSTANTS
// ========================================
#define BUTTON_DEBOUNCE_MS     50
#define ENC_DEBOUNCE_MS        60

// ========================================
// TASK PRIORITIES
// ========================================
#define PRIORITY_AUDIO         5
#define PRIORITY_ENCODER       3
#define PRIORITY_HEADPHONE     4

// ========================================
// AUDIO SETTINGS
// ========================================
#define VOLUME_MIN             0
#define VOLUME_MAX             100    // 10 steps of 10% -> v1.wav..v10.wav
#define VOLUME_DEFAULT         60

// ========================================
// FILE LIMITS
// ========================================
#define MAX_FOLDERS            32
#define MAX_WAV_FILES          64
#define ANNOUNCE_PATH_MAX      512

// ========================================
// NOTIFICATION BITS
// ========================================
#define NOTIFY_ANNOUNCE_BIT    (1u << 31)

#endif // CONFIG_H
