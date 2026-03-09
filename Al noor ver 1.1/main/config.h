/**
 * config.h
 * Central configuration file for Noor Audio Player
 *
 * Changes vs previous version:
 * ----------------------------
 * 1. ENC_DEBOUNCE_MS raised from 60 → 200.
 *    Log analysis showed ghost bounce pulses arriving up to 170ms after
 *    the real click.  200ms catches all observed cases with 30ms margin.
 *
 * 2. ENC_SETTLE_MS raised from 180 → 250.
 *    The settle timer must always be longer than ENC_DEBOUNCE_MS so that
 *    the timer cannot fire before the debounce window has closed.
 *    250ms > 200ms, maintaining the 50ms margin.
 *
 * All other values are unchanged.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================
 * CRITICAL: FreeRTOS.h MUST BE FIRST
 * ======================================================================== */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

/* ========================================================================
 * Standard C Libraries
 * ======================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

/* ========================================================================
 * ESP-IDF System Headers
 * ======================================================================== */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

/* ========================================================================
 * ESP-IDF Driver Headers
 * ======================================================================== */
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"

/* ========================================================================
 * ESP-IDF VFS and SD Card
 * ======================================================================== */
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

/* ========================================================================
 * I2S PIN DEFINITIONS
 * ======================================================================== */
#define I2S_BCLK_PIN        18
#define I2S_WS_PIN          17
#define I2S_DIN_PIN         16
/* Aliases used by audio.c legacy driver */
#define I2S_BCK_PIN         I2S_BCLK_PIN
#define I2S_DO_PIN          I2S_DIN_PIN

/* ========================================================================
 * SD CARD SPI PIN DEFINITIONS
 * ======================================================================== */
#define SD_CS_PIN           10
#define SD_MOSI_PIN         11
#define SD_SCK_PIN          12
#define SD_MISO_PIN         13

/* Aliases used by sd_card.c / sdspi driver */
#define PIN_NUM_CS          SD_CS_PIN
#define PIN_NUM_MOSI        SD_MOSI_PIN
#define PIN_NUM_CLK         SD_SCK_PIN
#define PIN_NUM_MISO        SD_MISO_PIN

/* ========================================================================
 * BUTTON PIN DEFINITIONS
 * ======================================================================== */
#define BTN_PLAY_PIN        14
#define BTN_HOME_PIN        15
#define BTN_VOLUP_PIN        4
#define BTN_VOLDN_PIN        5

/* ========================================================================
 * ROTARY ENCODER PIN DEFINITIONS
 * ======================================================================== */
#define ENC_CLK_PIN          1
#define ENC_DT_PIN           2
#define ENC_SW_PIN          21

/* Encoder ISR queue depth — raised to 32 to survive fast spinning */
#define ENC_QUEUE_LEN       32

/* ========================================================================
 * HEADPHONE DETECTION
 * ======================================================================== */
#define HEADPHONE_DETECT_PIN    48
#define SPEAKER_ENABLE_PIN      -1

#define VOLUME_HEADPHONE        60   /* % — headphones plugged in   */
#define VOLUME_SPEAKER          70   /* % — speaker (no headphones) */

/* Debounce for jack-detect GPIO (ms) */
#define HP_DETECT_DEBOUNCE_MS   100

/* How often the monitoring task polls GPIO48 (ms) */
#define HP_DETECT_POLL_MS       200

/* ========================================================================
 * TIMING CONSTANTS  (milliseconds)
 * ======================================================================== */
#define BUTTON_DEBOUNCE_MS      50

/* Hardware debounce window for the rotary encoder GPIO ISR.
 *
 * RAISED from 60 ms to 200 ms.
 *
 * Log analysis (session starting at ~370 s) showed mechanical ghost
 * pulses arriving 60–170 ms after a real encoder click.  At 60 ms the
 * debounce window closed before the ghost arrived, letting it through.
 * 200 ms catches all observed ghosts and still feels instantaneous to
 * the user (a deliberate second click takes >300 ms in practice).
 *
 * The timestamp is now taken IN THE ISR (not in the task) so the full
 * 200 ms window is measured from when the GPIO actually fired, not from
 * when the task happened to process the queued event. */
#define ENC_DEBOUNCE_MS         200

/* Settle window for encoder-driven announcements.
 *
 * RAISED from 180 ms to 250 ms.
 *
 * Must always be > ENC_DEBOUNCE_MS so the settle timer cannot fire
 * while the debounce window is still open.  250 ms > 200 ms, keeping
 * a 50 ms safety margin.  Still feels instant to the user. */
#define ENC_SETTLE_MS           250

/* ========================================================================
 * TASK PRIORITIES
 * ======================================================================== */
#define PRIORITY_AUDIO           5
#define PRIORITY_ENCODER         3
#define PRIORITY_HEADPHONE       4

/* ========================================================================
 * AUDIO / VOLUME SETTINGS
 * ======================================================================== */
#define VOLUME_MIN              0
#define VOLUME_MAX              100   /* Hard cap — do NOT raise above 100 */
#define VOLUME_DEFAULT          70

/* ========================================================================
 * FILE / DIRECTORY LIMITS
 * ======================================================================== */
#define MAX_FOLDERS             32
#define MAX_WAV_FILES           64

/* ========================================================================
 * ANNOUNCEMENT SYSTEM CONSTANTS
 * ======================================================================== */
#define ANNOUNCE_PATH_MAX       512

/*
 * NOTIFY_ANNOUNCE_BIT — FreeRTOS task-notification bit used to wake the
 * audio task when an announcement is pending.
 */
#define NOTIFY_ANNOUNCE_BIT     (1UL << 31)
#define NOTIFY_SETTLE_BIT       (1UL << 30)  /* enc_settle_timer -> audio_task */

#endif /* CONFIG_H */
