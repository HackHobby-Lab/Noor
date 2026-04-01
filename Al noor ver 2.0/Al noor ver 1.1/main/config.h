/**
 * config.h
 * Central configuration file for Noor Audio Player
 *
 * Changes vs previous version:
 * ----------------------------
 * 1. ENC_DEBOUNCE_MS raised from 60 -> 200 (ghost bounce fix).
 * 2. ENC_SETTLE_MS raised from 180 -> 250 (must exceed ENC_DEBOUNCE_MS).
 * 3. Added NOTIFY_USB_MSC_CHANGED_BIT (bit 27) — sent by usb_manager to
 *    audio_task when SD card is returned from USB host so it can rescan.
 * 4. Added NOTIFY_QUIZ_OPTS_BIT (bit 30) — sent by audio_task to itself
 *    after the quiz question finishes, triggering the options intro.
 *    (NOTIFY_SETTLE_BIT reused bit 30 in old firmware; quiz firmware
 *    no longer uses a settle timer so bit 30 is safe to repurpose.)
 * 5. Added NOTIFY_HOME_AUDIO_BIT (bit 28) — triggers welcome+rotate
 *    playback via audio_task on boot / home-button press.
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

#define VOLUME_HEADPHONE        75   /* % — headphones plugged in   */
#define VOLUME_SPEAKER          100   /* % — speaker (no headphones) */

/* Debounce for jack-detect GPIO (ms) */
#define HP_DETECT_DEBOUNCE_MS   100

/* How often the monitoring task polls GPIO48 (ms) */
#define HP_DETECT_POLL_MS       200

/* ========================================================================
 * TIMING CONSTANTS  (milliseconds)
 * ======================================================================== */
#define BUTTON_DEBOUNCE_MS      50

/* Raised from 60 ms to 200 ms — ghost bounce fix (see encoder.c). */
#define ENC_DEBOUNCE_MS         200

/* Must be > ENC_DEBOUNCE_MS.  250 ms > 200 ms (50 ms margin). */
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
#define VOLUME_MAX              100
#define VOLUME_DEFAULT          100

/* ========================================================================
 * FILE / DIRECTORY LIMITS
 * ======================================================================== */
#define MAX_FOLDERS             32
#define MAX_WAV_FILES           64

/* ========================================================================
 * ANNOUNCEMENT SYSTEM CONSTANTS
 * ======================================================================== */
#define ANNOUNCE_PATH_MAX       512

/* ========================================================================
 * TASK NOTIFICATION BITS
 *
 * Bit layout for audio_task notifications:
 *   Bit 31  NOTIFY_ANNOUNCE_BIT       — announcement pending in queue
 *   Bit 30  NOTIFY_QUIZ_OPTS_BIT      — play quiz options intro
 *   Bit 29  (reserved)
 *   Bit 28  NOTIFY_HOME_AUDIO_BIT     — play welcome.wav + rotate.wav
 *   Bit 27  NOTIFY_USB_MSC_CHANGED_BIT— SD returned from USB host, rescan
 *   Bits 0-26: direct track index + 1 for playback
 * ======================================================================== */
#define NOTIFY_ANNOUNCE_BIT          (1UL << 31)
#define NOTIFY_QUIZ_OPTS_BIT         (1UL << 30)
#define NOTIFY_HOME_AUDIO_BIT        (1UL << 28)
#define NOTIFY_USB_MSC_CHANGED_BIT   (1UL << 27)

/* NOTIFY_SETTLE_BIT kept for reference — not used in this firmware
 * (the settle timer approach was replaced by live nav state reads) */
#define NOTIFY_SETTLE_BIT            (1UL << 30)  /* same bit as QUIZ_OPTS */

#endif /* CONFIG_H */
