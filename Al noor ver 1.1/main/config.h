/**
 * config.h
 * Central configuration file for Noor Audio Player
 *
 * This is the ONE AND ONLY place where hardware pins, limits, and
 * shared constants are defined.  Individual module headers must NOT
 * redefine any of these — they should #include "config.h" instead.
 *
 * CRITICAL: FreeRTOS.h MUST be included before any FreeRTOS components!
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

/* Encoder driver constants */
#define ENC_QUEUE_LEN       16

/* ========================================================================
 * HEADPHONE DETECTION
 *
 * Set HEADPHONE_DETECT_PIN to the GPIO number of the jack-detect switch.
 * Set to -1 to disable the feature entirely (skips GPIO config).
 *
 * Your hardware: jack switch wired to GPIO48.
 * HIGH = headphone inserted (normally-open switch to VCC).
 * LOW  = no headphone / speaker mode.
 *
 * SPEAKER_ENABLE_PIN: set to -1 if no dedicated speaker-enable GPIO.
 *
 * Volume levels:
 *   VOLUME_HEADPHONE  — set when headphones are detected (quieter, safe for ears)
 *   VOLUME_SPEAKER    — set when no headphones (louder, drives speaker)
 *
 * Both values must be within [VOLUME_MIN, VOLUME_MAX].
 * ======================================================================== */
#define HEADPHONE_DETECT_PIN    48
#define SPEAKER_ENABLE_PIN      -1

#define VOLUME_HEADPHONE        30   /* % — headphones plugged in   */
#define VOLUME_SPEAKER          60   /* % — speaker (no headphones) */

/* Debounce for jack-detect GPIO (ms) */
#define HP_DETECT_DEBOUNCE_MS   100

/* How often the monitoring task polls GPIO48 (ms) */
#define HP_DETECT_POLL_MS       200

/* ========================================================================
 * TIMING CONSTANTS  (milliseconds)
 * ======================================================================== */
#define BUTTON_DEBOUNCE_MS      50

/* Hardware debounce window for the rotary encoder GPIO ISR.
 * Pulses arriving within this window after the first edge are discarded.
 * Raised to 80 ms to reject mechanical contact-bounce on cheap encoders. */
#define ENC_DEBOUNCE_MS         60

/* Settle window for encoder-driven announcements.
 * After the last rotation event, the settle timer fires after this many ms
 * and plays the announcement for the current selection.  This means:
 *   - Spinning fast: only the *final* folder/track is announced.
 *   - Ghost bounces: a bounce arriving within this window re-arms the
 *     timer without triggering an extra announcement.
 * 180 ms feels instant to the user but filters all normal bounce/spin. */
#define ENC_SETTLE_MS           180

/* ========================================================================
 * TASK PRIORITIES
 * ======================================================================== */
#define PRIORITY_AUDIO           5
#define PRIORITY_ENCODER         3
#define PRIORITY_HEADPHONE       4

/* ========================================================================
 * AUDIO / VOLUME SETTINGS
 *
 * VOLUME_MAX is 100 — this is a software percentage applied to the PCM
 * samples before sending to I2S.  Values above 100 cause integer overflow
 * / clipping artefacts and distortion.  Use the amplifier gain or the
 * hardware volume pot for louder output, not this software multiplier.
 * ======================================================================== */
#define VOLUME_MIN              0
#define VOLUME_MAX              100   /* Hard cap — do NOT raise above 100 */
#define VOLUME_DEFAULT          60

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
 *
 * Bit 31 is used to avoid collisions with other notification bits that
 * lower-numbered bits might carry.  Must be consistent between the sender
 * (announcements.c) and the receiver (main.c audio task).
 */
#define NOTIFY_ANNOUNCE_BIT     (1UL << 31)
#define NOTIFY_SETTLE_BIT       (1UL << 30)  /* enc_settle_timer -> audio_task */

#endif /* CONFIG_H */
