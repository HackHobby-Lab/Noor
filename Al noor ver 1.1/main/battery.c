/**
 * battery.c
 * Li-ion battery percentage monitor with low-battery audio alerts
 *
 * Voltage divider: R1=10K (top), R2=47K (bottom)
 *   V_adc = V_bat × 47 / (10 + 47)
 *   V_bat = V_adc × 57 / 47
 *
 * Audio alerts (played once per threshold crossing):
 *   b1.wav → battery <= 40%   ("low battery")
 *   b2.wav → battery <= 20%   ("very low battery")
 *   b3.wav → battery <= 5%    ("critical — shutting down soon")
 *
 * WHY ALERTS WERE SILENT — three root-cause bugs fixed here:
 * -----------------------------------------------------------
 *
 * BUG 1 — stop_flag was set TRUE before audio_play_file ran.
 *   The old code passed &g_stop_flag (main's shared stop flag) to
 *   announce_check_and_play(). That function internally calls
 *   announce_request() which sets *stop_flag = true BEFORE sending
 *   NOTIFY_ANNOUNCE_BIT. So audio_task woke up, called
 *   audio_play_file(path, &g_stop_flag, …), and the very first
 *   iteration of the playback loop saw stop_flag==true → exited
 *   immediately → 0 bytes played → total silence.
 *
 *   Fix: battery task no longer touches g_stop_flag at all.
 *   It uses NOTIFY_BATTERY_ALERT_BIT (a dedicated bit) to wake
 *   audio_task. audio_task calls battery_play_pending_alert() which
 *   uses its OWN local stop flag, completely isolated from the rest.
 *
 * BUG 2 — alert permanently suppressed while story was playing.
 *   The old _check_alerts() returned early (without setting s_warned)
 *   when s_playing_flag was true — BUT the battery polled again in
 *   10 seconds and by then pct might have stayed ≤40, so s_warned_40
 *   was set on that second poll while still playing, meaning the alert
 *   was silently discarded forever.
 *   Actually worse: the original code DID set s_warned before checking
 *   playing_flag, so any threshold crossing during playback was marked
 *   warned and the alert was lost for that entire discharge cycle.
 *
 *   Fix: alert is DEFERRED, not suppressed. s_pending_alert stores the
 *   filename. s_warned is only set when the alert actually fires.
 *   Every poll, if playback has stopped and a pending alert exists,
 *   it is dispatched immediately.
 *
 * BUG 3 — no dedicated notification bit for battery alerts.
 *   Battery alerts competed with NOTIFY_ANNOUNCE_BIT (used for story
 *   announcements, folder names, etc.). If an announcement was already
 *   pending when the battery alert fired, the alert path was overwritten
 *   in pending_announce_path and the announcement was lost (or vice versa).
 *
 *   Fix: battery uses its own NOTIFY_BATTERY_ALERT_BIT passed in from
 *   main.c. audio_task handles it in a separate branch that calls
 *   battery_play_pending_alert() — no shared queue, no overwriting.
 */

#include "battery.h"
#include "audio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <string.h>
#include <unistd.h>
#include <stdio.h>

static const char *TAG = "BATTERY";

#define BAT_ADC_UNIT      ADC_UNIT_1
#define BAT_ADC_CHANNEL   ADC_CHANNEL_5
#define BAT_ADC_ATTEN     ADC_ATTEN_DB_12
#define BAT_ADC_SAMPLES   16

#define BAT_R1_KOHM       10
#define BAT_R2_KOHM       47
#define BAT_SCALE_NUM     (BAT_R1_KOHM + BAT_R2_KOHM)   /* 57 */
#define BAT_SCALE_DEN     BAT_R2_KOHM                    /* 47 */

/* Directories searched for b1/b2/b3.wav (same as announcements) */
static const char * const BAT_SEARCH_DIRS[] = {
    "/sdcard/ANNOUN~1",
    "/sdcard/announcements",
    "/sdcard",
    NULL
};

static adc_oneshot_unit_handle_t s_adc_handle  = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool                      s_cali_ok     = false;
static volatile int              s_last_mv     = 0;
static volatile int              s_last_pct    = -1;

/* audio_task handle + dedicated notification bit (set by main.c) */
static TaskHandle_t s_audio_task     = NULL;
static uint32_t     s_bat_notify_bit = 0;

/* g_playing pointer — for deferral logic only (never to stop audio) */
static volatile bool *s_playing_flag = NULL;

/* Alert state */
static bool s_warned_40 = false;
static bool s_warned_20 = false;
static bool s_warned_5  = false;

/* Deferred alert filename — set when playing, cleared after fire */
static char s_pending_alert[16] = {0};
static bool s_has_pending       = false;

/* No-battery detection */
static int  s_no_bat_count = 0;
static bool s_no_battery   = false;

/* Resolved alert directory (found once) */
static char s_alert_dir[128]  = {0};
static bool s_alert_dir_found = false;

/* -------------------------------------------------------------------------
 * Find the directory that contains b1.wav (called once, cached)
 * ---------------------------------------------------------------------- */
static bool _find_alert_dir(void)
{
    if (s_alert_dir_found) return true;

    for (int i = 0; BAT_SEARCH_DIRS[i]; i++) {
        char probe[160];
        snprintf(probe, sizeof(probe), "%s/b1.wav", BAT_SEARCH_DIRS[i]);
        if (access(probe, F_OK) == 0) {
            strncpy(s_alert_dir, BAT_SEARCH_DIRS[i], sizeof(s_alert_dir) - 1);
            s_alert_dir_found = true;
            ESP_LOGI(TAG, "Battery alert dir resolved: %s", s_alert_dir);
            return true;
        }
    }
    ESP_LOGW(TAG, "Battery alert dir not found — b1.wav missing from all search paths");
    return false;
}

/* -------------------------------------------------------------------------
 * battery_play_pending_alert
 *
 * Called by audio_task when NOTIFY_BATTERY_ALERT_BIT fires.
 * Uses a LOCAL stop flag so the alert is never killed by g_stop_flag.
 * ---------------------------------------------------------------------- */
void battery_play_pending_alert(void)
{
    if (!s_has_pending || s_pending_alert[0] == '\0') return;

    if (!_find_alert_dir()) {
        ESP_LOGW(TAG, "Cannot play alert — alert dir not found");
        s_has_pending = false;
        return;
    }

    char path[160];
    snprintf(path, sizeof(path), "%s/%s", s_alert_dir, s_pending_alert);

    if (access(path, F_OK) != 0) {
        ESP_LOGW(TAG, "Alert file not found: %s", path);
        s_has_pending = false;
        memset(s_pending_alert, 0, sizeof(s_pending_alert));
        return;
    }

    ESP_LOGW(TAG, "Playing battery alert: %s", path);

    /*
     * KEY FIX: use a LOCAL stop flag initialised to FALSE.
     * This is completely isolated from g_stop_flag, so no encoder
     * movement, button press, or story start can kill this playback.
     */
    bool local_stop = false;
    audio_play_file(path, &local_stop, NULL);

    s_has_pending = false;
    memset(s_pending_alert, 0, sizeof(s_pending_alert));
}

/* -------------------------------------------------------------------------
 * Dispatch an alert immediately (not playing) or defer it (playing)
 * ---------------------------------------------------------------------- */
static void _fire_or_defer(const char *filename, bool *p_warned)
{
    bool currently_playing = (s_playing_flag && *s_playing_flag);

    if (!currently_playing) {
        /* Fire immediately */
        *p_warned = true;
        strncpy(s_pending_alert, filename, sizeof(s_pending_alert) - 1);
        s_pending_alert[sizeof(s_pending_alert) - 1] = '\0';
        s_has_pending = true;
        ESP_LOGW(TAG, "Battery alert queued: %s", filename);
        xTaskNotify(s_audio_task, s_bat_notify_bit, eSetBits);
    } else {
        /*
         * Defer — do NOT set *p_warned yet. We only mark warned once
         * the alert actually plays. Store highest-priority pending alert
         * (b3 > b2 > b1 — only upgrade, never downgrade).
         */
        bool upgrade = false;
        if (!s_has_pending) {
            upgrade = true;
        } else if (filename[1] > s_pending_alert[1]) {
            /* b3 > b2 > b1 (compare digit character) */
            upgrade = true;
        }
        if (upgrade) {
            strncpy(s_pending_alert, filename, sizeof(s_pending_alert) - 1);
            s_pending_alert[sizeof(s_pending_alert) - 1] = '\0';
            s_has_pending = true;
            ESP_LOGW(TAG, "Battery alert deferred (playing): %s", filename);
        }
    }
}

/* -------------------------------------------------------------------------
 * Check thresholds every 10 s
 * ---------------------------------------------------------------------- */
static void _check_alerts(int pct)
{
    if (!s_audio_task || !s_bat_notify_bit) return;
    if (s_no_battery) return;

    bool currently_playing = (s_playing_flag && *s_playing_flag);

    /* Flush deferred alert if playback just stopped */
    if (s_has_pending && !currently_playing) {
        /* Mark the right threshold as warned */
        if      (strncmp(s_pending_alert, "b3", 2) == 0) s_warned_5  = true;
        else if (strncmp(s_pending_alert, "b2", 2) == 0) s_warned_20 = true;
        else if (strncmp(s_pending_alert, "b1", 2) == 0) s_warned_40 = true;

        ESP_LOGW(TAG, "Flushing deferred battery alert: %s", s_pending_alert);
        xTaskNotify(s_audio_task, s_bat_notify_bit, eSetBits);
        /* s_has_pending stays true — cleared inside battery_play_pending_alert */
        return;  /* don't re-evaluate thresholds this cycle */
    }

    /* 40% — b1.wav */
    if (pct <= 40 && !s_warned_40)
        _fire_or_defer("b1.wav", &s_warned_40);
    else if (pct > 45)
        s_warned_40 = false;

    /* 20% — b2.wav */
    if (pct <= 20 && !s_warned_20)
        _fire_or_defer("b2.wav", &s_warned_20);
    else if (pct > 25)
        s_warned_20 = false;

    /* 5% — b3.wav */
    if (pct <= 5 && !s_warned_5)
        _fire_or_defer("b3.wav", &s_warned_5);
    else if (pct > 10)
        s_warned_5 = false;
}

/* -------------------------------------------------------------------------
 * ADC helpers
 * ---------------------------------------------------------------------- */
static void _cali_init(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = BAT_ADC_UNIT,
        .chan     = BAT_ADC_CHANNEL,
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cfg, &s_cali_handle) == ESP_OK) {
        s_cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting");
        return;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg2 = {
        .unit_id  = BAT_ADC_UNIT,
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cfg2, &s_cali_handle) == ESP_OK) {
        s_cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: line fitting");
        return;
    }
#endif
    ESP_LOGW(TAG, "ADC calibration not available — using raw conversion");
}

static int _read_adc_mv(void)
{
    if (!s_adc_handle) return 0;
    int32_t sum = 0;
    for (int i = 0; i < BAT_ADC_SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &raw);
        int mv = 0;
        if (s_cali_ok)
            adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        else
            mv = (raw * 3300) / 4095;
        sum += mv;
    }
    return (int)(sum / BAT_ADC_SAMPLES);
}

static int _mv_to_percent(int bat_mv)
{
    static const struct { int mv; int pct; } lut[] = {
        { 4200, 100 }, { 4100,  90 }, { 4000,  80 },
        { 3900,  70 }, { 3800,  60 }, { 3700,  50 },
        { 3600,  40 }, { 3500,  30 }, { 3400,  20 },
        { 3200,  10 }, { 3000,   0 },
    };
    static const int LUT_LEN = sizeof(lut) / sizeof(lut[0]);

    if (bat_mv >= lut[0].mv)         return 100;
    if (bat_mv <= lut[LUT_LEN-1].mv) return 0;

    for (int i = 0; i < LUT_LEN - 1; i++) {
        if (bat_mv <= lut[i].mv && bat_mv > lut[i+1].mv) {
            int mv_range  = lut[i].mv  - lut[i+1].mv;
            int pct_range = lut[i].pct - lut[i+1].pct;
            return lut[i+1].pct +
                   (bat_mv - lut[i+1].mv) * pct_range / mv_range;
        }
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * Battery monitor task
 * ---------------------------------------------------------------------- */
static void _battery_task(void *arg)
{
    while (1) {
        int adc_mv = _read_adc_mv();
        int bat_mv = (adc_mv * BAT_SCALE_NUM) / BAT_SCALE_DEN;
        int pct    = _mv_to_percent(bat_mv);

        s_last_mv  = bat_mv;
        s_last_pct = pct;

        if (bat_mv < 2500) {
            if (++s_no_bat_count >= 3 && !s_no_battery) {
                s_no_battery = true;
                ESP_LOGW(TAG, "No battery detected (ADC floating) — alerts disabled");
            }
        } else {
            s_no_bat_count = 0;
            s_no_battery   = false;
        }

        int filled = pct / 10;
        char bar[11];
        for (int i = 0; i < 10; i++) bar[i] = (i < filled) ? '#' : '-';
        bar[10] = '\0';

        ESP_LOGI(TAG, "Battery: %d%% [%s]  %d mV  (ADC: %d mV)%s%s",
                 pct, bar, bat_mv, adc_mv,
                 s_no_battery   ? "  [NO BATTERY]"    : "",
                 s_has_pending  ? "  [ALERT PENDING]" : "");

        _check_alerts(pct);

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void battery_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = BAT_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle,
                                               BAT_ADC_CHANNEL, &chan_cfg));
    _cali_init();
    ESP_LOGI(TAG, "Battery monitor init: GPIO6, R1=10K R2=47K, atten=12dB");

    xTaskCreatePinnedToCore(_battery_task, "battery", 4096, NULL,
                            2, NULL, tskNO_AFFINITY);
}

/**
 * battery_set_audio_task
 * @task        : handle of audio_task
 * @notify_bit  : NOTIFY_BATTERY_ALERT_BIT defined in main.c (e.g. 1UL<<26)
 *                Must be a bit not used by any other notification.
 */
void battery_set_audio_task(TaskHandle_t task, uint32_t notify_bit)
{
    s_audio_task     = task;
    s_bat_notify_bit = notify_bit;
    ESP_LOGI(TAG, "Audio task registered (battery alert bit=0x%08lx)", notify_bit);
}

void battery_set_playing_flag(volatile bool *playing_flag)
{
    s_playing_flag = playing_flag;
    ESP_LOGI(TAG, "Playing flag registered (alerts deferred during playback)");
}

int battery_get_percent(void) { return s_last_pct; }
int battery_get_mv(void)      { return s_last_mv;  }
