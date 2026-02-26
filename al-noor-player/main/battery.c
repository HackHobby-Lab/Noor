/**
 * battery.c
 * Li-ion battery percentage monitor with low-battery audio alerts
 *
 * Voltage divider: R1=10K (top), R2=47K (bottom)
 *   V_adc = V_bat × 47 / (10 + 47)
 *   V_bat = V_adc × 57 / 47
 *
 * Audio alerts (played once per threshold crossing):
 *   b1.wav → battery <= 40%
 *   b2.wav → battery <= 20%
 *   b3.wav → battery <= 5%
 */

#include "battery.h"
#include "announcements.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "BATTERY";

#define BAT_ADC_UNIT      ADC_UNIT_1
#define BAT_ADC_CHANNEL   ADC_CHANNEL_5      /* GPIO6 = ADC1_CH5 */
#define BAT_ADC_ATTEN     ADC_ATTEN_DB_12    /* 0–3100 mV input range */
#define BAT_ADC_SAMPLES   16                 /* average N readings */

/* Voltage divider: R1=10K (top), R2=47K (bottom)
 * NOTE: At full charge (4.2V), V_adc = 3.47V which exceeds ADC 3.1V max.
 * ADC saturates above ~3.76V battery → anything above ~60% reads as 100%.
 * Low-battery alerts (40%, 20%, 5%) still work correctly. */
#define BAT_R1_KOHM       10
#define BAT_R2_KOHM       47
#define BAT_SCALE_NUM     (BAT_R1_KOHM + BAT_R2_KOHM)   /* 57 */
#define BAT_SCALE_DEN     BAT_R2_KOHM                    /* 47 */

static adc_oneshot_unit_handle_t s_adc_handle  = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool                      s_cali_ok     = false;
static volatile int              s_last_mv     = 0;
static volatile int              s_last_pct    = -1;

/* Audio task handle for playing low-battery alerts */
static TaskHandle_t    s_audio_task = NULL;
static volatile bool  *s_stop_flag  = NULL;

/* Alert tracking — each fires once, resets when battery charges back up */
static bool s_warned_40 = false;
static bool s_warned_20 = false;
static bool s_warned_5  = false;

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
        if (s_cali_ok) {
            adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        } else {
            mv = (raw * 3300) / 4095;
        }
        sum += mv;
    }
    return (int)(sum / BAT_ADC_SAMPLES);
}

static int _mv_to_percent(int bat_mv)
{
    static const struct { int mv; int pct; } lut[] = {
        { 4200, 100 },
        { 4100,  90 },
        { 4000,  80 },
        { 3900,  70 },
        { 3800,  60 },
        { 3700,  50 },
        { 3600,  40 },
        { 3500,  30 },
        { 3400,  20 },
        { 3200,  10 },
        { 3000,   0 },
    };
    static const int LUT_LEN = sizeof(lut) / sizeof(lut[0]);

    if (bat_mv >= lut[0].mv)          return 100;
    if (bat_mv <= lut[LUT_LEN-1].mv)  return 0;

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

/**
 * Play a low-battery alert WAV file from /sdcard root.
 * Uses the announcement system to queue playback on the audio task.
 */
static void _play_alert(const char *filename)
{
    if (!s_audio_task || !s_stop_flag) return;
    ESP_LOGW(TAG, "Low battery alert: %s", filename);
    announce_check_and_play("/sdcard", NULL, filename,
                            s_audio_task, s_stop_flag);
}

/**
 * Check thresholds and play alerts once per crossing.
 * Hysteresis: alert fires when dropping below threshold,
 * resets when charging 5% above it.
 */
static void _check_alerts(int pct)
{
    if (!s_audio_task) return;

    /* 40% threshold */
    if (pct <= 40 && !s_warned_40) {
        s_warned_40 = true;
        _play_alert("b1.wav");
    } else if (pct > 45) {
        s_warned_40 = false;
    }

    /* 20% threshold */
    if (pct <= 20 && !s_warned_20) {
        s_warned_20 = true;
        _play_alert("b2.wav");
    } else if (pct > 25) {
        s_warned_20 = false;
    }

    /* 5% threshold */
    if (pct <= 5 && !s_warned_5) {
        s_warned_5 = true;
        _play_alert("b3.wav");
    } else if (pct > 10) {
        s_warned_5 = false;
    }
}

static void _battery_task(void *arg)
{
    while (1) {
        int adc_mv = _read_adc_mv();
        int bat_mv = (adc_mv * BAT_SCALE_NUM) / BAT_SCALE_DEN;
        int pct = _mv_to_percent(bat_mv);

        s_last_mv  = bat_mv;
        s_last_pct = pct;

        int filled = pct / 10;
        char bar[11];
        for (int i = 0; i < 10; i++)
            bar[i] = (i < filled) ? '#' : '-';
        bar[10] = '\0';

        ESP_LOGI(TAG, "Battery: %d%% [%s]  %d mV  (ADC pin: %d mV)",
                 pct, bar, bat_mv, adc_mv);

        _check_alerts(pct);

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

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

void battery_set_audio_task(TaskHandle_t task, volatile bool *stop_flag)
{
    s_audio_task = task;
    s_stop_flag  = stop_flag;
    ESP_LOGI(TAG, "Audio task registered for low-battery alerts");
}

int battery_get_percent(void) { return s_last_pct; }
int battery_get_mv(void)      { return s_last_mv; }