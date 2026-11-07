#include <inttypes.h>
#include <math.h>
#include "imu_source.h"
#include "sample_buf.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

#define TWO_PI       6.28318530718f
#define ONE_G_RAW    16384   /* MMA8451 ±2g, 14-bit căn trái */

static const char *TAG = "imu_sim";

static esp_timer_handle_t s_timer;
static uint64_t s_period_us;
static float s_phase;
static bool s_running;

static inline int16_t to_raw14(float v)
{
    int32_t r = (int32_t)v;
    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;
    return (int16_t)(r & ~3);   /* 2 bit thấp luôn = 0 như MMA8451 */
}

static inline float noise(void)
{
    return (float)((int32_t)(esp_random() % 41) - 20);
}

static void sim_cb(void *arg)
{
    int64_t t = esp_timer_get_time();
    /* Các tần số 0.5/1/2 Hz đều tuần hoàn trong 10s -> lấy modulo để float không mất chính xác */
    float ts = (float)(t % 10000000) / 1e6f;

    float x = 2000.0f * sinf(TWO_PI * 1.0f * ts + s_phase) + noise();
    float y = 1000.0f * sinf(TWO_PI * 0.5f * ts + s_phase) + noise();
    float z = ONE_G_RAW + 800.0f * sinf(TWO_PI * 2.0f * ts) + noise();

    sbuf_push(t, to_raw14(x), to_raw14(y), to_raw14(z));
}

esp_err_t imu_src_init(const imu_src_cfg_t *cfg)
{
    if (cfg->rate_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    s_period_us = (uint64_t)(1000000LL * (1000000LL + cfg->odr_err_ppm) / ((int64_t)cfg->rate_hz * 1000000LL));
    s_phase = cfg->phase;

    const esp_timer_create_args_t args = {
        .callback = sim_cb,
        .name = "imu_sim",
    };
    esp_err_t err = esp_timer_create(&args, &s_timer);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "simulated IMU: %" PRIu32 " Hz, period %" PRIu64 " us", cfg->rate_hz, s_period_us);
    }
    return err;
}

esp_err_t imu_src_start(void)
{
    if (s_running) {
        return ESP_OK;
    }
    esp_err_t err = esp_timer_start_periodic(s_timer, s_period_us);
    s_running = (err == ESP_OK);
    return err;
}

esp_err_t imu_src_stop(void)
{
    if (!s_running) {
        return ESP_OK;
    }
    s_running = false;
    return esp_timer_stop(s_timer);
}

bool imu_src_running(void)
{
    return s_running;
}
