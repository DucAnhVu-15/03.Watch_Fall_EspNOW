/*
 * Backend IMU thật: ULP đọc FIFO MMA8451 (ulp/ulp_main.c), main core gắn timestamp
 * rồi đẩy vào sample_buf.
 *
 * FIFO không lưu thời gian, nên thời điểm mẫu được suy ra từ cạnh watermark: ULP báo
 * ngay khi thấy INT1 (tín hiệu ANCHOR), ISR chốt esp_timer tại đó - coi như thời điểm
 * mẫu mới nhất của batch. Giá trị này rung theo chu kỳ poll của ULP, còn ODR thật của
 * MMA8451 lại lệch vài % so với danh định, nên mỗi batch đi qua một vòng khóa PI để
 * vừa lọc rung vừa bám ODR thật. Khoảng giữa hai batch được chia đều cho từng mẫu.
 *
 * ULP và vòng khóa chạy liên tục từ lúc init để ODR đã khóa sẵn trước khi START;
 * imu_src_start/stop chỉ bật/tắt việc đẩy mẫu vào sample_buf.
 */
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ulp_common.h"
#include "ulp_riscv.h"
#include "imu_source.h"
#include "imu_ulp_shared.h"
#include "mma8451.h"
#include "mma8451_defs.h"
#include "sample_buf.h"

#define TASK_PRIO           6
#define TASK_STACK          4096
#define STALL_TIMEOUT_MS    1000
#define STATUS_PERIOD_US    5000000
#define RAW_PER_G           4096.0f     /* raw 14-bit căn trái @ ±8g */

/* Vòng khóa PI, tính theo batch: tắt dần sau ~10 batch, hệ số tắt dần ~0.7 */
#define PLL_KP              0.2
#define PLL_KI              0.02
#define PERIOD_MAX_ERR      0.10    /* ODR thật không thể lệch hơn 10% so với danh định */
#define RESYNC_PERIODS      5.0     /* lệch quá ngần này chu kỳ -> mất mẫu, bắt lại mốc */

_Static_assert((ULP_RING_SAMPLES & (ULP_RING_SAMPLES - 1)) == 0, "ring size must be a power of 2");
_Static_assert(ULP_RING_SAMPLES >= 2 * MMA8451_FIFO_SIZE, "ring must hold 2 full FIFOs");

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static const char *TAG = "imu_ulp";

static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_anchor_us;         /* chỉ ISR dùng */
static int64_t s_ready_anchor_us;   /* mốc của batch vừa READY */
static volatile bool s_running;

static double  s_nominal_us;        /* chu kỳ mẫu danh định */
static double  s_period_us;         /* chu kỳ mẫu ước lượng */
static int64_t s_t_last;            /* timestamp mẫu cuối đã gán */
static bool    s_locked;
static uint32_t s_total, s_resyncs;

static void ulp_isr(void *arg)
{
    int64_t now = esp_timer_get_time();

    if (ulp_batch_phase == ULP_PHASE_ANCHOR) {
        s_anchor_us = now;
        return;
    }
    portENTER_CRITICAL_ISR(&s_lock);
    s_ready_anchor_us = s_anchor_us;
    portEXIT_CRITICAL_ISR(&s_lock);

    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

/* Cập nhật vòng khóa với batch n mẫu có mẫu cuối tại anchor, trả về khoảng cách giữa các mẫu */
static double clock_update(uint32_t n, int64_t anchor)
{
    if (!s_locked) {
        s_t_last = anchor - (int64_t)(n * s_period_us);
        s_locked = true;
    }

    double err = (double)(anchor - s_t_last) - n * s_period_us;
    if (fabs(err) > RESYNC_PERIODS * s_nominal_us) {
        s_t_last = anchor - (int64_t)(n * s_period_us);
        err = 0;
        s_resyncs++;
    }

    double step = s_period_us + PLL_KP * err / n;
    if (step < s_nominal_us / 2) {
        step = s_nominal_us / 2;    /* timestamp phải tăng dần cho sbuf_find_time */
    }

    s_period_us += PLL_KI * err / n;
    double lo = s_nominal_us * (1.0 - PERIOD_MAX_ERR);
    double hi = s_nominal_us * (1.0 + PERIOD_MAX_ERR);
    s_period_us = s_period_us < lo ? lo : (s_period_us > hi ? hi : s_period_us);

    return step;
}

static void log_status(int16_t x, int16_t y, int16_t z)
{
    ESP_LOGI(TAG, "%" PRIu32 " samples, ODR %.2f Hz, last x=%d y=%d z=%d"
             " | i2c_err=%" PRIu32 " fifo_ovf=%" PRIu32 " ring_drop=%" PRIu32
             " poll_batch=%" PRIu32 " resync=%" PRIu32,
             s_total, 1e6 / s_period_us, x, y, z,
             ulp_stat_i2c_errors, ulp_stat_fifo_overflows, ulp_stat_ring_drops,
             ulp_stat_poll_batches, s_resyncs);
}

static void imu_task(void *arg)
{
    uint32_t tail = 0;
    int16_t x = 0, y = 0, z = 0;
    int64_t next_status = esp_timer_get_time() + STATUS_PERIOD_US;

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(STALL_TIMEOUT_MS)) == 0) {
            ESP_LOGW(TAG, "no data from ULP for %d ms", STALL_TIMEOUT_MS);
            log_status(x, y, z);
        }

        uint32_t n = ulp_ring_head - tail;
        if (n > 0) {
            portENTER_CRITICAL(&s_lock);
            int64_t anchor = s_ready_anchor_us;
            portEXIT_CRITICAL(&s_lock);

            double step = clock_update(n, anchor);
            for (uint32_t i = 0; i < n; i++, tail++) {
                uint32_t w = (tail & (ULP_RING_SAMPLES - 1)) * 2;
                uint32_t xy = ulp_ring[w];
                x = (int16_t)(xy & 0xFFFFu);
                y = (int16_t)(xy >> 16);
                z = (int16_t)(ulp_ring[w + 1] & 0xFFFFu);
                int64_t t = s_t_last + (int64_t)((i + 1) * step);
                if (s_running) {
                    sbuf_push(t, x, y, z);
                }
#if CONFIG_WF_IMU_TEST_ONLY
                ESP_LOGI(TAG, "t=%" PRId64 " x=%d y=%d z=%d | %.3f %.3f %.3f g",
                         t, x, y, z, x / RAW_PER_G, y / RAW_PER_G, z / RAW_PER_G);
#endif
            }
            s_t_last += (int64_t)(n * step);
            s_total += n;
            ulp_ring_tail = tail;
        }

        int64_t now = esp_timer_get_time();
        if (now >= next_status) {
            next_status = now + STATUS_PERIOD_US;
            log_status(x, y, z);
        }
    }
}

esp_err_t imu_src_init(const imu_src_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg->rate_hz > 0, ESP_ERR_INVALID_ARG, TAG, "rate_hz");
    s_nominal_us = 1e6 / cfg->rate_hz;
    s_period_us = s_nominal_us;

    /* Reset mềm không dừng ULP: chương trình cũ có thể vẫn đang giữ bus RTC I2C */
    ulp_riscv_halt();

    ESP_RETURN_ON_ERROR(mma8451_start_fifo(cfg->rate_hz, CONFIG_WF_IMU_FIFO_WATERMARK), TAG, "sensor");

    ESP_RETURN_ON_ERROR(ulp_riscv_load_binary(ulp_main_bin_start, ulp_main_bin_end - ulp_main_bin_start),
                        TAG, "ULP load");
    ESP_RETURN_ON_ERROR(ulp_set_wakeup_period(0, CONFIG_WF_IMU_ULP_POLL_US), TAG, "ULP period");

    ESP_RETURN_ON_FALSE(xTaskCreate(imu_task, "imu", TASK_STACK, NULL, TASK_PRIO, &s_task) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    ESP_RETURN_ON_ERROR(ulp_riscv_isr_register(ulp_isr, NULL, ULP_RISCV_SW_INT), TAG, "ULP isr");

    ulp_riscv_cfg_t ulp_cfg = { .wakeup_source = ULP_RISCV_WAKEUP_SOURCE_TIMER };
    ESP_RETURN_ON_ERROR(ulp_riscv_config_and_run(&ulp_cfg), TAG, "ULP run");

    ESP_LOGI(TAG, "ULP running, poll %d us", CONFIG_WF_IMU_ULP_POLL_US);
    return ESP_OK;
}

esp_err_t imu_src_start(void)
{
    s_running = true;
    return ESP_OK;
}

esp_err_t imu_src_stop(void)
{
    s_running = false;
    return ESP_OK;
}

bool imu_src_running(void)
{
    return s_running;
}
