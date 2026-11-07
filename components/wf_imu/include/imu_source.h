/*
 * Nguồn dữ liệu IMU. Mọi backend đều đẩy mẫu {t_us, x, y, z} vào sample_buf.
 *
 * Chọn backend trong menuconfig (CONFIG_WF_IMU_BACKEND_*):
 *  - imu_ulp.c: MMA8451 thật. ULP RISC-V đọc FIFO, báo lõi chính theo watermark,
 *               lõi chính gắn timestamp rồi gọi sbuf_push.
 *  - imu_sim.c: sóng sin giả lập, 1 mẫu mỗi chu kỳ ODR.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t rate_hz;
    int32_t  odr_err_ppm;   /* chỉ dùng cho bản giả lập */
    float    phase;         /* lệch pha sóng giả lập để phân biệt 2 thiết bị */
} imu_src_cfg_t;

esp_err_t imu_src_init(const imu_src_cfg_t *cfg);
esp_err_t imu_src_start(void);
esp_err_t imu_src_stop(void);
bool      imu_src_running(void);
