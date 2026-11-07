/*
 * Cấu hình MMA8451 từ main core qua RTC I2C. Chỉ gọi trước khi chạy ULP:
 * sau đó bus RTC I2C thuộc về ULP.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Khởi tạo RTC I2C, kiểm tra WHO_AM_I và cấu hình FIFO circular + ngắt watermark
 * trên INT1 (tích cực mức thấp), range ±8g. Cảm biến ở chế độ active khi trả về. */
esp_err_t mma8451_start_fifo(uint32_t rate_hz, uint8_t watermark);
