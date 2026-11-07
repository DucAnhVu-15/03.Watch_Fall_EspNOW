/*
 * Ring buffer mẫu IMU có timestamp, đánh số tuần tự (seq tăng dần từ 0 sau mỗi reset).
 * Mẫu seq nằm trong buffer khi sbuf_oldest() <= seq < sbuf_head().
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int64_t t_us;       /* đồng hồ cục bộ của thiết bị */
    int16_t x, y, z;
} sample_t;

esp_err_t sbuf_init(void);
void      sbuf_reset(void);
void      sbuf_push(int64_t t_us, int16_t x, int16_t y, int16_t z);

uint32_t  sbuf_head(void);     /* seq của mẫu kế tiếp sẽ được ghi */
uint32_t  sbuf_oldest(void);   /* seq cũ nhất còn trong buffer   */
bool      sbuf_last_time(int64_t *t_us);

/* Copy tối đa n mẫu từ seq. Trả về 0 nếu seq đã bị ghi đè hoặc chưa có */
size_t    sbuf_read(uint32_t seq, sample_t *out, size_t n);

/* Tìm seq đầu tiên có t_us >= t. false nếu t mới hơn mẫu mới nhất.
 * Nếu t cũ hơn mẫu cũ nhất thì trả về sbuf_oldest(). */
bool      sbuf_find_time(int64_t t_us, uint32_t *seq);
