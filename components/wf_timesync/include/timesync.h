/*
 * Đồng bộ đồng hồ của node theo đồng hồ chung (esp_timer của C3).
 *
 * Mỗi beacon cho 1 cặp (giờ node lúc nhận, giờ C3 lúc phát). Giữ các cặp gần nhất và
 * fit đường thẳng common = a * local + b, nên bù được cả lệch lẫn trôi tinh thể.
 *
 * Trễ từ lúc C3 lấy giờ đến lúc gói lên sóng là chung cho cả 2 node vì cùng nghe
 * một gói broadcast, nên nó không tạo sai lệch giữa 2 node.
 *
 * Các hàm không có khóa: chỉ gọi từ một task.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void    timesync_reset(void);
void    timesync_add(int64_t local_us, int64_t common_us);
bool    timesync_valid(void);
int64_t timesync_to_common(int64_t local_us);
int64_t timesync_to_local(int64_t common_us);

/* Tốc độ đồng hồ node so với C3, phần triệu */
double  timesync_ppm(void);
