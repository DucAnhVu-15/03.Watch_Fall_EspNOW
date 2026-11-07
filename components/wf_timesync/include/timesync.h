/*
 * Đồng bộ thời gian kiểu NTP 2 chiều giữa Slave (client) và Master (server).
 *
 *   Slave  t1 ---- SYNC_REQ ----> t2  Master
 *   Slave  t4 <--- SYNC_RESP ---- t3  Master
 *
 *   offset = ((t2 - t1) + (t3 - t4)) / 2      (master_time = local_time + offset)
 *   rtt    = (t4 - t1) - (t3 - t2)
 *
 * Giữ cửa sổ N lần đo gần nhất, dùng lần đo có RTT nhỏ nhất (ít bị trễ hàng đợi nhất).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

void    timesync_reset(void);

/* Thêm 1 lần đo. Trả về false nếu bị loại (RTT bất thường) */
bool    timesync_add(int64_t t1, int64_t t2, int64_t t3, int64_t t4);

bool    timesync_valid(void);
int64_t timesync_to_master(int64_t local_us);
void    timesync_get(int64_t *offset_us, int32_t *rtt_us);
