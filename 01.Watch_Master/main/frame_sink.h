/*
 * Nơi nhận dữ liệu đã ghép (khung Slave + đoạn dữ liệu Master tương ứng).
 * Hiện tại: log tóm tắt + (tuỳ chọn) in CSV. Sau này thay bằng ghi LittleFS.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "proto.h"
#include "sample_buf.h"

void sink_begin_session(uint16_t session);

/*
 * slave     : khung nhận từ Slave (timestamp đã theo đồng hồ Master)
 * slave_dt  : chu kỳ mẫu của Slave suy ra từ t_first/t_last
 * m, n_m    : mẫu Master tương ứng, m_overlap mẫu đầu là overlap
 */
void sink_write(const pkt_data_t *slave, int64_t slave_dt_us,
                const sample_t *m, size_t n_m, uint8_t m_overlap);
