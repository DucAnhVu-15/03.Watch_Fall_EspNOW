#include <inttypes.h>
#include <stdio.h>
#include "frame_sink.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "sink";

void sink_begin_session(uint16_t session)
{
    ESP_LOGI(TAG, "=== session %u ===", session);
#if CONFIG_WF_MASTER_DUMP_CSV
    printf("#S,timestamp_us,frame_idx,sample_idx,acc_x_raw,acc_y_raw,acc_z_raw,is_overlap\n");
    printf("#M,timestamp_us,frame_idx,sample_idx,acc_x_raw,acc_y_raw,acc_z_raw,is_overlap\n");
#endif
}

void sink_write(const pkt_data_t *slave, int64_t slave_dt_us,
                const sample_t *m, size_t n_m, uint8_t m_overlap)
{
    int64_t s_new0 = slave->t_first_us + (int64_t)slave->n_overlap * slave_dt_us;
    int64_t m_new0 = n_m > m_overlap ? m[m_overlap].t_us : 0;

    /* phase = lệch pha lấy mẫu giữa 2 IMU (0..1 chu kỳ là bình thường, không phải lỗi đồng bộ) */
    ESP_LOGI(TAG, "frame %" PRIu32 ": S n=%u ov=%u t0=%" PRId64 " dt=%" PRId64
             " | M n=%u ov=%u | phase=%" PRId64 " us rtt=%" PRIu32 " retry=%u",
             slave->frame_idx, slave->n_total, slave->n_overlap, s_new0, slave_dt_us,
             (unsigned)n_m, m_overlap, n_m ? m_new0 - s_new0 : 0,
             slave->sync_rtt_us, slave->retry);

#if CONFIG_WF_MASTER_DUMP_CSV
    for (uint16_t i = 0; i < slave->n_total; i++) {
        pkt_sample_t s = slave->s[i];
        printf("S,%" PRId64 ",%" PRIu32 ",%u,%d,%d,%d,%d\n",
               slave->t_first_us + (int64_t)i * slave_dt_us, slave->frame_idx, i,
               s.x, s.y, s.z, i < slave->n_overlap);
    }
    for (size_t i = 0; i < n_m; i++) {
        printf("M,%" PRId64 ",%" PRIu32 ",%u,%d,%d,%d,%d\n",
               m[i].t_us, slave->frame_idx, (unsigned)i,
               m[i].x, m[i].y, m[i].z, i < m_overlap);
    }
#endif
}
