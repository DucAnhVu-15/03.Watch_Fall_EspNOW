#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "proto.h"
#include "sd_store.h"
#include "selftest.h"

#define ONE_G_RAW 4096

typedef struct {
    uint8_t  id;
    double   period_us;     /* mô phỏng ODR thật khác nhau giữa 2 cảm biến */
    int64_t  clock_offset;  /* đồng hồ riêng của node so với đồng hồ chung */
    uint32_t seq;           /* seq của mẫu kế tiếp */
    uint32_t expected_lines;
} fake_node_t;

static const char *TAG = "selftest";

static fake_node_t s_fake[STORE_NODES];
static int64_t  s_base;
static uint32_t s_frame;

static int16_t fake_axis(int base)
{
    /* Căn trái 14-bit như MMA8451: 2 bit thấp luôn bằng 0 */
    return (int16_t)((base + (int)(esp_random() % 41) - 20) & ~3);
}

/* Tạo khung k của 1 node: mọi mẫu nằm trong [k, k+1) khung, kèm overlap mẫu cũ */
static pkt_data_t *fake_frame(fake_node_t *n, uint32_t k)
{
    int64_t end_us = (int64_t)(k + 1) * PROTO_FRAME_US;
    uint32_t n_new = 0;
    while ((int64_t)((n->seq + n_new) * n->period_us) < end_us) {
        n_new++;
    }
    uint32_t n_ov = n->seq < CONFIG_WF_OVERLAP_SAMPLES ? n->seq : CONFIG_WF_OVERLAP_SAMPLES;
    uint32_t total = n_ov + n_new;
    uint32_t first = n->seq - n_ov;

    pkt_data_t *d = malloc(sizeof(pkt_data_t) + total * sizeof(pkt_sample_t));
    if (d == NULL) {
        return NULL;
    }
    int64_t t_first = s_base + (int64_t)(first * n->period_us);
    int64_t t_last = s_base + (int64_t)((first + total - 1) * n->period_us);

    proto_hdr_init(&d->hdr, PKT_DATA);
    d->node_id = n->id;
    d->retry = 0;
    d->session = 1;
    d->frame_idx = k;
    d->last_seq = first + total - 1;
    d->t_first_us = t_first;
    d->t_last_us = t_last;
    d->t_first_local_us = t_first + n->clock_offset;
    d->fifo_ovf = 0;
    d->ring_drop = 0;
    d->n_total = total;
    d->n_overlap = n_ov;
    for (uint32_t i = 0; i < total; i++) {
        pkt_sample_t s = { fake_axis(-100), fake_axis(50), fake_axis(ONE_G_RAW) };
        memcpy(&d->s[i], &s, sizeof(s));
    }

    n->seq += n_new;
    n->expected_lines += total;
    return d;
}

void selftest_begin(void)
{
    memset(s_fake, 0, sizeof(s_fake));
    s_fake[0] = (fake_node_t){ .id = 1, .period_us = 9738.0,  .clock_offset = 73210000 };
    s_fake[1] = (fake_node_t){ .id = 2, .period_us = 10002.0, .clock_offset = 65120000 };
    s_base = esp_timer_get_time();
    s_frame = 0;
    ESP_LOGW(TAG, "this file contains fake data");
}

void selftest_tick(void)
{
    if (esp_timer_get_time() < s_base + (int64_t)(s_frame + 1) * PROTO_FRAME_US) {
        return;
    }
    for (int i = 0; i < STORE_NODES; i++) {
        pkt_data_t *d = fake_frame(&s_fake[i], s_frame);
        if (d == NULL) {
            ESP_LOGE(TAG, "out of memory");
        } else if (!store_push_frame(d, d)) {
            free(d);
        }
    }
    s_frame++;
}

/* Đếm số dòng của file, gồm cả dòng tiêu đề */
static int32_t count_lines(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return -1;
    }
    static char buf[512];
    int32_t lines = 0;
    size_t got;
    while ((got = fread(buf, 1, sizeof(buf), f)) > 0) {
        for (size_t i = 0; i < got; i++) {
            lines += buf[i] == '\n';
        }
    }
    fclose(f);
    return lines;
}

void selftest_verify(void)
{
    store_stats_t st;
    store_get_stats(&st);
    bool pass = st.queue_drops == 0 && st.write_errors == 0;

    for (int i = 0; i < STORE_NODES; i++) {
        const char *path = store_node_path(s_fake[i].id);
        int32_t got = count_lines(path);
        int32_t want = (int32_t)s_fake[i].expected_lines + 1;   /* +1 dòng tiêu đề */
        ESP_LOGI(TAG, "%s: %" PRId32 " lines, expected %" PRId32 ", lost samples %" PRIu32,
                 path, got, want, st.node[i].lost_samples);
        pass = pass && got == want && st.node[i].lost_samples == 0;
    }
    ESP_LOGI(TAG, "max write %" PRIu32 " ms, queue peak %" PRIu32 "/%d, drops %" PRIu32
             ", write errors %" PRIu32,
             st.max_frame_ms, st.queue_peak, CONFIG_WF_STORE_QUEUE_LEN,
             st.queue_drops, st.write_errors);
    if (pass) {
        ESP_LOGI(TAG, "SELFTEST PASS");
    } else {
        ESP_LOGE(TAG, "SELFTEST FAIL");
    }
}
