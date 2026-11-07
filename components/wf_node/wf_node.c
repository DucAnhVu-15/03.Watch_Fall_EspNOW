#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espnow_link.h"
#include "proto.h"
#include "sample_buf.h"
#include "timesync.h"
#include "wf_node.h"

#define NODE_ID             CONFIG_WF_NODE_ID
#define TASK_PRIO           5
#define MAX_PENDING         8       /* 8 khung, sample_buf giữ được khoảng 10 giây */
#define SLOT_BASE_US        10000   /* khe của node 1, tính từ lúc nhận beacon */
#define SLOT_GAP_US         5000    /* 1 khung mất khoảng 1 ms trên sóng ở 6 Mbps */
#define RETRY_US            250000
#define C3_LOST_US          3000000
#define STATUS_PERIOD_US    5000000

/* Khung đã đóng, chờ C3 xác nhận. Gửi lại bằng cách đọc lại sample_buf. */
typedef struct {
    bool     used;
    uint32_t frame_idx;
    uint32_t first_seq;     /* seq của mẫu mới đầu tiên */
    uint16_t n_new;
    uint8_t  n_ov;
    uint8_t  tries;
    int64_t  t_first, t_last, t_first_local;
    int64_t  last_tx;
} pend_t;

static const char *TAG = "node";

static uint8_t  s_c3_mac[6];
static bool     s_have_c3;
static uint16_t s_my_tail;
static bool     s_joined;
static int64_t  s_last_beacon;
static bool     s_c3_lost_logged;
static int64_t  s_slot_at;          /* 0 = không có khe chờ */

static bool     s_active;
static uint16_t s_session;
static uint32_t s_frame;            /* khung kế tiếp cần đóng */
static uint32_t s_stop_frame = PROTO_NO_FRAME;
static bool     s_have_seq;
static uint32_t s_next_seq;         /* seq của mẫu mới kế tiếp */

static pend_t   s_pend[MAX_PENDING];
static uint32_t s_sent, s_resent, s_dropped, s_overruns;

static sample_t s_tmp[PKT_DATA_MAX_SAMPLES];
static uint8_t  s_tx[ESP_NOW_MAX_DATA_LEN_V2] __attribute__((aligned(4)));

/* ------------------------------------------------------------------------- */

static void pending_clear(void)
{
    memset(s_pend, 0, sizeof(s_pend));
}

static int pending_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_PENDING; i++) {
        n += s_pend[i].used;
    }
    return n;
}

static pend_t *pending_alloc(void)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        if (!s_pend[i].used) {
            return &s_pend[i];
        }
    }
    return NULL;
}

static void ack_apply(const pkt_ack_t *a)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        pend_t *p = &s_pend[i];
        if (!p->used) {
            continue;
        }
        uint32_t off = p->frame_idx - a->next_frame;
        if (p->frame_idx < a->next_frame ||
            (off >= 1 && off <= PROTO_ACK_BITS && (a->bitmap & (1u << (off - 1))))) {
            p->used = false;
        }
    }
}

/* ------------------------------------------------------------------------- */

static void session_start(const pkt_beacon_t *b)
{
    pending_clear();
    s_session = b->session;
    s_frame = b->start_frame;
    s_stop_frame = PROTO_NO_FRAME;
    s_have_seq = false;
    s_active = true;
    ESP_LOGI(TAG, "START session %u from frame %" PRIu32, b->session, b->start_frame);
}

static void session_stop(void)
{
    pending_clear();
    s_active = false;
    ESP_LOGI(TAG, "STOP session %u: sent %" PRIu32 ", resent %" PRIu32 ", dropped %" PRIu32
             ", overruns %" PRIu32, s_session, s_sent, s_resent, s_dropped, s_overruns);
}

static void on_beacon(const link_rx_t *m)
{
    pkt_beacon_t b;
    if (m->len < sizeof(b)) {
        return;
    }
    memcpy(&b, m->data, sizeof(b));

    if (!s_have_c3 || memcmp(m->src, s_c3_mac, 6) != 0) {
        if (link_add_peer(m->src) != ESP_OK) {
            return;
        }
        memcpy(s_c3_mac, m->src, 6);
        s_have_c3 = true;
        s_joined = false;
        timesync_reset();
        ESP_LOGI(TAG, "found C3 " MACSTR, MAC2STR(m->src));
    }

    s_last_beacon = m->rx_us;
    s_c3_lost_logged = false;
    timesync_add(m->rx_us, b.t_c3_us);
    s_slot_at = m->rx_us + SLOT_BASE_US + (NODE_ID - 1) * SLOT_GAP_US;

    const pkt_ack_t *a = &b.ack[NODE_ID - 1];
    bool joined = a->mac_tail == s_my_tail;
    if (joined != s_joined) {
        s_joined = joined;
        ESP_LOGI(TAG, "%s C3 as node %d", joined ? "joined" : "left", NODE_ID);
    }

    if (b.running) {
        if (!s_active || b.session != s_session) {
            session_start(&b);
        }
        s_stop_frame = b.stop_frame;
        ack_apply(a);
    } else if (s_active) {
        session_stop();
    }
}

/* ------------------------------------------------------------------------- */

/* Khung k gồm mẫu trong [k, k+1) của đồng hồ chung. Mẫu mới luôn nối tiếp khung trước
 * theo seq nên không thiếu không trùng dù đường fit đồng hồ có nhích nhẹ. */
static void frame_tick(int64_t now)
{
    if (!s_active || !timesync_valid()) {
        return;
    }
    if (s_stop_frame != PROTO_NO_FRAME && s_frame > s_stop_frame) {
        return;
    }

    if (!s_have_seq) {
        /* Vào muộn, ví dụ node vừa khởi động lại giữa phiên: bắt đầu từ khung hiện tại */
        uint32_t cur = (uint32_t)(timesync_to_common(now) / PROTO_FRAME_US);
        if (cur > s_frame + 1) {
            s_frame = cur;
        }
        if (!sbuf_find_time(timesync_to_local((int64_t)s_frame * PROTO_FRAME_US), &s_next_seq)) {
            return;
        }
        s_have_seq = true;
    }

    uint32_t end;
    if (!sbuf_find_time(timesync_to_local((int64_t)(s_frame + 1) * PROTO_FRAME_US), &end)) {
        return;     /* chưa đủ mẫu tới hết khung */
    }
    pend_t *p = pending_alloc();
    if (p == NULL) {
        return;     /* C3 chưa ACK kịp: dữ liệu vẫn nằm trong sample_buf */
    }

    uint32_t oldest = sbuf_oldest();
    uint32_t first = s_next_seq;
    if (first < oldest) {
        s_overruns += oldest - first;
        ESP_LOGW(TAG, "sample_buf overrun: %" PRIu32 " samples lost", oldest - first);
        first = oldest;
    }
    uint32_t n_ov = first - oldest;
    if (n_ov > CONFIG_WF_OVERLAP_SAMPLES) {
        n_ov = CONFIG_WF_OVERLAP_SAMPLES;
    }
    uint32_t n_new = end > first ? end - first : 0;
    if (n_new + n_ov > PKT_DATA_MAX_SAMPLES) {
        n_new = PKT_DATA_MAX_SAMPLES - n_ov;    /* phần dư sang khung sau */
    }
    if (n_new == 0) {
        s_frame++;
        return;
    }

    uint32_t total = n_ov + n_new;
    if (sbuf_read(first - n_ov, s_tmp, total) != total) {
        s_overruns += n_new;
        s_next_seq = first + n_new;
        s_frame++;
        return;
    }

    *p = (pend_t){
        .used = true,
        .frame_idx = s_frame,
        .first_seq = first,
        .n_new = (uint16_t)n_new,
        .n_ov = (uint8_t)n_ov,
        .t_first = timesync_to_common(s_tmp[0].t_us),
        .t_last = timesync_to_common(s_tmp[total - 1].t_us),
        .t_first_local = s_tmp[0].t_us,
    };
    s_next_seq = first + n_new;
    s_frame++;
}

static void frame_send(pend_t *p, int64_t now)
{
    uint32_t total = (uint32_t)p->n_ov + p->n_new;
    if (sbuf_read(p->first_seq - p->n_ov, s_tmp, total) != total) {
        ESP_LOGW(TAG, "frame %" PRIu32 " overwritten before ACK", p->frame_idx);
        s_dropped++;
        p->used = false;
        return;
    }

    uint32_t fifo_ovf, ring_drop;
    imu_src_get_drops(&fifo_ovf, &ring_drop);

    pkt_data_t *d = (pkt_data_t *)s_tx;
    proto_hdr_init(&d->hdr, PKT_DATA);
    d->node_id = NODE_ID;
    d->retry = p->tries;
    d->session = s_session;
    d->frame_idx = p->frame_idx;
    d->last_seq = p->first_seq + p->n_new - 1;
    d->t_first_us = p->t_first;
    d->t_last_us = p->t_last;
    d->t_first_local_us = p->t_first_local;
    d->fifo_ovf = (uint16_t)fifo_ovf;
    d->ring_drop = (uint16_t)ring_drop;
    d->n_total = (uint16_t)total;
    d->n_overlap = p->n_ov;
    for (uint32_t i = 0; i < total; i++) {
        pkt_sample_t s = { .x = s_tmp[i].x, .y = s_tmp[i].y, .z = s_tmp[i].z };
        memcpy(&d->s[i], &s, sizeof(s));
    }

    link_send(s_c3_mac, d, sizeof(*d) + total * sizeof(pkt_sample_t));
    if (p->tries == 0) {
        s_sent++;
    } else {
        s_resent++;
    }
    p->tries++;
    p->last_tx = now;
}

/* Mỗi khe gửi 1 gói: khung cũ nhất còn chờ, đủ cho 10 gói/giây với beacon 100 ms */
static void slot_tick(int64_t now)
{
    if (s_slot_at == 0 || now < s_slot_at) {
        return;
    }
    s_slot_at = 0;

    if (!s_joined) {
        pkt_join_t j;
        proto_hdr_init(&j.hdr, PKT_JOIN);
        j.node_id = NODE_ID;
        link_send(s_c3_mac, &j, sizeof(j));
        return;
    }

    pend_t *best = NULL;
    for (int i = 0; i < MAX_PENDING; i++) {
        pend_t *p = &s_pend[i];
        if (!p->used || (p->tries > 0 && now - p->last_tx < RETRY_US)) {
            continue;
        }
        if (best == NULL || p->frame_idx < best->frame_idx) {
            best = p;
        }
    }
    if (best) {
        frame_send(best, now);
    }
}

/* ------------------------------------------------------------------------- */

#if CONFIG_WF_SYNC_TEST
static volatile int64_t s_edge_us;
static volatile bool s_edge_pending;

static void IRAM_ATTR sync_test_isr(void *arg)
{
    s_edge_us = esp_timer_get_time();
    s_edge_pending = true;
}

static void sync_test_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_WF_SYNC_TEST_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_WF_SYNC_TEST_GPIO, sync_test_isr, NULL));
    ESP_LOGI(TAG, "sync test: listening on GPIO%d", CONFIG_WF_SYNC_TEST_GPIO);
}

static void sync_test_tick(void)
{
    if (!s_edge_pending) {
        return;
    }
    s_edge_pending = false;
    if (!s_have_c3 || !timesync_valid()) {
        return;
    }
    pkt_sync_probe_t p;
    proto_hdr_init(&p.hdr, PKT_SYNC_PROBE);
    p.node_id = NODE_ID;
    p.t_edge_us = timesync_to_common(s_edge_us);
    link_send(s_c3_mac, &p, sizeof(p));
}
#endif

/* ------------------------------------------------------------------------- */

static void status_tick(int64_t now, int64_t *next)
{
    if (s_have_c3 && !s_c3_lost_logged && now - s_last_beacon > C3_LOST_US) {
        ESP_LOGW(TAG, "no beacon from C3");
        s_c3_lost_logged = true;
    }
    if (now < *next) {
        return;
    }
    *next = now + STATUS_PERIOD_US;

    link_stats_t ls;
    link_get_stats(&ls);
    ESP_LOGI(TAG, "[status] node %d %s sync=%s %.1f ppm | %s sess=%u frame=%" PRIu32
             " sent=%" PRIu32 " resent=%" PRIu32 " pend=%d drop=%" PRIu32 " ovr=%" PRIu32
             " | tx ok/fail=%" PRIu32 "/%" PRIu32 " rx=%" PRIu32,
             NODE_ID, s_joined ? "joined" : "no-C3", timesync_valid() ? "ok" : "no",
             timesync_ppm(), s_active ? "RUN" : "IDLE", s_session, s_frame, s_sent, s_resent,
             pending_count(), s_dropped, s_overruns, ls.tx_ok, ls.tx_fail, ls.rx);
}

void wf_node_run(const imu_src_cfg_t *imu)
{
    vTaskPrioritySet(NULL, TASK_PRIO);

    ESP_ERROR_CHECK(sbuf_init());
    ESP_ERROR_CHECK(link_init(CONFIG_WF_ESPNOW_CHANNEL));
    timesync_reset();

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    s_my_tail = proto_mac_tail(mac);

    /* IMU chạy liên tục để 10 mẫu cũ của khung đầu tiên luôn có sẵn trong RAM */
    ESP_ERROR_CHECK(imu_src_init(imu));
    ESP_ERROR_CHECK(imu_src_start());

#if CONFIG_WF_SYNC_TEST
    sync_test_init();
#endif

    ESP_LOGI(TAG, "node %d ready, MAC " MACSTR, NODE_ID, MAC2STR(mac));

    int64_t next_status = esp_timer_get_time() + STATUS_PERIOD_US;
    for (;;) {
        link_rx_t *m = link_recv(pdMS_TO_TICKS(1));
        if (m) {
            if (proto_check(m->data, m->len) == PKT_BEACON) {
                on_beacon(m);
            }
            free(m);
        }

        int64_t now = esp_timer_get_time();
        frame_tick(now);
        slot_tick(now);
#if CONFIG_WF_SYNC_TEST
        sync_test_tick();
#endif
        status_tick(now, &next_status);
    }
}
