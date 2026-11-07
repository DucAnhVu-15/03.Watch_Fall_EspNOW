/*
 * Watch Slave (cổ tay trái)
 *  - Tìm Master bằng SYNC_REQ broadcast, sau đó đồng bộ thời gian định kỳ (unicast).
 *  - Theo trạng thái Master (SYNC_RESP / CMD): START thì bắt đầu đọc IMU, STOP thì dừng.
 *  - Mỗi CONFIG_WF_FRAME_SAMPLES mẫu mới -> gửi 1 khung DATA (kèm overlap),
 *    timestamp đã quy về đồng hồ Master. Không có ACK thì gửi lại.
 */
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "espnow_link.h"
#include "imu_source.h"
#include "proto.h"
#include "sample_buf.h"
#include "timesync.h"

#define LOOP_WAIT_MS         10
#define DISCOVER_PERIOD_US   200000     /* broadcast tìm Master / đồng bộ lúc chưa ổn định */
#define MASTER_LOST_US       5000000
#define STATUS_PERIOD_US     5000000
#define MAX_PENDING          4
#define RETRY_TIMEOUT_US     300000
#define MAX_TRIES            5

static const char *TAG = "slave";

typedef struct {
    bool     used;
    uint32_t frame_idx;
    uint8_t  tries;
    int64_t  last_tx;
    uint16_t len;
    uint8_t *buf;
} pending_t;

static uint8_t  s_master_mac[6];
static bool     s_paired;
static bool     s_running;
static uint16_t s_session;
static uint32_t s_frame_idx;
static uint32_t s_sync_seq;
static int64_t  s_next_sync;
static int64_t  s_last_resp;
static bool     s_master_lost_logged;
static pending_t s_pend[MAX_PENDING];
static uint32_t s_frames_sent, s_frames_dropped, s_overruns;

/* ------------------------------------------------------------------------- */

static void pending_free(pending_t *p)
{
    free(p->buf);
    p->buf = NULL;
    p->used = false;
}

static void pending_clear_all(void)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        if (s_pend[i].used) {
            pending_free(&s_pend[i]);
        }
    }
}

static int pending_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_PENDING; i++) {
        n += s_pend[i].used;
    }
    return n;
}

static void pending_add(uint32_t frame_idx, uint8_t *buf, uint16_t len, int64_t now)
{
    pending_t *slot = NULL;
    for (int i = 0; i < MAX_PENDING; i++) {
        if (!s_pend[i].used) {
            slot = &s_pend[i];
            break;
        }
        if (slot == NULL || s_pend[i].frame_idx < slot->frame_idx) {
            slot = &s_pend[i];   /* đầy: ghi đè khung cũ nhất */
        }
    }
    if (slot->used) {
        ESP_LOGW(TAG, "pending full, drop frame %" PRIu32, slot->frame_idx);
        s_frames_dropped++;
        pending_free(slot);
    }
    *slot = (pending_t){ .used = true, .frame_idx = frame_idx, .tries = 1,
                         .last_tx = now, .len = len, .buf = buf };
}

/* ------------------------------------------------------------------------- */

static void start_session(uint16_t session)
{
    pending_clear_all();
    sbuf_reset();
    s_session = session;
    s_frame_idx = 0;
    imu_src_start();
    s_running = true;
    ESP_LOGI(TAG, "START session %u", session);
}

static void stop_session(void)
{
    imu_src_stop();
    s_running = false;
    ESP_LOGI(TAG, "STOP session %u", s_session);
}

static void apply_master_state(bool running, uint16_t session)
{
    if (running) {
        /* Chỉ bắt đầu khi đã đồng bộ thời gian, để timestamp ngay từ khung đầu đã đúng */
        if (timesync_valid() && (!s_running || session != s_session)) {
            start_session(session);
        }
    } else if (s_running) {
        stop_session();
    }
}

/* ------------------------------------------------------------------------- */

static void send_sync_req(void)
{
    pkt_sync_req_t r;
    proto_hdr_init(&r.hdr, PKT_SYNC_REQ);
    r.seq = ++s_sync_seq;
    r.t1_us = esp_timer_get_time();
    link_send(s_paired ? s_master_mac : LINK_BCAST, &r, sizeof(r));
}

static void handle_sync_resp(const link_rx_t *m)
{
    pkt_sync_resp_t r;
    if (m->len < sizeof(r)) {
        return;
    }
    memcpy(&r, m->data, sizeof(r));

    if (!s_paired) {
        memcpy(s_master_mac, m->src, 6);
        if (link_add_peer(s_master_mac) != ESP_OK) {
            return;
        }
        s_paired = true;
        ESP_LOGI(TAG, "paired with master " MACSTR, MAC2STR(s_master_mac));
    } else if (memcmp(m->src, s_master_mac, 6) != 0) {
        return;
    }

    s_last_resp = m->rx_us;
    s_master_lost_logged = false;
    timesync_add(r.t1_us, r.t2_us, r.t3_us, m->rx_us);
    apply_master_state(r.running, r.session);
}

static void handle_cmd(const link_rx_t *m)
{
    pkt_cmd_t c;
    if (!s_paired || m->len < sizeof(c) || memcmp(m->src, s_master_mac, 6) != 0) {
        return;
    }
    memcpy(&c, m->data, sizeof(c));
    ESP_LOGI(TAG, "CMD %s session %u", c.cmd == CMD_START ? "START" : "STOP", c.session);
    apply_master_state(c.cmd == CMD_START, c.session);
}

static void handle_ack(const link_rx_t *m)
{
    pkt_data_ack_t a;
    if (m->len < sizeof(a)) {
        return;
    }
    memcpy(&a, m->data, sizeof(a));
    if (a.session != s_session) {
        return;
    }
    for (int i = 0; i < MAX_PENDING; i++) {
        if (s_pend[i].used && s_pend[i].frame_idx == a.frame_idx) {
            pending_free(&s_pend[i]);
            break;
        }
    }
}

/* ------------------------------------------------------------------------- */

static void sync_tick(int64_t now)
{
    if (now < s_next_sync) {
        return;
    }
    bool fast = !s_paired || !timesync_valid();
    s_next_sync = now + (fast ? DISCOVER_PERIOD_US : CONFIG_WF_SYNC_INTERVAL_MS * 1000LL);
    send_sync_req();

    if (s_paired && !s_master_lost_logged && now - s_last_resp > MASTER_LOST_US) {
        ESP_LOGW(TAG, "master not responding");
        s_master_lost_logged = true;
    }
}

static void frame_tick(int64_t now)
{
    static sample_t tmp[PKT_DATA_MAX_SAMPLES];
    const uint32_t N = CONFIG_WF_FRAME_SAMPLES;

    if (!s_running) {
        return;
    }
    uint32_t new_start = s_frame_idx * N;
    if (sbuf_head() < new_start + N) {
        return;
    }

    uint32_t oldest = sbuf_oldest();
    if (new_start < oldest) {
        /* Buffer bị ghi đè (task bị chặn quá lâu) -> bỏ khung này */
        ESP_LOGW(TAG, "sample overrun, skip frame %" PRIu32, s_frame_idx);
        s_overruns++;
        s_frame_idx++;
        return;
    }
    uint32_t ov = new_start - oldest;
    if (ov > CONFIG_WF_OVERLAP_SAMPLES) {
        ov = CONFIG_WF_OVERLAP_SAMPLES;
    }
    uint32_t total = ov + N;
    if (sbuf_read(new_start - ov, tmp, total) != total) {
        s_overruns++;
        s_frame_idx++;
        return;
    }

    uint16_t len = sizeof(pkt_data_t) + total * sizeof(pkt_sample_t);
    uint8_t *buf = malloc(len);
    if (buf == NULL) {
        s_frames_dropped++;
        s_frame_idx++;
        return;
    }
    pkt_data_t *d = (pkt_data_t *)buf;
    int64_t offset;
    int32_t rtt;
    timesync_get(&offset, &rtt);

    proto_hdr_init(&d->hdr, PKT_DATA);
    d->session = s_session;
    d->frame_idx = s_frame_idx;
    d->t_first_us = tmp[0].t_us + offset;
    d->t_last_us = tmp[total - 1].t_us + offset;
    d->n_total = total;
    d->n_overlap = ov;
    d->retry = 0;
    d->sync_rtt_us = rtt < 0 ? 0 : (uint32_t)rtt;
    for (uint32_t i = 0; i < total; i++) {
        pkt_sample_t s = { .x = tmp[i].x, .y = tmp[i].y, .z = tmp[i].z };
        memcpy(&d->s[i], &s, sizeof(s));
    }

    link_send(s_master_mac, buf, len);
    pending_add(s_frame_idx, buf, len, now);
    s_frames_sent++;
    s_frame_idx++;
}

static void retry_tick(int64_t now)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        pending_t *p = &s_pend[i];
        if (!p->used || now - p->last_tx < RETRY_TIMEOUT_US) {
            continue;
        }
        if (p->tries >= MAX_TRIES) {
            ESP_LOGW(TAG, "no ACK, drop frame %" PRIu32, p->frame_idx);
            s_frames_dropped++;
            pending_free(p);
            continue;
        }
        ((pkt_data_t *)p->buf)->retry = p->tries;
        link_send(s_master_mac, p->buf, p->len);
        p->tries++;
        p->last_tx = now;
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
    if (!s_paired || !timesync_valid()) {
        return;
    }
    pkt_sync_probe_t p;
    proto_hdr_init(&p.hdr, PKT_SYNC_PROBE);
    p.t_edge_master_us = timesync_to_master(s_edge_us);
    link_send(s_master_mac, &p, sizeof(p));
}
#endif

static void status_log(void)
{
    int64_t offset;
    int32_t rtt;
    link_stats_t ls;
    timesync_get(&offset, &rtt);
    link_get_stats(&ls);
    ESP_LOGI(TAG, "[status] %s sess=%u sync=%s offset=%" PRId64 " rtt=%" PRId32
             " | sent=%" PRIu32 " pend=%d drop=%" PRIu32 " ovr=%" PRIu32
             " | tx ok/fail=%" PRIu32 "/%" PRIu32 " rx=%" PRIu32,
             s_running ? "RUN" : "IDLE", s_session, timesync_valid() ? "ok" : "no",
             offset, rtt, s_frames_sent, pending_count(), s_frames_dropped, s_overruns,
             ls.tx_ok, ls.tx_fail, ls.rx);
}

void app_main(void)
{
    imu_src_cfg_t imu = {
        .rate_hz = CONFIG_WF_SAMPLE_RATE_HZ,
        .odr_err_ppm = CONFIG_WF_SIM_ODR_ERR_PPM,
        .phase = 1.0f,
    };

#if CONFIG_WF_IMU_TEST_ONLY
    ESP_ERROR_CHECK(imu_src_init(&imu));    /* task IMU tự in log, app_main kết thúc tại đây */
    return;
#endif

    ESP_ERROR_CHECK(sbuf_init());
    ESP_ERROR_CHECK(link_init(CONFIG_WF_ESPNOW_CHANNEL));
    timesync_reset();

    ESP_ERROR_CHECK(imu_src_init(&imu));
#if CONFIG_WF_SYNC_TEST
    sync_test_init();
#endif

    int64_t next_status = esp_timer_get_time() + STATUS_PERIOD_US;
    for (;;) {
        link_rx_t *m = link_recv(pdMS_TO_TICKS(LOOP_WAIT_MS));
        if (m) {
            switch (proto_check(m->data, m->len)) {
            case PKT_SYNC_RESP:
                handle_sync_resp(m);
                break;
            case PKT_CMD:
                handle_cmd(m);
                break;
            case PKT_DATA_ACK:
                handle_ack(m);
                break;
            default:
                break;
            }
            free(m);
        }

        int64_t now = esp_timer_get_time();
        sync_tick(now);
        frame_tick(now);
        retry_tick(now);
#if CONFIG_WF_SYNC_TEST
        sync_test_tick();
#endif
        if (now >= next_status) {
            next_status += STATUS_PERIOD_US;
            status_log();
        }
    }
}
