/*
 * Watch Master (cổ tay phải)
 *  - Đọc IMU của chính nó liên tục vào sample_buf.
 *  - Làm server đồng bộ thời gian cho Slave (trả SYNC_RESP kèm trạng thái ghi).
 *  - Nhận khung DATA từ Slave -> ACK -> ghép với đoạn dữ liệu Master cùng thời điểm -> sink.
 *  - Nút BOOT: START/STOP phiên ghi, gửi CMD cho Slave.
 */
#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "espnow_link.h"
#include "frame_sink.h"
#include "imu_source.h"
#include "proto.h"
#include "sample_buf.h"

#define LOOP_WAIT_MS        20
#define FRAME_QUEUE_LEN     4
#define MASTER_WAIT_MS      600    /* chờ dữ liệu Master phủ hết khung Slave */
#define STATUS_PERIOD_US    5000000
#define DEDUP_N             16

static const char *TAG = "master";

static uint8_t  s_slave_mac[6];
static bool     s_paired;
static bool     s_running;
static uint16_t s_session;
static QueueHandle_t s_frame_q;

/* Thống kê theo phiên */
static uint32_t s_recent[DEDUP_N];
static uint8_t  s_recent_pos, s_recent_cnt;
static uint32_t s_uniq_frames, s_dup_frames, s_max_frame;
static int8_t   s_last_rssi;

/* ------------------------------------------------------------------------- */

static bool frame_seen(uint32_t idx)
{
    for (int i = 0; i < s_recent_cnt; i++) {
        if (s_recent[i] == idx) {
            return true;
        }
    }
    s_recent[s_recent_pos] = idx;
    s_recent_pos = (s_recent_pos + 1) % DEDUP_N;
    if (s_recent_cnt < DEDUP_N) {
        s_recent_cnt++;
    }
    return false;
}

static void send_cmd(uint8_t cmd)
{
    if (!s_paired) {
        return;
    }
    pkt_cmd_t c;
    proto_hdr_init(&c.hdr, PKT_CMD);
    c.cmd = cmd;
    c.session = s_session;
    link_send(s_slave_mac, &c, sizeof(c));
}

static void set_running(bool run)
{
    if (run == s_running) {
        return;
    }
    if (run) {
        s_session++;
        s_recent_cnt = s_recent_pos = 0;
        s_uniq_frames = s_dup_frames = s_max_frame = 0;
        sink_begin_session(s_session);
    }
    s_running = run;
    ESP_LOGI(TAG, "%s (session %u)", run ? "START" : "STOP", s_session);
    send_cmd(run ? CMD_START : CMD_STOP);
}

/* ------------------------------------------------------------------------- */

static void handle_sync_req(const link_rx_t *m)
{
    pkt_sync_req_t req;
    if (m->len < sizeof(req)) {
        return;
    }
    memcpy(&req, m->data, sizeof(req));

    if (!s_paired) {
        memcpy(s_slave_mac, m->src, 6);
        if (link_add_peer(s_slave_mac) != ESP_OK) {
            return;
        }
        s_paired = true;
        ESP_LOGI(TAG, "paired with slave " MACSTR, MAC2STR(s_slave_mac));
    } else if (memcmp(m->src, s_slave_mac, 6) != 0) {
        return;
    }

    pkt_sync_resp_t r;
    proto_hdr_init(&r.hdr, PKT_SYNC_RESP);
    r.seq = req.seq;
    r.t1_us = req.t1_us;
    r.t2_us = m->rx_us;
    r.running = s_running;
    r.session = s_session;
    r.t3_us = esp_timer_get_time();
    link_send(s_slave_mac, &r, sizeof(r));
}

/* Trả về true nếu đã chuyển quyền sở hữu m cho writer task */
static bool handle_data(link_rx_t *m)
{
    const pkt_data_t *d = (const pkt_data_t *)m->data;
    if (m->len < sizeof(pkt_data_t) ||
        m->len < sizeof(pkt_data_t) + d->n_total * sizeof(pkt_sample_t) ||
        d->n_overlap > d->n_total) {
        return false;
    }
    if (!s_paired || memcmp(m->src, s_slave_mac, 6) != 0) {
        return false;
    }
    s_last_rssi = m->rssi;

    /* Luôn ACK (kể cả gói trùng / khác phiên) để Slave ngừng gửi lại */
    pkt_data_ack_t a;
    proto_hdr_init(&a.hdr, PKT_DATA_ACK);
    a.session = d->session;
    a.frame_idx = d->frame_idx;
    link_send(s_slave_mac, &a, sizeof(a));

    if (!s_running || d->session != s_session) {
        return false;
    }
    if (frame_seen(d->frame_idx)) {
        s_dup_frames++;
        return false;
    }
    s_uniq_frames++;
    if (d->frame_idx > s_max_frame) {
        s_max_frame = d->frame_idx;
    }
    if (xQueueSend(s_frame_q, &m, 0) != pdTRUE) {
        ESP_LOGW(TAG, "frame queue full, drop frame %" PRIu32, d->frame_idx);
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------- */

static void wait_master_cover(int64_t t_us)
{
    int64_t last;
    for (int waited = 0; waited < MASTER_WAIT_MS; waited += 10) {
        if (sbuf_last_time(&last) && last >= t_us) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void writer_task(void *arg)
{
    sample_t *ms = malloc(PKT_DATA_MAX_SAMPLES * sizeof(sample_t));
    assert(ms);

    for (;;) {
        link_rx_t *m;
        xQueueReceive(s_frame_q, &m, portMAX_DELAY);
        const pkt_data_t *d = (const pkt_data_t *)m->data;

        uint16_t n_new = d->n_total - d->n_overlap;
        int64_t dt = d->n_total > 1 ? (d->t_last_us - d->t_first_us) / (d->n_total - 1) : 0;
        int64_t t_new0 = d->t_first_us + (int64_t)d->n_overlap * dt;

        wait_master_cover(d->t_last_us);

        /* Mẫu Master gần nhất với mẫu mới đầu tiên của Slave (trong ±dt/2) */
        size_t n_m = 0;
        uint8_t ov_m = 0;
        uint32_t seq0;
        if (sbuf_find_time(t_new0 - dt / 2, &seq0)) {
            uint32_t avail_old = seq0 - sbuf_oldest();
            ov_m = d->n_overlap < avail_old ? d->n_overlap : (uint8_t)avail_old;
            n_m = sbuf_read(seq0 - ov_m, ms, ov_m + n_new);
        }
        if (n_m == 0) {
            ov_m = 0;
            ESP_LOGW(TAG, "no master data for frame %" PRIu32, d->frame_idx);
        }

        sink_write(d, dt, ms, n_m, ov_m);
        free(m);
    }
}

/* ------------------------------------------------------------------------- */

#if CONFIG_WF_SYNC_TEST
#define EDGE_HIST 4
static int64_t s_edges[EDGE_HIST];
static uint8_t s_edge_pos;
static portMUX_TYPE s_edge_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_st_min = INT64_MAX, s_st_max = INT64_MIN, s_st_sum;
static uint32_t s_st_n;

static void sync_test_edge_cb(void *arg)
{
    static int level;
    level ^= 1;
    gpio_set_level(CONFIG_WF_SYNC_TEST_GPIO, level);
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL(&s_edge_lock);
    s_edges[s_edge_pos] = t;
    s_edge_pos = (s_edge_pos + 1) % EDGE_HIST;
    portEXIT_CRITICAL(&s_edge_lock);
}

static void sync_test_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_WF_SYNC_TEST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    const esp_timer_create_args_t args = { .callback = sync_test_edge_cb, .name = "sync_test" };
    esp_timer_handle_t tm;
    ESP_ERROR_CHECK(esp_timer_create(&args, &tm));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tm, 500000));
    ESP_LOGI(TAG, "sync test: toggling GPIO%d every 500 ms", CONFIG_WF_SYNC_TEST_GPIO);
}

static void handle_sync_probe(const link_rx_t *m)
{
    pkt_sync_probe_t p;
    if (m->len < sizeof(p)) {
        return;
    }
    memcpy(&p, m->data, sizeof(p));

    int64_t best = INT64_MAX;
    portENTER_CRITICAL(&s_edge_lock);
    for (int i = 0; i < EDGE_HIST; i++) {
        int64_t e = p.t_edge_master_us - s_edges[i];
        if (llabs(e) < llabs(best)) {
            best = e;
        }
    }
    portEXIT_CRITICAL(&s_edge_lock);
    if (llabs(best) > 100000) {
        return;
    }
    s_st_n++;
    s_st_sum += best;
    if (best < s_st_min) s_st_min = best;
    if (best > s_st_max) s_st_max = best;
    if (s_st_n % 10 == 0) {
        ESP_LOGI(TAG, "SYNC ERROR: last=%" PRId64 " us  min=%" PRId64 " max=%" PRId64 " avg=%" PRId64 " (n=%" PRIu32 ")",
                 best, s_st_min, s_st_max, s_st_sum / (int64_t)s_st_n, s_st_n);
    }
}
#endif

/* ------------------------------------------------------------------------- */

static void button_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_WF_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static void button_poll(void)
{
    static uint8_t hist = 0xFF;
    hist = (uint8_t)((hist << 1) | gpio_get_level(CONFIG_WF_BUTTON_GPIO));
    if ((hist & 0x0F) == 0x08) {    /* 1 -> 0 ổn định 3 lần đọc (~60ms) */
        set_running(!s_running);
    }
}

static void status_log(void)
{
    link_stats_t ls;
    link_get_stats(&ls);
    uint32_t expected = s_uniq_frames ? s_max_frame + 1 : 0;
    ESP_LOGI(TAG, "[status] %s sess=%u slave=%s frames=%" PRIu32 " lost=%" PRIu32 " dup=%" PRIu32
             " rssi=%d | tx ok/fail=%" PRIu32 "/%" PRIu32 " rx=%" PRIu32 " drop=%" PRIu32,
             s_running ? "RUN" : "STOP", s_session, s_paired ? "paired" : "none",
             s_uniq_frames, expected - s_uniq_frames, s_dup_frames, s_last_rssi,
             ls.tx_ok, ls.tx_fail, ls.rx, ls.rx_drop);
}

void app_main(void)
{
    imu_src_cfg_t imu = {
        .rate_hz = CONFIG_WF_SAMPLE_RATE_HZ,
        .odr_err_ppm = CONFIG_WF_SIM_ODR_ERR_PPM,
        .phase = 0.0f,
    };

#if CONFIG_WF_IMU_TEST_ONLY
    ESP_ERROR_CHECK(imu_src_init(&imu));    /* task IMU tự in log, app_main kết thúc tại đây */
    return;
#endif

    ESP_ERROR_CHECK(sbuf_init());
    ESP_ERROR_CHECK(link_init(CONFIG_WF_ESPNOW_CHANNEL));

    ESP_ERROR_CHECK(imu_src_init(&imu));
    ESP_ERROR_CHECK(imu_src_start());   /* Master luôn đọc IMU */

    s_frame_q = xQueueCreate(FRAME_QUEUE_LEN, sizeof(link_rx_t *));
    assert(s_frame_q);
    xTaskCreate(writer_task, "writer", 4096, NULL, 5, NULL);

    button_init();
#if CONFIG_WF_SYNC_TEST
    sync_test_init();
#endif

    s_session = (uint16_t)esp_random();   /* tránh trùng phiên cũ sau khi Master reset */
#if CONFIG_WF_MASTER_AUTOSTART
    set_running(true);
#endif

    int64_t next_status = esp_timer_get_time() + STATUS_PERIOD_US;
    for (;;) {
        link_rx_t *m = link_recv(pdMS_TO_TICKS(LOOP_WAIT_MS));
        if (m) {
            bool consumed = false;
            switch (proto_check(m->data, m->len)) {
            case PKT_SYNC_REQ:
                handle_sync_req(m);
                break;
            case PKT_DATA:
                consumed = handle_data(m);
                break;
#if CONFIG_WF_SYNC_TEST
            case PKT_SYNC_PROBE:
                handle_sync_probe(m);
                break;
#endif
            default:
                break;
            }
            if (!consumed) {
                free(m);
            }
        }

        button_poll();

        if (esp_timer_get_time() >= next_status) {
            next_status += STATUS_PERIOD_US;
            status_log();
        }
    }
}
