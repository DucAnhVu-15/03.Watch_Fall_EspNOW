/*
 * Watch Collection (ESP32-C3 + thẻ SD): bộ thu trung tâm của 2 đồng hồ.
 *
 *  - Phát BEACON mỗi CONFIG_WF_BEACON_MS: đồng hồ chung, START/STOP, ACK.
 *  - Nhận JOIN, lưu MAC theo node_id vào NVS. Giữ cả 2 nút lúc bật nguồn để pair lại.
 *  - Nhận khung DATA: kiểm tra node_id khớp MAC, lọc trùng, đưa vào hàng đợi ghi thẻ.
 *    Chỉ ACK khi khung đã vào hàng đợi; hàng đợi đầy thì node sẽ tự gửi lại.
 *  - SW1 (START) mở bộ file có số kế tiếp. SW2 (STOP) chờ node gửi hết rồi đóng file.
 */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "espnow_link.h"
#include "proto.h"
#include "sd_store.h"
#include "selftest.h"

#define TASK_PRIO           10      /* cao hơn task ghi SD để beacon không bị trễ */
#define BUTTON_PERIOD_US    10000
#define STATUS_PERIOD_US    5000000
#define BEACON_US           (CONFIG_WF_BEACON_MS * 1000LL)
#define START_DELAY_FRAMES  2       /* chờ để 2 node nhận lệnh và bắt đầu cùng 1 khung */
#define DRAIN_TIMEOUT_US    8000000 /* bằng thời gian node còn giữ khung để gửi lại */
#define NODE_LOST_US        3000000
#define PAIR_NVS_NS         "wf_pair"

typedef enum { ST_IDLE, ST_RECORDING, ST_DRAINING, ST_CLOSING } state_t;

typedef struct {
    uint8_t  mac[6];
    bool     paired;
    uint32_t next_frame;    /* khung kế tiếp cần nhận */
    uint8_t  bitmap;        /* bit i: đã nhận khung next_frame + 1 + i */
    uint32_t frames, dups, lost_frames, rejected, queue_full;
    int8_t   rssi;
    int64_t  last_seen;
} node_t;

static const char *TAG = "collect";

static node_t   s_nodes[PROTO_NODES];
static state_t  s_state = ST_IDLE;
static bool     s_card_ok;
static uint16_t s_session;
static uint32_t s_start_frame;
static uint32_t s_stop_frame = PROTO_NO_FRAME;
static int64_t  s_drain_deadline;
static uint32_t s_beacon_seq;

/* ------------------------------------------------------------------------- */
/* Pair */

static const char *const PAIR_KEY[PROTO_NODES] = { "mac1", "mac2" };

static void pair_load(void)
{
    nvs_handle_t h;
    if (nvs_open(PAIR_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    for (int i = 0; i < PROTO_NODES; i++) {
        size_t size = sizeof(s_nodes[i].mac);
        if (nvs_get_blob(h, PAIR_KEY[i], s_nodes[i].mac, &size) == ESP_OK &&
            link_add_peer(s_nodes[i].mac) == ESP_OK) {
            s_nodes[i].paired = true;
            ESP_LOGI(TAG, "node %d paired: " MACSTR, i + 1, MAC2STR(s_nodes[i].mac));
        }
    }
    nvs_close(h);
}

static void pair_save(int idx)
{
    nvs_handle_t h;
    if (nvs_open(PAIR_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_blob(h, PAIR_KEY[idx], s_nodes[idx].mac, 6) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

static void pair_clear(void)
{
    nvs_handle_t h;
    if (nvs_open(PAIR_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "pairing cleared");
}

static void on_join(const link_rx_t *m)
{
    pkt_join_t j;
    if (m->len < sizeof(j)) {
        return;
    }
    memcpy(&j, m->data, sizeof(j));
    if (j.node_id < 1 || j.node_id > PROTO_NODES) {
        return;
    }

    node_t *n = &s_nodes[j.node_id - 1];
    if (n->paired) {
        if (memcmp(n->mac, m->src, 6) != 0) {
            ESP_LOGW(TAG, "JOIN rejected: node %u is " MACSTR ", got " MACSTR
                     ". Hold both buttons at power-on to pair again",
                     j.node_id, MAC2STR(n->mac), MAC2STR(m->src));
        }
        return;
    }
    for (int i = 0; i < PROTO_NODES; i++) {
        if (s_nodes[i].paired && memcmp(s_nodes[i].mac, m->src, 6) == 0) {
            ESP_LOGW(TAG, "JOIN rejected: " MACSTR " is already node %d, check CONFIG_WF_NODE_ID",
                     MAC2STR(m->src), i + 1);
            return;
        }
    }
    if (link_add_peer(m->src) != ESP_OK) {
        return;
    }
    memcpy(n->mac, m->src, 6);
    n->paired = true;
    pair_save(j.node_id - 1);
    ESP_LOGI(TAG, "node %u paired: " MACSTR, j.node_id, MAC2STR(n->mac));
}

/* ------------------------------------------------------------------------- */
/* Nhận khung */

/* Coi next_frame là xong rồi tiến tới khung chưa có đầu tiên */
static void ack_advance(node_t *n)
{
    bool have;
    do {
        n->next_frame++;
        have = n->bitmap & 1;
        n->bitmap >>= 1;
    } while (have);
}

static bool ack_is_duplicate(const node_t *n, uint32_t f)
{
    if (f < n->next_frame) {
        return true;
    }
    uint32_t off = f - n->next_frame;
    return off >= 1 && off <= PROTO_ACK_BITS && (n->bitmap & (1u << (off - 1)));
}

static void ack_mark(node_t *n, uint32_t f)
{
    /* Khung vượt quá cửa sổ: khung cũ nhất coi như mất để node không chờ mãi */
    while (f - n->next_frame > PROTO_ACK_BITS) {
        n->lost_frames++;
        ack_advance(n);
    }
    uint32_t off = f - n->next_frame;
    if (off == 0) {
        ack_advance(n);
    } else {
        n->bitmap |= 1u << (off - 1);
    }
}

/* Trả về true nếu đã chuyển quyền sở hữu m cho sd_store */
static bool on_data(link_rx_t *m)
{
#if CONFIG_WF_STORE_SELFTEST
    return false;   /* đang thử thẻ bằng dữ liệu giả */
#endif
    const pkt_data_t *d = (const pkt_data_t *)m->data;
    if (m->len < sizeof(pkt_data_t) ||
        m->len < sizeof(pkt_data_t) + d->n_total * sizeof(pkt_sample_t) ||
        d->n_overlap > d->n_total || d->node_id < 1 || d->node_id > PROTO_NODES) {
        return false;
    }

    node_t *n = &s_nodes[d->node_id - 1];
    if (!n->paired || memcmp(m->src, n->mac, 6) != 0) {
        n->rejected++;
        return false;
    }
    n->last_seen = m->rx_us;
    n->rssi = m->rssi;

    if ((s_state != ST_RECORDING && s_state != ST_DRAINING) || d->session != s_session) {
        return false;
    }
    if (ack_is_duplicate(n, d->frame_idx)) {
        n->dups++;
        return false;
    }
    uint32_t f = d->frame_idx;      /* sau khi đẩy vào hàng đợi, m thuộc về sd_store */
    if (!store_push_frame(m, d)) {
        n->queue_full++;    /* không ACK: node sẽ gửi lại */
        return false;
    }
    n->frames++;
    ack_mark(n, f);
    return true;
}

static void send_beacon(void)
{
    pkt_beacon_t b;
    proto_hdr_init(&b.hdr, PKT_BEACON);
    b.seq = ++s_beacon_seq;
    b.session = s_session;
    b.running = s_state == ST_RECORDING || s_state == ST_DRAINING;
    b.start_frame = s_start_frame;
    b.stop_frame = s_stop_frame;
    for (int i = 0; i < PROTO_NODES; i++) {
        const node_t *n = &s_nodes[i];
        b.ack[i].mac_tail = n->paired ? proto_mac_tail(n->mac) : 0;
        b.ack[i].next_frame = n->next_frame;
        b.ack[i].bitmap = n->bitmap;
    }
    b.t_c3_us = esp_timer_get_time();   /* lấy giờ ngay trước khi gửi */
    link_send(LINK_BCAST, &b, sizeof(b));
}

/* ------------------------------------------------------------------------- */
/* START / STOP */

static void recording_start(int64_t now)
{
    if (s_state != ST_IDLE) {
        return;
    }
    if (!s_card_ok) {
        ESP_LOGE(TAG, "START ignored: no SD card");
        return;
    }
    if (store_open() != ESP_OK) {
        ESP_LOGE(TAG, "START failed: cannot create files");
        return;
    }

    s_session = (uint16_t)store_index();
    s_start_frame = (uint32_t)(now / PROTO_FRAME_US) + START_DELAY_FRAMES;
    s_stop_frame = PROTO_NO_FRAME;
    for (int i = 0; i < PROTO_NODES; i++) {
        node_t *n = &s_nodes[i];
        n->next_frame = s_start_frame;
        n->bitmap = 0;
        n->frames = n->dups = n->lost_frames = n->rejected = n->queue_full = 0;
        if (!n->paired) {
            ESP_LOGW(TAG, "node %d is not paired yet", i + 1);
        }
    }
    s_state = ST_RECORDING;
#if CONFIG_WF_STORE_SELFTEST
    selftest_begin();
#endif
    ESP_LOGI(TAG, "START file %u, first frame %" PRIu32, s_session, s_start_frame);
}

static void recording_stop(int64_t now)
{
    if (s_state != ST_RECORDING) {
        return;
    }
    /* Ghi nốt khung đang chạy, thêm thời gian cho node gửi lại */
    uint32_t cur = (uint32_t)(now / PROTO_FRAME_US);
    s_stop_frame = cur >= s_start_frame ? cur : s_start_frame;
    s_drain_deadline = now + PROTO_FRAME_US + DRAIN_TIMEOUT_US;
    s_state = ST_DRAINING;
    ESP_LOGI(TAG, "STOP at frame %" PRIu32 ", waiting for nodes", s_stop_frame);
}

#if !CONFIG_WF_STORE_SELFTEST
static bool nodes_finished(void)
{
    for (int i = 0; i < PROTO_NODES; i++) {
        if (s_nodes[i].paired && s_nodes[i].next_frame <= s_stop_frame) {
            return false;
        }
    }
    return true;
}
#endif

static void recording_tick(int64_t now)
{
    switch (s_state) {
    case ST_RECORDING:
#if CONFIG_WF_STORE_SELFTEST
        selftest_tick();
#endif
        break;

    case ST_DRAINING:
#if !CONFIG_WF_STORE_SELFTEST
        if (!nodes_finished() && now < s_drain_deadline) {
            break;
        }
#endif
        if (store_close_async()) {
            s_state = ST_CLOSING;
        }
        break;

    case ST_CLOSING:
        if (store_close_done()) {
            s_state = ST_IDLE;
            ESP_LOGI(TAG, "file %u closed, safe to remove the card", s_session);
#if CONFIG_WF_STORE_SELFTEST
            selftest_verify();
#endif
        }
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Nút SW1, SW2 */

static void buttons_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CONFIG_WF_BTN_START_GPIO) | (1ULL << CONFIG_WF_BTN_STOP_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

/* Trả về true đúng 1 lần mỗi lần nhấn: mức thấp ổn định qua 3 lần đọc liên tiếp */
static bool button_pressed(int gpio, uint8_t *hist)
{
    *hist = (uint8_t)((*hist << 1) | gpio_get_level(gpio));
    return (*hist & 0x0F) == 0x08;
}

/* ------------------------------------------------------------------------- */
/* Test đồng bộ: cùng một cạnh GPIO, so giờ 2 node báo về */

#if CONFIG_WF_SYNC_TEST
#define PROBE_MATCH_US 100000

static int64_t  s_probe[PROTO_NODES];
static int64_t  s_st_min = INT64_MAX, s_st_max = INT64_MIN, s_st_sum;
static uint32_t s_st_n;

static void sync_test_cb(void *arg)
{
    static int level;
    level ^= 1;
    gpio_set_level(CONFIG_WF_SYNC_TEST_GPIO, level);
}

static void sync_test_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_WF_SYNC_TEST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    const esp_timer_create_args_t args = { .callback = sync_test_cb, .name = "sync_test" };
    esp_timer_handle_t tm;
    ESP_ERROR_CHECK(esp_timer_create(&args, &tm));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tm, 500000));
    ESP_LOGI(TAG, "sync test: toggling GPIO%d every 500 ms", CONFIG_WF_SYNC_TEST_GPIO);
}

static void on_sync_probe(const link_rx_t *m)
{
    pkt_sync_probe_t p;
    if (m->len < sizeof(p)) {
        return;
    }
    memcpy(&p, m->data, sizeof(p));
    if (p.node_id < 1 || p.node_id > PROTO_NODES) {
        return;
    }
    s_probe[p.node_id - 1] = p.t_edge_us;
    if (s_probe[0] == 0 || s_probe[1] == 0) {
        return;
    }

    int64_t diff = s_probe[0] - s_probe[1];
    if (diff > PROBE_MATCH_US || diff < -PROBE_MATCH_US) {
        return;     /* 2 giá trị thuộc 2 cạnh khác nhau */
    }
    s_probe[0] = s_probe[1] = 0;
    s_st_n++;
    s_st_sum += diff;
    s_st_min = diff < s_st_min ? diff : s_st_min;
    s_st_max = diff > s_st_max ? diff : s_st_max;
    if (s_st_n % 10 == 0) {
        ESP_LOGI(TAG, "SYNC node1-node2: last=%" PRId64 " us min=%" PRId64 " max=%" PRId64
                 " avg=%" PRId64 " (n=%" PRIu32 ")",
                 diff, s_st_min, s_st_max, s_st_sum / (int64_t)s_st_n, s_st_n);
    }
}
#endif

/* ------------------------------------------------------------------------- */

static void status_log(int64_t now)
{
    static const char *const STATE[] = { "IDLE", "REC", "DRAIN", "CLOSE" };
    link_stats_t ls;
    link_get_stats(&ls);

    ESP_LOGI(TAG, "[status] %s file=%u | tx ok/fail=%" PRIu32 "/%" PRIu32 " rx=%" PRIu32
             " rx_drop=%" PRIu32, STATE[s_state], s_session, ls.tx_ok, ls.tx_fail, ls.rx, ls.rx_drop);
    for (int i = 0; i < PROTO_NODES; i++) {
        const node_t *n = &s_nodes[i];
        bool seen = n->last_seen != 0 && now - n->last_seen < NODE_LOST_US;
        ESP_LOGI(TAG, "  node %d %s frames=%" PRIu32 " next=%" PRIu32 " dup=%" PRIu32
                 " lost=%" PRIu32 " full=%" PRIu32 " rejected=%" PRIu32 " rssi=%d",
                 i + 1, !n->paired ? "unpaired" : (seen ? "online" : "silent"), n->frames,
                 n->next_frame, n->dups, n->lost_frames, n->queue_full, n->rejected, n->rssi);
    }
}

void app_main(void)
{
    vTaskPrioritySet(NULL, TASK_PRIO);
    buttons_init();

    ESP_ERROR_CHECK(link_init(CONFIG_WF_ESPNOW_CHANNEL));
    if (gpio_get_level(CONFIG_WF_BTN_START_GPIO) == 0 &&
        gpio_get_level(CONFIG_WF_BTN_STOP_GPIO) == 0) {
        pair_clear();
    }
    pair_load();

    esp_err_t err = store_init();
    s_card_ok = err == ESP_OK;
    if (!s_card_ok) {
        ESP_LOGE(TAG, "SD not ready: %s", esp_err_to_name(err));
    }
#if CONFIG_WF_SYNC_TEST
    sync_test_init();
#endif
    ESP_LOGI(TAG, "ready: SW1 (GPIO%d) = START, SW2 (GPIO%d) = STOP",
             CONFIG_WF_BTN_START_GPIO, CONFIG_WF_BTN_STOP_GPIO);

    uint8_t start_hist = 0xFF, stop_hist = 0xFF;
    int64_t now = esp_timer_get_time();
    int64_t next_beacon = now, next_button = now, next_status = now + STATUS_PERIOD_US;

    for (;;) {
        link_rx_t *m = link_recv(pdMS_TO_TICKS(1));
        if (m) {
            bool consumed = false;
            switch (proto_check(m->data, m->len)) {
            case PKT_JOIN:
                on_join(m);
                break;
            case PKT_DATA:
                consumed = on_data(m);
                break;
#if CONFIG_WF_SYNC_TEST
            case PKT_SYNC_PROBE:
                on_sync_probe(m);
                break;
#endif
            default:
                break;
            }
            if (!consumed) {
                free(m);
            }
        }

        now = esp_timer_get_time();
        if (now >= next_beacon) {
            next_beacon += BEACON_US;
            if (next_beacon <= now) {
                next_beacon = now + BEACON_US;
            }
            send_beacon();
        }
        if (now >= next_button) {
            next_button = now + BUTTON_PERIOD_US;
            if (button_pressed(CONFIG_WF_BTN_START_GPIO, &start_hist)) {
                recording_start(now);
            }
            if (button_pressed(CONFIG_WF_BTN_STOP_GPIO, &stop_hist)) {
                recording_stop(now);
            }
        }
        recording_tick(now);

        if (now >= next_status) {
            next_status = now + STATUS_PERIOD_US;
            status_log(now);
        }
    }
}
