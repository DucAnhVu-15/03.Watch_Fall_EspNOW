/*
 * Giao thức ESP-NOW giữa 2 node S3 và bộ thu C3.
 * Tất cả gói tin đều packed, little-endian.
 *
 * C3 phát BEACON broadcast theo chu kỳ, một gói làm 4 việc: tìm C3, đồng bộ thời gian,
 * điều khiển START/STOP và ACK khung dữ liệu. Node chỉ gửi trong khe giờ của mình
 * ngay sau beacon nên 2 node không đụng nhau.
 *
 * Mốc thời gian _us theo micro-giây. "Đồng hồ chung" là esp_timer của C3.
 * Khung thứ k gồm các mẫu mới trong [k, k+1) * CONFIG_WF_FRAME_MS của đồng hồ chung,
 * nên frame_idx giống nhau ở 2 node luôn là cùng một khoảng thời gian.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_now.h"
#include "sdkconfig.h"

#define PROTO_MAGIC    0x5746u   /* "FW" */
#define PROTO_VERSION  2
#define PROTO_NODES    2         /* node 1 = Master (tay phải), node 2 = Slave (tay trái) */
#define PROTO_ACK_BITS 8         /* số khung đến sau next_frame mà bitmap theo dõi */
#define PROTO_NO_FRAME 0xFFFFFFFFu

#define PROTO_FRAME_US (CONFIG_WF_FRAME_MS * 1000LL)

typedef enum {
    PKT_BEACON     = 1,  /* C3 -> tất cả: đồng hồ, START/STOP, ACK */
    PKT_JOIN       = 2,  /* node -> C3: xin vào mạng               */
    PKT_DATA       = 3,  /* node -> C3: khung dữ liệu IMU          */
    PKT_SYNC_PROBE = 4,  /* node -> C3: test đồng bộ bằng dây GPIO */
} pkt_type_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
} pkt_hdr_t;

/* Trạng thái nhận của 1 node trên C3 */
typedef struct __attribute__((packed)) {
    uint16_t mac_tail;      /* 2 byte cuối MAC của node đã pair, 0 = chưa pair */
    uint32_t next_frame;    /* khung kế tiếp C3 cần; mọi khung trước đó đã xong */
    uint8_t  bitmap;        /* bit i = 1: đã nhận khung next_frame + 1 + i      */
} pkt_ack_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    uint32_t  seq;
    int64_t   t_c3_us;      /* đồng hồ chung lúc phát                        */
    uint16_t  session;      /* số thứ tự bộ file, 0 = chưa ghi lần nào       */
    uint8_t   running;      /* 1: phiên đang mở                              */
    uint32_t  start_frame;  /* khung đầu tiên của phiên                      */
    uint32_t  stop_frame;   /* khung cuối của phiên, PROTO_NO_FRAME: chưa STOP */
    pkt_ack_t ack[PROTO_NODES];
} pkt_beacon_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    uint8_t   node_id;
} pkt_join_t;

typedef struct __attribute__((packed)) {
    int16_t x, y, z;        /* raw 14-bit căn trái (MMA8451: 1g ~ 4096 @ ±8g) */
} pkt_sample_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t    hdr;
    uint8_t      node_id;
    uint8_t      retry;        /* lần gửi lại (0 = lần đầu)                    */
    uint16_t     session;
    uint32_t     frame_idx;
    uint32_t     last_seq;     /* số thứ tự của mẫu s[n_total-1] trong node    */
    int64_t      t_first_us;   /* thời điểm mẫu s[0], đồng hồ chung            */
    int64_t      t_last_us;    /* thời điểm mẫu s[n_total-1], đồng hồ chung    */
    int64_t      t_first_local_us; /* thời điểm mẫu s[0], đồng hồ của node     */
    uint16_t     fifo_ovf;     /* số lần FIFO cảm biến tràn, cộng dồn          */
    uint16_t     ring_drop;    /* số mẫu ULP phải bỏ, cộng dồn                 */
    uint16_t     n_total;      /* = n_overlap + số mẫu mới                     */
    uint8_t      n_overlap;    /* số mẫu cũ ở đầu khung                        */
    pkt_sample_t s[];
} pkt_data_t;

#define PKT_DATA_MAX_SAMPLES ((ESP_NOW_MAX_DATA_LEN_V2 - sizeof(pkt_data_t)) / sizeof(pkt_sample_t))

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    uint8_t   node_id;
    int64_t   t_edge_us;    /* cạnh GPIO, đã quy về đồng hồ chung */
} pkt_sync_probe_t;

/* Chừa 10% vì ODR thật của cảm biến có thể lệch so với danh định */
_Static_assert(CONFIG_WF_SAMPLE_RATE_HZ * CONFIG_WF_FRAME_MS / 1000 * 11 / 10 + CONFIG_WF_OVERLAP_SAMPLES
               <= PKT_DATA_MAX_SAMPLES,
               "Khung dữ liệu vượt quá 1 gói ESP-NOW v2 (1470 byte)");

static inline void proto_hdr_init(pkt_hdr_t *h, pkt_type_t type)
{
    h->magic = PROTO_MAGIC;
    h->version = PROTO_VERSION;
    h->type = (uint8_t)type;
}

/* Trả về loại gói nếu hợp lệ, 0 nếu không */
static inline uint8_t proto_check(const void *buf, int len)
{
    pkt_hdr_t h;
    if (len < (int)sizeof(h)) {
        return 0;
    }
    memcpy(&h, buf, sizeof(h));
    if (h.magic != PROTO_MAGIC || h.version != PROTO_VERSION) {
        return 0;
    }
    return h.type;
}

static inline uint16_t proto_mac_tail(const uint8_t mac[6])
{
    return (uint16_t)((mac[4] << 8) | mac[5]);
}
