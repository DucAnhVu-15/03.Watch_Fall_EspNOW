/*
 * Giao thức ESP-NOW giữa Master và Slave.
 * Tất cả gói tin đều packed, little-endian.
 * Mốc thời gian (_us) là micro-giây theo esp_timer_get_time().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_now.h"
#include "sdkconfig.h"

#define PROTO_MAGIC    0x5746u   /* "FW" */
#define PROTO_VERSION  1

typedef enum {
    PKT_SYNC_REQ   = 1,  /* S->M: yêu cầu đồng bộ thời gian (t1)          */
    PKT_SYNC_RESP  = 2,  /* M->S: trả lời (t1, t2, t3) + trạng thái Master */
    PKT_CMD        = 3,  /* M->S: START / STOP                             */
    PKT_DATA       = 4,  /* S->M: khung dữ liệu IMU                        */
    PKT_DATA_ACK   = 5,  /* M->S: xác nhận đã nhận khung                   */
    PKT_SYNC_PROBE = 6,  /* S->M: test độ chính xác đồng bộ bằng GPIO      */
} pkt_type_t;

typedef enum {
    CMD_STOP  = 0,
    CMD_START = 1,
} cmd_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  version;
    uint8_t  type;
} pkt_hdr_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    uint32_t  seq;
    int64_t   t1_us;        /* Slave: thời điểm gửi (đồng hồ Slave)  */
} pkt_sync_req_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    uint32_t  seq;
    int64_t   t1_us;        /* copy lại từ SYNC_REQ                  */
    int64_t   t2_us;        /* Master: thời điểm nhận (đồng hồ Master) */
    int64_t   t3_us;        /* Master: thời điểm gửi (đồng hồ Master)  */
    uint8_t   running;      /* Master đang ghi hay không             */
    uint16_t  session;      /* phiên ghi hiện tại                    */
} pkt_sync_resp_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    uint8_t   cmd;          /* cmd_t */
    uint16_t  session;
} pkt_cmd_t;

typedef struct __attribute__((packed)) {
    int16_t x, y, z;        /* raw 14-bit căn trái (MMA8451: 1g ~ 4096 @ ±8g) */
} pkt_sample_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t    hdr;
    uint16_t     session;
    uint32_t     frame_idx;
    int64_t      t_first_us;   /* thời điểm mẫu s[0], đã quy về đồng hồ Master     */
    int64_t      t_last_us;    /* thời điểm mẫu s[n_total-1], đồng hồ Master       */
    uint16_t     n_total;      /* = n_overlap + số mẫu mới                         */
    uint8_t      n_overlap;    /* số mẫu cũ ở đầu khung                            */
    uint8_t      retry;        /* lần gửi lại (0 = lần đầu)                        */
    uint32_t     sync_rtt_us;  /* RTT của mẫu đồng bộ đang dùng (chẩn đoán)        */
    pkt_sample_t s[];
} pkt_data_t;

#define PKT_DATA_MAX_SAMPLES ((ESP_NOW_MAX_DATA_LEN_V2 - sizeof(pkt_data_t)) / sizeof(pkt_sample_t))

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    uint16_t  session;
    uint32_t  frame_idx;
} pkt_data_ack_t;

typedef struct __attribute__((packed)) {
    pkt_hdr_t hdr;
    int64_t   t_edge_master_us;  /* thời điểm cạnh GPIO Slave bắt được, quy về đồng hồ Master */
} pkt_sync_probe_t;

_Static_assert(CONFIG_WF_FRAME_SAMPLES + CONFIG_WF_OVERLAP_SAMPLES <= PKT_DATA_MAX_SAMPLES,
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
