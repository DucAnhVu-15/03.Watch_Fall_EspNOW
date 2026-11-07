/*
 * Lớp vận chuyển ESP-NOW: khởi tạo Wi-Fi, quản lý peer, gửi/nhận qua queue.
 * Callback nhận chạy trong task Wi-Fi nên chỉ ghi thời điểm nhận + copy dữ liệu,
 * xử lý thực tế nằm ở task ứng dụng (link_recv).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    uint8_t  src[6];
    bool     broadcast;   /* gói gửi tới địa chỉ broadcast        */
    int64_t  rx_us;       /* esp_timer_get_time() lúc vào callback */
    int8_t   rssi;
    uint16_t len;
    uint8_t  data[];
} link_rx_t;

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t rx;
    uint32_t rx_drop;
} link_stats_t;

extern const uint8_t LINK_BCAST[6];

esp_err_t  link_init(uint8_t channel);
esp_err_t  link_add_peer(const uint8_t mac[6]);
esp_err_t  link_send(const uint8_t mac[6], const void *data, size_t len);

/* Trả về gói nhận được (người gọi phải free()), NULL nếu hết thời gian chờ */
link_rx_t *link_recv(TickType_t wait);

void       link_get_stats(link_stats_t *out);
