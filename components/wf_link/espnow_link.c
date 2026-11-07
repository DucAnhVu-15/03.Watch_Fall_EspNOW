#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "espnow_link.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/queue.h"
#include "nvs_flash.h"

#define RX_QUEUE_LEN   16
#define SEND_RETRIES   5

static const char *TAG = "link";

const uint8_t LINK_BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static QueueHandle_t s_rxq;
static uint8_t s_channel;
static volatile link_stats_t s_stats;

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    int64_t now = esp_timer_get_time();

    if (len <= 0 || len > ESP_NOW_MAX_DATA_LEN_V2) {
        return;
    }
    link_rx_t *m = malloc(sizeof(*m) + len);
    if (m == NULL) {
        s_stats.rx_drop++;
        return;
    }
    memcpy(m->src, info->src_addr, 6);
    m->broadcast = info->des_addr && memcmp(info->des_addr, LINK_BCAST, 6) == 0;
    m->rx_us = now;
    m->rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
    m->len = (uint16_t)len;
    memcpy(m->data, data, len);

    if (xQueueSend(s_rxq, &m, 0) != pdTRUE) {
        free(m);
        s_stats.rx_drop++;
        return;
    }
    s_stats.rx++;
}

static void send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_stats.tx_ok++;
    } else {
        s_stats.tx_fail++;
    }
}

esp_err_t link_init(uint8_t channel)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_rxq = xQueueCreate(RX_QUEUE_LEN, sizeof(link_rx_t *));
    if (s_rxq == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_channel = channel;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    /* Tắt power-save để độ trễ gửi/nhận ổn định (quan trọng cho đồng bộ thời gian) */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));
    ESP_ERROR_CHECK(link_add_peer(LINK_BCAST));

    uint8_t mac[6];
    uint32_t ver = 0;
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    esp_now_get_version(&ver);
    ESP_LOGI(TAG, "ESP-NOW v%" PRIu32 " ready, channel %u, MAC " MACSTR, ver, channel, MAC2STR(mac));
    return ESP_OK;
}

static esp_err_t set_peer_rate(const uint8_t mac[6])
{
    esp_now_rate_config_t cfg = {
#if CONFIG_WF_PHY_RATE_1M
        .phymode = WIFI_PHY_MODE_11B,
        .rate = WIFI_PHY_RATE_1M_L,
#elif CONFIG_WF_PHY_RATE_12M
        .phymode = WIFI_PHY_MODE_11G,
        .rate = WIFI_PHY_RATE_12M,
#elif CONFIG_WF_PHY_RATE_24M
        .phymode = WIFI_PHY_MODE_11G,
        .rate = WIFI_PHY_RATE_24M,
#else
        .phymode = WIFI_PHY_MODE_11G,
        .rate = WIFI_PHY_RATE_6M,
#endif
    };
    return esp_now_set_peer_rate_config(mac, &cfg);
}

esp_err_t link_add_peer(const uint8_t mac[6])
{
    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }
    esp_now_peer_info_t peer = {
        .channel = s_channel,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac, 6);
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        return err;
    }
    /* Beacon broadcast giữ tốc độ mặc định: chậm hơn nhưng xa hơn */
    if (memcmp(mac, LINK_BCAST, 6) != 0) {
        err = set_peer_rate(mac);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "set peer rate failed: %s", esp_err_to_name(err));
        }
    }
    ESP_LOGI(TAG, "peer added " MACSTR, MAC2STR(mac));
    return ESP_OK;
}

esp_err_t link_send(const uint8_t mac[6], const void *data, size_t len)
{
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < SEND_RETRIES; i++) {
        err = esp_now_send(mac, data, len);
        if (err != ESP_ERR_ESPNOW_NO_MEM) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send failed: %s", esp_err_to_name(err));
    }
    return err;
}

link_rx_t *link_recv(TickType_t wait)
{
    link_rx_t *m = NULL;
    if (xQueueReceive(s_rxq, &m, wait) != pdTRUE) {
        return NULL;
    }
    return m;
}

void link_get_stats(link_stats_t *out)
{
    out->tx_ok = s_stats.tx_ok;
    out->tx_fail = s_stats.tx_fail;
    out->rx = s_stats.rx;
    out->rx_drop = s_stats.rx_drop;
}
