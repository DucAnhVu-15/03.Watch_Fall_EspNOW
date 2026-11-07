idf_component_register(SRCS "espnow_link.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi
                       PRIV_REQUIRES esp_netif esp_event nvs_flash esp_timer)
