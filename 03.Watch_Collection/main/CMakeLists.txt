set(srcs "main.c" "sd_store.c")
if(CONFIG_WF_STORE_SELFTEST)
    list(APPEND srcs "selftest.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES wf_proto wf_link nvs_flash fatfs sdmmc esp_driver_sdspi
                                     esp_driver_spi esp_driver_gpio esp_timer)
