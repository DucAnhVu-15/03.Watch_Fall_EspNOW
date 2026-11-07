idf_component_register(SRCS "main.c" "frame_sink.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES wf_proto wf_link wf_sbuf wf_imu esp_timer esp_driver_gpio)
