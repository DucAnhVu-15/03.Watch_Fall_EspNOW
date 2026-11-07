idf_component_register(SRCS "wf_node.c"
                       INCLUDE_DIRS "include"
                       REQUIRES wf_imu
                       PRIV_REQUIRES wf_proto wf_link wf_timesync wf_sbuf esp_timer esp_driver_gpio)
