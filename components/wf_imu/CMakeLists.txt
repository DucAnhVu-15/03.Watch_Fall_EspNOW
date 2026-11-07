if(CONFIG_WF_IMU_BACKEND_ULP)
    set(srcs "imu_ulp.c" "mma8451.c")
else()
    set(srcs "imu_sim.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES ulp    # public: ULP app chỉ thấy include path public
                       PRIV_REQUIRES wf_sbuf esp_timer esp_hw_support)

if(CONFIG_WF_IMU_BACKEND_ULP)
    ulp_embed_binary(ulp_main "ulp/ulp_main.c" "imu_ulp.c")
endif()
