/*
 * Watch Master - node 1, cổ tay phải
 * Toàn bộ logic nằm ở component wf_node, dùng chung cho 2 đồng hồ.
 */
#include "esp_err.h"
#include "imu_source.h"
#include "sdkconfig.h"
#include "wf_node.h"

void app_main(void)
{
    imu_src_cfg_t imu = {
        .rate_hz = CONFIG_WF_SAMPLE_RATE_HZ,
        .odr_err_ppm = CONFIG_WF_SIM_ODR_ERR_PPM,
        .phase = 0.0f,
    };

#if CONFIG_WF_IMU_TEST_ONLY
    ESP_ERROR_CHECK(imu_src_init(&imu));    /* task IMU tự in log, app_main kết thúc tại đây */
    return;
#endif

    wf_node_run(&imu);
}
