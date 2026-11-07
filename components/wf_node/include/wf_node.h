/*
 * Logic chung của 2 đồng hồ S3: đọc IMU vào sample_buf, đồng bộ theo beacon của C3,
 * đóng khung theo thời gian và gửi về C3 trong khe giờ của mình.
 * Master và Slave chỉ khác CONFIG_WF_NODE_ID.
 */
#pragma once

#include "imu_source.h"

/* Không trả về */
void wf_node_run(const imu_src_cfg_t *imu);
