/*
 * Thử thẻ SD bằng dữ liệu giả của 2 node, bật bằng CONFIG_WF_STORE_SELFTEST.
 */
#pragma once

void selftest_begin(void);
void selftest_tick(void);       /* gọi liên tục trong lúc đang ghi */
void selftest_verify(void);     /* gọi sau khi file đã đóng */
