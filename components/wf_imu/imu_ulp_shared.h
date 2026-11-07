/*
 * Vùng nhớ dùng chung giữa ULP (ulp/ulp_main.c) và main core (imu_ulp.c).
 *
 * Biến nằm trong RTC slow memory. Phía main core ký hiệu có thêm tiền tố "ulp_"
 * do ulp_embed_binary sinh ra; khai báo lại ở đây để có volatile (header sinh tự
 * động thì không có).
 *
 * ring là hàng đợi 1 ghi (ULP) - 1 đọc (main core). ring_head/ring_tail là bộ đếm
 * tự do; mỗi mẫu chiếm 2 word: [x | y << 16] [z], dùng truy cập 32-bit.
 */
#pragma once

#include <stdint.h>

#define ULP_RING_SAMPLES        128     /* phải là lũy thừa của 2 */
#define ULP_RING_WORDS          (ULP_RING_SAMPLES * 2)

/* Giá trị batch_phase: cho ISR biết tín hiệu vừa nhận mang ý nghĩa gì */
#define ULP_PHASE_ANCHOR        1       /* vừa thấy FIFO đạt watermark -> chốt thời điểm */
#define ULP_PHASE_READY         2       /* đã đưa xong batch vào ring                    */

#if defined(__riscv)
#define ULP_SYM(name) name
#else
#define ULP_SYM(name) ulp_##name
#endif

extern volatile uint32_t ULP_SYM(ring_head);
extern volatile uint32_t ULP_SYM(ring_tail);
extern volatile uint32_t ULP_SYM(ring)[ULP_RING_WORDS];
extern volatile uint32_t ULP_SYM(batch_phase);

/* Thống kê chẩn đoán, chỉ ULP ghi */
extern volatile uint32_t ULP_SYM(stat_i2c_errors);
extern volatile uint32_t ULP_SYM(stat_fifo_overflows);
extern volatile uint32_t ULP_SYM(stat_ring_drops);
extern volatile uint32_t ULP_SYM(stat_poll_batches);    /* batch phát hiện bằng F_STATUS thay vì INT1 */
