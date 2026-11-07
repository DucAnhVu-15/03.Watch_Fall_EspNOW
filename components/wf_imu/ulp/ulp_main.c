/*
 * Chương trình ULP RISC-V: đọc FIFO MMA8451 qua RTC I2C rồi đưa mẫu vào ring.
 *
 * ULP được timer đánh thức mỗi CONFIG_WF_IMU_ULP_POLL_US. Mỗi lần chỉ kiểm tra
 * mức chân INT1 (không tốn I2C); cứ ULP_STATUS_POLL_TICKS lần thì đọc F_STATUS
 * để vẫn chạy được khi INT1 không nối.
 *
 * Mỗi batch báo main core 2 lần:
 *   ANCHOR - ngay khi thấy FIFO đạt watermark: ISR chốt thời điểm mẫu mới nhất.
 *   READY  - khi mẫu đã nằm trong ring (đọc I2C mất cỡ 10 ms nên phải báo riêng).
 *
 * Biến toàn cục giữ nguyên giữa các lần ULP thức (.bss chỉ bị xóa lúc nạp binary).
 */
#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "ulp_riscv_gpio.h"
#include "ulp_riscv_i2c_ulp_core.h"
#include "ulp_riscv_utils.h"
#include "../imu_ulp_shared.h"
#include "../mma8451_defs.h"

#define INT1_GPIO               CONFIG_WF_IMU_INT1_GPIO
#define ULP_STATUS_POLL_TICKS   25
#define RING_MASK               (ULP_RING_SAMPLES - 1)

volatile uint32_t ring_head;
volatile uint32_t ring_tail;
volatile uint32_t ring[ULP_RING_WORDS];
volatile uint32_t batch_phase;

volatile uint32_t stat_i2c_errors;
volatile uint32_t stat_fifo_overflows;
volatile uint32_t stat_ring_drops;
volatile uint32_t stat_poll_batches;

static bool s_gpio_ready;
static uint32_t s_idle_ticks;

static inline void signal_main(uint32_t phase)
{
    batch_phase = phase;
    ulp_riscv_wakeup_main_processor();
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    ulp_riscv_i2c_master_set_slave_addr(CONFIG_WF_IMU_I2C_ADDR);
    ulp_riscv_i2c_master_set_slave_reg_addr(reg);
    return ulp_riscv_i2c_master_read_from_device(buf, len);
}

/* Giữ nguyên giá trị raw 14-bit căn trái như trong thanh ghi */
static inline uint32_t raw16(const uint8_t *p)
{
    return ((uint32_t)p[0] << 8) | p[1];
}

static bool drain_fifo(uint8_t n)
{
    uint8_t data[MMA8451_BYTES_PER_SAMPLE * MMA8451_FIFO_SIZE];

    /* Luôn đọc hết để INT1 nhả, kể cả khi ring đầy và phải bỏ mẫu */
    if (read_regs(REG_OUT_X_MSB, data, (size_t)n * MMA8451_BYTES_PER_SAMPLE) != ESP_OK) {
        stat_i2c_errors++;
        return false;
    }

    uint32_t head = ring_head;
    if (ULP_RING_SAMPLES - (head - ring_tail) < n) {
        stat_ring_drops += n;
        return false;
    }

    const uint8_t *p = data;
    for (uint8_t i = 0; i < n; i++, p += MMA8451_BYTES_PER_SAMPLE) {
        uint32_t w = (head & RING_MASK) * 2;
        ring[w] = raw16(p) | (raw16(p + 2) << 16);
        ring[w + 1] = raw16(p + 4);
        head++;
    }
    ring_head = head;   /* công bố sau khi dữ liệu đã ghi xong */
    return true;
}

int main(void)
{
    if (!s_gpio_ready) {
        ulp_riscv_gpio_init(INT1_GPIO);
        ulp_riscv_gpio_input_enable(INT1_GPIO);
        ulp_riscv_gpio_pulldown_disable(INT1_GPIO);
        s_gpio_ready = true;
    }

    bool int1 = ulp_riscv_gpio_get_level(INT1_GPIO) == 0;   /* INT1 tích cực mức thấp */
    if (!int1 && ++s_idle_ticks < ULP_STATUS_POLL_TICKS) {
        return 0;
    }
    s_idle_ticks = 0;

    if (int1) {
        signal_main(ULP_PHASE_ANCHOR);  /* báo trước khi đọc I2C để sát cạnh INT1 nhất */
    }

    uint8_t status;
    if (read_regs(REG_F_STATUS, &status, 1) != ESP_OK) {
        stat_i2c_errors++;
        return 0;
    }
    if (status & F_STATUS_F_OVF) {
        stat_fifo_overflows++;
    }

    uint8_t n = status & F_STATUS_F_CNT_MASK;
    if (n > MMA8451_FIFO_SIZE) {
        n = MMA8451_FIFO_SIZE;
    }
    if (n < CONFIG_WF_IMU_FIFO_WATERMARK) {
        return 0;
    }

    if (!int1) {
        stat_poll_batches++;
        signal_main(ULP_PHASE_ANCHOR);
    }

    if (drain_fifo(n)) {
        signal_main(ULP_PHASE_READY);
    }
    return 0;
}
