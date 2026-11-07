#include <stdbool.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "ulp_riscv_i2c.h"
#include "mma8451.h"
#include "mma8451_defs.h"

static const char *TAG = "mma8451";

static esp_err_t i2c_init(void)
{
    ulp_riscv_i2c_cfg_t cfg = ULP_RISCV_I2C_DEFAULT_CONFIG();
    cfg.i2c_pin_cfg.sda_io_num = CONFIG_WF_IMU_SDA_GPIO;
    cfg.i2c_pin_cfg.scl_io_num = CONFIG_WF_IMU_SCL_GPIO;
    cfg.i2c_pin_cfg.sda_pullup_en = true;
    cfg.i2c_pin_cfg.scl_pullup_en = true;
    cfg.i2c_timing_cfg.scl_low_period = 3;
    cfg.i2c_timing_cfg.scl_high_period = 3;
    cfg.i2c_timing_cfg.sda_duty_period = 1;
    cfg.i2c_timing_cfg.scl_start_period = 3;
    cfg.i2c_timing_cfg.scl_stop_period = 6;
    cfg.i2c_timing_cfg.i2c_trans_timeout = 20;

    ESP_RETURN_ON_ERROR(ulp_riscv_i2c_master_init(&cfg), TAG, "RTC I2C init failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t read_reg(uint8_t reg, uint8_t *value)
{
    ulp_riscv_i2c_master_set_slave_addr(CONFIG_WF_IMU_I2C_ADDR);
    ulp_riscv_i2c_master_set_slave_reg_addr(reg);
    return ulp_riscv_i2c_master_read_from_device(value, 1);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    ulp_riscv_i2c_master_set_slave_addr(CONFIG_WF_IMU_I2C_ADDR);
    ulp_riscv_i2c_master_set_slave_reg_addr(reg);
    esp_err_t err = ulp_riscv_i2c_master_write_to_device(&value, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t rate_to_dr(uint32_t rate_hz, uint8_t *dr)
{
    switch (rate_hz) {
    case 800: *dr = 0; return ESP_OK;
    case 400: *dr = 1; return ESP_OK;
    case 200: *dr = 2; return ESP_OK;
    case 100: *dr = 3; return ESP_OK;
    case 50:  *dr = 4; return ESP_OK;
    default:
        ESP_LOGE(TAG, "unsupported rate %u Hz (use 50/100/200/400/800)", (unsigned)rate_hz);
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t mma8451_start_fifo(uint32_t rate_hz, uint8_t watermark)
{
    uint8_t dr;
    ESP_RETURN_ON_ERROR(rate_to_dr(rate_hz, &dr), TAG, "bad rate");
    ESP_RETURN_ON_ERROR(i2c_init(), TAG, "i2c");

    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(read_reg(REG_WHO_AM_I, &id), TAG,
                        "no I2C response at 0x%02X (SDA=%d SCL=%d)",
                        CONFIG_WF_IMU_I2C_ADDR, CONFIG_WF_IMU_SDA_GPIO, CONFIG_WF_IMU_SCL_GPIO);
    ESP_RETURN_ON_FALSE(id == MMA8451_DEVICE_ID || id == MMA8451_DEVICE_ID_ALT,
                        ESP_ERR_NOT_FOUND, TAG, "unexpected WHO_AM_I 0x%02X", id);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);

    /* Thanh ghi cấu hình chỉ ghi được khi standby */
    ESP_RETURN_ON_ERROR(write_reg(REG_CTRL_REG1, 0x00), TAG, "standby");
    vTaskDelay(pdMS_TO_TICKS(2));

    const struct { uint8_t reg, val; } seq[] = {
        { REG_CTRL_REG2,     CTRL_REG2_MODS_LOW_POWER },
        { REG_CTRL_REG1,     (uint8_t)(dr << CTRL_REG1_DR_SHIFT) },
        { REG_XYZ_DATA_CFG,  XYZ_RANGE_8G },
        { REG_F_SETUP,       (uint8_t)(F_SETUP_MODE_CIRCULAR | (watermark & F_SETUP_WMRK_MASK)) },
        { REG_CTRL_REG4,     0x00 },    /* tắt mọi nguồn ngắt và tính năng phụ */
        { REG_CTRL_REG5,     0x00 },
        { REG_TRIG_CFG,      0x00 },
        { REG_PL_CFG,        0x00 },
        { REG_FF_MT_CFG,     0x00 },
        { REG_TRANSIENT_CFG, 0x00 },
        { REG_PULSE_CFG,     0x00 },
        { REG_ASLP_COUNT,    0x00 },
        { REG_CTRL_REG4,     CTRL_REG4_INT_EN_FIFO },
        { REG_CTRL_REG5,     CTRL_REG5_INT_CFG_FIFO },
        { REG_CTRL_REG3,     0x00 },    /* INT1 push-pull, tích cực mức thấp */
        { REG_CTRL_REG1,     (uint8_t)((dr << CTRL_REG1_DR_SHIFT) | CTRL_REG1_ACTIVE) },
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        ESP_RETURN_ON_ERROR(write_reg(seq[i].reg, seq[i].val), TAG, "config");
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* INT là mức: đọc các thanh ghi nguồn ngắt để xóa trạng thái cũ trước khi vào việc */
    const uint8_t src[] = { REG_F_STATUS, REG_PL_STATUS, REG_FF_MT_SRC, REG_TRANSIENT_SRC,
                            REG_PULSE_SRC, REG_SYSMOD, REG_INT_SOURCE };
    for (size_t i = 0; i < sizeof(src); i++) {
        uint8_t dummy;
        ESP_RETURN_ON_ERROR(read_reg(src[i], &dummy), TAG, "clear int");
    }

    ESP_LOGI(TAG, "FIFO on: %u Hz, +/-8g, watermark %u, INT1 GPIO%d",
             (unsigned)rate_hz, watermark, CONFIG_WF_IMU_INT1_GPIO);
    return ESP_OK;
}
