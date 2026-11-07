/*
 * Thanh ghi MMA8451 dùng chung cho main core (mma8451.c) và ULP (ulp/ulp_main.c).
 */
#pragma once

#define MMA8451_DEVICE_ID           0x1A
#define MMA8451_DEVICE_ID_ALT       0x2A
#define MMA8451_FIFO_SIZE           32
#define MMA8451_BYTES_PER_SAMPLE    6

#define REG_F_STATUS                0x00
#define REG_OUT_X_MSB               0x01
#define REG_F_SETUP                 0x09
#define REG_TRIG_CFG                0x0A
#define REG_SYSMOD                  0x0B
#define REG_INT_SOURCE              0x0C
#define REG_WHO_AM_I                0x0D
#define REG_XYZ_DATA_CFG            0x0E
#define REG_PL_STATUS               0x10
#define REG_PL_CFG                  0x11
#define REG_FF_MT_CFG               0x15
#define REG_FF_MT_SRC               0x16
#define REG_TRANSIENT_CFG           0x1D
#define REG_TRANSIENT_SRC           0x1E
#define REG_PULSE_CFG               0x21
#define REG_PULSE_SRC               0x22
#define REG_ASLP_COUNT              0x29
#define REG_CTRL_REG1               0x2A
#define REG_CTRL_REG2               0x2B
#define REG_CTRL_REG3               0x2C
#define REG_CTRL_REG4               0x2D
#define REG_CTRL_REG5               0x2E

#define CTRL_REG1_ACTIVE            (1 << 0)
#define CTRL_REG1_DR_SHIFT          3
#define CTRL_REG2_MODS_LOW_POWER    0x03
#define CTRL_REG4_INT_EN_FIFO       (1 << 6)
#define CTRL_REG5_INT_CFG_FIFO      (1 << 6)    /* FIFO -> chân INT1 */

#define XYZ_RANGE_8G                2

#define F_SETUP_MODE_CIRCULAR       (1 << 6)
#define F_SETUP_WMRK_MASK           0x3F

#define F_STATUS_F_OVF              (1 << 7)
#define F_STATUS_F_CNT_MASK         0x3F
