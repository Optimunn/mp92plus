#ifndef REGMAP_H_
#define REGMAP_H_

#include "pico/stdlib.h"

#define READ_CONST (uint8_t)0x80

// Mpu6500 registers
#define ACCEL_OUT (uint8_t)0x3B
#define TEMP_OUT (uint8_t)0x41
#define GYRO_OUT (uint8_t)0x43
#define EXT_SENS_DATA_00 (uint8_t)0x49
#define CONFIG (uint8_t)0x1A
//-------------------------------------------
#define ACCEL_CONFIG (uint8_t)0x1C
#define ACCEL_CONFIG2 (uint8_t)0x1D
#define ACCEL_FS_SEL_2G (uint8_t)0x00
#define ACCEL_FS_SEL_4G (uint8_t)0x08
#define ACCEL_FS_SEL_8G (uint8_t)0x10
#define ACCEL_FS_SEL_16G (uint8_t)0x18
//--------------------------------------------
#define GYRO_CONFIG (uint8_t)0x1B
#define GYRO_FS_SEL_250DPS (uint8_t)0x00
#define GYRO_FS_SEL_500DPS (uint8_t)0x08
#define GYRO_FS_SEL_1000DPS (uint8_t)0x10
#define GYRO_FS_SEL_2000DPS (uint8_t)0x18
//---------------------------------------------
#define DLPF_184 (uint8_t)0x01
#define DLPF_92 (uint8_t)0x02
#define DLPF_41 (uint8_t)0x03
#define DLPF_20 (uint8_t)0x04
#define DLPF_10 (uint8_t)0x05
#define DLPF_5 (uint8_t)0x06
//---------------------------------------------
#define XG_OFFSET_H (uint8_t)0x13
#define XA_OFFSET_H (uint8_t)0x77
#define SMPDIV (uint8_t)0x19
#define INT_PIN_CFG (uint8_t)0x37
#define OPEN_DRAIN (uint8_t)0x40
#define LATCH_INT_EN (uint8_t) 0x20
#define ACTL (uint8_t)0x80
#define BYPASS_EN (uint8_t)0x02
#define INT_ENABLE (uint8_t)0x38
#define INT_DISABLE (uint8_t)0x00
#define INT_STATUS (uint8_t)0x3A
#define WAIT_FOR_ES (uint8_t)0x40
#define INT_RAW_RDY_EN (uint8_t)0x01
#define INT_RAW_DATA_RDY_INT (uint8_t)0x01
#define PWR_MGMNT_1 (uint8_t)0x6B
#define PWR_CYCLE (uint8_t)0x20
#define PWR_RESET (uint8_t)0x80
#define CLOCK_SEL_PLL (uint8_t)0x01
#define PWR_MGMNT_2 (uint8_t)0x6C
#define SEN_ENABLE (uint8_t)0x00
#define DIS_GYRO (uint8_t)0x07
#define USER_CTRL (uint8_t)0x6A
#define I2C_MST_EN (uint8_t)0x20
#define I2C_MST_CLK (uint8_t)0x0D
#define I2C_MST_CTRL (uint8_t)0x24
#define I2C_SLV0_ADDR (uint8_t)0x25
#define I2C_SLV0_REG (uint8_t)0x26
#define I2C_SLV0_DO (uint8_t)0x63
#define I2C_SLV0_CTRL (uint8_t)0x27
#define I2C_SLV0_EN (uint8_t)0x80
#define I2C_READ_FLAG (uint8_t)0x80
#define WHO_AM_I (uint8_t)0x75

// AK8963 registers
#define AK8963_I2C_ADDR (uint8_t)0x0C
#define AK8963_HXL (uint8_t)0x03
#define AK8963_CNTL1 (uint8_t)0x0A
#define AK8963_PWR_DOWN (uint8_t)0x00
#define AK8963_CNT_MEAS1 (uint8_t)0x12
#define AK8963_CNT_MEAS2 (uint8_t)0x16
#define AK8963_FUSE_ROM (uint8_t)0x0F
#define AK8963_CNTL2 (uint8_t)0x0B
#define AK8963_RESET (uint8_t)0x01
#define AK8963_ASAX (uint8_t)0x10
#define AK8963_ASAY (uint8_t)0x11
#define AK8963_ASAZ (uint8_t)0x12
#define AK8963_16_BIT (uint8_t)0x10
#define AK8963_WHO_AM_I (uint8_t)0x00

#endif