#include "mp92plus.h"
#include "regmap.h"

#if MP92_USE_SPI
static inline void cs_select(uint pin_cs)
{
    gpio_put(pin_cs, 0);
    asm volatile("nop \n nop");
}

static inline void cs_deselect(uint pin_cs)
{
    gpio_put(pin_cs, 1);
    asm volatile("nop \n nop");
}

static inline void __attribute__((always_inline)) this_interface_init(spi_pin_t *spi_pins, spi_inst_t *inst, uint32_t frequency)
{
    spi_init(inst, frequency);
    gpio_set_function(spi_pins->miso, GPIO_FUNC_SPI);
    gpio_set_function(spi_pins->sck, GPIO_FUNC_SPI);
    gpio_set_function(spi_pins->mosi, GPIO_FUNC_SPI);

    gpio_init(spi_pins->cs);
    gpio_set_dir(spi_pins->cs, GPIO_OUT);
    gpio_put(spi_pins->cs, 1);
}
#else
static inline void __attribute__((always_inline)) this_interface_init(uint sda, uint scl, uint8_t addr, i2c_inst_t *inst, uint32_t frequency)
{
    i2c_init(inst, frequency);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);

    gpio_pull_up(sda);
    gpio_pull_up(scl);
}
#endif

static void mpu_write_reg(mpu9250_t *config, uint8_t addr, uint8_t reg)
{
    uint8_t buf[] = {addr, reg};
#if MP92_USE_SPI
    cs_select(config->cs);
    spi_write_blocking(config->inst, buf, 2);
    cs_deselect(config->cs);
#else
    i2c_write_blocking(config->inst, config->addr, buf, 2, false);
#endif
    busy_wait_us(550);
}

static void mpu_read_reg(mpu9250_t *config, uint8_t addr, uint8_t *buf, uint8_t len)
{
#if MP92_USE_SPI
    addr |= READ_CONST;
    cs_select(config->cs);
    spi_write_blocking(config->inst, &addr, 1);
    spi_read_blocking(config->inst, 0, buf, len);
    cs_deselect(config->cs);
#else
    i2c_write_blocking(config->inst, config->addr, &addr, 1, true);
    i2c_read_blocking(config->inst, config->addr, buf, len, false);
#endif
}

#if MP92_USE_COMPASS
static void mag_write_reg(mpu9250_t *config, uint8_t addr, uint8_t reg)
{
    mpu_write_reg(config, I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu_write_reg(config, I2C_SLV0_REG, addr);
    mpu_write_reg(config, I2C_SLV0_DO, reg);
    mpu_write_reg(config, I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);
}

static void mag_enable_reg(mpu9250_t *config, uint8_t addr, uint8_t len)
{
    mpu_write_reg(config, I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);
    mpu_write_reg(config, I2C_SLV0_REG, addr);
    mpu_write_reg(config, I2C_SLV0_CTRL, I2C_SLV0_EN | len);
    busy_wait_ms(1);
}

static void mag_read_reg(mpu9250_t *config, uint8_t addr, uint8_t *buf, uint8_t len)
{
    mag_enable_reg(config, addr, len);
    mpu_read_reg(config, EXT_SENS_DATA_00, buf, len);
}

static inline __attribute__ ((always_inline)) void mag_get_asa_values(mpu9250_t *config)
{
    uint8_t buf[3];
    mag_read_reg(config, AK8963_ASAX, buf, sizeof(buf));
    for (uint8_t item = 0; item < 3; item++)
        config->asa_values[item] = (0.5 * (buf[item] - 128) / 128.0) + 1.0;
}
#endif

static uint8_t mpu6500_status(mpu9250_t *config)
{
    uint8_t buf;
    mpu_read_reg(config, WHO_AM_I, &buf, 1);
    return (buf == 0x71 || buf == 0x73) ? 0 : 1;
}

#if MP92_USE_COMPASS
static uint8_t ak8963_status(mpu9250_t *config)
{
    uint8_t buf;
    mag_read_reg(config, AK8963_WHO_AM_I, &buf, 1);
    return (buf == 0x48) ? 0 : 1;
}
#endif

#if MP92_USE_SPI
error_t mpu9250_reset(mpu9250_t *config, spi_pin_t *spi_pins, spi_inst_t *inst, uint32_t frequency)
{
    config->cs = spi_pins->cs;
    config->inst = inst;

    this_interface_init(spi_pins, inst, frequency);

    config->interrupt = 0x80;

    return mpu_init_default(config);
}
#else
error_t mpu9250_reset(mpu9250_t *config, uint sda, uint scl, uint8_t addr, i2c_inst_t *inst, uint32_t frequency)
{
    config->addr = addr;
    config->inst = inst;

    this_interface_init(sda, scl, addr, inst, frequency);

    config->interrupt = 0x80;

    return mpu_init_default(config);
}
#endif

static inline __attribute__((always_inline)) error_t mpu_init_default(mpu9250_t *config)
{
    mpu_write_reg(config, PWR_MGMNT_1, CLOCK_SEL_PLL);

    mpu_write_reg(config, PWR_MGMNT_1, PWR_RESET);
    busy_wait_ms(1);

    mpu_write_reg(config, PWR_MGMNT_1, CLOCK_SEL_PLL);

    if (mpu6500_status(config))
        return 1;

    mpu_write_reg(config, PWR_MGMNT_2, SEN_ENABLE);
    mpu_write_reg(config, INT_PIN_CFG, BYPASS_EN);

    // setting accel range to 16G as default
    mpu_write_reg(config, ACCEL_CONFIG, ACCEL_FS_SEL_16G);
    config->factors[0] = 16.0f / 32767.5f;

    // setting the gyro range to 2000DPS as default
    mpu_write_reg(config, GYRO_CONFIG, GYRO_FS_SEL_2000DPS & ~0x03);
    config->factors[1] = 2000.0f / 32767.5f;

    // setting bandwidth to 184Hz as default
    mpu_write_reg(config, CONFIG, DLPF_184);
    mpu_write_reg(config, ACCEL_CONFIG2, DLPF_184 & ~0x08);

#if MP92_USE_COMPASS
    mpu_write_reg(config, SMPDIV, 0x00);

    mpu_write_reg(config, USER_CTRL, I2C_MST_EN);
    mpu_write_reg(config, I2C_MST_CTRL, I2C_MST_CLK);

    mag_write_reg(config, AK8963_CNTL1, AK8963_PWR_DOWN);

    mag_write_reg(config, AK8963_CNTL2, AK8963_RESET);
    busy_wait_ms(1);

    if (ak8963_status(config))
        return 2;

    mag_write_reg(config, AK8963_CNTL1, AK8963_PWR_DOWN);

    mag_write_reg(config, AK8963_CNTL1, AK8963_FUSE_ROM);

    mag_get_asa_values(config);

    mag_write_reg(config, AK8963_CNTL1, AK8963_PWR_DOWN);

    mag_write_reg(config, AK8963_CNTL1, AK8963_CNT_MEAS2);
#endif

    mpu_write_reg(config, PWR_MGMNT_1, CLOCK_SEL_PLL);

    mpu_write_reg(config, SMPDIV, 0x09);

#if MP92_USE_COMPASS
    mag_enable_reg(config, AK8963_HXL, 7);
#endif

    return 0;
}

error_t mpu9250_setup(mpu9250_t *config, mpu_settings_t *settings, int16_t gyro_zero[3])
{
    uint8_t gyro_bytes[6];

    for (int item = 0; item < 3; item++) {
        gyro_bytes[2 * item] = (0 - gyro_zero[item] >> 8) & 0xFF;
        gyro_bytes[2 * item + 1] = 0 - gyro_zero[item] & 0xFF;
    }

    for(uint8_t item = 0; item < 6; item++)
        mpu_write_reg(config, XG_OFFSET_H + item, gyro_bytes[item]);

    switch (settings->accel_range)
    {
    case ACCEL_RANGE_2G:
        config->factors[0] = 2.0f / 32767.5f;
        break;
    case ACCEL_RANGE_4G:
    case ACCEL_RANGE_8G:
        config->factors[0] = settings->accel_range / 2.0f / 32767.5f;
        break;
    case ACCEL_RANGE_16G:
        config->factors[0] = 16.0f / 32767.5f;
        break;
    }
    mpu_write_reg(config, ACCEL_CONFIG, settings->accel_range);

    switch (settings->gyro_range)
    {
    case GYRO_RANGE_250DPS:
        config->factors[1] = 250.0f / 32767.5f;
        break;
    case GYRO_RANGE_500DPS:
    case GYRO_RANGE_1000DPS:
        config->factors[1] = settings->gyro_range * 62.5f / 32767.5f;
        break;
    case GYRO_RANGE_2000DPS:
        config->factors[1] = 2000.0f / 32767.5f;
        break;
    }
    mpu_write_reg(config, GYRO_CONFIG, settings->gyro_range & ~0x03);

    mpu_write_reg(config, CONFIG, settings->dlpf_filter);
    mpu_write_reg(config, ACCEL_CONFIG2, settings->dlpf_filter & ~0x08);

#if MP92_USE_COMPASS
    mpu_write_reg(config, SMPDIV, 0x00);

    mag_write_reg(config, AK8963_CNTL1, AK8963_PWR_DOWN);

    if (settings->sample_rate_divider > 124)
        mag_write_reg(config, AK8963_CNTL1, AK8963_CNT_MEAS1);
    else
        mag_write_reg(config, AK8963_CNTL1, AK8963_CNT_MEAS2);

    if (ak8963_status(config))
        return 2;

    mag_enable_reg(config, AK8963_HXL, 7);
#endif

    mpu_write_reg(config, SMPDIV, settings->sample_rate_divider);

    if (mpu6500_status(config))
        return 1;

    return 0;
}

void mpu9250_attach_interrupt(mpu9250_t *config, uint interrupt_pin, gpio_irq_callback_t callback)
{
    gpio_init(interrupt_pin);
    gpio_set_dir(interrupt_pin, GPIO_IN);
    gpio_pull_up(interrupt_pin);
    config->interrupt |= interrupt_pin;
    config->interrupt &= ~0x80;

    if (!irq_is_enabled(IO_IRQ_BANK0))
    {
        gpio_set_irq_enabled_with_callback(
            interrupt_pin,
            GPIO_IRQ_EDGE_RISE,
            true,
            callback);
    }
    else
    {
        gpio_set_irq_enabled(
            interrupt_pin,
            GPIO_IRQ_EDGE_RISE,
            true);
    }

    uint8_t buffer;
    mpu_read_reg(config, I2C_MST_CTRL, &buffer, 1);
    mpu_write_reg(config, I2C_MST_CTRL, buffer | WAIT_FOR_ES);
    mpu_read_reg(config, INT_PIN_CFG, &buffer, 1);
    mpu_write_reg(config, INT_PIN_CFG, buffer | LATCH_INT_EN | OPEN_DRAIN);
    mpu_write_reg(config, INT_ENABLE, INT_RAW_RDY_EN);
}

error_t mpu9250_terminate_interrupt(mpu9250_t *config)
{
    if (config->interrupt & 0x80)
        return 1;

    mpu_write_reg(config, INT_ENABLE, INT_DISABLE);

    gpio_set_irq_enabled(
        config->interrupt & 0x3F,
        GPIO_IRQ_EDGE_RISE,
        false);
    gpio_deinit(config->interrupt & 0x3F);

    config->interrupt |= 0x80;
    return 0;
}

uint8_t mpu9250_continue_interrupt(mpu9250_t *config)
{
    uint8_t buffer;
    mpu_read_reg(config, INT_STATUS, &buffer, 1);
    return buffer;
}

error_t mpu9250_status(mpu9250_t *config)
{
    return mpu6500_status(config);
}

#if MP92_USE_COMPASS
void mpu9250_read_raw_mag(mpu9250_t *config, int16_t mag[3])
{
    uint8_t mag_data[7];

    mpu_read_reg(config, EXT_SENS_DATA_00, mag_data, sizeof(mag_data));

    mag[0] = mag_data[1] << 8 | mag_data[0];
    mag[1] = mag_data[3] << 8 | mag_data[2];
    mag[2] = mag_data[5] << 8 | mag_data[4];
}

void mpu9250_read_magnetometer(mpu9250_t *config, float mag[3])
{
    int16_t mag_raw[3];

    mpu9250_read_raw_mag(config, mag_raw);

    for (uint8_t item = 0; item < 3; item++)
        mag[item] = (float)mag_raw[item] * config->asa_values[item] * 4912 / 32760;
}
#endif

void mpu9250_read_raw_motion(mpu9250_t *config, int16_t accel[3], int16_t gyro[3])
{
    uint8_t buffer[14];

    mpu_read_reg(config, ACCEL_OUT, buffer, sizeof(buffer));

    accel[0] = buffer[0] << 8 | buffer[1];
    accel[1] = buffer[2] << 8 | buffer[3];
    accel[2] = buffer[4] << 8 | buffer[5];

    gyro[0] = buffer[8] << 8 | buffer[9];
    gyro[1] = buffer[10] << 8 | buffer[11];
    gyro[2] = buffer[12] << 8 | buffer[13];
}

void mpu9250_read_motion(mpu9250_t *config, float accel[3], float gyro[3])
{
    int16_t accel_raw[3], gyro_raw[3];

    mpu9250_read_raw_motion(config, accel_raw, gyro_raw);

    for (uint8_t item = 0; item < 3; item++)
        accel[item] = (float)accel_raw[item] * config->factors[0];

    for (uint8_t item = 0; item < 3; item++)
        gyro[item] = (float)gyro_raw[item] * config->factors[1];
}

void mpu9250_read_raw_temperature(mpu9250_t *config, int16_t *temp)
{
    uint8_t buffer[2];

    mpu_read_reg(config, TEMP_OUT, buffer, sizeof(buffer));

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu9250_read_temperature(mpu9250_t *config, float *temp)
{
    uint8_t buffer[2];

    mpu_read_reg(config, TEMP_OUT, buffer, sizeof(buffer));

    *temp = (float)(buffer[0] << 8 | buffer[1]) / 333.87f + 21.0f;
}
