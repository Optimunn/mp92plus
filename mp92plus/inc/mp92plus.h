#ifndef MP92PLUS_H_
#define MP92PLUS_H_

#include "pico/stdlib.h"
#include <errno.h>

/**Use magnetometer or not. If true, magnetometer is used.*/
#define MP92_USE_COMPASS true
/**Use SPI or I2C. If true, SPI is used.*/
#define MP92_USE_SPI true

#ifndef MP92_USE_COMPASS
#define MP92_USE_COMPASS false
#endif

#ifndef MP92_USE_SPI
#define MP92_USE_SPI false
#endif

#if MP92_USE_SPI
#include "hardware/spi.h"
#else
#include "hardware/i2c.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**DLPF filter.*/
    typedef enum
    {
        DLPF_184HZ = 0x01,
        DLPF_92HZ = 0x02,
        DLPF_41HZ = 0x03,
        DLPF_20HZ = 0x04,
        DLPF_10HZ = 0x05,
        DLPF_5HZ = 0x06
    } DLPFfilter;

    /**Accel range.*/
    typedef enum
    {
        ACCEL_RANGE_2G = 0x00,
        ACCEL_RANGE_4G = 0x08,
        ACCEL_RANGE_8G = 0x10,
        ACCEL_RANGE_16G = 0x18
    } AccelRange;

    /**Gyro range.*/
    typedef enum
    {
        GYRO_RANGE_250DPS = 0x00,
        GYRO_RANGE_500DPS = 0x08,
        GYRO_RANGE_1000DPS = 0x10,
        GYRO_RANGE_2000DPS = 0x18
    } GyroRange;

    /**The desired settings on the MPU-9250.*/
    typedef struct mpu_settings_s
    {
        /** Sets the accelerometer range:
        2G, 4G, 8G, 16G.*/
        AccelRange accel_range;
        /** Sets the gyroscope range:
        250DPS, 500DPS, 1000DPS, 2000DPS.*/
        GyroRange gyro_range;
        /** Sets the digital low pass filter (DLPF):
        5Hz, 10Hz, 20Hz, 41Hz, 92Hz, 184Hz.*/
        DLPFfilter dlpf_filter;
        /** Sample rate divider:
        Data update rate (in Hz) = 1000 / (1 + sample_rate_divider)*/
        uint8_t sample_rate_divider;

    } mpu_settings_t;

#if MP92_USE_SPI
    /**The SPI pin configuration.*/
    typedef struct spi_pin_s
    {
        uint8_t miso;
        uint8_t mosi;
        uint8_t sck;
        uint8_t cs;
    } spi_pin_t;
#endif

    /**The MPU9250 configuration.*/
    typedef struct mpu9250_s
    {
#if MP92_USE_SPI
        uint cs;
        spi_inst_t *inst;
#else
        uint8_t addr;
        i2c_inst_t *inst;
#endif
        float factors[2];
#if MP92_USE_COMPASS
        float asa_values[3];
#endif
        uint8_t interrupt;
    } mpu9250_t;

// --------------------- Global function  ---------------------
#if MP92_USE_SPI
    /**
     * @brief Initialize the MPU9250 sensor.
     *
     * This function performs the following steps:
     *   1. Initializes the mpu9250 sensor.
     *   2. Sets accel range to 16G.
     *   3. Sets gyro range to 2000DPS.
     *   4. Sets DLPF filter to 184Hz.
     *   5. Sets sample rate divider to 9 (update frequency 100 Hz).
     *
     * @param config The configuration struct for the MPU9250 sensor.
     * @param spi_pins The SPI pin configuration.
     * @param inst The SPI instance.
     * @param frequency The SPI frequency in Hz.
     *
     * @return 0 if successful, 1 motion sensor failed, 2 magnetometer failed.
     *
     * @note Function execution time ≈ 40 ms
     */
    error_t mpu9250_reset(mpu9250_t *config, spi_pin_t *spi_pins, spi_inst_t *inst, uint32_t frequency);
#else
    /**
     * @brief Initialize the MPU9250 sensor.
     *
     * This function performs the following steps:
     *   1. Initializes the mpu9250 sensor.
     *   2. Sets accel range to 16G.
     *   3. Sets gyro range to 2000DPS.
     *   4. Sets DLPF filter to 184Hz.
     *   5. Sets sample rate divider to 9 (update frequency 100 Hz).
     *
     * @param config The configuration struct for the MPU9250 sensor.
     * @param sda The number of sda pin.
     * @param scl The number of scl pin.
     * @param addr The I2C device address.
     * @param inst The I2C instance.
     * @param frequency The I2C frequency in Hz.
     *
     * @return 0 if successful, 1 motion sensor failed, 2 magnetometer failed.
     *
     * @note Function execution time ≈ 40 ms
     */
    error_t mpu9250_reset(mpu9250_t *config, uint sda, uint scl, uint8_t addr, i2c_inst_t *inst, uint32_t frequency);
#endif

    /**
     * @brief Configures the MPU-9250 sensor.
     *
     * This function sets the desired settings on the MPU-9250. The settings
     * include the accelerometer range, gyroscope range, DLPF filter, and internal frequency divider.
     * The gyro zero values are also set.
     *
     * @param config The configuration struct for the MPU9250 sensor.
     * @param settings The desired settings
     * @param gyro_zero The gyro calibration values
     *
     * @return 0 if successful, 1 motion sensor failed.
     *
     * @note Function execution time ≈ 15 ms
     * 
     * @note To set the values `gyro_zero` correctly, first set {0,0,0}, then call 
     * function `mpu9250_read_raw_motion()` then, during the subsequent build, enter
     * the resulting values in the order {x,y,z}.
     */
    error_t mpu9250_setup(mpu9250_t *config, mpu_settings_t *settings, int16_t gyro_zero[3]);

    /**
     * @brief Attach an interrupt callback to the interrupt pin of the MPU-9250 sensor.
     *
     * This function will attach an interrupt callback to the interrupt pin of the MPU-9250.
     * The callback will be called when the interrupt pin is triggered.
     *
     * @param config The configuration struct for the MPU9250 sensor.
     * @param interrupt_pin The GPIO pin of the interrupt pin.
     * @param callback The callback function.
     *
     * @note Function execution time ≈ 2 ms
     */
    void mpu9250_attach_interrupt(mpu9250_t *config, uint interrupt_pin, gpio_irq_callback_t callback);

    /**
     * @brief Disables and deinitializes the interrupt for the MPU9250 sensor.
     *
     * This function disables the interrupt and deinitializes
     * the GPIO pin associated with the interrupt.
     *
     * @param config The configuration struct for the MPU9250 sensor.
     *
     * @return 0 if successful, 1 if the interrupt is already not connected.
     */
    error_t mpu9250_terminate_interrupt(mpu9250_t *config);

    /**
     * @brief Checks and returns the interrupt status of the MPU9250 sensor,
     *  activates the ability to receive the next interrupt.
     *
     * This function reads the INT_STATUS register of the MPU9250 to obtain the
     * interrupt status.
     *
     * @param config The configuration struct for the MPU9250 sensor.
     *
     * @return The interrupt status byte from the INT_STATUS register.
     */
    uint8_t mpu9250_continue_interrupt(mpu9250_t *config);

#if MP92_USE_COMPASS
    /**
     * @brief Read the magnetometer data from the MPU-9250 sensor.
     *
     * The magnetometer data is read from the MPU-9250 and scaled using the
     * sensitivity values stored in the config.
     *
     * @param config The configuration struct for the MPU9250 sensor.
     * @param mag The magnetometer data in uT.
     */
    void mpu9250_read_magnetometer(mpu9250_t *config, float mag[3]);
#endif

    /**
     * @brief Read the motion sensor data from the MPU-9250 sensor.
     *
     * This function reads the raw data from the motion sensor and scales it
     * using the sensitivity values stored in the config.
     *
     * @param config The configuration struct for the MPU9250 sensor.
     * @param accel The acceleration data in m/s^2.
     * @param gyro The gyroscope data in deg/s.
     */
    void mpu9250_read_motion(mpu9250_t *config, float accel[3], float gyro[3]);

    /**
     * @brief Reads the temperature from the MPU-9250 sensor.
     *
     * This function reads the raw temperature data from the sensor,
     * converts it to degrees Celsius, and stores it in the provided
     * temperature variable.
     *
     * @param config The configuration struct for the MPU9250 sensor.
     * @param temp Variable where the temperature in degrees Celsius will be stored.
     */
    void mpu9250_read_temperature(mpu9250_t *config, float *temp);

    /** Check the status of the MPU-9250 sensor.
     * @return 0 if successful, 1 if the sensor error.
    */
    error_t mpu9250_status(mpu9250_t *config);

#if MP92_USE_COMPASS
    /** Read raw magnetometer data from the MPU-9250 sensor.*/
    void mpu9250_read_raw_mag(mpu9250_t *config, int16_t mag[3]);
#endif
    /** Read raw motion sensor data from the MPU-9250 sensor.*/
    void mpu9250_read_raw_motion(mpu9250_t *config, int16_t accel[3], int16_t gyro[3]);

// --------------------- Internal function  ---------------------
#if MP92_USE_SPI
    static inline void cs_select(uint pin_cs);
    static inline void cs_deselect(uint pin_cs);
    static inline void  this_interface_init(spi_pin_t *spi_pins, spi_inst_t *inst, uint32_t frequency);
#else
    static inline void this_interface_init(uint sda, uint scl, uint8_t addr, i2c_inst_t *i2c, uint32_t frequency);
#endif
    static inline error_t mpu_init_default(mpu9250_t *config);
    //----------------------------------------------------------------
    static void mpu_write_reg(mpu9250_t *config, uint8_t addr, uint8_t reg);
    static void mpu_read_reg(mpu9250_t *config, uint8_t addr, uint8_t *buf, uint8_t len);
#if MP92_USE_COMPASS
    static void mag_write_reg(mpu9250_t *config, uint8_t addr, uint8_t reg);
    static void mag_read_reg(mpu9250_t *config, uint8_t addr, uint8_t *buf, uint8_t len);
    static void mag_enable_reg(mpu9250_t *config, uint8_t addr, uint8_t len);
    static inline void mag_get_asa_values(mpu9250_t *config);
#endif
    //----------------------------------------------------------------
    static uint8_t mpu6500_status(mpu9250_t *config);
#if MP92_USE_COMPASS
    static uint8_t ak8963_status(mpu9250_t *config);
#endif

#ifdef __cplusplus
}
#endif

#endif
