/*
* Example for the mp92plus library (MPU6500)
* 
* This example shows how to get acceleration,
* gyroscope, and temperature data from the MPU6050 using interrupts.
*
* The sensor uses the i2c interface in this example.
*
* Don't forget to set 
* #define MP92_USE_COMPASS false 
* #define MP92_USE_SPI false
* in file mp92plus.h
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "mp92plus.h"

mpu9250_t mpu92;

float accel[3], gyro[3], temp;

void gpio_callback(uint gpio, uint32_t events)
{
    mpu9250_read_motion(&mpu92, accel, gyro);
    mpu9250_read_temperature(&mpu92, &temp);
    printf("Acceleration in G     X = %10.4f,  Y = %10.4f,  Z = %10.4f\n", accel[0], accel[1], accel[2]);
    printf("Gyroscope in Deg/s    X = %10.4f,  Y = %10.4f,  Z = %10.4f\n", gyro[0], gyro[1], gyro[2]);
    printf("Temperature in C      %10.4f\n\n", temp);
    mpu9250_continue_interrupt(&mpu92);
}

#define SDA_PIN 4
#define SCL_PIN 5
#define INTERRUPT_PIN 6
#define DEVICE_ADDRESS 0x68

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    uint8_t activate = mpu9250_reset(&mpu92, SDA_PIN, SCL_PIN, DEVICE_ADDRESS, i2c0, 100 * 1000);
    if(activate)
        printf("Failed to initialize MPU9250!\n");

    mpu_settings_t set = {
        .accel_range = ACCEL_RANGE_16G,
        .gyro_range = GYRO_RANGE_2000DPS,
        .dlpf_filter = DLPF_41HZ,
        .sample_rate_divider = 249};
    int16_t correct_gyro[] = {0, 0, 0};
    uint8_t settings = mpu9250_setup(&mpu92, &set, correct_gyro);
    if(settings)
        printf("Failed to setup MPU9250!\n");

    mpu9250_attach_interrupt(&mpu92, INTERRUPT_PIN, gpio_callback);

    while (1)
    {
        /* Other code */
        sleep_ms(1000);
    } 
}