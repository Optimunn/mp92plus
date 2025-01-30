/*
* Example for the mp92plus library (MPU9250)
* 
* This example shows how to get acceleration,
* gyroscope, and compass data from the MPU9250.
*
* The sensor uses the spi interface in this example.
*
* Don't forget to set 
* #define MP92_USE_COMPASS true 
* #define MP92_USE_SPI true
* in file mp92plus.h
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "mp92plus.h"

mpu9250_t mpu92;

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    spi_pin_t my_spi = {
        .miso = 12,
        .mosi = 15,
        .sck = 14,
        .cs = 13,
    };
    uint8_t activate = mpu9250_reset(&mpu92, &my_spi, spi1, 1000 * 1000);
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

    float accel[3], gyro[3], magnet[3], temp;

    while (1)
    {
        mpu9250_read_motion(&mpu92, accel, gyro);
        mpu9250_read_magnetometer(&mpu92, magnet);
        mpu9250_read_temperature(&mpu92, &temp);
        printf("Acceleration in G     X = %10.4f,  Y = %10.4f,  Z = %10.4f\n", accel[0], accel[1], accel[2]);
        printf("Gyroscope in Deg/s    X = %10.4f,  Y = %10.4f,  Z = %10.4f\n", gyro[0], gyro[1], gyro[2]);
        printf("Magnetometer in uT    X = %10.4f,  Y = %10.4f,  Z = %10.4f\n", magnet[0], magnet[1], magnet[2]);
        printf("Temperature in C      %10.4f\n\n", temp);
        sleep_ms(2000);
    } 
}