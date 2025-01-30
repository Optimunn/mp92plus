# MPU9250 (MPU6500) library
A simple MPU9250 (MPU6500) driver for Raspberry pi pico (RP2040) using I2C or SPI.

## How To Use
In the `mp92plus.h` file, you must set up two definitions:
* `#define MP92_USE_COMPASS`
This definition is responsible for the operation of the accelerometer. If set to `false`, all actions with the magnetometer will be disabled.
(Mandatory `false` for MPU6500)

* `#define MP92_USE_SPI`
This definition is responsible for using the SPI interface. If `true`, then SPI is enabled.

Next, you need to use `mpu9250_reset()` to initialize the device.
The initial setup is completed and the sensor is ready to collect data.
To get the processed data, there are three functions to get the appropriate measurements:
* `mpu9250_read_motion();`
* `mpu9250_read_magnetometer();`
* `mpu9250_read_temperature()`

There are also functions for raw measurements.
* `mpu9250_read_raw_motion()`
* `mpu9250_read_raw_mag()`
- - -
The main functions for sensor operation have been described above. Next, we will consider functions for more specific tasks.

Function `mpu9250_setup()` is called after `mpu9250_reset()` and is necessary for setting and calibrating the sensor.

Functions `mpu9250_attach_interrupt()` and `mpu9250_terminate_interrupt()` are required to connect and disable the interrupt accordingly.

It is also important to note that after an interrupt is triggered, in order for the next interrupt to work, you must call function `mpu9250_continue_interrupt()`, which will allow the next interrupt.

Also, to check the status of the device, there is a function `mpu9250_status()`
**You can learn more about these functions from the library header file.**
### Also see the examples
* `mpu6500_i2c_interrupt`
The program is designed for MPU6500. The sensor operates via the i2c interface. The data is updated on an interrupt.
* `mpu9250_spi_data`
The program is designed for MPU9250. The sensor operates via interface spi. The sensor collects all the data and is updated in a cycle.