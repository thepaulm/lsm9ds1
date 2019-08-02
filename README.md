# lsm9ds1
## Introduction
This is a device driver that enables a single board computer (Raspberry Pi, Jetson Nano, etc) to access an LSM9DS1 IMU sensor.
The LSM9DS1 combines a 3D accelerometer, 3D rate gyroscope and a 3D magnetometer. It's useful in robotics and other applications for
measuring a robot's attitude and movement. The LSM9DS1 is made by ST Microelectronics. The datasheet is available from
[their website](https://www.st.com/en/mems-and-sensors/lsm9ds1.html).


This is based largely off the hard work of Richard Gemmell (from over here: https://github.com/Richard-Gemmell/lsm9ds1-rjg). This version
has simply added the scaled versions for the request functions (so you can get accel in g, mag heading in degrees, temp in f, etc), and
collapsed it into a single file.

## Driver Features
* Connects with I2C or SPI (although the SPI is largely untested in this fork)
* Preconfigured for 230 Hz output data rate
* Converts accelerometer and gyro output to use the same axes as the magnetometer by flipping the direction of the x axis.
* Detects data ready in software or with an optional GPIO hardware interrupt.

## Limitations
* Not a complete implementation of the sensor's API.

## Data Ready
If you don't check for data ready and just read data then you will miss some samples.
Worse than that, in some cases you'll read part of one sample and part of the next.
This provides plausible looking but incorrect data. You _must_ wait for data ready
before reading the sensor values. See the examples.
