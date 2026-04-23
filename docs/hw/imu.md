# IMU over I2C Implementation Description
The IMU module provides orientation information for the telescope system using the BNO055 sensor. It is mainly used to measure the current heading of the telescope and support directional alignment during operation. This is especially necessary, as the telescope may be picked up and rotated as a whole system, not just around the provided yaw swivel. The module also supports tare functionality and calibration data storage for more stable long-term use.

## Hardware Interface
The IMU sensor communicates with the STM32 through an I2C interface. The system reads orientation and calibration status registers from the BNO055 and writes configuration or calibration data back when needed.

The IMU is mounted at the bottom of the tripod, kept away from the direct EMI of the components around the telescope tube. It is fit into a custom 3D printed mount, and screwed on.

## Software Architecture
The primary code for the IMU is housed in imu.cpp
* Initializes the BNO055 sensor, sets the operating mode, and manages register-level I2C communication.
* Reads Euler heading data from the sensor and stores the current heading value. Also reads the calibration status to track sensor readiness.
* Allows the current heading to be stored as a reference offset, so future heading readings are reported relative to that direction.
* Supports saving calibration offsets to flash memory and reloading them later, so the sensor does not need full recalibration every time the system starts.
* Provides processed heading and calibration information to the rest of the telescope system.

## IMU Data Processing Pipeline
* Initialize the BNO055 sensor
* Set operating mode and load calibration data if available
* Read heading data from the sensor
* Apply tare offset to compute relative heading
* Read calibration status
* Provide processed heading data to the controller

## Features Implemented
* BNO055 initialization
* Heading update and retrieval
* Tare functionality
* Calibration status reading
* Calibration data save/load through flash memory
* I2C register read/write support

