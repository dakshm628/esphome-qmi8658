# ESPHome QMI8658C IMU Component

## Purpose
An ESPHome external component for the QMI8658C 6-axis Inertial Measurement Unit (IMU) sensor by QST Corporation. This component allows integration of the QMI8658C sensor with ESPHome-based devices.

## Features
- 3-axis accelerometer (±2g/±4g/±8g/±16g range options)
- 3-axis gyroscope (±16 to ±2048 °/s range options)
- On-chip temperature sensor
- I2C interface support (default address: 0x6B)
- ESP-IDF framework compatible

## Tech Stack
- **Platform**: ESPHome external component
- **Languages**: 
  - C++ for sensor driver implementation (qmi8658.h, qmi8658.cpp)
  - Python for ESPHome configuration validation and code generation (sensor.py)
- **Framework**: ESP-IDF (via ESPHome)
- **Communication**: I2C protocol
- **Target Hardware**: ESP32 (specifically ESP32-S3 in examples)

## License
Universal Permissive License v1.0 (UPL-1.0)

## Codeowner
@Djelibeybi (Avi Miller)
