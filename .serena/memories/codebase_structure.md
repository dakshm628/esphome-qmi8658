# Codebase Structure

```
esphome-qmi8658/
├── components/
│   └── qmi8658/                    # ESPHome component directory
│       ├── __init__.py             # Required empty file for ESPHome component recognition
│       ├── sensor.py               # Python config validation and code generation
│       ├── qmi8658.h               # C++ header with register definitions and class declaration
│       └── qmi8658.cpp             # C++ implementation of sensor driver
├── example.yaml                     # Example ESPHome configuration
├── README.md                        # Project documentation
├── LICENSE                          # UPL-1.0 license
└── .gitignore                       # Git ignore rules
```

## Key Files

### sensor.py
- Defines configuration schema for the sensor
- Validates user configuration
- Generates C++ code for ESPHome compilation
- Specifies dependencies (i2c) and codeowner

### qmi8658.h
- Contains register address constants
- Defines AccelScale and GyroScale enums
- Declares QMI8658Component class inheriting from PollingComponent and I2CDevice
- Provides sensor setter methods

### qmi8658.cpp
- Implements setup() for sensor initialization (reset, ID verification, configuration)
- Implements update() for reading and publishing sensor data
- Implements dump_config() for logging configuration
- Handles burst reading of temperature, accelerometer, and gyroscope data
