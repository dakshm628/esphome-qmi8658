# Code Style and Conventions

## C++ Style (qmi8658.h, qmi8658.cpp)

### Naming Conventions
- **Classes**: PascalCase (e.g., `QMI8658Component`)
- **Methods**: snake_case (e.g., `set_accel_x_sensor`, `dump_config`)
- **Constants**: ALL_CAPS with prefix (e.g., `QMI8658_REG_WHO_AM_I`)
- **Member variables**: snake_case with trailing underscore (e.g., `accel_x_sensor_`)
- **Local variables**: snake_case (e.g., `raw_temp`, `accel_x`)

### Code Organization
- Use `#pragma once` for header guards
- Organize code within `esphome::qmi8658` namespace
- Use static const for register addresses and constants
- Use enum class for typed constants (AccelScale, GyroScale)

### Documentation
- Use `///` for Doxygen-style documentation comments
- Document register purposes and scale factors

### ESPHome Patterns
- Inherit from `PollingComponent` for periodic updates
- Inherit from `i2c::I2CDevice` for I2C communication
- Use `ESP_LOG*` macros for logging (LOGI, LOGD, LOGW, LOGE, LOGV)
- Use `mark_failed()` for initialization failures
- Use `status_set_warning()` and `status_clear_warning()` for runtime issues

## Python Style (sensor.py)

### Naming Conventions
- **Constants**: ALL_CAPS (e.g., `CONF_ACCEL_X`, `CODEOWNERS`)
- **Variables**: snake_case (e.g., `accel_schema`, `gyro_schema`)

### Code Organization
- Import esphome modules at top
- Define configuration constants
- Create namespace and component class references
- Define sensor schemas with appropriate units and icons
- Build CONFIG_SCHEMA using cv.Schema
- Implement async `to_code()` function for code generation

### ESPHome Patterns
- Use `cv.Optional()` for optional configuration
- Use `sensor.sensor_schema()` to define sensor configurations
- Use `extend()` to add polling_component_schema and i2c_device_schema
- Use `cg.add()` to generate C++ code calls
