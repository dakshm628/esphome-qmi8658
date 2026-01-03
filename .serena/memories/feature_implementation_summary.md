# QMI8658C Feature Implementation Summary

## Date: 2026-01-03 (updated)

## Features Implemented

### 1. Configurable Accelerometer Scale
- Options: 2G, 4G, 8G, 16G (default: 2G)
- YAML: `accel_range: 4G`

### 2. Configurable Gyroscope Scale
- Options: 16DPS to 2048DPS (default: 2048DPS)
- YAML: `gyro_range: 512DPS`

### 3. Configurable Output Data Rate (ODR)
- Options: 31.25HZ to 8000HZ (default: 500HZ)
- Separate for accel and gyro: `accel_odr`, `gyro_odr`

### 4. Pitch/Roll Angle Sensors
- Calculates tilt angles from accelerometer gravity vector
- Unit: degrees
- YAML: `pitch:` and `roll:` sub-configs

### 5. Motion Binary Sensor
- Software-based motion detection relative to gravity
- Configurable threshold (m/s² deviation)
- Platform: `binary_sensor` with `motion:` sub-config

### 6. Orientation Text Sensor
- Reports device orientation:
  - face_up, face_down
  - portrait, portrait_inverted
  - landscape_left, landscape_right
  - unknown
- Platform: `text_sensor` with `orientation:` sub-config

### 7. Wake on Motion (WoM)
- Hardware-based motion detection via interrupt
- Requires interrupt pin configuration
- Configurable threshold (1-255 mg, default 100)
- Configurable interrupt pin (INT1 or INT2)
- Platform: `binary_sensor` with `wom:` sub-config
- **Trade-offs**: Gyroscope disabled and accelerometer in low-power mode per datasheet. Orientation detection and software motion sensor/trigger won't work reliably.
- **Best for**: Low-power/deep-sleep scenarios where IMU interrupt wakes the MCU

### 8. Low-Pass Filter (LPF) Configuration
- Hardware-based noise filtering in CTRL5 register
- Options: DISABLED, 2.66%, 3.63%, 5.39%, 13.37% (% of ODR)
- Default: 2.66% (most aggressive smoothing)
- Separate for accel and gyro: `accel_lpf`, `gyro_lpf`
- At 500Hz ODR with 2.66%, cutoff is ~13Hz

### 9. Automation Triggers
- **on_motion**: Fires automation when motion is detected (leading edge only)
  - Configurable threshold (0.1-10.0 m/s² deviation, default 0.5)
  - Works without binary_sensor platform
  - YAML: `on_motion:` with `threshold:` and `then:` actions
- **on_orientation_change**: Fires automation when orientation changes
  - Passes `orientation` string variable to automation
  - Values: face_up, face_down, portrait, portrait_inverted, landscape_left, landscape_right, unknown
  - Works without text_sensor platform
  - YAML: `on_orientation_change:` with `then:` actions


## Files Modified/Created
- `components/qmi8658/__init__.py` - Updated exports
- `components/qmi8658/sensor.py` - Added configs, pitch/roll
- `components/qmi8658/binary_sensor.py` - NEW: motion, WoM
- `components/qmi8658/text_sensor.py` - NEW: orientation
- `components/qmi8658/qmi8658.h` - Added enums, classes
- `components/qmi8658/qmi8658.cpp` - Implemented features
- `example.yaml` - Updated with new features
- `.gitignore` - Added secrets.yaml, build dirs
