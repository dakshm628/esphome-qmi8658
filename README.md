# ESPHome QMI8658C IMU Component

An ESPHome external component for the QMI8658C 6-axis Inertial Measurement Unit (IMU) sensor.

## Features

- 3-axis accelerometer with configurable range (±2g/±4g/±8g/±16g)
- 3-axis gyroscope with configurable range (±16 to ±2048 °/s)
- Configurable output data rate (31.25 Hz to 8000 Hz)
- On-chip temperature sensor
- Pitch and roll angle calculation
- Motion detection binary sensor
- Device orientation text sensor
- Hardware Wake on Motion (WoM) with interrupt support
- I2C interface support
- ESP-IDF framework compatible

## Sensor Specifications

| Specification | Value |
|---------------|-------|
| Manufacturer | QST Corporation |
| Interface | I2C (address: 0x6B) |
| Accelerometer Range | ±2g (default), ±4g, ±8g, ±16g |
| Gyroscope Range | ±2048 °/s (default), ±16 to ±1024 °/s |
| Output Data Rate | 31.25 Hz to 8000 Hz (default: 500 Hz) |
| Package | 2.5 x 3.0 x 0.86 mm LGA-14 |
| Temperature Range | -40°C to +85°C |

## Installation

### Using as a Local Component

1. Copy the `components/qmi8658` folder to your ESPHome configuration directory.

2. Add the external component reference to your YAML:

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [qmi8658]
```

### Using from GitHub

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Djelibeybi/esphome-qmi8658
      ref: main
    components: [qmi8658]
```

## Configuration

### I2C Bus

First, configure the I2C bus:

```yaml
i2c:
  sda: GPIO8
  scl: GPIO9
  scan: true
```

### Component Configuration

The QMI8658 component must be configured first, then sensor, binary_sensor, and text_sensor platforms reference it:

```yaml
# Create the main QMI8658 component
qmi8658:
  id: imu_sensor              # Required for platform references
  address: 0x6B               # Optional, 0x6B is default
  update_interval: 100ms      # Optional, 60s is default

  # Range and ODR configuration (all optional)
  accel_range: 2G             # Options: 2G, 4G, 8G, 16G
  gyro_range: 2048DPS         # Options: 16DPS, 32DPS, 64DPS, 128DPS, 256DPS, 512DPS, 1024DPS, 2048DPS
  accel_odr: 500HZ            # Options: 8000HZ, 4000HZ, 2000HZ, 1000HZ, 500HZ, 250HZ, 125HZ, 62.5HZ, 31.25HZ
  gyro_odr: 500HZ             # Options: 8000HZ, 4000HZ, 2000HZ, 1000HZ, 500HZ, 250HZ, 125HZ, 62.5HZ, 31.25HZ
```

### Sensor Platform Configuration

```yaml
sensor:
  - platform: qmi8658
    qmi8658_id: imu_sensor    # Reference to the component

    # Accelerometer outputs (m/s²)
    accel_x:
      name: "Accelerometer X"
    accel_y:
      name: "Accelerometer Y"
    accel_z:
      name: "Accelerometer Z"

    # Gyroscope outputs (°/s)
    gyro_x:
      name: "Gyroscope X"
    gyro_y:
      name: "Gyroscope Y"
    gyro_z:
      name: "Gyroscope Z"

    # Temperature (°C)
    temperature:
      name: "IMU Temperature"

    # Tilt angles (degrees)
    pitch:
      name: "Pitch"
    roll:
      name: "Roll"
```

### Component Configuration Variables

#### General Options

- **id** (*Required*, string): Component ID. Required for platform references.
- **address** (*Optional*, int): The I2C address of the sensor. Default is `0x6B`.
- **update_interval** (*Optional*, time): The interval to check the sensor. Default is `60s`.

#### Range and ODR Configuration

- **accel_range** (*Optional*): Accelerometer full-scale range. Default is `2G`.
  - `2G`: ±2g (highest sensitivity)
  - `4G`: ±4g
  - `8G`: ±8g
  - `16G`: ±16g (lowest sensitivity, highest range)

- **gyro_range** (*Optional*): Gyroscope full-scale range. Default is `2048DPS`.
  - `16DPS`: ±16 °/s (highest sensitivity)
  - `32DPS`: ±32 °/s
  - `64DPS`: ±64 °/s
  - `128DPS`: ±128 °/s
  - `256DPS`: ±256 °/s
  - `512DPS`: ±512 °/s
  - `1024DPS`: ±1024 °/s
  - `2048DPS`: ±2048 °/s (lowest sensitivity, highest range)

- **accel_odr** (*Optional*): Accelerometer output data rate. Default is `500HZ`.
- **gyro_odr** (*Optional*): Gyroscope output data rate. Default is `500HZ`.
  - Options: `8000HZ`, `4000HZ`, `2000HZ`, `1000HZ`, `500HZ`, `250HZ`, `125HZ`, `62.5HZ`, `31.25HZ`

### Sensor Platform Configuration Variables

- **qmi8658_id** (*Required*): Reference to the component ID.

#### Sensor Outputs

- **accel_x** (*Optional*): Acceleration on the X-axis in m/s².
- **accel_y** (*Optional*): Acceleration on the Y-axis in m/s².
- **accel_z** (*Optional*): Acceleration on the Z-axis in m/s².
- **gyro_x** (*Optional*): Angular velocity on the X-axis in °/s.
- **gyro_y** (*Optional*): Angular velocity on the Y-axis in °/s.
- **gyro_z** (*Optional*): Angular velocity on the Z-axis in °/s.
- **temperature** (*Optional*): Internal temperature sensor in °C.
- **pitch** (*Optional*): Pitch angle (rotation around Y-axis) in degrees.
- **roll** (*Optional*): Roll angle (rotation around X-axis) in degrees.

All sensor outputs accept the standard [Sensor](https://esphome.io/components/sensor/#config-sensor) configuration options.

### Binary Sensor Platform Configuration

The binary sensor platform provides motion detection capabilities.

```yaml
binary_sensor:
  - platform: qmi8658
    qmi8658_id: imu_sensor      # Reference to the component

    # Software-based motion detection
    motion:
      name: "Motion Detected"
      threshold: 0.5            # Optional, deviation from gravity in m/s² (default: 0.5)

    # Hardware Wake on Motion (requires interrupt pin)
    wom:
      name: "Wake on Motion"
      threshold: 100            # Optional, threshold in mg (1-255, default: 100)
      pin: GPIO4                # Required, interrupt pin from QMI8658C INT1 or INT2
```

#### Binary Sensor Configuration Variables

- **qmi8658_id** (*Required*): Reference to the component ID.

##### Motion Detection (Software)

- **motion** (*Optional*): Software-based motion detection.
  - **threshold** (*Optional*, float): Motion detection threshold as deviation from gravity in m/s². Default is `0.5`. Higher values are less sensitive.

##### Wake on Motion (Hardware)

- **wom** (*Optional*): Hardware-based Wake on Motion using the QMI8658C's built-in motion detection.
  - **threshold** (*Optional*, int): WoM threshold in milligrams (1-255). Default is `100`.
  - **pin** (*Required*): GPIO pin connected to QMI8658C INT1 or INT2 output.

### Text Sensor Platform Configuration

The text sensor platform provides device orientation detection.

```yaml
text_sensor:
  - platform: qmi8658
    qmi8658_id: imu_sensor      # Reference to the component

    orientation:
      name: "Device Orientation"
```

#### Text Sensor Configuration Variables

- **qmi8658_id** (*Required*): Reference to the component ID.

##### Orientation Detection

- **orientation** (*Optional*): Reports device orientation based on gravity vector.
  - Possible values:
    - `face_up`: Device is flat, screen facing up
    - `face_down`: Device is flat, screen facing down
    - `portrait`: Device is upright
    - `portrait_inverted`: Device is upside down
    - `landscape_left`: Device is tilted left
    - `landscape_right`: Device is tilted right
    - `unknown`: Device is at an intermediate angle

## Complete Example

```yaml
# Full example with all features

# Create the main component
qmi8658:
  id: imu_sensor
  address: 0x6B
  update_interval: 100ms
  accel_range: 4G
  gyro_range: 512DPS
  accel_odr: 500HZ
  gyro_odr: 500HZ

# Sensor platform
sensor:
  - platform: qmi8658
    qmi8658_id: imu_sensor
    accel_x:
      name: "Accelerometer X"
    accel_y:
      name: "Accelerometer Y"
    accel_z:
      name: "Accelerometer Z"
    gyro_x:
      name: "Gyroscope X"
    gyro_y:
      name: "Gyroscope Y"
    gyro_z:
      name: "Gyroscope Z"
    temperature:
      name: "IMU Temperature"
    pitch:
      name: "Pitch"
    roll:
      name: "Roll"

# Binary sensor platform
binary_sensor:
  - platform: qmi8658
    qmi8658_id: imu_sensor
    motion:
      name: "Motion Detected"
      threshold: 0.5

# Text sensor platform
text_sensor:
  - platform: qmi8658
    qmi8658_id: imu_sensor
    orientation:
      name: "Device Orientation"
```

## Example Output

When the device is flat and stationary (face up):
- Accelerometer X and Y: ~0 m/s²
- Accelerometer Z: ~9.8 m/s² (gravity)
- Gyroscope X, Y, Z: ~0 °/s
- Pitch: ~0°
- Roll: ~0°
- Orientation: "face_up"
- Motion: OFF

## Wiring

### Basic Wiring (I2C only)

| QMI8658C Pin | ESP32-S3 Pin |
|--------------|--------------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO8 (or your configured SDA) |
| SCL | GPIO9 (or your configured SCL) |

### With Wake on Motion Interrupt

| QMI8658C Pin | ESP32-S3 Pin |
|--------------|--------------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |
| INT1 | GPIO4 (or any available GPIO) |

## Troubleshooting

### Sensor not detected

1. Check I2C wiring connections
2. Verify the I2C address (default is 0x6B)
3. Enable I2C bus scanning in your configuration:
   ```yaml
   i2c:
     scan: true
   ```
4. Check the logs for I2C errors

### Invalid readings

1. Ensure the sensor is properly powered (3.3V)
2. Verify the I2C pull-up resistors are present (many breakout boards include these)
3. Try reducing the I2C frequency if experiencing communication errors

### Motion detection too sensitive/insensitive

1. Adjust the `threshold` value for the motion binary sensor
2. Lower values are more sensitive (detect smaller movements)
3. Higher values require more significant motion to trigger

### Orientation not updating

1. Ensure the device experiences sufficient tilt (~45°) to trigger orientation change
2. The sensor uses a threshold of approximately 7 m/s² (~45° tilt) to determine orientation

## License

Universal Permissive License v1.0 (UPL-1.0)

## References

- [QMI8658C Datasheet](https://files.waveshare.com/upload/3/36/QMI8658C_datasheet_rev_0.8.pdf)
- [ESPHome Documentation](https://esphome.io/)
- [ESPHome External Components](https://esphome.io/components/external_components/)
- [ESPHome Sensor Component](https://esphome.io/components/sensor/)
- [ESPHome Binary Sensor Component](https://esphome.io/components/binary_sensor/)
- [ESPHome Text Sensor Component](https://esphome.io/components/text_sensor/)
