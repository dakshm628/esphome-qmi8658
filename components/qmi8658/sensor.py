import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BRIEFCASE_DOWNLOAD,
    ICON_SCREEN_ROTATION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_DEGREES,
    UNIT_METER_PER_SECOND_SQUARED,
)
from . import qmi8658_ns, QMI8658Component, CONF_QMI8658_ID

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"
CONF_PITCH = "pitch"
CONF_ROLL = "roll"

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)

gyro_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)

temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

angle_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon="mdi:angle-acute",
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_QMI8658_ID): cv.use_id(QMI8658Component),
        cv.Optional(CONF_ACCEL_X): accel_schema,
        cv.Optional(CONF_ACCEL_Y): accel_schema,
        cv.Optional(CONF_ACCEL_Z): accel_schema,
        cv.Optional(CONF_GYRO_X): gyro_schema,
        cv.Optional(CONF_GYRO_Y): gyro_schema,
        cv.Optional(CONF_GYRO_Z): gyro_schema,
        cv.Optional(CONF_TEMPERATURE): temperature_schema,
        cv.Optional(CONF_PITCH): angle_schema,
        cv.Optional(CONF_ROLL): angle_schema,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_QMI8658_ID])

    for axis, conf_key in [
        ("x", CONF_ACCEL_X),
        ("y", CONF_ACCEL_Y),
        ("z", CONF_ACCEL_Z),
    ]:
        if conf_key in config:
            sens = await sensor.new_sensor(config[conf_key])
            cg.add(getattr(parent, f"set_accel_{axis}_sensor")(sens))

    for axis, conf_key in [
        ("x", CONF_GYRO_X),
        ("y", CONF_GYRO_Y),
        ("z", CONF_GYRO_Z),
    ]:
        if conf_key in config:
            sens = await sensor.new_sensor(config[conf_key])
            cg.add(getattr(parent, f"set_gyro_{axis}_sensor")(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(parent.set_temperature_sensor(sens))

    if CONF_PITCH in config:
        sens = await sensor.new_sensor(config[CONF_PITCH])
        cg.add(parent.set_pitch_sensor(sens))

    if CONF_ROLL in config:
        sens = await sensor.new_sensor(config[CONF_ROLL])
        cg.add(parent.set_roll_sensor(sens))
