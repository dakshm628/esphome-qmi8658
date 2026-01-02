import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from . import qmi8658_ns, QMI8658Component

CONF_QMI8658_ID = "qmi8658_id"
CONF_ORIENTATION = "orientation"

QMI8658OrientationTextSensor = qmi8658_ns.class_(
    "QMI8658OrientationTextSensor", text_sensor.TextSensor, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_QMI8658_ID): cv.use_id(QMI8658Component),
        cv.Optional(CONF_ORIENTATION): text_sensor.text_sensor_schema(
            QMI8658OrientationTextSensor,
            icon="mdi:screen-rotation",
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_QMI8658_ID])

    if CONF_ORIENTATION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_ORIENTATION])
        await cg.register_component(sens, config[CONF_ORIENTATION])
        cg.add(sens.set_parent(parent))
