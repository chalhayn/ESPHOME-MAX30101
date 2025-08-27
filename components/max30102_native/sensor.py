import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ID

from . import MAX30102NativeSensor

# Make sure ESPHome loads i2c and sensor first
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor"]

CONF_HEART_RATE = "heart_rate"
CONF_SPO2 = "spo2"

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(MAX30102NativeSensor),

        cv.Optional(CONF_HEART_RATE): sensor.sensor_schema(
            unit_of_measurement="bpm",
            accuracy_decimals=0,
            icon="mdi:heart-pulse",
        ),

        cv.Optional(CONF_SPO2): sensor.sensor_schema(
            unit_of_measurement="%",
            accuracy_decimals=1,
            icon="mdi:percent",
        ),
    })
    .extend(i2c.i2c_device_schema(0x57))
    .extend(cv.polling_component_schema("1s"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_HEART_RATE in config:
        hr = await sensor.new_sensor(config[CONF_HEART_RATE])
        cg.add(var.set_heart_rate_sensor(hr))

    if CONF_SPO2 in config:
        s2 = await sensor.new_sensor(config[CONF_SPO2])
        cg.add(var.set_spo2_sensor(s2))
