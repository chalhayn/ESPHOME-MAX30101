import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, 
    STATE_CLASS_MEASUREMENT,
    UNIT_BEATS_PER_MINUTE,
    UNIT_PERCENT,
    ICON_HEART_PULSE,
    ICON_PERCENT
)

from . import max30102_ns, MAX30102Component

CONF_HEART_RATE = 'heart_rate'
CONF_SPO2 = 'spo2'

MAX30102Sensor = max30102_ns.class_('MAX30102Sensor', sensor.Sensor, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MAX30102Component),
    cv.Optional(CONF_HEART_RATE): sensor.sensor_schema(
        unit_of_measurement=UNIT_BEATS_PER_MINUTE,
        icon=ICON_HEART_PULSE,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_SPO2): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        icon=ICON_PERCENT,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}).extend(cv.COMPONENT_SCHEMA).extend(cv.polling_component_schema('1s'))

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)

    if CONF_HEART_RATE in config:
        sens = yield sensor.new_sensor(config[CONF_HEART_RATE])
        cg.add(var.set_heart_rate_sensor(sens))

    if CONF_SPO2 in config:
        sens = yield sensor.new_sensor(config[CONF_SPO2])
        cg.add(var.set_spo2_sensor(sens))
