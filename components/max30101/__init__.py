import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

DEPENDENCIES = ['i2c']

max30102_ns = cg.esphome_ns.namespace('max30102')
MAX30102Component = max30102_ns.class_('MAX30102Component', cg.Component, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MAX30102Component),
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x57))

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)
