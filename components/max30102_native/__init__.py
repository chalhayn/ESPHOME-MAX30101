# __init__.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c

DEPENDENCIES = ["i2c"]
CODEOWNERS = ["@yourusername"]

max30102_native_ns = cg.esphome_ns.namespace("max30102_native")
MAX30102NativeSensor = max30102_native_ns.class_("MAX30102NativeSensor", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(): cv.declare_id(MAX30102NativeSensor),
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x57))
)
