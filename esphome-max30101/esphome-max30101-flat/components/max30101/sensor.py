
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID, CONF_ADDRESS, UNIT_PERCENT, ICON_PULSE, UNIT_BEATS_PER_MINUTE
)

CODEOWNERS = ["@Chalhayn"]
DEPENDENCIES = ["i2c"]

max30101_ns = cg.esphome_ns.namespace("max30101")
Max30101Component = max30101_ns.class_("Max30101Component", cg.PollingComponent, i2c.I2CDevice)

CONF_HEART_RATE = "heart_rate"
CONF_SPO2 = "spo2"
CONF_IR_CURRENT = "ir_led_current_ma"
CONF_RED_CURRENT = "red_led_current_ma"
CONF_SAMPLE_RATE = "sample_rate_hz"
CONF_PULSE_WIDTH_US = "pulse_width_us"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Max30101Component),
    cv.Optional(CONF_ADDRESS, default=0x57): cv.i2c_address,
    cv.Optional(CONF_IR_CURRENT, default=7.6): cv.float_range(min=0.2, max=50.0),
    cv.Optional(CONF_RED_CURRENT, default=7.6): cv.float_range(min=0.2, max=50.0),
    cv.Optional(CONF_SAMPLE_RATE, default=100): cv.int_range(min=50, max=400),
    cv.Optional(CONF_PULSE_WIDTH_US, default=411): cv.one_of(118, 215, 411, 822, int=True),
    cv.Required(CONF_HEART_RATE): sensor.sensor_schema(
        unit_of_measurement=UNIT_BEATS_PER_MINUTE,
        icon=ICON_PULSE,
        accuracy_decimals=0
    ),
    cv.Required(CONF_SPO2): sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        icon="mdi:water-percent",
        accuracy_decimals=1
    ),
}).extend(cv.polling_component_schema("100ms")).extend(i2c.i2c_device_schema(0x57))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_ir_current_ma(config[CONF_IR_CURRENT]))
    cg.add(var.set_red_current_ma(config[CONF_RED_CURRENT]))
    cg.add(var.set_sample_rate_hz(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_pulse_width_us(config[CONF_PULSE_WIDTH_US]))

    hr = await sensor.new_sensor(config[CONF_HEART_RATE])
    spo2 = await sensor.new_sensor(config[CONF_SPO2])
    cg.add(var.set_hr_sensor(hr))
    cg.add(var.set_spo2_sensor(spo2))
