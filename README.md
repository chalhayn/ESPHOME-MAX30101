
# ESPHome External Component: MAX30101 (ESP32-C3 compatible)

Owner: **@Chalhayn**

Minimal ESPHome external component to expose **Heart Rate** and **SpO₂** from a MAX30101 sensor over I²C.

> The bundled `third_party/arduino_max30101` is a **stub** to allow clean compilation.
> Replace with an actual Arduino MAX30101 library for real measurements.

## Folder layout
```
esphome/components/max30101/
  ├─ __init__.py
  ├─ sensor.py
  ├─ max30101_component.h
  ├─ max30101_component.cpp
  └─ third_party/arduino_max30101/
     ├─ MAX30101_PulseOximeter.h
     └─ MAX30101_PulseOximeter.cpp
```

## Example ESPHome YAML
```yaml
esphome:
  name: spo2_node_30101

esp32:
  board: esp32-c3-devkitm-1
  framework:
    type: arduino

logger:
api:
ota:
improv_serial:

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

i2c:
  sda: GPIO7
  scl: GPIO6
  frequency: 400kHz

external_components:
  - source:
      type: git
      url: https://github.com/Chalhayn/esphome-max30101
    components: [max30101]

sensor:
  - platform: max30101
    address: 0x57
    ir_led_current_ma: 7.6
    red_led_current_ma: 7.6
    sample_rate_hz: 100
    pulse_width_us: 411
    heart_rate:
      name: "Heart Rate"
    spo2:
      name: "SpO2"
    update_interval: 100ms
```
