#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include <MAX30105.h>  // SparkFun MAX3010x library

namespace esphome {
namespace max30101 {

class Max30101Component : public PollingComponent {
 public:
  // Constructor: poll every 200ms by default
  Max30101Component() : PollingComponent(200) {}

  // User-facing sensors (optional – comment out any you don’t use)
  sensor::Sensor *red_sensor{nullptr};
  sensor::Sensor *ir_sensor{nullptr};
  sensor::Sensor *green_sensor{nullptr};
  sensor::Sensor *die_temp_sensor{nullptr};

  // Config setters (you can also wire these from YAML)
  void set_address(uint8_t addr) { address_ = addr; }
  void set_led_mode(uint8_t mode) { led_mode_ = mode; }                 // 1=Red, 2=Red+IR, 3=Red+IR+Green
  void set_sample_average(uint8_t avg) { sample_average_ = avg; }       // 1,2,4,8,16,32
  void set_sample_rate(int rate_hz) { sample_rate_hz_ = rate_hz; }      // 50..3200 (see SparkFun docs)
  void set_pulse_width_us(int pw_us) { pulse_width_us_ = pw_us; }       // 69,118,215,411
  void set_adc_range(int rng) { adc_range_ = rng; }                     // 2048,4096,8192,16384
  void set_brightness(uint8_t b) { led_brightness_ = b; }               // 0x00..0xFF

  void setup() override;
  void update() override;

 protected:
  void apply_sensor_config_();

  MAX30105 sensor_;                   // <-- SparkFun device handle (replaces PulseOximeter30101 pox_)
  uint8_t address_{0x57};

  // Defaults are safe; adjust from YAML
  uint8_t led_mode_{2};               // Red+IR
  uint8_t sample_average_{8};         // 8 samples avg
  int sample_rate_hz_{100};           // 100 Hz
  int pulse_width_us_{411};           // 411 us
  int adc_range_{16384};              // 16-bit range
  uint8_t led_brightness_{0x1F};      // modest brightness
};

}  // namespace max30101
}  // namespace esphome
