
#pragma once
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

// Vendored Arduino-style library bundled in third_party/
#include "components/max30101//MAX30105.h" 
max30105 pox_;   // member exists again
namespace esphome {
namespace max30101 {

class Max30101Component : public PollingComponent, public i2c::I2CDevice {
 public:
  Max30101Component() = default;

  void set_ir_current_ma(float i) { ir_current_ma_ = i; }
  void set_red_current_ma(float i) { red_current_ma_ = i; }
  void set_sample_rate_hz(int hz) { sample_rate_hz_ = hz; }
  void set_pulse_width_us(int pw) { pulse_width_us_ = pw; }
  void set_hr_sensor(sensor::Sensor *s) { hr_sensor_ = s; }
  void set_spo2_sensor(sensor::Sensor *s) { spo2_sensor_ = s; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void update() override;

 protected:
  PulseOximeter30101 pox_;
  bool ready_{false};

  float ir_current_ma_{7.6f};
  float red_current_ma_{7.6f};
  int sample_rate_hz_{100};
  int pulse_width_us_{411};

  sensor::Sensor *hr_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

  void apply_sensor_config_();
};

}  // namespace max30101
}  // namespace esphome
