#pragma once
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

// Use SparkFun MAX3010x (MAX30101/30102 family)
#include <SparkFun_MAX3010x.h>  // provides MAX30105 class

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
  MAX30105 sensor_;     // SparkFun class
  bool ready_{false};

  // user-configurable (from YAML)
  float ir_current_ma_{7.6f};
  float red_current_ma_{7.6f};
  int sample_rate_hz_{100};
  int pulse_width_us_{411};

  // outputs (placeholder until you add an algorithm)
  float hr_{NAN};
  float spo2_{NAN};

  sensor::Sensor *hr_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

  void apply_sensor_config_();
  uint8_t map_current_ma_to_reg_(float ma) const;  // 0..50 mA -> 0..255 approx
  uint8_t map_sr_hz_to_enum_(int hz) const;        // SparkFun enum mapping
  uint8_t map_pw_us_to_enum_(int us) const;        // SparkFun enum mapping
};

}  // namespace max30101
}  // namespace esphome
