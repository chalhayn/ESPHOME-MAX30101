#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "MAX30102.h"  // SparkFun library
#include "heartRate.h" // SparkFun library

namespace esphome {
namespace max30102 {

class MAX30102Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_heart_rate_sensor(sensor::Sensor *heart_rate_sensor) { heart_rate_sensor_ = heart_rate_sensor; }
  void set_spo2_sensor(sensor::Sensor *spo2_sensor) { spo2_sensor_ = spo2_sensor; }

 protected:
  MAX30105 particleSensor_;
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};
  
  const byte RATE_SIZE = 4;
  long rates_[4];
  byte rate_array_ = 0;
  long last_beat_ = 0;
  
  bool calculate_heart_rate();
  bool calculate_spo2();
};

}  // namespace max30102
}  // namespace esphome
