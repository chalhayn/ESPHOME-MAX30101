#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace max30102_native {

// MAX30102 Register addresses
static const uint8_t MAX30102_ADDRESS = 0x57;

// Status Registers
static const uint8_t REG_INTR_STATUS_1 = 0x00;
static const uint8_t REG_INTR_STATUS_2 = 0x01;
static const uint8_t REG_INTR_ENABLE_1 = 0x02;
static const uint8_t REG_INTR_ENABLE_2 = 0x03;

// FIFO Registers
static const uint8_t REG_FIFO_WR_PTR = 0x04;
static const uint8_t REG_OVF_COUNTER = 0x05;
static const uint8_t REG_FIFO_RD_PTR = 0x06;
static const uint8_t REG_FIFO_DATA = 0x07;

// Configuration Registers
static const uint8_t REG_FIFO_CONFIG = 0x08;
static const uint8_t REG_MODE_CONFIG = 0x09;
static const uint8_t REG_SPO2_CONFIG = 0x0A;
static const uint8_t REG_LED1_PA = 0x0C;
static const uint8_t REG_LED2_PA = 0x0D;

// Die Temperature Registers
static const uint8_t REG_TEMP_INTR = 0x1F;
static const uint8_t REG_TEMP_FRAC = 0x20;
static const uint8_t REG_TEMP_CONFIG = 0x21;

// Part ID Registers
static const uint8_t REG_REV_ID = 0xFE;
static const uint8_t REG_PART_ID = 0xFF;

// Expected values
static const uint8_t MAX30102_EXPECTEDPARTID = 0x15;

class MAX30102NativeSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_heart_rate_sensor(sensor::Sensor *heart_rate_sensor) { heart_rate_sensor_ = heart_rate_sensor; }
  void set_spo2_sensor(sensor::Sensor *spo2_sensor) { spo2_sensor_ = spo2_sensor; }

 protected:
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

  // Heart rate calculation variables
  static const uint8_t RATE_SIZE = 4;
  int32_t rates_[RATE_SIZE];
  uint8_t rate_array_ = 0;
  uint32_t last_beat_ = 0;
  
  // Internal methods
  bool initialize_sensor();
  uint32_t get_ir();
  uint32_t get_red();
  bool check_for_beat(int32_t sample);
  void clear_fifo();
  
  // Beat detection variables
  int32_t ir_ac_max_ = 20;
  int32_t ir_ac_min_ = -20;
  
  // Simple moving average for beat detection
  static const uint8_t MA4_SIZE = 4;
  int32_t an_x_[MA4_SIZE];
  int32_t an_y_[MA4_SIZE];
  uint8_t n_buffer_count_ = 0;
};

}  // namespace max30102_native
}  // namespace esphome
