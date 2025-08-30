#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace max30102_native {

// MAX30102 register addresses (subset we use)
static const uint8_t REG_FIFO_WR_PTR  = 0x04;
static const uint8_t REG_OVF_COUNTER  = 0x05;
static const uint8_t REG_FIFO_RD_PTR  = 0x06;
static const uint8_t REG_FIFO_DATA    = 0x07;
static const uint8_t REG_FIFO_CONFIG  = 0x08;
static const uint8_t REG_MODE_CONFIG  = 0x09;
static const uint8_t REG_SPO2_CONFIG  = 0x0A;
static const uint8_t REG_LED1_PA      = 0x0C;  // RED
static const uint8_t REG_LED2_PA      = 0x0D;  // IR
static const uint8_t REG_PART_ID      = 0xFF;
static const uint8_t MAX30102_EXPECTEDPARTID = 0x15;

class MAX30102NativeSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  // Component lifecycle
  void setup() override;
  void update() override;
  void dump_config() override;

  // Internal helpers
  bool initialize_sensor();
  void clear_fifo();

  // Legacy helpers (kept for compatibility)
  uint32_t get_ir();
  uint32_t get_red();
  bool check_for_beat(int32_t sample);

  // Setters wired from codegen
  void set_heart_rate_sensor(sensor::Sensor *s) { heart_rate_sensor_ = s; }
  void set_spo2_sensor(sensor::Sensor *s)      { spo2_sensor_       = s; }

 protected:
  // Output sensors
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

// SpO2 algorithm buffers (expects ~25 Hz and ~100-sample window)
static const uint16_t SPO2_WIN = 100;
static const uint8_t  SPO2_DECIM = 4;

uint32_t ir_buf_[SPO2_WIN]  = {0};   // <— uint32_t
uint32_t red_buf_[SPO2_WIN] = {0};   // <— uint32_t
uint16_t spo2_count_ = 0;
uint8_t  decim_count_ = 0;


  // Beat timing / smoothing state
  static const uint8_t RATE_SIZE = 8;
  int32_t  rates_[RATE_SIZE] = {0};  // here used to store recent inter-beat intervals (ms)
  uint32_t rate_array_{0};
  uint32_t last_beat_{0};
};

}  // namespace max30102_native
}  // namespace esphome
