#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <heartRate.h>  // from SparkFun MAX3010x library

namespace esphome {
namespace max30102_native {

// ... keep your existing setup(), initialize_sensor(), clear_fifo(), etc ...

// Helper: read one RED+IR pair (6 bytes) atomically
static inline bool read_red_ir_pair(i2c::I2CDevice *dev, uint32_t &red, uint32_t &ir) {
  uint8_t data[6];
  if (!dev->read_bytes(REG_FIFO_DATA, data, 6)) return false;
  red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
  ir  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
  return true;
}

void MAX30102NativeSensor::update() {
  // How many 3-byte samples are waiting? (RED and IR alternate in SpO2 mode)
  uint8_t rd = 0, wr = 0;
  if (!this->read_byte(REG_FIFO_RD_PTR, &rd) || !this->read_byte(REG_FIFO_WR_PTR, &wr)) {
    ESP_LOGD(TAG, "FIFO pointer read failed");
    return;
  }
  uint8_t samples = (wr - rd) & 0x1F;   // 32-deep FIFO, modulo 32
  if (samples < 2) {
    // Read at least one pair so we keep moving even at low sample rates
    samples = 2;
  }

  // Process pairs (RED+IR). Each pair consumes 2 samples from FIFO.
  for (uint8_t i = 0; i + 1 < samples; i += 2) {
    uint32_t red_value = 0, ir_value = 0;
    if (!read_red_ir_pair(this, red_value, ir_value)) {
      ESP_LOGD(TAG, "FIFO read failed");
      break;
    }

    // Gentle finger-present gates (tune if needed)
    const uint32_t FINGER_IR_MIN  = 20000;
    const uint32_t FINGER_RED_MIN = 10000;

    // ---- Heart Rate via SparkFun detector on IR channel ----
    if (ir_value > FINGER_IR_MIN) {
      if (checkForBeat(ir_value)) {  // from heartRate.h
        uint32_t now = millis();
        if (this->last_beat_ != 0) {
          uint32_t dt = now - this->last_beat_;
          if (dt > 300 && dt < 3000) { // 20–200 BPM window
            int32_t bpm = 60000 / (int32_t) dt;

            // Smooth over a few beats
            this->rates_[this->rate_array_ % RATE_SIZE] = bpm;
            this->rate_array_++;
            int32_t total = 0;
            for (uint8_t k = 0; k < RATE_SIZE; k++) total += this->rates_[k];
            int32_t avg_bpm = total / RATE_SIZE;

            if (this->heart_rate_sensor_ != nullptr && avg_bpm > 40 && avg_bpm < 200) {
              this->heart_rate_sensor_->publish_state(avg_bpm);
              ESP_LOGD(TAG, "Heart Rate: %ld BPM", (long) avg_bpm);
            }
          }
        }
        this->last_beat_ = now;
      }
    }

    // ---- SpO2 (simple DC-level ratio) ----
    if (this->spo2_sensor_ != nullptr && red_value > FINGER_RED_MIN && ir_value > FINGER_IR_MIN) {
      float ratio = (float) red_value / (float) ir_value;  // RED/IR
      if (ratio > 0.1f && ratio < 3.0f) {
        float spo2 = 104.0f - 17.0f * ratio;               // crude linearization
        if (spo2 > 70.0f && spo2 < 100.0f) {
          this->spo2_sensor_->publish_state(spo2);
        }
      }
    }
  }
}

// ... keep your dump_config() ...
}  // namespace max30102_native
}  // namespace esphome
