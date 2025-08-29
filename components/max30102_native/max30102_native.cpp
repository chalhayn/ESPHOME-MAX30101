#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <heartRate.h>  // SparkFun MAX3010x library

namespace esphome {
namespace max30102_native {

static const char *const TAG = "max30102_native";

void MAX30102NativeSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX30102 Native...");

  uint8_t part_id;
  if (!this->read_byte(REG_PART_ID, &part_id)) {
    ESP_LOGE(TAG, "Failed to read part ID");
    this->mark_failed();
    return;
  }

  if (part_id != MAX30102_EXPECTEDPARTID) {
    ESP_LOGE(TAG, "MAX30102 not found. Expected: 0x%02X, Got: 0x%02X", MAX30102_EXPECTEDPARTID, part_id);
    this->mark_failed();
    return;
  }

  if (!this->initialize_sensor()) {
    ESP_LOGE(TAG, "Failed to initialize MAX30102");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "MAX30102 Native initialized successfully");
}

bool MAX30102NativeSensor::initialize_sensor() {
  // Reset
  if (!this->write_byte(REG_MODE_CONFIG, 0x40)) return false;
  delay(100);

  // FIFO: sample avg=4 (bits 7:5 = 0b010), rollover=0, almost_full=0x10 -> 0x50
  if (!this->write_byte(REG_FIFO_CONFIG, 0x50)) return false;

  // SpO2 mode
  if (!this->write_byte(REG_MODE_CONFIG, 0x03)) return false;

  // SpO2 config: ADC range=4096 nA, sample rate=100 Hz, pulse width=411 us -> 0x27
  if (!this->write_byte(REG_SPO2_CONFIG, 0x27)) return false;

  // LED currents (tune as needed)
  if (!this->write_byte(REG_LED1_PA, 0x24)) return false;  // RED
  if (!this->write_byte(REG_LED2_PA, 0x24)) return false;  // IR

  this->clear_fifo();
  return true;
}

void MAX30102NativeSensor::clear_fifo() {
  this->write_byte(REG_FIFO_WR_PTR, 0);
  this->write_byte(REG_OVF_COUNTER, 0);
  this->write_byte(REG_FIFO_RD_PTR, 0);
}

// Legacy helpers (not used in update anymore)
uint32_t MAX30102NativeSensor::get_ir() {
  uint8_t data[3];
  if (!this->read_bytes(REG_FIFO_DATA, data, 3)) return 0;
  return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
}

uint32_t MAX30102NativeSensor::get_red() {
  uint8_t data[3];
  if (!this->read_bytes(REG_FIFO_DATA, data, 3)) return 0;
  return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
}

bool MAX30102NativeSensor::check_for_beat(int32_t) {
  // Unused; we use SparkFun's checkForBeat()
  return false;
}

void MAX30102NativeSensor::update() {
  // How many 3-byte samples are waiting?
  uint8_t rd = 0, wr = 0;
  if (!this->read_byte(REG_FIFO_RD_PTR, &rd) || !this->read_byte(REG_FIFO_WR_PTR, &wr)) {
    ESP_LOGD(TAG, "FIFO pointer read failed");
    return;
  }
  uint8_t samples = (wr - rd) & 0x1F;  // 32-deep FIFO, modulo 32
  if (samples < 2) samples = 2;        // ensure at least one RED+IR pair

  // Process in pairs (RED, IR) = 6 bytes per pair
  for (uint8_t i = 0; i + 1 < samples; i += 2) {
    uint8_t data[6];
    if (!this->read_bytes(REG_FIFO_DATA, data, 6)) {
      ESP_LOGD(TAG, "FIFO read failed");
      break;
    }

    uint32_t red_value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    uint32_t ir_value  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];

    const uint32_t FINGER_IR_MIN  = 20000;
    const uint32_t FINGER_RED_MIN = 10000;

    // ---- HR on IR using SparkFun peak detector ----
    if (ir_value > FINGER_IR_MIN) {
      if (checkForBeat((int32_t) ir_value)) {
        uint32_t now = millis();
        if (this->last_beat_ != 0) {
          uint32_t dt = now - this->last_beat_;  // inter-beat interval (ms)
          if (dt > 300 && dt < 3000) {           // 20–200 BPM
            // Keep a short history of IBIs (ms)
            this->rates_[this->rate_array_ % RATE_SIZE] = (int32_t) dt;
            this->rate_array_++;

            // Average non-zero IBIs → steadier BPM
            uint32_t sum_dt = 0;
            uint8_t count = 0;
            for (uint8_t k = 0; k < RATE_SIZE; k++) {
              int32_t ibi = this->rates_[k];
              if (ibi > 0) { sum_dt += (uint32_t) ibi; count++; }
            }
            if (count >= 2) {
              float avg_dt = (float) sum_dt / (float) count;
              float bpm = 60000.0f / avg_dt;
              if (this->heart_rate_sensor_ != nullptr && bpm > 40.0f && bpm < 200.0f) {
                this->heart_rate_sensor_->publish_state(bpm);
                ESP_LOGD(TAG, "HR dt=%lums avg_dt=%.1fms -> bpm=%.1f (n=%u)",
                         (unsigned long) dt, avg_dt, bpm, count);
              }
            }
          }
        }
        this->last_beat_ = now;
      }

      // ---- SpO2 (simple DC ratio) ----
      if (this->spo2_sensor_ != nullptr && red_value > FINGER_RED_MIN) {
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
}

void MAX30102NativeSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX30102 Native:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX30102 failed!");
  }
  LOG_SENSOR("  ", "Heart Rate", this->heart_rate_sensor_);
  LOG_SENSOR("  ", "SpO2", this->spo2_sensor_);
}

}  // namespace max30102_native
}  // namespace esphome
