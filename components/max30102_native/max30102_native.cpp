#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <heartRate.h>  // from SparkFun MAX3010x library

namespace esphome {
namespace max30102_native {

static const char *const TAG = "max30102_native";

void MAX30102NativeSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX30102 Native...");

  // Check if sensor is present
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
  // Reset sensor
  if (!this->write_byte(REG_MODE_CONFIG, 0x40)) return false;
  delay(100);

  // FIFO: avg=4, rollover=0, almost_full=0x10 (0x50)
  if (!this->write_byte(REG_FIFO_CONFIG, 0x50)) return false;

  // SpO2 mode
  if (!this->write_byte(REG_MODE_CONFIG, 0x03)) return false;

  // SpO2 config: ADC=4096nA, SR=100Hz, PW=411us (0x27)
  if (!this->write_byte(REG_SPO2_CONFIG, 0x27)) return false;

  // LED amplitudes (mid-level; tune if needed)
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

bool MAX30102NativeSensor::check_for_beat(int32_t) {
  // UNUSED now; we use SparkFun's checkForBeat() instead.
  return false;
}

void MAX30102NativeSensor::update() {
  // Determine how many 3-byte samples are waiting in FIFO
  uint8_t rd = 0, wr = 0;
  if (!this->read_byte(REG_FIFO_RD_PTR, &rd) || !this->read_byte(REG_FIFO_WR_PTR, &wr)) {
    ESP_LOGD(TAG, "FIFO pointer read failed");
    return;
  }
  uint8_t samples = (wr - rd) & 0x1F;  // 32-deep FIFO, modulo 32
  if (samples < 2) samples = 2;        // process at least one RED+IR pair

  // Process pairs (RED+IR). Each pair consumes 2 samples (6 bytes).
  for (uint8_t i = 0; i + 1 < samples; i += 2) {
    uint8_t data[6];
    if (!this->read_bytes(REG_FIFO_DATA, data, 6)) {
      ESP_LOGD(TAG, "FIFO read failed");
      break;
    }

    // In SpO2 mode, order is RED then IR, each 18-bit
    uint32_t red_value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    uint32_t ir_value  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];

    const uint32_t FINGER_IR_MIN  = 20000;
    const uint32_t FINGER_RED_MIN = 10000;

    // ---- Heart Rate via SparkFun detector on IR channel ----
    if (ir_value > FINGER_IR_MIN) {
      if (checkForBeat(ir_value)) {  // from heartRate.h
        uint32_t now = millis();
        if (this->last_beat_ != 0) {
          uint32_t dt = now - this->last_beat_;
          if (dt > 300 && dt < 3000) { // 20–200 BPM
            int32_t bpm = 60000 / (int32_t) dt;

            // Average a few beats for stability
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

      // ---- SpO2 (simple DC-level ratio) ----
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
