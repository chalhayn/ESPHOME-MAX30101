#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

// Keep C-style algo headers OUTSIDE any namespace,
// so their symbols aren't pulled into esphome::... namespaces.
#include <heartRate.h>           // SparkFun MAX3010x beat detector
#include <spo2_algorithm.h>      // Maxim/ProtoCentral SpO2 algorithm
#include <cstring>               // std::memmove

namespace esphome {
namespace max30102_native {

static const char *const TAG = "max30102_native";

void MAX30102NativeSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX30102 Native...");

  uint8_t part_id = 0;
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

// Legacy helpers (not used by update() anymore; kept for compatibility)
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
  // Unused; we leverage SparkFun's checkForBeat() in update()
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

  // Process in RED+IR pairs (6 bytes per pair)
  for (uint8_t i = 0; i + 1 < samples; i += 2) {
    uint8_t data[6];
    if (!this->read_bytes(REG_FIFO_DATA, data, 6)) {
      ESP_LOGD(TAG, "FIFO read failed");
      break;
    }

    // Unpack 18-bit values
    uint32_t red_u = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    uint32_t ir_u  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];

    // Gentle finger-present gates (tune for your device/contact)
    const uint32_t FINGER_IR_MIN  = 20000;
    const uint32_t FINGER_RED_MIN = 10000;
    if (ir_u < FINGER_IR_MIN || red_u < FINGER_RED_MIN) continue;

    // ---- Heart Rate via SparkFun detector on IR channel ----
    if (checkForBeat((int32_t) ir_u)) {
      uint32_t now = millis();
      if (this->last_beat_ != 0) {
        uint32_t dt = now - this->last_beat_;  // inter-beat interval (ms)
        if (dt > 300 && dt < 3000) {           // 20–200 BPM
          // Store IBI (ms) into ring buffer; we'll compute BPM from mean IBI for steadiness
          this->rates_[this->rate_array_ % RATE_SIZE] = (int32_t) dt;
          this->rate_array_++;

          uint32_t sum_dt = 0;
          uint8_t count = 0;
          for (uint8_t k = 0; k < RATE_SIZE; k++) {
            int32_t ibi = this->rates_[k];
            if (ibi > 0) { sum_dt += (uint32_t) ibi; count++; }
          }
          if (count >= 2) {
            float avg_dt = (float) sum_dt / (float) count;   // ms
            float bpm = 60000.0f / avg_dt;                   // bpm from mean IBI
            if (this->heart_rate_sensor_ && bpm > 40.0f && bpm < 200.0f) {
              this->heart_rate_sensor_->publish_state(bpm);
              ESP_LOGD(TAG, "HR (IBI) avg_dt=%.1fms -> bpm=%.1f (n=%u)", avg_dt, bpm, count);
            }
          }
        }
      }
      this->last_beat_ = now;
    }

    // ---- SpO2 via spo2_algorithm.h ----
    // Decimate 100 Hz -> 25 Hz (push every 4th sample to the algorithm buffer)
    if (++this->decim_count_ >= SPO2_DECIM) {
      this->decim_count_ = 0;

      if (this->spo2_count_ < SPO2_WIN) {
        this->ir_buf_[this->spo2_count_]  = ir_u;   // uint32_t buffers
        this->red_buf_[this->spo2_count_] = red_u;
        this->spo2_count_++;
      }

      if (this->spo2_count_ >= SPO2_WIN) {
        int32_t spo2 = 0, hr_algo = 0;
        int8_t  spo2_valid = 0, hr_valid = 0;

        // The reference algorithm expects ~25 Hz and BUFFER_SIZE=100
        maxim_heart_rate_and_oxygen_saturation(
          this->ir_buf_, (int32_t) SPO2_WIN,
          this->red_buf_,
          &spo2, &spo2_valid,
          &hr_algo, &hr_valid
        );

        // Publish SpO2 if valid
        if (this->spo2_sensor_ && spo2_valid && spo2 > 70 && spo2 <= 100) {
          this->spo2_sensor_->publish_state((float) spo2);
          ESP_LOGD(TAG, "SpO2 algo: %ld%% (valid=%d)", (long) spo2, (int) spo2_valid);
        }

        // Optionally publish HR from the algorithm too (often steadier)
        if (this->heart_rate_sensor_ && hr_valid && hr_algo > 40 && hr_algo < 200) {
          this->heart_rate_sensor_->publish_state((float) hr_algo);
          ESP_LOGD(TAG, "HR algo: %ld bpm (valid=%d)", (long) hr_algo, (int) hr_valid);
        }

        // Slide the window by half (50 samples) to keep outputs responsive
        const uint16_t half = SPO2_WIN / 2;  // 50
        std::memmove(this->ir_buf_,  this->ir_buf_  + half, half * sizeof(uint32_t));
        std::memmove(this->red_buf_, this->red_buf_ + half, half * sizeof(uint32_t));
        this->spo2_count_ = half;
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
