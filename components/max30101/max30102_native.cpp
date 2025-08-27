#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

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
  if (!this->write_byte(REG_MODE_CONFIG, 0x40)) {
    return false;
  }
  delay(100);

  // Configure FIFO: Sample averaging = 4, FIFO rollover = false, FIFO almost full = 17
  if (!this->write_byte(REG_FIFO_CONFIG, 0x50)) {
    return false;
  }

  // Mode Configuration: SpO2 mode
  if (!this->write_byte(REG_MODE_CONFIG, 0x03)) {
    return false;
  }

  // SpO2 Configuration: ADC range = 4096nA, sample rate = 100 Hz, LED pulseWidth = 411uS
  if (!this->write_byte(REG_SPO2_CONFIG, 0x27)) {
    return false;
  }

  // LED Pulse Amplitude Configuration
  if (!this->write_byte(REG_LED1_PA, 0x24)) {  // Red LED
    return false;
  }
  if (!this->write_byte(REG_LED2_PA, 0x24)) {  // IR LED
    return false;
  }

  // Clear FIFO
  this->clear_fifo();
  
  return true;
}

void MAX30102NativeSensor::clear_fifo() {
  this->write_byte(REG_FIFO_WR_PTR, 0);
  this->write_byte(REG_OVF_COUNTER, 0);
  this->write_byte(REG_FIFO_RD_PTR, 0);
}

uint32_t MAX30102NativeSensor::get_ir() {
  uint8_t data[3];
  if (!this->read_bytes(REG_FIFO_DATA, data, 3)) {
    return 0;
  }
  return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
}

uint32_t MAX30102NativeSensor::get_red() {
  uint8_t data[3];
  if (!this->read_bytes(REG_FIFO_DATA, data, 3)) {
    return 0;
  }
  return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
}

bool MAX30102NativeSensor::check_for_beat(int32_t sample) {
  bool beat_detected = false;

  // Store sample in circular buffer
  this->an_x_[this->n_buffer_count_ % MA4_SIZE] = sample;

  // Calculate moving average
  int32_t n_ma = 0;
  for (uint8_t i = 0; i < MA4_SIZE; i++) {
    n_ma += this->an_x_[i];
  }
  n_ma /= MA4_SIZE;

  // Store moving average
  this->an_y_[this->n_buffer_count_ % MA4_SIZE] = n_ma;

  // Simple beat detection
  if (this->n_buffer_count_ >= MA4_SIZE) {
    int32_t n_th1 = n_ma + (this->ir_ac_max_ - this->ir_ac_min_) / 8;
    
    if (this->an_y_[(this->n_buffer_count_ - 1) % MA4_SIZE] < n_th1 && 
        this->an_y_[this->n_buffer_count_ % MA4_SIZE] >= n_th1) {
      beat_detected = true;
    }
    
    // Update AC signal bounds
    if (n_ma > this->ir_ac_max_) {
      this->ir_ac_max_ = n_ma;
    }
    if (n_ma < this->ir_ac_min_) {
      this->ir_ac_min_ = n_ma;
    }
    
    // Decay the bounds
    this->ir_ac_max_ = this->ir_ac_max_ - (this->ir_ac_max_ / 16);
    this->ir_ac_min_ = this->ir_ac_min_ + (this->ir_ac_min_ / 16);
  }

  this->n_buffer_count_++;
  return beat_detected;
}

void MAX30102NativeSensor::update() {
  uint32_t ir_value = this->get_ir();
  uint32_t red_value = this->get_red();

  if (ir_value > 50000) {  // Valid finger detection
    if (this->check_for_beat(ir_value)) {
      uint32_t current_time = millis();
      uint32_t delta = current_time - this->last_beat_;
      this->last_beat_ = current_time;

      if (delta > 300 && delta < 3000) {  // Valid heart rate range (20-200 BPM)
        int32_t bpm = 60000 / delta;
        
        // Store in circular buffer for averaging
        this->rates_[this->rate_array_ % RATE_SIZE] = bpm;
        this->rate_array_++;

        // Calculate average
        int32_t total = 0;
        for (uint8_t i = 0; i < RATE_SIZE; i++) {
          total += this->rates_[i];
        }
        int32_t avg_bpm = total / RATE_SIZE;

        if (this->heart_rate_sensor_ != nullptr && avg_bpm > 40 && avg_bpm < 200) {
          this->heart_rate_sensor_->publish_state(avg_bpm);
          ESP_LOGD(TAG, "Heart Rate: %ld BPM", avg_bpm);
        }
      }
    }

    // Simple SpO2 calculation
    if (this->spo2_sensor_ != nullptr && red_value > 50000) {
      float ratio = (float)red_value / (float)ir_value;
      float spo2 = 104 - 17 * ratio;  // Simplified linear approximation
      
      if (spo2 > 70 && spo2 < 100) {
        this->spo2_sensor_->publish_state(spo2);
        ESP_LOGD(TAG, "SpO2: %.1f%%, Ratio: %.3f", spo2, ratio);
      }
    }
  } else {
    ESP_LOGD(TAG, "No finger detected (IR: %ld)", ir_value);
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
