
#include "max30101_component.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace max30101 {

static const char *const TAG = "max30101";

void Max30101Component::setup() {
  if (!this->pox_.begin(this->address_)) {
    ESP_LOGE(TAG, "MAX30101 not found at 0x%02X", this->address_);
    return;
  }
  this->apply_sensor_config_();
  this->ready_ = true;
  ESP_LOGI(TAG, "MAX30101 initialized");
}

void Max30101Component::apply_sensor_config_() {
  this->pox_.setIRLedCurrent(this->ir_current_ma_);
  this->pox_.setRedLedCurrent(this->red_current_ma_);
  this->pox_.setSamplingRate(this->sample_rate_hz_);
  this->pox_.setPulseWidthUS(this->pulse_width_us_);
}

void Max30101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX30101:");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  IR current: %.1f mA", this->ir_current_ma_);
  ESP_LOGCONFIG(TAG, "  Red current: %.1f mA", this->red_current_ma_);
  ESP_LOGCONFIG(TAG, "  Sample rate: %d Hz", this->sample_rate_hz_);
  ESP_LOGCONFIG(TAG, "  Pulse width: %d us", this->pulse_width_us_);
  LOG_UPDATE_INTERVAL(this);
}

void Max30101Component::update() {
  if (!this->ready_)
    return;

  this->pox_.update();

  float hr = this->pox_.getHeartRate();
  float spo2 = this->pox_.getSpO2();

  if (!std::isnan(hr) && this->hr_sensor_ != nullptr) {
    this->hr_sensor_->publish_state(hr);
  }
  if (!std::isnan(spo2) && this->spo2_sensor_ != nullptr) {
    this->spo2_sensor_->publish_state(spo2);
  }
}

}  // namespace max30101
}  // namespace esphome
