#include "max30101_component.h"

namespace esphome {
namespace max30101 {

static const char *const TAG = "max30101";

void Max30101Component::setup() {
  // Initialize I2C device
  if (!this->sensor_.begin(Wire, I2C_SPEED_FAST, this->address_)) {
    ESP_LOGE(TAG, "MAX3010x not found at 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  apply_sensor_config_();

  ESP_LOGI(TAG, "MAX3010x initialized (addr=0x%02X, mode=%u, rate=%dHz, pw=%dus, range=%d, avg=%u, bright=0x%02X)",
           this->address_, this->led_mode_, this->sample_rate_hz_, this->pulse_width_us_,
           this->adc_range_, this->sample_average_, this->led_brightness_);
}

void Max30101Component::apply_sensor_config_() {
  // SparkFun’s setup packs most config into one call
  this->sensor_.setup(
      this->led_brightness_,   // 0x00..0xFF
      this->sample_average_,   // 1,2,4,8,16,32
      this->led_mode_,         // 1=Red, 2=Red+IR, 3=Red+IR+Green
      this->sample_rate_hz_,   // 50..3200
      this->pulse_width_us_,   // 69,118,215,411
      this->adc_range_         // 2048,4096,8192,16384
  );

  // Optional: tweak per-LED amplitudes (leave at default or expose)
  // this->sensor_.setPulseAmplitudeRed(this->led_brightness_);
  // this->sensor_.setPulseAmplitudeIR(this->led_brightness_);
  // this->sensor_.setPulseAmplitudeGreen(this->led_brightness_);
}

void Max30101Component::update() {
  // Non-blocking sample readiness (max time to check in ms)
  if (!this->sensor_.safeCheck(5)) {
    return;  // nothing ready
  }

  // Publish raw channels if sensors are configured
  if (this->ir_sensor != nullptr) {
    this->ir_sensor->publish_state(static_cast<float>(this->sensor_.getIR()));
  }
  if (this->red_sensor != nullptr) {
    this->red_sensor->publish_state(static_cast<float>(this->sensor_.getRed()));
  }
  if (this->green_sensor != nullptr && this->led_mode_ == 3) {
    this->green_sensor->publish_state(static_cast<float>(this->sensor_.getGreen()));
  }

  // Optional: die temperature (call occasionally; it’s relatively slow)
  if (this->die_temp_sensor != nullptr) {
    float t = this->sensor_.readTemperature();
    if (!isnan(t)) this->die_temp_sensor->publish_state(t);
  }
}

}  // namespace max30101
}  // namespace esphome
