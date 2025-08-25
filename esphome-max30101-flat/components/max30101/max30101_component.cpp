#include "max30101_component.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace max30101 {

static const char *const TAG = "max30101";

void Max30101Component::setup() {
  // Initialize via SparkFun API. ESPHome’s I2C already set SDA/SCL, but we still pass Wire.
  bool ok = this->sensor_.begin(Wire, I2C_SPEED_FAST, this->address_);  // default addr 0x57
  if (!ok) {
    ESP_LOGE(TAG, "MAX30101 not found at 0x%02X", this->address_);
    return;
  }

  // Basic bring-up: LED mode = Red+IR (2-LED), default ADC range, etc.
  // SparkFun helper: setup() with defaults, then override specifics.
  this->sensor_.setup();                 // load library defaults
  this->sensor_.setLEDMode(2);           // 2 = Red + IR
  this->sensor_.setFIFOAverage(4);       // average samples to reduce noise

  this->apply_sensor_config_();
  this->ready_ = true;
  ESP_LOGI(TAG, "MAX30101 initialized");
}

void Max30101Component::apply_sensor_config_() {
  // Map currents (mA) to register scale (0..255). SparkFun expects 0..255.
  uint8_t ir_amp = this->map_current_ma_to_reg_(this->ir_current_ma_);
  uint8_t red_amp = this->map_current_ma_to_reg_(this->red_current_ma_);
  this->sensor_.setPulseAmplitudeIR(ir_amp);
  this->sensor_.setPulseAmplitudeRed(red_amp);

  // Sample rate & pulse width enums
  this->sensor_.setSampleRate(this->map_sr_hz_to_enum_(this->sample_rate_hz_));
  this->sensor_.setPulseWidth(this->map_pw_us_to_enum_(this->pulse_width_us_));

  // You can also tune ADC range: 0=2048nA,1=4096nA,2=8192nA,3=16384nA; start with default
  // this->sensor_.setADCRange(MAX30105_ADCRANGE_16384); // example
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

  // Pull newest FIFO samples so a future algorithm has data.
  // (SparkFun pattern: while (sensor_.available()) { sensor_.getRed(); sensor_.getIR(); sensor_.nextSample(); })
  while (this->sensor_.available()) {
    volatile uint32_t red = this->sensor_.getRed();
    volatile uint32_t ir  = this->sensor_.getIR();
    (void) red; (void) ir;
    this->sensor_.nextSample();
  }

  // No algorithm wired yet -> publish NaN (safe) so HA doesn’t show bogus numbers
  if (this->hr_sensor_ != nullptr)   this->hr_sensor_->publish_state(NAN);
  if (this->spo2_sensor_ != nullptr) this->spo2_sensor_->publish_state(NAN);
}

uint8_t Max30101Component::map_current_ma_to_reg_(float ma) const {
  // MAX3010x LED current roughly 0..50 mA; SparkFun uses 0..255 bytes
  if (ma <= 0) return 0;
  if (ma >= 50.0f) return 255;
  return static_cast<uint8_t>(std::round(ma * (255.0f / 50.0f)));
}

uint8_t Max30101Component::map_sr_hz_to_enum_(int hz) const {
  // Map common rates to SparkFun enums (see library docs)
  // 50,100,200,400 are typical; fall back to nearest
  if (hz <= 50)   return 50;
  if (hz <= 100)  return 100;
  if (hz <= 200)  return 200;
  return 400;
}

uint8_t Max30101Component::map_pw_us_to_enum_(int us) const {
  // SparkFun uses discrete pulse widths; choose nearest
  if (us <= 118)  return 118;
  if (us <= 215)  return 215;
  if (us <= 411)  return 411;
  return 822;
}

}  // namespace max30101
}  // namespace esphome
