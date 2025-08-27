
#include "esphome/core/log.h"
#include "MAX30102.h" 

namespace esphome {
namespace max30102 {

static const char *TAG = "max30102";

void MAX30102Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX30102...");
  
  if (!particleSensor_.begin()) {
    ESP_LOGE(TAG, "MAX30102 was not found. Please check wiring/power.");
    this->mark_failed();
    return;
  }

  particleSensor_.setup(); // Configure sensor with default settings
  particleSensor_.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor_.setPulseAmplitudeGreen(0); // Turn off Green LED
}

void MAX30102Component::update() {
  long irValue = particleSensor_.getIR();
  
  if (checkForBeat(irValue)) {
    // Calculate time between beats
    long delta = millis() - last_beat_;
    last_beat_ = millis();

    // Store valid reading
    rates_[rate_array_++] = (60 / (delta / 1000.0));
    rate_array_ %= RATE_SIZE;

    // Take average of readings to smooth out data
    long total = 0;
    for (byte i = 0; i < RATE_SIZE; i++) {
      total += rates_[i];
    }
    long beatsPerMinute = total / RATE_SIZE;

    if (heart_rate_sensor_ != nullptr && beatsPerMinute > 20 && beatsPerMinute < 255) {
      heart_rate_sensor_->publish_state(beatsPerMinute);
    }
  }

  // Calculate SpO2 (simplified - you may want to implement the full algorithm)
  if (spo2_sensor_ != nullptr) {
    // This is a simplified calculation - implement proper SpO2 calculation
    // based on the ratio of red and IR light absorption
    uint32_t redValue = particleSensor_.getRed();
    float ratio = (float)redValue / (float)irValue;
    float spo2 = 110 - 25 * ratio; // Simplified formula
    
    if (spo2 > 80 && spo2 < 100) {
      spo2_sensor_->publish_state(spo2);
    }
  }
}

void MAX30102Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX30102:");
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Heart Rate", this->heart_rate_sensor_);
  LOG_SENSOR("  ", "SpO2", this->spo2_sensor_);
}

}  // namespace max30102
}  // namespace esphome
