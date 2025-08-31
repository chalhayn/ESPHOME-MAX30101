#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

// Keep C-style algo headers OUTSIDE any namespace.
#include <heartRate.h>        // SparkFun MAX3010x PBA beat detector
#include <spo2_algorithm.h>   // Maxim/ProtoCentral SpO2 algorithm

#include <cstring>            // std::memmove
#include <cmath>              // std::isnan, std::fabs
#include <algorithm>          // std::sort
#include <math.h>             // NAN (ensure macro exists)

namespace esphome {
namespace max30102_native {

static const char *const TAG = "max30102_native";

// ---- helpers (file-local) ----
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static float median_from_ibis(const int32_t *buf, uint8_t n) {
  // Collect non-zero IBIs into a small local array
  float tmp[16]; // RATE_SIZE ≤ 8, 16 is safe headroom
  uint8_t m = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (buf[i] > 0) tmp[m++] = (float) buf[i];
  }
  if (m == 0) return NAN;
  std::sort(tmp, tmp + m);
  if (m & 1) return tmp[m/2];
  return 0.5f * (tmp[m/2 - 1] + tmp[m/2]);
}

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

  // Initialize publication state
  this->last_spo2_ = NAN;
  this->last_ibi_bpm_ = NAN;
  this->spo2_settle_windows_ = 0;
  this->ibi_valid_beats_ = 0;

  ESP_LOGCONFIG(TAG, "MAX30102 Native initialized successfully");
}

bool MAX30102NativeSensor::initialize_sensor() {
  // Soft reset
  if (!this->write_byte(REG_MODE_CONFIG, 0x40)) return false;
  delay(100);

  // FIFO config: sample avg=8, rollover=0, fifo_a_full default
  if (!this->write_byte(REG_FIFO_CONFIG, FIFO_CFG_BYTE)) return false;

  // SpO2 mode (0x03)
  if (!this->write_byte(REG_MODE_CONFIG, 0x03)) return false;

  // SpO2 config: ADC range=4096nA, sample rate=100Hz, pulse width=411us -> 0x27
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

// Optional/legacy helpers (not used by update(); kept for compatibility)
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
  // Unused; we leverage SparkFun's checkForBeat() directly in update()
  return false;
}

void MAX30102NativeSensor::update() {
  // Determine how many 3-byte samples are waiting in FIFO
  uint8_t rd = 0, wr = 0;
