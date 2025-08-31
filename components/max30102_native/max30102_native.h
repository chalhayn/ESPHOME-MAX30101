#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace max30102_native {

// MAX30102 register addresses (subset we use)
static const uint8_t REG_FIFO_WR_PTR  = 0x04;
static const uint8_t REG_OVF_COUNTER  = 0x05;
static const uint8_t REG_FIFO_RD_PTR  = 0x06;
static const uint8_t REG_FIFO_DATA    = 0x07;
static const uint8_t REG_FIFO_CONFIG  = 0x08;
static const uint8_t REG_MODE_CONFIG  = 0x09;
static const uint8_t REG_SPO2_CONFIG  = 0x0A;
static const uint8_t REG_LED1_PA      = 0x0C;  // RED
static const uint8_t REG_LED2_PA      = 0x0D;  // IR
static const uint8_t REG_PART_ID      = 0xFF;

static const uint8_t MAX30102_EXPECTEDPARTID = 0x15;

class MAX30102NativeSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  // Component lifecycle
  void setup() override;
  void update() override;
  void dump_config() override;

  // Setters wired from codegen
  void set_heart_rate_sensor(sensor::Sensor *s) { heart_rate_sensor_ = s; }
  void set_spo2_sensor(sensor::Sensor *s)      { spo2_sensor_       = s; }

 protected:
  // Internal helpers
  bool initialize_sensor();
  void clear_fifo();

  // Legacy helpers (optional/compat)
  uint32_t get_ir();
  uint32_t get_red();
  bool check_for_beat(int32_t sample);

  // Output sensors
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

  // ---------- Tunables / guards ----------
  // FIFO averaging: use 8-sample average to reduce noise (bits 7:5 = 0b011)
  static constexpr uint8_t FIFO_CFG_BYTE = 0x60;

  // SpO₂ algorithm buffers (expects ~25 Hz, ~100-sample window)
  static constexpr uint16_t SPO2_WIN   = 100;
  static constexpr uint8_t  SPO2_DECIM = 4;    // 100 Hz -> 25 Hz

  // Basic finger-present thresholds (raw 18-bit)
  static constexpr uint32_t FINGER_IR_MIN  = 20000;
  static constexpr uint32_t FINGER_RED_MIN = 10000;

  // DC tracking for crude perfusion index (AC/DC%)
  static constexpr float DC_ALPHA     = 0.95f;  // EMA baseline
  static constexpr float PERF_MIN_PCT = 0.10f;  // relaxed (was 0.5)
  static constexpr float PERF_MAX_PCT = 12.0f;  // relaxed (was 5.0)

  // SpO₂ publication guards
  static constexpr float   SPO2_JUMP_MAX_PCT    = 5.0f;   // widened (was 3.0)
  static constexpr uint8_t SPO2_SETTLE_WINDOWS  = 2;      // shorter settle (was 5)

  // ---- SpO₂ calibration (simple bias & soft ceiling) ----
  static constexpr float   SPO2_OFFSET          = -2.0f;  // default −2% (tune to your pulse-ox)
  static constexpr float   SPO2_MAX_PUBLISH     = 99.0f;  // soft cap so we don't peg at 100

  // ---- HR preferences: prefer IBI; optionally publish HR_algo if it agrees tightly ----
  static constexpr bool    PUBLISH_HR_ALGO         = false; // default OFF (IBI is authoritative)
  static constexpr uint8_t HR_IBI_MIN_BEATS        = 5;      // need ≥5 valid IBI samples
  static constexpr float   HR_ALGO_IBI_DIFF_TIGHT  = 12.0f;  // max allowed diff (bpm)

  // ---- HR robustness: reject IBI outliers vs rolling median ----
  static constexpr float   IBI_OUTLIER_LOW_FACTOR  = 0.60f;  // reject < 0.60× median
  static constexpr float   IBI_OUTLIER_HIGH_FACTOR = 1.40f;  // reject > 1.40× median

  // ---------- State ----------
  // SpO₂ buffers
  uint32_t ir_buf_[SPO2_WIN]  = {0};
  uint32_t red_buf_[SPO2_WIN] = {0};
  uint16_t spo2_count_ = 0;
  uint8_t  decim_count_ = 0;

  // Beat timing / smoothing state (IBI → BPM)
  static constexpr uint8_t RATE_SIZE = 8;
  int32_t  rates_[RATE_SIZE] = {0};  // recent inter-beat intervals (ms)
  uint32_t rate_array_{0};
  uint32_t last_beat_{0};
  float    last_ibi_bpm_{0.0f};      // set to NaN in setup()
  uint8_t  ibi_valid_beats_{0};      // how many usable IBI samples in the current avg

  // DC baselines for perfusion index
  float dc_ir_{0.0f};
  float dc_red_{0.0f};

  // SpO₂ publication stability
  float   last_spo2_{0.0f};          // set to NaN in setup()
  uint8_t spo2_settle_windows_{0};
};

}  // namespace max30102_native
}  // namespace esphome
