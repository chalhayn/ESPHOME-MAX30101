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
static const uint8_t REG_LED2_PA      = 0xD;   // IR (note: some headers use 0x0D)
static const uint8_t REG_PART_ID      = 0xFF;

static const uint8_t MAX30102_EXPECTEDPARTID = 0x15;

class MAX30102NativeSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_heart_rate_sensor(sensor::Sensor *s) { heart_rate_sensor_ = s; }
  void set_spo2_sensor(sensor::Sensor *s)      { spo2_sensor_       = s; }

 protected:
  bool initialize_sensor();
  void clear_fifo();

  // Legacy helpers
  uint32_t get_ir();
  uint32_t get_red();
  bool check_for_beat(int32_t sample);

  // Outputs
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

  // ---------- Tunables ----------
  // FIFO averaging: default 4-sample avg (bits 7:5=0b010) for stronger AC
  static constexpr uint8_t FIFO_CFG_BYTE = 0x50;

  // SpO₂ algorithm buffers (expects ~25 Hz, ~100-sample window)
  static constexpr uint16_t SPO2_WIN   = 100;
  static constexpr uint8_t  SPO2_DECIM = 4;     // 100 Hz -> 25 Hz
  static constexpr uint32_t SAMPLE_PERIOD_MS = 10; // 100 Hz

  // Finger-present gates (raw 18-bit)
  static constexpr uint32_t FINGER_IR_MIN  = 20000;
  static constexpr uint32_t FINGER_RED_MIN = 10000;

  // DC tracking for perfusion (AC/DC %)
  static constexpr float DC_ALPHA     = 0.95f;
  static constexpr float PERF_MIN_PCT = 0.10f;
  static constexpr float PERF_MAX_PCT = 12.0f;

  // SpO₂ guards
  static constexpr float   SPO2_JUMP_MAX_PCT    = 5.0f;
  static constexpr uint8_t SPO2_SETTLE_WINDOWS  = 2;
  static constexpr float   SPO2_OFFSET          = -2.0f;
  static constexpr float   SPO2_MAX_PUBLISH     = 99.0f;

  // HR: prefer IBI; HR_algo publish is optional and gated to IBI
  static constexpr bool    PUBLISH_HR_ALGO         = false;
  static constexpr uint8_t HR_IBI_MIN_BEATS        = 5;
  static constexpr float   HR_ALGO_IBI_DIFF_TIGHT  = 12.0f;

  // HR absolute plausibility (24–200 bpm)
  static constexpr uint32_t HR_MIN_DT_MS = 300;
  static constexpr uint32_t HR_MAX_DT_MS = 2500;

  // Acquisition (wide) vs stable gates
  static constexpr float ACQ_MED_LOW  = 0.45f;
  static constexpr float ACQ_MED_HIGH = 2.40f;  // widened so 140→70 bpm can lock (≈×2)

  static constexpr float MED_LOW  = 0.55f;
  static constexpr float MED_HIGH = 1.60f;

  static constexpr float LAST_LOW  = 0.70f;
  static constexpr float LAST_HIGH = 1.50f;     // a touch wider

  // Auto-resyncs
  static constexpr uint8_t  REJECT_STREAK_FOR_RESYNC = 4;
  static constexpr float    REJECT_STREAK_SPREAD_MAX = 1.15f;

  // NEW: timeout rescue & “two-consistent-candidates”
  static constexpr uint32_t HR_NO_ACCEPT_TIMEOUT_MS  = 3500; // ~3.5 s
  static constexpr float    HR_CANDIDATE_PAIR_TOL    = 0.20f; // ±20% match → accept

  // ---------- State ----------
  // SpO₂ buffers
  uint32_t ir_buf_[SPO2_WIN]  = {0};
  uint32_t red_buf_[SPO2_WIN] = {0};
  uint16_t spo2_count_ = 0;
  uint8_t  decim_count_ = 0;

  // Sample clock
  uint32_t sample_tick_ms_{0};

  // HR state
  static constexpr uint8_t RATE_SIZE = 8;
  int32_t  rates_[RATE_SIZE] = {0};
  uint32_t rate_array_{0};

  uint32_t last_beat_tick_ms_{0};    // tick at last **accepted** beat
  uint32_t last_good_dt_ms_{0};      // last accepted IBI
  float    last_ibi_bpm_{0.0f};      // NaN in setup
  uint8_t  ibi_valid_beats_{0};

  // Reject tracking
  uint8_t  reject_streak_{0};
  uint32_t reject_min_dt_{0};
  uint32_t reject_max_dt_{0};

  // NEW: candidate-pair rescue
  uint32_t last_candidate_dt_ms_{0};

  // DC baselines
  float dc_ir_{0.0f};
  float dc_red_{0.0f};

  // SpO₂ publication stability
  float   last_spo2_{0.0f};   // NaN in setup
  uint8_t spo2_settle_windows_{0};
};

}  // namespace max30102_native
}  // namespace esphome
