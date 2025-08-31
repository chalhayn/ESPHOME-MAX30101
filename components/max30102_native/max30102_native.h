#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace max30102_native {

// ------- MAX30102 registers (subset) -------
static const uint8_t REG_INTR_STATUS_1 = 0x00;
static const uint8_t REG_INTR_STATUS_2 = 0x01;
static const uint8_t REG_INTR_ENABLE_1 = 0x02;
static const uint8_t REG_INTR_ENABLE_2 = 0x03;
static const uint8_t REG_FIFO_WR_PTR   = 0x04;
static const uint8_t REG_OVF_COUNTER   = 0x05;
static const uint8_t REG_FIFO_RD_PTR   = 0x06;
static const uint8_t REG_FIFO_DATA     = 0x07;
static const uint8_t REG_FIFO_CONFIG   = 0x08;
static const uint8_t REG_MODE_CONFIG   = 0x09;
static const uint8_t REG_SPO2_CONFIG   = 0x0A;
static const uint8_t REG_LED1_PA       = 0x0C;  // RED current
static const uint8_t REG_LED2_PA       = 0x0D;  // IR current
static const uint8_t REG_PART_ID       = 0xFF;

static const uint8_t MAX30102_EXPECTEDPARTID = 0x15;

// Map FIFO sample-average (1/2/4/8/16/32) to bits (7:5)
static inline uint8_t fifo_avg_to_bits(uint8_t n) {
  switch (n) {
    default: case 1:  return 0b00000000; // 0x00
    case 2:  return 0b00100000;          // 0x20
    case 4:  return 0b01000000;          // 0x40
    case 8:  return 0b01100000;          // 0x60
    case 16: return 0b10000000;          // 0x80
    case 32: return 0b10100000;          // 0xA0
  }
}

class MAX30102NativeSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  // ---- Lifecycle ----
  void setup() override;
  void update() override;
  void dump_config() override;

  // ---- Exposed sensors ----
  void set_heart_rate_sensor(sensor::Sensor *s) { heart_rate_sensor_ = s; }
  void set_spo2_sensor(sensor::Sensor *s)      { spo2_sensor_       = s; }

  // ---- YAML-tunable setters (callable from on_boot lambda) ----
  // Hardware / sampling
  void set_led_currents(uint8_t red_pa, uint8_t ir_pa) { led_red_pa_ = red_pa; led_ir_pa_ = ir_pa; reinit_needed_ = true; }
  void set_fifo_average(uint8_t n) { fifo_avg_sel_ = n; reinit_needed_ = true; }
  void set_sample_rate_hz(uint16_t hz) { if (hz == 100) sample_rate_hz_ = 100; reinit_needed_ = true; } // only 100 Hz supported here

  // Finger-present thresholds
  void set_finger_thresholds(uint32_t ir_min, uint32_t red_min) { finger_ir_min_ = ir_min; finger_red_min_ = red_min; }

  // SpO2 tuning
  void set_spo2_offset(float v)       { spo2_offset_ = v; }
  void set_spo2_max_publish(float v)  { spo2_max_pub_ = v; }
  void set_spo2_step_clamp_pct(float v) { spo2_step_clamp_pct_ = v; }
  void set_spo2_stuck_release(uint8_t recent_n, uint8_t windows, float jump_abs) {
    spo2_recent_n_ = (recent_n >= 3 && recent_n <= 7) ? recent_n : spo2_recent_n_;
    spo2_stuck_windows_ = windows;
    spo2_jump_release_abs_ = jump_abs;
  }

  // HR tuning
  void set_hr_publish_algo(bool en)         { publish_hr_algo_ = en; }
  void set_hr_refractory_ms(uint32_t ms)    { hr_refractory_ms_ = ms; }
  void set_hr_abs_range(uint32_t min_ms, uint32_t max_ms) { hr_min_dt_ms_ = min_ms; hr_max_dt_ms_ = max_ms; }
  void set_hr_no_accept_timeout_ms(uint32_t ms) { hr_no_accept_timeout_ms_ = ms; }
  void set_hr_candidate_pair_tol(float tol) { hr_candidate_pair_tol_ = tol; }
  void set_hr_pair_rescue_only_slower(bool en) { hr_pair_rescue_only_slower_ = en; }

  // Gating windows
  void set_acq_med_gates(float low, float high) { acq_med_low_ = low; acq_med_high_ = high; }
  void set_med_gates(float low, float high)     { med_low_ = low; med_high_ = high; }
  void set_last_gates(float low, float high)    { last_low_ = low; last_high_ = high; }

 protected:
  // ---- Internals ----
  bool initialize_sensor();
  void clear_fifo();

  // Legacy helpers (kept for compatibility with older lambdas)
  uint32_t get_ir();
  uint32_t get_red();
  bool check_for_beat(int32_t sample);

  // ---- Config (defaults) ----
  // HW / sampling
  uint8_t  led_red_pa_{0x24};
  uint8_t  led_ir_pa_{0x24};
  uint8_t  fifo_avg_sel_{4};   // 1/2/4/8/16/32
  uint16_t sample_rate_hz_{100};

  // Finger-present
  uint32_t finger_ir_min_{20000};
  uint32_t finger_red_min_{10000};

  // SpO2 (window ~100 @ 25Hz, half-slide)
  uint16_t spo2_win_{100};
  uint8_t  spo2_decim_{4};              // 100Hz -> 25Hz
  float    spo2_offset_{-2.0f};
  float    spo2_max_pub_{99.0f};
  float    spo2_step_clamp_pct_{5.0f};  // ±step allowed
  uint8_t  spo2_settle_windows_{2};

  // Stuck-release (median over recent corrected windows)
  uint8_t  spo2_recent_n_{5};           // keep 5 recent windows (~10s span)
  uint8_t  spo2_stuck_windows_{4};      // accept after ≥4 windows (~8s)
  float    spo2_jump_release_abs_{8.0f}; // if median differs ≥8%

  // HR preferences
  bool     publish_hr_algo_{false};
  uint8_t  hr_ibi_min_beats_{5};
  float    hr_algo_ibi_diff_tight_{12.0f}; // bpm
  uint32_t hr_min_dt_ms_{300};
  uint32_t hr_max_dt_ms_{2500};
  // Acquisition vs stable gates
  float    acq_med_low_{0.45f}, acq_med_high_{2.40f};
  float    med_low_{0.55f},     med_high_{1.60f};
  float    last_low_{0.70f},    last_high_{1.50f};
  // Rescues
  uint8_t  reject_streak_for_resync_{4};
  float    reject_streak_spread_max_{1.15f};
  uint32_t hr_no_accept_timeout_ms_{3500};
  float    hr_candidate_pair_tol_{0.20f};
  bool     hr_pair_rescue_only_slower_{true};
  uint32_t hr_refractory_ms_{300};

  // ---- State ----
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};

  bool     reinit_needed_{false};

  // Sample clock (ticks +10ms per IR+RED pair at 100Hz)
  uint32_t sample_tick_ms_{0};

  // SpO2 buffers / state
  uint32_t *ir_buf_{nullptr};
  uint32_t *red_buf_{nullptr};
  uint16_t spo2_count_{0};
  uint8_t  decim_count_{0};
  float    last_spo2_{NAN};
  // recent corrected windows for stuck-release
  float   *spo2_recent_{nullptr};
  uint8_t  spo2_recent_count_{0};
  uint8_t  spo2_recent_idx_{0};
  uint16_t spo2_windows_since_pub_{0};

  // DC baselines (for perfusion index and HR quality)
  float dc_ir_{0.0f};
  float dc_red_{0.0f};

  // HR state
  static constexpr uint8_t RATE_SIZE = 8;
  int32_t  rates_[RATE_SIZE] = {0};
  uint32_t rate_array_{0};
  uint32_t last_beat_tick_ms_{0};
  uint32_t last_good_dt_ms_{0};
  float    last_ibi_bpm_{NAN};
  uint8_t  ibi_valid_beats_{0};
  // reject tracking
  uint8_t  reject_streak_{0};
  uint32_t reject_min_dt_{0};
  uint32_t reject_max_dt_{0};
  // candidate pair + refractory
  uint32_t last_candidate_dt_ms_{0};
  uint32_t last_detect_tick_ms_{0};
};

}  // namespace max30102_native
}  // namespace esphome
