#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace max30102_native {

// ---------- Registers ----------
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
static const uint8_t REG_LED1_PA       = 0x0C;  // RED
static const uint8_t REG_LED2_PA       = 0x0D;  // IR
static const uint8_t REG_PART_ID       = 0xFF;

static const uint8_t MAX30102_EXPECTEDPARTID = 0x15;

static inline uint8_t fifo_avg_to_bits(uint8_t n) {
  switch (n) {
    default: case 1:  return 0b00000000;
    case 2:  return 0b00100000;
    case 4:  return 0b01000000;
    case 8:  return 0b01100000;
    case 16: return 0b10000000;
    case 32: return 0b10100000;
  }
}

class MAX30102NativeSensor : public PollingComponent, public i2c::I2CDevice {
 public:
  // Lifecycle
  void setup() override;
  void update() override;
  void dump_config() override;

  // Outputs
  void set_heart_rate_sensor(sensor::Sensor *s) { heart_rate_sensor_ = s; }
  void set_spo2_sensor(sensor::Sensor *s)      { spo2_sensor_       = s; }

  // ---------- YAML-settable knobs ----------
  // HW / sampling
  void set_led_currents(uint8_t red_pa, uint8_t ir_pa) { led_red_pa_ = red_pa; led_ir_pa_ = ir_pa; reinit_needed_ = true; }
  void set_fifo_average(uint8_t n) { fifo_avg_sel_ = n; reinit_needed_ = true; }
  void set_sample_rate_hz(uint16_t hz) { if (hz == 100) sample_rate_hz_ = 100; reinit_needed_ = true; } // fixed 100Hz build

  // Finger-present thresholds
  void set_finger_thresholds(uint32_t ir_min, uint32_t red_min) { finger_ir_min_ = ir_min; finger_red_min_ = red_min; }

  // SpO2
  void set_spo2_offset(float v)       { spo2_offset_ = v; }
  void set_spo2_max_publish(float v)  { spo2_max_pub_ = v; }
  void set_spo2_step_clamp_pct(float v) { spo2_step_clamp_pct_ = v; }
  void set_spo2_stuck_release(uint8_t recent_n, uint8_t windows, float jump_abs) {
    if (recent_n >= 3 && recent_n <= 7) spo2_recent_n_ = recent_n;
    spo2_stuck_windows_ = windows;
    spo2_jump_release_abs_ = jump_abs;
  }

  // HR: algorithm publishing + fallback
  void set_heart_rate_sensor_publish_algo(bool en) { publish_hr_algo_ = en; }
  void set_hr_algo_fallback_timeout_ms(uint32_t ms) { hr_algo_fallback_timeout_ms_ = ms; }

  // HR: detection & gating
  void set_hr_refractory_ms(uint32_t ms)    { hr_refractory_ms_ = ms; }
  void set_hr_abs_range(uint32_t min_ms, uint32_t max_ms) { hr_min_dt_ms_ = min_ms; hr_max_dt_ms_ = max_ms; }
  void set_hr_no_accept_timeout_ms(uint32_t ms) { hr_no_accept_timeout_ms_ = ms; }
  void set_hr_candidate_pair_tol(float tol) { hr_candidate_pair_tol_ = tol; }
  void set_hr_pair_rescue_only_slower(bool en) { hr_pair_rescue_only_slower_ = en; }
  void set_acq_med_gates(float low, float high) { acq_med_low_ = low; acq_med_high_ = high; }
  void set_med_gates(float low, float high)     { med_low_ = low; med_high_ = high; }
  void set_last_gates(float low, float high)    { last_low_ = low; last_high_ = high; }

 protected:
  // Internals
  bool initialize_sensor();
  void clear_fifo();

  // Legacy helpers (compat)
  uint32_t get_ir();
  uint32_t get_red();
  bool check_for_beat(int32_t sample);

  // ---------- Defaults ----------
  // HW
  uint8_t  led_red_pa_{0x26};         // a tad higher by default
  uint8_t  led_ir_pa_{0x26};
  uint8_t  fifo_avg_sel_{4};          // 4-sample avg
  uint16_t sample_rate_hz_{100};      // fixed here

  // Finger-present
  uint32_t finger_ir_min_{15000};     // eased to ensure HR path runs
  uint32_t finger_red_min_{8000};

  // SpO2
  uint16_t spo2_win_{100};
  uint8_t  spo2_decim_{4};            // 100Hz -> 25Hz
  float    spo2_offset_{-2.0f};
  float    spo2_max_pub_{99.0f};
  float    spo2_step_clamp_pct_{8.0f};     // allow bigger steps
  uint8_t  spo2_settle_windows_{2};
  // Stuck-release
  uint8_t  spo2_recent_n_{5};
  uint8_t  spo2_stuck_windows_{3};         // ≈6 s
  float    spo2_jump_release_abs_{8.0f};

  // HR prefs
  bool     publish_hr_algo_{false};
  uint8_t  hr_ibi_min_beats_{5};
  float    hr_algo_ibi_diff_tight_{12.0f};  // bpm
  uint32_t hr_min_dt_ms_{250};              // 40..240 bpm
  uint32_t hr_max_dt_ms_{3000};             // let slow rates seed too
  // gates
  float    acq_med_low_{0.45f}, acq_med_high_{2.60f};
  float    med_low_{0.55f},     med_high_{1.60f};
  float    last_low_{0.70f},    last_high_{1.50f};
  // rescues
  uint8_t  reject_streak_for_resync_{4};
  float    reject_streak_spread_max_{1.15f};
  uint32_t hr_no_accept_timeout_ms_{3000};
  float    hr_candidate_pair_tol_{0.20f};
  bool     hr_pair_rescue_only_slower_{true};
  uint32_t hr_refractory_ms_{280};          // slightly tighter
  // NEW: publish HR_algo if IBI silent too long
  uint32_t hr_algo_fallback_timeout_ms_{5000};

  // ---------- State ----------
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *spo2_sensor_{nullptr};
  bool     reinit_needed_{false};

  // Sample clock
  uint32_t sample_tick_ms_{0};

  // SpO2 buffers / state
  uint32_t *ir_buf_{nullptr};
  uint32_t *red_buf_{nullptr};
  uint16_t spo2_count_{0};
  uint8_t  decim_count_{0};
  float    last_spo2_{NAN};
  // recent corrected windows
  float   *spo2_recent_{nullptr};
  uint8_t  spo2_recent_count_{0};
  uint8_t  spo2_recent_idx_{0};
  uint16_t spo2_windows_since_pub_{0};

  // DC baselines
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
  // rejects
  uint8_t  reject_streak_{0};
  uint32_t reject_min_dt_{0};
  uint32_t reject_max_dt_{0};
  // candidates + refractory
  uint32_t last_candidate_dt_ms_{0};
  uint32_t last_detect_tick_ms_{0};
  // debug: raw beat triggers seen (to diagnose “no HR detection”)
  uint16_t raw_beats_in_window_{0};
};

}  // namespace max30102_native
}  // namespace esphome
