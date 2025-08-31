#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

// Keep C-style algo headers OUTSIDE any namespace.
#include <heartRate.h>        // SparkFun beat detector
#include <spo2_algorithm.h>   // Maxim/ProtoCentral SpO2

#include <cstring>
#include <cmath>
#include <algorithm>
#include <limits>

namespace esphome {
namespace max30102_native {

static const char *const TAG = "max30102_native";

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static float median_from_ibis(const int32_t *buf, uint8_t n) {
  float tmp[16]; uint8_t m = 0;
  for (uint8_t i = 0; i < n; i++) if (buf[i] > 0) tmp[m++] = (float) buf[i];
  if (m == 0) return NAN;
  std::sort(tmp, tmp + m);
  return (m & 1) ? tmp[m/2] : 0.5f * (tmp[m/2 - 1] + tmp[m/2]);
}
static float medianf(const float *in, uint8_t n) {
  if (n == 0) return NAN;
  float tmp[8];
  for (uint8_t i = 0; i < n; i++) tmp[i] = in[i];
  std::sort(tmp, tmp + n);
  return (n & 1) ? tmp[n/2] : 0.5f * (tmp[n/2 - 1] + tmp[n/2]);
}

void MAX30102NativeSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX30102 Native...");

  // dynamic buffers
  ir_buf_  = new uint32_t[spo2_win_]{0};
  red_buf_ = new uint32_t[spo2_win_]{0};
  spo2_recent_ = new float[spo2_recent_n_];
  for (uint8_t i = 0; i < spo2_recent_n_; i++) spo2_recent_[i] = NAN;

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

  // state
  sample_tick_ms_ = 0;
  last_beat_tick_ms_ = 0;
  last_good_dt_ms_ = 0;
  last_ibi_bpm_ = NAN;
  ibi_valid_beats_ = 0;
  reject_streak_ = 0;
  reject_min_dt_ = std::numeric_limits<uint32_t>::max();
  reject_max_dt_ = 0;
  last_candidate_dt_ms_ = 0;
  last_detect_tick_ms_ = 0;
  raw_beats_in_window_ = 0;

  last_spo2_ = NAN;
  spo2_count_ = 0;
  decim_count_ = 0;
  spo2_recent_count_ = 0;
  spo2_recent_idx_ = 0;
  spo2_windows_since_pub_ = 0;
  dc_ir_ = dc_red_ = 0.0f;

  ESP_LOGCONFIG(TAG, "MAX30102 Native initialized successfully");
}

bool MAX30102NativeSensor::initialize_sensor() {
  if (!this->write_byte(REG_MODE_CONFIG, 0x40)) return false;  // reset
  delay(100);

  uint8_t fifo_cfg = fifo_avg_to_bits(fifo_avg_sel_);
  if (!this->write_byte(REG_FIFO_CONFIG, fifo_cfg)) return false;

  if (!this->write_byte(REG_MODE_CONFIG, 0x03)) return false;  // SpO2 mode
  if (!this->write_byte(REG_SPO2_CONFIG, 0x27)) return false;  // 4096nA, 100Hz, 411us

  if (!this->write_byte(REG_LED1_PA, led_red_pa_)) return false;
  if (!this->write_byte(REG_LED2_PA, led_ir_pa_))  return false;

  this->clear_fifo();
  return true;
}

void MAX30102NativeSensor::clear_fifo() {
  this->write_byte(REG_FIFO_WR_PTR, 0);
  this->write_byte(REG_OVF_COUNTER, 0);
  this->write_byte(REG_FIFO_RD_PTR, 0);
}

// legacy helpers
uint32_t MAX30102NativeSensor::get_ir()  { uint8_t d[3]; if (!this->read_bytes(REG_FIFO_DATA, d, 3)) return 0;
  return ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2]; }
uint32_t MAX30102NativeSensor::get_red() { uint8_t d[3]; if (!this->read_bytes(REG_FIFO_DATA, d, 3)) return 0;
  return ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2]; }
bool MAX30102NativeSensor::check_for_beat(int32_t) { return false; }

void MAX30102NativeSensor::update() {
  if (reinit_needed_) {
    ESP_LOGI(TAG, "Re-initializing sensor due to YAML changes...");
    this->initialize_sensor();
    reinit_needed_ = false;
  }

  uint8_t rd=0, wr=0;
  if (!this->read_byte(REG_FIFO_RD_PTR, &rd) || !this->read_byte(REG_FIFO_WR_PTR, &wr)) {
    ESP_LOGD(TAG, "FIFO pointer read failed"); return;
  }
  uint8_t samples = (wr - rd) & 0x1F;
  if (samples < 2) samples = 2;

  for (uint8_t i = 0; i + 1 < samples; i += 2) {
    uint8_t data[6];
    if (!this->read_bytes(REG_FIFO_DATA, data, 6)) { ESP_LOGD(TAG, "FIFO read failed"); break; }

    uint32_t red_u = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    uint32_t ir_u  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    red_u &= 0x3FFFF; ir_u &= 0x3FFFF;

    // sample clock (+10 ms/pair)
    sample_tick_ms_ += 1000u / sample_rate_hz_;

    // contact gate
    if (ir_u < finger_ir_min_ || red_u < finger_red_min_) {
      continue;
    }

    // DC/perfusion
    const float alpha = 0.95f;
    dc_ir_  = (dc_ir_  == 0.0f) ? (float)ir_u  : alpha * dc_ir_  + (1.0f - alpha) * (float)ir_u;
    dc_red_ = (dc_red_ == 0.0f) ? (float)red_u : alpha * dc_red_ + (1.0f - alpha) * (float)red_u;
    float ac_ir  = std::fabs((float) ir_u  - dc_ir_);
    float ac_red = std::fabs((float) red_u - dc_red_);
    float pi_ir  = (dc_ir_  > 1.0f) ? (ac_ir  / dc_ir_ ) * 100.0f : 0.0f;
    float pi_red = (dc_red_ > 1.0f) ? (ac_red / dc_red_) * 100.0f : 0.0f;
    const bool perf_ok = (pi_ir >= 0.10f && pi_ir <= 12.0f) && (pi_red >= 0.10f && pi_red <= 12.0f);

    // ---------------- HR path ----------------
    if (checkForBeat((int32_t) ir_u)) {
      raw_beats_in_window_++;

      // refractory for raw triggers
      uint32_t since_last_detect = (last_detect_tick_ms_ == 0)
        ? std::numeric_limits<uint32_t>::max()
        : (sample_tick_ms_ - last_detect_tick_ms_);
      if (since_last_detect < hr_refractory_ms_) {
        continue; // ignore too-close double counts
      }
      last_detect_tick_ms_ = sample_tick_ms_;

      const uint32_t dt = (last_beat_tick_ms_ == 0) ? 0 : (sample_tick_ms_ - last_beat_tick_ms_);
      if (last_beat_tick_ms_ != 0) {
        bool accept = false;
        const bool plausible = (dt >= hr_min_dt_ms_ && dt <= hr_max_dt_ms_);
        const uint32_t since_accept = (last_beat_tick_ms_ == 0) ? 0 : (sample_tick_ms_ - last_beat_tick_ms_);

        // candidate-pair similarity
        bool pair_ok = false;
        if (last_candidate_dt_ms_ > 0 && dt > 0) {
          const uint32_t hi = std::max(dt, last_candidate_dt_ms_);
          const uint32_t lo = std::min(dt, last_candidate_dt_ms_);
          pair_ok = ((float)hi / (float)lo <= (1.0f + hr_candidate_pair_tol_));
        }
        last_candidate_dt_ms_ = (dt > 0) ? dt : last_candidate_dt_ms_;

        // A) timeout rescue
        if (!accept && plausible && since_accept >= hr_no_accept_timeout_ms_) {
          accept = true;
          ESP_LOGD(TAG, "HR rescue(timeout): dt=%lu ms after %lu ms no-accept",
                   (unsigned long)dt, (unsigned long)since_accept);
        }

        // B) pair-rescue (slower only, perf OK)
        if (!accept && plausible && pair_ok && perf_ok) {
          bool ok_slow = !hr_pair_rescue_only_slower_
                       || (last_good_dt_ms_ == 0)
                       || (dt >= (uint32_t)(1.15f * (float)last_good_dt_ms_));
          if (ok_slow) {
            accept = true;
            ESP_LOGD(TAG, "HR rescue(pair): dt=%lu ms (consistent + slower)", (unsigned long) dt);
          }
        }

        // C) normal gating vs median/last
        if (!accept && plausible) {
          const float med = median_from_ibis(rates_, RATE_SIZE);
          const bool have_med  = !std::isnan(med) && med > 0.0f;
          const bool have_last = (last_good_dt_ms_ > 0);
          const bool in_acq = (ibi_valid_beats_ < hr_ibi_min_beats_);

          bool ok_med = false;
          if (have_med) {
            const float lo = (in_acq ? acq_med_low_ : med_low_) * med;
            const float hi = (in_acq ? acq_med_high_ : med_high_) * med;
            ok_med = (dt >= (uint32_t) lo && dt <= (uint32_t) hi);
          }
          bool ok_last = false;
          if (have_last) {
            const float lo = (in_acq ? acq_med_low_ : last_low_) * (float)last_good_dt_ms_;
            const float hi = (in_acq ? acq_med_high_ : last_high_) * (float)last_good_dt_ms_;
            ok_last = (dt >= (uint32_t) lo && dt <= (uint32_t) hi);
          }
          accept = (!have_med && !have_last) || ok_med || ok_last;

          if (!accept) {
            reject_streak_++;
            reject_min_dt_ = std::min(reject_min_dt_, dt);
            reject_max_dt_ = std::max(reject_max_dt_, dt);
            if (reject_streak_ >= reject_streak_for_resync_ &&
                reject_min_dt_ > 0 &&
                (float)reject_max_dt_ / (float)reject_min_dt_ <= reject_streak_spread_max_) {
              accept = true;
              ESP_LOGD(TAG, "HR rescue(streak): dt=%lu ms after %u consistent rejects (%lu..%lu ms)",
                       (unsigned long)dt, (unsigned)reject_streak_,
                       (unsigned long)reject_min_dt_, (unsigned long)reject_max_dt_);
            } else {
              ESP_LOGD(TAG, "IBI rejected: dt=%lu ms", (unsigned long) dt);
            }
          } else {
            reject_streak_ = 0;
            reject_min_dt_ = std::numeric_limits<uint32_t>::max();
            reject_max_dt_ = 0;
          }
        }

        if (accept) {
          rates_[rate_array_ % RATE_SIZE] = (int32_t) dt;
          rate_array_++;
          last_good_dt_ms_   = dt;
          last_beat_tick_ms_ = sample_tick_ms_;

          uint32_t sum_dt = 0; uint8_t count = 0;
          for (uint8_t k = 0; k < RATE_SIZE; k++) { int32_t ibi = rates_[k]; if (ibi > 0) { sum_dt += (uint32_t)ibi; count++; } }
          if (count >= 2) {
            float avg_dt = (float)sum_dt / (float)count;
            float bpm    = 60000.0f / avg_dt;
            if (heart_rate_sensor_ && bpm > 35.0f && bpm < 210.0f) {
              heart_rate_sensor_->publish_state(bpm);
              last_ibi_bpm_    = bpm;
              ibi_valid_beats_ = count;
              ESP_LOGD(TAG, "HR (IBI) avg_dt=%.1fms -> bpm=%.1f (n=%u)", avg_dt, bpm, count);
            }
          }
        }
      } else {
        last_beat_tick_ms_ = sample_tick_ms_; // seed
      }
    }

    // ---------------- SpO2 path ----------------
    if (++decim_count_ >= spo2_decim_) {
      decim_count_ = 0;
      if (spo2_count_ < spo2_win_) {
        ir_buf_[spo2_count_]  = ir_u;
        red_buf_[spo2_count_] = red_u;
        spo2_count_++;
      }

      if (spo2_count_ >= spo2_win_) {
        int32_t spo2=0, hr_algo=0; int8_t spo2_valid=0, hr_valid=0;
        maxim_heart_rate_and_oxygen_saturation(
          ir_buf_, (int32_t)spo2_win_,
          red_buf_,
          &spo2, &spo2_valid,
          &hr_algo, &hr_valid
        );

        bool settled = (spo2_windows_since_pub_ >= spo2_settle_windows_);
        if (!settled) {
          spo2_windows_since_pub_++;
          ESP_LOGD(TAG, "SpO2 settle window %u/%u (holding publication)",
                   (unsigned)spo2_windows_since_pub_, (unsigned)spo2_settle_windows_);
        }

        // --- HR-algo fallback if IBI is silent too long but algo has a good HR ---
        if (hr_valid && hr_algo > 40 && hr_algo < 200 && perf_ok) {
          uint32_t since_accept = (last_beat_tick_ms_ == 0)
                                  ? std::numeric_limits<uint32_t>::max()
                                  : (sample_tick_ms_ - last_beat_tick_ms_);
          if (since_accept >= hr_algo_fallback_timeout_ms_) {
            float bpm = (float)hr_algo;
            if (heart_rate_sensor_) {
              heart_rate_sensor_->publish_state(bpm);
            }
            // re-seed IBI history from algo
            uint32_t dt = (uint32_t)(60000.0f / bpm);
            rates_[rate_array_ % RATE_SIZE] = (int32_t) dt;
            rate_array_++;
            last_good_dt_ms_   = dt;
            last_beat_tick_ms_ = sample_tick_ms_;
            last_ibi_bpm_      = bpm;
            ibi_valid_beats_   = std::min<uint8_t>(ibi_valid_beats_ + 1, RATE_SIZE);
            ESP_LOGD(TAG, "HR fallback(algo): published %ld bpm and reseeded IBI", (long)hr_algo);
          }
        }

        // --- SpO2 publishing with step clamp + stuck release ---
        if (spo2_valid && spo2 > 70 && spo2 <= 100 && settled && spo2_sensor_) {
          float corrected = clampf((float)spo2 + spo2_offset_, 70.0f, spo2_max_pub_);

          // keep recent corrected windows
          if (spo2_recent_count_ < spo2_recent_n_) spo2_recent_count_++;
          spo2_recent_[spo2_recent_idx_ % spo2_recent_n_] = corrected;
          spo2_recent_idx_++;

          bool accept = false;
          if (std::isnan(last_spo2_)) {
            accept = true;
            spo2_windows_since_pub_ = 0;
          } else {
            float step = std::fabs(corrected - last_spo2_);
            if (step <= spo2_step_clamp_pct_) {
              accept = true;
              spo2_windows_since_pub_ = 0;
            } else {
              spo2_windows_since_pub_++;
              float med_recent = medianf(spo2_recent_, spo2_recent_count_);
              float med_step   = std::fabs(med_recent - last_spo2_);
              bool release = (spo2_windows_since_pub_ >= spo2_stuck_windows_) &&
                             (med_step >= spo2_jump_release_abs_);
              if (release) {
                accept = true;
                corrected = med_recent;
                ESP_LOGD(TAG, "SpO2 jump RELEASE (median): accepting %.1f after %u windows (Δ_med=%.1f%% ≥ %.1f%%)",
                         corrected, (unsigned)spo2_windows_since_pub_, med_step, (double)spo2_jump_release_abs_);
                spo2_windows_since_pub_ = 0;
              } else {
                ESP_LOGD(TAG, "SpO2 jump filtered: %.1f -> %.1f (Δ=%.1f%% > %.1f%%) [since_pub=%u]",
                         last_spo2_, corrected, step, (double)spo2_step_clamp_pct_,
                         (unsigned)spo2_windows_since_pub_);
              }
            }
          }

          if (accept) {
            spo2_sensor_->publish_state(corrected);
            last_spo2_ = corrected;
            ESP_LOGD(TAG, "SpO2 algo raw=%ld%% -> corrected=%.1f%% (valid=%d)", (long)spo2, corrected, (int)spo2_valid);
          }
        }

        // debug: how many raw beat triggers did we see in this ~2s window?
        if (raw_beats_in_window_ > 0) {
          ESP_LOGD(TAG, "HR raw triggers in last window: %u", (unsigned)raw_beats_in_window_);
        } else {
          ESP_LOGD(TAG, "HR raw triggers in last window: 0 (check LED current / thresholds)");
        }
        raw_beats_in_window_ = 0;

        // slide by 50
        const uint16_t half = spo2_win_ / 2;
        std::memmove(ir_buf_,  ir_buf_  + half, half * sizeof(uint32_t));
        std::memmove(red_buf_, red_buf_ + half, half * sizeof(uint32_t));
        spo2_count_ = half;
      }
    }
  }
}

void MAX30102NativeSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX30102 Native:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) ESP_LOGE(TAG, "Communication with MAX30102 failed!");
  LOG_SENSOR("  ", "Heart Rate", this->heart_rate_sensor_);
  LOG_SENSOR("  ", "SpO2", this->spo2_sensor_);

  ESP_LOGCONFIG(TAG, "  LED currents: RED=0x%02X, IR=0x%02X", led_red_pa_, led_ir_pa_);
  ESP_LOGCONFIG(TAG, "  FIFO avg=%u, SR=%u Hz", (unsigned)fifo_avg_sel_, (unsigned)sample_rate_hz_);
  ESP_LOGCONFIG(TAG, "  Finger gates: IR>=%lu, RED>=%lu", (unsigned long)finger_ir_min_, (unsigned long)finger_red_min_);
  ESP_LOGCONFIG(TAG, "  SpO2: offset=%.1f, max_pub=%.1f, step_clamp=±%.1f%%, settle=%u",
                (double)spo2_offset_, (double)spo2_max_pub_, (double)spo2_step_clamp_pct_, (unsigned)spo2_settle_windows_);
  ESP_LOGCONFIG(TAG, "        stuck-release: recent_n=%u, stuck_windows=%u, jump_abs=%.1f%%",
                (unsigned)spo2_recent_n_, (unsigned)spo2_stuck_windows_, (double)spo2_jump_release_abs_);
  ESP_LOGCONFIG(TAG, "  HR abs: [%ums..%ums], refractory=%ums, no_accept_timeout=%ums, algo_fallback=%ums",
                (unsigned)hr_min_dt_ms_, (unsigned)hr_max_dt_ms_,
                (unsigned)hr_refractory_ms_, (unsigned)hr_no_accept_timeout_ms_,
                (unsigned)hr_algo_fallback_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  HR gates: acq_med=[%.2fx..%.2fx], med=[%.2fx..%.2fx], last=[%.2fx..%.2fx], pair_tol=±%.0f%% slower_only=%s",
                (double)acq_med_low_, (double)acq_med_high_, (double)med_low_, (double)med_high_,
                (double)last_low_, (double)last_high_, (double)(hr_candidate_pair_tol_*100.0f),
                hr_pair_rescue_only_slower_ ? "yes":"no");
}

}  // namespace max30102_native
}  // namespace esphome
