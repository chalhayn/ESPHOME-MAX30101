#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <heartRate.h>        // SparkFun beat detector
#include <spo2_algorithm.h>   // Maxim/ProtoCentral SpO2

#include <cstring>
#include <cmath>
#include <algorithm>
#include <limits>
#include <math.h>

namespace esphome {
namespace max30102_native {

static const char *const TAG = "max30102_native";

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static float median_from_ibis(const int32_t *buf, uint8_t n) {
  float tmp[16];
  uint8_t m = 0;
  for (uint8_t i = 0; i < n; i++) if (buf[i] > 0) tmp[m++] = (float) buf[i];
  if (m == 0) return NAN;
  std::sort(tmp, tmp + m);
  return (m & 1) ? tmp[m/2] : 0.5f * (tmp[m/2 - 1] + tmp[m/2]);
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

  // Init state
  this->last_spo2_ = NAN;
  this->last_ibi_bpm_ = NAN;
  this->spo2_settle_windows_ = 0;

  this->sample_tick_ms_ = 0;
  this->last_beat_tick_ms_ = 0;
  this->last_good_dt_ms_ = 0;
  this->reject_streak_ = 0;
  this->reject_min_dt_ = std::numeric_limits<uint32_t>::max();
  this->reject_max_dt_ = 0;
  this->last_candidate_dt_ms_ = 0;

  ESP_LOGCONFIG(TAG, "MAX30102 Native initialized successfully");
}

bool MAX30102NativeSensor::initialize_sensor() {
  if (!this->write_byte(REG_MODE_CONFIG, 0x40)) return false;  // reset
  delay(100);

  if (!this->write_byte(REG_FIFO_CONFIG, FIFO_CFG_BYTE)) return false; // avg=4
  if (!this->write_byte(REG_MODE_CONFIG, 0x03)) return false;          // SpO2 mode
  if (!this->write_byte(REG_SPO2_CONFIG, 0x27)) return false;          // 4096nA, 100Hz, 411us
  if (!this->write_byte(REG_LED1_PA, 0x24)) return false;              // RED current
  if (!this->write_byte(REG_LED2_PA, 0x24)) return false;              // IR current

  this->clear_fifo();
  return true;
}

void MAX30102NativeSensor::clear_fifo() {
  this->write_byte(REG_FIFO_WR_PTR, 0);
  this->write_byte(REG_OVF_COUNTER, 0);
  this->write_byte(REG_FIFO_RD_PTR, 0);
}

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
bool MAX30102NativeSensor::check_for_beat(int32_t) { return false; }

void MAX30102NativeSensor::update() {
  uint8_t rd = 0, wr = 0;
  if (!this->read_byte(REG_FIFO_RD_PTR, &rd) || !this->read_byte(REG_FIFO_WR_PTR, &wr)) {
    ESP_LOGD(TAG, "FIFO pointer read failed"); return;
  }
  uint8_t samples = (wr - rd) & 0x1F;
  if (samples < 2) samples = 2;

  for (uint8_t i = 0; i + 1 < samples; i += 2) {
    uint8_t data[6];
    if (!this->read_bytes(REG_FIFO_DATA, data, 6)) {
      ESP_LOGD(TAG, "FIFO read failed"); break;
    }

    uint32_t red_u = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    uint32_t ir_u  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    red_u &= 0x3FFFF; ir_u &= 0x3FFFF;

    // advance synthetic sample clock (per IR+RED pair)
    this->sample_tick_ms_ += SAMPLE_PERIOD_MS;

    // finger-present gating
    if (ir_u < FINGER_IR_MIN || red_u < FINGER_RED_MIN) {
      // this->spo2_settle_windows_ = 0; // optional
      continue;
    }

    // ----- HR (sample-clocked) -----
    if (checkForBeat((int32_t) ir_u)) {
      const uint32_t dt = (this->last_beat_tick_ms_ == 0)
                            ? 0
                            : (this->sample_tick_ms_ - this->last_beat_tick_ms_);

      const uint32_t since_accept = (this->last_beat_tick_ms_ == 0)
                                      ? 0
                                      : (this->sample_tick_ms_ - this->last_beat_tick_ms_);

      // Track candidate consistency (vs previous candidate dt)
      bool candidate_pair_ok = false;
      if (this->last_candidate_dt_ms_ > 0 && dt > 0) {
        const uint32_t hi = std::max(dt, this->last_candidate_dt_ms_);
        const uint32_t lo = std::min(dt, this->last_candidate_dt_ms_);
        candidate_pair_ok = ( (float)hi / (float)lo <= (1.0f + HR_CANDIDATE_PAIR_TOL) );
      }
      this->last_candidate_dt_ms_ = (dt > 0) ? dt : this->last_candidate_dt_ms_;

      // Only evaluate dt if we have a previous accept; otherwise just seed timestamp below
      if (this->last_beat_tick_ms_ != 0) {
        bool accept = false;

        // 0) Absolute plausibility
        const bool plausible = (dt >= HR_MIN_DT_MS && dt <= HR_MAX_DT_MS);

        // **A) Timeout rescue:** if no accept for a while, take the next plausible beat
        if (!accept && plausible && since_accept >= HR_NO_ACCEPT_TIMEOUT_MS) {
          accept = true;
          ESP_LOGD(TAG, "HR rescue(timeout): accepting dt=%lu ms after %lu ms without accept",
                   (unsigned long)dt, (unsigned long)since_accept);
        }

        // **B) Two-consistent-candidates rescue**
        if (!accept && plausible && candidate_pair_ok) {
          accept = true;
          ESP_LOGD(TAG, "HR rescue(pair): accepting dt=%lu ms (consistent with previous candidate)",
                   (unsigned long)dt);
        }

        // **C) Normal gating (acq/stable vs median/last)**
        if (!accept && plausible) {
          const float med = median_from_ibis(this->rates_, RATE_SIZE);
          const bool have_med  = !std::isnan(med) && med > 0.0f;
          const bool have_last = (this->last_good_dt_ms_ > 0);
          const bool in_acq = (this->ibi_valid_beats_ < HR_IBI_MIN_BEATS);

          bool ok_med = false;
          if (have_med) {
            const float low  = (in_acq ? ACQ_MED_LOW : MED_LOW) * med;
            const float high = (in_acq ? ACQ_MED_HIGH : MED_HIGH) * med;
            ok_med = (dt >= (uint32_t) low && dt <= (uint32_t) high);
          }

          bool ok_last = false;
          if (have_last) {
            const float low  = (in_acq ? ACQ_MED_LOW : LAST_LOW) * (float) this->last_good_dt_ms_;
            const float high = (in_acq ? ACQ_MED_HIGH : LAST_HIGH) * (float) this->last_good_dt_ms_;
            ok_last = (dt >= (uint32_t) low && dt <= (uint32_t) high);
          }

          accept = (!have_med && !have_last) || ok_med || ok_last;

          if (!accept) {
            // streak-based resync if rejects are “consistent”
            this->reject_streak_++;
            this->reject_min_dt_ = std::min(this->reject_min_dt_, dt);
            this->reject_max_dt_ = std::max(this->reject_max_dt_, dt);

            const bool ready = (this->reject_streak_ >= REJECT_STREAK_FOR_RESYNC);
            const bool consistent = (this->reject_min_dt_ > 0 &&
              (float)this->reject_max_dt_ / (float)this->reject_min_dt_ <= REJECT_STREAK_SPREAD_MAX);

            if (ready && consistent) {
              accept = true;
              ESP_LOGD(TAG, "HR rescue(streak): accepting dt=%lu ms after %u consistent rejects "
                            "(range %lu..%lu ms)",
                       (unsigned long) dt, (unsigned) this->reject_streak_,
                       (unsigned long) this->reject_min_dt_, (unsigned long) this->reject_max_dt_);
            } else {
              // diagnostic
              const float med_use = have_med ? med : 0.0f;
              const float lmed = (in_acq ? ACQ_MED_LOW : MED_LOW) * med_use;
              const float hmed = (in_acq ? ACQ_MED_HIGH : MED_HIGH) * med_use;
              const float llast = (in_acq ? ACQ_MED_LOW : LAST_LOW) * (float)this->last_good_dt_ms_;
              const float hlast = (in_acq ? ACQ_MED_HIGH : LAST_HIGH) * (float)this->last_good_dt_ms_;
              ESP_LOGD(TAG, "IBI rejected: dt=%lu ms (med=%.1f, med_range=%.1f..%.1f ms, "
                            "last=%lu ms, last_range=%.1f..%.1f ms, acq=%d)",
                       (unsigned long) dt, (double) med_use,
                       (double) lmed, (double) hmed,
                       (unsigned long) this->last_good_dt_ms_,
                       (double) llast, (double) hlast,
                       (int) in_acq);
            }
          } else {
            // reset streak on acceptance
            this->reject_streak_ = 0;
            this->reject_min_dt_ = std::numeric_limits<uint32_t>::max();
            this->reject_max_dt_ = 0;
          }
        }

        if (accept) {
          // commit beat
          this->rates_[this->rate_array_ % RATE_SIZE] = (int32_t) dt;
          this->rate_array_++;
          this->last_good_dt_ms_  = dt;
          this->last_beat_tick_ms_ = this->sample_tick_ms_;

          uint32_t sum_dt = 0; uint8_t count = 0;
          for (uint8_t k = 0; k < RATE_SIZE; k++) {
            int32_t ibi = this->rates_[k];
            if (ibi > 0) { sum_dt += (uint32_t) ibi; count++; }
          }
          if (count >= 2) {
            float avg_dt = (float) sum_dt / (float) count;
            float bpm = 60000.0f / avg_dt;
            if (this->heart_rate_sensor_ && bpm > 40.0f && bpm < 200.0f) {
              this->heart_rate_sensor_->publish_state(bpm);
              this->last_ibi_bpm_    = bpm;
              this->ibi_valid_beats_ = count;
              ESP_LOGD(TAG, "HR (IBI) avg_dt=%.1fms -> bpm=%.1f (n=%u)", avg_dt, bpm, count);
            }
          }
        }
      } else {
        // first accepted beat will timestamp here
        this->last_beat_tick_ms_ = this->sample_tick_ms_;
      }
    }

    // ----- SpO₂ -----
    this->dc_ir_  = (this->dc_ir_  == 0.0f) ? (float) ir_u  : DC_ALPHA * this->dc_ir_  + (1.0f - DC_ALPHA) * (float) ir_u;
    this->dc_red_ = (this->dc_red_ == 0.0f) ? (float) red_u : DC_ALPHA * this->dc_red_ + (1.0f - DC_ALPHA) * (float) red_u;

    float ac_ir  = std::fabs((float) ir_u  - this->dc_ir_);
    float ac_red = std::fabs((float) red_u - this->dc_red_);
    float pi_ir  = (this->dc_ir_  > 1.0f) ? (ac_ir  / this->dc_ir_ ) * 100.0f : 0.0f;
    float pi_red = (this->dc_red_ > 1.0f) ? (ac_red / this->dc_red_) * 100.0f : 0.0f;
    (void)pi_ir; (void)pi_red; // available for debug if desired

    if (++this->decim_count_ >= SPO2_DECIM) {
      this->decim_count_ = 0;
      if (this->spo2_count_ < SPO2_WIN) {
        this->ir_buf_[this->spo2_count_]  = ir_u;
        this->red_buf_[this->spo2_count_] = red_u;
        this->spo2_count_++;
      }
      if (this->spo2_count_ >= SPO2_WIN) {
        int32_t spo2 = 0, hr_algo = 0;
        int8_t  spo2_valid = 0, hr_valid = 0;

        maxim_heart_rate_and_oxygen_saturation(
          this->ir_buf_, (int32_t) SPO2_WIN,
          this->red_buf_,
          &spo2, &spo2_valid,
          &hr_algo, &hr_valid
        );

        bool settled = (this->spo2_settle_windows_ >= SPO2_SETTLE_WINDOWS);
        if (!settled) {
          this->spo2_settle_windows_++;
          ESP_LOGD(TAG, "SpO2 settle window %u/%u (holding publication)",
                   this->spo2_settle_windows_, (unsigned)SPO2_SETTLE_WINDOWS);
        }

        if (spo2_valid && spo2 > 70 && spo2 <= 100 && settled && this->spo2_sensor_) {
          float corrected = clampf((float)spo2 + SPO2_OFFSET, 70.0f, SPO2_MAX_PUBLISH);
          bool accept = false;

          if (std::isnan(this->last_spo2_)) {
            accept = true;
          } else {
            float step = std::fabs(corrected - this->last_spo2_);
            accept = (step <= SPO2_JUMP_MAX_PCT);
            if (!accept) {
              ESP_LOGD(TAG, "SpO2 jump filtered: %.1f -> %.1f (Δ=%.1f%% > %.1f%%)",
                       this->last_spo2_, corrected, step, (double)SPO2_JUMP_MAX_PCT);
            }
          }

          if (accept) {
            this->spo2_sensor_->publish_state(corrected);
            this->last_spo2_ = corrected;
            ESP_LOGD(TAG, "SpO2 algo raw=%ld%% -> corrected=%.1f%% (valid=%d)",
                     (long) spo2, corrected, (int) spo2_valid);
          }
        }

        // optional HR_algo publishing (kept disabled by default)
        if (PUBLISH_HR_ALGO && hr_valid && hr_algo > 40 && hr_algo < 200 && this->heart_rate_sensor_) {
          if (!std::isnan(this->last_ibi_bpm_) && this->ibi_valid_beats_ >= HR_IBI_MIN_BEATS) {
            float diff = std::fabs((float) hr_algo - this->last_ibi_bpm_);
            if (diff <= HR_ALGO_IBI_DIFF_TIGHT) {
              this->heart_rate_sensor_->publish_state((float) hr_algo);
              ESP_LOGD(TAG, "HR algo: %ld bpm (valid=%d, Δ=%.1f to IBI)", (long) hr_algo, (int) hr_valid, diff);
            } else {
              ESP_LOGD(TAG, "HR algo gated: %ld vs IBI %.1f (Δ=%.1f > %.1f)",
                       (long) hr_algo, this->last_ibi_bpm_, diff, (double)HR_ALGO_IBI_DIFF_TIGHT);
            }
          }
        }

        // slide window by 50
        const uint16_t half = SPO2_WIN / 2;
        std::memmove(this->ir_buf_,  this->ir_buf_  + half, half * sizeof(uint32_t));
        std::memmove(this->red_buf_, this->red_buf_ + half, half * sizeof(uint32_t));
        this->spo2_count_ = half;
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

  ESP_LOGCONFIG(TAG, "  FIFO avg: 4 samples, sample clock: %lu ms (100 Hz)", (unsigned long)SAMPLE_PERIOD_MS);
  ESP_LOGCONFIG(TAG, "  SpO2 settle windows: %u, step clamp: ±%.1f%%, offset: %.1f%%, max publish: %.1f%%",
                (unsigned)SPO2_SETTLE_WINDOWS, (double)SPO2_JUMP_MAX_PCT,
                (double)SPO2_OFFSET, (double)SPO2_MAX_PUBLISH);
  ESP_LOGCONFIG(TAG, "  HR IBI acceptance: abs=[%ums..%ums], acq_med=[%.2fx..%.2fx], "
                     "med=[%.2fx..%.2fx], last=[%.2fx..%.2fx]",
                (unsigned)HR_MIN_DT_MS, (unsigned)HR_MAX_DT_MS,
                (double)ACQ_MED_LOW, (double)ACQ_MED_HIGH,
                (double)MED_LOW, (double)MED_HIGH,
                (double)LAST_LOW, (double)LAST_HIGH);
  ESP_LOGCONFIG(TAG, "  HR rescues: timeout=%lu ms, pair_tol=±%.0f%%, streak=%u (spread≤%.2fx)",
                (unsigned long)HR_NO_ACCEPT_TIMEOUT_MS, (double)(HR_CANDIDATE_PAIR_TOL*100.0f),
                (unsigned)REJECT_STREAK_FOR_RESYNC, (double)REJECT_STREAK_SPREAD_MAX);
}

}  // namespace max30102_native
}  // namespace esphome
