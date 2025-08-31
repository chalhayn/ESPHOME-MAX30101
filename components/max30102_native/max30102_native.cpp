#include "max30102_native.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

// Keep C-style algo headers OUTSIDE any namespace.
#include <heartRate.h>        // SparkFun MAX3010x PBA beat detector
#include <spo2_algorithm.h>   // Maxim/ProtoCentral SpO2 algorithm

#include <cstring>            // std::memmove
#include <cmath>              // std::isnan, std::fabs
#include <math.h>             // NAN (ensure macro exists)

namespace esphome {
namespace max30102_native {

static const char *const TAG = "max30102_native";

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
  if (!this->read_byte(REG_FIFO_RD_PTR, &rd) || !this->read_byte(REG_FIFO_WR_PTR, &wr)) {
    ESP_LOGD(TAG, "FIFO pointer read failed");
    return;
  }
  uint8_t samples = (wr - rd) & 0x1F;  // 32-deep FIFO, modulo 32
  if (samples < 2) samples = 2;        // ensure at least one RED+IR pair

  // Process in RED+IR pairs (6 bytes per pair)
  for (uint8_t i = 0; i + 1 < samples; i += 2) {
    uint8_t data[6];
    if (!this->read_bytes(REG_FIFO_DATA, data, 6)) {
      ESP_LOGD(TAG, "FIFO read failed");
      break;
    }

    // Unpack 18-bit values and mask defensively
    uint32_t red_u = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    uint32_t ir_u  = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    red_u &= 0x3FFFF;
    ir_u  &= 0x3FFFF;

    // Gentle finger-present gates (skip pair entirely if no contact)
    if (ir_u < FINGER_IR_MIN || red_u < FINGER_RED_MIN) {
      // Optional: reset settle timer on loss of contact
      // this->spo2_settle_windows_ = 0;
      continue;
    }

    // ---- Heart Rate via SparkFun detector on IR channel (always attempt) ----
    if (checkForBeat((int32_t) ir_u)) {
      uint32_t now = millis();
      if (this->last_beat_ != 0) {
        uint32_t dt = now - this->last_beat_;  // inter-beat interval (ms)
        if (dt > 300 && dt < 3000) {           // ~20–200 BPM
          // Store IBI (ms) into ring buffer
          this->rates_[this->rate_array_ % RATE_SIZE] = (int32_t) dt;
          this->rate_array_++;

          // Compute BPM from mean IBI (smoother)
          uint32_t sum_dt = 0;
          uint8_t count = 0;
          for (uint8_t k = 0; k < RATE_SIZE; k++) {
            int32_t ibi = this->rates_[k];
            if (ibi > 0) { sum_dt += (uint32_t) ibi; count++; }
          }
          if (count >= 2) {
            float avg_dt = (float) sum_dt / (float) count;   // ms
            float bpm    = 60000.0f / avg_dt;                // bpm
            if (this->heart_rate_sensor_ && bpm > 40.0f && bpm < 200.0f) {
              this->heart_rate_sensor_->publish_state(bpm);
              this->last_ibi_bpm_    = bpm;
              this->ibi_valid_beats_ = count;  // NEW: track usable IBI beats in average
              ESP_LOGD(TAG, "HR (IBI) avg_dt=%.1fms -> bpm=%.1f (n=%u)", avg_dt, bpm, count);
            }
          }
        }
      }
      this->last_beat_ = now;
    }

    // ---- SpO₂ via spo2_algorithm.h ----
    // Track DC baselines and perfusion index (AC/DC%)
    this->dc_ir_  = (this->dc_ir_  == 0.0f) ? (float) ir_u  : DC_ALPHA * this->dc_ir_  + (1.0f - DC_ALPHA) * (float) ir_u;
    this->dc_red_ = (this->dc_red_ == 0.0f) ? (float) red_u : DC_ALPHA * this->dc_red_ + (1.0f - DC_ALPHA) * (float) red_u;

    float ac_ir   = std::fabs((float) ir_u  - this->dc_ir_);
    float ac_red  = std::fabs((float) red_u - this->dc_red_);
    float pi_ir   = (this->dc_ir_  > 1.0f) ? (ac_ir  / this->dc_ir_ ) * 100.0f : 0.0f;
    float pi_red  = (this->dc_red_ > 1.0f) ? (ac_red / this->dc_red_) * 100.0f : 0.0f;

    const bool perf_ok = (pi_ir >= PERF_MIN_PCT && pi_ir <= PERF_MAX_PCT) &&
                         (pi_red >= PERF_MIN_PCT && pi_red <= PERF_MAX_PCT);

    // Optional verbose line to help debug perfusion:
    ESP_LOGV(TAG, "PI%% ir=%.2f red=%.2f perf_ok=%d spo2_count=%u",
             (double)pi_ir, (double)pi_red, perf_ok, (unsigned)this->spo2_count_);

    // Decimate 100 Hz -> 25 Hz; ALWAYS fill the SpO₂ window so the algorithm runs.
    if (++this->decim_count_ >= SPO2_DECIM) {
      this->decim_count_ = 0;

      if (this->spo2_count_ < SPO2_WIN) {
        this->ir_buf_[this->spo2_count_]  = ir_u;   // uint32_t buffers
        this->red_buf_[this->spo2_count_] = red_u;
        this->spo2_count_++;
      }

      // Run algorithm when the window fills
      if (this->spo2_count_ >= SPO2_WIN) {
        int32_t spo2 = 0, hr_algo = 0;
        int8_t  spo2_valid = 0, hr_valid = 0;

        maxim_heart_rate_and_oxygen_saturation(
          this->ir_buf_, (int32_t) SPO2_WIN,
          this->red_buf_,
          &spo2, &spo2_valid,
          &hr_algo, &hr_valid
        );

        // Progress the settle timer once per processed window (even if invalid)
        bool settled = (this->spo2_settle_windows_ >= SPO2_SETTLE_WINDOWS);
        if (!settled) {
          this->spo2_settle_windows_++;
          ESP_LOGD(TAG, "SpO2 settle window %u/%u (holding publication)",
                   this->spo2_settle_windows_, (unsigned)SPO2_SETTLE_WINDOWS);
        }

        // ---- Publish SpO₂ with stability guards (only when settled & plausible) ----
        if (spo2_valid && spo2 > 70 && spo2 <= 100 && settled) {
          bool accept = false;
          if (std::isnan(this->last_spo2_)) {
            accept = true;
          } else {
            float step = std::fabs((float) spo2 - this->last_spo2_);
            accept = (step <= SPO2_JUMP_MAX_PCT);
            if (!accept) {
              ESP_LOGD(TAG, "SpO2 jump filtered: %.1f -> %ld (Δ=%.1f%% > %.1f%%)",
                       this->last_spo2_, (long) spo2, step, (double)SPO2_JUMP_MAX_PCT);
            }
          }
          if (accept && this->spo2_sensor_) {
            this->spo2_sensor_->publish_state((float) spo2);
            this->last_spo2_ = (float) spo2;
            ESP_LOGD(TAG, "SpO2 algo: %ld%% (valid=%d)", (long) spo2, (int) spo2_valid);
          }
        }

        // ---- Prefer IBI; only publish HR_algo if enabled AND it agrees tightly AFTER IBI ----
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
          } else {
            // No reliable IBI yet -> do NOT publish HR_algo (prevents early wild spikes)
            ESP_LOGV(TAG, "HR algo suppressed: IBI not established (beats=%u, last_ibi=%.1f)",
                     (unsigned)this->ibi_valid_beats_, this->last_ibi_bpm_);
          }
        }

        // Slide the window by half (50 samples) to keep outputs responsive
        const uint16_t half = SPO2_WIN / 2;  // 50
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
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX30102 failed!");
  }
  LOG_SENSOR("  ", "Heart Rate", this->heart_rate_sensor_);
  LOG_SENSOR("  ", "SpO2", this->spo2_sensor_);

  ESP_LOGCONFIG(TAG, "  FIFO avg: 8 samples");
  ESP_LOGCONFIG(TAG, "  SpO2 settle windows: %u", (unsigned) SPO2_SETTLE_WINDOWS);
  ESP_LOGCONFIG(TAG, "  SpO2 step clamp: ±%.1f%%", (double) SPO2_JUMP_MAX_PCT);
  ESP_LOGCONFIG(TAG, "  HR algo publish: %s (min IBI beats=%u, tight diff=%.1f bpm)",
                PUBLISH_HR_ALGO ? "ENABLED" : "DISABLED",
                (unsigned)HR_IBI_MIN_BEATS, (double)HR_ALGO_IBI_DIFF_TIGHT);
}

}  // namespace max30102_native
}  // namespace esphome
