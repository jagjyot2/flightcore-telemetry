#include "vl53l0x_driver.h"
#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#endif

namespace flightcore {

namespace {

constexpr uint8_t REG_SYSRANGE_START = 0x00;
constexpr uint8_t REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
constexpr uint8_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0B;
constexpr uint8_t REG_RESULT_INTERRUPT_STATUS = 0x13;
constexpr uint8_t REG_RESULT_RANGE_STATUS = 0x14;
constexpr uint8_t REG_I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
constexpr uint8_t REG_VHV_CONFIG_PAD_SCL_SDA = 0x89;
constexpr uint8_t REG_IDENTIFICATION_MODEL_ID = 0xC0;
constexpr uint8_t EXPECTED_MODEL_ID = 0xEE;

}  // namespace

TofGlitchFilter::TofGlitchFilter(float ema_alpha,
                                 uint16_t min_valid_mm,
                                 uint16_t max_valid_mm)
    : ema_alpha_(ema_alpha),
      min_valid_mm_(min_valid_mm),
      max_valid_mm_(max_valid_mm) {
    reset();
}

void TofGlitchFilter::reset() {
    memset(window_, 0, sizeof(window_));
    window_idx_ = 0;
    window_count_ = 0;
    ema_seeded_ = false;
    ema_value_ = 0.0f;
    invalid_streak_ = 0;
    diag_ = TofDiagnostics{};
}

uint16_t TofGlitchFilter::compute_median() const {
    if (window_count_ == 0) {
        return TOF_INVALID_SENTINEL;
    }
    uint16_t sorted[TOF_MEDIAN_WINDOW] = {0};
    for (uint8_t i = 0; i < window_count_; ++i) {
        sorted[i] = window_[i];
    }
    for (uint8_t i = 1; i < window_count_; ++i) {
        const uint16_t key = sorted[i];
        int j = static_cast<int>(i) - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = key;
    }
    return sorted[window_count_ / 2U];
}

uint16_t TofGlitchFilter::update(uint16_t raw_mm, bool measurement_ok) {
    ++diag_.total_samples;
    diag_.last_raw_mm = raw_mm;

    const bool in_bounds =
        measurement_ok && (raw_mm >= min_valid_mm_) && (raw_mm <= max_valid_mm_);

    if (!in_bounds) {
        ++diag_.out_of_range_count;
        ++invalid_streak_;
        if (invalid_streak_ >= 3) {
            diag_.range_valid = false;
            diag_.filtered_mm = TOF_INVALID_SENTINEL;
            ema_seeded_ = false;
        }
        return diag_.filtered_mm;
    }

    invalid_streak_ = 0;
    ++diag_.valid_samples;

    // Detect isolated single-sample spike (> 350 mm jump relative to settled EMA)
    if (ema_seeded_) {
        const float delta = fabsf(static_cast<float>(raw_mm) - ema_value_);
        if (delta > 350.0f) {
            ++diag_.glitch_rejected_count;
        }
    }

    window_[window_idx_] = raw_mm;
    window_idx_ = static_cast<uint8_t>((window_idx_ + 1U) % TOF_MEDIAN_WINDOW);
    if (window_count_ < TOF_MEDIAN_WINDOW) {
        ++window_count_;
    }

    const uint16_t median = compute_median();
    diag_.median_mm = median;

    if (!ema_seeded_) {
        ema_value_ = static_cast<float>(median);
        ema_seeded_ = true;
    } else {
        ema_value_ = ema_alpha_ * static_cast<float>(median) +
                     (1.0f - ema_alpha_) * ema_value_;
    }

    diag_.filtered_mm = static_cast<uint16_t>(lroundf(ema_value_));
    diag_.range_valid = true;
    return diag_.filtered_mm;
}

void TofGlitchFilter::record_timeout() {
    ++diag_.timeout_count;
    update(TOF_INVALID_SENTINEL, false);
}

void TofGlitchFilter::record_i2c_error() {
    ++diag_.i2c_error_count;
}

// ---------------------------------------------------------------------------
// Hardware ESP32 I2C Driver
// ---------------------------------------------------------------------------
Vl53l0xDriver::Vl53l0xDriver()
    : addr_(VL53L0X_I2C_ADDR),
      xshut_pin_(PIN_VL53L0X_XSHUT),
      model_id_(0),
      stop_variable_(0),
      ranging_active_(false),
      range_start_ms_(0),
      filter_(TOF_EMA_ALPHA, TOF_MIN_VALID_MM, TOF_MAX_VALID_MM) {}

bool Vl53l0xDriver::write_reg8(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0) {
        filter_.record_i2c_error();
        return false;
    }
    return true;
#else
    (void)reg;
    (void)val;
    return false;
#endif
}

bool Vl53l0xDriver::read_reg8(uint8_t reg, uint8_t& val) {
#ifdef ARDUINO
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        filter_.record_i2c_error();
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(addr_), 1) != 1) {
        filter_.record_i2c_error();
        return false;
    }
    val = static_cast<uint8_t>(Wire.read());
    return true;
#else
    (void)reg;
    (void)val;
    return false;
#endif
}

bool Vl53l0xDriver::read_reg16(uint8_t reg, uint16_t& val) {
#ifdef ARDUINO
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        filter_.record_i2c_error();
        return false;
    }
    if (Wire.requestFrom(static_cast<int>(addr_), 2) != 2) {
        filter_.record_i2c_error();
        return false;
    }
    const uint8_t hi = static_cast<uint8_t>(Wire.read());
    const uint8_t lo = static_cast<uint8_t>(Wire.read());
    val = static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    return true;
#else
    (void)reg;
    (void)val;
    return false;
#endif
}

bool Vl53l0xDriver::hardware_reset() {
#ifdef ARDUINO
    pinMode(xshut_pin_, OUTPUT);
    digitalWrite(xshut_pin_, LOW);
    delay(10);
    digitalWrite(xshut_pin_, HIGH);
    delay(10);
    return true;
#else
    return false;
#endif
}

bool Vl53l0xDriver::init_sensor_registers() {
    // Standard ST VL53L0X 2.8V I/O + stop variable initialization sequence
    uint8_t pad = 0;
    if (!read_reg8(REG_VHV_CONFIG_PAD_SCL_SDA, pad)) {
        return false;
    }
    if (!write_reg8(REG_VHV_CONFIG_PAD_SCL_SDA, static_cast<uint8_t>(pad | 0x01U))) return false;
    if (!write_reg8(0x88, 0x00)) return false;

    if (!write_reg8(0x80, 0x01)) return false;
    if (!write_reg8(0xFF, 0x01)) return false;
    if (!write_reg8(0x00, 0x00)) return false;
    if (!read_reg8(0x91, stop_variable_)) return false;
    if (!write_reg8(0x00, 0x01)) return false;
    if (!write_reg8(0xFF, 0x00)) return false;
    if (!write_reg8(0x80, 0x00)) return false;

    // Configure new-sample-ready interrupt on GPIO1 (active low)
    if (!write_reg8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)) return false;
    uint8_t gpio_hv = 0;
    if (!read_reg8(0x84, gpio_hv)) return false;
    if (!write_reg8(0x84, static_cast<uint8_t>(gpio_hv & ~0x10U))) return false;
    if (!write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) return false;

    return true;
}

bool Vl53l0xDriver::start_single_range() {
    if (!write_reg8(0x80, 0x01)) return false;
    if (!write_reg8(0xFF, 0x01)) return false;
    if (!write_reg8(0x00, 0x00)) return false;
    if (!write_reg8(0x91, stop_variable_)) return false;
    if (!write_reg8(0x00, 0x01)) return false;
    if (!write_reg8(0xFF, 0x00)) return false;
    if (!write_reg8(0x80, 0x00)) return false;
    if (!write_reg8(REG_SYSRANGE_START, 0x01)) return false;
    return true;
}

bool Vl53l0xDriver::begin(uint8_t i2c_addr, uint8_t xshut_pin) {
    addr_ = i2c_addr;
    xshut_pin_ = xshut_pin;
    ranging_active_ = false;
    filter_.reset();

#ifdef ARDUINO
    hardware_reset();
    if (!read_reg8(REG_IDENTIFICATION_MODEL_ID, model_id_)) {
        filter_.set_healthy(false);
        return false;
    }
    if (model_id_ != EXPECTED_MODEL_ID) {
        filter_.set_healthy(false);
        return false;
    }
    if (!init_sensor_registers()) {
        filter_.set_healthy(false);
        return false;
    }
    filter_.set_healthy(true);
    if (start_single_range()) {
        ranging_active_ = true;
        range_start_ms_ = millis();
    }
    return true;
#else
    return false;
#endif
}

bool Vl53l0xDriver::poll() {
#ifdef ARDUINO
    if (!filter_.diagnostics().healthy) {
        return false;
    }

    const uint32_t now = millis();
    if (!ranging_active_) {
        if (start_single_range()) {
            ranging_active_ = true;
            range_start_ms_ = now;
        }
        return false;
    }

    uint8_t irq_status = 0;
    if (!read_reg8(REG_RESULT_INTERRUPT_STATUS, irq_status)) {
        return false;
    }

    if ((irq_status & 0x07U) == 0U) {
        if ((now - range_start_ms_) > TOF_READ_TIMEOUT_MS) {
            filter_.record_timeout();
            write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
            ranging_active_ = start_single_range();
            range_start_ms_ = now;
        }
        return false;
    }

    uint8_t range_status = 0;
    uint16_t raw_mm = TOF_INVALID_SENTINEL;
    const bool ok_stat = read_reg8(REG_RESULT_RANGE_STATUS, range_status);
    const bool ok_dist = read_reg16(static_cast<uint8_t>(REG_RESULT_RANGE_STATUS + 10U), raw_mm);
    write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    // Bits [6:3] encode internal device range status; 11 == Range Valid
    const uint8_t dev_status = static_cast<uint8_t>((range_status >> 3) & 0x0FU);
    const bool valid = ok_stat && ok_dist && (dev_status == 11U || dev_status == 0U);
    filter_.update(raw_mm, valid);

    ranging_active_ = start_single_range();
    range_start_ms_ = now;
    return true;
#else
    return false;
#endif
}

}  // namespace flightcore
