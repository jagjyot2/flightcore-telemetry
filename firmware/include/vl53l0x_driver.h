#pragma once

#include <stdint.h>
#include "config.h"

namespace flightcore {

struct TofDiagnostics {
    uint32_t total_samples = 0;
    uint32_t valid_samples = 0;
    uint32_t glitch_rejected_count = 0;
    uint32_t out_of_range_count = 0;
    uint32_t timeout_count = 0;
    uint32_t i2c_error_count = 0;
    uint16_t last_raw_mm = TOF_INVALID_SENTINEL;
    uint16_t median_mm = TOF_INVALID_SENTINEL;
    uint16_t filtered_mm = TOF_INVALID_SENTINEL;
    bool healthy = false;
    bool range_valid = false;
};

/**
 * Pure-C++ 5-tap median + EMA glitch filter and diagnostic tracker for VL53L0X ToF.
 */
class TofGlitchFilter {
public:
    explicit TofGlitchFilter(float ema_alpha = TOF_EMA_ALPHA,
                             uint16_t min_valid_mm = TOF_MIN_VALID_MM,
                             uint16_t max_valid_mm = TOF_MAX_VALID_MM);

    void reset();

    /**
     * Ingests a raw range sample (in mm) and validity flag (false on timeout/status error).
     * Returns filtered range in mm, or TOF_INVALID_SENTINEL (0xFFFF) when invalid.
     */
    uint16_t update(uint16_t raw_mm, bool measurement_ok);

    void record_timeout();
    void record_i2c_error();
    void set_healthy(bool healthy) { diag_.healthy = healthy; }

    const TofDiagnostics& diagnostics() const { return diag_; }
    uint16_t filtered_mm() const { return diag_.filtered_mm; }
    bool is_range_valid() const { return diag_.range_valid; }

private:
    float ema_alpha_;
    uint16_t min_valid_mm_;
    uint16_t max_valid_mm_;

    uint16_t window_[TOF_MEDIAN_WINDOW];
    uint8_t window_idx_;
    uint8_t window_count_;
    bool ema_seeded_;
    float ema_value_;
    uint8_t invalid_streak_;
    TofDiagnostics diag_;

    uint16_t compute_median() const;
};

/**
 * Hardware I2C VL53L0X ToF Ranging Driver (register-level, non-blocking).
 */
class Vl53l0xDriver {
public:
    Vl53l0xDriver();

    bool begin(uint8_t i2c_addr = VL53L0X_I2C_ADDR,
               uint8_t xshut_pin = PIN_VL53L0X_XSHUT);
    bool poll();
    bool hardware_reset();

    uint16_t distance_mm() const { return filter_.filtered_mm(); }
    bool is_healthy() const { return filter_.diagnostics().healthy; }
    bool is_range_valid() const { return filter_.is_range_valid(); }
    const TofDiagnostics& diagnostics() const { return filter_.diagnostics(); }
    uint8_t model_id() const { return model_id_; }

private:
    uint8_t addr_;
    uint8_t xshut_pin_;
    uint8_t model_id_;
    uint8_t stop_variable_;
    bool ranging_active_;
    uint32_t range_start_ms_;
    TofGlitchFilter filter_;

    bool write_reg8(uint8_t reg, uint8_t val);
    bool read_reg8(uint8_t reg, uint8_t& val);
    bool read_reg16(uint8_t reg, uint16_t& val);
    bool init_sensor_registers();
    bool start_single_range();
};

}  // namespace flightcore
