#pragma once

#include <stdint.h>
#include "config.h"

namespace flightcore {

struct RpmDiagnostics {
    uint16_t rpm = 0;
    uint32_t total_pulses = 0;
    uint32_t rejected_glitches = 0;
    uint32_t last_period_us = 0;
    bool active = false;
};

/**
 * Pure-C++ debounced pulse-timing RPM calculator.
 * Hardware-independent so ISR edge cases and debounce behavior can be unit tested.
 */
class RpmCalculator {
public:
    explicit RpmCalculator(uint8_t pulses_per_rev = HALL_DEFAULT_PULSES_PER_REV,
                           uint32_t debounce_us = HALL_DEBOUNCE_US,
                           uint32_t timeout_ms = HALL_TIMEOUT_MS,
                           uint16_t max_rpm = HALL_RPM_MAX);

    void reset();
    void set_pulses_per_rev(uint8_t ppr);
    uint8_t pulses_per_rev() const { return ppr_; }

    /**
     * Records a pulse timestamp in microseconds. Returns true if accepted (not bounced).
     */
    bool on_pulse_us(uint64_t timestamp_us);

    /**
     * Updates timeout decay and returns current smoothed RPM.
     */
    uint16_t update(uint64_t now_us);

    const RpmDiagnostics& diagnostics() const { return diag_; }
    uint16_t rpm() const { return diag_.rpm; }
    bool is_active() const { return diag_.active; }

private:
    uint8_t ppr_;
    uint32_t debounce_us_;
    uint32_t timeout_ms_;
    uint16_t max_rpm_;

    bool has_last_pulse_;
    uint64_t last_pulse_us_;
    float ema_rpm_;
    RpmDiagnostics diag_;
};

/**
 * Hardware ESP32 GPIO Interrupt Hall-Effect RPM Driver (A3144 open-collector).
 */
class HallRpmDriver {
public:
    HallRpmDriver();

    bool begin(uint8_t pin = PIN_HALL_RPM,
               uint8_t pulses_per_rev = HALL_DEFAULT_PULSES_PER_REV);
    uint16_t poll();
    void set_pulses_per_rev(uint8_t ppr);

    uint16_t rpm() const { return calc_.rpm(); }
    bool is_active() const { return calc_.is_active(); }
    const RpmDiagnostics& diagnostics() const { return calc_.diagnostics(); }

private:
    uint8_t pin_;
    bool initialized_;
    RpmCalculator calc_;
};

}  // namespace flightcore
