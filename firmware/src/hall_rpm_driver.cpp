#include "hall_rpm_driver.h"
#include <math.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <esp_timer.h>
#endif

namespace flightcore {

RpmCalculator::RpmCalculator(uint8_t pulses_per_rev,
                             uint32_t debounce_us,
                             uint32_t timeout_ms,
                             uint16_t max_rpm)
    : ppr_(pulses_per_rev == 0 ? 1 : pulses_per_rev),
      debounce_us_(debounce_us),
      timeout_ms_(timeout_ms),
      max_rpm_(max_rpm),
      has_last_pulse_(false),
      last_pulse_us_(0),
      ema_rpm_(0.0f),
      diag_() {}

void RpmCalculator::reset() {
    has_last_pulse_ = false;
    last_pulse_us_ = 0;
    ema_rpm_ = 0.0f;
    diag_ = RpmDiagnostics{};
}

void RpmCalculator::set_pulses_per_rev(uint8_t ppr) {
    ppr_ = (ppr == 0) ? 1 : ppr;
}

bool RpmCalculator::on_pulse_us(uint64_t timestamp_us) {
    if (!has_last_pulse_) {
        has_last_pulse_ = true;
        last_pulse_us_ = timestamp_us;
        ++diag_.total_pulses;
        return true;
    }

    if (timestamp_us <= last_pulse_us_) {
        ++diag_.rejected_glitches;
        return false;
    }

    const uint64_t dt_us = timestamp_us - last_pulse_us_;
    if (dt_us < static_cast<uint64_t>(debounce_us_)) {
        ++diag_.rejected_glitches;
        return false;
    }

    // Check against max plausible RPM bound
    const double inst_rpm_d =
        60000000.0 / (static_cast<double>(dt_us) * static_cast<double>(ppr_));
    if (inst_rpm_d > static_cast<double>(max_rpm_)) {
        ++diag_.rejected_glitches;
        return false;
    }

    last_pulse_us_ = timestamp_us;
    ++diag_.total_pulses;
    diag_.last_period_us = static_cast<uint32_t>(dt_us);
    diag_.active = true;

    const float inst_rpm = static_cast<float>(inst_rpm_d);
    if (ema_rpm_ <= 0.1f) {
        ema_rpm_ = inst_rpm;
    } else {
        ema_rpm_ = HALL_EMA_ALPHA * inst_rpm + (1.0f - HALL_EMA_ALPHA) * ema_rpm_;
    }
    diag_.rpm = static_cast<uint16_t>(lroundf(ema_rpm_));
    return true;
}

uint16_t RpmCalculator::update(uint64_t now_us) {
    if (!has_last_pulse_ || now_us <= last_pulse_us_) {
        return diag_.rpm;
    }
    const uint64_t age_us = now_us - last_pulse_us_;
    if (age_us > static_cast<uint64_t>(timeout_ms_) * 1000ULL) {
        diag_.active = false;
        diag_.rpm = 0;
        ema_rpm_ = 0.0f;
    }
    return diag_.rpm;
}

// ---------------------------------------------------------------------------
// Hardware ESP32 ISR Wrapper
// ---------------------------------------------------------------------------
#ifdef ARDUINO
namespace {

portMUX_TYPE g_hall_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t g_isr_last_edge_us = 0;
volatile uint64_t g_isr_accepted_ts_ring[16] = {0};
volatile uint8_t g_isr_head = 0;
volatile uint8_t g_isr_tail = 0;
volatile uint32_t g_isr_glitches = 0;

void IRAM_ATTR hall_isr_handler() {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    portENTER_CRITICAL_ISR(&g_hall_mux);
    if (g_isr_last_edge_us == 0 || (now_us - g_isr_last_edge_us) >= HALL_DEBOUNCE_US) {
        g_isr_last_edge_us = now_us;
        const uint8_t next_head = static_cast<uint8_t>((g_isr_head + 1U) & 0x0FU);
        if (next_head != g_isr_tail) {
            g_isr_accepted_ts_ring[g_isr_head] = now_us;
            g_isr_head = next_head;
        }
    } else {
        g_isr_glitches = g_isr_glitches + 1U;
    }
    portEXIT_CRITICAL_ISR(&g_hall_mux);
}

}  // namespace
#endif

HallRpmDriver::HallRpmDriver()
    : pin_(PIN_HALL_RPM), initialized_(false), calc_() {}

bool HallRpmDriver::begin(uint8_t pin, uint8_t pulses_per_rev) {
    pin_ = pin;
    calc_.reset();
    calc_.set_pulses_per_rev(pulses_per_rev);

#ifdef ARDUINO
    pinMode(pin_, INPUT_PULLUP);
    portENTER_CRITICAL(&g_hall_mux);
    g_isr_last_edge_us = 0;
    g_isr_head = 0;
    g_isr_tail = 0;
    g_isr_glitches = 0;
    portEXIT_CRITICAL(&g_hall_mux);
    attachInterrupt(digitalPinToInterrupt(pin_), hall_isr_handler, FALLING);
    initialized_ = true;
    return true;
#else
    initialized_ = false;
    return false;
#endif
}

void HallRpmDriver::set_pulses_per_rev(uint8_t ppr) {
    calc_.set_pulses_per_rev(ppr);
}

uint16_t HallRpmDriver::poll() {
    if (!initialized_) {
        return 0;
    }
#ifdef ARDUINO
    uint64_t batch[16] = {0};
    uint8_t count = 0;

    portENTER_CRITICAL(&g_hall_mux);
    while (g_isr_tail != g_isr_head && count < 16) {
        batch[count++] = g_isr_accepted_ts_ring[g_isr_tail];
        g_isr_tail = static_cast<uint8_t>((g_isr_tail + 1U) & 0x0FU);
    }
    portEXIT_CRITICAL(&g_hall_mux);

    for (uint8_t i = 0; i < count; ++i) {
        calc_.on_pulse_us(batch[i]);
    }
    return calc_.update(static_cast<uint64_t>(esp_timer_get_time()));
#else
    return 0;
#endif
}

}  // namespace flightcore
