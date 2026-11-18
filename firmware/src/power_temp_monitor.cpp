#include "power_temp_monitor.h"
#include <math.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace flightcore {

PowerTempCalculator::PowerTempCalculator()
    : state_(),
      prev_ibat_a_(0.0f),
      has_prev_current_(false),
      vbat_seeded_(false),
      vbat_ema_mv_(0.0f),
      low_vbat_mv_(10500) {}

void PowerTempCalculator::reset_mah() {
    state_.consumed_mah = 0.0f;
}

float PowerTempCalculator::ntc_resistance_from_mv(float adc_mv,
                                                  float vref_mv,
                                                  float r_series) {
    // Detect open circuit (pulled up near Vref) or short to GND
    if (adc_mv <= 10.0f || adc_mv >= (vref_mv - 10.0f)) {
        return -1.0f;
    }
    // Circuit: +Vref -- [R_series] -- ADC_PIN -- [R_NTC] -- GND
    return r_series * (adc_mv / (vref_mv - adc_mv));
}

float PowerTempCalculator::steinhart_hart_celsius(float resistance_ohms,
                                                  float a,
                                                  float b,
                                                  float c) {
    if (resistance_ohms <= 0.0f) {
        return -273.15f;
    }
    const double ln_r = log(static_cast<double>(resistance_ohms));
    const double inv_t = static_cast<double>(a) +
                         static_cast<double>(b) * ln_r +
                         static_cast<double>(c) * (ln_r * ln_r * ln_r);
    if (inv_t <= 0.0) {
        return -273.15f;
    }
    return static_cast<float>((1.0 / inv_t) - 273.15);
}

void PowerTempCalculator::update(float vbat_adc_mv,
                                 float ibat_adc_mv,
                                 float ntc_adc_mv,
                                 float dt_sec) {
    // 1. Battery Voltage (100k/10k divider -> ratio 11.0)
    const float raw_vbat_mv = (vbat_adc_mv > 0.0f) ? (vbat_adc_mv * VBAT_DIVIDER_RATIO) : 0.0f;
    if (!vbat_seeded_) {
        vbat_ema_mv_ = raw_vbat_mv;
        vbat_seeded_ = true;
    } else {
        vbat_ema_mv_ = 0.25f * raw_vbat_mv + 0.75f * vbat_ema_mv_;
    }

    const float clamped_vbat_mv = (vbat_ema_mv_ < 0.0f) ? 0.0f :
                                  (vbat_ema_mv_ > 65535.0f ? 65535.0f : vbat_ema_mv_);
    state_.vbat_mv = static_cast<uint16_t>(lroundf(clamped_vbat_mv));
    state_.vbat_v = clamped_vbat_mv / 1000.0f;
    state_.vbat_valid = (state_.vbat_mv >= 3000U);
    state_.low_battery_warn = state_.vbat_valid && (state_.vbat_mv < low_vbat_mv_);

    // 2. Battery Current (INA180A2 gain 50 across 1 mOhm shunt -> 50 mV/A)
    const float v_sense_v = (ibat_adc_mv - CURRENT_ZERO_OFFSET_MV) / 1000.0f;
    float ibat_a = v_sense_v / (CURRENT_SHUNT_OHMS * CURRENT_AMP_GAIN);
    if (fabsf(ibat_a) < CURRENT_DEADBAND_A) {
        ibat_a = 0.0f;
    }
    if (ibat_a > 327.0f) ibat_a = 327.0f;
    if (ibat_a < -327.0f) ibat_a = -327.0f;
    state_.ibat_a = ibat_a;
    state_.ibat_ca = static_cast<int16_t>(lroundf(ibat_a * 100.0f));

    // 3. Trapezoidal mAh Integration
    if (dt_sec > 0.0f && dt_sec <= 5.0f && has_prev_current_) {
        const float avg_a = 0.5f * (prev_ibat_a_ + ibat_a);
        if (avg_a > 0.0f) {
            state_.consumed_mah += avg_a * (dt_sec / 3600.0f) * 1000.0f;
        }
    }
    prev_ibat_a_ = ibat_a;
    has_prev_current_ = true;

    // 4. NTC Steinhart-Hart Temperature
    const float r_ntc = ntc_resistance_from_mv(ntc_adc_mv);
    if (r_ntc <= 0.0f) {
        state_.ntc_resistance_ohm = 0.0f;
        state_.ntc_temp_c = 0.0f;
        state_.ntc_temp_cdegc = TEMP_INVALID_CDEGC;
        state_.ntc_valid = false;
    } else {
        const float temp_c = steinhart_hart_celsius(r_ntc);
        state_.ntc_resistance_ohm = r_ntc;
        state_.ntc_temp_c = temp_c;
        state_.ntc_valid = (temp_c >= -45.0f && temp_c <= 150.0f);
        if (state_.ntc_valid) {
            state_.ntc_temp_cdegc = static_cast<int16_t>(lroundf(temp_c * 100.0f));
        } else {
            state_.ntc_temp_cdegc = TEMP_INVALID_CDEGC;
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware ESP32 ADC1 Monitor
// ---------------------------------------------------------------------------
PowerTempMonitor::PowerTempMonitor()
    : vbat_pin_(PIN_ADC_VBAT),
      ibat_pin_(PIN_ADC_IBAT),
      ntc_pin_(PIN_ADC_NTC),
      initialized_(false) {}

bool PowerTempMonitor::begin(uint8_t vbat_pin, uint8_t ibat_pin, uint8_t ntc_pin) {
    vbat_pin_ = vbat_pin;
    ibat_pin_ = ibat_pin;
    ntc_pin_ = ntc_pin;

#ifdef ARDUINO
    pinMode(vbat_pin_, INPUT);
    pinMode(ibat_pin_, INPUT);
    pinMode(ntc_pin_, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    initialized_ = true;
    return true;
#else
    initialized_ = false;
    return false;
#endif
}

float PowerTempMonitor::read_oversampled_mv(uint8_t pin) const {
#ifdef ARDUINO
    uint32_t sum_mv = 0;
    for (uint8_t i = 0; i < ADC_OVERSAMPLE_COUNT; ++i) {
        sum_mv += analogReadMilliVolts(pin);
    }
    return static_cast<float>(sum_mv) / static_cast<float>(ADC_OVERSAMPLE_COUNT);
#else
    (void)pin;
    return 0.0f;
#endif
}

void PowerTempMonitor::poll(float dt_sec) {
    if (!initialized_) {
        return;
    }
    const float vbat_mv = read_oversampled_mv(vbat_pin_);
    const float ibat_mv = read_oversampled_mv(ibat_pin_);
    const float ntc_mv = read_oversampled_mv(ntc_pin_);
    calc_.update(vbat_mv, ibat_mv, ntc_mv, dt_sec);
}

}  // namespace flightcore
