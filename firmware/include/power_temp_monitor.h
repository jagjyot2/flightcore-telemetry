#pragma once

#include <stdint.h>
#include "config.h"

namespace flightcore {

struct PowerTempState {
    uint16_t vbat_mv = 0;
    float vbat_v = 0.0f;
    float ibat_a = 0.0f;
    int16_t ibat_ca = 0;
    float consumed_mah = 0.0f;
    float ntc_resistance_ohm = 0.0f;
    float ntc_temp_c = 0.0f;
    int16_t ntc_temp_cdegc = TEMP_INVALID_CDEGC;
    bool ntc_valid = false;
    bool vbat_valid = false;
    bool low_battery_warn = false;
};

/**
 * Pure-C++ battery voltage divider, INA180 shunt current & mAh integrator,
 * and 3-coefficient Steinhart-Hart NTC temperature solver.
 */
class PowerTempCalculator {
public:
    PowerTempCalculator();

    void reset_mah();
    void set_low_voltage_threshold_mv(uint16_t mv) { low_vbat_mv_ = mv; }

    /**
     * Computes NTC thermistor resistance (ohms) from ADC millivolts.
     * Circuit: +3.3V -> R_series (10k) -> ADC_NODE -> R_NTC -> GND.
     * Returns <= 0.0f on open/short circuit fault.
     */
    static float ntc_resistance_from_mv(float adc_mv,
                                        float vref_mv = ADC_VREF_MV,
                                        float r_series = NTC_SERIES_RESISTOR_OHMS);

    /**
     * Evaluates 3-term Steinhart-Hart equation:
     *   1 / T_kelvin = A + B*ln(R) + C*(ln(R))^3
     */
    static float steinhart_hart_celsius(float resistance_ohms,
                                        float a = NTC_SH_A,
                                        float b = NTC_SH_B,
                                        float c = NTC_SH_C);

    /**
     * Updates voltage, current, trapezoidal mAh consumption, and NTC temperature.
     */
    void update(float vbat_adc_mv,
                float ibat_adc_mv,
                float ntc_adc_mv,
                float dt_sec);

    const PowerTempState& state() const { return state_; }

private:
    PowerTempState state_;
    float prev_ibat_a_;
    bool has_prev_current_;
    bool vbat_seeded_;
    float vbat_ema_mv_;
    uint16_t low_vbat_mv_;
};

/**
 * Hardware ESP32 ADC1 Monitor (oversampled eFuse-calibrated analogReadMilliVolts).
 */
class PowerTempMonitor {
public:
    PowerTempMonitor();

    bool begin(uint8_t vbat_pin = PIN_ADC_VBAT,
               uint8_t ibat_pin = PIN_ADC_IBAT,
               uint8_t ntc_pin = PIN_ADC_NTC);
    void poll(float dt_sec);
    void reset_mah() { calc_.reset_mah(); }

    const PowerTempState& state() const { return calc_.state(); }

private:
    uint8_t vbat_pin_;
    uint8_t ibat_pin_;
    uint8_t ntc_pin_;
    bool initialized_;
    PowerTempCalculator calc_;

    float read_oversampled_mv(uint8_t pin) const;
};

}  // namespace flightcore
