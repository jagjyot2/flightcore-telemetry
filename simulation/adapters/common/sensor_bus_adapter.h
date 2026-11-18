#pragma once

#include <stddef.h>
#include <stdint.h>

namespace flightcore::sim {

/**
 * Canonical simulator-agnostic sensor & telemetry transport interface.
 *
 * SAFETY BOUNDARY:
 * Strictly passive sensor acquisition and telemetry transport only.
 * Zero motor/ESC PWM or flight-control actuation methods exist.
 */
#ifndef FLIGHTCORE_STRICTLY_TELEMETRY_ONLY
#define FLIGHTCORE_STRICTLY_TELEMETRY_ONLY 1
#endif

struct SimSensorSnapshot {
    uint32_t timestamp_ms = 0;

    // Inertial (MPU6050 equivalent)
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 1.0f;
    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;
    int16_t imu_temp_cdegc = 2500;

    // Time-of-Flight (VL53L0X equivalent)
    uint16_t tof_mm = 0xFFFFU;
    bool tof_valid = false;

    // Rotor RPM (A3144 Hall equivalent)
    uint16_t rpm = 0;
    uint32_t hall_pulse_period_us = 0;

    // Power & Thermal (11:1 VBAT divider, INA180A2 50 mV/A, 10k B3950 NTC)
    uint16_t vbat_mv = 0;
    int16_t ibat_ca = 0;
    uint16_t consumed_mah = 0;
    int16_t ntc_temp_cdegc = -32768;

    // Link diagnostics (SX1278 433 MHz LoRa)
    int8_t lora_rssi_dbm = -120;
    uint8_t tx_retries = 0;
    uint8_t cam_status = 0;
};

struct SimFaultProfile {
    bool inject_crc_flip = false;
    bool inject_tof_timeout = false;
    bool inject_ntc_fault = false;
    bool inject_high_vibration = false;
    uint16_t vbat_sag_mv = 0;
    uint8_t forced_tx_retries = 0;
};

struct SimTransportStats {
    uint32_t frames_sent = 0;
    uint32_t frames_corrupted = 0;
    uint32_t frames_dropped = 0;
};

class ISensorBusAdapter {
public:
    virtual ~ISensorBusAdapter() = default;

    virtual const char* adapter_name() const = 0;
    virtual bool initialize() = 0;
    virtual bool read_snapshot(SimSensorSnapshot& out_snapshot) = 0;
    virtual bool transmit_telemetry_frame(const uint8_t* frame, size_t len) = 0;
    virtual void set_fault_profile(const SimFaultProfile& profile) = 0;
    virtual SimTransportStats transport_stats() const = 0;
};

}  // namespace flightcore::sim
