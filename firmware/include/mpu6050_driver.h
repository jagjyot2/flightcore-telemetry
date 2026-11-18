#pragma once

#include <stdint.h>
#include "config.h"

namespace flightcore {

struct MpuRawSample {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t temp_raw = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
};

struct ImuState {
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 1.0f;
    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float vib_rms_mg = 0.0f;
    float die_temp_c = 25.0f;
    bool calibrated = false;
    bool initialized = false;
};

/**
 * Pure-C++ MPU6050 scaling, startup bias calibration, complementary attitude
 * filter, and 3-axis AC vibration RMS processor. Hardware-independent for unit testing.
 */
class ImuAttitudeProcessor {
public:
    explicit ImuAttitudeProcessor(float alpha = IMU_COMPLEMENTARY_ALPHA,
                                  uint16_t cal_target_samples = IMU_CAL_SAMPLES);

    void reset();
    void start_calibration(uint16_t target_samples = IMU_CAL_SAMPLES);

    /**
     * Ingests one raw IMU sample and time delta dt_sec.
     * During calibration phase, accumulates gyro bias and initial tilt alignment.
     * After calibration, updates complementary roll/pitch filter and vibration RMS.
     */
    void update(const MpuRawSample& raw, float dt_sec);

    const ImuState& state() const { return state_; }
    bool is_calibrating() const { return calibrating_; }
    uint16_t cal_progress() const { return cal_count_; }
    uint16_t cal_target() const { return cal_target_; }

    float gyro_bias_x_dps() const { return gx_bias_dps_; }
    float gyro_bias_y_dps() const { return gy_bias_dps_; }
    float gyro_bias_z_dps() const { return gz_bias_dps_; }

private:
    float alpha_;
    uint16_t cal_target_;
    uint16_t cal_count_;
    bool calibrating_;

    double gx_sum_dps_;
    double gy_sum_dps_;
    double gz_sum_dps_;
    double ax_sum_g_;
    double ay_sum_g_;
    double az_sum_g_;

    float gx_bias_dps_;
    float gy_bias_dps_;
    float gz_bias_dps_;
    float ax_bias_g_;
    float ay_bias_g_;
    float az_bias_g_;

    // Rolling window for AC vibration RMS (mean-subtracted frame vibration)
    float vib_ax_[IMU_VIB_WINDOW_SAMPLES];
    float vib_ay_[IMU_VIB_WINDOW_SAMPLES];
    float vib_az_[IMU_VIB_WINDOW_SAMPLES];
    uint8_t vib_idx_;
    uint8_t vib_count_;

    ImuState state_;

    void compute_vibration_rms();
};

/**
 * Hardware I2C MPU6050 Driver (ESP32 Arduino Wire).
 */
class Mpu6050Driver {
public:
    Mpu6050Driver();

    bool begin(uint8_t i2c_addr = MPU6050_I2C_ADDR);
    bool poll(float dt_sec);
    void trigger_calibration(uint16_t samples = IMU_CAL_SAMPLES);

    const ImuState& state() const { return processor_.state(); }
    bool is_healthy() const { return healthy_; }
    bool is_calibrating() const { return processor_.is_calibrating(); }
    uint32_t i2c_error_count() const { return i2c_errors_; }
    uint8_t who_am_i() const { return who_am_i_; }

    ImuAttitudeProcessor& processor() { return processor_; }

private:
    uint8_t addr_;
    uint8_t who_am_i_;
    bool healthy_;
    uint32_t i2c_errors_;
    uint8_t consecutive_errors_;
    ImuAttitudeProcessor processor_;

    bool write_reg(uint8_t reg, uint8_t val);
    bool read_regs(uint8_t start_reg, uint8_t* buf, uint8_t len);
};

}  // namespace flightcore
