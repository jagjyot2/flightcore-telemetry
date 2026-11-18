#include "mpu6050_driver.h"
#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#endif

namespace flightcore {

namespace {

constexpr double RAD_TO_DEG_D = 57.295779513082320876;

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_INT_ENABLE = 0x38;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

inline int16_t be_to_i16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

}  // namespace

ImuAttitudeProcessor::ImuAttitudeProcessor(float alpha, uint16_t cal_target_samples)
    : alpha_(alpha), cal_target_(cal_target_samples) {
    reset();
}

void ImuAttitudeProcessor::reset() {
    cal_count_ = 0;
    calibrating_ = (cal_target_ > 0);
    gx_sum_dps_ = 0.0;
    gy_sum_dps_ = 0.0;
    gz_sum_dps_ = 0.0;
    ax_sum_g_ = 0.0;
    ay_sum_g_ = 0.0;
    az_sum_g_ = 0.0;
    gx_bias_dps_ = 0.0f;
    gy_bias_dps_ = 0.0f;
    gz_bias_dps_ = 0.0f;
    ax_bias_g_ = 0.0f;
    ay_bias_g_ = 0.0f;
    az_bias_g_ = 0.0f;
    memset(vib_ax_, 0, sizeof(vib_ax_));
    memset(vib_ay_, 0, sizeof(vib_ay_));
    memset(vib_az_, 0, sizeof(vib_az_));
    vib_idx_ = 0;
    vib_count_ = 0;
    state_ = ImuState{};
}

void ImuAttitudeProcessor::start_calibration(uint16_t target_samples) {
    cal_target_ = target_samples;
    cal_count_ = 0;
    calibrating_ = (target_samples > 0);
    gx_sum_dps_ = 0.0;
    gy_sum_dps_ = 0.0;
    gz_sum_dps_ = 0.0;
    ax_sum_g_ = 0.0;
    ay_sum_g_ = 0.0;
    az_sum_g_ = 0.0;
    state_.calibrated = !calibrating_;
}

void ImuAttitudeProcessor::compute_vibration_rms() {
    if (vib_count_ < 2) {
        state_.vib_rms_mg = 0.0f;
        return;
    }

    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float mean_z = 0.0f;
    for (uint8_t i = 0; i < vib_count_; ++i) {
        mean_x += vib_ax_[i];
        mean_y += vib_ay_[i];
        mean_z += vib_az_[i];
    }
    const float inv_n = 1.0f / static_cast<float>(vib_count_);
    mean_x *= inv_n;
    mean_y *= inv_n;
    mean_z *= inv_n;

    float sq_sum = 0.0f;
    for (uint8_t i = 0; i < vib_count_; ++i) {
        const float dx = vib_ax_[i] - mean_x;
        const float dy = vib_ay_[i] - mean_y;
        const float dz = vib_az_[i] - mean_z;
        sq_sum += (dx * dx + dy * dy + dz * dz);
    }

    const float rms_g = sqrtf(sq_sum * inv_n);
    state_.vib_rms_mg = rms_g * 1000.0f;
}

void ImuAttitudeProcessor::update(const MpuRawSample& raw, float dt_sec) {
    const float raw_ax_g = static_cast<float>(raw.ax) / MPU_ACCEL_LSB_PER_G;
    const float raw_ay_g = static_cast<float>(raw.ay) / MPU_ACCEL_LSB_PER_G;
    const float raw_az_g = static_cast<float>(raw.az) / MPU_ACCEL_LSB_PER_G;
    const float raw_gx_dps = static_cast<float>(raw.gx) / MPU_GYRO_LSB_PER_DPS;
    const float raw_gy_dps = static_cast<float>(raw.gy) / MPU_GYRO_LSB_PER_DPS;
    const float raw_gz_dps = static_cast<float>(raw.gz) / MPU_GYRO_LSB_PER_DPS;
    state_.die_temp_c = (static_cast<float>(raw.temp_raw) / 340.0f) + 36.53f;

    if (calibrating_) {
        gx_sum_dps_ += raw_gx_dps;
        gy_sum_dps_ += raw_gy_dps;
        gz_sum_dps_ += raw_gz_dps;
        ax_sum_g_ += raw_ax_g;
        ay_sum_g_ += raw_ay_g;
        az_sum_g_ += raw_az_g;
        ++cal_count_;

        if (cal_count_ >= cal_target_) {
            const double inv = 1.0 / static_cast<double>(cal_count_);
            gx_bias_dps_ = static_cast<float>(gx_sum_dps_ * inv);
            gy_bias_dps_ = static_cast<float>(gy_sum_dps_ * inv);
            gz_bias_dps_ = static_cast<float>(gz_sum_dps_ * inv);

            // Assume level mounting during startup bias calibration (+1g on +Z)
            ax_bias_g_ = static_cast<float>(ax_sum_g_ * inv);
            ay_bias_g_ = static_cast<float>(ay_sum_g_ * inv);
            az_bias_g_ = static_cast<float>(az_sum_g_ * inv) - 1.0f;

            calibrating_ = false;
            state_.calibrated = true;
            state_.initialized = true;
            state_.roll_deg = 0.0f;
            state_.pitch_deg = 0.0f;
        }
        return;
    }

    const float ax_g = raw_ax_g - ax_bias_g_;
    const float ay_g = raw_ay_g - ay_bias_g_;
    const float az_g = raw_az_g - az_bias_g_;
    const float gx_dps = raw_gx_dps - gx_bias_dps_;
    const float gy_dps = raw_gy_dps - gy_bias_dps_;
    const float gz_dps = raw_gz_dps - gz_bias_dps_;

    state_.ax_g = ax_g;
    state_.ay_g = ay_g;
    state_.az_g = az_g;
    state_.gx_dps = gx_dps;
    state_.gy_dps = gy_dps;
    state_.gz_dps = gz_dps;

    // Accel attitude estimate (protected against zero-gravity singular vector)
    const float accel_mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    float accel_roll_deg = state_.roll_deg;
    float accel_pitch_deg = state_.pitch_deg;

    if (accel_mag > 0.1f) {
        accel_roll_deg = static_cast<float>(
            atan2(static_cast<double>(ay_g), static_cast<double>(az_g)) * RAD_TO_DEG_D);
        const float horiz = sqrtf(ay_g * ay_g + az_g * az_g);
        accel_pitch_deg = static_cast<float>(
            atan2(static_cast<double>(-ax_g), static_cast<double>(horiz)) * RAD_TO_DEG_D);
    }

    if (!state_.initialized) {
        state_.roll_deg = accel_roll_deg;
        state_.pitch_deg = accel_pitch_deg;
        state_.initialized = true;
    } else if (dt_sec > 0.0f && dt_sec <= 0.25f) {
        const float gyro_roll = state_.roll_deg + gx_dps * dt_sec;
        const float gyro_pitch = state_.pitch_deg + gy_dps * dt_sec;
        state_.roll_deg = alpha_ * gyro_roll + (1.0f - alpha_) * accel_roll_deg;
        state_.pitch_deg = alpha_ * gyro_pitch + (1.0f - alpha_) * accel_pitch_deg;
    }

    // Store sample for AC vibration RMS window
    vib_ax_[vib_idx_] = ax_g;
    vib_ay_[vib_idx_] = ay_g;
    vib_az_[vib_idx_] = az_g;
    vib_idx_ = static_cast<uint8_t>((vib_idx_ + 1U) % IMU_VIB_WINDOW_SAMPLES);
    if (vib_count_ < IMU_VIB_WINDOW_SAMPLES) {
        ++vib_count_;
    }
    compute_vibration_rms();
}

// ---------------------------------------------------------------------------
// Hardware ESP32 I2C Driver
// ---------------------------------------------------------------------------
Mpu6050Driver::Mpu6050Driver()
    : addr_(MPU6050_I2C_ADDR),
      who_am_i_(0),
      healthy_(false),
      i2c_errors_(0),
      consecutive_errors_(0),
      processor_(IMU_COMPLEMENTARY_ALPHA, IMU_CAL_SAMPLES) {}

bool Mpu6050Driver::write_reg(uint8_t reg, uint8_t val) {
#ifdef ARDUINO
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0) {
        ++i2c_errors_;
        return false;
    }
    return true;
#else
    (void)reg;
    (void)val;
    return false;
#endif
}

bool Mpu6050Driver::read_regs(uint8_t start_reg, uint8_t* buf, uint8_t len) {
#ifdef ARDUINO
    Wire.beginTransmission(addr_);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) {
        ++i2c_errors_;
        return false;
    }
    const uint8_t got = Wire.requestFrom(static_cast<int>(addr_), static_cast<int>(len));
    if (got != len) {
        ++i2c_errors_;
        return false;
    }
    for (uint8_t i = 0; i < len; ++i) {
        buf[i] = static_cast<uint8_t>(Wire.read());
    }
    return true;
#else
    (void)start_reg;
    (void)buf;
    (void)len;
    return false;
#endif
}

bool Mpu6050Driver::begin(uint8_t i2c_addr) {
    addr_ = i2c_addr;
    healthy_ = false;
    consecutive_errors_ = 0;

#ifdef ARDUINO
    uint8_t id = 0;
    if (!read_regs(REG_WHO_AM_I, &id, 1)) {
        return false;
    }
    who_am_i_ = id;
    // MPU6050 returns 0x68 (or 0x70/0x98 on certain compatible clones)
    if (id != 0x68 && id != 0x70 && id != 0x98) {
        return false;
    }

    // Wake up with PLL X-gyro clock source
    if (!write_reg(REG_PWR_MGMT_1, 0x01)) return false;
    delay(20);
    // 200 Hz sample rate: 1000 Hz / (1 + 4) = 200 Hz
    if (!write_reg(REG_SMPLRT_DIV, 0x04)) return false;
    // DLPF_CFG = 3 (~44 Hz accel, ~42 Hz gyro bandwidth)
    if (!write_reg(REG_CONFIG, 0x03)) return false;
    // Gyro +/-500 deg/s (FS_SEL = 1 -> 0x08)
    if (!write_reg(REG_GYRO_CONFIG, 0x08)) return false;
    // Accel +/-4g (AFS_SEL = 1 -> 0x08)
    if (!write_reg(REG_ACCEL_CONFIG, 0x08)) return false;
    // Data ready interrupt enable
    if (!write_reg(REG_INT_ENABLE, 0x01)) return false;

    healthy_ = true;
    processor_.reset();
    return true;
#else
    return false;
#endif
}

bool Mpu6050Driver::poll(float dt_sec) {
    if (!healthy_) {
        return false;
    }

    uint8_t raw_bytes[14] = {0};
    if (!read_regs(REG_ACCEL_XOUT_H, raw_bytes, 14)) {
        ++consecutive_errors_;
        if (consecutive_errors_ >= 10) {
            healthy_ = false;
        }
        return false;
    }
    consecutive_errors_ = 0;

    MpuRawSample sample;
    sample.ax = be_to_i16(raw_bytes[0], raw_bytes[1]);
    sample.ay = be_to_i16(raw_bytes[2], raw_bytes[3]);
    sample.az = be_to_i16(raw_bytes[4], raw_bytes[5]);
    sample.temp_raw = be_to_i16(raw_bytes[6], raw_bytes[7]);
    sample.gx = be_to_i16(raw_bytes[8], raw_bytes[9]);
    sample.gy = be_to_i16(raw_bytes[10], raw_bytes[11]);
    sample.gz = be_to_i16(raw_bytes[12], raw_bytes[13]);

    processor_.update(sample, dt_sec);
    return true;
}

void Mpu6050Driver::trigger_calibration(uint16_t samples) {
    processor_.start_calibration(samples);
}

}  // namespace flightcore
