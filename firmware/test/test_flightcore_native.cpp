#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "hall_rpm_driver.h"
#include "lora_sx1278.h"
#include "mpu6050_driver.h"
#include "power_temp_monitor.h"
#include "telemetry_protocol.h"
#include "vl53l0x_driver.h"

using namespace flightcore;

namespace {

int g_tests_run = 0;
int g_tests_passed = 0;

#define RUN_TEST(fn)                                          \
    do {                                                      \
        ++g_tests_run;                                        \
        fn();                                                 \
        ++g_tests_passed;                                     \
        printf("[PASS] %s\n", #fn);                           \
    } while (0)

void expect_near(double actual, double expected, double tol, const char* msg) {
    if (fabs(actual - expected) > tol) {
        fprintf(stderr, "FAIL (%s): actual=%.6f expected=%.6f tol=%.6f\n",
                msg, actual, expected, tol);
        assert(false);
    }
}

void test_crc16_ccitt_standard_vector() {
    // Standard CRC-16/CCITT-FALSE ("123456789") = 0x29B1
    const uint8_t msg[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    const uint16_t crc = crc16_ccitt(msg, sizeof(msg));
    assert(crc == 0x29B1U);
}

void test_telemetry_encode_decode_and_corruption() {
    TelemetryFrame tx;
    tx.seq = 42;
    tx.timestamp_ms = 1234567UL;
    tx.roll_cdeg = -1245;
    tx.pitch_cdeg = 789;
    tx.vib_rms_mg = 145;
    tx.tof_mm = 642;
    tx.rpm = 8450;
    tx.vbat_mv = 15840;
    tx.ibat_ca = 1875;
    tx.consumed_mah = 612;
    tx.ntc_temp_cdegc = 3650;
    tx.imu_temp_cdegc = 3420;
    tx.lora_rssi_dbm = -72;
    tx.sensor_status = STATUS_MPU_OK | STATUS_IMU_CALIBRATED | STATUS_TOF_OK | STATUS_LORA_OK;
    tx.tx_retries = 1;
    tx.cam_status = 2;

    uint8_t buf[PROTO_FRAME_SIZE] = {0};
    const size_t n = encode_telemetry_frame(tx, buf, sizeof(buf));
    assert(n == PROTO_FRAME_SIZE);
    assert(buf[0] == 0xFC && buf[1] == 0x01);

    TelemetryFrame rx;
    assert(decode_telemetry_frame(buf, n, rx) == DecodeStatus::OK);
    assert(rx.seq == tx.seq);
    assert(rx.timestamp_ms == tx.timestamp_ms);
    assert(rx.roll_cdeg == tx.roll_cdeg);
    assert(rx.pitch_cdeg == tx.pitch_cdeg);
    assert(rx.vib_rms_mg == tx.vib_rms_mg);
    assert(rx.tof_mm == tx.tof_mm);
    assert(rx.rpm == tx.rpm);
    assert(rx.vbat_mv == tx.vbat_mv);
    assert(rx.ibat_ca == tx.ibat_ca);
    assert(rx.consumed_mah == tx.consumed_mah);
    assert(rx.ntc_temp_cdegc == tx.ntc_temp_cdegc);
    assert(rx.imu_temp_cdegc == tx.imu_temp_cdegc);
    assert(rx.lora_rssi_dbm == tx.lora_rssi_dbm);
    assert(rx.sensor_status == tx.sensor_status);
    assert(rx.tx_retries == tx.tx_retries);
    assert(rx.cam_status == tx.cam_status);

    // Flip 1 bit in payload -> CRC mismatch
    buf[15] ^= 0x04;
    assert(decode_telemetry_frame(buf, n, rx) == DecodeStatus::ERR_CRC);
    buf[15] ^= 0x04;

    // Stream resync across garbage + partial sync bytes
    FrameStreamParser parser;
    const uint8_t prefix[] = {0xAA, 0xFC, 0x99, 0x00, 0xFC};
    for (uint8_t b : prefix) {
        assert(!parser.feed(b, rx));
    }
    bool decoded = false;
    for (size_t i = 0; i < n; ++i) {
        if (parser.feed(buf[i], rx)) {
            decoded = true;
            assert(i == n - 1);
        }
    }
    assert(decoded);
    assert(parser.valid_frames() == 1);

    char json[320] = {0};
    assert(format_telemetry_json(rx, json, sizeof(json)) > 20);
    assert(strstr(json, "\"seq\":42") != nullptr);
}

void test_steinhart_hart_and_power_integrator() {
    // At 25C, R_ntc = 10000 ohm -> V_adc = Vref/2 = 1650 mV
    const float r = PowerTempCalculator::ntc_resistance_from_mv(1650.0f);
    expect_near(r, 10000.0, 0.5, "NTC midpoint resistance");
    const float t25 = PowerTempCalculator::steinhart_hart_celsius(r);
    expect_near(t25, 25.0, 0.05, "NTC 25C Steinhart-Hart");

    // Hot: R = 2488 ohm -> 60.0 C
    const float t60 = PowerTempCalculator::steinhart_hart_celsius(2488.0f);
    expect_near(t60, 60.0, 0.05, "NTC 60C Steinhart-Hart");

    // Fault open/short detection
    assert(PowerTempCalculator::ntc_resistance_from_mv(2.0f) < 0.0f);
    assert(PowerTempCalculator::ntc_resistance_from_mv(3298.0f) < 0.0f);

    // Power integrator: 14.8V (1345.45 mV ADC), 18A (900 mV ADC @ 50 mV/A) for 3600s -> 18000 mAh
    PowerTempCalculator pwr;
    for (int i = 0; i < 3600; ++i) {
        pwr.update(1345.4545f, 900.0f, 1650.0f, 1.0f);
    }
    expect_near(pwr.state().vbat_v, 14.8, 0.05, "VBAT 14.8V");
    expect_near(pwr.state().ibat_a, 18.0, 0.05, "IBAT 18.0A");
    // Note: first sample establishes prev_ibat_a_ -> 3599 intervals of 1.0s
    expect_near(pwr.state().consumed_mah, 18000.0 * (3599.0 / 3600.0), 1.0, "mAh integrator");
}

void test_mpu6050_calibration_complementary_and_vibration() {
    ImuAttitudeProcessor imu(0.98f, 20);
    // Feed 20 calibration samples with constant gyro bias (gx=2.0 deg/s = 131 LSB)
    MpuRawSample cal{};
    cal.ax = 0;
    cal.ay = 0;
    cal.az = 8192;  // +1.0 g
    cal.gx = 131;   // +2.0 deg/s bias
    cal.gy = -65;
    cal.gz = 0;
    cal.temp_raw = 0;  // 36.53 C

    for (int i = 0; i < 20; ++i) {
        imu.update(cal, 0.005f);
    }
    assert(!imu.is_calibrating());
    assert(imu.state().calibrated);
    expect_near(imu.gyro_bias_x_dps(), 2.0, 0.02, "Gyro X bias removed");

    // Step tilt to +15 deg roll: ay = sin(15)*1g, az = cos(15)*1g, gyro stays unbiased
    const double rad = 15.0 * 3.14159265358979323846 / 180.0;
    MpuRawSample tilted = cal;
    tilted.ay = static_cast<int16_t>(lround(sin(rad) * 8192.0));
    tilted.az = static_cast<int16_t>(lround(cos(rad) * 8192.0));
    // Run 600 steps (3.0 seconds @ 200 Hz) so complementary filter converges to accel tilt
    for (int i = 0; i < 600; ++i) {
        imu.update(tilted, 0.005f);
    }
    expect_near(imu.state().roll_deg, 15.0, 0.4, "Complementary filter roll convergence");

    // Inject +/-0.1g square wave on Z -> AC RMS should be ~100 mg
    for (int i = 0; i < 32; ++i) {
        MpuRawSample vib = cal;
        vib.az = static_cast<int16_t>(8192 + ((i % 2 == 0) ? 819 : -819));
        imu.update(vib, 0.005f);
    }
    expect_near(imu.state().vib_rms_mg, 100.0, 5.0, "Vibration RMS 100 mg");
}

void test_vl53l0x_median_ema_and_glitch_rejection() {
    TofGlitchFilter tof(0.5f, 30, 2000);
    // Steady readings around 500 mm with one huge 1900 mm laser glitch
    tof.update(500, true);
    tof.update(502, true);
    tof.update(1900, true);  // Specular spike
    tof.update(498, true);
    const uint16_t filtered = tof.update(501, true);
    assert(tof.is_range_valid());
    assert(tof.diagnostics().glitch_rejected_count >= 1);
    assert(filtered >= 495 && filtered <= 520);

    // 3 consecutive invalid readings trigger invalid sentinel
    tof.update(TOF_INVALID_SENTINEL, false);
    tof.update(TOF_INVALID_SENTINEL, false);
    tof.update(TOF_INVALID_SENTINEL, false);
    assert(!tof.is_range_valid());
    assert(tof.filtered_mm() == TOF_INVALID_SENTINEL);
}

void test_hall_rpm_debounce_and_timeout() {
    RpmCalculator hall(2, 350, 500, 30000);  // 2 pulses/rev -> 6000 RPM = 5000 us/pulse
    uint64_t t = 1000000ULL;
    assert(hall.on_pulse_us(t));
    // Bounce 100 us later -> rejected
    assert(!hall.on_pulse_us(t + 100ULL));
    assert(hall.diagnostics().rejected_glitches == 1);

    for (int i = 0; i < 15; ++i) {
        t += 5000ULL;
        assert(hall.on_pulse_us(t));
    }
    assert(hall.is_active());
    assert(hall.rpm() >= 5950 && hall.rpm() <= 6050);

    // Timeout after 600 ms without pulse -> 0 RPM
    assert(hall.update(t + 600000ULL) == 0);
    assert(!hall.is_active());
}

class MockSx1278Transport : public ISx1278Transport {
public:
    uint8_t regs[128] = {0};
    uint8_t fifo[256] = {0};
    size_t fifo_len = 0;
    bool dio0 = false;

    MockSx1278Transport() {
        regs[LoraSx1278::REG_VERSION] = LoraSx1278::EXPECTED_VERSION;
    }
    bool begin() override { return true; }
    void reset_pulse() override {}
    uint8_t read_register(uint8_t reg) override { return regs[reg & 0x7F]; }
    void write_register(uint8_t reg, uint8_t value) override {
        if (reg == LoraSx1278::REG_IRQ_FLAGS) {
            regs[reg] &= static_cast<uint8_t>(~value);
        } else {
            regs[reg & 0x7F] = value;
        }
    }
    void write_fifo(const uint8_t* data, size_t len) override {
        memcpy(fifo, data, len);
        fifo_len = len;
    }
    void read_fifo(uint8_t* out, size_t len) override {
        memcpy(out, fifo, len);
    }
    bool read_dio0() const override { return dio0; }
};

void test_sx1278_tx_retry_and_recovery() {
    MockSx1278Transport mock;
    LoraSx1278 radio(&mock);
    assert(radio.begin());
    assert(radio.is_healthy());

    uint8_t pkt[PROTO_FRAME_SIZE] = {0xFC, 0x01};
    assert(radio.start_transmit(pkt, sizeof(pkt), 1000));
    assert(radio.is_busy());

    // Simulate 1 TX timeout -> backoff -> retry -> TX_DONE
    radio.poll(1000 + LORA_TX_TIMEOUT_MS + 5);
    assert(radio.diagnostics().tx_timeouts == 1);
    assert(radio.diagnostics().tx_retries_total == 1);

    // Advance past retry backoff (25 ms) so 2nd attempt starts
    radio.poll(1000 + LORA_TX_TIMEOUT_MS + 40);
    mock.regs[LoraSx1278::REG_IRQ_FLAGS] = LoraSx1278::IRQ_TX_DONE_MASK;
    radio.poll(1000 + LORA_TX_TIMEOUT_MS + 50);

    assert(!radio.is_busy());
    assert(radio.diagnostics().tx_success == 1);
    assert(radio.diagnostics().last_tx_retries == 1);
}

}  // namespace

int main() {
    printf("=== FlightCore Native C++ Verification Suite ===\n");
    RUN_TEST(test_crc16_ccitt_standard_vector);
    RUN_TEST(test_telemetry_encode_decode_and_corruption);
    RUN_TEST(test_steinhart_hart_and_power_integrator);
    RUN_TEST(test_mpu6050_calibration_complementary_and_vibration);
    RUN_TEST(test_vl53l0x_median_ema_and_glitch_rejection);
    RUN_TEST(test_hall_rpm_debounce_and_timeout);
    RUN_TEST(test_sx1278_tx_retry_and_recovery);
    printf("Result: %d/%d C++ tests PASSED.\n", g_tests_passed, g_tests_run);
    return (g_tests_passed == g_tests_run) ? 0 : 1;
}
