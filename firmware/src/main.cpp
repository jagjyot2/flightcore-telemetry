#ifdef ARDUINO

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "hall_rpm_driver.h"
#include "lora_sx1278.h"
#include "mpu6050_driver.h"
#include "power_temp_monitor.h"
#include "telemetry_protocol.h"
#include "vl53l0x_driver.h"

static_assert(FLIGHTCORE_STRICTLY_TELEMETRY_ONLY == 1,
              "FlightCore is strictly telemetry-only; motor/ESC actuation is prohibited.");

using namespace flightcore;

namespace {

Mpu6050Driver g_mpu;
Vl53l0xDriver g_tof;
HallRpmDriver g_hall;
PowerTempMonitor g_pwr;
Esp32Sx1278Transport g_lora_spi;
LoraSx1278 g_lora(&g_lora_spi);

TelemetryFrame g_latest_frame;
uint16_t g_seq_counter = 0;
uint32_t g_telem_interval_ms = TELEM_DEFAULT_INTERVAL_MS;

bool g_stream_json = false;
bool g_stream_bin = false;

// Scheduler timestamps
uint32_t g_last_imu_ms = 0;
uint32_t g_last_tof_ms = 0;
uint32_t g_last_pwr_ms = 0;
uint32_t g_last_telem_ms = 0;
uint32_t g_last_cam_hb_ms = 0;
uint32_t g_last_led_ms = 0;
bool g_led_state = false;

// Debounced calibration button state
bool g_cal_btn_prev = HIGH;
uint32_t g_cal_btn_press_ms = 0;

// Serial CLI buffer
char g_cli_buf[96];
size_t g_cli_len = 0;

// ESP32-CAM UART2 Bridge Stub state
char g_cam_rx_buf[96];
size_t g_cam_rx_len = 0;
uint32_t g_cam_last_rx_ms = 0;
uint32_t g_cam_rx_lines = 0;
uint8_t g_cam_status_code = 0;  // 0=offline/stub, 1=ready, 2=streaming

int16_t clamp_i16(float val) {
    const long r = lroundf(val);
    if (r > 32767L) return 32767;
    if (r < -32768L) return -32768;
    return static_cast<int16_t>(r);
}

uint16_t clamp_u16(float val) {
    const long r = lroundf(val);
    if (r < 0L) return 0U;
    if (r > 65535L) return 65535U;
    return static_cast<uint16_t>(r);
}

void build_telemetry_frame(uint32_t now_ms) {
    const ImuState& imu = g_mpu.state();
    const PowerTempState& pwr = g_pwr.state();
    const LoraDiagnostics& lora_diag = g_lora.diagnostics();

    TelemetryFrame f;
    f.version = PROTO_VERSION;
    f.msg_type = PROTO_MSG_TELEMETRY;
    f.seq = g_seq_counter;
    f.timestamp_ms = now_ms;
    f.roll_cdeg = clamp_i16(imu.roll_deg * 100.0f);
    f.pitch_cdeg = clamp_i16(imu.pitch_deg * 100.0f);
    f.vib_rms_mg = clamp_u16(imu.vib_rms_mg);
    f.tof_mm = g_tof.is_range_valid() ? g_tof.distance_mm() : TOF_INVALID_SENTINEL;
    f.rpm = g_hall.rpm();
    f.vbat_mv = pwr.vbat_mv;
    f.ibat_ca = pwr.ibat_ca;
    f.consumed_mah = clamp_u16(pwr.consumed_mah);
    f.ntc_temp_cdegc = pwr.ntc_valid ? pwr.ntc_temp_cdegc : TEMP_INVALID_CDEGC;
    f.imu_temp_cdegc = clamp_i16(imu.die_temp_c * 100.0f);
    f.lora_rssi_dbm = lora_diag.last_rssi_dbm;
    f.tx_retries = lora_diag.last_tx_retries;
    f.cam_status = g_cam_status_code;

    uint8_t status = 0;
    if (g_mpu.is_healthy()) status |= STATUS_MPU_OK;
    if (imu.calibrated) status |= STATUS_IMU_CALIBRATED;
    if (g_tof.is_healthy()) status |= STATUS_TOF_OK;
    if (g_tof.is_range_valid()) status |= STATUS_TOF_RANGE_VALID;
    if (g_hall.is_active()) status |= STATUS_RPM_ACTIVE;
    if (pwr.vbat_valid && pwr.ntc_valid) status |= STATUS_POWER_NTC_OK;
    if (g_lora.is_healthy()) status |= STATUS_LORA_OK;
    if (g_cam_last_rx_ms != 0 && (now_ms - g_cam_last_rx_ms) < 5000UL) {
        status |= STATUS_CAM_LINK_OK;
    }
    f.sensor_status = status;

    g_latest_frame = f;
}

void print_cli_help() {
    Serial.println(F("\n=== FlightCore Telemetry CLI (STRICTLY TELEMETRY ONLY) ==="));
    Serial.println(F("  help           : Show available diagnostic commands"));
    Serial.println(F("  status         : Print sensor health, values, and RF diagnostics"));
    Serial.println(F("  json           : Toggle live JSON telemetry stream"));
    Serial.println(F("  bin            : Toggle live binary 37-byte frame stream"));
    Serial.println(F("  hex            : Print latest 37-byte binary frame in HEX"));
    Serial.println(F("  interval <ms>  : Set LoRa TX interval (100..10000 ms)"));
    Serial.println(F("  cal imu        : Restart MPU6050 zero-bias calibration"));
    Serial.println(F("  reset mah      : Reset integrated battery mAh counter"));
    Serial.println(F("  ppr <1..8>     : Set Hall sensor pulses per revolution"));
    Serial.println(F("  cam ping       : Send UART2 ping to ESP32-CAM bridge"));
    Serial.println(F("  cam status     : Print ESP32-CAM UART2 bridge diagnostics\n"));
}

void print_status(uint32_t now_ms) {
    const ImuState& imu = g_mpu.state();
    const TofDiagnostics& tof = g_tof.diagnostics();
    const RpmDiagnostics& rpm = g_hall.diagnostics();
    const PowerTempState& pwr = g_pwr.state();
    const LoraDiagnostics& lora = g_lora.diagnostics();

    Serial.println(F("--- FlightCore System Status ---"));
    Serial.printf("Uptime: %lu ms | TX Interval: %lu ms | Seq: %u\n",
                  static_cast<unsigned long>(now_ms),
                  static_cast<unsigned long>(g_telem_interval_ms),
                  static_cast<unsigned>(g_seq_counter));
    Serial.printf("MPU6050 [0x%02X]: healthy=%d cal=%d roll=%.2f deg pitch=%.2f deg vib=%.1f mg i2c_err=%lu\n",
                  g_mpu.who_am_i(), g_mpu.is_healthy(), imu.calibrated,
                  imu.roll_deg, imu.pitch_deg, imu.vib_rms_mg,
                  static_cast<unsigned long>(g_mpu.i2c_error_count()));
    Serial.printf("VL53L0X [0x%02X]: healthy=%d valid=%d dist=%u mm (med=%u) glitches=%lu timeouts=%lu\n",
                  g_tof.model_id(), tof.healthy, tof.range_valid,
                  tof.filtered_mm, tof.median_mm,
                  static_cast<unsigned long>(tof.glitch_rejected_count),
                  static_cast<unsigned long>(tof.timeout_count));
    Serial.printf("Hall RPM [PPR=%u]: active=%d rpm=%u pulses=%lu glitches=%lu\n",
                  g_hall.diagnostics().active ? 1 : 0, rpm.active, rpm.rpm,
                  static_cast<unsigned long>(rpm.total_pulses),
                  static_cast<unsigned long>(rpm.rejected_glitches));
    Serial.printf("Power/NTC: vbat=%.3f V (low=%d) ibat=%.2f A mah=%.2f ntc=%.2f C (valid=%d)\n",
                  pwr.vbat_v, pwr.low_battery_warn, pwr.ibat_a, pwr.consumed_mah,
                  pwr.ntc_temp_c, pwr.ntc_valid);
    Serial.printf("SX1278 [ver=0x%02X]: ok=%d tx_ok=%lu tx_att=%lu retries=%lu timeouts=%lu rssi=%d dBm\n",
                  lora.chip_version, lora.initialized,
                  static_cast<unsigned long>(lora.tx_success),
                  static_cast<unsigned long>(lora.tx_attempts),
                  static_cast<unsigned long>(lora.tx_retries_total),
                  static_cast<unsigned long>(lora.tx_timeouts),
                  static_cast<int>(lora.last_rssi_dbm));
    Serial.printf("CAM UART2: code=%u rx_lines=%lu last_rx_ms=%lu\n",
                  g_cam_status_code,
                  static_cast<unsigned long>(g_cam_rx_lines),
                  static_cast<unsigned long>(g_cam_last_rx_ms));
}

void print_latest_hex() {
    uint8_t wire[PROTO_FRAME_SIZE] = {0};
    const size_t n = encode_telemetry_frame(g_latest_frame, wire, sizeof(wire));
    Serial.print(F("FRAME_HEX:"));
    for (size_t i = 0; i < n; ++i) {
        Serial.printf(" %02X", wire[i]);
    }
    Serial.println();
}

void process_cli_line(const char* line, uint32_t now_ms) {
    if (strcmp(line, "help") == 0) {
        print_cli_help();
    } else if (strcmp(line, "status") == 0) {
        print_status(now_ms);
    } else if (strcmp(line, "json") == 0) {
        g_stream_json = !g_stream_json;
        if (g_stream_json) g_stream_bin = false;
        Serial.printf("JSON stream: %s\n", g_stream_json ? "ON" : "OFF");
    } else if (strcmp(line, "bin") == 0) {
        g_stream_bin = !g_stream_bin;
        if (g_stream_bin) g_stream_json = false;
        Serial.printf("Binary frame stream: %s\n", g_stream_bin ? "ON" : "OFF");
    } else if (strcmp(line, "hex") == 0) {
        build_telemetry_frame(now_ms);
        print_latest_hex();
    } else if (strncmp(line, "interval ", 9) == 0) {
        const long ms = strtol(line + 9, nullptr, 10);
        if (ms >= static_cast<long>(TELEM_MIN_INTERVAL_MS) &&
            ms <= static_cast<long>(TELEM_MAX_INTERVAL_MS)) {
            g_telem_interval_ms = static_cast<uint32_t>(ms);
            Serial.printf("LoRa TX interval set to %lu ms\n",
                          static_cast<unsigned long>(g_telem_interval_ms));
        } else {
            Serial.printf("ERR: interval must be %lu..%lu ms\n",
                          static_cast<unsigned long>(TELEM_MIN_INTERVAL_MS),
                          static_cast<unsigned long>(TELEM_MAX_INTERVAL_MS));
        }
    } else if (strcmp(line, "cal imu") == 0) {
        g_mpu.trigger_calibration(IMU_CAL_SAMPLES);
        Serial.println(F("MPU6050 zero-bias calibration started (keep airframe level)..."));
    } else if (strcmp(line, "reset mah") == 0) {
        g_pwr.reset_mah();
        Serial.println(F("Battery consumed mAh counter reset to 0."));
    } else if (strncmp(line, "ppr ", 4) == 0) {
        const long ppr = strtol(line + 4, nullptr, 10);
        if (ppr >= 1 && ppr <= 8) {
            g_hall.set_pulses_per_rev(static_cast<uint8_t>(ppr));
            Serial.printf("Hall PPR set to %ld\n", ppr);
        } else {
            Serial.println(F("ERR: PPR must be 1..8"));
        }
    } else if (strcmp(line, "cam ping") == 0) {
        Serial2.println(F("FC:PING"));
        Serial.println(F("Sent FC:PING on UART2 (GPIO17->CAM_RX)."));
    } else if (strcmp(line, "cam status") == 0) {
        Serial.printf("ESP32-CAM UART2 Stub: code=%u rx_lines=%lu last_rx=%lu ms\n",
                      g_cam_status_code,
                      static_cast<unsigned long>(g_cam_rx_lines),
                      static_cast<unsigned long>(g_cam_last_rx_ms));
    } else if (line[0] != '\0') {
        Serial.printf("Unknown command '%s'. Type 'help'.\n", line);
    }
}

void poll_serial_cli(uint32_t now_ms) {
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            if (g_cli_len > 0) {
                g_cli_buf[g_cli_len] = '\0';
                process_cli_line(g_cli_buf, now_ms);
                g_cli_len = 0;
            }
        } else if (g_cli_len < sizeof(g_cli_buf) - 1) {
            g_cli_buf[g_cli_len++] = static_cast<char>(c);
        }
    }
}

void poll_cam_uart_bridge(uint32_t now_ms) {
    while (Serial2.available() > 0) {
        const int c = Serial2.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            if (g_cam_rx_len > 0) {
                g_cam_rx_buf[g_cam_rx_len] = '\0';
                g_cam_last_rx_ms = now_ms;
                ++g_cam_rx_lines;
                if (strncmp(g_cam_rx_buf, "CAM:READY", 9) == 0) {
                    g_cam_status_code = 1;
                } else if (strncmp(g_cam_rx_buf, "CAM:STREAM", 10) == 0) {
                    g_cam_status_code = 2;
                }
                g_cam_rx_len = 0;
            }
        } else if (g_cam_rx_len < sizeof(g_cam_rx_buf) - 1) {
            g_cam_rx_buf[g_cam_rx_len++] = static_cast<char>(c);
        }
    }

    if ((now_ms - g_last_cam_hb_ms) >= CAM_HB_INTERVAL_MS) {
        g_last_cam_hb_ms = now_ms;
        // Telemetry summary frame pushed to ESP32-CAM OSD/metadata UART stub
        Serial2.printf("FC:TELEM,%u,%u,%u,%.2f\n",
                       static_cast<unsigned>(g_seq_counter),
                       static_cast<unsigned>(g_latest_frame.vbat_mv),
                       static_cast<unsigned>(g_latest_frame.tof_mm),
                       static_cast<double>(g_latest_frame.ntc_temp_cdegc) / 100.0);
    }
}

}  // namespace

void setup() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);
    pinMode(PIN_CAL_BUTTON, INPUT_PULLUP);

    Serial.begin(CLI_SERIAL_BAUD);
    delay(150);
    Serial.println(F("\n========================================================"));
    Serial.printf("%s v%s Initializing...\n", FW_NAME, FW_VERSION);
    Serial.println(F("SAFETY NOTICE: Telemetry & sensing ONLY (NO ESC/motor PWM)"));
    Serial.println(F("========================================================"));

    // Shared I2C Bus (GPIO21 SDA, GPIO22 SCL @ 400 kHz)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK_HZ);

    // ESP32-CAM UART2 Bridge Stub
    Serial2.begin(CAM_UART_BAUD, SERIAL_8N1, PIN_CAM_UART_RX, PIN_CAM_UART_TX);

    const bool mpu_ok = g_mpu.begin(MPU6050_I2C_ADDR);
    Serial.printf("[INIT] MPU6050 (0x%02X): %s\n", MPU6050_I2C_ADDR, mpu_ok ? "OK" : "FAULT");

    const bool tof_ok = g_tof.begin(VL53L0X_I2C_ADDR, PIN_VL53L0X_XSHUT);
    Serial.printf("[INIT] VL53L0X (0x%02X): %s\n", VL53L0X_I2C_ADDR, tof_ok ? "OK" : "FAULT");

    const bool hall_ok = g_hall.begin(PIN_HALL_RPM, HALL_DEFAULT_PULSES_PER_REV);
    Serial.printf("[INIT] Hall RPM (GPIO%u): %s\n", PIN_HALL_RPM, hall_ok ? "OK" : "FAULT");

    const bool pwr_ok = g_pwr.begin(PIN_ADC_VBAT, PIN_ADC_IBAT, PIN_ADC_NTC);
    Serial.printf("[INIT] Power/NTC ADC1: %s\n", pwr_ok ? "OK" : "FAULT");

    const bool lora_ok = g_lora.begin(LORA_FREQ_HZ,
                                      LORA_TX_POWER_DBM,
                                      LORA_SPREADING_FACTOR,
                                      LORA_BANDWIDTH_HZ,
                                      LORA_CODING_RATE,
                                      LORA_SYNC_WORD);
    Serial.printf("[INIT] SX1278 433MHz (ver=0x%02X): %s\n",
                  g_lora.diagnostics().chip_version,
                  lora_ok ? "OK" : "FAULT");

    print_cli_help();

    const uint32_t now = millis();
    g_last_imu_ms = now;
    g_last_tof_ms = now;
    g_last_pwr_ms = now;
    g_last_telem_ms = now;
    g_last_cam_hb_ms = now;
}

void loop() {
    const uint32_t now_ms = millis();

    // 1. 200 Hz MPU6050 Attitude & Vibration RMS Update
    if ((now_ms - g_last_imu_ms) >= IMU_LOOP_INTERVAL_MS) {
        const float dt_sec = static_cast<float>(now_ms - g_last_imu_ms) / 1000.0f;
        g_last_imu_ms = now_ms;
        g_mpu.poll(dt_sec);
    }

    // 2. 25 Hz VL53L0X ToF Ranging & Glitch Filter
    if ((now_ms - g_last_tof_ms) >= TOF_LOOP_INTERVAL_MS) {
        g_last_tof_ms = now_ms;
        g_tof.poll();
    }

    // 3. 20 Hz ADC1 Power/NTC + Hall RPM Update
    if ((now_ms - g_last_pwr_ms) >= PWR_LOOP_INTERVAL_MS) {
        const float dt_sec = static_cast<float>(now_ms - g_last_pwr_ms) / 1000.0f;
        g_last_pwr_ms = now_ms;
        g_pwr.poll(dt_sec);
        g_hall.poll();
    }

    // 4. Service LoRa TX state machine & incoming RX ACK/command packets
    g_lora.poll(now_ms);
    uint8_t rx_pkt[64] = {0};
    const size_t rx_len = g_lora.read_received_packet(rx_pkt, sizeof(rx_pkt));
    if (rx_len >= PROTO_FRAME_SIZE) {
        TelemetryFrame ack;
        if (decode_telemetry_frame(rx_pkt, rx_len, ack) == DecodeStatus::OK) {
            if (ack.msg_type == PROTO_MSG_ACK) {
                // RSSI/SNR automatically recorded in LoraSx1278::read_received_packet
            }
        }
    }

    // 5. Periodic Binary Telemetry Dispatch
    if ((now_ms - g_last_telem_ms) >= g_telem_interval_ms) {
        g_last_telem_ms = now_ms;
        build_telemetry_frame(now_ms);

        uint8_t wire[PROTO_FRAME_SIZE] = {0};
        const size_t n = encode_telemetry_frame(g_latest_frame, wire, sizeof(wire));
        if (n == PROTO_FRAME_SIZE) {
            if (g_lora.is_healthy() && !g_lora.is_busy()) {
                g_lora.start_transmit(wire, n, now_ms);
            }
            if (g_stream_bin) {
                Serial.write(wire, n);
            } else if (g_stream_json) {
                char json_buf[320] = {0};
                if (format_telemetry_json(g_latest_frame, json_buf, sizeof(json_buf)) > 0) {
                    Serial.println(json_buf);
                }
            }
            ++g_seq_counter;
        }
    }

    // 6. Calibration tactile button (GPIO4 active LOW, 50 ms debounce)
    const bool btn_now = digitalRead(PIN_CAL_BUTTON);
    if (btn_now == LOW && g_cal_btn_prev == HIGH) {
        g_cal_btn_press_ms = now_ms;
    } else if (btn_now == HIGH && g_cal_btn_prev == LOW) {
        if ((now_ms - g_cal_btn_press_ms) >= 50UL) {
            g_mpu.trigger_calibration(IMU_CAL_SAMPLES);
            Serial.println(F("[BTN] MPU6050 calibration triggered."));
        }
    }
    g_cal_btn_prev = btn_now;

    // 7. Heartbeat LED (faster blink during calibration or radio fault)
    const uint32_t led_period = g_mpu.is_calibrating() ? 100UL : (g_lora.is_healthy() ? 500UL : 200UL);
    if ((now_ms - g_last_led_ms) >= led_period) {
        g_last_led_ms = now_ms;
        g_led_state = !g_led_state;
        digitalWrite(PIN_STATUS_LED, g_led_state ? HIGH : LOW);
    }

    // 8. CLI & ESP32-CAM UART Bridge
    poll_serial_cli(now_ms);
    poll_cam_uart_bridge(now_ms);
}

#endif  // ARDUINO
