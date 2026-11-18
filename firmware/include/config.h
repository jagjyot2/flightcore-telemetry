#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * FlightCore Master Hardware & Calibration Configuration
 *
 * SAFETY BOUNDARY:
 * FlightCore is strictly a sensor acquisition and 433 MHz LoRa telemetry
 * expansion system. It does NOT generate motor/ESC PWM signals or perform
 * flight stabilization control.
 */
#ifndef FLIGHTCORE_STRICTLY_TELEMETRY_ONLY
#define FLIGHTCORE_STRICTLY_TELEMETRY_ONLY 1
#endif

namespace flightcore {

// ---------------------------------------------------------------------------
// Firmware Identity
// ---------------------------------------------------------------------------
constexpr const char* FW_NAME = "FlightCore-Telemetry";
constexpr const char* FW_VERSION = "1.0.0";

// ---------------------------------------------------------------------------
// ESP32 DevKit v1 Pin Mapping (Single Source of Truth)
// Note: Strapping pins GPIO0, GPIO12, GPIO15 and flash pins GPIO6-11 avoided.
// ---------------------------------------------------------------------------
// Shared I2C Bus (400 kHz, 4.7k pull-ups to +3V3_BUS)
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint32_t I2C_CLOCK_HZ = 400000UL;

// MPU6050 6-Axis IMU (I2C 0x68, AD0=GND)
constexpr uint8_t MPU6050_I2C_ADDR = 0x68;
constexpr uint8_t PIN_MPU6050_INT = 32;

// VL53L0X Time-of-Flight Laser Ranging (I2C 0x29)
constexpr uint8_t VL53L0X_I2C_ADDR = 0x29;
constexpr uint8_t PIN_VL53L0X_XSHUT = 26;
constexpr uint8_t PIN_VL53L0X_INT = 25;

// SX1278 433 MHz LoRa Transceiver (VSPI Bus)
constexpr uint8_t PIN_LORA_SCK = 18;
constexpr uint8_t PIN_LORA_MISO = 19;
constexpr uint8_t PIN_LORA_MOSI = 23;
constexpr uint8_t PIN_LORA_CS = 5;
constexpr uint8_t PIN_LORA_RST = 14;
constexpr uint8_t PIN_LORA_DIO0 = 27;

// A3144 Hall-Effect RPM Sensor (Open-collector, 10k pull-up to +3V3_BUS)
constexpr uint8_t PIN_HALL_RPM = 33;

// Analog Sensing Channels (Strictly ADC1 to avoid Wi-Fi/ADC2 conflicts)
constexpr uint8_t PIN_ADC_VBAT = 34;  // ADC1_CH6 (Input-only)
constexpr uint8_t PIN_ADC_IBAT = 35;  // ADC1_CH7 (Input-only)
constexpr uint8_t PIN_ADC_NTC = 36;   // ADC1_CH0 / VP (Input-only)

// ESP32-CAM Telemetry Bridge (UART2)
constexpr uint8_t PIN_CAM_UART_RX = 16;
constexpr uint8_t PIN_CAM_UART_TX = 17;
constexpr uint32_t CAM_UART_BAUD = 115200UL;

// Diagnostics UI
constexpr uint8_t PIN_STATUS_LED = 2;
constexpr uint8_t PIN_CAL_BUTTON = 4;
constexpr uint32_t CLI_SERIAL_BAUD = 115200UL;

// ---------------------------------------------------------------------------
// Sensor Scaling & Filtering Constants
// ---------------------------------------------------------------------------
// MPU6050 (+/-4g accel, +/-500 deg/s gyro)
constexpr float MPU_ACCEL_LSB_PER_G = 8192.0f;
constexpr float MPU_GYRO_LSB_PER_DPS = 65.5f;
constexpr float IMU_COMPLEMENTARY_ALPHA = 0.98f;
constexpr uint16_t IMU_CAL_SAMPLES = 250;
constexpr uint8_t IMU_VIB_WINDOW_SAMPLES = 32;

// VL53L0X ToF Ranging
constexpr uint8_t TOF_MEDIAN_WINDOW = 5;
constexpr float TOF_EMA_ALPHA = 0.35f;
constexpr uint16_t TOF_MIN_VALID_MM = 30;
constexpr uint16_t TOF_MAX_VALID_MM = 2000;
constexpr uint16_t TOF_INVALID_SENTINEL = 0xFFFFU;
constexpr uint32_t TOF_READ_TIMEOUT_MS = 35;

// Hall-Effect RPM
constexpr uint8_t HALL_DEFAULT_PULSES_PER_REV = 1;
constexpr uint32_t HALL_DEBOUNCE_US = 350;
constexpr uint32_t HALL_TIMEOUT_MS = 1000;
constexpr uint16_t HALL_RPM_MAX = 40000;
constexpr float HALL_EMA_ALPHA = 0.40f;

// Power & NTC Steinhart-Hart Constants (10k NTC, B3950/Vishay NTCLE100E3103 class)
constexpr float ADC_VREF_MV = 3300.0f;
constexpr float VBAT_DIVIDER_RATIO = 11.0f;         // 100k top / 10k bottom (up to ~28V / 6S LiPo)
constexpr float CURRENT_SHUNT_OHMS = 0.001f;        // 1 mOhm 2512 3W shunt
constexpr float CURRENT_AMP_GAIN = 50.0f;           // INA180A2 (50 V/V -> 50 mV/A)
constexpr float CURRENT_ZERO_OFFSET_MV = 0.0f;      // Unidirectional zero-offset calibration
constexpr float CURRENT_DEADBAND_A = 0.05f;         // Suppress idle ADC noise < 50 mA
constexpr float NTC_SERIES_RESISTOR_OHMS = 10000.0f;// 10k 1% pull-up to 3.3V
constexpr float NTC_SH_A = 1.127125950e-3f;
constexpr float NTC_SH_B = 2.344443098e-4f;
constexpr float NTC_SH_C = 8.649322763e-8f;
constexpr int16_t TEMP_INVALID_CDEGC = -32768;      // INT16_MIN indicates thermistor fault
constexpr uint8_t ADC_OVERSAMPLE_COUNT = 16;

// ---------------------------------------------------------------------------
// LoRa SX1278 433 MHz Profile & Scheduler Intervals
// ---------------------------------------------------------------------------
constexpr uint32_t LORA_FREQ_HZ = 433000000UL;
constexpr uint8_t LORA_TX_POWER_DBM = 17;           // +17 dBm safe default (PA_BOOST)
constexpr uint8_t LORA_SPREADING_FACTOR = 7;        // SF7
constexpr uint32_t LORA_BANDWIDTH_HZ = 125000UL;    // 125 kHz
constexpr uint8_t LORA_CODING_RATE = 5;             // 4/5
constexpr uint8_t LORA_SYNC_WORD = 0x12;            // Private network sync word
constexpr uint8_t LORA_MAX_TX_RETRIES = 3;
constexpr uint32_t LORA_TX_TIMEOUT_MS = 400;

constexpr uint32_t IMU_LOOP_INTERVAL_MS = 5;        // 200 Hz
constexpr uint32_t TOF_LOOP_INTERVAL_MS = 40;       // 25 Hz
constexpr uint32_t PWR_LOOP_INTERVAL_MS = 50;       // 20 Hz
constexpr uint32_t TELEM_DEFAULT_INTERVAL_MS = 200; // 5 Hz configurable
constexpr uint32_t TELEM_MIN_INTERVAL_MS = 100;
constexpr uint32_t TELEM_MAX_INTERVAL_MS = 10000;
constexpr uint32_t CAM_HB_INTERVAL_MS = 1000;

// ---------------------------------------------------------------------------
// Telemetry Binary Protocol Constants
// ---------------------------------------------------------------------------
constexpr uint8_t PROTO_SYNC_0 = 0xFC;
constexpr uint8_t PROTO_SYNC_1 = 0x01;
constexpr uint8_t PROTO_VERSION = 0x01;
constexpr uint8_t PROTO_MSG_TELEMETRY = 0x01;
constexpr uint8_t PROTO_MSG_DIAGNOSTICS = 0x02;
constexpr uint8_t PROTO_MSG_ACK = 0x03;
constexpr size_t PROTO_PAYLOAD_LEN = 30;
constexpr size_t PROTO_FRAME_SIZE = 37;             // 5 header + 30 body + 2 CRC-16-CCITT

// Sensor status bitmask flags (TelemetryFrame::sensor_status)
constexpr uint8_t STATUS_MPU_OK = (1U << 0);
constexpr uint8_t STATUS_IMU_CALIBRATED = (1U << 1);
constexpr uint8_t STATUS_TOF_OK = (1U << 2);
constexpr uint8_t STATUS_TOF_RANGE_VALID = (1U << 3);
constexpr uint8_t STATUS_RPM_ACTIVE = (1U << 4);
constexpr uint8_t STATUS_POWER_NTC_OK = (1U << 5);
constexpr uint8_t STATUS_LORA_OK = (1U << 6);
constexpr uint8_t STATUS_CAM_LINK_OK = (1U << 7);

}  // namespace flightcore
