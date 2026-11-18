#pragma once

#include <stddef.h>
#include <stdint.h>
#include "config.h"

namespace flightcore {

/**
 * Packed logical representation of the 37-byte FlightCore binary telemetry frame.
 * Wire layout (Little-Endian):
 *   [0]     sync0          = 0xFC
 *   [1]     sync1          = 0x01
 *   [2]     version        = 0x01
 *   [3]     msg_type       = 0x01 (Telemetry), 0x02 (Diag), 0x03 (ACK)
 *   [4]     payload_len    = 30
 *   [5..6]  seq            (uint16)
 *   [7..10] timestamp_ms   (uint32)
 *   [11..12] roll_cdeg     (int16, 0.01 deg)
 *   [13..14] pitch_cdeg    (int16, 0.01 deg)
 *   [15..16] vib_rms_mg    (uint16, 0.001 g)
 *   [17..18] tof_mm        (uint16, mm; 0xFFFF invalid)
 *   [19..20] rpm           (uint16, RPM)
 *   [21..22] vbat_mv       (uint16, mV)
 *   [23..24] ibat_ca       (int16, 0.01 A)
 *   [25..26] consumed_mah  (uint16, mAh)
 *   [27..28] ntc_temp_cdegc(int16, 0.01 C; -32768 invalid)
 *   [29..30] imu_temp_cdegc(int16, 0.01 C)
 *   [31]    lora_rssi_dbm  (int8, dBm)
 *   [32]    sensor_status  (uint8 bitmask)
 *   [33]    tx_retries     (uint8)
 *   [34]    cam_status     (uint8)
 *   [35..36] crc16         (uint16, CRC-16-CCITT over bytes [0..34])
 */
struct TelemetryFrame {
    uint8_t version = PROTO_VERSION;
    uint8_t msg_type = PROTO_MSG_TELEMETRY;
    uint16_t seq = 0;
    uint32_t timestamp_ms = 0;
    int16_t roll_cdeg = 0;
    int16_t pitch_cdeg = 0;
    uint16_t vib_rms_mg = 0;
    uint16_t tof_mm = TOF_INVALID_SENTINEL;
    uint16_t rpm = 0;
    uint16_t vbat_mv = 0;
    int16_t ibat_ca = 0;
    uint16_t consumed_mah = 0;
    int16_t ntc_temp_cdegc = TEMP_INVALID_CDEGC;
    int16_t imu_temp_cdegc = 0;
    int8_t lora_rssi_dbm = 0;
    uint8_t sensor_status = 0;
    uint8_t tx_retries = 0;
    uint8_t cam_status = 0;
};

enum class DecodeStatus : uint8_t {
    OK = 0,
    ERR_LENGTH,
    ERR_SYNC,
    ERR_VERSION,
    ERR_PAYLOAD_LEN,
    ERR_CRC
};

/**
 * Computes CRC-16-CCITT (polynomial 0x1021, init 0xFFFF, no reflection, xorout 0x0000).
 */
uint16_t crc16_ccitt(const uint8_t* data, size_t length);

/**
 * Serializes a TelemetryFrame into a 37-byte wire buffer with appended CRC-16-CCITT.
 * Returns PROTO_FRAME_SIZE (37) on success, or 0 if out_buf is null or max_len < 37.
 */
size_t encode_telemetry_frame(const TelemetryFrame& frame, uint8_t* out_buf, size_t max_len);

/**
 * Validates and deserializes a 37-byte wire buffer into TelemetryFrame.
 */
DecodeStatus decode_telemetry_frame(const uint8_t* buf, size_t len, TelemetryFrame& out_frame);

/**
 * Formats a TelemetryFrame into a compact JSON string for Serial CLI diagnostics.
 * Returns bytes written (excluding null terminator), or 0 if buffer is insufficient.
 */
size_t format_telemetry_json(const TelemetryFrame& frame, char* out_buf, size_t max_len);

/**
 * Stateful byte-by-byte stream resynchronizer for noisy UART / radio streams.
 */
class FrameStreamParser {
public:
    FrameStreamParser();
    void reset();

    /**
     * Feeds one byte into the parser.
     * Returns true when a valid CRC-verified TelemetryFrame is assembled in out_frame.
     */
    bool feed(uint8_t byte, TelemetryFrame& out_frame);

    uint32_t valid_frames() const { return valid_frames_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t header_resyncs() const { return header_resyncs_; }

private:
    uint8_t buf_[PROTO_FRAME_SIZE];
    size_t idx_;
    uint32_t valid_frames_;
    uint32_t crc_errors_;
    uint32_t header_resyncs_;
};

}  // namespace flightcore
