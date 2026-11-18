#include "telemetry_protocol.h"
#include <stdio.h>
#include <string.h>

namespace flightcore {

namespace {

inline void write_u16_le(uint8_t* dst, uint16_t val) {
    dst[0] = static_cast<uint8_t>(val & 0xFFU);
    dst[1] = static_cast<uint8_t>((val >> 8) & 0xFFU);
}

inline void write_i16_le(uint8_t* dst, int16_t val) {
    write_u16_le(dst, static_cast<uint16_t>(val));
}

inline void write_u32_le(uint8_t* dst, uint32_t val) {
    dst[0] = static_cast<uint8_t>(val & 0xFFUL);
    dst[1] = static_cast<uint8_t>((val >> 8) & 0xFFUL);
    dst[2] = static_cast<uint8_t>((val >> 16) & 0xFFUL);
    dst[3] = static_cast<uint8_t>((val >> 24) & 0xFFUL);
}

inline uint16_t read_u16_le(const uint8_t* src) {
    return static_cast<uint16_t>(src[0]) |
           (static_cast<uint16_t>(src[1]) << 8);
}

inline int16_t read_i16_le(const uint8_t* src) {
    return static_cast<int16_t>(read_u16_le(src));
}

inline uint32_t read_u32_le(const uint8_t* src) {
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
}

}  // namespace

uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFFU;
    if (!data) {
        return crc;
    }
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000U) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

size_t encode_telemetry_frame(const TelemetryFrame& frame, uint8_t* out_buf, size_t max_len) {
    if (!out_buf || max_len < PROTO_FRAME_SIZE) {
        return 0;
    }

    out_buf[0] = PROTO_SYNC_0;
    out_buf[1] = PROTO_SYNC_1;
    out_buf[2] = frame.version;
    out_buf[3] = frame.msg_type;
    out_buf[4] = static_cast<uint8_t>(PROTO_PAYLOAD_LEN);

    write_u16_le(&out_buf[5], frame.seq);
    write_u32_le(&out_buf[7], frame.timestamp_ms);
    write_i16_le(&out_buf[11], frame.roll_cdeg);
    write_i16_le(&out_buf[13], frame.pitch_cdeg);
    write_u16_le(&out_buf[15], frame.vib_rms_mg);
    write_u16_le(&out_buf[17], frame.tof_mm);
    write_u16_le(&out_buf[19], frame.rpm);
    write_u16_le(&out_buf[21], frame.vbat_mv);
    write_i16_le(&out_buf[23], frame.ibat_ca);
    write_u16_le(&out_buf[25], frame.consumed_mah);
    write_i16_le(&out_buf[27], frame.ntc_temp_cdegc);
    write_i16_le(&out_buf[29], frame.imu_temp_cdegc);
    out_buf[31] = static_cast<uint8_t>(frame.lora_rssi_dbm);
    out_buf[32] = frame.sensor_status;
    out_buf[33] = frame.tx_retries;
    out_buf[34] = frame.cam_status;

    const uint16_t crc = crc16_ccitt(out_buf, PROTO_FRAME_SIZE - 2);
    write_u16_le(&out_buf[35], crc);

    return PROTO_FRAME_SIZE;
}

DecodeStatus decode_telemetry_frame(const uint8_t* buf, size_t len, TelemetryFrame& out_frame) {
    if (!buf || len < PROTO_FRAME_SIZE) {
        return DecodeStatus::ERR_LENGTH;
    }
    if (buf[0] != PROTO_SYNC_0 || buf[1] != PROTO_SYNC_1) {
        return DecodeStatus::ERR_SYNC;
    }
    if (buf[2] != PROTO_VERSION) {
        return DecodeStatus::ERR_VERSION;
    }
    if (buf[4] != static_cast<uint8_t>(PROTO_PAYLOAD_LEN)) {
        return DecodeStatus::ERR_PAYLOAD_LEN;
    }

    const uint16_t expected_crc = crc16_ccitt(buf, PROTO_FRAME_SIZE - 2);
    const uint16_t wire_crc = read_u16_le(&buf[35]);
    if (expected_crc != wire_crc) {
        return DecodeStatus::ERR_CRC;
    }

    out_frame.version = buf[2];
    out_frame.msg_type = buf[3];
    out_frame.seq = read_u16_le(&buf[5]);
    out_frame.timestamp_ms = read_u32_le(&buf[7]);
    out_frame.roll_cdeg = read_i16_le(&buf[11]);
    out_frame.pitch_cdeg = read_i16_le(&buf[13]);
    out_frame.vib_rms_mg = read_u16_le(&buf[15]);
    out_frame.tof_mm = read_u16_le(&buf[17]);
    out_frame.rpm = read_u16_le(&buf[19]);
    out_frame.vbat_mv = read_u16_le(&buf[21]);
    out_frame.ibat_ca = read_i16_le(&buf[23]);
    out_frame.consumed_mah = read_u16_le(&buf[25]);
    out_frame.ntc_temp_cdegc = read_i16_le(&buf[27]);
    out_frame.imu_temp_cdegc = read_i16_le(&buf[29]);
    out_frame.lora_rssi_dbm = static_cast<int8_t>(buf[31]);
    out_frame.sensor_status = buf[32];
    out_frame.tx_retries = buf[33];
    out_frame.cam_status = buf[34];

    return DecodeStatus::OK;
}

size_t format_telemetry_json(const TelemetryFrame& frame, char* out_buf, size_t max_len) {
    if (!out_buf || max_len == 0) {
        return 0;
    }
    const double roll_deg = static_cast<double>(frame.roll_cdeg) / 100.0;
    const double pitch_deg = static_cast<double>(frame.pitch_cdeg) / 100.0;
    const double vbat_v = static_cast<double>(frame.vbat_mv) / 1000.0;
    const double ibat_a = static_cast<double>(frame.ibat_ca) / 100.0;
    const double ntc_c = (frame.ntc_temp_cdegc == TEMP_INVALID_CDEGC)
                             ? -999.0
                             : (static_cast<double>(frame.ntc_temp_cdegc) / 100.0);
    const double imu_c = static_cast<double>(frame.imu_temp_cdegc) / 100.0;

    int written = snprintf(
        out_buf,
        max_len,
        "{\"seq\":%u,\"ts_ms\":%lu,\"roll_deg\":%.2f,\"pitch_deg\":%.2f,"
        "\"vib_mg\":%u,\"tof_mm\":%u,\"rpm\":%u,\"vbat_v\":%.3f,"
        "\"ibat_a\":%.2f,\"mah\":%u,\"ntc_c\":%.2f,\"imu_c\":%.2f,"
        "\"rssi_dbm\":%d,\"status\":\"0x%02X\",\"retries\":%u,\"cam\":%u}",
        static_cast<unsigned>(frame.seq),
        static_cast<unsigned long>(frame.timestamp_ms),
        roll_deg,
        pitch_deg,
        static_cast<unsigned>(frame.vib_rms_mg),
        static_cast<unsigned>(frame.tof_mm),
        static_cast<unsigned>(frame.rpm),
        vbat_v,
        ibat_a,
        static_cast<unsigned>(frame.consumed_mah),
        ntc_c,
        imu_c,
        static_cast<int>(frame.lora_rssi_dbm),
        static_cast<unsigned>(frame.sensor_status),
        static_cast<unsigned>(frame.tx_retries),
        static_cast<unsigned>(frame.cam_status));

    if (written < 0 || static_cast<size_t>(written) >= max_len) {
        out_buf[max_len - 1] = '\0';
        return 0;
    }
    return static_cast<size_t>(written);
}

FrameStreamParser::FrameStreamParser()
    : idx_(0), valid_frames_(0), crc_errors_(0), header_resyncs_(0) {
    memset(buf_, 0, sizeof(buf_));
}

void FrameStreamParser::reset() {
    idx_ = 0;
    memset(buf_, 0, sizeof(buf_));
}

bool FrameStreamParser::feed(uint8_t byte, TelemetryFrame& out_frame) {
    if (idx_ == 0) {
        if (byte == PROTO_SYNC_0) {
            buf_[0] = byte;
            idx_ = 1;
        }
        return false;
    }

    if (idx_ == 1) {
        if (byte == PROTO_SYNC_1) {
            buf_[1] = byte;
            idx_ = 2;
        } else if (byte == PROTO_SYNC_0) {
            buf_[0] = byte;
            idx_ = 1;
            ++header_resyncs_;
        } else {
            idx_ = 0;
            ++header_resyncs_;
        }
        return false;
    }

    buf_[idx_++] = byte;
    if (idx_ < PROTO_FRAME_SIZE) {
        return false;
    }

    const DecodeStatus status = decode_telemetry_frame(buf_, PROTO_FRAME_SIZE, out_frame);
    if (status == DecodeStatus::OK) {
        ++valid_frames_;
        idx_ = 0;
        return true;
    }

    if (status == DecodeStatus::ERR_CRC) {
        ++crc_errors_;
    } else {
        ++header_resyncs_;
    }

    // Shift buffer by 1 byte and search for next 0xFC 0x01 sync sequence
    for (size_t i = 1; i < PROTO_FRAME_SIZE; ++i) {
        if (buf_[i] == PROTO_SYNC_0) {
            const size_t remaining = PROTO_FRAME_SIZE - i;
            if (remaining >= 2 && buf_[i + 1] != PROTO_SYNC_1) {
                continue;
            }
            memmove(buf_, &buf_[i], remaining);
            idx_ = remaining;
            return false;
        }
    }

    idx_ = 0;
    return false;
}

}  // namespace flightcore
