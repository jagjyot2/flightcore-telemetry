"""
FlightCore Binary Telemetry Protocol (Python Twin of firmware/src/telemetry_protocol.cpp).

Wire Format (37 bytes, Little-Endian):
  [0..1]   sync          : 0xFC 0x01
  [2]      version       : 0x01
  [3]      msg_type      : 0x01 (Telemetry), 0x02 (Diagnostics), 0x03 (ACK)
  [4]      payload_len   : 30
  [5..6]   seq           : uint16
  [7..10]  timestamp_ms  : uint32
  [11..12] roll_cdeg     : int16 (0.01 deg)
  [13..14] pitch_cdeg    : int16 (0.01 deg)
  [15..16] vib_rms_mg    : uint16 (0.001 g)
  [17..18] tof_mm        : uint16 (mm, 0xFFFF = invalid)
  [19..20] rpm           : uint16 (RPM)
  [21..22] vbat_mv       : uint16 (mV)
  [23..24] ibat_ca       : int16 (0.01 A)
  [25..26] consumed_mah  : uint16 (mAh)
  [27..28] ntc_temp_cdegc: int16 (0.01 C, -32768 = invalid)
  [29..30] imu_temp_cdegc: int16 (0.01 C)
  [31]     lora_rssi_dbm : int8 (dBm)
  [32]     sensor_status : uint8 bitmask
  [33]     tx_retries    : uint8
  [34]     cam_status    : uint8
  [35..36] crc16         : uint16 CRC-16-CCITT (poly=0x1021, init=0xFFFF) over bytes [0..34]
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import struct
from typing import Dict, List, Optional

SYNC_0 = 0xFC
SYNC_1 = 0x01
SYNC_BYTES = bytes([SYNC_0, SYNC_1])
PROTO_VERSION = 0x01
MSG_TELEMETRY = 0x01
MSG_DIAGNOSTICS = 0x02
MSG_ACK = 0x03
PAYLOAD_LEN = 30
FRAME_SIZE = 37

TOF_INVALID_SENTINEL = 0xFFFF
TEMP_INVALID_CDEGC = -32768

# 5-byte header + 30-byte body = 35 bytes
_BODY_STRUCT = struct.Struct("<BBBBBHIhhHHHHhHhhbBBB")
# 35-byte body + 2-byte CRC16 = 37 bytes
_FRAME_STRUCT = struct.Struct("<BBBBBHIhhHHHHhHhhbBBBH")

assert _BODY_STRUCT.size == FRAME_SIZE - 2
assert _FRAME_STRUCT.size == FRAME_SIZE

STATUS_BITS: Dict[int, str] = {
    0: "MPU_OK",
    1: "IMU_CALIBRATED",
    2: "TOF_OK",
    3: "TOF_RANGE_VALID",
    4: "RPM_ACTIVE",
    5: "POWER_NTC_OK",
    6: "LORA_OK",
    7: "CAM_LINK_OK",
}


class FrameDecodeError(ValueError):
    """Raised when a telemetry frame fails structural or CRC-16 validation."""


def crc16_ccitt(data: bytes) -> int:
    """Computes CRC-16-CCITT (poly=0x1021, init=0xFFFF, non-reflected, xorout=0)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class TelemetryFrame:
    seq: int = 0
    timestamp_ms: int = 0
    roll_cdeg: int = 0
    pitch_cdeg: int = 0
    vib_rms_mg: int = 0
    tof_mm: int = TOF_INVALID_SENTINEL
    rpm: int = 0
    vbat_mv: int = 0
    ibat_ca: int = 0
    consumed_mah: int = 0
    ntc_temp_cdegc: int = TEMP_INVALID_CDEGC
    imu_temp_cdegc: int = 0
    lora_rssi_dbm: int = 0
    sensor_status: int = 0
    tx_retries: int = 0
    cam_status: int = 0
    version: int = PROTO_VERSION
    msg_type: int = MSG_TELEMETRY

    @property
    def roll_deg(self) -> float:
        return self.roll_cdeg / 100.0

    @property
    def pitch_deg(self) -> float:
        return self.pitch_cdeg / 100.0

    @property
    def vbat_v(self) -> float:
        return self.vbat_mv / 1000.0

    @property
    def ibat_a(self) -> float:
        return self.ibat_ca / 100.0

    @property
    def tof_valid(self) -> bool:
        return self.tof_mm != TOF_INVALID_SENTINEL

    @property
    def ntc_valid(self) -> bool:
        return self.ntc_temp_cdegc != TEMP_INVALID_CDEGC

    @property
    def ntc_temp_c(self) -> Optional[float]:
        return None if not self.ntc_valid else (self.ntc_temp_cdegc / 100.0)

    @property
    def imu_temp_c(self) -> float:
        return self.imu_temp_cdegc / 100.0

    @property
    def status_labels(self) -> List[str]:
        return [name for bit, name in STATUS_BITS.items() if (self.sensor_status & (1 << bit))]

    def encode(self) -> bytes:
        return encode_frame(self)

    def to_dict(self) -> Dict[str, object]:
        data = asdict(self)
        data.update(
            {
                "roll_deg": round(self.roll_deg, 2),
                "pitch_deg": round(self.pitch_deg, 2),
                "vbat_v": round(self.vbat_v, 3),
                "ibat_a": round(self.ibat_a, 2),
                "ntc_temp_c": None if self.ntc_temp_c is None else round(self.ntc_temp_c, 2),
                "imu_temp_c": round(self.imu_temp_c, 2),
                "tof_valid": self.tof_valid,
                "ntc_valid": self.ntc_valid,
                "status_flags": self.status_labels,
            }
        )
        return data

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))


def encode_frame(frame: TelemetryFrame) -> bytes:
    """Serializes TelemetryFrame into a 37-byte CRC-16-CCITT protected binary frame."""
    body = _BODY_STRUCT.pack(
        SYNC_0,
        SYNC_1,
        frame.version,
        frame.msg_type,
        PAYLOAD_LEN,
        frame.seq,
        frame.timestamp_ms,
        frame.roll_cdeg,
        frame.pitch_cdeg,
        frame.vib_rms_mg,
        frame.tof_mm,
        frame.rpm,
        frame.vbat_mv,
        frame.ibat_ca,
        frame.consumed_mah,
        frame.ntc_temp_cdegc,
        frame.imu_temp_cdegc,
        frame.lora_rssi_dbm,
        frame.sensor_status,
        frame.tx_retries,
        frame.cam_status,
    )
    crc = crc16_ccitt(body)
    return body + struct.pack("<H", crc)


def decode_frame(data: bytes) -> TelemetryFrame:
    """Validates sync, header, and CRC-16-CCITT and returns a TelemetryFrame."""
    if len(data) != FRAME_SIZE:
        raise FrameDecodeError(f"Expected {FRAME_SIZE} bytes, got {len(data)}")

    (
        sync0,
        sync1,
        version,
        msg_type,
        payload_len,
        seq,
        timestamp_ms,
        roll_cdeg,
        pitch_cdeg,
        vib_rms_mg,
        tof_mm,
        rpm,
        vbat_mv,
        ibat_ca,
        consumed_mah,
        ntc_temp_cdegc,
        imu_temp_cdegc,
        lora_rssi_dbm,
        sensor_status,
        tx_retries,
        cam_status,
        wire_crc,
    ) = _FRAME_STRUCT.unpack(data)

    if sync0 != SYNC_0 or sync1 != SYNC_1:
        raise FrameDecodeError(f"Invalid sync header: 0x{sync0:02X} 0x{sync1:02X}")
    if version != PROTO_VERSION:
        raise FrameDecodeError(f"Unsupported protocol version: {version}")
    if payload_len != PAYLOAD_LEN:
        raise FrameDecodeError(f"Invalid payload length: {payload_len}")

    expected_crc = crc16_ccitt(data[:-2])
    if wire_crc != expected_crc:
        raise FrameDecodeError(
            f"CRC-16 mismatch: wire=0x{wire_crc:04X}, expected=0x{expected_crc:04X}"
        )

    return TelemetryFrame(
        version=version,
        msg_type=msg_type,
        seq=seq,
        timestamp_ms=timestamp_ms,
        roll_cdeg=roll_cdeg,
        pitch_cdeg=pitch_cdeg,
        vib_rms_mg=vib_rms_mg,
        tof_mm=tof_mm,
        rpm=rpm,
        vbat_mv=vbat_mv,
        ibat_ca=ibat_ca,
        consumed_mah=consumed_mah,
        ntc_temp_cdegc=ntc_temp_cdegc,
        imu_temp_cdegc=imu_temp_cdegc,
        lora_rssi_dbm=lora_rssi_dbm,
        sensor_status=sensor_status,
        tx_retries=tx_retries,
        cam_status=cam_status,
    )


class StreamDecoder:
    """Stateful resynchronizing parser for noisy serial/RF byte streams."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.valid_frames: int = 0
        self.crc_errors: int = 0
        self.resync_count: int = 0
        self.dropped_packets: int = 0
        self._last_seq: Optional[int] = None

    def reset(self) -> None:
        self._buf.clear()

    def feed_bytes(self, chunk: bytes) -> List[TelemetryFrame]:
        frames: List[TelemetryFrame] = []
        self._buf.extend(chunk)

        while len(self._buf) >= FRAME_SIZE:
            idx = self._buf.find(SYNC_BYTES)
            if idx == -1:
                # Preserve trailing 0xFC byte in case next chunk starts with 0x01
                keep_tail = self._buf.endswith(bytes([SYNC_0]))
                discarded = len(self._buf) - (1 if keep_tail else 0)
                if discarded > 0:
                    self.resync_count += 1
                self._buf = bytearray([SYNC_0]) if keep_tail else bytearray()
                break

            if idx > 0:
                self.resync_count += 1
                del self._buf[:idx]

            if len(self._buf) < FRAME_SIZE:
                break

            candidate = bytes(self._buf[:FRAME_SIZE])
            try:
                frame = decode_frame(candidate)
            except FrameDecodeError as exc:
                if "CRC-16" in str(exc):
                    self.crc_errors += 1
                else:
                    self.resync_count += 1
                del self._buf[:1]
                continue

            del self._buf[:FRAME_SIZE]
            self.valid_frames += 1
            if self._last_seq is not None:
                delta = (frame.seq - self._last_seq) & 0xFFFF
                if delta > 1:
                    self.dropped_packets += delta - 1
            self._last_seq = frame.seq
            frames.append(frame)

        return frames
