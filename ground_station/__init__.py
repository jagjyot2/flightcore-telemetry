"""FlightCore Ground Station Telemetry Decoder & CLI Package."""

from .protocol import (
    FRAME_SIZE,
    PAYLOAD_LEN,
    PROTO_VERSION,
    STATUS_BITS,
    SYNC_BYTES,
    FrameDecodeError,
    StreamDecoder,
    TelemetryFrame,
    crc16_ccitt,
    decode_frame,
    encode_frame,
)

__all__ = [
    "FRAME_SIZE",
    "PAYLOAD_LEN",
    "PROTO_VERSION",
    "STATUS_BITS",
    "SYNC_BYTES",
    "FrameDecodeError",
    "StreamDecoder",
    "TelemetryFrame",
    "crc16_ccitt",
    "decode_frame",
    "encode_frame",
]
