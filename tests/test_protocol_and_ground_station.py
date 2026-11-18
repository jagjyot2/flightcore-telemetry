"""End-to-end verification for FlightCore binary CRC-16-CCITT protocol and ground station CLI."""

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ground_station.protocol import (  # noqa: E402
    FRAME_SIZE,
    PROTO_VERSION,
    SYNC_BYTES,
    TEMP_INVALID_CDEGC,
    TOF_INVALID_SENTINEL,
    FrameDecodeError,
    StreamDecoder,
    TelemetryFrame,
    crc16_ccitt,
    decode_frame,
    encode_frame,
)
from ground_station.receiver_cli import (  # noqa: E402
    generate_simulated_stream,
    main as cli_main,
    parse_hex_payload,
)


def test_crc16_ccitt_standard_test_vector() -> None:
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_encode_decode_roundtrip_and_wire_layout() -> None:
    tx = TelemetryFrame(
        seq=1025,
        timestamp_ms=987654,
        roll_cdeg=-1542,
        pitch_cdeg=835,
        vib_rms_mg=215,
        tof_mm=745,
        rpm=9120,
        vbat_mv=14920,
        ibat_ca=2245,
        consumed_mah=842,
        ntc_temp_cdegc=3840,
        imu_temp_cdegc=3510,
        lora_rssi_dbm=-79,
        sensor_status=0x7F,
        tx_retries=2,
        cam_status=1,
    )
    wire = encode_frame(tx)
    assert len(wire) == FRAME_SIZE == 37
    assert wire[0:2] == SYNC_BYTES
    assert wire[2] == PROTO_VERSION
    assert wire[4] == 30

    rx = decode_frame(wire)
    assert rx == tx
    assert rx.roll_deg == pytest.approx(-15.42)
    assert rx.pitch_deg == pytest.approx(8.35)
    assert rx.vbat_v == pytest.approx(14.92)
    assert rx.ibat_a == pytest.approx(22.45)
    assert rx.ntc_temp_c == pytest.approx(38.40)
    assert rx.tof_valid is True
    assert rx.ntc_valid is True
    assert "MPU_OK" in rx.status_labels
    assert "LORA_OK" in rx.status_labels


def test_invalid_sentinels_and_corruption_rejection() -> None:
    tx = TelemetryFrame(
        seq=1,
        tof_mm=TOF_INVALID_SENTINEL,
        ntc_temp_cdegc=TEMP_INVALID_CDEGC,
        ibat_ca=-150,
    )
    wire = bytearray(encode_frame(tx))
    rx = decode_frame(bytes(wire))
    assert not rx.tof_valid
    assert not rx.ntc_valid
    assert rx.ntc_temp_c is None
    assert rx.ibat_a == pytest.approx(-1.50)

    # Corrupt payload byte
    bad = bytearray(wire)
    bad[17] ^= 0x10
    with pytest.raises(FrameDecodeError, match="CRC-16"):
        decode_frame(bytes(bad))

    # Corrupt sync
    bad_sync = bytearray(wire)
    bad_sync[0] = 0xAA
    with pytest.raises(FrameDecodeError, match="sync"):
        decode_frame(bytes(bad_sync))

    # Wrong length
    with pytest.raises(FrameDecodeError, match="Expected 37 bytes"):
        decode_frame(bytes(wire[:20]))


def test_stream_decoder_resync_and_bytewise_feed() -> None:
    frames_in = [TelemetryFrame(seq=i, timestamp_ms=100 * i, vbat_mv=15000 - i) for i in (10, 11, 13)]
    raw_0 = encode_frame(frames_in[0])
    raw_1 = bytearray(encode_frame(frames_in[1]))
    raw_1[20] ^= 0xFF  # Corrupt seq=11
    raw_2 = encode_frame(frames_in[2])

    noisy_stream = b"\x00\xFF\xFC\x99\x12" + raw_0 + bytes(raw_1) + b"\xFC" + raw_2
    decoder = StreamDecoder()
    decoded = []
    # Feed 1 byte at a time to stress chunk boundaries & trailing 0xFC handling
    for b in noisy_stream:
        decoded.extend(decoder.feed_bytes(bytes([b])))

    assert [f.seq for f in decoded] == [10, 13]
    assert decoder.valid_frames == 2
    assert decoder.crc_errors == 1
    assert decoder.resync_count >= 2
    assert decoder.dropped_packets == 2  # seq 11 & 12 missed between 10 and 13


def test_ground_station_cli_modes(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    # 1. HEX mode
    frame = TelemetryFrame(seq=77, timestamp_ms=5000, vbat_mv=16200, rpm=6400, ntc_temp_cdegc=2500)
    hex_str = " ".join(f"{b:02X}" for b in encode_frame(frame))
    assert len(parse_hex_payload(f"FRAME_HEX: {hex_str}")) == 37
    rc = cli_main(["--hex", hex_str, "--format", "json"])
    assert rc == 0
    out = capsys.readouterr().out
    assert '"seq":77' in out
    assert '"vbat_v":16.2' in out

    # 2. Binary file + CSV mode
    bin_path = tmp_path / "capture.bin"
    bin_path.write_bytes(generate_simulated_stream(count=4, inject_corrupt_byte=True))
    log_path = tmp_path / "out.jsonl"
    rc = cli_main(["--file", str(bin_path), "--format", "csv", "--out-log", str(log_path)])
    assert rc == 0
    csv_out = capsys.readouterr().out.strip().splitlines()
    assert csv_out[0].startswith("seq,ts_ms,roll_deg")
    assert len(csv_out) == 4  # 1 header + 3 valid frames (seq=2 corrupted)
    assert len(log_path.read_text(encoding="utf-8").strip().splitlines()) == 3

    # 3. Simulate mode
    rc = cli_main(["--simulate", "--count", "3", "--format", "table"])
    assert rc == 0
    table_out = capsys.readouterr().out
    assert "[SEQ 00000" in table_out
    assert "[SEQ 00002" in table_out
