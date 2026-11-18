"""
FlightCore Ground Station Receiver & Telemetry Decoder CLI.

Supports:
  - Live USB serial ingestion (--serial COM3 --baud 115200)
  - Direct HEX capture decoding (--hex "FC 01 ...")
  - Binary log playback (--file telemetry.bin)
  - End-to-end synthetic stream verification (--simulate)

Safety Notice: Telemetry monitoring ONLY. Never issues flight/motor commands.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Iterable, Optional, TextIO

from .protocol import (
    FRAME_SIZE,
    StreamDecoder,
    TelemetryFrame,
    decode_frame,
    encode_frame,
)


def parse_hex_payload(hex_str: str) -> bytes:
    cleaned = hex_str.replace("FRAME_HEX:", "").strip()
    tokens = cleaned.replace(",", " ").replace(":", " ").split()
    if len(tokens) == 1 and len(tokens[0]) >= 2 and len(tokens[0]) % 2 == 0:
        return bytes.fromhex(tokens[0])
    return bytes(int(tok, 16) for tok in tokens)


def generate_simulated_stream(
    count: int = 10,
    inject_corrupt_byte: bool = False,
) -> bytes:
    """Generates a deterministic sequence of valid 37-byte frames with realistic drone telemetry."""
    chunks = bytearray()
    mah_acc = 120.0
    for seq in range(count):
        t_s = seq * 0.2
        roll_cdeg = int(round(450 * math.sin(t_s)))
        pitch_cdeg = int(round(-280 * math.cos(t_s * 0.7)))
        vib_mg = 85 + int(25 * abs(math.sin(t_s * 2.0)))
        tof_mm = 720 + int(110 * math.sin(t_s * 0.5))
        rpm = 7800 + int(450 * math.sin(t_s))
        vbat_mv = 15600 - seq * 8
        ibat_ca = 1650 + int(120 * math.cos(t_s))
        mah_acc += (ibat_ca / 100.0) * (0.2 / 3600.0) * 1000.0
        ntc_cdegc = 3420 + seq * 5
        imu_cdegc = 3280 + seq * 3

        frame = TelemetryFrame(
            seq=seq,
            timestamp_ms=1000 + seq * 200,
            roll_cdeg=roll_cdeg,
            pitch_cdeg=pitch_cdeg,
            vib_rms_mg=vib_mg,
            tof_mm=tof_mm,
            rpm=rpm,
            vbat_mv=vbat_mv,
            ibat_ca=ibat_ca,
            consumed_mah=int(round(mah_acc)),
            ntc_temp_cdegc=ntc_cdegc,
            imu_temp_cdegc=imu_cdegc,
            lora_rssi_dbm=-68 - (seq % 7),
            sensor_status=0x7F,
            tx_retries=1 if (seq % 5 == 4) else 0,
            cam_status=1,
        )
        wire = bytearray(encode_frame(frame))
        if inject_corrupt_byte and seq == 2:
            # Corrupt one frame and prepend leading noise to verify resync
            chunks.extend(b"\xAA\x55\xFC\x99")
            wire[12] ^= 0xFF
            chunks.extend(wire)
            continue
        chunks.extend(wire)
    return bytes(chunks)


def format_frame_line(frame: TelemetryFrame, fmt: str, header_written: bool) -> str:
    if fmt == "json":
        return frame.to_json()
    if fmt == "csv":
        lines = []
        if not header_written:
            lines.append(
                "seq,ts_ms,roll_deg,pitch_deg,vib_mg,tof_mm,rpm,vbat_v,ibat_a,"
                "mah,ntc_c,imu_c,rssi_dbm,status,retries"
            )
        ntc_s = "" if frame.ntc_temp_c is None else f"{frame.ntc_temp_c:.2f}"
        lines.append(
            f"{frame.seq},{frame.timestamp_ms},{frame.roll_deg:.2f},"
            f"{frame.pitch_deg:.2f},{frame.vib_rms_mg},{frame.tof_mm},"
            f"{frame.rpm},{frame.vbat_v:.3f},{frame.ibat_a:.2f},"
            f"{frame.consumed_mah},{ntc_s},{frame.imu_temp_c:.2f},"
            f"{frame.lora_rssi_dbm},0x{frame.sensor_status:02X},{frame.tx_retries}"
        )
        return "\n".join(lines)

    # Human-readable table line
    ntc_str = "FAULT" if frame.ntc_temp_c is None else f"{frame.ntc_temp_c:5.1f}C"
    tof_str = "INVALID" if not frame.tof_valid else f"{frame.tof_mm:4d}mm"
    return (
        f"[SEQ {frame.seq:05d} | {frame.timestamp_ms:8d}ms] "
        f"R:{frame.roll_deg:+6.2f}° P:{frame.pitch_deg:+6.2f}° Vib:{frame.vib_rms_mg:4d}mg | "
        f"ToF:{tof_str} RPM:{frame.rpm:5d} | "
        f"Bat:{frame.vbat_v:5.2f}V {frame.ibat_a:5.2f}A {frame.consumed_mah:5d}mAh | "
        f"NTC:{ntc_str} RSSI:{frame.lora_rssi_dbm:4d}dBm Retry:{frame.tx_retries}"
    )


def emit_frames(
    frames: Iterable[TelemetryFrame],
    fmt: str,
    out_stream: TextIO,
    log_handle: Optional[TextIO] = None,
) -> int:
    count = 0
    header_written = False
    for frame in frames:
        line = format_frame_line(frame, fmt, header_written)
        header_written = True
        print(line, file=out_stream)
        if log_handle is not None:
            log_handle.write(frame.to_json() + "\n")
        count += 1
    return count


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="flightcore-gs",
        description="FlightCore Ground Station Binary Telemetry Decoder (Strictly Telemetry Only)",
    )
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--serial", help="Serial port device (e.g. COM3 or /dev/ttyUSB0)")
    src.add_argument("--hex", help="Decode hex-encoded 37-byte frame(s)")
    src.add_argument("--file", type=Path, help="Read raw binary telemetry capture file")
    src.add_argument("--simulate", action="store_true", help="Run synthetic stream loopback check")

    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default 115200)")
    parser.add_argument("--count", type=int, default=10, help="Simulated frame count (default 10)")
    parser.add_argument("--interval", type=float, default=0.0, help="Delay between simulated frames (s)")
    parser.add_argument("--inject-noise", action="store_true", help="Inject corrupt frame in simulation")
    parser.add_argument(
        "--format",
        choices=("table", "json", "csv"),
        default="table",
        help="Output display format",
    )
    parser.add_argument("--out-log", type=Path, help="Optional JSONL output log path")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    decoder = StreamDecoder()
    log_handle: Optional[TextIO] = None

    try:
        if args.out_log is not None:
            args.out_log.parent.mkdir(parents=True, exist_ok=True)
            log_handle = args.out_log.open("a", encoding="utf-8")

        if args.hex:
            raw = parse_hex_payload(args.hex)
            if len(raw) == FRAME_SIZE:
                frames = [decode_frame(raw)]
                decoder.valid_frames = 1
            else:
                frames = decoder.feed_bytes(raw)
            emit_frames(frames, args.format, sys.stdout, log_handle)
        elif args.file:
            raw = args.file.read_bytes()
            frames = decoder.feed_bytes(raw)
            emit_frames(frames, args.format, sys.stdout, log_handle)
        elif args.simulate:
            stream = generate_simulated_stream(
                count=args.count,
                inject_corrupt_byte=args.inject_noise,
            )
            header_written = False
            for offset in range(0, len(stream), FRAME_SIZE):
                chunk = stream[offset : offset + FRAME_SIZE]
                for frame in decoder.feed_bytes(chunk):
                    line = format_frame_line(frame, args.format, header_written)
                    header_written = True
                    print(line)
                    if log_handle is not None:
                        log_handle.write(frame.to_json() + "\n")
                if args.interval > 0:
                    time.sleep(args.interval)
        elif args.serial:
            try:
                import serial  # type: ignore
            except ImportError:
                print("ERROR: pyserial is required for --serial mode (pip install pyserial)", file=sys.stderr)
                return 2

            print(f"Listening on {args.serial} @ {args.baud} baud (Ctrl+C to exit)...", file=sys.stderr)
            header_written = False
            with serial.Serial(args.serial, args.baud, timeout=0.2) as ser:
                try:
                    while True:
                        chunk = ser.read(128)
                        if not chunk:
                            continue
                        for frame in decoder.feed_bytes(chunk):
                            line = format_frame_line(frame, args.format, header_written)
                            header_written = True
                            print(line, flush=True)
                            if log_handle is not None:
                                log_handle.write(frame.to_json() + "\n")
                except KeyboardInterrupt:
                    pass

        summary = {
            "valid_frames": decoder.valid_frames,
            "crc_errors": decoder.crc_errors,
            "resync_events": decoder.resync_count,
            "dropped_packets": decoder.dropped_packets,
        }
        print(f"# Decoder Summary: {json.dumps(summary)}", file=sys.stderr)
        return 0
    finally:
        if log_handle is not None:
            log_handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
