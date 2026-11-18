"""Canonical simulator-agnostic sensor & telemetry transport adapter for FlightCore.

Strictly telemetry-only (no motor/ESC PWM or flight-control actuation).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any

from ground_station.protocol import (
    FRAME_SIZE,
    STATUS_BITS,
    TEMP_INVALID_CDEGC,
    TOF_INVALID_SENTINEL,
    TelemetryFrame,
    decode_frame,
    encode_frame,
)

STATUS_MPU_OK = 1 << 0
STATUS_IMU_CALIBRATED = 1 << 1
STATUS_TOF_OK = 1 << 2
STATUS_TOF_RANGE_VALID = 1 << 3
STATUS_RPM_ACTIVE = 1 << 4
STATUS_POWER_NTC_OK = 1 << 5
STATUS_LORA_OK = 1 << 6
STATUS_CAM_LINK_OK = 1 << 7


@dataclass
class FaultInjectionConfig:
    """Deterministic fault-injection controls for simulation scenarios."""

    inject_crc_flip: bool = False
    crc_flip_byte_index: int = 15
    crc_flip_mask: int = 0x01
    inject_tof_timeout: bool = False
    inject_ntc_fault: bool = False
    vbat_sag_mv: int = 0
    forced_tx_retries: int = 0


@dataclass
class SensorSnapshot:
    """Canonical physical sensor snapshot independent of Wokwi or BenchTwin."""

    timestamp_ms: int
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    vib_rms_mg: int = 0
    tof_mm: int = TOF_INVALID_SENTINEL
    tof_valid: bool = False
    rpm: int = 0
    vbat_mv: int = 15600
    ibat_a: float = 0.0
    consumed_mah: int = 0
    ntc_temp_c: float = 25.0
    imu_temp_c: float = 26.0
    lora_rssi_dbm: int = -70
    tx_retries: int = 0
    cam_status: int = 1
    ntc_fault: bool = False

    def to_telemetry_frame(self, seq: int) -> TelemetryFrame:
        status = STATUS_MPU_OK | STATUS_IMU_CALIBRATED | STATUS_LORA_OK
        if not self.ntc_fault:
            status |= STATUS_POWER_NTC_OK
        if self.rpm > 0:
            status |= STATUS_RPM_ACTIVE
        if self.tof_mm != TOF_INVALID_SENTINEL:
            status |= STATUS_TOF_OK
        if self.tof_valid and self.tof_mm != TOF_INVALID_SENTINEL:
            status |= STATUS_TOF_RANGE_VALID

        ntc_cdegc = TEMP_INVALID_CDEGC if self.ntc_fault else int(round(self.ntc_temp_c * 100.0))
        return TelemetryFrame(
            seq=seq & 0xFFFF,
            timestamp_ms=self.timestamp_ms & 0xFFFFFFFF,
            roll_cdeg=int(round(self.roll_deg * 100.0)),
            pitch_cdeg=int(round(self.pitch_deg * 100.0)),
            vib_rms_mg=max(0, min(65535, int(self.vib_rms_mg))),
            tof_mm=max(0, min(65535, int(self.tof_mm))),
            rpm=max(0, min(65535, int(self.rpm))),
            vbat_mv=max(0, min(65535, int(self.vbat_mv))),
            ibat_ca=int(round(self.ibat_a * 100.0)),
            consumed_mah=max(0, min(65535, int(self.consumed_mah))),
            ntc_temp_cdegc=ntc_cdegc,
            imu_temp_cdegc=int(round(self.imu_temp_c * 100.0)),
            lora_rssi_dbm=max(-128, min(127, int(self.lora_rssi_dbm))),
            sensor_status=status,
            tx_retries=max(0, min(255, int(self.tx_retries))),
            cam_status=max(0, min(255, int(self.cam_status))),
        )


@dataclass
class TransportResult:
    wire_bytes: bytes
    crc_valid: bool
    corrupted: bool
    decoded_frame: TelemetryFrame | None = None
    error: str | None = None


class SensorBusAdapter(ABC):
    """Abstract base class for simulator backends (Wokwi, BenchTwin, Mock)."""

    @property
    @abstractmethod
    def backend_name(self) -> str:
        """Return canonical simulator backend name."""

    @abstractmethod
    def load_scenario(self, scenario_path: Path) -> dict[str, Any]:
        """Load a JSON scenario definition."""

    @abstractmethod
    def step(self, snapshot: SensorSnapshot, seq: int, fault: FaultInjectionConfig | None = None) -> TransportResult:
        """Apply optional fault injection, encode 37-byte frame, and verify wire transport."""


class CanonicalScenarioAdapter(SensorBusAdapter):
    """Simulator-independent reference implementation for scenario & fault verification."""

    def __init__(self, backend: str = "canonical") -> None:
        self._backend = backend
        self.frames_transmitted: int = 0
        self.frames_corrupted: int = 0

    @property
    def backend_name(self) -> str:
        return self._backend

    def load_scenario(self, scenario_path: Path) -> dict[str, Any]:
        return json.loads(scenario_path.read_text(encoding="utf-8"))

    @staticmethod
    def apply_faults(snapshot: SensorSnapshot, fault: FaultInjectionConfig | None) -> SensorSnapshot:
        if fault is None:
            return snapshot
        vbat = max(0, snapshot.vbat_mv - fault.vbat_sag_mv)
        tof_mm = TOF_INVALID_SENTINEL if fault.inject_tof_timeout else snapshot.tof_mm
        tof_valid = False if fault.inject_tof_timeout else snapshot.tof_valid
        ntc_fault = snapshot.ntc_fault or fault.inject_ntc_fault
        retries = max(snapshot.tx_retries, fault.forced_tx_retries)
        return SensorSnapshot(
            timestamp_ms=snapshot.timestamp_ms,
            roll_deg=snapshot.roll_deg,
            pitch_deg=snapshot.pitch_deg,
            vib_rms_mg=snapshot.vib_rms_mg,
            tof_mm=tof_mm,
            tof_valid=tof_valid,
            rpm=snapshot.rpm,
            vbat_mv=vbat,
            ibat_a=snapshot.ibat_a,
            consumed_mah=snapshot.consumed_mah,
            ntc_temp_c=snapshot.ntc_temp_c,
            imu_temp_c=snapshot.imu_temp_c,
            lora_rssi_dbm=snapshot.lora_rssi_dbm,
            tx_retries=retries,
            cam_status=snapshot.cam_status,
            ntc_fault=ntc_fault,
        )

    def step(self, snapshot: SensorSnapshot, seq: int, fault: FaultInjectionConfig | None = None) -> TransportResult:
        effective = self.apply_faults(snapshot, fault)
        frame = effective.to_telemetry_frame(seq=seq)
        raw = bytearray(encode_frame(frame))
        assert len(raw) == FRAME_SIZE

        corrupted = False
        if fault is not None and fault.inject_crc_flip:
            idx = fault.crc_flip_byte_index % FRAME_SIZE
            raw[idx] ^= (fault.crc_flip_mask & 0xFF) or 0x01
            corrupted = True
            self.frames_corrupted += 1

        self.frames_transmitted += 1
        wire = bytes(raw)
        try:
            decoded = decode_frame(wire)
            return TransportResult(wire_bytes=wire, crc_valid=True, corrupted=corrupted, decoded_frame=decoded)
        except ValueError as exc:
            return TransportResult(wire_bytes=wire, crc_valid=False, corrupted=corrupted, error=str(exc))

    def run_scenario_file(self, scenario_path: Path) -> list[TransportResult]:
        data = self.load_scenario(scenario_path)
        fault_cfg = data.get("fault_injection", {})
        fault = FaultInjectionConfig(
            inject_crc_flip=bool(fault_cfg.get("inject_crc_flip", False)),
            crc_flip_byte_index=int(fault_cfg.get("crc_flip_byte_index", 15)),
            crc_flip_mask=int(fault_cfg.get("crc_flip_mask", 1)),
            inject_tof_timeout=bool(fault_cfg.get("inject_tof_timeout", False)),
            inject_ntc_fault=bool(fault_cfg.get("inject_ntc_fault", False)),
            vbat_sag_mv=int(fault_cfg.get("vbat_sag_mv", 0)),
            forced_tx_retries=int(fault_cfg.get("forced_tx_retries", 0)),
        )
        results: list[TransportResult] = []
        for idx, sample in enumerate(data.get("samples", [])):
            snap = SensorSnapshot(
                timestamp_ms=int(sample["timestamp_ms"]),
                roll_deg=float(sample.get("roll_deg", 0.0)),
                pitch_deg=float(sample.get("pitch_deg", 0.0)),
                vib_rms_mg=int(sample.get("vib_rms_mg", 0)),
                tof_mm=int(sample.get("tof_mm", TOF_INVALID_SENTINEL)),
                tof_valid=bool(sample.get("tof_valid", False)),
                rpm=int(sample.get("rpm", 0)),
                vbat_mv=int(sample.get("vbat_mv", 15600)),
                ibat_a=float(sample.get("ibat_a", 0.0)),
                consumed_mah=int(sample.get("consumed_mah", 0)),
                ntc_temp_c=float(sample.get("ntc_temp_c", 25.0)),
                imu_temp_c=float(sample.get("imu_temp_c", 26.0)),
                lora_rssi_dbm=int(sample.get("lora_rssi_dbm", -70)),
                tx_retries=int(sample.get("tx_retries", 0)),
                cam_status=int(sample.get("cam_status", 1)),
                ntc_fault=bool(sample.get("ntc_fault", False)),
            )
            results.append(self.step(snap, seq=idx, fault=fault))
        return results
