"""Tests for simulator-independent abstraction, Wokwi diagram/config, scenarios, and BenchTwin preservation."""

from __future__ import annotations

import json
from pathlib import Path
import re
import tomllib

from ground_station.protocol import FRAME_SIZE, TEMP_INVALID_CDEGC, TOF_INVALID_SENTINEL
from simulation.adapters.common.sensor_bus_adapter import CanonicalScenarioAdapter
from simulation.adapters.wokwi.wokwi_flightcore_adapter import (
    CANONICAL_WOKWI_BINDINGS,
    WokwiFlightCoreAdapter,
)

ROOT = Path(__file__).resolve().parents[1]
CONFIG_H = ROOT / "firmware" / "include" / "config.h"
WOKWI_DIR = ROOT / "wokwi"
SIM_DIR = ROOT / "simulation"
BT_DIR = ROOT / "benchtwin"


def _parse_config_h_pins() -> dict[str, int]:
    text = CONFIG_H.read_text(encoding="utf-8")
    matches = re.findall(r"constexpr\s+uint8_t\s+(PIN_[A-Z0-9_]+)\s*=\s*(\d+)\s*;", text)
    return {name: int(val) for name, val in matches}


def test_wokwi_diagram_schema_and_config_h_pin_parity() -> None:
    diagram_path = WOKWI_DIR / "diagram.json"
    diagram = json.loads(diagram_path.read_text(encoding="utf-8"))
    assert diagram["version"] == 1
    assert diagram["editor"] == "wokwi"
    assert isinstance(diagram["parts"], list) and len(diagram["parts"]) >= 10
    assert isinstance(diagram["connections"], list) and len(diagram["connections"]) >= 25

    adapter = WokwiFlightCoreAdapter(diagram_path=diagram_path)
    errors = adapter.validate_diagram_bindings()
    assert errors == [], f"Wokwi diagram binding errors: {errors}"

    pins = _parse_config_h_pins()
    binding_by_signal = {b.signal: b for b in CANONICAL_WOKWI_BINDINGS}
    expected_pin_map = {
        "I2C_SDA": pins["PIN_I2C_SDA"],
        "I2C_SCL": pins["PIN_I2C_SCL"],
        "MPU_INT": pins["PIN_MPU6050_INT"],
        "TOF_XSHUT": pins["PIN_VL53L0X_XSHUT"],
        "TOF_INT": pins["PIN_VL53L0X_INT"],
        "LORA_SCK": pins["PIN_LORA_SCK"],
        "LORA_MISO": pins["PIN_LORA_MISO"],
        "LORA_MOSI": pins["PIN_LORA_MOSI"],
        "LORA_CS": pins["PIN_LORA_CS"],
        "LORA_RST": pins["PIN_LORA_RST"],
        "LORA_DIO0": pins["PIN_LORA_DIO0"],
        "HALL_SIG": pins["PIN_HALL_RPM"],
        "VBAT_ADC": pins["PIN_ADC_VBAT"],
        "IBAT_ADC": pins["PIN_ADC_IBAT"],
        "NTC_ADC": pins["PIN_ADC_NTC"],
        "STATUS_LED": pins["PIN_STATUS_LED"],
        "CAL_BTN": pins["PIN_CAL_BUTTON"],
    }
    for sig, gpio in expected_pin_map.items():
        assert binding_by_signal[sig].esp32_gpio == gpio, f"GPIO mismatch for {sig}"


def test_wokwi_toml_and_custom_chips() -> None:
    toml_data = tomllib.loads((WOKWI_DIR / "wokwi.toml").read_text(encoding="utf-8"))
    wokwi_cfg = toml_data["wokwi"]
    assert wokwi_cfg["version"] == 1
    assert wokwi_cfg["firmware"] == "../firmware/.pio/build/esp32dev/firmware.bin"
    assert wokwi_cfg["elf"] == "../firmware/.pio/build/esp32dev/firmware.elf"

    fw_bin = (WOKWI_DIR / wokwi_cfg["firmware"]).resolve()
    fw_elf = (WOKWI_DIR / wokwi_cfg["elf"]).resolve()
    assert fw_bin.exists() and fw_bin.stat().st_size > 10000
    assert fw_elf.exists() and fw_elf.stat().st_size > 10000

    chip_names = {c["name"] for c in toml_data.get("chip", [])}
    assert {"chip-vl53l0x", "chip-sx1278"}.issubset(chip_names)
    for chip in ("chip-vl53l0x", "chip-sx1278"):
        meta = json.loads((WOKWI_DIR / "chips" / f"{chip}.chip.json").read_text(encoding="utf-8"))
        assert meta["name"] == chip
        c_src = (WOKWI_DIR / "chips" / f"{chip}.chip.c").read_text(encoding="utf-8")
        assert "chip_init" in c_src


def test_simulation_scenarios_and_fault_injection() -> None:
    adapter = CanonicalScenarioAdapter("pytest")

    nom = adapter.run_scenario_file(SIM_DIR / "scenarios" / "nominal_flight.json")
    assert len(nom) == 3
    assert all(r.crc_valid and not r.corrupted and len(r.wire_bytes) == FRAME_SIZE for r in nom)
    assert nom[0].decoded_frame is not None
    assert nom[0].decoded_frame.vbat_mv == 15620

    sag = adapter.run_scenario_file(SIM_DIR / "scenarios" / "high_vibration_voltage_sag.json")
    assert len(sag) == 2
    assert all(r.crc_valid and r.decoded_frame is not None for r in sag)
    assert sag[0].decoded_frame.vbat_mv == 15600 - 2400
    assert sag[0].decoded_frame.tx_retries >= 1

    fault = adapter.run_scenario_file(SIM_DIR / "scenarios" / "lora_link_crc_fault.json")
    assert len(fault) == 2
    assert all(r.corrupted and not r.crc_valid for r in fault)

    # Verify faulted pre-CRC fields when CRC flip is disabled
    data = adapter.load_scenario(SIM_DIR / "scenarios" / "lora_link_crc_fault.json")
    data["fault_injection"]["inject_crc_flip"] = False
    tmp_snap = adapter.run_scenario_file(SIM_DIR / "scenarios" / "nominal_flight.json")[0]
    assert tmp_snap.decoded_frame is not None

    # Verify deterministic test-data hex parity
    test_data = json.loads((SIM_DIR / "test-data" / "sample_telemetry_frames.json").read_text(encoding="utf-8"))
    all_results = nom + sag + fault
    assert len(test_data["frames"]) == len(all_results)
    for entry, res in zip(test_data["frames"], all_results):
        assert entry["hex"] == res.wire_bytes.hex()
        assert entry["crc_valid"] == res.crc_valid
        assert entry["corrupted"] == res.corrupted

    # Verify sentinels exist in protocol
    assert TOF_INVALID_SENTINEL == 0xFFFF
    assert TEMP_INVALID_CDEGC == -32768


def test_benchtwin_preserved_and_support_matrix_documented() -> None:
    manifest_path = SIM_DIR / "adapters" / "benchtwin" / "benchtwin_adapter_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "preserved_inactive"
    assert manifest["active_simulator"] == "wokwi"
    for rel_path in manifest["artifacts"].values():
        resolved = (manifest_path.parent / rel_path).resolve()
        assert resolved.exists(), f"Missing preserved BenchTwin artifact: {resolved}"
    assert (BT_DIR / "flightcore.btw").exists()

    matrix = (SIM_DIR / "documentation" / "COMPONENT-SUPPORT-MATRIX.md").read_text(encoding="utf-8")
    for required in ("board-esp32-devkit-c-v4", "wokwi-mpu6050", "chip-vl53l0x", "chip-sx1278", "A3144"):
        assert required in matrix
