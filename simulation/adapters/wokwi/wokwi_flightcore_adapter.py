"""Wokwi simulator adapter mapping canonical FlightCore pins to Wokwi diagram.json."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from simulation.adapters.common.sensor_bus_adapter import CanonicalScenarioAdapter


@dataclass(frozen=True)
class WokwiPinBinding:
    signal: str
    esp32_gpio: int
    esp32_pin_label: str
    wokwi_part_id: str
    wokwi_part_type: str
    wokwi_part_pin: str
    support_mode: str  # "native", "analog_standin", "manual_pulse_stub", "custom_chip_stub"


CANONICAL_WOKWI_BINDINGS: tuple[WokwiPinBinding, ...] = (
    WokwiPinBinding("I2C_SDA", 21, "21", "imu_mpu6050", "wokwi-mpu6050", "SDA", "native"),
    WokwiPinBinding("I2C_SCL", 22, "22", "imu_mpu6050", "wokwi-mpu6050", "SCL", "native"),
    WokwiPinBinding("MPU_INT", 32, "32", "imu_mpu6050", "wokwi-mpu6050", "INT", "native"),
    WokwiPinBinding("NTC_ADC", 36, "VP", "ntc_probe", "wokwi-ntc-temperature-sensor", "OUT", "native"),
    WokwiPinBinding("VBAT_ADC", 34, "34", "pot_vbat", "wokwi-potentiometer", "SIG", "analog_standin"),
    WokwiPinBinding("IBAT_ADC", 35, "35", "pot_ibat", "wokwi-potentiometer", "SIG", "analog_standin"),
    WokwiPinBinding("HALL_SIG", 33, "33", "btn_hall", "wokwi-pushbutton", "1.l", "manual_pulse_stub"),
    WokwiPinBinding("CAL_BTN", 4, "4", "btn_cal", "wokwi-pushbutton", "1.l", "native"),
    WokwiPinBinding("STATUS_LED", 2, "2", "led_status", "wokwi-led", "A", "native"),
    WokwiPinBinding("TOF_SDA", 21, "21", "tof_vl53l0x", "chip-vl53l0x", "SDA", "custom_chip_stub"),
    WokwiPinBinding("TOF_SCL", 22, "22", "tof_vl53l0x", "chip-vl53l0x", "SCL", "custom_chip_stub"),
    WokwiPinBinding("TOF_XSHUT", 26, "26", "tof_vl53l0x", "chip-vl53l0x", "XSHUT", "custom_chip_stub"),
    WokwiPinBinding("TOF_INT", 25, "25", "tof_vl53l0x", "chip-vl53l0x", "INT", "custom_chip_stub"),
    WokwiPinBinding("LORA_SCK", 18, "18", "lora_sx1278", "chip-sx1278", "SCK", "custom_chip_stub"),
    WokwiPinBinding("LORA_MISO", 19, "19", "lora_sx1278", "chip-sx1278", "MISO", "custom_chip_stub"),
    WokwiPinBinding("LORA_MOSI", 23, "23", "lora_sx1278", "chip-sx1278", "MOSI", "custom_chip_stub"),
    WokwiPinBinding("LORA_CS", 5, "5", "lora_sx1278", "chip-sx1278", "NSS", "custom_chip_stub"),
    WokwiPinBinding("LORA_RST", 14, "14", "lora_sx1278", "chip-sx1278", "RST", "custom_chip_stub"),
    WokwiPinBinding("LORA_DIO0", 27, "27", "lora_sx1278", "chip-sx1278", "DIO0", "custom_chip_stub"),
)


class WokwiFlightCoreAdapter(CanonicalScenarioAdapter):
    """Wokwi-specific adapter extending the canonical FlightCore scenario bus."""

    def __init__(self, diagram_path: Path | None = None) -> None:
        super().__init__(backend="wokwi")
        self.diagram_path = diagram_path

    def validate_diagram_bindings(self, diagram_path: Path | None = None) -> list[str]:
        """Verify all canonical bindings exist in wokwi/diagram.json."""
        target = diagram_path or self.diagram_path
        if target is None:
            raise ValueError("diagram_path must be provided")
        diagram: dict[str, Any] = json.loads(target.read_text(encoding="utf-8"))
        parts_by_id = {p["id"]: p for p in diagram.get("parts", [])}
        connections = diagram.get("connections", [])
        edge_pairs = {(c[0], c[1]) for c in connections if len(c) >= 2} | {
            (c[1], c[0]) for c in connections if len(c) >= 2
        }

        errors: list[str] = []
        if "esp32" not in parts_by_id or parts_by_id["esp32"].get("type") != "board-esp32-devkit-c-v4":
            errors.append("Missing esp32 board-esp32-devkit-c-v4 part")

        for b in CANONICAL_WOKWI_BINDINGS:
            part = parts_by_id.get(b.wokwi_part_id)
            if part is None:
                errors.append(f"Missing Wokwi part '{b.wokwi_part_id}' for {b.signal}")
                continue
            if part.get("type") != b.wokwi_part_type:
                errors.append(
                    f"Part '{b.wokwi_part_id}' type mismatch: expected {b.wokwi_part_type}, got {part.get('type')}"
                )
            esp_ep = f"esp32:{b.esp32_pin_label}"
            part_ep = f"{b.wokwi_part_id}:{b.wokwi_part_pin}"
            if (esp_ep, part_ep) not in edge_pairs:
                errors.append(f"Missing Wokwi connection {esp_ep} <-> {part_ep} ({b.signal})")
        return errors
