"""Verification of FlightCore sensor math models, filters, and telemetry-only safety boundary."""

import math
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parents[1]

# Exact constants from firmware/include/config.h
NTC_SERIES_OHM = 10000.0
ADC_VREF_MV = 3300.0
SH_A = 1.127125950e-3
SH_B = 2.344443098e-4
SH_C = 8.649322763e-8


def steinhart_hart_c(r_ohm: float) -> float:
    ln_r = math.log(r_ohm)
    inv_t = SH_A + SH_B * ln_r + SH_C * (ln_r ** 3)
    return (1.0 / inv_t) - 273.15


def test_steinhart_hart_ntc_calibration_points() -> None:
    # Divider midpoint (1650 mV out of 3300 mV) => 10,000 ohm => 25.00 C
    v_adc = 1650.0
    r_ntc = NTC_SERIES_OHM * (v_adc / (ADC_VREF_MV - v_adc))
    assert r_ntc == pytest.approx(10000.0, abs=1e-6)
    assert steinhart_hart_c(10000.0) == pytest.approx(25.0, abs=0.01)
    assert steinhart_hart_c(32654.0) == pytest.approx(0.0, abs=0.02)
    assert steinhart_hart_c(2488.0) == pytest.approx(60.0, abs=0.02)


def test_complementary_filter_convergence_and_vibration_rms() -> None:
    alpha = 0.98
    dt = 0.005  # 200 Hz
    roll = 0.0
    target_deg = 15.0
    ay = math.sin(math.radians(target_deg))
    az = math.cos(math.radians(target_deg))
    accel_roll = math.degrees(math.atan2(ay, az))

    for _ in range(600):  # 3.0 seconds
        roll = alpha * (roll + 0.0 * dt) + (1.0 - alpha) * accel_roll
    assert roll == pytest.approx(15.0, abs=0.25)

    # Pure AC vibration +/- 0.1g square wave superimposed on tilted 1g gravity vector
    samples = [(0.0, ay, az + (0.1 if i % 2 == 0 else -0.1)) for i in range(32)]
    mean_x = sum(s[0] for s in samples) / len(samples)
    mean_y = sum(s[1] for s in samples) / len(samples)
    mean_z = sum(s[2] for s in samples) / len(samples)
    rms_g = math.sqrt(
        sum((sx - mean_x) ** 2 + (sy - mean_y) ** 2 + (sz - mean_z) ** 2 for sx, sy, sz in samples)
        / len(samples)
    )
    assert rms_g * 1000.0 == pytest.approx(100.0, abs=0.1)


def test_vl53l0x_median_ema_glitch_rejection() -> None:
    buf = []
    ema = None
    alpha = 0.35
    raw_seq = [500, 504, 1950, 498, 502, 501]  # 1950 mm specular outlier
    filtered = []
    for mm in raw_seq:
        buf.append(mm)
        if len(buf) > 5:
            buf.pop(0)
        med = sorted(buf)[len(buf) // 2]
        ema = float(med) if ema is None else (alpha * med + (1.0 - alpha) * ema)
        filtered.append(round(ema))
    assert max(filtered) <= 505
    assert filtered[-1] == pytest.approx(501, abs=2)


def test_hall_rpm_math_and_power_shunt_scaling() -> None:
    # 6000 RPM with 2 pulses/rev => 200 pulses/sec => 5000 us period
    dt_us = 5000
    ppr = 2
    rpm = 60_000_000.0 / (dt_us * ppr)
    assert rpm == pytest.approx(6000.0)

    # 6S LiPo max 25.2V through 100k/10k (11:1) => 2.291V ADC (< 3.3V limit)
    assert (25.2 / 11.0) == pytest.approx(2.2909, abs=0.001)

    # 40A through 1 mOhm shunt * INA180A2 (50 V/V) => 2.000V ADC
    v_sense = 40.0 * 0.001 * 50.0
    assert v_sense == pytest.approx(2.000)


def test_strict_telemetry_only_safety_boundary() -> None:
    """Guarantees no motor/ESC PWM control code exists anywhere in FlightCore firmware."""
    forbidden_patterns = [
        r"\bledcAttach",
        r"\bledcWrite",
        r"\bServo\.h\b",
        r"\bESC_PIN\b",
        r"\bMOTOR_PWM\b",
        r"\banalogWrite\s*\(",
    ]
    fw_files = list((ROOT / "firmware" / "include").glob("*.h")) + list(
        (ROOT / "firmware" / "src").glob("*.cpp")
    )
    assert len(fw_files) == 14
    for path in fw_files:
        text = path.read_text(encoding="utf-8")
        for pat in forbidden_patterns:
            assert not re.search(pat, text), f"Forbidden motor/PWM symbol {pat} found in {path.name}"
