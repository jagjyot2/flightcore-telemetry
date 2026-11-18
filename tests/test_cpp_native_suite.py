"""Compiles and executes the C++ native firmware unit test suite against firmware/src/*.cpp."""

from pathlib import Path
import shutil
import subprocess
import tempfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
FW_DIR = ROOT / "firmware"

CPP_SOURCES = [
    FW_DIR / "test" / "test_flightcore_native.cpp",
    FW_DIR / "src" / "telemetry_protocol.cpp",
    FW_DIR / "src" / "mpu6050_driver.cpp",
    FW_DIR / "src" / "vl53l0x_driver.cpp",
    FW_DIR / "src" / "hall_rpm_driver.cpp",
    FW_DIR / "src" / "power_temp_monitor.cpp",
    FW_DIR / "src" / "lora_sx1278.cpp",
]

VCVARS_CANDIDATES = [
    Path(r"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"),
    Path(r"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"),
]


def test_compile_and_run_cpp_native_unit_tests() -> None:
    vcvars = next((p for p in VCVARS_CANDIDATES if p.exists()), None)
    gxx = shutil.which("g++") or shutil.which("clang++")

    if vcvars is None and gxx is None:
        pytest.skip("Neither MSVC vcvars64.bat nor g++/clang++ found on host")

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        exe_path = tmp / "flightcore_native_test.exe"

        if vcvars is not None:
            srcs = " ".join(f'"{s}"' for s in CPP_SOURCES)
            bat = tmp / "build_and_test.bat"
            bat.write_text(
                "\n".join([
                    "@echo off",
                    f'call "{vcvars}" >nul',
                    f'cl /nologo /EHsc /std:c++17 /W4 /DFLIGHTCORE_NATIVE=1 /I"{FW_DIR / "include"}" /Fo"{tmp}/" /Fe:"{exe_path}" {srcs}',
                    "if errorlevel 1 exit /b %errorlevel%",
                    f'"{exe_path}"',
                ]),
                encoding="utf-8",
            )
            proc = subprocess.run(
                ["cmd", "/c", str(bat)],
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
        else:
            cmd = [
                gxx,
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-DFLIGHTCORE_NATIVE=1",
                f"-I{FW_DIR / 'include'}",
                *[str(s) for s in CPP_SOURCES],
                "-o",
                str(exe_path),
                "-lm",
            ]
            comp = subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=False)
            assert comp.returncode == 0, f"C++ compile failed:\n{comp.stderr}"
            proc = subprocess.run([str(exe_path)], capture_output=True, text=True, timeout=30, check=False)

        assert proc.returncode == 0, f"C++ native tests failed:\n{proc.stdout}\n{proc.stderr}"
        assert "7/7 C++ tests PASSED" in proc.stdout
