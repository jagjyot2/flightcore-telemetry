"""Structural & geometric validation for KiCad 8, Gerbers, BOM, BenchTwin, SVG & 3D STL."""

from collections import Counter
import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
HW_DIR = ROOT / "hardware"
GERBER_DIR = HW_DIR / "gerbers"
BT_DIR = ROOT / "benchtwin"


def _check_sexpr_balanced(text: str, filename: str) -> None:
    depth = 0
    in_quotes = False
    escape = False
    for ch in text:
        if escape:
            escape = False
            continue
        if ch == "\\" and in_quotes:
            escape = True
            continue
        if ch == '"':
            in_quotes = not in_quotes
            continue
        if in_quotes:
            continue
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            assert depth >= 0, f"Unmatched ')' in {filename}"
    assert not in_quotes, f"Unclosed quote in {filename}"
    assert depth == 0, f"Unbalanced S-expression in {filename}: remaining depth={depth}"


def test_kicad_project_files_valid() -> None:
    pro = json.loads((HW_DIR / "flightcore.kicad_pro").read_text(encoding="utf-8"))
    assert pro["meta"]["filename"] == "flightcore.kicad_pro"
    assert len(pro["net_settings"]["classes"]) >= 4

    for name in ("flightcore.kicad_sch", "flightcore.kicad_pcb", "flightcore.net"):
        txt = (HW_DIR / name).read_text(encoding="utf-8")
        _check_sexpr_balanced(txt, name)

    sch = (HW_DIR / "flightcore.kicad_sch").read_text(encoding="utf-8")
    assert "(kicad_sch" in sch and "(lib_symbols" in sch
    pcb = (HW_DIR / "flightcore.kicad_pcb").read_text(encoding="utf-8")
    assert "(kicad_pcb" in pcb and '(layer "Edge.Cuts")' in pcb and "(segment" in pcb


def test_gerbers_excellon_and_drc_zero_errors() -> None:
    required = [
        "flightcore-F_Cu.gtl",
        "flightcore-B_Cu.gbl",
        "flightcore-F_Mask.gts",
        "flightcore-B_Mask.gbs",
        "flightcore-F_Silkscreen.gto",
        "flightcore-B_Silkscreen.gbo",
        "flightcore-Edge_Cuts.gm1",
        "flightcore-PTH.drl",
        "drc_report.txt",
    ]
    for fname in required:
        p = GERBER_DIR / fname
        assert p.exists() and p.stat().st_size > 40, f"Missing/empty Gerber artifact: {fname}"

    for g in [f for f in required if not f.endswith((".drl", ".txt"))]:
        txt = (GERBER_DIR / g).read_text(encoding="utf-8")
        assert "%FSLAX46Y46*%" in txt
        assert "%MOMM*%" in txt
        assert "M02*" in txt

    drl = (GERBER_DIR / "flightcore-PTH.drl").read_text(encoding="utf-8")
    assert "M48" in drl and "METRIC" in drl and "M30" in drl

    drc = (GERBER_DIR / "drc_report.txt").read_text(encoding="utf-8")
    assert "Unconnected/Violations: 0 errors" in drc
    assert "[FAIL]" not in drc


def test_bom_and_netlist_consistency() -> None:
    hw_bom = (HW_DIR / "BOM.csv").read_text(encoding="utf-8")
    bt_bom = (BT_DIR / "BOM.csv").read_text(encoding="utf-8")
    assert hw_bom == bt_bom
    assert (HW_DIR / "flightcore.net").read_text(encoding="utf-8") == (
        BT_DIR / "flightcore.net"
    ).read_text(encoding="utf-8")

    with (HW_DIR / "BOM.csv").open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    refs = {r["Designator"]: r for r in rows}
    for required_ref in ("U1", "U2", "U3", "U4", "U5", "U6", "U7", "U8", "R7", "R8", "R9", "R11", "RT1"):
        assert required_ref in refs, f"Missing BOM designator {required_ref}"
    assert "ESP32" in refs["U1"]["Value/Spec"]
    assert "MPU6050" in refs["U2"]["Value/Spec"]
    assert "VL53L0X" in refs["U3"]["Value/Spec"]
    assert "SX1278" in refs["U4"]["Value/Spec"]
    assert "A3144" in refs["U8"]["Value/Spec"]


def test_benchtwin_bundle_and_svg_valid() -> None:
    graph = json.loads((BT_DIR / "flightcore-benchtwin.json").read_text(encoding="utf-8"))
    btw = json.loads((BT_DIR / "flightcore.btw").read_text(encoding="utf-8"))

    assert graph["schema"] == "benchtwin/circuit-v1"
    assert btw["format"] == "BenchTwin-Project-Archive"
    for net in ("I2C_SDA", "I2C_SCL", "VSPI_MOSI", "VSPI_MISO", "VSPI_SCK", "LORA_CS",
                "VBAT_ADC", "IBAT_ADC", "NTC_ADC", "HALL_SIG", "+3V3_BUS", "+5V_BUS", "GND"):
        assert net in graph["nets"], f"Net {net} missing from BenchTwin graph"

    svg_root = ET.fromstring((BT_DIR / "schematic_preview.svg").read_text(encoding="utf-8"))
    assert svg_root.tag.endswith("svg")


def test_enclosure_stl_is_closed_manifold() -> None:
    lines = (BT_DIR / "enclosure_mount.stl").read_text(encoding="utf-8").splitlines()
    assert lines[0].startswith("solid ")
    assert lines[-1].startswith("endsolid ")

    tris = []
    cur = []
    for ln in lines:
        s = ln.strip()
        if s.startswith("vertex "):
            cur.append(tuple(round(float(v), 4) for v in s.split()[1:4]))
        elif s.startswith("endloop"):
            assert len(cur) == 3
            tris.append(tuple(cur))
            cur = []

    assert len(tris) >= 500
    directed = Counter()
    undirected = Counter()
    vol = 0.0
    xs, ys, zs = [], [], []
    for a, b, c in tris:
        for p in (a, b, c):
            xs.append(p[0])
            ys.append(p[1])
            zs.append(p[2])
        for u, v in ((a, b), (b, c), (c, a)):
            directed[(u, v)] += 1
            undirected[tuple(sorted((u, v)))] += 1
        vol += (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        ) / 6.0

    assert all(count == 2 for count in undirected.values()), "Non-manifold edge found in STL"
    assert all(directed[(u, v)] == 1 and directed[(v, u)] == 1 for (u, v) in directed), "Inconsistent winding in STL"
    assert vol > 1000.0
    assert (max(xs) - min(xs)) == 102.0
    assert (max(ys) - min(ys)) == 74.0
    assert (max(zs) - min(zs)) == 8.0
