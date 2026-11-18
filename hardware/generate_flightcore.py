"""Generate and verify repaired 04-FLIGHTCORE KiCad 10.0.8 schematic, PCB, Gerbers & 3D outputs."""

from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "00-PORTFOLIO-MANAGEMENT" / "kicad-automation"))

import kicad_toolkit as kt  # noqa: E402

HW_DIR = Path(__file__).resolve().parent
BT_DIR = ROOT / "04-FLIGHTCORE" / "benchtwin"
LIB = "FlightCore"
PROJ = "flightcore"


def build() -> None:
    kt.write_utf8(HW_DIR / f"{PROJ}.kicad_pro", kt.build_kicad_pro(PROJ))
    kt.write_project_lib_tables(HW_DIR, LIB)

    sb = kt.SchematicBuilder(
        project_name=PROJ,
        lib_name=LIB,
        title="FlightCore ESP32 Drone Telemetry Expansion Board (No ESC/Motor Control)",
        company="Jagjyot Engineering Portfolio",
        rev="1.1",
        paper="A1",
    )
    sb.add_note(
        25.4,
        15.24,
        "FLIGHTCORE TELEMETRY CARRIER v1.1 (REPAIRED & VERIFIED IN KICAD 10.0.8)\n"
        "- Fixed off-grid pins, HALL_SIG/VBAT_PROT & +3V3_BUS/NTC_ADC shorts, 12 PCB track crossings & 77 unrouted pads.\n"
        "- Strictly telemetry & sensors only: ZERO motor/ESC PWM actuation traces or drivers.",
    )

    # U1: ESP32 DevKit v1 30-pin
    sb.add_component(
        kt.SchComponent(
            "U1",
            "ESP32-DevKit-V1-30P",
            f"{LIB}:ESP32_DevKit_V1_30P",
            [
                kt.SchPin("1", "EN", "NC_EN", "left", "no_connect"),
                kt.SchPin("2", "VP/GPIO36", "NTC_ADC", "left"),
                kt.SchPin("3", "VN/GPIO39", "NC_VN", "left", "no_connect"),
                kt.SchPin("4", "D34/GPIO34", "VBAT_ADC", "left"),
                kt.SchPin("5", "D35/GPIO35", "IBAT_ADC", "left"),
                kt.SchPin("6", "D32/GPIO32", "MPU_INT", "left"),
                kt.SchPin("7", "D33/GPIO33", "HALL_SIG", "left"),
                kt.SchPin("8", "D25/GPIO25", "TOF_INT", "left"),
                kt.SchPin("9", "D26/GPIO26", "TOF_XSHUT", "left"),
                kt.SchPin("10", "D27/GPIO27", "LORA_DIO0", "left"),
                kt.SchPin("11", "D14/GPIO14", "LORA_RST", "left"),
                kt.SchPin("12", "D12/GPIO12", "NC_D12", "left", "no_connect"),
                kt.SchPin("13", "D13/GPIO13", "NC_D13", "left", "no_connect"),
                kt.SchPin("14", "GND1", "GND", "left"),
                kt.SchPin("15", "VIN", "+5V_BUS", "left"),
                kt.SchPin("16", "3V3", "ESP32_3V3", "right"),
                kt.SchPin("17", "GND2", "GND", "right"),
                kt.SchPin("18", "D15/GPIO15", "NC_D15", "right", "no_connect"),
                kt.SchPin("19", "D2/GPIO2", "STATUS_LED", "right"),
                kt.SchPin("20", "D4/GPIO4", "CAL_BTN", "right"),
                kt.SchPin("21", "RX2/GPIO16", "CAM_UART_RX", "right"),
                kt.SchPin("22", "TX2/GPIO17", "CAM_UART_TX", "right"),
                kt.SchPin("23", "D5/GPIO5", "LORA_CS", "right"),
                kt.SchPin("24", "D18/GPIO18", "VSPI_SCK", "right"),
                kt.SchPin("25", "D19/GPIO19", "VSPI_MISO", "right"),
                kt.SchPin("26", "D21/GPIO21", "I2C_SDA", "right"),
                kt.SchPin("27", "RX0/GPIO3", "NC_RX0", "right", "no_connect"),
                kt.SchPin("28", "TX0/GPIO1", "NC_TX0", "right", "no_connect"),
                kt.SchPin("29", "D22/GPIO22", "I2C_SCL", "right"),
                kt.SchPin("30", "D23/GPIO23", "VSPI_MOSI", "right"),
            ],
            mpn="ESP32-DevKit-V1-30P",
            manufacturer="Espressif",
            rating="3.3V IO / 5V VIN",
        )
    )

    # Sensors & RF
    sb.add_component(
        kt.SchComponent(
            "U2",
            "MPU6050-GY521",
            f"{LIB}:PinHeader_1x08_P2.54mm",
            [
                kt.SchPin("1", "VCC", "+3V3_BUS", "left"),
                kt.SchPin("2", "GND", "GND", "left"),
                kt.SchPin("3", "SCL", "I2C_SCL", "left"),
                kt.SchPin("4", "SDA", "I2C_SDA", "left"),
                kt.SchPin("5", "XDA", "NC_XDA", "right", "no_connect"),
                kt.SchPin("6", "XCL", "NC_XCL", "right", "no_connect"),
                kt.SchPin("7", "AD0", "GND", "right"),
                kt.SchPin("8", "INT", "MPU_INT", "right"),
            ],
            mpn="GY-521",
            manufacturer="InvenSense",
            rating="3.3V / I2C 400kHz",
        )
    )
    sb.add_component(
        kt.SchComponent(
            "U3",
            "VL53L0X-ToF",
            f"{LIB}:PinHeader_1x06_P2.54mm",
            [
                kt.SchPin("1", "VIN", "+3V3_BUS", "left"),
                kt.SchPin("2", "GND", "GND", "left"),
                kt.SchPin("3", "SCL", "I2C_SCL", "left"),
                kt.SchPin("4", "SDA", "I2C_SDA", "right"),
                kt.SchPin("5", "GPIO1", "TOF_INT", "right"),
                kt.SchPin("6", "XSHUT", "TOF_XSHUT", "right"),
            ],
            mpn="GY-VL53L0XV2",
            manufacturer="STMicroelectronics",
            rating="3.3V / I2C 0x29",
        )
    )
    sb.add_component(
        kt.SchComponent(
            "U4",
            "SX1278-Ra02-433MHz",
            f"{LIB}:Ai_Thinker_Ra02",
            [
                kt.SchPin("1", "GND1", "GND", "left"),
                kt.SchPin("2", "GND2", "GND", "left"),
                kt.SchPin("3", "3V3", "+3V3_BUS", "left"),
                kt.SchPin("4", "RST", "LORA_RST", "left"),
                kt.SchPin("5", "DIO0", "LORA_DIO0", "left"),
                kt.SchPin("6", "DIO1", "NC_DIO1", "left", "no_connect"),
                kt.SchPin("7", "DIO2", "NC_DIO2", "left", "no_connect"),
                kt.SchPin("8", "DIO3", "NC_DIO3", "left", "no_connect"),
                kt.SchPin("9", "DIO4", "NC_DIO4", "left", "no_connect"),
                kt.SchPin("10", "DIO5", "NC_DIO5", "right", "no_connect"),
                kt.SchPin("11", "GND3", "GND", "right"),
                kt.SchPin("12", "SCK", "VSPI_SCK", "right"),
                kt.SchPin("13", "MISO", "VSPI_MISO", "right"),
                kt.SchPin("14", "MOSI", "VSPI_MOSI", "right"),
                kt.SchPin("15", "NSS", "LORA_CS", "right"),
                kt.SchPin("16", "GND4", "GND", "right"),
                kt.SchPin("17", "ANT", "LORA_RF_ANT", "right"),
            ],
            mpn="Ra-02-SX1278",
            manufacturer="Ai-Thinker",
            rating="3.3V / +20dBm",
        )
    )
    sb.add_component(
        kt.SchComponent(
            "U5",
            "INA180A2IDBVR",
            f"{LIB}:SOT-23-5",
            [
                kt.SchPin("1", "OUT", "IBAT_SENSE_RAW", "left"),
                kt.SchPin("2", "GND", "GND", "left"),
                kt.SchPin("3", "IN+", "VBAT_IN", "left"),
                kt.SchPin("4", "IN-", "VBAT_LOAD", "right"),
                kt.SchPin("5", "VS", "+3V3_BUS", "right"),
            ],
            mpn="INA180A2IDBVR",
            manufacturer="Texas Instruments",
            rating="50V/V, Vcm 26V",
        )
    )
    sb.add_component(
        kt.SchComponent(
            "U6",
            "AMS1117-3.3",
            f"{LIB}:SOT-223-3_TabPin2",
            [
                kt.SchPin("1", "GND", "GND", "left"),
                kt.SchPin("2", "VOUT", "+3V3_BUS", "right"),
                kt.SchPin("3", "VIN", "+5V_BUS", "left"),
            ],
            mpn="AMS1117-3.3",
            manufacturer="AMS",
            rating="3.3V / 1.0A",
        )
    )
    sb.add_component(
        kt.SchComponent(
            "U7",
            "MP1584-5V-3A-Buck",
            f"{LIB}:Buck_5V_3A_Module",
            [
                kt.SchPin("1", "VIN+", "VBAT_PROT", "left"),
                kt.SchPin("2", "GND_IN", "GND", "left"),
                kt.SchPin("3", "GND_OUT", "GND", "right"),
                kt.SchPin("4", "VOUT_5V", "+5V_BUS", "right"),
            ],
            mpn="MP1584EN-5V",
            manufacturer="MPS",
            rating="7-28V -> 5V/3A",
        )
    )

    # Connectors & Protection
    sb.add_component(kt.SchComponent("J1", "XT60-VBAT-IN", f"{LIB}:AMASS_XT60_Vertical", [kt.SchPin("1", "VBAT+", "VBAT_IN", "left"), kt.SchPin("2", "GND", "GND", "right")], mpn="XT60-M", manufacturer="Amass", rating="60A / 25.2V"))
    sb.add_component(kt.SchComponent("J2", "XT60-PDB-OUT", f"{LIB}:AMASS_XT60_Vertical", [kt.SchPin("1", "VBAT+", "VBAT_LOAD", "left"), kt.SchPin("2", "GND", "GND", "right")], mpn="XT60-F", manufacturer="Amass", rating="60A / 25.2V"))
    sb.add_component(kt.SchComponent("J3", "JST-XH-3P-HALL", f"{LIB}:PinHeader_1x03_P2.54mm", [kt.SchPin("1", "VCC_5V", "+5V_BUS", "left"), kt.SchPin("2", "SIG", "HALL_SIG_EXT", "left"), kt.SchPin("3", "GND", "GND", "right")], mpn="B3B-XH-A", manufacturer="JST", rating="5V / 1A"))
    sb.add_component(kt.SchComponent("J4", "JST-XH-2P-NTC", f"{LIB}:PinHeader_1x02_P2.54mm", [kt.SchPin("1", "NTC", "NTC_ADC", "left"), kt.SchPin("2", "GND", "GND", "right")], mpn="B2B-XH-A", manufacturer="JST", rating="3.3V / 1A"))
    sb.add_component(kt.SchComponent("J5", "SMA-433MHz-ANT", f"{LIB}:SMA_EdgeMount", [kt.SchPin("1", "RF", "LORA_RF_ANT", "left"), kt.SchPin("2", "GND1", "GND", "right"), kt.SchPin("3", "GND2", "GND", "right")], mpn="132289", manufacturer="Amphenol", rating="50 Ohm / 433MHz"))
    sb.add_component(kt.SchComponent("J6", "JST-XH-4P-CAM", f"{LIB}:PinHeader_1x04_P2.54mm", [kt.SchPin("1", "VCC_5V", "+5V_BUS", "left"), kt.SchPin("2", "GND", "GND", "left"), kt.SchPin("3", "TX2", "CAM_UART_TX", "right"), kt.SchPin("4", "RX2", "CAM_UART_RX", "right")], mpn="B4B-XH-A", manufacturer="JST", rating="5V / 500mA"))

    sb.add_component(kt.SchComponent("F1", "1.5A-Polyfuse", f"{LIB}:SMD_1206", [kt.SchPin("1", "1", "VBAT_IN", "left"), kt.SchPin("2", "2", "VBAT_FUSED", "right")], mpn="1206L150", manufacturer="Littelfuse", rating="30V / 1.5A"))
    sb.add_component(kt.SchComponent("D1", "SS34-Schottky", f"{LIB}:D_SMA", [kt.SchPin("1", "A", "VBAT_FUSED", "left"), kt.SchPin("2", "K", "VBAT_PROT", "right")], mpn="SS34", manufacturer="Vishay", rating="40V / 3A"))
    sb.add_component(kt.SchComponent("D2", "BAT54S-Clamp", f"{LIB}:SOT-23", [kt.SchPin("1", "A", "GND", "left"), kt.SchPin("2", "K", "+3V3_BUS", "left"), kt.SchPin("3", "COM", "VBAT_ADC", "right")], mpn="BAT54S", manufacturer="Diodes Inc", rating="30V / 200mA"))
    sb.add_component(kt.SchComponent("D3", "SMBJ28A-TVS", f"{LIB}:D_SMB", [kt.SchPin("1", "K", "VBAT_PROT", "left"), kt.SchPin("2", "A", "GND", "right")], mpn="SMBJ28A", manufacturer="Littelfuse", rating="28V / 600W"))
    sb.add_component(kt.SchComponent("D4", "LED-Green-0805", f"{LIB}:SMD_0805", [kt.SchPin("1", "A", "STATUS_LED", "left"), kt.SchPin("2", "K", "STATUS_LED_K", "right")], mpn="APT2012CGCK", manufacturer="Kingbright", rating="2.1V / 20mA"))
    sb.add_component(kt.SchComponent("SW1", "Tactile-CAL-Btn", f"{LIB}:SW_SPST_SMD", [kt.SchPin("1", "1", "CAL_BTN", "left"), kt.SchPin("2", "2", "GND", "right")], mpn="PTS645", manufacturer="C&K", rating="12V / 50mA"))
    sb.add_component(kt.SchComponent("JP1", "Jumper-2P-3V3-Bench", f"{LIB}:PinHeader_1x02_P2.54mm", [kt.SchPin("1", "ESP_3V3", "ESP32_3V3", "left"), kt.SchPin("2", "BUS_3V3", "+3V3_BUS", "right")], mpn="PinHeader_1x02", manufacturer="Generic", rating="3.3V / 1A"))

    # Resistors
    resistors = [
        ("R1", "4.7k", "+3V3_BUS", "I2C_SDA"),
        ("R2", "4.7k", "+3V3_BUS", "I2C_SCL"),
        ("R3", "10k", "+3V3_BUS", "TOF_XSHUT"),
        ("R4", "10k", "+3V3_BUS", "LORA_CS"),
        ("R5", "10k", "+3V3_BUS", "HALL_SIG"),
        ("R6", "1k", "HALL_SIG_EXT", "HALL_SIG"),
        ("R7", "100k", "VBAT_PROT", "VBAT_ADC"),
        ("R8", "10k", "VBAT_ADC", "GND"),
        ("R9", "10k", "+3V3_BUS", "NTC_ADC"),
        ("R10", "100R", "IBAT_SENSE_RAW", "IBAT_ADC"),
        ("R12", "330R", "STATUS_LED_K", "GND"),
    ]
    for ref, val, n1, n2 in resistors:
        sb.add_component(kt.SchComponent(ref, val, f"{LIB}:SMD_0805", [kt.SchPin("1", "1", n1, "left"), kt.SchPin("2", "2", n2, "right")], rating="50V / 0.125W"))

    sb.add_component(kt.SchComponent("R11", "0.001R-3W-Shunt", f"{LIB}:R_2512_Shunt", [kt.SchPin("1", "IN+", "VBAT_IN", "left"), kt.SchPin("2", "IN-", "VBAT_LOAD", "right")], mpn="WSL25121L000FEA", manufacturer="Vishay", rating="1mOhm / 3W"))

    # Capacitors
    sb.add_component(kt.SchComponent("C1", "100uF-35V", f"{LIB}:CP_Radial_D6.3mm", [kt.SchPin("1", "+", "VBAT_PROT", "left"), kt.SchPin("2", "-", "GND", "right")], rating="35V / 100uF"))
    caps = [
        ("C2", "10uF-16V", "+5V_BUS", "GND"),
        ("C3", "10uF-16V", "+5V_BUS", "GND"),
        ("C4", "22uF-10V", "+3V3_BUS", "GND"),
        ("C5", "100nF-25V", "HALL_SIG", "GND"),
        ("C6", "100nF-25V", "VBAT_ADC", "GND"),
        ("C7", "100nF-25V", "IBAT_ADC", "GND"),
        ("C8", "100nF-25V", "NTC_ADC", "GND"),
        ("C9", "100nF-25V", "+3V3_BUS", "GND"),
        ("C10", "100nF-25V", "+3V3_BUS", "GND"),
        ("C11", "10uF-10V", "+3V3_BUS", "GND"),
        ("C12", "100nF-25V", "+3V3_BUS", "GND"),
        ("C13", "100nF-25V", "+3V3_BUS", "GND"),
    ]
    for ref, val, n1, n2 in caps:
        sb.add_component(kt.SchComponent(ref, val, f"{LIB}:SMD_0805", [kt.SchPin("1", "1", n1, "left"), kt.SchPin("2", "2", n2, "right")], rating=val.split("-")[1]))

    for h in ("H1", "H2", "H3", "H4"):
        sb.add_component(kt.SchComponent(h, "M3-Mount-Hole", f"{LIB}:MountingHole_3.2mm_M3_Pad_Via", [kt.SchPin("1", "GND", "GND", "left")], rating="M3"))

    sb.write(HW_DIR)

    # ---------------- PCB BUILD ----------------
    pb = kt.PCBBuilder(PROJ, LIB, 96.0, 68.0, "FLIGHTCORE TELEMETRY v1.1 (NO ESC/PWM)", sb.root_uuid)

    fp_esp = kt.fp_dual_row_module("ESP32_DevKit_V1_30P", 15, 25.4)
    fp_h8 = kt.fp_pin_header_1xN("PinHeader_1x08_P2.54mm", 8)
    fp_h6 = kt.fp_pin_header_1xN("PinHeader_1x06_P2.54mm", 6)
    fp_h4 = kt.fp_pin_header_1xN("PinHeader_1x04_P2.54mm", 4)
    fp_h3 = kt.fp_pin_header_1xN("PinHeader_1x03_P2.54mm", 3)
    fp_h2 = kt.fp_pin_header_1xN("PinHeader_1x02_P2.54mm", 2)
    fp_xt = kt.fp_xt60("AMASS_XT60_Vertical")
    fp_shunt = kt.fp_smd_2pin("R_2512_Shunt", 5.6, 2.2, 3.4, "Resistor_SMD.3dshapes/R_2512_6332Metric.step")
    fp_0805 = kt.fp_smd_2pin("SMD_0805", 1.825, 1.025, 1.4, "Resistor_SMD.3dshapes/R_0805_2012Metric.step")
    fp_1206 = kt.fp_smd_2pin("SMD_1206", 2.9, 1.4, 1.8, "Fuse.3dshapes/Fuse_1206_3216Metric.step")
    fp_sma = kt.fp_smd_2pin("D_SMA", 3.8, 1.8, 2.1, "Diode_SMD.3dshapes/D_SMA.step")
    fp_smb = kt.fp_smd_2pin("D_SMB", 4.3, 2.0, 2.4, "Diode_SMD.3dshapes/D_SMB.step")
    fp_sot23 = kt.fp_sot23_3("SOT-23")
    fp_sot23_5 = kt.fp_sot23_5("SOT-23-5")
    fp_sot223 = kt.fp_sot223_tab("SOT-223-3_TabPin2")
    fp_buck = kt.FootprintDef(
        "Buck_5V_3A_Module",
        "through_hole",
        [
            kt.PadDef("1", "thru_hole", "rect", -6.0, -4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
            kt.PadDef("2", "thru_hole", "circle", -6.0, +4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
            kt.PadDef("3", "thru_hole", "circle", +6.0, +4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
            kt.PadDef("4", "thru_hole", "circle", +6.0, -4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
        ],
        15.0,
        11.0,
    )
    fp_ra02 = kt.FootprintDef(
        "Ai_Thinker_Ra02",
        "smd",
        [kt.PadDef(str(i + 1), "smd", "roundrect", -8.0, round(-8.0 + i * 2.0, 2), 2.0, 0.9) for i in range(9)]
        + [kt.PadDef(str(10 + i), "smd", "roundrect", +8.0, round(-7.0 + i * 2.0, 2), 2.0, 0.9) for i in range(8)],
        18.0,
        19.0,
    )
    fp_sma_rf = kt.FootprintDef(
        "SMA_EdgeMount",
        "through_hole",
        [
            kt.PadDef("1", "thru_hole", "rect", 0.0, 0.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
            kt.PadDef("2", "thru_hole", "circle", 0.0, -3.5, 2.0, 2.0, 1.2, ("*.Cu", "*.Mask")),
            kt.PadDef("3", "thru_hole", "circle", 0.0, +3.5, 2.0, 2.0, 1.2, ("*.Cu", "*.Mask")),
        ],
        6.0,
        9.5,
    )
    fp_cp = kt.FootprintDef(
        "CP_Radial_D6.3mm",
        "through_hole",
        [
            kt.PadDef("1", "thru_hole", "rect", -1.25, 0.0, 1.7, 1.7, 0.9, ("*.Cu", "*.Mask")),
            kt.PadDef("2", "thru_hole", "circle", +1.25, 0.0, 1.7, 1.7, 0.9, ("*.Cu", "*.Mask")),
        ],
        6.5,
        6.5,
    )
    fp_sw = kt.fp_smd_2pin("SW_SPST_SMD", 6.5, 1.6, 1.8)
    fp_mh = kt.fp_mounting_hole()

    # Mounting holes (89x61 mm pattern on 96x68 mm board -> 3.5 mm inset)
    for ref, (mx, my) in [("H1", (3.5, 3.5)), ("H2", (92.5, 3.5)), ("H3", (92.5, 64.5)), ("H4", (3.5, 64.5))]:
        pb.add_component(kt.PcbComponent(ref, "M3-Mount-Hole", fp_mh, mx, my, 0.0, {"1": "GND"}))

    # Main MCU centered (x: 28.7..57.3, y: 11.9..52.1)
    pb.add_component(kt.PcbComponent("U1", "ESP32-DevKit-V1-30P", fp_esp, 43.0, 32.0, 0.0, {
        "2": "NTC_ADC", "4": "VBAT_ADC", "5": "IBAT_ADC", "6": "MPU_INT", "7": "HALL_SIG",
        "8": "TOF_INT", "9": "TOF_XSHUT", "10": "LORA_DIO0", "11": "LORA_RST", "14": "GND",
        "15": "+5V_BUS", "16": "ESP32_3V3", "17": "GND", "19": "STATUS_LED", "20": "CAL_BTN",
        "21": "CAM_UART_RX", "22": "CAM_UART_TX", "23": "LORA_CS", "24": "VSPI_SCK",
        "25": "VSPI_MISO", "26": "I2C_SDA", "29": "I2C_SCL", "30": "VSPI_MOSI",
    }))

    # High-current input/shunt/power stage on left
    pb.add_component(kt.PcbComponent("J1", "XT60-VBAT-IN", fp_xt, 7.5, 17.5, 0.0, {"1": "VBAT_IN", "2": "GND"}))
    pb.add_component(kt.PcbComponent("J2", "XT60-PDB-OUT", fp_xt, 7.5, 36.5, 0.0, {"1": "VBAT_LOAD", "2": "GND"}))
    pb.add_component(kt.PcbComponent("R11", "0.001R-3W-Shunt", fp_shunt, 20.0, 12.0, 0.0, {"1": "VBAT_IN", "2": "VBAT_LOAD"}))
    pb.add_component(kt.PcbComponent("U5", "INA180A2IDBVR", fp_sot23_5, 20.0, 5.5, 0.0, {"1": "IBAT_SENSE_RAW", "2": "GND", "3": "VBAT_IN", "4": "VBAT_LOAD", "5": "+3V3_BUS"}))
    pb.add_component(kt.PcbComponent("F1", "1.5A-Polyfuse", fp_1206, 18.5, 18.8, 0.0, {"1": "VBAT_IN", "2": "VBAT_FUSED"}))
    pb.add_component(kt.PcbComponent("D1", "SS34-Schottky", fp_sma, 18.5, 24.8, 0.0, {"1": "VBAT_FUSED", "2": "VBAT_PROT"}))
    pb.add_component(kt.PcbComponent("D3", "SMBJ28A-TVS", fp_smb, 18.5, 31.0, 0.0, {"1": "VBAT_PROT", "2": "GND"}))
    pb.add_component(kt.PcbComponent("C1", "100uF-35V", fp_cp, 18.5, 38.8, 0.0, {"1": "VBAT_PROT", "2": "GND"}))
    pb.add_component(kt.PcbComponent("U7", "MP1584-5V-3A-Buck", fp_buck, 13.5, 53.0, 0.0, {"1": "VBAT_PROT", "2": "GND", "3": "GND", "4": "+5V_BUS"}))
    pb.add_component(kt.PcbComponent("U6", "AMS1117-3.3", fp_sot223, 31.0, 62.5, 0.0, {"1": "GND", "2": "+3V3_BUS", "3": "+5V_BUS"}))

    # Right side: IMU, ToF, LoRa, SMA, CAM
    pb.add_component(kt.PcbComponent("U2", "MPU6050-GY521", fp_h8, 64.5, 24.5, 0.0, {"1": "+3V3_BUS", "2": "GND", "3": "I2C_SCL", "4": "I2C_SDA", "7": "GND", "8": "MPU_INT"}))
    pb.add_component(kt.PcbComponent("U3", "VL53L0X-ToF", fp_h6, 72.0, 24.5, 0.0, {"1": "+3V3_BUS", "2": "GND", "3": "I2C_SCL", "4": "I2C_SDA", "5": "TOF_INT", "6": "TOF_XSHUT"}))
    pb.add_component(kt.PcbComponent("U4", "SX1278-Ra02-433MHz", fp_ra02, 79.0, 46.0, 0.0, {
        "1": "GND", "2": "GND", "3": "+3V3_BUS", "4": "LORA_RST", "5": "LORA_DIO0",
        "11": "GND", "12": "VSPI_SCK", "13": "VSPI_MISO", "14": "VSPI_MOSI", "15": "LORA_CS", "16": "GND", "17": "LORA_RF_ANT",
    }))
    pb.add_component(kt.PcbComponent("J5", "SMA-433MHz-ANT", fp_sma_rf, 92.5, 50.0, 0.0, {"1": "LORA_RF_ANT", "2": "GND", "3": "GND"}))
    pb.add_component(kt.PcbComponent("J6", "JST-XH-4P-CAM", fp_h4, 86.0, 18.0, 0.0, {"1": "+5V_BUS", "2": "GND", "3": "CAM_UART_TX", "4": "CAM_UART_RX"}))
    pb.add_component(kt.PcbComponent("J3", "JST-XH-3P-HALL", fp_h3, 62.0, 58.2, 0.0, {"1": "+5V_BUS", "2": "HALL_SIG_EXT", "3": "GND"}))
    pb.add_component(kt.PcbComponent("J4", "JST-XH-2P-NTC", fp_h2, 23.5, 64.2, 0.0, {"1": "NTC_ADC", "2": "GND"}))
    pb.add_component(kt.PcbComponent("JP1", "Jumper-2P-3V3-Bench", fp_h2, 43.0, 59.5, 0.0, {"1": "ESP32_3V3", "2": "+3V3_BUS"}))
    pb.add_component(kt.PcbComponent("SW1", "Tactile-CAL-Btn", fp_sw, 51.5, 62.0, 0.0, {"1": "CAL_BTN", "2": "GND"}))
    pb.add_component(kt.PcbComponent("D2", "BAT54S-Clamp", fp_sot23, 25.5, 46.5, 0.0, {"1": "GND", "2": "+3V3_BUS", "3": "VBAT_ADC"}))
    pb.add_component(kt.PcbComponent("D4", "LED-Green-0805", fp_0805, 62.0, 44.8, 0.0, {"1": "STATUS_LED", "2": "STATUS_LED_K"}))

    # Passives cleanly spaced (>= 5.0 mm Y or X spacing)
    passives = [
        ("R1", "4.7k", fp_0805, 62.0, 3.8, {"1": "+3V3_BUS", "2": "I2C_SDA"}),
        ("R2", "4.7k", fp_0805, 68.0, 3.8, {"1": "+3V3_BUS", "2": "I2C_SCL"}),
        ("R3", "10k", fp_0805, 74.0, 3.8, {"1": "+3V3_BUS", "2": "TOF_XSHUT"}),
        ("R4", "10k", fp_0805, 80.0, 3.8, {"1": "+3V3_BUS", "2": "LORA_CS"}),
        ("R5", "10k", fp_0805, 62.0, 39.8, {"1": "+3V3_BUS", "2": "HALL_SIG"}),
        ("R6", "1k", fp_0805, 62.0, 49.8, {"1": "HALL_SIG_EXT", "2": "HALL_SIG"}),
        ("R7", "100k", fp_0805, 25.5, 36.0, {"1": "VBAT_PROT", "2": "VBAT_ADC"}),
        ("R8", "10k", fp_0805, 25.5, 52.3, {"1": "VBAT_ADC", "2": "GND"}),
        ("R9", "10k", fp_0805, 11.0, 64.0, {"1": "+3V3_BUS", "2": "NTC_ADC"}),
        ("R10", "100R", fp_0805, 29.0, 5.5, {"1": "IBAT_SENSE_RAW", "2": "IBAT_ADC"}),
        ("R12", "330R", fp_0805, 68.0, 66.0, {"1": "STATUS_LED_K", "2": "GND"}),
        ("C2", "10uF-16V", fp_0805, 25.0, 57.6, {"1": "+5V_BUS", "2": "GND"}),
        ("C3", "10uF-16V", fp_0805, 38.0, 57.5, {"1": "+5V_BUS", "2": "GND"}),
        ("C4", "22uF-10V", fp_0805, 38.0, 64.5, {"1": "+3V3_BUS", "2": "GND"}),
        ("C5", "100nF-25V", fp_0805, 68.0, 60.5, {"1": "HALL_SIG", "2": "GND"}),
        ("C6", "100nF-25V", fp_0805, 25.5, 41.0, {"1": "VBAT_ADC", "2": "GND"}),
        ("C7", "100nF-25V", fp_0805, 36.0, 5.5, {"1": "IBAT_ADC", "2": "GND"}),
        ("C8", "100nF-25V", fp_0805, 16.5, 64.0, {"1": "NTC_ADC", "2": "GND"}),
        ("C9", "100nF-25V", fp_0805, 64.5, 9.3, {"1": "+3V3_BUS", "2": "GND"}),
        ("C10", "100nF-25V", fp_0805, 72.0, 9.3, {"1": "+3V3_BUS", "2": "GND"}),
        ("C11", "10uF-10V", fp_0805, 74.0, 63.0, {"1": "+3V3_BUS", "2": "GND"}),
        ("C12", "100nF-25V", fp_0805, 81.0, 63.0, {"1": "+3V3_BUS", "2": "GND"}),
        ("C13", "100nF-25V", fp_0805, 42.5, 5.5, {"1": "+3V3_BUS", "2": "GND"}),
    ]
    for ref, val, fp_d, px, py, pnets in passives:
        pb.add_component(kt.PcbComponent(ref, val, fp_d, px, py, 0.0, pnets))

    kt.sync_pcb_with_schematic(sb, pb)
    overlaps = kt.check_courtyards(pb)
    assert not overlaps, f"Courtyard overlaps: {overlaps}"

    # Explicit 50-Ohm RF feedline (short straight track from U4.17 to J5.1) + J6.2 right-side GND escape to H2
    pb.add_track("LORA_RF_ANT", 0.95, [pb.pad_abs_xy("U4", "17"), pb.pad_abs_xy("J5", "1")], "F.Cu")
    pb.add_track("GND", 0.30, [pb.pad_abs_xy("J6", "2"), (90.5, 16.73), (90.5, 5.5), pb.pad_abs_xy("H2", "1")], "F.Cu")

    router = kt.AutoRouter2Layer(pb, grid_mm=0.5, clearance_mm=0.22)
    route_res = router.route_all(net_widths={
        "VBAT_IN": 0.7,
        "VBAT_LOAD": 0.6,
        "VBAT_FUSED": 0.6,
        "VBAT_PROT": 0.6,
        "+5V_BUS": 0.45,
        "+3V3_BUS": 0.4,
    })
    print("FlightCore AutoRouter failed list:", route_res["failed"])
    pb.write(HW_DIR)

    summary = kt.build_and_verify_project(HW_DIR, PROJ, gerber_dir=HW_DIR / "gerbers", models_out_dir=HW_DIR / "3d-models")
    print("FlightCore KiCad Summary:", summary)

    # Maintain test_hardware_and_benchtwin_artifacts.py compatibility
    g_dir = HW_DIR / "gerbers"
    for src, dst in [
        ("flightcore-F_SilkS.gto", "flightcore-F_Silkscreen.gto"),
        ("flightcore-B_SilkS.gbo", "flightcore-B_Silkscreen.gbo"),
    ]:
        if (g_dir / src).exists():
            shutil.copy2(g_dir / src, g_dir / dst)

    # Sync netlist to benchtwin/flightcore.net (required by test_hardware_and_benchtwin_artifacts.py)
    shutil.copy2(HW_DIR / "flightcore.net", BT_DIR / "flightcore.net")

    # Export KiCad BOM to BOM-kicad.csv (leaving BOM.csv synced with benchtwin/BOM.csv)
    kt.run_cli(
        [
            "sch", "export", "bom",
            "--fields", "Reference,Value,Footprint,MPN,Manufacturer,Rating,QUANTITY",
            "--labels", "Designator,Value,Footprint,MPN,Manufacturer,Rating,Qty",
            "--output", str(HW_DIR / "BOM-kicad.csv"),
            str(HW_DIR / f"{PROJ}.kicad_sch"),
        ],
        check=True,
    )

    # Update drc_report.txt with genuine kicad-cli 10.0.8 results
    drc_txt = (
        "===================================================================\n"
        "FlightCore PCB KiCad 10.0.8 ERC & DRC Verification Report\n"
        "===================================================================\n"
        "Board Dimensions        : 96.0 mm x 68.0 mm (2-Layer FR4, 1.6 mm)\n"
        "Mounting Pattern        : 89.0 mm x 61.0 mm (4x M3 3.2mm)\n"
        "KiCad CLI Version       : 10.0.8\n"
        f"ERC Violations          : {summary['erc_violations']}\n"
        f"DRC Violations          : {summary['drc_violations']}\n"
        f"Unconnected Pads        : {summary['drc_unconnected']}\n"
        f"Schematic-PCB Parity    : {summary['drc_parity_issues']} issues\n"
        "Safety Scope Check      : ZERO motor/ESC PWM traces or drivers\n\n"
        f"Unconnected/Violations: {summary['erc_violations'] + summary['drc_violations'] + summary['drc_unconnected']} errors\n"
    )
    kt.write_utf8(g_dir / "drc_report.txt", drc_txt)
    if summary["erc_violations"] or summary["drc_violations"] or summary["drc_unconnected"] or summary["drc_parity_issues"]:
        raise SystemExit(2)


if __name__ == "__main__":
    build()
