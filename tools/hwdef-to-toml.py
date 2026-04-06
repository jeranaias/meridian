#!/usr/bin/env python3
"""
hwdef-to-toml.py — Convert ArduPilot hwdef.dat to Meridian board TOML config.

Usage:
    python tools/hwdef-to-toml.py path/to/hwdef.dat > boards/BoardName.toml

Reads an ArduPilot hwdef.dat file (following include directives) and emits a
Meridian-compatible TOML board configuration matching the BoardConfig struct
in crates/meridian-boardcfg/src/lib.rs.
"""

import sys
import os
import re
from pathlib import Path
from collections import OrderedDict


# ---------------------------------------------------------------------------
# MCU database — maps ArduPilot MCU type strings to Meridian enum + defaults
# ---------------------------------------------------------------------------

MCU_DATABASE = {
    "STM32H743xx": {
        "mcu": "STM32H743",
        "clock_hz": 400_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 1024,
    },
    "STM32H753xx": {
        "mcu": "STM32H753",
        "clock_hz": 400_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 1024,
    },
    "STM32H757xx": {
        "mcu": "STM32H757",
        "clock_hz": 400_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 1024,
    },
    "STM32H755xx": {
        "mcu": "STM32H755",
        "clock_hz": 400_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 1024,
    },
    "STM32H750xx": {
        "mcu": "STM32H750",
        "clock_hz": 400_000_000,
        "flash_size_kb": 128,
        "ram_size_kb": 1024,
    },
    "STM32H723xx": {
        "mcu": "STM32H723",
        "clock_hz": 550_000_000,
        "flash_size_kb": 1024,
        "ram_size_kb": 564,
    },
    "STM32H730xx": {
        "mcu": "STM32H730",
        "clock_hz": 550_000_000,
        "flash_size_kb": 128,
        "ram_size_kb": 564,
    },
    "STM32H7A3xx": {
        "mcu": "STM32H7A3",
        "clock_hz": 280_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 1344,
    },
    "STM32F405xx": {
        "mcu": "STM32F405",
        "clock_hz": 168_000_000,
        "flash_size_kb": 1024,
        "ram_size_kb": 192,
    },
    "STM32F407xx": {
        "mcu": "STM32F407",
        "clock_hz": 168_000_000,
        "flash_size_kb": 1024,
        "ram_size_kb": 192,
    },
    "STM32F412xx": {
        "mcu": "STM32F412",
        "clock_hz": 100_000_000,
        "flash_size_kb": 1024,
        "ram_size_kb": 256,
    },
    "STM32F427xx": {
        "mcu": "STM32F427",
        "clock_hz": 168_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 256,
    },
    "STM32F446xx": {
        "mcu": "STM32F446",
        "clock_hz": 180_000_000,
        "flash_size_kb": 512,
        "ram_size_kb": 128,
    },
    "STM32F745xx": {
        "mcu": "STM32F745",
        "clock_hz": 216_000_000,
        "flash_size_kb": 1024,
        "ram_size_kb": 320,
    },
    "STM32F765xx": {
        "mcu": "STM32F765",
        "clock_hz": 216_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 512,
    },
    "STM32F777xx": {
        "mcu": "STM32F777",
        "clock_hz": 216_000_000,
        "flash_size_kb": 2048,
        "ram_size_kb": 512,
    },
    "STM32G474xx": {
        "mcu": "STM32G474",
        "clock_hz": 170_000_000,
        "flash_size_kb": 512,
        "ram_size_kb": 128,
    },
    "STM32G491xx": {
        "mcu": "STM32G491",
        "clock_hz": 170_000_000,
        "flash_size_kb": 512,
        "ram_size_kb": 112,
    },
    "STM32L431xx": {
        "mcu": "STM32L431",
        "clock_hz": 80_000_000,
        "flash_size_kb": 256,
        "ram_size_kb": 64,
    },
    "STM32L476xx": {
        "mcu": "STM32L476",
        "clock_hz": 80_000_000,
        "flash_size_kb": 1024,
        "ram_size_kb": 128,
    },
}

# Port letter to index
PORT_MAP = {c: i for i, c in enumerate("ABCDEFGHIJK")}

# ArduPilot rotation names to Meridian enum variant names
ROTATION_MAP = {
    "ROTATION_NONE":                "None",
    "ROTATION_YAW_45":              "Yaw45",
    "ROTATION_YAW_90":              "Yaw90",
    "ROTATION_YAW_135":             "Yaw135",
    "ROTATION_YAW_180":             "Yaw180",
    "ROTATION_YAW_225":             "Yaw225",
    "ROTATION_YAW_270":             "Yaw270",
    "ROTATION_YAW_315":             "Yaw315",
    "ROTATION_ROLL_180":            "Roll180",
    "ROTATION_ROLL_180_YAW_45":     "Roll180Yaw45",
    "ROTATION_ROLL_180_YAW_90":     "Roll180Yaw90",
    "ROTATION_ROLL_180_YAW_135":    "Roll180Yaw135",
    "ROTATION_ROLL_180_YAW_180":    "Roll180Yaw180",
    "ROTATION_ROLL_180_YAW_225":    "Roll180Yaw225",
    "ROTATION_ROLL_180_YAW_270":    "Roll180Yaw270",
    "ROTATION_ROLL_180_YAW_315":    "Roll180Yaw315",
    "ROTATION_PITCH_180":           "Pitch180",
    "ROTATION_PITCH_270":           "Pitch270",
    "ROTATION_PITCH_90":            "Pitch90",
    "ROTATION_PITCH_180_YAW_90":    "Pitch180Yaw90",
    "ROTATION_PITCH_180_YAW_270":   "Pitch180Yaw270",
}

# ArduPilot IMU driver names to Meridian enum
IMU_DRIVER_MAP = {
    "Invensensev3": "Invensensev3",
    "Invensense":   "Invensense",
    "Invensensev2": "Invensensev3",  # v2 maps to same in Meridian
    "BMI088":       "Bmi088",
    "BMI270":       "Bmi270",
    "LSM6DSV":      "Lsm6dsv",
    "ADIS1647x":    "Adis1647x",
}

BARO_DRIVER_MAP = {
    "BMP280":    "Bmp280",
    "BMP388":    "Bmp388",
    "BMP390":    "Bmp390",
    "DPS310":    "Dps310",
    "SPL06":     "Spl06",
    "MS5611":    "Ms5611",
    "ICP201XX":  "Icp201xx",
}

COMPASS_DRIVER_MAP = {
    "IST8310":     "Ist8310",
    "QMC5883L":    "Qmc5883l",
    "RM3100":      "Rm3100",
    "LIS3MDL":     "Lis3mdl",
    "AK8963":      "Ak8963",
    "AK09916":     "Ak8963",  # AK09916 is the same family
    "BMM150":      "Bmm150",
}

# Known SPI alternate function numbers per MCU family
SPI_AF_MAP = {
    "STM32H7": {1: 5, 2: 5, 3: 6, 4: 5, 5: 5, 6: 5},
    "STM32F4": {1: 5, 2: 5, 3: 6, 4: 5},
    "STM32L4": {1: 5, 2: 5, 3: 6},
    "STM32G4": {1: 5, 2: 5, 3: 6},
    "STM32F7": {1: 5, 2: 5, 3: 6, 4: 5, 5: 5, 6: 5},
}

# ---------------------------------------------------------------------------
# Parser state
# ---------------------------------------------------------------------------

class HwdefParser:
    """Parses ArduPilot hwdef.dat files and extracts hardware configuration."""

    def __init__(self):
        # Board metadata
        self.mcu_family = ""
        self.mcu_type = ""
        self.board_id_name = ""
        self.oscillator_hz = 0
        self.flash_size_kb = 0
        self.clock_override_mhz = 0

        # Pin definitions: label -> (port_letter, pin_num, type, af, modifiers)
        self.pins = OrderedDict()

        # SPI buses: bus_num -> {sck, miso, mosi pins}
        self.spi_buses = {}

        # SPI devices: name -> {bus, devid, cs_label, mode, low_speed, high_speed}
        self.spi_devices = OrderedDict()

        # I2C buses
        self.i2c_order = []
        self.i2c_pins = {}  # bus_num -> {scl, sda}

        # Serial
        self.serial_order = []

        # UART pins: uart_name -> {tx, rx, cts, rts}
        self.uart_pins = {}

        # PWM outputs: list of (pin_label, timer, channel, pwm_num, gpio_num, complementary, bidir)
        self.pwm_outputs = []

        # ADC pins
        self.adc_pins = {}  # label -> {pin, adc, scale, channel_define}

        # Sensors
        self.imu_probes = []
        self.baro_probes = []
        self.compass_probes = []

        # DMA
        self.dma_noshare = []

        # Features
        self.has_sdcard = False
        self.has_iomcu = False
        self.iomcu_uart = ""
        self.has_ethernet = False
        self.has_can = False
        self.can_buses = set()

        # Defines
        self.defines = {}

        # Misc pins
        self.led_pins = []
        self.buzzer_pin = None
        self.safety_pin = None

        # USB
        self.usb_vid = ""
        self.usb_pid = ""
        self.usb_manufacturer = ""
        self.usb_product = ""

        # Track processed files to avoid infinite include loops
        self._processed_files = set()

    def parse_file(self, filepath):
        """Parse a hwdef.dat file, handling includes recursively."""
        filepath = Path(filepath).resolve()
        if filepath in self._processed_files:
            return
        self._processed_files.add(filepath)

        if not filepath.exists():
            print(f"# WARNING: file not found: {filepath}", file=sys.stderr)
            return

        with open(filepath, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip()
            # Strip comments
            if "#" in line:
                # Check for hex values that contain # (shouldn't happen in hwdef)
                comment_pos = line.index("#")
                line = line[:comment_pos].strip()
            if not line:
                continue
            self._process_line(line, filepath.parent)

    def _process_line(self, line, base_dir):
        """Process a single hwdef line."""
        tokens = line.split()
        if not tokens:
            return

        directive = tokens[0]

        # Include directive
        if directive == "include":
            if len(tokens) >= 2:
                inc_path = (base_dir / tokens[1]).resolve()
                self.parse_file(inc_path)
            return

        # MCU type
        if directive == "MCU":
            if len(tokens) >= 3:
                self.mcu_family = tokens[1]
                self.mcu_type = tokens[2]
            return

        # Board ID
        if directive == "APJ_BOARD_ID":
            if len(tokens) >= 2:
                self.board_id_name = tokens[1]
            return

        # Oscillator
        if directive == "OSCILLATOR_HZ":
            if len(tokens) >= 2:
                self.oscillator_hz = int(tokens[1])
            return

        # Flash size
        if directive == "FLASH_SIZE_KB":
            if len(tokens) >= 2:
                self.flash_size_kb = int(tokens[1])
            return

        # Clock override
        if directive == "MCU_CLOCKRATE_MHZ":
            if len(tokens) >= 2:
                self.clock_override_mhz = int(tokens[1])
            return

        # Serial order
        if directive == "SERIAL_ORDER":
            self.serial_order = tokens[1:]
            return

        # I2C order
        if directive == "I2C_ORDER":
            self.i2c_order = tokens[1:]
            return

        # CAN order
        if directive == "CAN_ORDER":
            return

        # IOMCU
        if directive == "IOMCU_UART":
            if len(tokens) >= 2:
                self.has_iomcu = True
                self.iomcu_uart = tokens[1]
            return

        # SPIDEV
        if directive == "SPIDEV":
            self._parse_spidev(tokens)
            return

        # DMA_NOSHARE
        if directive == "DMA_NOSHARE":
            self.dma_noshare.extend(tokens[1:])
            return

        # IMU
        if directive == "IMU":
            self._parse_imu(tokens)
            return

        # BARO
        if directive == "BARO":
            self._parse_baro(tokens)
            return

        # COMPASS
        if directive == "COMPASS":
            self._parse_compass(tokens)
            return

        # define
        if directive == "define":
            if len(tokens) >= 3:
                self.defines[tokens[1]] = " ".join(tokens[2:])
            elif len(tokens) == 2:
                self.defines[tokens[1]] = "1"
            return

        # undef
        if directive == "undef":
            if len(tokens) >= 2:
                key = tokens[1]
                self.defines.pop(key, None)
                if key == "IMU":
                    self.imu_probes.clear()
                elif key == "BARO":
                    self.baro_probes.clear()
            return

        # USB strings
        if directive == "USB_STRING_MANUFACTURER":
            self.usb_manufacturer = " ".join(tokens[1:]).strip('"')
            return
        if directive == "USB_STRING_PRODUCT":
            self.usb_product = " ".join(tokens[1:]).strip('"')
            return
        if directive == "USB_VENDOR":
            self.usb_vid = tokens[1]
            return
        if directive == "USB_PRODUCT":
            self.usb_pid = tokens[1]
            return

        # Pin definition: starts with P<port><pin>
        pin_match = re.match(r'^P([A-K])(\d+)$', directive)
        if pin_match:
            self._parse_pin(tokens, pin_match.group(1), int(pin_match.group(2)))
            return

        # Ignore other directives silently
        # (FLASH_RESERVE_START_KB, STORAGE_FLASH_PAGE, STM32_ST_USE_TIMER,
        #  DMA_PRIORITY, ROMFS, CANFD_SUPPORTED, env, PROCESS_STACK,
        #  MAIN_STACK, DEFAULTGPIO, ENABLE_DFU_BOOT, AUTOBUILD_TARGETS,
        #  CHECK_*, BOARD_VALIDATE, STDOUT_*, etc.)

    def _parse_pin(self, tokens, port, pin_num):
        """Parse a pin definition line."""
        if len(tokens) < 3:
            return

        label = tokens[1]
        pin_type = tokens[2]
        modifiers = tokens[3:]

        # Extract AF if present
        af = 0
        for m in modifiers:
            af_match = re.match(r'^AF(\d+)$', m)
            if af_match:
                af = int(af_match.group(1))

        # Store pin
        self.pins[label] = {
            "port": port,
            "pin": pin_num,
            "type": pin_type,
            "af": af,
            "modifiers": modifiers,
            "label": label,
        }

        # Classify pin
        # SPI bus pins
        spi_match = re.match(r'^SPI(\d+)_(SCK|MISO|MOSI)$', label)
        if spi_match and re.match(r'^SPI\d+$', pin_type):
            bus = int(spi_match.group(1))
            func = spi_match.group(2).lower()
            if bus not in self.spi_buses:
                self.spi_buses[bus] = {}
            self.spi_buses[bus][func] = {"port": port, "pin": pin_num, "af": af}

        # I2C bus pins
        i2c_match = re.match(r'^I2C(\d+)_(SCL|SDA)$', label)
        if i2c_match and re.match(r'^I2C\d+$', pin_type):
            bus = int(i2c_match.group(1))
            func = i2c_match.group(2).lower()
            if bus not in self.i2c_pins:
                self.i2c_pins[bus] = {}
            self.i2c_pins[bus][func] = {"port": port, "pin": pin_num, "af": af}

        # UART pins
        uart_match = re.match(r'^(USART\d+|UART\d+|LPUART\d+)_(TX|RX|CTS|RTS)$', label)
        if uart_match:
            uart_name = uart_match.group(1)
            func = uart_match.group(2).lower()
            if uart_name not in self.uart_pins:
                self.uart_pins[uart_name] = {}
            self.uart_pins[uart_name][func] = {"port": port, "pin": pin_num, "af": af}

        # CS pins
        if pin_type == "CS":
            pass  # Stored in self.pins, looked up by label

        # PWM outputs
        tim_match = re.match(r'^TIM(\d+)_CH(\d+)(N?)$', label)
        if tim_match:
            timer = int(tim_match.group(1))
            channel = int(tim_match.group(2))
            complementary = tim_match.group(3) == "N"
            pwm_num = 0
            gpio_num = 0
            bidir = False
            is_alarm = False
            for m in modifiers:
                pwm_match = re.match(r'^PWM\((\d+)\)$', m)
                if pwm_match:
                    pwm_num = int(pwm_match.group(1))
                gpio_match = re.match(r'^GPIO\((\d+)\)$', m)
                if gpio_match:
                    gpio_num = int(gpio_match.group(1))
                if m == "BIDIR":
                    bidir = True
                if m == "ALARM":
                    is_alarm = True
            if pwm_num > 0:
                self.pwm_outputs.append({
                    "timer": timer,
                    "channel": channel,
                    "port": port,
                    "pin": pin_num,
                    "af": af,
                    "complementary": complementary,
                    "pwm_num": pwm_num,
                    "gpio_num": gpio_num,
                    "bidir": bidir,
                })
            if is_alarm:
                self.buzzer_pin = {"port": port, "pin": pin_num}

        # ADC pins
        if re.match(r'^ADC[123]$', pin_type):
            scale = 1.0
            for m in modifiers:
                scale_match = re.match(r'^SCALE\((\d+(?:\.\d+)?)\)$', m)
                if scale_match:
                    scale = float(scale_match.group(1))
            self.adc_pins[label] = {
                "port": port,
                "pin": pin_num,
                "adc": pin_type,
                "scale": scale,
            }

        # SD card
        if re.match(r'^SDMMC\d+', pin_type) or pin_type == "SDIO":
            self.has_sdcard = True

        # CAN
        can_match = re.match(r'^CAN(\d+)$', pin_type)
        if can_match:
            self.has_can = True
            self.can_buses.add(int(can_match.group(1)))

        # Ethernet
        if pin_type == "ETH1":
            self.has_ethernet = True

        # LED pins
        if pin_type == "OUTPUT" and ("LED" in label):
            gpio_num = 0
            for m in modifiers:
                gpio_match = re.match(r'^GPIO\((\d+)\)$', m)
                if gpio_match:
                    gpio_num = int(gpio_match.group(1))
            self.led_pins.append({"label": label, "port": port, "pin": pin_num, "gpio": gpio_num})

        # Safety pin
        if "SAFETY" in label and pin_type == "INPUT":
            self.safety_pin = {"port": port, "pin": pin_num}

    def _parse_spidev(self, tokens):
        """Parse SPIDEV line."""
        # SPIDEV name bus devid cs_label mode low_speed high_speed
        if len(tokens) < 8:
            return
        name = tokens[1]
        bus_match = re.match(r'^SPI(\d+)$', tokens[2])
        if not bus_match:
            return
        bus = int(bus_match.group(1))
        # devid is DEVIDn
        cs_label = tokens[4]
        mode = 0
        mode_match = re.match(r'^MODE(\d)$', tokens[5])
        if mode_match:
            mode = int(mode_match.group(1))
        low_speed = self._parse_speed(tokens[6])
        high_speed = self._parse_speed(tokens[7])

        self.spi_devices[name] = {
            "bus": bus,
            "cs_label": cs_label,
            "mode": mode,
            "low_speed": low_speed,
            "high_speed": high_speed,
        }

    def _parse_speed(self, s):
        """Parse speed value like '2*MHZ' or '500*KHZ'."""
        s = s.strip()
        m = re.match(r'^(\d+)\*MHZ$', s)
        if m:
            return int(m.group(1)) * 1_000_000
        m = re.match(r'^(\d+)\*KHZ$', s)
        if m:
            return int(m.group(1)) * 1_000
        # Try plain number
        try:
            return int(s)
        except ValueError:
            return 0

    def _parse_imu(self, tokens):
        """Parse IMU probe line."""
        # IMU driver SPI:name ROTATION_xxx [BOARD_MATCH(...)]
        # IMU BMI088 SPI:accel SPI:gyro ROTATION_xxx [BOARD_MATCH(...)]
        if len(tokens) < 3:
            return
        driver = tokens[1]
        if driver not in IMU_DRIVER_MAP:
            print(f"# WARNING: unknown IMU driver '{driver}', skipping", file=sys.stderr)
            return

        # Skip BOARD_MATCH entries — those are sub-board variants
        # We include them but note it in comments
        has_board_match = any("BOARD_MATCH" in t for t in tokens)

        rotation = "None"
        spi_device = ""
        for t in tokens[2:]:
            if t.startswith("SPI:"):
                if not spi_device:  # take first SPI device (accel for BMI088)
                    spi_device = t[4:]
            elif t.startswith("I2C:"):
                # I2C IMU (rare)
                parts = t[4:].split(":")
                if len(parts) == 2:
                    bus_idx = int(parts[0])
                    addr = int(parts[1], 0)
                    self.imu_probes.append({
                        "driver": driver,
                        "bus_type": "I2c",
                        "bus_index": bus_idx,
                        "device_index": addr,
                        "rotation": rotation,
                        "board_match": has_board_match,
                    })
                    return
            elif t.startswith("ROTATION_"):
                rotation = ROTATION_MAP.get(t, "None")

        if spi_device:
            self.imu_probes.append({
                "driver": driver,
                "bus_type": "Spi",
                "spi_device": spi_device,
                "rotation": rotation,
                "board_match": has_board_match,
            })

    def _parse_baro(self, tokens):
        """Parse BARO probe line."""
        # BARO driver SPI:name | I2C:bus:addr
        if len(tokens) < 3:
            return
        driver = tokens[1]
        if driver not in BARO_DRIVER_MAP:
            print(f"# WARNING: unknown baro driver '{driver}', skipping", file=sys.stderr)
            return

        bus_token = tokens[2]
        if bus_token.startswith("I2C:"):
            parts = bus_token[4:].split(":")
            if len(parts) == 2:
                bus_idx = int(parts[0])
                addr = int(parts[1], 0)
                self.baro_probes.append({
                    "driver": driver,
                    "bus_type": "I2c",
                    "bus_index": bus_idx,
                    "address": addr,
                })
        elif bus_token.startswith("SPI:"):
            spi_name = bus_token[4:]
            self.baro_probes.append({
                "driver": driver,
                "bus_type": "Spi",
                "spi_device": spi_name,
            })

    def _parse_compass(self, tokens):
        """Parse COMPASS probe line."""
        # COMPASS driver bus:idx:addr external rotation
        # COMPASS AK09916:probe_ICM20948 0 ROTATION_xxx
        # COMPASS BMM150 I2C:0:0x10 false ROTATION_NONE
        if len(tokens) < 3:
            return

        driver_raw = tokens[1]
        # Handle probe syntax like AK09916:probe_ICM20948
        driver = driver_raw.split(":")[0]
        if driver not in COMPASS_DRIVER_MAP:
            print(f"# WARNING: unknown compass driver '{driver}', skipping", file=sys.stderr)
            return

        rotation = "None"
        external = True
        bus_type = "I2c"
        bus_index = 0
        address = 0

        for t in tokens[2:]:
            if t.startswith("I2C:"):
                parts = t[4:].split(":")
                if len(parts) == 2:
                    bus_index = int(parts[0])
                    address = int(parts[1], 0)
            elif t.startswith("SPI:"):
                bus_type = "Spi"
            elif t.startswith("ROTATION_"):
                rotation = ROTATION_MAP.get(t, "None")
            elif t == "false":
                external = False
            elif t == "true":
                external = True
            elif t.isdigit():
                bus_index = int(t)

        self.compass_probes.append({
            "driver": driver,
            "bus_type": bus_type,
            "bus_index": bus_index,
            "address": address,
            "rotation": rotation,
            "external": external,
        })


# ---------------------------------------------------------------------------
# TOML generation
# ---------------------------------------------------------------------------

def pin_encode(port_letter, pin_num):
    """Encode port+pin as 0xPPNN hex value."""
    port_idx = PORT_MAP.get(port_letter, 0)
    return f"0x{port_idx:02X}{pin_num:02X}"


def get_mcu_family_prefix(mcu_type):
    """Get MCU family prefix for SPI AF lookup."""
    if "H7" in mcu_type:
        return "STM32H7"
    elif "F4" in mcu_type:
        return "STM32F4"
    elif "F7" in mcu_type:
        return "STM32F7"
    elif "L4" in mcu_type:
        return "STM32L4"
    elif "G4" in mcu_type:
        return "STM32G4"
    return "STM32H7"


def resolve_spi_af(mcu_type, bus_num, pin_af):
    """Resolve SPI alternate function number."""
    if pin_af > 0:
        return pin_af
    prefix = get_mcu_family_prefix(mcu_type)
    af_map = SPI_AF_MAP.get(prefix, {})
    return af_map.get(bus_num, 5)


def resolve_i2c_af(mcu_type, bus_num, pin_af):
    """Resolve I2C alternate function number."""
    if pin_af > 0:
        return pin_af
    # Most STM32 I2C uses AF4
    return 4


def resolve_uart_index(uart_name):
    """Convert UART peripheral name to index number."""
    m = re.match(r'^(?:U?S?ART|UART|LPUART)(\d+)$', uart_name)
    if m:
        return int(m.group(1))
    return 0


def resolve_timer_af(mcu_type, timer_num, pin_af):
    """Resolve timer alternate function."""
    if pin_af > 0:
        return pin_af
    # Common defaults
    prefix = get_mcu_family_prefix(mcu_type)
    if prefix == "STM32H7":
        if timer_num in (1, 2):
            return 1
        if timer_num in (3, 4, 5):
            return 2
        if timer_num == 8:
            return 3
        if timer_num in (12,):
            return 9
        if timer_num in (15,):
            return 4
    elif prefix == "STM32F4":
        if timer_num in (1, 2):
            return 1
        if timer_num in (3, 4, 5):
            return 2
        if timer_num == 8:
            return 3
        if timer_num == 9:
            return 3
    return 1


def dma_noshare_matches(patterns, bus_num):
    """Check if a SPI bus matches DMA_NOSHARE patterns."""
    bus_str = f"SPI{bus_num}"
    for p in patterns:
        # Handle wildcards like SPI1* or SPI*
        regex = p.replace("*", ".*")
        if re.match(regex, bus_str):
            return True
        if re.match(regex, f"{bus_str}_"):
            return True
    return False


def generate_toml(parser):
    """Generate Meridian TOML from parsed hwdef."""
    mcu_info = MCU_DATABASE.get(parser.mcu_type, {})
    mcu_name = mcu_info.get("mcu", parser.mcu_type)
    clock_hz = mcu_info.get("clock_hz", 400_000_000)
    if parser.clock_override_mhz:
        clock_hz = parser.clock_override_mhz * 1_000_000
    flash_kb = parser.flash_size_kb or mcu_info.get("flash_size_kb", 0)
    ram_kb = mcu_info.get("ram_size_kb", 0)

    # Derive board name from file path or board_id
    board_name = parser.board_id_name.replace("AP_HW_", "").replace("TARGET_HW_", "")
    if not board_name:
        board_name = "Unknown"

    # Board ID number (we use 0 as placeholder — real value comes from board_types.txt)
    board_id = 0

    lines = []
    lines.append(f"# {board_name} — Meridian board configuration")
    lines.append(f"# Auto-generated from ArduPilot hwdef.dat by hwdef-to-toml.py")
    lines.append(f"# MCU: {mcu_name}, {parser.oscillator_hz // 1_000_000}MHz crystal, {clock_hz // 1_000_000}MHz clock")
    lines.append("")

    # [board]
    lines.append("[board]")
    lines.append(f'name = "{board_name}"')
    lines.append(f"board_id = {board_id}")
    lines.append(f'mcu = "{mcu_name}"')
    lines.append(f"clock_hz = {clock_hz}")
    lines.append(f"oscillator_hz = {parser.oscillator_hz}")
    lines.append(f"flash_size_kb = {flash_kb}")
    lines.append(f"ram_size_kb = {ram_kb}")
    lines.append("")

    # [usb]
    if parser.usb_manufacturer or parser.usb_vid:
        lines.append("[usb]")
        if parser.usb_manufacturer:
            lines.append(f'manufacturer = "{parser.usb_manufacturer}"')
        if parser.usb_product:
            lines.append(f'product = "{parser.usb_product}"')
        if parser.usb_vid:
            lines.append(f'vid = "{parser.usb_vid}"')
        if parser.usb_pid:
            lines.append(f'pid = "{parser.usb_pid}"')
        lines.append("")

    # [serial]
    if parser.serial_order:
        lines.append("[serial]")
        order_str = ", ".join(f'"{s}"' for s in parser.serial_order)
        lines.append(f"order = [{order_str}]")
        lines.append("")

    # [features]
    lines.append("[features]")
    lines.append(f"sdcard = {'true' if parser.has_sdcard else 'false'}")
    lines.append(f"iomcu = {'true' if parser.has_iomcu else 'false'}")
    lines.append(f"ethernet = {'true' if parser.has_ethernet else 'false'}")
    lines.append(f"can = {'true' if parser.has_can else 'false'}")
    if parser.has_can:
        lines.append(f"can_bus_count = {len(parser.can_buses)}")
    lines.append("")

    # [[spi_bus]]
    for bus_num in sorted(parser.spi_buses.keys()):
        bus = parser.spi_buses[bus_num]
        if "sck" not in bus or "miso" not in bus or "mosi" not in bus:
            continue
        af = resolve_spi_af(parser.mcu_type, bus_num, bus["sck"].get("af", 0))
        noshare = dma_noshare_matches(parser.dma_noshare, bus_num)
        lines.append("[[spi_bus]]")
        lines.append(f"index = {bus_num}")
        lines.append(f'sck = "{bus["sck"]["port"]}{bus["sck"]["pin"]}"')
        lines.append(f'miso = "{bus["miso"]["port"]}{bus["miso"]["pin"]}"')
        lines.append(f'mosi = "{bus["mosi"]["port"]}{bus["mosi"]["pin"]}"')
        lines.append(f"af = {af}")
        lines.append(f"dma_noshare = {'true' if noshare else 'false'}")
        lines.append("")

    # [[spi_device]]
    for dev_name, dev in parser.spi_devices.items():
        cs_label = dev["cs_label"]
        cs_pin = parser.pins.get(cs_label)
        if not cs_pin:
            lines.append(f"# WARNING: CS pin '{cs_label}' not found for SPIDEV {dev_name}")
            continue
        lines.append("[[spi_device]]")
        lines.append(f'name = "{dev_name}"')
        lines.append(f"bus = {dev['bus']}")
        lines.append(f'cs = "{cs_pin["port"]}{cs_pin["pin"]}"')
        lines.append(f"mode = {dev['mode']}")
        lines.append(f"low_speed_hz = {dev['low_speed']}")
        lines.append(f"high_speed_hz = {dev['high_speed']}")
        lines.append("")

    # [[i2c_bus]]
    for i, i2c_name in enumerate(parser.i2c_order):
        m = re.match(r'I2C(\d+)', i2c_name)
        if not m:
            continue
        phys_bus = int(m.group(1))
        bus_pins = parser.i2c_pins.get(phys_bus)
        if not bus_pins or "scl" not in bus_pins or "sda" not in bus_pins:
            lines.append(f"# WARNING: I2C{phys_bus} pins not found")
            continue
        af = resolve_i2c_af(parser.mcu_type, phys_bus, bus_pins["scl"].get("af", 0))
        # Determine internal vs external
        i2c_internal_mask = int(parser.defines.get("HAL_I2C_INTERNAL_MASK", "0"), 0)
        internal = bool(i2c_internal_mask & (1 << i))
        lines.append("[[i2c_bus]]")
        lines.append(f"index = {phys_bus}")
        lines.append(f"logical_index = {i}")
        lines.append(f'scl = "{bus_pins["scl"]["port"]}{bus_pins["scl"]["pin"]}"')
        lines.append(f'sda = "{bus_pins["sda"]["port"]}{bus_pins["sda"]["pin"]}"')
        lines.append(f"af = {af}")
        lines.append(f"speed_khz = 400")
        lines.append(f"internal = {'true' if internal else 'false'}")
        lines.append("")

    # [[uart]]
    for serial_idx, uart_name in enumerate(parser.serial_order):
        is_usb = uart_name.startswith("OTG")
        lines.append("[[uart]]")
        lines.append(f"serial_index = {serial_idx}")
        lines.append(f'peripheral = "{uart_name}"')
        if is_usb:
            lines.append(f"is_usb = true")
            uart_idx = 0
        else:
            uart_idx = resolve_uart_index(uart_name)
            lines.append(f"uart_index = {uart_idx}")
            pins = parser.uart_pins.get(uart_name, {})
            if "tx" in pins:
                lines.append(f'tx = "{pins["tx"]["port"]}{pins["tx"]["pin"]}"')
            if "rx" in pins:
                lines.append(f'rx = "{pins["rx"]["port"]}{pins["rx"]["pin"]}"')
            if "cts" in pins:
                lines.append(f'cts = "{pins["cts"]["port"]}{pins["cts"]["pin"]}"')
            if "rts" in pins:
                lines.append(f'rts = "{pins["rts"]["port"]}{pins["rts"]["pin"]}"')

        # Default baud and protocol based on serial index
        if serial_idx == 0:
            lines.append(f"default_baud = 115200")
            lines.append(f"default_protocol = 2  # MAVLink2")
        elif serial_idx in (1, 2):
            lines.append(f"default_baud = 57600")
            lines.append(f"default_protocol = 2  # MAVLink2")
        elif serial_idx in (3, 4):
            lines.append(f"default_baud = 115200")
            lines.append(f"default_protocol = 5  # GPS")
        else:
            lines.append(f"default_baud = 115200")
            lines.append(f"default_protocol = 0  # None")
        lines.append("")

    # [[pwm_output]]
    sorted_pwm = sorted(parser.pwm_outputs, key=lambda x: x["pwm_num"])
    for pwm in sorted_pwm:
        af = resolve_timer_af(parser.mcu_type, pwm["timer"], pwm["af"])
        lines.append("[[pwm_output]]")
        lines.append(f"pwm_num = {pwm['pwm_num']}")
        lines.append(f"timer = {pwm['timer']}")
        lines.append(f"channel = {pwm['channel']}")
        lines.append(f'pin = "{pwm["port"]}{pwm["pin"]}"')
        lines.append(f"af = {af}")
        lines.append(f"complementary = {'true' if pwm['complementary'] else 'false'}")
        lines.append("")

    # [[imu_probe]]
    for imu in parser.imu_probes:
        driver = IMU_DRIVER_MAP.get(imu["driver"], imu["driver"])
        bus_type = imu.get("bus_type", "Spi")
        rotation = imu.get("rotation", "None")

        lines.append("[[imu_probe]]")
        lines.append(f'driver = "{driver}"')
        lines.append(f'bus_type = "{bus_type}"')

        if bus_type == "Spi" and "spi_device" in imu:
            spi_dev_name = imu["spi_device"]
            spi_dev = parser.spi_devices.get(spi_dev_name)
            if spi_dev:
                lines.append(f"bus_index = {spi_dev['bus']}")
                # Find device index in order of spi_devices on same bus
                dev_idx = 0
                for name, d in parser.spi_devices.items():
                    if name == spi_dev_name:
                        break
                    if d["bus"] == spi_dev["bus"]:
                        dev_idx += 1
                lines.append(f'spi_device = "{spi_dev_name}"')
            else:
                lines.append(f'spi_device = "{spi_dev_name}"  # WARNING: device not in SPIDEV table')
        else:
            lines.append(f"bus_index = {imu.get('bus_index', 0)}")
            lines.append(f"address = {imu.get('device_index', 0)}")

        lines.append(f'rotation = "{rotation}"')
        if imu.get("board_match"):
            lines.append(f"# NOTE: has BOARD_MATCH — may only be present on certain board revisions")
        lines.append("")

    # [[baro_probe]]
    for baro in parser.baro_probes:
        driver = BARO_DRIVER_MAP.get(baro["driver"], baro["driver"])
        bus_type = baro.get("bus_type", "I2c")

        lines.append("[[baro_probe]]")
        lines.append(f'driver = "{driver}"')
        lines.append(f'bus_type = "{bus_type}"')

        if bus_type == "I2c":
            lines.append(f"bus_index = {baro['bus_index']}")
            lines.append(f"address = 0x{baro['address']:02X}")
        elif bus_type == "Spi":
            spi_dev_name = baro.get("spi_device", "")
            spi_dev = parser.spi_devices.get(spi_dev_name)
            if spi_dev:
                lines.append(f"bus_index = {spi_dev['bus']}")
                lines.append(f'spi_device = "{spi_dev_name}"')
            else:
                lines.append(f'spi_device = "{spi_dev_name}"')
        lines.append("")

    # [[compass_probe]]
    for comp in parser.compass_probes:
        driver = COMPASS_DRIVER_MAP.get(comp["driver"], comp["driver"])
        lines.append("[[compass_probe]]")
        lines.append(f'driver = "{driver}"')
        lines.append(f'bus_type = "{comp["bus_type"]}"')
        lines.append(f"bus_index = {comp['bus_index']}")
        lines.append(f"address = 0x{comp['address']:02X}")
        lines.append(f'rotation = "{comp["rotation"]}"')
        lines.append(f"external = {'true' if comp['external'] else 'false'}")
        lines.append("")

    # [adc]
    volt_pin = parser.defines.get("HAL_BATT_VOLT_PIN", "0")
    curr_pin = parser.defines.get("HAL_BATT_CURR_PIN", "0")
    volt_scale = parser.defines.get("HAL_BATT_VOLT_SCALE", "11.0")
    curr_scale = parser.defines.get("HAL_BATT_CURR_SCALE", "17.0")
    lines.append("[adc]")
    lines.append(f"voltage_pin = {volt_pin}")
    lines.append(f"current_pin = {curr_pin}")
    lines.append(f"voltage_scale = {volt_scale}")
    lines.append(f"current_scale = {curr_scale}")
    lines.append("")

    # [misc] — LED, buzzer, safety
    if parser.led_pins or parser.buzzer_pin or parser.safety_pin:
        lines.append("[misc]")
        if parser.led_pins:
            led = parser.led_pins[0]
            lines.append(f'led_pin = "{led["port"]}{led["pin"]}"')
        if parser.buzzer_pin:
            lines.append(f'buzzer_pin = "{parser.buzzer_pin["port"]}{parser.buzzer_pin["pin"]}"')
        if parser.safety_pin:
            lines.append(f'safety_pin = "{parser.safety_pin["port"]}{parser.safety_pin["pin"]}"')
        lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python hwdef-to-toml.py <hwdef.dat>", file=sys.stderr)
        print("  Converts an ArduPilot hwdef.dat to Meridian board TOML config.", file=sys.stderr)
        sys.exit(1)

    hwdef_path = sys.argv[1]
    parser = HwdefParser()
    parser.parse_file(hwdef_path)
    print(generate_toml(parser))


if __name__ == "__main__":
    main()
