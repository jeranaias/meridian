# ArduPilot Board Configuration System Audit

**Target**: ArduPilot `libraries/AP_HAL_ChibiOS/hwdef/` system  
**Meridian target MCU**: STM32H743 (MatekH743-WING as reference board)  
**Date**: 2026-04-02  
**Auditor**: Automated audit

---

## 1. hwdef.dat Format

### What is hwdef.dat?

`hwdef.dat` is a board definition file written in a custom DSL. It fully describes the hardware wiring of a flight controller — every pin assignment, peripheral assignment, SPI/I2C/UART topology, DMA constraints, and C preprocessor defines needed to compile ArduPilot for that board.

One hwdef.dat file exists per board (in `libraries/AP_HAL_ChibiOS/hwdef/<BoardName>/hwdef.dat`). A companion `hwdef-bl.dat` covers the bootloader build, which is a stripped-down version. The processing script (`chibios_hwdef.py`) reads these and emits `hwdef.h`, `ldscript.ld`, and `common.ld` into the build directory.

Comments use `#`. Lines are space-separated tokens. A file can include another with:
```
include ../OtherBoard/hwdef.dat
```
This is the primary inheritance mechanism — peripheral boards (e.g. `MatekH743-periph`) start with `include ../MatekH743/hwdef.dat` then `undef` whatever they don't need.

---

### Every Directive Type

#### MCU and Clock

| Directive | Example | Meaning |
|---|---|---|
| `MCU` | `MCU STM32H7xx STM32H743xx` | First arg: ChibiOS MCU family string. Second arg: exact MCU type — must match a `.py` file in `hwdef/scripts/` |
| `OSCILLATOR_HZ` | `OSCILLATOR_HZ 8000000` | External crystal frequency. If 0, HSI is used. The H7 mcuconf.h has cases for 8/12/16/24/25/32 MHz only |
| `STM32_ST_USE_TIMER` | `STM32_ST_USE_TIMER 12` | Which STM32 timer ChibiOS uses for its system tick. Must be 32-bit. Common choices: 2, 5, 12 |
| `define CH_CFG_ST_RESOLUTION 16` | (define) | ChibiOS timer resolution. Use 16 for 16-bit timers (TIM12), 32 for TIM5 |
| `MCU_CLOCKRATE_MHZ` | `MCU_CLOCKRATE_MHZ 480` | Override default 400 MHz to request 480 MHz (H743 rev V silicon only) |

#### Flash and Memory Layout

| Directive | Example | Meaning |
|---|---|---|
| `FLASH_SIZE_KB` | `FLASH_SIZE_KB 2048` | Total internal flash in KB. Used to generate ldscript |
| `FLASH_RESERVE_START_KB` | `FLASH_RESERVE_START_KB 128` | Offset from flash start where main firmware is placed. Bootloader occupies 0 to this offset |
| `FLASH_BOOTLOADER_LOAD_KB` | `FLASH_BOOTLOADER_LOAD_KB 128` | In hwdef-bl.dat: where the bootloader will load the app firmware from (i.e. the expected fw start) |
| `STORAGE_FLASH_PAGE` | `STORAGE_FLASH_PAGE 14` | Which flash page (0-indexed) to use for parameter storage. H743 has 16 pages of 128 KB each |
| `EXT_FLASH_SIZE_MB` | `EXT_FLASH_SIZE_MB 32` | External QSPI/OSPI flash size in MB |
| `define HAL_STORAGE_SIZE` | `define HAL_STORAGE_SIZE 32768` | How many bytes of parameter storage to allocate |

#### Pin Definitions

The core format is:
```
P<Port><Pin>  <Label>  <Type>  [<AF>]  [<Modifiers>...]
```

- **Port/Pin**: Letter A–K + number 0–15 (e.g. `PA5`, `PD7`, `PE14`)
- **Label**: Logical name. For peripherals this must exactly match the function name in the MCU's alternate function table (e.g. `SPI1_SCK`, `USART1_TX`, `TIM8_CH2N`). For other pins, a descriptive name is used (e.g. `IMU1_CS`, `LED0`, `BATT_VOLTAGE_SENS`)
- **Type**: One of the peripheral class keywords below
- **AF** (alternate function number): Explicit AF override (e.g. `AF5`). Usually inferred automatically from the MCU's AF table in `STM32H743xx.py`, but can be forced
- **Modifiers**: Zero or more extra tokens that refine pin behavior

**Valid Type keywords:**
- `SPI1`–`SPI6`: SPI bus assignment
- `I2C1`–`I2C4`: I2C bus assignment
- `USART1`–`USART6`, `UART4`–`UART8`, `LPUART1`: UART assignment
- `OTG1`, `OTG2`: USB OTG
- `TIM1`–`TIM17`: Timer (used for PWM, RC input, buzzer)
- `ADC1`, `ADC2`, `ADC3`: Analog input
- `SDMMC1`, `SDMMC2`, `SDIO`: SD card
- `CAN1`, `CAN2`: CAN bus
- `QUADSPI1`, `OCTOSPI`: QSPI/OSPI flash
- `SWD`: SWD debug pins (JTMS-SWDIO, JTCK-SWCLK)
- `CS`: Chip select (software-controlled GPIO, active low)
- `INPUT`: GPIO input
- `OUTPUT`: GPIO output
- `ETH1`: Ethernet MAC

**Pin modifiers (4th+ fields):**

| Modifier | Meaning |
|---|---|
| `LOW` / `HIGH` | Default output state |
| `PULLUP` / `PULLDOWN` / `FLOATING` | Input pull configuration |
| `OPENDRAIN` / `PUSHPULL` | Output driver type |
| `SPEED_VERYLOW` / `SPEED_LOW` / `SPEED_MEDIUM` / `SPEED_HIGH` | GPIO slew rate |
| `GPIO(n)` | Expose this pin as ArduPilot GPIO number n (accessible via hal.gpio) |
| `PWM(n)` | This TIM channel is PWM output n (1-indexed). Sets motor/servo output ordering |
| `ADC1` / `ADC2` / `ADC3` | Which ADC instance this analog pin belongs to |
| `SCALE(x)` | Voltage divider multiplier for ADC (e.g. `SCALE(2)` means the actual voltage is 2x the pin voltage) |
| `ALARM` | This TIM pin is the buzzer output |
| `RCIN` | This TIM pin is the RC input (PPM capture mode) |
| `RCININT` | This TIM pin is RC input using UART capture mode (e.g. CRSF, DSM). Usually combined with a USART ALT config on the same physical pin |
| `NODMA` | Disable DMA for this UART (software-only IO) |
| `ALT(n)` | Register this pin as alternate config number n. Alt configs allow one physical pin to serve different purposes at runtime |
| `BIDIR` | Bidirectional DShot on this TIM channel |
| `UP_SHARED` | Timer update channel is shared |
| `LOW_NOISE` | Mark RX pin as low-noise input |

#### Peripheral Ordering

| Directive | Example | Meaning |
|---|---|---|
| `SERIAL_ORDER` | `SERIAL_ORDER OTG1 UART7 USART1 USART2 USART3 UART8 UART4 USART6 OTG2` | Maps SERIALn_ parameter index to physical UART. Position 0 = SERIAL0 (console/USB), 1 = SERIAL1 (telem1), 3 = GPS, etc. Use `EMPTY` to skip a slot |
| `I2C_ORDER` | `I2C_ORDER I2C2 I2C1` | Maps hal.i2c bus index 0,1,... to physical I2C peripherals. Index 0 is conventionally "internal" |
| `IOMCU_UART` | `IOMCU_UART USART6` | Which UART connects to a separate IOMCU (IO MCU) if present |

#### SPI Device Table

```
SPIDEV <name>  <bus>  <devid>  <cs_label>  <mode>  <lowspeed>  <highspeed>
```

Example:
```
SPIDEV mpu6000   SPI1 DEVID4 MPU_CS   MODE3  2*MHZ  8*MHZ
SPIDEV ms5611    SPI1 DEVID3 BARO_CS  MODE3 20*MHZ 20*MHZ
SPIDEV ramtron   SPI2 DEVID10 FRAM_CS MODE3  8*MHZ  8*MHZ
```

- `<name>`: String key used by drivers to `open()` the device. Must match expected device names in driver code.
- `<bus>`: SPI1–SPI6. Must have corresponding SCK/MISO/MOSI pins defined.
- `<devid>`: DEVID1–DEVID15. Creates a unique device ID bitmask for parameter persistence (so ArduPilot can detect when a sensor is removed/added between boots). The DEVID number is just a slot index within the bus.
- `<cs_label>`: Must match a pin label that was declared with type `CS`.
- `<mode>`: MODE0 (CPOL=0,CPHA=0), MODE1 (CPOL=0,CPHA=1), MODE2 (CPOL=1,CPHA=0), MODE3 (CPOL=1,CPHA=1)
- `<lowspeed>`: Speed for initialization/configuration transactions (e.g. `1*MHZ`, `500*KHZ`)
- `<highspeed>`: Speed for data transactions (e.g. `8*MHZ`, `16*MHZ`)

Speed values are parsed literally as `N*MHZ` or `N*KHZ`. The actual achievable clock is rounded down to the nearest SPI prescaler value.

#### Sensor Configuration

```
IMU <driver> SPI:<spidev_name>|I2C:<bus>:<addr>  ROTATION_xxx
BARO <driver> SPI:<spidev_name>|I2C:<bus>:<addr>
```

Examples:
```
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180
IMU Invensense   SPI:mpu6000  ROTATION_ROLL_180_YAW_270
BARO MS5611 I2C:0:0x77
BARO DPS310 I2C:0:0x76
```

Multiple IMU/BARO lines create an ordered probe list. The driver tries each in order at boot. `undef IMU` clears the entire list. I2C bus numbers here are ArduPilot logical bus numbers (0 = first bus in `I2C_ORDER`, etc.), not physical peripheral numbers.

#### DMA Configuration

| Directive | Example | Meaning |
|---|---|---|
| `DMA_PRIORITY` | `DMA_PRIORITY TIM* SPI*` | Space-separated list of wildcard patterns. Earlier patterns get DMA channels first during the resolver's constraint satisfaction run |
| `DMA_NOSHARE` | `DMA_NOSHARE SPI1* SPI4*` | These peripherals cannot share a DMA stream with anything else. Used for high-speed IMU buses to prevent jitter |
| `DMA_PRIORITY S*` | `DMA_PRIORITY S*` | A wildcard matching all SDMMC/SPI peripherals (common pattern) |
| `NODMA` | (on a pin line) | Disable DMA on a specific UART TX or RX |

#### Other Directives

| Directive | Example | Meaning |
|---|---|---|
| `define` | `define HAL_STORAGE_SIZE 32768` | Literal `#define` in the generated hwdef.h |
| `undef` | `undef SDMMC1` | Remove a previous configuration (pins, defines, sensor lists, or device type) |
| `env` | `env AP_PERIPH 1` | Set a build environment variable (passed to Makefile, not injected into C headers) |
| `APJ_BOARD_ID` | `APJ_BOARD_ID AP_HW_MATEKH743` | Board ID name, resolved to a number from `Tools/AP_Bootloader/board_types.txt`. Used by the bootloader to validate firmware compatibility |
| `USB_STRING_MANUFACTURER` | `USB_STRING_MANUFACTURER "ArduPilot"` | USB descriptor string |
| `ROMFS` | `ROMFS io_firmware.bin Tools/IO_Firmware/iofirmware_lowpolh.bin` | Embed a file into the ROMFS filesystem in flash |
| `ROMFS_WILDCARD` | `ROMFS_WILDCARD libraries/AP_OSD/fonts/font*.bin` | Embed all matching files into ROMFS |
| `DATAFLASH` | `DATAFLASH littlefs:w25nxx` | Configure an external flash chip as the dataflash/logging target |
| `PROCESS_STACK` | `PROCESS_STACK 0x6000` | Size of the main task process stack |
| `MAIN_STACK` | `MAIN_STACK 0x2000` | Size of the interrupt/startup stack |
| `QSPIDEV` / `OSPIDEV` | `OSPIDEV flash OSPI1 ...` | QSPI/OctoSPI device table (same format as SPIDEV) |
| `AUTOBUILD_TARGETS` | `AUTOBUILD_TARGETS None` | Which vehicle binaries to auto-build for this board. `None` means it's a dev/eval board only |
| `BOOTLOADER_DEV_LIST` | (implicit) | List of devices (USB, UART) the bootloader listens on |

---

## 2. MatekH743 Complete Pin Assignment

**Board**: Matek H743-WING (all hardware revisions share this hwdef)  
**MCU**: STM32H743 @ 400 MHz (8 MHz HSE crystal)  
**Firmware flash start**: 128 KB offset (bootloader in first 128 KB sector)  
**Parameter storage**: Flash pages 14–15 (last two 128 KB pages of 2 MB flash)

### SPI Buses

| Bus | Pins | Devices |
|---|---|---|
| SPI1 | SCK=PA5, MISO=PA6, MOSI=PD7 | ICM42688P (DEVID1, CS=PC15), MPU6000 (DEVID1, CS=PC15) — alternate hardware variants share the same CS slot |
| SPI2 | SCK=PB13, MISO=PB14, MOSI=PB15 | MAX7456 OSD (DEVID4, CS=PB12, MODE0, 10 MHz) |
| SPI3 | SCK=PB3, MISO=PB4, MOSI=PB5 | External connector — PixartFlow (DEVID1, CS=PD4); EXT_CS2=PE2 available |
| SPI4 | SCK=PE12, MISO=PE13, MOSI=PE14 | ICM20602 (DEVID1, CS=PE11), ICM42605 (DEVID1, CS=PC13) — alternate hardware variants |

SPI1 and SPI4 have `DMA_NOSHARE` — dedicated DMA channels to avoid jitter on the primary IMU buses.

### CS Pins

| Label | Pin | Connected To |
|---|---|---|
| `IMU1_CS` | PC15 | SPI1: ICM42688P / MPU6000 |
| `IMU2_CS` | PE11 | SPI4: ICM20602 |
| `IMU3_CS` | PC13 | SPI4: ICM42605 |
| `MAX7456_CS` | PB12 | SPI2: MAX7456 OSD |
| `EXT_CS1` | PD4 | SPI3: External device slot 1 |
| `EXT_CS2` | PE2 | SPI3: External device slot 2 |

### I2C Buses

| ArduPilot Bus Index | Physical | Pins | Devices |
|---|---|---|---|
| 0 (internal) | I2C2 | SCL=PB10, SDA=PB11 | DPS310/BMP280 baro @ 0x76, MS5611 @ 0x77 |
| 1 (external) | I2C1 | SCL=PB6, SDA=PB7 | All external compass types probed automatically |

`HAL_I2C_INTERNAL_MASK 0` means no bus is designated "internal" — all compass probing is external-only. `AP_COMPASS_PROBING_ENABLED 1` enables auto-probing all known compass types on I2C.

### UARTs (SERIAL_ORDER mapping)

| SERIAL# | Physical UART | Pins | Default Use |
|---|---|---|---|
| SERIAL0 | OTG1 (USB) | PA11 DM, PA12 DP | MAVLink console |
| SERIAL1 | UART7 | RX=PE7, TX=PE8, CTS=PE10, RTS=PE9 | Telem 1 (with flow control) |
| SERIAL2 | USART1 | RX=PA10, TX=PA9 | Telem 2 |
| SERIAL3 | USART2 | TX=PD5, RX=PD6 | GPS 1 |
| SERIAL4 | USART3 | RX=PD9, TX=PD8 | GPS 2 |
| SERIAL5 | UART8 | RX=PE0, TX=PE1 | Spare |
| SERIAL6 | UART4 | TX=PB9, RX=PB8 | Spare |
| SERIAL7 | USART6 | TX=PC6 (NODMA), RX=PC7 | RC input (primary) |
| SERIAL8 | OTG2 | (second USB endpoint) | Secondary MAVLink |

USART6 (SERIAL7) has dual config:
- Primary: PC7 = `TIM3_CH2 TIM3 RCININT PULLDOWN LOW` (hardware timer RC capture for CRSF/SBUS/DSM)
- Alt config 1: PC7 = `USART6_RX USART6 NODMA ALT(1)` (bi-directional UART protocol like FPort without inverter)

### PWM Outputs

| PWM# | Pin | Timer | GPIO# | Notes |
|---|---|---|---|---|
| PWM1 | PB0 | TIM8_CH2N | GPIO(50) | Complementary output |
| PWM2 | PB1 | TIM8_CH3N | GPIO(51) | Complementary output |
| PWM3 | PA0 | TIM5_CH1 | GPIO(52) | |
| PWM4 | PA1 | TIM5_CH2 | GPIO(53) | |
| PWM5 | PA2 | TIM5_CH3 | GPIO(54) | |
| PWM6 | PA3 | TIM5_CH4 | GPIO(55) | |
| PWM7 | PD12 | TIM4_CH1 | GPIO(56) | |
| PWM8 | PD13 | TIM4_CH2 | GPIO(57) | |
| PWM9 | PD14 | TIM4_CH3 | GPIO(58) | |
| PWM10 | PD15 | TIM4_CH4 | GPIO(59) | |
| PWM11 | PE5 | TIM15_CH1 | GPIO(60) | |
| PWM12 | PE6 | TIM15_CH2 | GPIO(61) | |
| PWM13 | PA8 | TIM1_CH1 | GPIO(62) | Also WS2812 LED output |

Timers used for PWM: TIM1, TIM4, TIM5, TIM8, TIM15. Note TIM8_CH2N and CH3N are complementary (N-suffix) outputs — these drive the high side of a half-bridge or inverted.

### LEDs and Buzzer

| Label | Pin | Default State | GPIO# | Notes |
|---|---|---|---|---|
| `LED0` | PE3 | LOW (on) | GPIO(90) | Blue, labeled "ACT" |
| `LED1` | PE4 | LOW (on) | GPIO(91) | Green, labeled "B/E" |
| Buzzer | PA15 | — | GPIO(32) | TIM2_CH1, ALARM type |

`HAL_GPIO_A_LED_PIN 91` (green), `HAL_GPIO_B_LED_PIN 90` (blue).

### ADC Pins

| Label | Pin | ADC | Channel | Scale | ArduPilot Pin# | Notes |
|---|---|---|---|---|---|---|
| BATT_VOLTAGE_SENS | PC0 | ADC1 | — | 1x | 10 | Battery 1 voltage |
| BATT_CURRENT_SENS | PC1 | ADC1 | — | 1x | 11 | Battery 1 current |
| BATT2_VOLTAGE_SENS | PA4 | ADC1 | — | 1x | 18 | Battery 2 voltage |
| BATT2_CURRENT_SENS | PA7 | ADC1 | — | 1x | 7 | Battery 2 current |
| PRESSURE_SENS | PC4 | ADC1 | — | 2x | 4 | Airspeed (HAL_DEFAULT_AIRSPEED_PIN 4) |
| RSSI_ADC | PC5 | ADC1 | — | — | 8 | RSSI analog input |

Default voltage scales: `HAL_BATT_VOLT_SCALE 11.0`, `HAL_BATT_CURR_SCALE 40.0`.

### CAN Bus

| Label | Pin | Notes |
|---|---|---|
| CAN1_RX | PD0 | |
| CAN1_TX | PD1 | |
| GPIO_CAN1_SILENT | PD3 | OUTPUT, PUSHPULL, SPEED_LOW, default LOW. Drives CAN transceiver silent pin |

### SD Card

SDMMC1 used (4-bit SDIO-equivalent):  
PC8=D0, PC9=D1, PC10=D2, PC11=D3, PC12=CK, PD2=CMD

### Miscellaneous GPIO

| Label | Pin | GPIO# | Default | Notes |
|---|---|---|---|---|
| PINIO1 | PD10 | GPIO(81) | LOW | General purpose switched power output |
| PINIO2 | PD11 | GPIO(82) | LOW | General purpose switched power output |

### IMU Configuration (probe order)

```
IMU Invensensev3 SPI:icm42688 ROTATION_YAW_180      # H743-V3
IMU Invensensev3 SPI:icm42605 ROTATION_YAW_270      # H743-V1.5/V2
IMU Invensense   SPI:icm20602 ROTATION_ROLL_180_YAW_270  # H743-V1
IMU Invensense   SPI:mpu6000  ROTATION_ROLL_180_YAW_270  # H743-V1
```

ArduPilot probes all four at startup. Whichever responds gets used. The rotation constants correct for the physical board orientation of each sensor.

### Barometer (I2C bus 0 = I2C2)

```
BARO MS5611 I2C:0:0x77
BARO DPS310 I2C:0:0x76
BARO BMP280 I2C:0:0x76
```

### SPIDEV Full Table (MatekH743)

```
SPIDEV icm42688    SPI1 DEVID1 IMU1_CS    MODE3  2*MHZ 16*MHZ
SPIDEV mpu6000     SPI1 DEVID1 IMU1_CS    MODE3  1*MHZ  4*MHZ
SPIDEV icm20602    SPI4 DEVID1 IMU2_CS    MODE3  1*MHZ  4*MHZ
SPIDEV icm42605    SPI4 DEVID1 IMU3_CS    MODE3  2*MHZ 16*MHZ
SPIDEV osd         SPI2 DEVID4 MAX7456_CS MODE0 10*MHZ 10*MHZ
SPIDEV pixartflow  SPI3 DEVID1 EXT_CS1    MODE3  2*MHZ  2*MHZ
```

Note: icm42688 and mpu6000 share DEVID1/IMU1_CS — at most one is present per board revision. Same for icm20602 / icm42605 on SPI4.

---

## 3. How hwdef.dat Becomes C Code

### Processing Pipeline

1. **Trigger**: `waf configure --board MatekH743` calls `generate_hwdef_h()` in `Tools/ardupilotwaf/chibios.py`
2. **Instantiation**: Creates `ChibiOSHWDef(hwdef=['...MatekH743/hwdef.dat'])` and calls `.run()`
3. **Parsing**: `process_hwdefs()` reads all lines, handles `include` directives recursively, dispatches each line to `process_line()`
4. **Pin resolution**: Each `Pxy ...` line looks up the alternate function number in `STM32H743xx.py`'s `AltFunction_map`. The AF number is stored in the pin object.
5. **DMA resolution**: `dma_resolver.write_dma_header()` solves a constraint satisfaction problem to assign DMAMUX channels to all peripherals. It respects `DMA_PRIORITY` ordering and `DMA_NOSHARE` exclusions.
6. **Output generation**: Multiple `write_*()` functions emit sections of `hwdef.h`

### Output Files Generated

| File | Contents |
|---|---|
| `hwdef.h` | All hardware defines: MCU type, crystal frequency, SPI/I2C/UART/ADC/GPIO/PWM/DMA configuration tables, board-specific `#define` values |
| `ldscript.ld` | Board-specific linker script: flash origin/size, RAM region layout from `STM32H743xx.py`'s `RAM_MAP` |
| `common.ld` | Copied from `hwdef/common/common.ld` — shared linker fragments |
| `hw.dat` | Raw copy of all hwdef lines (stored in ROMFS for runtime introspection) |

### Key hwdef.h Sections

**MCU identification**:
```c
#define STM32H7xx_MCUCONF
#define STM32H743xx
#define STM32_HSECLK 8000000U
```

**SPI bus table** (one entry per bus):
```c
#define HAL_SPI1_CONFIG { &SPID1, 1, STM32_SPI_SPI1_DMA_STREAMS, PAL_LINE(GPIOA,5U) }
```

**SPI device table** (one entry per SPIDEV):
```c
#define HAL_SPI_DEVICE0 SPIDesc("icm42688", 0, 1, PAL_LINE(GPIOC,15U), SPIDEV_MODE3, 2*MHZ, 16*MHZ)
```

**UART config** (one entry per serial):
```c
#define HAL_SERIAL0_DRIVER ChibiOS::UARTDriver serial0Driver(0)
#define HAL_UART7_CONFIG { ... tx_line, rx_line, rts_line, cts_line ... }
```

**DMA assignment** (solved by dma_resolver):
```c
#define STM32_SPI_SPI1_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 0)
#define STM32_UART_UART7_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 1)
```

**GPIO table**:
```c
#define HAL_GPIO_PINS { ... PAL_LINE/direction/default for each GPIO(n) pin ... }
```

**PWM config**:
```c
#define HAL_PWM_COUNT 13
#define HAL_SERVO_PWM_PINS { PAL_LINE(GPIOB,0U), PAL_LINE(GPIOB,1U), ... }
```

### How the ChibiOS MCU Config Works

`hwdef.h` is included by `board.h` (from `hwdef/common/board.h`). `board.h` is included by ChibiOS's `halconf.h`, which is included by `mcuconf.h`. The sequence:

1. `hwdef.h` defines `STM32_HSECLK` from `OSCILLATOR_HZ`
2. `stm32h7_mcuconf.h` has `#if STM32_HSECLK == 8000000U` blocks that set PLL dividers to achieve 400 MHz
3. PLL1P output → 400 MHz system clock
4. PLL2 → 200 MHz for QSPI, ADC clocks
5. PLL3Q → 48 MHz USB
6. `halconf.h` enables/disables ChibiOS subsystems based on `HAL_USE_SPI`, `HAL_USE_I2C`, `HAL_USE_SDC`, etc., which are set in `hwdef.h`

### Build System Board Selection

```
./waf configure --board MatekH743
```

This invokes WAF, which calls `add_dynamic_boards_from_hwdef_dir()` to auto-discover all boards by scanning `hwdef/*/hwdef.dat`. Each discovered directory becomes a named board class. The `chibios` WAF module calls `generate_hwdef_h(env)` during the configure phase, which runs the Python hwdef processor synchronously and writes `hwdef.h` into the build directory before compilation begins.

---

## 4. H7 Board Survey

### All H743/H755/H757/H7A3 Boards in hwdef Directory

**STM32H743** (34 boards found):

| Board | Notable Differences |
|---|---|
| MatekH743 | Reference board for Meridian. 8 MHz HSE, 400 MHz. SPI1+SPI4 IMUs, SPI2 OSD, SPI3 external. 4x I2C (2 used). 13 PWM outputs. SDMMC1 SD card. |
| MatekH743-bdshot | Identical to MatekH743 but enables bidirectional DShot on motor outputs |
| MatekH743-periph | AP_PERIPH firmware variant — GPS/compass/baro/RC outputs over CAN |
| BlitzH743Pro | 480 MHz clock override (`MCU_CLOCKRATE_MHZ 480`). Different bootloader flash reserve (384 KB vs 128 KB). OSD on SPI2. 9 UARTs. |
| KakuteH7v2 | Very similar peripheral set; different pin assignments |
| KakuteH7Mini | Smaller form factor, same MCU |
| NucleoH743 | Development board, minimal peripherals, AUTOBUILD_TARGETS None |
| NucleoH753ZI | H753 (AES crypto added), otherwise H743-compatible |
| BROTHERHOBBYH743 | FPV stack board |
| MambaH743v4 | Stack with integrated ESC telemetry wiring |
| SkystarsH7HD | HDMI/OSD variant |
| TMotorH743 | T-Motor branded board |

**STM32H755/H757** (3 boards):

| Board | Notable Differences |
|---|---|
| NucleoH755 | H755 is dual-core (CM7+CM4). Uses `MCU STM32H7xx STM32H757xx` (H755 and H757 share the py file). `define CORE_CM7` and `define SMPS_PWR` required. Bootloader reserve 256 KB (larger than H743). |
| H757I_EVAL | H757 dual-core eval board. 25 MHz HSE. No ADC. QSPI flash. `define SMPS_PWR`. Primarily CAN-based. |
| H757I_EVAL_intf | Companion interface board for H757 eval |

**STM32H7A3** (2 boards):

| Board | Notable Differences |
|---|---|
| MatekH7A3 | H7A3 uses a different flash structure: 256 pages × 8 KB (vs H743's 16 pages × 128 KB). `STORAGE_FLASH_PAGE 252`. `STM32_ST_USE_TIMER 5` with 32-bit resolution. 16 MHz HSE. External NAND flash via SPI2. OTG_HS (not OTG_FS). Single I2C3 bus. `AP_FLASH_STORAGE_DOUBLE_PAGE 1` needed for effective 16 KB parameter storage. |
| MatekH7A3-Wing | Wing variant of the above |

### H7-Specific Common Defines

The following are automatically applied to all H7 boards by the processing script:

```c
#define HAL_HAVE_HARDWARE_DOUBLE 1   // H7 has FPU with double precision
#define HAL_WITH_MCU_MONITORING 1    // H7 internal temp/voltage via ADC3
#define STM32H7 1
#define HAL_USE_HW_RNG TRUE          // H7 has hardware RNG
```

**H7 RAM Map** (from `STM32H743xx.py`):

| Region | Address | Size | DMA | Notes |
|---|---|---|---|---|
| SRAM1+SRAM2 | 0x30000000 | 256 KB | Yes | General DMA-capable RAM |
| DTCM | 0x20000000 | 128 KB | No | Tightly coupled, fast, stack/ISR use |
| AXI SRAM | 0x24000000 | 512 KB | Yes (SDMMC safe) | Largest region, main heap |
| ITCM | 0x00000400 | 63 KB | No | Instruction TCM (1 KB reserved at base) |
| SRAM3 | 0x30040000 | 32 KB | Yes | |
| SRAM4 | 0x38000000 | 64 KB | BDMA only | SPI6, I2C4, ADC3 via BDMA |

**Critical H7 DMA constraint**: DTCM and ITCM cannot be used for DMA. DMA buffers must be in AXI SRAM, SRAM1/2/3/4. The linker script generated from `RAM_MAP` handles this by placing stack/ISR data in DTCM and DMA-accessible data structures in AXI SRAM.

**H7 DMAMUX**: H743 uses DMAMUX1 (for DMA1/DMA2) and DMAMUX2 (for BDMA). Peripherals on BDMA include SPI6, I2C4, and ADC3. The `dma_resolver.py` script handles this distinction automatically via the `DMAMUX2_peripherals` list.

**H7 Timer quirk**: ChibiOS system tick timer must be a 32-bit timer. MatekH743 uses TIM12 with 16-bit resolution (TIM12 is technically 16-bit; `CH_CFG_ST_RESOLUTION 16` tells ChibiOS). H7A3 uses TIM5 (32-bit, `CH_CFG_ST_RESOLUTION 32`).

**H755/H757 differences**:
- Dual-core: CM7 runs the main autopilot; CM4 is currently unused by ArduPilot
- Must define `CORE_CM7` and `SMPS_PWR` (SMPS internal power regulator)
- Bootloader reserve must be 256 KB (due to larger flash sector size on H755)
- Uses `STM32H757xx.py` for both H755 and H757 (they are pin-compatible)
- Clock setup is slightly different due to SMPS domain

---

## 5. Peripheral Assignment Patterns

### SPI Device Assignment (SPIDEV)

Drivers call `hal.spi->get_device("mpu6000")` to open a device by name. The name string is matched against the SPIDEV table at runtime. The resolved device struct contains: SPI bus index, DEVID slot, CS pin PAL line, MODE, and speed limits.

The DEVID number (1–15) creates a unique 32-bit device ID bitmask per bus. This ID is stored as a parameter, so ArduPilot can detect sensor changes between boots and invalidate calibration data.

Multiple SPIDEV entries can share the same bus+DEVID+CS (as with the ICM42688/MPU6000 alternates on MatekH743 SPI1). The driver tries each name in priority order; only the first that responds is used.

### I2C Internal vs External

`I2C_ORDER I2C2 I2C1` — the first bus in I2C_ORDER gets logical index 0, the second gets index 1.

`HAL_I2C_INTERNAL_MASK` is a bitmask of which logical bus indices are "internal" (on-board). A 0 value means all buses are treated as external. This affects compass probing: internal buses are probed for known internal compasses first; external buses are probed for all types.

Example from fmuv3: `I2C_ORDER I2C2 I2C1` with I2C2 as internal (index 0, `HAL_I2C_INTERNAL_MASK 1`) means I2C2 is on-board and I2C1 is the external port.

### Serial Port Numbering

```
SERIAL_ORDER OTG1 UART7 USART1 USART2 USART3 UART8 UART4 USART6 OTG2
```

- Index 0 → `hal.serial(0)` → SERIAL0_ parameters → always the primary console (USB preferred)
- Index 1 → `hal.serial(1)` → SERIAL1_ → telem1
- Index 3 → SERIAL3_ → primary GPS
- `EMPTY` placeholder → creates an empty/null driver at that index
- The UART hardware name (UART4, USART6, etc.) is STM32's naming, not ArduPilot's serial number

The IOMCU UART (if present) is appended after the user-visible serial list but hidden from SERIAL parameter numbering if `hide_iomcu_uart` is true.

### AP_PERIPH Firmware Differences

AP_PERIPH is a variant firmware that runs on a CAN node co-processor (like `MatekH743-periph`). It:

1. Set via `env AP_PERIPH 1` in the hwdef
2. Strips out most vehicle logic; replaces with CAN DroneCAN node code
3. Undefines `SDMMC1`, `OSD_ENABLED`, `HAL_OS_FATFS_IO`, most IMU drivers
4. Adds `AP_PERIPH_GPS_ENABLED`, `AP_PERIPH_MAG_ENABLED`, `AP_PERIPH_BARO_ENABLED`, etc.
5. Uses the same base hwdef `include` mechanism — `include ../MatekH743/hwdef.dat` then `undef` the flight-controller-specific items
6. Board ID is typically the same as the main firmware board to allow the same bootloader

### BDSHOT Variant Pattern

`MatekH743-bdshot` is identical to `MatekH743` but adds bidirectional DShot capability. This is handled by the build system (`bdshot_encoder.py` in the scripts directory) and typically adds `BIDIR` to motor timer pins.

---

## 6. What Meridian Needs

### The Equivalent of hwdef for RTIC/STM32H743

RTIC (Real-Time Interrupt-driven Concurrency) on STM32H743 via the `stm32h7xx-hal` crate requires the following information to be captured:

#### 1. Clock Configuration

The hwdef `OSCILLATOR_HZ` and MCU type translate directly to PAC/HAL clock setup. For 8 MHz HSE → 400 MHz sys_ck on H743:

```
HSE = 8 MHz
PLL1: M=1, N=100, P=2 → 400 MHz (sys_ck)
PLL1: Q=10           → 40 MHz (FDCAN)
PLL2: M=1, N=75, P=3 → 200 MHz (QSPI)
PLL2: Q=6, R=3       → 100 MHz (SPI45), 200 MHz (ADC)
PLL3: M=2, N=72, Q=6 → 48 MHz (USB)
APB dividers: D1HPRE/2, D1/2/3PPRE/2 → peripheral buses at 200 MHz
```

All of this is implicit in ArduPilot's `stm32h7_mcuconf.h`. In Meridian, this needs to be in a board-specific `clock_config.rs` or embedded in the RTIC `#[init]` task.

#### 2. Pin Mux / Alternate Function Table

Every `PA5 SPI1_SCK SPI1` line needs to become a Rust GPIO configuration call:

```rust
// hwdef: PA5 SPI1_SCK SPI1
let sck = gpioa.pa5.into_alternate::<5>(); // AF5 = SPI1 on H743
```

The AF number comes from `STM32H743xx.py`'s `AltFunction_map`. For Meridian, this table needs to be replicated in a lookup or hard-coded per board.

#### 3. SPI Bus Instances

Each `SPI1`–`SPI4` entry with its speed bounds needs to become an `stm32h7xx_hal::spi::Spi` instance. Speed is constrained by `N*MHZ` values in SPIDEV — the HAL rounds to the nearest valid prescaler.

Key detail: SPI1–SPI5 clock source is PCLK2 (100 MHz on H743). SPI6 is on PCLK4 (100 MHz). The actual achievable SPI clock for each SPIDEV entry is `PCLK / prescaler`. E.g., for `16*MHZ` on a 100 MHz SPI bus, prescaler = 8 → actual = 12.5 MHz.

#### 4. DMA Channel Assignment

The DMA resolver solves a constraint satisfaction problem. For Meridian, the key constraints for MatekH743 are:

- **SPI1 (IMU1) and SPI4 (IMU2)**: `DMA_NOSHARE` — each must have dedicated DMA streams. No sharing with any other peripheral.
- **H743 DMAMUX1** covers DMA1 (8 streams) and DMA2 (8 streams), 16 streams total
- Recommended assignment for IMU-critical buses: SPI1 gets DMA1_Stream0+1, SPI4 gets DMA2_Stream0+1
- UART RX/TX can share streams (they are in `SHARED_MAP` by default)

In RTIC, DMA is configured per-peripheral. The `stm32h7xx-hal` provides DMA abstractions via `dma` module. Each IMU SPI transfer should use a dedicated DMA stream to avoid jitter.

#### 5. RAM Regions

For correct DMA operation, SPI/UART DMA buffers must be placed in AXI SRAM or SRAM1/2 (not DTCM). In Rust, this requires either:
- `#[link_section = ".axisram.buffers"]` attribute on DMA buffer statics
- A custom linker script that places designated buffers in DMA-capable RAM

DTCM (0x20000000) is the default stack location and fastest for CPU access but cannot be used for DMA. The linker script from hwdef already handles this — Meridian needs an equivalent custom `memory.x`.

**Linker memory regions for MatekH743**:
```
MEMORY {
  FLASH   (rx)  : ORIGIN = 0x08020000, LENGTH = 1920K  /* starts at 128K offset */
  DTCM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K   /* no DMA */
  SRAM1   (rwx) : ORIGIN = 0x30000000, LENGTH = 256K   /* DMA OK */
  AXI     (rwx) : ORIGIN = 0x24000000, LENGTH = 512K   /* DMA OK, SDMMC OK */
  ITCM    (rx)  : ORIGIN = 0x00000400, LENGTH = 63K    /* no DMA */
  SRAM3   (rwx) : ORIGIN = 0x30040000, LENGTH = 32K    /* DMA OK */
  SRAM4   (rwx) : ORIGIN = 0x38000000, LENGTH = 64K    /* BDMA only */
}
```

The firmware starts at `ORIGIN = 0x08020000` (flash base + 128 KB bootloader reserve).

#### 6. Multi-Board Support Strategy

ArduPilot handles multi-board support by having one hwdef.dat per board — the entire board identity is in that single file plus the included common files. For Meridian, the recommended approach is:

1. **Feature flags via Cargo features**: One feature per supported board (`matek-h743`, `kakute-h7v2`, etc.)
2. **Board module**: Each board gets a module (e.g., `src/boards/matek_h743.rs`) that exports:
   - `ClockConfig` struct with PLL values
   - `PinConfig` struct/trait implementation with GPIO assignments
   - DMA channel assignments as constants
   - SPIDEV entries as a static array
   - ADC channel mappings
3. **Common HAL layer**: The rest of the codebase uses only abstract types (`impl SpiBus`, `impl UartDevice`, `impl AdcPin`) — board specifics are injected at `main()` or the RTIC `#[init]` task via generics or type aliases
4. **Rotation constants**: IMU rotations need to be board-defined constants passed to the sensor fusion layer

#### 7. Critical Items to Capture for MatekH743

The minimum viable board configuration for Meridian Phase 9:

| Item | ArduPilot Source | Meridian Target |
|---|---|---|
| 8 MHz HSE, 400 MHz sys_ck | `OSCILLATOR_HZ 8000000` + mcuconf.h | `ClockConfig` with PLL1 N=100, P=2 |
| SPI1 pins (PA5/PA6/PD7) + AF5/AF5/AF5 | Pin lines + H743 AF table | `Spi1Pins` type with correct AF |
| SPI4 pins (PE12/PE13/PE14) + AF5 | Pin lines + H743 AF table | `Spi4Pins` type |
| IMU1 CS = PC15, IMU2 CS = PE11 | CS pin lines | GPIO outputs in board init |
| ICM42688 on SPI1, MODE3, up to 16 MHz | SPIDEV line | `SpiDevConfig` for IMU1 |
| IMU rotation: YAW_180 | IMU line | `ROTATION_YAW_180` constant in sensor fusion |
| UART7 = telem (PE7/PE8/PE9/PE10) | SERIAL_ORDER + pin lines | UART7 configured at 115200 (or param-driven) |
| USART2 = GPS (PD5/PD6) | SERIAL_ORDER + pin lines | UART for GPS parser |
| I2C2 = baro bus (PB10/PB11) | I2C_ORDER + pin lines | I2C2 master for DPS310/MS5611 |
| SPI1+SPI4 DMA_NOSHARE | DMA_NOSHARE line | Dedicated DMA1 Stream0+1 for SPI1, DMA2 Stream0+1 for SPI4 |
| 13 PWM outputs (TIM1/4/5/8/15) | PWM lines | Timer configurations with correct AF |
| SDMMC1 (PC8-PC12, PD2) | SDMMC pin lines | SDMMC1 peripheral with 4-bit SDIO |
| Flash start offset = 0x08020000 | FLASH_RESERVE_START_KB 128 | `memory.x` FLASH origin |
| DMA buffers in AXI SRAM | RAM_MAP in STM32H743xx.py | `#[link_section]` or linker script regions |
| CAN1 (PD0/PD1) | CAN pin lines | FDCAN1 peripheral + transceiver silent pin PD3 |

---

## Files Audited

1. `libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat` — complete line-by-line
2. `libraries/AP_HAL_ChibiOS/hwdef/MatekH743/hwdef-bl.dat` — bootloader variant
3. `libraries/AP_HAL_ChibiOS/hwdef/fmuv3/hwdef.dat` — Pixhawk1 reference (heavily documented)
4. `libraries/AP_HAL_ChibiOS/hwdef/BlitzH743Pro/hwdef.dat` — second H743 board
5. `libraries/AP_HAL_ChibiOS/hwdef/MatekH743-periph/hwdef.dat` — AP_PERIPH variant
6. `libraries/AP_HAL_ChibiOS/hwdef/NucleoH743/hwdef.dat` — minimal H743 dev board
7. `libraries/AP_HAL_ChibiOS/hwdef/NucleoH755/hwdef.dat` — H755 dual-core
8. `libraries/AP_HAL_ChibiOS/hwdef/H757I_EVAL/hwdef.dat` — H757 eval
9. `libraries/AP_HAL_ChibiOS/hwdef/MatekH7A3/hwdef.dat` + `hwdef.inc` — H7A3 variant
10. `libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py` — main processing script (3169 lines)
11. `libraries/AP_HAL_ChibiOS/hwdef/scripts/dma_resolver.py` — DMA constraint solver
12. `libraries/AP_HAL_ChibiOS/hwdef/scripts/STM32H743xx.py` — H743 MCU database (pin AF table, RAM map, clock expectations)
13. `libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h` — H7 PLL/clock configuration (733 lines)
14. `libraries/AP_HAL_ChibiOS/hwdef/common/halconf.h` — ChibiOS HAL subsystem enable/disable
15. `libraries/AP_HAL_ChibiOS/hwdef/common/board.h` — minimal board init hook header
16. `libraries/AP_HAL/hwdef/scripts/hwdef.py` — base hwdef directive parser
17. `Tools/ardupilotwaf/boards.py` — WAF build system board discovery and chibios class
18. `Tools/ardupilotwaf/chibios.py` — WAF ChibiOS configure/build, `generate_hwdef_h()`
19. `Tools/AP_Bootloader/board_types.txt` — board ID registry (APJ_BOARD_ID resolution)
