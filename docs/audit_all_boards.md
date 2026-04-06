# ArduPilot hwdef Board Support Matrix — Complete Inventory
**Scope**: All 430 board directories in `libraries/AP_HAL_ChibiOS/hwdef/`
**Date**: 2026-04-02
**Auditor**: Research-only pass; no Rust code written

---

## Table of Contents
1. [hwdef System Overview](#1-hwdef-system-overview)
2. [MCU Family Summary](#2-mcu-family-summary)
3. [Complete Board Inventory — All 430 Boards](#3-complete-board-inventory)
4. [Flight Controllers by MCU Family](#4-flight-controllers-by-mcu-family)
5. [AP_PERIPH Peripheral Nodes](#5-ap_periph-peripheral-nodes)
6. [Special and Infrastructure Boards](#6-special-and-infrastructure-boards)
7. [Sensor Population Statistics](#7-sensor-population-statistics)
8. [Pin Mapping Patterns](#8-pin-mapping-patterns)
9. [Minimum Viable MCU Analysis for Meridian](#9-minimum-viable-mcu-analysis)
10. [Meridian Multi-Board Abstraction Design](#10-meridian-multi-board-abstraction-design)

---

## 1. hwdef System Overview

### What hwdef.dat Is

Every board has one directory under `libraries/AP_HAL_ChibiOS/hwdef/<BoardName>/`. Inside lives a `hwdef.dat` file written in a custom DSL. The script `libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py` reads it and emits:
- `hwdef.h` — C preprocessor header with all board defines
- `ldscript.ld` and `common.ld` — linker scripts derived from the MCU's RAM/flash map

A companion `hwdef-bl.dat` covers the bootloader build (stripped-down peripherals, smaller flash footprint).

### Inheritance

Boards inherit via include directives:
```
include ../OtherBoard/hwdef.dat
include ../CommonBase/hwdef.inc
```

There are **270 base boards** (have their own `MCU` directive) and **149 derived boards** (include-only, override a few lines). The derived boards are almost exclusively:
- `*-bdshot` — bidirectional DSHOT variant (same hardware, different PWM driver init)
- `*-ODID` — Open Drone ID variant (adds cellular modem support)
- `*-periph` — same MCU repurposed as a DroneCAN peripheral node
- `*-SimOnHardWare` — adds software-in-the-loop simulation shim
- `*-PPPGW` — PPP Ethernet gateway variant

### Key Directives

| Directive | Example | Meaning |
|-----------|---------|---------|
| `MCU STM32H7xx STM32H743xx` | MCU class + exact variant | Selects pin/DMA tables from `scripts/STM32H743xx.py` |
| `FLASH_SIZE_KB 2048` | 2048 | Usable flash in KB |
| `OSCILLATOR_HZ 8000000` | 8 MHz | External crystal (most boards: 8 or 16 MHz) |
| `SERIAL_ORDER OTG1 UART7...` | — | Maps ArduPilot serial ports 0..N to hardware UARTs |
| `IOMCU_UART USARTx` | USART6 | Designates UART connected to the IO MCU (STM32F1/F3 coprocessor) |
| `SPIDEV name SPIx DEVID CS_pin MODE freq` | — | Names a device on a SPI bus |
| `IMU Driver SPI:name ROTATION_...` | — | Declares an inertial sensor |
| `BARO Driver I2C:bus:addr` | — | Declares a barometer |
| `COMPASS Driver SPI/I2C:... external ROTATION` | — | Declares a compass |
| `AP_PERIPH` | (no args) | Marks board as a DroneCAN peripheral (no vehicle firmware) |
| `env AP_PERIPH 1` | — | Alternative peripheral marker (in `env` namespace) |
| `STORAGE_FLASH_PAGE N` | 14 | Use flash page N for parameter storage (no SD card) |
| `define HAL_STORAGE_SIZE 32768` | — | Size of EEPROM-equivalent storage |
| `ROMFS file.bin path` | — | Embed a binary (e.g. IO firmware) in the flash |

### MCU-Specific Tables

Each `scripts/STM32xxxx.py` file provides:
- `RAM_MAP` — list of `(base_addr, size_kb, priority)` tuples for all SRAM regions
- `ALT_RAM_MAP` — alternate layout (for PX4 bootloader compatibility on some boards)
- `AltFunction_map` — every GPIO pin → alternate function number
- `DMA1_map`, `DMA2_map` — every peripheral → available DMA streams
- `EXPECTED_CLOCK` — CPU frequency in Hz

---

## 2. MCU Family Summary

### Board Count by MCU Chip

| MCU Chip | Count (boards with own MCU line) | CPU MHz | Flash KB | Total RAM |
|----------|----------------------------------|---------|----------|-----------|
| **STM32H743xx** | **93** | 400 | 2048 | 1055 KB usable (DTCM128 + AXI512 + SRAM1/2 256 + SRAM3 32 + SRAM4 64) |
| **STM32F405xx** | **63** | 168 | 1024 | 192 KB (SRAM 128 + CCM 64) |
| **STM32F427xx** | **16** | 168 | 2048 | 256 KB |
| STM32F412Rx | 13 | 100 | 512–1024 | 256 KB |
| STM32H757xx | 10 | 400 | 2048 | 1 MB |
| STM32F767xx | 10 | 216 | 2048 | 512 KB |
| STM32F745xx | 10 | 216 | 1024 | 512 KB |
| STM32L431xx | 8 | 80 | 256 | 64 KB |
| STM32F407xx | 8 | 168 | 1024 | 192 KB |
| STM32G474xx | 7 | 160 | 512 | 128 KB |
| STM32H750xx | 4 | 400 | 128 (XIP from external flash) | ~1 MB |
| STM32G491xx | 4 | 160 | 256–512 | 96 KB |
| STM32F103xB | 4 | 72 | 64–128 | 20 KB |
| STM32F777xx | 3 | 216 | 2048 | 512 KB |
| STM32F303xC | 2 | 72 | 256 | 40 KB |
| STM32F100xB | 2 | 24 | 64 | 8 KB |
| CKS32F407xx | 2 | 168 | 1024 | 192 KB |
| STM32L4R5xx | 1 | 120 | 2048 | 640 KB |
| STM32L496xx | 1 | 80 | 1024 | 320 KB |
| STM32L476xx | 1 | 80 | 1024 | 128 KB |
| STM32H755xx | 1 | 400 | 2048 | 1 MB |
| STM32H753xx | 1 | 400 | 2048 | 1 MB |
| STM32H730xx | 1 | 520 | 128 (XIP) | ~1 MB |
| STM32H723xx | 1 | 400 | 1024 | 564 KB |
| STM32H7A3xx | 1 | 280 | 2048 | 1280 KB |
| STM32G441xx | 1 | 170 | 128 | 32 KB |
| STM32F732xx | 1 | 216 | 512 | 256 KB |
| STM32F469xx | 1 | 180 | 2048 | 384 KB |
| STM32F105xC | 1 | 72 | 256 | 64 KB |

**Note**: 149 boards have no own MCU line — they inherit from the base board via `include`.

### Active vs. Legacy Families

**Actively used (dominant in new designs 2023+)**:
- STM32H743xx — the clear primary target; 93 boards
- STM32F405xx — still dominant in budget FPV hardware

**Mature/maintained (no new designs being added)**:
- STM32F427xx, STM32F745xx, STM32F767xx, STM32H757xx

**Legacy (no new boards, support only)**:
- STM32F407xx, STM32F777xx, STM32F427xx (partly), STM32F103xB (IOMCU only), STM32F100xB (IOMCU only)

**Niche/special purpose**:
- STM32L431xx — DroneCAN periph nodes (low-power, small)
- STM32G474xx / STM32G491xx — newer CAN periph (hardware FDCAN)
- STM32H750xx, STM32H730xx — "external flash" boards (tiny internal flash, XIP from QSPI)
- CKS32F407xx — Chinese STM32 clone (QioTek boards)

### Crystal Frequency Distribution

| Crystal Hz | Board count |
|------------|-------------|
| 8 MHz | 141 |
| 16 MHz | 67 |
| 24 MHz | 48 |
| 25 MHz | 5 |

---

## 3. Complete Board Inventory

All 430 board directory names, organized by type. Boards marked (D) are derived (include-only).

### 3.1 All Board Names Alphabetically

```
3DR-L431             3DR-L431-ASAUAV (D)    3DRControlZeroG
ACNS-CM4Pilot        ACNS-F405AIO           AEDROXH7
AEROFOX-H7           AET-H743-Basic         AIRBRAINH743
AIRLink              AR-F407SmartBat        ARKV6X
ARKV6X-bdshot (D)    ARK_CANNODE            ARK_FPV (D)
ARK_GPS              ARK_PI6X (D)           ARK_RTK_GPS
ATOMRCF405NAVI-Deluxe AcctonGodwit_GA1      AeroCogito-H7Digital
AeroFox-Airspeed (D) AeroFox-Airspeed-DLVR (D) AeroFox-GNSS_F9P
AeroFox-PMU (D)      Aeromind6X             Airvolute-DCS2
AnyleafH7 (D)        Aocoda-RC-H743Dual     Atlas-Control
AtomRCF405NAVI       BETAFPV-F405           BETAFPV-F405-I2C (D)
BOTWINGF405          BROTHERHOBBYF405v3     BROTHERHOBBYH743
BeastF7              BeastF7v2 (D)          BeastH7
BeastH7v2 (D)        BirdCANdy              BlitzF745
BlitzF745AIO         BlitzH743Pro           BlitzMiniF745 (D)
BlitzWingH743 (D)    BotBloxDroneNet        BrahmaF4
C-RTK2-HP            CBU-H7-LC-Stamp        CBU-H7-Stamp
CORVON405V2_1        CORVON743V1            CSKY405
CSKY_PMU             CUAV-7-Nano            CUAV-7-Nano-ODID (D)
CUAV-Nora            CUAV-Nora-ODID (D)     CUAV-Nora-bdshot (D)
CUAV-Pixhack-v3 (D)  CUAV-V6X-v2            CUAV-V6X-v2-ODID (D)
CUAV-X25-EVO         CUAV-X25-EVO-ODID (D)  CUAV-X7
CUAV-X7-ODID (D)     CUAV-X7-bdshot (D)     CUAV_GPS
CUAVv5 (D)           CUAVv5-bdshot (D)      CUAVv5Nano (D)
CUAVv5Nano-bdshot (D) CarbonixF405          CrazyF405
CubeBlack (D)        CubeBlack+ (D)         CubeBlack-periph (D)
CubeGreen-solo (D)   CubeNode               CubeNode-ETH (D)
CubeOrange           CubeOrange-ODID (D)    CubeOrange-SimOnHardWare (D)
CubeOrange-bdshot (D) CubeOrange-joey (D)   CubeOrange-periph (D)
CubeOrange-periph-heavy (D) CubeOrangePlus  CubeOrangePlus-ODID (D)
CubeOrangePlus-SimOnHardWare (D) CubeOrangePlus-bdshot (D) CubePurple (D)
CubeRedPrimary       CubeRedPrimary-PPPGW (D) CubeRedSecondary
CubeRedSecondary-IO  CubeSolo (D)           CubeYellow
CubeYellow-bdshot (D) DAKEFPVF405           DAKEFPVH743 (D)
DAKEFPVH743Pro       DAKEFPVH743_SLIM       DevEBoxH7v2
DroneerF405          DrotekP3Pro            Durandal
Durandal-bdshot (D)  F35Lightning           F4BY
F4BY_F427            F4BY_H743              FlyingMoonF407
FlyingMoonF427       FlyingMoonH743         FlysparkF4
FlywooF405HD-AIOv2 (D) FlywooF405Pro        FlywooF405S-AIO
FlywooF745           FlywooF745Nano (D)     FlywooH743Pro
FoxeerF405v2         FoxeerH743v1           FreeflyRTK
G4-ESC               GEPRCF745BTHD          GEPRC_TAKER_H743
GreenSightUltraBlue  H757I_EVAL             H757I_EVAL_intf
HEEWING-F405         HEEWING-F405v2 (D)     HWH7
Here4AP              Here4FC                Hitec-Airspeed
HitecMosaic          HolybroF4_PMU          HolybroG4_Airspeed
HolybroG4_Compass    HolybroG4_GPS          HolybroGPS
IFLIGHT_2RAW_H7      JFB100                 JFB110
JHEMCU-GSF405A       JHEMCU-GSF405A-RX2 (D) JHEMCU-H743HD
JHEMCUF405PRO        JHEMCUF405WING         JHEM_JHEF405
JPilot-C             KT-FMU-F1              KakuteF4
KakuteF4-Wing        KakuteF4Mini (D)       KakuteF7
KakuteF7-bdshot (D)  KakuteF7Mini           KakuteH7
KakuteH7-Wing        KakuteH7-bdshot (D)    KakuteH7Mini
KakuteH7Mini-Nand (D) KakuteH7v2 (D)       LongBowF405WING
LumenierLUXF765-NDAA MFE_AirSpeed_CAN (D)  MFE_PDB_CAN
MFE_POS3_CAN         MFT-SEMA100            MUPilot (D)
MambaF405-2022 (D)   MambaF405US-I2C        MambaF405v2
MambaH743v4          MatekF405              MatekF405-CAN
MatekF405-STD (D)    MatekF405-TE           MatekF405-TE-bdshot (D)
MatekF405-Wing       MatekF405-Wing-bdshot (D) MatekF405-bdshot (D)
MatekF765-SE (D)     MatekF765-Wing         MatekF765-Wing-bdshot (D)
MatekG474            MatekG474-DShot (D)    MatekG474-GPS (D)
MatekG474-Periph (D) MatekH743              MatekH743-bdshot (D)
MatekH743-periph (D) MatekH7A3              MatekH7A3-Wing (D)
MatekL431            MatekL431-ADSB (D)     MatekL431-APDTelem (D)
MatekL431-AUAV (D)   MatekL431-Airspeed (D) MatekL431-BattMon (D)
MatekL431-BatteryTag (D) MatekL431-DShot (D) MatekL431-EFI (D)
MatekL431-GPS (D)    MatekL431-HWTelem (D)  MatekL431-MagHiRes (D)
MatekL431-Periph (D) MatekL431-Proximity (D) MatekL431-RC (D)
MatekL431-Rangefinder (D) MatekL431-Serial (D) MatekL431-bdshot (D)
MazzyStarDrone       MicoAir405Mini         MicoAir405v2
MicoAir743           MicoAir743-AIO         MicoAir743-Lite
MicoAir743v2         Morakot                NarinFC-H5 (D)
NarinFC-H7           NarinFC-X3             Nucleo-G491
Nucleo-L476          Nucleo-L496            NucleoH743
NucleoH753ZI         NucleoH755             NxtPX4v2
OMNIBUSF7V2          ORBITH743              OmnibusNanoV6
OmnibusNanoV6-bdshot (D) OrqaF405Pro        OrqaH7QuadCore
PH4-mini (D)         PH4-mini-bdshot (D)    Pix32v5 (D)
PixC4-Jetson         PixFlamingo            PixFlamingo-F767
PixPilot-C3          PixPilot-V3            PixPilot-V6
PixPilot-V6PRO       PixSurveyA1 (D)        PixSurveyA1-IND
PixSurveyA2-IND      Pixhawk1 (D)           Pixhawk1-1M (D)
Pixhawk1-1M-bdshot (D) Pixhawk1-bdshot (D)  Pixhawk4 (D)
Pixhawk4-bdshot (D)  Pixhawk5X              Pixhawk6C
Pixhawk6C-bdshot (D) Pixhawk6X              Pixhawk6X-ODID (D)
Pixhawk6X-PPPGW (D)  Pixhawk6X-bdshot (D)   Pixracer
Pixracer-bdshot (D)  Pixracer-periph        QioTekAdeptF407
QioTekZealotF427     QioTekZealotH743       QioTekZealotH743-bdshot (D)
R9Pilot              RADIX2HD               RadiolinkF405
RadiolinkPIX6        ReaperF745             ResoluteH7
SDMODELH7V1 (D)      SDMODELH7V2 (D)        SIYI_N7
SPEDIXF405           SPEDIXH743             SPRacingH7
SPRacingH7RF         SULILGH7-P1-P2         SVehicle-E2
SequreH743           Sierra-F405            Sierra-F412
Sierra-F9P           Sierra-L431            Sierra-PrecisionPoint
Sierra-TrueNavIC     Sierra-TrueNavPro      Sierra-TrueNavPro-G4
Sierra-TrueNorth     Sierra-TrueSpeed       SkyRukh_Surge_H7
SkySakuraH743        SkystarsF405v2         SkystarsH7HD
SkystarsH7HD-bdshot (D) SkystarsH7HDv2 (D)  SpeedyBeeF405AIO
SpeedyBeeF405Mini    SpeedyBeeF405WING      StellarF4
StellarF4V2          StellarH7V2            SuccexF4
Swan-K1 (D)          TBS-Colibri-F7 (D)     TBS-L431
TBS-L431-Airspeed (D) TBS-L431-BattMon (D)  TBS-L431-CurrMon (D)
TBS-L431-PWM (D)     TBS_LUCID_H7           TBS_LUCID_H7_WING (D)
TBS_LUCID_H7_WING_AIO (D) TBS_LUCID_PRO     TMotorH743
VM-L431              VM-L431-BMS (D)        VM-L431-BatteryTag (D)
VM-L431-Periph-Pico (D) VM-L431-SRV-Hub-4CHP (D) VRBrain-v51
VRBrain-v52          VRBrain-v54            VRCore-v10
VRUBrain-v51         VUAV-TinyV7            VUAV-V7pro
X-MAV-AP-H743r1      X-MAV-AP-H743v2        YARIV6X
YJUAV_A6             YJUAV_A6SE             YJUAV_A6SE_H743
YJUAV_A6Ultra        ZeroOneX6              ZeroOneX6_Air
ZubaxGNSS            airbotf4               crazyflie2
f103-ADSB (D)        f103-Airspeed (D)      f103-GPS (D)
f103-HWESC (D)       f103-QiotekPeriph (D)  f103-RangeFinder (D)
f103-Trigger (D)     f303-GPS (D)           f303-HWESC (D)
f303-M10025 (D)      f303-M10070 (D)        f303-MatekGPS
f303-PWM (D)         f303-TempSensor (D)    f303-Universal (D)
f405-MatekAirspeed (D) f405-MatekGPS        fmuv2 (D)
fmuv3                fmuv3-bdshot           fmuv5
iomcu                iomcu-dshot (D)        iomcu-f103
iomcu-f103-8MHz-dshot (D) iomcu-f103-dshot (D) iomcu_f103_8MHz (D)
kha_eth              luminousbee4           luminousbee5
mRo-M10095           mRoCANPWM-M10126       mRoCZeroOEMH7-bdshot (D)
mRoControlZeroClassic mRoControlZeroF7     mRoControlZeroH7
mRoControlZeroH7-bdshot (D) mRoControlZeroOEMH7 mRoKitCANrevC (D)
mRoNexus             mRoPixracerPro         mRoPixracerPro-bdshot (D)
mRoX21 (D)           mRoX21-777             mindpx-v2
mini-pix             modalai_fc-v1          omnibusf4 (D)
omnibusf4pro         omnibusf4pro-bdshot (D) omnibusf4pro-one (D)
omnibusf4v6          rFCU                   rGNSS
revo-mini            revo-mini-bdshot (D)   revo-mini-i2c
revo-mini-i2c-bdshot (D) revo-mini-sd (D)  skyviper-f412-rev1
skyviper-journey     skyviper-v2450 (D)     sparky2
speedybeef4          speedybeef4v3          speedybeef4v4
speedybeef4v5        sw-boom-f407           sw-nav-f405
sw-spar-f407         thepeach-k1            thepeach-r1
uav-dev-auav-g4      uav-dev-fc-um982       uav-dev-powermodule
uav-dev_m10s
```

**Total directories**: 430 (includes `common/`, `scripts/`, `include/` infrastructure; ~415 are actual board entries)

---

## 4. Flight Controllers by MCU Family

### 4.1 STM32H743xx Flight Controllers (93 unique boards)

The dominant family. All have 2048 KB flash, 400 MHz, and use ChibiOS RTOS. Most have SDMMC for logging. Crystal is typically 8 or 16 MHz.

| Board | Flash KB | Crystal Hz | IMUs | Baros | Compass | SPI buses | I2C buses | PWM max | SD | FRAM | Eth | IOMCU |
|-------|----------|------------|------|-------|---------|-----------|-----------|---------|----|----|-----|-------|
| 3DRControlZeroG | 2048 | 24M | BMI088, Invensense, Invensensev2 | DPS310 | AK09916 | 3 | 3 | 0 | yes | yes | no | no |
| AEDROXH7 | 2048 | 8M | Invensensev3 | DPS310 | — | 2 | 2 | 0 | no | no | no | no |
| AEROFOX-H7 | 2048 | 24M | ADIS1647x, Invensensev3 | SPL06, BMP280 | QMC5883L | 3 | 2 | 0 | no | yes | no | no |
| AET-H743-Basic | 2048 | 8M | Invensensev3 ×2 | SPL06, DPS310 | — | 3 | 2 | 0 | yes | no | no | no |
| AIRBRAINH743 | 2048 | 8M | Invensensev3 | DPS310 | LIS2MDL | 2 | 1 | 0 | no | no | no | no |
| AcctonGodwit_GA1 | 2048 | 16M | Invensensev3, BMI088 | ICP201XX ×2 | RM3100 | 4 | 4 | 0 | yes | yes | yes | no |
| AeroCogito-H7Digital | 2048 | 16M | Invensensev3 | DPS310 | — | 1 | 2 | 0 | yes | no | no | no |
| Aeromind6X | 2048 | 16M | Invensensev3 | ICP201XX ×2 | RM3100 | 4 | 4 | 0 | yes | yes | yes | no |
| Airvolute-DCS2 | 2048 | 16M | BMI088, Invensense | BMP388 | — | 2 | 1 | 0 | yes | no | yes | no |
| Aocoda-RC-H743Dual | 2048 | 8M | Invensense, BMI270 | MS5611, DPS310, BMP280 | — | 4 | 2 | 0 | no | no | no | no |
| Atlas-Control | 2048 | 16M | ADIS1647x, Invensensev3, Invensense | MS5611 ×2 | RM3100 | 4 | 4 | 0 | yes | yes | no | no |
| BROTHERHOBBYH743 | 2048 | 8M | Invensensev3, BMI088 | SPL06 | — | 3 | 2 | 0 | yes | no | no | no |
| BeastH7 | 2048 | 8M | Invensense | DPS310 | — | 3 | 1 | 0 | no | no | no | no |
| BlitzH743Pro | 2048 | 8M | Invensensev3 | DPS310, SPL06 | — | 2 | 2 | 0 | yes | no | no | no |
| CBU-H7-LC-Stamp | 2048 | 16M | Invensensev3 | BMP280 | — | 1 | 3 | 0 | yes | no | no | no |
| CBU-H7-Stamp | 2048 | 16M | Invensensev3 | BMP280 | BMM150 | 1 | 4 | 0 | yes | no | yes | no |
| CORVON743V1 | 2048 | 8M | BMI088, BMI270 | DPS310 | IST8310 ×2 | 2 | 2 | 0 | yes | no | no | no |
| CUAV-7-Nano | 2048 | 16M | Invensensev3, BMI088 | BMP581, ICP201XX | IST8310, IIS2MDC | 4 | 4 | 0 | yes | yes | yes | no |
| CUAV-Nora | 2048 | 16M | Invensense, BMI088, Invensensev3, Invensensev2 | MS5611 ×2 | IST8310, RM3100 | 4 | 4 | 0 | yes | yes | no | no |
| CUAV-V6X-v2 | 2048 | 16M | Invensensev3, BMI088 | BMP581, ICP201XX | RM3100 | 4 | 4 | 0 | yes | yes | yes | no |
| CUAV-X25-EVO | 2048 | 16M | Invensensev3 ×3 | BMP581, ICP201XX | RM3100 | 4 | 4 | 0 | yes | yes | yes | no |
| CUAV-X7 | 2048 | 16M | ADIS1647x, Invensensev3, Invensense, BMI088, Invensensev2 | MS5611 ×2 | IST8310, RM3100 | 4 | 4 | 0 | yes | yes | no | no |
| CubeOrange | 2048 | 24M | (from hwdef.inc) | (from hwdef.inc) | (from hwdef.inc) | — | 2 | 0 | no | no | no | yes |
| CubeOrangePlus | (from hwdef.inc) | — | Invensensev3, Invensensev2 ×7 | — | AK09916 | 2 | 1 | 0 | no | no | no | — |
| CubeRedPrimary | 2048 | 24M | Invensensev3, Invensensev2 | MS5611 ×2 | RM3100 | 3 | 2 | 0 | yes | no | yes | yes |
| CubeRedSecondary | 2048 | 24M | Invensensev3 | MS5611 | — | 2 | 0 | 0 | no | no | yes | no |
| DAKEFPVH743Pro | 2048 | 8M | Invensensev3 ×2 | SPL06 | — | 4 | 1 | 0 | no | no | no | no |
| DAKEFPVH743_SLIM | 2048 | 8M | Invensensev3 ×2 | SPL06 | — | 4 | 1 | 0 | yes | no | no | no |
| Durandal | 2048 | 16M | Invensense, BMI088, BMI055 | MS5611 | IST8310 ×2 | 5 | 4 | 0 | yes | yes | no | no |
| F4BY_H743 | 2048 | 8M | Invensense, Invensensev3 | MS5611 | HMC5843, QMC5883L, QMC5883P | 3 | 2 | 0 | no | yes | no | no |
| FlyingMoonH743 | 2048 | 8M | Invensense, Invensensev2, Invensensev3 | SPL06, MS5611 | RM3100 | 2 | 2 | 0 | no | yes | no | no |
| FlywooH743Pro | 2048 | 8M | Invensensev3 ×2 | SPL06 | — | 3 | 2 | 0 | yes | no | no | no |
| FoxeerH743v1 | 2048 | 8M | Invensense, Invensensev3 | DPS310 | — | 3 | 1 | 0 | no | no | no | no |
| GEPRC_TAKER_H743 | 2048 | 8M | Invensensev3, Invensense | SPL06 | — | 4 | 1 | 0 | no | no | no | no |
| GreenSightUltraBlue | 2048 | 16M | BMI088, Invensensev2 | DPS310 | IST8308, IST8310 | 4 | 4 | 0 | yes | yes | no | no |
| HWH7 | 2048 | 8M | BMI270, Invensensev3 | SPL06, DPS310 | — | 3 | 1 | 0 | yes | no | no | no |
| IFLIGHT_2RAW_H7 | 2048 | 8M | Invensensev3 | SPL06 | — | 2 | 2 | 0 | no | no | no | no |
| JFB110 | 2048 | 24M | SCHA63T, Invensensev3 | MS5611 ×2 | IST8310 ×2 | 3 | 4 | 0 | yes | yes | no | no |
| JHEMCU-H743HD | 2048 | 8M | Invensensev3 ×2 | DPS310 | — | 4 | 2 | 0 | no | no | no | no |
| JPilot-C | 2048 | 16M | Invensensev3 ×2 | DPS280 | — | 2 | 1 | 0 | yes | no | no | no |
| KT-FMU-F1 | 2048 | 8M | BMI088, BMI270 | DPS310 | IST8310 ×2 | 2 | 2 | 0 | yes | no | no | no |
| KakuteH7 | 2048 | 8M | Invensense, Invensensev3, BMI270 | BMP280, SPL06 | — | 3 | 1 | 0 | no | no | no | no |
| KakuteH7-Wing | 2048 | 16M | BMI088, Invensensev3 | BMP280, SPL06 | — | 3 | 3 | 0 | yes | no | no | no |
| KakuteH7Mini | 2048 | 8M | Invensense, Invensensev3 | BMP280, SPL06 | — | 3 | 1 | 0 | no | no | no | no |
| MFT-SEMA100 | 2048 | 16M | BMI088 | BMP388 | LIS3MDL | 2 | 4 | 0 | no | no | no | no |
| MambaH743v4 | 2048 | 8M | Invensense, BMI270, Invensensev3 | DPS280, SPL06 | — | 4 | 2 | 0 | no | no | no | no |
| MatekH743 | 2048 | 8M | Invensensev3, Invensense | MS5611, DPS310, BMP280 | — | 4 | 2 | 8–13 | yes | no | no | no |
| MicoAir743 | 2048 | 8M | BMI088, BMI270 | DPS310 | IST8310 ×2 | 2 | 2 | 0 | yes | no | no | no |
| MicoAir743-AIO | 2048 | 8M | BMI088, BMI270 | DPS310 | — | 2 | 2 | 0 | yes | no | no | no |
| MicoAir743-Lite | 2048 | 8M | Invensensev3 | SPL06 | — | 2 | 2 | 0 | yes | no | no | no |
| MicoAir743v2 | 2048 | 8M | BMI088, BMI270 | SPL06 | QMC5883L ×2 | 3 | 2 | 0 | yes | no | no | no |
| Morakot | 2048 | 8M | Invensensev3 | BMP388 | IIS2MDC, QMC5883L | 2 | 1 | 0 | yes | no | yes | no |
| NarinFC-H7 | 2048 | 16M | ADIS1647x, Invensensev3, Invensense, BMI088, Invensensev2 | MS5611 ×2 | IST8310, RM3100 | 4 | 4 | 0 | yes | yes | no | no |
| NarinFC-X3 | 2048 | 8M | Invensensev3 ×2 | DPS310 | — | 3 | 2 | 0 | yes | no | no | no |
| NucleoH743 | 2048 | 8M | — | — | — | 0 | 0 | 0 | no | no | no | no |
| NxtPX4v2 | 2048 | 16M | BMI088 ×2 | SPL06 | — | 2 | 2 | 0 | yes | no | no | no |
| ORBITH743 | 2048 | 8M | Invensensev3 ×2 | DPS310 | — | 3 | 1 | 0 | no | no | no | no |
| OrqaH7QuadCore | 2048 | 8M | Invensensev3 ×2 | DPS310 | — | 4 | 2 | 0 | yes | no | no | no |
| PixC4-Jetson | 2048 | 16M | Invensensev3, Invensense | MS5611 | RM3100 | 3 | 4 | 0 | yes | yes | no | yes |
| PixPilot-V6 | 2048 | 24M | Invensensev3 ×3 | MS5611 ×2 | IST8310 | 3 | 3 | 0 | no | yes | no | yes |
| PixPilot-V6PRO | 2048 | 24M | Invensensev3 ×3 | BMP388 ×2 | — | 3 | 2 | 0 | yes | yes | no | yes |
| PixSurveyA2-IND | 2048 | 24M | Invensensev3 ×3 | BMP388 ×2 | — | 3 | 2 | 0 | yes | yes | no | yes |
| Pixhawk6C | 2048 | 16M | Invensensev3, LSM6DSV, BMI055, BMI088 | MS5611, BMP388 | IST8310 | 2 | 3 | 0 | yes | yes | no | yes |
| Pixhawk6X | 2048 | 16M | Invensensev3, BMI088, Invensensev2, ADIS1647x | BMP388, ICP201XX | BMM150, RM3100 | 4 | 4 | 8 | yes | yes | yes | yes |
| QioTekZealotH743 | 2048 | 8M | ADIS1647x, Invensense, Invensensev3 | DPS280 ×2 | IST8310, QMC5883L | 3 | 2 | 0 | no | yes | no | no |
| RADIX2HD | 128 | 16M | BMI270 | DPS310 | — | 1 | 1 | 0 | yes | no | no | no |
| ResoluteH7 | 2048 | 8M | Invensensev3 | DPS310 | — | 3 | 1 | 0 | no | no | no | no |
| SIYI_N7 | 2048 | 16M | Invensense, Invensensev3, BMI088 | MS5611 | IST8310 | 3 | 4 | 0 | yes | yes | no | yes |
| SPEDIXH743 | 2048 | 8M | Invensensev3 ×2 | SPL06 | — | 4 | 3 | 0 | no | no | no | no |
| SULILGH7-P1-P2 | 2048 | 16M | BMI088, Invensensev3, Invensense | BMP581, ICP201XX | IST8310 ×2 | 5 | 4 | 0 | yes | yes | yes | yes |
| SVehicle-E2 | 2048 | 16M | Invensensev3, BMI088, Invensensev2 | ICP201XX ×2 | RM3100 | 4 | 4 | 0 | yes | yes | yes | yes |
| SDMODELH7V1/V2 | 2048 | — | (derived from KakuteH7) | — | — | — | — | — | — | — | — | — |
| SequreH743 | 2048 | 8M | Invensense, Invensensev3 | BMP280, DPS310 | — | 3 | 1 | 0 | no | no | no | no |
| SkyRukh_Surge_H7 | 2048 | 16M | BMI088, Invensense | DPS310 | — | 5 | 2 | 0 | yes | yes | no | no |
| SkySakuraH743 | 2048 | 8M | Invensensev3 ×2 | ICP201XX, DPS310 | IST8310 | 2 | 2 | 0 | yes | no | no | no |
| SkystarsH7HD | 2048 | 8M | BMI270 ×2 | BMP280 | — | 4 | 2 | 0 | no | no | no | no |
| StellarH7V2 | 2048 | 8M | Invensensev3 | DPS310, BMP280 | — | 3 | 1 | 0 | yes | no | no | no |
| TBS_LUCID_H7 | 2048 | 8M | Invensensev3 ×2 | DPS310 | — | 3 | 2 | 0 | yes | no | no | no |
| TBS_LUCID_PRO | 1024 | 8M | Invensensev3, Invensense | BMP388 | — | 3 | 1 | 0 | no | no | no | no |
| TMotorH743 | 2048 | 8M | BMI270, Invensensev3 | DPS310 | — | 3 | 1 | 0 | no | no | no | no |
| VUAV-TinyV7 | 2048 | 16M | Invensensev3, BMI088 | ICP201XX | IST8310, QMC5883L | 2 | 3 | 0 | yes | yes | no | no |
| VUAV-V7pro | 2048 | 16M | ADIS1647x, Invensensev3 | MS5611 ×2 | RM3100 | 5 | 4 | 0 | yes | yes | no | yes |
| X-MAV-AP-H743r1 | 2048 | 8M | Invensensev3, BMI270 | SPL06 | QMC5883P ×2 | 3 | 3 | 0 | yes | yes | no | yes |
| X-MAV-AP-H743v2 | 2048 | 8M | BMI088, Invensensev3 | DPS310 | IST8310 ×2 | 3 | 2 | 0 | yes | no | no | no |
| YARIV6X | 2048 | 16M | Invensensev3 ×3 | ICP201XX ×2 | BMM350 | 4 | 4 | 0 | yes | yes | yes | yes |
| YJUAV_A6 | 2048 | 16M | Invensense, Invensensev3 | DPS310, BMP388 | RM3100, IST8310 | 3 | 2 | 0 | yes | yes | no | no |
| YJUAV_A6SE_H743 | 2048 | 16M | Invensense, Invensensev3 | DPS310, MS5611, ICP201XX | IST8310 ×2 | 3 | 4 | 0 | yes | yes | no | yes |
| YJUAV_A6Ultra | 2048 | 16M | Invensense, Invensensev3 | DPS310 ×2 | IST8310 ×2 | 3 | 4 | 0 | yes | yes | no | yes |
| ZeroOneX6 | 2048 | 16M | Invensensev3, BMI088 | ICP201XX ×2 | RM3100 | 4 | 4 | 0 | yes | yes | yes | yes |
| ZeroOneX6_Air | 2048 | 16M | Invensensev3 ×2 | ICP201XX ×2 | IST8310 | 3 | 4 | 0 | yes | yes | yes | yes |
| kha_eth | 2048 | 25M | — | — | — | 0 | 1 | 0 | no | no | yes | no |
| luminousbee5 | 2048 | 24M | Invensensev3 ×2 | DPS280 | HMC5843 | 3 | 1 | 0 | yes | yes | no | no |
| mRoControlZeroClassic | 2048 | 24M | BMI088, Invensense, Invensensev2 | DPS310 | AK09916 | 3 | 2 | 0 | yes | yes | no | no |
| mRoControlZeroH7 | 2048 | 24M | BMI088, Invensense, Invensensev2 | DPS310 | AK09916 | 3 | 1 | 0 | yes | yes | no | no |
| mRoControlZeroOEMH7 | 2048 | 24M | BMI088, Invensense, Invensensev2 | DPS310 | AK09916 | 3 | 3 | 0 | yes | yes | no | no |
| mRoNexus | 2048 | 25M | ADIS1647x, Invensensev3 | DPS310 | RM3100 | 3 | 1 | 0 | yes | yes | no | no |
| mRoPixracerPro | 2048 | 24M | BMI088, Invensense, Invensensev2 | DPS310 | AK09916 | 3 | 1 | 0 | yes | yes | no | no |
| rFCU | 2048 | 16M | ADIS1647x, Invensensev3 | MS5611, BMP388 ×4 | RM3100 | 4 | 4 | 0 | yes | yes | no | no |
| uav-dev-fc-um982 | 2048 | 24M | Invensensev3 | DPS310 | BMM150 | 1 | 1 | 0 | yes | no | yes | no |

**H750xx / External Flash H7 sub-family** (128 KB internal, XIP from QSPI flash):

| Board | Notes |
|-------|-------|
| DevEBoxH7v2 | Dev board, 128KB internal, SD only |
| RADIX2HD | Racing FC, 128KB internal, external QSPI |
| SPRacingH7 | Racing FC, 128KB internal, external QSPI |
| YJUAV_A6SE | 2048KB listed but H750; external flash |

**H730xx (STM32H730 @ 520 MHz)**:
- SPRacingH7RF — racing FC, 128KB internal, external QSPI, single ICM-based IMU

**H723xx**:
- BotBloxDroneNet — network switch / DroneCAN node with Ethernet

**H757xx boards (dual-core M7+M7)**:

| Board | IMUs | Notes |
|-------|------|-------|
| CubeOrangePlus | Invensensev3 ×7, Invensensev2 | 3 isolated IMU sets; flagship Cube |
| CubeNode | Invensensev3 | DroneCAN node with Ethernet |
| CubeRedPrimary | Invensensev3, Invensensev2 | Redundant flight system |
| CubeRedSecondary | Invensensev3 | Secondary FC in redundant pair |
| CubeRedSecondary-IO | — | IO controller for redundant pair |
| H757I_EVAL | — | ST eval board, no sensors |
| H757I_EVAL_intf | — | Interface board for eval pair |
| Here4AP | Invensensev3 | DroneCAN GPS node |
| Here4FC | Invensensev3 | Full FC in Here4 ecosystem |
| NucleoH755 | Invensensev2 | Dev board |
| JFB110 | SCHA63T, Invensensev3 | High-end tactical |

**H753xx**:
- NucleoH753ZI — dev board

**H755xx**:
- JFB110 (listed above under H757, but MCU is STM32H755)

---

### 4.2 STM32F405xx Flight Controllers (63 boards)

Budget FPV and mini FC market. 1024 KB flash, 168 MHz, 192 KB RAM. No SD card on most (flash storage via internal pages).

**High-volume consumer FPV boards**:
- speedybeef4, speedybeef4v3/v4/v5 — Speedy Bee series (Invensensev3/BMI270, SPL06)
- omnibusf4pro, omnibusf4v6 — OmnibusF4 series (Invensense, BMP280)
- SpeedyBeeF405AIO/Mini/WING — newer Speedy Bee (Invensensev3, SPL06)
- KakuteF4, KakuteF4-Wing — Holybro Kakute (Invensense, BMP280/SPL06)
- revo-mini — Revolution Mini (Invensense, MS5611)

**Wing/fixed-wing oriented**:
- HEEWING-F405 — (Invensensev3, SPL06)
- MatekF405-Wing — (Invensense/Invensensev3, BMP280/DPS310)
- LongBowF405WING — (Invensensev3, SPL06)
- JHEMCUF405WING — (Invensensev3, SPL06)
- KakuteF4-Wing — (Invensensev3, SPL06)

**Notable full-featured F405**:
- CarbonixF405 — UAV company board (MS5611, QMC5883P, IST8310, no IMU embedded)
- RadiolinkF405 — (Invensense, Invensensev3, BMI270, multi-baro)
- crazyflie2 — Bitcraze research platform (BMI088, BMP388, LPS2XH)
- DrotekP3Pro — STM32F469 (one-off 180MHz F4)

**Representative data** (selected):

| Board | IMUs | Baro | Compass | SD | FRAM | PWM |
|-------|------|------|---------|----|----|-----|
| speedybeef4v5 | Invensensev3 | SPL06 | — | no | no | 3 |
| omnibusf4pro | Invensense, BMI270 | BMP280 | — | no | no | 3 |
| MatekF405-Wing | Invensense, Invensensev3 | BMP280, DPS310 | — | no | no | 8 |
| KakuteF4 | Invensense | BMP280, SPL06 | — | no | no | 8 |
| revo-mini | Invensense | MS5611 | HMC5843 | no | no | 6 |
| MambaF405v2 | Invensense, Invensensev3 | BMP280 | — | no | no | 4 |
| RadiolinkF405 | Invensense, Invensensev3, BMI270 | SPL06, BMP280 | — | no | no | 9 |

---

### 4.3 STM32F427xx Flight Controllers (16 boards)

2048 KB flash, 168 MHz, 256 KB RAM. The "classic" high-end F4 — Pixhawk 1, Pixracer, etc. Most use FRAM for parameter storage instead of SD.

| Board | IMUs | Baro | Compass | FRAM | Notes |
|-------|------|------|---------|------|-------|
| fmuv3 | (base reference) | — | — | yes | Base for Pixhawk1, CubeBlack, CubeSolo |
| Pixracer | Invensense ×2 | MS5611 | HMC5843, LIS3MDL, AK8963 | yes | First Pixracer |
| luminousbee4 | BMI088 | MS5611 | HMC5843, LIS3MDL | yes | — |
| mindpx-v2 | Invensense, LSM9DS0 | MS5611 | HMC5843, LSM303D | yes | — |
| thepeach-k1/r1 | Invensense ×2 | MS5611 | AK8963 | yes | Korean |
| VRBrain-v54 | Invensense | MS5611 | HMC5843 ×2 | yes | Italian |
| QioTekZealotF427 | Invensense, BMI055, BMI088, Invensensev2, Invensensev3 | DPS280, MS5611 | IST8310, QMC5883L | yes | — |
| PixSurveyA1-IND | Invensensev3 ×3 | MS5611 ×2 | IST8310 | yes | Survey FC |
| PixPilot-C3/V3 | Invensensev3, Invensensev3 ×3 | BMP388, MS5611 | — | yes | — |
| FlyingMoonF427 | Invensense, Invensensev2, Invensensev3 | SPL06, MS5611 | RM3100 | yes | — |
| F4BY_F427 | Invensense | MS5611 ×2 | HMC5843, QMC5883L, QMC5883P | yes | — |

---

### 4.4 STM32F745xx / F767xx / F777xx Flight Controllers (23 boards)

F7 family — 1024–2048 KB flash, 216 MHz, 512 KB RAM. Bridge generation between F4 and H7.

**F745xx (1024 KB, 216 MHz)**:
| Board | IMUs | Baro | Notes |
|-------|------|------|-------|
| BeastF7 | Invensense | BMP280 | — |
| BlitzF745 | Invensensev3 | DPS310, SPL06 | — |
| BlitzF745AIO | BMI270, Invensensev3 | DPS310, SPL06 | All-in-one |
| FlywooF745 | Invensense, BMI270, Invensensev3 | BMP280, SPL06, DPS310 | — |
| GEPRCF745BTHD | Invensensev3, Invensense | BMP280, SPL06 | HD FPV |
| KakuteF7/F7Mini | Invensense | BMP280 | — |
| MazzyStarDrone | BMI088, Invensense | BMP280 | — |
| OMNIBUSF7V2 | Invensense ×2 | BMP280 | — |
| ReaperF745 | BMI270, Invensensev3, Invensense | — | Racing |

**F767xx (2048 KB, 216 MHz)**:
| Board | IMUs | Baro | Notes |
|-------|------|------|-------|
| AIRLink | Invensense ×3 | MS5611, BMP388 | AirLink companion |
| JFB100 | SCHA63T, Invensense | MS5611 | High-end tactical |
| LumenierLUXF765-NDAA | Invensensev3 | BMP280 | NDAA-compliant |
| MatekF765-Wing | Invensense ×2 | BMP280, DPS310, MS5611 | Wing |
| Pixhawk5X | Invensensev3, Invensensev2, Invensense | BMP388 ×2 | Mid-range pro |
| R9Pilot | Invensense, Invensensev2 | SPL06 | RC receiver with autopilot |
| RadiolinkPIX6 | Invensensev3, BMI088 | SPL06 | — |
| PixFlamingo-F767 | Invensensev3, Invensense | MS5611, BMP280, DPS310 | — |
| fmuv5 | Invensense, BMI055, BMI088 | MS5611 | Base for Pixhawk4/CUAVv5 |
| modalai_fc-v1 | Invensensev3, Invensense | BMP388 | ModalAI companion |

**F777xx (2048 KB, 216 MHz)**:
| Board | Notes |
|-------|-------|
| CubeYellow | H7 generation Cube in F777 |
| mRoControlZeroF7 | mRo F7 variant |
| mRoX21-777 | mRo X2.1 777 |

---

### 4.5 STM32F412Rx Flight Controllers and Periph Nodes (13 boards)

512–1024 KB flash, 100 MHz, 256 KB RAM. Primarily GPS/periph nodes, a few small FCs.

| Board | Role | IMUs | Notes |
|-------|------|------|-------|
| ARK_CANNODE | Periph | — | Generic CAN node |
| ARK_GPS | Periph GPS | — | BMP388, BMM150 |
| ARK_RTK_GPS | Periph GPS | — | BMP388, BMM150 |
| BirdCANdy | Periph | — | CAN adapter |
| C-RTK2-HP | Periph GPS | — | RM3100 compass |
| CSKY_PMU | Periph | — | Power module |
| CUAV_GPS | Periph GPS | — | Multi-baro, IST8310/RM3100 |
| FreeflyRTK | Periph GPS | — | BMP388, IST8310 |
| HolybroGPS | Periph GPS | BMI088 | BMP388, BMM150 |
| Sierra-F412 | Periph | — | DPS310, RM3100 |
| Sierra-F9P | Periph GPS/RTK | — | F9P RTK, DPS310, QMC5883L |
| Sierra-PrecisionPoint | Periph | Invensensev3 | DPS310, RM3100 |
| skyviper-f412-rev1 | FC | Invensense | ICM20789, BMM150 |
| skyviper-journey | FC | Invensense | ICM20789, BMM150 |

---

### 4.6 STM32F407xx Legacy Flight Controllers (8 boards)

1024 KB flash, 168 MHz, 192 KB RAM. Effectively STM32F405 family. Mostly legacy Italian/educational boards.

| Board | IMUs | Notes |
|-------|------|-------|
| AR-F407SmartBat | — (CKS32F407 clone) | Battery management |
| F4BY | Invensense | MS5611, HMC5843 |
| FlyingMoonF407 | Invensense | MS5611, IST8310 |
| QioTekAdeptF407 | Invensensev3, Invensense | CKS32F407 clone |
| VRBrain-v51 | Invensense | MS5611, HMC5843 |
| VRBrain-v52 | Invensense | MS5611, HMC5843 |
| VRCore-v10 | Invensense | MS5611, HMC5843 |
| VRUBrain-v51 | Invensense | MS5611, HMC5843 |
| sw-boom-f407 | — | SW Periph |
| sw-spar-f407 | — | SW Periph compass node |

---

### 4.7 Other MCU Families (FCs)

**STM32F469xx** (1 board):
- DrotekP3Pro — 2048 KB, 180 MHz; Invensense ×2, MS5611, FRAM

**STM32L4R5xx** (1 board):
- PixFlamingo — 2048 KB, 120 MHz; Invensensev3+LSM9DS0, MS5611, SD card

**STM32F732xx** (1 board):
- FreeflyRTK — 512 KB, 216 MHz; GPS/RTK periph node

**STM32F105xC** (1 board):
- ZubaxGNSS — 256 KB, 72 MHz; legacy CAN GPS node

---

## 5. AP_PERIPH Peripheral Nodes

111 boards in total are AP_PERIPH nodes. These run the `AP_Periph` firmware — a lean CAN/DroneCAN node, no GCS/MAVLink, no vehicle code. They communicate via DroneCAN to the main FC.

### 5.1 STM32L431xx Periph Nodes (8 boards)

256 KB flash, 80 MHz, 64 KB RAM. Low-power CAN node. Most are the "MatekL431" family:

| Board | Function |
|-------|----------|
| MatekL431 | Base board |
| MatekL431-ADSB | ADS-B receiver |
| MatekL431-APDTelem | APD telemetry |
| MatekL431-AUAV | AUAV baro |
| MatekL431-Airspeed | MS4525/DLVR airspeed |
| MatekL431-BattMon | Battery monitor |
| MatekL431-BatteryTag | Battery ID tag |
| MatekL431-DShot | DShot ESC telem bridge |
| MatekL431-EFI | EFI data |
| MatekL431-GPS | GPS + RM3100/QMC5883L |
| MatekL431-HWTelem | Hardware telemetry |
| MatekL431-MagHiRes | RM3100 high-res compass |
| MatekL431-Periph | General periph (GPS+baro+compass) |
| MatekL431-Proximity | Rangefinder |
| MatekL431-RC | RC receiver |
| MatekL431-Rangefinder | Rangefinder |
| MatekL431-Serial | Serial bridge |
| MatekL431-bdshot | Bidirectional DShot |
| 3DR-L431 | 3DR L431 node |
| AeroFox-GNSS_F9P | RTK GPS (QMC5883L) |
| AeroFox-PMU | Power module |
| Sierra-L431 | Sierra GPS RM3100 |
| Sierra-TrueNavIC | GPS DPS310, QMC5883L |
| Sierra-TrueNavPro | GPS DPS310, RM3100 |
| Sierra-TrueNorth | Compass RM3100 |
| Sierra-TrueSpeed | Airspeed BMM150 |
| TBS-L431 | TBS CAN node base |
| TBS-L431-Airspeed | Airspeed |
| TBS-L431-BattMon | Battery monitor |
| TBS-L431-CurrMon | Current monitor |
| TBS-L431-PWM | PWM output |
| VM-L431 | VM base |
| VM-L431-BMS | BMS |
| VM-L431-BatteryTag | Battery tag |
| VM-L431-Periph-Pico | Pico size periph |
| VM-L431-SRV-Hub-4CHP | 4-channel servo hub |
| uav-dev_m10s | M10S GPS BMM150 |

### 5.2 STM32F412Rx Periph Nodes

See section 4.6 above — ARK_GPS, ARK_CANNODE, BirdCANdy, CUAV_GPS, HolybroGPS, C-RTK2-HP, FreeflyRTK, Sierra-F412, Sierra-F9P, etc.

### 5.3 STM32G474xx / G491xx Periph Nodes

| Board | MCU | Flash | Function |
|-------|-----|-------|----------|
| G4-ESC | STM32G474 | 512 KB | ESC node |
| HolybroG4_Airspeed | STM32G474 | 512 KB | Airspeed |
| HolybroG4_Compass | STM32G474 | 512 KB | RM3100 compass |
| HolybroG4_GPS | STM32G474 | 512 KB | GPS+IMU+baro |
| MatekG474 | STM32G474 | 512 KB | Base CAN node |
| MatekG474-DShot | STM32G474 | 512 KB | DShot bridge |
| MatekG474-GPS | STM32G474 | 512 KB | GPS + RM3100 |
| MatekG474-Periph | STM32G474 | 512 KB | General periph |
| mRoCANPWM-M10126 | STM32G474 | 512 KB | CAN PWM driver |
| mRo-M10095 | STM32G491 | 512 KB | GPS node |
| rGNSS | STM32G491 | 512 KB | GPS BMP388 RM3100 |
| Sierra-TrueNavPro-G4 | STM32G491 | 256 KB | GPS DPS310 RM3100 |
| Nucleo-G491 | STM32G491 | 256 KB | Dev board |
| uav-dev-auav-g4 | STM32G474 | 512 KB | Dev node |
| uav-dev-powermodule | STM32G474 | 512 KB | Power module |
| Hitec-Airspeed | STM32G441 | 128 KB | Airspeed only |

### 5.4 STM32F103xB Periph Nodes (8 boards)

128 KB flash, 72 MHz, 20 KB RAM. Tiny CAN/GPS peripherals. Legacy only.

| Board | Function |
|-------|----------|
| f103-periph | Base for F103 periph |
| f103-ADSB | ADS-B |
| f103-Airspeed | Airspeed |
| f103-GPS | GPS |
| f103-HWESC | HW ESC |
| f103-QiotekPeriph | Qiotek CAN |
| f103-RangeFinder | Rangefinder |
| f103-Trigger | Camera trigger |
| MFE_AirSpeed_CAN | Airspeed |
| AeroFox-Airspeed | Airspeed |
| AeroFox-Airspeed-DLVR | DLVR airspeed |
| ZubaxGNSS | GPS/compass node (F105xC) |

### 5.5 STM32F303xC Periph Nodes

| Board | Function |
|-------|----------|
| f303-MatekGPS | GPS + DPS310 + QMC5883L |
| f303-GPS | GPS + RM3100 |
| f303-HWESC | HW ESC |
| f303-M10025/M10070 | GPS variants |
| f303-PWM | PWM output |
| f303-TempSensor | Temperature sensor |
| f303-Universal | Universal periph |
| mRoKitCANrevC | CAN node |

### 5.6 STM32F405xx Periph Nodes

| Board | Function |
|-------|----------|
| CarbonixF405 | Primary FC (also acts as periph) |
| HolybroF4_PMU | Power module |
| Sierra-F405 | DPS310 + RM3100 |
| f405-MatekGPS | GPS + DPS310 + RM3100/QMC5883L |
| f405-MatekAirspeed | Airspeed |
| sw-nav-f405 | SW nav periph |
| MFE_POS3_CAN | Position/compass (F427) |

### 5.7 Other / Special Periph Boards

| Board | MCU | Function |
|-------|-----|----------|
| BotBloxDroneNet | STM32H723 | Ethernet/CAN switch |
| CubeNode | STM32H757 | CAN+Ethernet node |
| CubeNode-ETH | STM32H757 | CAN+Ethernet (derived) |
| Here4AP | STM32H757 | DroneCAN GPS in Here4 |
| kha_eth | STM32H743 | Ethernet research node |
| HitecMosaic | STM32F303 | High-precision compass module |
| Morakot | STM32H743 | CAN+Ethernet hybrid |
| MFE_PDB_CAN | STM32L431 | Power distribution board |

---

## 6. Special and Infrastructure Boards

### 6.1 IOMCU Boards (6 boards)

The IOMCU is a secondary microcontroller embedded on some full-featured FCs (Pixhawk, Cube series). It handles RC input, PWM output, and safety functions under the primary MCU's direction. The primary MCU runs ArduPilot and sends commands to the IOMCU over a dedicated UART (usually USART6).

| Board | MCU | Flash | Crystal | Role |
|-------|-----|-------|---------|------|
| iomcu | STM32F100xB | 64 KB | 24 MHz | Standard IOMCU (8 PWM, RC in) |
| iomcu-dshot | STM32F100xB | 64 KB | — | IOMCU with DShot output |
| iomcu-f103 | STM32F103xB | 128 KB | — | F103-based IOMCU |
| iomcu-f103-8MHz-dshot | STM32F103xB | 128 KB | 8 MHz | F103 IOMCU DShot |
| iomcu-f103-dshot | STM32F103xB | 128 KB | — | F103 IOMCU DShot |
| iomcu_f103_8MHz | STM32F103xB | 128 KB | 8 MHz | F103 IOMCU 8MHz |

**IOMCU provides**: 8 PWM outputs (3 timers), RC input (SBUS + PPM), safety button, safety LED. It communicates via a custom UART protocol at high baud rate. 44 boards connect a primary MCU (H743/F7/F427) to an IOMCU via USART6.

### 6.2 Reference/Template Boards

| Board | MCU | Notes |
|-------|-----|-------|
| fmuv2 | (from fmuv3) | Pixhawk 1.0 — obsolete, 1M flash |
| fmuv3 | STM32F427xx | Pixhawk 2.4.8 reference |
| fmuv3-bdshot | STM32F427xx | fmuv3 + DShot |
| fmuv5 | STM32F767xx | Pixhawk 4 / CUAVv5 reference |

### 6.3 Development / Eval Boards

| Board | MCU | Purpose |
|-------|-----|---------|
| H757I_EVAL | STM32H757 | ST eval board |
| H757I_EVAL_intf | STM32H757 | Interface for above |
| NucleoH743 | STM32H743 | ST Nucleo dev |
| NucleoH753ZI | STM32H753 | ST Nucleo dev |
| NucleoH755 | STM32H757 | ST Nucleo dev |
| Nucleo-G491 | STM32G491 | ST Nucleo dev |
| Nucleo-L476 | STM32L476 | ST Nucleo dev |
| Nucleo-L496 | STM32L496 | ST Nucleo dev |
| DevEBoxH7v2 | STM32H750 | WeAct dev board |
| STM32CubeConf | — | CubeMX config reference |
| uav-dev-auav-g4 | STM32G474 | Dev periph node |
| uav-dev-powermodule | STM32G474 | Dev power module |

### 6.4 Boards with Ethernet

22 boards have Ethernet MAC support (via RMII):

| Board | MCU | Role |
|-------|-----|------|
| ARKV6X | STM32H743 | Full FC + ETH |
| AcctonGodwit_GA1 | STM32H743 | Full FC + ETH |
| Aeromind6X | STM32H743 | Full FC + ETH |
| Airvolute-DCS2 | STM32H743 | FC + ETH |
| BotBloxDroneNet | STM32H723 | Network switch periph |
| CBU-H7-Stamp | STM32H743 | Stamp-form FC + ETH |
| CUAV-7-Nano | STM32H743 | FC + ETH |
| CUAV-V6X-v2 | STM32H743 | FC + ETH |
| CUAV-X25-EVO | STM32H743 | FC + ETH |
| CubeNode | STM32H757 | CAN/ETH node |
| CubeRedPrimary | STM32H757 | Redundant FC + ETH |
| Morakot | STM32H743 | FC + ETH |
| NucleoH753ZI | STM32H753 | Dev |
| Pixhawk5X | STM32F767 | FC + ETH |
| Pixhawk6X | STM32H743 | FC + ETH |
| SULILGH7-P1-P2 | STM32H743 | FC + ETH |
| SVehicle-E2 | STM32H743 | FC + ETH |
| YARIV6X | STM32H743 | FC + ETH |
| ZeroOneX6 | STM32H743 | FC + ETH |
| ZeroOneX6_Air | STM32H743 | FC + ETH |
| kha_eth | STM32H743 | Ethernet-only research |
| uav-dev-fc-um982 | STM32H743 | Dev FC + ETH |

---

## 7. Sensor Population Statistics

### 7.1 IMU Driver Distribution

| Driver | Count | Chips covered |
|--------|-------|---------------|
| Invensensev3 | 221 | ICM-42688-P, ICM-42605, ICM-42670, IIM-42652, ICM-45686 |
| Invensense | 140 | MPU-6000, ICM-20689, ICM-20608, MPU-9250, ICM-20948 |
| BMI088 | 50 | Bosch BMI088 6-DoF |
| BMI270 | 33 | Bosch BMI270 6-DoF |
| Invensensev2 | 31 | ICM-20649, ICM-20649 high-g |
| ADIS1647x | 9 | Analog Devices ADIS16470/ADIS16477 (high-end tactical) |
| LSM9DS0 | 7 | ST legacy (Pixhawk 1/2 era) |
| BMI055 | 6 | Bosch legacy |
| SCHA63T | 2 | Murata SCHA63T (automotive-grade) |
| LSM6DSV | 1 | ST new gen |
| BMI160 | 1 | Bosch legacy |

**Most common combination**: Invensensev3 (primary) + Invensense or BMI088 (secondary) on high-end H743 boards.

### 7.2 Barometer Distribution

| Driver | Count | Chips |
|--------|-------|-------|
| MS5611 | 85 | TE MS5611 — legacy standard |
| DPS310 | 73 | Infineon DPS310 — current mainstream |
| SPL06 | 55 | GOERTEK SPL06 — budget |
| BMP280 | 44 | Bosch BMP280 — budget |
| BMP388 | 35 | Bosch BMP388 — new generation |
| ICP201XX | 24 | TDK InvenSense ICP-20100 — new high-res |
| DPS280 | 10 | Infineon DPS280 |
| BMP581 | 4 | Bosch BMP581 — newest |
| ICM20789 | 3 | Invensense combined IMU+baro |

### 7.3 Compass Distribution

| Driver | Count | Chips |
|--------|-------|-------|
| RM3100 | 67 | PNI RM3100 — high-end, SPI/I2C |
| IST8310 | 57 | iSentek IST8310 — mid-range I2C |
| QMC5883L | 21 | QST QMC5883L — budget I2C |
| HMC5843 | 20 | Honeywell legacy |
| BMM150 | 14 | Bosch BMM150 — mid-range |
| AK09916 (via ICM20948) | 12 | AKM combo |
| AK8963 (via MPU9250) | 11 | AKM combo legacy |
| LIS3MDL | 10 | ST |
| LSM303D | 7 | ST legacy |
| QMC5883P | 5 | QST high-performance |
| IIS2MDC | 4 | ST industrial |

### 7.4 SPI Bus Assignment Patterns

Most common sensor→bus assignments across all boards:

| Sensor class | Bus 1 (SPI1) | Bus 2 (SPI2) | Bus 3/4 |
|--------------|-------------|-------------|---------|
| Primary IMU | IMU/ICM42688/MPU6000 (378 entries) | OSD MAX7456 (56), ICM42688 (12) | Secondary IMU (SPI4) |
| OSD | osd (17) | osd (56) | osd SPI3 (17) |
| FRAM/NV storage | ms5611/rm3100 | ramtron (47), ramtron SPI5 (19) | — |
| Dataflash | — | dataflash (15) | dataflash SPI3 (45) |
| SD card | — | sdcard (17) | sdcard SPI3 (24) |

**Key pattern**: SPI1 → primary IMU. SPI2 → OSD chip (MAX7456) or secondary IMU. SPI3/4 → secondary IMU, dataflash. SPI5 → FRAM (Pixhawk family only).

This is NOT fully standardized — many budget boards ignore SPI2-for-OSD and put the IMU there. The only reliable pattern is SPI1 for the first/primary IMU.

### 7.5 PWM Output Distribution

| PWM count | Boards |
|-----------|--------|
| 1–3 | 20 |
| 4 | 31 |
| 5–6 | 47 |
| 7–8 | 62 (most common for mini quads) |
| 9–10 | 55 |
| 11–12 | 29 |
| 13–14 | 34 |
| 15–16 | 11 |

**Typical configurations**:
- Budget FPV quad (F405/H743): 4–8 outputs
- Wing/fixed-wing: 8–12 outputs
- Professional FC (Pixhawk6X, Cube, Durandal): 14–16 outputs (some via IOMCU)

---

## 8. Pin Mapping Patterns

### 8.1 Is There a Standard?

No strict standard exists, but strong conventions:

**H743 boards (community FPV/Wing)**:
- SPI1 → primary IMU CS on PC15/PE4/similar
- SPI2 → OSD (MAX7456) or secondary IMU
- SPI3 or SPI4 → secondary IMU or dataflash
- SDMMC1 → SD card (PC8-PC12, PD2 on all boards using it)
- I2C1 on PB6/PB7 → barometer, compass
- I2C2 on PB10/PB11 → secondary compass or external
- UART7 → Telem1 (PE7/PE8)
- USART1 → GPS1 (PA9/PA10 or PB6/PB7)
- USART3 → GPS2 or debug
- OTG1 → PA11/PA12 (USB, universal)
- USART6 → IOMCU (PC6/PC7) when present
- CAN1 → PD0/PD1 (universal)
- CAN2 → PB12/PB13 or PB6/PB7

**fmuv3 / Cube family (F427/H743 traditional)**:
- SERIAL_ORDER: OTG1 USART2 USART3 UART4 UART8 UART7 OTG2
- IOMCU on USART6
- FRAM on SPI2 (ramtron at SPI2/DEVID1)
- MS5611 on SPI1
- Primary IMU on SPI1
- Secondary IMU on SPI1 (different CS)
- External SPI on SPI4 or SPI5

**fmuv5 / Pixhawk4 family (F767)**:
- SERIAL_ORDER: OTG1 UART7 USART2 USART1 UART4 UART8 UART3 OTG2
- IOMCU on USART6
- Dual FRAM (ramtron SPI2+SPI3)
- 3× IMU on 3 independent SPI buses

### 8.2 Clock/Timer Conventions

Almost universal for H7 boards:
- `STM32_ST_USE_TIMER 2` (32-bit) for system tick — on fmuv6/Pixhawk6X style
- `STM32_ST_USE_TIMER 12` (16-bit) for FPV/racing style (MatekH743, KakuteH7)
- `define CH_CFG_ST_RESOLUTION 16` accompanies timer 12

### 8.3 UART Serial Order

The `SERIAL_ORDER` directive maps ArduPilot serial port numbers (0=USB, 1-7=hardware) to hardware UARTs. Across all H743 boards:

```
SERIAL_ORDER OTG1 [Telem1] [Telem2] [GPS1] [GPS2] [spare...] [OTG2]
```

Serial 0 is always OTG1 (USB). The rest vary per board. No pin-level standard forces a specific UART onto any specific function — that's all soft-configured via MAVLink parameters at runtime.

---

## 9. Minimum Viable MCU Analysis

### What Does ArduPilot Need to Run?

From examining the flash/RAM constraints:

| Resource | Minimum (AP_Periph) | Minimum (full ArduPilot FC) | Comfortable FC |
|----------|--------------------|-----------------------------|----------------|
| Flash | 128 KB | 1024 KB | 2048 KB |
| RAM | 20 KB | 192 KB (F405) | 512+ KB (F7/H7) |
| CPU | 72 MHz (not recommended) | 168 MHz (F4) | 216+ MHz (F7/H7) |

### H750xx Special Case (External Flash)

Boards like SPRacingH7, RADIX2HD, DevEBoxH7v2 use STM32H750 which has only 128 KB internal flash. They execute code from external QSPI/Octo-SPI NOR flash via XIP (execute-in-place). This is valid but adds latency and requires QSPI hardware. The build system handles this via `env USE_EXTERNAL_FLASH True` + special linker scripts.

### Flash Usage Reality

Based on build sizes seen in the wild:
- STM32F405xx (1024 KB) — ArduPilot fills ~750–900 KB, leaving minimal room for features
- STM32F427xx (2048 KB) — comfortable; features can be enabled
- STM32H743xx (2048 KB) — ample; full feature set including EKF3

### RAM Usage Reality

- STM32F405xx (192 KB effective) — EKF2, no EKF3, limited logging buffer
- STM32F7xx (512 KB) — EKF3, full logging
- STM32H7xx (1055+ KB) — EKF3, multiple EKF instances, large DMA buffers

### Recommendation for Meridian

**Primary target**: STM32H743xx
- 93 boards available as hardware targets
- 2048 KB flash, ~1 MB effective RAM
- 400 MHz Cortex-M7 with double-precision FPU
- All features available

**Secondary target**: STM32F7xx (STM32F745, STM32F767)
- 10+ FC boards
- 512 KB RAM is adequate for navigation
- 216 MHz, adequate for all real-time tasks

**Minimum target**: STM32F405xx
- 63 boards
- 192 KB RAM is tight — no double-precision EKF, limited features
- Fine for basic attitude control + position hold

**Avoid for new Meridian board targets**: STM32F427xx (aging), STM32F103xB/F100xB (IOMCU only, 20KB RAM).

---

## 10. Meridian Multi-Board Abstraction Design

### 10.1 Can We Use Build-Time Config (Like hwdef)?

Yes, and this is the right approach. ArduPilot's hwdef system is the gold standard for this exact problem. For Meridian:

**Recommendation**: Build-time board config via Cargo feature flags + a board-specific config crate.

Each board gets a crate `meridian-board-<boardname>` (or a feature flag `board-matekh743`) that implements a `Board` trait. The trait provides:
- Pin assignments (SPI bus numbers, CS pins, UART numbers)
- Clock configuration (crystal Hz, PLL multiplier)
- Sensor list (which drivers to enable at compile time)
- Memory map (flash base, stack region, DMA-safe regions)

### 10.2 Minimum Board-Specific Information Required

For each supported board, Meridian needs to know:

```rust
// Required per-board constants (can be const generics or feature flags)
const BOARD_MCU: McuType;        // STM32H743, STM32F405, etc.
const OSCILLATOR_HZ: u32;        // External crystal frequency
const FLASH_SIZE_KB: u32;        // For storage allocation
const STORAGE_FLASH_PAGE: u32;   // Which flash page(s) for params

// SPI bus assignments
const IMU_SPI_BUS: u8;           // Which SPI bus has the primary IMU
const IMU_CS_PIN: GpioPin;       // CS pin for primary IMU

// I2C bus assignments
const BARO_I2C_BUS: u8;
const COMPASS_I2C_BUS: u8;

// UART assignments (ArduPilot-style serial order)
const SERIAL_ORDER: &[UartId];

// Optional features
const HAS_IOMCU: bool;
const IOMCU_UART: Option<UartId>;
const HAS_SDCARD: bool;
const HAS_ETHERNET: bool;
const HAS_FRAM: bool;
const FRAM_SPI_BUS: u8;
```

### 10.3 How ArduPilot Handles Sensor Discovery vs. Meridian

ArduPilot probes for sensors at runtime — it tries every IMU driver in the list and uses the ones that respond. The hwdef just provides the list to try. Meridian can use compile-time sensor selection (faster, smaller binary) or runtime probing (more flexible).

Given Rust's zero-cost abstractions, **compile-time selection with feature flags** is preferred:
```
cargo build --features board-matekh743
```
This enables `ICM42688` and `ICM42605` drivers, disables `ADIS1647x` and `SCHA63T`.

### 10.4 Trait Object vs. Generic Approach

For Meridian, prefer **generic types** for HAL drivers (SpiDevice<SPI1>, SpiDevice<SPI2>) rather than trait objects (`Box<dyn SpiDevice>`). This avoids allocation and vtable overhead, which matters on F4-class parts with limited RAM.

For higher-level sensor abstraction (switching between e.g. ICM42688 and BMI270 at the same SPI slot), a **trait object with static dispatch** via an enum is appropriate:
```rust
enum ImuDriver { ICM42688(Icm42688Driver), BMI270(Bmi270Driver), ... }
```

### 10.5 The IOMCU Question

44 boards use a separate IOMCU for PWM output. For Meridian, the choice is:
1. **Support IOMCU**: Implement the IOMCU communication protocol (UART-based, custom ArduPilot protocol). Required for Pixhawk6X, CubeOrange, Durandal, etc.
2. **Direct PWM only**: Skip boards with IOMCU. MatekH743, KakuteH7, most FPV boards do direct timer-based PWM without IOMCU.

**Recommendation**: Start with direct-PWM boards (no IOMCU). This covers all 49+ H743 boards that don't use IOMCU. Add IOMCU support in a later milestone for Pixhawk family compatibility.

### 10.6 Suggested Priority Order for Meridian Board Support

| Priority | Board | MCU | Reason |
|----------|-------|-----|--------|
| 1 | MatekH743 | STM32H743 | Well-documented, community reference, 8MHz crystal, direct PWM |
| 2 | KakuteH7 | STM32H743 | Popular, simple sensor set, widely available |
| 3 | Pixhawk6X | STM32H743 | Industry standard, tests IOMCU path |
| 4 | KakuteF7 | STM32F745 | F7 validation |
| 5 | MatekF405-Wing | STM32F405 | F4 validation, wing use case |
| 6 | CubeOrangePlus | STM32H757 | Dual-core H7 validation, professional use |
| 7 | SpeedyBeeF405AIO | STM32F405 | AIO pattern (ESC+FC in one) |
| 8 | MatekL431-Periph | STM32L431 | DroneCAN node validation |

---

## Appendix A: All 430 Board Directories

For reference, the complete sorted list:
```
3DR-L431                3DR-L431-ASAUAV         3DRControlZeroG
ACNS-CM4Pilot           ACNS-F405AIO            AEDROXH7
AEROFOX-H7              AET-H743-Basic          AIRBRAINH743
AIRLink                 AR-F407SmartBat         ARKV6X
ARKV6X-bdshot           ARK_CANNODE             ARK_FPV
ARK_GPS                 ARK_PI6X                ARK_RTK_GPS
ATOMRCF405NAVI-Deluxe   AcctonGodwit_GA1        AeroCogito-H7Digital
AeroFox-Airspeed        AeroFox-Airspeed-DLVR   AeroFox-GNSS_F9P
AeroFox-PMU             Aeromind6X              Airvolute-DCS2
AnyleafH7               Aocoda-RC-H743Dual      Atlas-Control
AtomRCF405NAVI          BETAFPV-F405            BETAFPV-F405-I2C
BOTWINGF405             BROTHERHOBBYF405v3      BROTHERHOBBYH743
BeastF7                 BeastF7v2               BeastH7
BeastH7v2               BirdCANdy               BlitzF745
BlitzF745AIO            BlitzH743Pro            BlitzMiniF745
BlitzWingH743           BotBloxDroneNet         BrahmaF4
C-RTK2-HP               CBU-H7-LC-Stamp         CBU-H7-Stamp
CORVON405V2_1           CORVON743V1             CSKY405
CSKY_PMU                CUAV-7-Nano             CUAV-7-Nano-ODID
CUAV-Nora               CUAV-Nora-ODID          CUAV-Nora-bdshot
CUAV-Pixhack-v3         CUAV-V6X-v2             CUAV-V6X-v2-ODID
CUAV-X25-EVO            CUAV-X25-EVO-ODID       CUAV-X7
CUAV-X7-ODID            CUAV-X7-bdshot          CUAV_GPS
CUAVv5                  CUAVv5-bdshot           CUAVv5Nano
CUAVv5Nano-bdshot       CarbonixF405            CrazyF405
CubeBlack               CubeBlack+              CubeBlack-periph
CubeGreen-solo          CubeNode                CubeNode-ETH
CubeOrange              CubeOrange-ODID         CubeOrange-SimOnHardWare
CubeOrange-bdshot       CubeOrange-joey         CubeOrange-periph
CubeOrange-periph-heavy CubeOrangePlus          CubeOrangePlus-ODID
CubeOrangePlus-SimOnHardWare CubeOrangePlus-bdshot CubePurple
CubeRedPrimary          CubeRedPrimary-PPPGW    CubeRedSecondary
CubeRedSecondary-IO     CubeSolo                CubeYellow
CubeYellow-bdshot       DAKEFPVF405             DAKEFPVH743
DAKEFPVH743Pro          DAKEFPVH743_SLIM        DevEBoxH7v2
DroneerF405             DrotekP3Pro             Durandal
Durandal-bdshot         F35Lightning            F4BY
F4BY_F427               F4BY_H743               FlyingMoonF407
FlyingMoonF427          FlyingMoonH743          FlysparkF4
FlywooF405HD-AIOv2      FlywooF405Pro           FlywooF405S-AIO
FlywooF745              FlywooF745Nano          FlywooH743Pro
FoxeerF405v2            FoxeerH743v1            FreeflyRTK
G4-ESC                  GEPRCF745BTHD           GEPRC_TAKER_H743
GreenSightUltraBlue     H757I_EVAL              H757I_EVAL_intf
HEEWING-F405            HEEWING-F405v2          HWH7
Here4AP                 Here4FC                 Hitec-Airspeed
HitecMosaic             HolybroF4_PMU           HolybroG4_Airspeed
HolybroG4_Compass       HolybroG4_GPS           HolybroGPS
IFLIGHT_2RAW_H7         JFB100                  JFB110
JHEMCU-GSF405A          JHEMCU-GSF405A-RX2      JHEMCU-H743HD
JHEMCUF405PRO           JHEMCUF405WING          JHEM_JHEF405
JPilot-C                KT-FMU-F1               KakuteF4
KakuteF4-Wing           KakuteF4Mini            KakuteF7
KakuteF7-bdshot         KakuteF7Mini            KakuteH7
KakuteH7-Wing           KakuteH7-bdshot         KakuteH7Mini
KakuteH7Mini-Nand       KakuteH7v2              LongBowF405WING
LumenierLUXF765-NDAA    MFE_AirSpeed_CAN        MFE_PDB_CAN
MFE_POS3_CAN            MFT-SEMA100             MUPilot
MambaF405-2022          MambaF405US-I2C         MambaF405v2
MambaH743v4             MatekF405               MatekF405-CAN
MatekF405-STD           MatekF405-TE            MatekF405-TE-bdshot
MatekF405-Wing          MatekF405-Wing-bdshot   MatekF405-bdshot
MatekF765-SE            MatekF765-Wing          MatekF765-Wing-bdshot
MatekG474               MatekG474-DShot         MatekG474-GPS
MatekG474-Periph        MatekH743               MatekH743-bdshot
MatekH743-periph        MatekH7A3               MatekH7A3-Wing
MatekL431               MatekL431-ADSB          MatekL431-APDTelem
MatekL431-AUAV          MatekL431-Airspeed      MatekL431-BattMon
MatekL431-BatteryTag    MatekL431-DShot         MatekL431-EFI
MatekL431-GPS           MatekL431-HWTelem       MatekL431-MagHiRes
MatekL431-Periph        MatekL431-Proximity     MatekL431-RC
MatekL431-Rangefinder   MatekL431-Serial        MatekL431-bdshot
MazzyStarDrone          MicoAir405Mini          MicoAir405v2
MicoAir743              MicoAir743-AIO          MicoAir743-Lite
MicoAir743v2            Morakot                 NarinFC-H5
NarinFC-H7              NarinFC-X3              Nucleo-G491
Nucleo-L476             Nucleo-L496             NucleoH743
NucleoH753ZI            NucleoH755              NxtPX4v2
OMNIBUSF7V2             ORBITH743               OmnibusNanoV6
OmnibusNanoV6-bdshot    OrqaF405Pro             OrqaH7QuadCore
PH4-mini                PH4-mini-bdshot         Pix32v5
PixC4-Jetson            PixFlamingo             PixFlamingo-F767
PixPilot-C3             PixPilot-V3             PixPilot-V6
PixPilot-V6PRO          PixSurveyA1             PixSurveyA1-IND
PixSurveyA2-IND         Pixhawk1                Pixhawk1-1M
Pixhawk1-1M-bdshot      Pixhawk1-bdshot         Pixhawk4
Pixhawk4-bdshot         Pixhawk5X               Pixhawk6C
Pixhawk6C-bdshot        Pixhawk6X               Pixhawk6X-ODID
Pixhawk6X-PPPGW         Pixhawk6X-bdshot        Pixracer
Pixracer-bdshot         Pixracer-periph         QioTekAdeptF407
QioTekZealotF427        QioTekZealotH743        QioTekZealotH743-bdshot
R9Pilot                 RADIX2HD                RadiolinkF405
RadiolinkPIX6           ReaperF745              ResoluteH7
SDMODELH7V1             SDMODELH7V2             SIYI_N7
SPEDIXF405              SPEDIXH743              SPRacingH7
SPRacingH7RF            SULILGH7-P1-P2          SVehicle-E2
SequreH743              Sierra-F405             Sierra-F412
Sierra-F9P              Sierra-L431             Sierra-PrecisionPoint
Sierra-TrueNavIC        Sierra-TrueNavPro       Sierra-TrueNavPro-G4
Sierra-TrueNorth        Sierra-TrueSpeed        SkyRukh_Surge_H7
SkySakuraH743           SkystarsF405v2          SkystarsH7HD
SkystarsH7HD-bdshot     SkystarsH7HDv2          SpeedyBeeF405AIO
SpeedyBeeF405Mini       SpeedyBeeF405WING       StellarF4
StellarF4V2             StellarH7V2             SuccexF4
Swan-K1                 TBS-Colibri-F7          TBS-L431
TBS-L431-Airspeed       TBS-L431-BattMon        TBS-L431-CurrMon
TBS-L431-PWM            TBS_LUCID_H7            TBS_LUCID_H7_WING
TBS_LUCID_H7_WING_AIO   TBS_LUCID_PRO           TMotorH743
VM-L431                 VM-L431-BMS             VM-L431-BatteryTag
VM-L431-Periph-Pico     VM-L431-SRV-Hub-4CHP    VRBrain-v51
VRBrain-v52             VRBrain-v54             VRCore-v10
VRUBrain-v51            VUAV-TinyV7             VUAV-V7pro
X-MAV-AP-H743r1         X-MAV-AP-H743v2         YARIV6X
YJUAV_A6                YJUAV_A6SE              YJUAV_A6SE_H743
YJUAV_A6Ultra           ZeroOneX6               ZeroOneX6_Air
ZubaxGNSS               airbotf4                crazyflie2
common (infra)          f103-ADSB               f103-Airspeed
f103-GPS                f103-HWESC              f103-QiotekPeriph
f103-RangeFinder        f103-Trigger            f303-GPS
f303-HWESC              f303-M10025             f303-M10070
f303-MatekGPS           f303-PWM                f303-TempSensor
f303-Universal          f405-MatekAirspeed      f405-MatekGPS
fmuv2                   fmuv3                   fmuv3-bdshot
fmuv5                   iomcu                   iomcu-dshot
iomcu-f103              iomcu-f103-8MHz-dshot   iomcu-f103-dshot
iomcu_f103_8MHz         include (infra)         kha_eth
luminousbee4            luminousbee5            mRo-M10095
mRoCANPWM-M10126        mRoCZeroOEMH7-bdshot    mRoControlZeroClassic
mRoControlZeroF7        mRoControlZeroH7        mRoControlZeroH7-bdshot
mRoControlZeroOEMH7     mRoKitCANrevC           mRoNexus
mRoPixracerPro          mRoPixracerPro-bdshot   mRoX21
mRoX21-777              mindpx-v2               mini-pix
modalai_fc-v1           omnibusf4               omnibusf4pro
omnibusf4pro-bdshot     omnibusf4pro-one        omnibusf4v6
rFCU                    rGNSS                   revo-mini
revo-mini-bdshot        revo-mini-i2c           revo-mini-i2c-bdshot
revo-mini-sd            scripts (infra)         skyviper-f412-rev1
skyviper-journey        skyviper-v2450          sparky2
speedybeef4             speedybeef4v3           speedybeef4v4
speedybeef4v5           sw-boom-f407            sw-nav-f405
sw-spar-f407            thepeach-k1             thepeach-r1
uav-dev-auav-g4         uav-dev-fc-um982        uav-dev-powermodule
uav-dev_m10s
```

---

## Appendix B: Key MCU Memory Maps

### STM32H743xx (primary Meridian target)
- Flash: 2 × 1 MB banks, 16 sectors × 128 KB each
- Flash in use for ArduPilot: sectors 0–13 (~1.75 MB), sectors 14–15 used for parameter storage
- DTCM: 128 KB at 0x20000000 (fastest, no DMA)
- SRAM1+2: 256 KB at 0x30000000 (DMA-safe)
- AXI SRAM: 512 KB at 0x24000000 (DMA-safe, used for SDMMC)
- SRAM3: 32 KB at 0x30040000
- SRAM4: 64 KB at 0x38000000
- ITCM: 64 KB at 0x00000400 (instruction cache, no DMA)
- **Total usable**: ~1055 KB

### STM32F405xx (budget Meridian target)
- Flash: 1 MB (512 KB × 2, or 1 MB single bank)
- SRAM1/2: 128 KB at 0x20000000 (DMA-safe)
- CCM: 64 KB at 0x10000000 (no DMA, fast)
- **Total**: 192 KB

### STM32L431xx (DroneCAN periph target)
- Flash: 256 KB
- SRAM1: 48 KB at 0x20000000
- SRAM2: 16 KB at 0x10000000
- **Total**: 64 KB

---

*Audit complete. 430 board directories cataloged. No Rust code written.*
