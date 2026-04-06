# ArduPilot DroneCAN / CAN Bus Audit

**Purpose:** Document everything Meridian needs to implement CAN bus support, with primary focus on DroneCAN (UAVCAN v0) and AP_Periph ecosystem compatibility.

**Source audited:** `D:\projects\ardupilot` — 24 source files read across AP_CANManager, AP_DroneCAN, AP_Periph, AP_KDECAN, AP_PiccoloCAN, AP_HAL_ChibiOS, and sensor driver libraries.

---

## 1. CAN Bus Architecture (`libraries/AP_CANManager/`)

### Overview

`AP_CANManager` is the central manager for all CAN activity. It is a singleton (`AP_CANManager::get_singleton()`). On init it iterates over physical CAN interfaces (`HAL_NUM_CAN_IFACES`), assigns each to a protocol driver, and initializes both layers.

### Interface and Driver Counts

The counts are defined at board level via `hwdef.dat` scripts and default to 0 in `AP_HAL/AP_HAL_Boards.h`:

```
#define HAL_NUM_CAN_IFACES 0        // physical interfaces (hardware)
#define HAL_MAX_CAN_PROTOCOL_DRIVERS HAL_NUM_CAN_IFACES  // logical drivers
```

In practice ArduPilot supports up to **3 simultaneous physical CAN interfaces** (`P1_`, `P2_`, `P3_` parameter groups). Most Pixhawk-class boards have 1 or 2. High-end H7 boards (e.g., CubeOrange+) have 2. The `HAL_MAX_CAN_PROTOCOL_DRIVERS` is always equal to `HAL_NUM_CAN_IFACES`, so each physical interface gets exactly one protocol driver.

A single DroneCAN driver instance can, however, have **multiple physical interfaces attached to it** (redundant bus topology). That is how you get CAN1+CAN2 both running DroneCAN — they are registered under the same `AP_DroneCAN` driver instance.

### Baud Rates

Default baud rate is **1 Mbit/s** (set as the default parameter value `_bitrate = 1000000` in `AP_CANIfaceParams.cpp`). The parameter range is 10,000–1,000,000 bps. The STM32 bxCAN driver computes timings from `STM32_PCLK1` to achieve the requested rate, targeting an 87.5% sample point. Supported rates in the ecosystem: 125k, 250k, 500k, 1M.

On STM32H7/G4 boards with FDCAN, a separate `_fdbitrate` parameter controls the CAN FD data-phase baud rate (1–8 Mbit/s, parameter values: 1, 2, 4, 5, 8 in MHz). CANFD is opt-in via `Options::CANFD_ENABLED` bit.

### Protocol Detection / Selection

Protocol selection is **not automatic** — it is a parameter. Each driver slot has a `CAN_Dx_PROTOCOL` parameter (type `AP_CAN::Protocol`). Supported values at time of audit:

| Value | Protocol |
|-------|----------|
| 0 | None |
| 1 | DroneCAN (UAVCAN v0) |
| 4 | PiccoloCAN |
| 6 | EFI_NWPMU |
| 7 | USD1 (radar) |
| 8 | KDECAN |
| 10 | Scripting |
| 11 | Benewake |
| 12 | Scripting2 |
| 13 | TOFSenseP |
| 14 | RadarCAN |

Slots 2, 3, 5, 9 are reserved (previously used, do not reuse). When the manager allocates driver slots it reads `_drv_param[n]._driver_type` and instantiates the correct `AP_CANDriver` subclass. Secondary protocols (for 11-bit address frames on the same bus) are registered via `register_11bit_driver()`.

### Filter Configuration

Filtering is not exposed at the ArduPilot parameter level. It is done inside the HAL `CANIface::init()` — the bxCAN hardware has 14 filter registers. ArduPilot uses a **receive-all** filter (no ID filtering at hardware level); filtering is done in software inside each protocol driver's message dispatch loop.

### Logging

An optional `LOG_ALL_FRAMES` option (bit 0 of `CAN_Px_OPTIONS`) enables per-bus CAN frame logging to the dataflash log in `CANF`/`CAFD` message types.

---

## 2. DroneCAN (UAVCAN v0) — `libraries/AP_DroneCAN/`

### What is DroneCAN?

DroneCAN was formerly called UAVCAN v0 (protocol version 0). It uses Canard, a lightweight C implementation of the protocol, and DSDL-defined message types. ArduPilot exclusively implements UAVCAN v0 / DroneCAN — **there is no UAVCAN v1 support in the main tree**. "DroneCAN" and "UAVCAN" are used interchangeably in the codebase.

Key library: `libcanard` bundled at `libraries/AP_DroneCAN/canard/`. The DSDL-generated C headers are in `libraries/AP_DroneCAN/dsdl/`.

The C++ wrapper `AP_DroneCAN` adds publisher/subscriber/client/server objects using a Canard++ interface layer (`canard/publisher.h`, `canard/subscriber.h`, `canard/service_client.h`, `canard/service_server.h`).

### Memory Pool

DroneCAN requires a heap-like memory pool for message fragmentation and reassembly. Default pool sizes:

- Regular CAN: `DRONECAN_NODE_POOL_SIZE = 8192` bytes
- CANFD: `DRONECAN_NODE_POOL_SIZE = 16384` bytes

This pool is allocated at init time from the heap (`NEW_NOTHROW uint32_t[_pool_size/sizeof(uint32_t)]`). It is tunable via the `CAN_Dn_UC_POOL` parameter (range 1024–16384).

AP_Periph uses a separate static pool, `HAL_CAN_POOL_SIZE`:
- H7 boards / GPS moving baseline: 8000–16000 bytes
- F4/L4/G4 boards: 4000 bytes

### Node IDs

- Valid node IDs: **1–125** (126 and 127 are reserved by the protocol)
- Flight controller default: **node 10** (`AP_DRONECAN_DEFAULT_NODE = 10`)
- Peripheral nodes: assigned dynamically by the DNA server (see below), or set statically via parameter

### Dynamic Node Allocation (DNA) Server

The FC acts as a UAVCAN Dynamic Node ID Allocation server (`AP_DroneCAN_DNA_Server`). Each peripheral that boots without a static node ID broadcasts an allocation request containing its 16-byte hardware unique ID. The server:

1. Receives the request
2. Looks up the unique ID in its persistent database (stored in flash via `StorageManager`)
3. If previously seen: returns the same node ID
4. If new: assigns the next free ID
5. Broadcasts the allocation response

Database storage: 1024 bytes in flash, storing `NodeRecord` entries indexed by node ID (6-byte UID hash + 1-byte CRC per slot). Max nodes: 125.

The DNA server also monitors `uavcan.protocol.NodeStatus` from all nodes, requests `GetNodeInfo` responses for verification, and flags unhealthy or duplicate nodes via pre-arm checks.

Options that affect DNA behavior:
- `DNA_CLEAR_DATABASE` (bit 0): clear all stored node assignments on reboot
- `DNA_IGNORE_DUPLICATE_NODE` (bit 1): don't fail pre-arm on duplicates
- `DNA_IGNORE_UNHEALTHY_NODE` (bit 3): don't fail pre-arm on unhealthy nodes

### Peripheral Discovery

Discovery is purely reactive — the FC does not broadcast discovery requests. Peripherals announce themselves via `uavcan.protocol.NodeStatus` (sent at 1 Hz). The FC subscribes to `NodeStatus` and uses `GetNodeInfo` requests to retrieve hardware/software version and unique ID.

The `AP_DroneCAN::init()` method calls `subscribe_msgs()` on each sensor driver class (GPS, compass, baro, etc.) at startup. Those drivers register callbacks for the DSDL message types they care about. When a message arrives from any node on the bus, Canard dispatches it to all matching subscribers.

### Outgoing Messages (FC → Peripherals)

| DSDL Message | Purpose |
|---|---|
| `uavcan.equipment.esc.RawCommand` | ESC throttle commands (−8192 to +8191) |
| `uavcan.equipment.actuator.ArrayCommand` | Servo/actuator commands (normalized or PWM) |
| `uavcan.equipment.indication.LightsCommand` | RGB LED color |
| `uavcan.equipment.indication.BeepCommand` | Buzzer tone |
| `uavcan.equipment.gnss.RTCMStream` | RTCM corrections to GPS node |
| `ardupilot.indication.SafetyState` | Safety switch state |
| `uavcan.equipment.safety.ArmingStatus` | Arm/disarm state |
| `ardupilot.indication.NotifyState` | Vehicle state for LED/buzzer notification |
| `uavcan.protocol.NodeStatus` | FC's own heartbeat |
| `dronecan.protocol.CanStats` | CAN bus statistics |
| `dronecan.protocol.Stats` | Protocol statistics |
| `ardupilot.gnss.Heading` | GPS heading (GNSS send mode) |
| `uavcan.equipment.gnss.Fix2` | GPS fix re-broadcast (GNSS send mode) |
| `uavcan.equipment.hardpoint.Command` | Relay/hardpoint commands |
| `com.himark.servo.ServoCmd` | Himark servo commands (optional) |
| `com.hobbywing.esc.RawCommand` | Hobbywing ESC commands (optional) |

ESC commands use `CANARD_TRANSFER_PRIORITY_HIGH - 1` (highest priority). Servo/actuator commands use `CANARD_TRANSFER_PRIORITY_HIGH`. Status/LED/notify use `CANARD_TRANSFER_PRIORITY_LOW`.

### Incoming Messages (Peripherals → FC)

| DSDL Message | Driver |
|---|---|
| `uavcan.equipment.gnss.Fix2` | `AP_GPS_DroneCAN` |
| `uavcan.equipment.gnss.Auxiliary` | `AP_GPS_DroneCAN` |
| `ardupilot.gnss.Heading` | `AP_GPS_DroneCAN` |
| `ardupilot.gnss.Status` | `AP_GPS_DroneCAN` |
| `ardupilot.gnss.MovingBaselineData` | `AP_GPS_DroneCAN` |
| `ardupilot.gnss.RelPosHeading` | `AP_GPS_DroneCAN` |
| `uavcan.equipment.ahrs.MagneticFieldStrength` | `AP_Compass_DroneCAN` |
| `uavcan.equipment.air_data.StaticPressure` | `AP_Baro_DroneCAN` |
| `uavcan.equipment.air_data.StaticTemperature` | `AP_Baro_DroneCAN` |
| `uavcan.equipment.air_data.RawAirData` | `AP_Airspeed_DroneCAN` |
| `uavcan.equipment.range_sensor.Measurement` | `AP_RangeFinder_DroneCAN` |
| `uavcan.equipment.power.BatteryInfo` | `AP_BattMonitor_DroneCAN` |
| `ardupilot.equipment.power.BatteryInfoAux` | `AP_BattMonitor_DroneCAN` |
| `uavcan.equipment.esc.Status` | `AP_DroneCAN` ESC telem backend |
| `uavcan.equipment.esc.StatusExtended` | `AP_DroneCAN` extended ESC telem |
| `uavcan.equipment.actuator.Status` | `AP_Servo_Telem` |
| `ardupilot.indication.Button` | Safety button |
| `ardupilot.equipment.trafficmonitor.TrafficReport` | ADSB traffic |
| `uavcan.protocol.debug.LogMessage` | Debug text from node |
| `dronecan.protocol.FlexDebug` | Scripting debug data |

---

## 3. AP_Periph — CAN Peripheral Firmware (`Tools/AP_Periph/`)

### What is AP_Periph?

AP_Periph is an ArduPilot-based firmware that runs **on the peripheral MCU itself** — not on the flight controller. It converts an STM32 microcontroller with attached sensors into a DroneCAN peripheral node. The FC communicates with it over CAN as if it were a commercial GPS/compass/ESC node.

This is the mechanism by which products like Here3 GPS, ARK GPS, Matek GPS, Holybro DroneCAN nodes, and many others work. They all run AP_Periph (or a compatible implementation).

### Supported MCUs

Per `Tools/AP_Periph/README.md`, all ArduPilot STM32 targets are supported:
- STM32F1xx (e.g., F103 — minimal feature set)
- STM32F3xx
- STM32F4xx
- STM32F7xx
- STM32H7xx
- STM32L4xx (e.g., **STM32L431** — very common for CAN GPS/compass nodes)
- STM32G4xx (e.g., **STM32G474** — increasingly used for ESC/sensor nodes)

**Memory footprint for STM32L431 (3DR-L431 target):**
- Flash: 256 KB total, 64 KB reserved for bootloader+storage, ~192 KB available for firmware
- RAM: STM32L431 has 64 KB SRAM
- CAN pool: 6000 bytes (L431 uses bxCAN, non-FD)
- Parameter storage: 1800 bytes in flash

**Memory footprint for STM32G474 (MatekG474 target):**
- Flash: 512 KB total, 36 KB reserved for bootloader+storage
- RAM: STM32G474 has 128 KB SRAM
- CAN pool: 6000 bytes (G474 has FDCAN, but pool is same default)

**Can AP_Periph run on 64 KB flash / 32 KB RAM?** Likely not in its full form. The STM32L431 target (64 KB RAM, 256 KB flash) is considered minimal — feature sets are trimmed by `#if AP_PERIPH_xxx_ENABLED` guards. An STM32F103 with 128 KB flash can run a stripped AP_Periph with only GPS or only compass. 32 KB RAM would be extremely tight — a bare minimum DroneCAN node (single sensor, no GCS, minimal features) might fit, but the 4000-byte CAN pool alone plus stack plus code would be challenging.

### What AP_Periph Can Provide

Each feature is individually compile-time gated. A node can provide any combination:

| Function | Source File | DroneCAN Messages Sent |
|---|---|---|
| GPS (incl. RTK) | `gps.cpp` | `Fix2`, `Auxiliary`, `ardupilot.gnss.Status`, `Heading`, `MovingBaselineData`, `RelPosHeading` |
| Magnetometer | `compass.cpp` | `ahrs.MagneticFieldStrength` (or `dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes`) |
| Barometer | `baro.cpp` | `air_data.StaticPressure`, `air_data.StaticTemperature` |
| Airspeed | `airspeed.cpp` | `air_data.RawAirData` (differential pressure + temperature) |
| Rangefinder | `rangefinder.cpp` | `range_sensor.Measurement` |
| Battery monitor | `battery.cpp` | `power.BatteryInfo`, `ardupilot.equipment.power.BatteryInfoAux` (cell voltages) |
| ADSB | `adsb.cpp` | `ardupilot.equipment.trafficmonitor.TrafficReport` |
| RC output (ESC/servo) | `rc_out.cpp` | Receives `esc.RawCommand` / `actuator.ArrayCommand`; outputs PWM/DShot |
| RC input | `rc_in.cpp` | Sends RC input over CAN to FC |
| IMU | `imu.cpp` | `ardupilot.equipment.imu.static` |
| EFI | `efi.cpp` | EFI status |
| Proximity | `proximity.cpp` | Proximity sensor data |
| Temperature sensor | `temperature.cpp` | Temperature data |
| Safety LED/switch | Multiple | `ardupilot.indication.Button` |
| Buzzer | `buzzer.cpp` | Receives `BeepCommand` |
| LED (RGB/NeoPixel) | Various | Receives `LightsCommand` |
| RPM sensor | `rpm.cpp` | RPM stream |
| MSP bridge | `msp.cpp` | MSP protocol passthrough |
| Networking | `networking.cpp` | Ethernet/UDP passthrough |
| Serial tunnel | `serial_tunnel.cpp` | DroneCAN serial passthrough |
| Relay | `relay.cpp` | Receives `hardpoint.Command` |
| KDE ESC telem | `can.cpp` + AP_KDECAN | ESC telemetry bridge |
| Battery balance | `batt_balance.cpp` | Balance monitor |

### How AP_Periph Differs from Main Firmware

- No autopilot logic (no navigation, no control loops, no GCS MAVLink by default)
- No `AP_Vehicle` base class — runs `AP_Periph_FW` directly
- Uses raw Canard C API directly (not the C++ driver layer used by main fw)
- Single main loop calling `can_update()` which calls `processTx()` / `processRx()`
- Parameter storage in flash (not EEPROM). Parameters: `CAN_NODE`, `CAN_BAUDRATE`, `CAN_PROTOCOL`, sensor-specific params
- Watchdog support for automatic recovery
- Bootloader stored in ROMFS, can be self-updated over CAN via `FLASH_BOOTLOADER=1` parameter
- Firmware update over CAN using `uavcan.protocol.file.BeginFirmwareUpdate` (MissionPlanner or DroneCAN GUI Tool)
- CAN termination resistor can be software-controlled (`CAN_TERMINATE` parameter) or via hardware switch GPIO

### AP_Periph CAN Stack

AP_Periph uses the raw Canard C API (not the ArduPilot C++ wrapper layer). It maintains a `dronecan_protocol_t` struct with:
- `CanardInstance canard` — the canard state machine
- `canard_memory_pool[HAL_CAN_POOL_SIZE/sizeof(uint32_t)]` — static memory pool
- `tid_map` linked list — transfer ID tracking per (data_type_id, transfer_type, src, dst) tuple
- DNA allocation state variables

The `shouldAcceptTransfer` callback is the software filter — it checks incoming data_type_ids against all enabled features and accepts only matching types.

---

## 4. KDECAN — `libraries/AP_KDECAN/`

### Protocol

KDECAN is a proprietary KDE Direct ESC protocol over CAN. It uses **29-bit extended CAN frames** (not DroneCAN protocol). Frame IDs encode:

```
[bits 31–27] priority (5 bits)
[bits 26–24] unused (3 bits)
[bits 23–16] source_id (8 bits)
[bits 15–8]  destination_id (8 bits)
[bits 7–0]   object_address (8 bits)
```

Autopilot node ID = 0. Broadcast ID = 1. ESC IDs start at 2 (ESC #1 = node 2, etc.). Max 8 ESCs.

### Object Addresses

| Address | Object |
|---|---|
| 0 | ESC_INFO |
| 1 | SET_PWM |
| 2 | VOLTAGE |
| 3 | CURRENT |
| 4 | RPM |
| 5 | TEMPERATURE |
| 6 | GET_PWM_INPUT |
| 7 | GET_PWM_OUTPUT |
| 8 | MCU_ID |
| 9 | UPDATE_NODE_ID |
| 10 | ENUM |
| 11 | TELEMETRY |

### Telemetry

KDECAN telemetry frame (object address 11, 8 bytes):

| Bytes | Data | Scale |
|---|---|---|
| 0–1 | Voltage | × 0.01 V |
| 2–3 | Current | × 0.01 A |
| 4–5 | Electrical RPM | × 60 × 2 / num_poles → mechanical RPM |
| 6 | Temperature | × 100 → °C × 100 (centi-degrees) |

Telemetry is requested at 100 ms interval. The number of motor poles is configurable (`KDECAN_NPOLE` parameter, default 14) for RPM conversion.

### Integration

KDECAN is registered as an 11-bit auxiliary driver on the DroneCAN interface (not a standalone protocol driver in newer code — protocol enum value = 8 `AP_CAN::Protocol::KDECAN`). It is also supported inside AP_Periph as a passthrough (AP_Periph can receive KDE ESC telemetry and re-broadcast it as DroneCAN ESC status).

---

## 5. PiccoloCAN — `libraries/AP_PiccoloCAN/`

### Protocol

PiccoloCAN is the Currawong Engineering (Velocity Series) ESC and servo protocol over CAN. It supports:
- Up to 16 ESCs (`PICCOLO_CAN_MAX_NUM_ESC = 16`)
- Up to 16 servos (`PICCOLO_CAN_MAX_NUM_SERVO`)
- ECU (electronic control unit / engine control)
- Cortex (Currawong flight controller integration)

Unlike DroneCAN, PiccoloCAN uses **Currawong-specific DSDL-like packets** (`piccolo_protocol/` directory). It is a full standalone driver (`AP_CANDriver` subclass), not built on Canard.

### ESC Telemetry

`AP_PiccoloCAN_ESC` tracks three status message types:
- `ESC_StatusA_t`: includes RPM, software inhibit/hardware inhibit flags
- `ESC_StatusB_t`: voltage (× 0.01 V), current (× 0.01 A), ESC temperature, motor temperature
- `ESC_StatusC_t`: FET temperature

Telemetry reported to `AP_ESC_Telem_Backend`: RPM, voltage, current, temperature (max of ESC temp and FET temp), motor temperature.

### What's Unique

PiccoloCAN is unique in that it supports both ESC **and servo** control under a single protocol, plus ECU (engine/generator) telemetry. The Currawong Cortex controller adds more advanced integration. Update rates are independently configurable per device type (ESC Hz, servo Hz, ECU Hz).

---

## 6. CAN Hardware

### STM32 bxCAN (F1, F3, F4, F7, L4)

The STM32 bxCAN peripheral is used on most ArduPilot-supported flight controller MCUs. ArduPilot drives it directly through `libraries/AP_HAL_ChibiOS/bxcan.hpp` and `CanIface.cpp`.

Key bxCAN characteristics:
- 3 transmit mailboxes
- 2 receive FIFOs with 14 configurable filter registers
- Operates at up to 1 Mbit/s
- **Not** CAN FD capable
- Baud rate computed from APB1 clock (`STM32_PCLK1`)

### STM32 FDCAN (H7, G4)

The STM32H7 and STM32G4 families have the Bosch MCAN (M_CAN) core, marketed as FDCAN. ArduPilot uses this via `CANFDIface.h` / `CANFDIface.cpp`, using `FDCAN_GlobalTypeDef` directly.

FDCAN supports:
- Classical CAN frames at up to 1 Mbit/s
- CAN FD frames with data-phase baud rates up to 8 Mbit/s
- Up to 64 bytes per frame (classical CAN: 8 bytes)
- Hardware message RAM filtering (more flexible than bxCAN's 14 filters)

ArduPilot enables FDCAN mode per-driver via the `CANFD_ENABLED` option bit. When FDCAN is active, the Canard layer is aware (`canfdout()`) and uses longer frames where beneficial.

### Linux / SITL

ArduPilot on Linux and SITL uses **SocketCAN** (`AP_HAL_Linux/CANSocketIface.cpp`, `AP_HAL_SITL/CANSocketIface.cpp`). This interfaces with Linux kernel `vcan` or physical CAN via `can0`/`can1` interfaces. The SITL implementation allows testing DroneCAN peripherals in software using virtual CAN sockets.

### CAN Transceivers

ArduPilot's software does not specify a particular transceiver — that is a board design decision. Common transceivers in the ecosystem:
- **SN65HVD230** — 3.3V, popular on STM32L4/G4 AP_Periph nodes
- **TJA1051T/3** — 5V, used on many Pixhawk-class boards
- **MCP2551** — 5V, older designs
- **ISO1042** — isolated transceiver for high-noise environments

### CAN Termination

The CAN bus requires 120-ohm termination at each end of the bus (two terminating resistors total — one at each physical endpoint). AP_Periph firmware supports software-controlled termination via:

- `CAN_TERMINATE` / `CAN2_TERMINATE` / `CAN3_TERMINATE` parameters (0 or 1)
- Hardware switch override via `HAL_GPIO_PIN_GPIO_CANx_TERM_SWITCH`
- LED indicator via `HAL_GPIO_PIN_GPIO_CANx_TERM_LED`

The FC typically has hard-soldered termination. Peripheral nodes that are bus endpoints (e.g., a GPS at the far end of the harness) should enable termination. Nodes in the middle of a daisy chain should not terminate.

---

## 7. Sensor Integration via DroneCAN

### GPS

**DSDL messages:** `uavcan.equipment.gnss.Fix2` (primary), `uavcan.equipment.gnss.Auxiliary`, `ardupilot.gnss.Status`, `ardupilot.gnss.Heading`

`Fix2` fields relevant to Meridian:
- `timestamp.usec` — microsecond timestamp (UTC epoch once locked)
- `longitude_deg_1e8`, `latitude_deg_1e8` — position × 1e8
- `height_ellipsoid_mm`, `height_msl_mm` — altitude in mm
- `ned_velocity[3]` — NED velocity in m/s
- `sats_used` — satellite count
- `status` — NO_FIX / 2D_FIX / 3D_FIX
- `mode` — SINGLE / DGPS / RTK
- `sub_mode` — RTK_FLOAT / RTK_FIXED / etc.
- `covariance[6]` — position and velocity covariance (float16, diagonal of 3×3 each)

Auxiliary fields: HDOP, VDOP.

`ardupilot.gnss.Status`: healthy flag, armable flag, error codes.

RTK support: RTCM corrections flow **from FC to GPS node** via `uavcan.equipment.gnss.RTCMStream`. Moving baseline uses `ardupilot.gnss.MovingBaselineData` and `ardupilot.gnss.RelPosHeading` for dual-GPS heading.

Fix type enum mapping:
- `UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX` = 0
- `UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX` = 1
- `UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX` = 2

ArduPilot GPS type parameters: `GPS_TYPE_UAVCAN` (3), `GPS_TYPE_UAVCAN_RTK_BASE` (10), `GPS_TYPE_UAVCAN_RTK_ROVER` (11).

### Compass / Magnetometer

**DSDL message:** `uavcan.equipment.ahrs.MagneticFieldStrength`

Fields:
- `magnetic_field_ga[3]` — X/Y/Z raw field strength in **Gauss** (float16)

AP_Periph sends raw mag field (not processed heading). The FC applies calibration, declination, and orientation transforms. Update rate: capped at `AP_PERIPH_MAG_MAX_RATE = 25 Hz` in AP_Periph to avoid flooding the bus.

Hi-res variant: `dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes` (full float32, not float16). Available on AP_Periph when `AP_PERIPH_MAG_HIRES` is set.

### Barometer

**DSDL messages:** `uavcan.equipment.air_data.StaticPressure` + `uavcan.equipment.air_data.StaticTemperature`

Separate messages for pressure and temperature (not combined). Fields:
- `static_pressure` — Pascals (float32)
- `static_pressure_variance` — currently sent as 0
- `static_temperature` — Kelvin (float32)
- `static_temperature_variance` — currently sent as 0

The FC side (`AP_Baro_DroneCAN`) receives both messages and converts temperature from Kelvin back to Celsius for internal use. Altitude is computed by the FC from pressure (not transmitted).

### Airspeed

**DSDL message:** `uavcan.equipment.air_data.RawAirData`

Fields used:
- `differential_pressure` — Pa (measured differential pressure from Pitot tube)
- `static_air_temperature` — Kelvin (ambient temperature if available)
- `pitot_temperature` — Kelvin (Pitot probe temperature if separate sensor present)
- Unused fields set to NaN: `static_pressure`, `static_pressure_sensor_temperature`, `differential_pressure_sensor_temperature`

AP_Periph sends corrected (calibrated) differential pressure. Update rate: max 20 Hz.

### Rangefinder

**DSDL message:** `uavcan.equipment.range_sensor.Measurement`

Fields:
- `sensor_id` — instance/address of the sensor
- `sensor_type` — LIDAR / SONAR / RADAR / UNDEFINED
- `reading_type` — VALID_RANGE / TOO_CLOSE / TOO_FAR / UNDEFINED
- `range` — distance in meters (float32)

AP_Periph sends one message per instance per update. Rate is configurable via `RNGFND_MAX_RATE` parameter. Max instances: `RANGEFINDER_MAX_INSTANCES`.

### ESC Commands and Telemetry

**Command:** `uavcan.equipment.esc.RawCommand`

- Array of int16 values, one per ESC
- Range: −8192 to +8191
- Value 0 = stop; 8191 = full forward throttle
- Negative values used for reversible ESCs (`ESC_RV` bitmask)
- The FC sends the command at highest priority

**Telemetry received by FC:** `uavcan.equipment.esc.Status`

Fields:
- `error_count` — cumulative error counter
- `voltage` — V
- `current` — A
- `temperature` — Kelvin
- `rpm` — signed RPM
- `power_rating_pct` — 0–127 (optional)
- `esc_index` — which ESC in the array

Extended telemetry: `uavcan.equipment.esc.StatusExtended` adds additional fields.

**Servo:** `uavcan.equipment.actuator.ArrayCommand` with `COMMAND_TYPE_UNITLESS` (normalized −1 to +1) or `COMMAND_TYPE_POSITION` (position in radians). Telemetry via `uavcan.equipment.actuator.Status`.

### Battery

**DSDL messages:** `uavcan.equipment.power.BatteryInfo` + `ardupilot.equipment.power.BatteryInfoAux`

`BatteryInfo` fields:
- `battery_id` — instance/serial number identifier
- `voltage` — total pack voltage (V)
- `current` — charge current (A, positive = charging)
- `temperature` — Kelvin
- `state_of_charge_pct` — 0–100%
- `state_of_health_pct` — 0–100% (or `UNKNOWN = 127`)
- `model_instance_id` — instance index
- `model_name` — string name

`BatteryInfoAux` (ArduPilot extension) fields:
- `battery_id` — matches BatteryInfo
- `voltage_cell[]` — individual cell voltages (V, float16 array)
- `max_current` — current from main BatteryInfo
- `timestamp.usec` — timestamp

AP_Periph sends `BatteryInfoAux` only when the battery monitor has cell voltage data. Both messages are sent at 10 Hz.

### LED / Lighting

**DSDL message:** `uavcan.equipment.indication.LightsCommand`

The FC sends an array of light commands, each with:
- `light_id` — which LED/group
- `color.red`, `color.green`, `color.blue` — 5-bit RGB values (0–31)

AP_Periph receives this and drives NeoPixel LEDs, NCP5623, Toshiba LED controllers, etc.

### Safety Switch / Safety State

**FC → peripherals:** `ardupilot.indication.SafetyState`
- `status` — `SAFETY_ON` (locked), `SAFETY_OFF` (armed/unlocked)

**Peripherals → FC:** `ardupilot.indication.Button`
- `button` — button index
- `press_time` — duration in 10 ms units
- Reports safety button presses to the FC

### ADSB Traffic

`ardupilot.equipment.trafficmonitor.TrafficReport` — AP_Periph receives ADSB traffic reports over serial (from Ping-compatible receivers) and re-broadcasts them as DroneCAN messages. The FC subscribes and populates its traffic monitor.

---

## 8. What Meridian Needs

### Why CAN Matters

CAN bus is the primary peripheral connection for modern autopilot hardware. The commercial ecosystem is built around DroneCAN:
- **GPS:** Here3, ARK GPS, Holybro DroneCAN GPS, Matek M10 CAN — all DroneCAN
- **ESC:** HobbyWing Platinum V4, KDE UAS series, T-Motor, Myxa — DroneCAN or KDE/PiccoloCAN
- **Compass:** embedded in GPS nodes (all the above have onboard compass)
- **Battery:** Tattu Smart batteries, APD ESC monitors — DroneCAN
- **Airspeed:** Matek AS-CAN, Holybro airspeed — DroneCAN

A Meridian autopilot without DroneCAN support cannot use any of these devices. This is the primary compatibility gap vs. ArduPilot.

### Current State of Meridian

Meridian has no CAN-related code at this time. The `meridian-hal`, `meridian-drivers`, and `meridian-platform-stm32` crates exist but have minimal implementations (just `#![no_std]`). There is no `socketcan`, `embedded-can`, or `bxcan` dependency in `Cargo.toml`.

### Rust CAN Library Options

Three relevant Rust crates exist for CAN support:

**1. `bxcan` crate (crates.io)**
- Rust driver for STM32 bxCAN peripheral
- Covers F0, F1, F3, F4, L4 (not H7/G4 FDCAN)
- Works with `embedded-hal` traits
- Well-maintained, widely used in the Rust embedded community

**2. `socketcan` crate (crates.io)**
- Linux SocketCAN bindings
- Needed for the `meridian-platform-linux` and `meridian-sitl` crates
- Enables testing with `vcan0` virtual interfaces and real CAN hardware on Linux

**3. `embedded-can` crate (crates.io)**
- Trait definitions for CAN bus in embedded contexts
- Part of the `embedded-hal` ecosystem
- Provides `Frame`, `Id`, `ExtendedId`, `StandardId` traits
- This is the abstraction layer — actual drivers (`bxcan`, FDCAN drivers) implement these traits

For FDCAN (STM32H7/G4), no dominant Rust crate exists yet. The `fdcan` crate is in development but not as mature as `bxcan`. Meridian may need a thin HAL wrapper calling STM32H7-HAL directly.

### DroneCAN in Rust

Two approaches for implementing DroneCAN in Meridian:

**Option A: Port/wrap `libcanard`**

The ArduPilot approach — use the existing C library (`libcanard v0`) via FFI. Pros: proven, complete DSDL implementation, direct compatibility. Cons: unsafe FFI, C memory model, harder to test in Rust, potential alignment issues on no_std.

**Option B: Use `canadensis` (Rust UAVCAN v1)**

`canadensis` (crates.io) is a pure-Rust UAVCAN v1 implementation. UAVCAN v1 (Cyphal) is a breaking redesign — **not compatible with DroneCAN/UAVCAN v0**. Most hardware currently ships with UAVCAN v0. Using canadensis would only interoperate with UAVCAN v1 devices (rare today).

**Option C: Implement a minimal DroneCAN v0 layer in Rust**

Write a Rust crate implementing the Canard protocol framing (CRC, multi-frame reassembly, transfer ID management) + DSDL message encoding/decoding for the subset of messages Meridian needs. This is feasible for the key message types (GPS Fix2, ESC RawCommand, ESC Status, BatteryInfo, MagneticFieldStrength). DSDL encoding is documented and deterministic.

**Recommended approach for Meridian:**

Start with Option A (FFI to libcanard) for fastest path to interoperability, while building a Rust abstraction layer above it. Wrap canard behind a `DroneCan` trait in `meridian-drivers` so the FFI is isolated and replaceable. Long-term, evaluate a pure Rust Canard v0 implementation as a replacement once protocol behavior is well-understood.

### Architecture for Meridian CAN Support

```
meridian-platform-stm32/
    can_driver.rs        -- bxcan or FDCAN driver, implements embedded-can traits

meridian-platform-linux/
    can_driver.rs        -- socketcan driver

meridian-drivers/
    dronecan/
        mod.rs           -- DroneCan trait + node manager
        dna_client.rs    -- Dynamic Node Allocation client (not server — we're a FC)
        dna_server.rs    -- DNA server (allocates IDs to peripherals)
        messages/
            gps.rs       -- Fix2, Auxiliary, ardupilot.gnss.* codecs
            compass.rs   -- MagneticFieldStrength codec
            baro.rs      -- StaticPressure, StaticTemperature codecs
            airspeed.rs  -- RawAirData codec
            battery.rs   -- BatteryInfo, BatteryInfoAux codecs
            esc.rs       -- RawCommand encoder, Status decoder
            actuator.rs  -- ArrayCommand encoder, Status decoder
            range.rs     -- Measurement decoder
            indication.rs-- LightsCommand, BeepCommand, SafetyState encoders
            node.rs      -- NodeStatus, GetNodeInfo service
```

### Minimum Viable DroneCAN for Meridian

For initial ecosystem compatibility, implement in this priority order:

1. **DNA server** — required for any dynamic node (everything commercial)
2. **GPS Fix2 receive** — navigation data (most critical sensor)
3. **ESC RawCommand send** — motor control (required for flight)
4. **NodeStatus broadcast** — required by protocol (announces FC presence)
5. **GetNodeInfo service** — standard node discovery
6. **BatteryInfo receive** — power monitoring
7. **MagneticFieldStrength receive** — compass
8. **StaticPressure + StaticTemperature receive** — barometer
9. **ESC Status receive** — ESC telemetry
10. **SafetyState + ArmingStatus send** — arm/disarm propagation to peripherals

Items 11+ (airspeed, rangefinder, LED, ADSB, KDECAN, PiccoloCAN) can follow in later work.

### Key Implementation Details from ArduPilot

- **Node ID range:** 1–125; reserve 126–127 (broadcast/anonymous)
- **FC default node ID:** 10 (configurable)
- **Memory pool:** minimum 4000 bytes for F4-class, 8192 for H7/FDCAN
- **Transfer priorities:** ESC commands at highest, servo commands at high, status/monitoring at low
- **ESC command range:** −8192 to +8191 (int16)
- **Servo command range:** −1.0 to +1.0 normalized, or radians for position
- **Baro sends pressure in Pascals** (not altitude — altitude computation is on the FC)
- **Mag sends field in Gauss** (not heading — heading computation is on the FC)
- **Battery temperature in Kelvin** over the wire; convert to Celsius on receive
- **Covariance in Fix2 is float16** — watch for NaN encoding of large/infinite values
- **DSDL tail byte:** last byte of each CAN frame encodes transfer ID and start/end flags; the Canard library handles this transparently

---

*Audited from ArduPilot source — 24 files read. Date: 2026-04-02.*
