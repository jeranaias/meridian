# Meridian Audit Wave 2: RC Protocol + Telemetry Stack

**Source**: ArduPilot `libraries/AP_RCProtocol/`, `AP_RCTelemetry/`, `AP_Frsky_Telem/`, `AP_Hott_Telem/`, `AP_LTM_Telem/`, `AP_IBus_Telem/`, `AP_CRSF/`
**Date**: 2026-04-02
**Auditor**: Research pass, no Rust written

---

## Table of Contents

1. [AP_RCProtocol Architecture](#1-ap_rcprotocol-architecture)
2. [Protocol Backends — Full List](#2-protocol-backends--full-list)
3. [CRSF — Full Protocol Detail](#3-crsf--full-protocol-detail)
4. [SBUS — Full Protocol Detail](#4-sbus--full-protocol-detail)
5. [SRXL2 — Spektrum Bidirectional](#5-srxl2--spektrum-bidirectional)
6. [FPort — FrSky Combined RC+Telemetry](#6-fport--frsky-combined-rctelemetry)
7. [PPM — Pulse Position Modulation](#7-ppm--pulse-position-modulation)
8. [DSM/DSMX — Spektrum Satellite](#8-dsmdsmx--spektrum-satellite)
9. [IBUS — FlySky RC Protocol](#9-ibus--flyskyrc-protocol)
10. [SUMD — Graupner HoTT RC](#10-sumd--graupner-hott-rc)
11. [ST24 — Yuneec](#11-st24--yuneec)
12. [RC Channel Mapping](#12-rc-channel-mapping)
13. [Failsafe Detection — Per Protocol](#13-failsafe-detection--per-protocol)
14. [AP_RCTelemetry Architecture](#14-ap_rctelemetry-architecture)
15. [CRSF Telemetry](#15-crsf-telemetry)
16. [FrSky SPort Telemetry](#16-frsky-sport-telemetry)
17. [FrSky Passthrough Telemetry](#17-frsky-passthrough-telemetry)
18. [Hott Telemetry](#18-hott-telemetry)
19. [LTM (Lightweight Telemetry)](#19-ltm-lightweight-telemetry)
20. [IBus Telemetry](#20-ibus-telemetry)
21. [Meridian Port Notes](#21-meridian-port-notes)

---

## 1. AP_RCProtocol Architecture

### Frontend / Backend Model

`AP_RCProtocol` is a singleton frontend that owns an array of `AP_RCProtocol_Backend*` pointers, one slot per protocol enum value (up to `NONE`). All backends are allocated in `init()`.

```
AP_RCProtocol (frontend, singleton)
  ├── backend[PPMSUM]   → AP_RCProtocol_PPMSum
  ├── backend[IBUS]     → AP_RCProtocol_IBUS
  ├── backend[SBUS]     → AP_RCProtocol_SBUS (inverted=true, 100000 baud)
  ├── backend[SBUS_NI]  → AP_RCProtocol_SBUS (inverted=false, 100000 baud)
  ├── backend[FASTSBUS] → AP_RCProtocol_SBUS (inverted=true, 200000 baud)
  ├── backend[DSM]      → AP_RCProtocol_DSM
  ├── backend[SUMD]     → AP_RCProtocol_SUMD
  ├── backend[SRXL]     → AP_RCProtocol_SRXL
  ├── backend[SRXL2]    → AP_RCProtocol_SRXL2
  ├── backend[CRSF]     → AP_RCProtocol_CRSF
  ├── backend[FPORT2]   → AP_RCProtocol_FPort2 (inverted=true)
  ├── backend[ST24]     → AP_RCProtocol_ST24
  ├── backend[FPORT]    → AP_RCProtocol_FPort (inverted=true)
  ├── backend[DRONECAN] → AP_RCProtocol_DroneCAN
  ├── backend[GHST]     → AP_RCProtocol_GHST
  └── ... (MAVLinkRadio, SFML, UDP, FDM, Radio, IOMCU, EMLID_RCIO)
```

The maximum number of channels is defined as `MAX_RCIN_CHANNELS = 18`, minimum `MIN_RCIN_CHANNELS = 5`.

### Auto-Detection

Detection runs independently for **pulse input** and **byte (UART) input**. The system prefers UART when available:

**Pulse path (`process_pulse`):**
- If a protocol is already detected and locked, feed only that backend.
- Otherwise scan all backends in enum order (PPMSUM=0, IBUS=1, SBUS=2, ...).
- A backend increments `rc_frame_count` when it recognizes a valid frame.
- When `frame_count2 > frame_count`, that protocol becomes the detected one.
- Protocols with weak CRC require **3 good frames** before locking: PPMSUM, SBUS, SBUS_NI, FASTSBUS, DSM, FPORT, FPORT2, CRSF, GHST.
- Protocols with strong CRC/checksum lock on **1 frame**: IBUS, SUMD, SRXL, SRXL2, ST24.

**Byte path (`process_byte`):**
- Same logic but feeds `process_byte(byte, baudrate)` to all backends.
- Once detected via bytes, pulse input is disabled (`hal.rcin->pulse_input_enable(false)`).

**UART scanning (`check_added_uart`):**
- A serial port registered via `add_uart()` is scanned on each `update()` call.
- If no protocol detected within 1 second, the baud rate config rotates through:
  ```
  115200 8N1 non-inverted
  115200 8N1 inverted
  100000 8E2 inverted  (SBUS)
  200000 8E2 inverted  (FastSBUS, if enabled)
  416666 8N1 non-inverted  (CRSF)
  2000000 8N1 non-inverted  (CRSFv3 sticky rate)
  ```
- On power-loss recovery for CRSF, `is_rx_active()` returns false, causing baud re-negotiation.

**Re-search:**
- `should_search()` returns true when 200ms has elapsed without input, OR if MULTI_RECEIVER_SUPPORT option is set (allows multiple receivers).
- IOMCU always takes precedence when active.

### Key Limits
- `MAX_RCIN_CHANNELS = 18`
- `MIN_RCIN_CHANNELS = 5`
- `TELEM_TIME_SLOT_MAX = 20` (scheduler slots)

---

## 2. Protocol Backends — Full List

| Enum # | Name        | Class                      | Input Type       | Channels | Baud      | Notes |
|--------|-------------|----------------------------|------------------|----------|-----------|-------|
| 0      | PPMSUM      | AP_RCProtocol_PPMSum       | Pulse            | up to 18 | n/a       | Pulse-position sum |
| 1      | IBUS        | AP_RCProtocol_IBUS         | Pulse/Byte       | 14       | 115200    | FlySky |
| 2      | SBUS        | AP_RCProtocol_SBUS         | Pulse/Byte       | 16+2     | 100000    | inverted, 8E2 |
| 3      | SBUS_NI     | AP_RCProtocol_SBUS         | Byte             | 16+2     | 100000    | non-inverted variant |
| 4      | DSM         | AP_RCProtocol_DSM          | Pulse/Byte       | up to 16 | 115200    | 10 or 11-bit |
| 5      | SUMD        | AP_RCProtocol_SUMD         | Pulse/Byte       | up to 32 | 115200    | Graupner HoTT |
| 6      | SRXL        | AP_RCProtocol_SRXL         | Byte             | varies   | 115200    | Legacy Spektrum SRXL |
| 7      | SRXL2       | AP_RCProtocol_SRXL2        | Byte             | up to 32 | 115200    | Bidirectional |
| 8      | CRSF        | AP_RCProtocol_CRSF         | Byte             | up to 24 | 416666    | TBS / ELRS |
| 9      | ST24        | AP_RCProtocol_ST24         | Pulse/Byte       | varies   | 115200    | Yuneec |
| 10     | FPORT       | AP_RCProtocol_FPort        | Pulse/Byte       | 16       | 115200    | FrSky combined, inverted |
| 11     | FPORT2      | AP_RCProtocol_FPort2       | Pulse/Byte       | 16       | 115200    | FrSky v2, inverted |
| 12     | FASTSBUS    | AP_RCProtocol_SBUS         | Byte             | 16+2     | 200000    | High-speed SBUS |
| 13     | DRONECAN    | AP_RCProtocol_DroneCAN     | async (poll)     | up to 18 | CAN       | |
| 14     | GHST        | AP_RCProtocol_GHST         | Byte             | 12+      | 420000    | ImmersionRC Ghost |
| 15     | MAVLINK_RADIO | AP_RCProtocol_MAVLinkRadio | async (poll)   | 18       | MAVLink   | |
| 16     | JOYSTICK_SFML | AP_RCProtocol_Joystick_SFML | async         | 18       | SFML      | SITL only |
| 17     | UDP         | AP_RCProtocol_UDP          | async (poll)     | 18       | UDP       | SITL/Linux |
| 18     | FDM         | AP_RCProtocol_FDM          | async (poll)     | 18       | IPC       | SITL FDM |
| 19     | RADIO       | AP_RCProtocol_Radio        | async (poll)     | varies   | varies    | SiK radio |
| 20     | IOMCU       | AP_RCProtocol_IOMCU        | async (poll)     | 18       | SPI/IPC   | IOMCU coprocessor |
| 21     | EMLID_RCIO  | AP_RCProtocol_Emlid_RCIO   | async (poll)     | varies   | varies    | Emlid RCIO board |

---

## 3. CRSF — Full Protocol Detail

**Source files:** `AP_RCProtocol_CRSF.cpp/.h`, `AP_CRSF/AP_CRSF_Protocol.h`

### Wire Format

- Half-duplex single-wire UART
- 420000 baud (CRSF bootstrap standard) or 416666 (legacy) — configured at `CRSF_BAUDRATE = 416666U`, `ELRS_BAUDRATE = 420000U`
- 8 data bits, 1 stop bit, no parity, not inverted
- CRSFv3 can negotiate up to 2 Mbps (stored as sticky on soft reboot)
- Byte time: 21.43 µs at 416666 baud
- Max frame size: 64 bytes (`CRSF_FRAMELEN_MAX = 64`)

### Frame Structure

```
[device_address][length][type][payload...][crc8]
     1 byte       1 byte  1 byte            1 byte
```

- `device_address`: Target device (FC = `0xC8`)
- `length`: byte count from `type` through `crc8` inclusive
- `type`: frame type byte
- CRC: CRC-8/DVB-S2 over `type` + `payload` (excludes address and length bytes)

Header length is 2 bytes (`CRSF_HEADER_LEN = 2`). Full frame = `length + 2`.

### Device Addresses

| Address | Device |
|---------|--------|
| 0x00 | Broadcast |
| 0x10 | USB |
| 0x80 | TBS Core PnP Pro |
| 0xC0 | PnP Pro Current Sensor |
| 0xC2 | PnP Pro GPS |
| 0xC4 | TBS Blackbox |
| 0xC8 | Flight Controller (ArduPilot target) |
| 0xCE | VTX |
| 0xEA | Radio Transmitter |
| 0xEC | CRSF Receiver |
| 0xEE | CRSF Transmitter |

### Frame Types

| Value | Name | Direction | Notes |
|-------|------|-----------|-------|
| 0x02 | GPS | FC→TX | GPS position telemetry |
| 0x07 | VARIO | FC→TX | Vertical speed |
| 0x08 | BATTERY_SENSOR | FC→TX | Voltage/current/capacity |
| 0x09 | BARO_VARIO | FC→TX | Barometric altitude + vspeed |
| 0x0B | HEARTBEAT | FC→TX | Keepalive |
| 0x0F | VTX | FC→TX | VTX state |
| 0x10 | VTX_TELEM | TX→FC | VTX telemetry from receiver |
| 0x14 | LINK_STATISTICS | TX→FC | RSSI, LQ, SNR, power, antenna |
| 0x16 | RC_CHANNELS_PACKED | TX→FC | Standard 16ch × 11-bit RC frame |
| 0x17 | SUBSET_RC_CHANNELS_PACKED | TX→FC | CRSFv3 variable-resolution subset |
| 0x18 | RC_CHANNELS_PACKED_11BIT | TX→FC | Alternate 11-bit format |
| 0x1C | LINK_STATISTICS_RX | TX→FC | RX-side link stats |
| 0x1D | LINK_STATISTICS_TX | TX→FC | TX-side link stats |
| 0x1E | ATTITUDE | FC→TX | Pitch/roll/yaw angles |
| 0x21 | FLIGHT_MODE | FC→TX | Flight mode string |
| 0x28 | PARAM_DEVICE_PING | TX→FC | Device discovery |
| 0x29 | PARAM_DEVICE_INFO | FC→TX | Device info response |
| 0x2B | PARAMETER_SETTINGS_ENTRY | FC→TX | Parameter menu entry |
| 0x2C | PARAMETER_READ | TX→FC | Request parameter value |
| 0x2D | PARAMETER_WRITE | TX→FC | Write parameter value |
| 0x32 | COMMAND | Bidirectional | Sub-commands for FC, VTX, etc. |
| 0x7F | AP_CUSTOM_TELEM_LEGACY | FC→TX | ArduPilot custom (fw < 4.06) |
| 0x80 | AP_CUSTOM_TELEM | FC→TX | ArduPilot custom (reserved by TBS) |

### RC Channel Packing — Standard (0x16)

16 channels × 11 bits = 176 bits = 22 bytes payload.

Each channel value is 11 bits packed LSB-first into a contiguous bitstream. Channels are numbered 1–16 (internally 0–15).

**Scaling (TBS spec):** `TICKS_TO_US(x) = (x - 992) * 5/8 + 1500`

In code: `decode_11bit_channels(payload, CRSF_MAX_CHANNELS, channels, 5U, 8U, 880U)`

This gives:
- Raw 0 → ~330 µs (min)
- Raw 992 → 1500 µs (center)  
- Raw 1984 → ~2670 µs (max)
- Effective range: 172–1811 (raw) maps to ~988–2012 µs

`CRSF_DIGITAL_CHANNEL_MIN = 172`, `CRSF_DIGITAL_CHANNEL_MAX = 1811`

The scale factor `CRSF_RC_CHANNEL_SCALE_LEGACY = 0.62477120195241` converts raw to µs.

### RC Channel Packing — Subset/Variable (0x17, CRSFv3)

The first byte of payload is a bitfield:
```
bits [4:0] = starting_channel (which channel slot starts)
bits [6:5] = res_configuration (resolution mode)
bit  [7]   = digital_switch_flag
```

Resolution modes:
| Config | Bits | Mask | Scale |
|--------|------|------|-------|
| 0 | 10 | 0x03FF | 1.0 |
| 1 | 11 | 0x07FF | 0.5 |
| 2 | 12 | 0x0FFF | 0.25 |
| 3 | 13 | 0x1FFF | 0.125 |

Number of channels packed = `((frame_length - 2) * 8 - 5) / channelBits`

Channels are decoded and stored at `channels[starting_channel + n]`, allowing sparse subset updates without touching other channel slots.

Final value: `value = channelScale * raw + 988`

### Link Statistics Frame (0x14)

```c
struct LinkStatisticsFrame {
    uint8_t uplink_rssi_ant1;   // dBm * -1
    uint8_t uplink_rssi_ant2;   // dBm * -1
    uint8_t uplink_status;      // link quality percent (0–100)
    int8_t  uplink_snr;         // dB
    uint8_t active_antenna;     // 0=ant1, 1=ant2
    uint8_t rf_mode;            // enum, see RFMode below
    uint8_t uplink_tx_power;    // enum: 0mW=0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW
    uint8_t downlink_rssi;      // dBm * -1
    uint8_t downlink_status;    // downlink LQ percent
    int8_t  downlink_dnr;       // dB
};
```

RSSI conversion to ArduPilot scale (0–255):
- `rssi_dbm < 50` → 255 (excellent)
- `rssi_dbm > 120` → 0 (no signal)
- Linear interpolation: `255 * (1 - (rssi_dbm - 50) / 70)`

Alternatively the `USE_CRSF_LQ_AS_RSSI` option scales LQ: `lq * 2.5` (capped at 255).

### Link Statistics RX Frame (0x1C)

```c
struct LinkStatisticsRXFrame {
    uint8_t rssi_db;        // RSSI in dBm * -1
    uint8_t rssi_percent;   // 0–100
    uint8_t link_quality;   // 0–100
    int8_t  snr;            // dB
    uint8_t rf_power_db;    // dBm
};
```

### Link Statistics TX Frame (0x1D)

```c
struct LinkStatisticsTXFrame {
    uint8_t rssi_db;        // RSSI in dBm * -1
    uint8_t rssi_percent;   // 0–100
    uint8_t link_quality;   // 0–100
    int8_t  snr;            // dB
    uint8_t rf_power_db;    // dBm
    uint8_t fps;            // frames per second / 10
};
```

### RF Mode Table

```
CRSF modes (index 0–3):  4, 50, 150, 250 Hz
ELRS modes (index 4–18): 4, 25, 50, 100, 100, 150, 200, 250, 333, 500, 250, 500, 500, 1000, 50 Hz
```

Protocol detection: ELRS detected by `ProtocolType::PROTOCOL_ELRS` (set from device info response). CRSF Tracer hardcoded to 250 Hz.

### Command Frame (0x32)

```c
struct CommandFrame {
    uint8_t destination;
    uint8_t origin;
    uint8_t command_id;    // CommandID enum
    uint8_t payload[9];   // up to 8 bytes data + crc8
};
```

Command IDs: FC=0x01, Bluetooth=0x03, OSD=0x05, VTX=0x08, LED=0x09, General=0x0A, RX=0x10, ACK=0xFF

FC sub-commands: DISARM=0x01, SCALE_CHANNEL=0x02

RX sub-commands: BIND=0x01, CANCEL_BIND=0x02, SET_BIND_ID=0x03

General sub-commands include `CRSF_SPEED_PROPOSAL (0x70)` and `CRSF_SPEED_RESPONSE (0x71)` for baud rate negotiation.

### Baud Rate Negotiation (CRSFv3)

The TX sends `COMMAND_GENERAL_CRSF_SPEED_PROPOSAL` with a proposed baud rate. ArduPilot responds with `COMMAND_GENERAL_CRSF_SPEED_RESPONSE`. If accepted, both sides switch after flushing pending TX data + 4 ms delay.

Valid rates up to `CRSF_BAUDRATE_2MBIT = 2000000`. STM32H7 can go to 2 Mbps without DMA requirement; other MCUs require DMA.

After power loss, CRSF drops back to 416 kbps after receiving 200 incorrect characters. ArduPilot detects this via `is_rx_active()` → false, then rotates baud configs.

### Parameter Exchange

Extended frames (0x28–0x2D) carry destination+origin addressing. The parameter menu system supports up to 20 `TELEM_TIME_SLOT_MAX` scheduler slots. Parameter types: UINT8, INT8, UINT16, INT16, FLOAT, TEXT_SELECTION, STRING, FOLDER, INFO, COMMAND.

### CRSF Failsafe

CRSF has no explicit failsafe flag in the RC frame itself. Failsafe is inferred by timeout:
- `CRSF_TX_TIMEOUT = 500000 µs` (500 ms) — transmitter considered disconnected
- `CRSF_RX_TIMEOUT = 150000 µs` (150 ms) — receiver considered disconnected

`is_tx_active()`: last TX frame within 500 ms
`is_rx_active()`: last RX frame within 150 ms

When `is_tx_active()` is false, telemetry output changes behavior (reduced rates, omit certain frames).

---

## 4. SBUS — Full Protocol Detail

**Source file:** `AP_RCProtocol_SBUS.cpp/.h`

### Wire Format

- Inverted UART (active-high → logic 0)
- 100,000 baud (standard), 200,000 baud (FastSBUS variant)
- 8 data bits, **even parity**, **2 stop bits** (8E2)
- Frame size: 25 bytes exactly
- Frame interval: ~14 ms (Futaba), ~7 ms (some receivers)
- Frame gap detection: `HAL_SBUS_FRAME_GAP = 2000 µs` (2 ms)

### Frame Structure

```
Byte 0:    0x0F (header/start byte)
Bytes 1–22: 16 channels × 11 bits = 176 bits packed LSB-first into 22 bytes
Byte 23:   flags
Byte 24:   0x00 (end byte, not checked)
```

### Flags Byte (offset 23)

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | CH17 | Digital channel 17 (1998 µs = high, 998 µs = low) |
| 1 | CH18 | Digital channel 18 |
| 2 | FRAME_LOST | A frame was lost (single frame dropout, NOT failsafe) |
| 3 | FAILSAFE | Receiver is in failsafe mode (signal lost) |

### Channel Decoding

Uses the common `decode_11bit_channels()` with these parameters:
```
mult   = SBUS_TARGET_RANGE  = 1000
div    = SBUS_RANGE_RANGE   = 1600
offset = SBUS_SCALE_OFFSET  = 875
```

Raw SBUS range: 200–1800 → maps to 1000–2000 µs.

The formula: `value = (raw * 1000 / 1600) + 875`

Channels 17 and 18 (digital): extracted from bits 0–1 of the flags byte → 998 µs (low) or 1998 µs (high).

### Failsafe Logic

```c
if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) {
    sbus_failsafe = true;    // explicit failsafe
} else if (invalid_data) {
    sbus_failsafe = true;    // any of channels 1–4 at or below 875 µs
} else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) {
    sbus_failsafe = false;   // single frame lost, NOT failsafe
} else {
    sbus_failsafe = false;
}
```

Additional validity check: if any of the first 4 channels decode to <= `SBUS_SCALE_OFFSET` (875), the frame is treated as failsafe (protocol weakness guard).

Requires **3 good frames** before locking detection (weak protocol, no strong CRC).

### Soft Parsing (UART mode)

When SBUS arrives via UART byte-by-byte, `_process_byte()` accumulates bytes into a 25-byte buffer, enforcing:
1. First byte must be 0x0F
2. A frame gap >= 2 ms must precede the start byte
3. On full 25-byte accumulation, `sbus_decode()` is called

---

## 5. SRXL2 — Spektrum Bidirectional

**Source file:** `AP_RCProtocol_SRXL2.cpp/.h`

### Wire Format

- 115200 baud 8N1, not inverted
- Half-duplex (bidirectional on same wire)
- Max frame length: 80 bytes (`SRXL2_FRAMELEN_MAX`)
- Max channels: 32 (`SRXL2_MAX_CHANNELS`)
- Frame rate: ~91 Hz; telemetry reply rate: ~46 Hz (every other frame)

### Frame Structure

```
Byte 0: 0xA6 (SPEKTRUM_SRXL_ID — the sync byte)
Byte 1: packet type / ID byte
Byte 2: frame_length (total byte count)
Bytes 3..N-2: payload
Bytes N-1..N: CRC-16 (big-endian, covers entire frame except CRC itself)
```

### State Machine

Three states:
1. `STATE_IDLE` — waiting for sync byte 0xA6
2. `STATE_NEW` — got sync, buffering header
3. `STATE_COLLECT` — accumulating until `buflen == frame_len_full`

### Handshake / Bootstrap

On first frame where `packet_type == 0x21` (handshake frame), the device initializes using Spektrum's SRXL2 library (`srxlInitDevice`, `srxlInitBus`). A `device_id` is assigned.

Bootstrap requires detecting the handshake frame before any RC data is processed. If non-handshake frames arrive before bootstrap, they are dropped.

### Channel Scaling

Channels come from Spektrum library callback `capture_scaled_input()`. SRXL2 encodes channels as 16-bit unsigned (0–65532, center=32768):

```c
value = ((int32_t)(raw >> 5) * 1194) / 2048 + 903
```

Channel reordering matches DSM convention (Spektrum channels 0/1/2 → ArduPilot 2/0/1 = throttle/roll/pitch).

### Telemetry (Bidirectional)

Telemetry is sent during the RX's reply window after receiving an RC frame. `AP_Spektrum_Telem` handles telemetry content (battery, GPS, attitude). SRXL2 calls `send_on_uart()` within a strict timing window.

### Failsafe

Failsafe flag is passed from the Spektrum library through `capture_scaled_input(bool in_failsafe)`. The SRXL2 library detects loss via frame timeout and sets the failsafe state internally.

---

## 6. FPort — FrSky Combined RC+Telemetry

**Source file:** `AP_RCProtocol_FPort.cpp/.h`

### Wire Format

- Single wire, half-duplex
- 115200 baud, inverted
- Combines RC control frames (TX→RX→FC direction) with SPort telemetry (FC→RX direction)
- Frame gap detection: 2 ms between frames

### Control Frame (Type 0x00, length 0x19 = 25 bytes)

```
0x7E          header
0x19          length (25)
0x00          type = control
data[22]      16 channels × 11 bits packed (same encoding as SBUS)
flags         failsafe/framelost bits
rssi          0–50 range (receiver RSSI)
crc           sum of bytes 1..N-1 wrapped to 0xFF
0x00          end byte
```

Total frame size: 29 bytes (`FPORT_CONTROL_FRAME_SIZE`).

Failsafe bit: bit 3 of flags. Frame-lost bit: bit 2.

RSSI scaling: `scaled_rssi = min(frame.rssi * 5.1, 255)` (converts 0–50 → 0–255).

Channel scaling:
```
mult = CHAN_SCALE_FACTOR1 = 1000
div  = CHAN_SCALE_FACTOR2 = 1600
offset = CHAN_SCALE_OFFSET = 875
```
Identical to SBUS.

### Downlink Frame (Type 0x01, length 0x08 = 8 bytes)

```
0x7E          header
0x08          length
0x01          type = downlink
prim          0x00=null, 0x10=data, 0x30=read, 0x31=write
appid[2]      FrSky SPort app ID (little-endian)
data[4]       SPort payload
crc           checksum
0x00          end byte
```

This is the SPort telemetry request from the receiver to the FC. ArduPilot responds via the same UART within a strict timing window (< 2.5 ms after the control frame ends).

### Byte Stuffing

Bytes 0x7E and 0x7D are escaped: `[0x7D, byte XOR 0x20]`. The decoder handles unstuffing.

### Telemetry Response

When a downlink frame arrives:
1. `prim == 0x10` (DATA): FC replies with an SPort telemetry packet
2. `prim == 0x00` (NULL): FC replies unless consecutive frame limit is exceeded
3. `prim == 0x30/0x31` (READ/WRITE): Bidirectional SPort parameter access

The response format is 10 bytes (stuffed): `[0x08, 0x81, frame, appid_lo, appid_hi, data[4], crc]`.

---

## 7. PPM — Pulse Position Modulation

**Source file:** `AP_RCProtocol_PPMSum.cpp/.h`

### Wire Format

- Analog PWM/PPM signal on a single GPIO (capture via hardware timer)
- All channels encoded as sequential pulses in a single frame
- Sync gap: any pulse >= 2700 µs marks frame boundary

### Decoding

Each pulse = width_s0 + width_s1. If the total >= 2700 µs, it's the sync pulse. Otherwise, it's a channel pulse.

Valid channel range: 700–2300 µs. Values outside this range are ignored (this also filters SBUS on the same pin, which has pulses < 100 µs).

On sync detection: if accumulated channel count >= `MIN_RCIN_CHANNELS`, `add_input()` is called with the captured values.

After `MAX_RCIN_CHANNELS` channels, the accumulator is flushed and reset (desync protection).

**No failsafe mechanism** — PPM has no signal-loss indication. Failsafe relies entirely on timeout.

Requires 3 good frames for detection lock.

---

## 8. DSM/DSMX — Spektrum Satellite

**Source file:** `AP_RCProtocol_DSM.cpp/.h`

### Wire Format

- 115200 baud, 8N1, not inverted
- Frame size: 16 bytes
- Frame interval: 11 ms (DSM2) or 22 ms (DSMX)
- Frame gap for sync: >= 5 ms

### Frame Structure

```
Byte 0:    Fade count (number of dropped frames)
Byte 1:    System (protocol info, partial)
Bytes 2–15: 7 channels × 2 bytes each (big-endian)
```

No header byte. Synchronization is achieved by inter-frame timing (>= 5 ms gap).

### Channel Encoding

Each 16-bit value contains:
- Bits [15]: Phase bit (frame 1 vs frame 2 in 2-frame protocols)
- Bits [14:shift]: Channel number (4 bits)
- Bits [shift-1:0]: Channel value

**10-bit mode** (shift=10): `channel = (raw >> 10) & 0xF`, `value = raw & 0x3FF` → range 0–1023 → scaled × 2 = 0–2046

**11-bit mode** (shift=11): `channel = (raw >> 11) & 0xF`, `value = raw & 0x7FF` → range 0–2047

**Format detection heuristic:** After 5 frames, count which bit patterns consistently produce valid channel numbers. Prefer 11-bit if unambiguous, else fall back to 10-bit.

### Channel Scaling

```c
value = ((int32_t)(value - 1024) * 1000) / 1700 + 1500
```

For 10-bit mode, raw is first doubled. Midpoint is 1024 raw → 1500 µs. 100% deflection ≈ ±400 µs.

### Channel Reordering

DSM physical channels 0/1/2 → ArduPilot channels 2/0/1 (throttle/roll/pitch). Channels 3+ pass through unchanged.

### Multi-Frame Protocols

Some Spektrum TX send 2 frames (channels 1–7 in frame 1, channels 8–14 in frame 2). The phase bit distinguishes them. Values accumulate in `last_values[]` across frames.

### VTX Control

If the last 4 bytes of the frame match the Spektrum VTX control pattern (`0xE000E000` masked with `0xF000F000`), the VTX band/channel/power/pitmode is extracted and passed to `AP_VideoTX`.

### Failsafe

No explicit failsafe flag in the DSM frame. Format reset happens if inter-frame gap > 200 ms. Loss detection relies on external timeout only.

Requires 3 good frames for detection lock.

### Bind Procedure

Spektrum bind uses GPIO control of a dedicated power pin:
1. Power off satellite (HAL_GPIO_SPEKTRUM_PWR)
2. Hold RC pin high
3. Power on
4. Send 9 bind pulses (120 µs high/low) within 72 ms of power-on
5. Release GPIO to RX mode

---

## 9. IBUS — FlySky RC Protocol

**Source file:** `AP_RCProtocol_IBUS.cpp/.h`

### Wire Format

- 115200 baud, 8N1, not inverted
- Frame size: 32 bytes (`IBUS_FRAME_SIZE`)
- Frame gap: >= 2 ms

### Frame Structure

```
Byte 0:    0x20 (header byte 1)
Byte 1:    0x40 (header byte 2)
Bytes 2–29: 14 channels × 2 bytes each (little-endian, 12-bit values in lower 12 bits)
Bytes 30–31: checksum (two bytes)
```

### Channel Count

14 channels (`IBUS_INPUT_CHANNELS`). Values are 12-bit: `value = frame[pick] | (frame[pick+1] & 0x0F) << 8`

### Checksum

Running sum of all preceding bytes starting at 96 (initial value), plus the last two bytes of the frame, must equal 0xFFFF. This is an inverted checksum stored in the last 2 bytes.

### Failsafe

Detected by high nibble of certain channel bytes:
```c
if ((frame[3] & 0xF0) || (frame[9] & 0xF0)) {
    *ibus_failsafe = true;
}
```
The high nibbles of the 2nd and 5th channel data bytes are normally zero; non-zero indicates failsafe mode.

Requires 1 good frame for detection lock (strong checksum).

---

## 10. SUMD — Graupner HoTT RC

**Source file:** `AP_RCProtocol_SUMD.cpp`

### Wire Format

- 115200 baud, 8N1
- Variable frame length: 3 + (channels × 2) + 2 bytes
- Max channels: 32 (`SUMD_MAX_CHANNELS`)

### Frame Structure

```
Byte 0:    0xA8 (SUMD_HEADER_ID)
Byte 1:    status (0x01=normal, 0x81=failsafe)
Byte 2:    channel count N
Bytes 3..3+2N-1: N channels × 2 bytes big-endian
Last 2 bytes: CRC-16 (XModem polynomial 0x1021)
```

### Channel Scaling

Raw is 16-bit, range: 1400 × 4 to 1800 × 4 per the Graupner spec. Conversion to µs: `value_us = raw_value / 8` → typically 700–2250 µs. ArduPilot further scales within standard 1000–2000 range.

### Failsafe

Status byte 0x81 (`SUMD_ID_FAILSAFE`) explicitly signals failsafe. Timeout detection: if gap between bytes > 5 ms, state machine resets to UNSYNCED.

Requires 1 good frame for detection lock (CRC-16 is strong).

---

## 11. ST24 — Yuneec

**Source file:** `AP_RCProtocol_ST24.cpp`

### Frame Structure

```
Byte 0:    0x55 (ST24_STX1)
Byte 1:    0x55 (ST24_STX2)
Byte 2:    length
Byte 3:    packet type
payload... 
Last byte: CRC-8
```

CRC-8 polynomial: 0x07. Packet types include 24-channel RC and telemetry frames.

---

## 12. RC Channel Mapping

### Raw Protocol → ArduPilot Channel Numbers

All RC backends call `add_input(num_channels, values[], in_failsafe)`. The `values` array is indexed 0–N and stored as PWM microseconds. Channel 0 = first channel.

**ArduPilot channel convention** (via `RC_Channels`):
- Channel 1 (index 0) = Roll
- Channel 2 (index 1) = Pitch
- Channel 3 (index 2) = Throttle
- Channel 4 (index 3) = Yaw

**Protocol-specific remapping at ingestion:**

| Protocol | Physical order | After remapping |
|----------|---------------|-----------------|
| DSM/DSMX | Throttle=0, Roll=1, Pitch=2, Yaw=3+ | → index 2,0,1,3+ |
| SRXL2 | Same as DSM (Spektrum convention) | → index 2,0,1,3+ |
| SBUS | Roll=ch1, Pitch=ch2, Throttle=ch3, Yaw=ch4 | No remapping |
| CRSF | TBS convention: same as SBUS | No remapping |
| IBUS | FlySky convention | No remapping |
| FPort | SBUS-identical | No remapping |
| PPM | As-transmitted | No remapping |

**Mapping is then governed by `RC_Channel` parameters:**

Each of the 18 RC channels in ArduPilot has a configurable `FUNCTION` (e.g., `RCn_OPTION`). The primary flight control mapping is via `RC_Channels::get_roll_channel()`, `get_pitch_channel()`, `get_throttle_channel()`, `get_yaw_channel()`. These look up which physical channel index has been assigned each function (201=ROLL, 202=PITCH, 203=THROTTLE, 204=YAW).

**Default mapping** for a freshly configured system: channels 1/2/3/4 → roll/pitch/throttle/yaw. User can remap via `RCn_OPTION` parameters.

All channels are reported to upper layers as µs values (1000–2000 typical range, 700–2300 absolute limits).

---

## 13. Failsafe Detection — Per Protocol

| Protocol | Explicit FS Flag | Implicit FS | Timeout |
|----------|-----------------|-------------|---------|
| SBUS | Bit 3 of flags byte | Ch1–4 ≤ 875 µs | External |
| CRSF | None | None | 500 ms TX timeout |
| FPort | Bit 3 of flags byte | None | External |
| DSM | None | None | External (format reset at 200 ms) |
| SRXL2 | Library callback | Library timeout | Spektrum library |
| IBUS | High nibble of ch2/ch5 bytes | None | External |
| SUMD | Status byte 0x81 | None | 5 ms inter-byte gap |
| PPM | None | None | External only |
| ST24 | Protocol-specific | None | External |

**Common timeout:** If no valid frames arrive within 200 ms, `should_search()` returns true and re-detection starts. The main vehicle code has its own failsafe timers (typically 1–3 seconds) above this.

**Failsafe channels:** When `in_failsafe=true` is passed to `add_input()`, `rc_input_count` is NOT incremented (only `rc_frame_count` is). This prevents failsafe data from being used for control, while still logging frames.

The `IGNORE_FAILSAFE` RC option bypasses protocol-reported failsafe.

---

## 14. AP_RCTelemetry Architecture

**Source file:** `AP_RCTelemetry.h/.cpp`

### Design

Abstract base class for all telemetry backends. Uses a **Weighted Fair Queuing (WFQ)** scheduler with up to 20 time slots (`TELEM_TIME_SLOT_MAX = 20`).

Each scheduler slot has:
- `packet_weight` — higher weight = lower priority (WFQ denominator)
- `packet_min_period_ms` — minimum interval between consecutive transmissions of this slot type

### Scheduler Operation

`run_wfq_scheduler()` picks the highest-priority slot whose minimum period has elapsed. The WFQ formula: `priority = 1 / weight`. Slots with lower `weight` values get sent more frequently.

Derived classes must implement:
- `setup_wfq_scheduler()` — configure slot weights and periods
- `process_packet(idx)` — send the telemetry for slot `idx`
- `is_packet_ready(idx, queue_empty)` — optional readiness gating
- `adjust_packet_weight(queue_empty)` — dynamic priority adjustment

Status text messages are queued separately in a ring buffer (capacity 5 items) and consume a scheduler slot.

### What data is sent back (common across backends)

- Battery voltage, current, capacity, remaining %
- GPS: lat/lon, speed, altitude, satellite count, fix type
- Attitude: pitch, roll, yaw/heading (radians or degrees depending on protocol)
- Flight mode: string name
- Home distance and bearing
- Vertical speed (barometric climb rate)
- RSSI/link quality (echoed back where protocol supports it)
- Arming status
- EKF health / sensor status
- Status text messages

---

## 15. CRSF Telemetry

**Source file:** `AP_RCTelemetry/AP_CRSF_Telem.cpp/.h`

### Scheduler Default Rates

| Slot | Type | Weight | Min Period | Approx Rate |
|------|------|--------|-----------|-------------|
| HEARTBEAT | Heartbeat | 50 | 100 ms | 10 Hz |
| PARAMETERS | Param exchange | 5 | 20 ms | 50 Hz (when active) |
| BARO_VARIO | Altitude+vspeed | 50 | 200 ms | 5 Hz |
| VARIO | Vertical speed | 50 | 200 ms | 5 Hz |
| ATTITUDE | Pitch/roll/yaw | 50 | 120 ms | 8 Hz |
| VTX_PARAMETERS | VTX state | 200 | 1000 ms | 1 Hz |
| BATTERY | Voltage/current | 1300 | 500 ms | 2 Hz |
| GPS | Position | 550 | 280 ms | 3 Hz |
| FLIGHT_MODE | Mode string | 550 | 500 ms | 2 Hz |
| PASSTHROUGH | FrSky passthrough | 5000 | 100 ms | max 10 Hz |
| STATUS_TEXT | Text messages | 5000 | 500 ms | max 2 Hz |
| GENERAL_COMMAND | Commands | 5 | 20 ms | 50 Hz (when active) |
| VERSION_PING | Version query | 5 | 500 ms | 2 Hz (startup) |
| DEVICE_PING | Device ping | 5 | 100 ms | 10 Hz (TX loss) |

Rates are adjusted based on RF mode (high-speed vs low-speed links).

### Telemetry Frame Formats

**GPS Frame (0x02):**
```c
struct GPSFrame {
    int32_t latitude;   // degree / 10,000,000
    int32_t longitude;  // degree / 10,000,000
    uint16_t groundspeed;  // km/h / 100
    uint16_t gps_heading;  // degree / 100
    uint16_t altitude;     // meter + 1000m offset
    uint8_t satellites;
};
```

**Battery Frame (0x08):**
```c
struct BatteryFrame {
    uint16_t voltage;    // mV * 100 (i.e., decivolts)
    uint16_t current;    // mA * 100 (i.e., deciamps)
    uint8_t capacity[3]; // mAh (24-bit)
    uint8_t remaining;   // percent
};
```

**Attitude Frame (0x1E):**
```c
struct AttitudeFrame {
    int16_t pitch_angle;  // rad * 10000
    int16_t roll_angle;   // rad * 10000
    int16_t yaw_angle;    // rad * 10000
};
```

**Flight Mode Frame (0x21):**
```c
struct FlightModeFrame {
    char flight_mode[16];  // null-terminated string
};
```

**BaroVario Frame (0x09):**
```c
struct BaroVarioFrame {
    uint16_t altitude_packed;     // altitude above start
    int8_t vertical_speed_packed; // vertical speed
};
```

### Custom ArduPilot Telemetry (0x80 / 0x7F)

Three sub-types packed into the custom frame:

**Single Packet Passthrough (0xF0):**
```c
struct PassthroughSinglePacketFrame {
    uint8_t sub_type;   // 0xF0
    uint16_t appid;     // FrSky SPort app ID
    uint32_t data;      // FrSky SPort 32-bit value
};
```

**Multi-Packet Passthrough (0xF2):**
Up to 9 `{appid, data}` pairs in a single CRSF frame, for burst telemetry.

**Status Text (0xF1):**
```c
struct StatusTextFrame {
    uint8_t sub_type;   // 0xF1
    uint8_t severity;   // MAVLink severity
    char text[50];      // null-terminated
};
```

### Custom Telemetry Activation

Custom telemetry is disabled by default. Activated by RC_Channels `CRSF_CUSTOM_TELEMETRY` option. When active, it disables some standard FrSky SPort passthrough slots and substitutes CRSF equivalents.

### Parameter Exchange

When the TX requests parameters (user opens CRSF configuration menu):
1. TX pings with `PARAM_DEVICE_PING (0x28)`
2. FC responds with `PARAM_DEVICE_INFO (0x29)` containing firmware version and device name
3. TX sends `PARAMETER_READ (0x2C)` for each parameter
4. FC responds with `PARAMETER_SETTINGS_ENTRY (0x2B)` containing type, value, range, and display string
5. TX sends `PARAMETER_WRITE (0x2D)` to modify values

Lua-scripted menus can create custom hierarchical parameter menus via the `ScriptedMenu` system.

---

## 16. FrSky SPort Telemetry

**Source file:** `AP_Frsky_Telem/AP_Frsky_SPort.cpp/.h`

### Protocol

SPort is a polling bus. Each sensor has a physical ID. The transmitter (or receiver) broadcasts poll requests: `[0x7E, sensor_id]`. The polled sensor replies with a data frame.

**Poll/Data frames:**

Poll from TX: `0x7E [sensor_id]`
Response from sensor:
```
[0x10 or 0x00] [appid_lo] [appid_hi] [data byte 0..3] [crc]
```
- Frame type: `SPORT_DATA_FRAME = 0x10`
- Total: 8 bytes (before byte-stuffing: 0x7E and 0x7D are escaped)

### Sensor IDs Used by ArduPilot

| Sensor ID | Name | Data sent |
|-----------|------|-----------|
| 0x00 | VARIO (ID0) | Altitude (ALT_ID 0x0100), VSpeed (VARIO_ID 0x0110) |
| 0x02 | FAS (ID2) | Battery remaining (FUEL_ID 0x0600), Voltage (VFAS_ID 0x0210), Current (CURR_ID 0x020A) |
| 0x03 | GPS (ID3) | GPS lat/lon (GPS_LONG_LATI_FIRST_ID 0x0800), Speed (GPS_SPEED_ID), Altitude (GPS_ALT_ID), Course (GPS_COURS_ID) |
| 0x04 | RPM (ID4) | RPM if available |
| 0x1B | ACC (various) | Accelerometer data |

SPort polling is driven by the bus master (TX). ArduPilot's SPort code reads incoming bytes and replies to its own sensor IDs.

### GPS Lat/Lon Packing

GPS coordinates are packed into a 32-bit SPort value:
- Bits [0:1]: Hemisphere (lat/lon, N/S/E/W)
- Bits [2:30]: Coordinate value in degrees × 10^4 in minutes format
- The `send_latitude` toggle alternates between transmitting lat and lon

---

## 17. FrSky Passthrough Telemetry

**Source file:** `AP_Frsky_Telem/AP_Frsky_SPort_Passthrough.cpp/.h`

### Overview

Passthrough telemetry extends SPort by multiplexing complex FC data into 32-bit SPort values using app IDs in the `0x5000` range. This avoids the polling constraint of standard SPort sensors.

OpenTX/EdgeTX on the transmitter decodes passthrough app IDs and displays them on the screen.

### Passthrough App IDs and Bit Layouts

**0x5000 — Status Text**
Up to 4 ASCII chars per SPort packet. Multiple packets form a message. Status and severity bits.

**0x5001 — AP Status**
```
bits [4:0]   = flight mode (0–31)
bit  [5]     = simple mode
bit  [6]     = super-simple mode
bit  [7]     = flying flag
bit  [8]     = armed
bit  [9]     = battery failsafe
bit  [10]    = EKF failsafe
bit  [12]    = generic failsafe
bit  [13]    = fence present
bit  [14]    = fence breached
bits [18:19] = throttle %
bits [25:26] = IMU temperature
```

**0x5002 — GPS Status**
```
bits [3:0]   = satellite count (capped at 15)
bits [5:4]   = GPS fix type (0–3)
bits [13:6]  = HDOP (0–255)
bits [21:14] = advanced GPS status
bits [31:22] = altitude MSL (dm)
```

**0x5004 — Home**
```
bits [11:0]  = altitude above home (scaled)
bits [24:12] = home distance (m)
bits [31:25] = bearing to home (2-degree steps)
```

**0x5005 — Velocity and Yaw**
```
bits [8:0]   = horizontal speed (dm/s)
bits [16:9]  = vertical speed (scaled)
bits [27:17] = yaw (0.2-degree steps)
bit  [28]    = airspeed vs groundspeed toggle
```

**0x5006 — Attitude**
```
bits [10:0]  = roll (centidegrees scaled)
bits [20:11] = pitch (centidegrees scaled)
bits [31:21] = range finder distance
```

**0x5007 — Parameters** — configuration parameters sent on demand.

**0x5008 — Battery** — voltage (9-bit), current (7-bit), total mAh (15-bit), per battery instance.

**0x500A — RPM** — two RPM sensor values packed.

**0x500B — Terrain** — terrain altitude, health flag.

**0x500C — Wind** — wind angle and speed (apparent and real).

**0x500D — Waypoint** — waypoint number, distance, bearing.

**0x0800 — GPS Lat/Lon** — shared with standard SPort GPS.

### WFQ Scheduler (Passthrough)

17 packet types (`WFQ_LAST_ITEM = 17`). Weights and periods are balanced to maximize information throughput within the SPort polling rate (~10 Hz per sensor).

---

## 18. Hott Telemetry

**Source file:** `AP_Hott_Telem/AP_Hott_Telem.cpp/.h`

### Wire Format

- 19200 baud, 8N1, non-inverted, half-duplex
- Binary request/response protocol
- The transmitter (GR-12/GR-16) polls sensors with a 2-byte request
- 4 ms initial delay before reply; 1.2 ms inter-byte delay

### Request Format

```
byte 0: 0x80 (PROT_BINARY — binary mode indicator)
byte 1: sensor ID to query
```

Sensor IDs:
- `0x8E` (PROT_ID_EAM) — Electric Air Model
- `0x8A` (PROT_ID_GPS) — GPS module  
- `0x89` (PROT_ID_VARIO) — Variometer

### EAM Response (44 bytes + CRC)

```c
struct {
    uint8_t  start_byte = 0x7C;
    uint8_t  eam_sensor_id = 0x8E;
    uint8_t  warning_beeps;
    uint8_t  sensor_id = 0xE0;
    uint16_t alarms;
    uint8_t  cell_low[7];      // 0.02V steps per cell
    uint8_t  cell_high[7];
    uint16_t batt1_voltage;    // 100mV steps
    uint16_t batt2_voltage;    // 100mV steps
    uint8_t  temp1;            // 20=0°C, offset +20
    uint8_t  temp2;
    uint16_t altitude;         // meters, 500=0m offset
    uint16_t current;          // 0.1A steps
    uint16_t main_voltage;     // 0.1V steps
    uint16_t batt_used;        // 10mAh steps
    uint16_t climbrate;        // 0.01m/s, 30000=0m/s
    uint8_t  climbrate3s;      // m/3s, 120=0m/3s
    uint16_t rpm;              // 10 rev/min steps
    uint8_t  electric_min;
    uint8_t  electric_sec;
    uint16_t speed;            // km/h steps
    uint8_t  stop_byte = 0x7D;
    // + 1 byte CRC (sum of all bytes)
};
```

### GPS Response (44 bytes + CRC)

Position encoded as degrees-decimal-minutes (DDM):
- `pos_NS_dm`: degrees × 100 + minutes (e.g., 4830 = 48°30')
- `pos_NS_sec`: decimal seconds × 10000

Altitude: uint16, 500=0m offset.
Speed: km/h.
Home distance: meters.
Fix char: '2'=2D, '3'=3D, 'D'=DGPS, '-'=no fix.

### Vario Response (44 bytes + CRC)

Altitude (min/max/current), climbrate at 1s/3s/10s resolution, flight mode text (3 lines × 7 chars), yaw in 2-degree steps.

### CRC

Simple byte sum of all payload bytes (excluding start/stop bytes in some docs, but ArduPilot sums all bytes in `send_packet()`).

---

## 19. LTM (Lightweight Telemetry)

**Source file:** `AP_LTM_Telem/AP_LTM_Telem.cpp/.h`

### Wire Format

- 2400 baud (designed for extremely low-bandwidth links)
- One-way (FC→ground station), no polling
- 3 frame types in a round-robin schedule
- CRC: XOR of all bytes from offset 3 to N-2 (excludes header and CRC byte itself)

### Frame Schedule

Generated at 10 Hz (`generate_LTM()` called from IO timer every 100 ms):
- Odd ticks: A frame (attitude)
- Even non-modulo-4: G frame (GPS)
- Modulo-4: S frame (sensors)

This gives: A at 5 Hz, G and S each at 2.5 Hz.

### G Frame (GPS, 18 bytes)

```
$TG [lat:4] [lon:4] [speed:1] [alt:4] [sats_fix:1] [CRC:1]
```
- Lat/Lon: int32 in degrees × 10^-7 (1e7 scale), little-endian
- Speed: uint8 m/s
- Alt: int32 cm relative to home, little-endian
- sats_fix: bits [7:2] = satellite count, bits [1:0] = fix type (0–3)

### A Frame (Attitude, 10 bytes)

```
$TA [pitch:2] [roll:2] [heading:2] [CRC:1]
```
- Pitch: int16 degrees
- Roll: int16 degrees
- Heading: int16 degrees (all little-endian)

### S Frame (Sensors, 11 bytes)

```
$TS [vbat:2] [current:2] [rssi:1] [airspeed:1] [status:1] [CRC:1]
```
- vbat: uint16 millivolts
- current: uint16 hundredths of amps (cA)
- rssi: uint8 percent
- airspeed: uint8 m/s
- status bits [7:2] = flight mode, bit [1] = failsafe, bit [0] = armed

---

## 20. IBus Telemetry

**Source file:** `AP_IBus_Telem/AP_IBus_Telem.cpp/.h`

### Wire Format

- 115200 baud, 8N1, half-duplex on the same wire as IBUS RC input
- Polling protocol: the receiver queries sensors by ID
- Query interval: ~7 ms between polls
- Gap threshold for new message: 3 ms

### Packet Formats

**Incoming command (from receiver, 4 bytes):**
```c
struct CommandPacket {
    uint8_t message_length;  // 0x04
    uint8_t sensor_id : 4;   // 0–14 (max 15 sensors)
    uint8_t command : 4;     // 0x8=discover, 0x9=type, 0xA=value
    uint16_t checksum;       // 0xFFFF - sum of preceding bytes
};
```

**Response — Discover (4 bytes):**
```
[0x04, 0x8X, checksum_lo, checksum_hi]
```
(Echo back the discover command with the sensor present)

**Response — Type (6 bytes):**
```
[0x06, 0x9X, sensor_type, 0x00, checksum_lo, checksum_hi]
```
sensor_type = one of `IBUS_SENSOR_TYPE_*` constants.

**Response — 2-byte Value (6 bytes):**
```
[0x06, 0xAX, value_lo, value_hi, checksum_lo, checksum_hi]
```

**Response — 4-byte Value (8 bytes):**
```
[0x08, 0xAX, byte0, byte1, byte2, byte3, checksum_lo, checksum_hi]
```

Checksum: `0xFFFF - sum(all bytes except checksum)`.

### Supported Sensors (ordered by priority)

ArduPilot reports up to 15 sensors (receiver polling limit). Listed by type ID:

| Type ID | Name | Size | Units |
|---------|------|------|-------|
| 0x15 | ARMED | 2 | 0/1 |
| 0x16 | FLIGHT_MODE | 2 | mode enum |
| 0x0B | GPS_STATUS | 2 | fix<<4\|sats |
| 0x06 | FUEL | 2 | percent |
| 0x03 | EXTERNAL_VOLTAGE | 2 | centivolts |
| 0x83 | ALT | 4 | cm |
| 0x14 | GPS_DIST | 2 | meters |
| 0x09 | CLIMB_RATE | 2 | cm/s |
| 0x13 | GROUND_SPEED | 2 | cm/s |
| 0x0F | ROLL | 2 | centidegrees |
| 0x10 | PITCH | 2 | centidegrees |
| 0x11 | YAW | 2 | centidegrees |
| 0x7E | SPEED | 2 | km/h |
| 0x41 | TEMPERATURE_PRESSURE | 4 | combined |
| 0x07 | RPM | 2 | 0.01 percent |
| 0x05 | BATTERY_CURRENT | 2 | centiamps |
| 0x04 | AVERAGE_CELL_VOLTAGE | 2 | centivolts |
| 0x08 | COMPASS_HEADING | 2 | degrees |
| 0x0A | COG | 2 | centidegrees |
| 0x80 | GPS_LAT | 4 | 1e7 degrees |
| 0x81 | GPS_LNG | 4 | 1e7 degrees |
| 0x82 | GPS_ALT | 4 | cm |
| 0x0C | ACC_X | 2 | cm/s² |
| 0x0D | ACC_Y | 2 | cm/s² |
| 0x0E | ACC_Z | 2 | cm/s² |

Flight mode is mapped from ArduPilot mode numbers to IBus vehicle mode constants (Stabilize=0, Acro=1, AltHold=2, Auto=3, Guided=4, Loiter=5, RTL=6, Circle=7, PosHold=8, Land=9).

---

## 21. Meridian Port Notes

### Priority for Meridian

Meridian currently uses CRSF and SBUS. These are the highest-priority ports.

**CRSF port checklist:**
- [ ] Frame parser: `[addr][len][type][payload...][crc8_dvb_s2]`
- [ ] CRC-8/DVB-S2 implementation (polynomial 0xD5, not 0x07)
- [ ] Standard channel decoder: 16ch × 11-bit, formula `(raw * 5/8) + 880` → µs
- [ ] Subset channel decoder: variable bit-width, 5-bit starting channel + 2-bit resolution field
- [ ] Link statistics parser (both 0x14 and 0x1C/0x1D variants)
- [ ] Baud negotiation (CRSFv3): handle `CRSF_SPEED_PROPOSAL / RESPONSE` commands
- [ ] Failsafe by timeout only (500 ms TX timeout, 150 ms RX timeout)
- [ ] Telemetry write path: battery, GPS, attitude, flight mode, heartbeat frames
- [ ] Bootstrap baud: 416666 (standard) or 420000 (ELRS option)
- [ ] WFQ scheduler for telemetry output

**SBUS port checklist:**
- [ ] SoftSerial or inverted UART: 100000 baud, 8E2, inverted logic level
- [ ] Frame parser: 25 bytes, header=0x0F, 2 ms frame gap detection
- [ ] Channel decoder: 16ch × 11-bit using `decode_11bit_channels(mult=1000, div=1600, offset=875)`
- [ ] Digital channels 17/18 from flags byte bits 0–1
- [ ] Failsafe from flags bit 3; frame-lost from bit 2 (NOT failsafe)
- [ ] Secondary failsafe: any of ch1–4 at or below 875 µs
- [ ] Require 3 good frames before locking

**Common backend infrastructure:**
- [ ] `add_input(num_ch, values[], in_failsafe, rssi, lq)` equivalent
- [ ] `rc_frame_count` vs `rc_input_count` distinction (failsafe frames count frames but not inputs)
- [ ] Protocol detection state machine with 200 ms re-search window
- [ ] UART baud cycling if no protocol detected after 1 second

### Channel Mapping Note

CRSF and SBUS channels are delivered in the "natural" order (ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw in most TX configurations). DSM/Spektrum protocols deliver ch1=throttle, ch2=roll, ch3=pitch and need remapping.

Meridian should implement a configurable channel map (default passthrough) that can be overridden via config for DSM use.

### CRC-8/DVB-S2 Note

The CRSF CRC uses **CRC-8/DVB-S2** specifically:
- Initial value: 0x00
- Polynomial: 0xD5 (reflected: 0xAB)
- No input/output reflection
- No final XOR

The ArduPilot implementation is in `AP_Math/crc.h` as `crc8_dvb_s2()` and `crc8_dvb_s2_update()`.

### Telemetry Scheduler

For CRSF telemetry output, ArduPilot uses a WFQ scheduler with 14 telemetry types and rates tuned to the link rate (4–1000 Hz). At the slowest link (4 Hz), output drops to ~0.3 Hz per slot. At 500 Hz ELRS, passthrough runs at 8+ Hz.

For Meridian, the minimum viable telemetry is: battery (2 Hz), GPS (1 Hz), attitude (3–8 Hz), heartbeat (10 Hz). Flight mode and status text can be lower priority.

### Timing Constraints

- CRSF frame every 4 ms minimum (250 Hz link rate)
- Telemetry must be written within the same 4 ms window as the RC frame is received
- FPort telemetry must respond within 2.5 ms of the control frame ending (very tight)
- SBUS frames are 25 bytes at 100 kbaud = 2.5 ms transmission time; minimum inter-frame 14 ms (Futaba)

### Protocols Meridian Does NOT Currently Need

Lower priority — can be added later if needed:
- DSM/DSMX (Spektrum satellite)
- SRXL2 (Spektrum bidirectional)
- FPort (FrSky combined)
- SUMD (Graupner HoTT)
- ST24 (Yuneec)
- IBUS RC (FlySky)

Telemetry backends lower priority:
- Hott (Graupner proprietary, niche)
- LTM (low-bandwidth only, niche)
- IBus telemetry (FlySky ecosystem only)
- FrSky SPort/Passthrough (FrSky receivers, can add if needed)

---

*End of audit. Source files read: AP_RCProtocol.h/.cpp, AP_RCProtocol_Backend.h/.cpp, AP_RCProtocol_CRSF.h/.cpp, AP_CRSF_Protocol.h, AP_RCProtocol_SBUS.h/.cpp, AP_RCProtocol_DSM.h/.cpp, AP_RCProtocol_SRXL2.h/.cpp, AP_RCProtocol_FPort.h/.cpp, AP_RCProtocol_PPMSum.h/.cpp, AP_RCProtocol_IBUS.h/.cpp, AP_RCProtocol_SUMD.cpp, AP_RCProtocol_ST24.cpp, AP_RCTelemetry.h/.cpp, AP_CRSF_Telem.h/.cpp (partial), AP_Frsky_SPort.h/.cpp, AP_Frsky_SPort_Passthrough.h/.cpp (partial), AP_Hott_Telem.h/.cpp, AP_LTM_Telem.cpp, AP_IBus_Telem.h/.cpp.*
