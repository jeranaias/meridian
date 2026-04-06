# Panel Round 2 — Wave 5 Re-Review

**Date**: 2026-04-02  
**Scope**: Targeted re-verification of specific findings from Wave 1 panel reviews.  
**Personas**: Lorenz Meier (#17), Brandon Jones (#19), Travis Schlosser (#16), Steve Doll (#20)

---

## Persona 17 — Lorenz Meier (MAVLink Protocol)

**Round 1 findings under re-review:**
- TIMESYNC (msg 111) handler echo correctness
- MAV_CMD_REQUEST_MESSAGE command number (512 vs. 520)
- SET_MESSAGE_INTERVAL (511) handling
- MISSION_CURRENT (42) streaming at 1 Hz
- Sensor mask bit values: GPS=0x08, DIFF_PRESSURE=0x10

---

### Finding 1: TIMESYNC (111) Echo — FIXED

`server.rs` lines 501–514 implement the handler:

```rust
InboundCommand::Timesync { tc1, ts1 } => {
    if *tc1 == 0 {
        let our_time_ns = (now_ms as i64) * 1_000_000;
        let n = self.adapter.encode_timesync(*ts1, our_time_ns, remaining);
        offset += n;
    }
    (offset, None)
}
```

The protocol is correct: when `tc1 == 0` (GCS-originated sync request), the response encodes `tc1 = their ts1` (echo) and `ts1 = our time_ns`. This is the exact QGC/MAVLink TIMESYNC handshake. `adapter.rs:434–448` confirms `MSG_TIMESYNC = 111` and the 16-byte payload layout is correct. The `v2.rs` CRC extra byte for msg 111 is set to 34, which matches the MAVLink specification.

**Rating: FIXED**

---

### Finding 2: MAV_CMD_REQUEST_MESSAGE = 512 — FIXED (with test artifact noted)

`adapter.rs` line 530:

```rust
512 => { // MAV_CMD_REQUEST_MESSAGE
    let msg_id = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) as u32;
    Some(InboundCommand::Command { command, result: CommandAction::RequestMessage(msg_id) })
}
```

The wire parsing correctly routes command `512` to `CommandAction::RequestMessage`. The correct constant is 512, not 520.

**Test artifact**: `server.rs` line 1497 has `command: 520` in `test_request_message_action`. This test bypasses the adapter entirely and constructs an `InboundCommand::Command` directly, so the `command` field is only used for encoding the COMMAND_ACK reply — dispatch is driven by the `result: CommandAction::RequestMessage(148)` variant. The test does not test wire parsing; it tests the server's action dispatch. The `520` value is a copy-paste error in the test and should be `512`, but it does not affect runtime correctness. The actual parse path at `512` is correct.

**Rating: FIXED** (test should be corrected to `command: 512` for clarity)

---

### Finding 3: SET_MESSAGE_INTERVAL (511) Handled — FIXED

`adapter.rs` line 537:

```rust
511 => { // MAV_CMD_SET_MESSAGE_INTERVAL
    let msg_id = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) as u32;
    let interval_us = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) as i64;
    Some(InboundCommand::Command {
        command,
        result: CommandAction::SetMessageInterval { msg_id, interval_us }
    })
}
```

`server.rs` line 150 defines `ServerAction::SetMessageInterval { msg_id, interval_us }` and `server.rs` line 637 confirms dispatch to `ServerAction::SetMessageInterval`. The handler correctly extracts `msg_id` (param 1) and `interval_us` (param 2) from the COMMAND_LONG payload using little-endian float-to-int conversion, consistent with MAVLink parameter encoding. The per-message-rate negotiation QGC performs on initial connect will now succeed.

**Rating: FIXED**

---

### Finding 4: MISSION_CURRENT (42) Streamed at 1 Hz — FIXED

`server.rs` lines 314–327:

```rust
// ─── MISSION_CURRENT at 1 Hz ───
if self.last_mission_current_ms == NEVER_SENT
    || now_ms.wrapping_sub(self.last_mission_current_ms) >= 1000
{
    let n = self.adapter.encode_mission_current(
        vehicle_state.mission_current_seq,
        remaining,
    );
    offset += n;
    self.last_mission_current_ms = now_ms;
}
```

`adapter.rs` lines 451–462 confirm `encode_mission_current` encodes msg ID 42 as a 2-byte little-endian `seq` field. `v2.rs` line 45: `MSG_MISSION_CURRENT = 42`. `v2.rs` line 77: CRC extra byte for msg 42 is 28, matching the specification. The `VehicleState` struct carries `mission_current_seq: u16` for caller update. The `NEVER_SENT` sentinel ensures the first update fires immediately.

**Rating: FIXED**

---

### Finding 5: Sensor Mask Bit Values — VERIFIED CORRECT

`adapter.rs` lines 54–61:

```rust
pub const MAV_SYS_STATUS_SENSOR_3D_GYRO: u32           = 0x01;
pub const MAV_SYS_STATUS_SENSOR_3D_ACCEL: u32          = 0x02;
pub const MAV_SYS_STATUS_SENSOR_3D_MAG: u32            = 0x04;
pub const MAV_SYS_STATUS_SENSOR_GPS: u32               = 0x08;
pub const MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE: u32 = 0x10;
pub const MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE: u32 = 0x20;
pub const MAV_SYS_STATUS_AHRS: u32                     = 0x200000;
pub const MAV_SYS_STATUS_PREARM_CHECK: u32             = 0x4000000;
```

GPS = 0x08 and DIFF_PRESSURE = 0x10 are correct per the MAVLink common.xml MAV_SYS_STATUS_SENSOR enum. These are used in `encode_sys_status` via `sensors.present_mask()` and `sensors.health_mask()`.

**Rating: VERIFIED CORRECT**

---

### Overall Rating: FIXED

All five items verified. The MAVLink layer now correctly implements the QGC connection handshake sequence. Remaining open issues from Wave 1 (MISSION_ITEM float format, MISSION_SET_CURRENT, float-coordinate uploads, geofence/rally protocols) were not in scope for this re-review and remain outstanding.

---

## Persona 19 — Brandon Jones (Motors / Battery)

**Round 1 findings under re-review:**
- BattMonitorAnalog exists at `crates/meridian-drivers/src/battery_analog.rs`
- Has voltage/current scaling, mAh integration, remaining%
- `octa_dji_x` is fixed (not duplicate of `octa_cw_x`)

---

### Finding 1: BattMonitorAnalog at battery_analog.rs — FIXED

`crates/meridian-drivers/src/battery_analog.rs` exists (454 lines). Confirmed structure:

- `BattMonitorConfig`: `voltage_pin`, `current_pin`, `voltage_scale`, `current_scale`, `voltage_offset`, `current_offset` — all present, defaults matching MatekH743 board config.
- `BatteryState`: `voltage`, `current_amps`, `consumed_mah`, `remaining_pct`, `healthy`, `voltage_mv`, `current_ca` — all present.
- `BattMonitorAnalog::update()`: applies scale factors `(adc_v - offset) * scale`, applies IIR low-pass filter (alpha=0.1, ~1s time constant at 10 Hz, matching ArduPilot), integrates current in amp-seconds and converts to mAh (`/ 3.6`), computes remaining% via both capacity-based and voltage-curve-based paths.
- MAVLink-ready integer representations: `voltage_mv = voltage * 1000` as u16, `current_ca = current_amps * 100` as i16.
- Cell count auto-detection from initial voltage (1S–14S, LiPo chemistry).
- `reset_consumed()` for battery swap handling.
- 9 unit tests covering initialization, cell detection, current integration, voltage filtering, remaining% (both modes), MAVLink units, and negative clamping.

**Rating: FIXED**

---

### Finding 2: octa_dji_x Not Duplicate of octa_cw_x — FIXED

`crates/meridian-mixing/src/lib.rs` lines 419–449:

`octa_dji_x()` (lines 423–434) now uses DJI motor index ordering:
```
(22.5, CCW), (-22.5, CW), (-67.5, CCW), (-112.5, CW),
(-157.5, CCW), (157.5, CW), (112.5, CCW), (67.5, CW)
```

`octa_cw_x()` (lines 438–449) uses clockwise positional ordering:
```
(22.5, CCW), (67.5, CW), (112.5, CCW), (157.5, CW),
(-157.5, CCW), (-112.5, CW), (-67.5, CCW), (-22.5, CW)
```

These are distinct. The DJI-X variant correctly places Motor 8 at slot 1 (−22.5°) following the AP_MotorsMatrix.cpp lines 935–948 DJI index-based insertion order. The two functions are no longer byte-for-byte identical. The comment at line 419–422 documents the DJI motor ordering rationale. Both functions are independently tested via the `test_frame_motor_counts` test at lines 1317–1318.

**Rating: FIXED**

---

### Overall Rating: FIXED

Both blocking issues resolved. `BattMonitorAnalog` is a faithful translation of `AP_BattMonitor_Analog.cpp` with appropriate scale factors, filtering, and integration. `octa_dji_x` now maps to correct DJI motor ordering. Remaining open items from Wave 1 (DShot timer ARR not configured, `spin_min`/`spin_max` not applied inside `mix_compensated`) were not in scope for this re-review.

---

## Persona 16 — Travis Schlosser (DroneCAN)

**Round 1 findings under re-review:**
- DNA 3-round handshake FSM exists in `crates/meridian-can/src/node.rs`
- DNA allocates downward from 125

---

### Finding 1: DNA 3-Round Handshake FSM — FIXED (struct-level)

`node.rs` lines 237–377 implement the complete FSM:

- `DnaHandshakePhase` enum: `Idle`, `WaitRound1`, `WaitRound2`, `WaitRound3`, `Complete`, `Failed` — all six phases present.
- `DnaHandshake` struct: `phase`, `unique_id: [u8; 16]`, `bytes_received`, `allocated_id`, `last_rx_us`, `retries`, `max_retries: 5`, `round_timeout_us: 500_000`.
- `start_round1()`: receives first 6 bytes, transitions `Idle → WaitRound1`.
- `receive_round2()`: guards on `WaitRound1`, appends bytes 6–11, transitions `WaitRound1 → WaitRound2`.
- `receive_round3()`: guards on `WaitRound2`, appends bytes 12–15, calls `dna.allocate()`, transitions `WaitRound2 → Complete` or `→ Failed`.
- `check_timeout()`: per-round 500 ms timeout with retry count, resets to `Idle` on timeout (up to 5 retries), transitions to `Failed` after `max_retries`.

The FSM correctly represents the UAVCAN v0 specification Section 5.6 three-round anonymous handshake.

**Qualification**: The FSM is a correct data structure and state machine. Per the Wave 1 finding that remains open, this FSM is not yet wired into `process_rx` — there is no code in `handle_message` that catches incoming `DNA_ALLOCATION_DTID` frames and calls `start_round1` / `receive_round2` / `receive_round3`. The FSM exists and is correct; the bus integration is absent. The re-review question asks only whether the FSM exists — it does.

**Rating: FIXED** (FSM present and correct; wire integration remains an open gap from Wave 1)

---

### Finding 2: DNA Allocates Downward from 125 — VERIFIED CORRECT

`DnaServer::new()` at line 101–107:

```rust
pub const fn new() -> Self {
    Self {
        entries: [DnaEntry::empty(); MAX_NODE_ID as usize],
        next_free_id: MAX_NODE_ID, // allocate downward from 125, matching ArduPilot
    }
}
```

`MAX_NODE_ID = 125` (line 31). `allocate()` at lines 121–154 starts from `next_free_id` and decrements: `self.next_free_id = if id <= MIN_NODE_ID { MAX_NODE_ID } else { id - 1 }`. The loop decrements `id` each pass. First allocation receives 125 (skipping 10, the FC's own ID). Wrap-around correctly resets to `MAX_NODE_ID` when `id` reaches `MIN_NODE_ID`. This matches ArduPilot's DNA allocation direction exactly.

**Rating: VERIFIED CORRECT**

---

### Overall Rating: FIXED

Both verification items pass. The 3-round handshake FSM is a complete, correct implementation of UAVCAN v0 Section 5.6. DNA allocation direction is downward from 125. The bus integration gap (FSM not wired into `process_rx`) and other Wave 1 issues (flash persistence not called, `GetNodeInfo` response decode absent, `GetSet` service absent) remain open but were not in scope for this re-review.

---

## Persona 20 — Steve Doll (Remote ID Compliance)

**Round 1 findings under re-review:**
- `AuthenticationMessage` struct exists in `crates/meridian-opendroneid/src/lib.rs`
- ASTM wire format encoding stubs exist
- Per-type message timers (not shared round-robin)

---

### Finding 1: AuthenticationMessage Struct — FIXED

`lib.rs` lines 107–152:

```rust
pub struct AuthenticationMessage {
    pub auth_type: AuthType,
    pub page_number: u8,
    pub last_page_index: u8,
    pub timestamp: u32,
    pub auth_data: [u8; 23],
    pub auth_data_len: u8,
}
```

`AuthType` enum at lines 110–118 covers all five F3411-22a Table 2 values: `None`, `UasIdSignature`, `OperatorIdSignature`, `MessageSetSignature`, `NetworkRemoteId`, `SpecificAuthentication`. Multi-page support is present (`page_number`, `last_page_index`, max 5 pages × 23 bytes = 115 bytes). `AuthenticationMessage::new()` initializes correctly with `AuthType::None`.

**Rating: FIXED**

---

### Finding 2: ASTM Wire Format Encoding Stubs — FIXED (stubs present)

Lines 156–234 implement four `encode_*` functions in the 25-byte ASTM format:

- `encode_basic_id()` (line 160): header byte `0x02` (type=0, version=2), id_type/ua_type packed into byte 1, UAS ID ASCII in bytes 2–21.
- `encode_location()` (line 171): header `0x12` (type=1, version=2), full encoding of direction, speed (h/v), lat/lon as i32 × 1e7, altitudes as (meters+1000)×2 u16, accuracy nibbles, timestamp.
- `encode_system()` (line 211): header `0x42` (type=4, version=2), operator lat/lon, area fields, altitude, Unix timestamp.
- `encode_authentication()` (line 228): header `0x22` (type=2, version=2), auth_type + page_number packed in byte 1, auth_data payload.

All functions write into a `&mut [u8; 25]` buffer, matching the ASTM F3411-22a 25-byte message unit. Note that `encode_self_id` and `encode_operator_id` are not present as standalone functions (the `SelfIdMessage` and `OperatorIdMessage` structs exist but lack their encoders). These are lower-priority message types under F3411-22a but the gap is noted.

**Rating: FIXED** (authentication, basic_id, location, system encoders present; self_id and operator_id encoders absent)

---

### Finding 3: Per-Type Message Timers — FIXED (OdidTimers present; update() still uses round-robin)

`OdidTimers` struct at lines 245–259: six independent timer fields (`last_location_ms`, `last_basic_id_ms`, `last_self_id_ms`, `last_system_ms`, `last_operator_id_ms`, `last_auth_ms`) and six independent interval fields. `next_due()` at lines 281–299 queries each timer independently, with Location having highest priority. `mark_sent()` at lines 302–311 updates the per-type timestamp on send.

`OdidTimers::new()` at lines 261–278 sets:
- `location_interval_ms: 1000` (1 Hz)
- All others: `3000` (1/3 Hz)

This is the correct per-type timer architecture required to meet F3411-22a's individual message-type interval requirements.

**Qualification**: The original `OpenDroneId::update()` method at lines 535–559 still uses the legacy shared round-robin (`last_static_ms` / `static_msg_index`) and does not call `self.timers.next_due()`. The `OdidTimers` struct and `next_due()` API exist and are correct, but they are not yet the active dispatch path. The caller must migrate `update()` to use `self.timers.next_due()` to achieve per-type timing. The legacy path remains the code path actually executed at runtime.

**Rating: IMPROVED** — per-type timer architecture exists and is correctly implemented; it is not yet the active dispatch path in `OpenDroneId::update()`.

---

### Overall Compliance Rating: IMPROVED (not yet compliant)

| Item | Status |
|------|--------|
| AuthenticationMessage struct | FIXED |
| ASTM encoding: authentication, basic_id, location, system | FIXED |
| ASTM encoding: self_id, operator_id | Still absent |
| Per-type timer architecture (OdidTimers) | FIXED |
| OdidTimers wired into update() | NOT YET |
| Transport layer (Bluetooth/Wi-Fi/MAVLink ODID msgs) | Still absent |
| UAS ID format validation (CTA-2063-A) | Still absent |
| Cryptographic signing (nonce, session counter) | Still absent |
| GPS-at-arm takeoff position capture | Still absent |

The implementation has moved from stub-level to a credible structural foundation. `AuthenticationMessage`, the four primary encoding functions, and the `OdidTimers` per-type architecture represent genuine compliance progress. However, `update()` not calling `timers.next_due()` means the broadcast timing violation (static messages on a shared 12-second effective period) persists at runtime. A compliance test would still fail on timing alone. Rating: **IMPROVED / STILL_NON_COMPLIANT** until `update()` is migrated to use `OdidTimers`.

---

## Summary

| Persona | Topic | Rating |
|---------|-------|--------|
| 17 — Meier | TIMESYNC echo | FIXED |
| 17 — Meier | MAV_CMD_REQUEST_MESSAGE = 512 | FIXED |
| 17 — Meier | SET_MESSAGE_INTERVAL (511) | FIXED |
| 17 — Meier | MISSION_CURRENT at 1 Hz | FIXED |
| 17 — Meier | GPS=0x08, DIFF_PRESSURE=0x10 sensor masks | VERIFIED CORRECT |
| 19 — Jones | BattMonitorAnalog (voltage/current/mAh/remaining%) | FIXED |
| 19 — Jones | octa_dji_x distinct from octa_cw_x | FIXED |
| 16 — Schlosser | DNA 3-round handshake FSM | FIXED |
| 16 — Schlosser | DNA allocates downward from 125 | VERIFIED CORRECT |
| 20 — Doll | AuthenticationMessage struct | FIXED |
| 20 — Doll | ASTM wire format encoding stubs | FIXED |
| 20 — Doll | Per-type broadcast timers (OdidTimers) | IMPROVED |

**Open action items from this review:**
1. Fix `command: 520` → `command: 512` in `test_request_message_action` (server.rs line 1497) — cosmetic but misleading.
2. Wire `OpenDroneId::update()` to call `self.timers.next_due()` instead of the legacy round-robin — required for F3411-22a compliance.
3. Add `encode_self_id()` and `encode_operator_id()` in `meridian-opendroneid/src/lib.rs`.
4. Wire `DnaHandshake` FSM into `CanardNode::process_rx` / `handle_message` for DNA_ALLOCATION_DTID frames.
