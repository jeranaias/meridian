# Panel Review 17 — MAVLink Protocol Completeness and Correctness

**Reviewer**: Lorenz Meier — Creator of MAVLink, PX4  
**Date**: 2026-04-02  
**Scope**: `crates/meridian-mavlink/src/{server.rs, adapter.rs, v2.rs}` + `docs/identification_comms_protocols.md`  
**Overall Rating**: **PARTIAL**

---

## Executive Summary

Meridian's MAVLink layer is a competent but fundamentally incomplete implementation. The wire format is correct, the mission upload state machine is structurally sound, and the HEARTBEAT encoding is mostly right. However, three specific defects will cause QGC or Mission Planner to malfunction in normal usage: the missing TIMESYNC handler will cause QGC to show a persistent connection warning; the absence of MISSION_CURRENT streaming will prevent the mission map from updating during flight; and the MAV_CMD_SET_MESSAGE_INTERVAL (511) handler is missing entirely, which means QGC's message rate negotiation on initial connect fails silently. These are not edge cases — they are part of the QGC connect sequence and will be hit within the first 30 seconds of every connection.

---

## 1. HEARTBEAT Encoding

**Finding: CORRECT with one minor deficiency.**

The HEARTBEAT encoder in `adapter.rs:encode_heartbeat()` is structurally correct:

- Payload layout matches the spec: `custom_mode[4] | type[1] | autopilot[1] | base_mode[1] | system_status[1] | mavlink_version[1]` — 9 bytes total. Field order is correct.
- `MAV_AUTOPILOT_ARDUPILOTMEGA (3)` is hardcoded. This is the right constant and is correct for a vehicle claiming ArduPilot compatibility with QGC.
- `mavlink_version = 3` is correct for MAVLink v2.
- `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (0x01)` is always set. Correct — all ArduPilot-family vehicles use custom modes and this bit must always be high.
- The `MAV_MODE_FLAG_SAFETY_ARMED (0x80)` bit is set from `vs.armed`. Correct.
- Mode flag inference (STABILIZE/GUIDED/AUTO bits) from `custom_mode` integer is a reasonable approximation. QGC does not use these legacy base_mode bits for mode display in custom-mode vehicles — it reads custom_mode directly — so this is not a correctness issue.

**Minor deficiency**: The GCS system ID is never learned from the received HEARTBEAT. The `gcs_sysid` field is stored but `parse_inbound` for MSG_HEARTBEAT returns `Some(InboundCommand::GcsHeartbeat)` without extracting `sysid` from the frame header. As a result, `encode_command_ack`, `encode_mission_count`, and all other targeted replies always set `target_system = 0` (broadcast). This works — broadcast is valid — but violates the MAVLink convention that responses should be addressed to the originating system. QGC accepts broadcast; Mission Planner is more strict in some configurations.

---

## 2. Mission Upload/Download State Machine

**Finding: STRUCTURALLY CORRECT, but missing three sub-states that will cause real-world failures.**

### What is correct

The core state machine in `server.rs` implements the standard MAVLink mission upload protocol correctly:

1. GCS sends `MISSION_COUNT(n)` → FC sends `MISSION_REQUEST_INT(0)` — implemented.
2. GCS sends `MISSION_ITEM_INT(seq=0)` → FC sends `MISSION_REQUEST_INT(1)` — implemented.
3. Repeat until all items received → FC sends `MISSION_ACK(ACCEPTED)` — implemented.
4. Sequence gap detection: `seq != self.mission_upload_received` → NACK — implemented.
5. Upload timeout (2 seconds per item) — implemented.
6. Download: `MISSION_REQUEST_LIST` → `MISSION_COUNT(n)` → `MISSION_REQUEST_INT(seq)` → `MISSION_ITEM_INT(seq)` — implemented.
7. `MISSION_CLEAR_ALL` → reset store + ACK — implemented.

### What is missing and will cause failures

**A. MISSION_ITEM (float, message ID 39) is not handled.**

QGC currently sends `MISSION_ITEM_INT` by default, but Mission Planner and some older GCS versions send the legacy `MISSION_ITEM` (float coordinates). The identification doc confirms this gap. Any float-coordinate mission upload will silently drop all items. The adapter's `parse_inbound` switch falls through to `None` for msg ID 39 — the frame is discarded without any NACK. The GCS will time out waiting for `MISSION_REQUEST_INT(0)` and declare the upload failed.

**B. MISSION_SET_CURRENT (message ID 41) is not handled.**

This message is how a GCS jumps to a specific waypoint during a mission. When the user clicks "Jump to waypoint N" in QGC's mission plan view, QGC sends `MISSION_SET_CURRENT`. Without this handler, the vehicle ignores the jump silently. No NACK is sent, so QGC's UI state diverges from vehicle state.

**C. MISSION_CURRENT (message ID 42) is never sent.**

The spec requires the autopilot to broadcast `MISSION_CURRENT` at 1 Hz during a mission, updating the active waypoint sequence number. QGC uses this to move the "active waypoint" highlight on the map. Meridian never sends `MISSION_CURRENT` — it is not in the stream group dispatch, not in `update()`, and not encoded anywhere. The map will show waypoint 0 permanently throughout a mission.

**D. MISSION_ACK for download completion is missing.**

When the GCS finishes downloading a mission (it has received all `MISSION_ITEM_INT` frames it requested), it sends `MISSION_ACK(ACCEPTED)` back to the FC to confirm the download succeeded. Meridian does not handle this inbound `MISSION_ACK`. This is non-fatal since the FC does not require this ACK for anything currently, but it means the download handshake is technically incomplete per spec.

**E. No mission type field in MISSION_COUNT.**

The MAVLink mission protocol since 2019 includes a `mission_type` field in `MISSION_COUNT` (byte offset 2) to distinguish waypoint missions from fence plans and rally points. Meridian's `encode_mission_count` encodes this field as 0 (MAV_MISSION_TYPE_MISSION), which is correct for waypoint missions, but the fence and rally protocols are entirely absent. QGC will show the geofence editor as empty and non-functional.

---

## 3. AUTOPILOT_VERSION Correctness

**Finding: FUNCTIONAL but capability bits need attention.**

`adapter.rs:encode_autopilot_version()` constructs a 78-byte payload and sets capabilities:

```rust
let capabilities: u64 = (1 << 1) | (1 << 7) | (1 << 8) | (1 << 13);
```

Decoding against the MAVLink `MAV_PROTOCOL_CAPABILITY` enum:

| Bit | Constant | Meaning | Assessment |
|-----|----------|---------|------------|
| 1 | `MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT` | Supports float params | Correct — param table is all REAL32 |
| 7 | `MAV_PROTOCOL_CAPABILITY_COMMAND_INT` | Supports COMMAND_INT | Correct — adapter parses COMMAND_INT |
| 8 | `MAV_PROTOCOL_CAPABILITY_MAVLINK2` | Supports MAVLink v2 | Correct |
| 13 | `MAV_PROTOCOL_CAPABILITY_MISSION_INT` | Supports MISSION_ITEM_INT | Correct |

**Problem 1**: `MAV_PROTOCOL_CAPABILITY_MAVLINK2` is bit 8 (value 256). The expression `(1 << 8)` evaluates to 256 as a u64, which is correct. However, `MAV_PROTOCOL_CAPABILITY_COMMAND_INT` is bit 7 (value 128). Verify this against the canonical enum — some versions have different numbering. If the numbering has shifted in the spec version QGC is compiled against, QGC will not offer COMMAND_INT to the vehicle.

**Problem 2**: `MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET` (bit 4) is advertised as absent. QGC will not offer guided attitude control to the vehicle, which is fine for now.

**Problem 3**: `MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION` (bit 16) is correctly absent. Do not add this without implementing the handler — QGC and some safety systems will send `DO_FLIGHTTERMINATION` if they believe the vehicle supports it.

**Problem 4**: The `uid` field at payload offset 36 is all zeros. QGC accepts zero UID; it displays it as "not set" and continues normally. This is acceptable for development.

**Problem 5**: The `flight_sw_version` encoding uses the ArduPilot format: `(major << 24) | (minor << 16) | (patch << 8) | type`. The code sets `0.1.0-dev`. This is correct in format; QGC displays this in the vehicle summary panel.

---

## 4. Parameter Streaming Bandwidth Limiting

**Finding: CORRECT in mechanism; scaling formula has an edge case.**

The bandwidth limiter in `server.rs:update()`:

```rust
const MAX_PARAMS_PER_UPDATE: u16 = 5;
```

Each call to `update()` sends at most 5 PARAM_VALUE frames. At a typical 100 Hz update loop and a 57600 baud link, each PARAM_VALUE frame is 25-byte payload + 12-byte overhead = 37 bytes + 2 CRC = 39 bytes, or ~39 bytes per frame. Five frames per update cycle = 195 bytes per loop tick = 19,500 bytes/second at 100 Hz. At 57600 baud (5760 bytes/sec effective), this saturates the link in ~3 update cycles. The constant is appropriate for higher-baud links (460800 baud = 46 KB/s effective — 19.5 KB/s is 42% of link, which is within the "~30%" comment's intent but actually exceeds it at 100 Hz).

**Recommendation**: Make `MAX_PARAMS_PER_UPDATE` scale with the loop rate or add a time-based minimum interval between parameter frames (e.g., send at most 1 param per 10 ms). The current design is correct in architecture but the constant assumes a lower loop frequency than a real FC will run.

The param streaming correctly gates on `param_streaming` flag, increments `param_send_index`, and clears the flag when complete. The `send_param_by_name_or_index` correctly handles both name and index lookup. The `param_type` is hardcoded to 9 (`MAV_PARAM_TYPE_REAL32`) for all parameters — correct for the current table, but will need to be extended when integer parameters are added.

---

## 5. Message Handlers That Will Cause QGC/Mission Planner to Malfunction

These are ranked by severity — top entries cause failures in the normal connect/operate workflow.

### CRITICAL — Will break QGC connect sequence

**1. MAV_CMD_SET_MESSAGE_INTERVAL (511) — NOT HANDLED**

QGC sends this on initial connection to negotiate stream rates. It supersedes the legacy `REQUEST_DATA_STREAM (66)` protocol. The identification doc shows Meridian only handles `REQUEST_DATA_STREAM`. Current QGC versions send `SET_MESSAGE_INTERVAL` first. Meridian returns `MAV_RESULT_UNSUPPORTED` (via the `Unknown` branch), which causes QGC to log a warning and fall back to `REQUEST_DATA_STREAM`. The fallback works, but QGC marks the connection as degraded and some panel features are disabled. This must be implemented for full QGC compat.

**2. TIMESYNC (message ID 111) — NOT HANDLED**

QGC and Mission Planner both send `TIMESYNC` messages to synchronize flight computer time. The protocol is bidirectional — the GCS sends a `TIMESYNC` with `tc1 = 0`; the FC must echo it back with `tc1 = ts1 = now_ns`. QGC uses the round-trip to estimate link latency and display it in the status bar. More critically, QGC has a timeout: if it does not receive a `TIMESYNC` response within ~5 seconds of connecting, it displays a "Communication lost" banner even if telemetry is flowing. The incoming `TIMESYNC` frame is silently discarded by Meridian (falls through to `None` in `parse_inbound`). This will trigger a false "comm lost" state in QGC within 5 seconds of every connection.

**3. MISSION_CURRENT (message ID 42) — NEVER SENT**

Already noted in the mission state machine section. During any mission execution, the active waypoint indicator in QGC will be frozen at item 0. Not a crash, but a fundamental navigation display failure.

### HIGH — Normal operations broken

**4. MISSION_ITEM (float, message ID 39) — NOT HANDLED**

Mission Planner sends float missions. All MP-originated mission uploads will fail silently.

**5. MISSION_SET_CURRENT (message ID 41) — NOT HANDLED**

Jump-to-waypoint command silently ignored. QGC user clicks "fly to this waypoint" and nothing happens.

**6. SET_MODE (message ID 11) — NOT HANDLED**

This is the legacy mode-set message. Some GCS versions (and ATAK-based systems) send `SET_MODE` rather than `MAV_CMD_DO_SET_MODE`. Mode changes from legacy GCS clients will be silently dropped. Meridian handles `DO_SET_MODE` via COMMAND_LONG but not the direct `SET_MODE` message.

**7. MAV_CMD_DO_SET_HOME (179) — NOT HANDLED**

QGC has a "Set Home" action in the map right-click menu. This is handled by `MAV_CMD_DO_SET_HOME`. Without this handler, the home position cannot be updated from QGC.

**8. COMMAND_ACK (message ID 77, inbound) — NOT HANDLED**

When Meridian sends a `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` or any command to the GCS, the GCS may ACK it. Inbound COMMAND_ACK is listed in `identification_comms_protocols.md` as not handled. This is low priority since Meridian rarely originates commands, but it becomes relevant if a companion computer or relay is added.

### MEDIUM — Features missing but not crash-inducing

**9. MAV_CMD_GET_HOME_POSITION (410) — NOT HANDLED**

QGC requests current home position on connect. Meridian streams HOME_POSITION periodically, so QGC will eventually receive it, but the on-demand request will return unsupported.

**10. MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246) — NOT HANDLED**

QGC's "Reboot vehicle" button sends this. It will receive `MAV_RESULT_UNSUPPORTED` and display an error.

**11. RADIO_STATUS / RADIO (message IDs 109, 166) — NOT HANDLED**

SiK radios inject `RADIO_STATUS` into the MAVLink stream. Without this handler, link quality data is not extracted, and Meridian cannot throttle its own output rate based on radio buffer fullness (the ArduPilot `radio_status` flow control mechanism).

---

## 6. Minimum Handler Set for QGC Compatibility

This is the minimum that must be implemented before QGC will connect cleanly and operate without error banners:

| Priority | Message / Command | Required Action |
|----------|------------------|-----------------|
| P0 | TIMESYNC (111) | Echo back with `tc1 = ts1 = now_ns` |
| P0 | MAV_CMD_SET_MESSAGE_INTERVAL (511) | Map to stream group intervals; ACK ACCEPTED |
| P0 | MISSION_CURRENT (42) outbound | Stream at 1 Hz during mission |
| P1 | MISSION_ITEM (39) | Parse float coords → store as MISSION_ITEM_INT equivalent |
| P1 | MISSION_SET_CURRENT (41) | Jump active waypoint; send MISSION_CURRENT |
| P1 | SET_MODE (11) | Set custom_mode from `msg.custom_mode` |
| P2 | MAV_CMD_GET_HOME_POSITION (410) | Send HOME_POSITION on demand |
| P2 | MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246) | ACK + trigger reset |
| P2 | RADIO_STATUS (109) | Extract `txbuf` field for flow control |
| P2 | MAV_CMD_DO_SET_HOME (179) | Update home lat/lon/alt |

---

## 7. Wire Format Correctness

**Finding: CORRECT.**

The MAVLink v2 framing in `v2.rs` is correct:

- STX byte is `0xFD` (correct for v2; v1 is `0xFE`).
- Header layout (10 bytes): `STX | len | incompat_flags | compat_flags | seq | sysid | compid | msgid[0] | msgid[1] | msgid[2]` — correct.
- CRC is X.25 over bytes `[1..header+payload]` with crc_extra appended — correct algorithm, correct range.
- Message ID is encoded little-endian across 3 bytes — correct.
- The stream parser is a correct byte-at-a-time state machine with proper resync on `0xFD`.

The CRC extra table in `v2.rs:crc_extra()` was spot-checked:

| Message | Expected CRC extra | In code | Status |
|---------|-------------------|---------|--------|
| HEARTBEAT (0) | 50 | 50 | OK |
| SYS_STATUS (1) | 124 | 124 | OK |
| PARAM_VALUE (22) | 220 | 220 | OK |
| ATTITUDE (30) | 39 | 39 | OK |
| GLOBAL_POSITION_INT (33) | 104 | 104 | OK |
| MISSION_REQUEST_INT (51) | 196 | 196 | OK |
| MISSION_ITEM_INT (73) | 38 | 38 | OK |
| MISSION_ACK (47) | 153 | 153 | OK |
| AUTOPILOT_VERSION (148) | 178 | 178 | OK |
| COMMAND_ACK (77) | 143 | 143 | OK |

No errors found. Any message not in the table gets CRC extra = 0, which is wrong for any message actually used. The fallback `_ => 0` is safe only because those messages are never sent or received — if a new message is added without updating this table, silent CRC failures will result.

---

## 8. COMMAND_ACK Payload Correctness

**Finding: UNDERSIZED PAYLOAD.**

`adapter.rs:encode_command_ack()` encodes 3 bytes: `command[2] | result[1]`. The MAVLink spec for COMMAND_ACK (ID 77) defines the payload as:

```
command:   uint16_t  (2 bytes)
result:    uint8_t   (1 byte)
progress:  uint8_t   (1 byte)  — added in MAVLink 2
result_param2: int32_t (4 bytes) — added in MAVLink 2
target_system:    uint8_t (1 byte)
target_component: uint8_t (1 byte)
```

Total: 11 bytes. The code sends 3 bytes. Under MAVLink v2 payload truncation rules, trailing zero bytes can be omitted, so a 3-byte COMMAND_ACK is technically valid as long as QGC treats the missing fields as zero. QGC does accept truncated payloads. However, if QGC ever reads `target_system` to verify the ACK is addressed to it, a truncated ACK will not match and may be dropped. In practice, current QGC accepts this. Flag as technical debt.

---

## 9. SYS_STATUS Sensor Mask Correctness

**Finding: SENSOR BIT COLLISION.**

In `adapter.rs`:

```rust
pub const MAV_SYS_STATUS_SENSOR_GPS: u32 = 0x08;
pub const MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE: u32 = 0x08;
```

Both GPS and differential pressure are assigned bit 0x08. This is a copy-paste error. The correct values from the MAVLink spec:

- `MAV_SYS_STATUS_SENSOR_GPS` = 0x00000008 (bit 3) — correct
- `MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE` = 0x00000010 (bit 4) — **wrong in code, shows 0x08**

This does not affect the current implementation since `present_mask()` and `health_mask()` only reference the GPS constant. But if differential pressure is ever added to `SensorHealth`, the mask will report GPS status instead of baro diff pressure status. Fix the constant.

---

## 10. Capability Flag Cross-Reference Issue

**Finding: MAV_CMD_REQUEST_MESSAGE uses wrong command ID in documentation.**

The identification doc at section 3 lists:

```
MAV_CMD_GET_MESSAGE_INTERVAL | 512 | NO
MAV_CMD_REQUEST_MESSAGE      | 512 | YES (partial — adapter.rs case 520)
```

These have the same ID (512) which is wrong. The correct IDs are:

- `MAV_CMD_REQUEST_MESSAGE` = 512
- `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` = 520

The adapter correctly uses 520 for `RequestAutopilotVersion` and 520 for `REQUEST_MESSAGE` is wrong — the code at line 499 handles command ID `520` and labels it `MAV_CMD_REQUEST_MESSAGE`, but the real `MAV_CMD_REQUEST_MESSAGE` is **512**, not 520. This means when QGC sends `MAV_CMD_REQUEST_MESSAGE(512)` requesting a specific message on demand, Meridian's adapter dispatches it to `CommandAction::Unknown` and returns `MAV_RESULT_UNSUPPORTED`. The case 520 in the adapter is handling `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` correctly (519 is its documented ID in some versions, 520 in others — verify against target QGC build). The `REQUEST_MESSAGE (512)` path is completely missing.

**This is an active bug**: QGC sends `MAV_CMD_REQUEST_MESSAGE(512)` to request `AUTOPILOT_VERSION` as part of the initial connection handshake. Meridian only responds to command 519/520. If QGC sends 512, Meridian will not send `AUTOPILOT_VERSION`, and QGC will display "Vehicle does not support MAVLink 2" in the summary panel.

---

## Summary Scorecard

| Area | Status | Notes |
|------|--------|-------|
| Wire format (v2 framing, CRC) | COMPLIANT | No defects |
| HEARTBEAT encoding | COMPLIANT | GCS sysid not learned |
| Mission upload state machine | PARTIAL | Float items missing, MISSION_CURRENT never sent |
| Mission download state machine | PARTIAL | ACK completion not handled |
| AUTOPILOT_VERSION | PARTIAL | Capabilities correct; REQUEST_MESSAGE(512) bug means it may never be triggered |
| Parameter streaming | PARTIAL | Bandwidth limit correct in structure, constant needs tuning |
| TIMESYNC | BROKEN | Will cause false "comm lost" in QGC within 5 seconds |
| MAV_CMD_SET_MESSAGE_INTERVAL | BROKEN | QGC rate negotiation fails |
| Sensor mask constants | DEFECT | GPS/differential pressure bit collision |
| MAV_CMD_REQUEST_MESSAGE ID | DEFECT | 512 not handled, only 519/520 |

**Overall Rating: PARTIAL**

The system will connect to QGC and display basic telemetry. It will not complete a clean connection sequence, will show a communication warning within 5 seconds, cannot jump waypoints, and will fail all Mission Planner mission uploads. The five items marked P0/P1 in Section 6 must be resolved before flight testing with a real GCS.
