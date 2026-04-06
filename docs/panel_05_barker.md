# Infrastructure Review — Peter Barker
## Meridian Autopilot: Parameters, Logging, MAVLink, SITL, Infrastructure

**Reviewer**: Peter Barker — ArduPilot core infrastructure, AP_Param, AP_Logger, MAVLink, SITL  
**Date**: 2026-04-02  
**Files Reviewed**:  
- `crates/meridian-params/src/lib.rs`  
- `crates/meridian-log/src/lib.rs`  
- `crates/meridian-mavlink/src/server.rs`  
- `crates/meridian-mavlink/src/adapter.rs`  
- `bin/meridian-sitl/src/main.rs`  
- `docs/identification_payload_infra.md`  
- `docs/identification_comms_protocols.md`

---

## Executive Summary

Infrastructure bugs are the most insidious class of defect in autopilot software. They don't cause immediate crashes during development. They cause intermittent failures in the field — parameters that silently revert to defaults, logs that are unreadable after an incident, GCS connections that degrade unexpectedly, missions that stall mid-upload. Several of Meridian's infrastructure layers are structurally correct but are operating on incomplete foundations. The system will appear to work in SITL and short bench tests. It will fail in production.

Three subsystems are rated BROKEN for embedded deployment. One is SOLID for SITL-only use. The MAVLink layer is FRAGILE in ways that will cause real problems with QGroundControl.

---

## 1. Parameter Persistence

**Rating: FRAGILE (SITL) / BROKEN (embedded)**

The serialization layer in `meridian-params/src/lib.rs` is one of the better-constructed pieces of this codebase. The binary format has a defined magic (`0x4D505241`), version field, CRC32 footer, and bounded entry size. The `ParamBackend` trait is correctly defined. The dirty-tracking with a 16-entry save queue is a reasonable pattern — it matches the deferred-write model we use in AP_Param to avoid hammering flash on every parameter change.

**What is broken:**

No `ParamBackend` implementation exists anywhere. The trait is defined and the serialization helpers compile, but there is no type that implements `ParamBackend::save()` or `ParamBackend::load()` for any target. On embedded, this means zero persistence — every power cycle resets all parameters to compiled-in defaults. There is also no file-backed implementation for SITL, which means that even on Linux, the SITL binary in `bin/meridian-sitl/src/main.rs` calls `params.register_defaults()` and never loads or saves anything. The `ParamStore::apply_loaded()` and `flush_one()` paths are dead code.

**Critical design issues beyond the missing backend:**

The `save_queue` uses `swap_remove(0)` to pop the front element. `swap_remove` replaces index 0 with the last element, so parameters are saved in an undefined order. This is harmless for correctness but means that if power is lost mid-save, the most recently changed parameter (which may be the most important one) is not guaranteed to be the one that was written. In AP_Param we process the save queue in order, oldest first, and we verify each write before marking it clean.

The `MAX_SAVE_QUEUE` of 16 entries is too small for initialization scenarios. When first commissioning a system, a tuning session might touch 50 or more parameters. Anything beyond the 16th change is silently dropped from the queue. The dirty flag on the store is set, but individual parameter names above the queue limit are lost — they will never be saved unless the caller performs a full-store save. There is no mechanism to detect this overflow. You need either a larger queue, or a full-sweep fallback when the queue overflows.

There is no version migration path. If a future firmware update changes the parameter set — adds new params, removes old ones, or changes default values — the `deserialize_params()` function will simply silently skip unknown names and leave missing names at their compiled defaults. That is acceptable behavior, but there is no mechanism to detect that a format migration occurred or to notify the user that some parameters were reset.

The param name search in `get()` is a linear scan over up to 512 entries on every parameter access. At 512 parameters, this is measurable overhead in the control loop. AP_Param uses a sorted table with index caching. This is not a correctness issue but it will show up in task timing profilers on embedded.

**What needs to exist before deployment:**

1. A `LinuxParamBackend` that reads/writes a binary file using `std::fs`. This unblocks SITL persistence.
2. A `FlashParamBackend` that wraps the embedded flash driver (once AP_FlashStorage and StorageManager equivalents exist). This unblocks embedded persistence.
3. Load-on-boot wiring in the SITL main and in the embedded main loop.
4. Fix `swap_remove(0)` to use proper FIFO ordering.
5. Queue overflow handling — either a full-sweep flag or a larger queue.

---

## 2. Flight Logging

**Rating: FRAGILE (SITL) / BROKEN (embedded)**

The schema design in `meridian-log/src/lib.rs` is correct. The `LogMessage` trait with `msg_id()`, `schema()`, and `serialize_into()` mirrors the AP_Logger `LogStructure` approach cleanly. The message type IDs are numbered to match ArduPilot numbering for log viewer compatibility. The `FmtEntry` serialization produces an 89-byte self-describing header compatible with the DataFlash FMT format. This is good work.

The `LogBuffer<N>` ring buffer is the core runtime construct. It compiles and its write logic is correct for in-memory buffering.

**What is broken:**

There is no backend that flushes `LogBuffer` to any persistent storage. The log buffer accumulates entries in RAM and overwrites them when full. After every flight, all data is gone. This is not a minor gap — it means post-flight analysis is impossible, incident investigation is impossible, and any certification requirement for data retention cannot be met.

The SITL main loop has a console print every 800 ticks (`tick % (MAIN_HZ * 2) == 0`) that outputs altitude, attitude, and mode. This is the only "logging" happening. It is not machine-readable, not timestamped with GPS time, and not structured. It is a debug print, not a flight log.

**Specific missing pieces:**

The `LogBuffer::write()` function truncates the 64-bit microsecond timestamp to 32 bits (`timestamp.as_micros() & 0xFFFF_FFFF`). This gives a 4295-second (~71 minute) rollover. Any flight longer than 71 minutes will have timestamp discontinuities in the log. ArduPilot's DataFlash format uses a 32-bit timestamp with an epoch message at log start — this requires a defined `LOG_REF_TIME` message to reconstruct absolute time. Meridian has no equivalent. Missions exceeding 71 minutes (survey flights, long-endurance platforms) will produce logs that are unanalyzable.

The `FmtEntry` serialization hardcodes `buf[6..6+flen]` for the format string starting at offset 6, then starts the label section at offset 22 without filling the intervening bytes (`6+16 = 22` — this is correct). But the format string itself is a raw byte array, not a typed format descriptor string. AP_Logger uses format characters (`f` for float, `i` for int32, `B` for uint8, etc.) to tell log viewers how to interpret each field. Meridian's `LogFieldType` enum has the information but the serialization does not generate the corresponding format character string. A log file written by Meridian today would be parsed incorrectly by MAVExplorer or Mission Planner log viewer.

There is no `LOG_FMT` header written at the start of each log. Without the FMT messages, any log viewer must fall back to hardcoded schema knowledge, which defeats the self-describing design.

There is no mechanism to write a parameter snapshot on arm. AP_Logger writes a PARM message for every parameter value when the vehicle arms. This is essential for post-flight analysis — without it, you cannot know what PID values were in use during a flight.

**What needs to exist before deployment:**

1. A POSIX file backend for SITL: open a new `.bin` file on each arm, flush the `LogBuffer` contents periodically, close on disarm.
2. Fix the timestamp truncation — either extend to 64-bit storage or add an epoch reference message.
3. Add format character string generation to `FmtEntry::serialize_into()`.
4. Implement FMT header emission at log open.
5. Implement parameter snapshot on arm.
6. For embedded: a flash write backend targeting the W25NXX or JEDEC flash interface.

---

## 3. MAVLink Completeness

**Rating: FRAGILE**

The MAVLink layer is the most architecturally complete of the infrastructure components. The `MavlinkServer` + `MavlinkAdapter` split is clean. The stream rate scheduler mirrors ArduPilot's `SR0_*` parameter system. The parameter streaming with `MAX_PARAMS_PER_UPDATE = 5` per update tick is the correct bandwidth-limiting approach. The mission upload state machine handles the normal case correctly.

However, the gap between what QGC expects and what Meridian implements is large enough to cause real problems.

**Inbound message handlers:**

Meridian handles 12 of approximately 60 inbound message types. The handled set (heartbeat, param request/set/read, stream request, mission protocol for waypoints, arm/disarm command, takeoff) covers the absolute minimum for a QGC first-connect. What's missing that will cause QGC to malfunction:

- `MAVLINK_MSG_ID_TIMESYNC (111)`: QGC sends TIMESYNC immediately after connecting to synchronize clocks. Without a response, QGC logs warnings and the connection latency display is wrong. This is a two-line handler — receive, echo back with `ts1` set.
- `MAVLINK_MSG_ID_SET_MODE (11)`: Legacy mode set message. QGC uses this for some mode-change operations in addition to `MAV_CMD_DO_SET_MODE`. Without it, certain QGC mode-change UI elements silently fail.
- `MAVLINK_MSG_ID_MISSION_ITEM (39)` (float variant): QGC sends float-format MISSION_ITEM in some older code paths and when connecting to some legacy-compat modes. The server only handles `MISSION_ITEM_INT`. A GCS using float items will trigger the sequence-gap NACK and the upload will loop or stall.
- `MAVLINK_MSG_ID_MISSION_SET_CURRENT (41)`: No handler. QGC's "fly to waypoint" button sends this. Without it, manual waypoint jumping during auto missions is impossible.
- `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE (70)`: No handler. QGC's joystick support and the virtual joystick in the GCS use this message. Required for joystick-assisted flight.
- `MAVLINK_MSG_ID_COMMAND_ACK (77)`: Meridian sends command ACKs but has no handler for receiving them. When Meridian issues outbound commands (not currently implemented, but required for future features), there is no ACK tracking.

**Outbound messages missing that QGC displays prominently:**

- `EKF_STATUS_REPORT (193)`: QGC shows a prominent EKF health indicator on its HUD. Without this message, the indicator shows a red "EKF bad" state permanently, regardless of actual EKF health. Users will assume the vehicle is unsafe.
- `LOCAL_POSITION_NED (32)`: QGC's 3D visualization requires this. The map display will work (uses GLOBAL_POSITION_INT), but the attitude indicator and 3D vehicle model will not track correctly.
- `MISSION_CURRENT (42)`: QGC highlights the active waypoint on the mission view. Without this message, the mission display does not update as the vehicle progresses through waypoints.
- `EXTENDED_SYS_STATE (245)`: QGC uses this to determine landed/in-air state for UI elements. Without it, the "arm" button behavior is sometimes incorrect.
- `VIBRATION (241)`: Vibration monitoring is a first-line diagnostic for prop/motor balance issues. No message means no vibration data in QGC's vibration graph.
- `PID_TUNING (194)`: Critical for any in-field tuning workflow. Without it, Mission Planner and QGC PID tuning tools show no data.

**`MAV_CMD_SET_MESSAGE_INTERVAL (511)` is absent.** QGC sends this command to configure individual message rates. Without it, QGC falls back to `REQUEST_DATA_STREAM (66)` (which Meridian does handle), but `SET_MESSAGE_INTERVAL` is the MAVLink v2 standard and QGC logs a warning when it gets no response.

**Mission protocol gaps:**

The mission upload state machine handles only `MISSION_ITEM_INT`. There is no retry logic if an item is lost in transit — the server times out after 2 seconds, resets state, and emits nothing. The GCS will see silence and eventually retry the entire upload from the beginning. This creates a bad user experience on noisy serial links.

The mission download path (`MissionRequestList` → `MissionCount` → GCS requests items by index) has a silent bug: when the GCS requests `MISSION_REQUEST_INT(seq)` and `seq` is out of range, the handler returns `(0, None)` — it sends nothing. The GCS will time out waiting for `MISSION_ITEM_INT(seq)` and the download will stall. The correct behavior is to send `MISSION_ACK` with type `MAV_MISSION_INVALID_SEQUENCE`.

The mission protocol has no fence or rally protocol. QGC will show "fence upload failed" silently because there is no `MISSION_COUNT` response for fence items.

**The `gcs_sysid` field is populated but never updated.** The `GcsHeartbeat` handler updates `last_gcs_heartbeat_ms` but never sets `self.gcs_sysid`. The `gcs_sysid()` accessor will always return 0. Any targeted reply that needs to address the GCS will use system ID 0, which is broadcast — it will reach the GCS but also every other MAVLink node on the network.

**MAVLink signing is a stub.** `signing.rs` exists but is unimplemented. The SETUP_SIGNING handler is absent. For any deployment in a shared RF environment, link signing should be available as an option.

**Multi-link routing is absent.** If a second MAVLink channel is added (e.g., USB + telemetry radio simultaneously), there is no routing table. Packets received on one channel will not be forwarded to the other. This means Mission Planner over USB will not see traffic from a telemetry-attached tablet. This is not blocking for initial deployment but limits operational configurations.

---

## 4. SITL Fidelity

**Rating: SOLID (for what it claims to be)**

The SITL main loop in `bin/meridian-sitl/src/main.rs` is well-structured. The rate split — 400 Hz main loop with 1200 Hz physics at 3 substeps per tick — is the correct architecture. ArduPilot SITL uses the same pattern: a fast physics integration loop with the flight controller code running at a lower rate. The sensor simulation rates (GPS at 10 Hz, baro at 50 Hz, mag at 100 Hz) match realistic hardware.

The IMU sample pipeline is correct: physics state drives `SensorSim::sample_imu()`, which feeds `EkfCore::predict()`, and the EKF output drives the controller chain. This is the proper signal flow. The controller pipeline — position → attitude → rate → mixer — is correctly ordered and correctly conditioned on the armed state.

The TCP listener accepts exactly one connection, operates non-blocking, and handles disconnect cleanly. The `TcpNoDelay` flag is set, which is necessary to avoid Nagle algorithm delays in the MAVLink stream.

**Issues that affect SITL fidelity:**

The tick timing uses `std::thread::sleep(tick_duration - elapsed)`. On Windows, `thread::sleep` has ~15ms minimum resolution. At 400 Hz (2.5ms ticks), the sleep duration is always less than the minimum sleep resolution. The loop will run slower than 400 Hz and faster ticks will bunch. The actual achieved rate depends on the OS scheduler. This will cause the EKF time integration to be incorrect because `MAIN_DT` is hardcoded as `1/400` but actual elapsed time may be longer. For serious SITL fidelity work, this needs `spin_sleep` or a busy-wait loop with elapsed time correction. Real ArduPilot SITL measures actual elapsed time and passes it to the controller update functions.

The `ModeOutput` match has an unreachable `_ =>` arm. For any mode not returning `PositionTarget` or `AttitudeTarget`, motors go to zero and `Quaternion::identity()`. This means that during mode transitions or in any mode that returns other output types, the vehicle will silently cut throttle. This is not a crash but it will cause surprising behavior during multi-mode SITL testing.

There is no log write in the SITL main loop beyond the console print. The `LogBuffer` types from `meridian-log` are imported but never instantiated or written. Post-flight SITL analysis is not possible.

The SITL does not load parameters from disk on startup. `params.register_defaults()` is called but no file is loaded. Parameter changes made during a SITL session are discarded on exit. This makes iterative tuning workflows impossible — you cannot save a good tune in SITL and then test it again the next day.

The SITL does not simulate wind, turbulence, or sensor noise injection (beyond `SensorSim`). This is an acceptable limitation for a first iteration but worth noting: behavior in real flight will differ because SITL is currently a perfect-physics environment.

---

## 5. Mission Protocol State Machine

**Rating: FRAGILE**

The upload state machine covers the happy path: GCS sends count → server requests items sequentially → items arrive in order → server ACKs complete. This works when the link is clean, the GCS is well-behaved, and all items fit within `MAX_MISSION_ITEMS = 128`.

**Timeout handling is incomplete.** When the upload times out (`now_ms > mission_upload_timeout_ms`), the state is silently reset — `mission_upload_count` and `mission_upload_received` go to zero, but no `MISSION_ACK` with a failure code is sent to the GCS. QGC will see its last `MISSION_ITEM_INT` go unacknowledged, wait for its own timeout (typically 5 seconds), then send `MISSION_COUNT` again, triggering a fresh upload attempt. This is usually recoverable but produces confusing behavior on marginal links.

**Missing `MISSION_WRITE_PARTIAL_LIST` handler.** ArduPilot supports partial mission updates — rewriting from waypoint N onward without re-uploading the entire mission. QGC uses this for live mission editing during flight. Without this handler, any in-flight mission edit requires a full re-upload.

**The `mission_download_seq` field is incremented nowhere.** It is reset to 0 in `MissionRequestList` handling but is never incremented as items are sent. If the GCS requests items sequentially (the normal case), the server responds to each `MissionRequestInt(seq)` correctly because it indexes directly into `mission_items[seq]`. But `mission_download_seq` cannot be used to detect out-of-order requests or to send un-requested items proactively. The download is purely reactive (GCS requests each item by index), which is correct per protocol but the tracking variable is dead.

**Item count limit is 128 with no graceful rejection above that.** The `MissionCount` handler sends a NACK only if `count > MAX_MISSION_ITEMS`. It does not distinguish between "count is zero" and "count is too large" in the NACK type — both get `encode_mission_ack(1, ...)`. ArduPilot returns `MAV_MISSION_NO_SPACE` for overflow and `MAV_MISSION_COUNT_NOT_FOUND` for zero. QGC interprets the ACK type to give the user an appropriate error message.

---

## 6. Log Download Protocol

**Rating: BROKEN**

Log download is a stub. From the identification document:

```
LOG_REQUEST_LIST (117)  → STUB (log_download.rs)
LOG_REQUEST_DATA (119)  → STUB
LOG_ERASE (121)         → NO
LOG_REQUEST_END (122)   → NO
REMOTE_LOG_BLOCK_STATUS → NO
```

There are no logs to download because there is no log write backend. There is nothing to serve even if the protocol were implemented. This is a single consistent gap — both halves of the system (write and serve) are absent — rather than a partial implementation where the write side works but the download side is broken. That said, the consequence is the same: QGC's "Download Logs" button will produce no results. Post-incident investigation requires manual console output analysis only.

MAVFTP (`FILE_TRANSFER_PROTOCOL`, message 110) is also a stub. ArduPilot serves log files via MAVFTP as well as the native log protocol. Without either path, there is no mechanism for remote log retrieval.

---

## 7. Missing Infrastructure That Blocks Deployment

**AP_FlashStorage / StorageManager equivalents: ABSENT — blocks embedded deployment**

These are the physical and logical layers that sit between `ParamStore::flush_one()` and actual flash hardware. Without them, parameter persistence on any embedded target is impossible regardless of how complete the serialization layer becomes. The `ParamBackend` trait has no implementor because there is nothing to implement it on top of. This is not a quick fix — a correct wear-leveling flash storage implementation requires careful handling of power-loss scenarios, and getting it wrong causes data corruption that manifests as random parameter resets.

**AP_Filesystem equivalent: ABSENT — blocks logging, terrain, and firmware update**

The SITL log backend could be implemented with direct `std::fs` calls without an AP_Filesystem abstraction. But for embedded, any file-based operation (log to SD card, cache terrain tiles, load Lua scripts) requires a VFS layer. Without it, the Logger file backend, the Terrain SD cache, and any scripting system cannot be built.

**AP_RTC equivalent: ABSENT — affects log timestamps and Remote ID**

All log entries currently carry relative timestamps from `Instant::now()` at boot. There is no GPS time synchronization. This means log files cannot be correlated to real-world events with wall-clock timestamps. For post-incident analysis ("what was the aircraft doing at 14:23:07 UTC?"), you cannot answer the question. For Remote ID, timestamp synchronization to UTC is required by the ASTM F3411-22a specification — Meridian's Remote ID stub cannot become compliant without AP_RTC.

**AP_Relay: ABSENT — blocks parachute, camera shutter, gripper EPM, ICEngine ignition**

These are all marked STUB or PARTIAL in the payload review. The reason most of them are stubs is that they depend on relay output, which does not exist. The parachute deployment sequence — disarm motors, wait, pulse relay — cannot be completed because there is no relay output API. This is a safety-critical gap for any commercial or military deployment.

**AP_Declination: ABSENT — compass heading is wrong globally**

Every flight using compass-based heading (which is all of them, except GPS-velocity heading) will have a systematic heading error equal to the local magnetic declination. At my location in Australia this is about 12 degrees east. In some parts of the US it exceeds 20 degrees west. An autopilot that does not correct for declination will fly with a constant heading bias. This is not intermittent — it is a consistent error that affects every auto mission globally except near the zero-declination line.

**AP_AccelCal: ABSENT — accelerometer calibration is hardcoded or absent**

Without a 6-point calibration procedure, the accelerometer offsets are assumed to be zero. Any physical mounting imperfection or manufacturing bias directly adds to the attitude estimate error. On a new board, this will manifest as a vehicle that hovers slightly tilted, or drifts in position hold. The pre-arm check system (also not fully implemented) should gate arming on calibration completion.

**Task overrun monitoring: ABSENT**

The SITL main loop sleeps for whatever remains of the 2.5ms tick. If a task runs over budget, the loop simply runs late with no notification. ArduPilot's scheduler tracks per-task runtime and logs `SCHED` overrun events. Without equivalent monitoring, performance regressions in the embedded build will be invisible until they cause visible control degradation.

---

## Summary Ratings

| Domain | Rating | Notes |
|---|---|---|
| Parameter persistence (SITL) | FRAGILE | Serialization exists, no file backend |
| Parameter persistence (embedded) | BROKEN | No flash backend, no StorageManager |
| Flight logging (SITL) | BROKEN | Ring buffer in RAM, no file write, no replay |
| Flight logging (embedded) | BROKEN | No flash backend, timestamps truncated at 71min |
| MAVLink core (heartbeat/param/mission) | SOLID | Correct protocol handling |
| MAVLink completeness (QGC compat) | FRAGILE | EKF_STATUS, MISSION_CURRENT, SET_MODE missing |
| Mission upload state machine | FRAGILE | Happy path only, timeout sends no NACK |
| Mission download state machine | FRAGILE | Out-of-range seq returns silence not NACK |
| Log download protocol | BROKEN | Stub only, nothing to serve |
| MAVFTP | BROKEN | Stub only |
| SITL loop architecture | SOLID | Rate split, sensor pipeline, TCP handling correct |
| SITL timing accuracy | FRAGILE | thread::sleep resolution inadequate on Windows |
| SITL parameter persistence | BROKEN | No load/save across sessions |
| AP_FlashStorage / StorageManager | BROKEN | Absent, blocks all embedded persistence |
| AP_Filesystem | BROKEN | Absent, blocks embedded logging and terrain cache |
| AP_RTC | BROKEN | Absent, log timestamps are relative only |
| AP_Relay | BROKEN | Absent, blocks parachute, camera, gripper |
| AP_Declination | BROKEN | Absent, compass heading systematically wrong |
| AP_AccelCal | BROKEN | Absent, accelerometer bias uncorrected |
| MAVLink signing | BROKEN | Stub only |
| MAVLink routing (multi-link) | BROKEN | Single link only |

---

## What Must Be Fixed Before Any Field Deployment

In order of severity:

1. **Parameter persistence backend** (file for SITL, flash for embedded). Without this, every power cycle resets all tuning. This is not optional.

2. **Log file write backend** (POSIX for SITL at minimum). Without logs, no incident investigation is possible. SITL tuning work cannot be preserved or analyzed.

3. **AP_Declination**. This is a small library (10 files, ~500 lines in C++) with no dependencies. A Rust equivalent using the WMM2025 lookup table and bilinear interpolation is perhaps a week of work. The impact of not having it is that every auto mission flown anywhere except on the magnetic prime meridian has a systematic heading error.

4. **`EKF_STATUS_REPORT` outbound message**. QGC shows a permanent red EKF warning without it. Users will not trust the system.

5. **`TIMESYNC` handler**. Two-line fix. QGC sends it immediately on connect; silence causes connection quality warnings.

6. **Mission upload timeout NACK**. When a mission upload times out, send `MISSION_ACK` with the appropriate failure code. Currently the timeout is silent.

7. **Mission download out-of-range NACK**. Send `MISSION_ACK(MAV_MISSION_INVALID_SEQUENCE)` when a requested item index is out of range.

8. **AP_Relay**. Blocks parachute, which is safety-critical on any crewed-adjacent platform.

9. **Fix `swap_remove(0)` in save queue**. Replace with proper FIFO ordering before queue overflow causes a subtle parameter loss scenario.

10. **Log timestamp rollover**. Either extend to 64-bit or add an epoch reference message before 71-minute flights are planned.

---

## Closing Assessment

The developer who wrote the serialization layer for parameters and the MAVLink frame encoder clearly understands what correct protocol behavior looks like. The structural choices — the `ParamBackend` trait, the `LogMessage` trait, the stream group scheduler — are all idiomatic and appropriate. The problem is not incorrect design; it is incomplete vertical integration. The abstractions exist but they do not connect to any physical storage. The protocol machinery exists but it handles only the subset of messages needed to make a demo look functional.

ArduPilot has had 15 years of field failures teaching us which infrastructure corners cannot be cut. The parameter persistence gap will cause the most user-visible failures. The logging gap will make diagnosing those failures impossible. The compass heading gap will cause real navigation errors in real missions. These are not stretch goals for production polish — they are table stakes for a system that flies.

The SITL implementation is the best part of this submission. It is coherent, the architecture is sound, and the physics loop is correctly structured. Build on it: add file-based logging to SITL first, then add parameter file load/save, then use SITL as the validation environment for every infrastructure component before porting to embedded.

The infrastructure is the hardest part of autopilot development precisely because it is invisible until it fails. Do not let it remain this incomplete.
