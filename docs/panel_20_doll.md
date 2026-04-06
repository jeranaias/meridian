# Panel Review 20 — Remote ID & ADS-B Compliance
**Reviewer**: Steve Doll — FAA UAS Integration Specialist  
**Specialization**: Remote ID (ASTM F3411-22a), ADS-B equipage, Part 107, UTM integration  
**Date**: 2026-04-02  
**Files Reviewed**:
- `crates/meridian-opendroneid/src/lib.rs`
- `crates/meridian-adsb/src/lib.rs`
- `docs/identification_payload_infra.md` (OpenDroneID and ADSB sections)

---

## Overall Rating: NON_COMPLIANT

Meridian's OpenDroneID implementation cannot currently pass an FAA Remote ID compliance test. The broadcast state machine, timing, and pre-arm gate logic in `meridian-opendroneid` represent real improvement over what the audit document describes as a pure stub — but critical structural gaps remain that render it non-compliant with ASTM F3411-22a and 14 CFR Part 89. The ADSB-In implementation in `meridian-adsb` is functionally richer but is built on unimplemented hardware backends and has its own significant gaps.

---

## Section 1: OpenDroneID — ASTM F3411-22a Compliance

### 1.1 Message Type Coverage

ASTM F3411-22a requires five message types. Meridian defines structs for four of them:

| Message Type | Required by F3411-22a | Meridian Status |
|---|---|---|
| Basic ID | YES | Struct defined, encoding absent |
| Location/Vector | YES | Struct defined, encoding absent |
| Authentication | YES | **Missing entirely** |
| Self-ID | YES | Struct defined, encoding absent |
| System | YES | Struct defined, encoding absent |
| Operator ID | OPTIONAL (EU-mandated) | Struct defined, encoding absent |

**Authentication message is completely absent.** There is no `AuthenticationMessage` struct, no nonce generation, no timestamp signing, and no session counter. Authentication is a mandatory message type under F3411-22a Section 5.4.3. Its omission alone is sufficient grounds for a compliance failure at an FAA test event. Notably, the audit document in `identification_payload_infra.md` correctly identifies this gap ("Authentication message generation (timestamp + nonce)") but the implementation has not addressed it.

**Message encoding is entirely absent.** The five structs are well-designed data containers, but there is no serialization to the ASTM F3411-22a wire format (the 25-byte packed binary payload). Without encoding, nothing can be transmitted to a Bluetooth or Wi-Fi broadcast module. The `BroadcastAction` enum correctly names what to send next, but there is no `encode_*` function anywhere in the crate.

### 1.2 Broadcast Timing

The timing logic in `OpenDroneId::update()` is the strongest part of the implementation and correctly reflects the standard's requirements:

- **Location at 1 Hz**: `location_interval_ms = 1000`. The wrapping subtraction is correct and handles rollover. The `last_location_ms = u32::MAX` sentinel correctly forces first-call broadcast. **This meets F3411-22a Section 5.5.1 minimum 1 Hz requirement.**

- **Static messages at ~3-second round-robin**: `static_interval_ms = 3000`. F3411-22a requires Basic ID, System, and the optional Self-ID / Operator ID to be broadcast at a maximum interval of 3 seconds each. The round-robin across four messages means each static message fires every 12 seconds in the worst case (4 messages × 3 s each), which **exceeds the 3-second maximum interval** for Basic ID and System. This is a regulatory violation.

  The correct approach is either to broadcast all static messages every 3 seconds (pack them into a message pack), or to run independent timers per message type. ArduPilot uses message packs — a single Bluetooth/Wi-Fi beacon containing all four static messages packed together every 3 seconds — which is the preferred F3411-22a compliance path.

- **Location message priority is correct**: Location preempts static in the update loop.

### 1.3 Pre-Arm Transponder Handshake

The pre-arm gate is correctly implemented:

```rust
pub fn prearm_ok(&self) -> bool {
    if !self.options.enforce_prearm_checks { return true; }
    self.transponder_connected && self.transponder_arm_status_ok
}
```

This matches the ArduPilot pattern and the FAA's operational expectation: an autopilot must not arm unless the Remote ID module confirms it is functional and broadcasting. The `enforce_prearm_checks: true` default is correct — it must never ship with that default inverted.

**Gap**: The handshake only checks a boolean flag. There is no mechanism to actually receive `OPEN_DRONE_ID_ARM_STATUS` MAVLink messages from the companion module, parse them, and set `transponder_arm_status_ok`. The MAVLink `OPEN_DRONE_ID_*` message set is entirely unimplemented (confirmed by `identification_payload_infra.md`). The handshake is a well-designed gate that has no signal to open it.

### 1.4 UAS ID Management

Positive elements:
- `IdType` enum correctly covers ANSI/CTA-2063-A serial number, CAA registration, UTM-assigned, and specific session — all four F3411-22a ID types are present.
- `lock_uas_id()` and `load_persistent_id()` provide the persistence behavior required to prevent ID spoofing via param reload.
- `lock_uas_id_on_first_rx` option mirrors ArduPilot's `ODID_OPTIONS` bit correctly.

Gap: There is no validation that the UAS ID conforms to ANSI/CTA-2063-A serial number format when `IdType::SerialNumber` is used. An ID field containing arbitrary ASCII will pass through to broadcast. An FAA compliance test will check that the serial number format is valid.

### 1.5 Transport Layer

`OdidTransport` defines `MavlinkSerial` and `DroneCan` variants. Neither is connected to a real transport driver. The standard requires actual Bluetooth 5 Long Range (Coded PHY) or Wi-Fi Neighbor Awareness Networking (NaN) broadcasts for Standard Remote ID, or a broadcast module accepting the MAVLink `OPEN_DRONE_ID_*` messages for Network Remote ID. There is no path from `BroadcastAction` to any physical transmission.

### 1.6 Operator Location

`set_operator_location()` accepts a lat/lon/alt for the System message. The `OperatorLocationType` enum correctly offers Takeoff, Dynamic, and Fixed. However:
- There is no mechanism to capture takeoff position automatically from GPS at arm time.
- There is no dynamic operator location update path (for operators using a GCS with GPS).
- The System message `timestamp` field is `u32` (Unix epoch) but the crate has no RTC dependency. `identification_payload_infra.md` confirms `AP_RTC` is entirely absent from Meridian. Without UTC time, the System message timestamp will be zero or relative, which is non-compliant.

### 1.7 Summary: OpenDroneID Gaps vs F3411-22a

| Requirement | Status |
|---|---|
| 5 message types defined | PARTIAL (4 of 5 — Authentication missing) |
| Wire format encoding (25-byte packed) | ABSENT |
| Location broadcast >= 1 Hz | TIMING CORRECT, no physical TX |
| Static messages every <= 3 s each | NON-COMPLIANT (round-robin = up to 12 s) |
| Authentication message with timestamp/nonce | ABSENT |
| Pre-arm gate (transponder arm_status) | GATE LOGIC CORRECT, no MAVLink RX path |
| UAS ID format validation (CTA-2063-A) | ABSENT |
| Bluetooth 5 / Wi-Fi NaN transport | ABSENT |
| UTC timestamp in System message | ABSENT (no RTC) |
| MAVLink OPEN_DRONE_ID_* message set | ABSENT |

---

## Section 2: ADS-B — Threat Detection Adequacy

### 2.1 What Is Implemented

`meridian-adsb` is a meaningful step beyond stub. The following is functional:

- **Aircraft database** with 25-aircraft capacity, ICAO-keyed update-in-place, oldest-entry replacement when full.
- **Stale timeout pruning** at 5 seconds — appropriate for ADS-B message rate (~1 Hz typical).
- **Spatial pre-filter** (radius + altitude band) — mirrors ArduPilot's `_list_radius` and `_list_altMax` params.
- **TTCA threat classification** with four levels (None / Advisory / Warning / Alert). The math is correct: haversine 2D distance, proper relative velocity decomposition in NE frame, standard TTCA dot-product formula. The thresholds (1 NM / 30 s Alert, 2 NM / 60 s Warning, 5 NM / 120 s Advisory) are operationally reasonable for small UAS.
- **Test suite**: Nine tests, all meaningful — including head-on Alert, diverging None, closest-aircraft selection, database overflow, and haversine validation with a known-good NYC-to-LAX distance. This is solid.

### 2.2 Critical Backend Gaps

The hardware receivers are stubs returning `None` unconditionally:

```rust
pub fn parse_adsb_vehicle(&self, _payload: &[u8]) -> Option<AircraftState> {
    // Stub: real implementation would parse MAVLink msg #246
    None
}
```

No data will ever enter the `AircraftDatabase` without implementing at least one backend. The identified gaps from `identification_payload_infra.md` are accurate:
- uAvionix MAVLink backend (MAVLink message #246 `ADSB_VEHICLE` parsing)
- uAvionix UCP direct serial backend
- Sagetech MXS backend (transponder with squawk control)
- GDL-90 encoder (tablet/EFB integration)
- `ADSB_VEHICLE` outbound MAVLink broadcast to GCS
- Avoidance system integration hook (threat level must trigger maneuver, not just report)

### 2.3 ADS-B Out

`AdsbOutConfig` is defined with ICAO address, callsign, emitter type, and squawk fields. The `enabled: false` default is correct. However, there is no implementation of ADS-B Out transmission. For operations in Class B/C/D/E airspace, ADS-B Out (1090 ES) is required under 14 CFR Part 91.225. This is separate from Remote ID and is required for any flight near an airport.

### 2.4 Altitude Handling

The threat assessment is strictly 2D (horizontal distance + horizontal TTCA). There is no vertical separation component. A fast-descending or climbing manned aircraft could pass within 30 feet vertically and register as Advisory or None based purely on horizontal geometry. For a system targeting controlled airspace integration, 3D threat assessment is necessary. This is a known limitation of the current approach and should be documented.

### 2.5 Receiver Count vs ArduPilot

ArduPilot's AP_ADSB tracks up to ~50 aircraft across multiple backends simultaneously. Meridian caps at 25 across a single active backend. For low-density airspace this is adequate. For operations near busy airports (Class B/C), 25 may be insufficient. This should be a configurable parameter, not a compile-time constant.

---

## Section 3: FAA Compliance Test Readiness

An FAA Remote ID compliance test (as conducted by declared identification areas or under 14 CFR Part 89.115) checks:

1. **Broadcast presence**: Is the UAS broadcasting Remote ID on Bluetooth 5 or Wi-Fi NaN? — **FAIL**: No physical transport.
2. **Message completeness**: Are all required message types present and well-formed? — **FAIL**: Authentication absent, no encoding.
3. **Broadcast rate**: Location at >= 1 Hz, Basic ID <= 3 s, System <= 3 s? — **PARTIAL FAIL**: Timing logic is correct for Location, non-compliant for static interval distribution.
4. **UAS ID validity**: Is the UAS ID in valid CTA-2063-A format? — **UNVERIFIED**: No format validation.
5. **Operator location present**: Is a valid operator location included in the System message? — **FAIL**: UTC timestamp absent (no RTC).
6. **Pre-arm blocking**: Does the system refuse to arm without active Remote ID? — **PASS** (logic is correct, even if the signal path is incomplete).

The system would fail items 1, 2, 3 (partial), and 5 at a compliance test today.

---

## Section 4: Priority Findings and Recommendations

### P1 — Critical (Blocking for any US flight)

1. **Authentication message**: Add `AuthenticationMessage` struct and implement F3411-22a Section 5.4.3 encoding. At minimum, the basic auth type with UNIX timestamp and nonce. Without this, the message set is incomplete and non-compliant.

2. **Wire format encoding**: Implement `encode_basic_id()`, `encode_location()`, `encode_authentication()`, `encode_self_id()`, `encode_system()` to produce the 25-byte ASTM packed binary format. Consider a message pack encoder as well for Bluetooth efficiency.

3. **Static message interval**: Change from a shared 3-second round-robin to either (a) per-message-type timers each firing at 3 seconds, or (b) a message pack broadcast every 3 seconds containing all static messages. The current 12-second worst-case interval violates the standard.

4. **MAVLink OPEN_DRONE_ID_* message handling**: Implement inbound parsing of `OPEN_DRONE_ID_ARM_STATUS`, `OPEN_DRONE_ID_BASIC_ID`, `OPEN_DRONE_ID_LOCATION`, `OPEN_DRONE_ID_SYSTEM`, and outbound serialization of the full message set toward the companion broadcast module.

5. **UTC timestamp / RTC dependency**: The System message requires UTC wall-clock time. Block Remote ID from operating without a GPS time fix, or at minimum set the timestamp from GPS epoch. This requires `meridian-rtc` to be implemented first.

### P2 — High (Required for production)

6. **ADS-B backends**: Implement at minimum the uAvionix MAVLink backend (MAVLink #246 parsing). Without it, the threat detection logic has no input.

7. **CTA-2063-A format validation**: Validate serial number format on `set_uas_id()`. Reject or log non-conforming IDs.

8. **3D threat assessment**: Add vertical separation to `threat_level()`. At minimum, suppress Alert/Warning if vertical separation exceeds 500 ft.

### P3 — Moderate

9. **Avoidance integration hook**: `threat_level()` should feed a system-level avoidance callback, not just return a value to a caller that may not act on it.

10. **Takeoff position capture**: Auto-populate `system.operator_latitude/longitude` from GPS at transition to armed state.

---

## Conclusion

The `meridian-opendroneid` crate has advanced beyond pure stub — the broadcast state machine architecture, timing logic, pre-arm gate design, and message type taxonomy are well-structured and mirror ArduPilot's implementation intent. The `meridian-adsb` crate has solid threat classification math and a working aircraft database.

However, neither crate is functional for actual flight. The OpenDroneID crate cannot transmit any Remote ID signal — there is no Authentication message, no wire format encoding, no MAVLink message handling, and no physical transport driver. The broadcast rate for static messages is non-compliant. The ADS-B crate's threat logic is complete but its hardware backends return `None` unconditionally. 

The `identification_payload_infra.md` audit correctly characterizes OpenDroneID as CRITICAL priority. It must be resolved before any US airspace operation. The gaps are well-understood; the implementation path is clear. What is needed is execution.

**Rating: NON_COMPLIANT**  
The system cannot pass an FAA Remote ID compliance test in its current state. The structural foundation is sound; the execution layer is absent.
