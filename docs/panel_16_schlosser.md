# Panel Review 16 — DroneCAN / CAN Bus
**Reviewer**: Travis Schlosser, DroneCAN protocol maintainer  
**Date**: 2026-04-02  
**Scope**: `crates/meridian-can/src/` (frames.rs, node.rs, messages.rs, sensors.rs) vs. ArduPilot `AP_DroneCAN`, `AP_CANManager`  
**Reference**: `docs/identification_comms_protocols.md` §19–20

---

## Overall Rating: PARTIAL

The frame layer is correct and the DNA server is sound. Message coverage is thin — enough to spin ESCs and receive sensor telemetry, not enough to boot a real peripheral bus. The gaps identified below are real blockers for production use, not polish items.

---

## 1. Frame Encoding (29-bit Extended ID)

**Rating: COMPLIANT**

`encode_can_id` / `decode_can_id` in `frames.rs` are correct. The bit-field layout matches UAVCAN v0 exactly:

```
bits 28..24 — priority (5 bits)
bits 23..8  — data type ID (16 bits)
bit  7      — service_not_message
bits 6..0   — source node ID (7 bits)
```

The `extended: true` flag is set on every outbound frame. `process_rx` in `node.rs` correctly drops non-extended frames. The tail byte bit layout (SOT=7, EOT=6, toggle=5, transfer_id=4..0) is right. Roundtrip test passes with the known-good NodeStatus DTID 341. No issues here.

One minor point: `decode_can_id` does not mask out bit 7 from the type_id field when `service_not_message = 1`. In the service case, bit 7 of the 16-bit field is the request_not_response flag, not part of the type_id. The decode function returns the raw 16-bit value including that bit, which would give the wrong DTID if someone tries to use it for service dispatch. The code never does that today (services are not dispatched by the callback), but it is a latent correctness issue.

---

## 2. Multi-Frame CRC16

**Rating: COMPLIANT**

`crc16_ccitt` passes the canonical test vector `"123456789" → 0x29B1`. Polynomial 0x1021, init 0xFFFF, no final XOR — correct for UAVCAN v0.

The multi-frame encoder prepends the CRC as two bytes LE at the start of the first chunk, which is the UAVCAN v0 wire convention. The assembler strips those two bytes and re-checks them on EOT before returning the payload. CRC mismatch returns `None` and the test for corruption confirms this path works.

One thing to watch: `encode_multi_frame` computes the CRC over the original `payload` slice, then prepends `crc_bytes = crc.to_le_bytes()` — but the decode side computes the CRC over `buf[2..self.len]`, i.e., the received payload bytes (not including the CRC prefix). Those are the same bytes. The logic is consistent, just unusual to read.

The toggle sequence initializes at `false` for the first frame and flips after it, so the second frame carries `toggle=true`. The assembler sets `expected_toggle = true` after the SOT frame. This matches. UAVCAN v0 spec says the first frame always has toggle=0, which this implements correctly.

---

## 3. DNA Server

**Rating: PARTIAL**

### What is correct
- Allocation direction matches ArduPilot: downward from node ID 125.
- Skips node ID 10 (the FC's own ID). Good.
- `lookup` → `allocate` idempotency on re-request. Correct.
- The bus-full detection (wrap-around guard) is present.
- `save_to_bytes` / `restore_from_bytes` are implemented with a sensible binary format (1-byte count + 17 bytes per entry). The save/restore logic is correct and matches what ArduPilot stores.

### What is missing or wrong

**Multi-stage allocation handshake is not implemented.** This is a hard compliance gap.

Real DroneCAN DNA allocation is a three-round anonymous handshake. A requesting node sends `Allocation` with its full unique_id split across up to three frames. The server does not respond to partial requests until it has seen a complete unique_id transmission. The protocol uses the anonymous transfer mechanism (source_node=0) and a discriminator field in the CAN ID to prevent bus collisions.

`CanardNode.dna` has `allocate(unique_id: &[u8; 16])` which takes the entire 16-byte ID in one call. That is a convenient internal API, but there is no code in `process_rx` or `handle_message` that catches incoming `DNA_ALLOCATION_DTID` (DTID 1) frames, reassembles the multi-stage request, and calls `dna.allocate()`. Without this, peripherals will never receive a node ID and the bus stays dead.

**DNA_ALLOCATION_DTID is wrong.** `messages.rs` line 43 defines `DNA_ALLOCATION_DTID: u16 = 1`. That is the GetNodeInfo service DTID. The correct DTID for `uavcan.protocol.dynamic_node_id.Allocation` is **1.** — wait, both are 1? Let me be precise. In UAVCAN v0, GetNodeInfo is *service* ID 1, and dynamic node allocation *message* is also ID 1. They are distinguished by the `service_not_message` bit. Since `DNA_ALLOCATION_DTID` and `GETNODEINFO_DTID` are both set to 1 but one is a message and one is a service, using the same constant name for both is hazardous. The code compiles only because neither is wired into `process_rx` dispatch, but if someone adds dispatch on DTID 1 without checking `service_not_message` they will misroute frames. The constants need to be named and documented more carefully.

**No timeout/retry on DNA.** ArduPilot implements a 400 ms quiet-period check before responding to allocation requests. The spec requires the server to wait until the bus has been quiet for a period before responding. No timing logic exists here.

**Flash persistence is implemented but never called.** `save_to_bytes` / `restore_from_bytes` are correct functions, but nothing in `CanardNode` calls them. There is no HAL hook for flash read/write, no on-boot restore, no dirty-flag to trigger a save after a new allocation. The DNA table is re-built from scratch on every power cycle, which means peripherals get different node IDs across reboots. ArduPilot stores this in its parameter storage. Meridian has the serialization primitives but zero integration.

---

## 4. Flash Persistence

**Rating: NON_COMPLIANT (not integrated)**

As noted above: the byte-level save/restore API exists and is correct. It is dead code. A real flight controller needs to:

1. Read the DNA table from NVM at boot before the CAN bus comes up.
2. Save the table after any new allocation.
3. Provide a HAL abstraction so `CanardNode` can call `hal_nvm_write()` without knowing about flash drivers.

None of this is wired. `CanardNode` has no `save_dna` or `restore_dna` method and no reference to any NVM/flash HAL trait. This is a clean design to add, but it is not present.

---

## 5. Message Type Coverage

**Rating: PARTIAL — missing types block real peripheral usage**

### Implemented and correct
| Message | DTID | Notes |
|---------|------|-------|
| NodeStatus | 341 | Encode + decode, heartbeat at 1 Hz |
| Fix2 | 1063 | Full encode/decode, 72 bytes multi-frame |
| MagneticFieldStrength2 | 1002 | f16 encode/decode, 7 bytes |
| RawAirData | 1027 | 24 bytes, decode accepts 16+ bytes (partial OK) |
| RawCommand | 1030 | int14 bit-packing correct, tested |
| BatteryInfo | 1092 | 26 bytes, convention flip documented |
| LightsCommand | 1081 | RGB555 packing correct |
| SafetyState | 20000 | 1 byte |
| ArmingStatus | 1100 | 1 byte |
| EscStatus | 1034 | Decode only, 22 bytes |
| ActuatorArrayCommand | 1010 | Encode + decode stub present |
| ActuatorStatus | 1011 | Decode only |
| RangeSensorMeasurement | 1050 | Decode only |
| CircuitStatus | 1091 | Decode only |

### Missing types that block real peripheral usage

**uavcan.protocol.dynamic_node_id.Allocation (message 1)** — No wire encoding/decoding struct. Cannot run the DNA handshake.

**uavcan.protocol.GetNodeInfo response decode** — `GetNodeInfoResponse` has encode but no decode. The FC sends the request, gets back a multi-frame response with the node name and hardware unique_id, but cannot parse it. Peripheral enumeration stays blind.

**uavcan.protocol.param.GetSet (service 11)** — DTID constant defined, no struct. Cannot read or write parameters on any DroneCAN peripheral (ESC calibration, GPS config, etc.). This is the single most used service in AP_Periph.

**uavcan.protocol.RestartNode (service 5)** — DTID constant only. Cannot restart nodes after firmware update or configuration change.

**uavcan.equipment.esc.StatusExtended** — Not defined. Some ESCs (Myxa, Kotleta20) only publish StatusExtended. If they are on the bus the FC is blind to their telemetry.

**uavcan.equipment.rc.ChannelsPackedArray** — Not defined. Without this, DroneCAN RC input is impossible. If the RC receiver is a DroneCAN peripheral (common in long-range setups), there is no path to get RC channels.

**ardupilot.indication.Button** — Not defined. DroneCAN safety switch integration is broken.

**uavcan.equipment.gnss.Auxiliary** — Not defined. Provides satellite counts, DOP, and horizontal accuracy separately from Fix2. Used by AP_GPS_DroneCAN to populate GCS telemetry.

**Firmware update (file server, BeginFirmwareUpdate, FileRead)** — Entirely absent. In-field firmware updates over DroneCAN (the primary update path for commercial ESCs and GPS units) are not possible. This is a significant operational gap.

---

## 6. Fix2 Wire Format Divergence

**Rating: PARTIAL — functional but not wire-compatible**

The actual UAVCAN v0 Fix2 DSDL uses bit-packed fields, not a flat struct of native types. Specifically:

- `longitude_deg_1e8` is a signed 37-bit integer in the spec (not i64).
- `latitude_deg_1e8` is a signed 37-bit integer (not i64).
- The DSDL uses 56-bit microsecond timestamp, not u64.
- `sats_used` is u6 (not u8), `status` is u2, `mode` is u3, `sub_mode` is u6 — all packed together.
- The covariance array is a `dynamic` array of float16, not fixed-size float32.

Meridian's Fix2 uses full-width native types with flat byte layout, encoding 72 bytes. A real ArduPilot GPS peripheral emitting the spec-compliant packed DSDL Fix2 will produce a different byte sequence. The current decoder will parse garbage without error because it does not validate field ranges and the byte widths differ.

This needs a proper DSDL-aware encoder/decoder or an explicit note that Meridian only talks to GPS peripherals running a compatible simplified encoding. Neither option is acceptable for a production FC; the former is the right answer.

The same concern applies to MagneticFieldStrength2 (covariance field in the spec is a dynamic float16 array) and EscStatus (several bit-packed fields in the spec).

---

## 7. 8 KB Reassembly Buffer

**Rating: ADEQUATE for current use, marginal for production**

`TRANSFER_BUF_SIZE = 8192` in `frames.rs`. ArduPilot's libcanard default is also 8 KB. For standard sensor messages (Fix2 at 72 bytes, BatteryInfo at 26 bytes) this is more than sufficient.

The limitation surfaces with firmware update transfers. A typical AP_Periph firmware image is 250–400 KB. The file server protocol reads it in 200-byte chunks over the FileRead service, so each individual transfer is small, but you still need the 8 KB buffer present to handle a full GetNodeInfo response (node name + UID, up to ~130 bytes) and the firmware file chunks cleanly. Current size is fine for that.

The real concern is the stack copy in `handle_multi_frame`:

```rust
let mut tmp = [0u8; 2048];
let plen = assembled_payload.len().min(tmp.len());
```

This silently truncates any reassembled payload larger than 2048 bytes, even though the assembler can hold 8192. A 72-byte Fix2 is unaffected, but if a future message type produces a larger payload, the first 2048 bytes will be passed to the callback and the rest silently dropped without error. The truncation is noted in a comment but not signaled to the caller. This should either be removed (pass the full slice reference) or replaced with an explicit error.

---

## 8. Service Frame Encoding — Known Issue

`request()` in `node.rs` encodes the service destination node ID and request flag into the 16-bit type_id field (bit 15 = request, bits 14..8 = dest node, bits 7..0 = service type). This matches the UAVCAN v0 spec for service transfers.

However, the function only handles single-frame service requests (DLC up to 7 bytes). Service *responses* can be multi-frame (GetNodeInfo response is 50–130+ bytes). There is no `respond()` function and no path in `process_rx` to identify incoming service responses vs. message broadcasts. Since the FC acts as the client (not the server) for most services, incoming service responses need to be dispatched separately from broadcast messages. They currently are not. Any multi-frame service response will land in the multi-frame assembler keyed on (type_id, source_node) but once assembled will be passed to the same flat callback as message broadcasts. The callback has no way to know it is a service response without inspecting the CAN ID `service_not_message` bit, which is not included in the callback signature `fn(type_id, source_node, payload)`.

---

## Summary Table

| Criterion | Status | Notes |
|-----------|--------|-------|
| 29-bit extended CAN ID encoding | COMPLIANT | Bit layout correct, extended flag set |
| Multi-frame CRC16-CCITT | COMPLIANT | 0x29B1 vector passes, toggle sequence correct |
| DNA server allocation logic | PARTIAL | Correct algorithm; handshake FSM absent, not wired to RX |
| DNA flash persistence | NON_COMPLIANT | API exists, zero integration |
| NodeStatus heartbeat | COMPLIANT | 1 Hz, correct encoding |
| ESC RawCommand (int14 packing) | COMPLIANT | Bit-pack correct, tested |
| GPS Fix2 | PARTIAL | Functional internally; not DSDL-wire-compatible with real peripherals |
| MagneticFieldStrength2 | COMPLIANT | f16 conversion correct |
| RawAirData / BatteryInfo | COMPLIANT | Standard layout, correct |
| param.GetSet service | NON_COMPLIANT | No struct, no transport |
| Firmware update | NON_COMPLIANT | Entirely absent |
| RC channels (DroneCAN RC) | NON_COMPLIANT | No message type, no subscriber |
| Safety button subscriber | NON_COMPLIANT | No message type |
| 8 KB buffer adequacy | ADEQUATE | Marginal due to 2 KB stack-copy truncation |
| Service response dispatch | PARTIAL | No separate RX path; callback has no service flag |

---

## Priority Fixes Before Real Hardware Test

1. **Implement DNA allocation handshake FSM** — The allocator logic is correct; it just needs to be driven by incoming `Allocation` broadcast frames (source_node=0, DTID=1). Without this, nothing gets a node ID.

2. **Wire DNA save/restore to a NVM HAL trait** — Define `trait CanNvm { fn read(&self, buf: &mut [u8]); fn write(&self, buf: &[u8]); }` and add `fn save_dna(&self, nvm: &dyn CanNvm)` / `fn restore_dna(&mut self, nvm: &dyn CanNvm)` to `CanardNode`.

3. **Implement param.GetSet transport** — Virtually every DroneCAN peripheral requires this to be configured (ESC direction, motor pole count, GPS baud, etc.). No param protocol = no remote configuration.

4. **Fix Fix2 DSDL bit-pack** — Either implement proper DSDL bit-field encoding or document the encoding divergence and gate it behind a `cfg` flag so it can be replaced cleanly later.

5. **Remove the 2 KB stack-copy truncation** — Pass the assembler slice directly to the callback, or at minimum return an error when plen < assembled_payload.len().

6. **Add RC channels subscriber** — Required before DroneCAN-based RC input works at all.
