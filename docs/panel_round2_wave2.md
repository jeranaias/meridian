# Panel Round 2 — Wave 2 Verification
**Personas: Randy Mackay (P4), Peter Barker (P5), Henry Wurzburg (P6)**
**Date: 2026-04-02**
**Scope: Verify Round 1 fix claims against current source**

---

## Persona 4 — Randy Mackay (Modes / Mission / Safety)

**File reviewed:** `crates/meridian-modes/src/multirotor.rs`,
`crates/meridian-failsafe/src/lib.rs`,
`crates/meridian-arming/src/lib.rs`

---

### Item 4.1 — Land detector: full 8-condition check in Land mode

**Status: FIXED**

`update_land()` (line 986) explicitly lists all 8 ArduPilot conditions inline:

1. `throttle_at_min` — motors at spin_min
2. `throttle_mix_at_min` — attitude corrections negligible
3. `small_angle_error` — attitude error < 30 deg
4. `accel_near_1g` — not in free-fall or vibrating
5. `low_vspeed` — |climb_rate| < 0.5 m/s
6. `not_ascending` — climb_rate ≤ 0
7. `rangefinder_near_ground` — condition 7
8. `land_complete_timer > 1.0` — conditions sustained ≥ 1 s (the timer)

The comment block names the source (`land_detector.cpp`) and each condition is tagged. Motor shutdown fires `ModeOutput::Idle` only after the 1-second timer expires. This matches ArduCopter behavior exactly.

One observation worth noting: condition 8 in the code is the sustain timer, not an eighth boolean input flag. This is correct — ArduPilot's condition 8 is the hold timer, not a separate sensor. The labeling is accurate.

---

### Item 4.2 — Land detector: wired into RTL Land phase

**Status: FIXED — with a minor consistency note**

`RtlState::Land` (line 925) reproduces the same 7 boolean conditions plus the `land_detect_timer` sustain check — identical logic to `update_land()`. When the timer exceeds 1.0 s, the state machine advances to `RtlState::Complete` which returns `ModeOutput::Idle`.

The minor note: the RTL Land phase uses a separate timer field (`land_detect_timer`) while Land mode uses `land_complete_timer`. The logic is functionally identical but the duplication means a future refactor could diverge them. Not a correctness defect — just a maintenance note. The check at `land_detect_timer > 1.0` before transitioning to `Complete` is correct.

---

### Item 4.3 — Battery failsafe inversion: `should_suppress_ext()` suppresses when LANDED

**Status: FIXED**

`should_suppress_ext()` (line 692, `meridian-failsafe/src/lib.rs`):

```rust
FailsafeReason::BatteryLow | FailsafeReason::BatteryCritical => {
    // Suppress battery failsafe while LANDED (not while airborne).
    self.battery_continue_if_not_landed && is_landed
}
```

The inversion is correct: suppression fires when `is_landed == true`, meaning the battery failsafe is held while on the ground. While airborne (`is_landed == false`), the failsafe fires normally. The test at line 2452 validates both cases explicitly:

- `is_landed = true` → suppressed (no failsafe on pad)
- `is_landed = false` → not suppressed (failsafe fires in flight)

Round 1 fix confirmed. The original inversion (suppressing while airborne) is gone.

---

### Item 4.4 — Mode check at arm: blocks dangerous modes

**Status: FIXED**

`check_prearm_for_mode()` (line 310, `meridian-arming/src/lib.rs`) implements two checks:

**L1 — Dangerous mode block:**
- Acro (mode 1) and Sport (mode 13) blocked if AHRS attitude is invalid
- Flip (mode 14) blocked unconditionally
- Comment: "Flip is never safe to arm into"

**L2 — Auto mode mission check:**
- Auto (mode 3) blocked if `!state.mission_loaded`
- Error message: "No mission for Auto"

Tests at lines 607 and 617 verify both. Flip arm-block test passes. Auto-without-mission test passes.

---

### Item 4.5 — GPS skip for non-GPS modes (Stabilize/AltHold arm without GPS)

**Status: FIXED**

`ArmingMode` enum (line 152) distinguishes `RequiresGps` from `NoGpsRequired`.

`check_prearm_for_mode()` (line 210):
```rust
if is_enabled(ArmingCheck::GpsLock) && arming_mode == ArmingMode::RequiresGps {
    // GPS checks only run for GPS-requiring modes
}
```

When `ArmingMode::NoGpsRequired` is passed (Stabilize, AltHold, Acro), GPS lock and satellite count checks are skipped. Test at line 569 confirms: arming into a GPS-required mode fails without fix; arming into NoGpsRequired passes without fix.

The backwards-compatible `check_prearm()` (line 162) defaults to `RequiresGps` so existing callers are not broken.

---

### Item 4.6 — RTL→Land GPS fallback: failsafe downgrades RTL to Land without GPS

**Status: FIXED**

`resolve_failsafe_action()` (line 1511, `meridian-failsafe/src/lib.rs`):

```rust
pub fn resolve_failsafe_action(action: FailsafeAction, gps_valid: bool) -> FailsafeAction {
    if !gps_valid {
        match action {
            FailsafeAction::ReturnToLaunch => FailsafeAction::Land,
            FailsafeAction::SmartReturnToLaunch => FailsafeAction::Land,
            ...
        }
    }
}
```

Both RTL and SmartRTL are downgraded to Land when GPS is invalid. Three tests at line 2530 confirm: RTL without GPS → Land, SmartRTL without GPS → Land, RTL with GPS → RTL preserved. The comment explicitly names the failure mode this prevents ("EKF navigates blind").

---

### Item 4.7 — Ground idle disarm: 15-second timer

**Status: FIXED**

`GroundIdleDisarm` struct (line 1532, `meridian-failsafe/src/lib.rs`):

```rust
pub struct GroundIdleDisarm {
    pub timeout_s: f32,   // default 15.0
    idle_time: f32,
    triggered: bool,
}
```

`new()` (line 1543) sets `timeout_s: 15.0`. The `update()` method accumulates idle time when `armed && on_ground && throttle_low`, resets on throttle input, and returns `true` when `idle_time >= timeout_s`.

Three tests at line 2557 verify: triggers after 15 s, resets on throttle, does not trigger when disarmed.

**One gap:** This timer lives in `meridian-failsafe` but there is no evidence in this review scope of the vehicle loop calling `ground_idle_disarm.update()` and acting on the result. The implementation is correct in isolation. Whether it is plumbed into the vehicle main loop is outside the scope of what was checked here — that connection needs verification separately.

---

### Mackay Round 2 Summary

| Item | Status |
|------|--------|
| 4.1 Land detector 8-condition in Land mode | **FIXED** |
| 4.2 Land detector wired into RTL Land phase | **FIXED** |
| 4.3 Battery failsafe suppress-when-LANDED (not airborne) | **FIXED** |
| 4.4 Mode check at arm (dangerous modes blocked) | **FIXED** |
| 4.5 GPS skip for non-GPS modes | **FIXED** |
| 4.6 RTL→Land GPS fallback in failsafe | **FIXED** |
| 4.7 Ground idle disarm 15 s timer | **FIXED** (implementation; plumbing unverified) |

All seven items that were tagged as broken in Round 1 are now correctly implemented at the source level. The one caution: ground idle disarm needs a vehicle-loop integration check before it can be called fully closed.

---

---

## Persona 5 — Peter Barker (Infrastructure)

**Files reviewed:** `crates/meridian-mavlink/src/server.rs`,
`crates/meridian-mavlink/src/adapter.rs`,
`crates/meridian-mavlink/src/v2.rs`,
`crates/meridian-drivers/src/battery_analog.rs`

---

### Item 5.1 — TIMESYNC handler in server.rs

**Status: FIXED**

`InboundCommand::Timesync` handler (line 501, `server.rs`):

```rust
InboundCommand::Timesync { tc1, ts1 } => {
    if *tc1 == 0 {
        let our_time_ns = (now_ms as i64) * 1_000_000;
        let n = self.adapter.encode_timesync(*ts1, our_time_ns, remaining);
        offset += n;
    }
}
```

Protocol is correct: GCS sends `tc1=0, ts1=gcs_time_ns`; autopilot echoes `tc1=ts1, ts1=our_time_ns`. This matches the MAVLink TIMESYNC spec (msg 111). The boot_time_ns field is tracked for continuity. The `encode_timesync()` call in adapter.rs uses `MSG_TIMESYNC = 111` (confirmed in `v2.rs` line 44).

---

### Item 5.2 — MAV_CMD_REQUEST_MESSAGE ID = 512

**Status: FIXED**

`adapter.rs` line 530:

```rust
512 => { // MAV_CMD_REQUEST_MESSAGE
    let msg_id = ...;
    Some(InboundCommand::Command { command, result: CommandAction::RequestMessage(msg_id) })
}
```

ID 512 is used, matching the MAVLink spec for `MAV_CMD_REQUEST_MESSAGE`. The parsing extracts the requested message ID from payload bytes 0–3. Correct.

---

### Item 5.3 — SET_MESSAGE_INTERVAL (511) handler

**Status: FIXED**

`adapter.rs` line 537:

```rust
511 => { // MAV_CMD_SET_MESSAGE_INTERVAL
    let msg_id = ...;
    let interval_us = ...;
    Some(InboundCommand::Command {
        command,
        result: CommandAction::SetMessageInterval { msg_id, interval_us },
    })
}
```

And `CommandAction::SetMessageInterval` is defined (line 684) with both `msg_id: u32` and `interval_us: i64`. The command parses correctly.

**One open question:** Whether the server.rs dispatch loop acts on `SetMessageInterval` to actually alter streaming rates is not evidenced in the server update path reviewed here. The decode side is present. The effect side (storing the interval and using it to gate telemetry) would need a separate check.

---

### Item 5.4 — MISSION_CURRENT streaming at 1 Hz (msg 42)

**Status: FIXED**

`server.rs` (line 314):

```rust
// MISSION_CURRENT at 1 Hz
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

`encode_mission_current()` in adapter.rs uses `MSG_MISSION_CURRENT = 42` (confirmed in `v2.rs` line 45, `const MSG_MISSION_CURRENT: u32 = 42`). The CRC extra byte for msg 42 is registered as 28 (line 77). The 1 Hz gate uses a wrapping subtraction, which handles the `NEVER_SENT` sentinel and millisecond counter rollover correctly.

---

### Item 5.5 — BattMonitor_Analog: real implementation exists

**Status: FIXED**

`crates/meridian-drivers/src/battery_analog.rs` is a full implementation, not a stub. It includes:

- `BattMonitorConfig` with voltage/current pin, scale factors, offsets (MatekH743 defaults)
- IIR low-pass filter (α=0.1 at 10 Hz = ~1 s time constant, matches ArduPilot)
- Current integration to mAh (`accumulated_as / 3.6`)
- Dual remaining% estimation: capacity-based (if `BATT_CAPACITY > 0`) and voltage-curve-based
- Cell count auto-detection from initial voltage (1S–14S, LiPo chemistry)
- Health check (`voltage > MIN_VALID_VOLTAGE`)
- MAVLink-friendly outputs: `voltage_mv: u16`, `current_ca: i16`
- 10 unit tests covering initialization, filtering, current integration, capacity tracking, MAVLink units, negative clamp

This is substantively correct. The filter constant, integration method, and scale factors match the ArduPilot source (`AP_BattMonitor_Analog.cpp`).

---

### Item 5.6 — Sensor mask: GPS=0x08, DIFF_PRESSURE=0x10 (no collision)

**Status: FIXED**

Constants in `adapter.rs` lines 54–58:

```rust
pub const MAV_SYS_STATUS_SENSOR_3D_GYRO:              u32 = 0x01;
pub const MAV_SYS_STATUS_SENSOR_3D_ACCEL:             u32 = 0x02;
pub const MAV_SYS_STATUS_SENSOR_3D_MAG:               u32 = 0x04;
pub const MAV_SYS_STATUS_SENSOR_GPS:                  u32 = 0x08;
pub const MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE: u32 = 0x10;
pub const MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:    u32 = 0x20;
```

Each bit is unique — no collision. GPS=0x08 and DIFF_PRESSURE=0x10 are distinct.

`DIFFERENTIAL_PRESSURE` is defined but not set in `present_mask()` or `health_mask()`. This means airspeed sensor health is not reported in `SYS_STATUS`. This is acceptable if the platform has no airspeed sensor (multirotor default), but a fixed-wing build with airspeed hardware will silently report no airspeed sensor to the GCS. Worth tracking as a follow-on item but not a collision defect.

---

### Barker Round 2 Summary

| Item | Status |
|------|--------|
| 5.1 TIMESYNC handler | **FIXED** |
| 5.2 MAV_CMD_REQUEST_MESSAGE ID = 512 | **FIXED** |
| 5.3 SET_MESSAGE_INTERVAL (511) decoder | **FIXED** |
| 5.4 MISSION_CURRENT streaming at 1 Hz (msg 42) | **FIXED** |
| 5.5 BattMonitor_Analog: real implementation | **FIXED** |
| 5.6 Sensor mask no collision (GPS=0x08, DP=0x10) | **FIXED** |

All six items are correctly implemented at the layer reviewed. Two residual questions for the next pass: (a) does the server actually apply `SetMessageInterval` to live streaming rates? (b) does DIFF_PRESSURE get reported for fixed-wing builds with airspeed sensors?

---

---

## Persona 6 — Henry Wurzburg (Fixed-Wing)

**File reviewed:** `crates/meridian-modes/src/fixed_wing.rs`

---

### Item 6.1 — Shadow TECS removed: fixed_wing.rs uses meridian_control::tecs::Tecs

**Status: FIXED**

Import at line 12:

```rust
use meridian_control::tecs::{Tecs, TecsParams};
```

`FixedWingModes` holds `pub tecs: Tecs` (line 116), constructed as `Tecs::new()` (line 155). The `tecs_update()` helper (line 224) delegates to:

```rust
self.tecs.update_50hz(current_alt, climb_rate, current_speed, accel_x);
let out = self.tecs.update(target_alt, target_speed, dt);
```

The comment at line 222 explicitly says "Replaces the old shadow TECS inline .update() which was a broken mini-controller used for ALL FW auto modes." All auto modes (Auto, RTL, Loiter, Circle, Guided, FBWB, Cruise, Takeoff) route through `tecs_update()`. There is no competing local energy controller anywhere in the file.

---

### Item 6.2 — Landing flare calls tecs.update_flare()

**Status: FIXED**

`LandState::Flare` handler (line 643):

```rust
self.tecs.update_50hz(
    input.altitude, 0.0,
    input.airspeed.max(input.ground_speed), 0.0,
);
let flare_out = self.tecs.update_flare(
    input.altitude,
    self.params.land_approach_speed,
    distance_past_land,
    dt,
);

ModeOutput::FixedWingTarget {
    roll: 0.0,
    pitch: self.clamp_pitch(flare_out.pitch),
    throttle: flare_out.throttle,
    yaw_rate: 0.0,
}
```

The TECS state estimator is fed first (`update_50hz`), then `update_flare()` is called with AGL altitude, approach speed, and distance-past-touchdown. Pitch and throttle come from the flare output, not from hardcoded constants. The comment at line 644 explicitly labels this "WZ2: Wire the real TECS flare function instead of hardcoded pitch/throttle."

The `distance_past_land` calculation is geometrically correct: it uses the dot product of velocity against the direction-to-touchdown vector to determine when the aircraft has passed the threshold.

---

### Item 6.3 — QuadPlane stall guard: transition blocks FBWA below stall speed

**Status: FIXED**

`update_vtol_transition()` (line 713):

```rust
// WZ3 safety guard: do NOT apply FBWA surface commands below stall speed.
// Below stall speed, fixed-wing surfaces are ineffective and can cause
// a departure. Output Idle to let multirotor motors handle authority.
if input.airspeed < self.params.airspeed_min {
    return ModeOutput::Idle;
}
```

The guard uses `airspeed_min` (default 12.0 m/s) as the stall threshold. Below stall speed, `ModeOutput::Idle` is returned, yielding control authority to the multirotor side. Above stall speed, FBWA control surfaces take over. Source is attributed to `ArduPlane/quadplane.cpp transition_state check`.

This is the right behavior. The only limitation is that `airspeed_min` is a static parameter — there is no dynamic stall model — but that matches what ArduPilot does in practice.

---

### Item 6.4 — Speed scaling NOT applied to FBWA attitude targets

**Status: FIXED**

`update_fbwa()` (line 361):

```rust
// WZ4 fix: speed_scaling must NOT be applied to the target roll/pitch angle.
// In ArduPilot, FBWA maps pilot stick directly to target attitude angles.
// Speed scaling is applied at the servo output level by the attitude controller,
// not here.
let roll = input.rc_roll * self.params.roll_limit;
```

No `speed_scaling()` call anywhere in `update_fbwa()`. The comment explicitly documents why (scaling belongs at servo output, not target). This matches ArduPilot FBWA behavior.

**Residual issue noted:** FBWB (`update_fbwb()`, line 379–381) still applies speed scaling to the roll target:

```rust
let scaling = self.speed_scaling(input.airspeed);
let roll = input.rc_roll * self.params.roll_limit * scaling;
```

This appears to be a leftover from before the WZ4 fix, since FBWB is similar to FBWA. ArduPilot FBWB also does not apply speed scaling to target angles — it applies at servo output. This is a new finding, not part of the original audit item. It does not affect the FBWA fix verdict but should be logged for the next round.

---

### Wurzburg Round 2 Summary

| Item | Status |
|------|--------|
| 6.1 Shadow TECS removed, uses meridian_control::tecs::Tecs | **FIXED** |
| 6.2 Landing flare calls tecs.update_flare() | **FIXED** |
| 6.3 QuadPlane stall guard blocks FBWA below stall speed | **FIXED** |
| 6.4 Speed scaling NOT applied to FBWA attitude targets | **FIXED** |

All four items are resolved. One new finding: FBWB still applies speed scaling to the roll target angle in the same manner that FBWA had before WZ4. Recommend adding a WZ4b item to the tracking list.

---

---

## Cross-Persona Observations

**Duplicate logic (Mackay note):** Land mode and RTL Land phase both implement the 8-condition check inline with separate timer fields. A `LandConditions::check()` helper shared between them would reduce the risk of future divergence.

**Plumbing completeness (Barker note):** Three fixed structures (`GroundIdleDisarm`, `SetMessageInterval` effect, DIFF_PRESSURE reporting) are correctly implemented but their connection to the running vehicle loop is not evidenced in this review's scope. These should be verified against `crates/meridian-vehicle/` or the SITL main loop before a flight readiness sign-off.

**FBWB speed scaling (Wurzburg finding):** New defect identified during this wave. Not a regression from Round 1 — it was not in the original audit list. Classify as WZ4b: *FBWB applies speed scaling to roll target (same as pre-fix FBWA).*

---

*End of panel_round2_wave2.md*
