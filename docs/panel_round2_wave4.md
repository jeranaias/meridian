# Panel Round 2 — Wave 4
_Verification of Round 1 fixes. Date: 2026-04-02._

---

## Persona 11 — Phil Koopman (Safety)

### K1: Hardware watchdog — IWDG register writes
**FIXED.**

`crates/meridian-platform-stm32/src/watchdog.rs` lines 136–163 contain real MMIO writes
inside `#[cfg(target_arch = "arm")]` unsafe blocks. The sequence is correct:

1. Write `IWDG_KEY_ENABLE (0xCCCC)` to KR — starts IWDG.
2. Write `IWDG_KEY_WRITE_ACCESS (0x5555)` to KR — unlocks PR/RLR.
3. Write `IWDG_PRESCALER_REG (4 = /64)` to PR.
4. Write `IWDG_RELOAD (1000 counts = 2 s)` to RLR.
5. Poll SR until PVU/RVU clear.
6. Write `IWDG_KEY_RELOAD (0xAAAA)` — first pat.

`pat()` (lines 189–193) writes `0xAAAA` to KR via `write_volatile`. This is a
genuine hardware watchdog, not a software timer. The non-ARM path is a no-op stub
behind `#[cfg(not(target_arch = "arm"))]` — correct behavior for SITL/tests.

The `check_reset_cause()` and persistent SRAM functions remain TODO-commented,
which is a documentation debt but not a safety regression on the watchdog itself.

### K2: RTL Land motor cutoff — full land detector vs. raw baro
**FIXED.**

`crates/meridian-modes/src/multirotor.rs` `update_rtl()` lines 925–958: The
`RtlState::Land` branch implements a 7-condition land check identical to
`update_land()` (throttle at min, throttle-mix at min, small angle error, accel
near 1g, low vertical speed, not ascending, rangefinder near ground), plus a 1 s
sustain timer (`land_detect_timer`). Motor cutoff happens at `RtlState::Complete`
→ `ModeOutput::Idle` only after all conditions are met for >1 s.

One residual concern: `SmartRtlState::Descending` (lines 1476–1487) still uses the
raw `input.altitude < 0.3` threshold for state transition to `Complete`, which then
returns `ModeOutput::Idle`. This path does not use the full land detector. This was
not the primary K2 finding (which targeted RTL mode), but it is a similar risk in
the SmartRTL descent path and should be tracked as a follow-on item.

### Battery failsafe Vec<16> saturation
**STILL_BROKEN (partially).**

The `heapless::Vec<(FailsafeReason, FailsafeAction), 16>` capacity is unchanged.
All `push()` calls discard the result with `let _ = ...` (e.g., lines 948, 951,
955, 958, 962–964, 975, 978). There is no overflow detection, no telemetry, and no
test that verifies behavior when more than 16 candidates are queued simultaneously.
With 10+ concurrent monitors (RC loss, GPS loss, comms, battery, EKF, dead
reckoning, terrain, GPS glitch, geofence, motor) it is arithmetically possible to
fill the vec and silently drop a lower-priority failsafe event. No fix has been
applied.

### Ground idle disarm timer
**FIXED.**

`crates/meridian-failsafe/src/lib.rs` lines 1524–1595: `GroundIdleDisarm` struct
and impl exist with configurable `timeout_s` (default 15 s). `update()` accumulates
`idle_time` when `on_ground && low_throttle && armed`, resets on throttle input or
disarm, and returns `true` (triggering disarm) at timeout. Three tests at lines
2557–2591 verify: trigger after 15 s, reset on throttle, no trigger when disarmed.

---

## Persona 12 — Nancy Leveson (System Safety)

### UCA-1: Arming in dangerous mode blocked
**ADDRESSED.**

`crates/meridian-arming/src/lib.rs` lines 310–323: arming is blocked when:
- Mode == Acro (1) or Sport (13) without a valid attitude estimate.
- Mode == Flip (14) unconditionally.

These are the modes identified in UCA-1 as requiring direct rate control without
adequate state estimation. Test `test_acro_no_attitude_fails` (line 601) and
`test_flip_mode_blocked` (line 611) confirm the guards. The check fires inside
`check_prearm_for_mode()`, which is the canonical arming gate.

### UCA-4: Auto mode without mission blocked
**ADDRESSED.**

Lines 326–329: `if state.flight_mode == 3 && !state.mission_loaded` adds a "No
mission for Auto" failure. `ArmingState` carries `mission_loaded: bool` and
`mission_item_count: u16` (lines 99–102). Tests `test_auto_no_mission_fails` (line
618) and `test_auto_with_mission_passes` (line 630) confirm correct behavior.

### UCA-5: RTL without GPS falls back to Land
**ADDRESSED.**

`crates/meridian-failsafe/src/lib.rs` lines 1499–1522: `resolve_failsafe_action()`
downgrades `ReturnToLaunch` and `SmartReturnToLaunch` to `Land` when `gps_valid ==
false`. `SmartRtlLand` also collapses to `Land`. Tests at lines 2531–2551 verify
all three downgrade cases and confirm RTL is preserved when GPS is valid.

Note: the caller is responsible for passing `gps_valid` correctly into
`resolve_failsafe_action()`. The `FailsafeOptions::resolve_action()` wrapper does
not automatically gate on GPS; integration-level review is needed to confirm the
function is invoked with live GPS status on every failsafe dispatch path.

### UCA-12: Dead man's switch (ground idle disarm)
**ADDRESSED.**

`GroundIdleDisarm` (lines 1524–1595) implements the dead man's switch: 15 s of
armed, on-ground, low-throttle state triggers auto-disarm. The struct is present in
the `FailsafeManager` codebase (see K4 above) and is tested. Integration into the
main vehicle loop is assumed but not verified in this pass.

---

## Persona 13 — Raffaello D'Andrea (Dynamics)

### octa_dji_x: no longer duplicate of octa_cw_x
**FIXED.**

`crates/meridian-mixing/src/lib.rs`:

`octa_cw_x()` (lines 438–449) is ordered strictly CW by motor index:
slots 0–7 at angles 22.5, 67.5, 112.5, 157.5, -157.5, -112.5, -67.5, -22.5 deg,
alternating CCW/CW yaw.

`octa_dji_x()` (lines 423–434) uses DJI interleaved ordering (1, 8, 7, 6, 5, 4, 3,
2 → slots 0–7): angles 22.5, -22.5, -67.5, -112.5, -157.5, 157.5, 112.5, 67.5 deg.

The slot sequences are distinct. `octa_dji_x` slot 1 is at -22.5 deg (Motor 8);
`octa_cw_x` slot 1 is at 67.5 deg (Motor 2). The yaw coupling signs differ per
slot as well. These are two genuinely different physical configurations. No test
asserts their inequality, but the definitions are correct.

### SITL yaw torque: no longer double-counts cmd
**FIXED.**

`crates/meridian-sitl/src/physics.rs` lines 147–150:

```rust
// DA1 fix: torque is proportional to thrust alone, not cmd*thrust.
// cmd is already baked into thrust via the expo curve above.
let yaw_torque = motor.yaw_factor * motor.yaw_coupling * thrust;
```

Prior to the Round 1 fix, the expression was `cmd * thrust` which double-applied
the throttle command (once through the expo thrust curve, once as a direct
multiplier). The fix drives yaw reaction torque from `thrust` only. The expo curve
(line 134: `linearized = (1-expo)*cmd + expo*cmd²`) already encodes `cmd`
nonlinearly into `thrust`, so this is the correct single-application path. The
comment explicitly records the reason for the fix.

---

## Summary Table

| ID | Finding | Persona | Status |
|----|---------|---------|--------|
| K1 | IWDG hardware register writes | Koopman | FIXED |
| K2 | RTL Land motor cutoff — full detector | Koopman | FIXED (SmartRTL residual) |
| K3 | Battery Vec<16> saturation | Koopman | STILL_BROKEN |
| K4 | Ground idle disarm timer | Koopman | FIXED |
| UCA-1 | Arming in dangerous mode blocked | Leveson | ADDRESSED |
| UCA-4 | Auto without mission blocked | Leveson | ADDRESSED |
| UCA-5 | RTL without GPS → Land | Leveson | ADDRESSED |
| UCA-12 | Dead man's switch | Leveson | ADDRESSED |
| DA1 | octa_dji_x distinct from octa_cw_x | D'Andrea | FIXED |
| DA2 | SITL yaw torque no double-count | D'Andrea | FIXED |
