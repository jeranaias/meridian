# Panel 11 — Safety Architecture Review
## Reviewer: Phil Koopman, Carnegie Mellon University
### Domain: Failsafe, Watchdog, Fault Detection, Mode Transitions

---

## Scope

Files reviewed:
- `crates/meridian-failsafe/src/lib.rs`
- `crates/meridian-arming/src/lib.rs`
- `crates/meridian-modes/src/multirotor.rs` (Land, RTL, and mode dispatch)
- `docs/PANEL_WAVE1_SUMMARY.md`

---

## Overall Rating: UNSAFE

This codebase has a safety architecture that is structurally sound in its individual monitor designs, but contains multiple unmitigated hazards that could result in an airborne vehicle failing to land safely or continuing to fly with a stalled main loop. Several of these were flagged by Wave 1 and remain unresolved at the integration level. I add new findings and deepen the existing ones from a systems-safety perspective.

---

## Finding 1 — CRITICAL: Watchdog Is Software-Only and Has No Actuation Path

**Severity: UNSAFE**

The `WatchdogMonitor` in `lib.rs` (lines 431–488) is a pure software timer. When `feed()` is not called within 2 seconds, `check()` returns `WatchdogState::MotorMin`; after 3 seconds, `WatchdogState::Disarm`.

The problem: **there is no hardware enforcement**. If the main loop stalls — the exact condition a watchdog is designed to catch — the same stalled loop that stopped calling `feed()` also stops calling `check()`. The watchdog's output is never read, and the motors are never cut.

A correct watchdog pattern requires one of:
1. A dedicated hardware watchdog peripheral (STM32 IWDG/WWDG) that resets or disarms the vehicle independently of main loop execution.
2. A high-priority interrupt-driven monitor that reads `last_feed` and cuts motor PWM directly via hardware timer compare outputs.

As implemented, `WatchdogMonitor::check()` is ceremonial. It does nothing when it matters most. This is a classic "I verified the watchdog fires in testing" trap — it fires because the test loop calls `check()`. In a real stall it cannot.

**Reference**: `PANEL_WAVE1_SUMMARY.md` confirms individual failsafe monitors are "confirmed correct," but the integration gap — who calls `check_watchdog()` and what happens when they can't — is unaddressed.

---

## Finding 2 — CRITICAL: RTL Land Phase Cuts Motors on Barometric Altitude with No Land-Detector Confirmation

**Severity: UNSAFE**

In `update_rtl()` (lines 925–937):

```rust
RtlState::Land => {
    let target_alt = (input.altitude - self.rtl_config.land_speed * dt).max(0.0);
    if input.altitude < 0.3 {
        self.rtl_state = RtlState::Complete;
    }
    ...
}
RtlState::Complete => ModeOutput::Idle,
```

`ModeOutput::Idle` is motor shutdown. The trigger is `input.altitude < 0.3` — a raw barometric altitude reading with no land-detector confirmation whatsoever.

Baro altitude has well-known error sources: pressure waves from rotor downwash (ground effect), sensor drift in cold temperatures, and initialization offsets. A baro reading of 0.3m does not mean the vehicle is on the ground. In ground effect at 0.5–1.5m, baro can read lower than actual height due to increased static pressure from rotor wash.

The `update_land()` function (lines 964–1001) implements a 3-condition land detector (altitude + vertical speed + not-ascending + 1s timer). The RTL land phase uses none of these conditions. This is an **inconsistency between the two landing paths** — Land mode uses a detector, RTL Land does not.

**Failure mode**: Vehicle in RTL descends over uneven terrain or through a pressure disturbance. Baro reads < 0.3m while vehicle is at 0.8m. Motors cut. Vehicle drops 0.8m. At the payload densities typical for multirotors, this is a hard landing. With a payload or fragile frame, it is a crash.

Wave 1 (Mackay) flagged this at a high level. This review specifies the exact mechanism.

---

## Finding 3 — HIGH: Land Detector in Land Mode Uses Only 3 of the 8 Required Conditions

**Severity: REVIEW (blocking for autonomous flight)**

`update_land()` (lines 968–995) documents that 5 of the 8 ArduPilot land detector conditions are deferred to "the external LandDetector in meridian-failsafe." That external LandDetector, however, is not wired into the `update_land()` decision. The motor shutdown at line 993 (`if self.land_complete_timer > 1.0`) depends only on the 3 inline conditions.

The missing conditions (attitude error, accel near 1G, throttle mix ratio) exist precisely to handle the case where the vehicle is still airborne but sinking slowly near the ground — for example, during a controlled autorotation or when being caught by a net. Shutting down motors based on altitude and vertical speed alone will trigger in those scenarios.

This is the same gap Mackay identified (PANEL_WAVE1_SUMMARY item 9), but the code comment on line 974 implies the external LandDetector handles this — it does not. The comment is misleading and masks a safety gap.

---

## Finding 4 — HIGH: Failsafe Priority Escalation Is One-Way But Clearing Is Not Symmetric

**Severity: REVIEW**

The `FailsafeManager` implements monotonic escalation correctly: once `current_priority` is raised, lower-priority events are ignored. However, `is_cleared()` (lines 850–873) contains this:

```rust
FailsafeReason::BatteryLow | FailsafeReason::BatteryCritical => false, // never clears
```

Battery failsafes never clear. This is the correct policy for BatteryCritical, but BatteryLow clearing is debatable — some landing scenarios restore apparent voltage (load removed on touchdown). The hard "never clears" for BatteryLow means that once triggered, battery failsafe priority permanently holds, potentially blocking lower-priority geofence or terrain failsafes from being re-evaluated after the aircraft lands and re-arms.

More importantly: `current_priority` recalculates correctly on `is_cleared()`, but the battery entry remains in `self.active` permanently. Over a multi-leg mission this means the active list grows without bound (bounded only by the `heapless::Vec<_, 16>` capacity). At 16 entries of permanent BatteryLow events, the active list saturates and subsequent `push()` silently fails — new failsafe events are lost.

**Failure mode**: After 16 battery-low events over multiple flights, the active list is full. A new Crash or ThrustLoss event cannot be pushed. The vehicle has a Crash event that the system cannot act on.

---

## Finding 5 — HIGH: The Software Watchdog Has No Graduated Response — MotorMin Is Not Actionable

**Severity: REVIEW**

Even granting that `check_watchdog()` is called by something outside this crate, the `WatchdogState::MotorMin` state prescribes cutting motors to minimum. For a multirotor at altitude, motors-to-minimum means immediate uncontrolled descent and likely a crash.

An embedded safety watchdog should either:
- Reset the processor to a known safe state and re-establish control (requires hardware watchdog + fast boot).
- Cut all motor outputs immediately (since "minimum" throttle mid-air is not safe, only ground idle is safe, and we don't know if we're on the ground).

The 2-second MotorMin window before Disarm is a grace period that provides no benefit in a genuine stall scenario — the vehicle will be in uncontrolled flight for those 2 seconds — and actively delays the safer Disarm state.

Recommendation: Remove `MotorMin` from the watchdog response. Watchdog → immediate `Disarm` (or hardware reset if available).

---

## Finding 6 — MEDIUM: ArmingCheck Disabled-Mask Can Bypass EKF Health Check

**Severity: REVIEW**

In `check_prearm()` (arming/src/lib.rs, lines 207–217), EKF health is gated behind `is_enabled(ArmingCheck::EkfHealth)`. The `disabled_mask` can turn this off:

```rust
if is_enabled(ArmingCheck::EkfHealth) {
    ...
}
```

Tridgell's Wave 1 item 2 is "No EKF health gate in arming path." While this code does implement the gate, it is bypassable at configuration time with no additional protection. A misconfigured `disabled_mask = 0xFFFF` silently disables all checks, including the EKF check that is described as a hard block.

There is no minimum required set of checks that cannot be disabled. Safety-critical systems should have a non-maskable core that remains active regardless of parameter settings. At minimum, `BatteryLevel`, `EkfHealth`, and `InsHealth` should not be disableable in flight configurations.

---

## Finding 7 — MEDIUM: IMU Consistency Check Has No 10-Second Sustain Timer at Arming

**Severity: REVIEW**

The `check_imu_consistency()` function (arming/src/lib.rs, lines 336–374) does the instantaneous check correctly. The comment on line 335 states "the caller should track sustain time externally." The comment in `PANEL_WAVE1_SUMMARY.md` under "NEEDS WORK" states: "No 10s sustain timer on IMU consistency check."

The code then immediately goes ahead and calls `check_imu_consistency()` in `check_prearm()` (line 313) without any sustain timer. The pre-arm result can fail on a single bad sample during a vibration event that lasts under one millisecond.

This is not just a false-positive issue. The real hazard is the inverse: a transient but real IMU divergence that resolves in < 10 seconds allows arming. The purpose of the 10-second sustain is to ensure the IMUs are stable, not just instantaneously consistent.

The pre-arm check at line 313 is a duplicate of the check at line 199 (using `state.imu_consistent`), creating two inconsistent evaluation paths.

---

## Finding 8 — MEDIUM: CrashDetector Clears on Disarm, Masking Crash Evidence

**Severity: REVIEW**

`CrashDetector::check()` (lines 250–268) resets `inverted_count` and `triggered` when `armed == false`. This means if a vehicle crashes and the disarm path runs before the crash detector check, the triggered state is cleared before it can be acted upon.

In the priority model, Crash is the highest-priority failsafe (priority 12). But if the disarm pathway runs first (e.g., via the software watchdog or RC failsafe), the crash event is never logged or acted upon as a crash. The post-flight investigation has no record of the crash condition being detected.

This is a logging and fault isolation problem as much as a safety problem, but given that logging is already confirmed broken (PANEL_WAVE1_SUMMARY item 14), it means crash events are completely undetectable post-flight.

---

## Finding 9 — LOW: Motor Imbalance Detector Permanently Latches After First Trigger

**Severity: REVIEW**

`MotorMonitor::check()` (lines 210–228) sets `self.triggered = true` and never resets it except through a full re-instantiation of the struct. This means a motor imbalance during a spool-up transient permanently latches the failsafe.

`TimeoutMonitor` has a `clear()` method; `MotorMonitor` does not. If the system attempts to re-arm after a false-positive motor imbalance trigger, the motor failsafe fires immediately.

More seriously: `FailsafeManager::is_cleared()` checks `MotorFailure` via `!self.motor.is_triggered()` — which can never become false once triggered. This means a MotorFailure event permanently holds in `self.active` and permanently occupies a slot in the heapless vec (contributing to the saturation issue in Finding 4).

---

## Summary Table

| # | Finding | Severity | File |
|---|---------|----------|------|
| 1 | Watchdog is software-only — no actuation on main loop stall | CRITICAL/UNSAFE | failsafe/lib.rs:431 |
| 2 | RTL Land cuts motors on baro < 0.3m with no land detector | CRITICAL/UNSAFE | multirotor.rs:927 |
| 3 | Land mode motor shutdown uses 3/8 land detector conditions | HIGH/REVIEW | multirotor.rs:982 |
| 4 | Battery failsafe never clears; active list saturation at 16 entries | HIGH/REVIEW | failsafe/lib.rs:855 |
| 5 | Watchdog MotorMin state is unsafe at altitude | HIGH/REVIEW | failsafe/lib.rs:469 |
| 6 | disabled_mask can bypass EKF health check; no non-maskable floor | MEDIUM/REVIEW | arming/lib.rs:207 |
| 7 | IMU consistency check has no 10s sustain timer; duplicate check paths | MEDIUM/REVIEW | arming/lib.rs:313 |
| 8 | CrashDetector clears on disarm; crash evidence destroyed | MEDIUM/REVIEW | failsafe/lib.rs:253 |
| 9 | MotorMonitor permanently latches — never clearable | LOW/REVIEW | failsafe/lib.rs:219 |

---

## Recommended Actions (Priority Order)

**Before any powered outdoor flight:**

1. **Finding 1**: Replace `WatchdogMonitor` software timer with an actual hardware watchdog peripheral call (STM32 IWDG). The software timer can remain as a secondary alarm, but motor cutoff must be enforced by hardware.

2. **Finding 2**: RTL Land phase must call the same land-detector logic as `update_land()`. Replace the bare `input.altitude < 0.3` check with a call to the existing 3-condition check (at minimum), and wire in the external LandDetector conditions when available. Do not transition to `ModeOutput::Idle` without confirmed landing.

3. **Finding 3**: Wire the `LandDetector` from meridian-failsafe into `update_land()`'s motor-shutdown decision. The comment on line 974 claims it is checked externally — make that explicit and verified, not assumed.

**Before extended flight testing:**

4. **Finding 4**: Add a `clear_battery_low()` method to `BatteryMonitor` and `FailsafeManager`. Cap the permanent-latch behavior to `BatteryCritical` only. Add an assertion or overflow handler to the heapless vec push path.

5. **Finding 5**: Remove `WatchdogState::MotorMin`. Watchdog violation goes directly to `Disarm`.

6. **Finding 6**: Add a `NON_MASKABLE_CHECKS` constant (EkfHealth, InsHealth, BatteryLevel) that cannot be included in `disabled_mask`.

7. **Finding 7**: Implement a sustain counter in `check_prearm()` for IMU consistency, or remove the duplicate check at line 313 and rely only on the caller-provided `imu_consistent` with documented external sustain requirement.

8. **Finding 9**: Add a `clear()` method to `MotorMonitor` (analogous to `TimeoutMonitor::clear()`).

---

*Review conducted against code as read on 2026-04-02. Individual monitor algorithms (TimeoutMonitor, BatteryMonitor, EkfHealthMonitor, GeofenceMonitor) are structurally correct and match ArduPilot reference behavior. The issues identified are in integration, actuation, and edge-case handling — not in the core monitoring logic.*
