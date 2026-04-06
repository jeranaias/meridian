# Panel Review 12 — Nancy Leveson, MIT
## STPA Safety Analysis of Meridian Autopilot

**Domain**: System-level safety / STAMP (Systems-Theoretic Accident Model and Processes)  
**Date**: 2 April 2026  
**Files reviewed**: `crates/meridian-failsafe/src/lib.rs`, `crates/meridian-modes/src/multirotor.rs`, `crates/meridian-arming/src/lib.rs`, `docs/PANEL_WAVE1_SUMMARY.md`

---

## Framing Note

STPA does not ask "what can go wrong with a component?" It asks: "what system-level control actions, when provided or not provided, can lead to losses?" The distinction matters. Many of Meridian's individual algorithms are correct (Riseborough and Hall confirmed this). The danger is in the *control structure* — who issues what commands, under what assumed conditions, with what feedback. That is where I will focus.

---

## Step 1: Losses (Unacceptable Outcomes)

The following losses define the analysis boundary. All safety constraints below are derived from the requirement to prevent these outcomes.

| ID | Loss |
|----|------|
| L-1 | Vehicle crashes into the ground, structure, or person |
| L-2 | Vehicle enters uncontrolled flyaway (fails to return or land) |
| L-3 | Vehicle remains airborne with degraded control until power exhaustion |
| L-4 | Vehicle injures bystander during motor spin-up, arming, or unexpected flight |
| L-5 | Operator loses situational awareness and cannot intervene |
| L-6 | Vehicle causes property damage through unexpected motion |

---

## Step 2: Unsafe Control Actions

STPA identifies four categories of unsafe control action (UCA). I examine the major control authorities in Meridian: the arming gate, the mode transition controller, and the failsafe arbiter.

### 2.1 Arming Gate

| UCA | Type | Leads To |
|-----|------|----------|
| **UCA-1**: Arm command is accepted in Acro mode | Provided in wrong context | L-4. No altitude hold, no self-leveling on arm. Immediate crash if throttle is not at minimum. |
| **UCA-2**: Arm command accepted with GPS check disabled (`disabled_mask`) and GPS-dependent mode active | Provided in wrong context | L-1, L-2. GPS-requiring modes (Loiter, RTL, Auto, PosHold) execute with zero position reference. |
| **UCA-3**: EKF health gate bypassed during arming (PANEL_WAVE1 Hard Block #2) | Not provided | L-1. `gyro_corrected()` returns bias (Hard Block #1). EKF built on corrupted gyro input. No gate prevents arming into this state. |
| **UCA-4**: Arming permitted in Auto mode with no mission loaded | Provided in wrong context | L-1, L-2. `update_auto()` calls `wp_nav.update()` immediately; `WaypointNav` with zero waypoints behavior is undefined in this codebase. |

**Finding on UCA-1**: The arming subsystem (`check_prearm`) contains no check for the *current flight mode*. It validates sensors, EKF, GPS, battery, RC, and safety switch. It does not validate whether the selected mode is safe to arm into. Arming in Acro or Flip is possible and will pass all checks provided sensors are healthy. This is a missing safety constraint.

**Finding on UCA-4**: `set_mode(FlightModeId::Auto, ...)` in `multirotor.rs` calls `enter()`, which does nothing for Auto — no check that `wp_nav` has waypoints loaded. `load_mission()` is a separate call. The arming check enum includes `ArmingCheck::Mission = 13`, but this check is *not implemented* in `check_prearm()`. The bit exists in the bitmask definition; the enforcement code is absent.

### 2.2 Mode Transition Controller

| UCA | Type | Leads To |
|-----|------|----------|
| **UCA-5**: Transition to RTL when GPS is unavailable (EKF in dead-reckoning or GNSS lost) | Provided in wrong context | L-1, L-2. RTL climbs to 15m using position controller; with no valid GPS position, the climb target is `Vec3::zero()` (home initialization default). Vehicle climbs to altitude and holds at incorrect home position. |
| **UCA-6**: Failsafe triggers RTL while GPS is simultaneously lost | Provided at wrong time | L-2. RC loss fires `ReturnToLaunch`. GPS loss fires `Land`. If both fire concurrently, priority arbiter should apply; see UCA-10. |
| **UCA-7**: Transition to Land mode triggers motor shutdown at 0.3m barometric altitude without land-detector confirmation | Provided too early | L-1. Hard Block #10 from Mackay panel. Baro reads 0.3m during a hard flare or gusty descent. Motors cut. Vehicle falls. |
| **UCA-8**: RTL FinalDescent transitions to Land state based purely on barometric altitude condition (`altitude <= final_alt + 0.5`) | Provided too early | L-1. Same as UCA-7 but in RTL Land phase. The 0.3m check in `update_rtl` Land state (`input.altitude < 0.3`) has no land-detector gate. The full LandDetector in `meridian-failsafe` is not wired into `update_rtl`. |
| **UCA-9**: Loiter entered as default/fallback when unknown `FlightModeId` is received | Provided in wrong context | L-3. The `_ => self.update_loiter(input)` catch-all silently executes Loiter for `SystemId`, `FlowHold`, `AvoidAdsb`, and any future unrecognized mode. Operator sees active flight, not a fault. |
| **UCA-10**: Mode transition from Land to Loiter during active landing (via FS_OPTIONS `continue_if_landing`) | Not provided (missing constraint) | L-2. `FailsafeOptions::should_suppress_ext` suppresses failsafe action when `in_landing == true`. But the implementation of `check_all_full` does not track a Landing state variable — `in_landing` is a caller-supplied boolean with no enforcement mechanism. If the caller incorrectly passes `in_landing = true` during a non-landing descent, failsafe is silently suppressed. |

### 2.3 Failsafe Arbiter

| UCA | Type | Leads To |
|-----|------|----------|
| **UCA-11**: High-priority failsafe can be downgraded by a lower-priority concurrent failsafe | Provided (wrong value) | L-2. Priority arbiter prevents *escalation* of a resolved failsafe but the AUDIT_GAPS.md entry confirms "lower ones can't downgrade" was originally missing. The `FailsafeManager` implementation in lib.rs does implement priority using `current_priority` and `reason_priority()`. However, the `check_all_full` function does not check `current_priority` against *newly arriving* events — it only gates incoming events at the candidate collection phase. If a Terminate-class action is active and GPS loss (Land) arrives, the arbiter admits Land because `reason_priority(GnssLoss) = 0` which does not exceed `current_priority`. This appears correct. **Residual risk**: the priority table assigns `GnssLoss` priority 0 (baseline) while `RcLoss` is 5. During concurrent RC+GPS loss, RTL fires first (RC loss, priority 5) and GPS loss is suppressed. RTL then executes without GPS. This is not a bug in the arbiter; it is a hazardous interaction between correct components. |
| **UCA-12**: Dead man's switch absent — no timer-based automatic disarm for sustained hover without pilot input | Not provided | L-3. No idle-disarm timer detected in any reviewed file. ArduPilot disarms on the ground after configurable idle time (DISARM_DELAY). Meridian's WatchdogMonitor handles main loop stalls (2s MotorMin, 3s Disarm) but this is a crash-recovery mechanism, not a ground-idle safety disarm. |
| **UCA-13**: EKF health gate (`EkfHealthMonitor`) not wired to arming path | Not provided | L-1. Confirmed by Tridgell (Hard Block #2). `check_prearm` has `ArmingCheck::EkfHealth` and correctly reads `state.ekf_healthy`. But `ekf_healthy` is a boolean supplied by the caller — it originates from EKF state that has Hard Block #1 (corrupted gyro input) and Hard Block #8 (AidingMode::Relative never activated). The signal exists; the signal's source is broken. |

---

## Step 3: Causal Scenarios

STPA's third step traces each UCA back to its root causes. I identify four primary causal pathways.

### Causal Pathway A: Architectural Disconnection

The codebase has correct individual components that are not connected into a coherent safety control loop. This produces UCAs 1, 4, 7, 8, 13.

**Mechanism**: Each crate (`meridian-arming`, `meridian-modes`, `meridian-failsafe`) is internally consistent. The integration layer — which should enforce invariants *across* crates — does not exist. There is no top-level safety monitor that combines "current mode + current EKF state + failsafe status" before accepting control transitions.

**Example**: The ArmingCheck::Mission enum variant exists (line 39, `meridian-arming/src/lib.rs`) but has zero enforcement code. The check was designed, enumerated, and silently omitted.

### Causal Pathway B: Mode Controller Without State Precondition Enforcement

`set_mode()` in `multirotor.rs` accepts any `FlightModeId` and calls `enter()`. There is no precondition table. RTL does not verify GPS validity. Auto does not verify mission length. Land does not capture land-detector reference altitude. Each of these creates a window where the mode begins executing in an unsafe initial state.

**Mechanism**: In ArduPilot, mode entry functions (`init()`) return a boolean — entry is *rejected* if preconditions fail. `set_mode()` in Meridian is `fn set_mode(&mut self, mode: FlightModeId, input: &ModeInput)` — void return, unconditional execution. The safe-entry gate has no representation in the type system.

### Causal Pathway C: Land Detector Wiring Gap

The full 8-condition LandDetector in `meridian-failsafe` (identified in PANEL_WAVE1 as Hard Block #9) is not wired into the Land mode state machine (`update_land`) or the RTL Land state (`update_rtl`). The in-mode check uses only 3 conditions: altitude < 0.5m, |climb_rate| < 0.5 m/s, not ascending. This check is insufficient to distinguish:

- Vehicle on the ground in light wind (correct: shut down)
- Vehicle at 0.3m during a fast auto-descent in a gust (incorrect: shut down)
- Vehicle bounced to 0.3m after hard landing (ambiguous)

Motor shutdown at `land_complete_timer > 1.0` seconds uses the 3-condition check. The 1-second timer provides minimal margin against the 0.3m altitude condition being transiently met during descent.

### Causal Pathway D: Gyro Bug Propagating Through All Attitude-Dependent Safety Checks

Hard Block #1 (`gyro_corrected()` returns bias, not corrected rate) is not isolated to the gyro driver. The corrupted rate propagates to:
- The attitude controller (wrong attitude estimate)
- The EKF state vector (wrong bias correction)
- The crash detector (wrong roll/pitch to assess inversion — `CrashDetector::check()` uses `roll` and `pitch` from AHRS output)
- The land detector (wrong vertical velocity estimate)
- The EKF health check used in arming

Until Hard Block #1 is resolved, every safety check that depends on vehicle state is operating on systematically corrupted input. This is not a software architecture problem; it is a prerequisite failure that makes all downstream safety analysis conditional.

---

## Step 4: Missing Safety Constraints

The following constraints are **required** by STPA but **absent** from the current implementation.

### SC-1 (CRITICAL): Mode entry must verify preconditions, not merely execute

**Constraint**: `set_mode()` must be a checked transition. Auto requires `wp_nav.waypoint_count() > 0`. RTL requires GPS valid or a GPS-denied fallback mode explicitly selected. Land requires altitude reference capture and land-detector initialization. Mode entry failure must result in a fallback to a safe mode (Loiter or Stabilize), not silent execution.

**Current state**: No preconditions. Unconditional mode entry. Missing from both `set_mode()` and arming checks.

### SC-2 (CRITICAL): Arming in high-risk modes must be explicitly restricted

**Constraint**: Arming in Acro, Flip, Turtle, or any mode without self-leveling capability must be blocked unless a specific override parameter is set by an expert operator. At minimum, arming in Acro must require throttle at minimum AND a mode-specific override bit.

**Current state**: `check_prearm()` does not inspect current mode.

### SC-3 (CRITICAL): ArmingCheck::Mission must be enforced

**Constraint**: When `ArmingCheck::Mission` is enabled and vehicle mode is Auto, mission must contain at least one valid NAV waypoint. This check must execute during `check_prearm()`, not be silently bypassed.

**Current state**: Enum variant declared, no enforcement code.

### SC-4 (HIGH): Motor shutdown during landing must require land-detector confirmation

**Constraint**: `ModeOutput::Idle` (motor shutdown) during Land or RTL Land phase must require the full 8-condition land detector to report landed for a sustained period (minimum 2 seconds), not a 3-condition proxy at 0.3m altitude.

**Current state**: 3-condition check, 1-second timer, 0.3m threshold. Confirmed dangerous by Mackay panel.

### SC-5 (HIGH): RTL must not be the failsafe action when GPS is unavailable

**Constraint**: If `GnssLoss` or `DeadReckoning` failsafe is active, `ReturnToLaunch` must be rejected as a failsafe action. The fallback must be `Land` or `Stabilize` depending on altitude. The failsafe arbiter must cross-reference GPS availability before issuing RTL.

**Current state**: The `FailsafeManager` does not cross-check GPS state against the RTL action. The priority system allows RcLoss (priority 5) to fire RTL while GnssLoss (priority 0, baseline) is simultaneously active but not yet triggered.

### SC-6 (HIGH): Failsafe suppression via FS_OPTIONS requires explicit state verification

**Constraint**: `continue_if_landing = true` must only suppress failsafe when the vehicle is demonstrably in a landing sequence, verified by the failsafe manager's own state, not by a boolean passed from the caller. The FailsafeManager must own the "currently landing" determination.

**Current state**: `in_landing` is a caller-supplied boolean to `should_suppress_ext()` with no internal verification.

### SC-7 (MEDIUM): Ground idle disarm timer required

**Constraint**: A vehicle that remains armed on the ground with throttle at minimum for longer than a configurable period (e.g., 15 seconds) must automatically disarm. This prevents the scenario where a pilot walks away from an armed vehicle on a flat surface.

**Current state**: WatchdogMonitor addresses loop stalls, not ground idle. No idle-disarm mechanism found.

### SC-8 (MEDIUM): Mode fallback for unknown FlightModeId must be explicit, not silent

**Constraint**: The `_ => self.update_loiter(input)` catch-all must be replaced with an explicit fault: unknown modes must trigger a notification and transition to a well-defined safe mode (Stabilize). Silent loiter masks configuration errors and unimplemented mode stubs that should never be commanded in flight.

**Current state**: `FlightModeId::SystemId | FlightModeId::FlowHold | FlightModeId::AvoidAdsb` and any unmatched variant silently execute Loiter.

---

## Specific Questions Addressed

**Q: Can the vehicle enter Auto mode without a valid mission?**

Yes. `set_mode(FlightModeId::Auto, ...)` executes unconditionally. `enter()` for Auto is the `_ => {}` catch-all — no initialization, no precondition check. `update_auto()` immediately calls `wp_nav.update(&input.position, ...)`. `ArmingCheck::Mission` exists but is not enforced in `check_prearm()`. This is SC-3 above. The vehicle can arm in Auto with zero waypoints.

**Q: Can failsafe downgrade from Land to Loiter?**

The priority arbiter in `FailsafeManager` assigns `Land` as the action for `GnssLoss` (GPS loss monitor). `Loiter` is not a `FailsafeAction` variant — it does not appear in the action enum. So a failsafe cannot command Loiter. However, FS_OPTIONS `continue_if_landing` *suppresses* a failsafe action, which has the practical effect of allowing the current mode (which could be Loiter) to continue. If the vehicle is in Loiter descending slowly and `continue_if_landing` is set, an RC loss will be suppressed. The vehicle continues in Loiter, not Land. This is not a downgrade of the action but of the *protection level*. SC-6 above addresses this.

**Q: Can the pilot accidentally arm in a dangerous mode?**

Yes. `check_prearm()` does not inspect current `FlightModeId`. Arming in Acro is possible and no warning is generated. Arming in Flip is possible. Arming in Throw is possible — Throw idles motors until throw detection, so this is relatively safe, but arming in Acro with any throttle above minimum creates immediate uncontrolled full-rate flight.

The PANEL_WAVE1 Hard Block #12 (GPS arm check blocks non-GPS modes) is the *opposite* problem — it currently blocks arming in Stabilize without GPS, which is overly restrictive. Both the missing Acro restriction and the false GPS restriction in Stabilize represent mode-awareness failures in the same arming gate.

**Q: Is there dead man's switch protection?**

No. The `WatchdogMonitor` detects main loop stalls (2s MotorMin, 3s Disarm) — this is a CPU liveness monitor, not a pilot engagement monitor. There is no RC inactivity timer, no ground-idle disarm, and no "arm-and-walk-away" detection. ArduPilot's `DISARM_DELAY` parameter equivalent is absent. SC-7 above addresses this.

---

## Overall Assessment

### What is working

The failsafe monitor library contains correct individual monitors: the battery dual-threshold system, the EKF innovation gate (5-second sustain), the dead reckoning monitor, the GPS glitch detector, and the compass consistency checker are all logically sound. The priority arbiter correctly prevents downgrade from high-priority failsafes. The FS_OPTIONS system is complete and correctly structured.

The arming check framework is comprehensive in its enumeration. The 17 checks cover the right failure modes. The IMU consistency internal computation (Gap 44) is correctly implemented with ArduPilot's exact thresholds.

### What is broken at the system level

Meridian has a fundamental architectural gap: the *control structure* that connects sensors → state estimates → safety decisions → mode transitions does not enforce safety constraints at the boundaries between these layers. Each layer is internally correct; the inter-layer enforcement is absent.

In STAMP terms: the system has correct *process model* components (sensors, EKF, monitors) but the *control actions* (arm, change mode, apply failsafe) do not enforce the *safety constraints* that should gate them. A safety controller that does not enforce its own constraints is not a safety controller.

This is compounded by Hard Block #1 (gyro returning bias, not corrected rate), which corrupts the process model entirely. All safety analysis of the flight software is conditional on this bug being fixed. A correct safety architecture built on wrong sensor data is not safe.

---

## Verdict

**UNACCEPTABLE for any powered outdoor flight.**

The system does not meet the minimum safety constraint requirements for autonomous operation. The specific blocking issues are:

1. Motor shutdown during landing is not land-detector-gated (SC-4 / Hard Block #10)
2. Auto mode can be entered and armed without a mission (SC-3)
3. Arming in Acro or other high-risk modes is not blocked (SC-2)
4. RTL can execute without GPS validity cross-check (SC-5)
5. Hard Block #1 (gyro bug) corrupts all downstream safety functions

**CONDITIONAL** for bench motor spin testing with props removed, provided Hard Block #1 is resolved first and the safety switch is functional.

The architecture is recoverable. The missing constraints (SC-1 through SC-8) are implementable in days, not weeks. The underlying algorithmic work is sound. Fix the control structure, not the algorithms.

---

*Nancy Leveson*  
*MIT Engineering Systems Lab*  
*STPA review conducted per STAMP methodology, "Engineering a Safer World" (2011), Chapter 9 methodology*
