# Fixed-Wing / VTOL / Navigation Review — Henry Wurzburg
## Meridian Autopilot: TECS, L1, Fixed-Wing Modes, QuadPlane, Landing, Tailsitter

**Reviewer**: Henry Wurzburg — ArduPlane specialist, TECS / L1 navigation, fixed-wing modes, QuadPlane transitions, landing sequences  
**Date**: 2026-04-02  
**Files Reviewed**:  
- `crates/meridian-modes/src/fixed_wing.rs`  
- `crates/meridian-control/src/tecs.rs`  
- `crates/meridian-nav/src/l1.rs`  
- `crates/meridian-nav/src/scurve.rs`  
- `docs/identification_vehicles_modes.md`  
- `docs/identification_core_flight.md`

---

## Executive Summary

Meridian's fixed-wing stack divides into a well-built navigation layer and a critically incomplete mode layer. The standalone TECS in `meridian-control/src/tecs.rs` is the best piece of work in this entire domain — the energy balance is correct, the underspeed protection is sound, and the flare mode is structured properly. The L1 controller is also solidly implemented with the right K_L1 coefficient, proper crosstrack integral, and prevent-indecision logic.

The flight mode implementations in `fixed_wing.rs` are a different story. They use a *second, private* TECS implementation — a stripped-down shadow struct — rather than calling the full-featured `meridian-control/src/tecs.rs`. This shadow TECS has no underspeed protection, no proper energy balance, and no flare logic. Every automated mode that runs through `fixed_wing.rs` — Auto, RTL, Loiter, Guided, Land — is running on the wrong controller.

The landing sequence has a specific and dangerous flaw. The QuadPlane and tailsitter subsystems are missing entirely. WeatherVane is absent. Training and Thermal are stubs that silently run other modes. The combination of these issues means Meridian can fly a straight-and-level fixed-wing circuit in FBWA or Manual, but any autonomous mission, any landing, any VTOL operation, and any underspeed recovery is either broken, dangerous, or absent.

---

## Section 1: TECS Energy Balance

**Rating: SPLIT — `meridian-control/src/tecs.rs` is FLY_IT for the algorithm; `fixed_wing.rs` inline TECS is DANGEROUS**

### The good: `meridian-control/src/tecs.rs`

The real TECS is a full implementation of the Riseborough energy control system.

- **Total energy error** (`ste_error`) and **energy balance error** (`seb_error`) are computed correctly from `g*h + 0.5*v^2` and `g*h - 0.5*v^2` respectively. The signs are right.
- **Rate errors** (`ste_rate_error`, `seb_rate_error`) correctly multiply by the respective velocity terms (`GRAVITY * height_rate + speed * speed_rate`). This is the subtle part most people get wrong.
- **Speed/height weighting** (`spdweight`): the `seb_w` / `ste_w` split is implemented correctly. When `spdweight > 1.0` (speed priority), `seb_w` decreases while `ste_w` stays at 1.0. When `spdweight < 1.0` (altitude priority), `ste_w` decreases. This matches ArduPilot's AP_TECS behavior exactly.
- **Throttle P/D/I and pitch P/D/I** are normalized against `GRAVITY * max_climb` which keeps gain scaling sensible across different aircraft sizes.
- **Underspeed protection**: max throttle forced, pitch clamped to `<= 0`. Underspeed detection at 90% of airspeed_min (`speed_estimate < airspeed_min * 0.9`) matches ArduPilot's hysteresis band.
- **Bad descent detection**: `height_rate < -max_sink_rate && height_estimate < height_demand`. Correct.
- **3rd-order complementary height filter** and **2nd-order speed filter**: reasonable omega values (3.0 and 2.0 rad/s), matching ArduPilot defaults.
- **Flare mode** (`update_flare()`): separate entry point, sink rate ramp based on distance past land waypoint, throttle cut (or maintained for approach speed). The structure is correct.

**Notable gaps in the full TECS** (not dangerous for current scope, but worth filing):
- No `TECS_VERT_ACC` vertical acceleration limiter on climb/sink demands. ArduPilot uses `vert_accel_limit` to rate-limit the height demand ramp in `_update_height_demand()`. Without this, a large altitude step creates an instantaneous energy demand spike and the throttle integrator winds up during the first few seconds of a climb command.
- Height demand rate-limiting is done via a simple `clamp(-max_sink_rate, max_climb_rate)` on the error term rather than a proper demand integrator. This means demand changes are still instantaneous — just bounded. ArduPilot's `_hgt_dem_adj` is a genuine first-order integrator on the demand.
- No `TECS_LAND_TCONST` adjustment to the height filter during approach — ArduPilot switches to a tighter time constant (`land_time_const`) in `_update_height_estimate()` before flare entry, not just in `update_flare()`.
- `throttle_cruise` is a fixed parameter. ArduPilot computes a real-time cruise throttle estimate from achieved energy rate.

These are NEEDS_TUNING items in the full TECS, not dangerous flaws.

### The critical problem: the shadow TECS in `fixed_wing.rs`

`FixedWingModes` contains its own private `Tecs` struct (lines 85–173 in `fixed_wing.rs`). This is not `meridian_control::tecs::Tecs`. It is a different, simpler implementation that was probably written as a placeholder and never replaced. It is what actually runs in every automated mode.

**This shadow TECS has no:**
- Energy balance error (`seb_error`)
- Rate error terms
- Underspeed protection of any kind
- Bad descent detection
- Proper energy-based throttle calculation

The shadow TECS throttle computation (line ~155) is:
```
throttle = 0.25 + speed_error * 0.05 + energy_error_integral * 0.01
```

This is a speed-only P+I controller with a 0.25 bias. The throttle has no knowledge of altitude. In a climb, the aircraft will pitch up (correct), but the throttle does not increase to compensate for the energy being traded to potential energy — it only responds to the resulting airspeed drop. This delay causes speed oscillation and will cause underspeed in any sustained climb.

The pitch computation (line ~163) is:
```
pitch = atan2(climb_rate_target, current_speed)
```

where `climb_rate_target = (target_alt - current_alt) * 0.3`. This is purely geometric — climb angle from altitude error. There is no feed-forward from speed state, no damping term, no energy balance. In a speed-up command with constant altitude, pitch stays near zero. In a headwind with altitude hold, pitch and throttle are independent — no coordination. The aircraft will hunt.

**Verdict**: Every automated mode in `fixed_wing.rs` — Auto, RTL, Loiter, Circle, Guided, Land approach — uses a broken energy controller. On a real aircraft with a high wing-loading, this will cause airspeed oscillation in cruise and stall in a sustained climb.

---

## Section 2: L1 Navigation — K_L1 Coefficient

**Rating: FLY_IT**

The L1 implementation in `meridian-nav/src/l1.rs` is correct and complete for the ArduPilot-equivalent algorithm.

**K_L1 verification**: Line 165:
```rust
let k_l1 = 4.0 * self.params.damping * self.params.damping;
```

With default `damping = 0.75`: K_L1 = 4.0 × 0.5625 = **2.25**. This matches the recent ArduPilot fix (previously erroneously computed as 2*d^2 in some implementations; the correct formula is 4*d^2 from the Park/Deyst/How derivation). The test at line 264 explicitly verifies this value. Confirmed correct.

**L1 distance formula** (line 107):
```rust
let l1_dist_raw = (1.0 / PI) * damping * period * groundspeed;
```
Correct. With default period=17.0, damping=0.75, speed=20 m/s: L1 = (1/π) × 0.75 × 17.0 × 20 = **81.2 m**. Reasonable for a 17-second period.

**Speed scaling behavior**: L1 distance scales with groundspeed — slower flight = tighter turns, faster flight = gentler turns. This is the correct behavior for maintaining consistent path-following geometry across the speed envelope.

**Crosstrack integral** (lines 175–185): Integrates `nu1 = atan2(cross, l1_dist)` only when `|nu1| < 5 degrees`, capped at ±0.1 rad, with gain XTRACK_I = 0.02. This is ArduPilot-correct — the conditional integration prevents windup during large crosstrack deviations or turns.

**Prevent-indecision latch** (lines 154–161): Correct implementation. Latches the last `nu` when `|nu| > 0.9π` and the sign would flip. This prevents the 180-degree oscillation that occurs when the vehicle overshoots a waypoint.

**Loiter mode** (lines 203–230): Simple centripetal + proportional radial error. This is a simplified model compared to ArduPilot's full loiter geometry (which uses a proper tangent-entry approach), but it is stable and flyable. The radial error gain of 0.5 is modest. With default loiter radius of 60 m and a 20 m/s aircraft, centripetal acceleration is 20²/60 = 6.67 m/s² — well within a typical bank angle budget. No instability risk here.

**Gaps (not dangerous):**
- No loiter entry geometry (ArduPilot computes a tangent approach path to enter the circle tangentially rather than cutting across). Meridian will exhibit a brief S-turn on loiter entry.
- No altitude-dependent L1 distance minimum (ArduPilot uses a terrain-clearance-aware floor). Safe to ignore for now.
- `update_waypoint()` always uses the vehicle's current position as `prev_wp` (line 504–507 in fixed_wing.rs). This means L1 is tracking from the current position rather than the true leg start. The path-following geometry degrades for waypoint sequences — the controller always references "where I am now" rather than "where this leg began." It works but produces lazy turns at waypoints.

---

## Section 3: Speed Scaling in FBWA

**Rating: NEEDS_TUNING**

`speed_scaling()` (lines 298–303 in `fixed_wing.rs`):
```rust
fn speed_scaling(&self, airspeed: f32) -> f32 {
    if airspeed < 1.0 { return 2.0; }
    (self.params.scaling_speed / airspeed).clamp(0.5, 2.0)
}
```

The scaling is then applied as a **multiplier on the stick-to-command gain** in FBWA:
```rust
let roll = input.rc_roll * self.params.roll_limit * scaling;
```

**Problem**: ArduPilot applies speed scaling as a **divisor on the output** to control surface deflection — at low speed, more deflection is needed for the same attitude change; at high speed, less. The current implementation **increases the bank angle command** at low speed and **decreases it** at high speed, which is the opposite of what speed scaling is meant to correct.

At low airspeed (say 8 m/s with scaling_speed=15): scaling = 15/8 = 1.875, clamped to 2.0. The code commands `roll_limit * 2.0 = 90 degrees` from a full stick input. That will either saturate the roll limit clamp (saving it) or, if roll_limit is large, produce an overbanked turn that increases induced drag and accelerates the underspeed. This is a NEEDS_TUNING issue because the roll_limit clamp (`clamp_roll()`) prevents the worst outcome, but the underlying intent of speed scaling is inverted.

In FBWB and Cruise modes, `speed_scaling()` is applied to roll only (the pitch goes to TECS), so the impact is bounded. In stabilized modes with airspeed sensor failures (airspeed = 1 m/s default fallback), the scaling is hardcoded to 2.0 — double the bank authority. Survivable with a 45-degree roll limit.

**The scaling_speed default is 15.0 m/s.** With `airspeed_min = 12.0 m/s`, the aircraft is at the stall margin before scaling exceeds 1.25×. The clamp floor of 0.5 kicks in at 30 m/s. This is a reasonable operating range. The inversion is a bug but not an immediate crash risk if roll_limit is sanely configured.

---

## Section 4: QuadPlane Transition Stub Completeness

**Rating: DANGEROUS (for any VTOL operation)**

`update_vtol_transition()` (lines 759–763 in `fixed_wing.rs`):
```rust
fn update_vtol_transition(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
    self.update_fbwa(input, dt)
}
```

This runs `FlightModeId::VtolTransitionToFw` and `FlightModeId::VtolTransitionToHover` as pure FBWA. There is no:
- Airspeed monitoring during transition
- VTOL motor spin-up/spin-down
- Tilt-rotor angle progression
- Control surface authority blending between multirotor and fixed-wing controllers
- Q_ASSIST activation on underspeed
- Transition timeout / abort logic

The PARITY gap audit confirms: the entire QuadPlane subsystem (`quadplane.cpp`, 4,976 lines in ArduPilot) has no Meridian equivalent. The SLT_Transition state machine, `VTOL_update()`, `motors_output()`, throttle suppression, XY position hold during VTOL — none of it exists.

**What happens if a QuadPlane operator commands transition**: The aircraft enters FBWA with VTOL motors at whatever state they are in. If transitioning from hover to forward flight, VTOL motors are spinning and the fixed-wing surfaces are generating drag but no lift (below stall speed). The FBWA controller will command roll and pitch based on stick input with no awareness that the aircraft is below fixed-wing flying speed. If the VTOL motors are throttled down during "transition" and the aircraft is not yet at flying speed, it falls.

This is not a latent bug. It is an active hazard for any platform wiring `VtolTransitionToFw` into hardware control loops.

**Q_ASSIST is absent at the architecture level**: ArduPilot's Q_ASSIST monitors airspeed continuously in all fixed-wing modes and activates VTOL motors when airspeed drops below `Q_ASSIST_SPEED`. In Meridian, fixed-wing modes have no awareness of VTOL motor authority. An aircraft that stalls in Auto mode gets TECS underspeed protection (pitch down, max throttle from the main engine) — but no multirotor lift assistance. For a tiltrotor or tailsitter in mountainous terrain, that is a crash.

---

## Section 5: Landing Flare Sequence

**Rating: DANGEROUS**

The landing sequence in `update_land()` (lines 683–733) has a specific flaw that will cause a hard landing or crash on real hardware.

### Approach phase

The approach logic targets:
```rust
let target_alt = self.params.land_flare_height + 2.0; // aim slightly above flare
```

With default `land_flare_height = 5.0 m`, this targets 7.0 m AGL as the approach altitude. The aircraft descends toward 7 m and transitions to flare at `input.altitude <= land_flare_height` (5.0 m). **The approach uses the shadow TECS, not the real one.** As established in Section 1, the shadow TECS has no energy balance. Approach speed is commanded but not reliably maintained.

The L1 guidance uses `input.position` as `prev_wp` (both legs of the L1 call use current position). This means the L1 is not tracking a final approach course — it is always homing to the land target as if it were a standalone waypoint. For a straight-in approach this mostly works, but the aircraft will not maintain runway alignment on a crosswind. More critically, when transitioning from the final approach to the flare, there is no `isHeadingLinedUp()` gate (identified as missing in the gap audit for `mode_loiter.cpp`). ArduPilot requires heading alignment before descending through the threshold. Meridian will attempt to flare regardless of runway alignment.

### Flare phase

```rust
LandState::Flare => {
    if input.altitude <= 0.5 {
        self.land_state = LandState::Ground;
    }
    ModeOutput::FixedWingTarget {
        roll: 0.0,
        pitch: self.params.land_flare_pitch,  // 8 degrees
        throttle: 0.1,
        yaw_rate: 0.0,
    }
}
```

**Critical issue 1 — fixed pitch in flare**: The flare commands a constant `land_flare_pitch` of 8 degrees from the moment the aircraft hits 5 m AGL until it reaches 0.5 m AGL. There is no height-referenced pitch progression, no sink rate control, no time-constant-based flare. ArduPilot's TECS flare (`update_flare()`) ramps the pitch target based on height, sink rate, and `land_time_const`. A constant 8-degree nose-up attitude at 5 m AGL with 14 m/s approach speed will cause the aircraft to float — possibly hundreds of meters past the touchdown zone before the altitude drops to 0.5 m.

**Critical issue 2 — flare trigger is barometric altitude, not AGL**: `input.altitude` is the barometric altitude above sea level (or above home). If the landing zone is at elevation or if there is any baro drift, the flare will trigger at the wrong height. A 5 m baro error at flare trigger means the aircraft noses up 8 degrees at 10 m AGL (balloon) or at 0 m AGL (impacts runway at approach speed before flare).

ArduPilot's flare trigger uses terrain-adjusted altitude or rangefinder when available (`RNGFND_LANDING = 1`). Meridian has no rangefinder integration in the land mode. At minimum, the flare height should be computed against home altitude, not raw baro.

**Critical issue 3 — no throttle cut on ground**: The Ground state correctly sets throttle to 0.0. But the transition from Flare to Ground requires `input.altitude <= 0.5` — again barometric. If baro reads 0.5 m when wheels touch at 0 m, throttle stays at 0.1 (idle-ish) through the bounce. ArduPilot detects landing via accelerometer-based touchdown detection and cuts throttle on contact. Meridian has no touchdown detection.

**Critical issue 4 — the full TECS `update_flare()` is never called**: `meridian-control/src/tecs.rs` has a correct `update_flare()` function that handles the progressive sink rate ramp, reduced time constant, and throttle management. It is never called from `fixed_wing.rs`. The landing mode instead runs the fixed pitch stub above. The real flare logic exists in the codebase and simply isn't wired in.

**Summary**: The landing sequence will float on a calm day, drift badly in crosswind, and produce a hard landing or runway excursion in any operational condition. This is the highest-priority safety issue in the fixed-wing stack.

---

## Section 6: Tailsitter Stub

**Rating: DANGEROUS (non-existent)**

There is no tailsitter implementation anywhere in Meridian. The gap audit (`identification_vehicles_modes.md`, lines 871–886) confirms:

> `tailsitter.cpp` (1,060 lines) — MERIDIAN EQUIVALENT: None — PARITY STATUS: MISSING

And from `identification_core_flight.md`:
> `AC_AttitudeControl_TS.cpp` (136 lines) — MERIDIAN EQUIVALENT: NONE — PARITY STATUS: MISSING

`FlightModeId::VtolTransitionToFw` and `FlightModeId::VtolTransitionToHover` both fall through to `update_vtol_transition()` which runs FBWA. For a tailsitter, FBWA applied at the wrong phase of transition means the control surfaces are commanding attitude in the wrong body axis. A tailsitter in hover has its thrust axis horizontal relative to a forward-flight reference frame. Applying roll/pitch commands from FBWA directly will command the wrong physical surfaces.

No tailsitter velocity remapping, no transition angle threshold, no `set_VTOL_roll_pitch_limit()`, no `allow_weathervane()` integration, no `is_control_surface_tailsitter()` check. This subsystem is not a stub — it does not exist.

**Recommended action**: Gate `VtolTransitionToFw` and `VtolTransitionToHover` with a compile-time feature flag and return an error if a tailsitter vehicle type is detected. Running FBWA on a tailsitter in mid-transition is worse than not running any fixed-wing mode.

---

## Section 7: WeatherVane Absence

**Rating: NEEDS_TUNING (for VTOL efficiency) / DANGEROUS (for QuadPlane hover stability)**

`AC_WeatherVane.cpp` (241 lines in ArduPilot) — MERIDIAN EQUIVALENT: NONE.

WeatherVane provides yaw alignment with the wind during VTOL hover. For a conventional QuadPlane hovering in wind, not weather-vaning means the fuselage presents broadside drag to the wind. Depending on wing loading, this can generate yaw moments that exceed the yaw authority of the VTOL motors. ArduPilot addresses this with `weathervane_yaw_rate()` which adds a corrective yaw rate proportional to lateral drag.

For pure-hovering operations (QHover, QLoiter modes — which are also absent), this is academic. For the transition stub (`update_vtol_transition()` = FBWA), the combination of no VTOL motors, no weathervane, and a real aircraft with a 20-knot crosswind is a survivability-limiting scenario.

The absence of WeatherVane is a second-order issue relative to the shadow TECS and the landing flare, but should be noted as a required implementation for any VTOL variant.

---

## Section 8: S-Curve Trajectory Planner

**Rating: FLY_IT (algorithm correct, scope limited)**

`meridian-nav/src/scurve.rs` is a well-implemented 7-phase jerk-limited trajectory planner. The phase progression is correct: Jerk+, ConstAccel, Jerk-, Cruise, Jerk-, ConstDecel, Jerk+. The `compute_phase_durations()` function correctly handles:
- Triangular accel profile when segment is too short to reach full accel (no constant accel phase)
- Binary search (`compute_reduced_profile()`) for too-short segments, 30-iteration convergence

The `evaluate_at()` function correctly maintains position/velocity continuity across phase boundaries by carrying forward the state at the end of each phase.

The Catmull-Rom `SplineSegment` provides C1-continuous corner transitions. This is the right data structure for smooth waypoint navigation.

**The scope limitation**: The S-curve planner is used by copter-mode waypoint navigation (via `WaypointNav`) but does not feed into the fixed-wing mode stack. Fixed-wing Auto mode calls `wp_nav.update()` which returns a target position, but there is no velocity or jerk feed-forward into the L1 controller. ArduPilot's fixed-wing Auto mode does not use S-curves (fixed-wing uses L1 directly for path following) so this is not a gap — it is appropriate architecture. The S-curve planner is fine for what it does.

---

## Section 9: Auto Mode — Prev-WP Tracking Gap

**Rating: NEEDS_TUNING**

In `update_auto()` (lines 499–528), the L1 call passes `input.position` as `prev_wp` for both waypoints:

```rust
let prev_wp = if self.wp_nav.current_index() > 0 {
    input.position  // BUG: should be the actual previous waypoint
} else {
    input.position
};
```

Both branches return `input.position`. The comment ("Use current position as approximation") understates the problem. The L1 controller uses `prev_wp` and `next_wp` to define the **path line** it is tracking. When `prev_wp == position`, the L1 is not tracking a leg — it is homing to the next waypoint as a point-navigation problem. There is no crosstrack error relative to the intended leg, so the integral correction is referencing the wrong line.

The practical effect: the aircraft will fly curved paths instead of straight legs when approaching waypoints from the side. For a mission with tightly spaced waypoints (< L1 distance = ~80 m at 20 m/s), the aircraft may never establish on the intended leg at all.

Fix: `WaypointNav` should expose the previous waypoint coordinate, and `update_auto()` should pass it to `update_waypoint_ext()`.

---

## Section 10: Takeoff Mode

**Rating: NEEDS_TUNING**

`update_takeoff()` (lines 655–679) commands fixed pitch (15 degrees) and full throttle until `input.altitude >= takeoff_altitude`. This is functional for a hand-launch or catapult launch scenario. Issues:

- No ground roll or rotation speed modeling. A runway takeoff will apply full pitch immediately, which for a tail-dragger causes a prop strike.
- No crosswind correction during climb. The wings-level command (`roll: 0.0`) ignores any drift. An aircraft climbing through a crosswind will drift laterally and may leave the runway corridor.
- The `TakeoffState::Complete` state outputs `throttle: 0.5` (hardcoded cruise), not TECS cruise. If the aircraft is at takeoff altitude in a headwind, 0.5 throttle may be above or below what is needed for cruise speed — the shadow TECS is not in the loop.
- No transition to Auto or FBWA after takeoff — the mode stays in `TakeoffState::Complete` with wings level and 0.5 throttle indefinitely.

These are NEEDS_TUNING for the hand-launch use case. For runway operations, the absence of rotation speed and ground roll is DANGEROUS.

---

## Rating Summary

| Component | Rating | Key Issue |
|---|---|---|
| `tecs.rs` — full TECS energy balance | FLY_IT | Correct energy balance, underspeed protection, flare structure |
| `tecs.rs` — demand rate limiting | NEEDS_TUNING | No `vert_accel_limit` on height demand; instantaneous demand steps |
| `fixed_wing.rs` — inline shadow TECS | DANGEROUS | No energy balance; no underspeed protection; wrong controller in every auto mode |
| `l1.rs` — K_L1 coefficient (4d²) | FLY_IT | Correct derivation, verified by test |
| `l1.rs` — crosstrack integral | FLY_IT | Correct conditional integration, correct cap |
| `l1.rs` — prev_wp usage in Auto | NEEDS_TUNING | Auto mode always passes current position as prev_wp |
| `fixed_wing.rs` — FBWA speed scaling | NEEDS_TUNING | Scaling direction inverted vs ArduPilot intent; clamped roll limit prevents worst outcome |
| `fixed_wing.rs` — Land / flare | DANGEROUS | Fixed-pitch flare floats; baro-only trigger; real `update_flare()` never called |
| `fixed_wing.rs` — QuadPlane transition | DANGEROUS | Zero VTOL logic; FBWA at sub-stall speed = freefall for VTOL platforms |
| Tailsitter | DANGEROUS | Entirely absent; FBWA applied in wrong axis |
| WeatherVane | NEEDS_TUNING | Absent; VTOL crosswind stability degraded |
| `scurve.rs` — S-curve planner | FLY_IT | Correct 7-phase algorithm; appropriate for copter nav |
| `fixed_wing.rs` — Takeoff | NEEDS_TUNING | Hand-launch functional; runway takeoff unsafe |
| `fixed_wing.rs` — Manual / Stabilize | FLY_IT | Direct passthrough; no algorithmic risk |
| `fixed_wing.rs` — FBWA | FLY_IT (with speed scaling caveat) | Core stabilization correct; speed scaling inverted but bounded |
| `fixed_wing.rs` — RTL | NEEDS_TUNING | No rally points; QRTL transition absent; shadow TECS in loop |
| `fixed_wing.rs` — Auto | NEEDS_TUNING | Shadow TECS; wrong prev_wp; functional for flat terrain surveys |

---

## Priority Defects for Immediate Action

**P0 — Crash-causing, must fix before any autonomous flight:**

1. **Wire the real TECS into `fixed_wing.rs`**. Remove the shadow `Tecs` struct from `fixed_wing.rs`. Import and call `meridian_control::tecs::Tecs` for all mode updates. This single change fixes FBWB, Cruise, Auto, RTL, Loiter, Circle, and Guided in one step.

2. **Wire `update_flare()` into the Land mode**. The flare logic exists in `tecs.rs` and is correct. `LandState::Flare` must call `tecs.update_flare(height_agl, approach_speed, distance_past_land, dt)` and use its output. Fix the altitude source to be AGL (terrain-relative or rangefinder), not raw baro.

3. **Gate QuadPlane and tailsitter modes with a feature flag or `unimplemented!()`**. Do not silently fall through to FBWA. An operator who activates a VTOL transition on a QuadPlane must get a compile error or a panic, not a false sense of safety.

**P1 — Hazardous in operational conditions:**

4. **Fix `prev_wp` in `update_auto()`** to pass the actual previous waypoint from `WaypointNav`. This makes leg-tracking work correctly for multi-waypoint missions.

5. **Add a flare height gate on landing approach**. Before entering `LandState::Flare`, require heading alignment within ±10 degrees of runway bearing (isHeadingLinedUp equivalent). Add a minimum approach speed check using the full TECS speed estimate.

6. **Correct speed scaling direction in FBWA** or remove it pending a proper control law audit. The current implementation doubles bank authority at low airspeed.

**P2 — Quality / completeness:**

7. Add `vert_accel_limit` demand rate-limiting to `tecs.rs` `update()`.
8. Expose previous waypoint from `WaypointNav` for proper L1 leg tracking.
9. Implement WeatherVane yaw rate when QuadPlane subsystem is built.
10. Add rotation speed / ground roll to Takeoff mode for runway operations.

---

*Review complete. The fixed-wing navigation math is sound. The flight mode implementation layer is not ready for autonomous operation. The landing sequence is unsafe as written. Do not fly autonomous missions or attempt any VTOL operation until P0 items are resolved.*
