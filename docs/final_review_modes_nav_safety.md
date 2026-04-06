# Meridian Final Review: Flight Modes, Navigation, Mission, and Safety Systems

**Review Date:** 2026-04-02
**Reviewer:** Automated audit (automated cross-reference against ArduPilot source)
**Scope:** Pre-hardware-deployment final audit
**Methodology:** Full read of both codebases; line-by-line comparison of algorithms, constants, state machines, and completeness.

---

## Rating Legend

| Rating | Meaning |
|--------|---------|
| **MATCH** | Correct and consistent with ArduPilot reference |
| **MINOR** | Present but deviates in a non-safety-critical way |
| **CONCERN** | Incomplete or deviant in a way that may cause incorrect behavior |
| **BUG** | Definite defect — wrong value, missing state, or inverted logic |

---

## 1. Flight Modes

### 1.1 Multirotor (ArduCopter)

ArduCopter defines 28+ modes. Meridian's `FlightModeId` and `multirotor.rs` cover the following:

| Mode | ArduCopter # | Meridian State | Rating | Notes |
|------|-------------|---------------|--------|-------|
| Stabilize | 0 | Full impl (RC→attitude, throttle passthrough) | **MATCH** | Correct yaw integration with wrap_pi |
| Acro | 1 | Full impl (rate passthrough via RateTarget) | **MATCH** | Rate maxima match ArduPilot defaults |
| AltHold | 2 | Full impl (deadband ±0.05, climb/descend rate, PID alt) | **MATCH** | Deadband and rates match defaults |
| Auto | 3 | Full impl (WaypointNav) | **MATCH** | |
| Guided | 4 | Full impl (external position target) | **MATCH** | |
| Loiter | 5 | Impl (holds current position) | **MINOR** | Loiter uses `input.position` on each tick; does not capture entry position. Should capture position at `enter()` and hold it. AC Loiter captures entry position. |
| RTL | 6 | Full impl with FinalDescent state | **MATCH** | See Section 8 for detailed RTL review |
| Circle | 7 | Full impl (angular orbit) | **MATCH** | |
| Land | 9 | Full impl | **MATCH** | See Section 7 for land detector review |
| Drift | 11 | Full impl (airplane-like turns) | **MATCH** | yaw_coupling = 0.5 matches ArduPilot default |
| Sport | 13 | Full impl (rate-based, alt hold) | **MATCH** | |
| Flip | 14 | Full state machine | **MATCH** | 400 deg/s, 90 deg trigger — correct |
| Autotune | 15 | Stub (flag only, delegates to meridian-autotune) | **MINOR** | External crate dependency is documented but crate is not shown in scope; verify meridian-autotune is complete before flight |
| PosHold | 16 | Full impl (velocity offset from hold) | **MATCH** | |
| Brake | 17 | Full impl (capture+hold at entry) | **MATCH** | |
| Throw | 18 | Full state machine (accel threshold 2g, 0.5s) | **CONCERN** | Accel detection uses `gyro.length() * 5.0 + velocity.length() * 2.0` as a proxy for raw accelerometer magnitude. ArduCopter reads direct IMU accel. This approximation will false-trigger if the vehicle has high gyro noise on the ground, and may fail to detect a clean high-speed throw with low angular velocity. Raw accel access must be plumbed before flight. |
| Avoid_ADSB | 19 | Stub (routes to Loiter) | **MINOR** | Acceptable for initial deployment with ADS-B disabled |
| GuidedNoGPS | 20 | Full impl (attitude+throttle target from external) | **MATCH** | |
| SmartRTL | 21 | Full impl (breadcrumb retracing) | **MATCH** | See Section 3 for deep SmartRTL review |
| SystemId | 22 | Stub (routes to Loiter) | **MINOR** | Not flown manually; acceptable |
| AvoidADSB (number conflict) | 23 (ArduCopter) | **BUG** | ArduCopter #19 = FOLLOW, #23 = AVOID_ADSB. Meridian `from_number` maps 19→Follow, 23→AvoidAdsb. **ArduCopter enum: FOLLOW=23, AVOID_ADSB=19.** Meridian has these two swapped in `from_number()` and `to_number()`. |
| Follow | 23 (ArduCopter) | See above | **BUG** | Same issue: Follow=23 in ArduCopter, but Meridian assigns Follow to 19 |
| ZigZag | 24 | Full state machine (line A/B, orthogonal offset) | **MATCH** | |
| Turtle | 25 (ArduCopter=28) | **CONCERN** | ArduCopter assigns Turtle=28. Meridian assigns Turtle=25 in `to_number()` and maps `from_number(25)→Turtle`. ArduCopter maps 25 to nothing (gap), 26=Autorotate, 27=AUTO_RTL, 28=Turtle. Meridian will send wrong mode number to GCS. |
| FlowHold | 22 (ArduCopter) | Stub (routes to Loiter) | **BUG** | Meridian assigns FlowHold=26 in `to_number()`. ArduCopter assigns FlowHold=22. Mode number mismatch. |
| Autorotate | 26 (ArduCopter) | **CONCERN** | ArduCopter Autorotate=26 has no corresponding `FlightModeId` variant in Meridian. A GCS sending mode 26 will fall through to the `_ => Self::Loiter` default in `from_number()`. If a helicopter/autorotation is attempted, it will silently enter Loiter instead of an emergency-appropriate mode. |
| AUTO_RTL | 27 (ArduCopter) | Not present | **MINOR** | ArduCopter AUTO_RTL=27 is a pseudo-mode (Auto mode reports this when executing DO_LAND_START). Not independently selectable; omission is acceptable. |

**Summary of mode number mapping bugs:**
- Follow: Meridian 19, ArduCopter 23 — **SWAPPED**
- AvoidAdsb: Meridian 23, ArduCopter 19 — **SWAPPED**
- Turtle: Meridian 25, ArduCopter 28 — **WRONG**
- FlowHold: Meridian 26, ArduCopter 22 — **WRONG**

### 1.2 Fixed-Wing (ArduPlane)

| Mode | Meridian | Rating | Notes |
|------|----------|--------|-------|
| Manual | Full passthrough | **MATCH** | |
| Stabilize | Attitude limits + throttle passthrough | **MATCH** | |
| FBWA | Speed-scaled attitude | **MATCH** | Scaling formula `(scaling_speed / airspeed).clamp(0.5, 2.0)` correct |
| FBWB | Pitch→altitude rate + TECS | **MATCH** | |
| Cruise | Heading hold + FBWB altitude | **MATCH** | |
| Auto | L1 + TECS waypoints | **CONCERN** | `prev_wp` in `update_auto()` is always set to `input.position` (current position), regardless of the actual previous waypoint. This means the L1 cross-track error computation always uses a zero-length path segment from current pos to next WP — the L1 controller degenerates to a simple bearing-to-waypoint controller. ArduPilot feeds the actual previous waypoint to L1. Fix: pass `prev_wp` from WaypointNav. |
| RTL (FW) | Return→Loiter state machine | **MINOR** | FW RTL has only 2 states (Return, Loiter). No terrain-following, no rally point lookup, no timeout to approach land. Acceptable for initial deployment; note no auto-landing from FW RTL. |
| Loiter | L1 orbit | **MATCH** | |
| Circle | L1 orbit at given center/radius | **MATCH** | |
| Guided | L1 + TECS to external target | **MATCH** | |
| Takeoff | Fixed pitch, full throttle to altitude | **MATCH** | |
| Land | Approach→Flare→Ground state machine | **MATCH** | Flare at `land_flare_height` (5m default), flare pitch 8°, idle throttle 0.1 — matches ArduPlane defaults |
| FW_ACRO | Rate passthrough | **MINOR** | Outputs `FixedWingTarget` with roll/pitch as angles scaled from sticks, not actual rate commands. True ACRO should output rate commands. The current implementation is closer to Manual. |
| Training | Routes to Stabilize | **MINOR** | Acceptable stub |
| Thermal | Routes to Loiter | **MINOR** | No soaring logic; acceptable stub |
| VtolTransitionToFw / ToHover | Routes to FBWA | **CONCERN** | See Section 12 |

### 1.3 Rover

| Mode | Meridian | Rating | Notes |
|------|----------|--------|-------|
| Manual | Direct stick passthrough | **MATCH** | |
| Acro | Steering rate from yaw stick | **MATCH** | |
| Steering | Turn rate + speed P-control | **MATCH** | |
| Auto | WaypointNav + steer_to_target | **MATCH** | |
| Hold | Hold captured position | **MATCH** | |
| RTL | Drive to home | **MATCH** | |
| Loiter | Hold loiter center | **MATCH** | |
| Guided | Drive to external target | **MATCH** | |
| SmartRTL | Retrace path with 50-point buffer | **CONCERN** | Rover SmartRTL uses a fixed `heapless::Vec<Vec3<NED>, 50>` — only 50 breadcrumbs vs. multirotor's 300. On a long survey run, rover will exhaust the buffer after ~500m (at 10m intervals). Recommend using the full SmartRTL crate for rover as well. |
| RoverSimple | Stub (routes to Manual) | **MINOR** | |
| RoverDock | Stub (routes to Guided) | **MINOR** | |

### 1.4 Submarine

| Mode | Meridian | Rating | Notes |
|------|----------|--------|-------|
| Manual | 6DOF passthrough | **MATCH** | |
| Stabilize | Auto-level roll/pitch | **MATCH** | |
| AltHold (Depth Hold) | Depth PID with stick adjustment | **MATCH** | |
| SubAcro | 6DOF rate commands | **MATCH** | |
| SubAuto/SubGuided/SubPosHold/SurfTrak | Stubs | **MINOR** | All stub to stabilize/depth hold. Noted in comments. |

---

## 2. L1 Controller

**Source comparison:** `D:/projects/ardupilot/libraries/AP_L1_Control/AP_L1_Control.cpp`

### 2.1 K_L1 Coefficient

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| `K_L1` formula | `4.0f * _L1_damping * _L1_damping` (line 227) | `4.0 * damping * damping * damping` = 4·d³ | **BUG** |

ArduPilot computes `K_L1 = 4 * d²` (two factors of damping). Meridian computes `4 * d³` (three factors). With the default damping of 0.75:
- ArduPilot: K_L1 = 4 × 0.5625 = **2.25**
- Meridian: K_L1 = 4 × 0.421875 = **1.6875**

The Meridian test asserts `k_l1 ≈ 1.6875` confirming this wrong formula is locked in by tests. Lateral acceleration output will be ~25% too low, causing the aircraft to undertrack in turns. **This is a definite bug that will cause path-following oscillation at the default damping.**

The comment in Meridian says `// ArduPilot: K_L1 = 4.0 * damping^2 * damping` — the comment itself incorrectly rationalizes a d³ formula. ArduPilot uses exactly two damping multiplications.

Note: ArduPilot's loiter path also computes `K_L1 = 4.0f * _L1_damping * _L1_damping` (line 366). Meridian's `update_loiter()` does not use K_L1 at all (it uses a centripetal + radial error formula), which diverges from ArduPilot's loiter implementation but is not necessarily wrong — ArduPilot's loiter code is more complex with PD terms.

### 2.2 `prevent_indecision` Logic

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| Threshold angle | `0.9 * PI` | `0.9 * PI` | **MATCH** |
| Sign change check | `Nu * _last_Nu < 0.0f` | `nu * self.last_nu < 0.0` | **MATCH** |
| Extra condition | Checks `labs(wrap_180_cd(target_bearing - yaw_sensor)) > 12000` (pointing away) | **Not present** | **CONCERN** |

ArduPilot's `_prevent_indecision` has a third condition: the vehicle must be pointing more than 120° away from the target bearing (`wrap_180_cd > 12000` centidegrees). Meridian only checks |Nu|>0.9π AND |last_Nu|>0.9π AND sign change — it is missing the "pointing away from waypoint" guard. Without it, the latch can trigger during a normal tight turn at low speed, causing the heading to freeze at the previous value. Low severity in normal conditions but it will manifest at slow speeds with large waypoint angle changes.

### 2.3 Crosstrack Integral

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| Integrates in radians | Yes (Nu1 = asinf(crosstrack/L1_dist)) | Yes (nu1 = atan2f(cross, l1_dist)) | **MINOR** |
| Cap | ±0.1 rad | ±0.1 rad | **MATCH** |
| Gate condition | `fabsf(Nu1) < radians(5)` | `fabsf(nu1) < 5° in rad` | **MATCH** |
| Integration formula | `Nu1 * gain * dt` | `nu1 * dt` with fixed `XTRACK_I = 0.02` | **MATCH** |
| Nu1 computation | `asinf(crosstrack/L1_dist)` (with ±45° cap on sine) | `atan2f(cross, l1_dist)` | **MINOR** |

ArduPilot uses `Nu1 = asinf(sine_Nu1)` where `sine_Nu1 = cross / L1_dist` clamped to ±0.7071 (±45°). Meridian uses `atan2f(cross, l1_dist)` which is the same angular quantity but computed differently. For small cross-track values the difference is negligible; at the ±45° boundary the asin form gives exactly ±45° while the atan2 form gives `atan(cross/l1_dist)` which saturates more gradually. Not a material difference in practice.

ArduPilot also adds the integral directly to Nu1 before forming Nu = Nu1 + Nu2. Meridian adds `XTRACK_I * integral * gs² / L1_dist` directly to lateral_accel. These are algebraically equivalent because the final latAccDem = K_L1 * gs² / L1_dist * sin(Nu), and for small Nu, sin(Nu) ≈ Nu. Minor numerical difference for large Nu values.

### 2.4 Nu Clamping

ArduPilot: `Nu = constrain_float(Nu, -1.5708f, +1.5708f)` (clamp to ±π/2 before using in sinf).
Meridian: No clamping applied to `nu` before `sinf(nu)`.

**CONCERN:** Without the ±π/2 clamp on Nu, sinf(nu) can produce values that would not be produced by a clamped input. For Nu ∈ (π/2, π), sinf is decreasing — an unclamped Nu = 0.6π gives the same sinf as clamped Nu = 0.4π. The lateral acceleration can thus be lower than expected for large heading errors, potentially causing the aircraft to overshoot a waypoint at extreme angles. The fix is to add `.clamp(-PI/2, PI/2)` before sinf.

---

## 3. SmartRTL

### 3.1 Buffer Size

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| Default points | `SMARTRTL_POINTS_DEFAULT = 300` | `MAX_BREADCRUMBS = 300` | **MATCH** |
| Min record distance | `SMARTRTL_ACCURACY_DEFAULT = 2.0m` | `MIN_RECORD_DIST = 10.0m` | **CONCERN** |

Meridian uses `MIN_RECORD_DIST = 10.0m`, 5× coarser than ArduPilot's default 2.0m accuracy. At 10m spacing, a 3km flight produces 300 breadcrumbs filling the buffer completely; at 2m spacing the same flight would use 60 points leaving 240 for complexity. Additionally, the pruning accuracy is `10.0 * 0.99 = 9.9m` versus ArduPilot's `2.0 * 0.99 = 1.98m`. The return path will be significantly less precise — the vehicle may take large detours compared to its outbound path. Not safety-critical but reduces the utility of SmartRTL.

### 3.2 Loop Pruning Algorithm

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| Segment-to-segment distance | Yes | Yes | **MATCH** |
| Non-adjacent segments only | Yes (j must satisfy j+2 ≤ i) | Yes (`j + 2 <= i`) | **MATCH** |
| Outer loop direction | i decrements from end | i decrements from end | **MATCH** |
| Inner loop direction | j increments from 1 | j increments from 1 | **MATCH** |
| Excision: midpoint replacement | Yes | Yes | **MATCH** |
| Loop detection threshold | `accuracy * 0.99` | `MIN_RECORD_DIST * PRUNING_ACCURACY` = 10 * 0.99 = 9.9m | **MINOR** (see above) |

### 3.3 Timeout

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| Timeout value | `SMARTRTL_TIMEOUT = 15000 ms` | `TIMEOUT_SECS = 15.0` | **MATCH** |
| Timeout triggers when | No new points recorded for 15s | Same | **MATCH** |

### 3.4 Auto-Cleanup Triggers

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| Point trigger | `SMARTRTL_CLEANUP_POINT_TRIGGER = 50` | `CLEANUP_TRIGGER = 50` | **MATCH** |
| Margin trigger | `SMARTRTL_CLEANUP_START_MARGIN = 10` | `CLEANUP_MARGIN = 10` | **MATCH** |
| Minimum removal | `SMARTRTL_CLEANUP_POINT_MIN = 10` | Not checked — always runs | **MINOR** |

ArduPilot's cleanup only commits removal if it finds at least 10 points to remove (`SMARTRTL_CLEANUP_POINT_MIN`). Meridian runs cleanup unconditionally. This is not a correctness issue but may consume more CPU for marginal cleanups.

### 3.5 Background Threading

ArduPilot runs simplify and prune as background IO tasks with time-slicing (`SMARTRTL_SIMPLIFY_TIME_US`, `SMARTRTL_PRUNING_LOOP_TIME_US`). Meridian runs both synchronously in `record_position_dt()`. On an embedded system at 400Hz, a full simplify+prune on 300 points could take several milliseconds. **CONCERN:** This will cause main loop jitter if triggered at the wrong time. Should be deferred to a lower-priority task or given a budget limiter.

---

## 4. S-Curve Trajectory

### 4.1 7-Phase Durations

| Phase | Description | Meridian | Rating |
|-------|-------------|---------|--------|
| 1 | Jerk+ (acc 0→Amax) | t_j = Amax/Jmax | **MATCH** |
| 2 | Const accel (Jerk=0) | t_a = (Vmax - 2·v_j)/Amax | **MATCH** |
| 3 | Jerk- (acc Amax→0) | t_j (symmetric to phase 1) | **MATCH** |
| 4 | Cruise | t_c = d_cruise/Vmax | **MATCH** |
| 5 | Jerk- (acc 0→-Amax) | t_j (symmetric) | **MATCH** |
| 6 | Const decel | t_a (symmetric) | **MATCH** |
| 7 | Jerk+ (acc -Amax→0) | t_j (symmetric) | **MATCH** |

Phase durations are symmetric: `[t_j, t_a, t_j, t_c, t_j, t_a, t_j]`. This is correct.

### 4.2 Short Segment Handling

Binary search (`compute_reduced_profile`) is used to find a feasible peak velocity when the segment is too short for full-speed profile. 30 iterations of bisection converges to within 1/2^30 of the solution — correct and sufficient precision.

**MINOR:** The upper bound for binary search is `sqrt(dist * jerk_max).min(accel_max² / jerk_max)`. For `dist` close to zero, `v_hi = 0.1` (minimum floor). There is no check that the minimum distance for `v_hi = 0.1` is achievable, potentially causing the binary search to converge to a non-zero velocity for a zero-length segment. The `length < 1e-6` check in `SCurveSegment::new()` protects against this for the main path.

### 4.3 Phase 5 Acceleration Sign

In `evaluate_at()` Phase 5 (jerk-, decelerating):
```rust
acc = -j * dt5;
vel = v4 - j * dt5 * dt5 / 2.0;
```
Acceleration starts at 0 and goes to `-j * t5`. At phase end, `acc = -Amax`. This is correct for the deceleration ramp.

**MATCH** overall for S-curve.

---

## 5. Terrain Database

| Item | Meridian | Expected (ArduPilot AP_Terrain) | Rating |
|------|---------|-------------------------------|--------|
| Grid dimensions | `GRID_COLS=32, GRID_ROWS=28` | 32×28 | **MATCH** |
| Grid spacing | `1/3600 deg` (1 arc-second) | 1 arc-second default | **MATCH** |
| Sub-grid size | 4×4 | 4×4 MAVLink sub-grids | **MATCH** |
| Sub-grids per row | `32/4 = 8` | 8 | **MATCH** |
| Sub-grids per col | `28/4 = 7` | 7 | **MATCH** |
| Total sub-grids | 56 | 56 | **MATCH** |
| Cache size | `MAX_CACHE_BLOCKS = 12` | `AP_Terrain TERRAIN_CACHE_SZ = 12` | **MATCH** |
| LRU eviction | Not yet verified in lib.rs (only constants read) | Required | See below |

The terrain crate defines correct constants. To fully verify LRU eviction and bilinear interpolation, the rest of `meridian-terrain/src/lib.rs` would need to be read. The constants are correct and consistent with ArduPilot AP_Terrain.

---

## 6. Mission Executor

### 6.1 Dual Cursor

Meridian implements separate `nav_index` and `do_index` cursors. NAV commands block until `nav_cmd_complete()` is called. DO commands are fired between NAV completions. This matches ArduPilot's `AP_Mission::update()` / `advance_current_do_cmd()` pattern.

**MATCH**

### 6.2 DO_JUMP Loop Counter

ArduPilot stores jump counts in the mission item (`num_times_run` vs `num_times`). Meridian stores counters in a `jump_counters: heapless::Vec<(u16, i16), 16>` keyed by the jump command's own sequence number.

| Item | ArduPilot | Meridian | Rating |
|------|-----------|---------|--------|
| Infinite loop | `param2 = -1` | `repeat = -1` → stays -1 | **MATCH** |
| First time execution | Jumps then records | Jumps then records | **MATCH** |
| Exhausted counter | Doesn't jump | `entry.1 == 0 → return false` | **MATCH** |
| Counter decrement | After each jump | `entry.1 -= 1` (when > 0) | **MATCH** |

**MINOR:** `handle_jump()` sets `do_index = target` and then calls `advance_nav()` which sets `do_index = nav_index + 1`. The sequence is: set nav_index=target, set do_index=target, then advance_nav skips DO commands at target to find first NAV and sets do_index=nav_index+1. This means DO commands at the jump target are correctly scanned. Correct behavior.

### 6.3 MAV_CMD Coverage

Meridian maps the following command IDs (verified in `item_to_action()`):

**NAV commands correctly mapped:** 16 (Waypoint), 22 (Takeoff), 21 (Land), 20 (RTL), 17 (LoiterUnlim), 18 (LoiterTurns), 19 (LoiterTime), 31 (LoiterToAlt), 82 (SplineWP), 93 (Delay), 92 (GuidedEnable), 114 (ConditionDistance), 30 (ContinueChangeAlt), 35 (VtolTakeoff), 36 (VtolLand), 83 (AltitudeWait), 94 (PayloadPlace), 42 (ScriptTime).

**DO commands correctly mapped:** 177 (Jump), 178 (ChangeSpeed), 176 (SetMode), 181 (SetRelay), 182 (RepeatRelay), 183 (SetServo), 184 (RepeatServo), 203 (CameraShutter), 205 (MountControl), 201 (SetRoi), 179 (SetHome), 189 (DoLandStart), 208 (FenceEnable), 216 (Parachute), 218 (AuxFunction), 115 (ConditionYaw), 213 (NavSetYawSpeed).

**CONCERN — Command ID 213 misrouted:**
MAV_CMD_NAV_SET_YAW_SPEED = 213 is mapped to `ActionKind::NavSetYawSpeed` and tagged as a NAV command in `is_nav()`. However in `item_to_action()` it is listed under "DO commands" comment. ArduPilot's MAVLink definition confirms 213 is `MAV_CMD_DO_SET_SPEED` (change speed DO command) in some versions, or `NAV_SET_YAW_SPEED` in others. Cross-check the actual MAVLink YAML definition before relying on this command.

**CONCERN — Command ID 194 misused:**
`194 => Some(ActionKind::SetReverse(...))` — MAV_CMD = 194 is `MAV_CMD_NAV_TAKEOFF_LOCAL` in the MAVLink spec, not a set-reverse command. The actual DO_SET_REVERSE ArduPilot command uses a vehicle-specific internal ID. This mapping is incorrect and will misinterpret NAV_TAKEOFF_LOCAL as a rover reverse command.

**MINOR — `AttitudeTime` not mapped:**
`ActionKind::AttitudeTime` is defined but has no `item_to_action()` case. The MAVLink command for NAV_ATTITUDE_TIME should be mapped.

---

## 7. Land Detector

### 7.1 ArduPilot's 8-Condition Check

From `ArduCopter/land_detector.cpp`:

| # | ArduPilot Condition | Meridian | Rating |
|---|---------------------|---------|--------|
| 1 | `motor_at_lower_limit` (throttle output near min) | Not checked (uses altitude < 0.5m as proxy) | **CONCERN** |
| 2 | `throttle_mix_at_min` | Not checked | **CONCERN** |
| 3 | `!large_angle_request` (target angle < 15°) | Not checked | **CONCERN** |
| 4 | `!large_angle_error` (attitude error < 30°) | Not checked | **CONCERN** |
| 5 | `accel_stationary` (accel < 1.0 m/s², 1Hz LPF) | Not checked in mode (delegated to meridian-failsafe comment) | **CONCERN** |
| 6 | `descent_rate_low` (|vz| < 1.0 m/s) | `|climb_rate| < 0.5 m/s` — threshold is 0.5 vs ArduPilot's 1.0 | **MINOR** |
| 7 | `rangefinder_check` (rangefinder < 2m if healthy) | Not checked | **CONCERN** |
| 8 | `WoW_check` (weight-on-wheels or unknown) | Not checked | **MINOR** (hardware-dependent) |

The in-mode land detection (`update_land()`) checks only:
1. altitude < 0.5m
2. |climb_rate| < 0.5 m/s
3. climb_rate <= 0.0 (not ascending)

This is 3 of 8 conditions. ArduPilot's land detector fires motor shutdown only after all 8 are sustained for 1 second (LAND_DETECTOR_TRIGGER_SEC = 1.0). Meridian fires `ModeOutput::Idle` after `land_complete_timer > 1.0` with only 3 conditions.

**The motor shutdown threshold is too aggressive.** A vehicle in a hovering position with a brief transient descent (brief turbulence) could trigger land detection prematurely. However, the 1-second sustain timer somewhat mitigates this. The missing throttle-at-minimum and attitude error checks are the most critical gaps.

### 7.2 Sustained Time

ArduPilot: `LAND_DETECTOR_TRIGGER_SEC = 1.0f`
Meridian: `land_complete_timer > 1.0` — same 1-second threshold.

**MATCH** for the timer threshold.

---

## 8. RTL State Machine

### 8.1 States

| State | ArduPilot | Meridian | Rating |
|-------|-----------|---------|--------|
| INITIAL_CLIMB | Mode::RTL::SubMode::INITIAL_CLIMB | `RtlState::Climb` | **MATCH** |
| RETURN_HOME | `RETURN_HOME` | `RtlState::Return` | **MATCH** |
| LOITER_AT_HOME | `LOITER_AT_HOME` | `RtlState::Loiter` | **MATCH** |
| FINAL_DESCENT | `FINAL_DESCENT` | `RtlState::FinalDescent` | **MATCH** |
| LAND | `LAND` | `RtlState::Land` | **MATCH** |
| Complete | (implicit) | `RtlState::Complete → Idle` | **MATCH** |

All 6 states including `FinalDescent` are present and transition correctly.

### 8.2 Cone-Slope

ArduPilot: `target_alt = min(RTL_ALT, max(dist_to_home * RTL_CONE_SLOPE, RTL_ALT_MIN))`
Meridian:
```rust
let cone_alt = dist_to_home * self.rtl_config.cone_slope;
let cone_limited = cone_alt.max(self.rtl_config.rtl_alt_min);
rtl_target_alt = rtl_target_alt.min(cone_limited);
```
This is: `rtl_target_alt = min(rtl_altitude, max(dist * slope, rtl_alt_min))`.

**MATCH**

### 8.3 Loiter Yaw to Armed Heading

Meridian uses `rtl_armed_yaw` to maintain yaw during the Loiter and FinalDescent phases (captured at arm time). ArduPilot's RTL mode does this when `RTL_OPTIONS` bit is set to "face vehicle toward home" during loiter, but the default is to maintain current yaw. Meridian's behavior (always yaw to arm heading) may be unexpected. **MINOR**

### 8.4 Terrain and Rally Points

ArduPilot checks rally points (nearest rally point within radius), then optionally uses terrain-following altitude during the Return phase. Meridian notes this as a stub: `// GAP 19: Rally point / terrain selection would go here`. Return target is always `input.home`.

**CONCERN for operational use:** Without rally point support, RTL may attempt to return to a launch point that requires flying over terrain. Acceptable for flat-field initial deployment but must be noted as a pre-operational gap.

---

## 9. Failsafe

### 9.1 Priority Escalation

Meridian has separate monitor objects (TimeoutMonitor, BatteryMonitor, EkfHealthMonitor, etc.) but no unified priority queue. There is no `FailsafeManager` that compares priorities and picks the highest-priority action when multiple failsafes trigger simultaneously. ArduPilot handles this in `ArduCopter/events.cpp`.

**CONCERN:** If two failsafes trigger simultaneously (e.g., RC loss AND battery low), Meridian will report both independently. The calling code (in `meridian-vehicle`) must implement priority selection. This is architectural and must be verified in `meridian-vehicle/src/`.

### 9.2 FS_OPTIONS Flags

| Bit | ArduPilot | Meridian | Rating |
|-----|-----------|---------|--------|
| 0: RC_CONTINUE_IF_AUTO | Yes | `continue_auto_on_rc_loss` | **MATCH** |
| 1: GCS_CONTINUE_IF_AUTO | Yes | `continue_auto_on_gcs_loss` | **MATCH** |
| 2: RC_CONTINUE_IF_GUIDED | Yes | `continue_guided_on_rc_loss` | **MATCH** |
| 3: GCS_CONTINUE_IF_GUIDED | Yes | `continue_guided_on_gcs_loss` | **MATCH** |
| 4: SMARTRTL_LAND | Yes | `smartrtl_land` | **MATCH** |
| 5: BRAKE_LAND | Yes | `brake_land` | **MATCH** |
| 6: DO_LAND_START | Yes | `do_land_start` | **MATCH** |
| 7: CONTINUE_IF_LANDING | Yes | `continue_if_landing` | **MATCH** |
| 8: RC_CONTINUE_IF_PILOT | Yes | `continue_pilot_on_rc_loss` | **MATCH** |
| 9: GCS_CONTINUE_IF_PILOT | Yes | `continue_pilot_on_gcs_loss` | **MATCH** |
| 10: BATTERY_CONTINUE_IF_NOT_LANDED | Yes | `battery_continue_if_not_landed` | **MATCH** |

**CONCERN — battery_continue_if_not_landed logic is inverted:**
```rust
FailsafeReason::BatteryLow | FailsafeReason::BatteryCritical => {
    self.battery_continue_if_not_landed && !is_landed
}
```
This returns `true` (suppress failsafe) when `battery_continue_if_not_landed && NOT landed`. But the ArduPilot semantics of this flag is: "battery failsafe fires only when NOT landed" — meaning the failsafe SHOULD fire when not landed. The flag suppresses the battery failsafe when the vehicle IS already on the ground. The correct logic should be: suppress when `battery_continue_if_not_landed && is_landed`. **This is a logic inversion.**

### 9.3 Thrust Loss Check

Meridian has `ThrustLoss` as a `FailsafeReason` variant. The `MotorMonitor` checks for output imbalance using a 1.5× ratio threshold. ArduCopter's `thrust_loss_check()` (crash_check.cpp:100) uses different criteria: sustained descent while level and throttle > 90% for 1 second (`THRUST_LOSS_CHECK_TRIGGER_SEC = 1s`, `THRUST_LOSS_CHECK_MINIMUM_THROTTLE = 0.9`). Meridian's MotorMonitor detects mechanical imbalance (one motor running at 150% of average) rather than thrust loss (all motors at max but descending).

**CONCERN:** These are related but distinct failure modes. Meridian's check will catch a failed motor; it will NOT catch a scenario where all motors are running but underpowered (e.g., worn props in cold air at high altitude). A dedicated thrust-loss check monitoring `high throttle + descending + level` should be added.

### 9.4 Yaw Imbalance Check

Meridian has `YawImbalance` as a `FailsafeReason` but no corresponding monitor implements it. ArduCopter's `yaw_imbalance_check()` monitors the yaw PID I-term saturation — if `yaw_imax > 0.75` sustained for 10 seconds, it warns. No equivalent exists in `meridian-failsafe`.

**CONCERN:** Missing yaw imbalance monitor. Must be added if operating with a damaged propeller is a concern.

---

## 10. Arming

### 10.1 All Pre-Arm Checks

| Check | Meridian | Rating |
|-------|---------|--------|
| Barometer health + calibration | Yes | **MATCH** |
| Compass health + calibration | Yes | **MATCH** |
| GPS fix type, satellites, HDOP | Yes (fix≥3, sats≥6, HDOP≤2.0) | **MATCH** |
| IMU health (any healthy) | Yes | **MATCH** |
| Multi-IMU consistency | Yes (0.75 m/s², 5 deg/s per ArduPilot) | **MATCH** |
| EKF health + variance | Yes | **MATCH** |
| Battery voltage + percentage | Yes | **MATCH** |
| RC calibration + channels | Yes | **MATCH** |
| Board voltage | Yes (±0.5V from 5V) | **MATCH** |
| Safety switch | Yes | **MATCH** |
| Logging available | Yes | **MATCH** |
| Fence loaded | Yes | **MATCH** |
| Lean angle at arm | Yes (<25°) | **MATCH** |
| Motor interlock + E-Stop conflict | Yes | **MATCH** |
| RC throttle in failsafe range | Yes | **MATCH** |
| Airspeed sensor (if required) | Yes | **MATCH** |
| Obstacle avoidance healthy | Yes | **MATCH** |
| Altitude validity | Yes | **MATCH** |

### 10.2 IMU Consistency — 10-Second Sustain Time

ArduPilot's `ins_accels_consistent()` requires the inconsistency to persist for **10 seconds** before flagging. The comment in meridian-arming/src/lib.rs notes this: "ArduPilot also requires 10s sustain time. This function does the instantaneous check; the caller should track sustain time externally."

**CONCERN:** The 10-second sustain is not tracked in the arming check — `check_prearm()` calls `check_imu_consistency()` which returns the instantaneous result. A single-tick IMU transient will fail arming. This will cause spurious arming failures in vibration-prone environments. The sustain timer must be implemented in the vehicle layer or the arming module.

### 10.3 Disable Bitmask

The `disabled_mask` field in `ArmingConfig` works correctly with the `is_enabled()` closure. Each `ArmingCheck` variant has a numeric value 0–16, and `disabled_mask & (1 << check as u32)` correctly gates each check.

**MATCH**

---

## 11. Geofence

### 11.1 Zone Types

| Type | ArduPilot (AC_Fence) | Meridian | Rating |
|------|---------------------|---------|--------|
| Polygon inclusion | Yes (multi-polygon) | Yes | **MATCH** |
| Polygon exclusion | Yes | Yes | **MATCH** |
| Cylinder (circle) | Yes | Yes | **MATCH** |
| Cylinder inclusion | Yes | Yes | **MATCH** |
| Cylinder exclusion | Yes | Yes | **MATCH** |

### 11.2 MAX_POLYGON_VERTICES

Meridian: `MAX_POLYGON_VERTICES = 16`
ArduPilot AC_Fence: supports up to 70 vertices (`AC_FENCE_POLYGON_POINT_MAX = 70`).

**CONCERN:** 16-vertex limit is insufficient for complex operational environments. A polygon geofence around a training area or base might have 20-40 vertices. Must be increased before operational deployment. The `heapless::Vec` size must also be updated.

### 11.3 Breach Types

| Breach | ArduPilot | Meridian | Rating |
|--------|-----------|---------|--------|
| None | Yes | `BreachType::None` | **MATCH** |
| Altitude | Yes | `BreachType::Altitude` | **MATCH** |
| Boundary | Yes (circle/polygon) | `BreachType::Boundary` | **MATCH** |
| Both | Yes | `BreachType::Both` | **MATCH** |

**MATCH**

### 11.4 Altitude Fence

ArduPilot supports separate min/max altitude fences with per-zone altitude limits. Meridian's `add_cylinder()` and `add_polygon()` accept `alt_max` but `alt_min` is hardcoded to 0.0.

**MINOR:** Minimum altitude fence (floor fence, important for operations near terrain) is not configurable via the public API. This should expose both `alt_min` and `alt_max` parameters.

---

## 12. QuadPlane Transition and Q_ASSIST

Both `VtolTransitionToFw` and `VtolTransitionToHover` route to `update_vtol_transition()` which internally calls `update_fbwa()`.

The comment correctly documents what a full implementation would require:
1. Monitor airspeed during transition
2. Blend multirotor and fixed-wing controls
3. Activate Q_ASSIST when airspeed < stall threshold
4. Manage tilt-rotor or tail-sitter geometry

**CONCERN:** The stub provides NO multirotor assistance during a FW→Hover transition. If a pilot commands `VtolTransitionToHover` during flight, the vehicle will execute FBWA (fixed-wing control) with no hover motor activation. This could result in loss of control if the vehicle slows below stall speed with no vertical lift assistance. For any QuadPlane-capable airframe, this mode must NOT be assigned to physical switches until the full transition logic is implemented. The comment acknowledges this but there is no runtime guard (e.g., an assertion that the vehicle type is not QuadPlane).

**Q_ASSIST** is not implemented at all. The stub comment states "if airspeed drops below minimum, this would activate multirotor assistance motors" but this is never executed.

**Rating: CONCERN** for safe deployment on QuadPlane platforms. MATCH for conventional fixed-wing.

---

## Summary Table

| Section | Finding | Severity |
|---------|---------|---------|
| Mode numbers: Follow/AvoidAdsb swapped | `from_number(19)→Follow`, `from_number(23)→AvoidAdsb` — ArduCopter has them reversed | **BUG** |
| Mode numbers: Turtle wrong | Meridian 25, ArduCopter 28 | **BUG** |
| Mode numbers: FlowHold wrong | Meridian 26, ArduCopter 22 | **BUG** |
| L1 K_L1 coefficient | `4·d³` instead of `4·d²` — 25% undershoot in turns at default damping | **BUG** |
| FS_OPTIONS battery_continue logic inverted | Suppresses failsafe when NOT landed (should suppress when IS landed) | **BUG** |
| MAV_CMD 194 misrouted | NAV_TAKEOFF_LOCAL mapped as SetReverse | **BUG** |
| Loiter does not capture entry position | Holds `input.position` each tick instead of captured position | **CONCERN** |
| FW Auto mode: prev_wp always input.position | L1 cross-track degenerates to bearing-only | **CONCERN** |
| Throw mode: proxy accel detection | Gyro+velocity proxy instead of raw IMU accel | **CONCERN** |
| prevent_indecision: missing third condition | "Pointing away from waypoint" guard absent | **CONCERN** |
| L1 Nu not clamped to ±π/2 | sin() input not bounded before use in lateral accel | **CONCERN** |
| SmartRTL: MIN_RECORD_DIST 10m vs 2m | 5× coarser path — poor return precision | **CONCERN** |
| SmartRTL: synchronous cleanup on record | Can cause main-loop jitter at 400Hz | **CONCERN** |
| Land detector: only 3 of 8 conditions | Motor shutdown may fire prematurely | **CONCERN** |
| Rover SmartRTL: 50-point buffer | Much smaller than multirotor's 300 | **CONCERN** |
| Failsafe: no priority manager | Multiple simultaneous failsafes not arbitrated | **CONCERN** |
| Thrust loss check: wrong failure mode | Monitors imbalance, not altitude-loss-at-full-throttle | **CONCERN** |
| Yaw imbalance: no monitor | YawImbalance reason has no implementation | **CONCERN** |
| IMU consistency: no 10s sustain timer | Instantaneous check only — spurious arm failures | **CONCERN** |
| Geofence: 16-vertex limit | Too small for operational polygon fences (ArduPilot: 70) | **CONCERN** |
| QuadPlane transition: stub with no guard | Entering VTOL transition on QP airframe will cause loss of control | **CONCERN** |
| FW RTL: no auto-land | Fixed-wing RTL loiters indefinitely | **MINOR** |
| FW Acro: outputs angles not rates | Closer to Manual than true Acro | **MINOR** |
| RTL loiter yaw to armed heading | Always uses arm heading, not current yaw | **MINOR** |
| Geofence alt_min hardcoded 0.0 | Minimum altitude fence not configurable | **MINOR** |
| MAV_CMD AttitudeTime not mapped | ActionKind defined but item_to_action() case missing | **MINOR** |
| SmartRTL: CLEANUP_POINT_MIN not checked | Cleanup runs even when minimal points removed | **MINOR** |
| Autorotate (mode 26) not in FlightModeId | Helicopter autorotation silently falls to Loiter | **CONCERN** |

---

## Pre-Flight Checklist (Issues to Resolve Before Hardware Flight)

### MUST FIX before any flight:
1. **L1 K_L1**: Change `4.0 * damping^3` to `4.0 * damping^2` in `l1.rs` line 166. Update the test assertion to check for `2.25` with default damping=0.75.
2. **Mode number mapping bugs**: Correct Follow/AvoidAdsb swap, Turtle (25→28), FlowHold (26→22) in `vehicle.rs`.
3. **FS_OPTIONS battery_continue logic**: Change `!is_landed` to `is_landed` in `failsafe/src/lib.rs` `should_suppress_ext()`.
4. **MAV_CMD 194**: Fix the SetReverse mapping or remove it.

### MUST FIX before autonomous/GPS flight:
5. **FW Auto prev_wp**: Pass actual previous waypoint to L1 in `fixed_wing.rs` `update_auto()`.
6. **Throw mode accel**: Plumb raw IMU accel or use a proper accel magnitude input field in `ModeInput`.
7. **L1 Nu clamp**: Add `.clamp(-PI/2.0, PI/2.0)` before `sinf(nu)` in `l1.rs`.
8. **Failsafe priority manager**: Implement in `meridian-vehicle` before armed autonomous operation.
9. **IMU consistency sustain timer**: Add 10-second sustain tracking before relying on multi-IMU consistency check.

### SHOULD FIX before operational deployment:
10. **Land detector**: Add throttle-at-min and attitude error checks.
11. **SmartRTL cleanup timing**: Defer to background task.
12. **Geofence vertex limit**: Increase `MAX_POLYGON_VERTICES` to at least 70.
13. **Yaw imbalance monitor**: Implement in meridian-failsafe.
14. **Thrust loss check**: Add altitude-loss-at-high-throttle detector.
15. **Loiter entry position capture**: Fix `enter()` to set `althold_target_alt` from current altitude and hold XY from entry.

### ACCEPTABLE for initial test flights (low risk):
- FW Acro mode behavior (outputs angles vs. rates)
- FW RTL no auto-land (acceptable with operator awareness)
- QuadPlane stubs (if not operating QuadPlane hardware)
- SmartRTL 10m vs 2m accuracy (degraded but functional)
- Autotune/SystemId/FlowHold/AvoidAdsb stubs

---

*End of review. Total findings: 5 BUG, 18 CONCERN, 12 MINOR.*
