# Parity Verification: ArduPilot vs Meridian
## Subsystems: Navigation, Modes, Mission, Failsafe/Arming
**Audit Date:** 2026-04-02
**Auditor:** Automated audit (automated code audit)

---

## Scope

Four subsystems compared:

1. `AP_L1_Control` / `AC_WPNav` / `AP_SmartRTL` → `meridian-nav`
2. `ArduCopter/mode*.cpp`, `ArduPlane/`, `Rover/`, `ArduSub/` → `meridian-modes`
3. `AP_Mission` → `meridian-mission`
4. `AP_Arming` / `ArduCopter/failsafe.cpp` → `meridian-failsafe` / `meridian-arming`

---

## 1. Navigation (`meridian-nav`)

### 1.1 L1 Controller (`l1.rs`)

**Core formula status: MOSTLY CORRECT, one divergence**

ArduPilot computes lateral acceleration as:

```cpp
float K_L1 = 4.0f * _L1_damping * _L1_damping;
_latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);
```

Meridian computes:

```rust
self.lateral_accel = 2.0 * groundspeed * groundspeed * sin_err / l1_dist;
```

**GAP 1 (HIGH): L1 lateral accel formula coefficient is wrong.**
ArduPilot uses `K_L1 = 4 * damping^2` as the numerator multiplier. With default damping=0.75, K_L1=2.25. Meridian hardcodes 2.0. At default settings the error is 11%. The correct formula is `4 * damping^2 * V^2 * sin(Nu) / L1`.

**GAP 2 (HIGH): prevent_indecision logic is different.**
ArduPilot's `_prevent_indecision(Nu)` checks four conditions simultaneously:
- `|Nu| > 0.9π` (bearing error > 162°)
- `|_last_Nu| > 0.9π`
- wrap_180 of (target_bearing - yaw_sensor) > 12000 cd (pointing away)
- `Nu * _last_Nu < 0.0` (sign flip)

Meridian does NOT implement this function at all. The `wp_overshoot` flag is a completely different mechanism (it detects past-waypoint overshoot, not bearing oscillation). The anti-oscillation Nu latch is absent.

**GAP 3 (MEDIUM): Crosstrack integral accumulation condition.**
ArduPilot only integrates when `|Nu1| < radians(5)` (vehicle nearly aligned with path). Meridian integrates unconditionally whenever `dt > 0.0`. This will cause incorrect integral buildup during large-angle captures and initial path intercepts.

**GAP 4 (MEDIUM): L1 dist_min parameter.**
ArduPilot's `update_waypoint` accepts a `dist_min` parameter that floors the L1 distance: `_L1_dist = MAX(0.3183099f * damping * period * groundSpeed, dist_min)`. Meridian always uses `l1_dist.max(0.1)` with no configurable floor. This matters at low groundspeed near waypoints.

**GAP 5 (LOW): Reverse mode yaw wrapping.**
ArduPilot has `get_yaw()` / `get_yaw_sensor()` which wraps yaw by π when `_reverse` is true (for reverse taxiing). Meridian has no reverse mode support.

**Status summary:**
- L1 distance formula: CORRECT (`damping * period * V / π`)
- Crosstrack integral field: PRESENT (but accumulates incorrectly)
- Integral gain constant (0.02): CORRECT
- Integral windup cap (20.0): PRESENT (ArduPilot uses 0.1 radians, different units — see below)

**GAP 6 (MEDIUM): Integral units and cap differ.**
ArduPilot integrates in radian units (Nu1) clamped to ±0.1 rad. Meridian integrates in meter·seconds (crosstrack × dt) clamped to ±20.0. These are not equivalent even though both use the same `XTRACK_I = 0.02` gain. ArduPilot's gain multiplies angle; Meridian's multiplies distance×time. The contribution to `lateral_accel` is dimensionally different.

---

### 1.2 Waypoint Navigation / AC_WPNav (`waypoint.rs`, `scurve.rs`)

**GAP 7 (HIGH): Spline waypoints use Catmull-Rom, not ArduPilot's Hermite formulation.**
ArduPilot uses `AC_WPNav` with a custom cubic Hermite spline that matches tangent continuity at each waypoint based on the approach/departure speed. Meridian's `SplineSegment` uses Catmull-Rom with uniform parameterization. For most cases these produce similar curves, but the entry/exit tangent computation differs. ArduPilot's spline pre-blends the tangent using both the previous and next waypoint vectors scaled by speed; Meridian uses the simple Catmull-Rom average tangent. This is a semantic gap, not a structural absence.

**S-curve 7-phase trajectory: PRESENT and CORRECT.**
`SCurveSegment` in `scurve.rs` implements all 7 phases (Jerk+, Const-Accel, Jerk-, Cruise, Jerk-, Const-Decel, Jerk+) with correct kinematic equations. Phase duration computation and state machine are sound.

**Terrain following in WPNav: NOT PRESENT in meridian-nav.**
ArduPilot's `AC_WPNav` has `set_wp_destination_loc()` which accepts `Location` objects with altitude frames (`ABOVE_TERRAIN`). The 32×28 grid-based terrain lookup is handled by `AP_Terrain`. Meridian has `meridian-terrain` (correct 32×28 grid, bilinear interpolation, LRU cache) but `meridian-nav`'s `WaypointNav` only stores NED positions — it has no altitude frame field and no call into `meridian-terrain`. The terrain-following path for copter/plane is structurally disconnected.

---

### 1.3 SmartRTL (`smartrtl.rs`)

**GAP 8 (CRITICAL): Loop pruning is absent. Only Douglas-Peucker is implemented.**

ArduPilot SmartRTL has a two-stage cleanup:
1. **Douglas-Peucker simplification** — removes collinear/redundant points
2. **Loop pruning** — detects and removes path loops by finding pairs of points within `SMARTRTL_PRUNING_DELTA` of each other that indicate the vehicle traced a closed loop

The loop pruner is the safety-critical differentiator. It prevents the vehicle from retracing dangerous ground-level detours. ArduPilot keeps a separate `_prune.loops[]` buffer, runs in a time-limited background thread (200 μs max), and removes entire loop segments from the breadcrumb trail.

Meridian `SmartRTL` only implements Douglas-Peucker (`simplify_path()`). There is no loop detection, no loop buffer, and no pruning. A vehicle that circled back over an obstacle during the outbound leg will attempt to fly back through it on return.

**GAP 9 (HIGH): Breadcrumb buffer is too small.**
ArduPilot defaults to 300 points (`SMARTRTL_POINTS_DEFAULT`) with a max of 500 (`SMARTRTL_POINTS_MAX`). Each point is ~20 bytes. Meridian uses `MAX_BREADCRUMBS = 50`. This supports only ~8 minutes of flight at the 10-meter minimum spacing, far below ArduPilot's ~50 minutes at default settings. The path cache in `multirotor.rs` also caps at 50 entries.

**GAP 10 (MEDIUM): No timeout disablement.**
ArduPilot disables SmartRTL if no breadcrumbs have been recorded for 15 seconds (`SMARTRTL_TIMEOUT = 15000`). Meridian has no such timeout guard.

**GAP 11 (MEDIUM): No cleanup trigger on breadcrumb count.**
ArduPilot runs simplification when `SMARTRTL_CLEANUP_POINT_TRIGGER = 50` new points have been added, and again when only `SMARTRTL_CLEANUP_START_MARGIN = 10` slots remain. Meridian only calls `simplify_path()` explicitly — there is no automatic background cleanup.

---

## 2. Flight Modes (`meridian-modes`)

### 2.1 Multirotor Modes

**Modes present in ArduCopter (29 total including heli-specific):**

| Mode | ArduPilot file | Meridian | Status |
|---|---|---|---|
| STABILIZE | mode_stabilize.cpp | `update_stabilize` | PRESENT |
| ACRO | mode_acro.cpp | `update_acro` | PRESENT (simplified) |
| ALT_HOLD | mode_althold.cpp | `update_althold` | PRESENT |
| AUTO | mode_auto.cpp | `update_auto` | PRESENT (simplified) |
| GUIDED | mode_guided.cpp | `update_guided` | PRESENT (simplified) |
| LOITER | mode_loiter.cpp | `update_loiter` | PRESENT |
| RTL | mode_rtl.cpp | `update_rtl` | PRESENT (GAPS, see below) |
| CIRCLE | mode_circle.cpp | `update_circle` | PRESENT |
| LAND | mode_land.cpp | `update_land` | PRESENT (GAPS, see below) |
| DRIFT | mode_drift.cpp | `update_drift` | PRESENT |
| SPORT | mode_sport.cpp | `update_sport` | PRESENT |
| FLIP | mode_flip.cpp | `update_flip` | PRESENT |
| POSHOLD | mode_poshold.cpp | `update_poshold` | PRESENT |
| BRAKE | mode_brake.cpp | `update_brake` | PRESENT |
| THROW | mode_throw.cpp | `update_throw` | PRESENT |
| FOLLOW | mode_follow.cpp | `update_follow` | PRESENT |
| ZIGZAG | mode_zigzag.cpp | `update_zigzag` | PRESENT |
| GUIDED_NOGPS | mode_guided_nogps.cpp | `update_guided_nogps` | PRESENT |
| SMART_RTL | mode_smart_rtl.cpp | `update_smartrtl` | PRESENT (GAPS, see below) |
| AUTOTUNE | mode_autotune.cpp | — | **ABSENT** |
| AUTOROTATE | mode_autorotate.cpp | — | **ABSENT** (heli-specific) |
| STABILIZE_HELI | mode_stabilize_heli.cpp | — | **ABSENT** (heli-specific) |
| ACRO_HELI | mode_acro_heli.cpp | — | **ABSENT** (heli-specific) |
| SYSTEMID | mode_systemid.cpp | — | **ABSENT** |
| TURTLE | mode_turtle.cpp | — | **ABSENT** |
| FLOWHOLD | mode_flowhold.cpp | — | **ABSENT** |
| AVOID_ADSB | mode_avoid_adsb.cpp | — | **ABSENT** |
| GUIDED_CUSTOM | mode_guided_custom.cpp | — | **ABSENT** |

**GAP 12 (MEDIUM): AUTOTUNE mode absent.**
ArduPilot's AutoTune (mode 68) runs PID auto-calibration. Meridian has a `meridian-autotune` crate but no `AutoTune` flight mode entry in `FlightModeId` or `MultirotorModes`.

**GAP 13 (LOW): SYSTEMID mode absent.**
System identification for frequency response measurement. Primarily used by developers, rarely in production.

**GAP 14 (LOW): TURTLE mode absent.**
Motor-reversal to flip a crashed/inverted drone. Safety-relevant for field recovery.

**GAP 15 (LOW): FLOWHOLD mode absent.**
Optical flow-based position hold without GPS. Relevant for indoor/GPS-denied operation.

**GAP 16 (LOW): AVOID_ADSB mode absent.**
ADS-B avoidance mode. Required for commercial operations in controlled airspace.

**Note on heli modes (AUTOROTATE, STABILIZE_HELI, ACRO_HELI):** These are helicopter-specific and not applicable to Meridian's current scope.

---

#### RTL 6-State Machine — Status

ArduPilot's RTL has 6 sub-states:
```
STARTING → INITIAL_CLIMB → RETURN_HOME → LOITER_AT_HOME → FINAL_DESCENT → LAND
```

Meridian's RTL has 5 states:
```
Climb → Return → Loiter → Land → Complete
```

**GAP 17 (HIGH): FINAL_DESCENT state is merged into Land.**
ArduPilot has a distinct `FINAL_DESCENT` phase where the vehicle descends to `RTL_ALT_FINAL` altitude and hovers before the Land phase. This allows configuring a final hold altitude above ground (e.g., 2m) to confirm position before commitment. Meridian goes directly from `Loiter` to `Land` with no intermediate descent-to-hover.

**GAP 18 (HIGH): RTL cone-slope calculation is absent.**
ArduPilot computes: `target_alt = MIN(target_alt, MAX(rtl_return_dist * cone_slope, min_rtl_alt))`. This prevents the vehicle from climbing unnecessarily high when close to home. Meridian's RTL always climbs to `rtl_altitude` regardless of horizontal distance.

**GAP 19 (HIGH): RTL terrain/rally multi-path selection is absent.**
ArduPilot's `build_path()` selects the return target by:
1. Checking for the nearest rally point within radius
2. Selecting altitude type (RELATIVE / TERRAIN / RANGEFINDER) based on `RTL_ALT_TYPE`
3. Applying terrain data to convert altitude frames

Meridian RTL returns to `input.home` at `rtl_altitude`. No rally point selection, no terrain frame conversion.

**GAP 20 (MEDIUM): RTL yaw-to-armed-heading on loiter is absent.**
ArduPilot resets yaw to `initial_armed_bearing` during `LOITER_AT_HOME` before landing. Meridian has no armed-bearing concept.

---

#### Land Detector — Status

**GAP 21 (CRITICAL): Land detector 8-condition check is absent.**

ArduPilot's `update_land_detector()` checks all 8 conditions simultaneously before declaring landed:
1. `motor_at_lower_limit` — throttle output at minimum
2. `throttle_mix_at_min` — attitude controller throttle mix at minimum
3. `!large_angle_request` — no large roll/pitch commanded (< 15°)
4. `!large_angle_error` — attitude error < 30°
5. `accel_stationary` — filtered earth-frame acceleration ≤ 3 m/s²
6. `descent_rate_low` — vertical speed < threshold
7. `rangefinder_check` — rangefinder < 2m (if healthy)
8. `WoW_check` — weight-on-wheels sensor (if present)

All 8 must be true for `LAND_DETECTOR_TRIGGER_SEC` before `land_complete = true`.

Meridian's `update_land()` in `multirotor.rs` checks only altitude < 0.5m. There is no multi-condition check, no filtered-acceleration test, no throttle-mix test, no angle-error test. This means land detection can fire while still in motion (descent false-positives) or fail to fire when resting (if barometer drift causes altitude > 0.5m).

**GAP 22 (HIGH): Motor shutdown sequence on landing is absent.**
ArduPilot disarms via `arming.disarm(AP_Arming::Method::LANDED)` only when `land_complete && spool_state == GROUND_IDLE`. Meridian's land mode transitions immediately from altitude check to `ModeOutput::Idle` with no motor-spooldown state machine.

---

### 2.2 Fixed-Wing Modes

**Modes present in ArduPlane:**

| Mode | ArduPilot | Meridian | Status |
|---|---|---|---|
| MANUAL | ✓ | `update_manual` | PRESENT |
| STABILIZE | ✓ | `update_stabilize` | PRESENT |
| FLY_BY_WIRE_A | ✓ | `update_fbwa` | PRESENT |
| FLY_BY_WIRE_B | ✓ | `update_fbwb` | PRESENT |
| CRUISE | ✓ | `update_cruise` | PRESENT |
| AUTO | ✓ | `update_auto` | PRESENT |
| RTL | ✓ | `update_rtl` | PRESENT (simplified) |
| LOITER | ✓ | `update_loiter` | PRESENT |
| GUIDED | ✓ | `update_guided` | PRESENT |
| TAKEOFF | ✓ | `update_takeoff` | PRESENT |
| LAND | ✓ | `update_land` | PRESENT |
| CIRCLE | ✓ | `update_circle` | PRESENT |
| ACRO | ✓ | — | **ABSENT** |
| TRAINING | ✓ | — | **ABSENT** |
| INITIALISING | internal | — | Not applicable |
| THERMAL | ✓ | — | **ABSENT** |
| LOITER_ALT_QFE | ✓ | — | **ABSENT** |

**GAP 23 (MEDIUM): Fixed-wing ACRO mode absent.**
Rate-based control without leveling. Used for aerobatics.

**GAP 24 (LOW): TRAINING mode absent.**
Beginner mode with leveling assistance. Low priority.

**GAP 25 (LOW): THERMAL mode absent.**
Soaring thermal detection and circling. Relevant for glider operations.

**Speed scaling formula: PRESENT and CORRECT.**
Meridian's `speed_scaling()` implements: `airspeed / scaling_speed`, clamped to [0.5, 2.0]. ArduPilot's equivalent is `groundspeed_scalar = constrain(airspeed / scaling_speed, 0.5, 2.0)`. Formula is identical.

**GAP 26 (CRITICAL): QuadPlane transition logic is absent.**
ArduPilot has extensive QuadPlane code (Q_ASSIST, tilt-rotor, tail-sitter transitions). Meridian has `FlightModeId::VtolTransitionToFw` and `VtolTransitionToHover` in the enum but no implementation in `fixed_wing.rs` or any other module. The VTOL transition state machine — including Q_ASSIST activation at low airspeed, throttle management during transition, control surface scheduling — does not exist.

**GAP 27 (CRITICAL): Q_ASSIST is absent.**
QuadPlane Assist mode activates multirotor motors when airspeed drops below stall threshold. This is a flight-safety feature with no Meridian equivalent.

**TECS: PRESENT but simplified.**
Meridian's `Tecs::update()` provides a functional energy management model (throttle from speed error, pitch from altitude error). ArduPilot's AP_TECS is significantly more complex with separate specific energy error and specific energy rate error controllers, integrators on both channels, sink rate limits, and feedforward terms. The simplified TECS is adequate for basic flight but will not match ArduPilot performance at envelope boundaries.

---

### 2.3 Rover Modes

| Mode | ArduPilot Rover | Meridian | Status |
|---|---|---|---|
| MANUAL | ✓ | `update_manual` | PRESENT |
| ACRO | ✓ | `update_acro` | PRESENT |
| STEERING | ✓ | `update_steering` | PRESENT |
| HOLD | ✓ | `update_hold` | PRESENT |
| AUTO | ✓ | `update_auto` | PRESENT |
| RTL | ✓ | `update_rtl` | PRESENT |
| LOITER | ✓ | `update_loiter` | PRESENT |
| SMARTRTL | ✓ | `update_smartrtl` | PRESENT |
| GUIDED | ✓ | `update_guided` | PRESENT |
| SIMPLE | ✓ | — | **ABSENT** |
| DOCK | ✓ | — | **ABSENT** |

**GAP 28 (LOW): Rover SIMPLE mode absent.**
Heading-locked forward/strafe control. Used for beginners.

**GAP 29 (LOW): Rover DOCK mode absent.**
Precision docking to a beacon. Requires separate infrastructure.

---

### 2.4 Submarine Modes

| Mode | ArduSub | Meridian | Status |
|---|---|---|---|
| MANUAL | ✓ | `update_manual` | PRESENT |
| STABILIZE | ✓ | `update_stabilize` | PRESENT |
| ALT_HOLD (Depth Hold) | ✓ | `update_depth_hold` | PRESENT |
| ACRO | ✓ | — | **ABSENT** |
| AUTO | ✓ | — | **ABSENT** |
| GUIDED | ✓ | — | **ABSENT** |
| POSHOLD | ✓ | — | **ABSENT** |
| SURFTRAK | ✓ | — | **ABSENT** |

**GAP 30 (HIGH): Sub AUTO, GUIDED, POSHOLD, SURFTRAK absent.**
Meridian's `SubModes` only handles Manual, Stabilize, and Depth Hold. All autonomous submarine modes are missing.

**GAP 31 (MEDIUM): Sub ACRO mode absent.**

---

## 3. Mission System (`meridian-mission`)

### 3.1 MAV_CMD Coverage

**ArduPilot supports ~65 unique MAV_CMD IDs. Meridian's `ActionKind` covers ~35.**

**Present in Meridian:**
- NAV: Waypoint, Takeoff, Land, RTL, LoiterUnlim, LoiterTurns, LoiterTime, LoiterToAlt, SplineWaypoint, Delay, GuidedEnable, SetYaw, ConditionDistance
- DO: SetMode, SetSpeed, Jump, JumpTag, SetRelay, RepeatRelay, SetServo, RepeatServo, CameraShutter, CameraVideo, GripperRelease, MountControl, SetRoi, SetHome, DoLandStart, ChangeSpeed, ChangeAlt, Digicam, Parachute, EngineControl, SetReverse, FenceEnable, AuxFunction

**GAP 32 (HIGH): MAV_CMD IDs absent from Meridian:**
- `MAV_CMD_NAV_ARC_WAYPOINT` — arc path between waypoints (fixed-wing)
- `MAV_CMD_NAV_ALTITUDE_WAIT` — wait at current position until altitude change
- `MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT` — continue nav while changing altitude
- `MAV_CMD_NAV_SET_YAW_SPEED` — set yaw and speed simultaneously
- `MAV_CMD_NAV_PAYLOAD_PLACE` — precision payload placement
- `MAV_CMD_NAV_SCRIPT_TIME` — Lua scripting time-based nav
- `MAV_CMD_NAV_ATTITUDE_TIME` — hold attitude for time
- `MAV_CMD_NAV_GUIDED_ENABLE` — enable guided mode from mission (present in AP but not in Meridian's NAV list)
- `MAV_CMD_NAV_VTOL_TAKEOFF` / `MAV_CMD_NAV_VTOL_LAND` — QuadPlane specific
- `MAV_CMD_CONDITION_DELAY` — wait N seconds
- `MAV_CMD_CONDITION_YAW` — wait for yaw condition
- `MAV_CMD_DO_AUTOTUNE_ENABLE` — autotune from mission
- `MAV_CMD_DO_GUIDED_LIMITS` — set guided mode position/altitude limits
- `MAV_CMD_DO_INVERTED_FLIGHT` — fixed-wing inverted flight
- `MAV_CMD_DO_GO_AROUND` — fixed-wing go-around
- `MAV_CMD_DO_VTOL_TRANSITION` — QuadPlane transition
- `MAV_CMD_DO_WINCH` — winch control
- `MAV_CMD_DO_SPRAYER` — agricultural sprayer
- `MAV_CMD_DO_SEND_SCRIPT_MESSAGE` — Lua scripting
- `MAV_CMD_DO_SET_RESUME_REPEAT_DIST` — resume distance for repeat
- `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` — gimbal control
- `MAV_CMD_DO_PAUSE_CONTINUE` — in-mission pause/continue
- `MAV_CMD_DO_RETURN_PATH_START` — mark return path start
- `MAV_CMD_NAV_RALLY_POINT` — rally point definition in mission
- `MAV_CMD_NAV_FENCE_*` (5 variants) — geofence definition commands
- Camera capture commands: `IMAGE_START_CAPTURE`, `IMAGE_STOP_CAPTURE`
- Camera control: `SET_CAMERA_ZOOM`, `SET_CAMERA_FOCUS`, `SET_CAMERA_SOURCE`
- Video: `VIDEO_START_CAPTURE`, `VIDEO_STOP_CAPTURE`

**Total gap: ~30 MAV_CMD IDs (roughly half the ArduPilot set)**

---

### 3.2 Dual-Cursor Execution

**Status: PRESENT and structurally correct.**
`MissionExecutor` maintains separate `nav_index` and `do_index` cursors. `ActionKind::is_nav()` / `is_do()` correctly classifies commands. NAV cursor blocks; DO cursor advances independently.

**GAP 33 (MEDIUM): DO cursor does not advance past NAV commands.**
ArduPilot's DO cursor scans ahead of the NAV cursor and fires all DO commands between the current NAV index and the next NAV command. Meridian's DO cursor logic is not fully implemented in the `tick()` path — there is no evidence of a DO cursor advancing independently within the `MissionExecutor::tick()` method (the file ends at line ~1150 without showing a full `tick()` loop that advances the do cursor).

---

### 3.3 DO_JUMP

**Status: PRESENT.**
`ActionKind::Jump { target, repeat }` with `repeat = -1` for infinite. `jump_counters` Vec tracks per-target loop counts.

**GAP 34 (MEDIUM): DO_JUMP counter does not zero on loop completion.**
ArduPilot has a `DontZeroCounter` option (MISSION_OPTIONS bit 3). When NOT set, the jump counter is zeroed at loop end so re-entry to the loop resets it. Meridian has no MISSION_OPTIONS equivalent and does not zero counters.

---

### 3.4 Mission Upload/Download Protocol

**Status: PARTIAL.**
`MissionExecutor` has `set_item()` (upload) and `get_item()` (download) for single-item access. The MAVLink protocol for mission upload (MISSION_COUNT → MISSION_REQUEST_LIST → MISSION_ITEM_INT handshake) is handled in `meridian-mavlink`, not audited here, but the item storage API supports it.

---

### 3.5 Mission Pause/Resume with Rewind

**Status: PRESENT and correct.**
`pause()`, `resume()`, and `resume_with_rewind()` are implemented. The rewind mechanism correctly captures the paused NAV waypoint, checks drift distance (> 3m threshold), inserts a rewind flight target, and transitions back to normal execution on `rewind_complete()`. Tests exist for all three paths.

---

## 4. Failsafe (`meridian-failsafe`)

### 4.1 Failsafe Types — Coverage

| ArduPilot Failsafe | Meridian Monitor | Status |
|---|---|---|
| RC/Radio loss (`failsafe_radio_on_event`) | `TimeoutMonitor` (rc_loss) | PRESENT |
| GCS/Comms loss (`failsafe_gcs_check`) | `TimeoutMonitor` (comms_loss) | PRESENT |
| GPS loss | `TimeoutMonitor` (gps_loss) | PRESENT |
| Battery low | `BatteryMonitor` (low) | PRESENT |
| Battery critical | `BatteryMonitor` (critical) | PRESENT |
| EKF unhealthy | `EkfHealthMonitor` | PRESENT |
| Geofence breach | `GeofenceMonitor` | PRESENT |
| Motor failure | `MotorMonitor` | PRESENT |
| Crash / inverted | `CrashDetector` | PRESENT |
| Vibration / clipping | `VibrationMonitor` | PRESENT |
| Dead reckoning | `DeadReckoningMonitor` | PRESENT |
| Terrain data stale | `TerrainMonitor` | PRESENT |
| Watchdog / CPU stall | `WatchdogMonitor` | PRESENT |
| GPS glitch (EKF innov) | `GpsGlitchMonitor` | PRESENT |
| Thrust loss check | — | **ABSENT** |
| Yaw imbalance check | — | **ABSENT** |
| AFS (Advanced Failsafe) | — | **ABSENT** |
| Parachute check | — | **ABSENT** |

**GAP 35 (HIGH): Thrust loss check absent.**
ArduPilot's `thrust_loss_check()` (in `land_detector.cpp`) detects when the vehicle cannot maintain altitude despite full throttle (indicating a motor failure or propeller loss). This triggers an emergency land sequence. No equivalent in `meridian-failsafe`.

**GAP 36 (MEDIUM): Yaw imbalance check absent.**
ArduPilot's `yaw_imbalance_check()` detects when excessive yaw correction is required, indicating a failed/damaged motor. Triggers emergency action.

**GAP 37 (LOW): Advanced Failsafe (AFS) absent.**
ArduPilot's `AP_AdvancedFailsafe` provides geofence + termination logic for UAV operations with ground stations. Out of scope for most users.

---

### 4.2 FS_OPTIONS Continuation Flags

**Status: PRESENT but incomplete.**

ArduPilot's `FailsafeOption` bitmask has 7 relevant flags:
1. `CONTINUE_IF_LANDING` — don't interrupt landing for RC/GCS loss
2. `RC_CONTINUE_IF_AUTO` — don't RTL on RC loss if in Auto mode
3. `RC_CONTINUE_IF_GUIDED` — don't RTL on RC loss if in Guided mode
4. `GCS_CONTINUE_IF_AUTO` — don't RTL on GCS loss if in Auto mode
5. `GCS_CONTINUE_IF_PILOT_CONTROL` — don't RTL on GCS loss if in pilot mode
6. `RC_CONTINUE_IF_PILOT_CONTROLLED` — don't RTL on RC loss if in pilot mode
7. `BATTERY_CONTINUE_IF_NOT_LANDED` — battery failsafe doesn't interrupt unless landed

Meridian's `FailsafeOptions` has only 2 flags:
- `continue_auto_on_rc_loss` (= ArduPilot flag 2)
- `continue_auto_on_gcs_loss` (= ArduPilot flag 4)

**GAP 38 (HIGH): 5 of 7 FS_OPTIONS flags are absent:**
- `CONTINUE_IF_LANDING`
- `RC_CONTINUE_IF_GUIDED`
- `GCS_CONTINUE_IF_PILOT_CONTROL`
- `RC_CONTINUE_IF_PILOT_CONTROLLED`
- `BATTERY_CONTINUE_IF_NOT_LANDED`

---

### 4.3 Priority Escalation

**GAP 39 (HIGH): Priority escalation logic is absent.**

ArduPilot's failsafe system has a priority ordering where higher-severity failsafes override lower ones, and certain failsafes suppress others:
- Battery critical overrides battery low
- Any ongoing landing cannot be interrupted by RC loss if `CONTINUE_IF_LANDING` is set
- The system checks `get_highest_failsafe_priority()` before switching actions
- SmartRTL is preferred over RTL if SmartRTL is healthy; RTL is preferred over Land

Meridian's `FailsafeManager::check_all()` emits events independently with no inter-failsafe priority arbitration. When two failsafes trigger simultaneously, both are added to the `active` Vec without resolution. No priority ordering is defined or enforced.

---

### 4.4 Failsafe Action Completeness

**GAP 40 (HIGH): Multiple FailsafeAction variants absent.**

ArduPilot's `FailsafeAction` enum (in events.cpp) includes:
- `NONE` — do nothing / continue
- `RTL` — return to launch
- `LAND` — land immediately
- `SMARTRTL` — use SmartRTL first, fall back to RTL
- `SMARTRTL_LAND` — use SmartRTL first, fall back to land
- `BRAKE_LAND` — brake first, then land
- `AUTO_DO_LAND_START` — jump to DO_LAND_START in mission if present, else RTL
- Disarm immediately

Meridian's `FailsafeAction`:
- `Warn`
- `ReturnToLaunch`
- `SmartReturnToLaunch`
- `Land`
- `Terminate`

**Missing:** `SMARTRTL_LAND` (SmartRTL with land fallback, not RTL fallback), `BRAKE_LAND`, `AUTO_DO_LAND_START`. The distinction between SmartRTL→RTL and SmartRTL→Land is safety-meaningful.

---

### 4.5 Dead Reckoning Failsafe

**Status: PRESENT and correct.**
`DeadReckoningMonitor` correctly triggers after configurable timeout (default 20s) when `ekf_dead_reckoning` is true. GPS restoration clears the trigger.

---

## 5. Arming (`meridian-arming`)

### 5.1 Pre-Arm Check Coverage

| ArduPilot Check | Meridian | Status |
|---|---|---|
| Baro healthy | `BaroHealth` | PRESENT |
| Baro calibrated | (in Baro check) | PRESENT |
| Compass healthy | `CompassHealth` | PRESENT |
| Compass calibrated | (in Compass check) | PRESENT |
| GPS fix type ≥ 3D | `GpsLock` | PRESENT |
| GPS satellites ≥ 6 | (in GPS check) | PRESENT |
| GPS HDOP ≤ 2.0 | (in GPS check) | PRESENT |
| IMU (gyro+accel) healthy | `InsHealth` | PRESENT |
| Multi-IMU consistency | `imu_consistent` field | PRESENT (see below) |
| EKF healthy | `EkfHealth` | PRESENT |
| EKF velocity variance | (in EKF check) | PRESENT |
| EKF position variance | (in EKF check) | PRESENT |
| Battery voltage | `BatteryLevel` | PRESENT |
| Battery percentage | (in Battery check) | PRESENT |
| RC calibrated | `RcChannels` | PRESENT |
| RC channels valid ≥ 4 | (in RC check) | PRESENT |
| Board voltage ±0.5V from 5V | `BoardVoltage` | PRESENT |
| Safety switch | `SafetySwitch` | PRESENT |
| Lean angle < 25° | (lean check) | PRESENT |
| Logging available | `Logging` | PRESENT |
| Fence loaded | `FenceGeofence` | PRESENT |
| Motor interlock conflict | — | **ABSENT** |
| E-Stop / Emergency Stop conflict | — | **ABSENT** |
| Disarm switch check | — | **ABSENT** |
| Motor arming check | — | **ABSENT** |
| RC throttle failsafe check | — | **ABSENT** |
| OA (obstacle avoidance) check | — | **ABSENT** |
| Winch check | — | **ABSENT** |
| Airspeed sensor check | — | **ABSENT** |
| Alt check (valid altitude) | — | **ABSENT** |
| Autotune config check (heli) | — | N/A |
| Temperature cal running | — | **ABSENT** |
| Parameters check | `Parameters` (enum only) | NOT IMPLEMENTED |

**GAP 41 (MEDIUM): Motor interlock / E-Stop conflict check absent.**
ArduPilot's `run_pre_arm_checks()` verifies that MOTOR_INTERLOCK and MOTOR_ESTOP aux switches are not both configured simultaneously. Meridian's `ArmingState` has a `motor_interlock` field but no conflict check.

**GAP 42 (MEDIUM): RC throttle failsafe pre-arm check absent.**
ArduPilot checks that the throttle failsafe trigger value is valid relative to the current RC calibration before arming.

**GAP 43 (LOW): OA, winch, airspeed, alt checks absent.**
These are system-specific checks that apply to certain configurations.

---

### 5.2 Multi-IMU Consistency Check

**Status: PRESENT in data model, NOT IMPLEMENTED in check logic.**

ArduPilot's consistency check (`AP_Arming::ins_accels_consistent()`):
- Accel threshold: **0.75 m/s²** (from `AP_ARMING_ACCEL_ERROR_THRESHOLD`)
- Gyro threshold: **5 deg/s**
- Must pass consistently for **10 seconds** before arming

Meridian's `ArmingState` has `imu_consistent: bool` which the caller must populate. The `check_prearm()` function uses it:
```rust
if state.imu_count > 1 && !state.imu_consistent {
    add_check(&mut result, "IMU", false, "IMUs inconsistent");
}
```

**GAP 44 (HIGH): The actual consistency computation (0.75 m/s², 5 deg/s, 10s) is not implemented in meridian-arming.**
The caller must pre-compute `imu_consistent`. There is no `ins_accels_consistent()` or `ins_gyros_consistent()` function in Meridian that performs the three-threshold check with the 10-second sustain timer. If the caller passes a naive bool (e.g., always true or based on a different threshold), the check is meaningless. The reference thresholds from ArduPilot must be implemented in Meridian and applied before calling `check_prearm()`.

---

## Summary of All Gaps

### Critical (safety-affecting, must fix before flight)

| # | Gap | Location |
|---|---|---|
| 1 | L1 lateral accel coefficient uses 2.0 instead of 4*damping² (≈2.25 at default) | `l1.rs` |
| 8 | SmartRTL loop pruning absent — only Douglas-Peucker, no loop detection | `smartrtl.rs` |
| 21 | Land detector 8-condition check absent — altitude-only detection is insufficient | `multirotor.rs` |
| 22 | Motor shutdown sequence on landing absent | `multirotor.rs` |
| 26 | QuadPlane transition logic absent (VTOL modes in enum but not implemented) | `fixed_wing.rs` |
| 27 | Q_ASSIST absent — no motor assist at low airspeed for QuadPlane | `fixed_wing.rs` |
| 30 | Sub AUTO, GUIDED, POSHOLD, SURFTRAK modes absent | `submarine.rs` |
| 35 | Thrust loss check absent | `meridian-failsafe` |
| 39 | Failsafe priority escalation logic absent | `meridian-failsafe` |
| 44 | IMU consistency thresholds (0.75 m/s², 5 deg/s, 10s) not implemented | `meridian-arming` |

### High (significant behavioral gaps)

| # | Gap | Location |
|---|---|---|
| 2 | prevent_indecision Nu latch absent from L1 | `l1.rs` |
| 3 | Crosstrack integral integrates unconditionally (should only when |Nu1| < 5°) | `l1.rs` |
| 4 | L1 dist_min parameter not supported | `l1.rs` |
| 9 | SmartRTL breadcrumb buffer too small (50 vs 300 default) | `smartrtl.rs` |
| 17 | RTL FINAL_DESCENT state absent | `multirotor.rs` |
| 18 | RTL cone-slope altitude reduction absent | `multirotor.rs` |
| 19 | RTL terrain/rally selection absent | `multirotor.rs` |
| 32 | ~30 MAV_CMD IDs absent from mission system | `meridian-mission` |
| 36 | Yaw imbalance check absent | `meridian-failsafe` |
| 38 | 5 of 7 FS_OPTIONS continuation flags absent | `meridian-failsafe` |
| 40 | SMARTRTL_LAND, BRAKE_LAND, AUTO_DO_LAND_START actions absent | `meridian-types` |

### Medium (behavioral differences, lower safety risk)

| # | Gap | Location |
|---|---|---|
| 5 | Reverse mode yaw wrapping absent (L1) | `l1.rs` |
| 6 | Crosstrack integral units differ (distance×time vs angle) | `l1.rs` |
| 7 | Spline uses Catmull-Rom vs ArduPilot's Hermite tangent blending | `scurve.rs` |
| 10 | SmartRTL 15s timeout disablement absent | `smartrtl.rs` |
| 11 | SmartRTL automatic background cleanup trigger absent | `smartrtl.rs` |
| 12 | AUTOTUNE mode absent | `meridian-modes` |
| 20 | RTL yaw-to-armed-heading on loiter absent | `multirotor.rs` |
| 23 | Fixed-wing ACRO mode absent | `fixed_wing.rs` |
| 28 | Rover SIMPLE mode absent | `rover.rs` |
| 29 | Rover DOCK mode absent | `rover.rs` |
| 31 | Sub ACRO mode absent | `submarine.rs` |
| 33 | DO cursor advancement in mission tick not fully visible | `meridian-mission` |
| 34 | DO_JUMP counter zero-on-completion option absent | `meridian-mission` |
| 41 | Motor interlock/E-Stop conflict pre-arm check absent | `meridian-arming` |
| 42 | RC throttle failsafe pre-arm check absent | `meridian-arming` |

### Low (minor or configuration-only gaps)

| # | Gap | Location |
|---|---|---|
| 13 | SYSTEMID mode absent | `meridian-modes` |
| 14 | TURTLE mode absent | `meridian-modes` |
| 15 | FLOWHOLD mode absent | `meridian-modes` |
| 16 | AVOID_ADSB mode absent | `meridian-modes` |
| 24 | Fixed-wing TRAINING mode absent | `fixed_wing.rs` |
| 25 | Fixed-wing THERMAL mode absent | `fixed_wing.rs` |
| 37 | Advanced Failsafe (AFS) absent | `meridian-failsafe` |
| 43 | OA/winch/airspeed/alt pre-arm checks absent | `meridian-arming` |

---

## What Is Correct

The following items are correctly implemented and match ArduPilot behavior:

- L1 distance formula (`damping * period * V / π`)
- XTRACK_I gain constant (0.02)
- XTRACK_I windup limit present (different units, see Gap 6)
- S-curve 7-phase trajectory planner (phases, kinematics, duration computation)
- Catmull-Rom spline present (endpoint interpolation correct)
- Terrain grid dimensions (32×28) and bilinear interpolation
- MAVLink TERRAIN_REQUEST/DATA protocol
- 12-block LRU terrain cache
- Dual-cursor NAV/DO mission execution concept
- DO_JUMP with loop counter
- Mission pause/resume with rewind (including 3m drift threshold)
- All 14 failsafe types present (RC, GPS, GCS, battery×2, EKF, geofence, motor, crash, vibration, dead reckoning, terrain, watchdog, GPS glitch)
- Dead reckoning failsafe (timeout, GPS restoration clears)
- FS_OPTIONS partial flags (2 of 7)
- All standard arming checks (baro, compass, GPS, IMU, EKF, battery, RC, board voltage, safety switch, lean angle, logging, fence)
- IMU consistency field present in ArmingState (thresholds not computed internally — see Gap 44)
- Speed scaling formula for fixed-wing identical
- Multirotor RTL present with Climb→Return→Loiter→Land states
- SmartRTL Douglas-Peucker simplification correct
- Rally point nearest-best selection present
- All 19 present copter modes have correct mode-entry and basic update logic
- Multirotor mode dispatch (`update()` match arm) is exhaustive for all declared modes
