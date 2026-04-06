# Wave 2 Audit: ArduPilot Vehicle Main Code

**Audited:** ArduCopter, ArduPlane, Rover, ArduSub, Blimp  
**Source files read:** 30+  
**Scope:** Flight mode logic, scheduler structure, arming, failsafe, motor/actuator output

---

## 1. ArduCopter

### 1.1 Main Loop Structure

File: `ArduCopter/Copter.cpp`

ArduCopter uses the AP_Scheduler cooperative multitasking framework. Tasks are divided into two classes:

**FAST_TASK** — runs on every loop iteration (no rate limiting, best-effort):
1. `AP_InertialSensor::update` — reads latest IMU data (gyro + accel)
2. `run_rate_controller_main` — runs the low-level rate PID (roll/pitch/yaw rate from IMU only, not AHRS)
3. `motors_output_main` — pushes computed PWM to ESCs
4. `read_AHRS` — runs EKF state estimator (expensive, but must be fast)
5. `read_inertia` — updates inertial nav position estimate
6. `check_ekf_reset` — handles EKF yaw/position reset events
7. `update_flight_mode` — dispatches to current mode's `run()` function (the attitude + position controllers)
8. `update_home_from_EKF` — adjusts home position if EKF origin drifts
9. `update_land_and_crash_detectors` — runs land detector, crash detector, thrust-loss check
10. `update_rangefinder_terrain_offset` — surface tracking update
11. `camera_mount::update_fast` (if mount enabled)

**SCHED_TASK** — rate-limited background tasks (selected):
- `rc_loop` at 250 Hz — reads RC inputs
- `throttle_loop` at 50 Hz — handles auto-arm, throttle zero detection
- `fence_check` at 25 Hz
- `AP_GPS::update` at 50 Hz
- `AP_OpticalFlow::update` at 200 Hz
- `update_batt_compass` at 10 Hz
- `auto_disarm_check` at 10 Hz
- `read_rangefinder` at 20 Hz
- `AP_Proximity::update` at 200 Hz
- `update_altitude` at 10 Hz
- `run_nav_updates` at 50 Hz
- `update_throttle_hover` at 100 Hz
- `ModeSmartRTL::save_position` at 3 Hz
- `three_hz_loop` at 3 Hz
- `one_hz_loop` at 1 Hz
- `ekf_check` at 10 Hz
- `gpsglitch_check` at 10 Hz
- `GCS::update_receive` / `update_send` at 400 Hz

**Full call chain from loop tick to motor output (FAST_TASK path):**
```
scheduler tick
  → AP_InertialSensor::update()           # fresh IMU sample
  → run_rate_controller_main()            # inner rate loop (gyro-only, no EKF needed)
  → motors_output_main()                  # commit PWM to ESCs
  → read_AHRS()                           # EKF update (uses IMU + GPS + baro + compass)
  → read_inertia()                        # inertial nav position
  → check_ekf_reset()
  → update_flight_mode()                  # mode.run() → attitude + position controllers
      → attitude_control->input_*()       # AC_AttitudeControl: sets rate targets
      → pos_control->D_update_controller() / NE_update_controller()
  → update_land_and_crash_detectors()
```

The rate controller runs *before* the EKF and *before* the flight mode update. This is intentional: the rate loop only needs gyro data and can run at full loop rate (400 Hz) without waiting for the EKF, providing minimum latency for vibration rejection. The attitude+position controllers (flight mode) run after the EKF and feed targets to the rate controllers that will be consumed on the next tick.

**Loop rate:** Configurable via LOOP_RATE, typically 400 Hz for multirotors.

---

### 1.2 Flight Modes

All mode files: `ArduCopter/mode_*.cpp`

| Mode | File | Controllers Used | GPS Required | Notes |
|------|------|-----------------|--------------|-------|
| STABILIZE | mode_stabilize.cpp | attitude (angle) | No | Manual throttle, angle targets from sticks |
| ACRO | mode_acro.cpp | rate | No | Rate targets from sticks, no self-leveling |
| ACRO (Heli) | mode_acro_heli.cpp | rate | No | Helicopter-specific acro |
| ALT_HOLD | mode_althold.cpp | attitude + pos_control (D axis) | No | Baro altitude hold |
| LOITER | mode_loiter.cpp | attitude + pos_control (NE+D) + loiter_nav | Yes | Position + altitude hold |
| POS_HOLD | mode_poshold.cpp | attitude + pos_control + loiter_nav | Yes | Smoother than Loiter, brake on stick release |
| AUTO | mode_auto.cpp | wp_nav + pos_control + attitude | Yes | Mission execution |
| RTL | mode_rtl.cpp | wp_nav + pos_control + attitude | Yes | Return to launch state machine |
| LAND | mode_land.cpp | pos_control + attitude | Optional | Land detector, auto-disarm |
| GUIDED | mode_guided.cpp | wp_nav / pos_control + attitude | Yes | External position/velocity/attitude targets |
| GUIDED_NOGPS | mode_guided_nogps.cpp | attitude | No | Rate/angle targets without GPS |
| CIRCLE | mode_circle.cpp | circle_nav + pos_control + attitude | Yes | Orbit a point |
| DRIFT | mode_drift.cpp | attitude | Yes | Airplane-like roll+yaw coupled |
| SPORT | mode_sport.cpp | rate + pos_control (D) | No | Rate with alt hold |
| FLIP | mode_flip.cpp | attitude | No | Programmed flip maneuver |
| AUTO_TUNE | mode_autotune.cpp | attitude | No | PID autotuning |
| BRAKE | mode_brake.cpp | pos_control + attitude | Yes | Emergency stop |
| THROW | mode_throw.cpp | attitude | No | Hand-throw launch detection |
| AVOID_ADSB | mode_avoid_adsb.cpp | guided-like | Yes | ADS-B avoidance |
| SMART_RTL | mode_smart_rtl.cpp | wp_nav + pos_control | Yes | RTL along recorded path |
| FLOWHOLD | mode_flowhold.cpp | pos_control + attitude | Optical flow | Position hold via optical flow |
| FOLLOW | mode_follow.cpp | pos_control + attitude | Yes | Follow a beacon/vehicle |
| ZIGZAG | mode_zigzag.cpp | wp_nav + pos_control | Yes | Agricultural pattern |
| SYSTEM_ID | mode_systemid.cpp | attitude | No | System identification |
| AUTOROTATE | mode_autorotate.cpp | attitude | No | Heli autorotation |
| TURTLE | mode_turtle.cpp | (direct motor) | No | Inverted recovery |
| LOITER_TO_ALT (sub-mode in AUTO) | mode_auto.cpp | wp_nav | Yes | Climb to alt before proceeding |

---

### 1.3 mode_stabilize.cpp — Stick-to-Motor Path

File: `ArduCopter/mode_stabilize.cpp`

```
Pilot sticks → RC inputs (250 Hz rc_loop)
  → update_simple_mode()                 # optional: remap roll/pitch to be relative to pilot heading
  → get_pilot_desired_lean_angles_rad()  # roll/pitch sticks → target Euler angles (radians)
  → get_pilot_desired_yaw_rate_rads()    # yaw stick → target yaw rate (rad/s)
  → get_pilot_desired_throttle()         # throttle stick → 0..1
  → spool state check:
      SHUT_DOWN / GROUND_IDLE:
        attitude_control->reset_yaw_target_and_rate()
        attitude_control->reset_rate_controller_I_terms()
        throttle = 0
      THROTTLE_UNLIMITED:
        clear land_complete flag if throttle not limited
  → attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(roll, pitch, yaw_rate)
      # inside AC_AttitudeControl:
      #   converts Euler targets to quaternion
      #   computes angular error → rate targets via angle P gains
      #   feeds rate targets to rate controllers (PID)
      #   rate controller output → motor mix
  → attitude_control->set_throttle_out(pilot_throttle, true, g.throttle_filt)
      # feeds throttle to AP_Motors mixer
      # AP_Motors maps throttle + roll/pitch/yaw outputs to individual motor PWM
```

Key behaviors:
- No altitude stabilization. Pilot controls throttle directly.
- At zero throttle: I-terms reset smoothly, yaw target resets. Motors spool to GROUND_IDLE.
- `update_simple_mode()`: if SIMPLE mode is active, roll/pitch inputs are rotated to be relative to a reference heading (saved at arming or on stick command), so pilot always pushes "forward" relative to their perspective.

---

### 1.4 mode_althold.cpp — Throttle Stick to Altitude PID Chain

File: `ArduCopter/mode_althold.cpp`

```
throttle stick → get_pilot_desired_climb_rate_ms()
  # maps stick from center: deadband → 0 m/s
  # above center → positive climb rate (0 to PILOT_SPEED_UP m/s)
  # below center → negative climb rate (0 to PILOT_SPEED_DN m/s)
  → constrained to [-speed_dn, speed_up]

State machine (get_alt_hold_state_D_ms):
  MotorStopped:     reset I-terms, relax pos_control (throttle decays to 0)
  Landed_Ground_Idle / Landed_Pre_Takeoff:
                    reset I-terms smoothly, relax pos_control
  Takeoff:          takeoff.start_m(pilot_takeoff_alt_m) → climb to takeoff alt
                    avoidance-adjusted climb rate feeds takeoff controller
  Flying:           avoidance.adjust_roll_pitch_rad() (if AP_AVOIDANCE enabled)
                    surface_tracking.update_surface_offset() (rangefinder)
                    pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate)
                      # integrates climb rate → altitude target
                      # altitude target → position error → velocity error → throttle via PID

→ attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(roll, pitch, yaw_rate)
→ pos_control->D_update_controller()
    # PID: altitude error → vertical velocity target
    # PID: velocity error → throttle output
    # feeds throttle to motors via attitude_control
```

Init: checks if D-axis position controller is already active (smooth handoff from other modes). Sets max speed/accel from params PILOT_SPEED_UP, PILOT_SPEED_DN, PILOT_ACCEL_Z.

---

### 1.5 mode_loiter.cpp — Position Hold + Pilot Velocity Input

File: `ArduCopter/mode_loiter.cpp`

```
init():
  loiter_nav->init_target()        # capture current NE position as hold point
  pos_control->D_init_controller() # init altitude hold at current altitude
  set speed/accel limits

run() each tick:
  pilot lean angles → loiter_nav->set_pilot_desired_acceleration_rad(roll, pitch)
    # pilot lean angle → NE acceleration demand → loiter nav adds to position error
  pilot yaw → target_yaw_rate
  pilot throttle → target_climb_rate

  State machine (same as AltHold):
    MotorStopped: reset, loiter_nav->init_target() (don't drift while landed)
    Landed: reset I-terms, init target
    Takeoff: takeoff controller, loiter_nav->update()
    Flying:
      if precision_loiter active (AP_PRECLAND_ENABLED):
        precision_loiter_xy() → pos_control->input_pos_vel_accel_NE_m(precland_target)
      else:
        loiter_nav->update()
          # AC_Loiter:
          #   applies pilot accel demand to loiter target position
          #   runs position → velocity → acceleration PID
          #   outputs thrust_vector (NE accelerations)
      avoidance-adjusted climb rate
      surface tracking offset
      pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate)

→ attitude_control->input_thrust_vector_rate_heading_rads(
      loiter_nav->get_thrust_vector(), yaw_rate, false)
    # thrust vector (NE) becomes roll/pitch targets via tilt
→ pos_control->D_update_controller()  # vertical
```

Key difference from AltHold: horizontal position is controlled. The `loiter_nav` (AC_Loiter) converts pilot lean inputs into NE position/velocity targets and computes the thrust vector needed to hold position while allowing pilot to nudge it.

Precision loiter: if `AP_PRECLAND` has acquired a landing target, the loiter_xy controller is replaced with one that tracks the precland target position/velocity.

---

### 1.6 mode_auto.cpp — Mission Execution

File: `ArduCopter/mode_auto.cpp`

Auto mode dispatches to sub-modes based on the current mission command:

```
init():
  Check: mission must be present
  Check: if landed+armed, first command must be a takeoff
  _mode = SubMode::LOITER
  waiting_to_start = true (don't start until EKF has origin)

run() each tick:
  if waiting_to_start:
    check ahrs.get_origin() → when available: mission.start_or_resume()
  else:
    check for mission changes → restart WP command if changed
    mission.update()  # AP_Mission: advances through commands, calls nav_cmd handlers

  Dispatch to sub-mode runner:
    TAKEOFF      → takeoff_run()
    WP           → wp_run()           # wp_nav tracks waypoint, D controller for alt
    LAND         → land_run()         # delegates to ModeLand logic
    RTL          → rtl_run()          # delegates to ModeRTL logic
    CIRCLE       → circle_run()
    NAVGUIDED    → nav_guided_run()   # GUIDED sub-command from mission
    LOITER       → loiter_run()
    LOITER_TO_ALT→ loiter_to_alt_run()
    NAV_PAYLOAD_PLACE → payload_place.run()
    NAV_ATTITUDE_TIME → nav_attitude_time_run()
```

Waypoint switching: `AP_Mission::update()` calls registered `nav_cmd_fn` callbacks for each nav command. When a nav command completes (verified by `do_nav_wp()` checking if wp_nav has reached destination), `mission.advance_current_cmd()` is called which moves to the next command.

Auto RTL: when mission hits a landing sequence or completes, the auto_RTL flag is set and the mode pretends to be in RTL to GCS displays.

---

### 1.7 mode_rtl.cpp — RTL State Machine

File: `ArduCopter/mode_rtl.cpp`

RTL State machine: `SubMode` enum with 6 states:

```
STARTING → INITIAL_CLIMB → RETURN_HOME → LOITER_AT_HOME → FINAL_DESCENT → LAND
                                                         ↘ (if alt_final > 0 and no radio FS)
                                                           FINAL_DESCENT
```

**build_path()** — computes the full RTL path:
- `origin_point`: current stopping point (inertial nav)
- `return_target`: nearest rally point or home (can be above-terrain or absolute)
- `climb_target`: same lat/lng as origin, altitude = return_target altitude
- `descent_target`: return_target lat/lng, altitude = RTL_ALT_FINAL_M
- `land`: true if RTL_ALT_FINAL_M <= 0 (will land at home)
- Terrain following: selects between rangefinder and terrain database for alt type
- Cone slope: reduces climb if close to home (no need to climb full RTL_ALT if nearly home)
- Fence constraint: caps return_target alt below fence max alt

**State runners:**
- `INITIAL_CLIMB` + `RETURN_HOME`: both run `climb_return_run()`
  - `wp_nav->update_wpnav()`: waypoint navigator tracks climb_target then return_target
  - `pos_control->D_update_controller()`: vertical
  - `attitude_control->input_thrust_vector_heading()`: auto-yaw (hold during climb, toward home during return)
  - `_state_complete = wp_nav->reached_wp_destination()`
- `LOITER_AT_HOME`: `loiterathome_run()`
  - Same wp_nav loop but at the return_target position
  - Waits `g.rtl_loiter_time` milliseconds
  - Resets yaw to initial armed heading (unless pilot overrode yaw)
  - `_state_complete` when time elapsed AND yaw within 2 degrees of armed heading
- `FINAL_DESCENT`: `descent_run()`
  - Horizontal: pilot can reposition via sticks (if LAND_REPOSITIONING enabled), else holds NE position
  - Vertical: `pos_control->D_set_alt_target_with_slew_m(descent_target.alt)` → slews to RTL_ALT_FINAL_M
  - `_state_complete` when within 0.2m of final altitude
  - High throttle cancel: if THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND set and throttle high, switches to LOITER or ALT_HOLD
- `LAND`: `land_run()`
  - Delegates to `land_run_normal_or_precland()`
  - `_state_complete = copter.ap.land_complete`
  - Disarms when land_complete AND motors in GROUND_IDLE

Restart without terrain: if terrain data is unavailable during navigation, RTL restarts from STARTING with `terrain_following_allowed = false` and uses relative altitude instead.

---

### 1.8 mode_land.cpp — Land Detection and Motor Shutdown

File: `ArduCopter/mode_land.cpp`

```
init():
  control_position = copter.position_ok()  # use GPS if available
  NE controller init (if GPS)
  D controller init
  land_pause = true (4-second delay before descent)
  auto_yaw.set_mode(HOLD)
  precland_statemachine.init()

run():
  if control_position: gps_run()
  else: nogps_run()

gps_run():
  if land_complete AND motors GROUND_IDLE: arming.disarm(LANDED)
  if disarmed_or_landed: make_safe_ground_handling()
  else:
    wait out land_pause delay (LAND_WITH_DELAY_MS = 4s)
    land_run_normal_or_precland(land_pause)
      → if precland target acquired: precision_land_run()
      → else: land_run_vertical_control()

land_run_vertical_control():
  Two-stage descent:
    Above LAND_ALT_LOW_M: descend at land_speed_high_ms (or WP_SPD_DN if 0)
    Below LAND_ALT_LOW_M: descend at land_speed_ms (LAND_SPD_MS_DEFAULT = 0.5 m/s)
  pos_control->D_set_pos_target_from_climb_rate_ms(descent_rate)
  pos_control->D_update_controller()

nogps_run():
  No horizontal position control
  Pilot can roll/pitch to steer landing (if LAND_REPOSITIONING)
  High throttle cancels land (switches to ALT_HOLD)
  Same two-stage vertical descent
```

**Land detector** (`land_detector.cpp`):

The land detector is deliberately conservative — it avoids sensors that are unreliable close to ground (baro, EKF alt, rangefinder at very short range).

Conditions that must ALL be true for `land_complete`:
1. `motors->limit.throttle_lower` — motor output is at lower limit (motor at min)
2. `attitude_control->is_throttle_mix_min()` — throttle mix prioritizing landing
3. `!large_angle_request` — attitude target roll/pitch < 15 deg
4. `!large_angle_error` — actual attitude error < 30 deg
5. `accel_stationary` — filtered earth-frame acceleration < LAND_DETECTOR_ACCEL_MAX (minus gravity)
6. `descent_rate_low` — vertical speed < LAND_DETECTOR_VEL_Z_MAX (1 m/s)
7. `rangefinder_check` — rangefinder height < LAND_RANGEFINDER_MIN_ALT_M (2m) OR rangefinder not healthy
8. `WoW_check` — weight-on-wheels sensor shows WoW or unknown

All conditions must hold for `LAND_DETECTOR_TRIGGER_SEC` (default ~1 second) × loop rate ticks.

Motor shutdown sequence:
- `land_complete` set → `arming.disarm(LANDED)` called from land/RTL modes
- `set_land_complete()` optionally auto-disarms if `THR_BEHAVE_DISARM_ON_LAND_DETECT` set
- Disarm → `motors->armed(false)` → PWM output stops (or goes to disarmed PWM)

---

### 1.9 AP_Arming_Copter — Pre-Arm Checks

File: `ArduCopter/AP_Arming_Copter.cpp`

`run_pre_arm_checks()` gates:

**Always run (mandatory even if ARMING_CHECK disabled):**
- System initialized
- Motor interlock / E-Stop conflict check (can't have both interlock AND estop on RC)
- Motor interlock check: if using_interlock AND motor_interlock_switch is HIGH → fail
- Disarm switch check
- `motors->arming_checks()` — frame-specific motor checks

**If `should_skip_all_checks()` false (ARMING_CHECK not 0):**
- `parameter_checks()`:
  - FS_THR_VALUE sanity (must be > radio_min + 10, and > 910 PWM)
  - ACRO_BAL_ROLL/PITCH <= attitude controller angle P gains
  - PILOT_SPEED_UP > 0
  - Heli: motor interlock RC channel must be configured
  - Frame class must be valid for non-heli (no HELI_QUAD/DUAL/HELI in multicopter build)
  - RTL_ALT_TYPE terrain checks (rangefinder healthy, RTL_ALT below RNGFND_MAX)
  - ADSB threat check
  - `pos_control->pre_arm_checks()` — PSC parameter validation
  - `attitude_control->pre_arm_checks()` — ATC parameter validation
- `oa_checks()` — object avoidance pre-arm
- `gcs_failsafe_check()` — GCS failsafe config valid
- `winch_checks()`
- `rc_throttle_failsafe_checks()`:
  - RC receiver present
  - Throttle above failsafe_throttle_value
- `alt_checks()`
- `AP_Arming::airspeed_checks()` (if airspeed sensor enabled)
- `AP_Arming::pre_arm_checks()` — base class checks:
  - Barometer: altitude disparity between baro and EKF < PREARM_MAX_ALT_DISPARITY_M (1m)
  - INS: `AP_Arming::ins_checks()` + EKF attitude check (gyro bias acceptable)
  - Board voltage: battery not failed
  - GPS: HDOP < GPS_HDOP_GOOD, sufficient satellites, EKF healthy
  - Compass: calibrated, consistent
  - RC calibration: roll/pitch/throttle/yaw channels calibrated and centered

**Overridable checks (ARMING_CHECK bitmask):**
Each check has a corresponding `Check::` enum value. Setting `ARMING_CHECK` to 0 skips all optional checks. Individual bits can be cleared to skip specific checks (e.g., skip GPS check for indoor flying, skip compass check for GPS-denied ops).

---

### 1.10 events.cpp — Failsafe Logic

File: `ArduCopter/events.cpp`

**Radio failsafe** (`failsafe_radio_on_event`):

Configurable via `FS_THR_ENABLE`:
- `FS_THR_DISABLED`: no action
- `FS_THR_ENABLED_ALWAYS_RTL` / `FS_THR_ENABLED_CONTINUE_MISSION`: RTL
- `FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL`: SmartRTL → RTL fallback
- `FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND`: SmartRTL → Land fallback
- `FS_THR_ENABLED_ALWAYS_LAND`: Land
- `FS_THR_ENABLED_AUTO_RTL_OR_RTL`: AUTO DO_LAND_START → RTL fallback
- `FS_THR_ENABLED_BRAKE_OR_LAND`: Brake → Land fallback

Override conditions (higher priority than FS_THR_ENABLE setting):
- If on ground (should_disarm_on_failsafe): disarm immediately
  - STABILIZE/ACRO: disarm if throttle zero OR land_complete
  - AUTO: disarm if not auto-armed AND land_complete
  - All others (AltHold, Guided, Loiter, etc.): disarm if land_complete
- If already landing AND battery FS requires landing: continue landing
- If already landing AND FS_OPTIONS CONTINUE_IF_LANDING set: continue landing
- If in AUTO AND FS_OPTIONS RC_CONTINUE_IF_AUTO set: no action (continue mission)
- If in GUIDED AND FS_OPTIONS RC_CONTINUE_IF_GUIDED set: no action

**GCS failsafe** (`failsafe_gcs_check` + `failsafe_gcs_on_event`):

Triggered when `millis() - gcs_last_seen_ms > fs_gcs_timeout * 1000`.

Same action options as radio failsafe (configured via `FS_GCS_ENABLE`).

Override conditions similar to radio — landing, mission, guided all have options to continue.
Clears RC overrides when GCS failsafe triggers.

**Battery failsafe** (`handle_battery_failsafe`):

Actions from `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` (per-battery configurable):
- NONE, LAND, RTL, SmartRTL, SmartRTL_Land, TERMINATE, AUTO_DO_LAND_START, BRAKE_LAND

**Terrain failsafe:**
- Triggers if terrain data fails for > FS_TERRAIN_TIMEOUT_MS in Auto/RTL/etc.
- Action: restart RTL without terrain, or switch to Land

**Dead reckoning failsafe:**
- Triggers when EKF enters dead reckoning mode (no GPS) for > `FS_DR_TIMEOUT` seconds
- Only in modes requiring position estimate

**GPS glitch check:**
- Detected via `AP_AHRS::Status::GPS_GLITCHING`
- Logs and sends GCS text but no automatic mode change

**`do_failsafe_action()`** — central dispatcher:
```
NONE           → return
LAND           → set_mode_land_with_pause()
RTL            → set_mode_RTL_or_land_with_pause()  # falls back to LAND if RTL fails
SMARTRTL       → set_mode_SmartRTL_or_RTL()
SMARTRTL_LAND  → set_mode_SmartRTL_or_land_with_pause()
TERMINATE      → afs.gcs_terminate() or disarm()
AUTO_DO_LAND_START → set_mode_auto_do_land_start_or_RTL()
BRAKE_LAND     → set_mode_brake_or_land_with_pause()
```

If `FS_OPTIONS::RELEASE_GRIPPER` set, gripper releases on any failsafe.

---

### 1.11 Motor Interlock, Motor Test, Emergency Stop

**Motor interlock** (`ArduCopter/AP_Arming_Copter.cpp` + `RC_Channel_Copter`):

- An RC aux channel can be assigned to `MOTOR_INTERLOCK` function
- When `ap.using_interlock == true` and `ap.motor_interlock_switch == true` (switch HIGH): motors will NOT spin even if armed
- Used for helicopters primarily (allows arming while rotor is not spinning)
- Cannot use MOTOR_INTERLOCK and MOTOR_ESTOP simultaneously (pre-arm check fails)
- `motors->set_interlock(bool)` — enable/disable motor output

**Emergency stop** (`SRV_Channels::get_emergency_stop()`):

- RC aux channel assigned to `MOTOR_ESTOP` or `ARM_EMERGENCY_STOP`
- When active: immediately stops all motor output
- Cannot combine with MOTOR_INTERLOCK aux function
- Motor test checks this and refuses to start if E-Stop is active

**Motor test** (`ArduCopter/motor_test.cpp`):

- Triggered via MAVLink `MAV_CMD_DO_MOTOR_TEST`
- Pre-conditions: board initialized, landed, safety switch not SAFETY_DISARMED, E-Stop not active, RC calibrated (unless direct PWM)
- On start: `motors->output_min()`, `motors->armed(true)`, disables throttle/GCS/EKF failsafes
- During test: outputs specified PWM to specified motor sequence via `motors->output_test_seq()`
- Three throttle types: PERCENT (maps to PWM range), PWM (direct), PILOT (passthrough from throttle channel)
- Timeout: up to 600 seconds; multi-motor: alternates 50% on / 50% off between motors
- On stop: re-enables failsafes, disarms, clears ap.motor_test flag

---

## 2. ArduPlane

### 2.1 Control Architecture — TECS + L1/NavController + PID

File: `ArduPlane/Plane.cpp`, `ArduPlane/Attitude.cpp`

**Scheduler FAST_TASK sequence:**
```
ahrs_update()        # EKF update, roll/pitch limit scaling with airspeed
update_control_mode() # mode-specific update() call
stabilize()          # roll + pitch + yaw stabilization → PID outputs
set_servos()         # commit PID outputs to servo channels
```

**Three-layer control stack:**

**Layer 1 — Navigation (L1 / nav controller, 10 Hz):**
- `navigate()`: determines `prev_WP_loc` → `next_WP_loc` vector
- `nav_controller->update_waypoint()` (AP_L1_Control or AP_TECS_Nav): computes lateral acceleration demand → `nav_roll_cd`
- Loiter: `update_loiter()` → circles around a point
- L1 controller: pure pursuit + PD law based on cross-track error and heading error

**Layer 2 — Altitude / Speed (TECS, 50 Hz):**
- Total Energy Control System (TECS): `TECS_controller.update_50hz()`
- TECS controls pitch + throttle simultaneously to maintain both altitude and airspeed
- Inputs: target altitude (from `adjust_altitude_target()`), target airspeed (AIRSPEED_CRUISE or mode-specific)
- Outputs: `nav_pitch_cd` (pitch demand), `throttle_out` (0..100)
- TECS is disabled for manual, FBWA/FBWB (which use their own pitch/throttle logic)

**Layer 3 — Stabilization (Roll/Pitch/Yaw PID, fast loop):**
- `stabilize_roll()`: roll PID tracking `nav_roll_cd` → aileron output
- `stabilize_pitch()`: pitch PID tracking `nav_pitch_cd` → elevator output
- `stabilize_yaw()`: yaw/rudder coordination controller
- Speed scaling: `calc_speed_scaler()` scales all PID outputs by `SCALING_SPEED / airspeed`
  - At high speed: less surface deflection (more authority per degree)
  - At low speed: more deflection (less authority per degree)
  - Constrained [0.5, 2.0] to prevent extreme values
  - In VTOL flight: tighter limits around stall airspeed

---

### 2.2 VTOL/QuadPlane

File: `ArduPlane/quadplane.cpp`, `ArduPlane/VTOL_Assist.cpp`

QuadPlane adds a multicopter motor stack to a fixed-wing aircraft. Enabled by `Q_ENABLE = 1`.

**Transition logic (FW → VTOL and VTOL → FW):**

Forward transition (VTOL → Fixed Wing):
- Start multicopter motors at low throttle
- Ramp up forward thrust (main FW motor)
- As airspeed increases past `Q_ASSIST_SPEED`, VTOL motors reduce thrust
- After `Q_TRANSITION_MS` milliseconds past minimum airspeed: transition complete
- `Q_TRANS_FAIL` timeout: if transition takes too long → QLAND or complete forcibly
- During transition: `Q_TRAN_PIT_MAX` limits max pitch (prevents nose-up stall)

Backward transition (FW → VTOL):
- Cut forward motor, spin up VTOL motors
- Deceleration: `Q_TRANS_DECEL` m/s² stopping distance calculation
- Switches to VTOL position control when slow enough

**Q_ASSIST (VTOL Assist in fixed-wing modes):**
- Active when: airspeed < `Q_ASSIST_SPEED` (and > 0) AND vehicle is flying
- Also triggers on attitude error > `Q_ASSIST_ANGLE` for > `Q_ASSIST_DELAY` seconds
- Provides lift/stability backup from multicopter motors while FW surfaces still active
- Force assist: `Q_OPTIONS` bit 7 (always on)
- `VTOL_Assist.cpp`: monitors airspeed and attitude, sets `assist_enabled` flag

**VTOL modes available:**
- QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QACRO, QAUTOTUNE
- Each mirrors the corresponding ArduCopter mode but within ArduPlane context
- VTOL nav uses the same `pos_control`, `wp_nav`, `attitude_control` libraries as ArduCopter

**Tailsitter support:** separate transition logic where the vehicle pitches 90° rather than transitioning horizontally.

---

### 2.3 ArduPlane Modes

| Mode | File | Description |
|------|------|-------------|
| MANUAL | mode_manual.cpp | Direct RC passthrough. Roll/pitch/throttle/rudder → servos 1:1 (with expo). No stabilization. |
| STABILIZE | mode_stabilize.cpp | Roll/pitch stabilized. Pilot commands bank angle and pitch angle. Throttle manual. |
| TRAINING | mode_training.cpp | Like STABILIZE but limits bank/pitch to parameter limits. Releases limits if exceeded. |
| ACRO | mode_acro.cpp | Rate mode. Pilot commands roll rate, pitch rate. |
| FLY BY WIRE A (FBWA) | mode_fbwa.cpp | Pilot commands bank angle (within limits) and pitch angle. TECS controls throttle to maintain airspeed. Failsafe glide (zero roll/pitch, min throttle) if RC lost. |
| FLY BY WIRE B (FBWB) | mode_fbwb.cpp | Like FBWA but auto-throttle. Pilot commands speed via pitch stick. Altitude target set by throttle stick. No GPS needed. |
| CRUISE | mode_cruise.cpp | Like FBWB. When pilot releases roll, locks heading via L1 nav. Re-locks on stick release. |
| AUTO | mode_auto.cpp | Mission execution. TECS + L1 nav. |
| RTL | mode_rtl.cpp | Return to home. Climbs to RTL_ALTITUDE, navigates to home, loiters. Can auto-land if RTL_AUTOLAND set. Optionally switches to QRTL for QuadPlane. |
| LOITER | mode_loiter.cpp | Circles over a point at current altitude. L1 nav. TECS altitude. Optional FBWB-style altitude control with stick. |
| CIRCLE | mode_circle.cpp | Same as LOITER but radius/direction forced from mission command. |
| GUIDED | mode_guided.cpp | External position/velocity commands from GCS. |
| AVOID_ADSB | mode_avoidADSB.cpp | ADS-B avoidance maneuver. |
| THERMAL | mode_thermal.cpp | Soaring mode — exploits thermals. |
| AUTOLAND | mode_autoland.cpp | Automated landing approach and flare. |
| TAKEOFF | mode_takeoff.cpp | Automated takeoff to target altitude and heading. |
| QSTABILIZE | mode_qstabilize.cpp | QuadPlane VTOL stabilize. |
| QHOVER | mode_qhover.cpp | QuadPlane altitude hold. |
| QLOITER | mode_qloiter.cpp | QuadPlane position hold. |
| QLAND | mode_qland.cpp | QuadPlane landing. |
| QRTL | mode_qrtl.cpp | QuadPlane return to launch. |
| QACRO | mode_qacro.cpp | QuadPlane acro. |
| QAUTOTUNE | mode_qautotune.cpp | QuadPlane PID autotuning. |
| LOITER_ALT_QLAND | mode_LoiterAltQLand.cpp | Loiter at altitude then VTOL land. |

---

### 2.4 ArduPlane Landing

Files: `ArduPlane/mode_autoland.cpp`, `ArduPlane/altitude.cpp`

Fixed-wing landing sequence (AUTO mode with NAV_LAND):
1. **Approach**: L1 nav tracks final approach waypoint, TECS holds approach speed + glide slope altitude
2. **Flare**: when `relative_ground_altitude() < LAND_FLARE_ALT` OR `secs_until_landing < LAND_FLARE_SEC`:
   - Pitch up to `LAND_PITCH_CD` (flare pitch)
   - Throttle cut (or to minimum)
3. **Ground detection** (`is_flying.cpp`): monitors airspeed + accelerometer for transition from flying to rolling
4. **Rollout**: differential braking / rudder/steering to track centerline

Ground altitude: `relative_ground_altitude()` uses rangefinder (preferred), terrain database, or home altitude fallback.

VTOL landing (QuadPlane with `Q_OPTIONS` bit 2 clear):
- AUTO mode NAV_LAND command redirected to VTOL land
- Same POSHOLD-style descent as QLAND
- Uses `land_final_alt_m` for final descent speed transition

---

### 2.5 Airspeed Management — Stall Prevention

File: `ArduPlane/Attitude.cpp`, TECS library

**Speed scaling** (`calc_speed_scaler()`):
- Scales all PID surface outputs by `SCALING_SPEED / airspeed`
- At slow speeds: larger deflection (more control)
- At high speeds: less deflection (prevent over-stressing)
- Without airspeed sensor: estimates from throttle output
- VTOL: tighter limits around stall airspeed to prevent large uncommanded surfaces

**TECS stall prevention:**
- TECS has a minimum speed target it will not command below (AIRSPEED_MIN)
- If airspeed drops near stall: TECS commands maximum climb rate first (pitch nose down if needed)
- `aparm.airspeed_min` used as hard floor for speed target
- `aparm.airspeed_stall` used in VTOL QASSIST triggering

**FBWB/CRUISE speed control:**
- `update_fbwb_speed_height()`: throttle stick directly sets airspeed target
- TECS tracks this speed target while maintaining altitude via pitch

---

## 3. Rover

### 3.1 Steering Modes

Files: `ArduRover/Rover.cpp`, `ArduRover/mode_steering.cpp`, `ArduRover/Steering.cpp`

**Scheduler:** main tasks at 400 Hz (`ahrs_update`, `update_current_mode`, `set_servos`).

Three fundamental steering architectures supported:

**1. Ackermann (front-wheel steering):**
- `g2.motors.output()` → single throttle + single steering angle
- Lateral acceleration = V²/R where R is from steering angle and wheelbase
- `calc_steering_from_lateral_acceleration()` → maps lateral accel to steering angle

**2. Skid steer (differential):**
- Two independent motors (left/right)
- Steering = left-right throttle differential
- `g2.motors.have_skid_steering()` true
- Pivot turns: when desired_speed ≈ 0, `get_steering_out_rate()` → pure spin in place

**3. Boat (omni-style with rudder):**
- Separate motor for propulsion and rudder for steering
- Sailboat support: wind-vane driven sailing, tacking logic in `sailboat.cpp`

**Pivot turns** (skid steer only):
```
desired_speed ≈ 0 → pivot turn
  target_turn_rate = (steering_input / 4500) × ACRO_TURN_RATE (rad/s)
  steering_out = attitude_control.get_steering_out_rate(target_turn_rate)
  set_steering(steering_out × 4500)
```

---

### 3.2 Speed Control

Speed PID chain in `ModeSteering::update()` and base `Mode::calc_throttle()`:
```
desired_speed → g2.attitude_control.get_throttle_out_speed(desired_speed, ...)
  # PID: speed_error → throttle
  # Feed-forward from desired acceleration
  # Output: throttle [-1, 1]
→ g2.motors.set_throttle(throttle_out)
```

Speed nudging: in AUTO/Guided, pilot can nudge speed up to WP_SPEED with the throttle stick if `SPEED_NUDGE_STICK` enabled.

---

### 3.3 Rover Modes

| Mode | File | Description |
|------|------|-------------|
| MANUAL | mode_manual.cpp | Direct RC: throttle + steering → motors |
| STEERING | mode_steering.cpp | Pilot commands speed + turn rate. Skid steer: pivot if zero speed. |
| ACRO | mode_acro.cpp | Rate control: pilot commands turn rate directly |
| AUTO | mode_auto.cpp | Mission waypoints via WP_Nav |
| HOLD | mode_hold.cpp | Stop and hold position (braking) |
| RTL | mode_rtl.cpp | Return to home |
| SMART_RTL | mode_smart_rtl.cpp | RTL along recorded path |
| GUIDED | mode_guided.cpp | External position/velocity commands |
| LOITER | mode_loiter.cpp | Circle around a point (boat/rover) |
| FOLLOW | mode_follow.cpp | Follow a beacon or MAVLink vehicle |
| SIMPLE | mode_simple.cpp | Heading-free mode (like copter simple mode) |
| DOCK | mode_dock.cpp | Precision docking |
| CIRCLE | mode_circle.cpp | Circular path |

---

## 4. ArduSub

### 4.1 Scheduler Structure

File: `ArduSub/Sub.cpp`

FAST_TASK loop:
1. `AP_InertialSensor::update` — IMU
2. `run_rate_controller` — inner rate loop
3. `motors_output` — ESC output
4. `read_AHRS` — EKF
5. `read_inertia` — position
6. `check_ekf_yaw_reset`
7. `update_flight_mode` — mode run()
8. `update_home_from_EKF`
9. `update_surface_and_bottom_detector`

### 4.2 Depth Control

File: `ArduSub/mode_althold.cpp`

```
control_depth():
  Surface proximity: scale max_throttle between surface_max_throttle (near surface) and 1.0 (away)
    → motors.set_max_throttle(scaled)
  
  target_climb_rate = pilot throttle → get_pilot_desired_climb_rate()
  
  At surface (ap.at_surface): set pos_desired_Z to ≤ surface_depth (don't ascend above surface)
  At bottom (ap.at_bottom): set pos_desired_Z to ≥ current + 10cm (don't sink into bottom)
  
  pos_control->D_set_pos_target_from_climb_rate_cms(target_climb_rate)
  pos_control->D_update_controller()
    # barometer pressure → depth (via AP_Baro pressure-to-depth conversion)
    # depth error → vertical velocity target → throttle PID
    # feeds to Z-axis thrusters via motor mixer
```

Depth sensor: uses barometer (absolute pressure) converted to depth via water density. `depth = (pressure - surface_pressure) / (water_density * gravity)`.

`surface_bottom_detector.cpp`: monitors position relative to surface/bottom, sets `ap.at_surface` and `ap.at_bottom` flags used by depth control.

### 4.3 6DOF Thruster Mixing

File: `ArduSub/actuators.cpp`, `ArduSub/motors.cpp` (uses AP_Motors_6DOF)

Sub uses AP_Motors_6DOF which supports arbitrary thruster geometries. Standard BlueROV2 configuration has 6 thrusters:
- 4 horizontal thrusters at ±45° for surge (forward/back) + sway (left/right) + yaw
- 2 vertical thrusters for heave (up/down)

Motor mixing: `translate_*_rp()` functions map position/attitude controller outputs to lateral/forward normalized inputs, then the 6DOF motor library applies a configurable motor matrix to distribute to all thrusters.

`motors.cpp` translates `wp_nav` roll/pitch outputs to lateral/forward commands:
```
lateral  = wp_nav.get_roll() / angle_max     # normalized [-1, 1]
forward  = -wp_nav.get_pitch() / angle_max   # normalized, note negation
```

### 4.4 Leak Detection

File: `ArduSub/failsafe.cpp` — `failsafe_leak_check()`

```
status = leak_detector.get_status()
  # AP_LeakDetector: monitors digital pins connected to leak sensors
  # returns true if any sensor detects water

if status AND FS_LEAK != DISABLED:
  AP_Notify::flags.leak_detected = true
  if FS_LEAK == FS_LEAK_SURFACE:
    set_mode(SURFACE, LEAK_FAILSAFE)  # ascend to surface
  Log: FAILSAFE_LEAK
  Warn GCS every 20 seconds
```

No automatic motor shutoff on leak — ascending to surface is the preferred response. Operator then manually secures the vehicle.

### 4.5 Buoyancy Management

ArduSub has no active buoyancy actuators. Buoyancy is set mechanically (foam placement) to achieve neutral buoyancy at operating depth. The depth controller handles minor imbalances via the vertical thrusters. The `surface_max_throttle` parameter reduces available upward thrust near the surface to prevent uncontrolled surfacing when neutrally buoyant. The `at_surface` flag causes the depth controller to cap the altitude target at `surface_depth` to prevent surfacing.

### 4.6 ArduSub Failsafes

In addition to the generic failsafes:
- **Sensor health** (`failsafe_sensors_check`): depth sensor (baro) unhealthy → switch to MANUAL
- **Internal pressure** (`failsafe_internal_pressure_check`): enclosure pressure > `FS_PRESS_MAX` for >2s → warn every 30s
- **Internal temperature** (`failsafe_internal_temperature_check`): enclosure temperature > `FS_TEMP_MAX` for >2s → warn
- **Pilot input timeout** (`failsafe_pilot_input_check`): no RC input for `FS_PILOT_INPUT_TIMEOUT` → `set_neutral_controls()` or disarm
- **Mainloop lockup** (`mainloop_failsafe_check`): if main loop hasn't run for 2 seconds → `motors.output_min()`, then after another 1s → disarm
- **EKF variance** (`failsafe_ekf_check`): compass_variance or vel_variance > `FS_EKF_THRESH` for 2s → warn or disarm
- **Crash check** (`failsafe_crash_check`): attitude error > 30° for 2s → disarm (in stabilized modes only)

### 4.7 ArduSub Modes

| Mode | File | Description |
|------|------|-------------|
| MANUAL | mode_manual.cpp | Direct joystick → thruster |
| STABILIZE | mode_stabilize.cpp | Attitude stabilized, manual depth |
| ACRO | mode_acro.cpp | Rate mode |
| ALT_HOLD | mode_althold.cpp | Depth hold via baro |
| POS_HOLD | mode_poshold.cpp | 3D position hold (GPS or DVL) |
| AUTO | mode_auto.cpp | Mission execution |
| GUIDED | mode_guided.cpp | External position commands |
| CIRCLE | mode_circle.cpp | Circle maneuver |
| SURFACE | mode_surface.cpp | Ascend to surface, then idle |
| SURFTRAK | mode_surftrak.cpp | Track distance from seafloor via rangefinder |
| MOTOR_DETECT | mode_motordetect.cpp | Motor detection/test |

---

## 5. Blimp

### 5.1 What Makes a Blimp Different

File: `Blimp/Blimp.cpp`, `Blimp/Fins.cpp`, `Blimp/Loiter.cpp`

A blimp is neutrally buoyant by design — it needs zero sustained thrust to maintain altitude. This eliminates the multicopter's "hover throttle" concept entirely. The control challenge is that the vehicle is very slow, has high inertia, and the fin-based propulsion generates force via oscillation rather than steady thrust.

**No rate controller:** The Blimp scheduler has no `run_rate_controller` FAST_TASK. Attitude control is done directly through the fin mix, not via a separate inner loop.

**Fin-based propulsion:** 4 fins (Back, Front, Right, Left), each oscillating at a configurable frequency (`FINS_FREQ_HZ`, default 3 Hz). Thrust is generated by the oscillation's asymmetric drag — the fin sweeps faster in one direction than the other (DC offset on the oscillation), creating net force.

Each fin has amplitude and offset factors for 4 axes (right, front, down, yaw):
```
Back fin:  front-amplitude=1.0, down-amplitude=0.5
Front fin: front-amplitude=-1.0, down-amplitude=0.5
Right fin: right-amplitude=-1.0, yaw-amplitude=0.5
Left fin:  right-amplitude=1.0, yaw-amplitude=-0.5
```

Fin output: `_pos[i] = _amp[i] * cos(freq_hz × _freq[i] × time × 2π) + _off[i]`
- `_amp` = oscillation amplitude (controls how much that fin moves in total)
- `_off` = DC offset (controls net thrust direction)
- Turbo mode: if amplitude ≤ 0.6 and offset ≥ 0.4, doubles oscillation frequency for faster response

Normalization: if total amplitude + offset > 1, amplitude is reduced to keep total ≤ 1.

### 5.2 Blimp Control Architecture

**Loiter controller** (`Blimp/Loiter.cpp`):

Cascaded PID, all in NED earth frame:
```
Position error (target_pos - pos_ned) 
  → pid_pos_xy (2D PID) → target_vel_ef (m/s, NE)
  → pid_pos_z (1D PID) → target_vel_ef.z
  → Constrain to max_vel_xy, max_vel_z

Yaw error (wrap_PI(target_yaw - current_yaw))
  → pid_pos_yaw → target_vel_yaw

Scale by output saturation:
  scaler_xz: prevents front/down outputs from summing > 1
  scaler_yyaw: prevents right/yaw outputs from summing > 1

target_vel_ef scaled by saturations
  → pid_vel_xy → actuator_NE (2D)
  → pid_vel_z → act_down
  → pid_vel_yaw → act_yaw

Rotate actuator_NE from earth frame to body frame
  → blimp.motors.right_out, front_out, down_out, yaw_out
  → Fins::output() applies oscillation
```

Deadband: if position error < `PID_DZ` parameter, that axis PID outputs zero (prevents buzzing around setpoint).

Disabled axes: via `DIS_MASK` bitmask — can disable individual axes (useful for setup/testing).

**Modes:**
- MANUAL: direct fin control from sticks
- LOITER: position + yaw hold using cascaded PID above
- RTL: fly to home position using loiter controller
- LAND: descend to ground using loiter controller with downward target
- VELOCITY: fly at commanded velocity (no position hold)

---

## 6. Key Differences and Porting Priorities for Meridian

### ArduCopter → Meridian (Highest Priority)

1. **Scheduler pattern**: the FAST_TASK / SCHED_TASK split with deterministic priority ordering. Meridian needs an equivalent. Rate controller must run before EKF, flight mode must run after EKF.

2. **Rate controller separation**: the rate PID (gyro-only) is decoupled from the attitude + position controllers (EKF-dependent). This is essential for low-latency vibration rejection.

3. **Spool state machine**: `SHUT_DOWN → GROUND_IDLE → SPOOLING_UP → THROTTLE_UNLIMITED → SPOOLING_DOWN`. Every mode interacts with this. Meridian must implement equivalent.

4. **AltHold state machine**: `MotorStopped → Landed_Ground_Idle → Landed_Pre_Takeoff → Takeoff → Flying`. This is reused verbatim by AltHold, Loiter, PosHold, and many others.

5. **RTL path builder** (`build_path()`): complex logic for terrain-aware return altitude with cone-slope, fence constraints, rally points. All of this needs porting.

6. **Land detector**: the multi-condition detector (motor limit + throttle mix + angle + accel + velocity + rangefinder + WoW) is carefully tuned. Do not simplify.

7. **Failsafe action matrix**: the 8-way failsafe action enum with fallback chains (SmartRTL → RTL → Land) is production-critical.

8. **Pre-arm check architecture**: bitmask-controlled skippable checks, bitwise-AND to run all checks even if one fails (collect all failure messages rather than stop at first).

### ArduPlane → Meridian

1. **TECS**: Total Energy Control System is a major library. Speed + altitude controlled simultaneously via pitch + throttle. Cannot be simplified to separate PID loops without losing the coupled behavior.

2. **Speed scaling**: PID output scaled by `SCALING_SPEED / airspeed`. Critical for safe flight across speed range.

3. **L1 navigation**: lateral acceleration-based path tracking. The `calc_nav_roll()` path (L1 → nav_roll_cd → stabilize_roll → PID → servo) is the core FW navigation chain.

4. **QuadPlane transition**: complex state machine with timing, airspeed, and failure conditions. If Meridian supports VTOL, this needs careful porting.

### Rover → Meridian

1. **Motor type abstraction**: Ackermann vs skid-steer vs boat handled at the motors layer. Mode code uses the same `calc_throttle()` / `calc_steering_*()` regardless.

2. **Pivot turn logic**: zero-speed pivot is a special case in skid-steer that bypasses the lateral acceleration controller.

### ArduSub → Meridian

1. **6DOF motor matrix**: the AP_Motors_6DOF arbitrary thruster geometry is needed for any underwater or 6DOF vehicle.

2. **Depth from pressure**: baro library repurposed for underwater depth measurement.

3. **Leak detector failsafe**: unique to sub, surfaces vehicle on water ingress.

### Blimp → Meridian

1. **Fin oscillation model**: entirely unique to blimps. The amplitude/offset sine-wave approach with per-axis mixing factors is not shared with any other vehicle type.

2. **No hover throttle**: the absence of a hover-throttle concept means the position controller cannot assume any throttle offset.

---

## 7. Files Read

1. `ArduCopter/Copter.cpp` — scheduler tasks, main structure
2. `ArduCopter/mode_stabilize.cpp` — stick-to-motor path
3. `ArduCopter/mode_althold.cpp` — altitude hold controller
4. `ArduCopter/mode_loiter.cpp` — position hold + pilot input
5. `ArduCopter/mode_rtl.cpp` — full RTL state machine
6. `ArduCopter/mode_land.cpp` — land mode + land detector hook
7. `ArduCopter/land_detector.cpp` — land detection algorithm
8. `ArduCopter/mode_auto.cpp` (partial) — mission execution dispatch
9. `ArduCopter/AP_Arming_Copter.cpp` — all pre-arm checks
10. `ArduCopter/events.cpp` — failsafe events and actions
11. `ArduCopter/motor_test.cpp` — motor test + E-stop
12. `ArduPlane/Plane.cpp` — scheduler, ahrs_update, TECS call
13. `ArduPlane/Attitude.cpp` — speed scaling, stabilize_roll
14. `ArduPlane/altitude.cpp` — altitude target management
15. `ArduPlane/mode_manual.cpp` — direct passthrough
16. `ArduPlane/mode_fbwa.cpp` — fly-by-wire-A
17. `ArduPlane/mode_fbwb.cpp` — fly-by-wire-B
18. `ArduPlane/mode_cruise.cpp` — heading-lock cruise
19. `ArduPlane/mode_loiter.cpp` — loiter with L1
20. `ArduPlane/mode_rtl.cpp` — fixed-wing RTL + QuadPlane QRTL switch
21. `ArduPlane/quadplane.cpp` (params section) — QuadPlane parameters, assist/transition
22. `Rover/Rover.cpp` — scheduler
23. `Rover/Steering.cpp` — set_servos
24. `Rover/mode_steering.cpp` — steering mode + pivot turn
25. `ArduSub/Sub.cpp` — scheduler
26. `ArduSub/mode_althold.cpp` — depth hold
27. `ArduSub/motors.cpp` — thruster translation, motor test
28. `ArduSub/failsafe.cpp` — all sub-specific failsafes including leak
29. `Blimp/Blimp.cpp` — scheduler
30. `Blimp/Fins.cpp` — fin oscillation output
31. `Blimp/Loiter.cpp` — cascaded position PID
