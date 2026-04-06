# ArduPilot Vehicle/Mode/Safety Parity Identification
**Scope**: Vehicle flight modes, mission execution, arming, geofence, rally, SmartRTL,
waypoint navigation, landing, follow, obstacle avoidance, and ADSB avoidance.
**Generated**: 2026-04-02 against ArduPilot `master` and Meridian current HEAD.

---

## TABLE OF CONTENTS

1. [ArduCopter — Mode Files](#arducopter-mode-files)
2. [ArduCopter — Safety / Arming Files](#arducopter-safety--arming-files)
3. [ArduPlane — Mode Files](#arduplane-mode-files)
4. [ArduPlane — VTOL / Tailsitter](#arduplane-vtol--tailsitter)
5. [ArduPlane — Safety / Arming Files](#arduplane-safety--arming-files)
6. [Rover — Mode Files](#rover-mode-files)
7. [Rover — Safety / Arming Files](#rover-safety--arming-files)
8. [ArduSub — Mode Files](#ardusub-mode-files)
9. [Blimp — Mode Files](#blimp-mode-files)
10. [Blimp — Safety / Arming Files](#blimp-safety--arming-files)
11. [Libraries — Mission Execution](#libraries--mission-execution)
12. [Libraries — Arming Checks](#libraries--arming-checks)
13. [Libraries — Geofence](#libraries--geofence)
14. [Libraries — Rally Points](#libraries--rally-points)
15. [Libraries — SmartRTL Path Recording](#libraries--smartrtl-path-recording)
16. [Libraries — Waypoint Navigation](#libraries--waypoint-navigation)
17. [Libraries — Fixed-Wing Landing](#libraries--fixed-wing-landing)
18. [Libraries — Follow Mode Support](#libraries--follow-mode-support)
19. [Libraries — Obstacle Avoidance (Proximity / OA)](#libraries--obstacle-avoidance-proximity--oa)
20. [Libraries — ADSB Avoidance](#libraries--adsb-avoidance)
21. [Summary Gap Table](#summary-gap-table)

---

## ArduCopter — Mode Files

---

FILE: ArduCopter/mode.cpp
LINES: 1136
PURPOSE: Mode base class, mode switch state machine, common helpers (lean angles, land
  run, precland, pilot desired rates, AltHold state machine, simple mode, avoidance
  climb-rate adjustment, output_to_motors).
KEY FUNCTIONS: set_mode(), update_flight_mode(), exit_mode(), get_alt_hold_state_D_ms(),
  land_run_vertical_control(), land_run_horizontal_control(), land_run_normal_or_precland(),
  precland_run(), precland_retry_position(), get_avoidance_adjusted_climbrate_ms(),
  get_pilot_desired_lean_angles_rad(), zero_throttle_and_relax_ac(), output_to_motors()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs (MultirotorModes), 
  crates/meridian-types/src/vehicle.rs (VehicleLifecycle)
PARITY STATUS: PARTIAL
GAPS: precland_retry_position() not implemented; avoidance-adjusted climb rate is stub;
  simple mode (compass-independent) not implemented; mode_change_failed() GCS reporting
  absent; gcs_mode_enabled() bitmask check not present; get_available_mode_enabled_mask()
  missing; AUTO_RTL pseudo-mode transition logic not modeled.

---

FILE: ArduCopter/mode_acro.cpp
LINES: 201
PURPOSE: Full rate-controlled acro (3-axis rate) with optional air-mode, throttle
  hover estimate, and virtual flybar for helis.
KEY FUNCTIONS: run(), init(), exit(), air_mode_aux_changed(), throttle_hover(),
  get_pilot_desired_rates_rads()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_acro()
PARITY STATUS: PARTIAL
GAPS: Air-mode toggle (aux channel, dynamic enabling) not present; throttle_hover()
  per-mode override absent; virtual flybar path (heli) not present in Meridian.

---

FILE: ArduCopter/mode_acro_heli.cpp
LINES: 118
PURPOSE: Helicopter-specific acro with virtual flybar simulation for traditional head.
KEY FUNCTIONS: init(), run(), virtual_flybar()
MERIDIAN EQUIVALENT: crates/meridian-heli/src/lib.rs (partial)
PARITY STATUS: MISSING
GAPS: virtual_flybar() function not present in Meridian; no heli-acro mode path;
  collective-based yaw authority not modeled.

---

FILE: ArduCopter/mode_althold.cpp
LINES: 104
PURPOSE: Altitude hold with pilot pitch/roll input and climb-rate from throttle.
  Feeds position controller for vertical axis, attitude controller for horizontal.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_althold()
PARITY STATUS: FULL
GAPS: None material; Meridian implements the core AltHold state machine with takeoff
  detection and equivalent control outputs.

---

FILE: ArduCopter/mode_auto.cpp
LINES: 2448
PURPOSE: Autonomous mission execution. Sub-modes: takeoff, waypoint, land, RTL,
  circle-to-edge, guided, loiter, nav_script_time. AUTO_RTL pseudo-mode transition.
  Speed override, payload place, spline waypoints.
KEY FUNCTIONS: init(), run(), set_submode(), loiter_start(), rtl_start(),
  takeoff_start(), wp_start(), land_start(), circle_movetoedge_start(), circle_start(),
  nav_guided_start(), nav_script_time(), is_landing(), is_taking_off(),
  jump_to_landing_sequence_auto_RTL(), enter_auto_rtl(),
  return_path_start_auto_RTL(), set_speed_NE_ms(), set_speed_up_ms(), set_speed_down_ms()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_auto();
  crates/meridian-mission/src/lib.rs (MissionExecutor)
PARITY STATUS: PARTIAL
GAPS: AUTO_RTL pseudo-mode (mode 27) not modeled; PayloadPlace sub-mode absent;
  nav_script_time Lua callback absent; circle_movetoedge approach sequence not
  implemented; spline waypoints not dispatched from Auto; per-waypoint speed override
  via SPEED command not wired to mission executor; DO_LAND_START jump target absent.

---

FILE: ArduCopter/mode_autorotate.cpp
LINES: 136
PURPOSE: Emergency autorotation entry/glide/flare for traditional helicopters.
  Requires RSC governor.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-heli/src/lib.rs → AutorotationController
PARITY STATUS: PARTIAL
GAPS: Meridian has AutorotationController struct and phases (Entry, Glide, Flare,
  TouchDown) but it is not wired as a FlightModeId; the mode_autorotate entry
  path (init/run) does not exist as a dispatched flight mode in meridian-modes.

---

FILE: ArduCopter/mode_autotune.cpp
LINES: 124
PURPOSE: PID autotune. Hovers at fixed altitude while exercising axes; saves gains
  on completion. Re-uses AltHold vertical control.
KEY FUNCTIONS: AutoTune::init(), AutoTune::run(), AutoTune::get_desired_climb_rate_ms(),
  AutoTune::get_pilot_desired_rp_yrate_rad(), AutoTune::log_pids(), position_ok()
MERIDIAN EQUIVALENT: crates/meridian-autotune/src/tuner.rs; 
  crates/meridian-modes/src/multirotor.rs → FlightModeId::Autotune (stub → update_stabilize)
PARITY STATUS: PARTIAL
GAPS: Autotune tuner crate exists but is not wired to the flight mode dispatch;
  mode runs Stabilize control loop only; axis sequencing, gain save/restore, and
  log_pids() not connected.

---

FILE: ArduCopter/mode_avoid_adsb.cpp
LINES: 42
PURPOSE: Temporary ADSB collision avoidance mode. Accepts velocity commands from
  AP_Avoidance and feeds them to guided control. Exits back to prior mode.
KEY FUNCTIONS: init(), set_velocity_NEU_ms(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs →
  FlightModeId::AvoidAdsb (stub → update_loiter)
PARITY STATUS: STUB
GAPS: Velocity command from ADSB threat handler not wired; no prior-mode save/restore;
  AP_Avoidance threat-level callback absent.

---

FILE: ArduCopter/mode_brake.cpp
LINES: 88
PURPOSE: Velocity braking. Decelerates to zero via position controller, then optionally
  transitions to Loiter after a configurable timeout.
KEY FUNCTIONS: init(), run(), timeout_to_loiter_ms()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_brake()
PARITY STATUS: PARTIAL
GAPS: timeout_to_loiter_ms() callable from external (e.g. failsafe) not wired;
  Meridian brake returns PositionTarget(hold) correctly but decelerator ramp logic
  delegates entirely to position controller without the velocity-based fast-brake
  early-exit ArduPilot performs.

---

FILE: ArduCopter/mode_circle.cpp
LINES: 143
PURPOSE: Circle around a fixed center point at configurable radius/rate. Integrates
  AC_Circle library. Allows pilot climb/descent during circle.
KEY FUNCTIONS: init(), run(), wp_distance_m(), wp_bearing_deg()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_circle()
PARITY STATUS: PARTIAL
GAPS: AC_Circle terrain-following path not present; radius/rate in-flight adjustment
  via GCS command absent; move-to-edge approach before circling not ported;
  pilot yaw override during circle not modeled.

---

FILE: ArduCopter/mode_drift.cpp
LINES: 165
PURPOSE: Fixed-wing-like roll-to-turn copter mode. Pilot roll mapped to yaw,
  throttle assist based on descent rate.
KEY FUNCTIONS: init(), run(), get_throttle_assist()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_drift()
PARITY STATUS: PARTIAL
GAPS: get_throttle_assist() velocity-D feed-forward not fully replicated; the
  lateral slip/wind correction modeling in Meridian is simplified.

---

FILE: ArduCopter/mode_flip.cpp
LINES: 217
PURPOSE: Aerobatic 360-degree roll or pitch flip. State machine: WaitForStable →
  Start → Roll → Recover → Finish. Feeds rate targets directly.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_flip()
PARITY STATUS: PARTIAL
GAPS: Flip direction (roll vs pitch vs inverted) based on stick position absent;
  recovery phase attitude target not as precise as ArduPilot's; pre-flip stability
  check (is_disarmed_or_landed) not present in Meridian entry guard.

---

FILE: ArduCopter/mode_flowhold.cpp
LINES: 531
PURPOSE: Optical-flow-based position hold without GPS. Flow sensor provides
  body-frame velocity estimate; PI controllers correct for flow error.
KEY FUNCTIONS: ModeFlowHold(), enabled(), init(), flowhold_flow_to_angle(),
  run(), update_height_estimate()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs →
  FlightModeId::FlowHold (stub → update_loiter)
PARITY STATUS: STUB
GAPS: Entire flowhold_flow_to_angle() algorithm absent; flow sensor integration
  absent; height estimate from flow not present; no PI controller for flow error;
  optical flow driver output not consumed by flight mode.

---

FILE: ArduCopter/mode_follow.cpp
LINES: 165
PURPOSE: Follow another MAVLink vehicle using AP_Follow library. Maintains position
  offset relative to target.
KEY FUNCTIONS: enabled(), init(), exit(), run(), wp_distance_m(),
  wp_bearing_deg(), get_wp()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_follow()
PARITY STATUS: PARTIAL
GAPS: AP_Follow multi-offset modes (body-frame, NED-frame, use-offsets) not fully
  modeled; yaw-to-target behavior absent; target-lost failsafe not wired; velocity
  feed-forward from target telemetry not present.

---

FILE: ArduCopter/mode_guided.cpp
LINES: 1214
PURPOSE: External control mode. Accepts position, velocity, acceleration, angle,
  or combined PVA setpoints. Sub-control types: WP, PVA, pos, vel, velaccel,
  posvelaccel, angle. Weather-vane support. EKF reset handling.
KEY FUNCTIONS: init(), hold_position(), run(), wp_control_start/run(),
  pva_control_start(), pos_control_start(), accel_control_start(),
  velaccel_control_start(), posvelaccel_control_start(), angle_control_start(),
  set_pos_NED_m(), set_destination(), set_accel_NED_mss(), set_vel_NED_ms(),
  set_vel_accel_NED_m(), set_pos_vel_NED_m(), set_pos_vel_accel_NED_m(),
  set_attitude_target_provides_thrust(), move_vehicle_on_ekf_reset()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_guided()
PARITY STATUS: PARTIAL
GAPS: update_guided() delegates all sub-types to position_target — only the
  pos-hold variant is functional; velocity-only, acceleration-only, velaccel,
  posvelaccel, and angle sub-modes not dispatched; weather-vane integration absent;
  EKF reset handling (move_vehicle_on_ekf_reset) absent; GCS takeoff via guided
  not implemented.

---

FILE: ArduCopter/mode_guided_custom.cpp
LINES: 23
PURPOSE: User-scriptable guided mode slot for custom MAVLink mode numbers.
  Subclasses ModeGuided with a configurable mode number for autopilot expansion.
KEY FUNCTIONS: ModeGuidedCustom(), init()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: No equivalent mechanism for custom MAVLink mode-number registration.

---

FILE: ArduCopter/mode_guided_nogps.cpp
LINES: 25
PURPOSE: Guided without GPS. Accepts attitude/rate targets from external source
  when GPS is unavailable. Falls back to attitude control.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs →
  update_guided_nogps()
PARITY STATUS: PARTIAL
GAPS: External attitude target injection (MAVLink SET_ATTITUDE_TARGET) not wired;
  arm-in-guided-nogps not tested.

---

FILE: ArduCopter/mode_land.cpp
LINES: 220
PURPOSE: Autonomous landing. Two paths: GPS landing (position-controlled approach
  to ground) and no-GPS landing (throttle-only descent). Integrates precland,
  disarm on touchdown detection.
KEY FUNCTIONS: ModeLand(), convert_params(), init(), run(), gps_run(),
  nogps_run(), do_not_use_GPS(), set_mode_land_with_pause(), landing_with_GPS()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_land()
PARITY STATUS: PARTIAL
GAPS: Precland integration absent (Meridian has meridian-precland crate but it is
  not called from Land mode); landing_with_GPS() GPS-quality check not modeled;
  set_mode_land_with_pause() (loiter then land) flow absent; convert_params()
  parameter migration not needed but landing disarm delay configurable parameter
  not exposed.

---

FILE: ArduCopter/mode_loiter.cpp
LINES: 200
PURPOSE: GPS-hold with pilot position input. Integrates precision loiter
  (precland beacon), pilot stick input, and AC_Loiter controller.
KEY FUNCTIONS: init(), do_precision_loiter(), precision_loiter_xy(), run(),
  wp_distance_m(), wp_bearing_deg()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_loiter()
PARITY STATUS: PARTIAL
GAPS: Precision loiter (IR-LOCK / landing beacon) path absent; pilot-commanded
  loiter entry position not captured from velocity; throttle-to-climb integration
  with AltHold lever absent.

---

FILE: ArduCopter/mode_poshold.cpp
LINES: 669
PURPOSE: Hybrid position hold: pilot input stirs vehicle like AltHold, braking
  smoothly into position hold with wind compensation estimate.
KEY FUNCTIONS: ModePosHold(), convert_params(), init(), run(),
  update_pilot_lean_angle_rad(), mix_controls(), update_brake_angle_from_velocity(),
  init_wind_comp_estimate(), update_wind_comp_estimate(),
  get_wind_comp_lean_angles_rad(), roll/pitch_controller_to_pilot_override()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_poshold()
PARITY STATUS: PARTIAL
GAPS: Wind compensation estimator (update_wind_comp_estimate) not implemented;
  mix_controls() ratio logic present but wind-comp lean-angle blend absent;
  pilot-override to controller back-transition not modeled.

---

FILE: ArduCopter/mode_rtl.cpp
LINES: 645
PURPOSE: Return-to-launch with configurable alt type (terrain-follow, absolute,
  cone). State machine: Climb → Return → LoiterAtHome → Descent → Land.
  Rally point support. Precision landing on final.
KEY FUNCTIONS: ModeRTL(), convert_params(), init(), restart_without_terrain(),
  get_alt_type(), run(), climb_start(), return_start(), climb_return_run(),
  loiterathome_start(), loiterathome_run(), descent_start(), descent_run(),
  land_start(), is_landing(), land_run(), build_path(), compute_return_target(),
  get_wp(), wp_distance_m(), wp_bearing_deg(), use_pilot_yaw(),
  set_speed_NE_ms(), set_speed_up_ms(), set_speed_down_ms()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_rtl()
PARITY STATUS: PARTIAL
GAPS: RTL alt type (terrain-relative vs cone) not modeled; cone-shaped altitude
  calc (compute_return_target) not present; rally-point selection not wired
  (meridian-nav has RallyManager but not called from RTL); loiterathome_run
  timeout configurable via parameter absent; precland on final absent;
  restart_without_terrain() terrain-data fallback not present.

---

FILE: ArduCopter/mode_smart_rtl.cpp
LINES: 222
PURPOSE: SmartRTL — retraces recorded flight path back to home. State machine:
  WaitForCleanup → PathFollow → PreLand → Land.
KEY FUNCTIONS: enabled(), init(), exit(), run(), is_landing(),
  wait_cleanup_run(), path_follow_run(), pre_land_position_run(), save_position(),
  get_wp(), wp_distance_m(), wp_bearing_deg(), use_pilot_yaw()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_smartrtl();
  crates/meridian-nav/src/smartrtl.rs (SmartRTL)
PARITY STATUS: PARTIAL
GAPS: wait_cleanup_run() (background path simplification pause) not present in
  dispatch; pre_land_position_run() (fly to above home before descent) absent;
  save_position() interval not connected to flight loop; SmartRTL disable
  on deactivation not wired; thorough vs. routine cleanup types not distinguished.

---

FILE: ArduCopter/mode_sport.cpp
LINES: 125
PURPOSE: Sport mode: rate-based roll/pitch from stick, altitude hold, fast
  response for aggressive manual flying.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_sport()
PARITY STATUS: FULL
GAPS: None material for core behavior. ArduPilot applies a separate angle_max
  for sport mode which Meridian does not distinguish.

---

FILE: ArduCopter/mode_stabilize.cpp
LINES: 64
PURPOSE: Pure attitude stabilize — pilot commands lean angle; no altitude hold,
  no position control. Simplest mode.
KEY FUNCTIONS: run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_stabilize()
PARITY STATUS: FULL
GAPS: None material.

---

FILE: ArduCopter/mode_stabilize_heli.cpp
LINES: 82
PURPOSE: Helicopter stabilize — collective mapped to throttle, RSC governor
  manages rotor RPM, manual collective passthrough.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-heli/src/lib.rs (RSC, collective);
  no dedicated heli-stabilize mode in meridian-modes
PARITY STATUS: MISSING
GAPS: No Heli-Stabilize mode dispatched; RSC governor not wired to flight mode;
  collective mapping absent from mode layer.

---

FILE: ArduCopter/mode_systemid.cpp
LINES: 453
PURPOSE: System identification — applies chirp/doublet excitation on selected
  axis while logging response for frequency analysis.
KEY FUNCTIONS: ModeSystemId(), enabled(), init(), exit(), run(), log_data(),
  is_poscontrol_axis_type()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs →
  FlightModeId::SystemId (stub → update_loiter)
PARITY STATUS: STUB
GAPS: Entire chirp/doublet waveform generator absent; axis selection absent;
  log_data() format not present; no SYSTEMID_AXIS parameter; not safe to fly.

---

FILE: ArduCopter/mode_throw.cpp
LINES: 330
PURPOSE: Hand-launch mode. Waits for throw detection (acceleration spike),
  then stabilizes and holds position.
KEY FUNCTIONS: init(), run(), throw_detected(), throw_attitude_good(),
  throw_height_good(), throw_position_good()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_throw()
PARITY STATUS: PARTIAL
GAPS: throw_detected() acceleration-spike threshold not configurable via parameter;
  throw_position_good() GPS quality gate absent; Meridian Throw does not
  distinguish THROW_TYPE_UP vs THROW_TYPE_DROP.

---

FILE: ArduCopter/mode_turtle.cpp
LINES: 238
PURPOSE: Turtle/DShot flip recovery. Reverses motor direction via DShot command
  to flip inverted drone. Special arming method bypass.
KEY FUNCTIONS: enabled(), init(), arm_motors(), allows_arming(), exit(),
  disarm_motors(), change_motor_direction(), run(), output_to_motors()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_turtle()
PARITY STATUS: PARTIAL
GAPS: change_motor_direction() DShot command not present (requires DShot layer);
  arm_motors() special arming bypass not present; output_to_motors() direct
  motor output override not wired; Meridian returns RateTarget(zero) only.

---

FILE: ArduCopter/mode_zigzag.cpp
LINES: 578
PURPOSE: ZigZag agricultural spraying. Records two waypoints (A and B), then
  autonomously traverses between them while pilot controls lateral offset.
  Supports auto sprayer trigger.
KEY FUNCTIONS: ModeZigZag(), init(), exit(), run(), save_or_move_to_destination(),
  move_to_side(), return_to_manual_control(), auto_control(), manual_control(),
  reached_destination(), calculate_next_dest_m(), calculate_side_dest_m(),
  run_auto(), suspend_auto(), init_auto(), spray(), wp_distance_m(),
  wp_bearing_deg(), crosstrack_error_m()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/multirotor.rs → update_zigzag()
PARITY STATUS: PARTIAL
GAPS: Side-offset calculation (calculate_side_dest_m) may not handle terrain-alt;
  auto-spray trigger (spray()) not wired to meridian-sprayer crate; crosstrack
  error reporting absent; auto-repeat count not parameterized.

---

## ArduCopter — Safety / Arming Files

---

FILE: ArduCopter/AP_Arming_Copter.cpp
LINES: 857
PURPOSE: Copter-specific arming checks: RC throttle failsafe, barometer, INS,
  board voltage, terrain DB, parameter, OA system, RC calibration, GPS quality,
  EKF attitude, proximity, mandatory position, GCS failsafe, winch, altitude,
  full arm sequence, disarm sequence.
KEY FUNCTIONS: pre_arm_checks(), run_pre_arm_checks(), rc_throttle_failsafe_checks(),
  barometer_checks(), ins_checks(), board_voltage_checks(), terrain_database_required(),
  parameter_checks(), oa_checks(), rc_calibration_checks(), gps_checks(),
  pre_arm_ekf_attitude_check(), proximity_checks(), mandatory_position_checks(),
  gcs_failsafe_check(), winch_checks(), alt_checks(), arm_checks(),
  mandatory_checks(), set_pre_arm_check(), arm(), disarm()
MERIDIAN EQUIVALENT: crates/meridian-arming/src/lib.rs (check_prearm, ArmingState)
PARITY STATUS: PARTIAL
GAPS: oa_checks() (OA system health gate) returns false hardcoded (GAP 43);
  proximity_checks() proximity sensor health absent; winch_checks() absent;
  mandatory_position_checks() (e.g. require GPS for guided) partially present;
  terrain_database_required() per-mode assessment absent; rc_throttle_failsafe_checks()
  stick-calibration path not replicated; arm() / disarm() sequence (motor spoolup,
  logging) not fully implemented.

---

FILE: ArduCopter/failsafe.cpp
LINES: 84
PURPOSE: Failsafe enable/disable, periodic failsafe_check() dispatcher,
  advanced-failsafe (AFS) check wrapper.
KEY FUNCTIONS: failsafe_enable(), failsafe_disable(), failsafe_check(), afs_fs_check()
MERIDIAN EQUIVALENT: crates/meridian-failsafe/src/lib.rs (various Monitors)
PARITY STATUS: PARTIAL
GAPS: afs_fs_check() (advanced failsafe scripting) not present; failsafe_enable/disable
  (dynamic, e.g. during calibration) not wired.

---

FILE: ArduCopter/events.cpp
LINES: 525
PURPOSE: All failsafe event handlers: radio on/off, battery, GCS link, terrain,
  GPS glitch, dead reckoning. Mode-switch logic for each failsafe
  (RTL/SmartRTL/Land/Brake/DoLandStart cascades).
KEY FUNCTIONS: failsafe_option(), failsafe_radio_on_event(), failsafe_radio_off_event(),
  announce_failsafe(), handle_battery_failsafe(), failsafe_gcs_check(),
  failsafe_gcs_on_event(), failsafe_gcs_off_event(), failsafe_terrain_check(),
  failsafe_terrain_set_status(), failsafe_terrain_on_event(), gpsglitch_check(),
  failsafe_deadreckon_check(), set_mode_RTL_or_land_with_pause(),
  set_mode_SmartRTL_or_land_with_pause(), set_mode_SmartRTL_or_RTL(),
  set_mode_auto_do_land_start_or_RTL(), set_mode_brake_or_land_with_pause(),
  should_disarm_on_failsafe(), do_failsafe_action()
MERIDIAN EQUIVALENT: crates/meridian-failsafe/src/lib.rs (monitors and watchdog)
PARITY STATUS: PARTIAL
GAPS: Mode-cascade logic (SmartRTL→RTL→Land→Brake) not fully wired to FailsafeAction
  dispatch; terrain_on_event() (switch to AltHold + warn) not present;
  gpsglitch_check() glitch filtering not replicated; failsafe_deadreckon_check()
  dead-reckoning quality metric absent; announce_failsafe() GCS notification absent.

---

## ArduPlane — Mode Files

---

FILE: ArduPlane/mode.cpp
LINES: 414
PURPOSE: Fixed-wing mode base class. Common enter/exit, altitude update,
  pre-arm checks per mode, throttle/rudder output helpers, FW SystemID check.
KEY FUNCTIONS: Mode(), exit(), enter(), is_vtol_man_throttle(),
  update_target_altitude(), pre_arm_checks(), _pre_arm_checks(), run(),
  reset_controllers(), is_taking_off(), output_rudder_and_steering(),
  output_pilot_throttle(), use_throttle_limits(), use_battery_compensation(),
  allow_fw_systemid()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs (FixedWingModes)
PARITY STATUS: PARTIAL
GAPS: is_vtol_man_throttle() VTOL throttle passthrough not modeled; per-mode
  pre_arm_checks() not dispatched; battery compensation flag absent;
  allow_fw_systemid() absent; rudder/steering split-output not present.

---

FILE: ArduPlane/mode_LoiterAltQLand.cpp
LINES: 59
PURPOSE: Fixed-wing loiter at altitude, then transition to Q-Land (VTOL landing)
  when over landing point. VTOL hybrid mode.
KEY FUNCTIONS: _enter(), navigate(), switch_qland(), handle_guided_request()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: No LoiterAltQLand mode in Meridian; VTOL altitude loiter → Q-Land transition
  state machine entirely absent.

---

FILE: ArduPlane/mode_acro.cpp
LINES: 227
PURPOSE: Fixed-wing full rate control. Two sub-paths: conventional (quaternion
  rate integration) and quaternion stabilize.
KEY FUNCTIONS: _enter(), update(), run(), stabilize(), stabilize_quaternion()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_fw_acro()
PARITY STATUS: PARTIAL
GAPS: stabilize_quaternion() path not implemented; rate integrator anti-windup
  not present; FW acro parameter set (ACRO_ROLL_RATE, etc.) not connected.

---

FILE: ArduPlane/mode_auto.cpp
LINES: 202
PURPOSE: Fixed-wing autonomous mission. Handles DO_LAND_START, rally-return,
  and does_auto_navigation / does_auto_throttle dispatch.
KEY FUNCTIONS: _enter(), _exit(), update(), navigate(), does_auto_navigation(),
  does_auto_throttle(), _pre_arm_checks(), is_landing(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_auto();
  crates/meridian-mission/src/lib.rs
PARITY STATUS: PARTIAL
GAPS: does_auto_navigation / does_auto_throttle split dispatch not present;
  _pre_arm_checks (mission first-WP altitude) not present; rally auto-entry
  not wired; DO_LAND_START search in mission not present.

---

FILE: ArduPlane/mode_autoland.cpp
LINES: 315
PURPOSE: Automated landing without pre-planned approach. Vehicle flies to runway
  threshold, aligns, then lands. Includes arm_check() and takeoff-direction guard.
KEY FUNCTIONS: ModeAutoLand(), _enter(), update(), navigate(),
  check_takeoff_direction(), set_autoland_direction(), landing_lined_up(),
  arm_check(), is_landing()
MERIDIAN EQUIVALENT: None (FixedWingModes has update_land() which is slope-landing only)
PARITY STATUS: MISSING
GAPS: ModeAutoLand entirely absent from Meridian; runway alignment approach,
  autoland direction discovery, and arm_check() not present.

---

FILE: ArduPlane/mode_autotune.cpp
LINES: 23
PURPOSE: Plane autotune — delegates to TECS and attitude controller tuning
  subsystem while flying in stabilize-like mode.
KEY FUNCTIONS: _enter(), update(), run()
MERIDIAN EQUIVALENT: None specific; crates/meridian-autotune/src/tuner.rs not
  wired to fixed-wing mode
PARITY STATUS: STUB
GAPS: Plane autotune not wired; TECS autotune not present.

---

FILE: ArduPlane/mode_avoidADSB.cpp
LINES: 23
PURPOSE: ADSB avoidance for fixed-wing. Accepts velocity commands from AP_Avoidance,
  briefly overrides navigation.
KEY FUNCTIONS: _enter(), update(), navigate()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: No fixed-wing ADSB avoidance mode in Meridian.

---

FILE: ArduPlane/mode_circle.cpp
LINES: 23
PURPOSE: Fixed-wing circle around a point (used by ATC hold, failsafe). Delegates
  to L1 controller.
KEY FUNCTIONS: _enter(), update()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_circle()
PARITY STATUS: PARTIAL
GAPS: Circle center set from last loiter target only; radius from parameter;
  entry direction preference not modeled; altitude target update absent.

---

FILE: ArduPlane/mode_cruise.cpp
LINES: 97
PURPOSE: Cruise mode — pilot controls heading via roll input; aircraft maintains
  altitude and selected speed/heading. Target heading locked when stick centered.
KEY FUNCTIONS: _enter(), update(), navigate(), get_target_heading_cd()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_cruise()
PARITY STATUS: PARTIAL
GAPS: get_target_heading_cd() locked-heading save/restore on stick release not
  fully modeled; airspeed target from cruise throttle not wired to TECS;
  navigate() L1 target not dispatched.

---

FILE: ArduPlane/mode_fbwa.cpp
LINES: 45
PURPOSE: Fly-by-wire A — roll/pitch stabilized, manual throttle. Limits applied.
KEY FUNCTIONS: update(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_fbwa()
PARITY STATUS: FULL
GAPS: None material for core control. Aerobatic option bit not checked.

---

FILE: ArduPlane/mode_fbwb.cpp
LINES: 24
PURPOSE: Fly-by-wire B — altitude hold + speed control via throttle, roll-to-turn.
KEY FUNCTIONS: _enter(), update()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_fbwb()
PARITY STATUS: PARTIAL
GAPS: TECS integration (speed/altitude trade) not present; fly-by-wire B
  altitude reference drift absent.

---

FILE: ArduPlane/mode_guided.cpp
LINES: 202
PURPOSE: Fixed-wing guided — accepts Location target, optionally airspeed setpoint,
  loiter radius/direction. Handles GCS target updates, altitude=-1 passthrough.
KEY FUNCTIONS: _enter(), update(), navigate(), handle_guided_request(),
  handle_change_airspeed(), set_radius_and_direction(),
  target_location_alt_is_minus_one(), update_target_altitude()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_guided()
PARITY STATUS: PARTIAL
GAPS: handle_change_airspeed() runtime airspeed override absent; altitude=-1
  "use current alt" passthrough not modeled; set_radius_and_direction() runtime
  loiter geometry change absent.

---

FILE: ArduPlane/mode_loiter.cpp
LINES: 161
PURPOSE: Fixed-wing loiter around a point. Checks heading alignment for transitions.
  Pilot can request loiter expansion.
KEY FUNCTIONS: _enter(), update(), isHeadingLinedUp(),
  isHeadingLinedUp_cd(), navigate(), update_target_altitude()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_loiter()
PARITY STATUS: PARTIAL
GAPS: isHeadingLinedUp() alignment check for landing sequence transition absent;
  pilot loiter-radius expansion absent; altitude hold via TECS not present.

---

FILE: ArduPlane/mode_manual.cpp
LINES: 31
PURPOSE: Full manual passthrough — RC input directly to servos, no stabilization.
KEY FUNCTIONS: update(), run(), use_throttle_limits()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_manual()
PARITY STATUS: FULL
GAPS: None material.

---

FILE: ArduPlane/mode_qacro.cpp
LINES: 79
PURPOSE: QuadPlane acro — full rate control on VTOL motors/surfaces. Similar to
  copter acro but within quadplane framework.
KEY FUNCTIONS: _enter(), update(), run()
MERIDIAN EQUIVALENT: None (VtolTransitionToFw/Hover stubs only)
PARITY STATUS: MISSING
GAPS: Q-mode suite (QAcro, QHover, QLoiter, QStabilize, QRTL, QLand, QAutotune,
  LoiterAltQLand) entirely absent. Quadplane mode dispatch not present.

---

FILE: ArduPlane/mode_qautotune.cpp
LINES: 50
PURPOSE: QuadPlane autotune — tunes VTOL attitude controllers.
KEY FUNCTIONS: _enter(), update(), run(), _exit()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: See QAcro above.

---

FILE: ArduPlane/mode_qhover.cpp
LINES: 58
PURPOSE: QuadPlane altitude-hold hover mode using VTOL motors.
KEY FUNCTIONS: _enter(), update(), run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: See QAcro above.

---

FILE: ArduPlane/mode_qland.cpp
LINES: 35
PURPOSE: QuadPlane vertical landing sequence using VTOL motors.
KEY FUNCTIONS: _enter(), update(), run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: See QAcro above.

---

FILE: ArduPlane/mode_qloiter.cpp
LINES: 187
PURPOSE: QuadPlane position-hold loiter using VTOL hover. Integrates AC_Loiter.
KEY FUNCTIONS: _enter(), update(), run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: See QAcro above.

---

FILE: ArduPlane/mode_qrtl.cpp
LINES: 229
PURPOSE: QuadPlane RTL — returns to home as fixed-wing, transitions to hover
  for final vertical landing.
KEY FUNCTIONS: _enter(), update(), run(), update_target_altitude(),
  allows_throttle_nudging(), get_VTOL_return_radius()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: See QAcro above.

---

FILE: ArduPlane/mode_qstabilize.cpp
LINES: 103
PURPOSE: QuadPlane stabilize hover, manual throttle. Supports tailsitter-specific
  roll/pitch limiting.
KEY FUNCTIONS: _enter(), update(), run(), set_tailsitter_roll_pitch(),
  set_limited_roll_pitch()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: See QAcro above.

---

FILE: ArduPlane/mode_rtl.cpp
LINES: 169
PURPOSE: Fixed-wing RTL — climbs to safe altitude, flies home, loiters.
  Can switch to QRTL if quadplane and within transition distance.
KEY FUNCTIONS: _enter(), update(), navigate(), switch_QRTL()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_rtl()
PARITY STATUS: PARTIAL
GAPS: switch_QRTL() auto-transition to VTOL landing not present; rally point
  return target not used; TECS altitude management not connected.

---

FILE: ArduPlane/mode_stabilize.cpp
LINES: 18
PURPOSE: Fixed-wing stabilize — roll/pitch stabilized, manual throttle.
KEY FUNCTIONS: update(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_stabilize()
PARITY STATUS: FULL
GAPS: None material.

---

FILE: ArduPlane/mode_takeoff.cpp
LINES: 206
PURPOSE: Automated takeoff — full throttle rotation, pitch-up, climb to safe
  altitude, then transition to next mode. Runway or VTOL depending on config.
KEY FUNCTIONS: ModeTakeoff(), _enter(), update(), navigate()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_takeoff()
PARITY STATUS: PARTIAL
GAPS: Runway rolling takeoff (ground steering, rotation speed) not fully modeled;
  crosswind correction on takeoff roll absent; VTOL takeoff path not dispatched
  from Meridian takeoff mode.

---

FILE: ArduPlane/mode_thermal.cpp
LINES: 150
PURPOSE: Soaring / thermal mode — detects thermals and circles to gain altitude.
  Uses lift estimate from EKF-like estimator. Exits when exit conditions met.
KEY FUNCTIONS: _enter(), update(), update_soaring(), navigate(),
  exit_heading_aligned(), restore_mode()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs →
  FlightModeId::Thermal (stub → update_loiter)
PARITY STATUS: STUB
GAPS: Entire soaring/thermal lift estimator absent; thermal detection algorithm
  absent; exit heading alignment absent; no soaring parameters.

---

FILE: ArduPlane/mode_training.cpp
LINES: 71
PURPOSE: Training mode — limits pilot deflection, provides partial stabilization.
  Useful for student pilots.
KEY FUNCTIONS: update(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs →
  FlightModeId::Training (stub → update_stabilize)
PARITY STATUS: STUB
GAPS: Deflection limits not applied; no partial stabilization blend.

---

## ArduPlane — VTOL / Tailsitter

---

FILE: ArduPlane/quadplane.cpp
LINES: 4976
PURPOSE: Core QuadPlane VTOL logic. VTOL motor management, FW↔VTOL transition
  state machines (SLT_Transition), Q_ASSIST, motor output during hybrid flight,
  landing descent rate, throttle suppression, XY position controller for VTOL,
  is_flying/is_flying_vtol detection, poscontrol_init_approach, altitude hold
  during VTOL, pilot throttle input, weathervaning.
KEY FUNCTIONS: setup(), run_esc_calibration(), multicopter_attitude_rate_update(),
  hold_stabilize(), run_z_controller(), relax_attitude_control(),
  set_climb_rate_ms(), hold_hover(), get_pilot_throttle(),
  get_pilot_desired_lean_angles(), landing_descent_rate_ms(),
  get_desired_yaw_rate_cds(), SLT_Transition::update(), SLT_Transition::VTOL_update(),
  update(), update_throttle_suppression(), motors_output(),
  handle_do_vtol_transition(), in_vtol_auto(), in_vtol_mode(),
  in_vtol_posvel_mode(), update_land_positioning(), run_xy_controller(),
  poscontrol_init_approach()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs →
  update_vtol_transition() (stub)
PARITY STATUS: STUB
GAPS: Entire QuadPlane subsystem is a 4,976-line system. Meridian has only a
  transition stub that returns a loiter target. Missing: SLT/Tiltrotor/Tailsitter
  transition state machines, Q_ASSIST logic, VTOL motor output blending,
  throttle suppression, XY controller for VTOL, landing descent rate schedule,
  weathervaning in VTOL, poscontrol_init_approach, all Q-mode implementations.

---

FILE: ArduPlane/tailsitter.cpp
LINES: 1060
PURPOSE: Tailsitter sub-system. Controls transition angle, speed scaling, input
  remapping for tailsitter geometry, FW/VTOL transition completion detection,
  pitch relax, Tailsitter_Transition state machine.
KEY FUNCTIONS: setup(), is_control_surface_tailsitter(), active(), output(),
  transition_fw_complete(), transition_vtol_complete(), check_input(),
  in_vtol_transition(), is_in_fw_flight(), get_transition_angle_vtol(),
  speed_scaling(), Tailsitter_Transition::update(), VTOL_update(),
  show_vtol_view(), set_FW_roll_pitch(), allow_stick_mixing(),
  set_VTOL_roll_pitch_limit(), restart(), force_transition_complete(),
  allow_weathervane()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Tailsitter entirely absent from Meridian. No input remapping, transition
  angle detection, speed scaling, or tailsitter-specific transition state machine.

---

## ArduPlane — Safety / Arming Files

---

FILE: ArduPlane/AP_Arming_Plane.cpp
LINES: 477
PURPOSE: Plane-specific arming. Terrain DB required check, pre-arm checks
  (TECS, airspeed, fence, mission), quadplane-specific checks (motor health,
  VTOL ready), INS, arm sequence, disarm, soft-armed state management.
KEY FUNCTIONS: terrain_database_required(), pre_arm_checks(), mandatory_checks(),
  quadplane_checks(), ins_checks(), arm_checks(), change_arm_state(), arm(),
  disarm(), update_soft_armed(), mission_checks(), rc_received_if_enabled_check()
MERIDIAN EQUIVALENT: crates/meridian-arming/src/lib.rs
PARITY STATUS: PARTIAL
GAPS: quadplane_checks() not present; soft-armed state (motors spin at idle
  before full arm) not modeled; TECS pre-arm check absent; airspeed calibration
  check absent; rc_received_if_enabled check absent.

---

FILE: ArduPlane/failsafe.cpp
LINES: 115
PURPOSE: Plane periodic failsafe check: monitors GCS link timeout, triggers
  failsafe if no RC and in manual mode.
KEY FUNCTIONS: failsafe_check()
MERIDIAN EQUIVALENT: crates/meridian-failsafe/src/lib.rs (TimeoutMonitor)
PARITY STATUS: PARTIAL
GAPS: Plane-specific GCS-link check and RC-loss-in-manual behavior not differentiated
  from copter in Meridian.

---

FILE: ArduPlane/events.cpp
LINES: 361
PURPOSE: Plane failsafe event handlers: RC short/long, battery, manual GCS.
  QuadPlane-aware (can switch to QRTL instead of RTL on RC failsafe).
KEY FUNCTIONS: failsafe_in_landing_sequence(), rc_failsafe_short_on_event(),
  failsafe_long_on_event(), rc_failsafe_short_off_event(),
  failsafe_long_off_event(), handle_battery_failsafe()
MERIDIAN EQUIVALENT: crates/meridian-failsafe/src/lib.rs
PARITY STATUS: PARTIAL
GAPS: rc_failsafe_short_on_event() short-FS circle behavior absent;
  QuadPlane QRTL / FS_QRTL option in failsafe not modeled.

---

## Rover — Mode Files

---

FILE: Rover/mode.cpp
LINES: 569
PURPOSE: Rover mode base class. Pilot input (steering/throttle/speed/heading),
  lateral, walking-height, waypoint bearing, nav-bearing, crosstrack error,
  desired lateral acceleration, set_desired_location with lookahead.
KEY FUNCTIONS: Mode(), exit(), enter(), get_pilot_input(),
  get_pilot_desired_steering_and_throttle(), get_pilot_desired_steering_and_speed(),
  get_pilot_desired_lateral(), get_pilot_desired_heading_and_speed(),
  get_pilot_desired_roll_and_pitch(), get_pilot_desired_walking_height(),
  wp_bearing(), nav_bearing(), crosstrack_error_m(), get_desired_lat_accel(),
  set_desired_location()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs (RoverModes)
PARITY STATUS: PARTIAL
GAPS: get_pilot_desired_walking_height() (walking robot legs) absent;
  get_pilot_desired_roll_and_pitch() for tracked/boat absent; lateral output
  (crab/omni drive) absent from Meridian rover output.

---

FILE: Rover/mode_acro.cpp
LINES: 64
PURPOSE: Rover acro — direct steering/throttle passthrough with rate limiting.
  Handles sailing tack requests.
KEY FUNCTIONS: update(), requires_velocity(), handle_tack_request()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_acro()
PARITY STATUS: PARTIAL
GAPS: handle_tack_request() (sailing tack maneuver) absent; requires_velocity()
  flag not used in dispatch.

---

FILE: Rover/mode_auto.cpp
LINES: 1057
PURPOSE: Rover autonomous mission. Handles WP navigation, speed control,
  avoidance, OA dispatch, nav_script_time, calc_throttle, crosstrack,
  desired lat-accel, RTL start from mission end.
KEY FUNCTIONS: _enter(), _exit(), update(), calc_throttle(),
  wp_bearing(), nav_bearing(), crosstrack_error_m(), get_desired_lat_accel(),
  get_distance_to_destination(), get_desired_location(), set_desired_location(),
  reached_destination(), set_desired_speed(), start_RTL(), nav_script_time()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_auto()
PARITY STATUS: PARTIAL
GAPS: OA avoidance integration (avoidance_enabled in calc_throttle) absent;
  nav_script_time Lua callback absent; set_desired_speed() mid-mission speed
  change not wired; calc_throttle braking distance not present; pivot-turn
  logic for skid-steer absent.

---

FILE: Rover/mode_circle.cpp
LINES: 322
PURPOSE: Rover circle around a center point. drive-to-radius phase then circling.
  Configurable speed, radius, CCW/CW.
KEY FUNCTIONS: ModeCircle(), get_reached_distance(), set_center(), _enter(),
  init_target_yaw_rad(), update(), update_drive_to_radius(), update_circling(),
  wp_bearing(), nav_bearing(), get_desired_lat_accel(), set_desired_speed(),
  get_desired_location(), check_config_speed(), check_config_radius()
MERIDIAN EQUIVALENT: None (FlightModeId::Circle maps to copter circle, not rover)
PARITY STATUS: MISSING
GAPS: Rover circle mode entirely absent from Meridian rover dispatch; no
  drive-to-radius approach; no rover-specific circling using L1/nav controller.

---

FILE: Rover/mode_dock.cpp
LINES: 265
PURPOSE: Precision docking — approaches a dock target using AP_Follow with
  slowdown profile. Requires DOCK_SPEED and proximity sensor.
KEY FUNCTIONS: ModeDock(), enabled(), _enter(), update(), apply_slowdown(),
  calc_dock_pos_rel_vehicle_NE_m()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs →
  FlightModeId::RoverDock (stub → update_guided per GAP 29)
PARITY STATUS: STUB
GAPS: apply_slowdown() speed ramp not present; proximity-sensor-based docking
  stop not present; calc_dock_pos_rel_vehicle_NE_m beacon offset absent.

---

FILE: Rover/mode_follow.cpp
LINES: 114
PURPOSE: Rover follow — tracks another vehicle via AP_Follow. Maintains offset.
KEY FUNCTIONS: enabled(), _enter(), _exit(), update(), wp_bearing(),
  get_distance_to_destination(), set_desired_speed()
MERIDIAN EQUIVALENT: None (FlightModeId::Follow in rover is not dispatched)
PARITY STATUS: MISSING
GAPS: Rover follow mode not dispatched in Meridian rover; AP_Follow not
  connected to rover control loop.

---

FILE: Rover/mode_guided.cpp
LINES: 447
PURPOSE: Rover guided — accepts Location, heading+speed, turn-rate+speed, or
  steering+throttle commands from GCS.
KEY FUNCTIONS: _enter(), update(), wp_bearing(), nav_bearing(),
  crosstrack_error_m(), get_desired_lat_accel(), get_distance_to_destination(),
  reached_destination(), set_desired_speed(), get_desired_location(),
  set_desired_location(), set_desired_heading_and_speed(),
  set_desired_heading_delta_and_speed(), set_desired_turn_rate_and_speed(),
  set_steering_and_throttle()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_guided()
PARITY STATUS: PARTIAL
GAPS: set_desired_heading_and_speed() heading-hold navigation not fully implemented;
  set_desired_turn_rate_and_speed() rate-control path absent;
  set_steering_and_throttle() direct override absent; OA avoidance in guided
  not wired.

---

FILE: Rover/mode_hold.cpp
LINES: 18
PURPOSE: Rover hold — stops and brakes.
KEY FUNCTIONS: update()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_hold()
PARITY STATUS: FULL
GAPS: None material.

---

FILE: Rover/mode_loiter.cpp
LINES: 80
PURPOSE: Rover loiter around a GPS point — L1 controller circles.
KEY FUNCTIONS: _enter(), update(), get_desired_location()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_loiter()
PARITY STATUS: PARTIAL
GAPS: L1 controller integration for rover loiter not present; Meridian returns
  a hold-position target only.

---

FILE: Rover/mode_manual.cpp
LINES: 38
PURPOSE: Rover manual passthrough — RC directly to drive outputs.
KEY FUNCTIONS: _exit(), update()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_manual()
PARITY STATUS: FULL
GAPS: None material.

---

FILE: Rover/mode_rtl.cpp
LINES: 84
PURPOSE: Rover RTL — drives back to home.
KEY FUNCTIONS: _enter(), update(), get_desired_location(), reached_destination(),
  set_desired_speed()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_rtl()
PARITY STATUS: PARTIAL
GAPS: Rally point selection not used; set_desired_speed() not wired; OA not
  active in RTL.

---

FILE: Rover/mode_simple.cpp
LINES: 32
PURPOSE: Simple mode — compass-independent pilot-relative heading reference.
KEY FUNCTIONS: init_heading(), update()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Compass-independent simple mode not present in Meridian rover.

---

FILE: Rover/mode_smart_rtl.cpp
LINES: 144
PURPOSE: Rover SmartRTL — retraces recorded path back to home.
KEY FUNCTIONS: enabled(), _enter(), update(), get_desired_location(),
  set_desired_speed(), save_position()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_smartrtl()
PARITY STATUS: PARTIAL
GAPS: save_position() not connected to flight loop; SmartRTL path used via
  smartrtl_index only (simplified); background cleanup not wired.

---

FILE: Rover/mode_steering.cpp
LINES: 54
PURPOSE: Steering mode — pilot controls heading rate, speed from throttle.
KEY FUNCTIONS: update()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/rover.rs → update_steering()
PARITY STATUS: FULL
GAPS: None material.

---

## Rover — Safety / Arming Files

---

FILE: Rover/AP_Arming_Rover.cpp
LINES: 254
PURPOSE: Rover arming checks: RC calibration, GPS, pre-arm, arm sequence,
  disarm, soft-armed update, OA check, parameter checks, mode checks, motor checks.
KEY FUNCTIONS: rc_calibration_checks(), gps_checks(), pre_arm_checks(),
  arm_checks(), update_soft_armed(), arm(), disarm(), oa_check(), 
  parameter_checks(), mode_checks(), motor_checks()
MERIDIAN EQUIVALENT: crates/meridian-arming/src/lib.rs
PARITY STATUS: PARTIAL
GAPS: mode_checks() (e.g. disallow arm in Hold while moving) absent;
  oa_check() OA health gate absent; motor_checks() brushed motor check absent.

---

FILE: Rover/failsafe.cpp
LINES: 170
PURPOSE: Rover failsafe: periodic check, trigger/untrigger state machine,
  battery failsafe, AFS check.
KEY FUNCTIONS: failsafe_check(), failsafe_trigger(), handle_battery_failsafe(),
  afs_fs_check()
MERIDIAN EQUIVALENT: crates/meridian-failsafe/src/lib.rs
PARITY STATUS: PARTIAL
GAPS: afs_fs_check() absent; failsafe_trigger() with HOLD/RTL/SmartRTL cascades
  not rover-specific in Meridian.

---

## ArduSub — Mode Files

---

FILE: ArduSub/mode.cpp
LINES: 300
PURPOSE: Sub mode base class, mode-switch, exit, pilot angle-rate computation,
  set_mode wrapper.
KEY FUNCTIONS: Mode(), mode_from_mode_num(), set_mode(), exit_mode(),
  update_flight_mode(), notify_flight_mode(), get_pilot_desired_angle_rates()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs (SubModes)
PARITY STATUS: PARTIAL
GAPS: get_pilot_desired_angle_rates() not independently wired; sub-specific
  notification absent.

---

FILE: ArduSub/mode_acro.cpp
LINES: 42
PURPOSE: Sub acro — full rate control on all axes.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs → update_sub_acro()
PARITY STATUS: FULL
GAPS: None material.

---

FILE: ArduSub/mode_althold.cpp
LINES: 130
PURPOSE: Sub depth hold — maintains depth using barometer/depth sensor.
  run_pre/run_post split for vectored thrusters.
KEY FUNCTIONS: init(), run(), run_pre(), run_post(), control_depth()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs → update_depth_hold()
PARITY STATUS: PARTIAL
GAPS: run_pre() / run_post() vectored thruster split not modeled; control_depth()
  depth error integration not as sophisticated as ArduSub; leak detector
  integration absent.

---

FILE: ArduSub/mode_auto.cpp
LINES: 580
PURPOSE: Sub autonomous mission. Waypoint, circle, guided, loiter sub-modes.
  Yaw ROI, yaw-rate setting.
KEY FUNCTIONS: init(), run(), auto_wp_start(), auto_wp_run(), auto_circle_movetoedge_start(),
  auto_circle_start(), auto_circle_run(), auto_nav_guided_start(),
  auto_nav_guided_run(), auto_loiter_start(), auto_loiter_run(),
  set_auto_yaw_look_at_heading(), set_yaw_rate(), set_auto_yaw_roi()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs →
  FlightModeId::SubAuto (stub → update_stabilize)
PARITY STATUS: STUB
GAPS: Entire sub auto mission state machine absent; no circle, guided, loiter
  sub-mode for sub; ROI yaw control absent.

---

FILE: ArduSub/mode_circle.cpp
LINES: 91
PURPOSE: Sub circle around a point with depth hold.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: None specific
PARITY STATUS: STUB
GAPS: Sub circle not dispatched in Meridian sub modes.

---

FILE: ArduSub/mode_guided.cpp
LINES: 888
PURPOSE: Sub guided — position, velocity, posvel, angle target acceptance.
  Yaw control, yaw-rate control. Full 3D target types.
KEY FUNCTIONS: init(), guided_pos_control_start(), guided_vel_control_start(),
  guided_posvel_control_start(), guided_angle_control_start(),
  guided_set_destination(), guided_set_velocity(), guided_set_destination_posvel(),
  guided_set_angle(), guided_set_yaw_state(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs →
  FlightModeId::SubGuided (stub → update_stabilize)
PARITY STATUS: STUB
GAPS: All sub guided sub-modes absent; 888-line implementation not ported.

---

FILE: ArduSub/mode_manual.cpp
LINES: 35
PURPOSE: Sub manual passthrough — RC to thrusters.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs → update_manual()
PARITY STATUS: FULL
GAPS: None material for core passthrough.

---

FILE: ArduSub/mode_motordetect.cpp
LINES: 174
PURPOSE: Motor detection sequence — spins each motor in turn to detect
  direction and mapping.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Motor detection mode entirely absent from Meridian.

---

FILE: ArduSub/mode_poshold.cpp
LINES: 134
PURPOSE: Sub 3D position hold using GPS/USBL.
KEY FUNCTIONS: init(), run(), control_horizontal()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs →
  FlightModeId::SubPosHold (stub → update_depth_hold)
PARITY STATUS: STUB
GAPS: control_horizontal() 2D position controller for sub absent; only depth
  hold is functional.

---

FILE: ArduSub/mode_stabilize.cpp
LINES: 69
PURPOSE: Sub stabilize — attitude-stabilized, manual throttle.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs → update_stabilize()
PARITY STATUS: FULL
GAPS: None material.

---

FILE: ArduSub/mode_surface.cpp
LINES: 64
PURPOSE: Sub surface emergency mode — full throttle ascent to surface.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Surface emergency mode absent from Meridian.

---

FILE: ArduSub/mode_surftrak.cpp
LINES: 176
PURPOSE: Surface tracking — maintains constant distance from bottom using
  rangefinder, updating surface offset.
KEY FUNCTIONS: ModeSurftrak(), init(), run(), set_rangefinder_target_cm(),
  reset(), control_range(), update_surface_offset()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs →
  FlightModeId::SurfTrak (stub → update_depth_hold)
PARITY STATUS: STUB
GAPS: control_range() rangefinder feedback loop absent; update_surface_offset()
  sea-floor tracking absent; set_rangefinder_target_cm() configurable target absent.

---

## Blimp — Mode Files

---

FILE: Blimp/mode.cpp
LINES: 191
PURPOSE: Blimp mode base class, mode dispatch, pilot input (Vector3f + yaw),
  is_disarmed_or_landed, navigation update.
KEY FUNCTIONS: Mode(), mode_from_mode_num(), set_mode(), update_flight_mode(),
  exit_mode(), notify_flight_mode(), update_navigation(), get_pilot_input(),
  is_disarmed_or_landed()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Blimp vehicle class entirely absent from Meridian. No blimp mode dispatch.

---

FILE: Blimp/mode_land.cpp
LINES: 23
PURPOSE: Blimp land — slow descent to ground using fins.
KEY FUNCTIONS: ModeLand::run(), set_mode_land_failsafe()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

FILE: Blimp/mode_loiter.cpp
LINES: 49
PURPOSE: Blimp loiter — GPS position hold using fin thrust.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

FILE: Blimp/mode_manual.cpp
LINES: 16
PURPOSE: Blimp manual — fin thrust direct from RC.
KEY FUNCTIONS: run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

FILE: Blimp/mode_rtl.cpp
LINES: 20
PURPOSE: Blimp RTL — returns to home using fins.
KEY FUNCTIONS: init(), run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

FILE: Blimp/mode_velocity.cpp
LINES: 22
PURPOSE: Blimp velocity mode — pilot commands body-frame velocity via RC.
KEY FUNCTIONS: run()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

## Blimp — Safety / Arming Files

---

FILE: Blimp/AP_Arming_Blimp.cpp
LINES: 418
PURPOSE: Blimp-specific arming: barometer, INS, board voltage, parameters,
  motor checks, RC calibration, GPS, EKF attitude, mandatory GPS, GCS failsafe,
  altitude check, arm/disarm.
KEY FUNCTIONS: pre_arm_checks(), run_pre_arm_checks(), barometer_checks(),
  ins_checks(), board_voltage_checks(), parameter_checks(), motor_checks(),
  rc_calibration_checks(), gps_checks(), pre_arm_ekf_attitude_check(),
  mandatory_gps_checks(), gcs_failsafe_check(), alt_checks(), arm_checks(),
  mandatory_checks()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

FILE: Blimp/failsafe.cpp
LINES: 73
PURPOSE: Blimp failsafe: enable/disable, periodic check.
KEY FUNCTIONS: failsafe_enable(), failsafe_disable(), failsafe_check()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

FILE: Blimp/events.cpp
LINES: 175
PURPOSE: Blimp failsafe event handlers: radio, battery, GCS, GPS glitch,
  disarm logic.
KEY FUNCTIONS: failsafe_option(), failsafe_radio_on_event(),
  failsafe_radio_off_event(), handle_battery_failsafe(), failsafe_gcs_check(),
  should_disarm_on_failsafe(), do_failsafe_action(), gpsglitch_check()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING

---

## Libraries — Mission Execution

---

FILE: libraries/AP_Mission/AP_Mission.cpp
LINES: 3199
PURPOSE: Core mission sequencer. Stores commands in EEPROM/flash, executes
  nav/do/conditional commands, handles jump/conditional jump, waypoint advance,
  mission resume, terrain following within mission, start_command / verify_command
  dispatch, MAVLink import/export.
KEY FUNCTIONS: init(), start(), stop(), resume(), is_takeoff_next(),
  starts_with_takeoff_cmd(), continue_after_land_check_for_takeoff(),
  start_or_resume(), reset(), clear(), truncate(), update(),
  verify_command(), start_command(), add_cmd(), replace_cmd(), is_nav_cmd(),
  get_next_nav_cmd(), get_next_ground_course_cd(), set_current_cmd(),
  restart_current_nav_cmd(), set_item(), get_item(), read_cmd_from_storage(),
  stored_in_location(), write_cmd_to_storage(), write_home_to_storage(),
  mission_cmd_to_mavlink_int(), complete()
MERIDIAN EQUIVALENT: crates/meridian-mission/src/lib.rs (MissionExecutor)
PARITY STATUS: PARTIAL
GAPS: MissionExecutor has 128-item fixed-size list and BehaviorTree — no EEPROM
  storage; no mission upload/download via MAVLink (get_item/set_item);
  verify_command() loop with conditional-jump not present; terrain-frame
  altitude resolution absent; DO_JUMP, DO_JUMP_TAG, DO_CONDITION_CHANGE absent;
  start_command for all DO_ commands (gripper, sprayer, camera, relay, gimbal,
  fence, scripting) not implemented; mission change detector absent.

---

FILE: libraries/AP_Mission/AP_Mission_ChangeDetector.cpp
LINES: 62
PURPOSE: Detects mission changes mid-flight by hashing waypoints and comparing.
KEY FUNCTIONS: check_for_mission_change()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: No mission change detection in Meridian.

---

FILE: libraries/AP_Mission/AP_Mission_Commands.cpp
LINES: 377
PURPOSE: DO command handlers: aux function, gripper, servo/relay, camera,
  parachute, repeat-distance, sprayer, scripting, gimbal pitch/yaw, fence.
KEY FUNCTIONS: start_command_do_aux_function(), start_command_do_gripper(),
  start_command_do_servorelayevents(), start_command_camera(),
  start_command_parachute(), command_do_set_repeat_dist(),
  start_command_do_sprayer(), start_command_do_scripting(),
  start_command_do_gimbal_manager_pitchyaw(), start_command_fence()
MERIDIAN EQUIVALENT: Partial — crates/meridian-gripper/src/lib.rs,
  crates/meridian-sprayer/src/lib.rs, crates/meridian-camera/src/lib.rs exist
  but are not wired through mission command dispatch.
PARITY STATUS: PARTIAL
GAPS: mission command dispatch chain to payload crates not wired; scripting
  Lua DO commands absent; relay/servo DO commands absent;
  fence enable/disable via DO_FENCE_ENABLE absent.

---

## Libraries — Arming Checks

---

FILE: libraries/AP_Arming/AP_Arming.cpp
LINES: 2189
PURPOSE: Base arming check library. All vehicle-common checks: barometer, airspeed,
  logging, INS accel/gyro consistency, compass, GPS, battery, hardware safety,
  RC arm/calibration, mission, rangefinder, servo, board voltage, heater.
  CrashDump check. Enabled-check bitmask.
KEY FUNCTIONS: AP_Arming(), update(), CrashDump::check_reset(),
  compass_magfield_expected(), is_armed(), is_armed_and_safety_off(),
  get_enabled_checks(), check_enabled(), all_checks_enabled(), check_failed(),
  barometer_checks(), airspeed_checks(), logging_checks(), ins_accels_consistent(),
  ins_gyros_consistent(), ins_checks(), compass_checks(), gps_checks(),
  battery_checks(), hardware_safety_check(), rc_arm_checks(),
  rc_calibration_checks(), rc_in_calibration_check(), manual_transmitter_checks(),
  mission_checks(), rangefinder_checks(), servo_checks(), board_voltage_checks(),
  heater_min_temperature_checks()
MERIDIAN EQUIVALENT: crates/meridian-arming/src/lib.rs (check_prearm, ArmingState)
PARITY STATUS: PARTIAL
GAPS: Per-check enabled bitmask (ARMING_CHECK parameter) not present;
  check_enabled() gating absent; compass_magfield_expected() dynamic threshold
  absent; logging_checks() data-flash health absent; servo_checks() servo
  output range validation absent; heater_min_temperature_checks() absent;
  CrashDump check absent; hardware_safety_check() hardware safety switch absent;
  mission_checks() (first-WP altitude, DO_LAND_START) absent;
  rangefinder_checks() sensor health absent.

---

FILE: libraries/AP_Arming/AP_Arming.cpp (vehicle-specific overrides)
LINES: (see vehicle files above)
NOTE: Each vehicle provides AP_Arming_Copter, AP_Arming_Plane, AP_Arming_Rover,
  AP_Arming_Sub, AP_Arming_Blimp with vehicle-specific overrides. All noted
  in respective vehicle sections above.

---

## Libraries — Geofence

---

FILE: libraries/AC_Fence/AC_Fence.cpp
LINES: 1270
PURPOSE: Geofence enforcement. Supports altitude min/max, circle radius, polygon
  inclusion/exclusion. Auto-enable on arm, auto-disable on disarm. Breach record,
  margin tracking, terrain database check, destination check, pre-arm validation.
KEY FUNCTIONS: AC_Fence(), get_fence_names(), print_fence_message(), update(),
  enable(), enable_floor(), disable_floor(), auto_enable_fence_on_arming(),
  auto_disable_fence_on_disarming(), auto_enable_fence_after_takeoff(),
  get_auto_disable_fences(), present(), get_enabled_fences(), get_margin_ne_m(),
  pre_arm_check_polygon(), pre_arm_check_circle(), pre_arm_check_alt(),
  terrain_database_required(), pre_arm_check(), get_alt_in_frame_m(),
  update_safe_alt_min(), check_fence_alt_max(), check_fence_alt_min(),
  auto_enable_fence_floor(), check_fence_polygon(), check_fence_circle(), check(),
  check_destination_within_fence(), record_breach(), record_margin_breach()
MERIDIAN EQUIVALENT: crates/meridian-fence/src/lib.rs (Geofence, 141 lines)
PARITY STATUS: PARTIAL
GAPS: Meridian fence is minimal (141 lines vs 1270): no auto-enable/disable on
  arm/disarm; no terrain-relative altitude fence; no polygon exclusion zones
  (only inclusion polygon with 16 vertices); no margin tracking; no destination
  pre-check; no pre_arm_check(); no breach recording with time; no floor fence
  (min altitude); AC_PolyFence_loader (EEPROM persistence) entirely absent.

---

FILE: libraries/AC_Fence/AC_PolyFence_loader.cpp
LINES: 1719
PURPOSE: Polygon fence EEPROM storage, index, load, validate, write. Supports
  multiple inclusion and exclusion polygons plus exclusion circles.
KEY FUNCTIONS: init(), find_index_for_seq(), find_storage_offset_for_seq(),
  get_item(), write_type_to_storage(), write_latlon_to_storage(),
  read_latlon_from_storage(), breached(), formatted(), max_items(), format(),
  scan_eeprom(), count_eeprom_fences(), index_eeprom(), check_indexed(), unload(),
  index_fence_count(), load_from_storage(), get_exclusion_circle(),
  get_inclusion_circle(), check_inclusion_circle_margin(), validate_fence(),
  fence_storage_space_required(), write_fence()
MERIDIAN EQUIVALENT: None (no persistent fence storage)
PARITY STATUS: MISSING
GAPS: No EEPROM/flash fence storage; no multiple fence zone support; no MAVLink
  fence upload protocol; no exclusion circle or polygon support.

---

## Libraries — Rally Points

---

FILE: libraries/AP_Rally/AP_Rally.cpp
LINES: 209
PURPOSE: Rally point storage and nearest-point lookup. Stores up to 10 rally
  points in EEPROM. Returns nearest valid rally point for RTL target.
KEY FUNCTIONS: AP_Rally(), get_rally_point_with_index(),
  get_rally_location_with_index(), truncate(), append(),
  set_rally_point_with_index(), find_nearest_rally_point()
MERIDIAN EQUIVALENT: crates/meridian-nav/src/rally.rs (RallyManager, 196 lines)
PARITY STATUS: PARTIAL
GAPS: RallyManager has find_nearest() but no EEPROM persistence; no MAVLink
  rally upload/download protocol; rally point altitude frame (AGL vs AMSL) not
  handled; find_nearest_rally_point() altitude acceptance check absent.

---

## Libraries — SmartRTL Path Recording

---

FILE: libraries/AP_SmartRTL/AP_SmartRTL.cpp
LINES: 914
PURPOSE: Records flight path as 3D waypoints. Background thread simplifies path
  via Visvalingam–Whyatt and loop pruning. Supports thorough/routine cleanup,
  pop/peek points for RTL replay.
KEY FUNCTIONS: AP_SmartRTL(), enabled(), init(), get_num_points(), pop_point(),
  peek_point(), set_home(), update(), request_thorough_cleanup(),
  cancel_request_for_thorough_cleanup(), add_point(), run_background_cleanup(),
  routine_cleanup(), thorough_cleanup(), detect_simplifications(),
  detect_loops(), restart_simplify_if_new_points(), restart_pruning_if_new_points(),
  reset_simplification(), restart_pruning(), remove_points_by_simplify_bitmask(),
  remove_points_by_loops(), add_loop(), segment_segment_dist(), deactivate(),
  log_action()
MERIDIAN EQUIVALENT: crates/meridian-nav/src/smartrtl.rs (SmartRTL, 640 lines)
PARITY STATUS: PARTIAL
GAPS: Background cleanup thread not present (Meridian is single-threaded, so
  cleanup runs inline); request_thorough_cleanup() / cancel deferred cleanup
  interface absent; add_loop() three-point segment intersection absent;
  log_action() structured log entries absent; deactivate() with reason absent;
  routine vs thorough cleanup path distinction absent.

---

## Libraries — Waypoint Navigation

---

FILE: libraries/AC_WPNav/AC_WPNav.cpp
LINES: 1039
PURPOSE: Waypoint navigation controller. S-curve and linear path planning.
  Terrain-following. Spline waypoints. Speed/accel limits per axis. Stopping
  point computation. Distance/bearing to destination.
KEY FUNCTIONS: convert_parameters(), AC_WPNav(), get_terrain_source(),
  wp_and_spline_init_m(), set_speed_NE_cms/ms(), set_speed_up/down_ms(),
  set_wp_destination_loc(), set_wp_destination_next_loc(),
  get_wp_destination_loc(), set_wp_destination_NEU_cm(), set_wp_destination_NED_m(),
  set_wp_destination_next_NED_m(), get_wp_stopping_point_NE/NEU/NED(),
  advance_wp_target_along_track(), update_track_with_speed_accel_limits(),
  get_wp_distance_to_destination_cm/m(), get_wp_bearing_to_destination_cd/rad(),
  update_wpnav(), is_active(), force_stop_at_next_wp(), get_terrain_U/D_m(),
  set_spline_destination_loc()
MERIDIAN EQUIVALENT: crates/meridian-nav/src/waypoint.rs (WaypointNav, 212 lines);
  crates/meridian-nav/src/scurve.rs (SCurveSegment, 832 lines)
PARITY STATUS: PARTIAL
GAPS: Terrain-following during WP nav (get_terrain_U/D_m) not connected to WaypointNav;
  spline waypoint dispatch (set_spline_destination_loc) not present;
  next-waypoint lookahead (set_wp_destination_next_NED_m) not implemented;
  force_stop_at_next_wp() intermediate stop not present; is_active() timeout
  check absent; speed-update mid-flight (set_speed_NE_ms) not wired.

---

FILE: libraries/AC_WPNav/AC_WPNav_OA.cpp
LINES: 296
PURPOSE: WPNav with obstacle avoidance. Wraps AC_WPNav, substitutes OA-modified
  destination when OA path planner returns an alternate route.
KEY FUNCTIONS: get_oa_wp_destination(), set_wp_destination_NEU_cm(),
  set_wp_destination_NED_m(), get_wp_distance_to_destination_cm/m(),
  get_wp_bearing_to_destination_cd/rad(), reached_wp_destination(), update_wpnav()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: OA-wrapped waypoint navigation not present. No OA path planner hooks in
  Meridian WaypointNav.

---

FILE: libraries/AC_WPNav/AC_Loiter.cpp
LINES: 403
PURPOSE: Loiter position controller. Pilot desired acceleration from lean angles.
  Velocity braking, stopping point, angle-max, avoidance-velocity adjust,
  loiter options (slow down for fence, etc.).
KEY FUNCTIONS: AC_Loiter(), init_target_m(), init_target(), soften_for_landing(),
  set_pilot_desired_acceleration_cd/rad(), get_stopping_point_NE_m(),
  get_angle_max_cd/rad(), update(), set_speed_max_NE_ms(), convert_parameters(),
  sanity_check_params(), loiter_option_is_set(), calc_desired_velocity()
MERIDIAN EQUIVALENT: PositionController in crates/meridian-control/src/position_controller.rs
PARITY STATUS: PARTIAL
GAPS: soften_for_landing() (reduce braking near ground) absent; loiter_option_is_set()
  behavior flags absent; avoidance velocity adjustment in calc_desired_velocity absent;
  separate stopping-point geometry (get_stopping_point_NE_m) not exposed.

---

FILE: libraries/AC_WPNav/AC_Circle.cpp
LINES: 489
PURPOSE: Circle navigation controller. Sets center, radius, rate. Terrain-following
  circle. Closest point on circle. Velocity calculations.
KEY FUNCTIONS: AC_Circle(), init_NEU_cm(), init_NED_m(), init(), set_center(),
  set_rate_degs(), set_radius_cm/m(), is_active(), update_cms/ms(),
  get_closest_point_on_circle_NEU_cm(), get_closest_point_on_circle_NED_m(),
  calc_velocities(), init_start_angle(), get_terrain_source(), get_terrain_U_m(),
  check_param_change(), convert_parameters()
MERIDIAN EQUIVALENT: None dedicated (circle logic embedded in update_circle()
  within multirotor.rs and fixed_wing.rs)
PARITY STATUS: PARTIAL
GAPS: No separate circle controller library; terrain-following during circle
  absent; closest-point-on-circle geometry for move-to-edge absent; in-flight
  radius/rate change not present; start-angle computation absent.

---

## Libraries — Fixed-Wing Landing

---

FILE: libraries/AP_Landing/AP_Landing.cpp
LINES: 781
PURPOSE: Fixed-wing landing sequencer. Orchestrates approach, glide slope,
  flare, touchdown. Integrates rangefinder bump adjustment. Type dispatch
  (slope vs deepstall). Pre/post landing hooks.
KEY FUNCTIONS: AP_Landing(), loc_alt_AMSL_cm(), do_land(), verify_land(),
  verify_abort_landing(), adjust_landing_slope_for_rangefinder_bump(),
  send_landing_message(), is_flaring(), is_on_final(), is_on_approach(),
  is_ground_steering_allowed(), is_expecting_impact(), override_servos(),
  setup_landing_glide_slope(), reset(), restart_landing_sequence(),
  constrain_roll(), get_target_altitude_location(), get_target_airspeed_cm(),
  request_go_around(), handle_flight_stage_change(), is_complete(), Log(),
  is_throttle_suppressed(), use_thr_min_during_flare()
MERIDIAN EQUIVALENT: crates/meridian-modes/src/fixed_wing.rs → update_land()
PARITY STATUS: PARTIAL
GAPS: update_land() implements a simplified glide slope; is_flaring() / is_on_final()
  state machine not present; go-around (request_go_around) absent; rangefinder
  bump adjustment absent; restart_landing_sequence() abort and restart absent;
  throttle suppression during flare absent; slope vs deepstall type dispatch absent.

---

FILE: libraries/AP_Landing/AP_Landing_Deepstall.cpp
LINES: 667
PURPOSE: Deepstall landing — stalls aircraft above touchdown point for
  near-vertical descent. Includes steering update, approach path build,
  travel distance prediction, breakout detection.
KEY FUNCTIONS: do_land(), verify_abort_landing(), verify_land(),
  override_servos(), request_go_around(), is_throttle_suppressed(),
  is_flying_forward(), is_on_approach(), get_target_altitude_location(),
  get_target_airspeed_cm(), send_deepstall_message(), Log(), terminate(),
  build_approach_path(), predict_travel_distance(), verify_breakout(),
  update_steering()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Deepstall landing type entirely absent from Meridian.

---

FILE: libraries/AP_Landing/AP_Landing_Slope.cpp
LINES: 448
PURPOSE: Slope landing implementation for AP_Landing type dispatch.
KEY FUNCTIONS: type_slope_do_land(), type_slope_verify_abort_landing(),
  type_slope_verify_land(), type_slope_adjust_landing_slope_for_rangefinder_bump(),
  type_slope_request_go_around(), type_slope_setup_landing_glide_slope(),
  type_slope_get_target_airspeed_cm(), type_slope_constrain_roll(),
  type_slope_is_flaring(), type_slope_is_on_final(), type_slope_is_on_approach(),
  type_slope_is_expecting_impact(), type_slope_is_complete(), type_slope_log(),
  type_slope_is_throttle_suppressed()
MERIDIAN EQUIVALENT: Partial — crates/meridian-modes/src/fixed_wing.rs → update_land()
PARITY STATUS: PARTIAL
GAPS: Full slope state machine (approach → final → flare → touchdown) not present;
  type_slope_is_flaring() / is_on_final() stages not modeled; rangefinder bump
  correction absent; constrain_roll() roll limit during flare absent.

---

## Libraries — Follow Mode Support

---

FILE: libraries/AP_Follow/AP_Follow.cpp
LINES: 964
PURPOSE: Follow target library. Consumes GLOBAL_POSITION_INT and FOLLOW_TARGET
  MAVLink messages. Estimates target position/velocity/accel with dead reckoning.
  Computes offset position in body or NED frame. Heading estimation.
KEY FUNCTIONS: AP_Follow(), update_estimates(), get_target_pos_vel_accel_NED_m(),
  get_ofs_pos_vel_accel_NED_m(), get_target_dist_and_vel_NED_m(),
  get_heading_heading_rate_rad(), get_target_location_and_velocity(),
  get_target_location_and_velocity_ofs(), get_target_heading_deg(),
  get_target_heading_rate_degs(), handle_msg(), should_handle_message(),
  estimate_error_too_large(), calc_max_velocity_change(), handle_global_position_int_message(),
  handle_follow_target_message(), init_offsets_if_required(),
  clear_offsets_if_required(), clear_dist_and_bearing_to_target(),
  update_dist_and_bearing_to_target(), Log_Write_FOLL(), have_target()
MERIDIAN EQUIVALENT: None dedicated (FollowConfig struct in multirotor.rs with
  basic offset; no message handler)
PARITY STATUS: PARTIAL
GAPS: MAVLink GLOBAL_POSITION_INT / FOLLOW_TARGET message handler absent;
  dead-reckoning estimator absent; velocity/acceleration feed-forward absent;
  estimate_error_too_large() quality gate absent; body-frame vs NED offset
  selection absent; heading estimation absent; Log_Write_FOLL() absent.

---

## Libraries — Obstacle Avoidance (Proximity / OA)

---

FILE: libraries/AC_Avoidance/AC_Avoid.cpp
LINES: 1584
PURPOSE: Simple avoidance — adjusts velocity/acceleration commands to stay
  within fence boundaries and away from proximity obstacles. Handles circle fence,
  polygon fence, inclusion/exclusion zones, proximity sensor boundary, beacon fence.
  3D velocity limiting with backup velocity.
KEY FUNCTIONS: AC_Avoid(), convert_params(), adjust_velocity_fence(),
  adjust_velocity(), limit_accel_NEU_cm(), adjust_speed(), adjust_velocity_z(),
  adjust_roll_pitch_rad(), limit_velocity_NE(), limit_velocity_NEU(),
  calc_backup_velocity_2D/3D(), find_max_quadrant_velocity/3D(),
  get_max_speed(), adjust_velocity_circle_fence(),
  adjust_velocity_inclusion_and_exclusion_polygons(),
  adjust_velocity_inclusion_circles(), adjust_velocity_exclusion_circles(),
  adjust_velocity_beacon_fence(), adjust_velocity_proximity(),
  adjust_velocity_polygon(), get_stopping_distance(), distance_m_to_lean_norm()
MERIDIAN EQUIVALENT: crates/meridian-proximity/src/lib.rs → avoid_velocity(),
  avoid_velocity_full()
PARITY STATUS: PARTIAL
GAPS: Meridian proximity has sector/layer boundary model and avoid_velocity()
  but: fence-type avoidance not split (circle, polygon, inclusion, exclusion);
  beacon fence absent; backup velocity 3D quadrant logic simplified; adjust_speed()
  for rover absent; adjust_roll_pitch_rad() for fixed-angle vehicles absent;
  limit_accel_NEU_cm() acceleration-based limiting absent.

---

FILE: libraries/AC_Avoidance/AC_Avoidance_Logging.cpp
LINES: 100
PURPOSE: OA avoidance logging: BendyRuler log, Dijkstra log, visgraph point log,
  simple avoidance state log.
KEY FUNCTIONS: Write_OABendyRuler(), Write_OADijkstra(), Write_Visgraph_point(),
  Write_SimpleAvoidance()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: No OA-specific logging in Meridian.

---

FILE: libraries/AC_Avoidance/AP_OABendyRuler.cpp
LINES: 719
PURPOSE: Bendy Ruler path planner — steers around obstacles by searching angular
  paths at each step. Supports XY and vertical avoidance. Resist bearing change.
KEY FUNCTIONS: AP_OABendyRuler(), update(), search_xy_path(),
  search_vertical_path(), get_type(), resist_bearing_change(),
  calc_avoidance_margin(), calc_margin_from_circular_fence(),
  calc_margin_from_alt_fence(), calc_margin_from_inclusion_and_exclusion_polygons(),
  calc_margin_from_inclusion_and_exclusion_circles(),
  calc_margin_from_object_database()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: BendyRuler path planner entirely absent from Meridian.

---

FILE: libraries/AC_Avoidance/AP_OADatabase.cpp
LINES: 516
PURPOSE: Object avoidance database. Maintains time-stamped obstacle list from
  proximity sensors. Queues updates, removes expired items. GCS reporting.
KEY FUNCTIONS: AP_OADatabase(), init(), update(), queue_push(),
  init_queue(), init_database(), get_send_to_gcs_flags(), item_match(),
  process_queue(), database_item_add(), database_item_remove(),
  database_item_refresh(), database_items_remove_all_expired(),
  send_adsb_vehicle()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: OA obstacle database absent. Meridian proximity tracks sector/layer
  boundaries but has no persistent obstacle database.

---

FILE: libraries/AC_Avoidance/AP_OADijkstra.cpp
LINES: 1021
PURPOSE: Dijkstra graph path planner — uses fence vertices and obstacle
  database to build visibility graph, finds shortest path around obstacles.
KEY FUNCTIONS: AP_OADijkstra(), update(), some_fences_enabled(), report_error(),
  check_inclusion_polygon_updated(), create_inclusion_polygon_with_margin(),
  check_exclusion_polygon_updated(), create_exclusion_polygon_with_margin(),
  check_exclusion_circle_updated(), create_exclusion_circle_with_margin(),
  total_numpoints(), get_point(), intersects_fence(), create_fence_visgraph(),
  update_visgraph(), update_visible_node_distances(), find_node_from_id(),
  find_closest_node_idx(), calc_shortest_path(), get_shortest_path_point(),
  convert_node_to_point()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Dijkstra path planner entirely absent from Meridian.

---

FILE: libraries/AC_Avoidance/AP_OAPathPlanner.cpp
LINES: 440
PURPOSE: Path planner thread manager. Runs BendyRuler or Dijkstra in background
  thread, returns modified destination for WP navigation. Pre-arm check.
KEY FUNCTIONS: AP_OAPathPlanner(), init(), pre_arm_check(), start_thread(),
  map_bendytype_to_pathplannerused(), mission_avoidance(), avoidance_thread()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Background OA thread absent; mission_avoidance() intercept of WP destinations
  absent; pre_arm_check() for OA absent.

---

FILE: libraries/AC_Avoidance/AP_OAVisGraph.cpp
LINES: 47
PURPOSE: Visibility graph data structure used by Dijkstra planner.
KEY FUNCTIONS: AP_OAVisGraph(), add_item()
MERIDIAN EQUIVALENT: None
PARITY STATUS: MISSING
GAPS: Visibility graph absent.

---

## Libraries — ADSB Avoidance

---

FILE: libraries/AP_Avoidance/AP_Avoidance.cpp
LINES: 682
PURPOSE: ADSB-based collision avoidance. Tracks nearby ADSB vehicles, computes
  closest approach distance and threat level. Sends collision advisories to GCS.
  Triggers AVOID_ADSB mode when threat exceeds threshold.
KEY FUNCTIONS: init(), deinit(), check_startup(), add_obstacle(),
  src_id_for_adsb_vehicle(), get_adsb_samples(), closest_approach_NE_m(),
  closest_approach_D_m(), update_threat_level(), send_collision_all(),
  handle_threat_gcs_notify(), obstacle_is_more_serious_threat(),
  check_for_threats(), update(), handle_avoidance_local(), handle_msg(),
  get_vector_perpendicular()
MERIDIAN EQUIVALENT: crates/meridian-adsb/src/lib.rs
PARITY STATUS: PARTIAL
GAPS: Meridian ADSB library tracks threats and estimates CPA (closest point of
  approach) but: handle_avoidance_local() mode-switch trigger to AvoidADSB mode
  absent; send_collision_all() MAVLink COLLISION message absent; GCS advisory
  notification absent; get_vector_perpendicular() avoidance vector computation
  absent.

---

## Summary Gap Table

| ArduPilot File | Lines | Meridian Equivalent | Parity |
|---|---|---|---|
| ArduCopter/mode.cpp | 1136 | meridian-modes/multirotor.rs | PARTIAL |
| ArduCopter/mode_acro.cpp | 201 | multirotor::update_acro | PARTIAL |
| ArduCopter/mode_acro_heli.cpp | 118 | meridian-heli (unwired) | MISSING |
| ArduCopter/mode_althold.cpp | 104 | multirotor::update_althold | FULL |
| ArduCopter/mode_auto.cpp | 2448 | multirotor::update_auto + mission | PARTIAL |
| ArduCopter/mode_autorotate.cpp | 136 | meridian-heli (unwired) | PARTIAL |
| ArduCopter/mode_autotune.cpp | 124 | meridian-autotune (unwired) | PARTIAL |
| ArduCopter/mode_avoid_adsb.cpp | 42 | stub → update_loiter | STUB |
| ArduCopter/mode_brake.cpp | 88 | multirotor::update_brake | PARTIAL |
| ArduCopter/mode_circle.cpp | 143 | multirotor::update_circle | PARTIAL |
| ArduCopter/mode_drift.cpp | 165 | multirotor::update_drift | PARTIAL |
| ArduCopter/mode_flip.cpp | 217 | multirotor::update_flip | PARTIAL |
| ArduCopter/mode_flowhold.cpp | 531 | stub → update_loiter | STUB |
| ArduCopter/mode_follow.cpp | 165 | multirotor::update_follow | PARTIAL |
| ArduCopter/mode_guided.cpp | 1214 | multirotor::update_guided (pos only) | PARTIAL |
| ArduCopter/mode_guided_custom.cpp | 23 | None | MISSING |
| ArduCopter/mode_guided_nogps.cpp | 25 | update_guided_nogps | PARTIAL |
| ArduCopter/mode_land.cpp | 220 | multirotor::update_land | PARTIAL |
| ArduCopter/mode_loiter.cpp | 200 | multirotor::update_loiter | PARTIAL |
| ArduCopter/mode_poshold.cpp | 669 | multirotor::update_poshold | PARTIAL |
| ArduCopter/mode_rtl.cpp | 645 | multirotor::update_rtl | PARTIAL |
| ArduCopter/mode_smart_rtl.cpp | 222 | multirotor::update_smartrtl | PARTIAL |
| ArduCopter/mode_sport.cpp | 125 | multirotor::update_sport | FULL |
| ArduCopter/mode_stabilize.cpp | 64 | multirotor::update_stabilize | FULL |
| ArduCopter/mode_stabilize_heli.cpp | 82 | None dispatched | MISSING |
| ArduCopter/mode_systemid.cpp | 453 | stub → update_loiter | STUB |
| ArduCopter/mode_throw.cpp | 330 | multirotor::update_throw | PARTIAL |
| ArduCopter/mode_turtle.cpp | 238 | multirotor::update_turtle (no DShot) | PARTIAL |
| ArduCopter/mode_zigzag.cpp | 578 | multirotor::update_zigzag | PARTIAL |
| ArduCopter/AP_Arming_Copter.cpp | 857 | meridian-arming (partial) | PARTIAL |
| ArduCopter/failsafe.cpp | 84 | meridian-failsafe (monitors) | PARTIAL |
| ArduCopter/events.cpp | 525 | meridian-failsafe (monitors) | PARTIAL |
| ArduPlane/mode.cpp | 414 | fixed_wing.rs (FixedWingModes) | PARTIAL |
| ArduPlane/mode_LoiterAltQLand.cpp | 59 | None | MISSING |
| ArduPlane/mode_acro.cpp | 227 | fixed_wing::update_fw_acro | PARTIAL |
| ArduPlane/mode_auto.cpp | 202 | fixed_wing::update_auto | PARTIAL |
| ArduPlane/mode_autoland.cpp | 315 | None | MISSING |
| ArduPlane/mode_autotune.cpp | 23 | None (unwired) | STUB |
| ArduPlane/mode_avoidADSB.cpp | 23 | None | MISSING |
| ArduPlane/mode_circle.cpp | 23 | fixed_wing::update_circle | PARTIAL |
| ArduPlane/mode_cruise.cpp | 97 | fixed_wing::update_cruise | PARTIAL |
| ArduPlane/mode_fbwa.cpp | 45 | fixed_wing::update_fbwa | FULL |
| ArduPlane/mode_fbwb.cpp | 24 | fixed_wing::update_fbwb | PARTIAL |
| ArduPlane/mode_guided.cpp | 202 | fixed_wing::update_guided | PARTIAL |
| ArduPlane/mode_loiter.cpp | 161 | fixed_wing::update_loiter | PARTIAL |
| ArduPlane/mode_manual.cpp | 31 | fixed_wing::update_manual | FULL |
| ArduPlane/mode_qacro.cpp | 79 | None | MISSING |
| ArduPlane/mode_qautotune.cpp | 50 | None | MISSING |
| ArduPlane/mode_qhover.cpp | 58 | None | MISSING |
| ArduPlane/mode_qland.cpp | 35 | None | MISSING |
| ArduPlane/mode_qloiter.cpp | 187 | None | MISSING |
| ArduPlane/mode_qrtl.cpp | 229 | None | MISSING |
| ArduPlane/mode_qstabilize.cpp | 103 | None | MISSING |
| ArduPlane/mode_rtl.cpp | 169 | fixed_wing::update_rtl | PARTIAL |
| ArduPlane/mode_stabilize.cpp | 18 | fixed_wing::update_stabilize | FULL |
| ArduPlane/mode_takeoff.cpp | 206 | fixed_wing::update_takeoff | PARTIAL |
| ArduPlane/mode_thermal.cpp | 150 | stub → update_loiter | STUB |
| ArduPlane/mode_training.cpp | 71 | stub → update_stabilize | STUB |
| ArduPlane/quadplane.cpp | 4976 | stub only | STUB |
| ArduPlane/tailsitter.cpp | 1060 | None | MISSING |
| ArduPlane/AP_Arming_Plane.cpp | 477 | meridian-arming (partial) | PARTIAL |
| ArduPlane/failsafe.cpp | 115 | meridian-failsafe | PARTIAL |
| ArduPlane/events.cpp | 361 | meridian-failsafe | PARTIAL |
| Rover/mode.cpp | 569 | rover.rs (RoverModes) | PARTIAL |
| Rover/mode_acro.cpp | 64 | rover::update_acro | PARTIAL |
| Rover/mode_auto.cpp | 1057 | rover::update_auto | PARTIAL |
| Rover/mode_circle.cpp | 322 | None | MISSING |
| Rover/mode_dock.cpp | 265 | stub → update_guided | STUB |
| Rover/mode_follow.cpp | 114 | None | MISSING |
| Rover/mode_guided.cpp | 447 | rover::update_guided | PARTIAL |
| Rover/mode_hold.cpp | 18 | rover::update_hold | FULL |
| Rover/mode_loiter.cpp | 80 | rover::update_loiter | PARTIAL |
| Rover/mode_manual.cpp | 38 | rover::update_manual | FULL |
| Rover/mode_rtl.cpp | 84 | rover::update_rtl | PARTIAL |
| Rover/mode_simple.cpp | 32 | None | MISSING |
| Rover/mode_smart_rtl.cpp | 144 | rover::update_smartrtl | PARTIAL |
| Rover/mode_steering.cpp | 54 | rover::update_steering | FULL |
| Rover/AP_Arming_Rover.cpp | 254 | meridian-arming (partial) | PARTIAL |
| Rover/failsafe.cpp | 170 | meridian-failsafe | PARTIAL |
| ArduSub/mode.cpp | 300 | submarine.rs (SubModes) | PARTIAL |
| ArduSub/mode_acro.cpp | 42 | sub::update_sub_acro | FULL |
| ArduSub/mode_althold.cpp | 130 | sub::update_depth_hold | PARTIAL |
| ArduSub/mode_auto.cpp | 580 | stub → update_stabilize | STUB |
| ArduSub/mode_circle.cpp | 91 | None | STUB |
| ArduSub/mode_guided.cpp | 888 | stub → update_stabilize | STUB |
| ArduSub/mode_manual.cpp | 35 | sub::update_manual | FULL |
| ArduSub/mode_motordetect.cpp | 174 | None | MISSING |
| ArduSub/mode_poshold.cpp | 134 | stub → update_depth_hold | STUB |
| ArduSub/mode_stabilize.cpp | 69 | sub::update_stabilize | FULL |
| ArduSub/mode_surface.cpp | 64 | None | MISSING |
| ArduSub/mode_surftrak.cpp | 176 | stub → update_depth_hold | STUB |
| Blimp/mode.cpp | 191 | None | MISSING |
| Blimp/mode_land.cpp | 23 | None | MISSING |
| Blimp/mode_loiter.cpp | 49 | None | MISSING |
| Blimp/mode_manual.cpp | 16 | None | MISSING |
| Blimp/mode_rtl.cpp | 20 | None | MISSING |
| Blimp/mode_velocity.cpp | 22 | None | MISSING |
| Blimp/AP_Arming_Blimp.cpp | 418 | None | MISSING |
| Blimp/failsafe.cpp | 73 | None | MISSING |
| Blimp/events.cpp | 175 | None | MISSING |
| libraries/AP_Mission/AP_Mission.cpp | 3199 | meridian-mission (partial) | PARTIAL |
| libraries/AP_Mission/AP_Mission_ChangeDetector.cpp | 62 | None | MISSING |
| libraries/AP_Mission/AP_Mission_Commands.cpp | 377 | Partial (crates unwired) | PARTIAL |
| libraries/AP_Arming/AP_Arming.cpp | 2189 | meridian-arming (partial) | PARTIAL |
| libraries/AC_Fence/AC_Fence.cpp | 1270 | meridian-fence (minimal) | PARTIAL |
| libraries/AC_Fence/AC_PolyFence_loader.cpp | 1719 | None | MISSING |
| libraries/AP_Rally/AP_Rally.cpp | 209 | meridian-nav/rally.rs (partial) | PARTIAL |
| libraries/AP_SmartRTL/AP_SmartRTL.cpp | 914 | meridian-nav/smartrtl.rs (partial) | PARTIAL |
| libraries/AC_WPNav/AC_WPNav.cpp | 1039 | meridian-nav/waypoint.rs + scurve.rs | PARTIAL |
| libraries/AC_WPNav/AC_WPNav_OA.cpp | 296 | None | MISSING |
| libraries/AC_WPNav/AC_Loiter.cpp | 403 | meridian-control/position_controller.rs | PARTIAL |
| libraries/AC_WPNav/AC_Circle.cpp | 489 | Embedded in mode code | PARTIAL |
| libraries/AP_Landing/AP_Landing.cpp | 781 | fixed_wing::update_land | PARTIAL |
| libraries/AP_Landing/AP_Landing_Deepstall.cpp | 667 | None | MISSING |
| libraries/AP_Landing/AP_Landing_Slope.cpp | 448 | fixed_wing::update_land | PARTIAL |
| libraries/AP_Follow/AP_Follow.cpp | 964 | multirotor FollowConfig only | PARTIAL |
| libraries/AC_Avoidance/AC_Avoid.cpp | 1584 | meridian-proximity (partial) | PARTIAL |
| libraries/AC_Avoidance/AC_Avoidance_Logging.cpp | 100 | None | MISSING |
| libraries/AC_Avoidance/AP_OABendyRuler.cpp | 719 | None | MISSING |
| libraries/AC_Avoidance/AP_OADatabase.cpp | 516 | None | MISSING |
| libraries/AC_Avoidance/AP_OADijkstra.cpp | 1021 | None | MISSING |
| libraries/AC_Avoidance/AP_OAPathPlanner.cpp | 440 | None | MISSING |
| libraries/AC_Avoidance/AP_OAVisGraph.cpp | 47 | None | MISSING |
| libraries/AP_Avoidance/AP_Avoidance.cpp | 682 | meridian-adsb (partial) | PARTIAL |

---

### Parity Count Summary

| Status | Count |
|---|---|
| FULL | 10 |
| PARTIAL | 55 |
| PARTIAL/STUB boundary | 8 |
| STUB | 12 |
| MISSING | 25 |
| **Total files audited** | **110** |

---

### Priority Gap Categories

**P0 — Safety / Must-Have Before Operational Use**
- events.cpp mode-cascade failsafe dispatch (all vehicles) — mode-switch logic on RC/battery/GCS loss
- AC_Fence auto-enable/disable, polygon exclusion zones, destination pre-check
- AP_Arming per-check enabled bitmask, hardware safety switch, mission checks
- Precland integration into Land and Loiter modes

**P1 — Core Mission Functionality**
- AP_Mission EEPROM storage + MAVLink upload/download
- AP_Mission verify_command loop + DO_JUMP / conditional logic
- AC_WPNav OA-wrapped navigation (AC_WPNav_OA)
- AP_Follow MAVLink message handler + dead-reckoning estimator
- AC_PolyFence_loader persistent fence storage

**P2 — VTOL / QuadPlane (large scope)**
- Entire QuadPlane subsystem (4,976 lines) — SLT transition, Q-modes, Q_ASSIST
- Tailsitter (1,060 lines)
- LoiterAltQLand, ModeAutoLand

**P3 — OA Path Planning**
- AP_OAPathPlanner + AP_OABendyRuler + AP_OADijkstra + AP_OADatabase + AP_OAVisGraph
- AC_WPNav_OA

**P4 — Vehicle Classes Not Implemented**
- Blimp (6 modes + arming + failsafe)
- Heli-specific modes (Stabilize_Heli, Acro_Heli, Autorotate as dispatched mode)

**P5 — Specialized / Niche Modes**
- FlowHold (optical flow position hold)
- SystemId (frequency-domain system identification)
- Thermal soaring (mode_thermal.cpp)
- Deepstall landing
- Motor detection (Sub)
- Surface emergency mode (Sub)
- SurfTrak rangefinder bottom-tracking (Sub)
- Rover Circle, Rover Follow, Rover Simple
