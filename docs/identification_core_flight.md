# Core Flight Algorithms — Identification Report
## Agent 1 of 5: AP_NavEKF3, AC_AttitudeControl, AC_PID, AC_PosControl, AP_AHRS, AP_Math, Filter

Generated: 2026-04-02  
ArduPilot base: D:\projects\ardupilot  
Meridian base: D:\projects\meridian  

---

## METHODOLOGY

Each ArduPilot `.cpp` and `.h` file in the seven target libraries is catalogued. Meridian equivalents are identified by cross-referencing the Rust crate tree. Parity status is assessed by comparing function signatures, algorithms, constants, and structural completeness between the two codebases.

**PARITY STATUS DEFINITIONS**
- FULL: All key algorithms, constants, and functions are present and match
- PARTIAL: Core algorithm present but missing variants, constants, or edge cases
- STUB: File/struct exists but no real algorithm — placeholder only
- MISSING: No equivalent exists in Meridian at all

---

## LIBRARY 1: AP_NavEKF3

ArduPilot's third-generation Extended Kalman Filter. 16,542 total lines.

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3.cpp
LINES: 2182
PURPOSE: Top-level EKF3 manager: multi-core setup, IMU routing, health scoring, lane switching
KEY ALGORITHMS: Multi-core health scoring, primary lane selection, GNSS source weighting, IMU index assignment, per-core startup sequencing
KEY CONSTANTS: EKF_TARGET_DT = 0.01s, GPS_DELAY_MS default 220ms, 3 max cores
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/core.rs (EkfCore + EkfLane)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot supports up to 3 simultaneous EKF cores with independent IMU assignments; Meridian has EkfLane struct but single active core only
  - ArduPilot has nuanced GNSS source weighting (GPS1 vs GPS2) with per-source health tracking; Meridian has basic GPS blending (gps_blend.rs) but not the full source-weighting logic
  - ArduPilot has AP_DAL (Data Abstraction Layer) for replay/logging; Meridian has no DAL equivalent
  - Missing: setWindMagStateLearningMode(), controlFilterModes() state machine, detectFlight() flight-vs-ground detection
  - Missing: origin reset logic for long-range flights (reset_origin stub exists)

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3.h
LINES: 597
PURPOSE: Public EKF3 interface: parameter definitions, output API declarations, multi-core configuration
KEY ALGORITHMS: N/A (header)
KEY CONSTANTS: EKF_TARGET_DT = 0.01, EKF_TARGET_DT_MS = 10, GPS delay parameters, velocity/position noise defaults
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/lib.rs + crates/meridian-ekf/src/params.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot exposes 40+ tunable AP_Param parameters (EK3_*); Meridian EkfParams has ~20 params with no persistent storage
  - Missing: EK3_DRAG_* parameters for multicopter drag fusion
  - Missing: EK3_VELD_M_NSE, EK3_POSNE_M_NSE, EK3_ALT_M_NSE with correct default values

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_core.cpp
LINES: 2272
PURPOSE: EKF core initialization, state reset, re-initialization, and main update dispatcher
KEY ALGORITHMS: Covariance initialization, state reset from sensors, EKF time management, IMU buffer management
KEY CONSTANTS: Initial covariance values: P_att=0.1, P_vel=0.7, P_pos=15.0, P_gyro_bias=1e-9, P_accel_bias=1e-5, P_wind=25.0, P_mag=0.025
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/core.rs
PARITY STATUS: PARTIAL
GAPS:
  - Initial covariance values not matched to ArduPilot defaults (Meridian uses different starting values)
  - ArduPilot's IMU ring buffer uses dynamic sizing based on measured GPS delay; Meridian uses fixed IMU_DELAY_BUFFER_LEN=100
  - Missing: resetQuatStateYawOnly() for yaw-only resets without disrupting roll/pitch
  - Missing: ResetPosition() with source-specific logic (GPS/visual odometry/beacon)
  - Missing: ResetVelocity() with source-specific logic

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_core.h
LINES: 1674
PURPOSE: EKF core state vector definition (24 states), covariance indices, internal types
KEY ALGORITHMS: State vector layout definition
KEY CONSTANTS: State count=24: quat(4), vel(3), pos(3), gyro_bias(3), accel_bias(3), mag_NED(3), mag_body(3), wind(2); covariance matrix 24x24
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/state.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot state vector has 24 states; Meridian uses a reduced state vector without the 3 NED mag + 3 body mag + 2 wind states when inhibited
  - ArduPilot has stateIndexLim to dynamically reduce state count when wind/mag inhibited; Meridian does not implement stateIndexLim
  - ArduPilot has separate fusionModeGPS (0=3D, 1=2D+baro, 2=2D+rngfinder); Meridian doesn't expose this
  - Missing: storedIMU (ring buffer of IMU deltas), storedGPS, storedBaro, storedMag, storedOF, storedRange — fully typed delay buffers
  - Missing: dragFusionEnabled, tasDataDelayed, ofDataDelayed typed structs

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp
LINES: 893
PURPOSE: Filter mode control: aiding mode transitions (GPS/flow/none), mag learning, flight detection
KEY ALGORITHMS: setAidingMode() FSM, setWindMagStateLearningMode(), checkAttitudeAlignmentStatus(), detectFlight()
KEY CONSTANTS: PV_AidingMode enum: AID_ABSOLUTE=2, AID_RELATIVE=1, AID_NONE=0; inFlight threshold 1.5m/s
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/aiding.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot's setAidingMode() has 7 distinct transitions with hysteresis timers; Meridian's AidingMode is simpler
  - ArduPilot's detectFlight() uses combined gyro+accel+alt logic; Meridian FlightDetector is simpler
  - Missing: finalInflightYawInit (first in-flight yaw alignment flag) — critical for mag/GPS yaw handoff
  - Missing: inhibitMagStates/inhibitWindStates with full state-index management
  - Missing: GPS_VEL_YAW_ALIGN_MIN_SPD = 1.0 m/s (copter) / 5.0 m/s (plane) constant

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_GyroBias.cpp
LINES: 24
PURPOSE: Stub — gyro bias estimation is performed inside the prediction step
KEY ALGORITHMS: None (empty stub, actual gyro bias in prediction)
KEY CONSTANTS: None
MERIDIAN EQUIVALENT: Implicit in crates/meridian-ekf/src/predict.rs (gyro_bias in state)
PARITY STATUS: FULL
GAPS: None (empty file, ArduPilot also has no dedicated logic here)

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp
LINES: 1544
PURPOSE: Magnetometer fusion: 3-axis body-frame mag, NED field fusion, yaw fusion, GSF-based yaw
KEY ALGORITHMS: FuseMagnetometer() (full 3-axis EKF update), fuseEulerYaw() (heading-only update), controlMagYawReset(), magFieldStrength check
KEY CONSTANTS: MAG_GATE_SIGMA=3.0 (3-sigma gate), initial mag variance=0.025 gauss^2, yaw variance=0.05 rad^2
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/fusion.rs (fuse_mag)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot fuses both NED mag body (3 scalar measurements sequentially); Meridian fuses single heading scalar only
  - Missing: Full 3-axis body-frame mag fusion with cross-axis coupling through full H matrix
  - Missing: controlMagYawReset() — the complex state machine for yaw reset during initial alignment and in-flight
  - Missing: fuseEulerYaw() — GPS-velocity-derived yaw alignment (critical for no-compass operation)
  - Missing: mag field strength validation (expected ~0.5 Gauss, reject outliers)
  - Missing: Earth field vector calculation from declination/inclination for NED field fusion

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp
LINES: 669
PURPOSE: Airspeed and sideslip fusion for fixed-wing; drag-model fusion for multirotors
KEY ALGORITHMS: FuseAirspeed() (TAS scalar), FuseSideslip() (zero sideslip for planes), FuseDragForces() (multirotor drag model using motor current)
KEY CONSTANTS: TAS gate=3.0 sigma, drag gate=5.0 sigma, BCOEF_X/Y default, Mcoef default
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/airspeed_fusion.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot FuseDragForces() uses multicopter drag model for wind estimation; Meridian does not implement drag fusion
  - ArduPilot FuseSideslip() is plane-only zero-sideslip constraint; Meridian stub exists but not wired
  - ArduPilot airspeed fusion handles differential pressure sensor delay; Meridian assumes immediate data

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp
LINES: 760
PURPOSE: Optical flow fusion for GPS-denied navigation
KEY ALGORITHMS: FuseOptFlow() (2D velocity measurement from flow sensor), predictFlowAngRates(), calcOptFlowEarthRates()
KEY CONSTANTS: FLOW_GATE_SIGMA=3.0, flow delay default 5ms, min range 0.3m
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/flow_fusion.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot optical flow fusion includes full body-rate compensation with gyro data; Meridian has stub structure
  - Missing: calcOptFlowEarthRates() — converts body flow to earth frame
  - Missing: predictFlowAngRates() — forward-predicts flow from current state for innovation check
  - Missing: range finder height constraint coupling with optical flow

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
LINES: 2073
PURPOSE: GPS position + velocity fusion, barometer height fusion, range finder fusion
KEY ALGORITHMS: FuseVelPosNED() (sequential scalar fusion of 6 GPS measurements), FuseBodyVel(), selectHeightForFusion() (GPS/baro/rangefinder priority)
KEY CONSTANTS: GPS pos gate=5.0 sigma, GPS vel gate=5.0 sigma, baro gate=5.0 sigma, GPS acceleration check=4.0 m/s^2
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/fusion.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot fuses GPS pos(N,E) and vel(N,E,D) as 5 sequential scalar updates; Meridian fuses 3 pos + 3 vel = 6 scalars but without the exact ArduPilot H matrix structure
  - ArduPilot selectHeightForFusion() chooses GPS-alt vs baro vs rangefinder with fallback priority; Meridian always uses baro
  - Missing: FuseBodyVel() — optical flow body-frame velocity fusion
  - Missing: GPS acceleration consistency check (reject if delta-V > 4.0 m/s^2/s)
  - Missing: VELD_M_NSE (vertical velocity noise) separate from horizontal
  - ArduPilot has postype_t (double precision position) for long-range flights; Meridian uses f32 throughout

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp
LINES: 677
PURPOSE: Range beacon fusion for GPS-denied indoor positioning (UWB/ultrasonic beacons)
KEY ALGORITHMS: FuseRngBcn() (range measurement fusion to known beacon positions), InitialiseBeaconOrigin()
KEY CONSTANTS: BEACON_GATE_SIGMA=3.0 sigma
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No range beacon (UWB/ultrasonic) fusion capability exists in Meridian
  - Required for indoor/GPS-denied positioning

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_VehicleStatus.cpp
LINES: 482
PURPOSE: Vehicle status detection: landing, takeoff, bad IMU, bad mag
KEY ALGORITHMS: detectFlight(), BadIMUData detection (sudden accel spike), detectOptFlowVehicleMovement(), calcGpsGoodToAlign()
KEY CONSTANTS: IMU spike threshold=15.0 m/s^2, GPS speed threshold for yaw alignment=1.0 m/s
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/aiding.rs (FlightDetector, GpsQualityGate)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot's BadIMUData check uses successive comparison of delta-velocity; Meridian does not implement IMU spike rejection
  - ArduPilot calcGpsGoodToAlign() checks HDOP, satellite count, speed, age simultaneously; Meridian GpsQualityGate is simpler
  - Missing: calcGpsGoodForFlight() — stricter flight-mode GPS quality gate

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp
LINES: 695
PURPOSE: Output extraction: attitude, velocity, position, wind, mag field, gyro bias, innovation variances
KEY ALGORITHMS: getQuaternion(), getVelNED(), getPosNED(), getWind(), getMagNED(), getGyroScaleFactor(), getInnovations()
KEY CONSTANTS: N/A (data extraction)
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/output.rs + crates/meridian-ekf/src/state.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot output predictor runs at IMU rate (400 Hz) projecting delayed state forward; Meridian OutputPredictor exists but not fully connected to the IMU-rate loop
  - Missing: getGyroScaleFactor() — gyro scale factor compensation output
  - Missing: getInnovations() — full innovation vector output for telemetry/logging
  - Missing: getVariances() — EKF variance telemetry (used by GCS health display)

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_Logging.cpp
LINES: 456
PURPOSE: Data logging for EKF state and innovation telemetry
KEY ALGORITHMS: Log_Write_EKF3(), Log_Write_NKF() — MAVLink-formatted log messages
KEY CONSTANTS: Log message IDs for XKF1-XKF5 packets
MERIDIAN EQUIVALENT: NONE (Meridian has meridian-log crate but no EKF-specific telemetry)
PARITY STATUS: MISSING
GAPS:
  - No EKF telemetry logging output in Meridian
  - ArduPilot logs: NKF1 (attitude+vel), NKF2 (innovations), NKF3 (variances), NKF4 (gyro bias), NKF5 (mag)

---

FILE: libraries/AP_NavEKF3/AP_NavEKF3_feature.h
LINES: (included in h count)
PURPOSE: Feature flags for EKF3 compilation options
KEY ALGORITHMS: N/A
KEY CONSTANTS: EK3_FEATURE_BEACON_FUSION, EK3_FEATURE_DRAG_FUSION, EK3_FEATURE_EXTERNAL_NAV
MERIDIAN EQUIVALENT: Cargo.toml features
PARITY STATUS: PARTIAL
GAPS: Missing drag fusion and external nav feature flags

---

FILE: libraries/AP_NavEKF3/LogStructure.h
LINES: (header)
PURPOSE: Log message format definitions for EKF telemetry
KEY ALGORITHMS: N/A
KEY CONSTANTS: Log packet format strings for NKF1..NKF5, NKY0..NKY2
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS: No structured EKF log format definitions

---

## LIBRARY 2: AC_AttitudeControl

Attitude and position control. 8,288 total lines.

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
LINES: 1540
PURPOSE: Base attitude controller: quaternion error -> rate targets, input shaping, feedforward
KEY ALGORITHMS: attitude_controller_run_quat(), input_euler_angle_roll_pitch_yaw(), input_thrust_vector_heading(), shape_angle_vel_accel() for attitude shaping, feedforward from shaped rates
KEY CONSTANTS: ANG_LIM_MIN=10 deg, ANG_LIM_MAX=80 deg, ANGLE_MAX_DEFAULT=30 deg, INPUT_TC_DEFAULT=0.15s, THRUST_ERROR_ANGLE_RAD=0.5236 rad (30 deg)
MERIDIAN EQUIVALENT: crates/meridian-control/src/attitude_controller.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot attitude controller uses shape_angle_vel_accel() (jerk-limited angle shaping) for smooth angle commands; Meridian uses simpler acceleration-clamped shaping
  - ArduPilot's input_euler_angle_roll_pitch_yaw() applies per-axis input time constant shaping via AC_CommandModel; Meridian does not use CommandModel
  - ArduPilot has separate slew-limited yaw target with ATC_SLEW_YAW param; Meridian yaw is shaped same as roll/pitch
  - Missing: update_althold_lean_angle_max() — dynamic lean angle max based on throttle headroom (uses throttle hover + actual throttle)
  - Missing: proper angle boost formula; ArduPilot uses cos_tilt with inverted_factor = constrain(10*cos_tilt, 0, 1); Meridian has close approximation but not exact formula
  - Missing: ATC_ANGLE_BOOST param and enable/disable logic
  - Missing: body_frame_to_euler() output extraction path

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl.h
LINES: 723
PURPOSE: Attitude controller interface: parameter declarations, virtual methods for multi/heli variants
KEY ALGORITHMS: N/A (header)
KEY CONSTANTS: ATC_INPUT_TC_DEFAULT=0.15, ATC_ANG_RLL_P default=4.5, ATC_ANG_PIT_P default=4.5, ATC_ANG_YAW_P default=4.5, ATC_ACCEL_R_MAX default=110000 cdeg/s^2 (=19.2 rad/s^2), ATC_ACCEL_P_MAX=110000, ATC_ACCEL_Y_MAX=27000 cdeg/s^2 (=4.71 rad/s^2)
MERIDIAN EQUIVALENT: crates/meridian-control/src/attitude_controller.rs (AttitudeGains struct)
PARITY STATUS: PARTIAL
GAPS:
  - Missing: ATC_RATE_R_MAX, ATC_RATE_P_MAX, ATC_RATE_Y_MAX (separate per-axis rate limits)
  - Missing: ATC_SLEW_YAW (yaw slew rate limit in deg/s)
  - Missing: ATC_THR_MIX_MIN, ATC_THR_MIX_MAX, ATC_THR_MIX_MAN params

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
LINES: 534
PURPOSE: Multirotor-specific attitude controller: rate PIDs, throttle-RPY mix, gain boost on throttle transients
KEY ALGORITHMS: attitude_controller_run_quat() with feedforward attenuation, update_throttle_gain_boost() (boost P+D during rapid throttle slew), get_throttle_avg_max(), landed gain multipliers
KEY CONSTANTS: THR_MIX_DEFAULT=0.5, THR_MIX_MIN=0.1, THR_MIX_MAX=0.9, BOOST_THRESH=0.5/s, BOOST_P_MAX=2.0x, ATC_LAND_R/P/Y_MULT=1.0
MERIDIAN EQUIVALENT: crates/meridian-control/src/rate_controller.rs + attitude_controller.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot throttle gain boost applies to both the rate controller P/D AND the angle P; Meridian's update_throttle_gain_boost() only returns multiplier without applying it to angle P
  - ArduPilot landed gain multiplier is applied at each update call automatically; Meridian requires caller to manually apply/restore
  - Missing: AC_AttitudeControl_Multi::relax_attitude_controllers() — graceful ramp-down on disarm

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h
LINES: 158
PURPOSE: Multirotor attitude controller parameter declarations
KEY ALGORITHMS: N/A
KEY CONSTANTS: RAT_RLL_P=0.135, RAT_RLL_I=0.135, RAT_RLL_D=0.0036, RAT_RLL_FF=0.0, RAT_RLL_FLTD=20Hz, RAT_RLL_FLTT=20Hz; yaw: FLTE=2.5Hz
MERIDIAN EQUIVALENT: crates/meridian-control/src/rate_controller.rs (default_rate_* functions)
PARITY STATUS: FULL
GAPS: None — Meridian default gains match ArduPilot defaults

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
LINES: 563
PURPOSE: Traditional helicopter attitude controller with swashplate mixing, rotor speed governor interaction
KEY ALGORITHMS: Collective-to-yaw mixing, VFF (velocity feedforward), coll_angle_filt for collective trim
KEY CONSTANTS: Helicopter-specific: HELI_ACRO_COL_EXPO=0.0, collective mixing ratios
MERIDIAN EQUIVALENT: crates/meridian-heli/src/lib.rs (STUB)
PARITY STATUS: STUB
GAPS:
  - meridian-heli is a placeholder; ArduPilot's Heli attitude controller has 563 lines of helicopter-specific logic
  - Missing: entire collective/cyclic/tail mixing for traditional heli
  - Missing: rotor speed governor coupling
  - Missing: VFF (velocity feedforward) for heli rate control

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.h
LINES: 171
PURPOSE: Helicopter attitude controller parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: HELI_ACRO_COL_EXPO, VFF gains, swash mixing params
MERIDIAN EQUIVALENT: STUB
PARITY STATUS: STUB
GAPS: See AC_AttitudeControl_Heli.cpp above

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.cpp
LINES: 189
PURPOSE: 6-DoF multirotor (omnidirectional) attitude control with independent thrust vector and heading
KEY ALGORITHMS: Decoupled thrust vector + yaw control, six independent motor thrust allocation
KEY CONSTANTS: 6DoF-specific tilt angle limits
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No 6-DoF attitude control exists in Meridian
  - Required for omnidirectional drone support

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h
LINES: 120
PURPOSE: 6-DoF parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.cpp
LINES: 495
PURPOSE: Submarine attitude controller with pressure-referenced depth hold
KEY ALGORITHMS: Depth hold PID, quaternion yaw lock, attitude recovery from large angles
KEY CONSTANTS: Sub-specific: ROLL_PITCH_MAX=25 deg for sub stability
MERIDIAN EQUIVALENT: crates/meridian-modes/src/submarine.rs (minimal stub)
PARITY STATUS: STUB
GAPS:
  - ArduPilot Sub attitude controller has 495 lines; Meridian submarine mode is a stub
  - Missing: depth hold PID, pressure-referenced altitude
  - Missing: large-angle attitude recovery

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.h
LINES: 129
PURPOSE: Sub attitude parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: STUB
PARITY STATUS: STUB

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_TS.cpp
LINES: 136
PURPOSE: TailSitter VTOL attitude controller (quad-plane transitioning between hover and cruise)
KEY ALGORITHMS: Attitude blending during transition, separate hover and fixed-wing control paths
KEY CONSTANTS: Tailsitter-specific transition angle threshold
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No tailsitter/VTOL transition control in Meridian

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_TS.h
LINES: 28
PURPOSE: TailSitter parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_AttitudeControl/AC_CommandModel.cpp
LINES: 49
PURPOSE: Pilot input command model: rate command shaping with expo and time constant
KEY ALGORITHMS: Expo curve (input_expo function), first-order rate smoothing with RATE_TC time constant
KEY CONSTANTS: RATE default 200 deg/s, EXPO default 0.0, RATE_TC default 0.1s
MERIDIAN EQUIVALENT: NONE (RC input mapping is done inline in mode code)
PARITY STATUS: MISSING
GAPS:
  - ArduPilot's AC_CommandModel shapes pilot stick input with expo and rate smoothing
  - Meridian has no equivalent command model for RC input shaping
  - Missing: input_expo() applied to stick inputs before attitude commands

---

FILE: libraries/AC_AttitudeControl/AC_CommandModel.h
LINES: 37
PURPOSE: CommandModel parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_AttitudeControl/AC_PosControl.cpp
LINES: 1918
PURPOSE: 3D position controller: jerk-limited XY position hold + Z altitude hold
KEY ALGORITHMS: shape_pos_vel_accel_xy() pipeline, update_pos_vel_accel_xy() forward integration, limit_accel_corner_xy() corner handling, vertical pos P + vel PID, acceleration to lean angles conversion, hover throttle adaptation
KEY CONSTANTS: POSCONTROL_D_POS_P=1.0, POSCONTROL_D_VEL_P=5.0, POSCONTROL_NE_POS_P=1.0, POSCONTROL_NE_VEL_P=2.0, POSCONTROL_NE_VEL_I=1.0, POSCONTROL_NE_VEL_D=0.25, PSC_JERK_XY=17.0 m/s^3, WPNAV_SPEED=5.0 m/s, WPNAV_ACCEL=2.5 m/s^2, CORNER_ACCELERATION_RATIO=1/sqrt(2)=0.7071
MERIDIAN EQUIVALENT: crates/meridian-control/src/position_controller.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot uses true 2D vector operations throughout (Vector2p for double-precision position); Meridian uses scalar x/y pairs
  - ArduPilot's limit_accel_xy() uses braking-priority algorithm (prioritize braking over lateral); Meridian uses simple magnitude limit at corners
  - ArduPilot tracks limit_vector (Vector3f) for full 3D saturation feedback; Meridian only tracks X/Y saturation booleans
  - ArduPilot's vertical controller uses separate accel P gain (PSC_ACCZ_P) and I gain; Meridian uses single PID for vertical
  - Missing: kinematic_limit() — max velocity in arbitrary 3D direction given XY/Z limits
  - Missing: shape_pos_vel_accel_xy() operating on Vector2p (double-precision) for high-precision position
  - ArduPilot uses postype_t (double when HAL_WITH_POSTYPE_DOUBLE); Meridian is always f32
  - Missing: get_lean_angle_max_cd() — lean angle max in centidegrees for param compatibility

---

FILE: libraries/AC_AttitudeControl/AC_PosControl.h
LINES: 807
PURPOSE: Position controller parameters: PSC_POSXY_P, PSC_VELXY_*, PSC_POSZ_P, PSC_VELZ_*, PSC_JERK_*
KEY ALGORITHMS: N/A (header)
KEY CONSTANTS: PSC_POSXY_P=1.0, PSC_VELXY_P=2.0, PSC_VELXY_I=1.0, PSC_VELXY_D=0.5, PSC_VELXY_IMAX=1000, PSC_VELZ_P=5.0, PSC_ACCZ_P=0.5, PSC_ACCZ_I=1.0, PSC_JERK_XY=17.0, PSC_JERK_Z=15.0
MERIDIAN EQUIVALENT: crates/meridian-control/src/position_controller.rs (PositionGains)
PARITY STATUS: PARTIAL
GAPS:
  - Missing: PSC_ACCZ_P and PSC_ACCZ_I — ArduPilot vertical has a 3-cascade (pos->vel->accel->throttle); Meridian has 2-cascade (pos->vel->throttle)
  - Missing: PSC_JERK_Z (=15.0 m/s^3) for vertical jerk limiting; Meridian has no vertical jerk limiting
  - Missing: PILOT_SPEED_UP, PILOT_SPEED_DN, PILOT_ACCEL_Z params

---

FILE: libraries/AC_AttitudeControl/AC_PosControl_Logging.cpp
LINES: 108
PURPOSE: Position controller telemetry logging
KEY ALGORITHMS: Log_Write_PSC() — position, velocity, acceleration log
KEY CONSTANTS: Log message format PSCD (down) and PSCE (east) and PSCN (north)
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No position controller telemetry in Meridian

---

FILE: libraries/AC_AttitudeControl/AC_WeatherVane.cpp
LINES: 241
PURPOSE: Weather vane: allows yaw to align with wind for power efficiency (fixed-wing in hover)
KEY ALGORITHMS: weathervane_yaw_rate() — computes yaw rate correction to minimize lateral drag, Kff-based wind estimation
KEY CONSTANTS: WVANE_GAIN default=1.0, WVANE_ANG_MIN=10 deg, WVANE_SPD_MAX=2.0 m/s
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No weather vane yaw logic in Meridian
  - Required for VTOL fixed-wing hover efficiency

---

FILE: libraries/AC_AttitudeControl/AC_WeatherVane.h
LINES: 57
PURPOSE: WeatherVane parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp
LINES: 81
PURPOSE: Attitude controller logging (rate targets vs actuals)
KEY ALGORITHMS: Log_Write_Rate(), Log_Write_Attitude()
KEY CONSTANTS: Rate log packet format RATE, attitude log packet ATT
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS: No attitude/rate telemetry logging in Meridian

---

FILE: libraries/AC_AttitudeControl/LogStructure.h
LINES: 204
PURPOSE: Log structure definitions for attitude and position controller telemetry
KEY ALGORITHMS: N/A
KEY CONSTANTS: ATT, RATE, PSC*, PIDR, PIDP, PIDY packet formats
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

## LIBRARY 3: AC_PID

PID controllers. 2,205 total lines.

---

FILE: libraries/AC_PID/AC_PID.cpp
LINES: 448
PURPOSE: General-purpose PID with three independent LP filters (target/error/derivative), slew rate limiter, notch filter support, PDMX (PD sum max), D_FF (derivative feedforward)
KEY ALGORITHMS: update_all() (full update with limit flag), update_i() (integrator with anti-windup), SlewLimiter::modifier() (adaptive gain reduction), calc_lowpass_alpha_dt() (IIR alpha from Hz+dt), relax_integrator()
KEY CONSTANTS: WINDOW_MS=300ms, MODIFIER_GAIN=1.5, DERIVATIVE_CUTOFF_FREQ=25 Hz, integrator floor at 0.1x nominal
MERIDIAN EQUIVALENT: crates/meridian-control/src/pid.rs
PARITY STATUS: FULL
GAPS:
  - Meridian PidController fully implements all ArduPilot AC_PID features: three independent LP filters, slew limiter with 25 Hz cutoff, PDMX, D_FF, notch filter slots, anti-windup, relax_integrator
  - Minor: ArduPilot SlewLimiter uses AP_HAL::millis() for windowing; Meridian uses dt-accumulated decay (equivalent result)
  - Minor: ArduPilot update_all() accepts pd_scale parameter for scaling P+D before I; Meridian update_full() uses i_scale but not separate pd_scale

---

FILE: libraries/AC_PID/AC_PID.h
LINES: 216
PURPOSE: PID class interface with AP_Param integration
KEY ALGORITHMS: N/A (header)
KEY CONSTANTS: See AC_PID.cpp above
MERIDIAN EQUIVALENT: crates/meridian-control/src/pid.rs (PidGains, PidController)
PARITY STATUS: FULL
GAPS: Missing AP_Param persistent storage (Meridian uses compile-time gains, no EEPROM)

---

FILE: libraries/AC_PID/AC_P.cpp
LINES: 30
PURPOSE: Simple P controller (proportional only) with EEPROM gain storage
KEY ALGORITHMS: get_p() = error * kp
KEY CONSTANTS: None
MERIDIAN EQUIVALENT: Inline in pid.rs (kp * error is used directly)
PARITY STATUS: FULL
GAPS: None — trivially implemented inline

---

FILE: libraries/AC_PID/AC_P.h
LINES: 61
PURPOSE: P controller interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Inline
PARITY STATUS: FULL

---

FILE: libraries/AC_PID/AC_PI.cpp
LINES: 46
PURPOSE: PI controller (no derivative)
KEY ALGORITHMS: PI update: output = P*error + I (with windup clamp)
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Subset of PidController (kd=0)
PARITY STATUS: FULL
GAPS: None — covered by PidController with kd=0

---

FILE: libraries/AC_PID/AC_PI.h
LINES: 42
PURPOSE: PI controller interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: PidController with kd=0
PARITY STATUS: FULL

---

FILE: libraries/AC_PID/AC_HELI_PID.cpp
LINES: 132
PURPOSE: Helicopter-specific PID with leaky integrator (I-term that naturally decays to zero)
KEY ALGORITHMS: update_leaky_i() — integrator leaks toward ILMI (I Leak Min) at rate leak_rate per tick
KEY CONSTANTS: ILMI default=0.0 (disable leak), leak decay formula: I -= (I - ILMI) * leak_rate
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No leaky integrator PID variant in Meridian
  - Required for traditional helicopter tail rotor control (prevents I-term wind-up on the tail)

---

FILE: libraries/AC_PID/AC_HELI_PID.h
LINES: 32
PURPOSE: Helicopter PID parameters (ILMI)
KEY ALGORITHMS: N/A
KEY CONSTANTS: ILMI (I Leak Min) parameter
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_PID/AC_PID_2D.cpp
LINES: 217
PURPOSE: 2D PID controller operating on Vector2f (horizontal plane)
KEY ALGORITHMS: update_all() on 2D error vector, 2D integrator with vector magnitude windup clamp, 2D derivative filter
KEY CONSTANTS: AC_PID_2D_FILT_D_HZ_MIN=0.005 Hz
MERIDIAN EQUIVALENT: NONE (Meridian uses two separate 1D PIDs for X and Y)
PARITY STATUS: MISSING
GAPS:
  - ArduPilot's AC_PID_2D maintains a single integrator as a Vector2f, clamps by total magnitude; two separate 1D PIDs behave differently for diagonal motion
  - Required for proper NE position hold with coupled wind estimation

---

FILE: libraries/AC_PID/AC_PID_2D.h
LINES: 111
PURPOSE: 2D PID interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_PID/AC_PID_Basic.cpp
LINES: 183
PURPOSE: Simplified PID without filtering (for less critical loops)
KEY ALGORITHMS: Basic P+I+D with simple anti-windup, no input/error/derivative filters
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: PidController with filter frequencies set to 0
PARITY STATUS: FULL
GAPS: None — covered by PidController with filters disabled

---

FILE: libraries/AC_PID/AC_PID_Basic.h
LINES: 105
PURPOSE: Basic PID interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: PidController
PARITY STATUS: FULL

---

FILE: libraries/AC_PID/AC_PI_2D.cpp
LINES: 166
PURPOSE: 2D PI controller (no derivative) for position hold
KEY ALGORITHMS: 2D PI update with vector magnitude windup clamp
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE (two separate 1D PIDs with kd=0)
PARITY STATUS: MISSING
GAPS: Same as AC_PID_2D — vector magnitude windup clamp differs from two independent scalars

---

FILE: libraries/AC_PID/AC_PI_2D.h
LINES: 97
PURPOSE: 2D PI interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_PID/AC_P_1D.cpp
LINES: 94
PURPOSE: 1D P controller with output clamping (used in position P loop)
KEY ALGORITHMS: output = clamp(P*error, -output_max, output_max); optional deadband
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Inline in position_controller.rs
PARITY STATUS: FULL
GAPS: None — trivially implemented inline

---

FILE: libraries/AC_PID/AC_P_1D.h
LINES: 64
PURPOSE: 1D P controller interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Inline
PARITY STATUS: FULL

---

FILE: libraries/AC_PID/AC_P_2D.cpp
LINES: 77
PURPOSE: 2D P controller (Vector2f) with magnitude-based output clamping
KEY ALGORITHMS: output = clamp_magnitude(P * error_vec, output_max)
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - ArduPilot AC_P_2D clamps by 2D magnitude, not per-axis; Meridian position controller uses per-axis clamping which can exceed total magnitude limit at 45-degree error directions

---

FILE: libraries/AC_PID/AC_P_2D.h
LINES: 60
PURPOSE: 2D P controller interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AC_PID/AP_PIDInfo.h
LINES: 24
PURPOSE: PID telemetry struct definition (target, actual, error, P, I, D, FF, Dmod, slew_rate)
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-control/src/pid.rs (PidInfo struct)
PARITY STATUS: FULL
GAPS: None — PidInfo fields match exactly

---

## LIBRARY 4: AC_PosControl (in AC_AttitudeControl directory)

Note: ArduPilot's AC_PosControl library lives inside the AC_AttitudeControl directory. Files already catalogued above (AC_PosControl.cpp/h + AC_PosControl_Logging.cpp).

---

## LIBRARY 5: AP_AHRS

8,454 total lines.

---

FILE: libraries/AP_AHRS/AP_AHRS.cpp
LINES: 3815
PURPOSE: AHRS manager: source selection (EKF3/DCM/external), multi-IMU, attitude/velocity/position output API
KEY ALGORITHMS: Source selection state machine, EKF health check and fallback to DCM, wind compensation, body-frame velocity extraction, home position management, pre-arm check aggregation
KEY CONSTANTS: ATTITUDE_CHECK_THRESH_ROLL_PITCH=10 deg, ATTITUDE_CHECK_THRESH_YAW=20 deg, HAL_AHRS_EKF_TYPE_DEFAULT=3 (EKF3)
MERIDIAN EQUIVALENT: crates/meridian-ahrs/src/lib.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot AHRS is 3,815 lines; Meridian AHRS wrapper is 110 lines
  - ArduPilot has full source selection (EKF2/EKF3/DCM/external) with health-based fallback; Meridian only wraps EKF3, no DCM fallback path
  - ArduPilot manages wind estimation output and airspeed correction; Meridian doesn't expose wind vector
  - ArduPilot has home position management and relative-to-home position; Meridian doesn't track home
  - Missing: get_wind() — wind velocity estimate from EKF
  - Missing: get_airspeed() — airspeed estimate 
  - Missing: pre_arm_check() — AHRS-level arming checks (attitude alignment, gyro cal, etc.)
  - Missing: reset_gyro_drift() — external gyro recalibration reset
  - Missing: set_EKF_use() / get_EKF_type() — source switching API

---

FILE: libraries/AP_AHRS/AP_AHRS.h
LINES: 1096
PURPOSE: AHRS interface definition with complete output API
KEY ALGORITHMS: N/A (header)
KEY CONSTANTS: AHRS_EKF_TYPE_NONE=0, AHRS_EKF_TYPE_SIM=2, AHRS_EKF_TYPE_EKF3=3, AHRS_EKF_TYPE_EXTERNAL=11
MERIDIAN EQUIVALENT: crates/meridian-ahrs/src/lib.rs
PARITY STATUS: PARTIAL
GAPS: Same as AP_AHRS.cpp above

---

FILE: libraries/AP_AHRS/AP_AHRS_DCM.cpp
LINES: 1340
PURPOSE: DCM (Direction Cosine Matrix) fallback AHRS: Mahony complementary filter with gyro drift correction from accelerometer and GPS
KEY ALGORITHMS: matrix_update() (DCM integration with gyro), normalize() (orthonormalization), drift_correction_yaw() (GPS/compass yaw correction), drift_correction() (accel-based roll/pitch correction), Mahony PI controller for gyro bias, renorm()
KEY CONSTANTS: GPS_SPEED_MIN=3 m/s (yaw lock speed), SPIN_RATE_LIMIT=20 deg/s (I-term freeze), Ki=0.0087890625 (integral gain), Kp=0.4 (proportional gain), DCM_roll_pitch_ki = 0.00001
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/dcm.rs (partial)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot DCM is 1,340 lines; Meridian dcm.rs is 295 lines
  - ArduPilot DCM has full GPS-based yaw correction using GPS velocity; Meridian DCM stub doesn't have GPS yaw
  - Missing: Mahony PI gains (Ki=0.0087890625, Kp=0.4) — these specific values are tuned empirically
  - Missing: drift_correction_yaw() — GPS-velocity based yaw drift correction
  - Missing: matrix orthonormalization with renorm() — prevents DCM drift over time
  - Missing: use_fast_ahrs path for fixed-wing (no-GPS) operation

---

FILE: libraries/AP_AHRS/AP_AHRS_DCM.h
LINES: 281
PURPOSE: DCM AHRS parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: See AP_AHRS_DCM.cpp
MERIDIAN EQUIVALENT: crates/meridian-ekf/src/dcm.rs
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_AHRS/AP_AHRS_Backend.cpp
LINES: 313
PURPOSE: Base AHRS backend: common output extraction (attitude->body rates, NED velocity, groundspeed)
KEY ALGORITHMS: get_velocity_NED(), get_expected_mag_field_NED(), groundspeed_vector(), calc_trig() (pre-computed sin/cos of attitude angles)
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-ahrs/src/lib.rs (partial)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot's calc_trig() pre-computes trig values once per update for reuse; Meridian recomputes per call
  - Missing: get_expected_mag_field_NED() — expected field from declination/inclination model

---

FILE: libraries/AP_AHRS/AP_AHRS_Backend.h
LINES: 290
PURPOSE: Backend interface definition
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-ahrs/src/lib.rs
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_AHRS/AP_AHRS_External.cpp
LINES: 139
PURPOSE: External AHRS backend (e.g., VectorNav, MicroStrain, MATEK AHRS) — passes through external navigation data
KEY ALGORITHMS: Pass-through from AP_ExternalAHRS, attitude/velocity/position direct assignment
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No external AHRS (hardware navigation unit) support in Meridian

---

FILE: libraries/AP_AHRS/AP_AHRS_External.h
LINES: 84
PURPOSE: External AHRS backend interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AP_AHRS/AP_AHRS_SIM.cpp
LINES: 256
PURPOSE: SITL simulation AHRS backend — reads truth state directly from simulator
KEY ALGORITHMS: Direct state injection from SITL physics model
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-platform-sitl/src/sitl_util.rs (partial)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot SITL AHRS injects perfect truth directly bypassing EKF; Meridian SITL feeds sensor noise through the real EKF

---

FILE: libraries/AP_AHRS/AP_AHRS_SIM.h
LINES: 120
PURPOSE: SITL AHRS interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: PARTIAL
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_AHRS/AP_AHRS_View.cpp
LINES: 108
PURPOSE: "View" into the AHRS from a rotated perspective (e.g., gimbal camera orientation)
KEY ALGORITHMS: Rotate attitude output by a fixed transform for cameras/gimbals
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No rotated-view AHRS in Meridian
  - Required for gimbal/camera attitude reference

---

FILE: libraries/AP_AHRS/AP_AHRS_View.h
LINES: 225
PURPOSE: AHRS View interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AP_AHRS/AP_AHRS_config.h
LINES: 46
PURPOSE: AHRS compilation feature flags
KEY ALGORITHMS: N/A
KEY CONSTANTS: AP_AHRS_DCM_ENABLED, AP_AHRS_EKF3_ENABLED, AP_AHRS_EXTERNAL_ENABLED
MERIDIAN EQUIVALENT: Cargo.toml features
PARITY STATUS: PARTIAL
GAPS: No AP_AHRS_EXTERNAL_ENABLED equivalent

---

FILE: libraries/AP_AHRS/AP_AHRS_Logging.cpp
LINES: 162
PURPOSE: AHRS telemetry logging (attitude, gyro, accel to dataflash)
KEY ALGORITHMS: Log_Write_AHRS2(), Log_Write_POS()
KEY CONSTANTS: ATT, POS log packet formats
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AP_AHRS/LogStructure.h
LINES: 179
PURPOSE: AHRS log structure definitions
KEY ALGORITHMS: N/A
KEY CONSTANTS: Log packet formats for ATT, AHRS, POS, etc.
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

## LIBRARY 6: AP_Math

9,939 total lines.

---

FILE: libraries/AP_Math/control.cpp
LINES: 826
PURPOSE: Control math primitives: sqrt_controller, shape functions, velocity/position integrators, kinematic limit, input conversion
KEY ALGORITHMS: sqrt_controller() (hybrid linear/sqrt), inv_sqrt_controller(), sqrt_controller_accel() (chain-rule acceleration), shape_accel() (jerk-limited), shape_vel_accel(), shape_pos_vel_accel() (1D), shape_pos_vel_accel_xy() (2D), shape_angle_vel_accel() (angular), limit_accel_xy(), limit_accel_corner_xy() (braking-priority), kinematic_limit(), update_vel_accel(), update_pos_vel_accel(), input_expo(), rc_input_to_roll_pitch_rad(), angle conversions
KEY CONSTANTS: CORNER_ACCELERATION_RATIO=1/sqrt(2)=0.7071, GRAVITY_MSS=9.80665
MERIDIAN EQUIVALENT: crates/meridian-control/src/sqrt_controller.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot control.cpp has shape_accel_xy() operating on Vector2f (true 2D jerk limiting as single vector operation); Meridian does two scalar shape_accel calls
  - ArduPilot limit_accel_corner_xy() has full braking-priority algorithm decomposing into along-track and cross-track components; Meridian uses simpler corner angle detection
  - ArduPilot shape_vel_accel() has limit_total_accel boolean that allows total accel constraint vs separate axis constraint; Meridian does not have this flag
  - ArduPilot shape_pos_vel_accel_xy() operates on Vector2p (double-precision positions); Meridian uses f32
  - Missing: shape_angle_vel_accel() — angular version with angle_vel_max and angle_accel_max; Meridian has shape_angular_vel() but not the full angle-shaped version
  - Missing: kinematic_limit() — max achievable magnitude in arbitrary 3D direction given axis limits
  - Missing: input_expo() — expo curve for stick shaping (used by AC_CommandModel)
  - Missing: rc_input_to_roll_pitch_rad() — stick input to attitude target conversion
  - ArduPilot sqrt_controller_accel() has different signature: (error, rate_cmd, rate_state, p, second_ord_lim); Meridian uses (error, p, second_ord_lim, vel)

---

FILE: libraries/AP_Math/control.h
LINES: 227
PURPOSE: Control math function declarations
KEY ALGORITHMS: N/A (header)
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-control/src/sqrt_controller.rs
PARITY STATUS: PARTIAL
GAPS: See control.cpp above

---

FILE: libraries/AP_Math/quaternion.cpp
LINES: 840
PURPOSE: Full quaternion math: rotation matrices, Euler angles, axis-angle, SLERP, rotation from vectors
KEY ALGORITHMS: rotation_matrix(), from_euler(), to_euler(), from_axis_angle(), from_rotation(), slerp(), inverse(), normalize(), to_axis_angle(), rotate()
KEY CONSTANTS: Rotation table for 45 standard orientations (ROTATION_ROLL_180 etc.), special rotation constants
MERIDIAN EQUIVALENT: crates/meridian-math/src/quaternion.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot Quaternion has 840 lines; Meridian quaternion.rs has 332 lines
  - ArduPilot has full slerp() with shortest-path check; Meridian has slerp but should verify sign convention
  - ArduPilot has from_rotation() mapping to 45 discrete rotation enums (standard ArduPilot orientations); Meridian doesn't have discrete rotation mapping
  - Missing: to_euler() with full ZYX decomposition outputting roll/pitch/yaw consistently with ArduPilot convention
  - Missing: from_axis_angle(Vector3f) direct construction
  - ArduPilot uses q1=w, q2=x, q3=y, q4=z internally; Meridian uses w,x,y,z — convention matches

---

FILE: libraries/AP_Math/quaternion.h
LINES: 207
PURPOSE: Quaternion class template definition
KEY ALGORITHMS: N/A (header)
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-math/src/quaternion.rs
PARITY STATUS: PARTIAL
GAPS: See quaternion.cpp

---

FILE: libraries/AP_Math/location.cpp
LINES: 63
PURPOSE: Location math: bearing between 2D points, lat/lng range checks
KEY ALGORITHMS: get_bearing_rad(), get_bearing_cd(), check_lat(), check_lng()
KEY CONSTANTS: Lat range: ±90°, Lng range: ±180°
MERIDIAN EQUIVALENT: crates/meridian-math/src/geodetic.rs (partial)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot location.cpp has bearing calculation; Meridian geodetic.rs doesn't have 2D bearing
  - ArduPilot Location class has get_distance() using flat-earth approximation; Meridian uses haversine (more accurate but more expensive)

---

FILE: libraries/AP_Math/location.h
LINES: 42
PURPOSE: Location math declarations
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-math/src/geodetic.rs
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_Math/location_double.cpp
LINES: 132
PURPOSE: Double-precision location math for high-precision NED position calculation
KEY ALGORITHMS: location_diff() (lat/lng difference to NED meters), location_offset() (NED offset to new lat/lng), EARTH_RADIUS_MERIDIAN, EARTH_RADIUS_NORMAL
KEY CONSTANTS: RADIUS_OF_EARTH=6356752.3142 m (polar), LAT_SCALE_FACTOR, LNG_SCALE_FACTOR; WGS84 semi-major=6378137 m
MERIDIAN EQUIVALENT: crates/meridian-math/src/geodetic.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot location_diff() uses flat-earth approximation scaled by local radius; Meridian uses full ECEF conversion — more accurate but more expensive
  - Missing: location_offset() — apply NED offset to a lat/lng position (used heavily in navigation)
  - ArduPilot uses integer lat/lng (1e-7 degrees) for long-range precision; Meridian uses f64 degrees

---

FILE: libraries/AP_Math/matrix3.cpp
LINES: 257
PURPOSE: 3x3 matrix operations: multiplication, inverse, determinant, transpose
KEY ALGORITHMS: operator*(), operator+(), invert(), det(), transpose(), mul_transpose()
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-math/src/rotation.rs (partial)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot Matrix3 has invert() for 3x3; Meridian Rotation doesn't implement matrix inverse (not typically needed for rotation matrices since inverse = transpose)
  - ArduPilot has mat_inverse() in matrix_alg.cpp for NxN; Meridian has no general matrix solver

---

FILE: libraries/AP_Math/matrix3.h
LINES: 292
PURPOSE: 3x3 matrix template class
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-math/src/rotation.rs
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_Math/vector3.cpp
LINES: 637
PURPOSE: 3D vector operations: length, normalize, cross product, component-wise operators
KEY ALGORITHMS: All standard vector operations plus is_zero(), is_nan(), is_inf(), stabilize_sqrt()
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-math/src/vec3.rs
PARITY STATUS: FULL
GAPS:
  - Meridian Vec3 is type-parameterized (Vec3<Frame>) which ArduPilot doesn't have, giving it additional type safety
  - Minor: ArduPilot Vector3 has angle_between() (angle between two vectors); Meridian may not have this

---

FILE: libraries/AP_Math/vector3.h
LINES: 411
PURPOSE: 3D vector template class
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-math/src/vec3.rs
PARITY STATUS: FULL

---

FILE: libraries/AP_Math/vector2.cpp
LINES: 460
PURPOSE: 2D vector operations
KEY ALGORITHMS: Same as vector3 but 2D: dot, cross (returns scalar), normalize, angle_between
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-math/src/vec3.rs (2D subset inline)
PARITY STATUS: PARTIAL
GAPS:
  - Meridian doesn't have a dedicated Vec2 type; 2D operations are done with x/y fields of Vec3 or as scalar pairs
  - ArduPilot's 2D vector with proper type is used extensively in position controller; Meridian uses scalar pairs

---

FILE: libraries/AP_Math/vector2.h
LINES: 301
PURPOSE: 2D vector template class
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE (no Vec2 type)
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_Math/SCurve.cpp
LINES: 1201
PURPOSE: Full 7-segment jerk-limited S-curve trajectory planner with arc segments and speed change handling
KEY ALGORITHMS: set_speed_max(), calc_segs() (compute 7 phase durations), advance_target_along_track() (step along path), set_origin_speed_max(), calculate_path(), get_target_pos_vel_accel(), braking distance calculation
KEY CONSTANTS: SEG_CONST=15 (total segments), SEG_ACCEL_MAX=4, SEG_DECEL_END=22; snap_max parameter for 4th-derivative limiting
MERIDIAN EQUIVALENT: crates/meridian-nav/src/scurve.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot SCurve has 1,201 lines with 22+ segment types including arc segments and speed change handling; Meridian has 832 lines covering basic 7-phase profile
  - ArduPilot has arc segment support (curved paths for waypoint transitions); Meridian is straight-line only
  - ArduPilot has snap_max (4th derivative) limiting in addition to jerk; Meridian does not
  - ArduPilot SCurve tracks entry and exit velocity for chained segments; Meridian always starts/ends at zero
  - Missing: advance_target_along_track() — incremental step-along-path used by WPNav each frame

---

FILE: libraries/AP_Math/SCurve.h
LINES: 236
PURPOSE: SCurve interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-nav/src/scurve.rs
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_Math/SplineCurve.cpp
LINES: 252
PURPOSE: Cubic Hermite spline for smooth waypoint transitions
KEY ALGORITHMS: position_on_spline() using quintic polynomial, calculate_leg() (compute spline coefficients), advance_target()
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - ArduPilot uses SplineCurve for smooth waypoint-to-waypoint transitions with continuous velocity
  - Meridian has no spline waypoint transition support

---

FILE: libraries/AP_Math/SplineCurve.h
LINES: 68
PURPOSE: SplineCurve interface
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING

---

FILE: libraries/AP_Math/matrix_alg.cpp
LINES: 469
PURPOSE: General NxN matrix algorithms: Gaussian elimination, LU decomposition, inversion
KEY ALGORITHMS: mat_inverse() (Gaussian elimination with partial pivoting), used by EKF covariance operations and PolyFit
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - No general matrix inverse solver in Meridian
  - ArduPilot uses this for PolyFit (4th-order polynomial fitting used in gyro temperature compensation and airspeed calibration)

---

FILE: libraries/AP_Math/matrixN.cpp / matrixN.h
LINES: 60 / 48
PURPOSE: N-dimensional square matrix template (used by EKF covariance)
KEY ALGORITHMS: matN arithmetic operators, zero(), identity(), transpose()
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Inline array operations in crates/meridian-ekf/src/state.rs (Covariance)
PARITY STATUS: PARTIAL
GAPS:
  - Meridian's Covariance is a fixed-size 24x24 array struct, not a generic template; adequate for EKF but less flexible

---

FILE: libraries/AP_Math/vectorN.h
LINES: 176
PURPOSE: N-dimensional vector template (used by EKF state vector)
KEY ALGORITHMS: N/A
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Fixed arrays in state.rs
PARITY STATUS: PARTIAL

---

FILE: libraries/AP_Math/rotations.h
LINES: 94
PURPOSE: Rotation enum and lookup table for 45 standard ArduPilot board orientations
KEY ALGORITHMS: enum Rotation (45 values), rotation_to_str() (debugging), rotation_to_euler()
KEY CONSTANTS: ROTATION_NONE=0 through ROTATION_MAX=45; all standard orientations (ROLL_180, PITCH_90, etc.)
MERIDIAN EQUIVALENT: crates/meridian-boardcfg/src/lib.rs (partial)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot has 45 standard orientations; Meridian board configs support rotation but not full 45-enum lookup
  - Missing: rotation_to_euler() — convert rotation enum to Euler angles
  - Missing: apply_rotation() — apply rotation enum to a Vector3f

---

FILE: libraries/AP_Math/definitions.h
LINES: 126
PURPOSE: Physical constants and math constants
KEY ALGORITHMS: N/A
KEY CONSTANTS: DEG_TO_RAD=PI/180, RAD_TO_DEG=180/PI, GRAVITY_MSS=9.80665, M_2PI=6.28318, KNOTS_TO_M_PER_SEC=0.514444, FEET_TO_METERS=0.3048, METERS_TO_FEET=3.28084
MERIDIAN EQUIVALENT: Scattered across crates using core::f32::consts::PI and inline constants
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot has a centralized definitions.h; Meridian has no central constants file
  - GRAVITY constant in position_controller.rs (9.80665) matches, but other constants are not centralized
  - Missing: M_2PI, KNOTS conversion, pressure altitude constants

---

FILE: libraries/AP_Math/AP_Math.cpp
LINES: 595
PURPOSE: General math utilities: angle wrapping, safe_sqrt, constrain, linear_interpolate, matrix rotation from euler
KEY ALGORITHMS: wrap_PI(), wrap_360(), wrap_2PI(), safe_sqrt(), constrain_float/int, linear_interpolate(), euler_to_rotation_matrix(), is_equal() floats, is_positive(), is_negative(), is_zero()
KEY CONSTANTS: FLT_EPSILON for zero-check threshold
MERIDIAN EQUIVALENT: Scattered across crates (wrap_pi in quaternion.rs, constrain via .clamp())
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot's safe_sqrt() returns 0 for negative inputs; Meridian uses libm::sqrtf() which returns NaN for negatives (potential safety issue)
  - ArduPilot's linear_interpolate() is used for gain scheduling; Meridian has no centralized interpolation utility
  - Missing: is_zero(), is_positive(), is_negative() as distinct utility functions
  - ArduPilot's euler_to_rotation_matrix() produces the exact rotation matrix ArduPilot conventions use (ZYX order); Meridian's Quaternion::from_euler must match this exactly

---

FILE: libraries/AP_Math/AP_Math.h
LINES: 445
PURPOSE: AP_Math umbrella header; also defines Vector2F/Vector3F/Matrix3F/QuaternionF type aliases for double-precision EKF support
KEY ALGORITHMS: N/A
KEY CONSTANTS: NaNf constant
MERIDIAN EQUIVALENT: crates/meridian-math/src/lib.rs
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot supports HAL_WITH_EKF_DOUBLE (double-precision EKF); Meridian is f32 throughout

---

FILE: libraries/AP_Math/polygon.cpp / polygon.h
LINES: 237 / 46
PURPOSE: 2D polygon operations: point-in-polygon test, closest edge, polygon intersection
KEY ALGORITHMS: Polygon_outside() (ray casting), Polygon_complete() (closure check), Polygon_closest_edge()
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: crates/meridian-fence/src/lib.rs (partial)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot polygon operations are used for geofence; Meridian fence crate exists but may not have all polygon operations
  - Missing: Polygon_intersects() for geofence segment intersection detection

---

FILE: libraries/AP_Math/chirp.cpp / chirp.h
LINES: 92 / 59
PURPOSE: Frequency sweep (chirp) signal generator for system identification (autotune)
KEY ALGORITHMS: Chirp::update() — exponential frequency sweep with fade-in/fade-out windowing, Hanning window
KEY CONSTANTS: B = log(wMax/wMin), exponential frequency formula: w(t) = wMin * exp(B*(t-const_freq)/(record-const_freq))
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - ArduPilot chirp signal is used by autotune for frequency response measurement
  - Meridian autotune crate exists but uses different (step-response) approach, not chirp-based

---

FILE: libraries/AP_Math/polyfit.cpp / polyfit.h
LINES: 62 / 27
PURPOSE: Polynomial curve fitting (4th order) for temperature compensation
KEY ALGORITHMS: PolyFit::update() (accumulate normal equations), get_polynomial() (solve via matrix inversion)
KEY CONSTANTS: Template parameter: order=4, xtype=double, vtype=Vector3f
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - ArduPilot uses polyfit for IMU temperature calibration; Meridian IMU temperature compensation (imu_tempcal.rs exists in drivers but) does not have polynomial fitting

---

FILE: libraries/AP_Math/spline5.cpp / spline5.h
LINES: 71 / 6
PURPOSE: 5th-order spline interpolation for smooth path following
KEY ALGORITHMS: spline5() — 5th-order polynomial position/velocity interpolation
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - Used by WPNav for smooth wp transition (different from SplineCurve — this is the underlying math)

---

FILE: libraries/AP_Math/AP_GeodesicGrid.cpp / AP_GeodesicGrid.h
LINES: 474 / 298
PURPOSE: Geodesic grid for magnetometer interference field sampling
KEY ALGORITHMS: Icosahedral geodesic grid subdivison, section triangulation, grid lookup, neighbor finding
KEY CONSTANTS: NUM_SECTORS=3, NUM_TRIANGLES=20 (icosahedron)
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - Used by compass motor interference compensation (not critical for basic flight)

---

FILE: libraries/AP_Math/ftype.h
LINES: 76
PURPOSE: Floating-point type selection for EKF (float vs double)
KEY ALGORITHMS: N/A
KEY CONSTANTS: ftype=float or double based on HAL_WITH_EKF_DOUBLE
MERIDIAN EQUIVALENT: Implicit (Meridian always uses f32)
PARITY STATUS: PARTIAL
GAPS: No double-precision EKF path in Meridian

---

FILE: libraries/AP_Math/div1000.h
LINES: 26
PURPOSE: Fast integer division by 1000 using multiply-and-shift
KEY ALGORITHMS: div1000() inline — replaces /1000 with *0.001
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Inline in Rust
PARITY STATUS: FULL
GAPS: None — Rust compiler handles division optimization automatically

---

## LIBRARY 7: Filter

2,836 total lines.

---

FILE: libraries/Filter/LowPassFilter.cpp / LowPassFilter.h
LINES: 131 / 131
PURPOSE: First-order IIR low-pass filter with alpha = dt/(dt + 1/(2*pi*f))
KEY ALGORITHMS: apply() = prev + alpha*(input - prev), calc_lowpass_alpha_dt(dt, cutoff_hz)
KEY CONSTANTS: alpha calculation: rc = 1/(2*pi*f), alpha = dt/(dt+rc)
MERIDIAN EQUIVALENT: Inline in crates/meridian-control/src/pid.rs (lp_filter() function)
PARITY STATUS: FULL
GAPS:
  - ArduPilot LowPassFilter is a class with templated types (float, Vector2f, Vector3f); Meridian has it as an inline function
  - Functionality identical; no parity gap

---

FILE: libraries/Filter/LowPassFilter2p.cpp / LowPassFilter2p.h
LINES: 126 / 96
PURPOSE: 2nd-order Butterworth low-pass (biquad) filter for gyro/accel filtering
KEY ALGORITHMS: DigitalBiquadFilter::apply() (Direct Form I), compute_params() (Butterworth pole placement using tan/cos), reset() (initialize state to steady-state value)
KEY CONSTANTS: Butterworth design: ohm = tan(pi/fr), c = 1+2*cos(pi/4)*ohm+ohm^2, b0=ohm^2/c, b1=2*b0, b2=b0, a1=2*(ohm^2-1)/c, a2=(1-2*cos(pi/4)*ohm+ohm^2)/c
MERIDIAN EQUIVALENT: NONE (Meridian uses first-order IIR for all filtering)
PARITY STATUS: MISSING
GAPS:
  - ArduPilot uses 2nd-order Butterworth for all gyro/accel filtering; Meridian has no 2nd-order IIR
  - The 2nd-order filter has -40 dB/decade rolloff vs -20 dB/decade for 1st-order — significant difference for gyro noise suppression
  - Missing Butterworth coefficient calculation formula
  - Required for: rate controller derivative filter (same roll-off as ArduPilot)

---

FILE: libraries/Filter/NotchFilter.cpp / NotchFilter.h
LINES: 151 / 83
PURPOSE: Single-frequency 2nd-order IIR notch (band-reject) filter
KEY ALGORITHMS: NotchFilter::apply() (biquad form), compute_params() using Q-factor and center freq, frequency slew limiting (5% per update)
KEY CONSTANTS: HARMONIC_NYQUIST_CUTOFF=0.48 * Fs, Q = center_freq / bandwidth, biquad coefficients from bilinear transform
MERIDIAN EQUIVALENT: crates/meridian-control/src/notch_filter.rs
PARITY STATUS: FULL
GAPS:
  - Meridian notch filter matches ArduPilot: Q-based biquad coefficients, 5% slew limiter, attenuation-based gain
  - Minor: ArduPilot NotchFilter uses Hz-based Q calculation; Meridian uses same formula (verified)

---

FILE: libraries/Filter/HarmonicNotchFilter.cpp / HarmonicNotchFilter.h
LINES: 572 / 182
PURPOSE: Dynamic harmonic notch filter: tracks motor RPM (via throttle, RPM sensor, or FFT) and places notches at harmonics
KEY ALGORITHMS: update() per frame updating notch center frequency, apply() chaining multiple notches, dynamic frequency calculation: f = ref_freq * sqrt(throttle / ref_throttle), up to 8 harmonics
KEY CONSTANTS: HARMONIC_NYQUIST_CUTOFF=0.48, NOTCHFILTER_ATTENUATION_CUTOFF=0.25, ref_throttle=0.5, default harmonics=3, NOTCH_FREQ_DEFAULT=80 Hz, NOTCH_BW_DEFAULT=40 Hz, NOTCH_ATT_DEFAULT=40 dB
MERIDIAN EQUIVALENT: crates/meridian-fft/src/gyro_fft.rs (partial — FFT-based frequency detection only)
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot HarmonicNotchFilter chains multiple (up to 8) notch filters at harmonics; Meridian's gyro FFT detects peaks but doesn't dynamically move notch filters
  - ArduPilot supports 4 modes: Fixed, Throttle-based, RPM-based, FFT-based; Meridian only has FFT detection
  - Missing: throttle-based dynamic frequency (f = ref_freq * sqrt(throttle/ref_throttle)) — lowest complexity mode
  - Missing: per-motor independent notch for multi-RPM sensor equipped vehicles
  - ArduPilot's harmonic notch is applied inside the gyro hardware path before the PID; Meridian's notch is applied per-PID via the PID controller notch slots

---

FILE: libraries/Filter/AverageFilter.h
LINES: 177
PURPOSE: Sliding window average filter (templated, 1-N samples)
KEY ALGORITHMS: apply() — compute rolling average, push_sample() — shift buffer
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - Used for smoothing rangefinder readings and RC inputs
  - Not critical for core flight but used in sensor averaging paths

---

FILE: libraries/Filter/DerivativeFilter.cpp / DerivativeFilter.h
LINES: 140 / 55
PURPOSE: Derivative estimator with LP filter (used for barometer rate-of-climb calculation)
KEY ALGORITHMS: Savitzky-Golay derivative estimate from N-point window, LPF on derivative output
KEY CONSTANTS: Default window=5 samples, LP cutoff=20 Hz
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - ArduPilot uses DerivativeFilter for baro rate-of-climb (velocity estimate from barometer); Meridian uses EKF state for vertical velocity
  - Not critical if EKF is healthy

---

FILE: libraries/Filter/ModeFilter.cpp / ModeFilter.h
LINES: 102 / 73
PURPOSE: Statistical mode (most common value) filter for rangefinder spike rejection
KEY ALGORITHMS: insert and sort N samples, return middle value (median for odd N)
KEY CONSTANTS: Default N=5 samples
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - Used for rangefinder reading validation
  - Not critical for primary flight path

---

FILE: libraries/Filter/SlewLimiter.cpp / SlewLimiter.h
LINES: 138 / 49
PURPOSE: Actuator slew rate limiter — adaptive P+D gain reduction when output slew rate exceeds configured maximum
KEY ALGORITHMS: modifier() — compute gain modifier (0.1..1.0) based on peak filtered slew rate; LPF at 25 Hz on slew rate; decay peak with tau time constant; MODIFIER_GAIN=1.5 for gain reduction ratio
KEY CONSTANTS: WINDOW_MS=300ms, MODIFIER_GAIN=1.5, DERIVATIVE_CUTOFF_FREQ=25 Hz, minimum modifier=0.1
MERIDIAN EQUIVALENT: Inline in crates/meridian-control/src/pid.rs (SlewLimiterState + compute_slew_modifier())
PARITY STATUS: PARTIAL
GAPS:
  - ArduPilot SlewLimiter tracks time-stamped peaks with AP_HAL::millis() for decay window; Meridian tracks decay_pos/decay_neg time-accumulated — functionally equivalent but different implementation
  - ArduPilot uses two separate slew_rate_max/slew_rate_tau references (can be shared across instances); Meridian copies into each PID
  - Minor: ArduPilot slew_filter uses LowPassFilter2p (2nd-order Butterworth at 25 Hz); Meridian uses 1st-order IIR at 25 Hz — slightly less sharp roll-off

---

FILE: libraries/Filter/Butter.h
LINES: 105
PURPOSE: Butterworth filter coefficient calculator (general order)
KEY ALGORITHMS: Analog prototype poles, bilinear transform, cascade of 2nd-order sections
KEY CONSTANTS: Butterworth pole positions on unit circle
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - ArduPilot uses this for configurable Butterworth IIR filters
  - Meridian uses only fixed first-order IIR and the hard-coded notch biquad

---

FILE: libraries/Filter/Filter.h / FilterClass.h / FilterWithBuffer.h
LINES: 20 / 42 / 115
PURPOSE: Abstract filter base classes and buffer-backed filter template
KEY ALGORITHMS: Generic filter interface, circular buffer implementation
KEY CONSTANTS: N/A
MERIDIAN EQUIVALENT: Rust trait system (no explicit equivalent)
PARITY STATUS: FULL (design equivalent — Rust traits replace abstract base classes)

---

FILE: libraries/Filter/AP_Filter.cpp / AP_Filter.h
LINES: 157 / 92
PURPOSE: Runtime-configurable filter registry: maps parameter index to filter instance for per-PID notch attachment
KEY ALGORITHMS: get_filter() — lookup by index, setup_notch_filter() — configure and attach notch to PID
KEY CONSTANTS: MAX_FILTERS=8
MERIDIAN EQUIVALENT: NONE
PARITY STATUS: MISSING
GAPS:
  - ArduPilot's AP_Filter allows GCS to configure notch filters by index and attach them to any PID at runtime
  - Meridian's per-PID notch is compile-time attached, no runtime registry

---

FILE: libraries/Filter/AP_Filter_config.h
LINES: 17
PURPOSE: Feature flag for filter system
KEY ALGORITHMS: N/A
KEY CONSTANTS: AP_FILTER_ENABLED
MERIDIAN EQUIVALENT: Cargo feature flags
PARITY STATUS: PARTIAL

---

FILE: libraries/Filter/AP_Filter_params.cpp / AP_NotchFilter_params.cpp
LINES: 27 / 55
PURPOSE: AP_Param definitions for filter parameters
KEY ALGORITHMS: N/A
KEY CONSTANTS: FLTD, FILT, NOTCH param names and defaults
MERIDIAN EQUIVALENT: Compile-time gains (no runtime param system for filters)
PARITY STATUS: MISSING (no runtime param system)

---

## CROSS-CUTTING GAPS (affect multiple libraries)

1. **Double-precision position (postype_t)**: ArduPilot uses `double` for position when `HAL_WITH_POSTYPE_DOUBLE` is set; Meridian uses `f32` throughout. This matters for long-range (>1 km) flights where float precision degrades.

2. **AP_Param parameter system**: ArduPilot has runtime-tunable parameters stored in EEPROM with GCS access. Meridian uses compile-time constants with no persistence. Critical gap for any operational use.

3. **2D vector controller objects (AC_PID_2D, AC_PI_2D, AC_P_2D)**: ArduPilot's 2D PID variants maintain a single 2D integrator with magnitude-based clamping. Meridian uses two independent 1D PIDs, which behaves differently (can violate total magnitude constraint).

4. **Traditional helicopter support**: AC_AttitudeControl_Heli, AC_HELI_PID — no equivalent in Meridian beyond stubs.

5. **Telemetry/logging**: All ArduPilot Logging.cpp files are MISSING in Meridian. No structured flight data logs generated.

6. **2nd-order Butterworth filter**: ArduPilot uses LowPassFilter2p (Butterworth biquad) for all rate controller derivative filtering. Meridian uses first-order IIR — less roll-off, more noise in derivative.

7. **sqrt_controller_accel() signature mismatch**: ArduPilot signature is `(error, rate_cmd, rate_state, p, second_ord_lim)`; Meridian uses `(error, p, second_ord_lim, vel)`. The Meridian version drops `rate_cmd` as a separate parameter — semantically equivalent if `vel = -rate_state` and `rate_cmd` is derived internally, but should be verified.

8. **External AHRS**: No support for hardware navigation units (VectorNav, MicroStrain, etc.)

9. **Range beacon (UWB) fusion**: Completely missing from Meridian EKF.

10. **Chirp signal generator**: Missing from Meridian — needed for proper frequency-domain autotune.

---

## SUMMARY TABLE

| Library | Files | AP Lines | Meridian Equiv | Worst Status |
|---------|-------|----------|----------------|--------------|
| AP_NavEKF3 | 15 | 16,542 | meridian-ekf | PARTIAL |
| AC_AttitudeControl | 21 | 8,288 | meridian-control | MISSING (heli/6DoF/TS) |
| AC_PID | 19 | 2,205 | meridian-control | MISSING (2D PIDs, heli) |
| AC_PosControl | (in AttCtrl) | (in above) | meridian-control | PARTIAL |
| AP_AHRS | 15 | 8,454 | meridian-ahrs | PARTIAL (DCM partial, ext missing) |
| AP_Math | 37 | 9,939 | meridian-math/control | MISSING (SplineCurve, polyfit, chirp) |
| Filter | 24 | 2,836 | meridian-control/fft | MISSING (LPF2p, harmonic notch dynamic) |

**TOTAL ARDUPILOT LINES SCANNED**: ~48,264  
**CRITICAL MISSING ITEMS (flight safety)**:
1. LowPassFilter2p (2nd-order Butterworth) — all gyro derivative filtering is lower quality
2. 2D PID controllers (AC_PID_2D, AC_PI_2D, AC_P_2D) — positional hold behavior differs
3. Full mag fusion (3-axis body-frame, not just heading scalar)
4. Double-precision position path — float precision issues >1 km from home
5. Range beacon fusion (MISSING entirely)
6. SplineCurve (smooth waypoint transitions use straight segments only)
7. Dynamic harmonic notch (throttle-based frequency tracking missing)

**HIGH PRIORITY GAPS (affects tune quality but not safety)**:
1. shape_angle_vel_accel() vs current simplified angle shaping
2. shape_vel_accel() limit_total_accel flag
3. AC_CommandModel (RC expo/shaping)
4. AP_Param persistent storage
5. All telemetry logging (Logging.cpp files)
