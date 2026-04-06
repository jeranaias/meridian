# ArduPilot Python Autotest Suite — Full Audit for Meridian

**Source**: `D:\projects\ardupilot\Tools\autotest\`
**Audit Date**: 2026-04-02
**Purpose**: Document every test scenario Meridian needs to replicate for validation.

---

## 1. Test Framework Architecture

### Overview

The autotest suite lives in `Tools/autotest/`. The main runner is `autotest.py`; the base class for all vehicle test suites is `vehicle_test_suite.py` (~50,000 lines). There is no separate `common.py`; all shared utilities are in `vehicle_test_suite.py`.

### How Tests Are Structured

- Each vehicle inherits from `vehicle_test_suite.TestSuite`
- Tests are grouped into batches (`tests1a()`, `tests1b()`, etc.) for parallelism
- A `Test` wrapper class allows per-test settings: `Test(fn, attempts=4, speedup=8)`
- `disabled_tests()` returns a dict of tests that are known-broken with reasons
- The `tests()` method assembles all batches

### How a Test Works (SITL Lifecycle)

1. Test runner launches SITL binary: `sim_vehicle.py --vehicle ArduCopter --speedup 10`
2. MAVLink connection established via `pymavlink.mavutil.mavlink_connection()`
3. MAVProxy can be spawned for some tests but is mostly optional
4. Heartbeat loop started; `wait_ready_to_arm()` blocks until EKF is happy
5. Test function runs, calling helpers like `takeoff()`, `change_mode()`, `arm_vehicle()`, etc.
6. MAVLink messages read with `assert_receive_message(msg_type, timeout=N)`
7. Pass/fail via Python exceptions: `NotAchievedException`, `AutoTestTimeoutException`, etc.
8. If the test leaves the vehicle armed, `ArmedAtEndOfTestException` is raised
9. SITL is rebooted between tests that modify persistent parameters

### MAVProxy Role

MAVProxy is used only in specific tests that need it (e.g., `DriveSquare` saves waypoints via MAVProxy). The vast majority of tests use `pymavlink` directly. Key utilities: `start_mavproxy()`, `stop_mavproxy()`, `save_mission_to_file_using_mavproxy()`.

### Key Imports

```python
from pymavlink import mavutil, mavwp, mavparm, DFReader, mavextra
from MAVProxy.modules.lib import mp_elevation, mp_util
```

---

## 2. ArduCopter Tests — Complete List

`arducopter.py` is 16,496 lines. Tests are split into 7 batches. Below is every test method (capitalized methods are test scenarios).

### Batch: tests1a (baseline + ~19 copter tests)

Includes all base-class tests from `vehicle_test_suite.TestSuite.tests()` (~20 tests, ~5 min), plus:

| Test | Description |
|------|-------------|
| `NavDelayTakeoffAbsTime` | NAV_DELAY command triggers at absolute time during takeoff |
| `NavDelayAbsTime` | NAV_DELAY with absolute wall-clock time in mission |
| `NavDelay` | NAV_DELAY basic relative-time delay in mission |
| `GuidedSubModeChange` | GUIDED mode sub-mode transitions (pos/vel/accel control) |
| `MAV_CMD_CONDITION_YAW` | Condition yaw command during AUTO mission |
| `LoiterToAlt` | Loiter-to-altitude mission item; transitions at target alt |
| `PayloadPlaceMission` | Payload gripper pickup/place mission |
| `PayloadPlaceMissionOpenGripper` | Gripper opens and holds mid-mission |
| `PrecisionLoiterCompanion` | Companion-computer precision loiter via LANDING_TARGET |
| `Landing` | Landing speed/altitude transition checks (high/low zones) |
| `PrecisionLanding` | PLND backends (SITL, SITL-IRLOCK) — lands within **1.5m** of target |
| `SetModesViaModeSwitch` | Mode changes triggered by RC channel 5 |
| `BackupFence` | Backup fence breach triggers RTL |
| `SetModesViaAuxSwitch` | Mode changes via AUX switch RC options |
| `AuxSwitchOptions` | All AUX switch option codes exercised |
| `AuxFunctionsInMission` | AUX functions callable from AUTO mission (DO_AUX_FUNCTION) |
| `AutoTune` | Full AUTOTUNE cycle; verifies gains discarded on Land |
| `AutoTuneYawD` | AUTOTUNE with axes=12 (roll+yaw), verifies YAW_D updated |
| `NoRCOnBootPreArmFailure` | Boot with SIM_RC_FAIL=1 prevents arming ("RC not found") |

### Batch: tests1b

| Test | Description |
|------|-------------|
| `ThrowMode` | Throw-mode: arm mid-air, vehicle stabilizes |
| `ThrowModeRPMMin` | Throw mode with minimum RPM threshold |
| `BrakeMode` | BRAKE mode stops lateral movement, holds position |
| `RecordThenPlayMission` | Record flight via Ch7 WP save, replay as AUTO mission |
| `ThrottleFailsafe` | **Full RC failsafe suite** — see section 8 |
| `ThrottleFailsafePassthrough` | RC signal passes through during failsafe with FS_OPTIONS |
| `GCSFailsafe` | GCS heartbeat loss triggers failsafe response |
| `CustomController` | Lua custom controller backend integration |
| `WPArcs` | Waypoint arc commands (NAV_WP with radius) |
| `WPArcs2` | Waypoint arc edge cases |

### Batch: tests1c

| Test | Description |
|------|-------------|
| `BatteryFailsafe` | **Full battery failsafe suite** — all 7+ sub-tests, see section 8 |
| `BatteryMissing` | Battery monitor present but no reading |
| `VibrationFailsafe` | Extreme vibration triggers EKF variance failsafe |
| `EK3AccelBias` | EKF3 accel bias estimation |
| `StabilityPatch` | Copter recovers from inverted attitude in Stabilize |
| `OBSTACLE_DISTANCE_3D` | 3D obstacle avoidance via OBSTACLE_DISTANCE_3D message |
| `AC_Avoidance_Proximity` | Proximity sensor avoidance in Loiter |
| `AC_Avoidance_Proximity_AVOID_ALT_MIN` | Proximity avoidance with altitude minimum |
| `AC_Avoidance_Fence` | Fence-backed avoidance behaviour |
| `AC_Avoidance_Beacon` | Beacon-based avoidance |
| `AvoidanceAltFence` | Altitude fence interacts with avoidance |
| `BaroWindCorrection` | Wind estimation ±1 m/s speed, ±15° direction; height variation < 0.5m |
| `SetpointGlobalPos` | SET_POSITION_TARGET_GLOBAL_INT position tracking |
| `ThrowDoubleDrop` | Throw mode with two drop events |
| `SetpointGlobalVel` | SET_POSITION_TARGET_GLOBAL_INT velocity tracking |
| `SetpointBadVel` | Reject invalid velocity setpoints |
| `SplineTerrain` | Spline waypoints with terrain following (TERRAIN_ENABLE) |
| `TakeoffCheck` | Pre-takeoff checks: arming locked until conditions met |
| `GainBackoffTakeoff` | ATC_LAND_R/P/Y_MULT=0 zeros gains; confirms PID output zeros on takeoff |

### Batch: tests1d

| Test | Description |
|------|-------------|
| `HorizontalFence` | Circular fence breach triggers RTL; pre-arm fails outside fence |
| `HorizontalAvoidFence` | Horizontal fence avoidance (two avoid_behave modes) |
| `MaxAltFence` | Max altitude fence breaches in multiple frames (global, relative, terrain) |
| `MaxAltFenceAvoid` | Max alt fence with avoidance enabled |
| `MinAltFence` | Min altitude fence |
| `MinAltFenceAvoid` | Min alt fence with avoidance |
| `FenceFloorEnabledLanding` | Fence floor doesn't block landing when enabled |
| `FenceFloorAutoDisableLanding` | Fence floor auto-disables when landing |
| `FenceFloorAutoEnableOnArming` | Fence floor enables on arming |
| `FenceMargin` | Vehicle stops FENCE_MARGIN meters from fence edge |
| `FenceUpload_MissionItem` | Fence uploaded via mission item protocol |
| `AutoTuneSwitch` | AUTOTUNE switch toggles on/off, gains preserved |
| `AutoTuneAux` | AUTOTUNE via AUX switch |
| `GPSGlitchLoiter` | GPS glitch series in LOITER; vehicle stays within **20m** |
| `GPSGlitchLoiter2` | GPS glitch without twitching; attitude tolerance **1°** |
| `GPSGlitchAuto` | GPS glitch during AUTO mission; EKF resets cleanly |
| `ModeAltHold` | ALT_HOLD maintains **±1m** altitude under full stick inputs |
| `ModeLoiter` | LOITER holds position within **5m** horizontal, **5m** altitude for 10s |
| `SimpleMode` | SIMPLE mode: heading-independent control |
| `SuperSimpleCircle` | SUPER_SIMPLE mode with circle |
| `ModeCircle` | CIRCLE mode: 36s hold, circles at set radius |
| `MagFail` | Compass failure detection and EKF fallback |
| `OpticalFlow` | Optical flow navigation (no GPS) |
| `OpticalFlowLocation` | Optical flow position hold |
| `OpticalFlowLimits` | Optical flow speed limits enforced |
| `OpticalFlowCalibration` | Optical flow sensor calibration |
| `MotorFail` | Motor failure on octocopter: holds position, yaw error < 5°, alt drop < 20m |
| `ModeFlip` | FLIP mode: executes flip, recovers |
| `CopterMission` | Full copter mission file `copter_mission.txt`, all WPs completed |
| `TakeoffAlt` | Takeoff to specified altitude; tolerance tests |
| `SplineLastWaypoint` | Spline to last waypoint before land |
| `Gripper` | Gripper open/close via MAVLink |
| `TestLocalHomePosition` | Home position in local frame |
| `TestGripperMission` | Gripper triggered from AUTO mission |
| `VisionPosition` | External vision position (VICON) in GUIDED mode |
| `ATTITUDE_FAST` | ATTITUDE_QUATERNION at high rate |
| `BaseLoggingRates` | Default message rates are correct |
| `BodyFrameOdom` | Body-frame odometry updates EKF3 |
| `GPSViconSwitching` | Switch between GPS and VICON position sources |

### Batch: tests1e

| Test | Description |
|------|-------------|
| `BeaconPosition` | Beacon-based indoor positioning |
| `RTLSpeed` | RTL speed (RTLSPEED parameter) matches actual RTL groundspeed |
| `Mount` | Gimbal mount control modes |
| `MountYawVehicleForMountROI` | Vehicle yaws to track mount ROI |
| `MAV_CMD_DO_MOUNT_CONTROL` | Gimbal control via MAVLink command |
| `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` | Gimbal manager configuration |
| `AutoYawDO_MOUNT_CONTROL` | Auto yaw interacts with mount ROI |
| `MountPOIFromAuxFunction` | Mount points to POI via AUX switch |
| `Button` | Physical button events sent via BUTTON_CHANGE |
| `ShipTakeoff` | Takeoff from moving vessel (moving home) |
| `RangeFinder` | Rangefinder basic: RANGEFINDER vs GLOBAL_POSITION_INT within **1m** |
| `BaroDrivers` | Multiple baro driver tests |
| `SurfaceTracking` | Surface tracking mode (rangefinder alt hold) |
| `Parachute` | Parachute deployment (disabled — issue #4702) |
| `ParameterChecks` | PSC parameter validation pre-arm |
| `ManualThrottleModeChange` | Mode changes block when throttle not at minimum |
| `MANUAL_CONTROL` | MANUAL_CONTROL MAVLink message: pitch ≤ -12° achievable |
| `ModeZigZag` | ZigZag mode: A→B→A pattern |
| `PosHoldTakeOff` | POSHOLD takeoff: stays on ground until throttle input, rises to PILOT_TKO_ALT_M=7m (tolerance 6.9–8m, maintained 10s) |
| `ModeFollow` | FOLLOW mode: vehicle tracks target vehicle |
| `ModeFollow_with_FOLLOW_TARGET` | Follow via FOLLOW_TARGET MAVLink message |
| `RangeFinderDrivers` | Multiple RF driver types (SITL, I2C, UART) |
| `FlyRangeFinderMAVlink` | RF distance triggers altitude correction via MAVLink |
| `FlyRangeFinderSITL` | RF in SITL triggers altitude hold |
| `RangeFinderDriversMaxAlt_*` | Various RF drivers with max-alt limits |
| `RangeFinderDriversLongRange` | Long-range (>50m) RF driver |
| `RangeFinderSITLLongRange` | SITL long-range RF |
| `MaxBotixI2CXL` | MaxBotix I2C sonar driver |
| `MAVProximity` | Proximity via MAVLink OBSTACLE_DISTANCE |
| `ParameterValidation` | Bad parameter values rejected at pre-arm |
| `AltTypes` | Multiple altitude reference frames (absolute, relative, terrain) |
| `PAUSE_CONTINUE` | AUTO mission pause/resume via DO_PAUSE_CONTINUE |
| `PAUSE_CONTINUE_GUIDED` | Pause/continue in GUIDED |
| `RichenPower` | RichenPower generator integration |
| `IE24` | IE24 fuel cell integration |
| `LoweheiserAuto` | Loweheiser gas engine in AUTO |
| `LoweheiserManual` | Loweheiser in MANUAL |
| `MAVLandedStateTakeoff` | LANDED_STATE transitions during takeoff sequence |
| `Weathervane` | Weathervane yaw into wind |
| `MAV_CMD_AIRFRAME_CONFIGURATION` | Landing gear deploy/retract |
| `MAV_CMD_NAV_LOITER_UNLIM` | NAV_LOITER_UNLIM mission item |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | RTL via mission command |
| `MAV_CMD_NAV_VTOL_LAND` | VTOL land via mission (copter context) |
| `clear_roi` | Clear ROI command |
| `ReadOnlyDefaults` | Read-only parameter defaults cannot be overwritten |
| `FenceRelativePreArms` | Fence relative-to-home pre-arm checks |
| `FenceRelativeToHome*` | 10 fence reference-frame tests (max/min alt, origin above, cliff) |
| `FenceRelativeToOrigin*` | 4 fence relative-to-origin tests |
| `FenceRelativeToAMSL*` | 3 fence AMSL-reference tests |
| `FenceRelativeToTerrain*` | 2 fence terrain-reference tests |

### Batch: tests2a (compass calibration)

| Test | Description |
|------|-------------|
| `FixedYawCalibration` | Fixed-yaw compass calibration |
| `SITLCompassCalibration` | Full in-flight compass calibration (~8.5min) |

### Batch: tests2b (~130 tests, ~9.5 min)

| Test | Description |
|------|-------------|
| `MotorVibration` | Motor vibration FFT: peak at -15dB or more, 100–300 Hz |
| `DynamicNotches` | Harmonic notch filter: double/triple/quintuple notch reduces peak; each variant within 5% of previous |
| `PositionWhenGPSIsZero` | Handles GPS reporting lat=lon=0 gracefully |
| `DynamicRpmNotches` | ESC telemetry-driven notch filter validation |
| `DynamicRpmNotchesRateThread` | RPM notches on rate thread |
| `PIDNotches` | PID-output-driven notch filter |
| `mission_NAV_LOITER_TURNS` | Loiter turns in mission |
| `mission_NAV_LOITER_TURNS_off_center` | Off-center loiter turns |
| `StaticNotches` | Static frequency notch filter |
| `LuaParamLockdown` | Lua scripts cannot write locked parameters |
| `RefindGPS` | GPS lost and re-acquired mid-flight |
| `GyroFFT` | In-flight FFT: inject 250 Hz noise, detect within 5% with 128-bin FFT; motor noise peak ~175 Hz |
| `GyroFFTHarmonic` | FFT harmonic detection |
| `GyroFFTAverage` | FFT with averaging |
| `GyroFFTContinuousAveraging` | FFT with continuous averaging |
| `WPYawBehaviour1RTL` | WP_YAW_BEHAVIOR=1 during RTL |
| `GyroFFTPostFilter` | FFT after notch filter applied |
| `GyroFFTMotorNoiseCheck` | Motor noise in FFT |
| `CompassReordering` | Compass priority reordering |
| `SixCompassCalibrationAndReordering` | 6-compass setup calibration |
| `CRSF` | CRSF RC protocol support |
| `MotorTest` | Motor test via MAVLink |
| `AltEstimation` | No barometer → cannot enter ALT_HOLD; "Need Alt Estimate" pre-arm failure |
| `EKFSource` | EKF3 source pre-arm checks (bad yaw source, missing compass) |
| `GSF` | Gaussian sum filter yaw estimate |
| `GSF_reset` | GSF reset mid-flight |
| `AP_Avoidance` | ADSB-based avoidance |
| `RTL_ALT_FINAL_M` | RTL_ALT_FINAL parameter sets final descent altitude |
| `SMART_RTL` | Smart RTL traces path back to home |
| `SMART_RTL_EnterLeave` | Smart RTL enter/leave (disabled — panic) |
| `SMART_RTL_Repeat` | Smart RTL repeated (disabled — loop detection bug) |
| `RTL_TO_RALLY` | RTL goes to rally point, not home |
| `RTLYaw` | RTL yaw behaviour |
| `FlyEachFrame` | Fly every supported frame class/type in SITL |
| `ScriptParamRegistration` | Lua script registers new parameters |
| `GPSBlending` | Dual GPS blending: blended position halfway between two GPS offsets |
| `GPSWeightedBlending` | Weighted GPS blending |
| `GPSBlendingLog` | Blended GPS instance in dataflash (instances 0,1,2 all present) |
| `GPSBlendingAffinity` | GPS blending affinity settings |
| `DataFlash` | Dataflash logging functional |
| `DataFlashErase` | Flash erase with verify |
| `Callisto` | Callisto model-specific test |
| `PerfInfo` | Scheduler `tasks.txt` format: starts "TasksV2", ends "AP_Vehicle::update_arming" |
| `ModeAllowsEntryWhenNoPilotInput` | Certain modes can be entered without pilot input |
| `Replay` | Flight replay from dataflash |
| `FETtecESC` | FETtec ESC telemetry |
| `ProximitySensors` | Multiple proximity sensor driver tests |
| `GroundEffectCompensation_touchDownExpected` | Ground effect compensation on landing (disabled — flapping) |
| `GroundEffectCompensation_takeOffExpected` | Ground effect compensation on takeoff (disabled — flapping) |
| `DO_CHANGE_SPEED` | Speed change command in AUTO |
| `MISSION_START` | MAV_CMD_MISSION_START in various states |
| `AUTO_LAND_TO_BRAKE` | AUTO mission lands to BRAKE mode |
| `WP_SPEED` | WP_SPD parameter changes mid-mission affect speed |
| `RTLStoppingDistanceSpeed` | RTL stopping distance based on speed (disabled — vehicle goes off-course) |
| `WP_SPEED_UP` | WP_SPD_UP (climb rate) |
| `WP_SPEED_DN` | WP_SPD_DN (descent rate) |
| `DO_WINCH` | Winch servo command via mission |
| `SensorErrorFlags` | SYS_STATUS sensor error flags set correctly |
| `GPSForYaw` | Dual GPS for yaw estimation |
| `DefaultIntervalsFromFiles` | Message rate defaults loaded from SD files |
| `GPSTypes` | All GPS driver types in SITL |
| `MultipleGPS` | Multiple GPS instances |
| `WatchAlts` | Altitude watchdog monitoring |
| `GuidedEKFLaneChange` | EKF lane switch in GUIDED: altitude stays within **±2m** after lane switch (GPS source) |
| `Sprayer` | Spray system: pump/spinner channels at correct PWM values |
| `AutoContinueOnRCFailsafe` | AUTO mission continues despite RC failsafe with FS_OPTIONS |
| `EK3_RNG_USE_HGT` | EKF3 uses rangefinder for height when above threshold |
| `NoRC` | No RC on boot: operates without RC |
| `RCOverridesNoRCReceiver` | RC overrides work without physical RC |
| `TerrainDBPreArm` | Terrain database pre-arm checks |
| `ThrottleGainBoost` | THR_BOOST gain increase on descent |
| `ScriptMountPOI` | Lua mount-poi.lua: POI coordinates in statustext match home lat/lng |
| `ScriptMountAllModes` | All 7 mount modes (retract/neutral/mavlink/RC/GPS/sysid/home) tested via Lua |
| `ScriptCopterPosOffsets` | Lua copter-posoffset.lua: velocity offset 5 m/s, groundspeed 4.8–5.2 m/s |
| `MountSolo` | 3DR Solo gimbal |
| `MountSiyiZT30` | Siyi ZT30 gimbal |
| `MountTopotek` | Topotek gimbal |
| `MountViewPro` | ViewPro gimbal |
| `MountAVTCM62` | AVTCM62 gimbal |
| `MountAVTCM62Dual` | AVTCM62 dual gimbal |
| `FlyMissionTwice` | Mission re-run without rearm (disabled — PR #18561) |
| `FlyMissionTwiceWithReset` | Mission re-run with SITL reset |
| `MissionIndexValidity` | Invalid mission index commands rejected |
| `InvalidJumpTags` | DO_JUMP_TAG to non-existent tag rejected |
| `IMUConsistency` | SIM_ACC1_BIAS_X=5 → "Accels inconsistent"; SIM_GYR1_BIAS_X=10° → "Gyros inconsistent"; 8s to re-arm |
| `AHRSTrimLand` | AHRS trim of 0.12 rad doesn't confuse land detector |
| `IBus` | IBus RC protocol sensor values |
| `WaitAndMaintainAttitude_RCFlight` | RC inputs maintain specified attitude |
| `GuidedYawRate` | GUIDED yaw rate command |
| `RudderDisarmMidair` | Rudder-disarm rejected mid-air |
| `NoArmWithoutMissionItems` | Arming blocked in AUTO/GUIDED without mission |
| `DO_CHANGE_SPEED_in_guided` | Speed change in GUIDED mode |
| `ArmSwitchAfterReboot` | Arm via switch after reboot |
| `RPLidarA1` / `RPLidarA2` | RPLidar A1/A2 proximity sensor |
| `MISSION_OPTION_CLEAR_MISSION_AT_BOOT` | Mission cleared on reboot with option set |
| `SafetySwitch` | Hardware safety switch interaction |
| `RCProtocolFailsafe` | RC protocol failsafe detection |
| `BrakeZ` | BRAKE mode in Z axis (vertical braking) |
| `MAV_CMD_DO_FLIGHTTERMINATION` | Flight termination command |
| `MAV_CMD_DO_LAND_START` | DO_LAND_START mission item |
| `MAV_CMD_DO_SET_GLOBAL_ORIGIN` | Set GPS global origin; validates lat/lng/alt in GPS_GLOBAL_ORIGIN |
| `MAV_CMD_SET_EKF_SOURCE_SET` | EKF source selection via command |
| `MAV_CMD_NAV_TAKEOFF*` | Takeoff command variants (no location, with location, command_int) |
| `Ch6TuningWPSpeed` | CH6 tuning knob changes WP speed |
| `DualTuningChannels` | Two tuning channels simultaneously |
| `PILOT_THR_BHV` | Pilot throttle behavior flags |
| `GPSForYawCompassLearn` | GPS yaw with compass learning |
| `CameraLogMessages` | Camera trigger log messages |
| `LoiterToGuidedHomeVSOrigin` | LOITER→GUIDED transition home vs origin |
| `GuidedModeThrust` | GUIDED mode thrust setpoint |
| `CompassMot` | Compass motor interference calibration |
| `AutoRTL` | AUTO mission ends with RTL |
| `EK3_OGN_HGT_MASK_climbing` / `EK3_OGN_HGT_MASK` | EKF3 origin height mask |
| `FarOrigin` | Mission with GPS origin far from vehicle (NZ coords) |
| `GuidedForceArm` | Force arm in GUIDED mode |
| `GuidedWeatherVane` | Weathervane in GUIDED mode |
| `LUAConfigProfile` | Lua config profile switching |
| `Clamp` | Clamping mechanism deployment |
| `GripperReleaseOnThrustLoss` | Gripper releases on thrust loss |
| `GripperInitialPosition` | Gripper initial state correct |
| `REQUIRE_LOCATION_FOR_ARMING` | Arming blocked without valid position |
| `LoggingFormat` | Log file format validation |
| `MissionRTLYawBehaviour` | Yaw during RTL from mission |
| `BatteryInternalUseOnly` | Internal battery monitor not reported externally |
| `MAV_CMD_MISSION_START_p1_p2` | MISSION_START with p1/p2 start/end indices |
| `ScriptingAHRSSource` | Lua scripting sets AHRS source |
| `CommonOrigin` | Common local origin across EKF lanes |
| `AHRSOriginRecorded` | AHRS origin recorded in dataflash |
| `TestTetherStuck` | Tethered vehicle stuck detection |
| `ScriptingFlipMode` | Lua flip mode script |
| `RC_OPTIONS_1_FS_THR_ENABLE_0` | RC option 1 disables throttle failsafe |
| `ScriptingFlyVelocity` | Lua velocity control script |
| `Scripting6DoFMotors` | Lua 6DoF motor mixer |
| `EK3_EXT_NAV_vel_without_vert` | EKF3 external nav velocity without vertical component |
| `CompassLearnCopyFromEKF` | Compass offsets learned from EKF copied to params |
| `AHRSAutoTrim` | AHRS auto-trim function |
| `Ch6TuningLoitMaxXYSpeed` | CH6 tunes LOITER max XY speed |
| `IgnorePilotYaw` | IGNORE_PILOT_YAW option |
| `TestEKF3CompassFailover` | EKF3 switches compass when primary fails |
| `test_EKF3_option_disable_lane_switch` | EK3_OPTIONS=2 prevents lane switching; EK3_PRIMARY forces switch when re-enabled |
| `PLDNoParameters` | Precision landing with no parameters set |
| `PeriphMultiUARTTunnel` | Peripheral UART tunnel |
| `EKF3SRCPerCore` | EKF3 source configuration per-core |
| `UTMGlobalPosition` | UTM_GLOBAL_POSITION message: flight state ground→airborne, all required flags set |
| `UTMGlobalPositionWaypoint` | UTM next_lat/next_lon matches mission waypoint (epsilon=1 = 0.11m) |

### CAN Tests

| Test | Description |
|------|-------------|
| `CANGPSCopterMission` | GPS via CAN bus, fly full mission |
| `TestLogDownloadMAVProxyCAN` | Log download over CAN |
| `BattCANReplaceRuntime` | Swap CAN battery backend at runtime |
| `BattCANSplitAuxInfo` | CAN battery with split aux info |

---

## 3. ArduPlane Tests — Complete List

`arduplane.py` is 8,270 lines. Tests split into 3 batches (1a, 1b, 1c).

### Batch: tests1a

| Test | Description |
|------|-------------|
| *(base class tests)* | ~20 tests from vehicle_test_suite |
| `AuxModeSwitch` | Aux RC switch sets flight modes |
| `TestRCCamera` | RC camera trigger |
| `TestRCRelay` | RC relay output |
| `ThrottleFailsafe` | Throttle FS: goes via CIRCLE then RTL; SYS_STATUS receiver bits checked |
| `NeedEKFToArm` | Cannot arm without GPS/EKF: "AHRS: not using configured AHRS type" |
| `ThrottleFailsafeFence` | Throttle FS when outside fence |
| `NoShortFailsafe` | No short failsafe action with FS_SHORT_ACTN=3 |
| `SoaringClimbRate` | Soaring mode climb rate (disabled — bad sink rate) |
| `TestFlaps` | Flap servo deflection at correct PWM |
| `DO_CHANGE_SPEED` | Speed change in AUTO |
| `DO_REPOSITION` | DO_REPOSITION command moves to new location |
| `GuidedRequest` | GUIDED mode command from GCS |
| `MainFlight` | Comprehensive flight: takeoff, axial roll, inside loop, stabilize, ACRO, FBWB, CRUISE, RTL, LOITER, CIRCLE, AHRS2, mission |
| `TestGripperMission` | Gripper in AUTO mission |
| `Parachute` | Parachute deployment |
| `ParachuteSinkRate` | Parachute sink rate detection |
| `DO_PARACHUTE` | MAV_CMD_DO_PARACHUTE |
| `PitotBlockage` | Blocked pitot: ARSPD_USE→0 when ARSPD_RATIO=0.1, stays disabled at 1.0, re-enables at 2.0 |
| `AIRSPEED_AUTOCAL` | Airspeed autocal: status "Airspeed 1 calibrated" on boot |
| `RangeFinder` | Plane rangefinder basic |
| `FenceStatic` | Static polygon fence |
| `FenceRTL` | Fence breach → RTL |
| `FenceRTLRally` | Fence breach → RTL to rally |
| `FenceRetRally` | Fence return to rally |
| `FenceAltCeilFloor` | Altitude ceiling and floor fence |
| `FenceMinAltAutoEnable` | Min alt fence auto-enables |
| `FenceMinAltEnableAutoland` | Min alt fence enables autoland |
| `FenceMinAltAutoEnableAbort` | Min alt fence auto-enable abort |
| `FenceAutoEnableDisableSwitch` | Fence enable/disable via RC switch |
| `FenceCircleExclusionAutoEnable` | Circle exclusion fence auto-enable (speedup=20) |
| `FenceEnableDisableSwitch` | Fence on/off via RC |
| `FenceEnableDisableAux` | Fence on/off via AUX |
| `FenceBreachedChangeMode` | Mode change on fence breach |
| `FenceNoFenceReturnPoint` | No fence return point defined |
| `FenceNoFenceReturnPointInclusion` | No return point with inclusion zone |
| `FenceDisableUnderAction` | Fence disabled during breach action |
| `ADSBFailActionRTL` | ADSB threat → RTL failsafe |
| `ADSBResumeActionResumeLoiter` | ADSB clear → resume loiter |
| `SimADSB` | Simulated ADSB traffic |
| `Button` | Button events |
| `FRSkySPort` | FrSky S.Port telemetry |
| `FRSkyPassThroughStatustext` | FrSky passthrough statustext |
| `FRSkyPassThroughSensorIDs` | FrSky passthrough sensor IDs |
| `FRSkyMAVlite` | FrSky MAVlite protocol |
| `FRSkyD` | FrSky D-series telemetry |
| `LTM` | LTM telemetry protocol |
| `DEVO` | DEVO telemetry |
| `AdvancedFailsafe` | Advanced failsafe logic |
| `LOITER` | LOITER altitude maintained within **20m** over 4 circles |
| `MAV_CMD_NAV_LOITER_TURNS` | Loiter turns mission item |
| `MAV_CMD_NAV_LOITER_TO_ALT` | Loiter to altitude mission item |
| `DeepStall` | Deep-stall landing manoeuvre |
| `WatchdogHome` | Watchdog reboot resets home |
| `LargeMissions` | Large mission (>100 WPs) upload/download |
| `Soaring` | Thermal soaring mode |
| `Terrain` | Terrain following in AUTO |
| `TerrainMission` | Mission with terrain alt commands |
| `TerrainMissionInterrupt` | Interrupt terrain mission and resume |
| `UniversalAutoLandScript` | Lua universal autoland script |
| `Replay` | Plane flight replay |

### Batch: tests1b (~80 tests)

| Test | Description |
|------|-------------|
| `TerrainLoiter` | LOITER with terrain altitude |
| `VectorNavEAHRS` | VectorNav external AHRS |
| `MicroStrainEAHRS5/7` | MicroStrain EAHRS units |
| `InertialLabsEAHRS` | Inertial Labs EAHRS |
| `KebniSensAItionExternal*` | Kebni external INS/IMU |
| `GpsSensorPreArmEAHRS` | GPS sensor pre-arm with EAHRS |
| `Deadreckoning` | Dead reckoning without GPS |
| `EKFlaneswitch` | EKF lane switching |
| `AirspeedDrivers` | All airspeed sensor drivers |
| `RTL_CLIMB_MIN` | RTL minimum climb altitude |
| `ClimbBeforeTurn` | Climb before turn manoeuvre |
| `IMUTempCal` | IMU temperature calibration |
| `MAV_CMD_DO_AUX_FUNCTION` | AUX function via MAVLink command |
| `SmartBattery` | Smart battery integration |
| `FlyEachFrame` | Every plane frame type |
| `AutoLandMode` | Automatic landing mode |
| `RCDisableAirspeedUse` | RC channel disables airspeed |
| `AHRS_ORIENTATION` | AHRS orientation parameter |
| `AHRSTrim` | AHRS pitch/roll trim |
| `LandingDrift` | Landing drift (disabled — flapping) |
| `TakeoffAuto1-4` | Four AUTO takeoff scenarios |
| `TakeoffTakeoff1-5` | Five TAKEOFF mode scenarios |
| `TakeoffGround` | Ground takeoff |
| `TakeoffIdleThrottle` | Takeoff from idle throttle |
| `TakeoffBadLevelOff` | Bad level-off during takeoff |
| `TakeoffLevelOffWind` | Level-off in wind |
| `ForcedDCM` | Force DCM attitude estimator |
| `DCMFallback` | DCM fallback from EKF |
| `MAVFTP` | MAVLink FTP file transfer |
| `AUTOTUNE` | Plane autotune |
| `AutotuneFiltering` | Autotune with filtering |
| `MegaSquirt` | MegaSquirt EFI integration |
| `Hirth` | Hirth gas engine |
| `MSP_DJI` | DJI MSP OSD protocol |
| `SpeedToFly` | Speed-to-fly soaring |
| `AltitudeSlopeMaxHeight` | Altitude slope max height limit |
| `HIGH_LATENCY2` | HIGH_LATENCY2 telemetry message |
| `MidAirDisarmDisallowed` | Disarm rejected mid-air |
| `AerobaticsScripting` | Aerobatics via Lua script |
| `MANUAL_CONTROL` | MANUAL_CONTROL message |
| `RunMissionScript` | Lua mission script |
| `WindEstimates` | Wind triangle speed/direction estimates |
| `WindMessageSpeed` | WIND MAVLink message speed field |
| `AltResetBadGPS` | Altitude reset after bad GPS |
| `AirspeedCal` | Airspeed sensor calibration |
| `MissionJumpTags` | DO_JUMP_TAG in missions |
| `GCSFailsafe` | GCS heartbeat failsafe (speedup=8) |
| `SDCardWPTest` | Waypoints stored on SD card |
| `NoArmWithoutMissionItems` | No arm in AUTO without mission |
| `RudderArmedTakeoffRequiresNeutralThrottle` | Rudder arm needs neutral throttle |
| `MODE_SWITCH_RESET` | Mode switch reset on reboot |
| `ExternalPositionEstimate` | External position estimate (VISION_POSITION_ESTIMATE) |
| `SagetechMXS` | Sagetech MXS transponder |
| `MAV_CMD_GUIDED_CHANGE_ALTITUDE` | Guided altitude change command |
| `MAV_CMD_PREFLIGHT_CALIBRATION` | Preflight calibration commands |
| `MAV_CMD_DO_INVERTED_FLIGHT` | Inverted flight command |
| `MAV_CMD_DO_AUTOTUNE_ENABLE` | Enable autotune via command |
| `MAV_CMD_DO_GO_AROUND` | Go-around command during landing |
| `MAV_CMD_DO_FLIGHTTERMINATION` | Flight termination |
| `MAV_CMD_DO_LAND_START` | DO_LAND_START mission item |
| `MAV_CMD_NAV_ALTITUDE_WAIT` | Wait at altitude mission item |
| `InteractTest` | Interactive test (disabled — user interaction) |
| `CompassLearnInFlight` | In-flight compass learn |
| `MAV_CMD_MISSION_START` | Mission start command |
| `TerrainRally` | Rally point with terrain altitude |
| `MAV_CMD_NAV_LOITER_UNLIM` | Unlimited loiter |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | RTL command |
| `MinThrottle` | Minimum throttle in flight |
| `ClimbThrottleSaturation` | Throttle saturation during climb (disabled — needs PR #27106) |
| `GuidedAttitudeNoGPS` | GUIDED attitude mode without GPS |
| `ScriptStats` | Lua script statistics |
| `GPSPreArms` | GPS pre-arm checks |
| `SetHomeAltChange` / `*2` / `*3` | Home altitude change tests |
| `ForceArm` | Force arm command |
| `MAV_CMD_EXTERNAL_WIND_ESTIMATE` | External wind estimate command |
| `GliderPullup` | Glider pull-up manoeuvre |
| `BadRollChannelDefined` | Pre-arm fails with bad roll channel |
| `VolzMission` / `Volz` | Volz servo driver |
| `mavlink_AIRSPEED` | AIRSPEED MAVLink message |
| `LoggedNamedValueFloat/String` | Named value logging |
| `AdvancedFailsafeBadBaro` | Advanced failsafe with bad baro |
| `DO_CHANGE_ALTITUDE` | Altitude change command |
| `SET_POSITION_TARGET_GLOBAL_INT_for_altitude` | Position target for altitude only |
| `MAV_CMD_NAV_LOITER_TURNS_zero_turn` | Zero-turn loiter |
| `RudderArmingWithArmingChecksSkipped` | Rudder arm with checks skipped |
| `TerrainLoiterToCircle` | Terrain loiter transitioning to circle |
| `FenceDoubleBreach` | Double fence breach handling |
| `ScriptedArmingChecksApplet` / `*EStop` / `*Rally` | Lua arming check scripts |
| `PlaneFollowAppletSanity` | Plane follow Lua sanity |
| `PreflightRebootComponent` | Reboot component via MAVLink |
| `UTMGlobalPosition` / `UTMGlobalPositionWaypoint` | UTM position message tests |

### Batch: tests1c

| Test | Description |
|------|-------------|
| `DeadreckoningNoAirSpeed` | Dead reckoning without airspeed sensor |

---

## 4. ArduRover Tests — Complete List

`rover.py` is 7,580 lines.

| Test | Description |
|------|-------------|
| *(base class tests)* | ~20 from vehicle_test_suite |
| `MAVProxy_SetModeUsingSwitch` | Mode change via RC switch through MAVProxy |
| `HIGH_LATENCY2` | HIGH_LATENCY2 telemetry |
| `MAVProxy_SetModeUsingMode` | Mode change via MAVProxy |
| `ModeSwitch` | Mode switch via RC channel |
| `AuxModeSwitch` | AUX switch mode change |
| `DriveRTL` | RTL: returns to home |
| `SmartRTL` | Smart RTL path retracing |
| `DriveSquare` | 50m square via waypoint recording (Ch7 option), verify 7 WPs |
| `DriveMission` | Drive a full mission file |
| `MAV_CMD_DO_SEND_BANNER` | Banner text via MAVLink |
| `DO_SET_MODE` | DO_SET_MODE command |
| `MAVProxy_DO_SET_MODE` | DO_SET_MODE via MAVProxy |
| `ServoRelayEvents` | Servo/relay output events |
| `RCOverrides` | RC channel overrides |
| `RCOverridesCancel` | Cancel RC overrides |
| `MANUAL_CONTROL` | MANUAL_CONTROL message |
| `Sprayer` | Sprayer servo outputs |
| `AC_Avoidance` | Avoidance in AUTO |
| `CameraMission` | Camera trigger in mission |
| `Gripper` / `GripperMission` | Gripper in manual and mission |
| `SET_MESSAGE_INTERVAL` | Set message rate via MAVLink |
| `MESSAGE_INTERVAL_COMMAND_INT` | Message interval via COMMAND_INT |
| `REQUEST_MESSAGE` | Request specific MAVLink message |
| `MAV_GCS_ENFORCE` | GCS sysid enforcement |
| `SET_ATTITUDE_TARGET` | SET_ATTITUDE_TARGET heading control |
| `SET_ATTITUDE_TARGET_heading` | Heading via SET_ATTITUDE_TARGET |
| `SET_POSITION_TARGET_LOCAL_NED` | Position target in local frame |
| `MAV_CMD_DO_SET_MISSION_CURRENT` | Jump to mission index |
| `MAV_CMD_DO_CHANGE_SPEED` | Speed change |
| `MAV_CMD_MISSION_START` | Mission start |
| `MAV_CMD_NAV_SET_YAW_SPEED` | Yaw+speed command (disabled — compiled out) |
| `Button` | Button events |
| `Rally` | Rally point navigation |
| `Offboard` | Offboard/GUIDED mode via SET_POSITION_TARGET |
| `MAVProxyParam` | Parameter get/set via MAVProxy |
| `GCSFence` | Fence upload/download via GCS |
| `GCSFenceInvalidPoint` | Invalid fence point rejected |
| `GCSMission` | Mission upload/download via GCS |
| `MotorTest` | Motor test |
| `WheelEncoders` | Wheel encoder odometry |
| `DataFlashOverMAVLink` | Dataflash log over MAVLink |
| `DataFlash` | Dataflash logging |
| `SkidSteer` | Skid-steer drive test |
| `PolyFence` | Polygon fence |
| `SDPolyFence` | SD card polygon fence |
| `PolyFenceAvoidance` | Polygon fence avoidance |
| `PolyFenceObjectAvoidanceAuto` | Object avoidance in AUTO |
| `PolyFenceObjectAvoidanceGuided` | Object avoidance in GUIDED |
| `PolyFenceObjectAvoidanceBendyRuler` | BendyRuler avoidance (disabled — unreliable) |
| `SendToComponents` | MAVLink to components |
| `PolyFenceObjectAvoidanceBendyRulerEasierGuided` | Easier BendyRuler in GUIDED |
| `PolyFenceObjectAvoidanceBendyRulerEasierAuto` | Easier BendyRuler in AUTO |
| `SlewRate` | Steering slew rate limit (disabled — CI timing) |
| `Scripting` | Lua scripting |
| `ScriptingSteeringAndThrottle` | Lua steering+throttle control |
| `MissionFrames` | Mission items in various MAVLink frames |
| `SetpointGlobalPos` | Global position setpoint |
| `SetpointGlobalVel` | Global velocity setpoint |
| `AccelCal` | Accelerometer calibration |
| `RangeFinder` | Rangefinder basic |
| `AIS` | AIS maritime transponder |
| `AISMultipleVessels` | Multiple AIS vessels |
| `AISDataValidation` | AIS data field validation |
| `AP_Proximity_MAV` | MAVLink proximity sensor |
| `EndMissionBehavior` | Mission end: hold/RTL/disarm |
| `PositionTargetGlobalIntAltFrame` | Position target in global alt frame |
| `FlashStorage` / `FRAMStorage` | Flash/FRAM parameter storage |
| `DepthFinder` | Depth finder (boat mode) |
| `ChangeModeByNumber` | Change mode using numeric index |
| `EStopAtBoot` | E-stop active at boot |
| `MAV_CMD_NAV_RETURN_TO_LAUNCH` | RTL command |
| `StickMixingAuto` | Stick mixing in AUTO |
| `AutoDock` | Automatic docking |
| `PrivateChannel` | Private MAVLink channel |
| `GCSFailsafe` | GCS failsafe |
| `RoverInitialMode` | Initial mode after boot |
| `DriveMaxRCIN` | Max RC input drive |
| `NoArmWithoutMissionItems` | No arming in AUTO without mission |
| `PARAM_ERROR` | PARAM_ERROR message for non-existent param |
| `CompassPrearms` | Compass pre-arm checks |
| `ManyMAVLinkConnections` | Multiple simultaneous GCS connections |
| `MAV_CMD_DO_SET_REVERSE` | Drive in reverse |
| `MAV_CMD_GET_HOME_POSITION` | Get home position command |
| `MAV_CMD_DO_FENCE_ENABLE` | Enable/disable fence via command |
| `MAV_CMD_BATTERY_RESET` | Battery reset command |
| `GPSForYaw` | GPS dual-antenna yaw |
| `NetworkingWebServer` | Built-in web server |
| `NetworkingWebServerPPP` | Web server over PPP |
| `RTL_SPEED` | RTL speed setting |
| `ScriptingLocationBindings` | Lua location binding functions |
| `MissionRetransfer` | Mission re-upload robustness |
| `FenceFullAndPartialTransfer` | Full/partial fence upload |
| `MissionPolyEnabledPreArm` | Poly fence enabled pre-arm check |
| `OpticalFlow` | Optical flow |
| `RCDuplicateOptionsExist` | Duplicate RC option detection |
| `ClearMission` | Clear mission via protocol |
| `JammingSimulation` | GPS jamming simulation |
| `BatteryInvalid` | Invalid battery reading handling |
| `REQUIRE_LOCATION_FOR_ARMING` | Location required to arm |
| `GetMessageInterval` | GET_MESSAGE_INTERVAL command |
| `SafetySwitch` | Safety switch |
| `ThrottleFailsafe` | Throttle failsafe: LOITER, then HOLD on GPS loss |
| `DriveEachFrame` | All rover frame types |
| `AP_ROVER_AUTO_ARM_ONCE_ENABLED` | Auto-arm behavior |
| `GPSAntennaPositionOffset` | GPS antenna lever-arm offset |
| `UTMGlobalPosition` / `UTMGlobalPositionWaypoint` | UTM message tests |

---

## 5. ArduSub Tests — Complete List

`ardusub.py` is 1,399 lines.

| Test | Description |
|------|-------------|
| *(base class tests)* | ~20 from vehicle_test_suite |
| `DiveManual` | Manual dive: throttle input drives depth |
| `GCSFailsafe` | GCS heartbeat loss failsafe |
| `ThrottleFailsafe` | Throttle failsafe |
| `AltitudeHold` | ALT_HOLD mode: holds altitude ±0.3m over 5s; tests positive/negative buoyancy (SIM_BUOYANCY=±10) |
| `Surftrak` | Surface tracking: rangefinder maintains distance from seabed |
| `SimTerrainSurftrak` | Terrain-based surface tracking |
| `SimTerrainMission` | Mission with terrain-following |
| `RngfndQuality` | Rangefinder quality via Lua script |
| `PositionHold` | POSHOLD mode |
| `ModeChanges` | Mode change cycle with delta=0.2 |
| `MAV_mgs` | MAVLink message rate checks |
| `DiveMission` | Automated dive mission |
| `GripperMission` | Gripper in mission |
| `DoubleCircle` | Two consecutive circles |
| `MotorThrustHoverParameterIgnore` | MOT_THST_HOVER param ignored by sub |
| `SET_POSITION_TARGET_GLOBAL_INT` | Position target command |
| `TestLogDownloadMAVProxy` / `Network` / `Restart` | Log download variants |
| `MAV_CMD_NAV_LOITER_UNLIM` | Unlimited loiter |
| `MAV_CMD_NAV_LAND` | Land command (surface) |
| `MAV_CMD_MISSION_START` | Mission start |
| `MAV_CMD_DO_CHANGE_SPEED` | Speed change |
| `MAV_CMD_CONDITION_YAW` | Yaw condition |
| `MAV_CMD_DO_REPOSITION` | Reposition command |
| `TerrainMission` | Terrain mission |
| `SetGlobalOrigin` / `BackupOrigin` | Origin set/backup |
| `FuseMag` | Magnetometer fusion |
| `INA3221` | INA3221 power monitor |
| `PosHoldBounceBack` | POSHOLD bounce-back behaviour |
| `SHT3X` | SHT3X environmental sensor |
| `SurfaceSensorless` | Surface detection without sonar |
| `GPSForYaw` / `VisoForYaw` | GPS/VISO dual yaw |
| `WaterDepth` | WATER_DEPTH message rate at 2 Hz |
| `UTMGlobalPosition` / `UTMGlobalPositionWaypoint` | UTM message tests |

---

## 6. Common Test Patterns and Tolerances

### Position Accuracy

| Context | Tolerance | Method |
|---------|-----------|--------|
| `wait_distance()` default | ±2m | `accuracy=2` in `wait_and_maintain()` |
| `wait_location()` | configurable | `WaitAndMaintainLocation` class |
| `wait_waypoint()` final WP | `wp_dist < max_dist=2m` | NAV_CONTROLLER_OUTPUT |
| GPS glitch loiter drift | 20m max | manual position check |
| Precision landing | 1.5m max | `get_distance(target, landed_pos)` |
| RTL home return | `distance < 10m` and `alt <= 1m` | manual check in `wait_rtl_complete()` |
| EKF lane change altitude | ±2m | `watch_altitude_maintained` with GPS source |

### Altitude Hold Tolerances

| Mode | Tolerance | Method |
|------|-----------|--------|
| LOITER default | ±5m | `maxaltchange=5` in `ModeLoiter()` |
| ALT_HOLD | ±1m (9–11m band at 10m) | `watch_altitude_maintained(9, 11)` |
| ArduSub ALT_HOLD | ±0.3m | `watch_altitude_maintained(delta=0.3)` |
| Baro wind correction | <0.5m variation over 30s | `z_max - z_min` check |
| PosHoldTakeOff | 6.9–8m at 7m target, for 10s | `wait_altitude(6.9, 8, minimum_duration=10)` |

### Heading Tolerance

| Context | Tolerance |
|---------|-----------|
| `wait_heading()` default | ±5° (`accuracy=5`) |
| Level flight (plane) | ±5° roll and pitch |
| LOITER circles | ±10° heading |
| CIRCLE mode | ±10° heading |
| Attitude tolerance | ±1° for `wait_attitude(tolerance=1)` |
| `wait_attitude()` default | ±10° |

### Mode Transitions

Tested via `wait_mode(mode, timeout=60)`:
- Mode commanded via `change_mode()` → `MAV_CMD_DO_SET_MODE`
- HEARTBEAT custom_mode field polled
- Timeout 60s default; failsafe mode changes use 30s

### Failsafe Trigger and Response

Pattern used universally:
1. Set `SIM_RC_FAIL=1` (no-pulses) or `SIM_RC_FAIL=2` (throttle-to-950)
2. `wait_mode("RTL")` or `wait_mode("LAND")` — confirms failsafe triggered
3. `wait_landed_and_disarmed()` or `wait_rtl_complete()` — confirms full recovery
4. `SIM_RC_FAIL=0` to restore

RC failsafe options: `FS_THR_ENABLE` = 0 (disabled), 1 (RTL), 3 (Land), 4 (SmartRTL→RTL), 5 (SmartRTL→Land)

### Arming and Disarming

```
arm_vehicle(timeout=20, force=False):
  MAV_CMD_COMPONENT_ARM_DISARM(p1=1, p2=0)  # p2=2989 for force
  wait_armed(timeout=20)  # polls HEARTBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED

disarm_vehicle(timeout=60, force=False):
  MAV_CMD_COMPONENT_ARM_DISARM(p1=0, p2=0)  # p2=21196 for force
  wait_disarmed(timeout=30)
```

### Mission Execution

```
load_mission("filename.txt")  # uploads via mission item protocol
set_current_waypoint(1)
change_mode("AUTO")
arm_vehicle()
set_rc(3, 1550)  # or leave throttle mid
wait_waypoint(0, num_wp-1, timeout=400)  # max_dist=2m default
wait_disarmed()
```

Mission WP completion: `wp_dist < 2m` from NAV_CONTROLLER_OUTPUT

### GPS Loss Behavior

Tests inject `SIM_GPS1_ENABLE=0`:
- EKF goes unhealthy after ~5s
- RTL falls back to LAND if no position: `wait_mode("LAND")`
- SmartRTL deactivates with "SmartRTL deactivated: bad position" statustext

---

## 7. Key Utility Functions

### `wait_altitude(altitude_min, altitude_max, relative=False, timeout=30)`
- Waits for altitude in `[altitude_min, altitude_max]` range
- Source: `GLOBAL_POSITION_INT.alt` (absolute) or `.relative_alt` (relative)
- Uses `wait_and_maintain_range()` internally
- Default timeout: 30s

### `wait_location(loc, **kwargs)`
- `WaitAndMaintainLocation` object
- Accepts `epsilon` (distance tolerance in metres), `minimum_duration`, `timeout`

### `wait_heading(heading, accuracy=5, timeout=30)`
- Reads `VFR_HUD.heading`
- `heading_delta()` computes wrapped difference
- Default accuracy: ±5°

### `wait_mode(mode, timeout=60)`
- Polls `HEARTBEAT.custom_mode` against mode mapping
- Default timeout: 60s
- Raises `WaitModeTimeout` on failure

### `change_mode(mode, timeout=60)`
- Sends `MAV_CMD_DO_SET_MODE` repeatedly until confirmed
- Polls via `mode_is()` check

### `arm_vehicle(timeout=20, force=False)`
- `MAV_CMD_COMPONENT_ARM_DISARM(1, p2=0 or 2989)`
- Validates with `wait_armed()` (polls HEARTBEAT, 20s timeout)

### `disarm_vehicle(timeout=60, force=False)`
- `MAV_CMD_COMPONENT_ARM_DISARM(0, p2=0 or 21196)`
- Validates with `wait_disarmed()` (default 30s)

### `takeoff(alt_min=30, mode="STABILIZE", timeout=120, max_err=5)`
- Changes mode, waits ready, arms
- Copter: sets throttle to 1700 PWM
- Waits `altitude in [alt_min-1, alt_min+max_err]`
- Default target: 30m

### `do_RTL(distance_max=10, timeout=250)`
- Changes to RTL, zeroes throttle
- `wait_rtl_complete()`: polls until `alt <= 1m` AND `home_distance < 10m` AND disarmed

### `wait_groundspeed(speed_min, speed_max, timeout=30)`
- Reads `VFR_HUD.groundspeed`
- Range check with `wait_and_maintain_range()`

### `wait_distance(distance, accuracy=2, timeout=30)`
- Calculates distance from start location
- Default accuracy: ±2m

### `wait_waypoint(wpnum_start, wpnum_end, max_dist=2, timeout=400)`
- Monitors `NAV_CONTROLLER_OUTPUT.wp_dist` and current WP sequence
- Fails if mode exits AUTO (unless `ignore_RTL_mode_change=True`)
- Completion: `wp_dist < 2m`

### `set_parameter(name, value)` / `get_parameter(name)`
- PARAM_SET / PARAM_REQUEST_READ via MAVLink
- `set_parameters(dict)` sets multiple at once, verifies all written (epsilon_pct=0.00001)

### `assert_receive_message(msg_type, timeout=10)`
- Raises `MsgRcvTimeoutException` if no message in timeout
- Clears message queue before waiting

---

## 8. Edge Case and Failsafe Tests — Detail

### ThrottleFailsafe (ArduCopter) — 12+ sub-tests

1. Pre-takeoff RC fail → instant disarm (DISARM_DELAY=0)
2. `FS_THR_ENABLE=0`: no action on RC fail
3. Recovery test: RC fail → RTL → restore RC → change modes works
4. `FS_THR_ENABLE=1`: RTL, completes
5. `FS_THR_ENABLE=3`: LAND, completes
6. `FS_THR_ENABLE=4`: SMART_RTL, completes
7. `FS_THR_ENABLE=5`: SMART_RTL→LAND, completes
8. GPS fail + RC fail → RTL falls to LAND
9. GPS fail + SmartRTL → falls to LAND (bad position)
10. GPS fail/restore + SmartRTL fail → RTL (no path)
11. GPS fail/restore + SmartRTL→LAND fail → LAND (no path)
12. Additional FS_OPTIONS combinations (4,8,16,32,64)

### BatteryFailsafe (ArduCopter) — 7+ sub-tests

Voltage thresholds: `BATT_LOW_VOLT=11.5V`, `BATT_CRT_VOLT=10.1V`, sim at 12.5V normal

1. Disabled failsafe: `FS_LOW_ACT=0`, voltage → 11.4V → "Battery 1 is low" statustext, stays in ALT_HOLD
2. Two-stage RTL+Land: Low→RTL (11.4V), Critical→LAND (10.0V), waits disarmed
3. Two-stage SmartRTL: Low→SMART_RTL, Critical→SMART_RTL, waits disarmed
4. FS_OPTIONS=8 (continue landing): already LAND, battery failsafe doesn't interrupt
5. Critical landing with RC fail: LAND continues despite RC fail (RC failsafe suppressed)
6. Brake/Land at 7: low battery → BRAKE mode, groundspeed 8–10 m/s beforehand
7. Additional SMART_RTL and HOLD interactions

Battery charge states verified via `BATTERY_STATUS.charge_state`:
- Normal: `MAV_BATTERY_CHARGE_STATE_OK`
- Low: `MAV_BATTERY_CHARGE_STATE_LOW`
- Critical: `MAV_BATTERY_CHARGE_STATE_CRITICAL`

### GCSFailsafe (ArduCopter)
- Stop sending heartbeats → wait_mode("ALT_HOLD") for short failsafe
- Long absence → wait_mode("RTL")
- Restore heartbeat → recover

### MotorFail (ArduCopter)
- Octocopter only; `SIM_ENGINE_FAIL = 1 << servo_num`, `SIM_ENGINE_MUL=0`
- Monitors for 30s hold time
- Pass criteria: `int_error_yaw < 5°`, `alt_delta > -20m` (not descending more than 20m)

### GPSGlitchLoiter
- 7 glitch positions applied progressively over 30s
- After all glitches: `moved_distance < 20m` from start

### MotorVibration
- `SIM_VIB_MOT_MAX=350`, `SIM_GYR1_RND=20`
- FFT peak at 100–300 Hz, peak amplitude ≥ -15 dB

### IMUConsistency
- `SIM_ACC1_BIAS_X=5` → "Accels inconsistent" pre-arm failure
- `SIM_GYR1_BIAS_X=10°` → "Gyros inconsistent"
- Recovery requires ≥8 seconds after removing fault

### EKF3 Lane Switch
- `test_EKF3_option_disable_lane_switch`: EK3_OPTIONS=2 blocks automatic switch
- Re-enable (EK3_OPTIONS=0) + set EK3_PRIMARY=1 → forces switch to lane 1

### Sensor Failure Injection Parameters

| Parameter | Effect |
|-----------|--------|
| `SIM_RC_FAIL=1` | No-pulses RC failure |
| `SIM_RC_FAIL=2` | Throttle-to-950 failure |
| `SIM_GPS1_ENABLE=0` | Disable primary GPS |
| `SIM_GPS1_GLTCH_X/Y` | GPS position glitch (lat/lon offset) |
| `SIM_GPS2_GLTCH_Z` | GPS altitude glitch |
| `SIM_BATT_VOLTAGE` | Set simulated battery voltage |
| `SIM_BARO_DISABLE=1` | Disable primary barometer |
| `SIM_BAR2_DISABLE=1` | Disable secondary barometer |
| `SIM_ACC1_BIAS_X` | IMU accelerometer bias injection |
| `SIM_GYR1_BIAS_X` | IMU gyroscope bias injection |
| `SIM_VIB_MOT_MAX` | Motor vibration amplitude |
| `SIM_GYR1_RND` | Gyroscope noise |
| `SIM_ENGINE_FAIL` | Motor failure bitmask |
| `SIM_ENGINE_MUL` | Motor efficiency multiplier |
| `SIM_WIND_SPD/DIR` | Simulated wind speed/direction |
| `SIM_BUOYANCY` | Sub buoyancy force (N) |

---

## 9. Performance Benchmarks

### Loop Rate (PerfInfo test)
- `tasks.txt` from `@SYS/tasks.txt` via FTP
- Format: "TasksV2" header, last entry "AP_Vehicle::update_arming"
- `SCHED_OPTIONS=1` enables gathering

### FFT Frequency Resolution
- 128-bin FFT at 1000 Hz sample rate → 8 Hz/bin
- 250 Hz peak detectable within 5% (±12.5 Hz) per GyroFFT test
- 256-bin FFT tested for uint8_t boundary bugs

### EKF Convergence
- `wait_ekf_happy()` polls SYS_STATUS for EKF health bits
- GPS lock: typically 10–20s after boot in SITL
- EKF3 lane switch: `wait_statustext("EKF3 lane switch 1", timeout=10)`

### Wind Estimation
- `BaroWindCorrection`: wind speed ±1 m/s, direction ±15°
- After 30s yaw-spinning at 10m altitude

### GPS Blending
- Dual GPS with ±1m offset in X/Y
- Blended solution verified to be ~midpoint between two GPS positions

---

## 10. QuadPlane Tests

`quadplane.py` — VTOL-specific tests:

| Test | Description |
|------|-------------|
| `AirMode` | Air mode prevents disarm on motor idle |
| `TestMotorMask` | VTOL motor mask configuration |
| `QAUTOTUNE` | QuadPlane autotune in QLOITER |
| `GyroFFT` | FFT for QuadPlane |
| `PIDTuning` | PID tuning channels |
| `BootInAUTO` | Boot directly into AUTO mode |
| `PilotYaw` | Pilot yaw in VTOL hover |
| `FwdThrInVTOL` | Forward thrust during VTOL hover |
| `Weathervane` | Weathervane in VTOL |
| `CPUFailsafe` | CPU lockup copies RC to servos (plane behavior) |
| `QAssist` | Q-assist: VTOL motors help fixed-wing when too slow |
| `LoiterAltQLand` | Loiter altitude into Q-land |
| `GUIDEDToAUTO` | Transition GUIDED→AUTO |
| `Tailsitter` | Tailsitter frame type |
| `CopterTailsitter` | Copter-style tailsitter |
| `ICEngine` / `ICEngineMission` / `ICEngineRPMGovernor` | Internal combustion engine |
| `Ship` | Ship-moving-platform takeoff |
| `MidAirDisarmDisallowed` | Disarm rejected mid-air |
| `Mission` | Full QuadPlane AUTO mission |
| `VTOLLandSpiral` | Spiral descent VTOL landing |
| `VTOLQuicktune` | Quick VTOL tune |
| `PrecisionLanding` | VTOL precision landing |
| `ShipLanding` | Land on moving ship |
| `RCDisableAirspeedUse` | RC disables airspeed |
| `TransitionMinThrottle` | Min throttle during transition |
| `BackTransitionMinThrottle` | Min throttle back-transition |
| `DCMClimbRate` | DCM climb rate estimation |
| `AHRSFlyForwardFlag` | AHRS fly-forward flag |
| `WindEstimateConsistency` | Wind estimate consistency |
| `QLoiterRecovery` | QLOITER recovery from upset |
| `CruiseRecovery` | Cruise recovery |
| `FastInvertedRecovery` | Fast inverted flight recovery |
| `DoRepositionTerrain` | DO_REPOSITION with terrain |
| `RudderArmedTakeoffRequiresNeutralThrottle` | Rudder arm neutral throttle |
| `ScriptedArmingChecksApplet` | Scripted arming checks |
| `TerrainAvoidApplet` | Terrain avoidance Lua applet |
| `TakeoffCheck` | QuadPlane takeoff checks |

---

## 11. Helicopter Tests

`helicopter.py`:

| Test | Description |
|------|-------------|
| `RotorRunup` | Rotor runup sequence (RSC_SETPOINT) |
| `AVCMission` | AVC competition mission |
| `FlyEachFrame` | All helicopter frame types |
| `PosHoldTakeOff` | POSHOLD takeoff |
| `StabilizeTakeOff` | STABILIZE takeoff |
| `SplineWaypoint` | Spline waypoints (timeout=600) |
| `Autorotation` | Engine-off autorotation (timeout=600) |
| `AutorotationPreArm` | Autorotation pre-arm checks |
| `ManAutorotation` | Manual autorotation |
| `TestAutorotationConfig` | Autorotation config variants |
| `NastyMission` | Challenging mission profile |
| `MountFailsafeAction` | Mount failsafe |
| `AirspeedDrivers` | Airspeed sensors (timeout=600) |
| `TurbineCoolDown` | Turbine cool-down sequence |
| `TurbineStart` | Turbine start sequence |
| `PIDNotches` | PID notch filters |
| `AutoTune` | Helicopter autotune |

---

## 12. What Meridian Should Port — Priority Ordering

### Tier 1: Must Have (core flight safety)

These test the exact behaviors Meridian's control loops must implement correctly:

1. **`ModeAltHold`** — ±1m altitude hold under full stick deflection
2. **`ModeLoiter`** — ±5m position, ±5m altitude for 10s
3. **`ThrottleFailsafe`** — all RC fail modes (RTL, LAND, SmartRTL)
4. **`BatteryFailsafe`** — two-stage voltage thresholds with correct mode transitions
5. **`GCSFailsafe`** — heartbeat loss response
6. **`arm_vehicle/disarm_vehicle`** — full arming sequence with pre-arm checks
7. **`takeoff()`** — takeoff to altitude within max_err=5m
8. **`do_RTL()`** — return to within 10m, land within 1m altitude
9. **`CopterMission`** — fly complete AUTO mission waypoints
10. **`HorizontalFence`** — breach detection → RTL; pre-arm outside fence

### Tier 2: High Priority (navigation quality)

11. **`GPSGlitchLoiter`** — glitch resilience, <20m drift
12. **`PrecisionLanding`** — <1.5m landing accuracy
13. **`MotorFail`** — motor out survival, <5° yaw, <20m altitude loss
14. **`EKFSource`** — pre-arm EKF source validation
15. **`AltEstimation`** — no-baro pre-arm failure
16. **`IMUConsistency`** — accel/gyro consistency checks with 8s re-arm delay
17. **`GuidedEKFLaneChange`** — lane switch altitude stability ±2m
18. **`GPSBlending`** — dual-GPS blended position accuracy
19. **`BaroWindCorrection`** — wind speed ±1 m/s, direction ±15°, height <0.5m

### Tier 3: Important (modes and sensors)

20. **`ModeCircle`** / **`SuperSimpleCircle`** / **`SimpleMode`** — mode-specific behaviors
21. **`Landing`** — descent rate transitions at correct altitudes
22. **`AutoTune`** — gain convergence and discard on Land
23. **`RangeFinder`** — RANGEFINDER vs GLOBAL_POSITION_INT <1m match
24. **`PosHoldTakeOff`** — delayed takeoff, altitude 6.9–8m held 10s
25. **`MotorVibration`** — FFT motor peak detection
26. **`DynamicNotches`** — harmonic notch filter effectiveness

### Tier 4: Advanced (long-tail)

27. **`GPSForYaw`** — dual GPS yaw
28. **`VisionPosition`** — VICON external position
29. **`BeaconPosition`** — indoor beacon nav
30. **`Replay`** — dataflash replay fidelity
31. **`FlyEachFrame`** — all frame types

---

## 13. Test Infrastructure Meridian Needs

### Option A: Run ArduPilot Python Tests Against Meridian SITL (Recommended Short-Term)

Meridian exposes a MAVLink endpoint. The ArduPilot Python test scripts can connect to Meridian's SITL instead of ArduPilot's if Meridian implements:

1. **MAVLink heartbeat** with correct `MAV_TYPE`, `MAV_AUTOPILOT`, `base_mode` and `custom_mode` fields
2. **GLOBAL_POSITION_INT** — lat, lon, alt, relative_alt (mm-scale)
3. **VFR_HUD** — airspeed, groundspeed, heading, throttle, alt, climb
4. **ATTITUDE** — roll, pitch, yaw, rollspeed, pitchspeed, yawspeed (radians)
5. **SYS_STATUS** — sensor present/enabled/healthy bitmasks
6. **MAV_CMD_COMPONENT_ARM_DISARM** — arm/disarm command handler
7. **MAV_CMD_DO_SET_MODE** — mode change handler
8. **SERVO_OUTPUT_RAW** — motor PWM outputs
9. **NAV_CONTROLLER_OUTPUT** — wp_dist
10. **STATUSTEXT** — error/warning messages
11. **BATTERY_STATUS** — charge_state field
12. **WIND** — wind speed/direction

Parameters needed: `set_parameter` / `get_parameter` via PARAM_SET / PARAM_REQUEST_READ.

Simulation injection needed: `SIM_RC_FAIL`, `SIM_GPS1_ENABLE`, `SIM_BATT_VOLTAGE`, `SIM_GPS1_GLTCH_X/Y`, `SIM_ACC1_BIAS_X`, `SIM_GYR1_BIAS_X`, `SIM_VIB_MOT_MAX`, `SIM_ENGINE_FAIL/MUL`.

### Option B: Native Rust Test Framework

For tests that can't be driven via MAVLink alone (e.g., FFT tests that need internal data), Meridian needs:

```rust
// Test harness structure
#[cfg(test)]
mod tests {
    use crate::sitl::Sitl;
    use crate::mavlink::MavlinkTestClient;

    #[test]
    fn test_mode_alt_hold() {
        let mut sitl = Sitl::new();
        let mut mav = MavlinkTestClient::connect("localhost:5760");
        mav.wait_ready_to_arm(timeout_s=20);
        mav.arm_vehicle();
        mav.takeoff(alt_m=10.0, mode="ALT_HOLD");
        mav.watch_altitude_maintained(min=9.0, max=11.0, duration_s=5.0);
        // inject full stick
        mav.set_rc(1, 1000); mav.set_rc(2, 1000);
        mav.watch_altitude_maintained(min=9.0, max=11.0, duration_s=3.0);
        mav.do_rtl();
    }
}
```

### Key Parameters for Meridian SITL

The Python tests rely on these being settable:

```
FENCE_ENABLE, FENCE_TYPE, FENCE_RADIUS, FENCE_ALT_MAX, FENCE_ALT_MIN
FS_THR_ENABLE, FS_OPTIONS, FS_ACTION (rover)
BATT_LOW_VOLT, BATT_CRT_VOLT, BATT_FS_LOW_ACT, BATT_FS_CRT_ACT
EK3_ENABLE, EK3_SRC1_*, EK3_OPTIONS, EK3_PRIMARY
INS_HNTCH_ENABLE, INS_HNTCH_FREQ, INS_HNTCH_ATT, INS_HNTCH_BW
GPS_AUTO_SWITCH, GPS2_TYPE, SIM_GPS2_ENABLE
WP_SPD, WP_SPD_UP, WP_SPD_DN, RTLSPEED
PILOT_TKO_ALT_M, LAND_SPD_MS, LAND_ALT_LOW_M, LAND_SPD_HIGH_MS
```

---

## 14. Files Audited

| File | Lines | Role |
|------|-------|-------|
| `Tools/autotest/autotest.py` | 1,269 | Test runner, build scripts |
| `Tools/autotest/vehicle_test_suite.py` | ~50,000 | Base class, ALL utility functions |
| `Tools/autotest/arducopter.py` | 16,496 | Copter tests (~200 tests) |
| `Tools/autotest/arduplane.py` | 8,270 | Plane tests (~160 tests) |
| `Tools/autotest/rover.py` | 7,580 | Rover tests (~100 tests) |
| `Tools/autotest/ardusub.py` | 1,399 | Sub tests (~40 tests) |
| `Tools/autotest/quadplane.py` | ~3,100 | QuadPlane tests (~40 tests) |
| `Tools/autotest/helicopter.py` | ~1,100 | Helicopter tests (~20 tests) |
| `Tools/autotest/pysim/vehicleinfo.py` | — | SITL vehicle model registry |
| `Tools/autotest/pysim/util.py` | — | Build/process utilities |
