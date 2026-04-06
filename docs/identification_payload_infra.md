# Meridian Parity Identification: Payload Systems + Infrastructure
## Full Library-by-Library Analysis

**Date**: 2026-04-02  
**Source**: `D:\projects\ardupilot\libraries\`  
**Meridian root**: `D:\projects\meridian\`  
**Scope**: 47 libraries — payload, infrastructure, storage, calibration, specialty

---

## How to Read This Document

- **MERIDIAN HAS**: `YES` = substantive implementation exists; `STUB` = types/structs defined but no real logic; `NO` = crate does not exist
- **PRIORITY**: `CRITICAL` = needed for flight; `HIGH` = needed for production parity; `MEDIUM` = useful but deferrable; `LOW` = niche use case; `SKIP` = hardware-specific or genuinely out of scope
- Line counts are for C++ source + headers only (`.cpp` + `.h`)

---

## PAYLOAD SYSTEMS

---

### LIBRARY: AP_Camera
**FILES**: 31  
**LINES**: 5,310  
**PURPOSE**: Multi-backend camera control system. Manages shutter trigger (servo, relay, MAVLink, RunCam), video record, zoom, focus, tracking (point/rectangle), camera source switching (RGB/IR/NDVI), geo-tagging via AHRS+GPS, and distance/time interval triggering. Up to 2 simultaneous instances. Backend types: SERVO, RELAY, SOLOGIMBAL, MOUNT, MAVLINK (legacy), MAVLINK_CAMv2, SCRIPTING, RUNCAM.  
**MERIDIAN HAS**: YES — 5 files, 1,112 lines. Backend trait, trigger logic, geotag module, MAVLink backend, Servo backend.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Core payload for mapping, inspection, surveillance. Meridian has the skeleton but is missing several backends and the full feature surface.  
**GAPS**:
- RunCam serial protocol backend (`AP_RunCam`) entirely absent
- Solo gimbal camera backend absent
- Scripting camera backend absent (depends on Lua runtime)
- Multi-instance support (2 cameras) — Meridian appears single-instance
- Distance triggering (`trigg_dist`) — needs ground speed integration from nav
- Time-interval burst capture with `stop_capture()` path
- Camera source switching (IR/NDVI/RGB-wideangle lens types)
- Zoom rate vs. zoom percent dual modes
- Point/rectangle tracking API on camera (vs mount)
- `cam_mode_toggle()` image↔video mode switch
- MAVLink Camera Protocol v2 (`CAMERA_INFORMATION`, `CAMERA_SETTINGS`, `VIDEO_STREAM_INFORMATION` messages)

---

### LIBRARY: AP_Mount
**FILES**: 42  
**LINES**: 14,760  
**PURPOSE**: Gimbal/mount control framework. Target modes: RC passthrough, GPS targeting, ROI, sysid tracking. Backends: Servo, MAVLink (Gremsy/Siyi-old), Siyi ZH-series, Topotek, Viewpro, Xacti, XFRobot, CADDX, SToRM32 (MAVLink + serial), SoloGimbal (dedicated EKF). Rate limiting, follow mode, yaw lock/free. Full MAVLink `MOUNT_CONTROL`, `GIMBAL_DEVICE_*`, `GIMBAL_MANAGER_*` message sets.  
**MERIDIAN HAS**: YES — 7 files, 1,088 lines. Backend trait, manager, MAVLink, Servo, Siyi backends.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Essential for any camera-carrying platform. Meridian has solid core but is missing 9 of 14 backends.  
**GAPS**:
- Topotek backend (widely used in commercial gimbals)
- Viewpro backend
- Xacti backend (Sony Xacti cameras)
- XFRobot backend
- CADDX backend
- SToRM32 MAVLink backend
- SToRM32 serial backend
- SoloGimbal (3DR Solo) + dedicated `SoloGimbalEKF`
- Backend serial base class (`AP_Mount_Backend_Serial`)
- `GIMBAL_DEVICE_ATTITUDE_STATUS` inbound parsing for 3rd-party stabilized gimbals
- `GIMBAL_MANAGER_INFORMATION` / `GIMBAL_MANAGER_STATUS` outbound broadcasting
- Multi-instance mounting (ArduPilot supports 2 simultaneous gimbals)
- Yaw lock mode (body-frame vs earth-frame yaw tracking)
- Follow-mount target from GPS coordinates via mission items

---

### LIBRARY: AP_Gripper
**FILES**: 9  
**LINES**: 773  
**PURPOSE**: Cargo gripper with two backends: Servo (PWM position) and EPM (electropermanent magnet, pulsed relay). State machine: Relaxed → Grabbing → Grabbed → Releasing. MAVLink `COMMAND_LONG` with `MAV_CMD_DO_GRIPPER` (grab=0, release=1). Configurable grab/release PWM and timeout.  
**MERIDIAN HAS**: STUB — 1 file, 189 lines. Types and state machine defined, no PWM output or MAVLink command handling.  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for delivery, search-and-rescue, tethered payload use cases.  
**GAPS**:
- EPM (electropermanent magnet) backend — pulse timing logic absent
- MAVLink `MAV_CMD_DO_GRIPPER` command handler
- Servo PWM output integration with HAL SRV_Channel
- Grab timeout watchdog (auto-release if grab fails)
- Grabbed detection via current sensing or position feedback

---

### LIBRARY: AP_Winch
**FILES**: 9  
**LINES**: 1,164  
**PURPOSE**: Tether/winch control for payload lowering. Three modes: Relaxed, Position (absolute line length), Rate (m/s deploy/retract), RateFromRC. Two backends: PWM (simple speed control) and Daiwa (proprietary RS-485 protocol with feedback). Line length odometry, tension monitoring, MAVLink `WINCH_STATUS` telemetry.  
**MERIDIAN HAS**: STUB — 1 file, 169 lines. Types and mode enum defined, no backend dispatch or line odometry.  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for search-and-rescue lowering, package delivery with cable, firefighting.  
**GAPS**:
- PWM backend control loop
- Daiwa RS-485 backend (proprietary protocol decoder)
- Line length odometry (integrate rate to track deployed length)
- Position mode PID controller
- RC passthrough rate mode
- `WINCH_STATUS` MAVLink telemetry message
- Tension fault detection
- MAVLink `MAV_CMD_DO_WINCH` command handler

---

### LIBRARY: AP_Parachute
**FILES**: 5  
**LINES**: 456  
**PURPOSE**: Emergency parachute deployment. Sequence: (1) disarm motors, (2) wait configurable delay (~500ms for props to clear), (3) pulse servo or relay for 2000ms to fire chute, (4) servo returns to neutral. Triggered by crash detector, GCS command, or RC channel. Pre-arm check: chute enabled + channel configured. Altitude gate (only deploy above min safe altitude).  
**MERIDIAN HAS**: STUB — 1 file, 269 lines. State machine and types defined. Logic looks mostly complete for basic sequence.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Safety-critical. Required for any platform carrying the system commercially or militarily.  
**GAPS**:
- Relay output backend (currently servo only)
- Integration with crash detector (auto-trigger on inverted + low altitude)
- GCS `MAV_CMD_DO_PARACHUTE` command handler
- RC channel trigger path
- Altitude gate check (do not deploy below safe altitude)
- Pre-arm check integration (parachute configured before arm)
- Test mode (partial release for ground verification)

---

### LIBRARY: AP_LandingGear
**FILES**: 3  
**LINES**: 486  
**PURPOSE**: Retractable landing gear controller. States: Deployed/Deploying/Retracted/Retracting. Servo output via SRV_Channel. Auto-deploy below configurable altitude, auto-retract above altitude. RC channel trigger. MAVLink `LANDING_GEAR` message. Pre-arm: requires gear deployed before arm.  
**MERIDIAN HAS**: STUB — 1 file, 192 lines. Types and basic state machine. No servo output or altitude triggers.  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for fixed-gear platforms that convert to retractable for efficiency; common on larger platforms.  
**GAPS**:
- Servo PWM output via HAL SRV_Channel
- Altitude-based auto deploy/retract using AHRS terrain-adjusted altitude
- RC channel trigger integration
- `LANDING_GEAR` MAVLink status message
- Pre-arm check (gear down before arm)
- Transition completion detection (time-based or position-feedback)

---

### LIBRARY: AC_PrecLand
**FILES**: 17  
**LINES**: 2,147  
**PURPOSE**: Precision landing with visual or IR beacon target tracking. Backends: IRLock (IR LED grid), MAVLink (LANDING_TARGET messages from companion computer), SITL, Gazebo SITL. State machine manages lost/found/re-acquired target. Internal EKF (PosVelEKF) fuses target angle + IMU to estimate relative position. Inertial history buffer extrapolates target position across frame delays.  
**MERIDIAN HAS**: YES — 6 files, 917 lines. Backend trait, Kalman filter, inertial history, main state machine.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Required for autonomous package delivery, dock landing, carrier landing.  
**GAPS**:
- IRLock hardware backend (I2C frame parsing, `LANDING_TARGET_RAW_SENSOR` decode)
- MAVLink `LANDING_TARGET` inbound backend
- SITL backend for simulation testing
- State machine's "strict" mode (abort landing if target lost near ground)
- Failsafe: fall back to GPS position when target lost above threshold altitude
- Integration with landing mode descent rate controller

---

### LIBRARY: AC_Sprayer
**FILES**: 3  
**LINES**: 304  
**PURPOSE**: Agricultural sprayer control. Pump servo + spinner servo. Ground-speed-proportional pump rate. Enable/disable based on speed threshold. MAVLink `MAV_CMD_DO_SPRAYER`. Dry-run test mode.  
**MERIDIAN HAS**: STUB — 1 file, 148 lines. Struct defined with fields. No update loop or speed-proportional control.  
**PRIORITY**: LOW  
**JUSTIFICATION**: Agriculture-only. Not needed for general autopilot. Include if agricultural missions are a target market.  
**GAPS**:
- Ground-speed-proportional pump PWM calculation
- Speed threshold enable/disable hysteresis
- Spinner enable/disable independent of pump
- MAVLink `MAV_CMD_DO_SPRAYER` command handler
- Test mode (run pump on ground for calibration)
- Mission item `MISSION_CHANGE_SPEED` integration for spray rate adjustment

---

## AWARENESS / REGULATION

---

### LIBRARY: AP_OSD
**FILES**: 68  
**LINES**: 7,599  
**PURPOSE**: On-screen display for FPV/analog video. Backends: MAX7456 (SPI analog OSD chip), MSP (DJI O3/Walksnail digital), MSP DisplayPort, SITL preview. ~50 configurable screen elements: altitude, speed, heading, battery, RSSI, GPS, compass rose, horizon, flight mode, warnings. Two full screens switchable via RC. Parameter-edit screen via joystick. Custom fonts per backend.  
**MERIDIAN HAS**: YES — 1 file, 1,841 lines. MAX7456 backend, MSP backend, element rendering, screen layout.  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Consumer and FPV market expectation. Useful for all manned-adjacent systems with video downlink.  
**GAPS**:
- MSP DisplayPort backend for Walksnail Avatar (separate from MSP)
- Parameter-edit screen (OSD-based parameter tuning via joystick)
- Custom font upload to MAX7456 flash
- Second screen support (screen A/B switchable via RC)
- SITL OSD preview backend
- Canvas mode (arbitrary pixel drawing for advanced HUDs)
- All ~50 element types — Meridian likely implements ~15-20

---

### LIBRARY: AP_Notify
**FILES**: 75  
**LINES**: 8,569  
**PURPOSE**: LED and buzzer notification framework. Priority-based pattern selection. 30+ hardware backends: NeoPixel, NCP5623 (I2C RGB), ToshibaLED, LP5562, IS31FL3195, PCA9685, BoardLED, DShot LED, DShotLED, ProfiLED, OLED display (SH1106/SSD1306), DroneCAN RGB, GPIO LED, NeoPixel, MML tone alarm, SITL SFML preview. Status flags: armed, GPS lock, failsafe, low battery, etc.  
**MERIDIAN HAS**: YES — 1 file, 1,231 lines. LED pattern engine, NeoPixel/WS2812 backend, buzzer, priority system.  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for user feedback, regulatory (FAA anti-collision lighting), and debugging.  
**GAPS**:
- NCP5623 I2C RGB LED driver
- ToshibaLED I2C driver
- LP5562 I2C driver
- IS31FL3195 I2C driver
- PCA9685 I2C LED driver
- DShot LED integration (LED strip via ESC DShot signal)
- ProfiLED (addressable LED strips via SPI)
- OreoLED I2C (3DR Oreo LEDs)
- OLED display backends (SH1106, SSD1306 for status display)
- MML tone player (Musical Markup Language for richer tones)
- DroneCAN RGB LED backend
- Navigator LED (Blue Robotics Navigator HAT)
- SITL SFML visual LED preview

---

### LIBRARY: AP_ADSB
**FILES**: 54  
**LINES**: 6,405  
**PURPOSE**: ADS-B traffic surveillance. Backends: uAvionix MAVLink (serial bridge), uAvionix UCP (direct serial), Sagetech MXS (transponder with squawk), Sagetech (legacy). Tracks up to ~50 aircraft. Computes TTCA threat levels, publishes `ADSB_VEHICLE` MAVLink messages. Supports both In/Out (transponder) and In-only modes. GDL-90 protocol support for tablet display. Integrates with Avoidance system.  
**MERIDIAN HAS**: STUB — 1 file, 543 lines. Types, threat levels, vehicle tracking table defined. No backend drivers.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: FAA required for operations in controlled airspace. Also feeds collision avoidance. Regulatory pressure increasing.  
**GAPS**:
- uAvionix MAVLink backend (MAVLINK_ADSB_VEHICLE message parsing)
- uAvionix UCP serial backend
- Sagetech MXS backend (transponder control + squawk)
- Sagetech legacy backend
- GDL-90 protocol encoder for tablet connectivity
- `ADSB_VEHICLE` outbound MAVLink message broadcast
- TTCA threat level computation (currently stub types, no math)
- Integration hook to AP_Avoidance for actual maneuver commands
- Transponder health monitoring (power, ICAO config)
- Aircraft table expiry / stale entry pruning

---

### LIBRARY: AP_OpenDroneID
**FILES**: 4  
**LINES**: 1,266  
**PURPOSE**: FAA/EU Remote ID compliance (ASTM F3411-22a). Five message types broadcast: Basic ID, Location/Vector, Authentication, Self-ID, System. Bluetooth 5 / Wi-Fi NaN broadcast via MAVLink passthrough to companion module. Pre-arm gate: arm blocked until `arm_status` OK received from transponder. Operator ID, UAS ID, session ID tracking.  
**MERIDIAN HAS**: STUB — 1 file, 363 lines. Message type enums, ID types, UA classification defined. No broadcast logic.  
**PRIORITY**: CRITICAL  
**JUSTIFICATION**: FAA mandate effective Sep 2023. Any commercial or public operation in US airspace requires Remote ID. Blocking for US market.  
**GAPS**:
- Message encoding (Basic ID, Location, Authentication, Self-ID, System)
- Broadcast timing (Location at 1Hz minimum, others at required intervals)
- MAVLink `OPEN_DRONE_ID_*` message set (inbound from GCS + outbound to module)
- Pre-arm gate: block arm until `OPEN_DRONE_ID_ARM_STATUS` received OK
- Operator location update path (dynamic vs fixed operator location)
- Session ID management
- Bluetooth 5 transport integration (via companion computer MAVLink bridge)
- Wi-Fi NaN transport integration
- Authentication message generation (timestamp + nonce)

---

### LIBRARY: AP_VideoTX
**FILES**: 7  
**LINES**: 2,429  
**PURPOSE**: Video transmitter control for FPV systems. Two protocols: SmartAudio (TBS CRSF sideband, 1-wire) and Tramp (ImmersionRC, UART). Configures VTX power, frequency/channel, pit mode. RC/MAVLink channel control. OSD integration (current VTX state displayed).  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: FPV market only. Not relevant for autonomous beyond-visual-line operations. Add if FPV racing or FPV platform support is planned.  
**GAPS** (entire feature missing):
- SmartAudio protocol (TBS 1-wire VTX control)
- Tramp protocol (ImmersionRC UART VTX control)
- Power level management (25mW / 200mW / 600mW / 1W)
- Channel/frequency band selection (A/B/E/F/R/L bands)
- Pit mode (low power transmit for pits/proximity)
- RC channel control of power/frequency
- MAVLink `VIDEO_STREAM_INFORMATION` integration

---

### LIBRARY: AP_AIS
**FILES**: 4  
**LINES**: 1,369  
**PURPOSE**: Automatic Identification System for maritime vessel tracking. Decodes NMEA AIS sentences (types 1/2/3/18/21) from serial input. Tracks surface vessels — position, heading, speed, MMSI, name. Publishes `AIS_VESSEL` MAVLink messages. Stale vessel expiry. Used on USV platforms to avoid collision with ships.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: Maritime (USV) use case only. Skip unless adding marine vehicle support.  
**GAPS** (entire feature missing):
- NMEA sentence parser (VDM/VDO AIS sentence types)
- Message type 1/2/3 decoder (Class A position report)
- Message type 18 decoder (Class B position report)
- Message type 21 decoder (Aid-to-Navigation)
- Vessel tracking table with expiry
- `AIS_VESSEL` MAVLink broadcast
- Integration with Avoidance for USV maneuver commands

---

### LIBRARY: AP_Radio
**FILES**: 16  
**LINES**: 7,677  
**PURPOSE**: Embedded RC receiver chip drivers — not external RC protocols but chips physically on the flight controller. Three chip backends: BK2425 (BK2425 SPI FHSS transceiver — used in FrSky D8/D16 clones), CC2500 (TI CC2500 SPI transceiver — FrSky D8 clone), Cypress CYRF6936 (DSM/Spektrum clone). Supports bidirectional telemetry to TX, firmware upload to radio, factory test modes.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: SKIP  
**JUSTIFICATION**: This is for specific flight controller boards (Parrot Bebop / some FrSky-clone boards) with embedded radio chips soldered on. Modern flight controllers use external RC receivers (SBUS, ELRS, CRSF). Not relevant for Meridian's target hardware. Skip entirely.  
**GAPS**: N/A — intentionally out of scope.

---

## INFRASTRUCTURE / CORE

---

### LIBRARY: AP_Param
**FILES**: 7  
**LINES**: 4,668  
**PURPOSE**: Runtime parameter system. Typed parameters (Float, Int8, Int16, Int32) with names up to 16 chars. Static `PROGMEM` parameter table with min/max/default. Persistent storage via StorageManager (flash wear-leveling). MAVLink PARAM_VALUE/PARAM_SET/PARAM_REQUEST protocol. Group support (nested structs). Save/load/reset-to-defaults. Atomic update on param change (notify callbacks).  
**MERIDIAN HAS**: STUB — 1 file, 895 lines. Types, ParamEntry, storage struct defined. No actual persistence backend or MAVLink protocol.  
**PRIORITY**: CRITICAL  
**JUSTIFICATION**: Every subsystem depends on parameters. Without persistence, tuned values are lost on reboot. Blocking for production use.  
**GAPS**:
- Flash persistence backend (write to StorageManager / FlashStorage)
- MAVLink `PARAM_VALUE` enumerate all parameters response
- MAVLink `PARAM_REQUEST_READ` single parameter response
- MAVLink `PARAM_SET` with validation and callback notification
- MAVLink `PARAM_REQUEST_LIST` (full enumeration)
- Group nesting — parameters within sub-structs (ArduPilot's `AP_GROUPINFO` macro system)
- Reset-to-defaults command
- Parameter format version checking (detect incompatible flash layouts)
- TOML/JSON file backend for Linux/SITL
- Save-on-change flag (dirty tracking for deferred writes)

---

### LIBRARY: AP_Logger
**FILES**: 27  
**LINES**: 11,080  
**PURPOSE**: Structured binary flight logging. Backends: File (SD card), Flash JEDEC (onboard NOR flash), W25NXX (Winbond QSPI flash), MAVLink log transfer. Self-describing format with FMT messages defining schema. Log replay compatibility. Message writers for startup, parameter snapshot, mode changes. Circular buffer with configurable max size. Log download via GCS. Named log files with incrementing index.  
**MERIDIAN HAS**: STUB — 1 file, 919 lines. Types, schema, header defined. No backend write paths.  
**PRIORITY**: CRITICAL  
**JUSTIFICATION**: Flight logs are essential for debugging, certification, incident investigation, and regulatory compliance. Blocking for production use.  
**GAPS**:
- File backend (posix `write()` for Linux/SITL)
- JEDEC flash backend (block erase + page program for onboard NOR flash)
- W25NXX backend (Winbond QSPI flash — most common on current FCs)
- MAVLink log transfer backend (`LOG_DATA`, `LOG_ENTRY` message set)
- Circular log rotation (overwrite oldest when flash full)
- Parameter snapshot on arm (log all current params)
- FMT message format header generation
- Message writer queue (buffered async writes)
- Log index/naming (auto-increment log number)
- Log download protocol via GCS
- Arm/disarm event markers in log

---

### LIBRARY: AP_Filesystem
**FILES**: 26  
**LINES**: 6,458  
**PURPOSE**: Virtual filesystem abstraction. Backends: FATFS (SD card), LittleFS over Flash, ROMFS (read-only embedded), Mission FS (virtual files exposing mission data), Param FS (virtual files exposing parameters), POSIX compat (Linux), ESP32 SPIFFS. Unified `open()/read()/write()/stat()/opendir()` API used by Logger, Scripting, Terrain, and firmware update.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Required by Logger (file backend), Scripting (Lua scripts on SD), Terrain (SRTM data files), and firmware update. Without it, those features are blocked from their filesystem-backed modes.  
**GAPS** (entire feature missing):
- Filesystem trait / VFS abstraction layer
- FATFS backend (FatFs C library binding or pure-Rust fatfs crate)
- LittleFS backend (pure-Rust littlefs2 crate or C binding)
- POSIX backend for Linux/SITL (`std::fs` wrapping)
- ROMFS backend (compressed read-only embedded data)
- Mission virtual file backend
- Param virtual file backend
- `opendir()/readdir()/closedir()` directory listing
- Path routing (select backend based on path prefix)
- File transfer via MAVLink FTP (`FILE_TRANSFER_PROTOCOL` messages)

---

### LIBRARY: AP_Scheduler
**FILES**: 7  
**LINES**: 1,340  
**PURPOSE**: Cooperative task scheduler with timing budget. Each task has a rate_hz and maximum runtime_us. Runs tasks in a fixed loop — skips tasks that exceed their time budget. Tracks overruns and slow tasks via PerfInfo. Integrates with AP_Logger (logs task timing). Two scheduling modes: normal (variable time slice) and fast-loop (400Hz IMU-sync loop). Used by every vehicle to structure the main loop.  
**MERIDIAN HAS**: NO — crate does not exist (Meridian uses Rust async/RTIC instead)  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Meridian's RTIC/async approach is architecturally superior to ArduPilot's cooperative scheduler. A direct port is not needed. However, Meridian needs equivalent task budget monitoring and overrun detection for production use.  
**GAPS**:
- Task timing budget enforcement (detect tasks running over budget)
- Per-task performance counters (min/max/mean runtime)
- Overrun logging (log when tasks exceed budget)
- Scheduler statistics in flight log
- Note: RTIC interrupt-driven model is preferred — this is a MONITORING gap, not an architectural one

---

### LIBRARY: AP_Scripting
**FILES**: 443  
**LINES**: 8,996 (C++ only; Lua scripts in subdirectories add ~10k more lines)  
**PURPOSE**: Embedded Lua 5.4 runtime for user scripts. Scripts run in sandboxed coroutines at configurable Hz. Binding layer exposes ~400+ ArduPilot APIs to Lua: AHRS, GPS, motors, servos, MAVLink, serial, CAN, parameters. Built-in applets for terrain following, auto-landing, payload ops, custom modes. ROMFS-embedded scripts for common tasks. Script upload via SD card or MAVLink file transfer.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Major differentiator for commercial customers. Enables mission customization without recompiling. Significant engineering effort. Defer until core systems are complete.  
**GAPS** (entire feature missing):
- Lua 5.4 interpreter (Rust bindings to lua-sys or pure-Rust lua crate)
- Coroutine scheduler (each script runs as a coroutine at its hz)
- Sandboxed execution environment (memory limits, CPU limits)
- Binding layer generator (ArduPilot uses Python scripts to auto-generate ~400 bindings)
- Script upload (SD card path + MAVLink file transfer)
- ROMFS script embedding (compile-time inclusion)
- Error handling + watchdog (kill runaway scripts)
- Serial access from Lua
- CAN sensor access from Lua
- Parameter access from Lua
- MAVLink send/receive from Lua

---

## ADVANCED FEATURES

---

### LIBRARY: AC_AutoTune
**FILES**: 9  
**LINES**: 5,272  
**PURPOSE**: Automatic PID tuner for multirotors and helicopters. Multirotor algorithm: step-input twitch test on each axis (roll, pitch, yaw), measures peak response and bounce-back, iterates Rate-D, Rate-P, Angle-P gains over 5 steps per axis. Helicopter variant: frequency-response sweep with separate AC_AutoTune_FreqResp analyzer. Results saved to parameters on completion. Triggered via RC channel or GCS.  
**MERIDIAN HAS**: YES — 3 files, 769 lines. Multirotor tuner state machine, axis/step enums, TuneResults struct.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Essential for field commissioning — manual PID tuning is time-consuming and error-prone. Customers expect it.  
**GAPS**:
- Helicopter variant (`AC_AutoTune_Heli`) with frequency-response algorithm
- `AC_AutoTune_FreqResp` frequency response analyzer (FFT-based sweep)
- RC channel trigger to enter/exit tune mode
- Parameter save on completion (write results back to param system)
- GCS status reporting during tune (current axis, current gain values)
- Abort logic (excessive attitude error triggers abort)
- SITL simulation validation path
- Tune from hover requirement (validate hover stability before starting)

---

### LIBRARY: AP_GyroFFT
**FILES**: 17  
**LINES**: 9,868  
**PURPOSE**: Real-time gyro noise FFT for dynamic notch filter frequency tracking. Runs CMSIS-DSP FFT on gyro samples. Identifies 3 dominant frequency peaks (motor noise harmonics). Feeds peak frequencies to InertialSensor's harmonic notch filter in real time. Runs in ISR context on fast hardware or in the scheduler on slower hardware. Supports 32–512 sample windows, configurable averaging.  
**MERIDIAN HAS**: YES — 4 files, 744 lines. FFT engine, GyroFFT processor, 3-peak tracker.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Modern FC hardware expectation. Without dynamic notch filtering, vibration noise corrupts attitude estimates. Required for high-RPM platforms (racing, larger prop systems).  
**GAPS**:
- CMSIS-DSP backend (for Cortex-M targets with hardware DSP instructions)
- Multiple-axis simultaneous FFT (ArduPilot runs FFT on all 3 gyro axes)
- Integration with InertialSensor harmonic notch filter (Meridian-fft produces peaks but notch filter integration is unclear)
- Real-time frequency tracking with Hanning window
- Dynamic notch center frequency update path
- Throttle-based frequency estimation (fallback when FFT window not yet full)

---

### LIBRARY: AP_Soaring
**FILES**: 9  
**LINES**: 1,155  
**PURPOSE**: Thermal soaring for fixed-wing aircraft. Extended Kalman filter estimates thermal center and strength from variometer readings. Speed-to-fly calculator (MacCready theory) for cruise between thermals. State machine: Cruising → Thermal Detected → Circling → Thermal Lost. Integrates with L1 navigation controller for thermal circle. Variometer from barometer or dedicated sensor.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: Fixed-wing soaring niche. Only relevant if adding fixed-wing thermal soaring as a feature. Skip for now.  
**GAPS** (entire feature missing):
- Extended Kalman filter for thermal estimation
- Variometer (vertical speed from barometer delta)
- MacCready speed-to-fly calculator
- Thermal circle navigation state machine
- L1 controller integration for thermal centering
- Soaring flight mode

---

### LIBRARY: AP_Terrain
**FILES**: 8  
**LINES**: 2,225  
**PURPOSE**: SRTM terrain elevation database. 32x28 grid blocks cached in 12-block LRU. Bilinear interpolation for smooth elevation. Grid data fetched from GCS via `TERRAIN_REQUEST` / `TERRAIN_DATA` MAVLink message pair. Height above terrain (HAT) computation for terrain following and safe altitude checks. Grid files cached on SD card for offline use.  
**MERIDIAN HAS**: YES — 1 file, 784 lines. Grid block structure, LRU cache, bilinear interpolation, MAVLink request/response types.  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Required for terrain-following flight modes, safe RTL altitude, and survey missions.  
**GAPS**:
- GCS terrain request/response state machine (when to request, when to use pending data)
- SD card cache (persist received grid blocks to filesystem)
- Height-above-terrain continuous update loop (poll terrain height at nav frequency)
- Terrain pre-fetch (request terrain along planned route ahead of time)
- Integration with mission planner for terrain-clearance validation
- Terrain status reporting in MAVLink (`TERRAIN_REPORT` message)
- Multiple grid spacing support (3m / 30m / 100m resolution)

---

### LIBRARY: AC_Autorotation
**FILES**: 4  
**LINES**: 833  
**PURPOSE**: Helicopter autorotation controller for engine-out emergency. Two phases: Entry (collective drop, head speed management) and Glide (optimum descent rate for altitude gain). RSC autorotation mode. Integrates with `AP_MotorsHeli` rotor speed controller. Triggered by engine failure detection or manual RC switch.  
**MERIDIAN HAS**: NO (covered by meridian-heli which has RSC governor but not the autorotation state machine)  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for helicopter safety — engine failure without autorotation = crash. Critical for any helicopter product.  
**GAPS**:
- Entry phase controller (collective pitch schedule for head speed preservation)
- Glide phase controller (optimize descent for flare height)
- Flare phase (collective pull for touchdown deceleration)
- Engine failure detection trigger
- RSC autorotation mode integration
- RC manual override during autorotation

---

### LIBRARY: AP_ICEngine
**FILES**: 5  
**LINES**: 1,123  
**PURPOSE**: Internal combustion engine management. Ignition on/off via relay/servo. Starter motor control (PWM or relay). Throttle via servo. State machine: Off → Starting → Running → Stopping. Safety: no start without GPS or above minimum altitude. Stall detection and restart logic. TCA9554 I2C GPIO expander backend for engine control signals.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for gas-powered platforms (long-endurance fixed-wing, hybrid VTOL). Growing market segment.  
**GAPS** (entire feature missing):
- Ignition relay control
- Starter motor PWM/relay control
- Throttle servo control
- Start/stop state machine with safety gates
- Stall detection (RPM below threshold)
- Auto-restart after stall
- TCA9554 I2C GPIO expander backend
- EFI integration handoff (ICEngine provides scaffolding; EFI provides telemetry)

---

### LIBRARY: AP_EFI
**FILES**: 26  
**LINES**: 3,050  
**PURPOSE**: Electronic Fuel Injection telemetry and control. Multiple backends: Currawong ECU, Loweheiser ECU, Hirth serial, Lutan serial, MegaSquirt serial, NWPMU, DroneCAN EFI, MAVLink EFI, Scripting EFI. Reads RPM, fuel consumption, cylinder temperature, throttle position. Throttle linearization to compensate non-linear EFI response curves. Logging of engine state.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for gas-powered platforms. Works alongside ICEngine. Each EFI system has a different serial protocol.  
**GAPS** (entire feature missing):
- EFI state struct (RPM, fuel flow, CHT, throttle)
- Backend trait abstraction
- Currawong ECU serial backend
- Loweheiser ECU backend
- Hirth serial backend
- MegaSquirt serial backend
- DroneCAN EFI backend
- MAVLink EFI backend
- Throttle linearization table
- EFI health logging

---

### LIBRARY: AP_Generator
**FILES**: 18  
**LINES**: 4,065  
**PURPOSE**: Generator/fuel cell management for hybrid-electric aircraft. Backends: Cortex (Intelligent Energy), IE 2400 (Intelligent Energy), IE 650/800, IE FuelCell, Loweheiser, RichenPower. Monitors fuel level, power output, temperature. Load-sharing with battery. Fault detection. GCS telemetry via `GENERATOR_STATUS` MAVLink message. Integration with BattMonitor for power accounting.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: Specialized for hybrid-electric / fuel-cell platforms. Niche market. Defer until those platforms are targeted.  
**GAPS** (entire feature missing):
- Generator state struct (fuel level, output watts, temperature, fault code)
- Backend trait
- IE fuel cell backend series (5 variants)
- Loweheiser generator backend
- RichenPower backend
- `GENERATOR_STATUS` MAVLink message
- Load-share logic (battery + generator)

---

### LIBRARY: AP_Torqeedo
**FILES**: 9  
**LINES**: 1,964  
**PURPOSE**: Torqeedo electric marine motor control via proprietary TQBus RS-485 protocol. Throttle control, telemetry (RPM, power, temperature, fault codes). Motor health monitoring. Used on USV/ASV platforms. MAVLink motor status telemetry.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: SKIP  
**JUSTIFICATION**: Marine-only, proprietary protocol. Skip unless adding marine surface vehicle support.  
**GAPS**: N/A — intentionally out of scope.

---

### LIBRARY: AP_WheelEncoder
**FILES**: 10  
**LINES**: 1,304  
**PURPOSE**: Wheel odometry for ground vehicles (rovers). Quadrature encoder decoder (rising/falling edge counting). Two backends: Quadrature (hardware interrupt) and SITL. Wheel rate control PID. Integrates with AHRS/EKF as odometry source for GPS-degraded navigation. Reports wheel speed in m/s per wheel.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: Rover/ground vehicle only. Not needed for multirotor/fixed-wing core. Add when rover vehicle type is implemented.  
**GAPS** (entire feature missing):
- Quadrature encoder edge counting
- Wheel speed calculation (counts/second → m/s via wheel circumference)
- SITL backend
- EKF odometry source integration
- Wheel rate PID for differential steering

---

### LIBRARY: AP_LeakDetector
**FILES**: 8  
**LINES**: 258  
**PURPOSE**: Water/liquid leak detection for underwater/marine vehicles. Two backends: Analog (ADC threshold) and Digital (GPIO pin high/low). Triggers failsafe on leak detection. Pre-arm check: sensor present and healthy. Used in BlueROV2 and other underwater vehicles.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: SKIP  
**JUSTIFICATION**: Submarine/underwater vehicle only. Skip unless adding sub support.  
**GAPS**: N/A — intentionally out of scope.

---

### LIBRARY: AP_BLHeli
**FILES**: 3  
**LINES**: 2,165  
**PURPOSE**: BLHeli ESC configuration passthrough. Allows GCS to communicate directly with BLHeli32 ESCs via the flight controller UART. Acts as a transparent serial bridge. Also reads ESC telemetry during passthrough. Required for BLHeli Suite / BLHeli Configurator ESC setup. Not a runtime control path — only used for ESC firmware configuration and tuning.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: ESC configuration convenience feature. Not flight-critical. Users can configure ESCs directly. Add as a quality-of-life feature post-core.  
**GAPS** (entire feature missing):
- UART passthrough bridge
- BLHeli 4-way interface protocol
- ESC address/type detection
- Serial bridge to GCS MAVLink passthrough

---

### LIBRARY: AP_ESC_Telem
**FILES**: 8  
**LINES**: 1,420  
**PURPOSE**: ESC telemetry aggregator. Receives RPM, current, voltage, temperature from individual ESC backends (BLHeli32, Kontronik, APD, Hobbywing, etc.). Backends: generic BLHeli UART telem, SITL. Exposes per-ESC data to logging, OSD, and GCS. Integrates with AP_BattMonitor for per-motor current. Triggers motor failure detection.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Essential for motor health monitoring, failure detection, and efficient operation. Modern ESCs all support telemetry. Required for any production system.  
**GAPS** (entire feature missing):
- ESC telemetry backend trait
- BLHeli UART telemetry decoder (RPM, current, voltage, temp per ESC)
- SITL telemetry backend
- Per-ESC data aggregation table
- Motor failure detection (RPM drops to zero on armed vehicle)
- MAVLink `ESC_TELEMETRY_1_TO_4` / `ESC_TELEMETRY_5_TO_8` message broadcast
- OSD integration (per-motor RPM display)
- Logger integration (ESC_DATA log message)

---

## CALIBRATION

---

### LIBRARY: AP_AccelCal
**FILES**: 4  
**LINES**: 1,132  
**PURPOSE**: Multi-point accelerometer calibration. 6-point tumble procedure: hold each of 6 orientations (level, nose-up/down, right/left side, inverted), collect samples, fit ellipsoid. `AccelCalibrator` per IMU instance. GCS-guided procedure via `COMMAND_LONG MAV_CMD_ACCELCAL_VEHICLE_POS`. Results in scale factor and offset corrections. Saves to INS parameters.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Required for any flight controller. Without proper accel calibration, attitude estimation is biased. Every platform requires this on first setup.  
**GAPS** (entire feature missing):
- 6-position ellipsoid fitting algorithm
- Per-axis sample collection with variance checking
- GCS-guided procedure state machine
- `MAV_CMD_ACCELCAL_VEHICLE_POS` command handler
- Scale/offset result application to INS parameters
- Quality checks (sufficient samples, low variance)
- Multi-IMU support (calibrate each IMU independently)

---

### LIBRARY: AP_TempCalibration
**FILES**: 3  
**LINES**: 337  
**PURPOSE**: Temperature-compensated IMU calibration. Collects IMU bias samples across a temperature range (startup cold soak → operational temperature). Fits polynomial correction curves. Applied in real time via InertialSensor's TC (temperature compensation) tables. Reduces thermal drift that causes attitude drift during warm-up.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Important for precision applications and cold-weather operations. Reduces gyro drift during thermal stabilization. Most production FCs implement this.  
**GAPS** (entire feature missing):
- Temperature sample collection during warmup
- Polynomial curve fitting (3rd order for bias vs. temperature)
- Correction table storage in parameters
- Real-time bias correction application
- Calibration procedure trigger (MAVLink command or auto-detect)

---

### LIBRARY: AP_Declination
**FILES**: 10  
**LINES**: 506  
**PURPOSE**: World Magnetic Model — converts GPS position to magnetic declination (degrees). Lookup table of declination values on a compressed 1-degree grid covering the globe. Used by compass calibration and compass heading to compute true heading from magnetic heading. Updated with WMM epoch (currently WMM2020).  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Without correct magnetic declination, compass-based heading is wrong everywhere except the prime meridian. Required for accurate navigation globally.  
**GAPS** (entire feature missing):
- WMM declination lookup table (compressed grid)
- Bilinear interpolation for sub-degree accuracy
- API: `get_declination(lat, lon) -> f32` in degrees
- WMM2025 epoch update (WMM2020 expires 2025)

---

### LIBRARY: AP_Quicktune
**FILES**: 2  
**LINES**: 856  
**PURPOSE**: Simplified in-flight PID tuner (Lua-applet-style, now native C++). Less aggressive than AutoTune — adjusts one gain at a time by small increments based on oscillation detection. Triggered via RC switch. Runs in hover/cruise. Updates Rate-P, Rate-D, Rate-I incrementally. More conservative than AutoTune; suitable for larger/heavier vehicles where AutoTune's aggressive step input is unsafe.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Complement to AutoTune for vehicles too large/dangerous for full AutoTune procedure. Useful for production fleet commissioning.  
**GAPS** (entire feature missing):
- Oscillation detection algorithm (peak-to-peak tracking on rate error)
- Incremental gain adjust logic
- RC switch trigger/abort
- Per-axis gain state tracking
- Integration with param system (save incremented values)
- GCS status reporting during quicktune

---

## HARDWARE ABSTRACTION / BOARD SUPPORT

---

### LIBRARY: AP_IOMCU
**FILES**: 12  
**LINES**: 5,007  
**PURPOSE**: IO coprocessor communication layer. Used on Pixhawk-family boards where a separate STM32F103 IO processor handles RC input and servo output. Implements the IOMCU UART protocol: servo output commands, RC input reads, safety switch, voltage/current passthrough. Firmware upload to IOMCU. Handles IOMCU failsafe (servos freeze if main CPU stops sending).  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required specifically for Pixhawk FMUv2/v3/v5/v6 hardware which has an IO coprocessor. Without it, Meridian cannot run on the most widely deployed autopilot hardware in the world.  
**GAPS** (entire feature missing):
- IOMCU UART protocol (command/response framing)
- Servo output command path to IOMCU
- RC input read path from IOMCU
- Safety switch monitoring
- Firmware upload to IOMCU
- IOMCU health monitoring (heartbeat timeout → failsafe)
- Voltage/current ADC passthrough from IOMCU

---

### LIBRARY: AP_BoardConfig
**FILES**: 5  
**LINES**: 1,610  
**PURPOSE**: Board-specific hardware configuration. Applies board-level parameters: IMU SPI bus selection, external SPI chip enables, IMU heater target temperature, DroneCAN enable, safety switch behavior. IMU heater PID controller (maintains constant IMU temperature for calibration stability). Board driver setup calls. Board type detection from flash.  
**MERIDIAN HAS**: STUB — meridian-boardcfg exists but is minimal  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Without board configuration, hardware features are hardcoded. Required for multi-board support.  
**GAPS**:
- IMU heater PID controller
- Bus/chip-enable configuration parameters
- DroneCAN bus enable/disable
- Safety switch behavior configuration
- Board type detection from compiled-in metadata
- External sensors enable/disable via params

---

### LIBRARY: AP_FlashStorage
**FILES**: 4  
**LINES**: 1,131  
**PURPOSE**: Wear-leveling flash parameter storage. Stores key-value pairs in NOR flash with A/B sector rotation. Handles flash block erase cycles. Used by StorageManager as the physical backend. Single-copy format with CRC protection. Handles partial writes on power loss.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Required for parameter persistence on embedded hardware. Without it, all tuning is lost on reboot. Works alongside StorageManager.  
**GAPS** (entire feature missing):
- A/B sector rotation wear leveling
- Key-value record format with CRC
- Flash block erase management
- Power-loss safe write (verify before committing)
- Sector compaction (reclaim space from deleted entries)
- Flash driver trait (erase page, write page, read page)

---

### LIBRARY: StorageManager
**FILES**: 4  
**LINES**: 696  
**PURPOSE**: Logical address space manager over flash. Divides flash into named regions: PARAM, FENCE, RALLY, MISSION, TERRAIN. Each region has a fixed logical address range and size. Translates logical addresses to physical flash locations. Used by AP_Param, AP_Mission, AP_Fence, AP_Rally, AP_Terrain. Supports RAMTRON FRAM as an alternative backend.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Required glue layer between parameter/mission/fence storage and physical flash. Without it, nothing persists correctly on embedded hardware.  
**GAPS** (entire feature missing):
- Logical address space definition (region enum + size table)
- Address translation (logical → physical)
- PARAM region backing
- MISSION region backing
- FENCE region backing
- RALLY region backing
- TERRAIN cache region backing
- FRAM backend (SPI RAMTRON — zero erase cycle, byte-addressable)

---

### LIBRARY: AP_DAL
**FILES**: 21  
**LINES**: 2,853  
**PURPOSE**: Data Abstraction Layer — wraps sensor data for EKF3. Provides a frozen, consistent snapshot of sensor data for each EKF update step. Logging replay: DAL records all sensor inputs to log; log replay re-feeds identical data to EKF for debugging. Abstracts: AHRS, Baro, Compass, GPS, InertialSensor, Airspeed, Beacon, RangeFinder, VisualOdom. Without DAL, EKF cannot be log-replayed.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Critical for EKF debugging and certification (log replay). However, Meridian's Rust EKF may handle this differently. Consider a replay trait rather than a direct port.  
**GAPS** (entire feature missing):
- Sensor snapshot types for each sensor (Baro, GPS, IMU, Compass, etc.)
- Snapshot capture at EKF update rate
- Log-replay mode (replay sensor snapshots from log file)
- Per-sensor DAL wrapper (freeze sensor state at EKF tick)
- Logging of all DAL inputs (for post-flight replay)

---

### LIBRARY: AP_CheckFirmware
**FILES**: 6  
**LINES**: 4,291  
**PURPOSE**: Firmware integrity verification. Uses Monocypher Ed25519 signatures to verify firmware authenticity. Public key embedded in firmware. `secure_command` support for cryptographically signed GCS commands. Flash signature check on boot. Hardware-specific key management. Protects against unauthorized firmware on certified/restricted platforms.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: Required only for platforms with firmware signing requirements (military, certified aviation). Most open-source deployments skip this. Low priority for development; HIGH priority for any DoD/certified product.  
**GAPS** (entire feature missing):
- Ed25519 signature verification (use `ed25519-dalek` crate)
- Public key embedding in firmware binary
- Boot-time flash signature check
- Signed command verification
- Key management infrastructure

---

### LIBRARY: AP_ROMFS
**FILES**: 4  
**LINES**: 938  
**PURPOSE**: Read-only in-memory filesystem. Stores compressed files (Lua scripts, default parameters, OSD fonts, terrain files) directly in flash as a const byte array. `ROMFS::find(path)` returns a pointer to the decompressed data. Uses tinflate (tiny inflate) for gzip decompression. Enables SD-card-free operation with bundled default scripts.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: MEDIUM  
**JUSTIFICATION**: Required for Scripting (default Lua applets), OSD (fonts), and any feature that needs bundled resource files without an SD card.  
**GAPS** (entire feature missing):
- ROMFS file table (path → offset + compressed size + uncompressed size)
- Compile-time resource embedding (Rust `include_bytes!` + build.rs generator)
- tinflate gzip decompression (use `miniz_oxide` or `flate2` crate)
- `find(path) -> Option<&[u8]>` API
- Build tool to compress and generate the file table

---

## UTILITY SYSTEMS

---

### LIBRARY: AP_Stats
**FILES**: 2  
**LINES**: 319  
**PURPOSE**: Persistent flight statistics. Tracks total flight time (seconds), number of flights, total distance flown, first flight date. Persisted to StorageManager on disarm. Displayed in GCS and OSD. `STATUSTEXT` reporting of lifetime stats. Simple but useful for maintenance scheduling.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: Nice-to-have for fleet management and maintenance. Not flight-critical. Easy to implement once StorageManager exists.  
**GAPS** (entire feature missing):
- Flight timer (start on arm, stop on disarm)
- Flight counter increment on arm
- Distance accumulator (integrate ground speed)
- StorageManager persistence backend
- GCS `STATUSTEXT` reporting
- MAVLink parameter exposure (read-only stats params)

---

### LIBRARY: AP_RTC
**FILES**: 7  
**LINES**: 686  
**PURPOSE**: Real-time clock management. Sets system time from GPS or MAVLink `SYSTEM_TIME` message. Provides `get_utc_usec()` and `get_local_ms()`. `JitterCorrection` module smooths the GPS time updates to prevent time jumps. Used by Logger (timestamps), camera geo-tagging (EXIF time), and Remote ID (timestamp in auth messages).  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: GPS time is required for log timestamps (post-analysis), EXIF photo timestamps, and Remote ID authentication. Without it, all logs have relative timestamps only.  
**GAPS** (entire feature missing):
- GPS time synchronization (set RTC from GPS `time_epoch_usec`)
- MAVLink `SYSTEM_TIME` synchronization
- `JitterCorrection` PLL for smooth time updates
- `get_utc_usec() -> u64` API
- Monotonic fallback (system uptime when GPS unavailable)
- Time zone / local time offset parameter

---

### LIBRARY: AP_Button
**FILES**: 2  
**LINES**: 552  
**PURPOSE**: Physical button input handler. Up to 4 GPIO buttons with configurable press actions: arm, disarm, mode change, custom MAVLink command. Short press vs. long press differentiation. Debounce. RC channel simulation (button maps to RC channel for mode switching). Used for safety switch functionality beyond the dedicated safety switch.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: LOW  
**JUSTIFICATION**: Convenience feature for physical button inputs. Not flight-critical. Easy to add once GPIO abstraction is solid.  
**GAPS** (entire feature missing):
- GPIO debounce logic
- Short/long press differentiation
- Action dispatch table (per-button action enum)
- RC channel simulation for mode switching
- MAVLink command send on press

---

### LIBRARY: AP_Relay
**FILES**: 5  
**LINES**: 1,013  
**PURPOSE**: GPIO relay output control. Up to 16 relay outputs with configurable GPIO pins. Relay types: GPIO, RCIN (passthrough RC), DroneCAN relay, MAVLink passthrough. Used by: Camera (relay shutter), Parachute (relay fire), Gripper (EPM relay), ICEngine (ignition). MAVLink `MAV_CMD_DO_SET_RELAY`. RC channel trigger.  
**MERIDIAN HAS**: NO — crate does not exist  
**PRIORITY**: HIGH  
**JUSTIFICATION**: Foundation for multiple subsystems — Camera relay shutter, Parachute deployment, EPM gripper, ICEngine ignition all depend on relay output. Without it, those subsystems are incomplete.  
**GAPS** (entire feature missing):
- Relay output manager (up to 16 logical relays → GPIO pins)
- GPIO relay backend (HAL GPIO set/clear)
- DroneCAN relay backend
- RCIN passthrough relay
- `on()/off()/toggle()` API with pin mapping
- MAVLink `MAV_CMD_DO_SET_RELAY` command handler
- RC channel trigger
- Relay params (pin assignment, default state, inverted flag)

---

## SUMMARY TABLE

| Library | Files | Lines | Meridian Has | Priority |
|---|---|---|---|---|
| AP_Camera | 31 | 5,310 | YES (partial) | HIGH |
| AP_Mount | 42 | 14,760 | YES (partial) | HIGH |
| AP_Gripper | 9 | 773 | STUB | MEDIUM |
| AP_Winch | 9 | 1,164 | STUB | MEDIUM |
| AP_Parachute | 5 | 456 | STUB | HIGH |
| AP_LandingGear | 3 | 486 | STUB | MEDIUM |
| AC_PrecLand | 17 | 2,147 | YES (partial) | HIGH |
| AC_Sprayer | 3 | 304 | STUB | LOW |
| AP_OSD | 68 | 7,599 | YES (partial) | MEDIUM |
| AP_Notify | 75 | 8,569 | YES (partial) | MEDIUM |
| AP_ADSB | 54 | 6,405 | STUB | HIGH |
| AP_OpenDroneID | 4 | 1,266 | STUB | CRITICAL |
| AP_VideoTX | 7 | 2,429 | NO | LOW |
| AP_AIS | 4 | 1,369 | NO | LOW |
| AP_Radio | 16 | 7,677 | NO | SKIP |
| AP_Param | 7 | 4,668 | STUB | CRITICAL |
| AP_Logger | 27 | 11,080 | STUB | CRITICAL |
| AP_Filesystem | 26 | 6,458 | NO | HIGH |
| AP_Scheduler | 7 | 1,340 | NO (RTIC equiv) | MEDIUM |
| AP_Scripting | 443 | 8,996 | NO | MEDIUM |
| AC_AutoTune | 9 | 5,272 | YES (partial) | HIGH |
| AP_GyroFFT | 17 | 9,868 | YES (partial) | HIGH |
| AP_Soaring | 9 | 1,155 | NO | LOW |
| AP_Terrain | 8 | 2,225 | YES (partial) | HIGH |
| AC_Autorotation | 4 | 833 | NO | MEDIUM |
| AP_ICEngine | 5 | 1,123 | NO | MEDIUM |
| AP_EFI | 26 | 3,050 | NO | MEDIUM |
| AP_Generator | 18 | 4,065 | NO | LOW |
| AP_Torqeedo | 9 | 1,964 | NO | SKIP |
| AP_WheelEncoder | 10 | 1,304 | NO | LOW |
| AP_LeakDetector | 8 | 258 | NO | SKIP |
| AP_BLHeli | 3 | 2,165 | NO | LOW |
| AP_ESC_Telem | 8 | 1,420 | NO | HIGH |
| AP_AccelCal | 4 | 1,132 | NO | HIGH |
| AP_TempCalibration | 3 | 337 | NO | MEDIUM |
| AP_Declination | 10 | 506 | NO | HIGH |
| AP_Quicktune | 2 | 856 | NO | MEDIUM |
| AP_IOMCU | 12 | 5,007 | NO | MEDIUM |
| AP_BoardConfig | 5 | 1,610 | STUB | HIGH |
| AP_FlashStorage | 4 | 1,131 | NO | HIGH |
| StorageManager | 4 | 696 | NO | HIGH |
| AP_DAL | 21 | 2,853 | NO | MEDIUM |
| AP_CheckFirmware | 6 | 4,291 | NO | LOW |
| AP_ROMFS | 4 | 938 | NO | MEDIUM |
| AP_Stats | 2 | 319 | NO | LOW |
| AP_RTC | 7 | 686 | NO | HIGH |
| AP_Button | 2 | 552 | NO | LOW |
| AP_Relay | 5 | 1,013 | NO | HIGH |

---

## PRIORITY BREAKDOWN

### CRITICAL (3)
Must exist before any production deployment.

1. **AP_OpenDroneID** — FAA mandate. Arm is blocked without it in US airspace.
2. **AP_Param** — Persistence backend missing. All tuning lost on reboot.
3. **AP_Logger** — Write backends missing. No logs = no debugging, no certification.

### HIGH (16)
Required for a complete, production-quality autopilot.

AP_Camera (complete backends), AP_Mount (complete backends), AP_Parachute (complete), AC_PrecLand (backends), AP_ADSB (backends), AP_Filesystem, AC_AutoTune (heli + save), AP_GyroFFT (multi-axis + notch integration), AP_Terrain (GCS state machine + cache), AP_ESC_Telem, AP_AccelCal, AP_Declination, AP_BoardConfig, AP_FlashStorage, StorageManager, AP_RTC, AP_Relay

### MEDIUM (14)
Needed for full parity; can be phased.

AP_Gripper, AP_Winch, AP_LandingGear, AP_Scheduler (monitoring), AP_Scripting, AC_Autorotation, AP_ICEngine, AP_EFI, AP_TempCalibration, AP_Quicktune, AP_IOMCU, AP_DAL, AP_ROMFS, AP_BoardConfig

### LOW (8)
Niche or convenience features.

AC_Sprayer, AP_VideoTX, AP_BLHeli, AP_Generator, AP_WheelEncoder, AP_CheckFirmware, AP_Stats, AP_Button, AP_AIS

### SKIP (3)
Out of scope for Meridian's target platforms.

AP_Radio (embedded RC chips), AP_Torqeedo (marine only), AP_LeakDetector (submarine only)

---

*Generated: 2026-04-02 | Source: ArduPilot libraries scan + Meridian crate analysis*
