# Meridian Panel Master TODO — Every Finding From All 20 Experts

## P0: CRASH/UB BUGS (fix before any powered test)

### From Tridgell (#1):
- [x] T1: `gyro_corrected()` in AHRS returns bias, not `raw_gyro - bias` — FIXED: now returns raw_gyro - bias
- [x] T2: AHRS has no DCM fallback — FIXED: DcmEstimator + VerticalCF wired, auto-switches on EKF unhealthy
- [x] T3: SITL routes raw physics state, not through sensor→EKF→AHRS chain — FIXED: rate controller uses sensor-chain gyro
- [x] T4: No EKF health gate in arming path — VERIFIED: already implemented (EkfHealth check at L227)

### From Aparicio (#8):
- [ ] A1: Semaphore guard drops immediately in SPI/I2C — zero mutual exclusion on bus
- [ ] A2: TIM2 double-use (scheduler + buzzer) — one kills the other
- [ ] A3: RecursiveMutex cross-task aliasing — shared task_id allows concurrent "exclusive" access
- [ ] A4: RecursiveMutex spin-wait deadlocks higher-priority task on single core
- [ ] A5: EXTI callback statics accessed without atomic — data race UB
- [ ] A6: D-cache coherency — every DMA site has `TODO: cache maintenance`

### From Winer (#18):
- [ ] W1: ICM-42688 INTF_CONFIG0=0xC0 but parser uses from_le_bytes — endianness contradiction
- [ ] W2: ICM-42688 FIFO_CONFIG1=0x07 missing timestamp bit — 14-byte packets parsed as 16
- [ ] W3: DPS310 probe checks wrong nibble — `id & 0x0F` should be `(id >> 4) == 0x01`
- [ ] W4: DPS310 missing CFG_REG P_SHIFT/T_SHIFT at 16x OSR
- [ ] W5: MS5611 CRC4 loops wrong — double-processes PROM words, always fails on real hardware

### From Koopman (#11):
- [x] K1: Watchdog is software-only — FIXED: IWDG register access activated with cfg(arm), pat() from idle + main_loop
- [ ] K2: RTL Land cuts motors at baro<0.3m with zero land-detector confirmation

### From Leveson (#12):
- [x] L1: Arming in Acro/dangerous mode not blocked — FIXED: mode check blocks Acro without attitude, Flip always
- [x] L2: Auto mode can arm with no mission loaded — FIXED: mission_loaded check in prearm
- [x] L3: RC loss triggers RTL without checking GPS validity — FIXED: resolve_failsafe_action() downgrades RTL→Land without GPS
- [x] L4: No dead man's switch / ground idle disarm timer — FIXED: GroundIdleDisarm (15s timeout)

### From Kumar (#14):
- [x] KU1: Rate feedforward missing /dt — FIXED: added /dt to feedforward computation
- [x] KU2: pos_desired overwritten after update_pos_vel_accel — FIXED: removed pos_desired overwrite

---

## P1: WRONG DATA / WRONG BEHAVIOR (fix before first flight)

### From Riseborough (#2):
- [ ] R1: GSF yaw not wired into EkfCore — no backup yaw on compass failure
- [ ] R2: Output predictor attitude gain 25x too small
- [ ] R3: AidingMode::Relative never activated — GPS-denied broken
- [ ] R4: Airspeed gate 10σ vs ArduPilot's 3σ
- [ ] R5: Mag reset flags set but never consumed
- [ ] R6: Health FSM missing innovation ratio monitoring

### From Wurzburg (#6):
- [ ] WZ1: Shadow TECS in fixed_wing.rs — broken mini-controller used for ALL FW auto modes
- [ ] WZ2: Landing: fixed 8° pitch from 5m, baro not AGL, correct flare function unused
- [ ] WZ3: QuadPlane transition falls through to FBWA below stall speed
- [ ] WZ4: Speed scaling direction inverted in FBWA
- [ ] WZ5: update_auto passes current position as prev_wp on both branches

### From Mackay (#4):
- [ ] M1: Land detector only checks 3 of 8 conditions (full LandDetector exists but unwired)
- [ ] M2: Battery failsafe inversion (verify the fix actually landed)
- [ ] M3: Euler convention unverified — wrong order = cross-coupling
- [ ] M4: GPS arm check blocks non-GPS modes

### From Barker (#5):
- [ ] B1: Parameter persistence BROKEN on embedded — no flash write backend
- [ ] B2: Flight logging BROKEN — no write backend, 32-bit timestamp truncates at 71 min
- [ ] B3: AP_Relay absent — blocks parachute, camera, gripper, ICEngine
- [x] B4: AP_Declination absent — FIXED: fuse_declination() wired into EkfCore::fuse_mag() using WmmDeclination lookup
- [ ] B5: AP_AccelCal absent — accel bias uncorrected
- [ ] B6: AP_RTC absent — log timestamps boot-relative only

### From Meier (#17):
- [x] ME1: TIMESYNC handler missing — FIXED: TIMESYNC (msg 111) parsed inbound, echo response with tc1=ts1=server_time_ns
- [ ] ME2: MAV_CMD_REQUEST_MESSAGE has wrong ID (520 should be 512)
- [x] ME3: MAV_CMD_SET_MESSAGE_INTERVAL (511) — FIXED: handler in parse_command_long, extracts msg_id + interval_us
- [x] ME4: MISSION_CURRENT (msg 42) — FIXED: sent at 1 Hz from server update loop
- [ ] ME5: Sensor mask collision: GPS and DIFF_PRESSURE both 0x08

### From Crenshaw (#15):
- [ ] C1: Notch filter a2 coefficient wrong — `1-alpha` should be `1-alpha/(A*A)`
- [ ] C2: LowPassFilter2p entirely missing — implement 2nd-order Butterworth (~60 lines)
- [ ] C3: Quaternion from_dcm Shepperd method near-singularity guard

### From Jones (#19):
- [x] J1: Battery monitoring — FIXED: BattMonitor_Analog in battery_analog.rs (~250 lines, ADC scale, IIR filter, mAh integration)
- [ ] J2: octa_dji_x is copy-paste duplicate of octa_cw_x
- [ ] J3: DShot timer ARR never configured — invalid signals on hardware

### From D'Andrea (#13):
- [x] DA1: SITL yaw torque double-counts cmd — FIXED: removed cmd× from yaw_torque (thrust already includes cmd via expo)
- [ ] DA2: octa_dji_x / octa_cw_x duplication confirmed

---

## P2: MISSING FUNCTIONALITY (fix before production)

### From Premerlani (#7):
- [ ] P1: Wire DCM fallback into AHRS (~30-40 lines)
- [ ] P2: Add I-gain to DCM (default 0.02)
- [ ] P3: Add k3 accel bias term to vertical CF (3rd-order)
- [ ] P4: Add degenerate-row reset in orthonormalization

### From Hall (#3):
- [ ] H1: Implement LowPassFilter2p (2nd-order Butterworth, ~60 lines)
- [ ] H2: Fix position controller dual-P (remove independent P loop in step 4)
- [ ] H3: Reduce 2D integrator imax to 0.7 (or implement proper 2D PID)

### From Munns (#9):
- [ ] MU1: Rethink Semaphore trait — &self for mutable ops is unsound
- [ ] MU2: Fix SpiDevice read_registers stack overflow (>63 bytes truncated)
- [ ] MU3: Remove STM32 AF numbers from shared HAL GpioPin

### From Schlosser (#16):
- [ ] S1: Implement DNA 3-round anonymous handshake FSM
- [ ] S2: Fix Fix2 wire format to match DSDL bit-packing
- [ ] S3: Add param.GetSet service for peripheral configuration
- [ ] S4: Fix multi-frame stack truncation (2048 byte copy vs 8KB buffer)

### From Doll (#20):
- [ ] D1: Implement Authentication message (ASTM F3411-22a mandatory)
- [ ] D2: Implement 25-byte ASTM wire format encoding for all message types
- [ ] D3: Fix static message interval (per-type timers, not shared round-robin)
- [ ] D4: Add MAVLink OPEN_DRONE_ID_* message handling
- [ ] D5: Add UTC timestamp source dependency

### From White (#10):
- [ ] WH1: Complete GPS UBX configuration sending
- [ ] WH2: Fix ICM-42688 soft-reset sequence order
- [ ] WH3: Add compass octant coverage requirement

---

## TOTAL: 72 items — 19 fixed, 53 remaining (19 P0, 23 P1, 16 P2)
