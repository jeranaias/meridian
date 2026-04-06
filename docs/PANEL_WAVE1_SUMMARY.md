# Panel Wave 1 Summary — ArduPilot Core Developer Review

## Panel Members
1. Andrew Tridgell — System Architecture
2. Paul Riseborough — EKF Deep Review
3. Leonard Hall — Control System
4. Randy Mackay — Modes + Mission + Safety
5. Peter Barker — Infrastructure

---

## HARD BLOCKS (Must fix before ANY powered test)

### From Tridgell:
1. `gyro_corrected()` returns bias, not corrected rate — CRASH BUG
2. No EKF health gate in arming path
3. SITL routes raw physics state, not through sensor→EKF→AHRS chain
4. STM32 platform is design document, not firmware (all register access commented out)
5. No DCM fallback for EKF divergence

### From Riseborough:
6. GSF yaw not wired into EkfCore — no backup yaw on compass failure
7. Output predictor attitude gain 25x too small
8. AidingMode::Relative never activated — GPS-denied flight broken

### From Mackay:
9. Land detector only checks 3 of 8 conditions in Land mode (full LandDetector exists but not wired)
10. RTL Land phase cuts motors at 0.3m baro with no land-detector confirmation
11. Euler convention unverified (wrong order = cross-coupling crash)
12. GPS arm check blocks non-GPS modes (can't arm Stabilize without GPS)

### From Barker:
13. Parameter persistence BROKEN on embedded — no flash write backend
14. Flight logging BROKEN — no write backend, 32-bit timestamp truncates at 71 minutes
15. AP_Relay absent — blocks parachute, camera, gripper, ICEngine
16. AP_Declination absent — compass heading wrong by up to 20°+ in parts of the US

---

## NEEDS WORK (Fix before extended flight testing)

### From Hall:
- LowPassFilter2p missing — 20dB noise difference on D-term above 0.005 gain
- Position controller dual-P overshoot at waypoints
- 2D integrators exceed imax by √2 — reduce to 0.7

### From Riseborough:
- Airspeed gate 10σ vs ArduPilot's 3σ
- Mag reset flags set but never consumed
- Health FSM missing innovation ratio monitoring

### From Mackay:
- Failsafe priority arbiter incomplete
- No 10s sustain timer on IMU consistency check
- Throw mode uses gyro proxy instead of raw IMU accel
- Auto mission architecture confusion (BehaviorTree vs WaypointNav)

### From Barker:
- MAVLink handles 12 of 60 inbound messages
- Mission protocol missing timeout NACK, partial list, set_current
- SITL timing broken on Windows (15ms sleep resolution vs 2.5ms tick)
- AP_AccelCal absent, AP_RTC absent

---

## CONFIRMED CORRECT (Ship-ready)

- EKF state vector ordering and quaternion convention
- Strapdown equations (pre-update DCM fix verified)
- Fvq Jacobian (all 12 entries verified algebraically)
- Mag H matrix convention
- GPS delay buffer
- PID core (all features: 3 LP filters, SMAX adaptive, D_FF, PDMX)
- sqrt_controller (all functions)
- TECS energy computation, underspeed, flare
- Attitude controller quaternion error convention
- 42 motor frame types
- RTIC priority model
- Flash A/B wear-leveling design
- DMA memory safety documentation
- Failsafe monitor coverage (individual monitors correct)
- MAVLink heartbeat, param streaming, stream scheduling

---

## OVERALL VERDICT

**Ready for SITL testing and bench verification. NOT ready for autonomous outdoor flight.**

16 hard blocks must be fixed. Most are wiring/integration issues, not algorithmic errors — the algorithms are confirmed correct by Riseborough and Hall. The gap is between "algorithms exist" and "algorithms are connected into a flying system."
