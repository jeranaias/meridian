# Meridian vs ArduPilot — Master Parity Gap List

## Source: 6 verification agents, 2,438+ lines of reports

---

## CRITICAL (flight safety — must fix before any hardware flight)

### EKF (from parity_ekf.md)
1. **GPS delay buffer bypassed** — EKF runs on current IMU, not delayed state. Introduces 2.2m bias at 10m/s.
2. **Fvq Jacobian block zeroed** — velocity-quaternion coupling missing in covariance F matrix.
3. ~~GPS loss Absolute→None fallback~~ — **FIXED** (gps_lost_count added)
4. ~~ConstrainStates() never called~~ — **FIXED** (constrain_states() added to predict)
5. ~~Wrong constants (ABIAS, MAGB, MAGE, MAG_I_GATE)~~ — **FIXED**

### Navigation (from parity_nav_modes.md)
6. **L1 lateral accel coefficient wrong** — Meridian uses 2V²sin/L1, ArduPilot uses K_L1=4*damping²=2.25. 11% force error.
7. **SmartRTL loop pruning absent** — Only Douglas-Peucker, no loop detection. Vehicle retraces dangerous detours.
8. **Land detector only checks altitude** — Missing ArduPilot's 8-condition check (throttle, angle, accel, vertical speed).
9. **Motor shutdown on landing absent** — Motors don't spool down after land detection.
10. **Failsafe priority escalation absent** — Multiple simultaneous failsafes not resolved.

### Modes (from parity_nav_modes.md)
11. **QuadPlane transition + Q_ASSIST absent** — VTOL mode IDs exist but nothing implemented.

### Motors (from parity_motors.md)
12. **Thrust linearization formula wrong** — Using t^1.5 approximation instead of quadratic inversion. 2-3% error.

---

## HIGH (must fix before extended flight testing)

### EKF
- Optical flow fusion absent
- Airspeed/sideslip fusion absent
- GSF (Gaussian Sum Filter) yaw estimator absent
- WMM magnetic declination table absent
- Ground effect baro compensation absent
- Lane switching absent

### Control (pending parity_control.md)
- To be filled from control parity agent

### Motors
- Only 5 of 42 frame types (missing 37 frame presets)
- Battery+altitude compensation not wired into mix()
- Voltage compensation LP filter absent
- MOT_YAW_HEADROOM configurable absent
- Thrust boost / motor-loss mode absent
- Per-motor directional yaw headroom absent
- Limit flag reporting absent

### Navigation
- L1 prevent_indecision completely absent (wp_overshoot is different)
- Crosstrack integral dimensionally wrong (meters vs radians)
- SmartRTL breadcrumb buffer too small (50 vs 300)
- Terrain following structurally disconnected from WaypointNav

### Modes
- RTL missing FINAL_DESCENT, cone-slope, terrain/rally target
- 6+ copter modes absent (AUTOTUNE, SYSTEMID, TURTLE, FLOWHOLD, etc.)
- Sub missing AUTO, GUIDED, POSHOLD, SURFTRAK modes
- ~30 MAV_CMD IDs absent in mission system

### Failsafe
- 5 of 7 FS_OPTIONS flags absent
- SMARTRTL_LAND, BRAKE_LAND actions absent
- Thrust loss detection absent
- IMU consistency not computed internally

### Payload/OSD
- Parachute deployment entirely absent
- Gripper/Winch/LandingGear/Sprayer absent
- 7 of 9 camera backends missing
- 11 of 14 mount backends missing
- No MAVLink Camera Protocol v2
- 18 OSD items missing
- 26 buzzer tunes missing
- No ADSB receiver backends
- No sensor backends for proximity

---

## Status
- CRITICAL items 3,4,5 FIXED (EKF constants + GPS fallback + state constraining)
- 9 CRITICAL items remaining
- ~50 HIGH items remaining
- Many MEDIUM/LOW items in individual reports
