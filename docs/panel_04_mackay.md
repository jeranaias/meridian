# Panel Review — Randy Mackay, ArduCopter Lead
**Focus: Flight modes, mission system, failsafe, arming**
**Date: 2026-04-02**
**Reviewer note:** I've read the source, not just the gap list. I've also debugged enough
crash logs to know what the polished-looking code that kills things looks like. Some of
what's here is genuinely good. Some of it will kill the vehicle on first flight. I'll be
direct about which is which.

---

## 1. Can I arm and fly Stabilize?

**Short answer: Yes, conditionally. The happy path works. The edge cases will bite you.**

`update_stabilize()` does what it says. Stick to lean angle, yaw rate integration with
`wrap_pi`, throttle passthrough, builds a target quaternion. That is the right structure.
The constants are correct — 30 degree angle max, 200 deg/s yaw rate. This matches
ArduCopter defaults.

**What I like:** The yaw integration initializes from `input.yaw` on mode entry. That means
switching into Stabilize mid-flight won't snap the yaw target to zero and yaw the copter
into the ground. I've seen that bug kill vehicles. This version handles it correctly.

**What worries me:**

The `ModeOutput::AttitudeTarget` goes somewhere downstream that I cannot see from this
file. The attitude controller, the mixer, the motors — all of that is outside this review
scope. Stabilize is the mode most dependent on the downstream path being correct. If the
attitude controller is wrong, Stabilize fails before it can fail. I cannot rate this FLY_IT
without seeing that chain.

More concretely: `Quaternion::from_euler(roll_target, pitch_target, self.stabilize_yaw)`.
This is Euler → quaternion. The order matters — intrinsic ZYX is the standard for aircraft.
If this is XYZ or any other convention, roll and pitch will cross-couple under yaw and the
vehicle will oscillate in a circle. This has to be verified. It is not obvious from the
function name alone.

**Arming blocks for Stabilize:**

The arming check at `check_prearm()` requires 3D GPS fix, 6+ satellites, HDOP < 2.0 even
for GPS-not-required modes like Stabilize. This is overly restrictive. ArduPilot gates GPS
checks on whether the current mode requires GPS. Stabilize does not. On a cloudy day with
marginal GPS, this will block arming even though Stabilize could fly safely.

The `disabled_mask` allows bypassing this, but the default config arms all checks. Out of
the box, you cannot arm in Stabilize without GPS. That is not what ArduCopter does.

The IMU consistency check (`check_imu_consistency`) is instantaneous with no 10-second
sustain timer. On a vehicle with vibration — which is every vehicle — a single bad tick
from a motor starting nearby will fail the IMU check and block arming. This will cause
exactly the kind of field confusion where the pilot keeps trying to arm, the check keeps
failing for 50ms at a time, and eventually they disable the check entirely. Then they fly
with a miscalibrated IMU.

**Rating: TEST_MORE**

The Stabilize control path is structurally correct. I cannot call it FLY_IT until the
attitude controller downstream is verified and the Euler convention is confirmed.

---

## 2. Can I fly a waypoint mission in Auto?

**Short answer: Simple missions will work. Anything real-world will not.**

`update_auto()` calls `self.wp_nav.update()` which returns a position target. This is
correct in structure. The WaypointNav with dual NAV/DO cursors, DO_JUMP counter, and the
full `ActionKind` set is the best-implemented subsystem I read.

**What I like:** The behavior tree mission engine is a genuine improvement over ArduPilot's
linear list in terms of expressibility. The dual-cursor pattern (NAV blocks, DO fires
alongside) is correct. DO_JUMP with a `jump_counters` heap keyed by sequence number is
correct and avoids the ArduPilot gotcha where jump counts live in the mission item and get
corrupted across reboots.

**What kills missions in the field:**

The `eval_action()` in `BehaviorTree` evaluates every action type including NAV commands.
But `update_auto()` in `multirotor.rs` only calls `self.wp_nav.update()`. The behavior
tree in `meridian-mission` and the `WaypointNav` in `meridian-nav` appear to be two
parallel mission implementations that are not connected to each other. Which one actually
drives Auto mode? If it's WaypointNav, the full ActionKind set in the behavior tree is
decorative. If it's the behavior tree, then `update_auto()` is bypassing it entirely.
This is architectural confusion that must be resolved before any autonomous flight.

The `ActionKind::RTL => BtStatus::Running` — this never completes. RTL in a mission
(MAV_CMD_NAV_RETURN_TO_LAUNCH) blocks the mission cursor forever. Missions that end with
RTL will hang.

`ActionKind::LoiterTurns` and `ActionKind::LoiterUnlim` both return `Running`
unconditionally. LoiterTurns is supposed to count turns and complete. LoiterUnlim is
correct (holds forever). But LoiterTurns in a waypoint survey mission counts turns via
the circle heading — that heading tracking doesn't exist in `eval_action()`.

**Per-waypoint speed via MAV_CMD_DO_CHANGE_SPEED:** `ChangeSpeed` returns `Success`
immediately in `eval_action()`, which is correct for a DO command. But the speed value
is never propagated to the position controller. The action fires and is forgotten. Every
mission that uses variable speeds (survey legs at slow speed, transit at fast) will ignore
the speed commands.

**MAV_CMD 194 mapped as SetReverse:** This is `NAV_TAKEOFF_LOCAL` in the MAVLink spec.
Any mission with a local-frame takeoff will command the rover to reverse instead. On a
copter this does nothing visible until someone loads a mission that uses this command and
wonders why it doesn't take off. Documented as a bug in the previous review; remains
unfixed as of the code I read.

**Rating: TEST_MORE** for simple fixed-waypoint missions (fly to point, land).
**DONT_FLY** for missions with speed changes, loiter turns, or RTL-at-end.

---

## 3. Does RTL work correctly?

**Short answer: The state machine is correct. The landing termination is not.**

All 6 states are present: Climb → Return → Loiter → FinalDescent → Land → Complete.
This is the full ArduPilot state machine. The cone-slope altitude calculation matches the
ArduPilot formula exactly: `min(rtl_alt, max(dist * slope, rtl_alt_min))`. The loiter
timeout fires correctly. FinalDescent adds the intermediate hover before committing to
land — that was a gap in earlier versions and it's been addressed.

**The RTL landing termination problem:**

In `RtlState::Land`:
```rust
if input.altitude < 0.3 {
    self.rtl_state = RtlState::Complete;
}
```
`RtlState::Complete` returns `ModeOutput::Idle` immediately. This is motor shutdown.
The problem: `input.altitude` is barometer altitude, which at 0.3m above home may be
0.3m above the launch point but 2m above the actual landing surface if terrain has changed
or the vehicle drifted during flight. The motors cut at Idle with no land-detector
confirmation, no throttle-at-minimum check, and no attitude-error check.

On a flat field with good baro, this may work. But the proper ArduPilot sequence at this
point runs the full 8-condition land detector and only shuts down when ALL conditions are
met for 1 second. The Meridian RTL Land phase skips all of that and cuts motors at 0.3m
baro altitude.

If there is any wind, any sloshing in the vehicle's motion, or any baro lag, the motors
will cut while the vehicle is still 0.5-1.0m in the air and the vehicle drops. I have
read crash logs from exactly this failure mode.

**No rally points:** The comment `// GAP 19: Rally point / terrain selection would go here`
is honest about the gap. For any operational flight over terrain or near obstacles, RTL to
home without rally point consideration is a real hazard. Acceptable for a flat field test,
not acceptable for any operational environment.

**Rating: TEST_MORE**

The state machine works. The land termination in the RTL Land state is aggressive and
needs the same land-detector integration as standalone Land mode before I'd use RTL over
anything other than the original takeoff spot.

---

## 4. Does Land detect the ground reliably?

**Short answer: No. Three of eight conditions. This will cause premature motor shutdown.**

ArduPilot's land detector checks eight conditions simultaneously for one second before
cutting motors:
1. Motor output at lower limit
2. Throttle mix at minimum
3. No large angle requested by pilot (< 15 deg)
4. No large attitude error (< 30 deg)
5. Accelerometer stationary (< 1.0 m/s² filtered at 1Hz)
6. Descent rate low (|vz| < 1.0 m/s)
7. Rangefinder reading < 2m (if healthy)
8. Weight-on-wheels (if sensor present)

Meridian checks:
1. `input.altitude < 0.5m` — baro altitude, not a motor or throttle check
2. `|climb_rate| < 0.5 m/s` — tighter threshold than ArduPilot's 1.0 m/s
3. `climb_rate <= 0.0` — not ascending

The missing check that kills copters in the field is condition 3: **large angle request.**
If a pilot is touching the roll/pitch stick during landing (as pilots do when fighting
drift), the land detector should not trigger. Meridian has no angle-input check. A pilot
fighting a 20-degree crosswind drift will have the sticks pushed well off center; Meridian
will detect 0.3m altitude and 0.4 m/s descent and cut the motors while the vehicle is
still correcting drift. It will tip over and break a prop.

Condition 5 (accelerometer stationary) is also missing. This means the land detector
can fire during a jounced, turbulent descent — the vehicle bounces, altitude reads low
for one second, motors cut, and the vehicle is still a foot off the ground.

The 0.5 m/s descent threshold (vs ArduPilot's 1.0 m/s) is also tighter than it needs to
be. With this threshold, a normal 0.6 m/s landing flare will reset the timer and delay
motor shutdown. Not a crash risk, but it will cause vehicles to sit on the ground with
motors spinning longer than expected.

**Rating: DONT_FLY until angle-request check is added.**

---

## 5. Do failsafes fire correctly?

**The good:** All the monitors exist. TimeoutMonitor (RC, GPS, GCS), BatteryMonitor (low
and critical with separate actions), EkfHealthMonitor with a 5-second innovation window,
GeofenceMonitor, CrashDetector, DeadReckoningMonitor, TerrainMonitor, WatchdogMonitor.
The individual monitor logic is sound. The watchdog with a 2s motor-min and 3s disarm
threshold is correct.

**The fatal gap: no priority arbiter.**

Every monitor returns an `Option<(FailsafeReason, FailsafeAction)>` independently.
There is no `FailsafeManager` that collects all triggered actions and selects the
highest-priority one. The calling code — which I cannot see from here — must implement
that arbitration. If it doesn't, then simultaneous RC loss and battery critical will
fire two conflicting actions: one says RTL, one says Land immediately. Which wins depends
on the order the calling loop checks the monitors. That is not deterministic.

ArduPilot's `events.cpp` has an explicit priority list:
- If battery critical while airborne: Land immediately (not RTL — you may not have enough
  battery to reach home)
- If RC loss in Auto: continue mission
- If RC loss in Manual: RTL
- If GCS loss in Guided: RTL or hold position
- If GPS loss in GPS-required mode: AltHold if available, Land if not

None of these cascade rules are implemented in the monitors. The monitors only fire their
pre-configured action regardless of the current mode or other active failsafes.

**The battery_continue logic inversion is a flight-critical bug.**

From `failsafe/src/lib.rs` (as analyzed in the previous review):
```rust
FailsafeReason::BatteryLow | FailsafeReason::BatteryCritical => {
    self.battery_continue_if_not_landed && !is_landed
}
```
This returns `true` (suppress failsafe) when the battery flag is set AND the vehicle is
NOT landed. The ArduPilot semantics of `BATTERY_CONTINUE_IF_NOT_LANDED` is: only trigger
battery failsafe when the vehicle is NOT on the ground (suppress it if already landed,
because landing motors cut the battery faster). The correct logic is:
`suppress when is_landed`. As written, the code suppresses battery failsafe during flight
and allows it on the ground — exactly backwards. I confirmed this in the code I read.
This is a flight-critical inversion: your battery warning system is silent when you need
it most.

**Thrust loss check is the wrong failure mode.**

`MotorMonitor` detects one motor running at 1.5x the average output. That detects a
failed motor. What it does NOT detect is the scenario where all motors are running but the
vehicle is descending with throttle at 90%+ — underpowered, worn props, high altitude,
cold air, overweight payload. ArduPilot's `thrust_loss_check()` monitors this specifically:
sustained descent while level and throttle > 90% for 1 second. Meridian will not catch
this and the vehicle will descend into the ground with motors at full.

**Yaw imbalance monitor is declared but not implemented.** `YawImbalance` exists as a
`FailsafeReason` variant. No monitor uses it. In ArduPilot, sustained yaw I-term saturation
(> 75% for 10 seconds) triggers a warning and eventually a mode change. Without this, a
bent motor/prop producing a yaw moment will be invisible to the failsafe system.

**Rating: DONT_FLY for autonomous missions until the priority arbiter is written and the
battery logic inversion is fixed.**

---

## 6. Are arming checks complete?

The arming check coverage is genuinely good. Baro, compass, GPS, IMU, EKF, battery, RC,
board voltage, safety switch, lean angle, logging, fence, motor interlock/E-stop conflict,
RC throttle failsafe position, airspeed, OA health, altitude validity — all present.
The `disabled_mask` bitmask approach works correctly.

**Problems I'd catch in the field:**

The IMU consistency check (`check_imu_consistency`) uses squared-magnitude comparison
for the difference vector:
```rust
let accel_diff_sq = dx*dx + dy*dy + dz*dz;
if accel_diff_sq > ACCEL_THRESHOLD * ACCEL_THRESHOLD {
```
`ACCEL_THRESHOLD = 0.75 m/s²`. So the gate is `|diff|² > 0.5625`. This compares the
magnitude of the 3D difference vector against the threshold, which is what ArduPilot does.
The math is correct. However, as the code comment acknowledges: no 10-second sustain timer.
The instantaneous check fires on every tick. A single IMU glitch from motor startup, a
passing truck, or even a power supply transient will block arming. This will manifest as
"arming works at the bench but not in the field."

GPS check runs even for modes that don't require GPS. I mentioned this under Stabilize.
It also means you cannot arm in AltHold (which requires only baro) if GPS is marginal.
This is operationally limiting.

The `rc_throttle_fs_value > 0` guard in the throttle failsafe check means this check is
skipped if the failsafe threshold is not configured. Out of the box, this check is bypassed
entirely. ArduPilot's behavior is to require the throttle to be above the failsafe level —
the check should be mandatory, not optional.

**Missing: terrain database check per mode.** ArduCopter's `terrain_database_required()`
assesses whether the current flight mode (specifically Auto with terrain-following) needs
the terrain database. Meridian's arming check doesn't assess this. A mission that uses
terrain-relative altitude will arm and fly without a terrain database, then navigate
terrain-relative with no data.

**Missing: winch check.** Low impact for non-winch vehicles but worth noting for completeness.

**Rating: TEST_MORE**

The coverage is good. The sustain-timer gap will cause real arming frustration. The GPS
check running on non-GPS modes needs per-mode gating.

---

## 7. Mode ratings — what would fail on real hardware

| Mode | Rating | Reason |
|------|--------|--------|
| Stabilize | TEST_MORE | Control path structurally correct; Euler convention unverified; GPS arm check too restrictive |
| AltHold | TEST_MORE | Core AltHold logic is sound; same GPS arm-check issue; needs integration test |
| Loiter | TEST_MORE | Does NOT capture entry position — holds input.position each tick. Will drift if position update is noisy at mode entry. Fix: capture position in enter() |
| RTL | TEST_MORE | State machine correct; Land phase cuts motors at 0.3m baro without land detector; no rally points |
| Auto | TEST_MORE (simple) / DONT_FLY (complex) | Architectural confusion: BehaviorTree vs WaypointNav not reconciled; speed commands ignored; RTL action never completes |
| Guided | TEST_MORE | Position-target sub-mode works; velocity-only, acceleration, posvelaccel sub-modes not dispatched |
| Land | DONT_FLY | 3/8 conditions; missing angle-input check; will trigger under pilot input at landing |
| AltHold | TEST_MORE | Fine |
| Circle | TEST_MORE | Core orbit works; no move-to-edge approach; in-flight radius change absent |
| Brake | FLY_IT | Captures position on entry, holds it, zero velocity. Correct. |
| PosHold | TEST_MORE | Core logic works; wind compensation missing; pilot-override blend not complete |
| SmartRTL | TEST_MORE | Breadcrumb path correct; wait_cleanup pause and pre-land position run not present; synchronous cleanup causes loop jitter |
| Throw | DONT_FLY | Uses gyro+velocity proxy instead of raw IMU accel; will false-trigger on vibration; will miss clean throws |
| Acro | TEST_MORE | Rate passthrough correct; air-mode absent; no virtual flybar for heli |
| Sport | TEST_MORE | Correct; separate angle_max for sport not distinguished |
| Drift | TEST_MORE | Core correct; throttle assist velocity feed-forward simplified |
| Flip | TEST_MORE | State machine correct; no pre-flip stability check; direction selection from sticks absent |
| ZigZag | TEST_MORE | Line A/B and orthogonal offset implemented; auto-spray not wired; crosstrack reporting absent |
| GuidedNoGPS | TEST_MORE | Attitude/throttle injection correct; MAVLink SET_ATTITUDE_TARGET wiring unverified |
| Follow | DONT_FLY | Target-lost failsafe not wired; velocity feed-forward absent; vehicle will overshoot target on acceleration |
| Autotune | DONT_FLY | Runs Stabilize loop only; axis sequencing, gain save/restore absent; will not tune anything |
| Turtle | DONT_FLY | Returns zero rates only; DShot motor direction reversal not present; this mode will do nothing useful |
| FlowHold | DONT_FLY | Complete stub; no optical flow integration |
| AvoidAdsb | TEST_MORE (stub known) | Routes to Loiter; acceptable if ADS-B disabled |
| SystemId | DONT_FLY | Complete stub; no chirp/doublet waveform |
| Heli-Stabilize | DONT_FLY | No mode exists |
| Heli-Acro | DONT_FLY | No mode exists |
| Autorotate | DONT_FLY | `FlightModeId::Autorotate` doesn't exist; GCS command 26 silently falls to Loiter |

---

## 8. Minimum set for first flight: Stabilize → AltHold → Loiter → RTL

**Can this chain work?**

### Stabilize
Works if the downstream attitude controller is correct and the Euler convention is ZYX.
Arm-check GPS restriction needs to be mode-gated or disabled for Stabilize. With those two
things confirmed, Stabilize is flyable.

### AltHold
The AltHold implementation is clean. Deadband 0.05, climb/descend rate mapping, altitude
PID via `pos_controller.update_altitude()`. The position controller's altitude loop needs
to be verified (gains, anti-windup, integrator limits) but the mode wiring is correct.
AltHold → flyable if the position controller altitude loop is tuned.

### Loiter
The Loiter gap matters here. `update_loiter()` returns `ModeOutput::PositionTarget` with
`position: input.position` — it reads the current position every tick rather than holding
the captured entry position. On a vehicle with GPS noise or EKF lag, the "hold position"
command will drift with the noise. The vehicle will wander. This needs to be fixed: capture
`input.position` in `enter()` for Loiter and hold that. AltHold entry correctly captures
`input.altitude`; Loiter should do the same for XY.

### RTL
RTL will perform Climb → Return → Loiter → FinalDescent. The danger is at Land
termination. The 0.3m baro-altitude motor-cut is too aggressive. Before using RTL in the
first flight test chain, add the land-detector check to the RTL Land state (the same
3-condition lightweight check used in `update_land()`, not the full 8-condition check, but
at minimum check that `climb_rate <= 0` and altitude is stable for 1 second).

### Summary verdict on first-flight chain

| Step | Ready? | Blocker |
|------|--------|---------|
| Stabilize | Almost | Euler convention verification; mode-gate GPS arm check |
| AltHold | Almost | Position controller altitude loop verification |
| Loiter | Not quite | Entry position not captured; will drift |
| RTL | Not quite | Land phase cuts motors prematurely; land detector bypass |

---

## Top-10 things that will kill the vehicle, ranked by probability

1. **Battery failsafe logic inverted** — silent battery warnings during flight. Vehicle
   descends silently into ground or water with no warning to GCS.

2. **Land mode cuts motors at 0.3m without angle-input check** — pilot fighting crosswind
   at landing has sticks off-center; land detector fires; vehicle tips at 0.3m; propellers
   hit ground at angle; crash.

3. **RTL Land phase uses same 0.3m baro-altitude motor-cut** — same failure mode as #2,
   but triggered at the end of an autonomous return. No one is touching sticks; the
   slightest baro lag and the vehicle drops.

4. **Throw mode false-triggers on vibration** — gyro+velocity proxy for throw detection
   will fire on motor run-up or hard pavement placement. Armed vehicle detects "throw,"
   spins motors, and launches from hand. Multiple injuries documented in ArduPilot bug
   tracker from this class of failure. DO NOT ARM THROW MODE WITH THIS DETECTION.

5. **Auto mission BehaviorTree/WaypointNav confusion** — unclear which system actually
   drives the vehicle. If BehaviorTree is wired and `ActionKind::RTL` never completes,
   the mission hangs at the RTL node and the vehicle loiters at the last waypoint
   indefinitely.

6. **No failsafe priority arbiter** — simultaneous RC loss + battery critical fires
   conflicting actions. Outcome is non-deterministic. Could result in landing when RTL
   was correct, or vice versa.

7. **Loiter doesn't hold entry position** — entering Loiter while vehicle has nonzero
   velocity from a previous mode means the position controller gets a moving target
   reference and the vehicle does not stop where expected.

8. **MAV_CMD 194 mapped as SetReverse** — any mission waypoint with `cmd=194` will
   interpret `NAV_TAKEOFF_LOCAL` as a rover reverse command. On a copter this probably
   does nothing. On a rover executing a mission, this will reverse direction without warning.

9. **SmartRTL synchronous cleanup causes main-loop jitter** — `record_position_dt()` runs
   full simplify+prune synchronously. At 400Hz on an embedded target, this blocks for
   several milliseconds and can cause the attitude controller to miss ticks. Under
   aggressive maneuvering this can cause attitude estimate lag and instability.

10. **Loiter arm-check requires GPS even for non-GPS modes** — not a crash risk, but will
    cause random field arming failures that erode confidence in the system and lead pilots
    to disable checks wholesale.

---

## What needs to happen before first flight

### Hard blocks (fix before any powered test)
- Fix battery_continue failsafe logic inversion in `failsafe/src/lib.rs`
- Add land-detector angle-input check to `update_land()` (at minimum: if `|rc_roll| > 0.1
  || |rc_pitch| > 0.1`, reset land timer)
- Fix RTL Land phase to use land detector, not raw baro threshold
- Replace Throw mode accel detection with actual IMU accel input field in `ModeInput`
- Verify `Quaternion::from_euler()` convention is intrinsic ZYX

### Required before GPS/autonomous flight
- Write failsafe priority arbiter before arming in any autonomous mode
- Resolve BehaviorTree vs WaypointNav ownership for Auto mode
- Fix Loiter entry position capture in `enter()`
- Fix `ActionKind::RTL` never completing in `eval_action()`
- Gate GPS arm check on mode's `requires_gps()` result

### Should fix before any operational use
- Add 10-second sustain timer for IMU consistency check
- Add altitude-loss-at-high-throttle thrust loss check to `MotorMonitor`
- Implement yaw imbalance monitor
- Fix SmartRTL synchronous cleanup (budget limit or background task)
- Add rally point support to RTL

---

## Final summary

Meridian is better than most student projects I review and worse than most production
firmware I'd fly. The Stabilize and AltHold cores are structurally correct. The RTL state
machine is architecturally sound. The arming check coverage is comprehensive. The behavior
tree mission engine is a genuine contribution.

What it lacks is the operational hardening that comes from debugging crash logs. The land
detector is the most dangerous gap — this is the code that runs at the most critical moment
of every flight and it is missing half its safety conditions. The battery failsafe logic
inversion is a field emergency waiting to happen. The Throw mode detection will injure
someone.

None of those are hard to fix. But they have to be fixed before motors spin.

**Modes safe to fly:** Brake, Acro (if attitude controller verified)
**Modes flyable with fixes:** Stabilize, AltHold, Loiter (after entry-position fix), RTL (after land fix)
**Modes that need significant work:** Auto, Land, SmartRTL, PosHold
**Modes that must not be assigned to switches:** Throw, Follow, Autotune, Turtle, FlowHold, SystemId

I would not take this to a field test without the five hard-block items above resolved.
With those fixed and an attitude-controller review, the Stabilize → AltHold → Loiter →
RTL chain is achievable on a calm day over a flat field.

---

*Randy Mackay — ArduCopter Lead Developer*
*"The land detector is the last line of defense before a broken airframe."*
