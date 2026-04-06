# Panel Review — System Architecture, HAL, AHRS, Boot Sequence
## Reviewer: Andrew Tridgell (ArduPilot)
## Date: 2026-04-02

---

## Preamble

I have reviewed the full HAL trait tree (`meridian-hal`), the AHRS wrapper
(`meridian-ahrs`), the STM32H743 platform implementation
(`meridian-platform-stm32`), the SITL platform (`meridian-platform-sitl`), the
SITL main loop (`bin/meridian-sitl`), and the two identification reports.

I will be direct. I have seen autopilots crash because people were confident
about the wrong things. My job here is to find those things before the vehicle
does.

---

## 1. HAL Abstraction

**Rating: NEEDS WORK**

The HAL trait in `lib.rs` is structurally correct. The set of interfaces —
UART, SPI, I2C, GPIO, RcOutput, RcInput, Storage, Scheduler, AnalogIn, CAN,
Util — maps cleanly to ArduPilot's AP_HAL component list. The trait pattern
with associated types per platform is sound and more honest than AP_HAL's
virtual dispatch. I have no complaints about the approach.

What is missing or wrong:

**a. No PWM/servo channel abstraction at the HAL level.**

ArduPilot has `SRV_Channel` / `SRV_Channels` as a cross-cutting HAL service
that maps logical outputs (motor 1, aileron, flap, etc.) to physical pins with
per-channel trim, min, max, and reversal. Without this, every consumer that
needs a servo output has to talk to `RcOutput` directly at the raw pin level.
That is not a HAL; that is a register wrapper. The SRV_Channel layer is what
makes the same firmware binary run on a Cube Orange and a Matek H743 without
code changes. Meridian has no equivalent. Every board configuration would
require a different firmware build, which is not acceptable for a production
autopilot.

**b. The `Scheduler::create_thread` method is a lie.**

The comment says "RTIC does not support dynamic thread creation" and the
implementation is a no-op. That is fine for the RTIC target — RTIC tasks are
compile-time. But the trait itself promises `create_thread` exists, which means
any code calling it on RTIC silently does nothing. If a sensor driver or
subsystem calls `create_thread` expecting to start a background task, it will
succeed at compile time and produce a no-op at runtime. This is dangerous.
Either remove `create_thread` from the trait entirely (the RTIC model does not
need it) or make it return a `Result` with a meaningful error on targets that
cannot honour it.

**c. `delay_microseconds_boost` is silently degraded to a busy-wait.**

On ChibiOS this method temporarily raises the thread priority so the scheduler
wakes promptly after DMA completion. In the RTIC implementation it just calls
`delay_microseconds`. This is a performance regression, not a safety issue, but
it means the IMU sampling jitter is higher than ArduPilot on the same hardware,
which will corrupt EKF timing assumptions at >400 Hz. Document this explicitly
or implement it.

**d. `in_main_thread` always returns `true`.**

This is used by ArduPilot's semaphore system to enforce that certain operations
only happen in the main thread context. Returning `true` unconditionally removes
that protection. In RTIC, the correct implementation is to read the IPSR
register: `if cortex_m::register::ipsr::read().bits() == 0`. This is a
one-liner.

**e. The analog input trait has no `voltage_ratiometric` path.**

ArduPilot's `AP_HAL::AnalogSource::voltage_average_ratiometric()` returns the
voltage relative to the board supply voltage. This is needed for resistive
divider battery monitors on some boards where the ADC reference tracks Vcc.
Without it, battery voltage readings will be wrong on those boards.

**f. CAN has no DroneCAN/UAVCAN abstraction.**

The `CanIface` trait provides raw frame send/receive. That is fine. But there
is no higher-level DroneCAN service layer. ArduPilot has `AP_DroneCAN` on top
of `AP_HAL::CANIface`. Without it, GPS receivers, ESCs, airspeed sensors, and
magnetometers that use DroneCAN (which is every modern high-quality sensor)
cannot be used. The identification report lists `meridian-can` as partially
implemented. The HAL trait is fine; the gap is in the stack above it.

**g. No `semaphore()` factory on the HAL.**

ArduPilot's HAL provides `hal.util->new_semaphore()` used by every bus driver
to serialize bus access. Meridian uses `meridian_sync::RecursiveMutex` directly,
which is not wrong, but it means bus drivers import a concrete sync type rather
than going through the HAL abstraction. On SITL this works because `std::sync`
backs it. On RTIC it requires compile-time resource declarations. This is an
architectural inconsistency that will cause problems when adding new sensor
drivers.

---

## 2. AHRS Wrapper

**Rating: BLOCK**

This is the most serious structural problem in the codebase.

ArduPilot's `AP_AHRS` is 3,815 lines managing:
- Up to 3 simultaneous EKF cores with independent IMU lane assignments
- Primary lane health scoring and live lane switching
- DCM fallback when all EKF lanes fail
- External AHRS source support (VectorNav, MicroStrain, ADIS, external
  AHRS via serial)
- GPS blending across multiple GPS units before feeding the EKF
- Per-frame output extraction at IMU rate via the output predictor
- Yaw source management (magnetometer / GSF GPS-velocity-based yaw /
  external compass)
- Origin management for long-range flights (LLH → NED reference reset)
- Airspeed estimation when no airspeed sensor is fitted
- Health reporting for the GCS (`EKF_STATUS_REPORT` MAVLink message)

Meridian's `Ahrs` struct is 110 lines. It wraps a single `EkfCore` with no
fallback, no lane switching, no DCM, no origin management, and no output
predictor.

**The specific blockers:**

**a. No DCM fallback.**

`source` field can only be `AhrsSource::Ekf` or `AhrsSource::Dcm`, but the DCM
path is never entered. The EKF can diverge on the ground before GPS lock,
during aggressive maneuvers, or after a GPS glitch. When that happens there is
nothing to fall back to. In ArduPilot, DCM is what keeps the vehicle flying in
Stabilize while the EKF reinitializes. Without it, EKF failure equals crash.

**b. `gyro_corrected` returns the bias, not the corrected rate.**

```rust
pub fn gyro_corrected(&self) -> Vec3<Body> {
    // Return the last IMU gyro minus estimated bias
    // For now, just return the bias (caller subtracts from raw gyro)
    self.ekf.state.gyro_bias
}
```

The comment says "caller subtracts from raw gyro" but the method is named
`gyro_corrected`. Any caller that treats this as a corrected angular rate —
which is the entire point of calling `gyro_corrected` — will get wildly wrong
numbers. This is a latent crash. The method must return `raw_gyro - gyro_bias`
or it must be renamed and its callers audited. Neither has been done.

**c. No origin reset logic.**

For flights beyond approximately 500 m from home, the NED origin needs to be
reset to prevent floating-point precision loss in position (f32 position error
is ~1 mm per km, which is fine for most missions, but the EKF's barometric
altitude and GPS fusion assumes the origin is near the vehicle). The
identification report confirms `reset_origin` is a stub. For a SITL test over a
1 km grid this will silently introduce position drift.

**d. No output predictor connection.**

ArduPilot runs the EKF output predictor at full IMU rate (400+ Hz) to
extrapolate the delayed EKF state forward to the current time. Without this,
the attitude used by the rate controller is 10–20 ms old. At high rates and
rapid maneuvers, this phase lag degrades rate controller performance. The
identification report confirms the `OutputPredictor` struct exists but "not
fully connected to the IMU-rate loop." In SITL at 400 Hz over short flights
this may not be noticeable. On hardware at 800 Hz IMU rates it will cause
oscillation.

**e. No EKF health gate on arming.**

The SITL main loop arms unconditionally when it receives a MAVLink arm command.
There is no check that the EKF is healthy, that GPS lock is present, that
innovation variances are within bounds, or that the initial attitude alignment
has converged. ArduPilot blocks arm until EKF3 reports `Nav_ok` with defined
innovation thresholds. Meridian will arm into a diverged EKF. This is not
theoretical; I have seen this kill aircraft.

---

## 3. Boot Sequence

**Rating: NEEDS WORK**

The STM32 boot sequence, described in `rtic_app.rs` comments, is architecturally
correct. The ordering is:

1. Clock init  
2. Watchdog early check  
3. GPIO port clocks  
4. I2C bus clear  
5. Serial init  
6. ADC init  
7. Scheduler (timer config)  
8. SPI bus init / sensor probe  
9. PWM output init  
10. Flash storage load  
11. Arm production watchdog  

This matches ArduPilot's ChibiOS `AP_HAL_ChibiOS::Scheduler::start()` ordering.
The intent is sound.

**The problems:**

**a. The entire init function is a comment.**

Every hardware-interaction step is marked `// TODO: actual register access`. The
clock module (`clock.rs`) is the only section that actually calls HAL APIs
(stm32h7xx-hal). The watchdog `start()` is a no-op. The flash `load()` skips
the flash read. The timer `init()` calculates the correct PSC/ARR values in a
comment but never writes the registers. This platform does not boot on hardware.
I understand this is Phase 3 and the SITL is the proof-of-concept target, but
this distinction needs to be stated explicitly in all documentation rather than
presented as a complete platform.

**b. The SITL main loop has no pre-arm checks.**

```rust
ServerAction::Arm => {
    *armed = true;
    attitude_ctl.reset();
    rate_ctl.reset();
    ...
}
```

No checks. No EKF health. No GPS lock. No sensor sanity. ArduPilot's `AP_Arming`
has 2,000+ lines of pre-arm checks for a reason: they prevent arming into known
bad states. For SITL testing this is acceptable during development — but only if
the codebase makes it clear that production arming logic is missing. There is no
such label anywhere.

**c. The SITL loop mixes the physics gyro directly into the rate controller.**

```rust
let phys_gyro = meridian_math::Vec3::<meridian_math::frames::Body>::new(
    physics.gyro.x, physics.gyro.y, physics.gyro.z,
);
let (roll_cmd, pitch_cmd, yaw_cmd) =
    rate_ctl.update_simple(&rate_target, &phys_gyro, MAIN_DT);
```

The physics state gives perfect, noise-free, zero-latency gyro data. This
bypasses the entire IMU driver → EKF → AHRS → rate controller chain that would
run on hardware. The SITL is not testing the real code path. A flight controller
that works in SITL because it uses perfect physics-state gyro data may not work
on hardware when it uses IMU samples filtered through the EKF output predictor.
This needs to be fixed: SITL should sample the IMU through the sensor simulation
chain, and the rate controller should use `ahrs.gyro_corrected()` — once that
method is fixed — not raw physics state.

**d. The watchdog `ms_since_last_pat` calculation is wrong at boot.**

```rust
let elapsed = now.wrapping_sub(self.last_pat_cycles);
```

`last_pat_cycles` is initialized to 0. Before the first pat, `now` is typically
some small positive value after boot. After ~10.7 seconds (DWT wrap), if no pats
have occurred, `now` wraps and the subtraction produces a very large number
that may falsely indicate a stall. The fix is to initialize `last_pat_cycles` to
`clock::read_cycle_count()` at watchdog creation, not to zero.

---

## 4. Threading / Priority Model

**Rating: SHIP IT** (with caveats)

The RTIC priority mapping is well-reasoned and correctly documents the
ChibiOS-to-RTIC correspondence:

| ChibiOS Thread | RTIC Task    | Priority |
|----------------|--------------|----------|
| monitor (183)  | monitor      | 5        |
| timer (181)    | timer_1khz   | 4        |
| main (180)     | main_loop    | 3        |
| rcin (177)     | rcin         | 2        |
| io (58)        | slow_loop    | 1        |

The use of EXTI0/EXTI1/EXTI2 as software task dispatchers is the correct RTIC
idiom. The priority ceiling protocol for shared resources is theoretically sound:
RTIC's compile-time analysis makes priority inversion impossible, which is
strictly better than ChibiOS's runtime mutex system.

The DWT-based timing (`clock.rs`) is correct for STM32H743 at 400 MHz. The
overflow tracking in `Stm32Scheduler::update_overflow` correctly detects DWT
wraps at ~10.7 s intervals and assembles a 64-bit microsecond counter.

**The caveats:**

The `micros64` implementation has a subtle bug. It reconstructs total cycles as
`(overflow_count << 32) | current_cycle_count`. But if `update_overflow` misses
a wrap (e.g., if the 1 kHz timer fires late and DWT wraps twice between calls),
the overflow count will be off by one, causing a discontinuity of ~10.7 seconds
in the microsecond counter. ArduPilot handles this by also checking in the
`micros64()` call itself whether the counter appears to have gone backward. The
fix is straightforward: compare the current value against `last_cycle_count` in
`micros64()` and adjust `overflow_count` on the fly.

The memory layout documentation is accurate and important. The DTCM is not
DMA-accessible on H7 (confirmed at 0x2000_0000). The DMA safety note about
SRAM4 for bidirectional DShot is correct. This documentation alone will save
whoever writes the DMA transfer code from a day of debugging silent data
corruption.

The DMA ownership table (`dma.rs`) is sound. The `DMA_NOSHARE` concept for IMU
buses is correct — the IMU SPI streams should never be yielded to another
peripheral mid-transaction. The `is_contested` check is useful for detecting
contention at debug time.

---

## 5. What I Would Refuse to Ship

I would not let this fly on hardware today. The blocking issues, in order of
severity:

**BLOCK 1 — `gyro_corrected` returns bias instead of corrected rate.**
This is a bug that will produce wrong control outputs the moment DCM or any
caller that uses `gyro_corrected` is connected. Fix it before any hardware
flight attempt.

**BLOCK 2 — No pre-arm EKF health check.**
The arming path accepts a MAVLink ARM command unconditionally. Before hardware
flight, add minimum checks: EKF healthy flag set, GPS 3D lock (if GPS mode),
magnetometer calibrated, attitude innovation variance below threshold. These are
all accessible from `EkfCore` state today.

**BLOCK 3 — SITL uses physics-state gyro in rate controller, not EKF output.**
SITL validation is meaningless if it bypasses the actual signal chain. Fix the
SITL to route IMU samples through the sensor simulator → EKF predict → AHRS
`gyro_corrected` → rate controller. Until then, SITL test results prove nothing
about hardware behavior.

**BLOCK 4 — STM32 platform is a stub.**
Every hardware operation is commented out. This is not a platform — it is a
design document for a platform. It cannot be used to evaluate hardware behavior,
timing, or DMA correctness. This is acceptable for Phase 3 only if the
project scope says so clearly. Do not present it as a working STM32 target.

**BLOCK 5 — No DCM fallback in AHRS.**
An autopilot with a single EKF and no fallback will crash on EKF divergence.
DCM does not need to be the full ArduPilot DCM (3,000 lines); a basic
complementary filter using gyro integration + accel leveling + mag heading is
sufficient to keep the vehicle upright in Stabilize while the EKF reinitializes.

---

## 6. What Is Good

I do not spend all my time finding faults. There are things here that were done
right and should be noted.

**The HAL type system is better than AP_HAL.** Associated types per platform
give zero-cost dispatch and compile-time verification that every interface is
implemented. ArduPilot's virtual dispatch table was necessary in 2012; in 2026,
Rust's approach is strictly superior. The memory layout documentation in
`lib.rs` is exactly what every H7 firmware developer needs on day one.

**The flash storage design is correct.** The A/B sector wear leveling with
log-structured line writes, dirty-bit tracking, and single-line-per-tick flush
matches ArduPilot's `AP_FlashStorage` algorithm faithfully. The safety check
that validates page addresses before erase is good practice. The H7 errata note
about disabling the flash cache before write (commented but present) shows the
author knows the hardware.

**The RTIC priority mapping is correct and well-documented.** The comment table
mapping ChibiOS thread priorities to RTIC interrupt levels is accurate. The
choice of EXTI0/1/2 as software task dispatchers for the Matek H743 pin layout
is a legitimate board-specific decision. The use of RTIC's priority ceiling
protocol is mathematically sound and removes an entire class of priority
inversion bugs that have caused ArduPilot crashes in the past.

**The DWT overflow tracking is more thoughtful than most embedded implementations.**
Most people just use a 32-bit microsecond counter that wraps at 71 minutes.
The 64-bit extension using an overflow counter is correct (modulo the
`micros64` race described above).

**The geofence implementation is complete and tested.** The point-in-polygon
ray-casting algorithm is correct. The cylinder inclusion/exclusion logic is
correct. The unit tests cover the edge cases. This is the kind of defensive
testing a safety-critical system needs.

**The failsafe type hierarchy is well-designed.** `FailsafeReason` covers the
full set of conditions: RC loss, GNSS loss, EKF failure, battery levels,
geofence, terrain, motor failure, crash detection, thrust loss, yaw imbalance.
`FailsafeAction` includes the ArduPilot FS_OPTIONS combinations (SmartRTL→Land,
Brake→Land, DoLandStart). The enum design means exhaustive matching is enforced
at compile time. What is missing is the wiring: nothing in the codebase yet
evaluates these reasons and triggers the actions. The taxonomy is there; the
state machine is not.

---

## 7. Summary Ratings

| Area                         | Rating      |
|------------------------------|-------------|
| HAL trait coverage           | NEEDS WORK  |
| HAL trait correctness        | NEEDS WORK  |
| AHRS wrapper                 | BLOCK       |
| SITL main loop               | NEEDS WORK  |
| STM32 boot sequence (design) | NEEDS WORK  |
| STM32 platform (implementation) | BLOCK    |
| RTIC threading/priority model | SHIP IT    |
| Flash storage design         | SHIP IT     |
| DMA management               | SHIP IT     |
| Clock/timing                 | NEEDS WORK  |
| Geofence                     | SHIP IT     |
| Failsafe type system         | NEEDS WORK  |

**Overall: BLOCK — Do not fly on hardware in current state.**

The architectural direction is sound. The author understands the problem space
better than most people writing autopilot firmware from scratch. The RTIC
approach is correct. The HAL design is correct. The core issues are specific
bugs and missing connections, not fundamental design errors. With the five
blocking items fixed — `gyro_corrected`, pre-arm checks, SITL signal chain,
DCM fallback, and real hardware register writes — this becomes a credible Phase
3 flight stack worth serious flight testing.

The EKF parity gaps described in the identification report (no 3-axis mag
fusion, no GPS-velocity yaw alignment, no multi-core lane switching, single f32
precision throughout) are real but they are correctness gaps that will manifest
as degraded navigation quality, not crashes on day one. Fix the five BLOCK items
first. Then work the EKF gaps in priority order.

---

*Andrew Tridgell*  
*Canberra, 2026-04-02*
