//! Multirotor twitch auto-tune state machine.
//!
//! Algorithm per step:
//! 1. Command a step rate input (180 deg/s for roll/pitch, 90 deg/s for yaw)
//! 2. Measure peak response and bounce-back
//! 3. Adjust gain: if bounce-back > 10% of peak, reduce gain by 25%
//! 4. If 4 consecutive passes, advance to next step
//! 5. After all 5 steps for one axis, move to next axis

use libm::fabsf;

/// Axis under test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TuneAxis {
    Roll,
    Pitch,
    Yaw,
}

/// Tuning step within a single axis. Ordered sequence:
/// RateDUp -> RateDDown -> RatePUp -> AnglePDown -> AnglePUp
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TuneStep {
    RateDUp,
    RateDDown,
    RatePUp,
    AnglePDown,
    AnglePUp,
}

/// Overall tuning state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TuneState {
    Idle,
    Testing,
    Analyzing,
    Complete,
    Failed,
}

/// Final tuning results for one axis.
#[derive(Debug, Clone, Copy)]
pub struct TuneResults {
    pub axis: TuneAxis,
    pub rate_p: f32,
    pub rate_i: f32,
    pub rate_d: f32,
    pub angle_p: f32,
}

/// Bounce-back threshold: 10% of peak.
const BOUNCE_BACK_THRESHOLD: f32 = 0.10;
/// Gain reduction factor when bounce-back exceeds threshold.
const GAIN_REDUCTION: f32 = 0.75;
/// Gain increase factor on a passing step.
const GAIN_INCREASE: f32 = 1.05;
/// Consecutive passes required to advance a step.
const REQUIRED_PASSES: u8 = 4;
/// Maximum iterations per step before declaring failure.
const MAX_ITERATIONS: u16 = 400;
/// Minimum gain floor to prevent collapsing to zero.
const MIN_GAIN: f32 = 0.001;

/// Default test rate for roll/pitch (deg/s).
const TEST_RATE_RP: f32 = 180.0;
/// Default test rate for yaw (deg/s).
const TEST_RATE_YAW: f32 = 90.0;

/// Duration of the rate command pulse (seconds).
const TWITCH_DURATION: f32 = 0.15;
/// Settling window after pulse to measure bounce-back (seconds).
const SETTLE_DURATION: f32 = 0.40;

/// PID auto-tuner using the multirotor twitch method.
///
/// Source: ArduPilot AC_AutoTune_Multi.cpp
#[derive(Debug, Clone)]
pub struct AutoTuner {
    axis: TuneAxis,
    step: TuneStep,
    state: TuneState,
    /// Test rate command magnitude (deg/s).
    test_rate: f32,
    /// Consecutive passes on the current step.
    consecutive_passes: u8,
    /// Iteration counter for the current step.
    iteration: u16,

    // Current working gains
    rate_p: f32,
    rate_i: f32,
    rate_d: f32,
    angle_p: f32,

    // Measurement state
    peak_rate: f32,
    bounce_back_rate: f32,
    twitch_timer: f32,
    phase_timer: f32,
    /// true = commanding twitch pulse, false = settling / measuring
    commanding: bool,
    /// sign of the twitch (+1 or -1), alternates
    twitch_sign: f32,
}

impl AutoTuner {
    /// Create a new idle auto-tuner.
    pub fn new() -> Self {
        Self {
            axis: TuneAxis::Roll,
            step: TuneStep::RateDUp,
            state: TuneState::Idle,
            test_rate: TEST_RATE_RP,
            consecutive_passes: 0,
            iteration: 0,
            rate_p: 0.15,
            rate_i: 0.10,
            rate_d: 0.005,
            angle_p: 4.5,
            peak_rate: 0.0,
            bounce_back_rate: 0.0,
            twitch_timer: 0.0,
            phase_timer: 0.0,
            commanding: true,
            twitch_sign: 1.0,
        }
    }

    /// Begin tuning on the given axis.
    pub fn start(&mut self, axis: TuneAxis) {
        self.axis = axis;
        self.step = TuneStep::RateDUp;
        self.state = TuneState::Testing;
        self.test_rate = match axis {
            TuneAxis::Roll | TuneAxis::Pitch => TEST_RATE_RP,
            TuneAxis::Yaw => TEST_RATE_YAW,
        };
        self.consecutive_passes = 0;
        self.iteration = 0;
        self.reset_measurement();
        self.commanding = true;
        self.twitch_sign = 1.0;
        // Keep existing gains as starting point (caller can set them before start)
    }

    /// Tick the auto-tuner. Returns `Some(rate_command)` while commanding a
    /// twitch pulse, or `None` while analyzing the response.
    ///
    /// `measured_rate`: current gyro rate on the active axis (deg/s).
    /// `dt`: timestep (seconds).
    pub fn update(&mut self, measured_rate: f32, dt: f32) -> Option<f32> {
        match self.state {
            TuneState::Idle | TuneState::Complete | TuneState::Failed => return None,
            _ => {}
        }

        if dt < 1e-6 {
            return None;
        }

        if self.commanding {
            // Phase 1: command the twitch pulse
            self.twitch_timer += dt;

            // Track peak during command phase
            let abs_rate = fabsf(measured_rate);
            if abs_rate > self.peak_rate {
                self.peak_rate = abs_rate;
            }

            if self.twitch_timer >= TWITCH_DURATION {
                // Transition to settle phase
                self.commanding = false;
                self.phase_timer = 0.0;
                self.state = TuneState::Analyzing;
                return None;
            }

            Some(self.test_rate * self.twitch_sign)
        } else {
            // Phase 2: settling — measure bounce-back
            self.phase_timer += dt;

            let abs_rate = fabsf(measured_rate);

            // Continue tracking peak (may still be rising)
            if abs_rate > self.peak_rate {
                self.peak_rate = abs_rate;
            }

            // Track bounce-back (response opposite to command direction)
            // Bounce-back is measured as the max magnitude during settle
            if abs_rate > self.bounce_back_rate {
                self.bounce_back_rate = abs_rate;
            }

            if self.phase_timer >= SETTLE_DURATION {
                // Analyze this iteration
                self.analyze_iteration();
                return None;
            }

            None
        }
    }

    /// Check if tuning is complete.
    pub fn is_complete(&self) -> bool {
        self.state == TuneState::Complete
    }

    /// Get the current tuning state.
    pub fn state(&self) -> TuneState {
        self.state
    }

    /// Get the current step.
    pub fn step(&self) -> TuneStep {
        self.step
    }

    /// Get final tuning results.
    pub fn get_results(&self) -> TuneResults {
        TuneResults {
            axis: self.axis,
            rate_p: self.rate_p,
            rate_i: self.rate_i,
            rate_d: self.rate_d,
            angle_p: self.angle_p,
        }
    }

    /// Set initial gains before starting (use current vehicle gains).
    pub fn set_initial_gains(&mut self, rate_p: f32, rate_i: f32, rate_d: f32, angle_p: f32) {
        self.rate_p = rate_p;
        self.rate_i = rate_i;
        self.rate_d = rate_d;
        self.angle_p = angle_p;
    }

    // ── Internal ──────────────────────────────────────────────────────

    fn reset_measurement(&mut self) {
        self.peak_rate = 0.0;
        self.bounce_back_rate = 0.0;
        self.twitch_timer = 0.0;
        self.phase_timer = 0.0;
    }

    fn analyze_iteration(&mut self) {
        self.iteration += 1;

        // Check for failure — too many iterations on one step
        if self.iteration >= MAX_ITERATIONS {
            self.state = TuneState::Failed;
            return;
        }

        let pass = if self.peak_rate > 0.0 {
            (self.bounce_back_rate / self.peak_rate) <= BOUNCE_BACK_THRESHOLD
        } else {
            false
        };

        if pass {
            // Good response: optionally nudge gain up slightly
            self.adjust_gain_for_step(true);
            self.consecutive_passes += 1;
        } else {
            // Too much bounce-back: reduce gain
            self.adjust_gain_for_step(false);
            self.consecutive_passes = 0;
        }

        if self.consecutive_passes >= REQUIRED_PASSES {
            // Step complete — advance
            if !self.advance_step() {
                // All steps done for this axis
                // Set I = P * 0.5 (standard ratio)
                self.rate_i = self.rate_p * 0.5;
                self.state = TuneState::Complete;
                return;
            }
        }

        // Start next twitch
        self.reset_measurement();
        self.commanding = true;
        self.twitch_sign = -self.twitch_sign; // alternate direction
        self.state = TuneState::Testing;
    }

    fn adjust_gain_for_step(&mut self, pass: bool) {
        match self.step {
            TuneStep::RateDUp => {
                if pass {
                    self.rate_d = (self.rate_d * GAIN_INCREASE).max(MIN_GAIN);
                } else {
                    self.rate_d = (self.rate_d * GAIN_REDUCTION).max(MIN_GAIN);
                }
            }
            TuneStep::RateDDown => {
                if pass {
                    // D down step: passing means current D is fine
                } else {
                    self.rate_d = (self.rate_d * GAIN_REDUCTION).max(MIN_GAIN);
                }
            }
            TuneStep::RatePUp => {
                if pass {
                    self.rate_p = (self.rate_p * GAIN_INCREASE).max(MIN_GAIN);
                } else {
                    self.rate_p = (self.rate_p * GAIN_REDUCTION).max(MIN_GAIN);
                }
            }
            TuneStep::AnglePDown => {
                if !pass {
                    self.angle_p = (self.angle_p * GAIN_REDUCTION).max(MIN_GAIN);
                }
            }
            TuneStep::AnglePUp => {
                if pass {
                    self.angle_p = (self.angle_p * GAIN_INCREASE).max(MIN_GAIN);
                } else {
                    self.angle_p = (self.angle_p * GAIN_REDUCTION).max(MIN_GAIN);
                }
            }
        }
    }

    /// Advance to the next step. Returns `true` if there is a next step,
    /// `false` if all steps are complete.
    fn advance_step(&mut self) -> bool {
        self.consecutive_passes = 0;
        self.iteration = 0;
        match self.step {
            TuneStep::RateDUp => { self.step = TuneStep::RateDDown; true }
            TuneStep::RateDDown => { self.step = TuneStep::RatePUp; true }
            TuneStep::RatePUp => { self.step = TuneStep::AnglePDown; true }
            TuneStep::AnglePDown => { self.step = TuneStep::AnglePUp; true }
            TuneStep::AnglePUp => false,
        }
    }
}

// ─── Multi-axis automation ───

/// Sequential multi-axis auto-tuner. Automatically proceeds Roll -> Pitch -> Yaw.
pub struct MultiAxisAutoTuner {
    pub tuner: AutoTuner,
    axes: [TuneAxis; 3],
    current_axis_idx: usize,
    pub all_complete: bool,
    /// GCS status messages for progress reporting.
    pub status_messages: [AutoTuneStatusMsg; 8],
    pub status_count: usize,
}

/// GCS status message for autotune progress.
#[derive(Debug, Clone, Copy)]
pub struct AutoTuneStatusMsg {
    pub axis: TuneAxis,
    pub step: TuneStep,
    pub state: TuneState,
    pub iteration: u16,
}

impl MultiAxisAutoTuner {
    pub fn new() -> Self {
        Self {
            tuner: AutoTuner::new(),
            axes: [TuneAxis::Roll, TuneAxis::Pitch, TuneAxis::Yaw],
            current_axis_idx: 0,
            all_complete: false,
            status_messages: [AutoTuneStatusMsg {
                axis: TuneAxis::Roll,
                step: TuneStep::RateDUp,
                state: TuneState::Idle,
                iteration: 0,
            }; 8],
            status_count: 0,
        }
    }

    /// Start the multi-axis tuning sequence.
    pub fn start(&mut self) {
        self.current_axis_idx = 0;
        self.all_complete = false;
        self.status_count = 0;
        self.tuner.start(self.axes[0]);
    }

    /// Tick. Returns rate command if in twitch phase.
    pub fn update(&mut self, measured_rate: f32, dt: f32) -> Option<f32> {
        if self.all_complete {
            return None;
        }

        let result = self.tuner.update(measured_rate, dt);

        // Check if current axis is complete
        if self.tuner.is_complete() {
            // Emit status message
            self.push_status();

            self.current_axis_idx += 1;
            if self.current_axis_idx < self.axes.len() {
                self.tuner.start(self.axes[self.current_axis_idx]);
            } else {
                self.all_complete = true;
            }
        } else if self.tuner.state() == TuneState::Failed {
            self.push_status();
            self.all_complete = true;
        }

        result
    }

    /// Get the current axis being tuned.
    pub fn current_axis(&self) -> TuneAxis {
        if self.current_axis_idx < self.axes.len() {
            self.axes[self.current_axis_idx]
        } else {
            TuneAxis::Yaw
        }
    }

    fn push_status(&mut self) {
        if self.status_count < self.status_messages.len() {
            self.status_messages[self.status_count] = AutoTuneStatusMsg {
                axis: self.tuner.get_results().axis,
                step: self.tuner.step(),
                state: self.tuner.state(),
                iteration: 0,
            };
            self.status_count += 1;
        }
    }
}

// ─── Helicopter frequency sweep stub ───

/// Helicopter frequency sweep auto-tune stub.
///
/// ArduPilot's `AC_AutoTune_Heli` uses sinusoidal rate injection at multiple
/// frequencies to measure the transfer function. This is a stub that defines
/// the interface — the actual sweep logic is not yet implemented.
pub struct HeliAutoTuner {
    pub state: TuneState,
    pub axis: TuneAxis,
    /// Frequency sweep range (Hz).
    pub freq_min_hz: f32,
    pub freq_max_hz: f32,
    /// Current sweep frequency.
    pub current_freq_hz: f32,
    /// Number of frequency steps.
    pub num_steps: u8,
    current_step: u8,
}

impl HeliAutoTuner {
    pub fn new() -> Self {
        Self {
            state: TuneState::Idle,
            axis: TuneAxis::Roll,
            freq_min_hz: 1.0,
            freq_max_hz: 25.0,
            current_freq_hz: 1.0,
            num_steps: 20,
            current_step: 0,
        }
    }

    /// Start helicopter frequency sweep on the given axis.
    pub fn start(&mut self, axis: TuneAxis) {
        self.axis = axis;
        self.state = TuneState::Testing;
        self.current_step = 0;
        self.current_freq_hz = self.freq_min_hz;
    }

    /// Tick the frequency sweep. Returns the sinusoidal rate command.
    /// `measured_rate`: current gyro rate on the active axis (deg/s).
    /// `dt`: timestep (seconds).
    /// `elapsed_s`: total elapsed time since sweep started.
    pub fn update(&mut self, _measured_rate: f32, _dt: f32, elapsed_s: f32) -> Option<f32> {
        if self.state != TuneState::Testing {
            return None;
        }

        // Generate sinusoidal input at current frequency
        let amplitude = 20.0; // degrees/s
        let rate_cmd = amplitude * libm::sinf(2.0 * core::f32::consts::PI * self.current_freq_hz * elapsed_s);

        // Stub: after some time per frequency, advance to next
        // Real implementation would analyze the response gain/phase
        Some(rate_cmd)
    }

    /// Check if sweep is complete.
    pub fn is_complete(&self) -> bool {
        self.state == TuneState::Complete
    }
}
