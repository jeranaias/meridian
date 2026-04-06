#![no_std]

//! LED + buzzer notification subsystem for Meridian.
//!
//! Priority-based pattern selection: failsafe > armed state > GPS > info.
//! Drives WS2812 / NeoPixel LEDs and passive buzzers via HAL GPIO/PWM.
//!
//! Source: ArduPilot `libraries/AP_Notify/`

use heapless::Vec;

// ─── LED Patterns ───

/// A single LED color step: (R, G, B, duration_ms).
pub type LedStep = (u8, u8, u8, u16);

/// An LED pattern — up to 10 color steps, optionally repeating.
#[derive(Debug, Clone, Copy)]
pub struct LedPattern {
    pub steps: [LedStep; 10],
    pub step_count: u8,
    pub repeat: bool,
}

impl LedPattern {
    /// Create a pattern from a slice of steps.
    pub const fn from_steps(steps: &[LedStep], repeat: bool) -> Self {
        let mut pattern = LedPattern {
            steps: [(0, 0, 0, 0); 10],
            step_count: 0,
            repeat,
        };
        let count = if steps.len() > 10 { 10 } else { steps.len() };
        let mut i = 0;
        while i < count {
            pattern.steps[i] = steps[i];
            i += 1;
        }
        pattern.step_count = count as u8;
        pattern
    }

    /// Total duration of one cycle in milliseconds.
    pub fn cycle_duration_ms(&self) -> u32 {
        let mut total: u32 = 0;
        let count = self.step_count as usize;
        let mut i = 0;
        while i < count {
            total += self.steps[i].3 as u32;
            i += 1;
        }
        total
    }

    /// Get the (R, G, B) color at a given elapsed time (ms) within the pattern.
    pub fn color_at(&self, elapsed_ms: u32) -> (u8, u8, u8) {
        if self.step_count == 0 {
            return (0, 0, 0);
        }

        let cycle = self.cycle_duration_ms();
        if cycle == 0 {
            return (0, 0, 0);
        }

        let t = if self.repeat {
            elapsed_ms % cycle
        } else if elapsed_ms >= cycle {
            // Non-repeating: hold last step's color
            let last = &self.steps[self.step_count as usize - 1];
            return (last.0, last.1, last.2);
        } else {
            elapsed_ms
        };

        let mut acc: u32 = 0;
        for i in 0..self.step_count as usize {
            acc += self.steps[i].3 as u32;
            if t < acc {
                return (self.steps[i].0, self.steps[i].1, self.steps[i].2);
            }
        }

        // Fallback: last step
        let last = &self.steps[self.step_count as usize - 1];
        (last.0, last.1, last.2)
    }
}

// ─── Predefined LED Patterns ───

/// Double-blink blue — vehicle disarmed and safe.
pub const LED_DISARMED: LedPattern = LedPattern::from_steps(&[
    (0, 0, 255, 100),   // blue on
    (0, 0, 0, 100),     // off
    (0, 0, 255, 100),   // blue on
    (0, 0, 0, 700),     // off (long pause)
], true);

/// Solid green — vehicle armed.
pub const LED_ARMED: LedPattern = LedPattern::from_steps(&[
    (0, 255, 0, 1000),  // solid green
], true);

/// Rapid red blink — failsafe active.
pub const LED_FAILSAFE: LedPattern = LedPattern::from_steps(&[
    (255, 0, 0, 100),   // red on
    (0, 0, 0, 100),     // off
], true);

/// Slow yellow pulse — GPS searching.
pub const LED_GPS_SEARCHING: LedPattern = LedPattern::from_steps(&[
    (255, 200, 0, 500), // yellow on
    (0, 0, 0, 500),     // off
], true);

/// Pulse green — GPS locked.
pub const LED_GPS_LOCKED: LedPattern = LedPattern::from_steps(&[
    (0, 255, 0, 300),   // green on
    (0, 80, 0, 300),    // dim green
    (0, 255, 0, 300),   // green on
    (0, 0, 0, 600),     // off
], true);

/// Orange blink — low battery warning.
pub const LED_LOW_BATTERY: LedPattern = LedPattern::from_steps(&[
    (255, 100, 0, 200), // orange on
    (0, 0, 0, 200),     // off
    (255, 100, 0, 200), // orange on
    (0, 0, 0, 400),     // off (pause)
], true);

/// Alternating red/blue — initializing.
pub const LED_INITIALIZING: LedPattern = LedPattern::from_steps(&[
    (255, 0, 0, 500),   // red
    (0, 0, 255, 500),   // blue
], true);

/// ESC calibration / save trim pattern.
pub const LED_ESC_CAL: LedPattern = LedPattern::from_steps(&[
    (255, 0, 255, 200), // magenta
    (0, 0, 0, 200),     // off
], true);

/// EKF failsafe — failsafe + red.
pub const LED_EKF_FAILSAFE: LedPattern = LedPattern::from_steps(&[
    (255, 0, 0, 100),   // red on
    (0, 0, 0, 100),     // off
    (255, 0, 0, 100),   // red on
    (255, 80, 0, 200),  // orange accent
    (0, 0, 0, 500),     // off pause
], true);

/// Pre-arm check failing — double yellow blink.
pub const LED_PREARM_FAIL: LedPattern = LedPattern::from_steps(&[
    (255, 255, 0, 100), // yellow on
    (0, 0, 0, 100),     // off
    (255, 255, 0, 100), // yellow on
    (0, 0, 0, 700),     // off pause
], true);

/// Armed with no GPS — solid blue.
pub const LED_ARMED_NO_GPS: LedPattern = LedPattern::from_steps(&[
    (0, 0, 255, 1000),  // solid blue
], true);

/// Rainbow cycle — calibrating sensors.
pub const LED_CALIBRATING: LedPattern = LedPattern::from_steps(&[
    (255, 0, 0, 150),   // red
    (255, 127, 0, 150), // orange
    (255, 255, 0, 150), // yellow
    (0, 255, 0, 150),   // green
    (0, 0, 255, 150),   // blue
    (75, 0, 130, 150),  // indigo
    (148, 0, 211, 150), // violet
], true);

// ─── Buzzer Tunes ───

/// A note: (frequency_hz, duration_ms). frequency_hz = 0 means silence (rest).
pub type Note = (u16, u16);

/// A buzzer tune — up to 32 notes.
#[derive(Debug, Clone)]
pub struct BuzzerTune {
    pub notes: Vec<Note, 32>,
}

impl BuzzerTune {
    /// Create a tune from a slice of notes.
    pub fn from_notes(notes: &[Note]) -> Self {
        let mut tune = BuzzerTune {
            notes: Vec::new(),
        };
        for &n in notes {
            let _ = tune.notes.push(n);
        }
        tune
    }

    /// Total duration of the tune in milliseconds.
    pub fn duration_ms(&self) -> u32 {
        self.notes.iter().map(|n| n.1 as u32).sum()
    }

    /// Get the note playing at a given elapsed time (ms).
    /// Returns (frequency_hz, true) if a note is active, or (0, false) if the tune is finished.
    pub fn note_at(&self, elapsed_ms: u32) -> (u16, bool) {
        let mut acc: u32 = 0;
        for &(freq, dur) in self.notes.iter() {
            acc += dur as u32;
            if elapsed_ms < acc {
                return (freq, true);
            }
        }
        (0, false)
    }
}

// ─── Predefined Buzzer Tunes ───

/// Short rising tone — arm confirmation.
pub fn tune_arm() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (1000, 100),
        (1500, 100),
        (2000, 150),
    ])
}

/// Short falling tone — disarm confirmation.
pub fn tune_disarm() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (2000, 100),
        (1500, 100),
        (1000, 150),
    ])
}

/// Double chirp — GPS fix acquired.
pub fn tune_gps_fix() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (2500, 80),
        (0, 50),
        (2500, 80),
    ])
}

/// Insistent low beeping — low battery warning.
pub fn tune_low_battery() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (800, 200),
        (0, 100),
        (800, 200),
        (0, 100),
        (800, 200),
        (0, 500),
    ])
}

/// Urgent high-low siren — failsafe warning.
pub fn tune_failsafe() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (2500, 150),
        (1200, 150),
        (2500, 150),
        (1200, 150),
        (2500, 150),
        (0, 300),
    ])
}

/// Happy melody — calibration complete.
pub fn tune_calibration_complete() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (1047, 100), // C5
        (1319, 100), // E5
        (1568, 100), // G5
        (2093, 200), // C6 (hold)
    ])
}

// ─── Additional Buzzer Tunes ───

/// Startup melody.
pub fn tune_startup() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (523, 100), (659, 100), (784, 100), (1047, 200),
    ])
}

/// Waypoint complete.
pub fn tune_waypoint_complete() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1500, 80), (0, 40), (1500, 80)])
}

/// EKF alert.
pub fn tune_ekf_alert() -> BuzzerTune {
    BuzzerTune::from_notes(&[(2000, 200), (0, 100), (2000, 200), (0, 100), (2000, 200)])
}

/// Battery continuous siren.
pub fn tune_battery_critical() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (1000, 100), (600, 100), (1000, 100), (600, 100),
        (1000, 100), (600, 100), (0, 400),
    ])
}

/// Vehicle lost tone.
pub fn tune_vehicle_lost() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (2500, 300), (0, 200), (2500, 300), (0, 200),
        (2500, 300), (0, 500),
    ])
}

/// Land warning.
pub fn tune_land_warning() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1200, 150), (0, 100), (1200, 150)])
}

/// Autotune complete.
pub fn tune_autotune_complete() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (784, 100), (988, 100), (1175, 100), (1568, 200),
    ])
}

/// Autotune failed.
pub fn tune_autotune_failed() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (400, 200), (300, 200), (200, 400),
    ])
}

/// Autotune next axis.
pub fn tune_autotune_next_axis() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1000, 100), (0, 50), (1200, 100)])
}

/// Compass calibration start.
pub fn tune_compass_cal_start() -> BuzzerTune {
    BuzzerTune::from_notes(&[(800, 150), (1000, 150)])
}

/// Compass calibration saved.
pub fn tune_compass_cal_saved() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1000, 100), (1200, 100), (1500, 200)])
}

/// Compass calibration failed.
pub fn tune_compass_cal_failed() -> BuzzerTune {
    BuzzerTune::from_notes(&[(500, 200), (400, 200), (300, 300)])
}

/// Mission complete.
pub fn tune_mission_complete() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (1047, 100), (1319, 100), (1568, 100), (2093, 300),
    ])
}

/// No SD card warning.
pub fn tune_no_sd_card() -> BuzzerTune {
    BuzzerTune::from_notes(&[(400, 500), (0, 200), (400, 500)])
}

/// Shutdown tone.
pub fn tune_shutdown() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (2000, 100), (1500, 100), (1000, 100), (500, 200),
    ])
}

/// Parachute release.
pub fn tune_parachute_release() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (3000, 100), (2000, 100), (3000, 100), (2000, 100), (3000, 300),
    ])
}

/// Pre-arm check fail.
pub fn tune_prearm_fail() -> BuzzerTune {
    BuzzerTune::from_notes(&[(300, 300), (0, 100), (300, 300)])
}

/// Fence breach.
pub fn tune_fence_breach() -> BuzzerTune {
    BuzzerTune::from_notes(&[
        (2500, 100), (1500, 100), (2500, 100), (0, 200),
    ])
}

/// Mode change.
pub fn tune_mode_change() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1500, 50)])
}

/// Simple one-note tones (LOUD_1 through LOUD_7).
pub fn tune_loud(level: u8) -> BuzzerTune {
    let freq = 500 + (level as u16) * 300;
    BuzzerTune::from_notes(&[(freq, 300)])
}

/// Barometer glitch.
pub fn tune_baro_glitch() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1000, 100), (0, 50), (800, 100), (0, 50), (600, 100)])
}

/// GPS glitch.
pub fn tune_gps_glitch() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1500, 100), (0, 50), (1200, 100)])
}

/// Gripper grab.
pub fn tune_gripper_grab() -> BuzzerTune {
    BuzzerTune::from_notes(&[(800, 100), (1200, 100)])
}

/// Gripper release.
pub fn tune_gripper_release() -> BuzzerTune {
    BuzzerTune::from_notes(&[(1200, 100), (800, 100)])
}

/// Terrain warning.
pub fn tune_terrain_warning() -> BuzzerTune {
    BuzzerTune::from_notes(&[(2000, 150), (0, 100), (2000, 150), (0, 100)])
}

/// MAVLink PLAY_TUNE: custom notes from GCS.
pub fn tune_custom(notes: &[Note]) -> BuzzerTune {
    BuzzerTune::from_notes(notes)
}

// ─── WS2812 NeoPixel HAL Output Stub ───

/// WS2812 NeoPixel output interface stub.
/// Caller implements the actual SPI/DMA bit-banging timing.
pub struct NeoPixelOutput {
    /// Number of LEDs in the strip.
    pub num_leds: u8,
    /// Pending RGB data (up to 16 LEDs). Caller reads and pushes via SPI/DMA.
    pub led_data: [(u8, u8, u8); 16],
    pub data_pending: bool,
}

impl NeoPixelOutput {
    pub fn new(num_leds: u8) -> Self {
        Self {
            num_leds: num_leds.min(16),
            led_data: [(0, 0, 0); 16],
            data_pending: false,
        }
    }

    /// Set all LEDs to a single color.
    pub fn set_all(&mut self, r: u8, g: u8, b: u8) {
        for i in 0..self.num_leds as usize {
            self.led_data[i] = (r, g, b);
        }
        self.data_pending = true;
    }

    /// Set a single LED color.
    pub fn set_led(&mut self, index: u8, r: u8, g: u8, b: u8) {
        if (index as usize) < self.num_leds as usize {
            self.led_data[index as usize] = (r, g, b);
            self.data_pending = true;
        }
    }
}

// ─── I2C LED Controller Stub ───

/// I2C LED controller stub (covers ToshibaLED, NCP5623, LP5562, etc.).
pub struct I2cLedController {
    /// I2C bus index.
    pub bus: u8,
    /// I2C address.
    pub address: u8,
    /// Current RGB output.
    pub current_rgb: (u8, u8, u8),
    /// Brightness level (0-255).
    pub brightness: u8,
    /// Pending write flag.
    pub write_pending: bool,
}

impl I2cLedController {
    pub fn new(bus: u8, address: u8) -> Self {
        Self {
            bus, address,
            current_rgb: (0, 0, 0),
            brightness: 255,
            write_pending: false,
        }
    }

    pub fn set_rgb(&mut self, r: u8, g: u8, b: u8) {
        self.current_rgb = (r, g, b);
        self.write_pending = true;
    }

    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness;
        self.write_pending = true;
    }
}

// ─── DroneCAN Buzzer Stub ───

/// DroneCAN buzzer output stub.
pub struct DroneCanBuzzer {
    /// Pending frequency to broadcast (0 = silent).
    pub pending_freq_hz: u16,
    /// Pending duration (ms).
    pub pending_duration_ms: u16,
    /// Whether a CAN message needs to be sent.
    pub send_pending: bool,
}

impl DroneCanBuzzer {
    pub fn new() -> Self {
        Self {
            pending_freq_hz: 0,
            pending_duration_ms: 0,
            send_pending: false,
        }
    }

    pub fn play(&mut self, freq_hz: u16, duration_ms: u16) {
        self.pending_freq_hz = freq_hz;
        self.pending_duration_ms = duration_ms;
        self.send_pending = true;
    }
}

// ─── Notification Priority ───

/// Priority levels for notification selection.
/// Higher numeric value = higher priority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum NotifyPriority {
    /// Background info (GPS lock, etc.)
    Info = 0,
    /// GPS state changes.
    Gps = 1,
    /// Armed/disarmed state.
    ArmedState = 2,
    /// Low battery.
    LowBattery = 3,
    /// Calibrating.
    Calibrating = 4,
    /// Failsafe — highest priority.
    Failsafe = 5,
}

// ─── Notification State ───

/// Input state for the notification manager to select appropriate patterns/tunes.
#[derive(Debug, Clone, Copy)]
pub struct NotifyState {
    pub armed: bool,
    pub gps_fix: bool,
    pub failsafe_active: bool,
    pub calibrating: bool,
    pub low_battery: bool,
    pub initializing: bool,
    pub prearm_failing: bool,
    pub ekf_failsafe: bool,
    pub gps_glitching: bool,
}

impl Default for NotifyState {
    fn default() -> Self {
        Self {
            armed: false,
            gps_fix: false,
            failsafe_active: false,
            calibrating: false,
            low_battery: false,
            initializing: true,
            prearm_failing: false,
            ekf_failsafe: false,
            gps_glitching: false,
        }
    }
}

// ─── Notification Event ───

/// A queued notification event (e.g., one-shot buzzer tune on arming).
#[derive(Debug, Clone)]
pub struct NotifyEvent {
    pub priority: NotifyPriority,
    pub tune: Option<BuzzerTune>,
    pub elapsed_ms: u32,
}

// ─── Notification Manager ───

/// Notification manager — selects LED pattern and buzzer tune based on vehicle state.
///
/// Priority-based: failsafe > calibrating > low_battery > armed state > GPS > info.
/// LED patterns are continuous (looping). Buzzer tunes are one-shot events queued.
pub struct NotifyManager {
    /// Current LED pattern being displayed.
    current_pattern: LedPattern,
    current_priority: NotifyPriority,
    /// Elapsed time into current LED pattern (ms).
    led_elapsed_ms: u32,
    /// Previous state for detecting transitions.
    prev_state: NotifyState,
    /// One-shot buzzer event queue (max 4 pending).
    buzzer_queue: Vec<NotifyEvent, 4>,
    /// Currently playing buzzer tune.
    active_tune: Option<NotifyEvent>,
}

impl NotifyManager {
    pub fn new() -> Self {
        Self {
            current_pattern: LED_DISARMED,
            current_priority: NotifyPriority::Info,
            led_elapsed_ms: 0,
            prev_state: NotifyState::default(),
            buzzer_queue: Vec::new(),
            active_tune: None,
        }
    }

    /// Update notification state. Call at ~50 Hz.
    /// `dt_ms` is the time since the last call.
    ///
    /// Returns the current LED color (R, G, B) and active buzzer frequency (0 = silent).
    pub fn update(&mut self, state: &NotifyState, dt_ms: u32) -> NotifyOutput {
        // Detect state transitions and queue buzzer tunes
        self.detect_transitions(state);

        // Select LED pattern by priority
        let (pattern, priority) = select_led_pattern(state);
        if priority >= self.current_priority || priority != self.current_priority {
            if !patterns_equal(&self.current_pattern, &pattern) {
                self.current_pattern = pattern;
                self.current_priority = priority;
                self.led_elapsed_ms = 0;
            }
        }

        // Advance LED timer
        self.led_elapsed_ms += dt_ms;
        let (r, g, b) = self.current_pattern.color_at(self.led_elapsed_ms);

        // Advance buzzer
        let buzzer_freq = self.advance_buzzer(dt_ms);

        // Store previous state
        self.prev_state = *state;

        NotifyOutput {
            led_r: r,
            led_g: g,
            led_b: b,
            buzzer_freq_hz: buzzer_freq,
        }
    }

    /// Detect state transitions and queue appropriate buzzer tunes.
    fn detect_transitions(&mut self, state: &NotifyState) {
        // Armed transition
        if state.armed && !self.prev_state.armed {
            self.queue_tune(NotifyPriority::ArmedState, tune_arm());
        }

        // Disarmed transition
        if !state.armed && self.prev_state.armed {
            self.queue_tune(NotifyPriority::ArmedState, tune_disarm());
        }

        // GPS fix acquired
        if state.gps_fix && !self.prev_state.gps_fix {
            self.queue_tune(NotifyPriority::Gps, tune_gps_fix());
        }

        // Failsafe activated
        if state.failsafe_active && !self.prev_state.failsafe_active {
            self.queue_tune(NotifyPriority::Failsafe, tune_failsafe());
        }

        // Low battery started
        if state.low_battery && !self.prev_state.low_battery {
            self.queue_tune(NotifyPriority::LowBattery, tune_low_battery());
        }

        // Calibration finished
        if !state.calibrating && self.prev_state.calibrating {
            self.queue_tune(NotifyPriority::Calibrating, tune_calibration_complete());
        }
    }

    /// Queue a buzzer tune. Higher priority tunes can preempt lower ones.
    fn queue_tune(&mut self, priority: NotifyPriority, tune: BuzzerTune) {
        let event = NotifyEvent {
            priority,
            tune: Some(tune),
            elapsed_ms: 0,
        };

        // If the active tune has lower priority, preempt it
        if let Some(ref active) = self.active_tune {
            if priority > active.priority {
                self.active_tune = Some(event);
                return;
            }
        } else {
            // No active tune, play immediately
            self.active_tune = Some(event);
            return;
        }

        // Otherwise queue it (drop if full)
        let _ = self.buzzer_queue.push(event);
    }

    /// Advance the buzzer playback. Returns current frequency (0 = silent).
    fn advance_buzzer(&mut self, dt_ms: u32) -> u16 {
        if let Some(ref mut event) = self.active_tune {
            event.elapsed_ms += dt_ms;
            if let Some(ref tune) = event.tune {
                let (freq, active) = tune.note_at(event.elapsed_ms);
                if active {
                    return freq;
                }
            }
            // Tune finished — try next in queue
            self.active_tune = None;
        }

        // Pop next tune from queue
        if !self.buzzer_queue.is_empty() {
            // Find highest priority event in queue
            let mut best_idx = 0;
            for i in 1..self.buzzer_queue.len() {
                if self.buzzer_queue[i].priority > self.buzzer_queue[best_idx].priority {
                    best_idx = i;
                }
            }
            self.active_tune = Some(self.buzzer_queue.swap_remove(best_idx));
            // Start playing immediately
            if let Some(ref event) = self.active_tune {
                if let Some(ref tune) = event.tune {
                    let (freq, _) = tune.note_at(0);
                    return freq;
                }
            }
        }

        0
    }

    /// Get the current LED color without advancing state.
    pub fn current_led_color(&self) -> (u8, u8, u8) {
        self.current_pattern.color_at(self.led_elapsed_ms)
    }

    /// Check if a buzzer tune is currently playing.
    pub fn buzzer_active(&self) -> bool {
        self.active_tune.is_some()
    }

    /// Get the number of queued buzzer events.
    pub fn buzzer_queue_len(&self) -> usize {
        self.buzzer_queue.len()
    }
}

impl Default for NotifyManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Output from the notification manager each update tick.
#[derive(Debug, Clone, Copy)]
pub struct NotifyOutput {
    pub led_r: u8,
    pub led_g: u8,
    pub led_b: u8,
    pub buzzer_freq_hz: u16,
}

// ─── Pattern Selection ───

/// Select the highest-priority LED pattern for the current state.
fn select_led_pattern(state: &NotifyState) -> (LedPattern, NotifyPriority) {
    if state.ekf_failsafe {
        return (LED_EKF_FAILSAFE, NotifyPriority::Failsafe);
    }
    if state.failsafe_active {
        return (LED_FAILSAFE, NotifyPriority::Failsafe);
    }
    if state.calibrating {
        return (LED_CALIBRATING, NotifyPriority::Calibrating);
    }
    if state.low_battery {
        return (LED_LOW_BATTERY, NotifyPriority::LowBattery);
    }
    if state.initializing {
        return (LED_INITIALIZING, NotifyPriority::Info);
    }
    if state.prearm_failing {
        return (LED_PREARM_FAIL, NotifyPriority::ArmedState);
    }
    if state.armed && !state.gps_fix {
        return (LED_ARMED_NO_GPS, NotifyPriority::ArmedState);
    }
    if state.armed {
        return (LED_ARMED, NotifyPriority::ArmedState);
    }
    if state.gps_glitching {
        return (LED_GPS_SEARCHING, NotifyPriority::Gps);
    }
    if !state.gps_fix {
        return (LED_GPS_SEARCHING, NotifyPriority::Gps);
    }
    if state.gps_fix && !state.armed {
        return (LED_GPS_LOCKED, NotifyPriority::Gps);
    }
    (LED_DISARMED, NotifyPriority::Info)
}

/// Compare two patterns for equality (by steps and repeat flag).
fn patterns_equal(a: &LedPattern, b: &LedPattern) -> bool {
    if a.step_count != b.step_count || a.repeat != b.repeat {
        return false;
    }
    for i in 0..a.step_count as usize {
        if a.steps[i] != b.steps[i] {
            return false;
        }
    }
    true
}

// ─── Tests ───

#[cfg(test)]
mod tests {
    use super::*;

    // ── LedPattern tests ──

    #[test]
    fn test_led_pattern_cycle_duration() {
        assert_eq!(LED_DISARMED.cycle_duration_ms(), 1000); // 100+100+100+700
        assert_eq!(LED_ARMED.cycle_duration_ms(), 1000);
        assert_eq!(LED_FAILSAFE.cycle_duration_ms(), 200); // 100+100
    }

    #[test]
    fn test_led_pattern_color_at_start() {
        // Disarmed: starts blue
        let (r, g, b) = LED_DISARMED.color_at(0);
        assert_eq!((r, g, b), (0, 0, 255));
    }

    #[test]
    fn test_led_pattern_color_at_second_step() {
        // Disarmed: after 100ms, should be off
        let (r, g, b) = LED_DISARMED.color_at(100);
        assert_eq!((r, g, b), (0, 0, 0));
    }

    #[test]
    fn test_led_pattern_repeats() {
        // After one full cycle (1000ms), should be back to blue
        let (r, g, b) = LED_DISARMED.color_at(1000);
        assert_eq!((r, g, b), (0, 0, 255));
    }

    #[test]
    fn test_led_armed_solid_green() {
        let (r, g, b) = LED_ARMED.color_at(0);
        assert_eq!((r, g, b), (0, 255, 0));
        let (r, g, b) = LED_ARMED.color_at(500);
        assert_eq!((r, g, b), (0, 255, 0));
    }

    #[test]
    fn test_led_failsafe_rapid_red() {
        let (r, g, b) = LED_FAILSAFE.color_at(0);
        assert_eq!((r, g, b), (255, 0, 0));
        let (r, g, b) = LED_FAILSAFE.color_at(100);
        assert_eq!((r, g, b), (0, 0, 0));
        // Repeats at 200ms
        let (r, g, b) = LED_FAILSAFE.color_at(200);
        assert_eq!((r, g, b), (255, 0, 0));
    }

    #[test]
    fn test_led_gps_searching_yellow() {
        let (r, g, b) = LED_GPS_SEARCHING.color_at(0);
        assert_eq!((r, g, b), (255, 200, 0));
        let (r, g, b) = LED_GPS_SEARCHING.color_at(500);
        assert_eq!((r, g, b), (0, 0, 0));
    }

    #[test]
    fn test_led_calibrating_rainbow() {
        // First step: red
        let (r, g, b) = LED_CALIBRATING.color_at(0);
        assert_eq!((r, g, b), (255, 0, 0));
        // Third step (300ms): yellow
        let (r, g, b) = LED_CALIBRATING.color_at(300);
        assert_eq!((r, g, b), (255, 255, 0));
    }

    #[test]
    fn test_led_empty_pattern() {
        let p = LedPattern::from_steps(&[], false);
        let (r, g, b) = p.color_at(0);
        assert_eq!((r, g, b), (0, 0, 0));
    }

    // ── BuzzerTune tests ──

    #[test]
    fn test_arm_tune_duration() {
        let t = tune_arm();
        assert_eq!(t.duration_ms(), 350); // 100+100+150
    }

    #[test]
    fn test_disarm_tune_duration() {
        let t = tune_disarm();
        assert_eq!(t.duration_ms(), 350);
    }

    #[test]
    fn test_gps_fix_tune() {
        let t = tune_gps_fix();
        assert_eq!(t.duration_ms(), 210); // 80+50+80
    }

    #[test]
    fn test_tune_note_at() {
        let t = tune_arm();
        // First note: 1000 Hz
        let (freq, active) = t.note_at(0);
        assert_eq!(freq, 1000);
        assert!(active);
        // Second note: 1500 Hz (at 100ms)
        let (freq, active) = t.note_at(100);
        assert_eq!(freq, 1500);
        assert!(active);
        // Third note: 2000 Hz (at 200ms)
        let (freq, active) = t.note_at(200);
        assert_eq!(freq, 2000);
        assert!(active);
        // Past end
        let (freq, active) = t.note_at(400);
        assert_eq!(freq, 0);
        assert!(!active);
    }

    #[test]
    fn test_failsafe_tune_siren() {
        let t = tune_failsafe();
        let (freq, active) = t.note_at(0);
        assert_eq!(freq, 2500);
        assert!(active);
        let (freq, _) = t.note_at(150);
        assert_eq!(freq, 1200);
    }

    // ── Priority tests ──

    #[test]
    fn test_priority_ordering() {
        assert!(NotifyPriority::Failsafe > NotifyPriority::ArmedState);
        assert!(NotifyPriority::ArmedState > NotifyPriority::Gps);
        assert!(NotifyPriority::Gps > NotifyPriority::Info);
        assert!(NotifyPriority::LowBattery > NotifyPriority::ArmedState);
        assert!(NotifyPriority::Calibrating > NotifyPriority::LowBattery);
    }

    // ── Pattern selection tests ──

    #[test]
    fn test_select_failsafe_highest_priority() {
        let state = NotifyState {
            armed: true,
            gps_fix: true,
            failsafe_active: true,
            calibrating: false,
            low_battery: true,
            initializing: false,
            ..NotifyState::default()
        };
        let (pattern, priority) = select_led_pattern(&state);
        assert_eq!(priority, NotifyPriority::Failsafe);
        // Failsafe is rapid red
        assert_eq!(pattern.color_at(0), (255, 0, 0));
    }

    #[test]
    fn test_select_armed_over_gps() {
        let state = NotifyState {
            armed: true,
            gps_fix: true,
            initializing: false,
            ..NotifyState::default()
        };
        let (_, priority) = select_led_pattern(&state);
        assert_eq!(priority, NotifyPriority::ArmedState);
    }

    #[test]
    fn test_select_gps_searching() {
        let state = NotifyState {
            armed: false,
            gps_fix: false,
            initializing: false,
            ..NotifyState::default()
        };
        let (pattern, priority) = select_led_pattern(&state);
        assert_eq!(priority, NotifyPriority::Gps);
        // Yellow on
        assert_eq!(pattern.color_at(0), (255, 200, 0));
    }

    #[test]
    fn test_select_low_battery_over_armed() {
        let state = NotifyState {
            armed: true,
            gps_fix: true,
            low_battery: true,
            initializing: false,
            ..NotifyState::default()
        };
        let (_, priority) = select_led_pattern(&state);
        assert_eq!(priority, NotifyPriority::LowBattery);
    }

    // ── NotifyManager tests ──

    #[test]
    fn test_manager_initial_state() {
        let mgr = NotifyManager::new();
        assert!(!mgr.buzzer_active());
        assert_eq!(mgr.buzzer_queue_len(), 0);
    }

    #[test]
    fn test_manager_arm_transition_plays_tune() {
        let mut mgr = NotifyManager::new();

        // Initial disarmed state
        let state = NotifyState::default();
        mgr.update(&state, 20);

        // Arm
        let armed_state = NotifyState {
            armed: true,
            ..NotifyState::default()
        };
        let output = mgr.update(&armed_state, 20);

        // Should be playing the arm tune (1000 Hz first note)
        assert_eq!(output.buzzer_freq_hz, 1000);
        assert!(mgr.buzzer_active());
    }

    #[test]
    fn test_manager_disarm_transition() {
        let mut mgr = NotifyManager::new();

        // Set prev_state to armed by running a few ticks armed
        let armed_state = NotifyState {
            armed: true,
            ..NotifyState::default()
        };
        mgr.update(&armed_state, 20);
        // Drain the arm tune by advancing well past its duration
        mgr.update(&armed_state, 500);

        // Now disarm
        let disarmed_state = NotifyState {
            armed: false,
            ..NotifyState::default()
        };
        let output = mgr.update(&disarmed_state, 20);

        // Disarm tune: 2000 Hz first note (falling tone)
        assert_eq!(output.buzzer_freq_hz, 2000);
    }

    #[test]
    fn test_manager_gps_fix_tune() {
        let mut mgr = NotifyManager::new();

        // No GPS
        let no_gps = NotifyState {
            gps_fix: false,
            ..NotifyState::default()
        };
        mgr.update(&no_gps, 20);

        // GPS acquired
        let gps_fix = NotifyState {
            gps_fix: true,
            ..NotifyState::default()
        };
        let output = mgr.update(&gps_fix, 20);

        // GPS fix tune: 2500 Hz
        assert_eq!(output.buzzer_freq_hz, 2500);
    }

    #[test]
    fn test_manager_failsafe_led_pattern() {
        let mut mgr = NotifyManager::new();

        let state = NotifyState {
            failsafe_active: true,
            ..NotifyState::default()
        };
        let output = mgr.update(&state, 0);

        // Red LED for failsafe
        assert_eq!(output.led_r, 255);
        assert_eq!(output.led_g, 0);
        assert_eq!(output.led_b, 0);
    }

    #[test]
    fn test_manager_led_advances_over_time() {
        let mut mgr = NotifyManager::new();

        let state = NotifyState { initializing: false, ..NotifyState::default() };

        // At t=0, GPS searching (yellow on)
        let output = mgr.update(&state, 0);
        assert_eq!((output.led_r, output.led_g, output.led_b), (255, 200, 0));

        // At t=500ms, GPS searching (off phase)
        let output = mgr.update(&state, 500);
        assert_eq!((output.led_r, output.led_g, output.led_b), (0, 0, 0));

        // At t=1000ms, back to yellow
        let output = mgr.update(&state, 500);
        assert_eq!((output.led_r, output.led_g, output.led_b), (255, 200, 0));
    }

    #[test]
    fn test_manager_buzzer_finishes() {
        let mut mgr = NotifyManager::new();

        // Trigger arm tune
        let disarmed = NotifyState::default();
        mgr.update(&disarmed, 20);
        let armed = NotifyState { armed: true, ..NotifyState::default() };
        mgr.update(&armed, 20);

        // Advance past the entire arm tune (350ms)
        let output = mgr.update(&armed, 400);
        assert_eq!(output.buzzer_freq_hz, 0);
        assert!(!mgr.buzzer_active());
    }

    #[test]
    fn test_manager_failsafe_preempts_arm_tune() {
        let mut mgr = NotifyManager::new();

        // Start disarmed
        let disarmed = NotifyState::default();
        mgr.update(&disarmed, 20);

        // Arm (queues arm tune)
        let armed = NotifyState { armed: true, ..NotifyState::default() };
        mgr.update(&armed, 20);
        assert!(mgr.buzzer_active());

        // Failsafe triggers (should preempt arm tune)
        let failsafe = NotifyState {
            armed: true,
            failsafe_active: true,
            ..NotifyState::default()
        };
        let output = mgr.update(&failsafe, 20);

        // Failsafe tune: 2500 Hz
        assert_eq!(output.buzzer_freq_hz, 2500);
    }

    #[test]
    fn test_manager_calibration_complete_tune() {
        let mut mgr = NotifyManager::new();

        // Start calibrating
        let cal = NotifyState {
            calibrating: true,
            ..NotifyState::default()
        };
        mgr.update(&cal, 20);

        // Finish calibrating
        let done = NotifyState {
            calibrating: false,
            ..NotifyState::default()
        };
        let output = mgr.update(&done, 20);

        // Calibration complete tune: 1047 Hz (C5) first note
        assert_eq!(output.buzzer_freq_hz, 1047);
    }

    // ── patterns_equal tests ──

    #[test]
    fn test_patterns_equal() {
        assert!(patterns_equal(&LED_ARMED, &LED_ARMED));
        assert!(!patterns_equal(&LED_ARMED, &LED_FAILSAFE));
    }

    #[test]
    fn test_patterns_different_count() {
        assert!(!patterns_equal(&LED_DISARMED, &LED_ARMED));
    }
}
