#![no_std]

//! WASM extension runtime API surface and applet scheduling.
//!
//! Defines the host API that WASM applets will use to interact with the
//! autopilot, plus the applet scheduling infrastructure. The actual WASM
//! interpreter/JIT runtime is deferred to a later phase — this crate
//! establishes the contract and scheduling layer.

use heapless::Vec;

/// Maximum number of loaded WASM applets.
pub const MAX_APPLETS: usize = 16;

/// Host API trait that WASM applets call into for autopilot access.
///
/// Each method maps to a host function exported to the WASM sandbox.
/// The runtime will implement this trait against the real vehicle state.
pub trait WasmHostApi {
    /// Get current position: (latitude_deg, longitude_deg, altitude_m).
    fn get_position(&self) -> (f64, f64, f32);

    /// Get current attitude: (roll_rad, pitch_rad, yaw_rad).
    fn get_attitude(&self) -> (f32, f32, f32);

    /// Get battery state: (voltage_V, current_A, remaining_pct).
    fn get_battery(&self) -> (f32, f32, u8);

    /// Directly command a servo channel (1-based) with a PWM value (typically 1000-2000).
    fn set_servo(&mut self, channel: u8, pwm: u16);

    /// Read a named parameter. Returns None if the parameter does not exist.
    fn get_param(&self, name: &str) -> Option<f32>;

    /// Write a named parameter. Returns true if the write was accepted.
    fn set_param(&mut self, name: &str, value: f32) -> bool;

    /// Send a status text message to the GCS.
    /// Severity follows MAVLink STATUSTEXT convention: 0=EMERGENCY .. 7=DEBUG.
    fn send_statustext(&mut self, severity: u8, text: &str);

    /// Get current time in milliseconds since boot.
    fn get_time_ms(&self) -> u32;

    /// Get current flight mode as a numeric ID.
    fn get_mode(&self) -> u8;

    /// Check if the vehicle is armed.
    fn is_armed(&self) -> bool;
}

/// A registered WASM applet.
#[derive(Debug, Clone)]
pub struct WasmApplet {
    /// Applet name (up to 32 characters).
    pub name: heapless::String<32>,
    /// Whether this applet is enabled.
    pub enabled: bool,
    /// Desired execution interval (milliseconds).
    pub interval_ms: u32,
    /// Timestamp of last execution (milliseconds).
    pub last_run_ms: u32,
}

impl WasmApplet {
    /// Create a new applet with the given name and interval.
    pub fn new(name: &str, interval_ms: u32) -> Self {
        let mut applet_name = heapless::String::new();
        // Truncate if name is too long
        for c in name.chars() {
            if applet_name.push(c).is_err() {
                break;
            }
        }
        Self {
            name: applet_name,
            enabled: true,
            interval_ms,
            last_run_ms: 0,
        }
    }
}

/// WASM extension runtime — manages applet lifecycle and scheduling.
///
/// The actual bytecode execution is deferred to a later phase.
/// This struct handles registration, scheduling, and enable/disable.
pub struct WasmRuntime {
    applets: Vec<WasmApplet, MAX_APPLETS>,
}

impl WasmRuntime {
    pub fn new() -> Self {
        Self {
            applets: Vec::new(),
        }
    }

    /// Number of loaded applets.
    pub fn applet_count(&self) -> usize {
        self.applets.len()
    }

    /// Load a new applet with the given name and execution interval.
    /// Returns the applet index, or None if the registry is full.
    pub fn load_applet(&mut self, name: &str, interval_ms: u32) -> Option<usize> {
        let idx = self.applets.len();
        let applet = WasmApplet::new(name, interval_ms);
        if self.applets.push(applet).is_err() {
            return None;
        }
        Some(idx)
    }

    /// Enable an applet by index.
    pub fn enable(&mut self, index: usize) {
        if let Some(applet) = self.applets.get_mut(index) {
            applet.enabled = true;
        }
    }

    /// Disable an applet by index.
    pub fn disable(&mut self, index: usize) {
        if let Some(applet) = self.applets.get_mut(index) {
            applet.enabled = false;
        }
    }

    /// Check if an applet is enabled.
    pub fn is_enabled(&self, index: usize) -> bool {
        self.applets.get(index).map_or(false, |a| a.enabled)
    }

    /// Get a reference to an applet by index.
    pub fn get_applet(&self, index: usize) -> Option<&WasmApplet> {
        self.applets.get(index)
    }

    /// Find the next applet that is due for execution.
    /// Returns the index of the due applet, or None if no applet is due.
    pub fn is_due(&self, now_ms: u32) -> Option<usize> {
        for (i, applet) in self.applets.iter().enumerate() {
            if applet.enabled && now_ms.wrapping_sub(applet.last_run_ms) >= applet.interval_ms {
                return Some(i);
            }
        }
        None
    }

    /// Run the scheduling loop: find and execute all due applets.
    ///
    /// For each due applet, calls the host API (placeholder — the real runtime
    /// would invoke the WASM bytecode here). Updates `last_run_ms` timestamps.
    pub fn update(&mut self, now_ms: u32, _host: &mut dyn WasmHostApi) {
        for applet in self.applets.iter_mut() {
            if applet.enabled && now_ms.wrapping_sub(applet.last_run_ms) >= applet.interval_ms {
                // In the real runtime, this is where we'd invoke the WASM bytecode
                // with the host API as the import object.
                applet.last_run_ms = now_ms;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock host API for testing.
    struct MockHost {
        position: (f64, f64, f32),
        attitude: (f32, f32, f32),
        armed: bool,
        mode: u8,
        time_ms: u32,
        last_servo_channel: u8,
        last_servo_pwm: u16,
        last_statustext: heapless::String<64>,
    }

    impl MockHost {
        fn new() -> Self {
            Self {
                position: (35.0, -120.0, 100.0),
                attitude: (0.0, 0.0, 0.0),
                armed: false,
                mode: 0,
                time_ms: 0,
                last_servo_channel: 0,
                last_servo_pwm: 0,
                last_statustext: heapless::String::new(),
            }
        }
    }

    impl WasmHostApi for MockHost {
        fn get_position(&self) -> (f64, f64, f32) { self.position }
        fn get_attitude(&self) -> (f32, f32, f32) { self.attitude }
        fn get_battery(&self) -> (f32, f32, u8) { (12.6, 5.0, 80) }
        fn set_servo(&mut self, channel: u8, pwm: u16) {
            self.last_servo_channel = channel;
            self.last_servo_pwm = pwm;
        }
        fn get_param(&self, name: &str) -> Option<f32> {
            if name == "TEST_PARAM" { Some(42.0) } else { None }
        }
        fn set_param(&mut self, _name: &str, _value: f32) -> bool { true }
        fn send_statustext(&mut self, _severity: u8, text: &str) {
            self.last_statustext.clear();
            for c in text.chars() {
                if self.last_statustext.push(c).is_err() { break; }
            }
        }
        fn get_time_ms(&self) -> u32 { self.time_ms }
        fn get_mode(&self) -> u8 { self.mode }
        fn is_armed(&self) -> bool { self.armed }
    }

    #[test]
    fn test_load_applet() {
        let mut runtime = WasmRuntime::new();
        let idx = runtime.load_applet("test_app", 100);
        assert_eq!(idx, Some(0));
        assert_eq!(runtime.applet_count(), 1);

        let applet = runtime.get_applet(0).unwrap();
        assert_eq!(applet.name.as_str(), "test_app");
        assert_eq!(applet.interval_ms, 100);
        assert!(applet.enabled);
    }

    #[test]
    fn test_load_multiple_applets() {
        let mut runtime = WasmRuntime::new();
        for i in 0..MAX_APPLETS {
            let idx = runtime.load_applet("app", (i as u32 + 1) * 50);
            assert_eq!(idx, Some(i));
        }
        assert_eq!(runtime.applet_count(), MAX_APPLETS);

        // Should fail when full
        assert_eq!(runtime.load_applet("overflow", 100), None);
    }

    #[test]
    fn test_enable_disable() {
        let mut runtime = WasmRuntime::new();
        runtime.load_applet("toggle_me", 100);

        assert!(runtime.is_enabled(0));

        runtime.disable(0);
        assert!(!runtime.is_enabled(0));

        runtime.enable(0);
        assert!(runtime.is_enabled(0));
    }

    #[test]
    fn test_is_due_interval() {
        let mut runtime = WasmRuntime::new();
        runtime.load_applet("fast", 50);   // 50ms interval
        runtime.load_applet("slow", 200);  // 200ms interval

        let mut host = MockHost::new();

        // At t=0, elapsed since last_run(0) is 0ms — not yet due
        assert_eq!(runtime.is_due(0), None);

        // At t=50, fast is due (50ms elapsed >= 50ms interval)
        assert_eq!(runtime.is_due(50), Some(0));

        // Run at t=50
        runtime.update(50, &mut host);

        // At t=80, neither is due (fast: 30ms < 50ms, slow: 80ms < 200ms)
        assert_eq!(runtime.is_due(80), None);

        // At t=100, fast is due again (50ms elapsed)
        assert_eq!(runtime.is_due(100), Some(0));

        // Run fast at t=100
        runtime.update(100, &mut host);

        // At t=200, both due (fast: 100ms >= 50ms, slow: 200ms >= 200ms)
        let due_200 = runtime.is_due(200);
        assert!(due_200.is_some());
    }

    #[test]
    fn test_disabled_applet_not_scheduled() {
        let mut runtime = WasmRuntime::new();
        runtime.load_applet("disabled_app", 50);
        runtime.disable(0);

        // Should not be due even after interval passes
        assert_eq!(runtime.is_due(100), None);
    }

    #[test]
    fn test_update_advances_timestamps() {
        let mut runtime = WasmRuntime::new();
        runtime.load_applet("ticker", 100);

        let mut host = MockHost::new();

        // Run at t=0
        runtime.update(0, &mut host);
        assert_eq!(runtime.get_applet(0).unwrap().last_run_ms, 0);

        // Not due at t=50
        assert_eq!(runtime.is_due(50), None);

        // Due at t=100
        assert_eq!(runtime.is_due(100), Some(0));
        runtime.update(100, &mut host);
        assert_eq!(runtime.get_applet(0).unwrap().last_run_ms, 100);

        // Due at t=200
        assert_eq!(runtime.is_due(200), Some(0));
    }

    #[test]
    fn test_host_api_mock() {
        let mut host = MockHost::new();

        assert_eq!(host.get_position(), (35.0, -120.0, 100.0));
        assert_eq!(host.get_attitude(), (0.0, 0.0, 0.0));
        assert!(!host.is_armed());
        assert_eq!(host.get_mode(), 0);

        host.set_servo(3, 1500);
        assert_eq!(host.last_servo_channel, 3);
        assert_eq!(host.last_servo_pwm, 1500);

        assert_eq!(host.get_param("TEST_PARAM"), Some(42.0));
        assert_eq!(host.get_param("NONEXISTENT"), None);

        assert!(host.set_param("FOO", 1.0));

        host.send_statustext(6, "Hello");
        assert_eq!(host.last_statustext.as_str(), "Hello");
    }

    #[test]
    fn test_long_applet_name_truncated() {
        let mut runtime = WasmRuntime::new();
        let long_name = "this_is_a_very_long_applet_name_that_exceeds_32_chars";
        runtime.load_applet(long_name, 100);

        let applet = runtime.get_applet(0).unwrap();
        assert_eq!(applet.name.len(), 32);
    }

    #[test]
    fn test_wrapping_time() {
        let mut runtime = WasmRuntime::new();
        runtime.load_applet("wrap_test", 100);

        let mut host = MockHost::new();

        // Set last run near u32::MAX
        let near_max = u32::MAX - 50;
        runtime.update(near_max, &mut host);
        assert_eq!(runtime.get_applet(0).unwrap().last_run_ms, near_max);

        // Wrapping time: u32::MAX - 50 + 100 wraps to ~49
        let wrapped = near_max.wrapping_add(100);
        assert_eq!(runtime.is_due(wrapped), Some(0));
    }
}
