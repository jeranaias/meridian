//! Analog battery monitor driver.
//!
//! Reads ADC voltage and current pins, applies board-specific scale factors,
//! computes consumed capacity (mAh), and exposes state to the failsafe system.
//!
//! Source: ArduPilot `libraries/AP_BattMonitor/AP_BattMonitor_Analog.cpp`
//!
//! # Usage
//!
//! ```ignore
//! let mut batt = BattMonitorAnalog::new(BattMonitorConfig {
//!     voltage_pin: 10,
//!     current_pin: 11,
//!     voltage_scale: 11.0,
//!     current_scale: 17.0,
//!     voltage_offset: 0.0,
//!     current_offset: 0.0,
//! });
//!
//! // In the 10 Hz loop:
//! batt.update(adc_voltage_v, adc_current_v, dt_s);
//! let state = batt.state();
//! ```

/// Battery monitor configuration (from board config / parameters).
#[derive(Debug, Clone, Copy)]
pub struct BattMonitorConfig {
    /// ADC channel for battery voltage.
    pub voltage_pin: u8,
    /// ADC channel for battery current.
    pub current_pin: u8,
    /// Multiplier: ADC_voltage_V * voltage_scale = battery_voltage_V.
    /// Example: MatekH743 uses 11.0 (1:10 resistor divider + ADC_VREF).
    pub voltage_scale: f32,
    /// Multiplier: ADC_voltage_V * current_scale = battery_current_A.
    /// Example: MatekH743 uses 17.0 (based on hall sensor sensitivity).
    pub current_scale: f32,
    /// Voltage offset (V) — subtracted before scaling. Usually 0.
    pub voltage_offset: f32,
    /// Current offset (V) — subtracted before scaling. Usually 0.
    pub current_offset: f32,
}

impl Default for BattMonitorConfig {
    fn default() -> Self {
        // MatekH743 defaults
        Self {
            voltage_pin: 10,
            current_pin: 11,
            voltage_scale: 11.0,
            current_scale: 17.0,
            voltage_offset: 0.0,
            current_offset: 0.0,
        }
    }
}

/// Battery state snapshot — exposed to failsafe, telemetry, and logging.
#[derive(Debug, Clone, Copy)]
pub struct BatteryState {
    /// Battery voltage (V), filtered.
    pub voltage: f32,
    /// Battery current (A), filtered. Positive = discharging.
    pub current_amps: f32,
    /// Consumed capacity since boot (mAh).
    pub consumed_mah: f32,
    /// Remaining capacity estimate (0-100%), or -1 if unknown.
    pub remaining_pct: f32,
    /// Whether readings are valid (ADC initialized, non-zero voltage seen).
    pub healthy: bool,
    /// Voltage in millivolts (for MAVLink SYS_STATUS).
    pub voltage_mv: u16,
    /// Current in centiamps (for MAVLink SYS_STATUS).
    pub current_ca: i16,
}

impl Default for BatteryState {
    fn default() -> Self {
        Self {
            voltage: 0.0,
            current_amps: 0.0,
            consumed_mah: 0.0,
            remaining_pct: -1.0,
            healthy: false,
            voltage_mv: 0,
            current_ca: 0,
        }
    }
}

/// IIR filter coefficient for voltage/current smoothing.
/// 0.1 at 10 Hz update = ~1s time constant (matches ArduPilot).
const FILTER_ALPHA: f32 = 0.1;

/// Minimum ADC voltage to consider readings valid (below this = disconnected).
const MIN_VALID_VOLTAGE: f32 = 0.5;

/// Full battery capacity parameter (mAh). 0 = capacity tracking disabled.
/// This would normally come from the parameter system (BATT_CAPACITY).
const DEFAULT_CAPACITY_MAH: f32 = 0.0;

/// Number of cell voltages for remaining% estimate.
/// Empty cell = 3.3V, full cell = 4.2V.
const CELL_VOLTAGE_EMPTY: f32 = 3.3;
const CELL_VOLTAGE_FULL: f32 = 4.2;

/// Analog battery monitor driver.
///
/// Reads raw ADC voltage values, applies scale factors from the board
/// configuration, filters readings, and integrates current to compute
/// consumed capacity (mAh).
///
/// The driver does NOT access ADC hardware directly — the platform HAL
/// provides filtered ADC voltages via `AnalogSource::voltage_average()`.
/// This driver converts those voltages to battery voltage/current using
/// the board-specific scale factors.
pub struct BattMonitorAnalog {
    config: BattMonitorConfig,
    state: BatteryState,
    /// Full battery capacity (mAh). 0 = disabled.
    capacity_mah: f32,
    /// Number of cells (auto-detected from initial voltage).
    cell_count: u8,
    /// Whether we've seen a valid voltage reading.
    initialized: bool,
    /// Raw (unfiltered) voltage for initialization.
    raw_voltage: f32,
    /// Total accumulated charge (amp-seconds) for mAh calculation.
    accumulated_as: f32,
}

impl BattMonitorAnalog {
    /// Create a new analog battery monitor with the given configuration.
    pub fn new(config: BattMonitorConfig) -> Self {
        Self {
            config,
            state: BatteryState::default(),
            capacity_mah: DEFAULT_CAPACITY_MAH,
            cell_count: 0,
            initialized: false,
            raw_voltage: 0.0,
            accumulated_as: 0.0,
        }
    }

    /// Set the battery capacity for remaining% calculation.
    /// Pass 0.0 to disable capacity-based remaining% (will use voltage curve).
    pub fn set_capacity(&mut self, capacity_mah: f32) {
        self.capacity_mah = capacity_mah;
    }

    /// Update the battery monitor with new ADC readings.
    ///
    /// `adc_voltage_v`: Raw ADC voltage for the battery voltage pin (V).
    /// `adc_current_v`: Raw ADC voltage for the battery current pin (V).
    /// `dt`: Time since last update (seconds). Typically 0.1s at 10 Hz.
    ///
    /// Call this from the 10 Hz slow loop (or faster for better integration).
    pub fn update(&mut self, adc_voltage_v: f32, adc_current_v: f32, dt: f32) {
        // Apply scale factors to convert ADC voltage to battery voltage/current.
        // Source: AP_BattMonitor_Analog.cpp read()
        let raw_batt_v = (adc_voltage_v - self.config.voltage_offset) * self.config.voltage_scale;
        let raw_batt_a = (adc_current_v - self.config.current_offset) * self.config.current_scale;

        // Clamp to valid range
        let raw_batt_v = if raw_batt_v < 0.0 { 0.0 } else { raw_batt_v };
        let raw_batt_a = if raw_batt_a < 0.0 { 0.0 } else { raw_batt_a };

        self.raw_voltage = raw_batt_v;

        if !self.initialized {
            if raw_batt_v > MIN_VALID_VOLTAGE {
                // First valid reading — initialize filter and detect cell count.
                self.state.voltage = raw_batt_v;
                self.state.current_amps = raw_batt_a;
                self.cell_count = detect_cell_count(raw_batt_v);
                self.initialized = true;
                self.state.healthy = true;
            }
            return;
        }

        // IIR low-pass filter: filtered = alpha * raw + (1-alpha) * prev
        self.state.voltage += FILTER_ALPHA * (raw_batt_v - self.state.voltage);
        self.state.current_amps += FILTER_ALPHA * (raw_batt_a - self.state.current_amps);

        // Integrate current for consumed mAh.
        // mAh = integral of current (A) over time (hours).
        // accumulated_as is in amp-seconds; convert to mAh at the end.
        if dt > 0.0 && dt < 1.0 {
            self.accumulated_as += self.state.current_amps * dt;
        }
        self.state.consumed_mah = self.accumulated_as / 3.6; // A*s -> mAh

        // Remaining capacity estimate.
        if self.capacity_mah > 0.0 {
            // Capacity-based: remaining = (capacity - consumed) / capacity * 100
            let remaining = (self.capacity_mah - self.state.consumed_mah) / self.capacity_mah * 100.0;
            self.state.remaining_pct = clamp(remaining, 0.0, 100.0);
        } else if self.cell_count > 0 {
            // Voltage-based: linear interpolation between empty and full cell voltage.
            let cell_v = self.state.voltage / self.cell_count as f32;
            let pct = (cell_v - CELL_VOLTAGE_EMPTY)
                / (CELL_VOLTAGE_FULL - CELL_VOLTAGE_EMPTY)
                * 100.0;
            self.state.remaining_pct = clamp(pct, 0.0, 100.0);
        }

        // Health check: voltage below minimum = disconnected/failed.
        self.state.healthy = self.state.voltage > MIN_VALID_VOLTAGE;

        // MAVLink-friendly integer representations.
        self.state.voltage_mv = (self.state.voltage * 1000.0) as u16;
        self.state.current_ca = (self.state.current_amps * 100.0) as i16;
    }

    /// Get the current battery state.
    pub fn state(&self) -> &BatteryState {
        &self.state
    }

    /// Get consumed capacity in mAh.
    pub fn consumed_mah(&self) -> f32 {
        self.state.consumed_mah
    }

    /// Get the detected cell count.
    pub fn cell_count(&self) -> u8 {
        self.cell_count
    }

    /// Whether the monitor has been initialized (seen valid ADC readings).
    pub fn is_healthy(&self) -> bool {
        self.state.healthy
    }

    /// Get battery voltage (V), filtered.
    pub fn voltage(&self) -> f32 {
        self.state.voltage
    }

    /// Get battery current (A), filtered.
    pub fn current_amps(&self) -> f32 {
        self.state.current_amps
    }

    /// Reset consumed capacity counter (e.g., after battery swap).
    pub fn reset_consumed(&mut self) {
        self.accumulated_as = 0.0;
        self.state.consumed_mah = 0.0;
    }
}

/// Detect the number of cells from initial voltage.
/// Assumes LiPo chemistry (3.0-4.2V per cell).
fn detect_cell_count(voltage: f32) -> u8 {
    // Max cell voltage at full charge (with some margin)
    const MAX_CELL_V: f32 = 4.35;
    if voltage < 1.0 {
        return 0;
    }
    let cells = (voltage / MAX_CELL_V) as u8 + 1;
    // Sanity clamp: 1S to 14S
    if cells > 14 { 14 } else { cells }
}

fn clamp(v: f32, min: f32, max: f32) -> f32 {
    if v < min { min } else if v > max { max } else { v }
}

// ─── Tests ───

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = BattMonitorConfig::default();
        assert_eq!(config.voltage_pin, 10);
        assert_eq!(config.current_pin, 11);
        assert!((config.voltage_scale - 11.0).abs() < 1e-6);
        assert!((config.current_scale - 17.0).abs() < 1e-6);
    }

    #[test]
    fn test_initialization() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig::default());
        assert!(!batt.is_healthy());

        // Sub-threshold voltage — should not initialize
        batt.update(0.01, 0.0, 0.1);
        assert!(!batt.is_healthy());

        // Valid voltage (1.2V ADC * 11.0 scale = 13.2V battery)
        batt.update(1.2, 0.5, 0.1);
        assert!(batt.is_healthy());
        assert!((batt.voltage() - 13.2).abs() < 0.1);
    }

    #[test]
    fn test_cell_detection() {
        assert_eq!(detect_cell_count(4.2), 1);   // 1S
        assert_eq!(detect_cell_count(8.4), 2);   // 2S
        assert_eq!(detect_cell_count(11.1), 3);  // 3S
        assert_eq!(detect_cell_count(16.8), 4);  // 4S
        assert_eq!(detect_cell_count(25.2), 6);  // 6S
        assert_eq!(detect_cell_count(0.5), 0);   // No battery
    }

    #[test]
    fn test_current_integration() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig::default());

        // Initialize with 12.6V battery, 10A draw
        // ADC voltage = 12.6 / 11.0 = 1.145V
        // ADC current = 10.0 / 17.0 = 0.588V
        let adc_v = 12.6 / 11.0;
        let adc_a = 10.0 / 17.0;
        batt.update(adc_v, adc_a, 0.1);

        // Now draw 10A for 36 seconds (= 100 mAh)
        // 360 updates at 0.1s = 36s
        for _ in 0..360 {
            batt.update(adc_v, adc_a, 0.1);
        }

        // consumed_mah should be approximately 100 mAh
        // (10A * 36s = 360 A*s = 100 mAh)
        // Allow some tolerance for filter settling
        let consumed = batt.consumed_mah();
        assert!(
            consumed > 80.0 && consumed < 120.0,
            "Expected ~100 mAh, got {} mAh", consumed
        );
    }

    #[test]
    fn test_voltage_filtering() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig::default());
        let adc_v = 12.0 / 11.0;

        // Initialize
        batt.update(adc_v, 0.0, 0.1);
        let v1 = batt.voltage();

        // Sudden voltage spike — should be filtered
        batt.update(16.0 / 11.0, 0.0, 0.1);
        let v2 = batt.voltage();

        // Filtered value should not jump to 16V immediately
        assert!(v2 < 14.0, "Filtered voltage {} should lag behind spike", v2);
        assert!(v2 > v1, "Filtered voltage should increase towards spike");
    }

    #[test]
    fn test_remaining_pct_voltage_based() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig::default());

        // 4S battery at full charge: 4 * 4.2 = 16.8V
        let adc_v = 16.8 / 11.0;
        batt.update(adc_v, 0.0, 0.1);
        // Run a few cycles to let filter settle
        for _ in 0..50 {
            batt.update(adc_v, 0.0, 0.1);
        }
        assert!(batt.state().remaining_pct > 90.0,
            "Full battery should show >90%, got {}", batt.state().remaining_pct);

        // 4S at empty: 4 * 3.3 = 13.2V
        let adc_v_empty = 13.2 / 11.0;
        for _ in 0..200 {
            batt.update(adc_v_empty, 0.0, 0.1);
        }
        assert!(batt.state().remaining_pct < 10.0,
            "Empty battery should show <10%, got {}", batt.state().remaining_pct);
    }

    #[test]
    fn test_remaining_pct_capacity_based() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig::default());
        batt.set_capacity(5000.0); // 5000 mAh

        let adc_v = 14.8 / 11.0;
        let adc_a = 20.0 / 17.0; // 20A draw

        // Initialize
        batt.update(adc_v, adc_a, 0.1);

        // Draw 20A for 90 seconds = 500 mAh = 10% of 5000
        for _ in 0..900 {
            batt.update(adc_v, adc_a, 0.1);
        }

        // Should show ~90% remaining
        let pct = batt.state().remaining_pct;
        assert!(pct > 80.0 && pct < 95.0,
            "After 500mAh of 5000mAh, expected ~90%, got {}", pct);
    }

    #[test]
    fn test_mavlink_units() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig::default());

        // 12.6V, 10A
        let adc_v = 12.6 / 11.0;
        let adc_a = 10.0 / 17.0;
        batt.update(adc_v, adc_a, 0.1);

        // Let filter settle
        for _ in 0..100 {
            batt.update(adc_v, adc_a, 0.1);
        }

        let state = batt.state();
        // voltage_mv should be ~12600
        assert!(state.voltage_mv > 12000 && state.voltage_mv < 13000,
            "voltage_mv = {}", state.voltage_mv);
        // current_ca should be ~1000 (10A * 100)
        assert!(state.current_ca > 800 && state.current_ca < 1200,
            "current_ca = {}", state.current_ca);
    }

    #[test]
    fn test_reset_consumed() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig::default());
        let adc_v = 12.6 / 11.0;
        let adc_a = 10.0 / 17.0;

        batt.update(adc_v, adc_a, 0.1);
        for _ in 0..100 {
            batt.update(adc_v, adc_a, 0.1);
        }
        assert!(batt.consumed_mah() > 0.0);

        batt.reset_consumed();
        assert!((batt.consumed_mah() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_negative_clamp() {
        let mut batt = BattMonitorAnalog::new(BattMonitorConfig {
            voltage_offset: 0.5,
            current_offset: 0.5,
            ..BattMonitorConfig::default()
        });

        // ADC reading below offset — should clamp to 0, not go negative
        batt.update(0.3, 0.2, 0.1);
        // Should initialize with 0 voltage (below min valid)
        assert!(!batt.is_healthy());
    }
}
