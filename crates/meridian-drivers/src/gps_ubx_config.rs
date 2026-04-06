//! UBX configuration sequence — auto-configures u-blox GPS receivers.
//!
//! ArduPilot reference: `AP_GPS_UBLOX.cpp` config state machine.
//!
//! A factory-reset u-blox receiver defaults to NMEA at 9600 baud. This module
//! generates the UBX-CFG messages needed to configure the receiver for optimal
//! operation with Meridian (UBX binary, 115200+ baud, 10 Hz nav rate).

use crate::gps_ublox::ubx_frame;

// ---------------------------------------------------------------------------
// UBX class/ID constants
// ---------------------------------------------------------------------------

const CFG_CLASS: u8 = 0x06;
const CFG_PRT: u8 = 0x00;
const CFG_MSG: u8 = 0x01;
const CFG_NAV5: u8 = 0x24;
const CFG_SBAS: u8 = 0x16;
const CFG_GNSS: u8 = 0x3E;
const CFG_RATE: u8 = 0x08;
const CFG_CFG: u8 = 0x09;
const CFG_TP5: u8 = 0x31;

const NAV_CLASS: u8 = 0x01;
const NAV_PVT: u8 = 0x07;
const NAV_DOP: u8 = 0x04;
const NAV_SAT: u8 = 0x35;
const NAV_STATUS: u8 = 0x03;
const NAV_VELNED: u8 = 0x12;
const NAV_TIMEGPS: u8 = 0x20;

// ---------------------------------------------------------------------------
// Configuration steps
// ---------------------------------------------------------------------------

/// A single UBX-CFG frame to send.
#[derive(Debug, Clone)]
pub struct CfgStep {
    pub data: heapless::Vec<u8, 264>,
}

/// Generate the full UBX configuration sequence.
///
/// Returns up to 18 UBX frames that should be sent to the receiver in order,
/// with ~50 ms delays between each frame.
pub fn generate_config_sequence(baud_rate: u32, nav_rate_hz: u8) -> heapless::Vec<CfgStep, 20> {
    let mut steps = heapless::Vec::new();

    // Step 1: CFG-PRT — set UART1 to UBX only, desired baud rate
    let _ = steps.push(CfgStep { data: cfg_prt(baud_rate) });

    // Step 2: CFG-MSG — enable NAV-PVT at nav rate
    let _ = steps.push(CfgStep { data: cfg_msg(NAV_CLASS, NAV_PVT, 1) });

    // Step 3: CFG-MSG — enable NAV-DOP at 1/5 nav rate
    let _ = steps.push(CfgStep { data: cfg_msg(NAV_CLASS, NAV_DOP, 5) });

    // Step 4: CFG-MSG — enable NAV-SAT at 1/5 nav rate
    let _ = steps.push(CfgStep { data: cfg_msg(NAV_CLASS, NAV_SAT, 5) });

    // Step 5: CFG-MSG — disable NMEA GGA
    let _ = steps.push(CfgStep { data: cfg_msg(0xF0, 0x00, 0) });

    // Step 6: CFG-MSG — disable NMEA GLL
    let _ = steps.push(CfgStep { data: cfg_msg(0xF0, 0x01, 0) });

    // Step 7: CFG-MSG — disable NMEA GSA
    let _ = steps.push(CfgStep { data: cfg_msg(0xF0, 0x02, 0) });

    // Step 8: CFG-MSG — disable NMEA GSV
    let _ = steps.push(CfgStep { data: cfg_msg(0xF0, 0x03, 0) });

    // Step 9: CFG-MSG — disable NMEA RMC
    let _ = steps.push(CfgStep { data: cfg_msg(0xF0, 0x04, 0) });

    // Step 10: CFG-MSG — disable NMEA VTG
    let _ = steps.push(CfgStep { data: cfg_msg(0xF0, 0x05, 0) });

    // Step 11: CFG-NAV5 — dynamic model = Airborne <4g, fix mode auto 2D/3D
    let _ = steps.push(CfgStep { data: cfg_nav5() });

    // Step 12: CFG-SBAS — SBAS enabled, auto-scan
    let _ = steps.push(CfgStep { data: cfg_sbas() });

    // Step 13: CFG-GNSS — GPS + GLONASS + Galileo + BeiDou
    let _ = steps.push(CfgStep { data: cfg_gnss() });

    // Step 14: CFG-RATE — navigation rate
    let _ = steps.push(CfgStep { data: cfg_rate(nav_rate_hz) });

    // Step 15: CFG-MSG — enable NAV-VELNED (for legacy compatibility)
    let _ = steps.push(CfgStep { data: cfg_msg(NAV_CLASS, NAV_VELNED, 1) });

    // Step 16: CFG-MSG — enable NAV-TIMEGPS (for timing)
    let _ = steps.push(CfgStep { data: cfg_msg(NAV_CLASS, NAV_TIMEGPS, 5) });

    // Step 17: CFG-MSG — enable NAV-STATUS
    let _ = steps.push(CfgStep { data: cfg_msg(NAV_CLASS, NAV_STATUS, 1) });

    // Step 18: CFG-CFG — save to flash
    let _ = steps.push(CfgStep { data: cfg_save() });

    steps
}

// ---------------------------------------------------------------------------
// WH1: UBX config sender state machine
// ---------------------------------------------------------------------------

/// WH1: State machine for sending the UBX config sequence over UART.
///
/// The caller feeds bytes from `next_frame()` into the GPS UART, then
/// calls `advance()` after a 50 ms delay to move to the next step.
#[derive(Debug, Clone)]
pub struct UbxConfigSender {
    steps: heapless::Vec<CfgStep, 20>,
    current: usize,
    /// Whether the sequence is complete.
    pub complete: bool,
    /// Timestamp (ms) of when the last frame was sent.
    pub last_send_ms: u64,
    /// Inter-frame delay (ms). Default: 50 ms.
    pub delay_ms: u64,
}

impl UbxConfigSender {
    /// Create a new sender with the default 115200 baud, 10 Hz config.
    pub fn new(baud_rate: u32, nav_rate_hz: u8) -> Self {
        Self {
            steps: generate_config_sequence(baud_rate, nav_rate_hz),
            current: 0,
            complete: false,
            last_send_ms: 0,
            delay_ms: 50,
        }
    }

    /// Get the next config frame to send. Returns `None` if the sequence
    /// is complete or waiting for the inter-frame delay.
    pub fn next_frame(&mut self, now_ms: u64) -> Option<&[u8]> {
        if self.complete || self.current >= self.steps.len() {
            self.complete = true;
            return None;
        }
        if self.current > 0 && now_ms.wrapping_sub(self.last_send_ms) < self.delay_ms {
            return None; // waiting for delay
        }
        let frame = &self.steps[self.current].data;
        self.last_send_ms = now_ms;
        self.current += 1;
        if self.current >= self.steps.len() {
            self.complete = true;
        }
        Some(frame.as_slice())
    }

    /// How many steps have been sent.
    pub fn steps_sent(&self) -> usize {
        self.current
    }

    /// Total steps in the sequence.
    pub fn total_steps(&self) -> usize {
        self.steps.len()
    }

    /// Reset to start the sequence over.
    pub fn reset(&mut self, baud_rate: u32, nav_rate_hz: u8) {
        self.steps = generate_config_sequence(baud_rate, nav_rate_hz);
        self.current = 0;
        self.complete = false;
        self.last_send_ms = 0;
    }
}

/// CFG-PRT: configure UART1 for UBX protocol at given baud rate.
fn cfg_prt(baud: u32) -> heapless::Vec<u8, 264> {
    let mut payload = [0u8; 20];
    payload[0] = 1; // portID = UART1
    // Reserved byte
    // txReady (disabled)
    // mode: 8N1 = 0x000008C0
    payload[4] = 0xC0;
    payload[5] = 0x08;
    // baudRate (LE u32)
    let baud_bytes = baud.to_le_bytes();
    payload[8..12].copy_from_slice(&baud_bytes);
    // inProtoMask = UBX only (0x0001)
    payload[12] = 0x01;
    // outProtoMask = UBX only (0x0001)
    payload[14] = 0x01;
    ubx_frame(CFG_CLASS, CFG_PRT, &payload)
}

/// CFG-MSG: set message output rate on current port.
fn cfg_msg(class: u8, id: u8, rate: u8) -> heapless::Vec<u8, 264> {
    let payload = [class, id, rate];
    ubx_frame(CFG_CLASS, CFG_MSG, &payload)
}

/// CFG-NAV5: airborne <4g dynamic model, auto 2D/3D fix.
fn cfg_nav5() -> heapless::Vec<u8, 264> {
    let mut payload = [0u8; 36];
    // mask = 0x05 (apply dyn + fixMode)
    payload[0] = 0x05;
    // dynModel = Airborne <4g (8)
    payload[2] = 8;
    // fixMode = auto 2D/3D (3)
    payload[3] = 3;
    ubx_frame(CFG_CLASS, CFG_NAV5, &payload)
}

/// CFG-SBAS: enable SBAS with auto-scan.
fn cfg_sbas() -> heapless::Vec<u8, 264> {
    let mut payload = [0u8; 8];
    payload[0] = 0x03; // mode: enabled + test
    payload[1] = 0x07; // usage: range + diffCorr + integrity
    payload[2] = 3;    // maxSBAS = 3
    // scanmode: auto (all zeros)
    ubx_frame(CFG_CLASS, CFG_SBAS, &payload)
}

/// CFG-GNSS: enable GPS + GLONASS + Galileo + BeiDou.
fn cfg_gnss() -> heapless::Vec<u8, 264> {
    // Simplified: 4 config blocks (GPS, GLONASS, Galileo, BeiDou)
    let mut payload = [0u8; 36]; // 4 + 4*8
    payload[0] = 0;  // msgVer
    payload[1] = 0;  // numTrkChHw (read-only)
    payload[2] = 32; // numTrkChUse
    payload[3] = 4;  // numConfigBlocks

    // GPS: gnssId=0, minCh=8, maxCh=16, enabled
    let off = 4;
    payload[off] = 0; // gnssId = GPS
    payload[off + 1] = 8; // resTrkCh
    payload[off + 2] = 16; // maxTrkCh
    payload[off + 4] = 0x01; // enable
    payload[off + 5] = 0x00;
    payload[off + 6] = 0x01; // sigCfgMask: L1C/A
    payload[off + 7] = 0x01;

    // GLONASS: gnssId=6, minCh=4, maxCh=8, enabled
    let off = 12;
    payload[off] = 6;
    payload[off + 1] = 4;
    payload[off + 2] = 8;
    payload[off + 4] = 0x01;
    payload[off + 6] = 0x01;
    payload[off + 7] = 0x01;

    // Galileo: gnssId=2, minCh=4, maxCh=8, enabled
    let off = 20;
    payload[off] = 2;
    payload[off + 1] = 4;
    payload[off + 2] = 8;
    payload[off + 4] = 0x01;
    payload[off + 6] = 0x01;
    payload[off + 7] = 0x01;

    // BeiDou: gnssId=3, minCh=4, maxCh=8, enabled
    let off = 28;
    payload[off] = 3;
    payload[off + 1] = 4;
    payload[off + 2] = 8;
    payload[off + 4] = 0x01;
    payload[off + 6] = 0x01;
    payload[off + 7] = 0x01;

    ubx_frame(CFG_CLASS, CFG_GNSS, &payload)
}

/// CFG-RATE: set navigation rate.
fn cfg_rate(hz: u8) -> heapless::Vec<u8, 264> {
    let meas_rate_ms = if hz > 0 { 1000u16 / hz as u16 } else { 200 };
    let mut payload = [0u8; 6];
    let rate_bytes = meas_rate_ms.to_le_bytes();
    payload[0] = rate_bytes[0];
    payload[1] = rate_bytes[1];
    payload[2] = 1; // navRate = 1 (every measurement)
    payload[3] = 0;
    payload[4] = 0; // timeRef = UTC
    payload[5] = 0;
    ubx_frame(CFG_CLASS, CFG_RATE, &payload)
}

/// CFG-CFG: save current config to flash.
fn cfg_save() -> heapless::Vec<u8, 264> {
    let mut payload = [0u8; 12];
    // clearMask = 0 (don't clear anything)
    // saveMask = all sections
    payload[4] = 0xFF;
    payload[5] = 0xFF;
    payload[6] = 0xFF;
    payload[7] = 0xFF;
    // loadMask = 0
    ubx_frame(CFG_CLASS, CFG_CFG, &payload)
}

/// Stub for RTK moving baseline support.
///
/// ArduPilot reference: `MovingBase.cpp`
/// When two u-blox receivers are used, one acts as a base station and
/// sends RTCM corrections to the rover. The rover then computes its
/// position relative to the base, and the baseline vector is used for
/// heading determination.
pub struct RtkMovingBaseline {
    /// Whether moving baseline mode is enabled.
    pub enabled: bool,
    /// Base receiver index.
    pub base_index: u8,
    /// Rover receiver index.
    pub rover_index: u8,
}

impl RtkMovingBaseline {
    pub fn new() -> Self {
        Self {
            enabled: false,
            base_index: 0,
            rover_index: 1,
        }
    }

    /// Process an RTCM3 correction frame from the base and inject it to the rover.
    ///
    /// TODO: Implement RTCM3 parsing and injection once the UART injection
    /// path is available in the GPS driver.
    pub fn inject_rtcm(&mut self, _rtcm_data: &[u8]) -> bool {
        if !self.enabled {
            return false;
        }
        // Stub: RTCM injection will be routed through the UART HAL
        false
    }
}

/// Stub for GPS-for-yaw (dual antenna heading).
///
/// ArduPilot reference: `AP_GPS_UBLOX.cpp` UBX-NAV-RELPOSNED
/// Uses two u-blox receivers in a fixed baseline configuration to
/// compute heading from the baseline vector.
pub struct GpsForYaw {
    /// Whether GPS-for-yaw is enabled.
    pub enabled: bool,
    /// Heading from baseline vector [radians].
    pub heading_rad: f32,
    /// Baseline length [meters].
    pub baseline_m: f32,
    /// Whether heading is valid.
    pub heading_valid: bool,
}

impl GpsForYaw {
    pub fn new() -> Self {
        Self {
            enabled: false,
            heading_rad: 0.0,
            baseline_m: 0.0,
            heading_valid: false,
        }
    }

    /// Process a NAV-RELPOSNED message to extract heading.
    ///
    /// TODO: Parse the full RELPOSNED fields when the UBX parser
    /// supports this message type.
    pub fn process_relposned(&mut self, _payload: &[u8]) -> bool {
        if !self.enabled {
            return false;
        }
        // Stub: will extract relPosHeading, relPosLength, and flags
        false
    }
}

// ---------------------------------------------------------------------------
// GPS Auto-Baud Scanner
// ---------------------------------------------------------------------------

/// Baud rates to try during auto-detect, matching ArduPilot's scan order.
const AUTO_BAUD_RATES: [u32; 8] = [
    9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
];

/// UBX poll message: MON-VER (class=0x0A, id=0x04, empty payload).
/// If the receiver is outputting UBX, it will respond with its version string.
const UBX_PROBE: [u8; 8] = [
    0xB5, 0x62, // sync
    0x0A, 0x04, // MON-VER
    0x00, 0x00, // length = 0
    0x0E, 0x34, // checksum
];

/// State of the GPS auto-baud scanner.
///
/// ArduPilot reference: `AP_GPS.cpp`, `detect_instance()` — rotates through
/// 8 baud rates at ~1 second intervals, sending a UBX probe at each rate.
/// When a valid UBX response is received, the baud rate is locked.
#[derive(Debug, Clone, Copy)]
pub struct GpsAutoBaud {
    /// Current index into AUTO_BAUD_RATES.
    pub baud_index: usize,
    /// Timestamp (ms) when we last switched baud rate.
    pub last_switch_ms: u64,
    /// Duration (ms) to wait at each baud rate before advancing.
    pub dwell_ms: u64,
    /// Whether scanning has started (first poll called).
    started: bool,
    /// Whether a valid baud rate has been detected and locked.
    pub locked: bool,
    /// The locked baud rate (valid only when `locked == true`).
    pub locked_baud: u32,
}

impl GpsAutoBaud {
    /// Create a new auto-baud scanner.
    ///
    /// `dwell_ms`: time to wait at each baud rate (default 1200 ms).
    pub fn new(dwell_ms: u64) -> Self {
        Self {
            baud_index: 0,
            last_switch_ms: 0,
            dwell_ms,
            started: false,
            locked: false,
            locked_baud: 0,
        }
    }

    /// Create with default 1200 ms dwell time.
    pub fn default_dwell() -> Self {
        Self::new(1200)
    }

    /// The baud rate table used for scanning.
    pub fn baud_rates() -> &'static [u32; 8] {
        &AUTO_BAUD_RATES
    }

    /// Get the current baud rate to try.
    pub fn current_baud(&self) -> u32 {
        AUTO_BAUD_RATES[self.baud_index % AUTO_BAUD_RATES.len()]
    }

    /// Get the UBX probe frame to send at the current baud rate.
    /// The caller should send these bytes on the UART after switching baud.
    pub fn probe_frame() -> &'static [u8] {
        &UBX_PROBE
    }

    /// Called periodically. Returns `Some(new_baud)` when the scanner
    /// wants to advance to the next baud rate (caller must reconfigure UART).
    /// Returns `None` if still dwelling or already locked.
    pub fn poll(&mut self, now_ms: u64) -> Option<u32> {
        if self.locked {
            return None;
        }
        if !self.started {
            // First call — start scanning
            self.started = true;
            self.last_switch_ms = now_ms;
            return Some(self.current_baud());
        }
        if now_ms.wrapping_sub(self.last_switch_ms) >= self.dwell_ms {
            // Advance to next baud rate
            self.baud_index = (self.baud_index + 1) % AUTO_BAUD_RATES.len();
            self.last_switch_ms = now_ms;
            return Some(self.current_baud());
        }
        None
    }

    /// Call this when a valid UBX response is detected at the current baud rate.
    /// Locks the scanner so it stops rotating.
    pub fn lock(&mut self) {
        self.locked = true;
        self.locked_baud = self.current_baud();
    }

    /// Reset the scanner to start over.
    pub fn reset(&mut self) {
        self.baud_index = 0;
        self.last_switch_ms = 0;
        self.started = false;
        self.locked = false;
        self.locked_baud = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_sequence_length() {
        let steps = generate_config_sequence(115200, 10);
        assert_eq!(steps.len(), 18, "Expected 18 config steps");
    }

    #[test]
    fn test_cfg_prt_baud() {
        let frame = cfg_prt(115200);
        // Verify the frame has UBX sync bytes
        assert_eq!(frame[0], 0xB5);
        assert_eq!(frame[1], 0x62);
        assert_eq!(frame[2], CFG_CLASS);
        assert_eq!(frame[3], CFG_PRT);
    }

    #[test]
    fn test_cfg_rate_10hz() {
        let frame = cfg_rate(10);
        // measRate should be 100 ms for 10 Hz
        assert_eq!(frame[6], 100); // low byte
        assert_eq!(frame[7], 0);   // high byte
    }

    #[test]
    fn test_auto_baud_scan_advances() {
        let mut scanner = GpsAutoBaud::new(100);
        // First poll: returns first baud rate
        let first = scanner.poll(0);
        assert_eq!(first, Some(9600));
        assert!(!scanner.locked);

        // Before dwell expires: returns None
        assert!(scanner.poll(50).is_none());

        // After dwell: advances
        let second = scanner.poll(101);
        assert_eq!(second, Some(19200));
    }

    #[test]
    fn test_auto_baud_lock() {
        let mut scanner = GpsAutoBaud::new(100);
        scanner.poll(0);
        scanner.lock();
        assert!(scanner.locked);
        assert_eq!(scanner.locked_baud, 9600);

        // After lock, poll returns None
        assert!(scanner.poll(200).is_none());
    }

    #[test]
    fn test_auto_baud_wraps() {
        let mut scanner = GpsAutoBaud::new(10);
        let mut time = 0u64;
        for _ in 0..8 {
            scanner.poll(time);
            time += 11;
        }
        // Should wrap back to index 0
        let baud = scanner.poll(time);
        assert_eq!(baud, Some(9600));
    }

    #[test]
    fn test_auto_baud_reset() {
        let mut scanner = GpsAutoBaud::new(100);
        scanner.poll(0);
        scanner.lock();
        scanner.reset();
        assert!(!scanner.locked);
        assert_eq!(scanner.baud_index, 0);
    }

    #[test]
    fn test_probe_frame_valid_ubx() {
        let probe = GpsAutoBaud::probe_frame();
        assert_eq!(probe[0], 0xB5);
        assert_eq!(probe[1], 0x62);
        assert_eq!(probe.len(), 8);
    }
}

// =========================================================================
// F9P CFGv2 (VALSET / VALGET) — for RTK and advanced configuration
// =========================================================================

/// UBX-CFG-VALSET layer IDs.
#[allow(dead_code)]
pub mod cfg_layer {
    pub const RAM: u8 = 0x01;
    pub const BBR: u8 = 0x02;
    pub const FLASH: u8 = 0x04;
    pub const ALL: u8 = 0x07;
}

/// UBX-CFG-VALSET command (key-value configuration for F9P and later).
///
/// Packs one or more (key, value) pairs into a single VALSET message.
/// Keys are 32-bit IDs from the u-blox configuration key database.
pub fn valset_frame(layer: u8, keys: &[(u32, u32)]) -> heapless::Vec<u8, 264> {
    // Header: version(1) + layers(1) + reserved(2) = 4 bytes.
    let mut payload = heapless::Vec::<u8, 256>::new();
    let _ = payload.push(0x01); // version 1
    let _ = payload.push(layer);
    let _ = payload.push(0x00); // reserved
    let _ = payload.push(0x00);

    for &(key, value) in keys {
        // Key is 4 bytes LE.
        for b in key.to_le_bytes() { let _ = payload.push(b); }
        // Value size depends on key type (simplified: assume U4 = 4 bytes).
        for b in value.to_le_bytes() { let _ = payload.push(b); }
    }

    ubx_frame(CFG_CLASS, 0x8A, &payload) // 0x8A = CFG-VALSET
}

/// UBX-CFG-VALGET poll (request key values from receiver).
pub fn valget_frame(layer: u8, keys: &[u32]) -> heapless::Vec<u8, 264> {
    let mut payload = heapless::Vec::<u8, 128>::new();
    let _ = payload.push(0x00); // version 0
    let _ = payload.push(layer);
    let _ = payload.push(0x00); // position (0 = first)
    let _ = payload.push(0x00);

    for &key in keys {
        for b in key.to_le_bytes() { let _ = payload.push(b); }
    }

    ubx_frame(CFG_CLASS, 0x8B, &payload) // 0x8B = CFG-VALGET
}

// =========================================================================
// Common F9P configuration keys (from u-blox F9P Integration Manual)
// =========================================================================

#[allow(dead_code)]
pub mod f9p_keys {
    // UART1 output protocol
    pub const CFG_UART1OUTPROT_UBX: u32 = 0x10740001;
    pub const CFG_UART1OUTPROT_NMEA: u32 = 0x10740002;
    pub const CFG_UART1OUTPROT_RTCM3X: u32 = 0x10740004;

    // UART1 input protocol
    pub const CFG_UART1INPROT_UBX: u32 = 0x10730001;
    pub const CFG_UART1INPROT_NMEA: u32 = 0x10730002;
    pub const CFG_UART1INPROT_RTCM3X: u32 = 0x10730004;

    // Navigation rate
    pub const CFG_RATE_MEAS: u32 = 0x30210001; // Measurement period (ms)
    pub const CFG_RATE_NAV: u32 = 0x30210002;  // Nav solution rate (cycles)

    // RTCM3 output messages (for RTK base)
    pub const CFG_MSGOUT_RTCM_1005: u32 = 0x209102BD; // Station reference
    pub const CFG_MSGOUT_RTCM_1077: u32 = 0x209102CC; // GPS MSM7
    pub const CFG_MSGOUT_RTCM_1087: u32 = 0x209102D1; // GLONASS MSM7
    pub const CFG_MSGOUT_RTCM_1097: u32 = 0x20910318; // Galileo MSM7
    pub const CFG_MSGOUT_RTCM_1127: u32 = 0x209102D6; // BeiDou MSM7
    pub const CFG_MSGOUT_RTCM_1230: u32 = 0x20910303; // GLONASS code-phase biases

    // TMODE (survey-in / fixed base)
    pub const CFG_TMODE_MODE: u32 = 0x20030001;       // 0=disabled, 1=survey-in, 2=fixed
    pub const CFG_TMODE_SVIN_MIN_DUR: u32 = 0x40030010; // Survey-in min duration (s)
    pub const CFG_TMODE_SVIN_ACC_LIMIT: u32 = 0x40030011; // Survey-in accuracy limit (0.1mm)

    // Moving baseline
    pub const CFG_NAVHPG_DGNSSMODE: u32 = 0x20140011; // 3 = RTK Fixed
}

// =========================================================================
// RTK base station configuration
// =========================================================================

/// Configure a u-blox F9P as an RTK base station (survey-in mode).
///
/// `min_duration_s`: Minimum survey-in time (seconds).
/// `accuracy_limit_mm`: Required accuracy (mm * 10, i.e. 50 = 5.0mm).
pub fn configure_rtk_base(min_duration_s: u32, accuracy_limit_mm: u32) -> heapless::Vec<u8, 264> {
    valset_frame(cfg_layer::RAM, &[
        // Enable survey-in mode.
        (f9p_keys::CFG_TMODE_MODE, 1),
        (f9p_keys::CFG_TMODE_SVIN_MIN_DUR, min_duration_s),
        (f9p_keys::CFG_TMODE_SVIN_ACC_LIMIT, accuracy_limit_mm),
        // Enable RTCM3 output on UART1.
        (f9p_keys::CFG_UART1OUTPROT_RTCM3X, 1),
        // Output RTCM messages at 1Hz.
        (f9p_keys::CFG_MSGOUT_RTCM_1005, 1),
        (f9p_keys::CFG_MSGOUT_RTCM_1077, 1),
        (f9p_keys::CFG_MSGOUT_RTCM_1087, 1),
        (f9p_keys::CFG_MSGOUT_RTCM_1097, 1),
        (f9p_keys::CFG_MSGOUT_RTCM_1127, 1),
        (f9p_keys::CFG_MSGOUT_RTCM_1230, 5), // Every 5 seconds.
    ])
}

/// Configure a u-blox F9P as an RTK rover (receives RTCM corrections).
pub fn configure_rtk_rover() -> heapless::Vec<u8, 264> {
    valset_frame(cfg_layer::RAM, &[
        // Enable RTCM3 input on UART1.
        (f9p_keys::CFG_UART1INPROT_RTCM3X, 1),
        // Set DGNSS mode to RTK Fixed.
        (f9p_keys::CFG_NAVHPG_DGNSSMODE, 3),
    ])
}

// =========================================================================
// NAV-RELPOSNED parsing (for GPS yaw from moving baseline)
// =========================================================================

/// Parsed NAV-RELPOSNED message (relative position from base to rover).
#[derive(Clone, Default)]
pub struct NavRelPosNed {
    /// Version (should be 1 for F9P).
    pub version: u8,
    /// Reference station ID.
    pub ref_station_id: u16,
    /// GPS time of week (ms).
    pub itow: u32,
    /// Relative position North (cm).
    pub rel_pos_n: i32,
    /// Relative position East (cm).
    pub rel_pos_e: i32,
    /// Relative position Down (cm).
    pub rel_pos_d: i32,
    /// Relative position length (cm).
    pub rel_pos_length: i32,
    /// Relative position heading (degrees * 1e-5).
    pub rel_pos_heading: i32,
    /// Accuracy of North (0.1mm).
    pub acc_n: u32,
    /// Accuracy of East (0.1mm).
    pub acc_e: u32,
    /// Accuracy of Down (0.1mm).
    pub acc_d: u32,
    /// Accuracy of length (0.1mm).
    pub acc_length: u32,
    /// Accuracy of heading (degrees * 1e-5).
    pub acc_heading: u32,
    /// Flags: bit 0 = gnssFixOK, bit 2 = relPosValid, bit 3 = isMoving,
    /// bit 8 = relPosHeadingValid, bit 9 = relPosNormalized.
    pub flags: u32,
}

impl NavRelPosNed {
    /// Parse from a 72-byte NAV-RELPOSNED payload (UBX msg 0x01 0x3C).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 64 { return None; }

        Some(Self {
            version: data[0],
            ref_station_id: u16::from_le_bytes([data[2], data[3]]),
            itow: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
            rel_pos_n: i32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            rel_pos_e: i32::from_le_bytes([data[12], data[13], data[14], data[15]]),
            rel_pos_d: i32::from_le_bytes([data[16], data[17], data[18], data[19]]),
            rel_pos_length: i32::from_le_bytes([data[20], data[21], data[22], data[23]]),
            rel_pos_heading: i32::from_le_bytes([data[24], data[25], data[26], data[27]]),
            acc_n: u32::from_le_bytes([data[36], data[37], data[38], data[39]]),
            acc_e: u32::from_le_bytes([data[40], data[41], data[42], data[43]]),
            acc_d: u32::from_le_bytes([data[44], data[45], data[46], data[47]]),
            acc_length: u32::from_le_bytes([data[48], data[49], data[50], data[51]]),
            acc_heading: u32::from_le_bytes([data[52], data[53], data[54], data[55]]),
            flags: u32::from_le_bytes([data[60], data[61], data[62], data[63]]),
        })
    }

    /// Whether the relative position heading is valid.
    pub fn heading_valid(&self) -> bool {
        self.flags & (1 << 8) != 0
    }

    /// Whether the GNSS fix is valid.
    pub fn fix_ok(&self) -> bool {
        self.flags & 1 != 0
    }

    /// Whether the relative position is valid.
    pub fn rel_pos_valid(&self) -> bool {
        self.flags & (1 << 2) != 0
    }

    /// Heading in degrees (0-360).
    pub fn heading_deg(&self) -> f32 {
        self.rel_pos_heading as f32 * 1e-5
    }

    /// Heading accuracy in degrees.
    pub fn heading_accuracy_deg(&self) -> f32 {
        self.acc_heading as f32 * 1e-5
    }

    /// Baseline length in meters.
    pub fn baseline_length_m(&self) -> f32 {
        self.rel_pos_length as f32 / 100.0
    }
}

// =========================================================================
// RTCM3 injection
// =========================================================================

/// RTCM3 fragment reassembly buffer.
///
/// MAVLink GPS_RTCM_DATA messages carry RTCM3 data in 180-byte fragments.
/// This reassembles them into complete RTCM3 frames for injection into the
/// rover GPS receiver.
pub struct RtcmInjector {
    buf: [u8; 1024],
    len: usize,
    sequence_id: u8,
    fragment_count: u8,
    fragments_received: u8,
}

impl RtcmInjector {
    pub fn new() -> Self {
        Self {
            buf: [0u8; 1024],
            len: 0,
            sequence_id: 0,
            fragment_count: 0,
            fragments_received: 0,
        }
    }

    /// Add a fragment from a GPS_RTCM_DATA MAVLink message.
    ///
    /// `flags`: bit 0 = fragmented, bits 1-2 = fragment ID, bits 3-7 = sequence ID.
    /// `data`: fragment payload (up to 180 bytes).
    ///
    /// Returns the complete RTCM3 frame when all fragments received.
    pub fn add_fragment(&mut self, flags: u8, data: &[u8]) -> Option<&[u8]> {
        let fragmented = flags & 0x01 != 0;
        let frag_id = (flags >> 1) & 0x03;
        let seq_id = (flags >> 3) & 0x1F;

        if !fragmented {
            // Single fragment — return directly.
            let len = data.len().min(self.buf.len());
            self.buf[..len].copy_from_slice(&data[..len]);
            self.len = len;
            return Some(&self.buf[..self.len]);
        }

        // Multi-fragment.
        if seq_id != self.sequence_id || frag_id == 0 {
            // New sequence — reset.
            self.sequence_id = seq_id;
            self.len = 0;
            self.fragment_count = 0;
            self.fragments_received = 0;
        }

        // Append fragment data.
        let copy_len = data.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + copy_len].copy_from_slice(&data[..copy_len]);
        self.len += copy_len;
        self.fragments_received += 1;

        // Check if this is the last fragment (frag_id indicates total - 1).
        if frag_id == 0 {
            self.fragment_count = self.fragments_received;
        }

        // If we have all fragments, return the assembled frame.
        if self.fragment_count > 0 && self.fragments_received >= self.fragment_count {
            Some(&self.buf[..self.len])
        } else {
            None
        }
    }

    /// Reset the reassembly buffer.
    pub fn reset(&mut self) {
        self.len = 0;
        self.sequence_id = 0;
        self.fragment_count = 0;
        self.fragments_received = 0;
    }
}
