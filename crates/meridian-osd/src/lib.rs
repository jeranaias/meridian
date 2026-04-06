#![no_std]

//! On-Screen Display subsystem for Meridian.
//!
//! Supports two backends:
//! - **MAX7456** — SPI-based analog OSD chip (30x16 PAL, 30x13 NTSC)
//! - **MSP DisplayPort** — MSP protocol for DJI O3 / Walksnail digital goggles
//!
//! Source: ArduPilot `libraries/AP_OSD/`, BetaFlight OSD

use heapless::Vec;
use meridian_hal::spi::SpiDevice;
use meridian_hal::uart::UartDriver;
use meridian_types::vehicle::FlightModeId;

// ─── OSD Backend Trait ───

/// Backend abstraction for OSD rendering.
pub trait OsdBackend {
    /// Initialize the OSD hardware.
    fn init(&mut self) -> bool;

    /// Clear the entire screen buffer.
    fn clear(&mut self);

    /// Write a single character at grid position (x, y).
    fn write_char(&mut self, x: u8, y: u8, ch: u8);

    /// Write a string starting at grid position (x, y).
    fn write_string(&mut self, x: u8, y: u8, s: &str);

    /// Flush the buffer to the display.
    fn flush(&mut self);
}

// ─── MAX7456 Analog OSD Backend ───

/// Video standard selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoStandard {
    /// PAL: 30 columns x 16 rows (625-line, 50Hz)
    Pal,
    /// NTSC: 30 columns x 13 rows (525-line, 60Hz)
    Ntsc,
}

impl VideoStandard {
    pub fn columns(&self) -> u8 {
        30
    }

    pub fn rows(&self) -> u8 {
        match self {
            VideoStandard::Pal => 16,
            VideoStandard::Ntsc => 13,
        }
    }
}

// MAX7456 register addresses
const VM0: u8 = 0x00; // Video mode 0
const VM1: u8 = 0x01; // Video mode 1
const DMM: u8 = 0x04; // Display memory mode
const DMAH: u8 = 0x05; // Display memory address high
const DMAL: u8 = 0x06; // Display memory address low
const DMDI: u8 = 0x07; // Display memory data in
const OSDBL: u8 = 0x6C; // OSD black level

// VM0 register bits
const VM0_OSD_ENABLE: u8 = 0x08;
const VM0_PAL: u8 = 0x40;
const VM0_NTSC: u8 = 0x00;
const VM0_SYNC_AUTO: u8 = 0x00;

// DMM register bits
#[allow(dead_code)]
const DMM_CLEAR: u8 = 0x04;
const DMM_AUTO_INCREMENT: u8 = 0x01;

// End of auto-increment sequence
const END_STRING: u8 = 0xFF;

/// MAX7456 SPI-based analog OSD.
///
/// Character-mapped display using the MAX7456's built-in character ROM.
/// Each screen position holds an 8-bit character index.
pub struct Max7456Backend<S: SpiDevice> {
    spi: S,
    standard: VideoStandard,
    /// Shadow buffer: 30 * 16 = 480 bytes (PAL max)
    buffer: [u8; 480],
    /// Track which positions are dirty for partial updates.
    dirty: [bool; 480],
    initialized: bool,
}

impl<S: SpiDevice> Max7456Backend<S> {
    pub fn new(spi: S, standard: VideoStandard) -> Self {
        Self {
            spi,
            standard,
            buffer: [0x20; 480], // space character
            dirty: [false; 480],
            initialized: false,
        }
    }

    /// Calculate linear buffer index from (x, y) grid coordinates.
    fn grid_index(&self, x: u8, y: u8) -> Option<usize> {
        if x >= self.standard.columns() || y >= self.standard.rows() {
            return None;
        }
        Some(y as usize * self.standard.columns() as usize + x as usize)
    }

    /// Write a single register on the MAX7456.
    fn write_reg(&mut self, reg: u8, value: u8) {
        let tx = [reg, value];
        self.spi.write(&tx);
    }

    /// Read a single register from the MAX7456.
    fn read_reg(&mut self, reg: u8) -> u8 {
        let tx = [reg | 0x80, 0x00];
        let mut rx = [0u8; 2];
        self.spi.transfer(&tx, &mut rx);
        rx[1]
    }
}

impl<S: SpiDevice> OsdBackend for Max7456Backend<S> {
    fn init(&mut self) -> bool {
        // Soft reset
        self.write_reg(VM0, 0x02);

        // Wait for reset (normally ~100us, caller should ensure timing)

        // Disable OSD while configuring
        self.write_reg(VM0, 0x00);

        // Set OSD black level to auto
        let osdbl = self.read_reg(OSDBL);
        self.write_reg(OSDBL, osdbl & !0x10);

        // Configure video standard
        let vm0_val = VM0_OSD_ENABLE
            | VM0_SYNC_AUTO
            | match self.standard {
                VideoStandard::Pal => VM0_PAL,
                VideoStandard::Ntsc => VM0_NTSC,
            };
        self.write_reg(VM0, vm0_val);

        // Set VM1 defaults (background brightness, blink time)
        self.write_reg(VM1, 0x0C);

        self.initialized = true;
        true
    }

    fn clear(&mut self) {
        // Fill buffer with spaces
        for b in self.buffer.iter_mut() {
            *b = 0x20;
        }
        for d in self.dirty.iter_mut() {
            *d = true;
        }
    }

    fn write_char(&mut self, x: u8, y: u8, ch: u8) {
        if let Some(idx) = self.grid_index(x, y) {
            self.buffer[idx] = ch;
            self.dirty[idx] = true;
        }
    }

    fn write_string(&mut self, x: u8, y: u8, s: &str) {
        let cols = self.standard.columns();
        for (i, byte) in s.bytes().enumerate() {
            let cx = x + i as u8;
            if cx >= cols {
                break;
            }
            self.write_char(cx, y, byte);
        }
    }

    fn flush(&mut self) {
        if !self.initialized {
            return;
        }

        // Use auto-increment mode for runs of dirty characters
        let cols = self.standard.columns() as usize;
        let rows = self.standard.rows() as usize;
        let total = cols * rows;

        let mut i = 0;
        while i < total {
            if !self.dirty[i] {
                i += 1;
                continue;
            }

            // Found a dirty character — start auto-increment write
            let addr = i as u16;
            self.write_reg(DMAH, (addr >> 8) as u8);
            self.write_reg(DMAL, (addr & 0xFF) as u8);
            self.write_reg(DMM, DMM_AUTO_INCREMENT);

            // Write consecutive dirty chars
            while i < total && self.dirty[i] {
                self.write_reg(DMDI, self.buffer[i]);
                self.dirty[i] = false;
                i += 1;
            }

            // End auto-increment
            self.write_reg(DMDI, END_STRING);
        }
    }
}

// ─── MSP DisplayPort Backend ───

/// MSP DisplayPort command IDs.
const MSP_DISPLAYPORT: u8 = 182;

/// MSP DisplayPort sub-commands.
const MSP_DP_HEARTBEAT: u8 = 0;
#[allow(dead_code)]
const MSP_DP_RELEASE: u8 = 1;
const MSP_DP_CLEAR: u8 = 2;
const MSP_DP_WRITE_STRING: u8 = 3;
const MSP_DP_DRAW: u8 = 4;
// const MSP_DP_OPTIONS: u8 = 5; // reserved for future use

/// MSP DisplayPort grid: 30 columns x 16 rows (HD can be 50x18 or 53x20).
const MSP_DP_COLS: u8 = 30;
const MSP_DP_ROWS: u8 = 16;

/// MSP DisplayPort backend for DJI O3 / Walksnail / HDZero digital goggles.
///
/// Sends MSP frames over UART. Frame format:
/// `$M<` + direction + payload_size + cmd + payload + checksum
pub struct MspDisplayPortBackend<U: UartDriver> {
    uart: U,
    /// Shadow buffer for current frame
    buffer: [u8; 480], // 30 * 16
    dirty_rows: [bool; 16],
    initialized: bool,
}

impl<U: UartDriver> MspDisplayPortBackend<U> {
    pub fn new(uart: U) -> Self {
        Self {
            uart,
            buffer: [0x20; 480],
            dirty_rows: [false; 16],
            initialized: false,
        }
    }

    /// Send an MSP v1 frame: `$M<` + size + cmd + payload + checksum
    fn send_msp_frame(&mut self, cmd: u8, payload: &[u8]) {
        let size = payload.len() as u8;
        // Header
        self.uart.write(b"$M<");
        self.uart.write_byte(size);
        self.uart.write_byte(cmd);

        // Checksum: XOR of size, cmd, and all payload bytes
        let mut crc = size ^ cmd;
        for &b in payload {
            crc ^= b;
        }
        self.uart.write(payload);
        self.uart.write_byte(crc);
    }

    /// Send a DisplayPort sub-command with payload.
    fn send_dp_cmd(&mut self, subcmd: u8, data: &[u8]) {
        let mut payload: [u8; 64] = [0; 64];
        let len = 1 + data.len();
        if len > 64 {
            return;
        }
        payload[0] = subcmd;
        payload[1..1 + data.len()].copy_from_slice(data);
        self.send_msp_frame(MSP_DISPLAYPORT, &payload[..len]);
    }

    /// Grid index for (x, y).
    fn grid_index(x: u8, y: u8) -> Option<usize> {
        if x >= MSP_DP_COLS || y >= MSP_DP_ROWS {
            return None;
        }
        Some(y as usize * MSP_DP_COLS as usize + x as usize)
    }
}

impl<U: UartDriver> OsdBackend for MspDisplayPortBackend<U> {
    fn init(&mut self) -> bool {
        // Send heartbeat to grab display control
        self.send_dp_cmd(MSP_DP_HEARTBEAT, &[]);
        self.initialized = true;
        true
    }

    fn clear(&mut self) {
        for b in self.buffer.iter_mut() {
            *b = 0x20;
        }
        for d in self.dirty_rows.iter_mut() {
            *d = true;
        }
        // Also send the protocol-level clear
        if self.initialized {
            self.send_dp_cmd(MSP_DP_CLEAR, &[]);
        }
    }

    fn write_char(&mut self, x: u8, y: u8, ch: u8) {
        if let Some(idx) = Self::grid_index(x, y) {
            self.buffer[idx] = ch;
            self.dirty_rows[y as usize] = true;
        }
    }

    fn write_string(&mut self, x: u8, y: u8, s: &str) {
        for (i, byte) in s.bytes().enumerate() {
            let cx = x + i as u8;
            if cx >= MSP_DP_COLS {
                break;
            }
            self.write_char(cx, y, byte);
        }
    }

    fn flush(&mut self) {
        if !self.initialized {
            return;
        }

        // Send dirty rows as MSP_DP_WRITE_STRING sub-commands
        for row in 0..MSP_DP_ROWS {
            if !self.dirty_rows[row as usize] {
                continue;
            }

            let start = row as usize * MSP_DP_COLS as usize;
            let end = start + MSP_DP_COLS as usize;
            let row_data = &self.buffer[start..end];

            // Find first and last non-space character to minimize payload
            let first = row_data.iter().position(|&c| c != 0x20);
            let last = row_data.iter().rposition(|&c| c != 0x20);

            if let (Some(f), Some(l)) = (first, last) {
                // Payload: row, col, attribute, string bytes...
                let mut payload: [u8; 34] = [0; 34]; // 3 header + 30 chars max + 1 safety
                payload[0] = row;
                payload[1] = f as u8;
                payload[2] = 0; // attribute: normal
                let slice = &row_data[f..=l];
                let len = slice.len().min(30);
                payload[3..3 + len].copy_from_slice(&slice[..len]);
                self.send_dp_cmd(MSP_DP_WRITE_STRING, &payload[..3 + len]);
            }

            self.dirty_rows[row as usize] = false;
        }

        // Send draw command to present the frame
        self.send_dp_cmd(MSP_DP_DRAW, &[]);
    }
}

// ─── OSD Items ───

/// All OSD display items.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OsdItem {
    Altitude,
    AltitudeAgl,
    Airspeed,
    Groundspeed,
    BatteryVoltage,
    BatteryCurrent,
    BatteryRemaining,
    BatteryConsumed,
    GpsStatus,
    GpsSats,
    GpsHdop,
    GpsLatitude,
    GpsLongitude,
    HomeDirection,
    HomeDistance,
    Heading,
    Compass,
    Roll,
    Pitch,
    Throttle,
    FlightMode,
    ArmStatus,
    EkfStatus,
    MessageText,
    ClimbRate,
    Crosshair,
    Horizon,
    RssiValue,
    LinkQuality,
    CurrentDraw,
    Power,
    AvgCellVoltage,
    WindSpeed,
    WindDirection,
    Efficiency,
    RangefinderDistance,
    Temperature,
    FlipTime,
    EscRpm,
    EscTemperature,
    Clock,
    Callsign,
    TotalDistance,
    MaxAltitude,
    MaxSpeed,
    MaxDistance,
    FlightTime,
    MotorOutput,
    RcChannelValue,
    TerrainHeight,
    WaypointDistance,
    WaypointBearing,
    NextWaypoint,
    GpsSpeed3D,
    VerticalSpeed,
    BatteryVoltage2,
    // ── New parity items ──
    XtrackError,
    PlusCode,
    Airspeed2,
    Sidebars,
    RestingVoltage,
    AvgCellRestVolt,
    Battery2Voltage,
    Battery2Used,
    Current2,
    ClimbEfficiency,
    EscAmps,
    RotorRpm,
    VtxPower,
    FenceStatus,
    StatSummary,
    BaroTemp,
    AirspeedTemp,
    HeightAboveTerrain,
}

/// Configuration for a single OSD item placement.
#[derive(Debug, Clone, Copy)]
pub struct OsdItemConfig {
    pub item: OsdItem,
    pub x: u8,
    pub y: u8,
    pub enabled: bool,
}

// ─── OSD Screen ───

/// A single OSD screen layout — up to 64 positioned items.
pub struct OsdScreen {
    pub items: Vec<OsdItemConfig, 64>,
}

impl OsdScreen {
    pub fn new() -> Self {
        Self {
            items: Vec::new(),
        }
    }

    /// Add an item to this screen layout.
    pub fn add_item(&mut self, item: OsdItem, x: u8, y: u8) -> bool {
        self.items
            .push(OsdItemConfig {
                item,
                x,
                y,
                enabled: true,
            })
            .is_ok()
    }

    /// Enable or disable an item by type.
    pub fn set_enabled(&mut self, item: OsdItem, enabled: bool) {
        for cfg in self.items.iter_mut() {
            if cfg.item == item {
                cfg.enabled = enabled;
            }
        }
    }
}

impl Default for OsdScreen {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Vehicle State for OSD Rendering ───

/// Snapshot of vehicle state consumed by OSD rendering.
/// Decoupled from the message bus so OSD can be tested independently.
#[derive(Debug, Clone, Copy)]
pub struct OsdState {
    pub altitude_m: f32,
    pub altitude_agl_m: f32,
    pub airspeed_ms: f32,
    pub groundspeed_ms: f32,
    pub battery_voltage: f32,
    pub battery_current: f32,
    pub battery_remaining_pct: f32,
    pub battery_consumed_mah: f32,
    pub gps_fix: u8,
    pub gps_sats: u8,
    pub gps_hdop: f32,
    pub gps_lat: f64,
    pub gps_lon: f64,
    pub home_direction_deg: f32,
    pub home_distance_m: f32,
    pub heading_deg: f32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub throttle_pct: f32,
    pub flight_mode: FlightModeId,
    pub armed: bool,
    pub ekf_healthy: bool,
    pub message_text: [u8; 50],
    pub message_len: u8,
    pub climb_rate_ms: f32,
    pub rssi: u8,
    pub link_quality: u8,
    pub power_w: f32,
    pub avg_cell_voltage: f32,
    pub wind_speed_ms: f32,
    pub wind_dir_deg: f32,
    pub efficiency_mah_km: f32,
    pub rangefinder_m: f32,
    pub temperature_c: f32,
    pub esc_rpm: u16,
    pub esc_temp_c: f32,
    pub flight_time_s: u32,
    pub total_distance_m: f32,
    // ── New parity fields ──
    pub xtrack_error_m: f32,
    pub airspeed2_ms: f32,
    pub battery2_voltage: f32,
    pub battery2_used_mah: f32,
    pub current2_a: f32,
    pub climb_efficiency: f32,
    pub esc_amps: f32,
    pub rotor_rpm: u16,
    pub vtx_power_mw: u16,
    pub fence_breached: bool,
    pub baro_temp_c: f32,
    pub airspeed_temp_c: f32,
    pub height_above_terrain_m: f32,
    pub resting_voltage: f32,
    pub avg_cell_rest_volt: f32,
}

impl Default for OsdState {
    fn default() -> Self {
        Self {
            altitude_m: 0.0,
            altitude_agl_m: 0.0,
            airspeed_ms: 0.0,
            groundspeed_ms: 0.0,
            battery_voltage: 0.0,
            battery_current: 0.0,
            battery_remaining_pct: 0.0,
            battery_consumed_mah: 0.0,
            gps_fix: 0,
            gps_sats: 0,
            gps_hdop: 99.9,
            gps_lat: 0.0,
            gps_lon: 0.0,
            home_direction_deg: 0.0,
            home_distance_m: 0.0,
            heading_deg: 0.0,
            roll_deg: 0.0,
            pitch_deg: 0.0,
            throttle_pct: 0.0,
            flight_mode: FlightModeId::Stabilize,
            armed: false,
            ekf_healthy: false,
            message_text: [0; 50],
            message_len: 0,
            climb_rate_ms: 0.0,
            rssi: 0,
            link_quality: 0,
            power_w: 0.0,
            avg_cell_voltage: 0.0,
            wind_speed_ms: 0.0,
            wind_dir_deg: 0.0,
            efficiency_mah_km: 0.0,
            rangefinder_m: 0.0,
            temperature_c: 0.0,
            esc_rpm: 0,
            esc_temp_c: 0.0,
            flight_time_s: 0,
            total_distance_m: 0.0,
            xtrack_error_m: 0.0,
            airspeed2_ms: 0.0,
            battery2_voltage: 0.0,
            battery2_used_mah: 0.0,
            current2_a: 0.0,
            climb_efficiency: 0.0,
            esc_amps: 0.0,
            rotor_rpm: 0,
            vtx_power_mw: 0,
            fence_breached: false,
            baro_temp_c: 0.0,
            airspeed_temp_c: 0.0,
            height_above_terrain_m: 0.0,
            resting_voltage: 0.0,
            avg_cell_rest_volt: 0.0,
        }
    }
}

// ─── Item Renderer ───

/// Format buffer — small stack buffer for rendering numeric values.
struct FmtBuf {
    buf: [u8; 20],
    len: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 20],
            len: 0,
        }
    }

    fn push(&mut self, b: u8) {
        if self.len < 20 {
            self.buf[self.len] = b;
            self.len += 1;
        }
    }

    fn push_str(&mut self, s: &str) {
        for b in s.bytes() {
            self.push(b);
        }
    }

    fn as_str(&self) -> &str {
        // Safety: we only push ASCII bytes
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }

    /// Format an integer into the buffer.
    fn push_i32(&mut self, mut val: i32) {
        if val < 0 {
            self.push(b'-');
            val = -val;
        }
        self.push_u32(val as u32);
    }

    fn push_u32(&mut self, val: u32) {
        if val == 0 {
            self.push(b'0');
            return;
        }
        let mut digits = [0u8; 10];
        let mut n = val;
        let mut i = 0;
        while n > 0 {
            digits[i] = (n % 10) as u8 + b'0';
            n /= 10;
            i += 1;
        }
        // Push in reverse (most significant first)
        while i > 0 {
            i -= 1;
            self.push(digits[i]);
        }
    }

    /// Format a float with one decimal place.
    fn push_f32_1(&mut self, val: f32) {
        if val < 0.0 {
            self.push(b'-');
            self.push_f32_1(-val);
            return;
        }
        let integer = val as u32;
        let frac = ((val - integer as f32) * 10.0 + 0.5) as u32 % 10;
        self.push_u32(integer);
        self.push(b'.');
        self.push(frac as u8 + b'0');
    }
}

/// Render a single OSD item into the backend.
fn render_item(backend: &mut dyn OsdBackend, cfg: &OsdItemConfig, state: &OsdState) {
    if !cfg.enabled {
        return;
    }

    let mut buf = FmtBuf::new();

    match cfg.item {
        OsdItem::Altitude => {
            buf.push_f32_1(state.altitude_m);
            buf.push_str("m");
        }
        OsdItem::AltitudeAgl => {
            buf.push_f32_1(state.altitude_agl_m);
            buf.push_str("m");
        }
        OsdItem::Airspeed => {
            buf.push_f32_1(state.airspeed_ms);
            buf.push_str("m/s");
        }
        OsdItem::Groundspeed => {
            buf.push_f32_1(state.groundspeed_ms);
            buf.push_str("m/s");
        }
        OsdItem::BatteryVoltage => {
            buf.push_f32_1(state.battery_voltage);
            buf.push_str("V");
        }
        OsdItem::BatteryCurrent => {
            buf.push_f32_1(state.battery_current);
            buf.push_str("A");
        }
        OsdItem::BatteryRemaining => {
            buf.push_u32(state.battery_remaining_pct as u32);
            buf.push_str("%");
        }
        OsdItem::BatteryConsumed => {
            buf.push_u32(state.battery_consumed_mah as u32);
            buf.push_str("mAh");
        }
        OsdItem::GpsStatus => {
            match state.gps_fix {
                0 => buf.push_str("NOFIX"),
                2 => buf.push_str("2DFIX"),
                3 => buf.push_str("3DFIX"),
                4 => buf.push_str("DGPS"),
                5 => buf.push_str("RTKF"),
                6 => buf.push_str("RTK"),
                _ => buf.push_str("?FIX"),
            }
        }
        OsdItem::GpsSats => {
            buf.push_u32(state.gps_sats as u32);
            buf.push_str("sat");
        }
        OsdItem::GpsHdop => {
            buf.push_f32_1(state.gps_hdop);
        }
        OsdItem::GpsLatitude => {
            // Simplified: show as fixed-point degrees
            if state.gps_lat < 0.0 {
                buf.push(b'S');
            } else {
                buf.push(b'N');
            }
            let abs_lat = if state.gps_lat < 0.0 { -state.gps_lat } else { state.gps_lat };
            let deg = abs_lat as u32;
            let frac = ((abs_lat - deg as f64) * 10000.0) as u32;
            buf.push_u32(deg);
            buf.push(b'.');
            // Pad fractional part with leading zeros
            if frac < 1000 { buf.push(b'0'); }
            if frac < 100 { buf.push(b'0'); }
            if frac < 10 { buf.push(b'0'); }
            buf.push_u32(frac);
        }
        OsdItem::GpsLongitude => {
            if state.gps_lon < 0.0 {
                buf.push(b'W');
            } else {
                buf.push(b'E');
            }
            let abs_lon = if state.gps_lon < 0.0 { -state.gps_lon } else { state.gps_lon };
            let deg = abs_lon as u32;
            let frac = ((abs_lon - deg as f64) * 10000.0) as u32;
            buf.push_u32(deg);
            buf.push(b'.');
            if frac < 1000 { buf.push(b'0'); }
            if frac < 100 { buf.push(b'0'); }
            if frac < 10 { buf.push(b'0'); }
            buf.push_u32(frac);
        }
        OsdItem::HomeDirection => {
            buf.push_u32(state.home_direction_deg as u32);
            buf.push(b'\xf0'); // degree symbol in MAX7456 charset
        }
        OsdItem::HomeDistance => {
            if state.home_distance_m > 1000.0 {
                buf.push_f32_1(state.home_distance_m / 1000.0);
                buf.push_str("km");
            } else {
                buf.push_u32(state.home_distance_m as u32);
                buf.push_str("m");
            }
        }
        OsdItem::Heading => {
            buf.push_u32(state.heading_deg as u32);
            buf.push(b'\xf0');
        }
        OsdItem::Compass => {
            let dirs = ["N ", "NE", "E ", "SE", "S ", "SW", "W ", "NW"];
            let idx = (((state.heading_deg + 22.5) % 360.0) / 45.0) as usize;
            buf.push_str(dirs[idx.min(7)]);
        }
        OsdItem::Roll => {
            buf.push_str("R");
            buf.push_i32(state.roll_deg as i32);
        }
        OsdItem::Pitch => {
            buf.push_str("P");
            buf.push_i32(state.pitch_deg as i32);
        }
        OsdItem::Throttle => {
            buf.push_u32(state.throttle_pct as u32);
            buf.push_str("%");
        }
        OsdItem::FlightMode => {
            buf.push_str(state.flight_mode.name());
        }
        OsdItem::ArmStatus => {
            if state.armed {
                buf.push_str("ARMED");
            } else {
                buf.push_str("DISARMED");
            }
        }
        OsdItem::EkfStatus => {
            if state.ekf_healthy {
                buf.push_str("EKF:OK");
            } else {
                buf.push_str("EKF:BAD");
            }
        }
        OsdItem::MessageText => {
            let len = state.message_len as usize;
            if len > 0 && len <= state.message_text.len() {
                if let Ok(s) = core::str::from_utf8(&state.message_text[..len]) {
                    buf.push_str(s);
                }
            }
        }
        OsdItem::ClimbRate => {
            buf.push_f32_1(state.climb_rate_ms);
            buf.push_str("m/s");
        }
        OsdItem::Crosshair => {
            buf.push(b'+');
        }
        OsdItem::Horizon => {
            // Simplified horizon line: `-` shifted by pitch
            let offset = (state.pitch_deg / 5.0) as i8;
            if offset > 0 {
                buf.push(b'/');
            } else if offset < 0 {
                buf.push(b'\\');
            } else {
                buf.push(b'-');
            }
        }
        OsdItem::RssiValue => {
            buf.push_u32(state.rssi as u32);
            buf.push_str("dB");
        }
        OsdItem::LinkQuality => {
            buf.push_str("LQ:");
            buf.push_u32(state.link_quality as u32);
        }
        OsdItem::CurrentDraw => {
            buf.push_f32_1(state.battery_current);
            buf.push_str("A");
        }
        OsdItem::Power => {
            buf.push_u32(state.power_w as u32);
            buf.push_str("W");
        }
        OsdItem::AvgCellVoltage => {
            buf.push_f32_1(state.avg_cell_voltage);
            buf.push_str("V");
        }
        OsdItem::WindSpeed => {
            buf.push_f32_1(state.wind_speed_ms);
            buf.push_str("m/s");
        }
        OsdItem::WindDirection => {
            buf.push_u32(state.wind_dir_deg as u32);
            buf.push(b'\xf0');
        }
        OsdItem::Efficiency => {
            buf.push_u32(state.efficiency_mah_km as u32);
            buf.push_str("mAh/km");
        }
        OsdItem::RangefinderDistance => {
            buf.push_f32_1(state.rangefinder_m);
            buf.push_str("m");
        }
        OsdItem::Temperature => {
            buf.push_i32(state.temperature_c as i32);
            buf.push_str("C");
        }
        OsdItem::FlipTime | OsdItem::FlightTime => {
            let secs = state.flight_time_s;
            let m = secs / 60;
            let s = secs % 60;
            buf.push_u32(m);
            buf.push(b':');
            if s < 10 {
                buf.push(b'0');
            }
            buf.push_u32(s);
        }
        OsdItem::EscRpm => {
            buf.push_u32(state.esc_rpm as u32);
            buf.push_str("rpm");
        }
        OsdItem::EscTemperature => {
            buf.push_i32(state.esc_temp_c as i32);
            buf.push_str("C");
        }
        OsdItem::Clock => {
            // Flight time as HH:MM:SS
            let secs = state.flight_time_s;
            let h = secs / 3600;
            let m = (secs % 3600) / 60;
            let s = secs % 60;
            buf.push_u32(h);
            buf.push(b':');
            if m < 10 { buf.push(b'0'); }
            buf.push_u32(m);
            buf.push(b':');
            if s < 10 { buf.push(b'0'); }
            buf.push_u32(s);
        }
        OsdItem::Callsign => {
            // Callsign stored in message_text when item is Callsign
            // (placeholder — real implementation would read from params)
            buf.push_str("MRDN");
        }
        OsdItem::TotalDistance => {
            if state.total_distance_m > 1000.0 {
                buf.push_f32_1(state.total_distance_m / 1000.0);
                buf.push_str("km");
            } else {
                buf.push_u32(state.total_distance_m as u32);
                buf.push_str("m");
            }
        }
        OsdItem::MaxAltitude | OsdItem::MaxSpeed | OsdItem::MaxDistance => {
            // Stats items — rendered on stats screens with stored max values
            buf.push_str("--");
        }
        OsdItem::MotorOutput => {
            buf.push_u32(state.throttle_pct as u32);
            buf.push_str("%M");
        }
        OsdItem::RcChannelValue => {
            buf.push_str("RC:--");
        }
        OsdItem::TerrainHeight => {
            buf.push_f32_1(state.altitude_agl_m);
            buf.push_str("t");
        }
        OsdItem::WaypointDistance | OsdItem::WaypointBearing | OsdItem::NextWaypoint => {
            buf.push_str("WP:--");
        }
        OsdItem::GpsSpeed3D => {
            buf.push_f32_1(state.groundspeed_ms);
            buf.push_str("3D");
        }
        OsdItem::VerticalSpeed => {
            buf.push_f32_1(state.climb_rate_ms);
            buf.push_str("vs");
        }
        OsdItem::BatteryVoltage2 => {
            buf.push_f32_1(state.battery_voltage);
            buf.push_str("V2");
        }
        OsdItem::XtrackError => {
            buf.push_f32_1(state.xtrack_error_m);
            buf.push_str("xte");
        }
        OsdItem::PlusCode => {
            buf.push_str("OLC:--");
        }
        OsdItem::Airspeed2 => {
            buf.push_f32_1(state.airspeed2_ms);
            buf.push_str("a2");
        }
        OsdItem::Sidebars => {
            buf.push_str("|");
        }
        OsdItem::RestingVoltage => {
            buf.push_f32_1(state.resting_voltage);
            buf.push_str("Vr");
        }
        OsdItem::AvgCellRestVolt => {
            buf.push_f32_1(state.avg_cell_rest_volt);
            buf.push_str("Vc");
        }
        OsdItem::Battery2Voltage => {
            buf.push_f32_1(state.battery2_voltage);
            buf.push_str("V2");
        }
        OsdItem::Battery2Used => {
            buf.push_u32(state.battery2_used_mah as u32);
            buf.push_str("mA2");
        }
        OsdItem::Current2 => {
            buf.push_f32_1(state.current2_a);
            buf.push_str("A2");
        }
        OsdItem::ClimbEfficiency => {
            buf.push_f32_1(state.climb_efficiency);
            buf.push_str("ce");
        }
        OsdItem::EscAmps => {
            buf.push_f32_1(state.esc_amps);
            buf.push_str("eA");
        }
        OsdItem::RotorRpm => {
            buf.push_u32(state.rotor_rpm as u32);
            buf.push_str("rpm");
        }
        OsdItem::VtxPower => {
            buf.push_u32(state.vtx_power_mw as u32);
            buf.push_str("mW");
        }
        OsdItem::FenceStatus => {
            if state.fence_breached {
                buf.push_str("FENCE!");
            } else {
                buf.push_str("FNC:OK");
            }
        }
        OsdItem::StatSummary => {
            buf.push_str("STATS");
        }
        OsdItem::BaroTemp => {
            buf.push_i32(state.baro_temp_c as i32);
            buf.push_str("bC");
        }
        OsdItem::AirspeedTemp => {
            buf.push_i32(state.airspeed_temp_c as i32);
            buf.push_str("aC");
        }
        OsdItem::HeightAboveTerrain => {
            buf.push_f32_1(state.height_above_terrain_m);
            buf.push_str("tH");
        }
    }

    backend.write_string(cfg.x, cfg.y, buf.as_str());
}

// ─── OSD Manager ───

/// Screen type for the OSD manager.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScreenSlot {
    /// Normal flight screens (0-3), switched via RC channel.
    Flight(u8),
    /// Post-flight statistics screens (0-1).
    Stats(u8),
}

/// Screen switching mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScreenSwitchMode {
    /// PWM range bands (current behavior).
    PwmRange,
    /// Toggle: each transition advances to the next screen.
    Toggle,
    /// Auto-switch: armed=screen0, failsafe=screen1, disarmed=screen2.
    AutoSwitch,
}

/// Stats tracking for max-value items.
#[derive(Debug, Clone, Copy, Default)]
pub struct OsdStats {
    pub max_altitude_m: f32,
    pub max_speed_ms: f32,
    pub max_distance_m: f32,
}

impl OsdStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, state: &OsdState) {
        if state.altitude_m > self.max_altitude_m {
            self.max_altitude_m = state.altitude_m;
        }
        if state.groundspeed_ms > self.max_speed_ms {
            self.max_speed_ms = state.groundspeed_ms;
        }
        if state.home_distance_m > self.max_distance_m {
            self.max_distance_m = state.home_distance_m;
        }
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// MSP DisplayPort HD resolution support stub.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MspResolution {
    /// Standard definition: 30x16.
    Sd,
    /// HD 50x18.
    Hd50x18,
    /// HD 53x20.
    Hd53x20,
}

impl MspResolution {
    pub fn cols(&self) -> u8 {
        match self {
            MspResolution::Sd => 30,
            MspResolution::Hd50x18 => 50,
            MspResolution::Hd53x20 => 53,
        }
    }

    pub fn rows(&self) -> u8 {
        match self {
            MspResolution::Sd => 16,
            MspResolution::Hd50x18 => 18,
            MspResolution::Hd53x20 => 20,
        }
    }
}

/// SITL OSD backend stub — stores display content in memory for testing.
pub struct SitlOsdBackend {
    pub buffer: [[u8; 53]; 20],
    pub cols: u8,
    pub rows: u8,
    pub cleared: bool,
    pub flushed: bool,
}

impl SitlOsdBackend {
    pub fn new(cols: u8, rows: u8) -> Self {
        Self {
            buffer: [[0x20; 53]; 20],
            cols,
            rows,
            cleared: false,
            flushed: false,
        }
    }
}

impl OsdBackend for SitlOsdBackend {
    fn init(&mut self) -> bool { true }

    fn clear(&mut self) {
        for row in self.buffer.iter_mut() {
            for c in row.iter_mut() { *c = 0x20; }
        }
        self.cleared = true;
    }

    fn write_char(&mut self, x: u8, y: u8, ch: u8) {
        if (x as usize) < self.cols as usize && (y as usize) < self.rows as usize {
            self.buffer[y as usize][x as usize] = ch;
        }
    }

    fn write_string(&mut self, x: u8, y: u8, s: &str) {
        for (i, byte) in s.bytes().enumerate() {
            let cx = x as usize + i;
            if cx >= self.cols as usize { break; }
            self.buffer[y as usize][cx] = byte;
        }
    }

    fn flush(&mut self) { self.flushed = true; }
}

/// Font upload stub — placeholder for MAX7456 NVM font character upload.
pub struct FontUpload;

impl FontUpload {
    /// Upload a single character to the MAX7456 NVM.
    /// `char_index`: character position (0-255).
    /// `data`: 54 bytes of character pixel data.
    /// Returns Ok on success. Currently a stub — always succeeds.
    pub fn upload_char<S: SpiDevice>(_spi: &mut S, _char_index: u8, _data: &[u8; 54]) -> bool {
        // Stub: real implementation would write to MAX7456 NVM
        true
    }
}

/// OSD Manager — owns multiple screens and drives a backend.
///
/// 4 switchable flight screens + 2 stats screens.
/// Active screen selected by RC channel PWM ranges:
/// - 1000-1249 → Screen 0
/// - 1250-1499 → Screen 1
/// - 1500-1749 → Screen 2
/// - 1750-2000 → Screen 3
pub struct OsdManager {
    /// 4 flight screens + 2 stats screens = 6 total.
    screens: [OsdScreen; 6],
    active_screen: u8,
    /// RC channel index used for screen switching (0-based).
    switch_channel: u8,
    /// Screen switching mode.
    pub switch_mode: ScreenSwitchMode,
    /// Stats tracking state.
    pub stats: OsdStats,
    /// Debounce timer for screen switching (ms).
    last_switch_ms: u32,
    /// Previous PWM value for toggle detection.
    prev_switch_pwm: u16,
    /// MSP heartbeat interval timer (ms).
    pub msp_heartbeat_interval_ms: u32,
    last_msp_heartbeat_ms: u32,
}

impl OsdManager {
    pub fn new() -> Self {
        Self {
            screens: [
                OsdScreen::new(),
                OsdScreen::new(),
                OsdScreen::new(),
                OsdScreen::new(),
                OsdScreen::new(), // stats 0
                OsdScreen::new(), // stats 1
            ],
            active_screen: 0,
            switch_channel: 5,
            switch_mode: ScreenSwitchMode::PwmRange,
            stats: OsdStats::new(),
            last_switch_ms: 0,
            prev_switch_pwm: 0,
            msp_heartbeat_interval_ms: 500,
            last_msp_heartbeat_ms: 0,
        }
    }

    /// Get a mutable reference to a flight screen (0-3).
    pub fn flight_screen_mut(&mut self, index: u8) -> Option<&mut OsdScreen> {
        if index < 4 {
            Some(&mut self.screens[index as usize])
        } else {
            None
        }
    }

    /// Get a mutable reference to a stats screen (0-1).
    pub fn stats_screen_mut(&mut self, index: u8) -> Option<&mut OsdScreen> {
        if index < 2 {
            Some(&mut self.screens[4 + index as usize])
        } else {
            None
        }
    }

    /// Set which RC channel is used for screen switching.
    pub fn set_switch_channel(&mut self, ch: u8) {
        self.switch_channel = ch;
    }

    /// Get the currently active screen index.
    pub fn active_screen(&self) -> u8 {
        self.active_screen
    }

    /// Select the active screen from an RC channel PWM value.
    /// Respects the current `switch_mode` and 300ms debounce.
    pub fn select_screen_from_pwm(&mut self, pwm: u16) {
        self.select_screen_from_pwm_timed(pwm, 0);
    }

    /// Select screen with timestamp for debounce.
    pub fn select_screen_from_pwm_timed(&mut self, pwm: u16, now_ms: u32) {
        // 300ms debounce
        if now_ms.wrapping_sub(self.last_switch_ms) < 300 && self.last_switch_ms != 0 {
            self.prev_switch_pwm = pwm;
            return;
        }

        match self.switch_mode {
            ScreenSwitchMode::PwmRange => {
                let new_screen = match pwm {
                    0..=1249 => 0,
                    1250..=1499 => 1,
                    1500..=1749 => 2,
                    _ => 3,
                };
                if new_screen != self.active_screen {
                    self.active_screen = new_screen;
                    self.last_switch_ms = now_ms;
                }
            }
            ScreenSwitchMode::Toggle => {
                // Detect transition (PWM crosses 1500)
                let prev_high = self.prev_switch_pwm >= 1500;
                let curr_high = pwm >= 1500;
                if prev_high != curr_high {
                    self.active_screen = (self.active_screen + 1) % 4;
                    self.last_switch_ms = now_ms;
                }
            }
            ScreenSwitchMode::AutoSwitch => {
                // Auto-switch is handled in update_auto_switch
            }
        }
        self.prev_switch_pwm = pwm;
    }

    /// Auto-switch mode: select screen based on vehicle state.
    pub fn update_auto_switch(&mut self, armed: bool, failsafe: bool) {
        if self.switch_mode == ScreenSwitchMode::AutoSwitch {
            if failsafe {
                self.active_screen = 1;
            } else if armed {
                self.active_screen = 0;
            } else {
                self.active_screen = 2;
            }
        }
    }

    /// Switch to a stats screen (0 or 1). Returns false if index is out of range.
    pub fn show_stats_screen(&mut self, index: u8) -> bool {
        if index < 2 {
            self.active_screen = 4 + index;
            true
        } else {
            false
        }
    }

    /// Update: render all enabled items on the active screen to the backend.
    /// Also tracks max statistics for stats screens.
    pub fn update(&mut self, backend: &mut dyn OsdBackend, state: &OsdState) {
        // Track stats
        self.stats.update(state);

        backend.clear();

        let screen = &self.screens[self.active_screen as usize];
        for cfg in screen.items.iter() {
            // For stats items, render actual tracked values instead of "--"
            if cfg.enabled {
                match cfg.item {
                    OsdItem::MaxAltitude => {
                        let mut buf = FmtBuf::new();
                        buf.push_f32_1(self.stats.max_altitude_m);
                        buf.push_str("m");
                        backend.write_string(cfg.x, cfg.y, buf.as_str());
                        continue;
                    }
                    OsdItem::MaxSpeed => {
                        let mut buf = FmtBuf::new();
                        buf.push_f32_1(self.stats.max_speed_ms);
                        buf.push_str("m/s");
                        backend.write_string(cfg.x, cfg.y, buf.as_str());
                        continue;
                    }
                    OsdItem::MaxDistance => {
                        let mut buf = FmtBuf::new();
                        buf.push_f32_1(self.stats.max_distance_m);
                        buf.push_str("m");
                        backend.write_string(cfg.x, cfg.y, buf.as_str());
                        continue;
                    }
                    _ => {}
                }
            }
            render_item(backend, cfg, state);
        }

        backend.flush();
    }

    /// Check if MSP heartbeat is due and should be sent.
    pub fn msp_heartbeat_due(&mut self, now_ms: u32) -> bool {
        if now_ms.wrapping_sub(self.last_msp_heartbeat_ms) >= self.msp_heartbeat_interval_ms {
            self.last_msp_heartbeat_ms = now_ms;
            true
        } else {
            false
        }
    }
}

impl Default for OsdManager {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Tests ───

#[cfg(test)]
mod tests {
    use super::*;

    // ── Mock backend for testing ──

    struct MockBackend {
        chars: [[u8; 30]; 16],
        cleared: bool,
        flushed: bool,
        init_called: bool,
    }

    impl MockBackend {
        fn new() -> Self {
            Self {
                chars: [[0x20; 30]; 16],
                cleared: false,
                flushed: false,
                init_called: false,
            }
        }

        fn string_at(&self, x: u8, y: u8, len: usize) -> &str {
            let row = &self.chars[y as usize];
            let start = x as usize;
            let end = (start + len).min(30);
            core::str::from_utf8(&row[start..end]).unwrap_or("")
        }
    }

    impl OsdBackend for MockBackend {
        fn init(&mut self) -> bool {
            self.init_called = true;
            true
        }

        fn clear(&mut self) {
            for row in self.chars.iter_mut() {
                for c in row.iter_mut() {
                    *c = 0x20;
                }
            }
            self.cleared = true;
        }

        fn write_char(&mut self, x: u8, y: u8, ch: u8) {
            if (x as usize) < 30 && (y as usize) < 16 {
                self.chars[y as usize][x as usize] = ch;
            }
        }

        fn write_string(&mut self, x: u8, y: u8, s: &str) {
            for (i, byte) in s.bytes().enumerate() {
                let cx = x as usize + i;
                if cx >= 30 {
                    break;
                }
                self.chars[y as usize][cx] = byte;
            }
        }

        fn flush(&mut self) {
            self.flushed = true;
        }
    }

    // ── FmtBuf tests ──

    #[test]
    fn test_fmtbuf_u32() {
        let mut buf = FmtBuf::new();
        buf.push_u32(12345);
        assert_eq!(buf.as_str(), "12345");
    }

    #[test]
    fn test_fmtbuf_zero() {
        let mut buf = FmtBuf::new();
        buf.push_u32(0);
        assert_eq!(buf.as_str(), "0");
    }

    #[test]
    fn test_fmtbuf_i32_negative() {
        let mut buf = FmtBuf::new();
        buf.push_i32(-42);
        assert_eq!(buf.as_str(), "-42");
    }

    #[test]
    fn test_fmtbuf_f32() {
        let mut buf = FmtBuf::new();
        buf.push_f32_1(12.3);
        assert_eq!(buf.as_str(), "12.3");
    }

    #[test]
    fn test_fmtbuf_f32_negative() {
        let mut buf = FmtBuf::new();
        buf.push_f32_1(-5.7);
        assert_eq!(buf.as_str(), "-5.7");
    }

    // ── OsdScreen tests ──

    #[test]
    fn test_screen_add_item() {
        let mut screen = OsdScreen::new();
        assert!(screen.add_item(OsdItem::Altitude, 0, 0));
        assert_eq!(screen.items.len(), 1);
        assert_eq!(screen.items[0].item, OsdItem::Altitude);
        assert!(screen.items[0].enabled);
    }

    #[test]
    fn test_screen_set_enabled() {
        let mut screen = OsdScreen::new();
        screen.add_item(OsdItem::Altitude, 0, 0);
        screen.set_enabled(OsdItem::Altitude, false);
        assert!(!screen.items[0].enabled);
    }

    #[test]
    fn test_screen_max_items() {
        let mut screen = OsdScreen::new();
        for i in 0..64 {
            assert!(screen.add_item(OsdItem::Altitude, (i % 30) as u8, (i / 30) as u8));
        }
        // 65th should fail
        assert!(!screen.add_item(OsdItem::Altitude, 0, 0));
    }

    // ── OSD Manager tests ──

    #[test]
    fn test_manager_screen_switching_pwm() {
        let mut mgr = OsdManager::new();
        mgr.select_screen_from_pwm(1000);
        assert_eq!(mgr.active_screen(), 0);
        mgr.select_screen_from_pwm(1300);
        assert_eq!(mgr.active_screen(), 1);
        mgr.select_screen_from_pwm(1600);
        assert_eq!(mgr.active_screen(), 2);
        mgr.select_screen_from_pwm(1800);
        assert_eq!(mgr.active_screen(), 3);
    }

    #[test]
    fn test_manager_stats_screen() {
        let mut mgr = OsdManager::new();
        assert!(mgr.show_stats_screen(0));
        assert_eq!(mgr.active_screen(), 4);
        assert!(mgr.show_stats_screen(1));
        assert_eq!(mgr.active_screen(), 5);
        assert!(!mgr.show_stats_screen(2));
    }

    #[test]
    fn test_manager_update_renders_items() {
        let mut mgr = OsdManager::new();
        mgr.flight_screen_mut(0).unwrap().add_item(OsdItem::ArmStatus, 0, 0);
        mgr.flight_screen_mut(0).unwrap().add_item(OsdItem::BatteryVoltage, 0, 1);

        let mut backend = MockBackend::new();
        let mut state = OsdState::default();
        state.armed = true;
        state.battery_voltage = 12.6;

        mgr.update(&mut backend, &state);

        assert!(backend.cleared);
        assert!(backend.flushed);
        assert_eq!(backend.string_at(0, 0, 5), "ARMED");
        assert_eq!(backend.string_at(0, 1, 5), "12.6V");
    }

    #[test]
    fn test_manager_disabled_item_not_rendered() {
        let mut mgr = OsdManager::new();
        mgr.flight_screen_mut(0).unwrap().add_item(OsdItem::ArmStatus, 0, 0);
        mgr.flight_screen_mut(0).unwrap().set_enabled(OsdItem::ArmStatus, false);

        let mut backend = MockBackend::new();
        let state = OsdState::default();
        mgr.update(&mut backend, &state);

        // Row 0 should still be spaces
        assert_eq!(backend.string_at(0, 0, 8), "        ");
    }

    // ── Item rendering tests ──

    #[test]
    fn test_render_altitude() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::Altitude,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.altitude_m = 100.5;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 6), "100.5m");
    }

    #[test]
    fn test_render_gps_status_3d() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::GpsStatus,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.gps_fix = 3;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 5), "3DFIX");
    }

    #[test]
    fn test_render_gps_sats() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::GpsSats,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.gps_sats = 14;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 5), "14sat");
    }

    #[test]
    fn test_render_flight_mode() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::FlightMode,
            x: 5,
            y: 3,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.flight_mode = FlightModeId::Loiter;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(5, 3, 6), "LOITER");
    }

    #[test]
    fn test_render_compass_directions() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::Compass,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();

        // North
        state.heading_deg = 0.0;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 2), "N ");

        // East
        backend.clear();
        state.heading_deg = 90.0;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 2), "E ");

        // South
        backend.clear();
        state.heading_deg = 180.0;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 2), "S ");
    }

    #[test]
    fn test_render_home_distance_km() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::HomeDistance,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.home_distance_m = 2500.0;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 5), "2.5km");
    }

    #[test]
    fn test_render_throttle() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::Throttle,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.throttle_pct = 75.0;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 3), "75%");
    }

    #[test]
    fn test_render_clock() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::Clock,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.flight_time_s = 3661; // 1:01:01
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 7), "1:01:01");
    }

    #[test]
    fn test_render_ekf_status() {
        let mut backend = MockBackend::new();
        let cfg = OsdItemConfig {
            item: OsdItem::EkfStatus,
            x: 0,
            y: 0,
            enabled: true,
        };
        let mut state = OsdState::default();
        state.ekf_healthy = true;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 6), "EKF:OK");

        backend.clear();
        state.ekf_healthy = false;
        render_item(&mut backend, &cfg, &state);
        assert_eq!(backend.string_at(0, 0, 7), "EKF:BAD");
    }

    // ── Video standard tests ──

    #[test]
    fn test_video_standard_dimensions() {
        assert_eq!(VideoStandard::Pal.rows(), 16);
        assert_eq!(VideoStandard::Pal.columns(), 30);
        assert_eq!(VideoStandard::Ntsc.rows(), 13);
        assert_eq!(VideoStandard::Ntsc.columns(), 30);
    }

    // ── OSD item enum coverage ──

    #[test]
    fn test_osd_item_count() {
        // Ensure we have 56+ item variants
        let items = [
            OsdItem::Altitude, OsdItem::AltitudeAgl, OsdItem::Airspeed, OsdItem::Groundspeed,
            OsdItem::BatteryVoltage, OsdItem::BatteryCurrent, OsdItem::BatteryRemaining,
            OsdItem::BatteryConsumed, OsdItem::GpsStatus, OsdItem::GpsSats, OsdItem::GpsHdop,
            OsdItem::GpsLatitude, OsdItem::GpsLongitude, OsdItem::HomeDirection,
            OsdItem::HomeDistance, OsdItem::Heading, OsdItem::Compass, OsdItem::Roll,
            OsdItem::Pitch, OsdItem::Throttle, OsdItem::FlightMode, OsdItem::ArmStatus,
            OsdItem::EkfStatus, OsdItem::MessageText, OsdItem::ClimbRate, OsdItem::Crosshair,
            OsdItem::Horizon, OsdItem::RssiValue, OsdItem::LinkQuality, OsdItem::CurrentDraw,
            OsdItem::Power, OsdItem::AvgCellVoltage, OsdItem::WindSpeed, OsdItem::WindDirection,
            OsdItem::Efficiency, OsdItem::RangefinderDistance, OsdItem::Temperature,
            OsdItem::FlipTime, OsdItem::EscRpm, OsdItem::EscTemperature, OsdItem::Clock,
            OsdItem::Callsign, OsdItem::TotalDistance, OsdItem::MaxAltitude, OsdItem::MaxSpeed,
            OsdItem::MaxDistance, OsdItem::FlightTime, OsdItem::MotorOutput,
            OsdItem::RcChannelValue, OsdItem::TerrainHeight, OsdItem::WaypointDistance,
            OsdItem::WaypointBearing, OsdItem::NextWaypoint, OsdItem::GpsSpeed3D,
            OsdItem::VerticalSpeed, OsdItem::BatteryVoltage2,
            OsdItem::XtrackError, OsdItem::PlusCode, OsdItem::Airspeed2,
            OsdItem::Sidebars, OsdItem::RestingVoltage, OsdItem::AvgCellRestVolt,
            OsdItem::Battery2Voltage, OsdItem::Battery2Used, OsdItem::Current2,
            OsdItem::ClimbEfficiency, OsdItem::EscAmps, OsdItem::RotorRpm,
            OsdItem::VtxPower, OsdItem::FenceStatus, OsdItem::StatSummary,
            OsdItem::BaroTemp, OsdItem::AirspeedTemp, OsdItem::HeightAboveTerrain,
        ];
        assert!(items.len() >= 68, "Need at least 68 OSD items, got {}", items.len());
    }

    // ── Multi-screen rendering test ──

    #[test]
    fn test_manager_different_screens_render_different_items() {
        let mut mgr = OsdManager::new();

        // Screen 0: altitude only
        mgr.flight_screen_mut(0).unwrap().add_item(OsdItem::Altitude, 0, 0);

        // Screen 1: flight mode only
        mgr.flight_screen_mut(1).unwrap().add_item(OsdItem::FlightMode, 0, 0);

        let mut backend = MockBackend::new();
        let mut state = OsdState::default();
        state.altitude_m = 50.0;
        state.flight_mode = FlightModeId::RTL;

        // Render screen 0
        mgr.select_screen_from_pwm(1000);
        mgr.update(&mut backend, &state);
        assert_eq!(backend.string_at(0, 0, 5), "50.0m");

        // Switch to screen 1
        mgr.select_screen_from_pwm(1300);
        mgr.update(&mut backend, &state);
        assert_eq!(backend.string_at(0, 0, 3), "RTL");
    }
}
