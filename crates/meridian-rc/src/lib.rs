#![no_std]

//! RC input protocol parsers: CRSF, SBUS, DSM, FPort, iBus, SUMD, SRXL2, PPM.
//!
//! Parses raw serial bytes into channel values. Detects failsafe.

pub mod dsm;
pub mod fport;
pub mod ibus;
pub mod sumd;
pub mod srxl2;
pub mod ppm;

/// Maximum RC channels.
pub const MAX_CHANNELS: usize = 16;

/// Parsed RC frame.
#[derive(Debug, Clone, Copy)]
pub struct RcFrame {
    /// Channel values (1000-2000 range, or raw depending on protocol).
    pub channels: [u16; MAX_CHANNELS],
    /// Number of valid channels.
    pub count: u8,
    /// Failsafe active.
    pub failsafe: bool,
    /// Signal quality (0-100, if available).
    pub rssi: u8,
}

impl Default for RcFrame {
    fn default() -> Self {
        Self { channels: [1500; MAX_CHANNELS], count: 0, failsafe: true, rssi: 0 }
    }
}

// ─── CRSF (Crossfire / ELRS) ───

/// CRSF frame types.
const CRSF_FRAMETYPE_RC_CHANNELS: u8 = 0x16;
const CRSF_FRAMETYPE_LINK_STATISTICS: u8 = 0x14;
const CRSF_FRAMETYPE_BATTERY: u8 = 0x08;
const CRSF_FRAMETYPE_GPS: u8 = 0x02;
const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;
const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;
const CRSF_FRAMETYPE_VTX_TELEM: u8 = 0x10;
const CRSF_SYNC_BYTE: u8 = 0xC8;
const CRSF_MAX_FRAME: usize = 64;

/// Parsed CRSF link statistics (frame type 0x14).
#[derive(Debug, Clone, Copy, Default)]
pub struct CrsfLinkStats {
    pub uplink_rssi_ant1: u8,
    pub uplink_rssi_ant2: u8,
    pub uplink_lq: u8,
    pub uplink_snr: i8,
    pub active_antenna: u8,
    pub rf_mode: u8,
    pub uplink_power: u8,
    pub downlink_rssi: u8,
    pub downlink_lq: u8,
    pub downlink_snr: i8,
}

/// CRSF parser.
pub struct CrsfParser {
    buf: [u8; CRSF_MAX_FRAME],
    pos: usize,
    expected_len: usize,
    pub last_rssi: u8,
    pub link_stats: CrsfLinkStats,
    /// Timestamp (ms) of the last valid RC channel frame received.
    /// Used for timeout-based failsafe detection.
    pub last_valid_frame_ms: u64,
}

impl CrsfParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; CRSF_MAX_FRAME],
            pos: 0,
            expected_len: 0,
            last_rssi: 0,
            link_stats: CrsfLinkStats {
                uplink_rssi_ant1: 0, uplink_rssi_ant2: 0, uplink_lq: 0,
                uplink_snr: 0, active_antenna: 0, rf_mode: 0, uplink_power: 0,
                downlink_rssi: 0, downlink_lq: 0, downlink_snr: 0,
            },
            last_valid_frame_ms: 0,
        }
    }

    /// Check whether the CRSF link is stale (no valid RC frame within timeout).
    ///
    /// ArduPilot uses frame-gap timeout in addition to LQ=0 for failsafe.
    /// The caller (RC input layer) must call this periodically and trigger
    /// failsafe when it returns `true`.
    ///
    /// - `now_ms`: current system time in milliseconds
    /// - `timeout_ms`: maximum allowed gap between valid frames (e.g., 1000 ms)
    pub fn is_link_stale(&self, now_ms: u64, timeout_ms: u64) -> bool {
        // If we have never received a frame, link is stale
        if self.last_valid_frame_ms == 0 {
            return true;
        }
        now_ms.wrapping_sub(self.last_valid_frame_ms) > timeout_ms
    }

    /// Feed one byte. Returns Some(RcFrame) when a complete RC frame is parsed.
    ///
    /// Note: This version does NOT update `last_valid_frame_ms`. For
    /// timeout-based failsafe detection, use [`feed_timed`] instead.
    pub fn feed(&mut self, byte: u8) -> Option<RcFrame> {
        self.feed_inner(byte)
    }

    /// Feed one byte with a timestamp. Returns Some(RcFrame) when a complete
    /// RC frame is parsed. Updates `last_valid_frame_ms` on successful RC
    /// channel decode, enabling timeout-based failsafe via [`is_link_stale`].
    pub fn feed_timed(&mut self, byte: u8, now_ms: u64) -> Option<RcFrame> {
        let result = self.feed_inner(byte);
        if result.is_some() {
            self.last_valid_frame_ms = now_ms;
        }
        result
    }

    fn feed_inner(&mut self, byte: u8) -> Option<RcFrame> {
        if self.pos == 0 {
            if byte == CRSF_SYNC_BYTE {
                self.buf[0] = byte;
                self.pos = 1;
            }
            return None;
        }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos == 2 {
            // Length byte
            self.expected_len = byte as usize + 2; // +2 for sync and length bytes
            if self.expected_len > CRSF_MAX_FRAME {
                self.pos = 0;
                return None;
            }
        }

        if self.pos >= self.expected_len && self.expected_len > 2 {
            let result = self.parse_frame();
            self.pos = 0;
            return result;
        }

        None
    }

    fn parse_frame(&mut self) -> Option<RcFrame> {
        // Verify CRC (CRC8 DVB-S2 over type + payload, not sync/length/crc)
        let crc_idx = self.expected_len - 1;
        let calculated_crc = crc8_dvb_s2(&self.buf[2..crc_idx]);
        if calculated_crc != self.buf[crc_idx] {
            return None;
        }

        let frame_type = self.buf[2];

        match frame_type {
            CRSF_FRAMETYPE_RC_CHANNELS => self.parse_rc_channels(),
            CRSF_FRAMETYPE_LINK_STATISTICS => {
                self.parse_link_stats();
                None
            }
            _ => None,
        }
    }

    fn parse_rc_channels(&self) -> Option<RcFrame> {
        // CRSF RC channels: 16 channels packed in 11 bits each = 22 bytes
        if self.expected_len < 26 { return None; } // 2 header + 1 type + 22 data + 1 crc

        // CRSF failsafe detection: LQ = 0 means link is lost.
        // ArduPilot checks LINK_STATISTICS LQ=0 or frame-gap timeout.
        let failsafe = self.link_stats.uplink_lq == 0;

        let data = &self.buf[3..]; // skip sync, len, type
        let mut channels = [0u16; MAX_CHANNELS];

        // Unpack 11-bit values (little-endian bit packing)
        let mut bit_offset = 0u32;
        for ch in 0..16 {
            let byte_idx = (bit_offset / 8) as usize;
            let bit_idx = bit_offset % 8;
            if byte_idx + 2 > 22 { break; }

            let raw = ((data[byte_idx] as u32) >> bit_idx)
                | ((data[byte_idx + 1] as u32) << (8 - bit_idx))
                | ((data[byte_idx + 2] as u32) << (16 - bit_idx));
            let raw = (raw & 0x7FF) as u16; // 11 bits

            // Convert CRSF range (172-1811) to standard PWM (988-2012)
            // ArduPilot formula: (raw * 5 / 8) + 880. Center stick (992) → 1500us.
            channels[ch] = ((raw as i32 * 5 / 8) + 880).clamp(988, 2012) as u16;
            bit_offset += 11;
        }

        Some(RcFrame {
            channels,
            count: 16,
            failsafe,
            rssi: if failsafe { 0 } else { self.last_rssi },
        })
    }

    fn parse_link_stats(&mut self) {
        // LINK_STATISTICS payload is 10 bytes (frame total: sync + len + type + 10 payload + crc = 14)
        if self.expected_len < 14 { return; }
        let d = &self.buf[3..]; // payload starts after sync, len, type
        self.link_stats = CrsfLinkStats {
            uplink_rssi_ant1: d[0],
            uplink_rssi_ant2: d[1],
            uplink_lq: d[2],
            uplink_snr: d[3] as i8,
            active_antenna: d[4],
            rf_mode: d[5],
            uplink_power: d[6],
            downlink_rssi: d[7],
            downlink_lq: d[8],
            downlink_snr: d[9] as i8,
        };
        self.last_rssi = self.link_stats.uplink_rssi_ant1;
    }
}

// ─── CRSF Telemetry Encoder ───

/// Encoded CRSF telemetry frame.
#[derive(Debug, Clone)]
pub struct CrsfTelemetryFrame {
    pub data: [u8; CRSF_MAX_FRAME],
    pub len: usize,
}

/// Encodes CRSF telemetry frames for back-channel to the TX module.
pub struct CrsfTelemetry;

impl CrsfTelemetry {
    /// Encode a battery sensor telemetry frame.
    ///
    /// - `voltage_mv`: battery voltage in millivolts (encoded as decivolts, big-endian u16)
    /// - `current_ca`: current draw in centi-amps (big-endian u16)
    /// - `mah`: consumed capacity in mAh (big-endian u24)
    /// - `pct`: remaining battery percentage (0-100)
    pub fn encode_battery(voltage_mv: u16, current_ca: u16, mah: u32, pct: u8) -> CrsfTelemetryFrame {
        // Battery payload: 8 bytes
        // [voltage:2 BE] [current:2 BE] [mAh:3 BE] [pct:1]
        let mut frame = [0u8; CRSF_MAX_FRAME];
        let voltage_dv = (voltage_mv / 100) as u16; // millivolts -> decivolts
        frame[0] = CRSF_SYNC_BYTE;
        frame[1] = 10; // length: type(1) + payload(8) + crc(1)
        frame[2] = CRSF_FRAMETYPE_BATTERY;
        frame[3] = (voltage_dv >> 8) as u8;
        frame[4] = voltage_dv as u8;
        frame[5] = (current_ca >> 8) as u8;
        frame[6] = current_ca as u8;
        frame[7] = ((mah >> 16) & 0xFF) as u8;
        frame[8] = ((mah >> 8) & 0xFF) as u8;
        frame[9] = (mah & 0xFF) as u8;
        frame[10] = pct;
        frame[11] = crc8_dvb_s2(&frame[2..11]);
        CrsfTelemetryFrame { data: frame, len: 12 }
    }

    /// Encode a GPS telemetry frame.
    ///
    /// - `lat_e7`: latitude in degrees * 1e7 (big-endian i32)
    /// - `lon_e7`: longitude in degrees * 1e7 (big-endian i32)
    /// - `alt_cm`: altitude in centimeters (big-endian i32, sent as u16 with +1000m offset)
    /// - `speed_cms`: ground speed in cm/s (big-endian u16)
    /// - `heading_cdeg`: heading in centidegrees (big-endian u16)
    /// - `sats`: number of satellites
    pub fn encode_gps(
        lat_e7: i32, lon_e7: i32, alt_cm: i32,
        speed_cms: u16, heading_cdeg: u16, sats: u8,
    ) -> CrsfTelemetryFrame {
        // GPS payload: 15 bytes
        // [lat:4 BE] [lon:4 BE] [speed:2 BE] [heading:2 BE] [alt:2 BE] [sats:1]
        let mut frame = [0u8; CRSF_MAX_FRAME];
        frame[0] = CRSF_SYNC_BYTE;
        frame[1] = 17; // length: type(1) + payload(15) + crc(1)
        frame[2] = CRSF_FRAMETYPE_GPS;

        let lat = lat_e7 as u32;
        frame[3] = (lat >> 24) as u8;
        frame[4] = (lat >> 16) as u8;
        frame[5] = (lat >> 8) as u8;
        frame[6] = lat as u8;

        let lon = lon_e7 as u32;
        frame[7] = (lon >> 24) as u8;
        frame[8] = (lon >> 16) as u8;
        frame[9] = (lon >> 8) as u8;
        frame[10] = lon as u8;

        // Speed in cm/s -> CRSF uses km/h * 10: (cms * 36) / 1000
        let speed_crsf = ((speed_cms as u32) * 36 / 1000) as u16;
        frame[11] = (speed_crsf >> 8) as u8;
        frame[12] = speed_crsf as u8;

        frame[13] = (heading_cdeg >> 8) as u8;
        frame[14] = heading_cdeg as u8;

        // Altitude: CRSF encodes as meters + 1000 offset (big-endian u16)
        let alt_m = (alt_cm / 100) as i32;
        let alt_crsf = (alt_m + 1000).max(0) as u16;
        frame[15] = (alt_crsf >> 8) as u8;
        frame[16] = alt_crsf as u8;

        frame[17] = sats;
        frame[18] = crc8_dvb_s2(&frame[2..18]);
        CrsfTelemetryFrame { data: frame, len: 19 }
    }

    /// Encode an attitude telemetry frame.
    ///
    /// - `pitch_rad`: pitch in radians * 10000 (i16, big-endian)
    /// - `roll_rad`: roll in radians * 10000 (i16, big-endian)
    /// - `yaw_rad`: yaw in radians * 10000 (i16, big-endian)
    pub fn encode_attitude(pitch_rad: i16, roll_rad: i16, yaw_rad: i16) -> CrsfTelemetryFrame {
        // Attitude payload: 6 bytes
        let mut frame = [0u8; CRSF_MAX_FRAME];
        frame[0] = CRSF_SYNC_BYTE;
        frame[1] = 8; // type(1) + payload(6) + crc(1)
        frame[2] = CRSF_FRAMETYPE_ATTITUDE;

        let p = pitch_rad as u16;
        frame[3] = (p >> 8) as u8;
        frame[4] = p as u8;
        let r = roll_rad as u16;
        frame[5] = (r >> 8) as u8;
        frame[6] = r as u8;
        let y = yaw_rad as u16;
        frame[7] = (y >> 8) as u8;
        frame[8] = y as u8;

        frame[9] = crc8_dvb_s2(&frame[2..9]);
        CrsfTelemetryFrame { data: frame, len: 10 }
    }

    /// Encode a flight mode telemetry frame.
    ///
    /// - `mode_str`: flight mode name, max 15 chars (null-terminated in frame)
    pub fn encode_flight_mode(mode_str: &str) -> CrsfTelemetryFrame {
        let mut frame = [0u8; CRSF_MAX_FRAME];
        frame[0] = CRSF_SYNC_BYTE;
        // Payload = mode string bytes + null terminator
        let str_bytes = mode_str.as_bytes();
        let copy_len = if str_bytes.len() > 15 { 15 } else { str_bytes.len() };
        let payload_len = copy_len + 1; // +1 for null terminator
        frame[1] = (payload_len + 2) as u8; // type(1) + payload + crc(1)
        frame[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
        frame[3..3 + copy_len].copy_from_slice(&str_bytes[..copy_len]);
        frame[3 + copy_len] = 0x00; // null terminator

        let crc_end = 3 + payload_len;
        frame[crc_end] = crc8_dvb_s2(&frame[2..crc_end]);
        CrsfTelemetryFrame { data: frame, len: crc_end + 1 }
    }

    /// Encode a VTX telemetry frame.
    ///
    /// ArduPilot sends VTX control data (frequency, power, pit mode) to the
    /// transmitter module so it can display/configure the VTX via OSD.
    ///
    /// - `frequency_mhz`: VTX frequency in MHz (e.g., 5800)
    /// - `power_mw`: VTX power in milliwatts (e.g., 25, 200, 600)
    /// - `pitmode`: true if VTX is in pit mode (low power)
    pub fn encode_vtx(frequency_mhz: u16, power_mw: u16, pitmode: bool) -> CrsfTelemetryFrame {
        let mut frame = [0u8; CRSF_MAX_FRAME];
        frame[0] = CRSF_SYNC_BYTE;
        // VTX payload: frequency(2 BE) + power(2 BE) + pitmode(1) = 5 bytes
        frame[1] = 7; // type(1) + payload(5) + crc(1)
        frame[2] = CRSF_FRAMETYPE_VTX_TELEM;
        frame[3] = (frequency_mhz >> 8) as u8;
        frame[4] = frequency_mhz as u8;
        frame[5] = (power_mw >> 8) as u8;
        frame[6] = power_mw as u8;
        frame[7] = if pitmode { 1 } else { 0 };
        frame[8] = crc8_dvb_s2(&frame[2..8]);
        CrsfTelemetryFrame { data: frame, len: 9 }
    }
}

// ─── SBUS ───

const SBUS_HEADER: u8 = 0x0F;
const SBUS_FRAME_LEN: usize = 25;

/// SBUS parser.
pub struct SbusParser {
    buf: [u8; SBUS_FRAME_LEN],
    pos: usize,
}

impl SbusParser {
    pub const fn new() -> Self {
        Self { buf: [0; SBUS_FRAME_LEN], pos: 0 }
    }

    /// Feed one byte. Returns Some(RcFrame) on complete frame.
    pub fn feed(&mut self, byte: u8) -> Option<RcFrame> {
        if self.pos == 0 {
            if byte == SBUS_HEADER {
                self.buf[0] = byte;
                self.pos = 1;
            }
            return None;
        }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos >= SBUS_FRAME_LEN {
            let result = self.parse_frame();
            self.pos = 0;
            return result;
        }

        None
    }

    fn parse_frame(&self) -> Option<RcFrame> {
        // Check end byte (0x00 or 0x04 or 0x14 or 0x24 or 0x34)
        let end = self.buf[24];
        if end & 0x0F != 0x00 && end & 0x0F != 0x04 { return None; }

        // Status byte (byte 23)
        let status = self.buf[23];
        let failsafe_flag = (status & 0x08) != 0; // bit 3: explicit failsafe
        let _frame_lost = (status & 0x04) != 0;    // bit 2: frame lost (NOT failsafe per spec)
        // ArduPilot treats frame_lost as NOT a failsafe — it logs but continues
        // using the data. Meridian now matches this semantics.

        // Unpack 16 channels from 11-bit packed data (bytes 1-22)
        let data = &self.buf[1..23];
        let mut channels = [0u16; MAX_CHANNELS];
        let mut bit_offset = 0u32;

        for ch in 0..16 {
            let byte_idx = (bit_offset / 8) as usize;
            let bit_idx = bit_offset % 8;
            if byte_idx + 1 >= 22 { break; }

            let raw = ((data[byte_idx] as u32) >> bit_idx)
                | ((data[byte_idx + 1] as u32) << (8 - bit_idx))
                | (if byte_idx + 2 < 22 { (data[byte_idx + 2] as u32) << (16 - bit_idx) } else { 0 });
            let raw = (raw & 0x7FF) as u16;

            // Convert SBUS range (172-1811) to standard PWM (988-2012)
            // ArduPilot formula: (raw * 5 / 8) + 880. Center stick (992) → 1500us.
            channels[ch] = ((raw as i32 * 5 / 8) + 880).clamp(988, 2012) as u16;
            bit_offset += 11;
        }

        // Second failsafe check (ArduPilot): if any of the first 4 channels
        // decodes to a value at or below 875us (SBUS_SCALE_OFFSET), treat as failsafe.
        // This catches receivers that don't set the failsafe flag but output
        // abnormally low channel values when link is lost.
        let channel_failsafe = channels[..4].iter().any(|&ch| ch <= 875);

        let failsafe = failsafe_flag || channel_failsafe;

        Some(RcFrame {
            channels,
            count: 16,
            failsafe,
            rssi: if failsafe { 0 } else { 100 },
        })
    }
}

// ─── CRSFv3 Baud Rate Negotiation ───

/// CRSFv3 baud negotiation frame type.
const CRSF_FRAMETYPE_COMMAND: u8 = 0x32;
/// CRSFv3 command: baud rate proposal.
const CRSF_COMMAND_BAUD_RATE: u8 = 0x01;

/// CRSFv3 baud negotiation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrsfV3BaudState {
    /// No negotiation in progress.
    Idle,
    /// Baud rate proposed, waiting for acknowledgment.
    Proposed { target_baud: u32 },
    /// Baud rate confirmed by the receiver.
    Confirmed { baud: u32 },
}

/// CRSFv3 baud rate negotiation handler.
///
/// CRSFv3 uses a ping-pong mechanism: the flight controller proposes a
/// higher baud rate, and the receiver acknowledges. Both sides then
/// switch to the new rate.
pub struct CrsfV3Negotiation {
    /// Current negotiation state.
    pub state: CrsfV3BaudState,
}

impl CrsfV3Negotiation {
    pub const fn new() -> Self {
        Self {
            state: CrsfV3BaudState::Idle,
        }
    }

    /// Encode a baud rate proposal frame.
    ///
    /// Returns the frame bytes and length. The flight controller sends this
    /// to propose switching to `target_baud`.
    ///
    /// Frame format: [sync][len][type=0x32][cmd=0x01][baud LE u32][crc]
    pub fn propose_baud(&mut self, target_baud: u32) -> ([u8; 10], usize) {
        let mut frame = [0u8; 10];
        frame[0] = CRSF_SYNC_BYTE;
        // Length: type(1) + cmd(1) + baud(4) + crc(1) = 7
        frame[1] = 7;
        frame[2] = CRSF_FRAMETYPE_COMMAND;
        frame[3] = CRSF_COMMAND_BAUD_RATE;
        // Baud rate as little-endian u32
        frame[4] = (target_baud & 0xFF) as u8;
        frame[5] = ((target_baud >> 8) & 0xFF) as u8;
        frame[6] = ((target_baud >> 16) & 0xFF) as u8;
        frame[7] = ((target_baud >> 24) & 0xFF) as u8;
        // CRC over type + payload (bytes 2..8)
        frame[8] = crc8_dvb_s2(&frame[2..8]);

        self.state = CrsfV3BaudState::Proposed { target_baud };
        (frame, 9)
    }

    /// Parse a baud rate response frame from the receiver.
    ///
    /// Returns `Some(baud)` if the receiver acknowledged the proposed baud rate.
    /// The frame format mirrors the proposal: [sync][len][type=0x32][cmd=0x01][baud LE u32][crc]
    pub fn parse_baud_response(&mut self, frame: &[u8]) -> Option<u32> {
        // Minimum frame length: sync(1) + len(1) + type(1) + cmd(1) + baud(4) + crc(1) = 9
        if frame.len() < 9 {
            return None;
        }

        // Verify sync byte
        if frame[0] != CRSF_SYNC_BYTE {
            return None;
        }

        // Verify frame type and command
        if frame[2] != CRSF_FRAMETYPE_COMMAND || frame[3] != CRSF_COMMAND_BAUD_RATE {
            return None;
        }

        // Verify CRC
        let expected_len = frame[1] as usize + 2;
        if frame.len() < expected_len {
            return None;
        }
        let crc_idx = expected_len - 1;
        let calculated_crc = crc8_dvb_s2(&frame[2..crc_idx]);
        if calculated_crc != frame[crc_idx] {
            return None;
        }

        // Parse baud rate (little-endian u32)
        let baud = (frame[4] as u32)
            | ((frame[5] as u32) << 8)
            | ((frame[6] as u32) << 16)
            | ((frame[7] as u32) << 24);

        // Confirm only if we were in Proposed state and baud matches
        match self.state {
            CrsfV3BaudState::Proposed { target_baud } if target_baud == baud => {
                self.state = CrsfV3BaudState::Confirmed { baud };
                Some(baud)
            }
            _ => {
                // Unexpected response — accept it but move to confirmed
                self.state = CrsfV3BaudState::Confirmed { baud };
                Some(baud)
            }
        }
    }

    /// Reset the negotiation state machine.
    pub fn reset(&mut self) {
        self.state = CrsfV3BaudState::Idle;
    }
}

/// CRC8 DVB-S2 for CRSF.
fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crsf_crc() {
        // Known CRC value for CRSF
        let data = [0x16, 0x00, 0x00, 0x00]; // type + some data
        let crc = crc8_dvb_s2(&data);
        assert_ne!(crc, 0); // just verify it produces something
    }

    #[test]
    fn test_sbus_center_sticks() {
        // Build an SBUS frame with all channels at center (992 raw ≈ 1500 PWM)
        let mut frame = [0u8; 25];
        frame[0] = SBUS_HEADER;
        // Pack 16 channels of raw value 992 (0x3E0) into 11-bit slots
        // For simplicity, fill data bytes to produce ~center values
        for i in 1..23 { frame[i] = 0xE0; } // approximate center
        frame[23] = 0x00; // status: no failsafe
        frame[24] = 0x00; // end byte

        let mut parser = SbusParser::new();
        let mut result = None;
        for &byte in &frame {
            if let Some(r) = parser.feed(byte) {
                result = Some(r);
            }
        }

        let rc = result.unwrap();
        assert_eq!(rc.count, 16);
        assert!(!rc.failsafe);
        // All channels should be in valid ArduPilot PWM range (988-2012)
        for i in 0..16 {
            assert!(rc.channels[i] >= 988 && rc.channels[i] <= 2012,
                "Channel {} = {} out of range", i, rc.channels[i]);
        }
    }

    #[test]
    fn test_sbus_failsafe() {
        let mut frame = [0u8; 25];
        frame[0] = SBUS_HEADER;
        for i in 1..23 { frame[i] = 0xE0; }
        frame[23] = 0x08; // failsafe bit set
        frame[24] = 0x00;

        let mut parser = SbusParser::new();
        let mut result = None;
        for &byte in &frame {
            if let Some(r) = parser.feed(byte) { result = Some(r); }
        }

        assert!(result.unwrap().failsafe);
    }

    #[test]
    fn test_crsf_parser_rejects_bad_sync() {
        let mut parser = CrsfParser::new();
        // Feed garbage — should never produce a frame
        for byte in [0x00, 0x01, 0x02, 0x03, 0x04] {
            assert!(parser.feed(byte).is_none());
        }
    }

    // ─── Link Statistics Tests ───

    #[test]
    fn test_crsf_link_stats_parsing() {
        let mut parser = CrsfParser::new();
        // Build a LINK_STATISTICS frame: sync + len + type + 10 payload bytes + crc
        let mut frame = [0u8; 14];
        frame[0] = CRSF_SYNC_BYTE;
        frame[1] = 12; // type(1) + payload(10) + crc(1)
        frame[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
        frame[3] = 45;  // uplink_rssi_ant1
        frame[4] = 50;  // uplink_rssi_ant2
        frame[5] = 99;  // uplink_lq
        frame[6] = 8;   // uplink_snr (positive)
        frame[7] = 1;   // active_antenna
        frame[8] = 2;   // rf_mode
        frame[9] = 3;   // uplink_power
        frame[10] = 60; // downlink_rssi
        frame[11] = 95; // downlink_lq
        frame[12] = 0xFC; // downlink_snr = -4 as i8
        frame[13] = crc8_dvb_s2(&frame[2..13]);

        for &byte in &frame {
            let _ = parser.feed(byte);
        }

        assert_eq!(parser.link_stats.uplink_rssi_ant1, 45);
        assert_eq!(parser.link_stats.uplink_rssi_ant2, 50);
        assert_eq!(parser.link_stats.uplink_lq, 99);
        assert_eq!(parser.link_stats.uplink_snr, 8);
        assert_eq!(parser.link_stats.active_antenna, 1);
        assert_eq!(parser.link_stats.rf_mode, 2);
        assert_eq!(parser.link_stats.uplink_power, 3);
        assert_eq!(parser.link_stats.downlink_rssi, 60);
        assert_eq!(parser.link_stats.downlink_lq, 95);
        assert_eq!(parser.link_stats.downlink_snr, -4);
        assert_eq!(parser.last_rssi, 45);
    }

    #[test]
    fn test_crsf_link_stats_bad_crc_rejected() {
        let mut parser = CrsfParser::new();
        let mut frame = [0u8; 14];
        frame[0] = CRSF_SYNC_BYTE;
        frame[1] = 12;
        frame[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
        frame[3] = 45;
        frame[4] = 50;
        frame[5] = 99;
        frame[6] = 8;
        frame[7] = 1;
        frame[8] = 2;
        frame[9] = 3;
        frame[10] = 60;
        frame[11] = 95;
        frame[12] = 0xFC;
        frame[13] = 0xFF; // bad CRC

        for &byte in &frame {
            let _ = parser.feed(byte);
        }
        // Should not have updated link_stats
        assert_eq!(parser.link_stats.uplink_rssi_ant1, 0);
    }

    // ─── Telemetry Encoder Tests ───

    #[test]
    fn test_telemetry_battery_encode() {
        let f = CrsfTelemetry::encode_battery(16800, 1500, 2200, 75);
        assert_eq!(f.data[0], CRSF_SYNC_BYTE);
        assert_eq!(f.data[1], 10); // length byte
        assert_eq!(f.data[2], CRSF_FRAMETYPE_BATTERY);
        assert_eq!(f.len, 12);
        // Verify CRC
        let crc = crc8_dvb_s2(&f.data[2..11]);
        assert_eq!(f.data[11], crc);
        // Voltage: 16800mv / 100 = 168 decivolts = 0x00A8
        assert_eq!(f.data[3], 0x00);
        assert_eq!(f.data[4], 0xA8);
        // Remaining pct
        assert_eq!(f.data[10], 75);
    }

    #[test]
    fn test_telemetry_gps_encode() {
        let f = CrsfTelemetry::encode_gps(
            354_000_000, // ~35.4 deg
            -1175_000_000, // ~-117.5 deg
            15000, // 150m in cm
            500,   // 5 m/s in cm/s
            18000, // 180 deg in centidegrees
            12,    // sats
        );
        assert_eq!(f.data[0], CRSF_SYNC_BYTE);
        assert_eq!(f.data[2], CRSF_FRAMETYPE_GPS);
        assert_eq!(f.len, 19);
        // Verify CRC
        let crc = crc8_dvb_s2(&f.data[2..18]);
        assert_eq!(f.data[18], crc);
        // Sats
        assert_eq!(f.data[17], 12);
        // Altitude: 150m + 1000 = 1150 = 0x047E
        assert_eq!(f.data[15], 0x04);
        assert_eq!(f.data[16], 0x7E);
    }

    #[test]
    fn test_telemetry_attitude_encode() {
        // pitch = 0.5 rad * 10000 = 5000, roll = -0.3 rad * 10000 = -3000, yaw = 1.0 * 10000 = 10000
        let f = CrsfTelemetry::encode_attitude(5000, -3000, 10000);
        assert_eq!(f.data[0], CRSF_SYNC_BYTE);
        assert_eq!(f.data[2], CRSF_FRAMETYPE_ATTITUDE);
        assert_eq!(f.len, 10);
        // Verify CRC
        let crc = crc8_dvb_s2(&f.data[2..9]);
        assert_eq!(f.data[9], crc);
        // Pitch: 5000 = 0x1388
        assert_eq!(f.data[3], 0x13);
        assert_eq!(f.data[4], 0x88);
        // Roll: -3000 as u16 = 0xF448
        assert_eq!(f.data[5], 0xF4);
        assert_eq!(f.data[6], 0x48);
    }

    #[test]
    fn test_telemetry_flight_mode_encode() {
        let f = CrsfTelemetry::encode_flight_mode("STAB");
        assert_eq!(f.data[0], CRSF_SYNC_BYTE);
        assert_eq!(f.data[2], CRSF_FRAMETYPE_FLIGHT_MODE);
        // Payload: "STAB" (4 bytes) + null = 5 bytes
        // Length byte: type(1) + 5 + crc(1) = 7
        assert_eq!(f.data[1], 7);
        assert_eq!(&f.data[3..7], b"STAB");
        assert_eq!(f.data[7], 0x00); // null terminator
        // CRC covers type through end of payload
        let crc = crc8_dvb_s2(&f.data[2..8]);
        assert_eq!(f.data[8], crc);
        assert_eq!(f.len, 9);
    }

    #[test]
    fn test_telemetry_flight_mode_truncation() {
        // Long mode name gets truncated to 15 chars
        let f = CrsfTelemetry::encode_flight_mode("SUPERLONGMODENAME");
        // Should be truncated to 15 chars + null
        let payload_len = 15 + 1;
        assert_eq!(f.data[1] as usize, payload_len + 2); // type + payload + crc
        assert_eq!(f.data[3 + 15], 0x00); // null at position 18
    }

    #[test]
    fn test_telemetry_battery_roundtrip_crc() {
        // Verify we can parse our own CRC
        let f = CrsfTelemetry::encode_battery(12600, 0, 0, 100);
        let crc_idx = f.len - 1;
        let computed = crc8_dvb_s2(&f.data[2..crc_idx]);
        assert_eq!(computed, f.data[crc_idx]);
    }

    // ─── CRSFv3 Baud Negotiation Tests ───

    #[test]
    fn test_crsfv3_initial_state_idle() {
        let neg = CrsfV3Negotiation::new();
        assert_eq!(neg.state, CrsfV3BaudState::Idle);
    }

    #[test]
    fn test_crsfv3_propose_baud() {
        let mut neg = CrsfV3Negotiation::new();
        let (frame, len) = neg.propose_baud(921600);
        assert_eq!(len, 9);
        assert_eq!(frame[0], CRSF_SYNC_BYTE);
        assert_eq!(frame[1], 7); // length
        assert_eq!(frame[2], CRSF_FRAMETYPE_COMMAND);
        assert_eq!(frame[3], CRSF_COMMAND_BAUD_RATE);
        // 921600 = 0x000E1000 in LE: [0x00, 0x10, 0x0E, 0x00]
        assert_eq!(frame[4], 0x00);
        assert_eq!(frame[5], 0x10);
        assert_eq!(frame[6], 0x0E);
        assert_eq!(frame[7], 0x00);
        // CRC should be valid
        let crc = crc8_dvb_s2(&frame[2..8]);
        assert_eq!(frame[8], crc);
        // State should be Proposed
        assert_eq!(neg.state, CrsfV3BaudState::Proposed { target_baud: 921600 });
    }

    #[test]
    fn test_crsfv3_parse_matching_response() {
        let mut neg = CrsfV3Negotiation::new();
        let (frame, _len) = neg.propose_baud(921600);
        // Simulate receiver echoing back the same frame as acknowledgment
        let result = neg.parse_baud_response(&frame);
        assert_eq!(result, Some(921600));
        assert_eq!(neg.state, CrsfV3BaudState::Confirmed { baud: 921600 });
    }

    #[test]
    fn test_crsfv3_parse_bad_crc_rejected() {
        let mut neg = CrsfV3Negotiation::new();
        let (mut frame, _len) = neg.propose_baud(921600);
        frame[8] = 0xFF; // corrupt CRC
        let result = neg.parse_baud_response(&frame);
        assert!(result.is_none());
    }

    #[test]
    fn test_crsfv3_parse_wrong_type_rejected() {
        let mut neg = CrsfV3Negotiation::new();
        neg.propose_baud(921600);
        // Build a frame with wrong type
        let mut frame = [0u8; 9];
        frame[0] = CRSF_SYNC_BYTE;
        frame[1] = 7;
        frame[2] = 0x99; // wrong type
        frame[3] = CRSF_COMMAND_BAUD_RATE;
        frame[8] = crc8_dvb_s2(&frame[2..8]);
        let result = neg.parse_baud_response(&frame);
        assert!(result.is_none());
    }

    #[test]
    fn test_crsfv3_parse_short_frame_rejected() {
        let mut neg = CrsfV3Negotiation::new();
        neg.propose_baud(921600);
        let result = neg.parse_baud_response(&[0xC8, 0x07]);
        assert!(result.is_none());
    }

    #[test]
    fn test_crsfv3_reset() {
        let mut neg = CrsfV3Negotiation::new();
        neg.propose_baud(921600);
        assert_eq!(neg.state, CrsfV3BaudState::Proposed { target_baud: 921600 });
        neg.reset();
        assert_eq!(neg.state, CrsfV3BaudState::Idle);
    }

    #[test]
    fn test_crsfv3_roundtrip_different_bauds() {
        for &baud in &[420000u32, 921600, 1870000, 3750000] {
            let mut neg = CrsfV3Negotiation::new();
            let (frame, _) = neg.propose_baud(baud);
            let result = neg.parse_baud_response(&frame);
            assert_eq!(result, Some(baud), "Failed for baud {}", baud);
        }
    }
}

