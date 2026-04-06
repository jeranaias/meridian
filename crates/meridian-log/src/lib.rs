#![no_std]

//! Structured binary logging with replay capability.
//!
//! Self-describing format: schema header + timestamped entries.
//! Zero-copy serialization via postcard on embedded, serde for std.

use meridian_types::time::Instant;

/// Log message type identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LogMsgId(pub u16);

/// Log severity levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Debug = 0,
    Info = 1,
    Warning = 2,
    Error = 3,
    Critical = 4,
}

/// A single log entry header.
#[derive(Debug, Clone, Copy)]
pub struct LogHeader {
    pub msg_id: LogMsgId,
    pub timestamp: Instant,
    pub size: u16,
}

/// Schema entry describing a logged message type.
#[derive(Debug, Clone)]
pub struct LogSchema {
    pub msg_id: LogMsgId,
    pub name: heapless::String<32>,
    pub fields: heapless::Vec<LogField, 16>,
}

/// A field in a log schema.
#[derive(Debug, Clone)]
pub struct LogField {
    pub name: heapless::String<16>,
    pub field_type: LogFieldType,
}

/// Supported field types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogFieldType {
    F32,
    I32,
    U32,
    U16,
    U8,
    Bool,
}

impl LogFieldType {
    pub fn size(&self) -> usize {
        match self {
            Self::F32 | Self::I32 | Self::U32 => 4,
            Self::U16 => 2,
            Self::U8 | Self::Bool => 1,
        }
    }
}

/// Trait for types that can be logged.
pub trait LogMessage {
    fn msg_id() -> LogMsgId;
    fn schema() -> LogSchema;
    fn serialize_into(&self, buf: &mut [u8]) -> usize;
}

/// Ring buffer for log entries on embedded (fixed-size, overwrites oldest).
pub struct LogBuffer<const N: usize> {
    buf: [u8; N],
    write_pos: usize,
    entry_count: u32,
    overflows: u32,
}

impl<const N: usize> LogBuffer<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; N],
            write_pos: 0,
            entry_count: 0,
            overflows: 0,
        }
    }

    /// Write a log entry. Returns true if written, false if buffer full.
    pub fn write(&mut self, msg_id: LogMsgId, timestamp: Instant, data: &[u8]) -> bool {
        let header_size = 8; // 2 (msg_id) + 4 (timestamp_us low) + 2 (size)
        let total = header_size + data.len();
        if total > N {
            self.overflows += 1;
            return false;
        }

        // Wrap around if needed
        if self.write_pos + total > N {
            self.write_pos = 0;
            self.overflows += 1;
        }

        // Write header
        let pos = self.write_pos;
        self.buf[pos..pos+2].copy_from_slice(&msg_id.0.to_le_bytes());
        let ts = (timestamp.as_micros() & 0xFFFF_FFFF) as u32;
        self.buf[pos+2..pos+6].copy_from_slice(&ts.to_le_bytes());
        self.buf[pos+6..pos+8].copy_from_slice(&(data.len() as u16).to_le_bytes());

        // Write data
        self.buf[pos+8..pos+8+data.len()].copy_from_slice(data);

        self.write_pos += total;
        self.entry_count += 1;
        true
    }

    pub fn entry_count(&self) -> u32 { self.entry_count }
    pub fn overflows(&self) -> u32 { self.overflows }
    pub fn used_bytes(&self) -> usize { self.write_pos }
    pub fn capacity(&self) -> usize { N }
}

/// Text status message (like ArduPilot's STATUSTEXT).
#[derive(Debug, Clone)]
pub struct StatusMessage {
    pub level: LogLevel,
    pub text: heapless::String<64>,
}

impl StatusMessage {
    pub fn new(level: LogLevel, text: &str) -> Self {
        let mut s = heapless::String::new();
        let _ = s.push_str(text);
        Self { level, text: s }
    }
}

// ─── Concrete log message types for flight data ───
// Source: ArduPilot AP_Logger/LogStructure.h — minimum set for post-flight analysis.

/// FMT message — self-describing format header.
/// Written once per message type at log start.
/// Source: AP_Logger FMT message, enables log viewers to parse without hardcoded schemas.
pub const LOG_FMT_ID: u16 = 128;

/// FMT entry describing a log message type.
#[derive(Debug, Clone)]
pub struct FmtEntry {
    pub msg_type: u8,
    pub length: u8,
    pub name: [u8; 4],
    pub format: heapless::Vec<u8, 16>,
    pub labels: heapless::Vec<heapless::String<16>, 16>,
}

impl FmtEntry {
    pub fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 89 { return 0; }
        buf[0] = self.msg_type;
        buf[1] = self.length;
        buf[2..6].copy_from_slice(&self.name);
        // format string: 16 bytes
        let flen = self.format.len().min(16);
        buf[6..6+flen].copy_from_slice(&self.format[..flen]);
        // labels: comma-separated, 64 bytes
        let mut pos = 22;
        for (i, label) in self.labels.iter().enumerate() {
            if i > 0 && pos < 86 { buf[pos] = b','; pos += 1; }
            let bytes = label.as_bytes();
            let len = bytes.len().min(86 - pos);
            buf[pos..pos+len].copy_from_slice(&bytes[..len]);
            pos += len;
        }
        89
    }
}

// Log message IDs — match ArduPilot numbering for compatibility
pub const LOG_ATT: u16 = 1;   // Attitude
pub const LOG_IMU: u16 = 2;   // IMU
pub const LOG_GPS: u16 = 3;   // GPS
pub const LOG_BARO: u16 = 4;  // Barometer
pub const LOG_BAT: u16 = 5;   // Battery
pub const LOG_MODE: u16 = 6;  // Mode change
pub const LOG_ARM: u16 = 7;   // Arm/disarm event
pub const LOG_ERR: u16 = 8;   // Error/failsafe event
pub const LOG_POS: u16 = 9;   // Position
pub const LOG_RATE: u16 = 10; // Rate controller output
pub const LOG_PIDR: u16 = 11; // PID roll
pub const LOG_PIDP: u16 = 12; // PID pitch
pub const LOG_PIDY: u16 = 13; // PID yaw
pub const LOG_RCIN: u16 = 14; // RC input
pub const LOG_RCOU: u16 = 15; // RC output (motor PWM)
pub const LOG_EKF: u16 = 16;  // EKF state
pub const LOG_OVERRUN: u16 = 17; // Scheduler loop overrun

/// Attitude log entry (ATT).
#[derive(Debug, Clone, Copy)]
pub struct LogAtt {
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub des_roll_deg: f32,
    pub des_pitch_deg: f32,
    pub des_yaw_deg: f32,
}

impl LogMessage for LogAtt {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_ATT) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        let names = ["Roll", "Pitch", "Yaw", "DesRoll", "DesPitch", "DesYaw"];
        for name in &names {
            let mut n = heapless::String::new();
            let _ = n.push_str(name);
            let _ = fields.push(LogField { name: n, field_type: LogFieldType::F32 });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("ATT");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 24 { return 0; }
        buf[0..4].copy_from_slice(&self.roll_deg.to_le_bytes());
        buf[4..8].copy_from_slice(&self.pitch_deg.to_le_bytes());
        buf[8..12].copy_from_slice(&self.yaw_deg.to_le_bytes());
        buf[12..16].copy_from_slice(&self.des_roll_deg.to_le_bytes());
        buf[16..20].copy_from_slice(&self.des_pitch_deg.to_le_bytes());
        buf[20..24].copy_from_slice(&self.des_yaw_deg.to_le_bytes());
        24
    }
}

/// IMU log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogImu {
    pub gyr_x: f32,
    pub gyr_y: f32,
    pub gyr_z: f32,
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
}

impl LogMessage for LogImu {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_IMU) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        for name in &["GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"] {
            let mut n = heapless::String::new();
            let _ = n.push_str(name);
            let _ = fields.push(LogField { name: n, field_type: LogFieldType::F32 });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("IMU");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 24 { return 0; }
        buf[0..4].copy_from_slice(&self.gyr_x.to_le_bytes());
        buf[4..8].copy_from_slice(&self.gyr_y.to_le_bytes());
        buf[8..12].copy_from_slice(&self.gyr_z.to_le_bytes());
        buf[12..16].copy_from_slice(&self.acc_x.to_le_bytes());
        buf[16..20].copy_from_slice(&self.acc_y.to_le_bytes());
        buf[20..24].copy_from_slice(&self.acc_z.to_le_bytes());
        24
    }
}

/// GPS log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogGps {
    pub fix_type: u8,
    pub num_sats: u8,
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_cm: i32,
    pub speed_cm: u16,
    pub hacc_cm: u16,
}

impl LogMessage for LogGps {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_GPS) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        let entries: &[(&str, LogFieldType)] = &[
            ("Fix", LogFieldType::U8), ("NSat", LogFieldType::U8),
            ("Lat", LogFieldType::I32), ("Lon", LogFieldType::I32),
            ("Alt", LogFieldType::I32), ("Spd", LogFieldType::U16),
            ("HAcc", LogFieldType::U16),
        ];
        for (n, t) in entries {
            let mut name = heapless::String::new();
            let _ = name.push_str(n);
            let _ = fields.push(LogField { name, field_type: *t });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("GPS");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 18 { return 0; }
        buf[0] = self.fix_type;
        buf[1] = self.num_sats;
        buf[2..6].copy_from_slice(&self.lat_e7.to_le_bytes());
        buf[6..10].copy_from_slice(&self.lon_e7.to_le_bytes());
        buf[10..14].copy_from_slice(&self.alt_cm.to_le_bytes());
        buf[14..16].copy_from_slice(&self.speed_cm.to_le_bytes());
        buf[16..18].copy_from_slice(&self.hacc_cm.to_le_bytes());
        18
    }
}

/// Barometer log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogBaro {
    pub alt_m: f32,
    pub press_pa: f32,
    pub temp_c: f32,
}

impl LogMessage for LogBaro {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_BARO) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        for name in &["Alt", "Press", "Temp"] {
            let mut n = heapless::String::new();
            let _ = n.push_str(name);
            let _ = fields.push(LogField { name: n, field_type: LogFieldType::F32 });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("BARO");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 12 { return 0; }
        buf[0..4].copy_from_slice(&self.alt_m.to_le_bytes());
        buf[4..8].copy_from_slice(&self.press_pa.to_le_bytes());
        buf[8..12].copy_from_slice(&self.temp_c.to_le_bytes());
        12
    }
}

/// Battery log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogBat {
    pub voltage_v: f32,
    pub current_a: f32,
    pub consumed_mah: f32,
    pub remaining_pct: u8,
}

impl LogMessage for LogBat {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_BAT) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        let entries: &[(&str, LogFieldType)] = &[
            ("Volt", LogFieldType::F32), ("Curr", LogFieldType::F32),
            ("CurrTot", LogFieldType::F32), ("Rem", LogFieldType::U8),
        ];
        for (n, t) in entries {
            let mut name = heapless::String::new();
            let _ = name.push_str(n);
            let _ = fields.push(LogField { name, field_type: *t });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("BAT");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 13 { return 0; }
        buf[0..4].copy_from_slice(&self.voltage_v.to_le_bytes());
        buf[4..8].copy_from_slice(&self.current_a.to_le_bytes());
        buf[8..12].copy_from_slice(&self.consumed_mah.to_le_bytes());
        buf[12] = self.remaining_pct;
        13
    }
}

/// Mode change log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogMode {
    pub mode_num: u8,
    pub reason: u8, // 0=RC, 1=GCS, 2=failsafe, 3=auto
}

impl LogMessage for LogMode {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_MODE) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        for (n, t) in &[("Mode", LogFieldType::U8), ("Reason", LogFieldType::U8)] {
            let mut name = heapless::String::new();
            let _ = name.push_str(n);
            let _ = fields.push(LogField { name, field_type: *t });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("MODE");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 2 { return 0; }
        buf[0] = self.mode_num;
        buf[1] = self.reason;
        2
    }
}

/// Arm/disarm event log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogArm {
    pub armed: bool,
}

impl LogMessage for LogArm {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_ARM) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        let mut n = heapless::String::new();
        let _ = n.push_str("Armed");
        let _ = fields.push(LogField { name: n, field_type: LogFieldType::Bool });
        let mut name = heapless::String::new();
        let _ = name.push_str("ARM");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.is_empty() { return 0; }
        buf[0] = self.armed as u8;
        1
    }
}

/// Error/failsafe event log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogErr {
    pub subsys: u8,  // 1=Radio, 2=GPS, 3=GCS, 4=Fence, 5=Battery, 6=EKF, 7=Motor, 8=Crash
    pub code: u8,    // 0=resolved, 1=triggered, 2=critical
}

impl LogMessage for LogErr {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_ERR) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        for (n, t) in &[("Subsys", LogFieldType::U8), ("Code", LogFieldType::U8)] {
            let mut name = heapless::String::new();
            let _ = name.push_str(n);
            let _ = fields.push(LogField { name, field_type: *t });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("ERR");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 2 { return 0; }
        buf[0] = self.subsys;
        buf[1] = self.code;
        2
    }
}

/// EKF state log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogEkf {
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    pub vel_n: f32,
    pub vel_e: f32,
    pub vel_d: f32,
    pub pos_n: f32,
    pub pos_e: f32,
    pub pos_d: f32,
    pub gyro_bias_x: f32,
    pub gyro_bias_y: f32,
    pub gyro_bias_z: f32,
}

impl LogMessage for LogEkf {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_EKF) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        let names = ["Roll","Pitch","Yaw","VN","VE","VD","PN","PE","PD","GBX","GBY","GBZ"];
        for name in &names {
            let mut n = heapless::String::new();
            let _ = n.push_str(name);
            let _ = fields.push(LogField { name: n, field_type: LogFieldType::F32 });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("EKF");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 48 { return 0; }
        let vals = [self.roll_deg, self.pitch_deg, self.yaw_deg,
                    self.vel_n, self.vel_e, self.vel_d,
                    self.pos_n, self.pos_e, self.pos_d,
                    self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z];
        for (i, v) in vals.iter().enumerate() {
            buf[i*4..(i+1)*4].copy_from_slice(&v.to_le_bytes());
        }
        48
    }
}

/// Motor output log entry.
#[derive(Debug, Clone, Copy)]
pub struct LogRcOut {
    pub motors: [u16; 8], // PWM values for up to 8 motors
    pub count: u8,
}

impl LogMessage for LogRcOut {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_RCOU) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        for i in 0..8u8 {
            let mut n = heapless::String::new();
            let _ = n.push_str("C");
            // manually add digit
            let _ = n.push(char::from(b'1' + i));
            let _ = fields.push(LogField { name: n, field_type: LogFieldType::U16 });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("RCOU");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 17 { return 0; }
        for i in 0..8 {
            buf[i*2..(i+1)*2].copy_from_slice(&self.motors[i].to_le_bytes());
        }
        buf[16] = self.count;
        17
    }
}

/// Scheduler loop overrun log entry.
/// Logged when a task exceeds its time budget.
#[derive(Debug, Clone, Copy)]
pub struct LogOverrun {
    /// Task/rate-group identifier.
    pub task_id: u8,
    /// Actual execution time (microseconds).
    pub actual_us: u32,
    /// Budgeted execution time (microseconds).
    pub budget_us: u32,
}

impl LogMessage for LogOverrun {
    fn msg_id() -> LogMsgId { LogMsgId(LOG_OVERRUN) }
    fn schema() -> LogSchema {
        let mut fields = heapless::Vec::new();
        let entries: &[(&str, LogFieldType)] = &[
            ("Task", LogFieldType::U8),
            ("Actual", LogFieldType::U32),
            ("Budget", LogFieldType::U32),
        ];
        for (n, t) in entries {
            let mut name = heapless::String::new();
            let _ = name.push_str(n);
            let _ = fields.push(LogField { name, field_type: *t });
        }
        let mut name = heapless::String::new();
        let _ = name.push_str("OVRN");
        LogSchema { msg_id: Self::msg_id(), name, fields }
    }
    fn serialize_into(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 9 { return 0; }
        buf[0] = self.task_id;
        buf[1..5].copy_from_slice(&self.actual_us.to_le_bytes());
        buf[5..9].copy_from_slice(&self.budget_us.to_le_bytes());
        9
    }
}

/// Flight logger that manages the log buffer and message rate limiting.
pub struct FlightLogger<const N: usize> {
    pub buffer: LogBuffer<N>,
    schemas_written: bool,
    imu_divider: u16,
    imu_counter: u16,
    att_divider: u16,
    att_counter: u16,
}

impl<const N: usize> FlightLogger<N> {
    pub const fn new() -> Self {
        Self {
            buffer: LogBuffer::new(),
            schemas_written: false,
            imu_divider: 4,   // log IMU at 250Hz (from 1kHz)
            imu_counter: 0,
            att_divider: 10,  // log ATT at 100Hz
            att_counter: 0,
        }
    }

    /// Write FMT headers for all message types. Call once at log start.
    pub fn write_schemas(&mut self) {
        if self.schemas_written { return; }
        let schemas: [LogSchema; 11] = [
            LogAtt::schema(), LogImu::schema(), LogGps::schema(),
            LogBaro::schema(), LogBat::schema(), LogMode::schema(),
            LogArm::schema(), LogErr::schema(), LogEkf::schema(),
            LogRcOut::schema(), LogOverrun::schema(),
        ];
        let mut fmt_buf = [0u8; 89];
        for schema in &schemas {
            let entry = FmtEntry {
                msg_type: schema.msg_id.0 as u8,
                length: schema.fields.iter().map(|f| f.field_type.size() as u8).sum::<u8>() + 8,
                name: {
                    let mut n = [0u8; 4];
                    let bytes = schema.name.as_bytes();
                    let len = bytes.len().min(4);
                    n[..len].copy_from_slice(&bytes[..len]);
                    n
                },
                format: {
                    let mut f = heapless::Vec::new();
                    for field in &schema.fields {
                        let _ = f.push(match field.field_type {
                            LogFieldType::F32 => b'f',
                            LogFieldType::I32 => b'i',
                            LogFieldType::U32 => b'I',
                            LogFieldType::U16 => b'H',
                            LogFieldType::U8 => b'B',
                            LogFieldType::Bool => b'b',
                        });
                    }
                    f
                },
                labels: schema.fields.iter().map(|f| f.name.clone()).collect(),
            };
            let sz = entry.serialize_into(&mut fmt_buf);
            self.buffer.write(LogMsgId(LOG_FMT_ID), Instant::ZERO, &fmt_buf[..sz]);
        }
        self.schemas_written = true;
    }

    /// Log an IMU sample (rate-limited).
    pub fn log_imu(&mut self, imu: &LogImu, timestamp: Instant) -> bool {
        self.imu_counter += 1;
        if self.imu_counter < self.imu_divider { return false; }
        self.imu_counter = 0;
        let mut buf = [0u8; 24];
        let sz = imu.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_IMU), timestamp, &buf[..sz])
    }

    /// Log an attitude sample (rate-limited).
    pub fn log_att(&mut self, att: &LogAtt, timestamp: Instant) -> bool {
        self.att_counter += 1;
        if self.att_counter < self.att_divider { return false; }
        self.att_counter = 0;
        let mut buf = [0u8; 24];
        let sz = att.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_ATT), timestamp, &buf[..sz])
    }

    /// Log a GPS sample (every call — GPS is already low rate).
    pub fn log_gps(&mut self, gps: &LogGps, timestamp: Instant) -> bool {
        let mut buf = [0u8; 18];
        let sz = gps.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_GPS), timestamp, &buf[..sz])
    }

    /// Log a baro sample.
    pub fn log_baro(&mut self, baro: &LogBaro, timestamp: Instant) -> bool {
        let mut buf = [0u8; 12];
        let sz = baro.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_BARO), timestamp, &buf[..sz])
    }

    /// Log a battery sample.
    pub fn log_battery(&mut self, bat: &LogBat, timestamp: Instant) -> bool {
        let mut buf = [0u8; 13];
        let sz = bat.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_BAT), timestamp, &buf[..sz])
    }

    /// Log a mode change event.
    pub fn log_mode(&mut self, mode: &LogMode, timestamp: Instant) -> bool {
        let mut buf = [0u8; 2];
        let sz = mode.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_MODE), timestamp, &buf[..sz])
    }

    /// Log an arm/disarm event.
    pub fn log_arm(&mut self, arm: &LogArm, timestamp: Instant) -> bool {
        let mut buf = [0u8; 1];
        let sz = arm.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_ARM), timestamp, &buf[..sz])
    }

    /// Log an error/failsafe event.
    pub fn log_error(&mut self, err: &LogErr, timestamp: Instant) -> bool {
        let mut buf = [0u8; 2];
        let sz = err.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_ERR), timestamp, &buf[..sz])
    }

    /// Log EKF state.
    pub fn log_ekf(&mut self, ekf: &LogEkf, timestamp: Instant) -> bool {
        let mut buf = [0u8; 48];
        let sz = ekf.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_EKF), timestamp, &buf[..sz])
    }

    /// Log motor outputs.
    pub fn log_rcout(&mut self, rcout: &LogRcOut, timestamp: Instant) -> bool {
        let mut buf = [0u8; 17];
        let sz = rcout.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_RCOU), timestamp, &buf[..sz])
    }

    /// Log a scheduler loop overrun event.
    pub fn log_overrun(&mut self, overrun: &LogOverrun, timestamp: Instant) -> bool {
        let mut buf = [0u8; 9];
        let sz = overrun.serialize_into(&mut buf);
        self.buffer.write(LogMsgId(LOG_OVERRUN), timestamp, &buf[..sz])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_buffer_write() {
        let mut buf = LogBuffer::<1024>::new();
        let data = [1u8, 2, 3, 4, 5];
        assert!(buf.write(LogMsgId(1), Instant::from_micros(1000), &data));
        assert_eq!(buf.entry_count(), 1);
        assert!(buf.used_bytes() > 0);
    }

    #[test]
    fn test_log_buffer_overflow() {
        let mut buf = LogBuffer::<32>::new();
        let data = [0u8; 30]; // 8 header + 30 data = 38 > 32
        assert!(!buf.write(LogMsgId(1), Instant::from_micros(0), &data));
        assert_eq!(buf.overflows(), 1);
    }

    #[test]
    fn test_log_buffer_wrap() {
        let mut buf = LogBuffer::<64>::new();
        let data = [0u8; 20]; // 8+20 = 28 bytes per entry
        assert!(buf.write(LogMsgId(1), Instant::from_micros(0), &data));
        assert!(buf.write(LogMsgId(2), Instant::from_micros(1000), &data)); // 56 bytes total
        // Third write wraps
        assert!(buf.write(LogMsgId(3), Instant::from_micros(2000), &data));
        assert_eq!(buf.entry_count(), 3);
    }

    #[test]
    fn test_status_message() {
        let msg = StatusMessage::new(LogLevel::Warning, "GPS lost");
        assert_eq!(msg.level, LogLevel::Warning);
        assert_eq!(msg.text.as_str(), "GPS lost");
    }

    #[test]
    fn test_field_type_sizes() {
        assert_eq!(LogFieldType::F32.size(), 4);
        assert_eq!(LogFieldType::U16.size(), 2);
        assert_eq!(LogFieldType::Bool.size(), 1);
    }

    #[test]
    fn test_log_att_serialize() {
        let att = LogAtt {
            roll_deg: 5.2, pitch_deg: -3.1, yaw_deg: 180.0,
            des_roll_deg: 5.0, des_pitch_deg: -3.0, des_yaw_deg: 180.0,
        };
        let mut buf = [0u8; 24];
        let sz = att.serialize_into(&mut buf);
        assert_eq!(sz, 24);
        let roll = f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        assert!((roll - 5.2).abs() < 1e-6);
    }

    #[test]
    fn test_log_gps_serialize() {
        let gps = LogGps {
            fix_type: 3, num_sats: 12,
            lat_e7: 350000000, lon_e7: -1200000000,
            alt_cm: 5000, speed_cm: 250, hacc_cm: 150,
        };
        let mut buf = [0u8; 18];
        let sz = gps.serialize_into(&mut buf);
        assert_eq!(sz, 18);
        assert_eq!(buf[0], 3);
        assert_eq!(buf[1], 12);
    }

    #[test]
    fn test_log_mode_serialize() {
        let mode = LogMode { mode_num: 5, reason: 1 };
        let mut buf = [0u8; 2];
        assert_eq!(mode.serialize_into(&mut buf), 2);
        assert_eq!(buf[0], 5);
        assert_eq!(buf[1], 1);
    }

    #[test]
    fn test_log_err_serialize() {
        let err = LogErr { subsys: 2, code: 1 };
        let mut buf = [0u8; 2];
        assert_eq!(err.serialize_into(&mut buf), 2);
    }

    #[test]
    fn test_log_ekf_serialize() {
        let ekf = LogEkf {
            roll_deg: 1.0, pitch_deg: 2.0, yaw_deg: 3.0,
            vel_n: 0.1, vel_e: 0.2, vel_d: -0.3,
            pos_n: 10.0, pos_e: 20.0, pos_d: -30.0,
            gyro_bias_x: 0.001, gyro_bias_y: -0.002, gyro_bias_z: 0.0,
        };
        let mut buf = [0u8; 48];
        let sz = ekf.serialize_into(&mut buf);
        assert_eq!(sz, 48);
    }

    #[test]
    fn test_flight_logger_rate_limiting() {
        let mut logger = FlightLogger::<8192>::new();
        logger.write_schemas();

        let imu = LogImu { gyr_x: 0.0, gyr_y: 0.0, gyr_z: 0.0, acc_x: 0.0, acc_y: 0.0, acc_z: -9.8 };
        let t = Instant::from_micros(1000);

        // First 3 calls should be rate-limited (divider=4)
        assert!(!logger.log_imu(&imu, t));
        assert!(!logger.log_imu(&imu, t));
        assert!(!logger.log_imu(&imu, t));
        // 4th call should log
        assert!(logger.log_imu(&imu, t));
    }

    #[test]
    fn test_flight_logger_events() {
        let mut logger = FlightLogger::<4096>::new();
        let t = Instant::from_micros(5000);

        // Events should always log (no rate limiting)
        assert!(logger.log_arm(&LogArm { armed: true }, t));
        assert!(logger.log_mode(&LogMode { mode_num: 0, reason: 0 }, t));
        assert!(logger.log_error(&LogErr { subsys: 1, code: 1 }, t));
        assert_eq!(logger.buffer.entry_count(), 3);
    }

    #[test]
    fn test_all_schemas_valid() {
        // Verify all log types produce valid schemas
        let schemas = [
            LogAtt::schema(), LogImu::schema(), LogGps::schema(),
            LogBaro::schema(), LogBat::schema(), LogMode::schema(),
            LogArm::schema(), LogErr::schema(), LogEkf::schema(),
            LogRcOut::schema(), LogOverrun::schema(),
        ];
        for s in &schemas {
            assert!(!s.name.is_empty());
            assert!(!s.fields.is_empty());
        }
    }

    #[test]
    fn test_log_overrun_serialize() {
        let overrun = LogOverrun {
            task_id: 3,
            actual_us: 5000,
            budget_us: 2500,
        };
        let mut buf = [0u8; 9];
        let sz = overrun.serialize_into(&mut buf);
        assert_eq!(sz, 9);
        assert_eq!(buf[0], 3);
        let actual = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
        assert_eq!(actual, 5000);
        let budget = u32::from_le_bytes([buf[5], buf[6], buf[7], buf[8]]);
        assert_eq!(budget, 2500);
    }

    #[test]
    fn test_log_overrun_schema() {
        let schema = LogOverrun::schema();
        assert_eq!(schema.name.as_str(), "OVRN");
        assert_eq!(schema.fields.len(), 3);
        assert_eq!(schema.fields[0].name.as_str(), "Task");
        assert_eq!(schema.fields[1].name.as_str(), "Actual");
        assert_eq!(schema.fields[2].name.as_str(), "Budget");
    }

    #[test]
    fn test_log_overrun_too_small_buffer() {
        let overrun = LogOverrun { task_id: 0, actual_us: 100, budget_us: 50 };
        let mut buf = [0u8; 5]; // too small
        let sz = overrun.serialize_into(&mut buf);
        assert_eq!(sz, 0);
    }

    #[test]
    fn test_flight_logger_overrun() {
        let mut logger = FlightLogger::<4096>::new();
        let t = Instant::from_micros(1000);
        let overrun = LogOverrun { task_id: 1, actual_us: 3000, budget_us: 2500 };
        assert!(logger.log_overrun(&overrun, t));
        assert_eq!(logger.buffer.entry_count(), 1);
    }
}

