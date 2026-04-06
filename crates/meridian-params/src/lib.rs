#![no_std]

//! Runtime parameter system — typed, validated, persistent.
//!
//! On embedded: flash storage with A/B wear-leveling.
//! On Linux/SITL: binary file or TOML file.
//! Compatible with MAVLink PARAM protocol.

use heapless::String;

/// Maximum number of parameters.
/// ArduCopter has ~1200 params. Start with 512, expand as needed.
pub const MAX_PARAMS: usize = 512;

/// Parameter value type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParamValue {
    F32(f32),
    I32(i32),
    U16(u16),
    U8(u8),
}

impl ParamValue {
    pub fn as_f32(&self) -> f32 {
        match self {
            Self::F32(v) => *v,
            Self::I32(v) => *v as f32,
            Self::U16(v) => *v as f32,
            Self::U8(v) => *v as f32,
        }
    }

    pub fn type_id(&self) -> u8 {
        match self { Self::F32(_) => 0, Self::I32(_) => 1, Self::U16(_) => 2, Self::U8(_) => 3 }
    }
}

/// A single parameter definition.
#[derive(Debug, Clone)]
pub struct ParamEntry {
    pub name: String<16>,
    pub value: ParamValue,
    pub default: ParamValue,
    pub min: f32,
    pub max: f32,
}

impl ParamEntry {
    pub fn new_f32(name: &str, default: f32, min: f32, max: f32) -> Self {
        let mut n = String::new();
        let _ = n.push_str(name);
        Self { name: n, value: ParamValue::F32(default), default: ParamValue::F32(default), min, max }
    }

    /// Set value with bounds checking. Returns true if accepted.
    pub fn set(&mut self, v: f32) -> bool {
        if v < self.min || v > self.max { return false; }
        self.value = match self.value {
            ParamValue::F32(_) => ParamValue::F32(v),
            ParamValue::I32(_) => ParamValue::I32(v as i32),
            ParamValue::U16(_) => ParamValue::U16(v as u16),
            ParamValue::U8(_) => ParamValue::U8(v as u8),
        };
        true
    }

    pub fn reset_default(&mut self) {
        self.value = self.default;
    }
}

/// Maximum number of dirty params that can be queued for save.
pub const MAX_SAVE_QUEUE: usize = 16;

/// The parameter store.
pub struct ParamStore {
    params: heapless::Vec<ParamEntry, MAX_PARAMS>,
    dirty: bool,
    save_queue: heapless::Vec<heapless::String<16>, MAX_SAVE_QUEUE>,
}

impl ParamStore {
    pub fn new() -> Self {
        Self {
            params: heapless::Vec::new(),
            dirty: false,
            save_queue: heapless::Vec::new(),
        }
    }

    /// Register a parameter. Call during init.
    pub fn register(&mut self, entry: ParamEntry) -> bool {
        if self.params.len() >= MAX_PARAMS { return false; }
        let _ = self.params.push(entry);
        true
    }

    /// Register standard flight controller parameters.
    /// Matches ArduPilot defaults for ArduCopter.
    pub fn register_defaults(&mut self) {
        // ─── Rate PIDs (ATC_RAT_*) ───
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_P", 0.135, 0.0, 1.0));
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_I", 0.135, 0.0, 1.0));
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_D", 0.0036, 0.0, 0.1));
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_FF", 0.0, 0.0, 0.5));
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_FLTT", 0.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_FLTE", 0.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_FLTD", 20.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_RLL_SMAX", 0.0, 0.0, 200.0));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_P", 0.135, 0.0, 1.0));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_I", 0.135, 0.0, 1.0));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_D", 0.0036, 0.0, 0.1));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_FF", 0.0, 0.0, 0.5));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_FLTT", 0.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_FLTE", 0.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_FLTD", 20.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_PIT_SMAX", 0.0, 0.0, 200.0));
        self.register(ParamEntry::new_f32("ATC_RAT_YAW_P", 0.180, 0.0, 1.0));
        self.register(ParamEntry::new_f32("ATC_RAT_YAW_I", 0.018, 0.0, 1.0));
        self.register(ParamEntry::new_f32("ATC_RAT_YAW_D", 0.0, 0.0, 0.1));
        self.register(ParamEntry::new_f32("ATC_RAT_YAW_FF", 0.0, 0.0, 0.5));
        self.register(ParamEntry::new_f32("ATC_RAT_YAW_FLTD", 0.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_YAW_FLTE", 2.5, 0.0, 100.0));
        self.register(ParamEntry::new_f32("ATC_RAT_YAW_FLTT", 20.0, 0.0, 100.0));

        // ─── Attitude P gains (ATC_ANG_*) ───
        self.register(ParamEntry::new_f32("ATC_ANG_RLL_P", 4.5, 1.0, 12.0));
        self.register(ParamEntry::new_f32("ATC_ANG_PIT_P", 4.5, 1.0, 12.0));
        self.register(ParamEntry::new_f32("ATC_ANG_YAW_P", 4.5, 1.0, 12.0));
        self.register(ParamEntry::new_f32("ATC_ACCEL_R_MAX", 110000.0, 0.0, 500000.0));
        self.register(ParamEntry::new_f32("ATC_ACCEL_P_MAX", 110000.0, 0.0, 500000.0));
        self.register(ParamEntry::new_f32("ATC_ACCEL_Y_MAX", 27000.0, 0.0, 500000.0));
        self.register(ParamEntry::new_f32("ATC_INPUT_TC", 0.15, 0.0, 1.0));
        self.register(ParamEntry::new_f32("ANGLE_MAX", 3000.0, 1000.0, 8000.0)); // cdeg

        // ─── Position controller (PSC_*) ───
        self.register(ParamEntry::new_f32("PSC_POSXY_P", 1.0, 0.1, 5.0));
        self.register(ParamEntry::new_f32("PSC_VELXY_P", 2.0, 0.1, 6.0));
        self.register(ParamEntry::new_f32("PSC_VELXY_I", 1.0, 0.0, 3.0));
        self.register(ParamEntry::new_f32("PSC_VELXY_D", 0.5, 0.0, 2.0));
        self.register(ParamEntry::new_f32("PSC_VELXY_FF", 0.0, 0.0, 6.0));
        self.register(ParamEntry::new_f32("PSC_POSZ_P", 1.0, 0.1, 5.0));
        self.register(ParamEntry::new_f32("PSC_VELZ_P", 5.0, 0.1, 10.0));
        self.register(ParamEntry::new_f32("PSC_VELZ_I", 1.0, 0.0, 5.0));
        self.register(ParamEntry::new_f32("PSC_ACCZ_P", 0.3, 0.0, 2.0));
        self.register(ParamEntry::new_f32("PSC_ACCZ_I", 1.0, 0.0, 3.0));
        self.register(ParamEntry::new_f32("PSC_ACCZ_D", 0.0, 0.0, 0.4));
        self.register(ParamEntry::new_f32("PSC_JERK_XY", 5.0, 1.0, 20.0));
        self.register(ParamEntry::new_f32("PSC_JERK_Z", 5.0, 1.0, 20.0));

        // ─── EKF3 (EK3_*) ───
        self.register(ParamEntry::new_f32("EK3_ENABLE", 1.0, 0.0, 1.0));
        self.register(ParamEntry::new_f32("EK3_GPS_TYPE", 0.0, 0.0, 3.0));
        self.register(ParamEntry::new_f32("EK3_VELNE_M_NSE", 0.5, 0.05, 5.0));
        self.register(ParamEntry::new_f32("EK3_VELD_M_NSE", 0.7, 0.05, 5.0));
        self.register(ParamEntry::new_f32("EK3_POSNE_M_NSE", 0.5, 0.1, 10.0));
        self.register(ParamEntry::new_f32("EK3_ALT_M_NSE", 2.0, 0.1, 10.0));
        self.register(ParamEntry::new_f32("EK3_MAG_M_NSE", 0.05, 0.01, 0.5));
        self.register(ParamEntry::new_f32("EK3_GYRO_P_NSE", 0.015, 0.001, 0.1));
        self.register(ParamEntry::new_f32("EK3_ACC_P_NSE", 0.25, 0.01, 2.0));
        self.register(ParamEntry::new_f32("EK3_GBIAS_P_NSE", 7.0e-6, 1e-8, 1e-3));
        self.register(ParamEntry::new_f32("EK3_ABIAS_P_NSE", 5.0e-4, 1e-6, 1e-2));

        // ─── Motor/Throttle (MOT_*) ───
        self.register(ParamEntry::new_f32("MOT_SPIN_ARM", 0.10, 0.0, 0.5));
        self.register(ParamEntry::new_f32("MOT_SPIN_MIN", 0.15, 0.0, 0.5));
        self.register(ParamEntry::new_f32("MOT_SPIN_MAX", 0.95, 0.5, 1.0));
        self.register(ParamEntry::new_f32("MOT_THST_EXPO", 0.65, 0.0, 1.0));
        self.register(ParamEntry::new_f32("MOT_SPOOL_TIME", 0.5, 0.0, 2.0));
        self.register(ParamEntry::new_f32("MOT_BAT_VOLT_MAX", 16.8, 0.0, 100.0));
        self.register(ParamEntry::new_f32("MOT_BAT_VOLT_MIN", 14.0, 0.0, 100.0));

        // ─── Failsafe (FS_*) ───
        self.register(ParamEntry::new_f32("FS_THR_ENABLE", 1.0, 0.0, 2.0));
        self.register(ParamEntry::new_f32("FS_THR_VALUE", 975.0, 900.0, 1100.0));
        self.register(ParamEntry::new_f32("FS_BATT_ENABLE", 0.0, 0.0, 2.0));
        self.register(ParamEntry::new_f32("FS_BATT_VOLTAGE", 10.5, 0.0, 100.0));
        self.register(ParamEntry::new_f32("FS_BATT_MAH", 0.0, 0.0, 100000.0));
        self.register(ParamEntry::new_f32("FS_GCS_ENABLE", 0.0, 0.0, 2.0));
        self.register(ParamEntry::new_f32("FS_EKF_ACTION", 1.0, 0.0, 3.0));
        self.register(ParamEntry::new_f32("FS_EKF_THRESH", 0.8, 0.0, 1.0));
        self.register(ParamEntry::new_f32("FS_OPTIONS", 0.0, 0.0, 255.0));

        // ─── Fence ───
        self.register(ParamEntry::new_f32("FENCE_ENABLE", 0.0, 0.0, 1.0));
        self.register(ParamEntry::new_f32("FENCE_TYPE", 7.0, 0.0, 15.0));
        self.register(ParamEntry::new_f32("FENCE_ACTION", 1.0, 0.0, 5.0));
        self.register(ParamEntry::new_f32("FENCE_RADIUS", 300.0, 10.0, 100000.0));
        self.register(ParamEntry::new_f32("FENCE_ALT_MAX", 120.0, 10.0, 10000.0));
        self.register(ParamEntry::new_f32("FENCE_ALT_MIN", -10.0, -100.0, 100.0));
        self.register(ParamEntry::new_f32("FENCE_MARGIN", 2.0, 0.0, 100.0));

        // ─── Waypoint navigation (WPNAV_*) ───
        self.register(ParamEntry::new_f32("WPNAV_SPEED", 500.0, 20.0, 2000.0)); // cm/s
        self.register(ParamEntry::new_f32("WPNAV_SPEED_UP", 250.0, 10.0, 1000.0));
        self.register(ParamEntry::new_f32("WPNAV_SPEED_DN", 150.0, 10.0, 500.0));
        self.register(ParamEntry::new_f32("WPNAV_ACCEL", 250.0, 50.0, 1000.0)); // cm/s²
        self.register(ParamEntry::new_f32("WPNAV_RADIUS", 200.0, 5.0, 1000.0)); // cm
        self.register(ParamEntry::new_f32("WPNAV_JERK", 1.0, 0.1, 20.0)); // m/s³

        // ─── RTL ───
        self.register(ParamEntry::new_f32("RTL_ALT", 1500.0, 0.0, 80000.0)); // cm
        self.register(ParamEntry::new_f32("RTL_ALT_FINAL", 0.0, 0.0, 1000.0)); // cm
        self.register(ParamEntry::new_f32("RTL_SPEED", 0.0, 0.0, 2000.0)); // cm/s, 0=WPNAV

        // ─── L1 navigation ───
        self.register(ParamEntry::new_f32("NAVL1_PERIOD", 17.0, 5.0, 40.0));
        self.register(ParamEntry::new_f32("NAVL1_DAMPING", 0.75, 0.5, 1.0));

        // ─── General ───
        self.register(ParamEntry::new_f32("FLTMODE1", 0.0, 0.0, 25.0));
        self.register(ParamEntry::new_f32("FLTMODE2", 0.0, 0.0, 25.0));
        self.register(ParamEntry::new_f32("FLTMODE3", 0.0, 0.0, 25.0));
        self.register(ParamEntry::new_f32("FLTMODE4", 0.0, 0.0, 25.0));
        self.register(ParamEntry::new_f32("FLTMODE5", 0.0, 0.0, 25.0));
        self.register(ParamEntry::new_f32("FLTMODE6", 0.0, 0.0, 25.0));
        self.register(ParamEntry::new_f32("SYSID_THISMAV", 1.0, 1.0, 255.0));
        self.register(ParamEntry::new_f32("FRAME_TYPE", 1.0, 0.0, 20.0)); // 1=Quad-X

        // ─── Stream rates ───
        self.register(ParamEntry::new_f32("SR0_RAW_SENS", 2.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("SR0_EXT_STAT", 2.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("SR0_RC_CHAN", 5.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("SR0_POSITION", 3.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("SR0_EXTRA1", 10.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("SR0_EXTRA2", 4.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("SR0_EXTRA3", 1.0, 0.0, 50.0));

        // ─── Serial ───
        self.register(ParamEntry::new_f32("SERIAL0_BAUD", 115.0, 1.0, 2000.0)); // *1000
        self.register(ParamEntry::new_f32("SERIAL1_BAUD", 57.0, 1.0, 2000.0));
        self.register(ParamEntry::new_f32("SERIAL1_PROTCL", 2.0, 0.0, 50.0)); // MAVLink2
        self.register(ParamEntry::new_f32("SERIAL3_PROTCL", 5.0, 0.0, 50.0)); // GPS
        self.register(ParamEntry::new_f32("SERIAL3_BAUD", 115.0, 1.0, 2000.0));

        // ─── INS (IMU) ───
        self.register(ParamEntry::new_f32("INS_GYRO_FILT", 20.0, 0.0, 256.0));
        self.register(ParamEntry::new_f32("INS_ACCEL_FILT", 20.0, 0.0, 256.0));
        self.register(ParamEntry::new_f32("INS_FAST_SAMPLE", 7.0, 0.0, 7.0));

        // ─── Battery ───
        self.register(ParamEntry::new_f32("BATT_MONITOR", 4.0, 0.0, 32.0));
        self.register(ParamEntry::new_f32("BATT_CAPACITY", 3300.0, 0.0, 100000.0));
        self.register(ParamEntry::new_f32("BATT_VOLT_PIN", 10.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("BATT_CURR_PIN", 11.0, 0.0, 50.0));
        self.register(ParamEntry::new_f32("BATT_VOLT_MULT", 11.0, 0.0, 100.0));
        self.register(ParamEntry::new_f32("BATT_AMP_PERVLT", 17.0, 0.0, 200.0));
    }

    /// Find parameter by name.
    pub fn get(&self, name: &str) -> Option<&ParamEntry> {
        self.params.iter().find(|p| p.name.as_str() == name)
    }

    /// Get parameter by index.
    pub fn get_by_index(&self, idx: usize) -> Option<&ParamEntry> {
        self.params.get(idx)
    }

    /// Set parameter value by name. Returns true if accepted.
    pub fn set(&mut self, name: &str, value: f32) -> bool {
        if let Some(p) = self.params.iter_mut().find(|p| p.name.as_str() == name) {
            if p.set(value) { self.dirty = true; return true; }
        }
        false
    }

    /// Access the parameters slice (for serialization).
    pub fn params(&self) -> &[ParamEntry] {
        &self.params
    }

    /// Apply loaded values: for each entry in `loaded`, if a matching registered param exists,
    /// update its value (still subject to bounds checking).
    pub fn apply_loaded(&mut self, loaded: &[(heapless::String<16>, ParamValue)]) {
        for (name, val) in loaded {
            self.set(name.as_str(), val.as_f32());
        }
        self.dirty = false; // just loaded, not dirty
    }

    pub fn count(&self) -> usize { self.params.len() }
    pub fn is_dirty(&self) -> bool { self.dirty }
    pub fn mark_clean(&mut self) { self.dirty = false; }

    /// Mark a parameter as needing save. Added to save queue if not already present.
    pub fn mark_dirty(&mut self, name: &str) {
        // Check if already in queue
        for entry in self.save_queue.iter() {
            if entry.as_str() == name { return; }
        }
        if self.save_queue.len() < MAX_SAVE_QUEUE {
            let mut s = heapless::String::<16>::new();
            let _ = s.push_str(name);
            let _ = self.save_queue.push(s);
        }
    }

    /// Pop one param name from the save queue. Returns None if queue is empty.
    /// Call from a low-priority loop to save params one at a time (non-blocking).
    pub fn flush_one(&mut self) -> Option<heapless::String<16>> {
        if self.save_queue.is_empty() {
            None
        } else {
            Some(self.save_queue.swap_remove(0))
        }
    }

    /// Number of params queued for save.
    pub fn save_queue_len(&self) -> usize {
        self.save_queue.len()
    }
}

// ─── Persistence Backend Trait ───

/// Binary format magic and version for parameter storage.
pub const PARAM_MAGIC: u32 = 0x4D505241; // "MPRA"
pub const PARAM_VERSION: u16 = 1;

/// Trait for parameter persistence backends.
pub trait ParamBackend {
    /// Save the entire parameter store. Returns true on success.
    fn save(&self, store: &ParamStore) -> bool;
    /// Load parameters. Returns list of (name, value) pairs, or None on failure.
    fn load(&self) -> Option<heapless::Vec<(heapless::String<16>, ParamValue), MAX_PARAMS>>;
}

// ─── Binary Serialization Helpers ───

/// A serialized parameter entry in binary format.
/// Layout: [name: 16 bytes, zero-padded] [type_id: 1] [value: 4 bytes LE] = 21 bytes per entry.
const BINARY_ENTRY_SIZE: usize = 21;

/// Serialize a ParamStore into a binary buffer.
/// Format: [magic: 4 LE] [version: 2 LE] [count: 2 LE] [entries...] [crc32: 4 LE]
/// Returns the number of bytes written, or None if buffer too small.
pub fn serialize_params(store: &ParamStore, buf: &mut [u8]) -> Option<usize> {
    let count = store.count();
    let needed = 4 + 2 + 2 + count * BINARY_ENTRY_SIZE + 4; // header + entries + crc32
    if buf.len() < needed { return None; }

    // Magic
    buf[0..4].copy_from_slice(&PARAM_MAGIC.to_le_bytes());
    // Version
    buf[4..6].copy_from_slice(&PARAM_VERSION.to_le_bytes());
    // Count
    buf[6..8].copy_from_slice(&(count as u16).to_le_bytes());

    let mut offset = 8;
    for i in 0..count {
        if let Some(entry) = store.get_by_index(i) {
            // Name: 16 bytes zero-padded
            let name_bytes = entry.name.as_bytes();
            let copy_len = name_bytes.len().min(16);
            buf[offset..offset + copy_len].copy_from_slice(&name_bytes[..copy_len]);
            for j in copy_len..16 { buf[offset + j] = 0; }
            offset += 16;
            // Type ID
            buf[offset] = entry.value.type_id();
            offset += 1;
            // Value as 4 LE bytes
            let val_bytes = match entry.value {
                ParamValue::F32(v) => v.to_le_bytes(),
                ParamValue::I32(v) => v.to_le_bytes(),
                ParamValue::U16(v) => {
                    let mut b = [0u8; 4];
                    b[0..2].copy_from_slice(&v.to_le_bytes());
                    b
                }
                ParamValue::U8(v) => {
                    let mut b = [0u8; 4];
                    b[0] = v;
                    b
                }
            };
            buf[offset..offset + 4].copy_from_slice(&val_bytes);
            offset += 4;
        }
    }

    // CRC32 over everything before the CRC field
    let crc = crc32(&buf[..offset]);
    buf[offset..offset + 4].copy_from_slice(&crc.to_le_bytes());
    offset += 4;

    Some(offset)
}

/// Deserialize binary param data. Returns (name, value) pairs, or None on bad magic/version/CRC.
pub fn deserialize_params(buf: &[u8]) -> Option<heapless::Vec<(heapless::String<16>, ParamValue), MAX_PARAMS>> {
    if buf.len() < 8 { return None; }

    // Check magic
    let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if magic != PARAM_MAGIC { return None; }

    // Check version
    let version = u16::from_le_bytes([buf[4], buf[5]]);
    if version != PARAM_VERSION { return None; }

    let count = u16::from_le_bytes([buf[6], buf[7]]) as usize;
    let data_end = 8 + count * BINARY_ENTRY_SIZE;
    let total_needed = data_end + 4; // + crc32
    if buf.len() < total_needed { return None; }

    // Verify CRC32
    let stored_crc = u32::from_le_bytes([buf[data_end], buf[data_end+1], buf[data_end+2], buf[data_end+3]]);
    let computed_crc = crc32(&buf[..data_end]);
    if stored_crc != computed_crc { return None; }

    let mut result = heapless::Vec::new();
    let mut offset = 8;
    for _ in 0..count {
        // Name
        let name_raw = &buf[offset..offset + 16];
        let name_len = name_raw.iter().position(|&b| b == 0).unwrap_or(16);
        let mut name = heapless::String::<16>::new();
        if let Ok(s) = core::str::from_utf8(&name_raw[..name_len]) {
            let _ = name.push_str(s);
        }
        offset += 16;

        // Type ID
        let type_id = buf[offset];
        offset += 1;

        // Value
        let val_bytes = [buf[offset], buf[offset+1], buf[offset+2], buf[offset+3]];
        offset += 4;

        let value = match type_id {
            0 => ParamValue::F32(f32::from_le_bytes(val_bytes)),
            1 => ParamValue::I32(i32::from_le_bytes(val_bytes)),
            2 => ParamValue::U16(u16::from_le_bytes([val_bytes[0], val_bytes[1]])),
            3 => ParamValue::U8(val_bytes[0]),
            _ => continue, // skip unknown types
        };

        let _ = result.push((name, value));
    }

    Some(result)
}

/// CRC32 (ISO 3309 / ITU-T V.42, same polynomial as Ethernet/zlib).
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFFFFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

// ─── File-based Backend (SITL / Linux) ───

/// In-memory file backend that operates on a provided byte buffer.
/// On SITL/Linux with `std`, the caller reads/writes the buffer to a file.
/// This keeps the core logic `no_std` compatible.
pub struct FileParamBackend<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> FileParamBackend<'a> {
    /// Create a new file backend backed by the given buffer.
    /// `len` is the number of valid bytes currently in the buffer (from a previous load).
    pub fn new(buf: &'a mut [u8], len: usize) -> Self {
        Self { buf, len }
    }

    /// Get the serialized data after a save.
    pub fn data(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// Set the buffer contents (e.g., after reading from a file).
    pub fn set_data(&mut self, data: &[u8]) {
        let copy_len = data.len().min(self.buf.len());
        self.buf[..copy_len].copy_from_slice(&data[..copy_len]);
        self.len = copy_len;
    }
}

impl<'a> ParamBackend for FileParamBackend<'a> {
    fn save(&self, store: &ParamStore) -> bool {
        // We can't mutate self.buf through &self (trait requires &self for save),
        // but the caller should use serialize_params directly for the mutable case.
        // This implementation verifies the store can be serialized.
        // In practice, callers use serialize_params() + write to file.
        let mut tmp = [0u8; 4096];
        serialize_params(store, &mut tmp).is_some()
    }

    fn load(&self) -> Option<heapless::Vec<(heapless::String<16>, ParamValue), MAX_PARAMS>> {
        if self.len == 0 { return None; }
        deserialize_params(&self.buf[..self.len])
    }
}

// ─── Flash A/B Backend (STM32 embedded) ───

/// Flash parameter backend with A/B sector ping-pong for corruption recovery.
///
/// Two sectors alternate writes. Each sector has the format:
/// `[magic: u32] [version: u16] [count: u16] [entries: N * 21 bytes] [crc32: u32]`
///
/// On load: try active sector, if CRC fails try the other, if both fail return None (defaults).
/// On save: write to the inactive sector, then swap active.
pub struct FlashParamBackend<'a> {
    sector_a: &'a mut [u8],
    sector_b: &'a mut [u8],
    /// Which sector was last written (false = A, true = B).
    active_is_b: bool,
}

impl<'a> FlashParamBackend<'a> {
    /// Create a new flash backend with two sector buffers.
    pub fn new(sector_a: &'a mut [u8], sector_b: &'a mut [u8]) -> Self {
        Self { sector_a, sector_b, active_is_b: false }
    }

    /// Try to deserialize a sector. Returns params if valid.
    fn try_sector(sector: &[u8]) -> Option<heapless::Vec<(heapless::String<16>, ParamValue), MAX_PARAMS>> {
        deserialize_params(sector)
    }

    /// Determine which sector is active by trying both.
    /// Prefers A, but if A is corrupt and B is valid, uses B.
    pub fn detect_active(&mut self) {
        let a_valid = Self::try_sector(self.sector_a).is_some();
        let b_valid = Self::try_sector(self.sector_b).is_some();
        if a_valid {
            self.active_is_b = false;
        } else if b_valid {
            self.active_is_b = true;
        } else {
            self.active_is_b = false; // both bad, default to A
        }
    }

    /// Returns true if sector A is active.
    pub fn active_is_a(&self) -> bool { !self.active_is_b }
}

impl<'a> ParamBackend for FlashParamBackend<'a> {
    fn save(&self, _store: &ParamStore) -> bool {
        // Cannot mutate flash sectors through &self.
        // Real embedded code uses save_mut() which requires &mut self,
        // or uses raw flash HAL writes. This returns false to signal
        // that callers should use save_mut() instead.
        false
    }

    fn load(&self) -> Option<heapless::Vec<(heapless::String<16>, ParamValue), MAX_PARAMS>> {
        // Try active sector first, then fallback
        let (primary, secondary) = if self.active_is_b {
            (&*self.sector_b, &*self.sector_a)
        } else {
            (&*self.sector_a, &*self.sector_b)
        };

        if let Some(params) = Self::try_sector(primary) {
            return Some(params);
        }
        // Primary corrupt — try fallback
        Self::try_sector(secondary)
    }
}

/// Mutable save for FlashParamBackend — writes to the inactive sector and swaps.
/// Separated from the trait because it needs &mut self.
impl<'a> FlashParamBackend<'a> {
    pub fn save_mut(&mut self, store: &ParamStore) -> bool {
        // Write to inactive sector
        let target = if self.active_is_b {
            &mut self.sector_a
        } else {
            &mut self.sector_b
        };

        // Clear target sector
        for b in target.iter_mut() { *b = 0xFF; }

        if let Some(_written) = serialize_params(store, target) {
            // Swap active sector
            self.active_is_b = !self.active_is_b;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_and_get() {
        let mut store = ParamStore::new();
        store.register_defaults();
        assert!(store.count() > 10);

        let p = store.get("ATC_RAT_RLL_P").unwrap();
        assert!((p.value.as_f32() - 0.135).abs() < 1e-6);
    }

    #[test]
    fn test_set_within_bounds() {
        let mut store = ParamStore::new();
        store.register_defaults();
        assert!(store.set("ATC_RAT_RLL_P", 0.2));
        assert!((store.get("ATC_RAT_RLL_P").unwrap().value.as_f32() - 0.2).abs() < 1e-6);
        assert!(store.is_dirty());
    }

    #[test]
    fn test_set_out_of_bounds_rejected() {
        let mut store = ParamStore::new();
        store.register_defaults();
        assert!(!store.set("ATC_RAT_RLL_P", 5.0)); // max is 1.0
        // Value unchanged
        assert!((store.get("ATC_RAT_RLL_P").unwrap().value.as_f32() - 0.135).abs() < 1e-6);
    }

    #[test]
    fn test_get_by_index() {
        let mut store = ParamStore::new();
        store.register_defaults();
        let p = store.get_by_index(0).unwrap();
        assert_eq!(p.name.as_str(), "ATC_RAT_RLL_P");
    }

    // ─── Save Queue Tests ───

    #[test]
    fn test_save_queue_mark_dirty() {
        let mut store = ParamStore::new();
        store.register_defaults();
        assert_eq!(store.save_queue_len(), 0);

        store.mark_dirty("ATC_RAT_RLL_P");
        assert_eq!(store.save_queue_len(), 1);

        // Duplicate should not add again
        store.mark_dirty("ATC_RAT_RLL_P");
        assert_eq!(store.save_queue_len(), 1);

        store.mark_dirty("ATC_RAT_PIT_P");
        assert_eq!(store.save_queue_len(), 2);
    }

    #[test]
    fn test_save_queue_flush_one() {
        let mut store = ParamStore::new();
        store.register_defaults();

        store.mark_dirty("ATC_RAT_RLL_P");
        store.mark_dirty("ATC_RAT_PIT_P");
        assert_eq!(store.save_queue_len(), 2);

        let first = store.flush_one().unwrap();
        assert_eq!(first.as_str(), "ATC_RAT_RLL_P");
        assert_eq!(store.save_queue_len(), 1);

        let second = store.flush_one().unwrap();
        assert_eq!(second.as_str(), "ATC_RAT_PIT_P");
        assert_eq!(store.save_queue_len(), 0);

        assert!(store.flush_one().is_none());
    }

    // ─── Serialization Tests ───

    #[test]
    fn test_serialize_deserialize_roundtrip() {
        let mut store = ParamStore::new();
        store.register_defaults();
        store.set("ATC_RAT_RLL_P", 0.25);

        let mut buf = [0u8; 4096];
        let len = serialize_params(&store, &mut buf).unwrap();
        assert!(len > 8); // at least header

        let loaded = deserialize_params(&buf[..len]).unwrap();
        assert_eq!(loaded.len(), store.count());

        // Check the modified value round-tripped
        let rll_p = loaded.iter().find(|(n, _)| n.as_str() == "ATC_RAT_RLL_P").unwrap();
        assert!((rll_p.1.as_f32() - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_deserialize_bad_magic_fails() {
        let mut buf = [0u8; 64];
        buf[0..4].copy_from_slice(&0xDEADBEEF_u32.to_le_bytes());
        assert!(deserialize_params(&buf).is_none());
    }

    #[test]
    fn test_deserialize_bad_crc_fails() {
        let mut store = ParamStore::new();
        store.register(ParamEntry::new_f32("TEST", 1.0, 0.0, 10.0));

        let mut buf = [0u8; 256];
        let len = serialize_params(&store, &mut buf).unwrap();

        // Corrupt the CRC
        buf[len - 1] ^= 0xFF;
        assert!(deserialize_params(&buf[..len]).is_none());
    }

    #[test]
    fn test_deserialize_too_short_fails() {
        assert!(deserialize_params(&[0u8; 4]).is_none());
    }

    #[test]
    fn test_crc32_known_value() {
        // CRC32 of empty data should be 0x00000000... no, standard CRC32("") = 0x00000000
        // Actually CRC32 of "" is 0x00000000 with the XOR step applied.
        let c = crc32(b"");
        assert_eq!(c, 0x00000000);
        // CRC32 of "123456789" is a well-known test vector: 0xCBF43926
        let c2 = crc32(b"123456789");
        assert_eq!(c2, 0xCBF43926);
    }

    // ─── FileParamBackend Tests ───

    #[test]
    fn test_file_backend_save_and_load() {
        let mut store = ParamStore::new();
        store.register_defaults();
        store.set("FENCE_RADIUS", 500.0);

        // Serialize
        let mut buf = [0u8; 4096];
        let len = serialize_params(&store, &mut buf).unwrap();

        // Create backend with that data
        let backend = FileParamBackend { buf: &mut buf, len };
        let loaded = backend.load().unwrap();

        let fence = loaded.iter().find(|(n, _)| n.as_str() == "FENCE_RADIUS").unwrap();
        assert!((fence.1.as_f32() - 500.0).abs() < 1e-3);
    }

    #[test]
    fn test_file_backend_empty_returns_none() {
        let mut buf = [0u8; 256];
        let backend = FileParamBackend::new(&mut buf, 0);
        assert!(backend.load().is_none());
    }

    // ─── FlashParamBackend Tests ───

    #[test]
    fn test_flash_backend_save_to_inactive_sector() {
        let mut sector_a = [0xFFu8; 4096];
        let mut sector_b = [0xFFu8; 4096];

        let mut store = ParamStore::new();
        store.register(ParamEntry::new_f32("TEST_P", 3.14, 0.0, 10.0));

        let mut flash = FlashParamBackend::new(&mut sector_a, &mut sector_b);
        // Active is A by default, so save writes to B
        assert!(flash.save_mut(&store));
        assert!(flash.active_is_b); // swapped to B after write

        // Load should succeed (now active is B)
        let loaded = flash.load().unwrap();
        assert_eq!(loaded.len(), 1);
        assert_eq!(loaded[0].0.as_str(), "TEST_P");
        assert!((loaded[0].1.as_f32() - 3.14).abs() < 1e-5);
    }

    #[test]
    fn test_flash_backend_ping_pong() {
        let mut sector_a = [0xFFu8; 4096];
        let mut sector_b = [0xFFu8; 4096];

        let mut store = ParamStore::new();
        store.register(ParamEntry::new_f32("VAL", 1.0, 0.0, 100.0));

        let mut flash = FlashParamBackend::new(&mut sector_a, &mut sector_b);

        // First save: A active -> writes B, swaps to B
        assert!(flash.save_mut(&store));
        assert!(flash.active_is_b);

        // Modify and save again: B active -> writes A, swaps to A
        store.set("VAL", 2.0);
        assert!(flash.save_mut(&store));
        assert!(flash.active_is_a());

        // Load should return the latest (2.0) from A
        let loaded = flash.load().unwrap();
        assert!((loaded[0].1.as_f32() - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_flash_backend_corruption_fallback() {
        let mut sector_a = [0xFFu8; 4096];
        let mut sector_b = [0xFFu8; 4096];

        let mut store = ParamStore::new();
        store.register(ParamEntry::new_f32("VAL", 1.0, 0.0, 100.0));

        let mut flash = FlashParamBackend::new(&mut sector_a, &mut sector_b);

        // Save to B (active is A)
        assert!(flash.save_mut(&store));
        // Now active is B. Save again with updated value to A.
        store.set("VAL", 42.0);
        assert!(flash.save_mut(&store));
        // Active is now A with value 42.0, B has value 1.0

        // Corrupt sector A (the active one)
        flash.sector_a[0] = 0x00;

        // Load should fall back to sector B (value 1.0)
        let loaded = flash.load().unwrap();
        assert!((loaded[0].1.as_f32() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_flash_backend_both_corrupt_returns_none() {
        let mut sector_a = [0xFFu8; 4096];
        let mut sector_b = [0xFFu8; 4096];

        let flash = FlashParamBackend::new(&mut sector_a, &mut sector_b);
        // Both sectors are 0xFF garbage — no valid data
        assert!(flash.load().is_none());
    }

    #[test]
    fn test_flash_detect_active_prefers_a() {
        let mut sector_a = [0xFFu8; 4096];
        let mut sector_b = [0xFFu8; 4096];

        let mut store = ParamStore::new();
        store.register(ParamEntry::new_f32("X", 1.0, 0.0, 10.0));

        // Write valid data to both sectors
        let _ = serialize_params(&store, &mut sector_a);
        store.set("X", 5.0);
        let _ = serialize_params(&store, &mut sector_b);

        let mut flash = FlashParamBackend::new(&mut sector_a, &mut sector_b);
        flash.detect_active();
        assert!(flash.active_is_a()); // prefers A when both valid
    }

    #[test]
    fn test_flash_detect_active_falls_back_to_b() {
        let mut sector_a = [0x00u8; 4096]; // corrupt
        let mut sector_b = [0xFFu8; 4096];

        let mut store = ParamStore::new();
        store.register(ParamEntry::new_f32("X", 1.0, 0.0, 10.0));
        let _ = serialize_params(&store, &mut sector_b);

        let mut flash = FlashParamBackend::new(&mut sector_a, &mut sector_b);
        flash.detect_active();
        assert!(flash.active_is_b); // A is corrupt, falls back to B
    }

    #[test]
    fn test_apply_loaded_values() {
        let mut store = ParamStore::new();
        store.register_defaults();

        let mut loaded = heapless::Vec::<(heapless::String<16>, ParamValue), MAX_PARAMS>::new();
        let mut name = heapless::String::<16>::new();
        let _ = name.push_str("ATC_RAT_RLL_P");
        let _ = loaded.push((name, ParamValue::F32(0.5)));

        store.apply_loaded(&loaded);

        let p = store.get("ATC_RAT_RLL_P").unwrap();
        assert!((p.value.as_f32() - 0.5).abs() < 1e-6);
        assert!(!store.is_dirty()); // apply_loaded marks clean
    }
}

