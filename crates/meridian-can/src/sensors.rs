//! Sensor integration adapters for DroneCAN peripherals.
//!
//! These adapters receive decoded DroneCAN messages and produce Meridian
//! message bus types (`GnssPosition`, `MagField`, `BaroPressure`, etc.).

use meridian_math::geodetic::LatLonAlt;
use meridian_math::{Vec3, Body, NED};
use meridian_types::messages::{GnssFixType, GnssPosition, MagField, BaroPressure};
use meridian_types::time::Instant;

use crate::messages::*;

// ---------------------------------------------------------------------------
// CanGps — receives Fix2, produces GnssPosition
// ---------------------------------------------------------------------------

/// CAN GPS sensor adapter.
///
/// Receives `uavcan.equipment.gnss.Fix2` (DTID 1063) messages from a
/// DroneCAN GPS peripheral and converts them to Meridian `GnssPosition`.
pub struct CanGps {
    /// Source node ID (0 = accept from any node).
    pub source_node: u8,
    /// Sensor instance index.
    pub instance: u8,
    /// Last decoded position.
    pub last_fix: Option<GnssPosition>,
    /// Timestamp of last update.
    pub last_update_us: u64,
}

impl CanGps {
    pub fn new(instance: u8) -> Self {
        Self {
            source_node: 0,
            instance,
            last_fix: None,
            last_update_us: 0,
        }
    }

    /// Process a decoded Fix2 message. Returns `Some(GnssPosition)` on success.
    pub fn process_fix2(&mut self, fix: &Fix2, now: Instant) -> Option<GnssPosition> {
        let fix_type = match (fix.status, fix.mode, fix.sub_mode) {
            (GnssFixStatus::NoFix, _, _) => GnssFixType::NoFix,
            (GnssFixStatus::Fix2D, _, _) => GnssFixType::Fix2D,
            (GnssFixStatus::Fix3D, GnssFixMode::Single, _) => GnssFixType::Fix3D,
            (GnssFixStatus::Fix3D, GnssFixMode::Dgps, _) => GnssFixType::DGps,
            (GnssFixStatus::Fix3D, GnssFixMode::Rtk, GnssSubMode::RtkFloat) => GnssFixType::RtkFloat,
            (GnssFixStatus::Fix3D, GnssFixMode::Rtk, GnssSubMode::RtkFixed) => GnssFixType::RtkFixed,
            (GnssFixStatus::Fix3D, GnssFixMode::Rtk, _) => GnssFixType::RtkFloat,
        };

        // Convert 1e8 scaled integers to radians
        let lat_deg = fix.latitude_deg_1e8 as f64 / 1e8;
        let lon_deg = fix.longitude_deg_1e8 as f64 / 1e8;
        let alt_m = fix.height_msl_mm as f64 / 1000.0;

        let position = LatLonAlt::from_degrees(lat_deg, lon_deg, alt_m);

        let velocity_ned = Vec3::<NED>::new(
            fix.ned_velocity[0],
            fix.ned_velocity[1],
            fix.ned_velocity[2],
        );

        // Extract accuracy from covariance diagonal (sqrt of variance)
        let h_acc = libm::sqrtf(fix.covariance[0].max(fix.covariance[1]));
        let v_acc = libm::sqrtf(fix.covariance[2]);
        let s_acc = libm::sqrtf(fix.covariance[3].max(fix.covariance[4]));

        let pos = GnssPosition {
            timestamp: now,
            fix_type,
            position,
            velocity_ned,
            horizontal_accuracy: h_acc,
            vertical_accuracy: v_acc,
            speed_accuracy: s_acc,
            num_sats: fix.sats_used,
        };

        self.last_fix = Some(pos);
        self.last_update_us = fix.timestamp_usec;
        Some(pos)
    }
}

// ---------------------------------------------------------------------------
// CanCompass — receives MagneticFieldStrength2, produces MagField
// ---------------------------------------------------------------------------

/// CAN compass sensor adapter.
///
/// Receives `uavcan.equipment.ahrs.MagneticFieldStrength2` (DTID 1002)
/// from a DroneCAN magnetometer and converts to Meridian `MagField`.
pub struct CanCompass {
    /// Source node ID (0 = accept from any).
    pub source_node: u8,
    /// Sensor instance index.
    pub instance: u8,
    /// Last decoded magnetic field.
    pub last_field: Option<MagField>,
}

impl CanCompass {
    pub fn new(instance: u8) -> Self {
        Self {
            source_node: 0,
            instance,
            last_field: None,
        }
    }

    /// Process a decoded MagneticFieldStrength2 message.
    pub fn process_mag(&mut self, msg: &MagneticFieldStrength2, now: Instant) -> Option<MagField> {
        let field = Vec3::<Body>::new(
            msg.magnetic_field_ga[0],
            msg.magnetic_field_ga[1],
            msg.magnetic_field_ga[2],
        );

        let result = MagField {
            timestamp: now,
            mag_index: self.instance,
            field,
        };

        self.last_field = Some(result);
        Some(result)
    }
}

// ---------------------------------------------------------------------------
// CanBaro — receives RawAirData, produces BaroPressure
// ---------------------------------------------------------------------------

/// CAN barometer sensor adapter.
///
/// Receives `uavcan.equipment.air_data.RawAirData` (DTID 1027) from a
/// DroneCAN barometer and converts to Meridian `BaroPressure`.
pub struct CanBaro {
    /// Source node ID (0 = accept from any).
    pub source_node: u8,
    /// Sensor instance index.
    pub instance: u8,
    /// Last decoded baro reading.
    pub last_reading: Option<BaroPressure>,
}

impl CanBaro {
    pub fn new(instance: u8) -> Self {
        Self {
            source_node: 0,
            instance,
            last_reading: None,
        }
    }

    /// Process a decoded RawAirData message.
    pub fn process_air_data(&mut self, msg: &RawAirData, now: Instant) -> Option<BaroPressure> {
        // Convert temperature from Kelvin to Celsius
        let temp_c = msg.static_air_temperature - 273.15;

        // Compute altitude from pressure using international standard atmosphere.
        // h = 44330 * (1 - (P/P0)^(1/5.255))
        // P0 = 101325 Pa (sea level standard pressure)
        let p0 = 101325.0f32;
        let ratio = msg.static_pressure / p0;
        let altitude_m = 44330.0 * (1.0 - libm::powf(ratio, 1.0 / 5.255));

        let reading = BaroPressure {
            timestamp: now,
            baro_index: self.instance,
            pressure_pa: msg.static_pressure,
            temperature: temp_c,
            altitude_m,
        };

        self.last_reading = Some(reading);
        Some(reading)
    }
}

// ---------------------------------------------------------------------------
// CanBattery — receives BatteryInfo
// ---------------------------------------------------------------------------

/// CAN battery monitor adapter.
///
/// Receives `uavcan.equipment.power.BatteryInfo` (DTID 1092).
/// Stores the latest battery state for the vehicle state machine.
#[derive(Debug, Clone)]
pub struct CanBattery {
    /// Source node ID (0 = accept from any).
    pub source_node: u8,
    /// Latest voltage in volts.
    pub voltage: f32,
    /// Latest current in amps (positive = discharging in our convention).
    pub current: f32,
    /// Remaining charge percentage (0-100).
    pub remaining_pct: f32,
    /// Temperature in Celsius.
    pub temperature_c: f32,
    /// Whether we have received at least one update.
    pub valid: bool,
}

impl CanBattery {
    pub fn new() -> Self {
        Self {
            source_node: 0,
            voltage: 0.0,
            current: 0.0,
            remaining_pct: 0.0,
            temperature_c: 0.0,
            valid: false,
        }
    }

    /// Process a decoded BatteryInfo message.
    pub fn process_battery_info(&mut self, msg: &BatteryInfo) {
        self.voltage = msg.voltage;
        // DroneCAN: positive current = charging. Meridian convention: positive = discharging.
        self.current = -msg.current;
        self.remaining_pct = msg.state_of_charge_pct as f32;
        self.temperature_c = msg.temperature_k - 273.15;
        self.valid = true;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_instant(us: u64) -> Instant {
        // Instant is a wrapper around u64 microseconds
        Instant::from_micros(us)
    }

    #[test]
    fn test_can_gps_fix2_3d() {
        let mut gps = CanGps::new(0);

        let fix = Fix2 {
            timestamp_usec: 1_000_000,
            longitude_deg_1e8: -11739816200,
            latitude_deg_1e8: 3386668700,
            height_ellipsoid_mm: 150_000,
            height_msl_mm: 148_500,
            ned_velocity: [1.5, -0.5, 0.1],
            sats_used: 12,
            status: GnssFixStatus::Fix3D,
            mode: GnssFixMode::Single,
            sub_mode: GnssSubMode::DgpsOther,
            covariance: [2.25, 2.25, 9.0, 0.04, 0.04, 0.09],
        };

        let now = make_instant(1_000_000);
        let pos = gps.process_fix2(&fix, now).unwrap();

        assert_eq!(pos.fix_type, GnssFixType::Fix3D);
        assert_eq!(pos.num_sats, 12);
        assert!((pos.velocity_ned.x - 1.5).abs() < 1e-6);
        assert!((pos.velocity_ned.y - (-0.5)).abs() < 1e-6);
        // horizontal_accuracy = sqrt(max(2.25, 2.25)) = 1.5
        assert!((pos.horizontal_accuracy - 1.5).abs() < 0.01);
        // vertical_accuracy = sqrt(9.0) = 3.0
        assert!((pos.vertical_accuracy - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_can_gps_rtk_fixed() {
        let mut gps = CanGps::new(0);
        let fix = Fix2 {
            timestamp_usec: 2_000_000,
            longitude_deg_1e8: 0,
            latitude_deg_1e8: 0,
            height_ellipsoid_mm: 0,
            height_msl_mm: 0,
            ned_velocity: [0.0; 3],
            sats_used: 20,
            status: GnssFixStatus::Fix3D,
            mode: GnssFixMode::Rtk,
            sub_mode: GnssSubMode::RtkFixed,
            covariance: [0.0; 6],
        };

        let pos = gps.process_fix2(&fix, make_instant(2_000_000)).unwrap();
        assert_eq!(pos.fix_type, GnssFixType::RtkFixed);
    }

    #[test]
    fn test_can_compass() {
        let mut compass = CanCompass::new(0);
        let msg = MagneticFieldStrength2 {
            sensor_id: 0,
            magnetic_field_ga: [0.25, -0.13, 0.45],
        };

        let field = compass.process_mag(&msg, make_instant(100_000)).unwrap();
        assert_eq!(field.mag_index, 0);
        assert!((field.field.x - 0.25).abs() < 0.001);
        assert!((field.field.y - (-0.13)).abs() < 0.001);
        assert!((field.field.z - 0.45).abs() < 0.001);
    }

    #[test]
    fn test_can_baro() {
        let mut baro = CanBaro::new(0);
        let msg = RawAirData {
            static_pressure: 101325.0,
            differential_pressure: 0.0,
            static_air_temperature: 293.15, // 20 C
            pitot_temperature: 0.0,
        };

        let reading = baro.process_air_data(&msg, make_instant(200_000)).unwrap();
        assert!((reading.pressure_pa - 101325.0).abs() < 1.0);
        assert!((reading.temperature - 20.0).abs() < 0.01);
        // At sea level pressure, altitude should be ~0
        assert!(reading.altitude_m.abs() < 1.0);
    }

    #[test]
    fn test_can_baro_altitude() {
        let mut baro = CanBaro::new(0);
        // Approximately 1000m altitude pressure
        let msg = RawAirData {
            static_pressure: 89876.0,
            differential_pressure: 0.0,
            static_air_temperature: 281.65, // ~8.5 C
            pitot_temperature: 0.0,
        };

        let reading = baro.process_air_data(&msg, make_instant(300_000)).unwrap();
        // Should be roughly 1000m
        assert!((reading.altitude_m - 1000.0).abs() < 50.0);
    }

    #[test]
    fn test_can_battery() {
        let mut batt = CanBattery::new();
        assert!(!batt.valid);

        let msg = BatteryInfo {
            temperature_k: 298.15,
            voltage: 22.2,
            current: -5.5, // DroneCAN: negative = discharging
            full_charge_capacity_wh: 100.0,
            remaining_capacity_wh: 75.0,
            state_of_charge_pct: 75,
            state_of_health_pct: 95,
            battery_id: 0,
            model_instance_id: 0,
        };

        batt.process_battery_info(&msg);
        assert!(batt.valid);
        assert!((batt.voltage - 22.2).abs() < 0.01);
        assert!((batt.current - 5.5).abs() < 0.01); // flipped sign
        assert!((batt.remaining_pct - 75.0).abs() < 0.01);
        assert!((batt.temperature_c - 25.0).abs() < 0.01);
    }
}
