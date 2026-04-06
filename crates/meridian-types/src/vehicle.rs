use crate::time::Instant;

/// Top-level vehicle lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleLifecycle {
    /// Power on, initializing sensors, running pre-arm checks.
    Booting,
    /// Sensors online, waiting for arm command.
    Disarmed,
    /// Armed. Motors can spin.
    Armed { mode: FlightModeId },
    /// In flight.
    Flying { mode: FlightModeId, takeoff_time: Instant },
    /// Emergency state. Only failsafe actions permitted.
    Emergency { reason: FailsafeReason },
}

/// Every flight mode. Exhaustive matching required.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FlightModeId {
    Stabilize,
    Acro,
    AltHold,
    Loiter,
    RTL,
    Auto,
    Guided,
    PosHold,
    Land,
    Circle,
    Brake,
    Drift,
    Sport,
    Flip,
    Follow,
    ZigZag,
    Throw,
    GuidedNoGPS,
    SmartRTL,
    // Copter-only (stubs for parity)
    Autotune,
    SystemId,
    Turtle,
    FlowHold,
    AvoidAdsb,
    // Fixed-wing
    FlyByWireA,
    FlyByWireB,
    Cruise,
    Takeoff,
    FwAcro,
    Training,
    Thermal,
    // VTOL
    VtolTransitionToFw,
    VtolTransitionToHover,
    // Rover
    Steering,
    Hold,
    RoverSimple,
    RoverDock,
    // Sub
    SubAcro,
    SubAuto,
    SubGuided,
    SubPosHold,
    SurfTrak,
    // Common
    Manual,
}

impl FlightModeId {
    pub fn requires_gps(&self) -> bool {
        matches!(
            self,
            Self::Loiter | Self::RTL | Self::Auto | Self::Guided
                | Self::PosHold | Self::Circle | Self::Follow
                | Self::SmartRTL | Self::Cruise | Self::ZigZag
                | Self::Takeoff | Self::SubAuto | Self::SubGuided
                | Self::SubPosHold | Self::SurfTrak
                | Self::RoverSimple | Self::RoverDock
                | Self::AvoidAdsb
        )
    }

    pub fn requires_position(&self) -> bool {
        self.requires_gps() || matches!(self, Self::AltHold | Self::Brake | Self::Sport | Self::Drift | Self::FlowHold)
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Stabilize => "STABILIZE",
            Self::Acro => "ACRO",
            Self::AltHold => "ALT_HOLD",
            Self::Loiter => "LOITER",
            Self::RTL => "RTL",
            Self::Auto => "AUTO",
            Self::Guided => "GUIDED",
            Self::PosHold => "POSHOLD",
            Self::Land => "LAND",
            Self::Circle => "CIRCLE",
            Self::Brake => "BRAKE",
            Self::Drift => "DRIFT",
            Self::Sport => "SPORT",
            Self::Flip => "FLIP",
            Self::Follow => "FOLLOW",
            Self::ZigZag => "ZIGZAG",
            Self::Throw => "THROW",
            Self::GuidedNoGPS => "GUIDED_NOGPS",
            Self::SmartRTL => "SMART_RTL",
            Self::Autotune => "AUTOTUNE",
            Self::SystemId => "SYSTEMID",
            Self::Turtle => "TURTLE",
            Self::FlowHold => "FLOWHOLD",
            Self::AvoidAdsb => "AVOID_ADSB",
            Self::FlyByWireA => "FBWA",
            Self::FlyByWireB => "FBWB",
            Self::Cruise => "CRUISE",
            Self::Takeoff => "TAKEOFF",
            Self::FwAcro => "FW_ACRO",
            Self::Training => "TRAINING",
            Self::Thermal => "THERMAL",
            Self::VtolTransitionToFw => "VTOL_TO_FW",
            Self::VtolTransitionToHover => "VTOL_TO_HOVER",
            Self::Steering => "STEERING",
            Self::Hold => "HOLD",
            Self::RoverSimple => "SIMPLE",
            Self::RoverDock => "DOCK",
            Self::SubAcro => "SUB_ACRO",
            Self::SubAuto => "SUB_AUTO",
            Self::SubGuided => "SUB_GUIDED",
            Self::SubPosHold => "SUB_POSHOLD",
            Self::SurfTrak => "SURFTRAK",
            Self::Manual => "MANUAL",
        }
    }

    /// ArduPilot mode number → FlightModeId.
    /// Source: mode.h enum class Mode::Number
    pub fn from_number(n: u8) -> Self {
        match n {
            0 => Self::Stabilize,
            1 => Self::Acro,
            2 => Self::AltHold,
            3 => Self::Auto,
            4 => Self::Guided,
            5 => Self::Loiter,
            6 => Self::RTL,
            7 => Self::Circle,
            9 => Self::Land,
            11 => Self::Drift,
            13 => Self::Sport,
            14 => Self::Flip,
            15 => Self::Autotune,
            16 => Self::PosHold,
            17 => Self::Brake,
            18 => Self::Throw,
            19 => Self::AvoidAdsb,
            20 => Self::GuidedNoGPS,
            21 => Self::SmartRTL,
            22 => Self::FlowHold,
            23 => Self::Follow,
            24 => Self::ZigZag,
            25 => Self::SystemId,
            // 26 = Autorotate (not mapped)
            // 27 = AUTO_RTL (pseudo-mode)
            28 => Self::Turtle,
            // Fixed-wing
            50 => Self::FlyByWireA,
            51 => Self::FlyByWireB,
            52 => Self::Cruise,
            53 => Self::FwAcro,
            54 => Self::Training,
            55 => Self::Takeoff,
            56 => Self::Thermal,
            // Rover
            60 => Self::Steering,
            61 => Self::Hold,
            62 => Self::Manual,
            63 => Self::RoverSimple,
            64 => Self::RoverDock,
            // Sub
            70 => Self::SubAcro,
            71 => Self::SubAuto,
            72 => Self::SubGuided,
            73 => Self::SubPosHold,
            74 => Self::SurfTrak,
            _ => Self::Loiter, // default fallback
        }
    }

    /// FlightModeId → ArduPilot mode number.
    pub fn to_number(&self) -> u8 {
        match self {
            Self::Stabilize => 0,
            Self::Acro => 1,
            Self::AltHold => 2,
            Self::Auto => 3,
            Self::Guided => 4,
            Self::Loiter => 5,
            Self::RTL => 6,
            Self::Circle => 7,
            Self::Land => 9,
            Self::Drift => 11,
            Self::Sport => 13,
            Self::Flip => 14,
            Self::Autotune => 15,
            Self::PosHold => 16,
            Self::Brake => 17,
            Self::Throw => 18,
            Self::AvoidAdsb => 19,
            Self::GuidedNoGPS => 20,
            Self::SmartRTL => 21,
            Self::FlowHold => 22,
            Self::Follow => 23,
            Self::ZigZag => 24,
            Self::SystemId => 25,
            Self::Turtle => 28,
            Self::FlyByWireA => 50,
            Self::FlyByWireB => 51,
            Self::Cruise => 52,
            Self::FwAcro => 53,
            Self::Training => 54,
            Self::Takeoff => 55,
            Self::Thermal => 56,
            Self::VtolTransitionToFw => 57,
            Self::VtolTransitionToHover => 58,
            Self::Steering => 60,
            Self::Hold => 61,
            Self::Manual => 62,
            Self::RoverSimple => 63,
            Self::RoverDock => 64,
            Self::SubAcro => 70,
            Self::SubAuto => 71,
            Self::SubGuided => 72,
            Self::SubPosHold => 73,
            Self::SurfTrak => 74,
        }
    }
}

/// Failsafe trigger reasons.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FailsafeReason {
    RcLoss,
    GnssLoss,
    EkfFailure,
    BatteryLow,
    BatteryCritical,
    GeofenceBreach,
    TerrainBreach,
    MotorFailure,
    CommsLoss,
    DeadReckoning,
    TerrainStale,
    WatchdogTimeout,
    GpsGlitch,
    Crash,
    /// Vehicle cannot maintain altitude despite full throttle (motor/prop loss).
    ThrustLoss,
    /// Excessive yaw correction required (damaged motor or prop).
    YawImbalance,
}

/// Failsafe response actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FailsafeAction {
    Warn,
    ReturnToLaunch,
    SmartReturnToLaunch,
    Land,
    Terminate,
    /// FS_OPTIONS bit 4: try SmartRTL, fall back to Land.
    SmartRtlLand,
    /// FS_OPTIONS bit 5: try Brake, fall back to Land.
    BrakeLand,
    /// FS_OPTIONS bit 6: jump to mission's DO_LAND_START item.
    DoLandStart,
}
