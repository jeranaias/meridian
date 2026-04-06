#![no_std]

//! Behavior tree mission engine — arena-allocated, no_std compatible.
//!
//! Replaces ArduPilot's linear waypoint list with a composable behavior tree.
//! Simple missions (waypoint list) compile to a flat Sequence.

use meridian_types::vehicle::FlightModeId;
use meridian_types::time::Duration;

/// Maximum nodes in a tree.
pub const MAX_TREE_NODES: usize = 64;
/// Maximum children per composite node.
pub const MAX_CHILDREN: usize = 8;

/// Node identifier — index into the tree's arena.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeId(pub u16);

/// Behavior tree status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BtStatus {
    Running,
    Success,
    Failure,
}

/// Condition kinds (checked each tick).
#[derive(Debug, Clone, Copy)]
pub enum ConditionKind {
    BatteryAbove(f32),      // percent
    AltitudeBelow(f32),     // meters
    AltitudeAbove(f32),     // meters
    DistanceWithin(f32),    // meters from target
    SpeedBelow(f32),        // m/s
    WindBelow(f32),         // m/s (estimated wind speed)
    GpsAvailable,
    EkfHealthy,
    GeofenceOk,             // not breached
    CommsActive,            // GCS heartbeat recent
    Armed,
    OnGround,
}

/// Action kinds (executed until complete).
/// Full NAV/DO command set — covers ArduPilot's AP_Mission MAV_CMD IDs.
/// GAP 32: expanded to include all ~65 MAV_CMD IDs from ArduPilot.
#[derive(Debug, Clone, Copy)]
pub enum ActionKind {
    // ── NAV commands (move the vehicle) ──
    Waypoint { n: f32, e: f32, d: f32, radius: f32 },
    Takeoff(f32),           // altitude meters
    Land,
    RTL,
    LoiterUnlim { n: f32, e: f32, d: f32 },
    LoiterTurns { n: f32, e: f32, d: f32, turns: f32 },
    LoiterTime { n: f32, e: f32, d: f32, time_s: f32 },
    LoiterToAlt { alt: f32 },
    SplineWaypoint { n: f32, e: f32, d: f32 },
    Delay(u32),             // ms
    GuidedEnable(bool),
    SetYaw { angle_deg: f32, rate_dps: f32, relative: bool },
    ConditionDistance(f32),

    // GAP 32: new NAV commands
    /// MAV_CMD_NAV_ARC_WAYPOINT — arc path between waypoints (fixed-wing)
    ArcWaypoint { n: f32, e: f32, d: f32, radius: f32 },
    /// MAV_CMD_NAV_ALTITUDE_WAIT — wait at current position until altitude change
    AltitudeWait { alt: f32, descent_rate: f32, wiggle_time: f32 },
    /// MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT — continue nav while changing altitude
    ContinueAndChangeAlt { alt: f32 },
    /// MAV_CMD_NAV_SET_YAW_SPEED — set yaw and speed simultaneously
    NavSetYawSpeed { yaw_deg: f32, speed: f32 },
    /// MAV_CMD_NAV_PAYLOAD_PLACE — precision payload placement
    PayloadPlace { max_descent: f32 },
    /// MAV_CMD_NAV_SCRIPT_TIME — Lua scripting time-based nav
    ScriptTime { cmd: u16, timeout_s: f32, arg1: f32, arg2: f32 },
    /// MAV_CMD_NAV_ATTITUDE_TIME — hold attitude for time
    AttitudeTime { time_s: f32, roll_deg: f32, pitch_deg: f32, yaw_deg: f32 },
    /// MAV_CMD_NAV_VTOL_TAKEOFF — QuadPlane VTOL takeoff
    VtolTakeoff { alt: f32 },
    /// MAV_CMD_NAV_VTOL_LAND — QuadPlane VTOL landing
    VtolLand { n: f32, e: f32, approach_alt: f32 },
    /// MAV_CMD_NAV_RALLY_POINT — rally point definition in mission
    RallyPoint { n: f32, e: f32, d: f32 },

    // GAP 32: Condition commands
    /// MAV_CMD_CONDITION_DELAY — wait N seconds
    ConditionDelay { seconds: f32 },
    /// MAV_CMD_CONDITION_YAW — wait for yaw condition
    ConditionYaw { angle_deg: f32, rate_dps: f32, direction: i8, relative: bool },

    // GAP 32: NAV_FENCE commands
    /// MAV_CMD_NAV_FENCE_RETURN_POINT
    FenceReturnPoint { n: f32, e: f32, d: f32 },
    /// MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
    FencePolygonInclusion { n: f32, e: f32, count: u8 },
    /// MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
    FencePolygonExclusion { n: f32, e: f32, count: u8 },
    /// MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
    FenceCircleInclusion { n: f32, e: f32, radius: f32 },
    /// MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
    FenceCircleExclusion { n: f32, e: f32, radius: f32 },

    // ── DO commands (fire alongside NAV) ──
    SetMode(FlightModeId),
    SetSpeed(f32),
    Jump { target: u16, repeat: i16 },
    JumpTag(u16),
    SetRelay { relay: u8, state: bool },
    RepeatRelay { relay: u8, count: u16, cycle_s: f32 },
    SetServo { channel: u8, pwm: u16 },
    RepeatServo { channel: u8, pwm: u16, count: u16, cycle_s: f32 },
    CameraShutter,
    CameraVideo(bool),
    GripperRelease,
    MountControl { pitch_deg: f32, roll_deg: f32, yaw_deg: f32 },
    SetRoi { n: f32, e: f32, d: f32 },
    SetHome { use_current: bool, n: f32, e: f32, d: f32 },
    DoLandStart,
    ChangeSpeed { speed_type: u8, speed: f32, throttle: f32 },
    ChangeAlt { alt: f32, rate: f32 },
    Digicam { cmd: u8 },
    Parachute(u8),
    EngineControl { start: bool, cold: bool },
    SetReverse(bool),
    FenceEnable(bool),
    AuxFunction { function: u16, switch_pos: u8 },

    // GAP 32: new DO commands
    /// MAV_CMD_DO_AUTOTUNE_ENABLE — enable/disable autotune from mission
    AutotuneEnable(bool),
    /// MAV_CMD_DO_GUIDED_LIMITS — set guided mode limits
    GuidedLimits { timeout_s: f32, alt_min: f32, alt_max: f32, horiz_max: f32 },
    /// MAV_CMD_DO_INVERTED_FLIGHT — fixed-wing inverted flight
    InvertedFlight(bool),
    /// MAV_CMD_DO_GO_AROUND — fixed-wing go-around / abort landing
    GoAround { alt: f32 },
    /// MAV_CMD_DO_VTOL_TRANSITION — QuadPlane mode transition
    VtolTransition { target_state: u8 },
    /// MAV_CMD_DO_WINCH — winch control
    Winch { action: u8, length: f32, rate: f32 },
    /// MAV_CMD_DO_SPRAYER — agricultural sprayer on/off
    Sprayer(bool),
    /// MAV_CMD_DO_SEND_SCRIPT_MESSAGE — Lua scripting message
    SendScriptMessage { id: u16, p1: f32, p2: f32, p3: f32 },
    /// MAV_CMD_DO_SET_RESUME_REPEAT_DIST — resume distance for repeat
    SetResumeRepeatDist { dist: f32 },
    /// MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW — gimbal control
    GimbalManagerPitchYaw { pitch_deg: f32, yaw_deg: f32, pitch_rate: f32, yaw_rate: f32 },
    /// MAV_CMD_DO_PAUSE_CONTINUE — in-mission pause/continue
    PauseContinue { pause: bool },
    /// MAV_CMD_DO_RETURN_PATH_START — mark return path start
    ReturnPathStart,
    /// Camera capture commands
    ImageStartCapture { interval_s: f32, count: u32 },
    ImageStopCapture,
    VideoStartCapture { freq_hz: f32 },
    VideoStopCapture,
    /// Camera control
    SetCameraZoom { zoom_type: u8, zoom_value: f32 },
    SetCameraFocus { focus_type: u8, focus_value: f32 },
    SetCameraSource { device_id: u8, primary: u8, secondary: u8 },
}

/// Whether a command is a NAV command (blocks the nav cursor) or a DO command (runs alongside).
impl ActionKind {
    pub fn is_nav(&self) -> bool {
        matches!(self,
            ActionKind::Waypoint { .. } | ActionKind::Takeoff(_) | ActionKind::Land |
            ActionKind::RTL | ActionKind::LoiterUnlim { .. } | ActionKind::LoiterTurns { .. } |
            ActionKind::LoiterTime { .. } | ActionKind::LoiterToAlt { .. } |
            ActionKind::SplineWaypoint { .. } | ActionKind::Delay(_) |
            ActionKind::ConditionDistance(_) |
            ActionKind::ArcWaypoint { .. } | ActionKind::AltitudeWait { .. } |
            ActionKind::ContinueAndChangeAlt { .. } | ActionKind::NavSetYawSpeed { .. } |
            ActionKind::PayloadPlace { .. } | ActionKind::ScriptTime { .. } |
            ActionKind::AttitudeTime { .. } | ActionKind::VtolTakeoff { .. } |
            ActionKind::VtolLand { .. } | ActionKind::RallyPoint { .. } |
            ActionKind::ConditionDelay { .. } | ActionKind::ConditionYaw { .. } |
            ActionKind::FenceReturnPoint { .. } |
            ActionKind::FencePolygonInclusion { .. } | ActionKind::FencePolygonExclusion { .. } |
            ActionKind::FenceCircleInclusion { .. } | ActionKind::FenceCircleExclusion { .. }
        )
    }

    pub fn is_do(&self) -> bool {
        !self.is_nav()
    }
}

/// Node data in the arena.
#[derive(Debug, Clone)]
pub enum NodeData {
    Sequence(heapless::Vec<NodeId, MAX_CHILDREN>),
    Fallback(heapless::Vec<NodeId, MAX_CHILDREN>),
    Parallel { children: heapless::Vec<NodeId, MAX_CHILDREN>, success_threshold: u8 },
    RepeatUntilFail(NodeId),
    Inverter(NodeId),
    Condition(ConditionKind),
    Action(ActionKind),
}

/// A node in the tree with its current status.
#[derive(Debug, Clone)]
pub struct TreeNode {
    pub data: NodeData,
    pub status: BtStatus,
}

/// The behavior tree: a flat arena of nodes with a root pointer.
pub struct BehaviorTree {
    nodes: heapless::Vec<TreeNode, MAX_TREE_NODES>,
    root: NodeId,
}

/// Context provided to the tree each tick.
pub struct BtContext {
    pub battery_pct: f32,
    pub altitude: f32,
    pub distance_to_wp: f32,
    pub speed: f32,
    pub wind_speed: f32,
    pub gps_available: bool,
    pub ekf_healthy: bool,
    pub geofence_ok: bool,
    pub comms_active: bool,
    pub armed: bool,
    pub elapsed_ms: u32,
}

impl BehaviorTree {
    pub fn new() -> Self {
        Self {
            nodes: heapless::Vec::new(),
            root: NodeId(0),
        }
    }

    /// Add a node, returns its ID.
    pub fn add_node(&mut self, data: NodeData) -> NodeId {
        let id = NodeId(self.nodes.len() as u16);
        let _ = self.nodes.push(TreeNode { data, status: BtStatus::Running });
        id
    }

    /// Set the root node.
    pub fn set_root(&mut self, id: NodeId) {
        self.root = id;
    }

    /// Tick the tree. Returns the root node's status.
    pub fn tick(&mut self, ctx: &BtContext) -> BtStatus {
        self.tick_node(self.root, ctx)
    }

    fn tick_node(&mut self, id: NodeId, ctx: &BtContext) -> BtStatus {
        let idx = id.0 as usize;
        if idx >= self.nodes.len() { return BtStatus::Failure; }

        // Clone data to avoid borrow issues
        let data = self.nodes[idx].data.clone();

        let status = match data {
            NodeData::Condition(kind) => self.eval_condition(&kind, ctx),
            NodeData::Action(kind) => self.eval_action(&kind, ctx),
            NodeData::Sequence(children) => {
                let mut result = BtStatus::Success;
                for &child in children.iter() {
                    let s = self.tick_node(child, ctx);
                    if s == BtStatus::Failure { result = BtStatus::Failure; break; }
                    if s == BtStatus::Running { result = BtStatus::Running; break; }
                }
                result
            }
            NodeData::Fallback(children) => {
                let mut result = BtStatus::Failure;
                for &child in children.iter() {
                    let s = self.tick_node(child, ctx);
                    if s == BtStatus::Success { result = BtStatus::Success; break; }
                    if s == BtStatus::Running { result = BtStatus::Running; break; }
                }
                result
            }
            NodeData::Parallel { children, success_threshold } => {
                let mut successes = 0u8;
                let mut any_running = false;
                for &child in children.iter() {
                    match self.tick_node(child, ctx) {
                        BtStatus::Success => successes += 1,
                        BtStatus::Running => any_running = true,
                        BtStatus::Failure => {}
                    }
                }
                if successes >= success_threshold { BtStatus::Success }
                else if any_running { BtStatus::Running }
                else { BtStatus::Failure }
            }
            NodeData::Inverter(child) => {
                match self.tick_node(child, ctx) {
                    BtStatus::Success => BtStatus::Failure,
                    BtStatus::Failure => BtStatus::Success,
                    BtStatus::Running => BtStatus::Running,
                }
            }
            NodeData::RepeatUntilFail(child) => {
                match self.tick_node(child, ctx) {
                    BtStatus::Failure => BtStatus::Success,
                    _ => BtStatus::Running,
                }
            }
        };

        self.nodes[idx].status = status;
        status
    }

    fn eval_condition(&self, kind: &ConditionKind, ctx: &BtContext) -> BtStatus {
        let pass = match kind {
            ConditionKind::BatteryAbove(pct) => ctx.battery_pct > *pct,
            ConditionKind::AltitudeBelow(alt) => ctx.altitude < *alt,
            ConditionKind::AltitudeAbove(alt) => ctx.altitude > *alt,
            ConditionKind::DistanceWithin(dist) => ctx.distance_to_wp < *dist,
            ConditionKind::SpeedBelow(spd) => ctx.speed < *spd,
            ConditionKind::WindBelow(spd) => ctx.wind_speed < *spd,
            ConditionKind::GpsAvailable => ctx.gps_available,
            ConditionKind::EkfHealthy => ctx.ekf_healthy,
            ConditionKind::GeofenceOk => ctx.geofence_ok,
            ConditionKind::CommsActive => ctx.comms_active,
            ConditionKind::Armed => ctx.armed,
            ConditionKind::OnGround => ctx.altitude < 0.5,
        };
        if pass { BtStatus::Success } else { BtStatus::Failure }
    }

    fn eval_action(&self, kind: &ActionKind, ctx: &BtContext) -> BtStatus {
        // Actions return Running until their completion condition is met
        match kind {
            ActionKind::Waypoint { n: _, e: _, d: _, radius } => {
                if ctx.distance_to_wp < *radius { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::Takeoff(alt) => {
                if ctx.altitude >= *alt * 0.95 { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::Land => {
                if ctx.altitude < 0.3 { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::RTL => BtStatus::Running,
            ActionKind::LoiterUnlim { .. } => BtStatus::Running,
            ActionKind::LoiterTurns { .. } => BtStatus::Running,
            ActionKind::LoiterTime { time_s, .. } => {
                if ctx.elapsed_ms >= (*time_s * 1000.0) as u32 { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::LoiterToAlt { alt } => {
                if (ctx.altitude - *alt).abs() < 1.0 { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::SplineWaypoint { .. } => {
                if ctx.distance_to_wp < 2.0 { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::GuidedEnable(_) => BtStatus::Success,
            ActionKind::SetYaw { .. } => BtStatus::Success,
            ActionKind::ConditionDistance(dist) => {
                if ctx.distance_to_wp < *dist { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::SetMode(_) => BtStatus::Success,
            ActionKind::SetSpeed(_) | ActionKind::ChangeSpeed { .. } | ActionKind::ChangeAlt { .. } => BtStatus::Success,
            ActionKind::CameraShutter | ActionKind::Digicam { .. } => BtStatus::Success,
            ActionKind::CameraVideo(_) => BtStatus::Success,
            ActionKind::GripperRelease => BtStatus::Success,
            ActionKind::MountControl { .. } | ActionKind::SetRoi { .. } => BtStatus::Success,
            ActionKind::SetHome { .. } => BtStatus::Success,
            ActionKind::DoLandStart => BtStatus::Success,
            ActionKind::Jump { .. } | ActionKind::JumpTag(_) => BtStatus::Success,
            ActionKind::SetRelay { .. } | ActionKind::RepeatRelay { .. } => BtStatus::Success,
            ActionKind::SetServo { .. } | ActionKind::RepeatServo { .. } => BtStatus::Success,
            ActionKind::Parachute(_) | ActionKind::EngineControl { .. } => BtStatus::Success,
            ActionKind::SetReverse(_) | ActionKind::FenceEnable(_) => BtStatus::Success,
            ActionKind::AuxFunction { .. } => BtStatus::Success,
            ActionKind::Delay(ms) => {
                if ctx.elapsed_ms >= *ms { BtStatus::Success }
                else { BtStatus::Running }
            }
            // New NAV actions: running until completed by external system
            ActionKind::ArcWaypoint { .. } | ActionKind::AltitudeWait { .. }
            | ActionKind::ContinueAndChangeAlt { .. } | ActionKind::NavSetYawSpeed { .. }
            | ActionKind::PayloadPlace { .. } | ActionKind::ScriptTime { .. }
            | ActionKind::AttitudeTime { .. } | ActionKind::VtolTakeoff { .. }
            | ActionKind::VtolLand { .. } | ActionKind::RallyPoint { .. } => BtStatus::Running,
            // Condition commands
            ActionKind::ConditionDelay { seconds } => {
                if ctx.elapsed_ms >= (*seconds * 1000.0) as u32 { BtStatus::Success }
                else { BtStatus::Running }
            }
            ActionKind::ConditionYaw { .. } => BtStatus::Running,
            // Fence definitions — immediate success (informational)
            ActionKind::FenceReturnPoint { .. } | ActionKind::FencePolygonInclusion { .. }
            | ActionKind::FencePolygonExclusion { .. } | ActionKind::FenceCircleInclusion { .. }
            | ActionKind::FenceCircleExclusion { .. } => BtStatus::Success,
            // New DO actions — fire-and-forget
            ActionKind::AutotuneEnable(_) | ActionKind::GuidedLimits { .. }
            | ActionKind::InvertedFlight(_) | ActionKind::GoAround { .. }
            | ActionKind::VtolTransition { .. } | ActionKind::Winch { .. }
            | ActionKind::Sprayer(_) | ActionKind::SendScriptMessage { .. }
            | ActionKind::SetResumeRepeatDist { .. } | ActionKind::GimbalManagerPitchYaw { .. }
            | ActionKind::PauseContinue { .. } | ActionKind::ReturnPathStart
            | ActionKind::ImageStartCapture { .. } | ActionKind::ImageStopCapture
            | ActionKind::VideoStartCapture { .. } | ActionKind::VideoStopCapture
            | ActionKind::SetCameraZoom { .. } | ActionKind::SetCameraFocus { .. }
            | ActionKind::SetCameraSource { .. } => BtStatus::Success,
        }
    }

    /// Compile an ArduPilot-style waypoint list into a flat Sequence.
    pub fn from_waypoint_list(waypoints: &[(f32, f32, f32, f32)]) -> Self {
        let mut tree = Self::new();
        let mut children: heapless::Vec<NodeId, MAX_CHILDREN> = heapless::Vec::new();
        for &(n, e, d, radius) in waypoints {
            let id = tree.add_node(NodeData::Action(
                ActionKind::Waypoint { n, e, d, radius }
            ));
            let _ = children.push(id);
        }
        let root = tree.add_node(NodeData::Sequence(children));
        tree.set_root(root);
        tree
    }
}

// ─── Dual-cursor mission executor ───
// Source: AP_Mission.cpp — NAV and DO commands run in parallel.
// NAV cursor blocks on waypoint/loiter commands.
// DO cursor fires DO_ commands as it encounters them between NAV commands.

/// A mission item — one command in the mission list.
#[derive(Debug, Clone, Copy)]
pub struct MissionItem {
    pub seq: u16,
    pub command: u16,     // MAV_CMD ID
    pub frame: u8,        // coordinate frame
    pub params: [f32; 4], // param1-4
    pub x: i32,           // lat*1e7 or local x
    pub y: i32,           // lon*1e7 or local y
    pub z: f32,           // altitude
    pub autocontinue: bool,
}

/// Maximum items in a mission.
pub const MAX_MISSION_ITEMS: usize = 128;

/// DO_JUMP tracking — loop counters.
const MAX_JUMP_TAGS: usize = 16;

/// Mission state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionState {
    /// Not started.
    Idle,
    /// Running — executing commands.
    Running,
    /// Paused (can resume).
    Paused,
    /// Complete — all items executed.
    Complete,
}

/// Position where mission was paused (NED, meters).
#[derive(Debug, Clone, Copy)]
pub struct PausedPosition {
    pub n: f32,
    pub e: f32,
    pub d: f32,
}

/// Dual-cursor mission executor.
/// Source: AP_Mission.cpp update(), advance_current_nav_cmd(), advance_current_do_cmd()
pub struct MissionExecutor {
    items: heapless::Vec<MissionItem, MAX_MISSION_ITEMS>,
    pub state: MissionState,
    /// NAV cursor — index of current NAV command.
    nav_index: u16,
    /// DO cursor — index of current DO command (runs ahead of NAV).
    do_index: u16,
    /// Whether the current NAV command is complete.
    nav_complete: bool,
    /// DO_JUMP loop counters: (target_seq, remaining_count). -1 = infinite.
    jump_counters: heapless::Vec<(u16, i16), MAX_JUMP_TAGS>,
    /// Landing sequence start index (set by DO_LAND_START).
    land_start_index: Option<u16>,
    /// Current item count (for MAVLink reporting).
    pub count: u16,
    /// Position captured when mission was paused (for resume_with_rewind).
    paused_position: Option<PausedPosition>,
    /// If true, the executor is in a rewind phase — flying back to the paused waypoint.
    rewind_active: bool,
    /// The NAV waypoint action to return to after rewind.
    rewind_target: Option<ActionKind>,
}

impl MissionExecutor {
    pub fn new() -> Self {
        Self {
            items: heapless::Vec::new(),
            state: MissionState::Idle,
            nav_index: 0,
            do_index: 0,
            nav_complete: false,
            jump_counters: heapless::Vec::new(),
            land_start_index: None,
            count: 0,
            paused_position: None,
            rewind_active: false,
            rewind_target: None,
        }
    }

    /// Load mission items (replaces existing).
    pub fn load(&mut self, items: &[MissionItem]) {
        self.items.clear();
        for item in items {
            let _ = self.items.push(*item);
        }
        self.count = self.items.len() as u16;
        self.reset();
    }

    /// Add a single mission item (for upload protocol).
    pub fn set_item(&mut self, seq: u16, item: MissionItem) -> bool {
        let idx = seq as usize;
        while self.items.len() <= idx {
            let _ = self.items.push(MissionItem {
                seq: self.items.len() as u16, command: 0, frame: 0,
                params: [0.0; 4], x: 0, y: 0, z: 0.0, autocontinue: true,
            });
        }
        if idx < self.items.len() {
            self.items[idx] = item;
            self.count = self.items.len() as u16;
            true
        } else {
            false
        }
    }

    /// Get a mission item by sequence number.
    pub fn get_item(&self, seq: u16) -> Option<&MissionItem> {
        self.items.get(seq as usize)
    }

    /// Clear all items.
    pub fn clear(&mut self) {
        self.items.clear();
        self.count = 0;
        self.reset();
    }

    /// Start the mission.
    pub fn start(&mut self) {
        if self.items.is_empty() { return; }
        self.state = MissionState::Running;
        self.nav_index = 0;
        self.do_index = 0;
        self.nav_complete = false;
        self.jump_counters.clear();
        // Advance to first NAV command
        self.advance_nav();
    }

    /// Pause the mission.
    pub fn pause(&mut self) {
        if self.state == MissionState::Running {
            self.state = MissionState::Paused;
        }
    }

    /// Resume the mission (simple — continues from where it was paused).
    pub fn resume(&mut self) {
        if self.state == MissionState::Paused {
            self.state = MissionState::Running;
            self.paused_position = None;
            self.rewind_active = false;
            self.rewind_target = None;
        }
    }

    /// Resume with rewind: if vehicle has drifted from the paused waypoint,
    /// first flies back to the point where mission was paused, then continues.
    /// `current_pos`: vehicle's current NED position (n, e, d).
    pub fn resume_with_rewind(&mut self, current_pos: (f32, f32, f32)) {
        if self.state != MissionState::Paused {
            return;
        }
        // Get the NAV waypoint we were executing when paused
        let nav_action = if let Some(item) = self.items.get(self.nav_index as usize) {
            Self::item_to_action(item)
        } else {
            None
        };

        // Check if we have a NAV waypoint to return to
        if let Some(action) = nav_action {
            if let ActionKind::Waypoint { n, e, d, .. } = action {
                // Compute distance from current position to the paused waypoint
                let dn = current_pos.0 - n;
                let de = current_pos.1 - e;
                let dd = current_pos.2 - d;
                let dist_sq = dn * dn + de * de + dd * dd;

                // If drifted more than 3m, insert a rewind phase
                if dist_sq > 9.0 {
                    self.rewind_active = true;
                    self.rewind_target = Some(ActionKind::Waypoint {
                        n, e, d, radius: 2.0,
                    });
                }
            }
        }

        self.state = MissionState::Running;
    }

    /// Check if rewind phase is active (flying back to paused waypoint).
    pub fn is_rewind_active(&self) -> bool {
        self.rewind_active
    }

    /// Get the rewind target action (the waypoint to fly back to).
    pub fn rewind_nav_action(&self) -> Option<ActionKind> {
        if self.rewind_active {
            self.rewind_target
        } else {
            None
        }
    }

    /// Signal that the rewind waypoint has been reached. Continues normal mission.
    pub fn rewind_complete(&mut self) {
        self.rewind_active = false;
        self.rewind_target = None;
    }

    pub fn reset(&mut self) {
        self.state = MissionState::Idle;
        self.nav_index = 0;
        self.do_index = 0;
        self.nav_complete = false;
        self.jump_counters.clear();
        self.paused_position = None;
        self.rewind_active = false;
        self.rewind_target = None;
    }

    /// Get the current NAV command action (what the vehicle should be doing).
    /// If rewind is active, returns the rewind target instead of the normal mission item.
    pub fn current_nav_action(&self) -> Option<ActionKind> {
        if self.state != MissionState::Running { return None; }
        if self.rewind_active {
            return self.rewind_target;
        }
        let item = self.items.get(self.nav_index as usize)?;
        Self::item_to_action(item)
    }

    /// Get current NAV sequence number.
    pub fn current_nav_seq(&self) -> u16 { self.nav_index }

    /// Signal that the current NAV command is complete. Advances to next.
    pub fn nav_cmd_complete(&mut self) {
        self.nav_complete = true;
    }

    /// Tick the executor. Returns DO commands to fire this tick.
    /// Source: AP_Mission::update() — fire DO commands first, then advance NAV.
    pub fn update(&mut self) -> heapless::Vec<ActionKind, 4> {
        let mut do_cmds = heapless::Vec::new();
        if self.state != MissionState::Running { return do_cmds; }

        if self.nav_complete {
            self.nav_complete = false;

            // Fire DO commands that sit between completed NAV and next NAV
            while (self.do_index as usize) < self.items.len() {
                let item = self.items[self.do_index as usize];
                if let Some(action) = Self::item_to_action(&item) {
                    if action.is_nav() {
                        break; // Hit next NAV — stop DO processing
                    }
                    if let ActionKind::Jump { target, repeat } = action {
                        self.do_index += 1;
                        if self.handle_jump(target, repeat) {
                            return do_cmds; // Jump taken — cursors repositioned
                        }
                        continue;
                    }
                    if let ActionKind::DoLandStart = action {
                        self.land_start_index = Some(self.do_index);
                    }
                    let _ = do_cmds.push(action);
                }
                self.do_index += 1;
            }

            // Advance NAV to the next NAV command (which do_index stopped at)
            if (self.do_index as usize) < self.items.len() {
                self.nav_index = self.do_index;
                self.do_index = self.nav_index + 1;
            } else {
                self.nav_index = self.items.len() as u16;
            }
        }

        // Check if mission complete
        if self.nav_index as usize >= self.items.len() {
            self.state = MissionState::Complete;
        }

        do_cmds
    }

    /// Get the landing sequence start index (for RTL to use).
    pub fn land_start_index(&self) -> Option<u16> {
        self.land_start_index
    }

    /// Jump to a specific sequence (for DO_LAND_START RTL).
    pub fn jump_to(&mut self, seq: u16) {
        if (seq as usize) < self.items.len() {
            self.nav_index = seq;
            self.do_index = seq;
            self.nav_complete = false;
            self.advance_nav();
        }
    }

    fn advance_nav(&mut self) {
        // Skip past DO commands to find next NAV command
        while (self.nav_index as usize) < self.items.len() {
            if let Some(action) = Self::item_to_action(&self.items[self.nav_index as usize]) {
                if action.is_nav() {
                    // Set DO cursor to just after this NAV command
                    self.do_index = self.nav_index + 1;
                    return;
                }
            }
            self.nav_index += 1;
        }
    }

    fn handle_jump(&mut self, target: u16, repeat: i16) -> bool {
        // do_index was already incremented past this jump before calling
        let seq = self.do_index - 1; // the DO_JUMP's own seq
        for entry in self.jump_counters.iter_mut() {
            if entry.0 == seq {
                if entry.1 == 0 { return false; } // exhausted
                if entry.1 > 0 { entry.1 -= 1; }
                self.nav_index = target;
                self.do_index = target;
                self.advance_nav();
                return true;
            }
        }
        // First time seeing this jump
        if repeat != 0 {
            let remaining = if repeat > 0 { repeat - 1 } else { repeat }; // -1 = infinite stays -1
            let _ = self.jump_counters.push((seq, remaining));
            self.nav_index = target;
            self.do_index = target;
            self.advance_nav();
            true
        } else {
            let _ = self.jump_counters.push((seq, 0));
            false
        }
    }

    /// Convert a MissionItem to an ActionKind.
    fn item_to_action(item: &MissionItem) -> Option<ActionKind> {
        match item.command {
            // NAV commands
            16 => Some(ActionKind::Waypoint { // MAV_CMD_NAV_WAYPOINT
                n: item.x as f32 * 1e-7,
                e: item.y as f32 * 1e-7,
                d: -item.z,
                radius: item.params[0].max(2.0),
            }),
            22 => Some(ActionKind::Takeoff(item.z)), // MAV_CMD_NAV_TAKEOFF
            21 => Some(ActionKind::Land), // MAV_CMD_NAV_LAND
            20 => Some(ActionKind::RTL), // MAV_CMD_NAV_RETURN_TO_LAUNCH
            17 => Some(ActionKind::LoiterUnlim { // MAV_CMD_NAV_LOITER_UNLIM
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
            }),
            18 => Some(ActionKind::LoiterTurns { // MAV_CMD_NAV_LOITER_TURNS
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
                turns: item.params[0],
            }),
            19 => Some(ActionKind::LoiterTime { // MAV_CMD_NAV_LOITER_TIME
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
                time_s: item.params[0],
            }),
            31 => Some(ActionKind::LoiterToAlt { alt: item.z }), // MAV_CMD_NAV_LOITER_TO_ALT
            82 => Some(ActionKind::SplineWaypoint { // MAV_CMD_NAV_SPLINE_WAYPOINT
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
            }),
            93 => Some(ActionKind::Delay(item.params[0] as u32 * 1000)), // MAV_CMD_NAV_DELAY
            92 => Some(ActionKind::GuidedEnable(item.params[0] > 0.5)), // MAV_CMD_NAV_GUIDED_ENABLE
            114 => Some(ActionKind::ConditionDistance(item.params[0])), // MAV_CMD_CONDITION_DISTANCE

            // DO commands
            177 => Some(ActionKind::Jump { // MAV_CMD_DO_JUMP
                target: item.params[0] as u16,
                repeat: item.params[1] as i16,
            }),
            600 => Some(ActionKind::JumpTag(item.params[0] as u16)), // MAV_CMD_DO_JUMP_TAG
            178 => Some(ActionKind::ChangeSpeed { // MAV_CMD_DO_CHANGE_SPEED
                speed_type: item.params[0] as u8,
                speed: item.params[1],
                throttle: item.params[2],
            }),
            113 => Some(ActionKind::ChangeAlt { // MAV_CMD_DO_CHANGE_ALT (*not standard, custom)
                alt: item.z, rate: item.params[0],
            }),
            176 => Some(ActionKind::SetMode(FlightModeId::from_number(item.params[0] as u8))),
            181 => Some(ActionKind::SetRelay { // MAV_CMD_DO_SET_RELAY
                relay: item.params[0] as u8,
                state: item.params[1] > 0.5,
            }),
            182 => Some(ActionKind::RepeatRelay { // MAV_CMD_DO_REPEAT_RELAY
                relay: item.params[0] as u8,
                count: item.params[1] as u16,
                cycle_s: item.params[2],
            }),
            183 => Some(ActionKind::SetServo { // MAV_CMD_DO_SET_SERVO
                channel: item.params[0] as u8,
                pwm: item.params[1] as u16,
            }),
            184 => Some(ActionKind::RepeatServo { // MAV_CMD_DO_REPEAT_SERVO
                channel: item.params[0] as u8,
                pwm: item.params[1] as u16,
                count: item.params[2] as u16,
                cycle_s: item.params[3],
            }),
            203 => Some(ActionKind::CameraShutter), // MAV_CMD_DO_DIGICAM_CONTROL
            2500 => Some(ActionKind::CameraVideo(item.params[0] > 0.5)), // custom
            2501 => Some(ActionKind::GripperRelease), // custom
            205 => Some(ActionKind::MountControl { // MAV_CMD_DO_MOUNT_CONTROL
                pitch_deg: item.params[0], roll_deg: item.params[1], yaw_deg: item.params[2],
            }),
            201 => Some(ActionKind::SetRoi { // MAV_CMD_DO_SET_ROI_LOCATION
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
            }),
            179 => Some(ActionKind::SetHome { // MAV_CMD_DO_SET_HOME
                use_current: item.params[0] > 0.5,
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
            }),
            189 => Some(ActionKind::DoLandStart), // MAV_CMD_DO_LAND_START
            208 => Some(ActionKind::FenceEnable(item.params[0] > 0.5)), // MAV_CMD_DO_FENCE_ENABLE
            216 => Some(ActionKind::Parachute(item.params[0] as u8)), // MAV_CMD_DO_PARACHUTE
            223 => Some(ActionKind::EngineControl { // MAV_CMD_DO_ENGINE_CONTROL
                start: item.params[0] > 0.5, cold: item.params[1] > 0.5,
            }),
            // 194 = MAV_CMD_NAV_TAKEOFF_LOCAL (rarely used, not SetReverse). Unmapped.
            // 194 => None,
            115 => Some(ActionKind::ConditionYaw { // MAV_CMD_CONDITION_YAW
                angle_deg: item.params[0],
                rate_dps: item.params[1],
                direction: if item.params[2] < 0.0 { -1 } else { item.params[2] as i8 },
                relative: item.params[3] > 0.5,
            }),
            218 => Some(ActionKind::AuxFunction { // MAV_CMD_DO_AUX_FUNCTION
                function: item.params[0] as u16,
                switch_pos: item.params[1] as u8,
            }),

            // GAP 32: New MAV_CMD IDs
            // NAV commands
            38 => Some(ActionKind::ArcWaypoint { // MAV_CMD_NAV_ARC_WAYPOINT (proposed)
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
                radius: item.params[0],
            }),
            83 => Some(ActionKind::AltitudeWait { // MAV_CMD_NAV_ALTITUDE_WAIT
                alt: item.params[0],
                descent_rate: item.params[1],
                wiggle_time: item.params[2],
            }),
            30 => Some(ActionKind::ContinueAndChangeAlt { // MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
                alt: item.z,
            }),
            213 => Some(ActionKind::NavSetYawSpeed { // MAV_CMD_NAV_SET_YAW_SPEED
                yaw_deg: item.params[0],
                speed: item.params[1],
            }),
            94 => Some(ActionKind::PayloadPlace { // MAV_CMD_NAV_PAYLOAD_PLACE
                max_descent: item.params[0],
            }),
            42 => Some(ActionKind::ScriptTime { // MAV_CMD_NAV_SCRIPT_TIME
                cmd: item.params[0] as u16,
                timeout_s: item.params[1],
                arg1: item.params[2],
                arg2: item.params[3],
            }),
            35 => Some(ActionKind::VtolTakeoff { alt: item.z }), // MAV_CMD_NAV_VTOL_TAKEOFF
            36 => Some(ActionKind::VtolLand { // MAV_CMD_NAV_VTOL_LAND
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7,
                approach_alt: item.params[0],
            }),
            5100 => Some(ActionKind::RallyPoint { // MAV_CMD_NAV_RALLY_POINT
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
            }),
            // Condition commands
            112 => Some(ActionKind::ConditionDelay { // MAV_CMD_CONDITION_DELAY
                seconds: item.params[0],
            }),
            // Fence commands
            5000 => Some(ActionKind::FenceReturnPoint { // MAV_CMD_NAV_FENCE_RETURN_POINT
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7, d: -item.z,
            }),
            5001 => Some(ActionKind::FencePolygonInclusion { // MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7,
                count: item.params[0] as u8,
            }),
            5002 => Some(ActionKind::FencePolygonExclusion { // MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7,
                count: item.params[0] as u8,
            }),
            5003 => Some(ActionKind::FenceCircleInclusion { // MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7,
                radius: item.params[0],
            }),
            5004 => Some(ActionKind::FenceCircleExclusion { // MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
                n: item.x as f32 * 1e-7, e: item.y as f32 * 1e-7,
                radius: item.params[0],
            }),
            // DO commands
            211 => Some(ActionKind::AutotuneEnable(item.params[0] > 0.5)), // MAV_CMD_DO_AUTOTUNE_ENABLE
            222 => Some(ActionKind::GuidedLimits { // MAV_CMD_DO_GUIDED_LIMITS
                timeout_s: item.params[0], alt_min: item.params[1],
                alt_max: item.params[2], horiz_max: item.params[3],
            }),
            210 => Some(ActionKind::InvertedFlight(item.params[0] > 0.5)), // MAV_CMD_DO_INVERTED_FLIGHT
            191 => Some(ActionKind::GoAround { alt: item.z }), // MAV_CMD_DO_GO_AROUND
            3000 => Some(ActionKind::VtolTransition { // MAV_CMD_DO_VTOL_TRANSITION
                target_state: item.params[0] as u8,
            }),
            42600 => Some(ActionKind::Winch { // MAV_CMD_DO_WINCH
                action: item.params[0] as u8, length: item.params[1], rate: item.params[2],
            }),
            // Note: MAV_CMD_DO_SPRAYER (216) shares cmd ID with Parachute.
            // In practice, disambiguation is done via param interpretation.
            // Parachute handler (above) takes precedence in the match arm.
            217 => Some(ActionKind::SendScriptMessage { // MAV_CMD_DO_SEND_SCRIPT_MESSAGE
                id: item.params[0] as u16, p1: item.params[1], p2: item.params[2], p3: item.params[3],
            }),
            215 => Some(ActionKind::SetResumeRepeatDist { dist: item.params[0] }), // MAV_CMD_DO_SET_RESUME_REPEAT_DIST
            1000 => Some(ActionKind::GimbalManagerPitchYaw { // MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
                pitch_deg: item.params[0], yaw_deg: item.params[1],
                pitch_rate: item.params[2], yaw_rate: item.params[3],
            }),
            193 => Some(ActionKind::PauseContinue { pause: item.params[0] < 0.5 }), // MAV_CMD_DO_PAUSE_CONTINUE
            188 => Some(ActionKind::ReturnPathStart), // MAV_CMD_DO_RETURN_PATH_START
            // Camera capture
            2000 => Some(ActionKind::ImageStartCapture {
                interval_s: item.params[0], count: item.params[1] as u32,
            }),
            2001 => Some(ActionKind::ImageStopCapture),
            // Note: 2500 is already used for CameraVideo above.
            2502 => Some(ActionKind::VideoStartCapture { freq_hz: item.params[0] }),
            2503 => Some(ActionKind::VideoStopCapture),
            531 => Some(ActionKind::SetCameraZoom {
                zoom_type: item.params[0] as u8, zoom_value: item.params[1],
            }),
            532 => Some(ActionKind::SetCameraFocus {
                focus_type: item.params[0] as u8, focus_value: item.params[1],
            }),
            534 => Some(ActionKind::SetCameraSource {
                device_id: item.params[0] as u8, primary: item.params[1] as u8, secondary: item.params[2] as u8,
            }),
            // MAV_CMD_NAV_ATTITUDE_TIME (42703) — hold attitude for a duration
            42703 => Some(ActionKind::AttitudeTime {
                time_s: item.params[0],
                roll_deg: item.params[1],
                pitch_deg: item.params[2],
                yaw_deg: item.params[3],
            }),
            // MAV_CMD_DO_SPRAYER: ArduPilot uses cmd 216 for both Parachute and Sprayer.
            // Parachute (above) takes precedence at 216.  Sprayer is a custom extension
            // at 40001 to avoid ambiguity.
            40001 => Some(ActionKind::Sprayer(item.params[0] > 0.5)),

            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_ctx() -> BtContext {
        BtContext {
            battery_pct: 80.0, altitude: 10.0, distance_to_wp: 5.0,
            speed: 2.0, wind_speed: 1.0, gps_available: true, ekf_healthy: true,
            geofence_ok: true, comms_active: true, armed: true, elapsed_ms: 0,
        }
    }

    #[test]
    fn test_sequence() {
        let mut tree = BehaviorTree::new();
        let a = tree.add_node(NodeData::Condition(ConditionKind::Armed));
        let b = tree.add_node(NodeData::Condition(ConditionKind::GpsAvailable));
        let mut children = heapless::Vec::new();
        let _ = children.push(a);
        let _ = children.push(b);
        let root = tree.add_node(NodeData::Sequence(children));
        tree.set_root(root);

        assert_eq!(tree.tick(&default_ctx()), BtStatus::Success);
    }

    #[test]
    fn test_sequence_fails() {
        let mut tree = BehaviorTree::new();
        let a = tree.add_node(NodeData::Condition(ConditionKind::Armed));
        let b = tree.add_node(NodeData::Condition(ConditionKind::AltitudeAbove(100.0))); // will fail
        let mut children = heapless::Vec::new();
        let _ = children.push(a);
        let _ = children.push(b);
        let root = tree.add_node(NodeData::Sequence(children));
        tree.set_root(root);

        assert_eq!(tree.tick(&default_ctx()), BtStatus::Failure);
    }

    #[test]
    fn test_fallback() {
        let mut tree = BehaviorTree::new();
        let a = tree.add_node(NodeData::Condition(ConditionKind::AltitudeAbove(100.0))); // fail
        let b = tree.add_node(NodeData::Condition(ConditionKind::Armed)); // succeed
        let mut children = heapless::Vec::new();
        let _ = children.push(a);
        let _ = children.push(b);
        let root = tree.add_node(NodeData::Fallback(children));
        tree.set_root(root);

        assert_eq!(tree.tick(&default_ctx()), BtStatus::Success);
    }

    #[test]
    fn test_waypoint_action() {
        let mut tree = BehaviorTree::new();
        let wp = tree.add_node(NodeData::Action(ActionKind::Waypoint {
            n: 10.0, e: 0.0, d: -10.0, radius: 2.0,
        }));
        tree.set_root(wp);

        // Far from WP: running
        let mut ctx = default_ctx();
        ctx.distance_to_wp = 15.0;
        assert_eq!(tree.tick(&ctx), BtStatus::Running);

        // Close to WP: success
        ctx.distance_to_wp = 1.0;
        assert_eq!(tree.tick(&ctx), BtStatus::Success);
    }

    #[test]
    fn test_conditional_mission() {
        let mut tree = BehaviorTree::new();
        let bat_ok = tree.add_node(NodeData::Condition(ConditionKind::BatteryAbove(30.0)));
        let wp = tree.add_node(NodeData::Action(ActionKind::Waypoint {
            n: 100.0, e: 0.0, d: -50.0, radius: 5.0,
        }));
        let rtl = tree.add_node(NodeData::Action(ActionKind::RTL));

        let mut mission_children = heapless::Vec::new();
        let _ = mission_children.push(bat_ok);
        let _ = mission_children.push(wp);
        let mission = tree.add_node(NodeData::Sequence(mission_children));

        let mut fallback_children = heapless::Vec::new();
        let _ = fallback_children.push(mission);
        let _ = fallback_children.push(rtl);
        let root = tree.add_node(NodeData::Fallback(fallback_children));
        tree.set_root(root);

        let mut ctx = default_ctx();
        ctx.distance_to_wp = 50.0;
        assert_eq!(tree.tick(&ctx), BtStatus::Running);

        ctx.battery_pct = 20.0;
        assert_eq!(tree.tick(&ctx), BtStatus::Running);
    }

    // ─── Dual-cursor executor tests ───

    fn make_nav_wp(seq: u16, lat_e7: i32, lon_e7: i32, alt: f32) -> MissionItem {
        MissionItem {
            seq, command: 16, frame: 3, // MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT
            params: [5.0, 0.0, 0.0, 0.0], // acceptance radius 5m
            x: lat_e7, y: lon_e7, z: alt, autocontinue: true,
        }
    }

    fn make_do_speed(seq: u16, speed: f32) -> MissionItem {
        MissionItem {
            seq, command: 178, frame: 0, // MAV_CMD_DO_CHANGE_SPEED
            params: [0.0, speed, -1.0, 0.0],
            x: 0, y: 0, z: 0.0, autocontinue: true,
        }
    }

    fn make_do_jump(seq: u16, target: u16, repeat: i16) -> MissionItem {
        MissionItem {
            seq, command: 177, frame: 0, // MAV_CMD_DO_JUMP
            params: [target as f32, repeat as f32, 0.0, 0.0],
            x: 0, y: 0, z: 0.0, autocontinue: true,
        }
    }

    #[test]
    fn test_executor_basic_mission() {
        let mut exec = MissionExecutor::new();
        exec.load(&[
            make_nav_wp(0, 350000000, -1200000000, 50.0),
            make_nav_wp(1, 350001000, -1200001000, 50.0),
        ]);
        exec.start();
        assert_eq!(exec.state, MissionState::Running);
        assert_eq!(exec.current_nav_seq(), 0);

        // Complete first WP
        exec.nav_cmd_complete();
        exec.update();
        assert_eq!(exec.current_nav_seq(), 1);

        // Complete second WP
        exec.nav_cmd_complete();
        exec.update();
        assert_eq!(exec.state, MissionState::Complete);
    }

    #[test]
    fn test_executor_dual_cursor() {
        // WP0 → DO_CHANGE_SPEED → WP1
        let mut exec = MissionExecutor::new();
        exec.load(&[
            make_nav_wp(0, 350000000, -1200000000, 50.0),
            make_do_speed(1, 3.0),
            make_nav_wp(2, 350001000, -1200001000, 50.0),
        ]);
        exec.start();
        assert_eq!(exec.current_nav_seq(), 0);

        // Complete WP0 → DO_CHANGE_SPEED should fire, then advance to WP1
        exec.nav_cmd_complete();
        let do_cmds = exec.update();
        assert_eq!(do_cmds.len(), 1);
        match do_cmds[0] {
            ActionKind::ChangeSpeed { speed, .. } => assert!((speed - 3.0).abs() < 0.01),
            _ => panic!("Expected ChangeSpeed"),
        }
        assert_eq!(exec.current_nav_seq(), 2);
    }

    #[test]
    fn test_executor_do_jump() {
        // WP0 → WP1 → DO_JUMP(target=0, repeat=2)
        // Should loop: WP0→WP1→jump→WP0→WP1→jump→WP0→WP1→jump exhausted→complete
        let mut exec = MissionExecutor::new();
        exec.load(&[
            make_nav_wp(0, 350000000, -1200000000, 50.0),
            make_nav_wp(1, 350001000, -1200001000, 50.0),
            make_do_jump(2, 0, 2), // jump to seq 0, repeat 2 times
        ]);
        exec.start();

        // First pass: WP0 → WP1 → jump fires (repeat 2→1)
        exec.nav_cmd_complete(); exec.update(); // advance to WP1
        exec.nav_cmd_complete();
        let _ = exec.update(); // should jump back to WP0
        assert_eq!(exec.current_nav_seq(), 0, "Should jump back to WP0");

        // Second pass: WP0 → WP1 → jump fires (repeat 1→0)
        exec.nav_cmd_complete(); exec.update();
        exec.nav_cmd_complete();
        let _ = exec.update();
        assert_eq!(exec.current_nav_seq(), 0, "Should jump back again");

        // Third pass: WP0 → WP1 → jump exhausted → complete
        exec.nav_cmd_complete(); exec.update();
        exec.nav_cmd_complete();
        let _ = exec.update();
        assert_eq!(exec.state, MissionState::Complete);
    }

    #[test]
    fn test_executor_pause_resume() {
        let mut exec = MissionExecutor::new();
        exec.load(&[make_nav_wp(0, 0, 0, 50.0), make_nav_wp(1, 0, 0, 50.0)]);
        exec.start();
        exec.pause();
        assert_eq!(exec.state, MissionState::Paused);
        assert!(exec.current_nav_action().is_none()); // no action when paused
        exec.resume();
        assert_eq!(exec.state, MissionState::Running);
        assert!(exec.current_nav_action().is_some());
    }

    #[test]
    fn test_executor_do_land_start() {
        let mut exec = MissionExecutor::new();
        exec.load(&[
            make_nav_wp(0, 0, 0, 50.0),
            MissionItem { // DO_LAND_START
                seq: 1, command: 189, frame: 0,
                params: [0.0; 4], x: 0, y: 0, z: 0.0, autocontinue: true,
            },
            make_nav_wp(2, 0, 0, 10.0), // landing approach WP
            MissionItem { // LAND
                seq: 3, command: 21, frame: 0,
                params: [0.0; 4], x: 0, y: 0, z: 0.0, autocontinue: true,
            },
        ]);
        exec.start();
        exec.nav_cmd_complete();
        exec.update();
        assert!(exec.land_start_index().is_some());
    }

    #[test]
    fn test_action_kind_nav_vs_do() {
        assert!(ActionKind::Waypoint { n: 0.0, e: 0.0, d: 0.0, radius: 2.0 }.is_nav());
        assert!(ActionKind::Takeoff(10.0).is_nav());
        assert!(ActionKind::Land.is_nav());
        assert!(ActionKind::RTL.is_nav());
        assert!(ActionKind::SetSpeed(5.0).is_do());
        assert!(ActionKind::CameraShutter.is_do());
        assert!(ActionKind::Jump { target: 0, repeat: 1 }.is_do());
    }

    #[test]
    fn test_mission_item_upload() {
        let mut exec = MissionExecutor::new();
        // Simulate MAVLink upload: set count, then items one by one
        exec.set_item(0, make_nav_wp(0, 350000000, -1200000000, 50.0));
        exec.set_item(1, make_do_speed(1, 5.0));
        exec.set_item(2, make_nav_wp(2, 350001000, -1200001000, 50.0));
        assert_eq!(exec.count, 3);
        assert!(exec.get_item(0).is_some());
        assert_eq!(exec.get_item(1).unwrap().command, 178);
    }

    #[test]
    fn test_from_waypoint_list() {
        let wps = [(10.0, 0.0, -10.0, 2.0), (20.0, 10.0, -10.0, 2.0)];
        let mut tree = BehaviorTree::from_waypoint_list(&wps);

        // Far from all WPs: first WP running
        let mut ctx = default_ctx();
        ctx.distance_to_wp = 15.0;
        assert_eq!(tree.tick(&ctx), BtStatus::Running);

        // Close to WP: first succeeds, sequence continues to second (also running since same dist context)
        ctx.distance_to_wp = 1.0;
        // Both WPs use the same distance_to_wp context, so both succeed → whole sequence succeeds
        assert_eq!(tree.tick(&ctx), BtStatus::Success);
    }

    #[test]
    fn test_inverter() {
        let mut tree = BehaviorTree::new();
        let cond = tree.add_node(NodeData::Condition(ConditionKind::AltitudeAbove(100.0))); // fails
        let inv = tree.add_node(NodeData::Inverter(cond));
        tree.set_root(inv);

        assert_eq!(tree.tick(&default_ctx()), BtStatus::Success); // inverted failure = success
    }

    // ── resume_with_rewind tests ──

    #[test]
    fn test_resume_with_rewind_when_drifted() {
        let mut exec = MissionExecutor::new();
        // WP0 at (35.0, -120.0, -50.0) in NED
        exec.load(&[
            make_nav_wp(0, 350000000, -1200000000, 50.0),
            make_nav_wp(1, 350001000, -1200001000, 50.0),
        ]);
        exec.start();

        // Pause while on WP0
        exec.pause();
        assert_eq!(exec.state, MissionState::Paused);

        // Get the WP0 coordinates (item_to_action converts lat_e7*1e-7)
        let wp0_n = 350000000_f32 * 1e-7;
        let wp0_e = -1200000000_f32 * 1e-7;
        let wp0_d = -50.0;

        // Resume from a position far from WP0 (drifted > 3m)
        let current = (wp0_n + 10.0, wp0_e + 10.0, wp0_d);
        exec.resume_with_rewind(current);

        assert_eq!(exec.state, MissionState::Running);
        assert!(exec.is_rewind_active());

        // current_nav_action should return the rewind waypoint, not the normal WP0
        let action = exec.current_nav_action();
        assert!(action.is_some());
        if let Some(ActionKind::Waypoint { n, e, d, .. }) = action {
            // Should be the WP0 position
            assert!((n - wp0_n).abs() < 0.01);
            assert!((e - wp0_e).abs() < 0.01);
            assert!((d - wp0_d).abs() < 0.01);
        } else {
            panic!("Expected Waypoint rewind target");
        }

        // Signal rewind complete
        exec.rewind_complete();
        assert!(!exec.is_rewind_active());

        // Now current_nav_action should return the normal mission WP
        let action = exec.current_nav_action();
        assert!(action.is_some());
    }

    #[test]
    fn test_resume_with_rewind_no_drift() {
        let mut exec = MissionExecutor::new();
        exec.load(&[
            make_nav_wp(0, 350000000, -1200000000, 50.0),
            make_nav_wp(1, 350001000, -1200001000, 50.0),
        ]);
        exec.start();
        exec.pause();

        let wp0_n = 350000000_f32 * 1e-7;
        let wp0_e = -1200000000_f32 * 1e-7;
        let wp0_d = -50.0;

        // Resume from very close to WP0 (< 3m drift)
        let current = (wp0_n + 0.5, wp0_e + 0.5, wp0_d);
        exec.resume_with_rewind(current);

        assert_eq!(exec.state, MissionState::Running);
        // Should NOT activate rewind since drift < 3m
        assert!(!exec.is_rewind_active());
    }

    #[test]
    fn test_resume_with_rewind_only_when_paused() {
        let mut exec = MissionExecutor::new();
        exec.load(&[make_nav_wp(0, 0, 0, 50.0)]);
        exec.start();

        // Try to resume_with_rewind when not paused — should do nothing
        exec.resume_with_rewind((100.0, 100.0, -50.0));
        assert_eq!(exec.state, MissionState::Running);
        assert!(!exec.is_rewind_active());
    }

    #[test]
    fn test_simple_resume_clears_rewind_state() {
        let mut exec = MissionExecutor::new();
        exec.load(&[make_nav_wp(0, 0, 0, 50.0)]);
        exec.start();
        exec.pause();
        exec.resume(); // simple resume
        assert!(!exec.is_rewind_active());
    }

    // ─── GAP 32: MAV_CMD ID mapping tests ───

    #[test]
    fn test_condition_distance_at_cmd_114() {
        let item = MissionItem {
            seq: 0, command: 114, frame: 0,
            params: [100.0, 0.0, 0.0, 0.0],
            x: 0, y: 0, z: 0.0, autocontinue: true,
        };
        let action = MissionExecutor::item_to_action(&item);
        assert!(action.is_some());
        match action.unwrap() {
            ActionKind::ConditionDistance(d) => assert!((d - 100.0).abs() < 0.01),
            other => panic!("Expected ConditionDistance, got {:?}", other),
        }
    }

    #[test]
    fn test_condition_yaw_at_cmd_115() {
        let item = MissionItem {
            seq: 0, command: 115, frame: 0,
            params: [45.0, 10.0, 1.0, 0.0],
            x: 0, y: 0, z: 0.0, autocontinue: true,
        };
        let action = MissionExecutor::item_to_action(&item);
        assert!(action.is_some());
        match action.unwrap() {
            ActionKind::ConditionYaw { angle_deg, rate_dps, direction, relative } => {
                assert!((angle_deg - 45.0).abs() < 0.01);
                assert!((rate_dps - 10.0).abs() < 0.01);
                assert_eq!(direction, 1);
                assert!(!relative);
            }
            other => panic!("Expected ConditionYaw, got {:?}", other),
        }
    }

    #[test]
    fn test_attitude_time_at_cmd_42703() {
        let item = MissionItem {
            seq: 0, command: 42703, frame: 0,
            params: [5.0, 10.0, -5.0, 90.0],
            x: 0, y: 0, z: 0.0, autocontinue: true,
        };
        let action = MissionExecutor::item_to_action(&item);
        assert!(action.is_some());
        match action.unwrap() {
            ActionKind::AttitudeTime { time_s, roll_deg, pitch_deg, yaw_deg } => {
                assert!((time_s - 5.0).abs() < 0.01);
                assert!((roll_deg - 10.0).abs() < 0.01);
                assert!((pitch_deg - (-5.0)).abs() < 0.01);
                assert!((yaw_deg - 90.0).abs() < 0.01);
            }
            other => panic!("Expected AttitudeTime, got {:?}", other),
        }
    }

    #[test]
    fn test_sprayer_at_cmd_40001() {
        let item = MissionItem {
            seq: 0, command: 40001, frame: 0,
            params: [1.0, 0.0, 0.0, 0.0],
            x: 0, y: 0, z: 0.0, autocontinue: true,
        };
        let action = MissionExecutor::item_to_action(&item);
        assert!(action.is_some());
        match action.unwrap() {
            ActionKind::Sprayer(on) => assert!(on),
            other => panic!("Expected Sprayer, got {:?}", other),
        }
    }

    #[test]
    fn test_condition_yaw_is_nav() {
        // ConditionYaw should be a NAV command (blocks nav cursor)
        let cy = ActionKind::ConditionYaw {
            angle_deg: 90.0, rate_dps: 10.0, direction: 1, relative: false,
        };
        assert!(cy.is_nav());
    }

    #[test]
    fn test_sprayer_is_do() {
        let sp = ActionKind::Sprayer(true);
        assert!(sp.is_do());
    }
}

