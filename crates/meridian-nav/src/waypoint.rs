//! Waypoint navigation: sequencing, acceptance radius, cross-track error.
//!
//! Works for all vehicle types — multirotors, fixed-wing, rovers, boats.

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// Altitude reference frame for waypoints.
/// Source: MAV_FRAME in MAVLink, used by AC_WPNav::set_wp_destination_loc()
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AltFrame {
    /// Altitude relative to home (default).
    Relative,
    /// Altitude above mean sea level.
    AbsoluteMsl,
    /// Altitude above terrain (requires terrain data from meridian-terrain).
    AboveTerrain,
    /// Altitude from rangefinder.
    Rangefinder,
}

/// A single waypoint.
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    /// Position NED relative to home (meters).
    pub position: Vec3<NED>,
    /// Speed at this waypoint (m/s). 0 = default.
    pub speed: f32,
    /// Acceptance radius (meters). Vehicle advances to next WP when within this distance.
    pub accept_radius: f32,
    /// Whether to loiter at this waypoint (hold position).
    pub loiter: bool,
    /// Loiter time (seconds). 0 = indefinite.
    pub loiter_time: f32,
    /// Altitude reference frame — connects to meridian-terrain for ABOVE_TERRAIN.
    /// Source: Location::AltFrame in ArduPilot.
    pub alt_frame: AltFrame,
}

impl Waypoint {
    pub fn new(n: f32, e: f32, d: f32) -> Self {
        Self {
            position: Vec3::new(n, e, d),
            speed: 0.0,
            accept_radius: 2.0,
            loiter: false,
            loiter_time: 0.0,
            alt_frame: AltFrame::Relative,
        }
    }

    pub fn with_radius(mut self, r: f32) -> Self {
        self.accept_radius = r;
        self
    }

    pub fn with_loiter(mut self, time: f32) -> Self {
        self.loiter = true;
        self.loiter_time = time;
        self
    }
}

/// Status of waypoint navigation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaypointStatus {
    /// Navigating toward current waypoint.
    InProgress,
    /// Reached current waypoint, advancing to next.
    Reached,
    /// Loitering at current waypoint.
    Loitering,
    /// All waypoints completed.
    Complete,
}

/// Waypoint navigator: manages a sequence of waypoints.
pub struct WaypointNav {
    waypoints: heapless::Vec<Waypoint, 64>,
    current_index: usize,
    status: WaypointStatus,
    loiter_start_time: f32,
    /// Distance from previous WP to current WP (for cross-track error).
    leg_distance: f32,
    /// Unit vector from prev WP to current WP.
    leg_direction: Vec3<NED>,
}

impl WaypointNav {
    pub fn new() -> Self {
        Self {
            waypoints: heapless::Vec::new(),
            current_index: 0,
            status: WaypointStatus::Complete,
            loiter_start_time: 0.0,
            leg_distance: 0.0,
            leg_direction: Vec3::new(1.0, 0.0, 0.0),
        }
    }

    /// Load a mission (list of waypoints).
    pub fn load(&mut self, wps: &[Waypoint]) {
        self.waypoints.clear();
        for wp in wps {
            let _ = self.waypoints.push(*wp);
        }
        self.current_index = 0;
        self.status = if self.waypoints.is_empty() {
            WaypointStatus::Complete
        } else {
            WaypointStatus::InProgress
        };
        self.update_leg(Vec3::zero());
    }

    /// Get the current target waypoint.
    pub fn current_wp(&self) -> Option<&Waypoint> {
        self.waypoints.get(self.current_index)
    }

    /// Get the current target position.
    pub fn target_position(&self) -> Vec3<NED> {
        self.current_wp().map(|w| w.position).unwrap_or(Vec3::zero())
    }

    pub fn status(&self) -> WaypointStatus {
        self.status
    }

    pub fn current_index(&self) -> usize {
        self.current_index
    }

    pub fn total_waypoints(&self) -> usize {
        self.waypoints.len()
    }

    /// Update navigation state based on current position and time.
    ///
    /// Returns the target position for the controller.
    pub fn update(&mut self, current_pos: &Vec3<NED>, time: f32) -> Vec3<NED> {
        if self.status == WaypointStatus::Complete {
            return self.target_position();
        }

        let wp = match self.current_wp() {
            Some(wp) => *wp,
            None => {
                self.status = WaypointStatus::Complete;
                return Vec3::zero();
            }
        };

        let to_wp = wp.position - *current_pos;
        let dist = to_wp.length();

        match self.status {
            WaypointStatus::InProgress => {
                if dist < wp.accept_radius {
                    if wp.loiter {
                        self.status = WaypointStatus::Loitering;
                        self.loiter_start_time = time;
                    } else {
                        self.status = WaypointStatus::Reached;
                    }
                }
            }
            WaypointStatus::Loitering => {
                if wp.loiter_time > 0.0 && (time - self.loiter_start_time) >= wp.loiter_time {
                    self.status = WaypointStatus::Reached;
                }
            }
            WaypointStatus::Reached => {
                // Advance to next waypoint
                let prev_pos = wp.position;
                self.current_index += 1;
                if self.current_index >= self.waypoints.len() {
                    self.status = WaypointStatus::Complete;
                } else {
                    self.status = WaypointStatus::InProgress;
                    self.update_leg(prev_pos);
                }
            }
            WaypointStatus::Complete => {}
        }

        self.target_position()
    }

    /// Compute cross-track error: perpendicular distance from the current leg.
    pub fn cross_track_error(&self, current_pos: &Vec3<NED>) -> f32 {
        if self.current_index == 0 || self.leg_distance < 0.1 {
            return 0.0;
        }
        let prev_wp = self.waypoints[self.current_index - 1].position;
        let to_pos = *current_pos - prev_wp;
        let along = to_pos.dot(&self.leg_direction);
        let along_vec = self.leg_direction * along;
        let cross = to_pos - along_vec;
        cross.length()
    }

    fn update_leg(&mut self, prev_pos: Vec3<NED>) {
        if let Some(wp) = self.current_wp() {
            let leg = wp.position - prev_pos;
            self.leg_distance = leg.length();
            if self.leg_distance > 0.1 {
                self.leg_direction = leg.normalized();
            }
        }
    }
}
