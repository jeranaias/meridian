//! Camera backend trait and implementations.
//!
//! Source: ArduPilot `AP_Camera_Backend.h`, `AP_Camera_Servo.cpp`, `AP_Camera_Relay.cpp`,
//! `AP_Camera_MAVLinkCamV2.cpp`, `AP_Camera_Mount.cpp`, `AP_Camera_RunCam.cpp`

/// Focus control mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FocusMode {
    Rate(i8),
    Percent(u8),
    Auto,
}

/// Zoom control mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZoomMode {
    Rate(i8),
    Percent(u8),
}

/// Tracking mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrackingType {
    None,
    Point { x: f32, y: f32 },
    Rectangle { top_left_x: f32, top_left_y: f32, bot_right_x: f32, bot_right_y: f32 },
}

/// Camera backend trait.
pub trait CameraBackend {
    /// Trigger the shutter (take a photo).
    fn trigger_shutter(&mut self);

    /// Start video recording.
    fn start_video(&mut self);

    /// Stop video recording.
    fn stop_video(&mut self);

    /// Whether the backend is currently recording video.
    fn is_recording(&self) -> bool;

    /// Update — called each loop to handle timed pulse completion, etc.
    fn update(&mut self, now_ms: u32);

    /// Set focus mode (stub — only MAVLink CamV2 supports this).
    fn set_focus(&mut self, _mode: FocusMode) {}

    /// Set zoom mode (stub — only MAVLink CamV2 supports this).
    fn set_zoom(&mut self, _mode: ZoomMode) {}

    /// Set tracking mode (stub — only MAVLink CamV2 supports this).
    fn set_tracking(&mut self, _tracking: TrackingType) {}

    /// Set lens selection (stub).
    fn set_lens(&mut self, _lens_id: u8) {}
}

/// Servo-based camera trigger — fires a servo pulse to trigger the shutter.
///
/// Typical setup: servo moves from `off_pwm` to `on_pwm` for `pulse_ms` duration.
pub struct ServoCameraBackend {
    /// Output channel index (0-based).
    pub channel: u8,
    /// PWM value when shutter is idle.
    pub off_pwm: u16,
    /// PWM value when shutter is triggered.
    pub on_pwm: u16,
    /// Duration of the trigger pulse in milliseconds.
    pub pulse_ms: u32,
    /// Current PWM output — caller reads this to apply via RcOutput.
    pub current_pwm: u16,
    /// Timestamp when the pulse started (0 = idle).
    pulse_start_ms: u32,
    recording: bool,
}

impl ServoCameraBackend {
    pub fn new(channel: u8, off_pwm: u16, on_pwm: u16, pulse_ms: u32) -> Self {
        Self {
            channel,
            off_pwm,
            on_pwm,
            pulse_ms,
            current_pwm: off_pwm,
            pulse_start_ms: 0,
            recording: false,
        }
    }

    /// Whether a trigger pulse is currently active.
    pub fn is_pulsing(&self) -> bool {
        self.pulse_start_ms > 0
    }
}

impl CameraBackend for ServoCameraBackend {
    fn trigger_shutter(&mut self) {
        self.current_pwm = self.on_pwm;
        if self.pulse_start_ms == 0 {
            self.pulse_start_ms = u32::MAX; // sentinel: set real timestamp on next update()
        }
    }

    fn start_video(&mut self) {
        self.recording = true;
    }

    fn stop_video(&mut self) {
        self.recording = false;
    }

    fn is_recording(&self) -> bool {
        self.recording
    }

    fn update(&mut self, now_ms: u32) {
        if self.pulse_start_ms == u32::MAX {
            self.pulse_start_ms = now_ms;
            self.current_pwm = self.on_pwm;
        } else if self.pulse_start_ms > 0 {
            let elapsed = now_ms.wrapping_sub(self.pulse_start_ms);
            if elapsed >= self.pulse_ms {
                self.current_pwm = self.off_pwm;
                self.pulse_start_ms = 0;
            }
        }
    }
}

/// Relay-based camera trigger — toggles a GPIO pin.
pub struct RelayCameraBackend {
    /// GPIO pin number.
    pub pin: u16,
    /// Active-high (true) or active-low (false).
    pub active_high: bool,
    /// Duration of the trigger pulse in milliseconds.
    pub pulse_ms: u32,
    /// Current output state — caller reads this to apply via GpioPin.
    pub current_state: bool,
    /// Timestamp when the pulse started (0 = idle).
    pulse_start_ms: u32,
    recording: bool,
}

impl RelayCameraBackend {
    pub fn new(pin: u16, active_high: bool, pulse_ms: u32) -> Self {
        Self {
            pin,
            active_high,
            pulse_ms,
            current_state: !active_high, // idle = inactive
            pulse_start_ms: 0,
            recording: false,
        }
    }
}

impl CameraBackend for RelayCameraBackend {
    fn trigger_shutter(&mut self) {
        self.current_state = self.active_high;
        if self.pulse_start_ms == 0 {
            self.pulse_start_ms = u32::MAX;
        }
    }

    fn start_video(&mut self) {
        self.recording = true;
    }

    fn stop_video(&mut self) {
        self.recording = false;
    }

    fn is_recording(&self) -> bool {
        self.recording
    }

    fn update(&mut self, now_ms: u32) {
        if self.pulse_start_ms == u32::MAX {
            self.pulse_start_ms = now_ms;
            self.current_state = self.active_high;
        } else if self.pulse_start_ms > 0 {
            let elapsed = now_ms.wrapping_sub(self.pulse_start_ms);
            if elapsed >= self.pulse_ms {
                self.current_state = !self.active_high;
                self.pulse_start_ms = 0;
            }
        }
    }
}

/// Mount-delegate camera backend stub.
/// Delegates trigger to the mount/gimbal subsystem camera trigger line.
pub struct MountCameraBackend {
    /// Mount instance index.
    pub mount_instance: u8,
    /// Whether a shutter trigger has been requested (caller reads and forwards to mount).
    pub trigger_pending: bool,
    recording: bool,
}

impl MountCameraBackend {
    pub fn new(mount_instance: u8) -> Self {
        Self {
            mount_instance,
            trigger_pending: false,
            recording: false,
        }
    }
}

impl CameraBackend for MountCameraBackend {
    fn trigger_shutter(&mut self) {
        self.trigger_pending = true;
    }

    fn start_video(&mut self) {
        self.recording = true;
    }

    fn stop_video(&mut self) {
        self.recording = false;
    }

    fn is_recording(&self) -> bool {
        self.recording
    }

    fn update(&mut self, _now_ms: u32) {
        // Mount backend clears trigger_pending externally after forwarding
    }
}

/// MAVLink Camera Protocol v2 backend stub.
///
/// Handles CAMERA_INFORMATION, CAMERA_SETTINGS, CAMERA_CAPTURE_STATUS,
/// CAMERA_FOV_STATUS messages and proxies zoom/focus/tracking commands.
pub struct MavlinkCamV2Backend {
    pub target_sysid: u8,
    pub target_compid: u8,
    /// Pending camera command (caller reads and sends via MAVLink).
    pub pending_cmd: Option<CamV2Command>,
    recording: bool,
    focus_mode: FocusMode,
    zoom_mode: ZoomMode,
    tracking: TrackingType,
    lens_id: u8,
}

/// MAVLink Camera Protocol v2 command to send.
#[derive(Debug, Clone, Copy)]
pub enum CamV2Command {
    TakePhoto,
    StartVideo,
    StopVideo,
    SetFocus(FocusMode),
    SetZoom(ZoomMode),
    SetTracking(TrackingType),
    SetLens(u8),
    RequestInfo,
    RequestSettings,
    RequestCaptureStatus,
}

impl MavlinkCamV2Backend {
    pub fn new(target_sysid: u8, target_compid: u8) -> Self {
        Self {
            target_sysid,
            target_compid,
            pending_cmd: None,
            recording: false,
            focus_mode: FocusMode::Auto,
            zoom_mode: ZoomMode::Percent(0),
            tracking: TrackingType::None,
            lens_id: 0,
        }
    }
}

impl CameraBackend for MavlinkCamV2Backend {
    fn trigger_shutter(&mut self) {
        self.pending_cmd = Some(CamV2Command::TakePhoto);
    }

    fn start_video(&mut self) {
        self.recording = true;
        self.pending_cmd = Some(CamV2Command::StartVideo);
    }

    fn stop_video(&mut self) {
        self.recording = false;
        self.pending_cmd = Some(CamV2Command::StopVideo);
    }

    fn is_recording(&self) -> bool {
        self.recording
    }

    fn update(&mut self, _now_ms: u32) {
        // pending_cmd is cleared by caller after sending
    }

    fn set_focus(&mut self, mode: FocusMode) {
        self.focus_mode = mode;
        self.pending_cmd = Some(CamV2Command::SetFocus(mode));
    }

    fn set_zoom(&mut self, mode: ZoomMode) {
        self.zoom_mode = mode;
        self.pending_cmd = Some(CamV2Command::SetZoom(mode));
    }

    fn set_tracking(&mut self, tracking: TrackingType) {
        self.tracking = tracking;
        self.pending_cmd = Some(CamV2Command::SetTracking(tracking));
    }

    fn set_lens(&mut self, lens_id: u8) {
        self.lens_id = lens_id;
        self.pending_cmd = Some(CamV2Command::SetLens(lens_id));
    }
}

/// RunCam serial protocol backend stub.
///
/// Source: ArduPilot `AP_Camera_RunCam.cpp`
/// Uses a simple serial command protocol for RunCam Split/Nano cameras.
pub struct RunCamBackend {
    /// Whether a shutter trigger packet needs to be sent (caller reads).
    pub trigger_pending: bool,
    /// Whether a start-record packet needs to be sent.
    pub start_record_pending: bool,
    /// Whether a stop-record packet needs to be sent.
    pub stop_record_pending: bool,
    recording: bool,
}

impl RunCamBackend {
    pub fn new() -> Self {
        Self {
            trigger_pending: false,
            start_record_pending: false,
            stop_record_pending: false,
            recording: false,
        }
    }
}

impl CameraBackend for RunCamBackend {
    fn trigger_shutter(&mut self) {
        self.trigger_pending = true;
    }

    fn start_video(&mut self) {
        self.recording = true;
        self.start_record_pending = true;
    }

    fn stop_video(&mut self) {
        self.recording = false;
        self.stop_record_pending = true;
    }

    fn is_recording(&self) -> bool {
        self.recording
    }

    fn update(&mut self, _now_ms: u32) {
        // Caller clears pending flags after serial transmission
    }
}
