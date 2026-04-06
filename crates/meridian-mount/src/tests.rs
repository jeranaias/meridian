//! Unit tests for mount backends and manager.

#[cfg(test)]
mod servo_tests {
    use crate::servo::{ServoMount, ServoChannel};
    use crate::backend::{MountBackend, MountTarget};

    fn default_pitch_channel() -> ServoChannel {
        ServoChannel {
            channel: 1,
            pwm_min: 1000, pwm_max: 2000,
            angle_min: -core::f32::consts::FRAC_PI_2,
            angle_max: core::f32::consts::FRAC_PI_2,
            reversed: false,
        }
    }

    fn default_yaw_channel() -> ServoChannel {
        ServoChannel {
            channel: 2,
            pwm_min: 1000, pwm_max: 2000,
            angle_min: -core::f32::consts::PI,
            angle_max: core::f32::consts::PI,
            reversed: false,
        }
    }

    #[test]
    fn test_servo_init() {
        let mut mount = ServoMount::new(default_pitch_channel(), None, None);
        assert!(mount.init());
    }

    #[test]
    fn test_servo_angle_target() {
        let mut mount = ServoMount::new(default_pitch_channel(), None, Some(default_yaw_channel()));
        mount.init();
        mount.update(&MountTarget::Angle { roll: 0.0, pitch: 0.5, yaw: -0.3 }, 0.01);
        let (_, p, y) = mount.get_attitude();
        assert!((p - 0.5).abs() < 1e-5);
        assert!((y - (-0.3)).abs() < 1e-5);
    }

    #[test]
    fn test_servo_rate_target_integrates() {
        let mut mount = ServoMount::new(default_pitch_channel(), None, Some(default_yaw_channel()));
        mount.init();
        let target = MountTarget::Rate { roll_rate: 0.0, pitch_rate: 1.0, yaw_rate: -0.5 };
        for _ in 0..10 {
            mount.update(&target, 0.01);
        }
        let (_, p, y) = mount.get_attitude();
        assert!((p - 0.1).abs() < 1e-4);
        assert!((y - (-0.05)).abs() < 1e-4);
    }

    #[test]
    fn test_servo_pwm_output_neutral() {
        let mut mount = ServoMount::new(default_pitch_channel(), None, None);
        mount.init();
        mount.update(&MountTarget::Neutral, 0.01);
        assert_eq!(mount.output_pwm[1], 1500);
    }

    #[test]
    fn test_servo_channel_reversed() {
        let ch = ServoChannel {
            channel: 0, pwm_min: 1000, pwm_max: 2000,
            angle_min: -1.0, angle_max: 1.0, reversed: true,
        };
        assert_eq!(ch.angle_to_pwm(1.0), 1000);
        assert_eq!(ch.angle_to_pwm(-1.0), 2000);
    }
}

#[cfg(test)]
mod mavlink_tests {
    use crate::mavlink::MavlinkMount;
    use crate::backend::{MountBackend, MountTarget};

    #[test]
    fn test_mavlink_init() {
        let mut mount = MavlinkMount::new(1, 154);
        assert!(mount.init());
    }

    #[test]
    fn test_mavlink_angle_generates_message() {
        let mut mount = MavlinkMount::new(1, 154);
        mount.init();
        mount.update(&MountTarget::Angle { roll: 0.0, pitch: 0.5, yaw: -0.3 }, 0.01);
        assert!(mount.out_msg.pending);
        assert!((mount.out_msg.pitch_cd - 2865).abs() < 2);
    }
}

#[cfg(test)]
mod siyi_tests {
    use crate::siyi::{SiyiMount, crc16_ccitt};
    use crate::backend::{MountBackend, MountTarget};

    #[test]
    fn test_siyi_init() {
        let mut mount = SiyiMount::new();
        assert!(mount.init());
    }

    #[test]
    fn test_siyi_packet_magic() {
        let mut mount = SiyiMount::new();
        mount.init();
        mount.update(&MountTarget::Angle { roll: 0.0, pitch: 0.3, yaw: 0.0 }, 0.01);
        assert!(mount.out_packet.pending);
        assert_eq!(mount.out_packet.data[0], 0x66);
        assert_eq!(mount.out_packet.data[1], 0x55);
    }

    #[test]
    fn test_siyi_neutral_produces_zero_rate() {
        let mut mount = SiyiMount::new();
        mount.init();
        mount.update(&MountTarget::Neutral, 0.01);
        let yaw_scalar = mount.out_packet.data[8] as i8;
        let pitch_scalar = mount.out_packet.data[9] as i8;
        assert_eq!(yaw_scalar, 0);
        assert_eq!(pitch_scalar, 0);
    }

    #[test]
    fn test_siyi_attitude_polling() {
        let mut mount = SiyiMount::new();
        mount.init();
        assert!(mount.poll_attitude_if_due(0));
        assert!(mount.attitude_request_packet.pending);
        assert!(!mount.poll_attitude_if_due(30)); // too soon
        assert!(mount.poll_attitude_if_due(50)); // 20Hz interval
    }

    #[test]
    fn test_siyi_external_attitude() {
        let mut mount = SiyiMount::new();
        mount.init();
        mount.build_external_attitude(0.1, 0.2, 0.3);
        assert!(mount.external_attitude_packet.pending);
        assert_eq!(mount.external_attitude_packet.data[7], 0x22);
    }

    #[test]
    fn test_siyi_upside_down_transform() {
        let mut mount = SiyiMount::new();
        mount.upside_down = true;
        mount.set_attitude_feedback(0.0, 0.5, 1.0);
        let (_, p, y) = mount.get_attitude();
        // pitch_transformed = -(0.5 + PI)
        assert!((p - (-(0.5 + core::f32::consts::PI))).abs() < 0.01);
        // yaw_transformed = -1.0
        assert!((y - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn test_crc16_ccitt_known() {
        let data = b"123456789";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, 0x31C3);
    }

    #[test]
    fn test_siyi_sequence_increments() {
        let mut mount = SiyiMount::new();
        mount.init();
        mount.update(&MountTarget::Neutral, 0.01);
        let seq0 = mount.out_packet.data[5];
        mount.update(&MountTarget::Neutral, 0.01);
        let seq1 = mount.out_packet.data[5];
        assert_eq!(seq1, seq0 + 1);
    }
}

#[cfg(test)]
mod manager_tests {
    use crate::manager::{MountManager, MountMode, MountBackendType};
    use crate::servo::{ServoMount, ServoChannel};
    use crate::backend::MountTarget;

    fn test_servo() -> ServoMount {
        let pitch = ServoChannel {
            channel: 1, pwm_min: 1000, pwm_max: 2000,
            angle_min: -core::f32::consts::FRAC_PI_2,
            angle_max: core::f32::consts::FRAC_PI_2,
            reversed: false,
        };
        let yaw = ServoChannel {
            channel: 2, pwm_min: 1000, pwm_max: 2000,
            angle_min: -core::f32::consts::PI,
            angle_max: core::f32::consts::PI,
            reversed: false,
        };
        ServoMount::new(pitch, None, Some(yaw))
    }

    #[test]
    fn test_manager_default_neutral() {
        let mgr = MountManager::new(MountBackendType::Servo(test_servo()));
        assert_eq!(mgr.mode, MountMode::Neutral);
    }

    #[test]
    fn test_manager_set_mode_retract() {
        let mut mgr = MountManager::new(MountBackendType::Servo(test_servo()));
        mgr.init();
        mgr.set_mode(MountMode::Retract);
        assert_eq!(mgr.target, MountTarget::Retract);
    }

    #[test]
    fn test_manager_rc_targeting_rate_mode() {
        let mut mgr = MountManager::new(MountBackendType::Servo(test_servo()));
        mgr.init();
        mgr.rc_rate_scale = 2.0;
        mgr.rc_rate_mode = true;
        mgr.set_mode(MountMode::RcTargeting);
        mgr.update_rc_targeting(0.5, -0.3);
        mgr.update(0.01);
        let (_, p, y) = mgr.get_attitude();
        assert!((p - 0.01).abs() < 1e-4);
        assert!((y - (-0.006)).abs() < 1e-4);
    }

    #[test]
    fn test_manager_rc_targeting_angle_mode() {
        let mut mgr = MountManager::new(MountBackendType::Servo(test_servo()));
        mgr.init();
        mgr.rc_rate_mode = false;
        mgr.set_mode(MountMode::RcTargeting);
        mgr.update_rc_targeting(1.0, 0.0);
        mgr.update(0.01);
        let (_, p, _) = mgr.get_attitude();
        // Full deflection maps to PI/2
        assert!((p - core::f32::consts::FRAC_PI_2).abs() < 0.01);
    }

    #[test]
    fn test_manager_roi_resolution() {
        let mut mgr = MountManager::new(MountBackendType::Servo(test_servo()));
        mgr.init();
        mgr.set_vehicle_position(35.0, -120.0, 100.0);
        mgr.set_mode(MountMode::GpsTargeting);
        // ROI 100m north, at same altitude
        mgr.set_roi_target(35.0009, -120.0, 100.0);
        mgr.update(0.01);

        let (_, pitch, yaw) = mgr.get_attitude();
        // Bearing should be ~0 (north)
        assert!(yaw.abs() < 0.1, "Yaw should be ~0 for due north ROI, got {}", yaw);
        // Pitch should be ~0 for same altitude
        assert!(pitch.abs() < 0.1, "Pitch should be ~0 for same alt, got {}", pitch);
    }

    #[test]
    fn test_manager_roi_below() {
        let mut mgr = MountManager::new(MountBackendType::Servo(test_servo()));
        mgr.init();
        mgr.set_vehicle_position(35.0, -120.0, 200.0);
        mgr.set_mode(MountMode::GpsTargeting);
        // ROI directly below (same lat/lon but lower alt)
        mgr.set_roi_target(35.0001, -120.0, 0.0);
        mgr.update(0.01);

        let (_, pitch, _) = mgr.get_attitude();
        // Should be looking down (negative pitch in NED, or positive in our convention)
        assert!(pitch > 0.1, "Pitch should be positive (looking down), got {}", pitch);
    }
}
