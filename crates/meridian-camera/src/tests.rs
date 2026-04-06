//! Unit tests for camera backends, geotagging, and trigger logic.

#[cfg(test)]
mod servo_camera_tests {
    use crate::backend::{CameraBackend, ServoCameraBackend};

    #[test]
    fn test_servo_initial_state() {
        let cam = ServoCameraBackend::new(0, 1100, 1900, 100);
        assert_eq!(cam.current_pwm, 1100);
        assert!(!cam.is_pulsing());
        assert!(!cam.is_recording());
    }

    #[test]
    fn test_servo_trigger_pulse() {
        let mut cam = ServoCameraBackend::new(0, 1100, 1900, 100);

        cam.trigger_shutter();
        assert!(cam.is_pulsing());

        cam.update(1000);
        assert_eq!(cam.current_pwm, 1900);

        cam.update(1050);
        assert_eq!(cam.current_pwm, 1900);

        cam.update(1100);
        assert_eq!(cam.current_pwm, 1100);
        assert!(!cam.is_pulsing());
    }

    #[test]
    fn test_servo_video_toggle() {
        let mut cam = ServoCameraBackend::new(0, 1100, 1900, 100);
        assert!(!cam.is_recording());
        cam.start_video();
        assert!(cam.is_recording());
        cam.stop_video();
        assert!(!cam.is_recording());
    }
}

#[cfg(test)]
mod relay_camera_tests {
    use crate::backend::{CameraBackend, RelayCameraBackend};

    #[test]
    fn test_relay_initial_state() {
        let cam = RelayCameraBackend::new(5, true, 50);
        assert!(!cam.current_state);
    }

    #[test]
    fn test_relay_trigger_pulse() {
        let mut cam = RelayCameraBackend::new(5, true, 50);

        cam.trigger_shutter();
        cam.update(2000);
        assert!(cam.current_state);

        cam.update(2030);
        assert!(cam.current_state);

        cam.update(2050);
        assert!(!cam.current_state);
    }

    #[test]
    fn test_relay_active_low() {
        let mut cam = RelayCameraBackend::new(5, false, 50);
        assert!(cam.current_state);

        cam.trigger_shutter();
        cam.update(1000);
        assert!(!cam.current_state);

        cam.update(1050);
        assert!(cam.current_state);
    }
}

#[cfg(test)]
mod backend_stub_tests {
    use crate::backend::{CameraBackend, MountCameraBackend, RunCamBackend, MavlinkCamV2Backend, FocusMode, ZoomMode, TrackingType};

    #[test]
    fn test_mount_backend() {
        let mut b = MountCameraBackend::new(0);
        b.trigger_shutter();
        assert!(b.trigger_pending);
        assert!(!b.is_recording());
        b.start_video();
        assert!(b.is_recording());
    }

    #[test]
    fn test_runcam_backend() {
        let mut b = RunCamBackend::new();
        b.trigger_shutter();
        assert!(b.trigger_pending);
        b.start_video();
        assert!(b.is_recording());
        assert!(b.start_record_pending);
        b.stop_video();
        assert!(!b.is_recording());
        assert!(b.stop_record_pending);
    }

    #[test]
    fn test_mavlink_camv2_backend() {
        let mut b = MavlinkCamV2Backend::new(1, 100);
        b.trigger_shutter();
        assert!(b.pending_cmd.is_some());

        b.set_focus(FocusMode::Auto);
        b.set_zoom(ZoomMode::Percent(50));
        b.set_tracking(TrackingType::Point { x: 0.5, y: 0.5 });
        b.set_lens(2);
    }
}

#[cfg(test)]
mod geotag_tests {
    use crate::geotag::{GeotagEntry, GeotagLog};

    #[test]
    fn test_geotag_log_empty() {
        let log: GeotagLog<8> = GeotagLog::new();
        assert!(log.is_empty());
        assert_eq!(log.len(), 0);
        assert!(log.last().is_none());
    }

    #[test]
    fn test_geotag_log_push_and_last() {
        let mut log: GeotagLog<8> = GeotagLog::new();
        log.push(GeotagEntry {
            lat: 35.0,
            lon: -120.0,
            alt: 100.0,
            alt_agl: 50.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 1.57,
            timestamp_ms: 1000,
            image_index: 1,
        });
        assert_eq!(log.len(), 1);
        let last = log.last().unwrap();
        assert!((last.lat - 35.0).abs() < 1e-10);
        assert!((last.alt_agl - 50.0).abs() < 1e-5);
        assert_eq!(last.timestamp_ms, 1000);
    }

    #[test]
    fn test_geotag_log_wrap_around() {
        let mut log: GeotagLog<4> = GeotagLog::new();
        for i in 0..6 {
            log.push(GeotagEntry {
                lat: i as f64,
                timestamp_ms: i * 100,
                ..GeotagEntry::default()
            });
        }
        assert_eq!(log.len(), 4);
        let last = log.last().unwrap();
        assert!((last.lat - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_geotag_log_get_oldest() {
        let mut log: GeotagLog<4> = GeotagLog::new();
        for i in 0..6u32 {
            log.push(GeotagEntry {
                lat: i as f64,
                timestamp_ms: i * 100,
                ..GeotagEntry::default()
            });
        }
        let oldest = log.get(0).unwrap();
        assert!((oldest.lat - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_geotag_log_get_out_of_bounds() {
        let log: GeotagLog<4> = GeotagLog::new();
        assert!(log.get(0).is_none());
    }
}

#[cfg(test)]
mod trigger_tests {
    use crate::backend::{ServoCameraBackend, CameraBackend};
    use crate::trigger::{CameraManager, CameraBackendType, TriggerMode};

    fn test_camera() -> CameraManager {
        let backend = ServoCameraBackend::new(0, 1100, 1900, 100);
        CameraManager::new(CameraBackendType::Servo(backend))
    }

    #[test]
    fn test_manual_trigger() {
        let mut cam = test_camera();
        assert_eq!(cam.photo_count, 0);

        cam.trigger_shutter(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 1.57, 1000);
        assert_eq!(cam.photo_count, 1);
        assert_eq!(cam.geotags.len(), 1);
        let entry = cam.geotags.last().unwrap();
        assert!((entry.lat - 35.0).abs() < 1e-10);
        assert!((entry.alt_agl - 50.0).abs() < 1e-5);
    }

    #[test]
    fn test_time_trigger() {
        let mut cam = test_camera();
        cam.mode = TriggerMode::Time;
        cam.time_interval_ms = 500;

        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1000);
        assert_eq!(cam.photo_count, 1);

        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1200);
        assert_eq!(cam.photo_count, 1);

        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1500);
        assert_eq!(cam.photo_count, 2);
    }

    #[test]
    fn test_distance_trigger() {
        let mut cam = test_camera();
        cam.mode = TriggerMode::Distance;
        cam.distance_threshold_m = 50.0;

        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1000);
        assert_eq!(cam.photo_count, 1);

        cam.update(35.0001, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 2000);
        assert_eq!(cam.photo_count, 1);

        cam.update(35.001, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 3000);
        assert_eq!(cam.photo_count, 2);
    }

    #[test]
    fn test_manual_mode_no_auto_trigger() {
        let mut cam = test_camera();
        cam.mode = TriggerMode::Manual;

        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1000);
        assert_eq!(cam.photo_count, 0, "Manual mode should not auto-trigger");
    }

    #[test]
    fn test_burst_count_mode() {
        let mut cam = test_camera();
        cam.start_burst(100, 3); // 3 photos at 100ms interval

        for i in 0..10 {
            cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, i * 100);
        }
        assert_eq!(cam.photo_count, 3, "Should stop after burst_count photos");
    }

    #[test]
    fn test_min_interval_guard() {
        let mut cam = test_camera();
        cam.mode = TriggerMode::Time;
        cam.time_interval_ms = 10;
        cam.min_interval_ms = 500;

        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1000);
        assert_eq!(cam.photo_count, 1);

        // Try again at 100ms — min interval should block
        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1100);
        assert_eq!(cam.photo_count, 1, "Min interval should block trigger");

        // After min interval
        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1500);
        assert_eq!(cam.photo_count, 2);
    }

    #[test]
    fn test_auto_mode_only_flag() {
        let mut cam = test_camera();
        cam.mode = TriggerMode::Distance;
        cam.distance_threshold_m = 50.0;
        cam.auto_mode_only = true;
        cam.is_auto_mode = false;

        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1000);
        assert_eq!(cam.photo_count, 0, "Should not trigger when not in AUTO mode");

        cam.is_auto_mode = true;
        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1100);
        assert_eq!(cam.photo_count, 1, "Should trigger when in AUTO mode");
    }

    #[test]
    fn test_roll_limit_guard() {
        let mut cam = test_camera();
        cam.mode = TriggerMode::Time;
        cam.time_interval_ms = 100;
        cam.max_roll_rad = 0.1; // ~5.7 degrees

        // First photo at zero roll
        cam.update(35.0, -120.0, 100.0, 50.0, 0.0, 0.0, 0.0, 1000);
        assert_eq!(cam.photo_count, 1);

        // Excessive roll — should not trigger
        cam.update(35.0, -120.0, 100.0, 50.0, 0.3, 0.0, 0.0, 1200);
        assert_eq!(cam.photo_count, 1, "Should not trigger with excessive roll");

        // Roll within limit
        cam.update(35.0, -120.0, 100.0, 50.0, 0.05, 0.0, 0.0, 1300);
        assert_eq!(cam.photo_count, 2);
    }

    #[test]
    fn test_video_control() {
        let mut cam = test_camera();
        cam.start_video();
        match &cam.backend {
            CameraBackendType::Servo(b) => assert!(b.is_recording()),
            _ => panic!("Wrong backend type"),
        }
        cam.stop_video();
        match &cam.backend {
            CameraBackendType::Servo(b) => assert!(!b.is_recording()),
            _ => panic!("Wrong backend type"),
        }
    }
}
