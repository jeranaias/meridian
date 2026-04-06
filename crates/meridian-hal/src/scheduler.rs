//! Scheduler trait — task registration and timing.
//!
//! Source: ArduPilot `AP_HAL/Scheduler.h`
//! ChibiOS: 7 threads (timer@181/1kHz, main@180/400Hz, io@58/100Hz, etc.)
//! RTIC: maps to hardware interrupts + software tasks with priority ceiling.

/// Timer callback function pointer.
pub type TimerProc = fn();

/// IO callback function pointer (lower priority than timer).
pub type IoProc = fn();

/// Scheduler trait — manages periodic callbacks and timing.
pub trait Scheduler {
    /// Initialize the scheduler (spawn threads/tasks).
    fn init(&mut self);

    /// Register a timer callback (runs at 1kHz, high priority).
    /// Used for IMU accumulate, sensor sampling.
    /// Max 8 callbacks.
    fn register_timer_process(&mut self, proc: TimerProc) -> bool;

    /// Register an IO callback (runs at ~1kHz, low priority).
    /// Used for compass, baro, slower sensors.
    /// Max 8 callbacks.
    fn register_io_process(&mut self, proc: IoProc) -> bool;

    /// Delay for the specified number of microseconds.
    /// On RTIC: yields to other tasks. On SITL: real sleep.
    fn delay_microseconds(&self, us: u32);

    /// Delay with priority boost (temporarily raise to 182).
    /// Called by main loop to ensure prompt wakeup after sensor DMA.
    fn delay_microseconds_boost(&self, us: u32);

    /// Milliseconds since boot (wraps at ~49 days).
    fn millis(&self) -> u32;

    /// Microseconds since boot (wraps at ~71 minutes on 32-bit).
    fn micros(&self) -> u32;

    /// 64-bit microseconds (never wraps in practice).
    fn micros64(&self) -> u64;

    /// Whether the system has completed initialization.
    fn is_system_initialized(&self) -> bool;

    /// Mark system as initialized (called after setup() completes).
    fn set_system_initialized(&mut self);

    /// Check if we're in the main thread context.
    fn in_main_thread(&self) -> bool;

    /// Create a new thread/task with the given priority.
    fn create_thread(&mut self, name: &str, stack_size: u32, priority: u8, proc: fn());

    /// Reboot the system.
    fn reboot(&self, hold_in_bootloader: bool) -> !;
}
