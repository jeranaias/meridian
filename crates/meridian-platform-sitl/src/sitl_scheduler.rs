//! SITL scheduler — real OS timing and cooperative callbacks.
//!
//! Timer/IO callbacks are stored here and invoked by the main loop (cooperative).
//! Background threads use `std::thread::spawn`.

use std::thread;
use std::time::{Duration, Instant};

use meridian_hal::scheduler::{IoProc, Scheduler, TimerProc};

const MAX_TIMER_PROCS: usize = 8;
const MAX_IO_PROCS: usize = 8;

/// SITL scheduler using OS time and threads.
///
/// Timing uses `std::time::Instant` relative to a boot epoch.
/// Timer and IO callbacks are cooperative -- call `run_timer_procs()` and
/// `run_io_procs()` from the main loop at the appropriate rates.
pub struct SitlScheduler {
    boot_time: Instant,
    timer_procs: [Option<TimerProc>; MAX_TIMER_PROCS],
    timer_count: usize,
    io_procs: [Option<IoProc>; MAX_IO_PROCS],
    io_count: usize,
    system_initialized: bool,
    main_thread_id: Option<thread::ThreadId>,
}

impl SitlScheduler {
    pub fn new() -> Self {
        Self {
            boot_time: Instant::now(),
            timer_procs: [None; MAX_TIMER_PROCS],
            timer_count: 0,
            io_procs: [None; MAX_IO_PROCS],
            io_count: 0,
            system_initialized: false,
            main_thread_id: None,
        }
    }

    /// Run all registered timer callbacks (call from main loop at ~1kHz).
    pub fn run_timer_procs(&self) {
        for i in 0..self.timer_count {
            if let Some(proc) = self.timer_procs[i] {
                proc();
            }
        }
    }

    /// Run all registered IO callbacks (call from main loop at ~1kHz).
    pub fn run_io_procs(&self) {
        for i in 0..self.io_count {
            if let Some(proc) = self.io_procs[i] {
                proc();
            }
        }
    }
}

impl Scheduler for SitlScheduler {
    fn init(&mut self) {
        self.main_thread_id = Some(thread::current().id());
    }

    fn register_timer_process(&mut self, proc: TimerProc) -> bool {
        if self.timer_count >= MAX_TIMER_PROCS {
            return false;
        }
        let idx = self.timer_count;
        self.timer_procs[idx] = Some(proc);
        self.timer_count += 1;
        true
    }

    fn register_io_process(&mut self, proc: IoProc) -> bool {
        if self.io_count >= MAX_IO_PROCS {
            return false;
        }
        let idx = self.io_count;
        self.io_procs[idx] = Some(proc);
        self.io_count += 1;
        true
    }

    fn delay_microseconds(&self, us: u32) {
        thread::sleep(Duration::from_micros(us as u64));
    }

    fn delay_microseconds_boost(&self, us: u32) {
        // On SITL there is no priority to boost; just sleep normally.
        thread::sleep(Duration::from_micros(us as u64));
    }

    fn millis(&self) -> u32 {
        self.boot_time.elapsed().as_millis() as u32
    }

    fn micros(&self) -> u32 {
        self.boot_time.elapsed().as_micros() as u32
    }

    fn micros64(&self) -> u64 {
        self.boot_time.elapsed().as_micros() as u64
    }

    fn is_system_initialized(&self) -> bool {
        self.system_initialized
    }

    fn set_system_initialized(&mut self) {
        self.system_initialized = true;
    }

    fn in_main_thread(&self) -> bool {
        match self.main_thread_id {
            Some(id) => thread::current().id() == id,
            None => true,
        }
    }

    fn create_thread(&mut self, name: &str, _stack_size: u32, _priority: u8, proc: fn()) {
        let thread_name = name.to_string();
        thread::Builder::new()
            .name(thread_name)
            .spawn(proc)
            .expect("failed to spawn SITL thread");
    }

    fn reboot(&self, _hold_in_bootloader: bool) -> ! {
        std::process::exit(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    static TIMER_COUNT: AtomicU32 = AtomicU32::new(0);
    static IO_COUNT: AtomicU32 = AtomicU32::new(0);

    fn test_timer_cb() {
        TIMER_COUNT.fetch_add(1, Ordering::SeqCst);
    }

    fn test_io_cb() {
        IO_COUNT.fetch_add(1, Ordering::SeqCst);
    }

    #[test]
    fn test_scheduler_timing() {
        let sched = SitlScheduler::new();
        let t0 = sched.millis();
        thread::sleep(Duration::from_millis(10));
        let t1 = sched.millis();
        assert!(t1 >= t0 + 5, "millis should advance");
    }

    #[test]
    fn test_scheduler_callbacks() {
        TIMER_COUNT.store(0, Ordering::SeqCst);
        IO_COUNT.store(0, Ordering::SeqCst);

        let mut sched = SitlScheduler::new();
        assert!(sched.register_timer_process(test_timer_cb));
        assert!(sched.register_io_process(test_io_cb));

        sched.run_timer_procs();
        sched.run_io_procs();

        assert_eq!(TIMER_COUNT.load(Ordering::SeqCst), 1);
        assert_eq!(IO_COUNT.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_scheduler_max_procs() {
        let mut sched = SitlScheduler::new();
        for _ in 0..MAX_TIMER_PROCS {
            assert!(sched.register_timer_process(test_timer_cb));
        }
        // 9th should fail.
        assert!(!sched.register_timer_process(test_timer_cb));
    }

    #[test]
    fn test_scheduler_init_flag() {
        let mut sched = SitlScheduler::new();
        assert!(!sched.is_system_initialized());
        sched.set_system_initialized();
        assert!(sched.is_system_initialized());
    }

    #[test]
    fn test_scheduler_in_main_thread() {
        let mut sched = SitlScheduler::new();
        sched.init();
        assert!(sched.in_main_thread());
    }
}
