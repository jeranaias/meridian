//! Rate group scheduler for SITL.
//!
//! Deterministic time-stepped scheduler that runs subsystems at defined frequencies.
//! On real hardware (RTIC), rate groups map to interrupt priorities.
//! In SITL, we step explicitly for reproducible results.

use meridian_types::time::{Instant, Duration};

/// Rate group identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RateGroupId {
    /// 1000 Hz — IMU sampling
    ImuSample,
    /// 400 Hz — EKF predict, attitude/rate control, motor output
    FastLoop,
    /// 50 Hz — Position control, navigation, RC input, EKF fusion
    Medium,
    /// 10 Hz — Failsafe, mode logic, mission engine, telemetry, logging
    Slow,
    /// 1 Hz — GCS heartbeat, battery monitor, pre-arm checks
    Heartbeat,
}

/// A rate group with timing state.
#[derive(Debug, Clone)]
pub struct RateGroup {
    pub id: RateGroupId,
    pub interval: Duration,
    pub last_run: Instant,
    pub run_count: u64,
    pub max_elapsed: Duration,
}

impl RateGroup {
    pub fn new(id: RateGroupId, hz: u32) -> Self {
        Self {
            id,
            interval: Duration::from_hz(hz),
            last_run: Instant::ZERO,
            run_count: 0,
            max_elapsed: Duration::ZERO,
        }
    }

    /// Check if this group should run at the given time.
    pub fn should_run(&self, now: Instant) -> bool {
        // First run always fires
        self.run_count == 0 || now.as_micros() >= self.last_run.as_micros() + self.interval.as_micros()
    }

    /// Mark this group as having run. Advances by interval (not by now) to prevent drift.
    pub fn mark_run(&mut self, now: Instant) {
        if self.run_count == 0 {
            self.last_run = now;
        } else {
            // Advance by interval from last scheduled time (not from now)
            // This prevents timing drift and maintains correct long-term frequency
            self.last_run = Instant::from_micros(
                self.last_run.as_micros() + self.interval.as_micros()
            );
        }
        self.run_count += 1;
    }
}

/// Per-task timing budget tracker.
/// Call `start()` before executing, `end()` after, then check `overrun()`.
#[derive(Debug, Clone, Copy)]
pub struct TaskBudget {
    /// Time budget for this task (microseconds).
    pub budget_us: u32,
    /// Timestamp when task started (microseconds).
    start_us: u64,
    /// Actual execution time of last run (microseconds).
    actual_us: u32,
    /// Whether the last run exceeded the budget.
    overrun: bool,
    /// Maximum observed execution time (microseconds).
    max_actual_us: u32,
    /// Total overrun count.
    overrun_count: u32,
}

impl TaskBudget {
    /// Create a new budget tracker with the given budget in microseconds.
    pub fn new(budget_us: u32) -> Self {
        Self {
            budget_us,
            start_us: 0,
            actual_us: 0,
            overrun: false,
            max_actual_us: 0,
            overrun_count: 0,
        }
    }

    /// Mark the start of task execution.
    pub fn start(&mut self, now: Instant) {
        self.start_us = now.as_micros();
    }

    /// Mark the end of task execution. Computes actual time and flags overrun.
    pub fn end(&mut self, now: Instant) {
        let end_us = now.as_micros();
        self.actual_us = if end_us >= self.start_us {
            (end_us - self.start_us) as u32
        } else {
            0
        };

        if self.actual_us > self.max_actual_us {
            self.max_actual_us = self.actual_us;
        }

        self.overrun = self.actual_us > self.budget_us;
        if self.overrun {
            self.overrun_count += 1;
        }
    }

    /// Returns true if the last run exceeded the time budget.
    pub fn overrun(&self) -> bool {
        self.overrun
    }

    /// Actual execution time of the last run (microseconds).
    pub fn actual_us(&self) -> u32 {
        self.actual_us
    }

    /// Maximum observed execution time across all runs.
    pub fn max_actual_us(&self) -> u32 {
        self.max_actual_us
    }

    /// Total number of overruns observed.
    pub fn overrun_count(&self) -> u32 {
        self.overrun_count
    }
}

/// The SITL scheduler: deterministic, steps all rate groups.
pub struct Scheduler {
    pub groups: [RateGroup; 5],
    /// Per-group task budgets.
    pub budgets: [TaskBudget; 5],
}

impl Scheduler {
    pub fn new() -> Self {
        Self {
            groups: [
                RateGroup::new(RateGroupId::ImuSample, 1000),
                RateGroup::new(RateGroupId::FastLoop, 400),
                RateGroup::new(RateGroupId::Medium, 50),
                RateGroup::new(RateGroupId::Slow, 10),
                RateGroup::new(RateGroupId::Heartbeat, 1),
            ],
            budgets: [
                TaskBudget::new(1000),   // IMU: 1000us (1ms)
                TaskBudget::new(2500),   // FastLoop: 2500us (2.5ms)
                TaskBudget::new(20000),  // Medium: 20ms
                TaskBudget::new(100000), // Slow: 100ms
                TaskBudget::new(1000000),// Heartbeat: 1s
            ],
        }
    }

    /// Check which rate groups should run at the given time.
    /// Returns a bitmask of group indices that are due.
    pub fn tick(&mut self, now: Instant) -> [bool; 5] {
        let mut due = [false; 5];
        for (i, group) in self.groups.iter_mut().enumerate() {
            if group.should_run(now) {
                due[i] = true;
                group.mark_run(now);
            }
        }
        due
    }

    /// Get the group by ID.
    pub fn group(&self, id: RateGroupId) -> &RateGroup {
        let idx = Self::group_index(id);
        &self.groups[idx]
    }

    /// Start timing a task's execution.
    pub fn budget_start(&mut self, id: RateGroupId, now: Instant) {
        let idx = Self::group_index(id);
        self.budgets[idx].start(now);
    }

    /// End timing a task's execution. Returns true if overrun occurred.
    pub fn budget_end(&mut self, id: RateGroupId, now: Instant) -> bool {
        let idx = Self::group_index(id);
        self.budgets[idx].end(now);
        self.budgets[idx].overrun()
    }

    /// Get the budget for a rate group.
    pub fn budget(&self, id: RateGroupId) -> &TaskBudget {
        let idx = Self::group_index(id);
        &self.budgets[idx]
    }

    fn group_index(id: RateGroupId) -> usize {
        match id {
            RateGroupId::ImuSample => 0,
            RateGroupId::FastLoop => 1,
            RateGroupId::Medium => 2,
            RateGroupId::Slow => 3,
            RateGroupId::Heartbeat => 4,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rate_groups() {
        let mut sched = Scheduler::new();

        // At t=0, all groups should fire
        let due = sched.tick(Instant::from_micros(0));
        assert!(due.iter().all(|&d| d));

        // At t=500us (0.5ms), only IMU (1000Hz = 1ms interval) should NOT fire yet
        let due = sched.tick(Instant::from_micros(500));
        assert!(!due[0]); // IMU not yet (interval = 1000us)

        // At t=1000us (1ms), IMU should fire
        let due = sched.tick(Instant::from_micros(1000));
        assert!(due[0]); // IMU fires at 1ms

        // At t=2500us (2.5ms), IMU fires again, FastLoop fires (interval=2500us)
        let due = sched.tick(Instant::from_micros(2500));
        assert!(due[0]); // IMU (last ran at 1000, interval 1000, due at 2000)
        assert!(due[1]); // FastLoop (last ran at 0, interval 2500, due at 2500)
    }

    // ── TaskBudget tests ──

    #[test]
    fn test_task_budget_no_overrun() {
        let mut budget = TaskBudget::new(1000); // 1ms budget
        budget.start(Instant::from_micros(0));
        budget.end(Instant::from_micros(500)); // 500us < 1000us
        assert!(!budget.overrun());
        assert_eq!(budget.actual_us(), 500);
        assert_eq!(budget.overrun_count(), 0);
    }

    #[test]
    fn test_task_budget_overrun() {
        let mut budget = TaskBudget::new(1000); // 1ms budget
        budget.start(Instant::from_micros(0));
        budget.end(Instant::from_micros(1500)); // 1500us > 1000us
        assert!(budget.overrun());
        assert_eq!(budget.actual_us(), 1500);
        assert_eq!(budget.overrun_count(), 1);
    }

    #[test]
    fn test_task_budget_max_tracking() {
        let mut budget = TaskBudget::new(5000);

        budget.start(Instant::from_micros(0));
        budget.end(Instant::from_micros(1000));
        assert_eq!(budget.max_actual_us(), 1000);

        budget.start(Instant::from_micros(10000));
        budget.end(Instant::from_micros(13000)); // 3000us
        assert_eq!(budget.max_actual_us(), 3000);

        // Shorter run doesn't change max
        budget.start(Instant::from_micros(20000));
        budget.end(Instant::from_micros(20500));
        assert_eq!(budget.max_actual_us(), 3000);
    }

    #[test]
    fn test_task_budget_overrun_count_accumulates() {
        let mut budget = TaskBudget::new(100);

        // 3 overruns
        for i in 0..3u64 {
            budget.start(Instant::from_micros(i * 1000));
            budget.end(Instant::from_micros(i * 1000 + 200)); // 200us > 100us
        }
        assert_eq!(budget.overrun_count(), 3);

        // 1 non-overrun
        budget.start(Instant::from_micros(5000));
        budget.end(Instant::from_micros(5050)); // 50us < 100us
        assert!(!budget.overrun());
        assert_eq!(budget.overrun_count(), 3); // unchanged
    }

    #[test]
    fn test_task_budget_exact_boundary() {
        let mut budget = TaskBudget::new(1000);
        budget.start(Instant::from_micros(0));
        budget.end(Instant::from_micros(1000)); // exactly at budget
        assert!(!budget.overrun()); // at boundary = not overrun
    }

    #[test]
    fn test_scheduler_budget_integration() {
        let mut sched = Scheduler::new();

        // Simulate timing a FastLoop execution
        sched.budget_start(RateGroupId::FastLoop, Instant::from_micros(0));
        let overrun = sched.budget_end(RateGroupId::FastLoop, Instant::from_micros(3000));
        // Budget is 2500us, actual is 3000us → overrun
        assert!(overrun);
        assert_eq!(sched.budget(RateGroupId::FastLoop).actual_us(), 3000);
        assert_eq!(sched.budget(RateGroupId::FastLoop).overrun_count(), 1);
    }

    #[test]
    fn test_one_second_counts() {
        let mut sched = Scheduler::new();

        // Run for 1 second at physics rate
        let dt_us = (1_000_000u64 / super::super::PHYSICS_HZ as u64);
        let steps = super::super::PHYSICS_HZ;
        for i in 0..steps {
            let now = Instant::from_micros(i as u64 * dt_us);
            sched.tick(now);
        }

        // Check approximate counts (±1 for boundary effects)
        assert!((sched.groups[0].run_count as i64 - 1000).abs() <= 2, // IMU 1000Hz
            "IMU count: {}", sched.groups[0].run_count);
        assert!((sched.groups[1].run_count as i64 - 400).abs() <= 2,  // Fast 400Hz
            "Fast count: {}", sched.groups[1].run_count);
        assert!((sched.groups[2].run_count as i64 - 50).abs() <= 2,   // Medium 50Hz
            "Medium count: {}", sched.groups[2].run_count);
        assert!((sched.groups[3].run_count as i64 - 10).abs() <= 2,   // Slow 10Hz
            "Slow count: {}", sched.groups[3].run_count);
        assert!((sched.groups[4].run_count as i64 - 1).abs() <= 1,    // Heartbeat 1Hz
            "Heartbeat count: {}", sched.groups[4].run_count);
    }
}
