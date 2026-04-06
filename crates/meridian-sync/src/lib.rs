// meridian-sync: Synchronization primitives for Meridian autopilot
//
// This crate exists to solve a specific porting problem:
// ArduPilot uses recursive (reentrant) mutexes everywhere.
// Rust's core::sync and std::sync Mutex types are NOT recursive.
// Attempting to lock a standard Mutex from the same thread/task that
// already holds it is undefined behaviour (may deadlock or panic).
//
// This prototype implements RecursiveMutex<T> for no_std environments
// using the RTIC task-ID model.  See docs/recursive_semaphore_research.md
// for full rationale and design trade-offs.
//
// # Threading model assumed here
//
// RTIC on Cortex-M does not have "threads" in the POSIX sense.  Instead
// it has *tasks* identified by their hardware priority level (0–255).
// Within a given priority level only one task can run at a time.  Higher-
// priority tasks preempt lower-priority tasks.
//
// We therefore use a `u8` task-ID that callers supply to `lock()`.  On
// RTIC the natural choice is the numeric priority of the calling task,
// which you can obtain from the RTIC `cx.priority().value()` method.
//
// # Ownership invariant
//
//  * `owner == NO_OWNER`          →  mutex is free
//  * `owner == task_id`           →  this task holds the mutex (recursion ok)
//  * `owner != task_id, != FREE`  →  another task holds it; spin-wait
//
// # Usage
//
//  ```no_run
//  use meridian_sync::{RecursiveMutex, TASK_ID_MAIN};
//
//  static BUS_LOCK: RecursiveMutex<()> = RecursiveMutex::new(());
//
//  fn driver_init(task_id: u8) {
//      let _guard = BUS_LOCK.lock(task_id); // first lock
//      helper(task_id);
//  }
//
//  fn helper(task_id: u8) {
//      let _guard = BUS_LOCK.lock(task_id); // second lock from same task — ok
//      // do work...
//  }  // guard dropped, recursion_count decrements
//  ```

#![no_std]

use core::{
    cell::UnsafeCell,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering},
};

/// Sentinel value meaning "no task owns the mutex".
pub const NO_OWNER: u8 = 0xFF;

/// Conventional task IDs for non-RTIC use (e.g. Linux simulation).
/// In RTIC, use the actual priority value instead.
pub const TASK_ID_MAIN: u8 = 0;
pub const TASK_ID_SPI_BUS: u8 = 1;
pub const TASK_ID_I2C_BUS: u8 = 2;
pub const TASK_ID_TIMER: u8 = 3;

/// Maximum recursion depth before we panic.
/// ArduPilot's deepest observed nesting is 3 levels.  We allow 8 for safety.
const MAX_RECURSION: u32 = 8;

/// A recursive (reentrant) mutex suitable for no_std / RTIC environments.
///
/// Uses two atomics:
///   - `spin_lock`:       outer spin lock that serialises access to owner/count
///   - `owner`:           task ID of the current holder (NO_OWNER = free)
/// Plus one atomic counter:
///   - `recursion_count`: how many times the owning task has locked
///
/// The inner value `data` is protected by `UnsafeCell`.
///
/// # Safety notes
///
/// This is safe when:
///   1. Each task uses a unique, stable `task_id`.
///   2. You do NOT call `lock()` from an ISR that can preempt the
///      owning task (use a critical section / `cortex_m::interrupt::free`
///      instead for true ISR access).
///   3. On bare metal, `task_id` comes from RTIC priority and tasks at
///      the same priority never preempt each other.
pub struct RecursiveMutex<T> {
    /// Spin lock — held only while reading/writing owner and recursion_count.
    /// Released immediately; not held during the critical section body.
    spin_lock: AtomicBool,
    /// The task ID that currently owns the mutex, or NO_OWNER.
    owner: AtomicU8,
    /// How many times the owner has called lock() without a matching unlock().
    recursion_count: AtomicU32,
    /// Protected data.
    data: UnsafeCell<T>,
}

// SAFETY: RecursiveMutex<T> is Sync because we guarantee single-owner access
// through the spin lock + owner + recursion_count protocol.
unsafe impl<T: Send> Sync for RecursiveMutex<T> {}
unsafe impl<T: Send> Send for RecursiveMutex<T> {}

impl<T> RecursiveMutex<T> {
    /// Create a new, unlocked RecursiveMutex.
    /// This is a `const fn` so it can initialise `static` variables.
    pub const fn new(value: T) -> Self {
        Self {
            spin_lock: AtomicBool::new(false),
            owner: AtomicU8::new(NO_OWNER),
            recursion_count: AtomicU32::new(0),
            data: UnsafeCell::new(value),
        }
    }

    /// Acquire the spin lock (busy-wait).
    ///
    /// The spin lock is held for O(1) work (a few atomic reads/writes) and
    /// then immediately released.  It is NOT held while the caller's critical
    /// section runs.  This keeps the spin-window tiny.
    #[inline(always)]
    fn acquire_spin(&self) {
        while self.spin_lock.compare_exchange_weak(
            false, true,
            Ordering::Acquire,
            Ordering::Relaxed,
        ).is_err() {
            // Micro-spin; on Cortex-M this is just a few cycles.
            core::hint::spin_loop();
        }
    }

    /// Release the spin lock.
    #[inline(always)]
    fn release_spin(&self) {
        self.spin_lock.store(false, Ordering::Release);
    }

    /// Lock the mutex as `task_id`.
    ///
    /// If `task_id` already holds the mutex this returns immediately
    /// (recursive re-entry).  Otherwise spins until the mutex is free,
    /// then takes ownership.
    ///
    /// Returns a `RecursiveGuard` that will decrement the recursion count
    /// (and release ownership when count reaches 0) when dropped.
    ///
    /// # Panics
    ///
    /// Panics if `task_id == NO_OWNER` (reserved sentinel).
    /// Panics if recursion depth exceeds `MAX_RECURSION`.
    pub fn lock(&self, task_id: u8) -> RecursiveGuard<'_, T> {
        // Validate task_id BEFORE touching the spin lock so a panic here
        // never leaves the spin lock held.
        if task_id == NO_OWNER {
            panic!("task_id 0xFF is reserved as NO_OWNER");
        }

        loop {
            // ---- enter spin lock ----
            self.acquire_spin();

            let current_owner = self.owner.load(Ordering::Relaxed);

            if current_owner == NO_OWNER {
                // Mutex is free — take it.
                self.owner.store(task_id, Ordering::Relaxed);
                self.recursion_count.store(1, Ordering::Relaxed);
                self.release_spin();
                // SAFETY: we are now the sole owner.
                return RecursiveGuard { mutex: self, task_id };

            } else if current_owner == task_id {
                // We already own it — recursive re-entry.
                let depth = self.recursion_count.load(Ordering::Relaxed);
                if depth >= MAX_RECURSION {
                    // Must release spin lock BEFORE panicking, otherwise
                    // unwinding drop handlers (e.g. other guards) that try to
                    // acquire the spin lock to call unlock() would deadlock.
                    self.release_spin();
                    panic!(
                        "RecursiveMutex recursion depth exceeded MAX_RECURSION ({})",
                        MAX_RECURSION
                    );
                }
                self.recursion_count.store(depth + 1, Ordering::Relaxed);
                self.release_spin();
                return RecursiveGuard { mutex: self, task_id };

            } else {
                // Mutex is held by a different task.  Release spin lock and
                // spin-wait before trying again.
                self.release_spin();
                core::hint::spin_loop();
                // On real RTIC targets the competing task will finish and
                // release soon.  On Linux/std tests this tight loop is fine
                // for short critical sections.
            }
        }
    }

    /// Release one level of ownership.
    ///
    /// Called automatically by `RecursiveGuard::drop()`.  You should not
    /// normally call this directly.
    ///
    /// # Panics
    ///
    /// Panics if called by a task that does not own the mutex.
    fn unlock(&self, task_id: u8) {
        self.acquire_spin();

        let current_owner = self.owner.load(Ordering::Relaxed);
        if current_owner != task_id {
            self.release_spin();
            panic!(
                "unlock() called by task {} but owner is {}",
                task_id, current_owner
            );
        }

        let depth = self.recursion_count.load(Ordering::Relaxed);
        if depth == 0 {
            self.release_spin();
            panic!("unlock() with zero recursion count — double-free");
        }

        if depth == 1 {
            // Final release — hand back to free.
            self.owner.store(NO_OWNER, Ordering::Relaxed);
            self.recursion_count.store(0, Ordering::Relaxed);
        } else {
            self.recursion_count.store(depth - 1, Ordering::Relaxed);
        }

        self.release_spin();
    }

    /// Returns `true` if `task_id` currently owns the mutex.
    ///
    /// Equivalent to ChibiOS `Semaphore::check_owner()`.
    /// Used in ArduPilot as a defensive assertion before bus transfers.
    pub fn is_owner(&self, task_id: u8) -> bool {
        self.acquire_spin();
        let owned = self.owner.load(Ordering::Relaxed) == task_id;
        self.release_spin();
        owned
    }

    /// Returns the current recursion depth (0 = unlocked).
    /// Primarily useful for testing and assertions.
    pub fn recursion_depth(&self) -> u32 {
        self.acquire_spin();
        let d = self.recursion_count.load(Ordering::Relaxed);
        self.release_spin();
        d
    }
}

/// RAII guard returned by `RecursiveMutex::lock()`.
///
/// Releases one recursion level when dropped.
/// Implements `Deref` and `DerefMut` to give access to the protected data.
pub struct RecursiveGuard<'a, T> {
    mutex: &'a RecursiveMutex<T>,
    task_id: u8,
}

impl<'a, T> Drop for RecursiveGuard<'a, T> {
    fn drop(&mut self) {
        self.mutex.unlock(self.task_id);
    }
}

impl<'a, T> Deref for RecursiveGuard<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: we hold the mutex.
        unsafe { &*self.mutex.data.get() }
    }
}

impl<'a, T> DerefMut for RecursiveGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: we hold the mutex with exclusive ownership.
        unsafe { &mut *self.mutex.data.get() }
    }
}
