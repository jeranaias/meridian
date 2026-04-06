// Integration tests for RecursiveMutex
//
// These tests run under std (host) to verify the correctness of the prototype.
// The implementation itself is no_std; only the test harness needs std.

use meridian_sync::{RecursiveMutex, NO_OWNER, TASK_ID_MAIN, TASK_ID_SPI_BUS};

// ─────────────────────────────────────────────────────────────────────────────
// Basic locking

#[test]
fn test_single_lock_and_unlock() {
    static M: RecursiveMutex<u32> = RecursiveMutex::new(0);
    {
        let mut g = M.lock(TASK_ID_MAIN);
        assert_eq!(M.recursion_depth(), 1);
        *g = 42;
    }
    // Guard dropped — depth should be 0 again.
    assert_eq!(M.recursion_depth(), 0);
    assert!(!M.is_owner(TASK_ID_MAIN));
}

// ─────────────────────────────────────────────────────────────────────────────
// Re-entry from the same task

#[test]
fn test_recursive_reentry_two_levels() {
    static M: RecursiveMutex<u32> = RecursiveMutex::new(0);

    let _g1 = M.lock(TASK_ID_MAIN);
    assert_eq!(M.recursion_depth(), 1);
    assert!(M.is_owner(TASK_ID_MAIN));

    let _g2 = M.lock(TASK_ID_MAIN);   // recursive — must NOT deadlock
    assert_eq!(M.recursion_depth(), 2);

    drop(_g2);
    assert_eq!(M.recursion_depth(), 1);  // still locked by task 0

    drop(_g1);
    assert_eq!(M.recursion_depth(), 0);  // now free
}

#[test]
fn test_recursive_reentry_three_levels() {
    static M: RecursiveMutex<u32> = RecursiveMutex::new(0);
    // Simulates the ArduPilot pattern:
    //   init() → WITH_SEMAPHORE(bus_sem)
    //     → hardware_init() → WITH_SEMAPHORE(bus_sem)   (level 2)
    //       → _configure_slaves() → WITH_SEMAPHORE(backend._dev->get_semaphore())  (level 3)

    let _g1 = M.lock(TASK_ID_MAIN);
    let _g2 = M.lock(TASK_ID_MAIN);
    let _g3 = M.lock(TASK_ID_MAIN);
    assert_eq!(M.recursion_depth(), 3);

    drop(_g3);
    assert_eq!(M.recursion_depth(), 2);
    drop(_g2);
    assert_eq!(M.recursion_depth(), 1);
    drop(_g1);
    assert_eq!(M.recursion_depth(), 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Data mutation through nested guards

#[test]
fn test_data_visible_through_nested_guards() {
    static M: RecursiveMutex<[u8; 4]> = RecursiveMutex::new([0u8; 4]);

    let mut g1 = M.lock(TASK_ID_MAIN);
    g1[0] = 1;

    {
        let mut g2 = M.lock(TASK_ID_MAIN);
        g2[1] = 2;   // same data — should be visible after g2 drops
    }

    assert_eq!(g1[0], 1);
    assert_eq!(g1[1], 2);
}

// ─────────────────────────────────────────────────────────────────────────────
// is_owner()

#[test]
fn test_is_owner() {
    static M: RecursiveMutex<()> = RecursiveMutex::new(());

    assert!(!M.is_owner(TASK_ID_MAIN));
    assert!(!M.is_owner(TASK_ID_SPI_BUS));

    let _g = M.lock(TASK_ID_MAIN);
    assert!(M.is_owner(TASK_ID_MAIN));
    assert!(!M.is_owner(TASK_ID_SPI_BUS));

    drop(_g);
    assert!(!M.is_owner(TASK_ID_MAIN));
}

// ─────────────────────────────────────────────────────────────────────────────
// A different "task" blocks until first task releases
//
// We simulate two RTIC tasks using std threads.  Task 0 takes the lock,
// task 1 starts and spins (blocked).  When task 0 releases, task 1 should
// immediately acquire the lock.

#[test]
fn test_different_task_blocks_then_acquires() {
    use std::sync::{Arc, Barrier};
    use std::thread;

    // Note: in real RTIC the blocking spin is cooperative with the scheduler.
    // Here we verify the correctness of the owner-transfer protocol.

    static M: RecursiveMutex<u32> = RecursiveMutex::new(0);

    let barrier = Arc::new(Barrier::new(2));
    let barrier2 = barrier.clone();

    // Task 0 (TASK_ID_MAIN): lock, signal, hold for a moment, then release.
    let h = thread::spawn(move || {
        let mut g = M.lock(TASK_ID_MAIN);
        *g = 10;
        barrier2.wait();   // signal task 1 that we hold the lock
        // simulate some work
        for _ in 0..1_000 {
            core::hint::spin_loop();
        }
        *g = 20;
        drop(g);           // release
    });

    // Main thread acts as "task 1" (TASK_ID_SPI_BUS).
    barrier.wait();        // wait until task 0 holds the lock

    // At this point task 0 is the owner.  Our lock() call will spin.
    let g = M.lock(TASK_ID_SPI_BUS);
    // If we get here, task 0 must have released.
    assert_eq!(*g, 20, "expected task 0's final write to be visible");
    drop(g);

    h.join().unwrap();
}

// ─────────────────────────────────────────────────────────────────────────────
// Panic on exceeding MAX_RECURSION is tested via should_panic.
//
// We use a heap-allocated mutex here (not a static) so that the panic
// from this test does not leave a static in a broken state that would
// cause other tests to spin-loop waiting for the lock to free.

#[test]
#[should_panic(expected = "MAX_RECURSION")]
fn test_max_recursion_panics() {
    // Box keeps the mutex alive for the lifetime of all guards.
    let m: Box<RecursiveMutex<()>> = Box::new(RecursiveMutex::new(()));

    // We use raw pointers to hold guards while keeping the Box alive.
    // SAFETY: the Box outlives all guards since we don't drop it until after.
    let m_ref: &'static RecursiveMutex<()> = unsafe { &*(m.as_ref() as *const _) };

    let _g1 = m_ref.lock(TASK_ID_MAIN);
    let _g2 = m_ref.lock(TASK_ID_MAIN);
    let _g3 = m_ref.lock(TASK_ID_MAIN);
    let _g4 = m_ref.lock(TASK_ID_MAIN);
    let _g5 = m_ref.lock(TASK_ID_MAIN);
    let _g6 = m_ref.lock(TASK_ID_MAIN);
    let _g7 = m_ref.lock(TASK_ID_MAIN);
    let _g8 = m_ref.lock(TASK_ID_MAIN);
    let _g9 = m_ref.lock(TASK_ID_MAIN);  // should panic here
    drop(m);
}

// ─────────────────────────────────────────────────────────────────────────────
// Panic on NO_OWNER task_id

#[test]
#[should_panic(expected = "NO_OWNER")]
fn test_no_owner_task_id_panics() {
    let m: Box<RecursiveMutex<()>> = Box::new(RecursiveMutex::new(()));
    let m_ref: &'static RecursiveMutex<()> = unsafe { &*(m.as_ref() as *const _) };
    let _g = m_ref.lock(NO_OWNER);
    drop(m);
}

// ─────────────────────────────────────────────────────────────────────────────
// Simulate the ArduPilot SPI bus pattern:
//   driver_init() takes bus semaphore
//     calls hardware_init() which also takes bus semaphore (re-entry)
//       hardware_init() calls configure_slaves() which takes it a third time

#[test]
fn test_ardupilot_spi_init_pattern() {
    static BUS_SEM: RecursiveMutex<u32> = RecursiveMutex::new(0);

    fn configure_slaves(task: u8) {
        let mut g = BUS_SEM.lock(task);  // level 3
        assert_eq!(BUS_SEM.recursion_depth(), 3);
        *g += 1;
    }

    fn hardware_init(task: u8) {
        let mut g = BUS_SEM.lock(task);  // level 2
        assert_eq!(BUS_SEM.recursion_depth(), 2);
        *g += 10;
        configure_slaves(task);
        // recursion_depth is 2 again after configure_slaves returns
        assert_eq!(BUS_SEM.recursion_depth(), 2);
    }

    fn driver_init(task: u8) {
        let _g = BUS_SEM.lock(task);  // level 1
        assert_eq!(BUS_SEM.recursion_depth(), 1);
        hardware_init(task);
        assert_eq!(BUS_SEM.recursion_depth(), 1);
    }

    driver_init(TASK_ID_MAIN);

    assert_eq!(BUS_SEM.recursion_depth(), 0);
    // Final value should be 11 (hardware_init added 10, configure_slaves added 1)
    let g = BUS_SEM.lock(TASK_ID_MAIN);
    assert_eq!(*g, 11);
}
