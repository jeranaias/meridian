#![no_std]
#![allow(unused_imports)]

#[cfg(feature = "std")]
extern crate std;

use meridian_types::messages::Message;
use meridian_types::time::Instant;
use core::marker::PhantomData;

// ─── no_std implementation: single-core, interrupt-safe via RTIC ───

#[cfg(not(feature = "std"))]
mod inner {
    use core::cell::UnsafeCell;
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Latest-value store. Single-writer, single/multi-reader.
    /// Safe on single-core MCUs where RTIC ensures priority-based access.
    pub struct LatestValue<T: Copy> {
        value: UnsafeCell<Option<T>>,
        has_value: AtomicBool,
    }

    unsafe impl<T: Copy + Send> Sync for LatestValue<T> {}

    impl<T: Copy> LatestValue<T> {
        pub const fn new() -> Self {
            Self {
                value: UnsafeCell::new(None),
                has_value: AtomicBool::new(false),
            }
        }

        pub fn publish(&self, val: T) {
            unsafe { *self.value.get() = Some(val); }
            self.has_value.store(true, Ordering::Release);
        }

        pub fn read(&self) -> Option<T> {
            if self.has_value.load(Ordering::Acquire) {
                unsafe { *self.value.get() }
            } else {
                None
            }
        }
    }

    /// Fixed-size ring buffer for queued (lossless) subscriptions.
    pub struct RingBuffer<T: Copy, const N: usize> {
        buf: UnsafeCell<[Option<T>; N]>,
        head: core::sync::atomic::AtomicUsize,
        tail: core::sync::atomic::AtomicUsize,
    }

    unsafe impl<T: Copy + Send, const N: usize> Sync for RingBuffer<T, N> {}

    impl<T: Copy, const N: usize> RingBuffer<T, N> {
        pub fn push(&self, val: T) -> bool {
            let head = self.head.load(core::sync::atomic::Ordering::Relaxed);
            let next = (head + 1) % N;
            let tail = self.tail.load(core::sync::atomic::Ordering::Acquire);
            if next == tail {
                return false; // full
            }
            unsafe {
                (*self.buf.get())[head] = Some(val);
            }
            self.head.store(next, core::sync::atomic::Ordering::Release);
            true
        }

        pub fn pop(&self) -> Option<T> {
            let tail = self.tail.load(core::sync::atomic::Ordering::Relaxed);
            let head = self.head.load(core::sync::atomic::Ordering::Acquire);
            if tail == head {
                return None; // empty
            }
            let val = unsafe { (*self.buf.get())[tail] };
            self.tail.store((tail + 1) % N, core::sync::atomic::Ordering::Release);
            val
        }
    }
}

// ─── std implementation: thread-safe via crossbeam ───

#[cfg(feature = "std")]
mod inner {
    use std::sync::{Arc, RwLock};
    use crossbeam_channel::{Sender, Receiver, bounded};

    /// Latest-value store backed by RwLock.
    pub struct LatestValue<T: Copy> {
        value: RwLock<Option<T>>,
    }

    impl<T: Copy> LatestValue<T> {
        pub fn new() -> Self {
            Self { value: RwLock::new(None) }
        }

        pub fn publish(&self, val: T) {
            *self.value.write().unwrap() = Some(val);
        }

        pub fn read(&self) -> Option<T> {
            *self.value.read().unwrap()
        }
    }

    /// Channel-based queue for lossless subscriptions.
    pub struct QueueChannel<T> {
        pub tx: Sender<T>,
        pub rx: Receiver<T>,
    }

    impl<T> QueueChannel<T> {
        pub fn new(capacity: usize) -> Self {
            let (tx, rx) = bounded(capacity);
            Self { tx, rx }
        }
    }
}

// ─── Public API ───

/// A publisher for a specific message type.
/// Only one publisher per topic should exist (enforced by convention).
pub struct Publisher<T: Message + Copy> {
    #[cfg(feature = "std")]
    store: std::sync::Arc<inner::LatestValue<T>>,
    #[cfg(not(feature = "std"))]
    store: &'static inner::LatestValue<T>,
}

impl<T: Message + Copy> Publisher<T> {
    pub fn publish(&self, msg: T) {
        self.store.publish(msg);
    }
}

/// A subscriber that reads the latest value (lossy — for telemetry).
pub struct Subscriber<T: Message + Copy> {
    #[cfg(feature = "std")]
    store: std::sync::Arc<inner::LatestValue<T>>,
    #[cfg(not(feature = "std"))]
    store: &'static inner::LatestValue<T>,
}

impl<T: Message + Copy> Subscriber<T> {
    /// Read the latest published value, or None if nothing published yet.
    pub fn read(&self) -> Option<T> {
        self.store.read()
    }
}

/// A queued subscriber that receives all messages (lossless — for commands).
#[cfg(feature = "std")]
pub struct QueuedSubscriber<T: Message> {
    rx: crossbeam_channel::Receiver<T>,
}

#[cfg(feature = "std")]
impl<T: Message> QueuedSubscriber<T> {
    /// Try to receive the next message without blocking.
    pub fn try_recv(&self) -> Option<T> {
        self.rx.try_recv().ok()
    }
}

/// The typed pub/sub message bus.
/// In std mode, topics are created dynamically.
/// In no_std mode, topics must be statically allocated.
#[cfg(feature = "std")]
pub struct Bus {
    // In std mode, we use a type-erased map for dynamic topic creation.
    // For Phase 1, we use a simple approach: create topics explicitly.
    _private: (),
}

#[cfg(feature = "std")]
impl Bus {
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Create a publisher/subscriber pair for a message type.
    pub fn channel<T: Message + Copy>(&self) -> (Publisher<T>, Subscriber<T>) {
        let store = std::sync::Arc::new(inner::LatestValue::new());
        (
            Publisher { store: store.clone() },
            Subscriber { store },
        )
    }

    /// Create a publisher and queued subscriber pair.
    pub fn queued_channel<T: Message + Copy>(&self, depth: usize) -> (QueuedPublisher<T>, QueuedSubscriber<T>) {
        let ch = inner::QueueChannel::new(depth);
        (
            QueuedPublisher { tx: ch.tx },
            QueuedSubscriber { rx: ch.rx },
        )
    }
}

/// Publisher for queued channels (commands).
#[cfg(feature = "std")]
pub struct QueuedPublisher<T: Message> {
    tx: crossbeam_channel::Sender<T>,
}

#[cfg(feature = "std")]
impl<T: Message> QueuedPublisher<T> {
    pub fn publish(&self, msg: T) -> Result<(), T> {
        self.tx.try_send(msg).map_err(|e| e.into_inner())
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use meridian_types::time::Instant;

    #[derive(Debug, Clone, Copy)]
    struct TestMsg {
        ts: Instant,
        value: f32,
    }

    impl Message for TestMsg {
        const TOPIC: &'static str = "test/msg";
        fn timestamp(&self) -> Instant { self.ts }
    }

    #[test]
    fn test_latest_value_channel() {
        let bus = Bus::new();
        let (pub_, sub) = bus.channel::<TestMsg>();

        // Nothing published yet
        assert!(sub.read().is_none());

        // Publish and read
        let msg = TestMsg { ts: Instant::from_micros(1000), value: 42.0 };
        pub_.publish(msg);
        let received = sub.read().unwrap();
        assert_eq!(received.value, 42.0);

        // Overwrite with newer value
        let msg2 = TestMsg { ts: Instant::from_micros(2000), value: 99.0 };
        pub_.publish(msg2);
        let received2 = sub.read().unwrap();
        assert_eq!(received2.value, 99.0);
    }

    #[test]
    fn test_queued_channel() {
        let bus = Bus::new();
        let (pub_, sub) = bus.queued_channel::<TestMsg>(16);

        // Nothing yet
        assert!(sub.try_recv().is_none());

        // Publish multiple
        pub_.publish(TestMsg { ts: Instant::from_micros(1), value: 1.0 }).unwrap();
        pub_.publish(TestMsg { ts: Instant::from_micros(2), value: 2.0 }).unwrap();
        pub_.publish(TestMsg { ts: Instant::from_micros(3), value: 3.0 }).unwrap();

        // Read in order
        assert_eq!(sub.try_recv().unwrap().value, 1.0);
        assert_eq!(sub.try_recv().unwrap().value, 2.0);
        assert_eq!(sub.try_recv().unwrap().value, 3.0);
        assert!(sub.try_recv().is_none());
    }

    #[test]
    fn test_multiple_subscribers() {
        let bus = Bus::new();
        let (pub_, sub1) = bus.channel::<TestMsg>();
        // Create a second subscriber by cloning the Arc
        let sub2 = Subscriber { store: sub1.store.clone() };

        let msg = TestMsg { ts: Instant::from_micros(100), value: 7.0 };
        pub_.publish(msg);

        assert_eq!(sub1.read().unwrap().value, 7.0);
        assert_eq!(sub2.read().unwrap().value, 7.0);
    }
}
