//! Compatibility shim for loom concurrency testing.
//!
//! When compiled with `--cfg loom`, this module re-exports primitives from
//! the `loom` crate. Otherwise it provides thin wrappers around `std`
//! equivalents so that test code can be written once and run under both
//! standard `cargo test` and `RUSTFLAGS="--cfg loom" cargo test`.

// ---------------------------------------------------------------------------
// Mutex
// ---------------------------------------------------------------------------

// Loom's Mutex::lock() returns MutexGuard directly (infallible), while std's
// returns Result<MutexGuard, PoisonError>.  We normalise to the infallible
// signature so call-sites compile under both configurations.

#[cfg(loom)]
pub use loom::sync::Mutex;

#[cfg(not(loom))]
pub use self::std_mutex::Mutex;

#[cfg(not(loom))]
mod std_mutex {
    use std::sync::{Mutex as Inner, MutexGuard};

    pub struct Mutex<T>(Inner<T>);

    impl<T> Mutex<T> {
        pub fn new(val: T) -> Self {
            Self(Inner::new(val))
        }

        pub fn lock(&self) -> MutexGuard<'_, T> {
            self.0.lock().expect("mutex poisoned")
        }
    }
}

// ---------------------------------------------------------------------------
// Arc
// ---------------------------------------------------------------------------

#[cfg(loom)]
pub use loom::sync::Arc;

#[cfg(not(loom))]
pub use std::sync::Arc;

// ---------------------------------------------------------------------------
// Atomics
// ---------------------------------------------------------------------------

pub mod atomic {
    #[cfg(loom)]
    pub use loom::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    #[cfg(not(loom))]
    pub use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
}

// ---------------------------------------------------------------------------
// Thread
// ---------------------------------------------------------------------------

pub mod thread {
    #[cfg(loom)]
    pub use loom::thread::spawn;

    #[cfg(not(loom))]
    pub use std::thread::spawn;
}

// ---------------------------------------------------------------------------
// Model runner
// ---------------------------------------------------------------------------

/// Run a closure under the loom model checker (loom build).
#[cfg(loom)]
pub fn model<F: Fn() + Sync + Send + 'static>(f: F) {
    loom::model(f);
}

/// Run a closure directly (non-loom build).
#[cfg(not(loom))]
pub fn model<F: Fn() + Sync + Send + 'static>(f: F) {
    f();
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mutex_lock_unlock() {
        model(|| {
            let m = Mutex::new(0u32);
            let mut guard = m.lock();
            *guard += 1;
            assert_eq!(*guard, 1);
        });
    }

    #[test]
    fn thread_spawn_join() {
        model(|| {
            let m = Arc::new(Mutex::new(0u32));
            let m2 = m.clone();
            let handle = thread::spawn(move || {
                let mut guard = m2.lock();
                *guard += 1;
            });
            handle.join().unwrap();
            assert_eq!(*m.lock(), 1);
        });
    }

    #[test]
    fn model_runner_executes() {
        let executed = Arc::new(atomic::AtomicBool::new(false));
        let flag = executed.clone();
        model(move || {
            flag.store(true, atomic::Ordering::SeqCst);
        });
        assert!(executed.load(atomic::Ordering::SeqCst));
    }
}
