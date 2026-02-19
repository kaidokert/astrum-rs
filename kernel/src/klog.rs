//! Kernel logging macro with compile-time backend selection.
//!
//! Silent by default. Enable exactly one log feature for output.
//! Backend selection is handled by build.rs which emits `klog_backend`.

#[doc(hidden)]
#[macro_export]
#[cfg(klog_backend = "semihosting")]
macro_rules! __klog_impl {
    ($($arg:tt)*) => { cortex_m_semihosting::hprintln!($($arg)*); };
}

#[doc(hidden)]
#[macro_export]
#[cfg(klog_backend = "rtt")]
macro_rules! __klog_impl {
    ($($arg:tt)*) => { rtt_target::rprintln!($($arg)*); };
}

// TODO: The SWO implementation uses cortex_m::interrupt::free around write_fmt.
// format_args! and formatting machinery can introduce non-deterministic overhead
// inside the critical section, increasing worst-case interrupt latency. Consider
// buffered/deferred logging for high-frequency scenarios.
#[doc(hidden)]
#[macro_export]
#[cfg(klog_backend = "swo")]
macro_rules! __klog_impl {
    ($($arg:tt)*) => {{
        cortex_m::interrupt::free(|_| {
            // SAFETY: Critical section ensures exclusive ITM access.
            let stim = unsafe { &mut (*cortex_m::peripheral::ITM::PTR).stim[0] };
            cortex_m::itm::write_fmt(stim, format_args!($($arg)*));
        });
    }};
}

#[doc(hidden)]
#[macro_export]
#[cfg(klog_backend = "defmt")]
macro_rules! __klog_impl {
    ($($arg:tt)*) => { defmt::info!($($arg)*); };
}

#[doc(hidden)]
#[macro_export]
#[cfg(klog_backend = "none")]
macro_rules! __klog_impl {
    ($($arg:tt)*) => {{}};
}

/// Kernel logging macro. Silent when no log feature enabled.
#[macro_export]
macro_rules! klog {
    ($($arg:tt)*) => { $crate::__klog_impl!($($arg)*) };
}

/// Exit macro for test termination abstraction.
///
/// On semihosting backend, calls `cortex_m_semihosting::debug::exit`.
/// On RTT backend, prints `TEST PASSED`/`TEST FAILED` then halts via `bkpt`.
/// On other backends, enters an infinite loop.
///
/// # Usage
/// ```ignore
/// kexit!(success);  // Exit with success status
/// kexit!(failure);  // Exit with failure status
/// ```
#[doc(hidden)]
#[macro_export]
#[cfg(klog_backend = "semihosting")]
macro_rules! __kexit_impl {
    (success) => {
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_SUCCESS)
    };
    (failure) => {
        cortex_m_semihosting::debug::exit(cortex_m_semihosting::debug::EXIT_FAILURE)
    };
}

#[doc(hidden)]
#[macro_export]
#[cfg(klog_backend = "rtt")]
macro_rules! __kexit_impl {
    (success) => {{
        rtt_target::rprintln!("TEST PASSED");
        cortex_m::asm::bkpt();
    }};
    (failure) => {{
        rtt_target::rprintln!("TEST FAILED");
        cortex_m::asm::bkpt();
    }};
}

#[doc(hidden)]
#[macro_export]
#[cfg(not(any(klog_backend = "semihosting", klog_backend = "rtt")))]
macro_rules! __kexit_impl {
    ($_status:ident) => {
        loop {
            cortex_m::asm::wfi();
        }
    };
}

/// Exit macro for test termination.
///
/// On semihosting backend, terminates QEMU with the specified status.
/// On RTT backend, prints a test marker and halts via `bkpt`.
/// On other backends, enters an infinite loop (WFI).
///
/// # Arguments
/// * `success` - Exit indicating successful execution
/// * `failure` - Exit indicating failed execution
#[macro_export]
macro_rules! kexit {
    (success) => {
        $crate::__kexit_impl!(success)
    };
    (failure) => {
        $crate::__kexit_impl!(failure)
    };
}

#[cfg(test)]
mod tests {
    #[test]
    fn klog_compiles() {
        klog!("test");
        klog!("fmt: {}", 42);
    }

    /// Verify kexit macro expansion compiles for both variants.
    /// On test (host) builds, klog_backend is "none", so kexit expands to WFI loops
    /// (the catch-all gate excludes both semihosting and rtt backends).
    /// We can't actually call the macro since it would hang, but we verify it parses.
    #[test]
    fn kexit_macro_expansion_compiles() {
        // Verify macro syntax is valid by checking it can be parsed in a function body.
        // We use a never-executed branch to avoid actually entering the infinite loop.
        fn _check_success_syntax() {
            if false {
                kexit!(success);
            }
        }
        fn _check_failure_syntax() {
            if false {
                kexit!(failure);
            }
        }
        // If we get here, macro expansion compiled successfully.
    }

    /// Verify all three __kexit_impl cfg gates are mutually exclusive.
    /// Exactly one variant should be active for any valid klog_backend value.
    #[test]
    fn kexit_cfg_gates_are_mutually_exclusive() {
        // Count how many __kexit_impl variants are active at compile time.
        // On host builds with klog_backend="none", only the catch-all should be active.
        let mut active_count = 0u32;

        #[cfg(klog_backend = "semihosting")]
        {
            active_count += 1;
        }
        #[cfg(klog_backend = "rtt")]
        {
            active_count += 1;
        }
        #[cfg(not(any(klog_backend = "semihosting", klog_backend = "rtt")))]
        {
            active_count += 1;
        }

        assert_eq!(
            active_count, 1,
            "exactly one __kexit_impl variant must be active"
        );
    }
}
