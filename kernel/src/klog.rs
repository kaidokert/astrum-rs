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

#[cfg(test)]
mod tests {
    #[test]
    fn klog_compiles() {
        klog!("test");
        klog!("fmt: {}", 42);
    }
}
