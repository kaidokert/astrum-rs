//! Shared user-space helper macros for issuing SVCs and reading partition
//! entry arguments.
//!
//! These macros are `#[macro_export]`-ed so that examples (and eventually
//! real partitions) can call `kernel::svc!` / `kernel::partition_trampoline!`
//! instead of duplicating inline-asm boilerplate.

/// Issue an SVC #0 system call, passing a syscall ID in `r0` and up to
/// three arguments in `r1`–`r3`.  Returns the value the kernel placed in
/// `r0` after dispatch.
///
/// On non-ARM hosts the macro is a no-op that returns `0`, which lets
/// the examples compile (but not run) for `cargo check` / `cargo clippy`.
///
/// # Examples
///
/// ```ignore
/// let rc = kernel::svc!(SYS_YIELD, 0u32, 0u32, 0u32);
/// ```
#[macro_export]
macro_rules! svc {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{
        let r: u32;
        #[cfg(target_arch = "arm")]
        // SAFETY: The inline `svc #0` instruction triggers the SVCall
        // exception whose handler (`SVC_HANDLER`) inspects and validates
        // the arguments.  The register constraints match the kernel's
        // documented SVC ABI (id in r0, args in r1-r3, r12 clobbered).
        unsafe {
            core::arch::asm!(
                "svc #0",
                inout("r0") $id => r,
                in("r1") $a,
                in("r2") $b,
                in("r3") $c,
                out("r12") _,
            )
        }
        #[cfg(not(target_arch = "arm"))]
        {
            let _ = ($id, $a, $b, $c);
            r = 0;
        }
        r
    }};
}

/// Complete kernel runtime setup macro.
///
/// Generates kernel storage, accessor functions, and PendSV handler in a
/// single invocation. This macro wraps [`define_unified_kernel!`] and
/// [`define_pendsv!`] to provide a complete, foolproof runtime setup.
///
/// # Syntax
///
/// Static mode (standard context switch):
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>);
/// ```
///
/// Static mode with SysTick handler:
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, systick);
/// ```
///
/// Dynamic mode (with MPU region programming):
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY);
/// ```
///
/// Dynamic mode with SysTick handler:
/// ```ignore
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY, systick);
/// ```
///
/// # Generated Items
///
/// | Item | Description |
/// |------|-------------|
/// | `KERNEL` static | Mutex-protected kernel storage |
/// | `store_kernel()` | Stores kernel instance and installs SVC hook |
/// | `with_kernel()` | Immutable access within critical section |
/// | `with_kernel_mut()` | Mutable access within critical section |
/// | `get_current_partition()` | C ABI accessor for PendSV |
/// | `get_next_partition()` | C ABI accessor for PendSV |
/// | `get_partition_sp()` | C ABI accessor for PendSV |
/// | `set_partition_sp()` | C ABI accessor for PendSV |
/// | `set_current_partition()` | C ABI accessor for PendSV |
/// | `PendSV` | Context-switch exception handler |
/// | `SysTick` | (with systick) Exception handler for scheduler tick |
///
/// # Example
///
/// ```ignore
/// use kernel::config::KernelConfig;
/// use kernel::svc::Kernel;
///
/// struct MyConfig;
/// impl KernelConfig for MyConfig {
///     // ... configuration constants ...
/// }
///
/// // Static mode
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>);
///
/// // Static mode with SysTick handler
/// kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, systick);
///
/// // Or dynamic mode with MPU strategy
/// // kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY);
///
/// // Dynamic mode with SysTick handler
/// // kernel::define_kernel_runtime!(KERNEL: Kernel<MyConfig>, dynamic: STRATEGY, systick);
///
/// fn main() {
///     let k = Kernel::<MyConfig>::new_empty();
///     store_kernel(k);
/// }
/// ```
#[macro_export]
macro_rules! define_kernel_runtime {
    // Unified arm: handles static/dynamic mode and optional systick
    ($name:ident : Kernel<$Config:ty> $(, dynamic: $strategy:ident)? $(, systick)?) => {
        $crate::define_unified_kernel!($name : Kernel<$Config>);
        $crate::define_kernel_runtime!(@pendsv_impl $(dynamic: $strategy)?);
        $crate::define_kernel_runtime!(@systick_impl $Config $(, systick)?);
    };

    // Internal: emit PendSV for static mode
    (@pendsv_impl) => {
        $crate::define_pendsv!();
    };
    // Internal: emit PendSV for dynamic mode
    (@pendsv_impl dynamic: $strategy:ident) => {
        $crate::define_pendsv!(dynamic: $strategy);
    };
    // Internal: no SysTick handler
    (@systick_impl $Config:ty) => {};
    // Internal: emit SysTick handler
    (@systick_impl $Config:ty, systick) => {
        $crate::define_systick!($Config);
    };
}

/// Generate a `#[naked] extern "C" fn() -> !` trampoline that tail-calls a
/// partition body function of type `extern "C" fn(u32) -> !`.
///
/// On ARM targets the trampoline is a single `b` (branch) instruction.
/// Because the function is `#[naked]`, the compiler emits no prologue and
/// `r0` is guaranteed to still hold the value the kernel placed there at
/// partition entry.  This eliminates any compiler-reordering hazard that
/// would exist in a non-naked function.
///
/// On non-ARM test hosts the trampoline calls `$body(0)`.
///
/// # Usage
///
/// ```ignore
/// // (`ignore` — no_std crate cannot compile doc tests due to panic_halt;
/// //  see mod tests::partition_trampoline_has_correct_fn_type for host test.)
/// extern "C" fn my_partition_body(r0: u32) -> ! {
///     let _port_id = r0;
///     loop {}
/// }
/// kernel::partition_trampoline!(my_partition_entry => my_partition_body);
///
/// // The generated `my_partition_entry` is `extern "C" fn() -> !`
/// // and can be passed directly to boot():
/// let _: extern "C" fn() -> ! = my_partition_entry;
/// ```
#[macro_export]
macro_rules! partition_trampoline {
    ($name:ident => $body:path) => {
        #[cfg(target_arch = "arm")]
        // TODO: migrate to #[unsafe(no_mangle)] when moving to edition 2024;
        // currently edition 2021 where only #[naked] requires unsafe().
        #[no_mangle]
        #[unsafe(naked)]
        pub extern "C" fn $name() -> ! {
            // SAFETY: naked function — no prologue, so r0 is untouched.
            // The `b` instruction tail-calls $body which expects r0 as its
            // first argument per AAPCS.
            unsafe { core::arch::naked_asm!("b {0}", sym $body) }
        }

        #[cfg(not(target_arch = "arm"))]
        // TODO: migrate to #[unsafe(no_mangle)] when moving to edition 2024.
        #[no_mangle]
        pub extern "C" fn $name() -> ! {
            // On host targets, call the body with 0.
            let body: extern "C" fn(u32) -> ! = $body;
            body(0)
        }
    };
}

/// Generate a custom interrupt vector table and dispatch handler.
///
/// This macro binds hardware IRQ numbers to (partition, event_bits) pairs and
/// generates the static IVT required by cortex-m-rt 0.7.
///
/// # Syntax
///
/// ```ignore
/// kernel::bind_interrupts!(MyConfig, 70,
///     5  => (0, 0x01),  // IRQ 5  → partition 0, event 0x01
///     23 => (1, 0x04),  // IRQ 23 → partition 1, event 0x04
/// );
/// ```
///
/// # Generated Items
///
/// | Item | Description |
/// |------|-------------|
/// | `__IRQ_BINDINGS` | `static` array of `IrqBinding` |
/// | `__irq_dispatch` | Dispatch handler reading IPSR |
/// | `__INTERRUPTS` | IVT in `.vector_table.interrupts` |
#[macro_export]
macro_rules! bind_interrupts {
    ($Config:ty, $count:expr, $( $irq:expr => ($pid:expr, $evt:expr) ),+ $(,)?) => {
        // ---- compile-time validation (runs on all targets) ----
        const _: () = {
            const BINDINGS: [$crate::irq_dispatch::IrqBinding; 0 $( + { let _ = $irq; 1 } )+] = [
                $( $crate::irq_dispatch::IrqBinding::new($irq, $pid, $evt), )+
            ];
            assert!(
                !$crate::irq_dispatch::has_duplicate_irqs(&BINDINGS),
                "bind_interrupts!: duplicate IRQ number"
            );
            $(
                assert!(
                    ($irq as usize) < ($count as usize),
                    "bind_interrupts!: IRQ number >= count"
                );
            )+
        };

        // ---- feature gate (ARM only) ----
        #[cfg(all(not(test), target_arch = "arm", not(feature = "custom-ivt")))]
        compile_error!(
            "bind_interrupts! requires the `custom-ivt` feature; \
             add `features = [\"custom-ivt\"]` to your Cargo.toml"
        );

        // ---- binding table (ARM only) ----
        // TODO: binding list is repeated in validation and here; acceptable for a declarative macro
        #[cfg(all(not(test), target_arch = "arm"))]
        static __IRQ_BINDINGS: [$crate::irq_dispatch::IrqBinding; 0 $( + { let _ = $irq; 1 } )+] = [
            $( $crate::irq_dispatch::IrqBinding::new($irq, $pid, $evt), )+
        ];

        // ---- dispatch handler (ARM only) ----
        #[cfg(all(not(test), target_arch = "arm"))]
        extern "C" fn __irq_dispatch() {
            let ipsr: u32;
            // SAFETY: `mrs` reads the IPSR register, which is always
            // readable in any execution mode on Cortex-M.  The register
            // is read-only and has no side effects.
            unsafe { core::arch::asm!("mrs {}, ipsr", out(reg) ipsr) };
            let irq_num = (ipsr & 0x1FF).wrapping_sub(16) as u8;
            if let Some(idx) = $crate::irq_dispatch::lookup_binding(&__IRQ_BINDINGS, irq_num) {
                if let Some(b) = __IRQ_BINDINGS.get(idx) {
                    $crate::irq_dispatch::signal_partition_from_isr::<$Config>(
                        b.partition_id,
                        b.event_bits,
                    );
                }
            }
        }

        // ---- IVT static (ARM only) ----
        #[cfg(all(not(test), target_arch = "arm"))]
        #[link_section = ".vector_table.interrupts"]
        #[no_mangle]
        pub static __INTERRUPTS: [unsafe extern "C" fn(); $count] = {
            const HANDLER: unsafe extern "C" fn() =
                __irq_dispatch as unsafe extern "C" fn();
            [HANDLER; $count]
        };
    };
}

/// Generate a SysTick exception handler that advances the scheduler.
///
/// This macro emits a `SysTick` exception handler that calls
/// [`tick::systick_handler`](crate::tick::systick_handler) within a critical
/// section to advance the scheduler and handle partition switches.
///
/// # Usage
///
/// Typically invoked automatically by [`define_kernel_runtime!`] with the
/// `systick` option. Can also be used standalone:
///
/// ```ignore
/// kernel::define_systick!(MyConfig);
/// ```
///
/// # Generated Items
///
/// | Item | Description |
/// |------|-------------|
/// | `SysTick` | Exception handler that calls `systick_handler` |
#[macro_export]
macro_rules! define_systick {
    ($Config:ty) => {
        #[cfg(target_arch = "arm")]
        #[cortex_m_rt::exception]
        fn SysTick() {
            $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                $crate::tick::systick_handler::<$Config>(k);
            });
        }
    };
}

#[cfg(test)]
mod tests {
    // On non-ARM test hosts the svc! macro must compile and return 0.

    #[test]
    fn svc_returns_zero_on_host() {
        let r = svc!(1u32, 2u32, 3u32, 4u32);
        assert_eq!(r, 0);
    }

    #[allow(clippy::empty_loop)]
    extern "C" fn test_body(_r0: u32) -> ! {
        loop {}
    }

    partition_trampoline!(test_trampoline => test_body);

    #[test]
    fn partition_trampoline_has_correct_fn_type() {
        // The trampoline must be `extern "C" fn() -> !`, compatible with
        // boot() which expects that signature for partition entry points.
        let _: extern "C" fn() -> ! = test_trampoline;
    }

    #[test]
    fn partition_trampoline_is_not_null() {
        let ptr = test_trampoline as *const ();
        assert!(!ptr.is_null());
    }

    // Tests for define_kernel_runtime! macro require ARM target since the
    // macro generates code using cortex_m::interrupt::free and other ARM
    // intrinsics. The QEMU integration tests verify the actual functionality.
    // See svc.rs::unified_kernel_macro_tests for the gating pattern.

    // ---- bind_interrupts! compile-time validation tests ----
    // The const assertion block runs on all targets (including host).
    // ARM-specific codegen (IVT, dispatch handler) is cfg-gated out on host.

    // DefaultConfig is passed to bind_interrupts! as the Config type parameter.
    // On non-ARM hosts the ARM-specific codegen is cfg-gated out, so the type
    // is only consumed by the const validation block (which doesn't reference it).
    #[allow(unused_imports)]
    use crate::config::DefaultConfig;

    // Invoke the macro with valid bindings — the const assertion block
    // succeeds, and the ARM-only items are cfg-gated out on host.
    bind_interrupts!(DefaultConfig, 70,
        5  => (0, 0x01),
        23 => (1, 0x04),
    );

    #[test]
    fn bind_interrupts_const_validation_passes() {
        // If we reach here, the const assertion block in bind_interrupts!
        // accepted the bindings (no duplicates, all IRQs < 70).
    }

    // Verify the const validation also works with a single binding.
    bind_interrupts!(DefaultConfig, 10,
        9 => (0, 0xFF),
    );

    #[test]
    fn bind_interrupts_single_binding_accepted() {
        // Single binding: IRQ 9 < 10, no duplicates — const assertion passes.
    }
}
