//! Shared user-space helper macros for issuing SVCs and reading partition
//! entry arguments.
//!
//! These macros are `#[macro_export]`-ed so that examples (and eventually
//! real partitions) can call `kernel::svc!` / `kernel::partition_trampoline!`
//! instead of duplicating inline-asm boilerplate.

/// Compile-time assertion. Fails the build if `$cond` evaluates to false.
///
/// Use this instead of `const { assert!(...) }` when you need a standalone
/// item-position assertion outside an existing `const` block.
///
/// ```ignore
/// kernel::const_assert!(core::mem::size_of::<u32>() == 4);
/// kernel::const_assert!(2 + 2 == 4, "math is broken");
/// ```
#[macro_export]
macro_rules! const_assert {
    ($cond:expr $(,)?) => {
        const _: () = assert!($cond);
    };
    ($cond:expr, $msg:expr $(,)?) => {
        const _: () = assert!($cond, $msg);
    };
}

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
        {
            let id: u32 = $id;
            let a: u32 = $a;
            let b: u32 = $b;
            let c: u32 = $c;
            // SAFETY: The inline `svc #0` instruction triggers the SVCall
            // exception whose handler (`SVC_HANDLER`) inspects and validates
            // the arguments.  The register constraints match the kernel's
            // documented SVC ABI (id in r0, args in r1-r3, r12 clobbered).
            unsafe {
                core::arch::asm!(
                    "svc #0",
                    inout("r0") id => r,
                    in("r1") a,
                    in("r2") b,
                    in("r3") c,
                    out("r12") _,
                )
            }
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
///     // Build schedule and partition memory descriptors, then:
///     // let k = Kernel::<MyConfig>::new(schedule, &memories);
///     // store_kernel(k); // k is valid if new() returned Ok
/// }
/// ```
#[macro_export]
macro_rules! define_kernel_runtime {
    // Unified arm: handles static/dynamic mode and optional systick
    ($name:ident : Kernel<$Config:ty> $(, dynamic: $strategy:ident)? $(, systick)?) => {
        $crate::define_unified_kernel!($name : Kernel<$Config>);
        $crate::define_kernel_runtime!(@pendsv_impl $Config $(, dynamic: $strategy)?);
        $crate::define_kernel_runtime!(@systick_impl $Config $(, systick)?);
    };

    // Internal: emit PendSV for static mode
    (@pendsv_impl $Config:ty) => {
        $crate::define_pendsv!($Config);
    };
    // Internal: emit PendSV for dynamic mode
    (@pendsv_impl $Config:ty, dynamic: $strategy:ident) => {
        $crate::define_pendsv!(dynamic: $strategy, $Config);
    };
    // Internal: no SysTick handler
    (@systick_impl $Config:ty) => {};
    // Internal: emit SysTick handler
    (@systick_impl $Config:ty, systick) => {
        $crate::define_systick!($Config);
    };
}

/// Issue an SVC #0 system call, passing a syscall ID in `r0` and up to
/// three arguments in `r1`–`r3`.  Returns `(r0, r1)` — both values the
/// kernel placed in those registers after dispatch.
///
/// On non-ARM hosts the macro is a no-op that returns `(0, 0)`.
///
/// # Examples
///
/// ```ignore
/// let (r0, r1) = kernel::svc_r01!(SYS_BUF_LEND, slot, r2, 0u32);
/// ```
#[macro_export]
macro_rules! svc_r01 {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{
        let r0: u32;
        let r1: u32;
        #[cfg(target_arch = "arm")]
        {
            let id: u32 = $id;
            let a: u32 = $a;
            let b: u32 = $b;
            let c: u32 = $c;
            // SAFETY: The inline `svc #0` instruction triggers the SVCall
            // exception whose handler inspects and validates the arguments.
            // The register constraints match the kernel's SVC ABI (id in r0,
            // args in r1-r3, result in r0+r1, r12 clobbered).
            unsafe {
                core::arch::asm!(
                    "svc #0",
                    inout("r0") id => r0,
                    inout("r1") a => r1,
                    in("r2") b,
                    in("r3") c,
                    out("r12") _,
                )
            }
        }
        #[cfg(not(target_arch = "arm"))]
        {
            let _ = ($id, $a, $b, $c);
            r0 = 0;
            r1 = 0;
        }
        (r0, r1)
    }};
}

/// Generate a `#[naked]` [`PartitionEntry`] trampoline that tail-calls a
/// partition body function of type [`PartitionBody`].
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
/// // (`ignore` — no_std crate cannot compile doc tests without a std harness;
/// //  see mod tests::partition_trampoline_has_correct_fn_type for host test.)
/// use kernel::PartitionEntry;
///
/// extern "C" fn my_partition_body(r0: u32) -> ! {
///     let _port_id = r0;
///     loop {}
/// }
/// kernel::partition_trampoline!(my_partition_entry => my_partition_body);
///
/// // The generated `my_partition_entry` has type `PartitionEntry`
/// // and can be passed directly to boot():
/// let _: PartitionEntry = my_partition_entry;
/// ```
#[macro_export]
macro_rules! partition_trampoline {
    ($name:ident => $body:path) => {
        // Compile-time check: on ARM the const assert is the only type gate;
        // on non-ARM the `let` binding in the host shim already validates.
        #[cfg(target_arch = "arm")]
        const _: $crate::partition::PartitionBody = $body;

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
            // TODO: the `let` type-check triggers a misleading "try calling
            // the function" hint from rustc on signature mismatch; consider a
            // const-assert wrapper that suppresses it once a clean approach
            // is found.
            let body: $crate::partition::PartitionBody = $body;
            body(0)
        }

        // Compile-time check: the generated trampoline must be PartitionEntry.
        const _: $crate::partition::PartitionEntry = $name;
    };
}

/// Construct an [`IrqBinding`] from either a 2-tuple `(pid, evt)` or a
/// 3-tuple `(pid, evt, clear_model)`.
///
/// This is a `#[doc(hidden)]` helper for [`bind_interrupts!`]; it should not
/// be invoked directly.
#[macro_export]
#[doc(hidden)]
macro_rules! __make_irq_binding {
    ($irq:expr, ($pid:expr, $evt:expr)) => {
        $crate::irq_dispatch::IrqBinding::new($irq, $crate::PartitionId::new($pid as u32), $evt)
    };
    ($irq:expr, ($pid:expr, $evt:expr, handler: $handler:path)) => {
        $crate::irq_dispatch::IrqBinding::new($irq, $crate::PartitionId::new($pid as u32), $evt)
    };
    ($irq:expr, ($pid:expr, $evt:expr, never_mask)) => {
        $crate::irq_dispatch::IrqBinding::with_clear_model(
            $irq,
            $crate::PartitionId::new($pid as u32),
            $evt,
            $crate::irq_dispatch::IrqClearModel::NeverMask,
        )
    };
    ($irq:expr, ($pid:expr, $evt:expr, never_mask, handler: $handler:path)) => {
        $crate::irq_dispatch::IrqBinding::with_clear_model(
            $irq,
            $crate::PartitionId::new($pid as u32),
            $evt,
            $crate::irq_dispatch::IrqClearModel::NeverMask,
        )
    };
    ($irq:expr, ($pid:expr, $evt:expr, $clear:expr)) => {
        $crate::irq_dispatch::IrqBinding::with_clear_model(
            $irq,
            $crate::PartitionId::new($pid as u32),
            $evt,
            $clear,
        )
    };
    ($irq:expr, ($pid:expr, $evt:expr, clear: WriteRegister($addr:expr, $value:expr))) => {
        $crate::irq_dispatch::IrqBinding::with_clear_model(
            $irq,
            $crate::PartitionId::new($pid as u32),
            $evt,
            $crate::irq_dispatch::IrqClearModel::KernelClears(
                $crate::irq_dispatch::ClearStrategy::WriteRegister {
                    addr: $addr,
                    value: $value,
                },
            ),
        )
    };
    ($irq:expr, ($pid:expr, $evt:expr, clear: ClearBit($addr:expr, $bit:expr))) => {
        $crate::irq_dispatch::IrqBinding::with_clear_model(
            $irq,
            $crate::PartitionId::new($pid as u32),
            $evt,
            $crate::irq_dispatch::IrqClearModel::KernelClears(
                $crate::irq_dispatch::ClearStrategy::ClearBit {
                    addr: $addr,
                    bit: $bit,
                },
            ),
        )
    };
}

/// Emit a `const` reference for `handler:` bindings to suppress `dead_code`
/// on targets where `__INTERRUPTS` is not emitted (non-ARM / test hosts).
///
/// This is a `#[doc(hidden)]` helper for [`bind_interrupts!`]; it should not
/// be invoked directly.
#[macro_export]
#[doc(hidden)]
macro_rules! __ref_handler_if_custom {
    // handler: form – emit a const reference to keep the function alive.
    (($pid:expr, $evt:expr, handler: $handler:path)) => {
        const _: $crate::IsrHandler = $handler;
    };
    // 2-tuple: no custom handler.
    (($pid:expr, $evt:expr)) => {};
    // never_mask keyword form: no custom handler.
    (($pid:expr, $evt:expr, never_mask)) => {};
    // never_mask with handler: form – emit a const reference.
    (($pid:expr, $evt:expr, never_mask, handler: $handler:path)) => {
        const _: $crate::IsrHandler = $handler;
    };
    // 3-tuple with explicit clear model: no custom handler.
    (($pid:expr, $evt:expr, $clear:expr)) => {};
    // clear: WriteRegister keyword form: no custom handler.
    (($pid:expr, $evt:expr, clear: WriteRegister($addr:expr, $value:expr))) => {};
    // clear: ClearBit keyword form: no custom handler.
    (($pid:expr, $evt:expr, clear: ClearBit($addr:expr, $bit:expr))) => {};
}

/// Select the IVT handler for a binding: the default dispatch handler for
/// standard forms, or the user-supplied function for `handler:` forms.
///
/// This is a `#[doc(hidden)]` helper for [`bind_interrupts!`]; it should not
/// be invoked directly.
#[macro_export]
#[doc(hidden)]
macro_rules! __make_irq_handler {
    // 2-tuple: standard dispatch
    (($pid:expr, $evt:expr), $default:path) => {
        $default as $crate::IsrHandler
    };
    // handler: form – custom ISR
    (($pid:expr, $evt:expr, handler: $handler:path), $default:path) => {
        $handler as $crate::IsrHandler
    };
    // never_mask keyword form: standard dispatch
    (($pid:expr, $evt:expr, never_mask), $default:path) => {
        $default as $crate::IsrHandler
    };
    // never_mask with handler: form – custom ISR
    (($pid:expr, $evt:expr, never_mask, handler: $handler:path), $default:path) => {
        $handler as $crate::IsrHandler
    };
    // 3-tuple with explicit clear model: standard dispatch
    (($pid:expr, $evt:expr, $clear:expr), $default:path) => {
        $default as $crate::IsrHandler
    };
    // clear: WriteRegister keyword form: standard dispatch
    (($pid:expr, $evt:expr, clear: WriteRegister($addr:expr, $value:expr)), $default:path) => {
        $default as $crate::IsrHandler
    };
    // clear: ClearBit keyword form: standard dispatch
    (($pid:expr, $evt:expr, clear: ClearBit($addr:expr, $bit:expr)), $default:path) => {
        $default as $crate::IsrHandler
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
///     5  => (0, 0x01),                     // 2-tuple: PartitionAcks default
///     23 => (1, 0x04, IrqClearModel::KernelClears(
///         ClearStrategy::WriteRegister { addr: 0x100, value: 1 },
///     )),                                  // 3-tuple: explicit clear model
///     30 => (0, 0x02, never_mask),         // edge-triggered: no mask/clear
///     31 => (0, 0x04, never_mask, handler: my_custom_isr),  // never_mask + custom ISR
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
/// | `enable_bound_irqs` | Set priority and unmask all bound IRQs |
/// | `disable_bound_irqs` | Mask all bound IRQs |
#[macro_export]
macro_rules! bind_interrupts {
    ($Config:ty, $count:expr, $( $irq:expr => $args:tt ),+ $(,)?) => {
        const __IRQ_BINDINGS_INIT: [$crate::irq_dispatch::IrqBinding; 0 $( + { let _ = $irq; 1 } )+] = [$( $crate::__make_irq_binding!($irq, $args), )+];
        // ---- compile-time validation (runs on all targets) ----
        const _: () = {
            assert!(
                ($count as usize) <= 240,
                "bind_interrupts!: count exceeds Cortex-M3 maximum (240)"
            );
            assert!(
                !$crate::irq_dispatch::has_duplicate_irqs(&__IRQ_BINDINGS_INIT),
                "bind_interrupts!: duplicate IRQ number"
            );
            assert!(
                !$crate::irq_dispatch::has_invalid_partition_id(
                    &__IRQ_BINDINGS_INIT,
                    <$Config as $crate::config::KernelConfig>::N,
                ),
                "bind_interrupts!: partition_id out of range"
            );
            assert!(
                !$crate::irq_dispatch::has_zero_event_bits(&__IRQ_BINDINGS_INIT),
                "bind_interrupts!: event_bits == 0 is a no-op"
            );
            $(
                assert!(
                    ($irq as usize) < ($count as usize),
                    "bind_interrupts!: IRQ number >= count"
                );
            )+
        };

        // ---- suppress dead_code for handler: functions on non-ARM ----
        $( $crate::__ref_handler_if_custom!($args); )+

        // ---- feature gate (ARM only) ----
        #[cfg(all(not(test), target_arch = "arm", not(feature = "custom-ivt")))]
        compile_error!(
            "bind_interrupts! requires the `custom-ivt` feature; \
             add `features = [\"custom-ivt\"]` to your Cargo.toml"
        );

        // ---- binding table (ARM only) ----
        #[cfg(all(not(test), target_arch = "arm"))]
        static __IRQ_BINDINGS: [$crate::irq_dispatch::IrqBinding; 0 $( + { let _ = $irq; 1 } )+] = __IRQ_BINDINGS_INIT;

        // ---- direct dispatch table (ARM only) ----
        // $count is the IVT size (max IRQ slots), not the number of bindings;
        // the table covers all valid IRQ numbers.
        #[cfg(all(not(test), target_arch = "arm"))]
        const __IRQ_DIRECT_TABLE: [u8; $count] =
            $crate::irq_dispatch::build_direct_table::<{ $count }>(&__IRQ_BINDINGS_INIT);

        // ---- dispatch handler (ARM only) ----
        #[cfg(all(not(test), target_arch = "arm"))]
        extern "C" fn __irq_dispatch() {
            let ipsr: u32;
            // SAFETY: `mrs` reads the IPSR register, which is always
            // readable in any execution mode on Cortex-M.  The register
            // is read-only and has no side effects.
            unsafe { core::arch::asm!("mrs {}, ipsr", out(reg) ipsr) };
            let irq_num = $crate::irq_dispatch::ipsr_to_irq_num(ipsr);
            if let Some(&slot) = __IRQ_DIRECT_TABLE.get(irq_num as usize) {
                if slot == 0xFF {
                    return;
                }
                if let Some(b) = __IRQ_BINDINGS.get(slot as usize) {
                    // Clear/mask the interrupt source before signaling.
                    // All paths are data-only (no function pointers),
                    // guaranteeing bounded WCET.
                    match b.clear_model {
                        $crate::irq_dispatch::IrqClearModel::PartitionAcks => {
                            // Mask IRQ in NVIC to prevent level-triggered
                            // re-fire while the partition acknowledges.
                            cortex_m::peripheral::NVIC::mask(
                                $crate::irq_dispatch::IrqNr(irq_num),
                            );
                        }
                        $crate::irq_dispatch::IrqClearModel::KernelClears(strategy) => {
                            let (addr, value) = match strategy {
                                $crate::irq_dispatch::ClearStrategy::WriteRegister {
                                    addr, value,
                                } => (addr, value),
                                $crate::irq_dispatch::ClearStrategy::ClearBit {
                                    addr, bit,
                                } => (addr, 1u32.wrapping_shl(bit as u32)),
                            };
                            // SAFETY: The address is supplied by the caller of
                            // bind_interrupts! and is assumed to be a valid,
                            // 4-byte-aligned MMIO register suitable for a 32-bit
                            // volatile write.  No other aliasing concern applies
                            // because MMIO registers are inherently shared-access
                            // hardware and this write has a single-word width.
                            unsafe { (addr as *mut u32).write_volatile(value) };
                        }
                        $crate::irq_dispatch::IrqClearModel::NeverMask => {
                            // Edge-triggered / self-clearing source:
                            // no mask, no clear — fall through to signal.
                        }
                    }
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
        pub static __INTERRUPTS: [$crate::IsrHandler; $count] = {
            const DEFAULT: $crate::IsrHandler =
                __irq_dispatch as $crate::IsrHandler;
            let mut table = [DEFAULT; $count];
            $( table[$irq as usize] = $crate::__make_irq_handler!($args, __irq_dispatch); )+
            table
        };

        // ---- NVIC helpers ----

        /// Set the NVIC priority and unmask each IRQ bound by this macro.
        ///
        /// Returns `Err` if `priority` is numerically below
        /// `MIN_APP_IRQ_PRIORITY`, which would break the three-tier priority
        /// ordering.
        #[cfg(all(not(test), target_arch = "arm"))]
        pub fn enable_bound_irqs(nvic: &mut cortex_m::peripheral::NVIC, priority: u8) -> Result<(), &'static str> {
            $crate::config::validate_irq_priority(
                priority,
                <$Config as $crate::config::KernelConfig>::MIN_APP_IRQ_PRIORITY,
            )?;
            let _ = $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                k.store_irq_bindings(&__IRQ_BINDINGS);
            });
            $(
                // SAFETY: set_priority requires a valid IRQ number, which
                // is guaranteed by the compile-time assertion ($irq < $count).
                // unmask is unsafe because enabling an interrupt whose handler
                // is not installed would cause an unhandled exception.  Here,
                // the __INTERRUPTS IVT array (emitted by this same macro
                // invocation) contains the dispatch handler for every slot up
                // to $count, so the handler is already in place before this
                // function can be called.
                unsafe {
                    nvic.set_priority($crate::irq_dispatch::IrqNr($irq), priority);
                    cortex_m::peripheral::NVIC::unmask($crate::irq_dispatch::IrqNr($irq));
                }
            )+
            Ok(())
        }

        /// No-op on non-ARM hosts so examples compile with `cargo check`.
        /// The `nvic` parameter is generic to avoid referencing the
        /// architecture-specific `cortex_m::peripheral::NVIC` type.
        #[cfg(all(not(test), not(target_arch = "arm")))]
        pub fn enable_bound_irqs<T>(_nvic: &mut T, _priority: u8) -> Result<(), &'static str> { Ok(()) }

        /// Mask (disable) each IRQ bound by this macro.
        #[cfg(all(not(test), target_arch = "arm"))]
        pub fn disable_bound_irqs() {
            $(
                cortex_m::peripheral::NVIC::mask($crate::irq_dispatch::IrqNr($irq));
            )+
        }

        /// No-op on non-ARM hosts so examples compile with `cargo check`.
        #[cfg(all(not(test), not(target_arch = "arm")))]
        pub fn disable_bound_irqs() {}
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
            let _ = $crate::state::with_kernel_mut::<$Config, _, _>(|k| {
                $crate::tick::systick_handler::<$Config>(k);
            });
        }
    };
}

#[cfg(test)]
mod tests {
    use crate::PartitionEntry;

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
    fn svc_r01_returns_zero_pair_on_host() {
        let (r0, r1) = svc_r01!(1u32, 2u32, 3u32, 4u32);
        assert_eq!(r0, 0);
        assert_eq!(r1, 0);
    }

    #[test]
    fn partition_trampoline_has_correct_fn_type() {
        // The trampoline must have PartitionEntry type, compatible with
        // boot() which expects that signature for partition entry points.
        let _: PartitionEntry = test_trampoline;
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
    const _: () = {
        bind_interrupts!(DefaultConfig, 70, 5 => (0, 0x01), 23 => (1, 0x04));
    };

    #[test]
    fn bind_interrupts_const_validation_passes() {
        // If we reach here, the const assertion block in bind_interrupts!
        // accepted the bindings (no duplicates, all IRQs < 70).
    }

    // Verify the const validation also works with a single binding.
    const _: () = {
        bind_interrupts!(DefaultConfig, 10, 9 => (0, 0xFF));
    };

    #[test]
    fn bind_interrupts_single_binding_accepted() {
        // Single binding: IRQ 9 < 10, no duplicates — const assertion passes.
    }

    // Verify 3-tuple syntax with explicit clear model compiles.
    const _: () = {
        bind_interrupts!(DefaultConfig, 70,
            7 => (0, 0x02, crate::irq_dispatch::IrqClearModel::KernelClears(
                crate::irq_dispatch::ClearStrategy::WriteRegister { addr: 0x100, value: 1 },
            )),
        );
    };

    #[test]
    fn bind_interrupts_3tuple_accepted() {
        // If we reach here, the const assertion block accepted a 3-tuple binding.
    }

    // Verify mixed 2-tuple and 3-tuple bindings compile in a single invocation.
    const _: () = {
        bind_interrupts!(DefaultConfig, 80,
            10 => (0, 0x01),
            20 => (1, 0x08, crate::irq_dispatch::IrqClearModel::KernelClears(
                crate::irq_dispatch::ClearStrategy::ClearBit { addr: 0x200, bit: 3 },
            )),
            30 => (0, 0x10),
        );
    };

    #[test]
    fn bind_interrupts_mixed_tuples_accepted() {
        // Mixed 2-tuple and 3-tuple bindings in a single invocation compile.
    }

    // ---- __make_irq_binding! unit tests ----

    #[test]
    fn make_irq_binding_2tuple_defaults_to_partition_acks() {
        let b = __make_irq_binding!(5, (0, 0x01));
        assert_eq!(b.irq_num, 5);
        assert_eq!(b.partition_id, crate::PartitionId::new(0));
        assert_eq!(b.event_bits, 0x01);
        assert_eq!(
            b.clear_model,
            crate::irq_dispatch::IrqClearModel::PartitionAcks
        );
    }

    #[test]
    fn make_irq_binding_3tuple_uses_explicit_clear_model() {
        let clear = crate::irq_dispatch::IrqClearModel::KernelClears(
            crate::irq_dispatch::ClearStrategy::WriteRegister {
                addr: 0x4000_0000,
                value: 0xFF,
            },
        );
        let b = __make_irq_binding!(12, (1, 0x04, clear));
        assert_eq!(b.irq_num, 12);
        assert_eq!(b.partition_id, crate::PartitionId::new(1));
        assert_eq!(b.event_bits, 0x04);
        assert_eq!(b.clear_model, clear);
    }

    #[test]
    fn make_irq_binding_3tuple_clear_bit() {
        let clear = crate::irq_dispatch::IrqClearModel::KernelClears(
            crate::irq_dispatch::ClearStrategy::ClearBit {
                addr: 0x200,
                bit: 7,
            },
        );
        let b = __make_irq_binding!(42, (0, 0x80, clear));
        assert_eq!(b.irq_num, 42);
        assert_eq!(b.event_bits, 0x80);
        assert_eq!(b.clear_model, clear);
    }

    #[test]
    fn make_irq_binding_2tuple_matches_irq_binding_new() {
        let from_macro = __make_irq_binding!(99, (3, 0xDEAD));
        let direct = crate::irq_dispatch::IrqBinding::new(99, crate::PartitionId::new(3), 0xDEAD);
        assert_eq!(from_macro, direct);
    }

    // ---- __make_irq_binding! handler: form tests ----

    #[test]
    fn make_irq_binding_handler_defaults_to_partition_acks() {
        let b = __make_irq_binding!(5, (0, 0x01, handler: test_custom_isr));
        assert_eq!(b.irq_num, 5);
        assert_eq!(b.partition_id, crate::PartitionId::new(0));
        assert_eq!(b.event_bits, 0x01);
        assert_eq!(
            b.clear_model,
            crate::irq_dispatch::IrqClearModel::PartitionAcks
        );
    }

    #[test]
    fn make_irq_binding_handler_matches_irq_binding_new() {
        let from_macro = __make_irq_binding!(7, (1, 0x04, handler: test_custom_isr));
        let direct = crate::irq_dispatch::IrqBinding::new(7, crate::PartitionId::new(1), 0x04);
        assert_eq!(from_macro, direct);
    }

    // ---- __make_irq_handler! unit tests ----

    unsafe extern "C" fn test_default_dispatch() {}
    const _: crate::IsrHandler = test_default_dispatch;
    unsafe extern "C" fn test_custom_isr() {}
    const _: crate::IsrHandler = test_custom_isr;

    #[test]
    fn make_irq_handler_2tuple_returns_default() {
        let h = __make_irq_handler!((0, 0x01), test_default_dispatch);
        assert_eq!(
            h as *const () as usize,
            test_default_dispatch as *const () as usize
        );
    }

    #[test]
    fn make_irq_handler_3tuple_returns_default() {
        let _clear = crate::irq_dispatch::IrqClearModel::KernelClears(
            crate::irq_dispatch::ClearStrategy::WriteRegister {
                addr: 0x100,
                value: 1,
            },
        );
        let h = __make_irq_handler!((0, 0x01, _clear), test_default_dispatch);
        assert_eq!(
            h as *const () as usize,
            test_default_dispatch as *const () as usize
        );
    }

    #[test]
    fn make_irq_handler_custom_returns_custom_fn() {
        let h = __make_irq_handler!((0, 0x01, handler: test_custom_isr), test_default_dispatch);
        assert_eq!(
            h as *const () as usize,
            test_custom_isr as *const () as usize
        );
        assert_ne!(
            h as *const () as usize,
            test_default_dispatch as *const () as usize
        );
    }

    #[test]
    fn make_irq_handler_output_is_isr_handler() {
        // Verify the macro output is assignable to IsrHandler (not just any fn pointer).
        let h: crate::IsrHandler = __make_irq_handler!((0, 0x01), test_default_dispatch);
        assert_eq!(
            h as *const () as usize,
            test_default_dispatch as *const () as usize
        );

        let h2: crate::IsrHandler =
            __make_irq_handler!((0, 0x01, handler: test_custom_isr), test_default_dispatch);
        assert_eq!(
            h2 as *const () as usize,
            test_custom_isr as *const () as usize
        );
    }

    // ---- bind_interrupts! with handler: form ----

    const _: () = {
        bind_interrupts!(DefaultConfig, 90, 11 => (0, 0x01, handler: test_custom_isr));
    };

    #[test]
    fn bind_interrupts_handler_form_accepted() {
        // handler: form accepted by const validation.
    }

    const _: () = {
        bind_interrupts!(DefaultConfig, 100,
            15 => (0, 0x01),
            25 => (1, 0x04, handler: test_custom_isr),
            35 => (0, 0x10),
        );
    };

    #[test]
    fn bind_interrupts_mixed_with_handler_accepted() {
        // Mixed 2-tuple and handler: bindings compile in a single invocation.
    }

    // ---- handler: dead_code suppression on host ----

    // This function is ONLY referenced through bind_interrupts! handler:
    // form.  If the dead_code suppression emitted by __ref_handler_if_custom!
    // is not working, this will fail to compile with a dead_code warning
    // (the test crate uses #![deny(dead_code)]).
    unsafe extern "C" fn handler_only_via_macro() {}

    const _: () = {
        bind_interrupts!(DefaultConfig, 50, 3 => (0, 0x01, handler: handler_only_via_macro));
    };

    #[test]
    fn bind_interrupts_handler_no_dead_code_on_host() {
        // Confirms handler: functions referenced only through bind_interrupts!
        // compile without #[allow(dead_code)] on host builds.  The function
        // `handler_only_via_macro` has no other reference.
    }

    // enable_bound_irqs / disable_bound_irqs are emitted behind
    // cfg(all(not(test), target_arch = "arm")).  Functional testing is
    // via the qemu_custom_ivt integration example which calls
    // enable_bound_irqs(&mut p.NVIC, 0xC0) at boot.

    #[test]
    fn irq_nr_used_by_nvic_helpers() {
        // The NVIC helpers construct IrqNr for each bound IRQ.
        // Verify IrqNr correctly stores arbitrary IRQ numbers.
        let a = crate::irq_dispatch::IrqNr(0);
        let b = crate::irq_dispatch::IrqNr(255);
        assert_eq!(a.0, 0);
        assert_eq!(b.0, 255);
        assert_ne!(a, b);
    }

    // ---- clear: keyword form tests ----

    #[test]
    fn make_irq_binding_clear_write_register_keyword() {
        let b = __make_irq_binding!(10, (0, 0x02, clear: WriteRegister(0x4000_1000, 0xAB)));
        assert_eq!(b.irq_num, 10);
        assert_eq!(b.partition_id, crate::PartitionId::new(0));
        assert_eq!(b.event_bits, 0x02);
        assert_eq!(
            b.clear_model,
            crate::irq_dispatch::IrqClearModel::KernelClears(
                crate::irq_dispatch::ClearStrategy::WriteRegister {
                    addr: 0x4000_1000,
                    value: 0xAB
                },
            ),
        );
    }

    #[test]
    fn make_irq_binding_clear_clearbit_keyword() {
        let b = __make_irq_binding!(20, (1, 0x08, clear: ClearBit(0x4000_2000, 5)));
        assert_eq!(b.irq_num, 20);
        assert_eq!(b.partition_id, crate::PartitionId::new(1));
        assert_eq!(b.event_bits, 0x08);
        assert_eq!(
            b.clear_model,
            crate::irq_dispatch::IrqClearModel::KernelClears(
                crate::irq_dispatch::ClearStrategy::ClearBit {
                    addr: 0x4000_2000,
                    bit: 5
                },
            ),
        );
    }

    #[test]
    fn make_irq_handler_clear_keyword_returns_default() {
        let h1 = __make_irq_handler!(
            (0, 0x01, clear: WriteRegister(0x100, 1)),
            test_default_dispatch
        );
        let h2 = __make_irq_handler!(
            (0, 0x01, clear: ClearBit(0x200, 3)),
            test_default_dispatch
        );
        assert_eq!(
            h1 as *const () as usize,
            test_default_dispatch as *const () as usize
        );
        assert_eq!(
            h2 as *const () as usize,
            test_default_dispatch as *const () as usize
        );
    }

    // Mixed: 2-tuple, legacy 3-tuple, and clear: keyword forms together.
    const _: () = {
        bind_interrupts!(DefaultConfig, 80,
            41 => (0, 0x01),
            51 => (1, 0x04, clear: WriteRegister(0x100, 0xFF)),
            55 => (0, 0x02, clear: ClearBit(0x4000_0004, 7)),
            61 => (0, 0x10, crate::irq_dispatch::IrqClearModel::KernelClears(
                crate::irq_dispatch::ClearStrategy::ClearBit { addr: 0x200, bit: 2 },
            )),
        );
    };

    #[test]
    fn bind_interrupts_clear_keyword_and_mixed_accepted() {
        // 2-tuple, clear: keyword, and legacy 3-tuple forms compile together.
    }

    // ---- never_mask keyword form tests ----

    #[test]
    fn make_irq_binding_never_mask_keyword() {
        let b = __make_irq_binding!(15, (0, 0x04, never_mask));
        assert_eq!(b.irq_num, 15);
        assert_eq!(b.partition_id, crate::PartitionId::new(0));
        assert_eq!(b.event_bits, 0x04);
        assert_eq!(b.clear_model, crate::irq_dispatch::IrqClearModel::NeverMask);
    }

    #[test]
    fn make_irq_binding_never_mask_handler_keyword() {
        let b = __make_irq_binding!(16, (1, 0x08, never_mask, handler: test_custom_isr));
        assert_eq!(b.irq_num, 16);
        assert_eq!(b.partition_id, crate::PartitionId::new(1));
        assert_eq!(b.event_bits, 0x08);
        assert_eq!(b.clear_model, crate::irq_dispatch::IrqClearModel::NeverMask);
    }

    #[test]
    fn make_irq_handler_never_mask_returns_default() {
        let h = __make_irq_handler!((0, 0x01, never_mask), test_default_dispatch);
        assert_eq!(
            h as *const () as usize,
            test_default_dispatch as *const () as usize
        );
    }

    #[test]
    fn make_irq_handler_never_mask_handler_returns_custom() {
        let h = __make_irq_handler!(
            (0, 0x01, never_mask, handler: test_custom_isr),
            test_default_dispatch
        );
        assert_eq!(
            h as *const () as usize,
            test_custom_isr as *const () as usize
        );
    }

    // Verify never_mask compiles in bind_interrupts! (standalone and mixed).
    const _: () = {
        bind_interrupts!(DefaultConfig, 80,
            42 => (0, 0x01, never_mask),
        );
    };

    const _: () = {
        bind_interrupts!(DefaultConfig, 80,
            43 => (0, 0x01, never_mask, handler: test_custom_isr),
        );
    };

    const _: () = {
        bind_interrupts!(DefaultConfig, 90,
            44 => (0, 0x01),
            45 => (1, 0x02, never_mask),
            46 => (0, 0x04, never_mask, handler: test_custom_isr),
            47 => (1, 0x08, clear: WriteRegister(0x100, 1)),
        );
    };

    #[test]
    fn bind_interrupts_never_mask_accepted() {
        // never_mask keyword accepted by const validation.
    }

    #[test]
    fn bind_interrupts_never_mask_handler_accepted() {
        // never_mask + handler: form accepted by const validation.
    }

    #[test]
    fn bind_interrupts_never_mask_mixed_accepted() {
        // never_mask mixed with other forms in a single invocation compiles.
    }

    // ---- const_assert! tests ----

    // Exercise both const_assert! forms at item position.
    const_assert!(true);
    const_assert!(core::mem::size_of::<u32>() == 4, "u32 must be 4 bytes");

    #[test]
    fn const_assert_compiles_with_true_condition() {
        // The const_assert! invocations above compile, confirming the
        // macro produces valid `const _: () = assert!(...)` items.
        const_assert!(1 + 1 == 2);
        const_assert!(42 > 0, "positive");
    }
}
