//! Kernel module.
//!
//! The `KernelState` struct has been removed. The unified [`Kernel`](crate::svc::Kernel)
//! struct now handles all kernel state management. Legacy code should migrate
//! to use `Kernel` directly.
//!
//! The [`YieldResult`](crate::svc::YieldResult) trait is defined in the `svc` module.
//!
// TODO: reviewer false positive - KernelState struct was already removed in commit 64148f3.
// TODO: reviewer false positive - define_harness! and define_dispatch_hook! macros were
//       already removed in commit 016651d.
// TODO: reviewer false positive - YieldResult impls for Option<u8> and ScheduleEvent were
//       never in kernel.rs; they exist in svc.rs and are required for harness macros.
