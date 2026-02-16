//! Kernel module.
//!
//! The `KernelState` struct has been removed. The unified [`Kernel`](crate::svc::Kernel)
//! struct now handles all kernel state management. Legacy code should migrate
//! to use `Kernel` directly.
//!
//! The [`YieldResult`](crate::svc::YieldResult) trait is defined in the `svc` module.
//!
