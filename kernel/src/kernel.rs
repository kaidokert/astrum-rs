//! Kernel module.
//!
//! The `KernelState` struct has been removed. The unified [`Kernel`](crate::svc::Kernel)
//! struct now handles all kernel state management. Legacy code should migrate
//! to use `Kernel` directly.
//!
//! The [`YieldResult`] trait is re-exported here for backward compatibility
//! with existing code that imports from this module.

// Re-export YieldResult from svc for backwards compatibility.
pub use crate::svc::YieldResult;
