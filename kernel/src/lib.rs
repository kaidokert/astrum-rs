#![cfg_attr(not(test), no_std)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(where_clause_attrs)]

pub mod macros;

pub mod harness;

pub mod blackboard;
#[cfg(feature = "dynamic-mpu")]
pub mod buffer_pool;
pub mod config;
pub mod context;
pub mod events;
pub mod kernel;
pub mod message;
pub mod mpu;
#[cfg(feature = "dynamic-mpu")]
pub mod mpu_strategy;
pub mod mutex;
pub mod partition;
pub mod pendsv;
pub mod queuing;
pub mod sampling;
pub mod scheduler;
pub mod semaphore;
#[cfg(feature = "dynamic-mpu")]
pub mod split_isr;
pub mod svc;
pub mod syscall;
pub mod tick;
pub mod waitqueue;
