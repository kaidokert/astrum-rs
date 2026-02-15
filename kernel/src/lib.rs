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
#[cfg(feature = "dynamic-mpu")]
pub mod hw_uart;
pub mod kernel;
pub mod message;
pub mod mpu;
#[cfg(feature = "dynamic-mpu")]
pub mod mpu_strategy;
pub mod msg_pools;
pub mod mutex;
pub mod partition;
pub mod partition_core;
pub mod pendsv;
pub mod port_pools;
pub mod queuing;
pub mod sampling;
pub mod scheduler;
pub mod semaphore;
#[cfg(feature = "dynamic-mpu")]
pub mod split_isr;
pub mod svc;
pub mod sync_pools;
pub mod syscall;
pub mod tick;
#[cfg(feature = "dynamic-mpu")]
pub mod uart_hal;
#[cfg(feature = "dynamic-mpu")]
pub mod virtual_device;
#[cfg(feature = "dynamic-mpu")]
pub mod virtual_uart;
pub mod waitqueue;
