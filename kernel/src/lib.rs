#![cfg_attr(not(test), no_std)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

pub mod blackboard;
pub mod config;
pub mod context;
pub mod events;
pub mod kernel;
pub mod message;
pub mod mpu;
pub mod mutex;
pub mod partition;
pub mod pendsv;
pub mod queuing;
pub mod sampling;
pub mod scheduler;
pub mod semaphore;
pub mod svc;
pub mod syscall;
pub mod tick;
pub mod waitqueue;
