#![cfg_attr(not(test), no_std)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(where_clause_attrs)]
// Require SAFETY comments for all unsafe blocks in the kernel library
#![deny(clippy::undocumented_unsafe_blocks)]

// Provide the interrupt vector table required by cortex-m-rt 0.7.
// Without a device PAC crate, we supply a minimal one-entry array.
// The LM3S6965 (used in QEMU) has 70 interrupts, but we only need
// at least one entry to satisfy the linker's SIZEOF(.vector_table) > 0x40 assertion.
// All entries point to the default handler provided by cortex-m-rt.
#[cfg(all(not(test), target_arch = "arm"))]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [unsafe extern "C" fn(); 1] = [DefaultHandler; 1];

#[cfg(all(not(test), target_arch = "arm"))]
extern "C" {
    fn DefaultHandler();
}

pub mod klog;
pub mod kpanic;
pub mod macros;

pub mod boot;
pub mod harness;

pub mod blackboard;
#[cfg(feature = "dynamic-mpu")]
pub mod buffer_pool;
pub mod config;
pub mod context;
#[cfg(feature = "partition-debug")]
pub mod debug;
pub mod events;
#[cfg(feature = "dynamic-mpu")]
pub mod hw_uart;
pub mod invariants;
pub mod message;
pub mod mpu;
#[cfg(feature = "dynamic-mpu")]
pub mod mpu_strategy;
pub mod msg_pools;
pub mod mutex;
pub mod partition;
pub mod partition_core;
#[cfg(feature = "partition-debug")]
pub mod partition_debug;
pub mod pendsv;
pub mod pendsv_asm;
#[cfg(feature = "partition-debug")]
pub mod plib;
pub mod port_pools;
pub mod queuing;
pub mod sampling;
pub mod scheduler;
pub mod semaphore;
#[cfg(feature = "dynamic-mpu")]
pub mod split_isr;
pub mod state;
pub mod svc;
pub mod sync_pools;
pub mod syscall;
pub mod systick;
pub mod tick;

// Re-export handle_systick at crate root for convenience
#[cfg(not(test))]
pub use systick::handle_systick;
#[cfg(feature = "dynamic-mpu")]
pub mod uart_hal;
#[cfg(feature = "dynamic-mpu")]
pub mod virtual_device;
#[cfg(feature = "dynamic-mpu")]
pub mod virtual_uart;
pub mod waitqueue;

#[cfg(test)]
pub mod test_harness;
