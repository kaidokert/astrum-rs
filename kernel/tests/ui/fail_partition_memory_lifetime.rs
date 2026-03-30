// Compile-fail test: the `'mem` lifetime on `ExternalPartitionMemory`
// prevents constructing a `Kernel` from dangling stack memory.
//
// This test creates an `AlignedStack256B` in an inner scope, builds
// an `ExternalPartitionMemory` from it, then attempts to pass that
// descriptor to `Kernel::new` after the stack has been dropped.
// The compiler must reject this because the borrow does not live
// long enough.
//
// TODO: The `'mem` phantom lifetime on `Kernel` itself is not
// enforced by `Kernel::new` (which uses `'_` for the memories
// parameter) because the constructor copies all descriptor data.
// A future API tightening could bind `Kernel::new`'s memory
// parameter to `'mem`, providing direct Kernel-level enforcement.

#![no_std]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
extern crate std;

use kernel::partition::ExternalPartitionMemory;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::{AlignedStack256B, DefaultConfig};

fn main() {
    let mem;
    {
        let mut stack = AlignedStack256B::default();
        mem = ExternalPartitionMemory::from_aligned_stack(
            &mut stack,
            0x0800_0001,
            rtos_traits::partition::MpuRegion::new(0x2000_0000, 256, 0x03),
            kernel::PartitionId::new(0),
        )
        .unwrap();
    }
    // `stack` is dropped — `mem` holds a dangling &mut [u32].
    // Attempting to build a Kernel from the escaped descriptor fails
    // because the borrow checker sees `mem` used after the stack dies.
    let mut sched = ScheduleTable::<{ DefaultConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 5)).unwrap();
    sched.add_system_window(1).unwrap();
    sched.start();

    let k = Kernel::<DefaultConfig>::new(sched, &[mem]).unwrap();
    let _ = k.active_partition();
}
