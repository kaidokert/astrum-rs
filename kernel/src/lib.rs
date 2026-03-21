#![cfg_attr(not(test), no_std)]
#![allow(incomplete_features)]
// TODO: remove once const_trait_impl / const_index stabilise on nightly.
#![feature(const_index)]
#![feature(const_trait_impl)]
#![feature(generic_const_exprs)]
#![feature(where_clause_attrs)]
// Require SAFETY comments for all unsafe blocks in the kernel library
#![deny(clippy::undocumented_unsafe_blocks)]

// Provide the interrupt vector table required by cortex-m-rt 0.7.
// Without a device PAC crate, we supply a minimal one-entry array.
// The LM3S6965 (used in QEMU) has 70 interrupts, but we only need
// at least one entry to satisfy the linker's SIZEOF(.vector_table) > 0x40 assertion.
// All entries point to the default handler provided by cortex-m-rt.
#[cfg(all(not(test), target_arch = "arm", not(feature = "custom-ivt")))]
#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [IsrHandler; 1] = [DefaultHandler; 1];

#[cfg(all(not(test), target_arch = "arm", not(feature = "custom-ivt")))]
extern "C" {
    fn DefaultHandler();
}

pub mod api;

pub mod klog;
pub mod kpanic;
pub mod macros;

pub mod boot;
pub mod harness;

pub mod blackboard;
#[cfg(feature = "dynamic-mpu")]
pub mod buf_syscall;
#[cfg(feature = "dynamic-mpu")]
pub mod buffer_pool;
pub mod config;
pub use config::{
    DebugConfig, DebugDisabled, DebugEnabled, DefaultConfig, KernelConfig, MsgConfig, MsgMinimal,
    MsgRich, MsgSmall, MsgStandard, PartitionConfig, Partitions1, Partitions2, Partitions3,
    Partitions4, PortsConfig, PortsMinimal, PortsRich, PortsSmall, PortsStandard, PortsTiny,
    SyncConfig, SyncMinimal, SyncRich, SyncStandard,
};
pub mod context;
#[cfg(feature = "partition-debug")]
pub mod debug;
pub mod events;
#[cfg(feature = "dynamic-mpu")]
pub mod hw_uart;
pub mod invariants;
pub mod irq_ack;
pub mod irq_dispatch;
pub mod kernel_ptr;
pub(crate) mod layout_checks;
pub mod message;
pub mod mpu;
#[cfg(feature = "dynamic-mpu")]
pub mod mpu_strategy;
pub mod msg_pools;
pub mod mutex;
pub mod partition;
/// Re-exports from [`partition`].
///
/// **Deprecated:** [`body_point_addr`] and [`entry_point_addr`] are deprecated.
/// Use [`EntryAddr::from_body`] and [`EntryAddr::from_fn`] instead.
#[allow(deprecated)]
pub use partition::{
    body_point_addr, entry_point_addr, EntryAddr, ExternalPartitionMemory, IsrHandler,
    PartitionBody, PartitionEntry, PartitionMemory, PartitionSpec,
};
pub mod partition_core;
pub use partition_core::{
    AlignedStack1K, AlignedStack256B, AlignedStack2K, AlignedStack4K, AlignedStack512B,
    StackStorage,
};
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
pub mod split_isr;
pub mod state;
pub mod svc;
pub mod sync_pools;
pub mod syscall;
pub mod systick;
pub mod tick;

// Re-export handle_systick at crate root for convenience
pub use svc::SvcDispatchFn;
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
pub mod ipc_integrity_tests;
#[cfg(test)]
pub mod test_harness;
#[cfg(test)]
pub mod test_mmap;

#[cfg(test)]
#[allow(deprecated)]
mod reexport_tests {
    //! Verify that all config presets, traits, and DefaultConfig are
    //! accessible via the crate root (i.e. `crate::TypeName`).

    use super::*;

    fn _assert_partition_cfg<T: PartitionConfig>() {}
    fn _assert_sync_cfg<T: SyncConfig>() {}
    fn _assert_msg_cfg<T: MsgConfig>() {}
    fn _assert_ports_cfg<T: PortsConfig>() {}
    fn _assert_debug_cfg<T: DebugConfig>() {}

    #[test]
    fn partition_presets_via_root() {
        _assert_partition_cfg::<Partitions1>();
        _assert_partition_cfg::<Partitions2>();
        _assert_partition_cfg::<Partitions3>();
        _assert_partition_cfg::<Partitions4>();

        assert_eq!(Partitions1::COUNT, 1);
        assert_eq!(Partitions2::COUNT, 2);
        assert_eq!(Partitions3::COUNT, 3);
        assert_eq!(Partitions4::COUNT, 4);
    }

    #[test]
    fn sync_presets_via_root() {
        _assert_sync_cfg::<SyncMinimal>();
        _assert_sync_cfg::<SyncRich>();
        _assert_sync_cfg::<SyncStandard>();

        assert_eq!(SyncMinimal::SEMAPHORES, 1);
        const { assert!(SyncRich::SEMAPHORES > SyncMinimal::SEMAPHORES) };
    }

    #[test]
    fn msg_presets_via_root() {
        _assert_msg_cfg::<MsgMinimal>();
        _assert_msg_cfg::<MsgSmall>();
        _assert_msg_cfg::<MsgRich>();
        _assert_msg_cfg::<MsgStandard>();

        assert_eq!(MsgMinimal::QUEUES, 1);
        const { assert!(MsgRich::QUEUES >= MsgSmall::QUEUES) };
    }

    #[test]
    fn ports_presets_via_root() {
        _assert_ports_cfg::<PortsMinimal>();
        _assert_ports_cfg::<PortsTiny>();
        _assert_ports_cfg::<PortsRich>();
        _assert_ports_cfg::<PortsSmall>();
        _assert_ports_cfg::<PortsStandard>();

        assert_eq!(PortsMinimal::SAMPLING_PORTS, 1);
        const { assert!(PortsRich::SAMPLING_PORTS >= PortsTiny::SAMPLING_PORTS) };
    }

    #[test]
    fn ports_standard_constants() {
        assert_eq!(PortsStandard::SAMPLING_PORTS, 8);
        assert_eq!(PortsStandard::SAMPLING_MAX_MSG_SIZE, 4);
        assert_eq!(PortsStandard::BLACKBOARDS, 4);
        assert_eq!(PortsStandard::BLACKBOARD_MAX_MSG_SIZE, 4);
        assert_eq!(PortsStandard::BLACKBOARD_WAITQ, 4);
    }

    #[test]
    fn debug_presets_via_root() {
        _assert_debug_cfg::<DebugEnabled>();
        _assert_debug_cfg::<DebugDisabled>();

        const { assert!(DebugEnabled::AUTO_DRAIN_BUDGET > 0) };
        assert_eq!(DebugDisabled::AUTO_DRAIN_BUDGET, 0);
    }

    fn _assert_kernel_cfg<T: KernelConfig>() {}

    #[test]
    fn kernel_config_trait_via_root() {
        _assert_kernel_cfg::<DefaultConfig>();
    }

    #[test]
    fn default_config_via_root() {
        // DefaultConfig is composed — verify inherent constants
        // that originate from each sub-config preset.
        assert_eq!(DefaultConfig::N, 2); // Partitions2
        assert_eq!(DefaultConfig::S, 1); // SyncMinimal
        assert_eq!(DefaultConfig::QS, 1); // MsgMinimal
        assert_eq!(DefaultConfig::SP, 1); // PortsTiny
    }

    #[test]
    fn partition_type_aliases_via_root() {
        // PartitionBody, PartitionEntry, PartitionSpec are re-exported at crate root.
        #[allow(clippy::empty_loop)]
        extern "C" fn _body(_: u32) -> ! {
            loop {}
        }
        #[allow(clippy::empty_loop)]
        extern "C" fn _entry() -> ! {
            loop {}
        }
        let _: PartitionBody = _body;
        let _: PartitionEntry = _entry;
        let spec = PartitionSpec::new(_entry, 99);
        assert_eq!(spec.r0(), 99);
    }

    #[test]
    fn isr_handler_type_compatibility() {
        // A bare `unsafe extern "C" fn()` must be assignable to IsrHandler.
        unsafe extern "C" fn my_isr() {}
        let handler: IsrHandler = my_isr;
        // Verify the function pointer round-trips to the correct address.
        assert_eq!(handler as *const () as usize, my_isr as *const () as usize);
    }

    #[test]
    fn isr_handler_reexport_via_root() {
        // IsrHandler must be accessible as kernel::IsrHandler (root re-export).
        unsafe extern "C" fn another_isr() {}
        let _h: IsrHandler = another_isr;
        // Confirm it is the same type from the partition module.
        let _h2: crate::partition::IsrHandler = another_isr;
        assert_eq!(_h as *const () as usize, _h2 as *const () as usize);
    }

    #[test]
    fn entry_point_addr_via_root() {
        // entry_point_addr is re-exported at crate root.
        #[allow(clippy::empty_loop)]
        extern "C" fn _ep() -> ! {
            loop {}
        }
        let addr = entry_point_addr(_ep);
        assert_eq!(addr, _ep as *const () as u32);
    }

    #[test]
    fn body_point_addr_via_root() {
        #[allow(clippy::empty_loop)]
        extern "C" fn _bp(_r0: u32) -> ! {
            loop {}
        }
        let addr = body_point_addr(_bp);
        assert_eq!(addr, _bp as *const () as u32);
    }

    #[test]
    fn partition_memory_alias_via_root() {
        // PartitionMemory is re-exported at crate root as an alias for ExternalPartitionMemory
        fn _assert_same_type(_: PartitionMemory<'_>) {}
        fn _accept_external(e: ExternalPartitionMemory<'_>) {
            _assert_same_type(e);
        }
        let _ = _accept_external;
    }

    #[test]
    fn default_config_associated_types() {
        // Verify DefaultConfig's associated config types match expected presets.
        assert_eq!(
            <DefaultConfig as KernelConfig>::PartitionCfg::COUNT,
            Partitions2::COUNT
        );
        assert_eq!(
            <DefaultConfig as KernelConfig>::SyncCfg::SEMAPHORES,
            SyncMinimal::SEMAPHORES
        );
        assert_eq!(
            <DefaultConfig as KernelConfig>::MsgCfg::QUEUES,
            MsgMinimal::QUEUES
        );
        assert_eq!(
            <DefaultConfig as KernelConfig>::PortsCfg::SAMPLING_PORTS,
            PortsTiny::SAMPLING_PORTS
        );
        assert_eq!(
            <DefaultConfig as KernelConfig>::DebugCfg::AUTO_DRAIN_BUDGET,
            DebugEnabled::AUTO_DRAIN_BUDGET
        );
    }
}
