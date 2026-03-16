/// Shared compile-time layout assertions for `Kernel<'mem, Config>` / `Core`.
#[macro_export]
#[doc(hidden)]
macro_rules! assert_kernel_layout {
    ($Config:ty) => { const _: () = {
        type K = $crate::svc::Kernel<'static, $Config>;
        type C = <$Config as $crate::config::KernelConfig>::Core;
        const LIM: usize = $crate::pendsv::LITERAL_POOL_OFFSET_LIMIT;
        // All ABI-visible offsets must be < LITERAL_POOL_OFFSET_LIMIT.
        assert!(::core::mem::offset_of!(K, current_partition) < LIM,
            "KERNEL_CURRENT_PARTITION_OFFSET exceeds literal-pool offset limit");
        assert!(::core::mem::offset_of!(K, ticks_dropped) < LIM,
            "KERNEL_TICKS_DROPPED_OFFSET exceeds literal-pool offset limit");
        assert!(::core::mem::offset_of!(K, core) < LIM,
            "KERNEL_CORE_OFFSET exceeds literal-pool offset limit");
        assert!(::core::mem::offset_of!(K, core) + ::core::mem::offset_of!(C, next_partition) < LIM,
            "KERNEL_CORE_OFFSET + CORE_NEXT_PARTITION_OFFSET exceeds literal-pool offset limit");
        assert!(::core::mem::offset_of!(K, core) + ::core::mem::offset_of!(C, partition_sp) < LIM,
            "KERNEL_CORE_OFFSET + CORE_PARTITION_SP_OFFSET exceeds literal-pool offset limit");
        assert!(::core::mem::offset_of!(K, core) + ::core::mem::offset_of!(C, partition_stack_limits) < LIM,
            "KERNEL_CORE_OFFSET + CORE_PARTITION_STACK_LIMIT_OFFSET exceeds literal-pool offset limit");
        // Field ordering: current_partition < core; next_partition < partition_sp.
        assert!(::core::mem::offset_of!(K, current_partition) < ::core::mem::offset_of!(K, core),
            "current_partition must precede core in Kernel layout");
        assert!(::core::mem::offset_of!(C, next_partition) < ::core::mem::offset_of!(C, partition_sp),
            "next_partition must precede partition_sp in Core layout");
        // partition_sp elements must be u32 with stride 4 (PendSV uses lsl #2).
        #[allow(unused)]
        fn _assert_sp_elem_is_u32_stride_4(c: &C) {
            let _: u32 = c.partition_sp[0];
            let _: [u8; 4] = c.partition_sp[0].to_ne_bytes();
        }
        // partition_sp combined offset must be 4-byte aligned.
        assert!((::core::mem::offset_of!(K, core) + ::core::mem::offset_of!(C, partition_sp)) % 4 == 0,
            "partition_sp must be 4-byte aligned");
    }; };
}

#[cfg(test)]
mod tests {
    #[test]
    fn shared_layout_checks_pass_for_default_config() {
        crate::assert_kernel_layout!(crate::config::DefaultConfig);
    }
}
