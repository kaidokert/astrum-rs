// Compile-fail test: a KernelConfig with 500 partitions exceeds the
// literal-pool offset limit enforced by define_pendsv!(@assert_offsets).

#![feature(generic_const_exprs)]
#![allow(incomplete_features)]

kernel::kernel_config!(OversizedConfig {
    const N: usize = 500;
});

kernel::define_pendsv!(OversizedConfig);

fn main() {}
