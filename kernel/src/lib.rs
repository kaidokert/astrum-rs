// TODO: reviewer asked for #![no_std] unconditionally, but cfg_attr is needed
// so `cargo test` on the host can link the test harness which requires std.
#![cfg_attr(not(test), no_std)]
