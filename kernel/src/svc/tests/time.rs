use super::*;

#[test]
fn get_time_returns_zero_initially() {
    let mut k = kernel(0, 0, 0);
    let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
}

#[test]
fn get_time_after_increments() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick(5);
    let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 5);
}

#[test]
fn get_time_preserves_other_registers() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick(1);
    let mut ef = frame(crate::syscall::SYS_GET_TIME, 0xAA, 0xBB);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1);
    assert_eq!(ef.r1, 0xAA);
    assert_eq!(ef.r2, 0xBB);
}

#[test]
fn get_time_truncates_to_u32() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick((1u64 << 32) + 7);
    let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 7);
}

#[test]
fn sync_tick_updates_kernel_tick() {
    let mut k = kernel(0, 0, 0);
    assert_eq!(k.tick().get(), 0);
    k.sync_tick(42);
    assert_eq!(k.tick().get(), 42);
    k.sync_tick(1000);
    assert_eq!(k.tick().get(), 1000);
}

#[test]
fn sampling_dispatch_write_read_and_errors() {
    use crate::sampling::PortDirection;
    use crate::syscall::{SYS_SAMPLING_READ, SYS_SAMPLING_WRITE};
    let mut k = kernel(0, 0, 0);
    let src = k
        .sampling_mut()
        .create_port(PortDirection::Source, 1000)
        .unwrap();
    let dst = k
        .sampling_mut()
        .create_port(PortDirection::Destination, 1000)
        .unwrap();
    k.sampling_mut().connect_ports(src, dst).unwrap();
    // Write + read via pool (avoids 64-bit pointer truncation issue).
    let tick = k.tick().get();
    k.sampling_mut()
        .write_sampling_message(src, &[0xAA, 0xBB], tick)
        .unwrap();
    let mut buf = [0u8; 64];
    let tick = k.tick().get();
    let (n, _) = k
        .sampling_mut()
        .read_sampling_message(dst, &mut buf, tick)
        .unwrap();
    assert_eq!((n, &buf[..n]), (2, &[0xAA, 0xBB][..]));
    // Invalid port → InvalidResource error for both write and read.
    // Use an address inside partition 0's MPU region so pointer
    // validation passes and the invalid port ID (99) is reached.
    let inv = SvcError::InvalidResource.to_u32();
    let mut ef = frame4(SYS_SAMPLING_WRITE, 99, 1, 0x2000_0000);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, inv);
    let mut ef = frame4(SYS_SAMPLING_READ, 99, 0, 0x2000_0000);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, inv);
}

#[test]
fn sync_tick_then_get_time_returns_synced_value() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick(42);
    let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 42);
}

#[test]
fn sync_tick_overwrite_returns_latest_value() {
    let mut k = kernel(0, 0, 0);
    k.sync_tick(5);
    let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 5);
    k.sync_tick(10);
    let mut ef = frame(crate::syscall::SYS_GET_TIME, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 10);
}

#[cfg(feature = "dynamic-mpu")]
#[test]
fn sync_tick_affects_buffer_alloc_deadline() {
    use crate::syscall::SYS_BUF_ALLOC;
    let mut k = kernel(0, 0, 0);
    k.sync_tick(100);
    let mut ef = frame(SYS_BUF_ALLOC, 1, 50);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    let slot = ef.r0 as usize;
    assert_eq!(k.buffers().deadline(slot), Some(150));
}

#[test]
fn sync_tick_expire_timed_waits_uses_synced_tick() {
    use crate::sampling::PortDirection;
    use crate::syscall::SYS_QUEUING_RECV_TIMED;

    let mut k = kernel(0, 0, 0);
    let dst = k
        .queuing_mut()
        .create_port(PortDirection::Destination)
        .unwrap();

    // Sync tick to 100, then dispatch a timed recv with timeout=50.
    // The dispatch handler reads self.tick.get() (100) as current_tick,
    // so the receiver blocks with expiry = 100 + 50 = 150.
    k.sync_tick(100);
    let ptr = low32_buf(0);
    let r2 = (50u32 << 16) | 4;
    let mut ef = frame4(SYS_QUEUING_RECV_TIMED, dst as u32, r2, ptr as u32);
    // SAFETY: test-only dispatch on a valid ExceptionFrame.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
    assert_eq!(k.queuing().get(dst).unwrap().pending_receivers(), 1);

    // Expire at the synced deadline tick.
    k.expire_timed_waits::<8>(150);
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Ready
    );
}
