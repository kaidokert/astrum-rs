use super::*;

// -------------------------------------------------------------------------
// Device I/O dispatch tests (batch 1)
// -------------------------------------------------------------------------

#[test]
fn dev_open_dispatch_valid_and_invalid() {
    use crate::syscall::SYS_DEV_OPEN;
    let mut k = kernel(0, 0, 0);
    // Open device 0 (UART-A) — should succeed
    let mut ef = frame(SYS_DEV_OPEN, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    // Open device 1 (UART-B) — should succeed
    let mut ef = frame(SYS_DEV_OPEN, 1, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    // Open invalid device 99 — should return InvalidResource
    let mut ef = frame(SYS_DEV_OPEN, 99, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
}

#[test]
fn dev_write_dispatch_routes_to_uart() {
    use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_WRITE};
    let (registry, uart_a, _) = default_registry();
    let mut k = kernel_with_registry(0, 0, 0, registry);
    let mut ef = frame(SYS_DEV_OPEN, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    let ptr = low32_buf(0);
    // SAFETY: ptr points to a valid mmap'd page.
    unsafe { core::ptr::copy_nonoverlapping([0xAA, 0xBB, 0xCC].as_ptr(), ptr, 3) };
    let mut ef = frame4(SYS_DEV_WRITE, 0, 3, ptr as u32);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 3);
    // SAFETY: uart_a points to a leaked VirtualUartBackend that is only
    // accessed mutably here (dispatch is complete, no aliasing).
    let ua = unsafe { &mut *uart_a };
    assert_eq!(ua.pop_tx(), Some(0xAA));
    assert_eq!(ua.pop_tx(), Some(0xBB));
    assert_eq!(ua.pop_tx(), Some(0xCC));
}

#[test]
fn dev_read_dispatch_routes_to_uart() {
    use crate::syscall::{SYS_DEV_OPEN, SYS_DEV_READ};
    let (registry, uart_a, _) = default_registry();
    let mut k = kernel_with_registry(0, 0, 0, registry);
    let mut ef = frame(SYS_DEV_OPEN, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    // SAFETY: uart_a points to a leaked VirtualUartBackend; no aliasing
    // because we only touch it here outside of dispatch.
    unsafe { &mut *uart_a }.push_rx(&[0xDE, 0xAD]);
    let ptr = low32_buf(0);
    let mut ef = frame4(SYS_DEV_READ, 0, 4, ptr as u32);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 2);
    // SAFETY: ptr is valid for 4096 bytes (mmap via low32_buf), 2 were written.
    let out = unsafe { core::slice::from_raw_parts(ptr, 2) };
    assert_eq!(out, &[0xDE, 0xAD]);
}

#[test]
fn dev_ioctl_dispatch_routes_to_uart() {
    use crate::syscall::{SYS_DEV_IOCTL, SYS_DEV_OPEN};
    use crate::virtual_uart::IOCTL_AVAILABLE;
    let (registry, _, uart_b) = default_registry();
    let mut k = kernel_with_registry(0, 0, 0, registry);
    // Open device 1
    let mut ef = frame(SYS_DEV_OPEN, 1, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    // Push some RX data into UART-B
    // SAFETY: uart_b points to a leaked VirtualUartBackend; no aliasing
    // because we only touch it here outside of dispatch.
    unsafe { &mut *uart_b }.push_rx(&[1, 2, 3]);
    // IOCTL_AVAILABLE should return 3
    let mut ef = frame4(SYS_DEV_IOCTL, 1, IOCTL_AVAILABLE, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 3);
}

#[test]
fn dev_invalid_id_returns_invalid_resource() {
    use crate::syscall::{SYS_DEV_IOCTL, SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_WRITE};
    let inv = SvcError::InvalidResource.to_u32();
    let inv_ptr = SvcError::InvalidPointer.to_u32();
    let mut k = kernel(0, 0, 0);
    // Open invalid device
    let mut ef = frame(SYS_DEV_OPEN, 99, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, inv);
    // Write to invalid device — pointer validation rejects first
    let mut ef = frame4(SYS_DEV_WRITE, 99, 1, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, inv_ptr);
    // Read from invalid device — pointer validation rejects first
    let mut ef = frame4(SYS_DEV_READ, 99, 4, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, inv_ptr);
    // Ioctl on invalid device
    let mut ef = frame4(SYS_DEV_IOCTL, 99, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, inv);
}

pub(super) fn low32_buf(page: usize) -> *mut u8 {
    crate::test_mmap::low32_buf(page)
}

#[test]
fn dev_close_after_open_returns_success() {
    use crate::hw_uart::HwUartBackend;
    use crate::syscall::{SYS_DEV_CLOSE, SYS_DEV_OPEN};
    use crate::uart_hal::UartRegs;
    let (mut registry, _, _) = default_registry();
    // Type annotation needed: const generic N cannot be inferred through dyn VirtualDevice.
    let hw: &mut HwUartBackend<8> =
        Box::leak(Box::new(HwUartBackend::new(5, UartRegs::new(0x4000_C000))));
    registry.add(hw).unwrap();
    let mut k = kernel_with_registry(0, 0, 0, registry);
    // Open hw_uart device 5 (registered in the registry)
    let mut ef = frame(SYS_DEV_OPEN, 5, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    // Close device 5 — should succeed
    let mut ef = frame(SYS_DEV_CLOSE, 5, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
}

#[test]
fn dev_close_without_open_returns_error() {
    use crate::hw_uart::HwUartBackend;
    use crate::syscall::SYS_DEV_CLOSE;
    use crate::uart_hal::UartRegs;
    let (mut registry, _, _) = default_registry();
    // Type annotation needed: const generic N cannot be inferred through dyn VirtualDevice.
    let hw: &mut HwUartBackend<8> =
        Box::leak(Box::new(HwUartBackend::new(5, UartRegs::new(0x4000_C000))));
    registry.add(hw).unwrap();
    let mut k = kernel_with_registry(0, 0, 0, registry);
    // Close device 5 without opening — HwUartBackend checks require_open,
    // so this returns OperationFailed.
    let mut ef = frame(SYS_DEV_CLOSE, 5, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::OperationFailed.to_u32());
}

#[test]
fn dev_close_invalid_device_returns_invalid_resource() {
    use crate::syscall::SYS_DEV_CLOSE;
    let mut k = kernel(0, 0, 0);
    // Close non-existent device 99
    let mut ef = frame(SYS_DEV_CLOSE, 99, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
}
