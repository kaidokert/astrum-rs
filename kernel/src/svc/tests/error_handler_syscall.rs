use super::*;
use crate::error_handler::{ErrorStatus, FaultKind};
use crate::syscall::{SYS_GET_ERROR_STATUS, SYS_REGISTER_ERROR_HANDLER};

#[test]
fn dispatch_register_error_handler_stores_addr() {
    let mut ef = frame(SYS_REGISTER_ERROR_HANDLER, 0x0800_1001, 0);
    let mut t = tbl();
    dispatch_syscall(&mut ef, &mut t, 0);
    assert_eq!(ef.r0, 0);
    assert_eq!(t.get(0).unwrap().error_handler(), Some(0x0800_1001));
    assert_eq!(t.get(1).unwrap().error_handler(), None);
}

#[test]
fn kernel_register_and_overwrite() {
    let mut k = kernel(0, 0, 0);
    k.current_partition = 0;
    let mut ef = frame(SYS_REGISTER_ERROR_HANDLER, 0x0800_1001, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    assert_eq!(
        k.partitions().get(0).unwrap().error_handler(),
        Some(0x0800_1001)
    );
    let mut ef2 = frame(SYS_REGISTER_ERROR_HANDLER, 0x0800_2001, 0);
    unsafe { k.dispatch(&mut ef2) };
    assert_eq!(ef2.r0, 0);
    assert_eq!(
        k.partitions().get(0).unwrap().error_handler(),
        Some(0x0800_2001)
    );
}

#[test]
fn kernel_get_error_status_none_and_after_clear() {
    let mut k = kernel(0, 0, 0);
    k.current_partition = 0;
    let mut ef = frame(SYS_GET_ERROR_STATUS, 0, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    let es = ErrorStatus::new(FaultKind::UsageFault, 0, 0, 0x0001_0000, 0x0800_2000);
    k.set_last_error(0, es);
    k.clear_last_error(0);
    let mut ef2 = frame(SYS_GET_ERROR_STATUS, 0, 0);
    unsafe { k.dispatch(&mut ef2) };
    assert_eq!(ef2.r0, SvcError::InvalidResource.to_u32());
}

#[test]
fn kernel_get_error_status_packed_fields_and_isolation() {
    let mut k = kernel(0, 0, 0);
    let es = ErrorStatus::new(FaultKind::BusFault, 0, 0x4000_0000, 0x0200, 0x0800_1234);
    k.set_last_error(0, es);
    k.current_partition = 1;
    let mut ef = frame(SYS_GET_ERROR_STATUS, 0, 0);
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, SvcError::InvalidResource.to_u32());
    k.current_partition = 0;
    let mut ef2 = frame(SYS_GET_ERROR_STATUS, 0, 0);
    unsafe { k.dispatch(&mut ef2) };
    assert_eq!(ef2.r0, FaultKind::BusFault.as_u32() << 8);
    assert_eq!(ef2.r1, 0x4000_0000);
    assert_eq!(ef2.r2, 0x0200);
    assert_eq!(ef2.r3, 0x0800_1234);
}
