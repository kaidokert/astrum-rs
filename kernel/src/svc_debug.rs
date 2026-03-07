use crate::svc::SvcError;

/// Handle `DebugPrint` — forward pre-validated `data` to semihosting when enabled.
#[must_use]
pub fn handle_debug_print(data: &[u8]) -> u32 {
    #[cfg(feature = "log-semihosting")]
    {
        match core::str::from_utf8(data) {
            Ok(s) => {
                cortex_m_semihosting::hprint!("{}", s);
                0
            }
            Err(_) => SvcError::InvalidParameter.to_u32(),
        }
    }
    #[cfg(not(feature = "log-semihosting"))]
    {
        let _ = data;
        SvcError::NotImplemented.to_u32()
    }
}

/// Handle `DebugExit` — exit QEMU when the feature is enabled.
#[must_use]
pub fn handle_debug_exit(code: u32) -> u32 {
    #[cfg(feature = "qemu")]
    {
        match code {
            0 => crate::kexit!(success),
            _ => crate::kexit!(failure),
        }
        #[allow(unreachable_code)]
        0
    }
    #[cfg(not(feature = "qemu"))]
    {
        let _ = code;
        SvcError::NotImplemented.to_u32()
    }
}

/// Signal debug-pending for the calling partition.
#[cfg(feature = "partition-debug")]
#[must_use]
pub fn handle_debug_notify<const N: usize>(
    pt: &mut crate::partition::PartitionTable<N>,
    caller: usize,
) -> u32 {
    if let Some(pcb) = pt.get_mut(caller) {
        pcb.signal_debug_pending();
        0
    } else {
        SvcError::InvalidPartition.to_u32()
    }
}

/// Write pre-validated `data` into the partition's debug ring buffer.
#[cfg(feature = "partition-debug")]
#[must_use]
pub fn handle_debug_write<const N: usize>(
    pt: &mut crate::partition::PartitionTable<N>,
    caller: usize,
    data: &[u8],
) -> u32 {
    match pt.get_mut(caller) {
        None => SvcError::InvalidPartition.to_u32(),
        Some(pcb) => match pcb.debug_buffer() {
            None => SvcError::NotSupported.to_u32(),
            Some(buf) => buf.write(data) as u32,
        },
    }
}

#[cfg(all(test, not(feature = "log-semihosting"), not(feature = "qemu")))]
mod tests {
    use super::*;
    #[test]
    fn non_semihosting_paths() {
        let ni = SvcError::NotImplemented.to_u32();
        assert_eq!(handle_debug_print(b"hello"), ni);
        assert_eq!(handle_debug_print(b""), ni);
        assert_eq!(handle_debug_exit(0), ni);
        assert_eq!(handle_debug_exit(1), ni);
    }
}

#[cfg(all(test, feature = "partition-debug"))]
mod pd_tests {
    use super::*;
    use crate::debug::DebugRingBuffer;
    use crate::partition::{MpuRegion, PartitionControlBlock, PartitionState, PartitionTable};

    fn leak_buf() -> &'static DebugRingBuffer<64> {
        Box::leak(Box::new(DebugRingBuffer::<64>::new()))
    }

    #[test]
    fn notify_and_write() {
        let buf = leak_buf();
        let ip = SvcError::InvalidPartition.to_u32();
        let r = MpuRegion::new(0x2000_0000, 4096, 0);
        let mut pcb = PartitionControlBlock::new(0, 0x0800_0000, 0x2000_0000, 0x2000_1000, r);
        pcb.set_debug_buffer(buf);
        pcb.transition(PartitionState::Running).unwrap();
        let mut t = PartitionTable::<4>::new();
        t.add(pcb).unwrap();
        assert!(!t.get(0).unwrap().debug_pending());
        assert_eq!(handle_debug_notify(&mut t, 0), 0);
        assert!(t.get(0).unwrap().debug_pending());
        assert_eq!(handle_debug_notify(&mut t, 99), ip);
        assert_eq!(handle_debug_write(&mut t, 0, b"hello"), 5);
        let mut out = [0u8; 8];
        assert_eq!(buf.drain(&mut out, 8), 5);
        assert_eq!(&out[..5], b"hello");
        assert_eq!(handle_debug_write(&mut t, 99, b"x"), ip);
        // no-buffer path
        let r2 = MpuRegion::new(0x2000_0000, 4096, 0);
        let p2 = PartitionControlBlock::new(0, 0x0800_0000, 0x2000_0000, 0x2000_1000, r2);
        let mut t2 = PartitionTable::<4>::new();
        t2.add(p2).unwrap();
        let ns = SvcError::NotSupported.to_u32();
        assert_eq!(handle_debug_write(&mut t2, 0, b"x"), ns);
    }
}
