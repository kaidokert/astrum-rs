//! SysTick-driven scheduler tick and PendSV trigger.

use crate::kernel::KernelState;

/// ICSR register address (Interrupt Control and State Register).
pub const SCB_ICSR: u32 = 0xE000_ED04;
/// Bit 28: PENDSVSET — set PendSV pending.
pub const ICSR_PENDSVSET: u32 = 1 << 28;

/// Configure the SysTick timer with the given reload value.
#[cfg(not(test))]
pub fn configure_systick(syst: &mut cortex_m::peripheral::SYST, reload: u32) {
    use cortex_m::peripheral::syst::SystClkSource;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(reload);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();
}

/// Advance the schedule table by one tick. On partition switch, sets PendSV
/// pending and returns the new active partition id.
pub fn on_systick<const P: usize, const S: usize>(state: &mut KernelState<P, S>) -> Option<u8> {
    let next = state.advance_schedule_tick();
    if next.is_some() {
        #[cfg(not(test))]
        unsafe {
            core::ptr::write_volatile(SCB_ICSR as *mut u32, ICSR_PENDSVSET);
        }
    }
    next
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionConfig};
    use crate::scheduler::{ScheduleEntry, ScheduleTable};

    fn make_state() -> KernelState<4, 8> {
        let mut s = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 3)).unwrap();
        s.add(ScheduleEntry::new(1, 2)).unwrap();
        s.start();
        let r = MpuRegion::new(0, 4096, 0);
        KernelState::new(
            s,
            &[
                PartitionConfig {
                    id: 0,
                    entry_point: 0x1000,
                    stack_base: 0x2000_0000,
                    stack_size: 1024,
                    mpu_region: r,
                },
                PartitionConfig {
                    id: 1,
                    entry_point: 0x2000,
                    stack_base: 0x2000_1000,
                    stack_size: 1024,
                    mpu_region: r,
                },
            ],
        )
        .unwrap()
    }

    #[test]
    fn tick_drives_partition_switches() {
        let mut st = make_state();
        assert_eq!((on_systick(&mut st), st.active_partition()), (None, None));
        assert_eq!(on_systick(&mut st), None);
        assert_eq!(
            (on_systick(&mut st), st.active_partition()),
            (Some(1), Some(1))
        );
        assert_eq!(
            (on_systick(&mut st), st.active_partition()),
            (None, Some(1))
        );
        assert_eq!(
            (on_systick(&mut st), st.active_partition()),
            (Some(0), Some(0))
        );
    }

    #[test]
    fn multiple_major_frames() {
        let mut st = make_state();
        let mut sw = heapless::Vec::<u8, 8>::new();
        for _ in 0..10 {
            if let Some(p) = on_systick(&mut st) {
                sw.push(p).unwrap();
            }
        }
        assert_eq!(sw.as_slice(), &[1, 0, 1, 0]);
    }

    #[test]
    fn icsr_constants() {
        assert_eq!(SCB_ICSR, 0xE000_ED04);
        assert_eq!(ICSR_PENDSVSET, 1 << 28);
    }
}
