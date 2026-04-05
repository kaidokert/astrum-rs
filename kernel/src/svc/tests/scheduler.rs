use super::*;
use crate::partition_core::AlignedStack1K;
use crate::syscall::SYS_GET_PARTITION_RUN_COUNT;

/// Build a 2-partition kernel (P0, P1) with round-robin schedule (2 ticks each).
/// Both partitions start in `Ready` state.  The schedule is already started and
/// P0 is set as the initial active partition.
fn kernel_2p() -> Kernel<'static, TestConfig> {
    let mut schedule = ScheduleTable::<4>::new();
    schedule.add(ScheduleEntry::new(0, 2)).unwrap();
    schedule.add(ScheduleEntry::new(1, 2)).unwrap();
    schedule.add_system_window(1).unwrap();
    schedule.start();

    let mut stk0 = AlignedStack1K::default();
    let mut stk1 = AlignedStack1K::default();
    let mems = [
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk0,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            pid(0),
        )
        .unwrap(),
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk1,
            0x0800_1001,
            MpuRegion::new(0x2000_1000, 4096, 0),
            pid(1),
        )
        .unwrap(),
    ];
    let mut k = kernel_from_ext(schedule, &mems);
    // Transition P0 to Running and set as active.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Running)
        .unwrap();
    k.active_partition = Some(0);
    k
}

/// Build a 3-partition kernel (P0, P1, P2) with 2-tick slots each.
/// P0 starts Running/active; P1 and P2 start Ready.
fn kernel_3p() -> Kernel<'static, TestConfig> {
    let mut schedule = ScheduleTable::<4>::new();
    schedule.add(ScheduleEntry::new(0, 2)).unwrap();
    schedule.add(ScheduleEntry::new(1, 2)).unwrap();
    schedule.add(ScheduleEntry::new(2, 2)).unwrap();
    schedule.add_system_window(1).unwrap();
    schedule.start();

    let mut stk0 = AlignedStack1K::default();
    let mut stk1 = AlignedStack1K::default();
    let mut stk2 = AlignedStack1K::default();
    let mems = [
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk0,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            pid(0),
        )
        .unwrap(),
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk1,
            0x0800_1001,
            MpuRegion::new(0x2000_1000, 4096, 0),
            pid(1),
        )
        .unwrap(),
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk2,
            0x0800_2001,
            MpuRegion::new(0x2000_2000, 4096, 0),
            pid(2),
        )
        .unwrap(),
    ];
    let mut k = kernel_from_ext(schedule, &mems);
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Running)
        .unwrap();
    k.active_partition = Some(0);
    k
}

/// Fault a partition: Ready -> Running -> Faulted (or Running -> Faulted).
fn fault_partition(k: &mut Kernel<'static, TestConfig>, pid: u8) {
    let pcb = k.partitions_mut().get_mut(pid as usize).unwrap();
    if pcb.state() == PartitionState::Ready {
        pcb.transition(PartitionState::Running).unwrap();
    }
    pcb.transition(PartitionState::Faulted).unwrap();
    assert_eq!(pcb.state(), PartitionState::Faulted);
}

// -------------------------------------------------------------------------
// advance_schedule_tick skips Faulted partitions
// -------------------------------------------------------------------------

#[test]
fn faulted_partition_returns_none_on_switch() {
    let mut k = kernel_2p();
    // Fault P1 (Ready -> Running -> Faulted).
    fault_partition(&mut k, 1);

    // P0 slot: 2 ticks. tick 1 = interior (None), tick 2 = boundary -> P1.
    assert_eq!(
        svc_scheduler::advance_schedule_tick(&mut k),
        ScheduleEvent::None,
    );
    // Tick 2: boundary to P1 slot, but P1 is Faulted -> None.
    assert_eq!(
        svc_scheduler::advance_schedule_tick(&mut k),
        ScheduleEvent::None,
        "Faulted partition must be skipped"
    );
    // active_partition must still be P0.
    assert_eq!(k.active_partition(), Some(0));
}

#[test]
fn faulted_partition_never_becomes_active() {
    let mut k = kernel_3p();
    // Fault P1.
    fault_partition(&mut k, 1);

    // Walk through 2 full major frames (12 ticks) and verify P1 is never active.
    for tick in 0..12 {
        svc_scheduler::advance_schedule_tick(&mut k);
        let ap = k.active_partition();
        assert!(
            ap != Some(1),
            "tick {tick}: active_partition must never be Faulted P1, got {ap:?}"
        );
    }
}

#[test]
fn faulted_skip_three_partition_schedule() {
    let mut k = kernel_3p();
    // Fault P1.
    fault_partition(&mut k, 1);

    // P0 slot (2 ticks): interior, then boundary -> P1 (Faulted, skipped).
    assert_eq!(
        svc_scheduler::advance_schedule_tick(&mut k),
        ScheduleEvent::None
    );
    assert_eq!(
        svc_scheduler::advance_schedule_tick(&mut k),
        ScheduleEvent::None,
        "P1 Faulted -> skipped"
    );
    // P1 slot interior tick.
    assert_eq!(
        svc_scheduler::advance_schedule_tick(&mut k),
        ScheduleEvent::None
    );
    // P1 slot boundary -> P2 (Ready -> switch).
    assert_eq!(
        svc_scheduler::advance_schedule_tick(&mut k),
        ScheduleEvent::PartitionSwitch(2),
    );
    assert_eq!(k.active_partition(), Some(2));
    // P2 slot interior.
    assert_eq!(
        svc_scheduler::advance_schedule_tick(&mut k),
        ScheduleEvent::None
    );
    // P2 slot boundary -> wrap to P0 (via system window if dynamic-mpu).
    let mut event = svc_scheduler::advance_schedule_tick(&mut k);
    if event == ScheduleEvent::SystemWindow {
        event = svc_scheduler::advance_schedule_tick(&mut k);
    }
    assert_eq!(event, ScheduleEvent::PartitionSwitch(0));
    assert_eq!(k.active_partition(), Some(0));
}

// -------------------------------------------------------------------------
// transition_outgoing_ready does not touch Faulted partitions
// -------------------------------------------------------------------------

#[test]
fn transition_outgoing_ready_preserves_faulted() {
    let mut k = kernel_2p();
    // P0 is Running and active. Fault it directly.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Faulted)
        .unwrap();
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Faulted
    );

    // transition_outgoing_ready must not try to move Faulted -> Ready.
    svc_scheduler::transition_outgoing_ready(&mut k);

    // P0 must still be Faulted (not Ready, not panicked).
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Faulted,
        "Faulted partition must remain Faulted after transition_outgoing_ready"
    );
}

#[test]
fn faulted_active_not_transitioned_on_switch() {
    let mut k = kernel_3p();
    // P0 is Running, active. Fault it.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Faulted)
        .unwrap();

    // Tick through P0 slot to reach P1 boundary.
    // tick 1: interior.
    svc_scheduler::advance_schedule_tick(&mut k);
    // tick 2: boundary -> P1 (Ready -> switch). This calls
    // transition_outgoing_ready which must skip the Faulted P0.
    let event = svc_scheduler::advance_schedule_tick(&mut k);
    assert_eq!(event, ScheduleEvent::PartitionSwitch(1));
    assert_eq!(k.active_partition(), Some(1));
    // P0 must still be Faulted (not moved to Ready).
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Faulted
    );
}

#[test]
fn run_count_increments_on_partition_switch() {
    let mut k = kernel_2p();
    // P0 is active/running, run_count starts at 0
    assert_eq!(k.partitions().get(0).unwrap().run_count(), 0);
    assert_eq!(k.partitions().get(1).unwrap().run_count(), 0);

    // Tick through P0 slot (2 ticks), switches to P1
    svc_scheduler::advance_schedule_tick(&mut k); // tick 1: interior
    let event = svc_scheduler::advance_schedule_tick(&mut k); // tick 2: switch to P1
    assert_eq!(event, ScheduleEvent::PartitionSwitch(1));
    assert_eq!(k.partitions().get(1).unwrap().run_count(), 1);
    assert_eq!(k.partitions().get(0).unwrap().run_count(), 0); // P0 not re-entered yet

    // Tick through P1 slot (2 ticks), then system window (1 tick), then P0
    svc_scheduler::advance_schedule_tick(&mut k); // tick 1 of P1
    svc_scheduler::advance_schedule_tick(&mut k); // tick 2 of P1 -> SystemWindow
    svc_scheduler::advance_schedule_tick(&mut k); // system window tick -> P0
                                                  // The schedule is: P0(2), P1(2), SW(1) — 5 ticks total
                                                  // tick3=interior P1, tick4=SW, tick5=P0 switch
    assert_eq!(k.partitions().get(0).unwrap().run_count(), 1);
}

#[test]
fn run_count_not_incremented_for_skipped_partition() {
    let mut k = kernel_2p();
    // Move P1 to Faulted so it gets skipped
    k.partitions_mut()
        .get_mut(1)
        .unwrap()
        .transition(PartitionState::Faulted)
        .unwrap();

    // Tick through P0 slot, would switch to P1 but P1 is Faulted
    svc_scheduler::advance_schedule_tick(&mut k);
    let event = svc_scheduler::advance_schedule_tick(&mut k);
    assert_eq!(event, ScheduleEvent::None); // skipped
    assert_eq!(k.partitions().get(1).unwrap().run_count(), 0); // not incremented
}

#[test]
fn dispatch_get_partition_run_count_matches_pcb() {
    let mut k = kernel_2p();
    // (1) P0 is Running+active, P1 is Ready.  Neither has been switched to yet.
    assert_eq!(k.active_partition(), Some(0));

    // (2) Advance ticks until P1 switch.
    // Schedule: P0(2 ticks), P1(2 ticks), SW(1 tick).
    // tick 1: interior of P0
    svc_scheduler::advance_schedule_tick(&mut k);
    // tick 2: boundary -> P1 switch
    let event = svc_scheduler::advance_schedule_tick(&mut k);
    assert_eq!(event, ScheduleEvent::PartitionSwitch(1));
    assert_eq!(k.active_partition(), Some(1));

    // Verify PCB run_count directly.
    assert_eq!(k.partitions().get(1).unwrap().run_count(), 1);
    assert_eq!(k.partitions().get(0).unwrap().run_count(), 0);

    // (3) Dispatch GetPartitionRunCount for P1 and assert r0 == 1.
    let mut ef = frame(SYS_GET_PARTITION_RUN_COUNT, 1, 0);
    // SAFETY: `ef` is a valid stack-allocated ExceptionFrame; `k` is a properly
    // initialized test Kernel with valid partition tables.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1, "dispatch run_count for P1 must be 1");

    // (4) Dispatch GetPartitionRunCount for P0 and assert r0 == 0.
    let mut ef = frame(SYS_GET_PARTITION_RUN_COUNT, 0, 0);
    // SAFETY: See above.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "dispatch run_count for P0 must be 0");
}

// ── Intra-partition thread schedule tests ─────────────────────────────

#[cfg(feature = "intra-threads")]
use rtos_traits::ids::ThreadId;
#[cfg(feature = "intra-threads")]
use rtos_traits::thread::{ThreadControlBlock, ThreadState};

/// Build a 2-partition kernel where P0 has 2 threads with distinct SPs.
#[cfg(feature = "intra-threads")]
/// Returns (kernel, thread_0_sp, thread_1_sp).
fn kernel_2p_multithread() -> (Kernel<'static, TestConfig>, u32, u32) {
    let mut schedule = ScheduleTable::<4>::new();
    schedule.add(ScheduleEntry::new(0, 4)).unwrap();
    schedule.add(ScheduleEntry::new(1, 4)).unwrap();
    schedule.add_system_window(1).unwrap();
    schedule.start();

    let mut stk0 = AlignedStack1K::default();
    let mut stk1 = AlignedStack1K::default();
    let mems = [
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk0,
            0x0800_0001,
            MpuRegion::new(0x2000_0000, 4096, 0),
            pid(0),
        )
        .unwrap(),
        ExternalPartitionMemory::from_aligned_stack(
            &mut stk1,
            0x0800_1001,
            MpuRegion::new(0x2000_1000, 4096, 0),
            pid(1),
        )
        .unwrap(),
    ];
    let mut k = kernel_from_ext(schedule, &mems);

    // Transition P0 to Running and set as active.
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .transition(PartitionState::Running)
        .unwrap();
    k.active_partition = Some(0);

    // Record thread 0's original SP (set by init_main_thread during PCB construction).
    let t0_sp = k
        .partitions()
        .get(0)
        .unwrap()
        .thread_table()
        .get(ThreadId::new(0))
        .unwrap()
        .stack_pointer;

    // Add a second thread to P0's thread table with a distinct SP.
    let t1_sp = 0x2000_0200;
    let tcb1 = ThreadControlBlock {
        stack_pointer: t1_sp,
        id: ThreadId::new(1),
        state: ThreadState::Ready,
        priority: 1,
        stack_base: 0x2000_0000,
        stack_size: 512,
        entry_point: 0x0800_0100,
        r0_arg: 0,
    };
    k.partitions_mut()
        .get_mut(0)
        .unwrap()
        .thread_table_mut()
        .add_thread(tcb1)
        .unwrap();

    // Set partition_sp[0] to thread 0's SP (simulating PendSV restore).
    k.set_sp(0, t0_sp);

    (k, t0_sp, t1_sp)
}

#[cfg(feature = "intra-threads")]
#[test]
fn intra_thread_schedule_updates_partition_sp() {
    let (mut k, t0_sp, t1_sp) = kernel_2p_multithread();

    // Before advance: partition_sp[0] == thread 0's SP.
    assert_eq!(k.get_sp(0), Some(t0_sp));

    // Advance intra-thread schedule: should switch from thread 0 → thread 1.
    let switched = crate::svc::scheduler::advance_intra_thread_schedule(&mut k);
    assert!(switched, "must return true when thread changes");

    // After advance: partition_sp[0] should be thread 1's SP.
    assert_eq!(
        k.get_sp(0),
        Some(t1_sp),
        "partition_sp must reflect incoming thread's SP"
    );

    // Outgoing thread 0's TCB.stack_pointer should have been saved.
    let saved_sp = k
        .partitions()
        .get(0)
        .unwrap()
        .thread_table()
        .get(ThreadId::new(0))
        .unwrap()
        .stack_pointer;
    assert_eq!(
        saved_sp, t0_sp,
        "outgoing thread's TCB must have partition_sp saved"
    );
}

#[cfg(feature = "intra-threads")]
#[test]
fn intra_thread_schedule_single_thread_no_change() {
    // P1 has only 1 thread (default). advance_intra_thread_schedule should be a no-op.
    let mut k = kernel_2p();
    let original_sp = k.get_sp(0).unwrap();

    let switched = crate::svc::scheduler::advance_intra_thread_schedule(&mut k);
    assert!(!switched, "single-threaded partition must return false");

    assert_eq!(
        k.get_sp(0),
        Some(original_sp),
        "single-threaded partition must not change partition_sp"
    );
}

#[cfg(feature = "intra-threads")]
#[test]
fn intra_thread_schedule_round_robin_cycle() {
    let (mut k, t0_sp, t1_sp) = kernel_2p_multithread();

    // Advance 1: thread 0 → thread 1
    let switched = crate::svc::scheduler::advance_intra_thread_schedule(&mut k);
    assert!(switched, "first advance must switch threads");
    assert_eq!(k.get_sp(0), Some(t1_sp));

    // Advance 2: thread 1 → thread 0
    let switched = crate::svc::scheduler::advance_intra_thread_schedule(&mut k);
    assert!(switched, "second advance must switch back");
    assert_eq!(
        k.get_sp(0),
        Some(t0_sp),
        "round-robin must cycle back to thread 0"
    );
}

#[cfg(feature = "intra-threads")]
#[test]
fn intra_thread_schedule_no_active_partition_is_noop() {
    let (mut k, _t0_sp, _t1_sp) = kernel_2p_multithread();
    let sp_before = k.get_sp(0).unwrap();
    k.active_partition = None;

    let switched = crate::svc::scheduler::advance_intra_thread_schedule(&mut k);
    assert!(!switched, "no active partition must return false");

    // partition_sp unchanged.
    assert_eq!(k.get_sp(0), Some(sp_before));
}

#[cfg(feature = "intra-threads")]
#[test]
fn intra_thread_schedule_saves_outgoing_sp_before_advance() {
    let (mut k, _t0_sp, _t1_sp) = kernel_2p_multithread();

    // Set partition_sp to a distinct value (simulating PSP drift during execution).
    let drifted_sp = 0x2000_0ABC;
    k.set_sp(0, drifted_sp);

    let switched = crate::svc::scheduler::advance_intra_thread_schedule(&mut k);
    assert!(switched, "must switch when multiple runnable threads exist");

    // Outgoing thread 0's TCB should have the drifted SP saved.
    let saved = k
        .partitions()
        .get(0)
        .unwrap()
        .thread_table()
        .get(ThreadId::new(0))
        .unwrap()
        .stack_pointer;
    assert_eq!(
        saved, drifted_sp,
        "outgoing thread must save the actual partition_sp value"
    );
}
