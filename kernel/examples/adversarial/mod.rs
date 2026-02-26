//! Shared test infrastructure for adversarial MPU/fault testing.

#![allow(dead_code)]

#[cfg(feature = "qemu")]
use cortex_m::asm;
#[cfg(feature = "qemu")]
use cortex_m::peripheral::scb::Exception;
use cortex_m::peripheral::SCB;
#[cfg(feature = "qemu")]
use cortex_m::register::{control, psp};
#[cfg(feature = "qemu")]
use cortex_m_semihosting::hprintln;
#[cfg(feature = "qemu")]
use kernel::mpu;

pub const MMFSR_MMARVALID: u8 = 1 << 7;
pub const MMFSR_DACCVIOL: u8 = 1 << 1;
pub const MMFSR_IACCVIOL: u8 = 1 << 0;

/// Outcome of an adversarial fault test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultOutcome {
    Faulted { mmfsr: u8, mmfar: u32 },
    NoFault,
    Error(&'static str),
}

/// Captured fault information from a MemManage exception.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FaultInfo {
    pub faulted: bool,
    pub mmfsr: u8,
    pub mmfar: u32,
}

impl FaultInfo {
    pub const fn new() -> Self {
        Self {
            faulted: false,
            mmfsr: 0,
            mmfar: 0,
        }
    }
    pub const fn is_addr_valid(&self) -> bool {
        self.mmfsr & MMFSR_MMARVALID != 0
    }
    pub const fn is_daccviol(&self) -> bool {
        self.mmfsr & MMFSR_DACCVIOL != 0
    }
    pub const fn is_iaccviol(&self) -> bool {
        self.mmfsr & MMFSR_IACCVIOL != 0
    }
    pub const fn to_outcome(self) -> FaultOutcome {
        if self.faulted {
            FaultOutcome::Faulted {
                mmfsr: self.mmfsr,
                mmfar: self.mmfar,
            }
        } else {
            FaultOutcome::NoFault
        }
    }
}

/// Report test result via semihosting and exit.
#[cfg(feature = "qemu")]
pub fn report_result(test_name: &str, outcome: FaultOutcome) -> ! {
    use cortex_m_semihosting::{debug, hprintln};
    match outcome {
        FaultOutcome::Faulted { mmfsr, mmfar } => {
            hprintln!(
                "{}: fault mmfsr={:#04x} mmfar={:#010x}",
                test_name,
                mmfsr,
                mmfar
            );
            hprintln!("{}: PASS", test_name);
            debug::exit(debug::EXIT_SUCCESS);
        }
        FaultOutcome::NoFault => {
            hprintln!("{}: FAIL - no fault", test_name);
            debug::exit(debug::EXIT_FAILURE);
        }
        FaultOutcome::Error(msg) => {
            hprintln!("{}: FAIL - {}", test_name, msg);
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    loop {
        asm::wfi();
    }
}

/// Read fault registers from SCB.
///
/// # Safety
/// Caller must ensure this is called from a context where SCB access is valid
/// (e.g., exception handler in privileged mode).
pub unsafe fn read_fault_registers() -> FaultInfo {
    // SAFETY: Accessing SCB registers via cortex-m abstraction. The caller guarantees
    // we are in a privileged context (e.g., exception handler) where SCB access is valid.
    let scb = &*SCB::PTR;
    let cfsr = scb.cfsr.read();
    let mmfsr = (cfsr & 0xFF) as u8;
    let mmfar = scb.mmfar.read();
    FaultInfo {
        faulted: mmfsr != 0,
        mmfsr,
        mmfar,
    }
}

/// Clear the MMFSR bits in CFSR by writing 1s to the bits to clear.
///
/// # Safety
/// Caller must ensure this is called from a context where SCB access is valid
/// (e.g., exception handler in privileged mode).
pub unsafe fn clear_mmfsr() {
    // SAFETY: Accessing SCB registers via cortex-m abstraction. The caller guarantees
    // we are in a privileged context (e.g., exception handler) where SCB access is valid.
    // Writing the MMFSR bits (lower 8 bits of CFSR) clears them (write-1-to-clear).
    let scb = &*SCB::PTR;
    let cfsr = scb.cfsr.read();
    scb.cfsr.write(cfsr & 0xFF);
}

/// Define a MemManage handler that captures fault info and runs an action.
#[macro_export]
macro_rules! define_memmanage_handler {
    // Simple form: just capture fault and report for the given test name
    ($fault_info:ident, $test_name:ident) => {
        define_memmanage_handler!($fault_info, {
            // SAFETY: Reading FAULT from the exception handler that just wrote it.
            // No other code accesses FAULT concurrently (interrupts of same/lower
            // priority are blocked during this exception).
            let info = unsafe { core::ptr::read_volatile(&raw const $fault_info) };

            // Verify we got a DACCVIOL (data access violation)
            let outcome = if info.is_daccviol() {
                adversarial::FaultOutcome::Faulted {
                    mmfsr: info.mmfsr,
                    mmfar: info.mmfar,
                }
            } else if info.faulted {
                adversarial::FaultOutcome::Error("fault without DACCVIOL")
            } else {
                adversarial::FaultOutcome::NoFault
            };

            adversarial::report_result($test_name, outcome);
        });
    };
    // Full form: capture fault and run custom action
    ($fault_info:ident, $action:expr) => {
        #[cortex_m_rt::exception]
        fn MemoryManagement() {
            let fault_info_ptr = &raw mut $fault_info;
            // SAFETY: This exception handler runs in privileged mode, so SCB access is valid.
            // The static $fault_info is declared by the caller and is only accessed from this
            // exception context. Writing via raw pointer is safe because we have exclusive
            // access during the exception (interrupts of same/lower priority are blocked).
            unsafe {
                let info = adversarial::read_fault_registers();
                core::ptr::write_volatile(fault_info_ptr, info);
                adversarial::clear_mmfsr();
            }
            $action
        }
    };
}

// ---------------------------------------------------------------------------
// Shared stack and test infrastructure
// ---------------------------------------------------------------------------

/// Stack size in words (256 words = 1 KiB).
pub const STACK_WORDS: usize = 256;

/// Stack size in bytes.
pub const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32;

/// Code region size for MPU.
pub const CODE_SIZE: u32 = 32 * 1024;

/// Aligned stack for partition use (1 KiB, aligned to 1024 for MPU).
#[repr(C, align(1024))]
pub struct AlignedStack(pub [u32; STACK_WORDS]);

impl AlignedStack {
    pub const fn new() -> Self {
        Self([0; STACK_WORDS])
    }

    /// Get the base address of this stack.
    pub fn base(&self) -> u32 {
        (self as *const Self).cast::<u32>() as u32
    }

    /// Get the top address of this stack (base + size).
    pub fn top(&self) -> u32 {
        self.base() + STACK_SIZE
    }
}

/// Configure MPU for a single partition: only allow access to that partition's stack.
///
/// R0: code RX (flash 0x0) — priv+unpriv read-only
/// R1: data RW (partition stack) — full access, XN
///
/// Any address outside these regions will fault for unprivileged access.
#[cfg(feature = "qemu")]
fn configure_single_partition_mpu(mpu_periph: &cortex_m::peripheral::MPU, data_base: u32) {
    // SAFETY: single-core, interrupts disabled — exclusive MPU access.
    unsafe { mpu_periph.ctrl.write(0) };
    asm::dsb();
    asm::isb();

    // R0: code RX — covers flash binary (priv+unpriv read-only)
    let code_sf = mpu::encode_size(CODE_SIZE).expect("code size");
    let code_rbar = mpu::build_rbar(0x0000_0000, 0).expect("code rbar");
    let code_rasr = mpu::build_rasr(code_sf, mpu::AP_RO_RO, false, (false, false, false));
    mpu::configure_region(mpu_periph, code_rbar, code_rasr);

    // R1: data RW — partition stack (full access, XN)
    let data_sf = mpu::encode_size(STACK_SIZE).expect("data size");
    let data_rbar = mpu::build_rbar(data_base, 1).expect("data rbar");
    let data_rasr = mpu::build_rasr(data_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    mpu::configure_region(mpu_periph, data_rbar, data_rasr);

    // Enable MPU with PRIVDEFENA.
    // SAFETY: regions programmed; barriers ensure visibility.
    unsafe { mpu_periph.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    asm::dsb();
    asm::isb();
}

/// Drop to unprivileged mode using the given stack.
///
/// # Safety
/// The stack must be valid and accessible (MPU configured).
#[cfg(feature = "qemu")]
unsafe fn drop_to_unprivileged(stack_top: u32) {
    psp::write(stack_top);
    let mut ctrl = control::read();
    ctrl.set_spsel(control::Spsel::Psp);
    ctrl.set_npriv(control::Npriv::Unprivileged);
    control::write(ctrl);
}

/// Run an "other stack" test: partition 0 attempts to access partition 1's stack.
///
/// This sets up MPU, enables MemManage, drops to unprivileged mode, then
/// calls the provided closure to perform the access that should fault.
///
/// # Safety
/// - `p0_stack` and `p1_stack` must point to valid, non-overlapping `AlignedStack` instances.
/// - The `access_fn` closure should perform a memory access to `target_addr` that triggers a fault.
#[cfg(feature = "qemu")]
pub unsafe fn run_other_stack_test<F>(
    test_name: &str,
    p: &mut cortex_m::Peripherals,
    p0_stack: *const AlignedStack,
    p1_stack: *const AlignedStack,
    access_fn: F,
) -> !
where
    F: FnOnce(u32),
{
    hprintln!("{}: start", test_name);

    // Enable MemManage fault handler.
    p.SCB.enable(Exception::MemoryManagement);

    // Get stack addresses.
    let p0_stack_base = (*p0_stack).base();
    let p0_stack_top = (*p0_stack).top();
    let p1_stack_base = (*p1_stack).base();
    let p1_stack_top = (*p1_stack).top();

    // Target address: middle of partition 1's stack.
    let target_addr = p1_stack_base + STACK_SIZE / 2;

    hprintln!(
        "  P0 stack: {:#010x} - {:#010x}",
        p0_stack_base,
        p0_stack_top
    );
    hprintln!(
        "  P1 stack: {:#010x} - {:#010x}",
        p1_stack_base,
        p1_stack_top
    );
    hprintln!("  target (P1): {:#010x}", target_addr);

    // Verify stacks don't overlap (sanity check).
    assert!(
        p0_stack_top <= p1_stack_base || p1_stack_top <= p0_stack_base,
        "stacks must not overlap"
    );

    // Configure MPU for partition 0 before dropping privileges.
    configure_single_partition_mpu(&p.MPU, p0_stack_base);

    hprintln!("  MPU configured for P0, dropping to unprivileged...");

    // Drop to unprivileged mode.
    drop_to_unprivileged(p0_stack_top);

    // Now unprivileged on PSP, running as "partition 0".
    // No semihosting here — would fault before reaching the intentional access.
    //
    // Perform the access that should fault.
    access_fn(target_addr);

    // Should never reach here — if we do, no fault occurred.
    loop {
        asm::nop();
    }
}

/// Run a kernel-address access test: configure a single partition, drop to
/// unprivileged mode, then call the provided closure to perform an access
/// that should fault.
///
/// # Safety
/// - `stack` must point to a valid `AlignedStack` instance.
/// - `access_fn` should perform a memory access to `target_addr` that triggers a fault.
#[cfg(feature = "qemu")]
pub unsafe fn run_kernel_access_test<F>(
    test_name: &str,
    target_addr: u32,
    p: &mut cortex_m::Peripherals,
    stack: *const AlignedStack,
    access_fn: F,
) -> !
where
    F: FnOnce(),
{
    hprintln!("{}: start", test_name);

    // Enable MemManage fault handler.
    p.SCB.enable(Exception::MemoryManagement);

    // Get stack addresses.
    let stack_base = (*stack).base();
    let stack_top = (*stack).top();

    hprintln!("  stack: {:#010x} - {:#010x}", stack_base, stack_top);
    hprintln!("  target kernel addr: {:#010x}", target_addr);

    // Configure MPU before dropping privileges.
    configure_single_partition_mpu(&p.MPU, stack_base);

    hprintln!("  MPU configured, dropping to unprivileged...");

    // Drop to unprivileged mode.
    drop_to_unprivileged(stack_top);

    // Now unprivileged on PSP.
    // Perform the access that should fault.
    access_fn();

    // Should never reach here — if we do, no fault occurred.
    loop {
        asm::nop();
    }
}

/// MPU configuration for two-partition adversarial test setup.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdversarialMpuConfig {
    pub code_base: u32,
    pub code_size: u32,
    pub adversarial_data_base: u32,
    pub adversarial_data_size: u32,
    pub observer_data_base: u32,
    pub observer_data_size: u32,
}

/// Error type for invalid MPU configuration parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpuConfigError {
    /// Code size must be a power of two and >= 32 bytes.
    InvalidCodeSize,
    /// Data size must be a power of two and >= 32 bytes.
    InvalidDataSize,
}

impl AdversarialMpuConfig {
    /// Create a new MPU configuration.
    ///
    /// Returns an error if:
    /// - `code_size` is not a power of two or is less than 32 bytes
    /// - `data_size` is not a power of two or is less than 32 bytes
    pub fn new(
        code_size: u32,
        data_size: u32,
        adv_base: u32,
        obs_base: u32,
    ) -> Result<Self, MpuConfigError> {
        if !code_size.is_power_of_two() || code_size < 32 {
            return Err(MpuConfigError::InvalidCodeSize);
        }
        if !data_size.is_power_of_two() || data_size < 32 {
            return Err(MpuConfigError::InvalidDataSize);
        }
        Ok(Self {
            code_base: 0,
            code_size,
            adversarial_data_base: adv_base,
            adversarial_data_size: data_size,
            observer_data_base: obs_base,
            observer_data_size: data_size,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fault_outcome_derives() {
        let f = FaultOutcome::Faulted {
            mmfsr: 0x82,
            mmfar: 0x2000_F000,
        };
        assert!(format!("{:?}", f).contains("Faulted")); // Debug
        assert_eq!(f.clone(), f); // Clone
        let c = f;
        assert_eq!(f, c); // Copy + PartialEq
        assert_ne!(FaultOutcome::NoFault, f);
    }

    #[test]
    fn fault_info_methods() {
        let info = FaultInfo::new();
        assert!(!info.faulted && !info.is_addr_valid());
        assert_eq!(info.to_outcome(), FaultOutcome::NoFault);
        let info2 = FaultInfo {
            faulted: true,
            mmfsr: MMFSR_DACCVIOL | MMFSR_MMARVALID,
            mmfar: 0x1000,
        };
        assert!(info2.is_daccviol() && info2.is_addr_valid() && !info2.is_iaccviol());
    }

    #[test]
    fn adversarial_mpu_config_new() {
        let c = AdversarialMpuConfig::new(32 * 1024, 1024, 0x2000_0000, 0x2000_0400).unwrap();
        assert_eq!(c.code_size, 32 * 1024);
        assert_eq!(c.adversarial_data_base, 0x2000_0000);
    }

    #[test]
    fn adversarial_mpu_config_invalid_code_size() {
        let result = AdversarialMpuConfig::new(1000, 1024, 0x2000_0000, 0x2000_0400);
        assert_eq!(result, Err(MpuConfigError::InvalidCodeSize));
    }

    #[test]
    fn adversarial_mpu_config_too_small_data() {
        let result = AdversarialMpuConfig::new(1024, 16, 0x2000_0000, 0x2000_0400);
        assert_eq!(result, Err(MpuConfigError::InvalidDataSize));
    }
}
