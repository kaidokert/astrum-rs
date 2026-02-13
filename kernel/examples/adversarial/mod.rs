//! Shared test infrastructure for adversarial MPU/fault testing.

#![allow(dead_code)]

#[cfg(feature = "qemu")]
use cortex_m::asm;
use cortex_m::peripheral::SCB;

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
