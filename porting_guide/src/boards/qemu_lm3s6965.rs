//! QEMU LM3S6965 stub board — no real LED or delay hardware.

use crate::{Board, NoDelay, NoLed};

/// QEMU LM3S6965EVB board (Cortex-M3 emulation target).
///
/// This board has no user LED and no hardware delay peripheral,
/// so both associated types are no-op stubs.
pub struct QemuBoard;

impl Board for QemuBoard {
    type Led = NoLed;
    type Delay = NoDelay;

    fn init() -> (Self::Led, Self::Delay) {
        (NoLed, NoDelay)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn qemu_board_led_is_no_led() {
        assert_eq!(
            core::mem::size_of::<<QemuBoard as Board>::Led>(),
            0,
            "QemuBoard LED should be zero-sized NoLed"
        );
    }

    #[test]
    fn qemu_board_delay_is_no_delay() {
        assert_eq!(
            core::mem::size_of::<<QemuBoard as Board>::Delay>(),
            0,
            "QemuBoard Delay should be zero-sized NoDelay"
        );
    }

    #[test]
    fn qemu_board_init_is_deterministic() {
        // Call init twice — both should succeed (no side effects).
        let (_led1, _delay1) = QemuBoard::init();
        let (_led2, _delay2) = QemuBoard::init();
    }
}
