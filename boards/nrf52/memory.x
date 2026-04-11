/* nRF52833 (PCA10100) memory map */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 512K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Kernel state section — fixed address for assembly/linker access.
 * NOLOAD: not initialized by the C runtime (kernel initializes it).
 * Mirrors the layout used by the f429zi crate.
 */
SECTIONS
{
  .kernel_state (NOLOAD) : ALIGN(8)
  {
    __kernel_state_start = .;
    KEEP(*(.kernel_state .kernel_state.*))
    . = ALIGN(8);
    __kernel_state_end = .;
  } > RAM
} INSERT AFTER .bss;
