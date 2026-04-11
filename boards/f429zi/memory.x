/* STM32F429ZI memory layout for master branch kernel */
MEMORY
{
  /* STM32F429ZI: 2MB Flash, 256KB RAM (192KB main + 64KB CCM) */
  FLASH : ORIGIN = 0x08000000, LENGTH = 2M
  RAM : ORIGIN = 0x20000000, LENGTH = 192K
}

/* Kernel state section — reserves a fixed address for assembly access.
 * NOLOAD: not initialized by the C runtime (kernel initializes it).
 * ALIGN(4): 4-byte alignment required for Cortex-M word access.
 *
 * Symbols defined:
 *   __kernel_state_start — start address of kernel state region
 *   __kernel_state_end   — end address of kernel state region
 *
 * Usage: Assembly can load the kernel pointer directly via:
 *   ldr r0, =__kernel_state_start
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
