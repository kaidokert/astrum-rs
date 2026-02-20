MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Kernel state section — reserves a fixed address for assembly access.
 * NOLOAD: not initialized by the C runtime (kernel initializes it).
 * ALIGN(4096): must match repr(C, align(4096)) on KernelStorageBuffer so
 *   the linker-placed section satisfies the Rust type's alignment requirement.
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
  .kernel_state (NOLOAD) : ALIGN(4096)
  {
    __kernel_state_start = .;
    KEEP(*(.kernel_state .kernel_state.*))
    . = ALIGN(4);
    __kernel_state_end = .;
  } > RAM
} INSERT AFTER .bss;
