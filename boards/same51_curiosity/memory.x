MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 1M
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

SECTIONS
{
  .kernel_state (NOLOAD) : ALIGN(4)
  {
    __kernel_state_start = .;
    KEEP(*(.kernel_state .kernel_state.*))
    . = ALIGN(4);
    __kernel_state_end = .;
  } > RAM
} INSERT AFTER .bss;
