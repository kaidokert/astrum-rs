/* STM32F207ZG memory layout for kernel builds */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1M
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Kernel state section — placed at start of RAM */
SECTIONS
{
  .kernel_state (NOLOAD) : ALIGN(8) {
    *(.kernel_state .kernel_state.*)
  } > RAM
} INSERT BEFORE .data;
