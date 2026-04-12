# Board Status & Kernel Feature Matrix

## Hardware Boards

| Board | MCU | Core | RAM |
|-------|-----|------|-----|
| NUCLEO-F429ZI | STM32F429ZI | Cortex-M4F | 256 KB |
| PCA10100 DK | nRF52833 | Cortex-M4F | 128 KB |
| NUCLEO-F207ZG | STM32F207ZG | Cortex-M3 | 128 KB |
| SAME51 Curiosity | ATSAME51J20A | Cortex-M4F | 256 KB |

The default workspace (`Cargo.toml` at repo root) builds only
`boards/lm3s` (QEMU target). The hardware board crates above are
excluded from the default workspace but can be built individually
from their own directory.

## Kernel Feature Matrix

`yes` = verified on hardware. `--` = not yet attempted. `n/a` = hardware can't support it.

| Capability | F429ZI | nRF52 | F207 | SAME51 |
|---|---|---|---|---|
| **Kernel boot / context switch** | 19/19 hw-tests | 10/10 hw-tests | yes | 9/10 |
| **Semaphore / mutex / events** | yes | yes | -- | yes |
| **Message queues** | yes | yes | -- | yes |
| **Queuing ports** | yes | yes | -- | yes |
| **Sampling ports** | yes | yes | yes (stochastic) | yes |
| **Blackboard** | yes | yes | -- | yes |
| **SYS_SLEEP_TICKS** | yes | -- | -- | yes |
| **Partition debug (dprint)** | yes | -- | -- | yes |
| **MPU static** | yes | yes | yes | yes |
| **MPU dynamic** | yes | -- | yes | yes |
| **Buffer pool (lend/revoke)** | yes | -- | -- | yes |
| **Buffer pool (transfer)** | yes | -- | -- | yes |
| **DMA + buffer pool** | yes | -- | -- | yes |
| **DMA UART** | yes | -- | -- | yes |
| **DMA full-duplex echo** | yes | -- | -- | yes |
| **UART-to-USB pipeline** | yes | -- | -- | -- |
| **Hardware IRQ -> partition** | yes | yes | -- | yes |
| **Split-ISR (Model B)** | yes | yes | -- | -- |
| **Virtual Device (Model C)** | yes | -- | -- | yes |
| **SYS_DEV_IOCTL + CLOSE** | yes | -- | -- | -- |
| **USB CDC (partition)** | yes | -- | -- | yes |
| **FPU context save/restore** | yes | yes | n/a (M3) | yes |
| **Partition fault survival** | yes | -- | -- | -- |
| **Warm restart (non-FPU)** | -- | -- | yes | -- |
| **Warm restart under FPU+MPU** | yes | -- | n/a (M3) | -- |
| **Stochastic fault injection** | yes | -- | yes | -- |
| **Adversarial multi-attack** | yes | yes | -- | yes |
| **Privilege escalation (cpsid/msr)** | yes | -- | -- | -- |
| **sys_get_start_condition** | yes | -- | yes | -- |
| **All-faulted safe idle** | yes | yes | -- | yes |
| **Watchdog recovery** | yes | -- | -- | -- |
| **IPC stress (>10K msg/s)** | yes | -- | -- | -- |
| **Tiny kernel (512B stack)** | -- | -- | yes | -- |
| **Stack overflow detection** | yes | -- | -- | -- |
| **Starvation detection** | yes | -- | -- | -- |
| **Panic tombstone** | yes | -- | -- | -- |
| **Health monitor introspection** | yes | -- | -- | -- |

## Per-Board Notes

### STM32F429ZI — primary target, fully exercised

- **Feature flags:** kernel-example, kernel-mpu, kernel-mpu-hal, kernel-mpu-hal-fpu, kernel-irq, kernel-irq-mpu-hal, kernel-usb, kernel-usb-mpu, kernel-fpu
- ~55 examples spanning all kernel subsystems
- DMA pipeline: USART3 VCP + DMA1 + buffer pool + USB OTG CDC

### nRF52833 — IPC + FPU + MPU + UARTE

- **Feature flags:** bare, kernel-example, kernel-fpu, kernel-mpu, kernel-mpu-hal, kernel-irq
- 15 examples: blinky, rtt_hello, systick, mpu_info + 7 kernel IPC demos + hal_gpio_blink_mpu + fpu_context_test + adversarial_test + uarte_irq_partition

### STM32F207ZG — Cortex-M3 smoke + stochastic testing

- Cortex-M3 (no FPU); useful for testing M3-specific behaviour
- Stochastic fault injection and tiny kernel (512B stack) validated here

### SAME51 Curiosity — DMA + USB + IPC

- 7 IPC demos + 3 UART DMA + 3 USB CDC + fpu_context + adversarial

## QEMU-only (not yet on any hardware)

- `sys_queuing_status` — port occupancy query
- `sys_dev_read_timed` — timed device read
- `sys_query_bottom_half` — ISR work profiling
- `sys_debug_exit` — semihosting exit (QEMU-only concept)
