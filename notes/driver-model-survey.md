# Driver Model Survey

This document surveys the `embedded-hal` 1.0 trait ecosystem to inform the
design of a partition-facing hardware abstraction layer for the RTOS. The
kernel uses ARINC 653-inspired static partitioning with MPU isolation; driver
access from partitions must go through kernel-mediated trait objects (`dyn`)
so the kernel can enforce access control and scheduling policy.

Pin version: **embedded-hal 1.0.0** ([docs.rs](https://docs.rs/embedded-hal/1.0.0/embedded_hal/)).

---

## 1  embedded-hal 1.0 Blocking Trait Surface

### A  Trait Inventory

#### Error Architecture (the `ErrorType` pattern)

Every peripheral module (digital, i2c, spi, pwm) follows the same
three-layer error pattern:

```
ErrorKind enum  ←  Error trait  ←  ErrorType trait
```

Each module defines its own `ErrorType`, `Error`, and `ErrorKind`
independently:

```rust
// Per-module Error trait (digital, i2c, spi, pwm each define one)
pub trait Error: core::fmt::Debug {
    fn kind(&self) -> ErrorKind;
}

// Per-module ErrorType trait
pub trait ErrorType {
    type Error: Error;
}
```

Blanket impls exist for `&T` and `&mut T` where `T: ErrorType + ?Sized`.

The **delay** module is the exception: `DelayNs` has no `ErrorType`
supertrait — delays are infallible.

---

#### 1. `embedded_hal::digital::InputPin`

**Supertrait:** `ErrorType` (from `embedded_hal::digital`)

| Method | Kind | Signature |
|--------|------|-----------|
| `is_high` | Required | `fn is_high(&mut self) -> Result<bool, Self::Error>` |
| `is_low` | Required | `fn is_low(&mut self) -> Result<bool, Self::Error>` |

**Object-safe:** Yes (specify `Error` concretely, e.g. `dyn InputPin<Error = E>`).

---

#### 2. `embedded_hal::digital::OutputPin`

**Supertrait:** `ErrorType` (from `embedded_hal::digital`)

| Method | Kind | Signature |
|--------|------|-----------|
| `set_low` | Required | `fn set_low(&mut self) -> Result<(), Self::Error>` |
| `set_high` | Required | `fn set_high(&mut self) -> Result<(), Self::Error>` |
| `set_state` | Provided | `fn set_state(&mut self, state: PinState) -> Result<(), Self::Error>` |

Default `set_state` delegates to `set_low`/`set_high` via `match`.

Supporting type: `PinState { Low, High }`.

**Object-safe:** Yes.

---

#### 3. `embedded_hal::digital::StatefulOutputPin`

**Supertrait:** `OutputPin` (→ `ErrorType`)

| Method | Kind | Signature |
|--------|------|-----------|
| `is_set_high` | Required | `fn is_set_high(&mut self) -> Result<bool, Self::Error>` |
| `is_set_low` | Required | `fn is_set_low(&mut self) -> Result<bool, Self::Error>` |
| `toggle` | Provided | `fn toggle(&mut self) -> Result<(), Self::Error>` |

Default `toggle` reads driven state via `is_set_low()`, then calls
`set_state(PinState::from(was_low))`.

Digital module `ErrorKind`: `{ Other }` (`#[non_exhaustive]`).

**Object-safe:** Yes.

---

#### 4. `embedded_hal::i2c::I2c`

**Supertrait:** `ErrorType` (from `embedded_hal::i2c`)

**Generic:** `A: AddressMode = SevenBitAddress`
(`SevenBitAddress = u8`, `TenBitAddress = u16`; `AddressMode` is sealed.)

| Method | Kind | Signature |
|--------|------|-----------|
| `transaction` | Required | `fn transaction(&mut self, address: A, operations: &mut [Operation<'_>]) -> Result<(), Self::Error>` |
| `read` | Provided | `fn read(&mut self, address: A, read: &mut [u8]) -> Result<(), Self::Error>` |
| `write` | Provided | `fn write(&mut self, address: A, write: &[u8]) -> Result<(), Self::Error>` |
| `write_read` | Provided | `fn write_read(&mut self, address: A, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>` |

`Operation<'a>`: `Read(&'a mut [u8])` | `Write(&'a [u8])`.

Default `read`/`write`/`write_read` wrap single `Operation` variants in a
call to `transaction`.

I2C `ErrorKind`: `{ Bus, ArbitrationLoss, NoAcknowledge(NoAcknowledgeSource),
Overrun, Other }` (`#[non_exhaustive]`).

`NoAcknowledgeSource`: `{ Address, Data, Unknown }`.

**Object-safe:** Yes (specify `A` and `Error`, e.g.
`dyn I2c<SevenBitAddress, Error = E>`).

---

#### 5. `embedded_hal::spi::SpiBus`

**Supertrait:** `ErrorType` (from `embedded_hal::spi`)

**Generic:** `Word: Copy + 'static = u8`

| Method | Kind | Signature |
|--------|------|-----------|
| `read` | Required | `fn read(&mut self, words: &mut [Word]) -> Result<(), Self::Error>` |
| `write` | Required | `fn write(&mut self, words: &[Word]) -> Result<(), Self::Error>` |
| `transfer` | Required | `fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error>` |
| `transfer_in_place` | Required | `fn transfer_in_place(&mut self, words: &mut [Word]) -> Result<(), Self::Error>` |
| `flush` | Required | `fn flush(&mut self) -> Result<(), Self::Error>` |

All 5 methods are required; no provided defaults. `SpiBus` represents
exclusive access to clock+MOSI+MISO (no CS management).

**Object-safe:** Yes (specify `Word` and `Error`, e.g.
`dyn SpiBus<u8, Error = E>`).

---

#### 6. `embedded_hal::spi::SpiDevice`

**Supertrait:** `ErrorType` (from `embedded_hal::spi`)

**Generic:** `Word: Copy + 'static = u8`

| Method | Kind | Signature |
|--------|------|-----------|
| `transaction` | Required | `fn transaction(&mut self, operations: &mut [Operation<'_, Word>]) -> Result<(), Self::Error>` |
| `read` | Provided | `fn read(&mut self, buf: &mut [Word]) -> Result<(), Self::Error>` |
| `write` | Provided | `fn write(&mut self, buf: &[Word]) -> Result<(), Self::Error>` |
| `transfer` | Provided | `fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error>` |
| `transfer_in_place` | Provided | `fn transfer_in_place(&mut self, buf: &mut [Word]) -> Result<(), Self::Error>` |

SPI `Operation<'a, Word>`: `Read(&'a mut [Word])` | `Write(&'a [Word])` |
`Transfer(&'a mut [Word], &'a [Word])` | `TransferInPlace(&'a mut [Word])` |
`DelayNs(u32)`.

`SpiDevice` manages one device on a (possibly shared) bus. The
`transaction` implementation must lock the bus, assert CS, run all
operations, flush, deassert CS, then unlock.

SPI `ErrorKind`: `{ Overrun, ModeFault, FrameFormat, ChipSelectFault,
Other }` (`#[non_exhaustive]`).

**Object-safe:** Yes (specify `Word` and `Error`).

---

#### 7. `embedded_hal::delay::DelayNs`

**Supertrait:** None.

**Associated types:** None.

| Method | Kind | Signature |
|--------|------|-----------|
| `delay_ns` | Required | `fn delay_ns(&mut self, ns: u32)` |
| `delay_us` | Provided | `fn delay_us(&mut self, us: u32)` |
| `delay_ms` | Provided | `fn delay_ms(&mut self, ms: u32)` |

All methods are infallible (no `Result`). Default `delay_us`/`delay_ms`
loop in chunks to prevent `u32` overflow when converting to nanoseconds.

**Object-safe:** Yes — unconditionally. No associated types or generics
required. Usable directly as `dyn DelayNs`.

---

#### 8. `embedded_hal::pwm::SetDutyCycle`

**Supertrait:** `ErrorType` (from `embedded_hal::pwm`)

| Method | Kind | Signature |
|--------|------|-----------|
| `max_duty_cycle` | Required | `fn max_duty_cycle(&self) -> u16` |
| `set_duty_cycle` | Required | `fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error>` |
| `set_duty_cycle_fully_off` | Provided | `fn set_duty_cycle_fully_off(&mut self) -> Result<(), Self::Error>` |
| `set_duty_cycle_fully_on` | Provided | `fn set_duty_cycle_fully_on(&mut self) -> Result<(), Self::Error>` |
| `set_duty_cycle_fraction` | Provided | `fn set_duty_cycle_fraction(&mut self, num: u16, denom: u16) -> Result<(), Self::Error>` |
| `set_duty_cycle_percent` | Provided | `fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), Self::Error>` |

Note: `max_duty_cycle` takes `&self` (not `&mut self`) — the only
non-mutating method across all 8 traits.

PWM `ErrorKind`: `{ Other }` (`#[non_exhaustive]`).

**Object-safe:** Yes (specify `Error`).

---

### Object-Safety Summary

| Trait | Object-safe | Qualification needed |
|-------|:-----------:|----------------------|
| `InputPin` | Yes | `Error` |
| `OutputPin` | Yes | `Error` |
| `StatefulOutputPin` | Yes | `Error` |
| `I2c` | Yes | `A` + `Error` |
| `SpiBus` | Yes | `Word` + `Error` |
| `SpiDevice` | Yes | `Word` + `Error` |
| `DelayNs` | Yes | None |
| `SetDutyCycle` | Yes | `Error` |

**All 8 blocking traits are object-safe.** This is by design — the
embedded-hal 1.0 team deliberately ensured every blocking trait can be
used behind `dyn` dispatch.

---

### Partition-Facing vs Kernel-Internal Use

Because all blocking traits are object-safe, every trait is usable for
the partition-facing API via `dyn` dispatch across the SVC boundary:

- **Partition-facing (all 8 traits):** A partition calls
  `svc!(SYS_DEV_READ, ...)` which the kernel dispatches to
  `dyn I2c<Error = KernelI2cError>` (or similar) held in the kernel's
  device registry. The `dyn` trait object lives in kernel memory;
  partitions never hold raw hardware references.

- **Kernel-internal:** The kernel implements the concrete HAL driver
  (e.g. `Lm3s6965I2c`) which is stored behind a `dyn I2c` in the device
  registry. The kernel may also use `DelayNs` internally for hardware
  initialization sequences without exposing it to partitions.

Key design considerations for this RTOS:

| Concern | Approach |
|---------|----------|
| Error types across SVC | Define a kernel `DriverError` that implements each module's `Error` trait; partition sees a uniform error code via SVC return register |
| Generic parameters (`Word`, `AddressMode`) | Fix at the SVC boundary (e.g. `Word = u8`, `A = SevenBitAddress`); partitions do not choose generics |
| Ownership | Kernel owns all `dyn` trait objects; partition access is mediated by device ID + SVC |
| `&mut self` requirement | Kernel ensures exclusive access per time slot via the partition schedule — only one partition runs at a time on single-core Cortex-M |
