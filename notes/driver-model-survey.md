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

### B  embedded-hal-async 1.0 Trait Inventory

Pin version: **embedded-hal-async 1.0.0**
([docs.rs](https://docs.rs/embedded-hal-async/1.0.0/embedded_hal_async/)).

Mirrors blocking `embedded-hal`; error types re-exported from Part A.
All methods are `async fn`. **None of the async traits are
dyn-compatible** (`async fn` desugars to opaque `impl Future`).

#### 1. `embedded_hal_async::digital::Wait`

**Supertrait:** `ErrorType` (digital)

| Method | Kind | Signature |
|--------|------|-----------|
| `wait_for_high` | Required | `async fn wait_for_high(&mut self) -> Result<(), Self::Error>` |
| `wait_for_low` | Required | `async fn wait_for_low(&mut self) -> Result<(), Self::Error>` |
| `wait_for_rising_edge` | Required | `async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error>` |
| `wait_for_falling_edge` | Required | `async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error>` |
| `wait_for_any_edge` | Required | `async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error>` |

All 5 required. `wait_for_high`/`low` return immediately if already in
state; `*_edge` methods always wait for next transition.

#### 2. `embedded_hal_async::i2c::I2c`

**Supertrait:** `ErrorType` (i2c) | **Generic:** `A: AddressMode = SevenBitAddress`

| Method | Kind | Signature |
|--------|------|-----------|
| `transaction` | Required | `async fn transaction(&mut self, address: A, operations: &mut [Operation<'_>]) -> Result<(), Self::Error>` |
| `read` | Provided | `async fn read(&mut self, address: A, read: &mut [u8]) -> Result<(), Self::Error>` |
| `write` | Provided | `async fn write(&mut self, address: A, write: &[u8]) -> Result<(), Self::Error>` |
| `write_read` | Provided | `async fn write_read(&mut self, address: A, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>` |

Identical to blocking `I2c` except `async fn`.

#### 3. `embedded_hal_async::spi::SpiBus`

**Supertrait:** `ErrorType` (spi) | **Generic:** `Word: Copy + 'static = u8`

| Method | Kind | Signature |
|--------|------|-----------|
| `read` | Required | `async fn read(&mut self, words: &mut [Word]) -> Result<(), Self::Error>` |
| `write` | Required | `async fn write(&mut self, words: &[Word]) -> Result<(), Self::Error>` |
| `transfer` | Required | `async fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error>` |
| `transfer_in_place` | Required | `async fn transfer_in_place(&mut self, words: &mut [Word]) -> Result<(), Self::Error>` |
| `flush` | Required | `async fn flush(&mut self) -> Result<(), Self::Error>` |

All 5 required, matching blocking `SpiBus` (async fn only difference).

#### 4. `embedded_hal_async::spi::SpiDevice`

**Supertrait:** `ErrorType` (spi) | **Generic:** `Word: Copy + 'static = u8`

| Method | Kind | Signature |
|--------|------|-----------|
| `transaction` | Required | `async fn transaction(&mut self, operations: &mut [Operation<'_, Word>]) -> Result<(), Self::Error>` |
| `read` | Provided | `async fn read(&mut self, buf: &mut [Word]) -> Result<(), Self::Error>` |
| `write` | Provided | `async fn write(&mut self, buf: &[Word]) -> Result<(), Self::Error>` |
| `transfer` | Provided | `async fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error>` |
| `transfer_in_place` | Provided | `async fn transfer_in_place(&mut self, buf: &mut [Word]) -> Result<(), Self::Error>` |

Uses `embedded_hal::spi::Operation` (same enum as blocking).

#### 5. `embedded_hal_async::delay::DelayNs`

**Supertrait:** None. **Associated types:** None.

| Method | Kind | Signature |
|--------|------|-----------|
| `delay_ns` | Required | `async fn delay_ns(&mut self, ns: u32)` |
| `delay_us` | Provided | `async fn delay_us(&mut self, us: u32)` |
| `delay_ms` | Provided | `async fn delay_ms(&mut self, ms: u32)` |

Infallible. `async fn` alone prevents dyn dispatch (no generics needed).

---

### C  embedded-io-async Trait Inventory

Pin versions: **embedded-io-async 0.6.1** / **0.7.0**
([docs.rs 0.6](https://docs.rs/embedded-io-async/0.6.1/embedded_io_async/),
[0.7](https://docs.rs/embedded-io-async/0.7.0/embedded_io_async/)).
Both re-export `ErrorType`/`ErrorKind`/`SeekFrom` from `embedded-io`.

Error architecture: `ErrorType { type Error: Error }` where
`Error: Debug` provides `fn kind(&self) -> ErrorKind`. `ErrorKind`
(`#[non_exhaustive]`): `Other`, `NotFound`, `PermissionDenied`,
`ConnectionRefused`, `ConnectionReset`, `ConnectionAborted`,
`NotConnected`, `AddrInUse`, `AddrNotAvailable`, `BrokenPipe`,
`AlreadyExists`, `InvalidInput`, `InvalidData`, `TimedOut`,
`Interrupted`, `Unsupported`, `OutOfMemory`, `WriteZero`.

#### 1. `embedded_io_async::Read`

**Supertrait:** `ErrorType`

| Method | Kind | Signature |
|--------|------|-----------|
| `read` | Required | `async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>` |
| `read_exact` | Provided | `async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), ReadExactError<Self::Error>>` |

**Object-safe:** No — async fn.

#### 2. `embedded_io_async::Write`

**Supertrait:** `ErrorType`

| Method | Kind | 0.6 | 0.7 | Signature |
|--------|------|:---:|:---:|-----------|
| `write` | Required | R | R | `async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error>` |
| `flush` |  | P | **R** | `async fn flush(&mut self) -> Result<(), Self::Error>` |
| `write_all` | Provided | P | P | `async fn write_all(&mut self, buf: &[u8]) -> Result<(), Self::Error>` |

**0.7 breaking change:** `flush` promoted from provided to required.
**Object-safe:** No — async fn.

#### 3. `embedded_io_async::BufRead`

**Supertrait:** `ErrorType` (0.6) / `Read` (0.7)

| Method | Kind | Signature |
|--------|------|-----------|
| `fill_buf` | Required | `async fn fill_buf(&mut self) -> Result<&[u8], Self::Error>` |
| `consume` | Required | `fn consume(&mut self, amt: usize)` |

`consume` is synchronous. **Object-safe:** No — `fill_buf` is async.

#### 4. `embedded_io_async::ReadReady`

**Supertrait:** `ErrorType` (0.6) / `Read` (0.7)

| Method | Kind | Signature |
|--------|------|-----------|
| `read_ready` | Required | `fn read_ready(&mut self) -> Result<bool, Self::Error>` |

Synchronous method. **Object-safe (0.6):** Yes — usable as
`dyn ReadReady<Error = E>`. **Object-safe (0.7):** No — supertrait
`Read` has async fn.

#### 5. `embedded_io_async::WriteReady`

**Supertrait:** `ErrorType` (0.6) / `Write` (0.7)

| Method | Kind | Signature |
|--------|------|-----------|
| `write_ready` | Required | `fn write_ready(&mut self) -> Result<bool, Self::Error>` |

Synchronous method. **Object-safe (0.6):** Yes.
**Object-safe (0.7):** No — supertrait `Write` has async fn.

#### 6. `embedded_io_async::Seek`

**Supertrait:** `ErrorType`

| Method | Kind | Signature |
|--------|------|-----------|
| `seek` | Required | `async fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error>` |
| `rewind` | Provided | `async fn rewind(&mut self) -> Result<(), Self::Error>` |
| `stream_position` | Provided | `async fn stream_position(&mut self) -> Result<u64, Self::Error>` |

`SeekFrom`: `Start(u64)` | `End(i64)` | `Current(i64)`.
**Object-safe:** No — async fn.

---

### Consolidated Object-Safety Summary

| Crate | Trait | dyn-ok | Qualification |
|-------|-------|:------:|---------------|
| embedded-hal 1.0 | `InputPin` | **Yes** | `Error` |
| embedded-hal 1.0 | `OutputPin` | **Yes** | `Error` |
| embedded-hal 1.0 | `StatefulOutputPin` | **Yes** | `Error` |
| embedded-hal 1.0 | `I2c` | **Yes** | `A` + `Error` |
| embedded-hal 1.0 | `SpiBus` | **Yes** | `Word` + `Error` |
| embedded-hal 1.0 | `SpiDevice` | **Yes** | `Word` + `Error` |
| embedded-hal 1.0 | `DelayNs` | **Yes** | None |
| embedded-hal 1.0 | `SetDutyCycle` | **Yes** | `Error` |
| embedded-hal-async 1.0 | `Wait` | No | async fn |
| embedded-hal-async 1.0 | `I2c` (async) | No | async fn |
| embedded-hal-async 1.0 | `SpiBus` (async) | No | async fn |
| embedded-hal-async 1.0 | `SpiDevice` (async) | No | async fn |
| embedded-hal-async 1.0 | `DelayNs` (async) | No | async fn |
| embedded-io-async 0.6 | `Read` | No | async fn |
| embedded-io-async 0.6 | `Write` | No | async fn |
| embedded-io-async 0.6 | `BufRead` | No | async fn |
| embedded-io-async 0.6 | `ReadReady` | **Yes** | `Error` |
| embedded-io-async 0.6 | `WriteReady` | **Yes** | `Error` |
| embedded-io-async 0.6 | `Seek` | No | async fn |
| embedded-io-async 0.7 | `ReadReady` | No | supertrait `Read` has async fn |
| embedded-io-async 0.7 | `WriteReady` | No | supertrait `Write` has async fn |

8/8 blocking traits are dyn-compatible. 0/5 async HAL traits are.
For embedded-io-async, only `ReadReady`/`WriteReady` in **0.6** are
dyn-compatible; 0.7 broke this by changing supertraits to `Read`/`Write`.

---

### Partition-Facing vs Kernel-Internal Suitability

**Partition-facing API** (requires dyn dispatch across SVC boundary):
the **blocking embedded-hal 1.0 traits are the best candidates**. A
partition calls `svc!(SYS_DEV_READ, ...)` which the kernel dispatches
to `dyn I2c<Error = E>` in the device registry. Async traits cannot
be used directly because: (1) `async fn` returns opaque `impl Future`,
incompatible with `dyn`; (2) workarounds (manual vtable, type erasure)
are not `no_std`/`no_alloc` friendly; (3) the RTOS uses synchronous
SVC-based scheduling, not an async executor.

**Kernel-internal use:** async traits are useful inside the kernel for
interrupt-driven drivers or a future async executor. `ReadReady`/
`WriteReady` (0.6) could serve as non-blocking readiness checks even
without async, but upgrading to 0.7 loses dyn-compatibility.

| Concern | Approach |
|---------|----------|
| Error types across SVC | Kernel `DriverError` implements each module's `Error`; partition sees uniform SVC error code |
| Generic parameters | Fixed at SVC boundary (`Word = u8`, `A = SevenBitAddress`) |
| Ownership | Kernel owns `dyn` trait objects; partition access via device ID + SVC |
| `&mut self` | Single-core: only one partition runs at a time per schedule slot |
| Async traits for partitions | Not recommended; use blocking. Manual vtable if needed later |
| embedded-io-async version | Pin to 0.6 if `ReadReady`/`WriteReady` dyn dispatch needed |
