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

---

## 2  PAC Crate Structure Comparison

This section examines how real-world PAC and HAL crates expose
peripheral registers and implement `embedded-hal` traits, to inform
the kernel's driver-model design.

### 2.1  STM32F4 (stm32f4 0.16 + stm32f4xx-hal 0.22)

#### 2.1.1  svd2rust-Generated PAC Structure

The `stm32f4` crate (v0.16, generated by svd2rust from ST SVD files
with community patches from [stm32-rs](https://github.com/stm32-rs/stm32-rs))
provides a zero-cost, type-safe register access layer. Each device
variant (stm32f401, stm32f407, etc.) is a feature-gated module.

**Singleton ownership via `Peripherals::take()`:**

```rust
use stm32f4::stm32f407::Peripherals;

let dp = Peripherals::take().unwrap(); // Returns Some once, None after
let spi1 = dp.SPI1;    // Move out owned SPI1 handle
let gpioa = dp.GPIOA;  // Move out owned GPIOA handle
let rcc = dp.RCC;       // Move out owned RCC handle
```

Internally an `AtomicBool` ensures `take()` succeeds at most once.
An `unsafe fn steal()` escape hatch exists for split-init scenarios.

**Read / write / modify closures:**

Every register exposes three methods on its `Reg<SPEC>` type:

```rust
// read() — returns a reader proxy R with field accessors
let sr = spi1.sr().read();
let busy: bool = sr.bsy().bit_is_set();
let overrun: bool = sr.ovr().bit_is_set();

// write(|w| ...) — closure receives W pre-loaded with reset value
spi1.cr1().write(|w| {
    w.mstr().set_bit()     // master mode
     .spe().set_bit()      // SPI enable
     .br().bits(0b010)     // baud = f_PCLK / 8
     .cpol().clear_bit()   // clock polarity 0
     .cpha().clear_bit()   // clock phase 0
});

// modify(|r, w| ...) — read-modify-write in one closure
spi1.cr1().modify(|r, w| {
    w.spe().bit(!r.spe().bit()) // toggle SPI enable
});
```

**Field proxy methods — reader side:**

| Method | Returns | Usage |
|--------|---------|-------|
| `.bits()` | Raw multi-bit value | `cr1.read().br().bits()` → `u8` |
| `.bit()` | `bool` | Single-bit field |
| `.bit_is_set()` | `bool` | True if bit = 1 |
| `.bit_is_clear()` | `bool` | True if bit = 0 |
| `.variant()` | Enum | When SVD defines `enumeratedValues` |
| `.is_<name>()` | `bool` | Variant check, e.g. `.is_master()` |

**Field proxy methods — writer side:**

| Method | Description |
|--------|-------------|
| `.bits(val)` | Write raw bits (unsafe if not all patterns enumerated) |
| `.set_bit()` | Write 1 to single-bit field |
| `.clear_bit()` | Write 0 to single-bit field |
| `.variant(E)` | Write type-safe enum variant |
| `.<name>()` | Named variant shorthand, e.g. `.master()`, `.div4()` |

#### 2.1.2  Peripheral Block Organization

Each peripheral is a zero-sized type (e.g. `SPI1`, `I2C1`, `USART2`)
that `Deref`s to a `#[repr(C)]` `RegisterBlock`:

```rust
// SPI RegisterBlock (simplified)
#[repr(C)]
pub struct RegisterBlock {
    pub cr1: CR1,   // 0x00
    pub cr2: CR2,   // 0x04
    pub sr:  SR,    // 0x08
    pub dr:  DR,    // 0x0C
    pub crcpr: CRCPR, // 0x10
    pub rxcrcr: RXCRCR, // 0x14
    pub txcrcr: TXCRCR, // 0x18
    pub i2scfgr: I2SCFGR, // 0x1C
    pub i2spr: I2SPR,     // 0x20
}
```

Peripheral instances share a `RegisterBlock` type when register layouts
match (e.g. `SPI1`, `SPI2`, `SPI3` share the same SPI `RegisterBlock`).
The instances differ only in their base address. For STM32F407, the PAC
exposes ~76 peripheral fields spanning GPIO (A–K), SPI (1–3), I2C (1–3),
USART (1–6), TIM (1–14), ADC (1–3), DMA (1–2), USB OTG, Ethernet, etc.

#### 2.1.3  Minimal SPI Driver Using PAC Registers Directly

A register-level SPI1 init and 8-bit transfer on STM32F407, using only
the PAC — no HAL, no allocator:

```rust
use stm32f4::stm32f407::Peripherals;

fn spi1_init(dp: &Peripherals) {
    // 1. Enable clocks: GPIOA on AHB1, SPI1 on APB2
    dp.RCC.ahb1enr().modify(|_, w| w.gpioaen().set_bit());
    dp.RCC.apb2enr().modify(|_, w| w.spi1en().set_bit());

    // 2. Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as AF5
    dp.GPIOA.moder().modify(|_, w| {
        w.moder5().alternate()   // PA5 → AF mode
         .moder6().alternate()   // PA6 → AF mode
         .moder7().alternate()   // PA7 → AF mode
    });
    dp.GPIOA.afrl().modify(|_, w| {
        w.afrl5().af5()   // PA5 → AF5 (SPI1_SCK)
         .afrl6().af5()   // PA6 → AF5 (SPI1_MISO)
         .afrl7().af5()   // PA7 → AF5 (SPI1_MOSI)
    });

    // 3. SPI1 config: master, 8-bit, CPOL=0/CPHA=0, baud=PCLK/8
    dp.SPI1.cr1().write(|w| {
        w.mstr().set_bit()     // master mode
         .br().bits(0b010)     // baud rate = f_PCLK2 / 8
         .cpol().clear_bit()   // CPOL = 0
         .cpha().clear_bit()   // CPHA = 0
         .ssm().set_bit()      // software slave management
         .ssi().set_bit()      // internal slave select high
         .dff().clear_bit()    // 8-bit data frame
         .spe().set_bit()      // enable SPI
    });
    dp.SPI1.cr2().write(|w| w); // CR2 reset defaults (no DMA, no IRQ)
}

fn spi1_transfer_byte(dp: &Peripherals, tx: u8) -> u8 {
    // Wait until TX buffer empty
    while dp.SPI1.sr().read().txe().bit_is_clear() {}
    // Write byte to data register
    dp.SPI1.dr().write(|w| w.dr().bits(tx as u16));
    // Wait until RX buffer not empty
    while dp.SPI1.sr().read().rxne().bit_is_clear() {}
    // Read received byte
    dp.SPI1.dr().read().dr().bits() as u8
}
```

This code runs on bare-metal `no_std` with zero allocations. The PAC
provides compile-time-checked field names but no higher-level
abstractions — the developer manually manages clocks, GPIO alternate
functions, and register bit fields.

#### 2.1.4  What stm32f4xx-hal Adds

The `stm32f4xx-hal` crate (v0.22) layers type-safe abstractions on
top of the PAC, implementing `embedded-hal 1.0` traits.

**Type-state GPIO:**

Pins carry their mode in the type system, preventing misuse at compile
time:

```rust
use stm32f4xx_hal::gpio::GpioExt;

let gpioa = dp.GPIOA.split();  // Consumes PAC GPIOA, returns typed pins

// Each into_*() method returns a new type — mode transitions are moves
let sck  = gpioa.pa5.into_alternate::<5>();  // PA5<Alternate<5>>
let miso = gpioa.pa6.into_alternate::<5>();  // PA6<Alternate<5>>
let mosi = gpioa.pa7.into_alternate::<5>();  // PA7<Alternate<5>>
let led  = gpioa.pa0.into_push_pull_output(); // PA0<Output<PushPull>>
let btn  = gpioa.pa1.into_pull_up_input();    // PA1<Input>
```

Type erasure is available for runtime-generic pin collections:
`ErasedPin<Output<PushPull>>` erases both port and pin number.

**Clock configuration builder:**

```rust
use stm32f4xx_hal::rcc::RccExt;

let rcc = dp.RCC.constrain();
let clocks = rcc.cfgr
    .use_hse(8.MHz())     // 8 MHz external crystal
    .sysclk(168.MHz())    // 168 MHz system clock
    .pclk1(42.MHz())      // APB1 = 42 MHz
    .pclk2(84.MHz())      // APB2 = 84 MHz
    .freeze();             // Locks config, returns immutable Clocks
```

`freeze()` computes PLL parameters internally and returns an immutable
`Clocks` token used by peripheral constructors to derive baud rates.

**embedded-hal 1.0 trait implementations:**

| HAL Type | Trait(s) |
|----------|----------|
| `Pin<..., Input>` | `InputPin` |
| `Pin<..., Output<...>>` | `OutputPin`, `StatefulOutputPin` |
| `Spi<SPI>` | `SpiBus<u8>`, `SpiBus<u16>` |
| `I2c<I2C>` | `I2c` (7-bit) |
| Timer delay types | `DelayNs` |
| PWM channels | `SetDutyCycle` |
| `Serial` / `Tx` / `Rx` | `embedded_io::{Read, Write}` |

Constructing a HAL SPI driver (contrast with the PAC-only version):

```rust
use stm32f4xx_hal::spi::{Spi, SpiExt};

let spi = dp.SPI1.spi(
    (sck, miso, mosi),         // type-state AF pins
    embedded_hal::spi::MODE_0, // CPOL=0, CPHA=0
    1.MHz(),                   // clock rate
    &clocks,                   // from rcc.cfgr.freeze()
);
// spi implements embedded_hal::spi::SpiBus<u8>
```

**DMA transfer types:**

The DMA module provides zero-copy peripheral transfers with compile-time
stream/channel validation:

```rust
Transfer<STREAM, CHANNEL, PERIPHERAL, DIRECTION, BUF>
```

Direction types: `MemoryToPeripheral`, `PeripheralToMemory`,
`MemoryToMemory<T>`. Double buffering supported. Valid
stream + channel + peripheral combinations are enforced by trait bounds.

#### 2.1.5  no_alloc Usability

Both `stm32f4` and `stm32f4xx-hal` are **fully `no_std`, no `alloc`**.
All types are stack-allocated or statically sized. No heap allocator
required. This makes them directly usable in the RTOS partition
environment where `alloc` is unavailable.
