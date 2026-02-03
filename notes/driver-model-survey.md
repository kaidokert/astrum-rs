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

---

### 2.2  Microchip SAM D51 / E51 (atsamd51j 0.14 + atsamd-hal 0.23)

The SAM D5x/E5x family is a 32-bit ARM Cortex-M4F MCU from Microchip
(formerly Atmel). Common boards: Adafruit Feather M4, Metro M4, PyGamer,
EdgeBadge, Matrix Portal M4. The Rust ecosystem is provided by the
[atsamd-rs](https://github.com/atsamd-rs/atsamd) mono-repository, which
contains PAC crates, a shared HAL crate, and board-support packages.

#### 2.2.1  PAC Crate Structure

The PAC crates are generated by **svd2rust v0.33.5** from Microchip SVD
files. Each device variant gets its own crate — `atsamd51j`, `atsamd51g`,
`atsamd51p`, `atsame51j`, `atsame51g`, `atsame51n`, `atsame53j`, `atsame54p`,
etc. For the SAMD51J (64-pin, used in Feather M4 / Metro M4), the crate
is `atsamd51j`.

| Crate | Version | svd2rust | Target |
|-------|---------|----------|--------|
| `atsamd51j` | 0.14.2 | v0.33.5 | `thumbv7em-none-eabihf` |
| `atsame51j` | 0.14.2 | v0.33.5 | `thumbv7em-none-eabihf` |
| `atsamd-hal` | 0.23.1 | — | `thumbv7em-none-eabihf` |

**Singleton ownership — identical to STM32:**

```rust
use atsamd51j::Peripherals;

let dp = Peripherals::take().unwrap(); // Returns Some once, None after
let sercom0 = dp.SERCOM0;  // Move out owned SERCOM0 handle
let port    = dp.PORT;     // Move out owned PORT handle
let mclk    = dp.MCLK;    // Move out owned MCLK handle
```

Internally uses the same `AtomicBool` guard as STM32 PACs. The `unsafe
fn steal()` escape hatch exists. `CorePeripherals::take()` provides
separate access to the Cortex-M4 NVIC, SCB, SysTick, MPU, FPU, etc.

**Read / write / modify closures — identical API to STM32:**

```rust
// read() returns a reader with field accessors
let busy = sercom0.spim().syncbusy().read().enable().bit_is_set();

// write(|w| ...) closure receives W pre-loaded with reset value
sercom0.spim().ctrla().write(|w| {
    w.mode().spi_master()     // MODE = 0x3
     .dopo().pad0()           // MOSI = PAD[0], SCK = PAD[1]
     .dipo().bits(0x3)        // MISO = PAD[3]
     .cpol().clear_bit()
     .cpha().clear_bit()
     .dord().clear_bit()      // MSB first
});

// modify(|r, w| ...) read-modify-write
sercom0.spim().ctrla().modify(|_, w| w.enable().set_bit());
```

**Peripheral block organization:**

The `Peripherals` struct exposes ~55 peripheral instances. The SAMD51J
has 6 SERCOM instances (SERCOM0--SERCOM5), each represented as a
zero-sized type that `Deref`s to a shared `RegisterBlock`.

Unlike STM32 where SPI/I2C/UART are separate peripherals, each SERCOM
is a **multi-mode peripheral** that can be configured as UART, SPI, or
I2C. The PAC reflects this by exposing mode-specific register views:

```rust
// SERCOM0 provides six register views:
sercom0.spim()      // SPI Master mode RegisterBlock
sercom0.spis()      // SPI Slave mode RegisterBlock
sercom0.i2cm()      // I2C Master mode RegisterBlock
sercom0.i2cs()      // I2C Slave mode RegisterBlock
sercom0.usart_int() // USART Internal Clock mode RegisterBlock
sercom0.usart_ext() // USART External Clock mode RegisterBlock
```

These are all overlapping register views of the same physical register
space — the hardware layout is a union, and the MODE field in CTRLA
selects which interpretation is active.

#### 2.2.2  SERCOM Peripheral Model

**Instance count by variant:**

| Variant | Package | SERCOMs | Notes |
|---------|---------|:-------:|-------|
| SAMD51G | 48-pin QFN | 6 | SERCOM0--5 |
| SAMD51J | 64-pin QFN/TQFP | 6 | SERCOM0--5 (Feather M4, Metro M4) |
| SAMD51N | 100-pin TQFP | 8 | SERCOM0--7 |
| SAMD51P | 128-pin TQFP | 8 | SERCOM0--7 |
| SAME51J | 64-pin | 6 | SERCOM0--5 (+ CAN peripherals) |

Each SERCOM instance can independently operate as:
- **SPI** (master or slave)
- **I2C** (master or slave, up to 3.4 MHz)
- **USART** (with internal or external clock, supports LIN, RS485)

Each SERCOM has 4 interrupt vectors (vs. 1 on SAMD21): SERCOM*n*\_0
through SERCOM*n*\_3.

**Pad multiplexing:**

Each SERCOM instance has **4 pads** (PAD[0] -- PAD[3]). GPIO pins are
mapped to SERCOM pads via the PORT peripheral's PMUX (Pin Mux) register:

- **Peripheral Function C** (`PIO_SERCOM` / `AlternateC`): Primary
  SERCOM assignment
- **Peripheral Function D** (`PIO_SERCOM_ALT` / `AlternateD`): Alternate
  SERCOM assignment

A single GPIO pin can reach one SERCOM via Function C and a different
SERCOM via Function D. Example for SAMD51J:

| GPIO Pin | Function C (SERCOM) | Function D (SERCOM-ALT) |
|----------|--------------------|-----------------------|
| PA08 | SERCOM0 PAD[0] | SERCOM2 PAD[1] |
| PA09 | SERCOM0 PAD[1] | SERCOM2 PAD[0] |
| PA12 | SERCOM2 PAD[0] | SERCOM4 PAD[1] |
| PA13 | SERCOM2 PAD[1] | SERCOM4 PAD[0] |
| PA16 | SERCOM1 PAD[0] | SERCOM3 PAD[1] |
| PA17 | SERCOM1 PAD[1] | SERCOM3 PAD[0] |
| PB22 | SERCOM1 PAD[2] | SERCOM5 PAD[2] |
| PB23 | SERCOM1 PAD[3] | SERCOM5 PAD[3] |

**IoSet constraints (SAMx5x specific):**

On SAM D5x/E5x devices, pins assigned to the same SERCOM must belong
to the same **IOSET** (I/O Set). An IOSET is a predefined group of
pin-to-pad assignments published in the datasheet. Mixing pins from
different IOSETs on the same SERCOM is not allowed.

Example: for SERCOM1 in IoSet1, valid pins might be PA16 (PAD0),
PA17 (PAD1), PA18 (PAD2), PA19 (PAD3). Using PA16 from IoSet1 and
PB23 from IoSet2 on the same SERCOM is invalid — the `atsamd-hal`
enforces this at compile time.

The datasheet defines IoSet1 through IoSet6, plus the HAL adds
`UndocIoSet1` and `UndocIoSet2` for pin combinations verified to
work on Adafruit boards but not formally documented by Microchip.

#### 2.2.3  SYNCBUSY Barrier Pattern

SAMD51 peripherals operate across two clock domains — the CPU/bus clock
(up to 120 MHz) and the peripheral's generic clock (GCLK, can be lower).
When the CPU writes to a register in the peripheral clock domain, the
write must be synchronized across domains. The hardware indicates this
synchronization is in progress via the **SYNCBUSY** register.

**The barrier pattern: write, then poll SYNCBUSY until clear:**

```c
// C-level pattern:
SERCOM0->SPI.CTRLA.bit.ENABLE = 1;
while (SERCOM0->SPI.SYNCBUSY.bit.ENABLE) {}
```

```rust
// Rust PAC equivalent:
sercom.spim().ctrla().modify(|_, w| w.enable().set_bit());
while sercom.spim().syncbusy().read().enable().bit_is_set() {}
```

If the CPU writes to a peripheral register while SYNCBUSY is set for
a previous write, the CPU will stall (bus wait states) until the
previous synchronization completes. Explicitly polling avoids
unpredictable stall durations.

**SERCOM SPI SYNCBUSY register fields:**

| Bit | Field | Description |
|-----|-------|-------------|
| 0 | SWRST | Software Reset synchronization busy |
| 1 | ENABLE | SERCOM Enable synchronization busy |
| 2 | CTRLB | Control B register synchronization busy |
| 4 | LENGTH | LENGTH register synchronization busy (SAMx5x only) |

**Which SPI registers require SYNCBUSY barriers:**

| Register | Write-Synchronized | SYNCBUSY bit to poll |
|----------|--------------------|---------------------|
| CTRLA.SWRST | Yes | `SYNCBUSY.SWRST` |
| CTRLA.ENABLE | Yes | `SYNCBUSY.ENABLE` |
| CTRLB | Yes | `SYNCBUSY.CTRLB` |
| LENGTH | Yes (SAMx5x) | `SYNCBUSY.LENGTH` |
| BAUD | No | — (CPU domain only) |
| DATA | No | — (uses DRE/TXC flags) |
| CTRLA (other fields) | No | — (only SWRST and ENABLE are sync'd) |
| ADDR | No | — |
| DBGCTRL | No | — |

**Key insight vs. STM32:** STM32 SPI has no SYNCBUSY equivalent —
register writes take effect immediately because the bus and peripheral
share the same clock domain (APB). The SAMD51's dual-clock architecture
means drivers must insert explicit sync barriers. This is a significant
structural difference that affects driver design.

#### 2.2.4  SPI Register-Level Example on SERCOM

A register-level SPI init and 8-bit transfer on SAMD51J SERCOM0, using
only the PAC — no HAL, no allocator:

```rust
use atsamd51j::Peripherals;

fn sercom0_spi_init(dp: &Peripherals) {
    // 1. Enable clocks: SERCOM0 on MCLK APB bus, configure GCLK
    dp.MCLK.apbamask().modify(|_, w| w.sercom0_().set_bit());
    // (GCLK generator assignment to SERCOM0_CORE omitted for brevity)

    // 2. Configure GPIO pins via PORT PMUX for SERCOM0
    //    PA04 = PAD[0] (MOSI), PA05 = PAD[1] (SCK),
    //    PA06 = PAD[2] (SS), PA07 = PAD[3] (MISO)
    //    All via Peripheral Function D (SERCOM-ALT)
    dp.PORT.group0().pmux2().modify(|_, w| {
        w.pmuxe().d()  // PA04 → Function D
         .pmuxo().d()  // PA05 → Function D
    });
    dp.PORT.group0().pmux3().modify(|_, w| {
        w.pmuxe().d()  // PA06 → Function D
         .pmuxo().d()  // PA07 → Function D
    });
    dp.PORT.group0().pincfg4().modify(|_, w| w.pmuxen().set_bit());
    dp.PORT.group0().pincfg5().modify(|_, w| w.pmuxen().set_bit());
    dp.PORT.group0().pincfg6().modify(|_, w| w.pmuxen().set_bit());
    dp.PORT.group0().pincfg7().modify(|_, w| w.pmuxen().set_bit());

    // 3. Software reset SERCOM0
    dp.SERCOM0.spim().ctrla().write(|w| w.swrst().set_bit());
    while dp.SERCOM0.spim().syncbusy().read().swrst().bit_is_set() {}

    // 4. Configure CTRLA: SPI Master, DOPO=0, DIPO=3
    //    DOPO=0x0: DO=PAD[0](MOSI), SCK=PAD[1], SS=PAD[2]
    //    DIPO=0x3: DI=PAD[3](MISO)
    dp.SERCOM0.spim().ctrla().write(|w| {
        w.mode().spi_master()      // MODE = 0x3
         .dopo().pad0()            // DOPO=0 → MOSI=PAD0, SCK=PAD1
         .dipo().bits(0x3)         // DIPO=3 → MISO=PAD3
         .cpol().clear_bit()       // CPOL = 0
         .cpha().clear_bit()       // CPHA = 0
         .dord().clear_bit()       // MSB first
    });

    // 5. Configure CTRLB: enable receiver, 8-bit character size
    dp.SERCOM0.spim().ctrlb().write(|w| {
        w.rxen().set_bit()         // Enable receiver
         .chsize().bits(0x0)       // 8-bit characters
    });
    while dp.SERCOM0.spim().syncbusy().read().ctrlb().bit_is_set() {}

    // 6. Set baud rate: BAUD = (f_ref / (2 * f_baud)) - 1
    //    e.g. 48 MHz GCLK / (2 * 4 MHz) - 1 = 5
    // SAFETY: BAUD field accepts any u8 value; all bit patterns are valid
    // baud rates per the SAMD51 datasheet §36.8.1.
    dp.SERCOM0.spim().baud().write(|w| unsafe { w.baud().bits(5) });

    // 7. Enable SPI
    dp.SERCOM0.spim().ctrla().modify(|_, w| w.enable().set_bit());
    while dp.SERCOM0.spim().syncbusy().read().enable().bit_is_set() {}
}

fn sercom0_spi_transfer_byte(dp: &Peripherals, tx: u8) -> u8 {
    // Wait until Data Register Empty
    while dp.SERCOM0.spim().intflag().read().dre().bit_is_clear() {}
    // Write byte to DATA register
    // SAFETY: DATA field accepts any u32 value; the hardware uses only the
    // lower CHSIZE bits and ignores upper bits.
    dp.SERCOM0.spim().data().write(|w| unsafe { w.data().bits(tx as u32) });
    // Wait until Receive Complete
    while dp.SERCOM0.spim().intflag().read().rxc().bit_is_clear() {}
    // Read received byte from DATA register
    dp.SERCOM0.spim().data().read().data().bits() as u8
}
```

**CTRLA register fields (SPI Master mode — MODE=0x3):**

| Field | Bits | Values |
|-------|------|--------|
| SWRST | 0 | Software reset |
| ENABLE | 1 | Peripheral enable |
| MODE | 2:4 | 0x2=SPI Slave, 0x3=SPI Master |
| RUNSTDBY | 7 | Run in standby |
| IBON | 8 | Immediate buffer overflow notification |
| DOPO | 16:17 | Data Out Pinout — see table below |
| DIPO | 20:21 | Data In Pinout — PAD number (0--3) |
| FORM | 24:27 | 0x0=SPI frame, 0x2=SPI frame with address |
| CPHA | 28 | Clock phase (0=leading edge, 1=trailing) |
| CPOL | 29 | Clock polarity (0=idle low, 1=idle high) |
| DORD | 30 | Data order (0=MSB first, 1=LSB first) |

**DOPO field mapping (SAMD51 — more restricted than SAMD21):**

| DOPO | DO (MOSI) | SCK | SS (hw) | Notes |
|:----:|-----------|-----|---------|-------|
| 0x0 | PAD[0] | PAD[1] | PAD[2] | Default / most common |
| 0x2 | PAD[3] | PAD[1] | PAD[2] | Alternate output pad |

Values 0x1 and 0x3 are **reserved** on SAMD51 (only two valid DOPO
configurations, vs. four on SAMD21). SCK is always on PAD[1], SS
always on PAD[2]. DIPO selects any of PAD[0]--PAD[3] for data input.

**CTRLB register fields (SPI Master mode):**

| Field | Bits | Description |
|-------|------|-------------|
| CHSIZE | 0:2 | Character size (0x0=8-bit, 0x1=9-bit) |
| PLOADEN | 6 | Data preload enable |
| SSDE | 9 | Slave select low detect enable |
| MSSEN | 13 | Master slave select enable (hardware SS) |
| AMODE | 14:15 | Address mode |
| RXEN | 17 | Receiver enable |

**INTFLAG register (SPI status/interrupt flags):**

| Bit | Flag | Description |
|-----|------|-------------|
| 0 | DRE | Data Register Empty — TX buffer ready for new data |
| 1 | TXC | Transmit Complete — shift register empty |
| 2 | RXC | Receive Complete — data available in RX buffer |
| 3 | SSL | Slave Select Low (slave mode) |
| 7 | ERROR | Combined error flag |

**Additional SAMx5x registers (not present on SAMD21):**

| Register | Description |
|----------|-------------|
| CTRLC | Control C — 32-bit data support, inter-character spacing |
| LENGTH | Transaction length counter (1--255 bytes, hardware-managed) |

The LENGTH counter is unique to SAMx5x. When set, the hardware
automatically counts bytes and asserts SS after the specified number
of transfers, eliminating software byte-counting overhead. The HAL
always uses 32-bit extension mode (`CTRLC.DATA32B`) with the LENGTH
counter on SAMx5x.

#### 2.2.5  What atsamd-hal Adds

The `atsamd-hal` crate (v0.23.1) implements a **type-level programming
model** substantially more advanced than `stm32f4xx-hal`. It enforces
valid peripheral configurations entirely at compile time using Rust's
type system — invalid pin combinations, mode conflicts, and IoSet
violations are all compile errors.

**Type-level SERCOM pad configuration:**

The PAC treats all SERCOMs uniformly, but the HAL introduces type-level
types for each SERCOM instance and pad number:

```rust
use atsamd_hal::sercom::Sercom0;
use atsamd_hal::sercom::pad::{Pad0, Pad1, Pad2, Pad3};
```

Key traits:

| Trait | Purpose |
|-------|---------|
| `Sercom` | Type-level enum: `Sercom0`, `Sercom1`, ..., `Sercom5` |
| `PadNum` | Type-level enum: `Pad0`, `Pad1`, `Pad2`, `Pad3` |
| `GetPad<S>` | Maps a `PinId` + `Sercom` → `PadNum` + `PinMode` |
| `IsPad` | Marks a `Pin` correctly configured as a SERCOM pad |
| `OptionalPad` | `IsPad` or `NoneT` (type-level `Option`) |
| `InIoSet<I>` | Labels a Pin as belonging to IoSet `I` |
| `IoSet` | Type-level enum: `IoSet1`..`IoSet6`, `UndocIoSet1`, `UndocIoSet2` |

**SPI Pads builder (type-safe, compile-time validated):**

```rust
use atsamd_hal::gpio::Pins;
use atsamd_hal::sercom::{Sercom0, spi};

let pins = Pins::new(dp.PORT);

// Builder pattern: each method changes the type parameter
let pads = spi::Pads::<Sercom0>::default()
    .sclk(pins.pa09)       // Pads<Sercom0, NoneT, NoneT, PA09, NoneT>
    .data_in(pins.pa08)    // Pads<Sercom0, PA08, NoneT, PA09, NoneT>
    .data_out(pins.pa11);  // Pads<Sercom0, PA08, PA11, PA09, NoneT>
```

The `Pads` struct:

```rust
pub struct Pads<S, DI = NoneT, DO = NoneT, CK = NoneT, SS = NoneT>
where
    S: Sercom,
    DI: OptionalPad,
    DO: OptionalPad,
    CK: OptionalPad,
    SS: OptionalPad,
{ ... }
```

Alternatively, `PadsFromIds` allows defining Pads from `PinId` types
without constructing `Pin` values:

```rust
type MyPads = spi::PadsFromIds<Sercom0, PA08, NoneT, PA09>;
```

**IoSet enforcement (SAMx5x only):**

On SAMx5x, the `Pads` type carries an `IoSet` parameter, and all pads
must share a common IoSet. The `ShareIoSet` trait computes the
intersection of valid IoSets across all specified pads — if the
intersection is empty, the code fails to compile:

```rust
// Valid: PA16, PA17, PA18, PA19 all in IoSet1 for Sercom1
let pads = spi::Pads::<Sercom1>::default()
    .sclk(pins.pa17)
    .data_in(pins.pa19)
    .data_out(pins.pa16);  // Compiles: all share IoSet1

// Invalid: PA16 (IoSet1) + PB23 (IoSet2) → no common IoSet
// let pads = spi::Pads::<Sercom1>::default()
//     .sclk(pins.pa17)
//     .data_out(pins.pb23);  // Compile error!
```

**DipoDopo — compile-time DOPO/DIPO computation:**

The `DipoDopo` trait computes the hardware DIPO and DOPO register values
from the pad number assignments, entirely at compile time:

```rust
pub trait DipoDopo {
    const DIPO_DOPO: (u8, u8);  // (DIPO, DOPO) register values
}
```

The `Dipo` trait maps pad numbers to DIPO values (Pad0→0, Pad1→1,
Pad2→2, Pad3→3). The `Dopo` trait maps to DOPO values (Pad0→0,
Pad3→2). If an invalid pad is used for DO (e.g. Pad2 as data output),
the code fails to compile because there is no `Dopo` impl for `Pad2`.

When only one data direction is specified, the implementation
automatically selects a non-conflicting pad for the unused direction's
register value.

**Capability inference — how the HAL determines SPI vs half-duplex:**

The `Capability` trait is inferred from which pads are populated:

| DI pad | DO pad | Inferred Capability | Description |
|:------:|:------:|:-------------------:|-------------|
| Some | Some | `Duplex` | Full-duplex SPI (simultaneous TX+RX) |
| Some | None | `Rx` | Receive-only |
| None | Some | `Tx` | Transmit-only |

The `ValidPads` trait requires at least CK (clock) plus one of DI or
DO. The `Capability` type parameter on `Spi<C, A>` carries through to
trait implementations — e.g., `Spi<_, Duplex>` implements both `Read`
and `Write`, while `Spi<_, Tx>` implements only `Write`.

Wrapper types `PanicOnRead` and `PanicOnWrite` allow using a
half-duplex SPI with APIs that expect full-duplex, panicking if the
unconnected direction is accessed.

**Config and Spi structs (three type parameters):**

```rust
pub struct Config<P, M = Master, Z = DefaultSize>
where
    P: ValidPads,
    M: OpMode,   // Master | MasterHWSS | Slave
    Z: Size,     // Length type (U1..U255 on SAMx5x)
{ ... }

pub struct Spi<C, A, RxDma = NoneT, TxDma = NoneT>
where
    C: ValidConfig,
    A: Capability,  // Duplex | Rx | Tx
{ ... }
```

Configuration follows a builder pattern with type-changing methods:

```rust
let spi = spi::Config::new(&mclk, sercom0, pads, freq)
    .baud(1.mhz())
    .length::<U2>()            // Transaction length = 2 bytes
    .bit_order(BitOrder::LsbFirst)
    .spi_mode(MODE_1)         // CPOL=0, CPHA=1
    .enable();                 // Config → Spi (consumes Config)
```

On SAMx5x, the `length::<U>()` method leverages the hardware LENGTH
counter (1--255 bytes). The `char_size()` method changes the `Size`
type parameter. The `op_mode()` method changes between `Master`,
`MasterHWSS` (hardware slave-select), and `Slave`.

**DMA integration:**

The `Spi` struct supports optional DMA channel attachment:

```rust
// Attach both RX and TX DMA channels (for Master mode)
let spi = spi.with_dma_channels(rx_channel, tx_channel);

// Or single-channel variants
let spi = spi.with_tx_channel(tx_channel);
let spi = spi.with_rx_channel(rx_channel);  // Slave mode

// Reclaim channels
let (spi, rx_ch, tx_ch) = spi.take_dma_channels();
```

DMA channels are carried as additional type parameters `RxDma` and
`TxDma` on the `Spi` struct. When `NoneT`, no DMA is used; when a
`DmaChannel` type, DMA transfers are available. The same embedded-hal
trait methods work transparently whether DMA is attached or not.

**Async support via `SpiFuture`:**

```rust
// Convert Spi → SpiFuture for async operation
let spi = spi.into_future();

// Optionally add DMA channels for async DMA
let spi = spi.with_rx_dma_channel(rx_ch)
              .with_tx_dma_channel(tx_ch);
```

Important: `into_future()` must be called **before** attaching DMA
channels for async mode. The reverse order will not enable async DMA.
`SpiFuture` implements `embedded_hal_async::spi::SpiBus`.

Futures support `Drop`-based cancellation. However, using `forget()`
or `ManuallyDrop` without cleanup is unsound (can corrupt DMA buffers).

**embedded-hal 1.0 trait implementations:**

| HAL Type | Trait(s) |
|----------|----------|
| `Spi<_, Duplex>` | `SpiBus<u8>`, `embedded_io::Read`, `embedded_io::Write` |
| `Spi<_, Rx>` | `embedded_hal_nb::serial::Read`, `embedded_io::Read` |
| `Spi<_, Tx>` | `embedded_hal_nb::serial::Write`, `embedded_io::Write` |
| `SpiFuture<_, Duplex>` | `embedded_hal_async::spi::SpiBus` |
| GPIO pins (Input) | `InputPin` |
| GPIO pins (Output) | `OutputPin`, `StatefulOutputPin` |
| Timer types | `DelayNs` |

The HAL implements **both** `embedded-hal 0.2` and `embedded-hal 1.0`
concurrently (dependency on `^0.2` and `^1.0.0`).

#### 2.2.6  Structural Comparison: STM32F4 vs SAMD51

| Aspect | STM32F4 | SAMD51 |
|--------|---------|--------|
| PAC generator | svd2rust | svd2rust (same) |
| Peripherals::take() | AtomicBool singleton | AtomicBool singleton (same) |
| Register API | read/write/modify closures | read/write/modify closures (same) |
| SPI peripheral | Dedicated SPI1/SPI2/SPI3 | SERCOM0--5, multi-mode (SPI/I2C/UART) |
| Pin mux | GPIO AFRL/AFRH, AF number | PORT PMUX, Function C/D |
| Clock gating | RCC APBxENR | MCLK APBxMASK + GCLK generator |
| Sync barriers | None (single clock domain) | **SYNCBUSY register polling required** |
| HAL type-level model | AF const generic, mode type-state | **IoSet + PadNum + Capability inference** |
| SPI data register | DR (16-bit) | DATA (32-bit with LENGTH counter) |
| Hardware byte counter | No | **Yes (LENGTH register, SAMx5x)** |
| DMA in HAL | Stream/Channel/Direction types | RxDma/TxDma type params on Spi |
| embedded-hal version | 1.0 | 0.2 + 1.0 (dual) |
| Async SPI | Not in mainline HAL | **SpiFuture + optional DMA** |

#### 2.2.7  no_alloc Usability

Both `atsamd51j` (PAC) and `atsamd-hal` are **fully `no_std`, no
`alloc`**. All types are stack-allocated or statically sized. No heap
allocator required. The type-level programming model (IoSet, PadNum,
Capability) adds zero runtime cost — all validation occurs at compile
time and is erased before code generation. This makes the ecosystem
directly usable in the RTOS partition environment where `alloc` is
unavailable.

#### 2.2.8  Implications for the RTOS Driver Model

1. **SYNCBUSY is a new concern:** Unlike STM32, SAMD51 drivers must
   include sync barriers after certain register writes. The kernel
   driver model must account for these non-trivial latencies in
   scheduling — a SYNCBUSY poll could take multiple peripheral clock
   cycles (microseconds if the GCLK is slow).

2. **SERCOM multiplexing complicates resource management:** A single
   SERCOM can be SPI, I2C, or UART. The kernel's device registry
   must track which mode a SERCOM is in and prevent conflicting
   reconfigurations.

3. **IoSet constraints are compile-time only:** The type-level IoSet
   system is a compile-time concept that has no runtime representation.
   If the RTOS supports runtime peripheral assignment, IoSet validity
   must be checked by static analysis at build time, not at runtime.

4. **Hardware LENGTH counter enables efficient SPI:** The SAMx5x
   LENGTH register allows the kernel to offload byte counting to
   hardware, reducing interrupt overhead for fixed-size SPI transactions.

5. **DMA integration pattern differs:** The atsamd-hal attaches DMA
   channels as type parameters rather than using a separate DMA
   transfer type. The kernel's DMA manager may need to adapt to
   this model.

6. **Dual embedded-hal support aids migration:** Supporting both 0.2
   and 1.0 simultaneously means existing drivers work while migrating
   to 1.0 traits for the partition-facing API.

---

### 2.3  Nordic nRF52840 (nrf52840-pac 0.12 + nrf52840-hal 0.19)

The nRF52840 is a 32-bit ARM Cortex-M4F SoC from Nordic Semiconductor
with Bluetooth 5, Thread, Zigbee, and 802.15.4 radio. Common boards:
Nordic nRF52840-DK, Adafruit Feather nRF52840, Particle Xenon, Maker
Diary nRF52840 MDK. The Rust ecosystem is provided by the
[nrf-rs](https://github.com/nrf-rs) organization, split across a PAC
repo (`nrf-pacs`) and a HAL repo (`nrf-hal`).

#### 2.3.1  PAC Crate Structure

The PAC crate is generated by **svd2rust v0.25.1** from Nordic SVD files.
Each nRF variant gets its own crate — `nrf52840-pac`, `nrf52832-pac`,
`nrf52833-pac`, `nrf5340-app-pac`, `nrf5340-net-pac`, `nrf9160-pac`, etc.

| Crate | Version | svd2rust | Target |
|-------|---------|----------|--------|
| `nrf52840-pac` | 0.12.2 | v0.25.1 | `thumbv7em-none-eabihf` |
| `nrf52840-hal` | 0.19.0 | — | `thumbv7em-none-eabihf` |
| `nrf-hal-common` | 0.18.0 | — | (shared code crate) |

**Singleton ownership — identical to STM32/SAMD:**

```rust
use nrf52840_pac::Peripherals;

let dp = Peripherals::take().unwrap(); // Returns Some once, None after
let spim0 = dp.SPIM0;  // Move out owned SPIM0 handle
let p0    = dp.P0;     // Move out owned P0 GPIO handle
let p1    = dp.P1;     // Move out owned P1 GPIO handle
```

Internally uses the same `AtomicBool` guard. The `unsafe fn steal()` escape
hatch exists. The `Peripherals` struct exposes ~72 peripheral fields spanning
SPIM (0--3), TWIM (0--1), UARTE (0--1), TIMER (0--4), SAADC, USBD, RADIO,
CRYPTOCELL, etc.

**Read / write / modify closures — identical API to STM32/SAMD:**

```rust
// read() returns a reader with field accessors
let enabled = spim0.enable.read().enable().is_enabled();

// write(|w| ...) closure receives W pre-loaded with reset value
spim0.frequency.write(|w| w.frequency().m4());

// modify(|r, w| ...) read-modify-write
spim0.config.modify(|_, w| w.order().msb_first());
```

#### 2.3.2  Task/Event Peripheral Architecture

The nRF52840 uses a fundamentally different peripheral interaction model
from STM32 and SAMD51. Instead of writing data directly to registers and
polling status bits, peripherals are controlled through dedicated **task
registers** and **event registers**, each occupying its own 32-bit address.

**Task registers** — write `1` to trigger a hardware action:

```rust
// Trigger the START task to begin a SPI DMA transaction
// SAFETY: Task registers accept any u32 value; writing 1 triggers the task
// per the nRF52840 Product Specification §6.1.2.
spim0.tasks_start.write(|w| unsafe { w.bits(1) });
```

**Event registers** — hardware sets to `1` when the event fires; software
reads to poll, writes `0` to clear:

```rust
// Poll for END event (both TX and RX DMA complete)
while spim0.events_end.read().bits() == 0 {}
// Clear the event
spim0.events_end.write(|w| w);  // write reset value (0)
```

This is structurally different from STM32 (status bits packed into a shared
SR register, cleared by side-effect reads) and SAMD51 (status bits in
INTFLAG, cleared by W1C — write-1-to-clear).

**SPIM task registers:**

| Register | Offset | Description |
|----------|--------|-------------|
| `TASKS_START` | 0x010 | Start SPI transaction (begins EasyDMA) |
| `TASKS_STOP` | 0x014 | Stop SPI transaction |
| `TASKS_SUSPEND` | 0x01C | Suspend SPI transaction |
| `TASKS_RESUME` | 0x020 | Resume suspended transaction |

**SPIM event registers:**

| Register | Offset | Description |
|----------|--------|-------------|
| `EVENTS_STOPPED` | 0x104 | Transaction has stopped |
| `EVENTS_ENDRX` | 0x110 | RXD.MAXCNT bytes received |
| `EVENTS_END` | 0x118 | Both RXD and TXD transfers complete |
| `EVENTS_ENDTX` | 0x120 | TXD.MAXCNT bytes transmitted |
| `EVENTS_STARTED` | 0x14C | Transaction started (double-buffered regs safe to update) |

**SHORTS register (hardware event-to-task wiring):**

The SHORTS register (offset 0x200) connects events directly to tasks
within the same peripheral, in hardware, with zero CPU involvement:

```rust
// Wire END event to START task for continuous DMA transfers
spim0.shorts.write(|w| w.end_start().enabled());
```

This is unique to the nRF architecture — STM32 and SAMD51 have no
equivalent. Continuous transfers on those platforms require interrupt
handlers to restart DMA.

**PPI (Programmable Peripheral Interconnect):**

PPI extends the SHORTS concept across peripherals. Any peripheral's event
can trigger any peripheral's task via configurable PPI channels, entirely
in hardware. For example, a TIMER compare event can trigger a SPIM START
task without CPU involvement.

#### 2.3.3  EasyDMA — Built-In DMA per Peripheral

Unlike STM32 (separate DMA1/DMA2 controllers with streams and channels)
and SAMD51 (separate DMAC controller with trigger sources), each nRF52840
SPIM instance has its own **EasyDMA engine** built directly into the
peripheral. There is no separate DMA controller to configure, no
stream/channel arbitration, and no DMA manager type in the HAL.

**EasyDMA registers (TXD cluster — transmit):**

| Register | Offset | Access | Description |
|----------|--------|--------|-------------|
| `TXD.PTR` | 0x544 | rw | Data pointer (must point to RAM) |
| `TXD.MAXCNT` | 0x548 | rw | Number of bytes to transmit |
| `TXD.AMOUNT` | 0x54C | r | Bytes actually transferred |
| `TXD.LIST` | 0x550 | rw | List type (scatter/gather config) |

**EasyDMA registers (RXD cluster — receive):**

| Register | Offset | Access | Description |
|----------|--------|--------|-------------|
| `RXD.PTR` | 0x534 | rw | Data pointer (must point to RAM) |
| `RXD.MAXCNT` | 0x538 | rw | Maximum bytes to receive |
| `RXD.AMOUNT` | 0x53C | r | Bytes actually transferred |
| `RXD.LIST` | 0x540 | rw | List type (scatter/gather config) |

`PTR` and `MAXCNT` are **double-buffered** — they can be updated
immediately after `EVENTS_STARTED` fires, preparing the next transaction
while the current one is still running.

On nRF52840, MAXCNT supports up to **65,535 bytes** (16-bit), compared to
255 bytes on nRF52832 (8-bit MAXCNT).

**Additional SPIM registers:**

| Register | Offset | Description |
|----------|--------|-------------|
| `ENABLE` | 0x500 | Peripheral enable (0x0=Disabled, 0x7=Enabled) |
| `FREQUENCY` | 0x524 | SPI clock frequency (predefined values) |
| `CONFIG` | 0x554 | CPOL, CPHA, bit order |
| `ORC` | 0x5C0 | Over-read character (byte sent when RX > TX) |

**FREQUENCY register predefined values:**

| Variant | Register value | Speed |
|---------|---------------|-------|
| `K125` | `0x0200_0000` | 125 kbps |
| `K250` | `0x0400_0000` | 250 kbps |
| `K500` | `0x0800_0000` | 500 kbps |
| `M1` | `0x1000_0000` | 1 Mbps |
| `M2` | `0x2000_0000` | 2 Mbps |
| `M4` | `0x4000_0000` | 4 Mbps |
| `M8` | `0x8000_0000` | 8 Mbps |
| `M16` | `0x0A00_0000` | 16 Mbps |
| `M32` | `0x1400_0000` | 32 Mbps |

Unlike STM32 (baud rate = f_PCLK / prescaler) and SAMD51 (BAUD register
formula), the nRF52840 uses predefined frequency constants — no calculation
needed.

**CONFIG register fields:**

| Field | Bit | Values |
|-------|-----|--------|
| ORDER | 0 | `MsbFirst` (0) / `LsbFirst` (1) |
| CPHA | 1 | `Leading` (0) / `Trailing` (1) |
| CPOL | 2 | `ActiveHigh` (0=idle low) / `ActiveLow` (1=idle high) |

#### 2.3.4  PSEL — Flexible Pin Muxing

The nRF52840 uses a **peripheral-selects-pin** model, fundamentally
different from both STM32 (pin selects AF) and SAMD51 (pin selects PMUX
function). Each SPIM instance has PSEL registers that can address any
GPIO pin:

**PSEL cluster (offset 0x508--0x514):**

| Register | Offset | Description |
|----------|--------|-------------|
| `PSEL.SCK` | 0x508 | Pin select for SCK |
| `PSEL.MOSI` | 0x50C | Pin select for MOSI |
| `PSEL.MISO` | 0x510 | Pin select for MISO |
| `PSEL.CSN` | 0x514 | Pin select for CSN |

**PSEL register bit layout:**

```
Bit:  31       30:6      5       4:0
    +--------+---------+------+--------+
    |CONNECT | Reserved| PORT |  PIN   |
    +--------+---------+------+--------+
      1 bit              1 bit  5 bits
```

| Field | Bits | Description |
|-------|------|-------------|
| PIN | 4:0 | Pin number (0--31) within selected port |
| PORT | 5 | Port number (0 = P0, 1 = P1) |
| CONNECT | 31 | 0 = Connected, 1 = Disconnected (reset default) |

Example: P1.09 → `PIN=9, PORT=1, CONNECT=0` → register value `0x0000_0029`.

**No fixed pin mapping tables.** Any GPIO pin from P0 (32 pins) or P1
(16 pins) can be assigned to any SPIM signal. There are no alternate
function numbers, no PMUX function tables, and no IoSet constraints.
The peripheral selects the pin, not the other way around.

#### 2.3.5  SPI Register-Level Example

A register-level SPI init and DMA transfer on nRF52840 SPIM0, using
only the PAC — no HAL, no allocator:

```rust
use nrf52840_pac::Peripherals;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};

fn spim0_init(dp: &Peripherals) {
    // 1. No clock gating needed — SPIM0 is clocked from HFCLK
    //    by default. No RCC/MCLK equivalent to configure.

    // 2. Pin selection: P0.03 = SCK, P0.04 = MOSI, P0.28 = MISO
    //    Write directly to PSEL registers — no GPIO AF config needed
    dp.SPIM0.psel.sck.write(|w| {
        // SAFETY: PIN field is 5 bits wide (0--31); value 3 is in range.
        unsafe { w.pin().bits(3) };
        w.port().port0();
        w.connect().connected()
    });
    dp.SPIM0.psel.mosi.write(|w| {
        // SAFETY: PIN field is 5 bits wide (0--31); value 4 is in range.
        unsafe { w.pin().bits(4) };
        w.port().port0();
        w.connect().connected()
    });
    dp.SPIM0.psel.miso.write(|w| {
        // SAFETY: PIN field is 5 bits wide (0--31); value 28 is in range.
        unsafe { w.pin().bits(28) };
        w.port().port0();
        w.connect().connected()
    });

    // 3. Configure: MODE_0 (CPOL=idle-low, CPHA=leading edge), MSB first
    dp.SPIM0.config.write(|w| {
        w.order().msb_first()
         .cpha().leading()
         .cpol().active_high()
    });

    // 4. Set frequency to 4 MHz
    dp.SPIM0.frequency.write(|w| w.frequency().m4());

    // 5. Over-read character (transmitted when RX > TX)
    // SAFETY: ORC field accepts any u8 value; all bit patterns are valid.
    dp.SPIM0.orc.write(|w| unsafe { w.orc().bits(0x00) });

    // 6. Enable SPIM0
    dp.SPIM0.enable.write(|w| w.enable().enabled());
}

fn spim0_dma_transfer(dp: &Peripherals, tx: &[u8], rx: &mut [u8]) {
    // Compiler fence: ensure tx[] writes are visible to DMA
    compiler_fence(SeqCst);

    // Set up EasyDMA pointers — must point to RAM
    // SAFETY: PTR fields accept any u32 address; caller ensures buffers are
    // in Data RAM (0x2000_0000..0x3000_0000). MAXCNT fields are 16 bits wide
    // on nRF52840; caller ensures lengths fit in u16.
    dp.SPIM0.txd.ptr.write(|w| unsafe { w.ptr().bits(tx.as_ptr() as u32) });
    dp.SPIM0.txd.maxcnt.write(|w| unsafe { w.maxcnt().bits(tx.len() as u16) });
    dp.SPIM0.rxd.ptr.write(|w| unsafe { w.ptr().bits(rx.as_mut_ptr() as u32) });
    dp.SPIM0.rxd.maxcnt.write(|w| unsafe { w.maxcnt().bits(rx.len() as u16) });

    // Clear END event from any prior transfer
    dp.SPIM0.events_end.write(|w| w);

    // Trigger START task — begins DMA transfer
    // SAFETY: Task registers accept any u32 value; writing 1 triggers the task.
    dp.SPIM0.tasks_start.write(|w| unsafe { w.bits(1) });

    // Compiler fence: prevent reordering past DMA start
    compiler_fence(SeqCst);

    // Poll for END event (both TX and RX complete)
    while dp.SPIM0.events_end.read().bits() == 0 {}

    // Clear END event
    dp.SPIM0.events_end.write(|w| w);
}
```

Key structural differences from the STM32/SAMD examples:

1. **No clock gating step.** SPIM0 is clocked from HFCLK by default;
   there is no RCC/MCLK peripheral to configure.
2. **No GPIO alternate function config.** Pins are selected by writing
   to PSEL registers — no PORT/PMUX/AFRL setup.
3. **DMA is the only transfer path.** There is no byte-at-a-time data
   register — all transfers go through EasyDMA pointers. Even a single
   byte requires setting `TXD.PTR`, `TXD.MAXCNT=1`, and triggering
   `TASKS_START`.
4. **Compiler fences instead of SYNCBUSY.** No dual clock domain, so
   no register synchronization delays — just compiler memory ordering.

#### 2.3.6  What nrf52840-hal Adds

The `nrf52840-hal` crate (v0.19.0) is a thin re-export wrapper around
`nrf-hal-common` (v0.18.0), which contains driver implementations shared
across all nRF chips. The HAL is considerably thinner than `stm32f4xx-hal`
or `atsamd-hal` — it wraps PAC peripherals with ergonomic constructors
and embedded-hal trait impls but does not use advanced type-level
programming.

**Spim wrapper struct:**

```rust
pub struct Spim<T: Instance>(T);

pub trait Instance: Deref<Target = spim0::RegisterBlock> + sealed::Sealed {}

// Implementations for all SPIM instances
impl Instance for SPIM0 {}
impl Instance for SPIM1 {}
impl Instance for SPIM2 {}
impl Instance for SPIM3 {}
```

**Constructor:**

```rust
impl<T: Instance> Spim<T> {
    pub fn new(
        spim: T,             // PAC peripheral (moved in)
        pins: Pins,          // GPIO pin bundle
        frequency: Frequency, // Predefined frequency constant
        mode: Mode,          // SPI mode (CPOL + CPHA)
        orc: u8,             // Over-read character
    ) -> Self { ... }
}
```

The constructor configures PSEL, CONFIG, FREQUENCY, ORC, and ENABLE
registers in sequence. Pins are type-erased at construction time.

**Pins struct — port-erased, optional pins:**

```rust
pub struct Pins {
    pub sck:  Option<Pin<Output<PushPull>>>,
    pub mosi: Option<Pin<Output<PushPull>>>,
    pub miso: Option<Pin<Input<Floating>>>,
}
```

All three pins are `Option` — any can be omitted. The `Pin<MODE>` type is
port-erased (any P0 or P1 pin), carrying port+pin encoding in an internal
`pin_port: u8` field. The `psel_bits()` method returns this as `u32` for
direct PSEL register writes.

**GPIO typing model:**

```rust
// Per-pin types with type-state mode
pub struct P0_03<MODE> { _mode: PhantomData<MODE> }
pub struct P1_09<MODE> { _mode: PhantomData<MODE> }

// Port-erased generic pin
pub struct Pin<MODE> {
    pin_port: u8,  // Encodes port (bit 5) + pin (bits 4:0)
    _mode: PhantomData<MODE>,
}

// Mode type-states
pub struct Input<PULL>;    // PULL = Floating | PullUp | PullDown
pub struct Output<DRIVE>;  // DRIVE = PushPull | OpenDrain
pub struct Disconnected;

// Conversion methods (consume self, return new type)
impl<MODE> P0_03<MODE> {
    pub fn into_push_pull_output(self) -> P0_03<Output<PushPull>> { ... }
    pub fn into_floating_input(self) -> P0_03<Input<Floating>> { ... }
    pub fn degrade(self) -> Pin<MODE> { ... }  // Erase port/pin identity
}
```

Port initialization consumes the PAC peripheral:

```rust
let p0 = p0::Parts::new(dp.P0);   // Consumes PAC P0, returns typed pins
let p1 = p1::Parts::new(dp.P1);   // Consumes PAC P1, returns typed pins

let sck  = p0.p0_03.into_push_pull_output().degrade();
let mosi = p0.p0_04.into_push_pull_output().degrade();
let miso = p0.p0_28.into_floating_input().degrade();
```

**DMA transfer internals (`do_spi_dma_transfer`):**

```rust
fn do_spi_dma_transfer(
    &mut self,
    tx: DmaSlice,
    rx: DmaSlice,
) -> Result<(), Error> {
    compiler_fence(SeqCst);

    // Set up EasyDMA pointers
    // SAFETY: PTR/MAXCNT fields accept raw values; the HAL's slice_in_ram()
    // check (above) ensures pointers target Data RAM. Task register write
    // of 1 triggers the SPIM START task per nRF52840 PS §6.1.2.
    self.0.txd.ptr.write(|w| unsafe { w.ptr().bits(tx.ptr) });
    self.0.txd.maxcnt.write(|w| unsafe { w.maxcnt().bits(tx.len as _) });
    self.0.rxd.ptr.write(|w| unsafe { w.ptr().bits(rx.ptr) });
    self.0.rxd.maxcnt.write(|w| unsafe { w.maxcnt().bits(rx.len as _) });

    // Trigger START task
    self.0.tasks_start.write(|w| unsafe { w.bits(1) });

    compiler_fence(SeqCst);

    // Poll for END event
    while self.0.events_end.read().bits() == 0 {}
    self.0.events_end.write(|w| w);  // Clear event

    // Verify transfer counts
    if self.0.txd.amount.read().bits() != tx.len {
        return Err(Error::Transmit);
    }
    if self.0.rxd.amount.read().bits() != rx.len {
        return Err(Error::Receive);
    }
    Ok(())
}
```

**RAM-only buffer validation:**

EasyDMA can only access Data RAM (`0x2000_0000` -- `0x3000_0000`). It
**cannot access Flash** (`0x0000_0000` -- `0x0010_0000`). The HAL checks
this at runtime:

```rust
const SRAM_LOWER: usize = 0x2000_0000;
const SRAM_UPPER: usize = 0x3000_0000;

pub(crate) fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= SRAM_LOWER && (ptr + slice.len()) < SRAM_UPPER
}
```

For transmit: if the buffer is in Flash, the HAL copies data in chunks to
a stack-allocated buffer (`FORCE_COPY_BUFFER_SIZE` = 1024 bytes on
nRF52840) before DMA. For receive: returns
`Error::DMABufferNotInDataMemory` if the buffer is not in RAM.

**embedded-hal 1.0 trait implementations:**

| HAL Type | Trait(s) |
|----------|----------|
| `Spim<T>` | `SpiBus<u8>` (read, write, transfer, transfer\_in\_place, flush) |
| `Pin<Input<...>>` | `InputPin` |
| `Pin<Output<...>>` | `OutputPin`, `StatefulOutputPin` |

The HAL also implements legacy `embedded-hal 0.2` traits (`blocking::spi::Transfer`,
`blocking::spi::Write`) for backwards compatibility.

**Error enum:**

```rust
pub enum Error {
    DMABufferNotInDataMemory,
    Transmit,
    Receive,
}
```

#### 2.3.7  no_alloc Usability

Both `nrf52840-pac` and `nrf52840-hal` are **fully `no_std`, no `alloc`**.
All types are stack-allocated or statically sized. The GPIO type-state
model is zero-cost at runtime. The HAL's `slice_in_ram` check and
stack-buffer copy for Flash data are the only runtime overhead vs. direct
PAC usage. This makes the ecosystem directly usable in the RTOS partition
environment where `alloc` is unavailable.

#### 2.3.8  Implications for the RTOS Driver Model

1. **EasyDMA simplifies the kernel's DMA model.** Unlike STM32 where the
   kernel must manage shared DMA controllers, each nRF52840 peripheral has
   its own DMA engine. The kernel can assign a SPIM instance to a partition
   without DMA channel arbitration.

2. **RAM-only constraint requires buffer management.** The kernel must
   ensure DMA buffers passed via SVC are in RAM, not Flash. The
   `slice_in_ram()` check should be part of the SVC pointer validation
   path. Partitions cannot pass `const` or `static` data from Flash
   directly to EasyDMA.

3. **Compiler fences are sufficient.** On single-core Cortex-M4F, no
   hardware memory barriers (DMB/DSB) are needed for DMA — only compiler
   fences. This is simpler than SAMD51's SYNCBUSY polling.

4. **Flexible pin muxing simplifies resource allocation.** No fixed
   AF tables or IoSet constraints means the kernel's pin manager does not
   need compile-time validation tables — any pin can be assigned to any
   peripheral signal at runtime.

5. **Task/event model maps well to kernel mediation.** The explicit
   trigger-and-poll pattern maps cleanly to an SVC-based driver model:
   partition issues `SYS_DEV_WRITE`, kernel sets up EasyDMA, triggers
   `TASKS_START`, and blocks the partition until `EVENTS_END`.

6. **SHORTS and PPI enable zero-latency hardware chaining.** The kernel
   can use SHORTS for continuous DMA (e.g., sensor sampling) without
   interrupt overhead. PPI can chain timer events to SPI starts for
   precise periodic transfers.

---

### 2.4  Cross-Family Comparison

The following table compares the three chip families across key dimensions
relevant to the RTOS driver model design.

| Dimension | STM32F4 | SAMD51 | nRF52840 |
|-----------|---------|--------|----------|
| **PAC crate** | `stm32f4` 0.16 | `atsamd51j` 0.14 | `nrf52840-pac` 0.12 |
| **HAL crate** | `stm32f4xx-hal` 0.22 | `atsamd-hal` 0.23 | `nrf52840-hal` 0.19 |
| **Core** | Cortex-M4F, 168 MHz | Cortex-M4F, 120 MHz | Cortex-M4F, 64 MHz |
| **Peripheral model** | Dedicated (SPI1, I2C1) | SERCOM multi-mode | Dedicated + task/event |
| **SPI data path** | Byte DR ± DMA | Byte DATA ± DMAC; LENGTH counter | EasyDMA only (no byte mode) |
| **Transfer trigger** | Write DR | Write DATA | Write `TASKS_START` |
| **Completion detect** | `SR.TXE`/`RXNE` bits | `INTFLAG.DRE`/`RXC` bits | `EVENTS_END` register |
| **Pin mux direction** | Pin → peripheral (AFRL) | Pin → peripheral (PMUX) | Peripheral → pin (PSEL) |
| **Pin mux constraints** | Fixed AF table per pin | PMUX + IoSet groups | None — any pin, any signal |
| **Clock gating** | RCC APBxENR | MCLK + GCLK generator | None (HFCLK default) |
| **Sync barriers** | None | SYNCBUSY polling | `compiler_fence` only |
| **DMA model** | Shared DMA1/DMA2 controller | Shared DMAC controller | Per-peripheral EasyDMA |
| **DMA buffer source** | Flash or RAM | Flash or RAM | **RAM only** |
| **DMA max transfer** | 65,535 (16-bit NDTR) | Configurable beat count | 65,535 (16-bit MAXCNT) |
| **HW event chaining** | DMA request lines | DMAC triggers | SHORTS + PPI |
| **HAL GPIO typing** | `Pin<MODE>` + AF generic | `Pin<Id, Mode>` + IoSet/PadNum | `Pin<MODE>` port-erased |
| **Type-level complexity** | Moderate | High | Low |
| **HAL SPI constructor** | `.spi((pins), mode, freq, &clocks)` | `Config::new(...).baud().enable()` | `Spim::new(spim, pins, freq, mode, orc)` |
| **SPI error variants** | Overrun, ModeFault, Crc | sercom::spi::Error | DMABufferNotInDataMemory, Transmit, Receive |
| **Async SPI** | Not in mainline HAL | `SpiFuture` + DMA | Not in mainline (embassy-nrf) |
| **embedded-hal** | 1.0 | 0.2 + 1.0 (dual) | 0.2 + 1.0 (dual) |
| **`no_std`/`no_alloc`** | Yes / Yes | Yes / Yes | Yes / Yes |

**Key takeaways for the RTOS driver model:**

1. **All three PAC crates share identical svd2rust APIs** —
   `Peripherals::take()`, `read()/write()/modify()` closures, field proxy
   methods. A PAC-level abstraction layer can use a common pattern.

2. **DMA models diverge significantly.** STM32 and SAMD51 use shared DMA
   controllers requiring arbitration; nRF52840's per-peripheral EasyDMA
   needs no arbitration but requires RAM-only buffers. The kernel's DMA
   manager must handle both models.

3. **Pin muxing varies from fully constrained to fully flexible.** STM32
   has fixed AF tables, SAMD51 has IoSet constraints, nRF52840 has none.
   A compile-time pin validation layer can be strict for STM32/SAMD51 and
   permissive for nRF52840.

4. **Sync barrier requirements differ.** Only SAMD51 needs SYNCBUSY
   polling. A driver trait's `configure()` method must accommodate this
   latency without blocking the entire kernel.

5. **The nRF52840 task/event model maps most naturally to an SVC-based
   driver interface** — explicit trigger, poll for completion, clear. The
   register-centric STM32/SAMD51 model requires the kernel to abstract
   the byte-at-a-time + DMA duality.

---

## 3  Architectural Approaches for the Partition-Facing Driver API

This section evaluates concrete approaches for how partition code
accesses hardware peripherals through the kernel.

### 3.1  Approach A — embedded-hal-async Facade

Partition code programs against `embedded_hal_async::spi::SpiDevice`,
`embedded_hal_async::i2c::I2c`, etc.  Each trait method is a thin proxy
that issues SVCs to the kernel.  The kernel side owns the PAC-based
driver behind the existing `VirtualDevice` trait
(`kernel/src/virtual_device.rs`).

#### 3.1.1  Partition-Side Proxy: SpiDevice over SVC

Each method translates to `svc!` macro calls using the existing ABI
(id in r0, args in r1–r3, return in r0).  Showing the key methods:

```rust
pub struct SpiProxy { device_id: u8 }

#[derive(Debug)]
pub struct SpiSvcError(u32);

impl spi::Error for SpiSvcError {
    fn kind(&self) -> spi::ErrorKind { spi::ErrorKind::Other }
}
impl ErrorType for SpiProxy { type Error = SpiSvcError; }

/// Helper: check SVC return for error bit.
fn check(rc: u32) -> Result<(), SpiSvcError> {
    if rc & 0x8000_0000 != 0 { Err(SpiSvcError(rc)) } else { Ok(()) }
}

impl SpiBus for SpiProxy {
    async fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        check(svc!(SYS_DEV_READ, self.device_id as u32,
                    buf.len() as u32, buf.as_mut_ptr() as u32))
    }
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        check(svc!(SYS_DEV_WRITE, self.device_id as u32,
                    data.len() as u32, data.as_ptr() as u32))
    }
    async fn transfer(&mut self, rd: &mut [u8], wr: &[u8])
        -> Result<(), Self::Error>
    {   // No single SVC for simultaneous TX+RX — two SVCs.
        self.write(wr).await?; self.read(rd).await
    }
    async fn transfer_in_place(&mut self, buf: &mut [u8])
        -> Result<(), Self::Error>
    {   check(svc!(SYS_DEV_IOCTL, self.device_id as u32,
                    IOCTL_XFER_INPLACE, buf.as_mut_ptr() as u32))
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        check(svc!(SYS_DEV_IOCTL, self.device_id as u32, IOCTL_FLUSH, 0u32))
    }
}

impl SpiDevice for SpiProxy {
    async fn transaction(&mut self, ops: &mut [Operation<'_, u8>])
        -> Result<(), Self::Error>
    {
        check(svc!(SYS_DEV_IOCTL, self.device_id as u32,
                    IOCTL_CS_ASSERT, 0u32))?;
        for op in ops.iter_mut() {
            match op {
                Operation::Read(buf)            => self.read(buf).await?,
                Operation::Write(data)          => self.write(data).await?,
                Operation::Transfer(rd, wr)     => self.transfer(rd, wr).await?,
                Operation::TransferInPlace(buf) => self.transfer_in_place(buf).await?,
                Operation::DelayNs(_)           => { /* spin or SYS_DELAY */ }
            }
        }
        check(svc!(SYS_DEV_IOCTL, self.device_id as u32,
                    IOCTL_CS_DEASSERT, 0u32))
    }
}
```

Key observations:

1. **`async fn` bodies are synchronous.** Each `svc!` traps into the
   kernel; `async` satisfies the trait signature only.  The returned
   futures resolve in a single poll — they never yield `Pending`.

2. **Stateless handle.** All peripheral state lives in the kernel.
   The proxy is parameterised only by `device_id`.

3. **Error type is opaque.** `SpiSvcError` wraps the raw u32 SVC
   return.  `kind()` returns `Other` because `DeviceError` variants
   (NotOpen, BufferFull) have no `spi::ErrorKind` equivalents.

#### 3.1.2  Kernel-Side Dispatch

An SPI backend implements the existing `VirtualDevice` trait.  The
`dev_dispatch` method (`kernel/src/svc.rs:484`) already routes by
`device_id` and maps `DeviceError` → `SvcError::OperationFailed`:

```rust
pub struct SpiBackend {
    device_id: u8,
    open_partitions: u8,  // bitmask, max 8 partitions
    // regs: &'static RegisterBlock,  // chip-specific PAC
}

impl VirtualDevice for SpiBackend {
    fn device_id(&self) -> u8 { self.device_id }
    fn open(&mut self, pid: u8) -> Result<(), DeviceError> {
        if pid >= 8 { return Err(DeviceError::InvalidPartition); }
        if self.open_partitions & (1 << pid) != 0 {
            return Err(DeviceError::AlreadyOpen);
        }
        self.open_partitions |= 1 << pid;
        Ok(())
    }
    fn close(&mut self, pid: u8) -> Result<(), DeviceError> { /* clear bit */ Ok(()) }
    fn write(&mut self, pid: u8, data: &[u8]) -> Result<usize, DeviceError> {
        self.require_open(pid)?;
        // PAC: write bytes to DR/DATA, poll TXE/DRE per chip family.
        Ok(data.len())
    }
    fn read(&mut self, pid: u8, buf: &mut [u8]) -> Result<usize, DeviceError> {
        self.require_open(pid)?;
        // PAC: clock out dummy bytes, read RXNE/RXC data.
        Ok(buf.len())
    }
    fn ioctl(&mut self, pid: u8, cmd: u32, arg: u32) -> Result<u32, DeviceError> {
        self.require_open(pid)?;
        match cmd {
            IOCTL_CS_ASSERT   => { /* GPIO low  */ Ok(0) }
            IOCTL_CS_DEASSERT => { /* GPIO high */ Ok(0) }
            IOCTL_FLUSH       => { /* wait TX empty */ Ok(0) }
            IOCTL_XFER_INPLACE => { /* full-duplex DR xfer */ Ok(0) }
            _ => Err(DeviceError::NotFound),
        }
    }
}
```

No changes to the dispatch plumbing are needed — only a new backend.

#### 3.1.3  Syscall Count Analysis: Write-1 / Read-4 SPI Transaction

Typical SPI sensor read: write 1 command byte, read 4 data bytes,
single CS assertion.

| Path | SVCs | Description |
|------|:----:|-------------|
| **A: Individual calls** | **4** | IOCTL(CS_ASSERT) + DEV_WRITE + DEV_READ + IOCTL(CS_DEASSERT) |
| **B: transaction()** | **4** | Same — proxy iterates ops, each issues an SVC |
| **C: Kernel-managed CS** | **2** | DEV_WRITE + DEV_READ (CS auto-asserted on open) |
| **D: Batched SYS_SPI_TRANSACTION** | **1** | Kernel receives Operation slice pointer, iterates internally |

Path D is optimal but couples the kernel to the `Operation` enum ABI.
Path C is simple but limits CS granularity.

**SVC cost:** ~60–80 cycles per round-trip on M4 (exception entry +
stacking + dispatch + return).  4 SVCs at 168 MHz ≈ 1.5 µs overhead —
acceptable for most SPI devices, significant for high-rate polling.

#### 3.1.4  Async / Blocking Boundary Analysis

The `embedded_hal_async` traits use `async fn` returning opaque
`impl Future`.  The RTOS uses synchronous SVC-based scheduling.
This creates a fundamental impedance mismatch.

**No async executor in partitions.** Partitions run bare-metal without
an async runtime.  Each `svc!` blocks synchronously (partition enters
`Waiting` state).  The `async` keyword is syntactic overhead — futures
resolve immediately.

**Waker plumbing across the SVC boundary is complex.** True async
would require passing a `Waker` (fat pointer: `*const RawWakerVTable`
+ data) across the SVC boundary.  Options and issues:

| Approach | Complexity | Issue |
|----------|------------|-------|
| Waker ptr in r1, vtable in r2 | Medium | Kernel must validate; vtable in partition memory |
| Kernel-assigned waker ID | Low | Needs waker registry; partition polls via `SYS_EVT_WAIT` |
| No true async (sync SVC) | **None** | Current model; recommended |

**Recommendation:** Keep synchronous SVCs.  If interrupt-driven
completion is needed later, reuse `SYS_EVT_WAIT` / `event_set` from
the ISR bottom-half rather than implementing a Waker protocol.

#### 3.1.5  Error Propagation: DeviceError → SpiError Mapping

`DeviceError` (9 variants, `virtual_device.rs:10`) → `dev_dispatch`
maps all to `SvcError::OperationFailed` → partition sees single u32 →
`SpiSvcError` → `ErrorKind::Other`.  Original semantics lost.

```
DeviceError::BufferFull → SvcError::OperationFailed (0xFFFF_FFFA)
  → partition r0 = 0xFFFF_FFFA → SpiSvcError → ErrorKind::Other
```

To preserve detail, encode `DeviceError` discriminant in the lower
bits of the SVC return: `0x8000_0000 | (svc_class << 16) | dev_variant`.
The lower 16 bits of `SvcError` codes are currently unused.

#### 3.1.6  DMA Integration Path

Kernel owns the DMA controller.  Flow: (1) partition calls
`SYS_DEV_WRITE` with data pointer; (2) kernel validates pointer via
`validated_ptr!` macro; (3) kernel configures DMA source/dest/count;
(4) kernel blocks partition (→ `Waiting`); (5) DMA-complete ISR sets
event flags via bottom-half; (6) scheduler resumes partition.

| Family | DMA Setup | Complexity |
|--------|-----------|:----------:|
| STM32F4 | DMA stream/channel + NDTR | High (shared DMA1/DMA2) |
| SAMD51 | DMAC channel + BTCNT + trigger | High (shared DMAC) |
| nRF52840 | TXD.PTR/MAXCNT + TASKS_START | Low (per-peripheral EasyDMA) |

The existing `BufferPool` (`buffer_pool.rs`, dynamic-mpu) provides
kernel-owned DMA-safe buffers.  **nRF52840 constraint:** EasyDMA
requires RAM-only buffers (`0x2000_0000..0x3000_0000`); pointer
validation must check this.

#### 3.1.7  Porting Effort per Chip Family

| Aspect | STM32F4 | SAMD51 | nRF52840 |
|--------|---------|--------|----------|
| VirtualDevice impl | ~200 lines | ~250 lines (SYNCBUSY) | ~150 lines |
| Clock setup | RCC APB enable | MCLK + GCLK | None |
| Pin mux | AFRL/AFRH | PMUX + PINCFG | PSEL registers |
| DMA | Stream/channel config | DMAC trigger routing | EasyDMA built-in |
| Extra validation | — | — | RAM-only buffer check |
| New syscalls | 0 (reuse SYS_DEV_*) | 0 | 0 |
| Partition proxy | ~80 lines (shared) | ~80 lines (shared) | ~80 lines (shared) |

The partition proxy is **chip-independent** — porting is kernel-only.

#### 3.1.8  Summary: Approach A Trade-offs

| Dimension | Assessment |
|-----------|------------|
| **API familiarity** | High — developers know embedded-hal-async |
| **dyn-compatibility** | **No** — async fn traits not object-safe (Section 1) |
| **Async benefit** | None — SVCs synchronous; async is syntactic overhead |
| **Syscall overhead** | 2–4 SVCs without batching; 1 with dedicated syscall |
| **Error fidelity** | Low — DeviceError → ErrorKind::Other |
| **Waker complexity** | High if attempted; skip recommended |
| **DMA path** | Feasible; per-family complexity varies |
| **Porting effort** | 150–250 lines kernel-side; 80 lines shared proxy |
| **Compile-time safety** | Low — device_id is runtime u8 |

**Critical issue:** `embedded_hal_async` traits are not dyn-compatible
(`async fn` returns opaque `impl Future`).  The kernel cannot store
`dyn SpiDevice`.  The partition proxy satisfies async trait signatures,
but the kernel must use `VirtualDevice` (dyn-compatible) internally.
The async facade is cosmetic — it pays the complexity cost of the async
surface without gaining async behavior, because SVCs are synchronous.

---

### 3.2  Approach B — Kernel-Owned embedded-hal + Virtual Device Mirror

The kernel itself implements `embedded_hal::spi::SpiDevice` (blocking)
using the chip's PAC directly.  Partitions do **not** see embedded-hal
traits at all.  Instead, each partition calls a single batched syscall
whose shape mirrors `SpiDevice::transaction` — sending an operation
descriptor that the kernel executes atomically against the real
hardware.  This eliminates per-operation SVC overhead by design.

#### 3.2.1  Kernel-Side: embedded-hal SPI Impl over PAC

The kernel owns a concrete `SpiMaster` that implements the blocking
`embedded_hal::spi::SpiDevice` trait.  This impl talks directly to
PAC registers; no SVC boundary is involved.

```rust
use embedded_hal::spi::{self, ErrorType, Operation, SpiDevice};

pub struct SpiMaster { /* regs: &'static pac::SPI1, cs: GpioPin */ }

#[derive(Debug)]
pub struct SpiHalError;
impl spi::Error for SpiHalError {
    fn kind(&self) -> spi::ErrorKind { spi::ErrorKind::Other }
}
impl ErrorType for SpiMaster { type Error = SpiHalError; }

impl SpiDevice for SpiMaster {
    fn transaction(
        &mut self, operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.cs_assert();
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    for b in buf.iter_mut() {
                        self.regs_write_dr(0xFF); // clock out dummy
                        self.wait_rxne();
                        *b = self.regs_read_dr();
                    }
                }
                Operation::Write(data) => {
                    for &b in data.iter() {
                        self.regs_write_dr(b);
                        self.wait_txe();
                    }
                    self.wait_not_busy();
                }
                Operation::Transfer(rd, wr) => { /* full-duplex DR xfer */ }
                Operation::TransferInPlace(buf) => { /* in-place DR xfer */ }
                Operation::DelayNs(ns) => {
                    cortex_m::asm::delay(ns_to_cycles(*ns));
                }
            }
        }
        self.cs_deassert();
        Ok(())
    }
}
```

Key properties:  (1) **Standard blocking embedded-hal** — any driver
crate accepting `impl SpiDevice` works unmodified in the kernel.
(2) **dyn-compatible** — blocking traits avoid the `async fn` object-
safety problem; the kernel can store `&mut dyn SpiDevice<Error=E>`.
(3) **Full hardware control** — CS, timing, busy-waits all kernel-side.

#### 3.2.2  Partition-Side: Virtual Device Batched Syscall API

Partitions do not use embedded-hal traits.  They use a kernel-native
API that mirrors the *shape* of `SpiDevice::transaction` as a single
batched syscall.

```rust
/// Operation descriptor passed from partition to kernel.
/// Mirrors embedded_hal::spi::Operation but uses raw pointers
/// for cross-partition transfer via the SVC ABI.
#[repr(C)]
pub enum SpiOp {
    Read  { buf: *mut u8, len: u16 },
    Write { data: *const u8, len: u16 },
    Transfer { rd: *mut u8, wr: *const u8, len: u16 },
    TransferInPlace { buf: *mut u8, len: u16 },
    DelayNs(u32),
}

/// Partition-side SPI handle — analogous to SpiProxy from Approach A
/// but the transaction is a single SVC, not per-operation.
pub struct SpiHandle { device_id: u8 }

/// Error type for partition SPI calls.
#[derive(Debug)]
pub struct SpiError(u32);

impl SpiHandle {
    /// Execute a complete SPI transaction as a single syscall.
    /// The kernel asserts CS, executes all operations against real
    /// hardware, deasserts CS, and returns the result.
    ///
    /// Mirrors SpiDevice::transaction() but is RTOS-native:
    /// one SVC regardless of operation count.
    pub fn transaction(&self, ops: &mut [SpiOp]) -> Result<(), SpiError> {
        // r1 = device_id, r2 = ops.len(), r3 = ops.as_ptr()
        let rc = svc!(SYS_SPI_TRANSACTION,
                       self.device_id as u32,
                       ops.len() as u32,
                       ops.as_mut_ptr() as u32);
        if rc & 0x8000_0000 != 0 { Err(SpiError(rc)) } else { Ok(()) }
    }

    /// Convenience: write then read in one transaction (common
    /// sensor pattern: write command byte, read N data bytes).
    pub fn write_then_read(
        &self, cmd: &[u8], response: &mut [u8],
    ) -> Result<(), SpiError> {
        let mut ops = [
            SpiOp::Write { data: cmd.as_ptr(), len: cmd.len() as u16 },
            SpiOp::Read { buf: response.as_mut_ptr(),
                          len: response.len() as u16 },
        ];
        self.transaction(&mut ops)
    }
}
```

The kernel-side handler for `SYS_SPI_TRANSACTION`:

1. Validates the `ops` slice pointer via `validate_user_ptr`.
2. Validates each individual buffer pointer within `SpiOp` entries.
3. Translates `SpiOp` entries to `embedded_hal::spi::Operation`.
4. Calls `SpiMaster::transaction()` (the real embedded-hal impl).
5. Returns success or maps `SpiHalError` to an error code.

#### 3.2.3  Syscall Count Analysis: Write-1 / Read-4 SPI Transaction

Typical SPI sensor read: write 1 command byte, read 4 data bytes,
single CS assertion.  Comparing with Approach A:

| Approach | SVCs | Description |
|----------|:----:|-------------|
| **A: SpiProxy::transaction()** | **4** | CS_ASSERT + DEV_WRITE + DEV_READ + CS_DEASSERT |
| **A: Batched SYS_SPI_TRANSACTION** | **1** | Hypothetical; couples kernel to Operation ABI |
| **B: SpiHandle::transaction()** | **1** | Kernel receives SpiOp slice, executes all internally |
| **B: write_then_read()** | **1** | Convenience wrapper; same single SVC |

**Approach B achieves 1 SVC by design**, not as an optimization added
later.  The API *is* the batched transaction — there are no individual
read/write syscalls to accidentally use.

**SVC cost comparison** at 168 MHz Cortex-M4 (~70 cycles/SVC):

| | Approach A (4 SVCs) | Approach B (1 SVC) |
|---|:---:|:---:|
| Overhead | ~280 cycles / 1.7 µs | ~70 cycles / 0.4 µs |
| % of 1 MHz SPI transfer (5 bytes) | 0.7% | 0.2% |

The absolute savings are modest for slow SPI.  For high-rate polling
(accelerometer at 1 kHz, 6-byte reads), Approach A costs 4000 SVCs/s
vs. Approach B's 1000 SVCs/s — a 4× reduction in dispatch overhead.

#### 3.2.4  Async / Blocking Boundary Analysis

**No impedance mismatch.**  Approach B uses blocking embedded-hal on
the kernel side and blocking SVCs on the partition side — the boundary
is naturally synchronous in both directions.

| Aspect | Approach A | Approach B |
|--------|-----------|-----------|
| Partition API | `async fn` (resolves immediately) | Blocking fn |
| Kernel impl | VirtualDevice (blocking) | embedded-hal blocking |
| Async executor needed | No (async is cosmetic) | No |
| Waker plumbing | N/A (not truly async) | N/A |

**Interrupt-driven completion:** For long DMA transfers, the kernel
blocks the calling partition (transitions to `Waiting`), starts DMA,
and the DMA-complete ISR wakes the partition via `event_set` through
the existing bottom-half path.  The partition's `transaction()` SVC
returns only after the entire operation completes — no partial results,
no multi-SVC state to track.

**True async extension:** If partition-level async is ever needed, it
is simpler under Approach B: one SVC → one wakeup, versus Approach A's
4 SVCs each needing coordination.

#### 3.2.5  Error Propagation: HAL Errors → SvcError

The kernel translates HAL-level errors to RTOS error codes at a
single point — the `SYS_SPI_TRANSACTION` handler.

```
SpiHalError → SvcError::OperationFailed (0xFFFF_FFFA)
  → partition r0 = 0xFFFF_FFFA → SpiError(0xFFFF_FFFA)
```

Extended error encoding (same scheme as Approach A §3.1.5):

```
0x8000_0000 | (SVC_CLASS_SPI << 16) | hal_error_variant
```

| Error Source | Approach A | Approach B |
|---|---|---|
| HAL hardware error | Mapped through VirtualDevice → DeviceError → SvcError | Mapped directly from SpiHalError → SvcError |
| Pointer validation | SvcError::InvalidPointer per SVC | SvcError::InvalidPointer once |
| Device not open | Per-SVC DeviceError::NotOpen | Once at transaction start |
| Partial failure | Mid-transaction; CS may be stuck | Kernel handles cleanup (CS deassert in drop/error path) |

**Approach B advantage:** error handling is atomic.  If byte 3 of a
5-byte read fails, the kernel deasserts CS and returns an error.  In
Approach A, the partition must handle mid-transaction errors across
multiple SVC calls while CS is asserted — this is error-prone and
can leave CS stuck low if the partition panics between SVCs.

#### 3.2.6  DMA Integration

DMA is straightforward under Approach B because the kernel owns both
the SPI peripheral and the DMA controller.  Flow: partition calls
`SYS_SPI_TRANSACTION` → kernel validates all buffer pointers once →
asserts CS → for large ops, configures DMA and blocks partition →
DMA-complete ISR wakes via bottom-half → deasserts CS → returns.

| Aspect | Approach A | Approach B |
|--------|-----------|-----------|
| DMA setup per SVC | Yes — each DEV_WRITE/READ reconfigures DMA | No — kernel batches |
| CS held across DMA | Partition trusts kernel across 4 SVCs | Kernel holds CS natively |
| Buffer validation | Per-SVC (4×) | Once at transaction start |
| Scatter-gather | Requires per-op DMA; no cross-op batching | Kernel can chain DMA descriptors across ops |

The kernel's `BufferPool` (dynamic-mpu feature) provides DMA-safe
buffers directly.  nRF52840 EasyDMA RAM-only constraints are checked
once at transaction start rather than per-SVC.

#### 3.2.7  Porting Effort per Chip Family

| Aspect | STM32F4 | SAMD51 | nRF52840 |
|--------|---------|--------|----------|
| Kernel SpiMaster impl | ~200 lines | ~250 lines (SYNCBUSY) | ~150 lines |
| embedded-hal SpiDevice | Yes — standard trait | Yes | Yes |
| Clock setup | RCC APB enable | MCLK + GCLK | None |
| Pin mux | AFRL/AFRH | PMUX + PINCFG | PSEL registers |
| DMA | Stream/channel config | DMAC trigger routing | EasyDMA built-in |
| Extra validation | — | — | RAM-only buffer check |
| New syscalls | 1 (SYS_SPI_TRANSACTION) | 1 (shared) | 1 (shared) |
| Partition API change | **None** | **None** | **None** |

Porting requires a new `SpiMaster` impl using that chip's PAC.
The partition-side `SpiHandle` and `SpiOp` are **chip-independent**.
Unlike Approach A's `VirtualDevice` (RTOS-specific), `SpiMaster` can
leverage existing HAL crates (stm32f4xx-hal, atsamd-hal, nrf-hal).

#### 3.2.8  Summary: Approach B Trade-offs

| Dimension | Assessment |
|-----------|------------|
| **API familiarity** | Medium — partition API is RTOS-native, not embedded-hal |
| **dyn-compatibility** | **Yes** — blocking SpiDevice is object-safe |
| **Async benefit** | N/A — no async surface; blocking end-to-end |
| **Syscall overhead** | **1 SVC per transaction** (vs. 2–4 in Approach A) |
| **Error fidelity** | High — single translation point; atomic cleanup |
| **Waker complexity** | None |
| **DMA path** | **Straightforward** — kernel owns hardware + DMA natively |
| **Porting effort** | 150–250 lines kernel-side; 0 lines partition-side |
| **Compile-time safety** | Medium — SpiOp is typed; device_id is runtime u8 |
| **Ecosystem reuse** | **High** — kernel can use existing HAL crates + driver crates |
| **Transaction atomicity** | **Yes** — CS held by kernel throughout; error cleanup guaranteed |

**Contrast with Approach A:**

| | Approach A | Approach B |
|---|---|---|
| Partition sees | embedded-hal-async traits | RTOS-native batched API |
| SVCs per transaction | 2–4 (or 1 with dedicated batched SVC) | 1 (by design) |
| Kernel driver layer | VirtualDevice (custom trait) | embedded-hal (standard trait) |
| Existing HAL crate reuse | Not directly (must wrap in VirtualDevice) | Direct (SpiMaster can delegate) |
| Transaction atomicity | Partition-managed (fragile) | Kernel-managed (robust) |
| async fn in partition | Yes (cosmetic) | No |
| dyn dispatch in kernel | VirtualDevice (dyn-compatible) | SpiDevice (dyn-compatible) |

### 3.3  Approach C — Pure RTOS-Native Syscall API

Approach C abandons embedded-hal trait shapes entirely — both at the
partition surface (unlike A) and inside the kernel (unlike B).
Partitions issue purpose-built syscalls defined by the RTOS.  The
kernel dispatches each syscall to a chip-specific PAC driver with no
trait abstraction layer.

The existing `VirtualDevice` trait (`kernel/src/virtual_device.rs`)
is a **partial implementation** of this philosophy: its RTOS-native
`read`/`write`/`ioctl` already bypass embedded-hal.  Approach C goes
further by defining **peripheral-class-specific** syscalls (SPI, I2C,
UART) that encode full operations in a single SVC.

#### 3.3.1  RTOS-Native SPI Syscall: `SYS_SPI_TRANSACT`

A single syscall encodes an entire SPI transaction — command byte(s),
response buffer, transfer length, and behavioral flags — in one SVC.
No operation descriptors, no embedded-hal `Operation` enum, no
per-byte SVC overhead.

```rust
// ── Syscall number (kernel/src/syscall.rs) ──────────────────────
pub const SYS_SPI_TRANSACT: u32 = 30;

// ── SVC ABI register layout ─────────────────────────────────────
// r0 = SYS_SPI_TRANSACT (call number)
// r1 = device_id (u8, zero-extended)
// r2 = descriptor pointer → SpiTransactDesc (validated by kernel)
// r3 = flags (SPI_FLAG_* bitmask)

/// Flags for SPI transact behavior.
pub const SPI_FLAG_WRITE_ONLY: u32 = 0x0001; // ignore rx_buf
pub const SPI_FLAG_READ_ONLY: u32  = 0x0002; // tx sends 0xFF clocks
pub const SPI_FLAG_FULL_DUPLEX: u32 = 0x0000; // default: simultaneous tx+rx
pub const SPI_FLAG_CS_HOLD: u32    = 0x0010; // do not deassert CS after

/// Descriptor passed from partition to kernel via validated pointer.
/// Lives in the partition's MPU data region.
#[repr(C)]
pub struct SpiTransactDesc {
    pub tx_buf: *const u8,  // source data (or null for read-only)
    pub rx_buf: *mut u8,    // destination (or null for write-only)
    pub len: u16,           // transfer length in bytes
    pub _reserved: u16,     // pad to 4-byte alignment
}
```

Key difference from Approaches A and B: there is no `Operation` enum
or operation slice.  The syscall descriptor is a flat, fixed-size
struct.  For the common "write command, read response" pattern, the
caller sets `tx_buf` to the command bytes, `rx_buf` to the response
buffer, and `len` to the total frame length.  The kernel handles
CS assertion, clocking, and cleanup internally.

#### 3.3.2  Partition-Side Typed Wrapper

Partitions use a typed wrapper that constructs the descriptor and
invokes the SVC.  No embedded-hal traits — the API is RTOS-native.

```rust
/// Partition-side SPI handle.  No embedded-hal traits.
pub struct Spi { device_id: u8 }

impl Spi {
    pub const fn new(device_id: u8) -> Self { Self { device_id } }

    /// Full-duplex transfer: write tx_data while reading into rx_buf.
    /// Returns InvalidArgument if lengths differ or exceed u16::MAX.
    pub fn transact(
        &self, tx_data: &[u8], rx_buf: &mut [u8], flags: u32,
    ) -> Result<(), SvcError> {
        if tx_data.len() != rx_buf.len() {
            return Err(SvcError::InvalidArgument);
        }
        self.transact_raw(tx_data.as_ptr(), rx_buf.as_mut_ptr(),
                          tx_data.len(), flags)
    }

    /// Write-only: send data, discard received bytes.
    pub fn write(&self, data: &[u8]) -> Result<(), SvcError> {
        self.transact_raw(data.as_ptr(), core::ptr::null_mut(),
                          data.len(), SPI_FLAG_WRITE_ONLY)
    }

    /// Read-only: clock out 0xFF, capture received bytes.
    pub fn read(&self, buf: &mut [u8]) -> Result<(), SvcError> {
        self.transact_raw(core::ptr::null(), buf.as_mut_ptr(),
                          buf.len(), SPI_FLAG_READ_ONLY)
    }

    fn transact_raw(
        &self, tx: *const u8, rx: *mut u8, len: usize, flags: u32,
    ) -> Result<(), SvcError> {
        let len = u16::try_from(len)
            .map_err(|_| SvcError::InvalidArgument)?;
        let desc = SpiTransactDesc { tx_buf: tx, rx_buf: rx,
                                     len, _reserved: 0 };
        SvcError::from_u32(svc!(SYS_SPI_TRANSACT,
            self.device_id as u32,
            &desc as *const _ as u32, flags))
    }
}
```

Error propagation uses `SvcError` directly — no intermediate
`DeviceError` or `SpiHalError`.  The kernel returns the same error
codes used by every other syscall (`InvalidPointer`,
`InvalidResource`, `OperationFailed`).

#### 3.3.3  Kernel Dispatch to PAC Driver

The kernel handler validates inputs and drives the SPI peripheral
via PAC register access — no embedded-hal trait indirection.

```rust
/// Timeout (in poll iterations) for SPI status-flag polling.
/// Sized for worst-case at lowest SPI clock / highest core clock.
const SPI_POLL_TIMEOUT: u32 = 100_000;

fn handle_spi_transact(
    &mut self, pid: u8, desc_ptr: u32, flags: u32,
) -> Result<u32, SvcError> {
    // 1. Validate descriptor pointer in partition's MPU region
    validate_user_ptr(&self.partitions, pid,
                      desc_ptr, core::mem::size_of::<SpiTransactDesc>())?;
    // SAFETY: pointer validated against partition's MPU data region
    let desc = unsafe { &*(desc_ptr as *const SpiTransactDesc) };
    let len = desc.len as usize;

    // 2. Validate tx_buf / rx_buf if non-null
    if !desc.tx_buf.is_null() {
        validate_user_ptr(&self.partitions, pid, desc.tx_buf as u32, len)?;
    }
    if !desc.rx_buf.is_null() {
        validate_user_ptr(&self.partitions, pid, desc.rx_buf as u32, len)?;
    }

    // 3. Drive SPI via PAC registers (no trait)
    let spi = &self.spi_regs;
    self.cs_pin_assert();
    for i in 0..len {
        let tx_byte = if flags & SPI_FLAG_READ_ONLY != 0 || desc.tx_buf.is_null() {
            0xFF
        } else {
            // SAFETY: tx_buf validated, i < len
            unsafe { *desc.tx_buf.add(i) }
        };
        spi.dr.write(|w| w.dr().bits(tx_byte as u16));
        // Wait for RXNE with timeout to avoid hanging on hardware faults
        let mut timeout = SPI_POLL_TIMEOUT;
        while spi.sr.read().rxne().bit_is_clear() {
            timeout -= 1;
            if timeout == 0 {
                self.cs_pin_deassert();
                return Err(SvcError::OperationFailed);
            }
        }
        let rx_byte = spi.dr.read().dr().bits() as u8;
        if flags & SPI_FLAG_WRITE_ONLY == 0 && !desc.rx_buf.is_null() {
            // SAFETY: rx_buf validated, i < len
            unsafe { *desc.rx_buf.add(i) = rx_byte; }
        }
    }
    // Wait for BSY clear with timeout
    let mut timeout = SPI_POLL_TIMEOUT;
    while spi.sr.read().bsy().bit_is_set() {
        timeout -= 1;
        if timeout == 0 {
            self.cs_pin_deassert();
            return Err(SvcError::OperationFailed);
        }
    }
    if flags & SPI_FLAG_CS_HOLD == 0 { self.cs_pin_deassert(); }
    Ok(len as u32)
}
```

The kernel talks to SPI registers directly (`spi.dr`, `spi.sr`) —
the kernel *is* the driver.  No `SpiDevice` trait, no `VirtualDevice`
dispatch, no `dyn` indirection.

#### 3.3.4  Syscall Count Analysis: Write-1 / Read-4 SPI Transaction

| Approach | SVCs | Description |
|----------|:----:|-------------|
| **A: SpiProxy::transaction()** | **4** | CS_ASSERT + DEV_WRITE + DEV_READ + CS_DEASSERT |
| **B: SpiHandle::transaction()** | **1** | Kernel receives SpiOp slice, executes against embedded-hal impl |
| **C: Spi::transact()** | **1** | Kernel receives flat descriptor, drives PAC registers directly |
| **C: Spi::write() + Spi::read()** | **2** | Two separate SVCs; still fewer than A |

Approaches B and C both achieve 1 SVC for the common batched case.
Approach C's flat descriptor is simpler to validate (fixed-size struct
vs. variable-length `SpiOp` slice) but less flexible for multi-phase
transactions that mix read/write/delay segments.

For the multi-phase case, Approach C can add a `SYS_SPI_TRANSACT_BATCH`
that accepts a descriptor array — but this is an opt-in extension,
not the default API.

#### 3.3.5  Async Handling: Kernel-Controlled, Partition Blocks

**No async at the partition level.**  The partition issues a blocking
SVC; the kernel decides how to complete it (polled or DMA).

| Aspect | Approach A | Approach B | Approach C |
|--------|-----------|-----------|-----------|
| Partition API | `async fn` (cosmetic) | Blocking fn | Blocking fn |
| Kernel impl | VirtualDevice | embedded-hal blocking | PAC registers directly |
| Async executor | Not needed | Not needed | Not needed |
| Long-transfer | Busy-waits across SVCs | Kernel blocks, DMA | Kernel blocks, DMA |

For DMA: kernel validates pointers → transitions partition to
`Waiting` → configures DMA from descriptor → starts transfer →
DMA-complete ISR wakes partition via bottom-half → `transact()`
returns.  No intermediate states, no waker, no multi-SVC coordination.

#### 3.3.6  Error Propagation: `SvcError` Directly

Approach C eliminates all intermediate error types: `PAC register
status → SvcError variant → partition r0`.

| Error condition | Approach A | Approach B | Approach C |
|---|---|---|---|
| SPI bus overrun | PAC→SpiHalError→DeviceError→SvcError | PAC→SpiHalError→SvcError | PAC→**SvcError** |
| Invalid pointer | InvalidPointer per SVC | InvalidPointer once | InvalidPointer once |
| Device not open | DeviceError::NotOpen per SVC | DeviceError once | **SvcError::InvalidResource** |

The partition returns `Result<(), SvcError>` — the same error type
used by every other RTOS syscall.  Zero error-type proliferation.

#### 3.3.7  DMA Integration: Kernel Has Full Control

No abstraction layer between syscall handler and DMA controller.

| Aspect | Approach A | Approach B | Approach C |
|--------|-----------|-----------|-----------|
| DMA config | Through VirtualDevice | Through SpiDevice impl | **Direct PAC DMA registers** |
| Buffer validation | Per SVC (4×) | Once at transaction | Once from descriptor |
| Scatter-gather | Per-op only | Kernel chains across ops | **Descriptor→DMA directly** |
| Zero-copy BufferPool | Via DeviceWrite | Via SpiMaster | **Pool slot→DMA directly** |

`SpiTransactDesc` maps directly to DMA config (`tx_buf`→source,
`rx_buf`→dest, `len`→count) with no `Operation`/`SpiOp` translation.
`BufferPool` zero-copy: kernel detects pool pointers and feeds slot
addresses directly to the DMA controller.

#### 3.3.8  Porting Effort: Kernel Driver per Chip, Partition API Stable

| Aspect | STM32F4 | SAMD51 | nRF52840 |
|--------|---------|--------|----------|
| Kernel SPI driver | ~150 lines | ~200 lines (SYNCBUSY) | ~120 lines (EasyDMA) |
| Uses embedded-hal / HAL crate | **No** (PAC only) | **No** (PAC only) | **No** (PAC only) |
| New syscalls per peripheral | 1 (`SYS_SPI_TRANSACT`) | 1 (shared) | 1 (shared) |
| Partition API change | **None** | **None** | **None** |

Porting cost comparable to Approach B (150–200 lines) but simpler —
no `SpiDevice` trait to implement.  Downside: HAL crates cannot be
reused; every kernel driver is PAC-only.  Partition-side `Spi` and
`SpiTransactDesc` are **chip-independent** RTOS API types.

#### 3.3.9  Connection to Existing `VirtualDevice` Trait

The `VirtualDevice` trait (`kernel/src/virtual_device.rs`) is a
**partial implementation** of Approach C.  It provides RTOS-native
`read`/`write`/`ioctl` (not embedded-hal), partition-aware
`open`/`close` access control, `DeviceRegistry` dispatch, and
`SYS_DEV_*` syscall wiring.

Approach C extends this by adding **peripheral-class-specific**
syscalls with structured descriptors (vs. generic byte-stream
read/write), single-SVC transactions (vs. multi-SVC), `SvcError`
directly (vs. `DeviceError`), and direct PAC access (vs. `dyn
VirtualDevice`).

Migration path: keep `VirtualDevice` for byte-stream devices (UART,
GPIO); add `SYS_SPI_TRANSACT`, `SYS_I2C_TRANSACT` for protocols
where transaction semantics matter.

#### 3.3.10  Summary: Approach C Trade-offs

| Dimension | Assessment |
|-----------|------------|
| **API familiarity** | Low — fully RTOS-specific; no embedded-hal knowledge transfers |
| **dyn-compatibility** | N/A — no traits at partition boundary; syscalls are static dispatch |
| **Async benefit** | N/A — no async surface; kernel controls all timing |
| **Syscall overhead** | **1 SVC per transaction** (flat descriptor, no operation slice) |
| **Error fidelity** | **Highest** — SvcError directly, zero intermediate types |
| **Waker complexity** | None |
| **DMA path** | **Simplest** — descriptor maps directly to DMA configuration |
| **Porting effort** | 120–200 lines kernel-side; 0 lines partition-side; no HAL crate reuse |
| **Compile-time safety** | High — typed wrapper + descriptor struct; device_id is runtime u8 |
| **Ecosystem reuse** | **None** — cannot use embedded-hal driver crates in kernel |
| **Transaction atomicity** | **Yes** — kernel owns entire operation lifecycle |

**Three-way comparison:**

| | Approach A | Approach B | Approach C |
|---|---|---|---|
| Partition sees | embedded-hal-async traits | RTOS-native batched API (SpiOp slice) | RTOS-native flat descriptor |
| SVCs per transaction | 2–4 | 1 | 1 |
| Kernel driver uses | VirtualDevice trait | embedded-hal trait (blocking) | PAC registers directly |
| Embedded-hal crate reuse | Partition-side (async proxy) | Kernel-side (HAL crates) | **Neither** |
| Error type chain | DeviceError → SvcError | SpiHalError → SvcError | **SvcError only** |
| DMA complexity | Per-SVC reconfiguration | Through SpiDevice impl | **Direct descriptor→DMA** |
| Transaction descriptor | Operation enum slice | SpiOp enum slice | Flat `SpiTransactDesc` struct |
| Porting requires | VirtualDevice impl + SpiProxy | SpiMaster (embedded-hal) impl | PAC driver (no trait) |
| Existing VirtualDevice | Is the backend | Replaced by SpiMaster | **Extended** with per-peripheral syscalls |
