# HAL Design Review — Panel Member 09: James Munns
**Ferrous Systems | embedded-hal maintainer, no_std architecture**

---

## Scope

Reviewed: `crates/meridian-hal/src/` (all 12 files), `crates/meridian-drivers/src/` (lib.rs + imu_icm426xx.rs, baro_dps310.rs, compass_ist8310.rs, imu_bmi270.rs, gps_ublox.rs), `crates/meridian-platform-sitl/src/lib.rs`.

---

## Executive Summary

This is a competent ArduPilot-to-Rust port that gets many things right. It is `no_std`, avoids heap allocation in driver logic, uses type-associated implementations correctly, and the driver separation from the HAL is clean. However, several design decisions diverge from the embedded-hal ecosystem in ways that will cause real pain when this runs on real metal. The Semaphore trait is the most structurally concerning piece. The SpiDevice zero-cost abstraction question has a nuanced answer that depends on how callers are compiled. Overall this is a solid PROTOTYPE that needs targeted rework before I would call it PRODUCTION.

---

## 1. Overall HAL Architecture

**Rating: PROTOTYPE**

### What works

The `Hal` supertrait as an associated-type-based platform bundle is idiomatic Rust. Using `type Uart: UartDriver` rather than `Box<dyn UartDriver>` means the whole platform can be monomorphised at compile time with zero dispatch overhead — that is the right shape for embedded.

The crate is correctly gated `#![no_std]` and the Cargo.toml shows no `std` feature leakage into the trait definitions. The `std` feature exists only as an optional propagation flag to downstream crates.

The 1:1 mapping from ArduPilot's `AP_HAL.h` is a valid porting strategy. It is not idiomatic embedded-hal, but it is auditable against a known-good reference and ArduPilot's semantics are reasonably well understood.

### What needs work

**The `Hal` trait returns shared references for everything.** `fn spi(&self, bus: u8) -> Option<&Self::Spi>` returns `&SpiBus`, not `&mut SpiBus`. Getting a `SpiDevice` from a `SpiBus` via `get_device` does return an owned `Self::Device`, so exclusive access is recovered there, but the chain is awkward. If a second caller requests the same bus before the first device is dropped, the semaphore is the only thing preventing a collision — which pushes correctness enforcement from the type system into runtime state. This is exactly what embedded-hal 1.x's `SpiDevice` split was designed to avoid.

**Bus index lookups are runtime-indexed with `u8`.** In ArduPilot this was necessary because the HAL was a C++ global singleton. In Rust, bus indices can be type-level constants or zero-sized tokens. Keeping them as `u8` means no compile-time guarantee that bus 7 doesn't exist on a platform that only has 3.

---

## 2. SpiDevice Trait — Zero-Cost Abstraction Analysis

**Rating: PROTOTYPE**

```rust
pub trait SpiDevice {
    fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool;
    // ...
    fn get_semaphore(&self) -> &dyn Semaphore;
}
```

The drivers call `SpiDevice` through `&mut dyn SpiDevice`:

```rust
pub fn probe(&mut self, spi: &mut dyn SpiDevice) -> Result<Icm426xxVariant, Icm426xxError>
```

This is a dynamic dispatch call site. Every `spi.write_register(...)` becomes an indirect vtable call. On Cortex-M this is typically 4–6 cycles overhead per call plus icache pressure. For a 1 kHz IMU FIFO drain that does 2–3 SPI transactions per call, this is measurable but probably not catastrophic.

The more significant zero-cost question is `read_registers`. The default implementation allocates a 64-byte stack buffer and a second 64-byte rx buffer for every multi-register read:

```rust
fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> bool {
    let mut tx = [0u8; 64];
    let mut rx = [0u8; 64];
    let len = buf.len() + 1;
    if len > 64 { return false; }
    // ...
}
```

128 bytes of stack pressure per multi-register read is significant on a Cortex-M7 with 128 KB DTCM and multiple concurrent ISR stacks. More critically, a hard limit of 63 readable bytes per call is not documented in the trait signature — it is silently violated if the caller passes a buffer of 64 bytes or longer. A `const MAX_TRANSFER: usize` associated constant, or panicking with a clearer message, would be the right shape here. On a real IMU driver doing 16-byte FIFO burst reads this is fine, but the hazard is latent for anyone reading more.

**Can it be zero-cost?** If you monomorphise: `fn probe<S: SpiDevice>(spi: &mut S) -> ...` then yes, fully inlined, all overhead eliminated. The current `dyn SpiDevice` approach trades flexibility for performance. For SITL and testing this is acceptable. For a Cortex-M7 IMU driver running at 1 kHz, I'd want monomorphised generics or at minimum a cfg-gated no-dyn mode.

**Verdict**: The trait is zero-cost abstractable in principle; the current call sites do not take advantage of it. Mark as PROTOTYPE.

---

## 3. I2cDevice Trait

**Rating: PROTOTYPE**

The design is clean. The `set_address` pattern matches ArduPilot and is common in embedded C. However it introduces a hidden state hazard: the address is mutable state on the device, so:

```rust
fn probe(i2c: &mut dyn I2cDevice) -> Result<(), Ist8310Error> {
    i2c.set_address(IST8310_ADDR);   // side effect on shared bus object
    let id = i2c.read_register(...)?;
```

If two drivers share the same `I2cDevice` instance (e.g., two sensors on one I2C bus), the address must be re-set before every operation. This is enforced by convention in the drivers (`i2c.set_address(...)` at the top of every pub fn), but is not enforced by the type system.

The preferred embedded-hal 1.x pattern is to bake the address into the device type at construction time (`I2cDevice` wraps `(Bus, Addr)`) so it is impossible to forget. That pattern also eliminates the shared-mutable-state issue at compile time.

The `probe` default implementation is a minor correctness concern:

```rust
fn probe(&mut self) -> bool {
    let mut dummy = [0u8; 1];
    self.transfer(&[], &mut dummy)
}
```

Sending an empty write followed by a 1-byte read is not a reliable I2C probe on all devices — some chips NAK zero-length writes, some NAK reads to unimplemented registers. The ArduPilot convention of probing by WHO_AM_I is more reliable, which the drivers do correctly anyway. The default trait method is misleading dead code.

---

## 4. GPIO Trait

**Rating: PROTOTYPE**

```rust
pub trait GpioPin {
    fn set_mode(&mut self, pin: u16, mode: PinMode);
    fn read(&self, pin: u16) -> bool;
    fn write(&mut self, pin: u16, value: bool);
    fn toggle(&mut self, pin: u16);
    fn attach_interrupt(&mut self, pin: u16, edge: Edge, callback: fn(u16, bool)) -> bool;
```

The `GpioPin` trait represents a port controller, not a pin. Every operation takes a `pin: u16` index argument. This is an ArduPilot-ism that loses type-safety: nothing prevents passing pin 999 to a platform that has 16 GPIO lines. The canonical embedded-hal approach is one type per pin (or per port+pin tuple), making invalid pins unrepresentable.

The `callback: fn(u16, bool)` for interrupt attachment is a bare function pointer. This cannot capture any state. In practice, interrupt handlers on Cortex-M need to write into a static or shared structure. Using a bare fn limits the expressiveness substantially compared to what RTIC's `#[task]` or `critical_section`-guarded statics can achieve. This is workable, but it means every interrupt consumer must reach through a global.

The `Alternate(u8)` variant in `PinMode` is a platform-specific concept (STM32 alternate function numbers 0–15). Putting it in the shared HAL trait leaks implementation detail. A platform-agnostic HAL should represent this as "this pin is controlled by peripheral X" without encoding the MCU-specific AF number.

---

## 5. UART Trait

**Rating: PROTOTYPE**

The interface is complete and recognisable. A few embedded-specific notes:

`fn available(&self) -> u16` and `fn txspace(&self) -> u16` return `u16`. On a Cortex-M with DMA ring buffers you might have 4 KB FIFO entries — `u16` is sufficient but tight. Not a blocking issue.

`fn begin(&mut self, baud: u32, rx_buf: u16, tx_buf: u16)` takes buffer sizes as arguments. In a `no_std` environment, there is no allocator to back these. The trait implies the implementation handles its own buffer storage, which is fine, but the signature suggests dynamic sizing that embedded implementations cannot fulfil. It should document that `rx_buf`/`tx_buf` are advisory hints and the implementation may use compile-time-fixed buffers.

The optional methods with default no-op bodies (`set_half_duplex`, `set_inverted`) are correct Rust trait idiom and useful here.

---

## 6. Semaphore Trait

**Rating: RETHINK**

This is the most structurally problematic piece of the HAL.

```rust
pub trait Semaphore {
    fn take_blocking(&self);
    fn take(&self, timeout_us: u32) -> bool;
    fn give(&self);
}
```

**Problem 1: Shared reference for mutable operations.** `take_blocking` and `give` take `&self` (shared reference) yet must mutate internal lock state. This requires `UnsafeCell` or an atomics-backed interior mutability pattern inside every implementation. That is not wrong — `Mutex` in `core::sync` works this way — but the trait's surface does not communicate this requirement. A reviewer seeing `fn take_blocking(&self)` expects a pure query, not a side-effecting lock acquisition. At minimum this needs a doc comment explaining why `&self` is correct.

**Problem 2: The trait is object-safe but returns a trait object from device methods.** `fn get_semaphore(&self) -> &dyn Semaphore` — callers get a `&dyn Semaphore` and must then call `.take_blocking()` on it. This means bus locking goes through two layers of dynamic dispatch: once to get the semaphore from the device, once to call take/give. In hot paths (every SPI transaction) this is unnecessary indirection.

**Problem 3: No Sync bound.** `dyn Semaphore` is not `dyn Semaphore + Sync`. On a multi-threaded RTIC system, the semaphore must be shareable across task contexts. Without `+ Sync`, passing `&dyn Semaphore` between tasks requires unsafe. The trait should be `pub trait Semaphore: Sync`.

**Problem 4: Recursive semaphore semantics not encoded.** The comment says "ArduPilot uses recursive semaphores (408 call sites)" and that `meridian-sync::RecursiveMutex` handles this. But the `Semaphore` trait itself gives no indication that implementations must support re-entrant locking. A user implementing this trait with a standard `spin::Mutex` would deadlock when the same task tries to lock twice. This constraint must be in the trait documentation or enforced structurally.

**What I'd want instead**: For a Rust no_std system, bus arbitration should be done with `critical_section` or RTIC resources, not a runtime semaphore. If a semaphore is truly needed, it should be:

```rust
pub trait Semaphore: Sync {
    /// Acquire the lock. Implementations MUST support re-entrant calls from the same context.
    fn acquire(&self);
    /// Release one level of the recursive lock.
    fn release(&self);
}
```

And the `get_semaphore` method on bus traits should be removed — bus arbitration should be the platform's responsibility, not something the driver reaches for explicitly.

---

## 7. Scheduler Trait

**Rating: PROTOTYPE**

```rust
fn create_thread(&mut self, name: &str, stack_size: u32, priority: u8, proc: fn());
fn reboot(&self, hold_in_bootloader: bool) -> !;
```

`create_thread` is fine for SITL and Linux platforms. On RTIC it is not implementable — RTIC tasks are statically declared at compile time, not dynamically created. The trait as written forces the RTIC platform to either panic on this call or ignore it, which is an unsound abstraction for a safety-critical autopilot.

`fn micros(&self) -> u32` wraps every ~71 minutes. For flight software this is documented, but any code that computes elapsed time by subtraction without handling wrap will exhibit hard-to-reproduce bugs. The `micros64` variant is the right tool; drivers should be steered toward it.

`TimerProc = fn()` and `IoProc = fn()` are bare function pointers. Same capture problem as GPIO interrupt callbacks — no state injection without globals. In an RTOS-less environment this maps to hardware ISRs, but it prevents any driver from having instance-based callback context without a global singleton.

---

## 8. Storage and CAN Traits

**Rating: PROTOTYPE (Storage) / PROTOTYPE (CAN)**

`Storage` is minimal and correct. The 16 KB virtual EEPROM matches ArduPilot. No issues.

`CanFrame` with a 64-byte payload field copied by value is 72+ bytes on the stack per frame. A `receive()` that returns `Option<CanFrame>` copies this on every call. In a DroneCAN system polling at 1 kHz with multiple frame types, this is avoidable stack churn. Consider `fn receive(&mut self, frame: &mut CanFrame) -> bool` (in-place fill) or a `heapless::spsc::Queue` ring for driver-level buffering.

---

## 9. Drivers — Portability Assessment

**Rating: PROTOTYPE**

### ICM-426xx

The ICM-426xx driver is the best piece of code in the review. FIFO mode with proper header validation, AFSR disable with read-modify-write, bank switching with cleanup on failure, correct AAF configuration. The comment "caller must delay 300 us" after `init()` is an honest acknowledgment of a platform dependency that cannot be abstracted without a timer argument. This is correct and transparent.

Using `&mut dyn SpiDevice` rather than `impl SpiDevice` means it is portable but not zero-cost. For SITL: fine. For real metal: consider a generic variant.

The 512-byte stack allocation in `read_fifo` (`let mut fifo_buf = [0u8; MAX_FIFO_PACKETS * FIFO_PACKET_SIZE]` = 512 bytes) is substantial for an ISR or tight main-loop function. On H7 with 128 KB DTCM this is acceptable. On an F4 with 192 KB SRAM shared with DMA it should be a static.

### DPS310

The calibration coefficient parsing and compensation formula match the datasheet faithfully. The `I2cProbe` trait defined inside this module is a local escape hatch around the HAL's address-stateful `I2cDevice` — it is the right impulse (decouple probing) but the trait is module-private and disconnected from the main HAL. This should be resolved at the HAL level.

The `altitude_from_pressure` using `libm::powf` is correct for no_std. The `ground_pressure` auto-calibration on first 10 samples is a good ArduPilot fidelity match.

### IST8310

Clean, compact, well-tested. The mock I2C in the test suite correctly implements the full `I2cDevice` trait, including `get_semaphore` (backed by a static DummySem). This is the right testing pattern for no_std drivers.

The `i2c.set_address(IST8310_ADDR)` at the top of every public method is correct but fragile convention — see Section 3 above.

### GPS / UBX Parser

Byte-at-a-time state machine is the right architecture for serial protocol parsing in no_std. The 256-byte `[u8; MAX_PAYLOAD]` in `UbxParser` lives on the heap in SITL and on the stack or in BSS for embedded. For a static parser instance this is fine.

---

## 10. SITL Platform Implementation

**Rating: PRODUCTION (for what it is — a simulation backend)**

`SitlHal` implements all HAL traits cleanly. The associated types are concrete, the array-indexed bus lookup matches the trait, and the `#[test]` coverage is present and meaningful. The `Hal for SitlHal` implementation is a textbook example of how the Hal supertrait should be used.

One note: `fn spi(&self, bus: u8) -> Option<&Self::Spi>` returns a shared reference. `SpiBus::get_device` returns an owned `Device`. In SITL this is fine because `SitlSpiDevice` is presumably cheaply cloned or created. On real hardware, returning owned devices from a shared bus reference will require interior mutability or unsafe — something the RTIC backend will have to solve explicitly.

---

## 11. Alignment with embedded-hal Ecosystem

The HAL does not implement the `embedded-hal` 1.x traits (`embedded_hal::spi::SpiDevice`, `embedded_hal::i2c::I2c`, etc.). This is a deliberate choice (the codebase comment says "every interface mapped from ArduPilot AP_HAL.h"). The consequence is that no off-the-shelf `embedded-hal`-compatible drivers can be used directly — every sensor driver must be custom. Given the breadth of sensors needed (20+ in this crate alone), this is a significant maintenance commitment. A compatibility shim layer (`impl embedded_hal::spi::SpiDevice for &mut dyn meridian_hal::SpiDevice`) would allow pulling in community drivers for sensors not yet ported.

---

## Summary Ratings

| Component | Rating | Key Issue |
|---|---|---|
| Hal supertrait (overall architecture) | PROTOTYPE | Runtime bus indexing; shared refs for mutable resources |
| SpiDevice trait | PROTOTYPE | `dyn` dispatch at call sites; 128-byte stack in default read_registers |
| I2cDevice trait | PROTOTYPE | Stateful address; misleading default probe |
| GpioPin trait | PROTOTYPE | Port-level not pin-level; platform-specific AF in shared trait |
| UartDriver trait | PROTOTYPE | Buffer size semantics unclear for no_std |
| Semaphore trait | RETHINK | `&self` for mutable ops; missing Sync bound; re-entrant contract unenforced |
| Scheduler trait | PROTOTYPE | `create_thread` not implementable on RTIC; bare fn callbacks |
| Storage trait | PROTOTYPE | Correct but minimal |
| CAN trait | PROTOTYPE | Copy-heavy CanFrame in receive path |
| ICM-426xx driver | PROTOTYPE | Best driver in the crate; dyn dispatch; 512B stack in read_fifo |
| DPS310 driver | PROTOTYPE | Local I2cProbe disconnected from main HAL |
| IST8310 driver | PROTOTYPE | Stateful address by convention only |
| SITL platform | PRODUCTION | Clean, correct, well-tested for its purpose |

---

## Recommended Actions (Priority Order)

1. **Fix Semaphore.** Add `+ Sync` bound. Document re-entrant requirement explicitly. Remove `get_semaphore` from bus traits; manage bus arbitration at platform level using `critical_section` or RTIC resources.

2. **Address I2cDevice address statefulness.** Either (a) bake address into a device handle type at construction time, or (b) enforce `set_address` calls via a wrapper that prevents forgetting. The current convention-based approach is a correctness time bomb.

3. **Replace `dyn SpiDevice`/`dyn I2cDevice` in hot-path driver functions with generics.** `fn probe<S: SpiDevice>(spi: &mut S)` costs nothing at runtime and can still be tested with mock types.

4. **Document or enforce read_registers 63-byte limit.** Either a `const MAX_REGISTERS: usize` associated constant on `SpiDevice`, or a `debug_assert`, or rewrite using a caller-supplied tx buffer.

5. **Add `+ Sync` to callback types or replace bare `fn()` with a `FnMut` wrapper** in `Scheduler::create_thread` and `GpioPin::attach_interrupt`. The no-capture constraint is too restrictive for real driver integration.

6. **Consider an embedded-hal 1.x compatibility shim** to unblock use of community sensor drivers without full rewrite.

7. **Make `CanFrame::receive` non-allocating** by taking `&mut CanFrame` as an out-parameter.

---

*Reviewed 2026-04-02 against meridian-hal at current HEAD. James Munns, Ferrous Systems.*
