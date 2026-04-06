# Panel Review 08 — Jorge Aparicio
**Embedded Rust Architecture, RTIC Mapping, DMA Safety, no_std Patterns**
**Reviewer:** Jorge Aparicio (creator of cortex-m, RTIC)
**Date:** 2026-04-02
**Files reviewed:**
- `crates/meridian-hal/src/` — all trait files
- `crates/meridian-platform-stm32/src/` — all implementation files
- `crates/meridian-sync/src/lib.rs` — RecursiveMutex
- `crates/meridian-platform-stm32/memory.x` — linker script

---

## Executive Summary

The architecture is structurally sound in its intentions. The author clearly understands the STM32H7 memory map, RTIC's task model, and the DMA accessibility problem. The linker script is correct. DMA buffers are placed in the right regions. The RTIC task-to-priority mapping is defensible.

However, there are several concrete safety problems — some UNSOUND — that would cause UB or silent data corruption on real hardware. Four of them are serious enough that they would manifest immediately on first flight.

---

## 1. RTIC Task Priority Mapping

**Rating: SUSPECT**

The mapping in `rtic_app.rs` comments and the skeleton:

| Task | Priority | Trigger |
|---|---|---|
| monitor | 5 | TIM7 @ 10 Hz |
| timer_1khz | 4 | TIM2 @ 1 kHz |
| main_loop | 3 | TIM6_DAC @ 400 Hz |
| rcin | 2 | UART IDLE IRQ |
| slow_loop | 1 | TIM3 @ 50 Hz |

The relative ordering is correct and matches ArduPilot's ChibiOS priority hierarchy. Priority ceiling on shared resources (`scheduler`, `pwm`, `watchdog`, `adc`, `storage`) is implicit in RTIC's lock protocol — the ceiling is the maximum priority of all tasks accessing a resource.

**Issue 1 — monitor at priority 5 accesses `watchdog`, and so does `main_loop` at 3.**
The ceiling on `watchdog` becomes 5. Every time `main_loop` calls `cx.shared.watchdog.lock(...)`, RTIC raises BASEPRI to priority 5, blocking the monitor task. This is correct behavior — but it means the monitor can only detect a stall *after* main_loop releases `watchdog`. If main_loop deadlocks *inside* the watchdog lock, the monitor never runs. This is the same race ArduPilot's ChibiOS port has; it is acknowledged there. Document it explicitly.

**Issue 2 — TIM2 conflict.**
TIM2 is used both as the 1 kHz task trigger (`timer_1khz`) AND in `pwm.rs` it is noted as the Buzzer timer (`TIM2: ch1 (Buzzer/PA15)`). One timer cannot serve both roles simultaneously. This will silently disable either the 1 kHz scheduler or the buzzer, depending on initialization order. This is a real hardware conflict that will show up as an inexplicable missing interrupt. Fix: use TIM5 for buzzer or choose a different timer for the 1 kHz tick. This needs resolution before any hardware bringup.

**Issue 3 — Dispatcher interrupt choice.**
EXTI0, EXTI1, EXTI2 are proposed as software task dispatchers. The comment states these are not connected to any GPIO on the MatekH743. This is correct per the hwdef but must be verified against the actual board schematic. If any of these EXTI lines are ever connected and a software task fires, the handler will be stolen and the GPIO interrupt silently dropped. Low risk if the board config is authoritative, but worth a `compile_error!` assertion.

---

## 2. DMA Buffer Placement

**Rating: SOUND (with one UNSOUND caveat noted below)**

The memory.x linker script correctly defines `.axisram`, `.sram1`, `.sram2`, `.sram4`, `.bkpsram`, and `.param_flash` sections mapped to DMA-accessible addresses. All DMA buffers in `spi.rs`, `uart.rs`, `adc.rs`, `memory.rs`, and `pwm.rs` carry `#[link_section = ".axisram"]` or `#[link_section = ".sram4"]`. The DTCM warning is documented clearly and redundantly.

**Correct placements verified:**
- `DMA_TX_BUF` / `DMA_RX_BUF` → `.axisram` (SPI, AXI SRAM, DMA-safe) ✓
- `RX_BOUNCE` / `TX_BOUNCE` → `.axisram` (UART) ✓
- `ADC_DMA_BUF` → `.axisram` ✓
- `DSHOT_DMA_BUF` → `.axisram` ✓
- `BDSHOT_RX_BUF` → `.sram4` (uncached, required for bdshot) ✓
- `DSHOT_BUF` (in memory.rs) → `.sram4` ✓
- `SPI_DMA_BUF`, `UART_DMA_BUF`, `I2C_DMA_BUF` in memory.rs → `.axisram.dma`

**UNSOUND — linker section name mismatch:**
`memory.rs` uses `#[link_section = ".axisram.dma"]` for `SPI_DMA_BUF`, `UART_DMA_BUF`, and `I2C_DMA_BUF`. The linker script in `memory.x` defines the section as `.axisram` with pattern `*(.axisram .axisram.*)`. The glob `axisram.*` matches `axisram.dma`, so this is currently safe. However, `memory.rs` and `spi.rs` / `uart.rs` / `adc.rs` use *different* section names for the same purpose (`.axisram.dma` vs `.axisram`). This is confusing, non-uniform, and a maintenance trap. More critically, there are now *two separate sets* of DMA buffers for SPI and UART:

- `spi.rs` declares `DMA_TX_BUF` / `DMA_RX_BUF` in `.axisram`
- `memory.rs` declares `SPI_DMA_BUF` in `.axisram.dma`

Both exist at the same time as separate static arrays. The `spi.rs` driver uses its own private buffers directly. The `memory.rs` accessors (`spi_dma_buffer()`, `uart_dma_buffer()`) return pointers to the *other* set of buffers that nothing currently uses. This is dead code at best. If any future code calls `spi_dma_buffer()` and hands the result to the SPI DMA controller while `spi.rs` uses its own `DMA_TX_BUF`, the result is two concurrent DMA masters writing to different buffers that are both named "the SPI DMA buffer." The gap between intent and reality here is UNSOUND in any future use. The duplicate static buffers should be eliminated: either the `spi.rs`/`uart.rs`/`adc.rs` drivers should use the central pool from `memory.rs`, or `memory.rs`'s accessor functions should be removed. Currently no UB, but the structure invites it.

**SUSPECT — SRAM4 address in memory.x:**
`memory.x` maps SRAM4 at `0x38000000, LENGTH = 64K`. The STM32H743 reference manual (RM0433) specifies SRAM4 as 64 KB at 0x38000000. This is correct. However, the `is_dma_safe()` function in `memory.rs` checks `0x3800_0000..0x3801_0000` — which is 65536 bytes (64 KB). The end address is `0x38010000`, which equals `0x38000000 + 0x10000 = 0x38000000 + 65536`. This is correct. No issue.

**UNSOUND — stack in DTCM but `RAM` alias also points to DTCM:**
`memory.x` defines both `DTCM` at `0x20000000` and `RAM` at `0x20000000`. Both are the same region. `_stack_start` is set to `ORIGIN(DTCM) + LENGTH(DTCM)` = `0x20020000`. The cortex-m-rt crate uses `RAM` as the default region for `.data`, `.bss`, and the initial stack. Since `RAM` and `DTCM` are the same region, `.data`/`.bss` land in DTCM. This is intentional and correct for performance. But it means any code that obtains a pointer to a `.data` or `.bss` variable and passes it to a DMA controller will get a DTCM address — and the DMA will silently fail or corrupt adjacent memory. The linker script has no mechanism to prevent this. The `is_dtcm()` and `is_dma_safe()` runtime checks in `memory.rs` are the only guard. These must be used as debug assertions at every DMA setup site. Currently they are not called anywhere in the codebase. Add `debug_assert!(is_dma_safe(buf.as_ptr()), ...)` in `dma_exchange()`, `setup_dma_rx()`, `setup_dma_tx()`, and `AnalogIn::init()` before real DMA register writes land.

---

## 3. RecursiveMutex Soundness

**Rating: SUSPECT — not UNSOUND for its stated use case, but has two concrete failure modes**

The implementation is a well-structured recursive spinlock. The spin-lock / owner / recursion-count protocol is logically correct for the case it claims to support. The `const fn new()` is correct. `unsafe impl Sync` is justified by the protocol. Guard drop ordering is correct (spin-lock released before panic in the depth-exceeded path). The panic-before-spin-hold ordering in the recursion overflow branch is a notable and correct detail.

**UNSOUND — use of `task_id` = RTIC priority is semantically wrong for RTIC.**

This is the central design flaw. The comment says "In RTIC, use the actual priority value instead" for `task_id`. On Cortex-M RTIC, two *different* tasks can have the *same* numeric priority. RTIC allows tasks with the same priority to be dispatched on the same interrupt level. If Task A (priority 3) and Task B (priority 3) both call `SPI_BUS_MUTEX[0].lock(3)`, Task A gets the mutex. Task B spin-waits. But since they are at the *same* RTIC priority, Task B's spin-loop will never yield to Task A — this is a single-core machine and RTIC will not preempt same-priority tasks. The result is a **deadlock that burns 100% CPU and never fires the watchdog**, ultimately causing a hardware reset that appears as a random crash.

The only safe mapping is: one unique `task_id` per *task instance*, not per priority level. The `TASK_ID_SPI_BUS = 1` and `TASK_ID_I2C_BUS = 2` constants in `meridian-sync` are fine for dedicated single-task bus owners. The danger is if two tasks at different priorities ever call into the same bus with the *same* `task_id`. In `spi.rs`, `Stm32SpiSemaphore::take_blocking()` always passes `TASK_ID_SPI_BUS = 1` regardless of which RTIC task is calling. If `timer_1khz` (priority 4) and `main_loop` (priority 3) both use SPI and both pass `task_id = 1`, they will be treated as the same task — the higher-priority task can preempt the lower and re-enter the mutex as "recursive" when it is not. This is **UB**: the lower-priority task still holds the guard, and now both tasks have exclusive access simultaneously. The data is aliased.

Mitigation: ban recursive mutex use across tasks at different priorities. The RTIC lock protocol (BASEPRI elevation) is the correct mechanism for shared resources across priorities. `RecursiveMutex` should only be used when a *single* task needs recursive re-entry within its own call stack, not as a cross-task synchronization primitive.

**SUSPECT — spin-wait on Cortex-M with RTIC.**

The outer spin-loop in `lock()` (the "Mutex held by different task" branch) performs a `core::hint::spin_loop()`. On a real RTIC single-core system, if Task A at priority 3 is spinning waiting for Task B at priority 1 to release a mutex, Task B will *never run* because Task A's priority prevents it from being preempted. This is another deadlock. The comment says "On real RTIC targets the competing task will finish and release soon." — this statement is only true if the competing task has *higher* priority. If it has lower priority, the system locks up permanently.

This cross-priority spinning pattern is fundamentally incompatible with RTIC's non-preemptive same-priority model and its preemptive cross-priority model. On RTIC, shared resources must use the RTIC lock macro, which raises BASEPRI, not a spinlock that can hold a lower-priority task hostage. The RecursiveMutex is only safe when all callers are at the same RTIC priority level and no lower-priority task ever holds the mutex while a higher-priority task tries to acquire it. This is a narrow contract that is not enforced anywhere in the type system.

**SOUND — guard Drop implementation.**

`RecursiveGuard::drop()` calls `self.mutex.unlock(self.task_id)`. This is correct. The `Deref`/`DerefMut` impls are correct. The `unlock()` panic path releases the spin lock before panicking. The `MAX_RECURSION = 8` depth check is defensive and matches the stated ArduPilot nesting depth of 3.

---

## 4. Semaphore Take/Give Leak — UNSOUND

**Rating: UNSOUND**

In `spi.rs`, `Stm32SpiSemaphore::take_blocking()`:

```rust
fn take_blocking(&self) {
    let _guard = SPI_BUS_MUTEX[self.bus_index].lock(TASK_ID_SPI_BUS);
    // Guard is intentionally leaked here
}
```

The `_guard` is dropped at the end of `take_blocking()`, which calls `unlock()`. The mutex is immediately acquired and then immediately released. The semaphore provides zero mutual exclusion. `give()` is a no-op. Any caller that calls `take_blocking()` and then performs a bus transaction has no actual bus protection. Two callers can interleave bus transactions.

The comment acknowledges this: "Guard is intentionally leaked here — the HAL semaphore model is take/give rather than RAII. Real implementation will store the guard." But the guard is not leaked — it is *dropped*. To actually leak it, the code would need `core::mem::forget(_guard)`. As written, the semaphore is a fiction.

The I2C semaphore in `i2c.rs` has the exact same structure:

```rust
fn take_blocking(&self) {
    let _guard = I2C_BUS_MUTEX[self.bus_index].lock(TASK_ID_I2C_BUS);
}
```

Same problem. Both SPI and I2C buses are unprotected. On real hardware, concurrent SPI transactions (e.g., `timer_1khz` and `main_loop` both accessing IMU over shared SPI) will interleave, corrupting both reads. This is **UNSOUND** — it is a race condition on shared mutable hardware state that produces undefined output data and, for an IMU, will corrupt attitude estimates during flight.

Fix: The `Stm32SpiSemaphore` struct needs a field to hold the guard, or the semaphore model needs to be replaced with a closure-based API. Since `&self` is used (not `&mut self`), storing the guard requires `UnsafeCell` or changing the trait. The cleanest fix is to use RTIC's lock protocol for inter-task resource sharing and restrict `RecursiveMutex` to intra-task recursive calls only.

---

## 5. Send/Sync Analysis

**Rating: SUSPECT**

**`DmaBufPool` — SUSPECT:**
`DmaBufPool` is `unsafe impl Sync + Send`. The bump allocator uses an `AtomicUsize` CAS loop for thread-safe allocation. The `alloc()` method returns `&'static mut [u8]` slices from the pool. Each successful CAS gives a unique, non-overlapping region. This is sound for allocation. However, the `base: *mut u8` is a raw pointer. There is no lifetime tying the returned slice to the pool's lifetime. If the pool is constructed from a stack buffer (which `const unsafe fn new` allows), the `'static` lifetime would be a lie. The safety comment says "must point to DMA-safe memory that lives for `'static`" — this is correct but unenforceable at the type level. The `unsafe` tag on `new()` carries this invariant. Sound by convention.

**`EXTI_CALLBACKS` and `EXTI_PINS` — UNSOUND:**
`gpio.rs` uses `static mut EXTI_CALLBACKS: [Option<fn(u16, bool)>; 16]` accessed with raw `unsafe` reads and writes in `attach_interrupt()`, `detach_interrupt()`, and `exti_dispatch()`. These are accessed from both task context (when configuring interrupts) and interrupt context (when `exti_dispatch()` fires). There is no atomic or critical section protecting these accesses. A data race on function pointers is UB. The callback could be half-written when the interrupt fires, resulting in a jump to an arbitrary address.

Fix: use `AtomicPtr` or wrap in a critical section (`cortex_m::interrupt::free`) for all accesses. Alternatively, restrict callback registration to the `init` phase (before interrupts are enabled), which would make the statics effectively read-only after initialization.

**`Stm32Spi::peripheral_started` — SOUND:**
`AtomicBool` with `Relaxed` ordering for a flag that is only read/written by the bus owner after acquiring the semaphore. Given the semaphore is currently broken (issue above), this is incidentally safe only because there's no real concurrent access yet.

**`RecursiveMutex<T>` — conditional:**
`unsafe impl<T: Send> Sync`. The `T: Send` bound is correct. The protocol is sound for its stated constraints. The `Send` bound prevents wrapping non-Send types. The problem is the behavioral unsoundness described in section 3, not the Send/Sync impl itself.

---

## 6. link_section Attributes

**Rating: SOUND (with naming caveat)**

All DMA buffer statics use `#[link_section]` attributes that match defined sections in `memory.x`. Summary:

| Attribute | Section in memory.x | Destination |
|---|---|---|
| `#[link_section = ".axisram"]` | `.axisram` via `*(.axisram .axisram.*)` | AXISRAM @ 0x24000000 |
| `#[link_section = ".axisram.dma"]` | `.axisram` via glob `axisram.*` | AXISRAM @ 0x24000000 |
| `#[link_section = ".sram4"]` | `.sram4` | SRAM4 @ 0x38000000 |
| `#[link_section = ".sram1"]` | `.sram1` | SRAM1 @ 0x30000000 |

The glob match in the linker script `*(.axisram .axisram.*)` correctly captures `.axisram.dma`. This is linker-script-correct.

**SUSPECT — all DMA buffer statics are `static mut`:**
Every DMA buffer is declared `static mut`. The `unsafe` accesses are gated on bus ownership comments. This is the pre-2021-edition pattern. It compiles and works, but there is no enforcement that the bus owner invariant is actually respected. A `static Mutex<[u8; N]>` or `static UnsafeCell<[u8; N]>` with a safety comment would be equally correct and no more verbose. The `static mut` form allows accidental safe-code patterns on older Rust editions. Not a soundness issue but a maintainability one.

**SOUND — SRAM4 for bdshot:**
`BDSHOT_RX_BUF` in `pwm.rs` is correctly placed in `.sram4`. The comment correctly states that MPU must mark SRAM4 as non-cacheable. The `configure_mpu()` function in `memory.rs` is stubbed out with a TODO. Until `configure_mpu()` is implemented and called at boot, SRAM4 may not be uncached, meaning D-cache could serve stale data for bdshot reads. The memory.x placement is correct; the MPU configuration is missing. This will cause bdshot RPM telemetry to read wrong values but will not cause a crash. Mark as SUSPECT until MPU is wired in.

---

## 7. Priority Ceiling Protocol

**Rating: SOUND (for the RTIC skeleton, as documented)**

The RTIC skeleton in `rtic_app.rs` uses RTIC's `shared` resource model correctly. Each shared resource is accessed only via `cx.shared.resource.lock(|r| { ... })`. RTIC automatically computes the priority ceiling for each resource as the maximum priority of all tasks that declare it in their task attributes. This is zero-overhead on single-core Cortex-M: it temporarily raises BASEPRI to the ceiling, blocking all tasks at or below that priority.

The declared shared resources and their ceilings:

| Resource | Used by tasks (priorities) | Ceiling |
|---|---|---|
| `scheduler` | timer_1khz (4), main_loop (3), slow_loop (1) | 4 |
| `watchdog` | main_loop (3), monitor (5) | 5 |
| `pwm` | main_loop (3) | 3 |
| `adc` | timer_1khz (4) | 4 |
| `storage` | slow_loop (1) | 1 |

All accesses are through RTIC locks. No resource is accessed from inside an ISR without a lock. This is correct.

**SUSPECT — `imu_spi` in `#[local]`:**
`imu_spi` is a local resource of `timer_1khz`. Local resources are unshared — only the owning task can access them. This is correct for an IMU SPI bus on DMA_NOSHARE streams. However, if any other task ever needs to communicate over the IMU SPI bus (e.g., for OSD on the same bus), it would need to be promoted to a shared resource. This is an architecture constraint that should be documented as a deliberate design choice.

---

## 8. micros64() Overflow Logic — UNSOUND

**Rating: UNSOUND**

In `rtic_app.rs`, `Stm32Scheduler::micros64()`:

```rust
fn micros64(&self) -> u64 {
    let overflow = self.overflow_count as u64;
    let current = clock::read_cycle_count() as u64;
    let total_cycles = (overflow << 32) | current;
    total_cycles / 400
}
```

The DWT counter is 32 bits. `read_cycle_count()` returns `u32`. Casting to `u64` and OR-ing with `overflow << 32` is the correct pattern for reconstructing a 64-bit counter from a 32-bit hardware counter with a separate overflow count.

**However:** `update_overflow()` is called from `run_timer_procs()` in the 1 kHz task. If the DWT counter wraps in less than 1 ms — which at 400 MHz it cannot (wraps at ~10.7 seconds) — this would miss overflows. At 400 MHz, 1/1000th of a second is 400,000 cycles, far less than 2^32 cycles. The overflow detection is safe at 1 kHz.

**The actual bug:** `update_overflow()` checks `now < self.last_cycle_count` (wrap detection). This is correct for a single-increment check. However, `update_overflow()` is called from `run_timer_procs()`, which calls it *through* the registered `TimerProc` callbacks (`run_timer_procs` calls each `proc()`). But looking at the code, `update_overflow()` is a separate method that must be explicitly called. The RTIC skeleton comment shows:

```rust
cx.shared.scheduler.lock(|sched| {
    sched.update_overflow();
    sched.run_timer_procs();
});
```

This is correct. But `micros64()` takes `&self` while `update_overflow()` takes `&mut self`. If `micros64()` is called from a task that has only `&self` access to the scheduler (via a `lock` that returns `&T`), and `update_overflow` is only called from `timer_1khz`, there is a potential race: `micros64()` could read `overflow_count` between the DWT wrap and the next 1 kHz tick, returning a value that is 2^32 cycles (~10.7 seconds) too low. This is a non-atomic read of `overflow_count` (u32 field, not atomic) paired with a non-atomic read of the DWT counter. On single-core Cortex-M, as long as `micros64()` is never called from an interrupt that can preempt `timer_1khz` (priority 4), the overflow count is stable during the read. If called from `monitor` at priority 5, the race exists. Mark as SUSPECT for use from `monitor`; UNSOUND if exposed to tasks at priority > 4.

---

## 9. D-Cache Coherency — UNSOUND (deferred, but structurally necessary)

**Rating: UNSOUND (when real DMA register access is added)**

Every DMA setup site in `spi.rs`, `uart.rs`, and `adc.rs` has TODOs for D-cache invalidation/clean before and after DMA:

```rust
// Invalidate D-cache for RX buffer region before DMA.
// TODO: actual cache maintenance
// cortex_m::asm::dsb();
// SCB::clean_dcache_by_address(DMA_TX_BUF[bus].as_ptr() as usize, len);
// SCB::invalidate_dcache_by_address(DMA_RX_BUF[bus].as_ptr() as usize, len);
```

The STM32H7 Cortex-M7 has a 16 KB D-cache. AXI SRAM is cacheable. The DMA controller bypasses the cache (it accesses memory directly on the AXI bus). Without cache maintenance:

1. Before DMA TX: CPU writes to `DMA_TX_BUF` but dirty cache lines may not have been flushed to SRAM. DMA reads stale data from SRAM. Sensor commands are corrupted or lost.
2. After DMA RX: DMA writes to `DMA_RX_BUF` in SRAM. CPU reads the cache, which still holds the old pre-DMA values. Sensor readings are stale.

On real hardware this produces IMU reads that are delayed by one DMA transfer cycle, or reads that are completely stale from boot. This is not a theoretical concern — it is the most common bug reported on every H7 STM32 forum thread about DMA. The comments correctly identify it, but the code structure means it will be easy to add the DMA register access while forgetting to uncomment the cache lines.

Architecture recommendation: wrap every DMA transfer in a function that enforces the cache maintenance sequence:

```rust
fn dma_exchange_safe(tx: &[u8], rx: &mut [u8]) -> bool {
    // 1. Copy to DMA buf
    // 2. Clean TX cache lines (SCB::clean_dcache_by_address)
    // 3. Invalidate RX cache lines (SCB::invalidate_dcache_by_address)
    // 4. DSB (data sync barrier)
    // 5. Configure and start DMA
    // 6. Wait for completion
    // 7. DSB
    // 8. Invalidate RX cache lines again (DMA wrote to SRAM after step 3)
    // Copy from DMA buf
}
```

Steps 3 and 8 are both needed: step 3 prevents the CPU from reading pre-DMA values during the transfer; step 8 ensures the CPU sees the DMA-written values after completion. This is currently UNSOUND as a structural gap.

---

## 10. Stm32Scheduler::in_main_thread() — SUSPECT

**Rating: SUSPECT**

```rust
fn in_main_thread(&self) -> bool {
    true
}
```

This always returns `true`. ArduPilot uses `in_main_thread()` to gate operations that must only run from the main thread (parameter writes, MAVLink stream setup). Always returning `true` allows these operations from any task, including high-priority ISR contexts, bypassing the guards. On real hardware this could trigger parameter writes from the 1 kHz timer context. Not immediately fatal but creates a class of bugs that will be very hard to diagnose.

Fix: read the IPSR (Interrupt Program Status Register). If `IPSR == 0`, we are in thread mode (idle or main task). Any nonzero value means we are in an exception/interrupt handler.

```rust
fn in_main_thread(&self) -> bool {
    cortex_m::register::ipsr::read().bits() == 0
}
```

---

## Summary Table

| Area | Finding | Rating |
|---|---|---|
| RTIC task priority mapping (relative order) | Correct ChibiOS mapping | SOUND |
| TIM2 conflict (1 kHz tick vs buzzer) | Same timer used twice | UNSOUND |
| DMA buffer memory regions | Correct `#[link_section]` placement | SOUND |
| Duplicate DMA buffer statics (memory.rs vs spi/uart) | Dead code, invites future race | SUSPECT |
| RecursiveMutex protocol (single-task recursive use) | Logically correct | SOUND |
| RecursiveMutex cross-task use with same task_id | Aliasing UB possible | UNSOUND |
| RecursiveMutex spin-wait blocking lower priority tasks | Deadlock possible | UNSOUND |
| Semaphore take/give (guard immediately dropped) | No mutual exclusion | UNSOUND |
| link_section names and linker script matching | Correct (glob covers .dma suffix) | SOUND |
| SRAM4 MPU non-cacheable config | Stubbed out, bdshot will read stale | SUSPECT |
| Priority ceiling via RTIC lock | Correctly applied | SOUND |
| micros64() overflow race from monitor task | Non-atomic overflow + DWT read | SUSPECT |
| D-cache coherency (all DMA sites) | Cache maintenance TODOs | UNSOUND (deferred) |
| EXTI callback statics (data race) | No atomic or critical section | UNSOUND |
| in_main_thread() always returns true | IPSR not checked | SUSPECT |
| Memory.x linker script (region addresses) | All H743 addresses correct | SOUND |

---

## Critical Fixes Before Hardware Bringup (in priority order)

1. **TIM2 conflict:** Assign the 1 kHz scheduler tick to a timer not used by any PWM channel. TIM14 or TIM16 are good candidates on H743 that are not in ArduPilot's motor output list.

2. **Semaphore guard leak:** Add a `UnsafeCell<Option<RecursiveGuard<'static, ()>>>` to `Stm32SpiSemaphore` and `Stm32I2cSemaphore`. Store the guard in `take_blocking()`; drop it in `give()`. This is the minimum change to make bus locking functional.

3. **D-cache maintenance:** Before enabling any DMA stream in real register access code, add `SCB::clean_dcache_by_address` before DMA TX and `SCB::invalidate_dcache_by_address` after DMA RX. These are not optional on H7.

4. **RecursiveMutex task_id contract:** Enforce that `task_id` is unique per task, not per priority. Either assign each task a unique constant ID or eliminate cross-task spinlock use entirely by routing shared hardware access through RTIC resource locks.

5. **EXTI callbacks:** Gate `EXTI_CALLBACKS` accesses in `cortex_m::interrupt::free` or replace with `AtomicPtr`.

6. **MPU configuration:** Implement `configure_mpu()` to mark SRAM4 non-cacheable before enabling bdshot DMA.

7. **in_main_thread():** Read IPSR.

---

*— Jorge Aparicio*
