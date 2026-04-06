# ArduPilot HAL Audit — Phase 9 Hardware Port Reference
## Meridian Autopilot / Matek H743 Slim (STM32H743)

**Date:** 2026-04-02  
**Sources audited:** 30+ source files across AP_HAL_ChibiOS, AP_HAL_Linux, AP_HAL_SITL  
**Primary target:** STM32H743 on Matek H743 board

---

## 1. Boot Sequence — Power-On to First IMU Sample

### Entry Point (`HAL_ChibiOS_Class.cpp::run()`)

ChibiOS HAL boot is a flat sequence — no separate `main()` and RTOS start; `run()` is called from `main()` and immediately becomes the main thread.

```
1. (optional) stm32_watchdog_init()   ← early watchdog if HAL_EARLY_WATCHDOG_INIT
2. usb_initialise()                   ← USB CDC setup if USE_SERIAL_USB
3. g_callbacks = callbacks            ← store vehicle setup()/loop() pointers
4. main_loop()                        ← takeover, never returns
```

### `main_loop()` sequence

```
1.  daemon_task = chThdGetSelfX()        ← record main thread handle
2.  chThdSetPriority(APM_MAIN_PRIORITY)  ← priority 180 initially
3.  I2CBus::clear_all()                  ← 20 SCL pulses per bus, clears stuck devices
4.  Shared_DMA::init()                   ← initialize DMA channel mutex table
5.  peripheral_power_enable()            ← board-specific peripheral power GPIOs
6.  hal.serial(0)->begin(115200)         ← SERIAL0 (USB or UART) up first
7.  hal.analogin->init()
8.  hal.scheduler->init()               ← spawns 6 RTOS threads (see Scheduler section)
9.  hal_chibios_set_priority(APM_STARTUP_PRIORITY)  ← drop to priority 10 for setup
10. [if watchdog reset] stm32_watchdog_load() ← restore persistent data
11. schedulerInstance.hal_initialized()  ← unblocks timer/rcin/rcout threads
12. g_callbacks->setup()                 ← VEHICLE setup() runs here
13. apply_persistent_params()
14. [optional] stm32_watchdog_init()     ← production watchdog armed after setup
15. hal.scheduler->set_system_initialized()
16. chThdSetPriority(APM_MAIN_PRIORITY)  ← back to priority 180
17. [optional] mem_protect_enable()      ← H7 MPU: first 1k RAM fault-traps nullptr
18. LOOP: g_callbacks->loop()
         + 50us delay if no boost called
         + watchdog_pat()
```

### Clock Configuration (`hwdef/common/stm32h7_mcuconf.h`)

The H743 with 8 MHz crystal achieves **400 MHz system clock** via PLL1:

```
HSE = 8 MHz
PLL1_DIVM = 1   → VCO input = 8 MHz
PLL1_DIVN = 100 → VCO = 800 MHz
PLL1_DIVP = 2   → SYS_CK = 400 MHz
```

For 480 MHz (HAL_CUSTOM_MCU_CLOCKRATE=480000000), DIVN becomes 120.  
SPI clocks derive from PLL2 (200 MHz typically). UART clocks from PLL3.  
The `system.cpp` has compile-time `static_assert` to verify expected clocks match actual.

**H743 memory map for Meridian:**
- ITCM RAM: 64KB at 0x00000000 (first 1KB reserved/guarded as nullptr trap)
- DTCM RAM: 128KB at 0x20000000 (fastest for DMA)
- AXI SRAM: 512KB at 0x24000000 (cache-coherent)
- SRAM4: 64KB at 0x38000000 (marked uncached for bdshot, see NOCACHE_MPU_REGION_1)

---

## 2. Scheduler — ChibiOS Thread Model

**File:** `Scheduler.cpp`, `Scheduler.h`

### Thread Table

| Thread name | Priority | Stack | Rate | Function |
|------------|---------|-------|------|---------|
| main (daemon) | 180 (APM_MAIN_PRIORITY) | large | vehicle loop rate | vehicle loop() |
| timer | 181 (APM_TIMER_PRIORITY) | 1536 B | 1 kHz | registered timer procs, ADC tick |
| rcout | 181 (APM_RCOUT_PRIORITY) | 512 B | event-driven | DShot/PWM output |
| rcin | 177 (APM_RCIN_PRIORITY) | 1024 B | 1 kHz | RCInput._timer_tick() |
| io | 58 (APM_IO_PRIORITY) | 2048 B | 1 kHz | registered IO procs, SD card retry |
| storage | 59 (APM_STORAGE_PRIORITY) | 1024 B | 1 kHz | storage._timer_tick() |
| monitor | 183 (APM_MONITOR_PRIORITY) | 1024 B | 10 Hz | deadlock/watchdog detection |

Additionally per-UART threads:
- `UART0..9`: priority 60 (APM_UART_PRIORITY), stack 320 B each, event-driven (1 ms max wait)
- `UART_RX`: priority 60, stack 768 B, 1 kHz poll for all UART RX

SPI buses run at priority 181 (APM_SPI_PRIORITY), sharing with timer/rcout.  
I2C buses at 176 (APM_I2C_PRIORITY).

### Key Scheduling Mechanics

- **Timer at 1 kHz**: `_timer_thread` calls `_run_timers()` which dispatches up to 8 `MemberProc` callbacks. Registered via `register_timer_process()`. This is where IMU `_accumulate()` runs.
- **IO at 1 kHz**: `_run_io()` dispatches up to 8 IO procs. Registered via `register_io_process()`. Lower priority than timer — used for compass, baro, slower sensors.
- **Watchdog pat**: Done by main thread in `main_loop()` after each `loop()`. Also patted by timer thread during startup (before `_initialized`). Monitor thread saves persistent data every 100 ms.
- **Priority boost**: `delay_microseconds_boost()` raises main thread to 182 for the duration. Checked via `check_called_boost()` — if not called, main loop inserts a 50 µs yield to let lower-priority drivers run.
- **Deadlock recovery**: Monitor thread detects main loop stuck >500 ms, calls `try_force_mutex()` to release the blocking mutex via `chMtxForceReleaseS()`.
- **Stack sizes**: All stack sizes are `#define`-overridable. H7 checks stack usage in io thread at 1 Hz; minimum free stack threshold is 64 bytes.

### Thread Creation API

```cpp
bool Scheduler::thread_create(MemberProc proc, const char *name,
                               uint32_t stack_size, priority_base base, int8_t priority)
```

Priority bases map to fixed values:
- `PRIORITY_BOOST` → 182, `PRIORITY_MAIN` → 180, `PRIORITY_SPI` → 181
- `PRIORITY_TIMER` → 181, `PRIORITY_RCOUT` → 181, `PRIORITY_RCIN` → 177
- `PRIORITY_I2C` → 176, `PRIORITY_CAN` → 178, `PRIORITY_IO` → 58
- `PRIORITY_UART` → 60, `PRIORITY_STORAGE` → 59, `PRIORITY_LED` → 60

---

## 3. UARTDriver — DMA TX/RX Architecture

**Files:** `UARTDriver.cpp`, `UARTDriver.h`

### Serial Port Architecture

Each UART has its own TX thread (`UART0..9`) and shares one global RX thread (`UART_RX`). Serial port definitions come from `HAL_SERIAL_DEVICE_LIST` macro, populated by `hwdef.dat` processing.

### MatekH743 UART Map (from hwdef.dat)
```
SERIAL_ORDER OTG1 UART7 USART1 USART2 USART3 UART8 UART4 USART6 OTG2
  SERIAL0 = OTG1  (USB CDC)
  SERIAL1 = UART7 (Telem1, PE7/PE8, with CTS/RTS PE9/PE10)
  SERIAL2 = USART1 (Telem2, PA9/PA10)
  SERIAL3 = USART2 (GPS1, PD5/PD6)
  SERIAL4 = USART3 (GPS2, PD8/PD9)
  SERIAL5 = UART8  (spare, PE0/PE1)
  SERIAL6 = UART4  (spare, PB8/PB9)
  SERIAL7 = USART6 (RC input / bidirectional RC, PC6/PC7)
  SERIAL8 = OTG2   (second USB CDC)
```

### H743 Baud Rate Special Case

H743 UART clock is 100 MHz. Normal 16x oversampling limits to 6.25 Mbps. For higher rates, the driver switches to 8x oversampling:
```cpp
if (_baudrate > 100000000UL / 16U) {  // > 6.25 Mbps
    sercfg.cr1 |= USART_CR1_OVER8;
}
```
Maximum achievable: 12.5 Mbps (e.g., for high-speed telemetry or logging).

FIFO is enabled on H7 by default (`USART_CR1_FIFOEN`) for better high-baudrate performance. Can be disabled per-port with `OPTION_NOFIFO`.

### DMA RX — Double-Buffer Bounce

```cpp
// Two bounce buffers, alternating
uint8_t *rx_bounce_buf[2];  // RX_BOUNCE_BUFSIZE each (default 128 bytes)

// DMA setup in dma_rx_enable():
dmamode |= DMA_SxCR_TRBUFF;   // H743 errata 2.3.1 requirement
rx_bounce_idx ^= 1;            // flip to other buffer
stm32_cacheBufferInvalidate(rx_bounce_buf[rx_bounce_idx], RX_BOUNCE_BUFSIZE);
dmaStreamSetMemory0(rxdma, rx_bounce_buf[rx_bounce_idx]);
dmaStreamSetTransactionSize(rxdma, RX_BOUNCE_BUFSIZE);
dmaStreamSetMode(rxdma, ... | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
dmaStreamEnable(rxdma);
```

UART IDLE interrupt (`USART_CR1_IDLEIE`) triggers early DMA flush when the bus goes idle — critical for variable-length protocols like SBUS, CRSF, GPS NMEA.

On IDLE interrupt, `rx_irq_cb()` disables DMA to force a transfer-complete interrupt (the mechanism differs between F7/H7 vs L4/G4):
```cpp
#if defined(STM32F7) || defined(STM32H7)
    uart_drv->rxdma->stream->CR &= ~STM32_DMA_CR_EN;  // triggers TC interrupt
```

RX DMA is NOT shared (always owned). TX DMA IS shared via `Shared_DMA` to allow sharing with SPI or I2C on the same DMA stream.

### DMA TX

TX uses a single `tx_bounce_buf` (TX_BOUNCE_BUFSIZE, default 64 bytes). TX DMA auto-disabled for:
- Half-duplex mode (hardware UART turnaround can't handle DMA latency)
- Low baud rates on shared DMA streams (≤115200 on shared streams)
- After 1000 contention events at ≤460800 baud (FIFO-capable MCUs) or ≤115200 (others)

### UART Events
- `EVT_DATA` (mask 10): wake TX thread for data ready
- `EVT_TRANSMIT_END` (mask 12): half-duplex TX complete
- `EVT_TRANSMIT_DMA_COMPLETE` (mask 1): DMA done
- `EVT_TRANSMIT_UNBUFFERED` (mask 3): change thread priority for unbuffered write

---

## 4. SPIDevice — Bus Management and H7 Differences

**Files:** `SPIDevice.cpp`, `SPIDevice.h`

### H7 vs F4/F7 SPI Register Differences

H7 uses a completely different SPI peripheral (SPI v2):
```cpp
#if defined(STM32H7)
    // Mode bits in CFG2, not CR1
    #define SPIDEV_MODE0    0
    #define SPIDEV_MODE3    SPI_CFG2_CPOL | SPI_CFG2_CPHA
    // Clock divider in CFG1 (MBR field), not CR1 BR field
    return (i * SPI_CFG1_MBR_0) | SPI_CFG1_DSIZE_VALUE(7);  // 8-bit transfers
    // Dummy TX/RX buffers must be DMA-safe and 4-byte aligned
    bus.spicfg.cfg1 = freq_flag;
    bus.spicfg.cfg2 = device_desc.mode;
    bus.spicfg.dummytx = (uint32_t *)malloc_dma(4);  // H7-specific
    bus.spicfg.dummyrx = (uint32_t *)malloc_dma(4);
#else
    bus.spicfg.cr1 = (uint16_t)(freq_flag | device_desc.mode);
```

### MatekH743 SPI Buses
```
SPI1: PA5(SCK), PA6(MISO), PD7(MOSI)
  - IMU1: ICM42688 on PC15 CS, MODE3, 2–16 MHz
  - (fallback) MPU6000, MODE3, 1–4 MHz

SPI2: PB13(SCK), PB14(MISO), PB15(MOSI)
  - OSD: MAX7456, MODE0, 10 MHz

SPI3: PB3(SCK), PB4(MISO), PB5(MOSI)
  - External (EXT_CS1 PD4, EXT_CS2 PE2)
  - PixArt flow sensor on EXT_CS1, 2 MHz

SPI4: PE12(SCK), PE13(MISO), PE14(MOSI)
  - IMU2: ICM20602 on PE11 CS, MODE3, 1–4 MHz
  - IMU3: ICM42605 on PC13 CS, MODE3, 2–16 MHz (backup)
```

`DMA_NOSHARE SPI1* SPI4*` — IMU SPI buses have dedicated DMA (no sharing needed).

### Frequency Derivation

The H743 SPI clock tree: SPI1/SPI4 from PLL2 (typically 200 MHz), SPI2/SPI3 from PLL1.  
Divisor is a power-of-2 binary search to find the highest divisor that keeps frequency ≤ requested:
```cpp
while (spi_clock_freq > _frequency && i < 7) {
    spi_clock_freq >>= 1;
    i++;
}
// i=0 → /1, i=1 → /2, ..., i=7 → /256
```
For ICM42688 at 16 MHz with 200 MHz SPI clock: 200/16 = 12.5, so divisor 16 (i=4) → actual 12.5 MHz.

### DMA Sharing Mechanism

Each `SPIBus` has a `Shared_DMA` with `dma_deallocate` callback. When another peripheral needs the same DMA stream, `dma_deallocate` calls `stop_peripheral()` which calls `spiStop()`. On re-acquire, `start_peripheral()` calls `spiStart()` to restart with current config. The SCK line is forced to idle state during peripheral stop to avoid floating clock lines.

### Transfer Flow

```
acquire_bus(true)
  → dma_handle->lock()         [blocks if DMA contested]
  → spiAcquireBus()
  → stop_peripheral()          [reconfigure spicfg for this device]
  → start_peripheral()         [spiStart()]
  → spiSelectI()               [assert CS via pal_line]
  → spiStartExchangeI()        [DMA exchange, IRQ priority 12]
  → osalThreadSuspendTimeoutS  [sleep until DMA complete, timeout 20ms+32us/byte]
acquire_bus(false)
  → spiUnselectI()             [deassert CS]
  → spiReleaseBus()
  → dma_handle->unlock()
```

Polled fallback (`HAL_SPI_USE_POLLED`) uses `spiPolledExchange()` byte-by-byte.

---

## 5. I2CDevice — Clock Configuration and Error Recovery

**Files:** `I2CDevice.cpp`, `I2CDevice.h`

### H743 I2C TIMINGR Values

H7 uses a completely different timing register (TIMINGR) vs F4 (clock_speed field):
```cpp
#ifdef STM32H7
    HAL_I2C_H7_100_TIMINGR = 0x00707CBB   // 100 kHz on H7
    HAL_I2C_H7_400_TIMINGR = 0x00300F38   // 400 kHz on H7
```
These were computed with STM32CubeMX for the H7 peripheral clock (typically 100 MHz from PLL3).

Default maximum clock is 100 kHz (`HAL_I2C_MAX_CLOCK`). Can be raised to 400 kHz in hwdef.dat. If a device requests a lower clock than the bus default, the bus clock is dropped permanently for that bus.

### MatekH743 I2C Buses
```
I2C_ORDER I2C2 I2C1
  I2C1: PB6(SCL), PB7(SDA)   → external compass, baro, etc.
  I2C2: PB10(SCL), PB11(SDA) → external compass, baro, etc.
HAL_I2C_INTERNAL_MASK = 0    → no internal I2C devices
```
External compass auto-probed on both buses.

### Bus Clear (Stuck SDA Recovery)

Called at boot from `main_loop()` via `I2CBus::clear_all()`:
```cpp
// Toggle SCL 20 times to clock out stuck transaction
palSetLineMode(scl_line, PAL_MODE_OUTPUT_PUSHPULL);
for (uint8_t j = 0; j < 20; j++) {
    palToggleLine(scl_line);
    hal.scheduler->delay_microseconds(10);
}
palSetLineMode(scl_line, mode_saved);
```

### Per-Transfer Protocol

```
i2cAcquireBus()                     ← ChibiOS OS-level bus lock
bouncebuffer_setup()                ← DMA-safe buffer if needed
bus.dma_handle->lock()              ← acquire shared DMA stream
i2cStart()                         ← configure peripheral for this transfer
  [SMBUS mode sets I2C_CR1_SMBHEN]
i2cMasterTransmitTimeout() or
i2cMasterReceiveTimeout()
  timeout = max(4ms, 2 * (8000000/clock * bytes / 1000) ms)
i2cSoftStop()                       ← stop without full I2C STOP condition (avoids LidarLite issue)
bus.dma_handle->unlock()
[on timeout] I2CBus::clear_bus()    ← if SDA stuck low after timeout
i2cReleaseBus()
```

Retry loop: up to `_retries` (default 2) retries per transfer, re-acquiring DMA each time.

### Split Transfers

Some devices (LidarLite blue label) cannot handle a repeated START. The `_split_transfers` flag causes write and read to be issued as separate complete transfers (each with STOP).

---

## 6. RCOutput — PWM / DShot Architecture

**Files:** `RCOutput.cpp`, `RCOutput_bdshot.cpp`, `RCOutput_serial.cpp`

### PWM Group Model

PWM channels are organized into groups of up to 4 channels sharing one STM32 timer. The group table `pwm_group_list[]` is populated from `HAL_PWM_GROUPS` macro (hwdef.dat):

```
MatekH743 PWM groups:
  TIM8:  ch2N (M1/PB0), ch3N (M2/PB1)          — complementary outputs
  TIM5:  ch1 (M3/PA0), ch2 (M4/PA1), ch3 (M5/PA2), ch4 (M6/PA3)
  TIM4:  ch1 (M7/PD12), ch2 (M8/PD13), ch3 (M9/PD14), ch4 (M10/PD15)
  TIM15: ch1 (M11/PE5), ch2 (M12/PE6)
  TIM1:  ch1 (PA8) — WS2812 LED
  TIM2:  ch1 (PA15) — Buzzer/Alarm
```

### Output Modes

All modes share the same `pwm_group` struct, switching via `set_group_mode()`:

| Mode | Clock | Protocol | Notes |
|------|-------|----------|-------|
| `MODE_PWM_NORMAL` | 1 MHz (50–400 Hz) or 8 MHz (>400 Hz) | Standard servo PWM | `period = freq/rate` |
| `MODE_PWM_ONESHOT` | 8 MHz | Single pulse per loop | `period = 0` — no periodic output |
| `MODE_PWM_ONESHOT125` | 8 MHz | 125–250 µs pulses | Same as oneshot, narrower range |
| `MODE_PWM_BRUSHED` | varies | Direct speed control | |
| `MODE_DSHOT150/300/600/1200` | DMA-driven | DShot digital | Requires DMA |
| `MODE_PWM_NONE` | — | disabled | Alarm mode takes whole timer |

### DShot via DMA

DShot is implemented as timer DMA burst (DMAR): the timer CCR registers are written via DMA for each bit of the packet. This allows sending full 16-bit DShot frames to all 4 channels simultaneously on one DMA request per timer ARR overflow.

Key structure per group:
```cpp
uint32_t *dma_buffer;           // DMA buffer, DSHOT_BUFFER_LENGTH words
Shared_DMA *dma_handle;         // TX DMA channel
uint8_t dma_up_stream_id;       // DMA stream for DMAR
```

DShot timing (DSHOT600 example):
- Bit period: 1.67 µs (600 kbps)
- Bit '1': 75% high
- Bit '0': 37% high
- S (BLHeli_S) variant uses different tick ratios

### DShot Rate Control

Driven by a `virtual_timer_t _dshot_rate_timer`:
- `_dshot_rate = 0`: run at 1 kHz from timer thread
- `_dshot_rate > 0`: run at `_dshot_rate * loop_rate_hz`, e.g., rate=2, loop=400 Hz → 800 Hz DShot
- Minimum 800 Hz enforced (BLHeli32 16-bit counter wraps at ~732 Hz at 48 MHz)

### Bi-Directional DShot (bdshot)

After TX burst, channels switch to input capture mode to receive RPM telemetry. Each channel can have its own IC DMA handle, or share adjacent channel DMA if needed.

Pin configuration for bdshot:
```cpp
palSetLineMode(group.pal_lines[i],
    PAL_MODE_ALTERNATE(group.alt_functions[i])
    | PAL_STM32_OTYPE_PUSHPULL
    | PAL_STM32_PUPDR_PULLUP
    | PAL_STM32_OSPEED_MID1);   // medium speed to reduce noise on switch
```

SRAM4 (0x38000000) is mapped uncacheable for bdshot DMA buffers (see MCU config `NOCACHE_MPU_REGION_1`).

### Safety State

`safety_state` starts `SAFETY_DISARMED`. Updated by `safety_update()` (timer proc at 1 kHz). In disarmed state, all FMU outputs are forced to safe values. IOMCU (if present) also receives safety commands.

---

## 7. RCInput — Input Capture and UART RC Protocols

**File:** `RCInput.cpp`

### Two Input Paths

#### 1. Timer Input Capture (ICU)
```cpp
sig_reader.attach_capture_timer(&RCIN_ICU_TIMER, RCIN_ICU_CHANNEL,
                                 STM32_RCIN_DMA_STREAM, STM32_RCIN_DMA_CHANNEL);
pulse_input_enabled = true;
```
Used for: PPM (single-wire multi-channel), analog PWM single-channel.  
DMA captures timer edge timestamps into `sig_reader.sigbuf`.  
`_timer_tick()` reads pulse list and passes to `AP_RCProtocol::process_pulse_list()`.

MatekH743: `PC7 TIM3_CH2 TIM3 RCININT PULLDOWN LOW` — timer 3 channel 2 for RC input.

#### 2. UART RC (SBUS, CRSF, ELRS, FPort, etc.)
RC is received via USART6 (PC7/PC6 on MatekH743).  
When UART RC is active, `pulse_input_enable(false)` disables the ICU path.  
`AP_RCProtocol` handles the UART stream directly.

Alternative config: `PC7 USART6_RX USART6 NODMA ALT(1)` — the same pin switches from ICU to UART RX for bidirectional protocols (FPort, CRSF with telemetry) without extra hardware.

### _timer_tick() (1 kHz, rcin thread)

```cpp
while ((p = sig_reader.sigbuf.readptr(n)) != nullptr) {
    rcprot.process_pulse_list(p, n*2, sig_reader.need_swap);
    sig_reader.sigbuf.advance(n);
}
if (rcprot.new_input()) {
    WITH_SEMAPHORE(rcin_mutex);
    _rcin_timestamp_last_signal = micros();
    _num_channels = rcprot.num_channels();
    rcprot.read(_rc_values, _num_channels);
    _rssi = rcprot.get_RSSI();
}
```

Failsafe is handled by the vehicle code checking `new_input()` and comparing timestamp age — the HAL does not implement a failsafe timer itself.

---

## 8. Storage — Parameter Flash on STM32H7

**File:** `Storage.cpp`, `hwdef/common/flash.c`

### Storage Backends (Priority Order)

1. **FRAM** (SPI RAMTRON) — fastest, used on Cube/Pixhawk boards
2. **Flash** (internal STM32) — used on MatekH743
3. **SDCard** (FAT32) — fallback if no FRAM and no flash page defined

MatekH743 uses flash: `STORAGE_FLASH_PAGE 14` and `HAL_STORAGE_SIZE 32768` (32KB).

### H743 Flash Layout

H743 has 2 MB flash in 16 pages of 128 KB each:
```
STM32_FLASH_NPAGES = 2048 / 128 = 16
STM32_FLASH_FIXED_PAGE_SIZE = 128 (KB)
STM32_FLASH_NBANKS = 2048 / 1024 = 2

MatekH743 STORAGE_FLASH_PAGE = 14
  → Uses pages 14 and 15 (last 256 KB of bank 2)
  → Flash address: 0x08000000 + 14 * 128K = 0x081C0000
```

### Flash AP_FlashStorage Wear-Leveling

`Storage` uses the `AP_FlashStorage` class which implements a simple log-structured storage:
- Two pages are used as a pair (current and backup)
- Each 32-byte "line" can be written independently
- When a page is full, data is compacted and the other page is erased
- `AP_FLASH_STORAGE_DOUBLE_PAGE` flag uses 4 pages for double redundancy

### Write Path

`_timer_tick()` in the storage thread (1 kHz) writes at most one dirty line per call to minimize latency:
```cpp
// find first dirty line
for (i=0; i<CH_STORAGE_NUM_LINES; i++) {
    if (_dirty_mask.get(i)) break;
}
// copy line under semaphore
memcpy(tmpline, &_buffer[CH_STORAGE_LINE_SIZE*i], CH_STORAGE_LINE_SIZE);
// write to flash (retries 5 times)
_flash_write(i);
// only clear dirty if buffer hasn't changed during write
if (memcmp(tmpline, &_buffer[CH_STORAGE_LINE_SIZE*i], ...) == 0) {
    _dirty_mask.clear(i);
}
```

### H743 Flash Unlock/Lock

H743 has dual-bank flash with separate CR1/CR2 registers:
```c
FLASH->KEYR1 = FLASH_KEY1; FLASH->KEYR1 = FLASH_KEY2;  // bank 1
FLASH->KEYR2 = FLASH_KEY1; FLASH->KEYR2 = FLASH_KEY2;  // bank 2
// Wait: SR1/SR2 BSY|QW|WBNE bits
```
Flash cache is disabled before erase/write and re-enabled after (H7 errata).

Backup copies saved to `/APM/STRG_BAK/` on SD card at boot (up to 100 rolling backups).

---

## 9. GPIO — Pin Configuration and hwdef.dat Mapping

**File:** `GPIO.cpp`

### hwdef.dat to Hardware Mapping

`hwdef.dat` processing (Python script `chibios_pins.py`) generates:
- `HAL_GPIO_PINS` — array of `gpio_entry` structs with `{pin_num, enabled, pwm_num, pal_line}`
- `HAL_SPI_BUS_LIST`, `HAL_SPI_DEVICE_LIST` — SPI bus and device tables
- `HAL_I2C_DEVICE_LIST` — I2C bus table
- `HAL_PWM_GROUPS` — timer group table
- `HAL_SERIAL_DEVICE_LIST` — UART driver table

`pal_line` is a ChibiOS macro `PAL_LINE(GPIOX, n)` encoding port and pin.  
GPIO alternate function is encoded as `PAL_MODE_ALTERNATE(n)` where n is the STM32 AF number.

### Alternative Config System

`BRD_ALT_CONFIG` parameter selects alternate pin mappings at runtime. The `HAL_PIN_ALT_CONFIG` table maps `{alternate_num, mode, pal_line, periph_type, periph_instance}`.

Example (MatekH743): `ALT(1)` enables USART6_RX on PC7 instead of the default TIM3_CH2 RCInput. The driver calls `GPIO::resolve_alt_config()` to get the correct pal_line for its peripheral.

### GPIO ISR / Interrupt Quotas

GPIO interrupt handlers are rate-limited via `isr_quota`. The monitor thread calls `gpio->timer_tick()` every 100 ms which resets quotas. If too many ISRs fire, the interrupt is temporarily disabled (`isr_disabled_ticks`). This prevents runaway GPIO noise from locking up the system.

### GPIO Enable/Disable vs PWM

PWM pins are auto-disabled as GPIO when `SRV_Channels::is_GPIO()` returns true. This allows dynamically repurposing motor output pins as general GPIO for non-standard use cases.

---

## 10. Shared DMA System

**File:** `shared_dma.cpp`

### Problem Solved

The STM32H7 has limited DMA streams. Some peripherals must share DMA channels (e.g., UART TX and SPI RX may use the same BDMA channel). The `Shared_DMA` class manages cooperative sharing.

### Locking Protocol

```cpp
Shared_DMA(stream_id1, stream_id2, allocate_fn, deallocate_fn)

lock()   → acquires mutex(stream_id1), mutex(stream_id2)
         → if another peripheral owns either stream:
             calls their deallocate_fn()   [e.g., spiStop()]
         → calls own allocate_fn()         [e.g., configure DMA for SPI]
unlock() → releases mutexes

lock_nonblock() → tries mutex, returns false immediately if contested
```

`SHARED_DMA_MASK` bitmask defines which stream IDs are actually shared. Non-shared streams skip locking entirely (`is_shared()` returns false).

### Contention Tracking

`_contention_stats` array (one entry per stream) tracks `contended_locks`, `uncontended_locks`, `transactions`. Can be dumped to ground station for diagnostics.

---

## 11. Linux HAL — Threading Model

**Files:** `HAL_Linux_Class.cpp`, `Scheduler.cpp` (Linux)

### Key Differences from ChibiOS

| Feature | ChibiOS | Linux |
|---------|---------|-------|
| Thread model | ChibiOS RTOS threads | pthreads with SCHED_FIFO |
| Thread creation | `chThdCreateStatic()` | `pthread_create()` with pre-allocated 1MB stacks |
| Timing | Hardware timer via TIM12, 1 MHz SysTick | `clock_gettime(CLOCK_MONOTONIC)` |
| DMA | Hardware DMA streams | Not used — kernel drivers |
| GPIO | Direct register writes via ChibiOS PAL | sysfs, mmap, or BBB/RPi-specific drivers |
| SPI | ChibiOS SPI driver with DMA | Linux spidev kernel driver |
| I2C | ChibiOS I2C driver with DMA | Linux i2c-dev kernel driver |
| Priorities | ChibiOS priority levels (0-255) | Linux SCHED_FIFO priorities (1-20) |

### Linux Thread Priorities
```
APM_LINUX_MAX_PRIORITY   20
APM_LINUX_TIMER_PRIORITY 15  → 1000 Hz
APM_LINUX_UART_PRIORITY  14  → 100 Hz
APM_LINUX_RCIN_PRIORITY  13  → 100 Hz
APM_LINUX_MAIN_PRIORITY  12
APM_LINUX_IO_PRIORITY    10  → 50 Hz
APM_LINUX_SCRIPTING_PRIORITY 1
```

Linux uses `mlockall(MCL_CURRENT|MCL_FUTURE)` for real-time memory locking.  
CPU affinity can be set via `_cpu_affinity` mask.

### Barrier Synchronization

Linux scheduler uses a `pthread_barrier_t` initialized to N+1 (worker threads + main). All threads call `pthread_barrier_wait()` at startup to ensure all are running before the vehicle's `setup()` proceeds.

---

## 12. SITL HAL — Simulation Architecture

**Files:** `HAL_SITL_Class.cpp`, `Scheduler.cpp` (SITL), `SITL_State.cpp`

### Key Design Principle: Single-Threaded Simulation

Unlike ChibiOS (many RTOS threads) and Linux (many pthreads), SITL runs almost everything in a single main thread. "Thread" callbacks are dispatched cooperatively:

- Timer procs run in the main thread context (not a separate thread)
- IO procs run in the main thread context
- "Simulated" threads are pthreads but most vehicle logic is cooperative

### Time Control

SITL has controllable simulated time. `stop_clock(uint64_t usec)` advances the clock to `usec` even while blocking. This allows SITL to run faster-than-realtime or in lockstep with a physics sim.

`delay_microseconds()` calls `_sitlState->wait_clock(start + usec)` rather than sleeping — the clock can be advanced by the FDM input.

### SITL_State

`SITL_State` manages:
- FDM (Flight Dynamics Model) connection via UDP to JSBSim, ArduCopter SIM, etc.
- Serial port simulation (each SERIAL driver is a pair of UDP sockets)
- Simulated peripherals: IMU, compass, baro, GPS, rangefinder, cameras
- RC input simulation via UDP

`_fdm_input_step()` receives physics state from the FDM and injects it into the simulated sensors. The FDM runs at a configurable rate (default 1 kHz).

### SITL Storage

`Storage.cpp` (SITL) uses a flat file `eeprom.bin` in the working directory. No wear leveling needed.

---

## 13. Critical Port Notes for Meridian / STM32H743

### Must-Implement Items

1. **DMA_TRBUFF errata (H743 Rev A-Y)**  
   Any DMA stream connected to a UART must set `DMA_SxCR_TRBUFF`. This is an H743 errata fix. Failure causes sporadic DMA transfer errors.

2. **H7 SPI v2 register layout**  
   `SPI_CFG1` (divider, data size) and `SPI_CFG2` (mode, CPOL/CPHA) are completely different from F4/F7. Dummy TX/RX buffers for single-direction DMA transfers must be 4-byte aligned and DMA-safe (in non-cached RAM or with cache maintenance).

3. **I2C TIMINGR**  
   The H7 I2C peripheral requires a hardware timing register rather than a simple clock frequency. Values are PCLK-dependent. Use: `0x00707CBB` for 100 kHz, `0x00300F38` for 400 kHz at 100 MHz I2C clock.

4. **Flash dual-bank unlock**  
   H743 2MB flash has two independent banks with separate `KEYR1`/`KEYR2`. Both must be unlocked before writing. Wait on `SR1` and `SR2` separately.

5. **SRAM4 uncached for bdshot**  
   MPU region 5 marks 0x38000000–0x3800FFFF as Device memory (no cache) for bidirectional DShot DMA buffers. Without this, cache coherency causes sporadic corrupted RPM readings.

6. **UART IDLE interrupt + DMA double-buffer**  
   For SBUS/CRSF/GPS: enable `USART_CR1_IDLEIE` and disable `USART_CR1_RXNEIE`. The IDLE ISR must disable DMA to force a TC interrupt for the partial buffer. Use two alternating bounce buffers with `stm32_cacheBufferInvalidate()` before each.

7. **ChibiOS priority levels vs RTOS reality**  
   Priorities 180–183 are the hot zone. Main loop at 180, timer/SPI/rcout at 181, monitor at 183. I2C at 176. IO at 58. The IO priority gap (58 vs 176+) is intentional — IO tasks must never preempt sensor sampling.

8. **50 µs yield in main loop**  
   If `delay_microseconds_boost()` is not called during the INS loop, the main loop inserts a `delay_microseconds(50)` at the end of each `loop()`. This ensures the IO thread (priority 58) actually gets to run. Meridian must replicate this yield pattern.

9. **Watchdog pat every loop**  
   The IWDG is patted inside `watchdog_pat()` called from `main_loop()` at the end of every `loop()`. Independent window watchdog timeout is ~2 seconds. The monitor thread triggers a crashdump at 1.8 seconds before the watchdog fires (when AP_CRASHDUMP_ENABLED).

10. **Persistent data across resets**  
    `PersistentData` struct (contains fault info, scheduler task, last MAVLink msg, SPI/I2C counts) is saved to the first 32 bytes of the backup SRAM/RTC domain registers. On watchdog reset, this data is loaded to identify which task was running.

### Meridian Architecture Recommendation

For Meridian's Rust HAL targeting H743:

- **RTOS**: Use RTIC (Real-Time Interrupt-driven Concurrency) or Embassy async executor. Equivalent to ChibiOS threads at priorities 58–183.
- **DMA**: Use `stm32h7xx-hal` DMA abstractions but be aware of the TRBUFF errata and cache invalidation requirements.
- **SPI**: ICM42688 needs SPI v2 CFG1/CFG2 register layout. Use CPOL=1, CPHA=1 (MODE3), 2 MHz init / 16 MHz fast.
- **UART**: Implement double-buffer DMA RX with IDLE detection. Critical for GPS (NMEA), SBUS, CRSF.
- **I2C**: Use hardware TIMINGR values, implement 20-pulse SCL clear at boot.
- **Storage**: Two last 128KB pages of bank 2 (pages 14+15, addresses 0x081C0000–0x081FFFFF). Implement minimal log-structured wear-leveling or use external FRAM.
- **Thread priorities** (RTIC task priorities): Map APM priorities to RTIC software task priorities. Hardware interrupt priorities should be separate (12 for DMA, 13+ for UART/SPI).

---

## Key Source Files Audited

| File | Purpose |
|------|---------|
| `AP_HAL_ChibiOS/HAL_ChibiOS_Class.cpp` | Boot sequence, HAL construction, main_loop |
| `AP_HAL_ChibiOS/system.cpp` | Fault handlers, time functions, clock assertions |
| `AP_HAL_ChibiOS/Scheduler.cpp` | Thread spawning, timer/IO dispatch, watchdog, deadlock recovery |
| `AP_HAL_ChibiOS/Scheduler.h` | Priority constants, stack sizes, thread handles |
| `AP_HAL_ChibiOS/UARTDriver.cpp` | UART init, DMA TX/RX, bounce buffers, H7 FIFO/oversampling |
| `AP_HAL_ChibiOS/SPIDevice.cpp` | H7 SPI v2 config, DMA sharing, CS management, frequency derivation |
| `AP_HAL_ChibiOS/I2CDevice.cpp` | H7 TIMINGR values, bus clear, retry logic, DMA sharing |
| `AP_HAL_ChibiOS/RCOutput.cpp` | PWM group init, DShot rate control, timer configuration |
| `AP_HAL_ChibiOS/RCOutput_bdshot.cpp` | Bidirectional DShot, IC DMA setup, pin mode switching |
| `AP_HAL_ChibiOS/RCOutput_serial.cpp` | DShot command packets, DMA buffer fill |
| `AP_HAL_ChibiOS/RCInput.cpp` | ICU pulse capture, UART RC, _timer_tick dispatch |
| `AP_HAL_ChibiOS/Storage.cpp` | Flash/FRAM/SDCard backend, dirty-line write, backup |
| `AP_HAL_ChibiOS/GPIO.cpp` | gpio_entry table, alt config, ISR quota |
| `AP_HAL_ChibiOS/shared_dma.cpp` | DMA stream sharing, lock/unlock, contention stats |
| `AP_HAL_ChibiOS/hwdef/MatekH743/hwdef.dat` | Complete pin mapping for target board |
| `AP_HAL_ChibiOS/hwdef/common/flash.c` | H743 flash unlock, erase, write, bank layout |
| `AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h` | PLL config, VOS, NOCACHE regions, I2C/SPI clocks |
| `AP_HAL_Linux/HAL_Linux_Class.cpp` | Linux HAL construction, board subtype dispatch |
| `AP_HAL_Linux/Scheduler.cpp` | pthreads, SCHED_FIFO, CPU affinity, barrier sync |
| `AP_HAL_SITL/Scheduler.cpp` | Single-thread sim, semaphore hack, time control |
| `AP_HAL_SITL/SITL_State.cpp` | FDM connection, simulated peripheral injection |
| `AP_HAL_SITL/SITL_State_common.cpp` | Serial device simulation factory |

---

## PART II — Abstract HAL Interface Layer Deep Audit
### Sources: `libraries/AP_HAL/`, `libraries/AP_BoardConfig/`, `libraries/AP_IOMCU/`
### Date: 2026-04-02

---

## II.1 HAL Structure Overview

The HAL is a C++ abstract interface layer. All platform-specific code implements pure virtual methods. The top-level object is `AP_HAL::HAL`, which is a singleton holding pointers to every subsystem driver.

### II.1.1 Top-Level HAL Object (`AP_HAL/HAL.h`)

```cpp
class AP_HAL::HAL {
    // Constructor takes pointers to every subsystem
    HAL(UARTDriver* serial0..serial9,
        I2CDeviceManager* i2c_mgr,
        SPIDeviceManager* spi,
        WSPIDeviceManager* wspi,
        AnalogIn* analogin,
        Storage* storage,
        UARTDriver* console,
        GPIO* gpio,
        RCInput* rcin,
        RCOutput* rcout,
        Scheduler* scheduler,
        Util* util,
        OpticalFlow* opticalflow,
        Flash* flash,
        DSP* dsp,
        CANIface* can[])

    virtual void run(int argc, char* const argv[], Callbacks* callbacks) const = 0;
};
```

Serial port assignments: serial(0)=console, serial(1)=telem1, serial(2)=telem2, serial(3)=GPS1, serial(4)=GPS2, serial(5-9)=extra.

**Meridian**: A `Hal` struct holding references to each subsystem. The `run()` method maps to `#[entry]` launching the RTOS.

---

## II.2 Time Functions (`AP_HAL/system.h`)

Free functions in the `AP_HAL` namespace:

```cpp
uint16_t micros16();    // 16-bit us (wraps at 65535)
uint32_t micros();      // 32-bit us since boot
uint32_t millis();      // 32-bit ms since boot
uint16_t millis16();    // 16-bit ms
uint64_t micros64();    // 64-bit us since boot
uint64_t millis64();    // 64-bit ms since boot
void panic(const char* errormsg, ...) NORETURN;
```

Timeout helpers (handle wraparound correctly):
```cpp
template<typename T>
bool timeout_expired(T past_time, T now, T timeout);

template<typename T>
T timeout_remaining(T past_time, T now, T timeout);
```

**Meridian**: `embassy_time::Instant` + `Duration`. The `panic!()` macro replaces `panic()`.

---

## II.3 Scheduler Interface (`AP_HAL/Scheduler.h`, ChibiOS impl in `AP_HAL_ChibiOS/Scheduler.h`)

### Thread Architecture (ChibiOS implementation)

| Thread | ChibiOS Priority | Stack | Rate | Role |
|--------|-----------------|-------|------|------|
| monitor | 183 | 1024 B | 10 Hz | Watchdog, deadlock detection |
| timer | 181 | 1536 B | 1 kHz | Registered timer callbacks + ADC |
| rcout | 181 | 512 B | event | PWM DMA output |
| rcin | 177 | 1024 B | 1 kHz | RC input decoding |
| io | 58 | 2048 B | ~100 Hz | IO registered callbacks |
| storage | 59 | 1024 B | event | EEPROM/flash writes |
| main | 180 | 8-16 KB | ~400 Hz | Vehicle fast_loop |
| IOMCU | 182 (boost) | 1024 B | event | IO coprocessor comms |

Priority constants (higher number = higher priority on ChibiOS):
- `APM_MONITOR_PRIORITY = 183` (deadlock watchdog — highest)
- `APM_TIMER_PRIORITY = APM_SPI_PRIORITY = APM_RCOUT_PRIORITY = 181`
- `APM_MAIN_PRIORITY_BOOST = 182` (main during loop start)
- `APM_MAIN_PRIORITY = 180`
- `APM_RCIN_PRIORITY = 177`
- `APM_I2C_PRIORITY = 176`
- `APM_IO_PRIORITY = 58`
- `APM_STORAGE_PRIORITY = 59`

### Scheduler Methods

```cpp
virtual void init() = 0;
virtual void delay(uint16_t ms) = 0;
virtual void delay_microseconds(uint16_t us) = 0;

// Priority boost: raises main thread to BOOST priority at end of sleep
// so main loop pre-empts SPI/timer threads at start of each cycle
virtual void delay_microseconds_boost(uint16_t us);
virtual void boost_end(void);

// Watchdog suppression for expected long operations
virtual void expect_delay_ms(uint32_t ms);   // 0 = cancel
virtual bool in_expected_delay(void) const;

// Callback registration
virtual void register_timer_process(AP_HAL::MemberProc) = 0;  // 1kHz
virtual void register_io_process(AP_HAL::MemberProc) = 0;     // ~100Hz
virtual void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) = 0;
virtual void register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);

// State
virtual void set_system_initialized() = 0;
virtual bool is_system_initialized() = 0;
virtual void reboot(bool hold_in_bootloader = false) = 0;
virtual bool in_main_thread() const = 0;

// Critical section
virtual void* disable_interrupts_save(void);
virtual void restore_interrupts(void*);

// Thread creation
virtual bool thread_create(AP_HAL::MemberProc proc, const char* name,
                           uint32_t stack_size, priority_base base, int8_t priority);
```

Max timer callbacks: `CHIBIOS_SCHEDULER_MAX_TIMER_PROCS = 8` (compile-time limit).

**Timer thread behavior**: wakes every 1ms, calls all registered MemberProcs, calls failsafe if registered, calls `AnalogIn::_timer_tick()`. Runs whether or not the main loop is healthy.

**Priority boost mechanism**: `delay_microseconds_boost(us)` raises main thread to priority 182 before sleeping. When it wakes, main thread is higher than timer/SPI threads (181), ensuring it gets CPU first at the start of each loop cycle. `boost_end()` returns to 180.

**`EXPECT_DELAY_MS(ms)` macro**: RAII wrapper — constructor calls `expect_delay_ms(ms)`, destructor calls `expect_delay_ms(0)`. Suppresses watchdog and error messages during intentional long delays (SD card mount, sensor init, etc.).

**Meridian**: RTIC task priorities or Embassy task priorities. The boost mechanism requires priority ceiling protocol at loop boundaries. `disable_interrupts_save` → `cortex_m::interrupt::free(|cs| {...})`.

---

## II.4 UART Driver Interface (`AP_HAL/UARTDriver.h`)

### Public Methods

```cpp
void begin(uint32_t baud);
void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace);
void begin_locked(uint32_t baud, uint16_t rxSpace, uint16_t txSpace, uint32_t write_key);
void end();
void flush();
uint32_t available() override;
bool discard_input() override;
size_t write(uint8_t c) override;
size_t write(const uint8_t* buffer, size_t size) override;
size_t write(const char* str) override;
size_t write_locked(const uint8_t* buffer, size_t size, uint32_t key);
int16_t read(void) override;         // -1 if no data
bool read(uint8_t& b) override;
ssize_t read(uint8_t* buffer, uint16_t count) override;
ssize_t read_locked(uint8_t* buf, size_t count, uint32_t key);
bool lock_port(uint32_t write_key, uint32_t read_key);
bool is_initialized() = 0;
bool tx_pending() = 0;
uint32_t get_baud_rate() const;
bool is_dma_enabled() const;
uint64_t receive_time_constraint_us(uint16_t nbytes);
bool wait_timeout(uint16_t n, uint32_t timeout_ms);
bool set_options(uint16_t options);
void set_flow_control(enum flow_control);
void configure_parity(uint8_t v);   // 0=none, 1=odd, 2=even
void set_stop_bits(int n);
bool set_unbuffered_writes(bool on); // IOMCU uses this: bypasses ring buffer
```

### Option Bits
```cpp
OPTION_RXINV       = (1<<0)   // invert RX
OPTION_TXINV       = (1<<1)   // invert TX
OPTION_HDPLEX      = (1<<2)   // half-duplex (ESC one-wire)
OPTION_SWAP        = (1<<3)   // swap RX/TX pins
OPTION_PULLDOWN_RX = (1<<4)
OPTION_PULLUP_RX   = (1<<5)
OPTION_PULLDOWN_TX = (1<<6)
OPTION_PULLUP_TX   = (1<<7)
OPTION_NODMA_RX    = (1<<8)
OPTION_NODMA_TX    = (1<<9)
OPTION_NOFIFO      = (1<<11)
```

### Flow Control
```cpp
FLOW_CONTROL_DISABLE = 0
FLOW_CONTROL_ENABLE  = 1
FLOW_CONTROL_AUTO    = 2   // auto-detect by observing TX buffer full
FLOW_CONTROL_RTS_DE  = 3   // RTS pin = RS-485 driver enable
```

### Protected Backend (must implement)
```cpp
virtual void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) = 0;
virtual size_t _write(const uint8_t* buffer, size_t size) = 0;
virtual ssize_t _read(uint8_t* buffer, uint16_t count) = 0;
virtual void _end() = 0;
virtual void _flush() = 0;
virtual uint32_t _available() = 0;
virtual bool _discard_input(void) = 0;
```

The public methods check port locking before delegating to protected versions.

**Tricky behaviors**:
- `OPTION_HDPLEX`: half-duplex single-wire UART (ESC protocols like FrSky telemetry). TX and RX share one wire — hardware must disable RX while transmitting.
- `set_unbuffered_writes(true)`: IOMCU uses this — writes go directly to hardware FIFO/DMA, no ring buffer. Needed for the 1.5 Mbps IOMCU protocol.
- Lock keys: 32-bit arbitrary tokens. `write_locked()` / `read_locked()` return 0/-1 if key mismatch. Used for MAVLink passthrough.
- `receive_time_constraint_us(nbytes)`: estimates when a packet started arriving, accounting for baud rate delays. Used for GPS timestamp correction.

**Meridian**: `embedded_io::Read + Write` traits. HDPLEX maps to `embedded_hal::serial` with direction-pin control. Lock mechanism requires a `Mutex`-guarded key state. DMA UART via STM32 DMA peripheral.

---

## II.5 SPI / I2C Device Interfaces (`AP_HAL/Device.h`, `SPIDevice.h`, `I2CDevice.h`)

### Device Base Class

All bus devices (SPI, I2C, WSPI) share this base:

```cpp
class AP_HAL::Device {
    enum BusType { UNKNOWN=0, I2C=1, SPI=2, UAVCAN=3, SITL=4, MSP=5, SERIAL=6, WSPI=7 };
    enum Speed { SPEED_HIGH, SPEED_LOW };

    // 32-bit bus ID: [bus_type:3][bus:5][address:8][devtype:8]
    uint32_t get_bus_id(void) const;

    virtual bool transfer(const uint8_t* send, uint32_t send_len,
                          uint8_t* recv, uint32_t recv_len) = 0;
    virtual bool transfer_fullduplex(uint8_t* send_recv, uint32_t len);

    // Convenience wrappers built on transfer()
    bool read_registers(uint8_t first_reg, uint8_t* recv, uint32_t recv_len);
    bool write_register(uint8_t reg, uint8_t val, bool checked=false);
    bool read(uint8_t* recv, uint32_t recv_len);

    // Banked register access (for ICM-42xxx IMUs)
    bool read_bank_registers(uint8_t bank, uint8_t first_reg, uint8_t* recv, uint32_t recv_len);
    bool write_bank_register(uint8_t bank, uint8_t reg, uint8_t val, bool checked=false);

    // Background register integrity checking
    bool setup_checked_registers(uint8_t num_regs, uint8_t frequency=10);
    bool check_next_register(void);   // returns false on mismatch

    // Semaphore: one per bus, shared by all devices on that bus
    virtual AP_HAL::Semaphore* get_semaphore() = 0;

    // Periodic callback: runs in bus thread, semaphore already held
    // DO NOT acquire other semaphores inside the callback
    virtual PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) = 0;
    virtual bool adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) = 0;
    virtual bool unregister_callback(PeriodicHandle h);

    // DMA completion callback (for async transfers — semaphore must be held until callback)
    virtual void register_completion_callback(AP_HAL::MemberProc proc);

    // Direct CS control (for devices with unusual transfer patterns)
    virtual bool set_chip_select(bool set);

    void set_read_flag(uint8_t flag);  // ORed into register address on read (SPI: typically 0x80)
    virtual void set_speed(Speed speed) = 0;
    virtual void set_retries(uint8_t retries);
    void setup_bankselect_callback(BankSelectCb bank_select);
```

### SPI-Specific (`SPIDevice.h`)

```cpp
class AP_HAL::SPIDevice : public Device {
    virtual bool transfer_fullduplex(const uint8_t* send, uint8_t* recv, uint32_t len) = 0;
    virtual bool clock_pulse(uint32_t len);  // SD card SPI init (send N clocks with CS high)
    virtual void set_slowdown(uint8_t slowdown);  // reduce clock speed (for SD card issues)
};

class AP_HAL::SPIDeviceManager {
    virtual SPIDevice* get_device_ptr(const char* name) = 0;  // lookup by name string
    virtual uint8_t get_count();
    virtual const char* get_device_name(uint8_t idx);
};
```

**Naming convention**: Device names like `"mpu6000"`, `"icm42688"`, `"bmi088_g"` are defined in hwdef `.dat` files and matched by the SPI bus driver.

**Critical behavior**: `register_periodic_callback()` schedules work on the SPI bus thread. The callback runs with the bus semaphore held. Never acquire another semaphore inside the callback (deadlock risk if that semaphore is taken from main/timer threads).

### I2C-Specific (`I2CDevice.h`)

```cpp
class AP_HAL::I2CDevice : public Device {
    // Lookup by (bus, address) — not by name like SPI
    // Manager: get_device_ptr(bus, address, bus_clock=400000, use_smbus=false, timeout_ms=4)

    virtual bool read_registers_multiple(uint8_t first_reg, uint8_t* recv,
                                         uint32_t recv_len, uint8_t times) = 0;
    virtual void set_split_transfers(bool set);  // force STOP between write and read phases
};

class AP_HAL::I2CDeviceManager {
    virtual uint32_t get_bus_mask(void) const;          // all buses bitmask
    virtual uint32_t get_bus_mask_external(void) const; // external connector buses
    virtual uint32_t get_bus_mask_internal(void) const; // internal (IMU bus)
};

// Iteration macros
FOREACH_I2C(i)           // all buses
FOREACH_I2C_EXTERNAL(i)  // external buses
FOREACH_I2C_INTERNAL(i)  // internal buses
```

**Tricky**: `set_split_transfers(true)` forces a STOP+START between write and read phases. Some sensors (notably certain magnetometers) require this instead of a repeated START.

**Meridian**: `embedded_hal::i2c::I2c` trait. `bus_clock` → I2C peripheral speed config. `use_smbus` → SMBus mode (adds PEC, timing changes). `timeout_ms` → per-transfer watchdog.

---

## II.6 GPIO Interface (`AP_HAL/GPIO.h`)

```cpp
class AP_HAL::GPIO {
    // Direction: HAL_GPIO_INPUT=0, HAL_GPIO_OUTPUT=1, HAL_GPIO_ALT=2
    virtual void init() = 0;
    virtual void pinMode(uint8_t pin, uint8_t output) = 0;
    virtual void pinMode(uint8_t pin, uint8_t output, uint8_t alt);  // with alt function number
    virtual uint8_t read(uint8_t pin) = 0;
    virtual void write(uint8_t pin, uint8_t value) = 0;
    virtual void toggle(uint8_t pin) = 0;
    virtual bool valid_pin(uint8_t pin) const;
    virtual bool pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const;
    virtual bool get_mode(uint8_t pin, uint32_t& mode);  // save current mode
    virtual void set_mode(uint8_t pin, uint32_t mode);   // restore mode
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;  // OO pin object
    virtual bool usb_connected(void) = 0;

    // Interrupt attach: callback(pin, new_state, timestamp_us)
    enum INTERRUPT_TRIGGER_TYPE { INTERRUPT_NONE, INTERRUPT_FALLING, INTERRUPT_RISING, INTERRUPT_BOTH };
    virtual bool attach_interrupt(uint8_t pin, irq_handler_fn_t fn, INTERRUPT_TRIGGER_TYPE mode);
    virtual bool wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us);
};
```

**PWMSource** — measures PWM input on a GPIO pin using interrupts:
```cpp
class AP_HAL::PWMSource {
    bool set_pin(int16_t new_pin, const char* subsystem);
    uint16_t get_pwm_us();      // last measured pulse width
    uint16_t get_pwm_avg_us();  // averaged since last call
};
```
Uses `attach_interrupt(INTERRUPT_BOTH)` on rising/falling edges with timestamps.

**Meridian**: `embedded_hal::digital::{InputPin, OutputPin}`. Interrupts via RTIC/Embassy EXTI tasks. GPIO pin numbers are board-specific and typically defined via constants in hwdef.

---

## II.7 RC Output Interface (`AP_HAL/RCOutput.h`)

```cpp
// Range: 400–2100 us
class AP_HAL::RCOutput {
    virtual void init() = 0;
    virtual void set_freq(uint32_t chmask, uint16_t freq_hz) = 0;
    virtual uint16_t get_freq(uint8_t chan) = 0;
    virtual void enable_ch(uint8_t chan) = 0;
    virtual void disable_ch(uint8_t chan) = 0;
    virtual void write(uint8_t chan, uint16_t period_us) = 0;

    // Batched update: cork() buffers writes, push() commits atomically
    virtual void cork() = 0;
    virtual void push() = 0;

    virtual uint16_t read(uint8_t chan) = 0;  // last value (from IOMCU if present)
    virtual void set_failsafe_pwm(uint32_t chmask, uint16_t period_us);
    virtual bool force_safety_on(void);
    virtual void force_safety_off(void);

    enum output_mode {
        MODE_PWM_NONE,
        MODE_PWM_NORMAL,      // 50Hz standard servo
        MODE_PWM_ONESHOT,     // oneshot
        MODE_PWM_ONESHOT125,  // 125-250us pulses
        MODE_PWM_BRUSHED,     // 0-100% DC
        MODE_PWM_DSHOT150/300/600/1200,  // DShot protocols
        MODE_NEOPIXEL,        // WS2812B
        MODE_PROFILED,        // ProfiLED
        MODE_NEOPIXELRGB,
    };
    virtual void set_output_mode(uint32_t mask, enum output_mode mode);
    virtual uint32_t get_disabled_channels(uint32_t digital_mask);  // channels blocked by DShot group

    // DShot commands (BLHeli spec)
    enum BLHeliDshotCommand : uint8_t {
        DSHOT_RESET=0, DSHOT_BEEP1..5=1..5,
        DSHOT_ROTATE=7, DSHOT_REVERSE=21,
        DSHOT_3D_ON=10, DSHOT_3D_OFF=9,
        DSHOT_SAVE=12, DSHOT_NORMAL=20,
        // LED commands 22-29
    };
    virtual void send_dshot_command(uint8_t command, uint8_t chan=ALL_CHANNELS,
        uint32_t timeout_ms=0, uint16_t repeat_count=10, bool priority=false);

    // Bi-directional DShot (ERPM telemetry)
    virtual uint16_t get_erpm(uint8_t chan) const;
    virtual float get_erpm_error_rate(uint8_t chan) const;
    virtual bool new_erpm();
    virtual uint32_t read_erpm(uint16_t* erpm, uint8_t len);
    virtual void set_bidir_dshot_mask(uint32_t mask);

    // DShot bit timing (default: 8/3/6 ticks = 37.5%/75% duty)
    static constexpr uint32_t DSHOT_BIT_WIDTH_TICKS_DEFAULT = 8;
    static constexpr uint32_t DSHOT_BIT_0_TICKS_DEFAULT = 3;
    static constexpr uint32_t DSHOT_BIT_1_TICKS_DEFAULT = 6;

    // ESC scaling for percentage-based ESCs (UAVCAN, etc.)
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm);
    float scale_esc_to_unity(uint16_t pwm) const;  // maps pwm to [-1, 1]
};
```

**Tricky**: DShot channels are grouped by hardware timer. All channels in a group must use the same mode. `get_disabled_channels(mask)` returns channels that are incompatible with a DShot group mask. Bi-directional DShot requires input capture on the same wire — the timer fires DMA output then switches to capture mode between pulses.

**Meridian**: PWM via STM32 TIM peripheral in DMA mode. DShot: pre-computed bit patterns pushed via DMA. Bi-directional DShot requires TIM DMA Burst mode with CC + capture.

---

## II.8 RC Input Interface (`AP_HAL/RCInput.h`)

```cpp
// Range: 900–2100 us
class AP_HAL::RCInput {
    virtual void init() = 0;
    virtual bool new_input(void) = 0;        // true if new frame received
    virtual uint8_t num_channels() = 0;
    virtual uint16_t read(uint8_t ch) = 0;   // single channel (0-indexed)
    virtual uint8_t read(uint16_t* periods, uint8_t len) = 0;  // all channels
    virtual int16_t get_rssi(void);           // -1=unknown, 0=no link, 255=max
    virtual int16_t get_rx_link_quality(void);
    virtual const char* protocol() const;     // "SBUS", "CRSF", "IBUS", etc.
    virtual void pulse_input_enable(bool enable);  // disable when using UART RC
};
```

On boards with IOMCU: RC input is read from the IO MCU via `PAGE_RAW_RCIN`. The `num_channels` is up to `IOMCU_MAX_RC_CHANNELS = 16`.

---

## II.9 Analog Input Interface (`AP_HAL/AnalogIn.h`)

```cpp
class AP_HAL::AnalogIn {
    virtual void init() = 0;
    virtual AP_HAL::AnalogSource* channel(int16_t n) = 0;
    virtual float board_voltage(void) = 0;          // 5V rail in volts
    virtual float servorail_voltage(void);
    virtual uint16_t power_status_flags(void);       // MAV_POWER_STATUS bitmask

    enum class PowerStatusFlag : uint16_t {
        BRICK_VALID=1, SERVO_VALID=2, USB_CONNECTED=4,
        PERIPH_OVERCURRENT=8, PERIPH_HIPOWER_OVERCURRENT=16, CHANGED=32
    };

    // if HAL_WITH_MCU_MONITORING
    virtual float mcu_temperature(void);
    virtual float mcu_voltage(void);
};

class AP_HAL::AnalogSource {
    virtual float read_average() = 0;               // averaged, 0.0-1.0
    virtual float read_latest() = 0;                // most recent sample
    virtual bool set_pin(uint8_t p) = 0;
    virtual float voltage_average() = 0;            // in volts vs 5V reference
    virtual float voltage_latest() = 0;
    virtual float voltage_average_ratiometric() = 0;  // for ratiometric sensors
};
// Special pins: ANALOG_INPUT_BOARD_VCC=254, ANALOG_INPUT_NONE=255
```

---

## II.10 Storage Interface (`AP_HAL/Storage.h`)

```cpp
class AP_HAL::Storage {
    virtual void init() = 0;
    virtual bool erase();
    virtual void read_block(void* dst, uint16_t src, size_t n) = 0;
    virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;
    virtual void _timer_tick(void);         // background flush (called from timer thread)
    virtual bool healthy(void);
    virtual bool get_storage_ptr(void*& ptr, size_t& size);  // direct memory if available
};
```

Used by AP_Param for parameter storage. 16-bit address space (max 64KB). ChibiOS implementation: write-combining buffer in RAM, storage thread flushes dirty pages to internal flash. `write_block()` is not immediately persistent — `_timer_tick()` handles flush.

---

## II.11 Flash Interface (`AP_HAL/Flash.h`)

```cpp
class AP_HAL::Flash {
    virtual uint32_t getpageaddr(uint32_t page) = 0;
    virtual uint32_t getpagesize(uint32_t page) = 0;
    virtual uint32_t getnumpages(void) = 0;
    virtual bool erasepage(uint32_t page) = 0;
    virtual bool write(uint32_t addr, const void* buf, uint32_t count) = 0;
    virtual void keep_unlocked(bool set) = 0;  // for bootloader writes
    virtual bool ispageerased(uint32_t page) = 0;
};
```

Used for bootloader updates, dataflash log storage (H7 dual-bank), and parameter backup.

---

## II.12 Semaphore / Mutex Model (`AP_HAL/Semaphores.h`)

### Recursive Mutex (`AP_HAL::Semaphore`)

```cpp
class AP_HAL::Semaphore {
    virtual bool take(uint32_t timeout_ms) = 0;   // 0 = block forever
    virtual bool take_nonblocking() = 0;
    virtual void take_blocking();                  // always blocks
    virtual bool give() = 0;
};
```

**All ArduPilot semaphores are recursive** — same thread can take it multiple times, must give same number of times. ChibiOS implementation: `mutex_t` (ChibiOS mutexes are recursive by design).

RAII wrapper:
```cpp
WITH_SEMAPHORE(sem)   // blocks forever, releases at scope exit
// Expands to: WithSemaphore _getsem<counter>(sem, __LINE__)
// Constructor records persistent_data.semaphore_line for watchdog debugging
```

### Binary Semaphore (`AP_HAL::BinarySemaphore`)

```cpp
class AP_HAL::BinarySemaphore {
    BinarySemaphore(bool initial_state=false);  // true = pre-signaled
    virtual bool wait(uint32_t timeout_us) = 0;
    virtual bool wait_blocking() = 0;
    virtual bool wait_nonblocking();
    virtual void signal() = 0;
    virtual void signal_ISR();    // ISR-safe version
};
```

ChibiOS implementation: `binary_semaphore_t` (not `mutex_t`). Note: ArduPilot initial_state semantics are opposite to ChibiOS — the constructor negates `initial_state` when calling `chBSemObjectInit`.

### Locking Patterns

1. **SPI/I2C bus lock**: `WITH_SEMAPHORE(dev->get_semaphore())` — one per bus, held for entire transaction
2. **Timer process list**: protected by a `binary_semaphore_t _timer_semaphore`
3. **IO process list**: protected by `_io_semaphore`
4. **Per-driver state**: each driver has its own recursive mutex
5. **DMA completion**: `BinarySemaphore::signal_ISR()` from DMA ISR, `wait()` in bus thread

**Watchdog deadlock detection**: `WithSemaphore` records `persistent_data.semaphore_line`. Monitor thread checks if main loop is stuck >500ms then calls `try_force_mutex()` to forcibly release the held mutex.

**Critical for Meridian**: ArduPilot's recursive mutexes cannot be directly replaced with Rust's non-recursive `Mutex<T>`. Options:
1. Redesign drivers to avoid recursive lock acquisition (preferred)
2. Implement a counting recursive mutex wrapper with thread ID tracking
3. Use `RefCell` within single-threaded sections where re-entrancy occurs

---

## II.13 Util Interface (`AP_HAL/Util.h`)

### Key Methods

```cpp
class AP_HAL::Util {
    // Arming state management
    virtual void set_soft_armed(const bool b);
    bool get_soft_armed() const;
    uint32_t get_last_armed_change() const;

    // Watchdog state
    virtual bool was_watchdog_reset() const;
    bool was_watchdog_safety_off() const;
    bool was_watchdog_armed() const;

    // Safety switch
    virtual enum safety_state safety_switch_state(void);  // NONE, DISARMED, ARMED

    // RTC
    virtual void set_hw_rtc(uint64_t time_utc_usec) = 0;
    virtual uint64_t get_hw_rtc() const = 0;

    // DMA-aware memory allocation
    enum Memory_Type { MEM_DMA_SAFE, MEM_FAST, MEM_FILESYSTEM };
    virtual void* malloc_type(size_t size, Memory_Type mem_type);
    virtual void free_type(void* ptr, size_t size, Memory_Type mem_type);
    virtual uint32_t available_memory(void);

    // System ID
    virtual bool get_system_id(char buf[50]);

    // Bootloader
    enum class FlashBootloader { OK, NO_CHANGE, FAIL, NOT_AVAILABLE, NOT_SIGNED };
    virtual FlashBootloader flash_bootloader();
    virtual void boot_to_dfu(void);

    // IMU heater (thermal stability for accurate sensor readings)
    virtual void set_imu_temp(float current);
    virtual void set_imu_target_temp(int8_t* target);

    // PRNG
    virtual bool get_random_vals(uint8_t* data, size_t size);
    virtual bool get_true_random_vals(uint8_t* data, size_t size, uint32_t timeout_us);

    // Diagnostics
    virtual void thread_info(ExpandingString& str);
    virtual void dma_info(ExpandingString& str);
    virtual bool get_system_load(float& avg_load, float& peak_load) const;
```

### Persistent Data (Watchdog Survival) — 76 bytes max, stored in RTC backup regs

```cpp
struct PersistentData {
    float roll_rad, pitch_rad, yaw_rad;    // attitude at reset
    int32_t home_lat, home_lon, home_alt_cm;
    uint32_t fault_addr;                    // crash address
    uint32_t fault_icsr;                    // ICSR register
    uint32_t fault_lr;                      // link register
    uint32_t internal_errors;
    uint16_t internal_error_count;
    uint16_t internal_error_last_line;
    uint32_t spi_count;                     // debug: SPI transfer count
    uint32_t i2c_count;
    uint32_t i2c_isr_count;
    uint16_t waypoint_num;                  // current mission waypoint
    uint16_t last_mavlink_msgid;
    uint16_t last_mavlink_cmd;
    uint16_t semaphore_line;                // last line where main took a semaphore
    uint16_t fault_line;
    uint8_t fault_type;
    uint8_t fault_thd_prio;
    char thread_name4[4];
    int8_t scheduler_task;
    bool armed;
    enum safety_state safety_state;
    bool boot_to_dfu;
};
```

### Memory Model (STM32H7 specific — critical for DMA)

| `Memory_Type` | H7 Region | DMA Accessible | Usage |
|---------------|-----------|----------------|-------|
| `MEM_DMA_SAFE` | SRAM1/2/3 | Yes (all DMA) | SPI/UART/ADC DMA buffers |
| `MEM_FAST` | DTCM | No | FFT, filter state, frequently-accessed arrays |
| `MEM_FILESYSTEM` | SRAM / ext SDRAM | Varies | FS cache, log buffers |

**CRITICAL**: DMA buffers in DTCM or SRAM4 cause silent data corruption on STM32H7. All driver-allocated DMA buffers must use `malloc_type(size, MEM_DMA_SAFE)`.

**Meridian**: Use `#[link_section = ".sram1"]` / `#[link_section = ".dtcm"]` for statically allocated buffers. For dynamic allocation, create separate pool allocators per memory region.

Stack sizes (ChibiOS): timer=1536B, rcout=512B, rcin=1024B, io=2048B, storage=1024B, monitor=1024B, IOMCU=1024B.

---

## II.14 CAN Interface (`AP_HAL/CANIface.h`)

```cpp
struct AP_HAL::CANFrame {
    uint32_t id;            // with FlagEFF (1<<31), FlagRTR (1<<30), FlagERR (1<<29)
    uint8_t data[64];       // up to 64 for CAN-FD
    uint8_t dlc;
    bool canfd;
};

class AP_HAL::CANIface {
    virtual bool init(const uint32_t bitrate) = 0;
    virtual bool init(const uint32_t bitrate, const uint32_t fdbitrate);
    // send(): returns -1=error, 0=no space, 1=success
    virtual int16_t send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags);
    // receive(): returns -1=error, 0=no frame, 1=success
    virtual int16_t receive(CANFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags);
    virtual bool select(bool& read_select, bool& write_select, const CANFrame* pending_tx, uint64_t timeout);
    virtual bool set_event_handle(AP_HAL::BinarySemaphore* sem_handle);
    virtual bool register_frame_callback(FrameCb cb, uint8_t& cb_id);  // up to 3 per interface
    virtual bool is_initialized() const = 0;
    virtual bool is_busoff() const;
};
```

`tx_deadline`: 64-bit us timestamp; frames not sent by deadline are dropped. DroneCAN/UAVCAN v1 runs on top of this interface.

---

## II.15 AP_BoardConfig — Initialization Sequence and Sensor Detection

### Initialization Order

`AP_BoardConfig::init()`:
1. Parameter migration (`convert_parameter_width`)
2. `board_setup()`:
   - `hal.gpio->init()`
   - `hal.rcin->init()`
   - `hal.rcout->init()`
   - Set PWM voltage (3.3V or 5V via `BRD_PWM_VOLT_SEL`)
   - `board_setup_uart()` — apply RTSCTS settings to all serial ports
   - `board_setup_sbus()` — enable SBUS out if configured
   - `board_setup_drivers()` (if `AP_FEATURE_BOARD_DETECT`):
     - `board_autodetect()` — read WHO_AM_I registers via SPI
     - Set `px4_configured_board`
3. Set RTC from hardware
4. Apply boot delay (`BRD_BOOT_DELAY`)
5. Mount SD card with retry/slowdown loop

`AP_BoardConfig::init_safety()`:
1. `board_init_safety()` — force safety off if `BRD_SAFETY_DEFLT=0` or watchdog-armed
2. `board_init_debug()` — disable SWD pins if not in debug mode

### Board Auto-Detection (SPI WHO_AM_I Probe)

The `spi_check_register(devname, regnum, expected_value)` function:
1. Opens device by name via `hal.spi->get_device(devname)`
2. Takes bus semaphore
3. Sets SPEED_LOW
4. Reads register at `regnum`
5. Returns `value == expected`

**FMUv3 detection logic** (Cube/Pixhawk1/2):
- Pixhawk2 (Cube): external MPU + external LSM303D detected
- PixhawkMini: internal ICM20608-AM + internal MPU9250
- Pixhawk1: internal LSM303D + internal MPU6000/9250

**FMUv6 detection logic** (Pixhawk 6X family):
- ICM42688 + ICM42670 + (ICM20649 or BMI088) → Holybro 6X
- ICM20649_2 + ICM42688 + BMI088 → CUAV 6X
- 3x ICM45686 → Holybro 6X_45686
- IIM42652 + ICM45686 → Holybro 6X Rev6

`config_error()` enters an infinite GCS-connected error loop if detection fails — allows user to fix `BRD_TYPE` parameter via MAVLink without hardware access.

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `BRD_SAFETY_DEFLT` | 1 (if safety pin) | Safety starts flashing on boot |
| `BRD_SAFETY_MASK` | 0 | Channels ignoring safety switch |
| `BRD_SAFETYOPTION` | 0x03 | Button active for safety off + on |
| `BRD_OPTIONS` | WATCHDOG | Watchdog, FTP, debug pins, flash protect |
| `BRD_IO_ENABLE` | 1 | 0=no IOMCU, 1=enabled, 2=no FW update |
| `BRD_IO_DSHOT` | 0 | 1=load DShot firmware on IOMCU |
| `BRD_BOOT_DELAY` | 0 ms | Extra startup delay |
| `BRD_HEAT_TARG` | -1 | IMU heater target °C (-1=disabled) |
| `BRD_VBUS_MIN` | 4.3V | Minimum main power for arming |
| `BRD_VSERVO_MIN` | 0V | Minimum servo rail for arming |
| `BRD_PWM_VOLT_SEL` | 0 (3.3V) | PWM output voltage |

---

## II.16 IOMCU Protocol (`AP_IOMCU/AP_IOMCU.h`, `iofirmware/ioprotocol.h`)

### Role of the IO MCU

On Pixhawk-family boards (Cube, H743-based), a separate STM32F1xx handles:
- PWM output for 8 channels (with DMA-accurate timing)
- RC input decoding (SBUS, PPM, CRSF, DSM, IBUS — via `AP_RCProtocol`)
- Safety switch state machine (LED + button)
- Failsafe mixing (autonomous operation if FMU crashes)
- SBUS output
- ADC monitoring (servo rail voltage `vservo`, RSSI voltage `vrssi`)
- IMU heater control (via duty cycle)

### Communication

UART at **1.5 Mbps**, unbuffered writes. Packet format:

```cpp
struct IOPacket {
    uint8_t count:6;    // number of uint16_t payload registers
    uint8_t code:2;     // CODE_READ=0, CODE_WRITE=1 / CODE_SUCCESS=0, CODE_CORRUPT=1, CODE_ERROR=2
    uint8_t crc;        // CRC8 over entire packet
    uint8_t page;       // register page number
    uint8_t offset;     // offset within page
    uint16_t regs[22];  // payload (PKT_MAX_REGS=22)
};
// Packet size = count * 2 + 4 bytes
```

`IOMCU_MAX_RC_CHANNELS = 16`, `IOMCU_MAX_CHANNELS = 8` (physical outputs).

### Register Pages

| Page | ID | Key Contents |
|------|----|--------------|
| PAGE_CONFIG | 0 | Protocol version (v4/v10), MCU ID, CPU ID |
| PAGE_STATUS | 1 | Free mem, errors, timestamp, vservo, vrssi, safety_off flag |
| PAGE_ACTUATORS | 2 | Actuator targets |
| PAGE_SERVOS | 3 | Servo position readback |
| PAGE_RAW_RCIN | 4 | 16-ch PWM + failsafe flag + rc_ok + rc_protocol + RSSI |
| PAGE_SETUP | 50 | Features, arming, rates, force safety, heater, DSM bind |
| PAGE_DIRECT_PWM | 54 | Direct 8-ch PWM values |
| PAGE_FAILSAFE_PWM | 55 | Failsafe values (autonomous if FMU silent) |
| PAGE_MIXING | 200 | FMU-failsafe mixer: servo limits, RC limits, mixing gain |
| PAGE_GPIO | 201 | Channel GPIO mask + output state |
| PAGE_DSHOT | 202 | DShot command: telem_mask, command, chan, timeout, repeat |
| PAGE_RAW_DSHOT_ERPM | 203 | 4-ch ERPM + update_mask |
| PAGE_RAW_DSHOT_TELEM_1_4..13_16 | 204-207 | Error rate, voltage, current, temp per ESC |

### Key SETUP Registers

```
REG_SETUP_FEATURES:       SBUS1_OUT(1), SBUS2_OUT(2), ONESHOT(16), BRUSHED(32), HEATER(64)
REG_SETUP_ARMING:         IO_ARM_OK(1), FMU_ARMED(2), RC_HANDLING_DISABLED(64)
REG_SETUP_FORCE_SAFETY_OFF: write 22027 (FORCE_SAFETY_MAGIC)
REG_SETUP_FORCE_SAFETY_ON:  write 22027
REG_SETUP_IGNORE_SAFETY:  channel bitmask (channels that output when safety=ON)
REG_SETUP_REBOOT_BL:      write 14662 (REBOOT_BL_MAGIC) to reboot IO to bootloader
REG_SETUP_HEATER_DUTY_CYCLE: PWM duty cycle for IMU heater %
REG_SETUP_DSM_BIND:       trigger DSM receiver binding
REG_SETUP_RC_PROTOCOLS:   bitmask of enabled RC protocols (2 registers)
REG_SETUP_DSHOT_PERIOD:   period_us (u16) + rate (u16)
REG_SETUP_CHANNEL_MASK:   enabled output channel mask
```

### IOMCU Thread Event System

The FMU IOMCU thread uses ChibiOS event flags (`chEvtWaitAnyTimeout`) with a 10ms timeout. Events:

```
IOEVENT_INIT              — startup: read config, set arming register
IOEVENT_SEND_PWM_OUT      — write servo PWM to PAGE_DIRECT_PWM (highest priority)
IOEVENT_FORCE_SAFETY_OFF/ON — write magic value to setup page
IOEVENT_SET_RATES         — update altrate + chmask
IOEVENT_ENABLE_SBUS       — enable SBUS output at configured rate
IOEVENT_SET_HEATER_TARGET — update heater duty cycle
IOEVENT_SET_DEFAULT_RATE  — update default PWM rate
IOEVENT_SET_SAFETY_MASK   — update ignore_safety channel mask
IOEVENT_MIXING            — upload failsafe mixer config
IOEVENT_GPIO              — update GPIO state (PAGE_GPIO)
IOEVENT_SET_OUTPUT_MODE   — update mode_out page (DShot, oneshot, etc.)
IOEVENT_DSHOT             — send a DShot command from the command queue
```

**Lost contact**: If no register access in 1000ms → `INTERNAL_ERROR(iomcu_reset)`.

### IOMCU Initialization

1. Open UART at 1.5 Mbps, `set_unbuffered_writes(true)`
2. Verify firmware CRC (unless watchdog reset or `BRD_IO_ENABLE=2`)
3. If CRC mismatch: upload via bootloader protocol (`upload_fw()`)
4. Create IOMCU thread at `PRIORITY_BOOST+1 = 182`
5. Thread sends `IOEVENT_INIT` → reads PAGE_CONFIG
6. Sets `IO_ARM_OK | RC_HANDLING_DISABLED` in arming register

### Firmware Upload Protocol

The IO MCU bootloader uses a proprietary protocol (not STM32 standard):
```
PROTO_GET_SYNC = 0x21   → PROTO_INSYNC + PROTO_OK
PROTO_GET_DEVICE = 0x22 → board info
PROTO_CHIP_ERASE = 0x23 → erase flash
PROTO_PROG_MULTI = 0x27 → write up to 248 bytes
PROTO_GET_CRC = 0x29    → verify CRC
PROTO_REBOOT = 0x30     → reboot to application
BL_REV = 5              → supported bootloader protocol version
```

### FMU Failsafe Mixing

`setup_mixing(override_chan, mixing_gain, manual_rc_mask)` uploads `page_mixing` to IO MCU:
- `servo_min/max/trim[]`: servo limits for each output
- `servo_function[]`: output function assignments
- `servo_reversed[]`: reverse flags
- `rc_min/max/trim[]`: RC input limits (AETR order)
- `mixing_gain`: elevon/vtail mixing gain (x1000)
- `rc_chan_override`: channel that forces manual override
- `enabled = 1`: activate mixing

When FMU stops sending PWM updates, IO MCU autonomously mixes RC input to servo outputs using the stored mix configuration.

---

## II.17 Platform-Specific Behaviors Critical for Porting

### DMA Memory (STM32H7)
All SPI/UART/ADC DMA buffers must reside in SRAM1/2/3. Buffers in DTCM (0x20000000) or SRAM4 silently corrupt. `malloc_type(MEM_DMA_SAFE)` returns from the correct region. This must be enforced in Meridian via allocator regions or static placement attributes.

### SPI Bus Thread Architecture
Each SPI bus has a dedicated ChibiOS thread. The `register_periodic_callback()` posts work to that thread. Callbacks run with the bus semaphore held. **Never** take another semaphore inside a periodic callback that is held from a higher-priority thread.

### UART Lock Port Key
The lock key mechanism (`lock_port(write_key, read_key)`) allows exclusive ownership for protocols like MAVLink passthrough. Key=0 unlocks. The MAVLink library assigns arbitrary 32-bit keys.

### Safety Switch State Machine
Three states: NONE (no safety hw), DISARMED (flashing LED, PWM disabled), ARMED (solid LED, PWM enabled). Safety transition via button requires exactly 10 consecutive 10Hz presses. `BRD_SAFETYOPTION` controls which transitions are enabled. On IOMCU boards, safety is managed by IO MCU, FMU polls `PAGE_STATUS.flag_safety_off`.

### Watchdog Architecture
- ChibiOS uses STM32 IWDG (Independent Watchdog)
- Timer thread pats watchdog every 1ms when `in_expected_delay()` is true
- Monitor thread pats in its 100ms check loop
- Main loop pats via `watchdog_pat()` in AP_Scheduler
- Monitor detects main loop stuck >500ms → log + try force mutex
- Monitor detects >1800ms → deliberate crash to generate crash dump

### Priority Boost for Loop Timing
`delay_microseconds_boost(us)` raises main thread from priority 180 to 182 before sleeping. It wakes at priority 182 — higher than SPI/timer/rcout threads (181). This ensures the main loop pre-empts all sensor threads at the start of each loop cycle, reducing jitter.

---

## II.18 Meridian Implementation Priority Matrix

### Phase 1 — Must Have for Motor Spin

| Interface | Methods | Notes |
|-----------|---------|-------|
| `Scheduler::delay*()` | `delay()`, `delay_microseconds()` | Accurate to 100us |
| `Scheduler::thread_create()` | Full | Priority mapping |
| `Scheduler::in_main_thread()` | Full | Thread identity |
| `Semaphore` (recursive) | `take/give` | Critical — ArduPilot re-enters |
| `BinarySemaphore` | `wait/signal/signal_ISR` | DMA completion |
| `UARTDriver` | `begin/_write/_read` | For IOMCU |
| `RCOutput::write/cork/push` | Full | Motor commands |
| `RCOutput::force_safety_off` | Full | Arming |
| `Storage::read/write_block` | Full | Parameters |
| `Util::malloc_type(MEM_DMA_SAFE)` | Full | DMA safety |

### Phase 2 — Sensor Connectivity

| Interface | Methods | Notes |
|-----------|---------|-------|
| `SPIDevice::transfer` + periodic callback | Full | IMU, baro |
| `I2CDevice::transfer` | Full | Compass, sensors |
| `GPIO::attach_interrupt` with timestamp | Full | RC input |
| `AnalogIn` + voltage scaling | Full | Battery monitor |
| `AP_BoardConfig::init` | Full | Board detection |

### Phase 3 — Full Feature Parity

| Interface | Notes |
|-----------|-------|
| `DSP` (FFT/notch) | Harmonic notch filter for vibration rejection |
| `CANIface` | DroneCAN peripherals |
| `WSPIDevice` | External flash |
| `OpticalFlow` | Optical flow sensor |
| IOMCU full protocol | If using Pixhawk-family hardware |

### Key Design Decisions

1. **Recursive mutex**: Either (a) redesign to avoid re-entrancy, or (b) implement `RecursiveMutex` tracking thread ID + count
2. **DMA memory regions**: Separate pool allocators; `MEM_DMA_SAFE` must never return DTCM
3. **SPI bus thread**: One Embassy task per bus, with a queue for periodic callbacks
4. **Priority constants**: Define Rust constants matching ChibiOS numeric levels for RTIC/Embassy
5. **IOMCU replacement**: For custom Meridian hardware, drive PWM directly from FMU — implement same safety/failsafe semantics without the IO MCU protocol overhead
6. **Watchdog**: IWDG with 2-second window; timer task pats at 1ms; monitor task checks at 100ms
7. **Periodic callback scheduler**: Use Embassy `Ticker` or RTIC `monotonic.schedule` for per-device callbacks at exact rates (e.g., ICM42688 at 8kHz)
