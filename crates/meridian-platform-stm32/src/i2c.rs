//! STM32H743 I2C driver for barometer, compass, and external sensors.
//!
//! H7 I2C peripheral uses TIMINGR register (completely different from F4 clock_speed):
//!   - 0x00707CBB for 100 kHz at 100 MHz PCLK
//!   - 0x00300F38 for 400 kHz at 100 MHz PCLK
//! These values computed with STM32CubeMX for PLL3 peripheral clock.
//!
//! Bus clear: 20 SCL pulses at boot to unstick devices with stuck SDA lines.
//! Called from boot sequence before any I2C transfers.
//!
//! Default maximum clock is 100 kHz (HAL_I2C_MAX_CLOCK). If a device requests
//! a lower clock, the bus clock drops permanently for that bus.
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/I2CDevice.cpp`

use meridian_hal::i2c::I2cDevice;
use meridian_hal::util::Semaphore;
use meridian_sync::{RecursiveMutex, TASK_ID_I2C_BUS};

// ---------------------------------------------------------------------------
// H7 I2C TIMINGR constants (computed for 100 MHz peripheral clock from PLL3)
// ---------------------------------------------------------------------------

/// TIMINGR value for 100 kHz standard mode on H7 at 100 MHz PCLK.
const H7_I2C_TIMINGR_100KHZ: u32 = 0x00707CBB;

/// TIMINGR value for 400 kHz fast mode on H7 at 100 MHz PCLK.
const H7_I2C_TIMINGR_400KHZ: u32 = 0x00300F38;

/// Number of SCL pulses for bus clear (unstick stuck SDA).
const BUS_CLEAR_SCL_PULSES: u8 = 20;

/// Delay between SCL pulses during bus clear (microseconds).
const BUS_CLEAR_PULSE_DELAY_US: u32 = 10;

/// Maximum number of I2C buses on H743.
const MAX_I2C_BUSES: usize = 4;

/// Default transfer timeout (milliseconds).
/// Formula from ArduPilot: max(4ms, 2 * (8_000_000 / clock * bytes / 1000)).
const DEFAULT_TIMEOUT_MS: u32 = 10;

/// Maximum retry count per transfer.
const MAX_RETRIES: u8 = 2;

// ---------------------------------------------------------------------------
// Bus semaphores (recursive, one per physical bus)
// ---------------------------------------------------------------------------

static I2C_BUS_MUTEX: [RecursiveMutex<()>; MAX_I2C_BUSES] = [
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
];

struct Stm32I2cSemaphore {
    bus_index: usize,
}

impl Semaphore for Stm32I2cSemaphore {
    fn take_blocking(&self) {
        let _guard = I2C_BUS_MUTEX[self.bus_index].lock(TASK_ID_I2C_BUS);
    }

    fn take(&self, _timeout_us: u32) -> bool {
        // TODO: implement timeout via cortex-m systick
        let _guard = I2C_BUS_MUTEX[self.bus_index].lock(TASK_ID_I2C_BUS);
        true
    }

    fn give(&self) {
        // Guard dropped at scope exit in full implementation.
    }
}

static I2C_SEMAPHORES: [Stm32I2cSemaphore; MAX_I2C_BUSES] = [
    Stm32I2cSemaphore { bus_index: 0 },
    Stm32I2cSemaphore { bus_index: 1 },
    Stm32I2cSemaphore { bus_index: 2 },
    Stm32I2cSemaphore { bus_index: 3 },
];

// ---------------------------------------------------------------------------
// Bus clear — 20 SCL pulses to unstick hung I2C devices
// ---------------------------------------------------------------------------

/// Perform bus clear by manually toggling SCL as push-pull GPIO output.
///
/// This is called at boot for every I2C bus. Some I2C devices (especially
/// compasses and barometers) can latch SDA low if a transaction is
/// interrupted by a reset. Clocking SCL 20 times releases them.
///
/// After the pulses, the pin is restored to alternate function mode for I2C.
fn bus_clear(scl_port: u8, scl_pin: u8) {
    // TODO: actual register access
    //
    // 1. Save current pin mode
    // let mode_saved = gpio_get_mode(scl_port, scl_pin);
    //
    // 2. Reconfigure SCL as push-pull output
    // gpio_set_mode(scl_port, scl_pin, PinMode::Output);
    //
    // 3. Toggle SCL 20 times
    // for _ in 0..BUS_CLEAR_SCL_PULSES {
    //     gpio_toggle(scl_port, scl_pin);
    //     delay_us(BUS_CLEAR_PULSE_DELAY_US);
    // }
    //
    // 4. Restore original pin mode (alternate function for I2C)
    // gpio_set_mode(scl_port, scl_pin, mode_saved);
    let _ = (scl_port, scl_pin);
}

/// Clear all I2C buses. Called once at boot before any I2C transfers.
pub fn clear_all_buses(buses: &[(u8, u8)]) {
    for &(scl_port, scl_pin) in buses {
        bus_clear(scl_port, scl_pin);
    }
}

// ---------------------------------------------------------------------------
// TIMINGR selection
// ---------------------------------------------------------------------------

/// Select the TIMINGR value for a given target speed.
/// If the requested speed is >= 400 kHz, use fast mode; otherwise standard 100 kHz.
/// The bus clock can only drop, never increase — matching ArduPilot behavior.
fn timingr_for_speed(speed_hz: u32) -> u32 {
    if speed_hz >= 400_000 {
        H7_I2C_TIMINGR_400KHZ
    } else {
        H7_I2C_TIMINGR_100KHZ
    }
}

/// Compute transfer timeout in milliseconds.
/// Formula from ArduPilot: max(4, 2 * (8_000_000 / clock_hz * nbytes / 1000))
fn transfer_timeout_ms(clock_hz: u32, nbytes: usize) -> u32 {
    let bits = nbytes as u32 * 8;
    let time_ms = if clock_hz > 0 {
        (2 * bits * 1000) / clock_hz
    } else {
        DEFAULT_TIMEOUT_MS
    };
    time_ms.max(4)
}

// ---------------------------------------------------------------------------
// Stm32I2c — one instance per physical I2C bus
// ---------------------------------------------------------------------------

/// STM32H7 I2C device driver.
///
/// Each instance represents a full I2C bus. The `address` field selects which
/// device on the bus we're talking to (set via `set_address()`).
///
/// MatekH743 I2C map:
///   I2C1: PB6(SCL), PB7(SDA) - external compass/baro
///   I2C2: PB10(SCL), PB11(SDA) - external compass/baro
pub struct Stm32I2c {
    /// Bus index (0-based: I2C1=0, I2C2=1).
    bus_index: u8,
    /// Current target I2C address (7-bit).
    address: u8,
    /// Current bus speed (Hz). Can only decrease, never increase.
    speed_hz: u32,
    /// Current TIMINGR value for the configured speed.
    timingr: u32,
    /// Number of retries on transfer failure.
    retries: u8,
    /// SCL pin for bus clear (port, pin).
    scl_port: u8,
    scl_pin: u8,
    /// Whether to use split transfers (separate write + read instead of repeated START).
    /// Required for some devices like LidarLite blue label.
    split_transfers: bool,
}

impl Stm32I2c {
    /// Create a new I2C bus driver.
    pub fn new(bus_index: u8, speed_khz: u16, scl_port: u8, scl_pin: u8) -> Self {
        let speed_hz = speed_khz as u32 * 1000;
        Self {
            bus_index,
            address: 0,
            speed_hz,
            timingr: timingr_for_speed(speed_hz),
            retries: MAX_RETRIES,
            scl_port,
            scl_pin,
            split_transfers: false,
        }
    }

    /// Initialize the I2C peripheral hardware.
    pub fn init(&self) {
        // 1. Bus clear at boot — unstick any hung devices.
        bus_clear(self.scl_port, self.scl_pin);

        // TODO: actual register access
        // 2. Enable I2C peripheral clock in RCC
        // 3. Configure SCL and SDA pins as alternate function open-drain
        // 4. Set TIMINGR register
        // 5. Enable I2C peripheral (I2C_CR1_PE)
        //
        // unsafe {
        //     let i2c = i2c_peripheral(self.bus_index);
        //     i2c.timingr.write(|w| w.bits(self.timingr));
        //     i2c.cr1.modify(|_, w| w.pe().set_bit());
        // }
    }

    /// Bus index as usize for array indexing.
    fn bus_idx(&self) -> usize {
        self.bus_index as usize
    }

    /// Perform a hardware I2C master transmit.
    fn hw_transmit(&self, addr: u8, data: &[u8], timeout_ms: u32) -> bool {
        let _ = (addr, data, timeout_ms);
        // TODO: actual register access
        // 1. i2cAcquireBus()
        // 2. bouncebuffer_setup() — ensure DMA-safe buffer
        // 3. dma_handle.lock() — acquire shared DMA stream
        // 4. i2cStart() — configure peripheral for this transfer
        // 5. Set I2C_CR2: SADD (address), NBYTES, WR direction, START
        // 6. Feed data bytes via I2C_TXDR, wait for TXIS or NACK
        // 7. Wait for TC (transfer complete)
        // 8. dma_handle.unlock()
        // 9. i2cReleaseBus()
        true
    }

    /// Perform a hardware I2C master receive.
    fn hw_receive(&self, addr: u8, buf: &mut [u8], timeout_ms: u32) -> bool {
        let _ = (addr, buf, timeout_ms);
        // TODO: actual register access
        // 1. Set I2C_CR2: SADD (address), NBYTES, RD direction, START
        // 2. Read data bytes from I2C_RXDR, wait for RXNE
        // 3. Wait for TC (transfer complete)
        true
    }

    /// Combined write-then-read using repeated START (standard I2C protocol).
    fn hw_write_read(
        &self,
        addr: u8,
        send: &[u8],
        recv: &mut [u8],
        timeout_ms: u32,
    ) -> bool {
        let _ = (addr, send, recv, timeout_ms);
        // TODO: actual register access
        // 1. Set I2C_CR2: SADD, NBYTES=send.len(), WR, START
        // 2. Feed send bytes
        // 3. Wait for TC
        // 4. Set I2C_CR2: SADD, NBYTES=recv.len(), RD, START (repeated START)
        // 5. Read recv bytes
        // 6. Wait for TC
        // 7. i2cSoftStop() — stop without full STOP condition (avoids LidarLite issue)
        true
    }
}

impl I2cDevice for Stm32I2c {
    fn set_address(&mut self, addr: u8) {
        self.address = addr;
    }

    fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> bool {
        let timeout_ms = transfer_timeout_ms(self.speed_hz, send.len() + recv.len());

        // Retry loop — up to `retries` attempts, re-acquiring DMA each time.
        for attempt in 0..=self.retries {
            let ok = if self.split_transfers && !send.is_empty() && !recv.is_empty() {
                // Split: separate write + read (no repeated START).
                self.hw_transmit(self.address, send, timeout_ms)
                    && self.hw_receive(self.address, recv, timeout_ms)
            } else if send.is_empty() {
                // Read-only.
                self.hw_receive(self.address, recv, timeout_ms)
            } else if recv.is_empty() {
                // Write-only.
                self.hw_transmit(self.address, send, timeout_ms)
            } else {
                // Standard combined write-read with repeated START.
                self.hw_write_read(self.address, send, recv, timeout_ms)
            };

            if ok {
                return true;
            }

            // On timeout, attempt bus clear (20 SCL pulses to unstick SDA).
            if attempt < self.retries {
                bus_clear(self.scl_port, self.scl_pin);
            }
        }

        false
    }

    fn get_semaphore(&self) -> &dyn Semaphore {
        &I2C_SEMAPHORES[self.bus_idx()]
    }

    fn set_speed(&mut self, speed_hz: u32) {
        // Bus clock can only drop, never increase (ArduPilot behavior).
        if speed_hz < self.speed_hz {
            self.speed_hz = speed_hz;
            self.timingr = timingr_for_speed(speed_hz);
            // Peripheral must be restarted to pick up new TIMINGR.
            // TODO: actual register access — rewrite TIMINGR
        }
    }

    fn probe(&mut self) -> bool {
        // Send empty transfer to check if a device ACKs at self.address.
        // Timeout 1ms for probe — quick fail.
        self.hw_write_read(self.address, &[], &mut [0u8; 1], 1)
    }

    fn device_id(&self) -> u32 {
        // Encode bus type (I2C=2) | bus index | address.
        let bus_type: u32 = 2; // I2C
        (bus_type << 24) | ((self.bus_index as u32) << 16) | (self.address as u32)
    }
}
