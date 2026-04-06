//! STM32H743 GPIO pin configuration and control.
//!
//! GPIO ports A-K mapped to peripheral addresses.
//! Pin encoding: port (0=A, 1=B, ... 10=K) in upper byte, pin (0-15) in lower byte.
//! This matches the u16 pin encoding used throughout meridian-hal.
//!
//! Alternate function mapping comes from board config (meridian-boardcfg).
//! Each pin can be configured as input, output, or one of 16 alternate functions.
//!
//! Source: STM32H743 reference manual, ArduPilot `AP_HAL_ChibiOS/GPIO.cpp`

use meridian_hal::gpio::{Edge, GpioPin, PinMode};

// ---------------------------------------------------------------------------
// GPIO register offsets and constants
// ---------------------------------------------------------------------------

/// Number of GPIO ports on H743 (GPIOA through GPIOK = 11 ports).
const NUM_PORTS: usize = 11;

/// Pins per port.
const PINS_PER_PORT: usize = 16;

/// Total addressable pins.
const MAX_PINS: usize = NUM_PORTS * PINS_PER_PORT;

/// GPIO base address (GPIOA = 0x5802_0000, each port +0x400).
const GPIO_BASE: u32 = 0x5802_0000;
const GPIO_PORT_STRIDE: u32 = 0x400;

/// MODER register values.
const MODER_INPUT: u32 = 0b00;
const MODER_OUTPUT: u32 = 0b01;
const MODER_ALTERNATE: u32 = 0b10;
const MODER_ANALOG: u32 = 0b11;

/// OTYPER register values.
const OTYPER_PUSH_PULL: u32 = 0;
const OTYPER_OPEN_DRAIN: u32 = 1;

/// PUPDR register values.
const PUPDR_NONE: u32 = 0b00;
const PUPDR_PULL_UP: u32 = 0b01;
const PUPDR_PULL_DOWN: u32 = 0b10;

/// EXTI maximum lines on H7.
const MAX_EXTI_LINES: usize = 16;

// ---------------------------------------------------------------------------
// Pin encoding helpers
// ---------------------------------------------------------------------------

/// Decode a u16 pin number into (port_index, pin_within_port).
/// Encoding: port in upper byte (0=A, 1=B, ...), pin in lower byte (0-15).
#[inline]
fn decode_pin(pin: u16) -> (usize, usize) {
    let port = (pin >> 8) as usize;
    let pin_num = (pin & 0xFF) as usize;
    (port, pin_num)
}

/// Encode port and pin into the u16 format.
#[inline]
pub fn encode_pin(port: u8, pin: u8) -> u16 {
    ((port as u16) << 8) | (pin as u16)
}

/// Get GPIO port base address for a given port index.
#[inline]
fn port_base(port: usize) -> u32 {
    GPIO_BASE + (port as u32) * GPIO_PORT_STRIDE
}

// ---------------------------------------------------------------------------
// Interrupt callback storage
// ---------------------------------------------------------------------------

/// EXTI interrupt callbacks (one per EXTI line, shared across ports).
/// EXTI line N corresponds to pin N on whichever port is configured.
static mut EXTI_CALLBACKS: [Option<fn(u16, bool)>; MAX_EXTI_LINES] = [None; MAX_EXTI_LINES];

/// Pin numbers associated with each EXTI line (for callback dispatch).
static mut EXTI_PINS: [u16; MAX_EXTI_LINES] = [0; MAX_EXTI_LINES];

// ---------------------------------------------------------------------------
// Stm32Gpio
// ---------------------------------------------------------------------------

/// STM32H7 GPIO driver.
///
/// Manages all GPIO pins across ports A-K. Pin numbering uses the
/// (port << 8 | pin) encoding matching the boardcfg format.
pub struct Stm32Gpio {
    /// Track which pins are configured as outputs (for toggle).
    output_mask: [u16; NUM_PORTS],
}

impl Stm32Gpio {
    /// Create a new GPIO driver. Does not configure any pins.
    pub fn new() -> Self {
        Self {
            output_mask: [0; NUM_PORTS],
        }
    }

    /// Enable the GPIO port clock in RCC. Must be called before any pin access.
    pub fn enable_port_clock(port: usize) {
        if port >= NUM_PORTS {
            return;
        }
        // TODO: actual register access
        // RCC_AHB4ENR |= (1 << port)
        //
        // unsafe {
        //     let rcc = &*stm32h7xx_hal::pac::RCC::ptr();
        //     rcc.ahb4enr.modify(|r, w| w.bits(r.bits() | (1 << port)));
        // }
    }

    /// Enable clocks for all GPIO ports (called once at boot).
    pub fn enable_all_port_clocks() {
        for port in 0..NUM_PORTS {
            Self::enable_port_clock(port);
        }
    }

    /// Configure a pin's alternate function (AF0-AF15).
    /// Used during bus initialization (SPI SCK, I2C SDA/SCL, UART TX/RX, etc.).
    fn set_alternate_function(port: usize, pin: usize, af: u8) {
        if port >= NUM_PORTS || pin >= PINS_PER_PORT || af > 15 {
            return;
        }

        // TODO: actual register access
        // AFR[pin/8] — AFRL for pins 0-7, AFRH for pins 8-15
        //
        // let _base = port_base(port);
        // unsafe {
        //     let gpio = gpio_port(port);
        //     if pin < 8 {
        //         let shift = pin * 4;
        //         let mask = 0xF << shift;
        //         (*gpio).afrl.modify(|r, w| w.bits((r.bits() & !mask) | ((af as u32) << shift)));
        //     } else {
        //         let shift = (pin - 8) * 4;
        //         let mask = 0xF << shift;
        //         (*gpio).afrh.modify(|r, w| w.bits((r.bits() & !mask) | ((af as u32) << shift)));
        //     }
        // }
        let _ = (port, pin, af);
    }
}

impl GpioPin for Stm32Gpio {
    fn set_mode(&mut self, pin: u16, mode: PinMode) {
        let (port, pin_num) = decode_pin(pin);
        if port >= NUM_PORTS || pin_num >= PINS_PER_PORT {
            return;
        }

        let _base = port_base(port);

        // TODO: actual register access
        //
        // unsafe {
        //     let gpio = gpio_port(port);
        //
        //     // MODER: 2 bits per pin
        //     let moder_shift = pin_num * 2;
        //     let moder_mask = 0b11 << moder_shift;
        //
        //     // OTYPER: 1 bit per pin
        //     let otyper_mask = 1 << pin_num;
        //
        //     // PUPDR: 2 bits per pin
        //     let pupdr_shift = pin_num * 2;
        //     let pupdr_mask = 0b11 << pupdr_shift;
        //
        match mode {
            PinMode::Input => {
                // MODER = input, PUPDR = none
                // (*gpio).moder.modify(|r, w| w.bits((r.bits() & !moder_mask) | (MODER_INPUT << moder_shift)));
                // (*gpio).pupdr.modify(|r, w| w.bits((r.bits() & !pupdr_mask) | (PUPDR_NONE << pupdr_shift)));
                self.output_mask[port] &= !(1 << pin_num);
            }
            PinMode::InputPullUp => {
                // MODER = input, PUPDR = pull-up
                self.output_mask[port] &= !(1 << pin_num);
            }
            PinMode::InputPullDown => {
                // MODER = input, PUPDR = pull-down
                self.output_mask[port] &= !(1 << pin_num);
            }
            PinMode::Output => {
                // MODER = output, OTYPER = push-pull
                self.output_mask[port] |= 1 << pin_num;
            }
            PinMode::OutputOpenDrain => {
                // MODER = output, OTYPER = open-drain
                self.output_mask[port] |= 1 << pin_num;
            }
            PinMode::Alternate(af) => {
                // MODER = alternate function
                Self::set_alternate_function(port, pin_num, af);
                self.output_mask[port] &= !(1 << pin_num);
            }
        }
        //     }
    }

    fn read(&self, pin: u16) -> bool {
        let (port, pin_num) = decode_pin(pin);
        if port >= NUM_PORTS || pin_num >= PINS_PER_PORT {
            return false;
        }

        // TODO: actual register access — read IDR
        // unsafe {
        //     let gpio = gpio_port(port);
        //     ((*gpio).idr.read().bits() >> pin_num) & 1 == 1
        // }
        false
    }

    fn write(&mut self, pin: u16, value: bool) {
        let (port, pin_num) = decode_pin(pin);
        if port >= NUM_PORTS || pin_num >= PINS_PER_PORT {
            return;
        }

        // TODO: actual register access — write BSRR (atomic set/reset)
        // BSRR: bits 0-15 = set, bits 16-31 = reset
        //
        // unsafe {
        //     let gpio = gpio_port(port);
        //     if value {
        //         (*gpio).bsrr.write(|w| w.bits(1 << pin_num));
        //     } else {
        //         (*gpio).bsrr.write(|w| w.bits(1 << (pin_num + 16)));
        //     }
        // }
        let _ = value;
    }

    fn toggle(&mut self, pin: u16) {
        let (port, pin_num) = decode_pin(pin);
        if port >= NUM_PORTS || pin_num >= PINS_PER_PORT {
            return;
        }

        // TODO: actual register access — read ODR, XOR, write BSRR
        // unsafe {
        //     let gpio = gpio_port(port);
        //     let odr = (*gpio).odr.read().bits();
        //     let new_val = odr ^ (1 << pin_num);
        //     if (new_val >> pin_num) & 1 == 1 {
        //         (*gpio).bsrr.write(|w| w.bits(1 << pin_num));
        //     } else {
        //         (*gpio).bsrr.write(|w| w.bits(1 << (pin_num + 16)));
        //     }
        // }
    }

    fn attach_interrupt(&mut self, pin: u16, edge: Edge, callback: fn(u16, bool)) -> bool {
        let (port, pin_num) = decode_pin(pin);
        if port >= NUM_PORTS || pin_num >= PINS_PER_PORT {
            return false;
        }

        // EXTI line = pin number (0-15). Only one port per EXTI line.
        // TODO: actual register access
        //
        // 1. Configure SYSCFG_EXTICRx to select this port for the EXTI line
        //    SYSCFG.exticr[pin_num / 4] — set 4-bit field to port index
        //
        // 2. Configure EXTI trigger edge
        //    match edge {
        //        Edge::Rising  => EXTI.rtsr1 |= (1 << pin_num),
        //        Edge::Falling => EXTI.ftsr1 |= (1 << pin_num),
        //        Edge::Both    => { EXTI.rtsr1 |= (1 << pin_num); EXTI.ftsr1 |= (1 << pin_num); }
        //    }
        //
        // 3. Unmask the EXTI line
        //    EXTI.imr1 |= (1 << pin_num);
        //
        // 4. Enable the EXTI IRQ in NVIC
        //    NVIC::unmask(exti_irqn(pin_num));

        // Store callback for ISR dispatch.
        unsafe {
            EXTI_CALLBACKS[pin_num] = Some(callback);
            EXTI_PINS[pin_num] = pin;
        }

        let _ = edge;
        true
    }

    fn detach_interrupt(&mut self, pin: u16) {
        let (_port, pin_num) = decode_pin(pin);
        if pin_num >= PINS_PER_PORT {
            return;
        }

        // TODO: actual register access
        // 1. Mask the EXTI line: EXTI.imr1 &= !(1 << pin_num)
        // 2. Clear pending: EXTI.pr1 = (1 << pin_num)

        unsafe {
            EXTI_CALLBACKS[pin_num] = None;
        }
    }

    fn pin_valid(&self, pin: u16) -> bool {
        let (port, pin_num) = decode_pin(pin);
        port < NUM_PORTS && pin_num < PINS_PER_PORT
    }
}

// ---------------------------------------------------------------------------
// EXTI interrupt dispatch (called from actual EXTI ISR handlers)
// ---------------------------------------------------------------------------

/// Dispatch an EXTI interrupt for the given line.
/// Called from the actual EXTI ISR (EXTI0, EXTI1, ..., EXTI15_10).
///
/// # Safety
/// Must only be called from an interrupt context with the correct line number.
pub unsafe fn exti_dispatch(line: usize) {
    if line >= MAX_EXTI_LINES {
        return;
    }
    if let Some(cb) = EXTI_CALLBACKS[line] {
        let pin = EXTI_PINS[line];
        // Read current pin state for the callback.
        // TODO: actual register access — read IDR for the stored pin
        let state = false;
        cb(pin, state);
    }
}
