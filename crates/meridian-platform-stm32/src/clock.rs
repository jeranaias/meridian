//! Clock initialization for STM32H743.
//!
//! Configures the clock tree to match ArduPilot's ChibiOS mcuconf.h:
//!   HSE = 8 MHz crystal
//!   PLL1: DIVM=1, DIVN=100, DIVP=2 -> SYS_CK = 400 MHz
//!   PLL2: ~200 MHz for SPI1/SPI4 clocks (IMU buses)
//!   PLL3: UART peripheral clocks
//!
//! The stm32h7xx-hal crate's `cfgr` builder handles PLL configuration,
//! power supply, and flash wait-state calculation internally.

use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rcc;

/// Fully configured clock outputs after init.
pub struct Clocks {
    /// Core reference to configured clocks for peripheral init.
    pub ccdr: rcc::Ccdr,
}

/// Initialize the H743 clock tree.
///
/// This must be the first thing called after reset, before any peripheral init.
///
/// # PLL Configuration
///
/// - PLL1: 8 MHz HSE / DIVM=1 * DIVN=100 / DIVP=2 = 400 MHz sysclk
/// - PLL2: Configured by HAL for SPI peripheral clocks (~200 MHz)
/// - PLL3: Configured by HAL for UART peripheral clocks
///
/// # Power Supply
///
/// Uses LDO regulator (standard for most FC boards). Switch to SMPS
/// if the board uses a switching regulator (check schematic).
pub fn init_clocks(
    pwr: stm32h7xx_hal::pac::PWR,
    rcc: stm32h7xx_hal::pac::RCC,
    syscfg: &stm32h7xx_hal::pac::SYSCFG,
) -> Clocks {
    // Configure power supply -- LDO mode is the safe default for most FC boards.
    let pwrcfg = pwr.constrain().freeze();

    // Configure the clock tree.
    //
    // stm32h7xx-hal's rcc builder calculates PLL divisors, flash wait states,
    // and bus prescalers automatically from the target frequencies.
    let ccdr = rcc
        .constrain()
        .use_hse(8.MHz())       // 8 MHz external crystal (Matek H743)
        .sys_ck(400.MHz())      // PLL1 -> 400 MHz system clock
        .hclk(200.MHz())        // AHB bus = sysclk/2 = 200 MHz
        .pclk1(100.MHz())       // APB1 = 100 MHz (timers get 2x = 200 MHz)
        .pclk2(100.MHz())       // APB2 = 100 MHz
        .pclk3(100.MHz())       // APB3 = 100 MHz
        .pclk4(100.MHz())       // APB4 = 100 MHz
        .pll2_p_ck(200.MHz())   // PLL2_P for SPI1/SPI4 clocks (IMU buses)
        .pll2_q_ck(200.MHz())   // PLL2_Q available for peripherals
        .pll3_p_ck(100.MHz())   // PLL3_P for UART clocks
        .pll3_q_ck(100.MHz())   // PLL3_Q available for peripherals
        .freeze(pwrcfg, syscfg);

    Clocks { ccdr }
}

/// Initialize the DWT cycle counter for microsecond-resolution timing.
///
/// The DWT CYCCNT register counts core clock cycles (400 MHz = 2.5 ns/tick).
/// This provides sub-microsecond timing without consuming a hardware timer.
pub fn init_cycle_counter(mut core: cortex_m::Peripherals) {
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();
}

/// Read the DWT cycle counter (wraps at ~10.7 seconds at 400 MHz).
#[inline(always)]
pub fn read_cycle_count() -> u32 {
    cortex_m::peripheral::DWT::cycle_count()
}

/// Convert cycle count to microseconds at 400 MHz.
#[inline(always)]
pub fn cycles_to_us(cycles: u32) -> u32 {
    cycles / 400
}

/// Convert cycle count to milliseconds at 400 MHz.
#[inline(always)]
pub fn cycles_to_ms(cycles: u32) -> u32 {
    cycles / 400_000
}
