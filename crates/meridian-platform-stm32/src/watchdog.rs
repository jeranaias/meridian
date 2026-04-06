//! Hardware watchdog (IWDG) for STM32H743.
//!
//! The Independent Watchdog (IWDG) provides hardware reset if the firmware
//! locks up. It runs on the LSI oscillator (~32 kHz), independent of the
//! main clock -- so it works even if the PLL crashes.
//!
//! # Timeout Configuration
//!
//! - Normal operation: 2s timeout, patted from main loop after each loop()
//! - Stall >2s: watchdog fires, motor outputs go to minimum (failsafe)
//! - Stall >3s: full hardware reset (IWDG reset)
//!
//! ArduPilot uses a two-stage approach:
//! 1. Software monitor thread (priority 183) detects stalls at 500ms
//!    and attempts mutex recovery
//! 2. Hardware IWDG at 2-3s is the last resort
//!
//! # Persistent Data Across Watchdog Reset
//!
//! The H743 backup SRAM (4 KB at 0x3880_0000) retains data across resets
//! when VBAT is powered. The monitor thread saves 16 bytes of persistent
//! data every 100ms. On watchdog reset, the boot sequence reads this back
//! to determine the cause and log a crash dump.
//!
//! # Boot Sequence Integration
//!
//! Early watchdog init (HAL_EARLY_WATCHDOG_INIT) starts the IWDG before
//! any peripheral init -- protecting against boot-time lockups.
//! Production watchdog is armed after setup() completes.
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/Scheduler.cpp` (watchdog code),
//!         `hwdef/common/stm32_util.c`

// ---------------------------------------------------------------------------
// IWDG constants
// ---------------------------------------------------------------------------

/// IWDG key register values.
const IWDG_KEY_ENABLE: u16 = 0xCCCC;
const IWDG_KEY_WRITE_ACCESS: u16 = 0x5555;
const IWDG_KEY_RELOAD: u16 = 0xAAAA;

/// LSI oscillator frequency (Hz) -- approximate, varies with temperature.
const LSI_FREQ_HZ: u32 = 32_000;

/// IWDG prescaler value (64 = register value 4).
/// With prescaler 64 and LSI 32 kHz: counter clock = 500 Hz.
const IWDG_PRESCALER: u32 = 64;
const IWDG_PRESCALER_REG: u8 = 4; // PR register: 0=/4, 1=/8, 2=/16, 3=/32, 4=/64, 5=/128, 6=/256

/// Watchdog timeout in milliseconds.
const WATCHDOG_TIMEOUT_MS: u32 = 2000;

/// Reload value for the configured timeout.
/// reload = timeout_ms * (LSI / prescaler) / 1000
/// = 2000 * (32000 / 64) / 1000 = 2000 * 500 / 1000 = 1000
const IWDG_RELOAD: u16 = (WATCHDOG_TIMEOUT_MS * (LSI_FREQ_HZ / IWDG_PRESCALER) / 1000) as u16;

/// Backup SRAM base address (4 KB, retained across resets with VBAT).
#[allow(dead_code)]
const BACKUP_SRAM_BASE: u32 = 0x3880_0000;

/// Size of persistent data saved across watchdog resets (bytes).
const PERSISTENT_DATA_SIZE: usize = 16;

/// Magic value to validate persistent data.
#[allow(dead_code)]
const PERSISTENT_MAGIC: u32 = 0x4D454449; // "MEDI"

// ---------------------------------------------------------------------------
// Watchdog state
// ---------------------------------------------------------------------------

/// Hardware watchdog driver.
///
/// Once started, the IWDG cannot be stopped (hardware limitation).
/// It must be fed by calling `pat()` before the timeout expires.
pub struct Watchdog {
    /// Whether the watchdog has been started.
    started: bool,
    /// Last time the watchdog was patted (DWT cycle count).
    last_pat_cycles: u32,
    /// Whether a watchdog reset was detected at boot.
    was_watchdog_reset: bool,
}

impl Watchdog {
    /// Create a new watchdog driver (not yet started).
    pub const fn new() -> Self {
        Self {
            started: false,
            last_pat_cycles: 0,
            was_watchdog_reset: false,
        }
    }

    /// Check if the last reset was caused by the watchdog.
    ///
    /// Reads the RCC_RSR register to determine reset cause.
    /// Must be called early in boot before the flags are cleared.
    pub fn check_reset_cause(&mut self) {
        // TODO: actual register access
        //
        // unsafe {
        //     let rcc = &*stm32h7xx_hal::pac::RCC::ptr();
        //     let rsr = rcc.rsr.read().bits();
        //
        //     // IWDG1RSTF: bit 26 in RCC_RSR
        //     self.was_watchdog_reset = (rsr >> 26) & 1 == 1;
        //
        //     // Clear reset flags (write 1 to RMVF in RCC_RSR)
        //     rcc.rsr.modify(|_, w| w.rmvf().set_bit());
        // }
    }

    /// Whether the last reset was a watchdog reset.
    pub fn was_watchdog_reset(&self) -> bool {
        self.was_watchdog_reset
    }

    /// Start the hardware watchdog with the configured timeout.
    ///
    /// **WARNING:** Once started, the IWDG cannot be stopped.
    /// It must be patted before the timeout or the MCU resets.
    ///
    /// ArduPilot calls this:
    /// - Early at boot if HAL_EARLY_WATCHDOG_INIT is defined
    /// - After setup() completes for production builds
    pub fn start(&mut self) {
        if self.started {
            return;
        }

        // IWDG1 register base on STM32H743: 0x5800_4800
        // KR at +0x00, PR at +0x04, RLR at +0x08, SR at +0x0C
        #[cfg(target_arch = "arm")]
        unsafe {
            const IWDG1_BASE: usize = 0x5800_4800;
            let kr  = IWDG1_BASE as *mut u32;         // Key register
            let pr  = (IWDG1_BASE + 0x04) as *mut u32; // Prescaler register
            let rlr = (IWDG1_BASE + 0x08) as *mut u32; // Reload register
            let sr  = (IWDG1_BASE + 0x0C) as *const u32; // Status register

            // Enable IWDG
            core::ptr::write_volatile(kr, IWDG_KEY_ENABLE as u32);

            // Enable write access to PR and RLR
            core::ptr::write_volatile(kr, IWDG_KEY_WRITE_ACCESS as u32);

            // Set prescaler (64)
            core::ptr::write_volatile(pr, IWDG_PRESCALER_REG as u32);

            // Set reload value
            core::ptr::write_volatile(rlr, IWDG_RELOAD as u32);

            // Wait for PVU and RVU bits in SR to clear
            while core::ptr::read_volatile(sr) & 0x03 != 0 {
                core::arch::asm!("nop");
            }

            // Reload the counter (first pat)
            core::ptr::write_volatile(kr, IWDG_KEY_RELOAD as u32);
        }

        // On non-ARM targets (test/SITL), the hardware registers don't exist.
        // The watchdog is a no-op but the API contract is maintained.
        #[cfg(not(target_arch = "arm"))]
        {
            let _ = (IWDG_KEY_ENABLE, IWDG_KEY_WRITE_ACCESS, IWDG_KEY_RELOAD);
            let _ = (IWDG_PRESCALER_REG, IWDG_RELOAD);
        }

        self.started = true;
    }

    /// Pat (feed) the watchdog to prevent reset.
    ///
    /// Called from the main loop after each `loop()` iteration.
    /// Also patted by the timer thread during startup (before `_initialized`).
    ///
    /// This reloads the IWDG counter back to IWDG_RELOAD.
    #[inline]
    pub fn pat(&mut self) {
        if !self.started {
            return;
        }

        // Reload the IWDG counter via the KR register.
        #[cfg(target_arch = "arm")]
        unsafe {
            const IWDG1_KR: usize = 0x5800_4800;
            core::ptr::write_volatile(IWDG1_KR as *mut u32, IWDG_KEY_RELOAD as u32);
        }

        // Record pat time for stall detection by the monitor.
        self.last_pat_cycles = crate::clock::read_cycle_count();
    }

    /// Check if the main loop appears stalled.
    ///
    /// Called by the monitor task (priority 5, 10 Hz) to detect deadlocks.
    /// ArduPilot's monitor thread detects stalls at 500ms and attempts
    /// mutex recovery before the hardware watchdog fires at 2s.
    ///
    /// Returns the time since last pat in milliseconds.
    pub fn ms_since_last_pat(&self) -> u32 {
        if !self.started {
            return 0;
        }
        let now = crate::clock::read_cycle_count();
        let elapsed = now.wrapping_sub(self.last_pat_cycles);
        crate::clock::cycles_to_ms(elapsed)
    }

    // -----------------------------------------------------------------------
    // Persistent data across watchdog resets
    // -----------------------------------------------------------------------

    /// Save persistent data to backup SRAM.
    ///
    /// Called by the monitor thread every 100ms. The data survives
    /// watchdog resets (retained by VBAT power).
    ///
    /// Layout:
    ///   [0..3]   magic: PERSISTENT_MAGIC
    ///   [4..19]  user data (16 bytes)
    pub fn save_persistent(&self, data: &[u8]) {
        if data.len() > PERSISTENT_DATA_SIZE {
            return;
        }

        // Enable backup SRAM access:
        // 1. Set DBP bit in PWR_CR1 (disable backup domain write protection)
        // 2. Enable backup SRAM clock in RCC_AHB4ENR

        // TODO: actual register access
        //
        // unsafe {
        //     let pwr = &*stm32h7xx_hal::pac::PWR::ptr();
        //     pwr.cr1.modify(|_, w| w.dbp().set_bit());
        //
        //     let rcc = &*stm32h7xx_hal::pac::RCC::ptr();
        //     rcc.ahb4enr.modify(|_, w| w.bkpsramen().set_bit());
        //
        //     // Write magic + data to backup SRAM
        //     let base = BACKUP_SRAM_BASE as *mut u8;
        //     let magic_ptr = base as *mut u32;
        //     core::ptr::write_volatile(magic_ptr, PERSISTENT_MAGIC);
        //
        //     let data_ptr = base.add(4);
        //     for (i, &byte) in data.iter().enumerate() {
        //         core::ptr::write_volatile(data_ptr.add(i), byte);
        //     }
        // }
        let _ = data;
    }

    /// Load persistent data from backup SRAM after a watchdog reset.
    ///
    /// Returns `true` if valid persistent data was found.
    pub fn load_persistent(&self, data: &mut [u8]) -> bool {
        if data.len() > PERSISTENT_DATA_SIZE {
            return false;
        }

        // TODO: actual register access
        //
        // unsafe {
        //     let base = BACKUP_SRAM_BASE as *const u8;
        //     let magic_ptr = base as *const u32;
        //     let magic = core::ptr::read_volatile(magic_ptr);
        //
        //     if magic != PERSISTENT_MAGIC {
        //         return false;
        //     }
        //
        //     let data_ptr = base.add(4);
        //     for (i, byte) in data.iter_mut().enumerate() {
        //         *byte = core::ptr::read_volatile(data_ptr.add(i));
        //     }
        //     true
        // }
        false
    }

    /// Clear persistent data (called after successful boot to prevent
    /// stale data from confusing the next watchdog reset).
    pub fn clear_persistent(&self) {
        // TODO: actual register access
        //
        // unsafe {
        //     let base = BACKUP_SRAM_BASE as *mut u32;
        //     core::ptr::write_volatile(base, 0);
        // }
    }
}
