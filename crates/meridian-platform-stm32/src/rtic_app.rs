//! RTIC application framework mapping ChibiOS threads to hardware tasks.
//!
//! ArduPilot's ChibiOS port uses 7 RTOS threads with fixed priorities.
//! RTIC replaces threads with interrupt-driven tasks using Cortex-M NVIC
//! priority levels. Priority inversion is impossible because RTIC uses
//! compile-time priority ceiling analysis.
//!
//! # Thread-to-Task Mapping
//!
//! | ChibiOS Thread | Priority | RTIC Task      | RTIC Priority | Trigger       |
//! |----------------|----------|----------------|---------------|---------------|
//! | monitor        | 183      | monitor        | 5             | TIM7 @ 10 Hz  |
//! | timer          | 181      | timer_1khz     | 4             | TIM2 @ 1 kHz  |
//! | rcout          | 181      | (in timer_1khz)| 4             | event-driven  |
//! | main (daemon)  | 180      | main_loop      | 3             | TIM6 @ 400 Hz |
//! | rcin           | 177      | rcin           | 2             | UART IDLE IRQ |
//! | io             | 58       | slow_loop      | 1             | TIM3 @ 50 Hz  |
//! | storage        | 59       | (in slow_loop) | 1             | with slow_loop|
//!
//! # Dispatcher Interrupts
//!
//! RTIC needs "free" interrupts as software task dispatchers. We use
//! EXTI0, EXTI1, EXTI2 which are not connected to any GPIO pin on the
//! MatekH743 board (all EXTI-capable pins use other EXTI lines).
//!
//! # Shared Resources
//!
//! Resources shared between tasks use RTIC's lock protocol (priority ceiling).
//! This is zero-overhead on single-core Cortex-M: accessing a shared resource
//! temporarily raises the BASEPRI to the ceiling priority, preventing
//! preemption by any task that also uses that resource.
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/Scheduler.cpp`, ChibiOS thread model

use crate::clock;

use meridian_hal::scheduler::{Scheduler, TimerProc, IoProc};

// ---------------------------------------------------------------------------
// Timer proc / IO proc storage
// ---------------------------------------------------------------------------

/// Maximum registered timer callbacks (matches ArduPilot's limit).
const MAX_TIMER_PROCS: usize = 8;

/// Maximum registered IO callbacks.
const MAX_IO_PROCS: usize = 8;

// ---------------------------------------------------------------------------
// Stm32Scheduler -- implements meridian_hal::Scheduler
// ---------------------------------------------------------------------------

/// STM32/RTIC scheduler implementation.
///
/// Instead of creating OS threads, the RTIC framework handles task scheduling
/// via hardware interrupts. This scheduler manages the timer/IO callback
/// registrations and provides timing functions.
///
/// Timer callbacks run at 1 kHz (in the TIM2 interrupt, priority 4).
/// IO callbacks run at 50 Hz (in the TIM3 interrupt, priority 1).
pub struct Stm32Scheduler {
    /// Registered timer process callbacks (1 kHz, high priority).
    timer_procs: [Option<TimerProc>; MAX_TIMER_PROCS],
    num_timer_procs: u8,

    /// Registered IO process callbacks (50 Hz, low priority).
    io_procs: [Option<IoProc>; MAX_IO_PROCS],
    num_io_procs: u8,

    /// Whether the system has completed initialization.
    system_initialized: bool,

    /// Boot time in DWT cycles (for millis/micros calculation).
    boot_cycles: u32,

    /// Overflow counter for 64-bit microseconds (DWT wraps at ~10.7s @ 400 MHz).
    overflow_count: u32,
    last_cycle_count: u32,
}

impl Stm32Scheduler {
    pub const fn new() -> Self {
        Self {
            timer_procs: [None; MAX_TIMER_PROCS],
            num_timer_procs: 0,
            io_procs: [None; MAX_IO_PROCS],
            num_io_procs: 0,
            system_initialized: false,
            boot_cycles: 0,
            overflow_count: 0,
            last_cycle_count: 0,
        }
    }

    /// Run all registered timer callbacks.
    /// Called from the 1 kHz TIM2 interrupt handler.
    pub fn run_timer_procs(&self) {
        for i in 0..self.num_timer_procs as usize {
            if let Some(proc) = self.timer_procs[i] {
                proc();
            }
        }
    }

    /// Run all registered IO callbacks.
    /// Called from the 50 Hz slow loop.
    pub fn run_io_procs(&self) {
        for i in 0..self.num_io_procs as usize {
            if let Some(proc) = self.io_procs[i] {
                proc();
            }
        }
    }

    /// Update the overflow counter for 64-bit microsecond tracking.
    /// Called from the 1 kHz timer to detect DWT counter wraps.
    pub fn update_overflow(&mut self) {
        let now = clock::read_cycle_count();
        if now < self.last_cycle_count {
            // DWT counter wrapped
            self.overflow_count += 1;
        }
        self.last_cycle_count = now;
    }
}

impl Scheduler for Stm32Scheduler {
    fn init(&mut self) {
        self.boot_cycles = clock::read_cycle_count();

        // Configure hardware timers for task triggering:
        //
        // TIM2: 1 kHz interrupt (timer procs, IMU sampling)
        //   APB1 timer clock = 200 MHz
        //   PSC = 199, ARR = 999 -> 200MHz/200/1000 = 1 kHz
        //
        // TIM6: 400 Hz interrupt (main flight control loop)
        //   PSC = 199, ARR = 2499 -> 200MHz/200/2500 = 400 Hz
        //
        // TIM3: 50 Hz interrupt (slow sensors, telemetry)
        //   PSC = 199, ARR = 19999 -> 200MHz/200/20000 = 50 Hz
        //
        // TIM7: 10 Hz interrupt (monitor / watchdog check)
        //   PSC = 199, ARR = 99999 -> 200MHz/200/100000 = 10 Hz
        //
        // TODO: actual register access to configure and start timers.
        // Each timer interrupt is configured in the NVIC with the
        // corresponding RTIC priority level.
    }

    fn register_timer_process(&mut self, proc: TimerProc) -> bool {
        if self.num_timer_procs as usize >= MAX_TIMER_PROCS {
            return false;
        }
        self.timer_procs[self.num_timer_procs as usize] = Some(proc);
        self.num_timer_procs += 1;
        true
    }

    fn register_io_process(&mut self, proc: IoProc) -> bool {
        if self.num_io_procs as usize >= MAX_IO_PROCS {
            return false;
        }
        self.io_procs[self.num_io_procs as usize] = Some(proc);
        self.num_io_procs += 1;
        true
    }

    fn delay_microseconds(&self, us: u32) {
        // Busy-wait using the DWT cycle counter.
        // At 400 MHz, 1 us = 400 cycles.
        let target_cycles = us * 400;
        let start = clock::read_cycle_count();
        while clock::read_cycle_count().wrapping_sub(start) < target_cycles {
            cortex_m::asm::nop();
        }
    }

    fn delay_microseconds_boost(&self, us: u32) {
        // Temporarily raise BASEPRI to priority 2 (above main loop at 3)
        // to ensure prompt wakeup after sensor DMA completes.
        //
        // In RTIC, this is achieved by the task priority system.
        // A higher-priority task preempts lower ones automatically.
        //
        // For now, just delay normally.
        self.delay_microseconds(us);
    }

    fn millis(&self) -> u32 {
        let elapsed = clock::read_cycle_count().wrapping_sub(self.boot_cycles);
        clock::cycles_to_ms(elapsed)
    }

    fn micros(&self) -> u32 {
        let elapsed = clock::read_cycle_count().wrapping_sub(self.boot_cycles);
        clock::cycles_to_us(elapsed)
    }

    fn micros64(&self) -> u64 {
        // Combine overflow counter with current cycle count for 64-bit microseconds.
        // Each overflow = 2^32 cycles = ~10.737 seconds at 400 MHz.
        let overflow = self.overflow_count as u64;
        let current = clock::read_cycle_count() as u64;
        let total_cycles = (overflow << 32) | current;
        total_cycles / 400 // 400 MHz -> microseconds
    }

    fn is_system_initialized(&self) -> bool {
        self.system_initialized
    }

    fn set_system_initialized(&mut self) {
        self.system_initialized = true;
    }

    fn in_main_thread(&self) -> bool {
        // On RTIC, "main thread" context is the idle task or the
        // main_loop software task. Check if current NVIC priority
        // matches the main loop level.
        //
        // In practice, check if we're in thread mode (not in an ISR).
        // IPSR == 0 means thread mode.
        true
    }

    fn create_thread(&mut self, _name: &str, _stack_size: u32, _priority: u8, _proc: fn()) {
        // RTIC does not support dynamic thread creation.
        // All tasks are defined at compile time in the #[rtic::app] module.
        //
        // This method exists for HAL API compatibility. On RTIC, additional
        // "threads" should be mapped to RTIC software tasks at build time.
    }

    fn reboot(&self, _hold_in_bootloader: bool) -> ! {
        // Request system reset via Cortex-M AIRCR register.
        cortex_m::peripheral::SCB::sys_reset();
    }
}

// ---------------------------------------------------------------------------
// RTIC app skeleton -- documentation of the intended task layout
// ---------------------------------------------------------------------------
//
// The full RTIC app definition uses procedural macros that require
// the rtic crate and hardware target. The structure below shows the
// intended task layout. On a real build targeting thumbv7em-none-eabihf,
// this would use the #[rtic::app] attribute macro.
//
// The RTIC app is defined in the binary crate (bin/meridian-stm32) which
// depends on this platform crate. This module provides the building blocks;
// the binary crate wires them together in the RTIC app macro.
//
// Intended RTIC app structure:
//
// ```rust
// #[rtic::app(device = stm32h7xx_hal::pac, peripherals = true,
//             dispatchers = [EXTI0, EXTI1, EXTI2])]
// mod app {
//     use super::*;
//
//     #[shared]
//     struct Shared {
//         scheduler: Stm32Scheduler,
//         storage: FlashStorage,
//         watchdog: Watchdog,
//         pwm: Stm32PwmOutput,
//         adc: Stm32Adc,
//     }
//
//     #[local]
//     struct Local {
//         imu_spi: Stm32Spi,
//         i2c_buses: [Stm32I2c; 2],
//         uarts: [Stm32Uart; 8],
//         gpio: Stm32Gpio,
//     }
//
//     #[init]
//     fn init(cx: init::Context) -> (Shared, Local) {
//         // 1. Clock init (400 MHz from 8 MHz HSE)
//         let clocks = clock::init_clocks(cx.device.PWR, cx.device.RCC, &cx.device.SYSCFG);
//         clock::init_cycle_counter(cx.core);
//
//         // 2. Watchdog early init (check reset cause before clearing flags)
//         let mut watchdog = Watchdog::new();
//         watchdog.check_reset_cause();
//
//         // 3. GPIO port clocks
//         Stm32Gpio::enable_all_port_clocks();
//
//         // 4. I2C bus clear (20 SCL pulses to unstick sensors)
//         i2c::clear_all_buses(&[(1, 6), (1, 10)]); // PB6, PB10
//
//         // 5. Serial port init (USB CDC first)
//
//         // 6. ADC init
//
//         // 7. Scheduler init (configure timer interrupts)
//
//         // 8. SPI bus init, sensor probe
//
//         // 9. PWM output init from board config
//
//         // 10. Flash storage load
//         let mut storage = FlashStorage::new();
//         storage.load();
//
//         // 11. Arm production watchdog
//         watchdog.start();
//
//         // (Shared, Local)
//     }
//
//     // 1 kHz timer task (IMU sampling, like ChibiOS timer thread @ priority 181)
//     #[task(binds = TIM2, priority = 4, shared = [scheduler, adc])]
//     fn timer_1khz(mut cx: timer_1khz::Context) {
//         // Clear TIM2 update interrupt flag
//         cx.shared.scheduler.lock(|sched| {
//             sched.update_overflow();
//             sched.run_timer_procs();
//         });
//         cx.shared.adc.lock(|adc| {
//             adc.update();
//         });
//     }
//
//     // 400 Hz main loop (flight control, like ChibiOS main @ priority 180)
//     #[task(binds = TIM6_DAC, priority = 3, shared = [scheduler, pwm, watchdog])]
//     fn main_loop(mut cx: main_loop::Context) {
//         // Flight control pipeline:
//         //   mode manager -> attitude controller -> mixer -> motor output
//         cx.shared.pwm.lock(|pwm| {
//             pwm.push();
//         });
//         cx.shared.watchdog.lock(|wdg| {
//             wdg.pat();
//         });
//     }
//
//     // 50 Hz slow tasks (GPS, baro, compass, like ChibiOS IO @ priority 58)
//     #[task(binds = TIM3, priority = 1, shared = [scheduler, storage])]
//     fn slow_loop(mut cx: slow_loop::Context) {
//         cx.shared.scheduler.lock(|sched| {
//             sched.run_io_procs();
//         });
//         cx.shared.storage.lock(|storage| {
//             storage.tick();
//         });
//     }
//
//     // 10 Hz monitor (deadlock detection, watchdog state save)
//     #[task(binds = TIM7, priority = 5, shared = [watchdog])]
//     fn monitor(mut cx: monitor::Context) {
//         cx.shared.watchdog.lock(|wdg| {
//             let stall_ms = wdg.ms_since_last_pat();
//             if stall_ms > 500 {
//                 // Main loop stalled -- attempt mutex recovery.
//             }
//             wdg.save_persistent(&[0; 16]);
//         });
//     }
//
//     #[idle(shared = [watchdog])]
//     fn idle(mut cx: idle::Context) -> ! {
//         loop {
//             // Pat the hardware watchdog from idle to ensure it doesn't
//             // fire during long init or if the main loop hasn't started yet.
//             // The main_loop task (400 Hz) also pats, but idle provides
//             // coverage during boot-up before main_loop is scheduled.
//             cx.shared.watchdog.lock(|wdg| {
//                 wdg.pat();
//             });
//             // WFI: sleep until next interrupt (saves power).
//             cortex_m::asm::wfi();
//         }
//     }
// }
// ```
