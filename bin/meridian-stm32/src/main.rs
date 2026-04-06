// Meridian STM32H743 — Phase 9 Minimum Viable Firmware
//
// Build:  cargo build --target thumbv7em-none-eabihf -p meridian-stm32-bin --release
// Flash:  probe-rs run --chip STM32H743VIHx target/thumbv7em-none-eabihf/release/meridian-stm32-bin
//
// Hardware validation checklist:
//   [1] LED blinks at 2 Hz on PE3 (boot confirmation)
//   [2] ICM-42688 WHO_AM_I = 0x47 via SPI1 (PA5/PA6/PD7, CS=PC15)
//   [3] USART1 TX @ 57600 on PA9 — MNP heartbeat (COBS-framed postcard-serialized)
//       PRIMARY channel: FC -> RFD 900x radio -> ATAK plugin
//   [4] UART7 TX @ 57600 on PE8 — MAVLink v2 heartbeat (optional, for QGC/MP)
//       SECONDARY channel: only when a MAVLink GCS is connected
//   [5] TIM5 ch1-ch4 PWM at 50 Hz on PA0-PA3 (1500 us center pulse)
//   [6] TIM6 interrupt at 1 kHz toggling PA4 (oscilloscope verification)
//
// Protocol architecture:
//   Primary UART (USART1 / SERIAL2, PA9/PA10):
//     Meridian Native Protocol (MNP) — COBS-framed postcard-serialized messages.
//     See crates/meridian-comms/src/wire.rs for COBS frame format.
//     See crates/meridian-comms/src/messages.rs for MnpMessage::Heartbeat.
//     Frame: [0x00] [COBS(msg_id + seq_le + postcard_body)] [0x00]
//     FC speaks MNP directly over UART -> RFD 900x -> ATAK plugin. No bridge.
//
//   Secondary UART (UART7 / SERIAL1, PE8/PE7):
//     MAVLink v2 adapter — standard frames for QGC / Mission Planner compatibility.
//     Runs independently on a separate UART port. NOT the native protocol.
//
// Aparicio review deferred items addressed:
//   - TIM2 clash: using TIM6 for 1 kHz scheduler instead of TIM2
//   - Semaphore guard: deferred (known issue)
//   - D-cache: deferred (known issue, AXI SRAM buffers work without cache maint for MVP)

#![cfg_attr(target_arch = "arm", no_std)]
#![cfg_attr(target_arch = "arm", no_main)]

// On non-ARM hosts, compile as a regular binary with a message.
#[cfg(not(target_arch = "arm"))]
fn main() {
    println!("Meridian STM32H743 Phase 9 MVP — build for thumbv7em-none-eabihf to run on hardware");
    println!("  cargo build --target thumbv7em-none-eabihf -p meridian-stm32-bin --release");
    println!();
    println!("Primary UART (USART1/PA9):  MNP heartbeat — COBS-framed, postcard-serialized");
    println!("Secondary UART (UART7/PE8): MAVLink v2 heartbeat — optional GCS compat");
}

// ---------------------------------------------------------------------------
// Real firmware for ARM target
// ---------------------------------------------------------------------------
#[cfg(target_arch = "arm")]
mod firmware {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

    use cortex_m::peripheral::NVIC;
    use defmt_rtt as _; // global logger
    use panic_probe as _; // panic handler

    use stm32h7xx_hal::pac;
    use stm32h7xx_hal::pac::interrupt;
    use stm32h7xx_hal::prelude::*;

    // -----------------------------------------------------------------------
    // ICM-42688 constants
    // -----------------------------------------------------------------------
    /// WHO_AM_I register address (read: set bit 7)
    const ICM42688_WHO_AM_I: u8 = 0x75;
    /// Expected WHO_AM_I response
    const ICM42688_WHO_AM_I_VALUE: u8 = 0x47;

    // -----------------------------------------------------------------------
    // MNP imports (primary protocol)
    // -----------------------------------------------------------------------
    // Meridian Native Protocol: COBS-framed postcard-serialized messages.
    // The FC speaks this directly over UART -> RFD 900x -> ATAK plugin.
    use meridian_comms::messages::{Heartbeat, MnpMessage};
    use meridian_comms::wire::MAX_FRAME_SIZE;

    // -----------------------------------------------------------------------
    // MAVLink v2 heartbeat (secondary protocol, for QGC/Mission Planner)
    // -----------------------------------------------------------------------
    /// Pre-built MAVLink v2 heartbeat template (system_id=1, component_id=1)
    ///
    /// MAVLink v2 header (10 bytes) + payload (9 bytes) + CRC (2 bytes) = 21 bytes
    ///
    /// Header:
    ///   FD         STX (MAVLink v2 magic)
    ///   09         Payload length (9 bytes)
    ///   00         Incompatibility flags
    ///   00         Compatibility flags
    ///   XX         Sequence (filled at runtime)
    ///   01         System ID = 1
    ///   01         Component ID = 1 (autopilot)
    ///   00 00 00   Message ID = 0 (HEARTBEAT), 3 bytes LE
    ///
    /// Payload (9 bytes):
    ///   00 00 00 00  custom_mode = 0
    ///   02           type = 2 (quadrotor)
    ///   08           autopilot = 8 (generic)
    ///   00           base_mode = 0
    ///   00           system_status = 0 (uninit)
    ///   03           mavlink_version = 3
    ///
    /// CRC: 2 bytes (computed at send time)
    const MAVLINK_HEARTBEAT_TEMPLATE: [u8; 18] = [
        0xFD, // STX
        0x09, // payload len
        0x00, // incompat flags
        0x00, // compat flags
        0x00, // sequence (overwritten)
        0x01, // sys id
        0x01, // comp id
        0x00, 0x00, 0x00, // msg id = 0 (heartbeat) LE 24-bit
        // payload:
        0x00, 0x00, 0x00, 0x00, // custom_mode
        0x02, // type = quadrotor
        0x08, // autopilot = generic
        0x00, // base_mode
        0x00, // system_status = uninit
        // mavlink_version is last payload byte
    ];

    /// Final payload byte (mavlink_version = 3) kept separate for CRC computation clarity
    const MAVLINK_VERSION_BYTE: u8 = 0x03;

    /// CRC seed for HEARTBEAT (msg_id 0, CRC_EXTRA = 50)
    const HEARTBEAT_CRC_EXTRA: u8 = 50;

    /// MAVLink CRC (X.25/CCITT) accumulate one byte
    #[inline]
    fn crc_accumulate(byte: u8, crc: &mut u16) {
        let tmp = (byte as u16) ^ (*crc & 0xFF);
        let tmp = tmp ^ ((tmp << 4) & 0xFF);
        *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }

    /// Compute MAVLink CRC over header[1..] + payload + CRC_EXTRA
    fn mavlink_crc(header_and_payload: &[u8], extra: u8) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &b in header_and_payload {
            crc_accumulate(b, &mut crc);
        }
        crc_accumulate(extra, &mut crc);
        crc
    }

    // -----------------------------------------------------------------------
    // 1 kHz tick counter (written by TIM6 ISR, read by main)
    // -----------------------------------------------------------------------
    static TICK_COUNT: AtomicU32 = AtomicU32::new(0);
    static TIM6_TOGGLE: AtomicBool = AtomicBool::new(false);

    // -----------------------------------------------------------------------
    // Entry point
    // -----------------------------------------------------------------------
    #[cortex_m_rt::entry]
    fn main() -> ! {
        defmt::info!("Meridian Phase 9 MVP boot");

        // Take peripherals
        let cp = cortex_m::Peripherals::take().unwrap();
        let dp = pac::Peripherals::take().unwrap();

        // ===================================================================
        // [STEP 1] Clock init: 8 MHz HSE -> 400 MHz PLL1
        // ===================================================================
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            .use_hse(8.MHz())       // 8 MHz crystal on Matek H743
            .sys_ck(400.MHz())      // PLL1 -> 400 MHz system clock
            .hclk(200.MHz())        // AHB = 200 MHz
            .pclk1(100.MHz())       // APB1 = 100 MHz (timers get 2x = 200 MHz)
            .pclk2(100.MHz())       // APB2 = 100 MHz
            .pclk3(100.MHz())       // APB3 = 100 MHz
            .pclk4(100.MHz())       // APB4 = 100 MHz
            .pll2_p_ck(200.MHz())   // PLL2_P for SPI1/SPI4 clocks
            .freeze(pwrcfg, &dp.SYSCFG);

        defmt::info!("Clock: sysclk={} MHz, hclk={} MHz",
            ccdr.clocks.sys_ck().raw() / 1_000_000,
            ccdr.clocks.hclk().raw() / 1_000_000);

        // Enable DWT cycle counter for timing
        let mut cp_dcb = cp.DCB;
        let mut cp_dwt = cp.DWT;
        cp_dcb.enable_trace();
        cortex_m::peripheral::DWT::unlock();
        cp_dwt.enable_cycle_counter();

        // ===================================================================
        // [STEP 2] GPIO init: LED on PE3, CS on PC15, scope pin on PA4
        // ===================================================================

        // Split GPIO ports
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        // LED pin PE3 — push-pull output, start HIGH (LED on)
        let mut led = gpioe.pe3.into_push_pull_output();
        led.set_high();
        defmt::info!("LED on PE3: ON");

        // Scope verification pin PA4 — toggled by TIM6 ISR at 1 kHz
        let mut scope_pin = gpioa.pa4.into_push_pull_output();
        scope_pin.set_low();

        // SPI1 CS for ICM-42688 on PC15 — manual push-pull output, start HIGH (deselected)
        let mut imu_cs = gpioc.pc15.into_push_pull_output();
        imu_cs.set_high();

        // ===================================================================
        // [STEP 3] SPI1 init for ICM-42688: PA5(SCK), PA6(MISO), PD7(MOSI)
        // ===================================================================
        //
        // ICM-42688 talks SPI Mode 3 (CPOL=1, CPHA=1).
        // Low speed for initial probe: 1 MHz.
        // MatekH743 SPI1 pinout: SCK=PA5 AF5, MISO=PA6 AF5, MOSI=PD7 AF5.

        let sck  = gpioa.pa5.into_alternate::<5>();
        let miso = gpioa.pa6.into_alternate::<5>();
        let mosi = gpiod.pd7.into_alternate::<5>();

        // Configure SPI1 at 1 MHz for initial WHO_AM_I probe
        let mut spi1 = dp.SPI1.spi(
            (sck, miso, mosi),
            stm32h7xx_hal::spi::Config::new(stm32h7xx_hal::spi::MODE_3)
                .communication_mode(stm32h7xx_hal::spi::CommunicationMode::FullDuplex),
            1.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );

        defmt::info!("SPI1 configured: Mode 3, 1 MHz");

        // Read ICM-42688 WHO_AM_I register
        imu_cs.set_low();
        // Small delay after CS assert (ICM-42688 needs ~100ns setup time)
        cortex_m::asm::delay(100);

        // embedded-hal 0.2 SPI transfer: tx and rx happen in-place on the same buffer.
        // Write [reg|0x80, 0x00], read back response in same buffer.
        let mut buf: [u8; 2] = [ICM42688_WHO_AM_I | 0x80, 0x00];

        use stm32h7xx_hal::hal::blocking::spi::Transfer;
        match spi1.transfer(&mut buf) {
            Ok(result) => {
                imu_cs.set_high();
                let who = result[1];
                if who == ICM42688_WHO_AM_I_VALUE {
                    defmt::info!("ICM-42688 WHO_AM_I = 0x{:02x} -- CORRECT", who);
                } else {
                    defmt::warn!("SPI1 WHO_AM_I = 0x{:02x} (expected 0x47 for ICM42688; 0x12=ICM20602, 0x68=MPU6000)", who);
                }
            }
            Err(_) => {
                imu_cs.set_high();
                defmt::error!("SPI1 transfer failed");
            }
        }

        // ===================================================================
        // [STEP 4a] PRIMARY UART: USART1 TX @ 57600 on PA9 — MNP
        // ===================================================================
        //
        // MatekH743 SERIAL2 = USART1 (PA9 TX, PA10 RX). AF7.
        // Primary communication: Meridian Native Protocol (MNP).
        // COBS-framed postcard-serialized messages.
        // FC -> USART1 -> RFD 900x radio -> ATAK plugin.
        // No MAVLink bridge needed on this port.

        let tx_pin = gpioa.pa9.into_alternate::<7>();
        let rx_pin = gpioa.pa10.into_alternate::<7>();

        let serial_config = stm32h7xx_hal::serial::config::Config::new(57_600.bps());
        let mut usart1 = dp
            .USART1
            .serial(
                (tx_pin, rx_pin),
                serial_config,
                ccdr.peripheral.USART1,
                &ccdr.clocks,
            )
            .unwrap();

        defmt::info!("USART1 (PRIMARY): 57600 baud on PA9/PA10 — MNP protocol");

        // ===================================================================
        // [STEP 4b] SECONDARY UART: UART7 TX @ 57600 on PE8 — MAVLink
        // ===================================================================
        //
        // MatekH743 SERIAL1 = UART7 (PE8 TX, PE7 RX). AF7.
        // Secondary communication: MAVLink v2 adapter.
        // Standard MAVLink frames for QGC / Mission Planner compatibility.
        // This is NOT the native protocol — MNP on USART1 is primary.
        // Only active when a MAVLink GCS is connected.

        let uart7_tx = gpioe.pe8.into_alternate::<7>();
        let uart7_rx = gpioe.pe7.into_alternate::<7>();

        let uart7_config = stm32h7xx_hal::serial::config::Config::new(57_600.bps());
        let mut uart7 = dp
            .UART7
            .serial(
                (uart7_tx, uart7_rx),
                uart7_config,
                ccdr.peripheral.UART7,
                &ccdr.clocks,
            )
            .unwrap();

        defmt::info!("UART7 (SECONDARY): 57600 baud on PE8/PE7 — MAVLink v2 adapter");

        // ===================================================================
        // [STEP 5] TIM5 PWM at 50 Hz on PA0-PA3 (motor outputs 9-12)
        // ===================================================================
        //
        // MatekH743 M9-M12 = TIM5 ch1-ch4 on PA0-PA3 AF2.
        // 50 Hz PWM, 1500 us center pulse (safe for servo test without props).

        let pa0 = gpioa.pa0.into_alternate::<2>();
        let pa1 = gpioa.pa1.into_alternate::<2>();
        let pa2 = gpioa.pa2.into_alternate::<2>();
        let pa3 = gpioa.pa3.into_alternate::<2>();

        let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = dp.TIM5.pwm(
            (pa0, pa1, pa2, pa3),
            50.Hz(),
            ccdr.peripheral.TIM5,
            &ccdr.clocks,
        );

        // Set all channels to 1500 us (center/neutral)
        // At 50 Hz, period = 20,000 us. max_duty corresponds to full period.
        let max1 = pwm1.get_max_duty();
        let max2 = pwm2.get_max_duty();
        let max3 = pwm3.get_max_duty();
        let max4 = pwm4.get_max_duty();

        // 1500 us / 20000 us = 7.5% duty
        pwm1.set_duty(max1 * 1500 / 20000);
        pwm2.set_duty(max2 * 1500 / 20000);
        pwm3.set_duty(max3 * 1500 / 20000);
        pwm4.set_duty(max4 * 1500 / 20000);

        pwm1.enable();
        pwm2.enable();
        pwm3.enable();
        pwm4.enable();

        defmt::info!("TIM5 PWM: 50 Hz, 1500 us on PA0-PA3 (max_duty={})", max1);

        // ===================================================================
        // [STEP 6] TIM6 basic timer at 1 kHz interrupt
        // ===================================================================
        //
        // Using TIM6 instead of TIM2 (Aparicio review: TIM2 clash with buzzer PWM).
        // TIM6 is a basic timer — no output channels, just UIE interrupt.
        // APB1 timer clock = 200 MHz.
        // PSC = 199 -> counter at 1 MHz
        // ARR = 999 -> overflow at 1 kHz
        //
        // The ISR toggles PA4 for oscilloscope verification.

        // Enable TIM6 clock
        // Safety: single-threaded at this point, no other code touching RCC
        unsafe {
            let rcc_ptr = &*pac::RCC::ptr();
            rcc_ptr.apb1lenr.modify(|_, w| w.tim6en().set_bit());
            // Read back to ensure the clock is enabled (bus sync)
            let _ = rcc_ptr.apb1lenr.read().tim6en().bit();
        }

        // Configure TIM6
        unsafe {
            let tim6 = &*pac::TIM6::ptr();
            // Stop timer during config
            tim6.cr1.modify(|_, w| w.cen().clear_bit());
            // PSC = 199 -> 200 MHz / 200 = 1 MHz counter
            tim6.psc.write(|w| w.psc().bits(199));
            // ARR = 999 -> 1 MHz / 1000 = 1 kHz overflow
            tim6.arr.write(|w| w.arr().bits(999));
            // Enable update interrupt
            tim6.dier.modify(|_, w| w.uie().set_bit());
            // Generate update event to load PSC/ARR shadow registers
            tim6.egr.write(|w| w.ug().set_bit());
            // Clear the update interrupt flag that the UG event just set
            tim6.sr.modify(|_, w| w.uif().clear_bit());
            // Start timer
            tim6.cr1.modify(|_, w| w.cen().set_bit());
        }

        // Enable TIM6 interrupt in NVIC
        unsafe {
            NVIC::unmask(pac::Interrupt::TIM6_DAC);
        }

        defmt::info!("TIM6: 1 kHz interrupt enabled, toggling PA4");

        // ===================================================================
        // Main loop: LED blink + MNP heartbeat + MAVLink heartbeat + status
        // ===================================================================
        defmt::info!("=== Meridian Phase 9 MVP boot complete ===");
        defmt::info!("  Primary:   MNP heartbeat @ 1 Hz on USART1 (PA9)");
        defmt::info!("  Secondary: MAVLink HB   @ 1 Hz on UART7  (PE8)");

        let mut last_led_toggle: u32 = 0;
        let mut last_heartbeat: u32 = 0;
        let mut last_status: u32 = 0;
        let mut mnp_seq: u16 = 0;
        let mut mavlink_seq: u8 = 0;

        loop {
            let tick = TICK_COUNT.load(Ordering::Relaxed);

            // Toggle PA4 from the main context based on TIM6_TOGGLE flag.
            // We handle the GPIO toggle here because we own the pin. The ISR
            // just sets the flag. This gives a clean 1 kHz square wave on PA4
            // minus the small latency from WFI wakeup.
            if TIM6_TOGGLE.swap(false, Ordering::Relaxed) {
                if scope_pin.is_set_high() {
                    scope_pin.set_low();
                } else {
                    scope_pin.set_high();
                }
            }

            // LED blink at 2 Hz (toggle every 250 ms = 250 ticks)
            if tick.wrapping_sub(last_led_toggle) >= 250 {
                last_led_toggle = tick;
                if led.is_set_high() {
                    led.set_low();
                } else {
                    led.set_high();
                }
            }

            // Heartbeat at 1 Hz (every 1000 ms = 1000 ticks)
            if tick.wrapping_sub(last_heartbeat) >= 1000 {
                last_heartbeat = tick;

                // ─── PRIMARY: MNP heartbeat on USART1 ───
                //
                // Build an MnpMessage::Heartbeat, serialize with postcard,
                // COBS-frame it, and send over USART1. This is what the
                // RFD 900x radio and ATAK plugin receive natively.
                //
                // Frame on wire: [0x00] [COBS(0x01 + seq_le + postcard(Heartbeat))] [0x00]
                let mnp_hb = MnpMessage::Heartbeat(Heartbeat {
                    vehicle_type: 1,     // quad
                    armed: false,
                    mode: 0,             // stabilize
                    system_status: 3,    // standby
                });

                let mut mnp_frame = [0u8; MAX_FRAME_SIZE];
                let mnp_len = mnp_hb.encode(mnp_seq, &mut mnp_frame);
                mnp_seq = mnp_seq.wrapping_add(1);

                if mnp_len > 0 {
                    use stm32h7xx_hal::hal::blocking::serial::Write as BlockingWrite;
                    let _ = usart1.bwrite_all(&mnp_frame[..mnp_len]);
                    defmt::trace!("MNP HB #{} ({} bytes)", mnp_seq, mnp_len);
                } else {
                    defmt::error!("MNP heartbeat encode failed");
                }

                // ─── SECONDARY: MAVLink v2 heartbeat on UART7 ───
                //
                // Standard MAVLink frame for QGC/Mission Planner compatibility.
                // Runs on a separate UART, independent of MNP.
                let mut mav_frame: [u8; 21] = [0; 21];
                mav_frame[..18].copy_from_slice(&MAVLINK_HEARTBEAT_TEMPLATE);
                mav_frame[4] = mavlink_seq;           // sequence number in header
                mav_frame[18] = MAVLINK_VERSION_BYTE; // 9th payload byte (mavlink_version=3)

                mavlink_seq = mavlink_seq.wrapping_add(1);

                // CRC covers bytes 1..19 (skip STX at [0], include all header+payload)
                let crc = mavlink_crc(&mav_frame[1..19], HEARTBEAT_CRC_EXTRA);
                mav_frame[19] = (crc & 0xFF) as u8;
                mav_frame[20] = (crc >> 8) as u8;

                // Transmit via UART7 (blocking for MVP)
                {
                    use stm32h7xx_hal::hal::blocking::serial::Write as BlockingWrite;
                    let _ = uart7.bwrite_all(&mav_frame);
                }

                defmt::trace!("MAV HB #{}", mavlink_seq);
            }

            // Status log every 5 seconds
            if tick.wrapping_sub(last_status) >= 5000 {
                last_status = tick;
                let dwt_cycles = cortex_m::peripheral::DWT::cycle_count();
                defmt::info!("Alive: tick={}, DWT={}, MNP#{}, MAV#{}",
                    tick, dwt_cycles, mnp_seq, mavlink_seq);
            }

            // WFI: sleep until next interrupt (TIM6 wakes us at 1 kHz)
            cortex_m::asm::wfi();
        }
    }

    // ===================================================================
    // TIM6 ISR — 1 kHz tick
    // ===================================================================
    //
    // This is a bare interrupt handler (not RTIC). For the MVP we just
    // increment a counter and set a toggle flag. The full RTIC app will
    // replace this with proper task scheduling.
    #[interrupt]
    unsafe fn TIM6_DAC() {
        // Clear the update interrupt flag
        let tim6 = &*pac::TIM6::ptr();
        tim6.sr.modify(|_, w| w.uif().clear_bit());

        // Increment global tick counter
        TICK_COUNT.fetch_add(1, Ordering::Relaxed);

        // Signal main loop to toggle scope pin
        TIM6_TOGGLE.store(true, Ordering::Relaxed);
    }
}
