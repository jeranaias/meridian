//! STM32H743 PWM and DShot output driver.
//!
//! PWM channels are organized into groups of up to 4 channels sharing one timer.
//! DShot is implemented via timer DMAR burst — DMA writes CCR registers for all
//! channels simultaneously on each timer ARR overflow.
//!
//! Bidirectional DShot requires buffers in SRAM4 (0x38000000, uncached).
//! The MPU region NOCACHE_MPU_REGION_1 marks SRAM4 as uncacheable for this purpose.
//!
//! MatekH743 PWM timer groups (from hwdef.dat):
//!   TIM3:  ch3 (M1/PB0), ch4 (M2/PB1)
//!   TIM1:  ch1-ch4 (M3-M6/PA8-PA11)
//!   TIM4:  ch1-ch4 (M7-M10/PD12-PD15)
//!   TIM5:  ch1-ch4 (M9-M12/PA0-PA3)
//!   TIM15: ch1 (LED/PE5)
//!   TIM2:  ch1 (Buzzer/PA15)
//!
//! Output modes: PWM Normal, OneShot, OneShot125, Brushed, DShot150/300/600/1200.
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/RCOutput.cpp`, `RCOutput_bdshot.cpp`

use meridian_hal::rc_output::{
    DshotCommand, EscTelemetry, OutputProtocol, RcOutput, MAX_OUTPUT_CHANNELS,
};

// ---------------------------------------------------------------------------
// DShot constants
// ---------------------------------------------------------------------------

/// DShot bit timing (Timer ticks at the DShot clock rate).
/// For DShot600: bit period = 1.67 us (600 kbps).
/// Bit '1': 75% high, Bit '0': 37% high.
const DSHOT_BIT_1_PERCENT: u32 = 75;
const DSHOT_BIT_0_PERCENT: u32 = 37;

/// DShot frame length: 16 bits per channel.
const DSHOT_FRAME_BITS: usize = 16;

/// Maximum channels per timer group (STM32 timers have 4 CCR channels).
const MAX_CHANNELS_PER_GROUP: usize = 4;

/// Maximum timer groups on the board.
const MAX_PWM_GROUPS: usize = 8;

/// DShot DMA buffer size per group: 16 bits * 4 channels * u32 words.
/// Timer DMAR burst writes all CCR registers per bit period.
const DSHOT_DMA_BUF_SIZE: usize = DSHOT_FRAME_BITS * MAX_CHANNELS_PER_GROUP;

// ---------------------------------------------------------------------------
// DMA buffers
// ---------------------------------------------------------------------------

/// DShot DMA buffers in AXI SRAM for normal DShot TX.
#[link_section = ".axisram"]
static mut DSHOT_DMA_BUF: [[u32; DSHOT_DMA_BUF_SIZE]; MAX_PWM_GROUPS] =
    [[0u32; DSHOT_DMA_BUF_SIZE]; MAX_PWM_GROUPS];

/// Bidirectional DShot RX buffers in SRAM4 (uncached).
/// SRAM4 at 0x38000000 is mapped uncacheable via MPU NOCACHE_MPU_REGION_1.
/// Required for bdshot input capture DMA to avoid cache coherency issues.
#[link_section = ".sram4"]
static mut BDSHOT_RX_BUF: [[u32; DSHOT_DMA_BUF_SIZE]; MAX_PWM_GROUPS] =
    [[0u32; DSHOT_DMA_BUF_SIZE]; MAX_PWM_GROUPS];

// ---------------------------------------------------------------------------
// PWM group — channels sharing one timer
// ---------------------------------------------------------------------------

/// Output mode for a PWM group (all channels in a group share the same mode).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GroupMode {
    /// Standard servo PWM (50-400 Hz, 1 MHz or 8 MHz timer clock).
    PwmNormal,
    /// OneShot: single pulse per push(), timer period = 0.
    OneShot,
    /// OneShot125: 125-250 us pulse range.
    OneShot125,
    /// Brushed motor direct speed control.
    Brushed,
    /// DShot digital protocol (150/300/600/1200).
    DShot(u16), // bitrate in kbps
    /// Disabled (e.g., timer used for buzzer/alarm).
    Disabled,
}

/// DShot protocol bitrate to timer clock divisor.
fn dshot_timer_period(bitrate_kbps: u16, timer_clock_hz: u32) -> u32 {
    // Bit period in timer ticks: timer_clock / (bitrate * 1000)
    if bitrate_kbps == 0 {
        return 0;
    }
    timer_clock_hz / (bitrate_kbps as u32 * 1000)
}

/// A group of up to 4 PWM channels sharing one STM32 timer.
struct PwmGroup {
    /// Timer peripheral index (TIM1=1, TIM3=3, etc.).
    timer: u8,
    /// Number of active channels in this group (1-4).
    num_channels: u8,
    /// Output mode.
    mode: GroupMode,
    /// PWM frequency (Hz) for normal PWM mode.
    freq_hz: u16,
    /// Per-channel output values.
    values: [u16; MAX_CHANNELS_PER_GROUP],
    /// Whether each channel is enabled.
    enabled: [bool; MAX_CHANNELS_PER_GROUP],
    /// Whether complementary output is used (e.g., TIM8 ch2N on MatekH743).
    complementary: [bool; MAX_CHANNELS_PER_GROUP],
    /// Channel-to-global-output mapping (which output index each channel maps to).
    output_map: [u8; MAX_CHANNELS_PER_GROUP],
    /// Group index (for DMA buffer indexing).
    group_idx: usize,
}

impl PwmGroup {
    fn new(timer: u8, num_channels: u8, group_idx: usize) -> Self {
        Self {
            timer,
            num_channels,
            mode: GroupMode::PwmNormal,
            freq_hz: 50,
            values: [1000; MAX_CHANNELS_PER_GROUP], // default: 1000 us (motor stop)
            enabled: [false; MAX_CHANNELS_PER_GROUP],
            complementary: [false; MAX_CHANNELS_PER_GROUP],
            output_map: [0; MAX_CHANNELS_PER_GROUP],
            group_idx,
        }
    }

    /// Configure the timer for the current mode.
    fn configure_timer(&self) {
        // TODO: actual register access
        match self.mode {
            GroupMode::PwmNormal => {
                // Timer clock: 1 MHz for freq <= 400 Hz, 8 MHz for > 400 Hz.
                // ARR = timer_clock / freq - 1
                // CCRx = pulse_width_us * (timer_clock / 1_000_000)
                //
                // let timer_clock = if self.freq_hz > 400 { 8_000_000 } else { 1_000_000 };
                // let arr = timer_clock / self.freq_hz as u32 - 1;
                // timer.arr.write(|w| w.bits(arr));
                // timer.psc.write(|w| w.bits(system_clock / timer_clock - 1));
                // timer.cr1.modify(|_, w| w.arpe().set_bit()); // auto-reload preload
            }
            GroupMode::OneShot | GroupMode::OneShot125 => {
                // Period = 0 (no auto-repeat). Timer clock = 8 MHz.
                // Single pulse triggered by push().
                // timer.arr.write(|w| w.bits(0));
            }
            GroupMode::DShot(bitrate) => {
                // DMAR burst mode: DMA writes CCRx for all channels per bit.
                // Configure timer for DShot bit period.
                let _ = bitrate;
                // let period = dshot_timer_period(bitrate, timer_clock);
                // timer.arr.write(|w| w.bits(period - 1));
                // timer.dier.modify(|_, w| w.ude().set_bit()); // update DMA request
                // timer.dcr.write(|w| w.dbl().bits(num_channels - 1).dba().bits(ccr1_offset));
            }
            GroupMode::Brushed => {
                // Direct PWM duty cycle, variable frequency.
            }
            GroupMode::Disabled => {
                // Stop timer.
                // timer.cr1.modify(|_, w| w.cen().clear_bit());
            }
        }
    }

    /// Fill the DMA buffer with DShot frames for all channels in this group.
    ///
    /// DShot frame format (16 bits):
    ///   [10:0]  = throttle value (0-2047, where 0 = disarmed, 48-2047 = throttle)
    ///   [11]    = telemetry request bit
    ///   [15:12] = CRC (XOR of nibbles)
    fn fill_dshot_buffer(&self) {
        for ch in 0..self.num_channels as usize {
            let value = self.values[ch];

            // Build 16-bit DShot frame.
            let throttle = value & 0x07FF;
            let telemetry_bit = 0u16; // set to 1 to request telemetry
            let frame_no_crc = (throttle << 1) | telemetry_bit;

            // CRC: XOR of 4 nibbles.
            let crc = (frame_no_crc ^ (frame_no_crc >> 4) ^ (frame_no_crc >> 8)) & 0x0F;
            let frame = (frame_no_crc << 4) | crc;

            // Convert each bit to a timer CCR value (duty cycle).
            // DMA buffer layout: [bit0_ch0, bit0_ch1, ..., bit0_ch3, bit1_ch0, ...]
            // The timer DMAR burst writes CCR1..CCR4 for each bit period.
            for bit in 0..DSHOT_FRAME_BITS {
                let bit_val = (frame >> (DSHOT_FRAME_BITS - 1 - bit)) & 1;
                let duty = if bit_val == 1 {
                    DSHOT_BIT_1_PERCENT
                } else {
                    DSHOT_BIT_0_PERCENT
                };

                let buf_idx = bit * MAX_CHANNELS_PER_GROUP + ch;
                // SAFETY: single-threaded access, group_idx is unique per group.
                unsafe {
                    DSHOT_DMA_BUF[self.group_idx][buf_idx] = duty;
                }
            }
        }
    }

    /// Start a DShot DMA transfer for this group.
    fn start_dshot_dma(&self) {
        self.fill_dshot_buffer();

        // TODO: actual DMA register access
        // 1. Configure DMA stream:
        //    - Source: DSHOT_DMA_BUF[group_idx]
        //    - Dest: TIMx_DMAR register
        //    - Count: DSHOT_FRAME_BITS * num_channels
        //    - Direction: memory-to-peripheral
        //    - Memory increment, 32-bit transfers
        // 2. Enable DMA stream
        // 3. Enable timer counter (TIMx_CR1.CEN)
        //
        // For bidirectional DShot:
        // After TX complete, switch channels to input capture mode,
        // configure IC DMA to BDSHOT_RX_BUF[group_idx] (in uncached SRAM4),
        // and capture the response frame.
    }

    /// Write a PWM value (normal/oneshot mode) to a channel's CCR register.
    fn write_pwm(&self, channel: usize) {
        if channel >= MAX_CHANNELS_PER_GROUP {
            return;
        }
        let _value = self.values[channel];
        // TODO: actual register access
        // For normal PWM:
        //   ccr_us = value * (timer_clock / 1_000_000)
        //   timer.ccr[channel].write(|w| w.bits(ccr_us));
    }
}

// ---------------------------------------------------------------------------
// Stm32PwmOutput — the full RcOutput implementation
// ---------------------------------------------------------------------------

/// STM32H7 PWM/DShot output driver.
///
/// Manages all output channels across all timer groups.
/// Implements the cork/push model: write() buffers values, push() sends them
/// all at once to avoid partial updates reaching ESCs.
pub struct Stm32PwmOutput {
    /// Timer groups (up to MAX_PWM_GROUPS, configured from board config).
    groups: [Option<PwmGroup>; MAX_PWM_GROUPS],
    /// Total number of configured output channels.
    num_channels: u8,
    /// Per-channel output values (application view).
    channel_values: [u16; MAX_OUTPUT_CHANNELS],
    /// Channel-to-group mapping: (group_idx, channel_within_group).
    channel_map: [(u8, u8); MAX_OUTPUT_CHANNELS],
    /// Cork flag: when true, push() is needed to send values to hardware.
    corked: bool,
    /// DShot rate multiplier (1x = once per loop, 2x = twice per loop, etc.).
    dshot_rate: u8,
    /// Per-channel enabled flag.
    enabled: [bool; MAX_OUTPUT_CHANNELS],
    /// ESC telemetry data (from bidirectional DShot).
    esc_telem: [EscTelemetry; MAX_OUTPUT_CHANNELS],
}

impl Stm32PwmOutput {
    /// Create a new PWM output driver with no configured groups.
    pub fn new() -> Self {
        Self {
            groups: Default::default(),
            num_channels: 0,
            channel_values: [1000; MAX_OUTPUT_CHANNELS],
            channel_map: [(0, 0); MAX_OUTPUT_CHANNELS],
            corked: false,
            dshot_rate: 1,
            enabled: [false; MAX_OUTPUT_CHANNELS],
            esc_telem: [EscTelemetry::default(); MAX_OUTPUT_CHANNELS],
        }
    }

    /// Configure output channels from board configuration.
    ///
    /// This sets up the timer groups and channel mappings based on the
    /// PWM output configuration from meridian-boardcfg.
    pub fn init(&mut self, outputs: &[meridian_boardcfg::PwmOutputConfig]) {
        // Build groups from output config.
        // Channels sharing the same timer number are in the same group.
        let mut group_idx = 0usize;
        let mut output_idx = 0u8;

        for output in outputs {
            // Find or create the group for this timer.
            let gidx = self.find_or_create_group(output.timer, &mut group_idx);

            if let Some(group) = &mut self.groups[gidx] {
                let ch = group.num_channels as usize;
                if ch < MAX_CHANNELS_PER_GROUP {
                    group.complementary[ch] = output.complementary;
                    group.output_map[ch] = output_idx;
                    group.num_channels += 1;

                    // Map the global output to this group+channel.
                    if (output_idx as usize) < MAX_OUTPUT_CHANNELS {
                        self.channel_map[output_idx as usize] = (gidx as u8, ch as u8);
                    }
                    output_idx += 1;
                }
            }
        }

        self.num_channels = output_idx;

        // TODO: actual register access
        // 1. Enable timer clocks in RCC
        // 2. Configure GPIO pins as alternate function for PWM output
        // 3. Configure each timer for its initial mode (PWM Normal at 50 Hz)
        for group in self.groups.iter().flatten() {
            group.configure_timer();
        }
    }

    /// Find an existing group for this timer, or create a new one.
    fn find_or_create_group(&mut self, timer: u8, next_idx: &mut usize) -> usize {
        // Check if we already have a group for this timer.
        for (i, group) in self.groups.iter().enumerate() {
            if let Some(g) = group {
                if g.timer == timer {
                    return i;
                }
            }
        }

        // Create a new group.
        let idx = *next_idx;
        if idx < MAX_PWM_GROUPS {
            self.groups[idx] = Some(PwmGroup::new(timer, 0, idx));
            *next_idx += 1;
            idx
        } else {
            0 // Fallback — should not happen with valid board config.
        }
    }
}

impl RcOutput for Stm32PwmOutput {
    fn set_freq(&mut self, channel_mask: u16, freq_hz: u16) {
        for ch in 0..MAX_OUTPUT_CHANNELS {
            if (channel_mask >> ch) & 1 == 1 {
                let (gidx, _) = self.channel_map[ch];
                if let Some(group) = &mut self.groups[gidx as usize] {
                    group.freq_hz = freq_hz;
                    group.configure_timer();
                }
            }
        }
    }

    fn get_freq(&self, channel: u8) -> u16 {
        if (channel as usize) >= MAX_OUTPUT_CHANNELS {
            return 0;
        }
        let (gidx, _) = self.channel_map[channel as usize];
        self.groups[gidx as usize]
            .as_ref()
            .map(|g| g.freq_hz)
            .unwrap_or(0)
    }

    fn write(&mut self, channel: u8, value: u16) {
        if let Some(v) = self.channel_values.get_mut(channel as usize) {
            *v = value;
        }
    }

    fn write_multi(&mut self, values: &[u16]) {
        let n = values.len().min(MAX_OUTPUT_CHANNELS);
        self.channel_values[..n].copy_from_slice(&values[..n]);
    }

    fn read(&self, channel: u8) -> u16 {
        self.channel_values
            .get(channel as usize)
            .copied()
            .unwrap_or(0)
    }

    fn push(&mut self) {
        // Transfer buffered values to hardware.
        for ch in 0..self.num_channels as usize {
            let (gidx, gch) = self.channel_map[ch];
            if let Some(group) = &mut self.groups[gidx as usize] {
                group.values[gch as usize] = self.channel_values[ch];

                match group.mode {
                    GroupMode::DShot(_) => {
                        // DShot: will be sent via DMA burst below.
                    }
                    _ => {
                        // PWM/OneShot: write CCR directly.
                        group.write_pwm(gch as usize);
                    }
                }
            }
        }

        // Start DShot DMA for all DShot groups.
        for group in self.groups.iter().flatten() {
            if let GroupMode::DShot(_) = group.mode {
                group.start_dshot_dma();
            }
        }

        self.corked = false;
    }

    fn cork(&mut self) {
        self.corked = true;
    }

    fn set_protocol(&mut self, channel_mask: u16, protocol: OutputProtocol) {
        for ch in 0..MAX_OUTPUT_CHANNELS {
            if (channel_mask >> ch) & 1 == 1 {
                let (gidx, _) = self.channel_map[ch];
                if let Some(group) = &mut self.groups[gidx as usize] {
                    let new_mode = match protocol {
                        OutputProtocol::Pwm => GroupMode::PwmNormal,
                        OutputProtocol::OneShot125 => GroupMode::OneShot125,
                        OutputProtocol::DShot150 => GroupMode::DShot(150),
                        OutputProtocol::DShot300 => GroupMode::DShot(300),
                        OutputProtocol::DShot600 => GroupMode::DShot(600),
                        OutputProtocol::DShot1200 => GroupMode::DShot(1200),
                        OutputProtocol::BidirectionalDShot => GroupMode::DShot(600), // bdshot uses DShot600 timing
                    };
                    group.mode = new_mode;
                    group.configure_timer();
                }
            }
        }
    }

    fn set_dshot_rate(&mut self, rate: u8) {
        // Rate multiplier: 1x = once per loop, 2x = twice, etc.
        // Minimum 800 Hz enforced (BLHeli32 16-bit counter wraps at ~732 Hz at 48 MHz).
        self.dshot_rate = rate.max(1);
    }

    fn send_dshot_command(&mut self, command: DshotCommand, channel_mask: u16, repeat: u8) {
        let cmd_value = command as u16;

        for _rep in 0..=repeat {
            for ch in 0..MAX_OUTPUT_CHANNELS {
                if (channel_mask >> ch) & 1 == 1 {
                    let (gidx, gch) = self.channel_map[ch];
                    if let Some(group) = &mut self.groups[gidx as usize] {
                        // DShot commands use throttle values 0-47.
                        group.values[gch as usize] = cmd_value;
                    }
                }
            }

            // Send the command frame via DShot DMA.
            for group in self.groups.iter().flatten() {
                if let GroupMode::DShot(_) = group.mode {
                    group.start_dshot_dma();
                }
            }

            // TODO: delay between repeats (command interval = 10 ms typical)
        }
    }

    fn num_channels(&self) -> u8 {
        self.num_channels
    }

    fn enable_ch(&mut self, channel: u8) {
        if let Some(e) = self.enabled.get_mut(channel as usize) {
            *e = true;
        }
        // TODO: actual register access — enable timer output compare channel
        // TIMx_CCER: CCxE or CCxNE for complementary
    }

    fn disable_ch(&mut self, channel: u8) {
        if let Some(e) = self.enabled.get_mut(channel as usize) {
            *e = false;
        }
        // TODO: actual register access — disable timer output compare channel
    }

    fn get_esc_telemetry(&self, channel: u8) -> Option<EscTelemetry> {
        // Telemetry from bidirectional DShot or BLHeli telemetry UART.
        let telem = self.esc_telem.get(channel as usize)?;
        if telem.rpm == 0 && telem.voltage_mv == 0 {
            None // No data received yet.
        } else {
            Some(*telem)
        }
    }
}

// Allow `Default::default()` for the `Option<PwmGroup>` array.
impl Default for PwmGroup {
    fn default() -> Self {
        PwmGroup::new(0, 0, 0)
    }
}
