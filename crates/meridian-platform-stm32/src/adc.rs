//! STM32H743 ADC driver for battery monitoring.
//!
//! H7 ADC with DMA for continuous background sampling.
//! ADC1 is the primary ADC, running DMA continuous conversion on configured channels.
//!
//! MatekH743 ADC configuration:
//!   - Battery voltage: ADC channel 10 (PC0), scale 11.0
//!   - Battery current: ADC channel 11 (PC1), scale 17.0
//!
//! The ADC samples continuously via DMA into a buffer in AXI SRAM.
//! The timer thread (1 kHz) reads the DMA results and updates filtered values.
//!
//! H743 ADC: 16-bit resolution, 3.3V reference, DMA with circular mode.
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/AnalogIn.cpp`

use meridian_hal::analog::{AnalogIn, AnalogSource};

// ---------------------------------------------------------------------------
// ADC constants
// ---------------------------------------------------------------------------

/// ADC reference voltage (V).
const ADC_VREF: f32 = 3.3;

/// ADC resolution (16-bit on H7).
const ADC_RESOLUTION: u32 = 65536; // 2^16

/// Maximum number of ADC channels.
const MAX_ADC_CHANNELS: usize = 16;

/// IIR filter coefficient for voltage averaging (0-1, lower = more smoothing).
/// Matches ArduPilot's default 20-sample running average at 1 kHz timer rate.
const FILTER_ALPHA: f32 = 0.05;

/// Default board voltage (V) — 5.0V regulated.
const DEFAULT_BOARD_VOLTAGE: f32 = 5.0;

/// Default servo rail voltage (V).
const DEFAULT_SERVORAIL_VOLTAGE: f32 = 5.0;

// ---------------------------------------------------------------------------
// DMA buffer in AXI SRAM
// ---------------------------------------------------------------------------

/// ADC DMA buffer for continuous conversion results (in AXI SRAM).
/// Each entry is a 16-bit ADC reading, one per configured channel.
/// DMA runs in circular mode, continuously overwriting this buffer.
#[link_section = ".axisram"]
static mut ADC_DMA_BUF: [u16; MAX_ADC_CHANNELS] = [0u16; MAX_ADC_CHANNELS];

// ---------------------------------------------------------------------------
// Stm32AnalogSource — one per ADC channel
// ---------------------------------------------------------------------------

/// STM32H7 analog source representing one ADC channel.
///
/// Maintains both a filtered average and the latest raw reading.
/// The timer thread calls `update()` at 1 kHz to read from the DMA buffer.
pub struct Stm32AnalogSource {
    /// ADC channel number (0-15).
    channel: u8,
    /// Index into the DMA buffer (position in scan sequence).
    dma_index: u8,
    /// Latest raw ADC reading converted to voltage.
    voltage_latest: f32,
    /// IIR-filtered average voltage.
    voltage_average: f32,
    /// Whether this source has been initialized (first sample received).
    initialized: bool,
}

impl Stm32AnalogSource {
    /// Create a new analog source for the given ADC channel.
    fn new(channel: u8, dma_index: u8) -> Self {
        Self {
            channel,
            dma_index,
            voltage_latest: 0.0,
            voltage_average: 0.0,
            initialized: false,
        }
    }

    /// Convert a raw 16-bit ADC reading to voltage (0.0 - 3.3V).
    #[inline]
    fn raw_to_voltage(raw: u16) -> f32 {
        (raw as f32 / ADC_RESOLUTION as f32) * ADC_VREF
    }

    /// Update this source from the DMA buffer.
    /// Called by the timer thread at 1 kHz.
    pub fn update(&mut self) {
        // SAFETY: DMA writes and this read are on separate words; no tearing
        // for aligned 16-bit reads on Cortex-M7. Timer thread is the only reader.
        let raw = unsafe { ADC_DMA_BUF[self.dma_index as usize] };
        let voltage = Self::raw_to_voltage(raw);

        self.voltage_latest = voltage;

        if !self.initialized {
            // First sample: initialize average directly.
            self.voltage_average = voltage;
            self.initialized = true;
        } else {
            // IIR low-pass filter: avg = alpha * new + (1 - alpha) * old.
            self.voltage_average =
                FILTER_ALPHA * voltage + (1.0 - FILTER_ALPHA) * self.voltage_average;
        }
    }
}

impl AnalogSource for Stm32AnalogSource {
    fn voltage_average(&self) -> f32 {
        self.voltage_average
    }

    fn voltage_latest(&self) -> f32 {
        self.voltage_latest
    }

    fn set_pin(&mut self, pin: u8) {
        self.channel = pin;
        // Note: changing the pin at runtime requires reconfiguring the ADC
        // scan sequence, which is non-trivial with DMA circular mode.
        // In practice, pins are fixed at board config time.
        // TODO: reconfigure ADC scan sequence if pin changes.
    }

    fn set_stop_pin(&mut self, _pin: u8) {
        // TODO: implement stop-pin logic (rarely used on H7 boards).
    }
}

// ---------------------------------------------------------------------------
// Stm32Adc — the full AnalogIn implementation
// ---------------------------------------------------------------------------

/// STM32H7 ADC driver with DMA continuous conversion.
///
/// Manages ADC1 with a configurable set of channels in scan mode.
/// DMA runs continuously in circular mode, depositing results into
/// `ADC_DMA_BUF` in AXI SRAM. The `update()` method (called from timer
/// thread at 1 kHz) copies DMA results into per-channel filtered values.
pub struct Stm32Adc {
    /// Per-channel analog sources.
    channels: [Stm32AnalogSource; MAX_ADC_CHANNELS],
    /// Number of configured channels in the scan sequence.
    num_configured: u8,
    /// Board supply voltage (measured or nominal).
    board_voltage: f32,
    /// Servo rail voltage.
    servorail_voltage: f32,
}

impl Stm32Adc {
    /// Create a new ADC driver. Channels are not yet configured.
    pub fn new() -> Self {
        Self {
            channels: core::array::from_fn(|i| Stm32AnalogSource::new(i as u8, i as u8)),
            num_configured: 0,
            board_voltage: DEFAULT_BOARD_VOLTAGE,
            servorail_voltage: DEFAULT_SERVORAIL_VOLTAGE,
        }
    }

    /// Configure the ADC scan sequence from board config.
    /// Called once during initialization.
    pub fn configure(&mut self, voltage_pin: u8, current_pin: u8) {
        // Set up scan sequence: voltage channel first, then current channel.
        self.channels[0] = Stm32AnalogSource::new(voltage_pin, 0);
        self.channels[1] = Stm32AnalogSource::new(current_pin, 1);
        self.num_configured = 2;

        // Additional channels can be added for servo rail voltage, etc.
    }

    /// Update all channels from DMA buffer. Called from timer thread at 1 kHz.
    pub fn update(&mut self) {
        // Invalidate D-cache for the DMA buffer region before reading.
        // TODO: SCB::invalidate_dcache_by_address(ADC_DMA_BUF.as_ptr() as usize, ...)

        for i in 0..self.num_configured as usize {
            self.channels[i].update();
        }
    }
}

impl AnalogIn for Stm32Adc {
    fn init(&mut self) {
        // TODO: actual register access
        //
        // 1. Enable ADC1 clock in RCC (RCC_AHB1ENR.ADC12EN)
        // 2. Configure ADC clock prescaler (ADCCKSEL in RCC_D3CCIPR)
        // 3. Wake ADC from deep power-down (ADCR.DEEPPWD clear, ADCR.ADVREGEN set)
        // 4. Wait for LDORDY
        // 5. Calibrate ADC (single-ended, ADCAL bit, wait for completion)
        // 6. Configure resolution (16-bit: RES=0b000 in CFGR)
        // 7. Configure scan sequence:
        //    - SQR1.L = num_configured - 1
        //    - SQR1.SQ1 = channel[0], SQR1.SQ2 = channel[1], etc.
        // 8. Configure sampling time for each channel (SMPRx registers)
        // 9. Configure DMA:
        //    - Circular mode
        //    - Source: ADC1_DR
        //    - Dest: ADC_DMA_BUF
        //    - Count: num_configured
        //    - 16-bit transfers (half-word)
        //    - H743 errata: DMA_SxCR_TRBUFF must be set
        // 10. Enable continuous conversion mode (CONT bit in CFGR)
        // 11. Enable DMA request (DMAEN in CFGR)
        // 12. Start ADC (ADSTART)
    }

    fn channel(&self, channel: u8) -> Option<&dyn AnalogSource> {
        let idx = channel as usize;
        if idx < self.num_configured as usize {
            Some(&self.channels[idx] as &dyn AnalogSource)
        } else {
            None
        }
    }

    fn board_voltage(&self) -> f32 {
        self.board_voltage
    }

    fn servorail_voltage(&self) -> f32 {
        self.servorail_voltage
    }
}
