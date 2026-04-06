#![no_std]

//! Board configuration types — defines a flight controller board's hardware layout.
//!
//! Each board gets a TOML file in `boards/<BoardName>.toml` that specifies:
//! - MCU type, clock, flash/RAM sizes
//! - SPI bus pin assignments and device mappings
//! - I2C bus assignments
//! - UART serial port order
//! - PWM output timer/channel assignments
//! - IMU/baro/compass probe lists with rotations
//! - ADC pin assignments for battery monitoring
//! - LED, buzzer, safety switch pins
//!
//! At build time, `build.rs` reads the selected board TOML and generates
//! a Rust const struct with all configuration. No runtime parsing needed.
//!
//! This replaces ArduPilot's `hwdef.dat` + `chibios_hwdef.py` code generation.

/// MCU family identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McuFamily {
    Stm32H743,
    Stm32H757,
    Stm32H755,
    Stm32H750,
    Stm32H723,
    Stm32H730,
    Stm32H7A3,
    Stm32F405,
    Stm32F407,
    Stm32F412,
    Stm32F427,
    Stm32F446,
    Stm32F745,
    Stm32F765,
    Stm32F777,
    Stm32G474,
    Stm32G491,
    Stm32L431,
    Stm32L476,
}

/// SPI bus configuration.
#[derive(Debug, Clone, Copy)]
pub struct SpiBusConfig {
    pub bus_index: u8,
    pub sck_port: u8,   // 0=A, 1=B, 2=C, etc.
    pub sck_pin: u8,
    pub miso_port: u8,
    pub miso_pin: u8,
    pub mosi_port: u8,
    pub mosi_pin: u8,
    pub af: u8,          // alternate function number
    pub dma_noshare: bool,
}

/// SPI device on a bus.
#[derive(Debug, Clone, Copy)]
pub struct SpiDeviceConfig {
    pub bus_index: u8,
    pub cs_port: u8,
    pub cs_pin: u8,
    pub mode: u8,        // 0 or 3
    pub low_speed_hz: u32,
    pub high_speed_hz: u32,
}

/// I2C bus configuration.
#[derive(Debug, Clone, Copy)]
pub struct I2cBusConfig {
    pub bus_index: u8,
    pub scl_port: u8,
    pub scl_pin: u8,
    pub sda_port: u8,
    pub sda_pin: u8,
    pub af: u8,
    pub speed_khz: u16,  // 100 or 400
    pub internal: bool,   // true = onboard sensors, false = external connector
}

/// UART configuration.
#[derive(Debug, Clone, Copy)]
pub struct UartConfig {
    pub uart_index: u8,  // peripheral number (USART1=1, UART4=4, etc.)
    pub tx_port: u8,
    pub tx_pin: u8,
    pub rx_port: u8,
    pub rx_pin: u8,
    pub af: u8,
    pub is_usb: bool,
    pub default_baud: u32,
    pub default_protocol: u8, // 0=none, 1=MAVLink1, 2=MAVLink2, 5=GPS, etc.
}

/// PWM output channel configuration.
#[derive(Debug, Clone, Copy)]
pub struct PwmOutputConfig {
    pub timer: u8,       // TIM1, TIM4, etc.
    pub channel: u8,     // 1-4
    pub port: u8,
    pub pin: u8,
    pub af: u8,
    pub complementary: bool,
}

/// Sensor rotation (matches ArduPilot's ROTATION_* enum).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorRotation {
    None = 0,
    Yaw45 = 1,
    Yaw90 = 2,
    Yaw135 = 3,
    Yaw180 = 4,
    Yaw225 = 5,
    Yaw270 = 6,
    Yaw315 = 7,
    Roll180 = 8,
    Roll180Yaw45 = 9,
    Roll180Yaw90 = 10,
    Roll180Yaw135 = 11,
    Roll180Yaw180 = 12,
    Roll180Yaw225 = 13,
    Roll180Yaw270 = 14,
    Roll180Yaw315 = 15,
    Pitch180 = 24,
    Pitch270 = 25,
    Pitch90 = 26,
}

/// IMU driver type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuDriver {
    Invensensev3,  // ICM-42688, ICM-42605
    Invensense,    // MPU6000, MPU9250, ICM-20608
    Bmi270,
    Bmi088,
    Lsm6dsv,
    Adis1647x,
}

/// Barometer driver type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaroDriver {
    Bmp280,
    Bmp388,
    Bmp390,
    Dps310,
    Spl06,
    Ms5611,
    Icp201xx,
}

/// Compass driver type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompassDriver {
    Ist8310,
    Qmc5883l,
    Rm3100,
    Lis3mdl,
    Ak8963,  // on MPU9250 auxiliary bus
    Bmm150,
}

/// Sensor probe entry.
#[derive(Debug, Clone, Copy)]
pub struct ImuProbe {
    pub driver: ImuDriver,
    pub bus_type: BusType,
    pub bus_index: u8,
    pub device_index: u8, // index into spi_devices or i2c address
    pub rotation: SensorRotation,
}

#[derive(Debug, Clone, Copy)]
pub struct BaroProbe {
    pub driver: BaroDriver,
    pub bus_type: BusType,
    pub bus_index: u8,
    pub address: u8, // I2C address or SPI device index
}

#[derive(Debug, Clone, Copy)]
pub struct CompassProbe {
    pub driver: CompassDriver,
    pub bus_type: BusType,
    pub bus_index: u8,
    pub address: u8,
    pub rotation: SensorRotation,
    pub external: bool,
}

/// Bus type for sensor probing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusType {
    Spi,
    I2c,
}

/// ADC pin configuration for battery monitoring.
#[derive(Debug, Clone, Copy)]
pub struct AdcConfig {
    pub voltage_pin: u8,   // ADC channel for battery voltage
    pub current_pin: u8,   // ADC channel for battery current
    pub voltage_scale: f32, // multiplier: ADC reading * scale = battery voltage
    pub current_scale: f32, // multiplier: ADC reading * scale = battery current (A)
}

/// Complete board configuration — everything needed to run on a specific FC.
#[derive(Debug, Clone)]
pub struct BoardConfig {
    pub name: &'static str,
    pub board_id: u16,           // APJ board ID for firmware matching
    pub mcu: McuFamily,
    pub clock_hz: u32,
    pub oscillator_hz: u32,
    pub flash_size_kb: u32,
    pub ram_size_kb: u32,

    // Bus configuration
    pub spi_buses: &'static [SpiBusConfig],
    pub spi_devices: &'static [SpiDeviceConfig],
    pub i2c_buses: &'static [I2cBusConfig],
    pub serial_order: &'static [UartConfig],

    // Output
    pub pwm_outputs: &'static [PwmOutputConfig],

    // Sensors
    pub imu_probes: &'static [ImuProbe],
    pub baro_probes: &'static [BaroProbe],
    pub compass_probes: &'static [CompassProbe],

    // Power
    pub adc: AdcConfig,

    // Features
    pub has_sdcard: bool,
    pub has_iomcu: bool,
    pub has_ethernet: bool,
    pub has_can: bool,
    pub can_bus_count: u8,

    // Misc pins
    pub led_pin: Option<u16>,
    pub buzzer_pin: Option<u16>,
    pub safety_pin: Option<u16>,
}

/// The MatekH743 board configuration — Meridian's primary reference board.
/// Source: ArduPilot hwdef/MatekH743/hwdef.dat + docs/audit_board_configs.md
pub const MATEK_H743: BoardConfig = BoardConfig {
    name: "MatekH743",
    board_id: 140,
    mcu: McuFamily::Stm32H743,
    clock_hz: 400_000_000,
    oscillator_hz: 8_000_000,
    flash_size_kb: 2048,
    ram_size_kb: 1024,

    spi_buses: &[
        SpiBusConfig { bus_index: 1, sck_port: 0, sck_pin: 5, miso_port: 0, miso_pin: 6, mosi_port: 3, mosi_pin: 7, af: 5, dma_noshare: true },
        SpiBusConfig { bus_index: 2, sck_port: 1, sck_pin: 13, miso_port: 1, miso_pin: 14, mosi_port: 1, mosi_pin: 15, af: 5, dma_noshare: false },
        SpiBusConfig { bus_index: 3, sck_port: 1, sck_pin: 3, miso_port: 1, miso_pin: 4, mosi_port: 1, mosi_pin: 5, af: 6, dma_noshare: false },
        SpiBusConfig { bus_index: 4, sck_port: 4, sck_pin: 12, miso_port: 4, miso_pin: 13, mosi_port: 4, mosi_pin: 14, af: 5, dma_noshare: true },
    ],

    spi_devices: &[
        SpiDeviceConfig { bus_index: 1, cs_port: 2, cs_pin: 15, mode: 3, low_speed_hz: 2_000_000, high_speed_hz: 16_000_000 },  // IMU1: ICM42688
        SpiDeviceConfig { bus_index: 4, cs_port: 4, cs_pin: 11, mode: 3, low_speed_hz: 1_000_000, high_speed_hz: 4_000_000 },   // IMU2: ICM20602
        SpiDeviceConfig { bus_index: 4, cs_port: 2, cs_pin: 13, mode: 3, low_speed_hz: 2_000_000, high_speed_hz: 16_000_000 },  // IMU3: ICM42605
        SpiDeviceConfig { bus_index: 2, cs_port: 1, cs_pin: 12, mode: 0, low_speed_hz: 10_000_000, high_speed_hz: 10_000_000 }, // OSD: MAX7456
    ],

    i2c_buses: &[
        I2cBusConfig { bus_index: 1, scl_port: 1, scl_pin: 6, sda_port: 1, sda_pin: 7, af: 4, speed_khz: 400, internal: false },
        I2cBusConfig { bus_index: 2, scl_port: 1, scl_pin: 10, sda_port: 1, sda_pin: 11, af: 4, speed_khz: 400, internal: false },
    ],

    serial_order: &[
        UartConfig { uart_index: 0, tx_port: 0, tx_pin: 0, rx_port: 0, rx_pin: 0, af: 0, is_usb: true, default_baud: 115200, default_protocol: 2 },   // SERIAL0: USB
        UartConfig { uart_index: 7, tx_port: 4, tx_pin: 8, rx_port: 4, rx_pin: 7, af: 7, is_usb: false, default_baud: 57600, default_protocol: 2 },    // SERIAL1: Telem1
        UartConfig { uart_index: 1, tx_port: 0, tx_pin: 9, rx_port: 0, rx_pin: 10, af: 7, is_usb: false, default_baud: 57600, default_protocol: 2 },   // SERIAL2: Telem2
        UartConfig { uart_index: 2, tx_port: 3, tx_pin: 5, rx_port: 3, rx_pin: 6, af: 7, is_usb: false, default_baud: 115200, default_protocol: 5 },   // SERIAL3: GPS1
        UartConfig { uart_index: 3, tx_port: 3, tx_pin: 8, rx_port: 3, rx_pin: 9, af: 7, is_usb: false, default_baud: 115200, default_protocol: 5 },   // SERIAL4: GPS2
        UartConfig { uart_index: 8, tx_port: 4, tx_pin: 1, rx_port: 4, rx_pin: 0, af: 8, is_usb: false, default_baud: 115200, default_protocol: 0 },   // SERIAL5: spare
        UartConfig { uart_index: 4, tx_port: 1, tx_pin: 9, rx_port: 1, rx_pin: 8, af: 8, is_usb: false, default_baud: 115200, default_protocol: 0 },   // SERIAL6: spare
        UartConfig { uart_index: 6, tx_port: 2, tx_pin: 6, rx_port: 2, rx_pin: 7, af: 8, is_usb: false, default_baud: 115200, default_protocol: 23 },  // SERIAL7: RC
    ],

    pwm_outputs: &[
        PwmOutputConfig { timer: 3, channel: 3, port: 1, pin: 0, af: 2, complementary: false },  // M1
        PwmOutputConfig { timer: 3, channel: 4, port: 1, pin: 1, af: 2, complementary: false },  // M2
        PwmOutputConfig { timer: 1, channel: 1, port: 0, pin: 8, af: 1, complementary: false },  // M3
        PwmOutputConfig { timer: 1, channel: 2, port: 0, pin: 9, af: 1, complementary: false },  // M4 (note: swapped from some docs)
        PwmOutputConfig { timer: 1, channel: 3, port: 0, pin: 10, af: 1, complementary: false }, // M5
        PwmOutputConfig { timer: 1, channel: 4, port: 0, pin: 11, af: 1, complementary: false }, // M6 (note: shared with LED)
        PwmOutputConfig { timer: 4, channel: 1, port: 3, pin: 12, af: 2, complementary: false }, // M7
        PwmOutputConfig { timer: 4, channel: 2, port: 3, pin: 13, af: 2, complementary: false }, // M8
        PwmOutputConfig { timer: 5, channel: 1, port: 0, pin: 0, af: 2, complementary: false },  // M9
        PwmOutputConfig { timer: 5, channel: 2, port: 0, pin: 1, af: 2, complementary: false },  // M10
        PwmOutputConfig { timer: 5, channel: 3, port: 0, pin: 2, af: 2, complementary: false },  // M11
        PwmOutputConfig { timer: 5, channel: 4, port: 0, pin: 3, af: 2, complementary: false },  // M12
        PwmOutputConfig { timer: 15, channel: 1, port: 4, pin: 5, af: 4, complementary: false }, // LED
    ],

    imu_probes: &[
        ImuProbe { driver: ImuDriver::Invensensev3, bus_type: BusType::Spi, bus_index: 1, device_index: 0, rotation: SensorRotation::Yaw180 },
        ImuProbe { driver: ImuDriver::Invensense, bus_type: BusType::Spi, bus_index: 4, device_index: 1, rotation: SensorRotation::Roll180Yaw270 },
        ImuProbe { driver: ImuDriver::Invensensev3, bus_type: BusType::Spi, bus_index: 4, device_index: 2, rotation: SensorRotation::Yaw180 },
    ],

    baro_probes: &[
        BaroProbe { driver: BaroDriver::Dps310, bus_type: BusType::I2c, bus_index: 2, address: 0x76 },
        BaroProbe { driver: BaroDriver::Bmp280, bus_type: BusType::I2c, bus_index: 2, address: 0x76 },
        BaroProbe { driver: BaroDriver::Ms5611, bus_type: BusType::I2c, bus_index: 2, address: 0x77 },
    ],

    compass_probes: &[],  // External only on MatekH743

    adc: AdcConfig {
        voltage_pin: 10,
        current_pin: 11,
        voltage_scale: 11.0,
        current_scale: 17.0,
    },

    has_sdcard: true,
    has_iomcu: false,
    has_ethernet: false,
    has_can: false,
    can_bus_count: 0,

    led_pin: Some(0x0E05), // PE5
    buzzer_pin: Some(0x0D02), // PD2 (via BUZZER_PIN define)
    safety_pin: None,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_matek_h743_config() {
        assert_eq!(MATEK_H743.name, "MatekH743");
        assert_eq!(MATEK_H743.board_id, 140);
        assert_eq!(MATEK_H743.mcu, McuFamily::Stm32H743);
        assert_eq!(MATEK_H743.clock_hz, 400_000_000);
        assert_eq!(MATEK_H743.spi_buses.len(), 4);
        assert_eq!(MATEK_H743.serial_order.len(), 8);
        assert_eq!(MATEK_H743.pwm_outputs.len(), 13);
        assert_eq!(MATEK_H743.imu_probes.len(), 3);
        assert_eq!(MATEK_H743.baro_probes.len(), 3);
        assert!(MATEK_H743.has_sdcard);
        assert!(!MATEK_H743.has_iomcu);
    }

    #[test]
    fn test_spi_dma_noshare() {
        // SPI1 and SPI4 (IMU buses) should have DMA_NOSHARE
        assert!(MATEK_H743.spi_buses[0].dma_noshare); // SPI1
        assert!(!MATEK_H743.spi_buses[1].dma_noshare); // SPI2 (OSD)
        assert!(MATEK_H743.spi_buses[3].dma_noshare); // SPI4
    }

    #[test]
    fn test_serial_protocols() {
        // SERIAL0 = USB MAVLink2
        assert!(MATEK_H743.serial_order[0].is_usb);
        assert_eq!(MATEK_H743.serial_order[0].default_protocol, 2);
        // SERIAL3 = GPS
        assert_eq!(MATEK_H743.serial_order[3].default_protocol, 5);
        // SERIAL7 = RC (CRSF)
        assert_eq!(MATEK_H743.serial_order[7].default_protocol, 23);
    }

    #[test]
    fn test_imu_probes() {
        // First IMU: ICM42688 on SPI1
        assert_eq!(MATEK_H743.imu_probes[0].driver, ImuDriver::Invensensev3);
        assert_eq!(MATEK_H743.imu_probes[0].bus_index, 1);
        assert_eq!(MATEK_H743.imu_probes[0].rotation, SensorRotation::Yaw180);
        // Second IMU: ICM20602 on SPI4
        assert_eq!(MATEK_H743.imu_probes[1].driver, ImuDriver::Invensense);
        assert_eq!(MATEK_H743.imu_probes[1].bus_index, 4);
    }
}
