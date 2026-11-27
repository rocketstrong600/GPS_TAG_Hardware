//! Bosch **BMI160** 6-axis IMU driver over blocking `I2C1` (`PB8`=SCL, `PB9`=SDA, address `0x68`).
//!
//! Supports:
//! - Configurable accelerometer range (`±2g`..`±16g`) and ODR (`25..200 Hz`)
//! - Optional gyroscope (`±125..±2000 °/s`) with independent suspend/enable for power savings
//! - Hardware motion interrupts routed to `INT1` (`PB7`) or `INT2` (`PB6`):
//!   - **Any-Motion** (`enable_any_motion`) — wake on movement
//!   - **No-Motion** (`enable_no_motion`) — detect stationary state
//!   - **High-G** (`enable_high_g`) — impact / shock / tap detection
//!   - **Low-G** (`enable_low_g`) — freefall detection
//! - Single-burst 12-byte gyro+accel reads (`read_sample`) and die temperature (`read_temperature_mc`)
//!
//! See `examples/imu_motion_wake.rs` for a complete interrupt-driven wakeup example.

use defmt::{error, info, warn};
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Flex, Pull};
use embassy_stm32::i2c::{Config as I2cConfig, Error as I2cError, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals;
use embassy_stm32::time::khz;
use embassy_time::{Duration, Timer};

pub const BMI160_ADDR: u8 = 0x68;
pub const BMI160_CHIP_ID: u8 = 0xD1;

#[allow(dead_code)]
mod reg {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const PMU_STATUS: u8 = 0x03;
    pub const DATA_GYR_X_L: u8 = 0x0C; // 12 bytes: GYR X/Y/Z (0x0C..0x11) + ACC X/Y/Z (0x12..0x17)
    pub const DATA_ACC_X_L: u8 = 0x12; // 6 bytes: ACC X/Y/Z
    pub const STATUS: u8 = 0x1B;
    pub const INT_STATUS_0: u8 = 0x1C;
    pub const INT_STATUS_1: u8 = 0x1D;
    pub const TEMPERATURE_0: u8 = 0x20; // 2 bytes: 0x20..0x21
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const INT_EN_0: u8 = 0x50;
    pub const INT_EN_1: u8 = 0x51;
    pub const INT_EN_2: u8 = 0x52;
    pub const INT_OUT_CTRL: u8 = 0x53;
    pub const INT_LATCH: u8 = 0x54;
    pub const INT_MAP_0: u8 = 0x55;
    pub const INT_MAP_2: u8 = 0x57;
    pub const INT_LOWHIGH_0: u8 = 0x5A; // low-g duration
    pub const INT_LOWHIGH_1: u8 = 0x5B; // low-g threshold
    pub const INT_LOWHIGH_2: u8 = 0x5C; // low/high-g hysteresis + mode
    pub const INT_LOWHIGH_3: u8 = 0x5D; // high-g duration
    pub const INT_LOWHIGH_4: u8 = 0x5E; // high-g threshold
    pub const INT_MOTION_0: u8 = 0x5F; // anymo_dur + slo_nomo_dur
    pub const INT_MOTION_1: u8 = 0x60; // anymo_th
    pub const INT_MOTION_2: u8 = 0x61; // slo_nomo_th
    pub const CMD: u8 = 0x7E;
}

#[allow(dead_code)]
mod cmd {
    pub const ACC_PMU_SUSPEND: u8 = 0x10;
    pub const ACC_PMU_NORMAL: u8 = 0x11;
    pub const ACC_PMU_LOW_POWER: u8 = 0x12;
    pub const GYR_PMU_SUSPEND: u8 = 0x14;
    pub const GYR_PMU_NORMAL: u8 = 0x15;
    pub const INT_RESET: u8 = 0xB1;
    pub const SOFT_RESET: u8 = 0xB6;
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum Error {
    I2c(I2cError),
    WrongChipId(u8),
    DeviceError(u8),
    PmuNotReady(u8),
}

impl From<I2cError> for Error {
    fn from(e: I2cError) -> Self {
        Self::I2c(e)
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum AccelRange {
    G2,
    G4,
    G8,
    G16,
}

impl AccelRange {
    const fn reg_val(self) -> u8 {
        match self {
            Self::G2 => 0x03,
            Self::G4 => 0x05,
            Self::G8 => 0x08,
            Self::G16 => 0x0C,
        }
    }

    const fn lsb_per_g(self) -> i32 {
        match self {
            Self::G2 => 16_384,
            Self::G4 => 8_192,
            Self::G8 => 4_096,
            Self::G16 => 2_048,
        }
    }

    /// Any-motion threshold step in micro-g per LSB:
    /// ±2g: 3.91 mg, ±4g: 7.81 mg, ±8g: 15.63 mg, ±16g: 31.25 mg.
    const fn any_motion_ug_per_lsb(self) -> u32 {
        match self {
            Self::G2 => 3_910,
            Self::G4 => 7_810,
            Self::G8 => 15_625,
            Self::G16 => 31_250,
        }
    }

    /// High-G threshold step in micro-g per LSB:
    /// ±2g: 7.81 mg, ±4g: 15.63 mg, ±8g: 31.25 mg, ±16g: 62.5 mg.
    const fn high_g_ug_per_lsb(self) -> u32 {
        match self {
            Self::G2 => 7_810,
            Self::G4 => 15_625,
            Self::G8 => 31_250,
            Self::G16 => 62_500,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
#[repr(u8)]
pub enum AccelOdr {
    Hz25 = 0x06,
    Hz50 = 0x07,
    Hz100 = 0x08,
    Hz200 = 0x09,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
#[repr(u8)]
pub enum GyroRange {
    Dps2000 = 0x00,
    Dps1000 = 0x01,
    Dps500 = 0x02,
    Dps250 = 0x03,
    Dps125 = 0x04,
}

impl GyroRange {
    /// Tenths of LSB per dps (e.g. 16.4 LSB/(°/s) -> 164).
    const fn tenths_lsb_per_dps(self) -> i32 {
        match self {
            Self::Dps2000 => 164,
            Self::Dps1000 => 328,
            Self::Dps500 => 656,
            Self::Dps250 => 1312,
            Self::Dps125 => 2624,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum IntPin {
    Int1,
    Int2,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
#[repr(u8)]
pub enum IntLatch {
    None = 0x00,
    Ms10 = 0x05,
    Ms80 = 0x08,
    Ms320 = 0x0A,
    Latched = 0x0F,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct AnyMotionConfig {
    /// Slope threshold in milli-g (clamped to current `AccelRange`).
    pub threshold_mg: u16,
    /// Consecutive samples exceeding `threshold_mg` (`1..=4`).
    pub duration_samples: u8,
    /// Physical interrupt pin (`Int1` = PB7, `Int2` = PB6).
    pub pin: IntPin,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct HighGConfig {
    /// Acceleration threshold in milli-g (clamped to current `AccelRange`).
    pub threshold_mg: u16,
    /// Duration in milliseconds (`2.5 ms` steps, `3..=640`).
    pub duration_ms: u16,
    /// Physical interrupt pin (`Int1` = PB7, `Int2` = PB6).
    pub pin: IntPin,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct LowGConfig {
    /// Freefall threshold in milli-g (`1 LSB = 7.81 mg`, `0..=1992`).
    pub threshold_mg: u16,
    /// Duration in milliseconds (`2.5 ms` steps, `3..=640`).
    pub duration_ms: u16,
    /// Physical interrupt pin (`Int1` = PB7, `Int2` = PB6).
    pub pin: IntPin,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct NoMotionConfig {
    /// Slope threshold in milli-g (clamped to current `AccelRange`).
    pub threshold_mg: u16,
    /// Duration in seconds (`1..=80` approx; quantized to BMI160 `slo_nomo_dur`).
    pub duration_s: u16,
    /// Physical interrupt pin (`Int1` = PB7, `Int2` = PB6).
    pub pin: IntPin,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct IntStatus {
    pub any_motion: bool,
    pub high_g: bool,
    pub low_g: bool,
    pub no_motion: bool,
    pub raw: [u8; 2],
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct ImuSample {
    pub acc_raw: [i16; 3],
    pub gyr_raw: [i16; 3],
    /// Accelerometer in milli-g `[x, y, z]`.
    pub acc_mg: [i32; 3],
    /// Gyroscope in milli-degrees-per-second `[x, y, z]`.
    pub gyr_mdps: [i32; 3],
}

pub struct Bmi160<'d> {
    i2c: I2c<'d, Blocking, Master>,
    addr: u8,
    accel_range: AccelRange,
    gyro_range: GyroRange,
}

#[allow(dead_code)]
impl<'d> Bmi160<'d> {
    pub fn new(i2c: I2c<'d, Blocking, Master>, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            accel_range: AccelRange::G2,
            gyro_range: GyroRange::Dps2000,
        }
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.i2c.blocking_write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    pub fn read_regs(&mut self, start_reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.i2c.blocking_write_read(self.addr, &[start_reg], buf)?;
        Ok(())
    }

    pub fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.i2c.blocking_write(self.addr, &[reg, val])?;
        Ok(())
    }

    fn update_reg(&mut self, reg: u8, set_bits: u8, clear_bits: u8) -> Result<(), Error> {
        let val = (self.read_reg(reg)? & !clear_bits) | set_bits;
        self.write_reg(reg, val)
    }

    pub fn chip_id(&mut self) -> Result<u8, Error> {
        self.read_reg(reg::CHIP_ID)
    }

    pub fn err_reg(&mut self) -> Result<u8, Error> {
        self.read_reg(reg::ERR_REG)
    }

    pub fn pmu_status(&mut self) -> Result<u8, Error> {
        self.read_reg(reg::PMU_STATUS)
    }

    pub fn status(&mut self) -> Result<u8, Error> {
        self.read_reg(reg::STATUS)
    }

    /// Soft-resets the BMI160, verifies `CHIP_ID`, powers up accel (and optionally gyro),
    /// and applies range/ODR settings.
    pub async fn init(
        &mut self,
        accel_range: AccelRange,
        accel_odr: AccelOdr,
        enable_gyro: bool,
    ) -> Result<(), Error> {
        let id = self.chip_id()?;
        if id != BMI160_CHIP_ID {
            return Err(Error::WrongChipId(id));
        }

        self.write_reg(reg::CMD, cmd::SOFT_RESET)?;
        Timer::after_millis(10).await;

        let id_after = self.chip_id()?;
        if id_after != BMI160_CHIP_ID {
            return Err(Error::WrongChipId(id_after));
        }

        self.write_reg(reg::CMD, cmd::ACC_PMU_NORMAL)?;
        Timer::after_millis(10).await;

        if enable_gyro {
            self.write_reg(reg::CMD, cmd::GYR_PMU_NORMAL)?;
            Timer::after_millis(85).await;
        }

        let pmu = self.pmu_status()?;
        let acc_pmu = (pmu >> 4) & 0x03;
        let gyr_pmu = (pmu >> 2) & 0x03;
        if acc_pmu != 0b01 || (enable_gyro && gyr_pmu != 0b01) {
            return Err(Error::PmuNotReady(pmu));
        }

        self.set_accel_config(accel_range, accel_odr)?;
        if enable_gyro {
            self.write_reg(reg::GYR_CONF, 0x28)?; // 100 Hz, normal BWP
            self.write_reg(reg::GYR_RANGE, GyroRange::Dps2000 as u8)?;
            self.gyro_range = GyroRange::Dps2000;
        }

        Timer::after_millis(15).await;

        let err = self.err_reg()?;
        if err != 0 {
            return Err(Error::DeviceError(err));
        }

        Ok(())
    }

    pub fn set_accel_config(&mut self, range: AccelRange, odr: AccelOdr) -> Result<(), Error> {
        self.write_reg(reg::ACC_CONF, 0x20 | (odr as u8))?;
        self.write_reg(reg::ACC_RANGE, range.reg_val())?;
        self.accel_range = range;
        Ok(())
    }

    pub async fn set_gyro_enabled(&mut self, enable: bool, range: GyroRange) -> Result<(), Error> {
        if enable {
            self.write_reg(reg::CMD, cmd::GYR_PMU_NORMAL)?;
            Timer::after_millis(85).await;
            self.write_reg(reg::GYR_CONF, 0x28)?;
            self.write_reg(reg::GYR_RANGE, range as u8)?;
            self.gyro_range = range;
        } else {
            self.write_reg(reg::CMD, cmd::GYR_PMU_SUSPEND)?;
            Timer::after_millis(5).await;
        }
        Ok(())
    }

    /// Configures `INT1` (PB7) and `INT2` (PB6) as active-high push-pull outputs with `latch`.
    pub fn configure_int_pins(&mut self, latch: IntLatch) -> Result<(), Error> {
        self.write_reg(reg::INT_OUT_CTRL, 0xAA)?;
        self.write_reg(reg::INT_LATCH, latch as u8)
    }

    pub fn enable_any_motion(&mut self, cfg: AnyMotionConfig) -> Result<(), Error> {
        let th = ((cfg.threshold_mg as u32 * 1000) / self.accel_range.any_motion_ug_per_lsb())
            .min(255) as u8;
        let dur = cfg.duration_samples.saturating_sub(1).min(3);

        self.update_reg(reg::INT_MOTION_0, dur, 0x03)?;
        self.write_reg(reg::INT_MOTION_1, th)?;
        self.map_int(cfg.pin, 0x04, 0x00)?;
        self.update_reg(reg::INT_EN_0, 0x07, 0x00)
    }

    pub fn disable_any_motion(&mut self) -> Result<(), Error> {
        self.update_reg(reg::INT_EN_0, 0x00, 0x07)?;
        self.unmap_int(0x04)
    }

    pub fn enable_high_g(&mut self, cfg: HighGConfig) -> Result<(), Error> {
        let th = ((cfg.threshold_mg as u32 * 1000) / self.accel_range.high_g_ug_per_lsb()).min(255)
            as u8;
        let dur = ((cfg.duration_ms as u32 * 10) / 25)
            .saturating_sub(1)
            .min(255) as u8;

        self.write_reg(reg::INT_LOWHIGH_3, dur)?;
        self.write_reg(reg::INT_LOWHIGH_4, th)?;
        self.map_int(cfg.pin, 0x02, 0x00)?;
        self.update_reg(reg::INT_EN_1, 0x07, 0x00)
    }

    pub fn disable_high_g(&mut self) -> Result<(), Error> {
        self.update_reg(reg::INT_EN_1, 0x00, 0x07)?;
        self.unmap_int(0x02)
    }

    pub fn enable_low_g(&mut self, cfg: LowGConfig) -> Result<(), Error> {
        let th = ((cfg.threshold_mg as u32 * 1000) / 7_810).min(255) as u8;
        let dur = ((cfg.duration_ms as u32 * 10) / 25)
            .saturating_sub(1)
            .min(255) as u8;

        self.write_reg(reg::INT_LOWHIGH_0, dur)?;
        self.write_reg(reg::INT_LOWHIGH_1, th)?;
        self.map_int(cfg.pin, 0x01, 0x00)?;
        self.update_reg(reg::INT_EN_1, 0x08, 0x00)
    }

    pub fn disable_low_g(&mut self) -> Result<(), Error> {
        self.update_reg(reg::INT_EN_1, 0x00, 0x08)?;
        self.unmap_int(0x01)
    }

    pub fn enable_no_motion(&mut self, cfg: NoMotionConfig) -> Result<(), Error> {
        let th = ((cfg.threshold_mg as u32 * 1000) / self.accel_range.any_motion_ug_per_lsb())
            .min(255) as u8;
        // 0..15 -> (n + 1) * 1.28 s (up to 20.48 s); higher values step by 5.12 s / 10.24 s
        let dur_code = if cfg.duration_s <= 20 {
            ((cfg.duration_s as u32 * 100) / 128)
                .saturating_sub(1)
                .min(15) as u8
        } else {
            (16 + ((cfg.duration_s - 20) / 5)).min(31) as u8
        };

        self.update_reg(reg::INT_MOTION_0, dur_code << 2, 0xFC)?;
        self.write_reg(reg::INT_MOTION_2, th)?;
        self.map_int(cfg.pin, 0x08, 0x00)?;
        self.update_reg(reg::INT_EN_2, 0x0F, 0x00)
    }

    pub fn disable_no_motion(&mut self) -> Result<(), Error> {
        self.update_reg(reg::INT_EN_2, 0x00, 0x0F)?;
        self.unmap_int(0x08)
    }

    pub fn disable_all_interrupts(&mut self) -> Result<(), Error> {
        self.write_reg(reg::INT_EN_0, 0x00)?;
        self.write_reg(reg::INT_EN_1, 0x00)?;
        self.write_reg(reg::INT_EN_2, 0x00)?;
        self.write_reg(reg::INT_MAP_0, 0x00)?;
        self.write_reg(reg::INT_MAP_2, 0x00)?;
        self.clear_latched_interrupts()
    }

    pub fn read_int_status(&mut self) -> Result<IntStatus, Error> {
        let mut raw = [0u8; 2];
        self.read_regs(reg::INT_STATUS_0, &mut raw)?;
        Ok(IntStatus {
            any_motion: (raw[0] & 0x04) != 0,
            high_g: (raw[0] & 0x02) != 0,
            low_g: (raw[0] & 0x01) != 0,
            no_motion: (raw[1] & 0x80) != 0,
            raw,
        })
    }

    pub fn clear_latched_interrupts(&mut self) -> Result<(), Error> {
        self.write_reg(reg::CMD, cmd::INT_RESET)
    }

    fn map_int(&mut self, pin: IntPin, set_mask: u8, clear_mask: u8) -> Result<(), Error> {
        match pin {
            IntPin::Int1 => {
                self.update_reg(reg::INT_MAP_2, 0x00, set_mask)?;
                self.update_reg(reg::INT_MAP_0, set_mask, clear_mask)
            }
            IntPin::Int2 => {
                self.update_reg(reg::INT_MAP_0, 0x00, set_mask)?;
                self.update_reg(reg::INT_MAP_2, set_mask, clear_mask)
            }
        }
    }

    fn unmap_int(&mut self, mask: u8) -> Result<(), Error> {
        self.update_reg(reg::INT_MAP_0, 0x00, mask)?;
        self.update_reg(reg::INT_MAP_2, 0x00, mask)
    }

    /// Reads accelerometer `[x, y, z]` in milli-g using the configured `AccelRange`.
    pub fn read_accel_mg(&mut self) -> Result<[i32; 3], Error> {
        let mut buf = [0u8; 6];
        self.read_regs(reg::DATA_ACC_X_L, &mut buf)?;
        let ax = i16::from_le_bytes([buf[0], buf[1]]);
        let ay = i16::from_le_bytes([buf[2], buf[3]]);
        let az = i16::from_le_bytes([buf[4], buf[5]]);
        let lsb = self.accel_range.lsb_per_g();
        Ok([
            (ax as i32 * 1000) / lsb,
            (ay as i32 * 1000) / lsb,
            (az as i32 * 1000) / lsb,
        ])
    }

    /// Burst-reads gyro (0x0C..0x11) and accel (0x12..0x17) in a single I2C transaction.
    pub fn read_sample(&mut self) -> Result<ImuSample, Error> {
        let mut buf = [0u8; 12];
        self.read_regs(reg::DATA_GYR_X_L, &mut buf)?;

        let gx = i16::from_le_bytes([buf[0], buf[1]]);
        let gy = i16::from_le_bytes([buf[2], buf[3]]);
        let gz = i16::from_le_bytes([buf[4], buf[5]]);
        let ax = i16::from_le_bytes([buf[6], buf[7]]);
        let ay = i16::from_le_bytes([buf[8], buf[9]]);
        let az = i16::from_le_bytes([buf[10], buf[11]]);

        let acc_lsb = self.accel_range.lsb_per_g();
        let acc_mg = [
            (ax as i32 * 1000) / acc_lsb,
            (ay as i32 * 1000) / acc_lsb,
            (az as i32 * 1000) / acc_lsb,
        ];

        let gyr_tenths = self.gyro_range.tenths_lsb_per_dps();
        let gyr_mdps = [
            (gx as i32 * 10_000) / gyr_tenths,
            (gy as i32 * 10_000) / gyr_tenths,
            (gz as i32 * 10_000) / gyr_tenths,
        ];

        Ok(ImuSample {
            acc_raw: [ax, ay, az],
            gyr_raw: [gx, gy, gz],
            acc_mg,
            gyr_mdps,
        })
    }

    /// Reads die temperature in milli-degrees Celsius, or `None` if invalid (`0x8000`).
    pub fn read_temperature_mc(&mut self) -> Result<Option<i32>, Error> {
        let mut buf = [0u8; 2];
        self.read_regs(reg::TEMPERATURE_0, &mut buf)?;
        let raw = i16::from_le_bytes(buf);
        if raw == i16::MIN {
            return Ok(None);
        }
        Ok(Some(23_000 + (raw as i32 * 1000) / 512))
    }

    /// Drives INT1 and INT2 statically as push-pull outputs for board pin discovery.
    pub fn drive_int_pins(&mut self, int1_high: bool, int2_high: bool) -> Result<(), Error> {
        self.write_reg(reg::INT_EN_0, 0x00)?;
        self.write_reg(reg::INT_EN_1, 0x00)?;
        self.write_reg(reg::INT_EN_2, 0x00)?;
        self.write_reg(reg::INT_MAP_0, 0x00)?;
        self.write_reg(reg::INT_MAP_2, 0x00)?;
        self.write_reg(reg::INT_LATCH, 0x00)?;

        let mut out_ctrl = 0x80 | 0x08;
        if !int1_high {
            out_ctrl |= 0x02;
        }
        if !int2_high {
            out_ctrl |= 0x20;
        }
        self.write_reg(reg::INT_OUT_CTRL, out_ctrl)
    }

    pub fn disable_int_pins(&mut self) -> Result<(), Error> {
        self.write_reg(reg::INT_OUT_CTRL, 0x00)
    }
}

#[allow(dead_code)]
pub struct IntProbePins {
    pub pa0: Peri<'static, peripherals::PA0>,
    pub pa1: Peri<'static, peripherals::PA1>,
    pub pa2: Peri<'static, peripherals::PA2>,
    pub pa3: Peri<'static, peripherals::PA3>,
    pub pa4: Peri<'static, peripherals::PA4>,
    pub pa5: Peri<'static, peripherals::PA5>,
    pub pa6: Peri<'static, peripherals::PA6>,
    pub pa8: Peri<'static, peripherals::PA8>,
    pub pa9: Peri<'static, peripherals::PA9>,
    pub pa10: Peri<'static, peripherals::PA10>,
    pub pa11: Peri<'static, peripherals::PA11>,
    pub pa12: Peri<'static, peripherals::PA12>,
    pub pa15: Peri<'static, peripherals::PA15>,
    pub pb2: Peri<'static, peripherals::PB2>,
    pub pb3: Peri<'static, peripherals::PB3>,
    pub pb6: Peri<'static, peripherals::PB6>,
    pub pb7: Peri<'static, peripherals::PB7>,
    pub pb10: Peri<'static, peripherals::PB10>,
    pub pc0: Peri<'static, peripherals::PC0>,
    pub pc1: Peri<'static, peripherals::PC1>,
    pub pc2: Peri<'static, peripherals::PC2>,
    pub pc3: Peri<'static, peripherals::PC3>,
    pub pc5: Peri<'static, peripherals::PC5>,
    pub pc9: Peri<'static, peripherals::PC9>,
    pub pc10: Peri<'static, peripherals::PC10>,
    pub pc11: Peri<'static, peripherals::PC11>,
    pub pc12: Peri<'static, peripherals::PC12>,
    pub pc13: Peri<'static, peripherals::PC13>,
    pub pd2: Peri<'static, peripherals::PD2>,
}

#[allow(dead_code)]
async fn probe_interrupt_pins(imu: &mut Bmi160<'_>, pins: IntProbePins) {
    info!("=== Probing BMI160 INT1 / INT2 connections ===");

    let mut candidates: [(&'static str, Flex<'static>); 29] = [
        ("PA0", Flex::new(pins.pa0)),
        ("PA1", Flex::new(pins.pa1)),
        ("PA2", Flex::new(pins.pa2)),
        ("PA3", Flex::new(pins.pa3)),
        ("PA4", Flex::new(pins.pa4)),
        ("PA5", Flex::new(pins.pa5)),
        ("PA6", Flex::new(pins.pa6)),
        ("PA8", Flex::new(pins.pa8)),
        ("PA9", Flex::new(pins.pa9)),
        ("PA10", Flex::new(pins.pa10)),
        ("PA11", Flex::new(pins.pa11)),
        ("PA12", Flex::new(pins.pa12)),
        ("PA15", Flex::new(pins.pa15)),
        ("PB2", Flex::new(pins.pb2)),
        ("PB3", Flex::new(pins.pb3)),
        ("PB6", Flex::new(pins.pb6)),
        ("PB7", Flex::new(pins.pb7)),
        ("PB10", Flex::new(pins.pb10)),
        ("PC0", Flex::new(pins.pc0)),
        ("PC1", Flex::new(pins.pc1)),
        ("PC2", Flex::new(pins.pc2)),
        ("PC3", Flex::new(pins.pc3)),
        ("PC5", Flex::new(pins.pc5)),
        ("PC9", Flex::new(pins.pc9)),
        ("PC10", Flex::new(pins.pc10)),
        ("PC11", Flex::new(pins.pc11)),
        ("PC12", Flex::new(pins.pc12)),
        ("PC13", Flex::new(pins.pc13)),
        ("PD2", Flex::new(pins.pd2)),
    ];

    for (_, pin) in candidates.iter_mut() {
        pin.set_as_input(Pull::Down);
    }

    let mut patterns = [0u8; 29];
    let states = [(false, false), (true, false), (false, true), (true, true)];

    for (step, &(int1_hi, int2_hi)) in states.iter().enumerate() {
        if let Err(e) = imu.drive_int_pins(int1_hi, int2_hi) {
            error!("Failed to drive INT pins: {:?}", e);
            return;
        }
        Timer::after_millis(5).await;

        for (idx, (_, pin)) in candidates.iter().enumerate() {
            if pin.is_high() {
                patterns[idx] |= 1 << step;
            }
        }
    }

    let _ = imu.disable_int_pins();

    let mut found_any = false;
    for (idx, (name, _)) in candidates.iter().enumerate() {
        match patterns[idx] {
            0b1010 => {
                info!("  [MATCH] {} follows BMI160 INT1!", name);
                found_any = true;
            }
            0b1100 => {
                info!("  [MATCH] {} follows BMI160 INT2!", name);
                found_any = true;
            }
            0b0101 => {
                info!("  [MATCH] {} is inverted BMI160 INT1!", name);
                found_any = true;
            }
            0b0011 => {
                info!("  [MATCH] {} is inverted BMI160 INT2!", name);
                found_any = true;
            }
            0b0000 => {}
            other => {
                info!("  [OTHER] {} pattern=0b{:04b}", name, other);
            }
        }
    }

    if !found_any {
        warn!("  No free GPIO pin matched BMI160 INT1 or INT2.");
    }
}

/// Runs a bring-up and live-sampling test for the BMI160 on I2C1 (PB8=SCL, PB9=SDA).
pub async fn run_bringup_test(
    i2c1: Peri<'static, peripherals::I2C1>,
    pb8: Peri<'static, peripherals::PB8>,
    pb9: Peri<'static, peripherals::PB9>,
) {
    info!("=== BMI160 Bring-Up Test (I2C1 PB8/PB9 @ 0x68) ===");

    let mut cfg = I2cConfig::default();
    cfg.frequency = khz(100);
    cfg.timeout = Duration::from_millis(20);

    let i2c = I2c::new_blocking(i2c1, pb8, pb9, cfg);
    let mut imu = Bmi160::new(i2c, BMI160_ADDR);

    match imu.init(AccelRange::G2, AccelOdr::Hz100, true).await {
        Ok(()) => {
            let pmu = imu.pmu_status().unwrap_or(0);
            let status = imu.status().unwrap_or(0);
            info!(
                "BMI160 init OK! CHIP_ID=0x{:02X}, PMU_STATUS=0x{:02X}, STATUS=0x{:02X}",
                BMI160_CHIP_ID, pmu, status
            );
        }
        Err(e) => {
            error!("BMI160 init failed: {:?}", e);
            return;
        }
    }

    match imu.read_temperature_mc() {
        Ok(Some(mc)) => info!(
            "BMI160 die temp: {}.{:02} C",
            mc / 1000,
            (mc.abs() % 1000) / 10
        ),
        Ok(None) => warn!("BMI160 die temp: invalid"),
        Err(e) => error!("BMI160 temp read failed: {:?}", e),
    }

    for i in 0..20u8 {
        match imu.read_sample() {
            Ok(s) => {
                let [ax, ay, az] = s.acc_mg;
                let mag_sq = (ax as f32) * (ax as f32)
                    + (ay as f32) * (ay as f32)
                    + (az as f32) * (az as f32);
                let mag_mg = libm::sqrtf(mag_sq) as i32;
                info!(
                    "[#{:02}] ACC(mg)=[{}, {}, {}] |g|={}mg | GYR(mdps)=[{}, {}, {}]",
                    i, ax, ay, az, mag_mg, s.gyr_mdps[0], s.gyr_mdps[1], s.gyr_mdps[2]
                );
            }
            Err(e) => {
                error!("[#{:02}] sample read failed: {:?}", i, e);
            }
        }
        Timer::after_millis(100).await;
    }

    info!("=== BMI160 Bring-Up Test Complete ===");
}
