//! Multi-bus hardware I2C diagnostic scanner (`I2C1`, `I2C2`, `I2C3`) used during board
//! reverse-engineering and bring-up to detect external pull-ups, responding 7-bit addresses, and
//! common IMU/sensor `WHO_AM_I` registers.

use defmt::{info, warn};
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Input, Output, Pull};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals;
use embassy_stm32::time::khz;
use embassy_time::{Duration, Timer};

pub struct ScanPins {
    pub i2c1: Peri<'static, peripherals::I2C1>,
    pub i2c2: Peri<'static, peripherals::I2C2>,
    pub i2c3: Peri<'static, peripherals::I2C3>,
    pub pb6: Peri<'static, peripherals::PB6>,
    pub pb7: Peri<'static, peripherals::PB7>,
    pub pb8: Peri<'static, peripherals::PB8>,
    pub pb9: Peri<'static, peripherals::PB9>,
    pub pb10: Peri<'static, peripherals::PB10>,
    pub pb3: Peri<'static, peripherals::PB3>,
    pub pa8: Peri<'static, peripherals::PA8>,
    pub pc9: Peri<'static, peripherals::PC9>,
}

pub async fn run_i2c_scan(
    mut pins: ScanPins,
    uknwn_vreg: &mut Output<'_>,
    stm_out_2: &mut Output<'_>,
) {
    for (label, vreg_hi, out2_hi) in [
        ("PB14=LOW, PB13=LOW", false, false),
        ("PB14=HIGH, PB13=LOW", true, false),
        ("PB14=HIGH, PB13=HIGH", true, true),
    ] {
        info!("=== I2C Scan Pass: {} ===", label);
        if vreg_hi {
            uknwn_vreg.set_high();
        } else {
            uknwn_vreg.set_low();
        }
        if out2_hi {
            stm_out_2.set_high();
        } else {
            stm_out_2.set_low();
        }
        Timer::after_millis(100).await;

        // 1. Pull-down check: external I2C pull-ups (2.2k-10k) beat the STM32's ~40k pull-down
        check_pullups("I2C1 PB6/PB7", pins.pb6.reborrow(), pins.pb7.reborrow()).await;
        check_pullups("I2C1 PB8/PB9", pins.pb8.reborrow(), pins.pb9.reborrow()).await;
        check_pullups("I2C2 PB10/PB3", pins.pb10.reborrow(), pins.pb3.reborrow()).await;
        check_pullups("I2C3 PA8/PC9", pins.pa8.reborrow(), pins.pc9.reborrow()).await;

        let mut cfg = I2cConfig::default();
        cfg.frequency = khz(100);
        cfg.timeout = Duration::from_millis(20);

        // 2. Sweep all 4 hardware I2C pairs
        {
            let mut i2c = I2c::new_blocking(
                pins.i2c1.reborrow(),
                pins.pb6.reborrow(),
                pins.pb7.reborrow(),
                cfg,
            );
            scan_bus("I2C1 (PB6/PB7)", &mut i2c);
        }
        {
            let mut i2c = I2c::new_blocking(
                pins.i2c1.reborrow(),
                pins.pb8.reborrow(),
                pins.pb9.reborrow(),
                cfg,
            );
            scan_bus("I2C1 (PB8/PB9)", &mut i2c);
        }
        {
            let mut i2c = I2c::new_blocking(
                pins.i2c2.reborrow(),
                pins.pb10.reborrow(),
                pins.pb3.reborrow(),
                cfg,
            );
            scan_bus("I2C2 (PB10/PB3)", &mut i2c);
        }
        {
            let mut i2c = I2c::new_blocking(
                pins.i2c3.reborrow(),
                pins.pa8.reborrow(),
                pins.pc9.reborrow(),
                cfg,
            );
            scan_bus("I2C3 (PA8/PC9)", &mut i2c);
        }
    }

    info!("=== I2C Scan Complete ===");
}

async fn check_pullups<P1: embassy_stm32::gpio::Pin, P2: embassy_stm32::gpio::Pin>(
    name: &str,
    scl: Peri<'_, P1>,
    sda: Peri<'_, P2>,
) {
    let scl_in = Input::new(scl, Pull::Down);
    let sda_in = Input::new(sda, Pull::Down);
    Timer::after_millis(2).await;
    let scl_hi = scl_in.is_high();
    let sda_hi = sda_in.is_high();
    if scl_hi || sda_hi {
        info!(
            "  [Pull-up detected] {} -> SCL={}, SDA={}",
            name, scl_hi, sda_hi
        );
    }
}

fn scan_bus(name: &str, i2c: &mut I2c<'_, Blocking, Master>) {
    let mut found = 0u8;
    let mut dummy = [0u8; 1];

    for addr in 0x08u8..=0x77u8 {
        // 1-byte blocking_read is reliable on STM32F4 I2C v1 (avoids 0-len write quirks)
        if i2c.blocking_read(addr, &mut dummy).is_ok() {
            found += 1;
            info!("  [FOUND] {} @ 0x{:02X}", name, addr);
            probe_who_am_i(i2c, addr);
        }
    }

    if found == 0 {
        warn!("  [none]  {}", name);
    }
}

fn probe_who_am_i(i2c: &mut I2c<'_, Blocking, Master>, addr: u8) {
    // Common WHO_AM_I / CHIP_ID registers across ST, Bosch, InvenSense, Kionix, mCube
    for reg in [0x0Fu8, 0x00u8, 0x75u8, 0x0Du8, 0x3Bu8] {
        let mut val = [0u8; 1];
        if i2c.blocking_write_read(addr, &[reg], &mut val).is_ok() {
            let guess = identify_chip(reg, val[0]);
            info!("    reg 0x{:02X} = 0x{:02X} ({})", reg, val[0], guess);
        }
    }
}

fn identify_chip(reg: u8, val: u8) -> &'static str {
    match (reg, val) {
        (0x0F, 0x33) => "LIS3DH / LIS2DH12",
        (0x0F, 0x44) => "LIS2DW12",
        (0x0F, 0x69) | (0x0F, 0x6A) | (0x0F, 0x6C) => "LSM6DS3 / LSM6DSL / LSM6DSO",
        (0x0F, 0x15) | (0x0F, 0x35) => "KXTJ3 / KX022",
        (0x00, 0xD1) => "BMI160",
        (0x00, 0xFA) => "BMA253 / BMA255",
        (0x00, 0x90) => "BMA400",
        (0x75, 0x68) | (0x75, 0x70) | (0x75, 0x71) => "MPU6050 / MPU6500 / MPU9250",
        (0x0D, 0x1A) | (0x0D, 0x2A) => "MMA8451Q / MMA8452Q",
        (0x3B, 0xA0) | (0x3B, 0xA4) => "MC3416 / MC3479",
        _ => "unknown",
    }
}
