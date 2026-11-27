//! # `gps-tag-drivers`
//!
//! `no_std`, Embassy-based hardware drivers and board support package for the **STM32F412RG**
//! GPS Tag hardware platform.
//!
//! ## Hardware Overview
//!
//! | Subsystem | Part | Interface / Pins | Driver Module |
//! | --- | --- | --- | --- |
//! | **Board Init** | `STM32F412RGTx` | 16 MHz HSE, 32.768 kHz LSE, `TIM4` time driver | [`board`] |
//! | **Cellular + GNSS** | Quectel `EC21-AU` | `USART6` (`PC6`/`PC7`, DMA 4 KiB RX, `460800 bps` CMUX) + control GPIOs (`PB0`, `PB1`, `PB12`, `PB15`, `PC4`, `PC8`, `PA7`) | [`modem`] |
//! | **6-Axis IMU** | Bosch `BMI160` | `I2C1` (`PB8`/`PB9` @ `0x68`), `INT1` (`PB7`), `INT2` (`PB6`) | [`bmi160`] |
//! | **NFC Dynamic Tag** | ST `ST25DV16K` | Open-drain [`soft_i2c::SoftI2c`] (`PC10`=SCL+GPO/`EXTI10`, `PC11`=SDA) | [`st25dv`] |
//! | **Audio Buzzer** | Piezo / Speaker | `TIM3_CH1` PWM (`PB4`) | [`buzzer`] |
//! | **I2C Diagnostics** | — | Hardware `I2C1` / `I2C2` / `I2C3` bus scanner | [`i2c_scanner`] |
//!
//! ## Quick Start
//!
//! ```no_run
//! use embassy_executor::Spawner;
//! use gps_tag_drivers::{board, buzzer, st25dv};
//!
//! async fn run(spawner: Spawner) {
//!     let mut b = board::init();
//!     b.buzzer.play(&buzzer::songs::CHIRP).await;
//!
//!     if let Some(mut nfc) = st25dv::run_bringup_test(b.nfc.scl, b.nfc.sda).await {
//!         let _ = nfc.wait_for_rf_write(&mut b.nfc.exti10).await;
//!     }
//! }
//! ```
//!
//! ## Runnable Examples
//!
//! Flash any example directly onto the STM32F412RG board via ST-Link:
//!
//! ```bash
//! cargo run -p gps-tag-drivers --example buzzer_melody
//! cargo run -p gps-tag-drivers --example imu_motion_wake
//! cargo run -p gps-tag-drivers --example nfc_gpo_read
//! cargo run -p gps-tag-drivers --example nfc_gpo_protect
//! cargo run -p gps-tag-drivers --example nfc_vcard
//! cargo run -p gps-tag-drivers --example modem_gps_low_power
//! cargo run -p gps-tag-drivers --example modem_ppp_http
//! cargo run -p gps-tag-drivers --example modem_https
//! cargo run -p gps-tag-drivers --example modem_sse
//! cargo run -p gps-tag-drivers --example sms_to_nfc
//! ```

#![no_std]

pub mod bmi160;
pub mod board;
pub mod buzzer;
pub mod i2c_scanner;
pub mod modem;
pub mod soft_i2c;
pub mod st25dv;
