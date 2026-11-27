//! Initializes the Bosch BMI160 6-axis IMU on `I2C1` (`PB8`/`PB9` @ `0x68`), prints die
//! temperature and a few live samples, then configures hardware motion interrupts on `INT1`
//! (`PB7`) and `INT2` (`PB6`) to wake the STM32 asynchronously via `EXTI`.
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example imu_motion_wake
//! ```

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::khz;
use embassy_time::{Duration, Timer};
use gps_tag_drivers::bmi160::{
    AccelOdr, AccelRange, AnyMotionConfig, BMI160_ADDR, Bmi160, HighGConfig, IntLatch, IntPin,
};
use gps_tag_drivers::board;

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut b = board::init();

    let mut cfg = I2cConfig::default();
    cfg.frequency = khz(100);
    cfg.timeout = Duration::from_millis(20);

    let i2c = I2c::new_blocking(b.imu.i2c1, b.imu.scl, b.imu.sda, cfg);
    let mut imu = Bmi160::new(i2c, BMI160_ADDR);

    // Power up accelerometer at ±4 g / 50 Hz (gyro disabled for lower power).
    if let Err(e) = imu.init(AccelRange::G4, AccelOdr::Hz50, false).await {
        error!("BMI160 init failed: {:?}", e);
        loop {
            Timer::after_secs(5).await;
        }
    }

    if let Ok(Some(mc)) = imu.read_temperature_mc() {
        info!("BMI160 temp: {}.{:02} C", mc / 1000, (mc.abs() % 1000) / 10);
    }

    if let Ok(acc) = imu.read_accel_mg() {
        info!("Initial accel (mg): {:?}", acc);
    }

    // Configure INT1 (PB7) for gentle motion wake and INT2 (PB6) for sharp shock/tap detection.
    if let Err(e) = imu.configure_int_pins(IntLatch::Ms320) {
        warn!("configure_int_pins failed: {:?}", e);
    }
    let _ = imu.enable_any_motion(AnyMotionConfig {
        threshold_mg: 150,
        duration_samples: 2,
        pin: IntPin::Int1,
    });
    let _ = imu.enable_high_g(HighGConfig {
        threshold_mg: 2_500,
        duration_ms: 15,
        pin: IntPin::Int2,
    });

    info!("Sleeping on BMI160 EXTI interrupts (move or tap the board)...");
    loop {
        match select(
            b.imu.int1.wait_for_rising_edge(),
            b.imu.int2.wait_for_rising_edge(),
        )
        .await
        {
            Either::First(()) => {
                b.led.set_high();
                let st = imu.read_int_status();
                let acc = imu.read_accel_mg();
                info!("INT1 wake (any-motion): status={:?}, acc_mg={:?}", st, acc);
                Timer::after_millis(100).await;
                b.led.set_low();
            }
            Either::Second(()) => {
                b.led.set_high();
                let st = imu.read_int_status();
                let acc = imu.read_accel_mg();
                info!("INT2 wake (high-G): status={:?}, acc_mg={:?}", st, acc);
                Timer::after_millis(250).await;
                b.led.set_low();
            }
        }
    }
}
