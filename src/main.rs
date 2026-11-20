#![no_std]
#![no_main]

mod bmi160;
mod board;
mod buzzer;
#[allow(dead_code)]
mod i2c_scanner;
mod modem;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut b = board::init();

    bmi160::run_bringup_test(b.imu.i2c1, b.imu.scl, b.imu.sda).await;

    spawner.spawn(idle_task().unwrap());

    let (mut client, mut ppp_ch) =
        modem::init(&spawner, &mut b.modem_pins, b.modem_tx, b.modem_rx)
            .await
            .unwrap();

    b.buzzer.play(&buzzer::songs::NOKIA).await;
    spawner.spawn(blink_task(b.led).unwrap());

    // Bring up PPP on DLCI 2 while DLCI 1 remains active for AT/GPS
    if let Err(e) = modem::ppp::configure_apn(&mut client, "yesinternet").await {
        warn!("APN config failed: {:?}", e);
    }
    if modem::ppp::wait_for_lte_registration(&mut client, Duration::from_secs(30))
        .await
        .is_ok()
    {
        modem::gps::configure_xtra(&mut client).await;
        if let Err(e) = modem::gps::enable(&mut client).await {
            error!("Failed to enable GNSS: {:?}", e);
        }
        if modem::dial_ppp(&mut ppp_ch).await.is_ok() {
            let stack = modem::ppp::start(&spawner, ppp_ch, 0x1234_5678_9abc_def0);
            modem::ppp::test_connectivity(stack).await;
        }
    } else if let Err(e) = modem::gps::enable(&mut client).await {
        error!("Failed to enable GNSS: {:?}", e);
    }

    loop {
        match modem::gps::get_location(&mut client).await {
            Ok(response) => {
                info!(
                    "Lat:{},Long:{}",
                    str::from_utf8(&response.latitude).unwrap_or("0.0"),
                    str::from_utf8(&response.longitude).unwrap_or("0.0")
                );
            }
            Err(e) => {
                error!("{}", e);
            }
        }
        Timer::after_secs(30).await;
    }
}

#[embassy_executor::task]
async fn blink_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_secs(1).await;
        led.set_low();
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn idle_task() {
    loop {
        Timer::after_micros(200).await;
        cortex_m::asm::nop();
    }
}
