#![no_std]
#![no_main]

#[allow(dead_code)]
mod nfc_config;
#[allow(dead_code)]
mod nrf_boot;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};
use gps_tag_drivers::{bmi160, board, buzzer, modem, st25dv};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut b = board::init();

    // MBN52832 is held in reset (PC2 HIGH) unless the `nrf-fw` feature is enabled.
    #[cfg(feature = "nrf-fw")]
    nrf_boot::ensure_firmware(&mut b.ble, b.flash, nrf_boot::EMBEDDED_FIRMWARE)
        .await
        .unwrap();

    let _nfc = st25dv::run_bringup_test(b.nfc.scl, b.nfc.sda).await;

    bmi160::run_bringup_test(b.imu.i2c1, b.imu.scl, b.imu.sda).await;

    #[cfg(debug_assertions)]
    spawner.spawn(idle_task().unwrap());

    let (mut client, mut ppp_ch) = modem::init(&spawner, &mut b.modem_pins, b.modem_tx, b.modem_rx)
        .await
        .unwrap();

    b.buzzer.play(&buzzer::songs::NOKIA).await;
    spawner.spawn(blink_task(b.led).unwrap());

    // Bring up PPP on DLCI 2 while DLCI 1 remains active for AT/GPS
    if let Err(e) = modem::ppp::configure_apn(&mut client, "mdata.net.au").await {
        warn!("APN config failed: {:?}", e);
    }
    if modem::ppp::wait_for_lte_registration(&mut client, Duration::from_secs(30))
        .await
        .is_ok()
    {
        modem::gps::ensure_xtra(&mut client, "mdata.net.au").await;
        if let Err(e) = modem::gps::enable(&mut client).await {
            error!("Failed to enable GNSS: {:?}", e);
        }
        if modem::dial_ppp(&mut ppp_ch).await.is_ok() {
            let seed = embassy_time::Instant::now().as_ticks() ^ 0x1234_5678_9abc_def0;
            let stack = modem::ppp::start(&spawner, ppp_ch, seed);
            modem::ppp::test_connectivity(stack).await;
            modem::log_status(&mut client).await;
        }
    } else if let Err(e) = modem::gps::enable(&mut client).await {
        error!("Failed to enable GNSS: {:?}", e);
    }

    // if modem::sms::init(&mut client).await.is_ok() {
    //     match modem::sms::send(&mut client, "0487373877", "Hello From GPSTAG").await {
    //         Ok(_) => info!("Sent SMS"),
    //         Err(e) => error!("Failed to send SMS: {:?}", e),
    //     }
    // }

    let gnss_start = embassy_time::Instant::now();
    let mut got_first_fix = false;

    loop {
        match modem::gps::get_fix(&mut client).await {
            Ok(fix) => {
                if !got_first_fix {
                    got_first_fix = true;
                    info!("TTFF: {}s", gnss_start.elapsed().as_secs());
                }
                info!(
                    "Fix({:?}): lat={}, lon={}, alt={}m, hdop={}, sats={}",
                    fix.fix_type,
                    fix.latitude,
                    fix.longitude,
                    fix.altitude_m,
                    fix.hdop,
                    fix.satellites,
                );
            }
            Err(modem::gps::GpsError::NotFixed) => {
                info!("GPS acquiring fix ({}s)...", gnss_start.elapsed().as_secs());
            }
            Err(e) => {
                warn!("GPS error ({}s): {:?}", gnss_start.elapsed().as_secs(), e);
            }
        }
        Timer::after_secs(2).await;
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

#[cfg(debug_assertions)]
#[embassy_executor::task]
async fn idle_task() {
    loop {
        Timer::after_micros(200).await;
        cortex_m::asm::nop();
    }
}
