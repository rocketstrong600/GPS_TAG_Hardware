//! Demonstrates GNSS-aware low-power operation on the Quectel EC21-AU without cold-booting:
//!
//! 1. Boots the EC21-AU once (`460800 bps` CMUX) and injects gpsOneXTRA3 assistance data.
//! 2. Enables DTR slow-clock sleep (`AT+QSCLK=1`) and periodic GNSS with Qualcomm DPO
//!    (`fixrate = 5 s`).
//! 3. Uses [`power::poll_fix_and_sleep`] so the LTE/AP domain sleeps in DRX between polls
//!    (~100 ms wake latency).
//! 4. After acquiring a fix, demonstrates [`power::enter_standby`] (~1.5–2.5 mA with ephemeris
//!    and XTRA retained in RAM) followed by [`power::resume_with_gnss`] (fast Hot Start).
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example modem_gps_low_power
//! ```

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use gps_tag_drivers::board;
use gps_tag_drivers::modem::{self, gps, power, ppp};

use {defmt_rtt as _, panic_probe as _};

const APN: &str = "mdata.net.au";

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut b = board::init();

    let (mut client, _ppp_ch) = modem::init(&spawner, &mut b.modem_pins, b.modem_tx, b.modem_rx)
        .await
        .unwrap();

    let _ = ppp::configure_apn(&mut client, APN).await;
    if ppp::wait_for_lte_registration(&mut client, Duration::from_secs(30))
        .await
        .is_ok()
    {
        gps::ensure_xtra(&mut client, APN).await;
    }

    // Enable DTR-controlled slow-clock sleep and periodic 5 s GNSS with DPO.
    if let Err(e) = power::enable_slow_clock(&mut client, true).await {
        warn!("AT+QSCLK=1 failed: {:?}", e);
    }
    if let Err(e) = gps::enable_with_rate(&mut client, 5).await {
        error!("GNSS enable failed: {:?}", e);
    }

    // Allow slow-clock sleep immediately; poll_fix_and_sleep wakes via DTR for ~100 ms per poll.
    power::allow_sleep(&mut b.modem_pins);

    let start = Instant::now();
    loop {
        match power::poll_fix_and_sleep(&mut b.modem_pins, &mut client).await {
            Ok(fix) => {
                info!(
                    "Fix({:?}) lat={}, lon={}, hdop={}, sats={} (elapsed {}s)",
                    fix.fix_type,
                    fix.latitude,
                    fix.longitude,
                    fix.hdop,
                    fix.satellites,
                    start.elapsed().as_secs()
                );

                // Example: enter fast-wake standby for 15 s (GNSS paused via AT+QGPSEND,
                // ephemeris + XTRA retained in RAM for 1-5 s Hot Start; LTE stays registered).
                info!("Entering fast-wake standby for 15 s...");
                if power::wake(&mut b.modem_pins, &mut client).await.is_ok() {
                    let _ = power::enter_standby(&mut b.modem_pins, &mut client).await;
                }
                Timer::after_secs(15).await;

                info!("Resuming from standby (Hot Start)...");
                let _ = power::resume_with_gnss(&mut b.modem_pins, &mut client, 5).await;
                power::allow_sleep(&mut b.modem_pins);
            }
            Err(gps::GpsError::NotFixed) => {
                info!("Acquiring fix ({}s)...", start.elapsed().as_secs());
            }
            Err(e) => warn!("GNSS error: {:?}", e),
        }
        Timer::after_secs(5).await;
    }
}
