//! Demonstrates **interrupt-driven NDEF record rewrite protection** for the ST25DV16K using the
//! shared `GPO -> SCL` (`PC10` / `EXTI10`) interrupt line (**zero I2C polling while idle**).
//!
//! 1. Ensures Record 0 holds `https://example.com` via
//!    [`gps_tag_drivers::st25dv::St25dv::sync_leading_records`] (preserving any extra records
//!    already on the tag).
//! 2. Sleeps on `EXTI10` via [`gps_tag_drivers::st25dv::St25dv::protect_ndef_records_on_rf_write`].
//! 3. Whenever a phone writes to the tag:
//!    - If Record 0 still matches `https://example.com` (e.g. you added a 2nd Text or vCard
//!      record), leaves the tag untouched.
//!    - If Record 0 was modified or removed, restores Record 0 to `https://example.com` while
//!      preserving any trailing records (`Record 1+`).
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example nfc_gpo_protect
//! ```

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::Timer;
use gps_tag_drivers::board;
use gps_tag_drivers::st25dv::ndef::WriteRecord;
use gps_tag_drivers::st25dv::{self, DEFAULT_I2C_PASSWORD};

use {defmt_rtt as _, panic_probe as _};

const PROTECTED_URL: &str = "https://example.com";

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut b = board::init();

    let Some(mut nfc) = st25dv::run_bringup_test(b.nfc.scl, b.nfc.sda).await else {
        error!("ST25DV16K not detected on PC10/PC11");
        loop {
            Timer::after_secs(5).await;
        }
    };

    if let Err(e) = nfc.configure_gpo_rf_write(DEFAULT_I2C_PASSWORD) {
        warn!("configure_gpo_rf_write failed: {:?}", e);
    }

    let protected = [WriteRecord::Uri(PROTECTED_URL)];

    match nfc.sync_leading_records(&protected) {
        Ok(true) => info!("Provisioned protected Record 0 ({})", PROTECTED_URL),
        Ok(false) => info!("Protected Record 0 already intact ({})", PROTECTED_URL),
        Err(e) => error!("Initial sync_leading_records failed: {:?}", e),
    }
    nfc.clear_pending_rf_write();

    info!("Sleeping on EXTI10 (GPO->SCL). Try editing Record 0 or adding a 2nd record...");
    loop {
        b.led.set_high();
        let res = nfc
            .protect_ndef_records_on_rf_write(&mut b.nfc.exti10, &protected)
            .await;
        b.led.set_low();

        match res {
            Ok(true) => info!(
                "Record 0 tampered — restored {} (trailing records preserved)",
                PROTECTED_URL
            ),
            Ok(false) => info!("Record 0 intact — extra records preserved without rewrite"),
            Err(e) => warn!("protect_ndef_records_on_rf_write error: {:?}", e),
        }
        Timer::after_millis(200).await;
    }
}
