//! Demonstrates **read-on-interrupt** for the ST25DV16K using the shared `GPO -> SCL`
//! (`PC10` / `EXTI10`) interrupt line (**zero I2C polling while idle**).
//!
//! Sleeps on `EXTI10` via [`gps_tag_drivers::st25dv::St25dv::watch_rf_write`], wakes whenever
//! a phone writes to the tag, debounces multi-block writes without touching I2C, and iterates
//! every NDEF record using the generic per-record decoders in [`gps_tag_drivers::st25dv::ndef`].
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example nfc_gpo_read
//! ```

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::Timer;
use gps_tag_drivers::board;
use gps_tag_drivers::st25dv::{self, DEFAULT_I2C_PASSWORD, ndef};

use {defmt_rtt as _, panic_probe as _};

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

    log_all_records(&mut nfc);
    nfc.clear_pending_rf_write();

    info!("Sleeping on EXTI10 (GPO->SCL). Write to the tag with NFC Tools to read records...");
    loop {
        b.led.set_high();
        let res = nfc
            .watch_rf_write(&mut b.nfc.exti10, |dev| {
                log_all_records(dev);
                Ok(())
            })
            .await;
        b.led.set_low();

        if let Err(e) = res {
            warn!("watch_rf_write error: {:?}", e);
        }
        Timer::after_millis(200).await;
    }
}

fn log_all_records<I2C, E>(nfc: &mut st25dv::St25dv<I2C>)
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
    E: defmt::Format,
{
    let mut raw = [0u8; ndef::MAX_FRAME_BYTES];
    if nfc.read_user_memory(0x0000, &mut raw[..32]).is_err() {
        warn!("Failed to read EEPROM header");
        return;
    }
    let Ok(Some(needed)) = ndef::probe_ndef_frame_len(&raw[..32]) else {
        info!("Tag is empty or not NDEF-formatted");
        return;
    };
    let len = needed.min(ndef::MAX_FRAME_BYTES);
    if len > 32 && nfc.read_user_memory(32, &mut raw[32..len]).is_err() {
        warn!("Failed to read NDEF payload");
        return;
    }

    let Ok(records) = ndef::iter_type5_records(&raw[..len]) else {
        warn!("Malformed Type 5 NDEF TLV");
        return;
    };

    for (idx, item) in records.enumerate() {
        match item.and_then(ndef::decode_record) {
            Ok(ndef::DecodedRecord::Uri(uri)) => {
                info!("  [{}] URI: {}{}", idx, uri.prefix, uri.rest);
            }
            Ok(ndef::DecodedRecord::Text(txt)) => {
                info!("  [{}] Text ({}): {}", idx, txt.lang, txt.text);
            }
            Ok(ndef::DecodedRecord::VCard(vc)) => {
                info!(
                    "  [{}] vCard: name={:?} phone={:?} email={:?} org={:?}",
                    idx,
                    vc.name(),
                    vc.phone(),
                    vc.email(),
                    vc.org()
                );
            }
            Ok(ndef::DecodedRecord::Other(rec)) => {
                info!(
                    "  [{}] Raw (tnf={}, type_len={}, payload_len={})",
                    idx,
                    rec.tnf,
                    rec.rec_type.len(),
                    rec.payload.len()
                );
            }
            Err(e) => warn!("  [{}] Decode error: {:?}", idx, e),
        }
    }
}
