//! Demonstrates writing and reading a vCard 3.0 contact card (`text/vcard`, `TNF = 0x02`)
//! on the ST25DV16K Type 5 NFC tag.
//!
//! 1. Encodes a [`gps_tag_drivers::st25dv::ndef::ContactCard`] into a Type 5 NDEF EEPROM image
//!    and writes it to `0x0000..`.
//! 2. Reads the EEPROM back and decodes it with [`gps_tag_drivers::st25dv::ndef::decode_record`]
//!    to verify the round-trip.
//! 3. Sleeps on `EXTI10` (`GPO -> SCL`) so scanning or overwriting from a phone can be tested live.
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example nfc_vcard
//! ```

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::Timer;
use gps_tag_drivers::board;
use gps_tag_drivers::st25dv::ndef::{self, ContactCard, DecodedRecord, WriteRecord};
use gps_tag_drivers::st25dv::{self, DEFAULT_I2C_PASSWORD};

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

    let card = ContactCard {
        org: Some("GPS Tag Tracker"),
        phone: Some("+61400123456"),
        email: Some("tag@example.com"),
        url: Some("https://example.com"),
        note: Some("Scanned from ST25DV16K"),
        ..ContactCard::new("GPS Tag")
    };

    let mut frame = [0u8; ndef::MAX_FRAME_BYTES];
    let len = ndef::encode_type5_records(&[WriteRecord::Contact(card)], &mut frame)
        .expect("vCard fits in NDEF frame buffer");

    if let Err(e) = nfc.write_user_memory(0x0000, &frame[..len]) {
        error!("Failed to write vCard to EEPROM: {:?}", e);
    } else {
        info!(
            "Wrote vCard contact card ({} bytes) to EEPROM[0x0000..]",
            len
        );
    }

    info!("Reading back from EEPROM to verify round-trip:");
    log_all_records(&mut nfc);
    nfc.clear_pending_rf_write();

    info!(
        "Ready! Tap your phone to scan the contact card (or write a new record via NFC Tools)..."
    );
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
            Ok(DecodedRecord::Uri(uri)) => {
                info!("  [{}] URI: {}{}", idx, uri.prefix, uri.rest);
            }
            Ok(DecodedRecord::Text(txt)) => {
                info!("  [{}] Text ({}): {}", idx, txt.lang, txt.text);
            }
            Ok(DecodedRecord::VCard(vc)) => {
                info!("  [{}] vCard ({} bytes):", idx, vc.raw.len());
                info!("      name={:?} org={:?}", vc.name(), vc.org());
                info!("      phone={:?} email={:?}", vc.phone(), vc.email());
                info!("      url={:?} note={:?}", vc.url(), vc.note());
            }
            Ok(DecodedRecord::Other(rec)) => {
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
