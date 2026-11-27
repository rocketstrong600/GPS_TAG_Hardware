//! Receives incoming SMS messages on the Quectel EC21-AU and writes the SMS text onto the
//! ST25DV16K NFC tag as a standard NDEF UTF-8 Text record (scannable by any phone).
//!
//! Also demonstrates [`gps_tag_drivers::modem::status`] for reading battery percentage/voltage
//! (`AT+CBC`), signal strength (`AT+CSQ`), LTE registration, operator, and network UTC time.
//!
//! 1. Brings up the ST25DV16K (`PC10`/`PC11`) and EC21-AU (`USART6` CMUX).
//! 2. Waits for LTE registration and logs a full [`gps_tag_drivers::modem::status::ModemStatus`]
//!    snapshot.
//! 3. Enables SMS text mode + `+CMTI` URCs and processes any unread SMS already in storage.
//! 4. Waits asynchronously for each new incoming SMS, writes its body to the ST25DV16K as an
//!    NDEF Text record (`lang = "en"`), chirps the buzzer, and deletes the processed SMS slot.
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example sms_to_nfc
//! ```

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use gps_tag_drivers::board;
use gps_tag_drivers::buzzer;
use gps_tag_drivers::modem::{self, ppp, sms, status};
use gps_tag_drivers::soft_i2c::SoftI2c;
use gps_tag_drivers::st25dv::ndef::{self, WriteRecord};
use gps_tag_drivers::st25dv::{self, St25dv};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut b = board::init();

    let Some(mut nfc) = st25dv::run_bringup_test(b.nfc.scl, b.nfc.sda).await else {
        error!("ST25DV16K not detected on PC10/PC11");
        loop {
            Timer::after_secs(5).await;
        }
    };

    let (mut client, _ppp_ch) = modem::init(&spawner, &mut b.modem_pins, b.modem_tx, b.modem_rx)
        .await
        .expect("EC21-AU init failed");

    if let Err(e) = ppp::wait_for_lte_registration(&mut client, Duration::from_secs(45)).await {
        warn!("LTE registration did not complete: {:?}", e);
    }

    let st = status::snapshot(&mut client).await;
    status::log(&st);

    sms::init(&mut client).await.expect("SMS init failed");
    let mut sms_rx = sms::subscribe().expect("URC subscribe failed");

    // Process any unread SMS that arrived while booting.
    while let Ok(Some(msg)) = sms::read_first_unread(&mut client).await {
        handle_sms(&mut nfc, &mut b.buzzer, &mut b.led, &msg).await;
        if let Err(e) = sms::delete(&mut client, msg.index).await {
            warn!("Failed to delete SMS #{}: {:?}", msg.index, e);
        }
    }

    info!("Waiting for incoming SMS... Send a text message to the SIM to update the NFC tag.");
    loop {
        match sms_rx.next_message(&mut client).await {
            Ok(msg) => {
                handle_sms(&mut nfc, &mut b.buzzer, &mut b.led, &msg).await;
                if let Err(e) = sms::delete(&mut client, msg.index).await {
                    warn!("Failed to delete SMS #{}: {:?}", msg.index, e);
                }
                if let Ok(batt) = status::battery(&mut client).await {
                    info!(
                        "Battery: {}% ({} mV, {:?})",
                        batt.percent, batt.voltage_mv, batt.state
                    );
                }
            }
            Err(e) => {
                warn!("Error reading incoming SMS: {:?}", e);
                Timer::after_secs(2).await;
            }
        }
    }
}

async fn handle_sms(
    nfc: &mut St25dv<SoftI2c<'static>>,
    buzzer: &mut buzzer::Buzzer<'static, embassy_stm32::peripherals::TIM3>,
    led: &mut embassy_stm32::gpio::Output<'static>,
    msg: &sms::SmsMessage,
) {
    info!(
        "SMS #{} from {} ({}): {}",
        msg.index,
        msg.sender.as_str(),
        msg.timestamp.as_str(),
        msg.body.as_str()
    );

    let mut frame = [0u8; ndef::MAX_FRAME_BYTES];
    let len = match ndef::encode_type5_records(
        &[WriteRecord::Text {
            lang: "en",
            text: msg.body.as_str(),
        }],
        &mut frame,
    ) {
        Ok(n) => n,
        Err(e) => {
            warn!("Failed to encode SMS as NDEF Text record: {:?}", e);
            return;
        }
    };

    // Wait briefly if a phone is currently resting on the NFC antenna before writing EEPROM.
    for _ in 0..20 {
        match nfc.read_dynamic_status() {
            Ok(st) if !st.field_on => break,
            _ => Timer::after_millis(150).await,
        }
    }

    led.set_high();
    match nfc.write_user_memory(0x0000, &frame[..len]) {
        Ok(()) => {
            info!(
                "Updated ST25DV16K NFC tag ({} bytes) with SMS text! Tap with your phone to read.",
                len
            );
            buzzer.play(&buzzer::songs::CHIRP).await;
        }
        Err(e) => {
            warn!("Failed to write SMS text to ST25DV16K EEPROM: {:?}", e);
        }
    }
    led.set_low();
}
