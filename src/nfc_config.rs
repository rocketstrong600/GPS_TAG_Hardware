//! Game-facing NFC tag layout helper built on top of [`gps_tag_drivers::st25dv`].
//!
//! Layout on the ST25DV16K Type 5 NDEF tag:
//! - **Slot 1:** Authoritative URL record owned by the STM32 (automatically restored via the
//!   ST25DV `GPO -> SCL` `EXTI10` interrupt whenever an NFC app overwrites it).
//! - **Slot 2:** UTF-8 Text config written by the NFC Tools app (also accepts a Text-only write
//!   at Slot 1 and promotes it to Slot 2 while restoring Slot 1's URL).

use defmt::Format;
use embassy_stm32::{Peri, peripherals};
use embedded_hal::i2c::I2c;
use gps_tag_drivers::soft_i2c::{SoftI2c, SoftI2cError};
use gps_tag_drivers::st25dv::ndef::{self, NdefError, WriteRecord};
use gps_tag_drivers::st25dv::{Error, St25dv, USER_MEM_BYTES};

/// Decoded Slot 1 URL and Text config lengths from [`read_tag`].
#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
#[allow(dead_code)]
pub struct TagRecords {
    /// Byte length of the UTF-8 URL copied into `url_buf` (only `Some` if Slot 1 is a URI record).
    pub url_len: Option<usize>,
    /// Byte length of the UTF-8 Text config copied into `text_buf` (prefers Slot 2+, falls back
    /// to Slot 1 if NFC Tools overwrote Slot 1 with a single Text record).
    pub text_len: Option<usize>,
    /// Total number of NDEF records found in the message.
    pub record_count: u8,
}

/// Outcome of [`sync_url_and_read_config`] or [`wait_and_sync_config`].
#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
#[allow(dead_code)]
pub struct SyncResult {
    /// Byte length of the UTF-8 Text config copied into the caller's `text_buf`, if present.
    pub text_len: Option<usize>,
    /// `true` if Slot 1 was missing or modified and `[expected_url, text]` was rewritten to EEPROM.
    pub url_restored: bool,
}

/// Reads Slot 1 URL (into `url_buf`) and the Text config (into `text_buf`, preferring Slot 2+
/// and falling back to Slot 1 if NFC Tools wrote a single Text record).
#[allow(dead_code)]
pub fn read_tag<I2C, E>(
    nfc: &mut St25dv<I2C>,
    url_buf: &mut [u8],
    text_buf: &mut [u8],
) -> Result<TagRecords, Error<E>>
where
    I2C: I2c<Error = E>,
{
    let mut raw = [0u8; ndef::MAX_FRAME_BYTES];
    let initial_len = 32usize;
    nfc.read_user_memory(0x0000, &mut raw[..initial_len])?;

    let Some(needed_len) = ndef::probe_ndef_frame_len(&raw[..initial_len]).map_err(Error::Ndef)?
    else {
        return Ok(TagRecords {
            url_len: None,
            text_len: None,
            record_count: 0,
        });
    };

    let read_len = needed_len.min(ndef::MAX_FRAME_BYTES).min(USER_MEM_BYTES);
    if read_len > initial_len {
        nfc.read_user_memory(initial_len as u16, &mut raw[initial_len..read_len])?;
    }

    let mut url_len: Option<usize> = None;
    let mut text_len: Option<usize> = None;
    let mut text_from_slot2 = false;
    let mut slot = 0u8;

    for item in ndef::iter_type5_records(&raw[..read_len]).map_err(Error::Ndef)? {
        let rec = item.map_err(Error::Ndef)?;

        if slot == 0 {
            if let Some(uri) = ndef::decode_uri_record(&rec).map_err(Error::Ndef)? {
                url_len = Some(uri.write_into(url_buf).map_err(Error::Ndef)?);
            }
        }

        if !text_from_slot2 {
            if let Some(txt) = ndef::decode_text_record(&rec).map_err(Error::Ndef)? {
                let bytes = txt.text.as_bytes();
                if bytes.len() > text_buf.len() {
                    return Err(Error::Ndef(NdefError::BufferTooSmall));
                }
                text_buf[..bytes.len()].copy_from_slice(bytes);
                text_len = Some(bytes.len());
                if slot >= 1 {
                    text_from_slot2 = true;
                }
            }
        }

        slot = slot.saturating_add(1);
    }

    Ok(TagRecords {
        url_len,
        text_len,
        record_count: slot,
    })
}

/// Encodes and writes `[Slot 1: url, Slot 2: optional text]` to User EEPROM (`0x0000..`).
///
/// Returns [`Error::RfBusy`] if an RF phone field is currently present on the antenna.
#[allow(dead_code)]
pub fn write_tag<I2C, E>(
    nfc: &mut St25dv<I2C>,
    url: &str,
    text: Option<&str>,
) -> Result<(), Error<E>>
where
    I2C: I2c<Error = E>,
{
    if nfc.read_dynamic_status()?.field_on {
        return Err(Error::RfBusy);
    }
    let mut frame = [0u8; ndef::MAX_FRAME_BYTES];
    let len = match text {
        Some(t) => ndef::encode_type5_records(
            &[
                WriteRecord::Uri(url),
                WriteRecord::Text {
                    lang: "en",
                    text: t,
                },
            ],
            &mut frame,
        ),
        None => ndef::encode_type5_records(&[WriteRecord::Uri(url)], &mut frame),
    }
    .map_err(Error::Ndef)?;
    nfc.write_user_memory(0x0000, &frame[..len])
}

/// One-shot sync (call once at boot to read any offline NFC Tools writes and ensure Slot 1 holds
/// `expected_url`):
/// 1. Reads current tag records (treating an unformatted/corrupted tag as empty).
/// 2. Copies any Text config (Slot 2+, or Slot 1 fallback) into `text_buf`.
/// 3. If Slot 1 does not match `expected_url`, rewrites `[expected_url, text]` to EEPROM.
/// 4. Clears the driver's `pending_rf_write` latch.
#[allow(dead_code)]
pub fn sync_url_and_read_config<I2C, E>(
    nfc: &mut St25dv<I2C>,
    expected_url: &str,
    text_buf: &mut [u8],
) -> Result<SyncResult, Error<E>>
where
    I2C: I2c<Error = E>,
{
    let mut url_buf = [0u8; 256];
    let records = match read_tag(nfc, &mut url_buf, text_buf) {
        Ok(r) => r,
        Err(Error::Ndef(NdefError::InvalidCc | NdefError::Malformed)) => TagRecords {
            url_len: None,
            text_len: None,
            record_count: 0,
        },
        Err(e) => return Err(e),
    };

    let url_matches = records
        .url_len
        .and_then(|n| core::str::from_utf8(&url_buf[..n]).ok())
        .is_some_and(|u| u == expected_url);

    let url_restored = if !url_matches {
        let text_str = match records.text_len {
            Some(n) => Some(
                core::str::from_utf8(&text_buf[..n])
                    .map_err(|_| Error::Ndef(NdefError::InvalidEncoding))?,
            ),
            None => None,
        };
        write_tag(nfc, expected_url, text_str)?;
        true
    } else {
        false
    };

    nfc.clear_pending_rf_write();
    Ok(SyncResult {
        text_len: records.text_len,
        url_restored,
    })
}

/// Interrupt-driven rewrite protection + config reader using the ST25DV `GPO -> SCL` (`EXTI10`)
/// interrupt (**zero I2C polling while idle**).
///
/// Sleeps on `EXTI10` via [`St25dv::watch_rf_write`], debounces multi-block NFC Tools writes
/// without touching I2C, waits for the phone to lift off (`!field_on`), then reads the new Text
/// config into `text_buf` and automatically restores Slot 1 to `expected_url` if modified.
#[allow(dead_code)]
pub async fn wait_and_sync_config(
    nfc: &mut St25dv<SoftI2c<'static>>,
    exti10: &mut Peri<'static, peripherals::EXTI10>,
    expected_url: &str,
    text_buf: &mut [u8],
) -> Result<SyncResult, Error<SoftI2cError>> {
    nfc.watch_rf_write(exti10, |dev| {
        sync_url_and_read_config(dev, expected_url, text_buf)
    })
    .await
}
