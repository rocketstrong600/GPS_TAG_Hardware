use defmt::{Format, error, info};
use embassy_stm32::Peri;
use embassy_stm32::flash::{self, Flash};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals;
use embassy_stm32::usart::{self, Uart};
use embassy_time::Timer;

use gps_tag_drivers::board::BlePins;

/// nRF52832 application binary automatically built and flattened by `build.rs`.
#[cfg(feature = "nrf-fw")]
pub const EMBEDDED_FIRMWARE: &[u8] = include_bytes!(env!("NRF_FW_BIN"));

const SYNC_0: u8 = 0xAA;
const SYNC_1: u8 = 0x55;

const CMD_PING: u8 = 0x01;
const CMD_ERASE: u8 = 0x02;
const CMD_WRITE: u8 = 0x03;
const CMD_BOOT: u8 = 0x04;
const CMD_CRC: u8 = 0x05;

const RESP_ACK: u8 = 0x80;
const RESP_NACK: u8 = 0x81;

const PAGE_SIZE: usize = 4096;
const CHUNK_SIZE: usize = 256;

/// STM32F412RG Sector 11 (128 KiB at 0x080E_0000, above application code).
const META_OFFSET: u32 = 0x000E_0000;
const META_SECTOR_END: u32 = 0x0010_0000;
const META_MAGIC: u64 = 0x4E52_4642_4F4F_5431; // "NRFBOOT1"

#[derive(Debug, Format)]
pub enum Error {
    Uart(usart::Error),
    Flash(flash::Error),
    Nack,
    UnexpectedResponse([u8; 3]),
    CrcMismatch { expected: u32, actual: u32 },
    ImageTooLarge,
}

impl From<usart::Error> for Error {
    fn from(e: usart::Error) -> Self {
        Self::Uart(e)
    }
}

impl From<flash::Error> for Error {
    fn from(e: flash::Error) -> Self {
        Self::Flash(e)
    }
}

fn fnv1a64(data: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in data {
        hash ^= b as u64;
        hash = hash.wrapping_mul(0x0100_0000_01b3);
    }
    hash
}

/// Standard IEEE 802.3 CRC-32 matching `ble-boot`'s `CMD_CRC`.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

fn read_ack(uart: &mut Uart<'_, Blocking>) -> Result<(), Error> {
    // Sync to the [0xAA, 0x55, resp] header, skipping any stale bytes left in RX
    // from a previously running nRF52832 application.
    let mut b = [0u8; 1];
    let mut state = 0u8;
    for _ in 0..32 {
        match uart.blocking_read(&mut b) {
            Ok(()) => match (state, b[0]) {
                (0, SYNC_0) => state = 1,
                (1, SYNC_1) => state = 2,
                (2, RESP_ACK) => return Ok(()),
                (2, RESP_NACK) => return Err(Error::Nack),
                (2, other) => return Err(Error::UnexpectedResponse([SYNC_0, SYNC_1, other])),
                (_, SYNC_0) => state = 1,
                _ => state = 0,
            },
            Err(usart::Error::Overrun) if state == 0 => continue,
            Err(e) => return Err(e.into()),
        }
    }
    Err(Error::UnexpectedResponse([0, 0, 0]))
}

/// Flash `bin` onto the MBN52832 only if its FNV-1a hash differs from the hash
/// cached in STM32 Sector 11 (`0x080E_0000`).
///
/// Returns `Ok(true)` if the nRF52832 was reflashed, or `Ok(false)` if already up-to-date.
pub async fn ensure_firmware(
    ble: &mut BlePins,
    flash_periph: Peri<'_, peripherals::FLASH>,
    bin: &[u8],
) -> Result<bool, Error> {
    let want_hash = fnv1a64(bin);
    let mut flash = Flash::new_blocking(flash_periph);

    let mut meta = [0u8; 16];
    flash.blocking_read(META_OFFSET, &mut meta)?;
    let stored_magic = u64::from_le_bytes(meta[0..8].try_into().unwrap());
    let stored_hash = u64::from_le_bytes(meta[8..16].try_into().unwrap());

    if stored_magic == META_MAGIC && stored_hash == want_hash {
        info!(
            "nrf_boot: firmware up-to-date ({:#018x}), skipping flash",
            want_hash
        );
        ble.reset.set_low();
        return Ok(false);
    }

    let mut last_err = None;
    for _ in 0..2 {
        match flash_over_uart(ble, bin).await {
            Ok(()) => {
                last_err = None;
                break;
            }
            Err(e @ (Error::Uart(usart::Error::Overrun) | Error::CrcMismatch { .. })) => {
                error!("nrf_boot: transient error {:?}, retrying", e);
                last_err = Some(e);
            }
            Err(e) => return Err(e),
        }
    }
    if let Some(e) = last_err {
        return Err(e);
    }

    let mut new_meta = [0u8; 16];
    new_meta[0..8].copy_from_slice(&META_MAGIC.to_le_bytes());
    new_meta[8..16].copy_from_slice(&want_hash.to_le_bytes());
    flash.blocking_erase(META_OFFSET, META_SECTOR_END)?;
    flash.blocking_write(META_OFFSET, &new_meta)?;
    info!("nrf_boot: recorded firmware hash {:#018x}", want_hash);

    Ok(true)
}

/// Unconditionally reset the MBN52832 into `ble-boot`, erase enough 4 KiB pages,
/// stream `bin` over `USART2` at 115200 baud, verify flash CRC-32, and boot `0x0000_4000`.
pub async fn flash_over_uart(ble: &mut BlePins, bin: &[u8]) -> Result<(), Error> {
    let num_pages = bin.len().div_ceil(PAGE_SIZE);
    if num_pages == 0 || num_pages > u8::MAX as usize {
        return Err(Error::ImageTooLarge);
    }

    info!(
        "nrf_boot: flashing {} bytes ({} pages) to MBN52832",
        bin.len(),
        num_pages
    );

    // Pulse PC2 HIGH -> LOW to reset the nRF52832 into ble-boot's sync window.
    ble.reset.set_high();
    Timer::after_millis(50).await;
    ble.reset.set_low();
    Timer::after_millis(20).await;

    // 1. Handshake
    ble.uart.blocking_write(&[SYNC_0, SYNC_1, CMD_PING])?;
    read_ack(&mut ble.uart)?;
    info!("nrf_boot: bootloader synced");

    // 2. Erase pages
    ble.uart
        .blocking_write(&[SYNC_0, SYNC_1, CMD_ERASE, num_pages as u8])?;
    read_ack(&mut ble.uart)?;
    info!("nrf_boot: erased {} pages", num_pages);

    // 4. Stream image in <=256-byte, 4-byte-aligned chunks
    let mut frame = [0u8; 2 + 1 + 4 + 2 + CHUNK_SIZE + 1];
    for (idx, chunk) in bin.chunks(CHUNK_SIZE).enumerate() {
        let offset = (idx * CHUNK_SIZE) as u32;
        let padded_len = (chunk.len() + 3) & !3;

        frame[0] = SYNC_0;
        frame[1] = SYNC_1;
        frame[2] = CMD_WRITE;
        frame[3..7].copy_from_slice(&offset.to_le_bytes());
        frame[7..9].copy_from_slice(&(padded_len as u16).to_le_bytes());
        frame[9..9 + chunk.len()].copy_from_slice(chunk);
        for b in &mut frame[9 + chunk.len()..9 + padded_len] {
            *b = 0xFF;
        }

        let mut xor_sum = 0u8;
        for &b in &frame[2..9 + padded_len] {
            xor_sum ^= b;
        }
        frame[9 + padded_len] = xor_sum;

        ble.uart.blocking_write(&frame[..9 + padded_len + 1])?;
        if let Err(e) = read_ack(&mut ble.uart) {
            error!("nrf_boot: write failed at offset {:#x}: {:?}", offset, e);
            return Err(e);
        }
    }
    info!("nrf_boot: wrote {} bytes", bin.len());

    // 5. Verify flash CRC-32
    let expected_crc = crc32(bin);
    let len_bytes = (bin.len() as u32).to_le_bytes();
    ble.uart.blocking_write(&[
        SYNC_0,
        SYNC_1,
        CMD_CRC,
        len_bytes[0],
        len_bytes[1],
        len_bytes[2],
        len_bytes[3],
    ])?;
    read_ack(&mut ble.uart)?;
    let mut crc_buf = [0u8; 4];
    ble.uart.blocking_read(&mut crc_buf)?;
    let actual_crc = u32::from_le_bytes(crc_buf);
    if actual_crc != expected_crc {
        error!(
            "nrf_boot: CRC-32 mismatch (expected {:#010x}, got {:#010x})",
            expected_crc, actual_crc
        );
        return Err(Error::CrcMismatch {
            expected: expected_crc,
            actual: actual_crc,
        });
    }
    info!("nrf_boot: CRC-32 verified ({:#010x})", actual_crc);

    // 6. Boot application
    ble.uart.blocking_write(&[SYNC_0, SYNC_1, CMD_BOOT])?;
    read_ack(&mut ble.uart)?;
    info!("nrf_boot: CMD_BOOT acknowledged");

    Ok(())
}
