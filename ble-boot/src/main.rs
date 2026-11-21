#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};
use panic_halt as _;

const APP_ADDR: u32 = 0x0000_4000;
const FLASH_END: u32 = 0x0008_0000;
const PAGE_SIZE: u32 = 4096;

const MAGIC0: u8 = 0xAA;
const MAGIC1: u8 = 0x55;
const CMD_SYNC: u8 = 0x01;
const CMD_ERASE: u8 = 0x02;
const CMD_WRITE: u8 = 0x03;
const CMD_BOOT: u8 = 0x04;
const CMD_CRC: u8 = 0x05;
const CMD_BAUD_250K: u8 = 0x06;
const RESP_ACK: u8 = 0x80;
const RESP_NACK: u8 = 0x81;

const BAUD_115200: u32 = 0x01D7_E000;
const BAUD_250000: u32 = 0x0400_0000;

/// Per-byte timeout inside command payloads (~150 ms at 64 MHz).
const BYTE_TIMEOUT_LOOPS: u32 = 1_500_000;

// nRF52832 GPIO (P0)
const P0_OUTSET: *mut u32 = 0x5000_0508 as *mut u32;
const P0_DIRSET: *mut u32 = 0x5000_0518 as *mut u32;
const P0_PIN_CNF_14: *mut u32 = (0x5000_0700 + 14 * 4) as *mut u32;
const P0_PIN_CNF_15: *mut u32 = (0x5000_0700 + 15 * 4) as *mut u32;

// nRF52832 UART0
const UART_STARTRX: *mut u32 = 0x4000_2000 as *mut u32;
const UART_STOPRX: *mut u32 = 0x4000_2004 as *mut u32;
const UART_STARTTX: *mut u32 = 0x4000_2008 as *mut u32;
const UART_STOPTX: *mut u32 = 0x4000_200C as *mut u32;
const UART_RXDRDY: *mut u32 = 0x4000_2108 as *mut u32;
const UART_TXDRDY: *mut u32 = 0x4000_211C as *mut u32;
const UART_ERRORSRC: *mut u32 = 0x4000_2480 as *mut u32;
const UART_ENABLE: *mut u32 = 0x4000_2500 as *mut u32;
const UART_PSELTXD: *mut u32 = 0x4000_250C as *mut u32;
const UART_PSELRXD: *mut u32 = 0x4000_2514 as *mut u32;
const UART_RXD: *const u32 = 0x4000_2518 as *const u32;
const UART_TXD: *mut u32 = 0x4000_251C as *mut u32;
const UART_BAUDRATE: *mut u32 = 0x4000_2524 as *mut u32;
const UART_CONFIG: *mut u32 = 0x4000_256C as *mut u32;

// nRF52832 NVMC
const NVMC_READY: *const u32 = 0x4001_E400 as *const u32;
const NVMC_CONFIG: *mut u32 = 0x4001_E504 as *mut u32;
const NVMC_ERASEPAGE: *mut u32 = 0x4001_E508 as *mut u32;

// ARM Cortex-M4 SCB / NVIC
const SCB_VTOR: *mut u32 = 0xE000_ED08 as *mut u32;
const NVIC_ICER0: *mut u32 = 0xE000_E180 as *mut u32;
const NVIC_ICER1: *mut u32 = 0xE000_E184 as *mut u32;
const NVIC_ICPR0: *mut u32 = 0xE000_E280 as *mut u32;
const NVIC_ICPR1: *mut u32 = 0xE000_E284 as *mut u32;

#[inline(always)]
unsafe fn w(addr: *mut u32, val: u32) {
    unsafe { write_volatile(addr, val) };
}

#[inline(always)]
unsafe fn r(addr: *const u32) -> u32 {
    unsafe { read_volatile(addr) }
}

unsafe fn uart_init() {
    unsafe {
        w(P0_OUTSET, 1 << 15);
        w(P0_DIRSET, 1 << 15);
        w(P0_PIN_CNF_15, 0x0000_0001); // Output
        w(P0_PIN_CNF_14, 0x0000_000C); // Input, Pull-up

        w(UART_ENABLE, 0);
        w(UART_PSELTXD, 15);
        w(UART_PSELRXD, 14);
        w(UART_BAUDRATE, BAUD_115200);
        w(UART_CONFIG, 0);
        w(UART_ERRORSRC, 0xF);
        w(UART_RXDRDY, 0);
        w(UART_TXDRDY, 0);
        w(UART_ENABLE, 4);
        w(UART_STARTRX, 1);
        w(UART_STARTTX, 1);
    }
}

unsafe fn uart_deinit() {
    unsafe {
        // Allow last TX byte stop bit to shift out (~250 us at 64 MHz).
        cortex_m::asm::delay(16_000);
        w(UART_STOPRX, 1);
        w(UART_STOPTX, 1);
        w(UART_ENABLE, 0);
        w(P0_PIN_CNF_14, 2); // Disconnect input buffer
        w(P0_PIN_CNF_15, 2);
    }
}

unsafe fn uart_write_byte(b: u8) {
    unsafe {
        w(UART_TXDRDY, 0);
        w(UART_TXD, b as u32);
        while r(UART_TXDRDY) == 0 {}
    }
}

unsafe fn uart_send_resp(code: u8) {
    unsafe {
        uart_write_byte(MAGIC0);
        uart_write_byte(MAGIC1);
        uart_write_byte(code);
    }
}

unsafe fn uart_read_byte_timeout(mut loops: u32) -> Option<u8> {
    unsafe {
        while loops > 0 {
            let err = r(UART_ERRORSRC);
            if err != 0 {
                w(UART_ERRORSRC, err);
            }
            if r(UART_RXDRDY) != 0 {
                w(UART_RXDRDY, 0);
                return Some((r(UART_RXD) & 0xFF) as u8);
            }
            loops -= 1;
        }
        None
    }
}

unsafe fn uart_read_byte() -> u8 {
    unsafe {
        loop {
            if let Some(b) = uart_read_byte_timeout(64_000) {
                return b;
            }
        }
    }
}

unsafe fn uart_read_u32_timeout() -> Option<u32> {
    unsafe {
        let b0 = uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS)?;
        let b1 = uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS)?;
        let b2 = uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS)?;
        let b3 = uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS)?;
        Some(u32::from_le_bytes([b0, b1, b2, b3]))
    }
}

unsafe fn nvmc_wait() {
    unsafe { while (r(NVMC_READY) & 1) == 0 {} }
}

unsafe fn flash_erase_pages(num_pages: u8) -> bool {
    let total_bytes = (num_pages as u32) * PAGE_SIZE;
    if num_pages == 0 || APP_ADDR + total_bytes > FLASH_END {
        return false;
    }
    unsafe {
        nvmc_wait();
        w(NVMC_CONFIG, 2); // EEN
        nvmc_wait();
        let mut i = 0u32;
        while i < num_pages as u32 {
            w(NVMC_ERASEPAGE, APP_ADDR + i * PAGE_SIZE);
            nvmc_wait();
            i += 1;
        }
        w(NVMC_CONFIG, 0); // REN
        nvmc_wait();
    }
    true
}

unsafe fn flash_write_words(offset: u32, words: &[u32]) -> bool {
    let addr = APP_ADDR.wrapping_add(offset);
    let byte_len = (words.len() as u32) * 4;
    if addr < APP_ADDR || addr.saturating_add(byte_len) > FLASH_END || (offset & 3) != 0 {
        return false;
    }
    unsafe {
        nvmc_wait();
        w(NVMC_CONFIG, 1); // WEN
        nvmc_wait();
        let mut ok = true;
        let mut i = 0usize;
        while i < words.len() {
            let dst = (addr + (i as u32) * 4) as *mut u32;
            w(dst, words[i]);
            nvmc_wait();
            if r(dst as *const u32) != words[i] {
                ok = false;
            }
            i += 1;
        }
        w(NVMC_CONFIG, 0); // REN
        nvmc_wait();
        ok
    }
}

/// Standard IEEE 802.3 CRC-32 over flash bytes `APP_ADDR..APP_ADDR + len`.
unsafe fn flash_crc32(len: u32) -> Option<u32> {
    if len == 0 || APP_ADDR.saturating_add(len) > FLASH_END {
        return None;
    }
    let mut crc = 0xFFFF_FFFFu32;
    let mut i = 0u32;
    while i < len {
        let byte = unsafe { read_volatile((APP_ADDR + i) as *const u8) };
        crc ^= byte as u32;
        let mut bit = 0;
        while bit < 8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        i += 1;
    }
    Some(!crc)
}

fn app_is_valid() -> bool {
    let sp = unsafe { r(APP_ADDR as *const u32) };
    let rv = unsafe { r((APP_ADDR + 4) as *const u32) };
    let sp_ok = (0x2000_0000..=0x2001_0000).contains(&sp) && (sp & 3) == 0;
    let rv_ok = (APP_ADDR..FLASH_END).contains(&rv) && (rv & 1) == 1;
    sp_ok && rv_ok
}

unsafe fn jump_to_app() -> ! {
    unsafe {
        cortex_m::interrupt::disable();
        w(NVIC_ICER0, 0xFFFF_FFFF);
        w(NVIC_ICER1, 0xFFFF_FFFF);
        w(NVIC_ICPR0, 0xFFFF_FFFF);
        w(NVIC_ICPR1, 0xFFFF_FFFF);
        uart_deinit();
        w(SCB_VTOR, APP_ADDR);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        cortex_m::interrupt::enable();
        cortex_m::asm::bootload(APP_ADDR as *const u32);
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    unsafe { uart_init() };

    // Wait ~350 ms at 64 MHz for SYNC frame [0xAA, 0x55, 0x01]
    let mut synced = false;
    let mut state = 0u8;
    let mut ticks = 3_500_000u32;
    while ticks > 0 {
        if let Some(b) = unsafe { uart_read_byte_timeout(1) } {
            match (state, b) {
                (0, MAGIC0) => state = 1,
                (1, MAGIC1) => state = 2,
                (2, CMD_SYNC) => {
                    synced = true;
                    break;
                }
                (_, MAGIC0) => state = 1,
                _ => state = 0,
            }
        }
        ticks -= 1;
    }

    if !synced && app_is_valid() {
        unsafe { jump_to_app() };
    }

    if synced {
        unsafe { uart_send_resp(RESP_ACK) };
    }

    let mut word_buf = [0u32; 64]; // max 256 bytes per write packet
    loop {
        // Wait for frame header 0xAA 0x55 (handling repeated 0xAA bytes cleanly)
        let mut hdr_state = 0u8;
        while hdr_state < 2 {
            let b = unsafe { uart_read_byte() };
            hdr_state = match (hdr_state, b) {
                (_, MAGIC0) => 1,
                (1, MAGIC1) => 2,
                _ => 0,
            };
        }

        let Some(cmd) = (unsafe { uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS) }) else {
            continue;
        };

        match cmd {
            CMD_SYNC => unsafe { uart_send_resp(RESP_ACK) },
            CMD_ERASE => unsafe {
                let Some(num_pages) = uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS) else {
                    uart_send_resp(RESP_NACK);
                    continue;
                };
                let ok = flash_erase_pages(num_pages);
                uart_send_resp(if ok { RESP_ACK } else { RESP_NACK });
            },
            CMD_WRITE => unsafe {
                let (Some(o0), Some(o1), Some(o2), Some(o3), Some(l0), Some(l1)) = (
                    uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                    uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                    uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                    uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                    uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                    uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                ) else {
                    uart_send_resp(RESP_NACK);
                    continue;
                };
                let offset = u32::from_le_bytes([o0, o1, o2, o3]);
                let len = u16::from_le_bytes([l0, l1]) as usize;

                let mut xsum = CMD_WRITE ^ o0 ^ o1 ^ o2 ^ o3 ^ l0 ^ l1;
                if len == 0 || len > 256 || (len & 3) != 0 {
                    uart_send_resp(RESP_NACK);
                    continue;
                }
                let mut i = 0usize;
                let mut w_idx = 0usize;
                let mut timed_out = false;
                while i < len {
                    let (Some(b0), Some(b1), Some(b2), Some(b3)) = (
                        uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                        uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                        uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                        uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS),
                    ) else {
                        timed_out = true;
                        break;
                    };
                    xsum ^= b0 ^ b1 ^ b2 ^ b3;
                    word_buf[w_idx] = u32::from_le_bytes([b0, b1, b2, b3]);
                    w_idx += 1;
                    i += 4;
                }
                if timed_out {
                    uart_send_resp(RESP_NACK);
                    continue;
                }
                let Some(rx_xsum) = uart_read_byte_timeout(BYTE_TIMEOUT_LOOPS) else {
                    uart_send_resp(RESP_NACK);
                    continue;
                };
                if rx_xsum == xsum && flash_write_words(offset, &word_buf[..w_idx]) {
                    uart_send_resp(RESP_ACK);
                } else {
                    uart_send_resp(RESP_NACK);
                }
            },
            CMD_BOOT => unsafe {
                if app_is_valid() {
                    uart_send_resp(RESP_ACK);
                    jump_to_app();
                } else {
                    uart_send_resp(RESP_NACK);
                }
            },
            CMD_CRC => unsafe {
                let Some(len) = uart_read_u32_timeout() else {
                    uart_send_resp(RESP_NACK);
                    continue;
                };
                if let Some(crc) = flash_crc32(len) {
                    uart_send_resp(RESP_ACK);
                    for b in crc.to_le_bytes() {
                        uart_write_byte(b);
                    }
                } else {
                    uart_send_resp(RESP_NACK);
                }
            },
            CMD_BAUD_250K => unsafe {
                uart_send_resp(RESP_ACK);
                // Let the 115200 ACK finish shifting out before switching baud rate.
                cortex_m::asm::delay(16_000);
                w(UART_STOPRX, 1);
                w(UART_STOPTX, 1);
                w(UART_ENABLE, 0);
                w(UART_BAUDRATE, BAUD_250000);
                w(UART_ERRORSRC, 0xF);
                w(UART_RXDRDY, 0);
                w(UART_TXDRDY, 0);
                w(UART_ENABLE, 4);
                w(UART_STARTRX, 1);
                w(UART_STARTTX, 1);
            },
            _ => unsafe { uart_send_resp(RESP_NACK) },
        }
    }
}
