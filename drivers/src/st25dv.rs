//! STMicroelectronics **ST25DV16K** (16 Kbit / 2048 B) dual-interface NFC Forum Type 5 dynamic
//! tag driver (DS10925 Rev 12).
//!
//! ## Wiring & Shared `GPO -> SCL` Interrupt
//! - `PC10` = `SCL` + open-drain `GPO` (`IC_REF = 0x26`, `EXTI10`)
//! - `PC11` = `SDA`
//!
//! By configuring static `GPO = 0xC0` (`GPO_EN | RF_WRITE_EN`) and `IT_TIME = 0x00` (`301 µs`
//! active-low pulse) via [`St25dv::configure_gpo_rf_write`], `GPO` stays High-Z at idle and
//! emits a `301 µs` pulse on `SCL` whenever an RF reader writes EEPROM. Calling
//! [`St25dv::wait_for_rf_write`], [`St25dv::watch_rf_write`],
//! [`St25dv::protect_ndef_records_on_rf_write`], or [`St25dv::protect_region_on_rf_write`] sleeps
//! at **0 µA polling** on `EXTI10`, debounces
//! multi-block RF writes with zero I2C traffic (avoiding DS10925 Section 5.5 I2C-vs-RF `0x0F`
//! busy errors), and waits for `!field_on` before executing your I2C callback.
//!
//! ## Features
//! - 4-byte-page-aligned User EEPROM read/write with exact per-chunk `tW` ACK-polling timeouts
//! - I2C 64-bit password session management (`open_i2c_session` / `close_i2c_session`)
//! - Area 1 hardware RF write-lock (`RFA1SS`: [`St25dv::set_rf_write_locked`]) and dynamic RF
//!   disable/sleep (`RF_MNGT_Dyn`: [`St25dv::set_rf_mode`])
//! - 256-byte Fast Transfer Mode (FTM) mailbox (`enable_mailbox`, `disable_mailbox`,
//!   `write_mailbox`, `read_mailbox`)
//! - Generic Type 5 NDEF URI/Text/vCard codec in [`ndef`]
//!
//! See `examples/nfc_gpo_read.rs`, `examples/nfc_gpo_protect.rs`, and `examples/nfc_vcard.rs` for
//! runnable examples.

pub mod ndef;

use defmt::{Format, error, info, warn};
use embassy_stm32::gpio::Flex;
use embassy_stm32::{Peri, interrupt, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, WithTimeout};
use embedded_hal::i2c::I2c;

use crate::soft_i2c::{SoftI2c, SoftI2cError};
pub use ndef::NdefError;

/// 7-bit I2C address for User EEPROM (`0x0000..=0x07FF`), Dynamic registers (`0x2000..=0x2007`),
/// and Fast Transfer Mode mailbox RAM (`0x2008..=0x2107`) (`E2 = 0`).
pub const ADDR_USER: u8 = 0x53;
/// 7-bit I2C address for System Configuration registers (`0x0000..=0x0020`) and I2C password
/// (`0x0900`) (`E2 = 1`).
pub const ADDR_SYSTEM: u8 = 0x57;

/// ST25DV16K EEPROM capacity in bytes (16 Kbit = 2048 bytes).
pub const USER_MEM_BYTES: usize = 2048;
/// ST25DVxxK maximum I2C sequential write length (DS10925 Section 6.4.2).
const I2C_MAX_SEQ_WRITE: usize = 256;
/// Internal EEPROM write-page size in bytes (DS10925 Section 6.4.2: `tW` applies per 4-byte page).
const EEPROM_INTERNAL_PAGE: usize = 4;
/// Maximum single chunk written by [`St25dv::write_user_memory`] (keeps stack buffer compact).
const WRITE_CHUNK_MAX: usize = 32;
/// Worst-case `tW` per 4-byte EEPROM page (DS10925 Table 248/249: 5.5 ms max up to 125 C, rounded
/// up to 6 ms for margin).
const WRITE_MS_PER_PAGE: u64 = 6;
/// Extra safety margin added to EEPROM ACK-polling timeouts.
const WRITE_TIMEOUT_MARGIN_MS: u64 = 20;
/// Maximum FTM mailbox message size in bytes (DS10925 Section 4.5).
pub const MAILBOX_MAX_LEN: usize = 256;

/// Factory default 64-bit I2C password.
pub const DEFAULT_I2C_PASSWORD: [u8; 8] = [0u8; 8];

// STM32F412RG SYSCFG / EXTI registers for EXTI10 (PC10 = SCL + ST25DV open-drain GPO).
const SYSCFG_EXTICR3: *mut u32 = 0x4001_3810 as *mut u32;
const EXTI_IMR: *mut u32 = 0x4001_3C00 as *mut u32;
const EXTI_RTSR: *mut u32 = 0x4001_3C08 as *mut u32;
const EXTI_FTSR: *mut u32 = 0x4001_3C0C as *mut u32;
const EXTI_PR: *mut u32 = 0x4001_3C14 as *mut u32;
const EXTI10_BIT: u32 = 1 << 10;

static GPO_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[interrupt]
fn EXTI15_10() {
    // SAFETY: Reading/clearing EXTI10 bit in EXTI_PR and masking EXTI_IMR is atomic in ISR.
    unsafe {
        if (EXTI_PR.read_volatile() & EXTI10_BIT) != 0 {
            // Mask EXTI10 immediately so subsequent I2C SCL clocks do not re-trigger it.
            EXTI_IMR.write_volatile(EXTI_IMR.read_volatile() & !EXTI10_BIT);
            EXTI_PR.write_volatile(EXTI10_BIT);
            GPO_SIGNAL.signal(());
        }
    }
}

fn exti10_arm_pc10_falling() {
    cortex_m::interrupt::free(|_| unsafe {
        // Route EXTI10 (EXTICR3 bits 11:8) to Port C (0b0010).
        let exticr3 = SYSCFG_EXTICR3.read_volatile();
        SYSCFG_EXTICR3.write_volatile((exticr3 & !(0xF << 8)) | (0x2 << 8));
        // Falling-edge trigger only.
        EXTI_RTSR.write_volatile(EXTI_RTSR.read_volatile() & !EXTI10_BIT);
        EXTI_FTSR.write_volatile(EXTI_FTSR.read_volatile() | EXTI10_BIT);
        // Clear any stale pending edge, then unmask EXTI10.
        EXTI_PR.write_volatile(EXTI10_BIT);
        EXTI_IMR.write_volatile(EXTI_IMR.read_volatile() | EXTI10_BIT);
    });
}

fn exti10_disarm() {
    cortex_m::interrupt::free(|_| unsafe {
        EXTI_IMR.write_volatile(EXTI_IMR.read_volatile() & !EXTI10_BIT);
        EXTI_PR.write_volatile(EXTI10_BIT);
    });
}

struct Exti10Guard;

impl Drop for Exti10Guard {
    fn drop(&mut self) {
        exti10_disarm();
    }
}

// System configuration registers (I2C address 0x57, 16-bit big-endian sub-address; DS10925 Table 11)
#[allow(dead_code)]
mod sys_reg {
    pub const GPO: u16 = 0x0000;
    pub const IT_TIME: u16 = 0x0001;
    pub const EH_MODE: u16 = 0x0002;
    pub const RF_MNGT: u16 = 0x0003;
    pub const RFA1SS: u16 = 0x0004; // Area 1 RF security status (b3:b2 RW_PROT, b1:b0 PWD_CTRL)
    pub const ENDA1: u16 = 0x0005;
    pub const MB_MODE: u16 = 0x000D;
    pub const MB_WDG: u16 = 0x000E;
    pub const LOCK_CFG: u16 = 0x000F;
    pub const MEM_SIZE: u16 = 0x0014; // 2 bytes, little-endian (blocks - 1)
    pub const BLK_SIZE: u16 = 0x0016; // 1 byte (bytes_per_block - 1)
    pub const IC_REF: u16 = 0x0017;
    pub const UID: u16 = 0x0018; // 8 bytes, LSB first (byte 7 = 0xE0)
    pub const IC_REV: u16 = 0x0020;
    pub const I2C_PASSWD: u16 = 0x0900;
}

// Dynamic registers and mailbox RAM (I2C address 0x53, 16-bit big-endian sub-address; DS10925 Table 12)
#[allow(dead_code)]
mod dyn_reg {
    pub const GPO_CTRL: u16 = 0x2000; // b7 = GPO_EN (R/W), b6:b0 = RO mirror of static GPO
    pub const EH_CTRL: u16 = 0x2002;
    pub const RF_MNGT: u16 = 0x2003; // b0 = RF_DISABLE, b1 = RF_SLEEP
    pub const I2C_SSO: u16 = 0x2004; // b0 = I2C_SSO_Dyn
    pub const IT_STS: u16 = 0x2005; // Clear-on-read interrupt status
    pub const MB_CTRL: u16 = 0x2006; // b0 = MB_EN (R/W), b7:b1 = RO flags
    pub const MB_LEN: u16 = 0x2007; // Message length - 1
    pub const MAILBOX_RAM: u16 = 0x2008;
}

/// Static `GPO` (`0x0000`): `GPO_EN` (b7) | `RF_WRITE_EN` (b6) (DS10925 Table 26).
const GPO_RF_WRITE_PULSE: u8 = 0xC0;
/// Static `IT_TIME` (`0x0001`): `0x00` = 301 us active-low pulse (DS10925 Table 28, Eq. 1).
const IT_TIME_301US: u8 = 0x00;
/// Dynamic `GPO_CTRL_Dyn` (`0x2000`): `GPO_EN` is bit 7 on ST25DVxxK (DS10925 Table 30).
const GPO_DYN_EN: u8 = 0x80;

/// `RFA1SS` (`0x0004`): Area 1 RF read/write open (DS10925 Table 44).
const RFA1SS_OPEN: u8 = 0x00;
/// `RFA1SS` (`0x0004`): `RW_PROTECTION_A1 = 11b`, `PWD_CTRL_A1 = 00b` — RF read always allowed,
/// RF write always forbidden, I2C read/write unaffected (DS10925 Table 44).
const RFA1SS_RF_WRITE_FORBIDDEN: u8 = 0x0C;

/// Dynamic RF interface operating mode (`RF_MNGT_Dyn` `0x2003`; DS10925 Table 42).
#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
#[allow(dead_code)]
pub enum RfMode {
    /// Normal RF operation (`RF_SLEEP = 0, RF_DISABLE = 0`).
    Normal = 0x00,
    /// RF commands are demodulated but rejected with error `0x0F` (`RF_DISABLE = 1`).
    Disable = 0x01,
    /// RF interface completely silent with lowest RF power draw (`RF_SLEEP = 1`).
    Sleep = 0x02,
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub enum Error<E> {
    I2c(E),
    WriteTimeout,
    OutOfRange,
    AuthFailed,
    /// Fast Transfer Mode (`MB_EN = 1`) is active; deactivate FTM before writing EEPROM
    /// (DS10925 Section 6.4 Caution).
    MailboxActive,
    /// Fast Transfer Mode (`MB_EN = 0`) is inactive; enable FTM before accessing mailbox RAM.
    MailboxDisabled,
    MailboxEmpty,
    MailboxBusy,
    /// An NFC reader/phone RF field is currently active; retry once the phone moves away.
    RfBusy,
    Ndef(NdefError),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::I2c(e)
    }
}

/// Identification metadata read from the ST25DV system area (`0x57`).
#[derive(Debug, Copy, Clone, Format)]
pub struct DeviceInfo {
    /// `0x24` = ST25DV04K (OD), `0x25` = ST25DV04K (CMOS), `0x26` = ST25DV16K/64K (OD),
    /// `0x27` = ST25DV16K/64K (CMOS).
    pub ic_ref: u8,
    pub ic_rev: u8,
    /// 8-byte ISO 15693 UID in MSB-first display order (`uid[0] == 0xE0`, `uid[1] == 0x02`).
    pub uid: [u8; 8],
    pub mem_bytes: usize,
    pub block_size: u8,
}

/// Snapshot of volatile dynamic registers (`0x2002..=0x2006`; DS10925 Tables 19, 32, 37, 42, 68).
#[derive(Debug, Copy, Clone, Format)]
#[allow(dead_code)]
pub struct DynamicStatus {
    /// RF magnetic field is currently present on the antenna (`EH_CTRL_Dyn` b2).
    pub field_on: bool,
    /// VCC power supply is present and LPD is not forced (`EH_CTRL_Dyn` b3).
    pub vcc_on: bool,
    /// Energy harvesting output is enabled (`EH_CTRL_Dyn` b1).
    pub eh_on: bool,
    /// Raw `RF_MNGT_Dyn` (`0x2003`: b0 `RF_DISABLE`, b1 `RF_SLEEP`).
    pub rf_mngt: u8,
    /// I2C security session is currently unlocked (`I2C_SSO_Dyn` b0).
    pub i2c_session_open: bool,
    /// Raw `IT_STS_Dyn` (`0x2005`, read-to-clear: b0 `RF_USER`, b1 `RF_ACTIVITY`,
    /// b2 `RF_INTERRUPT`, b3 `FIELD_FALLING`, b4 `FIELD_RISING`, b5 `RF_PUT_MSG`,
    /// b6 `RF_GET_MSG`, b7 `RF_WRITE`).
    pub it_sts: u8,
    /// Raw `MB_CTRL_Dyn` (`0x2006`: b0 `MB_EN`, b1 `HOST_PUT_MSG`, b2 `RF_PUT_MSG`, ...).
    pub mb_ctrl: u8,
}

impl DynamicStatus {
    #[allow(dead_code)]
    pub fn mb_enabled(&self) -> bool {
        (self.mb_ctrl & (1 << 0)) != 0
    }

    #[allow(dead_code)]
    pub fn host_put_msg(&self) -> bool {
        (self.mb_ctrl & (1 << 1)) != 0
    }

    #[allow(dead_code)]
    pub fn rf_put_msg(&self) -> bool {
        (self.mb_ctrl & (1 << 2)) != 0
    }

    #[allow(dead_code)]
    pub fn rf_wrote_eeprom(&self) -> bool {
        (self.it_sts & (1 << 7)) != 0
    }
}

pub struct St25dv<I2C> {
    i2c: I2C,
    /// Software latch for `IT_STS_Dyn` bit 7 (`RF_WRITE`), which is clear-on-read in hardware.
    pending_rf_write: bool,
}

impl<I2C, E> St25dv<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            pending_rf_write: false,
        }
    }

    #[allow(dead_code)]
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Clears the software `RF_WRITE` latch (normally called after handling an RF write).
    #[allow(dead_code)]
    pub fn clear_pending_rf_write(&mut self) {
        self.pending_rf_write = false;
    }

    /// Reads IC reference, silicon revision, 8-byte UID, and EEPROM size from system memory (`0x57`).
    pub fn read_device_info(&mut self) -> Result<DeviceInfo, Error<E>> {
        let mut sys = [0u8; 13]; // 0x0014..=0x0020
        self.read_reg(ADDR_SYSTEM, sys_reg::MEM_SIZE, &mut sys)?;

        let blocks = (u16::from_le_bytes([sys[0], sys[1]]) as usize) + 1;
        let block_size = sys[2].saturating_add(1);
        let ic_ref = sys[3];
        let ic_rev = sys[12];

        // UID registers 0x0018..=0x001F are stored LSB-first; reverse to standard E0:02:... order.
        let mut uid = [0u8; 8];
        for i in 0..8 {
            uid[i] = sys[4 + (7 - i)];
        }

        Ok(DeviceInfo {
            ic_ref,
            ic_rev,
            uid,
            mem_bytes: blocks * (block_size as usize),
            block_size,
        })
    }

    /// Reads dynamic registers (`0x2002..=0x2006`).
    ///
    /// Because `IT_STS_Dyn` (`0x2005`) is clear-on-read in hardware, any observed `RF_WRITE`
    /// (bit 7) is latched internally so interrupt/polling workflows never drop an RF write event.
    pub fn read_dynamic_status(&mut self) -> Result<DynamicStatus, Error<E>> {
        let mut buf = [0u8; 5]; // 0x2002..=0x2006
        self.read_reg(ADDR_USER, dyn_reg::EH_CTRL, &mut buf)?;
        let eh = buf[0];
        let rf_mngt = buf[1];
        let sso = buf[2];
        let it_sts = buf[3];
        let mb_ctrl = buf[4];

        if (it_sts & (1 << 7)) != 0 {
            self.pending_rf_write = true;
        }

        Ok(DynamicStatus {
            eh_on: (eh & (1 << 1)) != 0,
            field_on: (eh & (1 << 2)) != 0,
            vcc_on: (eh & (1 << 3)) != 0,
            rf_mngt,
            i2c_session_open: (sso & 0x01) != 0,
            it_sts,
            mb_ctrl,
        })
    }

    /// Lightweight polling fallback: returns `Ok(true)` once an RF EEPROM write has been latched
    /// AND the phone RF field has turned off (`!field_on`).
    ///
    /// Prefer [`St25dv::watch_rf_write`] / [`St25dv::wait_for_rf_write`] when `GPO -> SCL`
    /// (`EXTI10`) is wired for zero-polling wakeups.
    #[allow(dead_code)]
    pub fn poll_rf_write_ready(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_dynamic_status()?;
        Ok(self.pending_rf_write && !status.field_on)
    }

    /// Reads arbitrary bytes from User EEPROM (`0x0000..USER_MEM_BYTES`).
    pub fn read_user_memory(&mut self, addr: u16, buf: &mut [u8]) -> Result<(), Error<E>> {
        if (addr as usize) + buf.len() > USER_MEM_BYTES {
            return Err(Error::OutOfRange);
        }
        if buf.is_empty() {
            return Ok(());
        }
        self.read_reg(ADDR_USER, addr, buf)
    }

    /// Writes arbitrary bytes to User EEPROM (`0x0000..USER_MEM_BYTES`).
    ///
    /// Complies with DS10925 Section 6.4:
    /// - Rejects with [`Error::MailboxActive`] if FTM (`MB_EN = 1`) is enabled (EEPROM writes
    ///   transit through the mailbox buffer and are NACKed while FTM is active).
    /// - Splits writes into <=32-byte chunks aligned to 4-byte internal EEPROM pages and never
    ///   crossing 256-byte sequential limits.
    /// - Computes ACK-polling timeout from the exact number of 4-byte internal pages touched
    ///   (`pages * 6 ms + 20 ms`).
    #[allow(dead_code)]
    pub fn write_user_memory(&mut self, mut addr: u16, mut data: &[u8]) -> Result<(), Error<E>> {
        if (addr as usize) + data.len() > USER_MEM_BYTES {
            return Err(Error::OutOfRange);
        }
        if data.is_empty() {
            return Ok(());
        }
        self.ensure_mailbox_inactive()?;

        while !data.is_empty() {
            let seq_rem = I2C_MAX_SEQ_WRITE - ((addr as usize) % I2C_MAX_SEQ_WRITE);
            let align_cap = WRITE_CHUNK_MAX - ((addr as usize) % EEPROM_INTERNAL_PAGE);
            let chunk_len = data.len().min(seq_rem).min(align_cap);

            let pages = (((addr as usize) % EEPROM_INTERNAL_PAGE) + chunk_len)
                .div_ceil(EEPROM_INTERNAL_PAGE);
            let timeout =
                Duration::from_millis((pages as u64) * WRITE_MS_PER_PAGE + WRITE_TIMEOUT_MARGIN_MS);

            self.write_reg_chunk(ADDR_USER, addr, &data[..chunk_len])?;
            self.wait_write_ready(ADDR_USER, timeout)?;
            addr += chunk_len as u16;
            data = &data[chunk_len..];
        }
        Ok(())
    }

    /// Presents the 64-bit I2C password (`0x0900` on `0x57`) to unlock system configuration writes
    /// (DS10925 Section 6.6.1).
    pub fn open_i2c_session(&mut self, password: [u8; 8]) -> Result<(), Error<E>> {
        self.present_i2c_password(password)?;
        let mut sso = [0u8; 1];
        self.read_reg(ADDR_USER, dyn_reg::I2C_SSO, &mut sso)?;
        if (sso[0] & 0x01) == 0 {
            return Err(Error::AuthFailed);
        }
        Ok(())
    }

    /// Closes an open I2C security session by presenting a non-matching password (DS10925
    /// Section 6.6.1: mismatched password clears `I2C_SSO_Dyn` to `0x00`).
    pub fn close_i2c_session(&mut self, password: [u8; 8]) -> Result<(), Error<E>> {
        let mut wrong = password;
        wrong[0] ^= 0xFF;
        self.present_i2c_password(wrong)
    }

    /// Reads the static `(GPO, IT_TIME)` configuration registers (`0x0000..=0x0001` on `0x57`).
    pub fn read_gpo_config(&mut self) -> Result<(u8, u8), Error<E>> {
        let mut buf = [0u8; 2];
        self.read_reg(ADDR_SYSTEM, sys_reg::GPO, &mut buf)?;
        Ok((buf[0], buf[1]))
    }

    /// Reads dynamic `GPO_CTRL_Dyn` (`0x2000` on `0x53`). Bit 7 is `GPO_EN`.
    pub fn read_gpo_dyn(&mut self) -> Result<u8, Error<E>> {
        let mut buf = [0u8; 1];
        self.read_reg(ADDR_USER, dyn_reg::GPO_CTRL, &mut buf)?;
        Ok(buf[0])
    }

    /// Dynamically enables or disables the `GPO` output pin via `GPO_CTRL_Dyn` bit 7 (`0x2000`,
    /// no I2C password required; DS10925 Table 30).
    #[allow(dead_code)]
    pub fn set_gpo_enabled_dyn(&mut self, enabled: bool) -> Result<(), Error<E>> {
        let val = if enabled { GPO_DYN_EN } else { 0x00 };
        self.write_reg_chunk(ADDR_USER, dyn_reg::GPO_CTRL, &[val])
    }

    /// Configures the open-drain `GPO` pin (wired to `SCL` / `PC10`) to emit a 301 us active-low
    /// pulse ONLY on RF EEPROM writes (`GPO = 0xC0`, `IT_TIME = 0x00`), keeping `SCL` High-Z
    /// whenever idle.
    ///
    /// Only programs static EEPROM if `GPO` or `IT_TIME` differ from the target values, closes
    /// the I2C security session afterward, and enables dynamic `GPO_CTRL_Dyn` (`0x80`).
    pub fn configure_gpo_rf_write(&mut self, password: [u8; 8]) -> Result<(), Error<E>> {
        let (cur_gpo, cur_it) = self.read_gpo_config()?;
        if cur_gpo != GPO_RF_WRITE_PULSE || cur_it != IT_TIME_301US {
            self.ensure_mailbox_inactive()?;
            self.open_i2c_session(password)?;
            let res = (|| -> Result<(), Error<E>> {
                let single_page =
                    Duration::from_millis(WRITE_MS_PER_PAGE + WRITE_TIMEOUT_MARGIN_MS);
                if cur_gpo != GPO_RF_WRITE_PULSE {
                    self.write_reg_chunk(ADDR_SYSTEM, sys_reg::GPO, &[GPO_RF_WRITE_PULSE])?;
                    self.wait_write_ready(ADDR_SYSTEM, single_page)?;
                }
                if cur_it != IT_TIME_301US {
                    self.write_reg_chunk(ADDR_SYSTEM, sys_reg::IT_TIME, &[IT_TIME_301US])?;
                    self.wait_write_ready(ADDR_SYSTEM, single_page)?;
                }
                Ok(())
            })();
            let _ = self.close_i2c_session(password);
            res?;
        }
        self.set_gpo_enabled_dyn(true)
    }

    /// Returns `true` if Area 1 (`RFA1SS` `0x0004`) currently forbids RF writes.
    pub fn is_rf_write_locked(&mut self) -> Result<bool, Error<E>> {
        let mut rfa1ss = [0u8; 1];
        self.read_reg(ADDR_SYSTEM, sys_reg::RFA1SS, &mut rfa1ss)?;
        Ok((rfa1ss[0] & 0x0C) == RFA1SS_RF_WRITE_FORBIDDEN)
    }

    /// Enables or disables hardware RF write protection on Area 1 (`RFA1SS` `0x0004` on `0x57`;
    /// DS10925 Table 44).
    ///
    /// - `locked = true` (`0x0C`): RF read always allowed, RF write always forbidden (I2C writes
    ///   remain unaffected).
    /// - `locked = false` (`0x00`): RF read and write open.
    ///
    /// Only writes static EEPROM when the value changes, and closes the I2C security session
    /// afterward.
    #[allow(dead_code)]
    pub fn set_rf_write_locked(&mut self, locked: bool, password: [u8; 8]) -> Result<(), Error<E>> {
        let target = if locked {
            RFA1SS_RF_WRITE_FORBIDDEN
        } else {
            RFA1SS_OPEN
        };
        let mut cur = [0u8; 1];
        self.read_reg(ADDR_SYSTEM, sys_reg::RFA1SS, &mut cur)?;
        if cur[0] == target {
            return Ok(());
        }
        self.ensure_mailbox_inactive()?;
        self.open_i2c_session(password)?;
        let res = (|| -> Result<(), Error<E>> {
            self.write_reg_chunk(ADDR_SYSTEM, sys_reg::RFA1SS, &[target])?;
            self.wait_write_ready(
                ADDR_SYSTEM,
                Duration::from_millis(WRITE_MS_PER_PAGE + WRITE_TIMEOUT_MARGIN_MS),
            )
        })();
        let _ = self.close_i2c_session(password);
        res
    }

    /// Sets the dynamic RF interface operating mode (`RF_MNGT_Dyn` `0x2003`; no password needed).
    #[allow(dead_code)]
    pub fn set_rf_mode(&mut self, mode: RfMode) -> Result<(), Error<E>> {
        self.write_reg_chunk(ADDR_USER, dyn_reg::RF_MNGT, &[mode as u8])
    }

    /// Enables Fast Transfer Mode (FTM) mailbox. Authorizes static `MB_MODE` via `password` if
    /// not already authorized, closes the security session, then sets dynamic `MB_EN = 1`.
    ///
    /// Remember: deactivate FTM via [`Self::disable_mailbox`] before any User/System EEPROM write.
    #[allow(dead_code)]
    pub fn enable_mailbox(&mut self, password: [u8; 8]) -> Result<(), Error<E>> {
        let mut mb_mode = [0u8; 1];
        self.read_reg(ADDR_SYSTEM, sys_reg::MB_MODE, &mut mb_mode)?;
        if (mb_mode[0] & 0x01) == 0 {
            self.ensure_mailbox_inactive()?;
            self.open_i2c_session(password)?;
            let res = (|| -> Result<(), Error<E>> {
                self.write_reg_chunk(ADDR_SYSTEM, sys_reg::MB_MODE, &[0x01])?;
                self.wait_write_ready(
                    ADDR_SYSTEM,
                    Duration::from_millis(WRITE_MS_PER_PAGE + WRITE_TIMEOUT_MARGIN_MS),
                )
            })();
            let _ = self.close_i2c_session(password);
            res?;
        }
        self.write_reg_chunk(ADDR_USER, dyn_reg::MB_CTRL, &[0x01])
    }

    /// Disables Fast Transfer Mode (`MB_EN = 0`), freeing the internal buffer for EEPROM writes.
    #[allow(dead_code)]
    pub fn disable_mailbox(&mut self) -> Result<(), Error<E>> {
        self.write_reg_chunk(ADDR_USER, dyn_reg::MB_CTRL, &[0x00])
    }

    /// Writes a message (1..=256 bytes) into the FTM mailbox RAM (`0x2008..`) for an RF reader.
    #[allow(dead_code)]
    pub fn write_mailbox(&mut self, data: &[u8]) -> Result<(), Error<E>> {
        if data.is_empty() || data.len() > MAILBOX_MAX_LEN {
            return Err(Error::OutOfRange);
        }
        let status = self.read_dynamic_status()?;
        if !status.mb_enabled() {
            return Err(Error::MailboxDisabled);
        }
        if status.host_put_msg() || status.rf_put_msg() {
            return Err(Error::MailboxBusy);
        }
        let mut frame = [0u8; 2 + MAILBOX_MAX_LEN];
        frame[0..2].copy_from_slice(&dyn_reg::MAILBOX_RAM.to_be_bytes());
        frame[2..2 + data.len()].copy_from_slice(data);
        self.i2c.write(ADDR_USER, &frame[..2 + data.len()])?;
        Ok(())
    }

    /// Reads a pending RF message from the FTM mailbox RAM into `buf` and returns its length.
    #[allow(dead_code)]
    pub fn read_mailbox(&mut self, buf: &mut [u8]) -> Result<usize, Error<E>> {
        let status = self.read_dynamic_status()?;
        if !status.mb_enabled() {
            return Err(Error::MailboxDisabled);
        }
        if !status.rf_put_msg() {
            return Err(Error::MailboxEmpty);
        }
        let mut len_reg = [0u8; 1];
        self.read_reg(ADDR_USER, dyn_reg::MB_LEN, &mut len_reg)?;
        let msg_len = (len_reg[0] as usize) + 1;
        if buf.len() < msg_len {
            return Err(Error::OutOfRange);
        }
        self.read_reg(ADDR_USER, dyn_reg::MAILBOX_RAM, &mut buf[..msg_len])?;
        Ok(msg_len)
    }

    /// Ensures the first `protected.len()` NDEF records on the tag match `protected`, preserving
    /// any additional trailing records (`protected.len()..`) written by an NFC reader.
    ///
    /// - If the tag already starts with `protected` (checked via [`ndef::record_matches`]),
    ///   EEPROM is not rewritten and `Ok(false)` is returned.
    /// - If the tag is empty, unformatted, malformed, or any leading record differs, rewrites
    ///   `[protected..., trailing_records...]` to EEPROM `0x0000..` and returns `Ok(true)`.
    /// - Returns [`Error::RfBusy`] if an RF field is currently active when a rewrite is needed.
    #[allow(dead_code)]
    pub fn sync_leading_records(
        &mut self,
        protected: &[ndef::WriteRecord<'_>],
    ) -> Result<bool, Error<E>> {
        const MAX_RECORDS: usize = 16;

        if protected.len() > MAX_RECORDS {
            return Err(Error::OutOfRange);
        }

        let mut raw_in = [0u8; ndef::MAX_FRAME_BYTES];
        let initial_len = 32usize;
        self.read_user_memory(0x0000, &mut raw_in[..initial_len])?;

        let read_len = match ndef::probe_ndef_frame_len(&raw_in[..initial_len]) {
            Ok(Some(needed)) => {
                if needed > ndef::MAX_FRAME_BYTES || needed > USER_MEM_BYTES {
                    return Err(Error::Ndef(NdefError::BufferTooSmall));
                }
                if needed > initial_len {
                    self.read_user_memory(initial_len as u16, &mut raw_in[initial_len..needed])?;
                }
                Some(needed)
            }
            Ok(None) | Err(NdefError::InvalidCc | NdefError::Malformed) => None,
            Err(e) => return Err(Error::Ndef(e)),
        };

        let mut combined = [ndef::WriteRecord::Raw {
            tnf: 0,
            rec_type: &[],
            payload: &[],
        }; MAX_RECORDS];
        combined[..protected.len()].copy_from_slice(protected);

        let mut leading_matches = false;
        let mut trailing_count = 0usize;

        if let Some(len) = read_len {
            if let Ok(iter) = ndef::iter_type5_records(&raw_in[..len]) {
                let mut matched = 0usize;
                let mut idx = 0usize;
                let mut valid = true;

                for item in iter {
                    let Ok(rec) = item else {
                        valid = false;
                        break;
                    };
                    if idx < protected.len() {
                        if ndef::record_matches(&rec, &protected[idx]) {
                            matched += 1;
                        }
                    } else if protected.len() + trailing_count < MAX_RECORDS {
                        combined[protected.len() + trailing_count] = ndef::WriteRecord::Raw {
                            tnf: rec.tnf,
                            rec_type: rec.rec_type,
                            payload: rec.payload,
                        };
                        trailing_count += 1;
                    }
                    idx += 1;
                }

                if valid && matched == protected.len() {
                    leading_matches = true;
                } else if !valid {
                    trailing_count = 0;
                }
            }
        }

        if leading_matches {
            return Ok(false);
        }

        if self.read_dynamic_status()?.field_on {
            return Err(Error::RfBusy);
        }

        let total_records = protected.len() + trailing_count;
        let mut frame = [0u8; ndef::MAX_FRAME_BYTES];
        let encoded_len = ndef::encode_type5_records(&combined[..total_records], &mut frame)
            .map_err(Error::Ndef)?;
        self.write_user_memory(0x0000, &frame[..encoded_len])?;
        Ok(true)
    }

    fn ensure_mailbox_inactive(&mut self) -> Result<(), Error<E>> {
        let mut mb_ctrl = [0u8; 1];
        self.read_reg(ADDR_USER, dyn_reg::MB_CTRL, &mut mb_ctrl)?;
        if (mb_ctrl[0] & 0x01) != 0 {
            return Err(Error::MailboxActive);
        }
        Ok(())
    }

    fn present_i2c_password(&mut self, password: [u8; 8]) -> Result<(), Error<E>> {
        let mut payload = [0u8; 19];
        payload[0..2].copy_from_slice(&sys_reg::I2C_PASSWD.to_be_bytes());
        payload[2..10].copy_from_slice(&password);
        payload[10] = 0x09; // Validation code for I2C Present Password (DS10925 Section 6.6.1)
        payload[11..19].copy_from_slice(&password);
        self.i2c.write(ADDR_SYSTEM, &payload)?;
        Ok(())
    }

    fn read_reg(&mut self, dev_addr: u8, reg: u16, buf: &mut [u8]) -> Result<(), Error<E>> {
        let addr_be = reg.to_be_bytes();
        self.i2c.write_read(dev_addr, &addr_be, buf)?;
        Ok(())
    }

    fn write_reg_chunk(&mut self, dev_addr: u8, reg: u16, data: &[u8]) -> Result<(), Error<E>> {
        let mut frame = [0u8; 2 + WRITE_CHUNK_MAX];
        if data.len() > WRITE_CHUNK_MAX {
            return Err(Error::OutOfRange);
        }
        frame[0..2].copy_from_slice(&reg.to_be_bytes());
        frame[2..2 + data.len()].copy_from_slice(data);
        self.i2c.write(dev_addr, &frame[..2 + data.len()])?;
        Ok(())
    }

    fn wait_write_ready(&mut self, dev_addr: u8, timeout: Duration) -> Result<(), Error<E>> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.i2c.write(dev_addr, &[]).is_ok() {
                return Ok(());
            }
            embassy_time::block_for(Duration::from_micros(500));
        }
        Err(Error::WriteTimeout)
    }
}

impl St25dv<SoftI2c<'static>> {
    /// Sleeps asynchronously with **zero I2C polling** until the ST25DV16K `GPO` pin (wired
    /// open-drain to `SCL` / `PC10`) pulls `PC10` low on a falling edge (`EXTI10`).
    ///
    /// Cancel-safe: dropping the future masks `EXTI10` so STM32-driven `SCL` clocks never
    /// re-trigger the interrupt.
    pub async fn wait_for_gpo_pulse(&mut self, _exti10: &mut Peri<'static, peripherals::EXTI10>) {
        let _guard = Exti10Guard;
        GPO_SIGNAL.reset();
        exti10_arm_pc10_falling();

        GPO_SIGNAL.wait().await;
        drop(_guard);

        // Wait for the 301 us open-drain GPO pulse to release SCL back HIGH before any I2C use.
        Timer::after_micros(500).await;
    }

    /// Zero-polling, RF-arbitration-safe wait for an RF phone to finish writing to EEPROM:
    /// 1. Sleeps on `EXTI10` (`GPO -> SCL` falling edge from `RF_WRITE`).
    /// 2. Debounces subsequent multi-block `GPO` pulses until 200 ms of quiet with **zero I2C
    ///    traffic** during the write burst (prevents DS10925 Section 5.5 I2C-vs-RF `0x0F` busy
    ///    collisions while a phone is writing multiple blocks).
    /// 3. Once quiet, verifies `RF_WRITE` in `IT_STS_Dyn` and waits gently for `!field_on`
    ///    (phone lifted away) before returning `Ok(())`.
    pub async fn wait_for_rf_write(
        &mut self,
        exti10: &mut Peri<'static, peripherals::EXTI10>,
    ) -> Result<(), Error<SoftI2cError>> {
        loop {
            if !self.pending_rf_write {
                // Sleep at 0 uA until first RF_WRITE GPO pulse.
                self.wait_for_gpo_pulse(exti10).await;

                // Debounce multi-block writes without touching I2C so RF never sees 0x0F I2C busy.
                while self
                    .wait_for_gpo_pulse(exti10)
                    .with_timeout(Duration::from_millis(200))
                    .await
                    .is_ok()
                {}
            }

            // Burst finished; confirm RF_WRITE and wait until phone leaves the RF field.
            loop {
                match self.read_dynamic_status() {
                    Ok(st) if self.pending_rf_write && !st.field_on => return Ok(()),
                    Ok(st) if !st.field_on => break, // Spurious edge without RF_WRITE; re-arm
                    Ok(_) => {}                      // Phone still resting on antenna
                    Err(_) => {}                     // Brief RF contention; retry shortly
                }
                Timer::after_millis(150).await;
            }
        }
    }

    /// Generic interrupt-driven RF-write handler.
    ///
    /// Sleeps on `EXTI10` (`GPO -> SCL`, zero I2C polling) via [`Self::wait_for_rf_write`],
    /// then invokes `on_rf_write(self)` once the RF write burst and RF field have ended so higher
    /// layers can inspect updated EEPROM contents and rewrite any protected records/regions.
    /// Clears the internal `pending_rf_write` latch upon success.
    #[allow(dead_code)]
    pub async fn watch_rf_write<F, T>(
        &mut self,
        exti10: &mut Peri<'static, peripherals::EXTI10>,
        mut on_rf_write: F,
    ) -> Result<T, Error<SoftI2cError>>
    where
        F: FnMut(&mut Self) -> Result<T, Error<SoftI2cError>>,
    {
        self.wait_for_rf_write(exti10).await?;
        let res = on_rf_write(self);
        if res.is_ok() {
            self.pending_rf_write = false;
        }
        res
    }

    /// Generic interrupt-driven rewrite protection for a raw EEPROM byte range.
    ///
    /// Waits on `EXTI10` (`GPO -> SCL`, zero I2C polling) for an RF write to finish, compares
    /// `EEPROM[addr..addr + expected.len()]` against `expected`, and rewrites `expected` if any
    /// byte was modified. Returns `Ok(true)` if a restore write was performed.
    ///
    /// Note: for protecting leading NDEF records while still allowing phones to append extra
    /// trailing records (record 2, 3, ...), use [`Self::protect_ndef_records_on_rf_write`]
    /// instead (raw byte protection includes the NDEF TLV length, `ME` flag, and `0xFE`
    /// terminator).
    #[allow(dead_code)]
    pub async fn protect_region_on_rf_write(
        &mut self,
        exti10: &mut Peri<'static, peripherals::EXTI10>,
        addr: u16,
        expected: &[u8],
    ) -> Result<bool, Error<SoftI2cError>> {
        if (addr as usize) + expected.len() > USER_MEM_BYTES {
            return Err(Error::OutOfRange);
        }
        self.watch_rf_write(exti10, |nfc| {
            let mut offset = 0usize;
            let mut buf = [0u8; 64];
            let mut mismatch = false;
            while offset < expected.len() {
                let n = (expected.len() - offset).min(buf.len());
                nfc.read_user_memory(addr + offset as u16, &mut buf[..n])?;
                if buf[..n] != expected[offset..offset + n] {
                    mismatch = true;
                    break;
                }
                offset += n;
            }
            if mismatch {
                nfc.write_user_memory(addr, expected)?;
                Ok(true)
            } else {
                Ok(false)
            }
        })
        .await
    }

    /// Generic interrupt-driven rewrite protection for leading NDEF records.
    ///
    /// Sleeps on `EXTI10` (`GPO -> SCL`, zero I2C polling) via [`Self::watch_rf_write`], then
    /// calls [`Self::sync_leading_records`] once the RF write and RF field finish:
    /// - If the leading `protected.len()` records still match `protected`, does nothing and
    ///   returns `Ok(false)` (leaving any extra trailing records untouched).
    /// - If any leading record was modified or removed, restores `protected` while preserving
    ///   all trailing records (`protected.len()..`) and returns `Ok(true)`.
    #[allow(dead_code)]
    pub async fn protect_ndef_records_on_rf_write(
        &mut self,
        exti10: &mut Peri<'static, peripherals::EXTI10>,
        protected: &[ndef::WriteRecord<'_>],
    ) -> Result<bool, Error<SoftI2cError>> {
        self.watch_rf_write(exti10, |nfc| nfc.sync_leading_records(protected))
            .await
    }
}

/// Initializes `SoftI2c` on `PC10`/`PC11`, auto-detecting whether `SCL` and `SDA` are swapped,
/// and returns a ready `St25dv<SoftI2c<'static>>` driver alongside a `swapped` flag.
pub async fn init_auto_detect(
    pc10: Peri<'static, peripherals::PC10>,
    pc11: Peri<'static, peripherals::PC11>,
) -> Option<(St25dv<SoftI2c<'static>>, bool)> {
    let mut i2c = SoftI2c::new(Flex::new(pc10), Flex::new(pc11));
    Timer::after_millis(5).await;

    let (scl_hi, sda_hi) = i2c.line_levels();
    info!(
        "ST25DV SoftI2C idle levels (PC10=SCL, PC11=SDA): SCL={} SDA={}",
        scl_hi, sda_hi
    );

    if matches!(i2c.probe(ADDR_USER), Ok(true)) || matches!(i2c.probe(ADDR_SYSTEM), Ok(true)) {
        info!("ST25DV detected with PC10=SCL, PC11=SDA (normal orientation)");
        return Some((St25dv::new(i2c), false));
    }

    warn!("ST25DV no ACK on (PC10=SCL, PC11=SDA); trying swapped (PC11=SCL, PC10=SDA)...");
    i2c.swap_pins();
    Timer::after_millis(5).await;

    if matches!(i2c.probe(ADDR_USER), Ok(true)) || matches!(i2c.probe(ADDR_SYSTEM), Ok(true)) {
        info!("ST25DV detected with PC11=SCL, PC10=SDA (SWAPPED orientation!)");
        return Some((St25dv::new(i2c), true));
    }

    // Full 7-bit bus scan on both orientations to aid wiring diagnostics if 0x53/0x57 didn't ACK.
    warn!("ST25DV not found at 0x53/0x57; scanning both orientations...");
    for (label, swapped) in [("PC11=SCL, PC10=SDA", true), ("PC10=SCL, PC11=SDA", false)] {
        if !swapped {
            i2c.swap_pins();
        }
        let mut found = 0u8;
        for addr in 0x08..=0x77u8 {
            if matches!(i2c.probe(addr), Ok(true)) {
                info!("SoftI2C ({}) ACK at 0x{:02X}", label, addr);
                found += 1;
            }
        }
        if found == 0 {
            warn!("SoftI2C ({}): no I2C ACKs found", label);
        }
    }
    None
}

/// Runs a hardware bring-up check on the ST25DV16K: auto-detects SCL/SDA orientation, logs
/// device info & dynamic status, configures `GPO -> SCL` (`RF_WRITE` 301 us pulse), and reports
/// Area 1 RF write-lock state.
pub async fn run_bringup_test(
    pc10: Peri<'static, peripherals::PC10>,
    pc11: Peri<'static, peripherals::PC11>,
) -> Option<St25dv<SoftI2c<'static>>> {
    info!("=== ST25DV16K Bring-Up Test (SoftI2C PC10/PC11) ===");
    let (mut nfc, swapped) = init_auto_detect(pc10, pc11).await?;

    match nfc.read_device_info() {
        Ok(di) => {
            info!(
                "ST25DV OK (swapped={}): IC_REF=0x{:02X}, IC_REV=0x{:02X}, MEM={}B (blk={}B)",
                swapped, di.ic_ref, di.ic_rev, di.mem_bytes, di.block_size
            );
            info!("ST25DV UID: {=[u8]:02x}", di.uid);
        }
        Err(e) => {
            error!("ST25DV read_device_info failed: {:?}", e);
            return None;
        }
    }

    match nfc.configure_gpo_rf_write(DEFAULT_I2C_PASSWORD) {
        Ok(()) => {
            if let (Ok((gpo, it_time)), Ok(gpo_dyn)) = (nfc.read_gpo_config(), nfc.read_gpo_dyn()) {
                info!(
                    "ST25DV GPO (RF_WRITE pulse): GPO=0x{:02X}, IT_TIME=0x{:02X}, GPO_DYN=0x{:02X}",
                    gpo, it_time, gpo_dyn
                );
            }
        }
        Err(e) => warn!("ST25DV configure_gpo_rf_write failed: {:?}", e),
    }

    match nfc.read_dynamic_status() {
        Ok(st) => info!(
            "ST25DV status: vcc={}, field={}, eh={}, sso={}, rf_mngt=0x{:02X}, mb_ctrl=0x{:02X}, it_sts=0x{:02X}",
            st.vcc_on,
            st.field_on,
            st.eh_on,
            st.i2c_session_open,
            st.rf_mngt,
            st.mb_ctrl,
            st.it_sts
        ),
        Err(e) => warn!("ST25DV read_dynamic_status failed: {:?}", e),
    }

    match nfc.is_rf_write_locked() {
        Ok(locked) => info!("ST25DV Area 1 RF write locked: {}", locked),
        Err(e) => warn!("ST25DV is_rf_write_locked failed: {:?}", e),
    }

    info!("=== ST25DV16K Bring-Up Test Complete ===");
    Some(nfc)
}
