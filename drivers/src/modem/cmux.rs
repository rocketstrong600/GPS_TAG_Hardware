//! 3GPP TS 27.010 Basic Option CMUX multiplexer for the Quectel EC21-AU.
//!
//! Splits `USART6` (`460800 bps`, 4 KiB DMA RX ring buffer) into `DLCI 0` (control), `DLCI 1`
//! (AT/GPS, 2 KiB RX pipe), and `DLCI 2` (PPP, 8 KiB RX pipe).
//!
//! ## Key Features
//! - **Bidirectional per-DLCI flow control (`MSC` `FC` bit + `FCon`/`FCoff`):** Pauses TX when
//!   the modem asserts `FC=1` and asserts `MSC(FC=1)` backpressure when our RX pipe fills instead
//!   of dropping PPP bytes (~34 KB/s lossless throughput at `460800 bps`).
//! - **Shared `0xF9` flag support & upfront header validation:** Handles back-to-back frames
//!   sharing a single delimiter and rejects false `0xF9` flags before reading bogus payloads.
//! - **EC21-AU `PppDeframer`:** De-interleaves small PPP frames the EC21 CMUX layer inserts
//!   mid-packet and validates PPP FCS-16 before passing frames to `embassy-net-ppp`.
//! - **Single-DMA TX & chunked RX:** Assembles each CMUX frame in one contiguous `write_all`
//!   transfer and reads UART RX in 256-byte chunks.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use defmt::{debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, WithTimeout};
use embedded_io_async::{BufRead, ErrorType, Read, Write};

/// Default TS 27.010 Basic Option N1 (max info frame size) for `AT+CMUX=0`.
pub const MAX_INFO_LEN: usize = 127;

const FLAG: u8 = 0xF9;
const EA: u8 = 0x01;
const CR: u8 = 0x02;
const PF: u8 = 0x10;

const CTRL_SABM: u8 = 0x2F;
const CTRL_UA: u8 = 0x63;
const CTRL_DM: u8 = 0x0F;
const CTRL_DISC: u8 = 0x43;
const CTRL_UIH: u8 = 0xEF;
const CTRL_UI: u8 = 0x03;

// TS 27.010 DLCI 0 control message types (upper 6 bits, C/R and EA masked off)
const MSG_NSC: u8 = 0x10;
const MSG_TEST: u8 = 0x20;
const MSG_FCOFF: u8 = 0x60;
const MSG_FCON: u8 = 0xA0;
const MSG_CLD: u8 = 0xC0;
const MSG_MSC: u8 = 0xE0;

// V.24 control octet bits in MSC
const V24_EA: u8 = 0x01;
const V24_FC: u8 = 0x02;
const V24_RTC: u8 = 0x04;
const V24_RTR: u8 = 0x08;
const V24_DV: u8 = 0x80;

pub const DLCI_CTRL: u8 = 0;
pub const DLCI_AT: u8 = 1;
pub const DLCI_PPP: u8 = 2;
const MAX_VALID_DLCI: u8 = 16;

const AT_PIPE_SIZE: usize = 2048;
const PPP_PIPE_SIZE: usize = 8192;
const MAX_FRAME_INFO: usize = 512;
const UART_RX_CHUNK: usize = 256;
const MAX_CONSECUTIVE_ERRORS: u32 = 10;

struct DlciState {
    opened: AtomicBool,
    /// Set when the modem sends `MSC(FC=1)` asking us to pause TX on this DLCI.
    tx_paused: AtomicBool,
    /// Tracks V.24 `DV` (Data Valid / Carrier Detect) reported by the modem.
    carrier_dv: AtomicBool,
    tx_resume: Signal<CriticalSectionRawMutex, ()>,
}

impl DlciState {
    const fn new() -> Self {
        Self {
            opened: AtomicBool::new(false),
            tx_paused: AtomicBool::new(false),
            carrier_dv: AtomicBool::new(false),
            tx_resume: Signal::new(),
        }
    }

    fn reset(&self) {
        self.opened.store(false, Ordering::Relaxed);
        self.tx_paused.store(false, Ordering::Relaxed);
        self.carrier_dv.store(false, Ordering::Relaxed);
        self.tx_resume.signal(());
    }
}

static UART_TX: Mutex<CriticalSectionRawMutex, Option<UartTx<'static, Async>>> = Mutex::new(None);
static DLCI1_PIPE: Pipe<CriticalSectionRawMutex, AT_PIPE_SIZE> = Pipe::new();
static DLCI2_PIPE: Pipe<CriticalSectionRawMutex, PPP_PIPE_SIZE> = Pipe::new();
static CTRL_ACK: Signal<CriticalSectionRawMutex, (u8, u8)> = Signal::new();
static PPP_FRAMING_ACTIVE: AtomicBool = AtomicBool::new(false);
static PPP_DEFRAMER_RESET: AtomicBool = AtomicBool::new(false);
static CTRL_OPENED: AtomicBool = AtomicBool::new(false);
static GLOBAL_TX_PAUSED: AtomicBool = AtomicBool::new(false);
static CONSECUTIVE_ERRORS: AtomicU32 = AtomicU32::new(0);
static DLCI_STATES: [DlciState; 2] = [DlciState::new(), DlciState::new()];

fn dlci_state(dlci: u8) -> Option<&'static DlciState> {
    match dlci {
        DLCI_AT => Some(&DLCI_STATES[0]),
        DLCI_PPP => Some(&DLCI_STATES[1]),
        _ => None,
    }
}

/// Enables or disables PPP frame de-interleaving and FCS-16 validation on `DLCI 2`.
/// Must be `false` while dialing (`ATD*99#`) and `true` once PPP data mode begins.
pub fn set_ppp_framing(active: bool) {
    PPP_DEFRAMER_RESET.store(true, Ordering::Release);
    PPP_FRAMING_ACTIVE.store(active, Ordering::Release);
}

/// Returns `true` if `dlci` (`1` or `2`) is currently open.
#[allow(dead_code)]
pub fn is_dlci_open(dlci: u8) -> bool {
    match dlci {
        DLCI_CTRL => CTRL_OPENED.load(Ordering::Relaxed),
        _ => dlci_state(dlci).is_some_and(|s| s.opened.load(Ordering::Relaxed)),
    }
}

/// Returns the modem's V.24 `DV` (Data Valid / Carrier Detect) state for `dlci`.
#[allow(dead_code)]
pub fn carrier_detected(dlci: u8) -> bool {
    dlci_state(dlci).is_some_and(|s| s.carrier_dv.load(Ordering::Relaxed))
}

/// Returns the current consecutive CMUX framing/FCS error count (resets to `0` on any valid frame).
#[allow(dead_code)]
pub fn consecutive_errors() -> u32 {
    CONSECUTIVE_ERRORS.load(Ordering::Relaxed)
}

#[rustfmt::skip]
const FCS_TABLE: [u8; 256] = [
    0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
    0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
    0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
    0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
    0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
    0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
    0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
    0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
    0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
    0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
    0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
    0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
    0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
    0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
    0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
    0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
];

fn fcs_update(mut fcs: u8, data: &[u8]) -> u8 {
    for &b in data {
        fcs = FCS_TABLE[(fcs ^ b) as usize];
    }
    fcs
}

fn calc_fcs(header: &[u8], extra_payload: Option<&[u8]>) -> u8 {
    let mut fcs = fcs_update(0xFF, header);
    if let Some(p) = extra_payload {
        fcs = fcs_update(fcs, p);
    }
    0xFF - fcs
}

fn is_valid_base_ctrl(base_ctrl: u8) -> bool {
    matches!(
        base_ctrl,
        CTRL_SABM | CTRL_UA | CTRL_DM | CTRL_DISC | CTRL_UIH | CTRL_UI
    )
}

#[derive(Debug, Copy, Clone, defmt::Format)]
pub struct CmuxError;

impl core::fmt::Display for CmuxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("CMUX error")
    }
}

impl core::error::Error for CmuxError {}

impl embedded_io_async::Error for CmuxError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

/// Waits if the modem has asserted flow control (`MSC FC=1` or `FCoff`) on `dlci`.
async fn wait_tx_allowed(dlci: u8) {
    let Some(state) = dlci_state(dlci) else {
        return;
    };
    while state.tx_paused.load(Ordering::Relaxed) || GLOBAL_TX_PAUSED.load(Ordering::Relaxed) {
        if state
            .tx_resume
            .wait()
            .with_timeout(Duration::from_secs(2))
            .await
            .is_err()
        {
            warn!("CMUX DLCI {} TX flow-control timeout; resuming", dlci);
            break;
        }
    }
}

/// Sends a complete TS 27.010 Basic Option frame in a single contiguous UART DMA write.
/// Supports both 1-byte (`< 128`) and 2-byte (`>= 128`) length headers.
async fn send_raw_frame(dlci: u8, ctrl: u8, info: &[u8]) -> Result<(), CmuxError> {
    if info.len() > MAX_FRAME_INFO {
        return Err(CmuxError);
    }

    let mut frame = [0u8; 7 + MAX_FRAME_INFO];
    frame[0] = FLAG;
    frame[1] = (dlci << 2) | CR | EA; // TE command / response with EA=1
    frame[2] = ctrl;

    let hdr_end = if info.len() < 128 {
        frame[3] = ((info.len() as u8) << 1) | EA;
        4
    } else {
        let [b1, b2] = ((info.len() as u16) << 1).to_le_bytes();
        frame[3] = b1; // EA=0
        frame[4] = b2;
        5
    };

    let fcs = calc_fcs(
        &frame[1..hdr_end],
        if (ctrl & !PF) == CTRL_UI {
            Some(info)
        } else {
            None
        },
    );

    let mut idx = hdr_end;
    if !info.is_empty() {
        frame[idx..idx + info.len()].copy_from_slice(info);
        idx += info.len();
    }
    frame[idx] = fcs;
    frame[idx + 1] = FLAG;
    let total = idx + 2;

    let mut guard = UART_TX.lock().await;
    let tx = guard.as_mut().ok_or(CmuxError)?;
    tx.write_all(&frame[..total]).await.map_err(|_| CmuxError)?;
    Ok(())
}

/// Sends a TS 27.010 `MSC` command on `DLCI 0` to pause (`fc_on=true`) or resume
/// (`fc_on=false`) incoming frames from the modem on `dlci`.
async fn send_msc_fc(dlci: u8, fc_on: bool) -> Result<(), CmuxError> {
    let v24 = if fc_on {
        V24_EA | V24_FC | V24_RTC | V24_DV
    } else {
        V24_EA | V24_RTC | V24_RTR | V24_DV
    };
    let payload = [
        MSG_MSC | CR | EA, // 0xE3: MSC command
        (2 << 1) | EA,     // 0x05: length = 2 bytes
        (dlci << 2) | CR | EA,
        v24,
    ];
    send_raw_frame(DLCI_CTRL, CTRL_UIH, &payload).await
}

/// Starts the TS 27.010 multiplexer task and returns `(at_channel, ppp_channel)`
/// after opening `DLCI 0` (control), `DLCI 1` (AT/GPS), and `DLCI 2` (PPP).
pub async fn start(
    spawner: &Spawner,
    tx: UartTx<'static, Async>,
    rx: RingBufferedUartRx<'static>,
) -> Result<(CmuxChannel<AT_PIPE_SIZE>, CmuxChannel<PPP_PIPE_SIZE>), CmuxError> {
    CTRL_OPENED.store(false, Ordering::Relaxed);
    GLOBAL_TX_PAUSED.store(false, Ordering::Relaxed);
    CONSECUTIVE_ERRORS.store(0, Ordering::Relaxed);
    for st in &DLCI_STATES {
        st.reset();
    }

    {
        let mut guard = UART_TX.lock().await;
        *guard = Some(tx);
    }

    spawner.spawn(cmux_rx_task(rx).unwrap());

    open_dlci(DLCI_CTRL).await?;
    open_dlci(DLCI_AT).await?;
    open_dlci(DLCI_PPP).await?;

    Ok((
        CmuxChannel::new(DLCI_AT, &DLCI1_PIPE),
        CmuxChannel::new(DLCI_PPP, &DLCI2_PIPE),
    ))
}

/// Opens a TS 27.010 logical channel (`SABM` -> `UA`).
pub async fn open_dlci(dlci: u8) -> Result<(), CmuxError> {
    for _ in 0..3 {
        CTRL_ACK.reset();
        send_raw_frame(dlci, CTRL_SABM | PF, &[]).await?;
        if let Ok((ack_dlci, ctrl)) = CTRL_ACK
            .wait()
            .with_timeout(Duration::from_millis(1000))
            .await
        {
            if ack_dlci == dlci && (ctrl & !PF) == CTRL_UA {
                if dlci == DLCI_CTRL {
                    CTRL_OPENED.store(true, Ordering::Relaxed);
                } else if let Some(st) = dlci_state(dlci) {
                    st.opened.store(true, Ordering::Relaxed);
                }
                debug!("CMUX DLCI {} opened", dlci);
                return Ok(());
            }
        }
    }
    error!("CMUX failed to open DLCI {}", dlci);
    Err(CmuxError)
}

#[allow(dead_code)]
pub async fn close_dlci(dlci: u8) -> Result<(), CmuxError> {
    send_raw_frame(dlci, CTRL_DISC | PF, &[]).await?;
    if dlci == DLCI_CTRL {
        CTRL_OPENED.store(false, Ordering::Relaxed);
    } else if let Some(st) = dlci_state(dlci) {
        st.opened.store(false, Ordering::Relaxed);
    }
    Ok(())
}

/// Cleanly closes open logical channels (`DISC`) and sends Multiplexer Close Down (`CLD`)
/// on `DLCI 0`.
#[allow(dead_code)]
pub async fn shutdown() -> Result<(), CmuxError> {
    for dlci in [DLCI_PPP, DLCI_AT] {
        if is_dlci_open(dlci) {
            let _ = close_dlci(dlci).await;
        }
    }
    if CTRL_OPENED.load(Ordering::Relaxed) {
        let _ = send_raw_frame(DLCI_CTRL, CTRL_UIH, &[MSG_CLD | CR | EA, EA]).await;
        CTRL_OPENED.store(false, Ordering::Relaxed);
    }
    Ok(())
}

/// Writes raw bytes to `DLCI 1` (AT channel), chunking to `MAX_INFO_LEN` and honoring flow control.
#[allow(dead_code)]
pub async fn write_at_raw(mut data: &[u8]) -> Result<(), CmuxError> {
    while !data.is_empty() {
        wait_tx_allowed(DLCI_AT).await;
        let n = data.len().min(MAX_INFO_LEN);
        send_raw_frame(DLCI_AT, CTRL_UIH, &data[..n]).await?;
        data = &data[n..];
    }
    Ok(())
}

/// Virtual serial channel over a TS 27.010 DLCI.
/// Implements `embedded_io_async::{Read, BufRead, Write}` for `atat` and `embassy-net-ppp`.
pub struct CmuxChannel<const BUF: usize> {
    dlci: u8,
    pipe: &'static Pipe<CriticalSectionRawMutex, BUF>,
    buf: [u8; 128],
    pos: usize,
    len: usize,
}

#[allow(dead_code)]
pub type AtChannel = CmuxChannel<AT_PIPE_SIZE>;
pub type AtRx = CmuxRx<AT_PIPE_SIZE>;
pub type PppChannel = CmuxChannel<PPP_PIPE_SIZE>;

impl<const BUF: usize> CmuxChannel<BUF> {
    fn new(dlci: u8, pipe: &'static Pipe<CriticalSectionRawMutex, BUF>) -> Self {
        Self {
            dlci,
            pipe,
            buf: [0u8; 128],
            pos: 0,
            len: 0,
        }
    }

    pub fn split(self) -> (CmuxRx<BUF>, CmuxTx) {
        (
            CmuxRx {
                pipe: self.pipe,
                buf: self.buf,
                pos: self.pos,
                len: self.len,
            },
            CmuxTx { dlci: self.dlci },
        )
    }
}

pub struct CmuxRx<const BUF: usize> {
    pipe: &'static Pipe<CriticalSectionRawMutex, BUF>,
    buf: [u8; 128],
    pos: usize,
    len: usize,
}

impl<const BUF: usize> ErrorType for CmuxRx<BUF> {
    type Error = CmuxError;
}

impl<const BUF: usize> Read for CmuxRx<BUF> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if self.pos < self.len {
            let avail = self.len - self.pos;
            let n = avail.min(buf.len());
            buf[..n].copy_from_slice(&self.buf[self.pos..self.pos + n]);
            self.pos += n;
            return Ok(n);
        }
        Ok(self.pipe.read(buf).await)
    }
}

pub struct CmuxTx {
    dlci: u8,
}

impl ErrorType for CmuxTx {
    type Error = CmuxError;
}

impl Write for CmuxTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        wait_tx_allowed(self.dlci).await;
        let n = buf.len().min(MAX_INFO_LEN);
        send_raw_frame(self.dlci, CTRL_UIH, &buf[..n]).await?;
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<const BUF: usize> ErrorType for CmuxChannel<BUF> {
    type Error = CmuxError;
}

impl<const BUF: usize> Read for CmuxChannel<BUF> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if self.pos < self.len {
            let avail = self.len - self.pos;
            let n = avail.min(buf.len());
            buf[..n].copy_from_slice(&self.buf[self.pos..self.pos + n]);
            self.pos += n;
            return Ok(n);
        }
        Ok(self.pipe.read(buf).await)
    }
}

impl<const BUF: usize> BufRead for CmuxChannel<BUF> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        if self.pos >= self.len {
            self.len = self.pipe.read(&mut self.buf).await;
            self.pos = 0;
        }
        Ok(&self.buf[self.pos..self.len])
    }

    fn consume(&mut self, amt: usize) {
        self.pos = (self.pos + amt).min(self.len);
    }
}

impl<const BUF: usize> Write for CmuxChannel<BUF> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        wait_tx_allowed(self.dlci).await;
        let n = buf.len().min(MAX_INFO_LEN);
        send_raw_frame(self.dlci, CTRL_UIH, &buf[..n]).await?;
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Chunked reader over `RingBufferedUartRx` that avoids per-byte async reads.
struct UartBufReader {
    rx: RingBufferedUartRx<'static>,
    buf: [u8; UART_RX_CHUNK],
    pos: usize,
    len: usize,
}

impl UartBufReader {
    fn new(rx: RingBufferedUartRx<'static>) -> Self {
        Self {
            rx,
            buf: [0u8; UART_RX_CHUNK],
            pos: 0,
            len: 0,
        }
    }

    async fn fill(&mut self) -> Result<(), embassy_stm32::usart::Error> {
        loop {
            match self.rx.read(&mut self.buf).await {
                Ok(0) => {}
                Ok(n) => {
                    self.pos = 0;
                    self.len = n;
                    return Ok(());
                }
                Err(e) => return Err(e),
            }
        }
    }

    async fn next_byte(&mut self) -> Result<u8, embassy_stm32::usart::Error> {
        if self.pos >= self.len {
            self.fill().await?;
        }
        let b = self.buf[self.pos];
        self.pos += 1;
        Ok(b)
    }

    async fn read_exact(&mut self, mut dst: &mut [u8]) -> Result<(), embassy_stm32::usart::Error> {
        while !dst.is_empty() {
            if self.pos >= self.len {
                self.fill().await?;
            }
            let avail = self.len - self.pos;
            let n = avail.min(dst.len());
            dst[..n].copy_from_slice(&self.buf[self.pos..self.pos + n]);
            self.pos += n;
            dst = &mut dst[n..];
        }
        Ok(())
    }
}

fn record_rx_error() {
    let errs = CONSECUTIVE_ERRORS.fetch_add(1, Ordering::Relaxed) + 1;
    if errs == MAX_CONSECUTIVE_ERRORS {
        error!(
            "CMUX reached {} consecutive frame errors; stream may be desynced",
            errs
        );
    }
}

#[embassy_executor::task]
async fn cmux_rx_task(rx: RingBufferedUartRx<'static>) {
    let mut reader = UartBufReader::new(rx);
    // TS 27.010 Basic Option permits consecutive frames to share a single 0xF9 flag.
    let mut have_opening_flag = false;
    let mut ppp_deframer = PppDeframer::new();
    let mut info = [0u8; MAX_FRAME_INFO];

    loop {
        // 1. Hunt for opening 0xF9 flag (unless previous frame's closing flag serves as opening flag)
        if !have_opening_flag {
            let mut rdy_win = [0u8; 3];
            loop {
                match reader.next_byte().await {
                    Ok(FLAG) => break,
                    Ok(b) => {
                        rdy_win.copy_within(1.., 0);
                        rdy_win[2] = b;
                        if &rdy_win == b"RDY" {
                            error!("CMUX lost sync: modem emitted raw RDY (unexpected reboot)");
                        }
                    }
                    Err(e) => warn!("CMUX UART RX err (hunt): {:?}", e),
                }
            }
        }
        have_opening_flag = false;

        // 2. Skip consecutive 0xF9 flags; first non-flag byte is Address
        let addr = loop {
            match reader.next_byte().await {
                Ok(FLAG) => {}
                Ok(b) => break b,
                Err(e) => {
                    warn!("CMUX UART RX err (addr): {:?}", e);
                    break FLAG;
                }
            }
        };
        if addr == FLAG {
            continue;
        }

        // 3. Read Control + first Length byte
        let mut hdr_tail = [0u8; 2];
        if let Err(e) = reader.read_exact(&mut hdr_tail).await {
            warn!("CMUX UART RX err (hdr): {:?}", e);
            continue;
        }
        let ctrl = hdr_tail[0];
        let len1 = hdr_tail[1];
        let dlci = addr >> 2;
        let base_ctrl = ctrl & !PF;

        // Upfront header validation catches false 0xF9 flags before consuming bogus payload bytes.
        if (addr & EA) == 0 || dlci > MAX_VALID_DLCI || !is_valid_base_ctrl(base_ctrl) {
            warn!(
                "CMUX invalid header (false FLAG?): addr=0x{:02X} ctrl=0x{:02X} len1=0x{:02X}",
                addr, ctrl, len1
            );
            record_rx_error();
            continue;
        }

        // 4. Decode 1-byte or 2-byte length
        let mut header = [addr, ctrl, len1, 0u8];
        let (header_len, info_len) = if (len1 & EA) != 0 {
            (3usize, (len1 >> 1) as usize)
        } else {
            let len2 = match reader.next_byte().await {
                Ok(b) => b,
                Err(e) => {
                    warn!("CMUX UART RX err (len2): {:?}", e);
                    continue;
                }
            };
            header[3] = len2;
            (4usize, ((len1 >> 1) as usize) | ((len2 as usize) << 7))
        };

        if info_len > MAX_FRAME_INFO {
            warn!("CMUX frame too large ({} B); resyncing", info_len);
            record_rx_error();
            continue;
        }

        // 5. Information field (bulk read)
        if info_len > 0 {
            if let Err(e) = reader.read_exact(&mut info[..info_len]).await {
                warn!("CMUX UART RX err (info): {:?}", e);
                record_rx_error();
                continue;
            }
        }

        // 6. FCS + closing 0xF9 flag
        let mut trailer = [0u8; 2];
        if let Err(e) = reader.read_exact(&mut trailer).await {
            warn!("CMUX UART RX err (trailer): {:?}", e);
            record_rx_error();
            continue;
        }
        let rx_fcs = trailer[0];
        let closing = trailer[1];

        if closing == FLAG {
            have_opening_flag = true;
        } else {
            warn!(
                "CMUX bad closing flag: 0x{:02X} (dlci={}, ctrl=0x{:02X}, len={})",
                closing, dlci, ctrl, info_len
            );
            record_rx_error();
            continue;
        }

        let expected_fcs = calc_fcs(
            &header[..header_len],
            if base_ctrl == CTRL_UI {
                Some(&info[..info_len])
            } else {
                None
            },
        );
        if rx_fcs != expected_fcs {
            warn!(
                "CMUX FCS mismatch on DLCI {}: got 0x{:02X}, expected 0x{:02X}",
                dlci, rx_fcs, expected_fcs
            );
            record_rx_error();
            continue;
        }

        CONSECUTIVE_ERRORS.store(0, Ordering::Relaxed);

        // 7. Dispatch validated frame
        match base_ctrl {
            CTRL_UA | CTRL_DM => {
                CTRL_ACK.signal((dlci, ctrl));
            }
            CTRL_SABM => {
                if dlci == DLCI_CTRL {
                    CTRL_OPENED.store(true, Ordering::Relaxed);
                    let _ = send_raw_frame(DLCI_CTRL, CTRL_UA | PF, &[]).await;
                } else if let Some(st) = dlci_state(dlci) {
                    st.opened.store(true, Ordering::Relaxed);
                    let _ = send_raw_frame(dlci, CTRL_UA | PF, &[]).await;
                } else {
                    let _ = send_raw_frame(dlci, CTRL_DM | PF, &[]).await;
                }
            }
            CTRL_DISC => {
                if dlci == DLCI_CTRL {
                    let was_open = CTRL_OPENED.swap(false, Ordering::Relaxed);
                    for st in &DLCI_STATES {
                        st.reset();
                    }
                    let resp = if was_open { CTRL_UA | PF } else { CTRL_DM | PF };
                    let _ = send_raw_frame(DLCI_CTRL, resp, &[]).await;
                    warn!("CMUX remote closed DLCI 0 (DISC)");
                } else if let Some(st) = dlci_state(dlci) {
                    let was_open = st.opened.swap(false, Ordering::Relaxed);
                    let resp = if was_open { CTRL_UA | PF } else { CTRL_DM | PF };
                    let _ = send_raw_frame(dlci, resp, &[]).await;
                    warn!("CMUX remote closed DLCI {} (DISC)", dlci);
                } else {
                    let _ = send_raw_frame(dlci, CTRL_DM | PF, &[]).await;
                }
            }
            CTRL_UIH | CTRL_UI => {
                let payload = &info[..info_len];
                match dlci {
                    DLCI_CTRL => handle_ctrl_uih(payload).await,
                    DLCI_AT => write_pipe_with_fc(&DLCI1_PIPE, payload, DLCI_AT).await,
                    DLCI_PPP => {
                        if PPP_DEFRAMER_RESET.swap(false, Ordering::Acquire) {
                            ppp_deframer.reset();
                        }
                        if PPP_FRAMING_ACTIVE.load(Ordering::Acquire) {
                            ppp_deframer.push_chunk(payload, &DLCI2_PIPE).await;
                        } else {
                            ppp_deframer.reset();
                            write_pipe_with_fc(&DLCI2_PIPE, payload, DLCI_PPP).await;
                        }
                    }
                    _ => {
                        debug!("CMUX UIH for unknown DLCI {}", dlci);
                    }
                }
            }
            _ => {}
        }
    }
}

/// Writes `data` into `pipe`, asserting TS 27.010 `MSC(FC=1)` backpressure on `dlci` if the
/// pipe fills up instead of silently dropping bytes and corrupting the stream.
async fn write_pipe_with_fc<const BUF: usize>(
    pipe: &'static Pipe<CriticalSectionRawMutex, BUF>,
    mut data: &[u8],
    dlci: u8,
) {
    // Fast path: non-blocking write while space is available.
    while !data.is_empty() {
        match pipe.try_write(data) {
            Ok(n) if n > 0 => data = &data[n..],
            _ => break,
        }
    }
    if data.is_empty() {
        return;
    }

    // Slow path: pipe full — tell the modem to pause this DLCI, wait for consumer to drain, resume.
    debug!(
        "CMUX DLCI {} RX pipe full ({} B pending); asserting MSC FC=1",
        dlci,
        data.len()
    );
    let _ = send_msc_fc(dlci, true).await;

    let write_all = async {
        while !data.is_empty() {
            let n = pipe.write(data).await;
            data = &data[n..];
        }
    };
    if write_all
        .with_timeout(Duration::from_secs(2))
        .await
        .is_err()
    {
        warn!(
            "CMUX DLCI {} RX pipe stalled >2s; dropped {} bytes",
            dlci,
            data.len()
        );
    }

    let _ = send_msc_fc(dlci, false).await;
    debug!("CMUX DLCI {} RX pipe drained; cleared MSC FC=0", dlci);
}

async fn handle_ctrl_uih(payload: &[u8]) {
    if payload.len() < 2 {
        return;
    }
    let type_byte = payload[0];
    let is_cmd = (type_byte & CR) != 0;
    let msg_type = type_byte & !(CR | EA);
    let val_len = (payload[1] >> 1) as usize;
    let val = payload.get(2..2 + val_len).unwrap_or(&[]);

    if !is_cmd {
        if msg_type == MSG_NSC && !val.is_empty() {
            warn!("CMUX modem returned NSC for cmd 0x{:02X}", val[0]);
        }
        return;
    }

    match msg_type {
        MSG_MSC => {
            if val.len() >= 2 {
                let target_dlci = val[0] >> 2;
                let v24 = val[1];
                if let Some(st) = dlci_state(target_dlci) {
                    let new_fc = (v24 & V24_FC) != 0;
                    let old_fc = st.tx_paused.swap(new_fc, Ordering::Relaxed);
                    if old_fc != new_fc {
                        if new_fc {
                            warn!("CMUX DLCI {} TX paused by modem (MSC FC=1)", target_dlci);
                        } else {
                            info!("CMUX DLCI {} TX resumed by modem (MSC FC=0)", target_dlci);
                            st.tx_resume.signal(());
                        }
                    }

                    let new_dv = (v24 & V24_DV) != 0;
                    let old_dv = st.carrier_dv.swap(new_dv, Ordering::Relaxed);
                    if old_dv && !new_dv {
                        warn!("CMUX DLCI {} carrier dropped (MSC DV=0)", target_dlci);
                    }
                }
            }
            send_ctrl_ack(payload).await;
        }
        MSG_FCOFF => {
            warn!("CMUX aggregate TX paused by modem (FCoff)");
            GLOBAL_TX_PAUSED.store(true, Ordering::Relaxed);
            let _ = send_raw_frame(DLCI_CTRL, CTRL_UIH, &[MSG_FCOFF | EA, EA]).await;
        }
        MSG_FCON => {
            info!("CMUX aggregate TX resumed by modem (FCon)");
            GLOBAL_TX_PAUSED.store(false, Ordering::Relaxed);
            for st in &DLCI_STATES {
                st.tx_resume.signal(());
            }
            let _ = send_raw_frame(DLCI_CTRL, CTRL_UIH, &[MSG_FCON | EA, EA]).await;
        }
        MSG_CLD => {
            warn!("CMUX remote requested close-down (CLD)");
            CTRL_OPENED.store(false, Ordering::Relaxed);
            for st in &DLCI_STATES {
                st.reset();
            }
            let _ = send_raw_frame(DLCI_CTRL, CTRL_UIH, &[MSG_CLD | EA, EA]).await;
        }
        MSG_TEST => {
            send_ctrl_ack(payload).await;
        }
        _ => {
            warn!(
                "CMUX unsupported DLCI 0 command 0x{:02X}; replying NSC",
                type_byte
            );
            let nsc = [MSG_NSC | EA, (1 << 1) | EA, type_byte];
            let _ = send_raw_frame(DLCI_CTRL, CTRL_UIH, &nsc).await;
        }
    }
}

async fn send_ctrl_ack(payload: &[u8]) {
    let mut resp = [0u8; 32];
    let n = payload.len().min(resp.len());
    resp[..n].copy_from_slice(&payload[..n]);
    resp[0] &= !CR; // Command -> Response
    let _ = send_raw_frame(DLCI_CTRL, CTRL_UIH, &resp[..n]).await;
}

const PPP_FLAG: u8 = 0x7E;
const PPP_ESC: u8 = 0x7D;

/// Reassembles PPP frames on `DLCI 2`, de-interleaving complete small PPP packets that the
/// Quectel EC21-AU CMUX layer occasionally inserts between 127-byte chunks of a larger PPP
/// frame, and validating PPP FCS-16 so malformed frames never poison `ppproto::FrameReader`.
struct PppDeframer {
    long_buf: [u8; 2048],
    long_len: usize,
    deferred_buf: [u8; 512],
    deferred_len: usize,
}

impl PppDeframer {
    const fn new() -> Self {
        Self {
            long_buf: [0u8; 2048],
            long_len: 0,
            deferred_buf: [0u8; 512],
            deferred_len: 0,
        }
    }

    fn reset(&mut self) {
        self.long_len = 0;
        self.deferred_len = 0;
    }

    async fn push_chunk(
        &mut self,
        payload: &[u8],
        pipe: &'static Pipe<CriticalSectionRawMutex, PPP_PIPE_SIZE>,
    ) {
        // Detect a complete, self-contained PPP frame interleaved mid-packet by EC21 CMUX.
        if self.long_len > 1
            && payload.len() >= 6
            && payload[0] == PPP_FLAG
            && payload[payload.len() - 1] == PPP_FLAG
            && is_valid_ppp_frame(payload)
        {
            if self.deferred_len + payload.len() <= self.deferred_buf.len() {
                self.deferred_buf[self.deferred_len..self.deferred_len + payload.len()]
                    .copy_from_slice(payload);
                self.deferred_len += payload.len();
            } else {
                warn!(
                    "PPP deferred buffer full, dropped {}B interleaved frame",
                    payload.len()
                );
            }
            return;
        }

        for &b in payload {
            if b == PPP_FLAG {
                if self.long_len == 0 {
                    self.long_buf[0] = PPP_FLAG;
                    self.long_len = 1;
                } else if self.long_len == 1 {
                    // Consecutive 0x7E flags; keep single opening flag.
                } else {
                    if self.long_len < self.long_buf.len() {
                        self.long_buf[self.long_len] = PPP_FLAG;
                        self.long_len += 1;
                        if is_valid_ppp_frame(&self.long_buf[..self.long_len]) {
                            write_pipe_with_fc(pipe, &self.long_buf[..self.long_len], DLCI_PPP)
                                .await;
                        } else {
                            warn!("PPP bad FCS, dropped {}B frame", self.long_len);
                        }
                    } else {
                        warn!("PPP frame exceeded buffer, dropped");
                    }
                    // Keep 0x7E as potential opening flag of the next frame (shared-flag safe).
                    self.long_buf[0] = PPP_FLAG;
                    self.long_len = 1;

                    if self.deferred_len > 0 {
                        write_pipe_with_fc(pipe, &self.deferred_buf[..self.deferred_len], DLCI_PPP)
                            .await;
                        self.deferred_len = 0;
                    }
                }
            } else if self.long_len > 0 {
                if self.long_len < self.long_buf.len() {
                    self.long_buf[self.long_len] = b;
                    self.long_len += 1;
                } else {
                    warn!("PPP frame overflow (>2048B), resetting");
                    self.long_len = 0;
                }
            }
        }
    }
}

fn ppp_crc16_step(mut seed: u16, b: u8) -> u16 {
    let e = (seed as u8) ^ b;
    let f = (e ^ (e << 4)) as u16;
    seed = (seed >> 8) ^ (f << 8) ^ (f << 3) ^ (f >> 4);
    seed
}

fn is_valid_ppp_frame(raw: &[u8]) -> bool {
    if raw.len() < 6 || raw[0] != PPP_FLAG || raw[raw.len() - 1] != PPP_FLAG || raw[1] != 0xFF {
        return false;
    }
    let mut crc: u16 = 0x00FF; // crc16(0xFFFF, &[0xFF])
    let mut escape = false;
    let mut count = 0usize;
    let mut first_ctrl = 0u8;

    for &b in &raw[2..raw.len() - 1] {
        if b == PPP_FLAG {
            return false;
        }
        if b == PPP_ESC {
            if escape {
                return false;
            }
            escape = true;
            continue;
        }
        let val = if escape {
            escape = false;
            b ^ 0x20
        } else {
            b
        };
        if count == 0 {
            first_ctrl = val;
        }
        crc = ppp_crc16_step(crc, val);
        count += 1;
    }

    !escape && count >= 3 && first_ctrl == 0x03 && crc == 0xF0B8
}
