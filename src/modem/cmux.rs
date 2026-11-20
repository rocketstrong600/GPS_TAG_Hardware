use defmt::{debug, error, warn};
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
const CTRL_SABM: u8 = 0x3F; // SABM with P=1
const CTRL_UA: u8 = 0x73; // UA with F=1
const CTRL_DM: u8 = 0x1F; // DM with F=1
const CTRL_DISC: u8 = 0x53; // DISC with P=1
const CTRL_UIH: u8 = 0xEF; // UIH with P/F=0

pub const DLCI_CTRL: u8 = 0;
pub const DLCI_AT: u8 = 1;
pub const DLCI_PPP: u8 = 2;

const PIPE_BUF_SIZE: usize = 1024;
const MAX_FRAME_INFO: usize = 512;

static UART_TX: Mutex<CriticalSectionRawMutex, Option<UartTx<'static, Async>>> = Mutex::new(None);
static DLCI1_PIPE: Pipe<CriticalSectionRawMutex, PIPE_BUF_SIZE> = Pipe::new();
static DLCI2_PIPE: Pipe<CriticalSectionRawMutex, PIPE_BUF_SIZE> = Pipe::new();
static CTRL_ACK: Signal<CriticalSectionRawMutex, (u8, u8)> = Signal::new();

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

fn calc_fcs(header: &[u8]) -> u8 {
    let mut fcs = 0xFFu8;
    for &b in header {
        fcs = FCS_TABLE[(fcs ^ b) as usize];
    }
    0xFF - fcs
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

async fn send_raw_frame(dlci: u8, ctrl: u8, info: &[u8]) -> Result<(), CmuxError> {
    if info.len() > MAX_INFO_LEN {
        return Err(CmuxError);
    }

    let mut frame = [0u8; 6 + MAX_INFO_LEN];
    frame[0] = FLAG;
    frame[1] = (dlci << 2) | 0x03; // CR=1 (TE command), EA=1
    frame[2] = ctrl;
    frame[3] = ((info.len() as u8) << 1) | 0x01; // EA=1

    let fcs = calc_fcs(&frame[1..4]);
    let mut idx = 4;
    if !info.is_empty() {
        frame[idx..idx + info.len()].copy_from_slice(info);
        idx += info.len();
    }
    frame[idx] = fcs;
    frame[idx + 1] = FLAG;
    let total = idx + 2;

    let mut guard = UART_TX.lock().await;
    let tx = guard.as_mut().ok_or(CmuxError)?;
    tx.write(&frame[..total]).await.map_err(|_| CmuxError)?;
    Ok(())
}

/// Starts the TS 27.010 multiplexer task and returns `(at_channel, ppp_channel)`
/// after opening `DLCI 0` (control), `DLCI 1` (AT/GPS), and `DLCI 2` (PPP).
pub async fn start(
    spawner: &Spawner,
    tx: UartTx<'static, Async>,
    rx: RingBufferedUartRx<'static>,
) -> Result<(CmuxChannel, CmuxChannel), CmuxError> {
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

async fn open_dlci(dlci: u8) -> Result<(), CmuxError> {
    for _ in 0..3 {
        CTRL_ACK.reset();
        send_raw_frame(dlci, CTRL_SABM, &[]).await?;
        if let Ok((ack_dlci, ctrl)) = CTRL_ACK
            .wait()
            .with_timeout(Duration::from_millis(1000))
            .await
        {
            if ack_dlci == dlci && (ctrl & !0x10) == (CTRL_UA & !0x10) {
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
    send_raw_frame(dlci, CTRL_DISC, &[]).await
}

/// Virtual serial channel over a TS 27.010 DLCI.
/// Implements `embedded_io_async::{Read, BufRead, Write}` for `atat` and `embassy-net-ppp`.
pub struct CmuxChannel {
    dlci: u8,
    pipe: &'static Pipe<CriticalSectionRawMutex, PIPE_BUF_SIZE>,
    buf: [u8; 128],
    pos: usize,
    len: usize,
}

impl CmuxChannel {
    fn new(dlci: u8, pipe: &'static Pipe<CriticalSectionRawMutex, PIPE_BUF_SIZE>) -> Self {
        Self {
            dlci,
            pipe,
            buf: [0u8; 128],
            pos: 0,
            len: 0,
        }
    }

    pub fn split(self) -> (CmuxRx, CmuxTx) {
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

pub struct CmuxRx {
    pipe: &'static Pipe<CriticalSectionRawMutex, PIPE_BUF_SIZE>,
    buf: [u8; 128],
    pos: usize,
    len: usize,
}

impl ErrorType for CmuxRx {
    type Error = CmuxError;
}

impl Read for CmuxRx {
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
        let n = buf.len().min(MAX_INFO_LEN);
        send_raw_frame(self.dlci, CTRL_UIH, &buf[..n]).await?;
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl ErrorType for CmuxChannel {
    type Error = CmuxError;
}

impl Read for CmuxChannel {
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

impl BufRead for CmuxChannel {
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

impl Write for CmuxChannel {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let n = buf.len().min(MAX_INFO_LEN);
        send_raw_frame(self.dlci, CTRL_UIH, &buf[..n]).await?;
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[embassy_executor::task]
async fn cmux_rx_task(mut rx: RingBufferedUartRx<'static>) {
    let mut byte = [0u8; 1];

    loop {
        // 1. Hunt for opening 0xF9 flag
        loop {
            if rx.read(&mut byte).await.is_err() {
                continue;
            }
            if byte[0] == FLAG {
                break;
            }
        }

        // 2. Skip consecutive 0xF9 flags; first non-flag byte is Address
        let addr = loop {
            if rx.read(&mut byte).await.is_err() {
                continue;
            }
            if byte[0] != FLAG {
                break byte[0];
            }
        };

        // 3. Control byte
        if rx.read(&mut byte).await.is_err() {
            continue;
        }
        let ctrl = byte[0];

        // 4. Length byte(s)
        if rx.read(&mut byte).await.is_err() {
            continue;
        }
        let len1 = byte[0];
        let mut header = [addr, ctrl, len1, 0u8];
        let (header_len, info_len) = if (len1 & 0x01) != 0 {
            (3usize, (len1 >> 1) as usize)
        } else {
            if rx.read(&mut byte).await.is_err() {
                continue;
            }
            let len2 = byte[0];
            header[3] = len2;
            (4usize, ((len1 >> 1) as usize) | ((len2 as usize) << 7))
        };

        if info_len > MAX_FRAME_INFO {
            warn!("CMUX frame too large: {} bytes", info_len);
            continue;
        }

        // 5. Information field
        let mut info = [0u8; MAX_FRAME_INFO];
        let mut read_ok = true;
        let mut got = 0usize;
        while got < info_len {
            match rx.read(&mut info[got..info_len]).await {
                Ok(0) | Err(_) => {
                    read_ok = false;
                    break;
                }
                Ok(n) => got += n,
            }
        }
        if !read_ok {
            continue;
        }

        // 6. FCS + closing 0xF9 flag
        if rx.read(&mut byte).await.is_err() {
            continue;
        }
        let rx_fcs = byte[0];
        if rx.read(&mut byte).await.is_err() || byte[0] != FLAG {
            continue;
        }

        let expected_fcs = calc_fcs(&header[..header_len]);
        if rx_fcs != expected_fcs {
            warn!(
                "CMUX FCS mismatch: got 0x{:02X}, expected 0x{:02X}",
                rx_fcs, expected_fcs
            );
            continue;
        }

        let dlci = addr >> 2;
        let base_ctrl = ctrl & !0x10;

        match base_ctrl {
            c if c == (CTRL_UA & !0x10) || c == (CTRL_DM & !0x10) => {
                CTRL_ACK.signal((dlci, ctrl));
            }
            c if c == (CTRL_UIH & !0x10) => {
                let payload = &info[..info_len];
                match dlci {
                    DLCI_CTRL => handle_ctrl_uih(payload).await,
                    DLCI_AT => {
                        let _ = DLCI1_PIPE.write(payload).await;
                    }
                    DLCI_PPP => {
                        let _ = DLCI2_PIPE.write(payload).await;
                    }
                    _ => {
                        debug!("CMUX UIH for unknown DLCI {}", dlci);
                    }
                }
            }
            _ => {
                debug!("CMUX unhandled ctrl=0x{:02X} dlci={}", ctrl, dlci);
            }
        }
    }
}

async fn handle_ctrl_uih(payload: &[u8]) {
    // Reply to MSC (Modem Status Command, type 0xE3) with MSC response (0xE1)
    if payload.len() >= 2 && payload[0] == 0xE3 {
        let mut resp = [0u8; 8];
        let n = payload.len().min(resp.len());
        resp[..n].copy_from_slice(&payload[..n]);
        resp[0] = 0xE1;
        let _ = send_raw_frame(DLCI_CTRL, CTRL_UIH, &resp[..n]).await;
    }
}
