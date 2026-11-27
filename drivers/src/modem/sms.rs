//! SMS text-mode send, read, URC receive, and delete helpers over CMUX `DLCI 1`
//! (`AT+CMGF=1`, `AT+CNMI`, `AT+CMGS`, `AT+CMGR`, `AT+CMGL`, `AT+CMGD`).

#![allow(dead_code)]

use atat::AtatCmd;
use atat::asynch::AtatClient;
use atat::heapless::String;
use defmt::info;
use embassy_time::Timer;

use super::cmux::{self, CmuxError};
use super::commands::{
    DeleteSms, DeleteSmsWithFlag, NoResponse, SetSmsIndication, SetSmsTextMode, Urc,
};
use super::{URC_CAPACITY, URC_SUBSCRIBERS};

#[derive(Debug, Clone, defmt::Format)]
pub enum SmsError {
    At(atat::Error),
    Cmux(CmuxError),
    InvalidInput,
    /// `URC_CHANNEL` has no free subscriber slots (max [`super::URC_SUBSCRIBERS`]).
    TooManySubscribers,
}

impl From<atat::Error> for SmsError {
    fn from(e: atat::Error) -> Self {
        Self::At(e)
    }
}

impl From<CmuxError> for SmsError {
    fn from(e: CmuxError) -> Self {
        Self::Cmux(e)
    }
}

/// Decoded text-mode SMS message returned by [`read`], [`read_first_unread`], or
/// [`SmsReceiver::next_message`].
#[derive(Debug, Clone)]
pub struct SmsMessage {
    /// Storage slot index (pass to [`delete`] once processed).
    pub index: u16,
    /// Message status (e.g. `"REC UNREAD"`, `"REC READ"`).
    pub status: String<16>,
    /// Sender phone number or alphanumeric originator (e.g. `"+61412345678"`).
    pub sender: String<32>,
    /// Service-center timestamp (e.g. `"26/11/24,23:10:05+44"`).
    pub timestamp: String<32>,
    /// Message text body (up to 320 UTF-8 bytes).
    pub body: String<320>,
}

/// Persistent subscriber for incoming `+CMTI` SMS notifications on `DLCI 1`.
///
/// Create once via [`subscribe`] after [`init`] so no incoming SMS notifications are missed
/// between reads.
pub struct SmsReceiver {
    sub: atat::UrcSubscription<'static, Urc, URC_CAPACITY, URC_SUBSCRIBERS>,
}

impl SmsReceiver {
    /// Waits asynchronously for the next `+CMTI` URC and returns its storage slot index.
    pub async fn next_index(&mut self) -> u16 {
        loop {
            if let Urc::NewSms(sms) = self.sub.next_message_pure().await {
                return sms.index;
            }
        }
    }

    /// Waits asynchronously for the next incoming SMS (`+CMTI`) and reads it via [`read`].
    pub async fn next_message<C: AtatClient>(
        &mut self,
        client: &mut C,
    ) -> Result<SmsMessage, SmsError> {
        loop {
            let idx = self.next_index().await;
            if let Some(msg) = read(client, idx).await? {
                return Ok(msg);
            }
        }
    }
}

/// Subscribes to incoming `+CMTI` SMS notifications from [`super::subscribe_urc`].
pub fn subscribe() -> Result<SmsReceiver, SmsError> {
    let sub = super::subscribe_urc().ok_or(SmsError::TooManySubscribers)?;
    Ok(SmsReceiver { sub })
}

struct RawSmsBody<'a> {
    body: &'a [u8],
}

impl<'a> AtatCmd for RawSmsBody<'a> {
    type Response = NoResponse;
    const MAX_TIMEOUT_MS: u32 = 10_000;
    const EXPECTS_RESPONSE_CODE: bool = true;

    fn write(&self, buf: &mut [u8]) -> usize {
        let n = self.body.len().min(160).min(buf.len().saturating_sub(1));
        buf[..n].copy_from_slice(&self.body[..n]);
        buf[n] = 0x1A; // Ctrl-Z
        n + 1
    }

    fn parse(
        &self,
        resp: Result<&[u8], atat::InternalError>,
    ) -> Result<Self::Response, atat::Error> {
        resp?;
        Ok(NoResponse)
    }
}

struct OptionalSmsResp(Option<SmsMessage>);
impl atat::AtatResp for OptionalSmsResp {}

struct ReadSmsCmd {
    index: u16,
}

impl AtatCmd for ReadSmsCmd {
    type Response = OptionalSmsResp;
    const MAX_TIMEOUT_MS: u32 = 2000;
    const EXPECTS_RESPONSE_CODE: bool = true;

    fn write(&self, buf: &mut [u8]) -> usize {
        let mut cmd = String::<24>::new();
        let _ = core::fmt::write(&mut cmd, format_args!("AT+CMGR={}\r", self.index));
        let bytes = cmd.as_bytes();
        buf[..bytes.len()].copy_from_slice(bytes);
        bytes.len()
    }

    fn parse(
        &self,
        resp: Result<&[u8], atat::InternalError>,
    ) -> Result<Self::Response, atat::Error> {
        let raw = resp?;
        Ok(OptionalSmsResp(parse_cmgr(self.index, raw)))
    }
}

struct ReadFirstUnreadCmd;

impl AtatCmd for ReadFirstUnreadCmd {
    type Response = OptionalSmsResp;
    const MAX_TIMEOUT_MS: u32 = 3000;
    const EXPECTS_RESPONSE_CODE: bool = true;

    fn write(&self, buf: &mut [u8]) -> usize {
        let cmd = b"AT+CMGL=\"REC UNREAD\"\r";
        buf[..cmd.len()].copy_from_slice(cmd);
        cmd.len()
    }

    fn parse(
        &self,
        resp: Result<&[u8], atat::InternalError>,
    ) -> Result<Self::Response, atat::Error> {
        let raw = resp?;
        Ok(OptionalSmsResp(parse_cmgl_first(raw)))
    }
}

/// Enables SMS text mode (`AT+CMGF=1`) and `+CMTI` new-message URCs (`AT+CNMI=2,1,0,0,0`).
pub async fn init<C: AtatClient>(client: &mut C) -> Result<(), SmsError> {
    client.send(&SetSmsTextMode { mode: 1 }).await?;
    client
        .send(&SetSmsIndication {
            mode: 2,
            mt: 1,
            bm: 0,
            ds: 0,
            bfr: 0,
        })
        .await?;
    info!("EC21-AU SMS initialized");
    Ok(())
}

/// Sends a text SMS (up to 160 ASCII bytes) over CMUX `DLCI 1`.
pub async fn send<C: AtatClient>(client: &mut C, number: &str, body: &str) -> Result<(), SmsError> {
    if number.is_empty() || number.len() > 24 || body.len() > 160 {
        return Err(SmsError::InvalidInput);
    }

    let mut cmd = [0u8; 40];
    let prefix = b"AT+CMGS=\"";
    let suffix = b"\"\r";
    let num = number.as_bytes();
    let total = prefix.len() + num.len() + suffix.len();
    cmd[..prefix.len()].copy_from_slice(prefix);
    cmd[prefix.len()..prefix.len() + num.len()].copy_from_slice(num);
    cmd[prefix.len() + num.len()..total].copy_from_slice(suffix);

    cmux::write_at_raw(&cmd[..total]).await?;
    Timer::after_millis(100).await;

    client
        .send(&RawSmsBody {
            body: body.as_bytes(),
        })
        .await?;
    info!("SMS sent to {}", number);
    Ok(())
}

/// Reads a stored text-mode SMS at `index` (`AT+CMGR=<index>`).
///
/// Returns `Ok(None)` if slot `index` is empty.
pub async fn read<C: AtatClient>(
    client: &mut C,
    index: u16,
) -> Result<Option<SmsMessage>, SmsError> {
    Ok(client.send(&ReadSmsCmd { index }).await?.0)
}

/// Reads the first unread text-mode SMS in storage (`AT+CMGL="REC UNREAD"`).
///
/// Useful at startup to pick up messages that arrived while booting before [`subscribe`] was
/// active. Reading marks the message as `"REC READ"` on the modem; call [`delete`] afterwards to
/// free the storage slot.
pub async fn read_first_unread<C: AtatClient>(
    client: &mut C,
) -> Result<Option<SmsMessage>, SmsError> {
    Ok(client.send(&ReadFirstUnreadCmd).await?.0)
}

/// Deletes an SMS at `index` from SIM/modem storage (`AT+CMGD=<index>`).
pub async fn delete<C: AtatClient>(client: &mut C, index: u16) -> Result<(), SmsError> {
    client.send(&DeleteSms { index }).await?;
    Ok(())
}

/// Deletes all SMS messages from preferred storage (`AT+CMGD=1,4`).
pub async fn delete_all<C: AtatClient>(client: &mut C) -> Result<(), SmsError> {
    client
        .send(&DeleteSmsWithFlag {
            index: 1,
            delflag: 4,
        })
        .await?;
    Ok(())
}

fn parse_cmgr(index: u16, raw: &[u8]) -> Option<SmsMessage> {
    let start = find_subslice(raw, b"+CMGR:")?;
    let after_tag = &raw[start + b"+CMGR:".len()..];
    let (hdr_bytes, body_bytes) = split_header_and_body(after_tag);
    let hdr = core::str::from_utf8(hdr_bytes).ok()?;

    let status = push_truncated::<16>(csv_field(hdr, 0).unwrap_or(""));
    let sender = push_truncated::<32>(csv_field(hdr, 1).unwrap_or(""));
    let timestamp = push_truncated::<32>(csv_field(hdr, 3).unwrap_or(""));
    let body = bytes_to_string_lossy::<320>(trim_crlf_end(body_bytes));

    Some(SmsMessage {
        index,
        status,
        sender,
        timestamp,
        body,
    })
}

fn parse_cmgl_first(raw: &[u8]) -> Option<SmsMessage> {
    let start = find_subslice(raw, b"+CMGL:")?;
    let after_tag = &raw[start + b"+CMGL:".len()..];
    let (hdr_bytes, rest) = split_header_and_body(after_tag);
    let hdr = core::str::from_utf8(hdr_bytes).ok()?;

    let index = csv_field(hdr, 0)?.trim().parse::<u16>().ok()?;
    let status = push_truncated::<16>(csv_field(hdr, 1).unwrap_or(""));
    let sender = push_truncated::<32>(csv_field(hdr, 2).unwrap_or(""));
    let timestamp = push_truncated::<32>(csv_field(hdr, 4).unwrap_or(""));

    let body_bytes = match find_subslice(rest, b"\n+CMGL:") {
        Some(next_entry) => &rest[..next_entry],
        None => rest,
    };
    let body = bytes_to_string_lossy::<320>(trim_crlf_end(body_bytes));

    Some(SmsMessage {
        index,
        status,
        sender,
        timestamp,
        body,
    })
}

fn split_header_and_body(slice: &[u8]) -> (&[u8], &[u8]) {
    match slice.iter().position(|&b| b == b'\n') {
        Some(nl) => {
            let hdr = slice[..nl].strip_suffix(b"\r").unwrap_or(&slice[..nl]);
            (hdr, &slice[nl + 1..])
        }
        None => (slice.strip_suffix(b"\r").unwrap_or(slice), &[]),
    }
}

fn trim_crlf_end(mut bytes: &[u8]) -> &[u8] {
    while let Some(&last) = bytes.last() {
        if last == b'\r' || last == b'\n' {
            bytes = &bytes[..bytes.len() - 1];
        } else {
            break;
        }
    }
    bytes
}

fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// Extracts the `target_idx`-th comma-separated field from an AT response header, respecting
/// double-quoted fields (such as `"26/11/24,23:10:05+44"`) and stripping surrounding quotes.
fn csv_field(line: &str, target_idx: usize) -> Option<&str> {
    let bytes = line.as_bytes();
    let mut field_idx = 0usize;
    let mut start = 0usize;
    let mut in_quotes = false;

    for (i, &b) in bytes.iter().enumerate() {
        if b == b'"' {
            in_quotes = !in_quotes;
        } else if b == b',' && !in_quotes {
            if field_idx == target_idx {
                return Some(strip_quotes(line[start..i].trim()));
            }
            field_idx += 1;
            start = i + 1;
        }
    }

    if field_idx == target_idx && start <= line.len() {
        Some(strip_quotes(line[start..].trim()))
    } else {
        None
    }
}

fn strip_quotes(s: &str) -> &str {
    s.strip_prefix('"')
        .and_then(|rest| rest.strip_suffix('"'))
        .unwrap_or(s)
}

fn push_truncated<const N: usize>(s: &str) -> String<N> {
    let mut out = String::<N>::new();
    for ch in s.chars() {
        if out.push(ch).is_err() {
            break;
        }
    }
    out
}

fn bytes_to_string_lossy<const N: usize>(mut bytes: &[u8]) -> String<N> {
    let mut out = String::<N>::new();
    while !bytes.is_empty() {
        match core::str::from_utf8(bytes) {
            Ok(valid) => {
                for ch in valid.chars() {
                    if out.push(ch).is_err() {
                        return out;
                    }
                }
                break;
            }
            Err(e) => {
                let valid = core::str::from_utf8(&bytes[..e.valid_up_to()]).unwrap_or("");
                for ch in valid.chars() {
                    if out.push(ch).is_err() {
                        return out;
                    }
                }
                if out.push('?').is_err() {
                    return out;
                }
                let advance = e.valid_up_to() + e.error_len().unwrap_or(1);
                bytes = &bytes[advance.min(bytes.len())..];
            }
        }
    }
    out
}
