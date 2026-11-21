#![allow(dead_code)]
use atat::AtatCmd;
use atat::asynch::AtatClient;
use defmt::info;
use embassy_time::Timer;

use super::cmux::{self, CmuxError};
use super::commands::{DeleteSms, NoResponse, SetSmsIndication, SetSmsTextMode};

#[derive(Debug, Clone, defmt::Format)]
pub enum SmsError {
    At(atat::Error),
    Cmux(CmuxError),
    InvalidInput,
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

/// Deletes an SMS at `index` from SIM/modem storage (`AT+CMGD=<index>`).
pub async fn delete<C: AtatClient>(client: &mut C, index: u16) -> Result<(), SmsError> {
    client.send(&DeleteSms { index }).await?;
    Ok(())
}
