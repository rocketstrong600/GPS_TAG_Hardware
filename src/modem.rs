pub mod cmux;
pub mod commands;
pub mod gps;
pub mod ppp;

use atat::asynch::{AtatClient, Client};
use atat::heapless::String;
use atat::{AtatIngress, DefaultDigester, Ingress, ResponseSlot, UrcChannel};
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::{Duration, Timer, WithTimeout};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use cmux::{CmuxChannel, CmuxError, CmuxRx, CmuxTx};
use commands::Urc;

pub const INGRESS_BUF_SIZE: usize = 1024;
pub const URC_CAPACITY: usize = 128;
pub const URC_SUBSCRIBERS: usize = 3;
const AT_CLIENT_BUF_SIZE: usize = 1024;

pub type ModemClient = Client<'static, CmuxTx, AT_CLIENT_BUF_SIZE>;

pub struct ModemPins {
    pub ap_ready: Output<'static>,
    pub wake: Output<'static>,
    pub batt: Output<'static>,
    pub pwr_key: Output<'static>,
    pub reset: Output<'static>,
    pub uart_enable: Output<'static>,
    pub uart_dtr: Output<'static>,
}

impl ModemPins {
    /// Powers on the Quectel EC21-AU using the board's verified sequence.
    pub async fn power_on(&mut self) {
        self.uart_enable.set_high();
        self.uart_dtr.set_low();
        self.reset.set_low();
        self.ap_ready.set_low();
        self.wake.set_low();
        self.batt.set_low();
        self.pwr_key.set_low();

        Timer::after_millis(400).await;

        self.ap_ready.set_high();
        self.wake.set_high();
        self.batt.set_high();
        Timer::after_millis(250).await;

        self.pwr_key.set_high();
        Timer::after_millis(750).await;
        self.pwr_key.set_low();
    }
}

#[derive(Debug, Copy, Clone, defmt::Format)]
pub enum ModemError {
    BootTimeout,
    CmuxSetupFailed,
    Cmux(CmuxError),
    PppDialFailed,
}

impl From<CmuxError> for ModemError {
    fn from(e: CmuxError) -> Self {
        Self::Cmux(e)
    }
}

/// Powers on the EC21-AU, waits for `RDY`, enters `AT+CMUX=0` (3GPP TS 27.010),
/// opens `DLCI 1` (AT/GPS) and `DLCI 2` (PPP), and returns `(at_client, ppp_channel)`.
pub async fn init(
    spawner: &Spawner,
    pins: &mut ModemPins,
    mut tx: UartTx<'static, Async>,
    mut rx: RingBufferedUartRx<'static>,
) -> Result<(ModemClient, CmuxChannel), ModemError> {
    pins.power_on().await;

    wait_for_rdy(&mut tx, &mut rx).await?;
    info!("EC21-AU RDY!");

    send_at_ok(&mut tx, &mut rx, b"ATE0\r\n").await?;
    query_sim_info(&mut tx, &mut rx).await;
    send_at_ok(&mut tx, &mut rx, b"AT+CMUX=0\r\n").await?;
    Timer::after_millis(50).await;

    let (at_ch, ppp_ch) = cmux::start(spawner, tx, rx).await?;
    let (at_rx, at_tx) = at_ch.split();

    static RES_SLOT: ResponseSlot<INGRESS_BUF_SIZE> = ResponseSlot::new();
    static URC_CHANNEL: UrcChannel<Urc, URC_CAPACITY, URC_SUBSCRIBERS> = UrcChannel::new();
    static INGRESS_BUF: StaticCell<[u8; INGRESS_BUF_SIZE]> = StaticCell::new();
    static CLIENT_BUF: StaticCell<[u8; AT_CLIENT_BUF_SIZE]> = StaticCell::new();

    let ingress = Ingress::new(
        DefaultDigester::<Urc>::default(),
        INGRESS_BUF.init([0; INGRESS_BUF_SIZE]),
        &RES_SLOT,
        &URC_CHANNEL,
    );

    spawner.spawn(at_reader_task(at_rx, ingress).unwrap());

    let mut client = Client::new(
        at_tx,
        &RES_SLOT,
        CLIENT_BUF.init([0; AT_CLIENT_BUF_SIZE]),
        atat::Config::default(),
    );

    if let Err(e) = set_lte_only(&mut client).await {
        warn!("Failed to set LTE-only mode: {:?}", e);
    }

    Ok((client, ppp_ch))
}

/// Locks the EC21-AU to LTE-only (`AT+QCFG="nwscanmode",3,1`) for Australian networks
/// where 3G has been decommissioned.
pub async fn set_lte_only<C: AtatClient>(client: &mut C) -> Result<(), atat::Error> {
    let mut param = String::<12>::new();
    let _ = param.push_str("nwscanmode");
    client
        .send(&commands::SetNwScanMode {
            param,
            mode: 3,   // LTE only
            effect: 1, // Apply immediately
        })
        .await?;
    info!("EC21-AU locked to LTE-only");
    Ok(())
}

/// Dials PPP (`ATD*99#`) directly on the PPP DLCI channel (`DLCI 2`), leaving
/// `DLCI 1` free for concurrent AT/GPS commands.
#[allow(dead_code)]
pub async fn dial_ppp(ppp_ch: &mut CmuxChannel) -> Result<(), ModemError> {
    ppp_ch.write(b"ATD*99#\r\n").await?;
    if wait_for_token(ppp_ch, b"CONNECT", Duration::from_secs(10)).await {
        info!("EC21-AU PPP CONNECT on DLCI 2");
        Ok(())
    } else {
        error!("EC21-AU PPP dial timeout/error");
        Err(ModemError::PppDialFailed)
    }
}

#[embassy_executor::task]
async fn at_reader_task(
    mut rx: CmuxRx,
    mut ingress: Ingress<
        'static,
        DefaultDigester<Urc>,
        Urc,
        INGRESS_BUF_SIZE,
        URC_CAPACITY,
        URC_SUBSCRIBERS,
    >,
) {
    ingress.read_from(&mut rx).await;
}

async fn query_sim_info(tx: &mut UartTx<'static, Async>, rx: &mut RingBufferedUartRx<'static>) {
    info!("=== EC21-AU SIM & Network Info ===");
    for (label, cmd) in [
        ("IMEI", b"AT+GSN\r\n".as_slice()),
        ("CPIN", b"AT+CPIN?\r\n".as_slice()),
        ("QCCID", b"AT+QCCID\r\n".as_slice()),
        ("CIMI", b"AT+CIMI\r\n".as_slice()),
        ("CSQ", b"AT+CSQ\r\n".as_slice()),
        ("CEREG", b"AT+CEREG?\r\n".as_slice()),
        ("COPS", b"AT+COPS?\r\n".as_slice()),
    ] {
        let mut buf = [0u8; 128];
        let _ = tx.write(cmd).await;
        let n = read_until_ok_or_error(rx, &mut buf, Duration::from_millis(1500)).await;
        let text = core::str::from_utf8(&buf[..n]).unwrap_or("<non-utf8>").trim();
        info!("  [{}] {}", label, text);
    }
}

async fn read_until_ok_or_error(
    rx: &mut RingBufferedUartRx<'static>,
    out: &mut [u8],
    timeout: Duration,
) -> usize {
    let mut len = 0usize;
    let mut byte = [0u8; 1];

    let _ = async {
        loop {
            match rx.read(&mut byte).await {
                Ok(1) => {
                    if len < out.len() {
                        out[len] = byte[0];
                        len += 1;
                    }
                    let slice = &out[..len];
                    if slice.ends_with(b"\r\nOK\r\n") || slice.ends_with(b"ERROR\r\n") {
                        break;
                    }
                }
                _ => Timer::after_millis(5).await,
            }
        }
    }
    .with_timeout(timeout)
    .await;

    len
}

async fn wait_for_rdy(
    tx: &mut UartTx<'static, Async>,
    rx: &mut RingBufferedUartRx<'static>,
) -> Result<(), ModemError> {
    if wait_for_token(rx, b"RDY", Duration::from_secs(12)).await {
        return Ok(());
    }
    warn!("EC21 RDY URC not seen, probing AT...");
    for _ in 0..5 {
        if send_at_ok(tx, rx, b"AT\r\n").await.is_ok() {
            return Ok(());
        }
        Timer::after_millis(500).await;
    }
    Err(ModemError::BootTimeout)
}

async fn send_at_ok(
    tx: &mut UartTx<'static, Async>,
    rx: &mut RingBufferedUartRx<'static>,
    cmd: &[u8],
) -> Result<(), ModemError> {
    for _ in 0..3 {
        let _ = tx.write(cmd).await;
        if wait_for_token(rx, b"OK", Duration::from_secs(2)).await {
            return Ok(());
        }
        Timer::after_millis(200).await;
    }
    Err(ModemError::CmuxSetupFailed)
}

async fn wait_for_token<R: Read>(rx: &mut R, token: &[u8], timeout: Duration) -> bool {
    let mut window = [0u8; 32];
    let mut len = 0usize;
    let mut byte = [0u8; 1];

    let fut = async {
        loop {
            match rx.read(&mut byte).await {
                Ok(1) => {
                    if len < window.len() {
                        window[len] = byte[0];
                        len += 1;
                    } else {
                        window.copy_within(1.., 0);
                        window[window.len() - 1] = byte[0];
                    }
                    if len >= token.len()
                        && window[..len].windows(token.len()).any(|w| w == token)
                    {
                        return true;
                    }
                }
                _ => Timer::after_millis(10).await,
            }
        }
    };

    fut.with_timeout(timeout).await.unwrap_or(false)
}
