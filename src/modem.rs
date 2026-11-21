pub mod cmux;
pub mod commands;
pub mod gps;
pub mod ppp;
pub mod sms;

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
use commands::{
    AdcReading, BatteryCharge, GetBatteryCharge, GetIccid, GetImei, GetSignalQuality, GetSimStatus,
    ReadAdc, SetCmeErrorMode, SetSleepMode, SignalQuality, Urc,
};

pub const INGRESS_BUF_SIZE: usize = 1024;
pub const URC_CAPACITY: usize = 128;
pub const URC_SUBSCRIBERS: usize = 3;
const AT_CLIENT_BUF_SIZE: usize = 1024;

pub type ModemClient = Client<'static, CmuxTx, AT_CLIENT_BUF_SIZE>;

static URC_CHANNEL: UrcChannel<Urc, URC_CAPACITY, URC_SUBSCRIBERS> = UrcChannel::new();

#[allow(dead_code)]
pub fn subscribe_urc() -> Option<atat::UrcSubscription<'static, Urc, URC_CAPACITY, URC_SUBSCRIBERS>>
{
    URC_CHANNEL.subscribe().ok()
}

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
    /// Powers on the Quectel EC21-AU using the board's verified sequence (cycles VBAT first).
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

    /// Cuts VBAT (`PB12`) to power down the EC21-AU completely.
    #[allow(dead_code)]
    pub fn power_off(&mut self) {
        self.pwr_key.set_low();
        self.reset.set_low();
        self.ap_ready.set_low();
        self.wake.set_low();
        self.batt.set_low();
    }

    /// Pulses the modem hardware reset line (`PA7`) for 300 ms.
    #[allow(dead_code)]
    pub async fn hard_reset(&mut self) {
        self.reset.set_high();
        Timer::after_millis(300).await;
        self.reset.set_low();
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
    send_at_ok(&mut tx, &mut rx, b"AT+CMEE=1\r\n").await?;

    // On cold boot, this EC21-AU firmware emits an early RDY where PMIC/ADC commands
    // (AT+CBC / AT+QADC) return ERROR and then performs an internal warm restart ~10-12s
    // later. Wait for that second RDY before entering CMUX so CMUX never loses sync.
    if !probe_pmic_ready(&mut tx, &mut rx).await {
        info!("EC21-AU stage-1 RDY (PMIC not ready); waiting for final RDY...");
        if wait_for_token(&mut rx, b"RDY\r\n", Duration::from_secs(18)).await {
            info!("EC21-AU final RDY!");
            send_at_ok(&mut tx, &mut rx, b"ATE0\r\n").await?;
            send_at_ok(&mut tx, &mut rx, b"AT+CMEE=1\r\n").await?;
        } else {
            warn!("EC21-AU second RDY not seen within 18s; continuing");
        }
    }

    let _ = send_at_ok(&mut tx, &mut rx, b"AT+QSCLK=0\r\n").await;
    send_at_ok(&mut tx, &mut rx, b"AT+CMUX=0\r\n").await?;
    Timer::after_millis(50).await;

    let (at_ch, ppp_ch) = cmux::start(spawner, tx, rx).await?;
    let (at_rx, at_tx) = at_ch.split();

    static RES_SLOT: ResponseSlot<INGRESS_BUF_SIZE> = ResponseSlot::new();
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

    let _ = client.send(&SetCmeErrorMode { mode: 1 }).await;
    if let Err(e) = set_lte_only(&mut client).await {
        warn!("Failed to set LTE-only mode: {:?}", e);
    }
    log_status(&mut client).await;

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

pub async fn get_imei<C: AtatClient>(client: &mut C) -> Result<String<20>, atat::Error> {
    Ok(client.send(&GetImei).await?.imei)
}

pub async fn get_sim_status<C: AtatClient>(client: &mut C) -> Result<String<16>, atat::Error> {
    Ok(client.send(&GetSimStatus).await?.code)
}

pub async fn get_iccid<C: AtatClient>(client: &mut C) -> Result<String<24>, atat::Error> {
    Ok(client.send(&GetIccid).await?.ccid)
}

pub async fn get_signal_quality<C: AtatClient>(
    client: &mut C,
) -> Result<SignalQuality, atat::Error> {
    client.send(&GetSignalQuality).await
}

pub async fn get_battery<C: AtatClient>(client: &mut C) -> Result<BatteryCharge, atat::Error> {
    client.send(&GetBatteryCharge).await
}

#[allow(dead_code)]
pub async fn read_adc<C: AtatClient>(client: &mut C, port: u8) -> Result<AdcReading, atat::Error> {
    client.send(&ReadAdc { port }).await
}

#[allow(dead_code)]
pub async fn set_sleep_mode<C: AtatClient>(
    client: &mut C,
    enable: bool,
) -> Result<(), atat::Error> {
    client
        .send(&SetSleepMode {
            mode: u8::from(enable),
        })
        .await?;
    Ok(())
}

/// Logs IMEI, SIM, ICCID, signal quality, and modem battery voltage over `DLCI 1`.
pub async fn log_status<C: AtatClient>(client: &mut C) {
    if let Ok(imei) = get_imei(client).await {
        info!("EC21 IMEI: {}", imei.as_str());
    }
    if let Ok(sim) = get_sim_status(client).await {
        info!("EC21 SIM: {}", sim.as_str());
    }
    if let Ok(iccid) = get_iccid(client).await {
        info!("EC21 ICCID: {}", iccid.as_str());
    }
    if let Ok(csq) = get_signal_quality(client).await {
        match csq.rssi_dbm() {
            Some(dbm) => info!("EC21 CSQ: rssi={} ({} dBm), ber={}", csq.rssi, dbm, csq.ber),
            None => info!("EC21 CSQ: unknown (rssi={})", csq.rssi),
        }
    }
    if let Ok(batt) = get_battery(client).await {
        info!(
            "EC21 VBAT: {} mV (bcs={}, bcl={}%)",
            batt.voltage_mv, batt.bcs, batt.bcl
        );
    }
}

/// Dials PPP (`ATD*99#`) directly on the PPP DLCI channel (`DLCI 2`), leaving
/// `DLCI 1` free for concurrent AT/GPS commands.
pub async fn dial_ppp(ppp_ch: &mut CmuxChannel) -> Result<(), ModemError> {
    drain_rx(ppp_ch, Duration::from_millis(25)).await;
    ppp_ch.write_all(b"ATD*99#\r\n").await?;
    match wait_for_dial_result(ppp_ch, Duration::from_secs(10)).await {
        DialResult::Connect => {
            info!("EC21-AU PPP CONNECT on DLCI 2");
            Ok(())
        }
        DialResult::Failed => {
            error!("EC21-AU PPP dial failed (NO CARRIER / ERROR)");
            Err(ModemError::PppDialFailed)
        }
        DialResult::Timeout => {
            error!("EC21-AU PPP dial timeout");
            Err(ModemError::PppDialFailed)
        }
    }
}

enum DialResult {
    Connect,
    Failed,
    Timeout,
}

async fn wait_for_dial_result<R: Read>(rx: &mut R, timeout: Duration) -> DialResult {
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
                    let slice = &window[..len];
                    if contains_bytes(slice, b"CONNECT") {
                        return DialResult::Connect;
                    }
                    if contains_bytes(slice, b"NO CARRIER") || contains_bytes(slice, b"ERROR") {
                        return DialResult::Failed;
                    }
                }
                _ => Timer::after_millis(10).await,
            }
        }
    };

    fut.with_timeout(timeout)
        .await
        .unwrap_or(DialResult::Timeout)
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
        let _ = tx.write_all(cmd).await;
        if wait_for_token(rx, b"OK\r\n", Duration::from_secs(2)).await {
            return Ok(());
        }
        Timer::after_millis(200).await;
    }
    Err(ModemError::CmuxSetupFailed)
}

/// Checks whether the EC21 PMIC/AP subsystem is ready by issuing `AT+CBC`.
async fn probe_pmic_ready(
    tx: &mut UartTx<'static, Async>,
    rx: &mut RingBufferedUartRx<'static>,
) -> bool {
    let _ = tx.write_all(b"AT+CBC\r\n").await;
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
                    let slice = &window[..len];
                    if contains_bytes(slice, b"OK\r\n") {
                        return contains_bytes(slice, b"+CBC:");
                    }
                    if contains_bytes(slice, b"ERROR\r\n") {
                        return false;
                    }
                }
                _ => Timer::after_millis(10).await,
            }
        }
    };

    fut.with_timeout(Duration::from_millis(600))
        .await
        .unwrap_or(false)
}

async fn drain_rx<R: Read>(rx: &mut R, idle_timeout: Duration) {
    let mut buf = [0u8; 64];
    while rx.read(&mut buf).with_timeout(idle_timeout).await.is_ok() {}
}

fn contains_bytes(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.len() >= needle.len() && haystack.windows(needle.len()).any(|w| w == needle)
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
                    if contains_bytes(&window[..len], token) {
                        return true;
                    }
                }
                _ => Timer::after_millis(10).await,
            }
        }
    };

    fut.with_timeout(timeout).await.unwrap_or(false)
}
