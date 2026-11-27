//! Modem health, battery, signal, and network status helpers over CMUX `DLCI 1`.
//!
//! Wraps the Quectel EC21-AU diagnostic AT commands into typed structs:
//! - [`battery`] (`AT+CBC`): charge state, battery percentage (`0..=100%`), and voltage in `mV`
//! - [`signal`] (`AT+CSQ`): raw RSSI, approximate `dBm`, and BER
//! - [`lte_registration`] (`AT+CEREG?`): EPS/LTE registration state
//! - [`operator`] (`AT+COPS?`): registered carrier name (e.g. `"Telstra"`)
//! - [`network_utc`] (`AT+QLTS=1`): cellular network UTC timestamp (NITZ)
//! - [`adc_mv`] (`AT+QADC=<port>`): EC21 auxiliary ADC voltage in `mV`
//! - [`imei`], [`iccid`], [`sim_status`]: modem & SIM identity
//! - [`snapshot`] / [`log`]: one-call combined telemetry snapshot

#![allow(dead_code)]

use atat::asynch::AtatClient;
use atat::heapless::String;
use defmt::info;

use super::commands::{
    GetBatteryCharge, GetEpsRegStatus, GetIccid, GetImei, GetNetworkTime, GetOperator,
    GetSignalQuality, GetSimStatus, ReadAdc,
};

/// Battery charging state reported by `AT+CBC`.
#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub enum ChargeState {
    NotCharging,
    Charging,
    ChargingFinished,
    Unknown(u8),
}

impl ChargeState {
    fn from_raw(bcs: u8) -> Self {
        match bcs {
            0 => Self::NotCharging,
            1 => Self::Charging,
            2 => Self::ChargingFinished,
            other => Self::Unknown(other),
        }
    }
}

/// Battery status reported by the EC21-AU PMIC (`AT+CBC`).
#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub struct BatteryStatus {
    /// Charge status (`0` = not charging, `1` = charging, `2` = charging finished).
    pub state: ChargeState,
    /// Estimated battery charge level (`0..=100` percent).
    pub percent: u8,
    /// Battery / VBAT voltage in millivolts.
    pub voltage_mv: u16,
}

/// Cellular RF signal quality (`AT+CSQ`).
#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub struct SignalStatus {
    /// Raw CSQ RSSI (`0..=31`, or `99` when unknown/undetectable).
    pub rssi: u8,
    /// Approximate signal power in dBm (`-113 + 2 * rssi`), or `None` if unknown.
    pub rssi_dbm: Option<i16>,
    /// Channel bit error rate (`0..=7`, or `99` when unknown).
    pub ber: u8,
}

/// EPS / LTE network registration state (`AT+CEREG?`).
#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub enum LteRegState {
    NotSearching,
    RegisteredHome,
    Searching,
    Denied,
    Unknown,
    RegisteredRoaming,
    Other(u8),
}

impl LteRegState {
    fn from_raw(stat: u8) -> Self {
        match stat {
            0 => Self::NotSearching,
            1 => Self::RegisteredHome,
            2 => Self::Searching,
            3 => Self::Denied,
            4 => Self::Unknown,
            5 => Self::RegisteredRoaming,
            other => Self::Other(other),
        }
    }

    /// Returns `true` if registered on either home or roaming LTE network.
    pub fn is_registered(self) -> bool {
        matches!(self, Self::RegisteredHome | Self::RegisteredRoaming)
    }
}

/// Combined modem health and cellular telemetry snapshot returned by [`snapshot`].
#[derive(Debug, Clone)]
pub struct ModemStatus {
    pub battery: Option<BatteryStatus>,
    pub signal: Option<SignalStatus>,
    pub lte: Option<LteRegState>,
    pub operator: Option<String<32>>,
    pub network_utc: Option<String<32>>,
}

/// Reads battery charge state, percentage (`0..=100%`), and VBAT voltage in millivolts (`AT+CBC`).
pub async fn battery<C: AtatClient>(client: &mut C) -> Result<BatteryStatus, atat::Error> {
    let raw = client.send(&GetBatteryCharge).await?;
    Ok(BatteryStatus {
        state: ChargeState::from_raw(raw.bcs),
        percent: raw.bcl,
        voltage_mv: raw.voltage_mv,
    })
}

/// Reads cellular RF signal quality (`AT+CSQ`).
pub async fn signal<C: AtatClient>(client: &mut C) -> Result<SignalStatus, atat::Error> {
    let raw = client.send(&GetSignalQuality).await?;
    Ok(SignalStatus {
        rssi: raw.rssi,
        rssi_dbm: raw.rssi_dbm(),
        ber: raw.ber,
    })
}

/// Reads EPS / LTE registration state (`AT+CEREG?`).
pub async fn lte_registration<C: AtatClient>(client: &mut C) -> Result<LteRegState, atat::Error> {
    let raw = client.send(&GetEpsRegStatus).await?;
    Ok(LteRegState::from_raw(raw.stat))
}

/// Reads the currently selected cellular operator name (`AT+COPS?`), or `None` if unregistered.
pub async fn operator<C: AtatClient>(client: &mut C) -> Result<Option<String<32>>, atat::Error> {
    Ok(client.send(&GetOperator).await?.name)
}

/// Reads the network-synchronized UTC time string (`AT+QLTS=1`), if available from the carrier.
pub async fn network_utc<C: AtatClient>(client: &mut C) -> Result<String<32>, atat::Error> {
    Ok(client.send(&GetNetworkTime { mode: 1 }).await?.utc)
}

/// Reads an EC21-AU auxiliary ADC channel (`port` = `0` for `ADC0`, `1` for `ADC1`) in millivolts.
pub async fn adc_mv<C: AtatClient>(client: &mut C, port: u8) -> Result<u16, atat::Error> {
    let reading = client.send(&ReadAdc { port }).await?;
    if reading.status == 1 {
        Ok(reading.voltage_mv)
    } else {
        Err(atat::Error::Error)
    }
}

/// Reads the modem's 15-digit IMEI (`AT+GSN`).
pub async fn imei<C: AtatClient>(client: &mut C) -> Result<String<20>, atat::Error> {
    Ok(client.send(&GetImei).await?.imei)
}

/// Reads the SIM ICCID (`AT+QCCID`).
pub async fn iccid<C: AtatClient>(client: &mut C) -> Result<String<24>, atat::Error> {
    Ok(client.send(&GetIccid).await?.ccid)
}

/// Reads SIM PIN state (`AT+CPIN?`, typically `"READY"`).
pub async fn sim_status<C: AtatClient>(client: &mut C) -> Result<String<16>, atat::Error> {
    Ok(client.send(&GetSimStatus).await?.code)
}

/// Collects a best-effort [`ModemStatus`] snapshot (battery, signal, LTE registration, operator,
/// and network UTC time) over `DLCI 1`.
pub async fn snapshot<C: AtatClient>(client: &mut C) -> ModemStatus {
    let battery = battery(client).await.ok();
    let signal = signal(client).await.ok();
    let lte = lte_registration(client).await.ok();
    let operator = operator(client).await.ok().flatten();
    let network_utc = network_utc(client)
        .await
        .ok()
        .filter(|s| !s.trim().is_empty());

    ModemStatus {
        battery,
        signal,
        lte,
        operator,
        network_utc,
    }
}

/// Logs a [`ModemStatus`] snapshot via `defmt::info!`.
pub fn log(status: &ModemStatus) {
    if let Some(b) = status.battery {
        info!(
            "Modem battery: {}% ({} mV, {:?})",
            b.percent, b.voltage_mv, b.state
        );
    }
    if let Some(s) = status.signal {
        match s.rssi_dbm {
            Some(dbm) => info!("Modem signal: {} dBm (rssi={}, ber={})", dbm, s.rssi, s.ber),
            None => info!("Modem signal: unknown (rssi={}, ber={})", s.rssi, s.ber),
        }
    }
    if let Some(lte) = status.lte {
        info!(
            "Modem LTE: {:?}, operator={:?}",
            lte,
            status.operator.as_deref()
        );
    }
    if let Some(ref utc) = status.network_utc {
        info!("Modem UTC: {}", utc.as_str());
    }
}
