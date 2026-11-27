//! Quectel EC21-AU multi-constellation GNSS and gpsOneXTRA3 assistance driver.
//!
//! - Uses `gnssconfig=1` (GPS + GLONASS + BeiDou + Galileo) and Qualcomm **Dynamic Power
//!   Optimization** (`dpoenable=1`) automatically in [`enable`] and [`enable_with_rate`].
//! - [`ensure_xtra`] downloads `xtra3grc.bin` (7-day GPS+GLONASS+BeiDou ephemeris, verified for
//!   Australia) with automatic `xtra2.bin` fallback, injecting NITZ UTC time (`AT+QLTS=1`).
//! - [`disable`] terminates GNSS cleanly via `AT+QGPSEND` so ephemeris, XTRA, and RTC stay hot
//!   in RAM for ~1–5 s Hot Starts.

use atat::asynch::AtatClient;
use atat::heapless::String;
use atat::{AtatCmd, CmeError, Error as AtatError};
use defmt::{info, warn};
use embassy_time::{Duration, WithTimeout};

use super::cmux;
use super::commands::{
    ActivatePdp, DeactivatePdp, DeleteFile, EndGpsSession, GetGpsLocation, GetNetworkTime,
    GpsLocation, HttpGet, HttpReadFile, InjectGpsXtraData, InjectGpsXtraTime, NoResponse,
    QueryGpsState, QueryGpsXtraData, SetGpsCfg, SetGpsPower, SetGpsPowerEx, SetGpsXtra, SetHttpCtx,
    SetPdpContext, Urc,
};

/// Qualcomm IZat XTRA3 (GPS+GLONASS+BeiDou, ideal for Australia) with XTRA2 fallback.
/// Note: `xtra3grcej.bin` (Galileo) is rejected by EC21AUTFAR02A09M4G (`valid_mins=0`).
const XTRA_SOURCES: &[(&[u8], &str)] = &[
    (
        b"http://xtrapath4.izatcloud.net/xtra3grc.bin",
        "RAM:xtra3grc.bin",
    ),
    (b"http://xtrapath4.izatcloud.net/xtra2.bin", "RAM:xtra2.bin"),
];

/// Quectel EC21 GNSS-specific CME error codes.
const CME_GNSS_SESSION_ONGOING: u16 = 504;
const CME_GNSS_SESSION_NOT_ACTIVE: u16 = 505;
const CME_GNSS_NOT_FIXED: u16 = 516;

#[derive(Debug, Clone, defmt::Format)]
pub enum GpsError {
    /// GNSS is active, but no satellite fix has been acquired yet (`+CME ERROR: 516`).
    NotFixed,
    /// GNSS session is not active (`+CME ERROR: 505`).
    Disabled,
    At(AtatError),
}

impl From<AtatError> for GpsError {
    fn from(e: AtatError) -> Self {
        match e {
            AtatError::CmeError(CmeError::ManufacturerSpecific(CME_GNSS_NOT_FIXED)) => {
                Self::NotFixed
            }
            AtatError::CmeError(CmeError::ManufacturerSpecific(CME_GNSS_SESSION_NOT_ACTIVE)) => {
                Self::Disabled
            }
            other => Self::At(other),
        }
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
pub enum FixType {
    NoFix,
    Fix2D,
    Fix3D,
}

/// Parsed GNSS position fix from `AT+QGPSLOC=2`.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct GpsFix {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f32,
    pub hdop: f32,
    pub speed_kmh: f32,
    pub course_deg: Option<f32>,
    pub fix_type: FixType,
    pub satellites: u8,
    pub utc: String<10>,
    pub date: String<6>,
}

impl GpsFix {
    fn from_raw(loc: GpsLocation) -> Result<Self, AtatError> {
        let lat_str = core::str::from_utf8(&loc.latitude).map_err(|_| AtatError::Parse)?;
        let lon_str = core::str::from_utf8(&loc.longitude).map_err(|_| AtatError::Parse)?;
        let latitude = lat_str
            .trim()
            .parse::<f64>()
            .map_err(|_| AtatError::Parse)?;
        let longitude = lon_str
            .trim()
            .parse::<f64>()
            .map_err(|_| AtatError::Parse)?;

        let nsat_str = core::str::from_utf8(&loc.nsat).unwrap_or("0").trim();
        let satellites = nsat_str.parse::<u8>().unwrap_or(0);

        let cog_str = core::str::from_utf8(&loc.cog).unwrap_or("").trim();
        let course_deg = cog_str.parse::<f32>().ok();

        let mut utc = String::<10>::new();
        if let Ok(s) = core::str::from_utf8(&loc.utc) {
            let _ = utc.push_str(s.trim());
        }
        let mut date = String::<6>::new();
        if let Ok(s) = core::str::from_utf8(&loc.date) {
            let _ = date.push_str(s.trim());
        }

        let fix_type = match loc.fix {
            2 => FixType::Fix2D,
            3 => FixType::Fix3D,
            _ => FixType::NoFix,
        };

        Ok(Self {
            latitude,
            longitude,
            altitude_m: loc.altitude,
            hdop: loc.hdop,
            speed_kmh: loc.spkm,
            course_deg,
            fix_type,
            satellites,
            utc,
            date,
        })
    }
}

struct RawHttpUrl<'a> {
    url: &'a [u8],
}

impl<'a> AtatCmd for RawHttpUrl<'a> {
    type Response = NoResponse;
    const MAX_TIMEOUT_MS: u32 = 5_000;
    const EXPECTS_RESPONSE_CODE: bool = true;

    fn write(&self, buf: &mut [u8]) -> usize {
        let n = self.url.len().min(buf.len());
        buf[..n].copy_from_slice(&self.url[..n]);
        n
    }

    fn parse(
        &self,
        resp: Result<&[u8], atat::InternalError>,
    ) -> Result<Self::Response, atat::Error> {
        resp?;
        Ok(NoResponse)
    }
}

/// Enables gpsOneXTRA (`AT+QGPSXTRA=1`), downloads the best supported XTRA file over LTE
/// if expired/missing (or when `force` is true), and injects network UTC time + XTRA data.
/// Must be called while GNSS is OFF.
pub async fn ensure_xtra<C: AtatClient>(client: &mut C, apn: &str) {
    ensure_xtra_inner(client, apn, false).await;
}

/// Forces a fresh XTRA download/injection even if cached XTRA is still valid.
#[allow(dead_code)]
pub async fn refresh_xtra<C: AtatClient>(client: &mut C, apn: &str) {
    ensure_xtra_inner(client, apn, true).await;
}

async fn ensure_xtra_inner<C: AtatClient>(client: &mut C, apn: &str, force: bool) {
    if let Err(e) = client.send(&SetGpsXtra { mode: 1 }).await {
        warn!("AT+QGPSXTRA=1 failed: {:?}", e);
        return;
    }

    if let Ok(xtra) = client.send(&QueryGpsXtraData).await {
        info!(
            "EC21 XTRA data: valid_mins={}, time={}",
            xtra.valid_mins,
            xtra.injected_time.as_str()
        );
        if !force && xtra.valid_mins > 60 {
            return;
        }
    }

    let Some(mut urc_sub) = super::subscribe_urc() else {
        warn!("No URC subscriber slot available for XTRA download");
        return;
    };

    // 1. Get network UTC time via NITZ (`AT+QLTS=1`)
    let mut utc = String::<24>::new();
    if let Ok(net_time) = client.send(&GetNetworkTime { mode: 1 }).await {
        // Format: "2026/11/20,10:20:00+44,0" -> take first 19 chars "YYYY/MM/DD,HH:MM:SS"
        let s = net_time.utc.as_str();
        if s.len() >= 19 {
            let _ = utc.push_str(&s[..19]);
        }
    }
    if utc.is_empty() {
        warn!("Network UTC time unavailable (AT+QLTS=1 empty); XTRA time not injected");
    }

    // 2. Configure PDP context 2 for modem-internal HTTP download
    let mut pdp_type = String::<8>::new();
    let _ = pdp_type.push_str("IP");
    let mut apn_str = String::<32>::new();
    if apn_str.push_str(apn).is_err() {
        warn!("XTRA APN too long");
        return;
    }
    let _ = client
        .send(&SetPdpContext {
            cid: 2,
            pdp_type,
            apn: apn_str,
        })
        .await;

    let mut ctx_param = String::<16>::new();
    let _ = ctx_param.push_str("contextid");
    let _ = client
        .send(&SetHttpCtx {
            param: ctx_param,
            value: 2,
        })
        .await;

    if let Err(e) = client.send(&ActivatePdp { cid: 2 }).await {
        warn!("XTRA PDP activation failed: {:?}", e);
        return;
    }

    // 3. Try XTRA sources in order (XTRA3 multi-constellation -> XTRA2 fallback)
    for &(url, ram_path) in XTRA_SOURCES {
        if let Ok(url_str) = core::str::from_utf8(url) {
            info!("Trying XTRA source: {}", url_str);
        }

        let mut url_cmd = String::<32>::new();
        let _ = core::fmt::write(&mut url_cmd, format_args!("AT+QHTTPURL={},10\r", url.len()));
        if cmux::write_at_raw(url_cmd.as_bytes()).await.is_err() {
            warn!("Failed to write AT+QHTTPURL");
            continue;
        }
        if !wait_connect(&mut urc_sub).await {
            warn!("AT+QHTTPURL did not receive CONNECT");
            continue;
        }

        if let Err(e) = client.send(&RawHttpUrl { url }).await {
            warn!("QHTTPURL body failed: {:?}", e);
            continue;
        }

        if let Err(e) = client.send(&HttpGet { timeout_s: 30 }).await {
            warn!("AT+QHTTPGET failed: {:?}", e);
            continue;
        }
        if !wait_http_get(&mut urc_sub).await {
            continue;
        }

        let mut path = String::<24>::new();
        let _ = path.push_str(ram_path);
        let _ = client.send(&DeleteFile { path: path.clone() }).await;

        if let Err(e) = client.send(&HttpReadFile { path: path.clone() }).await {
            warn!("AT+QHTTPREADFILE failed: {:?}", e);
            continue;
        }
        if !wait_http_readfile(&mut urc_sub).await {
            continue;
        }

        // Inject UTC time before XTRA data
        if !utc.is_empty() {
            if let Err(e) = client
                .send(&InjectGpsXtraTime {
                    op: 0,
                    utc: utc.clone(),
                    uncertainty: 1,
                    force: 1,
                    time_unc_ms: 5000,
                })
                .await
            {
                warn!("AT+QGPSXTRATIME failed: {:?}", e);
            }
        }

        let inject_res = client.send(&InjectGpsXtraData { path: path.clone() }).await;
        let _ = client.send(&DeleteFile { path }).await;

        match inject_res {
            Ok(_) => {
                // EC21 takes ~0.5-1s to validate and commit XTRA data before AT+QGPSXTRADATA? succeeds.
                for _ in 0..4 {
                    embassy_time::Timer::after_millis(400).await;
                    if let Ok(xtra) = client.send(&QueryGpsXtraData).await {
                        info!(
                            "EC21 XTRA injected ({})! valid_mins={}, time={}",
                            ram_path,
                            xtra.valid_mins,
                            xtra.injected_time.as_str()
                        );
                        if xtra.valid_mins > 0 {
                            let _ = client.send(&DeactivatePdp { cid: 2 }).await;
                            return;
                        }
                        warn!(
                            "XTRA {} returned OK but valid_mins=0; trying fallback",
                            ram_path
                        );
                        break;
                    }
                }
            }
            Err(e) => {
                warn!("AT+QGPSXTRADATA ({}) rejected: {:?}", ram_path, e);
            }
        }
    }

    let _ = client.send(&DeactivatePdp { cid: 2 }).await;
}

async fn wait_connect(
    urc_sub: &mut atat::UrcSubscription<
        'static,
        Urc,
        { super::URC_CAPACITY },
        { super::URC_SUBSCRIBERS },
    >,
) -> bool {
    let fut = async {
        loop {
            if matches!(urc_sub.next_message_pure().await, Urc::Connect) {
                return true;
            }
        }
    };
    fut.with_timeout(Duration::from_secs(2))
        .await
        .unwrap_or(false)
}

async fn wait_http_get(
    urc_sub: &mut atat::UrcSubscription<
        'static,
        Urc,
        { super::URC_CAPACITY },
        { super::URC_SUBSCRIBERS },
    >,
) -> bool {
    let fut = async {
        loop {
            if let Urc::HttpGet(res) = urc_sub.next_message_pure().await {
                info!(
                    "XTRA HTTP GET: err={}, status={:?}, len={:?}",
                    res.err, res.http_status, res.content_len
                );
                return res.err == 0 && res.http_status == Some(200);
            }
        }
    };
    fut.with_timeout(Duration::from_secs(20))
        .await
        .unwrap_or(false)
}

async fn wait_http_readfile(
    urc_sub: &mut atat::UrcSubscription<
        'static,
        Urc,
        { super::URC_CAPACITY },
        { super::URC_SUBSCRIBERS },
    >,
) -> bool {
    let fut = async {
        loop {
            if let Urc::HttpReadFile(res) = urc_sub.next_message_pure().await {
                info!("XTRA HTTP READFILE: err={}", res.err);
                return res.err == 0;
            }
        }
    };
    fut.with_timeout(Duration::from_secs(10))
        .await
        .unwrap_or(false)
}

/// Enables or disables Qualcomm GNSS Dynamic Power Optimization (`AT+QGPSCFG="dpoenable",<0|1>`).
///
/// When enabled, the GNSS engine automatically duty-cycles its RF front-end between satellite
/// measurements to reduce tracking current while maintaining a hot lock.
#[allow(dead_code)]
pub async fn set_dpo<C: AtatClient>(client: &mut C, enable: bool) -> Result<(), GpsError> {
    let mut param = String::<24>::new();
    let _ = param.push_str("dpoenable");
    client
        .send(&SetGpsCfg {
            param,
            value: u8::from(enable),
        })
        .await?;
    info!("EC21 GNSS DPO={}", enable);
    Ok(())
}

/// Returns `true` if the EC21 GNSS engine is currently active (`AT+QGPS?`).
#[allow(dead_code)]
pub async fn is_enabled<C: AtatClient>(client: &mut C) -> Result<bool, GpsError> {
    let state = client.send(&QueryGpsState).await?;
    Ok(state.enabled != 0)
}

/// Configures multi-constellation GNSS (`gnssconfig=1`: GPS+GLONASS+BeiDou+Galileo),
/// enables Dynamic Power Optimization (`dpoenable=1`), and starts continuous 1 Hz GNSS.
/// Idempotent if GNSS is already running (`CME 504`).
pub async fn enable<C: AtatClient>(client: &mut C) -> Result<(), GpsError> {
    configure_gnss_defaults(client).await;

    match client.send(&SetGpsPower { mode: 1 }).await {
        Ok(_) => {
            info!("EC21 GNSS enabled (1 Hz)");
            Ok(())
        }
        Err(AtatError::CmeError(CmeError::ManufacturerSpecific(CME_GNSS_SESSION_ONGOING))) => {
            info!("EC21 GNSS already enabled");
            Ok(())
        }
        Err(e) => Err(e.into()),
    }
}

/// Starts continuous GNSS with a configurable interval between fixes (`fix_rate_secs`,
/// `1..=65535`) via `AT+QGPS=1,30,50,0,<fix_rate_secs>` and DPO enabled.
///
/// Larger intervals (e.g. 5–30 s) let the GNSS engine duty-cycle internally instead of solving
/// every second while preserving a warm/hot fix. If GNSS is already running, restarts it at the
/// requested rate.
#[allow(dead_code)]
pub async fn enable_with_rate<C: AtatClient>(
    client: &mut C,
    fix_rate_secs: u16,
) -> Result<(), GpsError> {
    let rate = fix_rate_secs.max(1);
    configure_gnss_defaults(client).await;

    let cmd = SetGpsPowerEx {
        mode: 1,
        fixmaxtime: 30,
        fixmaxdist: 50,
        fixcount: 0, // continuous
        fixrate: rate,
    };

    match client.send(&cmd).await {
        Ok(_) => {
            info!("EC21 GNSS enabled (fixrate={}s)", rate);
            Ok(())
        }
        Err(AtatError::CmeError(CmeError::ManufacturerSpecific(CME_GNSS_SESSION_ONGOING))) => {
            disable(client).await?;
            client.send(&cmd).await?;
            info!("EC21 GNSS restarted (fixrate={}s)", rate);
            Ok(())
        }
        Err(e) => Err(e.into()),
    }
}

async fn configure_gnss_defaults<C: AtatClient>(client: &mut C) {
    let mut param = String::<24>::new();
    let _ = param.push_str("gnssconfig");
    let _ = client.send(&SetGpsCfg { param, value: 1 }).await;

    let mut dpo = String::<24>::new();
    let _ = dpo.push_str("dpoenable");
    if let Err(e) = client
        .send(&SetGpsCfg {
            param: dpo,
            value: 1,
        })
        .await
    {
        warn!("AT+QGPSCFG=\"dpoenable\",1 failed: {:?}", e);
    }
}

/// Cleanly stops the active GNSS session via `AT+QGPSEND`.
///
/// Stopping cleanly (instead of cutting modem VBAT) retains satellite ephemeris, XTRA data,
/// and RTC time in RAM so the next [`enable`] / [`enable_with_rate`] performs a ~1–5 s Hot Start
/// while letting the EC21 drop to ~1.5–2.5 mA slow-clock sleep in the meantime.
#[allow(dead_code)]
pub async fn disable<C: AtatClient>(client: &mut C) -> Result<(), GpsError> {
    match client.send(&EndGpsSession).await {
        Ok(_)
        | Err(AtatError::CmeError(CmeError::ManufacturerSpecific(CME_GNSS_SESSION_NOT_ACTIVE))) => {
            info!("EC21 GNSS disabled (AT+QGPSEND)");
            Ok(())
        }
        Err(e) => Err(e.into()),
    }
}

/// Queries the current GNSS fix in decimal degrees (`AT+QGPSLOC=2`).
/// Returns `Err(GpsError::NotFixed)` immediately when no fix is available yet.
pub async fn get_fix<C: AtatClient>(client: &mut C) -> Result<GpsFix, GpsError> {
    let raw = client.send(&GetGpsLocation { mode: 2 }).await?;
    GpsFix::from_raw(raw).map_err(GpsError::At)
}

#[allow(dead_code)]
pub async fn get_location<C: AtatClient>(client: &mut C) -> Result<GpsLocation, GpsError> {
    client
        .send(&GetGpsLocation { mode: 2 })
        .await
        .map_err(Into::into)
}
