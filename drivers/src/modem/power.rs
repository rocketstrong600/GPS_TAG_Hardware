//! GNSS-aware power-saving control for the Quectel EC21-AU without cold-booting.
//!
//! ## Power Profiles (All Without Cutting VBAT)
//! | Mode | API | LTE / PPP | GNSS | Wake Latency | Est. Draw |
//! | --- | --- | --- | --- | --- | --- |
//! | **Low-Power Tracking** | [`enable_slow_clock`]`(true)` + [`gps::enable_with_rate`] + [`poll_fix_and_sleep`] | Registered & PPP UP (DRX sleep between polls) | Tracking (DPO + `fixrate`) | ~85–110 ms | ~20–35 mA |
//! | **Fast-Wake Standby** | [`enter_standby`] → [`resume_with_gnss`] | Registered & PPP UP | Paused (`AT+QGPSEND`, Hot Start kept) | ~85 ms LTE/PPP; ~1–5 s GNSS | ~1.5–2.5 mA |
//! | **Flight Mode Idle** | [`set_rf_mode`]`(RfMode::Flight)` + [`enter_standby`] | RF off (PPP must re-dial on resume) | Paused (Hot Start kept) | Few s LTE + PPP | ~0.8–1.0 mA |
//!
//! `PC8` (`uart_dtr`) is verified non-inverted: `HIGH` allows sleep, `LOW` wakes the EC21 in
//! ~80 ms while keeping LTE registration, `460800 bps` CMUX, and PPP intact.

use atat::asynch::AtatClient;
use defmt::{info, warn};
use embassy_time::{Duration, Timer};

use super::ModemPins;
use super::commands::{AT, GetFuncMode, GetSleepMode, SetFuncMode, SetSleepMode};
use super::gps;

/// Delay after pulling DTR active before sending UART traffic so the EC21 main clock wakes up.
const DTR_WAKE_SETTLE: Duration = Duration::from_millis(80);

/// Cellular RF functionality mode (`AT+CFUN`).
#[derive(Debug, Copy, Clone, Eq, PartialEq, defmt::Format)]
#[allow(dead_code)]
pub enum RfMode {
    /// `AT+CFUN=0`: Minimum functionality (RF + SIM powered off; lowest idle current without
    /// cutting VBAT, re-register takes a few seconds, drops active PPP).
    Minimum = 0,
    /// `AT+CFUN=1`: Full cellular + GNSS functionality.
    Full = 1,
    /// `AT+CFUN=4`: Flight mode (cellular RF off, SIM + GNSS still usable; drops active PPP).
    Flight = 4,
}

impl RfMode {
    fn from_raw(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Minimum),
            1 => Some(Self::Full),
            4 => Some(Self::Flight),
            _ => None,
        }
    }
}

/// Enables or disables DTR-controlled slow-clock sleep (`AT+QSCLK=1` / `AT+QSCLK=0`).
///
/// With `QSCLK=1` enabled, the EC21 automatically enters low-power DRX sleep (~1.5–2.5 mA when
/// GNSS is off) whenever `uart_dtr` is driven to the sleep level and UART/USB are idle, while
/// keeping LTE registered, CMUX open, and PPP up (wakes in <100 ms with no cold boot).
#[allow(dead_code)]
pub async fn enable_slow_clock<C: AtatClient>(
    client: &mut C,
    enable: bool,
) -> Result<(), atat::Error> {
    client
        .send(&SetSleepMode {
            mode: u8::from(enable),
        })
        .await?;
    info!("EC21 slow-clock sleep (AT+QSCLK) = {}", u8::from(enable));
    Ok(())
}

/// Queries whether DTR slow-clock sleep is currently enabled (`AT+QSCLK?`).
#[allow(dead_code)]
pub async fn is_slow_clock_enabled<C: AtatClient>(client: &mut C) -> Result<bool, atat::Error> {
    Ok(client.send(&GetSleepMode).await?.mode != 0)
}

/// Releases the modem `DTR` line (`PC8` HIGH) so the EC21 can enter slow-clock sleep once idle.
///
/// Does **not** cut VBAT, drop LTE registration, or tear down CMUX/PPP.
/// Note: if GNSS is still running, the LTE/AP side sleeps while GNSS continues tracking.
/// Call [`gps::disable`] first for lowest (~1.5–2.5 mA) standby draw; GNSS retains ephemeris
/// and XTRA in RAM for a ~1–5 s Hot Start on the next [`gps::enable`].
#[allow(dead_code)]
pub fn allow_sleep(pins: &mut ModemPins) {
    pins.uart_dtr.set_high();
}

/// Asserts the modem `DTR` line (`PC8` LOW) and waits for the UART/clock domain to wake (~80 ms).
///
/// Use this before sending PPP traffic on `DLCI 2` or AT commands on `DLCI 1` after calling
/// [`allow_sleep`].
#[allow(dead_code)]
pub async fn wake_pins(pins: &mut ModemPins) {
    pins.uart_dtr.set_low();
    Timer::after(DTR_WAKE_SETTLE).await;
}

/// Wakes the EC21 via `DTR` (`PC8` LOW) and verifies AT responsiveness on `DLCI 1`.
#[allow(dead_code)]
pub async fn wake<C: AtatClient>(pins: &mut ModemPins, client: &mut C) -> Result<(), atat::Error> {
    wake_pins(pins).await;
    for attempt in 0..3 {
        if client.send(&AT).await.is_ok() {
            return Ok(());
        }
        warn!("EC21 wake AT ping {} failed; retrying...", attempt + 1);
        Timer::after_millis(50).await;
    }
    client.send(&AT).await?;
    Ok(())
}

/// Convenience helper: cleanly pauses GNSS (`AT+QGPSEND`, preserving ephemeris/XTRA for Hot
/// Start) and allows EC21 slow-clock sleep (`DTR` HIGH). LTE + CMUX + PPP stay alive.
#[allow(dead_code)]
pub async fn enter_standby<C: AtatClient>(
    pins: &mut ModemPins,
    client: &mut C,
) -> Result<(), gps::GpsError> {
    gps::disable(client).await?;
    allow_sleep(pins);
    Ok(())
}

/// Convenience helper: wakes the EC21 via `DTR` and resumes GNSS (Hot Start) at `fix_rate_secs`.
#[allow(dead_code)]
pub async fn resume_with_gnss<C: AtatClient>(
    pins: &mut ModemPins,
    client: &mut C,
    fix_rate_secs: u16,
) -> Result<(), gps::GpsError> {
    wake(pins, client).await.map_err(gps::GpsError::At)?;
    if fix_rate_secs <= 1 {
        gps::enable(client).await
    } else {
        gps::enable_with_rate(client, fix_rate_secs).await
    }
}

/// Convenience helper: wakes the EC21 via `DTR`, queries the latest GNSS fix (`AT+QGPSLOC=2`),
/// and immediately returns `DTR` HIGH to allow slow-clock sleep again.
///
/// Ideal for low-power tracking loops where GNSS runs with DPO + `enable_with_rate` while the
/// cellular/AP domain sleeps in DRX between polls.
#[allow(dead_code)]
pub async fn poll_fix_and_sleep<C: AtatClient>(
    pins: &mut ModemPins,
    client: &mut C,
) -> Result<gps::GpsFix, gps::GpsError> {
    wake(pins, client).await.map_err(gps::GpsError::At)?;
    let res = gps::get_fix(client).await;
    allow_sleep(pins);
    res
}

/// Switches cellular RF functionality (`AT+CFUN=0|1|4`).
///
/// Warning: entering [`RfMode::Flight`] or [`RfMode::Minimum`] drops any active PPP data call
/// on `DLCI 2`. Call [`super::ppp::set_auto_redial`]`(false)` beforehand to pause background
/// re-dial attempts while RF is off, then [`super::ppp::set_auto_redial`]`(true)` after returning
/// to [`RfMode::Full`]. Prefer [`enable_slow_clock`] + [`allow_sleep`] when you want sub-100 ms
/// wake with PPP kept up.
#[allow(dead_code)]
pub async fn set_rf_mode<C: AtatClient>(client: &mut C, mode: RfMode) -> Result<(), atat::Error> {
    client.send(&SetFuncMode { fun: mode as u8 }).await?;
    info!("EC21 RF mode set to {:?}", mode);
    Ok(())
}

/// Queries the current cellular functionality mode (`AT+CFUN?`).
#[allow(dead_code)]
pub async fn get_rf_mode<C: AtatClient>(client: &mut C) -> Result<Option<RfMode>, atat::Error> {
    Ok(RfMode::from_raw(client.send(&GetFuncMode).await?.fun))
}
