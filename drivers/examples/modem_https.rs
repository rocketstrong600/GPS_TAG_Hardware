//! Demonstrates TLS 1.3 **HTTPS GET** and **HTTPS POST (JSON)** over the Quectel EC21-AU
//! `460800 bps` CMUX + PPP stack using [`gps_tag_drivers::modem::https`] (`reqwless` +
//! `embedded-tls`) seeded with the STM32F412RG hardware TRNG (`b.rng`).
//!
//! 1. Boots EC21-AU, waits for LTE registration, and brings up PPP IPv4 on `DLCI 2`.
//! 2. Reads live modem telemetry (`modem::status::snapshot`).
//! 3. Performs an HTTPS `GET` to `https://example.com/`.
//! 4. Performs an HTTPS `POST` (JSON) with live battery/signal telemetry to
//!    `https://httpbin.org/post`.
//!
//! > Note: This example uses `TlsVerify::None` so it runs out of the box without bundling a CA
//! > root certificate. For production endpoints, pass `TlsVerify::Certificate { ca, .. }` with
//! > your server's DER-encoded root CA certificate.
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example modem_https
//! ```

#![no_std]
#![no_main]

use core::fmt::Write as _;

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use gps_tag_drivers::board;
use gps_tag_drivers::modem::https::{self, HttpsBuffers, TlsVerify};
use gps_tag_drivers::modem::{self, ppp, status};
use heapless::String;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

const APN: &str = "mdata.net.au";

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut b = board::init();

    let (mut client, mut ppp_ch) = modem::init(&spawner, &mut b.modem_pins, b.modem_tx, b.modem_rx)
        .await
        .expect("EC21-AU init failed");

    ppp::configure_apn(&mut client, APN)
        .await
        .expect("APN config failed");
    ppp::wait_for_lte_registration(&mut client, Duration::from_secs(45))
        .await
        .expect("LTE registration failed");

    let st = status::snapshot(&mut client).await;
    status::log(&st);

    modem::dial_ppp(&mut ppp_ch).await.expect("PPP dial failed");
    let net_seed = b.rng.next_u64();
    let stack = ppp::start(&spawner, ppp_ch, net_seed);

    let ip = ppp::wait_up(stack, Duration::from_secs(20))
        .await
        .expect("PPP IPv4 timeout");
    info!("PPP IPv4 up: {}", ip);

    static HTTPS_BUFS: StaticCell<HttpsBuffers> = StaticCell::new();
    let bufs = HTTPS_BUFS.init(HttpsBuffers::new());
    let mut body = [0u8; 2048];

    // 1. HTTPS GET (TLS 1.3)
    info!("=== HTTPS GET https://example.com/ ===");
    let t0 = Instant::now();
    match https::get(
        stack,
        bufs,
        b.rng.next_u64(),
        "https://example.com/",
        &[],
        TlsVerify::None,
        &mut body,
    )
    .await
    {
        Ok(resp) => {
            info!(
                "HTTPS GET OK: status={}, bytes={} (truncated={}) in {} ms",
                resp.status,
                resp.body_len,
                resp.truncated,
                t0.elapsed().as_millis()
            );
            if let Ok(text) = resp.body_str(&body) {
                let preview = &text[..text.len().min(160)];
                info!("Body preview: {}", preview);
            }
        }
        Err(e) => error!("HTTPS GET failed: {:?}", e),
    }

    // 2. HTTPS POST JSON with live modem telemetry
    let mut payload = String::<128>::new();
    let batt_pct = st.battery.map_or(0, |x| x.percent);
    let batt_mv = st.battery.map_or(0, |x| x.voltage_mv);
    let rssi_dbm = st.signal.and_then(|x| x.rssi_dbm).unwrap_or(0);
    let _ = write!(
        payload,
        "{{\"battery_pct\":{},\"battery_mv\":{},\"rssi_dbm\":{}}}",
        batt_pct, batt_mv, rssi_dbm
    );

    info!(
        "=== HTTPS POST https://httpbin.org/post ({}) ===",
        payload.as_str()
    );
    let t1 = Instant::now();
    match https::post_json(
        stack,
        bufs,
        b.rng.next_u64(),
        "https://httpbin.org/post",
        payload.as_bytes(),
        &[],
        TlsVerify::None,
        &mut body,
    )
    .await
    {
        Ok(resp) => {
            info!(
                "HTTPS POST OK: status={}, bytes={} (truncated={}) in {} ms",
                resp.status,
                resp.body_len,
                resp.truncated,
                t1.elapsed().as_millis()
            );
            if let Ok(text) = resp.body_str(&body) {
                let preview = &text[..text.len().min(240)];
                info!("Response preview: {}", preview);
            }
        }
        Err(e) => warn!("HTTPS POST failed: {:?}", e),
    }

    info!("HTTPS demo complete.");
    loop {
        b.led.toggle();
        Timer::after_secs(1).await;
    }
}
