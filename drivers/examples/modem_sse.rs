//! Demonstrates a persistent **Server-Sent Events (SSE)** command stream over TLS 1.3 HTTPS
//! on the Quectel EC21-AU `460800 bps` CMUX + PPP stack using
//! [`gps_tag_drivers::modem::https::sse`].
//!
//! 1. Boots EC21-AU, waits for LTE registration, and brings up PPP IPv4 on `DLCI 2`.
//! 2. Connects to a public HTTPS SSE endpoint (`https://ntfy.sh/gps-tag-test-demo/sse`) using
//!    hardware TRNG seeds from `b.rng`.
//! 3. Receives and logs parsed `SseEvent { event, id, data }` messages in an `async` callback
//!    (blinking the LED and chirping the buzzer on incoming events), automatically reconnecting
//!    with `Last-Event-ID` if the cellular link drops.
//!
//! Send a live command to the board from any terminal while it's running:
//! ```bash
//! curl -d "hello from server" https://ntfy.sh/gps-tag-test-demo
//! ```
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example modem_sse
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Duration;
use gps_tag_drivers::board;
use gps_tag_drivers::buzzer;
use gps_tag_drivers::modem::https::sse::{self, SseAction, SseConfig};
use gps_tag_drivers::modem::https::{HttpsBuffers, TlsVerify};
use gps_tag_drivers::modem::{self, ppp};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

const APN: &str = "mdata.net.au";
const SSE_URL: &str = "https://ntfy.sh/gps-tag-test-demo/sse";

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

    modem::dial_ppp(&mut ppp_ch).await.expect("PPP dial failed");
    let stack = ppp::start(&spawner, ppp_ch, b.rng.next_u64());

    let ip = ppp::wait_up(stack, Duration::from_secs(20))
        .await
        .expect("PPP IPv4 timeout");
    info!("PPP IPv4 up: {}", ip);

    static SSE_BUFS: StaticCell<HttpsBuffers> = StaticCell::new();
    let bufs = SSE_BUFS.init(HttpsBuffers::new());

    let mut cfg = SseConfig::new(SSE_URL, TlsVerify::None);
    cfg.idle_timeout = Some(Duration::from_secs(60));

    let mut count = 0u32;
    let rng = &mut b.rng;
    let led = &mut b.led;
    let buzz = &mut b.buzzer;

    info!("Connecting to HTTPS SSE stream: {}", SSE_URL);
    sse::listen(
        stack,
        bufs,
        || rng.next_u64(),
        &cfg,
        async |ev| {
            count += 1;
            led.toggle();
            info!(
                "SSE #{} [event={}, id={:?}]: {}",
                count, ev.event, ev.id, ev.data
            );
            if count <= 3 {
                buzz.play(&buzzer::songs::CHIRP).await;
            }
            SseAction::Continue
        },
    )
    .await;

    loop {
        embassy_time::Timer::after_secs(10).await;
    }
}
