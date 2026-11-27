//! Demonstrates concurrent `460800 bps` PPP networking (`embassy-net` on CMUX `DLCI 2`) and
//! live 1 Hz GNSS tracking (AT commands on CMUX `DLCI 1`) on the Quectel EC21-AU.
//!
//! Performs DNS resolution and an HTTP GET request to `example.com:80` over a `TcpSocket`
//! while GNSS continues running without interruption.
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example modem_ppp_http
//! ```

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::dns::DnsQueryType;
use embassy_net::tcp::TcpSocket;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use gps_tag_drivers::board;
use gps_tag_drivers::modem::{self, gps, ppp};

use {defmt_rtt as _, panic_probe as _};

const APN: &str = "mdata.net.au";

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut b = board::init();

    let (mut client, mut ppp_ch) = modem::init(&spawner, &mut b.modem_pins, b.modem_tx, b.modem_rx)
        .await
        .unwrap();

    ppp::configure_apn(&mut client, APN).await.unwrap();
    ppp::wait_for_lte_registration(&mut client, Duration::from_secs(30))
        .await
        .unwrap();

    gps::ensure_xtra(&mut client, APN).await;
    gps::enable(&mut client).await.unwrap();

    modem::dial_ppp(&mut ppp_ch).await.unwrap();
    let seed = Instant::now().as_ticks() ^ 0xA5A5_5A5A_1234_5678;
    let stack = ppp::start(&spawner, ppp_ch, seed);

    let ip = ppp::wait_up(stack, Duration::from_secs(20)).await.unwrap();
    info!("PPP up with IPv4: {}", ip);

    // Fetch example.com:80 over PPP while GNSS runs concurrently on DLCI 1.
    if let Ok(addrs) = stack.dns_query("example.com", DnsQueryType::A).await {
        if let Some(&remote_ip) = addrs.first() {
            let mut rx_buf = [0u8; 4096];
            let mut tx_buf = [0u8; 1024];
            let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
            sock.set_timeout(Some(Duration::from_secs(10)));

            let t0 = Instant::now();
            if sock.connect((remote_ip, 80)).await.is_ok() {
                let req = b"GET / HTTP/1.1\r\nHost: example.com\r\nConnection: close\r\n\r\n";
                if sock.write_all(req).await.is_ok() {
                    let mut scratch = [0u8; 512];
                    let mut total = 0usize;
                    while let Ok(n) = sock.read(&mut scratch).await {
                        if n == 0 {
                            break;
                        }
                        total += n;
                    }
                    info!(
                        "HTTP GET example.com:80 -> {} bytes in {} ms",
                        total,
                        t0.elapsed().as_millis()
                    );
                }
            } else {
                error!("TCP connect to example.com:80 failed");
            }
            sock.close();
        }
    }

    // Continue polling GNSS on DLCI 1.
    loop {
        match gps::get_fix(&mut client).await {
            Ok(fix) => info!(
                "Fix({:?}) lat={}, lon={}, sats={}",
                fix.fix_type, fix.latitude, fix.longitude, fix.satellites
            ),
            Err(gps::GpsError::NotFixed) => info!("GPS acquiring fix..."),
            Err(e) => warn!("GPS error: {:?}", e),
        }
        Timer::after_secs(2).await;
    }
}
