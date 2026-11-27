//! PPP (`embassy-net-ppp`) and IPv4 `embassy-net` stack integration over CMUX `DLCI 2`.
//!
//! Typical sequence:
//! 1. [`configure_apn`] and [`wait_for_lte_registration`] on `DLCI 1`.
//! 2. [`super::dial_ppp`] on `DLCI 2`.
//! 3. [`start`] spawns the PPP (with automatic re-dial on drop) and `embassy-net` tasks and
//!    returns a `Stack<'static>`.
//! 4. [`wait_up`] waits for IPv4 address + DNS servers.
//!
//! If the cellular network drops the PPP session (`LCP Terminate` / `NO CARRIER`), the background
//! PPP task automatically clears IPv4 config, returns `DLCI 2` to AT command mode (using Hayes
//! `+++` + `ATH` if needed), and re-dials `ATD*99#` with exponential backoff (`3 s`..`30 s`).
//! Use [`set_auto_redial`] to pause retries during intentional flight mode and [`request_redial`]
//! to trigger an immediate retry.
//!
//! Includes [`run_perf_test`] for live concurrent network throughput + 1 Hz GPS benchmarking.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use atat::asynch::AtatClient;
use atat::heapless::String;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::dns::DnsQueryType;
use embassy_net::{
    Config as NetConfig, ConfigV4, Ipv4Address, Ipv4Cidr, Runner as NetRunner, Stack,
    StackResources, StaticConfigV4,
};
use embassy_net_ppp::{
    Config as PppConfig, Device as PppDevice, Runner as PppRunner, State as PppState,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, WithTimeout};
use static_cell::StaticCell;

use super::cmux::PppChannel;
use super::commands::{GetEpsRegStatus, SetPdpContext};

static AUTO_REDIAL: AtomicBool = AtomicBool::new(true);
static REDIAL_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Enables or disables automatic PPP re-dialing (`ATD*99#`) when the PPP session drops.
///
/// Enabled by default (`true`). Set to `false` before entering [`super::power::RfMode::Flight`]
/// to avoid background re-dial attempts while cellular RF is intentionally off, then re-enable
/// with `true` when returning to [`super::power::RfMode::Full`] (which immediately wakes any
/// waiting backoff timer).
#[allow(dead_code)]
pub fn set_auto_redial(enabled: bool) {
    AUTO_REDIAL.store(enabled, Ordering::Relaxed);
    if enabled {
        REDIAL_WAKE.signal(());
    }
}

/// Immediately wakes any pending PPP reconnect backoff timer so `DLCI 2` re-dials right away.
#[allow(dead_code)]
pub fn request_redial() {
    REDIAL_WAKE.signal(());
}

#[derive(Debug, Clone, defmt::Format)]
pub enum PppError {
    At(atat::Error),
    InvalidApn,
    RegistrationDenied,
    RegistrationTimeout,
    ConfigTimeout,
}

impl From<atat::Error> for PppError {
    fn from(e: atat::Error) -> Self {
        Self::At(e)
    }
}

/// Configures PDP context 1 (`AT+CGDCONT=1,"IP","<apn>"`) on `DLCI 1`.
pub async fn configure_apn<C: AtatClient>(client: &mut C, apn: &str) -> Result<(), PppError> {
    if apn.is_empty() || apn.len() > 32 {
        return Err(PppError::InvalidApn);
    }
    let mut pdp_type = String::<8>::new();
    let _ = pdp_type.push_str("IP");
    let mut apn_str = String::<32>::new();
    apn_str.push_str(apn).map_err(|_| PppError::InvalidApn)?;

    client
        .send(&SetPdpContext {
            cid: 1,
            pdp_type,
            apn: apn_str,
        })
        .await?;
    info!("EC21-AU APN configured: {}", apn);
    Ok(())
}

/// Polls `AT+CEREG?` on `DLCI 1` until registered on LTE (`stat = 1` home or `5` roaming).
pub async fn wait_for_lte_registration<C: AtatClient>(
    client: &mut C,
    timeout: Duration,
) -> Result<(), PppError> {
    let deadline = Instant::now() + timeout;
    loop {
        match client.send(&GetEpsRegStatus).await {
            Ok(reg) => match reg.stat {
                1 | 5 => {
                    info!("EC21-AU LTE registered (stat={})", reg.stat);
                    return Ok(());
                }
                3 => {
                    error!("EC21-AU LTE registration denied (stat=3)");
                    return Err(PppError::RegistrationDenied);
                }
                other => {
                    info!("Waiting for LTE registration (stat={})...", other);
                }
            },
            Err(e) => warn!("CEREG query error: {:?}", e),
        }
        if Instant::now() >= deadline {
            return Err(PppError::RegistrationTimeout);
        }
        Timer::after_secs(2).await;
    }
}

/// Starts `embassy-net-ppp` (with automatic re-dial on disconnect) and the `embassy-net` stack
/// over CMUX `DLCI 2`.
pub fn start(spawner: &Spawner, ppp_ch: PppChannel, random_seed: u64) -> Stack<'static> {
    super::cmux::set_ppp_framing(true);

    static PPP_STATE: StaticCell<PppState<4, 4>> = StaticCell::new();
    static NET_RESOURCES: StaticCell<StackResources<6>> = StaticCell::new();

    let (device, ppp_runner) = embassy_net_ppp::new(PPP_STATE.init(PppState::new()));
    let (stack, net_runner) = embassy_net::new(
        device,
        NetConfig::default(),
        NET_RESOURCES.init(StackResources::new()),
        random_seed,
    );

    spawner.spawn(ppp_runner_task(ppp_runner, ppp_ch, stack).unwrap());
    spawner.spawn(net_runner_task(net_runner).unwrap());

    stack
}

#[embassy_executor::task]
async fn ppp_runner_task(
    mut runner: PppRunner<'static>,
    mut ppp_ch: PppChannel,
    stack: Stack<'static>,
) {
    const INITIAL_BACKOFF_SECS: u64 = 3;
    const MAX_BACKOFF_SECS: u64 = 30;
    let mut backoff_secs = INITIAL_BACKOFF_SECS;

    loop {
        super::cmux::set_ppp_framing(true);

        let cfg = PppConfig {
            username: b"",
            password: b"",
        };
        let came_up = Cell::new(false);

        let err = runner
            .run(&mut ppp_ch, cfg, |status| {
                info!("PPP IPv4 up: {:?}", status);
                if let Some(addr) = status.address {
                    came_up.set(true);
                    let mut dns_servers = heapless::Vec::new();
                    for dns in status.dns_servers.into_iter().flatten() {
                        let _ = dns_servers.push(dns);
                    }
                    if dns_servers.is_empty() {
                        let _ = dns_servers.push(Ipv4Address::new(1, 1, 1, 1));
                    }
                    stack.set_config_v4(ConfigV4::Static(StaticConfigV4 {
                        address: Ipv4Cidr::new(addr, 32),
                        gateway: status.peer_address,
                        dns_servers,
                    }));
                }
            })
            .await;

        super::cmux::set_ppp_framing(false);
        stack.set_config_v4(ConfigV4::None);
        warn!("PPP session ended ({:?}); will auto-redial on DLCI 2", err);

        if came_up.get() {
            backoff_secs = INITIAL_BACKOFF_SECS;
        }

        loop {
            if !AUTO_REDIAL.load(Ordering::Relaxed) {
                info!("PPP auto-redial paused; waiting for resume...");
                REDIAL_WAKE.wait().await;
                backoff_secs = INITIAL_BACKOFF_SECS;
                continue;
            }

            REDIAL_WAKE.reset();
            let _ = REDIAL_WAKE
                .wait()
                .with_timeout(Duration::from_secs(backoff_secs))
                .await;

            if !AUTO_REDIAL.load(Ordering::Relaxed) {
                continue;
            }

            info!("Re-dialing PPP on DLCI 2 (ATD*99#)...");
            match super::redial_ppp(&mut ppp_ch).await {
                Ok(()) => {
                    info!("PPP re-dialed; restarting IPv4 session");
                    break;
                }
                Err(e) => {
                    backoff_secs = backoff_secs.saturating_mul(2).min(MAX_BACKOFF_SECS);
                    warn!(
                        "PPP re-dial failed ({:?}); retrying in {}s",
                        e, backoff_secs
                    );
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn net_runner_task(mut runner: NetRunner<'static, PppDevice<'static>>) {
    runner.run().await;
}

/// Waits up to `timeout` for PPP IPv4 configuration to come up and returns the assigned IPv4 address.
pub async fn wait_up(stack: Stack<'static>, timeout: Duration) -> Result<Ipv4Address, PppError> {
    if stack.wait_config_up().with_timeout(timeout).await.is_err() {
        return Err(PppError::ConfigTimeout);
    }
    stack
        .config_v4()
        .map(|cfg| cfg.address.address())
        .ok_or(PppError::ConfigTimeout)
}

/// Waits for IPv4 config and runs a DNS lookup to verify end-to-end LTE data over `DLCI 2`.
#[allow(dead_code)]
pub async fn test_connectivity(stack: Stack<'static>) {
    info!("Waiting for PPP IPv4 config...");
    match wait_up(stack, Duration::from_secs(20)).await {
        Ok(ip) => info!("PPP IP ready: {}", ip),
        Err(_) => {
            error!("PPP IPv4 config timed out");
            return;
        }
    }

    match stack.dns_query("example.com", DnsQueryType::A).await {
        Ok(addrs) => info!("DNS example.com -> {:?}", addrs.as_slice()),
        Err(e) => error!("DNS query failed: {:?}", e),
    }
}

/// Runs a concurrent network performance benchmark on `DLCI 2` (DNS latency, small HTTP RTT,
/// sustained HTTP download ~100 KB, and HTTP upload ~8 KB) while polling GNSS on `DLCI 1` at
/// 1 Hz to verify zero CMUX/PPP interference.
#[allow(dead_code)]
pub async fn run_perf_test<C: AtatClient>(stack: Stack<'static>, client: &mut C) {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    use embassy_futures::join::join;
    use embassy_net::tcp::TcpSocket;
    use embedded_io_async::Write as _;

    info!("=== Concurrent CMUX Network + GPS Performance Test ===");

    let done = AtomicBool::new(false);
    let gps_ok = AtomicU32::new(0);
    let gps_fix = AtomicU32::new(0);
    let gps_err = AtomicU32::new(0);
    let gps_max_ms = AtomicU32::new(0);

    let gps_task = async {
        while !done.load(Ordering::Relaxed) {
            let t0 = Instant::now();
            match super::gps::get_fix(client).await {
                Ok(fix) => {
                    let ms = t0.elapsed().as_millis() as u32;
                    gps_ok.fetch_add(1, Ordering::Relaxed);
                    gps_fix.fetch_add(1, Ordering::Relaxed);
                    gps_max_ms.fetch_max(ms, Ordering::Relaxed);
                    info!(
                        "[GPS] Fix({:?}) lat={} lon={} sats={} hdop={} ({} ms)",
                        fix.fix_type, fix.latitude, fix.longitude, fix.satellites, fix.hdop, ms
                    );
                }
                Err(super::gps::GpsError::NotFixed) => {
                    let ms = t0.elapsed().as_millis() as u32;
                    gps_ok.fetch_add(1, Ordering::Relaxed);
                    gps_max_ms.fetch_max(ms, Ordering::Relaxed);
                }
                Err(e) => {
                    gps_err.fetch_add(1, Ordering::Relaxed);
                    warn!("[GPS] AT error during net traffic: {:?}", e);
                }
            }
            Timer::after_secs(1).await;
        }
    };

    let net_task = async {
        let mut rx_buf = [0u8; 16384];
        let mut tx_buf = [0u8; 4096];
        let mut read_scratch = [0u8; 2048];

        // 1. DNS latency (3x)
        let mut dns_total_ms = 0u64;
        let mut dns_ok = 0u32;
        for i in 0..3 {
            let t0 = Instant::now();
            if let Ok(addrs) = stack.dns_query("example.com", DnsQueryType::A).await {
                let ms = t0.elapsed().as_millis();
                dns_total_ms += ms;
                dns_ok += 1;
                info!(
                    "[NET] DNS #{} example.com -> {:?} ({} ms)",
                    i + 1,
                    addrs.as_slice(),
                    ms
                );
            } else {
                warn!("[NET] DNS #{} failed", i + 1);
            }
        }
        if dns_ok > 0 {
            info!(
                "[NET] DNS avg: {} ms ({}/3 ok)",
                dns_total_ms / dns_ok as u64,
                dns_ok
            );
        }

        // 2. Small HTTP GET RTT (5x example.com:80)
        if let Ok(addrs) = stack.dns_query("example.com", DnsQueryType::A).await {
            if let Some(&ip) = addrs.first() {
                let mut ok_cnt = 0u32;
                let mut sum_ms = 0u64;
                for i in 0..5 {
                    let t0 = Instant::now();
                    let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
                    sock.set_timeout(Some(Duration::from_secs(10)));
                    if sock.connect((ip, 80)).await.is_ok() {
                        let req =
                            b"GET / HTTP/1.1\r\nHost: example.com\r\nConnection: close\r\n\r\n";
                        if sock.write_all(req).await.is_ok() {
                            let mut total = 0usize;
                            while let Ok(n) = sock.read(&mut read_scratch).await {
                                if n == 0 {
                                    break;
                                }
                                total += n;
                            }
                            let ms = t0.elapsed().as_millis();
                            ok_cnt += 1;
                            sum_ms += ms;
                            info!("[NET] Small GET #{}: {} B in {} ms", i + 1, total, ms);
                        }
                    } else {
                        warn!("[NET] Small GET #{} connect failed", i + 1);
                    }
                    sock.close();
                }
                if ok_cnt > 0 {
                    info!(
                        "[NET] Small GET summary: {}/5 ok, avg {} ms",
                        ok_cnt,
                        sum_ms / ok_cnt as u64
                    );
                }
            }
        }

        // 3. Sustained download throughput (3x ~33.5 KB xtra3grc.bin = ~100 KB)
        if let Ok(addrs) = stack
            .dns_query("xtrapath4.izatcloud.net", DnsQueryType::A)
            .await
        {
            if let Some(&ip) = addrs.first() {
                let mut dl_bytes = 0usize;
                let mut dl_ms = 0u64;
                let mut dl_ok = 0u32;
                for i in 0..3 {
                    let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
                    sock.set_timeout(Some(Duration::from_secs(20)));
                    let t_conn = Instant::now();
                    if sock.connect((ip, 80)).await.is_ok() {
                        let conn_ms = t_conn.elapsed().as_millis();
                        let t_xfer = Instant::now();
                        let req = b"GET /xtra3grc.bin HTTP/1.1\r\nHost: xtrapath4.izatcloud.net\r\nConnection: close\r\n\r\n";
                        if sock.write_all(req).await.is_ok() {
                            let mut bytes = 0usize;
                            while let Ok(n) = sock.read(&mut read_scratch).await {
                                if n == 0 {
                                    break;
                                }
                                bytes += n;
                            }
                            let ms = t_xfer.elapsed().as_millis().max(1);
                            let bps = (bytes as u64 * 1000) / ms;
                            dl_bytes += bytes;
                            dl_ms += ms;
                            dl_ok += 1;
                            info!(
                                "[NET] Download #{}: {} B in {} ms (conn={} ms) => {}.{:02} KB/s",
                                i + 1,
                                bytes,
                                ms,
                                conn_ms,
                                bps / 1024,
                                ((bps % 1024) * 100) / 1024
                            );
                        }
                    } else {
                        warn!("[NET] Download #{} connect failed", i + 1);
                    }
                    sock.close();
                }
                if dl_ok > 0 && dl_ms > 0 {
                    let avg_bps = (dl_bytes as u64 * 1000) / dl_ms;
                    info!(
                        "[NET] Download total: {}/3 ok, {} B in {} ms => {}.{:02} KB/s",
                        dl_ok,
                        dl_bytes,
                        dl_ms,
                        avg_bps / 1024,
                        ((avg_bps % 1024) * 100) / 1024
                    );
                }
            }
        }

        // 3b. Sustained low-RTT AU-edge download (100 KB from Cloudflare Sydney edge)
        if let Ok(addrs) = stack
            .dns_query("speed.cloudflare.com", DnsQueryType::A)
            .await
        {
            if let Some(&ip) = addrs.first() {
                let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
                sock.set_timeout(Some(Duration::from_secs(20)));
                let t_conn = Instant::now();
                if sock.connect((ip, 80)).await.is_ok() {
                    let conn_ms = t_conn.elapsed().as_millis();
                    let t_xfer = Instant::now();
                    let req = b"GET /__down?bytes=102400 HTTP/1.1\r\nHost: speed.cloudflare.com\r\nConnection: close\r\n\r\n";
                    if sock.write_all(req).await.is_ok() {
                        let mut bytes = 0usize;
                        while let Ok(n) = sock.read(&mut read_scratch).await {
                            if n == 0 {
                                break;
                            }
                            bytes += n;
                        }
                        let ms = t_xfer.elapsed().as_millis().max(1);
                        let bps = (bytes as u64 * 1000) / ms;
                        info!(
                            "[NET] Cloudflare AU 100 KB: {} B in {} ms (conn={} ms) => {}.{:02} KB/s",
                            bytes,
                            ms,
                            conn_ms,
                            bps / 1024,
                            ((bps % 1024) * 100) / 1024
                        );
                    }
                }
                sock.close();
            }
        }

        // 4. Upload throughput (8 KB HTTP POST to example.com:80)
        if let Ok(addrs) = stack.dns_query("example.com", DnsQueryType::A).await {
            if let Some(&ip) = addrs.first() {
                let mut sock = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
                sock.set_timeout(Some(Duration::from_secs(15)));
                if sock.connect((ip, 80)).await.is_ok() {
                    const BODY_LEN: usize = 8192;
                    let hdr = b"POST / HTTP/1.1\r\nHost: example.com\r\nContent-Length: 8192\r\nConnection: close\r\n\r\n";
                    let chunk = [b'A'; 512];
                    let t_up = Instant::now();
                    let mut sent_ok = sock.write_all(hdr).await.is_ok();
                    let mut rem = BODY_LEN;
                    while sent_ok && rem > 0 {
                        let n = rem.min(chunk.len());
                        sent_ok = sock.write_all(&chunk[..n]).await.is_ok();
                        rem -= n;
                    }
                    if sent_ok {
                        let _ = sock.flush().await;
                        // Wait for server response bytes so TCP ACK confirms full upload receipt
                        let mut resp_bytes = 0usize;
                        while let Ok(n) = sock.read(&mut read_scratch).await {
                            if n == 0 {
                                break;
                            }
                            resp_bytes += n;
                        }
                        let ms = t_up.elapsed().as_millis().max(1);
                        let bps = ((BODY_LEN + hdr.len()) as u64 * 1000) / ms;
                        info!(
                            "[NET] Upload 8 KB POST: {} B sent, {} B resp in {} ms => {}.{:02} KB/s",
                            BODY_LEN + hdr.len(),
                            resp_bytes,
                            ms,
                            bps / 1024,
                            ((bps % 1024) * 100) / 1024
                        );
                    } else {
                        warn!("[NET] Upload write failed");
                    }
                }
                sock.close();
            }
        }

        done.store(true, Ordering::Relaxed);
    };

    join(gps_task, net_task).await;

    info!(
        "=== Perf Summary: GPS polls={} (fixes={}, errs={}, max_at={} ms), CMUX errs={}, carrier_ppp={} ===",
        gps_ok.load(Ordering::Relaxed),
        gps_fix.load(Ordering::Relaxed),
        gps_err.load(Ordering::Relaxed),
        gps_max_ms.load(Ordering::Relaxed),
        super::cmux::consecutive_errors(),
        super::cmux::carrier_detected(super::cmux::DLCI_PPP),
    );
}
