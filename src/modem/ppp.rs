use atat::asynch::AtatClient;
use atat::heapless::String;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::dns::DnsQueryType;
use embassy_net::{Config as NetConfig, ConfigV4, Ipv4Address, Ipv4Cidr, Runner as NetRunner, Stack, StackResources, StaticConfigV4};
use embassy_net_ppp::{Config as PppConfig, Device as PppDevice, Runner as PppRunner, State as PppState};
use embassy_time::{Duration, Instant, Timer, WithTimeout};
use static_cell::StaticCell;

use super::cmux::CmuxChannel;
use super::commands::{GetEpsRegStatus, SetPdpContext};

#[derive(Debug, Clone, defmt::Format)]
pub enum PppError {
    At(atat::Error),
    RegistrationDenied,
    RegistrationTimeout,
}

impl From<atat::Error> for PppError {
    fn from(e: atat::Error) -> Self {
        Self::At(e)
    }
}

/// Configures PDP context 1 (`AT+CGDCONT=1,"IP","<apn>"`) on `DLCI 1`.
pub async fn configure_apn<C: AtatClient>(client: &mut C, apn: &str) -> Result<(), PppError> {
    let mut pdp_type = String::<8>::new();
    let _ = pdp_type.push_str("IP");
    let mut apn_str = String::<32>::new();
    let _ = apn_str.push_str(apn);

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

/// Polls `AT+CEREG?` on `DLCI 1` until registered on LTE (`stat = 1` or `5`).
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

/// Starts `embassy-net-ppp` and the `embassy-net` stack over CMUX `DLCI 2`.
pub fn start(spawner: &Spawner, ppp_ch: CmuxChannel, random_seed: u64) -> Stack<'static> {
    static PPP_STATE: StaticCell<PppState<4, 4>> = StaticCell::new();
    static NET_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

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
    ppp_ch: CmuxChannel,
    stack: Stack<'static>,
) {
    let cfg = PppConfig {
        username: b"",
        password: b"",
    };

    let err = runner
        .run(ppp_ch, cfg, |status| {
            info!("PPP IPv4 up: {:?}", status);
            if let Some(addr) = status.address {
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

    error!("PPP runner exited: {:?}", err);
}

#[embassy_executor::task]
async fn net_runner_task(mut runner: NetRunner<'static, PppDevice<'static>>) {
    runner.run().await;
}

/// Waits for IPv4 config and runs a DNS lookup to verify end-to-end LTE data over `DLCI 2`.
pub async fn test_connectivity(stack: Stack<'static>) {
    info!("Waiting for PPP IPv4 config...");
    if stack
        .wait_config_up()
        .with_timeout(Duration::from_secs(20))
        .await
        .is_err()
    {
        error!("PPP IPv4 config timed out");
        return;
    }

    if let Some(cfg) = stack.config_v4() {
        info!("PPP IP ready: {}", cfg.address);
    }

    match stack.dns_query("example.com", DnsQueryType::A).await {
        Ok(addrs) => info!("DNS example.com -> {:?}", addrs.as_slice()),
        Err(e) => error!("DNS query failed: {:?}", e),
    }
}
