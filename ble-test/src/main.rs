#![no_std]
#![no_main]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::bind_interrupts;
use embassy_nrf::config::{Config as NrfConfig, LfclkSource};
use embassy_nrf::interrupt::Priority;
use embassy_nrf::mode::Async;
use embassy_nrf::peripherals;
use embassy_nrf::rng::Rng;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals as MpslPeripherals, raw as mpsl_raw};
use nrf_sdc::{self as sdc, Peripherals as SdcPeripherals, SoftdeviceController};
use static_cell::StaticCell;
use trouble_host::prelude::*;

use {defmt_rtt as _, panic_probe as _};

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;
const SDC_MEM_SIZE: usize = 4096;

const BLE_NAME: &str = "GPS-TAG";
const BLE_ADDR: [u8; 6] = [0x01, 0x54, 0x53, 0x50, 0x47, 0xC2];

bind_interrupts!(struct Irqs {
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    RNG => embassy_nrf::rng::InterruptHandler<peripherals::RNG>;
});

#[gatt_server(connections_max = CONNECTIONS_MAX, mutex_type = NoopRawMutex, attribute_table_size = 16)]
struct Server {
    bas: BatteryService,
}

#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 100)]
    level: u8,
}

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

/// Stop the LFCLK started by `embassy_nrf::init()` so `MPSL` can take clean ownership of `CLOCK`.
fn stop_lfclk_for_mpsl() {
    const CLOCK_TASKS_LFCLKSTOP: *mut u32 = 0x4000_000C as *mut u32;
    const CLOCK_LFCLKSTAT: *const u32 = 0x4000_0418 as *const u32;
    unsafe {
        core::ptr::write_volatile(CLOCK_TASKS_LFCLKSTOP, 1);
        while (core::ptr::read_volatile(CLOCK_LFCLKSTAT) & (1 << 16)) != 0 {}
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut nrf_cfg = NrfConfig::default();
    nrf_cfg.lfclk_source = LfclkSource::InternalRC;
    nrf_cfg.time_interrupt_priority = Priority::P2;
    nrf_cfg.gpiote_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(nrf_cfg);

    let lfclk_cfg = mpsl_raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl_raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: 16,
        rc_temp_ctiv: 2,
        accuracy_ppm: 500,
        skip_wait_lfclk_started: false,
    };

    stop_lfclk_for_mpsl();

    let mpsl_p = MpslPeripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let sdc_p = SdcPeripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    static RNG: StaticCell<Rng<'static, Async>> = StaticCell::new();
    let rng = RNG.init(Rng::new(p.RNG, Irqs));

    static SDC_MEM: StaticCell<sdc::Mem<SDC_MEM_SIZE>> = StaticCell::new();
    let sdc: SoftdeviceController<'static> = sdc::Builder::new()
        .unwrap()
        .support_adv()
        .support_peripheral()
        .build(sdc_p, rng, mpsl, SDC_MEM.init(sdc::Mem::new()))
        .unwrap();

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources)
        .set_random_address(Address::random(BLE_ADDR))
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: BLE_NAME,
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();
    let _ = &server.bas;

    info!("BLE stack ready, advertising as {}", BLE_NAME);

    let ble_task = async {
        let mut adv_data = [0u8; 31];
        let adv_len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::CompleteLocalName(BLE_NAME.as_bytes()),
            ],
            &mut adv_data[..],
        )
        .unwrap();

        loop {
            let acceptor = match peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..adv_len],
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(a) => a,
                Err(e) => {
                    warn!("advertise error: {:?}", e);
                    embassy_time::Timer::after_secs(1).await;
                    continue;
                }
            };

            let conn = match acceptor.accept().await {
                Ok(c) => match c.with_attribute_server(&server) {
                    Ok(gc) => gc,
                    Err(e) => {
                        warn!("with_attribute_server error: {:?}", e);
                        continue;
                    }
                },
                Err(e) => {
                    warn!("accept error: {:?}", e);
                    continue;
                }
            };

            info!("BLE client connected");
            loop {
                match conn.next().await {
                    GattConnectionEvent::Disconnected { reason } => {
                        info!("BLE disconnected: {:?}", reason);
                        break;
                    }
                    GattConnectionEvent::Gatt { event } => {
                        if let Ok(reply) = event.accept() {
                            reply.send().await;
                        }
                    }
                    _ => {}
                }
            }
        }
    };

    let _ = join(runner.run(), ble_task).await;
}
