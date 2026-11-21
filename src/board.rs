use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::rcc::*;
use embassy_stm32::time::{Hertz, hz};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{Config, Peri, bind_interrupts, peripherals, usart};
use static_cell::StaticCell;

use crate::buzzer::Buzzer;
use crate::modem::ModemPins;

bind_interrupts!(struct Irqs {
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    DMA2_STREAM6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH6>;
    DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
    EXTI9_5 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
});

pub struct ImuPins {
    pub i2c1: Peri<'static, peripherals::I2C1>,
    pub scl: Peri<'static, peripherals::PB8>,
    pub sda: Peri<'static, peripherals::PB9>,
    #[allow(dead_code)]
    pub int1: ExtiInput<'static, Async>,
    #[allow(dead_code)]
    pub int2: ExtiInput<'static, Async>,
}

pub struct BlePins {
    pub uart: Uart<'static, Blocking>,
    pub reset: Output<'static>,
}

pub struct Board {
    pub led: Output<'static>,
    pub buzzer: Buzzer<'static, peripherals::TIM3>,
    pub modem_pins: ModemPins,
    pub modem_tx: UartTx<'static, Async>,
    pub modem_rx: RingBufferedUartRx<'static>,
    pub imu: ImuPins,
    pub ble: BlePins,
    pub flash: Peri<'static, peripherals::FLASH>,
    #[allow(dead_code)]
    pub vreg_en: Output<'static>,
    #[allow(dead_code)]
    pub stm_out_2: Output<'static>,
}

pub fn init() -> Board {
    let mut config = Config::default();
    config.enable_debug_during_sleep = true;
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.ls = LsConfig {
        rtc: RtcClockSource::LSE,
        lsi: false,
        lse: Some(LseConfig {
            frequency: Hertz(32_768),
            mode: LseMode::Oscillator(LseDrive::MediumLow),
        }),
    };
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.sys = Sysclk::HSE;

    let p = embassy_stm32::init(config);

    let led = Output::new(p.PB5, Level::Low, Speed::Low);

    let spkr = PwmPin::new(p.PB4, OutputType::PushPull);
    let spkr_pwm = SimplePwm::new(
        p.TIM3,
        Some(spkr),
        None,
        None,
        None,
        hz(520),
        Default::default(),
    );
    let buzzer = Buzzer::new(spkr_pwm, Channel::Ch1);

    let modem_pins = ModemPins {
        ap_ready: Output::new(p.PB0, Level::Low, Speed::Low),
        wake: Output::new(p.PB1, Level::Low, Speed::Low),
        batt: Output::new(p.PB12, Level::Low, Speed::Low),
        pwr_key: Output::new(p.PC4, Level::Low, Speed::Low),
        reset: Output::new(p.PA7, Level::Low, Speed::Low),
        uart_enable: Output::new(p.PB15, Level::High, Speed::Low),
        uart_dtr: Output::new(p.PC8, Level::Low, Speed::Low),
    };

    let m_uart = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        p.DMA2_CH6,
        p.DMA2_CH2,
        Irqs,
        usart::Config::default(),
    )
    .unwrap();
    let (modem_tx, m_rx) = m_uart.split();
    static RING_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let modem_rx = m_rx.into_ring_buffered(RING_BUF.init([0; 1024]));

    let ble_uart = Uart::new_blocking(p.USART2, p.PA3, p.PA2, usart::Config::default()).unwrap();

    Board {
        led,
        buzzer,
        modem_pins,
        modem_tx,
        modem_rx,
        imu: ImuPins {
            i2c1: p.I2C1,
            scl: p.PB8,
            sda: p.PB9,
            int1: ExtiInput::new(p.PB7, p.EXTI7, Pull::Down, Irqs),
            int2: ExtiInput::new(p.PB6, p.EXTI6, Pull::Down, Irqs),
        },
        ble: BlePins {
            uart: ble_uart,
            reset: Output::new(p.PC2, Level::Low, Speed::Low),
        },
        flash: p.FLASH,
        vreg_en: Output::new(p.PB14, Level::Low, Speed::Low),
        stm_out_2: Output::new(p.PB13, Level::Low, Speed::Low),
    }
}
