//! STM32F412RG board initialization and peripheral pin grouping.
//!
//! Configures clocks (`16 MHz` HSE oscillator as `SYSCLK`, `48 MHz` PLL Q clock for hardware
//! `RNG`, `32.768 kHz` LSE oscillator for RTC, `TIM4` Embassy time driver), binds `USART6` /
//! `DMA2` / `EXTI9_5` / `RNG` interrupts, initializes the status LED (`PB5`), PWM buzzer (`PB4`),
//! hardware TRNG (`RNG`), modem UART (`USART6`, 4 KiB DMA RX ring buffer) and control GPIOs,
//! BMI160 `INT1`/`INT2` EXTI pins (`PB7`/`PB6`), ST25DV16K pins (`PC10`/`PC11`/`EXTI10`), and
//! holds the MBN52832 BLE chip in reset (`PC2` HIGH) by default.

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::rcc::*;
use embassy_stm32::rng::{self, Rng};
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
    RNG => rng::InterruptHandler<peripherals::RNG>;
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
    #[allow(dead_code)]
    pub uart: Uart<'static, Blocking>,
    #[allow(dead_code)]
    pub reset: Output<'static>,
}

pub struct NfcPins {
    pub scl: Peri<'static, peripherals::PC10>,
    pub sda: Peri<'static, peripherals::PC11>,
    #[allow(dead_code)]
    pub exti10: Peri<'static, peripherals::EXTI10>,
}

pub struct Board {
    pub led: Output<'static>,
    pub buzzer: Buzzer<'static, peripherals::TIM3>,
    #[allow(dead_code)]
    pub rng: Rng<'static, peripherals::RNG>,
    pub modem_pins: ModemPins,
    pub modem_tx: UartTx<'static, Async>,
    pub modem_rx: RingBufferedUartRx<'static>,
    pub imu: ImuPins,
    pub nfc: NfcPins,
    #[allow(dead_code)]
    pub ble: BlePins,
    #[allow(dead_code)]
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
    // Generate 48 MHz on PLL1_Q for the hardware TRNG (16 MHz / 8 * 96 / 4 = 48 MHz) while
    // keeping SYSCLK directly on the 16 MHz HSE oscillator.
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV8,
        mul: PllMul::MUL96,
        divp: None,
        divq: Some(PllQDiv::DIV4),
        divr: None,
    });
    config.rcc.sys = Sysclk::HSE;

    let p = embassy_stm32::init(config);

    let rng = Rng::new(p.RNG, Irqs);
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
    static RING_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
    let modem_rx = m_rx.into_ring_buffered(RING_BUF.init([0; 4096]));

    let ble_uart = Uart::new_blocking(p.USART2, p.PA3, p.PA2, usart::Config::default()).unwrap();

    Board {
        led,
        buzzer,
        rng,
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
        nfc: NfcPins {
            scl: p.PC10,
            sda: p.PC11,
            exti10: p.EXTI10,
        },
        ble: BlePins {
            uart: ble_uart,
            // Active-high via open-drain transistor: HIGH holds MBN52832 in reset.
            reset: Output::new(p.PC2, Level::High, Speed::Low),
        },
        flash: p.FLASH,
        vreg_en: Output::new(p.PB14, Level::Low, Speed::Low),
        stm_out_2: Output::new(p.PB13, Level::Low, Speed::Low),
    }
}
