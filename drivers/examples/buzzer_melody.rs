//! Plays built-in melodies and individual notes on the `PB4` (`TIM3_CH1`) PWM buzzer while
//! blinking the status LED (`PB5`).
//!
//! Run on the STM32F412RG board:
//! ```bash
//! cargo run -p gps-tag-drivers --example buzzer_melody
//! ```

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use gps_tag_drivers::board;
use gps_tag_drivers::buzzer::{Note, songs};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut b = board::init();

    info!("Playing CHIRP...");
    b.led.set_high();
    b.buzzer.play(&songs::CHIRP).await;
    b.led.set_low();
    Timer::after_millis(500).await;

    info!("Playing NOKIA...");
    b.led.set_high();
    b.buzzer.play(&songs::NOKIA).await;
    b.led.set_low();
    Timer::after_secs(1).await;

    info!("Playing individual ascending tones...");
    for note in [Note::C5, Note::E5, Note::G5, Note::C6] {
        b.led.set_high();
        b.buzzer.beep(note, 150).await;
        b.led.set_low();
        Timer::after_millis(100).await;
    }

    info!("Done — blinking LED");
    loop {
        b.led.toggle();
        Timer::after_secs(1).await;
    }
}
