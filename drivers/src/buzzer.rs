//! Asynchronous PWM buzzer driver (`TIM3_CH1` on `PB4`) with musical note definitions and
//! built-in melodies ([`songs`]).
//!
//! Plays single tones via [`Buzzer::beep`] or full BPM-timed melodies via [`Buzzer::play`]
//! (positive dividers = standard notes, negative dividers = dotted notes).
//!
//! See `examples/buzzer_melody.rs` for a runnable example.

use embassy_stm32::time::hz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embassy_time::Timer;

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u16)]
pub enum Note {
    Rest = 0,
    B0 = 31,
    C1 = 33,
    CS1 = 35,
    D1 = 37,
    DS1 = 39,
    E1 = 41,
    F1 = 44,
    FS1 = 46,
    G1 = 49,
    GS1 = 52,
    A1 = 55,
    AS1 = 58,
    B1 = 62,
    C2 = 65,
    CS2 = 69,
    D2 = 73,
    DS2 = 78,
    E2 = 82,
    F2 = 87,
    FS2 = 93,
    G2 = 98,
    GS2 = 104,
    A2 = 110,
    AS2 = 117,
    B2 = 123,
    C3 = 131,
    CS3 = 139,
    D3 = 147,
    DS3 = 156,
    E3 = 165,
    F3 = 175,
    FS3 = 185,
    G3 = 196,
    GS3 = 208,
    A3 = 220,
    AS3 = 233,
    B3 = 247,
    C4 = 262,
    CS4 = 277,
    D4 = 294,
    DS4 = 311,
    E4 = 330,
    F4 = 349,
    FS4 = 370,
    G4 = 392,
    GS4 = 415,
    A4 = 440,
    AS4 = 466,
    B4 = 494,
    C5 = 523,
    CS5 = 554,
    D5 = 587,
    DS5 = 622,
    E5 = 659,
    F5 = 698,
    FS5 = 740,
    G5 = 784,
    GS5 = 831,
    A5 = 880,
    AS5 = 932,
    B5 = 988,
    C6 = 1047,
    CS6 = 1109,
    D6 = 1175,
    DS6 = 1245,
    E6 = 1319,
    F6 = 1397,
    FS6 = 1480,
    G6 = 1568,
    GS6 = 1661,
    A6 = 1760,
    AS6 = 1865,
    B6 = 1976,
    C7 = 2093,
    CS7 = 2217,
    D7 = 2349,
    DS7 = 2489,
    E7 = 2637,
    F7 = 2794,
    FS7 = 2960,
    G7 = 3136,
    GS7 = 3322,
    A7 = 3520,
    AS7 = 3729,
    B7 = 3951,
    C8 = 4186,
    CS8 = 4435,
    D8 = 4699,
    DS8 = 4978,
}

impl Note {
    #[inline]
    pub const fn freq_hz(self) -> u32 {
        self as u32
    }
}

/// A melody defined by BPM tempo and `(Note, divider)` pairs.
/// Positive dividers represent standard note values (e.g. `4` = quarter note, `8` = eighth note);
/// negative dividers represent dotted notes (e.g. `-4` = dotted quarter note).
#[derive(Copy, Clone)]
pub struct Song<'a> {
    pub tempo_bpm: u16,
    pub notes: &'a [(Note, i16)],
}

pub struct Buzzer<'d, T: GeneralInstance4Channel> {
    pwm: SimplePwm<'d, T>,
    channel: Channel,
    duty_percent: u8,
}

impl<'d, T: GeneralInstance4Channel> Buzzer<'d, T> {
    pub fn new(mut pwm: SimplePwm<'d, T>, channel: Channel) -> Self {
        pwm.channel(channel).set_duty_cycle_fully_off();
        pwm.channel(channel).enable();
        Self {
            pwm,
            channel,
            duty_percent: 50,
        }
    }

    #[allow(dead_code)]
    pub fn with_duty(mut self, duty_percent: u8) -> Self {
        self.duty_percent = duty_percent.min(100);
        self
    }

    pub fn stop(&mut self) {
        self.pwm.channel(self.channel).set_duty_cycle_fully_off();
    }

    #[allow(dead_code)]
    pub async fn beep(&mut self, note: Note, duration_ms: u64) {
        let freq = note.freq_hz();
        if freq != 0 {
            self.pwm.set_frequency(hz(freq));
            self.pwm
                .channel(self.channel)
                .set_duty_cycle_percent(self.duty_percent);
        } else {
            self.stop();
        }
        Timer::after_millis(duration_ms).await;
        self.stop();
    }

    pub async fn play(&mut self, song: &Song<'_>) {
        if song.tempo_bpm == 0 {
            return;
        }
        let whole_note_ms: u64 = 240_000 / song.tempo_bpm as u64;

        for &(note, divider) in song.notes {
            if divider == 0 {
                continue;
            }
            let base_ms = whole_note_ms / divider.unsigned_abs() as u64;
            let note_ms = if divider < 0 {
                base_ms * 3 / 2
            } else {
                base_ms
            };

            let sound_ms = note_ms * 9 / 10;
            let gap_ms = note_ms.saturating_sub(sound_ms);

            let freq = note.freq_hz();
            if freq != 0 {
                self.pwm.set_frequency(hz(freq));
                self.pwm
                    .channel(self.channel)
                    .set_duty_cycle_percent(self.duty_percent);
            } else {
                self.stop();
            }

            Timer::after_millis(sound_ms).await;
            self.stop();
            if gap_ms > 0 {
                Timer::after_millis(gap_ms).await;
            }
        }
    }
}

#[allow(dead_code)]
pub mod songs {
    use super::{Note, Song};

    pub const NOKIA: Song<'static> = Song {
        tempo_bpm: 180,
        notes: &[
            (Note::E5, 8),
            (Note::D5, 8),
            (Note::FS4, 4),
            (Note::GS4, 4),
            (Note::CS5, 8),
            (Note::B4, 8),
            (Note::D4, 4),
            (Note::E4, 4),
            (Note::B4, 8),
            (Note::A4, 8),
            (Note::CS4, 4),
            (Note::E4, 4),
            (Note::A4, 2),
        ],
    };

    pub const STARWARS: Song<'static> = Song {
        tempo_bpm: 108,
        notes: &[
            (Note::AS4, 8),
            (Note::AS4, 8),
            (Note::AS4, 8),
            (Note::F5, 2),
            (Note::C6, 2),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::G5, 8),
            (Note::F6, 2),
            (Note::C6, 4),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::G5, 8),
            (Note::F6, 2),
            (Note::C6, 4),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::AS5, 8),
            (Note::G5, 2),
            (Note::C5, 8),
            (Note::C5, 8),
            (Note::C5, 8),
            (Note::F5, 2),
            (Note::C6, 2),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::G5, 8),
            (Note::F6, 2),
            (Note::C6, 4),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::G5, 8),
            (Note::F6, 2),
            (Note::C6, 4),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::AS5, 8),
            (Note::G5, 2),
            (Note::C5, -8),
            (Note::C5, 16),
            (Note::D5, -4),
            (Note::D5, 8),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::G5, 8),
            (Note::F5, 8),
            (Note::F5, 8),
            (Note::G5, 8),
            (Note::A5, 8),
            (Note::G5, 4),
            (Note::D5, 8),
            (Note::E5, 4),
            (Note::C5, -8),
            (Note::C5, 16),
            (Note::D5, -4),
            (Note::D5, 8),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::G5, 8),
            (Note::F5, 8),
            (Note::C6, -8),
            (Note::G5, 16),
            (Note::G5, 2),
            (Note::Rest, 8),
            (Note::C5, 8),
            (Note::D5, -4),
            (Note::D5, 8),
            (Note::AS5, 8),
            (Note::A5, 8),
            (Note::G5, 8),
            (Note::F5, 8),
            (Note::F5, 8),
            (Note::G5, 8),
            (Note::A5, 8),
            (Note::G5, 4),
            (Note::D5, 8),
            (Note::E5, 4),
            (Note::C6, -8),
            (Note::C6, 16),
            (Note::F6, 4),
            (Note::DS6, 8),
            (Note::CS6, 4),
            (Note::C6, 8),
            (Note::AS5, 4),
            (Note::GS5, 8),
            (Note::G5, 4),
            (Note::F5, 8),
            (Note::C6, 1),
        ],
    };

    pub const CHIRP: Song<'static> = Song {
        tempo_bpm: 240,
        notes: &[(Note::C6, 16), (Note::E6, 16), (Note::G6, 8)],
    };

    pub const ERROR: Song<'static> = Song {
        tempo_bpm: 180,
        notes: &[(Note::G4, 8), (Note::Rest, 16), (Note::C4, 4)],
    };
}
