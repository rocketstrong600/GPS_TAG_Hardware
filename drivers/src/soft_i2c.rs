//! Open-drain bit-banged I2C master implementing [`embedded_hal::i2c::I2c`].
//!
//! Designed for pins without a hardware I2C peripheral (e.g., `PC10`/`PC11` connected to the
//! ST25DV16K NFC tag). Features:
//! - ~100 kHz Standard Mode default timing (`5 µs` half-period, configurable via
//!   [`SoftI2c::set_half_period`])
//! - Open-drain `Flex` pins with internal pull-ups enabled (`Speed::Medium`, `Pull::Up`)
//! - I2C clock-stretching support (5 ms timeout), which also makes shared open-drain `GPO -> SCL`
//!   interrupt pulses safe even mid-transaction
//! - Automatic 9-clock bus recovery on initialization and runtime [`SoftI2c::swap_pins`]

use defmt::Format;
use embassy_stm32::gpio::{Flex, Pull, Speed};
use embassy_time::{Duration, Instant};
use embedded_hal::i2c::{
    Error as HalError, ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation,
};

/// Default half-clock delay (~5 us at 16 MHz HSE -> ~100 kHz Standard Mode I2C).
const DEFAULT_HALF_PERIOD_US: u64 = 5;
/// Maximum time to wait for a target to release SCL during clock stretching.
const CLOCK_STRETCH_TIMEOUT: Duration = Duration::from_millis(5);

#[derive(Debug, Copy, Clone, Eq, PartialEq, Format)]
pub enum SoftI2cError {
    /// Target did not acknowledge its address byte.
    NackAddress,
    /// Target did not acknowledge a data byte.
    NackData,
    /// SCL remained stuck low beyond the clock-stretch timeout.
    ClockStretchTimeout,
    /// Bus lines could not be released (SDA or SCL held low).
    BusStuck,
}

impl HalError for SoftI2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::NackAddress => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Self::NackData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Self::ClockStretchTimeout | Self::BusStuck => ErrorKind::Bus,
        }
    }
}

/// Open-drain bit-banged I2C master using two STM32 `Flex` pins.
///
/// Both pins are configured as open-drain with internal pull-ups enabled so the bus works
/// reliably with external pull-ups and remains functional even if pull-ups are weak.
pub struct SoftI2c<'d> {
    scl: Flex<'d>,
    sda: Flex<'d>,
    half_period: Duration,
}

impl<'d> SoftI2c<'d> {
    /// Creates a new ~100 kHz software I2C master and performs initial bus recovery if needed.
    pub fn new(mut scl: Flex<'d>, mut sda: Flex<'d>) -> Self {
        scl.set_high();
        sda.set_high();
        scl.set_as_input_output_pull(Speed::Medium, Pull::Up);
        sda.set_as_input_output_pull(Speed::Medium, Pull::Up);

        let mut i2c = Self {
            scl,
            sda,
            half_period: Duration::from_micros(DEFAULT_HALF_PERIOD_US),
        };
        i2c.bus_recover();
        i2c
    }

    /// Overrides the half-clock period (e.g. `Duration::from_micros(10)` for ~50 kHz).
    #[allow(dead_code)]
    pub fn set_half_period(&mut self, half_period: Duration) {
        self.half_period = half_period;
    }

    /// Swaps the SCL and SDA pins at runtime and re-runs bus recovery.
    pub fn swap_pins(&mut self) {
        core::mem::swap(&mut self.scl, &mut self.sda);
        self.scl.set_high();
        self.sda.set_high();
        self.bus_recover();
    }

    /// Returns `(scl_high, sda_high)` idle line levels for quick wiring diagnostics.
    pub fn line_levels(&self) -> (bool, bool) {
        (self.scl.is_high(), self.sda.is_high())
    }

    /// Clocks out up to 9 SCL pulses + STOP to release a slave holding SDA low after reset.
    pub fn bus_recover(&mut self) {
        self.sda.set_high();
        self.scl.set_high();
        self.delay_half();

        if self.sda.is_low() {
            for _ in 0..9 {
                self.scl.set_low();
                self.delay_half();
                self.scl.set_high();
                self.delay_half();
                if self.sda.is_high() {
                    break;
                }
            }
        }
        // Issue a clean STOP condition.
        self.sda.set_low();
        self.delay_half();
        self.scl.set_high();
        self.delay_half();
        self.sda.set_high();
        self.delay_half();
    }

    /// Probes whether a 7-bit I2C target acknowledges its write address.
    pub fn probe(&mut self, addr: u8) -> Result<bool, SoftI2cError> {
        self.start()?;
        let res = self.write_byte(addr << 1);
        self.stop();
        match res {
            Ok(acked) => Ok(acked),
            Err(e) => Err(e),
        }
    }

    #[inline]
    fn delay_half(&self) {
        embassy_time::block_for(self.half_period);
    }

    #[inline]
    fn scl_high_wait(&mut self) -> Result<(), SoftI2cError> {
        self.scl.set_high();
        let deadline = Instant::now() + CLOCK_STRETCH_TIMEOUT;
        while self.scl.is_low() {
            if Instant::now() >= deadline {
                return Err(SoftI2cError::ClockStretchTimeout);
            }
        }
        self.delay_half();
        Ok(())
    }

    fn start(&mut self) -> Result<(), SoftI2cError> {
        self.sda.set_high();
        self.scl_high_wait()?;
        if self.sda.is_low() {
            return Err(SoftI2cError::BusStuck);
        }
        self.sda.set_low();
        self.delay_half();
        self.scl.set_low();
        self.delay_half();
        Ok(())
    }

    fn repeated_start(&mut self) -> Result<(), SoftI2cError> {
        self.sda.set_high();
        self.delay_half();
        self.scl_high_wait()?;
        if self.sda.is_low() {
            return Err(SoftI2cError::BusStuck);
        }
        self.sda.set_low();
        self.delay_half();
        self.scl.set_low();
        self.delay_half();
        Ok(())
    }

    fn stop(&mut self) {
        self.sda.set_low();
        self.delay_half();
        let _ = self.scl_high_wait();
        self.sda.set_high();
        self.delay_half();
    }

    fn write_byte(&mut self, mut byte: u8) -> Result<bool, SoftI2cError> {
        for _ in 0..8 {
            if (byte & 0x80) != 0 {
                self.sda.set_high();
            } else {
                self.sda.set_low();
            }
            byte <<= 1;
            self.delay_half();
            self.scl_high_wait()?;
            self.scl.set_low();
            self.delay_half();
        }

        // Release SDA and sample 9th-clock ACK (low = ACK, high = NACK).
        self.sda.set_high();
        self.delay_half();
        self.scl_high_wait()?;
        let acked = self.sda.is_low();
        self.scl.set_low();
        self.delay_half();
        Ok(acked)
    }

    fn read_byte(&mut self, send_ack: bool) -> Result<u8, SoftI2cError> {
        let mut byte = 0u8;
        self.sda.set_high();

        for _ in 0..8 {
            self.delay_half();
            self.scl_high_wait()?;
            byte <<= 1;
            if self.sda.is_high() {
                byte |= 1;
            }
            self.scl.set_low();
        }

        // Drive ACK (low) or NACK (high) on the 9th clock.
        if send_ack {
            self.sda.set_low();
        } else {
            self.sda.set_high();
        }
        self.delay_half();
        self.scl_high_wait()?;
        self.scl.set_low();
        self.sda.set_high();
        self.delay_half();
        Ok(byte)
    }
}

impl ErrorType for SoftI2c<'_> {
    type Error = SoftI2cError;
}

impl I2c for SoftI2c<'_> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return match self.probe(address)? {
                true => Ok(()),
                false => Err(SoftI2cError::NackAddress),
            };
        }

        let res = (|| -> Result<(), SoftI2cError> {
            let mut prev_is_read: Option<bool> = None;

            for i in 0..operations.len() {
                let is_read = matches!(operations[i], Operation::Read(_));
                let next_is_read = operations
                    .get(i + 1)
                    .map(|op| matches!(op, Operation::Read(_)));

                // Emit START / REPEATED START whenever starting or switching direction.
                if prev_is_read != Some(is_read) {
                    if prev_is_read.is_none() {
                        self.start()?;
                    } else {
                        self.repeated_start()?;
                    }
                    let addr_byte = (address << 1) | u8::from(is_read);
                    if !self.write_byte(addr_byte)? {
                        return Err(SoftI2cError::NackAddress);
                    }
                }

                match &mut operations[i] {
                    Operation::Write(bytes) => {
                        for &b in bytes.iter() {
                            if !self.write_byte(b)? {
                                return Err(SoftI2cError::NackData);
                            }
                        }
                    }
                    Operation::Read(buf) => {
                        let more_reads_follow = next_is_read == Some(true);
                        let len = buf.len();
                        for (idx, slot) in buf.iter_mut().enumerate() {
                            let send_ack = (idx + 1 < len) || more_reads_follow;
                            *slot = self.read_byte(send_ack)?;
                        }
                    }
                }

                prev_is_read = Some(is_read);
            }
            Ok(())
        })();

        self.stop();
        res
    }
}
