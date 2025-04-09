#![cfg_attr(not(test), no_std)]

use crate::Option::None;
use crate::Option::Some;
use core::clone::Clone;
use core::cmp::PartialEq;
use core::fmt::Debug;
use core::option::Option;
use core::prelude::rust_2024::derive;

/// Calculates the IR transmit frequency and duty cycle based on PR2 and duty cycle byte.
///
/// Follows the logic described for the USB IR Toy with a 48MHz Fosc and 4x prescaler.
/// Basically re-implements a PIC controller's PWM module.
///
/// # Arguments
///
/// * `pr2_value` - The u8 written to the PIC's PR2 register. determines PWM period/frequency.
/// * `duty_cycle_byte` - If 0x00, results in a 50% duty cycle.
///                       If non-zero, it's interpreted as setting the middle 8 bits of the
///                       10-bit PWM duty cycle register
pub fn calculate_ir_params(pr2_value: u8, duty_cycle_byte: u8) -> (f64, f64) {
    // IR Toy doc: "the clock (fosc) is 48Mhz"
    const FOSC: f64 = 48_000_000.0;
    // IR Toy doc: "the prescaler is fixed at 4x"
    const PRESCALER: f64 = 4.0; // Fixed Timer2 prescaler value
    // Microchip datasheet: "Timer2 module has one source of input clock, the device clock (FOSC/4)."
    const PWM_FREQUENCY: f64 = FOSC / 4.0 / PRESCALER;

    let modulation_hz = PWM_FREQUENCY / (pr2_value as f64 + 1.0);

    let duty_cycle = if duty_cycle_byte == 0x00 {
        0.5
    } else {
        // equation 4.3 from Microchip TB3275 datasheet
        (((duty_cycle_byte as u32) << 1) as f64 / (4.0 * pr2_value as f64 + 1.0)).min(0.99)
    };

    (modulation_hz, duty_cycle)
}

pub fn calculate_timings_us(frequency: f64, duty_cycle: f64) -> (u8, u8) {
    let period_us = 1_000_000.0 / frequency;
    let up_time_us = period_us * duty_cycle;
    let down_time_us = period_us - up_time_us;
    (up_time_us as u8, down_time_us as u8)
}

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Reset,
    SumpRun,
    SumpId,
    EnterTransmitMode,
    FrequencyReport,
    SetupSampleTimer,
    SetupTransmitModulation,
    LedMuteOn,
    LedMuteOff,
    LedOn,
    LedOff,
    GetSettingsDescriptor,
    EnableTransmitByteCount,
    EnableTransmitNotifyOnComplete,
    EnableTransmitHandshake,
    IoWrite,
    IoDirection,
    IoRead,
    UartSetup,
    UartClose,
    UartWrite,
    // Deprecated commands
    LittleEndianByteFormat,
    BigEndianByteFormat,
    LiyinNoTerminatorMode,
    // Special commands
    EnterIrSampleMode,
    GetVersion,
    Terminator,
}

impl Command {
    pub fn parse(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Command::Reset),
            0x01 => Some(Command::SumpRun),
            0x02 => Some(Command::SumpId),
            0x03 => Some(Command::EnterTransmitMode),
            0x04 => Some(Command::FrequencyReport),
            0x05 => Some(Command::SetupSampleTimer),
            0x06 => Some(Command::SetupTransmitModulation),
            0x10 => Some(Command::LedMuteOn),
            0x11 => Some(Command::LedMuteOff),
            0x12 => Some(Command::LedOn),
            0x13 => Some(Command::LedOff),
            0x23 => Some(Command::GetSettingsDescriptor),
            0x24 => Some(Command::EnableTransmitByteCount),
            0x25 => Some(Command::EnableTransmitNotifyOnComplete),
            0x26 => Some(Command::EnableTransmitHandshake),
            0x30 => Some(Command::IoWrite),
            0x31 => Some(Command::IoDirection),
            0x32 => Some(Command::IoRead),
            0x40 => Some(Command::UartSetup),
            0x41 => Some(Command::UartClose),
            0x42 => Some(Command::UartWrite),
            // Deprecated commands
            0x20 => Some(Command::LittleEndianByteFormat),
            0x21 => Some(Command::BigEndianByteFormat),
            0x22 => Some(Command::LiyinNoTerminatorMode),
            // Special ASCII commands
            b's' | b'S' => Some(Command::EnterIrSampleMode),
            b'v' | b'V' => Some(Command::GetVersion),
            0xFF => Some(Command::Terminator),
            _ => None,
        }
    }

    pub fn as_byte(&self) -> u8 {
        match self {
            Command::Reset => 0x00,
            Command::SumpRun => 0x01,
            Command::SumpId => 0x02,
            Command::EnterTransmitMode => 0x03,
            Command::FrequencyReport => 0x04,
            Command::SetupSampleTimer => 0x05,
            Command::SetupTransmitModulation => 0x06,
            Command::LedMuteOn => 0x10,
            Command::LedMuteOff => 0x11,
            Command::LedOn => 0x12,
            Command::LedOff => 0x13,
            Command::GetSettingsDescriptor => 0x23,
            Command::EnableTransmitByteCount => 0x24,
            Command::EnableTransmitNotifyOnComplete => 0x25,
            Command::EnableTransmitHandshake => 0x26,
            Command::IoWrite => 0x30,
            Command::IoDirection => 0x31,
            Command::IoRead => 0x32,
            Command::UartSetup => 0x40,
            Command::UartClose => 0x41,
            Command::UartWrite => 0x42,
            Command::LittleEndianByteFormat => 0x20,
            Command::BigEndianByteFormat => 0x21,
            Command::LiyinNoTerminatorMode => 0x22,
            Command::EnterIrSampleMode => b's',
            Command::GetVersion => b'v',
            Command::Terminator => 0xFF,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::calculate_ir_params;

    #[test]
    fn test_ir_params() {
        fn assert_ir_params(pr2: u8, ccp: u8, expected_freq: f64, expected_duty: f64) {
            let (freq, duty) = calculate_ir_params(pr2, ccp);
            assert!((freq - expected_freq).abs() < 0.01);
            assert!((duty - expected_duty).abs() < 0.01);
        }

        for (pr2, ccp, freq, duty) in [
            (0x4d, 0x00, 38461.54, 0.5),
            (0x4d, 0xff, 38461.54, 0.99), // cap out at 99%
            (0x4d, 0x1f, 38461.54, 0.1987),
            (0x4d, 0x01, 38461.54, 0.0064),
            (0x52, 0x00, 36144.58, 0.5),
            (0x53, 0x00, 35714.29, 0.5),
        ] {
            assert_ir_params(pr2, ccp, freq, duty);
        }
    }
}
