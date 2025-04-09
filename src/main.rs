#![no_std]
#![no_main]

use rp_pico as bsp;

use bsp::entry;
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionSio, Pin, PullDown, SioInput, SioOutput},
    pac,
    sio::Sio,
    timer::Timer,
    usb::UsbBus,
    watchdog::Watchdog,
};

use cortex_m::delay::Delay;

use embedded_hal::digital::{InputPin, OutputPin};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use fugit::ExtU32;

// --- Configuration ---
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // 12 MHz

// USB VID/PID
const USB_VENDOR_ID: u16 = 0x04d8;
const USB_PRODUCT_ID: u16 = 0xfd08;

// IRToy Protocol Constants
const COMMAND_VERSION: u8 = b'v';
const COMMAND_SMODE_ENTER: u8 = b's';
const COMMAND_SMODE_EXIT: u8 = 0x00;
const COMMAND_TX_ENABLE_HANDSHAKE: u8 = 0x26;
const COMMAND_TX_ENABLE_BYTE_COUNT: u8 = 0x24;
const COMMAND_TX_ENABLE_NOTIFY: u8 = 0x25;
const COMMAND_TX_START: u8 = 0x03;

const HARDWARE_VERSION: u8 = 0x30;
const FIRMWARE_VERSION_H: u8 = b'2';
const FIRMWARE_VERSION_L: u8 = b'1';

const SUMP_MODE_RESPONSE: &[u8] = b"S01"; // IR Toy response for entering Sump Mode

// Timing Constants (Based on IRToy protocol description)
// The protocol states the 16-bit value should be multiplied by 21.3333us.
// This implies the underlying timer tick is 21.3333us.
// 1 / 21.3333e-6 = 46875 Hz.
// However, the RP2040 HAL Timer runs at 1MHz (1 tick = 1us).
// For simplicity here, we'll measure in microseconds directly using the HAL timer.
// The *host* software interacting with this firmware would need to know
// that the 16-bit values represent microseconds directly, NOT units of 21.3333us.
const MICROS_PER_TICK: f32 = 1.0; // We measure directly in microseconds
const TIMEOUT_US: u32 = 1_700_000; // 1.7 seconds in microseconds
const TIMEOUT_TICKS: u16 = 0xFFFF; // Max value sent over USB for timeout/end

// GPIO Pins (Update these for your specific board layout)
type IrRxPin = Pin<bsp::hal::gpio::bank0::Gpio21, FunctionSio<SioInput>, PullDown>; // Example: GP2 Input
type IrTxPin = Pin<bsp::hal::gpio::bank0::Gpio10, FunctionSio<SioOutput>, PullDown>; // Example: GP3 Output
type LedPin = Pin<bsp::hal::gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullDown>; // Example: GP25 Onboard LED

// Application State
enum Mode {
    Idle,
    Sump,
    // Transmit mode implicitly handled by command processing
}

struct IrToyState {
    mode: Mode,
    tx_handshake_enabled: bool,
    tx_byte_count_enabled: bool,
    tx_notify_enabled: bool,
}

// Helper to write to USB Serial, handling potential blocking
fn serial_write_blocking(serial: &mut SerialPort<'static, UsbBus>, data: &[u8]) -> Result<usize, UsbError> {
    let mut written = 0;
    while written < data.len() {
        match serial.write(&data[written..]) {
            Ok(count) => written += count,
            Err(UsbError::WouldBlock) => { /* Spin or yield */ } // In a real app, might yield or sleep briefly
            Err(e) => return Err(e),
        }
    }
    Ok(written)
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap(); // Need this for Delay
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .expect("Clock setup failed");

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let _delay = Delay::new(_core.SYST, clocks.system_clock.freq().to_Hz()); // If needed for basic delays

    let pins = bsp::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIOs
    let mut ir_rx_pin: IrRxPin = pins.gpio21.into_pull_down_input(); // Active Low assumed typical for TSOP sensors
    let mut ir_tx_pin: IrTxPin = pins.gpio10.into_push_pull_output();
    let mut led_pin: LedPin = pins.gpio25.into_push_pull_output(); // Pico onboard LED

    ir_tx_pin.set_low().ok();
    led_pin.set_low().ok();

    // --- USB Setup ---
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
    let usb_bus = unsafe {
        USB_BUS = Some(UsbBusAllocator::new(UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true, // Force VBUS detection
            &mut pac.RESETS,
        )));
        USB_BUS.as_ref().unwrap()
    };

    let mut serial = SerialPort::new(usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(USB_VENDOR_ID, USB_PRODUCT_ID))
        .device_class(0x02)
        .strings(&[
            StringDescriptors::new(LangID::EN_US)
                .manufacturer("flaviutamas.com")
                .product("RP2040 Infrared remote transciever (IR Toy)")
                .serial_number("0000001")
        ])
        .unwrap()
        .build();


    let mut state = IrToyState {
        mode: Mode::Idle,
        tx_handshake_enabled: false,
        tx_byte_count_enabled: false,
        tx_notify_enabled: false,
    };

    let mut read_buf = [0u8; 64]; // Buffer for incoming USB data

    loop {
        // --- USB Polling ---
        if usb_dev.poll(&mut [&mut serial]) {
            // Check for received data only if poll indicated activity
            match serial.read(&mut read_buf) {
                Ok(count) if count > 0 => {
                    led_pin.set_high().ok(); // Indicate activity

                    // Process received commands
                    for i in 0..count {
                        let command = read_buf[i];

                        match state.mode {
                            Mode::Idle => {
                                match command {
                                    COMMAND_VERSION => {
                                        let version_response = [
                                            b'V', // Response identifier
                                            HARDWARE_VERSION,
                                            FIRMWARE_VERSION_H,
                                            FIRMWARE_VERSION_L,
                                        ];
                                        serial_write_blocking(&mut serial, &version_response).ok();
                                    }
                                    COMMAND_SMODE_ENTER => {
                                        state.mode = Mode::Sump;
                                        serial_write_blocking(&mut serial, SUMP_MODE_RESPONSE).ok();
                                        // The actual sump logic runs below the command processing
                                    }
                                    COMMAND_TX_ENABLE_HANDSHAKE => {
                                        state.tx_handshake_enabled = true;
                                    }
                                    COMMAND_TX_ENABLE_BYTE_COUNT => {
                                        state.tx_byte_count_enabled = true;
                                    }
                                    COMMAND_TX_ENABLE_NOTIFY => {
                                        state.tx_notify_enabled = true;
                                    }
                                    COMMAND_TX_START => {
                                        // Only enter transmit if *all* required flags are set
                                        if state.tx_handshake_enabled && state.tx_byte_count_enabled && state.tx_notify_enabled {
                                            // Enter transmit mode (handled inline for simplicity)
                                            ir_transmit(&mut serial, &mut ir_tx_pin, &mut timer, &state);
                                        } else {
                                            // Optional: Send an error or ignore if flags not set
                                            // serial_write_blocking(&mut serial, b"E01").ok(); // Example error
                                        }
                                        // Transmit mode finishes within ir_transmit function
                                    }
                                    _ => { /* Ignore unknown commands in Idle mode */ }
                                }
                            }
                            Mode::Sump => {
                                // Only command recognized in Sump mode is Exit
                                if command == COMMAND_SMODE_EXIT {
                                    state.mode = Mode::Idle;
                                    // Break out of processing buffer, Sump logic loop will exit
                                    break;
                                }
                            }
                        }
                    }
                    led_pin.set_low().ok(); // Turn off LED after processing
                }
                Err(UsbError::WouldBlock) => { /* No data received */ }
                Err(_e) => { // Handle other USB read errors
                    // Maybe reset state or log error
                    state.mode = Mode::Idle;
                }
                _ => { /* Ok(0) or other cases */ }
            }
        }

        // --- Mode Logic ---
        match state.mode {
            Mode::Sump => {
                // Enter the Sump mode function. It will run until it exits
                // (timeout or 0x00 command received externally).
                ir_sample(&mut serial, &mut ir_rx_pin, &mut timer, &mut led_pin);
                // Once ir_sample returns, it means timeout or exit command occurred.
                // The command handler above should have already set state back to Idle
                // if 0x00 was received. If it timed out, set back to Idle here.
                state.mode = Mode::Idle;
            }
            Mode::Idle => {
                // Nothing specific to do in Idle other than process commands
            }
        }
    } // end loop
}


// --- IR Sampling (Sump Mode) ---
// Assumes active-low IR receiver (signal goes LOW when IR is detected)
fn ir_sample(
    serial: &mut SerialPort<'static, UsbBus>,
    ir_pin: &mut IrRxPin,
    timer: &mut Timer,
    led_pin: &mut LedPin,
) {
    let mut last_event_time = timer.get_counter();
    let mut buffer = [0u8; 64]; // Buffer for sending data
    let mut buf_idx = 0;

    loop {
        // --- Wait for initial pulse (pin goes low) ---
        while ir_pin.is_high().unwrap_or(true) {
            // Check for timeout (1.7s since last edge)
            if timer.get_counter().checked_duration_since(last_event_time)
                     .map_or(false, |d| d.to_micros() >= TIMEOUT_US as u64) {
                // Send timeout marker if needed (only after some activity)
                // The original protocol sends 0xFFFF *after* 1.7s of no activity.
                // This typically happens *after* the last blank measurement completes.
                // If we are *waiting* for the *first* pulse and 1.7s passes, we just return.
                return; // Exit Sump mode on timeout before first pulse
            }
            // Check for USB exit command (simple polling inside wait loop)
             // This is inefficient. Ideally USB poll would be interrupt driven or checked less frequently.
             // For this "dumb" implementation, we can't easily check USB here without
             // potentially missing short IR pulses. We rely on the main loop check.
             // If main loop detected 0x00, state.mode would be Idle, and this function
             // would have returned already before the next loop iteration.
        }
        led_pin.set_high().ok(); // Indicate receiving IR

        // --- Measure Pulse (pin is low) ---
        let pulse_start = timer.get_counter();
        while ir_pin.is_low().unwrap_or(false) {
            // Could add a check here for excessively long pulse (> 65535 us)
             if timer.get_counter().checked_duration_since(pulse_start)
                     .map_or(false, |d| d.to_micros() >= 65535u64) {
                 // Handle error or clamp? For now, just let it wrap or truncate.
                 break;
             }
        }
        let pulse_end = timer.get_counter();
        last_event_time = pulse_end; // Update last event time
        let pulse_ticks = pulse_end.checked_duration_since(pulse_start).map_or(0xFFFF, |v| v.to_micros());

        // Add pulse duration to buffer
        buffer[buf_idx..buf_idx + 2].copy_from_slice(&pulse_ticks.to_be_bytes());
        buf_idx += 2;


        // --- Measure Blank (pin is high) ---
        let blank_start = timer.get_counter();
        while ir_pin.is_high().unwrap_or(true) {
            // Check for timeout (1.7s since last edge) - this signifies end of transmission
             if timer.get_counter().checked_duration_since(last_event_time)
                      .map_or(false, |d| d.to_micros() >= TIMEOUT_US as u64) {
                 break; // Exit blank measurement loop on timeout
             }
        }
        let blank_end = timer.get_counter();
        let mut blank_ticks = blank_end.checked_duration_since(blank_start).map_or(0xFFFF, |v| v.to_micros()) as u16;


        // Check if the blank measurement loop exited due to timeout
        let timed_out = timer.get_counter().checked_duration_since(last_event_time)
                          .map_or(false, |d| d.to_micros() >= TIMEOUT_US as u64);

        if timed_out {
            blank_ticks = TIMEOUT_TICKS; // Use 0xFFFF to signify end
        } else {
             last_event_time = blank_end; // Update last event time only if not timed out
        }

         // Add blank duration to buffer
        buffer[buf_idx..buf_idx + 2].copy_from_slice(&blank_ticks.to_be_bytes());
        buf_idx += 2;


        // Send buffer if full or if timeout occurred
        if buf_idx >= buffer.len() || timed_out {
             serial_write_blocking(serial, &buffer[..buf_idx]).ok();
             buf_idx = 0; // Reset buffer index
        }

        led_pin.set_low().ok(); // Turn off LED after processing pulse/blank pair

        // If timeout occurred, end Sump mode
        if timed_out {
            return;
        }
        // No explicit check for 0x00 here, relying on the main loop's state change.
    }
}

// --- IR Transmit ---
// This function handles one transmission sequence based on data received over USB.
// Assumes active-high IR LED (set high to emit)
fn ir_transmit(
    serial: &mut SerialPort<'static, UsbBus>,
    ir_tx_pin: &mut IrTxPin,
    timer: &mut Timer,
    state: &IrToyState, // Read current flag settings
) {
    let mut data_buf = [0u8; 64]; // Buffer for timing data from host
    let mut buf_idx = 0; // Current position within data_buf for reading timings
    let mut bytes_in_buffer = 0; // How many valid bytes are in data_buf
    let mut total_bytes_processed: u16 = 0;
    let mut transmit_complete = false;
    let mut success = true; // Assume success unless underrun detected (hard in this simple model)

    // Initial handshake (required since we mandate flag 0x26)
    let initial_handshake: [u8; 1] = [62]; // Report buffer space (always 62 for now)
    serial_write_blocking(serial, &initial_handshake).ok();

    'transmit_loop: loop {
        // --- Receive Data Chunk ---
        // Try to read up to the buffer size, waiting for data
        while bytes_in_buffer < data_buf.len() {
            match serial.read(&mut data_buf[bytes_in_buffer..]) {
                Ok(count) => {
                    bytes_in_buffer += count;
                    if count > 0 { break; } // Got some data, proceed to process
                },
                Err(UsbError::WouldBlock) => { /* Continue waiting */ }
                Err(_) => {
                    success = false; // USB error during transmit
                    transmit_complete = true;
                    break 'transmit_loop;
                }
            }
            // Add a small delay or yield mechanism here if cpu usage is too high
            // cortex_m::asm::delay(1000); // crude delay
             // We also need a way to break if the host never sends data - a timeout?
             // For now, assume host sends data eventually or closes connection triggering error.
        }

        // Reset read index for the newly filled (or partially filled) buffer
        buf_idx = 0;

        // --- Process Received Data and Transmit IR ---
        while buf_idx < bytes_in_buffer {
             // Ensure we have at least 2 bytes for a duration pair
             if buf_idx + 1 >= bytes_in_buffer {
                 // Need more data, break processing and request more
                 break;
             }

            let high_byte = data_buf[buf_idx];
            let low_byte = data_buf[buf_idx + 1];
            let duration_ticks = u16::from_be_bytes([high_byte, low_byte]);
            total_bytes_processed = total_bytes_processed.saturating_add(2); // Count bytes processed
            buf_idx += 2;

            if duration_ticks == TIMEOUT_TICKS { // Check for end marker (0xFFFF)
                 // This should be the last blank duration.
                 // The protocol implies 0xFFFF *is* the final blank.
                 let duration_us = duration_ticks as u32; // Treat 0xFFFF as a duration value for delay
                 if duration_us > 0 {
                     // Delay for the final blank period
                    // Use Timer busy wait for more accuracy if needed
                    let target_time = timer.get_counter() + duration_us.micros();
                    while timer.get_counter() < target_time { /* busy wait */ }
                 }
                 transmit_complete = true;
                 break; // Exit processing loop for this buffer chunk
             }

            // First duration is PULSE
            let pulse_duration_us = duration_ticks as u32 * MICROS_PER_TICK as u32; // Assuming 1 tick = 1 us
            if pulse_duration_us > 0 {
                ir_tx_pin.set_high().ok();
                // Delay - Using timer busy wait loop for potentially better accuracy than cortex_m::delay
                let target_time = timer.get_counter() + pulse_duration_us.micros();
                 while timer.get_counter() < target_time { /* busy wait */ }
            }
            ir_tx_pin.set_low().ok(); // Pulse always ends with low


             // Ensure we have another 2 bytes for the blank duration
             if buf_idx + 1 >= bytes_in_buffer {
                 // Unexpected end of data after pulse, treat as error or request more?
                 // For simplicity, assume valid data stream or host handles error.
                 // Let's break and request more data.
                 buf_idx -= 2; // Rewind index to re-process pulse next time
                 total_bytes_processed = total_bytes_processed.saturating_sub(2);
                 break;
             }

            // Second duration is BLANK
            let high_byte_b = data_buf[buf_idx];
            let low_byte_b = data_buf[buf_idx + 1];
            let blank_duration_ticks = u16::from_be_bytes([high_byte_b, low_byte_b]);
            total_bytes_processed = total_bytes_processed.saturating_add(2);
            buf_idx += 2;


            if blank_duration_ticks == TIMEOUT_TICKS { // Check for end marker (0xFFFF) after pulse
                 // This is the final blank period
                 let duration_us = blank_duration_ticks as u32;
                 if duration_us > 0 {
                    let target_time = timer.get_counter() + duration_us.micros();
                    while timer.get_counter() < target_time { /* busy wait */ }
                 }
                 transmit_complete = true;
                 break; // Exit processing loop for this buffer chunk
            }

             let blank_duration_us = blank_duration_ticks as u32 * MICROS_PER_TICK as u32;
             if blank_duration_us > 0 {
                 // Delay for blank (IR LED already off)
                 let target_time = timer.get_counter() + blank_duration_us.micros();
                 while timer.get_counter() < target_time { /* busy wait */ }
             }
        } // End processing loop for this buffer chunk


        // Shift unprocessed data to the beginning of the buffer
        if buf_idx < bytes_in_buffer {
            // Copy remaining bytes `[buf_idx..bytes_in_buffer]` to `[0..]`
            data_buf.copy_within(buf_idx..bytes_in_buffer, 0);
            bytes_in_buffer -= buf_idx;
        } else {
            bytes_in_buffer = 0; // All data processed
        }

        // --- Handshake and Completion Check ---
        if transmit_complete {
            break 'transmit_loop; // Exit the main transmit loop
        } else if bytes_in_buffer < data_buf.len() {
             // If buffer isn't full and we processed some data, request more
             // Send handshake to request next chunk
             let handshake_req: [u8; 1] = [62]; // Requesting 62 bytes
             serial_write_blocking(serial, &handshake_req).ok();
        }
         // If buffer is still full (bytes_in_buffer == data_buf.len()) and we processed
         // data up to the end, loop will continue to request more data implicitly
         // when serial.read is called again.

    } // End 'transmit_loop

    ir_tx_pin.set_low().ok(); // Ensure TX pin is off

    // --- Post-Transmit Reporting (Required flags assumed set) ---

    // 1. Final Handshake (Protocol requires this even after 0xFFFF)
    let final_handshake: [u8; 1] = [62];
    serial_write_blocking(serial, &final_handshake).ok();

    // 2. Report Byte Count (if 0x24 enabled)
    if state.tx_byte_count_enabled {
        let count_response = [b't', // Identifier
                             (total_bytes_processed >> 8) as u8, // High byte
                             (total_bytes_processed & 0xFF) as u8]; // Low byte
        serial_write_blocking(serial, &count_response).ok();
    }

    // 3. Notify on Complete (if 0x25 enabled)
    if state.tx_notify_enabled {
        let status_code = if success { b'C' } else { b'F' }; // 'C' = Success, 'F' = Fail
        let notify_response = [status_code];
        serial_write_blocking(serial, &notify_response).ok();
    }
}
