#![no_std]
#![no_main]

mod util;

use rp_pico as bsp;

use bsp::entry;
use defmt::{error, info, warn};
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionPio0, FunctionSio, Pin, PullDown, PullUp, SioInput},
    pac,
    pio::{PIOBuilder, PIOExt, PinDir, PIO0SM0, Tx},
    sio::Sio,
    timer::Timer,
    usb::UsbBus,
    watchdog::Watchdog,
};

use embedded_hal::digital::{InputPin, OutputPin};
use fugit::MicrosDurationU64;
use heapless::Deque;
use mylib::{Command, calculate_ir_params};
use rp_pico::hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

// --- Configuration ---
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // 12 MHz

// Responses / Identifiers
const HARDWARE_VERSION: u8 = 0x30;
const FIRMWARE_VERSION_H: u8 = b'2';
const FIRMWARE_VERSION_L: u8 = b'1';
const IR_SAMPLE_MODE_RESPONSE: &[u8] = b"S01";

// PIO clock = 125MHz / 125 = 1MHz.
// Carrier loop: jmp!x(1) + set[12](13) + set[11](12) + jmpx--(1) = 27 cycles = 27µs ≈ 37kHz
const PIO_CARRIER_CYCLE_US: u32 = 27;

type IrRxPin = Pin<hal::gpio::bank0::Gpio21, FunctionSio<SioInput>, PullUp>;
/// Onboard LED
type LedPin = Pin<hal::gpio::bank0::Gpio25, FunctionSio<hal::gpio::SioOutput>, PullDown>;

// --- State ---
#[derive(PartialEq, Eq, Clone, Copy, defmt::Format)]
enum DeviceState {
    Idle,
    IrSample, // Receiving IR
    #[allow(dead_code)]
    Transmit, // Transmitting IR
}

#[derive(Default)]
struct IrToyFlags {
    tx_handshake_enabled: bool,
    tx_byte_count_enabled: bool,
    tx_notify_enabled: bool,
}

// Helper to convert microseconds duration to IRToy 16-bit count
fn us_to_irtoy_count(us: MicrosDurationU64) -> u16 {
    let count = us.to_micros() / 21;
    if count > u16::MAX as u64 {
        u16::MAX
    } else {
        count as u16
    }
}

// Helper to convert IRToy 16-bit count to microseconds duration
fn irtoy_count_to_us(count: u16) -> u32 {
    (count as u32) * 21
}

/// Build the PIO program for IR TX carrier generation.
///
/// Protocol: pairs of u32 words written to TX FIFO:
///   word[0]: on_carrier_cycles  (number of ~37kHz carrier cycles; 0 = skip carrier)
///   word[1]: off_us - 1         (space duration minus 1 in µs; loops off_us times)
///
/// PIO clock: 125MHz / 125 = 1MHz (1µs per tick)
/// Carrier: 13µs high + 12µs low + 1µs jmp = 26µs; +1µs for loop-check jmp ≈ 37kHz
fn ir_tx_pio_program() -> pio::Program<32> {
    use pio::{Assembler, JmpCondition, OutDestination, SetDestination};
    let mut a = Assembler::<32>::new();

    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut do_off = a.label();
    let mut on_loop = a.label();

    // --- On phase: generate carrier for on_carrier_cycles cycles ---
    a.bind(&mut wrap_target);
    a.pull(false, true); // blocking pull → OSR = on_carrier_cycles
    a.out(OutDestination::X, 32); // X = on_carrier_cycles

    a.bind(&mut on_loop);
    a.jmp(JmpCondition::XIsZero, &mut do_off); // if X==0, skip carrier
    a.set_with_delay(SetDestination::PINS, 1, 12); // 13 cycles high
    a.set_with_delay(SetDestination::PINS, 0, 11); // 12 cycles low
    a.jmp(JmpCondition::XDecNonZero, &mut on_loop); // 1 cycle; X--, loop if nonzero

    // --- Off phase: hold pin low for (off_us) µs ---
    a.bind(&mut do_off);
    a.pull(false, true); // blocking pull → OSR = off_us - 1
    a.out(OutDestination::X, 32); // X = off_us - 1

    a.bind(&mut wrap_source);
    a.jmp(JmpCondition::XDecNonZero, &mut wrap_source); // 1µs per iteration; wraps when X==0

    a.assemble_with_wrap(wrap_source, wrap_target)
}

fn poll_usb(
    serial: &mut SerialPort<UsbBus>,
    usb_dev: &mut UsbDevice<UsbBus>,
    usb_rx_buf: &mut Deque<u8, 255>,
) -> usize {
    usb_dev.poll(&mut [serial]);

    let mut total_read = 0;

    while usb_rx_buf.capacity() - usb_rx_buf.len() > 32 {
        let mut temp_buf = [0u8; 32];
        match serial.read(&mut temp_buf) {
            Ok(bytes_read) => {
                info!("RX: {:02x}", &temp_buf[..bytes_read]);
                for byte in &temp_buf[..bytes_read] {
                    usb_rx_buf.push_back(*byte).unwrap();
                }
                total_read += bytes_read;
                break;
            }
            Err(UsbError::WouldBlock) => break,
            Err(e) => {
                error!("USB Read Error: {:?}", defmt::Debug2Format(&e));
                continue;
            }
        };
    }

    total_read
}

/// Helper function to write to serial, handling potential blocking
fn serial_write_buffer(
    serial: &mut SerialPort<'static, UsbBus>,
    data: &[u8],
) -> Result<(), UsbError> {
    let mut sent = 0;
    while sent < data.len() {
        match serial.write(&data[sent..]) {
            Ok(len) => {
                info!("TX: {:02x}", data[sent..sent + len]);
                sent += len;
            }
            Err(UsbError::WouldBlock) => { /* continue */ }
            Err(e) => {
                error!("Serial write error {:?}", defmt::Debug2Format(&e));
                return Err(e);
            }
        }
    }

    while serial.flush() == Err(UsbError::WouldBlock) {
        // busy wait
    }

    Ok(())
}

/// Write a u32 to the PIO TX FIFO, polling USB while waiting for space.
fn pio_tx_write(
    ir_tx: &mut Tx<PIO0SM0>,
    serial: &mut SerialPort<'static, UsbBus>,
    usb_dev: &mut UsbDevice<'static, UsbBus>,
    usb_rx_buf: &mut Deque<u8, 255>,
    value: u32,
) {
    while !ir_tx.write(value) {
        poll_usb(serial, usb_dev, usb_rx_buf);
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
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
    .expect("Clocks failed to initialize");

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    static mut TIMER_PTR: Option<Timer> = None;
    unsafe { TIMER_PTR = Some(timer) };

    defmt::timestamp!("{=u64:us}", {
        unsafe { TIMER_PTR.unwrap().get_counter().ticks() }
    });

    // --- GPIO Setup ---
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin: LedPin = pins.led.into_push_pull_output(); // GP25
    let mut ir_rx_pin: IrRxPin = pins.gpio21.into_pull_up_input();
    led_pin.set_low().ok();

    // Configure GPIO10 as PIO0 output for IR TX
    let ir_tx_pin: Pin<_, FunctionPio0, _> = pins.gpio10.into_function();
    let ir_tx_pin_id = ir_tx_pin.id().num;

    // --- PIO Setup for IR TX ---
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = ir_tx_pio_program();
    let installed = pio.install(&program).unwrap();
    let (mut sm, _rx, mut ir_tx) = PIOBuilder::from_installed_program(installed)
        .set_pins(ir_tx_pin_id, 1)
        .clock_divisor_fixed_point(125, 0) // 125MHz / 125 = 1MHz
        .build(sm0);
    sm.set_pindirs([(ir_tx_pin_id, PinDir::Output)]);
    sm.start();

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
        (*core::ptr::addr_of!(USB_BUS)).as_ref().unwrap()
    };
    let mut serial = SerialPort::new(usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x04d8, 0xfd08))
        .device_class(0x02)
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .manufacturer("flaviutamas.com/p/ir-pico")
            .product("RP2040 Infrared remote transceiver (IR Toy)")
            .serial_number("0000001")])
        .unwrap()
        .build();
    info!("USB device configured");

    let mut device_state = DeviceState::Idle;
    let mut flags = IrToyFlags::default();

    let mut ir_sample_state = IrSampleState::default();

    let mut usb_rx_buf = Deque::<u8, 255>::new();
    loop {
        poll_usb(&mut serial, &mut usb_dev, &mut usb_rx_buf);

        while !usb_rx_buf.is_empty() {
            let cmd = match usb_rx_buf.pop_front().and_then(Command::parse) {
                Some(cmd) => cmd,
                None => {
                    info!("Ignoring invalid command byte");
                    continue;
                }
            };
            info!("CMD: {:?}", defmt::Debug2Format(&cmd));

            match cmd {
                Command::Terminator => {}
                Command::Reset => {
                    led_pin.set_low().ok();
                    flags = IrToyFlags::default();
                    ir_sample_state = IrSampleState::default();
                    device_state = DeviceState::Idle;
                }
                Command::GetVersion => {
                    serial_write_buffer(
                        &mut serial,
                        &[
                            b'V',
                            HARDWARE_VERSION,
                            FIRMWARE_VERSION_H,
                            FIRMWARE_VERSION_L,
                        ],
                    )
                    .unwrap();
                }
                Command::EnableTransmitHandshake => {
                    flags.tx_handshake_enabled = true;
                }
                Command::EnableTransmitByteCount => {
                    flags.tx_byte_count_enabled = true;
                }
                Command::EnableTransmitNotifyOnComplete => {
                    flags.tx_notify_enabled = true;
                }
                Command::SetupTransmitModulation => {
                    // Consume the two parameter bytes; PIO carrier is fixed at ~37kHz
                    let pr2_value = usb_rx_buf.pop_front().unwrap();
                    let duty_cycle_byte = usb_rx_buf.pop_front().unwrap();
                    let (frequency, duty_cycle_percent) =
                        calculate_ir_params(pr2_value, duty_cycle_byte);
                    info!(
                        "Modulation: {} Hz, {}% duty (PIO carrier fixed at ~37kHz)",
                        frequency as u32,
                        (duty_cycle_percent * 100.0) as u32
                    );
                }

                Command::EnterIrSampleMode => {
                    device_state = DeviceState::IrSample;
                    ir_sample_state = IrSampleState::default();
                    led_pin.set_high().ok();
                    serial_write_buffer(&mut serial, IR_SAMPLE_MODE_RESPONSE).unwrap();
                }
                Command::EnterTransmitMode => {
                    if !(flags.tx_handshake_enabled
                        && flags.tx_byte_count_enabled
                        && flags.tx_notify_enabled)
                    {
                        error!(
                            "We require all three flags to be enabled; ignoring EnterTransmitMode"
                        );
                    } else {
                        if !usb_rx_buf.is_empty() {
                            error!("Unexpected data in USB buffer");
                        }

                        run_transmit_mode(
                            &mut serial,
                            &mut usb_dev,
                            &mut led_pin,
                            &mut usb_rx_buf,
                            &mut ir_tx,
                            &mut timer,
                        );
                        info!("Finished transmit mode");
                        // The host expects sample mode to resume automatically after TX,
                        // so leave device_state alone.
                    }
                }

                unsupported => {
                    warn!(
                        "Unsupported command: {:?}",
                        defmt::Debug2Format(&unsupported)
                    );
                }
            }
        }

        if device_state == DeviceState::IrSample {
            ir_sample_state =
                run_ir_sample_mode(&mut serial, &timer, &mut ir_rx_pin, ir_sample_state);
        }
    }
}

#[derive(Debug)]
struct IrSampleState {
    last_edge_time: hal::timer::Instant,
    last_pin_state: bool,
}
impl Default for IrSampleState {
    fn default() -> Self {
        IrSampleState {
            last_edge_time: hal::timer::Instant::from_ticks(0),
            last_pin_state: false,
        }
    }
}

fn run_ir_sample_mode(
    serial: &mut SerialPort<'static, UsbBus>,
    timer: &Timer,
    ir_rx_pin: &mut IrRxPin,
    ir_sample_state: IrSampleState,
) -> IrSampleState {
    let current_pin_state = ir_rx_pin.is_low().unwrap();
    let now = timer.get_counter();

    if ir_sample_state.last_edge_time.ticks() == 0 {
        return if current_pin_state {
            IrSampleState {
                last_edge_time: now,
                last_pin_state: current_pin_state,
            }
        } else {
            IrSampleState::default()
        };
    }

    let ir_count = us_to_irtoy_count(now - ir_sample_state.last_edge_time);

    if current_pin_state != ir_sample_state.last_pin_state || ir_count == 0xFFFF {
        serial_write_buffer(serial, &ir_count.to_be_bytes()).unwrap();
        return if ir_count == 0xFFFF {
            IrSampleState::default()
        } else {
            IrSampleState {
                last_edge_time: now,
                last_pin_state: current_pin_state,
            }
        };
    }

    ir_sample_state
}

fn run_transmit_mode(
    serial: &mut SerialPort<'static, UsbBus>,
    usb_dev: &mut UsbDevice<'static, UsbBus>,
    led_pin: &mut LedPin,
    usb_rx_buf: &mut Deque<u8, 255>,
    ir_tx: &mut Tx<PIO0SM0>,
    timer: &mut Timer,
) {
    led_pin.set_high().unwrap();

    // The original device only supports max 63 bytes per handshake
    const BYTES_PER_HANDSHAKE: u32 = 62;

    let mut bytes_transmitted: u32 = 0;
    let mut handshakes_sent: u32 = 1;

    info!("Sending initial handshake");
    serial_write_buffer(serial, &[BYTES_PER_HANDSHAKE as u8]).unwrap();
    let _ = serial.flush();

    let mut last_sent = timer.get_counter();
    let mut earliest_completion = timer.get_counter();
    let mut is_pulse = true;

    'outer: loop {
        poll_usb(serial, usb_dev, usb_rx_buf);

        if (timer.get_counter() - last_sent).to_millis() > 100 {
            warn!("Timeout waiting for transmit data");
            break 'outer;
        }

        while usb_rx_buf.len() >= 2 {
            let value = (usb_rx_buf.pop_front().unwrap() as u16) << 8
                | (usb_rx_buf.pop_front().unwrap() as u16);
            bytes_transmitted += 2;

            if value == 0xFFFF {
                if !is_pulse {
                    // We already pushed an "on"; push a 1µs dummy "off" to keep PIO state consistent
                    pio_tx_write(ir_tx, serial, usb_dev, usb_rx_buf, 0);
                    let now = timer.get_counter();
                    if earliest_completion < now {
                        earliest_completion = now;
                    }
                    earliest_completion += MicrosDurationU64::micros(1);
                }
                break 'outer;
            }

            let duration_us = irtoy_count_to_us(value);
            last_sent = timer.get_counter();

            let actual_duration_us = if is_pulse {
                let on_cycles = duration_us / PIO_CARRIER_CYCLE_US;
                pio_tx_write(ir_tx, serial, usb_dev, usb_rx_buf, on_cycles);
                on_cycles * PIO_CARRIER_CYCLE_US
            } else {
                let off_x = duration_us.saturating_sub(1);
                pio_tx_write(ir_tx, serial, usb_dev, usb_rx_buf, off_x);
                duration_us
            };

            let now = timer.get_counter();
            if earliest_completion < now {
                earliest_completion = now;
            }
            earliest_completion += MicrosDurationU64::micros(actual_duration_us as u64);

            is_pulse = !is_pulse;
        }

        while bytes_transmitted >= handshakes_sent * BYTES_PER_HANDSHAKE {
            info!("Sending new handshake");
            handshakes_sent += 1;
            serial_write_buffer(serial, &[BYTES_PER_HANDSHAKE as u8]).unwrap();
        }
    }

    // PIO FIFO empty does not imply the SM has finished the last symbol;
    // a long trailing space can still be running. Reporting completion early
    // lets the host overlap the next transmission with this one.
    while timer.get_counter() < earliest_completion {
        poll_usb(serial, usb_dev, usb_rx_buf);
    }

    // number of bytes transmitted: t|high|low
    serial_write_buffer(
        serial,
        &[
            b't',
            (bytes_transmitted >> 8) as u8,
            bytes_transmitted as u8,
        ],
    )
    .unwrap();
    serial_write_buffer(serial, b"C").unwrap();

    // LED stays on: the device returns to sample mode, where the LED is on.
}
