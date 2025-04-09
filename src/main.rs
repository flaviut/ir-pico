#![no_std]
#![no_main]

mod util;

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicUsize, Ordering};
use rp_pico as bsp;

use bsp::entry;
use defmt::{error, info, warn};
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
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

use fugit::{HertzU32, MicrosDurationU64};
use heapless::spsc::Queue;
use heapless::{spsc, Deque};
use mylib::{calculate_ir_params, Command};
use rp_pico::hal;
use rp_pico::hal::gpio::PullUp;
use rp_pico::hal::multicore::{Multicore, Stack};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

// --- Configuration ---
const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // 12 MHz

// Responses / Identifiers
const HARDWARE_VERSION: u8 = 0x30;
const FIRMWARE_VERSION_H: u8 = b'2';
const FIRMWARE_VERSION_L: u8 = b'1';
const IR_SAMPLE_MODE_RESPONSE: &[u8] = b"S01";

type IrRxPin = Pin<hal::gpio::bank0::Gpio21, FunctionSio<SioInput>, PullUp>;
type IrTxPin = Pin<hal::gpio::bank0::Gpio10, FunctionSio<SioOutput>, PullDown>;
/// Onboard LED
type LedPin = Pin<hal::gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullDown>;

// --- State ---
#[derive(PartialEq, Eq, Clone, Copy, defmt::Format)]
enum DeviceState {
    Idle,
    IrSample, // Receiving IR
    Transmit, // Transmitting IR
}

struct IrToyFlags {
    tx_handshake_enabled: bool,
    tx_byte_count_enabled: bool,
    tx_notify_enabled: bool,
}

impl Default for IrToyFlags {
    fn default() -> Self {
        IrToyFlags {
            tx_handshake_enabled: false,
            tx_byte_count_enabled: false,
            tx_notify_enabled: false,
        }
    }
}

// Helper to convert microseconds duration to IRToy 16-bit count
fn us_to_irtoy_count(us: MicrosDurationU64) -> u16 {
    // Using integer math to avoid float math: count = us * 10000 / 213333
    let count = (us.to_micros() * 10000) / 213333;
    if count > u16::MAX as u64 {
        u16::MAX
    } else {
        count as u16
    }
}

// Helper to convert IRToy 16-bit count to microseconds duration
fn irtoy_count_to_us(count: u16) -> u16 {
    // us = count * 21.3333 = count * 213333 / 10000
    ((count as u64) * 213333 / 10000) as u16
}

#[derive(Debug)]
enum TransmitCommand {
    On(u16),
    Off(u16),
    SetDutyTimings(u8, u8),
}

type TransmitQueue = Queue<TransmitCommand, 1024>;

fn core1_task(
    mut ir_tx_pin: IrTxPin,
    sys_freq: HertzU32,
    mut command_queue: spsc::Consumer<TransmitCommand, 1024>,
) {
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut delay = Delay::new(core.SYST, sys_freq.to_Hz());

    let (mut up_time, mut down_time) = {
        let (up_time, down_time) = mylib::calculate_timings_us(38e3, 0.5);
        (up_time as u32, down_time as u32)
    };

    loop {
        let command = loop {
            match command_queue.dequeue() {
                Some(command) => break command,
                None => { /* busy wait */ }
            }
        };

        match command {
            TransmitCommand::SetDutyTimings(up, down) => {
                (up_time, down_time) = (up as u32, down as u32);
            }
            TransmitCommand::On(duration_us) => {
                let cycles = (duration_us as u32) / (up_time + down_time);
                for _ in 0..cycles {
                    ir_tx_pin.set_high().ok();
                    delay.delay_us(up_time);
                    ir_tx_pin.set_low().ok();
                    delay.delay_us(down_time);
                }
            }
            TransmitCommand::Off(duration_us) => {
                ir_tx_pin.set_low().ok();
                delay.delay_us(duration_us as u32);
            }
        }
    }
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
                for i in 0..bytes_read {
                    usb_rx_buf.push_back(temp_buf[i]).unwrap();
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
                info!("TX: {:02x}", data[sent..len]);
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

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

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
    .expect("Clocks failed to initialize");

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    static mut TIMER_PTR: Option<Timer> = None;
    unsafe { TIMER_PTR = Some(timer.clone()) };

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
    let mut ir_tx_pin: IrTxPin = pins.gpio10.into_push_pull_output();
    ir_tx_pin.set_low().ok(); // Ensure the transmitter is off initially

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
    let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x04d8, 0xfd08))
        .device_class(0x02)
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .manufacturer("flaviutamas.com/p/ir-pico")
            .product("RP2040 Infrared remote transceiver (IR Toy)")
            .serial_number("0000001")])
        .unwrap()
        .build();
    info!("USB device configured");

    static mut COMMAND_QUEUE_BUFFER: MaybeUninit<TransmitQueue> = MaybeUninit::uninit();
    let command_queue = unsafe {
        COMMAND_QUEUE_BUFFER.write(Queue::new());
        &mut *COMMAND_QUEUE_BUFFER.as_mut_ptr()
    };
    let (mut command_producer, command_consumer) = command_queue.split();

    static mut CORE1_STACK: Stack<4096> = Stack::new();
    // Other init code above this line
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            core1_task(ir_tx_pin, clocks.system_clock.freq(), command_consumer)
        })
        .unwrap();

    let mut device_state = DeviceState::Idle;
    let mut flags = IrToyFlags::default();

    let mut ir_sample_state = IrSampleState::default();

    let mut usb_rx_buf = Deque::<u8, 255>::new();
    loop {
        poll_usb(&mut serial, &mut usb_dev, &mut usb_rx_buf);

        while !usb_rx_buf.is_empty() {
            let cmd = match usb_rx_buf.pop_front().map(|b| Command::parse(b)).flatten() {
                Some(cmd) => cmd,
                None => {
                    // If we get here, we've received a byte that isn't a valid command.
                    // This may be a mismatch between the state of the host and the device.
                    // However, we can't do anything about it except ignore it and hope that the
                    // host realizes it needs to reset us.
                    info!("Ignoring invalid command byte");
                    continue;
                }
            };
            info!("CMD: {:?}", defmt::Debug2Format(&cmd));

            match cmd {
                Command::Terminator => {
                    // This is the terminator byte for when we're in transmit mode, and we may
                    // receive it at any time as part of the reset process.
                    // We don't need to do anything.
                }
                Command::Reset => {
                    led_pin.set_low().ok(); // LED off during reset
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
                    let pr2_value = usb_rx_buf.pop_front().unwrap();
                    let duty_cycle_byte = usb_rx_buf.pop_front().unwrap();
                    let (frequency, duty_cycle_percent) =
                        calculate_ir_params(pr2_value, duty_cycle_byte);
                    info!(
                        "Calculated frequency: {} Hz, Duty Cycle: {}%",
                        frequency as u32,
                        (duty_cycle_percent * 100.0) as u32
                    );
                    let (up_time, down_time) =
                        mylib::calculate_timings_us(frequency, duty_cycle_percent);
                    command_producer
                        .enqueue(TransmitCommand::SetDutyTimings(up_time, down_time))
                        .unwrap();
                }

                Command::EnterIrSampleMode => {
                    device_state = DeviceState::IrSample;
                    ir_sample_state = IrSampleState::default();
                    led_pin.set_high().ok(); // LED on during IrSample mode
                    timer.delay_ms(1);
                    serial_write_buffer(&mut serial, IR_SAMPLE_MODE_RESPONSE).unwrap();
                }
                Command::EnterTransmitMode => {
                    // the transmit mode will take over the main loop while it is active
                    device_state = DeviceState::Transmit;
                    if !(flags.tx_handshake_enabled
                        && flags.tx_byte_count_enabled
                        && flags.tx_notify_enabled)
                    {
                        error!("We require all three flags to be enabled");
                    }
                    if usb_rx_buf.len() > 0 {
                        // we only support handshake mode, during which we expect no data
                        // in the USB buffer until we send the handshake response
                        error!("Unexpected data in USB buffer");
                    }

                    run_transmit_mode(
                        &mut serial,
                        &mut usb_dev,
                        &mut led_pin,
                        &mut usb_rx_buf,
                        &mut command_producer,
                        &mut timer,
                    );
                    info!("Finished transmit mode");
                    device_state = DeviceState::Idle;
                }

                unsupported => {
                    warn!(
                        "Unsupported command: {:?}",
                        defmt::Debug2Format(&unsupported)
                    );
                }
            }
        }

        if device_state == DeviceState::Transmit {
        } else if device_state == DeviceState::IrSample {
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
    // is_low because the receiver pulls the pin low when it detects a pulse
    let current_pin_state = ir_rx_pin.is_low().unwrap();
    let now = timer.get_counter();

    if ir_sample_state.last_edge_time.ticks() == 0 {
        return if current_pin_state {
            // first time we see a pulse
            IrSampleState {
                last_edge_time: now,
                last_pin_state: current_pin_state,
            }
        } else {
            // we haven't seen a pulse yet
            IrSampleState::default()
        };
    }

    let ir_count = us_to_irtoy_count(now - ir_sample_state.last_edge_time);

    if current_pin_state != ir_sample_state.last_pin_state || ir_count == 0xFFFF {
        serial_write_buffer(serial, &ir_count.to_be_bytes()).unwrap();
        return if ir_count == 0xFFFF {
            // timeout! resetting.
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
    producer: &mut spsc::Producer<'static, TransmitCommand, 1024>,
    timer: &mut Timer,
) -> () {
    led_pin.set_high().unwrap();

    let mut bytes_transmitted: u32 = 0;

    fn get_remaining_bytes(producer: &spsc::Producer<'static, TransmitCommand, 1024>) -> usize {
        // the original device only supports a max of 63 bytes, and some drivers (Linux...) will
        // fail to understand that we have capacity for more than 63 bytes
        62.min(producer.capacity() * 2)
    }

    info!("Sending initial handshake");
    let mut bytes_expected = get_remaining_bytes(producer);
    serial_write_buffer(serial, &[bytes_expected as u8]).unwrap();
    let _ = serial.flush();

    let mut last_sent = timer.get_counter();

    let mut is_pulse = true;
    'outer: loop {
        let bytes_read = poll_usb(serial, usb_dev, usb_rx_buf);
        if bytes_read > bytes_expected {
            panic!("Got more bytes ({}) than expected ({}) ", bytes_read, bytes_expected);
        }
        bytes_expected -= bytes_read;

        if (timer.get_counter() - last_sent).to_millis() > 100 {
            warn!("Timeout waiting for transmit data");
            break 'outer;
        }

        while usb_rx_buf.len() >= 2 {
            let value = (usb_rx_buf.pop_front().unwrap() as u16) << 8
                | (usb_rx_buf.pop_front().unwrap() as u16);
            bytes_transmitted += 2;
            if value == 0xFFFF {
                // wait for the transmitter to fully drain the queue
                while producer.len() > 0 {
                    let bytes_read = poll_usb(serial, usb_dev, usb_rx_buf);
                    if bytes_read > 0 {
                        panic!("Got more bytes ({}) when transmit is complete", bytes_read);
                    }
                }
                break 'outer;
            }

            let duration_us = irtoy_count_to_us(value);

            last_sent = timer.get_counter();
            if is_pulse {
                producer.enqueue(TransmitCommand::On(duration_us)).unwrap();
            } else {
                producer.enqueue(TransmitCommand::Off(duration_us)).unwrap();
            }
            is_pulse = !is_pulse;
        }

        if bytes_expected == 0 {
            info!("Finished sending everything! New handshake");
            // finished sending everything! new handshake so we get more data
            bytes_expected = get_remaining_bytes(producer);
            serial_write_buffer(serial, &[bytes_expected as u8]).unwrap();
        }
    }

    // spec requires a final handshake even though we expect no data
    bytes_expected = get_remaining_bytes(producer);
    serial_write_buffer(serial, &[bytes_expected as u8]).unwrap();

    // number of bytes transmitted in the format t|high-8bits|low-8bits
    serial_write_buffer(
        serial,
        &[
            't' as u8,
            (bytes_transmitted >> 8) as u8,
            bytes_transmitted as u8,
        ],
    )
    .unwrap();
    // always success. might be possible to fail but in practice I expect this device is so much
    // faster than the original implementation that buffer underruns are not likely
    serial_write_buffer(serial, &['C' as u8]).unwrap();

    led_pin.set_low().unwrap();
}
