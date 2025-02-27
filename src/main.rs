#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use heapless::consts::*;
use heapless::Vec;

// Entry point
use bsp::entry;
// Logging support
use defmt::*;
use defmt_rtt as _;

use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_nb::serial::Read;

// Panic handler
use panic_probe as _;
use rp_pico::{
    self as bsp, // alias for the BSP
    hal::rtc::DateTime,
    hal::timer::Alarm,
    hal::timer::Alarm0,
    hal::timer::Timer,
};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock}, // Clocks
    pac,                                   // Peripherals
    pac::interrupt,                        // interrupt macro
    sio::Sio,                              // Peripherals
    watchdog::Watchdog,                    // Peripherals
};

use core::cell::RefCell;

use bsp::hal::gpio::bank0;
use bsp::hal::gpio::{
    FunctionSioInput, FunctionSioOutput, FunctionUart, Interrupt::EdgeLow, Pin, PinState, PullDown,
    PullNone, PullUp,
};
use bsp::hal::uart;
use bsp::hal::uart::{DataBits, StopBits, UartConfig};
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use fugit::{ExtU32, MicrosDurationU32, RateExtU32};
use once_cell::sync::Lazy;

#[derive(Copy, Clone, defmt::Format)]
enum DisplayMode {
    LongHigh,
    LongLow,
    LatHigh,
    LatLow,
    Time,
    Date,
    Year,
}

static DISPLAY_MODE: Lazy<Mutex<RefCell<DisplayMode>>> =
    Lazy::new(|| Mutex::new(RefCell::new(DisplayMode::Time)));

type UartPins = (
    Pin<bank0::Gpio0, FunctionUart, PullNone>,
    Pin<bank0::Gpio1, FunctionUart, PullNone>,
);

type Uart = uart::UartPeripheral<uart::Enabled, pac::UART0, UartPins>;

static TIMER_DURATION: Lazy<MicrosDurationU32> = Lazy::new(|| 100_u32.millis());

struct Machine {
    pub delay: Option<Delay>,
    pub alarm_0: Alarm0,
    pub led_system: Pin<bank0::Gpio25, FunctionSioOutput, PullDown>,
    pub led_user1: Pin<bank0::Gpio16, FunctionSioOutput, PullDown>,
    pub led_user2: Pin<bank0::Gpio17, FunctionSioOutput, PullDown>,
    pub button_1: Pin<bank0::Gpio3, FunctionSioInput, PullUp>,
    pub button_2: Pin<bank0::Gpio2, FunctionSioInput, PullUp>,
    pub button_3: Pin<bank0::Gpio4, FunctionSioInput, PullUp>,
    pub uart: Uart,
    #[allow(dead_code)]
    pub srclk_reset: Pin<bank0::Gpio23, FunctionSioOutput, PullDown>,
    pub srclk: Pin<bank0::Gpio24, FunctionSioOutput, PullDown>,
    pub rclk: Pin<bank0::Gpio26, FunctionSioOutput, PullDown>,
    pub ser1: Pin<bank0::Gpio27, FunctionSioOutput, PullDown>,
    pub ser2: Pin<bank0::Gpio28, FunctionSioOutput, PullDown>,
    pub ser3: Pin<bank0::Gpio7, FunctionSioOutput, PullDown>,
    pub ser4: Pin<bank0::Gpio8, FunctionSioOutput, PullDown>,
    pub ser5: Pin<bank0::Gpio9, FunctionSioOutput, PullDown>,
}

trait Display {
    fn shift(&mut self, data: &[u8]);
    fn disp(&mut self);
}

impl Display for Machine {
    fn shift(&mut self, data: &[u8]) {
        {
            let sers = self.ser_pins_as_trait();

            if sers.len() != data.len() {
                core::panic!("Data length does not match number of shift registers");
            }
        }

        for bit_index in 0..8 {
            {
                let mut sers = self.ser_pins_as_trait();
                for (ser, byte) in sers.iter_mut().zip(data.iter()) {
                    let bit = (*byte >> bit_index) & 1;
                    let state = if bit == 1 {
                        PinState::High
                    } else {
                        PinState::Low
                    };
                    ser.set_state(state).unwrap();
                }
            }
            self.blink_srclk();
        }
    }

    fn disp(&mut self) {
        self.blink_rclk()
    }
}

struct UartData {
    data: Vec<u8, U256>,
}

struct GpsData {
    pub latitude: f32,
    pub longitude: f32,
    pub utc_datetime: DateTime,
    pub status: bool,
}

pub trait DateTimeExt {
    fn add_1024_weeks(&mut self);
}

impl DateTimeExt for DateTime {
    fn add_1024_weeks(&mut self) {
        let days_to_add = 1024 * 7;
        for _ in 0..days_to_add {
            self.day += 1;
            if self.day > days_in_month(self.year, self.month) {
                self.day = 1;
                self.month += 1;
                if self.month > 12 {
                    self.month = 1;
                    self.year += 1;
                }
            }
        }
    }
}

fn days_in_month(year: u16, month: u8) -> u8 {
    match month {
        4 | 6 | 9 | 11 => 30,
        2 => {
            // Very simple leap check
            if (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0) {
                29
            } else {
                28
            }
        }
        _ => 31,
    }
}

fn convert_number_to_bits(number: u8) -> u8 {
    match number {
        0 => 0b01111011,
        1 => 0b01100000,
        2 => 0b01010111,
        3 => 0b01110110,
        4 => 0b01101100,
        5 => 0b00111110,
        6 => 0b00111111,
        7 => 0b01110000,
        8 => 0b01111111,
        9 => 0b01111110,
        _ => 0,
    }
}

impl GpsData {
    pub fn new() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            utc_datetime: bsp::hal::rtc::DateTime {
                year: 2025,
                month: 1,
                day: 1,
                day_of_week: bsp::hal::rtc::DayOfWeek::Wednesday,
                hour: 0,
                minute: 0,
                second: 0,
            },
            status: false,
        }
    }

    pub fn init_settings(uart: &mut Uart) {
        // send b"PSRF106，21*0F"
        uart.write_full_blocking(b"$PSRF106,21*0F\r\n");
    }
    pub fn parse_data(&mut self, data: &[u8]) {
        // let gps = GpsData::new();
        let mut iter = data.split(|&x| x == b',');
        let data = iter.next().unwrap();

        if core::str::from_utf8(data).unwrap() != "$GPRMC" {
            return;
        }

        let utc = iter.next().unwrap();
        self.utc_datetime.hour = core::str::from_utf8(&utc[0..2])
            .unwrap()
            .parse::<u8>()
            .unwrap();
        self.utc_datetime.minute = core::str::from_utf8(&utc[2..4])
            .unwrap()
            .parse::<u8>()
            .unwrap();
        self.utc_datetime.second = core::str::from_utf8(&utc[4..6])
            .unwrap()
            .parse::<u8>()
            .unwrap();

        debug!("hour: {:?}", self.utc_datetime.hour);
        debug!("minute: {:?}", self.utc_datetime.minute);
        debug!("second: {:?}", self.utc_datetime.second);

        let status = iter.next().unwrap();

        self.status = core::str::from_utf8(status).unwrap() == "A";

        let latitude_abs = iter.next().unwrap();
        let latitude_abs = core::str::from_utf8(latitude_abs)
            .unwrap()
            .parse::<f32>()
            .unwrap()
            / 100.0;
        let north_or_south = iter.next().unwrap();
        if core::str::from_utf8(north_or_south).unwrap() == "S" {
            self.latitude = -latitude_abs;
        } else if core::str::from_utf8(north_or_south).unwrap() == "N" {
            self.latitude = latitude_abs;
        } else {
            core::panic!("Invalid latitude direction");
        }
        let longitude_abs = iter.next().unwrap();
        let longitude_abs = core::str::from_utf8(longitude_abs)
            .unwrap()
            .parse::<f32>()
            .unwrap()
            / 100.0;
        let east_or_west = iter.next().unwrap();
        if core::str::from_utf8(east_or_west).unwrap() == "W" {
            self.longitude = -longitude_abs;
        } else if core::str::from_utf8(east_or_west).unwrap() == "E" {
            self.longitude = longitude_abs;
        } else {
            core::panic!("Invalid longitude direction");
        }

        info!("latitude: {:?}", self.latitude);
        info!("longitude: {:?}", self.longitude);

        let _ = iter.next();
        let _ = iter.next();
        let date = iter.next().unwrap();
        self.utc_datetime.day = core::str::from_utf8(&date[0..2])
            .unwrap()
            .parse::<u8>()
            .unwrap();
        self.utc_datetime.month = core::str::from_utf8(&date[2..4])
            .unwrap()
            .parse::<u8>()
            .unwrap();
        self.utc_datetime.year = core::str::from_utf8(&date[4..6])
            .unwrap()
            .parse::<u16>()
            .unwrap()
            + 2000;
        self.utc_datetime.add_1024_weeks();
        debug!("day: {:?}", self.utc_datetime.day);
        debug!("month: {:?}", self.utc_datetime.month);
        debug!("year: {:?}", self.utc_datetime.year);
    }
}

impl UartData {
    pub fn new() -> Self {
        Self { data: Vec::new() }
    }
    pub fn push(&mut self, byte: u8) {
        self.data.push(byte).unwrap();
    }
    pub fn clear(&mut self) {
        self.data.clear();
    }
}

impl Machine {
    fn reset_timer(&mut self) {
        self.alarm_0.enable_interrupt();
        if self.alarm_0.schedule(*TIMER_DURATION).is_err() {
            warn!("Error while initializing timer");
        }
    }
    fn enable_uart(&mut self) {
        self.uart.enable_rx_interrupt();
    }

    pub fn ser_pins_as_trait(
        &mut self,
    ) -> [&mut dyn OutputPin<Error = core::convert::Infallible>; 5] {
        [
            &mut self.ser1,
            &mut self.ser2,
            &mut self.ser3,
            &mut self.ser4,
            &mut self.ser5,
        ]
    }

    pub fn blink_srclk(&mut self) {
        self.srclk.set_state(PinState::High).unwrap();
        self.srclk.set_state(PinState::Low).unwrap();
    }
    pub fn blink_rclk(&mut self) {
        self.rclk.set_state(PinState::High).unwrap();

        self.rclk.set_state(PinState::Low).unwrap();
    }
}

static MACHINE: Lazy<Mutex<RefCell<Machine>>> = Lazy::new(|| {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let delay = Some(cortex_m::delay::Delay::new(
        core.SYST,
        clocks.system_clock.freq().to_Hz(),
    ));

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm_0 = timer.alarm_0().unwrap();
    alarm_0.enable_interrupt();
    if alarm_0.schedule(*TIMER_DURATION).is_err() {
        warn!("Error while initializing timer");
    }

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (pins.gpio0.reconfigure(), pins.gpio1.reconfigure());

    let uart = uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let led_system = pins.led.into_push_pull_output();
    let led_user1 = pins.gpio16.into_push_pull_output();
    let led_user2 = pins.gpio17.into_push_pull_output();
    let mut button_1 = pins.gpio3.into_pull_up_input();
    let mut button_2 = pins.gpio2.into_pull_up_input();
    let mut button_3 = pins.gpio4.into_pull_up_input();
    let srclk_reset = pins
        .b_power_save
        .into_push_pull_output_in_state(PinState::High);
    let srclk = pins.vbus_detect.into_push_pull_output();
    let rclk = pins.gpio26.into_push_pull_output();
    let ser1 = pins.gpio27.into_push_pull_output();
    let ser2 = pins.gpio28.into_push_pull_output();
    let ser3 = pins.gpio7.into_push_pull_output();
    let ser4 = pins.gpio8.into_push_pull_output();
    let ser5 = pins.gpio9.into_push_pull_output();

    button_1.set_interrupt_enabled(EdgeLow, true);
    button_2.set_interrupt_enabled(EdgeLow, true);
    button_3.set_interrupt_enabled(EdgeLow, true);
    button_1.clear_interrupt(EdgeLow);
    button_2.clear_interrupt(EdgeLow);
    button_3.clear_interrupt(EdgeLow);

    let mut m = Machine {
        delay,
        led_system,
        led_user1,
        led_user2,
        button_1,
        button_2,
        button_3,
        alarm_0,
        uart,
        srclk_reset,
        srclk,
        rclk,
        ser1,
        ser2,
        ser3,
        ser4,
        ser5,
    };
    m.reset_timer();
    m.enable_uart();
    Mutex::new(RefCell::new(m))
});

static GPS_DATA: Lazy<Mutex<RefCell<GpsData>>> = Lazy::new(|| {
    let gps = GpsData::new();
    Mutex::new(RefCell::new(gps))
});

static UART_DATA: Lazy<Mutex<RefCell<UartData>>> = Lazy::new(|| {
    let uart = UartData::new();
    Mutex::new(RefCell::new(uart))
});

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut delay =
        cortex_m::interrupt::free(|cs| MACHINE.borrow(cs).borrow_mut().delay.take().unwrap());

    delay.delay_ms(100);

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        pac::NVIC::unmask(pac::Interrupt::UART0_IRQ);
    }

    delay.delay_ms(1000);

    cortex_m::interrupt::free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        GpsData::init_settings(&mut m.uart);
    });

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        m.alarm_0.clear_interrupt();
        if let Err(e) = m.led_system.toggle() {
            warn!("Error while interrupt: {:?}", e);
        }

        let digits;
        let mode = *DISPLAY_MODE.borrow(cs).borrow();
        let gps = GPS_DATA.borrow(cs).borrow();
        match mode {
            DisplayMode::LongHigh => {
                let longitude_degree = gps.longitude as u8;
                let longitude_for_google = (longitude_degree as f32)
                    + (gps.longitude - (longitude_degree as f32)) * 100.0 / 60.0;
                let long_high = (longitude_for_google * 100.0) as u16;
                digits = [
                    convert_number_to_bits((long_high / 10000) as u8),
                    convert_number_to_bits(((long_high / 1000) % 10) as u8),
                    convert_number_to_bits(((long_high / 100) % 10) as u8),
                    convert_number_to_bits(((long_high / 10) % 10) as u8),
                    convert_number_to_bits((long_high % 10) as u8),
                ];
            }
            DisplayMode::LongLow => {
                let longitude_degree = gps.longitude as u8;
                let longitude_for_google = (longitude_degree as f32)
                    + (gps.longitude - (longitude_degree as f32)) * 100.0 / 60.0;
                let long_high = (longitude_for_google * 100.0) as u16;
                let long_low = (((longitude_for_google * 100.0) - (long_high as f32)) * 1e5) as u16;
                digits = [
                    convert_number_to_bits((long_low / 10000) as u8),
                    convert_number_to_bits(((long_low / 1000) % 10) as u8),
                    convert_number_to_bits(((long_low / 100) % 10) as u8),
                    convert_number_to_bits(((long_low / 10) % 10) as u8),
                    convert_number_to_bits((long_low % 10) as u8),
                ];
            }
            DisplayMode::LatHigh => {
                let latitude_degree = gps.latitude as u8;
                let latitude_for_google = (latitude_degree as f32)
                    + (gps.latitude - (latitude_degree as f32)) * 100.0 / 60.0;
                let lat_high = (latitude_for_google * 100.0) as u16;
                digits = [
                    convert_number_to_bits((lat_high / 10000) as u8),
                    convert_number_to_bits(((lat_high / 1000) % 10) as u8),
                    convert_number_to_bits(((lat_high / 100) % 10) as u8),
                    convert_number_to_bits(((lat_high / 10) % 10) as u8),
                    convert_number_to_bits((lat_high % 10) as u8),
                ];
            }
            DisplayMode::LatLow => {
                let latitude_degree = gps.latitude as u8;
                let latitude_for_google = (latitude_degree as f32)
                    + (gps.latitude - (latitude_degree as f32)) * 100.0 / 60.0;
                let lat_high = (latitude_for_google * 100.0) as u16;
                let lat_low = (((latitude_for_google * 100.0) - (lat_high as f32)) * 1e5) as u16;
                digits = [
                    convert_number_to_bits((lat_low / 10000) as u8),
                    convert_number_to_bits(((lat_low / 1000) % 10) as u8),
                    convert_number_to_bits(((lat_low / 100) % 10) as u8),
                    convert_number_to_bits(((lat_low / 10) % 10) as u8),
                    convert_number_to_bits((lat_low % 10) as u8),
                ];
            }
            DisplayMode::Time => {
                let hour = gps.utc_datetime.hour;
                let minute = gps.utc_datetime.minute;
                digits = [
                    0,
                    convert_number_to_bits(hour / 10),
                    convert_number_to_bits(hour % 10),
                    convert_number_to_bits(minute / 10),
                    convert_number_to_bits(minute % 10),
                ];
            }

            DisplayMode::Date => {
                let day = gps.utc_datetime.day;
                let month = gps.utc_datetime.month;
                digits = [
                    0,
                    convert_number_to_bits(day / 10),
                    convert_number_to_bits(day % 10),
                    convert_number_to_bits(month / 10),
                    convert_number_to_bits(month % 10),
                ];
            }
            DisplayMode::Year => {
                // Display year as 4 digits with blank in the middle.
                let year = gps.utc_datetime.year;
                let first_two = year / 100;
                let last_two = year % 100;
                let d1: u8 = (first_two / 10) as u8;
                let d2: u8 = (first_two % 10) as u8;
                let d3: u8 = (last_two / 10) as u8;
                let d4: u8 = (last_two % 10) as u8;
                digits = [
                    0,
                    convert_number_to_bits(d1),
                    convert_number_to_bits(d2),
                    convert_number_to_bits(d3),
                    convert_number_to_bits(d4),
                ];
            }
        }

        m.shift(&digits);
        m.disp();
        m.reset_timer();
    });
}

#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        if m.button_1.interrupt_status(EdgeLow) {
            cortex_m::interrupt::free(|cs2| {
                let current = *DISPLAY_MODE.borrow(cs2).borrow();
                let new_mode = match current {
                    DisplayMode::LongHigh => DisplayMode::LongLow,
                    _ => DisplayMode::LongHigh,
                };
                *DISPLAY_MODE.borrow(cs2).borrow_mut() = new_mode;
            });
            // let _ = m.led_user1.toggle();
            m.button_1.clear_interrupt(EdgeLow);
        }
        if m.button_2.interrupt_status(EdgeLow) {
            cortex_m::interrupt::free(|cs2| {
                let current = *DISPLAY_MODE.borrow(cs2).borrow();
                let new_mode = match current {
                    DisplayMode::LatHigh => DisplayMode::LatLow,
                    _ => DisplayMode::LatHigh,
                };
                *DISPLAY_MODE.borrow(cs2).borrow_mut() = new_mode;
            });
            // let _ = m.led_user2.toggle();
            m.button_2.clear_interrupt(EdgeLow);
        }
        if m.button_3.interrupt_status(EdgeLow) {
            // Toggle between Time and Year modes.
            cortex_m::interrupt::free(|cs2| {
                let current = *DISPLAY_MODE.borrow(cs2).borrow();
                let new_mode = match current {
                    DisplayMode::Time => DisplayMode::Date,
                    DisplayMode::Date => DisplayMode::Year,
                    _ => DisplayMode::Time,
                };
                *DISPLAY_MODE.borrow(cs2).borrow_mut() = new_mode;
            });
            m.button_3.clear_interrupt(EdgeLow);
        }
    });
}

#[interrupt]
fn UART0_IRQ() {
    cortex_m::interrupt::free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        let mut uart_data = UART_DATA.borrow(cs).borrow_mut();
        let mut gps = GPS_DATA.borrow(cs).borrow_mut();
        while let Ok(byte) = m.uart.read() {
            if byte == b'\n' || byte == b'\r' {
                if !uart_data.data.is_empty() {
                    gps.parse_data(&uart_data.data);
                }
                uart_data.clear();
                continue;
            }
            uart_data.push(byte);
        }
    });

    // Set an event to ensure the main thread always wakes up, even if it's in
    // the process of going to sleep.
    cortex_m::asm::sev();
}
// End of file
