#![no_std]
#![no_main]

// Entry point
use bsp::entry;
// Logging support
use defmt::*;
use defmt_rtt as _;

use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_nb::serial::{Read, Write};

// Panic handler
use panic_probe as _;

use rp_pico::{
    self as bsp, // alias for the BSP
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
    FunctionSioInput, FunctionSioOutput, FunctionUart, Interrupt::EdgeLow, Pin, PullDown, PullNone,
    PullUp,
};
use bsp::hal::uart;
use bsp::hal::uart::{DataBits, StopBits, UartConfig};
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use fugit::{ExtU32, MicrosDurationU32, RateExtU32};
use once_cell::sync::Lazy;

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

        self.uart
            .write_full_blocking(b"uart_interrupt example started...\n");
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
    let button_1 = pins.gpio3.into_pull_up_input();
    let button_2 = pins.gpio2.into_pull_up_input();
    let button_3 = pins.gpio4.into_pull_up_input();

    button_1.set_interrupt_enabled(EdgeLow, true);
    button_2.set_interrupt_enabled(EdgeLow, true);
    button_3.set_interrupt_enabled(EdgeLow, true);

    let mut m = Machine {
        delay: delay,
        led_system: led_system,
        led_user1: led_user1,
        led_user2: led_user2,
        button_1: button_1,
        button_2: button_2,
        button_3: button_3,
        alarm_0: alarm_0,
        uart: uart,
    };
    m.reset_timer();
    m.enable_uart();
    Mutex::new(RefCell::new(m))
});

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut delay =
        cortex_m::interrupt::free(|cs| MACHINE.borrow(cs).borrow_mut().delay.take().unwrap());
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        pac::NVIC::unmask(pac::Interrupt::UART0_IRQ);
    }

    delay.delay_ms(100);

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
        m.reset_timer();
    });
}

#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        if m.button_1.interrupt_status(EdgeLow) {
            info!("Button1 pressed");
            let _ = m.led_user1.toggle();
            m.button_1.clear_interrupt(EdgeLow);
        }
        if m.button_2.interrupt_status(EdgeLow) {
            info!("Button2 pressed");
            let _ = m.led_user2.toggle();
            m.button_2.clear_interrupt(EdgeLow);
        }
        if m.button_3.interrupt_status(EdgeLow) {
            info!("Button3 pressed");
            m.button_3.clear_interrupt(EdgeLow);
        }
    });
}

#[interrupt]
fn UART0_IRQ() {
    cortex_m::interrupt::free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        while let Ok(byte) = m.uart.read() {
            let _ = m.uart.write(byte);
        }
    });

    // Set an event to ensure the main thread always wakes up, even if it's in
    // the process of going to sleep.
    cortex_m::asm::sev();
}
// End of file
