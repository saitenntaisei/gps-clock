#![no_std]
#![no_main]

// Entry point
use bsp::entry;
// Logging support
use defmt::*;
use defmt_rtt as _;

use embedded_hal::digital::StatefulOutputPin;
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
    FunctionSioInput, FunctionSioOutput, Interrupt::EdgeLow, Pin, PullDown, PullUp,
};
use cortex_m::delay::Delay;
use cortex_m::interrupt::Mutex;
use fugit::{ExtU32, MicrosDurationU32};
use once_cell::sync::Lazy;

static TIMER_DURATION: Lazy<MicrosDurationU32> = Lazy::new(|| 100_u32.millis());

struct Machine {
    pub delay: Option<Delay>,
    pub alarm_0: Alarm0,
    pub led_pin: Pin<bank0::Gpio25, FunctionSioOutput, PullDown>,
    pub button_1: Pin<bank0::Gpio3, FunctionSioInput, PullUp>,
    pub button_2: Pin<bank0::Gpio2, FunctionSioInput, PullUp>,
    pub button_3: Pin<bank0::Gpio4, FunctionSioInput, PullUp>,
}

impl Machine {
    fn reset_timer(&mut self) {
        self.alarm_0.enable_interrupt();
        if self.alarm_0.schedule(*TIMER_DURATION).is_err() {
            warn!("Error while initializing timer");
        }
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

    let led_pin = pins.led.into_push_pull_output();
    let button_1 = pins.gpio3.into_pull_up_input();
    let button_2 = pins.gpio2.into_pull_up_input();
    let button_3 = pins.gpio4.into_pull_up_input();

    button_1.set_interrupt_enabled(EdgeLow, true);
    button_2.set_interrupt_enabled(EdgeLow, true);
    button_3.set_interrupt_enabled(EdgeLow, true);

    let mut m = Machine {
        delay: delay,
        led_pin: led_pin,
        button_1: button_1,
        button_2: button_2,
        button_3: button_3,
        alarm_0: alarm_0,
    };
    m.reset_timer();
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
        if let Err(e) = m.led_pin.toggle() {
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
            m.button_1.clear_interrupt(EdgeLow);
        }
        if m.button_2.interrupt_status(EdgeLow) {
            info!("Button2 pressed");
            m.button_2.clear_interrupt(EdgeLow);
        }
        if m.button_3.interrupt_status(EdgeLow) {
            info!("Button3 pressed");
            m.button_3.clear_interrupt(EdgeLow);
        }
    })
}

// End of file
