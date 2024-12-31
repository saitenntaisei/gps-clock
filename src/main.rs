#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use panic_probe as _;

use rp_pico::{self as bsp, hal::timer::Alarm, hal::timer::Alarm0, hal::timer::Timer};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use core::cell::RefCell;

use cortex_m::delay::Delay;
use cortex_m::interrupt::{free, Mutex};
use fugit::{ExtU32, MicrosDurationU32};
use once_cell::sync::Lazy;

use bsp::hal::pac::interrupt;

static TIMER_DURATION: Lazy<MicrosDurationU32> = Lazy::new(|| 100_u32.millis());

struct Machine {
    pub delay: Option<Delay>,
    pub alarm_0: Alarm0,
    pub led_pin: bsp::hal::gpio::Pin<
        bsp::hal::gpio::bank0::Gpio25,
        bsp::hal::gpio::FunctionSioOutput,
        bsp::hal::gpio::PullDown,
    >,
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

    let mut m = Machine {
        delay: delay,
        led_pin: led_pin,
        alarm_0: alarm_0,
    };
    m.reset_timer();
    Mutex::new(RefCell::new(m))
});

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut delay = free(|cs| MACHINE.borrow(cs).borrow_mut().delay.take().unwrap());
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    loop {
        delay.delay_ms(100);
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    free(|cs| {
        info!("interrupt!");
        let mut m = MACHINE.borrow(cs).borrow_mut();
        m.alarm_0.clear_interrupt();
        if let Err(e) = m.led_pin.toggle() {
            warn!("Error while interrupt: {:?}", e);
        }
        m.reset_timer();
    });
}

// End of file
