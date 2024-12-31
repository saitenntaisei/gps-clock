//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{self as bsp, hal::timer::Alarm};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use core::cell::RefCell;
use critical_section::Mutex;
use fugit::MicrosDurationU32;
use pac::interrupt;

type LedAndAlarm = (
    bsp::hal::gpio::Pin<
        bsp::hal::gpio::bank0::Gpio25,
        bsp::hal::gpio::FunctionSioOutput,
        bsp::hal::gpio::PullDown,
    >,
    bsp::hal::timer::Alarm0,
);

static mut LED_AND_ALARM: Mutex<RefCell<Option<LedAndAlarm>>> = Mutex::new(RefCell::new(None));

const FAST_BLINK_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::millis(10);

#[entry]
fn main() -> ! {
    info!("Program start");
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let led_pin = pins.led.into_push_pull_output();

    let mut timer = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    critical_section::with(|cs| {
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(FAST_BLINK_INTERVAL_US);
        alarm.enable_interrupt();
        unsafe {
            LED_AND_ALARM.borrow(cs).replace(Some((led_pin, alarm)));
        }
    });
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    loop {
        // info!("on!");
        // led_pin.set_high().unwrap();
        // delay.delay_ms(500);
        // info!("off!");
        // led_pin.set_low().unwrap();
        delay.delay_ms(100);
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    // (7)
    critical_section::with(|cs| {
        let ledalarm = unsafe { LED_AND_ALARM.borrow(cs).take() };
        if let Some((mut led, mut alarm)) = ledalarm {
            alarm.clear_interrupt();
            let _ = alarm.schedule(FAST_BLINK_INTERVAL_US);

            unsafe {
                static mut COUNT: u8 = 0;
                COUNT += 1;
                if 9 < COUNT {
                    COUNT = 0;
                    // Blink the LED so we know we hit this interrupt
                    led.toggle().unwrap();
                }
            }

            // Return LED_AND_ALARM into our static variable
            unsafe {
                LED_AND_ALARM
                    .borrow(cs)
                    .replace_with(|_| Some((led, alarm)));
            }
        }
    });
}

// End of file
