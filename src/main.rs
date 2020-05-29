/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/
#![no_main]
#![no_std]

//!
//! Measures incoming CPPM pulses on the
//! Pixracer (R12 version) board, using the port marked "RCIN"
//! This is intended as an example of using the ppm_decode library
//! with a real RC receiver (FrSky X4R) and microcontroller (stm32f4)
//!
//!

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use p_hal::gpio::{Edge, ExtiPin, GpioExt};
use p_hal::rcc::RccExt;
use p_hal::time::U32Ext;

use pac::interrupt;

use cortex_m_rt as rt;
use rt::entry;

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::interrupt::Mutex;
use ppm_decode::PpmParser;

use core::sync::atomic::{AtomicUsize, Ordering};

/// The resolution of the timer used to mark the arrival of PPM pulses
type PpmTimerTick = u16;
/// enough ticks to account for at least one full PPM frame
const MAX_PPM_TICK: PpmTimerTick = 0xFFFF;

/// Type of the GPIO input pin we use to read the CPPM input signal
type PpmInputPin = p_hal::gpio::gpiob::PB0<p_hal::gpio::Input<p_hal::gpio::PullUp>>;

/// Stores the shared PPM input pin
static PPM_INPUT_PIN: Mutex<RefCell<Option<PpmInputPin>>> = Mutex::new(RefCell::new(None));

/// Stores the PPM decoder touched by interrupt handler and main loop
static PPM_DECODER: Mutex<RefCell<Option<PpmParser>>> = Mutex::new(RefCell::new(None));

/// For debugging/tracing output: stores the timer tick of the most recent PPM pulse edge
static LAST_RAW_TICK: AtomicUsize = AtomicUsize::new(0);

/// Initialize peripherals for Pixracer board.
/// The Pixracer chip is:
/// [STM32F427VIT6 rev.3](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
///
pub fn setup_peripherals() -> (
    // LED output pins
    (
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
    ),
    // A delay source
    impl DelayMs<u8>,
) {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // enable system configuration controller clock
    dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());
    let rcc = dp.RCC.constrain();
    // configure clocks according to what we know the Pixracer board operates at:
    let clocks = rcc
        .cfgr
        .use_hse(24.mhz()) // 24 MHz xtal
        .sysclk(168.mhz()) // HCLK
        .pclk1(42.mhz()) // APB1 clock is HCLK/4
        .pclk2(84.mhz()) // APB2 clock is HCLK/2
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let user_led1 = gpiob.pb11.into_push_pull_output(); //red
    let user_led2 = gpiob.pb1.into_push_pull_output(); //green
    let user_led3 = gpiob.pb3.into_push_pull_output(); //blue

    // turn on hardware inverter of RCIN on Pixracer
    let mut rc_input_inv = gpioc.pc13.into_push_pull_output();
    rc_input_inv.set_high().expect("couldn't enable PPM signal inverter");

    // setup a free-running microsecond timer for timing PPM edges
    unsafe {
        config_hrt(&mut dp.TIM3);
    }

    // PPM-in pin is PB0
    let mut ppm_in = gpiob.pb0.into_pull_up_input();
    ppm_in.make_interrupt_source(&mut dp.SYSCFG);
    // Although the FrSky CPPM is inverted, we enabled a hw inverter above
    ppm_in.trigger_on_edge(&mut dp.EXTI, Edge::RISING);
    ppm_in.enable_interrupt(&mut dp.EXTI);

    cortex_m::interrupt::free(|cs| {
        PPM_INPUT_PIN.borrow(cs).replace(Some(ppm_in));
    });

    // Enable EXTI0 interrupt in NVIC
    pac::NVIC::unpend(pac::Interrupt::EXTI0);
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI0);
    };

    ((user_led1, user_led2, user_led3), delay_source)
}

/// Configure a free-running microsecond timer
/// for marking when we receive PPM pulse edges
unsafe fn config_hrt(tim: &mut pac::TIM3) {
    // attach the timer to the clock
    let rcc = &(*pac::RCC::ptr());
    rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());
    rcc.apb1rstr.modify(|_, w| w.tim3rst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());

    // disable the timer
    tim.cr1.modify(|_, w| w.cen().clear_bit());
    // reset counter
    tim.cnt.reset();

    // allow for up to this many ticks to be counted
    tim.arr.write(|w| w.bits(MAX_PPM_TICK as u32));

    // configure prescaler for 1 microsecond ticks (1 MHz)
    // APB1 is 42 MHz with ppre factor of 2 -> 84 MHz TIM_CLOCK
    const PSC: u16 = 84 - 1; // prescaler should be desired N - 1
    tim.psc.write(|w| w.psc().bits(PSC));

    // start timer free running
    tim.cr1.modify(|_, w| w.cen().set_bit());
}

/// This interrupt handler should be called whenever the microcontroller detects
/// a PPM pulse edge on the RCIN input pin.
#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        // clear the interrupt
        if let Some(ref mut pin) = PPM_INPUT_PIN.borrow(cs).borrow_mut().deref_mut() {
            pin.clear_interrupt_pending_bit();
        }
    });

    // get the timer count at the moment of the pulse edge
    // NOTE(unsafe) atomic read with no side effects
    let raw_tick = unsafe { (*pac::TIM3::ptr()).cnt.read().bits() as PpmTimerTick };

    // tell the decoder about the new pulse detected
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut decoder) = PPM_DECODER.borrow(cs).borrow_mut().deref_mut() {
            decoder.handle_pulse_start(raw_tick as ppm_decode::PpmTime);
        };
    });

    // the following is for debugging purposes: print the gap between detected edges
    let last_tick = LAST_RAW_TICK.load(Ordering::Relaxed) as PpmTimerTick;
    let wrap_delta = if raw_tick > last_tick {
        raw_tick - last_tick
    } else {
        (MAX_PPM_TICK - last_tick) + raw_tick
    };
    LAST_RAW_TICK.store(raw_tick as usize, Ordering::Relaxed);
    rprintln!("{}", wrap_delta);
}

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    cortex_m::interrupt::free(|cs| {
        let mut parser = ppm_decode::PpmParser::new();
        parser
            .set_minimum_channels(8)
            .set_sync_width(14000)
            .set_max_ppm_time(MAX_PPM_TICK  as ppm_decode::PpmTime);
        PPM_DECODER.borrow(cs).replace(Some(parser))
    });

    let ((mut user_led1, mut user_led2, mut user_led3), _delay_source) = setup_peripherals();

    let _ = user_led1.set_low();
    let _ = user_led2.set_high();
    let _ = user_led3.set_high();

    loop {
        let _ = user_led1.toggle();

        if let Some(frame) = cortex_m::interrupt::free(|cs| {
            if let Some(ref mut decoder) = PPM_DECODER.borrow(cs).borrow_mut().deref_mut() {
                decoder.next_frame()
            } else {
                None
            }
        })
        {
            // we received a complete frame
            let _ = user_led2.set_low();
            rprintln!(
                "chans {}: {:?}",
                frame.chan_count,
                &frame.chan_values[..frame.chan_count as usize]
            );
        }
        else {
            // no available PPM frame
            let _ = user_led2.set_high();
        }
    }
}
