#![no_main]
#![no_std]


use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::timer::CountDown;

use stm32f4xx_hal as p_hal;
use p_hal::stm32 as pac;

use p_hal::gpio::{GpioExt, ExtiPin};
use p_hal::rcc::RccExt;
use p_hal::time::{U32Ext};
use p_hal::dwt::{ClockDuration, DwtExt};
use cortex_m::peripheral::DWT;

use p_hal::timer::Timer;

use pac::interrupt;

use cortex_m_rt as rt;
use rt::entry;

use ppm_decode::{PpmParser};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use core::ops::DerefMut;
use p_hal::gpio::{Input, PullUp, Edge};

type PpmInputPin = p_hal::gpio::gpiob::PB0<p_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>;

/// Stores the shared input pin
static PPM_INPUT_PIN: Mutex<RefCell<Option< PpmInputPin >>> = Mutex::new(RefCell::new(None));

/// Stores the PPM decoder touched by interrupt handlers and main loop
static PPM_DECODER: Mutex<RefCell<Option<PpmParser>>>  = Mutex::new(RefCell::new(None));

/// Verify: clock ticks per microsecond
const TICKS_PER_MICRO: u32 =   2; //(168/8);
const CYCLES_PER_MICRO: u32 =   12;

/// Initialize peripherals for Pixracer. Pixracer chip is:
/// [STM32F427VIT6 rev.3](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
pub fn setup_peripherals()  -> (
    (
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
    ),
    impl DelayMs<u8>,
)
{
    let mut dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();


    // enable system configuration controller clock
    dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(24.mhz()) // 24 MHz xtal
        .sysclk(168.mhz()) // HCLK
        .pclk1(42.mhz()) // APB1 clock is HCLK/4
        .pclk2(84.mhz()) // APB2 clock is HCLK/2
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split();

    let user_led1 = gpiob.pb11.into_push_pull_output(); //red
    let user_led2 = gpiob.pb1.into_push_pull_output(); //green
    let user_led3 = gpiob.pb3.into_push_pull_output(); //blue

    // setup a microsecond timer for timing PPM edges
    // use cap/comp channel 3 on TIM3
    //let mut fast_timer = Timer::tim3(dp.TIM3, 1.mhz(), clocks);
    // You could use the 32-bit TIM2 counter,
    // prescaled down to 1MHz so ticks were each 1us,
    // you could use the maximal update interrupt to extend to 64-bit,
    // but you'd have to create methods to read both 32-bit portions in an atomic fashion;

    //init CYCCNT cortex_m peripheral
    DWT::unlock();
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    // let _dwt = cp.DWT.constrain(cp.DCB, clocks);

    // PPM input:
    // use TIM3 for HRT_TIMER
    // use cap/comp channel 3 on TIM3 for PPM
    // PPM in pin is gpiob.pb0  af2
    //TODO verify: no need to switch to AF2 ? (using into_alternate_af2() )
    let mut ppm_in = gpiob.pb0.into_pull_up_input();
    ppm_in.make_interrupt_source(&mut dp.SYSCFG);
    ppm_in.enable_interrupt(&mut dp.EXTI);
    ppm_in.trigger_on_edge(&mut dp.EXTI, Edge::FALLING);

    cortex_m::interrupt::free(|cs| {
        PPM_INPUT_PIN.borrow(cs).replace(Some(ppm_in));
    });

    // Enable EXTI0 interrupt in NVIC
    pac::NVIC::unpend(pac::Interrupt::EXTI0);
    unsafe { pac::NVIC::unmask(pac::Interrupt::EXTI0); };

    (
        (user_led1, user_led2, user_led3),
        delay_source,
    )
}

/// This interrupt handler should be called whenever
/// the PPM input pin detects a (rising) edge.
#[interrupt]
fn EXTI0() {
    let microtime =
        cortex_m::interrupt::free(|cs| {
            // clear the interrupt
            if let Some(ref mut pin) =  PPM_INPUT_PIN.borrow(cs).borrow_mut().deref_mut() {
                pin.clear_interrupt_pending_bit();
            }

            // get current time
            // let cur_cycles = DWT::get_cycle_count();
            // rprintln!("cycles: {}", cur_cycles);
            // let micros = cur_cycles/CYCLES_PER_MICRO;

            // // TODO get clock time in a safer way? this is atomic with no side effects
            let cur_sys_ticks = unsafe { (*pac::SYST::ptr()).cvr.read() };
            let monotonic_ticks = u32::max_value() - cur_sys_ticks;
            let micros = monotonic_ticks / TICKS_PER_MICRO;

            rprintln!("ticks: {} micros: {}", monotonic_ticks, micros);

            // tell the PPM decoder we got a pulse start
            if let Some(ref mut decoder) = PPM_DECODER.borrow(cs).borrow_mut().deref_mut() {
                decoder.handle_pulse_start(micros);
            }
            micros
        });

    //rprintln!("micros: {}", microtime);
}



#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    //create and stash the ppm decoder
    let parser = ppm_decode::PpmParser::new();
    cortex_m::interrupt::free(|cs| {
        PPM_DECODER.borrow(cs).replace(Some(parser))
    });

    let (
        (mut user_led1,
            mut user_led2,
            mut user_led3),
        mut delay_source,
    ) = setup_peripherals();

    let _ = user_led1.set_low();
    let _ = user_led2.set_high();
    let _ = user_led3.set_high();


    loop {
        // all the interesting stuff happens in interrupt handlers
        let _ = user_led1.toggle();

        if let Some(frame) =
            cortex_m::interrupt::free(|cs| {
                if let Some(ref mut parser) =
                PPM_DECODER.borrow(cs).borrow_mut().deref_mut() {
                    parser.next_frame()
                } else {
                    None
                }
            })
        {
            // valid PPM frame parsed
            let _ = user_led2.set_low();
            // let subset = frame.chan_values[..frame.chan_count]
            rprintln!("chans {}: {:?}", frame.chan_count, &frame.chan_values[..frame.chan_count as usize]);
        }
        else {
            // no available PPM frame
            let _ = user_led2.set_high();
            delay_source.delay_ms(10u8);

        }
    }
}