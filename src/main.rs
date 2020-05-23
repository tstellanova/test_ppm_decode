#![no_main]
#![no_std]


use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

use stm32f4xx_hal as p_hal;
use p_hal::stm32 as pac;

use p_hal::gpio::{GpioExt, ExtiPin};
use p_hal::rcc::RccExt;
use p_hal::time::{U32Ext};

use pac::interrupt;

use cortex_m_rt as rt;
use rt::entry;

use ppm_decode::{PpmParser};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use core::ops::DerefMut;
use stm32f4xx_hal::gpio::{Input, PullUp, Edge};

type PpmInputPin = p_hal::gpio::gpiob::PB0<p_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>;

//static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));
// Stores the EXTI interrupt configuration
// static MUTEX_EXTI:  Mutex<RefCell<Option<pac::EXTI>>>  = Mutex::new(RefCell::new(None));

/// Stores the shared input pin
static PPM_INPUT_PIN: Mutex<RefCell<Option< PpmInputPin >>> = Mutex::new(RefCell::new(None));

/// Stores the PPM decoder touched by interrupt handlers and main loop
static PPM_DECODER: Mutex<RefCell<Option<PpmParser>>>  = Mutex::new(RefCell::new(None));


/// Verify: clock ticks per microsecond
const TICKS_PER_MICRO: u32 =   (168/8);

/// Initialize peripherals for Pixracer.
/// Pixracer chip is [STM32F427VIT6 rev.3](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
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
    let cp = cortex_m::Peripherals::take().unwrap();
    //TODO verify: need to enable syscfg
    dp.RCC.apb2enr.write(|w| w.syscfgen().enabled());

    // Set up the system clock
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

    // PPM input:
    // use TIM3 for HRT_TIMER
    // use cap/comp channel 3 on TIM3 for PPM
    // PPM in pin is gpiob.pb0  af2
    //TODO verify: no need to switch to AF2 ? into_alternate_af2().
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

#[interrupt]
fn EXTI0() {
    let microtime =
        cortex_m::interrupt::free(|cs| {
            // clear the interrupt
            if let Some(ref mut pin) =  PPM_INPUT_PIN.borrow(cs).borrow_mut().deref_mut() {
                pin.clear_interrupt_pending_bit();
            }

            // get current time
            // TODO get clock time in a safer way? this is atomic with no side effects
            let cur_sys_ticks = unsafe { (*pac::SYST::ptr()).cvr.read() };
            // scale ticks to micros before providing to PPM decoder
            let micros = cur_sys_ticks / TICKS_PER_MICRO;
            // tell the PPM decoder we got a pulse start
            if let Some(ref mut decoder) = PPM_DECODER.borrow(cs).borrow_mut().deref_mut() {
                decoder.handle_pulse_start(micros);
            }
            micros
        });

    rprintln!("micros: {}", microtime);
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