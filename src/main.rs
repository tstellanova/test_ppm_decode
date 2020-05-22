#![no_main]
#![no_std]


use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

use stm32f4xx_hal as p_hal;
use p_hal::stm32 as pac;

use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;
use p_hal::time::{U32Ext};

use pac::interrupt;

use cortex_m_rt as rt;
use rt::entry;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use lazy_static::{lazy_static};

lazy_static! {
    static ref MUTEX_EXTI:  Mutex<RefCell<Option<pac::EXTI>>>  = Mutex::new(RefCell::new(None));
}


/// Initialize peripherals for Pixracer.
/// Pixracer chip is [STM32F427VIT6 rev.3](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
pub fn setup_peripherals()  -> (
    (
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
    ),
    impl DelayMs<u8>,
    PpmInputPin,
)
{
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

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

    // let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    // let gpioc = dp.GPIOC.split();
    // let gpiod = dp.GPIOD.split();
    // let gpioe = dp.GPIOE.split();

    let user_led1 = gpiob.pb11.into_push_pull_output(); //red
    let user_led2 = gpiob.pb1.into_push_pull_output(); //green
    let user_led3 = gpiob.pb3.into_push_pull_output(); //blue

    // PPM input:
    // use TIM3 for HRT_TIMER
    // use cap/comp channel 3 on TIM3 for PPM
    // PPM in pin is gpiob.pb0  af2
    let ppm_in = gpiob.pb0.into_alternate_af2().into_pull_up_input();

    let exti = dp.EXTI;

    // attach EXTI0 to PB0 pin (where PPM input is connected)
    dp.SYSCFG.exticr1.modify(|_, w| unsafe { w.exti0().bits(0b0001) });
    // unmask EXTI0 interrupt
    exti.imr.modify(|_, w| w.mr0().set_bit());
    //set  EXTI0 to trigger on the rising edge of the PPM input
    exti.rtsr.modify(|_, w| w.tr0().set_bit());
    //save exti into a mutex
    cortex_m::interrupt::free(|cs| {
        MUTEX_EXTI.borrow(cs).replace(Some(exti))
    });


    // enable EXTI0 interrrupt in NVIC
    unsafe { pac::NVIC::unmask(pac::interrupt::EXTI0); }

    (
        (user_led1, user_led2, user_led3),
        delay_source,
        ppm_in
    )
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        // clear the interrupt pending bit on EXTI line 0
        let exti = MUTEX_EXTI.borrow(cs).borrow();
        exti.as_ref().unwrap()
            .pr.modify(|_, w| w.pr0().set_bit());
    });
}



pub type PpmInputPin = p_hal::gpio::gpiob::PB0<p_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>;


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let     (
        (mut user_led1, mut user_led2, mut user_led3),
        mut delay_source,
        _ppm_in
    ) = setup_peripherals();

    let _ = user_led1.set_high();
    let _ = user_led2.set_low();
    let _ = user_led3.set_high();


    loop {
        // all the interesting stuff happens in interrupt handlers
        delay_source.delay_ms(250u8);
        let _ = user_led1.toggle();
    }
}