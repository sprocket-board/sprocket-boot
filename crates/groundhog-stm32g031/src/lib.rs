#![no_std]

use groundhog::RollingTimer;
use embedded_hal::blocking::delay::{DelayUs, DelayMs};
use core::sync::atomic::{AtomicPtr, Ordering};
use stm32g0xx_hal::{
    rcc::Rcc,
    stm32::{
        TIM2,
        RCC,
        tim2::RegisterBlock as Tim2Rb,
    },
};

static TIMER_PTR: AtomicPtr<Tim2Rb> = AtomicPtr::new(core::ptr::null_mut());

pub struct GlobalRollingTimer;

impl GlobalRollingTimer {
    pub const fn new() -> Self {
        Self
    }

    pub fn init(timer: TIM2) {
        let rcc = unsafe {&*RCC::ptr()};

        rcc.apbenr1.modify(|_, w| w.tim2en().set_bit());
        rcc.apbrstr1.modify(|_, w| w.tim2rst().set_bit());
        rcc.apbrstr1.modify(|_, w| w.tim2rst().clear_bit());


        // pause
        timer.cr1.modify(|_, w| w.cen().clear_bit());
        // reset counter
        timer.cnt.reset();

        // Calculate counter configuration

        timer.psc.write(|w| unsafe { w.psc().bits(63) });
        timer.arr.write(|w| unsafe { w.bits(0xFFFFFFFF) });
        timer.egr.write(|w| w.ug().set_bit());
        timer.cr1.modify(|_, w| w.cen().set_bit().urs().set_bit());

        // TODO: Critical section?
        let old_ptr = TIMER_PTR.load(Ordering::SeqCst);
        TIMER_PTR.store(TIM2::ptr() as *mut _, Ordering::SeqCst);

        debug_assert!(old_ptr == core::ptr::null_mut());
    }
}

// impl Monotonic for GlobalRollingTimer {
//     type Instant = i32;

//     fn ratio() -> Fraction {
//         Fraction {
//             numerator: 64,
//             denominator: 1,
//         }
//     }

//     fn now() -> Self::Instant {
//         Self::new().get_ticks() as i32
//     }

//     fn zero() -> Self::Instant {
//         0
//     }

//     unsafe fn reset() {
//         if let Some(t0) = TIMER_PTR.load(Ordering::SeqCst).as_ref() {
//             t0.tasks_clear.write(|w| w.bits(1));
//         }
//     }
// }

impl RollingTimer for GlobalRollingTimer {
    type Tick = u32;
    const TICKS_PER_SECOND: u32 = 1_000_000;

    fn get_ticks(&self) -> u32 {
        if let Some(t0) = unsafe { TIMER_PTR.load(Ordering::SeqCst).as_ref() } {
            t0.cnt.read().bits()
        } else {
            0
        }
    }
}

impl DelayUs<u32> for GlobalRollingTimer {
    fn delay_us(&mut self, us: u32) {
        let start = self.get_ticks();
        while self.ticks_since(start) < us { }
    }
}

impl DelayMs<u32> for GlobalRollingTimer {
    fn delay_ms(&mut self, ms: u32) {
        for _ in 0..ms {
            self.delay_us(1000)
        }
    }
}
