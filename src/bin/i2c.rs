#![no_main]
#![no_std]

#![allow(unused_imports, dead_code, unused_variables, unused_mut)]

use core::iter::Cloned;
use core::iter::Cycle;

use cortex_m::peripheral::SCB;
use sprocket_boot as _; // global logger + panicking-behavior + memory layout
use smart_leds::RGB;
use stm32g0xx_hal::gpio::Analog;
use stm32g0xx_hal::gpio::gpioa::PA11;
use stm32g0xx_hal::gpio::gpioa::PA12;
use stm32g0xx_hal::{
    stm32::{self, DWT, SPI2, TIM1, I2C2},
    prelude::*,
    // adc::{Adc, config::AdcConfig},
    spi::{Spi, NoSck, NoMiso},
    // serial::{Serial, config::Config},
    timer::Timer,
    rcc::{
        Config,
        SysClockSrc,
        PllConfig,
        Prescaler,
    },
};
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{RGB8, SmartLedsWrite, colors, gamma};
// use stm32f4_prng::{
//     seed_rng,
//     RngConfig,
//     EntropySources,
// };
// use nb::block;

use stm32g0xx_hal::i2c_periph::{
    I2CPeripheral,
};

fn inner_main() -> Result<(), ()> {
    // defmt::info!("Hello, world!");

    let board = stm32::Peripherals::take().ok_or(())?;
    let mut core = stm32::CorePeripherals::take().ok_or(())?;
    // core.DCB.enable_trace();
    // DWT::unlock();
    // core.DWT.enable_cycle_counter();

    let config = Config::pll()
        .pll_cfg(PllConfig::with_hsi(1, 8, 2))
        .ahb_psc(Prescaler::NotDivided)
        .apb_psc(Prescaler::NotDivided);
    let mut rcc = board.RCC.freeze(config);


    let gpioa = board.GPIOA.split(&mut rcc);
    let gpiob = board.GPIOB.split(&mut rcc);
    let gpioc = board.GPIOC.split(&mut rcc);

    let mut timer1 = board.TIM1.pwm(4096.hz(), &mut rcc);
    let mut timer2 = board.TIM2.pwm(4096.hz(), &mut rcc);
    let mut timer16 = board.TIM16.pwm(4096.hz(), &mut rcc);

    let mut led1 = timer2.bind_pin(gpioa.pa0);
    let _gpio1 = gpioa.pa1;
    let _uart_tx = gpioa.pa2;
    let _uart_rx = gpioa.pa3;
    let smartled = gpioa.pa4;
    let _gpio2 = gpioa.pa5;
    let _gpio3 = gpioa.pa6;
    let _gpio4 = gpioa.pa7;
    let _gpio7 = gpioa.pa8;
    // let i2c2_scl = gpioa.pa9; // note: shadows pa11
    // let i2c2_sda = gpioa.pa10; // note: shadows pa12
    let i2c2_scl = gpioa.pa11; // note: shadows pa11
    let i2c2_sda = gpioa.pa12; // note: shadows pa12
    // let _ = gpioa.pa11; // see above
    // let _ = gpioa.pa12; // see above
    let _swdio = gpioa.pa13;
    let _swclk = gpioa.pa14; // also boot0
    let _spi_csn = gpioa.pa15;

    let _gpio5 = gpiob.pb0;
    let _gpio6 = gpiob.pb1;
    let _spi_sck = gpiob.pb3;
    let _spi_cipo = gpiob.pb4;
    let _spi_copi = gpiob.pb5;
    let _i2c1_scl = gpiob.pb6;
    let _i2c1_sda = gpiob.pb7;
    let mut led2 = timer16.bind_pin(gpiob.pb8);
    // Other pins not on this chip: PB2, PB9-15

    let _gpio8 = gpioc.pc6;
    let button1 = gpioc.pc14.into_floating_input();
    let button2 = gpioc.pc15.into_floating_input();
    // Other pins not on this chip: PC0-5, PC7-13

    // No gpiod or gpioe
    // gpiof: only PF2 is available as nRST, not used

    let mut duty: u16 = 0;
    let mut is_pwm_on = false;
    let mut last_b1 = false;
    let mut last_b2 = false;

    led1.set_duty((duty as u32) << 16);
    led2.set_duty(duty);

    let mut i2c = I2CPeripheral::new(
        board.I2C2,
        i2c2_sda,
        i2c2_scl,
        &mut rcc,
    );
    let mut buf = [0x00u8; 256];

    loop {
        // defmt::info!("Waiting for address match...");
        let i2cpac = i2c.borrow_pac();

        loop {
            if i2cpac.isr.read().addr().bit_is_set() {
                break;
            }
        }

        // defmt::info!("Addr match!");

        if i2cpac.isr.read().addcode().bits() != 0x69 {
            // panic!("Bad address!");
            return Err(());
        }

        // 0: Write transfer, slave enters receiver mode.
        // 1: Read transfer, slave enters transmitter mode.
        if i2cpac.isr.read().dir().bit_is_set() {
            //
            // READ
            //

            // TODO: What is TXE? "Optional: Set I2C_ISR.TXE = 1"
            //
            // ISR::TXE
            //
            // This bit is set by hardware when the I2C_TXDR register is empty. It is cleared when the next
            // data to be sent is written in the I2C_TXDR register.
            // This bit can be written to ‘1’ by software in order to flush the transmit data register
            // I2C_TXDR.

            // ACK the address
            i2cpac.icr.write(|w| {
                w.addrcf().set_bit()
            });

            // defmt::info!("Acknowledged addr");

            for (i, byte) in buf.iter().enumerate() {
                while i2cpac.isr.read().txis().bit_is_clear() { }

                i2cpac.txdr.modify(|_, w| {
                    unsafe {
                        w.txdata().bits(*byte)
                    }
                });

                // defmt::info!("Sent byte {:?}", i);
            }

            while i2cpac.isr.read().txis().bit_is_clear() { }

            // defmt::info!("Read complete!");
        } else {
            //
            // WRITE
            //

            // TODO: What is TXE? "Optional: Set I2C_ISR.TXE = 1"
            //
            // ISR::TXE
            //
            // This bit is set by hardware when the I2C_TXDR register is empty. It is cleared when the next
            // data to be sent is written in the I2C_TXDR register.
            // This bit can be written to ‘1’ by software in order to flush the transmit data register
            // I2C_TXDR.

            // ACK the address
            i2cpac.icr.write(|w| {
                w.addrcf().set_bit()
            });

            // defmt::info!("Acknowledged addr");

            for (i, byte) in buf.iter_mut().enumerate() {
                while i2cpac.isr.read().rxne().bit_is_clear() { }

                *byte = i2cpac.rxdr.read().rxdata().bits();
                // defmt::info!("Received byte {:?}", i);
            }

            while i2cpac.isr.read().txis().bit_is_clear() { }

            // defmt::info!("Write complete!");
        }
    }
    // sprocket_boot::exit();
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let _ = inner_main();
    SCB::sys_reset()
}

#[inline(always)]
fn every_n_ms(time: u32, interval: u32) -> bool {
    if time == 0 {
        false
    } else {
        (time % interval) == 0
    }
}
