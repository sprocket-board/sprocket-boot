#![no_main]
#![no_std]

#![allow(unused_imports, dead_code, unused_variables, unused_mut)]

use core::iter::Cloned;
use core::iter::Cycle;

use sprocket_boot as _; // global logger + panicking-behavior + memory layout
use smart_leds::RGB;
use stm32g0xx_hal::{
    stm32::{self, DWT, SPI2, TIM1},
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
    flash::{
        self,
        Read as _,
        FlashPage,
        WriteErase as _,
        Error as FlashError,
    },
};
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{RGB8, SmartLedsWrite, colors, gamma};

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello, world!");

    let board = stm32::Peripherals::take().unwrap();
    let mut core = stm32::CorePeripherals::take().unwrap();

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
    let _i2c2_scl = gpioa.pa9; // note: shadows pa11
    let _i2c2_sda = gpioa.pa10; // note: shadows pa12
    let _ = gpioa.pa11; // see above
    let _ = gpioa.pa12; // see above
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

    let mut flash = if let Ok(ulf) = board.FLASH.unlock() {
        defmt::info!("unlocked.");
        ulf
    } else {
        defmt::error!("Unlock failed!");
        sprocket_boot::exit()
    };

    extern "C" {
        static _store_start: u32;
    }

    let store_start = unsafe { &_store_start as *const _ as usize };
    let store_page = FlashPage((store_start - flash::FLASH_START) / flash::PAGE_SIZE as usize);

    defmt::info!("Flash start location: {:?}", store_start);

    let mut buffer = [0u8; 2048];

    flash.read(store_start, &mut buffer);

    for (i, ch) in buffer.chunks_exact(16).enumerate() {
        defmt::info!("chunk {:?}: {:?}", i, ch);
    }

    buffer.iter_mut().for_each(|b| *b = b.wrapping_add(1));

    cortex_m::asm::delay(64_000_000);

    if let Err(e) = flash.erase_page(store_page) {
        defmt::error!("Erase failed!");

        match e {
            FlashError::Busy => defmt::info!("busy"),
            FlashError::Illegal => defmt::info!("illegal"),
            FlashError::EccError => defmt::info!("ecc err"),
            FlashError::PageOutOfRange => defmt::info!("page oor"),
            FlashError::Failure => defmt::info!("failure"),
        }

        sprocket_boot::exit()
    }
    defmt::info!("Erase completed.");

    cortex_m::asm::delay(64_000_000);

    if let Err(e) = flash.write(store_start, &buffer) {
        defmt::error!("Write failed!");
        sprocket_boot::exit()
    }
    defmt::info!("Write completed.");

    cortex_m::asm::delay(64_000_000);

    flash.read(store_start, &mut buffer);

    for (i, ch) in buffer.chunks_exact(16).enumerate() {
        defmt::info!("chunk {:?}: {:?}", i, ch);
    }

    sprocket_boot::exit()
}

#[inline(always)]
fn every_n_ms(time: u32, interval: u32) -> bool {
    if time == 0 {
        false
    } else {
        (time % interval) == 0
    }
}

const BASIC_COLORS: &[RGB8] = &[
    colors::RED,
    colors::ORANGE,
    colors::YELLOW,
    colors::GREEN,
    colors::BLUE,
    colors::INDIGO,
    colors::VIOLET,
    colors::WHITE,
    colors::BLACK,
];
