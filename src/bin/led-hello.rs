#![no_main]
#![no_std]

use core::iter::Cloned;
use core::iter::Cycle;
// use rand_core::RngCore;

use sprocket_bringup as _; // global logger + panicking-behavior + memory layout
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

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello, world!");

    let board = stm32::Peripherals::take().unwrap();
    let mut core = stm32::CorePeripherals::take().unwrap();
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

    let spi_lamp = Spi::spi2(
        board.SPI2,
        (NoSck, NoMiso, smartled),
        MODE,
        3_800_000.hz(),
        &mut rcc,
    );
    let mut lamp = Ws2812::new(spi_lamp);

    // let mut adc = Adc::adc1(board.ADC1, true, AdcConfig::default());

    // let mut timer = Timer::tim1(board.TIM1, 10_000_000.hz(), clocks);

    // let mut rng = seed_rng(
    //     &mut adc,
    //     RngConfig {
    //         min_init_cycles: 8192,
    //         max_init_cycles: 65535,
    //         entropy_sources: EntropySources::AllSources(stm32f4_prng::SampleTime::Cycles_480),
    //     }
    // );

    defmt::info!("Clocks good!");

    for _ in 0..3 {
        defmt::info!("Black!");
        let data: [RGB8; 2] = [colors::BLACK; 2];
        // fix_spi_errors();
        lamp.write(gamma(data.iter().cloned())).unwrap(); // .unwrap();
        delay(64_000_000 / 100);
    }

    let mut coloop = BASIC_COLORS.iter().cloned().cycle();
    let mut color = colors::BLACK;
    let mut red = 0;
    let mut grn = 0;
    let mut blu = 0;
    let mut down = false;

    let mut duty: u16 = 0;
    let mut is_pwm_on = false;
    let mut last_b1 = false;
    let mut last_b2 = false;

    led1.set_duty((duty as u32) << 16);
    led2.set_duty(duty);

    let mut ms = 0u32;
    loop {
        ms = ms.wrapping_add(1);

        let is_done = || {
            color.r == red &&
            color.g == grn &&
            color.b == blu
        };

        if !down && is_done() {
            defmt::info!("Down!");
            color = colors::BLACK;
            down = true;
        } else if down && is_done() {
            defmt::info!("Up!");
            color = coloop.next().unwrap();
            down = false;
        }

        let b1_now = button1.is_high().unwrap();
        let b2_now = button2.is_high().unwrap();

        if !b1_now && last_b1 {
            if is_pwm_on {
                led1.disable();
                led2.disable();
                duty = 0;
            } else {
                led1.set_duty(duty as u32);
                led2.set_duty(duty);
                led1.enable();
                led2.enable();
            }

            is_pwm_on = !is_pwm_on;
        }

        if !b2_now && last_b2 {
            duty = duty.wrapping_add(512);
            defmt::info!("duty: {:?}", duty);
            led1.set_duty((duty as u32));
            led2.set_duty(duty);

            // LED1, not so useful above 4096?
            // LED1, not so useful above 16384?
        }

        last_b1 = b1_now;
        last_b2 = b2_now;

        // if every_n_ms(ms, 10) {
        //     if down {
        //         red = red.saturating_sub(1);
        //         grn = grn.saturating_sub(1);
        //         blu = blu.saturating_sub(1);
        //     } else {
        //         red = (red.saturating_add(1)).min(color.r);
        //         grn = (grn.saturating_add(1)).min(color.g);
        //         blu = (blu.saturating_add(1)).min(color.b);
        //     }
        //     let colors = [
        //         RGB8 { r: red, g: grn, b: blu },
        //         RGB8 { r: red, g: grn, b: blu },
        //     ];
        //     lamp.write(gamma(colors.iter().cloned())).unwrap();
        // }

    //     if every_n_ms(ms, 1000) {
    //         // let _ = dotloop.next();
    //         for (idx, (led, col)) in cube.pix.iter_mut().zip(dotloop.clone()).enumerate() {
    //             led.current.set_color(col);
    //             if col == colors::WHITE {
    //                 defmt::info!("{:?}", idx);
    //             }

    //             // led.steps_remaining = (1000 / 16);
    //             // led.step = led.target.sub(&led.current).clamp(-1.0, 1.0).div(led.steps_remaining as f32);
    //         }
    //     }

    //     // if every_n_ms(ms, 16) {
    //     //     for led in cube.pix.iter_mut() {
    //     //         if led.steps_remaining >= 1 {
    //     //             led.current = led.current.add(&led.step).clamp(0.0, 1.0);
    //     //             led.steps_remaining -= 1;
    //     //         }
    //     //     }
    //     // }


    //     if every_n_ms(ms, 16) {
    //         let data = cube.to_array();
    //         fix_spi_errors();
    //         lamp.write(gamma(data.iter().cloned())).unwrap();
    //     }

    //     let _ = block!(timer.wait());
        cortex_m::asm::delay(64_000_000 / 1000);
    }
}

// #[derive(Clone, Copy, Debug)]
// struct ColorF {
//     r: f32,
//     g: f32,
//     b: f32,
// }

// impl ColorF {
//     fn set_color(&mut self, color: RGB8) {
//         self.r = color.r as f32 / 255.0;
//         self.g = color.g as f32 / 255.0;
//         self.b = color.b as f32 / 255.0;
//     }

//     fn to_color(&self) -> RGB8 {
//         RGB8 {
//             r: (self.r * 255.0) as u8,
//             g: (self.g * 255.0) as u8,
//             b: (self.b * 255.0) as u8,
//         }
//     }

//     fn add(&self, other: &Self) -> Self {
//         Self {
//             r: self.r + other.r,
//             g: self.g + other.g,
//             b: self.b + other.b,
//         }
//     }

//     fn sub(&self, other: &Self) -> Self {
//         Self {
//             r: self.r - other.r,
//             g: self.g - other.g,
//             b: self.b - other.b,
//         }
//     }

//     fn clamp(&self, min: f32, max: f32) -> Self {
//         Self {
//             r: libm::fminf(libm::fmaxf(self.r, min), max),
//             g: libm::fminf(libm::fmaxf(self.g, min), max),
//             b: libm::fminf(libm::fmaxf(self.b, min), max),
//         }
//     }

//     fn div(&self, val: f32) -> Self {
//         if val == 0.0 {
//             Self {
//                 r: 0.0,
//                 g: 0.0,
//                 b: 0.0,
//             }
//         } else {
//             Self {
//                 r: self.r / val,
//                 g: self.g / val,
//                 b: self.b / val,
//             }
//         }
//     }

//     fn black() -> Self {
//         Self {
//             r: 0.0f32,
//             g: 0.0f32,
//             b: 0.0f32,
//         }
//     }
// }

// #[derive(Clone, Copy, Debug)]
// struct Pixel {
//     current: ColorF,
//     target: ColorF,
//     step: ColorF,
//     steps_remaining: u32,
// }

// impl Pixel {
//     fn new() -> Self {
//         Self {
//             current: ColorF::black(),
//             target: ColorF::black(),
//             step: ColorF::black(),
//             steps_remaining: 0u32,
//         }
//     }
// }

// struct Cube {
//     pix: [Pixel; 8],
// }

// impl Cube {
//     fn new() -> Self {
//         Cube {
//             pix: [Pixel::new(); 8]
//         }
//     }

//     fn to_array(&self) -> [RGB8; 8] {
//         let mut arr = [colors::BLACK; 8];
//         arr.iter_mut()
//             .zip(self.pix.iter())
//             .for_each(|(a, p)| *a = p.current.to_color());
//         arr
//     }
// }

// fn row_col_to_pos(row: usize, col: usize) -> usize {
//     match (row, col) {
//         (0, 0) => 0,
//         (0, 1) => 1,
//         (0, 2) => 2,
//         (0, 3) => 3,
//         (1, 0) => 7,
//         (1, 1) => 6,
//         (1, 2) => 5,
//         (1, 3) => 4,
//         (2, 0) => 8,
//         (2, 1) => 9,
//         (2, 2) => 10,
//         (2, 3) => 11,
//         (3, 0) => 15,
//         (3, 1) => 14,
//         (3, 2) => 13,
//         (3, 3) => 12,
//         (4, 0) => 16,
//         (4, 1) => 17,
//         (4, 2) => 18,
//         (4, 3) => 19,
//         (5, 0) => 23,
//         (5, 1) => 22,
//         (5, 2) => 21,
//         (5, 3) => 20,
//         _ => 24
//     }
// }

#[inline(always)]
fn every_n_ms(time: u32, interval: u32) -> bool {
    if time == 0 {
        false
    } else {
        (time % interval) == 0
    }
}

// // Fix Errors
// fn fix_spi_errors() {
//     unsafe {
//         for spi in &[&*SPI2::ptr(), &*SPI5::ptr()] {
//             // }
//             // let spi = &*SPI5::ptr();

//             // Read from DR register to clear OVR
//             let _ = spi.dr.read();

//             // Read from SR to clear CRCERR and OVR
//             spi.sr.modify(|r, w| {
//                 let _ = r.txe().bit_is_set();

//                 if r.crcerr().bit_is_set() {
//                     w.crcerr().clear_bit();
//                 }
//                 w
//             });

//             // Write to CR1 to clear MODF
//             spi.cr1.modify(|_r, w| w);
//         }
//     }
// }

// fn defmt_color(color: &RGB8) {
//     match *color {
//         colors::BLACK => defmt::info!("BLACK"),
//         colors::DARK_BLUE => defmt::info!("DARK_BLUE"),
//         colors::BLUE => defmt::info!("BLUE"),
//         colors::TURQUOISE => defmt::info!("TURQUOISE"),
//         colors::GREEN => defmt::info!("GREEN"),
//         colors::NAVY => defmt::info!("NAVY"),
//         colors::DARK_GRAY => defmt::info!("DARK_GRAY"),
//         colors::DIM_GRAY => defmt::info!("DIM_GRAY"),
//         colors::SLATE_GRAY => defmt::info!("SLATE_GRAY"),
//         colors::DARK_SLATE_GRAY => defmt::info!("DARK_SLATE_GRAY"),
//         colors::WHITE => defmt::info!("WHITE"),
//         _ => defmt::info!("Color ???"),
//     }
// }

// const COLD_COLORS: &[RGB8] = &[
//     colors::BLACK,
//     colors::BLACK,
//     colors::BLACK,
//     colors::BLACK,
//     colors::BLACK,
//     colors::BLACK,
//     colors::BLACK,
//     colors::BLACK,
//     colors::DARK_BLUE,
//     colors::DARK_BLUE,
//     colors::DARK_BLUE,
//     colors::DARK_BLUE,
//     colors::BLUE,
//     colors::BLUE,
//     colors::BLUE,
//     colors::BLUE,
//     colors::BLUE,
//     colors::BLUE,
//     colors::BLUE,
//     colors::BLUE,
//     colors::TURQUOISE,
//     colors::GREEN,
//     colors::NAVY,
//     colors::NAVY,
//     colors::NAVY,
//     colors::NAVY,
//     colors::DARK_GRAY,
//     colors::DIM_GRAY,
//     colors::SLATE_GRAY,
//     colors::DARK_SLATE_GRAY,
//     colors::DARK_GRAY,
//     colors::DIM_GRAY,
//     colors::SLATE_GRAY,
//     colors::DARK_SLATE_GRAY,
//     colors::WHITE,
//     colors::WHITE,
// ];

// // pub const LIGHT_GRAY:               RGB8 = RGB8 { r: 0xD3, g: 0xD3, b: 0xD3 };
// // pub const DARK_GRAY:                RGB8 = RGB8 { r: 0xA9, g: 0xA9, b: 0xA9 };
// // pub const DIM_GRAY:                 RGB8 = RGB8 { r: 0x69, g: 0x69, b: 0x69 };
// // pub const LIGHT_SLATE_GRAY:         RGB8 = RGB8 { r: 0x77, g: 0x88, b: 0x99 };
// // pub const SLATE_GRAY:               RGB8 = RGB8 { r: 0x70, g: 0x80, b: 0x90 };
// // pub const DARK_SLATE_GRAY:          RGB8 = RGB8 { r: 0x2F, g: 0x4F, b: 0x4F };


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


const FIRE_COLORS: &[RGB8] = &[
    // colors::BLACK,
    // colors::BLACK,
    // colors::BLACK,
    // colors::BLACK,
    // colors::BLACK,
    // colors::BLACK,
    // colors::BLACK,
    // colors::BLACK,
    colors::DARK_RED,
    colors::DARK_RED,
    colors::DARK_RED,
    colors::DARK_RED,
    colors::ORANGE,
    colors::ORANGE,
    colors::ORANGE,
    colors::ORANGE,
    colors::ORANGE,
    colors::ORANGE,
    colors::ORANGE,
    colors::ORANGE,
    colors::ORANGE_RED,
    colors::ORANGE_RED,
    colors::ORANGE_RED,
    colors::ORANGE_RED,
    colors::CRIMSON,
    colors::CRIMSON,
    colors::CRIMSON,
    colors::CRIMSON,
    colors::RED,
    colors::RED,
    colors::RED,
    colors::RED,
    colors::LEMON_CHIFFON,
    colors::GOLD,
];
