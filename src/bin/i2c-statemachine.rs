// Note to self:
//
// A big part of this code is talking about READs and WRITEs. These are all from
// the perspective of the Controller, not us, the Peripheral.
//
// So:
//   * When you see READ, this means we are TRANSMITTING.
//   * When you see WRITE, this means we are RECEIVING.
//
// It's backwards. I know. I'm open to better naming schemes.
//
// TODO(AJM): Move this comment if/when I move the state machine code somewhere
// else, like a dedicated crate.

// TODO(AJM): What about when we get a stop before we expect?
// TODO(AJM): Should I be disabling I2C whenever I do a program or write?

#![no_main]
#![no_std]

#![allow(unused_imports, dead_code, unused_variables, unused_mut)]

use core::iter::Cloned;
use core::iter::Cycle;

use cortex_m::peripheral::SCB;
use sprocket_boot as _; // global logger + panicking-behavior + memory layout
use smart_leds::RGB;
use stm32g0xx_hal::flash;
use stm32g0xx_hal::flash::FlashPage;
use stm32g0xx_hal::flash::Read;
use stm32g0xx_hal::flash::UnlockedFlash;
use stm32g0xx_hal::flash::WriteErase;
use stm32g0xx_hal::flash::Error as FlashError;
use stm32g0xx_hal::gpio::Analog;
use stm32g0xx_hal::gpio::Output;
use stm32g0xx_hal::gpio::PushPull;
use stm32g0xx_hal::gpio::gpioa::PA0;
use stm32g0xx_hal::gpio::gpioa::PA11;
use stm32g0xx_hal::gpio::gpioa::PA12;
use stm32g0xx_hal::gpio::gpiob::PB8;
use stm32g0xx_hal::{
    stm32::{self, DWT, SPI2, TIM1, I2C2},
    prelude::*,
    // adc::{Adc, config::AdcConfig},
    spi::{Spi, NoSck, NoMiso},
    // serial::{Serial, config::Config},
    timer::{
        Timer,
        stopwatch::Stopwatch,
    },
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

use groundhog::RollingTimer;
use groundhog_stm32g031::GlobalRollingTimer;

use stm32g0xx_hal::i2c_periph::{
    I2CPeripheral,
};

const SKIP_FLASH: bool = false;

const TOTAL_PAGES: usize = 24;

// TODO: You REALLY can't change this, because I am bit-packing
const SUBPAGES_PER_PAGE: usize = 8;

const TOTAL_SUBPAGES: usize = TOTAL_PAGES * SUBPAGES_PER_PAGE;

struct PageMap {
    pages: [u8; TOTAL_PAGES],
    active_pages: usize,
    unlocked: bool,
    msp: usize,
    reset_vector: usize,
}

impl PageMap {
    fn get_page_mut(&mut self, page: usize) -> Option<&mut u8> {
        if !self.unlocked || page >= self.active_pages {
            return None;
        }

        self.pages.get_mut(page)
    }

    fn ready_to_write_vector_subpage(&self) -> bool {
        // defmt::info!("map: {:?}", self.pages);

        if !(self.unlocked && self.active_pages > 0) {
            return false;
        }

        // TODO: I probably want to ensure that the vector table SUBPAGE is written first in this page, because
        // if we lose power, we could be in trouble.
        self.pages[0] == 0xFE &&
            self.pages
                .iter()
                .skip(1)
                .take(self.active_pages - 1)
                .all(|p| *p == 0xFF)
    }

    fn bootload_complete(&self) -> bool {
        self.unlocked &&
            self.active_pages > 0 &&
            self.pages
                .iter()
                .take(self.active_pages)
                .all(|p| *p == 0xFF)
    }
}

fn inner_main() -> Result<(), ()> {
    // defmt::info!("Hello, world!");

    let board = stm32::Peripherals::take().ok_or(())?;
    let mut core = stm32::CorePeripherals::take().ok_or(())?;

    cortex_m::interrupt::disable();

    let config = Config::pll()
        .pll_cfg(PllConfig::with_hsi(1, 8, 2))
        .ahb_psc(Prescaler::NotDivided)
        .apb_psc(Prescaler::NotDivided);
    let mut rcc = board.RCC.freeze(config);


    let gpioa = board.GPIOA.split(&mut rcc);
    let gpiob = board.GPIOB.split(&mut rcc);
    let gpioc = board.GPIOC.split(&mut rcc);

    let mut timer1 = board.TIM1.pwm(4096.hz(), &mut rcc);
    let mut timer16 = board.TIM16.pwm(4096.hz(), &mut rcc);

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
    let mut led1 = gpioa.pa0.into_push_pull_output();
    let mut led2 = gpiob.pb8.into_push_pull_output();
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

    let mut i2c = I2CPeripheral::new(
        board.I2C2,
        i2c2_sda,
        i2c2_scl,
        &mut rcc,
    );

    GlobalRollingTimer::init(board.TIM2);
    let ght = GlobalRollingTimer::new();
    let mut start = ght.get_ticks();
    let mut ct = 0;

    // TODO: Probably don't need to do this at boot
    let mut flash = if let Ok(ulf) = board.FLASH.unlock() {
        // defmt::info!("unlocked.");
        ulf
    } else {
        // defmt::error!("Unlock failed!");
        sprocket_boot::exit()
    };

    // Read RAM settings
    let mut stay_bootloader = false;

    // Are both buttons held?
    stay_bootloader |= {
        let buttons_held = (Ok(true), Ok(true)) == (button1.is_low(), button2.is_low());

        if buttons_held {
            // defmt::info!("Holding in bootloader due to buttons!");
        }

        buttons_held
    };

    stay_bootloader |= {
        extern "C" {
            static _ram_flags: u8;
        }

        let mut buf = [0u8; 128];

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        let ram_flags_addr: *mut u8 = unsafe { &_ram_flags as *const _  as *mut _};
        let buffer_addr: *mut u8 = buf.as_mut_ptr();
        // defmt::info!("Loading ram page at {=u32:X}", ram_flags_addr as u32);

        unsafe {
            core::ptr::copy_nonoverlapping(ram_flags_addr, buffer_addr, 128);

            // Overwrite magic word
            ram_flags_addr.write_bytes(0x00, 4);
        }

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        if &0xCAFEB007u32.to_le_bytes() != &buf[..4] {
            // defmt::info!("No good RAM data.");
            false
        } else {
            match buf[4] {
                0x00 => true,
                0x01 => false,
                _ => {
                    // defmt::error!("Bad command! Staying in bootloader!");
                    true
                }
            }
        }
    };

    let settings_msp;
    let settings_rsv;

    stay_bootloader |= {
        // Read Settings page
        extern "C" {
            static _settings_start: u32;
            static _app_start: u32;
        }

        let settings_start = unsafe { &_settings_start as *const _ as usize };
        let app_start = unsafe { &_app_start as *const _ as usize };

        let mut settings_bytes = [0u8; 12];
        let mut app_bytes = [0u8; 8];

        flash.read(settings_start, &mut settings_bytes);
        let (magic_bytes_sl, remainder) = settings_bytes.split_at(4);
        let (msp_bytes_sl, rsv_bytes_sl) = remainder.split_at(4);

        let mut magic_bytes = [0u8; 4];
        let mut msp_bytes = [0u8; 4];
        let mut rsv_bytes = [0u8; 4];

        magic_bytes.copy_from_slice(magic_bytes_sl);
        msp_bytes.copy_from_slice(msp_bytes_sl);
        rsv_bytes.copy_from_slice(rsv_bytes_sl);

        let magic = u32::from_le_bytes(magic_bytes);
        settings_msp = u32::from_le_bytes(msp_bytes);
        settings_rsv = u32::from_le_bytes(rsv_bytes);

        // ---

        flash.read(app_start, &mut app_bytes);
        let (app_msp_bytes_sl, app_rsv_bytes_sl) = app_bytes.split_at(4);

        let mut app_msp_bytes = [0u8; 4];
        let mut app_rsv_bytes = [0u8; 4];

        app_msp_bytes.copy_from_slice(app_msp_bytes_sl);
        app_rsv_bytes.copy_from_slice(app_rsv_bytes_sl);

        let app_msp = u32::from_le_bytes(app_msp_bytes);
        let app_rsv = u32::from_le_bytes(app_rsv_bytes);

        if magic != 0xB007CAFEu32 {
            // defmt::info!("Bad settings magic!");
            // Bad magic, keep in bootloader
            true
        } else {
            extern "C" {
                static PreResetTrampoline: u32;
                static _stack_start: u32;
            }

            let boot_rsv = unsafe { &PreResetTrampoline as *const _ as u32 };
            let boot_msp = unsafe { &_stack_start as *const _ as u32 };

            // defmt::info!("app_msp: {=u32:X}", app_msp);
            // defmt::info!("app_rsv: {=u32:X}", app_rsv);

            // defmt::info!("boot_msp: {=u32:X}", boot_msp);
            // defmt::info!("boot_rsv: {=u32:X}", boot_rsv);

            // defmt::info!("settings_msp: {=u32:X}", settings_msp);
            // defmt::info!("settings_rsv: {=u32:X}", settings_rsv);

            !(
                // the APP MSP and RSV should actually point to the bootloader
                app_msp == boot_msp &&
                app_rsv == boot_rsv &&

                // Basic check on the MSP and RSV from settings (shouldn't be necessary)
                settings_msp != 0xFFFFFFFF &&
                settings_rsv != 0xFFFFFFFF
            )
        }
    };

    if !stay_bootloader {
        defmt::info!("bootloading!");
        defmt::info!("MSP: {=u32:X}", settings_msp);
        defmt::info!("RSV: {=u32:X}", settings_rsv);

        unsafe {
            for _ in 0..4 {
                led1.set_high().ok();
                led2.set_low().ok();

                cortex_m::asm::delay(64_000_000 / 8);

                led1.set_low().ok();
                led2.set_high().ok();

                cortex_m::asm::delay(64_000_000 / 8);
            }
            cortex_m::asm::bootstrap(settings_msp as *const u32, settings_rsv as *const u32);
        }
    }

    // defmt::info!("Launching bootloader!");

    let mut boot = BootMachine::new(i2c, flash, led1, led2);

    while let Ok(_) = boot.poll() {

    }

    loop {
        boot.led1.set_high().ok();
        boot.led2.set_low().ok();

        cortex_m::asm::delay(64_000_000 / 4);

        boot.led1.set_low().ok();
        boot.led2.set_high().ok();

        cortex_m::asm::delay(64_000_000 / 4);
    }
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

struct BootMachine {
    i2c: I2CPeripheral,
    timer: GlobalRollingTimer,
    transfer: Transfer,
    buffer: [u8; 512],
    flash: UnlockedFlash,
    map: PageMap,
    sq: StateQueue,
    flash_action: BootLoadAction,
    led1: PA0<Output<PushPull>>,
    led2: PB8<Output<PushPull>>,
}

#[derive(Debug, Eq, PartialEq)]
enum BootLoadAction {
    Idle,
    EraseThenWrite {
        page: usize,
        subpage: u8,
    },
    Writing {
        page: usize,
        subpage: u8,
    }
}

enum Transfer {
    Idle,
    Writing {
        addr: u8,
        len: usize,
        idx: usize,
    },
    Reading {
        addr: u8,
        len: usize,
        idx: usize,
    },
    Invalid,
}

#[derive(Debug, PartialEq, Eq)]
enum TransferDir {
    Read,
    Write,
}

type QueueFunc<T> = fn(&mut BootMachine) -> Result<T, ()>;

#[derive(Clone)]
struct StateQueueItem(QueueFunc<bool>);

impl Default for StateQueueItem {
    fn default() -> Self {
        Self(nop_state_err)
    }
}

fn nop_state_err<T>(_: &mut BootMachine) -> Result<T, ()> {
    // defmt::error!("Nop state error!");
    Err(())
}

fn nop_state_ok(_: &mut BootMachine) -> Result<(), ()> {
    Ok(())
}

fn nop_state_true(_: &mut BootMachine) -> Result<bool, ()> {
    Ok(true)
}

const ITEMS: usize = 8;

struct StateQueue {
    items: [StateQueueItem; ITEMS],
    idx: usize,
}

impl StateQueue {
    fn new() -> Self {
        Self {
            idx: 0,
            items: [
                StateQueueItem::default(),
                StateQueueItem::default(),
                StateQueueItem::default(),
                StateQueueItem::default(),
                StateQueueItem::default(),
                StateQueueItem::default(),
                StateQueueItem::default(),
                StateQueueItem::default(),
            ]
        }
    }

    fn from_slice(items: &[StateQueueItem]) -> Self {
        assert!(items.len() <= 8, "Increase Queue Size!");

        let mut new = Self::new();
        new.idx = 0;
        items.iter().rev().take(ITEMS).for_each(|i| {
            new.items[new.idx] = i.clone();
            new.idx += 1;
        });
        new
    }

    fn pop(&mut self) -> Option<StateQueueItem> {
        if self.idx == 0 {
            None
        } else {
            let mut replace = StateQueueItem::default();
            core::mem::swap(&mut replace, &mut self.items[self.idx - 1]);
            self.idx -= 1;
            Some(replace)
        }
    }

    fn push(&mut self, item: StateQueueItem) -> Result<(), ()> {
        if self.idx == ITEMS {
            // defmt::error!("Push fail!");
            return Err(());
        }

        self.items[self.idx] = item;
        self.idx += 1;
        Ok(())
    }

    fn len(&self) -> usize {
        self.idx
    }
}

impl BootMachine {
    fn new(i2c: I2CPeripheral, flash: UnlockedFlash, led1: PA0<Output<PushPull>>, led2: PB8<Output<PushPull>>) -> Self {
        Self {
            i2c,
            flash,
            buffer: [0u8; 512],
            timer: GlobalRollingTimer::new(),
            transfer: Transfer::Idle,
            map: PageMap {
                pages: [0; TOTAL_PAGES],
                active_pages: 0,
                unlocked: false,
                msp: 0,
                reset_vector: 0,
            },
            sq: StateQueue::from_slice(&[
                StateQueueItem(BootMachine::match_address_write),
                StateQueueItem(BootMachine::match_write_register),
            ]),
            flash_action: BootLoadAction::Idle,
            led1,
            led2,
        }
    }

    fn complete_write(&mut self) -> Result<bool, ()> {
        let (addr, mut idx, len) = if let Transfer::Writing { addr, len, idx } = &self.transfer {
            (*addr, *idx, *len)
        } else {
            // defmt::error!("Wrong state for complete_write!");
            return Err(());
        };

        if idx >= len {
            panic!("Why are you writing more")
        }

        let i2cpac = self.i2c.borrow_pac();

        if i2cpac.isr.read().rxne().bit_is_clear() {
            // Not done
            return Ok(false);
        }

        self.buffer[idx] = i2cpac.rxdr.read().rxdata().bits();
        idx += 1;

        // Some kind of timing race... when this line is uncommented
        // it causes the host to get a NAK??? But this line is never hit
        //
        // TODO: wat
        //
        // defmt::info!("Wrote byte {:?}/{:?}", idx, len);

        if idx >= len {
            // defmt::info!("Done! Waiting for stop");
            Ok(true)
        } else {
            self.transfer = Transfer::Writing { addr, len, idx };
            Ok(false)
        }
    }

    fn complete_read(&mut self) -> Result<bool, ()> {
        let (addr, mut idx, len) = if let Transfer::Reading { addr, len, idx } = &self.transfer {
            (*addr, *idx, *len)
        } else {
            // defmt::error!("wrong state!");
            return Err(());
        };

        let i2cpac = self.i2c.borrow_pac();

        if i2cpac.isr.read().txis().bit_is_clear() {
            // Not done
            return Ok(false);
        }

        if idx >= len {
            panic!("Bounds checking failure")
        }

        i2cpac.txdr.modify(|_, w| {
            unsafe {
                w.txdata().bits(self.buffer[idx])
            }
        });
        idx += 1;

        if idx == len {
            Ok(true)
        } else {
            self.transfer = Transfer::Reading { addr, len, idx };
            Ok(false)
        }
    }

    fn wait_for_stop(&mut self) -> Result<bool, ()> {
        let i2cpac = self.i2c.borrow_pac();

        let isr = i2cpac.isr.read();

        // Is the controller asking us for more data still?
        if isr.txis().bit_is_set() {
            self.nak();
            // defmt::error!("asked for more read when stop expected!");
            i2cpac.txdr.modify(|_, w| {
                unsafe {
                    w.txdata().bits(0)
                }
            });
        }

        // Is the controller giving us more data still?
        if isr.rxne().bit_is_clear() {
            self.nak();
            // defmt::error!("got write when stop expected!");
            let _ = i2cpac.rxdr.read().rxdata().bits();
        }

        // Is the controller finally done?
        if i2cpac.isr.read().stopf().bit_is_set() {
            i2cpac.icr.write(|w| {
                w.stopcf().set_bit()
            });
            // defmt::info!("got stop.");
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn match_write_register(&mut self) -> Result<bool, ()> {
        if let Some(data) = self.get_written_byte() {
            self.sq = self.start_txfr(data);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn match_address_read(&mut self) -> Result<bool, ()> {
        // We always expect a write from idle (write or write-then-read)
        if self.check_addr_match(TransferDir::Read) {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn match_address_write(&mut self) -> Result<bool, ()> {
        // We always expect a write from idle (write or write-then-read)
        if self.check_addr_match(TransferDir::Write) {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn poll_inner(&mut self) -> Result<(), ()> {
        let mut action = self.sq.pop().unwrap(); // TODO

        if !(action.0)(self)? {
            let x = self.sq.push(action);
            if x.is_err() {
                // defmt::error!("Queue is full???");
            }

            x?;
        }

        Ok(())
    }

    fn poll(&mut self) -> Result<(), ()> {
        match self.poll_inner() {
            Ok(()) => Ok(()),
            Err(()) => {
                todo!("clear the queue, go into some kind of recovery?");
            }
        }
    }

    fn erase_page(&mut self) -> Result<bool, ()> {
        let mut old_action = BootLoadAction::Idle;
        core::mem::swap(&mut old_action, &mut self.flash_action);

        let (page, subpage) = if let BootLoadAction::EraseThenWrite { page, subpage } = old_action {
            (page, subpage)
        } else {
            // defmt::error!("Tried to erase when not prepared!");
            return Err(())
        };

        // defmt::info!("erasing page {=u32}...", page as u32);

        if !SKIP_FLASH {
            if let Err(e) = self.flash.erase_page(FlashPage(page)) {
                // defmt::error!("Erase failed:");

                match e {
                    FlashError::Busy => defmt::info!("busy."),
                    FlashError::Illegal => defmt::info!("illegal."),
                    FlashError::EccError => defmt::info!("ecc err."),
                    FlashError::PageOutOfRange => defmt::info!("page oor."),
                    FlashError::Failure => defmt::info!("failure."),
                }

                return Err(());
            }
        } else {
            defmt::warn!("Skipped actual erase! Pretend it's good.");
        }

        // Erase good! Move on to write. We unconditionally write after an erase
        self.flash_action = BootLoadAction::Writing { page, subpage };

        Ok(true)
    }

    fn write_subpage(&mut self) -> Result<bool, ()> {
        let mut old_action = BootLoadAction::Idle;
        core::mem::swap(&mut old_action, &mut self.flash_action);

        let (page, subpage) = if let BootLoadAction::Writing { page, subpage } = old_action {
            (page, subpage)
        } else {
            // defmt::error!("Tried to write when not prepared!");
            return Err(())
        };

        let subpage_image = &self.buffer[1..257];

        if subpage_image.iter().all(|b| *b == 0xFF) {
            // defmt::info!("Page is all 0xFF, skipping write");
            return Ok(true);
        }

        extern "C" {
            static _app_start: u32;
        }

        let app_start = unsafe { &_app_start as *const _ as usize };

        // NOTE: app_start is already offset to the base of the temporary flash region
        let store_addr = (app_start + (page * flash::PAGE_SIZE as usize))
            + (subpage as usize * 256);

        // defmt::info!("Writing subpage at {=u32:X}...", store_addr as u32);

        if !SKIP_FLASH {
            self.flash.write(store_addr, subpage_image).map_err(|_| {
                // defmt::error!("Write failed!");
            })?;
        } else {
            defmt::warn!("Skipped actual write! Pretend it's good.");
            defmt::info!("page: {:?}", subpage_image);
        }

        Ok(true)
    }

    fn enable_i2c(&mut self) -> Result<bool, ()> {
        if self.i2c.is_enabled() {
            // defmt::error!("Oops, expected to be disabled but we're not");
            return Err(());
        }
        unsafe {
            self.i2c.enable();
        }
        Ok(true)
    }

    fn disable_i2c(&mut self) -> Result<bool, ()> {
        if !self.i2c.is_enabled() {
            // defmt::error!("Oops, expected to be enabled but we're not");
            return Err(());
        }
        unsafe {
            self.i2c.disable();
        }
        Ok(true)
    }

    fn check_write_page_data(&mut self) -> Result<bool, ()> {
        // Is bootloading active?
        // TODO: This duplicates the get_page_mut check
        if !self.map.unlocked {
            // defmt::error!("Bootloading not started!");
            return Err(());
        }

        let (addr, mut idx, len) = if let Transfer::Writing { addr, len, idx } = &self.transfer {
            (*addr, *idx, *len)
        } else {
            // defmt::error!("Wrong state for check_write_page_data!");
            return Err(());
        };

        if len != (1 + 256 + 4) {
            // defmt::error!("Wrong number of bytes!");
            return Err(());
        }
        // * 1: Page/Subpage
        //     * 0bPPPPP_SSS (32 pages, 8 sub-pages)
        //     * 0bPPPPP must be <= 23
        // * 256: subpage contents
        // * 4 byte: For now: 32-bit checksum. Later, CRC32 or Poly1305?
        let ps = self.buffer[0];

        let page = (ps >> 3) as usize;
        let subpage = ps & 0b111;

        assert!(page < TOTAL_PAGES);
        assert!((subpage as usize) < SUBPAGES_PER_PAGE);

        let mut checksum_unchecked_bytes = [0u8; 4];
        checksum_unchecked_bytes.copy_from_slice(&self.buffer[257..261]);
        let checksum_unchecked = u32::from_le_bytes(checksum_unchecked_bytes);

        let calc_checksum = generate_checksum(&self.buffer[1..257], None).ok_or(())?;

        if calc_checksum != checksum_unchecked {
            // defmt::error!("Checksum mismatch!");
            return Err(());
        }

        if page == 0 && subpage == 0 {
            extern "C" {
                static PreResetTrampoline: u32;
                static _stack_start: u32;
            }

            if !self.map.ready_to_write_vector_subpage() {
                // defmt::error!("Must write page 0 subpage 0 last!");
                return Err(());
            }

            let reset_vector = unsafe { &PreResetTrampoline as *const _ as usize };
            let msp = unsafe { &_stack_start as *const _ as usize };

            // defmt::info!("Patching reset vector to the bootloader's: {=usize:X}", reset_vector);
            // defmt::info!("Patching MSP to the bootloader's: {=usize:X}", msp);

            let mut reset_vector_bytes = [0u8; 4];
            let mut msp_bytes = [0u8; 4];

            msp_bytes.copy_from_slice(&self.buffer[1..5]);
            reset_vector_bytes.copy_from_slice(&self.buffer[5..9]);

            self.map.reset_vector = usize::from_le_bytes(reset_vector_bytes);
            self.map.msp = usize::from_le_bytes(msp_bytes);

            self.buffer[1..5].copy_from_slice(&msp.to_le_bytes());
            self.buffer[5..9].copy_from_slice(&reset_vector.to_le_bytes());
        }


        let map = self.map.get_page_mut(page).ok_or(())?;

        if *map == 0 {
            // defmt::info!("Subpage accepted. Erase + Flash started");
            // All subpages have never been written, time to erase page
            *map = *map | (1 << subpage);
            self.sq = StateQueue::from_slice(&[
                StateQueueItem(BootMachine::wait_for_stop),
                StateQueueItem(BootMachine::disable_i2c),
                StateQueueItem(BootMachine::erase_page),
                StateQueueItem(BootMachine::write_subpage),
                StateQueueItem(BootMachine::enable_i2c),
                StateQueueItem(BootMachine::match_address_write),
                StateQueueItem(BootMachine::match_write_register),
            ]);

            self.flash_action = BootLoadAction::EraseThenWrite {
                page,
                subpage,
            };
            Ok(true)
        } else if (*map & 1 << subpage) == 0 {
            // defmt::info!("Subpage accepted. Flash started");
            *map = *map | (1 << subpage);

            self.sq = StateQueue::from_slice(&[
                StateQueueItem(BootMachine::wait_for_stop),
                StateQueueItem(BootMachine::disable_i2c),
                StateQueueItem(BootMachine::write_subpage),
                StateQueueItem(BootMachine::enable_i2c),
                StateQueueItem(BootMachine::match_address_write),
                StateQueueItem(BootMachine::match_write_register),
            ]);

            self.flash_action = BootLoadAction::Writing {
                page,
                subpage,
            };
            Ok(true)
        } else {
            // defmt::error!("Can't re-write pages!");
            Err(())
        }
    }

    fn activate_bootload(&mut self) -> Result<bool, ()> {
        let len = if let Transfer::Writing { len, .. } = self.transfer {
            len
        } else {
            // defmt::error!("Wrong state!");
            return Err(())
        };

        if len != 5 {
            // defmt::error!("Wrong number of bytes!");
            return Err(());
        }

        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.buffer[..4]);

        let checksum = u32::from_le_bytes(checksum_bytes);
        let subpages = self.buffer[4];

        if subpages as usize > TOTAL_SUBPAGES {
            // defmt::error!("Too many subpages!");
            return Err(());
        }

        if subpages % (SUBPAGES_PER_PAGE as u8) != 0 {
            // defmt::error!("Must provide whole page!");
            return Err(())
        }

        let pages = (subpages >> 3) as usize;

        if self.map.unlocked {
            // defmt::warn!("Already unlocked, resetting programming!");
        }

        self.map.pages.iter_mut().for_each(|b| {
            *b = 0;
        });

        self.map.unlocked = true;
        self.map.active_pages = pages;

        // defmt::info!("Unlocked Bootloading; pages: {=u8}, checksum: {=u32:X}", subpages, checksum);

        Ok(true)
    }

    fn dummy_read(&mut self) {
        // defmt::info!("Dummy read!");
    }

    fn dummy_write(&mut self, len: usize) {
        let good_buf = &self.buffer[..len];
        // defmt::info!("Dummy write of {:?} bytes!", len);
    }

    fn write_settings_page(&mut self) -> Result<bool, ()> {
        extern "C" {
            static _settings_start: u32;
        }

        let settings_start = unsafe { &_settings_start as *const _ as usize };

        // defmt::info!("Reading settings page...");

        if !SKIP_FLASH {
            self.flash.read(settings_start, &mut self.buffer[..256]);
        } else {
            // defmt::warn!("Skipping real settings read! Loading all 0xFFs");
            self.buffer[..256].iter_mut().for_each(|b| *b = 0xFF);
        }

        // TODO: actual serialization/deserialization of settings page
        // defmt::warn!("Manually applying new reset vector and msp to settings page");

        // defmt::info!("App MSP: {=usize:X}", self.map.msp);
        // defmt::info!("App RsV: {=usize:X}", self.map.reset_vector);

        self.buffer[..4].copy_from_slice(&0xB007CAFEu32.to_le_bytes());
        self.buffer[4..8].copy_from_slice(&self.map.msp.to_le_bytes());
        self.buffer[8..12].copy_from_slice(&self.map.reset_vector.to_le_bytes());

        // TODO: De-duplicate this with erase page and write page. Right now it's
        // hardcoded to the offset write section, instead of the total range.

        // TODO: not magic numbers
        let real_page = (settings_start - 0x0800_0000) / 2048;

        if !SKIP_FLASH {
            if let Err(e) = self.flash.erase_page(FlashPage(real_page)) {
                // defmt::error!("Erase failed:");

                match e {
                    FlashError::Busy => defmt::info!("busy."),
                    FlashError::Illegal => defmt::info!("illegal."),
                    FlashError::EccError => defmt::info!("ecc err."),
                    FlashError::PageOutOfRange => defmt::info!("page oor."),
                    FlashError::Failure => defmt::info!("failure."),
                }

                return Err(());
            }
        } else {
            // defmt::warn!("Skipped actual settings erase! Pretend it's good.");
        }

        if !SKIP_FLASH {
            self.flash.write(settings_start, &self.buffer[..256]).map_err(|_| {
                // defmt::error!("Write failed!");
            })?;
        } else {
            // defmt::warn!("Skipped actual settings write! Pretend it's good.");
        }

        Ok(true)
    }

    fn reboot(&mut self) -> Result<bool, ()> {
        if !SKIP_FLASH {
            // TODO: Re-lock flash?
            // TODO: Clear RAM flags?
            for _ in 0..3 {
                self.led1.set_high().ok();
                self.led2.set_high().ok();

                cortex_m::asm::delay(64_000_000);

                self.led1.set_low().ok();
                self.led2.set_low().ok();

                cortex_m::asm::delay(64_000_000);
            }

            SCB::sys_reset()
        } else {
            panic!("Bootload complete!");
        }
    }

    fn start_txfr(&mut self, addr: u8) -> StateQueue {
        match addr {
            // * Write transactions:

            // * 0x40 - Start bootload
            //     * 4 bytes - total checksum (later crc32), little endian
            //     * 1 byte - total subpages to write
            //         * NOTE: must be less than 23 * 8 for now
            0x40 => {
                // defmt::info!("Start Bootload");
                self.transfer = Transfer::Writing {
                    addr,
                    len: 5,
                    idx: 0,
                };
                StateQueue::from_slice(&[
                    StateQueueItem(BootMachine::complete_write),
                    StateQueueItem(BootMachine::activate_bootload),
                    StateQueueItem(BootMachine::wait_for_stop),
                    StateQueueItem(BootMachine::match_address_write),
                    StateQueueItem(BootMachine::match_write_register),
                ])
            }

            // * 0x41 - write page command
            //     * Must have set 0x40 already
            //     * Always 1 + 1 + 256 + 4 bytes
            //         * 1: 0x41
            //         * 1: Page/Subpage
            //             * 0bPPPPP_SSS (32 pages, 8 sub-pages)
            //             * 0bPPPPP must be <= 23
            //         * 256: subpage contents
            //         * 4 byte: For now: 32-bit checksum. Later, CRC32 or Poly1305?
            //     * On writes to subpage zero: erase page
            //     * Stretch write ack until checksum + erase + write?
            //         * I can't use the flash while erasing (or writing?) anyway
            //     * On writes to 0:0: make sure the reset vector is 0x0800_C000
            //         * Make sure MSP is maxval?
            //         * 0:0 must be the last thing written
            0x41 => {
                // defmt::info!("Write Page");
                self.transfer = Transfer::Writing {
                    addr,
                    len: 1 + 256 + 4,
                    idx: 0,
                };
                StateQueue::from_slice(&[
                    StateQueueItem(BootMachine::complete_write),
                    StateQueueItem(BootMachine::check_write_page_data),
                    // check_write_page_data will set the sequence based on the response
                ])
            }

            //     * 0x42 - complete and reboot
            0x42 => {
                // defmt::info!("Complete and Reboot");
                if !self.map.bootload_complete() {
                    // defmt::error!("Bootload not complete! Not rebooting.");

                    StateQueue::from_slice(&[
                        StateQueueItem(BootMachine::wait_for_stop),
                        StateQueueItem(BootMachine::match_address_write),
                        StateQueueItem(BootMachine::match_write_register),
                    ])
                } else {
                    StateQueue::from_slice(&[
                        StateQueueItem(BootMachine::wait_for_stop),
                        StateQueueItem(BootMachine::disable_i2c),
                        StateQueueItem(BootMachine::write_settings_page),
                        StateQueueItem(BootMachine::reboot),
                    ])
                }
            }
            //     * 0x43 - abort bootload
            //         * No data
            //     * 0x44 - Offer image downstream
            //         * Only after finishing 0x40 transaction


            // * Read transactions
            //     * wr-then-rd 0x10 + 16 bytes => b'sprocket boot!!!'
            0x10 => {
                const ID: &[u8] = b"sprocket boot!!!";
                // defmt::info!("Got 0x10 write, going to read");

                (&mut self.buffer[..ID.len()]).copy_from_slice(ID);

                self.transfer = Transfer::Reading {
                    addr,
                    len: ID.len(),
                    idx: 0,
                };

                StateQueue::from_slice(&[
                    StateQueueItem(BootMachine::wait_for_stop),
                    StateQueueItem(BootMachine::match_address_read),
                    //  Nothing extra to do on complete
                    StateQueueItem(BootMachine::complete_read),
                    StateQueueItem(BootMachine::wait_for_stop),
                    StateQueueItem(BootMachine::match_address_write),
                    StateQueueItem(BootMachine::match_write_register),
                ])
            }
            //     * wr-then-rd 0x11 + 4 bytes => maj.min.triv.reserved
            //     * wr-then-rd [0x21, P:S] + 260 bytes => Read subpage
            //     * wr-then-rd 0x22 + 1 byte => status
            //     * wr-then-rd 0x23 + 4 bytes => children flashed
            _ => {
                // defmt::info!("Unexpected command: {=u8}", addr);
                todo!()
            }
        }
    }

    fn check_addr_match(&self, direction: TransferDir) -> bool {
        let i2cpac = self.i2c.borrow_pac();

        if !i2cpac.isr.read().addr().bit_is_set() {
            return false;
        }

        if i2cpac.isr.read().addcode().bits() != 0x69 {
            self.nak();
            self.ack_addr_match();
            // defmt::error!("Address Mismatch!");
            return false;
        }

        // 0: Write transfer, slave enters receiver mode.
        // 1: Read transfer, slave enters transmitter mode.
        let dir = if i2cpac.isr.read().dir().bit_is_set() {
            TransferDir::Read
        } else {
            TransferDir::Write
        };


        // Is this in the direction we expected?
        if dir != direction {
            self.nak();
            self.ack_addr_match();
            // defmt::error!("Direction Mismatch!");
            false
        } else {
            // Clear a NAK
            i2cpac.cr2.write(|w| {
                w.nack().clear_bit()
            });

            // defmt::info!("Acked a correct address+direction");
            if direction == TransferDir::Read {
                i2cpac.isr.write(|w| {
                    w.txe().set_bit()
                });
            }
            self.ack_addr_match();
            true
        }
    }

    fn ack_addr_match(&self) {
        let i2cpac = self.i2c.borrow_pac();

        // Clear the ADDR match flag
        i2cpac.icr.write(|w| {
            w.addrcf().set_bit()
        });
    }

    fn nak(&self) {
        let i2cpac = self.i2c.borrow_pac();

        // Command a NAK
        i2cpac.cr2.write(|w| {
            w.nack().set_bit()
        });
    }

    fn get_written_byte(&self) -> Option<u8> {
        let i2cpac = self.i2c.borrow_pac();

        if i2cpac.isr.read().rxne().bit_is_clear() {
            None
        } else {
            Some(i2cpac.rxdr.read().rxdata().bits())
        }
    }
}

// TODO: Replace me with something better like CRC
fn generate_checksum(data: &[u8], with_starting: Option<u32>) -> Option<u32> {
    if data.len() != 256 {
        return None;
    }

    let mut checksum = with_starting.unwrap_or(0xB007B007u32);
    for chunk in data.chunks_exact(4) {
        let mut word = [0u8; 4];
        word.copy_from_slice(chunk);
        let word = u32::from_le_bytes(word);
        checksum = checksum.wrapping_add(word);
    }

    Some(checksum)
}
