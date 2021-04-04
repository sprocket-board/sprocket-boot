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
use stm32g0xx_hal::flash::UnlockedFlash;
use stm32g0xx_hal::flash::WriteErase;
use stm32g0xx_hal::flash::Error as FlashError;
use stm32g0xx_hal::gpio::Analog;
use stm32g0xx_hal::gpio::gpioa::PA11;
use stm32g0xx_hal::gpio::gpioa::PA12;
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

const TOTAL_PAGES: usize = 4;

// TODO: You REALLY can't change this, because I am bit-packing
const SUBPAGES_PER_PAGE: usize = 8;

const TOTAL_SUBPAGES: usize = TOTAL_PAGES * SUBPAGES_PER_PAGE;

struct PageMap {
    pages: [u8; TOTAL_PAGES],
}

// TODO: This should go away. The real device will write at the
// beginning of flash.
const PAGE_OFFSET: usize = 32 - TOTAL_PAGES;

fn inner_main() -> Result<(), ()> {
    defmt::info!("Hello, world!");

    let board = stm32::Peripherals::take().ok_or(())?;
    let mut core = stm32::CorePeripherals::take().ok_or(())?;

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

    led2.set_duty(duty);

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

    let mut flash = if let Ok(ulf) = board.FLASH.unlock() {
        defmt::info!("unlocked.");
        ulf
    } else {
        defmt::error!("Unlock failed!");
        sprocket_boot::exit()
    };

    let mut boot = BootMachine::new(i2c, flash);

    loop {
        boot.poll().unwrap();
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

type WriteCompletion = fn(&mut BootMachine, usize);
type ReadCompletion = fn(&mut BootMachine);

struct BootMachine {
    i2c: I2CPeripheral,
    timer: GlobalRollingTimer,
    transfer: Transfer,
    state: BootLoadState,
    action: Option<BootLoadAction>,
    buffer: [u8; 512],
    waiting_for_stop: bool,
    on_read: Option<ReadCompletion>,
    on_write: Option<WriteCompletion>,
    flash: UnlockedFlash,
    map: PageMap,
}

#[derive(Debug, Eq, PartialEq)]
enum BootLoadAction {
    EraseThenWrite {
        erase_page: usize,
        write_page: usize,
        write_subpage: u8,
    },
    Writing {
        write_page: usize,
        write_subpage: u8,
    }
}

enum BootLoadState {
    Idle,
    BootloadEnabled {
        ttl_subpages: usize,
        checksum: u32,
    },
    Invalid,
}

enum Transfer {
    Idle,
    WriteWaitReg,
    Writing {
        addr: u8,
        len: usize,
        idx: usize,
    },
    WaitReadStart {
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

impl BootMachine {
    fn new(i2c: I2CPeripheral, flash: UnlockedFlash) -> Self {
        Self {
            i2c,
            flash,
            buffer: [0u8; 512],
            timer: GlobalRollingTimer::new(),
            transfer: Transfer::Idle,
            state: BootLoadState::Idle,
            waiting_for_stop: false,
            on_read: None,
            on_write: None,
            action: None,
            map: PageMap { pages: [0; TOTAL_PAGES] },
        }
    }

    fn poll(&mut self) -> Result<(), ()> {
        if self.waiting_for_stop {
            if self.process_stop() {
                defmt::info!("Got stop!");
                self.i2c.borrow_pac().cr2.write(|w| {
                    w.nack().clear_bit()
                });
                self.waiting_for_stop = false;
            } else {
                return Ok(());
            }
        }

        let mut done = false;

        self.action.take().map(|a| {
            // Disable I2C. We know we just got a stop. Maybe.
            // NAK everything until we are done
            unsafe {
                self.i2c.disable();
            }

            extern "C" {
                static _store_start: u32;
            }

            let store_start = unsafe { &_store_start as *const _ as usize };
            // let store_page = FlashPage((store_start - flash::FLASH_START) / flash::PAGE_SIZE as usize);

            let action = if let BootLoadAction::EraseThenWrite {
                erase_page,
                write_page,
                write_subpage,
            } = a {
                if let Err(e) = self.flash.erase_page(FlashPage(PAGE_OFFSET + erase_page)) {
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

                // Erase good! Move on to write
                BootLoadAction::Writing { write_page, write_subpage }
            } else {
                a
            };

            if let BootLoadAction::Writing {
                write_page,
                write_subpage,
            } = action {
                // NOTE: store_start is already offset to the base of the temporary flash region
                let store_addr = (store_start + (write_page * flash::PAGE_SIZE as usize))
                    + (write_subpage as usize * 256);
                if let Err(e) = self.flash.write(store_addr, &self.buffer[1..257]) {
                    defmt::error!("Write failed!");
                    sprocket_boot::exit()
                }
            } else {
                defmt::error!("What?!?!");
            };

            unsafe {
                self.i2c.enable();
            }
        });

        if done {
            return Ok(());
        }

        if !self.i2c.is_enabled() {
            defmt::error!("Oops, expected to be enabled but we're not");
            return Err(());
        }

        // HERE, I need to see if an action is pending, and start acting on it.

        let mut old_state = Transfer::Invalid;
        core::mem::swap(&mut self.transfer, &mut old_state);

        // Check state of active transfer
        self.transfer = match old_state {
            Transfer::Idle => {
                // We always expect a write from idle (write or write-then-read)
                if self.check_addr_match(TransferDir::Write) {
                    Transfer::WriteWaitReg
                } else {
                    Transfer::Idle
                }
            }
            Transfer::WriteWaitReg => {
                if let Some(data) = self.get_written_byte() {
                    if let Some(txfr) = self.start_txfr(data) {
                        txfr
                    } else {
                        defmt::error!("Bad txfr");
                        self.nak();
                        Transfer::Idle
                    }
                } else {
                    Transfer::WriteWaitReg
                }
            }
            Transfer::WaitReadStart { addr, len, idx } => {
                // TODO: I probably shouldn't NAK here, but if I got an unexpected
                // write command, I should just abort the expected read and move on
                //
                // check_addr_match probably needs some better ability to state whether
                // we are waiting, something bad happened, etc.
                if self.check_addr_match(TransferDir::Read) {
                    Transfer::Reading { addr, len, idx }
                } else {
                    Transfer::WaitReadStart { addr, len, idx }
                }
            }
            Transfer::Writing { addr, len, idx } => {
                self.process_write(addr, len, idx)
            }
            Transfer::Reading { addr, len, idx } => {
                self.process_read(addr, len, idx)
            }
            _ => todo!()
        };

        Ok(())
    }

    fn write_page(&mut self, len: usize) {
        if len != (1 + 256 + 4) {
            defmt::error!("Wrong number of bytes!");
            return;
        }
        //         * 1: Page/Subpage
        //             * 0bPPPPP_SSS (32 pages, 8 sub-pages)
        //             * 0bPPPPP must be <= 23
        //         * 256: subpage contents
        //         * 4 byte: For now: 32-bit checksum. Later, CRC32 or Poly1305?
        let ps = self.buffer[0];

        let page = (ps >> 3) as usize;
        let subpage = ps & 0b111;

        assert!(page < TOTAL_PAGES);
        assert!((subpage as usize) < SUBPAGES_PER_PAGE);

        todo!("check checksum!");

        let map = &mut self.map.pages[page];

        if *map == 0 {
            *map = *map | (1 << subpage);
            // All subpages have never been written, time to flash
            self.action = Some(BootLoadAction::EraseThenWrite {
                erase_page: page,
                write_page: page,
                write_subpage: subpage,
            })
        } else if (*map & 1 << subpage) == 0 {
            *map = *map | (1 << subpage);
            self.action = Some(BootLoadAction::Writing {
                write_page: page,
                write_subpage: subpage,
            });
        } else {
            defmt::error!("Can't re-write pages!");
        }

        //
    }

    fn activate_bootload(&mut self, len: usize) {
        if len != 5 {
            defmt::error!("Wrong number of bytes!");
            return;
        }

        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.buffer[..4]);

        let checksum = u32::from_le_bytes(checksum_bytes);
        let subpages = self.buffer[4];

        if subpages as usize > TOTAL_SUBPAGES {
            defmt::error!("Too many subpages!");
            return;
        }

        // TODO: check we are idle
        self.state = BootLoadState::BootloadEnabled {
            ttl_subpages: subpages as usize,
            checksum,
        };
        self.map.pages.iter_mut().for_each(|b| {
            *b = 0;
        });

        defmt::info!("Unlocked Bootloading; pages: {=u8}, checksum: 0x{=u32:X}", subpages, checksum);
    }

    fn dummy_read(&mut self) {
        defmt::info!("Dummy read!");
    }

    fn dummy_write(&mut self, len: usize) {
        let good_buf = &self.buffer[..len];
        defmt::info!("Dummy write of {:?} bytes!", len);
    }

    // TODO: not bool
    fn process_stop(&mut self) -> bool {
        let i2cpac = self.i2c.borrow_pac();

        let isr = i2cpac.isr.read();

        // Is the controller asking us for more data still?
        if isr.txis().bit_is_set() {
            self.nak();
            defmt::error!("asked for more read when stop expected!");
            i2cpac.txdr.modify(|_, w| {
                unsafe {
                    w.txdata().bits(0)
                }
            });
        }

        // Is the controller giving us more data still?
        if isr.rxne().bit_is_clear() {
            self.nak();
            defmt::error!("got write when stop expected!");
            let _ = i2cpac.rxdr.read().rxdata().bits();
        }

        // Is the controller finally done?
        if i2cpac.isr.read().stopf().bit_is_set() {
            i2cpac.icr.write(|w| {
                w.stopcf().set_bit()
            });
            defmt::info!("got stop.");
            true
        } else {
            false
        }
    }

    fn process_write(&mut self, addr: u8, len: usize, mut idx: usize) -> Transfer {
        if idx >= len {
            panic!("Why are you writing more")
        }

        let i2cpac = self.i2c.borrow_pac();

        if i2cpac.isr.read().rxne().bit_is_clear() {
            return Transfer::Writing { addr, len, idx };
        }

        self.buffer[idx] = i2cpac.rxdr.read().rxdata().bits();
        idx += 1;

        // Some kind of data race... when this line is uncommented
        // it causes the host to get a NAK??? But this line is never hit
        //
        // TODO: wat
        //
        // defmt::info!("Wrote byte {:?}/{:?}", idx, len);

        if idx >= len {
            defmt::info!("Done! Waiting for stop");
            self.waiting_for_stop = true;
            self.nak();
            if let Some(cb) = self.on_write.take() {
                cb(self, len);
            }
            Transfer::Idle
        } else {
            Transfer::Writing { addr, len, idx }
        }
    }

    fn process_read(&mut self, addr: u8, len: usize, mut idx: usize) -> Transfer {
        let i2cpac = self.i2c.borrow_pac();

        if i2cpac.isr.read().txis().bit_is_clear() {
            return Transfer::Reading { addr, len, idx };
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
        defmt::info!("Wrote byte to be read idx: {:?}/{:?}", idx, len);

        if idx == len {
            defmt::info!("Done! Waiting for stop");
            self.nak();
            self.waiting_for_stop = true;

            if let Some(cb) = self.on_read.take() {
                cb(self);
            }

            Transfer::Idle
        } else {
            Transfer::Reading { addr, len, idx }
        }
    }

    fn start_txfr(&mut self, addr: u8) -> Option<Transfer> {
        match addr {
            // * Write transactions:

            // * 0x40 - Start bootload
            //     * 4 bytes - total checksum (later crc32), little endian
            //     * 1 byte - total subpages to write
            //         * NOTE: must be less than 23 * 8 for now
            0x40 => {
                defmt::info!("Start Bootload");
                self.on_write = Some(Self::activate_bootload);
                Some(Transfer::Writing {
                    addr,
                    len: 5,
                    idx: 0,
                })
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
                defmt::info!("Write Page");
                self.on_write = Some(Self::dummy_write);
                Some(Transfer::Writing {
                    addr,
                    len: 1 + 256 + 4,
                    idx: 0,
                })
            }

            //     * 0x42 - reboot to new image
            //     * 0x43 - abort bootload
            //         * No data
            //     * 0x44 - Offer image downstream
            //         * Only after finishing 0x40 transaction


            // * Read transactions
            //     * wr-then-rd 0x10 + 16 bytes => b'sprocket boot!!!'
            0x10 => {
                const ID: &[u8] = b"sprocket boot!!!";
                defmt::info!("Got 0x10 write, going to read");
                (&mut self.buffer[..ID.len()]).copy_from_slice(ID);
                self.waiting_for_stop = true;
                self.on_read = Some(Self::dummy_read);
                // let i2cpac = self.i2c.borrow_pac();
                // i2cpac.cr2.modify(|_, w| unsafe {
                //     // TODO(AJM): handle writes longer than 255
                //     w.nbytes().bits(ID.len() as u8)
                //     .reload().clear_bit()
                // });

                Some(Transfer::WaitReadStart {
                    addr,
                    len: ID.len(),
                    idx: 0,
                })
            }
            //     * wr-then-rd 0x11 + 4 bytes => maj.min.triv.reserved
            //     * wr-then-rd [0x21, P:S] + 260 bytes => Read subpage
            //     * wr-then-rd 0x22 + 1 byte => status
            //     * wr-then-rd 0x23 + 4 bytes => children flashed
            _ => None
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
            defmt::error!("Address Mismatch!");
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
            defmt::error!("Direction Mismatch!");
            false
        } else {
            // Clear a NAK
            i2cpac.cr2.write(|w| {
                w.nack().clear_bit()
            });

            defmt::info!("Acked a correct address+direction");
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
