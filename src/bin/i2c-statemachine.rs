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

    let mut boot = BootMachine::new(i2c);

    loop {
        boot.poll().unwrap();
    }


    // loop {
    //     defmt::info!("Waiting for address match...");
    //     let i2cpac = i2c.borrow_pac();

    //     loop {
    //         if i2cpac.isr.read().addr().bit_is_set() {
    //             break;
    //         }
    //     }

    //     defmt::info!("Addr match!");

    //     if i2cpac.isr.read().addcode().bits() != 0x69 {
    //         // panic!("Bad address!");
    //         return Err(());
    //     }

    //     // 0: Write transfer, slave enters receiver mode.
    //     // 1: Read transfer, slave enters transmitter mode.
    //     if i2cpac.isr.read().dir().bit_is_set() {
    //         //
    //         // READ
    //         //

    //         // TODO: What is TXE? "Optional: Set I2C_ISR.TXE = 1"
    //         //
    //         // ISR::TXE
    //         //
    //         // This bit is set by hardware when the I2C_TXDR register is empty. It is cleared when the next
    //         // data to be sent is written in the I2C_TXDR register.
    //         // This bit can be written to ‘1’ by software in order to flush the transmit data register
    //         // I2C_TXDR.

    //         // ACK the address
    //         i2cpac.icr.write(|w| {
    //             w.addrcf().set_bit()
    //         });

    //         defmt::info!("Acknowledged addr");

    //         for (i, byte) in buf.iter().enumerate() {
    //             while i2cpac.isr.read().txis().bit_is_clear() { }

    //             i2cpac.txdr.modify(|_, w| {
    //                 unsafe {
    //                     w.txdata().bits(*byte)
    //                 }
    //             });

    //             defmt::info!("Sent byte {:?}", i);
    //         }

    //         while i2cpac.isr.read().txis().bit_is_clear() { }

    //         defmt::info!("Read complete!");
    //     } else {
    //         //
    //         // WRITE
    //         //

    //         // TODO: What is TXE? "Optional: Set I2C_ISR.TXE = 1"
    //         //
    //         // ISR::TXE
    //         //
    //         // This bit is set by hardware when the I2C_TXDR register is empty. It is cleared when the next
    //         // data to be sent is written in the I2C_TXDR register.
    //         // This bit can be written to ‘1’ by software in order to flush the transmit data register
    //         // I2C_TXDR.

    //         // ACK the address
    //         i2cpac.icr.write(|w| {
    //             w.addrcf().set_bit()
    //         });

    //         defmt::info!("Acknowledged addr");

    //         for (i, byte) in buf.iter_mut().enumerate() {
    //             while i2cpac.isr.read().rxne().bit_is_clear() { }

    //             *byte = i2cpac.rxdr.read().rxdata().bits();
    //             defmt::info!("Received byte {:?}", i);
    //         }

    //         while i2cpac.isr.read().txis().bit_is_clear() { }

    //         defmt::info!("Write complete!");
    //     }
    // }
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

struct BootMachine {
    i2c: I2CPeripheral,
    timer: GlobalRollingTimer,
    transfer: Transfer,
    state: State,
    buffer: [u8; 512],
}

enum State {
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
    fn new(i2c: I2CPeripheral) -> Self {
        Self {
            i2c,
            buffer: [0u8; 512],
            timer: GlobalRollingTimer::new(),
            transfer: Transfer::Idle,
            state: State::Idle,
        }
    }

    fn poll(&mut self) -> Result<(), ()> {
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
                todo!()
            }
            Transfer::Reading { addr, len, idx } => {
                self.process_read(addr, len, idx)
            }
            _ => todo!()
        };

        Ok(())
    }

    fn process_read(&mut self, addr: u8, len: usize, mut idx: usize) -> Transfer {
        let i2cpac = self.i2c.borrow_pac();

        if i2cpac.isr.read().txis().bit_is_clear() {
            return Transfer::Reading { addr, len, idx };
        }

        if idx < len {
            i2cpac.txdr.modify(|_, w| {
                unsafe {
                    w.txdata().bits(self.buffer[idx])
                }
            });
            idx += 1;

            if idx == len {
                Transfer::Idle
            } else {
                Transfer::Reading { addr, len, idx }
            }
        } else {
            defmt::error!("Read Overrun!");
            self.nak();
            Transfer::Idle
        }

    }

    fn start_txfr(&mut self, addr: u8) -> Option<Transfer> {
        match addr {
            // * Write transactions:
            //     * 0x40 - Start bootload
            //         * 4 bytes - total checksum (later crc32), little endian
            //         * 1 byte - total subpages to write
            //             * NOTE: must be less than 23 * 8 for now
            0x40 => {
                Some(Transfer::Writing {
                    addr,
                    len: 5,
                    idx: 0,
                })
            }

            //     * 0x41 - write page command
            //         * Must have set 0x40 already
            //         * Always 1 + 1 + 256 + 4 bytes
            //             * 1: 0x41
            //             * 1: Page/Subpage
            //                 * 0bPPPPP_SSS (32 pages, 8 sub-pages)
            //                 * 0bPPPPP must be <= 23
            //             * 256: subpage contents
            //             * 4 byte: For now: 32-bit checksum. Later, CRC32 or Poly1305?
            //         * On writes to subpage zero: erase page
            //         * Stretch write ack until checksum + erase + write?
            //             * I can't use the flash while erasing (or writing?) anyway
            //         * On writes to 0:0: make sure the reset vector is 0x0800_C000
            //             * Make sure MSP is maxval?
            //             * 0:0 must be the last thing written
            //     * 0x42 - reboot to new image
            //     * 0x43 - abort bootload
            //         * No data
            //     * 0x44 - Offer image downstream
            //         * Only after finishing 0x40 transaction
            // * Read transactions
            //     * wr-then-rd 0x10 + 16 bytes => b'sprocket boot!!!'
            0x10 => {
                const ID: &[u8] = b"sprocket boot!!!";
                (&mut self.buffer[..ID.len()]).copy_from_slice(ID);
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
            defmt::info!("Acked a correct address+direction");
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
