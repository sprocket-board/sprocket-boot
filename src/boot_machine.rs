use cortex_m::peripheral::SCB;
use crate::{
    self as _, // global logger + panicking-behavior + memory layout
    exit as yeet,
    consts::{
        SKIP_FLASH,
        SUBPAGES_PER_PAGE,
        TOTAL_SUBPAGES,
        TOTAL_PAGES,
        FLASH_MAGIC_WORD,
    },
    page_map::PageMap,
    generate_checksum,
};

use stm32g0xx_hal::i2c_periph::{
    I2CPeripheral,
    Instance
};
use stm32g0xx_hal::{
    flash::{self, FlashPage, Read, UnlockedFlash, WriteErase, Error as FlashError},
    gpio::{
        gpioa::PA0,
        gpiob::PB8,
        Output, PushPull,
    },
    prelude::*,
};

pub struct BootMachine<P: Instance> {
    i2c: I2CPeripheral<P>,
    current_i2c_addr: u8,
    transfer: Transfer,
    buffer: [u8; 512],
    flash: UnlockedFlash,
    map: PageMap,
    sq: StateQueue<Self>,
    flash_action: BootLoadAction,
    pub led1: PA0<Output<PushPull>>,
    pub led2: PB8<Output<PushPull>>,
    new_i2c_addr: Option<u8>,
}

#[derive(Debug, Eq, PartialEq)]
enum BootLoadAction {
    Idle,
    EraseThenWrite { page: usize, subpage: u8 },
    Writing { page: usize, subpage: u8 },
}

enum Transfer {
    Idle,
    Writing { addr: u8, len: usize, idx: usize },
    Reading { addr: u8, len: usize, idx: usize },
}

#[derive(Debug, PartialEq, Eq)]
enum TransferDir {
    Read,
    Write,
}

type QueueFunc<T> = fn(&mut T) -> Result<bool, ()>;

struct StateQueueItem<T>(QueueFunc<T>);

impl<T> Clone for StateQueueItem<T> {
    fn clone(&self) -> Self {
        StateQueueItem(self.0)
    }
}

impl<T> Default for StateQueueItem<T> {
    fn default() -> Self {
        Self(nop_state_err)
    }
}

fn nop_state_err<T, R>(_: &mut T) -> Result<R, ()> {
    sprkt_log!(error, "Nop state error!");
    Err(())
}

const ITEMS: usize = 8;

struct StateQueue<T> {
    items: [StateQueueItem<T>; ITEMS],
    idx: usize,
}

impl<T> StateQueue<T> {
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
            ],
        }
    }

    fn from_slice(items: &[StateQueueItem<T>]) -> Self {
        // assert!(items.len() <= 8, "Increase Queue Size!");

        let mut new = Self::new();
        new.idx = 0;
        items.iter().rev().take(ITEMS).for_each(|i| {
            new.items[new.idx] = (*i).clone();
            new.idx += 1;
        });
        new
    }

    fn pop(&mut self) -> Option<StateQueueItem<T>> {
        if self.idx == 0 {
            None
        } else {
            let mut replace = StateQueueItem::default();
            core::mem::swap(&mut replace, &mut self.items[self.idx - 1]);
            self.idx -= 1;
            Some(replace)
        }
    }

    fn push(&mut self, item: StateQueueItem<T>) -> Result<(), ()> {
        if self.idx >= self.items.len() {
            sprkt_log!(error, "Push fail!");
            return Err(());
        }

        self.items[self.idx] = item;
        self.idx += 1;
        Ok(())
    }
}

// This is the main interface for bootmachine. You basically create it
// and poll it until it produces an error.
impl<P: Instance> BootMachine<P> {
    pub fn new(
        i2c: I2CPeripheral<P>,
        flash: UnlockedFlash,
        led1: PA0<Output<PushPull>>,
        led2: PB8<Output<PushPull>>,
        addr: u8,
    ) -> Self {
        Self {
            i2c,
            flash,
            buffer: [0u8; 512],
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
            new_i2c_addr: None,
            current_i2c_addr: addr,
        }
    }

    pub fn poll(&mut self) -> Result<(), ()> {
        match self.poll_inner() {
            Ok(()) => Ok(()),
            Err(()) => {
                todo!("clear the queue, go into some kind of recovery?");
            }
        }
    }

    fn poll_inner(&mut self) -> Result<(), ()> {
        let action = self.sq.pop().ok_or(())?;

        if !(action.0)(self)? {
            let x = self.sq.push(action);
            if x.is_err() {
                sprkt_log!(error, "Queue is full???");
            }

            x?;
        }

        Ok(())
    }
}

// These are the scripted actions that can be used as building blocks
// for the bootmachine. They all have the same signature, which means:
//
// Ok(true)  - Task is complete, move on to the next
// Ok(false) - Task is still working, continue to call
// Err(_)    - Task has failed. For now, abort. In the future,
//   we might provide a recovery script instead
impl<P: Instance> BootMachine<P> {
    fn complete_write(&mut self) -> Result<bool, ()> {
        let (addr, mut idx, len) = if let Transfer::Writing { addr, len, idx } = &self.transfer {
            (*addr, *idx, *len)
        } else {
            sprkt_log!(error, "Wrong state for complete_write!");
            return Err(());
        };

        assert!(idx < len, "Why are you writing more");

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
        // sprkt_log!(info, "Wrote byte {:?}/{:?}", idx, len);

        if idx >= len {
            sprkt_log!(info, "Done! Waiting for stop");
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
            sprkt_log!(error, "wrong state!");
            return Err(());
        };

        let i2cpac = self.i2c.borrow_pac();

        if i2cpac.isr.read().txis().bit_is_clear() {
            // Not done
            return Ok(false);
        }

        assert!(idx < len, "Bounds checking failure");

        i2cpac
            .txdr
            .modify(|_, w| unsafe { w.txdata().bits(self.buffer[idx]) });
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
            sprkt_log!(error, "asked for more read when stop expected!");
            i2cpac.txdr.modify(|_, w| unsafe { w.txdata().bits(0) });
        }

        // Is the controller giving us more data still?
        if isr.rxne().bit_is_clear() {
            self.nak();
            sprkt_log!(error, "got write when stop expected!");
            let _ = i2cpac.rxdr.read().rxdata().bits();
        }

        // Is the controller finally done?
        if i2cpac.isr.read().stopf().bit_is_set() {
            i2cpac.icr.write(|w| w.stopcf().set_bit());
            sprkt_log!(info, "got stop.");
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
        // Expect a read match
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

    fn erase_page(&mut self) -> Result<bool, ()> {
        let mut old_action = BootLoadAction::Idle;
        core::mem::swap(&mut old_action, &mut self.flash_action);

        let (page, subpage) = if let BootLoadAction::EraseThenWrite { page, subpage } = old_action {
            (page, subpage)
        } else {
            sprkt_log!(error, "Tried to erase when not prepared!");
            return Err(());
        };

        sprkt_log!(info, "erasing page {=u32}...", page as u32);

        if !SKIP_FLASH {
            if let Err(e) = self.flash.erase_page(FlashPage(page)) {
                sprkt_log!(error, "Erase failed:");

                match e {
                    FlashError::Busy => sprkt_log!(info, "busy."),
                    FlashError::Illegal => sprkt_log!(info, "illegal."),
                    FlashError::EccError => sprkt_log!(info, "ecc err."),
                    FlashError::PageOutOfRange => sprkt_log!(info, "page oor."),
                    FlashError::Failure => sprkt_log!(info, "failure."),
                };

                return Err(());
            }
        } else {
            sprkt_log!(warn, "Skipped actual erase! Pretend it's good.");
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
            sprkt_log!(error, "Tried to write when not prepared!");
            return Err(());
        };

        let subpage_image = &self.buffer[1..257];

        if subpage_image.iter().all(|b| *b == 0xFF) {
            sprkt_log!(info, "Page is all 0xFF, skipping write");
            return Ok(true);
        }

        extern "C" {
            static _app_start: u32;
        }

        let app_start = unsafe { &_app_start as *const _ as usize };

        // NOTE: app_start is already offset to the base of the temporary flash region
        let store_addr =
            (app_start + (page * flash::PAGE_SIZE as usize)) + (subpage as usize * 256);

        sprkt_log!(info, "Writing subpage at {=u32:X}...", store_addr as u32);

        if !SKIP_FLASH {
            self.flash.write(store_addr, subpage_image).map_err(|_| {
                sprkt_log!(error, "Write failed!");
            })?;
        } else {
            sprkt_log!(warn, "Skipped actual write! Pretend it's good.");
            sprkt_log!(info, "page: {:?}", subpage_image);
        }

        Ok(true)
    }

    fn enable_i2c(&mut self) -> Result<bool, ()> {
        if self.i2c.is_enabled() {
            sprkt_log!(error, "Oops, expected to be disabled but we're not");
            return Err(());
        }
        unsafe {
            self.i2c.enable();
        }
        Ok(true)
    }

    fn disable_i2c(&mut self) -> Result<bool, ()> {
        if !self.i2c.is_enabled() {
            sprkt_log!(error, "Oops, expected to be enabled but we're not");
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
            sprkt_log!(error, "Bootloading not started!");
            return Err(());
        }

        let len = if let Transfer::Writing { len, .. } = &self.transfer {
            *len
        } else {
            sprkt_log!(error, "Wrong state for check_write_page_data!");
            return Err(());
        };

        if len != (1 + 256 + 4) {
            sprkt_log!(error, "Wrong number of bytes!");
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
            sprkt_log!(error, "Checksum mismatch!");
            return Err(());
        }

        if page == 0 && subpage == 0 {
            extern "C" {
                static PreResetTrampoline: u32;
                static _stack_start: u32;
            }

            if !self.map.ready_to_write_vector_subpage() {
                sprkt_log!(error, "Must write page 0 subpage 0 last!");
                return Err(());
            }

            let reset_vector = unsafe { &PreResetTrampoline as *const _ as usize };
            let msp = unsafe { &_stack_start as *const _ as usize };

            sprkt_log!(info, "Patching reset vector to the bootloader's: {=usize:X}", reset_vector);
            sprkt_log!(info, "Patching MSP to the bootloader's: {=usize:X}", msp);

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
            sprkt_log!(info, "Subpage accepted. Erase + Flash started");
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

            self.flash_action = BootLoadAction::EraseThenWrite { page, subpage };
            Ok(true)
        } else if (*map & 1 << subpage) == 0 {
            sprkt_log!(info, "Subpage accepted. Flash started");
            *map = *map | (1 << subpage);

            self.sq = StateQueue::from_slice(&[
                StateQueueItem(BootMachine::wait_for_stop),
                StateQueueItem(BootMachine::disable_i2c),
                StateQueueItem(BootMachine::write_subpage),
                StateQueueItem(BootMachine::enable_i2c),
                StateQueueItem(BootMachine::match_address_write),
                StateQueueItem(BootMachine::match_write_register),
            ]);

            self.flash_action = BootLoadAction::Writing { page, subpage };
            Ok(true)
        } else {
            sprkt_log!(error, "Can't re-write pages!");
            Err(())
        }
    }

    fn activate_bootload(&mut self) -> Result<bool, ()> {
        let len = if let Transfer::Writing { len, .. } = self.transfer {
            len
        } else {
            sprkt_log!(error, "Wrong state!");
            return Err(());
        };

        if len != 5 {
            sprkt_log!(error, "Wrong number of bytes!");
            return Err(());
        }

        let mut checksum_bytes = [0u8; 4];
        checksum_bytes.copy_from_slice(&self.buffer[..4]);

        // TODO: Use global checksum!
        let _checksum = u32::from_le_bytes(checksum_bytes);
        let subpages = self.buffer[4];

        if subpages as usize > TOTAL_SUBPAGES {
            sprkt_log!(error, "Too many subpages!");
            return Err(());
        }

        if subpages % (SUBPAGES_PER_PAGE as u8) != 0 {
            sprkt_log!(error, "Must provide whole page!");
            return Err(());
        }

        let pages = (subpages >> 3) as usize;

        if self.map.unlocked {
            sprkt_log!(warn, "Already unlocked, resetting programming!");
        }

        self.map.pages.iter_mut().for_each(|b| {
            *b = 0;
        });

        self.map.unlocked = true;
        self.map.active_pages = pages;

        sprkt_log!(info, "Unlocked Bootloading; pages: {=u8}, checksum: {=u32:X}", subpages, _checksum);

        Ok(true)
    }

    fn write_settings_page(&mut self) -> Result<bool, ()> {
        extern "C" {
            static _settings_start: u32;
        }

        let settings_start = unsafe { &_settings_start as *const _ as usize };

        sprkt_log!(info, "Reading settings page...");

        if !SKIP_FLASH {
            self.flash.read(settings_start, &mut self.buffer[..256]);
        } else {
            sprkt_log!(warn, "Skipping real settings read! Loading all 0xFFs");
            self.buffer[..256].iter_mut().for_each(|b| *b = 0xFF);
        }

        // TODO: actual serialization/deserialization of settings page
        sprkt_log!(warn, "Manually applying new reset vector and msp to settings page");

        sprkt_log!(info, "App MSP: {=usize:X}", self.map.msp);
        sprkt_log!(info, "App RsV: {=usize:X}", self.map.reset_vector);

        self.buffer[..4].copy_from_slice(&FLASH_MAGIC_WORD.to_le_bytes());
        self.buffer[4..8].copy_from_slice(&self.map.msp.to_le_bytes());
        self.buffer[8..12].copy_from_slice(&self.map.reset_vector.to_le_bytes());
        if let Some(addr) = self.new_i2c_addr.take() {
            sprkt_log!(info, "Updating i2c: from {=u8:X} to {=u8:X}", self.buffer[12], addr);
            self.buffer[12] = addr;
        } else {
            self.buffer[12] = self.current_i2c_addr;
        }

        // TODO: De-duplicate this with erase page and write page. Right now it's
        // hardcoded to the offset write section, instead of the total range.

        // TODO: not magic numbers
        let real_page = (settings_start - 0x0800_0000) / 2048;

        if !SKIP_FLASH {
            if let Err(e) = self.flash.erase_page(FlashPage(real_page)) {
                sprkt_log!(error, "Erase failed:");

                match e {
                    FlashError::Busy => sprkt_log!(info, "busy."),
                    FlashError::Illegal => sprkt_log!(info, "illegal."),
                    FlashError::EccError => sprkt_log!(info, "ecc err."),
                    FlashError::PageOutOfRange => sprkt_log!(info, "page oor."),
                    FlashError::Failure => sprkt_log!(info, "failure."),
                }

                return Err(());
            }
        } else {
            sprkt_log!(warn, "Skipped actual settings erase! Pretend it's good.");
        }

        if !SKIP_FLASH {
            self.flash
                .write(settings_start, &self.buffer[..256])
                .map_err(|_| {
                    sprkt_log!(error, "Write failed!");
                })?;
        } else {
            sprkt_log!(warn, "Skipped actual settings write! Pretend it's good.");
        }

        Ok(true)
    }

    fn reboot(&mut self) -> Result<bool, ()> {
        if !SKIP_FLASH {
            // TODO: Re-lock flash?
            // TODO: Clear RAM flags?
            // TODO: Clear interrupts?
            for _ in 0..8 {
                self.led1.set_low().ok();
                self.led2.set_high().ok();

                cortex_m::asm::delay(64_000_000 / 16);

                self.led1.set_high().ok();
                self.led2.set_low().ok();

                cortex_m::asm::delay(64_000_000 / 16);
            }

            SCB::sys_reset()
        } else {
            panic!("Bootload complete!");
        }
    }

    fn set_i2c(&mut self) -> Result<bool, ()> {
        self.new_i2c_addr = Some(self.buffer[1]);
        Ok(true)
    }

    fn start_txfr(&mut self, addr: u8) -> StateQueue<Self> {
        match addr {
            // * Write transactions:

            // * 0x40 - Start bootload
            //     * 4 bytes - total checksum (later crc32), little endian
            //     * 1 byte - total subpages to write
            //         * NOTE: must be less than 23 * 8 for now
            0x40 => {
                sprkt_log!(info, "Start Bootload");
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
                sprkt_log!(info, "Write Page");
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
                sprkt_log!(info, "Complete and Reboot");
                if !self.map.bootload_complete() {
                    sprkt_log!(error, "Bootload not complete! Not rebooting.");

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
            //     * 0x45 - new I2C address
            //         * 1 byte - 7-bit i2c addr
            0x45 => {
                sprkt_log!(info, "Start I2C addr");
                self.transfer = Transfer::Writing {
                    addr,
                    len: 1,
                    idx: 0,
                };
                StateQueue::from_slice(&[
                    StateQueueItem(BootMachine::complete_write),
                    StateQueueItem(BootMachine::set_i2c),
                    StateQueueItem(BootMachine::wait_for_stop),
                    StateQueueItem(BootMachine::match_address_write),
                    StateQueueItem(BootMachine::match_write_register),
                ])
            }

            // * Read transactions
            //     * wr-then-rd 0x10 + 16 bytes => b'sprocket boot!!!'
            0x10 => {
                const ID: &[u8] = b"sprocket boot!!!";
                sprkt_log!(info, "Got 0x10 write, going to read");

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
                sprkt_log!(info, "Unexpected command: {=u8}", addr);
                yeet()
            }
        }
    }

    fn check_addr_match(&self, direction: TransferDir) -> bool {
        let i2cpac = self.i2c.borrow_pac();

        if !i2cpac.isr.read().addr().bit_is_set() {
            return false;
        }

        if i2cpac.isr.read().addcode().bits() != self.current_i2c_addr {
            self.nak();
            self.ack_addr_match();
            sprkt_log!(error, "Address Mismatch!");
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
            sprkt_log!(error, "Direction Mismatch!");
            false
        } else {
            // Clear a NAK
            i2cpac.cr2.write(|w| w.nack().clear_bit());

            sprkt_log!(info, "Acked a correct address+direction");
            if direction == TransferDir::Read {
                i2cpac.isr.write(|w| w.txe().set_bit());
            }
            self.ack_addr_match();
            true
        }
    }

    fn ack_addr_match(&self) {
        let i2cpac = self.i2c.borrow_pac();

        // Clear the ADDR match flag
        i2cpac.icr.write(|w| w.addrcf().set_bit());
    }

    fn nak(&self) {
        let i2cpac = self.i2c.borrow_pac();

        // Command a NAK
        i2cpac.cr2.write(|w| w.nack().set_bit());
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


// TODO AJM THIS IS A HACK
impl<P: Instance> BootMachine<P> {
    pub async fn abort(&mut self) {
        panic!()
    }
}
