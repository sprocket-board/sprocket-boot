// #![allow(dead_code)]


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
    Instance,
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
    buffer: [u8; 256],
    flash: UnlockedFlash,
    map: PageMap,
    pub led1: PA0<Output<PushPull>>,
    pub led2: PB8<Output<PushPull>>,
    new_i2c_addr: Option<u8>,
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
            buffer: [0u8; 256],
            map: PageMap {
                pages: [0; TOTAL_PAGES],
                active_pages: 0,
                unlocked: false,
                msp: 0,
                reset_vector: 0,
            },
            led1,
            led2,
            new_i2c_addr: None,
            current_i2c_addr: addr,
        }
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

    async fn match_write_register(&mut self) -> u8 {
        self.i2c.get_written_byte().await
    }


    async fn erase_page(&mut self, page: usize) -> Result<(), ()> {
        sprkt_log!(info, "erasing page {=usize}...", page);

        if !SKIP_FLASH {
            // TODO: This is totally blocking, but it's going to stall
            // the bus anyway, so not much use in yielding
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

        Ok(())
    }


    async fn write_subpage(&mut self, page: usize, subpage: u8) -> Result<(), ()> {
        let subpage_image = &self.buffer[..256];

        if subpage_image.iter().all(|b| *b == 0xFF) {
            sprkt_log!(info, "Page is all 0xFF, skipping write");
            return Ok(());
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
            // TODO: This is totally blocking! But there's not much else to do here
            // while we're waiting, since flash writes stall the bus entirely
            self.flash.write(store_addr, subpage_image).map_err(|_| {
                sprkt_log!(error, "Write failed!");
            })?;
        } else {
            sprkt_log!(warn, "Skipped actual write! Pretend it's good.");
            sprkt_log!(info, "page: {:?}", subpage_image);
        }

        Ok(())
    }

    fn enable_i2c(&mut self) -> Result<(), ()> {
        if self.i2c.is_enabled() {
            sprkt_log!(error, "Oops, expected to be disabled but we're not");
            return Err(());
        }
        unsafe {
            self.i2c.enable();
        }
        Ok(())
    }

    fn disable_i2c(&mut self) -> Result<(), ()> {
        if !self.i2c.is_enabled() {
            sprkt_log!(error, "Oops, expected to be enabled but we're not");
            return Err(());
        }
        unsafe {
            self.i2c.disable();
        }
        Ok(())
    }

    async fn check_write_page_data(&mut self) -> Result<(), ()> {
        // Is bootloading active?
        // TODO: This duplicates the get_page_mut check
        // TODO: How can I make this check at compile time?
        if !self.map.unlocked {
            sprkt_log!(error, "Bootloading not started!");
            return Err(());
        }

        // * 1: Page/Subpage
        //     * 0bPPPPP_SSS (32 pages, 8 sub-pages)
        //     * 0bPPPPP must be <= 23
        // * 256: subpage contents
        // * 4 byte: For now: 32-bit checksum. Later, CRC32 or Poly1305?
        let ps = self.i2c.get_written_byte().await;

        let page = (ps >> 3) as usize;
        let subpage = ps & 0b111;

        assert!(page < TOTAL_PAGES);
        assert!((subpage as usize) < SUBPAGES_PER_PAGE);

        let msg = &mut self.buffer[..256];
        for byte in msg.iter_mut() {
            *byte = self.i2c.get_written_byte().await;
        }

        let mut checksum_unchecked_bytes = [0u8; 4];
        for byte in checksum_unchecked_bytes.iter_mut() {
            *byte = self.i2c.get_written_byte().await;
        }
        let checksum_unchecked = u32::from_le_bytes(checksum_unchecked_bytes);

        let calc_checksum = generate_checksum(msg, None).ok_or(())?;

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

            msp_bytes.copy_from_slice(&msg[..4]);
            reset_vector_bytes.copy_from_slice(&msg[4..8]);

            self.map.reset_vector = usize::from_le_bytes(reset_vector_bytes);
            self.map.msp = usize::from_le_bytes(msp_bytes);

            msg[..4].copy_from_slice(&msp.to_le_bytes());
            msg[4..8].copy_from_slice(&reset_vector.to_le_bytes());
        }

        let map = self.map.get_page_mut(page).ok_or(())?;

        if *map == 0 {
            sprkt_log!(info, "Subpage accepted. Erase + Flash started");
            // All subpages have never been written, time to erase page
            *map = *map | (1 << subpage);

            self.i2c.wait_for_stop().await;

            self.disable_i2c()?;
            self.erase_page(page).await?;
            self.write_subpage(page, subpage).await?;
            self.enable_i2c()?;

            Ok(())
        } else if (*map & 1 << subpage) == 0 {
            sprkt_log!(info, "Subpage accepted. Flash started");
            *map = *map | (1 << subpage);

            self.i2c.wait_for_stop().await;

            self.disable_i2c()?;
            self.write_subpage(page, subpage).await?;
            self.enable_i2c()?;

            Ok(())
        } else {
            sprkt_log!(error, "Can't re-write pages!");
            Err(())
        }
    }


    async fn activate_bootload(&mut self) -> Result<(), ()> {
        let mut checksum_bytes = [0u8; 4];
        for byte in checksum_bytes.iter_mut() {
            *byte = self.i2c.get_written_byte().await;
        }

        // TODO: Use global checksum!
        let _checksum = u32::from_le_bytes(checksum_bytes);
        let subpages = self.i2c.get_written_byte().await;

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

        self.i2c.wait_for_stop().await;

        Ok(())
    }

    fn write_settings_page(&mut self) -> Result<(), ()> {
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

        Ok(())
    }

    fn reboot(&mut self) -> ! {
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

    fn set_i2c_addr(&mut self, addr: u8) {
        self.new_i2c_addr = Some(addr);
    }

    async fn async_start_txfr(&mut self, addr: u8) -> Result<(), ()> {
        match addr {
            // * Write transactions:

            // * 0x40 - Start bootload
            //     * 4 bytes - total checksum (later crc32), little endian
            //     * 1 byte - total subpages to write
            //         * NOTE: must be less than 23 * 8 for now
            0x40 => {
                sprkt_log!(info, "Start Bootload");
                self.activate_bootload().await
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
                self.check_write_page_data().await
            }

            //     * 0x42 - complete and reboot
            0x42 => {
                sprkt_log!(info, "Complete and Reboot");
                if !self.map.bootload_complete() {
                    sprkt_log!(error, "Bootload not complete! Not rebooting.");
                    self.i2c.wait_for_stop().await;
                    // Maybe an error?
                    Ok(())
                } else {
                    self.i2c.wait_for_stop().await;
                    let _ = self.disable_i2c()?;
                    self.write_settings_page()?;

                    // See you later, space cowboy.
                    self.reboot();
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
                let new_addr = self.i2c.get_written_byte().await;
                self.set_i2c_addr(new_addr);
                self.i2c.wait_for_stop().await;
                Ok(())
            }

            // * Read transactions
            //     * wr-then-rd 0x10 + 16 bytes => b'sprocket boot!!!'
            0x10 => {
                const ID: &[u8] = b"sprocket boot!!!";
                sprkt_log!(info, "Got 0x10 write, going to read");
                self.i2c.wait_for_stop().await;
                self.i2c.match_address_read().await;
                for byte in ID.iter() {
                    self.i2c.send_read_byte(*byte).await;
                }
                self.i2c.wait_for_stop().await;

                Ok(())
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


    pub async fn entry(&mut self) -> Result<(), ()> {
        loop {
            self.i2c.match_address_write().await;
            let addr = self.match_write_register().await;
            self.async_start_txfr(addr).await?;
        }
    }
}

