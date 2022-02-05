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

use cortex_m::peripheral::SCB;
use sprocket_boot::{
    self as _, // global logger + panicking-behavior + memory layout
    sprkt_log,
    boot_machine::BootMachine,
    consts::{
        BootCommand,
        FLASH_MAGIC_WORD,
        RAM_MAGIC_WORD,
    },
};

use stm32g0xx_hal::i2c_periph::I2CPeripheral;
use stm32g0xx_hal::{
    flash::Read,
    prelude::*,
    rcc::{Config, PllConfig, Prescaler},
    stm32,
};

use cassette::{
    Cassette,
    pin_mut,
};

enum BootDecision {
    ForceBootload,
    BootApp,
}

fn inner_main() -> Result<(), ()> {
    //
    // Step 0: Start the boot decision process
    //
    cortex_m::interrupt::disable();
    let mut decision: Option<BootDecision> = None;

    //
    // Step 1: Check RAM Flags
    //
    {
        extern "C" {
            static _ram_flags: u8;
        }

        //
        // 1.1: Copy RAM command page to a local buffer, and clear them after
        // a read.
        //

        let mut buf = [0u8; 128];

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        let ram_flags_addr: *mut u8 = unsafe { &_ram_flags as *const _ as *mut _ };
        let buffer_addr: *mut u8 = buf.as_mut_ptr();
        sprkt_log!(info, "Loading ram page at {=u32:X}", ram_flags_addr as u32);

        unsafe {
            core::ptr::copy_nonoverlapping(ram_flags_addr, buffer_addr, 128);

            // Overwrite magic word
            ram_flags_addr.write_bytes(0x00, 4);
        }

        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

        //
        // Step 1.2: Check magic word. If it is set, check the boot command flag
        //
        if &RAM_MAGIC_WORD.to_le_bytes() != &buf[..4] {
            sprkt_log!(info, "No good RAM data.");
        } else {
            match buf[4].into() {
                BootCommand::StayInBootloader => {
                    sprkt_log!(info, "RAM: Stay in bootloader");
                    decision = Some(BootDecision::ForceBootload);
                }
                BootCommand::LoadApp => {
                    // A one means BOOT TO APP
                    sprkt_log!(info, "RAM: Attempt boot");
                    decision = Some(BootDecision::BootApp);
                }
                _ => {
                    sprkt_log!(warn, "RAM: Invalid!");
                    // Don't trust invalid variants. Keep the decision None.
                    //
                    // TODO: We could also treat this as "force bootloader"? Not
                    // sure which is a better default
                }
            }
        }
    }

    //
    // Step 2 - Load Flash Settings Page
    //

    // NOTE: Here we take the peripherals, but we DON'T set up RCC, as it
    // is unclear how to reset it without a reboot.
    //
    // TODO: We should probably be restoring/disabling anything we use here.
    let board = stm32::Peripherals::take().ok_or(())?;

    //
    // Step 2.1 - Unlock Flash access
    //

    // TODO: We can probably just do volatile reads without using the flash
    // peripheral at all.
    let flash = if let Ok(ulf) = board.FLASH.unlock() {
        sprkt_log!(info, "unlocked.");
        ulf
    } else {
        sprkt_log!(error, "Unlock failed!");

        // Delay a little time so we don't reboot TOO fast
        cortex_m::asm::delay(8_000_000);
        sprocket_boot::exit()
    };

    //
    // Step 2.2 - Load Flash Settings Page
    //
    let mut i2c_addr = 0x69;
    let settings_msp_rsv: Option<(u32, u32)> = {
        extern "C" {
            static _settings_start: u32;
        }

        let settings_start = unsafe { &_settings_start as *const _ as usize };

        let mut settings_bytes = [0u8; 13];

        flash.read(settings_start, &mut settings_bytes);
        let (magic_bytes_sl, remainder) = settings_bytes.split_at(4);
        let (msp_bytes_sl, remainder) = remainder.split_at(4);
        let (rsv_bytes_sl, i2c_byte_sl) = remainder.split_at(4);

        let mut magic_bytes = [0u8; 4];
        let mut msp_bytes = [0u8; 4];
        let mut rsv_bytes = [0u8; 4];

        magic_bytes.copy_from_slice(magic_bytes_sl);
        msp_bytes.copy_from_slice(msp_bytes_sl);
        rsv_bytes.copy_from_slice(rsv_bytes_sl);

        let magic = u32::from_le_bytes(magic_bytes);
        let settings_msp = u32::from_le_bytes(msp_bytes);
        let settings_rsv = u32::from_le_bytes(rsv_bytes);

        let good_magic = magic == FLASH_MAGIC_WORD;
        let good_msp = settings_msp != 0xFFFFFFFFu32;
        let good_rsv = settings_rsv != 0xFFFFFFFFu32;

        if good_magic && good_msp && good_rsv {
            sprkt_log!(info, "Good Settings data");
            i2c_addr = i2c_byte_sl[0];
            Some((settings_msp, settings_rsv))
        } else {
            sprkt_log!(warn, "Bad Settings data");
            decision = Some(BootDecision::ForceBootload);
            None
        }
    };

    //
    // Step 2.2 - Load App Vector Table
    //
    let app_msp_rsv: (u32, u32) = {
        // Read App Vector Table
        extern "C" {
            static _app_start: u32;
        }

        let app_start = unsafe { &_app_start as *const _ as usize };

        let mut app_bytes = [0u8; 8];

        flash.read(app_start, &mut app_bytes);
        let (app_msp_bytes_sl, app_rsv_bytes_sl) = app_bytes.split_at(4);

        let mut app_msp_bytes = [0u8; 4];
        let mut app_rsv_bytes = [0u8; 4];

        app_msp_bytes.copy_from_slice(app_msp_bytes_sl);
        app_rsv_bytes.copy_from_slice(app_rsv_bytes_sl);

        let app_msp = u32::from_le_bytes(app_msp_bytes);
        let app_rsv = u32::from_le_bytes(app_rsv_bytes);

        (app_msp, app_rsv)
    };

    //
    // Step 2.3 - Load Bootloader (our own) link time position information
    //
    let boot_msp_rsv: (u32, u32) = {
        extern "C" {
            // TODO: This goes away to Reset in newer version of `cortex-m`
            static PreResetTrampoline: u32;
            static _stack_start: u32;
        }

        let boot_rsv = unsafe { &PreResetTrampoline as *const _ as u32 };
        let boot_msp = unsafe { &_stack_start as *const _ as u32 };

        (boot_msp, boot_rsv)
    };

    //
    // Step 2.4 - Cross check for early boot
    //
    if let Some((stg_msp, stg_rsv)) = settings_msp_rsv {
        if let Some(BootDecision::BootApp) = decision {
            //
            // Step 2.5 - We have intent to boot. Final checks and bootload
            //
            let good_app_msp = app_msp_rsv.0 == boot_msp_rsv.0;
            let good_app_rsv = app_msp_rsv.1 == boot_msp_rsv.1;
            let good_stg_msp = stg_msp != 0xFFFFFFFF;
            let good_stg_rsv = stg_rsv != 0xFFFFFFFF;

            if good_app_msp && good_app_rsv && good_stg_msp && good_stg_rsv {
                sprkt_log!(info, "bootloading!");
                sprkt_log!(info, "MSP: {=u32:X}", stg_msp);
                sprkt_log!(info, "RSV: {=u32:X}", stg_rsv);

                unsafe {
                    cortex_m::asm::bootstrap(
                        stg_msp as *const u32,
                        stg_rsv as *const u32
                    );
                }
            } else {
                sprkt_log!(info, "Cross Check of values failed. Bootloading.");
                decision = Some(BootDecision::ForceBootload);
            }
        }
    } else {
        sprkt_log!(info, "Bad Settings. Bootloading.");
        decision = Some(BootDecision::ForceBootload);
    }

    //
    // Step 3.0 - Power on PLL clocks- Full 64MHz
    //
    let config = Config::pll()
        .pll_cfg(PllConfig::with_hsi(1, 8, 2))
        .ahb_psc(Prescaler::NotDivided)
        .apb_psc(Prescaler::NotDivided);
    let mut rcc = board.RCC.freeze(config);

    let gpioc = board.GPIOC.split(&mut rcc);
    let button1 = gpioc.pc14.into_floating_input();
    let button2 = gpioc.pc15.into_floating_input();

    //
    // Step 3.1 - Check buttons if we aren't sure about what to do yet
    //
    if decision.is_none() {
        sprkt_log!(info, "Decision unclear, checking buttons.");
        // Are both buttons held, signaling "stay in bootloader"?
        let buttons_held = (Ok(true), Ok(true)) == (button1.is_low(), button2.is_low());

        extern "C" {
            static _ram_flags: u8;
        }

        let ram_flags_addr: *mut u8 = unsafe { &_ram_flags as *const _ as *mut _ };

        // Nope!
        if !buttons_held {
            sprkt_log!(info, "No buttons, reboot with commanded boot to app");
            unsafe {
                // HACK: reboot to clear PLLs if buttons are pressed
                ram_flags_addr.cast::<u32>().write(RAM_MAGIC_WORD);

                // force boot to app
                ram_flags_addr.add(4).write(BootCommand::LoadApp as u8);
                SCB::sys_reset()
            }
        } else {
            sprkt_log!(info, "Buttons pressed. Bootloading.");
        }
    }

    //
    // Step 4.0 - Start bootloader. At this point we are unsure, or know we
    // must be bootloading, either way, let's bootload.
    //
    let gpioa = board.GPIOA.split(&mut rcc);
    let gpiob = board.GPIOB.split(&mut rcc);

    let i2c1_scl = gpiob.pb6; // note: shadows pa9
    let i2c1_sda = gpiob.pb7; // note: shadows pa10
    let mut led1 = gpioa.pa0.into_push_pull_output();
    let mut led2 = gpiob.pb8.into_push_pull_output();
    led1.set_low().ok();
    led2.set_low().ok();

    let i2c = I2CPeripheral::new(
        board.I2C1,
        i2c1_sda,
        i2c1_scl,
        &mut rcc,
        i2c_addr
    );

    sprkt_log!(info, "Launching bootloader!");

    //
    // Step 5 - Hand control over to BootMachine for sequencing of tasks
    //
    let mut boot = BootMachine::new(i2c, flash, led1, led2, i2c_addr);
    let x = boot.entry();

    pin_mut!(x);

    let mut machine = Cassette::new(x);

    loop {
        if let Some(_) = machine.poll_on() {
            sprkt_log!(warn, "Machine is done!");
            break;
        }
    }

    // Oh no, something has gone wrong.
    sprkt_log!(error, "Something has gone awry");
    // for _ in 0..10 {
    //     boot.led1.set_high().ok();
    //     boot.led2.set_low().ok();

    //     cortex_m::asm::delay(64_000_000 / 4);

    //     boot.led1.set_low().ok();
    //     boot.led2.set_high().ok();

    //     cortex_m::asm::delay(64_000_000 / 4);
    // }

    Err(())
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let _ = inner_main();
    SCB::sys_reset()
}
