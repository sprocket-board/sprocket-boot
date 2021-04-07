#![no_std]

// TODO(5) adjust HAL import
use stm32g0xx_hal as _; // memory layout

#[macro_use] pub mod log;
pub mod boot_machine;
pub mod page_map;
pub mod consts;

#[cfg(feature = "defmt-log")]
mod defmt {
    use core::sync::atomic::{AtomicUsize, Ordering};
    use defmt_rtt as _; // global logger
    use panic_probe as _;

    // same panicking *behavior* as `panic-probe` but doesn't print a panic message
    // this prevents the panic message being printed *twice* when `defmt::panic` is invoked
    #[defmt::panic_handler]
    fn panic() -> ! {
        cortex_m::asm::udf()
    }

    static COUNT: AtomicUsize = AtomicUsize::new(0);

    defmt::timestamp!("{=usize}", {
        // NOTE(no-CAS) `timestamps` runs with interrupts disabled
        let n = COUNT.load(Ordering::Relaxed);
        COUNT.store(n + 1, Ordering::Relaxed);
        n
    });
}

#[cfg(feature = "panic-reset")]
use panic_reset as _;

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
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
