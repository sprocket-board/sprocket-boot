pub const SKIP_FLASH: bool = cfg!(feature = "skip-flash");

// NOTE: Make sure this is in sync with the size of the application
// section in memory.x!
pub const TOTAL_PAGES: usize = 29;

// TODO: You REALLY can't change this, because I am bit-packing
pub const SUBPAGES_PER_PAGE: usize = 8;

pub const TOTAL_SUBPAGES: usize = TOTAL_PAGES * SUBPAGES_PER_PAGE;

#[repr(u8)]
#[non_exhaustive]
pub enum BootCommand {
    StayInBootloader = 0,
    LoadApp          = 1,
    _Invalid         = 255,
}

impl From<u8> for BootCommand {
    fn from(other: u8) -> Self {
        match other {
            0 => BootCommand::StayInBootloader,
            1 => BootCommand::LoadApp,
            _ => BootCommand::_Invalid,
        }
    }
}

pub const RAM_MAGIC_WORD: u32 = 0xCAFEB007;
pub const FLASH_MAGIC_WORD: u32 = 0xB007CAFE;
