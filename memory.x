MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM_FLAGS: ORIGIN = 0x20000000, LENGTH = 128
  RAM : ORIGIN = 0x20000080, LENGTH = (8K - 128)
}

/* TODO: Can I just have overlapping memory regions?    */
/*                                                      */
/* NOTE: Make sure the size of the app is in sync with  */
/* the number of pages defined in consts.rs!            */

_app_start = ORIGIN(FLASH);
_settings_start = (ORIGIN(FLASH) + 58K);
_bootloader_start = (ORIGIN(FLASH) + 60K);
_stext = _bootloader_start;
_ram_flags = ORIGIN(RAM_FLAGS);
