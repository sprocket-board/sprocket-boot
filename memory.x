MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM : ORIGIN = 0x20000000, LENGTH = 8K
}

/* TODO: Can I just have overlapping memory regions? */
_app_start = ORIGIN(FLASH);
_settings_start = (ORIGIN(FLASH) + 48K);
_bootloader_start = (ORIGIN(FLASH) + 50K);
_stext = _bootloader_start;
