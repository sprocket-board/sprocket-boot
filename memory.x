MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 56K
  STORE: ORIGIN = 0x800E000, LENGTH = 8K
  RAM : ORIGIN = 0x20000000, LENGTH = 8K
}

SECTIONS
{
  PROVIDE(_store_start = ORIGIN(STORE));
}
