MEMORY
{
    rom  (rx)  : ORIGIN = 0x01000000, LENGTH = 2M
    sram (rwx) : ORIGIN = 0x10000000, LENGTH = 192K
}

REGION_ALIAS("REGION_TEXT", rom);

INCLUDE tsb_common.ld

