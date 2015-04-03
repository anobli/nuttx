MEMORY
{
    sram (rwx) : ORIGIN = 0x10000000, LENGTH = 192K
}

REGION_ALIAS("REGION_TEXT", sram);

INCLUDE tsb_common.ld

