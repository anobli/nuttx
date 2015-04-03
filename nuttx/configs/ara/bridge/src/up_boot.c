#include <nuttx/config.h>
#include <arch/tsb/unipro.h>
#include <debug.h>

#ifdef CONFIG_ARCH_LOWPUTC
#define early_dbg(fmt, ...) lowsyslog(fmt, ##__VA_ARGS__)
#endif

#ifndef ARA_FW_VERSION
#define ARA_FW_VERSION "unknown"
#endif

#ifndef ARA_FW_BUILD_TIME
#define ARA_FW_BUILD_TIME "unknown"
#endif

void tsb_boardinitialize(void) {
    early_dbg("\n********************************************************************************\n");
    early_dbg("Ara Bridge FW version: %s\n", ARA_FW_VERSION);
    early_dbg("Build: %s\n", ARA_FW_BUILD_TIME);
    early_dbg("Chip Revision: %s\n", CONFIG_TSB_CHIP_REV);

#if defined CONFIG_BOOT_COPYTORAM
    early_dbg("Boot type: SPI flash copy to RAM\n");
#elif defined CONFIG_BOOT_RUNFROMISRAM
    early_dbg("Boot type: Internal SRAM\n");
#else
#error "Unsupported configuration"

#endif
    early_dbg("********************************************************************************\n");
}

