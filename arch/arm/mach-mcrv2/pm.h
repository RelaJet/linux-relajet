#include <asm/cacheflush.h>
#include <mach/mmp_register.h>
#include <mach/mmp_reg_dram.h>

#define wait_for_interrupt_enable()		cpu_do_idle()
u32 sdram_selfrefresh_enable(void);

