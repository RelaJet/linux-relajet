
#ifndef _INCLUDES_FW_H_
#define _INCLUDES_FW_H_

#include <linux/kernel.h>
#include <linux/string.h>

#define ALL_FW      (1)
#define BUILD_FW    (1)

#define OS_UCOSII   (1)
#define OS_LINUX    (2)
#define OS_TYPE     (OS_LINUX)

#define KERN_DIV64  (1)

#include <mach/mmp_err.h>
#include <mach/mmp_register.h>
#include <mach/mmpf_typedef.h>
#include <mach/os_wrap.h>
#include <mach/config_fw.h>

#if (KERN_DIV64 == 1)
    #include <asm/div64.h>
#endif

#define MERCURY_CORE_ID (0x80)
#define MCR_V1_CORE_ID  (0x81)

extern MMP_UBYTE    gbSystemCoreID;
extern MMP_UBYTE    gbSystemDramID;
extern MMP_USHORT   gsISPCoreID;

/// Porting Layer Macro ========================================================
#define ALIGN2(_a)      (((_a) + 1) >> 1 << 1)
#define FLOOR4(_a)      ((_a) >> 2 << 2)
#define ALIGN4(_a)      (((_a) + 3) >> 2 << 2)
#define ALIGN8(_a)      (((_a + 0x07) >> 3) << 3)
#define ALIGN16(_a)     (((_a) + 15) >> 4 << 4)
#define FLOOR32(_a)     ((_a) >> 5 << 5)
#define ALIGN32(_a)     (((_a) + 31) >> 5 << 5)
#define FLOOR512(_a)    ((_a) >> 8 << 8)
#define ALIGN512(_a)    (((_a) + 511) >> 10 << 10)
#define ALIGN_PAGE(_a)  ((((_a)+(1<<PAGE_SHIFT)-1)>>PAGE_SHIFT)<<PAGE_SHIFT)

#ifdef __GNUC__
#define packed_union    union __attribute__((packed))
#define packed_struct   struct __attribute__((packed))
#define __align(_a)     __attribute__((aligned(_a)))
#else
#define packed_union    __packed union
#define packed_struct   __packed struct
#endif

#ifndef NULL
    #define NULL        (0)
#endif

#define RTNA_DBG_Str(_x, _s)        printk(KERN_ERR _s)
#define RTNA_DBG_Long(_x, _v)       printk(KERN_ERR "%d", _v)
#define RTNA_DBG_Short(_x, _v)      printk(KERN_ERR "%d", _v)
#define RTNA_DBG_Byte(_x, _v)       printk(KERN_ERR "%d", _v)
#define DBG_S(_x, _v)               printk(KERN_ERR _v)

#define dbg_printf(_x, _s, ...)     printk(KERN_ERR _s, ##__VA_ARGS__)
#define PRINTF(_s, ...)             printk(KERN_ERR _s, ##__VA_ARGS__)

#define MEMSET(s, c, n)          memset(s, c, n)
#define MEMSET0(s)               memset(s, 0, sizeof(*s))
#define MEMCPY(d, s, c)          memcpy(d, s, c)


#endif // _INCLUDES_FW_H_

