#ifndef __CPU_CONMM_LINUX_DRIVER_H__
#define __CPU_CONMM_LINUX_DRIVER_H__

#include "linux/spinlock.h"
#include <mach/cpucomm/cpucomm-lx.h>


//------------------------------------------------------------------------------
//  Macro       : CpuComm_CriticalSectionInit()
//  Description : Encapsulated for critical section
//------------------------------------------------------------------------------
#define CpuComm_CriticalSectionDeclare()   DEFINE_SPINLOCK(cpucomm_lock);
#define CpuComm_CriticalSectionInit()      extern spinlock_t cpucomm_lock;	\
                                           unsigned long cpucomm_flags
#define CpuComm_CriticalSectionEnter()     spin_lock_irqsave(&cpucomm_lock,cpucomm_flags)
#define CpuComm_CriticalSectionLeave()     spin_unlock_irqrestore(&cpucomm_lock,cpucomm_flags)

//------------------------------------------------------------------------------
//  Macro       : CpuComm_DbgMsg()
//  Description : Define the debug message output func
//------------------------------------------------------------------------------
#define CpuComm_DbgMsg pr_debug
#endif // __CPU_CONMM_LINUX_DRIVER_H__

