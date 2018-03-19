//==============================================================================
//
//  File        : os_wrap.c
//  Description : OS wrapper functions
//  Author      : Jerry Tsao
//  Revision    : 1.0
//
//==============================================================================

#include "includes_fw.h"

#include "os_wrap.h"

#if (OS_TYPE == OS_LINUX)
#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#endif

/** @addtogroup MMPF_OS
@{
*/

static MMPF_OS_WORK_CTX m_OsWorks[MMPF_OS_WORKID_MAX];

#if (OS_TYPE == OS_LINUX)
struct MMPF_OS_SEM {
    struct semaphore sem;
    bool is_init;
};
static struct MMPF_OS_SEM m_OsSems[MMPF_OS_SEMID_MAX];
#endif


//==============================================================================
//
//                              OS Wrap Functions
//
//==============================================================================
/** @brief To do os wrapper layer initialization process, but be called before
any other OS related API.

@return None.
*/
void MMPF_OS_Initialize(void)
{
    static int once = 0;

    if (!once) {
        #if (OS_TYPE == OS_LINUX)
        memset((void*)m_OsWorks, 0, sizeof(m_OsWorks));
        memset((void*)m_OsSems, 0, sizeof(m_OsSems));
        #endif

        once = 1;
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_OS_CreateSem
//  Description :
//------------------------------------------------------------------------------
/** @brief This function creates a semaphore.

@param[in] ubSemValue : is the initial value for the semaphore.  If the value
                        is 0, no resource is
                        available.  You initialize the semaphore to a non-zero
                        value to specify how many resources are available.
@retval 0xFF for create semaphore internal error from OS
		0xFE the system maximum semaphore counts exceed.
		others, the ID to access the semaphore
*/
MMPF_OS_SEMID MMPF_OS_CreateSem(MMP_UBYTE ubSemValue)
{
    #if (OS_TYPE == OS_LINUX)
    MMP_ULONG i;

    for (i = 0; i < MMPF_OS_SEMID_MAX; i++) {
        if (!m_OsSems[i].is_init) {
            sema_init(&(m_OsSems[i].sem), ubSemValue);
            m_OsSems[i].is_init = 1;
            return i;
        }
    }
    #endif

    return 0xFE;
}
EXPORT_SYMBOL(MMPF_OS_CreateSem);

//------------------------------------------------------------------------------
//  Function    : MMPF_OS_DeleteSem
//  Description :
//------------------------------------------------------------------------------
/** @brief This function deletes a semaphore and readies all tasks pending on the semaphore.

@param[in] ulSemId : The semaphore ID that return by @ref MMPF_OS_CreateSem
@retval 0xFE for bad input semaphore id,
		0, return delete success.
*/
MMP_UBYTE  MMPF_OS_DeleteSem(MMPF_OS_SEMID ulSemId)
{
    #if (OS_TYPE == OS_LINUX)
    if ((ulSemId >= MMPF_OS_SEMID_MAX) || (!m_OsSems[ulSemId].is_init)) {
        return 0xFE;
    }

    m_OsSems[ulSemId].is_init = 0;
    #endif

    return 0;
}
EXPORT_SYMBOL(MMPF_OS_DeleteSem);


//------------------------------------------------------------------------------
//  Function    : MMPF_OS_AcquireSem
//  Description :
//------------------------------------------------------------------------------
/** @brief This function waits for a semaphore.

@param[in] ulSemId : The semaphore ID that return by @ref MMPF_OS_CreateSem
@param[in] ulTimeout : is an optional timeout period (in clock ticks).  If non-zero, your task will
                            wait for the resource up to the amount of time specified by this argument.
                            If you specify 0, however, your task will wait forever at the specified
                            semaphore or, until the resource becomes available.
@retval 0xFE for bad input semaphore id,
		0xFF for acquire semaphore internal error from OS
		0 for getting the resource.
		1 for time out happens
*/
MMP_UBYTE MMPF_OS_AcquireSem(MMPF_OS_SEMID ulSemId, MMP_ULONG ulTimeout)
{
    #if (OS_TYPE == OS_LINUX)
    if ((ulSemId >= MMPF_OS_SEMID_MAX) || (!m_OsSems[ulSemId].is_init)) {
        return 0xFE;
    }

    if (ulTimeout) {
        if (down_timeout(&(m_OsSems[ulSemId].sem), ulTimeout) == 0) {
            return 0;
        }
    }
    else {
        if (down_interruptible(&(m_OsSems[ulSemId].sem)) == 0) {
            return 0;
        }
    }
    #endif

    return 1;
}
EXPORT_SYMBOL(MMPF_OS_AcquireSem);

//------------------------------------------------------------------------------
//  Function    : MMPF_OS_ReleaseSem
//  Description :
//------------------------------------------------------------------------------
/** @brief This function signals a semaphore

@param[in] ulSemId : The semaphore ID that return by @ref MMPF_OS_CreateSem
@retval 0xFE for bad input semaphore id,
		0xFF for release semaphore internal error from OS
		0 for getting the resource.
		1 If the semaphore count exceeded its limit.
*/
MMP_UBYTE MMPF_OS_ReleaseSem(MMPF_OS_SEMID ulSemId)
{
    #if (OS_TYPE == OS_LINUX)
    if ((ulSemId >= MMPF_OS_SEMID_MAX) || (!m_OsSems[ulSemId].is_init)) {
        return 0xFE;
    }

    up(&(m_OsSems[ulSemId].sem));
    #endif

    return 0;
}
EXPORT_SYMBOL(MMPF_OS_ReleaseSem);

#if (OS_TYPE == OS_UCOSII)
//------------------------------------------------------------------------------
//  Function    : MMPF_OS_AcceptSem
//  Description :
//------------------------------------------------------------------------------
/** @brief This function requests for a semaphore.

@param[in] ulSemId : The semaphore ID that return by @ref MMPF_OS_CreateSem
@param[out] usCount : The return value
    If >0, semaphore value is decremented; value is returned before the decrement.
    If 0, then either resource is unavailable, event did not occur, or null or invalid pointer was passed to the function.
@retval:
*/
MMP_UBYTE MMPF_OS_AcceptSem(MMPF_OS_SEMID ulSemId, MMP_USHORT *usCount)
{

    return 0;
}
EXPORT_SYMBOL(MMPF_OS_AcceptSem);
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_OS_TryLockSem
//  Description :
//------------------------------------------------------------------------------
/** @brief This function requests for a semaphore.

@param[in] ulSemId : The semaphore ID that return by @ref MMPF_OS_CreateSem
@retval 0xFE for bad input semaphore id,
		0xFF for release semaphore internal error from OS
		0 for getting the resource.
		1 If the semaphore count exceeded its limit.
*/
MMP_UBYTE MMPF_OS_TrySem(MMPF_OS_SEMID ulSemId)
{
    #if (OS_TYPE == OS_LINUX)
    if ((ulSemId >= MMPF_OS_SEMID_MAX) || (!m_OsSems[ulSemId].is_init)) {
        return 0xFE;
    }

    if (down_trylock(&(m_OsSems[ulSemId].sem)) == 0) {
        return 0;
    }
    #endif

    return 1;
}
EXPORT_SYMBOL(MMPF_OS_TrySem);

MMP_UBYTE MMPF_OS_Sleep(MMP_USHORT usTickCount)
{
    #if (OS_TYPE == OS_LINUX)
    msleep(usTickCount);
    #endif

    return OS_NO_ERR;
}
EXPORT_SYMBOL(MMPF_OS_Sleep);

MMP_UBYTE MMPF_OS_Sleep_MS(MMP_USHORT ms)
{
    #if (OS_TYPE == OS_LINUX)
    msleep(ms);
    #endif

    return OS_NO_ERR;
}
EXPORT_SYMBOL(MMPF_OS_Sleep_MS);

MMP_UBYTE MMPF_OS_RegisterWork(MMPF_OS_WORKID WorkId, MMPF_OS_WORK_CTX *WorkCtx)
{
    m_OsWorks[WorkId] = *WorkCtx;

    #if (OS_TYPE == OS_LINUX)
    INIT_WORK(WorkCtx->Work, (void*)WorkCtx->Exec);
    #endif

    return OS_NO_ERR;
}
EXPORT_SYMBOL(MMPF_OS_RegisterWork);

MMP_UBYTE MMPF_OS_IssueWork(MMPF_OS_WORKID WorkId)
{
    int ret = 0;

    #if (OS_TYPE == OS_LINUX)
    ret = queue_work(m_OsWorks[WorkId].Task, m_OsWorks[WorkId].Work);
    #endif

    #if (OS_TYPE == OS_UCOSII)
    MMPF_OS_SetFlags(m_OsWorks[WorkId].Task, m_OsWorks[WorkId].Work, m_OsWorks[WorkId].ulParam);
    #endif

    return ret;
}
EXPORT_SYMBOL(MMPF_OS_IssueWork);

//------------------------------------------------------------------------------
//  Function    : MMPF_OS_GetTime
//  Description :
//------------------------------------------------------------------------------
/** @brief This function gets the 32-bit counter which keeps track of the number of clock ticks.

@param[out] ulTickCount : specifies the new value that OSTime needs to take.
@retval 0 always return 0 for success
*/
MMP_UBYTE MMPF_OS_GetTime(MMP_ULONG *ulTickCount)
{
    #if (OS_TYPE == OS_LINUX)
    *ulTickCount = jiffies;
    #endif

    #if (OS_TYPE == OS_UCOSII)
    *ulTickCount = OSTimeGet();
    #endif

    return 0;
}
EXPORT_SYMBOL(MMPF_OS_GetTime);

//------------------------------------------------------------------------------
//  Function    : MMPF_OS_GetTimeMs
//  Description :
//------------------------------------------------------------------------------
/** @brief This function gets the current system time

@param[out] TimeMs : the time in unit of milliseconds
@retval 0 always return 0 for success
*/
MMP_UBYTE MMPF_OS_GetTimestamp(MMP_ULONG64 *ulTime, MMPF_OS_TIME_UNIT Unit)
{
    #if (OS_TYPE == OS_LINUX)
    struct timespec ts;
    if(Unit==MMPF_OS_TIME_UNIT_JIFFIES) {
      get_monotonic_boottime(&ts);
    }
    else {
      getnstimeofday(&ts);
    }
    switch (Unit) {
    case MMPF_OS_TIME_UNIT_SEC:
        *ulTime = (MMP_ULONG64)ts.tv_sec;
        break;
    case MMPF_OS_TIME_UNIT_MS:
        *ulTime = (MMP_ULONG64)((MMP_ULONG64)ts.tv_sec*1000 + (ts.tv_nsec+500000)/1000000);
        break;
    case MMPF_OS_TIME_UNIT_US:
        *ulTime = (MMP_ULONG64)((MMP_ULONG64)ts.tv_sec*1000000 + (ts.tv_nsec+500)/1000);
        break;
    case MMPF_OS_TIME_UNIT_JIFFIES:    
    default:
        *ulTime = (MMP_ULONG64)((MMP_ULONG64)ts.tv_sec*1000 + (ts.tv_nsec+500000)/1000000);
        break;
    }
    #endif

    #if (OS_TYPE == OS_UCOSII)
    #endif

    return 0;
}
EXPORT_SYMBOL(MMPF_OS_GetTimestamp);



#if 1 /// Portings ===============================================

#if (OS_TYPE == OS_UCOSII)
#pragma O0
#endif
void RTNA_Wait_Count(MMP_ULONG count)
{
    while((*(volatile MMP_ULONG *)&count)--);
}
EXPORT_SYMBOL(RTNA_Wait_Count);
#if (OS_TYPE == OS_UCOSII)
#pragma
#endif

void RTNA_WAIT_CYCLE(MMP_ULONG cycle)
{
    RTNA_Wait_Count(cycle / 6);
}
EXPORT_SYMBOL(RTNA_WAIT_CYCLE);

void RTNA_WAIT_MS(MMP_ULONG ms)
{
    mdelay(ms);
}
EXPORT_SYMBOL(RTNA_WAIT_MS);

void RTNA_WAIT_US(MMP_ULONG us)
{
    udelay(us);
}
EXPORT_SYMBOL(RTNA_WAIT_US);

#endif
/** @} */ // MMPF_OS
