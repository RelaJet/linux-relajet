//==============================================================================
//
//  File        : os_wrap.h
//  Description : OS wrapper function for uC/OS-II
//  Author      : Jerry Tsao
//  Revision    : 1.0
//
//==============================================================================


#ifndef _OS_WRAP_H_
#define _OS_WRAP_H_

#include "includes_fw.h"
#if (OS_TYPE == OS_LINUX)
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#endif // (OS_TYPE == OS_LINUX)

#if (OS_TYPE == OS_UCOSII)
void        OSWrap_StatInit(void);
MMP_USHORT  OSWrap_TaskNameSet(MMP_UBYTE prio, char *pname);
MMP_USHORT  OSWrap_EventNameSet(OS_EVENT *pevent, char *pname);

#define EXPORT_SYMBOL(_s)
#endif // (OS_TYPE == OS_UCOSII)

//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================

#if (OS_TYPE == OS_LINUX)
#define MMPF_OS_SEMID_MAX   	    (64)

#define OS_CRITICAL_METHOD          (3)

#define OS_NO_ERR                     0u

#define OS_ERR_EVENT_TYPE             1u
#define OS_ERR_PEND_ISR               2u
#define OS_ERR_POST_NULL_PTR          3u
#define OS_ERR_PEVENT_NULL            4u
#define OS_ERR_POST_ISR               5u
#define OS_ERR_QUERY_ISR              6u
#define OS_ERR_INVALID_OPT            7u
#define OS_ERR_TASK_WAITING           8u
#define OS_ERR_PDATA_NULL             9u

#define OS_TIMEOUT                   10u
#define OS_TASK_NOT_EXIST            11u
#define OS_ERR_EVENT_NAME_TOO_LONG   12u
#define OS_ERR_FLAG_NAME_TOO_LONG    13u
#define OS_ERR_TASK_NAME_TOO_LONG    14u
#define OS_ERR_PNAME_NULL            15u
#define OS_ERR_TASK_CREATE_ISR       16u

#define OS_ENTER_CRITICAL()     local_irq_save(cpu_sr)
#define OS_EXIT_CRITICAL()      local_irq_restore(cpu_sr)
#endif // (OS_TYPE == OS_LINUX)

#if (OS_TYPE == OS_UCOSII)
#define MMPF_OS_SEMID_MAX   	OS_MAX_EVENTS
#define MMPF_OS_FLAGID_MAX  	OS_MAX_FLAGS
#define MMPF_OS_MUTEXID_MAX  	4
#define MMPF_OS_MQID_MAX   		OS_MAX_QS
#define MMPF_OS_TMRID_MAX   	OS_TMR_CFG_MAX

#endif // (OS_TYPE == OS_UCOSII)
//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef MMP_UBYTE   MMPF_OS_TASKID;
typedef MMP_ULONG   MMPF_OS_SEMID;
typedef MMP_ULONG   MMPF_OS_FLAGID;
typedef MMP_ULONG   MMPF_OS_MUTEXID;
typedef MMP_ULONG   MMPF_OS_MQID;
typedef MMP_ULONG   MMPF_OS_TMRID;

#if (OS_TYPE == OS_LINUX)
typedef MMP_ULONG   MMPF_OS_FLAGS;

typedef unsigned long   OS_CPU_SR;
#endif // (OS_TYPE == OS_LINUX)
#if (OS_TYPE == OS_UCOSII)
#if OS_FLAGS_NBITS == 16
typedef MMP_USHORT   MMPF_OS_FLAGS;
#endif
#if OS_FLAGS_NBITS == 32
typedef MMP_ULONG    MMPF_OS_FLAGS;
#endif
#endif // (OS_TYPE == OS_UCOSII)

#if (OS_TYPE == OS_UCOSII)
typedef MMP_USHORT  MMPF_OS_FLAG_WTYPE;
    #define MMPF_OS_FLAG_WAIT_CLR_ALL       OS_FLAG_WAIT_CLR_ALL
    #define MMPF_OS_FLAG_WAIT_CLR_ANY       OS_FLAG_WAIT_CLR_ANY
    #define MMPF_OS_FLAG_WAIT_SET_ALL       OS_FLAG_WAIT_SET_ALL
    #define MMPF_OS_FLAG_WAIT_SET_ANY       OS_FLAG_WAIT_SET_ANY
    #define MMPF_OS_FLAG_CONSUME            OS_FLAG_CONSUME
typedef MMP_USHORT  MMPF_OS_FLAG_OPT;
    #define MMPF_OS_FLAG_CLR                OS_FLAG_CLR
    #define MMPF_OS_FLAG_SET                OS_FLAG_SET
typedef MMP_UBYTE   MMPF_OS_TMR_OPT;
    #define MMPF_OS_TMR_OPT_ONE_SHOT        OS_TMR_OPT_ONE_SHOT
    #define MMPF_OS_TMR_OPT_PERIODIC        OS_TMR_OPT_PERIODIC
    #define MMPF_OS_TMR_OPT_NONE			OS_TMR_OPT_NONE
    #define MMPF_OS_TMR_OPT_CALLBACK		OS_TMR_OPT_CALLBACK

typedef OS_TMR_CALLBACK	MMPF_OS_TMR_CALLBACK;




typedef struct {
    MMP_ULONG   pbos;           // lower address
    MMP_ULONG   ptos;           // higer address
	MMP_UBYTE	ubPriority;
} MMPF_TASK_CFG;
#endif // (OS_TYPE == OS_UCOSII)

typedef void MMPF_OS_CALLBACK(void *);

typedef struct _MMPF_OS_EVENT_ACTION {
    MMPF_OS_CALLBACK    *Exec;
    void                *Arg;
} MMPF_OS_EVENT_ACTION;

typedef struct _MMPF_OS_WORK_CTX {
    MMPF_OS_CALLBACK        *Exec;

    #if (OS_TYPE == OS_LINUX)
    struct workqueue_struct *Task;
    struct work_struct      *Work;
    #endif
    #if (OS_TYPE == OS_UCOSII)
    MMPF_OS_FLAGID          Task;
    MMPF_OS_FLAGS           Work;
    MMP_ULONG               ulParam;
    #endif
} MMPF_OS_WORK_CTX;

typedef enum _MMPF_OS_WORKID {
    MMPF_OS_WORKID_ISP_FRM_ST = 0,
    MMPF_OS_WORKID_ENC_ST_0,
    MMPF_OS_WORKID_ENC_ST_1,
    MMPF_OS_WORKID_ENC_ST_2,
#if AITCAM_MULTI_STREAM_EN
    MMPF_OS_WORKID_ENC_ST_3,
    MMPF_OS_WORKID_ENC_ST_4,
#endif
    MMPF_OS_WORKID_MAX
} MMPF_OS_WORKID;

typedef enum _MMPF_OS_LOCK_CTX {
    MMPF_OS_LOCK_CTX_TASK = 0,       ///< allow sleep
    MMPF_OS_LOCK_CTX_ISR,
    MMPF_OS_LOCK_CTX_BYPASS
} MMPF_OS_LOCK_CTX;

typedef enum _MMPF_OS_TIME_UNIT {
    MMPF_OS_TIME_UNIT_SEC = 0,
    MMPF_OS_TIME_UNIT_MS,
    MMPF_OS_TIME_UNIT_US,
    MMPF_OS_TIME_UNIT_JIFFIES
} MMPF_OS_TIME_UNIT;

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

void                MMPF_OS_Initialize(void);
void                MMPF_OS_StartTask(void);

#if (OS_TYPE == OS_UCOSII)
// Task Related
MMPF_OS_TASKID      MMPF_OS_CreateTask(void (*taskproc)(void *param), MMPF_TASK_CFG *task_cfg, void *param);
MMPF_OS_TASKID  	MMPF_OS_ChangeTaskPriority(MMPF_OS_TASKID taskid, MMP_UBYTE ubNewPriority);
MMP_UBYTE  			MMPF_OS_DeleteTask(MMPF_OS_TASKID taskid);
MMP_UBYTE           MMPF_OS_SuspendTask(MMPF_OS_TASKID taskid);
MMP_UBYTE           MMPF_OS_ResumeTask(MMPF_OS_TASKID taskid);
#endif // (OS_TYPE == OS_UCOSII)

// Semaphore Related
MMPF_OS_SEMID       MMPF_OS_CreateSem(MMP_UBYTE ubSemValue);
MMP_UBYTE	        MMPF_OS_DeleteSem(MMPF_OS_SEMID ulSemId);
MMP_UBYTE 			MMPF_OS_ReleaseSem(MMPF_OS_SEMID ulSemId);
MMP_UBYTE      		MMPF_OS_AcquireSem(MMPF_OS_SEMID ulSemId, MMP_ULONG ulTimeout);
MMP_UBYTE           MMPF_OS_TrySem(MMPF_OS_SEMID ulSemId);
#if (OS_TYPE == OS_UCOSII)
MMP_UBYTE           MMPF_OS_AcceptSem(MMPF_OS_SEMID ulSemId, MMP_USHORT *usCount);
MMP_UBYTE 			MMPF_OS_QuerySem(MMPF_OS_SEMID ulSemId, MMP_USHORT *usCount);

// Task Related
MMPF_OS_FLAGID 		MMPF_OS_CreateEventFlagGrp(MMP_ULONG ulFlagValues);
MMP_UBYTE  			MMPF_OS_DeleteEventFlagGrp(MMPF_OS_FLAGID ulFlagId);
MMP_UBYTE 			MMPF_OS_SetFlags(MMPF_OS_FLAGID ulFlagID, MMPF_OS_FLAGS flags, MMPF_OS_FLAG_OPT opt);
MMP_UBYTE 			MMPF_OS_WaitFlags(MMPF_OS_FLAGID ulFlagID, MMPF_OS_FLAGS flags, MMPF_OS_FLAG_WTYPE waitType, 
							MMP_ULONG ulTimeout, MMPF_OS_FLAGS *ret_flags);
MMP_UBYTE 			MMPF_OS_QueryFlags(MMPF_OS_FLAGID ulFlagID, MMPF_OS_FLAGS *ret_flags);

// Mutex Related
MMPF_OS_MUTEXID 	MMPF_OS_CreateMutex(MMP_UBYTE	ubPriority);
MMP_UBYTE  			MMPF_OS_DeleteMutex(MMPF_OS_MUTEXID ulMutexId);
MMP_UBYTE 			MMPF_OS_AcquireMutex(MMPF_OS_MUTEXID ulMutexId, MMP_ULONG ulTimeout);
MMP_UBYTE 			MMPF_OS_ReleaseMutex(MMPF_OS_MUTEXID ulMutexId);

// Message Queue Related
MMPF_OS_MQID 		MMPF_OS_CreateMQueue(void **msg, MMP_UBYTE ubQueueSize);
MMP_UBYTE  			MMPF_OS_DeleteMQueue(MMPF_OS_MQID ulMQId);
MMP_UBYTE 			MMPF_OS_GetMessage(MMPF_OS_MQID ulMQId,void **msg, MMP_ULONG ulTimeout);
MMP_UBYTE 			MMPF_OS_PutMessage(MMPF_OS_MQID ulMQId, void *msg);
#endif // (OS_TYPE == OS_UCOSII)

MMP_UBYTE           MMPF_OS_RegisterWork(MMPF_OS_WORKID WorkId, MMPF_OS_WORK_CTX *WorkCtx);
MMP_UBYTE           MMPF_OS_IssueWork(MMPF_OS_WORKID WorkId);

// Time Related
MMP_UBYTE 			MMPF_OS_Sleep_MS(MMP_USHORT ms);
MMP_UBYTE 			MMPF_OS_Sleep(MMP_USHORT usTickCount);

MMP_UBYTE           MMPF_OS_GetTime(MMP_ULONG *ulTickCount);
MMP_UBYTE           MMPF_OS_GetTimestamp(MMP_ULONG64 *ulTime, MMPF_OS_TIME_UNIT Unit);

#if (OS_TYPE == OS_UCOSII)
// Timer Related
MMPF_OS_TMRID 	MMPF_OS_StartTimer(MMP_ULONG ulPeriod, MMPF_OS_TMR_OPT opt, MMPF_OS_TMR_CALLBACK callback,
								void *callback_arg);
MMP_UBYTE 		MMPF_OS_StopTimer(MMPF_OS_TMRID ulTmrId, MMPF_OS_TMR_OPT opt);

void*				MMPF_OS_Malloc(int len);
void* 				MMPF_OS_Calloc(int num, int size);
void    			MMPF_OS_MemFree(char *mem_ptr);
//------------------------------
// User Specified Configuration
//------------------------------


extern MMPF_TASK_CFG        task_cfg[];


#endif // (OS_TYPE == OS_UCOSII)
#endif // _OS_WRAP_H_
