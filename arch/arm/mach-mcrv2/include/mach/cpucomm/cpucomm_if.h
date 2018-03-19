#ifndef _INCLUDE_CPU_COMM_COMMUNICATION_IF_H_
#define _INCLUDE_CPU_COMM_COMMUNICATION_IF_H_

#include <mach/mmpf_typedef.h>
#include "cpucomm_id.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define	 LOCK_CORE()   OS_LockCore()
#define	 UNLOCK_CORE() OS_UnlockCore()

#define  CPU_LOCK_INIT() OS_CRITICAL_INIT()
	
#define  CPU_LOCK() OS_ENTER_CRITICAL(); \
					LOCK_CORE()
					
#define  CPU_UNLOCK() UNLOCK_CORE();		\
		 			OS_EXIT_CRITICAL()
				

//------------------------------------------------------------------------------
//  Enumeration : CPU_COMM_TYPE
//  Description : Define type of commnucation
 //------------------------------------------------------------------------------
typedef enum
{
    CPU_COMM_TYPE_DATA,                // A data swap service between 2 CPUs
    CPU_COMM_TYPE_SEM                  // A semaphore between two CPUs

    //CPU_COMM_TYPE_SOCKET,              // A socket service between 2 CPUs
    //CPU_COMM_TYPE_FLAG                 // A flag between two CPUs
} CPU_COMM_TYPE;

// Error message for socket operation
typedef enum __CPU_COMM_ERR
{
    CPU_COMM_ERR_NONE           = 0,    // no error
    CPU_COMM_ERR_INIT_FAIL      = 1,    // communication service is not init yet
    CPU_COMM_ERR_DESTROY        = 2,    // communication service is destroied
    CPU_COMM_ERR_OPENED         = 3,    // socket is opend alerady
    CPU_COMM_ERR_CLOSED         = 4,    // socket is closed already
    CPU_COMM_ERR_BUSY           = 5,    // request is porcessing data or client did not read yet
    CPU_COMM_ERR_IDLE           = 6,    // no request is send yet
    CPU_COMM_ERR_TIMEOUT        = 7,    // swap data timeout
    CPU_COMM_ERR_UNSUPPORT      = 8,    // Unsupport method
    CPU_COMM_ERR_EVENT          = 9,    // Error from event flag operation
    CPU_COMM_ERR_OVERFLOW       = 10,   // Data size is too big
    CPU_COMM_ERR_ISR            = 11,   // Illegal op in ISR
    CPU_COMM_ERR_CPU_TIMEOUT    = 12    // Another CPU has no response
} CPU_COMM_ERR;

typedef void(CPUCOMM_RCV_PREPROC)(CPU_COMM_ID, MMP_UBYTE *, MMP_ULONG);

CPU_COMM_ERR CpuComm_RegisterEntry( CPU_COMM_ID ulCommId, CPU_COMM_TYPE ulCommType );
CPU_COMM_ERR CpuComm_UnregisterEntry( CPU_COMM_ID ulCommId );
CPU_COMM_ERR CpuComm_RegisterISRService(CPU_COMM_ID ulCommId, unsigned int (*recv_cb)(void *slot));

// Flag operations (for dual CPU)
CPU_COMM_ERR CpuComm_SemPost( CPU_COMM_ID ulCommId, MMP_ULONG ulTimeout );
CPU_COMM_ERR CpuComm_SemWait( CPU_COMM_ID ulCommId, MMP_ULONG ulTimeout );


// Socket operations for client (service rountine is declared as callback)
CPU_COMM_ERR CpuComm_DataSend( CPU_COMM_ID ulCommId, MMP_UBYTE *pbyData, MMP_ULONG ulDataSize, MMP_ULONG ulTimeout );
CPU_COMM_ERR CpuComm_DataReceive( CPU_COMM_ID ulCommId, MMP_UBYTE *pbyData, MMP_ULONG ulDataSize, MMP_ULONG ulTimeout, CPUCOMM_RCV_PREPROC psPreproc );


CPU_COMM_ERR CpuComm_ReceiveDataStart( CPU_COMM_ID ulCommId,
                                    MMP_UBYTE *pbyData,
                                    MMP_ULONG ulDataSize );


CPU_COMM_ERR CpuComm_ReceiveDataEnd( CPU_COMM_ID ulCommId, unsigned long subCommandID);

// Peterson's 2-process Mutual Exclusion Algorithm
void OS_LockCore(void) ;
void OS_UnlockCore(void/*int cpu_sr*/);
void OS_CPULockInfo(void)  ;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _INCLUDE_CPU_COMM_COMMUNICATION_IF_H_
