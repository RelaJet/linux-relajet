//==============================================================================
//
//  File        : mmpf_cpucomm.c
//  Description : CPU communication between dual CPU
//  Author      : Chiket Lin
//  Revision    : 1.0
//
//==============================================================================

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================
#include "cpucomm_core.h"

#include <mach/cpucomm/cpucomm_api.h>


//==============================================================================
//
//                              Define
//
//==============================================================================

//------------------------------------------------------------------------------
//  Variable    : g_psCpuCommTable
//  Description : Store pointer of date entry sent from client modules.
//                Value is assigned because the first ID is 0, the same as init
//                vlaue
//------------------------------------------------------------------------------
static __align(CACHE_LINE_SIZE) CPU_COMM_ENTRY           g_psCpuCommTable[CPUCOMM_ID_MAX_NUM] = {{CPU_COMM_ID_ILLEGAL}};
static volatile _CPU_ID         s_ulCpuId;
AITPS_HINT g_pHINT;
static AITPS_HINT               s_psHintBase;
static CPU_COMM_SWAP_DATA       *s_psSwapData;
CpuComm_CriticalSectionDeclare();

char* strCommID[CPUCOMM_ID_MAX_NUM] = {

  "SYSFLAG",
  "UART",
  "MD",
};

//------------------------------------------------------------------------------
//  Function    : CpuComm_VolatileCopy
//  Description : Copy volatile data
//------------------------------------------------------------------------------
static __inline void CpuComm_VolatileCopy( volatile void *pbyDst, volatile void *pbySrc, MMP_ULONG ulSize )
{
    MMP_ULONG i;
    CPU_COMM_SWAP_DATA* p;
    unsigned long* pData;

	pr_debug( "Copy: 0x%x -> 0x%x %d Bytes:\r\n",pbySrc,pbyDst,ulSize);
    for( i = 0; i<ulSize; i++ )
    {
        ((volatile MMP_UBYTE*)pbyDst)[i] = ((volatile MMP_UBYTE*)pbySrc)[i];
    }

	if(ulSize==sizeof(CPU_COMM_SWAP_DATA))
	{
		p = pbyDst;
		pr_debug("ulCommId = %d (%s)\n",p->ulCommId,strCommID[p->ulCommId]);	
		pData = p->pbyBuffer;
		pr_debug("Data = %08x  %08x  %08x  %08x %08x\n",pData[0],pData[1],pData[2],pData[3]);
	}
	else
	{
		for( i = 0; i<ulSize; i++ )
		{
			pr_debug( " 0x%02x ", ((volatile MMP_UBYTE*)pbyDst)[i] );
			if(ulSize%8==0)
				pr_debug( "\r\n");	
		}
	}


}

//------------------------------------------------------------------------------
//  Function    : CpuComm_GetSwapData
//  Description : Get global swap data
//                psSwapData in function consists with two swap data for two CPU
//                psSwapData[0]: send CPU_A data to CPU_B
//                psSwapData[1]: send CPU_B data to CPU_A
//                This table is implemented by the CPU share register
//                If we need a bigger size of swap buffer, we have to move
//                the global swap buffer to SRAM. But it means we need to
//                another fixed address in SRAM or CPU A must pass the
//                the address to CPU B during init.
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] ulId : CPU ID
 @retval: a swap data pointer
*/
static __inline CPU_COMM_SWAP_DATA* CpuComm_GetGlobalSwapData( _CPU_ID ulId )
{
    CPU_COMM_SWAP_DATA  *psSwapData = (CPU_COMM_SWAP_DATA *)MMP_CPUCOMM_SHARE_REG_GET();

    return ulId==_CPU_ID_A ? &psSwapData[0] : &psSwapData[1];
}


//------------------------------------------------------------------------------
//  Function    : CpuComm_GetDataEntry
//  Description : Get the comm data entry
//                There is still a problem, we do not check that
//                two comm tables have same entries. Decalrations file
//                may not include in MCP file, and LX is more complicate.
//                Maybe we also need to introduce the table check before init.
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] ulCommId : communication ID, or the index value of communication data table.
 @retval: a communication data entry
*/
static __inline CPU_COMM_ENTRY* CpuComm_GetDataEntry( CPU_COMM_ID ulCommId )
{

    if( ulCommId >= CPUCOMM_ID_MAX_NUM )
    {
        pr_err("cpucomm@%s: Comm id %d is exceed %d\n",__func__,ulCommId,CPUCOMM_ID_MAX_NUM);

        return NULL;
    }

    return &g_psCpuCommTable[ulCommId];
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_InitData
//  Description : Reset and init communication data
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] psData : pointer of communication data
 @retval: TRUE:  init successfully
          FALSE: init fail
*/
static __inline CPU_COMM_ERR CpuComm_InitData( CPU_COMM_ENTRY* psData )
{
    MMPF_OS_SEMID   ulFlagId;
    MMPF_OS_SEMID   ulSemId;

    CpuComm_CriticalSectionInit();

    // Reset comm data
    memset( &psData->sCommData, 0, sizeof(CPU_COMM_DATA) );

    // Create flag and set the initial value as 0
    if( (ulFlagId = MMPF_OS_CreateSem( 0 )) >= 0xFE )
    {
        return CPU_COMM_ERR_INIT_FAIL;
    }

    // Create semaphore for data critical protection
    if( (ulSemId = MMPF_OS_CreateSem( 1 )) >= 0xFE )
    {
        MMPF_OS_DeleteSem( ulFlagId );
        return CPU_COMM_ERR_INIT_FAIL;
    }

    // Enter critial section to ensure operation will not be interrupt by other thread.
    CpuComm_CriticalSectionEnter();

    // Assing flag and semaphore to comm data
    pr_info("psData->ulCommId = %d ==> Flag ID = %d   , SemID = %d\n",psData->ulCommId ,ulFlagId, ulSemId);    
    psData->sCommData.ulFlagId = ulFlagId;
    psData->sCommData.ulSemId = ulSemId;

    // Leave critical section
    CpuComm_CriticalSectionLeave();


    return CPU_COMM_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_DestroyData
//  Description : Destroy comm data
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] psData : pointer of comm data
*/
static __inline void CpuComm_DestroyData( CPU_COMM_ENTRY* psCommEntry )
{
    volatile MMPF_OS_SEMID   ulSemId, ulFlagId;

    CpuComm_CriticalSectionInit();

    // Enter critial section to ensure operation will not be interrupt by other thread.
    CpuComm_CriticalSectionEnter();

    // copy flag and semaphore
    ulFlagId = psCommEntry->sCommData.ulFlagId;
    ulSemId = psCommEntry->sCommData.ulSemId;

    // Reset data
    psCommEntry->ulCommId = CPU_COMM_ID_ILLEGAL;
    psCommEntry->sCommData.ulFlagId = 0xFF;
    psCommEntry->sCommData.ulSemId  = 0xFF;

    // Leave critical section
    CpuComm_CriticalSectionLeave();

    // Delete flag
    MMPF_OS_DeleteSem( ulFlagId );

    // Delete semaphore
    MMPF_OS_DeleteSem( ulSemId );
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_DataCriticalEnter
//  Description : Critical section for data protection.
//------------------------------------------------------------------------------
static __inline MMP_BOOL CpuComm_DataCriticalEnter( CPU_COMM_ENTRY* psCommEntry )
{
    if( MMPF_OS_AcquireSem( psCommEntry->sCommData.ulSemId, 0 ) )
    {
        return MMP_FALSE;
    }

    if( psCommEntry->ulCommId == CPU_COMM_ID_ILLEGAL )
    {
        return MMP_FALSE;
    }

    return MMP_TRUE;
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_DataCriticalLeave
//  Description : Critical section for data protection.
//------------------------------------------------------------------------------
static __inline void CpuComm_DataCriticalLeave( CPU_COMM_ENTRY* psCommEntry )
{
    if( psCommEntry->ulCommId != CPU_COMM_ID_ILLEGAL )
    {
    MMPF_OS_ReleaseSem( psCommEntry->sCommData.ulSemId );
    }
}


//------------------------------------------------------------------------------
//  Function    : CpuComm_SwapCheck
//  Description : Check this CPU's swap data
//------------------------------------------------------------------------------
static void CpuComm_SwapCheck( MMP_BOOL bIRQ )
{
    CPU_COMM_SWAP_DATA*    psGlobalSwapData = CpuComm_GetGlobalSwapData(s_ulCpuId==_CPU_ID_A?_CPU_ID_B:_CPU_ID_A);
    CPU_COMM_ENTRY*        psCommEntry;

    // Get Comm entry
    psCommEntry = CpuComm_GetDataEntry( psGlobalSwapData->ulCommId );
    if( psCommEntry == NULL )
    {
        if( bIRQ )
        {
            CpuComm_DbgMsg( "comm swap: wrong id %d\r\n", psGlobalSwapData->ulCommId );
        }
        return;
    }

    // Show message if data processing is task, not IRQ
    if( !bIRQ ) 
    {
        CpuComm_DbgMsg( "comm swap: proc %d\r\n", psGlobalSwapData->ulCommId );
    }

    // Copy global data to comm data entry
     CpuComm_VolatileCopy( &psCommEntry->sCommData.sData, psGlobalSwapData, sizeof(CPU_COMM_SWAP_DATA) );

    // Set flags to notify data is arrived
    if( MMPF_OS_ReleaseSem( psCommEntry->sCommData.ulFlagId ) )
    {
        CpuComm_DbgMsg( "comm isr: sem error %d, %d\r\n", psGlobalSwapData->ulCommId, psCommEntry->sCommData.ulFlagId );
        // todo ?
    }

    // Clear ulFlags of global data to notify another CPU for next swap
    psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_SwapISR
//  Description : ISR for the communication between both CPU
//------------------------------------------------------------------------------
void CpuComm_SwapISR_CPUA( void )
{
    CPU_COMM_SWAP_DATA*    psGlobalSwapData = CpuComm_GetGlobalSwapData(_CPU_ID_B);
    CPU_COMM_ENTRY*        psCommEntry;

	struct cpu_comm_transfer_data *p;// =(struct cpu_comm_transfer_data*)psCommEntry->sCommData.sData.pbyBuffer;

    // If ulFalgData or ulCommandId is illegal, there is nothing to do.
    if( psGlobalSwapData->ulCommId == CPU_COMM_ID_ILLEGAL )
    {
        goto ret;
    }

    // Get Comm entry
    psCommEntry = CpuComm_GetDataEntry( psGlobalSwapData->ulCommId );
    if( psCommEntry == NULL )
    {
        goto ret;
    }

    // Copy global data to comm data entry
     CpuComm_VolatileCopy( &psCommEntry->sCommData.sData, psGlobalSwapData, sizeof(CPU_COMM_SWAP_DATA) );

	p =(struct cpu_comm_transfer_data*)psCommEntry->sCommData.sData.pbyBuffer;
	pr_debug("B2A IRQ CommID:%d %s\n",psCommEntry->sCommData.sData.ulCommId , p ->flag&CPUCOMM_FLAG_ACK?"ACK":"SND");
	
    // Set flags to notify data is arrived
    if( MMPF_OS_ReleaseSem( psCommEntry->sCommData.ulFlagId ) )
    {
        CpuComm_DbgMsg( "comm isr: sem error %d, %d\r\n", psGlobalSwapData->ulCommId, psCommEntry->sCommData.ulFlagId );
        // todo ?
    }

ret:

    // Clear IRQ flag
 //   MMP_CPUCOMM_IRQ_CLEAR( _CPU_ID_A);

    // Clear ulFlags of global data to notify another CPU for next swap
    //psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;
    return;
}


//------------------------------------------------------------------------------
//  Function    : CpuComm_SwapISR
//  Description : ISR for the communication between both CPU
//------------------------------------------------------------------------------
void CpuComm_SwapISR( void )
{
    CPU_COMM_SWAP_DATA*    psGlobalSwapData = CpuComm_GetGlobalSwapData(s_ulCpuId==_CPU_ID_A?_CPU_ID_B:_CPU_ID_A);
    CPU_COMM_ENTRY*        psCommEntry;

    // If ulFalgData or ulCommandId is illegal, there is nothing to do.
    if( psGlobalSwapData->ulCommId == CPU_COMM_ID_ILLEGAL )
    {
        goto ret;
    }

    // Get Comm entry
    psCommEntry = CpuComm_GetDataEntry( psGlobalSwapData->ulCommId );
    if( psCommEntry == NULL )
    {
        goto ret;
    }

    // Copy global data to comm data entry
     CpuComm_VolatileCopy( &psCommEntry->sCommData.sData, psGlobalSwapData, sizeof(CPU_COMM_SWAP_DATA) );

    // Set flags to notify data is arrived
    if( MMPF_OS_ReleaseSem( psCommEntry->sCommData.ulFlagId ) )
    {
        // todo ?
    }
    pr_debug("CPUCOMM: Release Sem %d\n",psCommEntry->sCommData.ulFlagId);		

ret:
    // Clear IRQ flag
    //MMP_CPUCOMM_IRQ_CLEAR( s_ulCpuId );

    // Clear ulFlags of global data to notify another CPU for next swap
    //psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;
    return;
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_Swap
//  Description : Swap data to another CPU
//                The implementation is still busy wait. In normal case, the max
//                wait time of uC/OS2 is aboue 2.6 us (loop count = 10) and the 
//                average is 1.1us. Because there is only swap buffer shared by all
//                entries, it may case a performance issue if implemented by ISR.
//
//                The force sleeping may cause the incorrect response when the local
//                CPU is very busy, the thread may be rescheduled when sending the
//                data: it is caused by the busy of local CPU, not another CPU.
//                It means the timeout occurs when the most CPU resources are occupied
//                by another thread. Sleeping is not good idea because it cause the time out
//                is the common case.
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] psSwapData:  pointer of swap data
 @param[in] ulTimeout:   timeout count in millisecond, 0 is wait forever 
 @retval: TRUE: swap successful
          FALSE swap timeout
*/
CPU_COMM_ERR CpuComm_Swap(CPU_COMM_SWAP_DATA* psSwapData)
{
//#define SWAP_STATISTIC
#define TIMEOUT_COUNT   (50) // 50 = 5 us
    CPU_COMM_SWAP_DATA* psGlobalSwapData = CpuComm_GetGlobalSwapData(s_ulCpuId);
    MMP_ULONG           ulTimeoutCount = TIMEOUT_COUNT;
    CPU_COMM_ERR        ulRet = CPU_COMM_ERR_NONE;
#if defined(SWAP_STATISTIC)    
    static MMP_ULONG    ulMax = 0, ulTotal = 0, ulTotalCount;
#endif

    CpuComm_CriticalSectionInit();

    CpuComm_CriticalSectionEnter();

    // Busy wait for another CPU,
     while( (psGlobalSwapData->ulCommId != CPU_COMM_ID_ILLEGAL))// && (--iTimeoutCount > 0) )
    {
        // To avoid dead lock of dual disabled IRQ, both side should
        // check thier ISR.
        //msleep(10);
        CpuComm_SwapISR();
    }

#if defined(SWAP_STATISTIC)
    ulTotal += TIMEOUT_COUNT - ulTimeoutCount;
    ulTotalCount++;

    if( ulMax < TIMEOUT_COUNT - ulTimeoutCount )
    {
        ulMax = TIMEOUT_COUNT - ulTimeoutCount;
        CpuComm_DbgMsg("comm stat:%d, %d, %d\r\n", ulMax, ulTotal, ulTotalCount );
    }
    
    if( ulTotalCount > 0xFFFFFFF0 )
    {
        CpuComm_DbgMsg("comm stat:%d, %d, %d\r\n", ulMax, ulTotal, ulTotalCount );
        ulTotal = ulTotalCount = 0;
    }
#endif
    // Swap timeout?
    if( ulTimeoutCount == 0 )
    {
        CpuComm_DbgMsg( "comm swap: timeout, %d\r\n", psSwapData->ulCommId );
        ulRet =  CPU_COMM_ERR_CPU_TIMEOUT;
        goto ERROR;
    }

    // copy data
     CpuComm_VolatileCopy( psGlobalSwapData, psSwapData, sizeof(CPU_COMM_SWAP_DATA) );

    // Trigger IRQ to antoher CPU
    
   // g_pHINT->HINT_SET_CPU_INT =  HINT_CPU2CPUB_INT;

	//pr_info("Send IRQ A2B %d\n",psGlobalSwapData->ulCommId);
	
	MMP_CPUCOMM_IRQ_SET( s_ulCpuId==_CPU_ID_A?_CPU_ID_B:_CPU_ID_A);

ERROR:
	CpuComm_CriticalSectionLeave();
	return ulRet;
}

static CPU_COMM_ERR CpuComm_CommWaitAck( CPU_COMM_ENTRY* psCommEntry,
                                        MMP_UBYTE *pbyData,
                                        MMP_ULONG ulDataSize,
                                        MMP_ULONG ulTimeout )
{
	pr_debug("CPUCOMM: CpuComm_CommWaitAck%d\n",psCommEntry->sCommData.ulFlagId);		
	struct cpu_comm_transfer_data *p =(struct cpu_comm_transfer_data*)psCommEntry->sCommData.sData.pbyBuffer;

	CPU_COMM_SWAP_DATA* psGlobalSwapData = CpuComm_GetGlobalSwapData(s_ulCpuId==_CPU_ID_A?_CPU_ID_B:_CPU_ID_A);	

    // Wait flag
    if( MMPF_OS_AcquireSem( psCommEntry->sCommData.ulFlagId, ulTimeout ) )
    {
        pr_err("CPUCOMM: %s CPU COMM error event\n",__func__);    
        return CPU_COMM_ERR_EVENT;
    }

	pr_debug("CPUCOMM: Exit CommWait %d\n",psCommEntry->sCommData.ulFlagId);		
	
    if( psCommEntry->ulCommId == CPU_COMM_ID_ILLEGAL )
    {
        pr_err("CPUCOMM: %s CPU COMM ID illegal\n",__func__);		    
        return CPU_COMM_ERR_DESTROY;
    }

    if( !(p->flag & CPUCOMM_FLAG_ACK ))
    {
	pr_err("CPUCOMM: %s Ack format is wrong. p->flag = %d\n",__func__,p->flag);
	WARN_ON(!(p->flag & CPUCOMM_FLAG_ACK ));

    	psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;     //unlock CPUB
       return CPU_COMM_ERR_UNSUPPORT;
    }

    if( (p->result!= CPUCOMM_FLAG_RESULT_OK ))
    {
    
	pr_err("CPUCOMM:%s result = %d\n",__func__,p->result);
	WARN_ON(p->result!= CPUCOMM_FLAG_RESULT_OK);
    	 psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;     //unlock CPUB
        return p->result;
    }

    // Copy Respose Data
    if( pbyData != NULL )
    {
        // memcpy( pbyData, psCommEntry->sCommData.sData.pbyBuffer, ulDataSize );
        CpuComm_VolatileCopy( pbyData, psCommEntry->sCommData.sData.pbyBuffer, ulDataSize );
    }
    psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;     //unlock CPUB
        	
    return CPU_COMM_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_CommWait
//  Description : Check or wait the response from another CPU
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] psCommEntry:  comm data entry
 @param[in] pbyData:      pointer to store the return data
 @param[in] ulDataSize:   data size
 @param[in] ulTimeout:    timeout count
 @retval: CPU_COMM_ERR
*/
static CPU_COMM_ERR CpuComm_CommWait( CPU_COMM_ENTRY* psCommEntry,
                                        MMP_UBYTE *pbyData,
                                        MMP_ULONG ulDataSize,
                                        MMP_ULONG ulTimeout )
{
    CPU_COMM_ERR ulRet;
	pr_debug("CPUCOMM: Start CommWait %d\n",psCommEntry->sCommData.ulFlagId);	
	
   CPU_COMM_SWAP_DATA*    psGlobalSwapData = CpuComm_GetGlobalSwapData(s_ulCpuId==_CPU_ID_A?_CPU_ID_B:_CPU_ID_A);
   
    // Wait flag
    switch( MMPF_OS_AcquireSem( psCommEntry->sCommData.ulFlagId, ulTimeout ) )
    {
        case 0:
            break;

        case 1:
            ulRet = CPU_COMM_ERR_TIMEOUT;
            goto ERROR;

        case 2:
            ulRet = CPU_COMM_ERR_ISR;
            goto ERROR;

        default:
            ulRet = CPU_COMM_ERR_EVENT;
            goto ERROR;
    }

    if( psCommEntry->ulCommId == CPU_COMM_ID_ILLEGAL )
    {
        pr_err("CPUCOMM: %s CPU COMM ID illegal\n",__func__);		    
        ulRet = CPU_COMM_ERR_DESTROY;
        goto ERROR;
    }

    // Copy Respose Data
    if( pbyData != NULL )
    {
        // memcpy( pbyData, psCommEntry->sCommData.sData.pbyBuffer, ulDataSize );
        CpuComm_VolatileCopy( pbyData, psCommEntry->sCommData.sData.pbyBuffer, ulDataSize );
    }

    //unlock CPUB	
    psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;
	
    return CPU_COMM_ERR_NONE;
    
ERROR:
    CpuComm_DbgMsg( "comm wait: err, %d, %d\r\n", psCommEntry->ulCommId, ulRet );
    psGlobalSwapData->ulCommId = CPU_COMM_ID_ILLEGAL;
    return ulRet;	
    
}

static CPU_COMM_ERR CpuComm_CommAck( CPU_COMM_ENTRY* psCommEntry,
                                        MMP_UBYTE *pbyData,
                                        MMP_ULONG ulDataSize,
                                        MMP_ULONG ulTimeout )
{
    // use local data to send data
    CPU_COMM_SWAP_DATA sData;
    CPU_COMM_ERR        ulRet;

    // Prepare swap buffer
    sData.ulCommId    = psCommEntry->ulCommId;
    pr_debug( "Post: ID: %d\r\n", psCommEntry->ulCommId );

    if( pbyData != NULL )
    {
        CpuComm_VolatileCopy( sData.pbyBuffer, pbyData, ulDataSize );
    }
    // Swap Data
    ulRet = CpuComm_Swap( &sData );
    return ulRet;	
}


//------------------------------------------------------------------------------
//  Function    : CpuComm_CommPost
//  Description : prepare and send data to another CPU
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] psCommEntry:  pointer of comm data entry
 @param[in] pbyData:      data which will be sent to another CPU
 @param[in] ulDataSize:   data size
 @retval: CPU_COMM_ERR
*/
static CPU_COMM_ERR CpuComm_CommPost( CPU_COMM_ENTRY* psCommEntry,
                                        MMP_UBYTE *pbyData,
                                        MMP_ULONG ulDataSize,
                                        MMP_ULONG ulTimeout )
{
    // use local data to send data
    CPU_COMM_SWAP_DATA sData;
    CPU_COMM_ERR        ulRet;
    MMP_ULONG ulCurrent, ulStart;

    // Prepare swap buffer
    sData.ulCommId    = psCommEntry->ulCommId;
    // memset( sData.pbyBuffer, 0, SWAP_BUFFER_SIZE );
    if( pbyData != NULL )
    {
		pr_debug("%s: pbyData copy\n",__func__);
        CpuComm_VolatileCopy( sData.pbyBuffer, pbyData, ulDataSize );
    }

    // Swap Data
    ulRet = CpuComm_Swap( &sData );
    return ulRet;
}

//------------------------------------------------------------------------------
//  Function    : CpuComm_SemPost
//  Description : Post a semaphore to another CPU
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in]  ulCommId:  comm ID
 @param[in] ulTimeout: timeout count in millisecond, 0 is wait forever 
 @retval: CPU_COMM_ERR
*/
CPU_COMM_ERR CpuComm_SemPost( CPU_COMM_ID ulCommId,
                                MMP_ULONG ulTimeout )
{
    CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);
    CPU_COMM_ERR    ulRet;

    CpuComm_DbgMsg( "comm post: %d\r\n", ulCommId );

    if( psEntry == NULL || psEntry->ulCommId != ulCommId )
    {
        ulRet = CPU_COMM_ERR_INIT_FAIL;
        goto ERROR;
    }

    // Only flag can call this API
    if( psEntry->ulCommType != CPU_COMM_TYPE_SEM )
    {
        ulRet = CPU_COMM_ERR_UNSUPPORT;
        goto ERROR;
    }

    return CpuComm_CommPost( psEntry, NULL, 0, ulTimeout );

ERROR:
    CpuComm_DbgMsg( "comm post: err, %d, %d\r\n", ulCommId, ulRet );

    return ulRet;
}
EXPORT_SYMBOL(CpuComm_SemPost);
//------------------------------------------------------------------------------
//  Function    : CpuComm_FlagGet
//  Description : event flag for dual cpu communication
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in]  ulCommId:    comm ID
 @param[in]  ulTimeout:   timeout count
 @retval: CPU_COMM_ERR
*/
CPU_COMM_ERR CpuComm_SemWait( CPU_COMM_ID ulCommId,
                              MMP_ULONG ulTimeout )
{
    CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);
    CPU_COMM_ERR    ulRet;

    CpuComm_DbgMsg( "comm swait: %d\r\n", ulCommId );

    if( psEntry == NULL || psEntry->ulCommId != ulCommId )
    {
        ulRet = CPU_COMM_ERR_INIT_FAIL;
        goto ERROR;
    }

    // Only flag can call this API
    if( psEntry->ulCommType != CPU_COMM_TYPE_SEM )
    {
        ulRet = CPU_COMM_ERR_UNSUPPORT;
        goto ERROR;
    }

    return CpuComm_CommWait( psEntry, NULL, 0, ulTimeout );
    
ERROR:
    CpuComm_DbgMsg( "comm swait: err, %d, %d\r\n", ulCommId, ulRet );

    return ulRet;
}
EXPORT_SYMBOL(CpuComm_SemWait);

//------------------------------------------------------------------------------
//  Function    : CpuComm_DataSend
//  Description : Send data to another CPU
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] ulCommId:   comm ID
 @param[in] pbyData:    data for the service routine
 @param[in] ulDataSize: data size
 @param[in] ulTimeout: timeout count in millisecond, 0 is wait forever 
 @retval: CPU_COMM_ERR
*/
CPU_COMM_ERR CpuComm_DataSend( CPU_COMM_ID ulCommId,
                                  MMP_UBYTE *pbyData,
                                MMP_ULONG ulDataSize,
                                MMP_ULONG ulTimeout )
{
    CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);
    CPU_COMM_ERR    ulRet;

    CpuComm_DbgMsg( "comm snd: %d\r\n", ulCommId );

    if( psEntry == NULL || psEntry->ulCommId != ulCommId )
    {
      	 pr_err("cpucomm@%s: Comm ID is illegal\n",__func__);
      	 pr_err("cpucomm@%s: psEntry = 0x%x  \n",__func__,psEntry);
		 if(psEntry)
	      	 	pr_err("cpucomm@%s:  psEntry->ulCommId = %d\n",__func__,psEntry->ulCommId);

        ulRet = CPU_COMM_ERR_INIT_FAIL;
        goto ERROR;
    }
    // Only client can call this API
    if( psEntry->ulCommType != CPU_COMM_TYPE_DATA )
    {
    	 pr_err("cpucomm@%s: Comm Type unsupport\n",__func__);
        ulRet = CPU_COMM_ERR_UNSUPPORT;
        goto ERROR;
    }
    if( ulDataSize > SWAP_BUFFER_SIZE )
    {
    	 pr_err("cpucomm@%s: Data Size is exceed SWAP_BUFFER_SIZE(%d)\n",__func__,SWAP_BUFFER_SIZE);    
        ulRet = CPU_COMM_ERR_OVERFLOW;
        goto ERROR;
    }
    // Enter data critical section
    if( !CpuComm_DataCriticalEnter( psEntry ) )
    {
        ulRet = CPU_COMM_ERR_DESTROY;
        goto ERROR;
    }

    // Send data to antoher CPU
    if( ( ulRet = CpuComm_CommPost( psEntry, pbyData, ulDataSize, ulTimeout ) ) != CPU_COMM_ERR_NONE )
    {
        CpuComm_DataCriticalLeave( psEntry );
    	 pr_err("cpucomm@%s:Send data to antoher CPU fail. (%d)\n",__func__,ulRet);    
		
        goto ERROR;
    }

    // Wait for acknoledge
    ulRet = CpuComm_CommWaitAck( psEntry, NULL, 0, ulTimeout );

    //pr_info("CpuComm_SocketSend -\n");

    // Leave data critical section
    CpuComm_DataCriticalLeave( psEntry );

    return ulRet;
    
ERROR:
    CpuComm_DbgMsg( "comm send: err, %d, %d\r\n", ulCommId, ulRet );

    return ulRet;
}
EXPORT_SYMBOL(CpuComm_DataSend);

//------------------------------------------------------------------------------
//  Function    : CpuComm_DataReceive
//  Description : Receive data from another CPU
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in]  ulCommId:   comm ID
 @param[out] pbyData:    data for the service side
 @param[in]  ulDataSize: data size
 @param[in]  ulTimeout: timeout count in millisecond, 0 is wait forever 
 @param[in]  pfPreProc: callback is called before the acknowledge is returned
 @retval: CPU_COMM_ERR
*/
CPU_COMM_ERR CpuComm_DataReceive( CPU_COMM_ID ulCommId,
                                    MMP_UBYTE *pbyData,
                                  MMP_ULONG ulDataSize,
                                  MMP_ULONG ulTimeout,
                                  CPUCOMM_RCV_PREPROC pfPreProc )
{
    CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);
    CPU_COMM_ERR    ulRet;

    CpuComm_DbgMsg( "comm recv: %d\r\n", ulCommId );

    if( psEntry == NULL || psEntry->ulCommId != ulCommId )
    {
        ulRet = CPU_COMM_ERR_INIT_FAIL;
        goto ERROR;
    }

    // Only client can call this API
   
    if( psEntry->ulCommType != CPU_COMM_TYPE_DATA )
    {
	    pr_err("cpucomm: Comm type is unsupported. psEntry->ulCommType = %d\n",psEntry->ulCommType ); 	
        ulRet = CPU_COMM_ERR_UNSUPPORT;
        goto ERROR;
    }

    if( ulDataSize > SWAP_BUFFER_SIZE )
    {
		pr_err("cpucomm: ulDataSize %d is exceed SWAP_BUFFER_SIZE\n",ulDataSize );    

        ulRet = CPU_COMM_ERR_OVERFLOW;
        goto ERROR;
    }

    // Enter data critical section

    
    if( !CpuComm_DataCriticalEnter( psEntry ) )
    {
        pr_err("cpucomm: Enter critical section fail.\n");    
        ulRet = CPU_COMM_ERR_DESTROY;
        goto ERROR;
    }
    // Wait for receiving data    
    ulRet = CpuComm_CommWait( psEntry, pbyData, ulDataSize, ulTimeout );

    // Response an acknoledge
    if( ulRet != CPU_COMM_ERR_NONE )
    {
		pr_err("cpucomm: CpuComm_CommWait error %d \n",ulRet ); 	
        goto ERROR;
    }

    // Call preprocess callback
    if( pfPreProc != NULL )
        pfPreProc(ulCommId, pbyData, ulDataSize );
			
    {
        #if 1
        struct cpu_comm_transfer_data ack,*get_data = (struct cpu_comm_transfer_data *)pbyData ;
        ack.command = get_data->command ;
        ack.result = CPUCOMM_FLAG_RESULT_OK; 
        ack.flag = CPUCOMM_FLAG_ACK;
        ack.phy_addr = 0;
        ack.size = 0; 
        #endif
        ulRet = CpuComm_CommAck( psEntry, &ack,sizeof(struct cpu_comm_transfer_data), ulTimeout );
    }


ERROR:
    // Leave data critical section
    CpuComm_DataCriticalLeave( psEntry );

	if( ulRet != CPU_COMM_ERR_NONE )
	{
    	CpuComm_DbgMsg( "comm recv: err, %d, %d\r\n", ulCommId, ulRet );
	}

    return ulRet;
}
EXPORT_SYMBOL(CpuComm_DataReceive);

CPU_COMM_ERR CpuComm_ReceiveDataStart( CPU_COMM_ID ulCommId,
                                    MMP_UBYTE *pbyData,
                                    MMP_ULONG ulDataSize )
{
    CPU_COMM_ERR          ulErr;
	MMP_UBYTE *tmpBuf;
    CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);
    pr_debug("cpucomm: Socket Recevie %d %d\n",psEntry->ulCommId, ulCommId);
    if( psEntry == NULL || psEntry->ulCommId != ulCommId )
        return CPU_COMM_ERR_INIT_FAIL;

    // Only client can call this API
    pr_debug("cpucomm: psEntry->ulCommType = %d\n",psEntry->ulCommType );    
    if( psEntry->ulCommType != CPU_COMM_TYPE_DATA )
        return CPU_COMM_ERR_UNSUPPORT;

    if( ulDataSize > SWAP_BUFFER_SIZE )
    {
	pr_info("cpucomm: ulDataSize %d is exceed SWAP_BUFFER_SIZE\n",ulDataSize );    

        return CPU_COMM_ERR_OVERFLOW;
    }
    // Enter data critical section



    if( !CpuComm_DataCriticalEnter( psEntry ) )
    {
        pr_info("cpucomm: Enter critical section fail.\n");    
        return CPU_COMM_ERR_DESTROY;
    }

    pr_debug("Data wait + \r\n");
    // Wait for receiving data    
    ulErr = CpuComm_CommWait( psEntry, pbyData, ulDataSize, 0 );
    pr_debug("Data wait - \r\n");
	
    return ulErr;
}
EXPORT_SYMBOL(CpuComm_ReceiveDataStart);

CPU_COMM_ERR CpuComm_ReceiveDataEnd( CPU_COMM_ID ulCommId, unsigned long subCommandID)
{
	CPU_COMM_ERR          ulErr;
	MMP_UBYTE *tmpBuf;
	CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);


	struct cpu_comm_transfer_data data;

	data.command = subCommandID;
	data.result = CPUCOMM_FLAG_RESULT_OK; 
	data.flag = CPUCOMM_FLAG_ACK; 

	pr_debug("cpucomm: CpuComm_ReceiveDataEnd %d %d\n",psEntry->ulCommId, ulCommId);
	if( psEntry == NULL || psEntry->ulCommId != ulCommId )
		return CPU_COMM_ERR_INIT_FAIL;

	// Only client can call this API
	pr_debug("cpucomm: psEntry->ulCommType = %d\n",psEntry->ulCommType );    
	if( psEntry->ulCommType != CPU_COMM_TYPE_DATA )
		return CPU_COMM_ERR_UNSUPPORT;


	// Response an acknoledge
	ulErr = CpuComm_CommAck( psEntry, &data, sizeof(data), 0);

	// Leave data critical section
	CpuComm_DataCriticalLeave( psEntry );

    return ulErr;
}
EXPORT_SYMBOL(CpuComm_ReceiveDataEnd);


//------------------------------------------------------------------------------
//  Function    : CpuComm_RegisterEntry
//  Description : Register entry
//------------------------------------------------------------------------------
CPU_COMM_ERR CpuComm_RegisterEntry( CPU_COMM_ID ulCommId, CPU_COMM_TYPE ulCommType )
{
    CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);
    pr_info("cpucomm: Register Entry%d\n",ulCommId);

    if( psEntry == NULL || psEntry->ulCommId == ulCommId )
        return CPU_COMM_ERR_INIT_FAIL;

    // Set ID & Type
    psEntry->ulCommId = ulCommId;
    psEntry->ulCommType = ulCommType;

    // init data
    CpuComm_InitData( psEntry );

    return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_RegisterEntry);

//------------------------------------------------------------------------------
//  Function    : CpuComm_UnregisterEntry
//  Description : unregister entry
//------------------------------------------------------------------------------
CPU_COMM_ERR CpuComm_UnregisterEntry( CPU_COMM_ID ulCommId )
{
    CPU_COMM_ENTRY *psEntry = CpuComm_GetDataEntry(ulCommId);
    pr_info("cpucomm: Unregister Entry%d\n",ulCommId);
    if( psEntry == NULL || psEntry->ulCommId != ulCommId )
        return CPU_COMM_ERR_INIT_FAIL;

    CpuComm_DestroyData( psEntry );

    // memset( psEntry, 0, sizeof(CPU_COMM_ENTRY) );
    psEntry->ulCommId = CPU_COMM_ID_ILLEGAL;

    return CPU_COMM_ERR_NONE;
}
EXPORT_SYMBOL(CpuComm_UnregisterEntry);

//------------------------------------------------------------------------------
//  Function    : CpuComm_HwInit
//  Description : Init cpucomm HW
//                Current Implementation hard-code the register access
//                We should modify the CpuComm_HwInit() to recive the
//                register settings from ouside world.
//------------------------------------------------------------------------------
void CpuComm_HwInit( _CPU_ID ulCpuId , void* baseaddr)
{
    CPU_COMM_ID     i;
    CPU_COMM_ENTRY  *psEntry;

    CpuComm_DbgMsg( "comm init: %d\r\n", ulCpuId );

    // assign CPU ID
    s_ulCpuId        = ulCpuId;
    g_pHINT = baseaddr;
	
    // Reset data entry
    for( i = 0; i < CPUCOMM_ID_MAX_NUM; i++ )
    {
        psEntry = CpuComm_GetDataEntry(i);

        if( psEntry != NULL )
        {
            // memset( psEntry, 0, sizeof(CPU_COMM_ENTRY) );
            psEntry->ulCommId = CPU_COMM_ID_ILLEGAL;
        }
    }

    // Reset globabl swap buffer
     CpuComm_GetGlobalSwapData(ulCpuId)->ulCommId = CPU_COMM_ID_ILLEGAL;
}

//------------------------------------------------------------------------------
//  Function    : CPUComm_HwWaitCpuReady
//  Description : Wait the ready of another CPU
//------------------------------------------------------------------------------
/**
 Parameters:
 @param[in] ulCpuId:   Which CPU is operating
 @param[in] ulTimeout: timeout count in millisecond, 0 is wait forever 
 @retval: CPU_COMM_ERR
*/
CPU_COMM_ERR CPUComm_HwWaitCpuReady( _CPU_ID ulCpuId, MMP_ULONG ulTimeout )
{
#define READY_SLEEP_COUNT_MS   (50)
    CPU_COMM_SWAP_DATA*     psGlobalSwapData = CpuComm_GetGlobalSwapData(s_ulCpuId==_CPU_ID_A?_CPU_ID_B:_CPU_ID_A);
    MMP_ULONG               ulSleepCount;

    CpuComm_DbgMsg( "comm init wait: %d\r\n", ulCpuId );

    #if 0
    // Trigger IRQ to ensure ISR work normally
    // Linux request_irq() seems has some delay between request_irq()
    // is called to service a IRQ. Maybe we need this 
    // if the entry timeout is too short.
    MMP_CPUCOMM_IRQ_SET( s_psHintBase, s_ulCpuId );
    while( MMP_CPUCOMM_IRQ_CHECK( s_psHintBase, ulCpuId ) )
    {
        // sleep and count down the timeout
        MMPF_OS_Sleep(READY_SLEEP_COUNT_MS);
    }
    #endif

    // 0 is a special case, wait forever
    if( ulTimeout == 0 )
    {
        ulSleepCount = 0;
        ulTimeout = 0xFFFFFFFF;
    }
    else
    {
        ulSleepCount = READY_SLEEP_COUNT_MS;
        ulTimeout += READY_SLEEP_COUNT_MS - ulTimeout%READY_SLEEP_COUNT_MS; // force ulTimeout being a multiple of SLEEP_COUNT_MS
    }
    
    // Using sleep to wait the access of another CPU
    while( psGlobalSwapData->ulCommId == CPU_COMM_ID_CPU_B_NOT_READY )
    {
        // wait timeout?
        if( ulTimeout < READY_SLEEP_COUNT_MS )
        {
            CpuComm_DbgMsg( "comm cwait: timeout\r\n" );
            return CPU_COMM_ERR_TIMEOUT;
        }
    
        // sleep and count down the timeout
        MMPF_OS_Sleep(READY_SLEEP_COUNT_MS);
        ulTimeout -= ulSleepCount;
    }

    return CPU_COMM_ERR_NONE;
}

