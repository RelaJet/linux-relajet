//==============================================================================
//
//  File        : mmpf_timer.h
//  Description : INCLUDE File for the Firmware Timer Control Driver
//  Author      : Jerry Lai
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPF_TIMER_H_
#define _MMPF_TIMER_H_

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

typedef enum _MMPF_TIMER_ID
{
	MMPF_TIMER_0 =0,
	MMPF_TIMER_1,
	MMPF_TIMER_2,
	#if (CHIP == MCR_V2) || (CHIP == MERCURY)
    MMPF_TIMER_3,
    MMPF_TIMER_4,
    MMPF_TIMER_5,
    #endif
	MMPF_TIMER_MAX
} MMPF_TIMER_ID;

typedef enum _MMPF_TIMER_EVT_MODE {
    MMPF_TIMER_EVT_PERIODIC = 0,
    MMPF_TIMER_EVT_ONESHOT,
    MMPF_TIMER_EVT_NONE
} MMPF_TIMER_EVT_MODE;

typedef enum _MMPF_TIMER_MCLK_DIV {
    MMPF_TIMER_MCLK_D2 = 0,
    MMPF_TIMER_MCLK_D8,
    MMPF_TIMER_MCLK_D32,
    MMPF_TIMER_MCLK_D128,
    MMPF_TIMER_MCLK_D1024,
    MMPF_TIMER_MCLK_D4,
    MMPF_TIMER_MCLK_D16,
    MMPF_TIMER_MCLK_D1,
    MMPF_TIMER_MCLK_DIV_NUM
} MMPF_TIMER_MCLK_DIV;

typedef enum _MMPF_TIMER_PRCN {
    MMPF_TIMER_PRCN_MSEC = 0,
    MMPF_TIMER_PRCN_USEC,
    MMPF_TIMER_PRCN_TICK
} MMPF_TIMER_PRCN;

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef void TimerCallBackFunc(void);              //New for callback function

typedef struct _MMPF_TIMER_ATTRIBUTE {
    TimerCallBackFunc       *Callback;
    MMPF_TIMER_EVT_MODE     EventMode;
    MMPF_TIMER_MCLK_DIV     MClkDiv;
    MMPF_TIMER_PRCN         TimeUnit;
    MMP_ULONG               ulDelayCnt;
    MMP_BOOL                bEnableTick;
} MMPF_TIMER_ATTRIBUTE;

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#include "reg_retina.h"

#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define MMPF_Timer_ReadCount(_id)           (((AITPS_TC)(&(AITC_BASE_TCB->TC[_id])))->TC_CVR)
#define MMPF_Timer_ReadCompareValue(_id)    (((AITPS_TC)(&(AITC_BASE_TCB->TC[_id])))->TC_RC)
#define MMPF_Timer_IsOnTime(_id)            (!!((((AITPS_TC)(&(AITC_BASE_TCB->TC[_id])))->TC_SR)&TC_CPCS))
#endif
#if (CHIP == MCR_V2)
#define MMPF_Timer_ReadCount(_id)           (((_id) < MMPF_TIMER_3)? \
                                                (((AITPS_TC)(&(AITC_BASE_TCB->TC0_2[_id])))->TC_CVR): \
                                                (((AITPS_TC)(&(AITC_BASE_TCB->TC3_5[_id])))->TC_CVR))
#define MMPF_Timer_ReadCompareValue(_id)    (((_id) < MMPF_TIMER_3)? \
                                                (((AITPS_TC)(&(AITC_BASE_TCB->TC0_2[_id])))->TC_RC): \
                                                (((AITPS_TC)(&(AITC_BASE_TCB->TC3_5[_id])))->TC_RC))
#define MMPF_Timer_IsOnTime(_id)            (((_id) < MMPF_TIMER_3)? \
                                                (!!((((AITPS_TC)(&(AITC_BASE_TCB->TC0_2[_id])))->TC_SR)&TC_CPCS)): \
                                                (!!((((AITPS_TC)(&(AITC_BASE_TCB->TC3_5[_id])))->TC_SR)&TC_CPCS)))
#endif

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

//New API: To set AIC source and assign callback function. It will call SetAICISR()
//call RTNA_AIC.... to set the interrupt source.
MMP_ERR MMPF_Timer_OpenInterrupt(MMPF_TIMER_ID id);             

//New API: To replace following 2 APIs with this one
//RTNA_AIC_IRQ_En or RTNA_AIC_IRQ_Dis to turn on the AIC interrupt status
//and then turn on Timer interrupt in Timer module. (Merge MMPF_TimerEnableIRQ function)
MMP_ERR MMPF_Timer_EnableInterrupt(MMPF_TIMER_ID id, MMP_BOOL bEnable);

void MMPF_TIMER_ISR(MMPF_TIMER_ID id);
void MMPF_Timers_ISR(void);

//FW is urge to use MMPF_Timer_OpenMS insted of MMPF_Timer_Open. 
//OpenMS will get CPU frequency and calculate the delay
MMP_ERR MMPF_Timer_Open(MMPF_TIMER_ID id, MMPF_TIMER_ATTRIBUTE *TmrAttr);
MMP_ERR MMPF_Timer_Close(MMPF_TIMER_ID id);
MMP_ERR MMPF_Timer_EnableTick(MMPF_TIMER_ID id, MMP_BOOL en);
MMP_ERR MMPF_Timer_RestartTick(MMPF_TIMER_ID id, MMP_BOOL bAdjustTickCnt, MMP_ULONG ulNextTickCnt);
MMP_ULONG MMPF_Timer_CalculateTickFreq(MMPF_TIMER_MCLK_DIV MClkDiv);


#endif // _MMPF_TIMER_H_
