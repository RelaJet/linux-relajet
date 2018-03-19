//==============================================================================
//
//  File        : mmpf_timer.c
//  Description : Firmware Timer Control Function
//  Author      : Jerry Lai
//  Revision    : 1.0
//
//==============================================================================

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include "includes_fw.h"
#include "mmpf_timer.h"
#include "lib_retina.h"
#include "mmpf_pll.h"

//==============================================================================
//
//                              VARIABLES
//
//==============================================================================
static MMP_BOOL  m_bTimerEnabled[MMPF_TIMER_MAX];
static MMPF_TIMER_EVT_MODE  TimerEventMode[MMPF_TIMER_MAX];


#if (OS_TYPE == OS_UCOSII)
static MMP_BOOL  m_bTimerIntOpened[MMPF_TIMER_MAX] = {MMP_FALSE, };
#endif
static MMP_BOOL  m_bTimerIntEnabled[MMPF_TIMER_MAX] = {MMP_FALSE, };

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================
static TimerCallBackFunc *TimerCallBacks[MMPF_TIMER_MAX] = {NULL, };

////------------------------------------------------------------------------------
//  Function    : Return the AITPS_TC of given id
//  Description : Helper function to get correct pTC from ID
//  Note        : Because timer hardware is split into 2 group. The ID
//                to pTC is handled here
//  Return      :
//------------------------------------------------------------------------------
static __inline AITPS_TC GetpTC(MMPF_TIMER_ID id)
{
    AITPS_TCB pTCB = AITC_BASE_TCB;

    #if (CHIP == MERCURY) || (CHIP == VSN_V3)
    if (id < MMPF_TIMER_MAX) {
        return ((AITPS_TC)(&(pTCB->TC[id])));
    }
    #endif
	#if (CHIP == MCR_V2)
	if (id < MMPF_TIMER_3) {
		return ((AITPS_TC)(&(pTCB->TC0_2[id])));
	}
	else if (id < MMPF_TIMER_MAX) {
		return ((AITPS_TC)(&(pTCB->TC3_5[id - MMPF_TIMER_3])));
	}
	#endif
    else {
        RTNA_DBG_Str(0, "ERROR: TIMER ID IS INVALID\r\n");
        return 0;
    }
}

//------------------------------------------------------------------------------
//  Function    : MMPF_TimerInternal_EnableAICTimerSRC
//  Description :
//  Note        :
//------------------------------------------------------------------------------
static MMP_ERR MMPF_TimerInternal_EnableAICTimerSRC(MMPF_TIMER_ID id, MMP_BOOL bEnable)
{
    #if (OS_TYPE == OS_UCOSII)
    AITPS_AIC pAIC = AITC_BASE_AIC;

    if (bEnable == MMP_TRUE)
    	RTNA_AIC_IRQ_En(pAIC, AIC_SRC_TC(id));
    else
    	RTNA_AIC_IRQ_Dis(pAIC, AIC_SRC_TC(id));

    #endif
    return MMP_ERR_NONE;
}

/**
@brief  To set AIC interrupt source
@param[in]  id   The timer you want to use as interrupt source
*/
MMP_ERR MMPF_Timer_OpenInterrupt(MMPF_TIMER_ID id)
{
    #if (OS_TYPE == OS_UCOSII)
    AITPS_AIC pAIC = AITC_BASE_AIC;


    if (m_bTimerIntOpened[id] == MMP_TRUE)
        return MMP_ERR_NONE;
    m_bTimerIntOpened[id] = MMP_TRUE;

    RTNA_AIC_Open(pAIC, AIC_SRC_TC(id), tcs_isr_a, AIC_INT_TO_IRQ |
                        AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 2);
    #endif


	return MMP_ERR_NONE;
}

/**
@brief  Start to recieve timer IRQ
@param[in]  id   The timer you want to use as interrupt source
@param[in]  bEnable  1 start the timer IRQ. 0 stop the timer IRQ
*/
MMP_ERR	MMPF_Timer_EnableInterrupt(MMPF_TIMER_ID id, MMP_BOOL bEnable)
{
	AITPS_TC pTC = GetpTC(id);

	if (bEnable == MMP_TRUE) {
        if (m_bTimerEnabled[id] == MMP_FALSE) {
		    //Enable AIC's Timer Source

		    if (m_bTimerIntEnabled[id] == MMP_FALSE) {
                MMPF_TimerInternal_EnableAICTimerSRC(id, MMP_TRUE);
                m_bTimerIntEnabled[id] = MMP_TRUE;
		    }

		    m_bTimerEnabled[id] = MMP_TRUE;
		}
		//clear counter to 0
		pTC->TC_CVR = 0;
		//Enable Timer IRQ
		pTC->TC_SR = pTC->TC_SR; //clear SR
		pTC->TC_IER = TC_CPCS;   //enable compare interrupt
	}
	else {
        if (m_bTimerEnabled[id] == MMP_TRUE) {
            //Disable AIC's Timer Source

	        if (m_bTimerIntEnabled[id] == MMP_TRUE) {
		        MMPF_TimerInternal_EnableAICTimerSRC(id, MMP_FALSE);
                m_bTimerIntEnabled[id] = MMP_FALSE;
		    }

		    m_bTimerEnabled[id] = MMP_FALSE;
	    }
		//Disable Timer IRQ
		pTC->TC_IDR = TC_CPCS;    //disable compare interrupt
		pTC->TC_SR = pTC->TC_SR;  //clear SR
	}

	return MMP_ERR_NONE;
}

/**
@brief  Set Timer triggered time, and set timer callback function, and flow mode

@param[in]  id   The timer you want to use as interrupt source
@param[in]  TmrAttr The timer related attributes
*/
const static MMP_UBYTE m_TimerMClkDivMap[MMPF_TIMER_MCLK_DIV_NUM] = {
    TC_CLK_MCK_D2, TC_CLK_MCK_D8, TC_CLK_MCK_D32, TC_CLK_MCK_D128,
    TC_CLK_MCK_D1024, TC_CLK_MCK_D4, TC_CLK_MCK_D16, TC_CLK_MCK
};
MMP_ERR MMPF_Timer_Open(MMPF_TIMER_ID id, MMPF_TIMER_ATTRIBUTE *TmrAttr)
{
	MMP_ULONG count;
	MMP_ULONG mode;
	MMP_ULONG TimerClkFreq = 0;
	AITPS_TC  pTC = GetpTC(id); //Get correct Timer

    #if (CHIP == VSN_V3) || (CHIP == MERCURY) || (CHIP == MCR_V2)
    if(MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &TimerClkFreq) != MMP_ERR_NONE) {
        return MMP_SYSTEM_ERR_SETPLL;
    }
    TimerClkFreq >>= 1;
    #endif

    if (TimerClkFreq == 0) {
        RTNA_DBG_Str(0, "ERROR : FW PLL not set.");
        return MMP_SYSTEM_ERR_SETPLL;
    }

    switch (TmrAttr->TimeUnit) {
	case MMPF_TIMER_PRCN_TICK:
	    count = TmrAttr->ulDelayCnt;
	    break;
    case MMPF_TIMER_PRCN_MSEC:
        //(1/Timer FREQ) * count = n * (1/1000)
        count = TmrAttr->ulDelayCnt * TimerClkFreq;
        break;
    case MMPF_TIMER_PRCN_USEC:
        //(1/Timer FREQ) * count = n * (1/1000000)
        count = TmrAttr->ulDelayCnt * (TimerClkFreq/1000);
        break;
    default:
        return MMP_SYSTEM_ERR_TIMER;
	}

	mode = TC_CPCTRG | m_TimerMClkDivMap[TmrAttr->MClkDiv];
	//should check if the delay is not too long for a timer

    // Start the Timer and set compare value
    pTC->TC_CCR = TC_CLKDIS;  //Stop Timer
    pTC->TC_IDR = TC_CPCS;    //Disable Interrupt mode

	pTC->TC_SR =  pTC->TC_SR; //Clear Status Register
    pTC->TC_CMR = mode;       //Set mode
    pTC->TC_RC = count;       //Set Compare value

    //Save the callback
    TimerCallBacks[id] = TmrAttr->Callback;
    TimerEventMode[id] = TmrAttr->EventMode;

    if (TmrAttr->bEnableTick) {
        //pTC->TC_CCR = TC_CLKEN;   //Start Timer again
        pTC->TC_CCR |= TC_SWTRG;  //Restart Timer
    }

	return MMP_ERR_NONE;
}

/**
@brief  Stop timer triggered time in ms, and clear timer callback function

@param[in]  id   The timer you want to use as interrupt source
*/
MMP_ERR MMPF_Timer_Close(MMPF_TIMER_ID id)
{
	AITPS_TC pTC = GetpTC(id);

	//Stop timer
	pTC->TC_CCR = TC_CLKDIS;
	pTC->TC_IDR = TC_CPCS;

	//clear callback
	TimerCallBacks[id] = NULL;

	return MMP_ERR_NONE;
}


/**
@brief  Start/stop timer counting, not start from zero.

@param[in]  id   The timer you want to use as interrupt source
*/
MMP_ERR MMPF_Timer_EnableTick(MMPF_TIMER_ID id, MMP_BOOL en)
{
    AITPS_TC pTC = GetpTC(id);

    pTC->TC_CCR = (en? TC_CLKEN: TC_CLKDIS);

    return MMP_ERR_NONE;
}

/**
@brief  Start/stop timer counting, not start from zero.

@param[in]  id   The timer you want to use as interrupt source
*/
MMP_ERR MMPF_Timer_RestartTick(MMPF_TIMER_ID id, MMP_BOOL bAdjustTickCnt, MMP_ULONG ulNextTickCnt)
{
    AITPS_TC pTC = GetpTC(id);

    pTC->TC_RC = ulNextTickCnt;

    pTC->TC_CCR = TC_SWTRG;

    return MMP_ERR_NONE;
}

/**
@brief  Calculates the ticks freq. per sec. based on given MCLK divider

@param[in]  id   The timer you want to use as interrupt source
@param[in]  MClkDiv   The MCLK divider 
*/
MMP_ULONG MMPF_Timer_CalculateTickFreq(MMPF_TIMER_MCLK_DIV MClkDiv)
{
    const static MMP_UBYTE m_TimerMClkShiftMap[MMPF_TIMER_MCLK_DIV_NUM] = {
        1/*TC_CLK_MCK_D2*/,     3/*TC_CLK_MCK_D8*/,     5/*TC_CLK_MCK_D32*/,
        7/*TC_CLK_MCK_D128*/,   10/*TC_CLK_MCK_D1024*/, 2/*TC_CLK_MCK_D4*/,
        4/*TC_CLK_MCK_D16*/,    0/*TC_CLK_MCK*/
    };
    MMP_ULONG TimerClkFreq = 0;

    #if (CHIP == VSN_V3) || (CHIP == MERCURY) || (CHIP == MCR_V2)
    if(MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &TimerClkFreq) != MMP_ERR_NONE) {
        return MMP_SYSTEM_ERR_SETPLL;
    }
    TimerClkFreq = ((TimerClkFreq * 1000) >> 1);
    #endif

    return (TimerClkFreq >> m_TimerMClkShiftMap[MClkDiv]);
}


////------------------------------------------------------------------------------
//  Function    : MMPF_TIMERX_ISR
//  Description : The ISR operation. Do the job of clear AIC. And call the callback
//  Note        :
//  Return      :
//------------------------------------------------------------------------------
void MMPF_TIMER_ISR(MMPF_TIMER_ID id)
{
    AITPS_TC  pTC;
    #if (OS_TYPE == OS_UCOSII)
    AITPS_AIC pAIC = AITC_BASE_AIC;
    #endif

    pTC = GetpTC(id);

    if (pTC->TC_SR & TC_CPCS) {
        if (TimerEventMode[id] == MMPF_TIMER_EVT_ONESHOT) {
            //pTC->TC_IDR = TC_CPCS;
            pTC->TC_CCR = TC_CLKDIS;
        }

        pTC->TC_SR = pTC->TC_SR; //clear the SR

        #if (OS_TYPE == OS_UCOSII)
		#if (CHIP == VSN_V3)
		pAIC->AIC_ICCR = 0x1 << (AIC_SRC_TC(id));
		#endif
		#if (CHIP == MCR_V2) || (CHIP == MERCURY)
		// Clear TC Interrupt on AIC
		if (i < 3)
		    pAIC->AIC_ICCR_LSB = 0x1 << (AIC_SRC_TC(id));
		else
		    pAIC->AIC_ICCR_MSB = 0x1 << (AIC_SRC_TC(id) - 0x20);
		#endif
        #endif // (OS_TYPE == OS_UCOSII)

	    //Execute the Timer callback
	    if (m_bTimerEnabled[id] && TimerCallBacks[id]) {
	        TimerCallBacks[id]();
	    }
    }

}

////------------------------------------------------------------------------------
//  Function    : RTNA_TIMERS_ISR
//  Description : The ISR operation. Clear AIC and call the callback
//  Note        :
//  Return      :
//------------------------------------------------------------------------------
#if (OS_TYPE == OS_UCOSII)
void MMPF_Timers_ISR(void)
{
	//Check All Timer
	for(i = 1; i < MMPF_TIMER_MAX; i++) {
        MMPF_TIMER_ISR(i);
	}
}
#endif

