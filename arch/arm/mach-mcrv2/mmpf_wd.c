//==============================================================================
//
//  File        : mmpf_wd.c
//  Description : CPU Watch Dog Control Interface
//  Author      : Ben Lu
//  Revision    : 1.0
//
//==============================================================================
/**
 *  @file mmpf_wd.c
 *  @brief The CPU Watch Dog Control Interface
 *  @author Ben Lu
 *  @version 1.0
 */

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include "includes_fw.h"
#include "config_fw.h"
#include "mmpf_typedef.h"
#include "lib_retina.h"
#include "mmp_err.h"
#include "reg_retina.h"
#include "mmpf_wd.h"
#include "mmpf_pll.h"

//==============================================================================
//
//                              VARIABLES
//
//==============================================================================

static WdCallBackFunc *WD_CallBackFunc = NULL;

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================

//------------------------------------------------------------------------------
//  Function    : MMPF_WD_ISR
//  Description :
//------------------------------------------------------------------------------
/** @brief The watch dog ISR handler function.

The watch dog ISR handler function.
*/
void MMPF_WD_ISR(void)
{
	AITPS_WD pWD = AITC_BASE_WD;

	//watch dog status is used to check what happened in this module, it
	//will not cause the next interrupt, so we don't need to clean it
	if(pWD->WD_SR & WD_OVF_SR) {
		RTNA_DBG_Str(0, "Watch dog Overflow \r\n");
		//pWD->WD_SR &= WD_OVERFLOW_SR;
	}
	
	#if 0
	//When watch dog time-out reset enable, we will have no enough time to ISR hander function.
	if(intsrc & WD_RESET_SR) {
		RTNA_DBG_Str(0, "Watch dog Reset \r\n");
		pWD->WD_SR &= WD_RESET_SR;
	}
	#endif
	
	if (WD_CallBackFunc) 
		(*WD_CallBackFunc)();
}

//------------------------------------------------------------------------------
//  Function    : MMPF_WD_GetStatus
//  Description :
//------------------------------------------------------------------------------
/** @brief To get watch dog overflow or reset status.

To get watch dog overflow or reset status.
@param[in] status : pointer which get the status.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_WD_GetStatus(MMP_ULONG *status)
{
	AITPS_WD pWD = AITC_BASE_WD;
	
	*status = pWD->WD_SR;
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_WD_Initialize
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used to regist CPU interrupt

 This function is used to regist CPU interrupt
@return It reports the status of the operation.
*/
MMP_ERR MMPF_WD_Initialize(void)
{
    #if (OS_TYPE == OS_UCOSII)
	AITPS_AIC   pAIC = AITC_BASE_AIC;
	static MMP_BOOL bWDInitFlag = MMP_FALSE;
	
	if (bWDInitFlag == MMP_FALSE) {
		bWDInitFlag = MMP_TRUE;
        #if (CHIP == MCR_V2)
        RTNA_AIC_Open(pAIC, AIC_SRC_WD_INNER, wd_isr_a,
	                AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 3);
        RTNA_AIC_IRQ_En(pAIC, AIC_SRC_WD_INNER);
        #endif
	}
    #endif
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_WD_EnableWD
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used to enable watch dog and its related function.

This function is used to enable watch dog and its related function.
@param[in] bEnable : enable/disable watch dog.
@param[in] bResetCPU : enable/disable watch dog reset CPU function.
@param[in] bEnableInterrupt : enable/disable watch dog interrupt mode.
@param[in] CallBackFunc : the callback function when interrupt occurs.
@param[in] bRestetAllModules : when Reset CPU enable & watch dog timeout, 
                               we can decide to reset all module or not.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_WD_EnableWD(MMP_BOOL bEnable, MMP_BOOL bResetCPU, 
                        MMP_BOOL bEnableInterrupt, WdCallBackFunc *CallBackFunc,
                        MMP_BOOL bRestetAllModules)
{
	AITPS_WD 	pWD 	= AITC_BASE_WD;
	#if (CHIP == VSN_V3)
	AITPS_GBL   pGBL 	= AITC_BASE_GBL;
	#endif
	#if (CHIP == MCR_V2)
	AITPS_CORE  pCORE   = AITC_BASE_CORE;
	#endif
	MMP_ULONG   ulValue = WD_CTL_ACCESS_KEY;

	if (bEnable) {
		if (!(bResetCPU | bEnableInterrupt)) {
			RTNA_DBG_Str(0, "Note: Strange Watch dog settings\r\n");
		}

		if (bEnableInterrupt) {
			WD_CallBackFunc = CallBackFunc;
			ulValue |= WD_INT_EN;
		}

		if (bResetCPU) {
			ulValue |= WD_RST_EN;

			if (bRestetAllModules) {
			    #if (CHIP == VSN_V3)
				pGBL->GBL_CHIP_CTL |= GBL_WD_RST_ALL_MODULE;
				#endif
				#if (CHIP == MCR_V2)
				pCORE->CORE_A_WD_CFG |= WD_RST_ALL_CPU_MDL_OPR;
				#endif
			}
		}

		ulValue |= WD_EN;
		pWD->WD_RE_ST = WD_RESTART; //Before enable, we shoudl set re-start first.
	}
	else {
		WD_CallBackFunc = NULL;
	}

	pWD->WD_MODE_CTL0 = ulValue;

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_WD_SetTimeOut
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used to adjust watch dog timeout interval.

This function is used to adjust watch dog timeout interval.
@param[in] ubCounter : timeout counter, the count tick = ubCounter x 16384.
@param[in] clockDiv : choose the input clock divider.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_WD_SetTimeOut(MMP_UBYTE ubCounter, MMP_USHORT clockDiv)
{
	AITPS_WD 	pWD = AITC_BASE_WD;
	MMP_ULONG 	ulValue = WD_CLK_CTL_ACCESS_KEY;

	if (ubCounter >= 32) {
		RTNA_DBG_Str(0, "Set CPU Watch dog counter Error\r\n");
		return MMP_ERR_NONE;
	}

	ulValue |= (ubCounter << 2);

    /* for P_V2 (external crystal clock (24MHz) / clockDiv)
     * for MCR_V2 (G0_slow (G0 clk/2) / clockDiv) */
	switch(clockDiv) {
	case 8:
		ulValue |= WD_CLK_MCK_D8;
		break;
	case 32:
		ulValue |= WD_CLK_MCK_D32;
		break;
	case 128:
		ulValue |= WD_CLK_MCK_D128;
		break;
	case 1024:
		ulValue |= WD_CLK_MCK_D1024;
		break;
	default:
		RTNA_DBG_Str(0, "Set CPU Watch dog CLK divider Error\r\n");
		return MMP_ERR_NONE;
		break;
	}

	pWD->WD_MODE_CTL1 = ulValue;
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_WD_Kick
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used to kick watch dog.

This function is used to kick watch dog.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_WD_Kick(void)
{
	AITPS_WD pWD = AITC_BASE_WD;

	pWD->WD_RE_ST = WD_RESTART;

	return MMP_ERR_NONE;
}

MMP_ULONG MMPF_WD_GetFreq(void)
{
    MMP_ULONG ClkFreq = 0;

    #if (CHIP == VSN_V3) || (CHIP == MERCURY) || (CHIP == MCR_V2)
    if(MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ClkFreq) != MMP_ERR_NONE) {
        return MMP_SYSTEM_ERR_SETPLL;
    }
    ClkFreq >>= 1;
    #endif

    return ClkFreq;
}

MMP_USHORT MMPF_WD_GetTimeOut(void)
{
	AITPS_WD 	pWD = AITC_BASE_WD;

	return (pWD->WD_MODE_CTL1 >> 2);
}
/** @} */ // MMPF_WD
