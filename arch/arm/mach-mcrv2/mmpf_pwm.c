//==============================================================================
//
//  File        : mmpf_pwm.c
//  Description : PWM control driver
//  Author      : Ben Lu
//  Revision    : 1.0
//
//==============================================================================
/**
 *  @file mmpf_pwm.c
 *  @brief PWM control driver
 *  @author Ben Lu
 *  @version 1.0
 */

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================
 
#include <mach/includes_fw.h>
#include <mach/config_fw.h>
#include <mach/mmpf_typedef.h>
#include <mach/lib_retina.h>
//#include "reg_retina.h"
#include <mach/mmp_register.h>
#include <mach/mmp_err.h>
#include <mach/os_wrap.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_gpio.h>
#include <mach/mmp_reg_vif.h>
#include <mach/mmpf_pio.h>
#include <mach/mmpf_pwm.h>
#include <mach/mmpf_system.h>
#include <mach/mmpf_pll.h>

//==============================================================================
//
//                              GLOBAL VARIABLE
//
//==============================================================================

static PwmCallBackFunc *gPwmCallBack[PWM_MAX_NUM];
static MMPF_OS_SEMID  	gPWMSemID[PWM_MAX_NUM];

//------------------------------------------------------------------------------
//  Function    : MMPF_PWM_ISR
//  Description : 
//------------------------------------------------------------------------------
/** @brief PWM interrupt handler function.

PWM interrupt handler function.
@return none
*/
void MMPF_PWM_ISR(void)
{
    AITPS_PWMB  pPWM = (AITPS_PWMB)AITC_BASE_PWM;
    MMP_ULONG   i = 0;
    MMP_ULONG   status = 0x0;
    
    //ISR of PWM
   	for (i = 0; i < PWM_MAX_NUM_P0; i++) {
	   	if (pPWM->PWM0[i].PWM_INT_CPU_SR != 0 ) {  
	   	 	status = pPWM->PWM0[i].PWM_INT_CPU_SR;
		   	if (gPwmCallBack[i] != NULL) {
		   		(*gPwmCallBack[i])(status);
		   	}
		   	pPWM->PWM0[i].PWM_INT_CPU_SR |= status; //Clean status
		   	break;
	   	}
   	} 
   	
   	for (i = 0; i < PWM_MAX_NUM_P1; i++) {
	   	if (pPWM->PWM1[i].PWM_INT_CPU_SR != 0 ) {  
	   	 	status = pPWM->PWM1[i].PWM_INT_CPU_SR;
		   	if (gPwmCallBack[i + PWM_MAX_NUM_P0] != NULL) {
		   		(*gPwmCallBack[i + PWM_MAX_NUM_P0])(status);
		   	}
		   	pPWM->PWM1[i].PWM_INT_CPU_SR |= status; //Clean status
		   	break;
	   	}
   	}
}

////----------------------------------------------------------------------------
//  Function    : GetpPWM
//  Description : Return the AITPS_PWM of the given id
//  Note        : Because we have 18 PWM engines.
//  Return      :
//------------------------------------------------------------------------------
static AITPS_PWM GetpPWM(MMP_UBYTE id)
{
    AITPS_PWMB  pPWMB = (AITPS_PWMB)AITC_BASE_PWM;

    if (id >= PWM_MAX_NUM) {
	    RTNA_DBG_Str(0, "ERR: Invalid PWM ID\r\n");
	    return NULL;
	}

    if (id < PWM_MAX_NUM_P0)
        return &pPWMB->PWM0[id];
    else
        return &pPWMB->PWM1[id - PWM_MAX_NUM_P0];
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PWM_Initialize
//  Description : 
//------------------------------------------------------------------------------
/** @brief Driver init

Driver init
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PWM_Initialize(void)
{
	MMP_UBYTE i = 0;
	#if (OS_TYPE == OS_UCOSII)
	AITPS_AIC pAIC = AITC_BASE_AIC;
	#endif
	static MMP_BOOL ubInitFlag = MMP_FALSE;

	if (ubInitFlag == MMP_FALSE) 
	{
    #if (OS_TYPE == OS_UCOSII)
        RTNA_AIC_Open(pAIC, AIC_SRC_PWM, pwm_isr_a,
                    AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 7);
        RTNA_AIC_IRQ_En(pAIC, AIC_SRC_PWM);
    #endif
    for(i = 0; i < PWM_MAX_NUM; i++) {
        gPwmCallBack[i] = NULL;
        gPWMSemID[i]    = MMPF_OS_CreateSem(1);
    }
    ubInitFlag = MMP_TRUE;
  }
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PWM_SetAttribe
//  Description : 
//------------------------------------------------------------------------------
/** @brief Driver init
Parameters:
@param[in] pwm_attribute : PWM attribute, please refer the structure MMPF_PWM_ATTRIBUTE
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PWM_SetAttribe(MMPF_PWM_ATTRIBUTE* pwm_attribute)
{
    AITPS_PWM   pPWM = GetpPWM(pwm_attribute->ubID);
	MMP_UBYTE   ret = 0xFF;

    if (pPWM == NULL)
        return MMP_PWM_ERR_PARAMETER;

	MMPF_SYS_EnableClock(MMPF_SYS_CLK_PWM, MMP_TRUE);

	ret = MMPF_OS_AcquireSem(gPWMSemID[pwm_attribute->ubID], PWM_SEM_TIMEOUT);

	if (pwm_attribute->uPulseID == MMPF_PWM_PULSE_ID_A) {
		pPWM->PWM_PULSE_A_T0 = pwm_attribute->ulClkDuty_T0;
		pPWM->PWM_PULSE_A_T1 = pwm_attribute->ulClkDuty_T1;
		pPWM->PWM_PULSE_A_T2 = pwm_attribute->ulClkDuty_T2;
		pPWM->PWM_PULSE_A_T3 = pwm_attribute->ulClkDuty_T3;
		pPWM->PWM_PULSE_A_PEROID = pwm_attribute->ulClkDuty_Peroid;
		pPWM->PWM_PULSE_A_NUM = pwm_attribute->ubNumOfPulses;
	}
	else if (pwm_attribute->uPulseID == MMPF_PWM_PULSE_ID_B) {
		pPWM->PWM_PULSE_B_T0 = pwm_attribute->ulClkDuty_T0;
		pPWM->PWM_PULSE_B_T1 = pwm_attribute->ulClkDuty_T1;
		pPWM->PWM_PULSE_B_T2 = pwm_attribute->ulClkDuty_T2;
		pPWM->PWM_PULSE_B_T3 = pwm_attribute->ulClkDuty_T3;
		pPWM->PWM_PULSE_B_PEROID = pwm_attribute->ulClkDuty_Peroid;
		pPWM->PWM_PULSE_B_NUM = pwm_attribute->ubNumOfPulses;
	}
    
	if (ret == 0) {
		MMPF_OS_ReleaseSem(gPWMSemID[pwm_attribute->ubID]);
	}
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PWM_EnableInterrupt
//  Description : 
//------------------------------------------------------------------------------
/** @brief Driver init
Parameters:
@param[in] uID : PWM ID
@param[in] bEnable : enable/disable interrupt
@param[in] CallBackFunc : call back function when interrupt happens.
@param[in] interupt_item : PWM interupt type.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PWM_EnableInterrupt(MMP_UBYTE ubID, MMP_BOOL bEnable, PwmCallBackFunc* CallBackFunc, MMPF_PWM_INT interupt_item)
{
	AITPS_PWM   pPWM = GetpPWM(ubID);
    MMP_UBYTE 	ret = 0xFF;

	if (pPWM == NULL)
        return MMP_PWM_ERR_PARAMETER;

	MMPF_SYS_EnableClock(MMPF_SYS_CLK_PWM, MMP_TRUE);

	ret = MMPF_OS_AcquireSem(gPWMSemID[ubID], PWM_SEM_TIMEOUT);

	if (bEnable == MMP_TRUE) {
		gPwmCallBack[ubID] = CallBackFunc;
		pPWM->PWM_INT_CPU_EN |= (0x1 << interupt_item);
	}
	else {
		pPWM->PWM_INT_CPU_EN &= ~(0x1 << interupt_item);
		gPwmCallBack[ubID] = NULL;
	}

	if (ret == 0) {
		MMPF_OS_ReleaseSem(gPWMSemID[ubID]);
	}

	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PWM_ControlSet
//  Description : Set PWM control attribute
//------------------------------------------------------------------------------
/** @brief Set PWM control attribute
Parameters:
@param[in] ubID : PWM ID
@param[in] control : control attribute               
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PWM_ControlSet(MMP_UBYTE ubID, MMP_UBYTE control)
{
    AITPS_PWM   pPWM = GetpPWM(ubID);
    MMP_UBYTE 	ret = 0xFF;

	if (pPWM == NULL)
        return MMP_PWM_ERR_PARAMETER;

	MMPF_SYS_EnableClock(MMPF_SYS_CLK_PWM, MMP_TRUE);

	ret = MMPF_OS_AcquireSem(gPWMSemID[ubID], PWM_SEM_TIMEOUT);

    pPWM->PWM_CTL = control;

	if (ret == 0) {
		MMPF_OS_ReleaseSem(gPWMSemID[ubID]);
	}
	
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PWM_EnableOutputPin
//  Description : 
//------------------------------------------------------------------------------
/** @brief enable/disable the PWM single output pin

enable/disable the PWM single output pin
@param[in] pwm_pin : PWM I/O pin selection, please refer MMPF_PWMPIN
@param[in] bEnable : enable/disable the specific PWM pin
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PWM_EnableOutputPin(MMPF_PWMPIN pwm_pin, MMP_BOOL bEnable)
{
	AITPS_GBL pGBL = AITC_BASE_GBL;
	
	if (bEnable) 
	{
        if (pwm_pin < MMPF_PWMPIN_MAX) {
            pGBL->GBL_PWM_IO_CFG |= (0x1 << PWN_GET_PIN_OFST(pwm_pin));
        }
        else {
            RTNA_DBG_Str(0, "Unsupport PWM IO\r\n");
        }
	}
	else {
	
        if (pwm_pin < MMPF_PWMPIN_MAX) {
            pGBL->GBL_PWM_IO_CFG &= ~(0x1 << PWN_GET_PIN_OFST(pwm_pin));
        }
        else {
            RTNA_DBG_Str(0, "Unsupport PWM IO\r\n");
        }
	}
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PWM_OutputPulse
//  Description : 
//------------------------------------------------------------------------------
/** @brief Simplely output some pulses (According to the parameters)

Simplely output some pulses (According to the parameters)
@param[in] pwm_pin      : PWM I/O pin selection, please refer MMPF_PWMPIN
@param[in] bEnableIoPin : Enable/disable the specific PWM pin
@param[in] ulFreq       : The pulse frequency in unit of Hz.
@param[in] bHigh2Low    : MMP_TRUE: High to Low pulse, MMP_FALSE: Low to High pulse
@param[in] bEnableInterrupt : enable interrupt or not
@param[in] pwm_callBack : Call back function when interrupt occurs
@param[in] ulNumOfPulse : Number of pulse, 0 stand for using PWM auto mode to generate infinite pulse.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PWM_OutputPulse(MMPF_PWMPIN pwm_pin, MMP_BOOL bEnableIoPin, 
                             MMP_ULONG ulFreq, MMP_BOOL bHigh2Low, 
                             MMP_BOOL           bEnableInterrupt, 
                             PwmCallBackFunc*   pwm_callBack, 
                             MMP_ULONG          ulNumOfPulse)
{
	MMPF_PWM_ATTRIBUTE pwm_attribute = {0,                   // ubID
                                        MMPF_PWM_PULSE_ID_A, // uPulseID
                                        0x1,                 // ulClkDuty_T0
                                        0x2,                 // ulClkDuty_T1
                                        0x2,                 // ulClkDuty_T2
                                        0x2,                 // ulClkDuty_T3
                                        0x1,                 // ulClkDuty_Peroid
                                        0x1                  // ubNumOfPulses
                                        };
	MMP_ULONG ulTargetCount = 0;
	MMP_ULONG ulMaxFreq;

    MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ulMaxFreq);
    ulMaxFreq = ulMaxFreq * 250; //(gbl_clk*1000/2/2)

    if ((ulFreq == 0x0) && bEnableIoPin) {
        RTNA_DBG_Str(0, "Error: PWM invalid speed\r\n");
        return MMP_PWM_ERR_PARAMETER;
    }
	else if (ulFreq > ulMaxFreq) {
		RTNA_DBG_Str(0, "Error: PWM exceed max supported speed\r\n");
		return MMP_PWM_ERR_PARAMETER;
	}
    #if 0 //Sunny20140924: Remove it because it seems not important
	if ((ulMaxFreq % ulFreq) != 0x0) {
		RTNA_DBG_Str(0, "Warn: PWM clock will not be precise\r\n");
	}
    #endif
	if (ulNumOfPulse > 0xFF) {
		RTNA_DBG_Str(0, "Error: PWM pulse number overflow\r\n");
		return MMP_PWM_ERR_PARAMETER;
	}

	if (pwm_pin >= MMPF_PWMPIN_MAX) {
		RTNA_DBG_Str(0, "Error: Un-support PWM pin\r\n");
		return MMP_PWM_ERR_PARAMETER;
	}

    pwm_attribute.ubID = PWM_GET_ID(pwm_pin);

	if (bEnableIoPin == MMP_TRUE)
	{
		MMPF_PWM_EnableOutputPin(pwm_pin, MMP_TRUE);

		ulTargetCount = ulMaxFreq/ulFreq;

		if (bHigh2Low == MMP_FALSE) {  //low2High pulse
			pwm_attribute.ulClkDuty_T0 = 0x0;
			pwm_attribute.ulClkDuty_T1 = 0x0;
			pwm_attribute.ulClkDuty_T2 = 0x0;
			pwm_attribute.ulClkDuty_T3 = 0x1;
		}

		//EROY CHECK
		pwm_attribute.ulClkDuty_T0 = pwm_attribute.ulClkDuty_T0*ulTargetCount;
		pwm_attribute.ulClkDuty_T1 = pwm_attribute.ulClkDuty_T1*ulTargetCount;
		pwm_attribute.ulClkDuty_T2 = pwm_attribute.ulClkDuty_T2*ulTargetCount;
		pwm_attribute.ulClkDuty_T3 = pwm_attribute.ulClkDuty_T3*ulTargetCount;
		
		if (bHigh2Low == MMP_FALSE)
            pwm_attribute.ulClkDuty_Peroid = ulTargetCount*2 - 1;
		else
            pwm_attribute.ulClkDuty_Peroid = pwm_attribute.ulClkDuty_T3 - 1;
		
		if (ulNumOfPulse == 0)
			pwm_attribute.ubNumOfPulses = 1;
		else
			pwm_attribute.ubNumOfPulses = ulNumOfPulse;

		MMPF_PWM_SetAttribe(&pwm_attribute);
		
		if (bEnableInterrupt == MMP_TRUE)
			MMPF_PWM_EnableInterrupt(pwm_attribute.ubID, MMP_TRUE,
			            (PwmCallBackFunc*)pwm_callBack, MMPF_PWM_INT_OneRound);

		if (ulNumOfPulse != 0) {
			MMPF_PWM_ControlSet(pwm_attribute.ubID, (PWM_PULSE_A_FIRST|
                                                PWM_ONE_ROUND|PWM_PULSE_B_NEG|
                                                PWM_PULSE_A_POS|PWM_EN));
		}
		else { //For PWM auto mode
		    MMPF_PWM_ControlSet(pwm_attribute.ubID, (PWM_PULSE_A_FIRST|
                                                PWM_AUTO_CYC|PWM_PULSE_B_NEG|
                                                PWM_PULSE_A_POS|PWM_EN));
		}
	}
	else {
        MMPF_PWM_ControlSet(pwm_attribute.ubID, 0);
		MMPF_PWM_EnableOutputPin(pwm_pin, MMP_FALSE);
	}
	return MMP_ERR_NONE;
}

MMP_ERR MMPF_PWM_SetFreqDuty(MMPF_PWMPIN pwm_pin,MMP_ULONG pdFreqHz, MMP_UBYTE pdDuty)
{

	MMPF_PWM_ATTRIBUTE pwm_attribute = {0,                   // ubID
                                        MMPF_PWM_PULSE_ID_A, // uPulseID
                                        0x1,                 // ulClkDuty_T0
                                        0x2,                 // ulClkDuty_T1
                                        0x2,                 // ulClkDuty_T2
                                        0x2,                 // ulClkDuty_T3
                                        0x1,                 // ulClkDuty_Peroid
                                        0x1                  // ubNumOfPulses
                                        };
	MMP_ULONG ulMaxFreq;

    MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ulMaxFreq);
    ulMaxFreq = ulMaxFreq * 250; //(gbl_clk*1000/2/2)

	MMPF_PWM_EnableOutputPin(pwm_pin,MMP_TRUE);

    
    //T0 = (1000000/(gbFreqHz)) * gbDuty/100  * (PWM_BLOCK_CLOCK/1000000);
    //T1 = (1000000/(gbFreqHz)) * 100/100  * (PWM_BLOCK_CLOCK/1000000);
    //above is original formula, below is simplified
    //nPosTime = (((MMP_ULONG)gbDuty*10000)/(gbFreqHz));
    //dbg_printf(3," POS %dms\r\n",(((MMP_ULONG)gbDuty*10000)/(gbFreqHz)));
    pwm_attribute.ubID = PWM_GET_ID(pwm_pin);
    //dbg_printf(3," PWM:F_%d D_%d,ID:%d\r\n",(pdFreqHz),pdDuty, pwm_attribute.ubID);
    pwm_attribute.ulClkDuty_T0 = ((MMP_ULONG)(ulMaxFreq/100)*pdDuty)/(pdFreqHz);
    pwm_attribute.ulClkDuty_T1 = ((MMP_ULONG)(ulMaxFreq)/(pdFreqHz));
    pwm_attribute.ulClkDuty_T2 = pwm_attribute.ulClkDuty_T1;
    pwm_attribute.ulClkDuty_T3 = pwm_attribute.ulClkDuty_T1;
    pwm_attribute.ulClkDuty_Peroid = (ulMaxFreq/pdFreqHz);
    
    MMPF_PWM_SetAttribe(&pwm_attribute);	
    MMPF_PWM_ControlSet(pwm_attribute.ubID, (PWM_PULSE_A_FIRST|
                                        PWM_AUTO_CYC|PWM_PULSE_B_NEG|
                                        PWM_PULSE_A_POS|PWM_EN));
	
	return MMP_ERR_NONE;
}

#if 1 //PWM test code
void MMPF_PWM_CallBack(MMP_ULONG status)
{
	RTNA_DBG_Str(0, "\r\n\r\n");
	RTNA_DBG_PrintLong(0, status);
}

void MMPF_PWM_Test1(MMP_ULONG hz) 
{
	AITPS_PWMB 	pPWM = (AITPS_PWMB)AITC_BASE_PWM;

	MMPF_PWM_Initialize();
	
	#if (CHIP == P_V2)
	MMPF_PWM_OutputPulse(MMPF_PWM0_PIN_CGPIO8, MMP_TRUE, 10000, MMP_TRUE,
	                     MMP_TRUE, (PwmCallBackFunc*)MMPF_PWM_CallBack, 0x3);
	#endif
	#if (CHIP == MCR_V2)
	MMPF_PWM_OutputPulse(MMPF_PWM0_PIN_AGPIO1, MMP_TRUE, 10000, MMP_TRUE,
	                     MMP_TRUE, (PwmCallBackFunc*)0/*MMPF_PWM_CallBack*/, 0/*0x3*/);
	#endif

	RTNA_DBG_PrintLong(0, (MMP_ULONG)&pPWM->PWM0[0].PWM_INT_CPU_EN);
	RTNA_DBG_PrintLong(0, pPWM->PWM0[0].PWM_INT_CPU_EN);
	MMPF_OS_Sleep(500);
	
	#if (CHIP == P_V2)
	MMPF_PWM_OutputPulse(MMPF_PWM0_PIN_CGPIO8, MMP_FALSE, 0x0, MMP_TRUE, 
	                     MMP_TRUE, NULL, 0x0);
    #endif
	#if (CHIP == MCR_V2)
	MMPF_PWM_OutputPulse(MMPF_PWM0_PIN_AGPIO1, MMP_FALSE, 0x0, MMP_TRUE,
	                     MMP_TRUE, NULL, 0x0);
    #endif
}

void MMPF_PWM_Test(MMP_ULONG hz) 
{
	AITPS_PWMB 	pPWM = (AITPS_PWMB)AITC_BASE_PWM;
	int duty = 0 ;
	MMPF_PWM_Initialize();
	for(duty=0;duty<100;duty++) {
		MMPF_PWM_SetFreqDuty(MMPF_PWM0_PIN_AGPIO1,hz,duty);
		MMPF_PWM_SetFreqDuty(MMPF_PWM1_PIN_AGPIO2,hz,duty);
		MMPF_PWM_SetFreqDuty(MMPF_PWM2_PIN_AGPIO3,hz,duty);
		MMPF_OS_Sleep(50);
		//MMPF_PWM_Test1(hz) ;
	}
}
#endif