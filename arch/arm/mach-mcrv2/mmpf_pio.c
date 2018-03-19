//==============================================================================
//
//  File        : mmpf_pio.c
//  Description : PIO Pins Control Interface
//  Author      : Ben Lu
//  Revision    : 1.0
//
//==============================================================================
/**
 *  @file mmpf_pio.c
 *  @brief The PIO pins control functions
 *  @author Ben Lu
 *  @version 1.0
 */

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include "includes_fw.h"

#include "lib_retina.h"
#include "mmp_reg_gbl.h"

#include "mmpf_pio.h"

//==============================================================================
//
//                              GLOBAL VARIABLE
//
//==============================================================================

/** @addtogroup MMPF_PIO
@{
*/
static MMPF_OS_SEMID    gPIO_GpioModeSemID;
static MMPF_OS_SEMID    gPIO_OutputModeSemID;
static MMPF_OS_SEMID    gPIO_SetDataSemID;
static MMPF_OS_SEMID    gPIO_EnableTrigModeSemID;
static MMPF_OS_SEMID    gPIO_EnableInterruptSemID;

static MMP_ULONG        gPIO_BoundingTime[PIO_MAX_PIN_SIZE];
static PioCallBackFunc *gPIO_CallBackFunc[PIO_MAX_PIN_SIZE];

//==============================================================================
//
//                              EXTERN VARIABLE
//
//==============================================================================

#if (OS_TYPE == OS_UCOSII)
extern MMPF_OS_MQID     HighTask_QId;
#endif

extern MMP_UBYTE		m_gbKeypadCheck[];

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================
//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_Initialize
//  Description :
//------------------------------------------------------------------------------
/** @brief The function registers the interrupt and create related semaphore for PIO pins.

The function registers the interrupt and create related semaphore for PIO pins.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PIO_Initialize(void)
{
    MMP_USHORT  i = 0;
    #if (OS_TYPE == OS_UCOSII)
    AITPS_AIC   pAIC = AITC_BASE_AIC;
    #endif
    static MMP_BOOL bPioInitFlag = MMP_FALSE;

    if (bPioInitFlag == MMP_FALSE) {

        #if (OS_TYPE == OS_UCOSII)
        RTNA_AIC_Open(pAIC, AIC_SRC_GPIO, gpio_isr_a,
                        AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 3);
        RTNA_AIC_IRQ_En(pAIC, AIC_SRC_GPIO);
        #endif //(OS_TYPE == OS_UCOSII)

        gPIO_GpioModeSemID 			= MMPF_OS_CreateSem(1);
        gPIO_OutputModeSemID        = MMPF_OS_CreateSem(1);
        gPIO_SetDataSemID           = MMPF_OS_CreateSem(1);
        gPIO_EnableTrigModeSemID    = MMPF_OS_CreateSem(1);
        gPIO_EnableInterruptSemID   = MMPF_OS_CreateSem(1);

        MEMSET0(gPIO_BoundingTime);

        for (i = 0; i < PIO_MAX_PIN_SIZE ; i++) {
            gPIO_CallBackFunc[i] = NULL;
            MMPF_PIO_EnableInterrupt(i, MMP_FALSE, 0, 0, MMPF_OS_LOCK_CTX_ISR);
        }

        bPioInitFlag = MMP_TRUE;
    }
    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_Enable
//  Description :
//------------------------------------------------------------------------------
/** @brief The function enable or disable GPIO mode of multifunction IO.

The function configure the multifunction IO mode, set enable to GPIO mode.
@param[in] piopin is the PIO number, please reference the data structure of MMPF_PIO_REG
@param[in] bEnable is the choice of GPIO mode or not
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PIO_EnableGpioMode(MMPF_PIO_REG piopin, MMP_BOOL bEnable, MMPF_OS_LOCK_CTX LockCtx)
{
    MMP_UBYTE   ret;
    #if (CHIP == MERCURY) || (CHIP == VSN_V3)
    MMP_ULONG   ulBitPos = 1 << (piopin & 0x07); //bit of byte
    #endif
    #if (CHIP == MCR_V2)
    MMP_ULONG   ulBitPos = 1 << (piopin & PIO_BITPOSITION_INFO); //bit of byte
    MMP_UBYTE   ubIndex = PIO_GET_INDEX(piopin); //index of 4-byte
    #endif
    AITPS_GBL   pGBL = AITC_BASE_GBL;

    if (LockCtx == MMPF_OS_LOCK_CTX_TASK) {
        ret = MMPF_OS_AcquireSem(gPIO_GpioModeSemID, PIO_SEM_TIMEOUT);
    } else {
        ret = MMPF_OS_TrySem(gPIO_GpioModeSemID);
    }

    if (ret) {
        RTNA_DBG_Byte0(ret);
        RTNA_DBG_Str0(": PIO EnableGpioMode sem failed\r\n");
        return MMP_PIO_ERR_SEMAPHORE_FAIL;
    }

    #if (CHIP == MERCURY) || (CHIP == VSN_V3)
    if (bEnable == MMP_TRUE) {
        if (piopin < MMPF_PIO_REG_GPIO8) {
            pGBL->GBL_IO_CTL5 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO16) {
            pGBL->GBL_IO_CTL6 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO24) {
            pGBL->GBL_IO_CTL7 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO32) {
            pGBL->GBL_IO_CTL8 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO40) {
            pGBL->GBL_IO_CTL9 |= ulBitPos;
        }
        #if (CHIP == VSN_V3)||(CHIP == MERCURY)
        else if (piopin < MMPF_PIO_REG_GPIO48) {
            pGBL->GBL_IO_CTL10 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO56) {
            pGBL->GBL_IO_CTL11 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO64) {
            pGBL->GBL_IO_CTL12 |= ulBitPos;
        }
        #endif
        #if (CHIP == VSN_V3)
        else {
            pGBL->GBL_IO_CTL13 |= ulBitPos;
        }
        #endif
        #if (CHIP == MERCURY)
        else if (piopin < MMPF_PIO_REG_GPIO72) {
            pGBL->GBL_IO_CTL13 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO80) {
            pGBL->GBL_IO_CTL14 |= ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO88) {
            pGBL->GBL_IO_CTL16 |= ulBitPos;
        }
        else {
            pGBL->GBL_IO_CTL17 |= ulBitPos;
        }
        #endif
    }
    else {
        if (piopin < MMPF_PIO_REG_GPIO8) {
            pGBL->GBL_IO_CTL5 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO16) {
            pGBL->GBL_IO_CTL6 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO24) {
            pGBL->GBL_IO_CTL7 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO32) {
            pGBL->GBL_IO_CTL8 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO40) {
            pGBL->GBL_IO_CTL9 &= ~ulBitPos;
        }
        #if (CHIP == VSN_V3)||(CHIP == MERCURY)
        else if (piopin < MMPF_PIO_REG_GPIO48) {
            pGBL->GBL_IO_CTL10 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO56) {
            pGBL->GBL_IO_CTL11 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO64) {
            pGBL->GBL_IO_CTL12 &= ~ulBitPos;
        }
        #endif
        #if (CHIP == VSN_V3)
        else {
            //pGBL->GBL_IO_CTL10 &= ~ulBitPos;
        }
        #endif
        #if (CHIP == MERCURY)
        else if (piopin < MMPF_PIO_REG_GPIO72) {
            pGBL->GBL_IO_CTL13 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO80) {
            pGBL->GBL_IO_CTL14 &= ~ulBitPos;
        }
        else if (piopin < MMPF_PIO_REG_GPIO88) {
            pGBL->GBL_IO_CTL16 &= ~ulBitPos;
        }
        else {
            pGBL->GBL_IO_CTL17 &= ~ulBitPos;
        }
        #endif
    }
    #endif //(CHIP == MERCURY) || (CHIP == VSN_V3)

    #if (CHIP == MCR_V2)
    if (bEnable)
        pGBL->GBL_GPIO_CFG[ubIndex] |= ulBitPos;
    else
        pGBL->GBL_GPIO_CFG[ubIndex] &= ~ulBitPos;
    #endif

    ret = MMPF_OS_ReleaseSem(gPIO_GpioModeSemID);
    
    if (ret) {
        RTNA_DBG_Byte0(ret);
        RTNA_DBG_Str0(": PIO sem rel failed\r\n");
        return MMP_PIO_ERR_SEMAPHORE_FAIL;
    }

    return MMP_ERR_NONE;
}
EXPORT_SYMBOL(MMPF_PIO_EnableGpioMode);
EXPORT_SYMBOL(MMPF_PIO_EnableOutputMode);
//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_EnableOutputMode
//  Description :
//------------------------------------------------------------------------------
/** @brief The function set the PIN as Output mode (bEnable = MMP_TRUE) or Input mode.

The function set the PIN as Output mode (bEnable = MMP_TRUE) or Input mode.
@param[in] piopin is the PIO number, please reference the data structure of MMPF_PIO_REG
@param[in] bEnable is the choice of output mode or input mode
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PIO_EnableOutputMode(MMPF_PIO_REG piopin, MMP_BOOL bEnable, MMPF_OS_LOCK_CTX LockCtx)
{
    MMP_ULONG   ulBitPos = 1 << (piopin & PIO_BITPOSITION_INFO); //bit of byte
    MMP_UBYTE   ubIndex  = PIO_GET_INDEX(piopin); //index of 4-byte
    AITPS_GPIO  pGPIO = AITC_BASE_GPIO;
    MMP_UBYTE   SemErr = OS_NO_ERR;

    if (LockCtx == MMPF_OS_LOCK_CTX_TASK) {
        SemErr = MMPF_OS_AcquireSem(gPIO_OutputModeSemID, PIO_SEM_TIMEOUT);
    } else {
        SemErr = MMPF_OS_TrySem(gPIO_OutputModeSemID);
    }
    if (SemErr) {
        RTNA_DBG_Byte0(SemErr);
        RTNA_DBG_Str0(": PIO EnableOutputMode sem failed\r\n");
        return MMP_PIO_ERR_SEMAPHORE_FAIL;
    }

    if (bEnable) {
        #if (CHIP == MCR_V2)
        /* GPIO81~96, GPI only */
        if ((piopin >= MMPF_PIO_REG_GPIO81) && (piopin <= MMPF_PIO_REG_GPIO96)) {
            RTNA_DBG_Str(0, "Not support output mode\r\n");
            MMPF_OS_ReleaseSem(gPIO_OutputModeSemID);
            return MMP_ERR_NONE;
        }
        #endif
        pGPIO->GPIO_OUT_EN[ubIndex] |= ulBitPos;
    }
    else {
        pGPIO->GPIO_OUT_EN[ubIndex] &= ~ulBitPos;
    }

    MMPF_OS_ReleaseSem(gPIO_OutputModeSemID);

    return MMP_ERR_NONE;
}

MMP_ERR MMPF_PIO_GetOutputMode(MMPF_PIO_REG piopin, MMP_UBYTE* returnValue)
{
	MMP_UBYTE bitPostionInfo = piopin & PIO_BITPOSITION_INFO;
	MMP_UBYTE IndexAddressInfo = PIO_GET_INDEX(piopin);
	AITPS_GPIO pGPIO = AITC_BASE_GPIO;	

	*returnValue = (pGPIO->GPIO_OUT_EN[IndexAddressInfo] >> bitPostionInfo) & 0x01;

	return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_SetData
//  Description :
//------------------------------------------------------------------------------
/** @brief The function set the PIO pin as High or Low (When the pin is at output mode).

The function set the PIO pin as High or Low (When the pin is at output mode).
@param[in] piopin is the PIO number, please reference the data structure of MMPF_PIO_REG
@param[in] outputValue is 0 stands for Low, otherwise it stands for High
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PIO_SetData(MMPF_PIO_REG piopin, MMP_UBYTE outputValue, MMPF_OS_LOCK_CTX LockCtx)
{
    MMP_ULONG   ulBitPos = 1 << (piopin & PIO_BITPOSITION_INFO); //bit of byte
    MMP_UBYTE   ubIndex  = PIO_GET_INDEX(piopin); //index of 4-byte
    AITPS_GPIO  pGPIO = AITC_BASE_GPIO;
    MMP_UBYTE   SemErr = OS_NO_ERR;

    if((pGPIO->GPIO_OUT_EN[ubIndex] & ulBitPos) == 0x0){
        //"Error !!! PIO Input Mode to call  MMPF_PIO_SetData
        return MMP_PIO_ERR_INPUTMODESETDATA;
    }

    if (LockCtx == MMPF_OS_LOCK_CTX_TASK) {
        SemErr = MMPF_OS_AcquireSem(gPIO_SetDataSemID, PIO_SEM_TIMEOUT);
    } else {
        SemErr = MMPF_OS_TrySem(gPIO_SetDataSemID);
    }
    if (SemErr) {
        RTNA_DBG_Byte0(SemErr);
        RTNA_DBG_Str0(": PIO SetData sem failed\r\n");
        return MMP_PIO_ERR_SEMAPHORE_FAIL;
    }
	
    #if (CHIP == MCR_V2)
    /* GPIO81~96, GPI only */
    if ((piopin >= MMPF_PIO_REG_GPIO81) && (piopin <= MMPF_PIO_REG_GPIO96)) {
        RTNA_DBG_Str(0, "Not support output mode\r\n");
	    MMPF_OS_ReleaseSem(gPIO_SetDataSemID);
        return MMP_ERR_NONE;
    }
	#endif

    if(outputValue)
        pGPIO->GPIO_DATA[ubIndex] |= ulBitPos;
    else
        pGPIO->GPIO_DATA[ubIndex] &= ~ulBitPos;

    MMPF_OS_ReleaseSem(gPIO_SetDataSemID);

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_GetData
//  Description :
//------------------------------------------------------------------------------
/** @brief The function get the PIO pin's singal. (When the pin is at input mode).

The function get the PIO pin's singal. (When the pin is at input mode).
@param[in] piopin is the PIO number, please reference the data structure of MMPF_PIO_REG
@param[out] returnValue is standing for the High or Low signal.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PIO_GetData(MMPF_PIO_REG piopin, MMP_UBYTE* returnValue)
{
    MMP_ULONG   ulBitPos = 1 << (piopin & PIO_BITPOSITION_INFO); //bit of byte
    MMP_UBYTE   ubIndex  = PIO_GET_INDEX(piopin); //index of 4-byte
    AITPS_GPIO  pGPIO = AITC_BASE_GPIO;

    if (pGPIO->GPIO_OUT_EN[ubIndex] & ulBitPos){
        //Error !!! PIO Output Mode to call MMPF_PIO_GetData
        return MMP_PIO_ERR_OUTPUTMODEGETDATA;
    }

    *returnValue = (pGPIO->GPIO_DATA[ubIndex] & ulBitPos)? 1 : 0;
    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_EnableTrigMode
//  Description :
//------------------------------------------------------------------------------
/** @brief The function get the PIO pin's singal. (When the pin is at input mode).

The function get the PIO pin's singal. (When the pin is at input mode).
@param[in] piopin is the PIO number, please reference the data structure of MMPF_PIO_REG
@param[in] trigmode set the pio pin as edge trigger (H2L or L2H) or level trigger (H or L)
@param[out] bEnable make the tirgger settings work or not.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PIO_EnableTrigMode(MMPF_PIO_REG piopin, MMPF_PIO_TRIGMODE trigmode, MMP_BOOL bEnable,
                                MMPF_OS_LOCK_CTX LockCtx)
{
    MMP_ULONG   ulBitPos = 1 << (piopin & PIO_BITPOSITION_INFO); //bit of byte
    MMP_UBYTE   ubIndex  = PIO_GET_INDEX(piopin); //index of 4-byte
    AITPS_GPIO  pGPIO = AITC_BASE_GPIO;
    MMP_UBYTE   SemErr = OS_NO_ERR;

    if (LockCtx == MMPF_OS_LOCK_CTX_TASK) {
        SemErr = MMPF_OS_AcquireSem(gPIO_EnableTrigModeSemID, PIO_SEM_TIMEOUT);
    } else {
        SemErr = MMPF_OS_TrySem(gPIO_EnableTrigModeSemID);
    }
    if (SemErr) {
        RTNA_DBG_Byte0(SemErr);
        RTNA_DBG_Str0(": PIO EnableTrigMode sem failed\r\n");
        return MMP_PIO_ERR_SEMAPHORE_FAIL;
    }

    if(bEnable) {
        switch(trigmode){
        case MMPF_PIO_TRIGMODE_EDGE_H2L:
            pGPIO->GPIO_INT_H2L_EN[ubIndex] |= ulBitPos;
            break;

        case MMPF_PIO_TRIGMODE_EDGE_L2H:
            pGPIO->GPIO_INT_L2H_EN[ubIndex] |= ulBitPos;
            break;

        case MMPF_PIO_TRIGMODE_LEVEL_H:
            pGPIO->GPIO_INT_H_EN[ubIndex] |= ulBitPos;
            break;

        case MMPF_PIO_TRIGMODE_LEVEL_L:
            pGPIO->GPIO_INT_L_EN[ubIndex] |= ulBitPos;
            break;
        default:
            break;
        }
    }
    else {
        switch(trigmode){
        case MMPF_PIO_TRIGMODE_EDGE_H2L:
            pGPIO->GPIO_INT_H2L_EN[ubIndex] &= ~ulBitPos;
            break;

        case MMPF_PIO_TRIGMODE_EDGE_L2H:
            pGPIO->GPIO_INT_L2H_EN[ubIndex] &= ~ulBitPos;
            break;

        case MMPF_PIO_TRIGMODE_LEVEL_H:
            pGPIO->GPIO_INT_H_EN[ubIndex] &= ~ulBitPos;
            break;

        case MMPF_PIO_TRIGMODE_LEVEL_L:
            pGPIO->GPIO_INT_L_EN[ubIndex] &= ~ulBitPos;
            break;
        default:
            break;
        }
    }

    MMPF_OS_ReleaseSem(gPIO_EnableTrigModeSemID);

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_EnableInterrupt
//  Description :
//------------------------------------------------------------------------------
/** @brief The function set the PIO pin's interrupt actions.

The function set the PIO pin's interrupt actions.
@param[in] piopin is the PIO number, please reference the data structure of MMPF_PIO_REG
@param[in] bEnable stands for enable interrupt or not.
@param[in] boundingTime is used for advanced interrupt settings.
@param[in] CallBackFunc is used by interrupt handler.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_PIO_EnableInterrupt(MMPF_PIO_REG piopin, MMP_BOOL bEnable, MMP_ULONG boundingTime,
                                    PioCallBackFunc *CallBackFunc, MMPF_OS_LOCK_CTX LockCtx)
{
    MMP_ULONG   ulBitPos = 1 << (piopin & PIO_BITPOSITION_INFO); //bit of byte
    MMP_UBYTE   ubIndex  = PIO_GET_INDEX(piopin); //index of 4-byte
    AITPS_GPIO  pGPIO = AITC_BASE_GPIO;
    MMP_UBYTE   SemErr = OS_NO_ERR;

    if (LockCtx == MMPF_OS_LOCK_CTX_TASK) {
        SemErr = MMPF_OS_AcquireSem(gPIO_EnableInterruptSemID, PIO_SEM_TIMEOUT);
    } else {
        SemErr = MMPF_OS_TrySem(gPIO_EnableInterruptSemID);
    }
    if (SemErr) {
        RTNA_DBG_Byte0(SemErr);
        RTNA_DBG_Str0(": PIO EnableInterrupt sem failed\r\n");
        return MMP_PIO_ERR_SEMAPHORE_FAIL;
    }

    if(bEnable) {
        gPIO_BoundingTime[piopin] = boundingTime;
        gPIO_CallBackFunc[piopin] = CallBackFunc;

        pGPIO->GPIO_INT_CPU_EN[ubIndex] |= ulBitPos;
    }
    else {
        pGPIO->GPIO_INT_CPU_EN[ubIndex] &= ~ulBitPos;

        gPIO_BoundingTime[piopin] = 0;
        gPIO_CallBackFunc[piopin] = NULL;

    }

    MMPF_OS_ReleaseSem(gPIO_EnableInterruptSemID);

    return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_PIO_ISR
//  Description : 
//------------------------------------------------------------------------------
/** @brief PIO pin's interrupt handler function.

PIO pin's interrupt handler function.
@return It reports the status of the operation.
*/

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio_keys.h>
void MMPF_PIO_ISR(unsigned int irq,struct irq_desc *desc)
{
	MMP_ULONG i = 0, j = 0;
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    MMP_ULONG       intsrc_H = 0, intsrc_L = 0, intsrc_H2L = 0, intsrc_L2H = 0;
    #endif
    MMPF_PIO_REG    piopin = MMPF_PIO_REG_UNKNOWN;
    AITPS_GPIO      pGPIO = AITC_BASE_GPIO;
	struct irq_desc * gpio_irq_desc;
	struct irqaction *action;// = desc->action;
	struct gpio_button_data *bdata;// = (struct gpio_button_data *)(action->dev_id) ;

    // To find out the GPIO number and clean the interrupt
  //  #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    for (i = 0; i < 4; i++) {
        intsrc_H    = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_H_SR[i];
        intsrc_L    = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_L_SR[i];
        intsrc_H2L  = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_H2L_SR[i];
        intsrc_L2H  = pGPIO->GPIO_INT_CPU_EN[i] & pGPIO->GPIO_INT_L2H_SR[i];

		if(intsrc_H2L != 0x0){
			int k;
			//pGPIO->GPIO_INT_H2L_SR[i] = intsrc_H2L;  //clean interrupt

			k = ffs(intsrc_H2L);
			pGPIO->GPIO_INT_H2L_SR[i] = intsrc_H2L&(1<<(ffs(intsrc_H2L)-1));
			piopin = i*0x20 + ffs(intsrc_H2L)-1;//(j-1);
			//break;
		}

		
		if(intsrc_L2H != 0x0&&(piopin == MMPF_PIO_REG_UNKNOWN)){
			//pGPIO->GPIO_INT_L2H_SR[i] = intsrc_L2H;  //clean interrupt
		
			pGPIO->GPIO_INT_L2H_SR[i] = intsrc_L2H&(1<<(ffs(intsrc_L2H)-1));				
			
			piopin = i*0x20 + ffs(intsrc_L2H)-1;// (j-1);
			//break;
		}
		
		if(intsrc_H != 0x0 &&(piopin == MMPF_PIO_REG_UNKNOWN)){
			//pGPIO->GPIO_INT_H_SR[i] = intsrc_H;  //clean interrupt
		
			pGPIO->GPIO_INT_H_SR[i] = intsrc_H&(1<<(ffs(intsrc_H)-1));					
			
			piopin = i*0x20 + ffs(intsrc_H)-1;// (j-1);
			//break;
		}
		if(intsrc_L != 0x0&&(piopin == MMPF_PIO_REG_UNKNOWN)){
		
			pGPIO->GPIO_INT_L_SR[i] = intsrc_L&(1<<(ffs(intsrc_L)-1));					
			
			piopin = i*0x20 + ffs(intsrc_L)-1;// (j-1);
			//break;
		}
		



		if(piopin != MMPF_PIO_REG_UNKNOWN)
		{
			gpio_irq_desc = irq_to_desc(NR_AIC_IRQS+piopin);

//			action = gpio_irq_desc->action;
//			bdata = (struct gpio_button_data *)(action->dev_id) ;

			//bdata->button->value=(int)piopin;

			pr_debug("%s:piopin = %d\n",__func__,piopin);	

			handle_simple_irq(NR_AIC_IRQS+piopin, gpio_irq_desc);

			piopin = MMPF_PIO_REG_UNKNOWN;
		}
	}
#if 0	
    if (piopin != MMPF_PIO_REG_UNKNOWN) {
        if (gPIO_CallBackFunc[piopin] != NULL)
            (*gPIO_CallBackFunc[piopin])(piopin);
    }
#endif
	return;
}
