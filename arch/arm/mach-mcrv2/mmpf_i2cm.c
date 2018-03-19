//==============================================================================
//
//  File        : mmpf_i2cm.c
//  Description : MMPF_I2C functions
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================
//#define DEBUG

#include "includes_fw.h"
#include "lib_retina.h"
#include "mmp_reg_i2cm.h"
#include "mmp_reg_vif.h"
#include "mmp_reg_pad.h"
#include "mmpf_pio.h"
#include "mmpf_i2cm.h"
#include "mmpf_vif.h"
#include "mmpf_system.h"

/** @addtogroup MMPF_I2CM
@{
*/

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define I2C_SEN             VIF_SIF_SEN
#define I2C_SCL             VIF_SIF_SCL
#define I2C_SDA             VIF_SIF_SDA
#define I2C_RST             VIF_SIF_RST

#define I2C_INPUT           MMP_FALSE
#define I2C_OUTPUT          MMP_TRUE

#define I2C_SET_OUT         MMP_TRUE
#define I2C_CLEAR_OUT       MMP_FALSE

//==============================================================================
//
//                              GLOBAL VARIABLES
//
//==============================================================================

static MMP_UBYTE   	m_bRegLen[MMPF_I2CM_MAX_COUNT] = {0};       //If in the future, each module keep its' I2CM attribute, this array can be replaced.
static MMP_UBYTE 	m_bDataLen[MMPF_I2CM_MAX_COUNT] = {0};	    //If in the future, each module keep its' I2CM attribute, this array can be replaced.
MMP_UBYTE			m_bSwI2cmSlaveAddr[I2CM_SW_MAX_COUNT] = {0};//If in the future, each module keep its' I2CM attribute, this array can be replaced.
MMP_UBYTE			m_bSwI2cmDelayTime[I2CM_SW_MAX_COUNT] = {0};//If in the future, each module keep its' I2CM attribute, this array can be replaced.
MMPF_OS_SEMID  		gI2cmSemID[MMPF_I2CM_MAX_COUNT];			//used for protection with MMPF_I2CM_ID
static MMP_ULONG    m_ulI2cRxReadTimeout[I2CM_HW_MAX_COUNT];

#if (I2CM_INT_MODE_EN == 0x1)
MMPF_OS_SEMID  		gI2cmIntSemID[I2CM_HW_MAX_COUNT];		    //used for HW I2C interrupt mode, protection with HW MMPF_I2CM_ID
#endif
  
//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

extern MMP_ERR MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP ubGroupNum, MMP_ULONG *ulGroupFreq);

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================


#if (CHIP == MCR_V2)
//------------------------------------------------------------------------------
//  Function    : MMPF_I2CM_ISR
//  Description : This function is the I2cm interrupt service routine.
//------------------------------------------------------------------------------
/** 
 * @brief This function is the I2cm interrupt service routine.
 * 
 * @return It return void.  
 */
void MMPF_I2CM_ISR(void)
{
    #if (I2CM_INT_MODE_EN == 0x1)
   	MMP_ULONG   i = 0;
   	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
   	#endif

   	#if (I2CM_INT_MODE_EN == 0x1)
   	for(i = 0; i < I2CM_HW_MAX_COUNT; i++) 
   	{
   		if(pI2CM->I2CMS[i].I2CM_INT_CPU_SR & I2CM_TX_DONE) {
   			pI2CM->I2CMS[i].I2CM_INT_CPU_SR &= I2CM_TX_DONE;  //Clean status
   			
			if (pI2CM->I2CMS[i].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
				RTNA_DBG_Str(0, "I2C Error SLAVE_NO_ACK\r\n");
		        //Clear FIFO. Otherwise, keeping for next transmission.
		        pI2CM->I2CMS[i].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
		    }
		   	MMPF_OS_ReleaseSem(gI2cmIntSemID[i]); 
		   	break;
		}
	}
 	#endif
}
#endif // (CHIP == MCR_V2)

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_InitializeDriver
//  Description : The function initialize i2cm interrupt and semaphore.
//------------------------------------------------------------------------------
/** 
 * @brief The function initialize i2cm interrupt and semaphore.
 *    
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_InitializeDriver(void)
{
	MMP_UBYTE i = 0;
	static MMP_BOOL m_bInitialFlag = MMP_FALSE;
	MMP_ERR status = MMP_ERR_NONE;
    #if (I2CM_INT_MODE_EN == 0x1)&&(CHIP == MCR_V2)
    AITPS_AIC pAIC = AITC_BASE_AIC;
    #endif

	if (m_bInitialFlag == MMP_FALSE) 
	{
		#if (I2CM_INT_MODE_EN == 0x1)
        #if (CHIP == MCR_V2)
		#if (OS_TYPE == OS_UCOSII)
        RTNA_AIC_Open(pAIC, AIC_SRC_I2CM, i2cm_isr_a,
                    AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE | 7);
		RTNA_AIC_IRQ_En(pAIC, AIC_SRC_I2CM);
		#endif
        #endif
		#endif // (I2CM_INT_MODE_EN == 0x1)

		for(i = 0; i < I2CM_SW_MAX_COUNT; i++) {
			m_bSwI2cmSlaveAddr[i] = 0;
			m_bSwI2cmDelayTime[i] = 0;
		}

		for(i = 0; i < I2CM_HW_MAX_COUNT; i++) {
			#if (I2CM_INT_MODE_EN == 0x1)
			gI2cmIntSemID[i] = MMPF_OS_CreateSem(0);
			#endif
			m_ulI2cRxReadTimeout[i] = 0;
		}
		
		for(i = 0; i < MMPF_I2CM_MAX_COUNT; i++) {
			gI2cmSemID[i]   = MMPF_OS_CreateSem(1);
			m_bRegLen[i]    = 0;
			m_bDataLen[i]   = 0;
		}
		
		MMPF_SYS_EnableClock(MMPF_SYS_CLK_I2CM, MMP_TRUE);

		m_bInitialFlag = MMP_TRUE;
	}
	return status;
}
#if 0
//------------------------------------------------------------------------------
//  Function    : MMPF_I2c_Start
//  Description : The function is the start signal of i2cm.
//------------------------------------------------------------------------------
/** 
 * @brief The function is the start signal of i2cm.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.     
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2c_Start(MMPF_I2CM_ATTRIBUTE *pI2cmAttr)
{
	MMP_UBYTE   ubDelay;
	MMP_ERR     status = MMP_ERR_NONE;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif
	
	if((pI2cmAttr->ulI2cmSpeed == 0) || (pI2cmAttr->ulI2cmSpeed > I2CM_SW_MAX_SPEED))  {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM speed\r\n");
		return MMP_I2CM_ERR_PARAMETER;
	}
	
	ubDelay = I2CM_SW_MAX_SPEED/pI2cmAttr->ulI2cmSpeed;
	
	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) {
#ifdef USE_VIF_I2C
	    status |= MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_OUTPUT);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, (I2C_SCL|I2C_SDA), I2C_SET_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SDA, I2C_CLEAR_OUT);
	    RTNA_WAIT_US(ubDelay);
#endif		
    }
    else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW){
    
    	status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_clk_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);
		status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
		RTNA_WAIT_US(ubDelay);
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
		RTNA_WAIT_US(ubDelay);
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
    	RTNA_WAIT_US(ubDelay);
    }
    else {
    	RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM ID\r\n");
    	return MMP_I2CM_ERR_PARAMETER;
    }

	return	status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2c_Stop
//  Description : The function is the stop signal of i2cm.
//------------------------------------------------------------------------------
/** 
 * @brief The function is the stop signal of i2cm.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.     
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2c_Stop(MMPF_I2CM_ATTRIBUTE *pI2cmAttr)
{
	MMP_UBYTE 	ubDelay;
	MMP_ERR		status = MMP_ERR_NONE;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif
	
	if((pI2cmAttr->ulI2cmSpeed == 0) || (pI2cmAttr->ulI2cmSpeed > I2CM_SW_MAX_SPEED))  {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM speed\r\n");
		return MMP_I2CM_ERR_PARAMETER;
	}
	
	ubDelay = I2CM_SW_MAX_SPEED/pI2cmAttr->ulI2cmSpeed;
	
 	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) {
#ifdef USE_VIF_I2C 	
	    status |= MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_OUTPUT);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SDA, I2C_CLEAR_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_SET_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SDA, I2C_SET_OUT);
	    RTNA_WAIT_US(ubDelay);
#endif		
	}
	else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW){
	
    	status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
		RTNA_WAIT_US(ubDelay);
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
    }
    else {
    	RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM ID\r\n");
    	return MMP_I2CM_ERR_PARAMETER;
    }
	
	return	status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2c_WriteData
//  Description : The function write one byte data to slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function write one byte data to slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.     
 * @param[in] ubData    : stands for the data which will write into device. 
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2c_WriteData(MMPF_I2CM_ATTRIBUTE *pI2cmAttr, MMP_UBYTE ubData)
{
    MMP_ULONG   i;
    MMP_UBYTE	ubShift;
	MMP_UBYTE 	ubDelay;
	MMP_ERR		status = MMP_ERR_NONE;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif
	
	if((pI2cmAttr->ulI2cmSpeed == 0) || (pI2cmAttr->ulI2cmSpeed > I2CM_SW_MAX_SPEED))  {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM speed\r\n");
		return MMP_I2CM_ERR_PARAMETER;
	}
	
	ubDelay = I2CM_SW_MAX_SPEED/pI2cmAttr->ulI2cmSpeed;
	
	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) 
	{
#ifdef USE_VIF_I2C 		
	    status |= MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_OUTPUT);

	    ubShift = 0x80;
	    
	    for(i = 0; i < 8; i++) {
	        if(ubData & ubShift) {
	            status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SDA, I2C_SET_OUT);
	            status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	        }
	        else {
	            status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SDA | I2C_SCL, I2C_CLEAR_OUT);
	        }

	        RTNA_WAIT_US(ubDelay);
	        status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_SET_OUT);
	        RTNA_WAIT_US(ubDelay);
	        status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	        RTNA_WAIT_US(ubDelay);
	        ubShift >>= 1;
	    }
#endif		
	}
	else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)
	{
    	status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);

		ubShift = 0x80;

    	for(i = 0; i < 8; i++) {
	        if(ubData & ubShift) {
	         	status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	        }
	        else {
	           	status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
	        }

			RTNA_WAIT_US(ubDelay);
	        status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	        RTNA_WAIT_US(ubDelay);
            status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
            
            if (i == 7)
            {
                status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_FALSE, pI2cmAttr->bOsProtectEn);
            }
	        RTNA_WAIT_US(ubDelay);
	        ubShift >>= 1;
    	}
    }
    else {
    	RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM ID\r\n");
    	return MMP_I2CM_ERR_PARAMETER;
    }

	return	status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2c_ReadData
//  Description : The function read one byte data from slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function read one byte data from slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.      
 * @return It return the data which read from device. . 
 */
MMP_UBYTE MMPF_I2c_ReadData(MMPF_I2CM_ATTRIBUTE *pI2cmAttr)
{
    MMP_ULONG   i;
    MMP_UBYTE  	ubReceiveData = 0;
    MMP_UBYTE  	ubBit_val =0 ;
	MMP_UBYTE 	ubDelay;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif
	
	if((pI2cmAttr->ulI2cmSpeed == 0) || (pI2cmAttr->ulI2cmSpeed > I2CM_SW_MAX_SPEED))  {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM speed\r\n");
		return 0;
	}
	
	ubDelay = I2CM_SW_MAX_SPEED/pI2cmAttr->ulI2cmSpeed;

	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) {
#ifdef USE_VIF_I2C 		
	    MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	    MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_INPUT);

	    ubReceiveData = 0;
	    
	    for(i = 0; i < 8; i++) {
	        RTNA_WAIT_US(ubDelay);
	        MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_SET_OUT);
		    
		    ubBit_val = (MMP_UBYTE)MMPF_VIF_GetPIOOutput(ubVifId, I2C_SDA);
	        ubReceiveData |= ubBit_val;
	        
	        if(i < 7){
	            ubReceiveData <<= 1;
	        }
	        RTNA_WAIT_US(ubDelay);
		    MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	    }
	    return (ubReceiveData);
#endif		
	}
	else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW){
    	
    	MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_FALSE, pI2cmAttr->bOsProtectEn);
	 
    	ubReceiveData = 0;
    	
    	for(i = 0; i < 8; i++) {
			MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	        MMPF_PIO_GetData(pI2cmAttr->sw_data_pin, &ubBit_val);
	        RTNA_WAIT_US(ubDelay);

	        ubReceiveData |= ubBit_val;
	        
	        if(i < 7){
	            ubReceiveData <<= 1;
	        }
	        
	        MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
			
			if (i == 7)
			{
				MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);
				MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
			}
	        RTNA_WAIT_US(ubDelay);
	     }
    }
    else {
    	RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM ID\r\n");
    	return 0;
    }
	
    return (ubReceiveData);
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2c_GetACK
//  Description : The function get ACK signal from slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function get ACK signal from slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.      
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2c_GetACK(MMPF_I2CM_ATTRIBUTE *pI2cmAttr)
{
    MMP_UBYTE  	ubBit_val;
    MMP_ERR 	status  = MMP_ERR_NONE;
    MMP_ULONG 	ulcount = 0x4FFF;
	MMP_UBYTE 	ubDelay;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif
	
	if((pI2cmAttr->ulI2cmSpeed == 0) || (pI2cmAttr->ulI2cmSpeed > I2CM_SW_MAX_SPEED))  {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM speed\r\n");
		return MMP_I2CM_ERR_PARAMETER;
	}
	
	ubDelay = I2CM_SW_MAX_SPEED/pI2cmAttr->ulI2cmSpeed;

	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) {
#ifdef USE_VIF_I2C 		
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	    status |= MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_INPUT);
	    
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_SET_OUT);
	    RTNA_WAIT_US(ubDelay);

        #if (CHIP == P_V2)
		if(gbSystemCoreID == CHIP_CORE_ID_PV2P) { //EROY CHECK
			RTNA_WAIT_US(100);
		}
		else
        #endif
		{
		    do{
		        ubBit_val = (MMP_UBYTE)MMPF_VIF_GetPIOOutput(ubVifId, I2C_SDA);
	            ulcount --;
	            if(ulcount == 0){
	                status = MMP_I2CM_ERR_SLAVE_NO_ACK;
	                RTNA_DBG_Str(0, "MMP_I2CM_ERR_SLAVE_NO_ACK\r\n");
	                break;
	            }
		    }while(ubBit_val);
	    }

	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_OUTPUT);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	    RTNA_WAIT_US(ubDelay);
#endif		
	}
	else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW){
		
	    status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_FALSE, pI2cmAttr->bOsProtectEn);
	    
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    
	    do{
	        status |= MMPF_PIO_GetData(pI2cmAttr->sw_data_pin, &ubBit_val);
            ulcount --;
            if(ulcount == 0){
                status = MMP_I2CM_ERR_SLAVE_NO_ACK;
                RTNA_DBG_Str(0, "Error : MMP_I2CM_ERR_SLAVE_NO_ACK\r\n");
                break;
            }
	    }while(ubBit_val);
	   
	   	//For Customer's wave-form improvement request
	    MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);    
	}
    else {
    	RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM ID\r\n");
    	return MMP_I2CM_ERR_PARAMETER;
    }

	return	status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2c_SendNACK
//  Description : The function send NACK signal to slave device for read operation.
//------------------------------------------------------------------------------
/** 
 * @brief The function send NACK signal to slave device for read operation.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.      
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2c_SendNACK(MMPF_I2CM_ATTRIBUTE *pI2cmAttr)
{
	MMP_UBYTE 	ubDelay;
	MMP_ERR		status = MMP_ERR_NONE;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif
	
	if((pI2cmAttr->ulI2cmSpeed == 0) || (pI2cmAttr->ulI2cmSpeed > I2CM_SW_MAX_SPEED))  {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM speed\r\n");
		return MMP_I2CM_ERR_PARAMETER;
	}
	
	ubDelay = I2CM_SW_MAX_SPEED/pI2cmAttr->ulI2cmSpeed;
	
	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) {
#ifdef USE_VIF_I2C 	
	    status |= MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_OUTPUT);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SDA, I2C_SET_OUT);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_SET_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SDA, I2C_CLEAR_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_SET_OUT);
	    RTNA_WAIT_US(ubDelay);
#endif		
	}
	else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW){
	 	status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);
	 	
	 	status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	}
    else {
    	RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM ID\r\n");
    	return MMP_I2CM_ERR_PARAMETER;
    }
	return	status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2c_SendACK
//  Description : The function send ACK signal to slave device for read operation.
//------------------------------------------------------------------------------
/** 
 * @brief The function send ACK signal to slave device for read operation.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.      
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2c_SendACK(MMPF_I2CM_ATTRIBUTE *pI2cmAttr)
{
	MMP_UBYTE 	ubDelay;
	MMP_ERR		status = MMP_ERR_NONE;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif
	
	if((pI2cmAttr->ulI2cmSpeed == 0) || (pI2cmAttr->ulI2cmSpeed > I2CM_SW_MAX_SPEED))  {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM speed\r\n");
		return MMP_I2CM_ERR_PARAMETER;
	}
	
	ubDelay = I2CM_SW_MAX_SPEED/pI2cmAttr->ulI2cmSpeed;
	
	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) {
#ifdef USE_VIF_I2C 	
	    status |= MMPF_VIF_SetPIODir(ubVifId, I2C_SDA, I2C_OUTPUT);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, (I2C_SCL | I2C_SDA), I2C_CLEAR_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_SET_OUT);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_VIF_SetPIOOutput(ubVifId, I2C_SCL, I2C_CLEAR_OUT);
#endif		
	 }
    else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW){
		status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, pI2cmAttr->bOsProtectEn);
		
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	    status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_LOW, pI2cmAttr->bOsProtectEn);
	    RTNA_WAIT_US(ubDelay);
	}
    else {
    	RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error soft I2CM ID\r\n");
    	return MMP_I2CM_ERR_PARAMETER;
    }

	return	status;
}
#endif
#if 0
void __I2CM_GENERAL_INTERFACE__(){}
#endif

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_StartSemProtect
//  Description : The function acquire semaphore to do i2cm operation.
//------------------------------------------------------------------------------
/** 
 * @brief The function acquire semaphore to do i2cm operation.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.      
 * @return It return the function status. 
 */
MMP_UBYTE MMPF_I2cm_StartSemProtect(MMPF_I2CM_ATTRIBUTE* pI2cmAttr) 
{
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		return MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	else {
		return 0;
	}
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_EndSemProtect
//  Description : The function release semaphore to end i2cm operation.
//------------------------------------------------------------------------------
/** 
 * @brief The function release semaphore to end i2cm operation.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.      
 * @return It return the function status. 
 */
MMP_UBYTE MMPF_I2cm_EndSemProtect(MMPF_I2CM_ATTRIBUTE* pI2cmAttr) 
{
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		return MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);
	}
	else {
		return 0;
	} 
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_WaitAllFree
//  Description : The function check all i2cm controllers are free or not.
//------------------------------------------------------------------------------
/** 
 * @brief The function check all i2cm controllers are free or not.
 *     
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_WaitAllFree(void)
{
	int i = 0;
	
	MMPF_I2cm_InitializeDriver();
	
	for(i= 0; i < MMPF_I2CM_MAX_COUNT; i++) {
		MMPF_OS_AcquireSem(gI2cmSemID[i], 0);
	}
	
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_SetAllFree
//  Description : The function release all i2cm controllers semaphore.
//------------------------------------------------------------------------------
/** 
 * @brief The function release all i2cm controllers semaphore.
 *     
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_SetAllFree(void)
{
	int i = 0;
	
	MMPF_I2cm_InitializeDriver();
	
	for(i= 0; i < MMPF_I2CM_MAX_COUNT; i++) {
		MMPF_OS_ReleaseSem(gI2cmSemID[i]);
	}
	
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_SetRxTimeout
//  Description : The function set Rx operation timeout count.
//------------------------------------------------------------------------------
/** 
 * @brief The function set Rx operation timeout count.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.
 * @param[in] ulTimeOut : stands for the timeout count, 
 *                        ulTimeOut = 0 means no Timeout, otherwise the unit is ms.       
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_SetRxTimeout(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_ULONG ulTimeOut)
{
	m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] = ulTimeOut;
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_Initialize
//  Description : The function intiailize HW/SW i2cm device.
//------------------------------------------------------------------------------
/** 
 * @brief The function intiailize HW/SW i2cm device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_Initialize(MMPF_I2CM_ATTRIBUTE *pI2cmAttr)
{
	AITPS_GBL   pGBL  = AITC_BASE_GBL;
    #if (CHIP == MCR_V2)
    AITPS_PAD   pPAD  = AITC_BASE_PAD;
    #endif
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_USHORT  usSckDiv = 0;
	MMP_ERR		status = MMP_ERR_NONE;
	MMP_ULONG   ulGrp0Freq = 0;
    #if (OS_TYPE == OS_LINUX)
    MMP_UBYTE   ubVifId = 0;
    #endif
    #if (OS_TYPE == OS_UCOSII)
	#if defined(ALL_FW)
	MMP_UBYTE	ubVifId = gsSensorFunction->MMPF_Sensor_GetCurVifPad();
	#else
	MMP_UBYTE	ubVifId = 0;
	#endif
    #endif

	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR) 
	{
#ifdef USE_VIF_I2C 		
		#if (CHIP == MERCURY)
		TODO
		#endif
        #if (CHIP == MCR_V2)
		pPAD->PAD_IO_CFG_PSNR[0x07] = PAD_OUT_DRIVING(1) | PAD_PULL_UP;
        	pGBL->GBL_MISC_IO_CFG |= GBL_VIF0_GPIO_EN;
        #endif
	
		m_bSwI2cmSlaveAddr[1] = (pI2cmAttr->ubSlaveAddr) << 1;
		
		status |= MMPF_VIF_SetPIODir(ubVifId, VIF_SIF_SDA, MMP_TRUE);
		status |= MMPF_VIF_SetPIOOutput(ubVifId, VIF_SIF_SDA, MMP_TRUE);
		status |= MMPF_VIF_SetPIODir(ubVifId, VIF_SIF_SCL, MMP_TRUE);
		status |= MMPF_VIF_SetPIOOutput(ubVifId, VIF_SIF_SCL, MMP_TRUE);
		m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW] = pI2cmAttr->ubDelayTime;
#endif		
	}
	else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)
	{
		if((pI2cmAttr->sw_clk_pin == 0) || (pI2cmAttr->sw_data_pin == 0)) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "SW I2C pin assign error !!\r\n");
			return MMP_I2CM_ERR_PARAMETER;
		}
		
        m_bSwI2cmSlaveAddr[0] = pI2cmAttr->ubSlaveAddr;
        
        //Pull High first !! before set output mode
        status |= MMPF_PIO_SetData(pI2cmAttr->sw_clk_pin, PIO_HIGH, MMPF_OS_LOCK_CTX_TASK);  
		status |= MMPF_PIO_SetData(pI2cmAttr->sw_data_pin, PIO_HIGH, MMPF_OS_LOCK_CTX_TASK);
        
		status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_clk_pin, MMP_TRUE, MMPF_OS_LOCK_CTX_TASK);
		status |= MMPF_PIO_EnableOutputMode(pI2cmAttr->sw_data_pin, MMP_TRUE, MMPF_OS_LOCK_CTX_TASK);
	
		m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW] = pI2cmAttr->ubDelayTime;	
    }
	else if (pI2cmAttr->i2cmID < MMPF_I2CM_ID_SW) 
	{	
		// Set I2cm Pad configuration
        #if (CHIP == VSN_V2) || (CHIP == VSN_V3)
		if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_0) {    //HW I2C
            pGBL->GBL_IO_CTL1 |= (GBL_I2C_0_PAD_EN);
			if(pI2cmAttr->ubPadNum == 0x0) {
				pGBL->GBL_IO_CTL1 &= (~GBL_I2C_0_PAD_SEL_SNR);
			}
			else {
				pGBL->GBL_IO_CTL1 |= GBL_I2C_0_PAD_SEL_SNR;
			}
		}
		else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_1) {    //HW I2C
			pGBL->GBL_IO_CTL1 |= GBL_I2C_1_PAD_EN;
			if (pI2cmAttr->ubPadNum == 0x0) {
				pGBL->GBL_IO_CTL0 &= ~(GBL_I2C_1_PIN_SEL);
			}
			else {
				pGBL->GBL_IO_CTL0 |= GBL_I2C_1_PIN_SEL;
			}
		}
        #endif
		#if (CHIP == MERCURY)
		TODO
		#endif
    	#if (CHIP == MCR_V2)
    	if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_0) {
            pGBL->GBL_I2CM_PAD_CFG &= ~(GBL_I2CM0_PAD_MASK);
            pGBL->GBL_I2CM_PAD_CFG |= GBL_I2CM0_PAD(pI2cmAttr->ubPadNum);
    	}
    	else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_1) {
    	    pGBL->GBL_I2CM_PAD_CFG &= ~(GBL_I2CM1_PAD_MASK);
            pGBL->GBL_I2CM_PAD_CFG |= GBL_I2CM1_PAD(pI2cmAttr->ubPadNum);
    	}
        else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_2) {
    	    pGBL->GBL_I2CM_PAD_CFG &= ~(GBL_I2CM2_PAD_MASK);
            pGBL->GBL_I2CM_PAD_CFG |= GBL_I2CM2_PAD(pI2cmAttr->ubPadNum);
    	}
        else if (pI2cmAttr->i2cmID == MMPF_I2CM_ID_3) {
    	    pGBL->GBL_I2CM_PAD_CFG &= ~(GBL_I2CM3_PAD_MASK);
            pGBL->GBL_I2CM_PAD_CFG |= GBL_I2CM3_PAD(pI2cmAttr->ubPadNum);
    	}
    	#endif

        if (pI2cmAttr->bWfclModeEn == MMP_TRUE){
    	    pI2cmAttr->ubRegLen = pI2cmAttr->ubDataLen;
    	}

    	// Note the SLAVE_ADDR is different among SW and HW I2CM
		if (pI2cmAttr->ubRegLen == 16) {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR = pI2cmAttr->ubSlaveAddr;
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL =   I2CM_REG_16_MODE
	    	    											| I2CM_SCK_OH_MODE	// output high by internal signal
		    	    										| I2CM_SDA_OD_MODE  // by pull-up current, or resistor
		    	    										| I2CM_STOP_IF_NOACK;
		} else {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR = pI2cmAttr->ubSlaveAddr;
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL =   I2CM_REG_8_MODE
															| I2CM_SCK_OH_MODE // output high by internal signal
	       													| I2CM_SDA_OD_MODE // by pull-up current, or resistor
															| I2CM_STOP_IF_NOACK;
		}
		
		// Set HW Feature
		if (pI2cmAttr->bInputFilterEn == MMP_FALSE) {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT |= I2CM_INPUT_FILTERN_DIS;
		}
		else {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT &= (~I2CM_INPUT_FILTERN_DIS);
		}
		
		if (pI2cmAttr->bDelayWaitEn == MMP_TRUE) {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT |= I2CM_DELAY_WAIT_EN;
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_WAIT_DELAY_CYC = pI2cmAttr->ubDelayCycle;
		}
		else {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT &= (~I2CM_DELAY_WAIT_EN);
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_WAIT_DELAY_CYC = 0;
		}
		
		if (pI2cmAttr->b10BitModeEn == MMP_TRUE) {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT |= I2CM_10BITMODE_EN;
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR1 = pI2cmAttr->ubSlaveAddr1;
		}
		else {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT &= (~I2CM_10BITMODE_EN);
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR1 = 0;
		}
		
		if (pI2cmAttr->bClkStretchEn == MMP_TRUE) {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CLK_STRETCH_EN |= I2CM_STRETCH_ENABLE;
		}
		else {
			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CLK_STRETCH_EN &= (~I2CM_STRETCH_ENABLE);
		}
        
        // Set Clock Division/Timing
		if((pI2cmAttr->ulI2cmSpeed) != 0) 
		{
		    MMPF_PLL_GetGroupFreq(MMPF_CLK_GRP_GBL, &ulGrp0Freq);
		    
			usSckDiv = (ulGrp0Freq >> 1)*1000/pI2cmAttr->ulI2cmSpeed; //I2C module's clock = (Group 0)/2
			
			if(usSckDiv == 0) {
				RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Attention !  ulGrp0Freq is not been initialized\r\n");
				usSckDiv = 48000000/pI2cmAttr->ulI2cmSpeed;  //Group 0 boot as 96MHz in P_V2
			}
		}
		else 
		{
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error HW I2CM Speed Settings ! \r\n");
			return MMP_I2CM_ERR_PARAMETER;
		}
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SCK_DIVIDER       =  usSckDiv;        //(I2CModule clock 96/2MHz -> 400KHz)
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DATA_HOLD_CNT     = (usSckDiv >> 3);  //MUST not > (usSckDiv/2). Sugguest as (usSckDiv/4)~(usSckDiv/8)
		#if 1
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT   |= (((usSckDiv>>3) - 1) & I2CM_WAIT_CNT_MASK); // Wait N+1 i2cm_clk for next I2C cmd set
		#else
	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CMDSET_WAIT_CNT   = 20;               //Wait N+1 i2cm_clk for next I2C cmd set
		#endif
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SCL_DUTY_CNT      = (usSckDiv >> 1);
    }
	else {
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Error I2CM ID ! \r\n");
		return MMP_I2CM_ERR_PARAMETER;
	}
	
    m_bRegLen[pI2cmAttr->i2cmID]   = pI2cmAttr->ubRegLen;
	m_bDataLen[pI2cmAttr->i2cmID]  = pI2cmAttr->ubDataLen;

	return status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_WriteReg
//  Description : The function write one set data to slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function write one set data to slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address. If pI2cmAttr->bWfclModeEn is enalbe. usReg is no use.
 * @param[in] usData    : stands for the data to be sent.
 * @return It return the function status. 
 */
MMP_ERR	MMPF_I2cm_WriteReg(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT usData)
{
    return MMPF_I2cm_WriteRegSet(pI2cmAttr, &usReg, &usData, 1);
}
#if 0
//------------------------------------------------------------------------------
//  Function    : MMPF_I2Cm_ReadEEDID
//  Description : The function read EDD data from slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function read EDD data from slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[out] ubData   : stands for 
 * @param[in] usSeg     : stands for 
 * @param[in] usOffset  : stands for 
 * @param[in] usSize    : stands for 
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2Cm_ReadEEDID(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_UBYTE *ubData, MMP_USHORT usSeg, MMP_USHORT usOffset, MMP_USHORT usSize)
{
    MMP_UBYTE	ubSemStatus = 0xFF;
    MMP_ERR		status = MMP_ERR_NONE;
	MMP_ULONG   i;
	#if (CHIP == MCR_V2)
	MMP_ULONG   ulI2cmTimeOut = 0;
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	#endif
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	
    status |= MMPF_I2cm_Initialize(pI2cmAttr);
    
    if ((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)||(pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)) 
    {
        if(((pI2cmAttr->sw_clk_pin == 0) || (pI2cmAttr->sw_data_pin == 0)) && (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "SW I2C pin assign error !!\r\n");
		    status = MMP_I2CM_ERR_PARAMETER;
		    goto L_I2cmOut;
		}
		
		status |= MMPF_I2c_Start(pI2cmAttr);
        status |= MMPF_I2c_WriteData(pI2cmAttr, 0x60);
        status |= MMPF_I2c_GetACK(pI2cmAttr);
        
        status |= MMPF_I2c_WriteData(pI2cmAttr, usSeg);
	    status |= MMPF_I2c_GetACK(pI2cmAttr);
	    
	    status |= MMPF_I2c_Start(pI2cmAttr);
        status |= MMPF_I2c_WriteData(pI2cmAttr, 0xA0);
        status |= MMPF_I2c_GetACK(pI2cmAttr);
        
        status |= MMPF_I2c_WriteData(pI2cmAttr, usOffset);
	    status |= MMPF_I2c_GetACK(pI2cmAttr);
	    
	    status |= MMPF_I2c_Start(pI2cmAttr);
        status |= MMPF_I2c_WriteData(pI2cmAttr, 0xA1);
        status |= MMPF_I2c_GetACK(pI2cmAttr);
        
        if((usSize + usOffset) > 256) {
            usSize = 256 - usOffset;
        }
        
        for(i = 0; i < usSize; i++) {
            *(ubData + i) = MMPF_I2c_ReadData(pI2cmAttr);
            if(i != (usSize - 1)) {
                status |= MMPF_I2c_SendACK(pI2cmAttr);
            } else {
                status |= MMPF_I2c_SendNACK(pI2cmAttr);
            }
        }
        status |= MMPF_I2c_Stop(pI2cmAttr);
    } 
    else 
    {
        #if (CHIP == VSN_V3)
		RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Not Support HW E-EDID Read !!\r\n");
		status = MMP_I2CM_ERR_PARAMETER;
        goto L_I2cmOut;
		#endif
        #if (CHIP == MERCURY)
        TODO
        #endif
		
		#if (CHIP == MCR_V2)
		if( pI2cmAttr->i2cmID == MMPF_I2CM_ID_0 || 
		    pI2cmAttr->i2cmID == MMPF_I2CM_ID_1 || 
		    pI2cmAttr->i2cmID == MMPF_I2CM_ID_2 ||
		    pI2cmAttr->i2cmID == MMPF_I2CM_ID_3) 
		{
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR             = 0x50;
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_EDDC_SLAVE_ADDR       = 0x30 | I2CM_EDDC_ENABLE;
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_EDDC_MODE_ADDR    = usSeg;
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_MODE_ADDR         = usOffset;
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_RX_ST_ADDR        = (MMP_ULONG)ubData;
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_RX_BYTE_CNT       = usSize;
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_MODE_CTRL         = I2CM_DMA_RX_EN;
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL                   = I2CM_REPEAT_MODE_EN | I2CM_MASTER_EN;
		    
		    if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
			    ulI2cmTimeOut = m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID];
    		}
    		
    		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN) 
    		{
    			if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
    				ulI2cmTimeOut--;
    				if(ulI2cmTimeOut == 0) {
    					return MMP_I2CM_ERR_READ_TIMEOUT;
    				}
    				else {
    					//Note, to use RxTimeout function, please make sure the read operations work in "task mode" instead of "ISR mode"
    					MMPF_OS_Sleep(1);
    				}
    			}
    		}
    		
    		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
    			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
    			if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
    				
    				if (ubSemStatus == OS_NO_ERR) {
    					MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
    				}
    			}
    			return MMP_I2CM_ERR_SLAVE_NO_ACK;
    		}
    		
    		// HDMI E-DDC has a bug, need to use software work-around    		
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_HOST_SR = I2CM_SLAVE_NO_ACK;
    		
    		*((volatile MMP_UBYTE *)AIT_OPR_P2V(0x80005D6A)) &= ~(0x04);
    		
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR         = 0x00 | I2CM_WRITE_MODE; // write
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_EDDC_SLAVE_ADDR   = 0x00;		
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_TX_ST_ADDR    = 0x100000; // temp write address   		
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_TX_BYTE_CNT   = 50;
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_MODE_CTRL     = I2CM_DMA_TX_EN;    		
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL               = I2CM_CONTI_IF_NOACK | I2CM_MASTER_EN;
    		
    		while(!(pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_HOST_SR & I2CM_SLAVE_NO_ACK));
    		
    		MMPF_OS_Sleep(1);
    		
    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_EDDC_SLAVE_ADDR |= I2CM_EDDC_ENABLE;
    		
    		if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
			    ulI2cmTimeOut = m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID];
    		}
    		
    		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN) {
    			if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
    				ulI2cmTimeOut --;
    				if(ulI2cmTimeOut == 0) {
    					return MMP_I2CM_ERR_READ_TIMEOUT;
    				}
    				else {
    					//Note, to use RxTimeout function, please make sure the read operations work in "task mode" instead of "ISR mode"
    					MMPF_OS_Sleep(1);
    				}
    			}
    		}
		    
		} else {
		    return MMP_I2CM_ERR_PARAMETER;
		}
		#endif
    }

L_I2cmOut: 
   
    if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
    return status;
}
#endif
//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_ReadReg
//  Description : The function read one set data from slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function read one set data from slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address.
 * @param[out] usData   : stands for the received data.
 * @return It return the function status. 
 */
MMP_ERR	MMPF_I2cm_ReadReg(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT *usData)
{
    return MMPF_I2cm_ReadRegSet(pI2cmAttr, &usReg, usData, 1);
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_WriteRegSet
//  Description : The function write multi-set data to slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function write multi-set data to slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address. If pI2cmAttr->bWfclModeEn is enalbe. usReg is no use.
 * @param[in] usData    : stands for the data to be sent.
 * @param[in] usSetCnt  : stands for the set count to be sent.
 * @return It return the function status. 
 */
MMP_ERR	MMPF_I2cm_WriteRegSet(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT *usReg, MMP_USHORT *usData, MMP_UBYTE usSetCnt)
{
	MMP_USHORT  i;
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
	MMP_ERR		status = MMP_ERR_NONE;

	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
#if 0	
	if ((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)||(pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)) 
	{
		for (i = 0; i < usSetCnt; i++) 
		{      	
        	status |= MMPF_I2cm_Initialize(pI2cmAttr);
        	
    		if(((pI2cmAttr->sw_clk_pin == 0) || (pI2cmAttr->sw_data_pin == 0)) && (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)) {
    			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "SW I2C pin assign error !!\r\n");
    			status = MMP_I2CM_ERR_PARAMETER;
    			goto L_I2cmOut;
    		}
    		
    	    status |= MMPF_I2c_Start(pI2cmAttr);
    	    status |= MMPF_I2c_WriteData(pI2cmAttr, m_bSwI2cmSlaveAddr[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
    	    status |= MMPF_I2c_GetACK(pI2cmAttr);
    		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
    		
    		if(pI2cmAttr->bWfclModeEn == MMP_FALSE){
        		//if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
        		if (pI2cmAttr->ubRegLen== 16) {
        		    status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)(usReg[i] >> 8));
        	    	status |= MMPF_I2c_GetACK(pI2cmAttr);
        		}
            	status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)usReg[i]);
           		status |= MMPF_I2c_GetACK(pI2cmAttr);
    		    RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
            }

    		//if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
    		if (pI2cmAttr->ubDataLen == 16) {
    		    	status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)(usData[i] >> 8));
    	    		status |= MMPF_I2c_GetACK(pI2cmAttr);
    		}
       		status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)usData[i]);
        	status |= MMPF_I2c_GetACK(pI2cmAttr);
    		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);

    	    status |= MMPF_I2c_Stop(pI2cmAttr);
        	
            if (status != MMP_ERR_NONE){
                return	status;
		    }
		}
	}
	else 
#endif
	{	
		status |= MMPF_I2cm_Initialize(pI2cmAttr);
		
		//if ((usSetCnt)*((m_bDataLen[pI2cmAttr->i2cmID]>>3)*2 + 1) > I2CM_TX_FIFO_DEPTH) {  //EROY CHECK
		if ( (usSetCnt)*((pI2cmAttr->ubDataLen>>3)*2 + 1) > I2CM_TX_FIFO_DEPTH) {  //EROY CHECK
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Attention! !! OVER I2C TX FIFO SIZE\r\n");
			
			if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
				if (ubSemStatus == OS_NO_ERR) {
					MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
				}
			}
			
			while(usSetCnt > 3) {
				status |= MMPF_I2cm_WriteRegSet(pI2cmAttr, usReg, usData, 3);
				usReg    = usReg + 3;
				usData   = usData + 3;
				usSetCnt = usSetCnt - 3;
			}
			status |= MMPF_I2cm_WriteRegSet(pI2cmAttr, usReg, usData, usSetCnt);
			return status;
		}
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_TX_DONE;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);

	    for (i = 0; i < usSetCnt; i++) 
	    {
		if(pI2cmAttr->bWfclModeEn == MMP_FALSE){
    	    	//if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
    	    	if (pI2cmAttr->ubRegLen == 16) {
    	    	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usReg[i] >> 8);
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg[i];
    	    	}
    		    else {
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg[i];
    	    	}
	    	
		//if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
		if (pI2cmAttr->ubDataLen == 16) {
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = 2;
    	    	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usData[i] >> 8);
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    	        }
    	        else {
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = 1;
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    	        }
    	    }else{
    		    //if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
    		 if (pI2cmAttr->ubDataLen == 16) {   
    	    	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usData[i] >> 8);
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    	        }
    	        else {
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    	        }
    	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = 0; //EROY CHECK
    	    }
	    }
	    
	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SET_CNT = usSetCnt;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_WRITE_MODE;
		
		#if (I2CM_INT_MODE_EN == 0x1)
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_EN |= I2CM_TX_DONE;
		#endif
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN;
		
		#if (I2CM_INT_MODE_EN == 0x1)
		if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
			MMPF_OS_AcquireSem(gI2cmIntSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
		}
		#else
		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN);

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			//RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
			pr_debug("I2C Error SLAVE_NO_ACK\r\n");
			
			//Clear FIFO. Otherwise, keeping for next transmission.
	        	pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
			
			status =  MMP_I2CM_ERR_SLAVE_NO_ACK;
			goto L_I2cmOut;
		}
		#endif
    }

L_I2cmOut:
    
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {

		if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
		    
    return status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_ReadRegSet
//  Description : The function read multi-set data from slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function read multi-set data from slave device.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address.
 * @param[in] usData    : stands for the received data.
 * @param[in] usSetCnt  : stands for the set count to be sent.
 * @return It return the function status. 
 */
MMP_ERR	MMPF_I2cm_ReadRegSet(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT *usReg, MMP_USHORT *usData, MMP_UBYTE usSetCnt)
{
	MMP_USHORT  i;
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
	MMP_ERR		status = MMP_ERR_NONE;
	MMP_ULONG	ulI2cmTimeOut = 0;

	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
#if 0	
	if ((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)||(pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)) 
	{
		for (i = 0; i < usSetCnt; i++) 
		{ 
        	status |= MMPF_I2cm_Initialize(pI2cmAttr);

    		if(((pI2cmAttr->sw_clk_pin == 0) || (pI2cmAttr->sw_data_pin == 0)) && (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)) {
    			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "SW I2C pin assign error !!\r\n");
    			status =  MMP_I2CM_ERR_PARAMETER;
    			goto L_I2cmOut;
    		}
    		
    		if(pI2cmAttr->bRfclModeEn == MMP_FALSE)
    		{
        	    status |= MMPF_I2c_Start(pI2cmAttr);
        	    status |= MMPF_I2c_WriteData(pI2cmAttr, m_bSwI2cmSlaveAddr[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
        	    status |= MMPF_I2c_GetACK(pI2cmAttr);
        		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
        	    
        		if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
        		    status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)(usReg[i] >> 8));
        	    	status |= MMPF_I2c_GetACK(pI2cmAttr);
        		}
            	status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)usReg[i]);
            	status |= MMPF_I2c_GetACK(pI2cmAttr);
        		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
        		
        	    status |= MMPF_I2c_Stop(pI2cmAttr);
            }
            
    	    status |= MMPF_I2c_Start(pI2cmAttr);
    	    status |= MMPF_I2c_WriteData(pI2cmAttr, m_bSwI2cmSlaveAddr[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW] + 1);
    	    status |= MMPF_I2c_GetACK(pI2cmAttr);
    		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);

    		usData[i] = 0;
    	    if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
    		    usData[i] = ((MMP_USHORT)MMPF_I2c_ReadData(pI2cmAttr)) << 8;
    	    	status |= MMPF_I2c_SendACK(pI2cmAttr);
    	    }
    	    usData[i] |= MMPF_I2c_ReadData(pI2cmAttr);
    	    status |= MMPF_I2c_SendNACK(pI2cmAttr);
    		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
    		
    	    status |= MMPF_I2c_Stop(pI2cmAttr);

            if (status != MMP_ERR_NONE){
                return status;
		    }
		}
	}
	else
#endif
	{		
		status |= MMPF_I2cm_Initialize(pI2cmAttr);
		
		//if ((usSetCnt)*(m_bDataLen[pI2cmAttr->i2cmID]>>3) > I2CM_RX_FIFO_DEPTH) 
		if ((usSetCnt)*(pI2cmAttr->ubDataLen>>3) > I2CM_RX_FIFO_DEPTH) 
		{  
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Attention! !! OVER I2C RX FIFO SIZE\r\n");
			
			if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
				if (ubSemStatus == OS_NO_ERR) {
					MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
				}
			}
			
			while(usSetCnt > 4) {
				status |= MMPF_I2cm_ReadRegSet(pI2cmAttr, usReg, usData, 4);
				usReg    = usReg + 4;
				usData   = usData + 4;
				usSetCnt = usSetCnt - 4;
			}
			status |= MMPF_I2cm_ReadRegSet(pI2cmAttr, usReg, usData, usSetCnt);
			return status;
		}
		
		// Rx need to reset FIFO first.	
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;

		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_TX_DONE;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);

	    for (i = 0; i < usSetCnt; i++) 
	    {
            if(pI2cmAttr->bRfclModeEn == MMP_FALSE){
    	    	//if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
    	    	if (pI2cmAttr->ubRegLen == 16) {
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usReg[i] >> 8);
    		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg[i];
    	    	}
    		    else {
    	    	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg[i];
    		    }
    		}
    		
	    	//if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
	    	if (pI2cmAttr->ubDataLen == 16) {
		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = 2;
	        }
	        else {
		        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = 1;
	        }
	    }

	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SET_CNT = usSetCnt;

		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_READ_MODE;
		#if (I2CM_INT_MODE_EN == 0x1)
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_EN |= I2CM_TX_DONE;
		#endif
		
		if(pI2cmAttr->bRfclModeEn == MMP_FALSE){
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN;
		}else{
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN | I2CM_RFCL_MODE_EN;
		}
		
		#if (I2CM_INT_MODE_EN == 0x1)
		if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
			MMPF_OS_AcquireSem(gI2cmIntSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
		}
		#else
		if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
			ulI2cmTimeOut = m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID];
		}
		
		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN) 
		{
			if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
				ulI2cmTimeOut --;
				if(ulI2cmTimeOut == 0) {
				    status = MMP_I2CM_ERR_READ_TIMEOUT;
					goto L_I2cmOut;
				}
				else {
					//Note, to use RxTimeout function, please make sure the read operations work in "task mode" instead of "ISR mode"
					MMPF_OS_Sleep(1);
				}
			}
		}

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
			
			//Clear FIFO. Otherwise, keeping for next transmission.
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
			
			status = MMP_I2CM_ERR_SLAVE_NO_ACK;
			goto L_I2cmOut;
		}
		#endif
		
		// Read Data from Rx FIFO
	    for (i = 0; i < usSetCnt; i++) {
	    	//if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
	    	if (pI2cmAttr->ubDataLen == 16) {
		        usData[i]  = (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RXFIFO_DATA.B[0] << 8);
		        usData[i] += (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RXFIFO_DATA.B[0]);
			 pr_debug("i2c addr=0x%.4X , data = 0x%.4x\r\n",(uint32_t)usReg[i],(uint32_t)usData[i]);
	    	}
		    else {
		        usData[i] = pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RXFIFO_DATA.B[0];
			 pr_debug("i2c addr=0x%.4X ,data = 0x%.2x\r\n",(uint32_t)usReg[i],(uint32_t)usData[i]);
	    	}
	    }
	}

L_I2cmOut:

    if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
    	if (ubSemStatus == OS_NO_ERR) {
    		MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
    	}
    }

    return status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_WriteBurstData
//  Description : The function burst write data to slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function burst write data to slave device. (reg address will auto increament)
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address. If pI2cmAttr->bWfclModeEn is enalbe. usReg is no use.
 * @param[in] usData    : stands for the data to be sent.
 * @param[in] usDataCnt : stands for the set count to be sent.
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_WriteBurstData(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT *usData, MMP_UBYTE usDataCnt)
{
	MMP_USHORT  i;
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
	MMP_ERR		status = MMP_ERR_NONE;
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	
	status |= MMPF_I2cm_Initialize(pI2cmAttr);
#if 0
	
	if ((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)||(pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)) 
	{
		if(((pI2cmAttr->sw_clk_pin == 0) || (pI2cmAttr->sw_data_pin == 0)) && (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "SW I2C pin assign error !!\r\n");
			status = MMP_I2CM_ERR_PARAMETER;
			goto L_I2cmOut;
		}
		
	    status |= MMPF_I2c_Start(pI2cmAttr);
	    status |= MMPF_I2c_WriteData(pI2cmAttr, m_bSwI2cmSlaveAddr[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
	    status |= MMPF_I2c_GetACK(pI2cmAttr);
		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
		
		if(pI2cmAttr->bWfclModeEn == MMP_FALSE){
    		if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
    		    status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)(usReg >> 8));
    	    	status |= MMPF_I2c_GetACK(pI2cmAttr);
    		}
    	    status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)usReg);
    	   	status |= MMPF_I2c_GetACK(pI2cmAttr);
    		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
        }

		for( i = 0; i < usDataCnt; i++) {
			if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
			    status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)(usData[i] >> 8));
		    	status |= MMPF_I2c_GetACK(pI2cmAttr);
			}	    
		   	status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)usData[i]);
		    status |= MMPF_I2c_GetACK(pI2cmAttr);
		    RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
	    }
	    
	    status |= MMPF_I2c_Stop(pI2cmAttr);
	}
	else 
#endif
	{	
		if ((usDataCnt*(m_bDataLen[pI2cmAttr->i2cmID]>>3) + 3) > I2CM_TX_FIFO_DEPTH) {  //3 for the case with 16bit register address and one byte for byte count
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Attention! !! OVER I2C TX FIFO SIZE\r\n");
			
			if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
				if (ubSemStatus == 0) {
					MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
				}
			}
			
			while(usDataCnt > 6) {
				status |= MMPF_I2cm_WriteBurstData(pI2cmAttr, usReg, usData, 6);
				usReg     = usReg + 6;
				usData    = usData + 6;
				usDataCnt = usDataCnt - 6;
			}
			status |= MMPF_I2cm_WriteBurstData(pI2cmAttr, usReg, usData, usDataCnt);
			return status;
		}
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_TX_DONE;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);

	    if(pI2cmAttr->bWfclModeEn == MMP_FALSE){
    	    if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
    	    	pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usReg >> 8);
    		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg;
    	    }
    		else {
    		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg;
    	   	}
	    		
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (m_bDataLen[pI2cmAttr->i2cmID] >> 3)*usDataCnt;
	        
    	    for (i = 0; i < usDataCnt; i++) {
    	    	if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
    	    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usData[i] >> 8);
    			    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    		    }
    		    else {
    			    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    		    }
    	   	}
	    }else{
            //EROY CHECK
	    	if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
	    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usData[0] >> 8);
			    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[0];
		    }
		    else {
			    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[0];
		    }
		    
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (m_bDataLen[pI2cmAttr->i2cmID] >> 3)*(usDataCnt - 1);
	        
    	    for (i = 1; i < usDataCnt; i++) {
    	    	if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
    	    		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usData[i] >> 8);
    			    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    		    }
    		    else {
    			    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData[i];
    		    }
    	   	}
	    }
	    	
	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SET_CNT = 1;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_WRITE_MODE;
		
		#if (I2CM_INT_MODE_EN == 0x1)
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_EN |= I2CM_TX_DONE;
		#endif
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN;
		
		#if (I2CM_INT_MODE_EN == 0x1)
		if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
			MMPF_OS_AcquireSem(gI2cmIntSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
		}
		#else
		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN);

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
			
			//Clear FIFO. Otherwise, keeping for next transmission.
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
			
			status = MMP_I2CM_ERR_SLAVE_NO_ACK;
			goto L_I2cmOut;
		}
		#endif
    }

L_I2cmOut:
    
    if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
	    if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
    return status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_ReadBurstData
//  Description : The function burst read data from slave device.
//------------------------------------------------------------------------------
/** 
 * @brief The function burst read data from slave device. (reg address will auto increament)
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address.
 * @param[in] usData    : stands for the address to stored data.
 * @param[in] usDataCnt : stands for the set count to be read.
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_ReadBurstData(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT *usData, MMP_UBYTE usDataCnt)
{
	MMP_USHORT  i;
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
	MMP_ERR		status = MMP_ERR_NONE;
	MMP_ULONG	ulI2cmTimeOut = 0;
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	
	status |= MMPF_I2cm_Initialize(pI2cmAttr);
#if 0
	if ((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)||(pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)) 
	{
		if(((pI2cmAttr->sw_clk_pin == 0) || (pI2cmAttr->sw_data_pin == 0)) && (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "SW I2C pin assign error !!\r\n");
			status =  MMP_I2CM_ERR_PARAMETER;
			goto L_I2cmOut;
		}
		
		if(pI2cmAttr->bRfclModeEn == MMP_FALSE){
		
    	    status |= MMPF_I2c_Start(pI2cmAttr);
    	    status |= MMPF_I2c_WriteData(pI2cmAttr, m_bSwI2cmSlaveAddr[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
    	    status |= MMPF_I2c_GetACK(pI2cmAttr);
    		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
    	    
    		if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
    		    status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)(usReg >> 8));
    	    	status |= MMPF_I2c_GetACK(pI2cmAttr);
    		}
    	    status |= MMPF_I2c_WriteData(pI2cmAttr, (MMP_UBYTE)usReg);
    	    status |= MMPF_I2c_GetACK(pI2cmAttr);
    		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
    		
    	    status |= MMPF_I2c_Stop(pI2cmAttr);
        }
        
	    status |= MMPF_I2c_Start(pI2cmAttr);
	    status |= MMPF_I2c_WriteData(pI2cmAttr, m_bSwI2cmSlaveAddr[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW] + 1);
	    status |= MMPF_I2c_GetACK(pI2cmAttr);
		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);

		for(i = 0; i < usDataCnt; i++) 
		{
			usData[i] = 0;
		    if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
			    usData[i] = ((MMP_USHORT)MMPF_I2c_ReadData(pI2cmAttr)) << 8;
		    	status |= MMPF_I2c_SendACK(pI2cmAttr);
		    }
		    usData[i] |= MMPF_I2c_ReadData(pI2cmAttr);
		    
		    if((usDataCnt > 1) && (i != (usDataCnt -1))) {
		    	status |= MMPF_I2c_SendACK(pI2cmAttr);
		    }
		    RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
	    }
	
	    status |= MMPF_I2c_SendNACK(pI2cmAttr);
		RTNA_WAIT_US(m_bSwI2cmDelayTime[pI2cmAttr->i2cmID - MMPF_I2CM_ID_SW]);
	    
	    status |= MMPF_I2c_Stop(pI2cmAttr);
	}
	else 
#endif
	{	
		if ((usDataCnt*(m_bDataLen[pI2cmAttr->i2cmID]>>3)) > I2CM_RX_FIFO_DEPTH) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "Attention! !! OVER I2C RX FIFO SIZE\r\n");
			
			if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
				if (ubSemStatus == OS_NO_ERR) {
					MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
				}
			}
			
			while(usDataCnt > 4) {
				status |= MMPF_I2cm_ReadBurstData(pI2cmAttr, usReg, usData, 4);
				usReg     = usReg + 4;
				usData    = usData + 4;
				usDataCnt = usDataCnt - 4;
			}
			status |= MMPF_I2cm_ReadBurstData(pI2cmAttr, usReg, usData, usDataCnt);
			return status;
		}
		
		// Rx need to reset FIFO first.
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;

		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_TX_DONE;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);

        if(pI2cmAttr->bRfclModeEn == MMP_FALSE){
    		if (m_bRegLen[pI2cmAttr->i2cmID] == 16) {
    			pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)(usReg >> 8);
    		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg;
    	    }
    		else {
    	    	pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usReg;
    		}
	    }
	    
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (m_bDataLen[pI2cmAttr->i2cmID]>>3)*usDataCnt;
	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SET_CNT = 1;
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_READ_MODE;
		#if (I2CM_INT_MODE_EN == 0x1)
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_EN |= I2CM_TX_DONE;
		#endif
		
		if(pI2cmAttr->bRfclModeEn == MMP_FALSE){
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN;
		}else{
		    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN | I2CM_RFCL_MODE_EN;
		}
		
		#if (I2CM_INT_MODE_EN == 0x1)
		if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
			MMPF_OS_AcquireSem(gI2cmIntSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
		}
		#else
		if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
			ulI2cmTimeOut = m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID];
		}
		
		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN) {
			if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
				ulI2cmTimeOut --;
				if(ulI2cmTimeOut == 0) {
					status = MMP_I2CM_ERR_READ_TIMEOUT;
				    goto L_I2cmOut;
				}
				else {
					//Note, to use RxTimeout function, please make sure the read operations work in "task mode" instead of "ISR mode"
					MMPF_OS_Sleep(1);
				}
			}
		}

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
			
			//Clear FIFO. Otherwise, keeping for next transmission.
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
			
			status = MMP_I2CM_ERR_SLAVE_NO_ACK;
			goto L_I2cmOut;
		}
		#endif
		
	    for (i = 0; i < usDataCnt; i++) {
	    	if (m_bDataLen[pI2cmAttr->i2cmID] == 16) {
		        usData[i] = (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RXFIFO_DATA.B[0] << 8);
		        usData[i] += (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RXFIFO_DATA.B[0]);
	    	}
		    else {
		        usData[i] = pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RXFIFO_DATA.B[0];
	    	}
	    }      
	}

L_I2cmOut:
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
	
    return status;
}
#if 0
//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_WriteNoRegMode
//  Description : The function write data to slave device without register address.
//------------------------------------------------------------------------------
/** 
 * @brief The function write data to slave device without register address.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usData    : stands for the data to be sent.
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_WriteNoRegMode(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usData)
{
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
	MMP_ERR		status = MMP_ERR_NONE;
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {	
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	
	status |= MMPF_I2cm_Initialize(pI2cmAttr);
	
	if ((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)||(pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)) {
	
	}
	else 
	{
		if(pI2cmAttr->ubRegLen != 0) {
			RTNA_DBG_Str(0, "NoRegMode only support No register address !! \r\n");
			status = MMP_SYSTEM_ERR_FORMAT;
		    goto L_I2cmOut;
		}
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_TX_DONE;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = (MMP_UBYTE)usData;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = 0; //EROY CHECK
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SET_CNT = 1;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_WRITE_MODE;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN;
		
		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN);

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
			
	        //Clear FIFO. Otherwise, keeping for next transmission.
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
	        
	        status = MMP_I2CM_ERR_SLAVE_NO_ACK;
	        goto L_I2cmOut;
		}
	}
	
L_I2cmOut:
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
	
	return MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_ReadNoRegMode
//  Description : The function read data from slave device without register address.
//------------------------------------------------------------------------------
/** 
 * @brief The function read data from slave device without register address.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[out] usData   : stands for the received data.
 * @param[in] usDataCnt : stands for the set count to be read.
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_ReadNoRegMode(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT *usData, MMP_UBYTE usDataCnt)
{
	AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
	MMP_ERR		status = MMP_ERR_NONE;
	MMP_ULONG	ulI2cmTimeOut = 0, i = 0;
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	
	status |= MMPF_I2cm_Initialize(pI2cmAttr);
	
	if ((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW)||(pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)) {
	
	}
	else 
	{
		if(pI2cmAttr->ubRegLen != 0) {
			RTNA_DBG_Str(0, "NoRegMode only support No register address !! \r\n");
			status =  MMP_SYSTEM_ERR_FORMAT;
		    goto L_I2cmOut;
		}
		
		// Rx need to reset FIFO first.	
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_TX_DONE;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);
		
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_TXFIFO_DATA.B[0] = usDataCnt; 
	    
	    pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SET_CNT = 1;
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_READ_MODE;	
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= (I2CM_MASTER_EN | I2CM_RFCL_MODE_EN);
		
		#if (I2CM_INT_MODE_EN != 0x1)
		if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
			ulI2cmTimeOut = m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID];
		}
		
		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN) {
			if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
				ulI2cmTimeOut --;
				if(ulI2cmTimeOut == 0) {
				    status = MMP_I2CM_ERR_READ_TIMEOUT;
					goto L_I2cmOut;
				}
				else {
					//Note, to use RxTimeout function, please make sure the read operations work in "task mode" instead of "ISR mode"
					MMPF_OS_Sleep(1);
				}
			}
		}

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
	        
	        //Clear FIFO. Otherwise, keeping for next transmission.
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
	        
	        status = MMP_I2CM_ERR_SLAVE_NO_ACK;
	        goto L_I2cmOut;
		}
		#endif
		
		for(i = usDataCnt; i > 0; i--) {
			*usData = pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RXFIFO_DATA.B[0];
			usData++;
		}	
	}

L_I2cmOut:
	
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
	
    return status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_DMAWriteBurstData
//  Description : The function write data to slave device using DMA burst mode.
//------------------------------------------------------------------------------
/** 
 * @brief The function write data to slave device using DMA burst mode.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address.
 * @param[in] usData    : stands for the data to be sent.
 * @param[in] usDataCnt : stands for the byte count to be writed.
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_DMAWriteBurstData(MMPF_I2CM_ATTRIBUTE *pI2cmAttr, 
                                    MMP_USHORT usReg, MMP_UBYTE *usData,
                                    MMP_USHORT usDataCnt)
{
    AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
    MMP_ERR     status = MMP_ERR_NONE;

	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	
	status |= MMPF_I2cm_Initialize(pI2cmAttr);

    if((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW) || (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)){
        RTNA_DBG_Str(0, "I2CM DMA only support HW I2C\r\n");
    }
    else
    {
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_TX_DONE;
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);

        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_MODE_ADDR     = usReg;
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_TX_BYTE_CNT   = usDataCnt;  
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_TX_ST_ADDR    = (MMP_ULONG)usData;

        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_MODE_CTRL |= I2CM_DMA_TX_EN;
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_WRITE_MODE;
        #if (I2CM_INT_MODE_EN == 0x1)
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_EN |= I2CM_DMA_TX2FIFO_DONE;
        #endif
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN;

        #if (I2CM_INT_MODE_EN != 0x1)
        while(pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN);

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
			
	        //Clear FIFO. Otherwise, keeping for next transmission.
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
	        
	        status = MMP_I2CM_ERR_SLAVE_NO_ACK;
		    goto L_I2cmOut;
		}
        #endif
    }
    
L_I2cmOut:

	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
    
    return status;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_I2cm_DMAReadBurstData
//  Description : The function read data from slave device using DMA burst mode.
//------------------------------------------------------------------------------
/** 
 * @brief The function read data from slave device using DMA burst mode.
 *
 * @param[in] pI2cmAttr : stands for the i2cm attribute.       
 * @param[in] usReg     : stands for the register sub address.
 * @param[in] usData    : stands for the buffer address to store data.
 * @param[in] usDataCnt : stands for the byte count to be read.
 * @return It return the function status. 
 */
MMP_ERR MMPF_I2cm_DMAReadBurstData(MMPF_I2CM_ATTRIBUTE *pI2cmAttr,
                                    MMP_USHORT usReg, MMP_UBYTE *usData,
                                    MMP_USHORT usDataCnt)
{
    AITPS_I2CM  pI2CM = AITC_BASE_I2CM;
	MMP_UBYTE	ubSemStatus = 0xFF;
    MMP_ERR     status = MMP_ERR_NONE;
    MMP_ULONG	ulI2cmTimeOut = 0;
    
	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		ubSemStatus = MMPF_OS_AcquireSem(gI2cmSemID[pI2cmAttr->i2cmID], I2CM_SEM_TIMEOUT);
	}
	
	status |= MMPF_I2cm_Initialize(pI2cmAttr);

    if((pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW) || (pI2cmAttr->i2cmID == MMPF_I2CM_ID_SW_SENSOR)){
        RTNA_DBG_Str(0, "I2CM DMA only support HW I2C\r\n");
    }
    else
    {
        // Rx need to reset FIFO first.	
		pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
    
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR = I2CM_SLAVE_NO_ACK | I2CM_DMA_RX2MEM_DONE;
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR &= ~(I2CM_READ_MODE | I2CM_WRITE_MODE);

        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_MODE_ADDR     = usReg;
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_RX_BYTE_CNT   = usDataCnt;  
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_RX_ST_ADDR    = (MMP_ULONG)usData;

        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_DMA_MODE_CTRL |= I2CM_DMA_RX_EN;
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_SLAV_ADDR |= I2CM_READ_MODE;
        #if (I2CM_INT_MODE_EN == 0x1)
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_EN |= I2CM_DMA_RX2MEM_DONE;
        #endif
        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL |= I2CM_MASTER_EN;

        #if (I2CM_INT_MODE_EN != 0x1)
		if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
			ulI2cmTimeOut = m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID];
		}
		
		while (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_CTL & I2CM_MASTER_EN) {
			if(m_ulI2cRxReadTimeout[pI2cmAttr->i2cmID] != 0) {
				ulI2cmTimeOut --;
				if(ulI2cmTimeOut == 0) {
				    status = MMP_I2CM_ERR_READ_TIMEOUT;
					goto L_I2cmOut;
				}
				else {
					//Note, to use RxTimeout function, please make sure the read operations work in "task mode" instead of "ISR mode"
					MMPF_OS_Sleep(1);
				}
			}
		}

		if (pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_INT_CPU_SR & I2CM_SLAVE_NO_ACK) {
			RTNA_DBG_Str(I2CM_DEBUG_LEVEL, "I2C Error SLAVE_NO_ACK\r\n");
			
	        //Clear FIFO. Otherwise, keeping for next transmission.
	        pI2CM->I2CMS[pI2cmAttr->i2cmID].I2CM_RST_FIFO_SW = I2CM_FIFO_RST;
	        
	        status = MMP_I2CM_ERR_SLAVE_NO_ACK;
	        goto L_I2cmOut;
		}
        #endif
    }

L_I2cmOut:

	if (pI2cmAttr->bOsProtectEn == MMP_TRUE) {
		if (ubSemStatus == OS_NO_ERR) {
			MMPF_OS_ReleaseSem(gI2cmSemID[pI2cmAttr->i2cmID]);   
		}
	}
    
    return status;
}
#endif
/** @}*/ //end of MMPF_I2CM
