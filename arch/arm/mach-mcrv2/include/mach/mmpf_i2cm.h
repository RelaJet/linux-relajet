//==============================================================================
//
//  File        : mmpf_i2cm.h
//  Description : INCLUDE File for the Firmware I2CM Control driver function
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPF_I2CM_H_
#define _MMPF_I2CM_H_

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include "includes_fw.h"
#include "mmpf_pio.h"

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#if (CHIP == P_V2)
#define I2CM_RX_FIFO_DEPTH		0x8
#define I2CM_TX_FIFO_DEPTH		0xF
#define I2CM_SW_MAX_SPEED		200000 	//For CPU 144 MHz, for loop delay set as 1, the one clock cycle is about 53 us
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
#define I2CM_RX_FIFO_DEPTH		0x20
#define I2CM_TX_FIFO_DEPTH		0x20
#define I2CM_SW_MAX_SPEED		200000 	//TBD For CPU 144 MHz, for loop delay set as 1, the one clock cycle is about 53 us
#endif

#define I2CM_DEBUG_LEVEL		0x0
#define I2CM_SEM_TIMEOUT 		0x0
#define I2CM_INT_MODE_EN		0x0		//I2CM Interrupt mode enable or not, 1 stands enable
                                        //Note: I2CM interrupt mode have worse performance and user should take care the risk of i2cm read
//Sensor HW I2C
#if !defined(SENSOR_SOFTWARE_I2CM)
#define SENSOR_I2CM_ID          MMPF_I2CM_ID_0
#else
#define SENSOR_I2CM_ID          MMPF_I2CM_ID_SW_SENSOR
#endif

#define I2CM_SW_MAX_COUNT   	0x2		//Number of supported SW I2CM
#if (CHIP == P_V2)
#define I2CM_HW_MAX_COUNT		0x3		//Number of HW I2CM controller
#endif                 
#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define I2CM_HW_MAX_COUNT		0x2		//Number of HW I2CM controller
#endif
#if (CHIP == MCR_V2)
#define I2CM_HW_MAX_COUNT		0x4		//Number of HW I2CM controller
#endif 

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

typedef enum _MMPF_I2CM_ID
{
    MMPF_I2CM_ID_0 = 0,   	//HW I2CM, reserved for Sensor HW I2C
    #if (CHIP == MERCURY) || (CHIP == VSN_V3)
    MMPF_I2CM_ID_1,
    #endif
    #if (CHIP == MCR_V2)
    /* Re-order due to the I2CM register map */
    MMPF_I2CM_ID_2,
    MMPF_I2CM_ID_1,
    MMPF_I2CM_ID_3,
    #endif
    MMPF_I2CM_ID_SW,		//SW I2CM, using GPIO pins
    MMPF_I2CM_ID_SW_SENSOR,	//SW I2CM, sensor uses VIF pins
    MMPF_I2CM_MAX_COUNT
} MMPF_I2CM_ID;

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef enum _MMPF_I2CM_SPEED_MODE
{
	MMPF_I2CM_SPEED_NONE = 0,
    MMPF_I2CM_SPEED_SW,  	//SW I2CM, the speed is control by each attribute ubDelayTime
    MMPF_I2CM_SPEED_HW_100K = 100*1000,
    MMPF_I2CM_SPEED_HW_250K = 250*1000,
    MMPF_I2CM_SPEED_HW_400K = 400*1000,
    MMPF_I2CM_SPEED_HW_550K = 550*1000,
    MMPF_I2CM_SPEED_HW_700K = 700*1000,
    MMPF_I2CM_SPEED_HW_850K = 850*1000,
    MMPF_I2CM_SPEED_HW_1MHZ = 1000*1000,
    MMPF_I2CM_MAX_SPEED_COUNT
} MMPF_I2CM_SPEED_MODE;
typedef struct _MMPF_I2CM_ATTRIBUTE
{
   MMPF_I2CM_ID i2cmID;				//MMPF_I2CM_ID_0 ~ MMPF_I2CM_ID_2 stand for HW I2CM
   MMP_UBYTE ubSlaveAddr;           //Indicate the slave device address.
   MMP_UBYTE ubRegLen;				//Indicate register as the 8 bit mode or 16 bit mode.
   MMP_UBYTE ubDataLen;				//Indicate data as the 8 bit mode or 16 bit mode.
   MMP_UBYTE ubDelayTime;			//The delay time after each ACK/NACK, which is used in SW I2CM (i2cmID = MMPF_I2CM_ID_SW or MMPF_I2CM_ID_SW_SENSOR)
   MMP_BOOL  bDelayWaitEn; 			//HW feature, to set delay between each I2CM accessing set
   MMP_BOOL  bInputFilterEn;		//HW feature, to filter input noise
   MMP_BOOL  b10BitModeEn;			//HW I2CM supports 10 bit slave address, the bit8 and bit9 are in ubSlaveAddr1
   MMP_BOOL	 bClkStretchEn; 		//HW support stretch clock
   MMP_UBYTE ubSlaveAddr1;			//10 bit slave address support used.
   MMP_UBYTE ubDelayCycle;  		//When bDelayWaitEn enable, set the delay cycle after each 8 bit transmission
   MMP_UBYTE ubPadNum;      		//HW pad map, the relate pad definition, please refer global register spec.
   MMP_ULONG ulI2cmSpeed; 			//SW and HW I2CM speed control, the unit is HZ.
   MMP_BOOL	 bOsProtectEn;			//Enable: I2CM driver with OS semaphore protect.
   									//Note: I2CM interrupt mode should always turn on OS protect
   MMPF_PIO_REG sw_clk_pin;  		//Used in SW I2CM (i2cmID = MMPF_I2CM_ID_SW only), indicate the clock pin
   MMPF_PIO_REG sw_data_pin;	 	//Used in SW I2CM (i2cmID = MMPF_I2CM_ID_SW only), indicate the data pin
   MMP_BOOL  bRfclModeEn;           //Used in read from current location mode. Read data without send register address.
   MMP_BOOL  bWfclModeEn;           //Used in write from current location mode. Write data without send register address.
   //kernel driver extension
   char* name;
   MMP_USHORT deviceID;
} MMPF_I2CM_ATTRIBUTE;

typedef void I2cmCallBackFunc(void);

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

MMP_ERR MMPF_I2cm_Initialize(MMPF_I2CM_ATTRIBUTE *i2cAttribute);
MMP_ERR	MMPF_I2cm_WriteReg(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT usData);
MMP_ERR	MMPF_I2cm_ReadReg(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT *usData);
MMP_ERR	MMPF_I2cm_WriteRegSet(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT *usReg, MMP_USHORT *usData, MMP_UBYTE usSetCnt);
MMP_ERR	MMPF_I2cm_ReadRegSet(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT *usReg, MMP_USHORT *usData, MMP_UBYTE usSetCnt);
MMP_ERR MMPF_I2cm_WriteBurstData(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT *usData, MMP_UBYTE usDataCnt);
MMP_ERR MMPF_I2cm_ReadBurstData(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_USHORT usReg, MMP_USHORT *usData, MMP_UBYTE usDataCnt);
MMP_UBYTE MMPF_I2cm_StartSemProtect(MMPF_I2CM_ATTRIBUTE* pI2cmAttr);
MMP_UBYTE MMPF_I2cm_EndSemProtect(MMPF_I2CM_ATTRIBUTE* pI2cmAttr);
MMP_ERR MMPF_I2cm_InitializeDriver(void);
MMP_ERR MMPF_I2cm_SetRxTimeout(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_ULONG ulTimeOut);
MMP_ERR MMPF_I2cm_WaitAllFree(void);
MMP_ERR MMPF_I2cm_SetAllFree(void);
MMP_ERR MMPF_I2Cm_ReadEEDID(MMPF_I2CM_ATTRIBUTE* pI2cmAttr, MMP_UBYTE *ubData, MMP_USHORT usSeg, MMP_USHORT usOffset, MMP_USHORT usSize);
MMP_ERR MMPF_I2cm_DMAWriteBurstData(MMPF_I2CM_ATTRIBUTE *pI2cmAttr, MMP_USHORT usReg, MMP_UBYTE *usData, MMP_USHORT usDataCnt);
MMP_ERR MMPF_I2cm_DMAReadBurstData(MMPF_I2CM_ATTRIBUTE *pI2cmAttr, MMP_USHORT usReg, MMP_UBYTE *usData, MMP_USHORT usDataCnt);

#endif // _MMPF_I2CM_H_
