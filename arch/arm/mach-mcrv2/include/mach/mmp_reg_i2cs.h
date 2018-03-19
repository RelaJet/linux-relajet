//==============================================================================
//
//  File        : mmp_reg_i2cs.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Eroy Yang
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_I2CS_H_
#define _MMP_REG_I2CS_H_

#include "mmp_register.h"

#if (CHIP == MCR_V2)
//--------------------------------
// I2CM structure (0x8000 3800)
//--------------------------------
typedef struct _AITS_I2CS {
    AIT_REG_B   I2CS_CTL;                                               // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define I2CS_SLAVE_EN               0x01
        #define I2CS_REG_16_MODE            0x02
        #define I2CS_REG_8_MODE             0x00
        #define I2CS_SDA_OH_MODE            0x04
        #define I2CS_SDA_OD_MODE            0x00
        #define I2CS_INPUT_FILTERN_DIS		0x08
        #define I2CS_ERR_TIMER_EN           0x10
        #define I2CS_ERR_RECOVERY_EN        0x20
        #define I2CS_STRETCH_EN	            0x40
         /*------------------------------------------------------------*/
    AIT_REG_B	_x01;
    AIT_REG_B   I2CS_DATA_HOLD_CNT;                                     // 0x02
    AIT_REG_B   I2CS_START_CHK_CNT;                                     // 0x03
    AIT_REG_B   I2CS_ERR_TIMER;                                         // 0x04
    AIT_REG_B   I2CS_CPU_WR_CTL;                                        // 0x05
    AIT_REG_B   I2CS_FSM_SR;                                            // 0x06
    	/*-DEFINE-----------------------------------------------------*/
        #define I2CS_FSM_SR_IDLE	        0x01
        #define I2CS_FSM_SR_DEV_ADDR	    0x02
        #define I2CS_FSM_SR_SLV_ACK         0x04
        #define I2CS_FSM_SR_REG_ADDR	    0x08
        #define I2CS_FSM_SR_CHK_STOP	    0x10
        #define I2CS_FSM_SR_WR_DATA	        0x20
        #define I2CS_FSM_SR_RD_DATA	        0x40
        #define I2CS_FSM_SR_MAS_ACK	        0x80
        /*------------------------------------------------------------*/
    AIT_REG_B   I2CS_SR;                                                // 0x07
    AIT_REG_W   I2CS_INT_HOST_EN;                                       // 0x08
    AIT_REG_W   I2CS_INT_HOST_SR;                                       // 0x0A
    AIT_REG_W   I2CS_INT_CPU_EN;                                        // 0x0C
    AIT_REG_W   I2CS_INT_CPU_SR;                                        // 0x0E
        /*-DEFINE-----------------------------------------------------*/
        #define I2CS_DET_START_BIT          0x0001
        #define I2CS_DET_STOP_BIT           0x0002
        #define I2CS_DET_SCK_RISE           0x0004
    	#define I2CS_DET_SCK_FALL		    0x0008        
    	#define I2CS_DET_SDA_RISE		    0x0010
    	#define I2CS_DET_SDA_FALL 		    0x0020
        #define I2CS_ERR_TIMER_TIMEOUT	    0x0040
        #define I2CS_ERR_RECOVER_OCCUR      0x0080
        /*------------------------------------------------------------*/
 
} AITS_I2CS, *AITPS_I2CS;
#endif

#endif // _MMP_REG_I2CS_H_