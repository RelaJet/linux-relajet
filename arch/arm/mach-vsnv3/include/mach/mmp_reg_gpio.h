//==============================================================================
//
//  File        : mmp_register_gpio.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Rogers Chen
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REG_GPIO_H_
#define _MMP_REG_GPIO_H_

#include "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/

#if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
//------------------------------
// GPIO Structure (0x8000 6600)
//------------------------------
typedef struct _AITS_GPIO {
    AIT_REG_D   GPIO_OUT_EN[4];             // 0x00
    AIT_REG_D   GPIO_DATA[4];               // 0x10
    AIT_REG_D   GPIO_INT_H2L_EN[4];         // 0x20
    AIT_REG_D   GPIO_INT_L2H_EN[4];         // 0x30
    AIT_REG_D   GPIO_INT_H_EN[4];        	// 0x40
    AIT_REG_D   GPIO_INT_L_EN[4];         	// 0x50
    AIT_REG_D   GPIO_INT_CPU_EN[4];         // 0x60
    AIT_REG_D   GPIO_INT_H2L_SR[4];         // 0x70
	AIT_REG_D   GPIO_INT_L2H_SR[4];         // 0x80
	AIT_REG_D   GPIO_INT_H_SR[4];        	// 0x90
	AIT_REG_D   GPIO_INT_L_SR[4];        	// 0xA0
	AIT_REG_D   GPIO_INT_HOST_EN[4];        // 0xB0
    AIT_REG_D   GPIO_INT_HOST_H2L_SR[4];    // 0xC0
	AIT_REG_D   GPIO_INT_HOST_L2H_SR[4];    // 0xD0
	AIT_REG_D   GPIO_INT_HOST_H_SR[4];      // 0xE0
    AIT_REG_D   GPIO_INT_HOST_L_SR[4];      // 0xF0
} AITS_GPIO, *AITPS_GPIO;

#if (CHIP == MCR_V2)
//------------------------------
// GPIO Counter Structure (0x8000 5500)
//------------------------------
typedef struct _AITS_GPIO_CNT {
    
    AIT_REG_B   GPIO_CNT0_CTL;              // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define GPIO_COUNTER_EN     0x01
        #define GPIO_COUNTER_L2H    0x00
        #define GPIO_COUNTER_H2L    0x02
        #define GPIO_COUNTER_INC    0x00
        #define GPIO_COUNTER_DEC    0x04
        /*------------------------------------------------------------*/
    AIT_REG_B   GPIO_CNT0_SRC_SEL;          // 0x01
        /*-DEFINE-----------------------------------------------------*/
        #define GPIO_CNT0_SRC_MASK  0xFF
        /*------------------------------------------------------------*/
    AIT_REG_W   GPIO_CNT0_INT_VAL;          // 0x02
    AIT_REG_W   GPIO_CNT0_TAR_VAL;          // 0x04
    AIT_REG_B                   _0x06[0x2];
    AIT_REG_W   GPIO_CNT0_CUR_VAL;          // 0x08
    AIT_REG_B                   _0x0A[0x6];
    
    AIT_REG_B   GPIO_CNT0_CPU_INT_EN;       // 0x10
    AIT_REG_B   GPIO_CNT0_HOST_INT_EN;      // 0x11
    AIT_REG_B   GPIO_CNT0_CPU_INT_SR;       // 0x12
    AIT_REG_B   GPIO_CNT0_HOST_INT_SR;      // 0x13    
    AIT_REG_B                   _0x14[0xC];

    AIT_REG_B   GPIO_CNT1_CTL;              // 0x20
    AIT_REG_B   GPIO_CNT1_SRC_SEL;          // 0x21
    AIT_REG_W   GPIO_CNT1_INT_VAL;          // 0x22
    AIT_REG_W   GPIO_CNT1_TAR_VAL;          // 0x24
    AIT_REG_B                   _0x26[0x2];
    AIT_REG_W   GPIO_CNT1_CUR_VAL;          // 0x28
    AIT_REG_B                   _0x2A[0x6];
    
    AIT_REG_B   GPIO_CNT1_CPU_INT_EN;       // 0x30
    AIT_REG_B   GPIO_CNT1_HOST_INT_EN;      // 0x31
    AIT_REG_B   GPIO_CNT1_CPU_INT_SR;       // 0x32
    AIT_REG_B   GPIO_CNT1_HOST_INT_SR;      // 0x33    
    AIT_REG_B                   _0x34[0xC];    
    
    AIT_REG_B   GPIO_WAKEUP_INT_EN;         // 0x40
        /*-DEFINE-----------------------------------------------------*/
        #define GPIO_WAKEUP_MODE_EN 0x01
        /*------------------------------------------------------------*/
} AITS_GPIO_CNT, *AITPS_GPIO_CNT;
#endif // (CHIP == MCR_V2)

typedef struct _AITS_PWM {
    #if (CHIP == VSN_V3)
    AIT_REG_B   PWM_INT_HOST_EN;            // 0x00
    AIT_REG_B                   _0x01[15];
    AIT_REG_B   PWM_INT_HOST_SR;            // 0x10
    AIT_REG_B                   _0x11[15];
    AIT_REG_B   PWM_INT_CPU_EN;             // 0x20
    AIT_REG_B                  _0x21[15];
    AIT_REG_B   PWM_INT_CPU_SR;             // 0x30
    AIT_REG_B                   _0x31[15];

    AIT_REG_W   PWM_PULSE_A_T0;             // 0x40
    AIT_REG_W                   _0x42;
    AIT_REG_W   PWM_PULSE_A_T1;             // 0x44
    AIT_REG_W                   _0x46;
    AIT_REG_W   PWM_PULSE_A_T2;             // 0x48
    AIT_REG_W                   _0x4A;
    AIT_REG_W   PWM_PULSE_A_T3;             // 0x4C
    AIT_REG_W                   _0x4E;
    AIT_REG_W   PWM_PULSE_A_CLK_DIV;        // 0x50
    AIT_REG_W                   _0x52[7];

    AIT_REG_W   PWM_PULSE_B_T0;             // 0x60
    AIT_REG_W                   _0x62;
    AIT_REG_W   PWM_PULSE_B_T1;             // 0x64
    AIT_REG_W                   _0x66;
    AIT_REG_W   PWM_PULSE_B_T2;             // 0x68
    AIT_REG_W                   _0x6A;
    AIT_REG_W   PWM_PULSE_B_T3;             // 0x6C
    AIT_REG_W                   _0x6E;
    AIT_REG_W   PWM_PULSE_B_CLK_DIV;        // 0x70
    AIT_REG_W                   _0x72[7];
    AIT_REG_D                   _0x80[16];
    #endif

     AIT_REG_B   PWM_CTL;                   // 0xC0, 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define PWM_DEFAULT_OUT_LOW     0x00
        #define PWM_DEFAULT_OUT_HIGH    0x20
        #define PWM_PULSE_A_FIRST       0x00
        #define PWM_PULSE_B_FIRST       0x10
        #define PWM_ONE_ROUND	        0x00
        #define PWM_AUTO_CYC            0x08
        #define PWM_PULSE_B_POS         0x00
        #define PWM_PULSE_B_NEG         0x04
        #define PWM_PULSE_A_POS         0x00
        #define PWM_PULSE_A_NEG         0x02
        #define PWM_EN                  0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   _x01;
    AIT_REG_B   PWM_PULSE_A_NUM;            // 0xC2, 0x02
    AIT_REG_B   _x03;
    AIT_REG_B   PWM_PULSE_B_NUM;            // 0xC4, 0x04
    #if (CHIP == VSN_V3)
    AIT_REG_B                   _0xC5;
    AIT_REG_W                   _0xC6[5];
    AIT_REG_D                   _0xD0[12];
	#endif

	#if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AIT_REG_B   _x05[0x0B];

    AIT_REG_B   PWM_INT_HOST_EN;            // 0x10
    AIT_REG_B   _x11;
    AIT_REG_B   PWM_INT_HOST_SR;            // 0x12
    AIT_REG_B   _x13;
    AIT_REG_B   PWM_INT_CPU_EN;             // 0x14
    AIT_REG_B   _x15;
    AIT_REG_B   PWM_INT_CPU_SR;             // 0x16
    AIT_REG_B   _x17[0x29];
        /*-DEFINE-----------------------------------------------------*/
        #define PWM_ONE_ROUND_PASSED    0x04
        #define PWM_ONE_PULSEB_PASSED   0x02
        #define PWM_ONE_PULSEA_PASSED   0x01
        /*------------------------------------------------------------*/        

    AIT_REG_D   PWM_PULSE_A_T0;             // 0x40
    AIT_REG_D   PWM_PULSE_A_T1;             // 0x44
    AIT_REG_D   PWM_PULSE_A_T2;             // 0x48
    AIT_REG_D   PWM_PULSE_A_T3;             // 0x4C
    
    AIT_REG_D   PWM_PULSE_A_PEROID;         // 0x50
    AIT_REG_B   _x54[0x0C];
    
    AIT_REG_D   PWM_PULSE_B_T0;             // 0x60
    AIT_REG_D   PWM_PULSE_B_T1;             // 0x64
    AIT_REG_D   PWM_PULSE_B_T2;             // 0x68
    AIT_REG_D   PWM_PULSE_B_T3;             // 0x6C
    
    AIT_REG_D   PWM_PULSE_B_PEROID;         // 0x70
    AIT_REG_B   _x74[0x0C];
	#endif
} AITS_PWM, *AITPS_PWM;

// ********************************
//   PWM structure (0x8000 0800)
// ********************************
typedef struct _AITS_PWMB {
    #if (CHIP == VSN_V3)
	AITS_PWM    PWM[4];                     // 0x0800 ~ 0x0BFF
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AITS_PWM    PWM0[8];
    AIT_REG_B   _xC00[0x2F00];
    AITS_PWM    PWM1[10];           
	#endif
} AITS_PWMB, *AITPS_PWMB;
#endif // (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)

/// @}
#endif // _MMPH_REG_GPIO_H_
