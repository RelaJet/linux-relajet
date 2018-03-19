//==============================================================================
//
//  File        : lib_retina.h
//  Description : Retina library header file
//  Author      : Jerry Tsao
//  Revision    : 1.0
//
//==============================================================================


#ifndef _LIB_RETINA_H_
#define _LIB_RETINA_H_

#include "config_fw.h"
#include "mmp_register.h"
#include "reg_retina.h"

#include "mmp_reg_gbl.h"
#if (OS_TYPE == OS_UCOSII)
#include "reg_pcm.h"
#endif

/** @addtogroup BSP
@{
*/

//==============================================================================
//
//                              COMPILER OPTION
//
//==============================================================================

#define DEBUG_RX_EN         (1)

//==============================================================================
//
//                              CPU FUNCTIONS
//
//==============================================================================

#if (OS_TYPE == OS_UCOSII)
#ifndef __thumb // 32-bit ARM instruction only

__inline unsigned int RTNA_ARM_ReadControl()
{
    register unsigned int ctl;
    __asm("MRC p15, 0, ctl, c1, c0, 0");
    return ctl;
}

__inline void RTNA_ARM_PowerDown()
{
    register unsigned int ctl = 0;
    __asm("MCR p15, 0, ctl, c7, c0, 4");
}

#endif // __thrumb
#endif // (OS_TYPE == OS_UCOSII)
#if (OS_TYPE == OS_LINUX)
//todo
#endif

//==============================================================================
//
//                              AIC FUNCTIONS
//
//==============================================================================


typedef void (IntHandler)(void);

static __inline void RTNA_AIC_SetProtectMode(AITPS_AIC pAIC, MMP_BOOL on)
{
    if (on)
        pAIC->AIC_DBR |= AIC_DBG_EN;
    else
        pAIC->AIC_DBR &= ~AIC_DBG_EN;
}

static __inline void RTNA_AIC_SetMode(AITPS_AIC pAIC, MMP_ULONG id, MMP_ULONG mask, MMP_ULONG mode)
{
    pAIC->AIC_SMR[id] = (pAIC->AIC_SMR[id] & ~mask) | (mode & mask);
}

static __inline void RTNA_AIC_SetVector(AITPS_AIC pAIC, MMP_ULONG id, MMP_ULONG vector)
{
    pAIC->AIC_SVR[id] = vector;
}

static __inline MMP_BOOL RTNA_AIC_GetState(AITPS_AIC pAIC, MMP_ULONG id)
{
    #if (CHIP == VSN_V3)
    return !!(pAIC->AIC_IMR & (1 << id));
    #endif

    #if (CHIP == MERCURY) || (CHIP == MCR_V2)
    return (id < 32)? (!!(pAIC->AIC_IMR_LSB & (1 << id))): (!!(pAIC->AIC_IMR_MSB & (1 << (id & 0x1F))));
    #endif
}

static __inline void RTNA_AIC_Open(AITPS_AIC pAIC, MMP_ULONG id, IntHandler pfHandler, MMP_ULONG type)
{
    #if (OS_TYPE == OS_UCOSII)
    #if (CHIP == VSN_V3)
    // Disable Interrupt of the specified ID on AIC
    pAIC->AIC_IDCR = 0x1 << id;
    #endif

    #if (CHIP == MERCURY) || (CHIP == MCR_V2)
    // Disable Interrupt of the specified ID on AIC
    if (id < 32)
        pAIC->AIC_IDCR_LSB = 0x1 << id;
    else
        pAIC->AIC_IDCR_MSB = 0x1 << (id - 32);
    #endif

    // Setup the vector for the specified interrupt
    pAIC->AIC_SVR[id] = (MMP_ULONG)pfHandler;
    // Setup the priority/trigger mode
    pAIC->AIC_SMR[id] = type;
    #endif
}

static __inline void RTNA_AIC_IRQ_En(AITPS_AIC pAIC, MMP_ULONG id)
{
    // Enable interrupt at AIC level
    #if (CHIP == VSN_V3)
    pAIC->AIC_IECR = 0x1 << id;
    #endif
    #if (CHIP == MERCURY) || (CHIP == MCR_V2)
    if (id < 32)
        pAIC->AIC_IECR_LSB = 0x1 << id;
    else
        pAIC->AIC_IECR_MSB = 0x1 << (id - 32);
    #endif
}

static __inline void RTNA_AIC_IRQ_Dis(AITPS_AIC pAIC, MMP_ULONG id)
{
    // Disable timer interrupt at AIC level
    #if (CHIP == VSN_V3)
    pAIC->AIC_IDCR = 0x1 << id;
    #endif
    #if (CHIP == MERCURY) || (CHIP == MCR_V2)
    if (id < 32)
        pAIC->AIC_IDCR_LSB = 0x1 << id;
    else
        pAIC->AIC_IDCR_MSB = 0x1 << (id - 32);
    #endif
}

static __inline void RTNA_AIC_IRQ_DisAll(AITPS_AIC pAIC)
{
    // Disable timer interrupt at AIC level
    #if (CHIP == VSN_V3)
    pAIC->AIC_IDCR = 0xFFFFFFFF;
    #endif
    #if (CHIP == MERCURY) || (CHIP == MCR_V2)
    pAIC->AIC_IDCR_LSB = 0xFFFFFFFF;
    pAIC->AIC_IDCR_MSB = 0xFFFFFFFF;
    #endif
}

static __inline void RTNA_AIC_IRQ_ClearAll(AITPS_AIC pAIC)
{
    // Clear ALL interrups if any pending
    #if (CHIP == VSN_V3)
    pAIC->AIC_ICCR = 0xFFFFFFFF;
    #endif
    #if (CHIP == MERCURY) || (CHIP == MCR_V2)
    pAIC->AIC_ICCR_LSB = 0xFFFFFFFF;
    pAIC->AIC_ICCR_MSB = 0xFFFFFFFF;
    #endif
}

static __inline void RTNA_AIC_IRQ_EOI(AITPS_AIC pAIC)
{
    // Support both protect mode and non-protect mode
    pAIC->AIC_IVR = pAIC->AIC_IVR;
    pAIC->AIC_EOICR = pAIC->AIC_EOICR;
}

static __inline void RTNA_AIC_IRQ_EOICR(AITPS_AIC pAIC)
{
    pAIC->AIC_EOICR = 0;
}

//==============================================================================
//
//                              TC FUNCTIONS
//
//==============================================================================
static __inline void RTNA_TC_SR_Clear(AITPS_TC pTC)
{
    // both for normal mode and debug mode
    pTC->TC_SR = pTC->TC_SR;
}

static __inline void RTNA_TC_Close(AITPS_TC pTC)
{
    pTC->TC_CCR = TC_CLKDIS;
    pTC->TC_IDR = TC_CPCS;
}

static __inline void RTNA_TC_Open(AITPS_TC pTC, MMP_ULONG mode, MMP_ULONG clkcnt)
{
    pTC->TC_CCR = TC_CLKDIS;
    pTC->TC_IDR = TC_CPCS;

    RTNA_TC_SR_Clear(pTC);                      // clear interrupt status register

    pTC->TC_CMR = mode;

    pTC->TC_RC = clkcnt;
    pTC->TC_CCR = TC_CLKEN;
}

static __inline void RTNA_TC_IRQ_En(AITPS_TC pTC, MMP_ULONG type)
{
    RTNA_TC_SR_Clear(pTC);                      // clear interrupt status register
    pTC->TC_IER = type;
}

static __inline void RTNA_TC_IRQ_Dis(AITPS_TC pTC, MMP_ULONG type)
{
    pTC->TC_IDR = type;
    RTNA_TC_SR_Clear(pTC);                      // clear interrupt status register
}

static __inline void RTNA_TC_SYNC(AITPS_TCB pTCB)
{
    pTCB->TC_BCR = TC_SYNC;
}

static __inline void RTNA_TC_DBG_En(AITPS_TCB pTCB)
{
    pTCB->TC_DBR |= TC_DBG_EN;
}

static __inline void RTNA_TC_DBG_Dis(AITPS_TCB pTCB)
{
    pTCB->TC_DBR &= ~TC_DBG_EN;
}
//==============================================================================
//
//                              USART FUNCTIONS
//
//==============================================================================


//==============================================================================
//
//                              DBG FUNCTIONS
//
//==============================================================================
#if 0
void    RTNA_DBG_Open(MMP_ULONG, MMP_ULONG);
/** @name Debug Messages
These functions are used to print out the debug messages.
@{ */
void    RTNA_DBG_Str(MMP_ULONG, const char*);
void    RTNA_DBG_Long(MMP_ULONG, MMP_ULONG);
void    RTNA_DBG_Short(MMP_ULONG, MMP_USHORT);
void    RTNA_DBG_Byte(MMP_ULONG, MMP_UBYTE);
//void    RTNA_DBG_Hex(MMP_ULONG level, MMP_ULONG val, int bit);
void MMPF_DBG_Int(MMP_ULONG val, MMP_SHORT digits);
#endif

/// @}
/** @page internal 
@section debug_level Debug Output and Debug Levels
- Purpose
  - Support compile time inclusion of debugging messages
    so that we can reduce code size in the same code by changing debug level
  - For incoming code base integration of Retina and Diamond,
    the symbols start with RTNA_ are considered to be revised.
  - The decimal output is also required from developers.
- Design
  - Because we use these symbols very frequently in the development,
    the new naming is designed as short as possible.

@warning The old naming RTNA_ are considered as obsolete in few months. In this period, old naming is still valid for
backward compatibility. 
*/
/** @page internal
@par Debug level setting:
If the users use the macro that is greater than this level,
then the code will not be compiled and shall generate nothing to reduce the code size.
Proposed level rules:\n
- 0:Solution messages
  - Show in the solution and evaluation kit
  - Should print very few lines such as firmware version and vital error code.
- 1:Demo messages
  - Show in the demo kit and the solution
  - Mainly for helping the demo and the demo kit setting
  - Could print many messages in start up but not in runtime.
- 2:Team Debug messages
  - Helping other members to debug
  - These messages should be readable by other members.
- 3:Module Debug messages
  - Most developing debug messages should go here.
  - Print any messages you want.
  - Other people might not understand the messages in this level.
- Abbreviation
  - _B: Byte (MMP_UBYTE)
  - _W: Word (MMP_USHORT)
  - _L: Long (MMP_ULONG)
  - _S: String (MMP_UBYTE*)
  - _D: Decimal (MMP_ULONG), The digits are assigned by users
  
@par Example
@code
MMP_USHORT us = 0x1234;
MMP_UBYTE ub = 0x12;
MMP_ULONG ul = 0x12345678;
MMP_ULONG ul2 = 0x87654321;
DBG_B(2, ub);  // Print one byte the value only
DBG_B3(ub);    // You could also use this, 3 is the level. However VAR_B3(ub) is not available.
DBG_S(2, " "); // Print a string
VAR_B(2, ub);  // Print the variable name and its value and new line for easier debugging
DBG_W(2, us);
VAR_W(2, us);
DBG_L(3, ul);
VAR_L(3, ul);
//Beware how the most significant digits are trimmed or added
DBG_S(3, "Decimal Example\r\n");
DBG_D(3, us, 3);// output us in 3 digits decimal
DBG_D(3, us, 4);// output us in 4 digits decimal
DBG_D(3, us, 5);// output us in 5 digits decimal
//Notice that the input value is long. Only long integer will be signed
DBG_S(3, "\r\nNegative Numbers\r\n");
DBG_D(3, 0xABCD, 5); // output us in 5 digits decimal
DBG_D(3, -0xABCD, 5);
DBG_D(3, -us, 5);    
DBG_D(3, ul2, 10);   // output us in 10 digits decimal
//Custom digits examples. Beware how the most significant digits are trimmed
DBG_S(3, "\r\nCustom Digits\r\n");
DBG(3, ul, 6);// 6 is digits(nibbles).
VAR(3, ul, 6);// 6 is digits(nibbles).
ASSERT(2, ul == ul); // Assertion, it will pass.
ASSERT(3, ul == ul2);// Assertion, it will half this task.
BREAK_POINT; // Break the task when pass the assert

Output when DBG_LEVEL = 3
 x12 x12 ub: x12
 x1234us: x1234
 x12345678ul: x12345678
Decimal Example
 660 4660 04660
Negative Numbers
 43981-43981-04660-2023406815
Custom Digits
 x345678ul: x345678

Assert:ul == ul2
mmpf_m_3gpparser.c, Line 03576
Break

Output when DBG_LEVEL = 2
 x12 ub: x12
 x1234us: x1234

Break

@endcode
*/

/** @name Code Size Saving Debug Messages
These functions are used to print out the debug messages with code size saving capabilities.
@{
*/
#if DBG_LEVEL >= 0
	#define RTNA_DBG_Str0(x) RTNA_DBG_Str(0,x);
	#define RTNA_DBG_Long0(x) RTNA_DBG_Long(0, x);
	#define RTNA_DBG_Short0(x) RTNA_DBG_Short(0, x);
	#define RTNA_DBG_Byte0(x) RTNA_DBG_Byte(0, x);
#else
	#define RTNA_DBG_Str0(x)
	#define RTNA_DBG_Long0(x)
	#define RTNA_DBG_Short0(x)
	#define RTNA_DBG_Byte0(x)
#endif

#if DBG_LEVEL >= 1
	#define RTNA_DBG_Str1(x) RTNA_DBG_Str(1,x);
	#define RTNA_DBG_Long1(x) RTNA_DBG_Long(1, x);
	#define RTNA_DBG_Short1(x) RTNA_DBG_Short(1, x);
	#define RTNA_DBG_Byte1(x) RTNA_DBG_Byte(1, x);
#else
	#define RTNA_DBG_Str1(x)
	#define RTNA_DBG_Long1(x)
	#define RTNA_DBG_Short1(x)
	#define RTNA_DBG_Byte1(x)
#endif

#if DBG_LEVEL >= 2
	#define RTNA_DBG_Str2(x) RTNA_DBG_Str(2,x);
	#define RTNA_DBG_Long2(x) RTNA_DBG_Long(2, x);
	#define RTNA_DBG_Short2(x) RTNA_DBG_Short(2, x);
	#define RTNA_DBG_Byte2(x) RTNA_DBG_Byte(2, x);
#else
	#define RTNA_DBG_Str2(x)
	#define RTNA_DBG_Long2(x)
	#define RTNA_DBG_Short2(x)
	#define RTNA_DBG_Byte2(x)
#endif

#if DBG_LEVEL >= 3
	#define RTNA_DBG_Str3(x) RTNA_DBG_Str(3,x);
	#define RTNA_DBG_Long3(x) RTNA_DBG_Long(3, x);
	#define RTNA_DBG_Short3(x) RTNA_DBG_Short(3, x);
	#define RTNA_DBG_Byte3(x) RTNA_DBG_Byte(3, x);
#else
	#define RTNA_DBG_Str3(x)
	#define RTNA_DBG_Long3(x)
	#define RTNA_DBG_Short3(x)
	#define RTNA_DBG_Byte3(x)
#endif

// code size saving macro
#define RTNA_DBG_PrintLong(level, x) { RTNA_DBG_Str##level(#x":"); RTNA_DBG_Long##level((x)); RTNA_DBG_Str##level("\r\n"); }
#define RTNA_DBG_PrintShort(level, x) { RTNA_DBG_Str##level(#x":"); RTNA_DBG_Short##level((x)); RTNA_DBG_Str##level("\r\n"); }
#define RTNA_DBG_PrintByte(level, x) { RTNA_DBG_Str##level(#x":"); RTNA_DBG_Byte##level((x)); RTNA_DBG_Str##level("\r\n"); }

#if 0
#if DBG_LEVEL >= 0
    #define DBG_B0(x) MMPF_DBG_Int(x, 2)
    #define DBG_W0(x) MMPF_DBG_Int(x, 4)
    #define DBG_L0(x) MMPF_DBG_Int(x, 8)
    #define DBG_S0(x) RTNA_DBG_Str(0, x)
    #define DBG_D0(x, digits) MMPF_DBG_Int(x, -digits)
    #define DBG0(x, digits) MMPF_DBG_Int(x, digits)
#else
    #define DBG_B0(x)
    #define DBG_W0(x)
    #define DBG_L0(x)
    #define DBG_S0(x)
    #define DBG_D0(x, digits)
    #define DBG0(x, digits)
#endif

#if DBG_LEVEL >= 1
    #define DBG_B1(x) MMPF_DBG_Int(x, 2)
    #define DBG_W1(x) MMPF_DBG_Int(x, 4)
    #define DBG_L1(x) MMPF_DBG_Int(x, 8)
    #define DBG_S1(x) RTNA_DBG_Str(1, x)
    #define DBG_D1(x, digits) MMPF_DBG_Int(x, -digits)
    #define DBG1(x, digits) MMPF_DBG_Int(x, digits)
#else
    #define DBG_B1(x)
    #define DBG_W1(x)
    #define DBG_L1(x)
    #define DBG_S1(x)
    #define DBG_D1(x, digits)
    #define DBG1(x, digits)
#endif

#if DBG_LEVEL >= 2
    #define DBG_B2(x) MMPF_DBG_Int(x, 2)
    #define DBG_W2(x) MMPF_DBG_Int(x, 4)
    #define DBG_L2(x) MMPF_DBG_Int(x, 8)
    #define DBG_S2(x) RTNA_DBG_Str(2, x)
    #define DBG_D2(x, digits) MMPF_DBG_Int(x, -digits)
    #define DBG2(x, digits) MMPF_DBG_Int(x, digits)
#else
    #define DBG_B2(x)
    #define DBG_W2(x)
    #define DBG_L2(x)
    #define DBG_S2(x)
    #define DBG_D2(x, digits)
    #define DBG2(x, digits)
#endif

#if DBG_LEVEL >= 3
    #define DBG_B3(x) MMPF_DBG_Int(x, 2)
    #define DBG_W3(x) MMPF_DBG_Int(x, 4)
    #define DBG_L3(x) MMPF_DBG_Int(x, 8)
    #define DBG_S3(x) RTNA_DBG_Str(3, x)
    #define DBG_D3(x, digits) MMPF_DBG_Int(x, -digits)
    #define DBG3(x, bits) MMPF_DBG_Int(x, bits)
#else
    #define DBG_B3(x)
    #define DBG_W3(x)
    #define DBG_L3(x)
    #define DBG_S3(x)
    #define DBG_D3(x, digits)
    #define DBG3(x, bits)
#endif

#define DBG_B(level, x) DBG_B##level(x)
#define DBG_W(level, x) DBG_W##level(x)
#define DBG_L(level, x) DBG_L##level(x)
#define DBG_S(level, x) DBG_S##level(x)
#define DBG_D(level, x, digits) DBG_D##level(x, digits)
#define VAR_B(level, x) { DBG_S##level(#x":"); DBG_B##level(x); DBG_S##level("\r\n"); }
#define VAR_W(level, x) { DBG_S##level(#x":"); DBG_W##level(x); DBG_S##level("\r\n"); }
#define VAR_L(level, x) { DBG_S##level(#x":"); DBG_L##level((MMP_ULONG)(x)); DBG_S##level("\r\n"); }
#define VAR_D(level, x, digits) { DBG_S##level(#x":"); DBG_D##level(x, digits); DBG_S##level("\r\n"); }
#define DBG(level, x, bits) DBG##level(x, bits)
#define VAR(level, x, bits) { DBG_S##level(#x":"); DBG##level(x, bits); DBG_S##level("\r\n"); }


/// @}
// Other utilities
/** @name Utilities
Here lists some useful utilities in firmware.
@{
*/
/// Print out the function name without name mangling, 
/// useful to observe the branch condition and calling sequence
#define PRINT_FUNCTION DBG_S(3, __prettyfunc__"()\r\n")

#define PRINTF(...) do {} while(0)
#endif
/// @}

//==============================================================================
//
//                              MISC. FUNCTIONS & MACROS
//
//==============================================================================

void    RTNA_Wait_Count(MMP_ULONG count);
void    RTNA_Init(void);

#define WHILE_CYCLE                     6

#if (OS_TYPE == OS_UCOSII)
#define RTNA_WAIT_MS(ms)                RTNA_Wait_Count(ms * RTNA_CPU_CLK_M / WHILE_CYCLE * 1000)
#define RTNA_WAIT_US(us)                RTNA_Wait_Count(us * RTNA_CPU_CLK_M / WHILE_CYCLE)
#define RTNA_WAIT_CYCLE(cycle)          RTNA_Wait_Count(cycle / WHILE_CYCLE);
#endif
#if (OS_TYPE == OS_LINUX)
void RTNA_WAIT_MS(MMP_ULONG ms);
void RTNA_WAIT_US(MMP_ULONG us);
void RTNA_WAIT_CYCLE(MMP_ULONG cycle); 
#endif

//==============================================================================

//
//                              FUNCTION PROTOTYPE
//
//==============================================================================

#if (OS_TYPE == OS_LINUX)
#define spi_isr_a       (0)
#define hif_isr_a       (0)
#define vif_isr_a       (0)
#define gpio_isr_a      (0)
#define isp_isr_a       (0)
#define jpg_isr_a       (0)
#define wd_isr_a        (0)
#define dma_isr_a       (0)
#define vdp_isr_a       (0)
#define video_isr_a     (0)
#define memc_isr_a      (0)
#define h264enc_isr_a   (0)
#define vld_isr_a       (0)
#define i2cm_isr_a      (0)
#define i2s_isr_a       (0)
#define uart_isr_a      (0)
#define tcs_isr_a       (0)
#define afe_isr_a       (0)
#define nand_isr_a      (0)
#define usb_isr_a       (0)
#define usbotg_isr_a    (0)
#define sd_isr_a        (0)
#define display_isr_a   (0)
#define gra_isr_a       (0)
#define ibc_isr_a       (0)
#define h264dec_isr_a   (0)
#define pwm_isr_a       (0)
#define i2c_pwm_isr_a   (0)
#define scal_isr_a      (0)
#if (HW_MP3D_EN == 1)
#define mp3d_isr_a      (0)
#endif
#if (CHIP == MCR_V2)
#define irda_isr_a      (0)
#define ldc_isr_a       (0)
#endif
#endif // (OS_TYPE == OS_LINUX)
#if (OS_TYPE == OS_UCOSII)
extern void         spi_isr_a(void);
extern void         hif_isr_a(void);
extern void         vif_isr_a(void);
extern void         gpio_isr_a(void);
extern void         isp_isr_a(void);
extern void         jpg_isr_a(void);
extern void         wd_isr_a(void);
extern void         dma_isr_a(void);
extern void         vdp_isr_a(void);
extern void         video_isr_a(void);
extern void         memc_isr_a(void);
extern void         h264enc_isr_a(void);
extern void         vld_isr_a(void);
extern void			i2cm_isr_a(void);
extern void         i2s_isr_a(void);
extern void         uart_isr_a(void);
extern void         tcs_isr_a(void);
extern void         afe_isr_a(void);
extern void         nand_isr_a(void);
extern void         usb_isr_a(void);
extern void         usbotg_isr_a(void);
extern void         sd_isr_a(void);
extern void         display_isr_a(void);
extern void         gra_isr_a(void);
extern void         ibc_isr_a(void);
extern void         h264dec_isr_a(void);
extern void         pwm_isr_a(void);
extern void         i2c_pwm_isr_a(void);
extern void         scal_isr_a(void);
#if (HW_MP3D_EN == 1)
extern void         mp3d_isr_a(void);
#endif
#if (CHIP == MCR_V2)
extern void         irda_isr_a(void);
extern void         ldc_isr_a(void);
#endif
#endif // (OS_TYPE == OS_UCOSII)

/** @} */ // BSP

#endif // _LIB_RETINA_H_
/** @} */ // BSP
