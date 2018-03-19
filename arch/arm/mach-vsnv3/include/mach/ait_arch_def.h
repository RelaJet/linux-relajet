/*
 * arch/arm/mach-iat/include/mach/ait_arch_def.h
 *
 */
#ifndef AIT_ARCH_DEF_H
#define AIT_ARCH_DEF_H

/* For assembler's PA/VA mapping */
#if (CHIP == MCR_V2) || (CHIP == MERCURY) ||(CHIP == VSN_V3)
#define AIT_TC0_PHYS_BASE           (0xFFFE0000)    // TC0      Base Address
#define AIT_WD_PHYS_BASE            (0xFFFF8000)    // WD       Base Address
#define AIT_AIC_PHYS_BASE           (0xFFFFF000)    // AIC      Base Address

#define AIT_UART0_PHYS_BASE         (0x80006A00)    // UART0    Base Address
#endif

										
#endif
