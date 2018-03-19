//==============================================================================
//
//  File        : mmp_reg_sif.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================



#ifndef _MMP_REG_SIF_H_
#define _MMP_REG_SIF_H_

#include    "mmp_register.h"

/** @addtogroup MMPH_reg
@{
*/

// *******************************
//   SIF structure (0x8000 6700)
// *******************************
typedef struct _AITS_SIF {
    AIT_REG_B   SIF_INT_CPU_EN;					//0x00
    AIT_REG_B   _x01[3];
        /*-DEFINE-----------------------------------------------------*/
        // Refer to Offset 0x0C
        /*------------------------------------------------------------*/
    AIT_REG_B   SIF_INT_HOST_EN;				//0x04
    AIT_REG_B   _x05[3];
        /*-DEFINE-----------------------------------------------------*/
        // Refer to Offset 0x0C
        /*------------------------------------------------------------*/
    AIT_REG_B   SIF_INT_CPU_SR;					//0x08
    AIT_REG_B   _x09[3];
        /*-DEFINE-----------------------------------------------------*/
        // Refer to Offset 0x0C
        /*------------------------------------------------------------*/
    AIT_REG_B   SIF_INT_HOST_SR;				//0x0C
    AIT_REG_B	_x0D[3];
        /*-DEFINE-----------------------------------------------------*/
        #define SIF_CMD_DONE				0x01
        #define SIF_AAI_CMD_DONE			0x02
        #define SIF_CLR_CMD_STATUS			0x01
        #define SIF_CLR_AAI_CMD_STATUS		0x02
        /*------------------------------------------------------------*/
    AIT_REG_B   SIF_CTL;						//0x10
        /*-DEFINE-----------------------------------------------------*/
        #define SIF_START               	0x80
        #define SIF_FAST_READ         	 	0x40
        #define SIF_R           			0x20
        #define SIF_W           			0x00
        #define SIF_DMA_EN           		0x10
        #define SIF_DATA_EN           		0x08
        #define SIF_ADDR_EN           		0x04
        #define SIF_ADDR_LEN_2        		0x02
        #define SIF_ADDR_LEN_1        		0x01
        #define SIF_ADDR_UNDETERMINED	0x00
        #define SIF_ADDR_2_BYTE        		0x01
        #define SIF_ADDR_3_BYTE        		0x02		
        /*------------------------------------------------------------*/
    AIT_REG_B   SIF_CTL2;					//0x11
        /*-DEFINE-----------------------------------------------------*/
    	#define SIF_DUAL_IO_EN				(1<<4)
    	#define SIF_QUAD_IO_EN     			(1<<3)		
    	#define SIF_AUTO_LOAD_MASK_DIS     	0x00
		#define SIF_WRITE_2_BYTES			0x04
    	#define SIF_AUTO_LOAD_MASK_EN		0x02
		#define SIF_AAI_MODE_DIS        	0x00
		#define SIF_AAI_MODE_EN         	0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   SIF_CLK_DIV;					//0x12
    	// Target clock rate = source clock rate / 2(SIF_CLK_DIV + 1)
    	/*-DEFINE-----------------------------------------------------*/
    	#define SIF_SET_CLK_DIV(_a)     		(_a/2 - 1)
    	/*------------------------------------------------------------*/
    AIT_REG_B   SIF_PD_CPU;						//0x13
    AIT_REG_B   SIF_CMD;						//0x14
    AIT_REG_B   					_x15[3];
	AIT_REG_D   SIF_FLASH_ADDR;					//0x18

	//AIT_REG_B   					_x1C[1];
	AIT_REG_B	SIF_BIST_EN;					//0x1C
		/*-DEFINE-----------------------------------------------------*/
    	#define SIF_BIST_ENABLE     		0x01
        #define SIF_CRC_AUTO_CLEAN_EN       0x02
    	/*------------------------------------------------------------*/
	AIT_REG_B	SIF_DATA_IN_LATCH;				//0x1D
	AIT_REG_W	SIF_CRC_OUT;					//0x1E

	AIT_REG_W   SIF_DATA_WR;					//0x20

//	AIT_REG_B	SIF_DATA_WR1;					//0x21
	AIT_REG_B   					_x22[14];

	AIT_REG_B   SIF_DATA_RD;					//0x30
	AIT_REG_B   					_x31[15];
	AIT_REG_D   SIF_DMA_ST;						//0x40
	AIT_REG_D   SIF_DMA_CNT;					//0x44
	AIT_REG_B   SIF_AAI_INTER_CMD_DELAY;		//0x48
} AITS_SIF, *AITPS_SIF;

/* Register access macros */
#define SIF_REG_OFFSET(reg) offsetof(AITS_SIF,reg)

#define ait_sif_readb(port,reg) \
	__raw_readb((port)->regs + SIF_REG_OFFSET(reg))
#define ait_sif_writeb(port,reg,value) \
	__raw_writeb((value), (port)->regs + SIF_REG_OFFSET(reg))

#define ait_sif_readw(port,reg) \
	__raw_readw((port)->regs + SIF_REG_OFFSET(reg))
#define ait_sif_writew(port,reg,value) \
	__raw_writew((value), (port)->regs + SIF_REG_OFFSET(reg))

#define ait_sif_readl(port,reg) \
	__raw_readl((port)->regs + SIF_REG_OFFSET(reg))
#define ait_sif_writel(port,reg,value) \
	__raw_writel((value), (port)->regs + SIF_REG_OFFSET(reg))



#define SIF_GET_INT_EN(port) ait_sif_readb(port,SIF_INT_CPU_EN)
#define SIF_SET_INT_EN(port,v) ait_sif_writeb(port,SIF_INT_CPU_EN,v)

#define SIF_GET_INT_SR(port) ait_sif_readw(port,SIF_INT_CPU_SR)
#define SIF_SET_INT_SR(port,v) ait_sif_writew(port,SIF_INT_CPU_SR,v)

#define SIF_GET_CTL(port) ait_sif_readb(port,SIF_CTL)
#define SIF_SET_CTL(port,v) ait_sif_writeb(port,SIF_CTL,v)


#define SIF_GET_CTL2(port) ait_sif_readb(port,SIF_CTL2)
#define SIF_SET_CTL2(port,v) ait_sif_writeb(port,SIF_CTL2,v)


#define SIF_GET_CLK_DIV(port) ait_sif_readb(port,SIF_CLK_DIV)
#define SIF_SET_CLK_DIV(port,v) ait_sif_writeb(port,SIF_CLK_DIV,v)

#define SIF_GET_CMD(port) ait_sif_readb(port,SIF_CMD)
#define SIF_SET_CMD(port,v) ait_sif_writeb(port,SIF_CMD,v)

#define SIF_GET_ADDR(port) ait_sif_readl(port,SIF_FLASH_ADDR)
#define SIF_SET_ADDR(port,v) ait_sif_writel(port,SIF_FLASH_ADDR,v&0x00ffffff)

#define SIF_GET_DATA_IN_LATCH(port) ait_sif_readb(port,SIF_DATA_IN_LATCH)
#define SIF_SET_DATA_IN_LATCH(port,v) ait_sif_writeb(port,SIF_DATA_IN_LATCH,v)

#define SIF_GET_CRC_OUT(port) ait_sif_readw(port,SIF_CRC_OUT)
#define SIF_SET_CRC_OUT(port,v) ait_sif_writew(port,SIF_CRC_OUT,v)

#define SIF_GET_DATA_WR(port) ait_sif_readw(port,SIF_DATA_WR)
#define SIF_SET_DATA_WR(port,v) ait_sif_writew(port,SIF_DATA_WR,v)

#define SIF_SET_DATA2_WR(port,v) ait_sif_writew(port,SIF_DATA_WR,v)

#define SIF_GET_DATA_RD(port) ait_sif_readb(port,SIF_DATA_RD)
#define SIF_SET_DATA_RD(port,v) ait_sif_writeb(port,SIF_DATA_RD,v)

#define SIF_GET_DMA_ADDR(port) ait_sif_readl(port,SIF_DMA_ST)
#define SIF_SET_DMA_ADDR(port,v) ait_sif_writel(port,SIF_DMA_ST,v)

#define SIF_GET_DMA_CNT(port) ait_sif_readl(port,SIF_DMA_CNT)
#define SIF_SET_DMA_CNT(port,v) ait_sif_writel(port,SIF_DMA_CNT,(v-1)&0x00ffffff)

#define SIF_GET_CMD_DELAY(port) ait_sif_readb(port,SIF_AAI_INTER_CMD_DELAY)
#define SIF_SET_CMD_DELAY(port,v) ait_sif_writeb(port,SIF_AAI_INTER_CMD_DELAY,v)


#endif // _MMPH_REG_SIF_H_
