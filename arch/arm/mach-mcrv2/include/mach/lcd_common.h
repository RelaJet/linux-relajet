#ifndef _LCD_COMMON_H_
#define _LCD_COMMON_H_

//==============================================================================
//
//                              INCLUDE FILE
//
//==============================================================================

#include <mach/mmp_reg_display.h>
#include <mach/mmpf_display.h>

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define LCD_WAIT(cnt)          mdelay(cnt)
#define LCD_CMD(idx, cmd)      MMPF_LCD_SendIndexCommand(idx, cmd)
#define MAINLCD_CMD(idx)       MMPF_LCD_SendIndex(idx)
#define MAINLCD_DATA(cmd)      MMPF_LCD_SendCommand(cmd)

//////////////////////////////////////////////////////////////////

#define DSPY_DECL               			AITPS_DSPY pDSPY = AITC_BASE_DSPY

#if 0 //djkim.2015.02.27
#define DSPY_WR_B(reg, val)					(*(AIT_REG_B *)((MMP_ULONG)&(pDSPY->##reg))) = ((MMP_UBYTE)(val))
#define DSPY_WR_W(reg, val)					(*(AIT_REG_W *)((MMP_ULONG)&(pDSPY->##reg))) = ((MMP_USHORT)(val))
#define DSPY_WR_D(reg, val)					(*(AIT_REG_D *)((MMP_ULONG)&(pDSPY->##reg))) = ((MMP_ULONG)(val))
#define DSPY_WR_OFST_B(reg, ofst, val)		(*(AIT_REG_B *)((MMP_ULONG)&(pDSPY->##reg) + ofst)) = ((MMP_UBYTE)(val))
#define DSPY_WR_OFST_W(reg, ofst, val)		(*(AIT_REG_W *)((MMP_ULONG)&(pDSPY->##reg) + ofst)) = ((MMP_USHORT)(val))
#define DSPY_WR_OFST_D(reg, ofst, val)		(*(AIT_REG_D *)((MMP_ULONG)&(pDSPY->##reg) + ofst)) = ((MMP_ULONG)(val))

#define DSPY_RD_B(reg)						(MMP_UBYTE) (*(AIT_REG_B *)((MMP_ULONG)&(pDSPY->##reg)))
#define DSPY_RD_W(reg)						(MMP_USHORT)(*(AIT_REG_W *)((MMP_ULONG)&(pDSPY->##reg)))
#define DSPY_RD_D(reg)						(MMP_ULONG) (*(AIT_REG_D *)((MMP_ULONG)&(pDSPY->##reg)))
#define DSPY_RD_OFST_B(reg, ofst)			(MMP_UBYTE) (*(AIT_REG_B *)((MMP_ULONG)&(pDSPY->##reg) + ofst))
#define DSPY_RD_OFST_W(reg, ofst)			(MMP_USHORT)(*(AIT_REG_W *)((MMP_ULONG)&(pDSPY->##reg) + ofst))
#define DSPY_RD_OFST_D(reg, ofst)			(MMP_ULONG) (*(AIT_REG_D *)((MMP_ULONG)&(pDSPY->##reg) + ofst))

#define GBL_DECL							AITPS_GBL pGBL = AITC_BASE_GBL
#define GBL_WR_B(reg, val)					(*(AIT_REG_B *)((MMP_ULONG)&(pGBL->##reg))) = ((MMP_UBYTE)(val))
#define GBL_RD_B(reg)						(MMP_UBYTE) (*(AIT_REG_B *)((MMP_ULONG)&(pGBL->##reg)))

#define MEM_SET_W(addr, val)   				MMPH_HIF_MemSetW(addr, val)

#define WIN_OPR_OFST(_w)					((_w - LCD_DISP_WIN_PIP) * 0x100)

#else

#define DSPY_WR_B(reg, val)					(*(AIT_REG_B *)((MMP_ULONG)&(reg))) = ((MMP_UBYTE)(val))
#define DSPY_WR_W(reg, val)					(*(AIT_REG_W *)((MMP_ULONG)&(reg))) = ((MMP_USHORT)(val))
#define DSPY_WR_D(reg, val)					(*(AIT_REG_D *)((MMP_ULONG)&(reg))) = ((MMP_ULONG)(val))
#define DSPY_WR_OFST_B(reg, ofst, val)		(*(AIT_REG_B *)((MMP_ULONG)&(reg) + ofst)) = ((MMP_UBYTE)(val))
#define DSPY_WR_OFST_W(reg, ofst, val)		(*(AIT_REG_W *)((MMP_ULONG)&(reg) + ofst)) = ((MMP_USHORT)(val))
#define DSPY_WR_OFST_D(reg, ofst, val)		(*(AIT_REG_D *)((MMP_ULONG)&(reg) + ofst)) = ((MMP_ULONG)(val))

#define DSPY_RD_B(reg)						(MMP_UBYTE) (*(AIT_REG_B *)((MMP_ULONG)&(reg)))
#define DSPY_RD_W(reg)						(MMP_USHORT)(*(AIT_REG_W *)((MMP_ULONG)&(reg)))
#define DSPY_RD_D(reg)						(MMP_ULONG) (*(AIT_REG_D *)((MMP_ULONG)&(reg)))
#define DSPY_RD_OFST_B(reg, ofst)			(MMP_UBYTE) (*(AIT_REG_B *)((MMP_ULONG)&(reg) + ofst))
#define DSPY_RD_OFST_W(reg, ofst)			(MMP_USHORT)(*(AIT_REG_W *)((MMP_ULONG)&(reg) + ofst))
#define DSPY_RD_OFST_D(reg, ofst)			(MMP_ULONG) (*(AIT_REG_D *)((MMP_ULONG)&(reg) + ofst))

#define GBL_DECL							AITPS_GBL pGBL = AITC_BASE_GBL
#define GBL_WR_B(reg, val)					(*(AIT_REG_B *)((MMP_ULONG)&(reg))) = ((MMP_UBYTE)(val))
#define GBL_RD_B(reg)						(MMP_UBYTE) (*(AIT_REG_B *)((MMP_ULONG)&(reg)))
#define MEM_SET_W(addr, val)   				(*(AIT_REG_W *)addr) = ((MMP_USHORT)(val))
#define WIN_OPR_OFST(_w)					((_w - LCD_DISP_WIN_PIP) * 0x100)

#endif



//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

/* LCD Aspect Ratio  */
typedef enum _LCD_ASPECT_RATIO {
	LCD_RATIO_4_3 = 0,
	LCD_RATIO_16_9,
	LCD_RATIO_MAX
} LCD_ASPECT_RATIO;

/* LCD Rotate Direction  */
typedef enum _LCD_DIR {
	LCD_DIR_UPLEFT_DOWNRIGHT = 0,
	LCD_DIR_DOWNLEFT_UPRIGHT,
	LCD_DIR_UPRIGHT_DOWNLEFT,
	LCD_DIR_DOWNRIGHT_UPLEFT
} LCD_DIR;

/* LCD Interface Type */
typedef enum _LCD_TYPE {
	LCD_TYPE_PLCD,
	LCD_TYPE_SLCD,
	LCD_TYPE_PLCD_FLM,
	LCD_TYPE_RGBLCD,
	#if (CHIP == MCR_V2)
	LCD_TYPE_RGB2LCD,	
	#endif
	LCD_TYPE_MAX
} LCD_TYPE;

/* LCD Controller */
typedef enum _LCD_CONTROLER {
	LCD_PRM_CONTROLER,
	LCD_SCD_CONTROLER
} LCD_CONTROLER;

/* LCD Bus Width */
typedef enum _LCD_BUS_WIDTH {
	LCD_BUS_WIDTH_8,
	LCD_BUS_WIDTH_12,
	LCD_BUS_WIDTH_16,
	LCD_BUS_WIDTH_18
} LCD_BUS_WIDTH;

/* Idle Status */
typedef enum _LCD_POLARITY {
	LCD_POLARITY0,
	LCD_POLARITY1
} LCD_POLARITY;

/* Latch Data Edge */
typedef enum _LCD_PHASE {
	LCD_PHASE0,
	LCD_PHASE1
} LCD_PHASE;

/* MCU I/F system */
typedef enum _LCD_MCU_SYS {
	LCD_MCU_68SYS,
	LCD_MCU_80SYS
} LCD_MCU_SYS;

/* Output RGB Order */
typedef enum _LCD_RGB_ORDER {
	LCD_RGB_ORDER_RGB,
	LCD_RGB_ORDER_BGR
} LCD_RGB_ORDER;

/* Display Window */
typedef enum _LCD_DISP_WIN {
	#if (CHIP == P_V2)
	LCD_DISP_WIN_MAIN = 0,
	LCD_DISP_WIN_PIP,
	LCD_DISP_WIN_OVLY,
	LCD_DISP_WIN_ICON,
	LCD_DISP_WIN_SCD
	#endif
	#if (CHIP == MCR_V2)
	LCD_DISP_WIN_MAIN = 0,
	LCD_DISP_WIN_PIP  = 1,
	LCD_DISP_WIN_OSD  = 3,
	LCD_DISP_WIN_OVLY = 4
	#endif
} LCD_DISP_WIN;

/* Signal Polarity */
typedef enum _LCD_SIG_POLARITY {
	LCD_SIG_POLARITY_H,
	LCD_SIG_POLARITY_L
} LCD_SIG_POLARITY;

/* Signal Polarity */
typedef enum _LCD_SPI_PIX_ORDER {
	LCD_SPI_PIX_ORDER_RGB,
	LCD_SPI_PIX_ORDER_RBG,
	LCD_SPI_PIX_ORDER_GRB,
	LCD_SPI_PIX_ORDER_GBR,
	LCD_SPI_PIX_ORDER_BRG,
	LCD_SPI_PIX_ORDER_BGR
} LCD_SPI_PIX_ORDER;

/* Window color format */
typedef enum _LCD_WIN_FMT {
	LCD_WIN_FMT_4BPP,
	LCD_WIN_FMT_8BPP,
	LCD_WIN_FMT_16BPP,
	LCD_WIN_FMT_24BPP,
	LCD_WIN_FMT_YUV420,
	LCD_WIN_FMT_YUV422,
	LCD_WIN_FMT_32BPP
} LCD_WIN_FMT;

//==============================================================================
//
//                              STRUCTURE
//
//==============================================================================

typedef void LcdInitSeqFunc(void); 
typedef void LcdInitIdxCmdFunc(MMP_UBYTE); 

typedef struct _MMPF_PANEL_ATTR {
	// Panel basic setting
	MMP_USHORT			usPanelW;
	MMP_USHORT			usPanelH;
	LCD_TYPE			ubDevType;
	LCD_CONTROLER		ubController;
	MMP_ULONG			ulBgColor;
	LCD_ASPECT_RATIO	ubRatio;

	// Panel initial sequence
	LcdInitSeqFunc*		pInitSeq;

	// Index/Cmd sequence
	LcdInitIdxCmdFunc*	pIdxCmdSeq;
	
	// MCU interface
	LCD_BUS_WIDTH		ubBusWidth;
	LCD_PHASE			ubPhase;
	LCD_POLARITY		ubPolarity;
	LCD_MCU_SYS   		ubMCUSystem;
	MMP_USHORT			usRsLeadCsCnt;
	MMP_USHORT			usCsLeadRwCnt;
	MMP_USHORT			usRwCycCnt;	
	LCD_RGB_ORDER		ubRgbOrder;
	
	// RGB interface
	MMP_BOOL			bPartialDisp;
	LCD_SIG_POLARITY	ubDclkPor;          // CarDV
	LCD_SIG_POLARITY	ubHsyncPor;
	LCD_SIG_POLARITY	ubVsyncPor;
	MMP_UBYTE			ubRgbFmt;
	
	MMP_USHORT			usDotClkRatio;		// Stand for [(Lcd Freq / Dot Clock Freq) - 1]
	MMP_USHORT			usHBPorch;			// Unit:pixel, Ragne:0~1023
	MMP_USHORT			usHBlanking;		// Unit:pixel, Range:0~1023
	MMP_USHORT			usHSyncW;			// Unit:pixel, Range:0~255
	MMP_USHORT			usVBPorch;			// Unit:line,  Range:0~1023
	MMP_USHORT			usVBlanking;		// Unit:line,  Range:0~1023
	MMP_USHORT			usVSyncW;			// Unit:line,  Range:0~255
	MMP_USHORT			usV2HdotClk;		// P_V2 only
	MMP_USHORT			usPRT2HdotClk;		// Unit:Dot Clock, Range:0~255
	
	MMP_USHORT			usPartStX;
	MMP_USHORT			usPartEndX;
	MMP_USHORT			usPartStY;
	MMP_USHORT			usPartEndY;
	
	MMP_BOOL			bDeltaRBG;
	MMP_BOOL			bDummyRBG;
	LCD_SPI_PIX_ORDER   ubEvenLnOrder;
	LCD_SPI_PIX_ORDER   ubOddLnOrder;
	
	// RGB SPI interface
	MMP_USHORT			usSpi2MciRatio;		// Stand for [(MCI Freq / SPI Clock Freq) - 1]
	MMP_USHORT			usCsSetupCyc;		// Unit:MCI Clock, Range:0~15
	MMP_USHORT			usCsHoldCyc;		// Unit:MCI Clock, Range:0~15
	MMP_USHORT			usCsHighWidth;		// Unit:MCI Clock, Range:0~255
	MMP_USHORT			usSPIRegBitCnt;

	// Window setting (For drawing test pattern)
	LCD_DISP_WIN		ubDispWinId;
	MMP_USHORT			usWinStX;
	MMP_USHORT			usWinStY;
	MMP_USHORT			usWinW;
	MMP_USHORT			usWinH;
	MMP_USHORT			usWinBPP;
	LCD_WIN_FMT			usWinFmt;
	MMP_ULONG			ulWinAddr;

	//djkim.2015.02.27
	MMP_ULONG			ulWinAddrPhys;
	MMP_ULONG			ulWinAddrVirt;

} MMPF_PANEL_ATTR;

//==============================================================================
//
//                              FUNCTION PROTOTYPE
//
//==============================================================================

void MMPF_LCD_Write16BitCmd(MMPF_PANEL_ATTR* pAttr, MMP_USHORT usData);
MMP_ERR MMPF_LCD_InitPanel(MMPF_PANEL_ATTR* pAttr);
MMP_ERR MMPF_LCD_DrawTestPattern(MMPF_PANEL_ATTR* pAttr);
MMP_USHORT MMPF_LCD_GetBestRatioH(MMP_UBYTE ubRatio, MMP_USHORT usWidth);

MMPF_PANEL_ATTR* RTNA_LCD_GetAttr(void);
void RTNA_LCD_Init(void);


#endif //_LCD_COMMON_H_
