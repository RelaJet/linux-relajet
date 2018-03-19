#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/board.h>
#include <mach/lcd_common.h>
#include <mach/mmp_register.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_display.h>
#include <mach/mmpf_pio.h>
#include <mach/mmpf_system.h>

//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================

static MMP_ERR MMPF_LCD_InitSLCD(MMPF_PANEL_ATTR* pAttr)
{
	return MMP_ERR_NONE;
}

static MMP_ERR MMPF_LCD_InitPLCD(MMPF_PANEL_ATTR* pAttr)
{
	DSPY_DECL;
	MMP_USHORT usTemp = 0;
	
	/* Initial Bus width, Polarity, Phase, System Type */
	usTemp = LCD_PL1_PL2;
	
	switch(pAttr->ubBusWidth)
	{
		case LCD_BUS_WIDTH_8:
			usTemp |= PLCD_BUS_8BPP;
		break;
		case LCD_BUS_WIDTH_12:
			usTemp |= PLCD_BUS_12BPP;
		break;
		case LCD_BUS_WIDTH_16:
			usTemp |= PLCD_BUS_16BPP;
		break;
		case LCD_BUS_WIDTH_18:
			usTemp |= PLCD_BUS_18BPP;
		break;
	}
	
	if (pAttr->ubPhase == LCD_PHASE0)
		usTemp |= PLCD_PHA_0;
	else
		usTemp |= PLCD_PHA_1;

	if (pAttr->ubPolarity == LCD_POLARITY0)
		usTemp |= PLCD_POR_0;
	else
		usTemp |= PLCD_POR_1;

	if (pAttr->ubMCUSystem == LCD_MCU_68SYS)
		usTemp |= PLCD_TYPE_68;
	else
		usTemp |= PLCD_TYPE_80;
	
    DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_CTL, usTemp);
	
	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_4, LCD_OUT_RGB | LCD_OUT_SEL_LCD1 | LCD_BG_COLR_888);
	
	/* Initial RS/CS/RW signal timing information */
    DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_RS_LEAD_CS_CYC, pAttr->usRsLeadCsCnt);
    DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_CS_LEAD_RW_CYC, pAttr->usCsLeadRwCnt);
    DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_RW_CYC,  		pAttr->usRwCycCnt);

	return MMP_ERR_NONE;
}

static MMP_ERR MMPF_LCD_InitRGBLCD(MMPF_PANEL_ATTR* pAttr)
{
	DSPY_DECL;
	MMP_USHORT usTemp = 0;
	
	usTemp = DOTCLK_NORMAL_MODE;

	if (pAttr->ubHsyncPor == LCD_SIG_POLARITY_H)
		usTemp |= HSYNC_POLAR_HIGH;
	else
		usTemp |= HSYNC_POLAR_LOW;	

	if (pAttr->ubVsyncPor == LCD_SIG_POLARITY_H)
		usTemp |= VSYNC_POLAR_HIGH;
	else
		usTemp |= VSYNC_POLAR_LOW;	
	
	if (pAttr->bPartialDisp == MMP_TRUE) {
		usTemp |= PARTIAL_MODE_EN;
	
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ST_X, pAttr->usPartStX);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ED_X, pAttr->usPartEndX);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ST_Y, pAttr->usPartStY);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB_PART_ED_Y, pAttr->usPartEndY);
	}	
	else { 
		usTemp &= ~(PARTIAL_MODE_EN);
	}

	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_CTL,    usTemp);

    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_FMT,    pAttr->ubRgbFmt); //TBD
	
	/* Set RBG interface signal feature */
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_DOT_CLK_RATIO, 		pAttr->usDotClkRatio);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_PORCH_HIGH_BIT_EXT, 	0);//TBD
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_H_BPORCH,	 		pAttr->usHBPorch);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_H_BLANK, 			pAttr->usHBlanking);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_HSYNC_W, 			pAttr->usHSyncW);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_V_BPORCH, 			pAttr->usVBPorch);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_V_BLANK, 			pAttr->usVBlanking);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_VSYNC_W, 			pAttr->usVSyncW);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_V_2_H_DOT, 			pAttr->usV2HdotClk);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_PRT_2_H_DOT, 		pAttr->usPRT2HdotClk);

	/* Set RGB interface SPI feature */
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CTL, 			pAttr->usSPIRegBitCnt);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_RATIO_SPI_MCI, 		pAttr->usSpi2MciRatio);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CS_SETUP_CYCLE, 	pAttr->usCsSetupCyc);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CS_HOLD_CYCLE, 	pAttr->usCsHoldCyc);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CS_HIGH_WIDTH, 	pAttr->usCsHighWidth);
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CTRL_REGISTER1, 	RGB_SPI_DATA_ONLY_MODE);
    
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_PLCD_IDX_CMD_NUM, 		0x01); 

    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SHARE_P_LCD_BUS,   	RGB_LCD_ONLY | RGBLCD_SRC_SEL_RGB);
    
    /* Set RGB detla mode feature */
    usTemp = 0;
    
    if (pAttr->bDeltaRBG)
		usTemp |= RGB_DELTA_MODE_ENABLE;
	else
		usTemp &= ~(RGB_DELTA_MODE_ENABLE);	

    if (pAttr->bDummyRBG)
		usTemp |= RGB_DUMMY_MODE_ENABLE;
	else
		usTemp &= ~(RGB_DUMMY_MODE_ENABLE);
	
	switch(pAttr->ubEvenLnOrder)
	{
		case LCD_SPI_PIX_ORDER_RGB:
			usTemp |= SPI_EVEN_LINE_RGB;
		break;
		case LCD_SPI_PIX_ORDER_RBG:
			usTemp |= SPI_EVEN_LINE_RBG;
		break;
		case LCD_SPI_PIX_ORDER_GRB:
			usTemp |= SPI_EVEN_LINE_GRB;
		break;
		case LCD_SPI_PIX_ORDER_GBR:
			usTemp |= SPI_EVEN_LINE_GBR;
		break;
		case LCD_SPI_PIX_ORDER_BRG:
			usTemp |= SPI_EVEN_LINE_BRG;
		break;
		case LCD_SPI_PIX_ORDER_BGR:
			usTemp |= SPI_EVEN_LINE_BGR;
		break;
	}

	switch(pAttr->ubOddLnOrder)
	{
		case LCD_SPI_PIX_ORDER_RGB:
			usTemp |= SPI_ODD_LINE_RGB;
		break;
		case LCD_SPI_PIX_ORDER_RBG:
			usTemp |= SPI_ODD_LINE_RBG;
		break;
		case LCD_SPI_PIX_ORDER_GRB:
			usTemp |= SPI_ODD_LINE_GRB;
		break;
		case LCD_SPI_PIX_ORDER_GBR:
			usTemp |= SPI_ODD_LINE_GBR;
		break;
		case LCD_SPI_PIX_ORDER_BRG:
			usTemp |= SPI_ODD_LINE_BRG;
		break;
		case LCD_SPI_PIX_ORDER_BGR:
			usTemp |= SPI_ODD_LINE_BGR;
		break;
	}
		    
    DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_DELTA_MODE, usTemp);
	
	return MMP_ERR_NONE;
}

#if (CHIP == MCR_V2)
static MMP_ERR MMPF_LCD_InitRGB2LCD(MMPF_PANEL_ATTR* pAttr)
{
	DSPY_DECL;
	MMP_USHORT usTemp = 0;

	if (pAttr->ubHsyncPor == LCD_SIG_POLARITY_H)
		usTemp |= HSYNC_POLAR_HIGH;
	else
		usTemp |= HSYNC_POLAR_LOW;	

	if (pAttr->ubVsyncPor == LCD_SIG_POLARITY_H)
		usTemp |= VSYNC_POLAR_HIGH;
	else
		usTemp |= VSYNC_POLAR_LOW;	
	
	if (pAttr->bPartialDisp == MMP_TRUE) {
		usTemp |= PARTIAL_MODE_EN;
	
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ST_X, pAttr->usPartStX);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ED_X, pAttr->usPartEndX);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ST_Y, pAttr->usPartStY);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_RGB2_PART_ED_Y, pAttr->usPartEndY);
	}	
	else { 
		usTemp &= ~(PARTIAL_MODE_EN);	
	}	
	
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_CTL,	usTemp);

	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_FMT,	RGB_D24BIT_RGB888); //TBD
	
	/* Set RBG interface signal feature */
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_DOT_CLK_RATIO,		pAttr->usDotClkRatio);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_PORCH_HIGH_BIT_EXT, 0);//TBD
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_H_BPORCH,			pAttr->usHBPorch);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_H_BLANK,			pAttr->usHBlanking);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_HSYNC_W,			pAttr->usHSyncW);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_V_BPORCH,			pAttr->usVBPorch);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_V_BLANK,			pAttr->usVBlanking);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_VSYNC_W,			pAttr->usVSyncW);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_PRT_2_H_DOT,		pAttr->usPRT2HdotClk);

	/* Set RBG interface SPI feature */
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CTL, 			SPI_PANEL_16BITS);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_RATIO_SPI_MCI,		pAttr->usSpi2MciRatio);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CS_SETUP_CYCLE,	pAttr->usCsSetupCyc);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CS_HOLD_CYCLE,	pAttr->usCsHoldCyc);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CS_HIGH_WIDTH,	pAttr->usCsHighWidth);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SPI_CTRL_REGISTER1,	RGB_SPI_DATA_ONLY_MODE);
	
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_PLCD_IDX_CMD_NUM,		0x01); 
	
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_SHARE_P_LCD_BUS, 	RGB_LCD_ONLY | RGBLCD_SRC_SEL_RGB2);
	
	/* Set RGB detla mode feature */
	usTemp = 0;
	
	if (pAttr->bDeltaRBG)
		usTemp |= RGB_DELTA_MODE_ENABLE;
	else
		usTemp &= ~(RGB_DELTA_MODE_ENABLE); 
		
	if (pAttr->bDummyRBG)
		usTemp |= RGB_DUMMY_MODE_ENABLE;
	else
		usTemp &= ~(RGB_DUMMY_MODE_ENABLE);
	
	switch(pAttr->ubEvenLnOrder)
	{
		case LCD_SPI_PIX_ORDER_RGB:
			usTemp |= SPI_EVEN_LINE_RGB;
		break;
		case LCD_SPI_PIX_ORDER_RBG:
			usTemp |= SPI_EVEN_LINE_RBG;
		break;
		case LCD_SPI_PIX_ORDER_GRB:
			usTemp |= SPI_EVEN_LINE_GRB;
		break;
		case LCD_SPI_PIX_ORDER_GBR:
			usTemp |= SPI_EVEN_LINE_GBR;
		break;
		case LCD_SPI_PIX_ORDER_BRG:
			usTemp |= SPI_EVEN_LINE_BRG;
		break;
		case LCD_SPI_PIX_ORDER_BGR:
			usTemp |= SPI_EVEN_LINE_BGR;
		break;
	}

	switch(pAttr->ubOddLnOrder)
	{
		case LCD_SPI_PIX_ORDER_RGB:
			usTemp |= SPI_ODD_LINE_RGB;
		break;
		case LCD_SPI_PIX_ORDER_RBG:
			usTemp |= SPI_ODD_LINE_RBG;
		break;
		case LCD_SPI_PIX_ORDER_GRB:
			usTemp |= SPI_ODD_LINE_GRB;
		break;
		case LCD_SPI_PIX_ORDER_GBR:
			usTemp |= SPI_ODD_LINE_GBR;
		break;
		case LCD_SPI_PIX_ORDER_BRG:
			usTemp |= SPI_ODD_LINE_BRG;
		break;
		case LCD_SPI_PIX_ORDER_BGR:
			usTemp |= SPI_ODD_LINE_BGR;
		break;
	}
			
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_DELTA_MODE, usTemp);

	return MMP_ERR_NONE;
}
#endif

void MMPF_LCD_Write16BitCmd(MMPF_PANEL_ATTR* pAttr, MMP_USHORT usData)
{
	DSPY_DECL;
	
    if (pAttr->ubController == LCD_SCD_CONTROLER)
    {
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_LCD_TX_0, usData);
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_SCD_CTL, LCD_IDX_RDY);
		while(DSPY_RD_B(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_IDX_RDY);
		
	}
    else
    {
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_LCD_TX_0, usData);
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_CTL_0, LCD_IDX_RDY);
		while(DSPY_RD_B(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_IDX_RDY);
	}
}

MMP_ERR MMPF_LCD_InitPanel(MMPF_PANEL_ATTR* pAttr)
{
	DSPY_DECL;
	MMP_USHORT usTemp = 0;
	
	/* Enable Display Clock */
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_DSPY, MMP_TRUE);
	
	/* Disable LCD Tx */
	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, 0);

	/* Disable RGB interface */
	if (pAttr->ubDevType == LCD_TYPE_RGBLCD) {
	
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_FMT, DSPY_RGB_SYNC_MODE_EN);
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) & ~(RGB_IF_EN));
		while(DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) & RGB_IF_EN);
	}
#if (CHIP == MCR_V2)
	else if (pAttr->ubDevType == LCD_TYPE_RGB2LCD) {
	 
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_FMT, DSPY_RGB_SYNC_MODE_EN);
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) & ~(RGB_IF_EN));
		while(DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) & RGB_IF_EN);
	}
#endif

	/* Initial Controller and Panel Type */
	if (pAttr->ubController == LCD_PRM_CONTROLER) {
	
		usTemp = PRM_DSPY_REG_READY | DSPY_PRM_EN;
		
		switch(pAttr->ubDevType)
		{
			case LCD_TYPE_PLCD:
				usTemp |= DSPY_PRM_SEL(DSPY_TYPE_PL_LCD);
			break;
			case LCD_TYPE_SLCD:
				usTemp |= DSPY_PRM_SEL(DSPY_TYPE_SPI_LCD);
			break;
			case LCD_TYPE_PLCD_FLM:
				usTemp |= DSPY_PRM_SEL(DSPY_TYPE_PL_LCD);
			break;
			case LCD_TYPE_RGBLCD:
				usTemp |= DSPY_PRM_SEL(DSPY_TYPE_RGB_LCD);
			break;
		#if (CHIP == MCR_V2)
			case LCD_TYPE_RGB2LCD:
				usTemp |= DSPY_PRM_SEL(DSPY_TYPE_PRM_RGB2);
			break;
		#endif
		}
	}
	else {
		
		/* Disable 2nd LCD controller */
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, 0);
	
		usTemp = DSPY_SCD_EN;
		
		switch(pAttr->ubDevType)
		{
			case LCD_TYPE_PLCD:
				usTemp |= DSPY_SCD_SEL(DSPY_TYPE_PL_LCD);
			break;
			case LCD_TYPE_SLCD:
				usTemp |= DSPY_SCD_SEL(DSPY_TYPE_SPI_LCD);
			break;
			case LCD_TYPE_PLCD_FLM:
				usTemp |= DSPY_SCD_SEL(DSPY_TYPE_PL_LCD);
			break;
			case LCD_TYPE_RGBLCD:
				usTemp |= DSPY_SCD_SEL(DSPY_TYPE_RGB_LCD);
			break;
		#if (CHIP == MCR_V2)
			case LCD_TYPE_RGB2LCD:
				usTemp |= DSPY_SCD_SEL(DSPY_TYPE_SCD_RGB2);
			break;
		#endif
		}

		DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, SCD_DSPY_REG_READY); 
	}
	
	DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_2, usTemp);

	
	if (pAttr->ubDevType == LCD_TYPE_PLCD ||
		pAttr->ubDevType == LCD_TYPE_PLCD_FLM)
	{
		MMPF_LCD_InitPLCD(pAttr);
	}
	else if (pAttr->ubDevType == LCD_TYPE_RGBLCD)
	{
		MMPF_LCD_InitRGBLCD(pAttr);
	}
#if (CHIP == MCR_V2)
	else if (pAttr->ubDevType == LCD_TYPE_RGB2LCD)
	{
		MMPF_LCD_InitRGB2LCD(pAttr);
	}
#endif
	else if (pAttr->ubDevType == LCD_TYPE_SLCD)
	{
		MMPF_LCD_InitSLCD(pAttr);
	}
	
	/* Initial Panel Width/Height/Pixel Count */
	if (pAttr->ubController == LCD_PRM_CONTROLER) {
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_W,				pAttr->usPanelW);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_H,				pAttr->usPanelH);
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_PIXL_CNT,		pAttr->usPanelW * pAttr->usPanelH);
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_BG_COLOR,		pAttr->ulBgColor);
	}
	else {
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_W,			pAttr->usPanelW);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_H,			pAttr->usPanelH);
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_PIXL_CNT,	pAttr->usPanelW * pAttr->usPanelH); 
		DSPY_WR_D(AITC_BASE_DSPY->DSPY_SCD_BG_COLOR,	pAttr->ulBgColor);
		//DSPY_WR_W(DSPY_SCD_CTL,		SCD_SRC_RGB888);
	}
	
	/* Window Scaling Setting */
	if (pAttr->ubDispWinId == LCD_DISP_WIN_PIP || 
		pAttr->ubDispWinId == LCD_DISP_WIN_OVLY)
	{
		MMP_SHORT offset = WIN_OPR_OFST(pAttr->ubDispWinId);

    #if (CHIP == P_V2)
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_BUF_STOP_THD, 16);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL,	 DSPY_SCAL_BYPASS);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL,	 DSPY_SOUT_RGB_EN);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_PIP_SEDGE_CTL,	 DSPY_SEDGE_BYPASS);
    #endif
    #if (CHIP == MCR_V2)
		// Work around for 256 byte alignment
    #if (VIDEO_DEC_4K2K_WORKAROUND == 0) 
		// HDMI performance become worse becasue set this. MP version should be OK. And if no display happens.
		// Be careful about the alignment issue.
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y_FIFO_CTL, offset, 0x1170);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_U_FIFO_CTL, offset, 0x1170);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_V_FIFO_CTL, offset, 0x1170);
    #endif
		// Bypass work around 
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_WT, offset,	0);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_WT, offset,	0);
		DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_N,  offset,	1);
		DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_SCAL_H_M,  offset,	1);
		DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_N,  offset,	1);
		DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_SCAL_V_M,  offset,	1);
		
		DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_BUF_FULL_THD, offset, 64);
		DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_BUF_STOP_THD, offset, 32);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCAL_CTL,	  offset, DSPY_SCAL_NM);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_CTL,	  offset, DSPY_SOUT_RGB_EN);
		DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SEDGE_CTL,	  offset, DSPY_SEDGE_BYPASS);

		// for MercuryV2 MP
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_WIN_BIND_SEL_MP, DSPY_RD_B(AITC_BASE_DSPY->DSPY_WIN_BIND_SEL_MP) &
													~(BIND_SEL_MASK << PIP_WIN_SHIFT));
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_WIN_BIND_SEL_MP, DSPY_RD_B(AITC_BASE_DSPY->DSPY_WIN_BIND_SEL_MP) |
													(PRM_SEL << PIP_WIN_SHIFT));
	#endif
	}
#if (CHIP == MCR_V2)
	else if (pAttr->ubController == LCD_SCD_CONTROLER &&
			 pAttr->ubDispWinId  == LCD_DISP_WIN_OSD) {
		// Work around for 256 byte alignment
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_Y_FIFO_CTL,	0x1170);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_U_FIFO_CTL,	0x1170);
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_V_FIFO_CTL,	0x1170);
	}
#endif
	
	/* Initial panel sequence */
	if (pAttr->ubDevType == LCD_TYPE_PLCD ||
		pAttr->ubDevType == LCD_TYPE_PLCD_FLM)
	{
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_CTL, (DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) & ~PLCD_CMD_BURST) );
		
		if (pAttr->pInitSeq != NULL) {
			pAttr->pInitSeq();
		}
		
		DSPY_WR_W(AITC_BASE_DSPY->DSPY_PLCD_CTL, (DSPY_RD_W(AITC_BASE_DSPY->DSPY_PLCD_CTL) & ~PLCD_CMD_BURST) );
	}
	
	/* Initial set index/command */
	if (pAttr->pIdxCmdSeq != NULL) {
		pAttr->pIdxCmdSeq(pAttr->ubController);
	}
	
	if (pAttr->ubDevType == LCD_TYPE_PLCD ||
		pAttr->ubDevType == LCD_TYPE_PLCD_FLM)
	{
		/* Enable auto transfer set address */
		if (pAttr->ubController == LCD_PRM_CONTROLER) {
			DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0,	LCD_FRAME_TX_SETADDR_EN);
		}
		else {
			DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0,	SCD_DSPY_REG_READY); 
			DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, LCD_FRAME_TX_SETADDR_EN);	 
		}
	}
	else if (pAttr->ubDevType == LCD_TYPE_RGBLCD)
	{
		/* Enable RGB interface */
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) | RGB_IF_EN);
	}
#if (CHIP == MCR_V2)
	else if (pAttr->ubDevType == LCD_TYPE_RGB2LCD)
	{
		/* Enable RGB interface */
		DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB2_CTL, DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) | RGB_IF_EN);
	}
#endif

	return MMP_ERR_NONE;
}

MMP_ERR MMPF_LCD_DrawTestPattern(MMPF_PANEL_ATTR* pAttr)
{
    int     x, y;
    static MMP_USHORT  lcd_pat[] = {0xF800, 0x07E0, 0x001F, 0xFFFF};
    DSPY_DECL;

#if 0
    for (x = 0; x < pAttr->usWinBPP*(pAttr->usWinW*pAttr->usWinH)/4; x++) {
        MEM_SET_W((pAttr->ulWinAddr) + x, lcd_pat[0]);
    }
    for (x = 0; x < pAttr->usWinBPP*(pAttr->usWinW*pAttr->usWinH)/4; x++) {
        MEM_SET_W((pAttr->ulWinAddr + pAttr->usWinBPP*(pAttr->usWinW*pAttr->usWinH)/4 + x), lcd_pat[1]);
    }
    for (x = 0; x < pAttr->usWinBPP*(pAttr->usWinW*pAttr->usWinH)/4; x++) {
        MEM_SET_W((pAttr->ulWinAddr + pAttr->usWinBPP*(pAttr->usWinW*pAttr->usWinH)/2 + x), lcd_pat[2]);
    }
    for (x = 0; x < pAttr->usWinBPP*(pAttr->usWinW*pAttr->usWinH)/4; x++) {
        MEM_SET_W((pAttr->ulWinAddr + pAttr->usWinBPP*(pAttr->usWinW*pAttr->usWinH)*3/4 + x), lcd_pat[3]);    
    }
#else
	/* Draw 4 color blocks */
	for (y = 0; y < pAttr->usWinH/2; y++) {
    	// Red block
        for (x = 0; x < pAttr->usWinW/2; x++) {
            MEM_SET_W((pAttr->ulWinAddrVirt+ pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[0]);
        }
        // Green block
        for (x = pAttr->usWinW/2; x < pAttr->usWinW; x++) {
            MEM_SET_W((pAttr->ulWinAddrVirt + pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[1]);
        }
    }
    for (y = pAttr->usWinH/2; y < pAttr->usWinH; y++) {
    	// Blue block
        for (x = 0; x < pAttr->usWinW/2; x++) {
            MEM_SET_W((pAttr->ulWinAddrVirt + pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[2]);
        }
        // White block
        for (x = pAttr->usWinW/2; x < pAttr->usWinW; x++) {
            MEM_SET_W((pAttr->ulWinAddrVirt + pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[3]);
        }
    }
#endif

	if (pAttr->ubController == LCD_SCD_CONTROLER) 
	{
		#if (CHIP == MCR_V2)
        // for MercuryV2 MP
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_OSD_TP.MP.R_COLOR, 0);
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_OSD_TP.MP.G_COLOR, 0);
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_OSD_TP.MP.B_COLOR, 0);

	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT,        WIN_16BPP);
	    
	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_ADDR_ST,    pAttr->ulWinAddrPhys);
	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_OFST_ST,    0);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_OFST_PIXL,  pAttr->usWinBPP);                      
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_OFST_ROW,   pAttr->usWinW * pAttr->usWinBPP);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_W,          pAttr->usWinW);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_H,          pAttr->usWinH);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_X,          0);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_Y,          0);
	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_PIXL_CNT,   pAttr->usWinW * pAttr->usWinH);

	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_CTL,        WIN_EN);
	    #endif
	    
		if (pAttr->ubDevType == LCD_TYPE_PLCD ||
			pAttr->ubDevType == LCD_TYPE_PLCD_FLM)
		{
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0,   DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | SCD_DSPY_REG_READY); 
			DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_FRAME_TX);
			while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_FRAME_TX);		
		}
	}
	else
	{
		MMP_SHORT offset = WIN_OPR_OFST(pAttr->ubDispWinId);
	    
	    DSPY_WR_B(AITC_BASE_DSPY->DSPY_WIN_PRIO,        (ICON_WIN  << WIN_1_SHFT) |
	                                    (OVLY_WIN  << WIN_2_SHFT) |
	                                    (PIP_WIN   << WIN_3_SHFT) |
	                                    (MAIN_WIN  << WIN_4_SHFT) );

        // for MercuryV2 MP
        DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_TP.MP.R_COLOR, offset, 0x00);
        DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_TP.MP.G_COLOR, offset, 0x00);
        DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_TP.MP.B_COLOR, offset, 0x00);

	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT,       offset, WIN_16BPP);
	    
	    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_ADDR_ST,   offset, pAttr->ulWinAddrPhys);
	    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_OFST_ST,   offset, 0);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, pAttr->usWinBPP);                       
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW,  offset, pAttr->usWinBPP * pAttr->usWinW);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W,         offset, pAttr->usWinW);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H,         offset, pAttr->usWinH);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X,         offset, 0);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y,         offset, 0);
	    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_PIXL_CNT,  offset, pAttr->usWinW * pAttr->usWinH);

	    /* LCD Scaling Setting */
	    if (pAttr->ubDispWinId == LCD_DISP_WIN_PIP || 
      		pAttr->ubDispWinId == LCD_DISP_WIN_OVLY)
      	{
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_W,       offset, pAttr->usWinW);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_H,       offset, pAttr->usWinH);
		    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_SOUT_GRAB_PIXL_CNT, offset, pAttr->usWinW * pAttr->usWinH);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ST, offset, 1);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ED, offset, pAttr->usWinW);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ST, offset, 1);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset, pAttr->usWinH);
		}
		
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, WIN_EN);
	    
		if (pAttr->ubDevType == LCD_TYPE_PLCD ||
			pAttr->ubDevType == LCD_TYPE_PLCD_FLM)
		{
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_FRAME_TX);
			while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_FRAME_TX);
		}
    }

	return MMP_ERR_NONE;
}

MMP_USHORT MMPF_LCD_GetBestRatioH(MMP_UBYTE ubRatio, MMP_USHORT usWidth)
{
	if (ubRatio == LCD_RATIO_4_3)
		return usWidth * 3 / 4;
	else if (ubRatio == LCD_RATIO_16_9)
		return usWidth * 9 / 16;
	else
		return usWidth;
}

MMP_ERR MMPF_LCD_DrawTestPattern_byFB(MMPF_PANEL_ATTR* pAttr, unsigned int phys, unsigned int virt)
{
    int     x, y;
    static MMP_USHORT  lcd_pat[] = {0xF800, 0x07E0, 0x001F, 0xFFFF};
    DSPY_DECL;

#if 1
	/* Draw 4 color blocks */
	for (y = 0; y < pAttr->usWinH/2; y++) {
    	// Red block
        for (x = 0; x < pAttr->usWinW/2; x++) {
            MEM_SET_W((virt + pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[0]);
        }
        // Green block
        for (x = pAttr->usWinW/2; x < pAttr->usWinW; x++) {
            MEM_SET_W((virt + pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[1]);
        }
    }
    for (y = pAttr->usWinH/2; y < pAttr->usWinH; y++) {
    	// Blue block
        for (x = 0; x < pAttr->usWinW/2; x++) {
            MEM_SET_W((virt + pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[2]);
        }
        // White block
        for (x = pAttr->usWinW/2; x < pAttr->usWinW; x++) {
            MEM_SET_W((virt + pAttr->usWinBPP*(pAttr->usWinW*y+x)), lcd_pat[3]);
        }
    }
#else
	memset(virt, 0x55, 153600);
#endif

	if (pAttr->ubController == LCD_SCD_CONTROLER) 
	{
		#if (CHIP == MCR_V2)
        // for MercuryV2 MP
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_OSD_TP.MP.R_COLOR, 0);
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_OSD_TP.MP.G_COLOR, 0);
        DSPY_WR_B(AITC_BASE_DSPY->DSPY_OSD_TP.MP.B_COLOR, 0);

	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_FMT,        WIN_16BPP);
	    
	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_ADDR_ST,    phys);
	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_OFST_ST,    0);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_OFST_PIXL,  pAttr->usWinBPP);                      
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_OFST_ROW,   pAttr->usWinW * pAttr->usWinBPP);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_W,          pAttr->usWinW);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_H,          pAttr->usWinH);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_X,          0);
	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_Y,          0);
	    DSPY_WR_D(AITC_BASE_DSPY->DSPY_OSD_PIXL_CNT,   pAttr->usWinW * pAttr->usWinH);

	    DSPY_WR_W(AITC_BASE_DSPY->DSPY_OSD_CTL,        WIN_EN);
	    #endif
	    
		if (pAttr->ubDevType == LCD_TYPE_PLCD ||
			pAttr->ubDevType == LCD_TYPE_PLCD_FLM)
		{
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0,   DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | SCD_DSPY_REG_READY); 
			DSPY_WR_W(AITC_BASE_DSPY->DSPY_SCD_CTL, DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) | LCD_FRAME_TX);
			while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_SCD_CTL) & LCD_FRAME_TX);		
		}
	}
	else
	{
		MMP_SHORT offset = WIN_OPR_OFST(pAttr->ubDispWinId);
	    
	    DSPY_WR_B(AITC_BASE_DSPY->DSPY_WIN_PRIO,        (ICON_WIN  << WIN_1_SHFT) |
	                                    (OVLY_WIN  << WIN_2_SHFT) |
	                                    (PIP_WIN   << WIN_3_SHFT) |
	                                    (MAIN_WIN  << WIN_4_SHFT) );

        // for MercuryV2 MP
        DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_TP.MP.R_COLOR, offset, 0x00);
        DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_TP.MP.G_COLOR, offset, 0x00);
        DSPY_WR_OFST_B(AITC_BASE_DSPY->DSPY_PIP_TP.MP.B_COLOR, offset, 0x00);

	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_FMT,       offset, WIN_16BPP);
	    
	    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_ADDR_ST,   offset, phys);
	    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_OFST_ST,   offset, 0);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_PIXL, offset, pAttr->usWinBPP);                       
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_OFST_ROW,  offset, pAttr->usWinBPP * pAttr->usWinW);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_W,         offset, pAttr->usWinW);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_H,         offset, pAttr->usWinH);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_X,         offset, 0);
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_Y,         offset, 0);
	    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_PIP_PIXL_CNT,  offset, pAttr->usWinW * pAttr->usWinH);

	    /* LCD Scaling Setting */
	    if (pAttr->ubDispWinId == LCD_DISP_WIN_PIP || 
      		pAttr->ubDispWinId == LCD_DISP_WIN_OVLY)
      	{
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_W,       offset, pAttr->usWinW);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SCA_IN_H,       offset, pAttr->usWinH);
		    DSPY_WR_OFST_D(AITC_BASE_DSPY->DSPY_SOUT_GRAB_PIXL_CNT, offset, pAttr->usWinW * pAttr->usWinH);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ST, offset, 1);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_H_ED, offset, pAttr->usWinW);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ST, offset, 1);
		    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_SOUT_GRAB_V_ED, offset, pAttr->usWinH);
		}
		
	    DSPY_WR_OFST_W(AITC_BASE_DSPY->DSPY_PIP_CTL, offset, WIN_EN);
	    
		if (pAttr->ubDevType == LCD_TYPE_PLCD ||
			pAttr->ubDevType == LCD_TYPE_PLCD_FLM)
		{
		    DSPY_WR_W(AITC_BASE_DSPY->DSPY_CTL_0, DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) | LCD_FRAME_TX);
			while (DSPY_RD_W(AITC_BASE_DSPY->DSPY_CTL_0) & LCD_FRAME_TX);
		}
    }
	
	return MMP_ERR_NONE;
}

