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
#include <mach/mmp_reg_gbl.h>
#include <mach/mmpf_pio.h>
#include <mach/mmpf_system.h>

#define LCD_GPIO_BACK_LIGHT             (MMPF_PIO_REG_GPIO5)    // AGPIO3 -->> LCD_BL //djkim
#define LCD_GPIO_BACK_LIGHT_ACT_LEVEL 	(PIO_HIGH)
#define LCD_GPIO_RESET_ACT_LEVEL 		(PIO_LOW)
#define LCD_GPIO_RESET                  (MMPF_PIO_REG_GPIO63)  // PCGPIO31 //djkim


// Temp. define
#define LCD_IF_NONE 					(0)
#define LCD_IF_SERIAL 					(1)
#define LCD_IF_PARALLEL 				(2)
#define LCD_IF_RGB 						(3)
#define LCD_IF 							(LCD_IF_RGB)

#define LCD_MODEL_TYPE 					(ILI8961) // TBD
// Set up LCD display Width/Height ratio. The default is 4/3 if No these definition.
#define LCD_MODEL_RATIO_X               (4)
#define LCD_MODEL_RATIO_Y               (3)

#define LCD_PANEL_WIDTH					(320)
#define LCD_PANEL_HEIGHT				(240)

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define Delayms(t) 					LCD_WAIT(t)

#define	DELTA_RGB 					(0)
#define	STRIP_RGB 					(1) // TBD??
#define	LCD_RGB_MODE 				(DELTA_RGB)

#define SECONDARY_DISPLAY 			(0)

static MMPF_PANEL_ATTR m_PanelAttr = 
{
	// Panel basic setting
	LCD_PANEL_WIDTH, 
	LCD_PANEL_HEIGHT,
	LCD_TYPE_RGBLCD,
	LCD_PRM_CONTROLER,
	0,

	// CarDV
#if ((16 == LCD_MODEL_RATIO_X) && (9 == LCD_MODEL_RATIO_Y))
	LCD_RATIO_16_9,
#else
	LCD_RATIO_4_3,
#endif

	// Panel initial sequence
	NULL,

	// Index/Cmd sequence
	NULL,

	// MCU interface
	LCD_BUS_WIDTH_8,
	LCD_PHASE0,
	LCD_POLARITY0,
	LCD_MCU_80SYS,
	0,
	0,
	0,
	LCD_RGB_ORDER_RGB,

	// RGB interface
	MMP_FALSE,
    LCD_SIG_POLARITY_L,     // DCLK Polarity
	LCD_SIG_POLARITY_L,     // H-Sync Polarity
	LCD_SIG_POLARITY_L,     // V-Sync Polarity
	RGB_D24BIT_RGB888,

	{0}
};

#define ENABLE_LCD_ILI8961_LOG 		(0)
#if defined(FAT_BOOT)
#define ENABLE_LCD_TEST_PATTERN 	(1)
#else
#define ENABLE_LCD_TEST_PATTERN 	(0)
#endif
#define LCD_TEST_ADDR				(0x04900000) //djkim.2015.02.27

#define LCD_BRIGETNESS_DEFAULT 		(1)
#define LCD_CONTRAST_DEFAULT 		(1)
#define LCD_VCOM_DEFAULT 			(0)
#define LCD_GAMMA_AUTO 				(1) // 0: Manual set gamma by R17h~R1Ah, 1: Auto set to gamma2.2


//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================

void RTNA_LCD_InitMainLCD(void)
{
#if (SECONDARY_DISPLAY == 0)
	RTNA_DBG_Str(0, "### RTNA_LCD_InitMainLCD -\r\n");

	m_PanelAttr.usPanelW 		= LCD_PANEL_WIDTH;
	m_PanelAttr.usPanelH 		= LCD_PANEL_HEIGHT;
	m_PanelAttr.ubDevType 		= LCD_TYPE_RGBLCD;
	m_PanelAttr.ubController 	= LCD_PRM_CONTROLER;
	m_PanelAttr.ulBgColor 		= 0;

	m_PanelAttr.pInitSeq 		= NULL;
	m_PanelAttr.pIdxCmdSeq 		= NULL;

	m_PanelAttr.bPartialDisp 	= MMP_FALSE;
	m_PanelAttr.ubDclkPor 		= LCD_SIG_POLARITY_H;
	m_PanelAttr.ubHsyncPor 		= LCD_SIG_POLARITY_L;
	m_PanelAttr.ubVsyncPor 		= LCD_SIG_POLARITY_L;
	m_PanelAttr.ubRgbFmt 		= RGB_D24BIT_RGB888;

	m_PanelAttr.usDotClkRatio 	= 8;

	#if (LCD_RGB_MODE == STRIP_RGB)
	m_PanelAttr.usHBPorch 		= 0x3C;
	m_PanelAttr.usHBlanking 	= 0x47;
	#else
	m_PanelAttr.usHBPorch 		= 24;
	m_PanelAttr.usHBlanking 	= 0x46;
	#endif
	m_PanelAttr.usHSyncW 		= 0;
	m_PanelAttr.usVBPorch 		= 21;
	m_PanelAttr.usVBlanking 	= 22;
	m_PanelAttr.usVSyncW 		= 0;
	m_PanelAttr.usV2HdotClk 	= 1;
	m_PanelAttr.usPRT2HdotClk 	= 2;

	#if (LCD_RGB_MODE == STRIP_RGB)
	m_PanelAttr.bDeltaRBG 		= MMP_FALSE; // TBD??
	m_PanelAttr.bDummyRBG 		= MMP_TRUE; // TBD??
	#else
	m_PanelAttr.bDeltaRBG 		= MMP_TRUE;
	m_PanelAttr.bDummyRBG 		= MMP_FALSE;
	#endif

	// CarDV
	if ((0 == m_PanelAttr.ubEvenLnOrder) || (0 == m_PanelAttr.ubOddLnOrder))
	{
		// The order maybe changed by RTNA_LCD_Direction
		#if (LCD_RGB_MODE == STRIP_RGB)
		m_PanelAttr.ubEvenLnOrder 	= LCD_SPI_PIX_ORDER_RGB;
		m_PanelAttr.ubOddLnOrder 	= LCD_SPI_PIX_ORDER_RGB;
		#else
		m_PanelAttr.ubEvenLnOrder 	= LCD_SPI_PIX_ORDER_GBR;
		m_PanelAttr.ubOddLnOrder 	= LCD_SPI_PIX_ORDER_RGB;
		#endif

		RTNA_DBG_Str(0, "Change PIX order to default\r\n");
	}

	m_PanelAttr.usSpi2MciRatio 	= 0x80;
	m_PanelAttr.usCsSetupCyc 	= 0x0F;
	m_PanelAttr.usCsHoldCyc 	= 0x0F;
	m_PanelAttr.usCsHighWidth 	= 0xFF;
	m_PanelAttr.usSPIRegBitCnt  = SPI_PANEL_16BITS;

	m_PanelAttr.ubDispWinId 	= LCD_DISP_WIN_PIP;
	m_PanelAttr.usWinStX 		= 0;
	m_PanelAttr.usWinStY 		= 0;
	m_PanelAttr.usWinW 			= LCD_PANEL_WIDTH;
	m_PanelAttr.usWinH 			= LCD_PANEL_HEIGHT;
	m_PanelAttr.usWinBPP 		= 2;
	m_PanelAttr.usWinFmt 		= LCD_WIN_FMT_16BPP;
	//m_PanelAttr.ulWinAddr 		= 0x01500000;
	m_PanelAttr.ulWinAddr 		= LCD_TEST_ADDR;

	// CarDV
	#if ((16 == LCD_MODEL_RATIO_X) && (9 == LCD_MODEL_RATIO_Y))
	m_PanelAttr.ubRatio 		= LCD_RATIO_16_9;
	#else
	m_PanelAttr.ubRatio 		= LCD_RATIO_4_3;
	#endif
	
	MMPF_LCD_InitPanel(&m_PanelAttr); //djkim
#endif
}

void RTNA_LCD_Init2ndLCD(void)
{
#if (SECONDARY_DISPLAY == 1)
	m_PanelAttr.usPanelW 		= LCD_PANEL_WIDTH;
	m_PanelAttr.usPanelH 		= LCD_PANEL_HEIGHT;
	m_PanelAttr.ubDevType 		= LCD_TYPE_RGBLCD;
	m_PanelAttr.ubController 	= LCD_SCD_CONTROLER;

	m_PanelAttr.pInitSeq 		= NULL;
	m_PanelAttr.pIdxCmdSeq 		= NULL;

	m_PanelAttr.bPartialDisp 	= MMP_FALSE;
	m_PanelAttr.ubHsyncPor 		= LCD_SIG_POLARITY_L;
	m_PanelAttr.ubVsyncPor 		= LCD_SIG_POLARITY_L;
	m_PanelAttr.ubRgbFmt 		= RGB_D24BIT_RGB888;

	m_PanelAttr.usDotClkRatio 	= 7;
	m_PanelAttr.usHBPorch 		= 0x3F;
	m_PanelAttr.usHBlanking 	= 0x46;
	m_PanelAttr.usHSyncW 		= 0;
	m_PanelAttr.usVBPorch 		= 13;
	m_PanelAttr.usVBlanking 	= 22;
	m_PanelAttr.usVSyncW 		= 0;
	m_PanelAttr.usV2HdotClk 	= 1;
	m_PanelAttr.usPRT2HdotClk 	= 2;

	m_PanelAttr.bDeltaRBG 		= MMP_TRUE;
	m_PanelAttr.bDummyRBG 		= MMP_TRUE;
	m_PanelAttr.ubEvenLnOrder 	= LCD_SPI_PIX_ORDER_RGB;
	m_PanelAttr.ubOddLnOrder 	= LCD_SPI_PIX_ORDER_RGB;

	m_PanelAttr.usSpi2MciRatio 	= 0x80;
	m_PanelAttr.usCsSetupCyc 	= 0x0F;
	m_PanelAttr.usCsHoldCyc 	= 0x0F;
	m_PanelAttr.usCsHighWidth 	= 0xFF;

	m_PanelAttr.ubDispWinId 	= LCD_DISP_WIN_OSD;
	m_PanelAttr.usWinStX 		= 0;
	m_PanelAttr.usWinStY 		= 0;
	m_PanelAttr.usWinW 			= LCD_PANEL_WIDTH;
	m_PanelAttr.usWinH 			= LCD_PANEL_HEIGHT;
	m_PanelAttr.usWinBPP 		= 2;
	m_PanelAttr.usWinFmt 		= LCD_WIN_FMT_16BPP;
	//m_PanelAttr.ulWinAddr 		= 0x02500000;
	m_PanelAttr.ulWinAddr 		= LCD_TEST_ADDR;

	MMPF_LCD_InitPanel(&m_PanelAttr);
#endif
}

#if 0
void __dump_reg(unsigned int addr, unsigned int size)
{
	int i = 0;
	unsigned int *p = AITC_BASE_DSPY;

	for (i = 0; i < (size / 4); i++) {
		pr_info("x%x: x%8x \n", p - (0x70000000), *(p)); //DSPY_RD_D(p)
		p++;
	}
}
#endif

void RTNA_LCD_Init(void)
{
	RTNA_DBG_Str(0, "### RTNA_LCD_Init for ILI8961 ...\r\n");

#if (SECONDARY_DISPLAY == 1)
	RTNA_LCD_Init2ndLCD();
#else
	RTNA_LCD_InitMainLCD();
#endif

	// Global reset
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x051E);
	Delayms(80);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x055D);
	Delayms(80);

	// Normal operation
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x2B01);			// STB(R2Bh[0]): Standby (Power saving) mode control
															// 0: Standby Mode. (default)
															// 1: Normal operation.
	Delayms(80);

#if (LCD_RGB_MODE == STRIP_RGB)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0418);			// 8-bit Dummy RGB
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x2F71);			// Stripe color filter.
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0695);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0746);
#else
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x040B);			// 8-bit RGB
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x2F61);			// Delta color filter. (default)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0695);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0748);
#endif

#if (LCD_BRIGETNESS_DEFAULT)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0300 | 0x40);	// Brightness(R03h[7:0]): RGB brightness level control
#else
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0300 | 80);		// Brightness(R03h[7:0]): RGB brightness level control
#endif
#if (LCD_CONTRAST_DEFAULT)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0D00 | 0x40);	// CONTRAST(R0Dh[7:0]): RGB contrast level setting, the gain changes (1/64) bit.
#else
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0D00 | 75);		// CONTRAST(R0Dh[7:0]): RGB contrast level setting, the gain changes (1/64) bit.
#endif
#if (LCD_VCOM_DEFAULT)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x019C);			// VCDC(R01h[5:0]): VCOM vlotage DC level selection (20mV/step)
#else
	//MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x019F); 			// VCDC(R01h[5:0]): VCOM vlotage DC level selection (20mV/step)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x019B);			// VCDC(R01h[5:0]): VCOM vlotage DC level selection (20mV/step) // new setting from Mio, TBD
#endif
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0336);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x0B81);

#if (LCD_GAMMA_AUTO)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1604);			// GON_EN(R16h[2]): Gamma op enable Function
															// 0: Manual set gamma by R17h~R1Ah
															// 1: Auto set to gamma2.2. (default)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1757);			// [6:4]: L016_SEL (101)
															// [2:0]: L008_SEL (111)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1875);			// [6:4]: L050_SEL (111)
															// [2:0]: L032_SEL (101)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1944);			// [6:4]: L096_SEL (100)
															// [2:0]: L072_SEL (100)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1A54);			// [6:4]: L120_SEL (101)
															// [2:0]: L110_SEL (100)
#else
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1600);			// GON_EN(R16h[2]): Gamma op enable Function
															// 0: Manual set gamma by R17h~R1Ah
															// 1: Auto set to gamma2.2. (default)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1777);			// [6:4]: L016_SEL (101)
															// [2:0]: L008_SEL (111)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1877);			// [6:4]: L050_SEL (111)
															// [2:0]: L032_SEL (101)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1933);			// [6:4]: L096_SEL (100)
															// [2:0]: L072_SEL (100)
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x1A43);			// [6:4]: L120_SEL (101)
															// [2:0]: L110_SEL (100)
#endif

	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0x9580);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0xAF04);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0xA522);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0xA612);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0xA706);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0xB301);
	MMPF_LCD_Write16BitCmd(&m_PanelAttr, 0xB506);
}

static void ili8961_ic_test(unsigned int phys, unsigned int virt)
{	
	m_PanelAttr.ulWinAddrPhys = phys;
	m_PanelAttr.ulWinAddrVirt = virt;

	MMPF_LCD_DrawTestPattern(&m_PanelAttr);
}

static int ili8961_reset(struct device *dev)
{
	MMPF_PIO_EnableGpioMode(LCD_GPIO_RESET,1,MMP_TRUE);
	MMPF_PIO_EnableOutputMode(LCD_GPIO_RESET, MMP_TRUE, MMP_TRUE);

	MMPF_PIO_SetData(LCD_GPIO_RESET, !LCD_GPIO_RESET_ACT_LEVEL, MMP_TRUE);
	MMPF_OS_Sleep_MS(20);

	MMPF_PIO_SetData(LCD_GPIO_RESET, LCD_GPIO_RESET_ACT_LEVEL, MMP_TRUE);
	MMPF_OS_Sleep_MS(100);

	MMPF_PIO_SetData(LCD_GPIO_RESET, !LCD_GPIO_RESET_ACT_LEVEL, MMP_TRUE);
	MMPF_OS_Sleep_MS(120);
	
	return 0;
}

static int ili8961_backlight_enable(struct device *dev)
{
	//struct ait_fb_platform_data_info *data = (struct ait_fb_platform_data_info *)dev->platform_data;
	//struct cl_fb_cpulcd_device_info *cpulcd = (struct cl_fb_cpulcd_device_info *)data->disp;

	MMPF_PIO_EnableGpioMode(LCD_GPIO_BACK_LIGHT,1,MMP_TRUE);
	MMPF_PIO_EnableOutputMode(LCD_GPIO_BACK_LIGHT, MMP_TRUE, MMP_TRUE);
	MMPF_PIO_SetData(LCD_GPIO_BACK_LIGHT, !LCD_GPIO_BACK_LIGHT_ACT_LEVEL, MMP_TRUE);
	MMPF_OS_Sleep_MS(20);
	MMPF_PIO_SetData(LCD_GPIO_BACK_LIGHT, LCD_GPIO_BACK_LIGHT_ACT_LEVEL, MMP_TRUE);
	MMPF_OS_Sleep_MS(20);

	return 0;
}

static int ili8961_ic_init(struct device *dev)
{
	ili8961_reset(dev);
	RTNA_LCD_Init();
	
	return 0;
}

static struct ait_fb_display_device_info ili8961_disp_info = {
	.name = "ALC027A(ILI8961)",
	.boot_on = 1,
	.init = ili8961_ic_init,
	.enable = ili8961_backlight_enable,
	.disable = 0,
	.close = 0,
	.test = ili8961_ic_test,
	.panel = &m_PanelAttr,
};

struct ait_fb_platform_data_info alc027a_disp_plat_data = {
	.fb = {
		.format 	= DU_IMAGE_FORMAT_RGB565,
		.width		= LCD_PANEL_WIDTH,
		.height 	= LCD_PANEL_HEIGHT,
		.rot_angle	= DU_STRP_ROT_0,
	},
	.disp = (struct ait_fb_display_device_info *)&ili8961_disp_info,
};
EXPORT_SYMBOL(alc027a_disp_plat_data);

