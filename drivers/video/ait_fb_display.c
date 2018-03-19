#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <mach/ait_fb_display.h>

//#define FB_DEBUG
#ifdef FB_DEBUG
#define fb_dbg(format, arg...) printk( format, ##arg)
#else
#define fb_dbg(format, arg...)
#endif
void __iomem *fb_reg_temp;


#ifdef FB_DEBUG
static void dump_dspy(struct ait_fb_display_info *aitfb)
{
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	fb_dbg("pDSPY->DSPY_CTL_0(0x00) = 0x%x\n", pDSPY->DSPY_CTL_0);
	fb_dbg("pDSPY->DSPY_CTL_2(0x02) = 0x%x\n", pDSPY->DSPY_CTL_2);
	fb_dbg("pDSPY->DSPY_INT_HOST_EN(0x04) = 0x%x\n", pDSPY->DSPY_INT_HOST_EN);
	fb_dbg("pDSPY->DSPY_INT_HOST_SR(0x06) = 0x%x\n", pDSPY->DSPY_INT_HOST_SR);
	fb_dbg("pDSPY->DSPY_INT_CPU_EN(0x08) = 0x%x\n", pDSPY->DSPY_INT_CPU_EN);
	fb_dbg("pDSPY->DSPY_INT_CPU_SR(0x0A) = 0x%x\n", pDSPY->DSPY_INT_CPU_SR);
	fb_dbg("pDSPY->DSPY_LCD_FLM_CTL(0x0C) = 0x%x\n", pDSPY->DSPY_LCD_FLM_CTL);
	fb_dbg("pDSPY->DSPY_LCD_VSYNC_CTL(0x0D) = 0x%x\n", pDSPY->DSPY_LCD_VSYNC_CTL);
	fb_dbg("pDSPY->DSPY_FLM_VSYNC_CNT(0x0E) = 0x%x\n", pDSPY->DSPY_FLM_VSYNC_CNT);
	fb_dbg("pDSPY->DSPY_LCD_TX_0(0x10) = 0x%x\n", pDSPY->DSPY_LCD_TX_0);
	fb_dbg("pDSPY->DSPY_LCD_TX_1(0x14) = 0x%x\n", pDSPY->DSPY_LCD_TX_1);
	fb_dbg("pDSPY->DSPY_LCD_TX_2(0x18) = 0x%x\n", pDSPY->DSPY_LCD_TX_2);
	fb_dbg("pDSPY->DSPY_LCD_TX_3(0x1C) = 0x%x\n", pDSPY->DSPY_LCD_TX_3);
	fb_dbg("pDSPY->DSPY_LCD_TX_4(0x20) = 0x%x\n", pDSPY->DSPY_LCD_TX_4);
	fb_dbg("pDSPY->DSPY_LCD_TX_5(0x24) = 0x%x\n", pDSPY->DSPY_LCD_TX_5);
	fb_dbg("pDSPY->DSPY_LCD_TX_6(0x28) = 0x%x\n", pDSPY->DSPY_LCD_TX_6);
	fb_dbg("pDSPY->DSPY_LCD_AUTO_CFG(0x2C) = 0x%x\n", pDSPY->DSPY_LCD_AUTO_CFG);
	fb_dbg("pDSPY->DSPY_PLCD_CTL(0x30) = 0x%x\n", pDSPY->DSPY_PLCD_CTL);
	fb_dbg("pDSPY->DSPY_PLCD_FMT(0x32) = 0x%x\n", pDSPY->DSPY_PLCD_FMT);
	fb_dbg("pDSPY->DSPY_LCD_SR(0x34) = 0x%x\n", pDSPY->DSPY_LCD_SR);
	fb_dbg("pDSPY->DSPY_TV_LINE(0x36) = 0x%x\n", pDSPY->DSPY_TV_LINE);
	fb_dbg("pDSPY->DSPY_PLCD_RS_LEAD_CS_CYC(0x38) = 0x%x\n", pDSPY->DSPY_PLCD_RS_LEAD_CS_CYC);
	fb_dbg("pDSPY->DSPY_PLCD_CS_LEAD_RW_CYC(0x3A) = 0x%x\n", pDSPY->DSPY_PLCD_CS_LEAD_RW_CYC);
	fb_dbg("pDSPY->DSPY_PLCD_RW_CYC(0x3C) = 0x%x\n", pDSPY->DSPY_PLCD_RW_CYC);
	fb_dbg("pDSPY->DSPY_PLCD_IDX_CMD_NUM(0x3E) = 0x%x\n", pDSPY->DSPY_PLCD_IDX_CMD_NUM);
	fb_dbg("pDSPY->DSPY_W(0x40) = 0x%x\n", pDSPY->DSPY_W);
	fb_dbg("pDSPY->DSPY_H(0x44) = 0x%x\n", pDSPY->DSPY_H);
	fb_dbg("pDSPY->DSPY_PIXL_CNT(0x48) = 0x%x\n", pDSPY->DSPY_PIXL_CNT);
	fb_dbg("pDSPY->DSPY_CTL_4(0x4C) = 0x%x\n", pDSPY->DSPY_CTL_4);
	fb_dbg("pDSPY->DSPY_OSD_MCI_PORT_SEL_MP(0x4D) = 0x%x\n", pDSPY->DSPY_OSD_MCI_PORT_SEL_MP);
	fb_dbg("pDSPY->DSPY_WIN_PRIO(0x4E) = 0x%x\n", pDSPY->DSPY_WIN_PRIO);
	fb_dbg("pDSPY->DSPY_WIN_BIND_SEL_MP(0x4F) = 0x%x\n", pDSPY->DSPY_WIN_BIND_SEL_MP);
	fb_dbg("pDSPY->DSPY_ICON_W(0x50) = 0x%x\n", pDSPY->DSPY_ICON_W);
	fb_dbg("pDSPY->DSPY_ICON_H(0x54) = 0x%x\n", pDSPY->DSPY_ICON_H);
	fb_dbg("pDSPY->DSPY_ICON_X(0x58) = 0x%x\n", pDSPY->DSPY_ICON_X);
	fb_dbg("pDSPY->DSPY_ICON_Y(0x5C) = 0x%x\n", pDSPY->DSPY_ICON_Y);
	fb_dbg("pDSPY->DSPY_ICON_CTL(0x60) = 0x%x\n", pDSPY->DSPY_ICON_CTL);
	fb_dbg("pDSPY->DSPY_BG_COLOR(0x64) = 0x%x\n", pDSPY->DSPY_BG_COLOR);
	fb_dbg("pDSPY->DSPY_PLCD_READ_PORT(0x68) = 0x%x\n", pDSPY->DSPY_PLCD_READ_PORT);
	fb_dbg("pDSPY->DSPY_FIFO_CLR(0x6C) = 0x%x\n", pDSPY->DSPY_FIFO_CLR);
	fb_dbg("pDSPY->TV_DAC_TEST_MODE_DATA(0x6E) = 0x%x\n", pDSPY->TV_DAC_TEST_MODE_DATA);
	fb_dbg("pDSPY->DSPY_MAIN_ADDR_ST(0x100) = 0x%x\n", pDSPY->DSPY_MAIN_ADDR_ST);
	fb_dbg("pDSPY->DSPY_MAIN_OFST_ST(0x108) = 0x%x\n", pDSPY->DSPY_MAIN_OFST_ST);
	fb_dbg("pDSPY->DSPY_MAIN_OFST_PIXL(0x110) = 0x%x\n", pDSPY->DSPY_MAIN_OFST_PIXL);
	fb_dbg("pDSPY->DSPY_MAIN_OFST_ROW(0x114) = 0x%x\n", pDSPY->DSPY_MAIN_OFST_ROW);
	fb_dbg("pDSPY->DSPY_MAIN_U_ADDR_ST(0x120) = 0x%x\n", pDSPY->DSPY_MAIN_U_ADDR_ST);
	fb_dbg("pDSPY->DSPY_MAIN_OFST_UV_ST(0x128) = 0x%x\n", pDSPY->DSPY_MAIN_OFST_UV_ST);
	fb_dbg("pDSPY->DSPY_MAIN_OFST_UV_PIXL(0x130) = 0x%x\n", pDSPY->DSPY_MAIN_OFST_UV_PIXL);
	fb_dbg("pDSPY->DSPY_MAIN_OFST_UV_ROW(0x134) = 0x%x\n", pDSPY->DSPY_MAIN_OFST_UV_ROW);
	fb_dbg("pDSPY->DSPY_MAIN_V_ADDR_ST(0x140) = 0x%x\n", pDSPY->DSPY_MAIN_V_ADDR_ST);
	fb_dbg("pDSPY->DSPY_MAIN_DBG_OBS_SEL_MP(0x148) = 0x%x\n", pDSPY->DSPY_MAIN_DBG_OBS_SEL_MP);
	fb_dbg("pDSPY->DSPY_MAIN_FIFO_MAX_CTL_MP(0x149) = 0x%x\n", pDSPY->DSPY_MAIN_FIFO_MAX_CTL_MP);
	fb_dbg("pDSPY->DSPY_MAIN_W(0x150) = 0x%x\n", pDSPY->DSPY_MAIN_W);
	fb_dbg("pDSPY->DSPY_MAIN_H(0x154) = 0x%x\n", pDSPY->DSPY_MAIN_H);
	fb_dbg("pDSPY->DSPY_MAIN_X(0x158) = 0x%x\n", pDSPY->DSPY_MAIN_X);
	fb_dbg("pDSPY->DSPY_MAIN_Y(0x15C) = 0x%x\n", pDSPY->DSPY_MAIN_Y);
	fb_dbg("pDSPY->DSPY_MAIN_CTL(0x160) = 0x%x\n", pDSPY->DSPY_MAIN_CTL);
	fb_dbg("pDSPY->DSPY_MAIN_CTL_2(0x162) = 0x%x\n", pDSPY->DSPY_MAIN_CTL_2);
	fb_dbg("pDSPY->DSPY_MAIN_FMT(0x164) = 0x%x\n", pDSPY->DSPY_MAIN_FMT);
	fb_dbg("pDSPY->DSPY_MAIN_PIXL_CNT(0x168) = 0x%x\n", pDSPY->DSPY_MAIN_PIXL_CNT);
	fb_dbg("pDSPY->DSPY_MAIN_TP_CTL(0x170) = 0x%x\n", pDSPY->DSPY_MAIN_TP_CTL);
	fb_dbg("pDSPY->DSPY_MAIN_GLOBAL_ALPHA_WT(0x172) = 0x%x\n", pDSPY->DSPY_MAIN_GLOBAL_ALPHA_WT);
	fb_dbg("pDSPY->DSPY_MAIN_WIDTH_DUP_DIV_MP(0x17A) = 0x%x\n", pDSPY->DSPY_MAIN_WIDTH_DUP_DIV_MP);
	fb_dbg("pDSPY->DSPY_MAIN_PIXL_CNT_DUP_DIV_MP(0x17C) = 0x%x\n", pDSPY->DSPY_MAIN_PIXL_CNT_DUP_DIV_MP);
	fb_dbg("pDSPY->DSPY_MAIN_UV_GAIN_11(0x190) = 0x%x\n", pDSPY->DSPY_MAIN_UV_GAIN_11);
	fb_dbg("pDSPY->DSPY_MAIN_UV_GAIN_12(0x191) = 0x%x\n", pDSPY->DSPY_MAIN_UV_GAIN_12);
	fb_dbg("pDSPY->DSPY_MAIN_UV_GAIN_21(0x192) = 0x%x\n", pDSPY->DSPY_MAIN_UV_GAIN_21);
	fb_dbg("pDSPY->DSPY_MAIN_UV_GAIN_22(0x193) = 0x%x\n", pDSPY->DSPY_MAIN_UV_GAIN_22);
	fb_dbg("pDSPY->DSPY_MAIN_RGB_GAIN(0x194) = 0x%x\n", pDSPY->DSPY_MAIN_RGB_GAIN);
	fb_dbg("pDSPY->DSPY_MAIN_RGB_OFST(0x196) = 0x%x\n", pDSPY->DSPY_MAIN_RGB_OFST);
	fb_dbg("pDSPY->DSPY_SCD_CTL(0x1A0) = 0x%x\n", pDSPY->DSPY_SCD_CTL);
	fb_dbg("pDSPY->DSPY_SCD_FLM_CTL(0x1A4) = 0x%x\n", pDSPY->DSPY_SCD_FLM_CTL);
	fb_dbg("pDSPY->DSPY_SCD_VSYNC_CTL(0x1A5) = 0x%x\n", pDSPY->DSPY_SCD_VSYNC_CTL);
	fb_dbg("pDSPY->DSPY_SCD_FLM_VSYNC_CNT(0x1A6) = 0x%x\n", pDSPY->DSPY_SCD_FLM_VSYNC_CNT);
	fb_dbg("pDSPY->DSPY_SCD_BG_COLOR(0x1A8) = 0x%x\n", pDSPY->DSPY_SCD_BG_COLOR);
	fb_dbg("pDSPY->DSPY_SCD_PLCD_READ_PORT(0x1AC) = 0x%x\n", pDSPY->DSPY_SCD_PLCD_READ_PORT);
	fb_dbg("pDSPY->DSPY_SCD_LCD_TX_0(0x1B0) = 0x%x\n", pDSPY->DSPY_SCD_LCD_TX_0);
	fb_dbg("pDSPY->DSPY_SCD_LCD_TX_1(0x1B4) = 0x%x\n", pDSPY->DSPY_SCD_LCD_TX_1);
	fb_dbg("pDSPY->DSPY_SCD_LCD_TX_2(0x1B8) = 0x%x\n", pDSPY->DSPY_SCD_LCD_TX_2);
	fb_dbg("pDSPY->DSPY_SCD_LCD_AUTO_CFG(0x1BC) = 0x%x\n", pDSPY->DSPY_SCD_LCD_AUTO_CFG);
	fb_dbg("pDSPY->DSPY_SCD_W(0x1C0) = 0x%x\n", pDSPY->DSPY_SCD_W);
	fb_dbg("pDSPY->DSPY_SCD_H(0x1C4) = 0x%x\n", pDSPY->DSPY_SCD_H);
	fb_dbg("pDSPY->DSPY_SCD_PIXL_CNT(0x1C8) = 0x%x\n", pDSPY->DSPY_SCD_PIXL_CNT);
	fb_dbg("pDSPY->DSPY_SCD_WIN_PIXL_CNT(0x1CC) = 0x%x\n", pDSPY->DSPY_SCD_WIN_PIXL_CNT);
	fb_dbg("pDSPY->DSPY_SCD_WIN_ADDR_ST(0x1D0) = 0x%x\n", pDSPY->DSPY_SCD_WIN_ADDR_ST);
	fb_dbg("pDSPY->DSPY_SCD_WIN_OFST_ST(0x1D4) = 0x%x\n", pDSPY->DSPY_SCD_WIN_OFST_ST);
	fb_dbg("pDSPY->DSPY_SCD_WIN_W(0x1D8) = 0x%x\n", pDSPY->DSPY_SCD_WIN_W);
	fb_dbg("pDSPY->DSPY_SCD_WIN_H(0x1DA) = 0x%x\n", pDSPY->DSPY_SCD_WIN_H);
	fb_dbg("pDSPY->DSPY_SCD_WIN_X(0x1DC) = 0x%x\n", pDSPY->DSPY_SCD_WIN_X);
	fb_dbg("pDSPY->DSPY_SCD_WIN_Y(0x1DE) = 0x%x\n", pDSPY->DSPY_SCD_WIN_Y);
	fb_dbg("pDSPY->DSPY_MAIN_TV_EVEN_FIELD_ST(0x1E0) = 0x%x\n", pDSPY->DSPY_MAIN_TV_EVEN_FIELD_ST);
	fb_dbg("pDSPY->DSPY_WIN_ALPHA_WT_1(0x1E8) = 0x%x\n", pDSPY->DSPY_WIN_ALPHA_WT_1);
	fb_dbg("pDSPY->DSPY_WIN_ALPHA_WT_2(0x1EC) = 0x%x\n", pDSPY->DSPY_WIN_ALPHA_WT_2);
	fb_dbg("pDSPY->DSPY_MAIN_Y_FIFO_CTL(0x1F8) = 0x%x\n", pDSPY->DSPY_MAIN_Y_FIFO_CTL);
	fb_dbg("pDSPY->DSPY_MAIN_U_FIFO_CTL(0x1FA) = 0x%x\n", pDSPY->DSPY_MAIN_U_FIFO_CTL);
	fb_dbg("pDSPY->DSPY_MAIN_V_FIFO_CTL(0x1FC) = 0x%x\n", pDSPY->DSPY_MAIN_V_FIFO_CTL);
	fb_dbg("pDSPY->DSPY_PIP_ADDR_ST(0x200) = 0x%x\n", pDSPY->DSPY_PIP_ADDR_ST);
	fb_dbg("pDSPY->DSPY_PIP_OFST_ST(0x208) = 0x%x\n", pDSPY->DSPY_PIP_OFST_ST);
	fb_dbg("pDSPY->DSPY_PIP_OFST_PIXL(0x210) = 0x%x\n", pDSPY->DSPY_PIP_OFST_PIXL);
	fb_dbg("pDSPY->DSPY_PIP_OFST_ROW(0x214) = 0x%x\n", pDSPY->DSPY_PIP_OFST_ROW);
	fb_dbg("pDSPY->DSPY_PIP_U_ADDR_ST(0x220) = 0x%x\n", pDSPY->DSPY_PIP_U_ADDR_ST);
	fb_dbg("pDSPY->DSPY_PIP_OFST_UV_ST(0x228) = 0x%x\n", pDSPY->DSPY_PIP_OFST_UV_ST);
	fb_dbg("pDSPY->DSPY_PIP_OFST_UV_PIXL(0x230) = 0x%x\n", pDSPY->DSPY_PIP_OFST_UV_PIXL);
	fb_dbg("pDSPY->DSPY_PIP_OFST_UV_ROW(0x234) = 0x%x\n", pDSPY->DSPY_PIP_OFST_UV_ROW);
	fb_dbg("pDSPY->DSPY_PIP_V_ADDR_ST(0x240) = 0x%x\n", pDSPY->DSPY_PIP_V_ADDR_ST);
	fb_dbg("pDSPY->DSPY_PIP_DBG_OBS_SEL_MP(0x248) = 0x%x\n", pDSPY->DSPY_PIP_DBG_OBS_SEL_MP);
	fb_dbg("pDSPY->DSPY_PIP_FIFO_MAX_CTL_MP(0x249) = 0x%x\n", pDSPY->DSPY_PIP_FIFO_MAX_CTL_MP);
	fb_dbg("pDSPY->DSPY_PIP_W(0x250) = 0x%x\n", pDSPY->DSPY_PIP_W);
	fb_dbg("pDSPY->DSPY_PIP_H(0x254) = 0x%x\n", pDSPY->DSPY_PIP_H);
	fb_dbg("pDSPY->DSPY_PIP_X(0x258) = 0x%x\n", pDSPY->DSPY_PIP_X);
	fb_dbg("pDSPY->DSPY_PIP_Y(0x25C) = 0x%x\n", pDSPY->DSPY_PIP_Y);
	fb_dbg("pDSPY->DSPY_PIP_CTL(0x260) = 0x%x\n", pDSPY->DSPY_PIP_CTL);
	fb_dbg("pDSPY->DSPY_PIP_CTL_2(0x262) = 0x%x\n", pDSPY->DSPY_PIP_CTL_2);
	fb_dbg("pDSPY->DSPY_PIP_FMT(0x264) = 0x%x\n", pDSPY->DSPY_PIP_FMT);
	fb_dbg("pDSPY->DSPY_PIP_PIXL_CNT(0x268) = 0x%x\n", pDSPY->DSPY_PIP_PIXL_CNT);
	fb_dbg("pDSPY->DSPY_SOUT_GRAB_PIXL_CNT(0x26C) = 0x%x\n", pDSPY->DSPY_SOUT_GRAB_PIXL_CNT);
	fb_dbg("pDSPY->DSPY_PIP_TP_CTL(0x270) = 0x%x\n", pDSPY->DSPY_PIP_TP_CTL);
	fb_dbg("pDSPY->DSPY_PIP_GLOBAL_ALPHA_WT(0x272) = 0x%x\n", pDSPY->DSPY_PIP_GLOBAL_ALPHA_WT);
	fb_dbg("pDSPY->DSPY_PIP_WIDTH_DUP_DIV_MP(0x27A) = 0x%x\n", pDSPY->DSPY_PIP_WIDTH_DUP_DIV_MP);
	fb_dbg("pDSPY->DSPY_PIP_PIXL_CNT_DUP_DIV_MP(0x27C) = 0x%x\n", pDSPY->DSPY_PIP_PIXL_CNT_DUP_DIV_MP);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_CTL(0x280) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_CTL);
	fb_dbg("pDSPY->DSPY_PIP_SOUT_CTL(0x282) = 0x%x\n", pDSPY->DSPY_PIP_SOUT_CTL);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_H_N(0x284) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_H_N);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_H_M(0x286) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_H_M);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_V_N(0x288) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_V_N);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_V_M(0x28A) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_V_M);
	fb_dbg("pDSPY->DSPY_PIP_SEDGE_CTL(0x28C) = 0x%x\n", pDSPY->DSPY_PIP_SEDGE_CTL);
	fb_dbg("pDSPY->DSPY_PIP_SEDGE_GAIN_VAL(0x28E) = 0x%x\n", pDSPY->DSPY_PIP_SEDGE_GAIN_VAL);
	fb_dbg("pDSPY->DSPY_PIP_SEDGE_CORE_VAL(0x28F) = 0x%x\n", pDSPY->DSPY_PIP_SEDGE_CORE_VAL);
	fb_dbg("pDSPY->DSPY_PIP_UV_GAIN_11(0x290) = 0x%x\n", pDSPY->DSPY_PIP_UV_GAIN_11);
	fb_dbg("pDSPY->DSPY_PIP_UV_GAIN_12(0x291) = 0x%x\n", pDSPY->DSPY_PIP_UV_GAIN_12);
	fb_dbg("pDSPY->DSPY_PIP_UV_GAIN_21(0x292) = 0x%x\n", pDSPY->DSPY_PIP_UV_GAIN_21);
	fb_dbg("pDSPY->DSPY_PIP_UV_GAIN_22(0x293) = 0x%x\n", pDSPY->DSPY_PIP_UV_GAIN_22);
	fb_dbg("pDSPY->DSPY_PIP_RGB_GAIN(0x294) = 0x%x\n", pDSPY->DSPY_PIP_RGB_GAIN);
	fb_dbg("pDSPY->DSPY_PIP_RGB_OFST(0x296) = 0x%x\n", pDSPY->DSPY_PIP_RGB_OFST);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_H_WT(0x298) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_H_WT);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_V_WT(0x29A) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_V_WT);
	fb_dbg("pDSPY->DSPY_PIP_SCAL_WT_FORCE1_EN(0x29C) = 0x%x\n", pDSPY->DSPY_PIP_SCAL_WT_FORCE1_EN);
	fb_dbg("pDSPY->DSPY_PIP_SCA_IN_W(0x2A0) = 0x%x\n", pDSPY->DSPY_PIP_SCA_IN_W);
	fb_dbg("pDSPY->DSPY_PIP_SCA_IN_H(0x2A4) = 0x%x\n", pDSPY->DSPY_PIP_SCA_IN_H);
	fb_dbg("pDSPY->DSPY_PIP_SOUT_GRAB_H_ST(0x2B0) = 0x%x\n", pDSPY->DSPY_PIP_SOUT_GRAB_H_ST);
	fb_dbg("pDSPY->DSPY_PIP_SOUT_GRAB_H_ED(0x2B4) = 0x%x\n", pDSPY->DSPY_PIP_SOUT_GRAB_H_ED);
	fb_dbg("pDSPY->DSPY_PIP_SOUT_GRAB_V_ST(0x2BB) = 0x%x\n", pDSPY->DSPY_PIP_SOUT_GRAB_V_ST);
	fb_dbg("pDSPY->DSPY_PIP_SOUT_GRAB_V_ED(0x2BC) = 0x%x\n", pDSPY->DSPY_PIP_SOUT_GRAB_V_ED);
	fb_dbg("pDSPY->DSPY_Y_ADDR_LOW_BOUND(0x2C0) = 0x%x\n", pDSPY->DSPY_Y_ADDR_LOW_BOUND);
	fb_dbg("pDSPY->DSPY_Y_ADDR_HIGH_BOUND(0x2C4) = 0x%x\n", pDSPY->DSPY_Y_ADDR_HIGH_BOUND);
	fb_dbg("pDSPY->DSPY_U_ADDR_LOW_BOUND(0x2C8) = 0x%x\n", pDSPY->DSPY_U_ADDR_LOW_BOUND);
	fb_dbg("pDSPY->DSPY_U_ADDR_HIGH_BOUND(0x2CC) = 0x%x\n", pDSPY->DSPY_U_ADDR_HIGH_BOUND);
	fb_dbg("pDSPY->DSPY_MPEG4_DEC_ROW_CNT(0x2D0) = 0x%x\n", pDSPY->DSPY_MPEG4_DEC_ROW_CNT);
	fb_dbg("pDSPY->DSPY_V_ADDR_LOW_BOUND(0x2D8) = 0x%x\n", pDSPY->DSPY_V_ADDR_LOW_BOUND);
	fb_dbg("pDSPY->DSPY_V_ADDR_HIGH_BOUND(0x2DC) = 0x%x\n", pDSPY->DSPY_V_ADDR_HIGH_BOUND);
	fb_dbg("pDSPY->DSPY_PIP_TV_EVEN_FIELD_ST(0x2E0) = 0x%x\n", pDSPY->DSPY_PIP_TV_EVEN_FIELD_ST);
	fb_dbg("pDSPY->DSPY_PIP_BUF_FULL_THD(0x2F0) = 0x%x\n", pDSPY->DSPY_PIP_BUF_FULL_THD);
	fb_dbg("pDSPY->DSPY_PIP_BUF_STOP_THD(0x2F1) = 0x%x\n", pDSPY->DSPY_PIP_BUF_STOP_THD);
	fb_dbg("pDSPY->DSPY_PRM_BUF_FULL_THD(0x2F4) = 0x%x\n", pDSPY->DSPY_PRM_BUF_FULL_THD);
	fb_dbg("pDSPY->DSPY_SCD_BUF_FULL_THD(0x2F6) = 0x%x\n", pDSPY->DSPY_SCD_BUF_FULL_THD);
	fb_dbg("pDSPY->DSPY_PIP_Y_FIFO_CTL(0x2F8) = 0x%x\n", pDSPY->DSPY_PIP_Y_FIFO_CTL);
	fb_dbg("pDSPY->DSPY_PIP_U_FIFO_CTL(0x2FA) = 0x%x\n", pDSPY->DSPY_PIP_U_FIFO_CTL);
	fb_dbg("pDSPY->DSPY_PIP_V_FIFO_CTL(0x2FC) = 0x%x\n", pDSPY->DSPY_PIP_V_FIFO_CTL);
	
}

static void dump_tv(struct ait_fb_display_info *aitfb)
{
	AITPS_TV pTV = aitfb->tv_reg;
	AITPS_GBL pGBL = AITC_BASE_GBL;

	fb_dbg("pTV->TVIF_TEST_1ST_Y2CrY1Cb(0x70) = 0x%x\n", pTV->TVIF_TEST_1ST_Y2CrY1Cb);
	fb_dbg("pTV->TVIF_TEST_2ND_Y2CrY1Cb(0x74) = 0x%x\n", pTV->TVIF_TEST_2ND_Y2CrY1Cb);
	fb_dbg("pTV->TVIF_DAC_IF_1ST_CTL(0x78) = 0x%x\n", pTV->TVIF_DAC_IF_1ST_CTL);
	fb_dbg("pTV->TVIF_DAC_IF_2ND_CTL(0x79) = 0x%x\n", pTV->TVIF_DAC_IF_2ND_CTL);
	fb_dbg("pTV->TVIF_DAC_IF_3RD_CTL(0x7A) = 0x%x\n", pTV->TVIF_DAC_IF_3RD_CTL);
	fb_dbg("pTV->TVIF_BACKGROUND_Y_COLOR(0x7C) = 0x%x\n", pTV->TVIF_BACKGROUND_Y_COLOR);
	fb_dbg("pTV->TVIF_BACKGROUND_U_COLOR(0x7B) = 0x%x\n", pTV->TVIF_BACKGROUND_U_COLOR);
	fb_dbg("pTV->TVIF_BACKGROUND_V_COLOR(0x7E) = 0x%x\n", pTV->TVIF_BACKGROUND_V_COLOR);
	fb_dbg("pTV->TVIF_CLK_DELAY_V1(0x7F) = 0x%x\n", pTV->TVIF_CLK_DELAY_V1);
	fb_dbg("pTV->TVIF_IF_EN(0x80) = 0x%x\n", pTV->TVIF_IF_EN);
	fb_dbg("pTV->TVIF_ENDLINE_OFFSET_CTL(0x81) = 0x%x\n", pTV->TVIF_ENDLINE_OFFSET_CTL);
	fb_dbg("pTV->TVIF_EARLY_PIXL(0x82) = 0x%x\n", pTV->TVIF_EARLY_PIXL);
	fb_dbg("pTV->TVIF_1ST_PXL_RQST_TIMING(0x83) = 0x%x\n", pTV->TVIF_1ST_PXL_RQST_TIMING);
	fb_dbg("pTV->TVIF_NTSC_ODFIELD_LINE(0x84) = 0x%x\n", pTV->TVIF_NTSC_ODFIELD_LINE);
	fb_dbg("pTV->TVIF_NTSC_EVFIELD_LINE(0x86) = 0x%x\n", pTV->TVIF_NTSC_EVFIELD_LINE);
	fb_dbg("pTV->TVIF_PAL_1ST_FIELD_LINE(0x88) = 0x%x\n", pTV->TVIF_PAL_1ST_FIELD_LINE);
	fb_dbg("pTV->TVIF_PAL_2ND_FIELD_LINE(0x8A) = 0x%x\n", pTV->TVIF_PAL_2ND_FIELD_LINE);
	fb_dbg("pTV->TVIF_NTSC_EVLINE_SUB1(0x8c) = 0x%x\n", pTV->TVIF_NTSC_EVLINE_SUB1);
	fb_dbg("pTV->TVIF_PAL_EVLINE_SUB1(0x8E) = 0x%x\n", pTV->TVIF_PAL_EVLINE_SUB1);
	fb_dbg("pTV->TVIF_INT1_CPU(0x90) = 0x%x\n", pTV->TVIF_INT1_CPU);
	fb_dbg("pTV->TVIF_INT2_CPU(0x92) = 0x%x\n", pTV->TVIF_INT2_CPU);
	fb_dbg("pTV->TVIF_INT1_HOST(0x94) = 0x%x\n", pTV->TVIF_INT1_HOST);
	fb_dbg("pTV->TVIF_INT2_HOST(0x96) = 0x%x\n", pTV->TVIF_INT2_HOST);
	fb_dbg("pTV->TVIF_IMAGE_WIDTH(0x98) = 0x%x\n", pTV->TVIF_IMAGE_WIDTH);
	fb_dbg("pTV->TVIF_IMAGE_HEIGHT(0x9A) = 0x%x\n", pTV->TVIF_IMAGE_HEIGHT);
	fb_dbg("pTV->TVIF_IMAGE_START_X(0x9C) = 0x%x\n", pTV->TVIF_IMAGE_START_X);
	fb_dbg("pTV->TVIF_IMAGE_START_Y(0x9E) = 0x%x\n", pTV->TVIF_IMAGE_START_Y);
	fb_dbg("pTV->TVENC_SYNC_CTL(0xA0)) = 0x%x\n", pTV->TVENC_SYNC_CTL);
	fb_dbg("pTV->TVENC_MODE_CTL(0xA4) = 0x%x\n", pTV->TVENC_MODE_CTL);
	fb_dbg("pTV->TVENC_CLOSED_CAPTION(0xA8) = 0x%x\n", pTV->TVENC_CLOSED_CAPTION);
	fb_dbg("pTV->TVENC_Y_SCALE_CTL(0xAC) = 0x%x\n", pTV->TVENC_Y_SCALE_CTL);
	fb_dbg("pTV->TVENC_U_SCALE_CTL(0xB0) = 0x%x\n", pTV->TVENC_U_SCALE_CTL);
	fb_dbg("pTV->TVENC_V_SCALE_CTL(0xB4) = 0x%x\n", pTV->TVENC_V_SCALE_CTL);
	fb_dbg("pTV->TVENC_GAMMA_COEF_0(0xB8) = 0x%x\n", pTV->TVENC_GAMMA_COEF_0);
	fb_dbg("pTV->TVENC_GAMMA_COEF_1_2(0xBC) = 0x%x\n", pTV->TVENC_GAMMA_COEF_1_2);
	fb_dbg("pTV->TVENC_GAMMA_COEF_3_4(0xC0) = 0x%x\n", pTV->TVENC_GAMMA_COEF_3_4);
	fb_dbg("pTV->TVENC_GAMMA_COEF_5_6(0xC4) = 0x%x\n", pTV->TVENC_GAMMA_COEF_5_6);
	fb_dbg("pTV->TVENC_GAMMA_COEF_7_8(0xC8) = 0x%x\n", pTV->TVENC_GAMMA_COEF_7_8);
	fb_dbg("pTV->TVENC_DAC_CONFIG(0xCC) = 0x%x\n", pTV->TVENC_DAC_CONFIG);
	fb_dbg("pTV->TVENC_COLOR_BURST_CONFIG(0xD0) = 0x%x\n", pTV->TVENC_COLOR_BURST_CONFIG);
	fb_dbg("pTV->TVENC_WSS_IF_MODE(0xD8) = 0x%x\n", pTV->TVENC_WSS_IF_MODE);
	fb_dbg("pTV->TVENC_UV_SCALE_GAIN_4_5(0xDC) = 0x%x\n", pTV->TVENC_UV_SCALE_GAIN_4_5);
	fb_dbg("pTV->TVENC_Y_LPF_COEF_00_03(0xE0) = 0x%x\n", pTV->TVENC_Y_LPF_COEF_00_03);
	fb_dbg("pTV->TVENC_Y_LPF_COEF_04_07(0xE4) = 0x%x\n", pTV->TVENC_Y_LPF_COEF_04_07);
	fb_dbg("pTV->TVENC_Y_LPF_COEF_08_0B(0xE8) = 0x%x\n", pTV->TVENC_Y_LPF_COEF_08_0B);
	fb_dbg("pTV->TVENC_Y_LPF_COEF_0C_0F(0xEC) = 0x%x\n", pTV->TVENC_Y_LPF_COEF_0C_0F);
	fb_dbg("pTV->TVENC_Y_LPF_COEF_10_13(0xF0) = 0x%x\n", pTV->TVENC_Y_LPF_COEF_10_13);
	fb_dbg("pTV->TVENC_C1_LPF_COEF_00_03(0xF4) = 0x%x\n", pTV->TVENC_C1_LPF_COEF_00_03);
	fb_dbg("pTV->TVENC_C1_LPF_COEF_04(0xF8) = 0x%x\n", pTV->TVENC_C1_LPF_COEF_04);

	fb_dbg("pGBL->GBL_TV_CLK_SRC = 0x%x\n", pGBL->GBL_TV_CLK_SRC);
	fb_dbg("pGBL->GBL_TV_CLK_DIV = 0x%x\n", pGBL->GBL_TV_CLK_DIV);
	fb_dbg("pGBL->GBL_CLK_DIS[0] = 0x%x\n", pGBL->GBL_CLK_DIS[0]);
	fb_dbg("pGBL->GBL_CLK_DIS[1] = 0x%x\n", pGBL->GBL_CLK_DIS[1]);
	fb_dbg("pGBL->GBL_CLK_EN[0] = 0x%x\n", pGBL->GBL_CLK_EN[0]);
	fb_dbg("pGBL->GBL_CLK_EN[1] = 0x%x\n", pGBL->GBL_CLK_EN[1]);
}
#endif

static int ait_fb_enable_layer(struct ait_fb_display_info *aitfb, int enable)
{
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	fb_dbg("%s +++\n", __func__);

	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 & ~(PRM_DSPY_REG_READY);

	if(enable == 0)
	{
        	switch(aitfb->dis_w_enable)
        	{
		case DIS_WIN_PIP:
			pDSPY->DSPY_PIP_CTL = pDSPY->DSPY_PIP_CTL & ~(WIN_EN);
			break;
		case DIS_WIN_OVLY:
			pDSPY->DSPY_OVLY_CTL = pDSPY->DSPY_OVLY_CTL & ~(WIN_EN);
			break;
		default:
			printk("ERROR:unknown display output layer = 0x%x", aitfb->dis_w_enable);
		}
	}else{
        	switch(aitfb->dis_w_enable)
        	{
		case DIS_WIN_PIP:
			pDSPY->DSPY_PIP_CTL = pDSPY->DSPY_PIP_CTL | WIN_EN;
			break;
		case DIS_WIN_OVLY:
			pDSPY->DSPY_OVLY_CTL = pDSPY->DSPY_OVLY_CTL | WIN_EN;
			break;
		default:
			printk("ERROR:unknown display output layer = 0x%x", aitfb->dis_w_enable);
		}
	}

        pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | PRM_DSPY_REG_READY;

	return 0;
}

static int ait_fb_enable_tp(struct ait_fb_display_info *aitfb, int enable)
{
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	fb_dbg("%s +++\n", __func__);

	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 & ~(PRM_DSPY_REG_READY);

	if(enable == 0)
	{
        	switch(aitfb->dis_w_enable)
        	{
		case DIS_WIN_PIP:
			pDSPY->DSPY_PIP_TP_CTL = pDSPY->DSPY_PIP_TP_CTL & ~(WIN_TP_EN);
			break;
		case DIS_WIN_OVLY:
			pDSPY->DSPY_OVLY_TP_CTL = pDSPY->DSPY_OVLY_TP_CTL & ~(WIN_TP_EN);
			break;
		default:
			printk("ERROR:unknown display output layer = 0x%x", aitfb->dis_w_enable);
		}
	}else{
        	switch(aitfb->dis_w_enable)
        	{
		case DIS_WIN_PIP:
			pDSPY->DSPY_PIP_TP.MP.B_COLOR = enable & 0xFF;
			pDSPY->DSPY_PIP_TP.MP.G_COLOR = (enable >> 8) & 0xFF;
			pDSPY->DSPY_PIP_TP.MP.R_COLOR = (enable >> 16) & 0xFF;
			pDSPY->DSPY_PIP_TP_CTL = pDSPY->DSPY_PIP_TP_CTL | WIN_TP_EN;
			break;
		case DIS_WIN_OVLY:
			pDSPY->DSPY_OVLY_TP.MP.B_COLOR = enable & 0xFF;
			pDSPY->DSPY_OVLY_TP.MP.G_COLOR = (enable >> 8) & 0xFF;
			pDSPY->DSPY_OVLY_TP.MP.R_COLOR = (enable >> 16) & 0xFF;
			pDSPY->DSPY_OVLY_TP_CTL = pDSPY->DSPY_OVLY_TP_CTL | WIN_TP_EN;
			break;
		default:
			printk("ERROR:unknown display output layer = 0x%x", aitfb->dis_w_enable);
		}
	}

        pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | PRM_DSPY_REG_READY;

	return 0;
}

static int ait_fb_display_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ait_fb_display_info *aitfb = container_of(info, struct ait_fb_display_info, fb);
	fb_dbg("%s +++\n", __func__);

	if((var->xres_virtual * var->yres_virtual * var->bits_per_pixel / 8) > aitfb->fb.fix.smem_len)
	{
		return -EINVAL;
	}

	return 0;
}

static int ait_fb_display_set_par(struct fb_info *info)
{
	struct ait_fb_display_info *aitfb = container_of(info, struct ait_fb_display_info, fb);
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	fb_dbg("%s +++\n", __func__);

	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 & ~(PRM_DSPY_REG_READY);

	switch(aitfb->dis_w_enable)
	{
	case DIS_WIN_PIP:		
		break;
	case DIS_WIN_OVLY:
		break;
	default:
		printk("ERROR:unknown display output layer = 0x%x", aitfb->dis_w_enable);
	}

	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | PRM_DSPY_REG_READY;

	return 0;
}

static int ait_fb_display_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct ait_fb_display_info *aitfb = container_of(info, struct ait_fb_display_info, fb);
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	fb_dbg("var->xoffset = 0x%x var->yoffset = 0x%x var->xres = 0x%x var->yres = 0x%x var->xres_virtual = 0x%x var->yres_virtual = 0x%x\n",
		var->xoffset, var->yoffset, var->xres, var->yres, var->xres_virtual, var->yres_virtual);

	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 & ~(PRM_DSPY_REG_READY);

	if(var->yoffset == 0)
	{
		switch(aitfb->dis_w_enable)
		{
		case DIS_WIN_PIP:
			pDSPY->DSPY_PIP_ADDR_ST = aitfb->fb.fix.smem_start;
			break;
		case DIS_WIN_OVLY:
			pDSPY->DSPY_OVLY_ADDR_ST = aitfb->fb.fix.smem_start;
			break;
		default:
			printk("ERROR:unknown display output layer = 0x%x", aitfb->dis_w_enable);
		}
	}else{
		switch(aitfb->dis_w_enable)
		{
		case DIS_WIN_PIP:
			pDSPY->DSPY_PIP_ADDR_ST = aitfb->fb.fix.smem_start + (aitfb->fb.var.xres * aitfb->fb.var.yres * 2);
			break;
		case DIS_WIN_OVLY:
			pDSPY->DSPY_OVLY_ADDR_ST = aitfb->fb.fix.smem_start + (aitfb->fb.var.xres * aitfb->fb.var.yres * 2);
			break;
		default:
			printk("ERROR:unknown display output layer = 0x%x", aitfb->dis_w_enable);
		}
	}

	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | PRM_DSPY_REG_READY;

	return 0;
}

static int ait_fb_display_ioctl(struct fb_info *info, unsigned int cmd,unsigned long arg)
{
	struct ait_fb_display_info *aitfb = container_of(info, struct ait_fb_display_info, fb);
	void __user *argp = (void __user *)arg;
	int enable;

	fb_dbg("%s +++\n", __func__);

	switch(cmd)
	{
	case AITFBIOPUT_LAYERENABLE:
		if(copy_from_user(&enable, argp, sizeof(enable)))
		{
			return -EFAULT;
		}
		ait_fb_enable_layer(aitfb, enable);
		break;
	case AITFBIOPUT_TPENABLE:
		if(copy_from_user(&enable, argp, sizeof(enable)))
		{
			return -EFAULT;
		}
		ait_fb_enable_tp(aitfb, enable);
		break;
	default:
		printk("CMD(0x%x) is not supported\n", cmd);
	}

	return 0;
}

static int setup_tv_interface(struct ait_fb_display_info *aitfb, int type)
{
	AITPS_TV pTV = aitfb->tv_reg;
	AITPS_GBL pGBL = AITC_BASE_GBL;
	unsigned int uldata;

	//enable tv clk
	pGBL->GBL_TV_CLK_SRC = 5; //pll4
	pGBL->GBL_TV_CLK_DIV = GRP_CLK_DIV_EN | GRP_CLK_DIV(7);

	msleep(1);
	pTV->TVIF_CLK_DELAY_V1 = DELAY_1T;

	if(type == DIS_OUT_TYPE_NTSC)
	{
		pTV->TVIF_IF_EN = (TV_IF_DAC_CTL | TV_DISPLAY_SPECIFIED_IMAGE | TV_TYPE_NTSC);
		pTV->TVIF_NTSC_ODFIELD_LINE = 20;
		pTV->TVIF_NTSC_EVFIELD_LINE = 283;
		pTV->TVIF_NTSC_EVLINE_SUB1 = pTV->TVIF_NTSC_EVFIELD_LINE - 1;
		pTV->TVIF_EARLY_PIXL = 30;
		pTV->TVIF_1ST_PXL_RQST_TIMING = 98;
	}else if(type == DIS_OUT_TYPE_PAL){
		pTV->TVIF_IF_EN = (TV_IF_DAC_CTL | TV_DISPLAY_SPECIFIED_IMAGE | TV_TYPE_PAL);
		pTV->TVIF_EARLY_PIXL = 24;
		pTV->TVIF_1ST_PXL_RQST_TIMING = 110;
	}else{
		return -EINVAL;
	}

	pTV->TVIF_DAC_IF_1ST_CTL = TV_DAC_POWER_DOWN_EN | TV_BGREF_POWER_DOWN_EN;
	pTV->TVIF_IMAGE_START_X = 1;
	pTV->TVIF_IMAGE_START_Y = 1;
	pTV->TVIF_IMAGE_WIDTH = aitfb->fb.var.xres;
	pTV->TVIF_IMAGE_HEIGHT = aitfb->fb.var.yres;
	pTV->TVIF_BACKGROUND_Y_COLOR = 0x00;
	pTV->TVIF_BACKGROUND_U_COLOR = 0x80;
	pTV->TVIF_BACKGROUND_V_COLOR = 0x80;
	pTV->TVIF_ENDLINE_OFFSET_CTL = 0x88;

	pTV->TVENC_SYNC_CTL |= TV_ENC_SYNC_SW_RST;
	pTV->TVENC_MODE_CTL = TV_SVIDEO_CVBS_EN | TV_OUTPUT_CVBS_MODE | TV_CHROMA_UPSAMPLE_EN | TV_DELAY_INPUT_Y_ONE_PIX_2 | TV_LUMA_LPF_EN | TV_CHROMA_LPF_EN;
	pTV->TVENC_DAC_CONFIG = TV_DAS_Y_OUTPUT_ON | TV_DAX_C_OUTPUT_ON;
	pTV->TVENC_SYNC_CTL &= ~(TV_ENC_SYNC_SW_RST);
	pTV->TVENC_MODE_CTL = pTV->TVENC_MODE_CTL | TV_SETUP_751RE_EN | TV_714MV_286MV_MODE;
	pTV->TVENC_Y_SCALE_CTL = 0x00040060;

	uldata = pTV->TVENC_Y_SCALE_CTL;
	uldata = (uldata & 0xFEFFFF00) | (0x60 & 0xFF);
	uldata = (0x60 & 0x100) ? (uldata | 0x01000000) : uldata;
	pTV->TVENC_Y_SCALE_CTL = uldata;

	uldata = (0x40 << 24) | (0x40 << 16) | (0x40 << 8) | (0x40);
	pTV->TVENC_U_SCALE_CTL = uldata;
	uldata = (0x5a << 24) | (0x5a << 16) | (0x5a << 8) | (0x5a);
	pTV->TVENC_V_SCALE_CTL = uldata;
	uldata = (0x5a << 24) | (0x5a << 16) | (0x40 << 8) | (0x40);
	pTV->TVENC_UV_SCALE_GAIN_4_5 = uldata;

	pTV->TVENC_GAMMA_COEF_0 = 0x0;

	//enable
	pTV->TVIF_IF_EN = pTV->TVIF_IF_EN | TV_ENC_IF_EN;

	return 0;
}

static int init_display_interface_for_TV(struct ait_fb_display_info *aitfb, int type, int windows)
{
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	switch(windows)
	{
	case DIS_WIN_PIP:
		//disable RGB interface 
		pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | TV_FIEND_SYNC_EN;
		pDSPY->DSPY_RGB_FMT = pDSPY->DSPY_RGB_FMT & ~(DSPY_RGB_SYNC_MODE_DIS);
		pDSPY->DSPY_RGB_CTL = pDSPY->DSPY_RGB_CTL & ~(RGB_IF_EN);
		while(pDSPY->DSPY_RGB_CTL & RGB_IF_EN);
		pDSPY->DSPY_RGB_SHARE_P_LCD_BUS = RGBLCD_SRC_SEL_RGB;

		//change display settings for tv 
		pDSPY->DSPY_BG_COLOR = 0x00;
		pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 & ~(DSPY_PRM_SEL_MASK);
		pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | DSPY_PRM_SEL(DSPY_TYPE_TV);
		pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | DSPY_PRM_EN;
		pDSPY->DSPY_H = aitfb->fb.var.yres >> 1;
		pDSPY->DSPY_PIXL_CNT = aitfb->fb.var.xres * (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_CTL_4 = pDSPY->DSPY_CTL_4 & ~(LCD_OUT_SEL_MASK);

		//change Latch to be interlace
		pDSPY->DSPY_PIP_CTL_2 = (pDSPY->DSPY_PIP_CTL_2 & ~(WIN_FIFO_INTERLACE_LATCH)) | WIN_FIFO_INTERLACE_LATCH;
		pDSPY->DSPY_PIP_TV_EVEN_FIELD_ST = aitfb->fb.var.xres * aitfb->dis_w_pixel_size;

		//change PIP window size and use scale to output RGB888
		pDSPY->DSPY_PIP_H = aitfb->fb.var.yres >> 1;
		pDSPY->DSPY_PIP_PIXL_CNT = aitfb->fb.var.xres * (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_PIP_SCAL_CTL = DSPY_SCAL_NM | DSPY_SCAL_WT_DYN;
		pDSPY->DSPY_PIP_SOUT_CTL = DSPY_SOUT_RGB_888 | DSPY_SOUT_RGB | DSPY_SOUT_RGB_EN;
		pDSPY->DSPY_PIP_SCAL_H_N = 1;
		pDSPY->DSPY_PIP_SCAL_H_M = 1;
		pDSPY->DSPY_PIP_SCAL_V_N = 1;
		pDSPY->DSPY_PIP_SCAL_V_M = 1;
		pDSPY->DSPY_PIP_SCAL_H_WT = pDSPY->DSPY_PIP_SCAL_H_WT & ~(DSPY_SCAL_WT_AVG);
		pDSPY->DSPY_PIP_SCAL_V_WT = pDSPY->DSPY_PIP_SCAL_V_WT & ~(DSPY_SCAL_WT_AVG);
		pDSPY->DSPY_PIP_SCA_IN_W = aitfb->fb.var.xres;
		pDSPY->DSPY_PIP_SCA_IN_H = (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_PIP_SOUT_GRAB_H_ST = 1;
		pDSPY->DSPY_PIP_SOUT_GRAB_H_ED = aitfb->fb.var.xres;
		pDSPY->DSPY_PIP_SOUT_GRAB_V_ST = 1;
		pDSPY->DSPY_PIP_SOUT_GRAB_V_ED = (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_SOUT_GRAB_PIXL_CNT = aitfb->fb.var.xres * (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_PIP_CTL_2 = (pDSPY->DSPY_PIP_CTL_2 & ~(WIN_DUP_MASK)) | WIN_V_1X | WIN_H_1X;

		//setup tv
		setup_tv_interface(aitfb, type);
		break;
	case DIS_WIN_OVLY:
		//change Latch to be interlace
		pDSPY->DSPY_OVLY_CTL_2 = (pDSPY->DSPY_OVLY_CTL_2 & ~(WIN_FIFO_INTERLACE_LATCH)) | WIN_FIFO_INTERLACE_LATCH;
		pDSPY->DSPY_OVLY_TV_EVEN_FIELD_ST = aitfb->fb.var.xres * aitfb->dis_w_pixel_size;

		//change OVLY window size and use scale to output RGB888
		pDSPY->DSPY_OVLY_H = aitfb->fb.var.yres >> 1;
		pDSPY->DSPY_OVLY_PIXL_CNT = aitfb->fb.var.xres * (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_OVLY_SCAL_CTL = DSPY_SCAL_NM | DSPY_SCAL_WT_DYN;
		pDSPY->DSPY_OVLY_SOUT_CTL = DSPY_SOUT_RGB_888 | DSPY_SOUT_RGB | DSPY_SOUT_RGB_EN;
		pDSPY->DSPY_OVLY_SCAL_H_N = 1;
		pDSPY->DSPY_OVLY_SCAL_H_M = 1;
		pDSPY->DSPY_OVLY_SCAL_V_N = 1;
		pDSPY->DSPY_OVLY_SCAL_V_M = 1;
		pDSPY->DSPY_OVLY_SCAL_H_WT = pDSPY->DSPY_OVLY_SCAL_H_WT & ~(DSPY_SCAL_WT_AVG);
		pDSPY->DSPY_OVLY_SCAL_V_WT = pDSPY->DSPY_OVLY_SCAL_V_WT & ~(DSPY_SCAL_WT_AVG);
		pDSPY->DSPY_OVLY_SCA_IN_W = aitfb->fb.var.xres;
		pDSPY->DSPY_OVLY_SCA_IN_H = (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_OVLY_SOUT_GRAB_H_ST = 1;
		pDSPY->DSPY_OVLY_SOUT_GRAB_H_ED = aitfb->fb.var.xres;
		pDSPY->DSPY_OVLY_SOUT_GRAB_V_ST = 1;
		pDSPY->DSPY_OVLY_SOUT_GRAB_V_ED = (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_OVLY_SCA_OUT_PIXL_CNT = aitfb->fb.var.xres * (aitfb->fb.var.yres >> 1);
		pDSPY->DSPY_OVLY_CTL_2 = (pDSPY->DSPY_OVLY_CTL_2 & ~(WIN_DUP_MASK)) | WIN_V_1X | WIN_H_1X;

		//Setup TP(color key) settings for OVLY layer
		pDSPY->DSPY_OVLY_TP.MP.B_COLOR = 0;
		pDSPY->DSPY_OVLY_TP.MP.G_COLOR = 0;
		pDSPY->DSPY_OVLY_TP.MP.R_COLOR = 0;
		pDSPY->DSPY_OVLY_TP_CTL = pDSPY->DSPY_OVLY_TP_CTL | WIN_TP_EN;
		break;
	default:
		printk("ERROR: %d window does not support TV out\n", windows);
		break;
	}

	return 0;
}

static int init_aitfb_display(struct ait_fb_display_info *aitfb)
{
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	//enable register settins
	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 & ~(PRM_DSPY_REG_READY);

	//set all window to be primary
#if defined(CONFIG_FB_OVERLAY_ENABLE)
	pDSPY->DSPY_WIN_PRIO = (OVLY_WIN << WIN_1_SHFT) | (PIP_WIN << WIN_2_SHFT) | (ICON_WIN << WIN_3_SHFT) | (MAIN_WIN << WIN_4_SHFT);
#else
	pDSPY->DSPY_WIN_PRIO = (PIP_WIN << WIN_1_SHFT) | (OVLY_WIN << WIN_2_SHFT) | (ICON_WIN << WIN_3_SHFT) | (MAIN_WIN << WIN_4_SHFT);
#endif
	pDSPY->DSPY_WIN_BIND_SEL_MP = (pDSPY->DSPY_WIN_BIND_SEL_MP & 0x00) | (PRM_SEL << MAIN_WIN_SHIFT) | (PRM_SEL << PIP_WIN_SHIFT) | 
					(PRM_SEL << OVLY_WIN_SHIFT) | (PRM_SEL << OSD_WIN_SHIFT);

	//PIP windows will be the main windows so display output will base on PIP windows
	if(aitfb->dis_w_enable == DIS_WIN_PIP)
	{
		pDSPY->DSPY_W = aitfb->fb.var.xres;
		pDSPY->DSPY_H = aitfb->fb.var.yres;
		pDSPY->DSPY_PIXL_CNT = aitfb->fb.var.xres * aitfb->fb.var.yres;
	}

	//initial windows 
	switch(aitfb->dis_w_enable)
	{
	case DIS_WIN_MAIN:
		//not support.
		break;
	case DIS_WIN_PIP:
		pDSPY->DSPY_PIP_ADDR_ST = aitfb->fb.fix.smem_start;
		pDSPY->DSPY_PIP_CTL = pDSPY->DSPY_PIP_CTL & ~(WIN_YUV420_INTERLEAVE_MASK);
		pDSPY->DSPY_PIP_FMT = aitfb->dis_w_format;
		pDSPY->DSPY_PIP_OFST_ST = 0;
		pDSPY->DSPY_PIP_OFST_PIXL = aitfb->dis_w_pixel_size;
		pDSPY->DSPY_PIP_OFST_ROW = aitfb->fb.var.xres * aitfb->dis_w_pixel_size;
		pDSPY->DSPY_PIP_CTL = pDSPY->DSPY_PIP_CTL & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN);
		pDSPY->DSPY_PIP_CTL_2 = (pDSPY->DSPY_PIP_CTL_2 & ~(WIN_DUP_MASK)) | WIN_V_1X | WIN_H_1X;
		pDSPY->DSPY_PIP_X = 0;
		pDSPY->DSPY_PIP_Y = 0;
		pDSPY->DSPY_PIP_W = aitfb->fb.var.xres;
		pDSPY->DSPY_PIP_H = aitfb->fb.var.yres;
		pDSPY->DSPY_PIP_PIXL_CNT = aitfb->fb.var.xres * aitfb->fb.var.yres;
		pDSPY->DSPY_PIP_CTL = pDSPY->DSPY_PIP_CTL | WIN_EN;
		break;
	case DIS_WIN_OVLY:
		pDSPY->DSPY_OVLY_ADDR_ST = aitfb->fb.fix.smem_start;
		pDSPY->DSPY_OVLY_CTL = pDSPY->DSPY_OVLY_CTL & ~(WIN_YUV420_INTERLEAVE_MASK);
		pDSPY->DSPY_OVLY_FMT = aitfb->dis_w_format;
		pDSPY->DSPY_OVLY_OFST_ST = 0;
		pDSPY->DSPY_OVLY_OFST_PIXL = aitfb->dis_w_pixel_size;
		pDSPY->DSPY_OVLY_OFST_ROW = aitfb->fb.var.xres * aitfb->dis_w_pixel_size;
		pDSPY->DSPY_OVLY_CTL = pDSPY->DSPY_OVLY_CTL & ~(WIN_ROTATE_EN | WIN_SRC_GRAB_EN);
		pDSPY->DSPY_OVLY_CTL_2 = (pDSPY->DSPY_OVLY_CTL_2 & ~(WIN_DUP_MASK)) | WIN_V_1X | WIN_H_1X;
		pDSPY->DSPY_OVLY_X = 0;
		pDSPY->DSPY_OVLY_Y = 0;
		pDSPY->DSPY_OVLY_W = aitfb->fb.var.xres;
		pDSPY->DSPY_OVLY_H = aitfb->fb.var.yres;
		pDSPY->DSPY_OVLY_PIXL_CNT = aitfb->fb.var.xres * aitfb->fb.var.yres;
		pDSPY->DSPY_OVLY_CTL = pDSPY->DSPY_OVLY_CTL | WIN_EN;
		break;
	case DIS_WIN_ICON:
		//not support
		break;
	default:
		printk("ERROR:unknown display windows = 0x%x\n", aitfb->dis_w_enable);
	}

	//initial display output
	switch(aitfb->dis_output_type)
	{
	case DIS_OUT_TYPE_NTSC:
		init_display_interface_for_TV(aitfb, DIS_OUT_TYPE_NTSC, aitfb->dis_w_enable);
		break;
	case DIS_OUT_TYPE_PAL:
		init_display_interface_for_TV(aitfb, DIS_OUT_TYPE_PAL, aitfb->dis_w_enable);
		break;
	default:
		printk("ERROR:unknown display output type = 0x%x\n", aitfb->dis_output_type);
	}

	//enable primary display 
	pDSPY->DSPY_CTL_2 = pDSPY->DSPY_CTL_2 | PRM_DSPY_REG_READY;
	
	return 0;
}
/*
 *  DIS_WIN_PIP	:fb0
 *  DIS_WIN_OVLY:fb1
 */
static int aitfb_display_setup(struct ait_fb_display_info *aitfb)
{
	char *options = NULL;
	//char *this_opt;

	//We need to enable clock before anything
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_TV, MMP_TRUE);
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_DSPY, MMP_TRUE);
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_HDMI, MMP_TRUE);

	switch(aitfb->dev->id)
	{
	case 0:
		fb_get_options("aitfb",&options);

		if(options != NULL)
		{

		}else{
			printk("Use default settings: TV NTSC output\n");
			//setup aitfb to be ntsc tv
			aitfb->fb.var.xres      	= 720;
			aitfb->fb.var.yres      	= 480;
			aitfb->fb.var.xres_virtual	= aitfb->fb.var.xres;
			aitfb->fb.var.yres_virtual	= aitfb->fb.var.yres*2;
			aitfb->fb.var.bits_per_pixel	= 16;
			aitfb->fb.var.grayscale		= 0;
			aitfb->fb.var.pixclock    	= 27 * 1000 * 1000;
			aitfb->fb.var.left_margin	= 0;
			aitfb->fb.var.right_margin	= 0;
			aitfb->fb.var.upper_margin	= 0;
			aitfb->fb.var.lower_margin	= 0;
			aitfb->fb.var.hsync_len		= 0;
			aitfb->fb.var.vsync_len		= 0;
			aitfb->fb.var.sync		= 0;
			aitfb->fb.var.vmode		= 0;
			aitfb->fb.var.activate		= FB_ACTIVATE_NOW;
			aitfb->fb.var.accel_flags	= 0;
			aitfb->dis_output_type		= DIS_OUT_TYPE_NTSC;
			aitfb->dis_w_format		= WIN_YUV422;
			aitfb->dis_w_pixel_size		= 2; //YUV422
		}
		aitfb->dis_w_enable		= DIS_WIN_PIP;
		break;
	case 1:
		aitfb->fb.var.xres      	= 720;
		aitfb->fb.var.yres      	= 480;
		aitfb->fb.var.xres_virtual	= aitfb->fb.var.xres;
		aitfb->fb.var.yres_virtual	= aitfb->fb.var.yres*2;
		aitfb->fb.var.bits_per_pixel	= 16;
		aitfb->fb.var.grayscale		= 0;
		aitfb->fb.var.pixclock    	= 27 * 1000 * 1000;
		aitfb->fb.var.left_margin	= 0;
		aitfb->fb.var.right_margin	= 0;
		aitfb->fb.var.upper_margin	= 0;
		aitfb->fb.var.lower_margin	= 0;
		aitfb->fb.var.hsync_len		= 0;
		aitfb->fb.var.vsync_len		= 0;
		aitfb->fb.var.sync		= 0;
		aitfb->fb.var.vmode		= 0;
		aitfb->fb.var.activate		= FB_ACTIVATE_NOW;
		aitfb->fb.var.accel_flags	= 0;
		aitfb->dis_output_type		= DIS_OUT_TYPE_NTSC;	//overlay mutst set to 
		aitfb->dis_w_format		= WIN_16BPP;
		aitfb->dis_w_pixel_size		= 2; //RGB565
		aitfb->dis_w_enable		= DIS_WIN_OVLY;
		break;
	default:
		printk("ERROR: not support layer %d\n", aitfb->dev->id);
	}

	return 0;
}

static irqreturn_t ait_fb_display_irq(int irq, void *dev_id)
{
	struct ait_fb_display_info *aitfb = (struct ait_fb_display_info *)dev_id;
	AITPS_DSPY pDSPY = aitfb->fb_reg;

	fb_dbg("%x %x", pDSPY->DSPY_INT_CPU_EN, pDSPY->DSPY_INT_CPU_SR);

	return IRQ_HANDLED;
}

static struct fb_ops ait_fb_display_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= ait_fb_display_check_var,
	.fb_set_par	= ait_fb_display_set_par,
	.fb_setcolreg   = NULL,
	.fb_blank	= NULL,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_pan_display	= ait_fb_display_pan_display,
	.fb_ioctl	= ait_fb_display_ioctl,
};

static int ait_fb_probe(struct platform_device *dev)
{
	struct ait_fb_display_info *aitfb;
	struct resource *r;
	int dspy_irq = -1;

	fb_dbg("%s +++\n", __func__);

	aitfb = (struct ait_fb_display_info *)kmalloc(sizeof(struct ait_fb_display_info), GFP_KERNEL);
	if(aitfb == NULL)
	{
		printk("ERROR: %s can't allocate memory\n", __func__);
		return -ENOMEM;
	}

	if(dev->id == 0)
	{
		r = platform_get_resource(dev, IORESOURCE_MEM, 0);
		if(r == NULL)
		{
			printk("ERROR: Can't get lcd IO mem resource\n");
			return -ENODEV;
		}

		r = request_mem_region(r->start, resource_size(r), dev->name);
        	if(r == NULL)
        	{
                	printk("ERROR: Can't request CPU mem resource\n");
                	return -EBUSY;
        	}

		fb_reg_temp = ioremap(r->start, resource_size(r));
        	if(fb_reg_temp == NULL)
        	{
                	printk("ERROR: Can't remap io resource\n");
                	return -ENODEV;
        	}

		dspy_irq = platform_get_irq(dev, 0);
		if(dspy_irq < 0)
		{
			printk("ERROR: Can'get dspy irq\n");
			return -ENODEV;
		}

		MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DSPY, MMP_FALSE);
	}

	memset(aitfb, 0, sizeof(struct ait_fb_display_info));
	aitfb->fb_reg 			= (AITPS_DSPY)fb_reg_temp;
	aitfb->tv_reg 			= (AITPS_TV)((unsigned char *)(fb_reg_temp) + 0x70);
	aitfb->fb.fbops 		= &ait_fb_display_ops;
	aitfb->fb.flags 		= FBINFO_FLAG_DEFAULT;
	aitfb->fb.pseudo_palette	= kmalloc(SZ_4K, GFP_KERNEL);
	strncpy(aitfb->fb.fix.id, DEVICE_NAME_FB, sizeof(aitfb->fb.fix.id));
	aitfb->dev = dev;

	aitfb_display_setup(aitfb);

	aitfb->fb.fix.type		= FB_TYPE_PACKED_PIXELS;
	aitfb->fb.fix.type_aux		= 0;
	aitfb->fb.fix.xpanstep		= 0;
	aitfb->fb.fix.ypanstep		= 1;
	aitfb->fb.fix.ywrapstep		= 0;
	aitfb->fb.fix.accel		= FB_ACCEL_NONE;
	aitfb->fb.fix.mmio_start = r->start;
	aitfb->fb.fix.mmio_len = resource_size(r);
	aitfb->fb.fix.smem_len = aitfb->fb.var.xres * aitfb->fb.var.yres * (aitfb->fb.var.bits_per_pixel/8) * 2;
	aitfb->fb.fix.line_length = aitfb->fb.var.xres_virtual * (aitfb->fb.var.bits_per_pixel/8);
	aitfb->fb.screen_base = (char __iomem *)dma_alloc_writecombine(&(dev->dev), (size_t)aitfb->fb.fix.smem_len, 
					(dma_addr_t *)&(aitfb->fb.fix.smem_start), GFP_KERNEL);
	if(aitfb->fb.screen_base == NULL)
	{
		printk("ERROR: Can't allocate memory for LCD\n");
		return -ENOMEM;
	}else{
		printk("Video buffer addr %x %x\n", (unsigned int)aitfb->fb.screen_base, (unsigned int)aitfb->fb.fix.smem_start);
	}

	init_aitfb_display(aitfb);

	if(dev->id == 0)
	{
		if(request_irq(dspy_irq, ait_fb_display_irq, 0, DEVICE_NAME_FB, aitfb))
		{
			printk("ERROR: Can't request irq = %d\n", dspy_irq);
			return -EIO;
		}
	}

	platform_set_drvdata(dev, aitfb);

	if(register_framebuffer(&aitfb->fb) != 0 )
	{
		printk("ERROR: register frame buffer error\n");
	}

#ifdef FB_DEBUG
	dump_dspy(aitfb);
	dump_tv(aitfb);
#endif

	fb_dbg("%s ---\n", __func__);

	return 0;
}

static int ait_fb_remove(struct platform_device *dev)
{

	return 0;
}

static struct platform_driver ait_fb_driver = {
	.probe		= ait_fb_probe,
	.remove		= ait_fb_remove,
//	.suspend	= ait_fb_suspend,
//	.resume		= ait_fb_resume,
	.driver		= {
		.name	= DEVICE_NAME_FB,
		.owner  = THIS_MODULE,
	},
};

static int __init ait_fb_init(void)
{
	return platform_driver_register(&ait_fb_driver);
}

static void __exit ait_fb_exit(void)
{
	platform_driver_unregister(&ait_fb_driver);
}

module_init(ait_fb_init);
module_exit(ait_fb_exit);
