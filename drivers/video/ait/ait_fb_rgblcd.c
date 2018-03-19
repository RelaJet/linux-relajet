#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <video/ait_fb_fops.h>
#include <mach/board.h>
#include <mach/lcd_common.h>
#include <mach/mmp_register.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_display.h>
#include <mach/mmpf_pio.h>
#include <mach/mmpf_system.h>

static int rgblcd_fb_ioctl(struct fb_info *fbinfo, unsigned int cmd, unsigned long arg)
{
	struct ait_fb_info *fbi = fbinfo->par;
	struct fb_ioc_layer_info layer;
	struct fb_ioc_address_info addr;
	struct fb_ioc_position_info pos;
	struct fb_ioc_order_info order;
	struct fb_ioc_alpha_info alpha;
	struct fb_ioc_chromakey_info chroma;
	int ret;

	ret = _checkIoctlData(cmd, arg);
	if (ret < 0) {
		dev_err(fbi->dev, "fail to data check for cl_fb cmd (%d)\n", cmd);
		return ret;
	}

	switch (cmd) 
	{
		case FB_IOC_REQUEST_LAYER:
			copy_from_user((void *)&layer, (const void *)arg, sizeof(struct fb_ioc_layer_info));
			ret = fb_create_mix_layer(fbinfo, &layer);
			break;
		case FB_IOC_RELEASE_LAYER:
			ret = fb_free_mix_layer(fbinfo, (int)arg);
			break;
		case FB_IOC_SET_LAYER_ADDRESS:
			copy_from_user((void *)&addr, (const void *)arg, sizeof(struct fb_ioc_address_info));				
			ret = fb_set_mix_layer_address(fbinfo, &addr);
			break;
		case FB_IOC_SET_LAYER_POSITION:
			copy_from_user((void *)&pos, (const void *)arg, sizeof(struct fb_ioc_position_info));
			//ret = fb_set_mix_layer_position(fbinfo, &pos);
			break;
		case FB_IOC_SET_LAYER_ORDER:
			copy_from_user((void *)&order, (const void *)arg, sizeof(struct fb_ioc_order_info));	
			ret = fb_set_mix_layer_order(fbinfo, &order);
			break;
		case FB_IOC_SET_LAYER_ALPHABLEND:
			copy_from_user((void *)&alpha, (const void *)arg, sizeof(struct fb_ioc_alpha_info));	
			//ret = fb_set_mix_layer_alphablend(fbinfo, &alpha);
			break;
		case FB_IOC_GET_LAYER_ALPHABLEND:
			//ret = fb_get_mix_layer_alphablend(fbinfo, (int)arg);
			break;
		case FB_IOC_SET_LAYER_CHROMAKEY:
			copy_from_user((void *)&chroma, (const void *)arg, sizeof(struct fb_ioc_chromakey_info));
			//ret = fb_set_mix_layer_chromakey(fbinfo, &chroma);
			break;
		case FB_IOC_SET_DISPLAY_DEV:
			//ret = _control_display_screen(fbinfo, (int)arg);
			break;
		default:
			dev_err(fbi->dev, "invalid IOCTL command (num:%d)\n", _IOC_NR(cmd));
			return -EINVAL;
	}

	return ret;
}

/*
 * IRQ handler for the RGB LCD
 */
static irqreturn_t rgblcd_fb_irq(int irq, void *dev_id)
{
#if 0
	int rgbif = (int)((int *)dev_id);
	pr_info(" >> %s %d (0x%x 0x%x) \n", __func__, rgbif, DSPY_RD_B(AITC_BASE_DSPY->DSPY_INT_HOST_SR), DSPY_RD_B(AITC_BASE_DSPY->DSPY_INT_CPU_SR));

	if (rgbif == MMPF_RGB_IF1)
	{
		if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) & RGB_IF_EN)
		{
			if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_INT_HOST_SR) & RGB_VSYNC_ACTIVE)
			{
				DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_HOST_SR, (RGB_VSYNC_ACTIVE | PRM_IDX_TX_END));
				DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_CPU_SR, (RGB_VSYNC_ACTIVE | PRM_IDX_TX_END));

				fb_refresh_display_screen_block(dev_id);
				return IRQ_HANDLED;
			}

		}
	}
	else if (rgbif == MMPF_RGB_IF2)
	{
		if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB2_CTL) & RGB_IF_EN)
		{
    	    if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_INT_HOST_SR) & RGB_VSYNC_ACTIVE)
			{
				DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_HOST_SR, (RGB_VSYNC_ACTIVE | PRM_IDX_TX_END));
				DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_CPU_SR, (RGB_VSYNC_ACTIVE | PRM_IDX_TX_END));
				return IRQ_HANDLED;
			}
        }
	}

	return IRQ_NONE;
#else
	struct fb_info *fbinfo = ((struct fb_info *)dev_id);

	if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_RGB_CTL) & RGB_IF_EN)
	{
		   if (DSPY_RD_B(AITC_BASE_DSPY->DSPY_INT_CPU_SR) & RGB_FRME_CNT_HIT)
		   {
				//DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_HOST_SR, (RGB_VSYNC_ACTIVE | PRM_IDX_TX_END | RGB_FRME_CNT_HIT));
				DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_CPU_SR, (RGB_VSYNC_ACTIVE | PRM_IDX_TX_END | RGB_FRME_CNT_HIT));
				fb_refresh_display_screen_block(fbinfo);
				DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_FRAM_CNT_CPU_INT, 1);
				return IRQ_HANDLED;
		   }

	}
	
	return IRQ_NONE;
#endif

}

static int rgblcd_fb_set_output(struct fb_info *fbinfo)
{
	MMPF_DISPLAY_CONTROLLER 	displayCtl = MMPF_DISPLAY_PRM_CTL;
	MMPF_DISPLAY_LCDATTRIBUTE 	lcdAttr;

	lcdAttr.usWidth = fbinfo->var.xres;
	lcdAttr.usHeight = fbinfo->var.yres;
	lcdAttr.colordepth = MMPF_LCD_COLORDEPTH_16;
	lcdAttr.ulBgColor = 0x0;
	lcdAttr.bFLMType = MMP_FALSE;

	MMPF_Display_SetRGBLCDOutput(displayCtl, &lcdAttr, MMPF_RGB_IF1);
	
	return 0;
}

static int rgblcd_fb_create_default_layer(struct fb_info *fbinfo)
{
	struct fb_ioc_layer_info layer;

	memset(&layer, 0x0, sizeof(struct fb_ioc_layer_info));
	layer.index = FB_DEFAULT_LAYER;
	layer.format = FB_RGB_FORMAT_RGB565;
	layer.src.sx = 0;
	layer.src.sy = 0;
	layer.src.ex = fbinfo->var.xres;
	layer.src.ey = fbinfo->var.yres;
	layer.src.width = fbinfo->var.xres;
	layer.src.height = fbinfo->var.yres;
	layer.dst.sx = 0;
	layer.dst.sy = 0;
	layer.dst.ex = fbinfo->var.xres;
	layer.dst.ey = fbinfo->var.yres;
	layer.dst.width = fbinfo->var.xres;
	layer.dst.height = fbinfo->var.yres;
	layer.addr.Y = (unsigned int)fbinfo->fix.smem_start;
	layer.addr.is_user_addr = 0;

	fb_create_mix_layer(fbinfo, &layer);
	
	return 0;
}

static struct irqaction rgblcd_fb_irqaction = {
	.name		= "rgblcd_fb_irq",
	.flags		= IRQF_SHARED | IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.dev_id     = MMPF_RGB_IF1,
	.handler	= rgblcd_fb_irq
};

static struct fb_ops rgblcd_fb_ops = {
	.owner		    = THIS_MODULE,
	.fb_check_var	= fb_check_var,
	.fb_set_par	    = fb_set_par,
	.fb_setcolreg	= fb_setcolreg,
	.fb_mmap		= fb_mmap,
	.fb_read        = fb_sys_read,
	.fb_write       = fb_sys_write,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_ioctl		= rgblcd_fb_ioctl,
};

static int rgblcd_fb_probe(struct platform_device *pdev)
{
	struct fb_info *fbinfo = NULL;
	struct ait_fb_platform_data_info *plat;
	struct ait_fb_info *fbi;
	struct fb_ioc_order_info order;
	
	int ret;

	if (!pdev || !pdev->dev.platform_data) {
		return -EINVAL;
	}
	
	plat = pdev->dev.platform_data;
	dev_info(&pdev->dev, "name: %s (w: %d, h: %d)\n", plat->disp->name, plat->disp->panel->usPanelW, plat->disp->panel->usPanelH);
	
	fbinfo = framebuffer_alloc(sizeof(struct ait_fb_info), &pdev->dev);
	if (!fbinfo) {
		dev_err(&pdev->dev, "fail to allocate framebuffer instance\n");
		return -ENOMEM;
    }
	platform_set_drvdata(pdev, fbinfo);
	fbi = fbinfo->par;
	fbi->dev = &pdev->dev;
	fbi->plat = plat;

	ret = fb_init_frambuffer_info(fbinfo, pdev->dev.platform_data);
	if (ret < 0) {	return ret; }
	fbinfo->fbops = &rgblcd_fb_ops;

	ret = register_framebuffer(fbinfo);
	if (ret < 0) {
		dev_err(&pdev->dev, "fail to register frame buffer device driver (ret=%d)\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "frame buffer probe done. (w: %d, h: %d)\n", fbinfo->var.xres, fbinfo->var.yres);

	/* init rgblcd pad config */
	GBL_WR_B(AITC_BASE_GBL->GBL_LCD_PAD_CFG, GBL_RD_B(AITC_BASE_GBL->GBL_LCD_PAD_CFG) & ~(GBL_LCD_RGB_SPI_MASK));
	GBL_WR_B(AITC_BASE_GBL->GBL_LCD_PAD_CFG, GBL_LCD_PAD_EN | GBL_LCD_RGB_SPI_PAD0);


	/* init lcd & driver ic */
	if (plat->disp->init) {
		plat->disp->init(&pdev->dev);
	}

	/* backlight enable */
	if (plat->disp->enable) {
		plat->disp->enable(&pdev->dev);
	}

	/* test lcd & driver ic */
	if (plat->disp->test) {
		//plat->disp->test(fbinfo->fix.smem_start, fbinfo->screen_base);
	}

	/* set up irq handler */
#if 0	
	setup_irq(AIC_SRC_DSPY, &rgblcd_fb_irqaction);
#else
	ret = request_irq(AIC_SRC_DSPY, rgblcd_fb_irq, IRQF_SHARED | IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL, "rgb_lcd_fb_irq", fbinfo);
	if (ret < 0) {
		dev_err(fbi->dev, "fail to request VSYNC IRQ(%d)\n", AIC_SRC_DSPY);
		return ret;
	}
#endif

	/* set output panel */
	rgblcd_fb_set_output(fbinfo);

	/* create fb default layer - for ui */
	//rgblcd_fb_create_default_layer(fbinfo);

	/* window priority */
	order.first = FB_ORDER_RGB0;
	order.second = FB_ORDER_VIDEO1;
	order.third = FB_ORDER_VIDEO0;
	order.fourth = FB_ORDER_DEF;
	fb_set_mix_layer_order(fbinfo, &order);

	/* enable vsync interrupt */
#if 0	
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_HOST_EN, RGB_VSYNC_ACTIVE);
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_CPU_EN, RGB_VSYNC_ACTIVE);
#else
	DSPY_WR_B(AITC_BASE_DSPY->DSPY_RGB_FRAM_CNT_CPU_INT, 1);
//	DSPY_WR_B(AITC_BASE_DSPY->DSPY_INT_CPU_EN, /*RGB_VSYNC_ACTIVE | */RGB_FRME_CNT_HIT);
#endif
	
	return 0;
}

static int rgblcd_fb_remove(struct platform_device *pdev)
{
	struct fb_info *fbinfo = platform_get_drvdata(pdev);
	
	dev_info(&pdev->dev, "%s", __func__);

	fb_free_frame_buffer_info(fbinfo);
	framebuffer_release(fbinfo);
	platform_set_drvdata(pdev, NULL);
	
	return 0;
}

static struct platform_driver rgblcd_fb_driver = {
	.probe		= rgblcd_fb_probe,
	.remove		= rgblcd_fb_remove,
	.driver		= {
		.name	= DEVICE_NAME_FB_RGBLCD,
		.owner	= THIS_MODULE,
	},
	.suspend	= NULL,
	.resume		= NULL,
};

int __init rgblcd_fb_init(void)
{
	int ret = platform_driver_register(&rgblcd_fb_driver);
	return ret;
}

static void __exit rgblcd_fb_exit(void)
{
	platform_driver_unregister(&rgblcd_fb_driver);
}

module_init(rgblcd_fb_init);
module_exit(rgblcd_fb_exit);
