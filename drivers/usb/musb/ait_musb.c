/*
 * Copyright (C) 2014 Alpha Image Corp.
 * Vincent Chen 
 *
 * Based on omap2430.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <mach/mmpf_usbphy.h>
#include <mach/mmpf_system.h>
#include <mach/mmp_reg_usb.h>

#include "musb_core.h"

struct ait_usb_glue {
	struct device		*dev;
	struct platform_device	*musb;
	struct clk		*clk;
};
#define glue_to_musb(g)	platform_get_drvdata(g->musb)

static int ait_musb_init(struct musb *musb)
{

	extern MMP_ERR MMPF_USB_StopDevice(void);
	extern void MMPF_USBPHY_SetSignalTune(MMP_ULONG ulTxCur, MMP_ULONG ulSQ);
	extern void USBEx_DriveVBus(MMP_BOOL on);


	AITPS_GBL pGBL = AITC_BASE_GBL;
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;	
	
	if(musb->config->gpio_vbus_sw)
	{
		
		if (gpio_request(musb->config->gpio_vbus_sw, "USB_VBUS_SW")) {
			printk(KERN_ERR "Failed ro request USB_VRSEL GPIO_%d\n",
				musb->config->gpio_vbus_sw);
			return -ENODEV;
		}

		gpio_direction_output(musb->config->gpio_vbus_sw, 0);
	}
	usb_nop_xceiv_register();
	
	musb->xceiv = otg_get_transceiver();
	if (!musb->xceiv) {
		gpio_free(musb->config->gpio_vbus_sw);		
		pr_err("HS USB OTG: no transceiver configured\n");
		return -ENODEV;
	}

	pGBL->GBL_USB_CLK_SRC  = MMPF_PLL_ID_PMCLK |
								GBL_USB_ROOT_CLK_SEL_MCLK |
                         		USBPHY_CLK_SRC_PMCLK;
	pGBL->GBL_USB_CLK_DIV  = GRP_CLK_DIV_EN | GRP_CLK_DIV(1);
	
	MMPF_USBPHY_SetSignalTune(HS_CUR_440mV,SQ_LEVEL_62d5mV);	//TX CUR=440mV SQ=62.5mV	
	MMPF_USB_StopDevice();
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);
		//	USBEx_DriveVBus(1);
		
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_USB,MMP_TRUE);

		
	MMPF_USBPHY_PowerDown();

  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);


	if(musb->config->mode== MUSB_HOST)
	{
		MMPF_USBPHY_Initialize(MMP_FALSE);

		//disable OTG_EN, force enable OTG_AVALID, OTG_VBUSVALID
		pUSB_DMA->USB_OTG_CTL = (OTG_AVALID_TRUE | OTG_VBUSVALID_TRUE);
		pUSB_DMA->USB_OTG_CTL |= OTG_HOSTDISCONN_EN;

		/* Enable USB OTG, signal from PHY */
		//pUSB_DMA->USB_OTG_CTL = OTG_EN|OTG_HOSTDISCONN_EN;

		/* Connect DMPULLDOWN to PHY, must set */
		pUSB_DMA->USB_UTMI_PHY_CTL1 |= DMPULLDOWN_TO_PHY_EN;

	}else if(musb->config->mode== MUSB_PERIPHERAL)
	{
	//Device

	    MMPF_USBPHY_Initialize(0);

	    pUSB_DMA->USB_UTMI_PHY_CTL0 &= ~(CLKOSC_OFF_IN_SUSPEND);
	    pUSB_DMA->USB_UTMI_PHY_CTL0 |= UTMI_PLL_ON_IN_SUSPEND;
	    // remove DP/DM Pulldown
	    pUSB_DMA->USB_UTMI_CTL1 &= ~(UTMI_DPPULLDOWN | UTMI_DMPULLDOWN);
	    // disable USB testmode
	    pUSB_DMA->USB_UTMI_CTL1 &= ~(UTMI_USBPHY_TESTMODE);

	    pUSB_DMA->USB_UTMI_PHY_CTL1 = UTMI_OUTCLK_PLL | UTMI_DATA_BUS_8BIT;
	}
       else
       {
            pr_err("ait_musb_init: no musb mode configured\n");
       }

	gpio_set_value(musb->config->gpio_vbus_sw, 1);


	return 0;
}

static int ait_musb_exit(struct musb *musb)
{
	gpio_free(musb->config->gpio_vbus_sw);		

	otg_put_transceiver(musb->xceiv);

	return 0;
}


static void ait_musb_set_vbus(struct musb *musb, int is_on)
{
	int value = musb->config->gpio_vbus_sw;

	dev_info(musb->controller, "VBUS %s\n",is_on?"ON":"OFF");
	
	if(!value)
		return;
	
	if (!is_on)
		value = !value;
	gpio_set_value(musb->config->gpio_vbus_sw, value);

	dev_info(musb->controller, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb->xceiv->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static const struct musb_platform_ops ait_musb_ops = {
	.init		= ait_musb_init,
	.exit		= ait_musb_exit,


//	.enable		= bfin_musb_enable,
//	.disable	= bfin_musb_disable,
//	.set_mode	= bfin_musb_set_mode,
//	.try_idle	= bfin_musb_try_idle,
//	.vbus_status	= bfin_musb_vbus_status,
	.set_vbus	= ait_musb_set_vbus,

//	.adjust_channel_params = bfin_musb_adjust_channel_params,	
};

static int __init ait_musb_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = pdev->dev.platform_data;
	struct platform_device		*musb;
	struct ait_usb_glue		*glue;
	struct clk			*clk;

	int				ret = -ENOMEM;

	glue = kzalloc(sizeof(*glue), GFP_KERNEL);
	if (!glue) {
		dev_err(&pdev->dev, "failed to allocate glue context\n");
		goto err0;
	}

	musb = platform_device_alloc(musb_driver_name, -1);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		goto err1;
	}

	clk = clk_get(&pdev->dev, "udc_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(clk);
		goto err2;
	}

	ret = clk_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err3;
	}

	musb->dev.parent		= &pdev->dev;
	musb->dev.dma_mask		= pdev->dev.dma_mask;
	musb->dev.coherent_dma_mask	= pdev->dev.coherent_dma_mask;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->clk			= clk;
	
	pdata->platform_ops		= &ait_musb_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
			pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err4;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err4;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err4;
	}

	return 0;

err4:
	clk_disable(clk);

err3:
	clk_put(clk);

err2:
	platform_device_put(musb);

err1:
	kfree(glue);

err0:
	return ret;
}

static int __exit ait_musb_remove(struct platform_device *pdev)
{
	struct ait_usb_glue *glue = platform_get_drvdata(pdev);

	platform_device_del(glue->musb);
	platform_device_put(glue->musb);
	clk_disable(glue->clk);
	clk_put(glue->clk);
	kfree(glue);

	return 0;
}

#ifdef CONFIG_PM
static int ait_musb_suspend(struct device *dev)
{
	struct ait_usb_glue	*glue = dev_get_drvdata(dev);
	struct musb		*musb = glue_to_musb(glue);

	otg_set_suspend(musb->xceiv, 1);
	clk_disable(glue->clk);

	return 0;
}

static int ait_musb_resume(struct device *dev)
{
	struct ait_usb_glue *glue = dev_get_drvdata(dev);
	struct musb		*musb = glue_to_musb(glue);
	int			ret;

	ret = clk_enable(glue->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	otg_set_suspend(musb->xceiv, 0);

	return 0;
}

static const struct dev_pm_ops ait_musb_pm_ops = {
	.suspend	= ait_musb_suspend,
	.resume		= ait_musb_resume,
};

#define DEV_PM_OPS	(&ait_musb_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static struct platform_driver ait_musb_driver = {
	.remove		= __exit_p(ait_musb_remove),
	.driver		= {
		.name	= "musb-ait",
		.pm	= DEV_PM_OPS,
	},
};


static int __init ait_usb_init(void)
{
	return platform_driver_probe(&ait_musb_driver, ait_musb_probe);
}
subsys_initcall(ait_usb_init);

static void __exit ait_usb_exit(void)
{
	platform_driver_unregister(&ait_musb_driver);
}
module_exit(ait_usb_exit);

MODULE_DESCRIPTION("AIT MUSB Glue Layer");
MODULE_AUTHOR("Vincent Chen");
MODULE_LICENSE("GPL v2");

