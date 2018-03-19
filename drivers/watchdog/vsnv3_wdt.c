/* /drivers/watchdog/vsnv3_wdt.c
 *
 * Copyright (c) 2013 Alpha Image Tech.
 *	Vincent Chen<vincent1103@a-i-t.com.tw>
 *
 *  AIT VSNV3 Watchdog Timer Support
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h> 
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/err.h>

//#include <mach/reg_retina.h>
//#include <mach/mmp_reg_gbl.h>

#include <mach/mmpf_wd.h>
#include <mach/mmpf_system.h>

#include "vsnv3_wdt.h"
#ifdef CONFIG_AIT_CHIP_MCR_V2
// this is not real time window,
// but try to use chip support max timeout window 
#define CONFIG_VSNV3_WATCHDOG_DEFAULT_TIME	(100) 
#else
#define CONFIG_VSNV3_WATCHDOG_DEFAULT_TIME	(4)
#endif

/* Timer heartbeat (500ms) */
#define WDT_TIMEOUT	(2*HZ)
#define WDT_COUNTDOWN_AFTER_PING 		20
static int nowayout	        = WATCHDOG_NOWAYOUT;
static int tmr_margin       = CONFIG_VSNV3_WATCHDOG_DEFAULT_TIME;
static int wdt_on_atboot    = 1;//CONFIG_S3C2410_WATCHDOG_ATBOOT;
static int wdt_reset;
static int debug;
static char wdt_stat = 0;
static int  tmp_counter=-1;

module_param(tmr_margin,  int, 0);
module_param(wdt_on_atboot,  int, 0);
module_param(nowayout,    int, 0);
module_param(wdt_reset, int, 0);
module_param(debug,	  int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. (default="
		__MODULE_STRING(CONFIG_VSNV3_WATCHDOG_DEFAULT_TIME) ")");

MODULE_PARM_DESC(wdt_on_atboot,"Watchdog is started at boot time if set to 1, default="
		__MODULE_STRING(CONFIG_S3C2410_WATCHDOG_ATBOOT));

MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
MODULE_PARM_DESC(wdt_reset, "Watchdog action, set to 1 to ignore reboots, "
			"0 to reboot (default 0)");
MODULE_PARM_DESC(debug, "Watchdog debug, set to >1 for debug (default 0)");

static struct device    *wdt_dev;	/* platform device attached to */
static struct resource	*wdt_irq;

static DEFINE_SPINLOCK(vsnv3_wdt_lock);

/* watchdog control routines */

#define WDT_DBG(msg...) do { \
	if (debug) \
		printk(KERN_INFO msg); \
	} while (0)


static unsigned long next_heartbeat;	/* the next_heartbeat for the timer */

static struct timer_list timer;	/* The timer that pings the watchdog */


static int ait_wdt_start(struct watchdog_device *wdd)
{
	WDT_DBG("%s: %s\n wdt_reset = %d\n", __func__, wdd->info->identity,wdt_reset);

	spin_lock(&vsnv3_wdt_lock);
    #if defined(CONFIG_ARCH_VSNV3)
	MMPF_WD_EnableWD(MMP_TRUE, !!wdt_reset, !wdt_reset, NULL, MMP_TRUE);
    #endif
	
	#if defined(CONFIG_ARCH_MCRV2)
	
	#if CONFIG_AIT_ENABLE_JTAG
	MMPF_WD_EnableWD(MMP_TRUE,MMP_FALSE,MMP_TRUE,0,MMP_TRUE); 
	#else
	MMPF_WD_EnableWD(MMP_TRUE,MMP_TRUE,MMP_FALSE,0,MMP_TRUE);
	AITPS_GBL   pGBL 	= AITC_BASE_GBL;
	pGBL->GBL_BOOT_STRAP_CTL = ROM_BOOT_MODE;
	#endif

    #endif
	
	wdt_stat = 1;
	
	tmp_counter=-1;

	spin_unlock(&vsnv3_wdt_lock);

	mod_timer(&timer, jiffies + WDT_TIMEOUT);

	return 0;
}

static int ait_wdt_stop(struct watchdog_device *wdd)
{
	WDT_DBG("%s: %s\n", __func__, wdd->info->identity);

	spin_lock(&vsnv3_wdt_lock);

    MMPF_WD_EnableWD(MMP_FALSE, MMP_FALSE, MMP_FALSE, NULL, MMP_FALSE);

    wdt_stat = 0;

	spin_unlock(&vsnv3_wdt_lock);

	return 0;
}

int timeout=0;
static void ait_wdt_kick(unsigned long data)
{
	WDT_DBG("%s: \n", __func__);

	spin_lock(&vsnv3_wdt_lock);

        MMPF_WD_Kick();
	
	if(tmp_counter<0)//if use space dose not enable watchdog , tmp_counter is -1
	  mod_timer(&timer, jiffies + WDT_TIMEOUT);
	else
	{//if use space enabled watchdog already , the value always > 0
	  tmp_counter--;
	  if(tmp_counter>0)
	    mod_timer(&timer, jiffies + WDT_TIMEOUT);
	  else //if (tmp_counter==0) the timer will not timeout at next,  
	    tmp_counter==0;
	  
	}

	spin_unlock(&vsnv3_wdt_lock);

	return;
}

void vsnv3_wdt_force_timeout(void)
{
	spin_lock(&vsnv3_wdt_lock);

	del_timer(&timer);

	spin_unlock(&vsnv3_wdt_lock);

	return;
}
EXPORT_SYMBOL(vsnv3_wdt_force_timeout);

static int ait_wdt_ping(struct watchdog_device *wdd)
{
	WDT_DBG("%s: %s\n", __func__, wdd->info->identity);

	spin_lock(&vsnv3_wdt_lock);

        MMPF_WD_Kick();
	
	tmp_counter=WDT_COUNTDOWN_AFTER_PING;
	
	mod_timer(&timer, jiffies + WDT_TIMEOUT);

	spin_unlock(&vsnv3_wdt_lock);

	return 0;
}

/*
 * Set the watchdog time interval in 1/256Hz (write-once)
 * Counter is 12 bit.
 */
static int ait_wdt_settimeout(struct watchdog_device *wdd, unsigned int timeout)
{
	MMP_USHORT usCounter = 31, usMclkDiv = 1024;

	if (unlikely(timeout<0))
		return -1;

	MMPF_WD_Kick();

	usCounter = (timeout * ((MMPF_WD_GetFreq()*1000)/usMclkDiv)) / 16384;
	if(usCounter > 31) {
	    usCounter = 31 ;
	    WDT_DBG("[AIT] : WD window=31(max)\n");
	}
	MMPF_WD_SetTimeOut(usCounter, usMclkDiv);

	return 0;
}


#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)

static const struct watchdog_info ait_wdt_ident = {
	.options          = OPTIONS,
	.firmware_version =	0,
	.identity         =	"VSNV3 Watchdog",
};

static struct watchdog_ops ait_wdt_ops = {
	.owner = THIS_MODULE,
	.start = ait_wdt_start,
	.stop = ait_wdt_stop,
	.ping = ait_wdt_ping,
	.set_timeout = ait_wdt_settimeout,
};

static struct watchdog_device ait_wdt_dev = {
	.info = &ait_wdt_ident,
	.ops = &ait_wdt_ops,
};

/* interrupt handler code */
static irqreturn_t ait_wdt_irq(int irqno, void *param)
{
	MMPF_SYS_ResetSystem(0);

	return IRQ_HANDLED;
}


#ifdef CONFIG_CPU_FREQ

static int ait_wdt_cpufreq_transition(struct notifier_block *nb,
					  unsigned long val, void *data)
{
	int ret;

	if (!wdt_stat)
		goto done;

	if (val == CPUFREQ_PRECHANGE) {
		/* To ensure that over the change we don't cause the
		 * watchdog to trigger, we perform an keep-alive if
		 * the watchdog is running.
		 */

		ait_wdt_ping(&ait_wdt_dev);
	} else if (val == CPUFREQ_POSTCHANGE) {
		ait_wdt_stop(&ait_wdt_dev);

		ret = ait_wdt_settimeout(&ait_wdt_dev, ait_wdt_dev.timeout);

		if (ret >= 0)
			ait_wdt_start(&ait_wdt_dev);
		else
			goto err;
	}

done:
	return 0;
err:
	return ret;
}

static struct notifier_block ait_wdt_cpufreq_transition_nb = {
	.notifier_call	= ait_wdt_cpufreq_transition,
};

static inline int ait_wdt_cpufreq_register(void)
{
	return cpufreq_register_notifier(&ait_wdt_cpufreq_transition_nb,
					 CPUFREQ_TRANSITION_NOTIFIER);
}

static inline void ait_wdt_cpufreq_deregister(void)
{
	cpufreq_unregister_notifier(&ait_wdt_cpufreq_transition_nb,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

#else
static inline int ait_wdt_cpufreq_register(void)
{
	return 0;
}

static inline void ait_wdt_cpufreq_deregister(void)
{
}
#endif

static int __devinit vsnv3_wdt_probe(struct platform_device *pdev)
{
	struct device *dev;
	int started = 0;
	int ret;

	WDT_DBG("%s: probe=%p\n", __func__, pdev->name);

	dev = &pdev->dev;
	wdt_dev = &pdev->dev;

	wdt_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (wdt_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err_map;
	}

	ret = request_irq(wdt_irq->start, ait_wdt_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_map;
	}

	if (ait_wdt_cpufreq_register() < 0) {
		printk(KERN_ERR "%s: failed to register cpufreq\n",dev_name(&pdev->dev));
		goto err_clk;
	}

	/* see if we can actually set the requested timer margin, and if
	 * not, try the default value */

	if (ait_wdt_settimeout(&ait_wdt_dev, tmr_margin)) {
			dev_info(dev, "timeout value out of range, default %d used\n",
			   CONFIG_VSNV3_WATCHDOG_DEFAULT_TIME);

	}

	ret = watchdog_register_device(&ait_wdt_dev);
	if (ret) {
		dev_err(dev, "cannot register watchdog (%d)\n", ret);
		goto err_cpufreq;
	}

    next_heartbeat = jiffies + CONFIG_VSNV3_WATCHDOG_DEFAULT_TIME * HZ;
    setup_timer(&timer, ait_wdt_kick, 0);

	if (wdt_on_atboot && started == 0) {
		dev_info(dev, "starting watchdog timer\n");
		ait_wdt_start(&ait_wdt_dev);

	} else if (!wdt_on_atboot) {
		/* if we're not enabling the watchdog, then ensure it is
		 * disabled if it has been left running from the bootloader
		 * or other source */

	    ait_wdt_stop(&ait_wdt_dev);
	}

	return 0;

err_cpufreq:
    ait_wdt_cpufreq_deregister();

err_clk:
	free_irq(wdt_irq->start, pdev);

err_map:

	return ret;
}

static int __devexit vsnv3_wdt_remove(struct platform_device *dev)
{
	watchdog_unregister_device(&ait_wdt_dev);

	ait_wdt_cpufreq_deregister();

	free_irq(wdt_irq->start, dev);
	wdt_irq = NULL;

	return 0;
}

static void vsnv3_wdt_shutdown(struct platform_device *dev)
{
    ait_wdt_stop(&ait_wdt_dev);
}

#ifdef CONFIG_PM
static int vsnv3_wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int vsnv3_wdt_resume(struct platform_device *dev)
{
	return 0;
}

#else
#define vsnv3_wdt_suspend NULL
#define vsnv3_wdt_resume  NULL
#endif /* CONFIG_PM */


static struct platform_driver vsnv3_wdt_driver = {
	.probe		= vsnv3_wdt_probe,
	.remove		= __devexit_p(vsnv3_wdt_remove),
	.shutdown	= vsnv3_wdt_shutdown,
	.suspend	= vsnv3_wdt_suspend,
	.resume		= vsnv3_wdt_resume,
	.driver		= {
		.name	= "vsnv3_wdt",
		.owner	= THIS_MODULE,
	},
};

static int __init watchdog_init(void)
{
	return platform_driver_register(&vsnv3_wdt_driver);
}

static void __exit watchdog_exit(void)
{
	platform_driver_unregister(&vsnv3_wdt_driver);
}

module_init(watchdog_init);
module_exit(watchdog_exit);


