/*
 * arch/arm/mach-ait/pm.c
 * AIT Power Management
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/gpio.h>
#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <linux/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#ifdef CONFIG_HAVE_TCM
#include <asm/tcm.h>
#endif
#include <mach/cpu.h>
#include <mach/mcrv2_gpio.h>
#include <mach/io.h>
#include "generic.h"
#include "pm.h"

#include <mach/mmpf_system.h>
#include <mach/mmp_reg_display.h>
#include <mach/mmp_reg_audio.h>
#include <mach/mmpf_mcrv2_audio_ctl.h>
#include <mach/mmpf_usbphy.h>
#include <mach/mmp_reg_vif.h>
#include <mach/mmp_reg_dram.h>
#include <mach/mmp_register.h>


#include <mach/mmp_reg_uart.h>

/*
 * Show the reason for the previous system reset.
 */
#if defined(AIT_SHDWC)

static void __init show_reset_status(void)
{

}
#else
static void __init show_reset_status(void) {}
#endif

static int ait_pm_valid_state(suspend_state_t state)
{
	printk(KERN_INFO"%s state = %d\n",__func__,state);
	return (state >= PM_SUSPEND_ON && state < PM_SUSPEND_MAX);
}


static suspend_state_t pm_state;

/*
 * Called after processes are frozen, but before we shutdown devices.
 */
static int ait_pm_begin(suspend_state_t state)
{

	printk(KERN_INFO"%s state = %d\n",__func__,state);
	pm_state = state;
	return 0;
}

/*
 * Verify that all the clocks are correct before entering
 * slow-clock mode.
 */
static int ait_pm_verify_clocks(void)
{
	return 1;
}
/*
 * Call this from platform driver suspend() to see how deeply to suspend.
 * For example, some controllers (like OHCI) need one of the PLL clocks
 * in order to act as a wakeup source, and those are not available when
 * going into slow clock mode.
 *
 * REVISIT: generalize as clk_will_be_available(clk)?  Other platforms have
 * the very same problem (but not using at91 main_clk), and it'd be better
 * to add one generic API rather than lots of platform-specific ones.
 */
 
int ait_suspend_entering_slow_clock(void)
{
	return (pm_state == PM_SUSPEND_MEM);
}
EXPORT_SYMBOL(ait_suspend_entering_slow_clock);
static void (*slow_clock)(void);


static u32 ait_wake_enable(unsigned char mask )
{
	AITPS_GBL pGBL = AITC_BASE_GBL;
	
	pGBL->GBL_WAKEUP_INT_HOST_EN |= mask;
	pGBL->GBL_WAKEUP_INT_CPU_EN |= mask;

printk(KERN_INFO"GBL_CLK_DIS = 0x%x   0x%x \n",pGBL->GBL_CLK_DIS[0],pGBL->GBL_CLK_DIS[1]);		
	return 0;
}



#ifdef TCM_TEST

/* Uninitialized data */
static u32 __tcmdata tcmvar;
/* Initialized data */
static u32 __tcmdata tcmassigned = 0x2BADBABEU;
/* Constant */
static const u32 __tcmconst tcmconst = 0xCAFEBABEU;

static void __tcmlocalfunc tcm_to_tcm(void)
{
	int i;
	for (i = 0; i < 100; i++)
		tcmvar ++;
}

static void __tcmfunc hello_tcm(void)
{
	/* Some abstract code that runs in ITCM */
	int i;
	for (i = 0; i < 100; i++) {
		tcmvar ++;
	}
	tcm_to_tcm();
}

//static inline u32 sdram_selfrefresh_enable(void)
inline u32 sdram_selfrefresh_enable(void)
{
	void* prt = ait_cpu_suspend;
	u32 *tcmem;
	int i;
	void* p = hello_tcm;
	printk(KERN_INFO"hello_tcm address = %x\n",p);

	hello_tcm();
	printk("Hello TCM executed from ITCM RAM\n");

	printk("TCM variable from testrun: %u @ %p\n", tcmvar, &tcmvar);
	tcmvar = 0xDEADBEEFU;
	printk("TCM variable: 0x%x @ %p\n", tcmvar, &tcmvar);

	printk("TCM assigned variable: 0x%x @ %p\n", tcmassigned, &tcmassigned);

	printk("TCM constant: 0x%x @ %p\n", tcmconst, &tcmconst);

	/* Allocate some TCM memory from the pool */
	tcmem = tcm_alloc(20);
	if (tcmem) {
		printk("TCM Allocated 20 bytes of TCM @ %p\n", tcmem);
		tcmem[0] = 0xDEADBEEFU;
		tcmem[1] = 0x2BADBABEU;
		tcmem[2] = 0xCAFEBABEU;
		tcmem[3] = 0xDEADBEEFU;
		tcmem[4] = 0x2BADBABEU;
		for (i = 0; i < 5; i++)
			printk("TCM tcmem[%d] = %08x\n", i, tcmem[i]);
		tcm_free(tcmem, 20);
	}

	
	//ait_sram_suspend = (void*)AIT_RAM_P2V(0x115A00);
	//ait_sram_push((void*)ait_sram_suspend,prt,ait_cpu_suspend_sz);
	//ait_sram_suspend(&ait_pm_config);	
}
#else

void MMPF_PIO_Enable(MMPF_PIO_REG piopin, MMP_BOOL bEnable)
{
    MMP_ULONG   ulBitPos = 1 << (piopin & PIO_BITPOSITION_INFO);//bit pos within 4-bytes
    MMP_UBYTE   ubIndex = PIO_GET_INDEX(piopin); 				//index of 4-byte
    AITPS_GBL   pGBL = AITC_BASE_GBL;

    if (bEnable)
        pGBL->GBL_GPIO_CFG[ubIndex] |= ulBitPos;
    else
        pGBL->GBL_GPIO_CFG[ubIndex] &= ~ulBitPos;
}




MMP_ERR MMPF_TV_PowerDownDAC(void)
{
	AITPS_TV pTV = AITC_BASE_TV;

	MMPF_SYS_EnableClock(MMPF_SYS_CLK_TV, MMP_TRUE);
	pTV->TVIF_DAC_IF_3RD_CTL &= ~(0x08);
	pTV->TVIF_IF_EN |= TV_IF_DAC_CTL;
	pTV->TVIF_DAC_IF_1ST_CTL = 0;
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_TV, MMP_FALSE);

	return MMP_ERR_NONE;
}


MMP_ERR MMPF_HDMI_PowerDownPHY(void)
{
    AITPS_HDMI_PHY pHDMI = AITC_BASE_HDMI_PHY;

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_HDMI, MMP_TRUE);

    #if (CHIP == P_V2)
    pHDMI->GBL_HDMI_PHY_TMDS_CTL0 |= (GBL_TDMS_BUF_POWER_DOWN | 0x01);
    pHDMI->GBL_HDMI_PHY_TMDS_CTL4 |= GBL_HDMI_PREEMP_GAIN_DATA_3d2DB;
    pHDMI->GBL_HDMI_PHY_TMDS_CTL5 |= GBL_HDMI_PREEMP_GAIN_CLK_3d2DB;
    RTNA_WAIT_MS(30);
    pHDMI->GBL_HDMI_PHY_PLL_CTL0 |= GBL_HDMI_PHY_POWER_DOWN_PLL;
    pHDMI->GBL_HDMI_PHY_BANDGAP_CTL0 |= (GBL_BG_POWER_DOWN | GBL_BS_POWER_DOWN);
    #endif
    #if (CHIP == MCR_V2)
    pHDMI->HDMI_PHY_TMDS_CTL[0] |= (TDMS_BUF_POWER_DOWN | 0x01);
    RTNA_WAIT_MS(30);
    pHDMI->HDMI_PHY_PLL_CTL[0] |= HDMI_PHY_PWR_DOWN_PLL;
    pHDMI->HDMI_PHY_BANDGAP_CTL |= (BG_PWR_DOWN | BS_PWR_DOWN);
    #endif

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_HDMI, MMP_FALSE);

    return MMP_ERR_NONE;
}


MMP_ERR MMPF_VIF_PowerDownMipiRx(void)
{
	AITPS_MIPI pMIPI = AITC_BASE_MIPI;

	MMPF_SYS_EnableClock(MMPF_SYS_CLK_VIF, MMP_TRUE);

	pMIPI->DATA_LANE[0].MIPI_DATA_CFG[0] &= ~(MIPI_DAT_LANE_EN);
	pMIPI->DATA_LANE[1].MIPI_DATA_CFG[0] &= ~(MIPI_DAT_LANE_EN);
	pMIPI->DATA_LANE[2].MIPI_DATA_CFG[0] &= ~(MIPI_DAT_LANE_EN);
	pMIPI->DATA_LANE[3].MIPI_DATA_CFG[0] &= ~(MIPI_DAT_LANE_EN);

	pMIPI->DATA_LANE[0].MIPI_DATA_CFG[1] &= ~(MIPI_DAT_LANE_EN);
	pMIPI->DATA_LANE[1].MIPI_DATA_CFG[1] &= ~(MIPI_DAT_LANE_EN);
	pMIPI->DATA_LANE[2].MIPI_DATA_CFG[1] &= ~(MIPI_DAT_LANE_EN);
	pMIPI->DATA_LANE[3].MIPI_DATA_CFG[1] &= ~(MIPI_DAT_LANE_EN);

	MMPF_SYS_EnableClock(MMPF_SYS_CLK_VIF, MMP_FALSE);

	return MMP_ERR_NONE;
}


#ifdef CONFIG_HAVE_TCM
__attribute__((optimize("O0")))
u32 __tcmfunc  sdram_selfrefresh_enable(void)
{
 
	static u32 __tcmdata ulClkDis[2], ofst, clk_bit, wait;
	static u32 __tcmdata irq_bckup[2];

	static AITPS_DRAM    __tcmdata pDRAM = AITC_BASE_DRAM;
	static AITPS_GBL    __tcmdata pGBL 	= AITC_BASE_GBL;  


	
        pGBL->GBL_LCD_BYPASS_CTL = (pGBL->GBL_LCD_BYPASS_CTL & ~(BYPASS_KEEP_PMCLK)) | BYPASS_AUTO_PWR_OFF_PLL | BYPASS_PWR_OFF_XTAL;

	/* Backup clock settings */
	ulClkDis[0] = pGBL->GBL_CLK_DIS[0];
	ulClkDis[1] = pGBL->GBL_CLK_DIS[1];
	pDRAM->DRAM_FUNC &= ~(DRAM_CLK_GATE_EN);  		// 6E18
	pDRAM->DRAM_CMD_CTL0 = DRAM_SRF_ST; 				// 6E30
	while((pDRAM->DRAM_INT_CPU_SR & DRAM_SRF_DONE) == 0);
	pDRAM->DRAM_INT_CPU_SR = DRAM_SRF_DONE; 			// 6E04, clear status
	pDRAM->DRAM_FUNC |= (DRAM_CLK_GATE_EN); 			// 6E18

	/* Disable most of modules' clock */
	pGBL->GBL_CLK_DIS[0] = ~(GBL_CLK_CPU_A | GBL_CLK_CPU_A_PHL);// | GBL_CLK_MCI);
	pGBL->GBL_CLK_DIS[1] = ~(GBL_CLK_GPIO);

    /* Enter sleep mode */
    pGBL->GBL_LCD_BYPASS_CTL |= BYPASS_TO_SELF_SLEEP;
    
    /* CPU need to execute NOP till clock stopped */
    for(wait = 0xFF; wait > 0; wait--) {}
	
	/* Exit sleep mode */
	pGBL->GBL_LCD_BYPASS_CTL &= ~(BYPASS_TO_SELF_SLEEP);

	/* Restore clock settings */
	ofst = 0;
	do {
	    clk_bit = 1 << ofst++;
	    
	    if ((ulClkDis[0] & clk_bit) == 0)
	        pGBL->GBL_CLK_EN[0] = clk_bit;
	    if ((ulClkDis[1] & clk_bit) == 0)
	        pGBL->GBL_CLK_EN[1] = clk_bit;
	        
	    /* insert some delay to avoid voltage drop */
	    for(wait = 0x1FF; wait > 0; wait--) {}
	} while(ofst < 32);

	pDRAM->DRAM_FUNC &= ~(DRAM_CLK_GATE_EN); 			// 6E18
	pDRAM->DRAM_CMD_CTL0 = DRAM_EXIT_SRF_ST; 			// 6E30
	while((pDRAM->DRAM_INT_CPU_SR & DRAM_EXIT_SRF_ST) == 0);
	pDRAM->DRAM_INT_CPU_SR = DRAM_EXIT_SRF_ST; 		// 6E04, clear status
	pDRAM->DRAM_FUNC |= (DRAM_CLK_GATE_EN); 			// 6E18

        pGBL->GBL_LCD_BYPASS_CTL = (pGBL->GBL_LCD_BYPASS_CTL | BYPASS_KEEP_PMCLK) &   ~(BYPASS_AUTO_PWR_OFF_PLL | BYPASS_PWR_OFF_XTAL);
		
	return 0;	
}
#else
__attribute__((optimize("O0")))
u32  sdram_selfrefresh_enable(void)
{
	return 0;
}

#endif
#endif
MMP_ERR MMPF_Audio_PowerDownCodec(void);

static int ait_pm_enter(suspend_state_t state)
{
	//u32 saved_lpr;
	vsnv3_gpio_suspend();
	ait_irq_suspend();

	pr_debug("AIT: PM - pm state %d\n",
			/* remember all the always-wake irqs */
			state);

	switch (state) {
		/*
		 * Suspend-to-RAM is like STANDBY plus slow clock mode, so
		 * drivers must suspend more deeply:  only the master clock
		 * controller may be using the main oscillator.
		 */
		case PM_SUSPEND_MEM:
			/*
			 * Ensure that clocks are in a valid state.
			 */
			if (!ait_pm_verify_clocks())
				goto error;

			/*
			 * Enter slow clock mode by switching over to clk32k and
			 * turning off the main oscillator; reverse on wakeup.
			 */
			if (slow_clock) {
#ifdef CONFIG_VSNV3_SLOW_CLOCK
				/* copy slow_clock handler to SRAM, and call it */
				memcpy(slow_clock, at91_slow_clock, at91_slow_clock_sz);
#endif
				slow_clock();
				break;
			} else {
				pr_info("AT91: PM - no slow clock mode enabled ...\n");
				/* FALLTHROUGH leaving master clock alone */
			}

		/*
		 * STANDBY mode has *all* drivers suspended; ignores irqs not
		 * marked as 'wakeup' event sources; and reduces DRAM power.
		 * But otherwise it's identical to PM_SUSPEND_ON:  cpu idle, and
		 * nothing fancy done with main or cpu clocks.
		 */
		case PM_SUSPEND_STANDBY:
			/*
			 * NOTE: the Wait-for-Interrupt instruction needs to be
			 * in icache so no SDRAM accesses are needed until the
			 * wakeup IRQ occurs and self-refresh is terminated.
			 * For ARM 926 based chips, this requirement is weaker
			 * as at91sam9 can access a RAM in self-refresh mode.
			 */
//			 ait_wake_enable(GPIO_WAKE_INT);
			/* Power down audio codec */
#ifdef CONFIG_SOUND
			MMPF_Audio_PowerDownCodec();
#endif			
			/* Power down TV DAC */
			MMPF_TV_PowerDownDAC();
#ifdef CONFIG_USB_MUSB_AIT						
			/* Power down USB PHY */
			MMPF_USBPHY_PowerDown();
#endif			
			/* Power down HDMI PHY */
			MMPF_HDMI_PowerDownPHY();
			/* Power down MIPI Rx */
			MMPF_VIF_PowerDownMipiRx();
			
			asm volatile (	"mov r0, #0\n\t"
					"b 1f\n\t"
					".align 5\n\t"
					"1: mcr p15, 0, r0, c7, c10, 4\n\t"
					: /* no output */
					: /* no input */
					: "r0");


			sdram_selfrefresh_enable();
			
			//wait_for_interrupt_enable();
		//	sdram_selfrefresh_disable();
			break;

		case PM_SUSPEND_ON:
			cpu_do_idle();
			break;

		default:
			pr_debug("AT91: PM - bogus suspend state %d\n", state);
			goto error;
	}

//	pr_debug("AT91: PM - wakeup %08x\n",
//			ait_sys_read(AT91_AIC_IPR) & ait_sys_read(AT91_AIC_IMR));

error:
	pm_state = PM_SUSPEND_ON;
	ait_irq_resume();	
	vsnv3_gpio_resume();
	
	return 0;
}

/*
 * Called right prior to thawing processes.
 */
static void ait_pm_end(void)
{
	printk(KERN_INFO"%s\n",__func__);
	pm_state = PM_SUSPEND_ON;
}


static const struct platform_suspend_ops ait_pm_ops = {
	.valid	= ait_pm_valid_state,
	.begin	= ait_pm_begin,
	.enter	= ait_pm_enter,
	.end	= ait_pm_end,
};

static int __init ait_pm_init(void)
{
	pr_info("AIT: Power Management\n");

	suspend_set_ops(&ait_pm_ops);

	show_reset_status();
	return 0;
}
arch_initcall(ait_pm_init);
