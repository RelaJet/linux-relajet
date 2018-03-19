/*
 * linux/arch/arm/mach-at91/board-ait8455evb.c
 *
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2006 Atmel
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

#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/bootmem.h>
#include <linux/interrupt.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>

#include <mach/system_rev.h>

#include "generic.h"

#include <mach/mmpf_system.h>

#include <mach/mmpf_pio.h>
#include <mach/mmpf_sd.h>
#include <mach/mmp_reg_pad.h>
#include <mach/mmp_reg_vif.h>
#include <mach/mmpf_vif.h>

static void ait_init_early_uart()
{
	/* aituart#0 -> ttyS0*/
 	ait_register_uart(0,0,-1); 	
 
	/* aituart#1 -> ttyS3*/
#if defined(CONFIG_SERIAL_AIT_PORT_1)
	ait_register_uart(1,3,-1); 	
#endif

	/* aituart#2 -> ttyS2*/
#if defined(CONFIG_SERIAL_AIT_PORT_2)
	ait_register_uart(2,2,-1); 	 
#endif

	/* aituart#3 -> ttyS1*/
#if defined(CONFIG_SERIAL_AIT_PORT_3)
	ait_register_uart(3,1-1); 	 
#endif

#if defined(CONFIG_AIT_EARLY_USART0) || defined(AT91_EARLY_DBGU) //select /dev/ttyS0 as early uart port 
	ait_set_serial_console(0); //set default console ttyS1 	
#elif defined(CONFIG_AIT_EARLY_USART1) //select /dev/ttyS1 as early uart port 
	ait_set_serial_console(1); //set default console ttyS1 
#elif defined(CONFIG_AIT_EARLY_USART2) //select /dev/ttyS2 as early uart port 
	ait_set_serial_console(2); //set default console ttyS1 
#elif defined(CONFIG_AIT_EARLY_USART3) //select /dev/ttyS3 as early uart port 
	ait_set_serial_console(3); //set default console ttyS1 
#endif 

}

static void __init ait_init_early(void)
{
 	AITPS_PAD pPAD = AITC_BASE_PAD;

//	pPAD->PAD_IO_CFG_PLCD[0] &= ~2;
	pPAD->PAD_IO_CFG_PLCD[0] = 0x44;	
	
	/* Initialize processor: 12 MHz crystal */
	ait_soc_initialize(12000000);

	ait_init_early_uart();	
}

/*
 * USB Device port
 */
static struct ait_udc_data __initdata ek_udc_data = {
	//.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

struct ait_otg_platform_data ait_otg_data = {
	.vbus_pin = 0,		    /* high == host powering us */
	.vbus_active_low = 0,	/* vbus polarity */
	.vbus_polled = 0,		/* Use polling, not interrupt */
	.pullup_pin = 0,		    /* active == D+ pulled up */
	.pullup_active_low = 0	/* true == pullup_pin is active low */

};



/*
 * SPI devices.
 */
#if defined(CONFIG_SPI_AIT) 
static struct spi_board_info ait8428_evb_pspi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "spidev",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33000000,// * 1000 * 1000,
		.bus_num	= 1,
		.mode	= SPI_MODE_0,		//For AX88796C
		.irq = gpio_to_irq(AIT_GPIO_ETHERNET_IRQ),
	},

};

static struct spi_board_info ait_pspi1_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "m25p80",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 1,
		.mode	= SPI_MODE_0,		
	},
#if 0	
	{	/* DataFlash chip */
		.modalias	= "spidev",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 0,
		.mode	= SPI_MODE_0,		
	},
#endif	
};

static struct spi_board_info ait_pspi3_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "m25p80",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 3,
		.mode	= SPI_MODE_0,		
	},
#if 0	
	{	/* DataFlash chip */
		.modalias	= "spidev",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 0,
		.mode	= SPI_MODE_0,		
	},
#endif	
};

static struct spi_board_info ait_sif_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "m25p80",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 0,
		.mode	= SPI_MODE_0,		
	},
#if 0	
	{	/* DataFlash chip */
		.modalias	= "spidev",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 0,
		.mode	= SPI_MODE_0,		
	},
#endif	
};
#endif


/*
 * MCI (SD/MMC)
 */
#if defined(CONFIG_MMC_AIT_IF0)
static struct ait_sd_data ait_sd0_data_sd = {
	.controller_id  = 0,
	.wire4          = 1,
	.bus_width	= 4,
	.pad_id         = MMPF_SD0_PAD0,

	.det_pin = 0,//AIT_GPIO_SD_DET_IRQ,
	.wp_pin = 0,		    /* (SD) writeprotect detect */
	.vcc_pin = 0,	    /* power switching (high == on) */
	.max_clk_rate	 = 52000000

};
#endif

#if defined(CONFIG_MMC_AIT_IF1)
static struct ait_sd_data ait_sd1_data_emmc = {
	.controller_id  = 1,
	.wire4          = 1,
	.bus_width	= 8,
	.pad_id         = MMPF_SD1_PAD1,
	.det_pin = 0,	
	.wp_pin = 0,		    /* (SD) writeprotect detect */
	.vcc_pin = 0,	    /* power switching (high == on) */
	.max_clk_rate	 = 25000000
};
#endif

#if defined(CONFIG_MMC_AIT_IF2)
static struct ait_sd_data ait_sd2_data_sdio = {
	.controller_id  = 2,
	.wire4          = 1,
	.bus_width	= 4,	
	.pad_id         = MMPF_SD2_PAD0,
	.det_pin = 0,	
	.wp_pin = 0,		    /* (SD) writeprotect detect */
	.vcc_pin = 0,	    /* power switching (high == on) */
	.max_clk_rate	 = 25000000
};
#endif

/*
 * I2C devices
 */
static struct at24_platform_data at24c512 = {
	.byte_len	= SZ_512K / 8,
	.page_size	= 128,
	.flags		= AT24_FLAG_ADDR16,
};

static struct i2c_board_info __initdata ek_phi2c_devices[] = {
	{
		I2C_BOARD_INFO("wm8973", 0x1A),
		.platform_data = &at24c512,
	},
	/* more devices can be added using expansion connectors */
};

/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_buttons[] = {
	{
		.gpio		= AT91_PIN_PA30,
		.code		= BTN_3,
		.desc		= "Button 3",
		.active_low	= 1,
		.wakeup		= 1,
	},
	{
		.gpio		= AT91_PIN_PA31,
		.code		= BTN_4,
		.desc		= "Button 4",
		.active_low	= 1,
		.wakeup		= 1,
	}
};

static struct gpio_keys_platform_data ek_button_data = {
	.buttons	= ek_buttons,
	.nbuttons	= ARRAY_SIZE(ek_buttons),
};

static struct platform_device ek_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ek_button_data,
	}
};

static void __init ek_add_device_buttons(void)
{
	platform_device_register(&ek_button_device);

}
#endif


static struct platform_device ait_camera_device = {
	.name   = "ait-cam",
	.id     = -1,

};

static struct platform_device ait_virtual_camera_device = {
	.name   = "ait-virtual-cam",
	.id     = -1,

};


static void ait_power_on_eeprom(int on) ;
static struct at24_platform_data at24a64t ={
         .byte_len 		= 1024, //32,   	// eeprom size
         .page_size   = 32,      		    // page size
         .flags 			= AT24_FLAG_ADDR16,
};

static struct ait_i2c_platform_data ait_eeprom_info ={
    .tag = AIT_I2C_INFO_TAG,
    .reg_len  = 16,
    .data_len = 8,    
    .power_on = ait_power_on_eeprom ,
};

static struct i2c_board_info __initdata i2c_bus0_device_info[] = {
    	{//device
        	I2C_BOARD_INFO("eeprom", 0x50),
        	//.platform_data = &at24a64t, // turn off at24 eeprom driver
        	.platform_data = &ait_eeprom_info, // connect to ait i2c driver
    	}
};

static struct ait_i2c_platform_data ait_pmu_info ={
    .tag = AIT_I2C_INFO_TAG,
    .reg_len  = 8,
    .data_len = 8,    
};


static struct i2c_board_info __initdata i2c_bus2_device_info[] = {
    {
        	I2C_BOARD_INFO("pmu", 0x12),
        	.platform_data = &ait_pmu_info,
    }   
};

static void peripheral_reset(void)
{
	AITPS_PAD   pPAD = AITC_BASE_PAD;
	pPAD->PAD_IO_CFG_PCGPIO[26] &= ~0x02; //CTS PAD: No Pull
	pPAD->PAD_IO_CFG_PCGPIO[29] &= ~0x02; //Rx  PAD: No Pull
	pPAD->PAD_IO_CFG_PCGPIO[28] &= ~0x02; //Tx  PAD: No Pull
	pPAD->PAD_IO_CFG_PCGPIO[28] |= 0x04; //Tx  PAD: pull high
	ait_set_gpio_output(AIT_GPIO_WIFI_RESET,1);	//wifi on	
	ait_set_gpio_input(AIT_GPIO_BT_CTX,1);		//bluetooth on, Host
	ait_set_gpio_output(AIT_GPIO_BT_RTX,0);		//bluetooth on, Host
}

static void ait_power_on_eeprom(int on) 
{
    
    AITPS_GBL   pGBL  = AITC_BASE_GBL;
    #if 1
    MMPF_SYS_EnableClock(MMPF_SYS_CLK_VIF, MMP_TRUE);
    pGBL->GBL_MISC_IO_CFG |= GBL_VIF0_GPIO_EN;
    MMPF_VIF_SetPIODir(0, VIF_SIF_SEN, MMP_TRUE);
    MMPF_VIF_SetPIOOutput(0, VIF_SIF_SEN,on?MMP_TRUE:MMP_FALSE);
    //MMPF_SYS_EnableClock(MMPF_SYS_CLK_VIF, MMP_FALSE);
    #endif
}
/*
implement gpio power off
*/
static void ait_power_off(void)
{
    // power off
    dbg_printf(0,"PWR_HOLD(gpio:%d):off\r\n",AIT_GPIO_POWER_HOLD);
    ait_set_gpio_output(AIT_GPIO_POWER_HOLD,0);        
}

static void ait_power_off_hook(void)
{
    pm_power_off = ait_power_off ;
}

static void __init ait_board_init(void)
{
	pr_info("## Board Device Initial ##");
			
	/* Serial */
	ait_add_device_serial();

	//ait_add_device_rtc();
	
	ait_add_otg(&ait_otg_data );

	peripheral_reset(); //reset wifi and bluetooth 
	ait_power_off_hook();

#if defined(CONFIG_SPI_AIT) 
	/* SPI */
	ait_add_device_sif(ait_sif_devices, ARRAY_SIZE(ait_sif_devices));
	ait_add_device_pspi(ait8428_evb_pspi_devices, ARRAY_SIZE(ait8428_evb_pspi_devices),0,2);
#endif
	/* MMC */
    #if defined(CONFIG_MMC_AIT_IF0)
	ait_add_device_mmc(0, &ait_sd0_data_sd);
    #endif
    #if defined(CONFIG_MMC_AIT_IF1)
	ait_add_device_mmc(1, &ait_sd1_data_emmc);
    #endif
    #if defined(CONFIG_MMC_AIT_IF2)
	ait_add_device_mmc(2, &ait_sd2_data_sdio);
    #endif

	/* I2C */
	//ait_add_device_phi2c(ek_phi2c_devices, ARRAY_SIZE(ek_phi2c_devices));
	ait_add_adapter_i2c(); //add i2c bus
  	// eeprom i2c0_pad0
	ait_add_device_i2c(i2c_bus0_device_info,ARRAY_SIZE(i2c_bus0_device_info),0); //add device to I2C bus0
  	// pmu
 	 ait_add_device_i2c(i2c_bus2_device_info,ARRAY_SIZE(i2c_bus2_device_info),2); //add device to I2C bus2

	ait_add_device_aud(0);
	ait_add_device_aud(1);
	platform_device_register(&ait_camera_device);


	//IRQ test 
	irq_set_irq_wake(PIN_BASE+112,1);
	//enable_irq_wake(PIN_BASE+112);
	
}


/*Vin: Modify for matching unmber which pass from u-boot MACHINE_START(AT91SAM9260EK, "Atmel AT91SAM9260-EK")*/
MACHINE_START(AT91SAM9X5EK, "AIT MCRV2 AIT8428 EVB")
	.timer      = &ait_pit_timer,
	.map_io     = ait_soc_map_io,
	.init_early = ait_init_early,            ///< .init_early() -> .init_irq() -> .init_machine()
	.init_irq   = ait_soc_init_irq_default,
	.init_machine = ait_board_init,
MACHINE_END
