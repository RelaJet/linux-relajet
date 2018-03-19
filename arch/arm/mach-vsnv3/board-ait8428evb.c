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
#if 0
static void ait_init_early_uart()
{
	/* aituart#0 -> ttyS0*/
 	ait_register_uart(0,0,-1); 	
 
	/* aituart#1 -> ttyS1*/
#if defined(CONFIG_SERIAL_AIT_PORT_1)
	ait_register_uart(1,1,-1); 	
#endif

	/* aituart#2 -> ttyS2*/
#if defined(CONFIG_SERIAL_AIT_PORT_2)
	ait_register_uart(2,2,-1); 	 
#endif

	/* aituart#3 -> ttyS3*/
#if defined(CONFIG_SERIAL_AIT_PORT_3)
	ait_register_uart(3,3,-1); 	 
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
#endif
static void __init ait_init_early(void)
{
 	AITPS_PAD pPAD = AITC_BASE_PAD;

//	pPAD->PAD_IO_CFG_PLCD[0] &= ~2;
	pPAD->PAD_IO_CFG_PLCD[0] = 0x44;	
	
	/* Initialize processor: 12 MHz crystal */
	ait_soc_initialize(12000000);

  #ifdef CONFIG_MACH_MCRV2_AIT8428EVB
	ait_init_leds(AIT_GPIO_CPU_LED, AIT_GPIO_TIMER_LED);
  #endif

	/* DBGU on ttyS0. (Rx & Tx only) */
	ait_init_early_uart();	
	
	/* set serial console to ttyS0 (ie, DBGU) */
	//ait_set_serial_console(0);
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

#if 0
/*
 * MACB Ethernet device
 */
static struct ait_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= ,
	.is_rmii	= 1,
};

/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "Partition 1",
		.offset	= 0,
		.size	= SZ_256K,
	},
	{
		.name	= "Partition 2",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct ait_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
//	.det_pin	= ... not connected
	.rdy_pin	= ,
	.enable_pin	= ,
	.parts		= ek_nand_partition,
	.num_parts	= ARRAY_SIZE(ek_nand_partition),
};

static void __init ek_add_device_nand(void)
{
	ek_nand_data.bus_width_16 = board_have_nand_16bit();

	ait_add_device_nand(&ek_nand_data);
}
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

static void __init ait_board_init(void)
{
	pr_info("## Board Device Initial ##");
			
	/* Serial */
	ait_add_device_serial();

	//ait_add_device_rtc();
	
	ait_add_otg(&ait_otg_data );

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
	ait_add_device_aud(0);
	ait_add_device_aud(1);
	platform_device_register(&ait_camera_device);
			
}


/*Vin: Modify for matching unmber which pass from u-boot MACHINE_START(AT91SAM9260EK, "Atmel AT91SAM9260-EK")*/
MACHINE_START(AT91SAM9X5EK, "AIT MCRV2 AIT8428 EVB")
	.timer      = &ait_pit_timer,
	.map_io     = ait_soc_map_io,
	.init_early = ait_init_early,            ///< .init_early() -> .init_irq() -> .init_machine()
	.init_irq   = ait_soc_init_irq_default,
	.init_machine = ait_board_init,
MACHINE_END
