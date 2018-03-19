/*
 * linux/arch/arm/mach-at91/board-ait8428evb.c
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


#include <mach/mmpf_pio.h>
#include <mach/mmpf_sd.h>
#include <mach/mmp_reg_pad.h>

static struct ait_uart_data ait_uart_custom_data[] = {
#if 1
	{
		.num = 0,
		.use_dma_tx	= 0,
		.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
		.uart_hw_id = MMPF_UART_ID_0, //0,
		.pad = 0,
		.baudrate = 115200,
		.fifo_size = 0x80,
		.hw_status = UART_UNINIT,
		.pUS = (AITPS_US) AITC_BASE_UART0,
	},
#endif	
#if defined(CONFIG_SERIAL_AIT_PORT_1	)
	{
		.num = 1,
		.use_dma_tx	= 0,
		.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
		.uart_hw_id = MMPF_UART_ID_1,
		//.uart_hw_id = MMPF_UART_ID_1,	
		
//		.padset = 0,  //PAD4 , pin assigned as below 
		/* ---------------------------------------------------------- */
        	/* | UART3 | TX         | RX          | CTS      | RTS      | */
        	/* ---------------------------------------------------------- */
       	 /* | PAD0  | PLCD13   | PLCD14    | N/A | N/A | */
		
		.baudrate = 115200,
		.fifo_size = 0x20,
		.hw_status = UART_UNINIT,
		.pUS = (AITPS_US) AITC_BASE_UART1,	//Only UART3 support CTS/RTS
		//.pUS = (AITPS_US) AITC_BASE_UART1,
	},	
#endif
#if defined(CONFIG_SERIAL_AIT_PORT_2	)
	{
		.num = 2,
		.use_dma_tx	= 0,
		.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
		.uart_hw_id = MMPF_UART_ID_2,
		//.uart_hw_id = MMPF_UART_ID_1,	
		
		.pad = 0,  //PAD4 , pin assigned as below 
		/* ---------------------------------------------------------- */
        	/* | UART3 | TX         | RX          | CTS      | RTS      | */
        	/* ---------------------------------------------------------- */
       	 /* | PAD0  | PLCD13   | PLCD14    | N/A | N/A | */
		
		.baudrate = 115200,
		.fifo_size = 0x20,
		.hw_status = UART_UNINIT,
		.pUS = (AITPS_US) AITC_BASE_UART2,	//Only UART3 support CTS/RTS
		//.pUS = (AITPS_US) AITC_BASE_UART1,
	},
#endif	
#if defined(CONFIG_SERIAL_AIT_PORT_3	)	
	{
		.num = 3,
		.use_dma_tx	= 0,
		.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
		.uart_hw_id = MMPF_UART_ID_3,
		//.uart_hw_id = MMPF_UART_ID_1,	
		
		.pad = 0,  //PAD0 , pin assigned as below 
		/* ---------------------------------------------------------- */
        	/* | UART3 | TX         | RX          | CTS      | RTS      | */
        	/* ---------------------------------------------------------- */
       	 /* | PAD0  | PCGPIO28   | PCGPIO29    | PCGPIO26 | PCGPIO27 | */
		
		.baudrate = 115200,
		.fifo_size = 0x20,
		.hw_status = UART_UNINIT,
		.pUS = (AITPS_US) AITC_BASE_UART3,	//Only UART3 support CTS/RTS
		//.pUS = (AITPS_US) AITC_BASE_UART1,
	}
#endif	
};


static void ait_init_early_uart()
{
	int i=0;
//pr_info("preferred_console = %d\n",preferred_console);

	for(i=0;i<ARRAY_SIZE(ait_uart_custom_data);++i)
		ait_register_uart(&ait_uart_custom_data[i]);	
	
	/* set serial console to ttyS0 (ie, DBGU) */
	ait_set_serial_console(0);
}
static void __init ait_init_early(void)
{
	AITPS_PAD pPAD = AITC_BASE_PAD;

	pPAD->PAD_IO_CFG_PLCD[0] = 0x44;	

	/* Initialize processor: 12 MHz crystal */
	ait_soc_initialize(12000000);
#ifdef __CHIKET_REMOVED__
	ait_init_leds(AIT_GPIO_CPU_LED, AIT_GPIO_TIMER_LED);
#endif // __CHIKET_REMOVED__

	ait_init_early_uart();


}

/*
 * USB Device port
 */
static struct ait_udc_data __initdata ek_udc_data = {
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
		.max_speed_hz	= 33 * 1000 * 1000,
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
		.chip_select	= 0,
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 3,
		.mode	= SPI_MODE_0,		
	},
};

static struct spi_board_info ait_sif_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "m25p80",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 0,
		.mode	= SPI_MODE_0,		
	},	
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

	.det_pin = 0,			//AIT_GPIO_SD_DET_IRQ,
	.wp_pin = 0,		    	/* (SD) writeprotect detect */
	.vcc_pin = 0,	    		/* power switching (high == on) */
	.max_clk_rate	 = 25000000

};
#endif

#if defined(CONFIG_MMC_AIT_IF1)
static struct ait_sd_data ait_sd1_data_emmc = {
	.controller_id  = 1,
	.wire4          = 1,
	.bus_width	= 8,
	.pad_id         = MMPF_SD1_PAD1,
	.det_pin = 0,	
	.wp_pin = 0,		    	/* (SD) writeprotect detect */
	.vcc_pin = 0,	   		 /* power switching (high == on) */
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
	.wp_pin = 0,		    	/* (SD) writeprotect detect */
	.vcc_pin = 0,	    		/* power switching (high == on) */
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
static struct gpio_keys_button ait_buttons[] = {
	{
		.gpio		= MMPF_PIO_REG_GPIO42,
		.code		= KEY_WAKEUP,
		.desc		= "Key Wake up",
		.debounce_interval = 20,
		.active_low	= 1,
		.wakeup		= 1,
	},
};

static struct gpio_keys_platform_data ait_button_data = {
	.buttons	= ait_buttons,
	.nbuttons	= ARRAY_SIZE(ait_buttons),
};

static struct platform_device ait_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ait_button_data,
	}
};

static void __init ait_add_device_buttons(void)
{
	platform_device_register(&ait_button_device);

}
#endif


static struct platform_device ait_camera_device = {
	.name   = "ait-cam",
	.id     = -1,

};

static struct platform_device ait_rtos_md_device = {
	.name   = "ait_md",
	.id     = -1,

};

static void __init ait_board_init(void)
{
	char *p;
	int i;
	
	pr_info("## 2OS CPUA Board Device Initial ##");
	/* Serial */
	ait_add_device_serial();
#ifdef __CHIKET_REMOVED__
//	ait_add_otg(&ait_otg_data );
	
#if defined(CONFIG_SPI_AIT) 
	/* SPI */
	ait_add_device_pspi(ait8428_evb_pspi_devices, ARRAY_SIZE(ait8428_evb_pspi_devices),0,2);
#endif
#endif // __CHIKET_REMOVED__

	/* MMC */
    #if defined(CONFIG_MMC_AIT_IF0)
	ait_add_device_mmc(0, &ait_sd0_data_sd);
    #endif
    #if defined(CONFIG_MMC_AIT_IF1)
	ait_add_device_mmc(1, &ait_sd1_data_emmc);
    #endif
    #if defined(CONFIG_MMC_AIT_IF2)
	//ait_add_device_mmc(2, &ait_sd2_data_sdio);
	
    #endif


    	/* CpuComm */
	ait_register_cpucomm();

	ait_add_device_cpub_mgr();
	
	platform_device_register(&ait_rtos_md_device);

	platform_device_register(&ait_camera_device);
//	ait_add_device_buttons();

}


MACHINE_START(AT91SAM9X5EK, "AIT VSNV3 AIT8455 EVB")
	.timer      = &ait_pit_timer,
	.map_io     = ait_soc_map_io,
	.init_early = ait_init_early,            ///< .init_early() -> .init_irq() -> .init_machine()
	.init_irq   = ait_soc_init_irq_default,
	.init_machine = ait_board_init,
MACHINE_END
