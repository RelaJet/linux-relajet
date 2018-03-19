/*
 * linux/arch/arm/mach-at91/board-ait8428g_hdk.c
 *
 *  Copyright (C) 2015 AIT
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
#include <mach/mmpf_system.h>


#define AIT_GPIO_ETHERNET_IRQ MMPF_PIO_REG_GPIO117 // 116 (AIT) , 117 (QD)
#define AIT_GPIO_SPIS_BUSY MMPF_PIO_REG_GPIO31

#define AIT_GPIO_SD_DET_IRQ MMPF_PIO_REG_GPIO38
#define AIT_GPIO_TIMER_LED          (MMPF_PIO_REG_GPIO59)
#define AIT_GPIO_USB_VBUS_SW (MMPF_PIO_REG_GPIO18) //18 (P), 119 (G)
//#define AIT_GPIO_CPU_LED            (MMPF_PIO_REG_GPIO4)

/* For Ethernet driver */
uint AIT_GPIO_ETHERNET_RESET = MMPF_PIO_REG_GPIO115;
EXPORT_SYMBOL(AIT_GPIO_ETHERNET_RESET);

/* for WIFI driver reset and OOB IRQ ping */
uint AIT_GPIO_WIFI_OOB_IRQ = MMPF_PIO_REG_GPIO42;
EXPORT_SYMBOL(AIT_GPIO_WIFI_OOB_IRQ);

uint AIT_GPIO_WIFI_RESET = MMPF_PIO_REG_GPIO53;
EXPORT_SYMBOL(AIT_GPIO_WIFI_RESET);

uint AIT_GPIO_PSPI_S2M_IRQ = MMPF_PIO_REG_GPIO115;
EXPORT_SYMBOL(AIT_GPIO_PSPI_S2M_IRQ);

static struct ait_uart_data ait_uart_custom_data[] = {

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


static void ait_init_early_uart(void)
{
	int i=0;

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
#if defined(CONFIG_MCRV2_AIT6366G_EVB)
	//ait_init_leds(AIT_GPIO_CPU_LED, AIT_GPIO_TIMER_LED);

#if defined(CONFIG_BCMDHD_1_88_45) || (CONFIG_BCMDHD_1_88_45_MODULE) || (CONFIG_BCMDHD)  || (CONFIG_BCMDHD_MODULE)
//  move to sdio.c in mmc/core ( reset after ocr failed
//	ait_set_gpio_output(AIT_GPIO_WIFI_RESET,0); //wifi on
//	mdelay(100);
#endif
#endif
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
	.gpio_vbus_sw = AIT_GPIO_USB_VBUS_SW,
	.vbus_active_low = 0,	/* vbus polarity */
	.vbus_polled = 0,		/* Use polling, not interrupt */
	.pullup_pin = 0,		    /* active == D+ pulled up */
	.pullup_active_low = 0	,/* true == pullup_pin is active low */
#ifdef CONFIG_USB_MUSB_OTG
	.mode = MUSB_OTG,
#elif defined(CONFIG_USB_DEVICE_AIT) || defined(CONFIG_USB_DEVICE_AIT_MODULE)
	.mode = MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_HDRC)||defined(CONFIG_USB_MUSB_AIT_MODULE)
	.mode = MUSB_HOST,
#endif
};



/*
 * SPI devices.
 */
#if defined(CONFIG_SPI_AIT_PSPI) || defined(CONFIG_SPI_AIT_PSPI_MODULE) || defined (CONFIG_SPI_AIT_FULL)
#if defined(CONFIG_IOT) && defined(CONFIG_IOT_MTK)
static struct iot_spi_platform_data iot_spi_platform_data = {
};

static struct spi_board_info ait8428_evb_pspi_devices[] = {
	{	/* Ethernet chip */
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
#if defined(CONFIG_MT7682)
		.modalias       = "spidev-iot-eth",
		.max_speed_hz	=48 * 1000 * 1000,
#else
		.modalias       = "spidev-mt7687",
		.max_speed_hz	=22 * 1000 * 1000,
#endif		
		.bus_num	= 1,
		.mode	= SPI_MODE_2,
		.irq = gpio_to_irq(AIT_GPIO_ETHERNET_IRQ),
		.platform_data = &iot_spi_platform_data,
	}

};
#if defined(CONFIG_SUPPORT_2WAY_SPI)

static struct iot_spi_platform_data iot_spi_platform_data = {
	.s2m_irq_gpio = MMPF_PIO_REG_GPIO115, //AIT_GPIO_PSPI_S2M_IRQ,
};


static struct spi_board_info ait8428_evb_pspi_slave_devices[] = {
	{	/* Ethernet chip */
		.modalias	= "spidev-mt7687-slave",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	=22 * 1000 * 1000,
		.bus_num	= 2,
		.mode	= SPI_MODE_0,
		.irq = gpio_to_irq(AIT_GPIO_ETHERNET_IRQ),
		.platform_data = &iot_spi_platform_data,
		
	}

};
#endif
#elif defined(CONFIG_CC3200)

static const struct iot_spi_platform_data iot_spi_platform_data = {
//	.s2m_irq_gpio = MMPF_PIO_REG_GPIO115, //AIT_GPIO_PSPI_S2M_IRQ,
	.s2m_spi_busy_gpio = AIT_GPIO_SPIS_BUSY
};

static struct spi_board_info ait8428_evb_pspi_devices[] = {
	{	/* Ethernet chip */
		.modalias	= "spidev-iot-cc3200",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	=20* 1000*1000,
		.bus_num	= 1,
		.mode	= SPI_MODE_0,
		.irq = gpio_to_irq(AIT_GPIO_ETHERNET_IRQ),
		.platform_data = &iot_spi_platform_data,		
	}

};

#else
static struct spi_board_info ait8428_evb_pspi_devices[] = {
	{	/* Ethernet chip */
		.modalias	= "spidev",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
		.max_speed_hz	=22 * 1000 * 1000,
		.bus_num	= 1,
		.mode	= SPI_MODE_2,
		.irq = gpio_to_irq(AIT_GPIO_ETHERNET_IRQ),
	}

};
#endif

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
#endif

#if defined(CONFIG_SPI_AIT) || defined(CONFIG_SPI_AIT_SIF) || defined(CONFIG_SPI_AIT_SIF_MODULE) || defined (CONFIG_SPI_AIT_FULL)
static struct spi_board_info ait_sif_devices[] = {
#ifndef CONFIG_MTD_NAND_AIT_SPINAND
	{	/* DataFlash chip */
		.modalias	= "m25p80",
		.chip_select	= 0,	// Must less than 	master->num_chipselect in sif driver probe.
#ifdef CONFIG_AIT_FAST_BOOT
		.max_speed_hz	= 66 * 1000 * 1000,
#else
		.max_speed_hz	= 33 * 1000 * 1000,
#endif
		.bus_num	= 0,
		.mode	= SPI_MODE_0,		
	},	
#else
	{
		.modalias	= "ait_spinand",
		.chip_select	= 0,
		.max_speed_hz	= 33 * 1000 * 1000,
		.bus_num	= 0,
		.mode		= SPI_MODE_0,
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

	.det_pin = AIT_GPIO_SD_DET_IRQ,                 //AIT_GPIO_SD_DET_IRQ,
	.wp_pin = 0,		    	/* (SD) writeprotect detect */
	.vcc_pin = 0,	    		/* power switching (high == on) */
	.max_clk_rate	 = 50000000, //25000000,
	.active_low = 1,
	.disable_sdio = 1
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
	.max_clk_rate	 = 50000000//25000000
};
#endif

static struct gpio_led gpio_leds[] = {
         {
                 .name                   = "LED-STATUS",
                 .gpio                   = MMPF_PIO_REG_GPIO4,
                 .active_low             = 0,
                 .default_trigger        = "none",
                 .default_state          = LEDS_GPIO_DEFSTATE_KEEP,
         },
         {
                 .name                   = "LED-WIFI",
                 .gpio                   = MMPF_PIO_REG_GPIO6,
                 .active_low             = 0,
                 .default_trigger        = "none",
                 .default_state          = LEDS_GPIO_DEFSTATE_KEEP,
         },

         {
                 .name                   = "LED-POWER",
                 .gpio                   = MMPF_PIO_REG_GPIO3,
                 .active_low             = 0,
                 .default_trigger        = "none",
                 .default_state          = LEDS_GPIO_DEFSTATE_KEEP,
         },

 };

 
#if 0
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
#else
struct ait_i2c_platform_data wm8971_i2c_data = 
{
	.tag = AIT_I2C_INFO_TAG,
	.reg_len = 8,
	.data_len = 8,    

};

struct ait_i2c_platform_data rt5627_i2c_data = 
{
	.tag = AIT_I2C_INFO_TAG,
	.reg_len = 8,
	.data_len = 16,    

};

static struct i2c_board_info __initdata mcrv2_phi2c_devices[] = {
#if 1	
	{
//		I2C_BOARD_INFO("wm8737", 0x1A), 
		I2C_BOARD_INFO("wm8973", 0x1A), 
//		.platform_data = &wm8971_i2c_data,

	},
#endif	
	{
		I2C_BOARD_INFO("rt5627", 0x18),
		.platform_data = &rt5627_i2c_data,
	},
	{
		I2C_BOARD_INFO("rt5627", 0x19),
		.platform_data = &rt5627_i2c_data,
	},

		
};
#endif

/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ait_buttons[] = {
	{
		.gpio		= MMPF_PIO_REG_GPIO40, //ori is MMPF_PIO_REG_GPIO42
		.code		= KEY_WAKEUP,
		.desc		= "Key Wake up",
		.debounce_interval = 20,
		.active_low	= 1,
		.wakeup		= 1,
	},
	{
		.gpio		= MMPF_PIO_REG_GPIO86,
		.code		= KEY_RESTART,
		.desc		= "Button Reset",
		.debounce_interval = 20,
		.active_low	= 1,
		.wakeup		= 0,
	},	
	{
		.gpio		= MMPF_PIO_REG_GPIO101,
		.code		= KEY_MODE,
		.desc		= "KEY_MODE",
		.debounce_interval = 20,
		#ifdef CONFIG_AIT_FAST_BOOT
		.active_low	= 0,
		#else
		.active_low	= 1,
		#endif
		.wakeup		= 0,
	},
  {  /* for capture */
          .gpio           = MMPF_PIO_REG_GPIO85,
          .code           = KEY_CAMERA,
          .desc           = "KEY_CAMERA",
          .debounce_interval = 20,
          .active_low     = 1,
          .wakeup         = 0,
  },
	#ifdef CONFIG_AIT_FAST_BOOT
	
	#if 0//ndef CONFIG_CC3200
  {  /* for power down */
          .gpio           = MMPF_PIO_REG_GPIO40,
          .code           = KEY_POWER,
          .desc           = "KEY_POWER",
          .debounce_interval = 20,
          .active_low     = 1,
          .wakeup         = 0,
  },
  #endif
  #if 1
  {  /* for PIR */
          .gpio           = MMPF_PIO_REG_GPIO56,
          .code           = KEY_LEFTALT,
          .desc           = "KEY_PIR",
          .debounce_interval = 20,
          .active_low     = 1,
          .wakeup         = 0,
  },
  #endif
  
  {  /* for DoorBell */
          .gpio           = MMPF_PIO_REG_GPIO57,
          .code           = KEY_SPACE,
          .desc           = "KEY_DOORBELL",
          .debounce_interval = 20,
          .active_low     = 1,
          .wakeup         = 0,
  },
  #endif
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


#if 0
static struct resource ait_cam_resources[] = {
    [0] = {
        .name = "ait-cam-memory",
        .start  = 0x4400000,
        .end    = 0x4400000 + CONFIG_AIT_VIDEO_RESERVED_MBYTE*1024*1024-1,
        .flags  = IORESOURCE_MEM,
    },

};
#endif

static struct platform_device ait_virtual_camera_device = {
	.name   = "ait-virtual-cam",
	.id     = -1,

};

// ait-md driver
static struct platform_device ait_rtos_md_device = {
	.name   = "ait_md",
	.id     = -1,

};

// ait-aac driver
static struct platform_device ait_rtos_aac_device = {
	.name   = "ait_aac",
	.id     = -1,
};

// ait-aes driver
static struct platform_device ait_rtos_aes_device = {
	.name   = "ait_aes",
	.id     = -1,
};

#if defined(CONFIG_FB_AIT)
struct ait_display_platform_data_info disp_plat_data = {
	.start  = AITC_BASE_PHY_DSPY,
	.end    = AITC_BASE_PHY_DSPY + (sizeof(AITS_DSPY) - 1),
	.irq = AIC_SRC_DSPY,
	.reserved = 0x8000,
};
#endif

#if defined(CONFIG_FB_AIT_RGBLCD)
extern struct ait_fb_platform_data_info alc027a_disp_plat_data;
#endif

#if 0
static void __init ait_soc_mcrv2_map_io(void)
{
	extern void __init ait_soc_resv_dram(int bank, unsigned long base, unsigned int length);

	pr_info("Reserve physical mem for video driver start: 0x%08x size: %d (KiB) \n",ait_cam_resources->start, resource_size(ait_cam_resources)/1024);
	ait_soc_resv_dram(0, ait_cam_resources->start, resource_size(ait_cam_resources));

	ait_soc_map_io();
}
#endif
static void __init ait_mem_reserv(void)
{
	//ait_dram_reserve_video_driver();
	ait_dram_reserve_cpub();
#ifdef CONFIG_MCRV2_I2CS
	ait_dram_reserve_i2cs();
#endif
}

static void ait_6366evb_power_off(void)
{
#ifdef CONFIG_AIT_FAST_BOOT
//#define AIT_GPIO_PWR_DET    MMPF_PIO_REG_GPIO40
#define AIT_GPIO_PWR_HOLD_AIT   MMPF_PIO_REG_GPIO41
#define AIT_GPIO_PWR_HOLD_QD    MMPF_PIO_REG_GPIO44

    int power_det_off = -1;
    //int power_adp_off = -1;
	
    //power_det_off = ait_get_gpio_value(AIT_GPIO_PWR_DET);
    //power_adp_off = ait_get_gpio_value(AIT_GPIO_PWR_ADP);    
#if 1
    pr_info("power_key : %s\n",  power_det_off?"HIGH":"LOW" );
#endif
    if(0/*power_det_off*/) {
        // trigger reboot
        MMPF_SYS_ResetSystem(0);
    }
    // Low -> power off
    else  {//adp(off), sw(off)
        ait_set_gpio_output(AIT_GPIO_PWR_HOLD_AIT,0);
        ait_set_gpio_output(AIT_GPIO_PWR_HOLD_QD ,0);
    }    
#endif
  
}

static void __init ait_board_init(void)
{
	char *p;
	int i;
	
	pr_info("## 2OS CPUA Board Device Initial ##");
	/* Serial */
	ait_add_device_serial();

#if defined(CONFIG_MMC_AIT_IF2)
	ait_add_device_mmc(2, &ait_sd2_data_sdio);
#endif

#if defined(CONFIG_USB_MUSB_AIT)||defined(CONFIG_USB_MUSB_AIT_MODULE)
	ait_add_otg(&ait_otg_data );
#endif	

#if defined(CONFIG_FB_AIT_DISPLAY)
	/* LCD */
	ait_add_device_fb_display(NULL); //argument is dummy
#endif
#if defined(CONFIG_MTD_NAND_AIT_NAND)
	/* Nand */
	ait_add_device_nand(NULL); //argument is dummy
#endif
#if 1 // defined(CONFIG_SPI_AIT) || defined(CONFIG_SPI_AIT_MODULE) ||defined(CONFIG_SPI_AIT_FULL) 
	/* SPI */
#if defined(CONFIG_SPI_AIT) || defined(CONFIG_SPI_AIT_SIF) || defined(CONFIG_SPI_AIT_SIF_MODULE) || defined (CONFIG_SPI_AIT_FULL)
	ait_add_device_sif(ait_sif_devices, ARRAY_SIZE(ait_sif_devices));
#endif
#if defined(CONFIG_SPI_AIT_PSPI) || defined(CONFIG_SPI_AIT_PSPI_MODULE) || defined (CONFIG_SPI_AIT_FULL)
	ait_add_device_pspi(ait8428_evb_pspi_devices, ARRAY_SIZE(ait8428_evb_pspi_devices),0,2);
#endif

#endif

#if (defined(CONFIG_SPI_AIT) && defined(CONFIG_SUPPORT_2WAY_SPI) && defined(CONFIG_MT7687) )
	ait_add_device_pspi(ait8428_evb_pspi_slave_devices, ARRAY_SIZE(ait8428_evb_pspi_slave_devices),1,0);
#endif


	/* MMC */
    #if defined(CONFIG_MMC_AIT_IF0)
	ait_add_device_mmc(0, &ait_sd0_data_sd);
    #endif
    #if defined(CONFIG_MMC_AIT_IF1)
	ait_add_device_mmc(1, &ait_sd1_data_emmc);
    #endif
  

	/* I2C */
	//ait_add_device_phi2c(ek_phi2c_devices, ARRAY_SIZE(ek_phi2c_devices));
#if defined(CONFIG_AIT_I2CM_1)
	ait_add_device_i2c(mcrv2_phi2c_devices, ARRAY_SIZE(mcrv2_phi2c_devices), 1);
#endif
#if defined(CONFIG_AIT_I2CM_2)
	ait_add_device_i2c(mcrv2_phi2c_devices, ARRAY_SIZE(mcrv2_phi2c_devices), 2);
#endif
	ait_add_adapter_i2c();

#if defined(CONFIG_SND_MCRV2_SOC_I2S_IF0)
	ait_add_device_aud_i2s(0);
#elif defined(CONFIG_SND_MCRV2_SOC_I2S_IF1)
	ait_add_device_aud_i2s(1);
#else
	ait_add_device_aud(0);
	ait_add_device_aud(1);
#endif

    	/* CpuComm */
	ait_register_cpucomm();

	ait_add_device_cpub_mgr();
	
	platform_device_register(&ait_rtos_md_device);

	platform_device_register(&ait_rtos_aac_device);

	platform_device_register(&ait_rtos_aes_device);

	/* LEDs */
#if defined(CONFIG_NEW_LEDS)
        ait_gpio_leds(gpio_leds, ARRAY_SIZE(gpio_leds));
#endif

//	platform_device_register(ait_cam_resources);
#if 0
	ait_add_device_ait_cam(ait_cam_resources,ARRAY_SIZE(ait_cam_resources));
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	ait_add_device_buttons();
#endif

  pm_power_off = ait_6366evb_power_off;

	platform_device_register(&ait_camera_device);

#if defined(CONFIG_FB_AIT)
	ait_add_device_display(&disp_plat_data);
#endif
#if defined(CONFIG_FB_AIT_RGBLCD)
	ait_add_device_fb(0, &alc027a_disp_plat_data);
#endif
		
}

MACHINE_START(MACH_MCRV2_6366, "AIT MCR V2 AIT6366 EVB")
	.timer      = &ait_pit_timer,
	.map_io     = ait_soc_map_io,//ait_soc_mcrv2_map_io,
	.init_early = ait_init_early,            ///< .init_early() -> .init_irq() -> .init_machine()
	.reserve = ait_mem_reserv,
	.init_irq   = ait_soc_init_irq_default,
	.init_machine = ait_board_init,
MACHINE_END
