/*
 * arch/arm/mach-vsnv3/at91sam9260_devices.c
 *
 *  Copyright (C) 2006 Atmel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/i2c-gpio.h>

#include <linux/delay.h>
#include <linux/semaphore.h>


#include <mach/board.h>
#include <mach/cpu.h>
#include <linux/usb/musb.h>
#include <linux/i2c/at24.h>

#include "generic.h"

#include <mach/includes_fw.h>

#include <mach/reg_retina.h>

#include <mach/mmp_reg_gbl.h>
#include <mach/mmpf_pll.h>

#include <mach/mmpf_sd.h>
#include <mach/mmpf_uart.h>

#include <mach/mmpf_i2cm.h>
#include <mach/mmp_reg_int.h>
#include <mach/cpucomm/cpucomm-lx.h>
#include <mach/cpucomm/cpucomm-uart.h>

/* --------------------------------------------------------------------
 *  USB Device (Gadget)
 * -------------------------------------------------------------------- */
//#define CONFIG_USB_AIT
#if defined(CONFIG_USB_AIT)
static struct ait_udc_data udc_data;
static struct ait_otg_platform_data* otg_data;


static struct resource otg_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_USBCTL,
		.end	= AITC_BASE_PHY_USBCTL + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AITC_BASE_PHY_USBDMA,
		.end	= AITC_BASE_PHY_USBDMA + SZ_256 - 1,
		.flags	= IORESOURCE_MEM,
	},	
	[2] = {
		.start	= AIC_SRC_USB,
		.end	= AIC_SRC_USB,
		.flags	= IORESOURCE_IRQ,
		.name	="mc"
	},
};

static struct platform_device vsnv3_udc_device = {
	.name		= "vsnv3_udc",
	.id		= -1,
	.dev		= {
				.platform_data		= &udc_data,
	},
	.resource	= otg_resources,
	.num_resources	= ARRAY_SIZE(otg_resources),
};

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 0,
	.num_eps	= 8,
	.ram_bits	= 12,
};


static struct musb_hdrc_platform_data musb_plat = {
//#ifdef CONFIG_USB_MUSB_OTG
//	.mode		= MUSB_OTG,
//#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode		= MUSB_HOST,
//#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
//	.mode		= MUSB_PERIPHERAL,
//#endif
	
	/* .clock is set dynamically */
	.config		= &musb_config,

	/* REVISIT charge pump on TWL4030 can supply up to
	 * 100 mA ... but this value is board-specific, like
	 * "mode", and should be passed to usb_musb_init().
	 */
	.power		= 250,			/* up to 100 mA */
};

static struct platform_device ait_musb_device = {
	.name		= "musb-ait",
	.id		= -1,
	.dev		= {
				.platform_data		= &musb_plat,
	},
	.resource	= otg_resources,
	.num_resources	= ARRAY_SIZE(otg_resources),
};

static struct platform_device ait_otg_device = {
	.name		= "ait-usb-otg",
	.id		= -1,
	.dev		= {
//				.platform_data		= &musb_plat,
	},
	.resource	= otg_resources,
	.num_resources	= ARRAY_SIZE(otg_resources),
};
	
void __init ait_add_device_udc(struct ait_udc_data *data)
{
	if (!data)
		return;

	udc_data = *data;
	platform_device_register(&vsnv3_udc_device);
}


void __init ait_add_otg(struct ait_otg_platform_data *data)
{
	if (!data)
		return;

	if ( platform_device_register( & ait_musb_device) )
		printk( KERN_ERR "usb: can't register otg device/n" ) ;
	else
		printk( KERN_INFO "usb: DR OTG registered/n" ) ;
	
}



#else
void __init ait_add_device_udc(struct ait_udc_data *data) {}
void __init ait_add_otg(struct ait_otg_platform_data *data){}

#endif


/* --------------------------------------------------------------------
 *  MMC / SD
 * -------------------------------------------------------------------- */

#if defined(CONFIG_MMC_AIT)

static u64 mmc_dmamask = DMA_BIT_MASK(32);

static struct resource mmc_resources_sd0[] = {
    [0] = {
        .name = "SD0 Reg",
        .start  = AITC_BASE_PHY_SD0,
        .end    = AITC_BASE_PHY_SD0 + SZ_256-1,
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start  = AIC_SRC_SD,
        .end    = AIC_SRC_SD,
        .flags  = IORESOURCE_IRQ,
    },
};
static struct resource mmc_resources_sd1[] = {
	[0] = {
		.name = "SD1 Reg",	
		.start	= AITC_BASE_PHY_SD1,
		.end	= AITC_BASE_PHY_SD1 + SZ_256-1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_SD,
		.end	= AIC_SRC_SD,
		.flags	= IORESOURCE_IRQ,
	},
};
//#if defined(CONFIG_AIT_CHIP_MCR_V2)
#if (CHIP==MCR_V2)
static struct resource mmc_resources_sd2[] = {
    [0] = {
        .name = "SD2 Reg",
        .start  = AITC_BASE_PHY_SD2,
        .end    = AITC_BASE_PHY_SD2 + SZ_256-1,
        .flags  = IORESOURCE_MEM,
    },
    [1] = {
        .start  = AIC_SRC_SD,
        .end    = AIC_SRC_SD,
        .flags  = IORESOURCE_IRQ,
    },
};
#endif

static struct ait_sd_data mmc_sd_data[3];

static struct platform_device ait_sd_devices[] = {
    [0] = {
        .name   = DEVICE_NAME_SD0,
        .id     = -1,
        .dev    = {
            .dma_mask           = &mmc_dmamask,
            .coherent_dma_mask  = DMA_BIT_MASK(32),
        //    .platform_data      = mmc_sd_data + 0,
        },
        .resource       = mmc_resources_sd0,
        .num_resources  = ARRAY_SIZE(mmc_resources_sd0),
    },
    [1] = {
        .name   = DEVICE_NAME_SD1,
        .id     = -1,
        .dev    = {
            .dma_mask           = &mmc_dmamask,
            .coherent_dma_mask  = DMA_BIT_MASK(32),
       //     .platform_data      = mmc_sd_data + 1,
        },
        .resource       = mmc_resources_sd1,
        .num_resources  = ARRAY_SIZE(mmc_resources_sd1),
    },
//#if defined(CONFIG_AIT_CHIP_MCR_V2)
#if (CHIP==MCR_V2)
    [2] = {
        .name   = DEVICE_NAME_SD2,
        .id     = -1,
        .dev    = {
            .dma_mask           = &mmc_dmamask,
            .coherent_dma_mask  = DMA_BIT_MASK(32),
          //  .platform_data      = & mmc_sd_data + 2,
        },
        .resource       = mmc_resources_sd2,
        .num_resources  = ARRAY_SIZE(mmc_resources_sd2),
    },
#endif
};

void __init ait_add_device_mmc(short mmc_id, struct ait_sd_data *data)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;
    stSDMMCHandler *pSdHdl;

	if (!data)
		return;

    pSdHdl = MMPF_SD_GetHandler(mmc_id);

    pSdHdl->ubSdPadMapping = data->pad_id;

    pSdHdl->ubSdBusWidth = data->bus_width;

    pGBL->GBL_SW_RST_EN[0] = (GBL_RST_SD0 << pSdHdl->id);
    mdelay(1);
    pGBL->GBL_SW_RST_DIS[0] = (GBL_RST_SD0 << pSdHdl->id);

    MMPF_SD_InitialInterface(pSdHdl);

    ait_sd_devices[mmc_id].dev.platform_data = data;
    platform_device_register(ait_sd_devices + mmc_id);
}

#else
void __init ait_add_device_mmc(short mmc_id, struct ait_sd_data *data) {}

#endif
#if 0
static struct resource ait_phi2c_resources[] = {
	[0] = {
		.start	= AIC_SRC_I2CM ,
		.end    = AIC_SRC_I2CM ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ait_vsnv3_phi2c= {
	.name		= "VsionV3_phi2c",
	.id			= -1,
	.resource	= ait_phi2c_resources,
	.num_resources	= ARRAY_SIZE(ait_phi2c_resources),
};


void __init ait_add_device_phi2c(struct i2c_board_info *devices, int nr_devices)
{
	i2c_register_board_info(1, devices, nr_devices);
	platform_device_register(&ait_vsnv3_phi2c);
}
#endif 

#if (CHIP==VSN_V3)
static struct resource ait_i2c_0_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_I2CM1,//AITC_BASE_PHY_I2CM,//AT91SAM9260_BASE_TWI,
		.end	= AITC_BASE_PHY_I2CM1+SZ_256-1,//AITC_BASE_PHY_I2CM+ sizeof(AITS_I2CMS) - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_I2CM ,
		.end	= AIC_SRC_I2CM ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ait_i2c_1_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_I2CM2,
		.end	= AITC_BASE_PHY_I2CM2+SZ_256-1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_I2CM ,
		.end	= AIC_SRC_I2CM ,
		.flags	= IORESOURCE_IRQ,
	},
};
#endif

#if (CHIP==MCR_V2)
static struct resource ait_i2c_0_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_I2CM0,
		.end	= AITC_BASE_PHY_I2CM0+SZ_128-1,//AITC_BASE_PHY_I2CM+ sizeof(AITS_I2CMS) - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_I2CM ,
		.end	= AIC_SRC_I2CM ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ait_i2c_1_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_I2CM1,
		.end	= AITC_BASE_PHY_I2CM1+SZ_128-1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_I2CM ,
		.end	= AIC_SRC_I2CM ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ait_i2c_2_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_I2CM2,
		.end	= AITC_BASE_PHY_I2CM2+SZ_128-1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_I2CM ,
		.end	= AIC_SRC_I2CM ,
		.flags	= IORESOURCE_IRQ,
	},
};
#endif

//#define MAX_I2C_HW 2

#if (CHIP==VSN_V3)
static struct semaphore* g_pI2C_sem[] = {0,0};
#endif
#if (CHIP==MCR_V2)
static struct semaphore* g_pI2C_sem[] = {0,0,0};
#endif

static struct semaphore g_I2C_sem[sizeof(g_pI2C_sem)/sizeof(struct semaphore*)];

#if (CHIP==VSN_V3)


static struct ait_i2c_extension ait_i2c_0_pad0_ext = {	//I2CM0, PAD 0
		.uI2cmID = 0,
		.pad = 0,
		//.i2c_hw_spinlock = &i2cm_i2c_0_lock,
		.i2c_hw_lock = &g_pI2C_sem[0],
};
static struct ait_i2c_extension ait_i2c_0_pad1_ext = {//I2CM2, PAD 1
		.uI2cmID = 1,
		.pad = 0,
		//.i2c_hw_spinlock = &i2cm_i2c_0_lock,
		.i2c_hw_lock = &g_pI2C_sem[1],
};
static struct ait_i2c_extension ait_i2c_1_pad0_ext = {//I2CM1, PAD 0
		.uI2cmID = 1,
		.pad = 0,
		//.i2c_hw_spinlock = 0,
		.i2c_hw_lock = &m_pI2C_sem[2],
};

static struct platform_device ait_i2c_bus[] = {
	[0] = {// phi2c , pad 1
		.name		= "AIT_I2C",
		.id			= 0,
		.resource	= ait_i2c_0_resources,
		.num_resources	= ARRAY_SIZE(ait_i2c_0_resources),
		.dev = {
			.platform_data = &ait_i2c_1_pad0_ext,
		},
	},
#if defined(CONFIG_AIT_I2CM_1)
	[1] = {// phi2c , pad 0
		.name		= "AIT_I2C",
		.id			= 1,
		.resource	= ait_i2c_1_resources,
		.num_resources	= ARRAY_SIZE(ait_i2c_1_resources), ////resource conflict
		.dev = {
			.platform_data = &ait_i2c_0_pad0_ext,
		},
	},
#endif
#if defined(CONFIG_AIT_I2CM_2)
	[2] = {// phi2c , pad 1
		.name		= "AIT_I2C",
		.id			= 2,
		.resource	= 0,//ait_vsnv3_phi2c_resources,
		.num_resources	= 0,//ARRAY_SIZE(ait_vsnv3_phi2c_resources),
		.dev = {
			.platform_data = &ait_i2c_0_pad1_ext,
		},
	},
#endif
};
#endif

#if (CHIP==MCR_V2)

static struct ait_i2c_extension ait_i2c_ext[] = {
	[0] = {	//I2CM 0, PAD 0
		.uI2cmID = MMPF_I2CM_ID_0,
		.pad = 0,
		//.i2c_hw_spinlock = &i2cm_i2c_0_lock,
		.i2c_hw_lock = &g_pI2C_sem[0] ,
	},
	[1] = {	//I2CM 1, PAD 0, TP_SCL, TP_SDA 
		//.uI2cmID = MMPF_I2CM_ID_2,
		//.pad = 0,
		.uI2cmID = MMPF_I2CM_ID_2,
		.pad = 1,
		//.i2c_hw_spinlock = &i2cm_i2c_0_lock,
		.i2c_hw_lock = &g_pI2C_sem[1],
	},
	[2] = {	//I2CM 2, PAD 0, LCD_SCL, LCD_SDA (PLCD_FLM)
		.uI2cmID = MMPF_I2CM_ID_1,
		.pad = 1,
		//.i2c_hw_spinlock = 0,
		.i2c_hw_lock = &g_pI2C_sem[2],
	},
};

static struct platform_device ait_i2c_bus[] = {
	[0]={// phi2c , pad 0
		.name		= "AIT_I2C",
		.id			= 0,
		.resource	= ait_i2c_0_resources,
		.num_resources	= ARRAY_SIZE(ait_i2c_0_resources),
		.dev = {
			.platform_data = &ait_i2c_ext[0],
		},
	},
#if defined(CONFIG_AIT_I2CM_1)
	[1]={// phi2c , pad 1
		.name		= "AIT_I2C",
		.id			= 1,
		.resource	= ait_i2c_1_resources,
		.num_resources	= ARRAY_SIZE(ait_i2c_1_resources), ////resource conflict
		.dev = {
			.platform_data = &ait_i2c_ext[1],
		},
	},
#endif
#if defined(CONFIG_AIT_I2CM_2)
	[2]={// phi2c , pad 0
		.name		= "AIT_I2C",
		.id			= 2,
		.resource	= ait_i2c_2_resources,
		.num_resources	= ARRAY_SIZE(ait_i2c_2_resources),
		.dev = {
			.platform_data = &ait_i2c_ext[2],
		},
	},
#endif
};
#endif

void __init ait_add_adapter_i2c()
{
	int n;
	for(n=0;n<ARRAY_SIZE(ait_i2c_bus);++n)
	{
		struct ait_i2c_extension* i2c_ext = (struct ait_i2c_extension*)ait_i2c_bus[n].dev.platform_data;
		if(i2c_ext->i2c_hw_lock) //check semaphore is initialized or not
		{
			int hw_id = i2c_ext->uI2cmID;
			if(!g_pI2C_sem[hw_id]) //inti it
			{
				g_pI2C_sem[hw_id] = &g_I2C_sem[hw_id];
				sema_init(g_pI2C_sem[hw_id],1);
			}
			//printk(KERN_INFO "i2c-ext[%d]:%x\n",n,i2c_ext->uI2cmID); 
		}		
		platform_device_register(&ait_i2c_bus[n]);
	}
}

void __init ait_add_device_i2c(struct i2c_board_info *devices, int nr_devices,int bus)
{
  struct ait_i2c_platform_data *pd = (struct ait_i2c_platform_data *)devices->platform_data ;  
  printk(KERN_INFO "ait-i2cbus%d : %s,slaveaddr:%x,board.platform_data:%x\n",bus,devices->type,devices->addr,(u32)devices->platform_data); 
  if(pd && pd->tag==AIT_I2C_INFO_TAG) {
    if(pd->power_on) 
        pd->power_on(1);
  }
  i2c_register_board_info(bus, devices, nr_devices);
	//platform_device_register(&ait_vsnv3_phi2c);
}
#if defined(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA)||defined(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUB)
/* cpucomm*/
static struct cpucomm_uart_data cpucomm_uart_platform_data = {CPU_COMM_ID_UART,AIT_SRAM_PHYS_BASE+0x4000};


static struct cpucomm_board_info __initdata cpucomm_client_dev[] = {
    {
        CPUCOMM_BOARD_INFO("cpucomm_uart"),
        .platform_data = (void*)&cpucomm_uart_platform_data,
    },
    {
        CPUCOMM_BOARD_INFO("cpucomm_dummy_2"),
    },
	/* more devices can be added using expansion connectors */

};

static struct resource cpucomm_adap_resources[] = {
	[0] = {
		.start	= AIC_SRC_CPU2CPU ,
		.end    = AIC_SRC_CPU2CPU ,
		.flags	= IORESOURCE_IRQ,
	},

    [1] = {
        .start  = AITC_BASE_PHY_HINT,
        .end    = ((resource_size_t)AITC_BASE_PHY_HINT) + sizeof(AITPS_HINT) - 1,
        .flags  = IORESOURCE_MEM,
    },

    [2] = {
        .start  = (resource_size_t)(AITC_BASE_PHY_CPU_SAHRE),
        .end    = ((resource_size_t)(AITC_BASE_PHY_CPU_SAHRE)) + sizeof(AITPS_CPU_SHARE) - 1,
        .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device cpucomm_adap_device= {
	.name		        = "cpucomm",
	.id			        = -1,
    .resource   = cpucomm_adap_resources,
    .num_resources  = ARRAY_SIZE(cpucomm_adap_resources),
};


void __init ait_register_cpucomm(void)
{
	cpucomm_register_board_info(cpucomm_client_dev, ARRAY_SIZE(cpucomm_client_dev));
	platform_device_register(&cpucomm_adap_device);
}
#else
void __init ait_register_cpucomm(void){return;}
#endif

/* --------------------------------------------------------------------
 *  SPI
 * -------------------------------------------------------------------- */

#if defined(CONFIG_SPI_AIT) 
#define SPI_BUS_ID 0
static u64 spi_dmamask = DMA_BIT_MASK(32);
static u64 sif_dmamask = DMA_BIT_MASK(32);

static struct resource spi0_resources[] = {
		[0] = {
			.start	= AITC_BASE_PHY_PSPI0,
			.end	= AITC_BASE_PHY_PSPI0 + SZ_64-1, //(0x6D00~0x6D3F)
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= AIC_SRC_SPI,
			.end	= AIC_SRC_SPI,
			.flags	= IORESOURCE_IRQ,
		},
};
	
#if defined(CONFIG_AIT_CHIP_MCR_V2_MP)

static struct resource spi1_resources[] = {
	
		[0] = {
			.start	= AITC_BASE_PHY_PSPI1,
			.end	= AITC_BASE_PHY_PSPI1 + SZ_64-1, 
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= AIC_SRC_SPI,
			.end	= AIC_SRC_SPI,
			.flags	= IORESOURCE_IRQ,
		},
	
};

static struct resource spi2_resources[] = {
		[0] = {
			.start	= AITC_BASE_PHY_PSPI2,
			.end	= AITC_BASE_PHY_PSPI2 + SZ_64-1, 
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start	= AIC_SRC_SPI,
			.end	= AIC_SRC_SPI,
			.flags	= IORESOURCE_IRQ,
		},
};
#endif	


static struct resource sif_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_SIF,
		.end	= AITC_BASE_PHY_SIF + SZ_128-1, //(0x6D00~0x6D3F)
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_SIF,
		.end	= AIC_SRC_SIF,
		.flags	= IORESOURCE_IRQ,
	},
};


static struct platform_device vsnv3_sif_device = {
	.name	= "vsnv3_sif",
	.id		= SPI_BUS_ID,
	.dev	= {
        .dma_mask		= &sif_dmamask,
        .coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	    = sif_resources,
	.num_resources	= ARRAY_SIZE(sif_resources),
};


static struct platform_device vsnv3_pspi_device[] = {
	{
		.name   = "ait_pspi",
		.id		= SPI_BUS_ID+1,
		.dev	= {
	        .dma_mask		= &spi_dmamask,
	        .coherent_dma_mask	= DMA_BIT_MASK(32),
		},
		.resource	    = spi0_resources,
		.num_resources	= ARRAY_SIZE(spi0_resources),
	},
#if defined(CONFIG_AIT_CHIP_MCR_V2_MP)

	{
		.name   = "ait_pspi",
		.id		= SPI_BUS_ID+2,
		.dev	= {
	        .dma_mask		= &spi_dmamask,
	        .coherent_dma_mask	= DMA_BIT_MASK(32),
		},
		.resource	    = spi1_resources,
		.num_resources	= ARRAY_SIZE(spi1_resources),
	},
	{
		.name   = "ait_pspi",
		.id		= SPI_BUS_ID+3,
		.dev	= {
	        .dma_mask		= &spi_dmamask,
	        .coherent_dma_mask	= DMA_BIT_MASK(32),
		},
		.resource	    = spi2_resources,
		.num_resources	= ARRAY_SIZE(spi2_resources),
	}
#endif	

};


//static const unsigned spi0_standard_cs[4] = { AT91_PIN_PA3, AT91_PIN_PC11, AT91_PIN_PC16, AT91_PIN_PC17 };

void __init ait_add_device_pspi(struct spi_board_info *devices, int nr_devices,int pspi_selection,int pad_selection)
{
	int i;
	AITPS_GBL   pGBL = AITC_BASE_GBL;

	BUG_ON(pspi_selection>=ARRAY_SIZE(vsnv3_pspi_device));
	BUG_ON(pad_selection>=3);

	switch(pspi_selection)
	{
		case 0:
//			pGBL->GBL_I2S_DMIC_CFG |=	GBL_DMIC_PAD_DISABLE&GBL_DMIC_PAD_MASK;
			pr_info("0x694D = 0x%x",pGBL->GBL_GPIO_CFG[3]);
		//	pGBL->GBL_I2S_DMIC_CFG |=	GBL_DMIC_PAD_DISABLE&GBL_DMIC_PAD_MASK;
			
		    	pGBL->GBL_SPI_PAD_CFG |= GBL_PSPI0_PAD(pad_selection)&GBL_PSPI0_PAD_MASK;
		    	break;
		case 1:
		    	pGBL->GBL_SPI_PAD_CFG |= GBL_PSPI1_PAD(pad_selection)&GBL_PSPI1_PAD_MASK;			
			break;

		case 2:
		    	pGBL->GBL_SPI_PAD_CFG |= GBL_PSPI2_PAD_EN&GBL_PSPI2_PAD_MASK;		
			break;
	}
	platform_device_register(&vsnv3_pspi_device[pspi_selection]);	
	
	spi_register_board_info(devices, nr_devices);
}

void __init ait_add_device_sif(struct spi_board_info *devices, int nr_devices)
{
	spi_register_board_info(devices, nr_devices);

	platform_device_register(&vsnv3_sif_device);
}


#else
void __init ait_add_device_pspi(struct spi_board_info *devices, int nr_devices,int pspi_selection,int pad_selection){}
void __init ait_add_device_sif(struct spi_board_info *devices, int nr_devices) {}

#endif

/* --------------------------------------------------------------------
 *  Watchdog
 * -------------------------------------------------------------------- */

#if defined(CONFIG_VSNV3_WATCHDOG)
static struct resource vsnv3_wdt_resources[] = {	
	[0] = {
		.start	= AITC_BASE_PHY_WD,
		.end    = AITC_BASE_PHY_WD+SZ_16 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_WD,
		.end	= AIC_SRC_WD,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device vsnv3_wdt_device = {
	.name           = "vsnv3_wdt",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(vsnv3_wdt_resources),
	.resource       = vsnv3_wdt_resources,
};

void __init ait_add_device_watchdog(void)
{
	platform_device_register(&vsnv3_wdt_device);
}
#else
void __init ait_add_device_watchdog(void) {}
#endif


/* --------------------------------------------------------------------
 *  SSC -- Synchronous Serial Controller
 * -------------------------------------------------------------------- */

#if defined(CONFIG_AIT_SSC) || defined(CONFIG_AIT_SSC_MODULE)

static u64 ssc_dmamask = DMA_BIT_MASK(32);

static struct resource ssc_resources[] = {
	
	[0] = {
		.start	= AITC_BASE_PHY_I2S0,
		.end		= AITC_BASE_PHY_I2S0 + SZ_256 - 1,
		.flags	= IORESOURCE_MEM,
	},
//#if (CHIP==VSN_V3)
	[1] = {
		.start	= AIC_SRC_AUD_FIFO,
		.end		= AIC_SRC_AUD_FIFO,
		.flags	= IORESOURCE_IRQ,
	},
//#elif (CHIP==MCR_V2)
//	[1] = {
//		.start	= AIC_SRC_AUD_I2S,
//		.end		= AIC_SRC_AUD_I2S,
//		.flags	= IORESOURCE_IRQ,
//	},
//#endif 

};


static struct resource i2s_resources[] = {
#if 1
	[0] = {
		.start	= AITC_BASE_PHY_I2S0,
		.end		= AITC_BASE_PHY_I2S0 + SZ_256 - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif	
	[1] = {
		.start	= AIC_SRC_AUD_FIFO,
		.end    	= AIC_SRC_AUD_FIFO,
		.flags	= IORESOURCE_IRQ,
	},
	
};


static struct resource ssc1_resources[] = {
#if 1	
	[0] = {
		.start	= AITC_BASE_PHY_AUD,
		.end		= AITC_BASE_PHY_AUD+SZ_128 - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif	
	[1] = {
		.start	= AIC_SRC_AFE_FIFO,
		.end		= AIC_SRC_AFE_FIFO,
		.flags	= IORESOURCE_IRQ,
	},
	
};

static struct resource afe_resources[] = {
#if 1
	[0] = {
		.start	= AITC_BASE_PHY_AUD,
		.end		= AITC_BASE_PHY_AUD+SZ_128 - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif	
	[1] = {
		.start	= AIC_SRC_AFE_FIFO,
		.end    	= AIC_SRC_AFE_FIFO,
		.flags	= IORESOURCE_IRQ,
	},
	
};


#if 0
static struct resource soc_audio_resources[] = {
#if 0	
	[0] = {
		.start	= + 1,
		.end	= + 2,//SZ_16K - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif
	[0] = {
		.start	= 14,
		.end	= 14,
		.flags	= IORESOURCE_IRQ,
	},

};

static struct resource wm8971_hifi_resources[] = {
	[0] = {
		.start	= + 2,
		.end	= + 3,
		.flags	= IORESOURCE_MEM,
	},
	
	[1] = {
		.start	= 15,
		.end	= 15,
		.flags	= IORESOURCE_IRQ,
	},
	
};
#endif

static struct platform_device ait_ssc_device = {
	.name	= "vsnv3aud",
	.id	= 0,
	.dev	= {
		.dma_mask		= &ssc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= ssc_resources,
	.num_resources	= ARRAY_SIZE(ssc_resources),
};

static struct platform_device vsnv3_i2s_codec_device = {
#if (CHIP==VSN_V3)
	.name = "vsnv3-i2s-codec",
#elif (CHIP==MCR_V2)
	.name = "mcrv2-i2s-codec",
#endif
	.id	= -1,

	.dev	= {
		.dma_mask		= &ssc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data 		=0,//(void*)&vsnv3afe_data,
	},
	.resource	= i2s_resources,
	.num_resources	= ARRAY_SIZE(i2s_resources),
};


static struct platform_device vsnv3_afe_device = {
	.name	= "vsnv3aud",
	.id	= 1,
	.dev	= {
		.dma_mask		= &ssc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.resource	= ssc1_resources,
	.num_resources	= ARRAY_SIZE(ssc1_resources),
};



static struct platform_device vsnv3_afe_codec_device = {
	.name = "vsnv3-afe-codec",	
	.id	= -1,

	.dev	= {
		.dma_mask		= &ssc_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data 		=0,//(void*)&vsnv3afe_data,
	},
	.resource	= afe_resources,
	.num_resources	= ARRAY_SIZE(afe_resources),
};

void __init ait_add_device_aud(int id)
{
	struct platform_device *pdev;
	/*
	 * NOTE: caller is responsible for passing information matching
	 * "pins" to whatever will be using each particular controller.
	 */
	switch (id) {
	case 0:
		pdev = &ait_ssc_device;
		platform_device_register(&vsnv3_i2s_codec_device);
		break;
	case 1:
		pdev = &vsnv3_afe_device;
		platform_device_register(&vsnv3_afe_codec_device);		
		break;
	default:
		return;
	}

	platform_device_register(pdev);

}

#else
void __init ait_add_device_aud(int id){}
#endif

#if 0
/* --------------------------------------------------------------------
 *  UART
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SERIAL_AIT)
static struct resource dbgu_resources[] = {
	[0] = {
		.start	= AIT_UART0_PHYS_BASE,
		.end	= AIT_UART0_PHYS_BASE + SZ_512 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_UART,
		.end	= AIC_SRC_UART,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct ait_uart_data dbgu_data = {
	.use_dma_tx	= 0,
	.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
	.regs = (void*)(AIT_OPR_P2V(AITC_BASE_PHY_UART_BOOT)),
	.padset     = MMPF_UART_PADSET_0,
	.baudrate = 115200

};

static u64 dbgu_dmamask = DMA_BIT_MASK(32);

static struct platform_device ait_dbgu_device = {
	.name       = "ait_usart",
	.id         = 0,
	.dev        = {
        .dma_mask           = &dbgu_dmamask,
        .coherent_dma_mask  = DMA_BIT_MASK(32),
        .platform_data      = &dbgu_data,
	},
	.resource       = dbgu_resources,
	.num_resources  = ARRAY_SIZE(dbgu_resources),
};

// uart 1
static struct resource ait_uart_resources_1[] = {
	[0] = {
		.start	= 0x80005C00, // &(AITC_BASE_UARTB->UART_1),
		.end	= 0x80005C00+256-1,// &(AITC_BASE_UARTB->UART_1) + sizeof(AITS_US) - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= AIC_SRC_UART,
		.end	= AIC_SRC_UART,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct ait_uart_data ait_uart_data_1 = {
	.use_dma_tx	= 0,
	.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
	.regs       = (void*)(AIT_OPR_P2V(0x80005C00)),
	.padset     = MMPF_UART_PADSET_0,
	.baudrate = 115200
};

static u64 ait_uart_dmamask_1 = DMA_BIT_MASK(32);

static struct platform_device ait_uart_device_1 = {
	.name       = "ait_usart",
	.id         = 1,
	.dev        = {
        .dma_mask           = &ait_uart_dmamask_1,
        .coherent_dma_mask  = DMA_BIT_MASK(32),
        .platform_data      = &ait_uart_data_1,
	},
	.resource       = ait_uart_resources_1,
	.num_resources  = ARRAY_SIZE(ait_uart_resources_1),
};
static inline void configure_dbgu_pins(void)
{
}

static struct platform_device *__initdata ait_uarts[AIT_MAX_UART];	/* the UARTs to use */
struct platform_device *ait_default_console_device;	/* the serial console device */

{
	struct platform_device *pdev;
	struct ait_uart_data *pdata;

	pdev = &ait_dbgu_device;
	pdata = pdev->dev.platform_data;
	pdata->num = pdev->id;//portnr;		/* update to mapped ID */

	//if (portnr < AIT_MAX_UART)
		ait_uarts[pdev->id] = pdev;

#if (AIT_MAX_UART == 2)
	pdev = &ait_uart_device_1;
	pdata = pdev->dev.platform_data;
	pdata->num = pdev->id;      /* update to mapped ID */
	ait_uarts[pdev->id] = pdev;
#endif
}

void __init ait_set_serial_console(unsigned portnr)
{
	if (portnr < AIT_MAX_UART) {
		ait_default_console_device = ait_uarts[portnr];
		ait_arch_set_console_clock(ait_uarts[portnr]->id);
	}
}

void __init ait_add_device_serial(void)
{
	int i;

	for (i = 0; i < AIT_MAX_UART; i++) {
		if (ait_uarts[i])
			platform_device_register(ait_uarts[i]);
	}

	if (!ait_default_console_device)
		printk(KERN_INFO "AIT: No default serial console defined.\n");
}
#else
void __init ait_set_serial_console(unsigned portnr) {}
void __init ait_add_device_serial(void) {}
#endif
#endif

/* --------------------------------------------------------------------
 *  UART
 * -------------------------------------------------------------------- */
#if defined(CONFIG_SERIAL_AIT)

static struct resource ait_uart0_resources[] = {
	[0] = {
		.start	= AITC_BASE_PHY_UART0, 
		.end	= AITC_BASE_PHY_UART0 + sizeof(AITS_US) -1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start = AIC_SRC_UART,
		.end	= AIC_SRC_UART,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ait_uart1_resources[] = {
	[0] = {
		.start= AITC_BASE_PHY_UART1,
		.end	= AITC_BASE_PHY_UART1 + sizeof(AITS_US) -1,
		.flags= IORESOURCE_MEM,
	},
	[1] = {  //resource conflict
		.start= AIC_SRC_UART,
		.end	= AIC_SRC_UART,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ait_uart2_resources[] = {
	[0] = {
		.start= AITC_BASE_PHY_UART2,
		.end	= AITC_BASE_PHY_UART2 + sizeof(AITS_US) -1,
		.flags= IORESOURCE_MEM,
	},
	[1] = {  //resource conflict
		.start= AIC_SRC_UART,
		.end	= AIC_SRC_UART,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ait_uart3_resources[] = {
	[0] = {
		.start= AITC_BASE_PHY_UART3,
		.end	= AITC_BASE_PHY_UART3 + sizeof(AITS_US) -1,
		.flags= IORESOURCE_MEM,
	},
	[1] = {  //resource conflict
		.start= AIC_SRC_UART,
		.end	= AIC_SRC_UART,
		.flags	= IORESOURCE_IRQ,
	},
};
#if 0 //Move to board config
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
	{
		.num = 1,
		.use_dma_tx	= 0,
		.use_dma_rx	= 0,		/* DBGU not capable of receive DMA */
		.uart_hw_id = MMPF_UART_ID_1,
		//.uart_hw_id = MMPF_UART_ID_1,	
		
		.pad = 0,  //PAD4 , pin assigned as below 
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
	},
};
#endif

static u64 dbgu_dmamask = DMA_BIT_MASK(32);

static struct platform_device ait_uart_device[] = {
	{
		.name		= "ait_usart",
		.id		= 0,
		.dev		= {
            .init_name		= "ait_usart0",
					.dma_mask		= &dbgu_dmamask,
					.coherent_dma_mask	= DMA_BIT_MASK(32),
					//.platform_data		= &ait_uart_custom_data[0],
		},
		.resource	= ait_uart0_resources,
		.num_resources	= ARRAY_SIZE(ait_uart0_resources),
	},
	{
		.name		= "ait_usart",
		.id			= 1,
		.dev		= {
            .init_name		= "ait_usart1",	
					.dma_mask		= &dbgu_dmamask,
					.coherent_dma_mask	= DMA_BIT_MASK(32),
					//.platform_data		= &ait_uart_custom_data[1],
		},
		.resource	= ait_uart1_resources,
		.num_resources	= ARRAY_SIZE(ait_uart1_resources),
	},

	{
		.name		= "ait_usart",
		.id			= 2,
		.dev		= {
			.init_name		= "ait_usart2",
			.dma_mask		= &dbgu_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			//.platform_data		= &ait_uart_custom_data[2],
		},
		.resource	= ait_uart2_resources,
		.num_resources	= ARRAY_SIZE(ait_uart2_resources),
	},
	{
		.name		= "ait_usart",
		.id			= 3,
		.dev		= {
			.init_name		= "ait_usart3",		
			.dma_mask		= &dbgu_dmamask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
			//.platform_data		= &ait_uart_custom_data[3],
		},
		.resource	= ait_uart3_resources,
		.num_resources	= ARRAY_SIZE(ait_uart3_resources),
	},
};

#define NUM_AIT_UART ARRAY_SIZE(ait_uart_device) 

//static inline void configure_dbgu_pins(void)
//{
//	at91_set_A_periph(AT91_PIN_PB14, 0);		/* DRXD */
//	at91_set_A_periph(AT91_PIN_PB15, 1);		/* DTXD */
//}

//static inline void configure_usart0_pins(unsigned pins)
//{
//}

static struct platform_device *__initdata ait_uarts[NUM_AIT_UART] = {0}; 

struct platform_device *ait_default_console_device;	/* the serial console device */

/*
//void __init vsnv3_register_uart(unsigned id, unsigned portnr, unsigned pins)
{
	unsigned int n;		
	for(n=0;n<NUM_AIT_UART;++n)
	{
		ait_uarts[n] =  &ait_uart_device[n];
	}
}
*/

void __init ait_register_uart(struct ait_uart_data *uartdata)
{
	//struct ait_uart_data* pdata = uartdata;//(struct ait_uart_data*)ait_uart_device[hw_id].dev.platform_data;
	//pdata->num = portnr;
	
	ait_uart_device[uartdata->num].dev.platform_data =(void*) uartdata;	
	ait_uarts[uartdata->num] = &ait_uart_device[uartdata->num];	
	//if(pin>=0)
	//	pdata->pad = pin;
	
		//( (struct ait_uart_data*)(ait_uart_device[hw_id].dev.platform_data) )->pad = pin;


	//dev_get_platdata(&ait_uart_device[pdata->num].dev) = uartdata;
}


//void __init vsnv3_set_serial_console(unsigned portnr)
void __init ait_set_serial_console(unsigned portnr)
{
	if (portnr < NUM_AIT_UART) {
		ait_default_console_device = &ait_uart_device[portnr];
	}
}

void __init ait_add_device_serial(void)
{
	int i;

	for (i = 0; i < NUM_AIT_UART; i++) {
		if (ait_uarts[i])
			platform_device_register(ait_uarts[i]);
	}

	if (!ait_default_console_device)
		printk(KERN_INFO "AT91: No default serial console defined.\n");
}
#else
void __init ait_register_uart(struct ait_uart_data *uartdata) {}
void __init vsnv3_set_serial_console(unsigned portnr) {}
void __init ait_add_device_serial(void) {}
#endif

/////////// RTC ////////////// 
#if defined(CONFIG_RTC_DRV_AIT8428)
static struct platform_device rtc_ait8428[] = {
	{
		.name		= "rtc-ait8428",
		.id			= -1,
	}
};
void __init ait_add_device_rtc(void)
{
	int i;
	for(i=0;i<ARRAY_SIZE(rtc_ait8428);++i)
		platform_device_register(&rtc_ait8428[i]);
} 
#else
void __init ait_add_device_rtc(void){}	
#endif

#if defined(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA)
static struct resource ait_cpub_mgr_resources[] = {	
	[0] = {
		.start	= AITC_BASE_PHY_CORE,
		.end    	= AITC_BASE_PHY_CORE+ sizeof(AITPS_CORE) - 1,
		.flags	= IORESOURCE_MEM,
	},
#if 0
	[1] = {
		.start	= AIT_CPUB_ITCM_PHYS_BASE,
		.end    	= AIT_CPUB_ITCM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},	
	[2] = {
		.start	= AIT_CPUB_DRAM_PHYS_BASE,
		.end    	= AIT_CPUB_DRAM_PHYS_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
#endif 
};

static struct platform_device  ait_cpub_mgr_device = {
	.name           = "cpub_mgr",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ait_cpub_mgr_resources),
	.resource       = ait_cpub_mgr_resources,
};

void __init ait_add_device_cpub_mgr(void)
{
	platform_device_register(&ait_cpub_mgr_device);
}
#else
void __init ait_add_device_cpub_mgr(void) {}
#endif
/* -------------------------------------------------------------------- */
/*
 * These devices are always present and don't need any board-specific
 * setup.
 */
static int __init ait_add_standard_devices(void)
{
    ait_add_device_rtc(); 
    ait_add_device_watchdog();
    return 0;
}


arch_initcall(ait_add_standard_devices);
