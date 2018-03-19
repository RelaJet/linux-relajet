/*
 * arch/arm/mach-mcrv2/include/mach/board.h
 *
 *  Copyright (C) 2015 AIT
 *  Copyright (C) 2005 HP Labs
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

/*
 * These are data structures found in platform_device.dev.platform_data,
 * and describing board-specific data needed by drivers.  For example,
 * which pin is used for a given GPIO role.
 *
 * In 2.6, drivers should strongly avoid board-specific knowledge so
 * that supporting new boards normally won't require driver patches.
 * Most board-specific knowledge should be in arch/.../board-*.c files.
 */

#ifndef __ASM_ARCH_BOARD_H
#define __ASM_ARCH_BOARD_H

#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/serial.h>
#include <video/ait_fb_fops.h>
#include <mach/mmpf_display.h>
#include <linux/usb/musb.h>

#include <mach/mmpf_uart.h>
#include <mach/mmpf_pio.h>
#include <mach/mmp_reg_uart.h>

 /* USB Device */
struct ait_udc_data {
	u8	vbus_pin;		    /* high == host powering us */
	u8	vbus_active_low;	/* vbus polarity */
	u8	vbus_polled;		/* Use polling, not interrupt */
	u8	pullup_pin;		    /* active == D+ pulled up */
	u8	pullup_active_low;	/* true == pullup_pin is active low */
};

 /* USB Device */
struct ait_otg_platform_data {
	u8	vbus_pin;		    /* high == host powering us */
	u8 	gpio_vbus_sw;			
	u8	vbus_active_low;	/* vbus polarity */
	u8	vbus_polled;		/* Use polling, not interrupt */
	u8	pullup_pin;		    /* active == D+ pulled up */
	u8	pullup_active_low;	/* true == pullup_pin is active low */
	void __iomem	*regs;
	u8 mode;
};
extern void __init ait_add_otg(struct ait_otg_platform_data *data);
extern void __init ait_add_device_udc(struct ait_udc_data *data);


 /* MMC / SD */
#define DEVICE_NAME_SD0 "ait_sd0"
#define DEVICE_NAME_SD1 "ait_sd1"
#define DEVICE_NAME_SD2 "ait_sd2"

  /* ait mmc platform config */
struct ait_sd_data {
	u8		det_pin;	/* card detect IRQ */
	u8		bus_width;
	u8      pad_id;
	u8      controller_id;
	u8      wire4;
	u8		wp_pin;		    /* (SD) writeprotect detect */
	u8		vcc_pin;	    /* power switching (high == on) */
	bool 	active_low;	
	u32		max_clk_rate;
	bool disable_sdio;
};
extern void __init ait_add_device_mmc(short mmc_id, struct ait_sd_data *data);

 /* Ethernet (EMAC & MACB) */
struct ait_eth_data {
	u32		phy_mask;
	u8		phy_irq_pin;    /* PHY IRQ */
	u8		is_rmii;        /* using RMII interface? */
};
extern void __init ait_add_device_eth(struct ait_eth_data *data);

#define eth_platform_data	ait_eth_data

/* FB DISPLAY */
struct ait_fb_data {
	u8 temp;
};
extern void __init ait_add_device_fb_display(struct ait_fb_data *data);

 /* NAND / SmartMedia */
struct ait_nand_data {
	u8		enable_pin;	/* chip enable */
	u8		det_pin;	/* card detect */
	u8		rdy_pin;	/* ready/busy */
	u8              rdy_pin_active_low;     /* rdy_pin value is inverted */
	u8		ale;		/* address line number connected to ALE */
	u8		cle;		/* address line number connected to CLE */
	u8		bus_width_16;	/* buswidth is 16 bit */
	struct mtd_partition *parts;
	unsigned int	num_parts;
};
extern void __init ait_add_device_nand(struct ait_nand_data *data);

 /* I2C*/
extern void __init ait_add_device_i2c(struct i2c_board_info *devices, int nr_devices,int bus_num);
extern void __init ait_add_adapter_i2c(void);

/* CpuComm */
extern void __init ait_register_cpucomm(void);
extern void __init ait_add_device_cpub_mgr(void);

 /* SPI */
struct iot_spi_platform_data {
	__u16 s2m_irq_gpio;
	__u16 s2m_spi_busy_gpio;	
};

extern void __init ait_add_device_pspi(struct spi_board_info *devices, int nr_devices,int pspi_selection,int pad_selection);
extern void __init ait_add_device_sif(struct spi_board_info *devices, int nr_devices);


 /* Serial */


extern struct platform_device *ait_default_console_device;

typedef enum AIT_UART_STATUS_{
	UART_UNINIT,
	UART_READY,
	UART_CONSOLE,
}AIT_UART_STATUS;
struct ait_uart_data {
	int			num;		/* port num */
	short			use_dma_tx;	/* use transmit DMA? */
	short			use_dma_rx;	/* use receive DMA? */
	void __iomem		*regs;		/* virt. base address, if any */
  //  MMPF_UART_PADSET padset;
	struct serial_rs485	rs485;		/* rs485 settings */
	__u8 				uart_hw_id;
	__u8 				pad;
	__u32				baudrate;
	__u32				fifo_size;
	AIT_UART_STATUS		hw_status; //ait uart status
	AITPS_US 			pUS;	//register address
};

extern void __init ait_register_uart(struct ait_uart_data *uartdata);
extern void __init ait_set_serial_console(unsigned portnr);
extern void __init ait_add_device_serial(void);



 /* LEDs */
extern void __init ait_init_leds(u8 cpu_led, u8 timer_led);
extern void __init ait_gpio_leds(struct gpio_led *leds, int nr);
extern void __init ait_pwm_leds(struct gpio_led *leds, int nr);

/* GPIO */

struct ait_i2c_extension{
	__u8 uI2cmID;	//I2C engine selector
	__u8 pad;		//SCL,SDA pin selector
	//spinlock_t *i2c_hw_spinlock;	//i2c register spinlock , for same i2c engine but different io pad case.
	struct semaphore **i2c_hw_lock;
};

extern uint AIT_GPIO_ETHERNET_RESET;
extern uint AIT_GPIO_WIFI_RESET;
extern uint AIT_GPIO_WIFI_OOB_IRQ;

/* Camera */
void __init ait_add_device_ait_cam(const struct resource *aitcam_res,int num_res);




/* Display */
#define DEVICE_NAME_FB_DISPLAY "ait_fb_display"
#define DEVICE_NAME_FB_RGBLCD "ait_fb_rgblcd"
extern void __init ait_add_device_display(struct ait_display_platform_data_info *data);
extern void __init ait_add_device_fb(short fb_id, struct ait_fb_platform_data_info *data);

/* Misc */
extern void __init ait_add_device_pwm(u32 mask);
extern void __init ait_add_device_aud(int);
extern void __init ait_add_device_aud_i2s(int id);

extern void __init ait_add_device_rtc(void);
extern void __init ait_add_device_watchdog(void);

#endif
