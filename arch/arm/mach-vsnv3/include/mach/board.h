/*
 * arch/arm/mach-at91/include/mach/board.h
 *
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
#include <linux/usb/atmel_usba_udc.h>
#include <linux/atmel-mci.h>
#include <sound/atmel-ac97c.h>
#include <linux/serial.h>

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
	u8	vbus_active_low;	/* vbus polarity */
	u8	vbus_polled;		/* Use polling, not interrupt */
	u8	pullup_pin;		    /* active == D+ pulled up */
	u8	pullup_active_low;	/* true == pullup_pin is active low */
void __iomem	*regs;
//	fsl_usb2_operating_modes operating_mode;
//	fsl_usb2_phy_modes phy_mode;
};
extern void __init ait_add_otg(struct ait_otg_platform_data *data);
extern void __init ait_add_device_udc(struct ait_udc_data *data);

 /* USB High Speed Device */
extern void __init at91_add_device_usba(struct usba_platform_data *data);

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
};
extern void __init ait_add_device_mmc(short mmc_id, struct ait_sd_data *data);

 /* Ethernet (EMAC & MACB) */
struct ait_eth_data {
	u32		phy_mask;
	u8		phy_irq_pin;    /* PHY IRQ */
	u8		is_rmii;        /* using RMII interface? */
};
extern void __init ait_add_device_eth(struct ait_eth_data *data);

#if defined(CONFIG_ARCH_VSNV3AIT845X)
#define eth_platform_data	ait_eth_data
#endif

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
extern void __init ait_add_adapter_i2c();

/* CpuComm */
extern void __init ait_register_cpucomm(void);

 /* SPI */
void __init ait_add_device_pspi(struct spi_board_info *devices, int nr_devices,int pspi_selection,int pad_selection);
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

extern void __init ait_add_device_pwm(u32 mask);

extern void __init ait_add_device_aud(int);

extern void __init ait_add_device_cpub_mgr(void);

extern void __init ait_add_device_rtc(void);

extern void __init ait_add_device_watchdog(void);
	
// /* Touchscreen Controller */
//struct at91_tsadcc_data {
//	unsigned int    adc_clock;
//	u8		pendet_debounce;
//	u8		ts_sample_hold_time;
//};
//extern void __init at91_add_device_tsadcc(struct at91_tsadcc_data *data);


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

#if defined(CONFIG_MACH_VSNV3_AIT8455EVB)
#define AIT_GPIO_CPU_LED            (MMPF_PIO_REG_GPIO4)
#define AIT_GPIO_TIMER_LED          (MMPF_PIO_REG_GPIO59)
#define AIT_GPIO_WIFI_RESET         (MMPF_PIO_REG_GPIO48)
#define AIT_GPIO_WIFI_OOB_IRQ     (MMPF_PIO_REG_GPIO42)
#define AIT_GPIO_I2S_0              (MMPF_PIO_REG_GPIO24)
#define AIT_GPIO_I2S_1              (MMPF_PIO_REG_GPIO25)
#define AIT_GPIO_I2S_2              (MMPF_PIO_REG_GPIO26)
#define AIT_GPIO_I2S_3              (MMPF_PIO_REG_GPIO27)
#endif //defined(CONFIG_MACH_VSNV3_AIT8455EVB)

#if defined(CONFIG_MACH_MCRV2_AIT8428EVB)
#define AIT_GPIO_CPU_LED            (MMPF_PIO_REG_GPIO4)
#define AIT_GPIO_TIMER_LED          (MMPF_PIO_REG_GPIO59)
#define AIT_GPIO_WIFI_RESET         (MMPF_PIO_REG_GPIO53)
#define AIT_GPIO_BT_RESET          (MMPF_PIO_REG_GPIO43)
#define AIT_GPIO_BT_TX			(MMPF_PIO_REG_GPIO60)//PCGPIO28
#define AIT_GPIO_BT_RX			(MMPF_PIO_REG_GPIO61)//PCGPIO29
#define AIT_GPIO_BT_CTX			(MMPF_PIO_REG_GPIO58)//PCGPIO26
#define AIT_GPIO_BT_RTX			(MMPF_PIO_REG_GPIO59)//PCGPIO27 

//#define AIT_GPIO_CGPIO6_WIFI_RESET  (AIT_GPIO_WIFI_RESET)
//#define AIT_GPIO_WIFI_RESET         (MMPF_PIO_REG_GPIO48) 
#define AIT_GPIO_WIFI_OOB_IRQ     (MMPF_PIO_REG_GPIO42)

#define AIT_GPIO_I2S_0              (MMPF_PIO_REG_GPIO24)
#define AIT_GPIO_I2S_1              (MMPF_PIO_REG_GPIO25)
#define AIT_GPIO_I2S_2              (MMPF_PIO_REG_GPIO26)
#define AIT_GPIO_I2S_3              (MMPF_PIO_REG_GPIO27)
#endif //defined(CONFIG_MACH_MCRV2_AIT8428EVB)

#if defined(CONFIG_MACH_MCRV2_LOT8428EVB) //Krypto board
#define AIT_GPIO_POWER_HOLD         (MMPF_PIO_REG_GPIO16)

//#define AIT_GPIO_CPU_LED            (MMPF_PIO_REG_GPIO4)
#define AIT_GPIO_B_LED              (MMPF_PIO_REG_GPIO3)
#define AIT_GPIO_R_LED              (MMPF_PIO_REG_GPIO4)
#define AIT_GPIO_G_LED              (MMPF_PIO_REG_GPIO5)

//#define AIT_GPIO_TIMER_LED          (MMPF_PIO_REG_GPIO59)

#define AIT_GPIO_WIFI_RESET         (MMPF_PIO_REG_GPIO53)
#define AIT_GPIO_BT_RESET          (MMPF_PIO_REG_GPIO43)
#define AIT_GPIO_BT_TX			(MMPF_PIO_REG_GPIO60)//PCGPIO28
#define AIT_GPIO_BT_RX			(MMPF_PIO_REG_GPIO61)//PCGPIO29
#define AIT_GPIO_BT_CTX			(MMPF_PIO_REG_GPIO58)//PCGPIO26
#define AIT_GPIO_BT_RTX			(MMPF_PIO_REG_GPIO59)//PCGPIO27 

//#define AIT_GPIO_CGPIO6_WIFI_RESET  (AIT_GPIO_WIFI_RESET)
//#define AIT_GPIO_WIFI_RESET         (MMPF_PIO_REG_GPIO48) 
#define AIT_GPIO_WIFI_OOB_IRQ     (MMPF_PIO_REG_GPIO42)
/* not in use
#define AIT_GPIO_I2S_0              (MMPF_PIO_REG_GPIO24)
#define AIT_GPIO_I2S_1              (MMPF_PIO_REG_GPIO25)
#define AIT_GPIO_I2S_2              (MMPF_PIO_REG_GPIO26)
#define AIT_GPIO_I2S_3              (MMPF_PIO_REG_GPIO27)
*/
#endif //defined(CONFIG_MACH_MCRV2_AIT8428EVB)


#define AIT_GPIO_SD_DET_IRQ MMPF_PIO_REG_GPIO38		//CGPIO6
#define AIT_GPIO_ETHERNET_IRQ MMPF_PIO_REG_GPIO116
#define AIT_GPIO_ETHERNET_RESET MMPF_PIO_REG_GPIO115



#endif
