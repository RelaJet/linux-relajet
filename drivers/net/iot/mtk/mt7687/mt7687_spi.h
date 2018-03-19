#ifndef _MT7687_SPI_H
#define _MT7687_SPI_H

#include <linux/spi/spi.h>
#include "iot.h"
/* Definition of SPI command */
#define MT7687_SPI_WRITE 0x80
#define MT7687_SPI_READ  0x00

#define CM4_SPISLAVE_BASE_AHB (CM4_SPISLAVE_BASE+0x700)

#define SPIS_REG_DATA_RD			( 0x00)
#define SPIS_REG_DATA_WR			(0x04)
#define SPIS_REG_ADDR				(0x08)
#define SPIS_REG_BUS_ACCESS		(0x0c)
#define SPIS_REG_STATUS			(0x10)
#define SPIS_REG_SWIRQ				(0x14)
#define SPIS_REG_MAILBOX_D2H		(0x18)
#define SPIS_REG_MAILBOX_H2D		(0x1c)	// Host to Device received Mail Box

/* SPIS_REG_SIZE Reg */
#define SPIS_BUS_ACCESS_WORD (0x2UL << 1)
#define SPIS_BUS_ACCESS_READ (0x0UL << 0)
#define SPIS_BUS_ACCESS_WRITE (0x1UL << 0)

/* SPIS_REG_STATUS Reg */
#define SPIS_STATUS_READY            0x00000000
#define SPIS_STATUS_BUSY            0x00000001

/* SPIS_REG_SWIRQ Reg */
#define SPIS_SWIRQ_ACTIVE (0x1UL)
#define SPIS_SWIRQ_CLEAR (0x1UL)	// Write clear

/* SPIS_REG_CONFIG Reg */
#define SPIS_CONFIG_MODE_ADDR_AUTO_INC (0x1UL<<4)

#define SPIS_CONFIG_MODE_CPOL_H (0x1UL<<1)		/* clock polarity */
#define SPIS_CONFIG_MODE_CPOL_L (0x0UL<<1)

	
#define SPIS_CONFIG_MODE_CPHA_H (0x1UL<<0)		/* clock phase */
#define SPIS_CONFIG_MODE_CPHA_L (0x0UL<<0)

/* SPIS Command */
#define SPIS_H2D_TYPE_DATA (1<<0)
#define SPIS_H2D_TYPE_EVENT (1<<1)
    #define SPIS_CMD_GET_MAC 1
    #define SPIS_CMD_GET_INFO_BUF_ADDR0 (SPIS_CMD_GET_MAC+1)
    #define SPIS_CMD_GET_INFO_BUF_ADDR7 (SPIS_CMD_GET_INFO_BUF_ADDR0+7)
	
    #define SPIS_CMD_GET_RD_BUF_ADDR (SPIS_CMD_GET_INFO_BUF_ADDR7+1)		// 10
    #define SPIS_CMD_GET_WR_BUF_ADDR (SPIS_CMD_GET_RD_BUF_ADDR+1)	
    #define SPIS_CMD_GET_RXINFO (SPIS_CMD_GET_WR_BUF_ADDR+1)
    #define SPIS_CMD_SET_WIFI (SPIS_CMD_GET_RXINFO+1)

    #define SPIS_CMD_GET_LINK (SPIS_CMD_SET_WIFI+1)
    #define SPIS_CMD_GET_INFO (SPIS_CMD_GET_LINK +1)	
    #define SPIS_CMD_SET_POWERMODE (SPIS_CMD_GET_INFO +1)		
    #define SPIS_CMD_SET_READ_DONE (SPIS_CMD_SET_POWERMODE +1)		
    #define SPIS_CMD_SET_RX_MODE (SPIS_CMD_SET_READ_DONE +1)		
	
#define SPIS_H2D_TYPE_RD_DATA_DONE (1<<2)

#define SPIS_H2D_TYPE_ISR_DONE (1<<3)	

#define SPIS_H2D_TYPE_SET_INFO (1<<4)


/* SPIS Status D2H */
#define SPIS_D2H_TYPE_DATA_READY (1<<0)	
#define SPIS_D2H_STATUS_INFO_READY (1<<5)	

/* ISR to Host */
#define SPIS_D2H_ISR_PKT_RECV (1<<1)
#define SPIS_D2H_ISR_EVENT (1<<2)

typedef struct {

    u32 spis_data_rd;
    u32 spis_data_wr;
    u32 spis_addr;
    u32 spis_bus_access;
    u32 spis_status;
    u32 spis_swirq;
    u32 spis_mailbox_d2h;
    u32 spis_mailbox_h2d;	
    u32 spis_config;
}spis_cr_info;

typedef struct {
	uint16_t	type;
#define DATA_TYPE_MAC 1	
#define DATA_TYPE_RD_ADDR 2
#define DATA_TYPE_WR_ADDR 3
	uint16_t status;
	uint16_t	len;
} hal_spi_slave_data_info_t;
#define SPIS_INFO_LEN sizeof(hal_spi_slave_data_info_t)

struct mt7687_data {
	struct spi_device	*spi;
	struct spi_message	rx_msg;
	struct spi_transfer	spi_rx_xfer[2];
	u8			cmd_buf[6];
	u8			comp;
	u32 info_addr;	
	u32 rd_addr;
	
#define NUM_OF_TX_Q 16	
	u32 wr_addr[NUM_OF_TX_Q];
};

struct spi_status {
	u16 isr;
	u8 status;
};


extern u16  mt7687_read_reg (struct mt7687_data *mt_spi, u8 reg, u16* val);
extern u16  mt7687_write_reg (struct mt7687_data *mt_spi, u16 reg, u16 value);

extern u16  mt7687_bus_access_read (struct mt7687_data *mt_spi);

extern u16  mt7687_bus_access_write (struct mt7687_data *mt_spi);
extern int   mt7687_write_address (struct mt7687_data *mt_spi, u32 addr );


extern int  mt7687_write_fifo (struct mt7687_data *mt_spi, u32 data );
#define mt7687_write_cmd mt7687_write_fifo

extern int  mt7687_read_fifo(struct mt7687_data *mt_spi, u32* data );


extern u16  mt7687_set_d2h_flag(struct mt7687_data *mt_spi, u16 flag);

extern u16  mt7687_clear_d2h_flag(struct mt7687_data *mt_spi, u16 flag);

extern u16  mt7687_get_d2h_flag(struct mt7687_data *mt_spi, u16* flag);

extern u16  mt7687_set_h2d_flag(struct mt7687_data *mt_spi, u16 flag);

extern u16  mt7687_clear_h2d_flag(struct mt7687_data *mt_spi, u16 flag);

extern u16  mt7687_get_h2d_flag(struct mt7687_data *mt_spi, u16* flag);
extern u16  mt7687_activate_swirq (struct mt7687_data *mt_spi);


extern u16  mt7687_read_status (struct mt7687_data *mt_spi, u16* status);

extern void mt7687_dump_regs (struct mt7687_data *mt_spi);

extern int mt7687_send_data (struct mt7687_data *mt_spi,int id, void *data, int len);
extern int mt7687_read_data (struct mt7687_data *mt_spi, u16 type, void *data, int len, int finished);
extern int mt7687_send_event(struct mt7687_data *mt_spi, u32 event);
extern int mt7687_read_spis_info_addr(struct mt7687_data *mt_spi, u32 *addr );
extern int mt7687_read_spis_rd_addr(struct mt7687_data *mt_spi, u32 *addr );
extern int mt7687_read_spis_wr_addr(struct mt7687_data *mt_spi, u32 *addr ,int num_queue);
extern int mt7687_read_spis_mac(struct mt7687_data *mt_spi, char *addr );
extern int mt7687_set_wifi(struct mt7687_data *mt_spi, iot_wifi_setting_t *wifisetting);
extern int iot_set_rx_mode(struct mt7687_data *mt_spi, u16 mode);
extern int mt7687_rx_done(struct mt7687_data *mt_spi, u16 seq);
extern int mt7687_read_linkstatus(struct mt7687_data *mt_spi, u32 *link );
extern int mt7687_iot_get_info(struct mt7687_data *mt_spi, mt7687_dev_info_t *info );
extern int mt7687_iot_set_power(struct mt7687_data *mt_spi, u32 mode );

extern int mt7687_iot_buf_send(struct mt7687_data *mt_spi, int ready_buf_mask , int* sent_time );
extern int mt7687_iot_buf_prepare(struct mt7687_data *mt_spi, u32 dest_buf_id, struct sk_buff *tx_skb );
#endif /* _MT7687_SPI_H */

