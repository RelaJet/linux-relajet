#ifndef _MT7687_SPI_H
#define _MT7687_SPI_H

#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include "iot.h"


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


#define CLR_STATUS_BIT_ALL(status_variable)  (status_variable = 0)
#define SET_STATUS_BIT(status_variable, bit) status_variable |= (1<<(bit))
#define CLR_STATUS_BIT(status_variable, bit) status_variable &= ~(1<<(bit))
#define CLR_STATUS_BIT_ALL(status_variable)   (status_variable = 0)
#define GET_STATUS_BIT(status_variable, bit) (0 != (status_variable & (1<<(bit))))


//#define SPI_BUSY_PIN MMPF_PIO_REG_GPIO31
// Status bits - These are used to set/reset the corresponding bits in 
// given variable
typedef enum{
    STATUS_BIT_NWP_INIT = 0, // If this bit is set: Network Processor is 
                             // powered up
                             
    STATUS_BIT_CONNECTION,   // If this bit is set: the device is connected to 
                             // the AP or client is connected to device (AP)
                             
    STATUS_BIT_IP_LEASED,    // If this bit is set: the device has leased IP to 
                             // any connected client

    STATUS_BIT_IP_AQUIRED,   // If this bit is set: the device has acquired an IP
    
    STATUS_BIT_SMARTCONFIG_START, // If this bit is set: the SmartConfiguration 
                                  // process is started from SmartConfig app

    STATUS_BIT_P2P_DEV_FOUND,    // If this bit is set: the device (P2P mode) 
                                 // found any p2p-device in scan

    STATUS_BIT_P2P_REQ_RECEIVED, // If this bit is set: the device (P2P mode) 
                                 // found any p2p-negotiation request

    STATUS_BIT_CONNECTION_FAILED, // If this bit is set: the device(P2P mode)
                                  // connection to client(or reverse way) is failed

    STATUS_BIT_PING_DONE         // If this bit is set: the device has completed
                                 // the ping operation

}e_StatusBits;


typedef struct {
	uint16_t	type;
#define DATA_TYPE_MAC 1	
#define DATA_TYPE_RD_ADDR 2
#define DATA_TYPE_WR_ADDR 3
	uint16_t status;
	uint16_t	len;
} hal_spi_slave_data_info_t;
#define SPIS_INFO_LEN sizeof(hal_spi_slave_data_info_t)

struct iot_pkt_head_t {
	char pkt_type[4];	
	u16 seq_num;	
	u16 pkt_len;
} __attribute__((packed));

struct iot_cmd_t{
//	struct list_head list;
	struct completion	wait_event;

	struct iot_pkt_head_t PktHdr;	
	u16 cmd;
	u16 len;
	u16 data[128];
} __packed;

struct iot_cmd_resp_t{
	struct list_head list;	
	struct iot_pkt_head_t PktHdr;		
	u16 cmd;
	u16 len;
	u16 data[128];
} __packed;

typedef struct iot_status{
	u8 available_spi2wifi_buf;
}iot_status_t;


typedef struct spi_pkt_iot_info{
	struct iot_pkt_head_t hdr;
	iot_status_t status;
}spi_pkt_iot_info_t;


struct cc3200_data {
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

#define IS_NW_PROCSR_ON(status_variable)     GET_STATUS_BIT(status_variable,\
                                                            STATUS_BIT_NWP_INIT)
#define IS_CONNECTED(status_variable)        GET_STATUS_BIT(status_variable,\
                                                         STATUS_BIT_CONNECTION)
#define IS_IP_LEASED(status_variable)        GET_STATUS_BIT(status_variable,\
                                                           STATUS_BIT_IP_LEASED)
#define IS_IP_ACQUIRED(status_variable)       GET_STATUS_BIT(status_variable,\
                                                          STATUS_BIT_IP_AQUIRED)
#define IS_SMART_CFG_START(status_variable)  GET_STATUS_BIT(status_variable,\
                                                   STATUS_BIT_SMARTCONFIG_START)
#define IS_P2P_DEV_FOUND(status_variable)    GET_STATUS_BIT(status_variable,\
                                                       STATUS_BIT_P2P_DEV_FOUND)
#define IS_P2P_REQ_RCVD(status_variable)     GET_STATUS_BIT(status_variable,\
                                                    STATUS_BIT_P2P_REQ_RECEIVED)
#define IS_CONNECT_FAILED(status_variable)   GET_STATUS_BIT(status_variable,\
                                                   STATUS_BIT_CONNECTION_FAILED)
#define IS_PING_DONE(status_variable)        GET_STATUS_BIT(status_variable,\
                                                           STATUS_BIT_PING_DONE


#define SPI_TX_WORD_BYTES (2)
#define SPI_TX_COUNT 800
#define SPI_BUF_SIZE (SPI_TX_COUNT*SPI_TX_WORD_BYTES)

#define SPI_DMA_BYTES (SPI_TX_COUNT*SPI_TX_WORD_BYTES)
static u8 spi_rxbuf[SPI_BUF_SIZE];

static int inline spi_rw_dma(struct cc3200_data *spi_priv_data,
		const void *txbuf, unsigned int n_tx,
		void *rxbuf, unsigned int n_rx)
{

	int			status;
	struct spi_message	message;
	struct spi_transfer	x[2];
	int i=0;
	unsigned long t1 = jiffies;
					
	struct iot_spi_platform_data* p_iotdata = (struct iot_spi_platform_data*)spi_priv_data->spi->dev.platform_data;

	while(!gpio_get_value(p_iotdata->s2m_spi_busy_gpio))
	{
		i++;
		if (time_after(jiffies, t1 + 1 * HZ)) {//		if(i>10000)
			pr_info("spis busy %d\n",i);
			return -ETIME; 
		}	
		msleep_interruptible(10);
	}

	spi_message_init(&message);
	memset(x, 0, sizeof x);

	x[0].len = SPI_DMA_BYTES;
	spi_message_add_tail(&x[0], &message);

	x[1].len = SPI_DMA_BYTES;
	spi_message_add_tail(&x[1], &message);

	x[0].tx_buf = txbuf;
	x[1].rx_buf = spi_rxbuf;

	/* do the i/o */
	status = spi_sync(spi_priv_data->spi, &message);

	if(rxbuf&&n_rx)
		memcpy(rxbuf, spi_rxbuf,n_rx);

	return status;
}

#endif /* _MT7687_SPI_H */

