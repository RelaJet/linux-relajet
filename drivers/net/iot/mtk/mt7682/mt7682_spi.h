#ifndef _MT7687_SPI_H
#define _MT7687_SPI_H

#include <linux/spi/spi.h>
#include "../../iot_ctrl.h"
/* Definition of SPI command */


/* SPIS Command */
#define SPIS_H2D_TYPE_DATA (1<<0)
#define SPIS_H2D_TYPE_EVENT (1<<1)
    #define SPIS_CMD_GET_MAC 1
    #define SPIS_CMD_GET_MAC_ETH1 2	
    #define SPIS_CMD_GET_INFO_BUF_ADDR0 (SPIS_CMD_GET_MAC+1)
    #define SPIS_CMD_GET_INFO_BUF_ADDR7 (SPIS_CMD_GET_INFO_BUF_ADDR0+7)
	
    #define SPIS_CMD_GET_RD_BUF_ADDR (SPIS_CMD_GET_INFO_BUF_ADDR7+1)		// 0x0A
    #define SPIS_CMD_GET_WR_BUF_ADDR (SPIS_CMD_GET_RD_BUF_ADDR+1)	
    #define SPIS_CMD_GET_RXINFO (SPIS_CMD_GET_WR_BUF_ADDR+1)
    #define SPIS_CMD_SET_WIFI (SPIS_CMD_GET_RXINFO+1)

    #define SPIS_CMD_GET_LINK (SPIS_CMD_SET_WIFI+1)
    #define SPIS_CMD_GET_INFO (SPIS_CMD_GET_LINK +1)	
    #define SPIS_CMD_SET_POWERMODE (SPIS_CMD_GET_INFO +1)		// 0x10
    #define SPIS_CMD_SET_READ_DONE (SPIS_CMD_SET_POWERMODE +1)		
    #define SPIS_CMD_SET_RX_MODE (SPIS_CMD_SET_READ_DONE +1)		

	//For MT7682
    #define SPIS_CMD_ETH_PKT_TX (SPIS_CMD_SET_RX_MODE+1)		

	//Ethernet PKT Slave to Host	
    #define SPIS_CMD_GET_ETH_PKT (SPIS_CMD_ETH_PKT_TX+1)	
	
// For eth1
    #define SPIS_CMD_GET_INFO_ETH1 (SPIS_CMD_GET_ETH_PKT +1)	
	
#define SPIS_H2D_TYPE_RD_DATA_DONE (1<<2)

#define SPIS_H2D_TYPE_ISR_DONE (1<<3)	

#define SPIS_H2D_TYPE_SET_INFO (1<<4)


struct iot_pkt_head_t {
	u8   cmd;
	u16 pkt_len;
	u16 seq_num;		
	u16 reserve;		

} __attribute__((packed));

#define TX_HEAD_SIZE			sizeof(struct iot_pkt_head_t)


#define SPIS_CFG_RD_CMD         0x0a
	#define SPIS_CFG_RD_CMD_LEN	9
#define SPIS_RD_CMD             0x81
#define SPIS_CFG_WR_CMD         0x0c
	#define SPIS_CFG_WR_CMD_LEN 9
#define SPIS_WR_CMD             0x0e
#define SPIS_RS_CMD             0x06
	#define SPIS_RS_CMD_LEN		1
#define SPIS_PWON_CMD           0x04
#define SPIS_PWOFF_CMD          0x02
#define SPIS_CT_CMD             0x10
#define SPI_TEST_DATA_TX_PATTERN   0x5A
#define SPI_TEST_DATA_RX_PATTERN   0xA5
#define SPIS_ADDRESS_ID         0x55aa0000
#define SPI_TEST_DATA_SIZE      1024
#define SPIS_CFG_LENGTH         (SPI_TEST_DATA_SIZE - 1)
#define SPIS_CFG_4BYTE_ADDR     (0x01 << 2)
#define SPIS_CFG_QUAD_MODE      (0x02 << 0)
#define COMMAND_DELAY           0


#endif /* _MT7687_SPI_H */

