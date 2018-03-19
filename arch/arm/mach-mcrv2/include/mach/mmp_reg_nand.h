//==============================================================================
//
//  File        : mmp_reg_nand.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPH_REG_NAND_H_
#define _MMPH_REG_NAND_H_

#include "mmp_register.h"

// ********************************
//   SM Controller  Structure (0x8000 A000)
// ********************************
typedef struct _AITS_NAND {
	AIT_REG_B	NAD_CAD;					//0xA000
	AIT_REG_B   	NAD_CTL;		 			//0xA001
		/*---------------------------------------------*/
		#define NAD_RB			0x80	
		#define NAD_CE2			0x40
		#define NAD_CE1			0x20
		#define NAD_WPN			0x10
		#define NAD_ALE			0x08
		#define NAD_CLE			0x04
		#define NAD_READ		0x02
		#define NAD_WRITE		0x00
		#define NAD_EN			0x01
		/*---------------------------------------------*/
	AIT_REG_B	NAD_TIMING;					//0xA002
		/*---------------------------------------------*/
		#define NAD_RCV_CYC(_a)		(_a << 6) & 0xC0	
		#define NAD_CMD_CYC(_a)		(_a << 3) & 0x38	
		/*---------------------------------------------*/
	AIT_REG_B   	NAD_TIMING2;
	AIT_REG_B	NAD_ECC0;					//0xA004
	AIT_REG_B	NAD_ECC1;					
	AIT_REG_B	NAD_ECC2;					
	AIT_REG_B						_x07;
	AIT_REG_B	NAD_ECC3;
	AIT_REG_B	NAD_ECC4;					
	AIT_REG_B	NAD_ECC5;					
	AIT_REG_B						_x0B;
	AIT_REG_D	NAD_DMA_ADDR;					//0xA00C ~ 0F
	AIT_REG_W	NAD_DMA_LEN;					//0xA010 ~ 11
	AIT_REG_B	NAD_DMA_CTL;					//0xA012
		/*---------------------------------------------*/
		#define NAD_GPIO_EN		0x04
		#define	NAD_FB_EN		0x02
		#define NAD_DMA_EN		0x01
		/*---------------------------------------------*/
	AIT_REG_B	_x13;
	AIT_REG_B	NAD_HOST_INT_EN;				//0xA014
	AIT_REG_B	NAD_CPU_INT_EN;					//0xA015
	AIT_REG_B	NAD_HOST_INT_SR;				//0xA016
	AIT_REG_B	NAD_CPU_INT_SR;					//0xA017
		/*-------------------------------------------*/
		#define NAD_DMA_DONE		0x80
		#define NAD_DMA_TO		0x40
		#define NAD_AUTO_ADDR_DONE	0x20
		#define NAD_RB_POS		0x02
		#define NAD_RB_NEG		0x01 
		/*-------------------------------------------*/
	AIT_REG_B						_x18[0x18];
	AIT_REG_B	NAD_CMD_CTL;					//0xA030
		/*-------------------------------------------*/
		#define NAD_CMD1_ADDR_CMD2	0x12
		#define NAD_CMD1_ADDR_DMA_CMD2	0x0A
		#define NAD_CMD1_ADDR_CMD2_DMA	0x06
		#define NAD_CMD1_ADDR_DMA	0x01
		/*-------------------------------------------*/
	AIT_REG_B	NAD_CMD_TIMING;					//0xA031
	AIT_REG_B	NAD_CMD_1;					//0xA032
	AIT_REG_B	NAD_CMD_2;					//0xA033
	AIT_REG_B	NAD_ADDR_CTL;					//0xA034
		/*-------------------------------------------*/
		#define NAD_BYPSS_ADDR		0x10
		#define NAD_ADDR_CYCLE_MASK     0x03
		/*-------------------------------------------*/
	AIT_REG_B	NAD_ADDR_TIMING;				//0xA035
		/*-------------------------------------------*/
		#define NAD_ALS_MASK		0x30
		#define NAD_ALH_MASK		0x03
		/*-------------------------------------------*/
	AIT_REG_B	NAD_ADDR[6];					//0xA036 - A03B
	AIT_REG_W	NAD_TR;						//0xA03C ~ 3D
	AIT_REG_B                       			_x3E;	//0xA03E
	AIT_REG_B	NAD_EXC_CTL;					//0xA03F
		/*------------------------------------------*/
		#define NAD_EXC_ST         	0x01
		/*------------------------------------------*/
	AIT_REG_B	NAD_RS[10];					//0xA040
	AIT_REG_B	NAD_RDN_BYTE;					//0xA04A
	AIT_REG_B	NAD_RDN_CTL;					//0xA04B
		/*-------------------------------------------*/
		#define NAD_RSECC_EN		0x04
		#define NAD_RS_EN		0x01
		/*-------------------------------------------*/
	AIT_REG_B	NAD_CMD_ST_HIGH;				//0xA04C
	AIT_REG_B	NAD_CMD_ST_LOW;					//0xA04D
	AIT_REG_W						_x4E;
	AIT_REG_D	NAD_ABORT_TIMING;				//0xA050 ~ 53
	AIT_REG_D	NAD_ECC_DMA_ADDR;				//0xA054 ~ 57
	AIT_REG_B	NAD_RS_ADDR_SEG;				//0xA058 
	AIT_REG_B	NAD_PAGE_SEG;					//0xA059
	AIT_REG_B	NAD_ECC_NUM;					//0xA05A?
	AIT_REG_B	NAD_ECC_STATE;					//0xA05B
		/*--------------------------------------------*/
		#define NAD_ECC_NO_ERR		0x00
		#define NAD_ECC_TOO_MANY_ERR	0x01
		#define NAD_ECC_CANCORRECT  	0x03
		/*--------------------------------------------*/
	AIT_REG_B	NAD_ECC_SERR;					//0xA05C?
	AIT_REG_B	NAD_ECC_CORR_CTL;           			//0xA05D
	AIT_REG_B						_x5E[2];//0xA05E ~ 5F
	AIT_REG_W	NAD_ERR_ADD[4];					//0xA060 ~ 67
	AIT_REG_W	NAD_ERR_VAL[4];					//0xA068 ~ 6F
} AITS_NAND, *AITPS_NAND;
#endif
