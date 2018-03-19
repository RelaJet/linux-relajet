//==============================================================================
//
//  File        : mmp_register_sd.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng and Philip Lin
//  Revision    : 1.2
//
//==============================================================================

#ifndef _MMP_REG_SD_H
#define _MMP_REG_SD_H

#include    "mmp_register.h"

typedef struct _AITS_SD {
    AIT_REG_B   SD_CTL_0;                                   // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define RISING_EDGE             0x80
        #define FALLING_EDGE            0x00
        #define SLOW_CLK_EN             0x20
        #define SD_DMA_EN               0x10
        #define CLK_IN_SYS_DIV          0x00
        #define CLK_IN_NORM_SYS         0x08
        #define CLK_IN_SYS_DIV_3        0x0C
        #define BUS_WIDTH_8             0x02
        #define BUS_WIDTH_4             0x01
        #define BUS_WIDTH_1             0x00
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_CTL_1;                                   // 0x01
        /*-DEFINE-----------------------------------------------------*/
        #define CLK_EN                  0x80
        #define CLK_DIS                 0x40
        #define AUTO_CLK_EN             0x10
        #define WAIT_LAST_BUSY_EN       0x08
        #define RD_TOUT_EN              0x04
        #define WR_TOUT_EN              0x02
        #define R1B_TOUT_EN             0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_FIFO_CTL;                                // 0x02
    AIT_REG_B   SDIO_ATA_CTL;                               // 0x03
        /*-DEFINE-----------------------------------------------------*/
        #define ATA_EN                  0x10
        #define SDIO_EN                 0x01
        /*------------------------------------------------------------*/
    AIT_REG_W   SD_CLK_CTL;                                 // 0x04
        /*-DEFINE-----------------------------------------------------*/
        #define SD_CLOCK_DIV_2          0x0000
        #define SD_CLOCK_DIV_4          0x0001
        #define SD_CLOCK_DIV_6          0x0002
        #define SD_CLOCK_DIV_8          0x0003
        #define SD_CLOCK_DIV_10         0x0004
        #define SD_CLOCK_DIV_16         0x0007
        #define SD_CLOCK_DIV_128        0x003F
        #define SD_CLOCK_DIV_256        0x007F
        #define SD_CLOCK_DIV_1024       0x01FF
        #define SD_CLOCK_DIV_2048       0x03FF
        #define SD_CLOCK_DIV_1          0x1000
        #define SD_CLOCK_DIV_3          0x2000
        /*------------------------------------------------------------*/
    AIT_REG_W   SD_SLOW_CLK_DIV;                            // 0x06
    AIT_REG_D   SD_CMD_ARG;                                 // 0x08
    AIT_REG_B   SD_CMD_REG_0;                               // 0x0C
        /*-DEFINE-----------------------------------------------------*/
        #define OTHER_RESP              0xC0
        #define R1B_RESP                0x80
        #define R2_RESP                 0x40
        #define NO_RESP                 0x00
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_CMD_REG_1;                               // 0x0D
        /*-DEFINE-----------------------------------------------------*/
        #define SEND_CMD                0x80
        #if (CHIP == MCR_V2)
        #define CONTD_WR_WO_STOP        0x40
        #endif
        #define ADTC_READ               0x02
        #define ADTC_WRITE              0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_CMD_RESP_TOUT_MAX;                       // 0x0E
    #if (CHIP == VSN_V3)
    AIT_REG_B   							_0x0F;
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AIT_REG_B   SD_DMA_BURST_LEN;
        /*-DEFINE-----------------------------------------------------*/
        #define SD_DMA_BYTE_CNT_16X(n)  (n - 1)
        /*------------------------------------------------------------*/
    #endif
    AIT_REG_D   SD_DATA_TOUT;                               // 0x10
    AIT_REG_W   SD_BLK_LEN;                                 // 0x14
    AIT_REG_W   							_0x16;
    AIT_REG_W   SD_BLK_NUM;                                 // 0x18
    AIT_REG_W   							_0x1A;
    AIT_REG_W   SD_BLK_LEFT;                                // 0x1C
    AIT_REG_W   							_0x1E;
    AIT_REG_W   SD_CPU_INT_SR;                              // 0x20
        /*-DEFINE-----------------------------------------------------*/
        #define SDIO_INT                0x1000
        #define DATA_CRC_ERR            0x0800
        #define DATA_TOUT               0x0400
        #define DATA_ST_BIT_ERR         0x0200
        #define DATA_SEND_DONE          0x0100
        #define CE_ATA_CMD_COMPLETE     0x0010
        #define CMD_RESP_CRC_ERR        0x0008
        #define CMD_RESP_TOUT           0x0004
        #define BUSY_TOUT               0x0002
        #define CMD_SEND_DONE           0x0001
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_CPU_FIFO_SR;                             // 0x22
        /*-DEFINE-----------------------------------------------------*/
        #define CMD_BUS                 0x80
        #define CLK_SWITCH_INDICATE     0x10
        #define FIFO_FULL               0x08
        #define FIFO_EMPTY              0x04
        #define FIFO_GE                 0x02
        #define FIFO_LE                 0x01
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_DATA_BUS_SR;                             // 0x23
        /*-DEFINE-----------------------------------------------------*/
        #define DAT7_SR                 0x80
        #define DAT6_SR                 0x40
        #define DAT5_SR                 0x20
        #define DAT4_SR                 0x10
        #define DAT3_SR                 0x08
        #define DAT2_SR                 0x04
        #define DAT1_SR                 0x02
        #define DAT0_SR                 0x01
        /*------------------------------------------------------------*/
    AIT_REG_W   SD_CPU_INT_EN;                              // 0x24
        /*-DEFINE-----------------------------------------------------*/
        // Refer to Offset 0x20
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_CPU_INT_EN_2;                            // 0x26
        /*-DEFINE-----------------------------------------------------*/
        // Refer to Offset 0x22
        /*------------------------------------------------------------*/
    AIT_REG_B   SD_CPU_INT_EN_3;                            // 0x27
        /*-DEFINE-----------------------------------------------------*/
        #define DAT0_INT_EN                 0x01
        /*------------------------------------------------------------*/
    AIT_REG_W   SD_HOST_INT_SR;                             // 0x28
    AIT_REG_B   SD_HOST_INT_SR_2;                           // 0x2A
    AIT_REG_B   SD_HOST_INT_SR_3;                           // 0x2B
    AIT_REG_W   SD_HOST_INT_EN;                             // 0x2C
    AIT_REG_B   SD_HOST_INT_EN_2;                           // 0x2E
    AIT_REG_B   SD_HOST_INT_EN_3;                           // 0x2F

    union _SD_RESP{                                         // 0x30 - 0x3F
    AIT_REG_B   B[16];
    AIT_REG_W   W[8];
    AIT_REG_D   D[4];
    }           SD_RESP;

    AIT_REG_D   SD_DMA_ST_ADDR;                             // 0x40
    AIT_REG_D   SD_FIFO_PORT;                               // 0x44
    AIT_REG_B   SD_FIFO_LEFT;                               // 0x48
    AIT_REG_B   _0x49[0xB6];                                // 0x49 - 0xFF
} AITS_SD, *AITPS_SD;


#if !defined(BUILD_FW)
// SD OPR
#define SD_CTL_0                    (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_CTL_0         )))
#define SD_CTL_1                    (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_CTL_1         )))
#define SD_SLOW_CLK_DIV             (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_SLOW_CLK_DIV  )))
#define SD_CMD_REG_0                (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_CMD_REG_0   )))
#define SD_CMD_REG_1                (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_CMD_REG_1  )))
#define SD_HOST_INT_SR              (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_HOST_INT_SR     )))
#define SD_HOST_INT_SR_2            (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_HOST_INT_SR_2   )))
#define SD_HOST_INT_SR_3            (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_HOST_INT_SR_3  )))
#define SD_CMD_RESP_TOUT_MAX        (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_CMD_RESP_TOUT_MAX  )))

#define SD_CLK_CTL                  (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_CLK_CTL       )))
#define SD_CMD_ARG                  (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_CMD_ARG       )))
#define SD_BLK_LEN                  (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_BLK_LEN         )))
#define SD_BLK_NUM                  (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_BLK_NUM       )))

#define SD_DATA_TOUT                (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_DATA_TOUT         )))

#define SD_RESP_REG_5               (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_RESP.B[5]  )))
#define SD_RESP_REG_6               (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_RESP.W[3]  )))
#define SD_RESP_REG_8               (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_RESP.W[4]  )))
#define SD_RESP_REG_A               (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_RESP.W[5]  )))
#define SD_RESP_REG_C               (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_RESP.D[3]  )))

#define SD_DMA_ST_ADDR              (SD_BASE +(MMP_ULONG)(&(((AITPS_SD )0)->SD_DMA_ST_ADDR  )))



#endif /* !defined(BUILD_FW) */

#endif /* End of _MMP_REG_SD_H */
