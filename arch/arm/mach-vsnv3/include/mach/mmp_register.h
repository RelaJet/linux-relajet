//==============================================================================
//
//  File        : mmp_register.h
//  Description : INCLUDE File for the Retina register map.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMP_REGISTER_H_
#define _MMP_REGISTER_H_

///@ait_only
#include <mach/hardware.h>

#include "config_fw.h"

/** @addtogroup MMPH_reg
@{
*/

typedef volatile unsigned char  AIT_REG_B;
typedef volatile unsigned short AIT_REG_W;
typedef volatile unsigned int   AIT_REG_D;

// ********************************
//   Register Naming Abbreviation
// ********************************

// AUD          : audio
// ADDR         : address (new)
// ADJST        : adjust
// ACC          : accumulate
// AF           : auto focus
// AWB          : auto white balance
// AE           : auto exporsure
// AZOOM        : auto zoom
// BLD          : blend
// BS           : bit stream
// BUF          : buffer
// CFG          : configuration
// CLK          : clock
// CMD          : command
// CNT          : count, counter
// COLR         : color
// COMPEN       : compensation
// COMP         : compress
// CONV         : convert
// CSR          : control and status register
// CTL          : control (new)
// CUR          : current (new)
// DBL          : double
// DBLK         : deblocking
// DEC          : decode (new)
// DLINE        : delay line
// DNSAMP       : down sample
// DNLD         : download
// DEFT         : defect
// Dn           : divided by n
// DSI          : display serial interface
// DSPY         : display
// EN,DIS       : enable, disable
// END          : finish, the end
// EXCL         : exclude, excluding
// EXT          : extension
// FRM          : frame
// FMT          : format
// FOUT         : FIFO out
// GAMA         : gamma
// GBL          : global
// PANL         : panel
// PAL          : palette
// PRED         : prediction (new)
// ENC,DEC      : encode, decode
// FINISH       : finish, consider using DONE if possible(new)
// FREQ         : frequency
// SHFT         : shift
// HSTGRM       : histogram
// H,V          : horizontal, vertical
// INT          : interrupt
// IN,OUT       : input, output
// IDX          : index
// IMG          : image
// LCL          : local
// LS           : lenshading
// LOWBD        : lower bound (new)
// LPF          : low-pass filter
// LRP          : linear interpolation
// MTX          : matrix
// MDL          : module
// NO           : number id
// NUM          : count number
// NR           : noise reduction
// OPT          : option (new)
// OVLY         : overlay
// OFST         : offset
// PWR          : power
// PAR          : parity
// PARM         : paramter
// PHL          : peripheral
// PRIO         : priority
// PREVW        : preview
// POLAR        : polarity
// PST,NEG      : positive, negative
// PIXL         : pixel
// QLTY         : quality
// RES          : resolution (new)
// RC,WC        : read cycle, cycle
// RW,RD,WR     : read/write, read, write
// RX           : receive
// RST          : reset
// ROTE         : rotate
// RSL          : result
// SEL          : select (new)
// SRC          : source id
// STG          : storage
// STS          : status
// ST,ED        : start and end point
// SENSR        : sensor
// SERL,PARL    : serial, parellel
// SR           : status register
// SCAL         : scaling
// SIN,SOUT     : scaling in, scaling out
// SPC          : space
// TBL          : table
// TH           : threshold (new)
// TX           : transfer
// U            : Cb (new)
// UPBND        : upper bound (new)
// UPD          : update
// V            : Cr (new), verticial
// VAL          : value (new)
// VID          : video
// WT           : weight
// W,H          : width , height
// WC           : Write cycle, RC
// WD           : watchdog
// WR           : write
// X,Y          : coordinate X, Y
//
// ADDR         : address
// BD           : bound
// BS           : bit stream
// BUF          : buffer
// CUR          : current
// CTL          : control
// FINISH       : finish, consider using DONE if possible
// HDR          : header
// LOWBD        : lower bound
// OPT          : option
// RES          : resolution
// SEL          : select
// TH           : threshold
// UPBD         : upper bound
// VAL          : value
// ===== naming that is not defined before =====
// BITSTREAM some of them uses BS

// *********************************************************************
//                              Retina Peripheral Base Address
// *********************************************************************

#define AITC_BASE_PHY_OPR               (0x80000000)

#define AITC_BASE_PHY_H264DEC_CTL       (0x80000000) // H264DEC  Base Address
#define AITC_BASE_PHY_H264DEC_REF       (0x80000010)
#define AITC_BASE_PHY_H264DEC_REC       (0x80000060)
#define AITC_BASE_PHY_H264DEC_DBLK      (0x80000070)
#define AITC_BASE_PHY_H264DEC_VLD       (0x80000080)
#define AITC_BASE_PHY_H264DEC_PARSE_EP3 (0x800000F0)
#define AITC_BASE_PHY_H264DEC_MB_DATA0  (0x80000100)
#define AITC_BASE_PHY_H264DEC_MB_DATA1  (0x80000180)
#define AITC_BASE_PHY_H264DEC_DBLK_ROT  (0x800002D8)
#define AITC_BASE_PHY_H264ENC           (0x80000400) // H264ENC  Base Address

#define AITC_BASE_PHY_PWM               (0x80000800) // PWM      Base Address
#define AITC_BASE_PHY_USBCTL            (0x80001000) // USB      Base Address
#define AITC_BASE_PHY_USBDMA            (0x80001400) // USB DMA  Base Address

#if (CHIP == VSN_V3)
#define AITC_BASE_PHY_LS                (0x80001800)
#endif
#if (CHIP == MERCURY)
#define AITC_BASE_PHY_TNR               (0x80003000)
#define AITC_BASE_PHY_RAWPROC2          (0x80003910)
#define AITC_BASE_PHY_IBC               (0x80005600)
#endif
#if (CHIP == MCR_V2)
#define AITC_BASE_PHY_HDMI              (0x80002000) // HDMI     Base Address
#define AITC_BASE_PHY_DSPY              (0x80002800) // LCD      Base Address
#define AITC_BASE_PHY_TV                (0x80002870) // TV       Base Address
#define AITC_BASE_PHY_DDR3              (0x80003000) // DDR3     Base Address
#define AITC_BASE_PHY_SD2               (0x80003500) // SD2      Base Address
#define AITC_BASE_PHY_I2S2_MP		(0x80003600) // I2S2     Base Address
#define AITC_BASE_PHY_I2CS              (0x80003800) // I2CS     Base Address
#define AITC_BASE_PHY_CORE              (0x80003A00) // CPU Core Base Address
#define AITC_BASE_PHY_PLL_JITTER_CAL    (0x80003A60) // PLL JITTER Calculation Base Address
#define AITC_BASE_PHY_SARADC_POR        (0x80003A80) // SARADC POR Base Address
#define AITC_BASE_PHY_RTC               (0x80003A90) // RTC      Base Address
#define AITC_BASE_PHY_HDMI_PHY          (0x80003AA0) // HDMI PHY Base Address
#define AITC_BASE_PHY_CPU_SAHRE         (0x80003AC0) // CPU Share Register Base Address
#define AITC_BASE_PHY_LDC               (0x80004000) // LDC      Base Address
#define AITC_BASE_PHY_LDC_LUT           (0x80004200) // LDC LUT  Base Address
#define AITC_BASE_PHY_SCAL              (0x80004500) // Scaler   Base Address
#define AITC_BASE_PHY_GRA               (0x80005000) // GRA      Base Address
#define AITC_BASE_PHY_PAD               (0x80005100) // PAD      Base Address
#define AITC_BASE_PHY_SPIS              (0x80005200) // SPIS     Base Address
#define AITC_BASE_PHY_UARTB             (0x80005300) // Uart     Base Address
#define AITC_BASE_PHY_GPIOCTL           (0x80005500) // GPIO Conter Base Address
#define AITC_BASE_PHY_IBC               (0x80005600) // IBC      Base Address
#define AITC_BASE_PHY_I2S1              (0x80005A00) // I2S1     Base Address
#define AITC_BASE_PHY_IRDA              (0x80005B00) // IRDA     Base Address
#endif

#define AITC_BASE_PHY_GBL               (0x80005D00) // Global   Base Address
#define AITC_BASE_PHY_NAND              (0x80005E00) // NAND(SM) Base Address
#define AITC_BASE_PHY_SD1               (0x80005F00) // SD1      Base Address
#define AITC_BASE_PHY_VIF               (0x80006000) // VIF      Base Address
#define AITC_BASE_PHY_MIPI              (0x80006110) // MIPI     Base Address

#if (CHIP == MERCURY)
#define AITC_BASE_PHY_VIF_SNR2          (0x800061C0)
#endif

#define AITC_BASE_PHY_JPG               (0x80006200) // JPEG     Base Address
#define AITC_BASE_PHY_SD0               (0x80006300) // SD0      Base Address
//#define AITC_BASE_PHY_SCAL              (0x80006400) // Scaler   Base Address

#if (CHIP == VSN_V3)
#define AITC_BASE_PHY_IBC               (0x80006500)
#endif
#if (CHIP == MCR_V2)
#define AITC_BASE_PHY_DADC_EXT          (0x80006500) // Audio ADC Digital Filter Coefficient  Base Address
#endif

#define AITC_BASE_PHY_GPIO              (0x80006600) // GPIO     Base Address
#define AITC_BASE_PHY_SIF               (0x80006700) // SIF      Base Address

#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_PHY_UARTB             (0x80005C00)
#define AITC_BASE_PHY_UART_BOOT         (0x80006A00)
#define AITC_BASE_PHY_UART0		(0x80006A00)
#define AITC_BASE_PHY_UART1		(0x80005C00)
#endif
#if (CHIP == MCR_V2)
#define AITC_BASE_PHY_HINT              (0x80006800) // HINT     Base Address
#define AITC_BASE_PHY_UART_BOOT         (0x80006A00)
#define AITC_BASE_PHY_UART0		(0x80006A00)
#define AITC_BASE_PHY_UART1		(0x80005C00)
#define AITC_BASE_PHY_UART2		(0x80005400)
#define AITC_BASE_PHY_UART3		(0x80005300)
#endif

#define AITC_BASE_PHY_RAWPROC           (0x80006B00) // RAWPROC  Base Address
#define AITC_BASE_PHY_ICOB              (0x80006C00) // Icon     Base Address
#define AITC_BASE_PHY_PSPI0              (0x80006D00) // PSPI     Base Address
#if (CHIP == MCR_V2)
#define AITC_BASE_PHY_PSPI1              (0x80006D40) // PSPI     Base Address
#define AITC_BASE_PHY_PSPI2              (0x80006D80) // PSPI     Base Address
#endif
#define AITC_BASE_PHY_DRAM              (0x80006E00) // DRAM     Base Address
#define AITC_BASE_PHY_ISP               (0x80007000) // ISP      Base Address

#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AITC_BASE_PHY_BAYERSCAL         (0x80007100) // BAYER SCAL Base Address
#endif
#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_PHY_DFT               (0x80007400)
#endif

#define AITC_BASE_PHY_DMA               (0x80007600) // DMA      Base Address
#define AITC_BASE_PHY_MCI               (0x80007700) // MCI      Base Address
#define AITC_BASE_PHY_I2S0              (0x80007800) // I2S0     Base Address

#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_PHY_GRA               (0x80007900)
#endif

#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_PHY_I2CM              (0x80007A00) // I2CM     Base Address
#define AITC_BASE_PHY_I2CM1		(0x80007A00)
#define AITC_BASE_PHY_I2CM2		(0x80007B00)
#endif
#if (CHIP == MCR_V2)
#define AITC_BASE_PHY_I2CM             	(0x80007A00) // I2CM     Base Address
#define AITC_BASE_PHY_I2CM0             (0x80007A00) // I2CM     Base Address
#define AITC_BASE_PHY_I2CM1		(0x80007B00)
#define AITC_BASE_PHY_I2CM2		(0x80007A80)
#define AITC_BASE_PHY_I2CM3             (0x80007B80) // I2CM     Base Address
#endif

#define AITC_BASE_PHY_TBL_Q             (0x80007C00) // Q-Table  Base Address

#if (CHIP == MCR_V2)
#define AITC_BASE_PHY_RAWPROC1          (0x80007E10) // RAWPROC1 Base Address
#endif
#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_PHY_RAWPROC1          (0x80007E10)
#endif

#define AITC_BASE_PHY_AUD               (0x80007F00) // AFE0     Base Address
#define AITC_BASE_PHY_ICON_LUT		(0x80013000) // Icon LUT	Base Address
#define AITC_BASE_PHY_RAW_OFST_TBL      (0x80013400) // Raw Offset Table  Base Address

#define AITC_BASE_PHY_WD                (0xFFFF8000) // WD       Base Address
#define AITC_BASE_PHY_I2S2_MP			(0x80003600) // I2S2     Base Address


// *********************************************************************
//            Physical Address to Virtual Address Mapping
// *********************************************************************

#define AITC_BASE_OPR               (((AIT_REG_B*)                  AIT_OPR_P2V(AITC_BASE_PHY_OPR))
#define AITC_BASE_H264DEC_CTL       ((AITPS_H264DEC_CTL) 			AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_CTL))
#define AITC_BASE_H264DEC_REF       ((AITPS_H264DEC_REF) 			AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_REF))
#define AITC_BASE_H264DEC_REC       ((AITPS_H264DEC_REC)			AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_REC))
#define AITC_BASE_H264DEC_DBLK      ((AITPS_H264DEC_DBLK)			AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_DBLK))
#define AITC_BASE_H264DEC_VLD       ((AITPS_H264DEC_VLD)			AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_VLD))
#define AITC_BASE_H264DEC_PARSE_EP3	((AITPS_H264DEC_PARSE_EP3)		AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_PARSE_EP3))
#define AITC_BASE_H264DEC_MB_DATA0	((AITPS_H264DEC_MB_DATA)		AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_MB_DATA0))
#define AITC_BASE_H264DEC_MB_DATA1	((AITPS_H264DEC_MB_DATA)		AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_MB_DATA1))
#define AITC_BASE_H264DEC_DBLK_ROT  ((AITPS_H264DEC_DBLK_ROT)       AIT_OPR_P2V(AITC_BASE_PHY_H264DEC_DBLK_ROT))
#define AITC_BASE_H264ENC           ((AITPS_H264ENC)                AIT_OPR_P2V(AITC_BASE_PHY_H264ENC))

#define AITC_BASE_PWM				((AITPS_PWM)    		        AIT_OPR_P2V(AITC_BASE_PHY_PWM ))
#define AITC_BASE_USBCTL			((AITPS_USB_CTL)		        AIT_OPR_P2V(AITC_BASE_PHY_USBCTL))
#define AITC_BASE_USBDMA			((AITPS_USB_DMA)	            AIT_OPR_P2V(AITC_BASE_PHY_USBDMA))

#if (CHIP == VSN_V3)
#define AITC_BASE_LS				((AIT_REG_B*)		            AIT_OPR_P2V(AITC_BASE_PHY_LS))
#endif

#if (CHIP == MERCURY)
#define AITC_BASE_TNR				((AITPS_TNR)	                AIT_OPR_P2V(AITC_BASE_PHY_TNR))
#define AITC_BASE_RAWPROC2          ((AITPS_RAWPROC2)               AIT_OPR_P2V(AITC_BASE_PHY_RAWPROC2))
#define AITC_BASE_IBC               ((AITPS_IBC)                    AIT_OPR_P2V(AITC_BASE_PHY_IBC))
#endif

#if (CHIP == MCR_V2)
#define AITC_BASE_HDMI              ((AITPS_HDMI)       AIT_OPR_P2V(AITC_BASE_PHY_HDMI))
#define AITC_BASE_DSPY              ((AITPS_DSPY)       AIT_OPR_P2V(AITC_BASE_PHY_DSPY))
#define AITC_BASE_TV                ((AITPS_TV)         AIT_OPR_P2V(AITC_BASE_PHY_TV))
#define AITC_BASE_DDR3              ((AITPS_DRAM_DDR3)  AIT_OPR_P2V(AITC_BASE_PHY_DDR3))
#define AITC_BASE_SD2               ((AITPS_SD)         AIT_OPR_P2V(AITC_BASE_PHY_SD2))
#define AITC_BASE_I2CS              ((AITPS_I2CS)       AIT_OPR_P2V(AITC_BASE_PHY_I2CS))
#define AITC_BASE_CORE              ((AITPS_CORE)       AIT_OPR_P2V(AITC_BASE_PHY_CORE))
#define AITC_BASE_SARADC_POR        ((AITPS_SARADC_POR) AIT_OPR_P2V(AITC_BASE_PHY_SARADC_POR))
#define AITC_BASE_RTC               ((AITPS_RTC)        AIT_OPR_P2V(AITC_BASE_PHY_RTC))
#define AITC_BASE_HDMI_PHY          ((AITPS_HDMI_PHY)   AIT_OPR_P2V(AITC_BASE_PHY_HDMI_PHY))
#define AITC_BASE_CPU_SAHRE         ((AITPS_CPU_SHARE)  AIT_OPR_P2V(AITC_BASE_PHY_CPU_SAHRE))
#define AITC_BASE_LDC               ((AITPS_LDC)        AIT_OPR_P2V(AITC_BASE_PHY_LDC))
#define AITC_BASE_LDC_LUT           ((AITPS_LDC_LUT)    AIT_OPR_P2V(AITC_BASE_PHY_LDC_LUT))
#define AITC_BASE_GRA               ((AITPS_GRA)        AIT_OPR_P2V(AITC_BASE_PHY_GRA))
#define AITC_BASE_PAD               ((AITPS_PAD)        AIT_OPR_P2V(AITC_BASE_PHY_PAD))
#define AITC_BASE_SPIS              ((AITPS_SPIS)       AIT_OPR_P2V(AITC_BASE_PHY_SPIS))
#define AITC_BASE_UARTB             ((AITPS_UARTB)      AIT_OPR_P2V(AITC_BASE_PHY_UARTB))
#define AITC_BASE_GPIOCTL           ((AITPS_GPIO_CNT)    AIT_OPR_P2V(AITC_BASE_PHY_GPIOCTL))
#define AITC_BASE_IBC               ((AITPS_IBC)        AIT_OPR_P2V(AITC_BASE_PHY_IBC))
#define AITC_BASE_I2S1              ((AITPS_I2S)        AIT_OPR_P2V(AITC_BASE_PHY_I2S1))
#define AITC_BASE_IRDA              ((AITPS_IRDA)       AIT_OPR_P2V(AITC_BASE_PHY_IRDA))
#endif

#define AITC_BASE_GBL		        ((AITPS_GBL )   	AIT_OPR_P2V(AITC_BASE_PHY_GBL))
#define AITC_BASE_NAND              ((AITPS_NAND)       AIT_OPR_P2V(AITC_BASE_PHY_NAND))
#define AITC_BASE_SD1		        ((AITPS_SD)		    AIT_OPR_P2V(AITC_BASE_PHY_SD1))
#define AITC_BASE_VIF		        ((AITPS_VIF )   	AIT_OPR_P2V(AITC_BASE_PHY_VIF))
#define AITC_BASE_MIPI		        ((AITPS_MIPI)   	AIT_OPR_P2V(AITC_BASE_PHY_MIPI))

#if (CHIP == MERCURY)
#define AITC_BASE_VIF_SNR2          ((AITPS_VIF_SNR2)   AIT_OPR_P2V(AITC_BASE_PHY_VIF_SNR2))
#endif

#define AITC_BASE_JPG		        ((AITPS_JPG )   	AIT_OPR_P2V(AITC_BASE_PHY_JPG))
#define AITC_BASE_SD0		        ((AITPS_SD)     	AIT_OPR_P2V(AITC_BASE_PHY_SD0))
#define AITC_BASE_SCAL		        ((AITPS_SCAL)	    AIT_OPR_P2V(AITC_BASE_PHY_SCAL))

#if (CHIP == VSN_V3)
#define AITC_BASE_IBC		        ((AITPS_IBC )		AIT_OPR_P2V(AITC_BASE_PHY_IBC))
#endif

#if (CHIP == MCR_V2)
#define AITC_BASE_DADC_EXT          ((AITPS_DADC_EXT)   AIT_OPR_P2V(AITC_BASE_PHY_DADC_EXT))
#endif

#define AITC_BASE_GPIO		        ((AITPS_GPIO)   	AIT_OPR_P2V(AITC_BASE_PHY_GPIO))
#define AITC_BASE_SIF		        ((AITPS_SIF)		AIT_OPR_P2V(AITC_BASE_PHY_SIF))

#if (CHIP == MCR_V2)
#define AITC_BASE_HINT              ((AITPS_HINT)       AIT_OPR_P2V(AITC_BASE_PHY_HINT))
#endif

#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_UARTB	            ((AITPS_UARTB)      AIT_OPR_P2V(AITC_BASE_PHY_UARTB))
#endif
#define AITC_BASE_UART_BASE	((AITPS_US)	AIT_OPR_P2V(AITC_BASE_PHY_UARTB))//UART base address
#define AITC_BASE_UART0		((AITPS_US)	AIT_OPR_P2V(AITC_BASE_PHY_UART0)) // US0 Base Address
#define AITC_BASE_UART1		((AITPS_US)	AIT_OPR_P2V(AITC_BASE_PHY_UART1)) // US0 Base Address
#define AITC_BASE_UART2		((AITPS_US)	AIT_OPR_P2V(AITC_BASE_PHY_UART2)) // US0 Base Address
#define AITC_BASE_UART3		((AITPS_US)	AIT_OPR_P2V(AITC_BASE_PHY_UART3)) // US0 Base Address

#define AITC_BASE_RAWPROC	        ((AITPS_RAWPROC)    AIT_OPR_P2V(AITC_BASE_PHY_RAWPROC))
#define AITC_BASE_ICOB		        ((AITPS_ICOB)	    AIT_OPR_P2V(AITC_BASE_PHY_ICOB))


#define AITC_BASE_SPIB		        ((AITPS_SPIB) 	    AIT_OPR_P2V(AITC_BASE_PHY_PSPI0))
#define AITC_BASE_PSPI0		        ((AITPS_SPIB) 	    AIT_OPR_P2V(AITC_BASE_PHY_PSPI0))
#if (CHIP==MCR_V2)
#define AITC_BASE_PSPI1		        ((AITPS_SPIB) 	    AIT_OPR_P2V(AITC_BASE_PHY_PSPI1))
#define AITC_BASE_PSPI2		        ((AITPS_SPIB) 	    AIT_OPR_P2V(AITC_BASE_PHY_PSPI2))
#endif

#define AITC_BASE_DRAM		        ((AITPS_DRAM)	    AIT_OPR_P2V(AITC_BASE_PHY_DRAM))
#define AITC_BASE_ISP		        ((AITPS_ISP )		AIT_OPR_P2V(AITC_BASE_PHY_ISP))

#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AITC_BASE_BAYERSCAL         ((AITPS_BAYER_SCAL)	AIT_OPR_P2V(AITC_BASE_PHY_BAYERSCAL))
#endif

#define AITC_BASE_RAWPROC1          ((AITPS_RAWPROC1)	AIT_OPR_P2V(AITC_BASE_PHY_RAWPROC1))

#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_DFT		        ((AIT_REG_B*)	    AIT_OPR_P2V(AITC_BASE_PHY_DFT))
#endif

#define AITC_BASE_DMA		        ((AITPS_DMA)		AIT_OPR_P2V(AITC_BASE_PHY_DMA))
#define AITC_BASE_MCI		        ((AITPS_MCI)		AIT_OPR_P2V(AITC_BASE_PHY_MCI))
#define AITC_BASE_I2S0              ((AITPS_I2S)        AIT_OPR_P2V(AITC_BASE_PHY_I2S0))
#if (CHIP == MERCURY) || (CHIP == VSN_V3)
#define AITC_BASE_GRA		        ((AITPS_GRA )	    AIT_OPR_P2V(AITC_BASE_PHY_GRA))
#define AITC_BASE_PAD		        ((AITPS_PAD)		AIT_OPR_P2V(AITC_BASE_PHY_PAD))
#endif
#define AITC_BASE_I2CM		        ((AITPS_I2CM)	    AIT_OPR_P2V(AITC_BASE_PHY_I2CM))
#define AITC_BASE_TBL_Q	            ((AIT_REG_B*)	    AIT_OPR_P2V(AITC_BASE_PHY_TBL_Q))
#define AITC_BASE_AUD		        ((AITPS_AUD)	    AIT_OPR_P2V(AITC_BASE_PHY_AUD))
#define AITC_BASE_AFE		        AITC_BASE_AUD
#define AITC_BASE_I2S2_MP		 ((AITPS_I2S)           AIT_OPR_P2V(AITC_BASE_PHY_I2S2_MP)) // I2S2     Base Address

/// @}

#endif // _MMPH_HIF_H_
///@end_ait_only
