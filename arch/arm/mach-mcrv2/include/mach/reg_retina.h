//==============================================================================
//
//  File        : reg_retina.h
//  Description : Retina regsiter definition header file
//  Author      : Jerry Tsao
//  Revision    : 1.0
//
//==============================================================================

#ifndef _REG_RETINA_H_
#define _REG_RETINA_H_

#include "mmp_register.h"

#include "config_fw.h"

//------------------------------
// Register Naming Abbreviation
//------------------------------

// AUD          : audio
// ADJST        : adjust
// ACC          : accumulate
// AF           : auto focus
// AWB          : auto white balance
// AE           : auto exporsure
// AZOOM        : auto zoom
// BLD          : blend
// CTL          : control
// CLK          : clock
// COMPEN       : compensation
// COMP         : compress
// CSR          : control and status register
// CONV         : convert
// CNT          : count
// CFG          : configuration
// COLR         : color
// CMD          : command
// DBL          : double
// DBLK         : deblocking
// DLINE        : delay line
// DNSAMP       : down sample
// DNLD         : download
// DEFT         : defect
// Dn           : divided by n
// DSPY         : display
// EN,DIS       : enable, disable
// END          : finish, the end
// FRM          : frame
// FMT          : format
// FOUT         : FIFO out
// GAMA         : gamma
// GBL          : global
// PANL         : panel
// PAL          : palette
// ENC,DEC      : encode, decode
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
// LPF          : low-pass filter
// LRP          : linear interpolation
// MTX          : matrix
// MDL          : module
// NO           : number id
// NUM          : count number
// NR           : noise reduction
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
// RC,WC        : read cycle, cycle
// RW,RD,WR     : read/write, read, write
// RX           : receive
// RST          : reset
// ROTE         : rotate
// SRC          : source id
// STG          : storage
// ST,ED        : start and end point
// SENSR        : sensor
// SERL,PARL    : serial, parellel
// SR           : status register
// SCAL         : scaling
// SIN,SOUT     : scaling in, scaling out
// SPC          : space
// TBL          : table
// TOUT         : timeout
// TX           : transfer
// UPBND        : upper bound
// UPD          : update
// VID          : video
// WT           : weight
// W,H          : width , height
// WD           : watchdog
// X,Y          : coordinate X, Y
//

//==============================================================================
//
//                              Retina Control Register Definitions
//
//==============================================================================

//-----------------------------
// MCI structure (0x8000 B000)
//-----------------------------
#if 0
typedef struct _AITS_MCI {
    AIT_REG_W   MCI_FB_CTL;                                             // 0x00
    AIT_REG_W                           _x02[7];

    AIT_REG_B   MCI_EXTM_TYPE;
        /*-DEFINE-----------------------------------------------------*/
        #define MCI_EXTM_NONE               0x00
        #define MCI_EXTM_SRAM               0x01
        #define MCI_EXTM_DRAM               0x02
        /*------------------------------------------------------------*/
    AIT_REG_B   MCI_DRAM_FMT;
    AIT_REG_B   MCI_SRAM_PAGE;
    AIT_REG_B   MCI_SRAM_SEL;
    AIT_REG_B   MCI_SRAM_RD_CYC;
    AIT_REG_B   MCI_SRAM_WR_CYC;
    AIT_REG_B   MCI_SRAM_WR_EN_CYC;
    AIT_REG_B   MCI_SRAM_WR_PRECHARGE;
    AIT_REG_B   MCI_SRAM_WR_TIMEING;
    AIT_REG_B                           _x19[7];

    AIT_REG_B   MCI_WT_MAJOR_SRC0;                                      // 0x20
    AIT_REG_B   MCI_WT_MAJOR_SRC1;
    AIT_REG_B   MCI_WT_MAJOR_SRC2;
    AIT_REG_B   MCI_WT_MAJOR_SRC3;
    AIT_REG_B   MCI_WT_MAJOR_SRC4;
    AIT_REG_B   MCI_WT_MAJOR_SRC5;
    AIT_REG_B                           _x26[10];

    AIT_REG_B   MCI_WT_MINOR_SRC0;                                      // 0x30
    AIT_REG_B   MCI_WT_MINOR_SRC1;
    AIT_REG_B   MCI_WT_MINOR_SRC2;
    AIT_REG_B   MCI_WT_MINOR_SRC3;
    AIT_REG_B   MCI_WT_MINOR_SRC4;
    AIT_REG_B   MCI_WT_MINOR_SRC5;
    AIT_REG_B   MCI_WT_MINOR_SRC6;
    AIT_REG_B   MCI_WT_MINOR_SRC7;
    AIT_REG_B   MCI_WT_MINOR_SRC8;
    AIT_REG_B   MCI_WT_MINOR_SRC9;
    AIT_REG_B   MCI_WT_MINOR_SRCA;
    AIT_REG_B                           _x3B[5];
} AITS_MCI, *AITPS_MCI;
#endif

// Peripheral Register of Retina
#ifndef BUILD_HOST

//==============================================================================
//
//                              Retina Peripheral Control Register Definitions
//
//==============================================================================

//------------------------------------------------------
// TC Structure (0xFFFE 0000, 0xFFFE 0040, 0xFFFE 0080)
//------------------------------------------------------
typedef struct _AITS_TC {
    AIT_REG_D   TC_CCR;             // Channel Control Register // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define TC_CLKEN            0x01
        #define TC_CLKDIS           0x02
        #define TC_SWTRG            0x04
        /*------------------------------------------------------------*/
    AIT_REG_D   TC_CMR;             // Channel Mode Register
        /*-DEFINE-----------------------------------------------------*/
        #define TC_CLK_MCK_D2       0x0000
        #define TC_CLK_MCK_D8       0x0001
        #define TC_CLK_MCK_D32      0x0002
        #define TC_CLK_MCK_D128     0x0003
        #define TC_CLK_MCK_D1024    0x0004
        #define TC_CLK_MCK_D4       0x0005
        #define TC_CLK_MCK_D16      0x0006
        #define TC_CLK_MCK          0x0007

        #define TC_CPCTRG           0x4000
        /*------------------------------------------------------------*/
    AIT_REG_D                           _x08[2];
    AIT_REG_D   TC_CVR;             // Counter Value            // 0x10
    AIT_REG_D                           _x14[2];
    AIT_REG_D   TC_RC;              // Register Compare         // 0x1C
    AIT_REG_D   TC_SR;              // Status Register          // 0x20
        /*-DEFINE-----------------------------------------------------*/
        #define TC_COVFS            0x000001    // Counter Overflow Status
        #define TC_CPCS             0x000010    // RC Compare Status
        #define TC_CLKSTA           0x010000    // Clock enable status
        /*------------------------------------------------------------*/
    AIT_REG_D   TC_IER;             // Interrupt Enable Register   0x24
    AIT_REG_D   TC_IDR;             // Interrupt Disable Register  0x28
    AIT_REG_D   TC_IMR;             // Interrupt Mask Register     0x2C
    AIT_REG_D                           _x30[4];                // 0x30~0x3F
} AITS_TC, *AITPS_TC;

typedef struct _AITS_TCB {
    #if (CHIP == VSN_V3)
    AITS_TC     TC[3];              // TC 0,1,2
    #endif
    #if (CHIP == MERCURY)
    AITS_TC     TC[6];              // TC 0,1,2,3,4,5
    #endif
	#if (CHIP == MCR_V2)
    AITS_TC     TC0_2[3];           // TC 0,1,2                 // 0x00~0xBF
	#endif
    AIT_REG_D   TC_BCR;             // Block Control Register   // 0xC0
        /*-DEFINE-----------------------------------------------------*/
        #define TC_SYNC             0x01        // Synchronisation Trigger
        /*------------------------------------------------------------*/
    AIT_REG_D                           _xC4;
    AIT_REG_D   TC_DBR;             // Debug Mode Register      // 0xC8
        /*-DEFINE-----------------------------------------------------*/
        #define TC_DBG_EN           0x01        // Debug Mode Enable
        /*------------------------------------------------------------*/
    AIT_REG_D                           _xCC[13];
	AITS_TC     TC3_5[3];           // TC 3,4,5                 // 0x100~0x1BF
} AITS_TCB, *AITPS_TCB;

//------------------------------------------------------
// WD Structure (0xFFFE 8000)
//------------------------------------------------------
typedef struct _AITS_WD {
    AIT_REG_D   WD_MODE_CTL0;                               // 0x00
        /*-DEFINE-----------------------------------------------------*/
        #define WD_CTL_ACCESS_KEY           0x2340
        #define WD_INT_EN                   0x0004
        #define WD_RST_EN                   0x0002
        #define WD_EN                       0x0001
        /*------------------------------------------------------------*/
    AIT_REG_D   WD_MODE_CTL1;                               // 0x04
        /*-DEFINE-----------------------------------------------------*/
        #define WD_CLK_CTL_ACCESS_KEY       0x3700
        /*------------------------------------------------------------*/
    AIT_REG_D   WD_RE_ST;                                   // 0x08
        /*-DEFINE-----------------------------------------------------*/
        #define WD_RESTART                  0xC071
        /*------------------------------------------------------------*/
    AIT_REG_D   WD_SR;                                      // 0x0C
        /*-DEFINE-----------------------------------------------------*/
        #define WD_RESET_SR                 0x0002
        #define WD_OVF_SR                   0x0001
        /*------------------------------------------------------------*/
} AITS_WD, *AITPS_WD;

//-----------------------------
// AIC Structure (0xFFFF F000)
//-----------------------------
typedef struct _AITS_AIC {
    #if (CHIP == VSN_V3)
    AIT_REG_D   AIC_SMR[32];        // Source Mode Register             // 0x000 - 0x07F
    AIT_REG_D   AIC_SVR[32];        // Source Vector Register           // 0x080 - 0x0FF
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AIT_REG_D   AIC_SMR[64];        // Source Mode Register             // 0x000 - 0x0FF
    #endif
    AIT_REG_D   AIC_IVR;            // IRQ Vector Register              // 0x100
    AIT_REG_D   AIC_FVR;            // FIQ Vector Register              // 0x104
    AIT_REG_D   AIC_ISR;            // Interrupt Status Register        // 0x108
    #if (CHIP == VSN_V3)
    AIT_REG_D   AIC_IPR;            // Interrupt Pending Register       // 0x10C
    AIT_REG_D   AIC_IMR;            // Interrupt Mask Register          // 0x110
    AIT_REG_D   AIC_CISR;           // Core Interrupt Status Register   // 0x114
    AIT_REG_D                           _x118[2];
    AIT_REG_D   AIC_IECR;           // Interrupt Enable Command Register// 0x120
    AIT_REG_D   AIC_IDCR;           // Interrupt Disable Command egister// 0x124
    AIT_REG_D   AIC_ICCR;           // Interrupt Clear Command Register // 0x128
    AIT_REG_D   AIC_ISCR;           // Interrupt Set Command Register   // 0x12C
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AIT_REG_D   AIC_IPR_LSB;        // Interrupt Pending Register 0~31  // 0x10C
    AIT_REG_D   AIC_IMR_LSB;        // Interrupt Mask Register 0~31     // 0x110
    AIT_REG_D   AIC_CISR;           // Core Interrupt Status Register   // 0x114
    AIT_REG_D   AIC_IMR_MSB;        // Interrupt Mask Register 63~32    // 0x118
    AIT_REG_D                           _x11C;
    AIT_REG_D   AIC_IECR_LSB;       // Interrupt Enable Cmd Reg 0~31    // 0x120
    AIT_REG_D   AIC_IDCR_LSB;       // Interrupt Disable Cmd Reg 0~31   // 0x124
    AIT_REG_D   AIC_ICCR_LSB;       // Interrupt Clear Cmd Reg 0~31     // 0x128
    AIT_REG_D   AIC_ISCR_LSB;       // Interrupt Set Cmd Reg 0~31       // 0x12C
    #endif
    AIT_REG_D   AIC_EOICR;          // End of Interrupt Cmd Reg         // 0x130
    AIT_REG_D                           _x134;
    AIT_REG_D   AIC_DBR;            // Debug Mode                       // 0x138
    #if (CHIP == MCR_V2) || (CHIP == MERCURY)
    AIT_REG_D   AIC_IPR_MSB;        // Interrupt Pending Register 32~63 // 0x13C
    AIT_REG_D   AIC_IECR_MSB;       // Interrupt Enable Cmd Reg 32~63   // 0x140
    AIT_REG_D   AIC_IDCR_MSB;       // Interrupt Disable Cmd Reg 32~63  // 0x144
    AIT_REG_D   AIC_ICCR_MSB;       // Interrupt Clear Cmd Reg 32~63    // 0x148
    AIT_REG_D   AIC_ISCR_MSB;       // Interrupt Set Cmd Reg 32~63      // 0x14C
    AIT_REG_D                           _x150[0x2C];
    AIT_REG_D   AIC_SVR[64];        // Source Vector Register           // 0x200 - 0x2FF
    #endif
} AITS_AIC, *AITPS_AIC;

// -------- AIC_SMR[]: Interrupt Source Mode Registers --------
#define AIC_INT_MASK                ((unsigned short)0x0080)            // Interrupt direct to IRQ or FIQ
#define AIC_PRIOR_MASK              ((unsigned short)0x0007)            // Priority
#define AIC_SRCTYPE_MASK            ((unsigned short)0x0060)            // Source Type Definition
// Interrupt Priority
#define AIC_PRIOR_LOWEST            ((unsigned int)0x0000)              // Lowest priority level
#define AIC_PRIOR_HIGHEST           ((unsigned int)0x0007)              // Highest priority level
// Interrupt Direction
#define AIC_INT_TO_FIQ              ((unsigned short)0x0080)            // Interrupt is route to FIQ
#define AIC_INT_TO_IRQ              ((unsigned short)0x0000)            // Interrupt is route to IRQ
// Interrupts Sensibility
#define AIC_SRCTYPE_LOW_LEVEL_SENSITIVE     ((unsigned short)0x0000)    // Low Level
#define AIC_SRCTYPE_NEGATIVE_EDGE_TRIGGERED ((unsigned short)0x0020)    // Negative Edge
#define AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE    ((unsigned short)0x0040)    // High Level
#define AIC_SRCTYPE_POSITIVE_EDGE_TRIGGERED ((unsigned short)0x0060)    // Positive Edge
// -------- AIC_ISR: Interrupt Status Register --------
#define AIC_IRQID_MASK              ((unsigned short)0x1F)              // Current source interrupt

// -------- AIC_CISR: Interrupt Core Status Register --------
#define AIC_NFIQ                    ((unsigned short)0x01)              // Core FIQ Status
#define AIC_NIRQ                    ((unsigned short)0x02)              // Core IRQ Status

// -------- AIC_DBR : ICE Debug Mode --------
#define AIC_DBG_EN                  ((unsigned short)0x1 << 0)          // Debug Mode Enable

// -------- AIC_SMR[], AIC_SVR[]: Interrupt Source --------
#define AIC_SRC_VIF                 0
#define AIC_SRC_ISP                 1
#define AIC_SRC_JPG                 2
#define AIC_SRC_SCAL                3
#define AIC_SRC_GRA                 4
#define AIC_SRC_IBC                 5
#if (CHIP == VSN_V3)
#define AIC_SRC_AFE                 6
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AIT_SRC_RAW                 6
#endif

//7

#define AIC_SRC_UART                8
#define AIC_SRC_TC0                 9
#define AIC_SRC_TC1                 10
#define AIC_SRC_TC2                 11
#define AIC_SRC_USB                 12
#define AIC_SRC_SDRAM               13

#if (CHIP == VSN_V3)
#define AIC_SRC_USBDMA              14
#endif
#if (CHIP == MERCURY)
#define AIC_SRC_USBPHY              14
#endif
#if (CHIP == MCR_V2)
#define AIC_SRC_HDMI_PHY            14
#endif

#if (CHIP == VSN_V3)
#define AIC_SRC_WD                  15
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AIC_SRC_WD_INNER            15
#define AIC_SRC_WD                  15
#endif

#define AIC_SRC_GPIO                16
#define AIC_SRC_I2S                 17
#define AIC_SRC_SD                  18

#if (CHIP == VSN_V3)
#define AIC_SRC_AFE_FIFO            19
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AIC_SRC_CCIR                19
#endif

//20 VSN_V3
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AIC_SRC_I2STIMER            20
#endif

#define AIC_SRC_H264ENC             21
#define AIC_SRC_H264DEC             21
#define AIC_SRC_H264                21

#define AIC_SRC_AUD_FIFO            22
//#define AIC_SRC_AUD_SAMPLE          23
#define AIC_SRC_AUD_I2S   	  23
#define AIC_SRC_I2CM                24

#if (CHIP == VSN_V3)
#define AIC_SRC_SIF                 25
#define AIC_SRC_SPI                 26
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AIC_SRC_SPI                 25
#define AIC_SRC_IRDA                26
#endif

#define AIC_SRC_PWM                 27
#define AIC_SRC_DMA                 28
#define AIC_SRC_GBL                 29
#endif
#define AIC_SRC_SM                  30

//31 VSN_V3
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AIC_SRC_SIF                 31
#define AIC_SRC_TC3                 32
#define AIC_SRC_TC4                 33
#define AIC_SRC_TC5                 34

#if (CHIP == MCR_V2)
#define AIC_SRC_LDC                 35
#endif
//35 MERCURY

#define AIC_SRC_WD_OUTER            36
#define AIC_SRC_CPU2CPU             37
#define AIC_SRC_MCI                 38
#define AIC_SRC_AFE                 39
#define AIC_SRC_AFE_FIFO            40

#if (CHIP == MCR_V2)
#define AIC_SRC_DSPY                41
#define AIC_SRC_HDMI                42
#define AIC_SRC_USBDMA              43
#define AIC_SRC_CALI                44
#endif
#if (CHIP == MERCURY)
#define AIC_SRC_DMIC                41
#define AIC_SRC_DMIC_FIFO           42
#define AIC_SRC_TNR                 43
#define AIC_SRC_MIPI_TX             44
#endif

#endif //(CHIP == MCR_V2) || (CHIP == MERCURY)

#if (CHIP == VSN_V3)
#define AIC_SRC_TC(id)              (AIC_SRC_TC0 + id)
#endif
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AIC_SRC_TC(id)              ((id < 3) ? (AIC_SRC_TC0 + id) : \
                                    (AIC_SRC_TC3 + id - 3))
#endif


//==============================================================================
//
//                              Retina Register Base Address
//
//==============================================================================

#define AITC_BASE_TCB               ((AITPS_TCB )AIT_CPUPHL_P2V(0xFFFE0000))     // TCs      Base Address (Timer control block)
#define AITC_BASE_TC0               ((AITPS_TC  )AIT_CPUPHL_P2V(0xFFFE0000))     // TC0      Base Address
#define AITC_BASE_TC1               ((AITPS_TC  )AIT_CPUPHL_P2V(0xFFFE0040))     // TC1      Base Address
#define AITC_BASE_TC2               ((AITPS_TC  )AIT_CPUPHL_P2V(0xFFFE0080))     // TC2      Base Address
#if (CHIP == MCR_V2) || (CHIP == MERCURY)
#define AITC_BASE_TC3               ((AITPS_TC  )AIT_CPUPHL_P2V(0xFFFE0100))     // TC3      Base Address
#define AITC_BASE_TC4               ((AITPS_TC  )AIT_CPUPHL_P2V(0xFFFE0140))     // TC4      Base Address
#define AITC_BASE_TC5               ((AITPS_TC  )AIT_CPUPHL_P2V(0xFFFE0180))     // TC5      Base Address
#endif
#define AITC_BASE_WD                ((AITPS_WD  )AIT_CPUPHL_P2V(0xFFFF8000))     // WD       Base Address
#define AITC_BASE_AIC               ((AITPS_AIC )AIT_CPUPHL_P2V(0xFFFFF000))     // AIC      Base Address

//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================

//==============================================================================
//
//                              MACROS
//
//==============================================================================

#define ALIGN_02(_a)                ((_a + 0x01) & ~0x01)
#define ALIGN_04(_a)                ((_a + 0x03) & ~0x03)
#define ALIGN_08(_a)                ((_a + 0x07) & ~0x07)
#define ALIGN_16(_a)                ((_a + 0x0F) & ~0x0F)
#define ALIGN_32(_a)                ((_a + 0x1F) & ~0x1F)
#define ALIGN_64(_a)                ((_a + 0x3F) & ~0x3F)

#define FLOOR_08(_a)                (_a & ~0x07)
#define FLOOR_16(_a)                (_a & ~0x0F)
#define FLOOR_32(_a)                (_a & ~0x1F)

#endif // _REG_RETINA_H_
