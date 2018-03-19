#ifndef _VSNV3_UDC_
#define _VSNV3_UDC_

#include <linux/usb/gadget.h>

#define VSNV3_UDC_DEBUG

#define VSNV3_MAX_NUM_EP 8

#define WSTR(x) size_t(x) 

#define CBW_STATE       00
#define CSW_STATE       01
#define TX_STATE        02
#define RX_STATE        03
#define WP_STATE        04

#define VENDOR_REQUEST      0x10
#define USB_DUMMY       0x00

//**********************************************************************************
//
//**********************************************************************************
#define VENDOR_CMD              0x30
#define VENDOR_IN               0x31


//**********************************************************************************
//         USB Vendor Command for PC Sync
//**********************************************************************************
#define VCMD_READ_REGISTER       0x00
#define VCMD_WRITE_REGISTER      0x01
#define VCMD_GET_MODE            0x10
#define VCMD_SET_BULK_IN          0x20
#define VCMD_SET_BULK_OUT         0x22
#define VCMD_SIZE_BULK_IN         0x24
#define VCMD_SIZE_BULK_OUT        0x26
#define VCMD_NOTIFY_BULK_IN_OK     0x30
#define VCMD_NOTIFY_BULK_OUT_OK    0x32


////////////////////////////////////////////////////////////////////////////////////
// for pcsync directly
#define VCMD_READ_PCSYNC         0x52
#define VCMD_WRITE_PCSYNC        0x54


//**********************************************************************************
//
//**********************************************************************************
#define GetROMByteCmd       0x06
#define GetDataCmd              0x15
#define SendDataCmd         0x16
#define SnapCmd                 0x2D
#define UpdateFlashDataCmd  0x38
#define UpdateFWCmd         0x39
#define ATCmd                   0x41
#define ChkIDCmd            0x55
#define SendIDCmd           0x56
#define SetRegCmd               0x81
#define SetResoCmd          0x8A
#define SetDevModeCmd       0x8C
#define SetBuffAccessAddrCmd    0xA0
#define GetBuffAccessSizeCmd    0xA1
#define GetRegCmd               0xF9

//***************************************************************
#define StdCmd                  0x60
#define GetStatusCmd            0x60
#define ClearFeatureCmd         0x61
#define SetFeatureCmd           0x63
#define SetAddressCmd           0x65
#define GetDescriptorCmd        0x66
#define SetDescriptorCmd        0x67
#define GetConfigCmd            0x68
#define SetConfigCmd            0x69
#define GetInterfaceCmd         0x6A
#define SetInterfaceCmd         0x6B
#define SynchFrameCmd           0x6C

#define GetDevDescrCmd          0x6D
#define GetConfigDescrCmd       0x6E
#define GetStringDescrCmd       0x6F

#define GET_STATUS              0x00
#define CLEAR_FEATURE           0x01
#define SET_FEATURE             0x03
#define SET_ADDRESS             0x05
#define GET_DESCRIPTOR          0x06
#define SET_DESCRIPTOR          0x07
#define GET_CONFIGURATION       0x08
#define SET_CONFIGURATION       0x09
#define GET_INTERFACE           0x0A
#define SET_INTERFACE           0x0B
#define SYNCH_FRAME             0x0C

#define DEVICE_DESCR            0x01
#define CONFIG_DESCR            0x02
#define STRING_DESCR            0x03
#define INTERFACE_DESCR         0x04
#define ENDPOINT_DESCR          0x05

#define DEVICE_REMOTE_WAKEUP    0x01
#define ENDPOINT_STALL          0x00

#define ClassCmd        0xF0
#define BotResetCmd     0xFF
#define BotGetMaxLunCmd     0xFE
#define CANCEL_REQUEST_COMMAND      0x64
#define GET_DEVICE_STATUS_COMMAND   0x67
#define CancelRequestCmd        0xF2
#define GetDeviceStatusCmd      0xF3
//*************************************************************************
//  USB registers
//*************************************************************************
#define USB_BASE_ADDR       0x8000A800

#define USB_BASE_B ((volatile unsigned char *)USB_BASE_ADDR)
#define USB_BASE_W ((volatile unsigned short *)USB_BASE_ADDR)
#define USB_BASE_L ((volatile unsigned long *)USB_BASE_ADDR)
#define FB_BASE_B   ((volatile unsigned char *)0)
#define FB_BASE_W   ((volatile unsigned short *)0)
#define FB_BASE_L   ((volatile unsigned long *)0)



//=====================================================================
#define USBHOSTINTENABLE1       0x00     // USB interrupt enable register 1
#define USBHOSTINTENABLE2       0x01   // USB interrupt enable register 2
#define USBHOSTINTSTATUS1       0x02   // USB interrupt status register 1
#define USBHOSTINTSTATUS2       0x03   // USB interrupt status register 2
#define USBARMINTENABLE1        0x04     // USB interrupt enable register 1
#define USBARMINTENABLE2        0x05   // USB interrupt enable register 2
#define USBARMINTSTATUS1        0x06   // USB interrupt status register 1
#define USBARMINTSTATUS2        0x07   // USB interrupt status register 2

#define USBEP0TXDMAEn           0x08     // USB DMA Enable register
#define USBEP0RXDMAEn           0x09     // USB DMA Enable register
#define USBEP1TXDMAEn           0x0A     // USB DMA Enable register
#define USBEP2RXDMAEn           0x0B     // USB DMA Enable register

#define USBDEVICEADDR           0x0C     // USB device address register
#define USBDEVICECTRL           0x0D     // USB device control register

#define USBTESTMODE             0x0E    // USB test mode control register

#define USBINTSEL               0x0F   // USB Every/Last Packet interrupt Select register
#define USBEP0CTRL              0x10   // USB endpoint control register
#define USBEP0TXSTATUS          0x11   // USB endpoint tx status
#define USBEP0RXSTATUS          0x12   // USB endpoint rx status
#define USBEP1CTRL              0x14   // USB endpoint control register
#define USBEP1TXSTATUS          0x15   // USB endpoint tx status
#define USBEP1RXSTATUS          0x16   // USB endpoint rx status
#define USBEP2CTRL              0x18   // USB endpoint control register
#define USBEP2TXSTATUS          0x19   // USB endpoint tx status
#define USBEP2RXSTATUS          0x1A   // USB endpoint rx status
#define USBEP3CTRL              0x1C   // USB endpoint control register
#define USBEP3TXSTATUS          0x1D   // USB endpoint tx status
#define USBEP3RXSTATUS          0x1E   // USB endpoint rx status

#define USBEP0TXSTARTADDR0      (0x20>>2)   // USB tx start address Low
//#define   USBEP0TXSTARTADDR1      0x21   // USB tx start address High
//#define   USBEP0TXSTARTADDR2      0x22   // USB tx start address Low
//#define   USBEP0TXSTARTADDR3      0x23   // USB tx start address High
#define USBEP0RXSTARTADDR0      (0x24>>2)   // USB rx start address Low
//#define   USBEP0RXSTARTADDR1      0x25   // USB rx start address High
//#define   USBEP0RXSTARTADDR2      0x26   // USB rx start address Low
//#define   USBEP0RXSTARTADDR3      0x27   // USB rx start address High
#define USBEP1TXSTARTADDR0      (0x28>>2)   // USB tx start address Low
//#define   USBEP1TXSTARTADDR1      0x29   // USB tx start address High
//#define   USBEP1TXSTARTADDR2      0x2A   // USB tx start address Low
//#define   USBEP1TXSTARTADDR3      0x2B   // USB tx start address High
#define USBEP2RXSTARTADDR0      (0x2C>>2)   // USB rx start address Low
//#define   USBEP2RXSTARTADDR1      0x2D   // USB rx start address High
//#define   USBEP2RXSTARTADDR2      0x2E   // USB rx start address Low
//#define   USBEP2RXSTARTADDR3      0x2F   // USB rx start address High

#define USBEP0TXPACKCNTLO       0x30   // USB tx packet count Low
#define USBEP0TXPACKCNTHI       0x31   // USB tx packet count High
#define USBEP0RXPACKCNTLO       0x32   // USB Rx packet count Low
#define USBEP0RXPACKCNTHI       0x33   // USB Rx packet count High
#define USBEP1TXPACKCNTLO       0x34   // USB tx packet count Low
#define USBEP1TXPACKCNTHI       0x35   // USB tx packet count High
#define USBEP2RXPACKCNTLO       0x36   // USB Rx packet count Low
#define USBEP2RXPACKCNTHI       0x37   // USB Rx packet count High
#define USBEP0TXLASTPACKSIZE    0x38   // USB TX Last Packet Size register
#define USBEP1TXLASTPACKSIZE    0x39   // USB TX Last Packet Size register
#define USBEP3TXLASTPACKSIZE    0x3A   // USB TX Last Packet Size register

#define USBREQTYPE              0x40    // USB request type
#define USBREQCODE              0x41    // USB request code
#define USBREQVALUE             0x42    // USB request value low byte
#define USBREQINDEX             0x44    // USB request index low byte
#define USBREQLENGTH            0x46    // USB request length low byte

#define USBPROBECTRL            0x50    // USB Probe Control register
#define USBPROBEREAD            0x51    // USB Probe read out data register
#define USBEP3TXENABLE       0x70//EP3 TX enable
#define USBEX3TXDATABYTE0            0x80 //EP3 TX DATA BYTE0

//**********************************************************************************
// descriptor table address
//**********************************************************************************
#define     DEVICE_QUALIFIER_DESCRIPTOR_DATA0_ADDR   (0)
#define     DEVICE_DESCRIPTOR_DATA0_ADDR             (DEVICE_QUALIFIER_DESCRIPTOR_DATA0_ADDR+0x0A)
#define     CONFIG_DESCRIPTOR_DATA0_ADDR             (DEVICE_DESCRIPTOR_DATA0_ADDR+0x12)
#define     DEVICE_QUALIFIER_DESCRIPTOR_DATA1_ADDR   (0)
#define     DEVICE_DESCRIPTOR_DATA1_ADDR             (DEVICE_QUALIFIER_DESCRIPTOR_DATA1_ADDR+0x0A)
#define     CONFIG_DESCRIPTOR_DATA1_ADDR             (DEVICE_DESCRIPTOR_DATA1_ADDR+0x12)
#define     DEVICE_QUALIFIER_DESCRIPTOR_DATA2_ADDR   (0)
#define     DEVICE_DESCRIPTOR_DATA2_ADDR             (DEVICE_QUALIFIER_DESCRIPTOR_DATA2_ADDR+0x0A)            
#define     CONFIG_DESCRIPTOR_DATA2_ADDR             (DEVICE_DESCRIPTOR_DATA2_ADDR+0x12)          
#define     DEVICE_QUALIFIER_DESCRIPTOR_DATA3_ADDR   (0)
#define     DEVICE_DESCRIPTOR_DATA3_ADDR             (DEVICE_QUALIFIER_DESCRIPTOR_DATA3_ADDR+0x0A)            
#define     CONFIG_DESCRIPTOR_DATA3_ADDR             (DEVICE_DESCRIPTOR_DATA3_ADDR+0x12)
#define     DEVICE_QUALIFIER_DESCRIPTOR_DATA4_ADDR   (0)
#define     DEVICE_DESCRIPTOR_DATA4_ADDR             (DEVICE_QUALIFIER_DESCRIPTOR_DATA4_ADDR+0x0A)
#define     CONFIG_DESCRIPTOR_DATA4_ADDR             (DEVICE_DESCRIPTOR_DATA4_ADDR+0x12)
#define     DEVICE_QUALIFIER_DESCRIPTOR_DATA5_ADDR   (0)
#define     DEVICE_DESCRIPTOR_DATA5_ADDR             (DEVICE_QUALIFIER_DESCRIPTOR_DATA5_ADDR+0x0A)            
#define     CONFIG_DESCRIPTOR_DATA5_ADDR             (DEVICE_DESCRIPTOR_DATA5_ADDR+0x12)

#define     LANGUAGE_ID_DATA_ADDR                   (CONFIG_DESCRIPTOR_DATA1_ADDR+CONFIG_DESCRIPTOR_LEN)              


#define USB_CTL_BASE_ADDR           (0x8000A800)
#define USB_DMA_BASE_ADDR       (0x80006000)


typedef enum _AIT_STORAGE_TYPE {
    AIT_STORAGE_MOVINAND = 0,
    AIT_STORAGE_ESD,
    AIT_STORAGE_EMMC,
    AIT_STORAGE_SIF,
    AIT_STORAGE_NAND,
    AIT_STORAGE_PCAM_SF
}AIT_STORAGE_TYPE;

#define USB_GBL_CLK_USBPHY_DIV      (0x15)

//Base addr 0x8000A800
#define USB_FADDR_B                 (0x00)
#define USB_POWER_B                 (0x01)
#define USB_INTRTX_W                (0x02>>1)
#define USB_INTRRX_W                (0x04>>1)
#define USB_INTRTXE_W               (0x06>>1)
#define USB_INTRRXE_W               (0x08>>1)
#define USB_INTRUSB_B               (0x0A)
#define USB_INTRUSBE_B              (0x0B)
#define USB_FRAME_W                 (0x0C>>1)
#define USB_INDEX_B                 (0x0E)
#define USB_TESTMODE_B              (0x0F)
#define USB_SOFT_CON_B              (0xBF)

#define USB_TXMAXP_W                ((0x10+ 0xF0 + (endpoint << 4))>>1)
#define USB_TXCSR_W                 ((0x12+ 0xF0 + (endpoint << 4))>>1)
#define USB_CSR0_W  USB_TXCSR_W
#define USB_RXMAXP_W                ((0x14+ 0xF0 + (endpoint << 4))>>1)
#define USB_RXCSR_W                 ((0x16+ 0xF0 + (endpoint << 4))>>1)
#define USB_RXCOUNT_W               ((0x18+ 0xF0 + (endpoint << 4))>>1)
#define USB_COUNT0_W    USB_RXCOUNT_W
#define USB_FIFOSIZE_B              (0x1F+ 0xF0 + (endpoint << 4))
#define USB_CONFIGDATA_B    USB_FIFOSIZE_B

typedef enum _MMPF_USB_EP_ID {
    MMPF_USB_EP0 = 0,
    MMPF_USB_EP1,
    MMPF_USB_EP2,
    MMPF_USB_EP3,
    MMPF_USB_EP4,
    MMPF_USB_EP5,
    MMPF_USB_EP6
} MMPF_USB_EP_ID;

#define USB_EP0_FIFO_B              (0x20)
#define USB_EP0_FIFO_W              (0x20>>1)
#define USB_EP1_FIFO_B              (0x24)
#define USB_EP1_FIFO_W              (0x24>>1)
#define USB_EP2_FIFO_B              (0x28)
#define USB_EP2_FIFO_W              (0x28>>1)
#define USB_EP3_FIFO_B              (0x2C)
#define USB_EP3_FIFO_W              (0x2C>>1)
#define USB_EP3_FIFO_L              (0x2C>>2)

#define USB_DEVCTL_B                (0x60)

//  USB_INTRTX_W
#define EP0_INT_BIT                 (0x0001)
#define EP1_TX_INT_BIT              (0x0002)
#define EP2_TX_INT_BIT              (0x0004)
#define EP3_TX_INT_BIT              (0x0008)
#define EP4_TX_INT_BIT              (0x0010)
#define EP5_TX_INT_BIT              (0x0020)
#define EP6_TX_INT_BIT              (0x0040)
#define EP7_TX_INT_BIT              (0x0080)

//  USB_POWER_W
#define HS_MODE_BIT                 (0x10)


//  USB_INTRRX_W
#define EP1_RX_INT_BIT              (0x0002)
#define EP2_RX_INT_BIT              (0x0004)
#define EP3_RX_INT_BIT              (0x0008)
#define EP4_RX_INT_BIT              (0x0010)
#define EP5_RX_INT_BIT              (0x0020)
#define EP6_RX_INT_BIT              (0x0040)
#define EP7_RX_INT_BIT              (0x0080)
//  USB_INTRUSB_B
#define SUSPEND_INT_BIT             (0x0001)
#define RESUME_INT_BIT              (0x0002)
#define RESET_INT_BIT               (0x0004)
#define SOF_INT_BIT                 (0x0008)

//  USB_CSR0_W
#define EP0_RXPKTRDY_BIT            (0x0001)
#define EP0_TXPKTRDY_BIT            (0x0002)
#define EP0_SENTSTALL_BIT           (0x0004)
#define EP0_DATAEND_BIT             (0x0008)
#define EP0_SETUPEND_BIT            (0x0010)
#define EP0_SENDSTALL_BIT           (0x0020)
#define CLEAR_EP0_SENTSTALL         (0x0000)
#define SET_EP0_SERVICED_SETUPEND   (0x0084)
#define SET_EP0_SERVICED_RXPKTRDY   (0x0044)
#define SET_EP0_TXPKTRDY            (0x0006)
#define SET_EP0_SENDSTALL           (0x0024)

//  USB_TXCSR_W
#define TX_TXPKTRDY_BIT             (0x0001)
#define TX_FIFO_NOTEMPTY_BIT        (0x0002)
#define TX_UNDERRUN_BIT             (0x0004)
#define TX_FLUSHFIFO_BIT            (0x0008)
#define TX_SENDSTALL_BIT            (0x0010)
#define TX_SENTSTALL_BIT            (0x0020)
#define TX_INCOMPTX_BIT             (0x0080)
#define SET_TX_CLRDATATOG           (0x00E6)
#define SET_TX_TXPKTRDY             (0x00A7)
#define SET_TX_SENDSTALL            (0x00B1)
#define CLEAR_TX_SENTSTALL          (0x0086)
#define SET_TX_FLUSHFIFO            (0x00AE)
#define TX_MODE_ENABLE              (0x2000)
#define TXCSR_RW_MASK               (0xFF10)
#define TX_DMAREQENAB       (0x0100)

#define SET_TX_MODE                 (0x2000)        //except EP1, EP2 and EP3 should set TX or RX mode
//#if USB_UVC_BULK_EP==1
//#define SET_TX_ISO                  (0x0000)
//#else
#define SET_TX_ISO                  (0x4000)
//#endif
#define SET_RX_MODE_J               (~0x2000)

//  USB_RXCSR_W
#define RX_RXPKTRDY_BIT             (0x0001)
#define RX_SENDSTALL_BIT            (0x0020)
#define RX_SENTSTALL_BIT            (0x0040)
#define SET_RX_CLRDATATOG           (0x01C5)
#define SET_RX_SENDSTALL            (0x0165)
#define CLEAR_RX_SENTSTALL          (0x0105)
#define CLEAR_RX_RXPKTRDY           (0x0144)
#define RXCSR_RW_MASK               (0xF820)
#define RX_FLUSHFIFO_BIT            (0x0010)
#define RX_SET_ISO                  (0x4000)
#define RX_DMA_ENABLE               (0x2000)


#define USB_CONTROL_IDLE            (0x0)
#define USB_CONTROL_TX              (0x1)
#define USB_CONTROL_RX              (0x2)

#define USB_REQUEST_DIRECTION_BIT   (0x80)

#define USB_REQUEST_TYPE_MASK       (0x60)
#define USB_REQUEST_RECEIVER_MASK   (0x1F)
#define USB_ENDPOINT_ADDRESS_MASK   (0x7F)

#define USB_STANDARD_REQUEST        (0x00)
#define USB_CLASS_REQUEST           (0x20)
#define USB_VENDOR_REQUEST          (0x40)


//***************************************************************
#define  STD_CMD                   0x60
#define  GET_STATUS_CMD            0x60
#define  CLEAR_FEATURE_CMD         0x61
#define  SET_FEATURE_CMD           0x62
#define  SET_ADDRESS_CMD           0x63
#define  GET_DESCRIPTOR_CMD        0x64
#define  SET_DESCRIPTOR_CMD        0x65
#define  GET_CONFIG_CMD            0x66
#define  SET_CONFIG_CMD            0x67
#define  GET_INTERFACE_CMD         0x68
#define  SET_INTERFACE_CMD         0x69
#define  SYNCH_FRAME_CMD           0x6A

#define  GET_DEV_DESCR_CMD          0x6B
#define  GET_CONFIG_DESCR_CMD       0x6C
#define  GET_STRING_DESCR_CMD       0x6D
#define  GET_DEVQUA_DESCR_CMD       0x6E
#define  GET_OTHERSP_DESCR_CMD      0x6F

#define  GET_HIDREPORT_DESCR_CMD    0x70


#define  GET_STATUS              0x00
#define  CLEAR_FEATURE           0x01
#define  SET_FEATURE             0x03
#define  SET_ADDRESS             0x05
#define  GET_DESCRIPTOR          0x06         
#define  SET_DESCRIPTOR          0x07
#define  GET_CONFIGURATION       0x08
#define  SET_CONFIGURATION       0x09
#define  GET_INTERFACE           0x0A
#define  SET_INTERFACE           0x0B
#define  SYNCH_FRAME             0x0C

#define  DEVICE_DESCR            0x01
#define  CONFIG_DESCR            0x02
#define  STRING_DESCR            0x03
#define  INTERFACE_DESCR         0x04
#define  ENDPOINT_DESCR          0x05
#define  DEVICE_QUALIFIER_DESCR  0x06
#define  OTHER_SPEED_CONFIG_DESC 0x07
#define  INTERFACE_POWER_DESC    0x08

#define  DEVICE_REMOTE_WAKEUP    0x01
#define  ENDPOINT_STALL          0x00

#define  CLASS_CMD    0xF0
#define  BOT_RESET_CMD    0xFF
#define  BOT_GET_MAX_LUN_CMD      0xFE

#define PCSYNC_HS_TX_PKTSIZE        (0x0200)
#define PCSYNC_HS_RX_PKTSIZE        (0x0200)
#define PCSYNC_FS_TX_PKTSIZE        (0x0040)
#define PCSYNC_FS_RX_PKTSIZE        (0x0040)


// for VC, AC
#define  SET_CUR_CMD             0x01
#define  GET_CUR_CMD             0x81
#define  GET_MIN_CMD             0x82              
#define  GET_MAX_CMD             0x83
#define  GET_RES_CMD             0x84
#define  GET_LEN_CMD             0x85
#define  GET_MEM_CMD             0x85
#define  GET_INFO_CMD            0x86
#define  GET_DEF_CMD             0x87

#define  CAP_SET_CUR_CMD            (1 << 0)
#define  CAP_GET_CUR_CMD            (1 << 1)
#define  CAP_GET_MIN_CMD            (1 << 2)
#define  CAP_GET_MAX_CMD            (1 << 3)
#define  CAP_GET_RES_CMD            (1 << 4)
#define  CAP_GET_LEN_CMD            (1 << 5)
#define  CAP_GET_MEM_CMD            (1 << 5)
#define  CAP_GET_INFO_CMD           (1 << 6)
#define  CAP_GET_DEF_CMD            (1 << 7)


// ERROR_CODE
#define CONTROL_NO_ERROR         0x00
#define CONTROL_NOT_READY        0x01
#define CONTROL_WRONG_STATE      0x02
#define CONTROL_POWER            0x03
#define CONTROL_OUT_OF_RANGE     0x04
#define CONTROL_INVALID_UNIT     0x05
#define CONTROL_INVALID_CONTROL  0x06
#define CONTROL_INVALID_REQUEST  0x07
#define CONTROL_UNKNOWN          0xFF



#define STD_GET_STATUS_CMD          (0x0)
#define STD_CLEAR_FEATURE_CMD       (0x1)
#define STD_SET_FEATURE_CMD         (0x3)
#define STD_SET_ADDRESS_CMD         (0x5)
#define STD_GET_DESCRIPTOR_CMD      (0x6)
#define STD_SET_DESCRIPTOR_CMD      (0x7)
#define STD_GET_CONFIGURATION_CMD   (0x8)
#define STD_SET_CONFIGURATION_CMD   (0x9)
#define STD_GET_INTERFACE_CMD       (0xA)
#define STD_SET_INTERFACE_CMD       (0xB)
#define STD_SYNCH_FRAME_CMD         (0xC)

#define REQUEST_RECIVER_DEVICE      (0x0)
#define REQUEST_RECIVER_INTERFACE   (0x1)
#define REQUEST_RECIVER_ENDPOINT    (0x2)

#define MAX_INTERFACE_NUM           (0x1)
#define MAX_RX_ENDPOINT_NUM         (0x6) //for VSN_V2, EP0~EP5 are avaliable
#define MAX_TX_ENDPOINT_NUM         (0x6)

#define CLEAR_WAKEUP_FEATURE        (0xFD)
#define SET_WAKEUP_FEATURE          (0x02)
#define CLEAR_HALT_FEATURE          (0xFE)
#define SET_HALT_FEATURE            (0x01)

#define ENDPOINT_HALT_FEATURE           (0x0)
#define DEVICE_REMOTE_WAKEUP_FEATURE    (0x1)
#define TEST_MODE_FEATURE               (0x2)

#define TEST_FORCE_ENABLE               (0x5)

#define USB_PCCAM_MODE              (0x0)
#define USB_MSDC_MODE               (0x1)
#define USB_PCSYNC_MODE             (0x2)
#define USB_MTP_MODE                (0x3)
#define USB_DPS_MODE                (0x4)

#define MSDC_CBW_STATE              (0x0)
#define MSDC_DATA_TRANSFER_STATE_RX (0x1)
#define MSDC_DATA_TRANSFER_STATE_TX (0x2)
#define MSDC_CSW_STATE              (0x3)

#define MSDC_CBW_SIZE               (0x1F)

#define CBW_CBWSIGNATURE_INDEX          (0x0)
#define CBW_CBWTAG_INDEX                (CBW_CBWSIGNATURE_INDEX + 0x04)
#define CBW_CBWDATATRANSFERLENGTH_INDEX (CBW_CBWTAG_INDEX   + 0x04)
#define CBW_CBWFLAGS_INDEX              (CBW_CBWDATATRANSFERLENGTH_INDEX + 0x04)
#define CBW_CBWLUN_INDEX                (CBW_CBWFLAGS_INDEX     + 0x01)
#define CBW_CBWLENGTH_INDEX             (CBW_CBWLUN_INDEX   + 0x01)
#define CBW_CBWCB_INDEX                 (CBW_CBWLENGTH_INDEX    + 0x01)

#define CSW_CSWSIGNATURE_INDEX          (0x0)
#define CSW_CSWTAG_INDEX                (CSW_CSWSIGNATURE_INDEX + 0x04)
#define CSW_CSWDATARESIDUE_INDEX        (CSW_CSWTAG_INDEX + 0x04)
#define CSW_CSWSTATUS_INDEX             (CSW_CSWDATARESIDUE_INDEX + 0x04)

#define MSDC_TEST_UNIT_READY_CMD                (0x00)
#define MSDC_REZERO_CMD                         (0x01)
#define MSDC_REQUEST_SENSE_CMD                  (0x03)
#define MSDC_FORMAT_UNIT_CMD                    (0x04)
#define MSDC_INQUIRY_CMD                        (0x12)
#define MSDC_MODE_SELECT_15_CMD                 (0x15)
#define MSDC_MODE_SENSE_1A_CMD                  (0x1A)
#define MSDC_START_STOP_UNIT_CMD                (0x1B)
#define MSDC_SEND_DIAGNOSTIC_CMD                (0x1D)
#define MSDC_PREVENT_ALLOW_MEDIUM_REMOVAL_CMD   (0x1E)
#define MSDC_READ_FORMAT_CAPACITY_CMD           (0x23)
#define MSDC_READ_CAPACITY_CMD                  (0x25)
#define MSDC_READ_10_CMD                        (0x28)
#define MSDC_WRITE_10_CMD                       (0x2A)
#define MSDC_SEEK_10_CMD                        (0x2B)
#define MSDC_WRITE_AND_VERIFY_CMD               (0x2E)
#define MSDC_VERIFY_CMD                         (0x2F)
#define MSDC_MODE_SELECT_55_CMD                 (0x55)
#define MSDC_MODE_SENSE_5A_CMD                  (0x5A)
#define MSDC_READ_12_CMD                        (0xA8)
#define MSDC_WRITE_12_CMD                       (0xAA)

#define CBWCB_12_AllOC_LENGTH_INDEX             (CBW_CBWCB_INDEX + 0x04)

#define CBWCB_28_LBA3_INDEX                     (CBW_CBWCB_INDEX + 0x02)
#define CBWCB_28_LBA2_INDEX                     (CBW_CBWCB_INDEX + 0x03)
#define CBWCB_28_TxLen_MSB_INDEX                (CBW_CBWCB_INDEX + 0x07)
#define CBWCB_28_TxLen_LSB_INDEX                (CBW_CBWCB_INDEX + 0x08)

#define CBWCB_2A_LBA3_INDEX                     (CBW_CBWCB_INDEX + 0x02)
#define CBWCB_2A_LBA2_INDEX                     (CBW_CBWCB_INDEX + 0x03)
#define CBWCB_2A_RXLEN_MSB_INDEX                (CBW_CBWCB_INDEX + 0x07)
#define CBWCB_2A_RxLEN_LSB_INDEX                (CBW_CBWCB_INDEX + 0x08)

#define CLASS_CANCEL_REQUEST_CMD                (0x64)
#define CLASS_RESET_DEVICE_CMD                  (0x66)
#define CLASS_GET_DEVICE_STATUS_CMD             (0x67)
#define CLASS_MASS_STORAGE_RESET_CMD            (0xFF)
#define CLASS_GET_MAXLUN_CMD                    (0xFE)

#define USB_HIGH_SPEED                          (0x0)
#define USB_FULL_SPEED                          (0x1)

// For USB operation
#define USB_FLAG_GENOP                              (0x00000001)
#define USB_FLAG_CLSOP                              (0x00000002)
// End of USB operation

#define MSDC_AIT_SCSI_OPCODE                    (0xFA)
#define MSDC_AIT_SCSI_MAGIC_NUM0                (0x55)
#define MSDC_AIT_SCSI_MAGIC_NUM1                (0x66)


/*AIT BOOT command*/
#define MSDC_AIT_SCSI_GETINFO                   (0x01)
#define MSDC_AIT_SCSI_WRITEBOOTMEMORY           (0x02)
#define MSDC_AIT_SCSI_SWITCHCONTEXT             (0x03)
#define MSDC_AIT_SCSI_READREGISTER              (0x04)
#define MSDC_AIT_SCSI_WRITEREGISTER             (0x05)
#define MSDC_AIT_SCSI_SELECTSTORAGE             (0x06)
#define MSDC_AIT_SCSI_AITREAD                   (0x07)
#define MSDC_AIT_SCSI_AITWRITE                  (0x08)
#define MSDC_AIT_SCSI_SETBOOTSIZE               (0x09)
#define MSDC_AIT_SCSI_FORMATFS                  (0x0a)
#define MSDC_AIT_SCSI_ERASE_PARTITIONS          (0x0b)
#define MSDC_AIT_SCSI_ERASE_ALL_SECOTOR         (0x0c)
//support nand information for muonizer, sejong123.park, 11/03/22
#define MSDC_AIT_SCSI_GET_NANDINFO              (0x0D)
#define MSDC_AIT_SCSI_FORMATFS_R2               (0x0e)
#define MSDC_AIT_SCSI_FORMATFS_Z3               (0x0f)
#define MSDC_AIT_SCSI_MMC_RESET                 (0x11)
#define MSDC_AIT_SCSI_SET_BURNCODE_INFO         (0x20) //Updater used only



#define ENABLE_HS_HB_NOW                        (1)
#if ENABLE_HS_HB_NOW==1
// UVC & UAC start

#if (CHIP==P_V2)||(CHIP == VSN_V2)||(CHIP == VSN_V3)  
#define PCCAM_AU_EP_ADDR                        (0x02)
#endif
#else
#define PCCAM_AU_EP_ADDR                        (0x01)
#endif

#define AU_EP_MAX_PK_SIZE                   (512)//(1016) // sean@2010_12_27 , change to 512

#if ENABLE_HS_HB_NOW==1
#if (CHIP==P_V2)||(CHIP == VSN_V2)||(CHIP == VSN_V3) 
#define PCCAM_TX_EP_ADDR                        (0x01)
#endif
#else
#define PCCAM_TX_EP_ADDR                        (0x02)
#endif

#if USB_UVC_BULK_EP
#define HS_TX_MAX_PK_SIZE                   (512)
#define FS_TX_MAX_PK_SIZE                   (64)
#else   
#define HS_TX_MAX_PK_SIZE                   (1016)
#define FS_TX_MAX_PK_SIZE                    (992)
#endif

#if ENABLE_HS_HB_NOW==1
#if (CHIP == P_V2)||(CHIP == VSN_V2)||(CHIP == VSN_V3) 
#define TX_PER_FRAME                            (3)
#endif
#else
#define TX_PER_FRAME                            (1)
#endif

#define TX_ADD_FRAME_BITS(x)                    ( (x) << 11)

#define PCCAM_EX_EP_ADDR                        (0x03)
#define EX_EP_MAX_PK_SIZE                   (8)

// UVC & UAC end
#define MTP_TX_EP_ADDR                         (0x01)
#define MTP_RX_EP_ADDR                         (0x01)
#define MTP_INT_EP_ADDR                        (0x03)

#define PCSYNC_TX_EP_ADDR                       (0x01)
#define PCSYNC_RX_EP_ADDR                       (0x01)



// bDescriptorType: Ref. USB 2.0 spec Chap9
#define DEVICE_DESCRIPTOR                           (0x1)
#define CONFIG_DESCRIPTOR                           (0x2)
#define STRING_DESCRIPTOR                           (0x3)
#define INTERFACE_DESCRIPTOR                        (0x4)
#define ENDPOINT_DESCRIPTOR                         (0x5)
#define DEVICE_QUALIFIER_DESCRIPTOR                 (0x6)
#define OTHER_SPEED_CONFIG_DESCRIPTOR               (0x7)
#define INTERFACE_POWER_DESCCRIPTOR                 (0x8)
//class type
//(0x00): means the class information is carry by interface descriptor
#define USB_AUDIO_CLASS                 (0x01)
#define USB_HID_CLASS                   (0x03)  //human interface device
#define USB_IMAGE_CLASS                 (0x06)
#define USB_PRINTER_CLASS               (0x07)
#define USB_MASS_STORAGE_CLASS          (0x08)
#define USB_HUB_CLASS                   (0x09)
#define USB_VIDEO_CLASS                 (0x0E)
#define USB_MISC_CLASS                  (0xEF)
#define USB_APP_SPECIFIC_CLASS          (0xFE)
#define USB_VENDOR_SPECIFIC_CLASS       (0xFF)

//Endpoint package size setting
#define MSDC_HS_TX_PKTSIZE          (0x0200)
#define MSDC_HS_RX_PKTSIZE          (0x0200)
#define MSDC_FS_TX_PKTSIZE          (0x0040)
#define MSDC_FS_RX_PKTSIZE          (0x0040)
//Wilson@101011: for Logitech chap8 test
#define MSDC_INT_PKTSIZE            (0x0020)

#define MSDC_CMD_BUFFER_ADDR            (0x100000)
#define UPDATER_SIF_DMA_ADDR            (0x104000)
#define UPDATER_AIT_HEADER_ADDR         (0x105000)
#define UPDATER_USB_RX_ADDR             (0x11F000)

//interrupt enable
#define USB_INT_SUSPEND		0x01
#define USB_INT_RESUME		0x02
#define USB_INT_RESET		0x04
#define USB_INT_SOF			0x08
#define USB_INT_CONN		0x10
#define USB_INT_DISCON		0x20
#define USB_INT_SESS_REQ	0x40
#define USB_INT_VBUS_ERROR	0x80

enum ep0_state {
	EP0_DISCONNECT,
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

static const char *ep0states_str[]= {
        "EP0_DISCONNECT",
        "EP0_IDLE",			
        "EP0_IN_DATA_PHASE",
        "EP0_OUT_DATA_PHASE",
        "EP0_END_XFER",
        "EP0_STALL",
};


struct vsnv3_ep
{
	struct usb_ep 	ep;
	struct list_head	queue;
	struct vsnv3_udc	*udc;


	u8 ep_addr; 
	const struct usb_endpoint_descriptor *desc;
	unsigned			bIsStopped:1;
	unsigned			bIsHalted:1;

	unsigned			maxpacket:16;
	u8				int_mask;
	unsigned			is_pingpong:1;
	u8 num;

	unsigned			is_in:1;
	unsigned			is_iso:1;
	unsigned			fifo_bank:1;	
};


struct vsnv3_udc
{
	struct usb_gadget			gadget;
	struct platform_device		*pdev;
	struct usb_gadget_driver		*driver;	
		
	struct clk* udc_clk;
	struct vsnv3_ep ep[VSNV3_MAX_NUM_EP];
	struct dentry			*regs_info;

	spinlock_t			lock;

	enum ep0_state ep0state;
	u16 irq;
	void __iomem *regs;

	unsigned			bIsReqStd: 1;   


	unsigned			req_std : 1;
	unsigned			req_config : 1;
	unsigned			req_pending : 1;
	u8			last_request;
	u8 				address;
};

struct vsnv3_request {
	struct usb_request		req;
	struct list_head		queue;
};


	
#endif

