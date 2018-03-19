/**
 @file ex_usb.c
 @brief USB related sample code
 @author Alterman
 @version 1.0
*/

#include <mach/includes_fw.h>
#include <mach/mmpf_pio.h>

#if (USB_EN)
//==============================================================================
//
//                              COMPILE OPTIONS
//
//==============================================================================

#define USB_HOST_VERIFY     (0)

//==============================================================================
//
//                              HEADER FILES
//
//==============================================================================

#include "lib_retina.h"

#if (CHIP == P_V2)
#include "mmp_reg_gbl.h"
#endif
#include "mmp_reg_usb.h"

#include "ex_usb_otg.h"
#include "ex_usb.h"

#include "mmps_fs.h"
#include "mmps_usb.h"
#include "mmps_display.h"

#include "mmpf_system.h"
#include "mmpf_usbphy.h"
#include "mmpf_pio.h"
#include "mmps_3gprecd.h"

#if (USB_HOST_VERIFY)
#include "usbeh_core.h"
#include "usbeh_uvc.h"
#endif

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

static void USBEx_DetectPlugInMode(MMP_ULONG scan_times);
static void USBEx_AccessPhyReg(void);
static void USBEx_SetUSBResumeEvent(MMP_BOOL enable);
static void USBEx_HostModeTest(void);
#if (USB_HOST_VERIFY)
void USBEx_USBEH_App(void);
#endif

#if (ENABLE_AUTO_TEST == 1)
extern void GetCmdFromMaster(MMP_UBYTE *ulCmd, MMP_UBYTE *ulSubCmd);
#endif
extern MMP_ULONG CharToInteger(char *Data, MMP_ULONG ulLen);

//==============================================================================
//
//                              FUNCTION DEFINITION
//
//==============================================================================

static MMP_BYTE USBEx_ShowMenu(void)
{
    #if (ENABLE_AUTO_TEST == 1)
    MMP_UBYTE           ubSubCmd;
    MMP_UBYTE           ubCmd;
    #else
    MMP_ULONG           len;
    #endif
    MMP_BYTE            rx[15], c;

    RTNA_DBG_Str(0, "\r\n");
    RTNA_DBG_Str(0, "------------------ USB ------------------\r\n");
    RTNA_DBG_Str(0, "1    Start Mass Storage Device ----------\r\n");
    RTNA_DBG_Str(0, "2    Start UVC Device -------------------\r\n");
    RTNA_DBG_Str(0, "3    Start OTG File Transfer ------------\r\n");
#if (USB_HOST_VERIFY)
    RTNA_DBG_Str(0, "4    Start USB Embedded Host ------------\r\n");
#endif
    RTNA_DBG_Str(0, "a    Aging Test -------------------------\r\n");
    RTNA_DBG_Str(0, "c    Charger or operation? (10 times) ---\r\n");
    RTNA_DBG_Str(0, "d    Disconnect -------------------------\r\n");
    RTNA_DBG_Str(0, "p    PHY SPI access ---------------------\r\n");
    RTNA_DBG_Str(0, "r    Enable USB Resume Event ------------\r\n");
    RTNA_DBG_Str(0, "R    Disable USB Resume Event -----------\r\n");
    RTNA_DBG_Str(0, "q    Quit menu --------------------------\r\n");
    #if (UVC_HOST_TEST)
    RTNA_DBG_Str(0, "h    UVC Host mode ----------------------\r\n");
    #endif
    RTNA_DBG_Str(0, "-----------------------------------------\r\n");
    RTNA_DBG_Str(0, "Please Input Command:");

    /* get user input */
    #if (ENABLE_AUTO_TEST == 1)
	GetCmdFromMaster(&ubCmd, &ubSubCmd);
	rx[0] = ubCmd;
	rx[1] = '\0';
	RTNA_DBG_Str(0, "Cmd:");
    RTNA_DBG_Str(0, rx);
	rx[0] = ubSubCmd;
	RTNA_DBG_Str(0, ", SubCmd:");
    RTNA_DBG_Str(0, rx);
    RTNA_DBG_Str(0, "\r\n");
    c = ubCmd;
	#else
	RTNA_DBG_Gets(rx, &len);
    c = rx[0];
    RTNA_DBG_Str(0, "\r\n");
	#endif

    return c;
}

void USB_Example(void)
{
    MMP_BYTE    c;
    MMP_ULONG   status;
    //AITPS_USB_DMA   pUSB_DMA = AITC_BASE_USBDMA;

    /* NOTE:
     * During porting, below code to unmount & mount SD is not necessary.
     * For our testing, We can insert SD just before we enter USB menu 
     * to test MSDC, because we reset SD everytime we enter this USB menu.
     * Here is just a convenience.
     */
    MMPS_FS_UnmountVolume("SD:\\", 5);
    MMPS_FS_IOCtl("SD:\\", 5, MMPD_FS_CMD_UNMOUNT_MEDIUM, NULL, &status);
    MMPS_FS_IOCtl("SD:\\", 5, MMPS_FS_CMD_MOUNT_MEDIUM, NULL, &status);
    MMPS_FS_MountVolume("SD:\\", 5);

	do {
        /* Get operation option from menu */
        c = USBEx_ShowMenu();

        MMPS_USB_AdjustSignal(MMPS_USB_TX_CUR_480mV, MMPS_USB_SQ_75mV);

		switch(c) {
		case '1':   /* Start Mass Storage Device */
            MMPS_USBMSDC_SetMaxUnitNum(MSDC_MAX_UNITS);
		    MMPS_USB_SetMode(MMPS_USB_MSDC_MODE);
			break;

        case '2':   /* Start UVC Device */
            MMPS_USB_SetMode(MMPS_USB_PCCAM_MODE);
            break;

        case '3':   /* Start Host mode */
            USBEx_DeviceDisconnect();
            USBEx_HostModeTest();
            break;
#if (USB_HOST_VERIFY)
        case '4':
            USBEx_USBEH_App();
            break;
#endif
        case 'c':   /* Charger or operation? (10 times) */
            USBEx_DetectPlugInMode(10);
            break;

        case 'a':
            do {
                MMPS_USB_SetMode(MMPS_USB_MSDC_MODE);
                MMPF_OS_Sleep(2000);
                USBEx_DeviceDisconnect();
                //pUSB_DMA->USB_UTMI_CTL1 |= UTMI_USBPHY_NOSUSPEND;
                MMPF_OS_Sleep(500);
            } while(1);
            break;

		case 'd':   /* Disconnect */
		    USBEx_DeviceDisconnect();
		    break;

        case 'p':   /* PHY SPI access */
            USBEx_AccessPhyReg();
            break;

        case 'r':   /* Enable USB resume to wakeup system,
                     * should be set before USB suspend */
            USBEx_SetUSBResumeEvent(MMP_TRUE);
            break;
        case 'R':   /* Disable USB resume to wakeup system,
                     * clear it after USB resume */
            USBEx_SetUSBResumeEvent(MMP_FALSE);
            break;

		case 'q':   /* Quit menu */
		    // Stop connection before quit always
		    USBEx_DeviceDisconnect();
		    break;

#if (UVC_HOST_TEST)
            case 'h':   /* UVC Host */
                // Stop connection before quit always
                MMPS_Display_SetOutputPanel(MMPS_DISPLAY_PRM_CTL, MMPS_DISPLAY_MAIN_LCD);
                USBEx_DeviceDisconnect();
                USBEx_UvcDeviceConnect();
                break;
#endif

		default:
			RTNA_DBG_Str(0, "Unsupported Command!\r\n");
			break;
		}
	} while(c != 'q');
}

/* Cable plug in for charger or normal USB operation? */
static void USBEx_DetectPlugInMode(MMP_ULONG scan_times)
{
    #if (CHIP == P_V2)
    AITPS_GBL           pGBL = AITC_BASE_GBL;
    AITPS_USB_DMA       pUSB_DMA = AITC_BASE_USBDMA;
    AITPS_USB_CTL       pUSB_CTL = AITC_BASE_USBCTL;
    MMP_UBYTE           i = 0;
    MMP_UBYTE           ubTimeOutLimit = 0x10;
    static MMP_USHORT   ubLastSofNumber = 0x0;
    MMPS_USB_OP_MODE    ubMode = MMPS_USB_NONE_MODE;

    pGBL->GBL_CLK_EN |= GBL_CLK_USB;
    pGBL->GBL_CLK_USB_DIV = 0x01;
    pGBL->GBL_CLK_PATH_CTL3 |= GBL_USBPHY_CLK_SRC_PMCLK;

    while(scan_times--) {
        // Force USB Phy to connect
        // Suspend mode will power-off USB phy for leakage issue.
        // We must resume USB phy before using it.
        MMPF_USBPHY_Write(PWR_CTL_PUPD_TEST, 0x0000);
        MMPF_USBPHY_Write(PLL_TEST_OTG_CTL, OTG_VREF_PWR_DOWN);

        pUSB_DMA->USB_UTMI_PHY_CTL0 &= ~(CLKOSC_OFF_IN_SUSPEND);
        pUSB_DMA->USB_UTMI_PHY_CTL0 |= UTMI_PLL_ON_IN_SUSPEND;
        // remove DP/DM Pulldown
        pUSB_DMA->USB_UTMI_CTL1 &= ~(UTMI_DPPULLDOWN | UTMI_DMPULLDOWN);
        // disable USB testmode
        pUSB_DMA->USB_UTMI_CTL1 &= ~(UTMI_USBPHY_TESTMODE);

        pUSB_CTL->USB_POWER = USB_DEV_SOFT_CONN | USB_HS_EN | USB_SUSPENDM_EN;
        pUSB_DMA->USB_UTMI_PHY_CTL1 = UTMI_OUTCLK_PLL | UTMI_DATA_BUS_8BIT;

        // Wait for USB power-up stable.
        MMPF_OS_Sleep(200);

        for(i = 0x0; i < ubTimeOutLimit ; i++) {
            // Show SOF num for debug
            RTNA_DBG_PrintByte(0, pUSB_CTL->USB_FRAME_NUM);

            if (pUSB_CTL->USB_FRAME_NUM != 0x0) {
                if (pUSB_CTL->USB_FRAME_NUM == ubLastSofNumber) {
                    continue;
                }
                else {
                    RTNA_DBG_Str(0, "\r\n-- USB MSDC operation\r\n");
                    // Normal USB connection, enter MSDC mode
                    MMPS_USB_GetMode(&ubMode);
                    if (ubMode != MMPS_USB_MSDC_MODE) {
                        MMPS_USB_SetMode(MMPS_USB_MSDC_MODE);
                    }
                    ubLastSofNumber = pUSB_CTL->USB_FRAME_NUM;
                    break;
                }
            }
             //To lower the checking frequency, it can be adjusted!
            MMPF_OS_Sleep(10);
        }

        if (i == ubTimeOutLimit) {
            RTNA_DBG_Str(0, "\r\n-- USB Adaptorer !!\r\n");
            MMPS_USB_SetMode(MMPS_USB_ADAPTER_MODE);
            // It is in charger mode, turn-off soft-connect
            pUSB_CTL->USB_POWER &= ~(USB_DEV_SOFT_CONN);
        }
    }
    #endif

    return;
}

/* Device disconnect & controller reset */
void USBEx_DeviceDisconnect(void)
{
    MMPS_USB_StopDevice();
    MMPS_USB_DisconnectDevice();
}

/*
 * The following functions are for Read/Write USB PHY registers
 */
static MMP_BYTE USBEX_ShowAccessPhyMenu(void)
{
    MMP_ULONG   len;
    MMP_BYTE    rx[10], c;

    RTNA_DBG_Str(0, "\r\n");
    RTNA_DBG_Str(0, "---------------- PHY SPI ----------------\r\n");
    RTNA_DBG_Str(0, "1    Read PHY Register ------------------\r\n");
    RTNA_DBG_Str(0, "2    Write PHY Register -----------------\r\n");
    RTNA_DBG_Str(0, "q    Quit menu --------------------------\r\n");
    RTNA_DBG_Str(0, "-----------------------------------------\r\n");
    RTNA_DBG_Str(0, "Please Input Option:");
    RTNA_DBG_Gets(rx, &len);
    c = rx[0];
    RTNA_DBG_Str(0, "\r\n");

    return c;
}

static void USBEx_ReadPhyReg(void)
{
    MMP_ULONG   len;
    MMP_BYTE    input[10];
    MMP_ULONG   reg_addr;
    MMP_USHORT  reg_val;

    RTNA_DBG_Str(0, "Register to read (1-Byte in Hex, ex: 04, E, AE):");
    RTNA_DBG_Gets(input, &len);
    RTNA_DBG_Str(0, "\r\n");
    reg_addr = CharToInteger(input, len);

    reg_val = MMPF_USBPHY_Read(reg_addr);
    RTNA_DBG_Str(0, "Get reg[");
    RTNA_DBG_Byte(0, reg_addr);
    RTNA_DBG_Str(0, "] =");
    RTNA_DBG_Short(0, reg_val);
    RTNA_DBG_Str(0, "\r\n");
}

static void USBEx_WritePhyReg(void)
{
    MMP_ULONG   len;
    MMP_BYTE    input[10];
    MMP_ULONG   reg_addr;
    MMP_USHORT  reg_val;

    RTNA_DBG_Str(0, "Register to write (1-Byte in Hex, ex: 04, E, AE):");
    RTNA_DBG_Gets(input, &len);
    RTNA_DBG_Str(0, "\r\n");
    reg_addr = CharToInteger(input, len);
    RTNA_DBG_Str(0, "Register value (2-Byte in Hex, ex: 0004, E, A80):");
    RTNA_DBG_Gets(input, &len);
    RTNA_DBG_Str(0, "\r\n");
    reg_val = CharToInteger(input, len);

    MMPF_USBPHY_Write(reg_addr, reg_val);
    reg_val = MMPF_USBPHY_Read(reg_addr);
    RTNA_DBG_Str(0, "Set reg[");
    RTNA_DBG_Byte(0, reg_addr);
    RTNA_DBG_Str(0, "] =");
    RTNA_DBG_Short(0, reg_val);
    RTNA_DBG_Str(0, "\r\n");
}

static void USBEx_AccessPhyReg(void)
{
    MMP_BYTE    c;

    do {
        /* Get operation option from menu */
        c = USBEX_ShowAccessPhyMenu();

		switch(c) {
		case '1':   /* Read PHY Register */
		    USBEx_ReadPhyReg();
			break;

        case '2':   /* Write PHY Register */
            USBEx_WritePhyReg();
            break;

		default:
			RTNA_DBG_Str(0, "Unsupported Command!\r\n");
			break;
		}
	} while(c != 'q');
}

static void USBEx_SetUSBResumeEvent(MMP_BOOL enable)
{
    MMPF_SYS_SetWakeUpEvent(enable, MMPF_WAKEUP_USB_RESUME, 0, 0);
}
#endif
/*
 * The following functions are for OTG
 */
//static 
void USBEx_DriveVBus(MMP_BOOL on)
{
    if (on) {
        #if (CHIP == MCR_V2)
//        MMPF_PIO_Enable(MMPF_PIO_REG_GPIO52, MMP_TRUE);
	 MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO52, MMP_TRUE,0);
        #endif
        MMPF_PIO_EnableOutputMode(MMPF_PIO_REG_GPIO52, MMP_TRUE, MMP_FALSE);
        MMPF_PIO_SetData(MMPF_PIO_REG_GPIO52, 1, MMP_FALSE);
    }
    else {
        MMPF_PIO_EnableOutputMode(MMPF_PIO_REG_GPIO52, MMP_FALSE, MMP_FALSE);
        #if (CHIP == MCR_V2)
    //    MMPF_PIO_Enable(MMPF_PIO_REG_GPIO52, MMP_FALSE);
	 MMPF_PIO_EnableGpioMode(MMPF_PIO_REG_GPIO52, MMP_FALSE,0);		
        #endif
    }
}
#if 0
static void USBEx_HostModeTest(void)
{
    MMP_ULONG       ulBufSize, ulWorkBuf;

    MMPF_SYS_GetFWEndAddr(&ulWorkBuf);

    /* Turn on VBus by CGPIO20 to power up device */
    USBEx_DriveVBus(MMP_TRUE);

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_USB, MMP_TRUE);

    /* Start the OTG example testing */
    USB_OTG_Initialize();
    USB_OTG_Main(ulWorkBuf, &ulBufSize);
}

#if (USB_HOST_VERIFY)
static MMP_BOOL m_bUVCDevConnected = MMP_FALSE;

signed int USBEx_UVC_Connect(void)
{
    RTNA_DBG_Str(0, "UVC connect\r\n");
    USBEH_UVC_Open();
    m_bUVCDevConnected = MMP_TRUE;
    return USBEH_ERR_NONE;
}

signed int USBEx_UVC_Disonnect(void)
{
    RTNA_DBG_Str(0, "UVC connect\r\n");
    USBEH_UVC_Close();
    return USBEH_ERR_NONE;
}

void USBEx_USBEH_App(void)
{
    signed int          err;
    UINT                data_buf, data_size, dump_cnt = 20;
    USBEH_RELEASE_VER   version;

    /* Display current USBEH stack version */
    USBEH_GetVersion(&version);
    RTNA_DBG_Str(0, "USBEH version:");
    RTNA_DBG_Byte(0, version.major);
    RTNA_DBG_Byte(0, version.minor);
    RTNA_DBG_Byte(0, version.build);
    RTNA_DBG_Str(0, "\r\n");

    /* Initialize USBEH stack */
    err = USBEH_Init();
    if (err) {
        RTNA_DBG_Str(0, "USBEH Init err\r\n");
        return;
    }

    /* Register the supported class driver */
    err = USBEH_Class_RegDriver(&g_uvc_drv, USBEx_UVC_Connect,
                                USBEx_UVC_Disonnect);
    if (err) {
        RTNA_DBG_Str(0, "USBEH Register Class err\r\n");
        return;
    }

    /* Start USBEH host controller */
    USBEH_Start();

    /* Wait for UVC device connected */
    while(!m_bUVCDevConnected) {
        MMPF_OS_Sleep(2);
    }

    /* Starting to dump one YUY2 frame with the committed resol. */
    MMPF_SYS_GetFWEndAddr(&data_buf);
    USBEH_UVC_Start();

    USBEH_UVC_GetYUY2Frame(data_buf, &data_size);
    /* According to CATC log, one more eof payload sent after the 1st frame.
     * So we have to read one more payload header.
     */
    USBEH_UVC_GetEOF(data_buf, &data_size);
    /* To make sure AE convergence, dump few frames */
    while(dump_cnt--) {
        USBEH_UVC_GetYUY2Frame(data_buf, &data_size);
    }
    USBEH_UVC_Stop();
}
#endif

#if (UVC_HOST_TEST)
void USBEx_UvcDeviceConnect(void)
{
    MMP_ULONG       ulBufSize, ulWorkBuf;


    /* Turn on VBus by CGPIO20 to power up device */
    USBEx_DriveVBus(MMP_TRUE);

    MMPF_SYS_EnableClock(MMPF_SYS_CLK_USB, MMP_TRUE);

    /* Start the OTG example testing */
    USB_OTG_Initialize();
    /* Start the OTG example testing */
//    MMPF_SYS_GetFWEndAddr(&ulWorkBuf);
    MMPS_3GPRECD_GetParameter(MMPS_3GPRECD_PARAMETER_DRAM_END, &ulWorkBuf);
//    ulWorkBuf = 0x03000000;

    ulWorkBuf = ALIGN4K(ulWorkBuf);
    RTNA_DBG_Str(0, "ulWorkBuf after align 4K:");
    RTNA_DBG_Long(0, ulWorkBuf);
    RTNA_DBG_Str(0, "\r\n");
    
    USB_UVCH_Main(ulWorkBuf, &ulBufSize);
}
#endif

#else

void USB_Example(void) {}

#endif //(USB_EN)
