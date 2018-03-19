//==============================================================================
//
//  File        : mmpf_system.c
//  Description : MMPF_SYS functions
//  Author      : Jerry Tsao
//  Revision    : 1.0
//
//==============================================================================

#include <linux/delay.h>
#include <linux/module.h>
#include "includes_fw.h"

#include "mmpf_system.h"
#include "mmp_reg_gbl.h"
#include "reg_retina.h"

#include "mmp_reg_mci.h"

#include "mmpf_wd.h"

#include "mmpf_pll.h"

#if (OS_TYPE == OS_UCOSII)
#include "mmpf_hif.h"
#include "mmpf_pll.h"
#include "mmpf_dma.h"
#if (defined(ALL_FW)||defined(UPDATER_FW))&&(SYS_SELF_SLEEP_ENABLE == 0x1)
#include "mmpf_audio_ctl.h"
#include "mmp_reg_audio.h"
#include "mmp_reg_vif.h"
#include "mmpf_pio.h"
#include "mmp_reg_usb.h"
#include "mmpf_dram.h"
#endif
#include "mmpf_i2cm.h"
#endif //(OS_TYPE == OS_UCOSII)

/** @addtogroup MMPF_SYS
@{
*/
//==============================================================================
//
//                              VARIABLES
//
//==============================================================================

#if (OS_TYPE == OS_UCOSII)
OS_STK                  SYS_Task_Stk[TASK_SYS_STK_SIZE]; // put into sram
OS_STK                  LTASK_Task_Stk[LTASK_STK_SIZE];

extern MMPF_OS_FLAGID   SYS_Flag_Hif;
extern MMPF_PLL_MODE    gPLLMode;
/// @deprecated Customer and project ID
extern MMP_CUSTOMER  gbCustomer;
/// @brief Human-maintained release version
extern MMP_RELEASE_VERSION gbFwVersion;

extern MMP_BOOL    gbHasInitAitDAC;
MMP_USHORT	m_gsISPCoreID;
#if ((DSC_R_EN)&&(FDTC_SUPPORT == 1))
extern MMPF_OS_FLAGID   FDTC_Flag;
#include "mmpf_fdtc.h"
#endif
#if (APP_EN)
extern MMPF_OS_FLAGID   APP_Flag;
#include "mmpf_appapi.h"
#endif

extern MMP_USHORT  gsCurrentSensor;
//==============================================================================
//
//                         MODULE VARIABLES
//
//==============================================================================
/// @brief Compiler build version
MMP_SYSTEM_BUILD_VERSION mmpf_buildVersion = {__DATE__, __TIME__};

/// @brief Define the firmware name to be printed for debugging
#if defined (ALL_FW)
	#define FIRMWARE_NAME "ALL"
#else
    #define FIRMWARE_NAME "UNKNOWN FW (Modify mmpf_system.c)"
#endif

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================
#if defined(ALL_FW)||defined(UPDATER_FW)
extern void SPI_Write(MMP_UBYTE addr, MMP_USHORT data);
extern MMP_ERR MMPF_Sensor_PowerDown(MMP_USHORT usSensorID);
#endif

extern void MMPF_MMU_FlushDCache(void);
extern void AT91F_DisableDCache(void);
extern void AT91F_DisableICache(void);
extern void AT91F_DisableMMU(void);


//==============================================================================
//
//                              FUNCTIONS
//
//==============================================================================

//Gason@20120112, for div 0 case, system will output message.
extern void $Super$$__rt_div0(void);
extern long SYS_FLOAT ;

void $Sub$$__rt_div0(void)
{
    RTNA_DBG_Str(0, "Div 0\r\n");
    $Super$$__rt_div0();
    // Force Watch dog timeout
    //dump_div0_info();
    while(1);
}

#if defined(ALL_FW)
#include "mmpf_vif.h"
MMP_ERR MMPF_Self_Rom_Boot(void){

	AITPS_GBL pGBL = AITC_BASE_GBL;
	//Make sure that VI Clock is ON
	pGBL->GBL_CLK_DIS0 &= (MMP_UBYTE)(~(GBL_CLK_VI_DIS));
	MMPF_VIF_SetPIODir(VIF_SIF_RST, 0x01);
	MMPF_VIF_SetPIOOutput(VIF_SIF_RST, 0x01);  // pull high this pin to force ROM boot download(MSDC) mode,
	                                       // if the pin is high => MSDC mode, if the pin is low => normal boot mode.
	MMPF_SYS_ResetSystem(0x1);  // 1: re-boot FW to MSDC mode,

	return MMP_ERR_NONE;

}
#endif

#endif // (OS_TYPE == OS_UCOSII)

MMP_UBYTE	gbSystemCoreID = SYSTEM_CORE_ID;
EXPORT_SYMBOL(gbSystemCoreID);
/**@brief Keep the reference count of the clock module
*/
static MMP_ULONG    m_ulMdlClkRefCount[MMPF_SYS_CLK_MDL_NUM] = {0};

//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_EnableClock
//  Description :
//------------------------------------------------------------------------------
/** @brief The function enables or disables the specified clock

The function enables or disables the specified clock from the clock type input by programming the
Global controller registers.

@param[in] ulClockType the clock type to be selected
@param[in] bEnableclock enable or disable the clock
@return It reports the status of the operation.
*/
MMP_ERR  MMPF_SYS_EnableClock(MMPF_SYS_CLK clocktype, MMP_BOOL bEnableclock)
{
    #if OS_CRITICAL_METHOD == 3
    OS_CPU_SR   cpu_sr = 0;
    #endif

    #if (CHIP == MCR_V2)
    static AIT_REG_B *ClkEnRegMap[((MMPF_SYS_CLK_MDL_NUM+7)>>3)] = {
    #if (CHIP == MCR_V2)
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_EN[0]) + 0,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_EN[0]) + 1,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_EN[0]) + 2,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_EN[0]) + 3,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_EN[1]) + 0,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_EN[1]) + 1
    #endif
    };
    #endif //(CHIP == MCR_V2) || (CHIP == MERCURY)

    static AIT_REG_B *ClkDisRegMap[((MMPF_SYS_CLK_MDL_NUM+7)>>3)] = {
    #if (CHIP == VSN_V3)
        &(AITC_BASE_GBL->GBL_CLK_DIS0),
        (AIT_REG_B*)&(AITC_BASE_GBL->GBL_CLK_DIS1),
        ((AIT_REG_B*)&(AITC_BASE_GBL->GBL_CLK_DIS1)) + 1,
        &(AITC_BASE_GBL->GBL_CLK_DIS2)
    #endif
    #if (CHIP == MCR_V2)
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_DIS[0]) + 0,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_DIS[0]) + 1,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_DIS[0]) + 2,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_DIS[0]) + 3,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_DIS[1]) + 0,
        (AIT_REG_B*)(&AITC_BASE_GBL->GBL_CLK_DIS[1]) + 1
    #endif
    };

    if (clocktype >= MMPF_SYS_CLK_MDL_NUM) {
        return MMP_SYSTEM_ERR_HW;
    }

    OS_ENTER_CRITICAL();

    if (bEnableclock) {
        if (m_ulMdlClkRefCount[clocktype]++ == 0) {
            #if (CHIP == VSN_V3)
            *ClkDisRegMap[(clocktype >> 3)] &= ~(1 << (clocktype & 0x7));
            #endif
            #if (CHIP == MCR_V2) || (CHIP == MERCURY)
            *ClkEnRegMap[(clocktype >> 3)] |= (1 << (clocktype & 0x7));
            #endif
        }
    } else {
        if (m_ulMdlClkRefCount[clocktype]) {
            m_ulMdlClkRefCount[clocktype]--;
        }

        if (m_ulMdlClkRefCount[clocktype] == 0) {
            *ClkDisRegMap[(clocktype >> 3)] |= (1 << (clocktype & 0x7));
        }
    }

    OS_EXIT_CRITICAL();

    return  MMP_ERR_NONE;
}

//Force Watch Dog timeout//
static void WatchDogReset(int ms)                                  
{
#if CHIP==MCR_V2
    MMP_ULONG clk_div,CLK_DIV[] = {1024, 128, 32 , 8 } ;            
    int i=0;                                                    
    MMP_ULONG g0_slow ,rst_c_1s ,rst_c,c ;                          
                                                                     
    MMPF_PLL_GetGroupFreq(0,&g0_slow );                             
    g0_slow = g0_slow / 2 ;                                         
    c = 0 ;                                                         
    do {                                                            
        clk_div = CLK_DIV[c] ;                                      
        rst_c_1s =  g0_slow *1000 / (  clk_div * 16384 ) ; // 1 secs
        rst_c = (rst_c_1s * ms ) / 1000 ;                           
        //dbg_printf(0,"rst_c : %d,DIV:%d\r\n",rst_c, clk_div ) ;     
        c++ ;                                                       
        if(c>=3) {                                                  
            break ;                                                 
        }                                                           
    }                                                               
    while ( (rst_c > 31) || (!rst_c) ) ;                            
                                                                    
                                                                    
                                                                    
    MMPF_WD_SetTimeOut(rst_c, clk_div );                            
    MMPF_WD_EnableWD(MMP_TRUE,MMP_TRUE,MMP_FALSE,0,MMP_TRUE);       
    do {                                                            
        //dbg_printf(0,"%d ms\r\n",i*100);                            
        //MMPF_OS_Sleep(100); 
	 AITPS_CORE  pCORE   = AITC_BASE_CORE;
	 AITPS_GBL   pGBL 	= AITC_BASE_GBL;

	 pGBL->GBL_BOOT_STRAP_CTL = 0x40;
	 	
       // pr_alert("wd conf 0x3A02=0X%x\r\n",pCORE->CORE_A_WD_CFG);	
       // pr_alert("wd conf 0x69F0=0X%.4x\r\n",pGBL->GBL_BOOT_STRAP_CTL);
		
        mdelay(1);
        pr_alert("%d ms.\r\n",i*100);
        i++ ;                                                       
    } while (1);
#endif
}    


//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_ResetSystem
//  Description :
//------------------------------------------------------------------------------
/** @brief The function is used to reset system and system re-run from ROM code.

 The function is used to reset system and system re-run from ROM code.
@return It reports the status of the operation.
*/
MMP_ERR	MMPF_SYS_ResetSystem(MMP_UBYTE ubSpecialCase)
{
#if (CHIP == VSN_V3)
#if (OS_TYPE == OS_LINUX)
    AITPS_WD    pWD = AITC_BASE_WD;
    AITPS_GBL   pGBL = AITC_BASE_GBL;

    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_VIF, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_ISP, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_JPG, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL0, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL1, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL2, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_GPIO, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_AUD, MMP_TRUE);

    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_RAW, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DMA, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_I2CS, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_H264, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_IBC, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_GRA, MMP_TRUE);

    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SD0, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PWM, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PSPI, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB_PHY, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_CPU_SRAM, MMP_TRUE);

    #if (CHIP == VSN_V3)
    pGBL->GBL_CLK_DIS0 = (MMP_UBYTE)(~(GBL_CLK_MCI_DIS | GBL_CLK_VI_DIS| GBL_CLK_JPG_DIS | GBL_CLK_CPU_DIS | GBL_CLK_GPIO_DIS));
    pGBL->GBL_CLK_DIS1 = (MMP_USHORT)(~(GBL_CLK_DRAM_DIS | GBL_CLK_PWM_DIS | GBL_CLK_I2C_DIS |
                            GBL_CLK_DMA_DIS | GBL_CLK_USB_DIS | GBL_CLK_CPU_PHL_DIS));
    pGBL->GBL_CLK_DIS2 = (MMP_UBYTE)(~(GBL_CLK_CIDC_DIS | GBL_CLK_GNR_DIS | GBL_CLK_COLOR_DIS)) ;
    pGBL->GBL_CLK_DIS1 &= ~GBL_CLK_BS_SPI_DIS;

    //Change the Boot Strapping as ROM boot
    pGBL->GBL_CHIP_CTL |= MOD_BOOT_STRAP_EN;
    pGBL->GBL_CHIP_CFG = ROM_BOOT_MODE;
    pGBL->GBL_CHIP_CTL &= ~MOD_BOOT_STRAP_EN;

    //Turn-off watch dog
    if((pWD->WD_MODE_CTL0 & WD_EN)!= 0x0) {
        //RTNA_DBG_Str(0, "\r\nTurn-off WD !!\r\n");
        pWD->WD_MODE_CTL0 = 0x2340;
    }

    //VSN_V3, CPU access ROM code have HW bug, so we use reset ROM controller to archieve this purpose.
    //Note: The busy waiting is necessary !!!  ROM controller need some time to re-load ROM code.
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_CPU_PHL, MMP_FALSE);

    MMPF_PLL_WaitCount(0x3FFFF);
    //Finally, use watch-dog do timeout-reset, this purpose is to reset PLL as normal speed for serial-flash acessing issue.


    pWD->WD_MODE_CTL1 = WD_CLK_CTL_ACCESS_KEY|(10 << 2)|WD_CLK_MCK_D128;

    pGBL->GBL_CHIP_CTL |= GBL_WD_RST_ALL_MODULE;

    pWD->WD_RE_ST = WD_RESTART; //Before enable, we shoudl set re-start first.

    pWD->WD_MODE_CTL0 = WD_CTL_ACCESS_KEY|WD_RST_EN|WD_EN;
    #endif // (CHIP == VSN_V3)

#endif //(OS_TYPE == OS_LINUX)

#if (OS_TYPE == OS_UCOSII)
#if !defined(UPDATER_FW)
	//AITPS_GBL pGBL = AITC_BASE_GBL;
	#if (OS_CRITICAL_METHOD == 3)
	OS_CPU_SR   cpu_sr = 0;
	#endif

	#if (CHIP == VSN_V3)
	void (*FW_Entry)(void) = NULL;
	AITPS_WD 	pWD = AITC_BASE_WD;
	MMP_ULONG counter = 0x0;
	#endif
	
	AITPS_GBL   pGBL = AITC_BASE_GBL;
	
	OS_ENTER_CRITICAL();
	
	#if (SYS_WD_ENABLE == 0x1)
	MMPF_WD_Kick();
	#endif
	
  	if(ubSpecialCase != 0x1) {
  	  MMPF_SYS_ResetHModule(MMPF_SYS_MDL_VIF, MMP_TRUE);
  	}
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_ISP, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_JPG, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL0, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL1, MMP_TRUE);
    MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL2, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_GPIO, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_AUD, MMP_TRUE);
    
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_RAW, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DMA, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_I2CS, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_H264, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_IBC, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_GRA, MMP_TRUE);
    
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SD0, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PWM, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PSPI, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB_PHY, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_CPU_SRAM, MMP_TRUE);
	
	MMPF_MMU_FlushDCache();
	AT91F_DisableDCache();
	AT91F_DisableICache();
	AT91F_DisableMMU();

	pGBL->GBL_CLK_DIS0 = (MMP_UBYTE)(~(GBL_CLK_MCI_DIS | GBL_CLK_VI_DIS| GBL_CLK_JPG_DIS | GBL_CLK_CPU_DIS | GBL_CLK_GPIO_DIS));
	pGBL->GBL_CLK_DIS1 = (MMP_USHORT)(~(GBL_CLK_DRAM_DIS | GBL_CLK_PWM_DIS | GBL_CLK_I2C_DIS | GBL_CLK_DMA_DIS | GBL_CLK_USB_DIS | GBL_CLK_CPU_PHL_DIS));
    #if (CHIP == VSN_V3)
	pGBL->GBL_CLK_DIS2 = (MMP_UBYTE)(~(GBL_CLK_CIDC_DIS | GBL_CLK_GNR_DIS | GBL_CLK_COLOR_DIS)) ;
    #endif

	pGBL->GBL_CLK_DIS1 &= ~GBL_CLK_BS_SPI_DIS;
	
    #if (CHIP == VSN_V3)
	//Change the Boot Strapping as ROM boot
	pGBL->GBL_CHIP_CTL |= MOD_BOOT_STRAP_EN;
	pGBL->GBL_CHIP_CFG = ROM_BOOT_MODE;
	pGBL->GBL_CHIP_CTL &= ~MOD_BOOT_STRAP_EN;
	
	//Turn-off watch dog
 	if((pWD->WD_MODE_CTL0 & WD_EN)!= 0x0) {
 		RTNA_DBG_Str(0, "\r\nTurn-off WD !!\r\n");
  	    pWD->WD_MODE_CTL0 = 0x2340;
 	}
	
	//VSN_V3, CPU access ROM code have HW bug, so we use reset ROM controller to archieve this purpose. 
	//Note: The busy waiting is necessary !!!  ROM controller need some time to re-load ROM code.
 	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_CPU_PHL, MMP_FALSE);
	#pragma O0
	for(counter = 0x3FFFF; counter > 0 ; counter --) {
	}
	#pragma
	
	if(ubSpecialCase == 0x1) {
		FW_Entry = (void (*)(void))(0x0);
		FW_Entry();	//enter the firmware entry
		while(1);
	}

	//Finally, use watch-dog do timeout-reset, this purpose is to reset PLL as normal speed for serial-flash acessing issue.
	MMPF_WD_SetTimeOut(31, 128);
	MMPF_WD_EnableWD(MMP_TRUE, MMP_TRUE, MMP_FALSE, NULL, MMP_TRUE);
    #endif

	#if 0  //The following test code is used CPU watch dog to archieve system reset
	//Change the Boot Strapping as ROM boot
	pGBL->GBL_CHIP_CTL |= MOD_BOOT_STRAP_EN;
    pGBL->GBL_CHIP_CFG = ROM_BOOT_MODE;
    pGBL->GBL_CHIP_CTL &= ~MOD_BOOT_STRAP_EN;
    
	MMPF_WD_Initialize();
	
	MMPF_WD_SetTimeOut(31, 128);
	
	MMPF_WD_EnableWD(MMP_TRUE, MMP_TRUE, MMP_FALSE, NULL, MMP_TRUE);
	#endif
#endif //#if !defined(UPDATER_FW)

#endif // (OS_TYPE == OS_UCOSII)
#endif // (CHIP == VSN_V3)

#if (CHIP == MERCURY)
	TODO
#endif

#if (CHIP == MCR_V2)

        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_VIF0, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_VIF1, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_ISP, MMP_TRUE);
   	 MMPF_SYS_ResetHModule(MMPF_SYS_MDL_RAW_S0, MMP_TRUE);
   	 MMPF_SYS_ResetHModule(MMPF_SYS_MDL_RAW_S1, MMP_TRUE);
   	 MMPF_SYS_ResetHModule(MMPF_SYS_MDL_RAW_F, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_IBC0, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_IBC1, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_IBC2, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_IBC3, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_ICON0, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_ICON1, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_ICON2, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_ICON3, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL0, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL1, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL2, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SCAL3, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_JPG, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_H264, MMP_TRUE);
        MMPF_SYS_ResetHModule(MMPF_SYS_MDL_GRA, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_AUD, MMP_TRUE);

  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DMA_M0, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DMA_M1, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DMA_R0, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_DMA_R1, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_I2CS, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);
    
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SD0, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SD1, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_SD2, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PWM, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_PSPI, MMP_TRUE);
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_LDC, MMP_TRUE);

	WatchDogReset(500);
#endif

	return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_ResetHModule
//  Description :
//------------------------------------------------------------------------------
/** @brief The function is used to reset HW modules.

The function is used to reset HW modules.
@param[in] moduletype is used to select which modules to reset.
@param[in] bResetRegister is used enable/disable reset module's registers. 
@return It reports the status of the operation.
*/
MMP_ERR MMPF_SYS_ResetHModule(MMPF_SYS_MDL moduletype, MMP_BOOL bResetRegister)
{
	AITPS_GBL pGBL = AITC_BASE_GBL;

    #if (CHIP == VSN_V3)
	if(moduletype < MMPF_SYS_MDL_CPU_SRAM) {
		if(bResetRegister) {
			pGBL->GBL_RST_REG_EN |= (0x1 << moduletype);
		} else {
            pGBL->GBL_RST_REG_EN &= ~(0x1 << moduletype);
		}

		pGBL->GBL_RST_CTL01 |= (0x1 << moduletype);
		MMPF_PLL_WaitCount(0x20);  //Note: 0x20 is just the test value used in PCAM project
		pGBL->GBL_RST_CTL01 &= ~(0x1 << moduletype);

		if(bResetRegister) {
			pGBL->GBL_RST_REG_EN &= ~(0x1 << moduletype);
		}
	}
	else if (moduletype < MMPF_SYS_MDL_SM) {
		MMP_UBYTE	offset = (moduletype - MMPF_SYS_MDL_CPU_SRAM);

		if (bResetRegister && (moduletype > MMPF_SYS_MDL_CPU_SRAM) && (moduletype < MMPF_SYS_MDL_USB_PHY)) {
			pGBL->GBL_RST_CTL2 |= (1 << (offset + 4));
		} else {
            pGBL->GBL_RST_CTL2 &= ~(1 << (offset + 4));
		}

		pGBL->GBL_RST_CTL2 |= (0x1 << offset);
		MMPF_PLL_WaitCount(0x20);
		pGBL->GBL_RST_CTL2 &= ~(0x1 << offset);

        if (bResetRegister && (moduletype > MMPF_SYS_MDL_CPU_SRAM) && (moduletype < MMPF_SYS_MDL_USB_PHY)) {
			pGBL->GBL_RST_CTL2 &= ~(0x1 << (offset + 4));
		}
	}
    else if (moduletype < MMPF_SYS_MDL_NUM) {
        MMP_UBYTE   offset = (moduletype - MMPF_SYS_MDL_SM);

        if (bResetRegister) {
            pGBL->GBL_RST_CTL3 |= (1 << (offset + 4));
        } else {
            pGBL->GBL_RST_CTL3 &= ~(1 << (offset + 4));
        }

        pGBL->GBL_RST_CTL3 |= (0x1 << offset);
        MMPF_PLL_WaitCount(0x20);
        pGBL->GBL_RST_CTL3 &= ~(0x1 << offset);

        if (bResetRegister) {
            pGBL->GBL_RST_CTL3 &= ~(0x1 << (offset + 4));
        }
    }
	else {
        PRINTF("Invalid module in sw reset\r\n");
        return MMP_SYSTEM_ERR_PARAMETER;
	}
    #endif

    #if (CHIP == MERCURY)
	TODO
    #endif

    #if (CHIP == MCR_V2)
    AIT_REG_D *plMdlRstEnReg, *plMdlRstDisReg, *plRegRstEnReg;
    MMP_ULONG ulMdlRstVal;

    if (moduletype >= MMPF_SYS_MDL_NUM) {
        PRINTF("Invalid module in sw reset\r\n");
        return MMP_SYSTEM_ERR_PARAMETER;
    }

    if (moduletype < MMPF_SYS_MDL_CPU_PHL) {
        plMdlRstEnReg = &pGBL->GBL_SW_RST_EN[0];
        plMdlRstDisReg = &pGBL->GBL_SW_RST_DIS[0];
        plRegRstEnReg = &pGBL->GBL_REG_RST_EN[0];
        ulMdlRstVal = 1 << moduletype;
    }
    else {
        plMdlRstEnReg = &pGBL->GBL_SW_RST_EN[1];
        plMdlRstDisReg = &pGBL->GBL_SW_RST_DIS[1];
        plRegRstEnReg = &pGBL->GBL_REG_RST_EN[1];
        ulMdlRstVal = 1 << (moduletype - MMPF_SYS_MDL_CPU_PHL);
    }

    if (bResetRegister){
        *plRegRstEnReg |= ulMdlRstVal;
    } else {
        *plRegRstEnReg &= ~ulMdlRstVal;
    }

    *plMdlRstEnReg = ulMdlRstVal;
    MMPF_PLL_WaitCount(100);
    *plMdlRstDisReg = ulMdlRstVal;

    if (bResetRegister){
        *plRegRstEnReg &= ~ulMdlRstVal;
    }
    #endif
	
	return MMP_ERR_NONE;
}


/**
 @brief Fine tune MCI priority to fit VGA size encoding. It's an access issue.
 @param[in] ubMode Mode.
 @retval MMP_ERR_NONE Success.
*/
MMP_ERR MMPF_SYS_TuneMCIPriority(MMP_UBYTE ubMode)
{
    AIT_REG_B   *pMciBase = (AIT_REG_B*)AITC_BASE_MCI;
    MMP_ULONG   addr;
    #if(CHIP == VSN_V3)

    if (ubMode == 2) {
        pMciBase[0x11] |= (0x80 | 0x40);
        pMciBase[0x29] = 0x17; // jpeg wr dram
        pMciBase[0x27] = 0x17; // jpeg compress r/w
        //pMciBase[0x22] = 0x18; // RAW store 0

        #if 1//FRAMEBASE_H264_H264_Y
        {
            //pMciBase[0x2B] = 0x1E; // h264e dm rd dram

            pMciBase[0x24] = 0x1F; // ibc 0
            pMciBase[0x64] = 0x07; // ibc 0 na
            pMciBase[0x84] = 0x07; // ibc 0 row hit
            pMciBase[0xA4] = 0x07; // ibc 0 conti

            pMciBase[0x25] = 0x1F; // ibc 1
            pMciBase[0x65] = 0x07; // ibc 0 na
            pMciBase[0x85] = 0x07; // ibc 0 row hit
            pMciBase[0xA5] = 0x07; // ibc 0 conti

            #if 0//Y crop patch
            pMciBase[0x26] = 0x1F; // ibc 2
            pMciBase[0x66] = 0x07; // ibc 2 na
            pMciBase[0x86] = 0x07; // ibc 2 row hit
            pMciBase[0xA6] = 0x07; // ibc 2 conti
            #else
            pMciBase[0x26] = 0x1D; // ibc 2
            #endif

        }
        #endif
        #if 0 //FRAMEBASE_H264_Y
        {
            pMciBase[0x2B] = 0x1E; // h264e dm rd dram
            pMciBase[0x2C] = 0x1E; // h264e inter rd
            pMciBase[0x2E] = 0x1E; // h264e dblk wr

            pMciBase[0x25] = 0x1F; // ibc 1
            pMciBase[0x65] = 0x07; // ibc 1 na
            pMciBase[0x85] = 0x07; // ibc 1 row hit
            pMciBase[0xA5] = 0x07; // ibc 1 conti

            #if 0//Y crop patch
            pMciBase[0x26] = 0x1F; // ibc 2
            pMciBase[0x66] = 0x07; // ibc 2 na
            pMciBase[0x86] = 0x07; // ibc 2 row hit
            pMciBase[0xA6] = 0x07; // ibc 2 conti
            #else
            pMciBase[0x26] = 0x1A; // ibc 2
            #endif
        }
        #endif
    } else if (ubMode == 1) {
        for (addr = 0x20; addr <= 0x36; addr++) {
            pMciBase[addr] = 0x50;
        }
        for (addr = 0x40; addr <= 0x56; addr++) {
            pMciBase[addr] = 0x40;
        }
        for (addr = 0x60; addr <= 0x76; addr++) {
            pMciBase[addr] = 0x11;
        }
        for (addr = 0x80; addr <= 0x96; addr++) {
            pMciBase[addr] = 0x11;
        }
        for (addr = 0xA0; addr <= 0xB6; addr++) {
            pMciBase[addr] = 0x11;
        }
        //pMciBase[0x11] &= ~0x80;
    }
    #endif

    #if (CHIP == MERCURY)
    #endif

    #if (CHIP == MCR_V2)
    if (ubMode == 2) {
        pMciBase[0x11] |= (0x80 | 0x40);
        pMciBase[0x2B] = 0x17; // jpeg wr dram
        pMciBase[0x33] = 0x17; // jpeg compress r/w

        pMciBase[0x2D] = 0x1F; // pSPI
        pMciBase[0x6D] = 0x07; // pSPI na
        pMciBase[0x8D] = 0x07; // pSPI row hit
        pMciBase[0xAD] = 0x07; // pSPI conti
        //pMciBase[0x22] = 0x18; // RAW store 0

        {
            //pMciBase[0x2B] = 0x1E; // h264e dm rd dram

            //pMciBase[0x27] = 0x1F; // ibc 0
            //pMciBase[0x67] = 0x07; // ibc 0 na
            //pMciBase[0x87] = 0x07; // ibc 0 row hit
            //pMciBase[0xA7] = 0x07; // ibc 0 conti

            //pMciBase[0x28] = 0x1F; // ibc 1
            //pMciBase[0x68] = 0x07; // ibc 1 na
            //pMciBase[0x88] = 0x07; // ibc 1 row hit
            //pMciBase[0xA8] = 0x07; // ibc 1 conti

            #if 0//Y crop patch
            pMciBase[0x26] = 0x1F; // ibc 2
            pMciBase[0x66] = 0x07; // ibc 2 na
            pMciBase[0x86] = 0x07; // ibc 2 row hit
            pMciBase[0xA6] = 0x07; // ibc 2 conti
            #else
            //pMciBase[0x29] = 0x1D; // ibc 2
            #endif

        }
    } else if (ubMode == 1) {
        for (addr = 0x20; addr <= 0x36; addr++) {
            pMciBase[addr] = 0x50;
        }
        for (addr = 0x40; addr <= 0x56; addr++) {
            pMciBase[addr] = 0x40;
        }
        for (addr = 0x60; addr <= 0x76; addr++) {
            pMciBase[addr] = 0x11;
        }
        for (addr = 0x80; addr <= 0x96; addr++) {
            pMciBase[addr] = 0x11;
        }
        for (addr = 0xA0; addr <= 0xB6; addr++) {
            pMciBase[addr] = 0x11;
        }
        pMciBase[0x11] &= ~0x80;
    }
    #endif

    return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_System_SetSensorIntputCapability
//  Description :
//------------------------------------------------------------------------------
/** @brief The function limits input resolution

The function limits sensor input resolution capability

@param[in] Capability max supported input resolution
@return It reports the status of the operation.
*/
MMP_ERR MMPF_SYS_SetSensorInputCapability(MMPF_SYSTEM_SNR_IF_CAP Capability)
{
    AITPS_GBL   pGBL  = AITC_BASE_GBL;

    switch (Capability) {
    case MMPF_SYSTEM_SNR_IF_CAP_3M:
        #if (CHIP == MCR_V2) || (CHIP == VSN_V3)
        pGBL->GBL_SW_RESOL_SEL
        #endif
            = GBL_SENSOR_3M;
        break;
    case MMPF_SYSTEM_SNR_IF_CAP_5M:
        #if (CHIP == MCR_V2) || (CHIP == VSN_V3)
        pGBL->GBL_SW_RESOL_SEL
        #endif
            = GBL_SENSOR_5M;
        break;
    case MMPF_SYSTEM_SNR_IF_CAP_8M:
    default:
        #if (CHIP == MCR_V2) || (CHIP == VSN_V3)
        pGBL->GBL_SW_RESOL_SEL
        #endif
            = GBL_SENSOR_8M;
        break;
    }

    return MMP_ERR_NONE;
}


#if (OS_TYPE == OS_UCOSII)
//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_SetWakeUpEvent
//  Description :
//------------------------------------------------------------------------------
/** @brief The function set the wake up event when CPU enter sleep

The function set the PIN as Output mode (bEnable = MMP_TRUE) or Input mode.
@param[in] bEnable is used to turn on/off wake up event.
@param[in] event is used to select wake up event type. 
@param[in] piopin is the PIO number, please reference the data structure of MMPF_PIO_REG
@param[in] polarity is used for GPIO event wake up. To set high(0)/low(1) level wake up.
@return It reports the status of the operation.
*/
MMP_ERR MMPF_SYS_SetWakeUpEvent(MMP_BOOL bEnable, MMPF_WAKEUP_EVNET event, MMPF_PIO_REG piopin, MMP_UBYTE polarity)
{
#if (defined(ALL_FW)||defined(UPDATER_FW))&&(SYS_SELF_SLEEP_ENABLE == 0x1)
	MMP_UBYTE ubShiftBit = 0x0;
	AITPS_GBL pGBL = AITC_BASE_GBL;
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
	
	if(bEnable == MMP_TRUE) {
		if(event == MMPF_WAKEUP_GPIO) {	
			switch(piopin) {
				case MMPF_PIO_REG_GPIO0:
					ubShiftBit = GBL_WAKE_GPIO0;
					break;
				case MMPF_PIO_REG_GPIO6:
					ubShiftBit = GBL_WAKE_GPIO6;
					break;
				#if (CHIP == VSN_V3)
				case MMPF_PIO_REG_GPIO23:
					ubShiftBit = GBL_WAKE_GPIO23;
					break;
				case MMPF_PIO_REG_GPIO50:
					ubShiftBit = GBL_WAKE_GPIO50;
					break;
				case MMPF_PIO_REG_GPIO63:
					ubShiftBit = GBL_WAKE_GPIO63;
					break;
				#endif
				default:
					return MMP_SYSTEM_ERR_FORMAT;
					break; 
			}
			
			MMPF_PIO_Initialize();
			MMPF_PIO_EnableGpioMode(piopin, MMP_TRUE, MMPF_OS_LOCK_CTX_ISR);
			MMPF_PIO_EnableOutputMode(piopin, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);
				
			if(polarity == 0x1) { //High level wake up
				pGBL->GBL_GPIO_WAKE_INT_POLARITY &= (~ubShiftBit);
			}
			else { //Low level wake up
				pGBL->GBL_GPIO_WAKE_INT_POLARITY |= (ubShiftBit);
			}
			
			pGBL->GBL_WAKE_CTL |= ubShiftBit;
		}
		else if(event == MMPF_WAKEUP_USB_RESUME) {
			if((pUSB_CTL->USB_POWER & 0x1) == 0x0) {
				pUSB_CTL->USB_POWER |= 0x1;
			}
			pGBL->GBL_WAKE_CTL |= GBL_WAKE_USB_RESUME;
		}
		else {
			return MMP_SYSTEM_ERR_FORMAT;
		} 
	}
	else {
		if(event == MMPF_WAKEUP_GPIO) {
			switch(piopin) {
				case MMPF_PIO_REG_GPIO0:
					ubShiftBit = GBL_WAKE_GPIO0;
					break;
				case MMPF_PIO_REG_GPIO6:
					ubShiftBit = GBL_WAKE_GPIO6;
					break;
				#if (CHIP == VSN_V3)
				case MMPF_PIO_REG_GPIO23:
					ubShiftBit = GBL_WAKE_GPIO23;
					break;
				case MMPF_PIO_REG_GPIO50:
					ubShiftBit = GBL_WAKE_GPIO50;
					break;
				case MMPF_PIO_REG_GPIO63:
					ubShiftBit = GBL_WAKE_GPIO63;
					break;
				#endif
				default:
					return MMP_SYSTEM_ERR_FORMAT;
					break; 
			}
			
			MMPF_PIO_EnableGpioMode(piopin, MMP_FALSE, MMPF_OS_LOCK_CTX_ISR);
			pGBL->GBL_WAKE_CTL &= ~ubShiftBit;
		}
		else if(event == MMPF_WAKEUP_USB_RESUME) {
			pGBL->GBL_WAKE_CTL &= ~GBL_WAKE_USB_RESUME;
		}
		else {
			return MMP_SYSTEM_ERR_FORMAT;
		} 
	}
#else
	RTNA_DBG_Str(0, "CPU wake up event un-support ! \r\n");
#endif
	return MMP_ERR_NONE;
}


//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_EnterSelfSleepMode
//  Description :
//------------------------------------------------------------------------------
/** @brief This function is used to do some operations before CPU sleep and make CPU sleep.
           After CPU wake up, the CPU start to run the code after it enters sleep.
           
    NOTE1: This function should co-work with MMPF_SYS_EnterPSMode !!!!
    NOTE2: It can not be placed into DRAM because DRAM will enter slef refresh mode !!
        
@return It reports the status of the operation.
*/
#pragma arm section code = "EnterSelfSleepMode", rwdata = "EnterSelfSleepMode",  zidata = "EnterSelfSleepMode"
MMP_ERR MMPF_SYS_EnterSelfSleepMode(void)
{
	
#if (defined(ALL_FW)||defined(UPDATER_FW))&&(SYS_SELF_SLEEP_ENABLE == 0x1)
	#if (OS_CRITICAL_METHOD == 3)
	static OS_CPU_SR   cpu_sr = 0;
	#endif
	static MMP_UBYTE    reset_sig;
	static MMP_UBYTE    ubClkDis0 = 0x0, ubClkDis2 = 0x0;
	static MMP_USHORT   usClkDis1 = 0x0;
	static MMP_ULONG    counter = 0, ulTempISR = 0x0;
	static AITPS_AIC 	pAIC = AITC_BASE_AIC;
	static AITPS_GBL    pGBL = AITC_BASE_GBL;
	static AITPS_USB_CTL pUSB = AITC_BASE_USBCTL ;
	//RTNA_DBG_Str(0, "Enter Sleep mode \r\n");
	static MMP_USHORT  usb_int ;// = pUSB_CTL->USB_INT_EVENT_SR & pUSB_CTL->USB_INT_EVENT_EN ;
	OS_ENTER_CRITICAL();
	ulTempISR = pAIC->AIC_IMR;
	pAIC->AIC_IDCR = 0xFFFFFFFF;
	
	#if (CHIP == VSN_V2)
	MMPF_SYS_SetWakeUpEvent(MMP_TRUE, MMPF_WAKEUP_GPIO, MMPF_PIO_REG_GPIO6, 0x0);
	#endif
	#if (CHIP == VSN_V3)
	MMPF_SYS_SetWakeUpEvent(MMP_TRUE, MMPF_WAKEUP_GPIO, MMPF_PIO_REG_GPIO23, WOV_SLEPP_TRIGGER_LEVEL);// wait for low level trigger.
	#endif
	//MMPF_SYS_SetWakeUpEvent(MMP_TRUE, MMPF_WAKEUP_USB_RESUME, 0x0, 0x0);// Gason@remove for GPIO suspend
	
	ubClkDis0 = pGBL->GBL_CLK_DIS0;
	usClkDis1 = pGBL->GBL_CLK_DIS1;
	#if (CHIP == VSN_V3)
	ubClkDis2 = pGBL->GBL_CLK_DIS2;
	#endif
	
	MMPF_DRAM_SetSelfRefresh(MMP_TRUE);
	// ?????
	//if( (pUSB->USB_POWER & 0x02) == 0 ) {
	//    reset_sig = 1 ;
	//    goto exit_ps ;    
	//}
	
	usb_int = pUSB->USB_INT_EVENT_SR & pUSB->USB_INT_EVENT_EN ;
	
	if( usb_int & (RESUME_INT_BIT | RESET_INT_BIT)) {
	    reset_sig = 1 ;
	    goto exit_ps ;
	}
	
	pGBL->GBL_CLK_DIS0 = (MMP_UBYTE)(~(GBL_CLK_MCI_DIS | GBL_CLK_CPU_DIS | GBL_CLK_GPIO_DIS));
//	pGBL->GBL_CLK_DIS1 = (MMP_USHORT)(~(GBL_CLK_DRAM_DIS | GBL_CLK_USB_DIS | GBL_CLK_CPU_PHL_DIS));
	pGBL->GBL_CLK_DIS1 = (MMP_USHORT)(~(GBL_CLK_DRAM_DIS | GBL_CLK_USB_DIS ));
	#if (CHIP == VSN_V3)
	pGBL->GBL_CLK_DIS2 = (MMP_UBYTE)(GBL_CLK_AUD_CODEC_DIS | GBL_CLK_CIDC_DIS | GBL_CLK_GNR_DIS | GBL_CLK_SM_DIS | GBL_CLK_COLOR_DIS); 
	#endif
	
	pGBL->GBL_LCD_BYPASS_CTL0 |= GBL_ENTER_SELF_SLEEP_MODE;

	//When set 0x8F88 enter sleep mode, CPU still alive in a short time
	//So, need to insert some useless commands before setting 0x8F88 as zero
	#pragma O0
	for(counter = 0x1000; counter > 0 ; counter --) {
	}
	#pragma

	pGBL->GBL_LCD_BYPASS_CTL0 &= (~GBL_ENTER_SELF_SLEEP_MODE);
	pGBL->GBL_CLK_DIS0 = ubClkDis0;
	pGBL->GBL_CLK_DIS1 = usClkDis1;
	#if (CHIP == VSN_V3)
	pGBL->GBL_CLK_DIS2 = ubClkDis2; 
	#endif
	
exit_ps:
	
	MMPF_DRAM_SetSelfRefresh(MMP_FALSE);

	#if (CHIP == VSN_V2)
	MMPF_SYS_SetWakeUpEvent(MMP_FALSE, MMPF_WAKEUP_GPIO, MMPF_PIO_REG_GPIO6, 0x0);
	#endif
	#if (CHIP == VSN_V3)
	MMPF_SYS_SetWakeUpEvent(MMP_FALSE, MMPF_WAKEUP_GPIO, MMPF_PIO_REG_GPIO23, WOV_SLEPP_TRIGGER_LEVEL); 
	#endif
	//MMPF_SYS_SetWakeUpEvent(MMP_FALSE, MMPF_WAKEUP_USB_RESUME, 0x0, 0x0); // Gason@remove for GPIO suspend
	
	pAIC->AIC_IECR = ulTempISR;
	OS_EXIT_CRITICAL();
	
	if(reset_sig) {
        //VAR_B(0,USB_REG_BASE_B[USB_POWER_B] );
	    RTNA_DBG_Str(0, "-ExPS.Reset-\r\n");
	} else {
	    RTNA_DBG_Str(0, "-ExPS-\r\n");
	}
	
	#else
	RTNA_DBG_Str(0, "CPU slef sleep mode un-support ! \r\n");
#endif
	return MMP_ERR_NONE;

}
#pragma arm section code, rwdata,  zidata
//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_EnterPSMode
//  Description : called by the task with host command queue to process
//                command for System group
//------------------------------------------------------------------------------
// 0 : Exit Power Saving Mode
// 1 : Enter Power Saving Mode
// 2 : Power On Initial the unused Moudle
MMP_ERR MMPF_SYS_EnterPSMode(MMP_BOOL bEnterPSMode)
{
#define DISABLE_USB_PHY (1) // Enable for more power consumption
extern MMP_UBYTE  gbUsbHighSpeed;
	#if (defined(ALL_FW)||defined(UPDATER_FW))&&(SYS_SELF_SLEEP_ENABLE == 0x1)
	AITPS_GBL   pGBL = AITC_BASE_GBL;
//	AITPS_USB_DMA   pUSBDMA = AITC_BASE_USBDMA;
	//AITPS_MIPI pMIPI = AITC_BASE_MIPI;
	volatile MMP_UBYTE* REG_BASE_B = (volatile MMP_UBYTE*)0x80000000;
//	static MMP_BOOL ubMipiUseLan0 = MMP_FALSE, ubMipiUseLan1 = MMP_FALSE;
	
	pGBL->GBL_CLK_DIS0 &= ~GBL_CLK_VI_DIS  ;
	 
	if(bEnterPSMode == MMP_TRUE) {
		//MIPI RX
		REG_BASE_B[0x6160] &= 0xFB; // MIPI DPHY0 use HW design OPR to control Power-down
		REG_BASE_B[0x6170] &= 0xFB;  // MIPI DPHY1 use HW design OPR to control Power-down
        #if DISABLE_USB_PHY==1
        //USB PHY
        //sean@2012_09_12,for FS suspend/resume test 
        if(gbUsbHighSpeed) {
            SPI_Write(0x00, 0x0B00);
            SPI_Write(0x0A, 0x8520); 
            SPI_Write(0x0C, 0xff40); // 0xff40
        }
        #endif
        
		pGBL->GBL_LCD_BYPASS_CTL0 |= GBL_LCD_BYPASS_PWN_DPLL;
		pGBL->GBL_LCD_BYPASS_CTL0 &= (~ GBL_LCD_BYPASS_CLK_ACTIVE);
		pGBL->GBL_LCD_BYPASS_CTL0 |= GBL_XTAL_OFF_BYPASS;
	}
	else {
	
		pGBL->GBL_LCD_BYPASS_CTL0 &= (~GBL_LCD_BYPASS_PWN_DPLL);
		pGBL->GBL_LCD_BYPASS_CTL0 |= GBL_LCD_BYPASS_CLK_ACTIVE;
		pGBL->GBL_LCD_BYPASS_CTL0 &= (~GBL_XTAL_OFF_BYPASS);
		//USB PHY
        #if DISABLE_USB_PHY==1
        //USB PHY
        if(gbUsbHighSpeed) {
            SPI_Write(0x0C, 0x0000);
            SPI_Write(0x0A, 0x0020);
            SPI_Write(0x00, 0x0300);
            RTNA_WAIT_US(100); // wait 100 us for 0x0B
        }
        #endif
          
	}
	pGBL->GBL_CLK_DIS0 |= GBL_CLK_VI_DIS ;
	
	#endif //#if defined(ALL_FW)
	return  MMP_ERR_NONE;

}

MMP_ERR MMPF_SYS_ConfigIOPad(void)
{
    //T.B.D for VSN_V2
    return MMP_ERR_NONE ;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_EnterBypassMode
//  Description : called by the task with host command queue to process
//                command for System group
//------------------------------------------------------------------------------
MMP_ERR MMPF_SYS_EnterBypassMode(MMP_BOOL bEnterBypassMode)
{
    AITPS_GBL   pGBL = AITC_BASE_GBL;

    if (bEnterBypassMode) {
      	pGBL->GBL_LCD_BYPASS_CTL0 &= (~GBL_LCD_BYPASS_PWN_DPLL);
		pGBL->GBL_LCD_BYPASS_CTL0 |= GBL_LCD_BYPASS_CLK_ACTIVE;
    }
    return  MMP_ERR_NONE;
}

//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_SuspendCPU
//  Description : called by the task with host command input to suspend
//                the OS timer
//------------------------------------------------------------------------------
MMP_ERR MMPF_SYS_SuspendCPU(MMP_BOOL bSuspend)
{
    AITPS_AIC   pAIC = AITC_BASE_AIC;

    if (bSuspend) {
		RTNA_AIC_IRQ_Dis(pAIC, AIC_SRC_TC0);
    }
    else {
		RTNA_AIC_IRQ_En(pAIC, AIC_SRC_TC0);
    }

    return  MMP_ERR_NONE;
}
//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_InitializeHIF
//  Description : Initialize host command ISR
//------------------------------------------------------------------------------
#if (defined(ALL_FW)&&(PCAM_EN == 0))

MMP_ERR MMPF_SYS_InitializeHIF(void)
{
    AITPS_AIC   pAIC = AITC_BASE_AIC;
    AITPS_GBL   pGBL = AITC_BASE_GBL;

    pGBL->GBL_HOST2CPU_INT_SR = GBL_HOST2CPU_INT;
    pGBL->GBL_HOST2CPU_INT_EN |= GBL_HOST2CPU_INT;

    RTNA_AIC_Open(pAIC, AIC_SRC_HOST, hif_isr_a,
                AIC_INT_TO_IRQ | AIC_SRCTYPE_HIGH_LEVEL_SENSITIVE    | 3);

    RTNA_AIC_IRQ_En(pAIC, AIC_SRC_HOST);

    return MMP_ERR_NONE;
}
#endif
/** @} */ // end of MMPF_HIF

//------------------------------------------------------------------------------
//  Function    : MMPF_SYS_GetFWEndAddr
//  Description : called by the task with host command queue to process
//                command for System group
//------------------------------------------------------------------------------

#if defined(ALL_FW)
extern unsigned int Image$$ALL_DRAM$$ZI$$Limit;
#endif
MMP_ERR MMPF_SYS_GetFWEndAddr(MMP_ULONG *ulEndAddr)
{
	#if defined(ALL_FW)
	*ulEndAddr = ((MMP_ULONG)&Image$$ALL_DRAM$$ZI$$Limit + 0xFFF) & (~0xFFF);
	#endif

    return MMP_ERR_NONE;
}

#if 0
/** @addtogroup MMPF_SYS
@{
*/

void SYS_Task(void)
{
	AITPS_GBL   pGBL = AITC_BASE_GBL;
   	#if !defined(ALL_FW)
    MMPF_OS_FLAGS   flags;
    #endif 
    
    #if defined(ALL_FW)
    msg_t *msg=0 ;
	MMP_USHORT err = MSG_ERROR_NONE ;
	#endif
    	
    RTNA_DBG_Str(0, "SYS_Tack() \r\n");	
    #if defined(ALL_FW)
	usb_ep_protect_sem = MMPF_OS_CreateSem(1);
 	gUSBDMASem = MMPF_OS_CreateSem(1);
	RTNA_DBG_PrintLong(0, gUSBDMASem);
	
	if(usb_ep_protect_sem==0xFF) {
        RTNA_DBG_Str(3,"<<usb_ep_protect_sem create sem err>>\r\n");
    }
	
	MMPF_I2cm_InitializeDriver();
	
	RTNA_DBG_PrintLong(0, usb_ep_protect_sem);
	init_msg_queue(1, SYS_Flag_Hif, SYS_FLAG_SYS);
	#endif
	
	
	/*while(1) {
		MMPF_OS_Sleep(1000);
		RTNA_DBG_Str(0, "!");
	}*/
	gbSystemCoreID = pGBL->GBL_CHIP_VER;
    m_gsISPCoreID = 989; //modify from 988
 
    //dbg_printf(0,"Core Id:%x,ISP Id:%d\r\n",gbSystemCoreID,m_gsISPCoreID);
    
	#if (DSC_R_EN)||(VIDEO_R_EN)||(VIDEO_P_EN)
    MMPF_DMA_Initialize();
   
    #endif
    
    #if (defined(ALL_FW)&&(PCAM_EN == 0))
	#if (VIDEO_P_EN)
    MMPF_Graphics_Initialize();
    #elif (VIDEO_R_EN)
    MMPF_Graphics_Initialize();
	#endif
	
	#endif
	
	
	RTNA_DBG_Str(0,"System Task Gason start.\r\n");
    while (TRUE) {
    #if defined(MBOOT_FW)||defined(UPDATER_FW)
        MMPF_OS_WaitFlags(SYS_Flag_Hif, SYS_FLAG_SYS,
                    MMPF_OS_FLAG_WAIT_SET_ANY | MMPF_OS_FLAG_CONSUME, 0, &flags);
    #endif
// add task event for usb interrupt here.
	#if defined(ALL_FW)
	msg = MMPF_SYS_GetMsg();
	if(!msg) {
	  continue ;
	}

    switch(msg->msg_id) {
		case SYS_MSG_USB_EP0_TX_INT:
			UsbEp0IntHandler();
			break;
		case SYS_MSG_USB_RESET_INT:
            USBDevAddr=0;
            UsbRestIntHandler();
            //RTNA_DBG_Str(0,"UsbRestIntHandler\r\n");
			break;
		case SYS_MSG_USB_SUSPEND_INT:
		#if USB_SUSPEND_TEST==1
            if(gbUSBSuspendEvent){
                gbUSBSuspendEvent = 0;
                if(gbUSBSuspendFlag == 0){
                    RTNA_DBG_Str(0, "-S3 SysTask-\r\n");
                    gbUSBSuspendFlag = 1;
                    USB_SuspendProcess() ;
                }
            }
        #endif

		    break ;
		case SYS_MSG_USB_EP3_RX_INT:
		     UsbEp3RxIntHandler();
			break;	
		case SYS_MSG_USB_EP3_TX_INT:
			 //UsbEp3TxIntHandler();
			break;
		case SYS_MSG_USB_EP4_TX_INT:
			 UsbEp4TxIntHandler();
			break;
		case SYS_MSG_USB_EP1_RX_INT:
			UsbEp1RxIntHandler();
			break;
		case SYS_MSG_USB_EP1_TX_INT:	//ANDY 20110617
			UsbEp1TxIntHandler();
			break;
		case SYS_MSG_USB_EP2_RX_INT:
			UsbEp1RxIntHandler();
			break;
		case SYS_MSG_USB_EP4_RX_INT:
			 UsbEp4RxIntHandler();
			break;			
		case SYS_MSG_USB_DMA_EP1_RX_INT:
			UsbEp1RxIntHandler();
			break;
		case SYS_MSG_USB_DMA_EP2_TX_INT:
			UsbEp2TxIntHandler();
			break;
		case SYS_MSG_USB_DMA_EP1_TX_INT:
			UsbEp1TxIntHandler();
			break;
		
    	default:
			RTNA_DBG_Str(0, "unregconize USB intr\r\n");
			break;
    }
		msg->err = err;
        /*Handler processed done*/
        /*Release semaphore*/
       /* if(msg->msg_sem) {
            MMPF_OS_ReleaseSem(*msg->msg_sem);
        } else {
            free_msg(msg, 1);
        }*/

    free_msg(msg, 1);


#if (defined(ALL_FW)&&(PCAM_EN == 0))
        MMPF_SYS_ProcessCmd();

        #ifdef BUILD_CE
        if (bWaitForSysCommandDone)
        {
	        MMPF_OS_SetFlags(SYS_Flag_Hif, SYS_FLAG_SYS_CMD_DONE, MMPF_OS_FLAG_SET);
	    }
        #endif
#endif

	#endif //#if defined(ALL_FW)        
    }
}
#endif 

#if defined(MBOOT_FW) || defined(UPDATER_FW)
void MMPF_Display_GpioISR(void) {}
void MMPF_Display_IbcISR(void) {}

#endif // (OS_TYPE == OS_UCOSII)

/** @}*/ //end of MMPF_SYS
#endif
