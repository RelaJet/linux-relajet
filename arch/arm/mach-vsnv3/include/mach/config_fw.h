///@ait_only
/** @file config_fw.h

All customer-dependent compiler options go here.
The purpose is to prepare many versions of the file to different use cases,
such as different customers and different platforms. When compiling, simply replace this file and then compile the whole project.
This file could be used both in firmware projects.

This file should not be included in includes_fw.h.
Because the file is used to config and recompile the whole project,
it's better to be used only in the file which needs the config.
And it's highly recommended to use a value in the \#define instead of \#define only. That is
@code #define DOWNLOAD_METHOD 0 @endcode
is better than
@code #define USE_SD_DOWNLOAD_FW @endcode \n
Since the .c files might not include this file and get an unexpected result with any compiler warning.

@brief Configuration compiler options
@author Truman Yang
@since 10-aug-06
@version
- 1.0 Original version
*/


#ifndef _CONFIG_FW_H_
#define _CONFIG_FW_H_

    //#define CHIP_CORE_ID_MCR_V2_SHT     (0x82)
    //#define CHIP_CORE_ID_MCR_V2_MP       (0x83)
    //#define SYSTEM_CORE_ID	CHIP_CORE_ID_MCR_V2_MP
	
/** @name System
@{ */
    // Python V1 : 160KB sram
    #define EXT_PMCLK_CKL	12000
    #define DRAM_ADAPTIVE_CLK_DLY   (1)

    #if (CHIP == MCR_V2)
    #define MCR_V2_UNDER_DGB        (1)
    #endif
/** @} */ // end of System

    #if defined(MBOOT_FW)||defined(UPDATER_FW)
	extern unsigned int glCPUFreq;
	#define RTNA_CLK_M                  (glCPUFreq/1000)
	#define RTNA_CPU_CLK_M              (RTNA_CLK_M/1)
    #endif

    #define USB_SQUELCH_LEVEL      (0x0D37)  //420mV //(0x0D0F) // (0x0D17)

	#define TURN_ON_WOV_BY_DEFAULT (0)

    #define VREF_CAPACITOR_UF   (1)        // ext capacitor for VREF
    #define ANA_PDN_DLY_MS      (1)
    #define ADC_PGA_PDN_DLY_MS  (10*VREF_CAPACITOR_UF)

    #define NORMAL_REL          (0)
    #define JTAGDBG_REL         (1)

    #define FW_RELEASE_TARGET   JTAGDBG_REL//NORMAL_REL

    #define JTAG_DBG_EN         (1) // Enable Uart debug message
    #if JTAG_DBG_EN==0
    #define HEARTBEAT_LED_EN    (0) // if GPIO-8(8423) connect a LED in AIT EV - board. (need to check )
    #define JTAG  (0)
    #else
    #define HEARTBEAT_LED_EN    (0)
    #define JTAG  (1)
    #endif

    /** @brief Define if version B code is compiled.
    */

	#define	VER_A	(0)
	#define	VER_B	(1)
	#define	VER_C	(2)

	#define DEMOKIT  (1)
	#define ADAGIO   (2)
	#define PRESTIGE (3)
	#define STAGE    (4)
	#define PURE     (5)
	#define HELIUM   (6)
	#define PROJECT     (DEMOKIT)

	#define KYE       (1)
	#define BIS       (2)
	#define AZW       (3)
	#define SAL       (4)
	#define LGT       (5)
	#define DMX       (6)
	#define FCN       (7)
	#define BIS_5M    (8)
	#define SEM       (9)
	#define LON       (10)
	#define STK       (11)
	#define NMG       (12)
	#define CUSTOMER    (LGT)

    #define FALCON      (0x00)

    #define PYTHON_ECO    (0)

	#define I2S_OUT_EN 					(0) //wilson: for VSN_V3
    // definition for MIC IN path
	#define MIC_IN_PATH_AFE                	(1)
	#define MIC_IN_PATH_I2S                	(2)
	#define MIC_IN_PATH_BOTH				(3)

	//only vsn_v3 can set this define as MIC_IN_PATH_BOTH
	#define MIC_SOURCE                   	(MIC_IN_PATH_AFE)

	#if (MIC_SOURCE==MIC_IN_PATH_I2S) || (MIC_SOURCE==MIC_IN_PATH_BOTH)
	#define AUDIN_CHANNEL               2
	#define AUDIN_SAMPLERATE            16000 // V06 : change to 32000 for default,// 16000
	#else
	#define AUDIN_CHANNEL               2 // sean@2010_08_12 for stere test
	#define AUDIN_SAMPLERATE            16000
	#endif

	//wilson@120406
	#define UAC_FIFO_ISR_EN				0x00
	#define UAC_TIMER_ISR_EN			0x01
	#define UAC_DATA_ISR_TYPE			UAC_FIFO_ISR_EN //UAC_TIMER_ISR_EN

	#define Monitor_Audio 				1
	#define Monitor_Video 				1
	#define NO_DAC                      0
	#define WM8737                      0x8737
	#define CX20709                     2
	//wilson@120301: for da7211
	#define DA7211						0x7211
	#define CX20869                     0x20869
	#define WM8973                      0x8973
	#define WM8750                      0x8750

	#if (MIC_SOURCE==MIC_IN_PATH_I2S) || (MIC_SOURCE==MIC_IN_PATH_BOTH)
	#define AUDEXT_DAC                  CX20869//DA7211//CX20709
	#if (FALCON == 1)
	#undef AUDEXT_DAC
	#define AUDEXT_DAC                  WM8973
	#endif
	#else
	#define AUDEXT_DAC                  NO_DAC
	#endif

	#define SUPPORT_AGC                 (0)
	#define FW_VER_BCD                  (0x0730)  // for USB BCDdevice version

	#define SENSOR_SXGA_CROPPING_HD     (0)        // only for sensor cropping issue
	#define PCCAM_VIDEO_COMPRESS_BUF    (0x01800000)

    #define SUPPORT_AUTO_FOCUS          (0)
    #define SUPPORT_DIGITAL_ZOOM        (1)
    #define SUPPORT_DIGITAL_PAN         (1)

    #define SUPPORT_UAC                 (1)

    #if defined(UPDATER_FW)||defined(MBOOT_FW)
    #define AUTO_DRAM_LOCKCORE          (1)
    #else
    #define AUTO_DRAM_LOCKCORE          (0)
    #endif

	#if defined(UPDATER_FW)
		#define SUPPORT_PCSYNC				(0)
		#define MSDC_SUPPORT_AIT_SPECIAL_SCSI_CMD	(1)
		#define AIT_SCSI_DYNAMIC_SWITCH_BOOT		(0)
		#define ENABLE_USB_EJECT_FEATURE	(0)
	#else
    	#define SUPPORT_PCSYNC				(1)
    #endif
    #if SUPPORT_PCSYNC
		#define PCSYNC_DBG_EP_ADDR		(5)
		#define PCSYNC_EP_ADDR			(4)	//endpoint 2 bulk-in/out
		#define PCSYNC_EP_MAX_PK_SIZE	(512)
    #endif

    #define H264_LOW_BITRATE_CONTROL    (0)
    #define AITCAM_MULTI_STREAM_EN      (0)

    #define SKYPE_H264_TEST             (0)  // 1: VGA H264, 0: HD H264
    #define DEFAULT_VIDEO_FMT_H264      (0)  // 0: YUV/MJPEG, 1: H.264

    #define LCD_DISPLAY_ON              (0)

    #define RAW_PROC_10_BIT_EN          (0)    // 0: 8 bits bayer raw, 1: 10 bits bayer raw

    // New definition for GNR OFF in high lux
	#define IQ_GNR_HIGH_LUX_OFF			(1)

    #define SUPPORT_AUDIO_PROCESSING_TASK    (1)  // support audio processing task for customer's algorithm
#if (CUSTOMER == NMG)
    #define CAM_EN         38  // CGPIO6
    #define AF_EN          40  // CGPIO8
    #define LED_READY      8   // BGPIO8
    #define SNAP_PIN       15  // BGPIO15
    #define FLIP_PIN       18  // BGPIO18
    #define IRHID_PIN      39  // CGPIO7 for PCB v5
#else
    #define CAM_EN         8  // BGPIO6
    #define LED_READY      8  // 8423
    #define AF_EN          18  // BGPIO18
    #define SNAP_PIN       255
#endif

    #define TEST_PIN1      255  // if no used, change to 255
    #define TEST_PIN2      255
    #define TEST_PIN3      255

	#if (LED_READY!=255)
	#define LED_READY_PAD_CFG (PAD_E8_CURRENT | PAD_PULL_LOW)
	#endif
	#if (AF_EN!=255)
	#define AF_EN_PAD_CFG (PAD_E8_CURRENT | PAD_PULL_LOW)
	#endif
	#if (SNAP_PIN!=255)
	#define SNAP_PIN_PAD_CFG (PAD_E8_CURRENT | PAD_PULL_LOW)
	#endif
	#if (TEST_PIN1!=255)
	#define TEST_PIN1_PAD_CFG (PAD_E8_CURRENT | PAD_PULL_LOW)
	#endif
	#if (TEST_PIN2!=255)
	#define TEST_PIN1_PAD_CFG (PAD_E8_CURRENT | PAD_PULL_LOW)
	#endif
	#if (TEST_PIN3!=255)
	#define TEST_PIN1_PAD_CFG (PAD_E8_CURRENT | PAD_PULL_LOW)
	#endif

    #define GPIO_INT_H2L   (0)
    #define GPIO_INT_L2H   (1)

	/** @brief JTAG configuration
     * Define which UART to use, and UART padset for output
     */
    #define SYS_JTAG_PAD_NONE 	(0x0)
	#define SYS_JTAG_PAD_BGPIO	(0x1)
	#define SYS_JTAG_PAD_DGPIO	(0x2)
    #define SYS_JTAG_PAD (SYS_JTAG_PAD_DGPIO)// SYS_JTAG_PAD_BGPIO for T32

    #define CPU_A       (0)
    #define CPU_B       (1)

    #ifndef CPU_ID
        #define CPU_ID   (CPU_A)
    #endif

    /** @brief UART configuration
    Define which UART to use, and UART padset for output
    */
    #define DEBUG_UART_NUM (MMPF_UART_ID_0)

	#if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
	#define DEBUG_UART_PIN (MMPF_UART_PADSET_0)		// for JTAG mode using Pad Set3(PGPIO29/PGPIO30)
	#endif
    /// Debug Level: see @ref debug_level for more detail.
    #define DBG_LEVEL (3)

/** @name System Clock
@{ */
	#if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
	#define EXT_CLK 12000 //for 12 MHz
	#endif
	//#define EXT_CLK 19200 //for 19.2 MHz

    #define SUPPORT_CPU_CLOCK_SW    (1) //1: Support dynamically CPU clock freq switch
    #define SUPPORT_G0_CLOCK_SW     (1) //1: Support dynamically group 0 clock freq switch
    #define SUPPORT_DRAM_CLOCK_SW   (1) //1: Support dynamically DRAM clock freq switch
    #if (SUPPORT_G0_CLOCK_SW == 0)
        #ifdef SUPPORT_DRAM_CLOCK_SW
        #undef SUPPORT_DRAM_CLOCK_SW
        #define SUPPORT_DRAM_CLOCK_SW   (0)
        #endif
    #endif

/** @} */ // end of System Clock


/** @name MMU Table address
@{ */
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    #define MMU_TRANSLATION_TABLE_ADDR         (0x100000)
    #define MMU_COARSEPAGE_TABLE_ADDR          (0x100400)
    #endif
/** @} */ // end of MMU Table address


/** @name USB Clock
@{ */
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
	#define	USB_CLKINPUT_CRYSTAL	(0)
	#define	USB_CLKINPUT_PLL		(1)
	#define	USB_CLKFREQ_12MHZ		(0)
	#define	USB_CLKFREQ_30MHZ		(1)

	#define	USB_CLK_INPUT 	USB_CLKINPUT_CRYSTAL
	#define	USB_CLK_FREQ  	USB_CLKFREQ_12MHZ

    #define USB_PHY_TEST_MODE_EN (1)

	#endif
/** @} */ // end of USB Clock

/** @name Audio Setting
@{ */


/** @} */ // end of Audio Setting

/** @name Sensor
@{ */

	#define SENSOR_EN  1

    #define TOTAL_SENSOR_NUMBER      1
	#define SENSOR_ID_OV9710         0
	#define SENSOR_ID_OV9726         0
	#define SENSOR_ID_OV2722         0
    #define SENSOR_ID_OV2724         0
	#define SENSOR_ID_OV5690         0
    #define SENSOR_ID_AR0330         1
    #define SENSOR_ID_IMX175         0
    #define SENSOR_ID_IMX188         0
    #define BIND_SENSOR_MT9M011      0
    #define BIND_SENSOR_MT9M112      0
    #define BIND_SENSOR_MT9T013      0
    #define BIND_SENSOR_MT9T012      0
    #define BIND_SENSOR_MT9T111R3    0
    #define BIND_SENSOR_MT9D011      0
    #define BIND_SENSOR_MT9D111      0
    #define BIND_SENSOR_MT9P001      0
    #define BIND_SENSOR_MT9P012      0

	#define BIND_SENSOR_OV2650       0
    #define BIND_SENSOR_OV2724       0
	#define BIND_SENSOR_OV3642       0
    #define BIND_SENSOR_OV7660       0
    #define BIND_SENSOR_OV7670       0
    #define BIND_SENSOR_OV9650       0
    #define BIND_SENSOR_OV9653       0
    #define BIND_SENSOR_OV9655       0
    #define BIND_SENSOR_OV9660       0
    #define BIND_SENSOR_HV7131RP     0
	#define BIND_SENSOR_S5K3C1FX	 0
	#define BIND_SENSOR_S5K4BAFB	 0
	#define BIND_SENSOR_S5K5BAFX	 0
	#define BIND_SENSOR_C2FKA244A    0
    //MIPI
    #define BIND_SENSOR_IMX175       0
	#define BIND_SENSOR_IMX045ES     0
	#define	BIND_SENSOR_IMX046TS	 0
	#define BIND_SENSOR_TCM9001MD    0
	#define	BIND_SENSOR_OV5650		 0
	#define	BIND_SENSOR_OV9726		 0
	#define	BIND_SENSOR_S5K6A1GX	 0
	#define	BIND_SENSOR_S5K4B2FX	 0
	#define	BIND_SENSOR_MT9P111	     0
	#define	BIND_SENSOR_OV2710		 0

	#define	BIND_SENSOR_OV9710		 0
	#define	BIND_SENSOR_MT9T002	     0
    #define BIND_SENSOR_AR0330       1
	#define	BIND_SENSOR_S5K5B3GX	 0
	#define BIND_SENSOR_MT9P014      0
	#define BIND_SENSOR_OV5653		 0
	#define BIND_SENSOR_VENUS 	 	 0
	#define BIND_SENSOR_S5K4E5       0
    #define BIND_SENSOR_OV10822      0

	#define BIND_SENSOR_OV5690       0
	#define BIND_SENSOR_OV2722       0
	#define	BIND_SENSOR_IMX188		 0


    #define SENSOR_IF_PARALLEL       (0)
    #define SENSOR_IF_MIPI_1_LANE    (1)
    #define SENSOR_IF_MIPI_2_LANE    (2)
    #define SENSOR_IF_MIPI_4_LANE    (3)

#if (BIND_SENSOR_OV9710) || (BIND_SENSOR_OV9726)
    #define SENSOR_IF                (SENSOR_IF_PARALLEL)
#elif  (BIND_SENSOR_AR0330) 
    #define SENSOR_IF                (SENSOR_IF_MIPI_2_LANE) // Krypto : 2 - lanes
#elif (BIND_SENSOR_OV2724) || (BIND_SENSOR_OV5690) || (BIND_SENSOR_OV5650) || (BIND_SENSOR_S5K4E5)\
    || (BIND_SENSOR_IMX175)
    #define SENSOR_IF                (SENSOR_IF_MIPI_2_LANE)
#else
    #define SENSOR_IF                (SENSOR_IF_MIPI_1_LANE)
#endif

#if ((CUSTOMER == LGT) && (BIND_SENSOR_OV2710)) || ((CUSTOMER == LGT) && (BIND_SENSOR_S5K5B3GX))
    #define SUPPORT_DEFAULT_ZOOM        (0)    // For Skype FOV, 1920x1080 -> 1280x720
#else
    #define SUPPORT_DEFAULT_ZOOM        (0)    // For Skype FOV, 1920x1080 -> 1280x720
#endif

#if SUPPORT_DEFAULT_ZOOM
    #if ((CUSTOMER == LGT) && (BIND_SENSOR_OV2710))
    #define DEFAULT_ZOOM_STEP           (4)
    #endif
    #if ((CUSTOMER == LGT) && (BIND_SENSOR_S5K5B3GX))
    #define DEFAULT_ZOOM_STEP           (4)
    #endif
#endif

    #define AUQILA_DYNAMIC_CHANGE_RESOLUTION (1)

    #define TEST_1152_648_MODE          (0)  // test only for MT9T002 3M sensor
    #define CROP_1080P_FROM_2304_1296   (0)  // test only for MT9T002 3M sensor
    #define ENABLE_ROW_BINNING_2304_648   (0)  // test only for MT9T002 3M sensor

	#if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
	#if (BIND_SENSOR_S5K4BAFB)
	#define	SOFTWARE_I2CM
	#else
	#undef SOFTWARE_I2CM
	#endif
	#endif

	#if	defined(SOFTWARE_I2CM)
	#if (BIND_SENSOR_S5K4BAFB)
	#define I2C_DELAY           70
	#else
	#define I2C_DELAY           30
	#endif
	#endif

    /** @brief Choose which initialization method to be used
    @note There should be new implementation for each customer.
    */
	/// @brief Sensor reset GPIO pin
	#define MIPI_LANE_CTL (0) // 0: Single lane 0, 1: Single lane 1, 2: Dual Lane.
	#define SENSOR_SINGLE_LANE_0 0
	#define SENSOR_SINGLE_LANE_1 0
	#define SENSOR_DUAL_LANE 1

    /** @} */ // end of Sensor

/** @name Video Player
@{ */

    /// Use software decoder for debugging or not. The SW library have to be added.
    #define SW_DECODER (0x10000)
    #define HW_MP4V (263)                  ///< Use hardware mpeg4 decoder
    #define SW_MP4V (HW_MP4V | SW_DECODER) ///< Use software mpeg4 decoder
    #define HW_H264 (264)                  ///< Use hardware h.264 decoder
    #define SW_H264 (HW_H264 | SW_DECODER) ///< Use software h.264 decoder

	/** @brief Support Rotate with DMA feature or not 0 for support. Set 1 to support.

	Set 0 to save code size. Some customer uses 823 and would never use this feature.
	@note If there is a chip ID config, replace with it.
	This could be removed later because we have CHIP_NAME now
	*/
	#ifdef VIDEO_DECODER
    #if VIDEO_DECODER == SW_H264
    	#define ROTATE_DMA_SUPPORT (1)
    #else
        #define ROTATE_DMA_SUPPORT (1) // If using 823, turn off this option to save code size.

        #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
        #define GRAPHIC_SCALER_SUPPORT  (1)
        #endif
    #endif
    #endif

/** @name Video Recorder
@{ */
	/** @brief Support landscape mode recording by rotate DMA.

	Instead of using LCD controller, frames for preview (in pipe 0) is rotated by rotate DMA.
	Presently it is only suitable for recording with large resolution (width>352),
	thus there will be distinct pipes for preview and encode.
	*/
	#define MGR_PROC_EN         (0)
	#define H264_SW_CODING_EN   (0)
	#define MTS_MUXER_EN        (0)
	#define VIDEO_FPS_SCALE     (100)
    #define LANDSCAPE_SUPPORT   (1)
    #define VID_TIME_SYNC_ST    (0)
    #define SLOT_RING           (0)
    #define FORCE_BASELINE_SUB_BLK_EN   (0) // may have level violation in small resolution(<720x576)
    #define H264E_RINGBUF       1
    #define SIMCAST_DROP        0
    #define SIMCAST_DROP_ID     1
    #define MIN_H264E_COMP_RATIO        (8)
    #define INIT_I_FRAME_NUM    (1)     // Number of consecutive i frames at start,
                                        // for UVC driver may loss 1st frame issue
    #define KITE_ONLY_VIDEO_PATCH       (1)
    #define FLAG_EOF_IN_LAST_SLICE      1
    #if (FLAG_EOF_IN_LAST_SLICE == 1)
        #define H264_UVC_PH_FLAG_IDR        (0x80000000)
        #define H264_UVC_PH_FLAG_EOF        (0x40000000)
        #define H264_UVC_PH_LAYER_ID_MASK   (0x0000FFFF)
        #define H264_UVC_PH_LAYER_ID(_a)    (_a & H264_UVC_PH_LAYER_ID_MASK )
        #define UVC_PH_EXT_BIT_SHIFT        (31)
        #define UVC_PH_EXT_BIT_MASK         (1<<UVC_PH_EXT_BIT_SHIFT)
    #endif
    #if (CHIP == VSN_V2)
    #define H264E_RINGBUF_SIZE  (400*1024)  // for H.264 720p
    #endif
    #if (CHIP == MCR_V2) || (CHIP == MERCURY) || (CHIP == VSN_V3)
    #define H264E_RINGBUF_SIZE  (640*1024)  // for H.264 1080p
    #endif
    /** @} */ // end of Video Recorder

/** @name storage
@{ */
	#define	FS_USE_UCS2			(0)
    #define SD_CARD_DETECT      (0)   // 1: enable card detect function, otherwise, don't use
    #define SD_WRITE_PROTECT    (0)   // 1: enable card write protect, otherwise, don't use
    #if (PROJECT == PRESTIGE||PROJECT == PURE)
    #define CARD_DET_PIN_NUM    (19)   // GPIO pin number
    #define SD_PWR_CTL          (1)   // 1: enable card power control, otherwise, don't use
    #define CARD_PWR_PIN_NUM    (7)   // GPIO pin number
    #define CARD_PWR_POLARITY   (0)   // card power enable ploarity, 1: power enable high active
    #define SD_PIN_PAD_NUM      (0)
    #endif
    #if (PROJECT == STAGE)
    #define CARD_DET_PIN_NUM    (19)   // GPIO pin number
    #define SD_PWR_CTL          (1)   // 1: enable card power control, otherwise, don't use
    #define CARD_PWR_PIN_NUM    (24)   // GPIO pin number
    #define CARD_PWR_POLARITY   (0)   // card power enable ploarity, 1: power enable high active
  	#define SD_PIN_PAD_NUM      (1)
    #endif
    #if (PROJECT == DEMOKIT)
    #define SD_PWR_CTL          (0)   // 1: enable card power control, otherwise, don't use
    #define CARD_DET_PIN_NUM    (6)   // GPIO pin number
    #define CARD_PWR_PIN_NUM    (7)   // GPIO pin number
    #define CARD_PWR_POLARITY   (0)   // card power enable ploarity, 1: power enable high active
    #define SD_PIN_PAD_NUM      (0)
    #endif
    #define CARD_DET_POLARITY   (1)   // Card detect polarity, 1: pin value is high when card in
    #define CARD_WP_PIN_NUM     (7)   // GPIO pin number
    #define CARD_WP_POLARITY    (1)   // card write protect ploarity, 1: pin value is high when write protect

	/* Config SD card*/
    #define EN_CARD_DETECT          (0)   // 1: enable card plug-in detect function, otherwise, don't use
    #define EN_CARD_WRITEPROTECT    (0)   // 1: enable card write protect, otherwise, don't use
    #define EN_CARD_PWRCTL          (0)   // 1: enable card power control, otherwise, don't use

    /* enable storage driver option */
    #define	USING_SD_IF	        (1)	        // 1:enable sd driver
    #define	USING_SM_IF	        (0)	        // 1:enable sm driver
    #define	USING_MEMORY_IF	    (0)	        // 1:enable sm driver
    #define	SM_2K_PAGE          (0)	        // 0: 512bytes flash(64MB below), 1: 2k bytes flash(128MB above)
    // It doesn't need to modify below
    #if ((USING_SM_IF&&USING_SD_IF) ||(USING_MEMORY_IF&&USING_SD_IF))
    #define USING_DUAL_IF       (1)
    #define MSDC_LUN_NUMBER     (2)
    #else
    #define USING_DUAL_IF       (0)
    #define MSDC_LUN_NUMBER     (1)
    #endif
    #define MSDC_RX_EP2         (1)         // 1: TX ep1 and RX ep2.
    /** @} */ // end of storage

/** @name MSDRM
@{ */
    #define MSDRM_WMA           (0) // 0: Disable WMA with MS-DRM, 1: Enable WMA with MS-DRM
    #define MSDRM_MTP			(0) // 0: Disable MTP with MS-DRM, 1: Enable MTP with MS-DRM
    /** @} */ //end of MSDRM

/** @name USB
@{ */
	#define USE_TASK_MODE_FOR_MSDC	(1)
	#define MSDC_WR_FLOW_TEST_EN	(0)
    #define SUPPORT_PCCAM_FUNC  (0) // 0: Disable PCCAM, 1: Enable PCCAM
    #define SUPPORT_MTP_FUNC    (0) // 0: Disable MTP, 1: Enable MTP
    #define SUPPORT_DPS_FUNC    (0) // 0: Disable DPS, 1: Enable DPS

	#if defined(ALL_FW)
		#define SYS_WOV_FUNC_EN			(0)
		#define SYS_SLEEP_FOR_WOV       (0)
		#define SYS_WD_ENABLE			(1) // 0: Disable CPU watch dog, 1: Enable
	   	#define SYS_SELF_SLEEP_ENABLE	(1) // 0: Disable System enter self-sleep (most power-saving) mode, 1: Enable
	   	#define SUPPORT_MSDC_FUNC   	(0)
	    #define SUPPORT_UVC_FUNC    	(1) // 0: Disable UVC, 1: Enable UVC
	    #define SUPPORT_PCSYNC_FUNC 	(0) // 0: Disable PCSync,1:Enable PCSync
	    #define USB_UVC_BULK_EP     	(1)
	    #define USB_UVC_SKYPE       	(1) // 0 : AIT H264 format, 1 : Skype format. (YUY2 + H264)
	    #define WOV_SLEPP_TRIGGER_LEVEL (0) // GPIO status: 0 -> active , 1 -> sleep.
	    #define E_PTZ_WIDTH             640
		#define E_PTZ_HEIGHT            480

	    #define USB_SUSPEND_LOOP_IN_ITCM    	(1)
	    #define DRAM_SELF_REFRESH   			(0) //sean@2010_08_19
	    #define USB_ENUM_AT_TASK_SUSPEND    	(1) // Start USB enum after tasks are suspend.
	    #define USB_MJPEGH264_STREAM 			(1) // Force MJPEG include H264.
	    #define USB_UVC_H264         			(1) // for Logitech H264
	    #define USB_UVC_1080P_EN     			(1)
	    #define USB_H264_USING_MOVEDMA  		(0)
	    #define USB_FRAMEBASE_H264_YUY2_STREAM	(1) // Frame Base H.264 with YUY2 stream
	    #define USB_UAC_TO_QUEUE        		(1)
		#define TEST_FDTC_RAWPATH               (1) // Test	use raw path to do FDTC.
	    #if (JTAG == 1)
	    	#define USB_SUSPEND_TEST    (0) // must be 0 for Linux
		#else
	    	#define USB_SUSPEND_TEST    (0) // must be 0 for Linux
		#endif

		#if (USB_UVC_BULK_EP == 0x1)
	    	#define UVC_DMA_SIZE   		(0x2000)
	    	#define UVC_DMA_3K_NUM      (0) // two 3K for one DMA...
	    #else
	    	#define UVC_DMA_3K_NUM      (1) // two 3K for one DMA...
	    #endif
    #endif //#if defined(ALL_FW)

    #if defined(UPDATER_FW)
    	#define SYS_WOV_FUNC_EN			(0)
    	#define SYS_WD_ENABLE			(1) // 0: Disable CPU watch dog, 1: Enable
	    #define SUPPORT_MSDC_FUNC   	(1) // 0: Disable MSDC, 1: Enable MSDC
	    #define SYS_SELF_SLEEP_ENABLE	(0) // 0: Disable System enter self-sleep (most power-saving) mode, 1: Enable
	    #define SUPPORT_UVC_FUNC    	(0) // 0: Disable UVC, 1: Enable UVC
	    #define SUPPORT_PCSYNC_FUNC 	(0) // 0: Disable PCSync,1:Enable PCSync
	    #define USB_SUSPEND_TEST    	(0) // must be 0 for Linux
	    #define USB_UVC_BULK_EP     	(0) // must be 0 for Linux
	    #define USB_UVC_SKYPE       	(0xFF) // 0 : AIT H264 format, 1 : Skype format. (YUY2 + H264)
	    #define UVC_DMA_SIZE   			(0)
		#define UVC_DMA_3K_NUM      	(0)

	    #define USB_SUSPEND_LOOP_IN_ITCM    	(0)
	    #define DRAM_SELF_REFRESH   			(0) //sean@2010_08_19
	    #define USB_ENUM_AT_TASK_SUSPEND    	(0) // Start USB enum after tasks are suspend.
	    #define USB_MJPEGH264_STREAM 			(0) // Force MJPEG include H264.
	    #define USB_UVC_H264         			(0) // for Logitech H264
	    #define USB_UVC_1080P_EN     			(0)
	    #define USB_H264_USING_MOVEDMA  		(0)
	    #define USB_FRAMEBASE_H264_YUY2_STREAM	(0) // Frame Base H.264 with YUY2 stream
	    #ifdef USB_UAC_TO_QUEUE
	    	#undef USB_UAC_TO_QUEUE
	    #endif
    #endif //#if defined(UPDATER_FW)


    #if defined(MBOOT_FW)
    #define SYS_WD_ENABLE			(1) // 0: Disable CPU watch dog, 1: Enable
    #define USB_UVC_SKYPE       	(0xFF) // 0 : AIT H264 format, 1 : Skype format. (YUY2 + H264)
    #define SYS_WOV_FUNC_EN			(0)
    #endif


    #define HDMI_OUTPUT_EN      (0)


    #define SIGMA_DELTA_TRACKING        (0)
    #define USING_STILL_METHOD_1    (1)  // 0: using still method 2, 1: using still method 1

	/** @} */ /* end of USB */

/** @name DSC
@{ */
    #define OMURON_FDTC_SUPPORT        (1) // 1: Enable face detection, 0:Disable face detection
    #define FDTC_SUPPORT        (0)
    #define EXIF_CARD_MODE_ENC 	(1) // 1: Support card mode EXIF encode, 0: Disable card mode EXIF encode
	#define EXIF_CARD_MODE_DEC 	(0) // 1: Support card mode EXIF decode, 0: Disable card mode EXIF decode
	#define EXIF_MAKER_CUS     "AIT Electronics Inc"
	#define EXIF_MODEL_CUS     "AIT-840D"
	#define EXIF_SOFTWARE_CUS  "AIT Corp"
	#define EXIF_ARTIST_CUS    "AIT Corp"
	#define EXIF_COPYRIGHT_CUS "AIT Corp"
	#define USE_TASK_DO3A       (1)

    #define SCALE_DNSAMP_2_MODE	(0) // 1: Enable scaler down sample 2 mode
    #define DSC_JPEG_DEBUG_FOR_ISP  (0)  //1: Write ISP register setting to JPEG file

    #define DSC_GRAB_CENTER     (0)
    #define DSC_SCALE_GRAB_IN_RAW_EN       (0)

    /** @} */ // end of DSC

/** @name AUDIO
@{ */
  #if (CHIP==MCR_V2)
  #define AUDIO_CODEC_DUPLEX_EN   	(1) // 1: allow ADC & DAC to work at the same time
  #endif
  
	#if (CHIP == MERCURY) || (CHIP == VSN_V3)
	#define	DEFAULT_DAC_DIGITAL_GAIN 		0x3F //per channel
	#define	DEFAULT_DAC_ANALOG_GAIN  		0xAA
	#define DEFAULT_DAC_LINEOUT_GAIN        0xCC
	#define	DEFAULT_ADC_DIGITAL_GAIN 		0x47 //per channel
	#define	DEFAULT_ADC_ANALOG_GAIN  		0x0B //20db
	#endif

    #if (CHIP == MCR_V2)

    
    #define	DEFAULT_DAC_DIGITAL_GAIN    (63 + 8*2 ) //+8 db per channel (0x00~0x57) 12db max
    //#define	DEFAULT_DAC_ANALOG_GAIN     (0x80) //-4db both channel (0x00~0xFF) 6db max
    #define DEFAULT_DAC_LINEOUT_GAIN    (0x00) //0 db one channel (0x00~0x3E)  0dB   ~ -45dB
    // TBD 
    #define	DEFAULT_ADC_DIGITAL_GAIN    (81) // 0x58 0db per channel (0x01~0xA1) -60dB ~  60dB
    #define	DEFAULT_ADC_ANALOG_GAIN    	(0x0c) //31db per channel (0x00~0x1F) 0dB   ~  31db max
    
    #define ADC_MIC_BIAS_NONE               (0)
    #define ADC_MIC_BIAS_0d65AVDD_MP        (0)
    #define ADC_MIC_BIAS_0d75AVDD_MP        (1)
    #define ADC_MIC_BIAS_0d85AVDD_MP        (2)
    #define ADC_MIC_BIAS_0d95AVDD_MP    (3)
    #define DEFAULT_ADC_MIC_BIAS_MP     ADC_MIC_BIAS_0d75AVDD_MP
    #endif

    #define MP3_LOW_POWER_EN    (0)
	#define MP3_POWER_SAVING_EN	(0)
	#define AUDIO_DEC_ENC_SHARE_WB	(1)
    #define OMA_DRM_EN          (0)
	#define	NOISE_REDUCTION_EN  (0)
	#define EXT_CODEC_SUPPORT   (1)

    #define BYPASS_FILETER_STAGE        (0)
    #define DOWN_SAMPLE_TIMES           (1)
    #define HIGH_SRATE_MODE             (DOWN_SAMPLE_TIMES)

	#define SBC_SUPPORT	        (0) // 0: Disable SBC encode, 1: Enable SBC encode
	#define SRC_SUPPORT	        (0) // 0: Disable SRC encode, 1: Enable SRC encode
	#define WAV_ENC_SUPPORT     (0)
	#define GAPLESS_EN          (0)
	#define AUDIO_STREAMING_EN	(0)
	#define AUDIO_MIXER_EN		(0) // 0: Disable software mixer 1: Enable software mixer
	#define PCM_TO_DNSE_EN		(0) // 0: Disable software mixer 1: Enable software mixer
	#define PCM_ENC_SUPPORT     (0)
    #define GAPLESS_EN          (0) // 0: Disable gapless playback, 1: Enable gapless playback
    #define AUDIO_CODEC_DUPLEX_EN (1)
    /** @} */ // end of AUDIO
    /** @name H264
    @{ */
    #define H264_DEC_SUPPORT    (0)	// mark by tomy
    /** @} */ // end of H264

    #define H264_IME_NUM       (0)

	#define SIF_BUFFER_START  		0x02F00000  //0x110000	//Temp Buf, size:0x1000

    #define SENSOR_SUPPORT_SCALER       	(0)
    #define SENSOR_SCALER_FOV_PATCH     	(1)
    #define SNR_SCAL_VIF_INIT_X         	(13)
    #define SNR_SCAL_VIF_INIT_Y         	(1)
    #define SNR_SCAL_VIF_INIT_WIDTH     	(1932)
    #define SNR_SCAL_VIF_INIT_HEIGHT    	(1092)
	#define SNR_SCAL_VIF_5M_WIDTH 			(2564)
	#define SNR_SCAL_VIF_5M_HEIGHT			(1924)
	#define SNR_SCAL_VIF_JPEGMODE_WIDTH		(1932)
	#define SNR_SCAL_VIF_JPEGMODE_HEIGHT	(1460)

    #define SNR_SCAL_INIT_WIDTH         (1920)  // for scaler calculation
    #define SNR_SCAL_INIT_HEIGHT        (1088)  // for scaler calculation
	#define DYN_GRA_PATH_BY_ZOOM    	(0)

	#define TEST_RAWPATH        		(0) // Enable the raw path preview
	#define INSERT_EXIF_EN 				1

	#if !defined(UPDATER_FW)
	#define FS_NEW_GET_FREE_SPACE 0
	#endif
	#define HW_MP3D_EN 0
	#define GAPLESS_PLAY_EN 0
	#define LGE_OAEP_EN 0
#if defined(ALL_FW)||defined(MBOOT_FW)||defined(UPDATER_FW)  // This part can be changed by difference project.
	#define FS_EN  0
	#define DSC_R_EN 0
	#define DSC_P_EN 0
	#define AAC_P_EN 0
	#define MP3_P_EN 0
	#define AAC_R_EN 0
	#define MP3_R_EN 0
	#define AMR_R_EN 0
	#define AMR_P_EN 0
	#define MIDI_EN 0
	#define WMA_EN 0
	#define AC3_P_EN 0
	#define VAMR_R_EN 0
	#define VAMR_P_EN 0
	#if defined(MBOOT_FW)||defined(UPDATER_FW)
		#define VAAC_R_EN 0
	#endif
	#if defined(ALL_FW)
		#define VAAC_R_EN 0
	#endif
	#define VAAC_P_EN 0
	#define VAC3_P_EN 0
	#define USB_EN 0
	#define OGG_EN 0
	#define WMAPRO10_EN 0
	#define RA_EN 0
	#define RV_EN 0
	#define VRA_P_EN 0
    #define WAV_P_EN 0
	#define VMP3_P_EN 0
	#define MP3HD_P_EN 0
	#define WAV_R_EN 0
	#define VMP3_P_EN 0
	#define VWAV_P_EN 0
	#define WMV_P_EN 0
	#define VWMA_P_EN 0
	#define FLAC_P_EN 0

	#define PCAM_EN ((USB_EN) && (SUPPORT_UVC_FUNC))

	#define AUDIO_EFFECT_EN     (0)

	#if (AAC_P_EN)||(MP3_P_EN)||(MP3HD_P_EN)||(AMR_P_EN)||(MIDI_EN)||(WMA_EN)||(OGG_EN)||(WMAPRO10_EN)||(RA_EN)||(WAV_P_EN)
		#define AUDIO_P_EN 1 // Use for some common audio play usage.
	#else
		#define AUDIO_P_EN 0
	#endif
	#if (AMR_R_EN)||(MP3_R_EN)||(AAC_R_EN)||(WAV_R_EN)
		#define AUDIO_R_EN 0 // Use for some common audio record usage.
	#else
		#define AUDIO_R_EN 0
	#endif
	#if (AAC_R_EN)||(VAAC_R_EN)
		#define AUDIO_AAC_R_EN 1 // Use for some common audio AAC record usage.
	#else
		#define AUDIO_AAC_R_EN 0
	#endif
	#if (AAC_P_EN)||(VAAC_P_EN)
		#define AUDIO_AAC_P_EN 1 // Use for some common audio AAC play usage.
	#else
		#define AUDIO_AAC_P_EN 0
	#endif
	#if (MP3_P_EN)||(VMP3_P_EN)
		#define AUDIO_MP3_P_EN 1
	#else
		#define AUDIO_MP3_P_EN 0
	#endif
	#if (AC3_P_EN)||(VAC3_P_EN)
		#define AUDIO_AC3_P_EN 1
	#else
	    #define AUDIO_AC3_P_EN 0
	#endif
	#if (AMR_R_EN)||(VAMR_R_EN)
		#define AUDIO_AMR_R_EN 1 // Use for some common audio AMR record usage.
	#else
		#define AUDIO_AMR_R_EN 0
	#endif
	#if (AMR_P_EN)||(VAMR_P_EN)
		#define AUDIO_AMR_P_EN 1 // Use for some common audio AMR play usage.
	#else
		#define AUDIO_AMR_P_EN 0
	#endif
	#if (VAMR_R_EN)||(VAAC_R_EN)
		#define VIDEO_R_EN 1 // Use for some common video record usage.
	#else
		#define VIDEO_R_EN 0
	#endif
	#if (VAMR_P_EN)||(VAAC_P_EN)||(VMP3_P_EN)
		#define VIDEO_P_EN 1 // Use for some common video play usage.
	#else
		#define VIDEO_P_EN 0
	#endif
	#if (VAMR_P_EN)||(VAAC_P_EN)||(DSC_P_EN)||(DSC_R_EN)||(VAAC_R_EN)||(VAMR_R_EN)
		#define TV_EN 0 // Use for some common tv function usage.
	#else
		#define TV_EN 0
	#endif
    #define APP_EN 0

	#define	SUPPORT_WATCHDOG	1 // charles 1 for watchdog enable, 0 for disable
#endif

#if defined(UPDATER_FW)
    #ifdef SUPPORT_MTP_FUNC
    	#undef SUPPORT_MTP_FUNC
    	#define SUPPORT_MTP_FUNC    0
    #endif

    #ifdef USB_EN
    	#undef USB_EN
    	#define USB_EN    1
    #else
    	#define USB_EN    1
    #endif
#endif

#if defined(MBOOT_FW)

    #ifdef SUPPORT_MTP_FUNC
    	#undef SUPPORT_MTP_FUNC
    	#define SUPPORT_MTP_FUNC    0
    #endif

    #ifdef USB_EN
    	#undef USB_EN
    	#define USB_EN    0
    #else
    	#define USB_EN    0
    #endif

    #ifdef TV_EN
    	#undef TV_EN
    	#define TV_EN    0
    #else
    	#define TV_EN    0
    #endif

#endif
#endif

#if (CHIP == MCR_V2)
//#define SUPPORT_TNR                   (1)
#endif
#if (CHIP == MERCURY) || (CHIP == VSN_V3)
//#define SUPPORT_TNR                   (0)
#endif

/* Flow Control Config */
#define MCR_IBC_MIRROR_WA           (1) //Mercury IBC mirror workaround

#if (CHIP == MERCURY)
#define PIPE3_ENABLE				(1)
#define SCALER_ISR_EN				(1)
#endif
#if (CHIP == VSN_V3) || (CHIP == MCR_V2)
#define PIPE3_ENABLE				(0)
#define SCALER_ISR_EN				(0)
#endif



///@end_ait_only
