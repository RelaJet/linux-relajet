//==============================================================================
//
//  File        : mmp_err.h
//  Description : Top level system error definition.
//  Author      : Penguin Torng
//  Revision    : 1.0
//
//==============================================================================
/**
 @file mmp_err.h
 @brief The header file of MMP error codes

 This is a common file used in firmware and the host side, it describle the error codes that shared between
 firmware and host side

 @author Penguin Torng
 @version 1.0 Original Version
*/



#ifndef _MMP_ERR_H_
#define _MMP_ERR_H_

//==============================================================================
//
//                              COMPILER OPTION
//
//==============================================================================

//==============================================================================
//
//                              CONSTANTS
//
//==============================================================================
#define	MODULE_ERR_SHIFT		24

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================
typedef enum _MMP_MODULE
{
    MMP_HIF = 0,
    MMP_SYSTEM = 1,
    MMP_SENSOR = 2,
    MMP_VIF = 3,
    MMP_ISP = 4,
    MMP_SCALER = 5,
    MMP_ICON = 6,
    MMP_IBC = 7,
	MMP_GRA = 8,
	MMP_INT = 9,
	MMP_DSPY = 10,
	MMP_DRAM = 11,
	MMP_I2CM = 12,
	MMP_PSPI = 13,
	MMP_DMA = 14,
	MMP_SD = 15,
	MMP_NAND = 16,
	MMP_VIDE = 17,
	MMP_MP4VD = 18,
	MMP_H264D = 19,
	MMP_USB = 20,
	MMP_FS = 21,
	MMP_3GPMGR = 22,
	MMP_3GPPSR = 23,
	MMP_AUDIO = 24,
	MMP_DSC = 25,
	MMP_FCTL = 26,
    MMP_VCONF = 27,
    MMP_3GPPLAY = 28,
    MMP_3GPRECD = 29,
    MMP_UART = 30,
    MMP_MDTV = 31,
    MMP_SPI = 32,
    MMP_PLL = 33,
    MMP_USER = 34,
    MMP_VT = 35,
    MMP_CCIR = 36,
    MMP_STORAGE = 37,
    MMP_PIO = 38,
    MMP_MFD = 39,
   	MMP_SF = 40,
   	MMP_JPEG = 41,
   	MMP_VIDBUF = 42,
   	MMP_PWM		= 43
} MMP_MODULE;

typedef enum _MMP_ERR
{
	MMP_ERR_NONE = 0x00000000,

    // MMP_HIF
    MMP_HIF_ERR_PARAMETER = (MMP_HIF << MODULE_ERR_SHIFT) | 0x000001,
	MMP_HIF_ERR_MODE_NOT_SET,

    // MMP_SYSTEM
    MMP_SYSTEM_ERR_PARAMETER = (MMP_SYSTEM << MODULE_ERR_SHIFT) | 0x000001,
    MMP_SYSTEM_ERR_CMDTIMEOUT,
	MMP_SYSTEM_ERR_HW,
	MMP_SYSTEM_ERR_CPUBOOT,
    MMP_SYSTEM_ERR_SETOPMODE,
    MMP_SYSTEM_ERR_NOT_IMPLEMENTED,
	MMP_SYSTEM_ERR_SETAPMODE,
	MMP_SYSTEM_ERR_GET_FW,
	MMP_SYSTEM_ERR_SETPSMODE,
	MMP_SYSTEM_ERR_VERIFY_FW,
	MMP_SYSTEM_ERR_SETPLL,
	MMP_SYSTEM_ERR_REGISTER_TEST_FAIL,
	MMP_SYSTEM_ERR_MEMORY_TEST_FAIL,
    MMP_SYSTEM_ERR_NOT_SUPPORT,
    MMP_SYSTEM_ERR_MALLOC,
	MMP_SYSTEM_ERR_FORMAT,
	MMP_SYSTEM_ERR_TIMER,

    // MMP_SENSOR
    MMP_SENSOR_ERR_PARAMETER = (MMP_SENSOR << MODULE_ERR_SHIFT) | 0x000001,
	MMP_SENSOR_ERR_INITIALIZE,
	MMP_SENSOR_ERR_INITIALIZE_NONE,
	MMP_SENSOR_ERR_FDTC,
	MMP_SENSOR_ERR_SETMODE,
	MMP_SENSOR_ERR_AF_MISS,

    // MMP_VIF
    MMP_VIF_ERR_PARAMETER = (MMP_VIF << MODULE_ERR_SHIFT) | 0x000001,

    // MMP_ISP
    MMP_ISP_ERR_PARAMETER = (MMP_ISP << MODULE_ERR_SHIFT) | 0x000001,

    // MMP_SCALER
    MMP_SCALER_ERR_PARAMETER = (MMP_SCALER << MODULE_ERR_SHIFT) | 0x000001,
    MMP_SCALER_ERR_CMDTIMEOUT,

    // MMP_ICON
    MMP_ICON_ERR_PARAMETER = (MMP_ICON << MODULE_ERR_SHIFT) | 0x000001,
	MMP_ICON_ERR_CMDTIMEOUT,

	// MMP_IBC
    MMP_IBC_ERR_PARAMETER = (MMP_IBC << MODULE_ERR_SHIFT) | 0x000001,
    MMP_IBC_ERR_CMDTIMEOUT,

	// MMP_GRA
    MMP_GRA_ERR_PARAMETER = (MMP_GRA << MODULE_ERR_SHIFT) | 0x000001,
    MMP_GRA_ERR_CMDTIMEOUT,
    MMP_GRA_ERR_HW,
    MMP_GRA_ERR_NOT_IMPLEMENT,
    MMP_GRA_ERR_BUSY,

	// MMP_INT
    MMP_INT_ERR_PARAMETER = (MMP_INT << MODULE_ERR_SHIFT) | 0x000001,

	// MMP_DISPLAY
    MMP_DISPLAY_ERR_PARAMETER = (MMP_DSPY << MODULE_ERR_SHIFT) | 0x000001,
    MMP_DISPLAY_ERR_PRM_NOT_INITIALIZE,
    MMP_DISPLAY_ERR_SCD_NOT_INITIALIZE,
    MMP_DISPLAY_ERR_NON_CONTROLLER_ENABLE,
    MMP_DISPLAY_ERR_NOT_SUPPORT,
    MMP_DISPLAY_ERR_HW,
    MMP_DISPLAY_ERR_OVERRANGE,
    MMP_DISPLAY_ERR_NOT_IMPLEMENTED,
	MMP_DISPLAY_ERR_INSUFFICIENT_OSDMEMORY,
	MMP_DISPLAY_ERR_RGBLCD_NOT_ENABLED,
	MMP_DISPLAY_ERR_LCD_BUSY,
	MMP_DISPLAY_ERR_FRAME_END,

	// MMP_DRAM
    MMP_DRAM_ERR_PARAMETER = (MMP_DRAM << MODULE_ERR_SHIFT) | 0x000001,
    MMP_DRAM_ERR_INITIALIZE,
    MMP_DRAM_ERR_NOT_SUPPORT,

	// MMP_I2CM
    MMP_I2CM_ERR_PARAMETER = (MMP_I2CM << MODULE_ERR_SHIFT) | 0x000001,
	MMP_I2CM_ERR_SLAVE_NO_ACK,
	MMP_I2CM_ERR_READ_TIMEOUT, //yidongq
	MMP_I2CM_ERR_NO_RESOURCE,

	// MMP_PSPI
    MMP_PSPI_ERR_PARAMETER = (MMP_PSPI << MODULE_ERR_SHIFT) | 0x000001,
    // MMP_DMA
    MMP_DMA_ERR_PARAMETER = (MMP_DMA << MODULE_ERR_SHIFT) | 0x000001,
	MMP_DMA_ERR_OTHER,
	MMP_DMA_ERR_NOT_SUPPORT,
	MMP_DMA_ERR_BUSY,

    // MMP_SD
    MMP_SD_ERR_COMMAND_FAILED = (MMP_SD << MODULE_ERR_SHIFT) | 0x000001,
    MMP_SD_ERR_RESET,
    MMP_SD_ERR_PARAMETER,
    MMP_SD_ERR_DATA,
    MMP_SD_ERR_NO_CMD,
    MMP_SD_ERR_BUSY,
    MMP_SD_ERR_CARD_REMOVED,
    // MMP_NAND
    MMP_NAND_ERR_PARAMETER = (MMP_NAND << MODULE_ERR_SHIFT) | 0x000001,
    MMP_NAND_ERR_RESET,
    MMP_NAND_ERR_HW_INT_TO,
    MMP_NAND_ERR_ECC,
    MMP_NAND_ERR_NOT_COMPLETE,
    MMP_NAND_ERR_ERASE,
    MMP_NAND_ERR_PROGRAM,
    MMP_NAND_ERR_ECC_CORRECTABLE,

	// MMP_VIDENC
    MMP_VIDE_ERR_PARAMETER = (MMP_VIDE << MODULE_ERR_SHIFT) | 0x000001,
	MMP_VIDE_ERR_NOT_SUPPORTED_BITSTREAM,
	MMP_VIDE_ERR_FRM_WORKBUF_SET_FULL,
	MMP_VIDE_ERR_FRM_WORKBUF_SET_OVERFLOW,
	MMP_VIDE_ERR_FRM_WORKBUF_SET_UNDERFLOW,
	MMP_VIDE_ERR_INVAL_PARAM,
	MMP_VIDE_ERR_INVAL_OP,

	// MMP_MP4VD
    MMP_MP4VD_ERR_BASE = MMP_MP4VD << MODULE_ERR_SHIFT, // 0x12
	    /** One or more parameters were not valid.
	    The input parameters are supported but are not valid value. E.g. it's out of range.*/
        MMP_MP4VD_ERR_PARAMETER = MMP_MP4VD_ERR_BASE | 0x1005,
        /** The buffer was emptied before the next buffer was ready */
        MMP_MP4VD_ERR_UNDERFLOW = MMP_MP4VD_ERR_BASE | 0x1007,
        /** The buffer was not available when it was needed */
        MMP_MP4VD_ERR_OVERFLOW = MMP_MP4VD_ERR_BASE | 0x1008,
        /** The hardware failed to respond as expected */
        MMP_MP4VD_ERR_HW = MMP_MP4VD_ERR_BASE | 0x1009,
        /** Stream is found to be corrupt */
        MMP_MP4VD_ERR_STREAM_CORRUPT = MMP_MP4VD_ERR_BASE | 0x100B,
        /** The component is not ready to return data at this time */
        MMP_MP4VD_ERR_NOT_READY = MMP_MP4VD_ERR_BASE | 0x1010,
        /** There was a timeout that occurred */
        MMP_MP4VD_ERR_TIME_OUT = MMP_MP4VD_ERR_BASE | 0x1011,
        /** Attempting a command that is not allowed during the present state. */
        MMP_MP4VD_ERR_INCORRECT_STATE_OPERATION = MMP_MP4VD_ERR_BASE | 0x1018,
        /** The values encapsulated in the parameter or config structure are not supported. */
        MMP_MP4VD_ERR_UNSUPPORTED_SETTING = MMP_MP4VD_ERR_BASE | 0x1019,

	// MMP_H264D
    MMP_H264D_ERR_BASE = MMP_H264D << MODULE_ERR_SHIFT, // 0x13
	    /** There were insufficient resources to perform the requested operation
	    E.g. The bitstream buffer is overflow. Since the video bitstream buffer is a
	    plain buffer, the buffer is not able to be full loaded when one video
	    frame is greater than the buffer.  For the size of the video bitstream
	    buffer, refer AIT for more detail.*/
        MMP_H264D_ERR_INSUFFICIENT_RESOURCES = MMP_H264D_ERR_BASE | 0x1000,
	    /** One or more parameters were not valid.
	    The input parameters are supported but are not valid value. E.g. it's out of range.*/
        MMP_H264D_ERR_PARAMETER = MMP_H264D_ERR_BASE | 0x1005,
        /** The buffer was emptied before the next buffer was ready */
        MMP_H264D_ERR_UNDERFLOW = MMP_H264D_ERR_BASE | 0x1007,
        /** The buffer was not available when it was needed */
        MMP_H264D_ERR_OVERFLOW = MMP_H264D_ERR_BASE | 0x1008,
        /** The hardware failed to respond as expected */
        MMP_H264D_ERR_HW = MMP_H264D_ERR_BASE | 0x1009,
        /** The component is in the state MMP_STATE_INVALID */
        MMP_H264D_ERR_INVALID_STATE = MMP_H264D_ERR_BASE | 0x100A,
        /** Stream is found to be corrupt */
        MMP_H264D_ERR_STREAM_CORRUPT = MMP_H264D_ERR_BASE | 0x100B,
        /** The component is not ready to return data at this time */
        MMP_H264D_ERR_NOT_READY = MMP_H264D_ERR_BASE | 0x1010,
        /** There was a timeout that occurred */
        MMP_H264D_ERR_TIME_OUT = MMP_H264D_ERR_BASE | 0x1011,
        /** Attempting a command that is not allowed during the present state. */
        MMP_H264D_ERR_INCORRECT_STATE_OPERATION = MMP_H264D_ERR_BASE | 0x1018,
        /** The values encapsulated in the parameter or config structure are not supported. */
        MMP_H264D_ERR_UNSUPPORTED_SETTING = MMP_H264D_ERR_BASE | 0x1019,
    MMP_H264D_ERR_MEM_UNAVAILABLE,//MMP_H264D_ERR_INSUFFICIENT_RESOURCES
    MMP_H264D_ERR_INIT_VIDEO_FAIL,//MMP_H264D_ERR_INVALID_STATE
    MMP_H264D_ERR_FRAME_NOT_READY,//MMP_H264D_ERR_NOT_READY

    // MMP_USB
    MMP_USB_ERR_PARAMETER = (MMP_USB << MODULE_ERR_SHIFT) | 0x000001,
    MMP_USB_ERR_PCSYNC_BUSY,
    MMP_USB_ERR_MEMDEV_ACK_TIMEOUT,
    MMP_USB_ERR_MEMDEV_NACK,
    MMP_USB_ERR_UNSUPPORT_MODE,

    // MMP_FS
    MMP_FS_ERR_PARAMETER = (MMP_FS << MODULE_ERR_SHIFT) | 0x000001,
    MMP_FS_ERR_TARGET_NOT_FOUND,
    MMP_FS_ERR_OPEN_FAIL,
    MMP_FS_ERR_CLOSE_FAIL,
    MMP_FS_ERR_READ_FAIL,
    MMP_FS_ERR_WRITE_FAIL,
    MMP_FS_ERR_FILE_SEEK_FAIL,
    MMP_FS_ERR_FILE_POS_ERROR,
    MMP_FS_ERR_FILE_COPY_FAIL,
    MMP_FS_ERR_FILE_ATTR_FAIL,
    MMP_FS_ERR_FILE_TIME_FAIL,
    MMP_FS_ERR_FILE_NAME_FAIL,
    MMP_FS_ERR_FILE_MOVE_FAIL,
    MMP_FS_ERR_FILE_REMOVE_FAIL,
    MMP_FS_ERR_FILE_REMNAME_FAIL,
    MMP_FS_ERR_INVALID_SIZE,
    MMP_FS_ERR_FILE_TRUNCATE_FAIL,
    MMP_FS_ERR_EXCEED_MAX_OPENED_NUM,
    MMP_FS_ERR_NO_MORE_ENTRY,
    MMP_FS_ERR_CREATE_DIR_FAIL,
    MMP_FS_ERR_DELETE_FAIL,
    MMP_FS_ERR_FILE_INIT_FAIL,
    MMP_FS_ERR_PATH_NOT_FOUND,
    MMP_FS_ERR_RESET_STORAGE,
    MMP_FS_ERR_EOF,
    MMP_FS_ERR_FILE_EXIST,
    MMP_FS_ERR_DIR_EXIST,
    MMP_FS_ERR_SEMAPHORE_FAIL,
    MMP_FS_ERR_NOT_SUPPORT,
    MMP_FS_ERR_GET_FREE_SPACE_FAIL,
    MMP_FS_ERR_IO_WRITE_FAIL,
    MMP_FS_ERR_CHECKRDY_TIMEOUT,

    // MMP_3GPMGR
    MMP_3GPMGR_ERR_PARAMETER = (MMP_3GPMGR << MODULE_ERR_SHIFT) | 0x000001,
	MMP_3GPMGR_ERR_HOST_CANCEL_SAVE,
    MMP_3GPMGR_ERR_MEDIA_FILE_FULL,
    MMP_3GPMGR_ERR_AVBUF_FULL,
    MMP_3GPMGR_ERR_AVBUF_EMPTY,
    MMP_3GPMGR_ERR_AVBUF_OVERFLOW,
    MMP_3GPMGR_ERR_AVBUF_UNDERFLOW,
    MMP_3GPMGR_ERR_AVBUF_FAILURE,
    MMP_3GPMGR_ERR_FTABLE_FULL,
    MMP_3GPMGR_ERR_FTABLE_EMPTY,
    MMP_3GPMGR_ERR_FTABLE_OVERFLOW,
    MMP_3GPMGR_ERR_FTABLE_FAILURE,
    MMP_3GPMGR_ERR_TTABLE_FULL,
    MMP_3GPMGR_ERR_TTABLE_EMPTY,
    MMP_3GPMGR_ERR_TTABLE_OVERFLOW,
    MMP_3GPMGR_ERR_TTABLE_FAILURE,
    MMP_VIDMGR_ERR_DESCQ_UNDERFLOW,
    MMP_VIDMGR_ERR_DESCQ_OVERFLOW,
    MMP_VIDMGR_ERR_DMA_TIMEOUT,

	// MMP_3GPPSR
    MMP_VIDPSR_ERR_BASE = (MMP_3GPPSR << MODULE_ERR_SHIFT), // 0x17
        //----- The followings ( < 0x1000) are not error -----
        /** End of stream */
        MMP_VIDPSR_ERR_EOS = MMP_VIDPSR_ERR_BASE | 0x0001,
        /** The max correct number of the error code*/
        MMP_VIDPSR_ERR_MIN = MMP_VIDPSR_ERR_BASE | 0x0FFF,
	    /** There were insufficient resources to perform the requested operation
	    E.g. The bitstream buffer is overflow. Since the video bitstream buffer is a
	    plain buffer, the buffer is not able to be full loaded when one video
	    frame is greater than the buffer.  For the size of the video bitstream
	    buffer, refer AIT for more detail.*/
        MMP_VIDPSR_ERR_INSUFFICIENT_RESOURCES = MMP_VIDPSR_ERR_BASE | 0x1000,
        /** There was an error, but the cause of the error could not be determined */
	    MMP_VIDPSR_ERR_UNDEFINED = MMP_VIDPSR_ERR_BASE | 0x1001,
	    /** One or more parameters were not valid.
	    The input parameters are supported but are not valid value. E.g. it's out of range.*/
        MMP_VIDPSR_ERR_PARAMETER = MMP_VIDPSR_ERR_BASE | 0x1005,
	    /** This functions has not been implemented yet.*/
	    MMP_VIDPSR_ERR_NOT_IMPLEMENTED = MMP_VIDPSR_ERR_BASE | 0x1006,
        /** The buffer was emptied before the next buffer was ready */
        MMP_VIDPSR_ERR_UNDERFLOW = MMP_VIDPSR_ERR_BASE | 0x1007,
        /** The buffer was not available when it was needed */
        MMP_VIDPSR_ERR_OVERFLOW = MMP_VIDPSR_ERR_BASE | 0x1008,
        /** The component is in the state MMP_STATE_INVALID */
        MMP_VIDPSR_ERR_INVALID_STATE = MMP_VIDPSR_ERR_BASE | 0x100A,
        /** There was a timeout that occurred */
        MMP_VIDPSR_ERR_TIME_OUT = MMP_VIDPSR_ERR_BASE | 0x1011,
        /** Resources allocated to an executing or paused component have been
          preempted, causing the component to return to the idle state */
        MMP_VIDPSR_ERR_RESOURCES_PREEMPTED = MMP_VIDPSR_ERR_BASE | 0x1013,
        /** Attempting a state transtion that is not allowed.
        The video player encounter an invalid state transition.
	    Such as try to PLAY while it is playing.*/
        MMP_VIDPSR_ERR_INCORRECT_STATE_TRANSITION = MMP_VIDPSR_ERR_BASE | 0x1017,
        /** Attempting a command that is not allowed during the present state. */
        MMP_VIDPSR_ERR_INCORRECT_STATE_OPERATION = MMP_VIDPSR_ERR_BASE | 0x1018,
        /** The values encapsulated in the parameter or config structure are not supported. */
        MMP_VIDPSR_ERR_UNSUPPORTED_SETTING = MMP_VIDPSR_ERR_BASE | 0x1019,
        /** A component reports this error when it cannot parse or determine the format of an input stream. */
        MMP_VIDPSR_ERR_FORMAT_NOT_DETECTED = MMP_VIDPSR_ERR_BASE | 0x1020,
        /** The content open operation failed. */
        MMP_VIDPSR_ERR_CONTENT_PIPE_OPEN_FAILED = MMP_VIDPSR_ERR_BASE | 0x1021,
        MMP_VIDPSR_ERR_CONTENT_CORRUPT          = MMP_VIDPSR_ERR_BASE | 0x1022,

	// MMP_AUDIO
    MMP_AUDIO_ERR_PARAMETER = (MMP_AUDIO << MODULE_ERR_SHIFT) | 0x000001,
    MMP_AUDIO_ERR_END_OF_FILE,
    MMP_AUDIO_ERR_STREAM_UNDERFLOW,
    MMP_AUDIO_ERR_STREAM_BUF_FULL,
    MMP_AUDIO_ERR_STREAM_BUF_EMPTY,
    MMP_AUDIO_ERR_STREAM_OVERFLOW,
    MMP_AUDIO_ERR_STREAM_POINTER,
    MMP_AUDIO_ERR_COMMAND_INVALID,
    MMP_AUDIO_ERR_OPENFILE_FAIL,
    MMP_AUDIO_ERR_FILE_CLOSED,
    MMP_AUDIO_ERR_FILE_IO_FAIL,
    MMP_AUDIO_ERR_INSUFFICIENT_BUF,
    MMP_AUDIO_ERR_UNSUPPORT_FORMAT,
    MMP_AUDIO_ERR_NO_AUDIO_FOUND,
    MMP_AUDIO_ERR_INVALID_EQ,
    MMP_AUDIO_ERR_INVALID_FLOW,
    MMP_AUDIO_ERR_DATABASE_SORT,
    MMP_AUDIO_ERR_DATABASE_FLOW,
    MMP_AUDIO_ERR_DATABASE_MEMORY_FULL,
    MMP_AUDIO_ERR_DATABASE_NOT_FOUND,
    MMP_AUDIO_ERR_DATABASE_NOT_SUPPORT,
    MMP_AUDIO_ERR_NO_MIXER_DATA,
    MMP_AUDIO_ERR_BUF_ALLOCATION,
    MMP_AUDIO_ERR_DECODER_INIT,
    MMP_AUDIO_ERR_SEEK,
    MMP_AUDIO_ERR_DECODE,

	// MMP_DSC
    MMP_DSC_ERR_PARAMETER = (MMP_DSC << MODULE_ERR_SHIFT) | 0x000001,
    MMP_DSC_ERR_SAVEOF_FAIL,
    MMP_DSC_ERR_JPEG_NODATASAVE,
    MMP_DSC_ERR_FILE_END,
    MMP_DSC_ERR_FILE_ERROR,
    MMP_DSC_ERR_DECODE_FAIL,
    MMP_DSC_ERR_FLOW_FAIL,
    MMP_DSC_ERR_HW,
    MMP_DSC_ERR_JPEGINFO_FAIL,
    MMP_DSC_ERR_CAPTURE_FAIL,
    MMP_DSC_ERR_FIFOOF_FAIL,
    MMP_DSC_ERR_OUTOFRANGE,
    MMP_DSC_ERR_INITIALIZE_FAIL,
    MMP_DSC_ERR_PREVIEW_FAIL,
    MMP_DSC_ERR_PLAYBACK_FAIL,
    MMP_DSC_ERR_ICON_FAIL,
    MMP_DSC_ERR_TIMEOUT_FAIL,
	MMP_DSC_ERR_EXIF_ENC,
	MMP_DSC_ERR_EXIF_DEC,
	MMP_DSC_ERR_EXIF_NOT_SUPPORT,

	// MMP_FCTL
    MMP_FCTL_ERR_PARAMETER = (MMP_FCTL << MODULE_ERR_SHIFT) | 0x000001,
    MMP_FCTL_ERR_HW,

	// MMP_VCONF
    MMP_VCONF_ERR_PARAMETER = (MMP_VCONF << MODULE_ERR_SHIFT) | 0x000001,

	// MMP_3GPPLAY
    MMP_3GPPLAY_ERR_BASE = (MMP_3GPPLAY << MODULE_ERR_SHIFT), // 0x1C
	    /** There were insufficient resources to perform the requested operation
	    E.g. The bitstream buffer is overflow. Since the video bitstream buffer is a
	    plain buffer, the buffer is not able to be full loaded when one video
	    frame is greater than the buffer.  For the size of the video bitstream
	    buffer, refer AIT for more detail.*/
        MMP_3GPPLAY_ERR_INSUFFICIENT_RESOURCES = MMP_3GPPLAY_ERR_BASE | 0x1000,
        /** There was an error, but the cause of the error could not be determined */
	    MMP_3GPPLAY_ERR_UNDEFINED = MMP_3GPPLAY_ERR_BASE | 0x1001,
	    /** One or more parameters were not valid.
	    The input parameters are supported but are not valid value. E.g. it's out of range.*/
        MMP_3GPPLAY_ERR_PARAMETER = MMP_3GPPLAY_ERR_BASE | 0x1005,
	    /** This functions has not been implemented yet.*/
	    MMP_3GPPLAY_ERR_NOT_IMPLEMENTED = MMP_3GPPLAY_ERR_BASE | 0x1006,
        /** The buffer was emptied before the next buffer was ready */
        MMP_3GPPLAY_ERR_UNDERFLOW = MMP_3GPPLAY_ERR_BASE | 0x1007,
        /** The buffer was not available when it was needed */
        MMP_3GPPLAY_ERR_OVERFLOW = MMP_3GPPLAY_ERR_BASE | 0x1008,
        /** The hardware failed to respond as expected */
        MMP_3GPPLAY_ERR_HW = MMP_3GPPLAY_ERR_BASE | 0x1009,
        /** The component is in the state MMP_STATE_INVALID */
        MMP_3GPPLAY_ERR_INVALID_STATE = MMP_3GPPLAY_ERR_BASE | 0x100A,
        /** The component is not ready to return data at this time */
        MMP_3GPPLAY_ERR_NOT_READY = MMP_3GPPLAY_ERR_BASE | 0x1010,
        /** There was a timeout that occurred */
        MMP_3GPPLAY_ERR_TIME_OUT = MMP_3GPPLAY_ERR_BASE | 0x1011,
        /** Attempting a state transtion that is not allowed.
        The video player encounter an invalid state transition.
	    Such as try to PLAY while it is playing.*/
        MMP_3GPPLAY_ERR_INCORRECT_STATE_TRANSITION = MMP_3GPPLAY_ERR_BASE | 0x1017,
        /** Attempting a command that is not allowed during the present state. */
        MMP_3GPPLAY_ERR_INCORRECT_STATE_OPERATION = MMP_3GPPLAY_ERR_BASE | 0x1018,
        /** The values encapsulated in the parameter or config structure are not supported. */
        MMP_3GPPLAY_ERR_UNSUPPORTED_SETTING = MMP_3GPPLAY_ERR_BASE | 0x1019,

	// MMP_3GPRECD
    MMP_3GPRECD_ERR_PARAMETER = (MMP_3GPRECD << MODULE_ERR_SHIFT) | 0x000001,
	MMP_3GPRECD_ERR_UNSUPPORTED_PARAMETERS, 	///< The input parameters are not supported.
	MMP_3GPRECD_ERR_INVALID_PARAMETERS,     	///< The input parameters are supported but are not valid value. E.g. it's out of range.
	MMP_3GPRECD_ERR_GENERAL_ERROR,          	///< General Error without specific define.
	MMP_3GPRECD_ERR_NOT_ENOUGH_SPACE,		 	///< Not enough space for minimum recording.
	MMP_3GPRECD_ERR_OPEN_FILE_FAILURE,		 	///< Open file failed.
	MMP_3GPRECD_ERR_CLOSE_FILE_FAILURE,	 		///< Close file failed.
	MMP_3GPRECD_ERR_BUFFER_OVERFLOW,			///< Buffer overflow
	MMP_3GPRECD_ERR_WRITE_FAILURE,				///< Host FWrite failure

    //MDTV
    MMP_MDTV_ERR_FAIL = (MMP_MDTV << MODULE_ERR_SHIFT) | 0x000001,
    MMP_MDTV_ERR_MEM_UNAVAILABLE,
    MMP_MDTV_ERR_DATA_UNAVAILABLE,
    MMP_MDTV_ERR_INIT_VIDEO_FAIL,
    MMP_MDTV_ERR_PLAY_SERVICE_FAIL,
    MMP_MDTV_ERR_STOP_SERVICE_FAIL,
    MMP_MDTV_ERR_SET_FREQUENCY_FAIL,
    MMP_MDTV_ERR_INIT_ESG_FAIL,
    MMP_MDTV_ERR_GET_SERVICE_NUM_FAIL,
    MMP_MDTV_ERR_GET_SERVICE_INFO_FAIL,
    MMP_MDTV_ERR_GET_PROGRAM_NUM_FAIL,
    MMP_MDTV_ERR_GET_PROGRAM_INFO_FAIL,
    MMP_MDTV_ERR_START_UPDATE_ESG_FAIL,
    MMP_MDTV_ERR_STOP_UPDATE_ESG_FAIL,
    MMP_MDTV_ERR_FLUSH_ESG_FAIL,
    MMP_MDTV_ERR_NO_SUPPORT_CA_SERVICE,

	// MMP_UART
    MMP_UART_ERR_PARAMETER = (MMP_UART << MODULE_ERR_SHIFT) | 0x000001,
    MMP_UART_SYSTEM_ERR,

    // SPI
    MMP_SPI_ERR_PARAMETER = (MMP_SPI << MODULE_ERR_SHIFT) | 0x000001,
    MMP_SPI_ERR_INIT,
    MMP_SPI_ERR_CMDTIMEOUT,
    MMP_SPI_ERR_TX_UNDERFLOW,
    MMP_SPI_ERR_RX_OVERFLOW,

	// MMP_PLL
	MMP_PLL_ERR_PARAMETER = (MMP_PLL << MODULE_ERR_SHIFT) | 0x000001,

	// MMP_USER
	MMP_USER_ERR_PARAMETER = (MMP_USER << MODULE_ERR_SHIFT) | 0x000001,
	MMP_USER_ERR_UNSUPPORTED,
	MMP_USER_ERR_INSUFFICIENT_BUF,
	MMP_USER_ERR_INIT,
	MMP_USER_ERR_UNINIT,

    // MMP_3GPRECD
    MMP_VT_ERR_PARAMETER = (MMP_VT << MODULE_ERR_SHIFT) | 0x000001,
	MMP_VT_ERR_UNSUPPORTED_PARAMETERS, 	        ///< The input parameters are not supported.
	MMP_VT_ERR_INVALID_PARAMETERS,     	        ///< The input parameters are supported but are not valid value. E.g. it's out of range.
	MMP_VT_ERR_GENERAL_ERROR,          	        ///< General Error without specific define.
	MMP_VT_ERR_INSUFFICIENT_BUF,                ///< No enough buffer for usage.
	MMP_VT_ERR_NOT_IMPLEMENTED,                 ///< Not implement functions.
    MMP_VT_ERR_TIME_OUT,                        ///< time out error

    // MMP_CCIR
    MMP_CCIR_ERR_PARAMETER = (MMP_CCIR << MODULE_ERR_SHIFT) | 0x000001,
    MMP_CCIR_ERR_UNSUPPORTED_PARAMETERS,
    MMP_CCIR_ERR_NOT_INIT,

    // MMP_STORAGE
    MMP_STORAGE_ERR_PARAMETER = (MMP_STORAGE << MODULE_ERR_SHIFT) | 0x000001,
    MMP_STORAGE_ERR_NOT_FOUND,
    MMP_STORAGE_ERR_PARTITION_INFO,
    MMP_STORAGE_ERR_IO_ACCESS,

     // MMP_PIO
    MMP_PIO_ERR_PARAMETER = (MMP_PIO << MODULE_ERR_SHIFT) | 0x000001,
    MMP_PIO_ERR_INPUTMODESETDATA,
    MMP_PIO_ERR_OUTPUTMODEGETDATA,
    MMP_PIO_ERR_SEMAPHORE_FAIL,

    //MMP_MFD
    MMP_MFD_ERR_PARAMETER = (MMP_MFD << MODULE_ERR_SHIFT) | 0x000001,
    MMP_MFD_ERR_INIT_FAILED,

    // MMP_PWM
    MMP_PWM_ERR_PARAMETER = (MMP_PWM << MODULE_ERR_SHIFT) | 0x000001,

    // MMP_SF
    MMP_SF_ERR_PARAMETER = (MMP_SF << MODULE_ERR_SHIFT) | 0x000001,
    MMP_SD_ERR_READ_ID,
    MMP_SF_ERR_CRC_ERROR,

    // MMP_JPEG
    MMP_JPEG_ERR_PARAMETER = (MMP_JPEG << MODULE_ERR_SHIFT) | 0x000001,
    MMP_JPEG_ERR_INIT,
    MMP_JPEG_ERR_LOCK,
    MMP_JPEG_ERR_INVAL_PARAM,
    MMP_JPEG_ERR_INVAL_OP,

    // MMP_VIDBUF
    MMP_VIDBUF_ERR_PARAMETER = (MMP_VIDBUF << MODULE_ERR_SHIFT) | 0x000001,
    MMP_VIDBUF_ERR_PARAM,
    MMP_VIDBUF_ERR_INIT

} MMP_ERR;

//==============================================================================
//
//                              MACRO FUNCTIONS
//
//==============================================================================
#endif // _MMP_ERR_H_

