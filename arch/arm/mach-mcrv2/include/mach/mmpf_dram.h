#ifndef _MMPF_DRAM_H
#define _MMPF_DRAM_H
/// @ait_only
/** @addtogroup MMPF_System
 *  @{
 */
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


//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================
#if (CHIP == MCR_V2)
#define AUTO_DLL_LOCK   (1)

#define DRAM_DDR       (0)
#define DRAM_DDR3      (1)
#ifndef DRAM_ID
    #define DRAM_ID    (DRAM_DDR)
#endif

#endif

typedef enum _MMPF_DRAM_TYPE
{
    MMPF_DRAM_TYPE_NONE = 0,	// no stack memory
    MMPF_DRAM_TYPE_1,			// first used
    MMPF_DRAM_TYPE_2,			// second used
    MMPF_DRAM_TYPE_3,			// third used
    MMPF_DRAM_TYPE_EXT,
    MMPF_DRAM_TYPE_AUTO
} MMPF_DRAM_TYPE;
 
typedef enum _MMPF_DRAM_MODE
{
    MMPF_DRAM_MODE_SDRAM = 0,	// SD RAM
    MMPF_DRAM_MODE_DDR,		// DDR RAM
    MMPF_DRAM_MODE_DDR2,
    MMPF_DRAM_MODE_DDR3,
    MMPF_DRMA_MAX_MODE
} MMPF_DRAM_MODE;

#if (CHIP == MCR_V2)
typedef enum _MMPF_DRAM_ID
{
    MMPF_DRAM_256Mb_WINBOND = 0,
    MMPF_DRAM_256Mb_PIECEMK = 1,
    MMPF_DRAM_128Mb_WINBOND = 2,
    MMPF_DRAM_128Mb_FIDELIX = 4,
    MMPF_DRAM_512Mb_WINBOND = 3,
    MMPF_DRAM_512Mb_PIECEMK = 5
}MMPF_DRAM_ID;

typedef packed_struct _MMP_DRAM_CLK_DLY_SET {
    MMP_UBYTE ubClock;
    MMP_USHORT usDelay;
} MMP_DRAM_CLK_DLY_SET;
#endif
//==============================================================================
//
//                              VARIABLES
//
//==============================================================================


//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

MMP_ERR MMPF_DRAM_Initialize(MMPF_DRAM_TYPE dramtype, MMP_ULONG *ulSize, MMP_ULONG ulClock, MMPF_DRAM_MODE drammode);
MMP_ERR MMPF_DRAM_SendCommand(MMP_USHORT usCmd);
void    MMPF_DRAM_InitSettings(void);
MMP_ERR MMPF_DRAM_GetStackSize(MMP_ULONG *ulSize);
MMP_ERR MMPF_DRAM_SetPowerDown(MMP_BOOL bEnterPowerDown);
MMP_ERR MMPF_DRAM_SetSelfRefresh(MMP_BOOL bEnterSelfRefresh);
MMP_ERR MMPF_DRAM_ConfigPad(MMP_BOOL bEnterPowerDown);
MMP_ERR MMPF_DRAM_ConfigClock(MMP_ULONG ulClock, MMP_ULONG ulWaitCnt);
#if (CHIP == MCR_V2)
void MMPF_DRAM_SendInitCmd(void);
#endif
#if (AUTO_DRAM_LOCKCORE)&&(defined(MBOOT_FW)||defined(UPDATER_FW))
MMP_ERR MMPF_DRAM_ScanNewLockCore(MMPF_DRAM_TYPE dramtype, MMP_ULONG *ulSize, MMP_ULONG ulClock, MMPF_DRAM_MODE drammode);
#endif
/// @}
#endif

/// @end_ait_only

