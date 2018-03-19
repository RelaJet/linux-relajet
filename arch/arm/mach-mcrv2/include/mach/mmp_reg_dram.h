//==============================================================================
//
//  File        : mmp_reg_dram.h
//  Description : Dram controller register map
//  Author      : Hans
//  Revision    : 1.0
//
//==============================================================================
/** @addtogroup MMPH_reg
@{
*/
#ifndef _MMP_REG_DRAM_H_
#define _MMP_REG_DRAM_H_

#include    "mmp_register.h"

// *********************************
//   DRAM  Structure (0x8000 6E00)
// *********************************
typedef struct _AITS_DRAM {
    AIT_REG_D   DRAM_INT_CPU_EN;                                        // 0x00
    AIT_REG_D   DRAM_INT_CPU_SR;										// 0x04
    AIT_REG_D   DRAM_INT_HOST_EN;                                       // 0x08
    AIT_REG_D   DRAM_INT_HOST_SR;                                       // 0x0C
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_ARF_DONE            	0x01
        #define DRAM_WAKUP_DONE             0x02
        #define DRAM_EXIT_DPD_DONE          0x04
        #define DRAM_DPD_DONE               0x08
        #define DRAM_LME_DONE               0x10
        #define DRAM_LM_DONE                0x20
        #define DRAM_EXIT_SRF_DONE          0x40
        #define DRAM_SRF_DONE               0x80
        #define DRAM_PRE_ALL_DONE           0x0100
        #define DRAM_INIT_DONE              0x0200
        #define DRAM_CMD_FINISH             0x0400
        /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_CTL_0;												// 0x10
        /*-DEFINE-----------------------------------------------------*/
        #define BANK_SIZE_1M	            0x00
        #define BANK_SIZE_2M	            0x10
        #define BANK_SIZE_4M	            0x20
        #define BANK_SIZE_8M	            0x30
        #define DDR_MODE					0x80
       
        #define DRAM_CAS_LATENCY_1			0x01        
        #define DRAM_CAS_LATENCY_2			0x02        
        #define DRAM_CAS_LATENCY_3			0x03
        #define DRAM_CAS_LATENCY_4			0x04        
        #define DRAM_CAS_LATENCY_5			0x05        
        #define DRAM_CAS_LATENCY_6			0x06        
        #define DRAM_CAS_LATENCY_7			0x07        
        /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_CTL_1;												// 0x11
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_BUS_32		            0x1
        #define DRAM_BUS_16	            	0x0
        #define DRAM_BANK_SEQ               0x4
        #define DRAM_BANK_INTLV           	0x0
        /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_CTL_2;												// 0x12
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_COL_128				0x0 //2^7
        #define DRAM_COL_256				0x1 //2^8
        #define DRAM_COL_512				0x2 //2^9
        #define DRAM_COL_1024				0x3 //2^10
        #define DRAM_COL_2048				0x4 //2^11
        
        #define DRAM_ROW_1024	       		0x00 //2^10
        #define DRAM_ROW_2048	       		0x10 //2^11
        #define DRAM_ROW_4096	       		0x20 //2^12
        #define DRAM_ROW_8192	       		0x30 //2^13
        #define DRAM_ROW_16384	       		0x40 //2^14
        /*------------------------------------------------------------*/
    AIT_REG_B   						_x13;
    AIT_REG_B   DRAM_CTL_4;												// 0x14
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_SW_INIT_ST		       	0x2
        #define DRAM_INIT_ST		       	0x1
        #define DRAM_INIT_CLEAN				0x00
        /*------------------------------------------------------------*/
    AIT_REG_B	DRAM_SW_REST_CTL;										// 0x15
    AIT_REG_B   						_x16[2];
    AIT_REG_W   DRAM_FUNC;												// 0x18
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_ARF_EN			       	0x0001
        #define DRAM_APD_EN			       	0x0002
        #define DRAM_PMP_EN			       	0x0004
        #define DRAM_SPRE_EN      			0x0008
        #define DRAM_NA_EN      			0x0010
        #define DRAM_CLK_GATE_EN      		0x0020
        #define DRAM_CMD_ISSUE_MODE    		0x0040
        #define DRAM_MCI_REQ_DIS   			0x0080
        #define DRAM_EH_SPRE_EN			    0x0100
		#define DRAM_DDR_RW_WAIT_EN	    	0x0400
        /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_SIGNAL_CTL;                                        // 0x1A
	    /*-DEFINE-----------------------------------------------------*/
	    #define DRAM_CLK_OUT_INV_EN         0x01
	    #define DRAM_CLK_OUT_DLY_EN			0x02
	    #define DRAM_CLK_IN_INV_EN          0x04
	    #define DRAM_CLK_IN_DLY_EN			0x08
	    #define DRAM_CYC_DLY_EN  			0x10
	    /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_DFI_CTL;											// 0x1B
	    /*-DEFINE-----------------------------------------------------*/
        
        /*------------------------------------------------------------*/
    AIT_REG_W   DRAM_ARF_CYC;											// 0x1C
    AIT_REG_B   DRAM_ARF_QUEUE_DEPTH;									// 0x1E
    AIT_REG_B   						_x1F;
    AIT_REG_D   DRAM_CTL_CYC_0;											// 0x20;
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_TRAS_CYC(_a)			(_a)
        #define DRAM_TRC_SHFT		       	4
        #define DRAM_TRC_CYC(_a)			(MMP_ULONG)(_a << DRAM_TRC_SHFT)
        #define DRAM_TRCD_SHFT		       	8
        #define DRAM_TRCD_CYC(_a)			(MMP_ULONG)(_a << DRAM_TRCD_SHFT)
        #define DRAM_TRFC_SHFT		       	12
        #define DRAM_TRFC_CYC(_a)			(MMP_ULONG)((_a & 0xF) << DRAM_TRFC_SHFT)
        #define DRAM_TRP_SHFT		       	16
        #define DRAM_TRP_CYC(_a)			(MMP_ULONG)(_a << DRAM_TRP_SHFT)
        #define DRAM_TRRD_SHFT		       	20
        #define DRAM_TRRD_CYC(_a)			(MMP_ULONG)(_a << DRAM_TRRD_SHFT)
        #define DRAM_TXSR_SHFT		       	24
        #define DRAM_TXSR_CYC(_a)			(MMP_ULONG)((_a & 0xF) << DRAM_TXSR_SHFT)
        #define DRAM_TMRD_SHFT		       	28
        #define DRAM_TMRD_CYC(_a)			((MMP_ULONG)_a << DRAM_TMRD_SHFT)
        /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_CTL_CYC_1;											// 0x24
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_TWR_CYC(_a)			(_a)
        #define DRAM_TRTP_SHFT		        4
        #define DRAM_TRTP_CYC(_a)		    (_a << DRAM_TRTP_SHFT)
        #define DRAM_TOEFREE_SHFT		    4
        #define DRAM_TOEFREE_CYC(_a)		(_a << DRAM_TOEFREE_SHFT)
        /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_CTL_CYC_DDR;                                       // 0x25
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_TWTR_SHFT		       	4
        #define DRAM_TWTR_CYC(_a)			((MMP_ULONG)_a << DRAM_TWTR_SHFT)
        #define DRAM_TXP_CYC(_a)			((MMP_ULONG)_a)
        /*------------------------------------------------------------*/
    AIT_REG_W   						_x26;							// 0x26~0x27 reserved
    AIT_REG_W   DRAM_MODE_REG;											// 0x28
    AIT_REG_W   DRAM_EXT_MODE_REG;										// 0x2A
    AIT_REG_B   DRAM_RDDATA_CLK_DLY;									// 0x2C
    AIT_REG_B   DRAM_CLK_OUT_DLY;										// 0x2D
    AIT_REG_B   DRAM_RDDATA_RDY_DLY;
    AIT_REG_B   						_x2F;                           // 0x2F reserved
    AIT_REG_W   DRAM_CMD_CTL0;											// 0x30
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_ARF_ST					0x0001
        #define DRAM_PUP_RDY_ST				0x0002
        #define DRAM_EXIT_DPD_ST			0x0004
        #define DRAM_DPD_ST					0x0008
        #define DRAM_LME_ST					0x0010
        #define DRAM_LM_ST					0x0020
        #define DRAM_EXIT_SRF_ST			0x0040
        #define DRAM_SRF_ST					0x0080
        #define DRAM_PRE_ALL_ST				0x0100
        /*------------------------------------------------------------*/
    AIT_REG_B   DRAM_CMD_CTL1;											// 0x32
    AIT_REG_B   						_x33[1];
    AIT_REG_B   DRAM_INIT_CYC;                                       	// 0x34
    AIT_REG_B   DRAM_DPD_CYC;                                       	// 0x35
    AIT_REG_W   DRAM_CTL_CYC_EXT;                                       // 0x36;
        /*-DEFINE-----------------------------------------------------*/
        #define DRAM_TRAS_EXT_CYC(_a)       (_a & 0x10)
        #define DRAM_TRC_EXT_SHFT           1
        #define DRAM_TRC_EXT_CYC(_a)        (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TRC_EXT_SHFT)
        #define DRAM_TRCD_EXT_SHFT          2
        #define DRAM_TRCD_EXT_CYC(_a)       (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TRCD_EXT_SHFT)
        #define DRAM_TRFC_EXT_SHFT          3
        #define DRAM_TRFC_EXT_CYC(_a)       (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TRFC_EXT_SHFT)
        #define DRAM_TRP_EXT_SHFT           4
        #define DRAM_TRP_EXT_CYC(_a)        (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TRP_EXT_SHFT)
        #define DRAM_TRRD_EXT_SHFT          5
        #define DRAM_TRRD_EXT_CYC(_a)       (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TRRD_EXT_SHFT)
        #define DRAM_TXSR_EXT_SHFT          6
        #define DRAM_TXSR_EXT_CYC(_a)       (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TXSR_EXT_SHFT)
        #define DRAM_TMRD_EXT_SHFT          7
        #define DRAM_TMRD_EXT_CYC(_a)       (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TMRD_EXT_SHFT)
        #define DRAM_TWR_EXT_SHFT           8
        #define DRAM_TWR_EXT_CYC(_a)        (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TWR_EXT_SHFT)
        #define DRAM_TXP_EXT_SHFT           10
        #define DRAM_TXP_EXT_CYC(_a)        (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TXP_EXT_SHFT)
        #define DRAM_TWTR_EXT_SHFT          11
        #define DRAM_TWTR_EXT_CYC(_a)       (MMP_USHORT)(((_a & 0x10) >> 4) << DRAM_TWTR_EXT_SHFT)
        /*------------------------------------------------------------*/
    AIT_REG_B	DRAM_CMD_QUEUE_SR;											//0x38
    AIT_REG_B   						_x39[0x17];
    AIT_REG_B	DRAM_DFI_UPDATE_MIN_CYC;									//0x50
    AIT_REG_B	DRAM_DFI_UPDATE_MAX_CYC;									//0x51
    AIT_REG_B   						_x52[0xE];
    #if (CHIP == VSN_V2)
    AIT_REG_B   DRAM_DDR_PHY_CTL;                                           // 0x60
        /*-DEFINE-----------------------------------------------------*/				
		#define DRAM_DDR_PWR_EN             0x02
		#define DRAM_DDR_DLL_RST            0x01
		#define DDR_PHY_CTL_CLEAN			0x00
		/*------------------------------------------------------------*/
	AIT_REG_B   						_x61;                               					
	AIT_REG_B   DRAM_DDR_DLY_LINE_CTL;										// 0x62
    AIT_REG_B   DRAM_DDR_LOCK_LOOP_CTL;                                     // 0x63
    	/*-DEFINE-----------------------------------------------------*/
        #define DDR_HW_CTL_BY_SW       		0x20;
        #define DDR_STOP_DLYBIT_CTL			0x40;
        #define DDR_STOP_DLYBIT_TEST		0x01;
        /*------------------------------------------------------------*/						
	AIT_REG_B   DRAM_DDR_LOCK_LOOP_OP;										// 0x64
	AIT_REG_B   DRAM_DDR_LOCK_JUDGE_CTL;									// 0x65
		/*-DEFINE-----------------------------------------------------*/
        #define DDR_LOCK_REF_CLK_CNT(_a)       (_a & 0x1F)
        /*------------------------------------------------------------*/
	AIT_REG_B   DRAM_DDR_LOCK_LOOP_SR;                                     // 0x66
    AIT_REG_B   DRAM_DDR_CLK_OPT;                                           // 0x67
    	/*-DEFINE-----------------------------------------------------*/
        #define DQCLK_DUTY_BYPASS      		0x08
        /*------------------------------------------------------------*/		
	AIT_REG_B   DRAM_DDR_PHY_DQB0_OP;                                       // 0x68
		/*-DEFINE-----------------------------------------------------*/
        #define DQCLK_DUTY_TUNING_BYPASS	0x08
        /*------------------------------------------------------------*/
	AIT_REG_B   DRAM_DDR_PHY_DQB0_CTL_OP;                                   // 0x69
		/*-DEFINE-----------------------------------------------------*/
        #define DQ_BLOCK_CTL(_a)       (_a)
        /*------------------------------------------------------------*/
	AIT_REG_B   DRAM_DDR_PHY_DQB1_OP;                                       // 0x6A
	AIT_REG_B   DRAM_DDR_PHY_DQB1_CTL_OP;                                   // 0x6B
	AIT_REG_B   DRAM_DDR_PHY_DQB2_OP;                                       // 0x6C
	AIT_REG_B   DRAM_DDR_PHY_DQB2_CTL_OP;                                   // 0x6D	
	AIT_REG_B   DRAM_DDR_PHY_DQB3_OP;                                       // 0x6E
	AIT_REG_B   DRAM_DDR_PHY_DQB3_CTL_OP;                                   // 0x6F
	#endif // (CHIP == VSN_V2)
	
	#if (CHIP == VSN_V3)
	AIT_REG_B	DRAM_DDR_DLL_CTL;											// 0x60
		/*-DEFINE-----------------------------------------------------*/				
		#define DRAM_DDR_DLL0_RST           0x01
		#define DRAM_DDR_DLL1_RST           0x02
		#define DRAM_DDR_DLL0_PWR_DOWN      0x04
		#define DRAM_DDR_DLL1_PWR_DOWN      0x08
		#define DDR_PHY_CTL_CLEAN			0x00
		/*------------------------------------------------------------*/
	AIT_REG_B	DRAM_DDR_DQPLK_PD_CTL;										// 0x61
	AIT_REG_B   						_x62;								// 0x62 reserved
	AIT_REG_B	DRAM_DDR_DLL0_CTL;											// 0x63
		/*-DEFINE-----------------------------------------------------*/
        #define DDR_HW_CTL_BY_SW       		0x20;
        #define DDR_STOP_DLYBIT_CTL			0x40;
        #define DDR_STOP_DLYBIT_TEST		0x01;
        /*------------------------------------------------------------*/
	AIT_REG_B	DRAM_DDR_DLL0_OPT;											// 0x64
	AIT_REG_B	DRAM_DDR_DLL0_LOCK_CTL;										// 0x65
		/*-DEFINE-----------------------------------------------------*/
        #define DDR_LOCK_REF_CLK_CNT(_a)       (_a & 0x1F)
        /*------------------------------------------------------------*/
	AIT_REG_B	DRAM_DDR_DLL0_LOCK_SR;										// 0x66
	AIT_REG_B	DRAM_DDR_DLL1_CTL;											// 0x67
	AIT_REG_B	DRAM_DDR_DLL1_OPT;											// 0x68
	AIT_REG_B	DRAM_DDR_DLL1_LOCK_CTL;										// 0x69
	AIT_REG_B	DRAM_DDR_DLL1_LOCK_SR;										// 0x6A
	AIT_REG_B	DRAM_DDR_CLOCK_OPT;											// 0x6B
		/*-DEFINE-----------------------------------------------------*/
        #define DQCLK_DUTY_BYPASS      		0x08
        /*------------------------------------------------------------*/
	AIT_REG_B   						_x6C[0x04];							// 0x6C~0x6F reserved
	#endif //#if (CHIP == VSN_V3)
		
    AIT_REG_B   DRAM_DDR_CLK_MACRO_DLY;										// 0x70
    AIT_REG_B   DRAM_DDR_OPR_CTL;											// 0x71
    	/*-DEFINE-----------------------------------------------------*/				
		#define DRAM_DDR_CLK_PAD_EN       	0x01
		#if (CHIP == VSN_V3)
		#define DRAM_DQBLK0_SDR_EN			0x02
		#define DRAM_DQBLK1_SDR_EN			0x04
		#define DRAM_DQBLK2_SDR_EN			0x08
		#define DRAM_DQBLK3_SDR_EN			0x10
		#define DRAM_DQBLK4_SDR_EN			0x20
		#define DRAM_DQBLK5_SDR_EN			0x40
		#endif
		#define DRAM_SDR_MODE             	0x80
		
		/*------------------------------------------------------------*/
	
    AIT_REG_B   DRAM_DDR_PHY_WR_CTL;										// 0x72
    AIT_REG_B   						_x73;
    AIT_REG_D   DRAM_SRR_DATA_BYTE;											// 0x74
    #if (CHIP == VSN_V2)
    AIT_REG_D   DRAM_DQBLK_CTL;												// 0x78
    AIT_REG_D   DRAM_DQBLK_SR;												// 0x7C
    AIT_REG_B   						_x80[0x7E];							// 0x80~0xFD reserved
    #endif
    #if (CHIP == VSN_V3)
    AIT_REG_B	DRAM_DQBLK0_SR;												// 0x78
    AIT_REG_B	DRAM_DQBLK1_SR;												// 0x79
    AIT_REG_B	DRAM_DQBLK2_SR;												// 0x7A
    AIT_REG_B	DRAM_DQBLK3_SR;												// 0x7B
    AIT_REG_B	DRAM_DQBLK4_SR;												// 0x7C
    AIT_REG_B	DRAM_DQBLK5_SR;												// 0x7D
    AIT_REG_B   						_x7E[0x2];							// 0x7E~0x7F reserved
    AIT_REG_B	DRAM_DQBLK0_RD_CTL;											// 0x80
    AIT_REG_B	DRAM_DQBLK0_WR_CTL;											// 0x81
    AIT_REG_B	DRAM_DQBLK1_RD_CTL;											// 0x82
    AIT_REG_B	DRAM_DQBLK1_WR_CTL;											// 0x83
    AIT_REG_B	DRAM_DQBLK2_RD_CTL;											// 0x84
    AIT_REG_B	DRAM_DQBLK2_WR_CTL;											// 0x85
    AIT_REG_B	DRAM_DQBLK3_RD_CTL;											// 0x86
    AIT_REG_B	DRAM_DQBLK3_WR_CTL;											// 0x87
    AIT_REG_B	DRAM_DQBLK4_RD_CTL;											// 0x88
    AIT_REG_B	DRAM_DQBLK4_WR_CTL;											// 0x89
    AIT_REG_B	DRAM_DQBLK5_RD_CTL;											// 0x8A
    AIT_REG_B	DRAM_DQBLK5_WR_CTL;											// 0x8B
    AIT_REG_B   						_x8C[0x4];							// 0x8C~0x8F reserved
    AIT_REG_B	DRAM_DDR_DQBLK0_OPT;										// 0x90
    AIT_REG_B	DRAM_DDR_DQBLK0_CTL_OPT;									// 0x91
    AIT_REG_B	DRAM_DDR_DQBLK1_OPT;										// 0x92
    AIT_REG_B	DRAM_DDR_DQBLK1_CTL_OPT;									// 0x93
    AIT_REG_B	DRAM_DDR_DQBLK2_OPT;										// 0x94
    AIT_REG_B	DRAM_DDR_DQBLK2_CTL_OPT;									// 0x95
    AIT_REG_B	DRAM_DDR_DQBLK3_OPT;										// 0x96
    AIT_REG_B	DRAM_DDR_DQBLK3_CTL_OPT;									// 0x97
    AIT_REG_B	DRAM_DDR_DQBLK4_OPT;										// 0x98
    AIT_REG_B	DRAM_DDR_DQBLK4_CTL_OPT;									// 0x99
    AIT_REG_B	DRAM_DDR_DQBLK5_OPT;										// 0x9A
    AIT_REG_B	DRAM_DDR_DQBLK5_CTL_OPT;									// 0x9B
    AIT_REG_B	DRAM_DDR_DQBLK0_CTL;										// 0x9C
    AIT_REG_B	DRAM_DDR_DQBLK1_CTL;										// 0x9D
    AIT_REG_B	DRAM_DDR_DQBLK2_CTL;										// 0x9E
    AIT_REG_B	DRAM_DDR_DQBLK3_CTL;										// 0x9F
    AIT_REG_B	DRAM_DDR_DQBLK4_CTL;										// 0xA0
    AIT_REG_B	DRAM_DDR_DQBLK5_CTL;										// 0xA1
    AIT_REG_B   						_xA2[0x5C];							// 0xA2~0xFD reserved
    #endif
    AIT_REG_B   DRAM_DDR_TEST_MODE;											// 0xFE
    AIT_REG_B   DRAM_ASIC_PROBE_SEL;										// 0xFF
} AITS_DRAM, *AITPS_DRAM;

#if !defined(BUILD_FW)
/*#define DRAM_INT_CPU_SR                 (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_INT_CPU_SR   )))
#define DRAM_CTL_0				        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_0        )))
#define DRAM_CTL_1				        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_1        )))
#define DRAM_CTL_2				        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_2        )))
#define DRAM_CTL_4				        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_4        )))
#define DRAM_FUNC				        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_FUNC         )))
#define DRAM_ARF_CYC  		        	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_ARF_CYC      )))
#define DRAM_ARF_QUEUE_DEPTH        	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_ARF_QUEUE_DEPTH)))
#define DRAM_CTL_CYC_0		        	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_CYC_0	  )))
#define DRAM_CTL_CYC_1		        	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_CYC_1	  )))
#define DRAM_MODE_REG               	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_MODE_REG     )))
#define DRAM_EXT_MODE_REG             	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_EXT_MODE_REG )))
#define DRAM_RDDATA_CLK_DLY         	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_RDDATA_CLK_DLY)))
#define DRAM_CLK_OUT_DLY	        	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CLK_OUT_DLY  )))
#define DRAM_CMD_CTL		        	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CMD_CTL      )))
#define DRAM_DPD_ARF_CYC            	(DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DPD_ARF_CYC  )))
#define DRAM_INT_HOST_SR		        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_INT_HOST_SR )))
#define DRAM_SIGNAL_CTL                 (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_SIGNAL_CTL ))) 
#define DRAM_DDR_PHY_CTL		        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_PHY_CTL )))
#define DRAM_DDR_LOOP_CTL		        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_LOOP_CTL )))
#define DRAM_DDR_PHY_OPT                (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_PHY_OPT )))
#define DRAM_DDR_CLK_OPT                (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_CLK_OPT )))
#define DRAM_DDR_PHY_DQB0_OP            (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_PHY_DQB0_OP )))
#define DRAM_DDR_PHY_DQB1_OP            (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_PHY_DQB1_OP )))
#define DRAM_DDR_PHY_DQB2_OP            (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_PHY_DQB2_OP )))
#define DRAM_DDR_PHY_DQB3_OP            (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_PHY_DQB3_OP )))
#define DRAM_CTL_CYC_EXT                (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_CYC_EXT )))
#define DRAM_CTL_CYC_DDR                (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_CTL_CYC_DDR  )))
#define DRAM_DDR_ANA_CLK_OUT_DLY        (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_DDR_ANA_CLK_OUT_DLY )))
#define DRAM_INIT_WAIT_CNT              (DRAM_BASE  +(MMP_ULONG)(&(((AITPS_DRAM)0)->DRAM_INIT_WAIT_CNT )))
*/
#endif

/// @}
#endif // _MMP_REG_DRAM_H_
