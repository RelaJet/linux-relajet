#ifndef _MMPF_PWM_H_
#define _MMPF_PWM_H_

//==============================================================================
//
//                              MACRO DEFINE
//
//==============================================================================

#define PWM_SEM_TIMEOUT			0x0

#if (CHIP == P_V2)
#define PWM_MAX_NUM             (2)
#endif
#if (CHIP == MCR_V2)
#define PWM_MAX_NUM_P0          (8)
#define PWM_MAX_NUM_P1          (10)
#define PWM_MAX_NUM             (18)

#define PWM_SET_ID(id)          (id)
#define PWM_SET_PAD(pad)        (pad << 8)
#define PWM_GET_ID(pin)         (pin & 0x1F)
#define PWM_GET_PAD(pin)        ((pin & 0x0100) >> 8)

#define PWN_GET_PIN_OFST(pin)   (PWM_GET_PAD(pin) == 0 ? \
                                 PWM_GET_ID(pin) : \
                                (PWM_GET_PAD(pin)*PWM_MAX_NUM) + PWM_GET_ID(pin) - 4)
#endif

typedef void PwmCallBackFunc(MMP_ULONG);

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

typedef enum _MMPF_PWM_PULSE_ID {
	MMPF_PWM_PULSE_ID_A = 0,
	MMPF_PWM_PULSE_ID_B,
	MMPF_PWM_PULSE_ID_MAX
}MMPF_PWM_PULSE_ID;

typedef enum _MMPF_PWM_INT {
	MMPF_PWM_INT_PulseA = 0x0,
	MMPF_PWM_INT_PulseB ,
	MMPF_PWM_INT_OneRound,
	MMPF_PWM_INT_MAX
}MMPF_PWM_INT;

typedef enum _MMPF_PWMPIN {
	#if (CHIP == P_V2)
	MMPF_PWM0_PIN_CGPIO8 = 0x00,
	MMPF_PWM0_PIN_BGPIO1,
	MMPF_PWM0_PIN_BGPIO8,
	MMPF_PWM1_PIN_HGPIO_CS,
	MMPF_PWM1_PIN_LCD17,
	MMPF_PWM1_PIN_BGPIO2,
	MMPF_PWM1_PIN_BGPIO9,
	#endif
	#if (CHIP == MCR_V2)
	MMPF_PWM0_PIN_AGPIO1       = PWM_SET_ID(0) |PWM_SET_PAD(0),
	MMPF_PWM1_PIN_AGPIO2       = PWM_SET_ID(1) |PWM_SET_PAD(0),
	MMPF_PWM2_PIN_AGPIO3       = PWM_SET_ID(2) |PWM_SET_PAD(0),
	MMPF_PWM3_PIN_AGPIO4       = PWM_SET_ID(3) |PWM_SET_PAD(0),
	MMPF_PWM4_PIN_BGPIO6       = PWM_SET_ID(4) |PWM_SET_PAD(0),
	MMPF_PWM5_PIN_BGPIO7       = PWM_SET_ID(5) |PWM_SET_PAD(0),
	MMPF_PWM6_PIN_BGPIO8       = PWM_SET_ID(6) |PWM_SET_PAD(0),
	MMPF_PWM7_PIN_BGPIO9       = PWM_SET_ID(7) |PWM_SET_PAD(0),
	MMPF_PWM8_PIN_BGPIO10      = PWM_SET_ID(8) |PWM_SET_PAD(0),
	MMPF_PWM9_PIN_BGPIO11      = PWM_SET_ID(9) |PWM_SET_PAD(0),
	MMPF_PWM10_PIN_PLCD0       = PWM_SET_ID(10)|PWM_SET_PAD(0),
	MMPF_PWM11_PIN_PLCD1       = PWM_SET_ID(11)|PWM_SET_PAD(0),
	MMPF_PWM12_PIN_PLCD2       = PWM_SET_ID(12)|PWM_SET_PAD(0),
	MMPF_PWM13_PIN_PLCD3       = PWM_SET_ID(13)|PWM_SET_PAD(0),
	MMPF_PWM14_PIN_PLCD4       = PWM_SET_ID(14)|PWM_SET_PAD(0),
	MMPF_PWM15_PIN_PLCD5       = PWM_SET_ID(15)|PWM_SET_PAD(0),
	MMPF_PWM16_PIN_PLCD1_CS    = PWM_SET_ID(16)|PWM_SET_PAD(0),
	MMPF_PWM17_PIN_PLCD2_CS    = PWM_SET_ID(17)|PWM_SET_PAD(0),
	MMPF_PWM4_PIN_CGPIO12      = PWM_SET_ID(4) |PWM_SET_PAD(1),
	MMPF_PWM5_PIN_CGPIO13      = PWM_SET_ID(5) |PWM_SET_PAD(1),
	MMPF_PWM6_PIN_CGPIO14      = PWM_SET_ID(6) |PWM_SET_PAD(1),
	MMPF_PWM7_PIN_CGPIO15      = PWM_SET_ID(7) |PWM_SET_PAD(1),
	MMPF_PWM8_PIN_CGPIO16      = PWM_SET_ID(8) |PWM_SET_PAD(1),
	MMPF_PWM9_PIN_CGPIO17      = PWM_SET_ID(9) |PWM_SET_PAD(1),
	MMPF_PWM10_PIN_PLCD8       = PWM_SET_ID(10)|PWM_SET_PAD(1),
	MMPF_PWM11_PIN_PLCD9       = PWM_SET_ID(11)|PWM_SET_PAD(1),
	MMPF_PWM12_PIN_PLCD10      = PWM_SET_ID(12)|PWM_SET_PAD(1),
	MMPF_PWM13_PIN_PLCD11      = PWM_SET_ID(13)|PWM_SET_PAD(1),
	MMPF_PWM14_PIN_PLCD12      = PWM_SET_ID(14)|PWM_SET_PAD(1),
	MMPF_PWM15_PIN_PLCD13      = PWM_SET_ID(15)|PWM_SET_PAD(1),
	#endif
	MMPF_PWMPIN_MAX
}MMPF_PWMPIN;

//==============================================================================
//
//                              ENUMERATION
//
//==============================================================================

typedef struct _MMPF_PWM_ATTRIBUTE {
    MMP_UBYTE   ubID;
    MMPF_PWM_PULSE_ID uPulseID;
    MMP_ULONG   ulClkDuty_T0;
    MMP_ULONG   ulClkDuty_T1;
    MMP_ULONG   ulClkDuty_T2;
    MMP_ULONG   ulClkDuty_T3;
    MMP_ULONG   ulClkDuty_Peroid;
    MMP_UBYTE   ubNumOfPulses;
} MMPF_PWM_ATTRIBUTE; 

//==============================================================================
//
//                              FUNCTION PROTOTYPES
//
//==============================================================================

#if (CHIP == P_V2)
MMP_ERR MMPF_I2C_PWM_Initialize(void);
#endif
MMP_ERR MMPF_PWM_Initialize(void);
MMP_ERR MMPF_PWM_SetAttribe(MMPF_PWM_ATTRIBUTE* uPwmAttribute);
MMP_ERR MMPF_PWM_EnableInterrupt(MMP_UBYTE ubID, MMP_BOOL bEnable,
                    PwmCallBackFunc *CallBackFunc, MMPF_PWM_INT IntItem);
MMP_ERR MMPF_PWM_ControlSet(MMP_UBYTE ubID, MMP_UBYTE control);
MMP_ERR MMPF_PWM_EnableOutputPin(MMPF_PWMPIN pwm_pin, MMP_BOOL bEnable);
MMP_ERR MMPF_PWM_OutputPulse(MMPF_PWMPIN pwm_pin, MMP_BOOL bEnableIoPin,
                    MMP_ULONG ulFreq, MMP_BOOL bHigh2Low,
                    MMP_BOOL bEnableInterrupt, PwmCallBackFunc *pwm_callBack,
                    MMP_ULONG ulNumOfPulse);

MMP_ERR MMPF_PWM_SetFreqDuty(MMPF_PWMPIN pwm_pin,MMP_ULONG pdFreqHz, MMP_UBYTE pdDuty);
#endif // _MMPF_PWM_H_