#ifndef _RT5627_H
#define _RT5627_H

#define RT5627_RESET						0X00			//RESET CODEC TO DEFAULT
#define RT5627_SPK_OUT_VOL					0X02			//SPEAKER OUT VOLUME
#define RT5627_HP_OUT_VOL					0X04			//HEADPHONE OUTPUT VOLUME
#define RT5627_AUXIN_VOL					0X08			//AUXIN VOLUME
#define RT5627_LINE_IN_VOL					0X0A			//LINE IN VOLUME
#define RT5627_STEREO_DAC_VOL				0X0C			//STEREO DAC VOLUME
#define RT5627_SOFT_VOL_CTRL_TIME			0X16			//SOFT DELAY VOLUME CONTROL TIME
#define RT5627_OUTPUT_MIXER_CTRL			0X1C			//OUTPUT MIXER CONTROL
#define	RT5627_AUDIO_DATA_CTRL				0X34			//STEREO AUDIO DATA CONTROL
#define	RT5627_DAC_CLK_CTRL					0X38			//STEREO DAC CLOCK CONTROL
#define	RT5627_PWR_MANAG_ADD1				0X3A			//POWER MANAGMENT ADDITION 1
#define RT5627_PWR_MANAG_ADD2				0X3C			//POWER MANAGMENT ADDITION 2
#define RT5627_PWR_MANAG_ADD3				0X3E			//POWER MANAGMENT ADDITION 3
#define RT5627_GEN_CTRL						0X40			//GENERAL PURPOSE CONTROL
#define	RT5627_GLOBAL_CLK_CTRL				0X42			//GLOBAL CLOCK CONTROL
#define RT5627_PLL_CTRL						0X44			//PLL CONTROL
#define RT5627_GPIO_PIN_CONFIG				0X48			//GPIO PIN CONFIG	
#define RT5627_GPIO_OUTPUT_PIN_CTRL			0X4A			//GPIO CONTROL
#define RT5627_JACK_DET_CTRL				0X5A			//JACK DETECT CONTROL 
#define RT5627_MISC1_CTRL					0X5C			//MISC1 CONTROL
#define RT5627_MISC2_CTRL					0X5E			//MISC2 CONTROL
#define RT5627_AVC_CTRL						0X68			//AVC CONTROL
#define RT5627_HID_CTRL_INDEX				0X6A			//PRIVATE REGISTER ADDRESS
#define RT5627_HID_CTRL_DATA				0X6C			//PRIVATE REGISTER DATA
#define RT5627_VENDOR_ID1	  		    	0x7C			//VENDOR ID1
#define RT5627_VENDOR_ID2	  		    	0x7E			//VENDOR ID2



//global definition
#define RT_L_MUTE						(0x1<<15)		//MUTE LEFT CONTROL BIT
#define RT_R_MUTE						(0x1<<7)		//MUTE RIGHT CONTROL BIT
#define RT_M_HP_MIXER					(0x1<<15)		//Mute source to HP Mixer
#define RT_M_SPK_MIXER					(0x1<<14)		//Mute source to Speaker Mixer
#define RT_M_MONO_MIXER					(0x1<<13)		//Mute source to Mono Mixer

#define ALL_FIELD				0xffff   

//Output Mixer Control(0x1C)
#define	SPKOUT_N_SOUR_MASK					(0x3<<14)
#define	SPKOUT_N_SOUR_MUTE					(0x3<<14)	
#define	SPKOUT_N_SOUR_LN					(0x2<<14)
#define	SPKOUT_N_SOUR_RP					(0x1<<14)
#define	SPKOUT_N_SOUR_RN					(0x0<<14)
#define SPKOUT_INPUT_SEL_MASK				(0x3<<10)
#define SPKOUT_INPUT_SEL_SPKMIXER			(0x2<<10)
#define SPKOUT_INPUT_SEL_HPMIXER			(0x1<<10)
#define SPKOUT_INPUT_SEL_VMID				(0x0<<10)
#define HPL_INPUT_SEL_HPLMIXER				(0x1<<9)
#define HPR_INPUT_SEL_HPRMIXER				(0x1<<8)
#define AUXOUT_L_INPUT_SEL_MASK				(0x1<<7)
#define AUXOUT_L_INPUT_SEL_HPLMIXER			(0x1<<7)	
#define AUXOUT_L_INPUT_SEL_SPKMIXER			(0x0<<7)
#define AUXOUT_R_INPUT_SEL_MASK				(0x1<<6)
#define AUXOUT_R_INPUT_SEL_HPLMIXER			(0x1<<6)	
#define AUXOUT_R_INPUT_SEL_SPKMIXER			(0x0<<6)
#define SPK_VOL_DIFF_NEG_SIG_ENA			(0x1<<2)
#define DAC_DIRECT_TO_HP					(0x1<<1)
#define DAC_DIRECT_TO_AUXOUT				(0x1)


//Audio Interface(0x34)			
#define SDP_MASTER_MODE				(0x0<<15)
#define SDP_SLAVE_MODE				(0x1<<15)
#define MAIN_I2S_BCLK_POL_CTRL		(0x1<<7)		//0:Normal 1:Invert
#define DAC_DATA_L_R_SWAP			(0x1<<4)		//0:DAC data appear at left phase of LRCK
													//1:DAC data appear at right phase of LRCK	
//Data Length Slection
#define I2S_DL_MASK				(0x3<<2)		//main i2s Data Length mask	
#define I2S_DL_16				(0x0<<2)		//16 bits
#define I2S_DL_20				(0x1<<2)		//20 bits
#define	I2S_DL_24				(0x2<<2)		//24 bits
													
//PCM Data Format Selection
#define I2S_DF_MASK				(0x3)			//IIS Data Format Mask
#define I2S_DF_I2S				(0x0)			//I2S FORMAT 
#define I2S_DF_LEFT				(0x1)			//LEFT JUSTIFIED FORMAT
#define	I2S_DF_PCM_A			(0x2)			//PCM MODE A
#define I2S_DF_PCM_B			(0x3)			//PCM MODE B

//Stereo AD/DA Clock Control(0x38h)
#define I2S_PRE_DIV_MASK		(0x7<<13)			
#define I2S_PRE_DIV_1			(0x0<<13)			//DIV 1
#define I2S_PRE_DIV_2			(0x1<<13)			//DIV 2
#define I2S_PRE_DIV_4			(0x2<<13)			//DIV 4
#define I2S_PRE_DIV_8			(0x3<<13)			//DIV 8
#define I2S_PRE_DIV_16			(0x4<<13)			//DIV 16
#define I2S_PRE_DIV_32			(0x5<<13)			//DIV 32

#define I2S_BCLK_SEL_64FS			(0x0<<12)			//32 BITS(64FS)
#define I2S_BCLK_SEL_32FS			(0x1<<12)			//16 BITS(32FS)

#define DAC_FILTER_CLK_SEL_256FS	(0<<2)			//256FS
#define DAC_FILTER_CLK_SEL_384FS	(1<<2)			//384FS

//Power managment addition 1 (0x3A),0:Disable,1:Enable
#define PWR_MAIN_I2S_EN				(0x1<<15)
#define PWR_ZC_DET_PD_EN			(0x1<<14)	
#define PWR_SOFTGEN_EN				(0x1<<8)
#define	PWR_HP_AMP					(0x1<<5)
#define	PWR_HP_ENH_AMP				(0x1<<4)

//Power managment addition 2(0x3C),0:Disable,1:Enable
#define PWR_CLASS_D					(0x1<<14)
#define PWR_VREF					(0x1<<13)
#define PWR_PLL						(0x1<<12)
#define PWR_THERMAL_SD				(0x1<<11)
#define PWR_DAC_REF_CIR				(0x1<<10)
#define PWR_L_DAC_CLK				(0x1<<9)
#define PWR_R_DAC_CLK				(0x1<<8)
#define PWR_L_DAC_L_D_S				(0x1<<7)
#define PWR_R_DAC_R_D_S				(0x1<<6)
#define PWR_L_HP_MIXER				(0x1<<5)
#define PWR_R_HP_MIXER				(0x1<<4)
#define PWR_SPK_MIXER				(0x1<<3)


//Power managment addition 3(0x3E),0:Disable,1:Enable
#define PWR_MAIN_BIAS				(0x1<<15)
#define PWR_SPK_OUT					(0x1<<12)
#define PWR_HP_L_OUT_VOL_AMP		(0x1<<10)
#define PWR_HP_R_OUT_VOL_AMP		(0x1<<9)
#define PWR_LINEIN_L_VOL			(0x1<<7)
#define PWR_LINEIN_R_VOL			(0x1<<6)
#define PWR_AUXIN_L_VOL				(0x1<<5)
#define PWR_AUXIN_R_VOL				(0x1<<4)

//Additional Control Register(0x40)
#define SPK_D_AMP_CTRL_MASK				(0x7<<9)
#define SPK_D_AMP_CTRL_RATIO_225		(0x0<<9)		//2.25 Vdd
#define SPK_D_AMP_CTRL_RATIO_200		(0x1<<9)		//2.00 Vdd
#define SPK_D_AMP_CTRL_RATIO_175		(0x2<<9)		//1.75 Vdd
#define SPK_D_AMP_CTRL_RATIO_150		(0x3<<9)		//1.50 Vdd
#define SPK_D_AMP_CTRL_RATIO_125		(0x4<<9)		//1.25 Vdd	
#define SPK_D_AMP_CTRL_RATIO_100		(0x5<<9)		//1.00 Vdd

#define STEREO_DAC_H_PASS_EN			(0x1<<8)		//enable HIGH PASS FILTER FOR DAC

//Global Clock Control Register(0x42)
#define SYSCLK_SOUR_SEL_MASK			(0x1<<15)
#define SYSCLK_SOUR_SEL_MCLK			(0x0<<15)		//system Clock source from MCLK
#define SYSCLK_SOUR_SEL_PLL				(0x1<<15)		//system Clock source from PLL
#define PLLCLK_SOUR_SEL_MCLK			(0x0<<14)		//PLL clock source from MCLK
#define PLLCLK_SOUR_SEL_BITCLK			(0x1<<14)		//PLL clock source from BITCLK

#define PLLCLK_DIV_RATIO_MASK			(0x3<<1)		
#define PLLCLK_DIV_RATIO_DIV1			(0x0<<1)		//DIV 1
#define PLLCLK_DIV_RATIO_DIV2			(0x1<<1)		//DIV 2
#define PLLCLK_DIV_RATIO_DIV4			(0x2<<1)		//DIV 4
#define PLLCLK_DIV_RATIO_DIV8			(0x3<<1)		//DIV 8

#define PLLCLK_PRE_DIV1					(0x0)			//DIV 1
#define PLLCLK_PRE_DIV2					(0x1)			//DIV 2

//PLL Control(0x44)

#define PLL_CTRL_M_VAL(m)		((m)&0xf)
#define PLL_CTRL_K_VAL(k)		(((k)&0x7)<<4)
#define PLL_CTRL_N_VAL(n)		(((n)&0xff)<<8)


//GPIO CONTROL(0x4A)
#define GPIO_PIN_SEL_MASK			(0x3<<14)
#define GPIO_PIN_SEL_LOG_OUT		(0x0<<14)
#define GPIO_PIN_SEL_IRQ			(0x1<<14)
#define GPIO_PIN_SEL_PLLOUT			(0x3<<14)

#define GPIO_PIN_CON_MASK			(0x1<<3)
#define GPIO_PIN_CON_OUTPUT			(0x0<<3)
#define GPIO_PIN_CON_INPUT			(0x1<<3)

#define GPIO_PIN_OUTPUT_SET_MASK	(0x1<<2)
#define GPIO_PIN_OUTPUT_SET_LOW		(0x0<<2)
#define GPIO_PIN_OUTPUT_SET_HIGH	(0x1<<2)

#define GPIO_PIN_POLARITY_INV		(0x1<<1)

//JACK DETECT CONTROL(0x5A)
#define JACK_DET_SEL_MASK			(0x3<<14)
#define JACK_DET_SEL_OFF			(0x0<<14)		//Jack Detect Select none
#define JACK_DET_SEL_GPIO			(0x1<<14)		//Jack Detect Select GPIO
#define JACK_DET_SEL_JD1			(0x2<<14)		//Jack Detect Select JD1,LineIn Left disable
#define JACK_DET_SEL_JD2			(0x3<<14)		//Jack Detect Select JD2,LineIn Right Disable

#define JACK_DET_TRI_VREF			(0x1<<13)		
#define JACK_DET_POL_TRI_VREF		(0x1<<12)
#define JACK_DET_TRI_HP				(0x1<<11)
#define JACK_DET_POL_TRI_HP			(0x1<<10)
#define JACK_DET_TRI_SPK			(0x1<<9)
#define JACK_DET_POL_TRI_SPK		(0x1<<8)
#define JACK_DET_POL				(0x1<<3)


//MISC1 CONTROL(0x5C)
#define SPK_L_ZC_CTRL_EN				(0x1<<15)
#define SPK_L_SV_CTRL_EN				(0x1<<14)
#define SPK_R_ZC_CTRL_EN				(0x1<<13)
#define SPK_R_SV_CTRL_EN				(0x1<<12)
#define HP_L_ZC_CTRL_EN					(0x1<<11)
#define HP_L_SV_CTRL_EN					(0x1<<10)
#define HP_R_ZC_CTRL_EN					(0x1<<9)
#define HP_R_SV_CTRL_EN					(0x1<<8)
#define AUXOUT_L_ZC_CTRL_EN				(0x1<<7)
#define AUXOUT_L_SV_CTRL_EN				(0x1<<6)
#define AUXOUT_R_ZC_CTRL_EN				(0x1<<5)
#define AUXOUT_R_SV_CTRL_EN				(0x1<<4)
#define DAC_ZC_CTRL_EN					(0x1<<3)
#define DAC_SV_CTRL_EN					(0x1<<2)


////MISC2 CONTROL(0x5E)
#define DISABLE_FAST_VREG			(0x1<<15)
#define THERMAL_SHUTDOWN_EN			(0x1<<14)
#define HP_DEPOP_MODE2_EN			(0x1<<9)
#define HP_DEPOP_MODE1_EN			(0x1<<8)
#define HP_L_M_UM_DEPOP_EN			(0x1<<7)
#define HP_R_M_UM_DEPOP_EN			(0x1<<6)
#define M_UM_DEPOP_EN				(0x1<<5)
#define M_DAC_L_INPUT				(0x1<<3)
#define M_DAC_R_INPUT				(0x1<<2)

//AVC Control(0x68)
#define AVC_ENABLE				(0x1<<15)
#define AVC_TARTGET_SEL_MASK	(0x1<<14)
#define	AVC_TARTGET_SEL_R 		(0x1<<14)
#define	AVC_TARTGET_SEL_L		(0x0<<14)


//WaveOut channel for realtek codec
enum 
{
	RT_WAVOUT_SPK  				=(0x1<<0),
	RT_WAVOUT_SPK_R				=(0x1<<1),
	RT_WAVOUT_SPK_L				=(0x1<<2),
	RT_WAVOUT_HP				=(0x1<<3),
	RT_WAVOUT_HP_R				=(0x1<<4),
	RT_WAVOUT_HP_L				=(0x1<<5),	
	RT_WAVOUT_MONO				=(0x1<<6),
	RT_WAVOUT_AUXOUT			=(0x1<<7),
	RT_WAVOUT_AUXOUT_R			=(0x1<<8),
	RT_WAVOUT_AUXOUT_L			=(0x1<<9),
	RT_WAVOUT_LINEOUT			=(0x1<<10),
	RT_WAVOUT_LINEOUT_R			=(0x1<<11),
	RT_WAVOUT_LINEOUT_L			=(0x1<<12),	
	RT_WAVOUT_DAC				=(0x1<<13),		
	RT_WAVOUT_ALL_ON			=(0x1<<14),
};

//WaveIn channel for realtek codec
enum
{
	RT_WAVIN_R_MONO_MIXER		=(0x1<<0),
	RT_WAVIN_R_SPK_MIXER		=(0x1<<1),
	RT_WAVIN_R_HP_MIXER			=(0x1<<2),
	RT_WAVIN_R_PHONE			=(0x1<<3),
	RT_WAVIN_R_AUXIN			=(0x1<<3),	
	RT_WAVIN_R_LINE_IN			=(0x1<<4),
	RT_WAVIN_R_MIC2				=(0x1<<5),
	RT_WAVIN_R_MIC1				=(0x1<<6),

	RT_WAVIN_L_MONO_MIXER		=(0x1<<8),
	RT_WAVIN_L_SPK_MIXER		=(0x1<<9),
	RT_WAVIN_L_HP_MIXER			=(0x1<<10),
	RT_WAVIN_L_PHONE			=(0x1<<11),
	RT_WAVIN_L_AUXIN			=(0x1<<11),
	RT_WAVIN_L_LINE_IN			=(0x1<<12),
	RT_WAVIN_L_MIC2				=(0x1<<13),
	RT_WAVIN_L_MIC1				=(0x1<<14),
};

enum 
{
	POWER_STATE_D0=0,
	POWER_STATE_D1,
	POWER_STATE_D1_PLAYBACK,
	POWER_STATE_D1_RECORD,
	POWER_STATE_D2,
	POWER_STATE_D2_PLAYBACK,
	POWER_STATE_D2_RECORD,
	POWER_STATE_D3,
	POWER_STATE_D4

}; 

/*virtual reg (0x84)*/
#define HPL_VOLUME_CONTROL_BIT (0x1)
#define HPR_VOLUME_CONTROL_BIT (0x1 << 1)
struct rt5627_setup_data
{
	int i2c_bus;
	int i2c_address;
};


#define RT5627_PLL_FR_MCLK 1
#define RT5627_PLL_FR_BCLK 2

#define USE_DAPM_CTRL 0
#define REALTEK_HWDEP 0

#if REALTEK_HWDEP

#include <linux/ioctl.h>
#include <linux/types.h>

struct rt56xx_reg_state
{
	unsigned int reg_index;
	unsigned int reg_value;
};

struct rt56xx_cmd
{
	size_t number;
	struct rt56xx_reg_state __user *buf;		
};


enum 
{
	RT_READ_CODEC_REG_IOCTL = _IOR('R', 0x01, struct rt56xx_cmd),
	RT_READ_ALL_CODEC_REG_IOCTL = _IOR('R', 0x02, struct rt56xx_cmd),
	RT_WRITE_CODEC_REG_IOCTL = _IOW('R', 0x03, struct rt56xx_cmd),
};

#endif

#endif

