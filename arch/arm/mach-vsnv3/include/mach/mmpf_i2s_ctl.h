#ifndef	MMPF_I2S_CTL_H
#define	MMPF_I2S_CTL_H

//======================================//
// I2S_MASTER_SLAVE_MODE_SELECT_L : 0x48//
//======================================//
#define	I2S_SLAVE_MODE		0x00
#define	I2S_MASTER_MODE		0x01

//======================================//
// I2S_HCK_MODE_SELECT_L : 0x4C         //
//======================================//
#define	I2S_256_FS		0x04

//======================================//
// I2S_OUTPUT_BITS_SIZE_SETTING_L : 0x34//
//======================================//
//#define	I2S_OUTPUT_16_BITS		0x01
//#define	I2S_OUTPUT_24_BITS		0x02
//#define	I2S_OUTPUT_32_BITS		0x04

//============================//
// I2S_LRCK_POLARITY_L : 0x1C //
//============================//
#define	LEFT_CHANNEL_HIGH		0x01
#define	LEFT_CHANNEL_LOW		0x00

//=====================================//
// I2S_OUTPUT_DATA_DELAY_MODE_L : 0x60 //
//=====================================//
//#define	I2S_STD_MODE			0x01
//#define	I2S_I2S_MODE			0x00

void	   MMPF_I2S_PADSET(MMP_UBYTE ch, MMP_BOOL pad);
void    MMPF_SetI2SMode(MMP_USHORT i2s_mode, MMP_USHORT lrck_mode, MMP_USHORT output_bits);
void    MMPF_SetI2SFreq(MMP_ULONG freq);
void    MMPF_PresetI2S(MMP_USHORT direction,MMP_USHORT mode,MMP_USHORT alignment);
void    MMPF_CloseI2S(void);

#endif
