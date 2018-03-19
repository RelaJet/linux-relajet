#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <asm/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include "rt5627.h"

#define RT5627_VERSION "ALSA 1.0.24 0.04"

struct rt5627_priv {
	unsigned int sysclk;
};
/*
 *   define a virtual reg for misc func
 *   bit0: hpl pga dapm control
 *   bit1: hpr pga dapm control
 */

struct rt5627_reg{

	u8 reg_index;
	u16 reg_value;

};

#define VIRTUAL_REG_FOR_MISC_FUNC 0x84
//static unsigned int reg84 = 0;

static const u16 rt5627_reg[RT5627_VENDOR_ID2 + 1] = {
	[0x00] = 0x0003,
	[0x02] = 0x9f9f,
	[0x04] = 0x9f9f,
	[0x08] = 0xc8c8,
	[0x0a] = 0xc8c8,
	[0x0c] = 0xffff,
	[0x16] = 0x0009,
	[0x1c] = 0x8004,
	[0x34] = 0x8000,
	[0x38] = 0x2000,
	[0x40] = 0x0100,
	[0x4A] = 0x0000,
	[0x68] = 0x100b,
	[0x7c] = 0x10ec,
	[0x7e] = 0x2700,
};
#if 0
static struct rt5627_reg init_data[] = {
	{RT5627_STEREO_DAC_VOL		,0x1010},//default stereo DAC volume to 0db
	{RT5627_OUTPUT_MIXER_CTRL	,0x0704},//default output mixer control
	{RT5627_GEN_CTRL			,0x0b00},//set Class D Vmid ratio is 1VDD and DAC have high pass filter
	{RT5627_AUDIO_DATA_CTRL		,0x8000},//set I2S codec to slave mode
	{RT5627_SPK_OUT_VOL			,0x8080},//default speaker volume to 0db 
	{RT5627_HP_OUT_VOL			,0x8888},//default HP volume to -12db	
};
#else
static struct rt5627_reg init_data[] = {
	{RT5627_STEREO_DAC_VOL		,0x1010},//default stereo DAC volume to 0db
	{RT5627_OUTPUT_MIXER_CTRL	,SPKOUT_N_SOUR_LN|SPKOUT_INPUT_SEL_SPKMIXER|SPK_VOL_DIFF_NEG_SIG_ENA},//,0x0704},//default output mixer control
	{RT5627_GEN_CTRL			,0x0b00},//set Class D Vmid ratio is 1VDD and DAC have high pass filter
	{RT5627_AUDIO_DATA_CTRL		,0x0000},//set I2S codec to master mode
	{RT5627_SPK_OUT_VOL			,0x0808},//default speaker volume to -12db 
	{RT5627_HP_OUT_VOL			,0x0808},//default HP volume to -12db	
	{RT5627_PWR_MANAG_ADD1		,0x8000},//jimmyhung
};

#endif
#define RT5627_INIT_REG_NUM ARRAY_SIZE(init_data)

#define rt5627_reset(c) snd_soc_write(c, RT5627_RESET, 0)

static int rt5627_init_reg(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < RT5627_INIT_REG_NUM; i++)
	{
		snd_soc_write(codec, init_data[i].reg_index, init_data[i].reg_value);
	}

	return 0;
}


/*
 *    RT5627 Controls
 */
static const char *rt5627_spk_l_source_sel[] = {"LPRN", "LPRP", "LPLN", "MM"};			/*0*/
static const char *rt5627_spk_source_sel[] = {"VMID", "HP Mixer", "Speaker Mixer"};		/*1*/
static const char *rt5627_hpl_source_sel[] = {"VMID", "HP Left Mixer"};				/*2*/
static const char *rt5627_hpr_source_sel[] = {"VMID", "HP Right Mixer"};			/*3*/
static const char *rt5627_classd_ratio_sel[] = {"2.25 VDD", "2.00 VDD", "1.75 VDD", "1.5 VDD","1.25 VDD", "1 VDD"};					/*4*/
static const char *rt5627_direct_out_sel[] = {"Normal", "Direct Out"};				/*5*/

static struct soc_enum rt5627_enum[] = {

	SOC_ENUM_SINGLE(RT5627_OUTPUT_MIXER_CTRL, 14, 4,rt5627_spk_l_source_sel),			/*0*/	
	SOC_ENUM_SINGLE(RT5627_OUTPUT_MIXER_CTRL, 10, 3, rt5627_spk_source_sel),			/*1*/
	SOC_ENUM_SINGLE(RT5627_OUTPUT_MIXER_CTRL, 9, 2, rt5627_hpl_source_sel),				/*2*/
	SOC_ENUM_SINGLE(RT5627_OUTPUT_MIXER_CTRL, 8, 2, rt5627_hpr_source_sel), 			/*3*/
	SOC_ENUM_SINGLE(RT5627_GEN_CTRL, 9, 6, rt5627_classd_ratio_sel),          				/*4*/
	SOC_ENUM_SINGLE(RT5627_OUTPUT_MIXER_CTRL, 1, 2, rt5627_direct_out_sel), 				/*5*/
	SOC_ENUM_SINGLE(RT5627_OUTPUT_MIXER_CTRL, 2, 2, rt5627_direct_out_sel), 				/*6*/	
};

static const struct snd_kcontrol_new rt5627_snd_controls[] = {
	SOC_ENUM("HP ENABLE", rt5627_enum[5]),	
	SOC_ENUM("SPK VOL DIFF", rt5627_enum[6]),			
	SOC_ENUM("Classd AMP Ratio", rt5627_enum[4]),
	SOC_ENUM("SEL HP R IN", rt5627_enum[3]),
	SOC_ENUM("SEL HP L IN", rt5627_enum[2]),
	SOC_ENUM("SEL SPK VOL IN", rt5627_enum[1]),
	SOC_ENUM("SPKL Source", rt5627_enum[0]),
	SOC_DOUBLE("SPK Playback Volume", RT5627_SPK_OUT_VOL, 8, 0, 31, 1),
	SOC_DOUBLE("SPK Playback Switch", RT5627_SPK_OUT_VOL, 15, 7, 1, 1),
	SOC_DOUBLE("HP Playback Volume", RT5627_HP_OUT_VOL, 8, 0, 31, 1),
	SOC_DOUBLE("HP Playback Switch", RT5627_HP_OUT_VOL, 15, 7, 1, 1),
	SOC_DOUBLE("AUXIN Playback Volume", RT5627_AUXIN_VOL, 8, 0, 31, 1),
	SOC_DOUBLE("LINEIN Playback Volume", RT5627_LINE_IN_VOL, 8, 0, 31, 1),
	SOC_DOUBLE("PCMIN Playback Volume", RT5627_STEREO_DAC_VOL, 8, 0, 63, 1),
	
};

static void hp_depop_mode2(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, 0x3e, 0x8000, 0x8000);
	snd_soc_update_bits(codec, 0x04, 0x8080, 0x8080);
	snd_soc_update_bits(codec, 0x3a, 0x0100, 0x0100);
	snd_soc_update_bits(codec, 0x3c, 0x2000, 0x2000);
	snd_soc_update_bits(codec, 0x3e, 0x0600, 0x0600);
	snd_soc_update_bits(codec, 0x5e, 0x0200, 0x0200);
	schedule_timeout_uninterruptible(msecs_to_jiffies(300));
//	snd_soc_update_bits(codec, 0x3a, 0x0030, 0x0030);
//	snd_soc_update_bits(codec, 0x5e, 0x0000, 0x0200);
}

static int hp_pga_event(struct snd_soc_dapm_widget *w, 
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int reg;

	reg = snd_soc_read(codec, VIRTUAL_REG_FOR_MISC_FUNC) & 0x3;
	if (reg != 0x3 && reg != 0)
		return 0;
	
	switch (event)
	{
		case SND_SOC_DAPM_POST_PMD:
			snd_soc_update_bits(codec, 0x3a, 0x0030, 0x0000);
			snd_soc_update_bits(codec, 0x04, 0x8080, 0x8080);
			snd_soc_update_bits(codec, 0x3e, 0x0600, 0x0000);
			break;
		case SND_SOC_DAPM_POST_PMU:	
			hp_depop_mode2(codec);
			snd_soc_update_bits(codec ,0x04, 0x8080, 0x0000 );
			break;
		default:
			return 0;
	}	
	return 0;
}

/*
 * _DAPM_ Controls
 */
/*left hp mixer*/
static const struct snd_kcontrol_new rt5627_left_hp_mixer_controls[] = {

SOC_DAPM_SINGLE("Left Linein Playback Switch", RT5627_LINE_IN_VOL, 15, 1, 1),
SOC_DAPM_SINGLE("Left Auxin Playback Switch", RT5627_AUXIN_VOL, 15, 1, 1),
SOC_DAPM_SINGLE("Left PCM Playback Switch", RT5627_STEREO_DAC_VOL, 15, 1, 1),

};

/*right hp mixer*/
static const struct snd_kcontrol_new rt5627_right_hp_mixer_controls[] = {

SOC_DAPM_SINGLE("Right Linein Playback Switch", RT5627_LINE_IN_VOL, 7, 1, 1),
SOC_DAPM_SINGLE("Right Auxin Playback Switch", RT5627_AUXIN_VOL, 7, 1, 1),
SOC_DAPM_SINGLE("Right PCM Playback Switch", RT5627_STEREO_DAC_VOL, 7, 1, 1),

};


/*spk mixer*/
static const struct snd_kcontrol_new rt5627_spk_mixer_controls[] = {

SOC_DAPM_SINGLE("Left Linein Playback Switch", RT5627_LINE_IN_VOL, 14, 1, 1),
SOC_DAPM_SINGLE("Right Linein Playback Switch", RT5627_LINE_IN_VOL, 6, 1, 1),
SOC_DAPM_SINGLE("Left Auxin Playback Switch", RT5627_AUXIN_VOL, 14, 1, 1),
SOC_DAPM_SINGLE("Right Auxin Playback Switch", RT5627_AUXIN_VOL, 6, 1, 1),
SOC_DAPM_SINGLE("Left PCM Playback Switch", RT5627_STEREO_DAC_VOL, 14, 1, 1),
SOC_DAPM_SINGLE("Right PCM Playback Switch", RT5627_STEREO_DAC_VOL, 6, 1, 1),

};

/*SPK Mux Out*/
static const struct snd_kcontrol_new rt5627_spk_mux_out_controls = 
SOC_DAPM_ENUM("Route", rt5627_enum[1]);

/*HPL Mux Out*/
static const struct snd_kcontrol_new rt5627_hpl_mux_out_controls = 
SOC_DAPM_ENUM("Route", rt5627_enum[2]);

/*HPR Mux Out*/
static const struct snd_kcontrol_new rt5627_hpr_mux_out_controls = 
SOC_DAPM_ENUM("Route", rt5627_enum[3]);

static const struct snd_soc_dapm_widget rt5627_dapm_widgets[] = {
/*Path before Hp mixer*/
SND_SOC_DAPM_INPUT("Left Line In"),
SND_SOC_DAPM_INPUT("Right Line In"),
SND_SOC_DAPM_INPUT("Left Auxin"),
SND_SOC_DAPM_INPUT("Right Auxin"),
SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback", RT5627_PWR_MANAG_ADD2, 9, 0),
SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback", RT5627_PWR_MANAG_ADD2, 8, 0),
SND_SOC_DAPM_PGA("Left Linein PGA", RT5627_PWR_MANAG_ADD3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Linein PGA", RT5627_PWR_MANAG_ADD3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left Auxin PGA", RT5627_PWR_MANAG_ADD3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Auxin PGA", RT5627_PWR_MANAG_ADD3, 4, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Left HP Mixer", RT5627_PWR_MANAG_ADD2, 5, 0, 
	&rt5627_left_hp_mixer_controls[0], ARRAY_SIZE(rt5627_left_hp_mixer_controls)),
SND_SOC_DAPM_MIXER("Right HP Mixer", RT5627_PWR_MANAG_ADD2, 4, 0,
	&rt5627_right_hp_mixer_controls[0], ARRAY_SIZE(rt5627_right_hp_mixer_controls)),
SND_SOC_DAPM_MIXER("SPK Mixer", RT5627_PWR_MANAG_ADD2, 3, 0, 
	&rt5627_spk_mixer_controls[0], ARRAY_SIZE(rt5627_spk_mixer_controls)),
/*HP mixer -->SPK out*/
SND_SOC_DAPM_MIXER("HP Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MUX("SPK Mux Out", SND_SOC_NOPM, 0, 0, &rt5627_spk_mux_out_controls),

SND_SOC_DAPM_PGA("Speaker", RT5627_PWR_MANAG_ADD3, 12, 0, NULL, 0),

SND_SOC_DAPM_PGA("SPK AMP", RT5627_PWR_MANAG_ADD2, 14, 0, NULL, 0),
SND_SOC_DAPM_OUTPUT("SPK"),

/*Path before HP Mux out*/
SND_SOC_DAPM_MUX("HPL Mux Out", SND_SOC_NOPM, 0, 0, &rt5627_hpl_mux_out_controls),
SND_SOC_DAPM_MUX("HPR Mux Out", SND_SOC_NOPM, 0, 0, &rt5627_hpr_mux_out_controls),

SND_SOC_DAPM_PGA_E("HPL Out PGA",VIRTUAL_REG_FOR_MISC_FUNC, 0, 0, NULL, 0, 
				hp_pga_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU ),
SND_SOC_DAPM_PGA_E("HPR Out PGA",VIRTUAL_REG_FOR_MISC_FUNC, 1, 0, NULL, 0,
				hp_pga_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU ),
/*hp out*/
SND_SOC_DAPM_OUTPUT("HP"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/*line3 input pga*/

	/*line in pga*/
	{"Left Linein PGA", NULL, "Left Line In"},
	{"Right Linein PGA", NULL, "Right Line In"},
	
	/*aux in pga*/
	{"Left Auxin PGA", NULL, "Left Auxin"},
	{"Right Auxin PGA", NULL, "Right Auxin"},

	/*left hp mixer*/

	{"Left HP Mixer", "Left Linein Playback Switch", "Left Linein PGA"},
	{"Left HP Mixer", "Left Auxin Playback Switch", "Left Auxin PGA"},
	{"Left HP Mixer", "Left PCM Playback Switch", "Left DAC"},

	/*right hp mixer*/

	{"Right HP Mixer", "Right Linein Playback Switch", "Right Linein PGA"},
	{"Right HP Mixer", "Right Auxin Playback Switch", "Right Auxin PGA"},
	{"Right HP Mixer", "Right PCM Playback Switch", "Right DAC"},

	/*spk mixer*/
	{"SPK Mixer", "Left Linein Playback Switch", "Left Linein PGA"},
	{"SPK Mixer", "Right Linein Playback Switch", "Right Linein PGA"},
	{"SPK Mixer", "Left Auxin Playback Switch", "Left Auxin PGA"},
	{"SPK Mixer", "Right Auxin Playback Switch", "Right Auxin PGA"},
	{"SPK Mixer", "Left PCM Playback Switch", "Left DAC"},
	{"SPK Mixer", "Right PCM Playback Switch", "Right DAC"},

	/*HP Mixer virtual*/
	{"HP Mixer", NULL, "Left HP Mixer"},
	{"HP Mixer", NULL, "Right HP Mixer"},

	/*SPK Mux Out*/
	{"SPK Mux Out", "HP Mixer", "HP Mixer"},
	{"SPK Mux Out", "Speaker Mixer", "SPK Mixer"},

	/*HPL Mux Out*/
	{"HPL Mux Out", "HP Left Mixer", "Left HP Mixer"},
	
	/*HPR Mux Out*/
	{"HPR Mux Out", "HP Right Mixer", "Right HP Mixer"},
	
	/*SPK Out PGA*/
	{"Speaker", NULL, "SPK Mux Out"},

	/*HPL Out PGA*/
	{"HPL Out PGA", NULL, "HPL Mux Out"},

	/*HPR Out PGA*/
	{"HPR Out PGA", NULL, "HPR Mux Out"},
	
	/*spk*/
	{"SPK AMP", NULL, "Speaker"},
	{"SPK", NULL, "SPK AMP"},

	/*HP*/
	{"HP", NULL, "HPL Out PGA"},
	{"HP", NULL, "HPR Out PGA"},
};



/*PLL divisors*/
struct _pll_div {
	u32 pll_in;
	u32 pll_out;
	u16 regvalue;
};

static const struct _pll_div codec_master_pll_div[] = {
	{  2048000,  8192000,	0x0ea0},		
	{  3686400,  8192000,	0x4e27},	
	{ 12000000,  8192000,	0x456b},   
	{ 13000000,  8192000,	0x495f},
	{ 13100000,  8192000,	0x0320},	
	{  2048000,  11289600,	0xf637},
	{  3686400,  11289600,	0x2f22},	
	{ 12000000,  11289600,	0x3e2f},   
	{ 13000000,  11289600,	0x4d5b},
	{ 13100000,  11289600,	0x363b},	
	{  2048000,  16384000,	0x1ea0},
	{  3686400,  16384000,	0x9e27},	
	{  4096000,  16384000,	0x5e24},	//jimmyhung
	{ 12000000,  16384000,	0x452b},   
	{ 13000000,  16384000,	0x542f},
	{ 13100000,  16384000,	0x03a0},	
	{  2048000,  16934400,	0xe625},
	{  3686400,  16934400,	0x9126},	
	{ 12000000,  16934400,	0x4d2c},   
	{ 13000000,  16934400,	0x742f},
	{ 13100000,  16934400,	0x3c27},			
	{  2048000,  22579200,	0x2aa0},
	{  3686400,  22579200,	0x2f20},	
	{ 12000000,  22579200,	0x7e2f},   
	{ 13000000,  22579200,	0x742f},
	{ 13100000,  22579200,	0x3c27},		
	{  2048000,  24576000,	0x2ea0},
	{  3686400,  24576000,	0xee27},
	{  4096000,  24576000,	0x5e22},	//jimmyhung
	{ 12000000,  24576000,	0x2915},   
	{ 13000000,  24576000,	0x772e},
	{ 13100000,  24576000,	0x0d20},	
};

static const struct _pll_div codec_slave_pll_div[] = {
	{  1024000,  16384000,  0x3ea0},	
	{  1411200,  22579200,	0x3ea0},
	{  1536000,  24576000,	0x3ea0},	
	{  2048000,  16384000,  0x1ea0},	
	{  2822400,  22579200,	0x1ea0},
	{  3072000,  24576000,	0x1ea0},
	{   705600,  11289600,	0x3ea0},
	{   705600,  8467200 , 	0x3ab0},
};

struct _coeff_div{
	u32 mclk;
	u32 rate;
	u16 fs;
	u16 regvalue;
};


static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{ 8192000,  8000, 256*4, 0x4000},
	{12288000,  8000, 384*4, 0x4004},

	/* 11.025k */
	{11289600, 11025, 256*4, 0x4000},
	{16934400, 11025, 384*4, 0x4004},

	/* 16k */
	{16384000, 16000, 256*4, 0x4000},
	{24576000, 16000, 384*4, 0x4004},
	/* 22.05k */
	{11289600, 22050, 256*2, 0x2000},
	{16934400, 22050, 384*2, 0x2004},
	{ 8467200, 22050, 384*1, 0x0004},

	/* 32k */
	{16384000, 32000, 256*2, 0x2000},
	{24576000, 32000, 384*2, 0x2004},

	/* 44.1k */
	{22579200, 44100, 256*2, 0x2000},
	/* 48k */
	{24576000, 48000, 256*2, 0x2000},
};

static int get_coeff(int mclk, int rate)
{
	int i;
	
	printk("get_coeff mclk=%d,rate=%d\n",mclk,rate);
	
	
	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}
	
	return -EINVAL;
}

static int rt5627_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{

	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	/*set master/slave interface*/
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK)
	{
		case SND_SOC_DAIFMT_CBM_CFM:
			iface = 0x0000;
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			iface = 0x8000;
			break;
		default:
			return -EINVAL;
	}

	/*interface format*/
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK)
	{
		case SND_SOC_DAIFMT_I2S:
			iface |= 0x0000;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			iface |= 0x0001;
			break;
		case SND_SOC_DAIFMT_DSP_A:
			iface |= 0x0002;
			break;
		case SND_SOC_DAIFMT_DSP_B:
			iface |= 0x0003;
			break;
		default:
			return -EINVAL;
	}

	/*clock inversion*/
	switch (fmt & SND_SOC_DAIFMT_INV_MASK)
	{
		case SND_SOC_DAIFMT_NB_NF:
			iface |= 0x0000;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			iface |= 0x0080;
			break;
		default:
			return -EINVAL;		
	}

	snd_soc_write(codec, RT5627_AUDIO_DATA_CTRL, iface);	
	return 0;
}


static int rt5627_pcm_hw_params(struct snd_pcm_substream *substream, 
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct rt5627_priv *rt5627 = snd_soc_codec_get_drvdata(codec);
	u16 iface = snd_soc_read(codec, RT5627_AUDIO_DATA_CTRL) & 0xfff3;
	int coeff = get_coeff(rt5627->sysclk, params_rate(params));

//	printk(KERN_DEBUG "enter %s\n", __func__);

	switch (params_format(params))
	{
		case SNDRV_PCM_FORMAT_S16_LE:
			break;
		case SNDRV_PCM_FORMAT_S20_3LE:
			iface |= 0x0004;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			iface |= 0x0008;
			break;
	}

	snd_soc_write(codec, RT5627_AUDIO_DATA_CTRL, iface);

//	snd_soc_update_bits(codec, 0x3a, 0x8000, 0x8000);
//	snd_soc_update_bits(codec, 0x3c, 0x0400, 0x0400);

	if (coeff >= 0)
		snd_soc_write(codec, RT5627_DAC_CLK_CTRL, coeff_div[coeff].regvalue);

	return 0;
}

static int rt5627_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rt5627_priv *rt5627 = snd_soc_codec_get_drvdata(codec);

	if ((freq >= (256 * 8000)) && (freq <= (512 *48000))) {
		rt5627->sysclk = freq;
		return 0;
	}
	
	printk(KERN_ERR "unsupported sysclk freq %d\n", freq);
	return 0;
}

static int rt5627_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,int source,unsigned int freq_in, unsigned int freq_out)
{
	int i;//, coeff;
	int ret = -EINVAL;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rt5627_priv *rt5627 = snd_soc_codec_get_drvdata(codec);

	if (pll_id < RT5627_PLL_FR_MCLK || pll_id > RT5627_PLL_FR_BCLK)
		return -EINVAL;


	if (!freq_in || !freq_out)
	{
		return 0;
	}
	
	if (RT5627_PLL_FR_MCLK == pll_id)
	{
		for (i = 0; i < ARRAY_SIZE(codec_master_pll_div); i ++)
		{
			if ((freq_in == codec_master_pll_div[i].pll_in) && (freq_out == codec_master_pll_div[i].pll_out))
			{
				rt5627->sysclk = freq_out;
				snd_soc_write(codec, RT5627_GLOBAL_CLK_CTRL, 0x0000);    			/*PLL source from MCLK*/
				snd_soc_write(codec, RT5627_PLL_CTRL, codec_master_pll_div[i].regvalue);   	/*set pll code*/
				snd_soc_update_bits(codec, RT5627_PWR_MANAG_ADD2, 0x1000, 0x1000);        	/*enable pll power*/
				ret = 0;
			}
		}
	}
	else     /*slave mode*/
	{
		for (i = 0; i < ARRAY_SIZE(codec_slave_pll_div); i ++)
		{
			if ((freq_in == codec_slave_pll_div[i].pll_in) && (freq_out == codec_slave_pll_div[i].pll_out))
			{
				snd_soc_write(codec, RT5627_GLOBAL_CLK_CTRL, 0x4000);    			/*PLL source from BCLK*/
				snd_soc_write(codec, RT5627_PLL_CTRL, codec_slave_pll_div[i].regvalue);  	 /*set pll code*/
				snd_soc_update_bits(codec, RT5627_PWR_MANAG_ADD2, 0x1000, 0x1000);       	 /*enable pll power*/
				ret = 0;
			}
		}
	}

	snd_soc_update_bits(codec, RT5627_GLOBAL_CLK_CTRL, 0x8000, 0x8000);

	return ret;
}


#define RT5627_RATES (SNDRV_PCM_RATE_8000_44100)	

#define RT5627_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)



static struct snd_soc_dai_ops rt5627_hifi_ops = {
	.hw_params = rt5627_pcm_hw_params,
	.set_fmt = rt5627_set_dai_fmt,
	.set_sysclk = rt5627_set_dai_sysclk,
	.set_pll = rt5627_set_dai_pll,
	
};



static struct snd_soc_dai_driver rt5627_dai = {
	.name = "rt5627_hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = RT5627_RATES,
		.formats = RT5627_FORMATS,},

		.ops = &rt5627_hifi_ops,
};


static int rt5627_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
	
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		
		break;
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		snd_soc_update_bits(codec, 0x02, 0x8080, 0x8080);
		snd_soc_update_bits(codec, 0x04, 0x8080, 0x8080);
		snd_soc_write(codec, 0x3e, 0x0000);
		snd_soc_write(codec, 0x3c, 0x0000);
		snd_soc_write(codec, 0x3a, 0x0000);
		break;	

	}
	codec->dapm.bias_level = level;
	return 0;
}



#if defined(CONFIG_SND_HWDEP)
#if REALTEK_HWDEP

#define RT_CE_CODEC_HWDEP_NAME "rt56xx hwdep "


static int rt56xx_hwdep_open(struct snd_hwdep *hw, struct file *file)
{
	printk("enter %s\n", __func__);
	return 0;
}

static int rt56xx_hwdep_release(struct snd_hwdep *hw, struct file *file)
{
	printk("enter %s\n", __func__);
	return 0;
}


static int rt56xx_hwdep_ioctl_common(struct snd_hwdep *hw, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rt56xx_cmd rt56xx;
	struct rt56xx_cmd __user *_rt56xx = arg;
	struct rt56xx_reg_state *buf;
	struct rt56xx_reg_state *p;
	struct snd_soc_codec *codec = hw->private_data;

	if (copy_from_user(&rt56xx, _rt56xx, sizeof(rt56xx)))
		return -EFAULT;
	buf = kmalloc(sizeof(*buf) * rt56xx.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt56xx.buf, sizeof(*buf) * rt56xx.number)) {
		goto err;
	}
	switch (cmd) {
		case RT_READ_CODEC_REG_IOCTL:
			for (p = buf; p < buf + rt56xx.number; p++)
			{
				p->reg_value = snd_soc_read(codec, p->reg_index);
			}
			if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
				goto err;
				
			break;
		case RT_WRITE_CODEC_REG_IOCTL:
			for (p = buf; p < buf + rt56xx.number; p++)
				snd_soc_write(codec, p->reg_index, p->reg_value);
			break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
	
}

static int rt56xx_codec_dump_reg(struct snd_hwdep *hw, struct file *file, unsigned long arg)
{
	struct rt56xx_cmd rt56xx;
	struct rt56xx_cmd __user *_rt56xx = arg;
	struct rt56xx_reg_state *buf;
	struct snd_soc_codec *codec = hw->private_data;
	int number = codec->reg_cache_size;
	int i;

	printk(KERN_DEBUG "enter %s, number = %d\n", __func__, number);	
	if (copy_from_user(&rt56xx, _rt56xx, sizeof(rt56xx)))
		return -EFAULT;
	
	buf = kmalloc(sizeof(*buf) * number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	for (i = 0; i < number; i++)
	{
		buf[i].reg_index = i << 1;
		buf[i].reg_value = snd_soc_read(codec, buf[i].reg_index);
	}
	if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * i))
		goto err;
	rt56xx.number = number;
	if (copy_to_user(_rt56xx, &rt56xx, sizeof(rt56xx)))
		goto err;
	kfree(buf);
	return 0;
err:
	kfree(buf);
	return -EFAULT;
	
}

static int rt56xx_hwdep_ioctl(struct snd_hwdep *hw, struct file *file, unsigned int cmd, unsigned long arg)
{
	if (cmd == RT_READ_ALL_CODEC_REG_IOCTL)
	{
		return rt56xx_codec_dump_reg(hw, file, arg);
	}
	else
	{
		return rt56xx_hwdep_ioctl_common(hw, file, cmd, arg);
	}
}

static int realtek_ce_init_hwdep(struct snd_soc_codec *codec)
{
	struct snd_hwdep *hw;
	struct snd_card *card = codec->card;
	int err;

	if ((err = snd_hwdep_new(card, RT_CE_CODEC_HWDEP_NAME, 0, &hw)) < 0)
		return err;
	
	strcpy(hw->name, RT_CE_CODEC_HWDEP_NAME);
	hw->private_data = codec;
	hw->ops.open = rt56xx_hwdep_open;
	hw->ops.release = rt56xx_hwdep_release;
	hw->ops.ioctl = rt56xx_hwdep_ioctl;
	return 0;
}

#endif
#endif

static int rt5627_remove(struct snd_soc_codec *codec)
{
	rt5627_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}


static int rt5627_suspend(struct snd_soc_codec *codec, pm_message_t mesg)
{
	rt5627_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int rt5627_resume(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, RT5627_PWR_MANAG_ADD3, PWR_MAIN_BIAS);
	snd_soc_write(codec, RT5627_PWR_MANAG_ADD2, PWR_VREF);

	hp_depop_mode2(codec);	

	rt5627_init_reg(codec);

	return 0;
}

static int rt5627_probe(struct snd_soc_codec *codec)
{
	int ret;

	pr_info("RT5627 Audio Codec %s", RT5627_VERSION);
	
	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_I2C);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	if(-ENXIO==rt5627_reset(codec))
	{
	 	return -ENXIO;
	}
	
	snd_soc_write(codec, RT5627_PWR_MANAG_ADD3, PWR_MAIN_BIAS);
	snd_soc_write(codec, RT5627_PWR_MANAG_ADD2, PWR_VREF | PWR_L_DAC_L_D_S | PWR_R_DAC_R_D_S);//jimmyhung enable LR DAC

	hp_depop_mode2(codec);	

	rt5627_init_reg(codec);


	codec->dapm.bias_level = SND_SOC_BIAS_STANDBY;
	
	
	snd_soc_add_controls(codec, rt5627_snd_controls,
			ARRAY_SIZE(rt5627_snd_controls));
	snd_soc_dapm_new_controls(&codec->dapm, rt5627_dapm_widgets,
			ARRAY_SIZE(rt5627_dapm_widgets));
	snd_soc_dapm_add_routes(&codec->dapm, audio_map,
			ARRAY_SIZE(audio_map));

#if defined(CONFIG_SND_HWDEP)
#if REALTEK_HWDEP

	realtek_ce_init_hwdep(codec);

#endif
#endif
	return ret;
}

static struct snd_soc_codec_driver soc_codec_dev_rt5627 = {
	.probe = 	rt5627_probe,
	.remove = 	rt5627_remove,
	.suspend = 	rt5627_suspend,
	.resume =	rt5627_resume,
	.set_bias_level = rt5627_set_bias_level,
	.reg_cache_size = RT5627_VENDOR_ID2 + 1,
	.reg_word_size = sizeof(u16),
	.reg_cache_default = rt5627_reg,
	.reg_cache_step = 2,
};


static int rt5627_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct rt5627_priv *rt5627;
	int ret;

	rt5627 = kzalloc(sizeof(struct rt5627_priv), GFP_KERNEL);
	if (rt5627 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, rt5627);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_rt5627,
			&rt5627_dai, 1);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register codec: %d\n", ret);
		kfree(rt5627);
	}

	return ret;
}

static int rt5627_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id rt5627_i2c_id[] = {
		{"rt5627", 0x19},
		{}
};

MODULE_DEVICE_TABLE(i2c, rt5627_i2c_id);
static struct i2c_driver rt5627_i2c_driver = {
	.driver = {
		.name = "rt5627-codec",
		.owner = THIS_MODULE,
	},
	.probe =    rt5627_i2c_probe,
	.remove =   rt5627_i2c_remove,
	.id_table = rt5627_i2c_id,
};

static int __init rt5627_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&rt5627_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "%s: can't add i2c driver", __func__);
		return ret;
	}

	return ret;
}

static void __exit rt5627_exit(void)
{
	//snd_soc_unregister_dai(&rt5627_dai);
	i2c_del_driver(&rt5627_i2c_driver);
}

module_init(rt5627_modinit);
module_exit(rt5627_exit);
MODULE_LICENSE("GPL");


