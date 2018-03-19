# AIT8453 Platform Support

snd-soc-ait-pcm-objs := vsnv3-i2s-pcm.o
snd-soc-ait_ssc_dai-objs := ait_ssc_dai.o

obj-$(CONFIG_SND_AIT8455_SOC_ALSA) += snd-soc-ait_ssc_dai.o snd-soc-ait-pcm.o

obj-$(CONFIG_SND_VSNV3_SOC_AIT8455_WM8973) += ait8455evb_i2s_wm8973.o
obj-$(CONFIG_SND_VSNV3_SOC_AIT8455_RT5627) += ait8455evb_i2s_rt5627.o

# AIT8455 Machine Support
obj-$(CONFIG_SND_AIT8455_SOC_ALSA) += snd-soc-ait_ssc_dai.o ait_afe_dai.o
obj-$(CONFIG_SND_AIT8455_SOC_ALSA) += ait8455evb_afe.o vsnv3-afe-pcm.o



