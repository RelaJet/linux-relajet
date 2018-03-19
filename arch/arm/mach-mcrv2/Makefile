#
# Makefile for the linux kernel.
#

obj-y		:= irq.o setup.o os_wrap.o
obj-m		:=
obj-n		:=
obj-		:=

ccflags-$(CONFIG_ARCH_MCRV2) += -Iarch/arm/mach-mcrv2/include/mach/


# CPU-specific support
obj-$(CONFIG_ARCH_MCRV2)	+= ait8x_arch.o ait_plat_devices.o ait_if.o ait_time.o ait_hrtime.o \
				mmpf_i2cm.o mmpf_pio.o  mmpf_wd.o mmpf_system.o mmpf_pll.o \
				mmpf_timer.o mmpf_uart.o mmpf_sd.o mcrv2_gpio.o mcrv2_clock.o mmpf_spi.o \
				mmpf_saradc.o mmpf_pwm.o #mmpf_vif.o

obj-$(CONFIG_RTC_DRV_AIT8428) += mmpf_rtc.o

obj-$(CONFIG_AIT_FAST_BOOT)	+= ait_alsa_ipc.o



obj-$(CONFIG_LEDS)		+= leds.o 

ifeq ($(CONFIG_SOUND),y)
obj-$(CONFIG_SOUND) += mmpf_mcrv2_audio_ctl.o
obj-$(CONFIG_SND_SOC) += mmpf_mcrv2_audio_ctl.o mcrv2_afe.o mcrv2_afe.o mcrv2_i2s.o mmpf_mcrv2_i2s_ctl.o 
endif


obj-$(CONFIG_FB_AIT)+=mmpf_display.o

# Power Management
obj-$(CONFIG_PM)	+= pm.o
#obj-$(CONFIG_CPU_IDLE)	+= cpuidle.o

obj-$(CONFIG_AIT_DMA)	+= ait_dma_ioctl.o


ifeq ($(CONFIG_PM_DEBUG),y)
CFLAGS_pm.o += -DDEBUG
endif

#ifeq ( $(CONFIG_USB_MUSB_AIT),y | $(CONFIG_AIT_USB_OTG),y )
#obj-y += usb_otg/
#endif

ifeq ($(CONFIG_USB_MUSB_AIT),y)
obj-y += usb_otg/
endif

ifeq ($(CONFIG_AIT_USB_OTG),y)
obj-y += usb_otg/
endif


obj-$(CONFIG_MCRV2_AIT8428G_HDK) +=board-ait8428g_hdk.o

obj-$(CONFIG_MCRV2_AIT6366G_EVB) +=board-ait6366g_evb.o
obj-y += cpucomm/
