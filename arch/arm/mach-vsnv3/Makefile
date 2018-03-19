#
# Makefile for the linux kernel.
#

obj-y		:= irq.o setup.o os_wrap.o
obj-m		:=
obj-n		:=
obj-		:=

ifeq ($(CONFIG_ARCH_VSNV3AIT845X),y)
ccflags-y += -Iarch/arm/mach-vsnv3/include/mach/
endif

# CPU-specific support
obj-$(CONFIG_ARCH_VSNV3AIT845X)	+= ait8x_arch.o ait_plat_devices.o ait_if.o
obj-$(CONFIG_ARCH_VSNV3AIT845X)	+= ait_time.o ait_hrtime.o

obj-$(CONFIG_ARCH_VSNV3AIT845X)	+= mmpf_i2cm.o mmpf_pio.o mmpf_vif.o mmpf_wd.o mmpf_system.o \
                                    mmpf_pll.o mmpf_timer.o mmpf_uart.o mmpf_sd.o

obj-y				+= leds.o 
#gpio.o 
#mmpf_audio_ctl.o vsnv3_afe.o 
obj-$(CONFIG_AIT_CHIP_VSN_V3)+=vsnv3_gpio.o board-ait8455evb.o clock.o
obj-$(CONFIG_AIT_CHIP_MCR_V2)+=mcrv2_gpio.o mmpf_mcrv2_audio_ctl.o mcrv2_afe.o clock.o
obj-$(CONFIG_AIT_CHIP_MCR_V2_MP)+=mcrv2_gpio.o \
				  mcrv2_clock.o mmpf_spi.o mmpf_saradc.o mmpf_pwm.o mmpf_rtc.o 

ifeq ($(CONFIG_SOUND),y)
obj-y +=mmpf_mcrv2_audio_ctl.o mcrv2_afe.o mcrv2_i2s.o mmpf_mcrv2_i2s_ctl.o
endif
ifeq  ($(CONFIG_AIT_MCRV2_DUAL_OS_DISABLE),y)
	obj-$(CONFIG_MACH_MCRV2_AIT8428EVB)+=board-ait8428evb.o
	obj-$(CONFIG_MACH_MCRV2_LOT8428EVB)+=board-krypto.o
else
ifeq ($(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA),y)
obj-y +=board-ait8428evb_cpua.o
obj-y += cpucomm/
endif #CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA

ifeq ($(CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUB),y)
obj-y +=board-ait8428evb_cpub.o

endif
endif

ifeq ($(CONFIG_SOUND),y)
obj-y +=mmpf_mcrv2_audio_ctl.o mcrv2_afe.o mcrv2_i2s.o mmpf_mcrv2_i2s_ctl.o
endif

obj-$(CONFIG_MACH_MCRV2_LOT8428EVB)+=board-krypto.o


# Power Management
obj-$(CONFIG_PM)	+= pm.o
#obj-$(CONFIG_CPU_IDLE)	+= cpuidle.o

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
