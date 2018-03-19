#
# Makefile for the Linux network IOT device drivers.
#

ifeq ($(CONFIG_MT7682),y)
obj-$(CONFIG_IOT) += iot_ether.o iot_ctrl.o iot_api.o
endif


obj-$(CONFIG_MT7687) += mtk/mt7687/
obj-$(CONFIG_MT7682) += mtk/mt7682/
obj-$(CONFIG_CC3200) += ti/cc3200/