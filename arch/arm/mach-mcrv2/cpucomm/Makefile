#
# Makefile for the linux kernel.
#

obj-$(CONFIG_AIT_CPUB_BOOT_DRIVER) += cpub_mgr_drv.o
cpub_mgr_drv-objs += cpub_ringbuf.o cpub_mgr.o 
#cpub_cmd.o 
ifeq ($(CONFIG_AIT_CPUB_CMD_THD),m)
cpub_mgr_drv-objs += cpub_cmd.o
endif
ifeq ($(CONFIG_AIT_CPUB_CMD_THD),y)
cpub_mgr_drv-objs += cpub_cmd.o
endif
obj-$(CONFIG_AIT_CPUB_DRIVER_MD) += md_algo/
obj-$(CONFIG_AIT_CPUB_DRIVER_AAC) += aac_algo/
#obj-$(CONFIG_AIT_CPUB_DRIVER_AES) += aes_algo/
