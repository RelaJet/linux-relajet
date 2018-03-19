# Note: the following conditions must always be true:
#   ZRELADDR == virt_to_phys(TEXTADDR)
#   PARAMS_PHYS must be within 4MB of ZRELADDR
#   INITRD_PHYS must be in RAM

ifeq ($(CONFIG_PHYS_OFFSET),0x03000000)
    zreladdr-y	+= 0x03008000

params_phys-y	:= 0x00104000
initrd_phys-y	:= 0x70410000
ULOADADDR:= 0x4000000
else
   zreladdr-y	+= 0x01008000
params_phys-y	:= 0x00100100
initrd_phys-y	:= 0x20410000
ifeq ($(CONFIG_AIT_FAST_BOOT),y)
ULOADADDR:= 0x1008000
else
ULOADADDR:= 0x2000000
endif
endif

dtb-$(CONFIG_MACH_AT91SAM_DT) += at91sam9m10g45ek.dtb usb_a9g20.dtb
