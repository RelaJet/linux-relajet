HOST_IF=sdio

ARCH=arm
export ARCH=arm


D_ARCH=__ARM__
CROSS_COMPILE=/opt/buildroot-vsnv3_2014q1/bin/arm-buildroot-linux-uclibcgnueabi-
LINUX_SRC=/home/vin/customer/linux/svn_working2/branches/linux-3.2.7_SkypeCam
PLATFORM_CFLAGS=
#INSTALL_PATH=.
#INSTALL_MODULE_NAME=wlan.ko
INSTALL_PATH=.
INSTALL_MODULE_NAME=

CONFIG_WIRELESS_EXT=y
CONFIG_CFG80211=y
CONFIG_BCMDHD_OOB=n

CONFIG_BCMDHD=y
CONFIG_BCMDHD_FW_PATH="/etc/firmware/fw_bcmdhd.bin"
CONFIG_BCMDHD_NVRAM_PATH ="/etc/firmware/nvram.txt"

CONFIG_BCMDHD_CONFIG_PATH = "/etc/firmware/config.txt"
CONFIG_BCMDHD_WEXT =y
CONFIG_WIRELESS_EXT=y
CONFIG_WEXT_PRIV=y
CONFIG_BCMDHD_SDIO_IRQ=y

CONFIG_WIFI_CONTROL_FUNC = y
CUSTOMER_HW2=y

# ---------------------------------------------------
# Tool settings
# ---------------------------------------------------
AS      = $(CROSS_COMPILE)as
LD      = $(CROSS_COMPILE)ld
CC      = $(CROSS_COMPILE)gcc
AR      = $(CROSS_COMPILE)ar
NM      = $(CROSS_COMPILE)nm
STRIP   = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
RANLIB  = $(CROSS_COMPILE)ranlib
SIZE    = $(CROSS_COMPILE)size
CPP     = $(CC) -E

# ---------------------------------------------------
# Directory List
# ---------------------------------------------------

# ---------------------------------------------------
# Objects List
# ---------------------------------------------------

# Export MGMT_OBJS, NIC_OBJS, CHIP_OBJS and OS_OBJS for Linux 2.4 & 2.6 sub-make

