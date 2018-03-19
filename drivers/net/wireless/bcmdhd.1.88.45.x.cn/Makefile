# bcmdhd
# 1. WL_ENABLE_P2P_IF must be removed if kernel version is 3.8 later, otherwise please add it.
# 2. WL_IFACE_COMB_NUM_CHANNELS must be added if Android version is 4.4 with Kernel version 3.0~3.4,
#    otherwise please remove it.
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DWLBTAMP -DBCMFILEIMAGE  \
	-DDHDTHREAD  -DSDTEST -DBDC -DTOE                          \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMDBG                                \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS             \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                \
	-DKEEP_ALIVE -DPKT_FILTER_SUPPORT                                     \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-DDHD_USE_IDLECOUNT -DSET_RANDOM_MAC_SOFTAP -DVSDB                    \
	-DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST -DSDIO_CRC_ERROR_FIX       \
	-DESCAN_RESULT_PATCH -DHT40_GO -DPASS_ARP_PACKET -DSUPPORT_PM2_ONLY   \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT -DAMPDU_HOSTREORDER       \
	-DCUSTOM_SDIO_F2_BLKSIZE=128 -DWLTDLS                        \
	-DPROP_TXSTATUS -DPROP_TXSTATUS_VSDB                                  \
	-DWL_SUPPORT_BACKPORTED_KPATCHES                                      \
	-Idrivers/net/wireless/bcmdhd.1.88.45.x.cn -Idrivers/net/wireless/bcmdhd.1.88.45.x.cn/include
ifeq ($(CONFIG_AIT_FAST_BOOT),)
#Vin Remove from DHDCFLAGS -DWL_SDO
DHDCFLAGS+= -DWL_SDO -DDHD_DEBUG
endif

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o bcmwifi_channels.o dhd_sdio.o bcmevent.o dhd_bta.o hndpmu.o  \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o dhd_wlfc.o dhd_ip.o linux_osl.o \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o \
	dhd_pno.o dhd_config.o

obj-$(CONFIG_BCMDHD_1_88_45) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
DHDOFILES += dhd_gpio.o
ifeq ($(CONFIG_MACH_ODROID_4210),y)
DHDOFILES += dhd_gpio.o
DHDCFLAGS += -Iarch/arm/plat-samsung/include
DHDCFLAGS += -DCUSTOMER_HW
#DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI
endif

ifeq ($(CONFIG_BCMDHD_1_88_45_INSERT_MAC),y)
DHDCFLAGS += -DGET_CUSTOM_MAC_ENABLE
DHDCFLAGS += -USET_RANDOM_MAC_SOFTAP
endif

ifeq ($(CONFIG_BCMDHD_1_88_45_PRIO_OVERRIDE),y)
DHDCFLAGS += -DPKTPRIO_OVERRIDE
endif

ifeq ($(CONFIG_BCMDHD_1_88_45_OOB),y)
DHDCFLAGS += -DOOB_INTR_ONLY -DHW_OOB -DCUSTOMER_OOB
ifeq ($(CONFIG_BCMDHD_DISABLE_WOWLAN),y)
DHDCFLAGS += -DDISABLE_WOWLAN
endif
#ifeq ($(CONFIG_BCMDHD_POWER_OFF_IN_SUSPEND),y)
#DHDCFLAGS += -DPOWER_OFF_IN_SUSPEND
#endif
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

ifeq ($(CONFIG_BCMDHD_AG),y)
DHDCFLAGS += -DBAND_AG
endif

ifeq ($(CONFIG_DHD_USE_STATIC_BUF),y)
DHDCFLAGS += -DSTATIC_WL_PRIV_STRUCT
endif

ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT -DUSE_IW
endif
ifneq ($(CONFIG_CFG80211),)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWLP2P -DWL_CFG80211_STA_EVENT -DWL_ENABLE_P2P_IF
#DHDCFLAGS += -DWL_IFACE_COMB_NUM_CHANNELS
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-65
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=28000
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=7
DHDCFLAGS += -DWL_SUPPORT_AUTO_CHANNEL
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
ifeq ($(CONFIG_BCMDHD_1_88_45),m)
EXTRA_LDFLAGS += --strip-debug
endif

#DHDCFLAGS += -DCUSTOM_SDIO_F2_BLKSIZE=512 -DDHD_FIRSTREAD=128 -DMAX_HDR_READ=128
#DHDCFLAGS += -DCUSTOM_MAX_TXGLOM_SIZE=16 -DCUSTOM_DEF_TXGLOM_SIZE=16
#DHDCFLAGS += -DBCMSDIOH_TXGLOM -DBCMSDIOH_TXGLOM_EXT -DCUSTOM_TXGLOM=1
#DHDCFLAGS += -DCUSTOM_GLOM_SETTING=1
#DHDCFLAGS += -DCONFIG_HIS -DPKT_STATICS
