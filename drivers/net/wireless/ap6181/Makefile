# bcmdhd
# WL_SUPPORT_BACKPORTED_KPATCHES should be remove if kernel version is 3.0 and Android version is 4.1
DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER                \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
	-DDHDTHREAD -DDHD_DEBUG -DSDTEST -DBDC -DTOE                          \
	-DDHD_BCMEVENTS -DSHOW_EVENTS -DBCMDBG                                \
	-DMMC_SDIO_ABORT -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS             \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                \
	-DKEEP_ALIVE -DPKT_FILTER_SUPPORT                                     \
	-DEMBEDDED_PLATFORM -DENABLE_INSMOD_NO_FW_LOAD -DPNO_SUPPORT          \
	-DDHD_USE_IDLECOUNT -DSET_RANDOM_MAC_SOFTAP -DVSDB                    \
	-DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST -DSDIO_CRC_ERROR_FIX       \
	-DESCAN_RESULT_PATCH -DHT40_GO -DPASS_ARP_PACKET -DSUPPORT_PM2_ONLY   \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT -DAMPDU_HOSTREORDER       \
	-DCUSTOM_SDIO_F2_BLKSIZE=128 -DWL_SDO \
	-Idrivers/net/wireless/ap6181 -Idrivers/net/wireless/ap6181/include
#By Vincent: Remove -DWL_SUPPORT_BACKPORTED_KPATCHES from DHDCFLAGS
DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_sdio.o bcmwifi_channels.o bcmevent.o hndpmu.o       \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o dhd_config.o linux_osl.o   \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o

obj-$(CONFIG_BCMDHD) += bcm_wlan.o

bcm_wlan-objs += $(DHDOFILES)
DHDOFILES += dhd_gpio.o
ifeq ($(CONFIG_MACH_ODROID_4210),y)
DHDOFILES += dhd_gpio.o
#DHDCFLAGS += -Iarch/arm/plat-samsung/include
DHDCFLAGS += -Iinclude
DHDCFLAGS += -DCUSTOMER_HW
#DHDCFLAGS += -DBCMWAPI_WPI -DBCMWAPI_WAI
endif

ifeq ($(CONFIG_BCMDHD_OOB),y)
DHDCFLAGS += -DOOB_INTR_ONLY -DHW_OOB -DCUSTOMER_OOB
ifeq ($(CONFIG_BCMDHD_DISABLE_WOWLAN),y)
DHDCFLAGS += -DDISABLE_WOWLAN
endif
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

ifeq ($(CONFIG_BCMDHD_AG),y)
DHDCFLAGS += -DBAND_AG
endif

ifneq ($(CONFIG_WIRELESS_EXT),)
bcm_wlan-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DWL_WIRELESS_EXT -DUSE_IW
endif
ifneq ($(CONFIG_CFG80211),)
bcm_wlan-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWLP2P 
#By Vin: Remove from DHDCFLAGS
#-DWL_ENABLE_P2P_IF 
#For AP Mode security
#-DWL_CFG80211_STA_EVENT 
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
ifeq ($(CONFIG_BCMDHD),m)
#EXTRA_LDFLAGS += --strip-unneeded
endif
