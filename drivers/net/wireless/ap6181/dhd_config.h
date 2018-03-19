
#ifndef _dhd_config_
#define _dhd_config_

#include <bcmdevs.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <wlioctl.h>
#include <proto/802.11.h>

//#define FW_PATH_AUTO_SELECT 1
#ifndef CONFIG_BCMDHD_FW_PATH
#define CONFIG_BCMDHD_FW_PATH "/etc/firmware/fw_bcmdhd.bin"
#endif

#ifndef CONFIG_BCMDHD_NVRAM_PATH 
#define CONFIG_BCMDHD_NVRAM_PATH "/etc/firmware/nvram.txt"
#endif

#ifndef CONFIG_BCMDHD_CONFIG_PATH 
#define CONFIG_BCMDHD_CONFIG_PATH  "/etc/firmware/config.txt"
#endif

extern char firmware_path[MOD_PARAM_PATHLEN];
extern uint dhd_doflow;

/* channel list */
typedef struct wl_channel_list {
	/* in - # of channels, out - # of entries */
	uint32 count;
	/* variable length channel list */
	uint32 channel[WL_NUMCHANNELS];
} wl_channel_list_t;

typedef struct wmes_param {
	int aifsn[AC_COUNT];
	int cwmin[AC_COUNT];
	int cwmax[AC_COUNT];
} wme_param_t;

typedef struct dhd_conf {
	char fw_path[MOD_PARAM_PATHLEN];		/* Firmware path */
	char nv_path[MOD_PARAM_PATHLEN];		/* NVRAM path */
	uint band;			/* Band, b:2.4G only, otherwise for auto */
	wl_country_t cspec;		/* Country */
	wl_channel_list_t channels;	/* Support channels */
	uint roam_off;		/* Roaming, 0:enable, 1:disable */
	uint roam_off_suspend;		/* Roaming in suspend, 0:enable, 1:disable */
	int roam_trigger[2];		/* The RSSI threshold to trigger roaming */
	int roam_scan_period[2];	/* Roaming scan period */
	int roam_delta[2];			/* Roaming candidate qualification delta */
	int fullroamperiod;			/* Full Roaming period */
	uint filter_out_all_packets;	/* Filter out all packets in early suspend */
	uint keep_alive_period;		/* The perioid in ms to send keep alive packet */
	uint force_wme_ac;
	wme_param_t wme;	/* WME parameters */
} dhd_conf_t;

extern void *bcmsdh_get_drvdata(void);
void dhd_conf_set_fw_name_by_chip(dhd_pub_t *dhd, char *dst, char *src);
#if defined(HW_OOB)
void dhd_conf_set_hw_oob_intr(bcmsdh_info_t *sdh, uint chip);
#endif
void dhd_conf_set_fw_path(dhd_pub_t *dhd, char *fw_path);
void dhd_conf_set_nv_path(dhd_pub_t *dhd, char *nv_path);
int dhd_conf_set_band(dhd_pub_t *dhd);
uint dhd_conf_get_band(dhd_pub_t *dhd);
int dhd_conf_set_country(dhd_pub_t *dhd);
//int dhd_conf_get_country(dhd_pub_t *dhd); // rev 1.28.23.7
int dhd_conf_get_country(dhd_pub_t *dhd, wl_country_t *cspec); // rev 1.28.23.8
int dhd_conf_fix_country(dhd_pub_t *dhd);
bool dhd_conf_match_channel(dhd_pub_t *dhd, uint32 channel);
int dhd_conf_set_roam(dhd_pub_t *dhd);
void dhd_conf_set_bw(dhd_pub_t *dhd);
void dhd_conf_force_wme(dhd_pub_t *dhd);
void dhd_conf_get_wme(dhd_pub_t *dhd, edcf_acparam_t *acp);
void dhd_conf_set_wme(dhd_pub_t *dhd);
int dhd_conf_download_config(dhd_pub_t *dhd);
int dhd_conf_preinit(dhd_pub_t *dhd);
int dhd_conf_attach(dhd_pub_t *dhd);
void dhd_conf_detach(dhd_pub_t *dhd);

extern void *bcmsdh_get_drvdata(void);

#endif /* _dhd_config_ */
