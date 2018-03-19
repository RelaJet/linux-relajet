#ifndef IOT_API_H
#define IOT_API_H
#include "iot_ctrl.h"

struct iot_dev_ops{
	void* priv_data;
	int (*init)(void* priv_data);
	int (*get_mac)(int eth_if, char* mac);
	int (*write)(u32 data_type, void *data, int len);		
	int (*read)(u32 data_type, void *data, int len);			
	int (*eth_tx)(u32 data_type, void *data, int len);	
	int (*set_wifi_config)(iot_wifi_setting_t *wifisetting);
	int (*sleep)(void);
	struct sk_buff * (*fill_skb_header)(struct net_device *ndev, struct sk_buff_head *q)	;
};

extern int iot_set_wifi(iot_wifi_setting_t *wifisetting);
extern int iot_set_power( u32 mode );
extern int iot_get_mac(int eth_if, char *addr );
extern int iot_get_info(int eth_if, iot_dev_info_t *info );
extern int iot_get_linkstatus( u32 *link );
extern struct sk_buff *
iot_fill_skb_header(struct net_device *ndev, struct sk_buff_head *q);
extern int iot_packet_send(struct sk_buff *tx_skb );
extern int iot_read_eth_packet(void *data, int len);
extern void iot_init(void* priv_data);
extern int register_iot_dev(struct iot_dev_ops* ops);



#endif

