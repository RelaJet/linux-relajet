#ifndef IOT_API_H
#define IOT_API_H
#include "iot.h"

struct iot_dev_ops{
	void* priv_data;
	int (*init)(void* priv_data);
	int (*get_mac)(char* mac);
	int (*write)(u32 data_type, void *data, int len);		
	int (*read)(u32 data_type, void *data, int len);			
	int (*eth_tx)(u32 data_type, void *data, int len);	
	int (*set_wifi_config)(iot_wifi_setting_t *wifisetting);
	int (*sleep)(void);
};
extern int iot_set_wifi(struct mt7687_data *mt_spi, iot_wifi_setting_t *wifisetting);
extern int iot_set_power(struct mt7687_data *mt_spi, u32 mode );
extern int iot_read_data (struct mt7687_data *mt_spi, u16 type, void *data, int len);
extern int iot_get_mac(struct mt7687_data *mt_spi, char *addr );
extern int iot_packet_send(struct mt7687_data *mt_spi, struct sk_buff *tx_skb );
extern void iot_init(void* priv_data);
extern int register_iot_dev(struct iot_dev_ops* ops);

#endif

