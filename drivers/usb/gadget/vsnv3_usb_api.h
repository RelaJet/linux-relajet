#ifndef __VSNV3_USB_API_H__
#define __VSNV3_USB_API_H__
//Controller 
void vsnv3_usb_phy_init(struct vsnv3_udc *udc);
void vsnv3_usb_enable(void);
void vsnv3_usb_write_fifo(u8 ep_num, u8* buf,u32 len);
void vsnv3_usb_read_fifo(u8 ep_num, u8* buf,u32 len);
u16 vsnv3_usb_read_ep_rx_count(u8 ep_num);


u16 vsnv3_usb_read_ep_tx_csr(u8 ep_num);
void vsnv3_usb_write_ep_tx_csr(u8 ep_num,u16 csr);
u16 vsnv3_usb_read_ep_rx_csr(u8 ep_num);
void vsnv3_usb_write_ep_rx_csr(u8 ep_num,u16 csr);
u16 vsnv3_usb_read_ep0_csr(void);
void vsnv3_usb_write_ep0_csr(u16 csr);

//u16 vsnv3_usb_read_ep_csr(u8 ep_num);
//void vsnv3_usb_write_ep_csr(u8 ep_num,u16 csr);


// Protocol
int vsnv3_usb_read_request(struct usb_ctrlrequest *req);

void vsnv3_usb_std_setaddress(u8 addr);
void vsnv3_usb_reset();
void vsnv3_usb_SendStall(u8 ep_num);
void vsnv3_usb_SendDataEnd(u8 ep_num);
void vsnv3_usb_ep0_data_out_end(void);
void vsnv3_usb_ServicedRxPktRdy(void);
void vsnv3_usb_ServicedSetupEnd(void);
void vsnv3_usb_ep_clear_RxPktRdy(u8 ep_num);

void vsnv3_usb_tx_ready(u8 ep_num, u8 is_txend);


void vsnv3_usb_set_ep_max_tx_pktsize(u8 ep_num, u32 size);
void vsnv3_usb_set_ep_max_rx_pktsize(u8 ep_num, u32 size);
void vsnv3_usb_ep_enable(u8 ep_num,u8 is_tx,u16 max_pkt_size);
#endif
