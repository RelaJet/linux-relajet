
#include <mach/mmp_register.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_usb.h>
#include "vsnv3_udc.h"

//#define _DBG
#ifdef VSNV3_UDC_DEBUG
#define _DBG printk
#else
#define _DBG
#endif

#define DBG0 //printk

//Controller
void vsnv3_usb_phy_init(struct vsnv3_udc *udc)
{
	AITPS_GBL   pGBL = AITC_BASE_GBL;
	AITPS_USB_CTL pUSB_CTL = udc->regs;

	udc->gadget.speed = USB_SPEED_FULL;

	//dev->pdata->phy_control(1);

	printk("Enable USB clock\n");
	//pGBL->GBL_CLK_DIS1 &= ~(GBL_CLK_USB_DIS);
	pGBL->GBL_CLK_DIS[1] &= ~(GBL_CLK_USB);
	udelay(1000);

	/*USB PHY0 Enable */
	printk("USB PHY0 Enable\n");
	pUSB_CTL->USB_POWER = 0x61;	// high speed enable

	/* Enable PHY */

	pUSB_CTL->USB_POWER &= ~0x40;

	udelay(10);

	//pUSB_CTL->USB_POWER |= 0x40;
	//udelay(10);
	pUSB_CTL->USB_RX_INT_EN = 0;//EP1_RX_INT_BIT| EP3_RX_INT_BIT;
	pUSB_CTL->USB_TX_INT_EN = 0;//EP0_INT_BIT |EP2_TX_INT_BIT;
	//pUSB_CTL->USB_INT_EVENT_EN = 0xff;
	pUSB_CTL->USB_INT_EVENT_EN = 	USB_INT_SUSPEND |
									USB_INT_RESUME |
									USB_INT_RESET |
									USB_INT_CONN |
									USB_INT_DISCON |
									USB_INT_SESS_REQ |
									USB_INT_VBUS_ERROR ;
}


void vsnv3_usb_enable(void)
{
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

	pUSB_CTL->USB_POWER |= 0x40;
	udelay(10);
}	



void vsnv3_usb_write_fifo(u8 ep_num, u8* buf,u32 len)
{
	int i;
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

	for(i=0;i<len;++i)
		pUSB_CTL->USB_FIFO_EP[ep_num].FIFO_B =  buf[i];
}

void vsnv3_usb_read_fifo(u8 ep_num, u8* buf,u32 len)
{
	int i;
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

	for(i=0;i<len;++i)
		buf[i] = pUSB_CTL->USB_FIFO_EP[ep_num].FIFO_B ;
}


u16 vsnv3_usb_read_ep_rx_count(u8 ep_num)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    u16 tmp = 0;


    //  pUSB_CTL->USB_INDEX_EP_SEL = endpoint;

    tmp = (pUSB_CTL->USB_EP[ep_num].USB_EP_COUNT & 0x1FFF);


    return tmp;
}




u16 vsnv3_usb_read_ep_tx_csr(u8 ep_num)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    return pUSB_CTL->USB_EP[ep_num].USB_EP_TX_CSR;
}

void vsnv3_usb_write_ep_tx_csr(u8 ep_num,u16 csr)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_EP[ep_num].USB_EP_TX_CSR = csr;
}



u16 vsnv3_usb_read_ep_rx_csr(u8 ep_num)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    return pUSB_CTL->USB_EP[ep_num].USB_EP_RX_CSR;
}

void vsnv3_usb_write_ep_rx_csr(u8 ep_num,u16 csr)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_EP[ep_num].USB_EP_RX_CSR = csr;
}

u16 vsnv3_usb_read_ep0_csr(void)
{
	return vsnv3_usb_read_ep_tx_csr(0);
}

void vsnv3_usb_write_ep0_csr(u16 csr)
{
	vsnv3_usb_write_ep_tx_csr(0,csr);
}


//read EP0 fifo
int vsnv3_usb_read_request(struct usb_ctrlrequest *req)
{
	unsigned char *outbuf = (unsigned char*)req;

	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
	int i;

	int bytes_read = 0;

	//udc_write(0, S3C2410_UDC_INDEX_REG);

	bytes_read = pUSB_CTL->USB_EP[0x0].USB_EP_COUNT; //point to EPO fifo

	DBG0( "%s: fifo_count=%d\n", __func__, bytes_read);

	if (bytes_read > sizeof(struct usb_ctrlrequest))
	{
		bytes_read = sizeof(struct usb_ctrlrequest);
		DBG0("usb error: EP0 FIFO overflow.\r\n");
	}
	//readsb(S3C2410_UDC_EP0_FIFO_REG + base_addr, outbuf, bytes_read);

	DBG0("EP0 Read FIFO:[");
	for(i=0;i<bytes_read ;++i)
	{
		*(outbuf +i) = pUSB_CTL->USB_FIFO_EP[0x0].FIFO_B;
		DBG0("0x%x ",*(outbuf+i));
	}
	DBG0("]\n");

	if(req)
	{
		_DBG( "%s: len=%d %02x:%02x {%x,%x,%x}\n", __func__,
				bytes_read, req->bRequest, req->bRequestType,
				req->wValue, req->wIndex, req->wLength);
	}
	return bytes_read;
	//return i;
}

void vsnv3_usb_std_setaddress(u8 addr)
{
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

	pUSB_CTL->USB_FADDR = addr;
}

void vsnv3_usb_reset(void)
{
	u16 csr;
	//vsnv3_usb_phy_init();
	csr = vsnv3_usb_read_ep_tx_csr(0x01);

	if((csr & 0x00FF) != 0){
		vsnv3_usb_write_ep_tx_csr(0x01, csr & 0xFF00);      
		csr = vsnv3_usb_read_ep_tx_csr(0x01);
	} 

	csr = csr & TXCSR_RW_MASK;
	vsnv3_usb_write_ep_tx_csr(0x01, (csr & (~TX_SENDSTALL_BIT)) | SET_TX_CLRDATATOG);



}	


void vsnv3_usb_SendStall(u8 ep_num)
{
    u16 csr;
    //_DBG("!!!!!StallTx\n");
    printk("EP%d Stall.\r\n",ep_num);
    csr = vsnv3_usb_read_ep0_csr()& TXCSR_RW_MASK;
    vsnv3_usb_write_ep0_csr(csr | SET_TX_SENDSTALL);
}

void vsnv3_usb_SendDataEnd(u8 ep_num)
{
	vsnv3_usb_write_ep0_csr(EP0_DATAEND_BIT | SET_EP0_SERVICED_RXPKTRDY);
}

void vsnv3_usb_ep0_data_out_end(void)
{
        vsnv3_usb_SendDataEnd(0);//UsbWriteEp0CSR(EP0_DATAEND_BIT | SET_EP0_SERVICED_RXPKTRDY);   // dont need SET_EP0_TXPKTRDY if zero-size data
}

void vsnv3_usb_ServicedRxPktRdy(void)
{
		vsnv3_usb_write_ep0_csr(SET_EP0_SERVICED_RXPKTRDY);	

}

void vsnv3_usb_ServicedSetupEnd(void)
{
	vsnv3_usb_write_ep0_csr(SET_EP0_SERVICED_SETUPEND);

}

void vsnv3_usb_ep_clear_RxPktRdy(u8 ep_num)
{
	u16 csr = vsnv3_usb_read_ep_rx_csr(ep_num);
	vsnv3_usb_write_ep_rx_csr(ep_num,(csr&RXCSR_RW_MASK)|CLEAR_RX_RXPKTRDY);
}

void vsnv3_usb_tx_ready(u8 ep_num, u8 is_txend)
{
	if(ep_num)
	{
		u16 csr = vsnv3_usb_read_ep_tx_csr(ep_num);
		csr = csr & TXCSR_RW_MASK;
		vsnv3_usb_write_ep_tx_csr(ep_num,csr|SET_TX_TXPKTRDY);
	}
	else
	{
		if(is_txend)
			vsnv3_usb_write_ep0_csr(EP0_DATAEND_BIT|SET_EP0_TXPKTRDY);
		else
			vsnv3_usb_write_ep0_csr(SET_EP0_TXPKTRDY);
	}
	
}

void vsnv3_usb_set_ep_max_tx_pktsize(u8 ep_num, u32 size)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_EP[ep_num].USB_EP_TX_MAXP = size;
}
void vsnv3_usb_set_ep_max_rx_pktsize(u8 ep_num, u32 size)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

    pUSB_CTL->USB_EP[ep_num].USB_EP_RX_MAXP = size;
}


void vsnv3_usb_ep_enable(u8 ep_num,u8 is_tx,u16 max_pkt_size)
{
	AITPS_GBL   pGBL = AITC_BASE_GBL;
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

	u16 csr;

	_DBG("%s:%d %s\n",__func__,ep_num,is_tx?"tx":"rx");
	csr = vsnv3_usb_read_ep_tx_csr(ep_num);

	if((csr & 0x00FF) != 0){
	    vsnv3_usb_write_ep_tx_csr(ep_num, csr & 0xFF00);      
	    csr = vsnv3_usb_read_ep_tx_csr(ep_num);
	} 

	csr = csr & TXCSR_RW_MASK;

	//usb_ep_init();
	//pUSB_CTL->USB_POWER |= 0x20; // high speed enable
	if(is_tx)
	{
		csr|=SET_TX_MODE;

		vsnv3_usb_set_ep_max_tx_pktsize(ep_num,max_pkt_size);
	

		pUSB_CTL->USB_TX_INT_EN |= 1<<ep_num;
	}		
	else
	{
		vsnv3_usb_set_ep_max_rx_pktsize(ep_num,max_pkt_size);
		pUSB_CTL->USB_RX_INT_EN |= 1<<ep_num;
	}

	vsnv3_usb_write_ep_tx_csr(ep_num, (csr & (~TX_SENDSTALL_BIT)) | SET_TX_CLRDATATOG);
	
}



