/*
 * drivers/usb/musb/ux500_dma.c
 *
 * U8500 and U5500 DMA support code
 *
 * Copyright (C) 2009 STMicroelectronics
 * Copyright (C) 2011 ST-Ericsson SA
 * Authors:
 *	Mian Yousaf Kaukab <mian.yousaf.kaukab@stericsson.com>
 *	Praveena Nadahally <praveen.nadahally@stericsson.com>
 *	Rajaram Regupathy <ragupathy.rajaram@stericsson.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//#define DEBUG
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pfn.h>
#include <linux/platform_data/ait_usb_dma.h>

#include <mach/mmp_reg_usb.h>
#include "musb_core.h"

struct ait_usb_dma_channel {
	struct dma_channel channel;
	struct ait_usb_dma_controller *controller;
	struct musb_hw_ep *hw_ep;
	struct work_struct channel_work;
	struct dma_chan *dma_chan;
	unsigned int cur_len;
	dma_cookie_t cookie;
	u8 ch_num;
	u8 is_tx;
	u8 is_allocated;
};

struct ait_usb_dma_controller {
	struct dma_controller controller;
	struct ait_usb_dma_channel rx_channels[AIT_USB_DMA_NUM_RX_CHANNELS];
	struct ait_usb_dma_channel tx_channels[AIT_USB_DMA_NUM_TX_CHANNELS];
	u32	num_rx_channels;
	u32	num_tx_channels;
	void *private_data;
	dma_addr_t phy_base;
	int irq;
};
#if 1
struct dma_desc_list
{
    u32 dwNextDescAddr;
    u32 dwPLDAddr;
    u16 wPLDSize;	
    u16 wPara;
	
	#define LIST_TXPKTRDY (1<<15)
	#define LIST_LAST (1<<14)	
	#define LIST_INDEX (1<<13)		
	#define LIST_FIX_DATA (1<<12)
} ;

struct dma_desc_list *pUsbDmaDesc;
struct dma_desc_list *UsbDmaDesc;//[100];
extern void MMPF_MMU_FlushDCacheMVA(MMP_ULONG ulRegion, MMP_ULONG ulSize);
void USBCore_dma_setlist(MMP_USHORT usSize, MMP_ULONG ulAddr, MMP_ULONG ulFbAddr, MMP_USHORT usPara)
{
    pUsbDmaDesc=(void*)ulAddr;

    if(usPara&LIST_LAST)    
       pUsbDmaDesc->dwNextDescAddr=0;
    else
	pUsbDmaDesc->dwNextDescAddr= virt_to_phys(pUsbDmaDesc+1);
//	pUsbDmaDesc->dwNextDescAddr=ulAddr + USBDMA_LIST_LEN;    
    
    pUsbDmaDesc->dwPLDAddr=ulFbAddr;
    pUsbDmaDesc->wPara=usPara;
    pUsbDmaDesc->wPLDSize=usSize-1; 

	MMPF_MMU_FlushDCacheMVA( (unsigned long)pUsbDmaDesc, sizeof( struct dma_desc_list) );

    #if 0
    pr_info("Addr: 0x%08x\n", ulAddr);
    pr_info("Next: 0x%08x\n", pUsbDmaDesc->dwNextDescAddr);
    RTNA_DBG_PrintLong(0, pUsbDmaDesc->dwPLDAddr);
    pr_info("Para: 0x%08x\n", pUsbDmaDesc->wPara);
    RTNA_DBG_PrintShort(0, pUsbDmaDesc->wPLDSize);
    #endif

   // pUsbDmaDesc++;
}


void USBCore_Dma1_Enable(MMP_ULONG ulAddr,MMP_UBYTE ubEnableInt, MMP_UBYTE ubEndpoint)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
//	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;

	pUSB_DMA->USB_DMA.INT_SR = 0x00;
//	pUSB_DMA->USB_DMA.CTL.INT_EN = 0x00; // ubEnableInt;
	//pUSB_DMA->USB_DMA3.CTL1 = 0x02;
//	RTNA_DBG_Str(0, "\r\n== DMA1 Enable ==\r\n");
	pUSB_DMA->USB_DMA.MODE.NORM.FB_ST_ADDR = virt_to_phys(ulAddr); // Descriptor List buffer address
	pUSB_DMA->USB_DMA.TAR_AND_VAL = 0xFF10FFFF;
	pUSB_DMA->USB_DMA.TAR_OR_VAL = 0x20A70000;
	pUSB_DMA->USB_DMA.MODE.NORM.CMD_ADDR = 0x1100|(ubEndpoint<<4);

#if 1//(TRY_DMA_INT_2)
//	pUSB_DMA->USB_DMA_INT_SR &= ~(USB_INT_DMA1_DONE_EN|USB_INT_DMA1_SET_TXPKTRDY);
	pUSB_DMA->USB_DMA_INT2_SR = 0x00;
	pUSB_DMA->USB_DMA_INT2_EN |= USB_INT_DMA1_DONE|USB_INT_DMA1_TXPKTRDY|USB_INT_DMA1_DESC_CNT; // ubEnableInt;
#else
//	pUSB_DMA->USB_DMA_INT_SR &= ~(USB_INT_DMA1_DONE_EN|USB_INT_DMA1_SET_TXPKTRDY);
	pUSB_DMA->USB_DMA.INT_SR = 0x00;
	pUSB_DMA->USB_DMA.CTL.INT_EN |= USB_INT_DMA1_DONE; // ubEnableInt;
#endif
 #define USB_DMA_EP(ep)          ((ep & 0x7) << 5)
	//DMA1 enable
	pUSB_DMA->USB_DMA.CTL1 = 0x01;//|USB_DMA_EP(1);
/*
    RTNA_DBG_Str(0, "EP_TX_FUNC_ADD");
    RTNA_DBG_Short(0, ubEndpoint);
    RTNA_DBG_Byte(0, pUSB_CTL->USB_MP_CTL[ubEndpoint].EP_TX_FUNC_ADDR);
    RTNA_DBG_Str(0, "\r\n");
*/
}


#if 0
void USBCore_ExtDmaDscrList(MMP_USHORT ep, MMP_ULONG fb_addr, MMP_USHORT pkt_byte, MMP_USHORT last_pkt_byte,
                    MMP_ULONG pkt_sum, MMP_ULONG zero_end)
{
    MMP_UBYTE i = 0x0;

    //RTNA_DBG_Str(0, "\r\n== Prepare EXTDMA Descriptor List ==\r\n");
      // printk(KERN_ALERT"-------------> %s,fb_addr:%x,pkt_byte:%d,last_pkt_byte:%d\n",__func__,fb_addr,pkt_byte,last_pkt_byte);
	for(i = 0x0; i < (pkt_sum-1); i++) {
		USBCore_dma_setlist(pkt_byte, UsbDmaDesc + i, fb_addr + i*(pkt_byte),
	                           LIST_TXPKTRDY|LIST_INDEX|ep);
	//                       LIST_TXPKTRDY|ep);
	}
	 
	USBCore_dma_setlist(last_pkt_byte, UsbDmaDesc+ i, fb_addr + i*(pkt_byte),
                             LIST_TXPKTRDY|LIST_LAST|ep);

}

#else
void USBCore_ExtDmaDscrList(MMP_USHORT ep, MMP_ULONG fb_addr, MMP_USHORT pkt_byte, MMP_USHORT last_pkt_byte,
                    MMP_ULONG pkt_sum, MMP_ULONG zero_end)
{
    MMP_UBYTE i = 0x0;

//    RTNA_DBG_Str(0, "\r\n== Prepare EXTDMA Descriptor List ==\r\n");
	//printk(KERN_ALERT"%s:pkt_byte:%d,pkt_sum:%d",__func__,pkt_byte,pkt_sum);
/*
	for(i = 0x0; i < (pkt_sum-1); i++) {
		USBCore_dma_setlist(pkt_byte, UsbDmaDesc + i, fb_addr + i*(pkt_byte),
	//                           LIST_TXPKTRDY|LIST_INDEX|ep);
	                       LIST_TXPKTRDY|ep);
	}
*/	 
	USBCore_dma_setlist(pkt_byte, UsbDmaDesc+ i, fb_addr + i*(pkt_byte),
                             LIST_TXPKTRDY|LIST_LAST|ep);

}
#endif
void USBCore_Dma1TxBuf_Config(MMP_USHORT ep, MMP_ULONG fb_addr,MMP_ULONG tx_byte)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    MMP_USHORT pkt_byte, last_pkt_byte;
    MMP_ULONG pkt_cnt;

    //printk(KERN_ALERT"%s---->%s  = = = >,tx_byte:%d\n",__FILE__,__func__,tx_byte);

//    pUSB_CTL->USB_MP_CTL[ep].EP_TX_FUNC_ADDR = otg_handle.dev_addr;
//    pUSB_CTL->USB_MP_CTL[ep].EP_RX_FUNC_ADDR = otg_handle.dev_addr;

#if 0// (USB_XFER_SPEED == USB_FULL_SPEED)
    pkt_byte = (tx_byte<0x40) ? 0 : 0x40;
//    last_pkt_byte = (tx_byte<0x40) ? (tx_byte) : 0x40;
    last_pkt_byte = (tx_byte&0x3F);
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x40 ;
    pkt_cnt = ((tx_byte+0x3F)>>6);
#endif
#if 0
    pkt_byte = (tx_byte<0x400) ? 0 : 0x400;
//    last_pkt_byte = (tx_byte<0x200) ? (tx_byte) : 0x200;
    last_pkt_byte = (tx_byte&0x3FF) ;
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x400 ;
    pkt_cnt = ((tx_byte+0x3FF)>>10);
#endif

#if 1
    pkt_byte = (tx_byte<0xC00) ? 0 : 0xC00;
//    last_pkt_byte = (tx_byte<0x200) ? (tx_byte) : 0x200;
    last_pkt_byte = (tx_byte&0xBFF) ;
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0xC00 ;
    pkt_cnt = (tx_byte+0xBFF)/(3<<10);

    //printk(KERN_ALERT"%s---->%s  = = = >,tx_byte:%d , pkt_cnt:%d \n",__FILE__,__func__,tx_byte,pkt_cnt);
#endif

    USBCore_ExtDmaDscrList(ep,fb_addr,tx_byte/*pkt_byte*/,/*last_pkt_byte*/0,/*pkt_cnt*/1,0x0);

    USBCore_Dma1_Enable(UsbDmaDesc, /*USB_DMA_DONE*/0, ep);

#if (POLLING_STATUS)
    MMPF_USBH_WaitDmaTxDone(ep, 1);
#endif
}

void USBCore_Dma2_Enable(MMP_ULONG ulAddr,MMP_UBYTE ubEnableInt, MMP_UBYTE ubEndpoint)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;

	RTNA_DBG_Str(0, "\r\n== DMA2 Enable ==\r\n");
	pUSB_DMA->USB_DMA2.MODE.NORM.FB_ST_ADDR = ulAddr; // Descriptor List buffer address
	pUSB_DMA->USB_DMA2.TAR_AND_VAL = 0xFF10FFFF;
	pUSB_DMA->USB_DMA2.TAR_OR_VAL = 0x00A70000;
	pUSB_DMA->USB_DMA2.MODE.NORM.CMD_ADDR = 0x1100|(ubEndpoint<<4);

	pUSB_DMA->USB_DMA.INT_SR &= ~(USB_INT_DMA2_DONE|USB_INT_DMA2_TXPKTRDY);
	pUSB_DMA->USB_DMA.CTL.INT_EN |= 0x08;

	//DMA2 enable
	pUSB_DMA->USB_DMA2.CTL1 |= 0x01;
}

void USBCore_Dma2TxBuf_Config(MMP_USHORT ep, MMP_ULONG fb_addr,MMP_ULONG tx_byte)
{
    AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
    MMP_USHORT pkt_byte, last_pkt_byte;
    MMP_ULONG pkt_cnt;

//    pUSB_CTL->USB_MP_CTL[ep].EP_TX_FUNC_ADDR = otg_handle.dev_addr;
//    pUSB_CTL->USB_MP_CTL[ep].EP_RX_FUNC_ADDR = otg_handle.dev_addr;

#if 0// (USB_XFER_SPEED == USB_FULL_SPEED)
    pkt_byte = (tx_byte<0x40) ? 0 : 0x40;
//    last_pkt_byte = (tx_byte<0x40) ? (tx_byte) : 0x40;
    last_pkt_byte = (tx_byte&0x3F);
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x40 ;
    pkt_cnt = ((tx_byte+0x3F)>>6);
#else
    pkt_byte = (tx_byte<0x200) ? 0 : 0x200;
//    last_pkt_byte = (tx_byte<0x200) ? (tx_byte) : 0x200;
    last_pkt_byte = (tx_byte&0x1FF) ;
    last_pkt_byte = (last_pkt_byte) ? (last_pkt_byte) : 0x200 ;
    pkt_cnt = ((tx_byte+0x1FF)>>9);
#endif

    USBCore_ExtDmaDscrList(ep,fb_addr,pkt_byte,last_pkt_byte,pkt_cnt,0x0);

    USBCore_Dma2_Enable(UsbDmaDesc, /*USB_DMA_DONE*/0, ep);

#if (POLLING_STATUS)
    MMPF_USBH_WaitDmaTxDone(ep, 2);
#endif
}
#endif
MMP_ERR MMPF_USBH_ConfigDma3Tx(MMP_UBYTE endpoint,MMP_ULONG fb_addr,MMP_USHORT pkt_byte,MMP_ULONG zero_end)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
	MMP_USHORT last_pkt_size;
	MMP_USHORT pkt_size;
	MMP_USHORT pkt_num;



	pkt_size = (pkt_byte<0x200) ? pkt_byte : 0x200;
//	last_pkt_size = (pkt_byte<=0x200) ? (pkt_byte) : (pkt_byte & 0x1FF);
	last_pkt_size = (pkt_byte%0x200) ? (pkt_byte & 0x1FF):0x200;
	pkt_num = ((pkt_byte+0x1FF)>>9);	
	
#if 0
	RTNA_DBG_Str(0, "\r\n========USB DMA3 TX CONFIG========\r\n");
	RTNA_DBG_PrintLong(0, endpoint);
	RTNA_DBG_PrintLong(0, fb_addr);
	RTNA_DBG_PrintLong(0, pkt_byte);
	RTNA_DBG_PrintLong(0, last_pkt_byte);
	RTNA_DBG_PrintLong(0, pkt_sum);
	RTNA_DBG_Str(0, "========USB DMA3 TX CONFIG========\r\n");
#endif
//	pUSB_DMA->USB_DMA3_CTL1 = 0x02;
	pUSB_DMA->USB_DMA3.MODE.NORM.FB_ST_ADDR = fb_addr;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE = pkt_size -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE_LAST = last_pkt_size -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_SUM = pkt_num -1;

	pUSB_DMA->USB_DMA3.TAR_AND_VAL = 0xFF10FFFF;

	pUSB_DMA->USB_DMA3.TAR_OR_VAL = 0x20A70000;
	pUSB_DMA->USB_DMA3.CTL2 = 2;	
	
	pUSB_DMA->LAST_TAR_AND_VAL= 0xFFFFFFFF;
	pUSB_DMA->LAST_TAR_OR_VAL = 0x20A70000;

	pUSB_DMA->USB_DMA.INT_SR = 0x00;
	pUSB_DMA->USB_DMA_INT2_SR = 0x00;

#if (1)
	pUSB_DMA->USB_DMA_INT2_EN |= USB_INT_DMA3_DONE;
#else
	pUSB_DMA->USB_DMA.CTL.INT_EN |= USB_INT_DMA3_DONE;
#endif
	pUSB_DMA->USB_DMA3.MODE.NORM.FIFO_ADDR = 0x1000+MUSB_FIFO_OFFSET(endpoint);
	pUSB_DMA->USB_DMA3.MODE.NORM.CMD_ADDR = 0x1000+MUSB_FLAT_OFFSET(endpoint,MUSB_TXMAXP);// + (<<4); 

	if(zero_end)
		pUSB_DMA->USB_DMA3.CTL1 =/*USB_DMA_EN|*/ USB_DMA_EP(endpoint) + USB_DMA_TX_ZLP;
	else
		pUSB_DMA->USB_DMA3.CTL1 =/*USB_DMA_EN |*/  USB_DMA_EP(endpoint);
//    pr_info("%d 0x%x\n",endpoint , fb_addr);
    return MMP_ERR_NONE;
}

MMP_ERR MMPF_USBH_ConfigDma3Rx(MMP_UBYTE endpoint,MMP_ULONG fb_addr,MMP_USHORT rx_byte,MMP_ULONG bPktReq)
	//(MMP_UBYTE endpoint,MMP_ULONG fb_addr,MMP_USHORT pkt_byte,MMP_USHORT last_pkt_byte,MMP_ULONG pkt_sum, MMP_BOOL bPktReq)
{
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
	MMP_USHORT pkt_byte;
	MMP_USHORT last_pkt_byte;
	MMP_ULONG pkt_cnt;

	pkt_byte = (rx_byte<0x200) ? (rx_byte) : 0x200;
	last_pkt_byte = (rx_byte<0x200) ? (rx_byte) : 0x200;
	pkt_cnt = ((rx_byte+0x1FF)>>9);

//	pr_info("en%d\n",endpoint);
//	pr_info("pkt_byte %d\n",rx_byte);	
#if 0	
	pr_info("addr 0x%x\n",fb_addr);

	

	pr_info("pkt_byte %d\n",pkt_byte);
	pr_info("last_pkt_byte %d\n",last_pkt_byte);
	pr_info("pkt_cnt %d\n",pkt_cnt);
#endif	
	pkt_byte = ALIGN32(pkt_byte);
	last_pkt_byte = ALIGN32(last_pkt_byte);
	
	pUSB_DMA->USB_DMA3.CTL1 = USB_DMA_STOP;
	pUSB_DMA->USB_DMA3.MODE.NORM.FB_ST_ADDR = fb_addr;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE = pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_BYTE_LAST = last_pkt_byte -1;
	pUSB_DMA->USB_DMA3.MODE.NORM.PKT_SUM = pkt_cnt -1;

	//CHIP_CORE_ID_MCR_V2_SHT: do (AND OR OP) after DMA done
	//CHIP_CORE_ID_MCR_V2_MP: do (AND OR OP) before DMA done
	bPktReq = 1;
	
	pUSB_DMA->USB_DMA3.TAR_AND_VAL = 0xF8200000;//0xF820FFFF;
	pUSB_DMA->USB_DMA3.TAR_OR_VAL = 0x00000000 | ((MMP_ULONG)bPktReq<<21)|512; // Set PktReq bit 
	
	pUSB_DMA->LAST_TAR_AND_VAL = 0xFFFFFFFF;//((~(MUSB_RXCSR_H_REQPKT| MUSB_RXCSR_H_AUTOREQ| MUSB_RXCSR_AUTOCLEAR))<<16)& 0xFFFFFFFF;
	pUSB_DMA->LAST_TAR_OR_VAL = 0;//MUSB_RXCSR_H_WZC_BITS<<16;//0x00000000;

	pUSB_DMA->USB_DMA_INT2_EN |= USB_INT_DMA3_DONE;
	
//	pUSB_DMA->USB_DMA.CTL.INT_EN |= USB_INT_DMA3_DONE; // Enable Int for dma3 done 

	pUSB_DMA->USB_DMA3.MODE.NORM.FIFO_ADDR = 0x1000+MUSB_FIFO_OFFSET(endpoint);
	pUSB_DMA->USB_DMA3.MODE.NORM.CMD_ADDR = 0x1000+MUSB_FLAT_OFFSET(endpoint,MUSB_RXMAXP);// + (<<4); 

	pUSB_DMA->USB_DMA3.CTL1= USB_DMA3_WR |USB_DMA_EP (endpoint); // Start USB EP DMA



	return MMP_ERR_NONE;
}

irqreturn_t ait_usbdma_irq_handle(int irq, void *dev_id)
{
	struct musb		*musb = dev_id;
	struct ait_usb_dma_controller		*ait_usbdma_controller;
	void __iomem		*tibase;
	//struct musb_hw_ep	*hw_ep = NULL;
	u32			rx, tx;
	int			i, index;
	unsigned long		uninitialized_var(flags);
	u8 usbdma_int_en,usbdma_int_sr;
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;


	spin_lock_irqsave(&musb->lock, flags);
		
	usbdma_int_en = pUSB_DMA->USB_DMA_INT2_EN;
	usbdma_int_sr = pUSB_DMA->USB_DMA_INT2_SR;

	//pr_info("usbdma_int_en = 0x%04x  usbdma_int_sr = 0x%04x\n",usbdma_int_en,usbdma_int_sr);
		
	usbdma_int_sr &=usbdma_int_en;

	if(usbdma_int_sr & USB_INT_DMA2_DONE)		
	{
		pUSB_DMA->USB_DMA_INT2_SR &=~USB_INT_DMA2_DONE;
		//pr_info("USBDMA 2 Done\n");
	}
	
	ait_usbdma_controller = container_of(musb->dma_controller, struct ait_usb_dma_controller, controller);

	if(usbdma_int_sr & USB_INT_DMA3_DONE)	{
			pUSB_DMA->USB_DMA3.CTL1 |= USB_DMA_STOP;


			pUSB_DMA->USB_DMA_INT2_SR &=~( USB_INT_DMA3_DONE);
#if 1
		if(ait_usbdma_controller->tx_channels->channel.status == MUSB_DMA_STATUS_BUSY)
		{

			struct musb_hw_ep	*hw_ep = ait_usbdma_controller->tx_channels->hw_ep;

			//void __iomem		*epio = hw_ep->regs;



			ait_usbdma_controller->tx_channels->channel.actual_len =ait_usbdma_controller->tx_channels->cur_len;
			ait_usbdma_controller->tx_channels->channel.status = MUSB_DMA_STATUS_FREE;


			//pr_info("tx=%d\n",ait_usbdma_controller->tx_channels->channel.actual_len );
			
			musb_host_tx(musb, hw_ep->epnum);
		}
#endif	
#if 0
		if(ait_usbdma_controller->rx_channels->channel.status == MUSB_DMA_STATUS_BUSY)
		{
			u16 *p;
			struct musb_hw_ep	*hw_ep = musb->endpoints + 4;

			void __iomem		*epio = hw_ep->regs;
			
				/* musb_ep_select(mbase, epnum); */
			u16 rx_count = musb_readw(epio, MUSB_RXCOUNT);
		
			if(rx_count >0xe00)
				ait_usbdma_controller->rx_channels->channel.actual_len =rx_count - 0xe00;//ait_usbdma_controller->rx_channels->cur_len;	
			else
			{
				ait_usbdma_controller->rx_channels->channel.actual_len =0;//ait_usbdma_controller->rx_channels->cur_len;	
			}
			p = (u32*)phys_to_virt(pUSB_DMA->USB_DMA3.MODE.NORM.FB_ST_ADDR);

			if((ait_usbdma_controller->rx_channels->channel.actual_len-8)!=*p)
			{
				pr_info("rx=%d %d\n",ait_usbdma_controller->rx_channels->channel.actual_len,*p );
				ait_usbdma_controller->rx_channels->channel.actual_len =0;
			}
			ait_usbdma_controller->rx_channels->channel.status = MUSB_DMA_STATUS_FREE;
			musb_host_rx(musb, 4);
		}
#endif		
	}
	
	if(usbdma_int_sr & USB_INT_DMA1_DONE)		
	{
		struct musb_hw_ep	*hw_ep = ait_usbdma_controller->tx_channels->hw_ep;
	
		pUSB_DMA->USB_DMA_INT2_SR &=~( USB_INT_DMA1_DONE|USB_INT_DMA1_TXPKTRDY|USB_INT_DMA1_DESC_CNT);


		if(ait_usbdma_controller->tx_channels->channel.status == MUSB_DMA_STATUS_BUSY)
		{	u16 tx_csr;
		
			ait_usbdma_controller->tx_channels->channel.actual_len =ait_usbdma_controller->tx_channels->cur_len;
			ait_usbdma_controller->tx_channels->channel.status = MUSB_DMA_STATUS_FREE;
			
			musb_ep_select(musb->mregs, hw_ep->epnum);
			/*
			tx_csr=musb_readw(hw_ep->regs, MUSB_TXCSR);
			tx_csr &= ~MUSB_TXCSR_P_UNDERRUN;
			musb_writew(hw_ep->regs, MUSB_TXCSR, tx_csr|MUSB_TXCSR_TXPKTRDY);
			*/
			//while((tx_csr = musb_readw(hw_ep->regs, MUSB_TXCSR))&(MUSB_TXCSR_FIFONOTEMPTY|MUSB_TXCSR_TXPKTRDY));



			//musb_g_tx(musb, hw_ep->epnum);

			//pr_info("tx=%d\n",ait_usbdma_controller->tx_channels->cur_len );		
		}

	}


//	if (ait_usbdma_controller->irq)
//		spin_unlock_irqrestore(&musb->lock, flags);
	spin_unlock_irqrestore(&musb->lock, flags);

	return IRQ_HANDLED;
}


/* Work function invoked from DMA callback to handle tx transfers. */
static void ait_usbdma_tx_work(struct work_struct *data)
{
	struct ait_usb_dma_channel *ait_usbdma_channel = container_of(data,
		struct ait_usb_dma_channel, channel_work);
	struct musb_hw_ep       *hw_ep = ait_usbdma_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	unsigned long flags;

	dev_info(musb->controller, "DMA tx transfer done on hw_ep=%d\n",
		hw_ep->epnum);

	spin_lock_irqsave(&musb->lock, flags);
	ait_usbdma_channel->channel.actual_len = ait_usbdma_channel->cur_len;
	ait_usbdma_channel->channel.status = MUSB_DMA_STATUS_FREE;
	musb_dma_completion(musb, hw_ep->epnum,ait_usbdma_channel->is_tx);
	spin_unlock_irqrestore(&musb->lock, flags);
}

/* Work function invoked from DMA callback to handle rx transfers. */
static void ait_usbdma_rx_work(struct work_struct *data)
{
	struct ait_usb_dma_channel *ait_usbdma_channel = container_of(data,
		struct ait_usb_dma_channel, channel_work);
	struct musb_hw_ep       *hw_ep = ait_usbdma_channel->hw_ep;
	struct musb *musb = hw_ep->musb;
	unsigned long flags;

	dev_info(musb->controller, "DMA rx transfer done on hw_ep=%d\n",
		hw_ep->epnum);

	spin_lock_irqsave(&musb->lock, flags);
	ait_usbdma_channel->channel.actual_len = ait_usbdma_channel->cur_len;
	ait_usbdma_channel->channel.status = MUSB_DMA_STATUS_FREE;
//todo
	musb_dma_completion(musb, hw_ep->epnum,ait_usbdma_channel->is_tx);//JJFFYY
	spin_unlock_irqrestore(&musb->lock, flags);
}

void ait_usbdma_callback(void *private_data)
{
	struct dma_channel *channel = (struct dma_channel *)private_data;
	struct ait_usb_dma_channel *ait_usbdma_channel = channel->private_data;

	schedule_work(&ait_usbdma_channel->channel_work);//JJFFYY
}

static bool ait_usbdma_configure_channel(struct dma_channel *channel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	struct ait_usb_dma_channel *ait_usbdma_channel = channel->private_data;
	struct musb_hw_ep *hw_ep = ait_usbdma_channel->hw_ep;
	struct dma_chan *dma_chan = ait_usbdma_channel->dma_chan;
	struct dma_async_tx_descriptor *dma_desc;
	enum dma_data_direction direction;
	struct scatterlist sg;
	struct dma_slave_config slave_conf;
	enum dma_slave_buswidth addr_width;
	dma_addr_t usb_fifo_addr = (MUSB_FIFO_OFFSET(hw_ep->epnum) +
					ait_usbdma_channel->controller->phy_base);
	struct musb *musb = ait_usbdma_channel->controller->private_data;
	u16 int_txe;
	void __iomem *musb_base = musb->mregs;

	//dev_info(musb->controller,
//		"packet_sz=%d, mode=%d, dma_addr=0x%x, len=%d is_tx=%d\n",
	//	packet_sz, mode, dma_addr, len, ait_usbdma_channel->is_tx);
//if(len>1564)
	//
	//pr_info("dma=%d\n",len);
#if 0 	
	sg_init_table(&sg, 1);
	sg_set_page(&sg, pfn_to_page(PFN_DOWN(dma_addr)), len,
					    offset_in_page(dma_addr));
	sg_dma_address(&sg) = dma_addr;
	sg_dma_len(&sg) = len;
#endif
	direction = ait_usbdma_channel->is_tx ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
	addr_width = (len & 0x3) ? DMA_SLAVE_BUSWIDTH_1_BYTE :
					DMA_SLAVE_BUSWIDTH_4_BYTES;

	slave_conf.direction = direction;
	slave_conf.src_addr = usb_fifo_addr;
	slave_conf.src_addr_width = addr_width;
	slave_conf.src_maxburst = 16;
	slave_conf.dst_addr = usb_fifo_addr;
	slave_conf.dst_addr_width = addr_width;
	slave_conf.dst_maxburst = 16;

	if(ait_usbdma_channel->is_tx)
	{	
#if 1
		ait_usbdma_channel->cur_len = min(len,512*6);//512*4
		//ait_usbdma_channel->cur_len = min(len,512*4);//512*4
		USBCore_Dma1TxBuf_Config(hw_ep->epnum,  dma_addr ,ait_usbdma_channel->cur_len);		
#else		
		ait_usbdma_channel->cur_len = min(len,50*1024);

	//if(len>1564)
	//	pr_info("len = %d\n",len);
		MMPF_USBH_ConfigDma3Tx(hw_ep->epnum,dma_addr,ait_usbdma_channel->cur_len ,mode);
#endif

	}
	else
	{
		ait_usbdma_channel->cur_len = min(len,512);
	
		MMPF_USBH_ConfigDma3Rx(hw_ep->epnum,dma_addr,ait_usbdma_channel->cur_len,1);
	}
#if 0
	dma_chan->device->device_control(dma_chan, DMA_SLAVE_CONFIG,
					     (unsigned long) &slave_conf);

	dma_desc = dma_chan->device->
			device_prep_slave_sg(dma_chan, &sg, 1, direction,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!dma_desc)
		return false;

	dma_desc->callback = ait_usbdma_callback;
	dma_desc->callback_param = channel;

	ait_usbdma_channel->cookie = dma_desc->tx_submit(dma_desc);

	dma_async_issue_pending(dma_chan);
#endif

	return true;
}

static struct dma_channel *ait_usbdma_channel_allocate(struct dma_controller *c,
				struct musb_hw_ep *hw_ep, u8 is_tx)
{
	struct ait_usb_dma_controller *controller = container_of(c,
			struct ait_usb_dma_controller, controller);
	struct ait_usb_dma_channel *ait_usbdma_channel = NULL;
	struct musb *musb = controller->private_data;
//	u8 ch_num = hw_ep->epnum - 1;
	u32 max_ch;
//dev_info(musb->controller, "ait_usbdma_channel_allocate  \n");

	/* Max 8 DMA channels (0 - 7). Each DMA channel can only be allocated
	 * to specified hw_ep. For example DMA channel 0 can only be allocated
	 * to hw_ep 1 and 9.
	 */
//	if (ch_num > 7)
//		ch_num -= 8;

	if (!(/*hw_ep->epnum==4||*//*hw_ep->epnum==5*/hw_ep->epnum==1))// && hw_ep->epnum!=3 && hw_ep->epnum!=1)
		return NULL;
	
	max_ch = is_tx ? controller->num_tx_channels :
			controller->num_rx_channels;

	ait_usbdma_channel = is_tx ? &(controller->tx_channels[0]) :
				&(controller->rx_channels[0]) ;
//TTEESSTT
	/* Check if channel is already used. */
#ifdef CONFIG_MUSB_PIO_ONLY
	if (ait_usbdma_channel->is_allocated)
		return NULL;
#endif
	ait_usbdma_channel->hw_ep = hw_ep;
	ait_usbdma_channel->is_allocated = 1;

//	dev_dbg(musb->controller, "hw_ep=%d, is_tx=0x%x, channel=%d\n",
//		hw_ep->epnum, is_tx, ch_num);

	return &(ait_usbdma_channel->channel);
}

static void ait_usbdma_channel_release(struct dma_channel *channel)
{
	struct ait_usb_dma_channel *ait_usbdma_channel = channel->private_data;
	struct musb *musb = ait_usbdma_channel->controller->private_data;

	if (ait_usbdma_channel->is_allocated) {
		//dev_info(musb->controller, "Ch %d Release\n", ait_usbdma_channel->ch_num);		
		ait_usbdma_channel->is_allocated = 0;
		channel->status = MUSB_DMA_STATUS_FREE;
		channel->actual_len = 0;
	}
}

static int ait_usbdma_is_compatible(struct dma_channel *channel,
		u16 maxpacket, void *buf, u32 length)
{
//pr_info("%s %d\n",__func__, __LINE__);

	if ((maxpacket & 0x3)		||
		((int)buf & 0x3))	//||
		//(length < 512)		||
		//(length & 0x3))
	{
		pr_warn("Not compatible!maxpacket = %d   buf = %d  length=%d\n",maxpacket,buf, length);
		return false;
	}
	else
		return true;
}

static int ait_usbdma_channel_program(struct dma_channel *channel,
				u16 packet_sz, u8 mode,
				dma_addr_t dma_addr, u32 len)
{
	int ret;

//pr_info("ait_usbdma_channel_program  \n");

//printk(KERN_ALERT"%s------>%s  \n",__FILE__,__func__);

	BUG_ON(channel->status == MUSB_DMA_STATUS_UNKNOWN ||
		channel->status == MUSB_DMA_STATUS_BUSY);

	if (!ait_usbdma_is_compatible(channel, packet_sz, (void *)dma_addr, len))
		return false;

	channel->status = MUSB_DMA_STATUS_BUSY;
	channel->actual_len = 0;

	ret = ait_usbdma_configure_channel(channel, packet_sz, mode, dma_addr, len);
	if (!ret)
		channel->status = MUSB_DMA_STATUS_FREE;

	return ret;
}

static int ait_usbdma_channel_abort(struct dma_channel *channel)
{
	struct ait_usb_dma_channel *ait_usbdma_channel = channel->private_data;
	struct ait_usb_dma_controller *controller = ait_usbdma_channel->controller;
	struct musb *musb = controller->private_data;
	void __iomem *epio = musb->endpoints[ait_usbdma_channel->hw_ep->epnum].regs;
	u16 csr;

	dev_info(musb->controller, "Abort channel=%d, is_tx=%d\n",
		ait_usbdma_channel->ch_num, ait_usbdma_channel->is_tx);

	if (channel->status == MUSB_DMA_STATUS_BUSY) {
#if 1		
		if (ait_usbdma_channel->is_tx) {
			csr = musb_readw(epio, MUSB_TXCSR);
			csr &= ~(MUSB_TXCSR_AUTOSET |
				 MUSB_TXCSR_DMAENAB |
				 MUSB_TXCSR_DMAMODE);
			musb_writew(epio, MUSB_TXCSR, csr);
		} else {
			csr = musb_readw(epio, MUSB_RXCSR);
			csr &= ~(MUSB_RXCSR_AUTOCLEAR |
				 MUSB_RXCSR_DMAENAB |
				 MUSB_RXCSR_DMAMODE);
			musb_writew(epio, MUSB_RXCSR, csr);
		}
   
#endif
#if 0
		ait_usbdma_channel->dma_chan->device->
				device_control(ait_usbdma_channel->dma_chan,
					DMA_TERMINATE_ALL, 0);
#endif		
		channel->status = MUSB_DMA_STATUS_FREE;
	}
	return 0;
}

static int ait_usbdma_controller_stop(struct dma_controller *c)
{
	struct ait_usb_dma_controller *controller = container_of(c,
			struct ait_usb_dma_controller, controller);
	struct ait_usb_dma_channel *ait_usbdma_channel;
	struct dma_channel *channel;
	u8 ch_num;

//pr_info("ait_usbdma_controller_stop  \n");

	for (ch_num = 0; ch_num < controller->num_rx_channels; ch_num++) {
		channel = &controller->rx_channels[ch_num].channel;
		ait_usbdma_channel = channel->private_data;

		ait_usbdma_channel_release(channel);

		if (ait_usbdma_channel->dma_chan)
			dma_release_channel(ait_usbdma_channel->dma_chan);
	}

	for (ch_num = 0; ch_num < controller->num_tx_channels; ch_num++) {
		channel = &controller->tx_channels[ch_num].channel;
		ait_usbdma_channel = channel->private_data;

		ait_usbdma_channel_release(channel);

		if (ait_usbdma_channel->dma_chan)
			dma_release_channel(ait_usbdma_channel->dma_chan);
	}

	return 0;
}

static int ait_usbdma_controller_start(struct dma_controller *c)
{
	struct ait_usb_dma_controller *controller = container_of(c,
			struct ait_usb_dma_controller, controller);
	struct ait_usb_dma_channel *ait_usbdma_channel = NULL;
	struct musb *musb = controller->private_data;
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev->platform_data;
	struct ait_usbdma_board_data *data = plat->board_data;
	struct dma_channel *dma_channel = NULL;
	u32 ch_num;
	//u8 dir;
	u8 is_tx = 0;

	void **param_array;
	struct ait_usb_dma_channel *channel_array;
	u32 ch_count;
	void (*musb_channel_work)(struct work_struct *);
	dma_cap_mask_t mask;

	if ((data->num_rx_channels > AIT_USB_DMA_NUM_RX_CHANNELS) ||
		(data->num_tx_channels > AIT_USB_DMA_NUM_TX_CHANNELS))
		return -EINVAL;

	controller->num_rx_channels = data->num_rx_channels;
	controller->num_tx_channels = data->num_tx_channels;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* Prepare the loop for RX channels */
	channel_array = controller->rx_channels;
	ch_count = data->num_rx_channels;
	param_array = data->dma_rx_param_array;
	musb_channel_work = ait_usbdma_rx_work;
UsbDmaDesc = kmalloc(sizeof(struct dma_desc_list )*10,__GFP_DMA);
	//for (dir = 0; dir < 2; dir++) {
		for (ch_num = 0; ch_num < ch_count; ch_num++) {
			ait_usbdma_channel = &channel_array[ch_num];
			ait_usbdma_channel->controller = controller;
			ait_usbdma_channel->ch_num = ch_num;
			ait_usbdma_channel->is_tx = is_tx;

			dma_channel = &(ait_usbdma_channel->channel);
			dma_channel->private_data = ait_usbdma_channel;
			dma_channel->status = MUSB_DMA_STATUS_FREE;
			dma_channel->max_len = SZ_16M;
#if 0
		//	ux500_channel->dma_chan = dma_request_channel(mask,
			//				data->dma_filter,
				//			param_array[ch_num]);
			if (!ait_usbdma_channel->dma_chan) {
				ERR("Dma pipe allocation error dir=%d ch=%d\n",
					dir, ch_num);

				/* Release already allocated channels */
				ait_usbdma_controller_stop(c);

				return -EBUSY;
			}
#endif
		//	INIT_WORK(&ait_usbdma_channel->channel_work,
		//		musb_channel_work);
		}

	/* Prepare the loop for TX channels */
	channel_array = controller->tx_channels;
	ch_count = data->num_tx_channels;
	param_array = data->dma_tx_param_array;
	musb_channel_work = ait_usbdma_tx_work;
	is_tx = 1;

		ait_usbdma_channel = controller->tx_channels;
		ait_usbdma_channel->controller = controller;
		ait_usbdma_channel->ch_num = 1;
		ait_usbdma_channel->is_tx = 1;

		dma_channel = &(ait_usbdma_channel->channel);
		dma_channel->private_data = ait_usbdma_channel;
		dma_channel->status = MUSB_DMA_STATUS_FREE;
		dma_channel->max_len = SZ_16M;
	//}
	return 0;
}

void dma_controller_destroy(struct dma_controller *c)
{
	struct ait_usb_dma_controller *controller = container_of(c,
			struct ait_usb_dma_controller, controller);
	kfree(UsbDmaDesc);
	kfree(controller);
}

struct dma_controller *__init
dma_controller_create(struct musb *musb, void __iomem *base)
{
	struct ait_usb_dma_controller *controller;
	struct platform_device *pdev = to_platform_device(musb->controller);
	struct resource	*iomem;
	int			irq = platform_get_irq_byname(pdev, "usb_dma");

dev_info(musb->controller, "dma_controller_create  base=0x%08x\n",base);

	controller = kzalloc(sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return NULL;

	controller->private_data = musb;

	/* Save physical address for DMA controller. */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	controller->phy_base = (dma_addr_t) iomem->start;

dev_info(musb->controller, "dma_controller_create  base=0x%08x\n",base);

	controller->controller.start = ait_usbdma_controller_start;
	controller->controller.stop = ait_usbdma_controller_stop;
	controller->controller.channel_alloc = ait_usbdma_channel_allocate;
	controller->controller.channel_release = ait_usbdma_channel_release;
	controller->controller.channel_program = ait_usbdma_channel_program;
	controller->controller.channel_abort = ait_usbdma_channel_abort;
	controller->controller.is_compatible =ait_usbdma_is_compatible;
	if (irq > 0) {
		if (request_irq(irq, ait_usbdma_irq_handle, 0, "usb-dma", musb)) {
//			dev_err(dev, "request_irq %d failed!\n", irq);
			dma_controller_destroy(&controller->controller);
			return NULL;
		}
		controller->irq = irq;
	}
	return &controller->controller;
}
