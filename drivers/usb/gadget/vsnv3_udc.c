#include <linux/err.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/usb/gadget.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/compiler.h>
#include <linux/clk.h>

#include <linux/debugfs.h>

#include <linux/module.h>
#include <linux/io.h>

#include <mach/mmp_register.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_usb.h>

#include "vsnv3_udc.h"
#include "vsnv3_usb_api.h"

#ifdef VDBG
#undef VDBG
#endif
#if defined(VSNV3_UDC_DEBUG)
#define VDBG dev_vdbg
#define _DBG printk
#else
#define VDBG
#define _DBG
#endif

#define DBG0 printk

#define DRIVER_NAME	"vsnv3_udc"
#define DRIVER_DESC	"AIT VSNV3 USB Device Controller Gadget"
#define DRIVER_VERSION	"3 Nov 2013"


/*-------------------------------------------------------------------------*/

/*
 * Every device has ep0 for control requests, plus up to 30 more endpoints,
 * in one of two types:
 *
 *   - Configurable:  direction (in/out), type (bulk, iso, etc), and endpoint
 *     number can be changed.  Names like "ep-a" are used for this type.
 *
 *   - Fixed Function:  in other cases.  some characteristics may be mutable;
 *     that'd be hardware-specific.  Names like "ep12out-bulk" are used.
 *
 * Gadget drivers are responsible for not setting up conflicting endpoint
 * configurations, illegal or unsupported packet lengths, and so on.
 */

static const char ep0name[] = "ep0-ctrl";

static const char *const ep_name [] = {
	ep0name,				/* everyone has ep0 */

	/* act like a net2280: high speed, six configurable endpoints */
	"ep1-io", "ep2-io", "ep3-io", "ep4-io", "ep5-io", "ep6-io","ep7-io",
};
#define VSNV3_UDC_ENDPOINTS	ARRAY_SIZE(ep_name)


static struct vsnv3_udc *the_controller;
static struct dentry *vsnv3_udc_dbgfs_root;
static const char gadget_name[] = DRIVER_NAME;

#define DEBUG_ERROR	0
#define DEBUG_NORMAL	1
#define DEBUG_VERBOSE	2

#if defined(VSNV3_UDC_DEBUG)
#define CONFIG_VSNV3_UDC_DEBUG
#endif

static u32 sof_cnt = 0;

#ifdef CONFIG_VSNV3_UDC_DEBUG
#define USB_VSNV3_DEBUG_LEVEL DEBUG_VERBOSE
static uint32_t vsnv3_udc_ticks= 0;
static int udc_pr(int level, const char *fmt, ...)
{
	static char printk_buf[1024];
	static long prevticks;
	static int invocation;
	va_list args;
	int len;

	if (level > USB_VSNV3_DEBUG_LEVEL)
		return 0;

	if (vsnv3_udc_ticks != prevticks) {
		prevticks = vsnv3_udc_ticks;
		invocation = 0;
	}

	len = scnprintf(printk_buf,
			sizeof(printk_buf), "%1lu.%02d USB: ",
			prevticks, invocation++);

	va_start(args, fmt);
	len = vscnprintf(printk_buf+len,
			sizeof(printk_buf)-len, fmt, args);
	va_end(args);

//	return printk(KERN_INFO "%s", printk_buf);
	return printk("%s", printk_buf);

}
#else
static int udc_pr(int level, const char *fmt, ...)
{
	return 0;
}
#endif //end of


static void done(struct vsnv3_ep *ep, struct vsnv3_request *req, int status)
{
	unsigned	stopped = ep->bIsStopped;
	struct vsnv3_udc	*udc = ep->udc;

	DBG0("done 0x%X\r\n",(u32)req);

	list_del_init(&req->queue);
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

#ifdef VSNV3_UDC_DEBUG
	if (status && status != -ESHUTDOWN)
	{
		VDBG(&the_controller->gadget.dev,"%s done %p, status %d\n", ep->ep.name, req, status);
	}
#endif

	ep->bIsStopped = 1;
	spin_unlock(&udc->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&udc->lock);
	ep->bIsStopped = stopped;

	/* ep0 is always ready; other endpoints need a non-empty queue */
//	if (list_empty(&ep->queue) && ep->int_mask != (1 << 0))
//		at91_udp_write(udc, AT91_UDP_IDR, ep->int_mask);
}


static void nuke(struct vsnv3_ep *ep, int status)
{
	struct vsnv3_request *req;

	/* terminate any request in the queue */
	ep->bIsStopped = 1;
	if (list_empty(&ep->queue))
		return;

	VDBG("%s %s\n", __func__, ep->ep.name);
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct vsnv3_request, queue);
		done(ep, req, status);
	}
}

void vsnv3_udc_ep0_chgage_state(const char *label,
			struct vsnv3_udc *udc, enum ep0_state stat)
{
	if(label)
	{
		VDBG(&udc->gadget.dev, "<%s> from %15s to %15s\n",
			label, ep0states_str[udc->ep0state], ep0states_str[stat]);
	}
	else
	{
		VDBG(&udc->gadget.dev, "from %15s to %15s\n",
			ep0states_str[udc->ep0state], ep0states_str[stat]);
	}
		
	if (udc->ep0state == stat)
		return;

	udc->ep0state = stat;
}




static inline int vsnv3_udc_write_ep_fifo(u8 ep_num, struct vsnv3_request *req,unsigned max_pkt_size)
{
	unsigned len = min(req->req.length - req->req.actual, max_pkt_size);
	u8 *buf = req->req.buf + req->req.actual;

	prefetch(buf);

//	printk("%s %d %d %d %d\n", __func__,
//		req->req.actual, req->req.length, len, req->req.actual + len);

	req->req.actual += len;

	DBG0("EP%d W:%d\r\n",ep_num,len);

	vsnv3_usb_write_fifo(ep_num,buf,len);
		
	return len;
}

/*
 * return:  0 = still running, 1 = completed, negative = errno
 */
static int vsnv3_udc_write_packet(struct vsnv3_ep *ep,
		struct vsnv3_request *req)
{
	unsigned	count;
	int		is_last;
	u8		idx;
	int		fifo_reg;
	u32		ep_csr;

	idx = ep->ep.address & 0x7F;

	count = vsnv3_udc_write_ep_fifo(idx, req, ep->ep.maxpacket);

	/* last packet is often short (sometimes a zlp) */
	if (count != ep->ep.maxpacket)
		is_last = 1;
	else if (req->req.length != req->req.actual || req->req.zero)
		is_last = 0;
	else
		is_last = 2;

	if (idx == 0)
	{
		//udc_pr(DEBUG_NORMAL,
		_DBG(	"Written ep%d %d.%d of %d b [last %d,z %d]\n",
			idx, count, req->req.actual, req->req.length,
			is_last, req->req.zero);
	}
	if (is_last) {

		if (idx == 0) {
			/* Reset signal => no need to say 'data sent' */
#if 0
			if (! (udc_read(S3C2410_UDC_USB_INT_REG)
					& S3C2410_UDC_USBINT_RESET))
				s3c2410_udc_set_ep0_de_in(base_addr);
#endif
			ep->udc->ep0state=EP0_IDLE;

			
		} else {
#if 0
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			ep_csr = udc_read(S3C2410_UDC_IN_CSR1_REG);
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			udc_write(ep_csr | S3C2410_UDC_ICSR1_PKTRDY,
					S3C2410_UDC_IN_CSR1_REG);
#endif

		}

		done(ep, req, 0);
		is_last = 1;
	} else {
#if 0	
		if (idx == 0) {
			/* Reset signal => no need to say 'data sent' */
			if (! (udc_read(S3C2410_UDC_USB_INT_REG)
					& S3C2410_UDC_USBINT_RESET))
				s3c2410_udc_set_ep0_ipr(base_addr);
		} else {
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			ep_csr = udc_read(S3C2410_UDC_IN_CSR1_REG);
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			udc_write(ep_csr | S3C2410_UDC_ICSR1_PKTRDY,
					S3C2410_UDC_IN_CSR1_REG);
		}
#endif		
	}

	vsnv3_usb_tx_ready(idx,is_last);
	return is_last;
}

static int vsnv3_udc_tx_packet(struct vsnv3_ep *ep,struct vsnv3_request *req)
{
	unsigned	count;
	int		is_last;
	u8		idx;
	int		fifo_reg;
	u32		ep_csr;

	idx = ep->ep.address & 0x7F;
	//need to check req->req.zero
	if(req->req.length == req->req.actual) //if no data need to be sent
	{
		done(ep, req, 0);
		return 1;
	}

	count = vsnv3_udc_write_ep_fifo(idx, req, ep->ep.maxpacket);
	vsnv3_usb_tx_ready(idx,is_last);
	return 0;
}

#if 0
static inline int vsnv3_udc_read_ep_fifo(u8 ep_num, struct vsnv3_request *req,unsigned max_pkt_size)
{
	int bufferspace;
	unsigned len;
	int i;
	u8 *buf = req->req.buf + req->req.actual;
	len = min(req->req.length - req->req.actual, max_pkt_size);
	req->req.actual += len;

	bufferspace = req->req.length - req->req.actual;
	if (!bufferspace) {
		udc_pr(DEBUG_NORMAL, "%s: buffer full!\n", __func__);
		return -1;
	}
	
	vsnv3_usb_read_fifo(ep_num,buf,len);
	len = min(len,16);
	_DBG("Data: ");
	for( i=0;i<len;++i)
	{
		_DBG("%x ",buf[i]);
	}
	_DBG("\n");
	//readsb(fifo + base_addr, buf, len);
	return len;
}
#else
static inline int vsnv3_udc_read_ep_fifo(u8 ep_num, struct vsnv3_request *req,unsigned max_pkt_size)
{
	int bufferspace;
	unsigned len;
	int i;
	u8 *buf;

	buf = req->req.buf + req->req.actual;
	len = min(req->req.length - req->req.actual, max_pkt_size);
	req->req.actual += len;

	bufferspace = req->req.length - req->req.actual;
	if (!bufferspace) {
		udc_pr(DEBUG_NORMAL, "%s: buffer full!\n", __func__);
		return -1;
	}

	vsnv3_usb_read_fifo(ep_num,buf,len);
	return len;
}
#endif



#if 0
/*
 * return:  0 = still running, 1 = queue empty, negative = errno
 */
static int vsnv3_udc_read_packet(struct vsnv3_ep *ep,
				 struct vsnv3_request *req)
{
	u8		*buf;
	u32		ep_csr;
	unsigned	bufferspace;
	int		is_last=1;
	unsigned	avail;
	int		fifo_count = 0;
	u32		idx;
	int		fifo_reg;

	idx = ep->ep.address & 0x7F;
	_DBG("vsnv3_udc_read_packet\n");
	if (!req->req.length)
		return 1;

//	udc_write(idx, S3C2410_UDC_INDEX_REG);

	fifo_count =  vsnv3_usb_read_ep_rx_count(idx);//s3c2410_udc_fifo_count_out();
	udc_pr(DEBUG_NORMAL, "%s fifo count : %d\n", __func__, fifo_count);

	if (fifo_count > ep->ep.maxpacket)
		avail = ep->ep.maxpacket;
	else
		avail = fifo_count;

	fifo_count = vsnv3_udc_read_ep_fifo(idx, req, avail);

	/* checking this with ep0 is not accurate as we already
	 * read a control request
	 **/
	if (idx != 0 && fifo_count < ep->ep.maxpacket) {
		is_last = 1;
		/* overflowed this request?  flush extra data */
		if (fifo_count != avail)
			req->req.status = -EOVERFLOW;
	} else {
		is_last = (req->req.length <= req->req.actual) ? 1 : 0;
	}

//	udc_write(idx, S3C2410_UDC_INDEX_REG);
//	fifo_count = vsnv3_usb_read_ep_rx_count(idx);//s3c2410_udc_fifo_count_out();

	/* Only ep0 debug messages are interesting */
	if (idx == 0)
	{
		fifo_count = vsnv3_usb_read_ep_rx_count(idx);
		udc_pr(DEBUG_VERBOSE, "%s fifo count : %d [last %d]\n",
			__func__, fifo_count,is_last);
	}
	if (is_last) {
		if (idx == 0) {
			//s3c2410_udc_set_ep0_de_out(base_addr);
			vsnv3_usb_ep0_data_out_end();
			ep->udc->ep0state = EP0_IDLE;
		} else {
#if 0		
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			ep_csr = udc_read(S3C2410_UDC_OUT_CSR1_REG);
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			udc_write(ep_csr & ~S3C2410_UDC_OCSR1_PKTRDY,
					S3C2410_UDC_OUT_CSR1_REG);
#endif			
			vsnv3_usb_ep_clear_RxPktRdy(idx);

			_DBG("clear ep%d csr = %x\n",idx,vsnv3_usb_read_ep_rx_csr(idx));
		}

//		s3c2410_udc_done(ep, req, 0);
		done(ep, req, 0);
	} else {
		if (idx == 0) {
			//s3c2410_udc_clear_ep0_opr(base_addr);
		} else {
#if 0		
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			ep_csr = udc_read(S3C2410_UDC_OUT_CSR1_REG);
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			udc_write(ep_csr & ~S3C2410_UDC_OCSR1_PKTRDY,
					S3C2410_UDC_OUT_CSR1_REG);
#endif			
		}
	}

	return is_last;
}
#else
/*
 * return:  0 = still running, 1 = queue empty, negative = errno
 */
static int vsnv3_udc_read_packet(struct vsnv3_ep *ep,
				 struct vsnv3_request *req)
{
	u8		*buf;
	u32		ep_csr;
	unsigned	bufferspace;
	int		is_last=1;
	int		fifo_count = 0;
	u32		idx;
	int		fifo_reg;
	u32		require = 0;
	u32		i=0;

	idx = ep->ep.address & 0x7F;
	_DBG("vsnv3_udc_read_packet\n");

	if (!req->req.length)
		return 1;

#ifdef VSNV3_UDC_DEBUG
	fifo_count =  vsnv3_usb_read_ep_rx_count(idx);//s3c2410_udc_fifo_count_out();
	DBG0("udc rx fifo %d bytes, require %d bytes\r\n",fifo_count,req->req.length);
#endif

	//if (fifo_count > ep->ep.maxpacket)
	//	avail = ep->ep.maxpacket;
	//else
	//	avail = fifo_count;

	//if(idx==0)
	if(1)
	{
		require = req->req.length - req->req.actual;
		require = (require > ep->ep.maxpacket) ?  ep->ep.maxpacket:require;

		//workaround : when TX ready has been set but fifo not ready
		//while(idx==0 && vsnv3_usb_read_ep_rx_count(0)<require)
		while(vsnv3_usb_read_ep_rx_count(idx)<require)
		{
			if(++i<100)
				udelay(20);
			else
			{
				require = vsnv3_usb_read_ep_rx_count(idx);
				DBG0("wait fifo count, timeout.\r\n");
			}
		}

		fifo_count = vsnv3_udc_read_ep_fifo(idx, req, require);
	}
	else
	{
		fifo_count =  vsnv3_usb_read_ep_rx_count(idx);
		if (fifo_count > ep->ep.maxpacket)
			require = ep->ep.maxpacket;
		else
			require = fifo_count;

		fifo_count = vsnv3_udc_read_ep_fifo(idx, req, require);
	}

	/* checking this with ep0 is not accurate as we already
	 * read a control request
	 **/
	if (idx != 0 && fifo_count < ep->ep.maxpacket) {
		is_last = 1; //we got last packet
		/* overflowed this request?  flush extra data */
		if (fifo_count != require)
			req->req.status = -EOVERFLOW;
	} else { //if EP0
		is_last = (req->req.length <= req->req.actual) ? 1 : 0;
	}

//	udc_write(idx, S3C2410_UDC_INDEX_REG);
//	fifo_count = vsnv3_usb_read_ep_rx_count(idx);//s3c2410_udc_fifo_count_out();

	/* Only ep0 debug messages are interesting */
	if (idx == 0)
	{
		//fifo_count = vsnv3_usb_read_ep_rx_count(idx);
		udc_pr(DEBUG_VERBOSE, "%s fifo count : %d [last %d]\n",
			__func__, vsnv3_usb_read_ep_rx_count(idx),is_last);
	}
	if (is_last) {
		if (idx == 0) {
			//s3c2410_udc_set_ep0_de_out(base_addr);
			vsnv3_usb_ep0_data_out_end();
			ep->udc->ep0state = EP0_IDLE;
		} else {
#if 0		
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			ep_csr = udc_read(S3C2410_UDC_OUT_CSR1_REG);
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			udc_write(ep_csr & ~S3C2410_UDC_OCSR1_PKTRDY,
					S3C2410_UDC_OUT_CSR1_REG);
#endif			
			vsnv3_usb_ep_clear_RxPktRdy(idx);

			_DBG("clear ep%d csr = %x\n",idx,vsnv3_usb_read_ep_rx_csr(idx));
		}

//		s3c2410_udc_done(ep, req, 0);
		done(ep, req, 0);
	} else {
		if (idx == 0) {
			//s3c2410_udc_clear_ep0_opr(base_addr);
		} else {
#if 0		
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			ep_csr = udc_read(S3C2410_UDC_OUT_CSR1_REG);
			udc_write(idx, S3C2410_UDC_INDEX_REG);
			udc_write(ep_csr & ~S3C2410_UDC_OCSR1_PKTRDY,
					S3C2410_UDC_OUT_CSR1_REG);
#endif			
		}
	}

	return is_last;
}
#endif 

static void vsnv3_udc_ep0_idle_handler(struct vsnv3_udc *udc,
					struct vsnv3_ep *ep,
					struct usb_ctrlrequest *req,
					u32 ep0csr)
{
	int len, ret, tmp;

	/* start control request? */
//	if (!(ep0csr & S3C2410_UDC_EP0_CSR_OPKRDY))
	if(unlikely(!(ep0csr & EP0_RXPKTRDY_BIT)))
	{
		if(udc->last_request == USB_REQ_SET_ADDRESS)
		{
			vsnv3_usb_std_setaddress(udc->address);
			udc->last_request = 0;	
		}			
		else
		{
			_DBG("%s: Handle rx ready but csr status wrong!\n");
		}
		
		
		return;
	}
	
	nuke( ep, -EPROTO);

	len = vsnv3_usb_read_request(req);
	if (len != sizeof(*req)) {
		_DBG( "setup begin: fifo READ ERROR"
			" wanted %d bytes got %d. Stalling out...\n",
			sizeof(*req), len);
		vsnv3_usb_SendStall(0);
		return;
	}

	_DBG( "bRequest = %d bRequestType %d wLength = %d\n",
	req->bRequest, req->bRequestType, req->wLength);

	/* cope with automagic for some standard requests. */
	udc->bIsReqStd = (unsigned)(req->bRequestType & USB_TYPE_MASK)	== USB_TYPE_STANDARD;
	udc->req_config = 0;
	udc->req_pending = 1;

	switch (req->bRequest) {
	case USB_REQ_SET_CONFIGURATION:
		printk( "USB_REQ_SET_CONFIGURATION ... \n");

		if (req->bRequestType == USB_RECIP_DEVICE) {
			udc->req_config = 1;
	//		s3c2410_udc_set_ep0_de_out(base_addr);
			//vsnv3_usb_SendDataEnd(0);
//			UsbWriteEp0CSR(EP0_DATAEND_BIT | SET_EP0_SERVICED_RXPKTRDY);   // dont need SET_EP0_TXPKTRDY if zero-size data

		}
		break;

	case USB_REQ_SET_INTERFACE:
		_DBG( "USB_REQ_SET_INTERFACE ... \n");

		if (req->bRequestType == USB_RECIP_INTERFACE) {
			udc->req_config = 1;
			vsnv3_usb_SendDataEnd(0);//s3c2410_udc_set_ep0_de_out(base_addr);
		}
		break;

	case USB_REQ_SET_ADDRESS:
		_DBG( "USB_REQ_SET_ADDRESS (%d)... \n",req->wValue & 0x7F);
		
		if (req->bRequestType == USB_RECIP_DEVICE) {
			vsnv3_usb_ServicedRxPktRdy();			
			tmp = req->wValue & 0x7F;
			udc->address = tmp;	// Set to register latter
			//udc_write((tmp | S3C2410_UDC_FUNCADDR_UPDATE),
			//		S3C2410_UDC_FUNC_ADDR_REG);
			udc->last_request = req->bRequest;
			vsnv3_usb_SendDataEnd(0);//s3c2410_udc_set_ep0_de_out(base_addr);
			return;
		}
		break;

	case USB_REQ_GET_STATUS:
		_DBG( "USB_REQ_GET_STATUS ... \n");
		//s3c2410_udc_clear_ep0_opr(base_addr);
		vsnv3_usb_ServicedRxPktRdy();


		if (udc->bIsReqStd) {
		//	if (!s3c2410_udc_get_status(udc, req)) {
		//		return;
		//	}
		}
		break;

	case USB_REQ_CLEAR_FEATURE:
		vsnv3_usb_ServicedRxPktRdy();//s3c2410_udc_clear_ep0_opr(base_addr);

		if (req->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (req->wValue != USB_ENDPOINT_HALT || req->wLength != 0)
			break;

		//gs3c2410_udc_set_halt(&udc->ep[req->wIndex & 0x7f].ep, 0);
		vsnv3_usb_SendDataEnd(0);//s3c2410_udc_set_ep0_de_out(base_addr);
		return;

	case USB_REQ_SET_FEATURE:
		vsnv3_usb_ServicedRxPktRdy();//s3c2410_udc_clear_ep0_opr(base_addr);

		if (req->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (req->wValue != USB_ENDPOINT_HALT || req->wLength != 0)
			break;

//		s3c2410_udc_set_halt(&udc->ep[req->wIndex & 0x7f].ep, 1);
		vsnv3_usb_SendDataEnd(0);//s3c2410_udc_set_ep0_de_out(base_addr);
		return;

	default:
		vsnv3_usb_ServicedRxPktRdy();//		s3c2410_udc_clear_ep0_opr(base_addr);
		break;
	}

	if (req->bRequestType & USB_DIR_IN)
		udc->ep0state = EP0_IN_DATA_PHASE;
	else
		udc->ep0state = EP0_OUT_DATA_PHASE;

	if (!udc->driver)
		return;

	/* deliver the request to the gadget driver */
	//DBG0("Send control request to gadget driver. \r\n");
	ret = udc->driver->setup(&udc->gadget, req);
	//DBG0("Returned from gadget driver. \r\n");
	if (ret < 0) {
		if (udc->req_config) {
			printk( "config change %02x fail %d?\n",
				req->bRequest, ret);
			return;
		}

		if (ret == -EOPNOTSUPP)
		{
			DBG0( "Operation not supported\n");
		}
		else
		{
			DBG0("dev->driver->setup failed. (%d)\n", ret);
		}

		udelay(5);
		vsnv3_usb_SendStall(0);//s3c2410_udc_set_ep0_ss(base_addr);
		vsnv3_usb_SendDataEnd(0);//s3c2410_udc_set_ep0_de_out(base_addr);
		udc->ep0state = EP0_IDLE;
		/* deferred i/o == no response yet */
	} else if (udc->req_pending) {
		if(req->bRequest==USB_REQ_SET_CONFIGURATION)
		{
			vsnv3_usb_SendDataEnd(0);		
			udc->ep0state = EP0_IDLE;
		}
		DBG0("dev->req_pending... what now?\n");
		udc->req_pending=0;
	}
	_DBG("ep0state %s\n", ep0states_str[udc->ep0state]);
}



static void vsnv3_udc_handle_ep0(struct vsnv3_udc *udc)
{
	u16			ep0csr;
	struct vsnv3_ep	*ep = &udc->ep[0];
	struct vsnv3_request	*req;
	struct usb_ctrlrequest	usbctlreq;

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct vsnv3_request, queue);

	ep0csr = vsnv3_usb_read_ep0_csr();//udc_read(S3C2410_UDC_IN_CSR1_REG);

	_DBG( "ep0csr 0x%x ep0state %s\n",
		ep0csr, ep0states_str[udc->ep0state]);

	/* clear stall status */
	if (ep0csr & EP0_SENTSTALL_BIT){//S3C2410_UDC_EP0_CSR_SENTSTL) {
		printk("EP0_SENTSTALL_BITx\n");

		vsnv3_usb_write_ep0_csr(CLEAR_EP0_SENTSTALL);

		//s3c2410_udc_nuke(dev, ep, -EPIPE);
		nuke(ep, -EPIPE);
		_DBG( "... clear SENT_STALL ...\n");
		//s3c2410_udc_clear_ep0_sst(base_addr);
		vsnv3_usb_SendStall(0);

		udc->ep0state = EP0_IDLE;
		return;
	}

	/* clear setup end */
	if (ep0csr & EP0_SETUPEND_BIT) {
		_DBG( "... serviced SETUP_END ...%d\n",sof_cnt);
		
		//nuke(ep, 0);//s3c2410_udc_nuke(dev, ep, 0);
		vsnv3_usb_ServicedSetupEnd();//s3c2410_udc_clear_ep0_se(base_addr);
		//udc->ep0state = EP0_IDLE;
	}

	switch (udc->ep0state) {
		case EP0_IDLE:
			vsnv3_udc_ep0_idle_handler(udc, ep, &usbctlreq, ep0csr);
			break;

		case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
			_DBG( "EP0_IN_DATA_PHASE ... what now?\n");
		//	if (!(ep0csr & S3C2410_UDC_EP0_CSR_IPKRDY) && req) {
				vsnv3_udc_write_packet(ep, req);
			//}
			break;

		case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
			_DBG( "EP0_OUT_DATA_PHASE ... what now?\n");
		//	if ((ep0csr & S3C2410_UDC_EP0_CSR_OPKRDY) && req ) {
			//	s3c2410_udc_read_fifo(ep,req);
			//}
			break;

		case EP0_END_XFER:
			_DBG( "EP0_END_XFER ... what now?\n");
			udc->ep0state = EP0_IDLE;
			break;

		case EP0_STALL:
			udc->ep0state = EP0_IDLE;
			vsnv3_udc_ep0_chgage_state(__func__,udc,EP0_IDLE);
			break;
	}
		
}


static void vsnv3_udc_handle_ep(struct vsnv3_ep *ep)
{
	struct vsnv3_request	*req;
	int			is_in = ep->ep.address & USB_DIR_IN;
	u32			csr;
	u32			idx;

	if (likely (!list_empty(&ep->queue)))
		req = list_entry(ep->queue.next,
				struct vsnv3_request, queue);
	else
		req = NULL;

	idx = ep->ep.address & 0x7F;

	udc_pr(DEBUG_VERBOSE, "ep%01d CSR = > rx %02x tx: %02x\n",
		idx, vsnv3_usb_read_ep_rx_csr(idx),vsnv3_usb_read_ep_tx_csr(idx));


	if (is_in) {
		//udc_write(idx, S3C2410_UDC_INDEX_REG);
//		ep_csr1 = udc_read(S3C2410_UDC_IN_CSR1_REG);
		//csr = vsnv3_usb_read_ep_csr(idx);
		csr = vsnv3_usb_read_ep_tx_csr(idx);
		
		udc_pr(DEBUG_VERBOSE, "ep%01d write csr:%02x %d\n",
			idx, csr, req ? 1 : 0);

		if (csr & TX_SENTSTALL_BIT) {//		if (csr & S3C2410_UDC_ICSR1_SENTSTL) {
			udc_pr(DEBUG_VERBOSE, "st\n");
			//udc_write(idx, S3C2410_UDC_INDEX_REG);
			//udc_write(ep_csr1 & ~S3C2410_UDC_ICSR1_SENTSTL,
			//		S3C2410_UDC_IN_CSR1_REG);
			udc_pr(DEBUG_VERBOSE,"EP%d STALL \r\n",idx);
			return;
		}

		//if (!(csr & S3C2410_UDC_ICSR1_PKTRDY) && req) {
		//if ((csr & TX_TXPKTRDY_BIT) && req) {
		if ( ((csr&TX_TXPKTRDY_BIT)==0) && req) {	//20140326 ANDY
			udc_pr(DEBUG_VERBOSE, "TX_TXPKTRDY_BIT\n");			
			//vsnv3_udc_write_packet(ep,req);
			if( vsnv3_udc_tx_packet(ep,req) && likely (!list_empty(&ep->queue)))
			{//if current packet finish , and queue is not empty
				req = list_entry(ep->queue.next, struct vsnv3_request, queue);
				vsnv3_udc_tx_packet(ep,req);
			}
		}
	} else {
		//udc_write(idx, S3C2410_UDC_INDEX_REG);
		csr = vsnv3_usb_read_ep_rx_csr(idx);
		udc_pr(DEBUG_VERBOSE, "ep%01d rd csr:%02x\n", idx, csr);

		if (csr & RX_SENTSTALL_BIT){//S3C2410_UDC_OCSR1_SENTSTL) {
			udc_pr(DEBUG_VERBOSE, "RX_SENTSTALL_BIT\n");								
			//udc_write(idx, S3C2410_UDC_INDEX_REG);
			//udc_write(ep_csr1 & ~S3C2410_UDC_OCSR1_SENTSTL,
			//		S3C2410_UDC_OUT_CSR1_REG);
			return;
		}

		//if ((csr & S3C2410_UDC_OCSR1_PKTRDY) && req) {
		if ((csr & RX_RXPKTRDY_BIT) && req) {
			udc_pr(DEBUG_VERBOSE, "RX_RXPKTRDY_BIT\n");						
			vsnv3_udc_read_packet(ep,req);
		}
	}
}


static irqreturn_t vsnv3_udc_irq(int irq, void *pdev)
{
	struct vsnv3_udc *udc = (struct vsnv3_udc*)pdev;
  	AITPS_USB_CTL pUSB_CTL = udc->regs;
	static num_sof;

//	int dma_int = pUSB_DMA->USB_DMA_INT_SR;// & pUSB_DMA->USB_DMA_INT_EN;
	int tx_int = pUSB_CTL->USB_TX_INT_SR;
	int rx_int = pUSB_CTL->USB_RX_INT_SR;
	int usb_int = pUSB_CTL->USB_INT_EVENT_SR;
	pUSB_CTL->USB_TX_INT_SR = 0;
	pUSB_CTL->USB_RX_INT_SR = 0;

	DBG0("USB IRQ, tx=0x%X, rx=0x%X, usb=0x%X\r\n",tx_int,rx_int,usb_int);
	
	if(tx_int)
	{
		int i;
		if(tx_int&EP0_INT_BIT)
		{
			_DBG("%s:EP0 0x%x (%d)\n",__func__,tx_int,sof_cnt);
			vsnv3_udc_handle_ep0(udc);
		}
#if 0
		else if(tx_int&EP1_TX_INT_BIT)
		{
			_DBG("tx %s:0x%x\n",__func__,tx_int);
		}
		else if(tx_int&EP2_TX_INT_BIT)
		{
			_DBG("tx %s:0x%x\n",__func__,tx_int);
		}
		else if(tx_int&EP3_TX_INT_BIT)
		{
			_DBG("tx %s:0x%x\n",__func__,tx_int);
		}
		else if(tx_int&EP4_TX_INT_BIT)
		{
			_DBG("tx %s:0x%x\n",__func__,tx_int);
		}
		else if(tx_int&EP5_TX_INT_BIT)
		{
			_DBG("tx %s:0x%x\n",__func__,tx_int);
		}
		else if(tx_int&EP6_TX_INT_BIT)
		{
			_DBG("tx %s:0x%x\n",__func__,tx_int);
		}
		else if(tx_int&EP7_TX_INT_BIT)
		{
			_DBG("tx %s:0x%x\n",__func__,tx_int);
		}
#else
		for (i=1;i<VSNV3_MAX_NUM_EP;++i)
		{
			if((tx_int>>i)&1)
			{
				DBG0("EP%d tx int.\r\n",i);
				vsnv3_udc_handle_ep(&udc->ep[i]);
			}
		}
#endif
	}

	if(rx_int)
	{
		int i ;
		_DBG("rx %s:0x%x\n",__func__,rx_int);
		for (i=1;i<VSNV3_MAX_NUM_EP;++i) //ANDY Test
		{
			if((rx_int>>i)&1)
			{
				DBG0("EP%d rx int.\r\n",i);
				vsnv3_udc_handle_ep(&udc->ep[i]);
			}
		}		
	}

	if(usb_int)
	{

		if(usb_int&SUSPEND_INT_BIT)
		{
			_DBG("%s:SUSPEND_INT_BIT 0x%x\n",__func__,usb_int);
			if (udc->driver && udc->driver->suspend) {
				spin_unlock(&udc->lock);
				udc->driver->suspend(&udc->gadget);
				spin_lock(&udc->lock);
			}
		}

		if(usb_int&RESUME_INT_BIT)		
		{
			_DBG("%s:RESUME_INT_BIT 0x%x\n",__func__,usb_int);
			if (udc->driver && udc->driver->resume) {
				spin_unlock(&udc->lock);
				udc->driver->resume(&udc->gadget);
				spin_lock(&udc->lock);
			}
		}
		
		if(usb_int&RESET_INT_BIT)
		{
			udc->ep0state = EP0_IDLE;
		
			_DBG("%s:RESET_INT_BIT 0x%x\n",__func__,usb_int);
			vsnv3_usb_reset();
		}


		if(usb_int&SOF_INT_BIT)
		{
#ifdef CONFIG_VSNV3_UDC_DEBUG
			vsnv3_udc_ticks++;
#endif
#if 0	
		sof_cnt++;
	//		printk("usb %s:0x%x\n",__func__,usb_int);
			num_sof++;
	//For Full Speed		

			if(num_sof==8*1000)
			{
				num_sof=0;
				printk("#");			
			}
#endif			
		}
	}

	return IRQ_HANDLED;
}


static int get_frame(struct usb_gadget *gadget){printk("%s\n",__func__);return 0;}
static int wakeup(struct usb_gadget *gadget){printk("%s\n",__func__);return 0;}
static int set_selfpowered(struct usb_gadget *gadget, int is_selfpowered)
{
	
	printk("%s: This device don't support self powerd\n",__func__);
	return 0;


}
static int vbus_session(struct usb_gadget *gadget, int is_active){printk("%s\n",__func__);return 0;}
static int vbus_draw(struct usb_gadget *gadget, unsigned mA){printk("%s\n",__func__);return 0;}
static int pullup(struct usb_gadget *gadget, int is_on)
{
	//gadget->speed = USB_SPEED_HIGH;
	vsnv3_usb_enable();
	
	printk("%s\n",__func__);
	return 0;
}
static int ioctl(struct usb_gadget *gadget,
				unsigned code, unsigned long param){printk("%s\n",__func__);return 0;}
static void get_config_params(struct usb_dcd_config_params *param){printk("%s\n",__func__);return 0;}
static int	udc_start(struct usb_gadget *gadget,
			struct usb_gadget_driver *udc_drv)
{
	printk("%s\n",__func__);
	return 0;
}
static int	udc_stop(struct usb_gadget *gadget,
			struct usb_gadget_driver *udc_drv){printk("%s\n",__func__);return 0;}

	/* Those two are deprecated */

static int	vsnv3_udc_start(struct usb_gadget *gadget,struct usb_gadget_driver *gadget_drv)
{
	int i = 0;
	struct vsnv3_udc	*udc = the_controller;

	if (!gadget_drv
			|| gadget_drv->speed < USB_SPEED_FULL
			//|| !bind
			|| !gadget_drv->setup) {
		printk("bad parameter.\n");
		return -EINVAL;
	}

	if (udc->driver) {
		printk("UDC already has a gadget driver\n");
		return -EBUSY;
	}
	

	udc->driver = gadget_drv;
	vsnv3_usb_phy_init(the_controller);

	/* dev->gadget.speed = USB_SPEED_UNKNOWN; */
	//udc->gadget.speed = USB_SPEED_FULL;
	udc->gadget.speed = USB_SPEED_HIGH;

	/* Set default power state */
	//udc_write(DEFAULT_POWER_STATE, S3C2410_UDC_PWR_REG);

	/* Enable reset and suspend interrupt interrupts */
	//udc_write(S3C2410_UDC_USBINT_RESET | S3C2410_UDC_USBINT_SUSPEND,
	//		S3C2410_UDC_USB_INT_EN_REG);

	vsnv3_usb_ep_enable(0,1,udc->ep[i].ep.maxpacket);

	_DBG("%s\n",__func__);return 0;

}
static int	stop(struct usb_gadget_driver *udc_drv){printk("%s\n",__func__);return 0;}
	
static const struct usb_gadget_ops vsnv3_udc_ops ={
	.get_frame = get_frame,//)(struct usb_gadget *);
	.wakeup = wakeup ,//)(struct usb_gadget *);
	.set_selfpowered = set_selfpowered,//) (struct usb_gadget *, int is_selfpowered);
	.vbus_session = vbus_session,//) (struct usb_gadget *, int is_active);
	.vbus_draw = vbus_draw,//) (struct usb_gadget *, unsigned mA);
	.pullup = pullup,///) (struct usb_gadget *, int is_on);
	.ioctl = ioctl,//)(struct usb_gadget *,
				//unsigned code, unsigned long param);
	.get_config_params = get_config_params,//)(struct usb_dcd_config_params *);
	.udc_start = vsnv3_udc_start,//)(struct usb_gadget *,
			//struct usb_gadget_driver *);
	.udc_stop = udc_stop,//)(struct usb_gadget *,
			//struct usb_gadget_driver *);

	/* Those two are deprecated */
	.start =0,//)(struct usb_gadget_driver *,
			//int (*bind)(struct usb_gadget *));
	.stop = 0//)(struct usb_gadget_driver *);
};



int vsnv3_ep_enable(struct usb_ep *_ep,const struct usb_endpoint_descriptor *desc)
{
	struct vsnv3_udc	*udc;
	struct vsnv3_ep	*ep;
	u32			max, tmp;
	unsigned long		flags;
	u32			csr1,csr2;
	u32			int_en_reg;

	printk("%s:addr = 0x%x\n",__func__,_ep->address);

	ep = container_of(_ep, struct vsnv3_ep, ep);

	if (!_ep || !desc || ep->desc
			|| _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	udc = ep->udc;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = usb_endpoint_maxp(desc) & 0x1fff;

	local_irq_save (flags);
	_ep->maxpacket = max & 0x7ff;
	ep->desc = desc;
	ep->bIsHalted = 0;
	ep->ep.address = desc->bEndpointAddress;

	/* set max packet */
	//udc_write(ep->num, S3C2410_UDC_INDEX_REG);
	//udc_write(max >> 3, S3C2410_UDC_MAXP_REG);
	printk("max pkt size = %d\n",max);
	/* set type, direction, address; reset fifo counters */
	if (desc->bEndpointAddress & USB_DIR_IN) {
#if 0		
		csr1 = S3C2410_UDC_ICSR1_FFLUSH|S3C2410_UDC_ICSR1_CLRDT;
		csr2 = S3C2410_UDC_ICSR2_MODEIN|S3C2410_UDC_ICSR2_DMAIEN;

		udc_write(ep->num, S3C2410_UDC_INDEX_REG);
		udc_write(csr1, S3C2410_UDC_IN_CSR1_REG);
		udc_write(ep->num, S3C2410_UDC_INDEX_REG);
		udc_write(csr2, S3C2410_UDC_IN_CSR2_REG);
#endif		
		//vsnv3_usb_set_ep_max_tx_pktsize(ep->num,max);
			
	} else {
		/* don't flush in fifo or it will cause endpoint interrupt */
#if 0		
		csr1 = S3C2410_UDC_ICSR1_CLRDT;
		csr2 = S3C2410_UDC_ICSR2_DMAIEN;

		udc_write(ep->num, S3C2410_UDC_INDEX_REG);
		udc_write(csr1, S3C2410_UDC_IN_CSR1_REG);
		udc_write(ep->num, S3C2410_UDC_INDEX_REG);
		udc_write(csr2, S3C2410_UDC_IN_CSR2_REG);

		csr1 = S3C2410_UDC_OCSR1_FFLUSH | S3C2410_UDC_OCSR1_CLRDT;
		csr2 = S3C2410_UDC_OCSR2_DMAIEN;

		udc_write(ep->num, S3C2410_UDC_INDEX_REG);
		udc_write(csr1, S3C2410_UDC_OUT_CSR1_REG);
		udc_write(ep->num, S3C2410_UDC_INDEX_REG);
		udc_write(csr2, S3C2410_UDC_OUT_CSR2_REG);
#endif		
		//v/snv3_usb_set_ep_max_rx_pktsize(ep->num,max);
	}



	
	/* enable irqs */
	//int_en_reg = udc_read(S3C2410_UDC_EP_INT_EN_REG);
	//udc_write(int_en_reg | (1 << ep->num), S3C2410_UDC_EP_INT_EN_REG);
	vsnv3_usb_ep_enable(ep->num,(desc->bEndpointAddress & USB_DIR_IN)?1:0,max);

	/* print some debug message */
	tmp = desc->bEndpointAddress;
	udc_pr (DEBUG_NORMAL, "enable %s(%d) ep%x%s-blk max %d\n",
		 _ep->name,ep->num, tmp,
		 desc->bEndpointAddress & USB_DIR_IN ? "in" : "out", max);

	local_irq_restore (flags);
	//s3c2410_udc_set_halt(_ep, 0);

	return 0;

}

int vsnv3_ep_disable(struct usb_ep *ep)
{_DBG	("%s\n",__func__);return 0;}


// Function will be called when insert class driver.
static struct usb_request *vsnv3_ep_alloc_request(struct usb_ep *ep,gfp_t gfp_flags)
{

	struct vsnv3_request *req;
	_DBG("%s(%p,%d)\n", __func__, ep, gfp_flags);

	req = kzalloc(sizeof (struct vsnv3_request), gfp_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;


}

void vsnv3_ep_free_request(struct usb_ep *ep, struct usb_request *req)
{_DBG("%s\n",__func__);return 0;}
#if 0
static int s3c2410_udc_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t gfp_flags)
{
	struct s3c2410_request	*req = to_s3c2410_req(_req);
	struct s3c2410_ep	*ep = to_s3c2410_ep(_ep);
	struct s3c2410_udc	*dev;
	u32			ep_csr = 0;
	int			fifo_count = 0;
	unsigned long		flags;

	if (unlikely (!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		dprintk(DEBUG_NORMAL, "%s: invalid args\n", __func__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely (!dev->driver
			|| dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		return -ESHUTDOWN;
	}

	local_irq_save (flags);

	if (unlikely(!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue))) {
		if (!_req)
			dprintk(DEBUG_NORMAL, "%s: 1 X X X\n", __func__);
		else {
			dprintk(DEBUG_NORMAL, "%s: 0 %01d %01d %01d\n",
				__func__, !_req->complete,!_req->buf,
				!list_empty(&req->queue));
		}

		local_irq_restore(flags);
		return -EINVAL;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	dprintk(DEBUG_VERBOSE, "%s: ep%x len %d\n",
		 __func__, ep->bEndpointAddress, _req->length);

	if (ep->bEndpointAddress) {
		udc_write(ep->bEndpointAddress & 0x7F, S3C2410_UDC_INDEX_REG);

		ep_csr = udc_read((ep->bEndpointAddress & USB_DIR_IN)
				? S3C2410_UDC_IN_CSR1_REG
				: S3C2410_UDC_OUT_CSR1_REG);
		fifo_count = s3c2410_udc_fifo_count_out();
	} else {
		udc_write(0, S3C2410_UDC_INDEX_REG);
		ep_csr = udc_read(S3C2410_UDC_IN_CSR1_REG);
		fifo_count = s3c2410_udc_fifo_count_out();
	}

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->halted) {
		if (ep->bEndpointAddress == 0 /* ep0 */) {
			switch (dev->ep0state) {
			case EP0_IN_DATA_PHASE:
				if (!(ep_csr&S3C2410_UDC_EP0_CSR_IPKRDY)
						&& s3c2410_udc_write_fifo(ep,
							req)) {
					dev->ep0state = EP0_IDLE;
					req = NULL;
				}
				break;

			case EP0_OUT_DATA_PHASE:
				if ((!_req->length)
					|| ((ep_csr & S3C2410_UDC_OCSR1_PKTRDY)
						&& s3c2410_udc_read_fifo(ep,
							req))) {
					dev->ep0state = EP0_IDLE;
					req = NULL;
				}
				break;

			default:
				local_irq_restore(flags);
				return -EL2HLT;
			}
		} else if ((ep->bEndpointAddress & USB_DIR_IN) != 0
				&& (!(ep_csr&S3C2410_UDC_OCSR1_PKTRDY))
				&& s3c2410_udc_write_fifo(ep, req)) {
			req = NULL;
		} else if ((ep_csr & S3C2410_UDC_OCSR1_PKTRDY)
				&& fifo_count
				&& s3c2410_udc_read_fifo(ep, req)) {
			req = NULL;
		}
	}

	/* pio or dma irq handler advances the queue. */
	if (likely (req != 0))
		list_add_tail(&req->queue, &ep->queue);

	local_irq_restore(flags);

	dprintk(DEBUG_VERBOSE, "%s ok\n", __func__);
	return 0;
}
#endif


//Currentlly we are using same api for endpoint in/out
static int vsnv3_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct vsnv3_request *req;
	struct vsnv3_ep	*ep;
	struct vsnv3_udc	*udc;
	u32			ep_csr = 0;
	int			fifo_count = 0;
	unsigned long		flags;
	DBG0("%s ep%x len %d Data 0x%x\n",__func__, _ep->address,_req->length,((char*)_req->buf)[0]);


	req = container_of(_req, struct vsnv3_request, req);
	ep = container_of(_ep, struct vsnv3_ep, ep);

	if (unlikely(!_ep 
			|| (!ep->desc && ep->ep.name != ep0name))) {
		udc_pr(DEBUG_ERROR, "invalid args. (%x %x) name = %s\n",_ep,ep->desc,ep->ep.name);
		return -EINVAL;
	}

	udc = ep->udc;

	if (!udc || !udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		udc_pr(DEBUG_ERROR, "invalid device\n");
		return -ESHUTDOWN;
	}

	local_irq_save (flags);

	if (unlikely(!_req 
			|| !_req->complete
			|| !_req->buf 
			|| !list_empty(&req->queue))) {
		udc_pr(DEBUG_ERROR, "invalid request\n");
		if (!_req)
			udc_pr(DEBUG_ERROR, "%s: 1 X X X\n", __func__);
		else {
			udc_pr(DEBUG_ERROR, "%s: 0 %01d %01d %01d\n",
				__func__, !_req->complete,!_req->buf,
				!list_empty(&req->queue));
		}		
		local_irq_restore(flags);
		
		return -EINVAL;
	}
	
	_req->status = -EINPROGRESS;
	_req->actual = 0;

	_DBG("%s: ep%x len %d\n", __func__, _ep->address, _req->length);

	if (_ep->address) {
//		udc_write(ep->bEndpointAddress & 0x7F, S3C2410_UDC_INDEX_REG);

/*		ep_csr = udc_read((ep->bEndpointAddress & USB_DIR_IN)
				? S3C2410_UDC_IN_CSR1_REG
				: S3C2410_UDC_OUT_CSR1_REG);
*/
		ep_csr = (_ep->address & USB_DIR_IN)
					?vsnv3_usb_read_ep_tx_csr(_ep->address& 0x7F)
					:vsnv3_usb_read_ep_rx_csr(_ep->address& 0x7F);

		// udc_read(S3C2410_UDC_IN_CSR1_REG);		
		
//		fifo_count = s3c2410_udc_fifo_count_out();
	//	fifo_count = vsnv3_usb_read_ep_rx_count(_ep->address& 0x7F);
	} else {
//		udc_write(0, S3C2410_UDC_INDEX_REG);
//		ep_csr = udc_read(S3C2410_UDC_IN_CSR1_REG);
		ep_csr = vsnv3_usb_read_ep0_csr();// udc_read(S3C2410_UDC_IN_CSR1_REG);
//		fifo_count = s3c2410_udc_fifo_count_out();

	}
	
	//ANDY 
	fifo_count = vsnv3_usb_read_ep_rx_count(_ep->address& 0x7F);
	//_DBG("csr = 0x%x\n",ep_csr);
	//_DBG("udc->ep0state = 0x%x\n",udc->ep0state);
	//_DBG("fifo_count = %d\n",fifo_count);
	//_DBG("queue= %d  Halted = %d\n",list_empty(&ep->queue),ep->bIsHalted);

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->bIsHalted) {
		if (_ep->address == 0 /* ep0 */) {
			switch (udc->ep0state) {
			case EP0_IN_DATA_PHASE:
				_DBG("EP0_IN_DATA_PHASE: len = %d\n",_req->length);
				if(vsnv3_udc_write_packet(ep,req)){
					udc->ep0state = EP0_IDLE;
					req = NULL;
				}
#if 0				
				if (!(ep_csr&EP0_RXPKTRDY_BIT)
						&& s3c2410_udc_write_fifo(ep,
							req)) {
					dev->ep0state = EP0_IDLE;
					req = NULL;
				}
#endif
				break;

			case EP0_OUT_DATA_PHASE:
				_DBG("EP0_OUT_DATA_PHASE: len = %d\n",_req->length);

				if(_req->length > 0) //wait RXReady
				{
					u32 n = 0;
					//DBG0("EP0 CSR = 0x%X\r\n",vsnv3_usb_read_ep0_csr());
					while( vsnv3_usb_read_ep0_csr()&EP0_RXPKTRDY_BIT == 0 ) //workaround
					{
						if(n++>1000)
						{
							printk("EP0 out phase, TIMEOUT.\r\n");
							break;
						}
						else
							udelay(100);
					}
					//DBG0("EP0 CSR = 0x%X\r\n",vsnv3_usb_read_ep0_csr());
				}

				if ((!_req->length) || vsnv3_udc_read_packet(ep,req)) {
					udc->ep0state = EP0_IDLE;
					req = NULL;
				}
				
				break;

			default:
				local_irq_restore(flags);
				return -EL2HLT;
			}
		}
#if 0
		else if ((_ep->address & USB_DIR_IN) != 0 && (!(ep_csr&TX_TXPKTRDY_BIT)) && vsnv3_udc_write_packet(ep, req) ){
			_DBG("TX_TXPKTRDY_BIT\n",ep_csr);
			req = NULL;
#else
		else if ( _ep->address & USB_DIR_IN ){

			DBG0("req in %d bytes. csr=0x%X req=0x%X\r\n",req->req.length,ep_csr,(u32)req);
			if( likely(list_empty(&ep->queue)) && (!(ep_csr&TX_TXPKTRDY_BIT)) )
			{
				vsnv3_udc_tx_packet(ep,req);
			}
			DBG0("csr = 0x%X\r\n",vsnv3_usb_read_ep_tx_csr(_ep->address& 0x7F));
#endif
		} else if ( (ep_csr & RX_RXPKTRDY_BIT) && fifo_count && vsnv3_udc_read_packet(ep, req)) {
			//fifo_count = vsnv3_usb_read_ep_rx_count(_ep->address& 0x7F);
			_DBG("%d\n",vsnv3_usb_read_ep_rx_count(_ep->address& 0x7F));
			req = NULL;
		}
	
	}

	/* pio or dma irq handler advances the queue. */
	if (likely (req != 0))
	{
		DBG0("queue usb req. 0x%X\r\n",(u32)req);
		list_add_tail(&req->queue, &ep->queue);
	}

	local_irq_restore(flags);
	
	_DBG("%s: ep_csr = 0x%x    ep0state %s\n",__func__, vsnv3_usb_read_ep0_csr() ,ep0states_str[udc->ep0state]);

	udc_pr(DEBUG_VERBOSE, "%s ok\n", __func__);
	return 0;
}


int vsnv3_ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{DBG0("%s\n",__func__);return 0;}
int vsnv3_ep_set_halt(struct usb_ep *ep, int value)
{_DBG("%s\n",__func__);return 0;}
int vsnv3_ep_set_wedge(struct usb_ep *ep)
{_DBG("%s\n",__func__);return 0;}

int vsnv3_ep_fifo_status(struct usb_ep *ep)
{_DBG("%s\n",__func__);return 0;}
void vsnv3_ep_fifo_flush(struct usb_ep *ep)
{_DBG("%s\n",__func__);return 0;}



static const struct usb_ep_ops vsnv3_ep_ops = {
	.enable		= vsnv3_ep_enable,
	.disable		= vsnv3_ep_disable,
	.alloc_request	= vsnv3_ep_alloc_request,
	.free_request	= vsnv3_ep_free_request,
	.queue		= vsnv3_ep_queue,
	.dequeue		= vsnv3_ep_dequeue,
	.set_halt		= vsnv3_ep_set_halt,
	.set_wedge 	= vsnv3_ep_set_wedge,
	.fifo_status 	= vsnv3_ep_fifo_status,
	.fifo_flush 	= vsnv3_ep_fifo_flush,
};

static void nop_release(struct device *dev)
{

}


static struct vsnv3_udc vsnv3_udc_ctrl = {
	
	.gadget = {
		.ops	= &vsnv3_udc_ops,
		.ep0	= &vsnv3_udc_ctrl.ep[0].ep,
		.name	= DRIVER_NAME,
		.dev	= {
			.init_name = "gadget",
			.release = nop_release,
		}
	},
	.ep[0] = {
		.num		= 0,	
		.ep = {
			.name	= ep0name,
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.maxpacket	= 64,
		.int_mask	= 1 << 0,
	},
	.ep[1] = {
		.num		= 1,	
		.ep = {
			.name	= "ep1",
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.is_pingpong	= 1,
		.maxpacket	= 512,
		.int_mask	= 1 << 1,
	},
	.ep[2] = {
		.num		= 2,	
		.ep = {
			.name	= "ep2",
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.is_pingpong	= 1,
		.maxpacket	= 512,
		.int_mask	= 1 << 2,
	},
	.ep[3] = {
		.num		= 3,	
		.ep = {
			.name	= "ep3",
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.maxpacket	= 512,
		.int_mask	= 1 << 3,
	},
	.ep[4] = {
		.num		= 4,	
		.ep = {
			.name	= "ep4",
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.is_pingpong	= 1,
		.maxpacket	= 512,
		.int_mask	= 1 << 4,
	},
	.ep[5] = {
		.num		= 5,	
		.ep = {
			.name	= "ep5",
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.is_pingpong	= 1,
		.maxpacket	= 64,
		.int_mask	= 1 << 5,
	},
	.ep[6] = {
		.num		= 6,	
		.ep = {
			.name	= "ep6",
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.is_pingpong	= 1,
		.maxpacket	= 64,
		.int_mask	= 1 << 6,
	},	
	.ep[7] = {
		.num		= 7,	
		.ep = {
			.name	= "ep7",
			.ops	= &vsnv3_ep_ops,
		},
		.udc		= &vsnv3_udc_ctrl,
		.is_pingpong	= 1,
		.maxpacket	= 64,
		.int_mask	= 1 << 7,
	},		
};

static void vsnv3_udc_reinit(struct vsnv3_udc *udc)
{
	u32 i;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
	udc->ep0state = EP0_IDLE;
	
	for (i = 0; i < VSNV3_MAX_NUM_EP; i++) {
		struct vsnv3_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		ep->desc = NULL;
		ep->bIsStopped = 0;
		ep->fifo_bank = 0;
		ep->ep.maxpacket = ep->maxpacket;
		//ep->creg = (void __iomem *) udc->udp_baseaddr + AT91_UDP_CSR(i);
		ep->bIsHalted = 0;

		/* initialize one queue per endpoint */
		INIT_LIST_HEAD(&ep->queue);
	}
}

dummy_gadget_release (struct device *dev)
{
	return;
}

static int vsnv3_udc_probe(struct platform_device *pdev)
{
	struct resource		*regs;
	struct vsnv3_udc *udc = &vsnv3_udc_ctrl;
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
	AITPS_GBL   pGBL = AITC_BASE_GBL;
	
  	int ret;

	printk("%s: Start !!\n",__func__);
	
//Get Base Address
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_dbg(&pdev->dev, "no mmio resource defined\n");
		ret = -ENXIO;
		goto err_mem_res;
	}

	udc->regs = ioremap(regs->start, resource_size(regs));
	if (!udc->regs) {
		dev_dbg(&pdev->dev, "ioremap failed\n");
		ret = -EINVAL;
		goto err_ioremap;
	}

//Enable Clock

	udc->udc_clk = clk_get(NULL, "udc_clk");
	if (IS_ERR(udc->udc_clk)) {
		dev_err(&pdev->dev, "failed to get udc clock source\n");
		ret = PTR_ERR(udc->udc_clk);
		goto err_get_clk;
	}

	ret = clk_enable(udc->udc_clk);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "unable to enable VSNV3 USB clock, err %d\n",ret);
		goto err_clk_enable;
	}
	printk("usb clock is enabled. \n");
	
	//pGBL->GBL_CLK_PATH_CTL &= (~USB_PHY_PLL_INPUT_EN);
	pGBL->GBL_USB_CLK_SRC = GBL_USB_ROOT_CLK_SEL_MCLK;
	pUSB_DMA->USB_UTMI_PHY_CTL1 &= 0xFB;
	
	mdelay(10);
	
    
// Request IRQ
	udc->irq = platform_get_irq(pdev, 0);
	ret = request_irq(udc->irq, vsnv3_udc_irq,
			0, DRIVER_NAME, udc);
	if (ret < 0) {
		dev_err(&pdev->dev,"request irq %d failed\n", udc->irq);
		goto err_irq;
	}

	
// Add Gadge UDC
	vsnv3_udc_reinit(udc);
	the_controller = udc;


	
	platform_set_drvdata(pdev, udc);
	      
	ret = usb_add_gadget_udc(&pdev->dev,&udc->gadget);
	if(ret)
	{
		_DBG("usb_add_gadget_udc fail\n");
	}

	if (vsnv3_udc_dbgfs_root) {
		udc->regs_info = debugfs_create_file("registers", S_IRUGO,
				vsnv3_udc_dbgfs_root,
				udc, &vsnv3_udc_dbgfs_root);
		if (!udc->regs_info)
			dev_warn(&pdev->dev, "debugfs file creation failed\n");
	}

	dev_set_name(&udc->gadget.dev, "gadget");
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.dev.release = dummy_gadget_release;
	ret = device_register (&udc->gadget.dev);
	
	if (ret < 0) {
		put_device(&udc->gadget.dev);
		return ret;
	}
	
	_DBG("%s: Probe OK!!\n",__func__);
	
	return 0;
err_irq:


err_get_clk:

err_clk_enable:

err_ioremap:

err_mem_res:


  return ret;
}


static int vsnv3_udc_remove(struct platform_device *pdev)
{

  
}

static struct platform_driver vsnv3_udc_driver = {
  .driver		= {
    .name	= "vsnv3_udc",
    .owner	= THIS_MODULE,
  },
  .probe		= vsnv3_udc_probe,
  .remove		= vsnv3_udc_remove,
  //.suspend	= vsnv3_udc_suspend,
  //.resume		= vsnv3_udc_resume,
  //.id_table	= vsnv3_udc_ids,
};


static int __init vsnv3_udc_init(void)
{
	int ret;
	printk("# AIT UDC Driver, (c) 2013 Alpha Image Inc. #\n");


	vsnv3_udc_dbgfs_root = debugfs_create_dir(DRIVER_NAME, NULL);
	if (IS_ERR(vsnv3_udc_dbgfs_root)) {
		_DBG(KERN_ERR "%s: debugfs dir creation failed %ld\n",
			DRIVER_NAME, PTR_ERR(vsnv3_udc_dbgfs_root));
		vsnv3_udc_dbgfs_root = NULL;
	}

	
  if (ret = platform_driver_register(&vsnv3_udc_driver))
    goto err;

  return 0;

err:
  return ret;  
}

static void __exit vsnv3_udc_exit(void)
{
	platform_driver_unregister(&vsnv3_udc_driver);
	debugfs_remove(vsnv3_udc_dbgfs_root);
}

module_init(vsnv3_udc_init);
module_exit(vsnv3_udc_exit)


MODULE_DESCRIPTION("VSNV3 UDC Driver");
MODULE_AUTHOR("Vincent Chen <vincent_chen@a-i-t.com.tw>");
MODULE_LICENSE("GPL v2");

