/*
 * Copyright (C) 2007,2008 Freescale semiconductor, Inc.
 *
 * Author: Li Yang <LeoLi@freescale.com>
 *         Jerry Huang <Chang-Ming.Huang@freescale.com>
 *
 * Initialization based on code from Shlomi Gridish.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/timer.h>
#include <linux/usb.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/fsl_devices.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

#include <asm/unaligned.h>

#include <mach/mmpf_usbphy.h>
#include <mach/mmpf_system.h>
#include <mach/mmp_reg_usb.h>
#include "ait_otg.h"


#define DRIVER_VERSION "Rev. 1.55"
#define DRIVER_AUTHOR "Vincent Chen"
#define DRIVER_DESC "AIT USB OTG Transceiver Driver"
#define DRIVER_INFO DRIVER_DESC " " DRIVER_VERSION

static const char driver_name[] = "ait-usb-otg";

const pm_message_t otg_suspend_state = {
	.event = 1,
};

#define HA_DATA_PULSE

//static struct AITPS_USB_CTL usb_dr_regs;
static AITPS_USB_CTL usb_dr_regs;
static struct ait_otg *fsl_otg_dev;
static int srp_wait_done;

/* FSM timers */
struct fsl_otg_timer *a_wait_vrise_tmr, *a_wait_bcon_tmr, *a_aidl_bdis_tmr,
	*b_ase0_brst_tmr, *b_se0_srp_tmr;

/* Driver specific timers */
struct fsl_otg_timer *b_data_pulse_tmr, *b_vbus_pulse_tmr, *b_srp_fail_tmr,
	*b_srp_wait_tmr, *a_wait_enum_tmr;

static struct list_head active_timers;

static struct fsl_otg_config fsl_otg_initdata = {
	.otg_port = 1,
};


//#include "otg_fsm.h"

/* Change USB protocol when there is a protocol change */
static int otg_set_protocol(struct otg_fsm *fsm, int protocol)
{
	int ret = 0;

	if (fsm->protocol != protocol) {
		VDBG("Changing role fsm->protocol= %d; new protocol= %d\n",
			fsm->protocol, protocol);
		/* stop old protocol */
		if (fsm->protocol == PROTO_HOST)
			ret = fsm->ops->start_host(fsm, 0);
		else if (fsm->protocol == PROTO_GADGET)
			ret = fsm->ops->start_gadget(fsm, 0);
		if (ret)
			return ret;

		/* start new protocol */
		if (protocol == PROTO_HOST)
			ret = fsm->ops->start_host(fsm, 1);
		else if (protocol == PROTO_GADGET)
			ret = fsm->ops->start_gadget(fsm, 1);
		if (ret)
			return ret;

		fsm->protocol = protocol;
		return 0;
	}

	return 0;
}

static int state_changed;

/* Called when leaving a state.  Do state clean up jobs here */
void otg_leave_state(struct otg_fsm *fsm, enum usb_otg_state old_state)
{
	switch (old_state) {
	case OTG_STATE_B_IDLE:
		otg_del_timer(fsm, b_se0_srp_tmr);
		fsm->b_se0_srp = 0;
		break;
	case OTG_STATE_B_SRP_INIT:
		fsm->b_srp_done = 0;
		break;
	case OTG_STATE_B_PERIPHERAL:
		break;
	case OTG_STATE_B_WAIT_ACON:
		otg_del_timer(fsm, b_ase0_brst_tmr);
		fsm->b_ase0_brst_tmout = 0;
		break;
	case OTG_STATE_B_HOST:
		break;
	case OTG_STATE_A_IDLE:
		break;
	case OTG_STATE_A_WAIT_VRISE:
		otg_del_timer(fsm, a_wait_vrise_tmr);
		fsm->a_wait_vrise_tmout = 0;
		break;
	case OTG_STATE_A_WAIT_BCON:
		otg_del_timer(fsm, a_wait_bcon_tmr);
		fsm->a_wait_bcon_tmout = 0;
		break;
	case OTG_STATE_A_HOST:
		otg_del_timer(fsm, a_wait_enum_tmr);
		break;
	case OTG_STATE_A_SUSPEND:
		otg_del_timer(fsm, a_aidl_bdis_tmr);
		fsm->a_aidl_bdis_tmout = 0;
		fsm->a_suspend_req = 0;
		break;
	case OTG_STATE_A_PERIPHERAL:
		break;
	case OTG_STATE_A_WAIT_VFALL:
		otg_del_timer(fsm, a_wait_vrise_tmr);
		break;
	case OTG_STATE_A_VBUS_ERR:
		break;
	default:
		break;
	}
}

/* Called when entering a state */
int otg_set_state(struct otg_fsm *fsm, enum usb_otg_state new_state)
{
	state_changed = 1;
	if (fsm->transceiver->state == new_state)
		return 0;
	VDBG("Set state: %s\n", otg_state_string(new_state));
	otg_leave_state(fsm, fsm->transceiver->state);
	switch (new_state) {
	case OTG_STATE_B_IDLE:
		otg_drv_vbus(fsm, 0);
		otg_chrg_vbus(fsm, 0);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_UNDEF);
		otg_add_timer(fsm, b_se0_srp_tmr);
		break;
	case OTG_STATE_B_SRP_INIT:
		otg_start_pulse(fsm);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_UNDEF);
		otg_add_timer(fsm, b_srp_fail_tmr);
		break;
	case OTG_STATE_B_PERIPHERAL:
		otg_chrg_vbus(fsm, 0);
		otg_loc_conn(fsm, 1);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_GADGET);
		break;
	case OTG_STATE_B_WAIT_ACON:
		otg_chrg_vbus(fsm, 0);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_HOST);
		otg_add_timer(fsm, b_ase0_brst_tmr);
		fsm->a_bus_suspend = 0;
		break;
	case OTG_STATE_B_HOST:
		otg_chrg_vbus(fsm, 0);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 1);
		otg_set_protocol(fsm, PROTO_HOST);
		usb_bus_start_enum(fsm->transceiver->host,
				fsm->transceiver->host->otg_port);
		break;
	case OTG_STATE_A_IDLE:
		otg_drv_vbus(fsm, 0);
		otg_chrg_vbus(fsm, 0);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_HOST);
		break;
	case OTG_STATE_A_WAIT_VRISE:
		otg_drv_vbus(fsm, 1);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_HOST);
		otg_add_timer(fsm, a_wait_vrise_tmr);
		break;
	case OTG_STATE_A_WAIT_BCON:
		otg_drv_vbus(fsm, 1);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_HOST);
		otg_add_timer(fsm, a_wait_bcon_tmr);
		break;
	case OTG_STATE_A_HOST:
		otg_drv_vbus(fsm, 1);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 1);
		otg_set_protocol(fsm, PROTO_HOST);
		/*
		 * When HNP is triggered while a_bus_req = 0, a_host will
		 * suspend too fast to complete a_set_b_hnp_en
		 */
		if (!fsm->a_bus_req || fsm->a_suspend_req)
			otg_add_timer(fsm, a_wait_enum_tmr);
		break;
	case OTG_STATE_A_SUSPEND:
		otg_drv_vbus(fsm, 1);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_HOST);
		otg_add_timer(fsm, a_aidl_bdis_tmr);

		break;
	case OTG_STATE_A_PERIPHERAL:
		otg_loc_conn(fsm, 1);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_GADGET);
		otg_drv_vbus(fsm, 1);
		break;
	case OTG_STATE_A_WAIT_VFALL:
		otg_drv_vbus(fsm, 0);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_HOST);
		break;
	case OTG_STATE_A_VBUS_ERR:
		otg_drv_vbus(fsm, 0);
		otg_loc_conn(fsm, 0);
		otg_loc_sof(fsm, 0);
		otg_set_protocol(fsm, PROTO_UNDEF);
		break;
	default:
		break;
	}

	fsm->transceiver->state = new_state;
	return 0;
}

/* State change judgement */
int otg_statemachine(struct otg_fsm *fsm)
{
	enum usb_otg_state state;
	unsigned long flags;

	spin_lock_irqsave(&fsm->lock, flags);

	state = fsm->transceiver->state;
	state_changed = 0;
	/* State machine state change judgement */

	switch (state) {
	case OTG_STATE_UNDEFINED:
		VDBG("fsm->id = %d\n", fsm->id);
		if (fsm->id)
			otg_set_state(fsm, OTG_STATE_B_IDLE);
		else
			otg_set_state(fsm, OTG_STATE_A_IDLE);
		break;
	case OTG_STATE_B_IDLE:
		if (!fsm->id)
			otg_set_state(fsm, OTG_STATE_A_IDLE);
		else if (fsm->b_sess_vld && fsm->transceiver->gadget)
			otg_set_state(fsm, OTG_STATE_B_PERIPHERAL);
		else if (fsm->b_bus_req && fsm->b_sess_end && fsm->b_se0_srp)
			otg_set_state(fsm, OTG_STATE_B_SRP_INIT);
		break;
	case OTG_STATE_B_SRP_INIT:
		if (!fsm->id || fsm->b_srp_done)
			otg_set_state(fsm, OTG_STATE_B_IDLE);
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (!fsm->id || !fsm->b_sess_vld)
			otg_set_state(fsm, OTG_STATE_B_IDLE);
		else if (fsm->b_bus_req && fsm->transceiver->
				gadget->b_hnp_enable && fsm->a_bus_suspend)
			otg_set_state(fsm, OTG_STATE_B_WAIT_ACON);
		break;
	case OTG_STATE_B_WAIT_ACON:
		if (fsm->a_conn)
			otg_set_state(fsm, OTG_STATE_B_HOST);
		else if (!fsm->id || !fsm->b_sess_vld)
			otg_set_state(fsm, OTG_STATE_B_IDLE);
		else if (fsm->a_bus_resume || fsm->b_ase0_brst_tmout) {
			fsm->b_ase0_brst_tmout = 0;
			otg_set_state(fsm, OTG_STATE_B_PERIPHERAL);
		}
		break;
	case OTG_STATE_B_HOST:
		if (!fsm->id || !fsm->b_sess_vld)
			otg_set_state(fsm, OTG_STATE_B_IDLE);
		else if (!fsm->b_bus_req || !fsm->a_conn)
			otg_set_state(fsm, OTG_STATE_B_PERIPHERAL);
		break;
	case OTG_STATE_A_IDLE:
		if (fsm->id)
			otg_set_state(fsm, OTG_STATE_B_IDLE);
		else if (!fsm->a_bus_drop && (fsm->a_bus_req || fsm->a_srp_det))
			otg_set_state(fsm, OTG_STATE_A_WAIT_VRISE);
		break;
	case OTG_STATE_A_WAIT_VRISE:
		if (fsm->id || fsm->a_bus_drop || fsm->a_vbus_vld ||
				fsm->a_wait_vrise_tmout) {
			otg_set_state(fsm, OTG_STATE_A_WAIT_BCON);
		}
		break;
	case OTG_STATE_A_WAIT_BCON:
		if (!fsm->a_vbus_vld)
			otg_set_state(fsm, OTG_STATE_A_VBUS_ERR);
		else if (fsm->b_conn)
			otg_set_state(fsm, OTG_STATE_A_HOST);
		else if (fsm->id | fsm->a_bus_drop | fsm->a_wait_bcon_tmout)
			otg_set_state(fsm, OTG_STATE_A_WAIT_VFALL);
		break;
	case OTG_STATE_A_HOST:
		if ((!fsm->a_bus_req || fsm->a_suspend_req) &&
				fsm->transceiver->host->b_hnp_enable)
			otg_set_state(fsm, OTG_STATE_A_SUSPEND);
		else if (fsm->id || !fsm->b_conn || fsm->a_bus_drop)
			otg_set_state(fsm, OTG_STATE_A_WAIT_BCON);
		else if (!fsm->a_vbus_vld)
			otg_set_state(fsm, OTG_STATE_A_VBUS_ERR);
		break;
	case OTG_STATE_A_SUSPEND:
		if (!fsm->b_conn && fsm->transceiver->host->b_hnp_enable)
			otg_set_state(fsm, OTG_STATE_A_PERIPHERAL);
		else if (!fsm->b_conn && !fsm->transceiver->host->b_hnp_enable)
			otg_set_state(fsm, OTG_STATE_A_WAIT_BCON);
		else if (fsm->a_bus_req || fsm->b_bus_resume)
			otg_set_state(fsm, OTG_STATE_A_HOST);
		else if (fsm->id || fsm->a_bus_drop || fsm->a_aidl_bdis_tmout)
			otg_set_state(fsm, OTG_STATE_A_WAIT_VFALL);
		else if (!fsm->a_vbus_vld)
			otg_set_state(fsm, OTG_STATE_A_VBUS_ERR);
		break;
	case OTG_STATE_A_PERIPHERAL:
		if (fsm->id || fsm->a_bus_drop)
			otg_set_state(fsm, OTG_STATE_A_WAIT_VFALL);
		else if (fsm->b_bus_suspend)
			otg_set_state(fsm, OTG_STATE_A_WAIT_BCON);
		else if (!fsm->a_vbus_vld)
			otg_set_state(fsm, OTG_STATE_A_VBUS_ERR);
		break;
	case OTG_STATE_A_WAIT_VFALL:
		if (fsm->id || fsm->a_bus_req || (!fsm->a_sess_vld &&
					!fsm->b_conn))
			otg_set_state(fsm, OTG_STATE_A_IDLE);
		break;
	case OTG_STATE_A_VBUS_ERR:
		if (fsm->id || fsm->a_bus_drop || fsm->a_clr_err)
			otg_set_state(fsm, OTG_STATE_A_WAIT_VFALL);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&fsm->lock, flags);

	VDBG("quit statemachine, changed = %d\n", state_changed);
	return state_changed;
}


#if 0
#ifdef CONFIG_PPC32
static u32 _fsl_readl_be(const unsigned __iomem *p)
{
	return in_be32(p);
}

static u32 _fsl_readl_le(const unsigned __iomem *p)
{
	return in_le32(p);
}

static void _fsl_writel_be(u32 v, unsigned __iomem *p)
{
	out_be32(p, v);
}

static void _fsl_writel_le(u32 v, unsigned __iomem *p)
{
	out_le32(p, v);
}

static u32 (*_fsl_readl)(const unsigned __iomem *p);
static void (*_fsl_writel)(u32 v, unsigned __iomem *p);

#define fsl_readl(p)		(*_fsl_readl)((p))
#define fsl_writel(v, p)	(*_fsl_writel)((v), (p))

#else
#define fsl_readl(addr)		readl(addr)
#define fsl_writel(val, addr)	writel(val, addr)
#endif /* CONFIG_PPC32 */

/* Routines to access transceiver ULPI registers */
u8 view_ulpi(u8 addr)
{
	u32 temp;

	temp = 0x40000000 | (addr << 16);
	fsl_writel(temp, &usb_dr_regs->ulpiview);
	udelay(1000);
	while (temp & 0x40)
		temp = fsl_readl(&usb_dr_regs->ulpiview);
	return (le32_to_cpu(temp) & 0x0000ff00) >> 8;
}

int write_ulpi(u8 addr, u8 data)
{
	u32 temp;

	temp = 0x60000000 | (addr << 16) | data;
	fsl_writel(temp, &usb_dr_regs->ulpiview);
	return 0;
}
#endif
/* -------------------------------------------------------------*/
/* Operations that will be called from OTG Finite State Machine */

/* Charge vbus for vbus pulsing in SRP */
void fsl_otg_chrg_vbus(int on)
{
	u32 tmp;

//	tmp = fsl_readl(&usb_dr_regs->otgsc) & ~OTGSC_INTSTS_MASK;

	if (on)
		/* stop discharging, start charging */
		tmp = (tmp & ~OTGSC_CTRL_VBUS_DISCHARGE) |
			OTGSC_CTRL_VBUS_CHARGE;
	else
		/* stop charging */
		tmp &= ~OTGSC_CTRL_VBUS_CHARGE;

//	fsl_writel(tmp, &usb_dr_regs->otgsc);
}

/* Discharge vbus through a resistor to ground */
void fsl_otg_dischrg_vbus(int on)
{
	u32 tmp;

//	tmp = fsl_readl(&usb_dr_regs->otgsc) & ~OTGSC_INTSTS_MASK;

	if (on)
		/* stop charging, start discharging */
		tmp = (tmp & ~OTGSC_CTRL_VBUS_CHARGE) |
			OTGSC_CTRL_VBUS_DISCHARGE;
	else
		/* stop discharging */
		tmp &= ~OTGSC_CTRL_VBUS_DISCHARGE;

//	fsl_writel(tmp, &usb_dr_regs->otgsc);
}

/* A-device driver vbus, controlled through PP bit in PORTSC */
void fsl_otg_drv_vbus(int on)
{
	u32 tmp;

	if (on) {
//		tmp = fsl_readl(&usb_dr_regs->portsc) & ~PORTSC_W1C_BITS;
//		fsl_writel(tmp | PORTSC_PORT_POWER, &usb_dr_regs->portsc);
	} else {
//		tmp = fsl_readl(&usb_dr_regs->portsc) &
//		      ~PORTSC_W1C_BITS & ~PORTSC_PORT_POWER;
//		fsl_writel(tmp, &usb_dr_regs->portsc);
	}
}

/*
 * Pull-up D+, signalling connect by periperal. Also used in
 * data-line pulsing in SRP
 */
void fsl_otg_loc_conn(int on)
{
	u32 tmp;

//	tmp = fsl_readl(&usb_dr_regs->otgsc) & ~OTGSC_INTSTS_MASK;

	if (on)
		tmp |= OTGSC_CTRL_DATA_PULSING;
	else
		tmp &= ~OTGSC_CTRL_DATA_PULSING;

	//fsl_writel(tmp, &usb_dr_regs->otgsc);
}

/*
 * Generate SOF by host.  This is controlled through suspend/resume the
 * port.  In host mode, controller will automatically send SOF.
 * Suspend will block the data on the port.
 */
void fsl_otg_loc_sof(int on)
{
	u32 tmp;

//	tmp = fsl_readl(&fsl_otg_dev->dr_mem_map->portsc) & ~PORTSC_W1C_BITS;
	if (on)
		tmp |= PORTSC_PORT_FORCE_RESUME;
	else
		tmp |= PORTSC_PORT_SUSPEND;

//	fsl_writel(tmp, &fsl_otg_dev->dr_mem_map->portsc);

}

/* Start SRP pulsing by data-line pulsing, followed with v-bus pulsing. */
void fsl_otg_start_pulse(void)
{
	u32 tmp;

	srp_wait_done = 0;
#ifdef HA_DATA_PULSE
//	tmp = fsl_readl(&usb_dr_regs->otgsc) & ~OTGSC_INTSTS_MASK;
	tmp |= OTGSC_HA_DATA_PULSE;
//	fsl_writel(tmp, &usb_dr_regs->otgsc);
#else
	fsl_otg_loc_conn(1);
#endif

	fsl_otg_add_timer(b_data_pulse_tmr);
}

void b_data_pulse_end(unsigned long foo)
{
#ifdef HA_DATA_PULSE
#else
	fsl_otg_loc_conn(0);
#endif

	/* Do VBUS pulse after data pulse */
	fsl_otg_pulse_vbus();
}

void fsl_otg_pulse_vbus(void)
{
	srp_wait_done = 0;
	fsl_otg_chrg_vbus(1);
	/* start the timer to end vbus charge */
	fsl_otg_add_timer(b_vbus_pulse_tmr);
}

void b_vbus_pulse_end(unsigned long foo)
{
	fsl_otg_chrg_vbus(0);

	/*
	 * As USB3300 using the same a_sess_vld and b_sess_vld voltage
	 * we need to discharge the bus for a while to distinguish
	 * residual voltage of vbus pulsing and A device pull up
	 */
	fsl_otg_dischrg_vbus(1);
	fsl_otg_add_timer(b_srp_wait_tmr);
}

void b_srp_end(unsigned long foo)
{
	fsl_otg_dischrg_vbus(0);
	srp_wait_done = 1;

	if ((fsl_otg_dev->otg.state == OTG_STATE_B_SRP_INIT) &&
	    fsl_otg_dev->fsm.b_sess_vld)
		fsl_otg_dev->fsm.b_srp_done = 1;
}

/*
 * Workaround for a_host suspending too fast.  When a_bus_req=0,
 * a_host will start by SRP.  It needs to set b_hnp_enable before
 * actually suspending to start HNP
 */
void a_wait_enum(unsigned long foo)
{
	VDBG("a_wait_enum timeout\n");
	if (!fsl_otg_dev->otg.host->b_hnp_enable)
		fsl_otg_add_timer(a_wait_enum_tmr);
	else
		otg_statemachine(&fsl_otg_dev->fsm);
}

/* The timeout callback function to set time out bit */
void set_tmout(unsigned long indicator)
{
	*(int *)indicator = 1;
}

/* Initialize timers */
int fsl_otg_init_timers(struct otg_fsm *fsm)
{
	/* FSM used timers */
	a_wait_vrise_tmr = otg_timer_initializer(&set_tmout, TA_WAIT_VRISE,
				(unsigned long)&fsm->a_wait_vrise_tmout);
	if (!a_wait_vrise_tmr)
		return -ENOMEM;

	a_wait_bcon_tmr = otg_timer_initializer(&set_tmout, TA_WAIT_BCON,
				(unsigned long)&fsm->a_wait_bcon_tmout);
	if (!a_wait_bcon_tmr)
		return -ENOMEM;

	a_aidl_bdis_tmr = otg_timer_initializer(&set_tmout, TA_AIDL_BDIS,
				(unsigned long)&fsm->a_aidl_bdis_tmout);
	if (!a_aidl_bdis_tmr)
		return -ENOMEM;

	b_ase0_brst_tmr = otg_timer_initializer(&set_tmout, TB_ASE0_BRST,
				(unsigned long)&fsm->b_ase0_brst_tmout);
	if (!b_ase0_brst_tmr)
		return -ENOMEM;

	b_se0_srp_tmr = otg_timer_initializer(&set_tmout, TB_SE0_SRP,
				(unsigned long)&fsm->b_se0_srp);
	if (!b_se0_srp_tmr)
		return -ENOMEM;

	b_srp_fail_tmr = otg_timer_initializer(&set_tmout, TB_SRP_FAIL,
				(unsigned long)&fsm->b_srp_done);
	if (!b_srp_fail_tmr)
		return -ENOMEM;

	a_wait_enum_tmr = otg_timer_initializer(&a_wait_enum, 10,
				(unsigned long)&fsm);
	if (!a_wait_enum_tmr)
		return -ENOMEM;

	/* device driver used timers */
	b_srp_wait_tmr = otg_timer_initializer(&b_srp_end, TB_SRP_WAIT, 0);
	if (!b_srp_wait_tmr)
		return -ENOMEM;

	b_data_pulse_tmr = otg_timer_initializer(&b_data_pulse_end,
				TB_DATA_PLS, 0);
	if (!b_data_pulse_tmr)
		return -ENOMEM;

	b_vbus_pulse_tmr = otg_timer_initializer(&b_vbus_pulse_end,
				TB_VBUS_PLS, 0);
	if (!b_vbus_pulse_tmr)
		return -ENOMEM;

	return 0;
}

/* Uninitialize timers */
void fsl_otg_uninit_timers(void)
{
	/* FSM used timers */
	if (a_wait_vrise_tmr != NULL)
		kfree(a_wait_vrise_tmr);
	if (a_wait_bcon_tmr != NULL)
		kfree(a_wait_bcon_tmr);
	if (a_aidl_bdis_tmr != NULL)
		kfree(a_aidl_bdis_tmr);
	if (b_ase0_brst_tmr != NULL)
		kfree(b_ase0_brst_tmr);
	if (b_se0_srp_tmr != NULL)
		kfree(b_se0_srp_tmr);
	if (b_srp_fail_tmr != NULL)
		kfree(b_srp_fail_tmr);
	if (a_wait_enum_tmr != NULL)
		kfree(a_wait_enum_tmr);

	/* device driver used timers */
	if (b_srp_wait_tmr != NULL)
		kfree(b_srp_wait_tmr);
	if (b_data_pulse_tmr != NULL)
		kfree(b_data_pulse_tmr);
	if (b_vbus_pulse_tmr != NULL)
		kfree(b_vbus_pulse_tmr);
}

/* Add timer to timer list */
void fsl_otg_add_timer(void *gtimer)
{
	struct fsl_otg_timer *timer = gtimer;
	struct fsl_otg_timer *tmp_timer;

	/*
	 * Check if the timer is already in the active list,
	 * if so update timer count
	 */
	list_for_each_entry(tmp_timer, &active_timers, list)
	    if (tmp_timer == timer) {
		timer->count = timer->expires;
		return;
	}
	timer->count = timer->expires;
	list_add_tail(&timer->list, &active_timers);
}

/* Remove timer from the timer list; clear timeout status */
void fsl_otg_del_timer(void *gtimer)
{
	struct fsl_otg_timer *timer = gtimer;
	struct fsl_otg_timer *tmp_timer, *del_tmp;

	list_for_each_entry_safe(tmp_timer, del_tmp, &active_timers, list)
		if (tmp_timer == timer)
			list_del(&timer->list);
}

/*
 * Reduce timer count by 1, and find timeout conditions.
 * Called by fsl_otg 1ms timer interrupt
 */
int fsl_otg_tick_timer(void)
{
	struct fsl_otg_timer *tmp_timer, *del_tmp;
	int expired = 0;

	list_for_each_entry_safe(tmp_timer, del_tmp, &active_timers, list) {
		tmp_timer->count--;
		/* check if timer expires */
		if (!tmp_timer->count) {
			list_del(&tmp_timer->list);
			tmp_timer->function(tmp_timer->data);
			expired = 1;
		}
	}

	return expired;
}

/* Reset controller, not reset the bus */
void otg_reset_controller(void)
{
	u32 command;

//	command = fsl_readl(&usb_dr_regs->usbcmd);
	command |= (1 << 1);
//	fsl_writel(command, &usb_dr_regs->usbcmd);
//	while (fsl_readl(&usb_dr_regs->usbcmd) & (1 << 1))
//		;
}

/* Call suspend/resume routines in host driver */
int fsl_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct otg_transceiver *xceiv = fsm->transceiver;
	struct device *dev;
	struct ait_otg *otg_dev = container_of(xceiv, struct ait_otg, otg);
	u32 retval = 0;
printk(KERN_INFO "%s,%d\n",__func__,__LINE__);

	if (!xceiv->host)
		return -ENODEV;
	dev = xceiv->host->controller;

	/*
	 * Update a_vbus_vld state as a_vbus_vld int is disabled
	 * in device mode
	 */
//	fsm->a_vbus_vld =
//		!!(fsl_readl(&usb_dr_regs->otgsc) & OTGSC_STS_A_VBUS_VALID);
	if (on) {
		/* start fsl usb host controller */
		if (otg_dev->host_working)
			goto end;
		else {
			otg_reset_controller();
			VDBG("host on......\n");
			if (dev->driver->pm && dev->driver->pm->resume) {
				retval = dev->driver->pm->resume(dev);
				if (fsm->id) {
					/* default-b */
					fsl_otg_drv_vbus(1);
					/*
					 * Workaround: b_host can't driver
					 * vbus, but PP in PORTSC needs to
					 * be 1 for host to work.
					 * So we set drv_vbus bit in
					 * transceiver to 0 thru ULPI.
					 */
//					write_ulpi(0x0c, 0x20);
				}
			}

			otg_dev->host_working = 1;
		}
	} else {
		/* stop fsl usb host controller */
		if (!otg_dev->host_working)
			goto end;
		else {
			VDBG("host off......\n");
			if (dev && dev->driver) {
				if (dev->driver->pm && dev->driver->pm->suspend)
					retval = dev->driver->pm->suspend(dev);
				if (fsm->id)
					/* default-b */
					fsl_otg_drv_vbus(0);
			}
			otg_dev->host_working = 0;
		}
	}
end:
	return retval;
}

/*
 * Call suspend and resume function in udc driver
 * to stop and start udc driver.
 */
int fsl_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct otg_transceiver *xceiv = fsm->transceiver;
	struct device *dev;
printk(KERN_INFO "%s,%d\n",__func__,__LINE__);

	if (!xceiv->gadget || !xceiv->gadget->dev.parent)
		return -ENODEV;

	VDBG("gadget %s\n", on ? "on" : "off");
	dev = xceiv->gadget->dev.parent;

	if (on) {
		if (dev->driver->resume)
			dev->driver->resume(dev);
	} else {
		if (dev->driver->suspend)
			dev->driver->suspend(dev, otg_suspend_state);
	}

	return 0;
}

/*
 * Called by initialization code of host driver.  Register host controller
 * to the OTG.  Suspend host for OTG role detection.
 */
static int fsl_otg_set_host(struct otg_transceiver *otg_p, struct usb_bus *host)
{
	struct ait_otg *otg_dev = container_of(otg_p, struct ait_otg, otg);
printk(KERN_INFO "%s,%d\n",__func__,__LINE__);
	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	otg_p->host = host;

	otg_dev->fsm.a_bus_drop = 0;
	otg_dev->fsm.a_bus_req = 1;

	if (host) {
		VDBG("host off......\n");

		otg_p->host->otg_port = fsl_otg_initdata.otg_port;
		otg_p->host->is_b_host = otg_dev->fsm.id;
		/*
		 * must leave time for khubd to finish its thing
		 * before yanking the host driver out from under it,
		 * so suspend the host after a short delay.
		 */
		otg_dev->host_working = 1;
		schedule_delayed_work(&otg_dev->otg_event, 100);
		return 0;
	} else {
		/* host driver going away */
//		if (!(fsl_readl(&otg_dev->dr_mem_map->otgsc) &
//		      OTGSC_STS_USB_ID)) {
			/* Mini-A cable connected */
			struct otg_fsm *fsm = &otg_dev->fsm;

			otg_p->state = OTG_STATE_UNDEFINED;
			fsm->protocol = PROTO_UNDEF;
//		}
	}

	otg_dev->host_working = 0;

	otg_statemachine(&otg_dev->fsm);

	return 0;
}

/* Called by initialization code of udc.  Register udc to OTG. */
static int fsl_otg_set_peripheral(struct otg_transceiver *otg_p,
				  struct usb_gadget *gadget)
{
	struct ait_otg *otg_dev = container_of(otg_p, struct ait_otg, otg);

	VDBG("otg_dev 0x%x\n", (int)otg_dev);
	VDBG("fsl_otg_dev 0x%x\n", (int)fsl_otg_dev);

	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	if (!gadget) {
		if (!otg_dev->otg.default_a)
			otg_p->gadget->ops->vbus_draw(otg_p->gadget, 0);
		usb_gadget_vbus_disconnect(otg_dev->otg.gadget);
		otg_dev->otg.gadget = 0;
		otg_dev->fsm.b_bus_req = 0;
		otg_statemachine(&otg_dev->fsm);
		return 0;
	}

	otg_p->gadget = gadget;
	otg_p->gadget->is_a_peripheral = !otg_dev->fsm.id;

	otg_dev->fsm.b_bus_req = 1;

	/* start the gadget right away if the ID pin says Mini-B */
	DBG("ID pin=%d\n", otg_dev->fsm.id);
	if (otg_dev->fsm.id == 1) {
		fsl_otg_start_host(&otg_dev->fsm, 0);
		otg_drv_vbus(&otg_dev->fsm, 0);
		fsl_otg_start_gadget(&otg_dev->fsm, 1);
	}

	return 0;
}

/* Set OTG port power, only for B-device */
static int fsl_otg_set_power(struct otg_transceiver *otg_p, unsigned mA)
{
	if (!fsl_otg_dev)
		return -ENODEV;
	if (otg_p->state == OTG_STATE_B_PERIPHERAL)
		pr_info("FSL OTG: Draw %d mA\n", mA);

	return 0;
}

/*
 * Delayed pin detect interrupt processing.
 *
 * When the Mini-A cable is disconnected from the board,
 * the pin-detect interrupt happens before the disconnnect
 * interrupts for the connected device(s).  In order to
 * process the disconnect interrupt(s) prior to switching
 * roles, the pin-detect interrupts are delayed, and handled
 * by this routine.
 */
static void ait_otg_event(struct work_struct *work)
{
	struct ait_otg *og = container_of(work, struct ait_otg, otg_event.work);
	struct otg_fsm *fsm = &og->fsm;
printk(KERN_INFO "%s,%d\n",__func__,__LINE__);

	if (fsm->id) {		/* switch to gadget */
		fsl_otg_start_host(fsm, 0);
		otg_drv_vbus(fsm, 0);
		fsl_otg_start_gadget(fsm, 1);
	}
}

/* B-device start SRP */
static int fsl_otg_start_srp(struct otg_transceiver *otg_p)
{
	struct ait_otg *otg_dev = container_of(otg_p, struct ait_otg, otg);

	if (!otg_p || otg_dev != fsl_otg_dev
	    || otg_p->state != OTG_STATE_B_IDLE)
		return -ENODEV;

	otg_dev->fsm.b_bus_req = 1;
	otg_statemachine(&otg_dev->fsm);

	return 0;
}

/* A_host suspend will call this function to start hnp */
static int fsl_otg_start_hnp(struct otg_transceiver *otg_p)
{
	struct ait_otg *otg_dev = container_of(otg_p, struct ait_otg, otg);

	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	DBG("start_hnp...n");

	/* clear a_bus_req to enter a_suspend state */
	otg_dev->fsm.a_bus_req = 0;
	otg_statemachine(&otg_dev->fsm);

	return 0;
}

/*
 * Interrupt handler.  OTG/host/peripheral share the same int line.
 * OTG driver clears OTGSC interrupts and leaves USB interrupts
 * intact.  It needs to have knowledge of some USB interrupts
 * such as port change.
 */
irqreturn_t ait_otg_isr(int irq, void *dev_id)
{
	struct otg_fsm *fsm = &((struct ait_otg *)dev_id)->fsm;
	struct otg_transceiver *otg = &((struct ait_otg *)dev_id)->otg;
	u8 otg_int_src, otg_sc;

//	otg_sc = fsl_readl(&usb_dr_regs->otgsc);
	otg_sc = usb_dr_regs->USB_INT_EVENT_SR;

//	otg_int_src = otg_sc & OTGSC_INTSTS_MASK & (otg_sc >> 8);
	otg_int_src = otg_sc & usb_dr_regs->USB_INT_EVENT_EN; 
	
	/* Only clear otg interrupts */
//	fsl_writel(otg_sc, &usb_dr_regs->otgsc);
//	usb_dr_regs->USB_INT_EVENT_SR = otg_sc;
	
	/*FIXME: ID change not generate when init to 0 */
//	fsm->id = (otg_sc & OTGSC_STS_USB_ID) ? 1 : 0;
	fsm->id = (usb_dr_regs->USB_ADT_DEV_CTL &USB_HOST_MODE)?0:1;                                // 0x60
	otg->default_a = (fsm->id == 0);

	/* process OTG interrupts */
	if (otg_int_src&USB_INT_STATUS_MASK) {

	        if (otg_int_src & USB_INT_SUSPEND)
			pr_warn(AIT_OTG_NAME ": USB_INT_SUSPEND.\n");

	        if (otg_int_src & USB_INT_RESUME)
			pr_warn(AIT_OTG_NAME ": USB_INT_RESUME.\n");

	        if (otg_int_src & USB_INT_RESET)
			pr_warn(AIT_OTG_NAME ": USB_INT_RESET.\n");

	        if (otg_int_src & USB_INT_BABBLE)
			pr_warn(AIT_OTG_NAME ": USB_INT_BABBLE.\n");

	        if (otg_int_src & USB_INT_SOF)
			pr_warn(AIT_OTG_NAME ": USB_INT_SOF.\n");

	        if (otg_int_src & USB_INT_CONN)
	        {
			usb_dr_regs->USB_INT_EVENT_SR = USB_INT_CONN;
			
			pr_warn(AIT_OTG_NAME ": USB_INT_CONN.\n");

			fsm->id = (usb_dr_regs->USB_ADT_DEV_CTL &USB_HOST_MODE)?0:1;                                // 0x60
			otg->default_a = (fsm->id == 0);
			/* clear conn information */
			if (fsm->id)
				fsm->b_conn = 0;
			else
				fsm->a_conn = 0;

			if (otg->host)
				otg->host->is_b_host = fsm->id;
			if (otg->gadget)
				otg->gadget->is_a_peripheral = !fsm->id;
			VDBG("ID int (ID is %d)\n", fsm->id);

			if (fsm->id) {	/* switch to gadget */
				schedule_delayed_work(
					&((struct ait_otg *)dev_id)->otg_event,
					100);
			} else {	/* switch to host */
				cancel_delayed_work(&
						    ((struct ait_otg *)dev_id)->otg_event);
				fsl_otg_start_gadget(fsm, 0);
				otg_drv_vbus(fsm, 1);
				fsl_otg_start_host(fsm, 1);
			}
			return IRQ_HANDLED;
	        }
	        if (otg_int_src & USB_INT_DISCON)
			pr_warn(AIT_OTG_NAME ": USB_INT_DISCON.\n");

	        if (otg_int_src & USB_INT_SESS_REQ)
			pr_warn(AIT_OTG_NAME ": USB_INT_SESS_REQ.\n");

	        if (otg_int_src & USB_INT_VBUS_ERR)
			pr_warn(AIT_OTG_NAME ": USB_INT_VBUS_ERR.\n");


#if 0		
		if (otg_int_src & OTGSC_INTSTS_USB_ID) {
			fsm->id = (otg_sc & OTGSC_STS_USB_ID) ? 1 : 0;
			otg->default_a = (fsm->id == 0);
			/* clear conn information */
			if (fsm->id)
				fsm->b_conn = 0;
			else
				fsm->a_conn = 0;

			if (otg->host)
				otg->host->is_b_host = fsm->id;
			if (otg->gadget)
				otg->gadget->is_a_peripheral = !fsm->id;
			VDBG("ID int (ID is %d)\n", fsm->id);

			if (fsm->id) {	/* switch to gadget */
				schedule_delayed_work(
					&((struct ait_otg *)dev_id)->otg_event,
					100);
			} else {	/* switch to host */
				cancel_delayed_work(&
						    ((struct ait_otg *)dev_id)->
						    otg_event);
				fsl_otg_start_gadget(fsm, 0);
				otg_drv_vbus(fsm, 1);
				fsl_otg_start_host(fsm, 1);
			}
			return IRQ_HANDLED;
		}
#endif		
	}
	return IRQ_NONE;
}

static struct otg_fsm_ops fsl_otg_ops = {
	.chrg_vbus = fsl_otg_chrg_vbus,
	.drv_vbus = fsl_otg_drv_vbus,
	.loc_conn = fsl_otg_loc_conn,
	.loc_sof = fsl_otg_loc_sof,
	.start_pulse = fsl_otg_start_pulse,

	.add_timer = fsl_otg_add_timer,
	.del_timer = fsl_otg_del_timer,

	.start_host = fsl_otg_start_host,
	.start_gadget = fsl_otg_start_gadget,
};

/* Initialize the global variable fsl_otg_dev and request IRQ for OTG */
static int ait_otg_conf(struct platform_device *pdev)
{
	struct ait_otg *fsl_otg_tc;
	int status;

	if (fsl_otg_dev)
		return 0;

	/* allocate space to fsl otg device */
	fsl_otg_tc = kzalloc(sizeof(struct ait_otg), GFP_KERNEL);
	if (!fsl_otg_tc)
		return -ENOMEM;

	INIT_DELAYED_WORK(&fsl_otg_tc->otg_event, ait_otg_event);

	INIT_LIST_HEAD(&active_timers);
	status = fsl_otg_init_timers(&fsl_otg_tc->fsm);
	if (status) {
		pr_info("Couldn't init OTG timers\n");
		goto err;
	}
	spin_lock_init(&fsl_otg_tc->fsm.lock);

	/* Set OTG state machine operations */
	fsl_otg_tc->fsm.ops = &fsl_otg_ops;

	/* initialize the otg structure */
	fsl_otg_tc->otg.label = DRIVER_DESC;
	fsl_otg_tc->otg.set_host = fsl_otg_set_host;
	fsl_otg_tc->otg.set_peripheral = fsl_otg_set_peripheral;
	fsl_otg_tc->otg.set_power = fsl_otg_set_power;
	fsl_otg_tc->otg.start_hnp = fsl_otg_start_hnp;
	fsl_otg_tc->otg.start_srp = fsl_otg_start_srp;

	fsl_otg_dev = fsl_otg_tc;

	/* Store the otg transceiver */
	status = otg_set_transceiver(&fsl_otg_tc->otg);
	if (status) {
		pr_warn(AIT_OTG_NAME ": unable to register OTG transceiver.\n");
		goto err;
	}

	return 0;
err:
	fsl_otg_uninit_timers();
	kfree(fsl_otg_tc);
	return status;
}

/* OTG Initialization */
int usb_otg_start(struct platform_device *pdev)
{
	extern MMP_ERR MMPF_USB_StopDevice(void);
	extern void MMPF_USBPHY_SetSignalTune(MMP_ULONG ulTxCur, MMP_ULONG ulSQ);
	extern void USBEx_DriveVBus(MMP_BOOL on);

	struct ait_otg *p_otg;
	struct otg_transceiver *otg_trans = otg_get_transceiver();
	struct otg_fsm *fsm;
	int status;
	struct resource *res;
	u32 temp;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	AITPS_GBL pGBL = AITC_BASE_GBL;
	AITPS_USB_CTL pUSB_CTL = AITC_BASE_USBCTL;
	AITPS_USB_DMA pUSB_DMA = AITC_BASE_USBDMA;	

	p_otg = container_of(otg_trans, struct ait_otg, otg);
	fsm = &p_otg->fsm;

	/* Initialize the state machine structure with default values */
	SET_OTG_STATE(otg_trans, OTG_STATE_UNDEFINED);
	fsm->transceiver = &p_otg->otg;
#if 0
	/* We don't require predefined MEM/IRQ resource index */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	/* We don't request_mem_region here to enable resource sharing
	 * with host/device */

	usb_dr_regs = (struct AITS_USB_CTL *) ioremap(res->start, resource_size(res));// sizeof(struct AITS_USB_CTL));
#endif	
	usb_dr_regs = (struct AITS_USB_CTL *) AITC_BASE_USBCTL;
	p_otg->regs = (struct AITS_USB_CTL *)usb_dr_regs;
	pdata->regs = (void *)usb_dr_regs;

	if (pdata->init && pdata->init(pdev) != 0)
		return -EINVAL;

	if (pdata->big_endian_mmio) {
//		_fsl_readl = _fsl_readl_be;
//		_fsl_writel = _fsl_writel_be;
	} else {
//		_fsl_readl = _fsl_readl_le;
	//	_fsl_writel = _fsl_writel_le;
	}

	/* request irq */

	p_otg->irq = 12;//platform_get_irq(pdev, 0);
	status = request_irq(p_otg->irq, ait_otg_isr,
				IRQF_SHARED, driver_name, p_otg);
	if (status) {
		dev_dbg(p_otg->otg.dev, "can't get IRQ %d, error %d\n",
			p_otg->irq, status);
		iounmap(p_otg->regs);
		kfree(p_otg);
		return status;
	}

	dev_info(p_otg->otg.dev, "Request IRQ %d success!\n",p_otg->irq);

        pGBL->GBL_USB_CLK_SRC = GRP_CLK_SRC0_PMCLK | USBPHY_CLK_SRC_PMCLK_DIS;
        pGBL->GBL_USB_CLK_DIV = GRP_CLK_DIV_EN | GRP_CLK_DIV(2);


	USBEx_DriveVBus(1);
	MMPF_SYS_EnableClock(MMPF_SYS_CLK_USB,MMP_TRUE);
	/* stop the controller */
//	temp = fsl_readl(&p_otg->dr_mem_map->usbcmd);
//	temp &= ~USB_CMD_RUN_STOP;
//	fsl_writel(temp, &p_otg->dr_mem_map->usbcmd);
	//MMPF_USB_StopDevice();
	
	/* reset the controller */
//	temp = fsl_readl(&p_otg->dr_mem_map->usbcmd);
//	temp |= USB_CMD_CTRL_RESET;
//	fsl_writel(temp, &p_otg->dr_mem_map->usbcmd);
	MMPF_USBPHY_PowerDown();
  	MMPF_SYS_ResetHModule(MMPF_SYS_MDL_USB, MMP_TRUE);

	MMPF_USBPHY_SetSignalTune(7,0);	//TX CUR=480mV SQ=75mV
	MMPF_USBPHY_Initialize(1);

	/* Enable USB OTG, signal from PHY */
	pUSB_DMA->USB_OTG_CTL = OTG_EN;

	pUSB_DMA->USB_UTMI_PHY_CTL1 |=DMPULLDOWN_TO_PHY_EN;

	pUSB_CTL->USB_POWER = USB_SUSPENDM_EN|USB_HS_EN;	
	pUSB_CTL->USB_ADT_DEV_CTL |= USB_SESSION;
	
	/* wait reset completed */
//	while (fsl_readl(&p_otg->dr_mem_map->usbcmd) & USB_CMD_CTRL_RESET)
//		;

	/* configure the VBUSHS as IDLE(both host and device) */
	temp = USB_MODE_STREAM_DISABLE | (pdata->es ? USB_MODE_ES : 0);
//	fsl_writel(temp, &p_otg->dr_mem_map->usbmode);

	/* configure PHY interface */
//	temp = fsl_readl(&p_otg->dr_mem_map->portsc);
	temp &= ~(PORTSC_PHY_TYPE_SEL | PORTSC_PTW);
	switch (pdata->phy_mode) {
	case FSL_USB2_PHY_ULPI:
		temp |= PORTSC_PTS_ULPI;
		break;
	case FSL_USB2_PHY_UTMI_WIDE:
		temp |= PORTSC_PTW_16BIT;
		/* fall through */
	case FSL_USB2_PHY_UTMI:
		temp |= PORTSC_PTS_UTMI;
		/* fall through */
	default:
		break;
	}
//	fsl_writel(temp, &p_otg->dr_mem_map->portsc);

	if (pdata->have_sysif_regs) {
		/* configure control enable IO output, big endian register */
		//temp = __raw_readl(&p_otg->dr_mem_map->control);
		temp |= USB_CTRL_IOENB;
		//__raw_writel(temp, &p_otg->dr_mem_map->control);
	}

	/* disable all interrupt and clear all OTGSC status */
//	temp = fsl_readl(&p_otg->dr_mem_map->otgsc);
	temp &= ~OTGSC_INTERRUPT_ENABLE_BITS_MASK;
	temp |= OTGSC_INTERRUPT_STATUS_BITS_MASK | OTGSC_CTRL_VBUS_DISCHARGE;
//	fsl_writel(temp, &p_otg->dr_mem_map->otgsc);

	/*
	 * The identification (id) input is FALSE when a Mini-A plug is inserted
	 * in the devices Mini-AB receptacle. Otherwise, this input is TRUE.
	 * Also: record initial state of ID pin
	 */
//	if (fsl_readl(&p_otg->dr_mem_map->otgsc) & OTGSC_STS_USB_ID) {
		p_otg->otg.state = OTG_STATE_UNDEFINED;
		p_otg->fsm.id = 1;
//	} else {
//		p_otg->otg.state = OTG_STATE_A_IDLE;
//		p_otg->fsm.id = 0;
//	}

	DBG("initial ID pin=%d\n", p_otg->fsm.id);

	/* enable OTG ID pin interrupt */
//	temp = fsl_readl(&p_otg->dr_mem_map->otgsc);
	temp |= OTGSC_INTR_USB_ID_EN;
	temp &= ~(OTGSC_CTRL_VBUS_DISCHARGE | OTGSC_INTR_1MS_TIMER_EN);
//	fsl_writel(temp, &p_otg->dr_mem_map->otgsc);

	usb_dr_regs->USB_INT_EVENT_EN = 0xff;//= (AIT_REG_B)0xff;
	return 0;
}

/*
 * state file in sysfs
 */
static int show_fsl_usb2_otg_state(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct otg_fsm *fsm = &fsl_otg_dev->fsm;
	char *next = buf;
	unsigned size = PAGE_SIZE;
	unsigned long flags;
	int t;

	spin_lock_irqsave(&fsm->lock, flags);

	/* basic driver infomation */
	t = scnprintf(next, size,
			DRIVER_DESC "\n" "fsl_usb2_otg version: %s\n\n",
			DRIVER_VERSION);
	size -= t;
	next += t;

	/* Registers */
	t = scnprintf(next, size,
			"OTGSC:   0x%08x\n"
			"PORTSC:  0x%08x\n"
			"USBMODE: 0x%08x\n"
			"USBCMD:  0x%08x\n"
			"USBSTS:  0x%08x\n"
			"USBINTR: 0x%08x\n",
//			fsl_readl(&usb_dr_regs->otgsc),
//			fsl_readl(&usb_dr_regs->portsc),
//			fsl_readl(&usb_dr_regs->usbmode),
//			fsl_readl(&usb_dr_regs->usbcmd),
//			fsl_readl(&usb_dr_regs->usbsts),
//			fsl_readl(&usb_dr_regs->usbintr)
			0,0,0,0,0,0);
	size -= t;
	next += t;

	/* State */
	t = scnprintf(next, size,
		      "OTG state: %s\n\n",
		      otg_state_string(fsl_otg_dev->otg.state));
	size -= t;
	next += t;

	/* State Machine Variables */
	t = scnprintf(next, size,
			"a_bus_req: %d\n"
			"b_bus_req: %d\n"
			"a_bus_resume: %d\n"
			"a_bus_suspend: %d\n"
			"a_conn: %d\n"
			"a_sess_vld: %d\n"
			"a_srp_det: %d\n"
			"a_vbus_vld: %d\n"
			"b_bus_resume: %d\n"
			"b_bus_suspend: %d\n"
			"b_conn: %d\n"
			"b_se0_srp: %d\n"
			"b_sess_end: %d\n"
			"b_sess_vld: %d\n"
			"id: %d\n",
			fsm->a_bus_req,
			fsm->b_bus_req,
			fsm->a_bus_resume,
			fsm->a_bus_suspend,
			fsm->a_conn,
			fsm->a_sess_vld,
			fsm->a_srp_det,
			fsm->a_vbus_vld,
			fsm->b_bus_resume,
			fsm->b_bus_suspend,
			fsm->b_conn,
			fsm->b_se0_srp,
			fsm->b_sess_end,
			fsm->b_sess_vld,
			fsm->id);
	size -= t;
	next += t;

	spin_unlock_irqrestore(&fsm->lock, flags);

	return PAGE_SIZE - size;
}

static DEVICE_ATTR(fsl_usb2_otg_state, S_IRUGO, show_fsl_usb2_otg_state, NULL);


/* Char driver interface to control some OTG input */

/*
 * Handle some ioctl command, such as get otg
 * status and set host suspend
 */
static long fsl_otg_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	u32 retval = 0;

	switch (cmd) {
	case GET_OTG_STATUS:
		retval = fsl_otg_dev->host_working;
		break;

	case SET_A_SUSPEND_REQ:
		fsl_otg_dev->fsm.a_suspend_req = arg;
		break;

	case SET_A_BUS_DROP:
		fsl_otg_dev->fsm.a_bus_drop = arg;
		break;

	case SET_A_BUS_REQ:
		fsl_otg_dev->fsm.a_bus_req = arg;
		break;

	case SET_B_BUS_REQ:
		fsl_otg_dev->fsm.b_bus_req = arg;
		break;

	default:
		break;
	}

	//otg_statemachine(&fsl_otg_dev->fsm);

	return retval;
}

static int fsl_otg_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int fsl_otg_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations otg_fops = {
	.owner = THIS_MODULE,
	.llseek = NULL,
	.read = NULL,
	.write = NULL,
	//.unlocked_ioctl = fsl_otg_ioctl,
	.open = fsl_otg_open,
	.release = fsl_otg_release,
};

static int __devinit ait_otg_probe(struct platform_device *pdev)
{
	int ret;

	if (!pdev->dev.platform_data)
		return -ENODEV;

	/* configure the OTG */
	ret = ait_otg_conf(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't configure OTG module\n");
		return ret;
	}

	/* start OTG */
	ret = usb_otg_start(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Can't init FSL OTG device\n");
		return ret;
	}

	ret = register_chrdev(FSL_OTG_MAJOR, AIT_OTG_NAME , &otg_fops);
	if (ret) {
		dev_err(&pdev->dev, "unable to register FSL OTG device\n");
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_fsl_usb2_otg_state);
	if (ret)
		dev_warn(&pdev->dev, "Can't register sysfs attribute\n");

	return ret;
}

static int __devexit ait_otg_remove(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	otg_set_transceiver(NULL);
	free_irq(fsl_otg_dev->irq, fsl_otg_dev);

	iounmap((void *)usb_dr_regs);

	fsl_otg_uninit_timers();
	kfree(fsl_otg_dev);

	device_remove_file(&pdev->dev, &dev_attr_fsl_usb2_otg_state);

	unregister_chrdev(FSL_OTG_MAJOR, AIT_OTG_NAME );

	if (pdata->exit)
		pdata->exit(pdev);

	return 0;
}

struct platform_driver ait_otg_driver = {
	.probe = ait_otg_probe,
	.remove = __devexit_p(ait_otg_remove),
	.driver = {
		.name = driver_name,
		.owner = THIS_MODULE,
	},
};

static int __init ait_usb_otg_init(void)
{
	pr_info(DRIVER_INFO "\n");
	return platform_driver_register(&ait_otg_driver);
}
module_init(ait_usb_otg_init);

static void __exit ait_usb_otg_exit(void)
{
	platform_driver_unregister(&ait_otg_driver);
}
module_exit(ait_usb_otg_exit);

MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
