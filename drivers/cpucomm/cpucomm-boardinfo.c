/*
 * cpucomm-boardinfo.c - collect pre-declarations of CPUCOMM devices
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/rwsem.h>

#include <mach/cpucomm/cpucomm-lx.h>
#include "cpucomm-bus.h"


/* These symbols are exported ONLY FOR the cpucomm core.
 * No other users will be supported.
 */
DECLARE_RWSEM(__cpucomm_board_lock);
EXPORT_SYMBOL_GPL(__cpucomm_board_lock);

LIST_HEAD(__cpucomm_board_list);
EXPORT_SYMBOL_GPL(__cpucomm_board_list);

/**
 * cpucomm_register_board_info - statically declare CPUCOMM devices
 * @busnum: identifies the bus to which these devices belong
 * @info: vector of cpucomm device descriptors
 * @len: how many descriptors in the vector; may be zero to reserve
 *	the specified bus number.
 *
 * Systems using the Linux CPUCOMM driver stack can declare tables of board info
 * while they initialize.  This should be done in board-specific init code
 * near arch_initcall() time, or equivalent, before any CPUCOMM adapter driver is
 * registered.  For example, mainboard init code could define several devices,
 * as could the init code for each daughtercard in a board stack.
 *
 * The CPUCOMM devices will be created later, after the adapter for the relevant
 * bus has been registered.  After that moment, standard driver model tools
 * are used to bind "new style" CPUCOMM drivers to the devices.  The bus number
 * for any device declared using this routine is not available for dynamic
 * allocation.
 *
 * The board info passed can safely be __initdata, but be careful of embedded
 * pointers (for platform_data, functions, etc) since that won't be copied.
 */
int __init
cpucomm_register_board_info(struct cpucomm_board_info const *info, unsigned len)
{
	int status;

	down_write(&__cpucomm_board_lock);

	for (status = 0; len; len--, info++) {
		struct cpucomm_devinfo	*devinfo;

		devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
		if (!devinfo) {
			pr_debug("cpucomm-core: can't register boardinfo!\n");
			status = -ENOMEM;
			goto err;
		}

		devinfo->board_info = *info;
		list_add_tail(&devinfo->list, &__cpucomm_board_list);
	}

err:

	up_write(&__cpucomm_board_lock);

	return status;
}
