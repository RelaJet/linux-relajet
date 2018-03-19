/*
 * cpucomm-core.h - interfaces internal to the CPUCOMM framework
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

#ifndef _LINUX_CPUCOMM_CORE_H
#define _LINUX_CPUCOMM_CORE_H

#include <linux/rwsem.h>

struct cpucomm_devinfo {
	struct list_head	list;
	struct cpucomm_board_info	board_info;
};

/* board_lock protects board_list and first_dynamic_bus_num.
 * only cpucomm core components are allowed to use these symbols.
 */
extern struct rw_semaphore	__cpucomm_board_lock;
extern struct list_head	__cpucomm_board_list;

#endif /* _LINUX_CPUCOMM_CORE_H */
