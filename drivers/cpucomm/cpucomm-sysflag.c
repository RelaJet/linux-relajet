/* cpucomm-sysflag.c - a device driver for the iic-bus interface		     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2014 Chiket Lin

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     */
/* ------------------------------------------------------------------------- */

#include "cpucomm-sysflag.h"


// current implemntation of the sys flag is simple. We may need implement it as a formal device driver for normal shut down sequence.

void sysflag_init(void)
{
    // register sysflag
    CpuComm_RegisterEntry(CPU_COMM_ID_SYSFLAG, CPU_COMM_TYPE_SEM );

    // Notify the cpucomm init is done
#ifdef CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUB
    CpuComm_SemPost(CPU_COMM_ID_SYSFLAG, 0);
#endif
    // we need a way to extablish the link between cpucomm semaphore and system event.
    // ??
}

