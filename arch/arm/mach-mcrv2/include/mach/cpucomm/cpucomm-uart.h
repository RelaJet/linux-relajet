/*
 * cpucomm-uart.h virtual uart driver on cpucomm bus
 *
 * Based on include/linux/platform_data/atmel.h (from linux 3.15)
 *
 * Author:	Chiket
 * Created:	Jan 9, 2015
 * Copyright:	A.I.T Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef __CPU_COMM_UART_H__
#define __CPU_COMM_UART_H__


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <mach/cpucomm/cpucomm_id.h>

struct cpucomm_uart_data {

    CPU_COMM_ID         id;         /* Cpucomm Entry ID */
    resource_size_t     work_buf;   /* Working buffer for two CPU */
};

#endif // __CPU_COMM_UART_H__
