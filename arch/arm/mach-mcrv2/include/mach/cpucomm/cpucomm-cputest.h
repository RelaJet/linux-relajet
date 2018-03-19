/*
 * cpucomm-cputest.h device data for cpucomm test
 *
 * Author:	Chiket
 * Created:	Mar. 26, 2015
 * Copyright:	A.I.T Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef __CPU_COMM_CPUTEST_INFO_H__
#define __CPU_COMM_CPUTEST_INFO_H__


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include "cpucomm_id.h"

typedef enum _CPU_TEST_TYPE {
    CPU_TEST_TYPE_DATA_SENDER   = 0,
    CPU_TEST_TYPE_DATA_RECEIVER = 1,
    CPU_TEST_TYPE_SEM_SENDER    = 2,
    CPU_TEST_TYPE_SEM_RECEIVER  = 3,
    CPU_TEST_TYPE_MAX           = 4
} CPU_TEST_TYPE;

struct cpucomm_cputest_dev_data {
    CPU_COMM_ID         comm_id;     /* Cpucomm Entry ID */
};

#endif // __CPU_COMM_CPUTEST_DATA_H__
