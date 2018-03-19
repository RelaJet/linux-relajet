/*
 * Driver for MD alog. on AIT CPUB
 *
 * Copyright (C) 2015 Vincent Chen @ AIT
 *
 * arch/arm/mach-vsnv3/cpucomm/cpub_md.c which has following copyrights:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef CPUB_AAC_CPUCOMM_H
#define CPUB_AAC_CPUCOMM_H
#include  "aac.h"

extern int cpub_aac_run(int sid,int ack_off);
extern int cpub_aac_init(int sid,struct AAC_init_s *aac_init);
extern int cpub_aac_register(int id);
extern int cpub_aac_unregister(void);

#endif
