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
#ifndef CPUB_AES_CPUCOMM_H
#define CPUB_AES_CPUCOMM_H
#include  "aes.h"

extern int cpub_aes_run(struct AES_proc_info_s *aes_proc);
extern int cpub_aes_init(struct AES_init_s *aes_init);
extern int cpub_aes_register(int id);
extern int cpub_aes_unregister(void);

#endif
