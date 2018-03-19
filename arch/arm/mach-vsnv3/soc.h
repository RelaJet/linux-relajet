/*
 * Copyright (C) 2011 Jean-Christophe PLAGNIOL-VILLARD <plagnioj@jcrosoft.com>
 *
 * Under GPLv2
 */

struct ait_init_soc {
	unsigned int *default_irq_priority;
	void (*map_io)(void);
	void (*register_clocks)(void);
	void (*init)(void);
};

extern struct ait_init_soc ait_boot_soc;
extern struct ait_init_soc ait8x_soc;

static inline int ait_soc_is_enabled(void)
{
	return ait_boot_soc.init != NULL;
}

#if !defined(CONFIG_ARCH_VSNV3AIT845X)
#define ait8x_soc       ait_boot_soc
#endif
