#
# Makefile for the cpucomm core.
#

obj-$(CONFIG_CPUCOMM)	+= cpucomm-boardinfo.o
obj-$(CONFIG_CPUCOMM)	+= cpucomm-bus.o
obj-$(CONFIG_CPUCOMM)	+= cpucomm-adapter.o
obj-$(CONFIG_CPUCOMM)	+= cpucomm_core.o
obj-$(CONFIG_CPUCOMM)	+= cpucomm-sysflag.o
obj-$(CONFIG_CPUCOMM)	+= cpucomm-uart.o
#obj-$(CONFIG_CPUCOMM)	+= cpucomm-cputest.o

obj-$(CONFIG_CPU_SHARE_MEM)	+= cpu_share_mem.o
obj-$(CONFIG_CPU_SHARE_MEM_TEST)	+= cpu_share_mem_test.o

# ccflags-$(CONFIG_I2C_DEBUG_CORE) := -DDEBUG
# CFLAGS_cpucomm-core.o := -Wno-deprecated-declarations
