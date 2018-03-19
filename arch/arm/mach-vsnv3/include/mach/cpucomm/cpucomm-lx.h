/* ------------------------------------------------------------------------- */
/*									     */
/* cpucomm.h - definitions for the cpucomm-bus interface			     */
/*									     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 1995-2000 Simon G. Vogl

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

/* With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and
   Frodo Looijaard <frodol@dds.nl> */

#ifndef _LINUX_CPUCOMM_H_
#define _LINUX_CPUCOMM_H_

#if defined(CONFIG_CPUCOMM)


#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/mod_devicetable.h>
#include <linux/device.h>	/* for struct device */
#include <linux/sched.h>	/* for completion */
#include <linux/mutex.h>
#include <linux/of.h>		/* for struct device_node */
#include <linux/swab.h>		/* for swab16 */
#include <mach/cpucomm/cpucomm_if.h>
#include <mach/cpucomm/cpucomm-uart.h>

/* CPUCOMM */

#define CPUCOMM_NAME_SIZE	32
#define CPUCOMM_MODULE_PREFIX "cpucomm:"

struct cpucomm_device_id {
	char name[CPUCOMM_NAME_SIZE];
	kernel_ulong_t driver_data	/* Data private to the driver */
			__attribute__((aligned(sizeof(kernel_ulong_t))));
};


extern struct bus_type cpucomm_bus_type;
extern struct device_type cpucomm_adapter_type;

/* --- General options ------------------------------------------------	*/

struct cpucomm_msg;
struct cpucomm_algorithm;
struct cpucomm_adapter;
struct cpucomm_client;
struct cpucomm_driver;
union cpucomm_smbus_data;
struct cpucomm_board_info;

struct module;

/**
 * struct cpucomm_driver - represent an CPUCOMM device driver
 * @class: What kind of cpucomm device we instantiate (for detect)
 * @attach_adapter: Callback for bus addition (deprecated)
 * @detach_adapter: Callback for bus removal (deprecated)
 * @probe: Callback for device binding
 * @remove: Callback for device unbinding
 * @shutdown: Callback for device shutdown
 * @suspend: Callback for device suspend
 * @resume: Callback for device resume
 * @alert: Alert callback, for example for the SMBus alert protocol
 * @command: Callback for bus-wide signaling (optional)
 * @driver: Device driver model driver
 * @id_table: List of CPUCOMM devices supported by this driver
 * @clients: List of detected clients we created (for cpucomm-core use only)
 *
 * The driver.owner field should be set to the module owner of this driver.
 * The driver.name field should be set to the name of this driver.
 *
 * For automatic device detection, both @detect and @address_data must
 * be defined. @class should also be set, otherwise only devices forced
 * with module parameters will be created. The detect function must
 * fill at least the name field of the cpucomm_board_info structure it is
 * handed upon successful detection, and possibly also the flags field.
 *
 * If @detect is missing, the driver will still work fine for enumerated
 * devices. Detected devices simply won't be supported. This is expected
 * for the many CPUCOMM/SMBus devices which can't be detected reliably, and
 * the ones which can always be enumerated in practice.
 *
 * The cpucomm_client structure which is handed to the @detect callback is
 * not a real cpucomm_client. It is initialized just enough so that you can
 * call cpucomm_smbus_read_byte_data and friends on it. Don't do anything
 * else with it. In particular, calling dev_dbg and friends on it is
 * not allowed.
 */
struct cpucomm_driver {
	unsigned int class;

	/* Standard driver model interfaces */
	int (*probe)(struct cpucomm_client * /*, const struct cpucomm_device_id * */);
	int (*remove)(struct cpucomm_client *);

	/* driver model interfaces that don't relate to enumeration  */
	void (*shutdown)(struct cpucomm_client *);
	int (*suspend)(struct cpucomm_client *, pm_message_t mesg);
	int (*resume)(struct cpucomm_client *);

	struct device_driver driver;
	const struct cpucomm_device_id *id_table;
};
#define to_cpucomm_driver(d) container_of(d, struct cpucomm_driver, driver)

/**
 * struct cpucomm_client - represent an CPUCOMM slave device
 * @flags: CPUCOMM_CLIENT_TEN indicates the device uses a ten bit chip address;
 *	CPUCOMM_CLIENT_PEC indicates it uses SMBus Packet Error Checking
 * @addr: Address used on the CPUCOMM bus connected to the parent adapter.
 * @name: Indicates the type of the device, usually a chip name that's
 *	generic enough to hide second-sourcing and compatible revisions.
 * @adapter: manages the bus segment hosting this CPUCOMM device
 * @driver: device's driver, hence pointer to access routines
 * @dev: Driver model device node for the slave.
 * @irq: indicates the IRQ generated by this device (if any)
 * @detected: member of an cpucomm_driver.clients list or cpucomm-core's
 *	userspace_devices list
 *
 * An cpucomm_client identifies a single device (i.e. chip) connected to an
 * cpucomm bus. The behaviour exposed to Linux is defined by the driver
 * managing the device.
 */
struct cpucomm_client {
	unsigned short type;
	char name[CPUCOMM_NAME_SIZE];
	struct cpucomm_adapter *adapter;	/* the adapter we sit on	*/
	struct cpucomm_driver *driver;	/* and our access routines	*/
	struct device dev;		/* the device structure		*/
    const struct cpucomm_device_id	*id_entry;  /* the cpucomm id matched   */	
};
#define to_cpucomm_client(d) container_of(d, struct cpucomm_client, dev)

static inline const struct cpucomm_device_id* cpucomm_get_device_id(struct cpucomm_client *client)
{
    return client->id_entry;
}

extern struct cpucomm_client *cpucomm_verify_client(struct device *dev);

static inline struct cpucomm_client *kobj_to_cpucomm_client(struct kobject *kobj)
{
	struct device * const dev = container_of(kobj, struct device, kobj);
	return to_cpucomm_client(dev);
}

static inline void *cpucomm_get_clientdata(const struct cpucomm_client *client)
{
	return dev_get_drvdata(&client->dev);
}

static inline void cpucomm_set_clientdata(struct cpucomm_client *client, void *data)
{
	dev_set_drvdata(&client->dev, data);
}

/**
 * struct cpucomm_board_info - template for device creation
 * @type: chip type, to initialize cpucomm_client.name
 * @flags: to initialize cpucomm_client.flags
 * @addr: stored in cpucomm_client.addr
 * @platform_data: stored in cpucomm_client.dev.platform_data
 * @archdata: copied into cpucomm_client.dev.archdata
 * @of_node: pointer to OpenFirmware device node
 * @irq: stored in cpucomm_client.irq
 *
 * CPUCOMM doesn't actually support hardware probing, although controllers and
 * devices may be able to use CPUCOMM_SMBUS_QUICK to tell whether or not there's
 * a device at a given address.  Drivers commonly need more information than
 * that, such as chip type, configuration, associated IRQ, and so on.
 *
 * cpucomm_board_info is used to build tables of information listing CPUCOMM devices
 * that are present.  This information is used to grow the driver model tree.
 * For mainboards this is done statically using cpucomm_register_board_info();
 * bus numbers identify adapters that aren't yet available.  For add-on boards,
 * cpucomm_new_device() does this dynamically with the adapter already known.
 */
struct cpucomm_board_info {
	const char	* name;//char		name[CPUCOMM_NAME_SIZE];
	int		    id;
	void		*platform_data;
	struct dev_archdata	*archdata;
	struct device_node *of_node;
};

/**
 * CPUCOMM_BOARD_INFO - macro used to list an cpucomm device and its address
 * @dev_type: identifies the device type
 * @dev_addr: the device's address on the bus.
 *
 * This macro initializes essential fields of a struct cpucomm_board_info,
 * declaring what has been provided on a particular board.  Optional
 * fields (such as associated irq, or device-specific platform_data)
 * are provided using conventional syntax.
 */
#define CPUCOMM_BOARD_INFO(dev_name) \
	.name = dev_name


/* Add-on boards should register/unregister their devices; e.g. a board
 * with integrated CPUCOMM, a config eeprom, sensors, and a codec that's
 * used in conjunction with the primary hardware.
 */
extern struct cpucomm_client *
cpucomm_new_device(struct cpucomm_adapter *adap, struct cpucomm_board_info const *info);

/* If you don't know the exact address of an CPUCOMM device, use this variant
 * instead, which can probe for device presence in a list of possible
 * addresses. The "probe" callback function is optional. If it is provided,
 * it must return 1 on successful probe, 0 otherwise. If it is not provided,
 * a default probing method is used.
 */
extern struct cpucomm_client *
cpucomm_new_probed_device(struct cpucomm_adapter *adap,
		      struct cpucomm_board_info *info,
		      unsigned short const *addr_list,
		      int (*probe)(struct cpucomm_adapter *, unsigned short addr));

/* Common custom probe functions */
extern int cpucomm_probe_func_quick_read(struct cpucomm_adapter *, unsigned short addr);

/* For devices that use several addresses, use cpucomm_new_dummy() to make
 * client handles for the extra addresses.
 */
extern struct cpucomm_client *
cpucomm_new_dummy(struct cpucomm_adapter *adap, u16 address);

extern void cpucomm_unregister_device(struct cpucomm_client *);

/* Mainboard arch_initcall() code should register all its CPUCOMM devices.
 * This is done at arch_initcall time, before declaring any cpucomm adapters.
 * Modules for add-on boards must use other calls.
 */
extern int
cpucomm_register_board_info(struct cpucomm_board_info const *info, unsigned n);

/*
 * The following structs are for those who like to implement new bus drivers:
 * cpucomm_algorithm is the interface to a class of hardware solutions which can
 * be addressed using the same bus algorithms - i.e. bit-banging or the PCF8584
 * to name two of the most common.
 */
struct cpucomm_algorithm {
    CPU_COMM_ERR (*register_entry)(struct cpucomm_adapter *adap, CPU_COMM_ID id, CPU_COMM_TYPE type);
    CPU_COMM_ERR (*unregister_entry)(struct cpucomm_adapter *adap, CPU_COMM_ID id);
    CPU_COMM_ERR (*sem_post)(struct cpucomm_adapter *adap, CPU_COMM_ID id, unsigned int timeout);
    CPU_COMM_ERR (*sem_wait)(struct cpucomm_adapter *adap, CPU_COMM_ID id, unsigned int timeout);
    CPU_COMM_ERR (*data_send)(struct cpucomm_adapter *adap, CPU_COMM_ID id, void *data, unsigned int size, unsigned int timeout);
    CPU_COMM_ERR (*data_receive)(struct cpucomm_adapter *adap, CPU_COMM_ID id, void *data, unsigned int size, unsigned int timeout, CPUCOMM_RCV_PREPROC preproc);
};

/*
 * cpucomm_adapter is the structure used to identify a physical cpucomm bus along
 * with the access algorithms necessary to access it.
 */
struct cpucomm_adapter {
	const struct cpucomm_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices	*/
	struct rt_mutex bus_lock;

	int timeout;			/* in jiffies */
	int retries;
	struct device dev;		/* the adapter device */

	char name[48];
	struct completion dev_released;

	void __iomem *baseaddr_hint;
	volatile void __iomem *baseaddr_share;	
	
};
#define to_cpucomm_adapter(d) container_of(d, struct cpucomm_adapter, dev)

static inline void *cpucomm_get_adapdata(const struct cpucomm_adapter *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void cpucomm_set_adapdata(struct cpucomm_adapter *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

static inline struct cpucomm_adapter *
cpucomm_parent_is_cpucomm_adapter(const struct cpucomm_adapter *adapter)
{
	struct device *parent = adapter->dev.parent;

	if (parent != NULL && parent->type == &cpucomm_adapter_type)
		return to_cpucomm_adapter(parent);
	else
		return NULL;
}

int cpucomm_for_each_dev(void *data, int (*fn)(struct device *, void *));

/* Adapter locking functions, exported for shared pin cases */
void cpucomm_lock_adapter(struct cpucomm_adapter *);
void cpucomm_unlock_adapter(struct cpucomm_adapter *);

/* ----- functions exported by cpucomm.o */

/* administration...
 */
extern int cpucomm_register_adapter(struct cpucomm_adapter *adapter);
extern int cpucomm_unregister_adapter(struct cpucomm_adapter *adapter);

extern int cpucomm_register_driver(struct module *, struct cpucomm_driver *);
extern void cpucomm_unregister_driver(struct cpucomm_driver *);

/* use a define to avoid include chaining to get THIS_MODULE */
#define cpucomm_add_driver(driver) \
	cpucomm_register_driver(THIS_MODULE, driver)

#endif /* __KERNEL__ */

static inline CPU_COMM_ERR cpucomm_register_entry(const struct cpucomm_client *client, CPU_COMM_ID id, CPU_COMM_TYPE type)
{
    return client->adapter->algo->register_entry(client->adapter, id, type);
}

static inline CPU_COMM_ERR cpucomm_unregister_entry(const struct cpucomm_client *client, CPU_COMM_ID id)
{
    return client->adapter->algo->unregister_entry(client->adapter, id);
}

static inline CPU_COMM_ERR cpucomm_sem_post(const struct cpucomm_client *client, CPU_COMM_ID id, unsigned int timeout)
{
    return client->adapter->algo->sem_post(client->adapter, id, timeout);
}

static inline CPU_COMM_ERR cpucomm_sem_wait(const struct cpucomm_client *client, CPU_COMM_ID id, unsigned int timeout)
{
    return client->adapter->algo->sem_wait(client->adapter, id, timeout);
}

static inline CPU_COMM_ERR cpucomm_data_send(const struct cpucomm_client *client, CPU_COMM_ID id, void *data, unsigned int size, unsigned int timeout)
{
    return client->adapter->algo->data_send( client->adapter, id, data, size, timeout);
}

static inline CPU_COMM_ERR cpucomm_data_receive(const struct cpucomm_client *client, CPU_COMM_ID id, void *data, unsigned int size, unsigned int timeout, CPUCOMM_RCV_PREPROC preproc)
{
    return client->adapter->algo->data_receive(client->adapter, id, data, size, timeout, preproc);
}


#endif /* CONFIG_CPUCOMM */

#endif /* _LINUX_CPUCOMM_H_ */
