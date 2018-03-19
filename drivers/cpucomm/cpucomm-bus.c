/* cpucomm-bus.c - a device driver for the cpucomm bus interface		     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2014 Chiket Lin

    Based on drivers/i2c/i2c-core.c

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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/completion.h>
#include <linux/hardirq.h>
#include <linux/irqflags.h>
#include <linux/rwsem.h>
#include <linux/pm_runtime.h>
#include <asm/uaccess.h>

#include <mach/cpucomm/cpucomm-lx.h>
#include "cpucomm-bus.h"


/* core_lock protects cpucomm_adapter_idr, and guarantees
   that device detection, deletion of detected devices, and attach_adapter
   and detach_adapter calls are serialized */
static DEFINE_MUTEX(core_lock);
// static struct cpucomm_adapter *_cpucomm_adapter = NULL;


static struct device_type cpucomm_client_type;

/* ------------------------------------------------------------------------- */

static const struct cpucomm_device_id *cpucomm_match_id(const struct cpucomm_device_id *id,
						struct cpucomm_client *client)
{
	while (id->name[0]) {
		if (strcmp(client->name, id->name) == 0) {
            client->id_entry = id;
			return id;
        }
		id++;
	}
	return NULL;
}

static int cpucomm_device_match(struct device *dev, struct device_driver *drv)
{
	struct cpucomm_client	*client = cpucomm_verify_client(dev);
	struct cpucomm_driver	*driver;

	if (!client)
		return 0;

	/* Attempt an OF style match */
	if (of_driver_match_device(dev, drv))
		return 1;

	driver = to_cpucomm_driver(drv);
	/* match on an id table if there is one */
	if (driver->id_table)
		return cpucomm_match_id(driver->id_table, client) != NULL;

    /* fall-back to driver name match */
    return (strcmp(client->name, drv->name) == 0);
}

#ifdef	CONFIG_HOTPLUG

/* uevent helps with hotplug: modprobe -q $(MODALIAS) */
static int cpucomm_device_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct cpucomm_client	*client = to_cpucomm_client(dev);

	if (add_uevent_var(env, "MODALIAS=%s%s",
			   CPUCOMM_MODULE_PREFIX, client->name))
		return -ENOMEM;
	dev_dbg(dev, "uevent\n");
	return 0;
}

#else
#define cpucomm_device_uevent	NULL
#endif	/* CONFIG_HOTPLUG */

static int cpucomm_device_probe(struct device *dev)
{
	struct cpucomm_client	*client = cpucomm_verify_client(dev);
	struct cpucomm_driver	*driver;
	int status;

	if (!client)
		return 0;

	driver = to_cpucomm_driver(dev->driver);
	if (!driver->probe)
		return -ENODEV;
	client->driver = driver;
	if (!device_can_wakeup(&client->dev))
		device_init_wakeup(&client->dev, true );
	dev_dbg(dev, "probe\n");

	status = driver->probe(client /*, cpucomm_match_id(driver->id_table, client)*/);
	if (status) {
		client->driver = NULL;
		cpucomm_set_clientdata(client, NULL);
	}
	return status;
}

static int cpucomm_device_remove(struct device *dev)
{
	struct cpucomm_client	*client = cpucomm_verify_client(dev);
	struct cpucomm_driver	*driver;
	int			status;

	if (!client || !dev->driver)
		return 0;

	driver = to_cpucomm_driver(dev->driver);
	if (driver->remove) {
		dev_dbg(dev, "remove\n");
		status = driver->remove(client);
	} else {
		dev->driver = NULL;
		status = 0;
	}
	if (status == 0) {
		client->driver = NULL;
		cpucomm_set_clientdata(client, NULL);
	}
	return status;
}

static void cpucomm_device_shutdown(struct device *dev)
{
	struct cpucomm_client *client = cpucomm_verify_client(dev);
	struct cpucomm_driver *driver;

	if (!client || !dev->driver)
		return;
	driver = to_cpucomm_driver(dev->driver);
	if (driver->shutdown)
		driver->shutdown(client);
}

#ifdef CONFIG_PM_SLEEP
static int cpucomm_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct cpucomm_client *client = cpucomm_verify_client(dev);
	struct cpucomm_driver *driver;

	if (!client || !dev->driver)
		return 0;
	driver = to_cpucomm_driver(dev->driver);
	if (!driver->suspend)
		return 0;
	return driver->suspend(client, mesg);
}

static int cpucomm_legacy_resume(struct device *dev)
{
	struct cpucomm_client *client = cpucomm_verify_client(dev);
	struct cpucomm_driver *driver;

	if (!client || !dev->driver)
		return 0;
	driver = to_cpucomm_driver(dev->driver);
	if (!driver->resume)
		return 0;
	return driver->resume(client);
}

static int cpucomm_device_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_suspend(dev);
	else
		return cpucomm_legacy_suspend(dev, PMSG_SUSPEND);
}

static int cpucomm_device_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_resume(dev);
	else
		return cpucomm_legacy_resume(dev);
}

static int cpucomm_device_pm_freeze(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_freeze(dev);
	else
		return cpucomm_legacy_suspend(dev, PMSG_FREEZE);
}

static int cpucomm_device_pm_thaw(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_thaw(dev);
	else
		return cpucomm_legacy_resume(dev);
}

static int cpucomm_device_pm_poweroff(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_poweroff(dev);
	else
		return cpucomm_legacy_suspend(dev, PMSG_HIBERNATE);
}

static int cpucomm_device_pm_restore(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_restore(dev);
	else
		return cpucomm_legacy_resume(dev);
}
#else /* !CONFIG_PM_SLEEP */
#define cpucomm_device_pm_suspend	NULL
#define cpucomm_device_pm_resume	NULL
#define cpucomm_device_pm_freeze	NULL
#define cpucomm_device_pm_thaw	NULL
#define cpucomm_device_pm_poweroff	NULL
#define cpucomm_device_pm_restore	NULL
#endif /* !CONFIG_PM_SLEEP */

static void cpucomm_client_dev_release(struct device *dev)
{
	kfree(to_cpucomm_client(dev));
}

static ssize_t
show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dev->type == &cpucomm_client_type ?
		       to_cpucomm_client(dev)->name : to_cpucomm_adapter(dev)->name);
}

static ssize_t
show_modalias(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cpucomm_client *client = to_cpucomm_client(dev);
	return sprintf(buf, "%s%s\n", CPUCOMM_MODULE_PREFIX, client->name);
}

static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
static DEVICE_ATTR(modalias, S_IRUGO, show_modalias, NULL);

static struct attribute *cpucomm_dev_attrs[] = {
	&dev_attr_name.attr,
	/* modalias helps coldplug:  modprobe $(cat .../modalias) */
	&dev_attr_modalias.attr,
	NULL
};

static struct attribute_group cpucomm_dev_attr_group = {
	.attrs		= cpucomm_dev_attrs,
};

static const struct attribute_group *cpucomm_dev_attr_groups[] = {
	&cpucomm_dev_attr_group,
	NULL
};

static const struct dev_pm_ops cpucomm_device_pm_ops = {
	.suspend = cpucomm_device_pm_suspend,
	.resume = cpucomm_device_pm_resume,
	.freeze = cpucomm_device_pm_freeze,
	.thaw = cpucomm_device_pm_thaw,
	.poweroff = cpucomm_device_pm_poweroff,
	.restore = cpucomm_device_pm_restore,
	SET_RUNTIME_PM_OPS(
		pm_generic_runtime_suspend,
		pm_generic_runtime_resume,
		pm_generic_runtime_idle
	)
};

struct bus_type cpucomm_bus_type = {
	.name		= "cpucomm",
	.match		= cpucomm_device_match,
	.probe		= cpucomm_device_probe,
	.remove		= cpucomm_device_remove,
	.shutdown	= cpucomm_device_shutdown,
	.pm		= &cpucomm_device_pm_ops,
};
EXPORT_SYMBOL_GPL(cpucomm_bus_type);

static struct device_type cpucomm_client_type = {
	.groups		= cpucomm_dev_attr_groups,
	.uevent		= cpucomm_device_uevent,
	.release	= cpucomm_client_dev_release,
};


/**
 * cpucomm_verify_client - return parameter as cpucomm_client, or NULL
 * @dev: device, probably from some driver model iterator
 *
 * When traversing the driver model tree, perhaps using driver model
 * iterators like @device_for_each_child(), you can't assume very much
 * about the nodes you find.  Use this function to avoid oopses caused
 * by wrongly treating some non-CPUCOMM device as an cpucomm_client.
 */
struct cpucomm_client *cpucomm_verify_client(struct device *dev)
{
	return (dev->type == &cpucomm_client_type)
			? to_cpucomm_client(dev)
			: NULL;
}
EXPORT_SYMBOL(cpucomm_verify_client);

/**
 * cpucomm_lock_adapter - Get exclusive access to an CPUCOMM bus segment
 * @adapter: Target CPUCOMM bus segment
 */
void cpucomm_lock_adapter(struct cpucomm_adapter *adapter)
{
	struct cpucomm_adapter *parent = cpucomm_parent_is_cpucomm_adapter(adapter);

	if (parent)
		cpucomm_lock_adapter(parent);
	else
		rt_mutex_lock(&adapter->bus_lock);
}
EXPORT_SYMBOL_GPL(cpucomm_lock_adapter);

/**
 * cpucomm_new_device - instantiate an cpucomm device
 * @adap: the adapter managing the device
 * @info: describes one CPUCOMM device; bus_num is ignored
 * Context: can sleep
 *
 * Create an cpucomm device. Binding is handled through driver model
 * probe()/remove() methods.  A driver may be bound to this device when we
 * return from this function, or any later moment (e.g. maybe hotplugging will
 * load the driver module).  This call is not appropriate for use by mainboard
 * initialization logic, which usually runs during an arch_initcall() long
 * before any cpucomm_adapter could exist.
 *
 * This returns the new cpucomm client, which may be saved for later use with
 * cpucomm_unregister_device(); or NULL to indicate an error.
 */
struct cpucomm_client *
cpucomm_new_device(struct cpucomm_adapter *adap, struct cpucomm_board_info const *info)
{
	struct cpucomm_client	*client;
	int			status;

	client = kzalloc(sizeof *client, GFP_KERNEL);
	if (!client)
		return NULL;

	client->adapter = adap;

	client->dev.platform_data = info->platform_data;

	if (info->archdata)
		client->dev.archdata = *info->archdata;

	strlcpy(client->name, info->name, sizeof(client->name));

	client->dev.parent = &client->adapter->dev;
	client->dev.bus = &cpucomm_bus_type;
	client->dev.type = &cpucomm_client_type;
	client->dev.of_node = info->of_node;

    if( info->id < 0 )
    	dev_set_name(&client->dev, "%s", client->name);
    else
        dev_set_name(&client->dev, "%s-%d", client->name, info->id);
    
	status = device_register(&client->dev);
	if (status)
		goto out_err;

	dev_dbg(&adap->dev, "client [%s] registered with bus id %s\n",
		client->name, dev_name(&client->dev));

	return client;

out_err:
	dev_err(&adap->dev, "Failed to register cpucomm client %s "
		"(%d)\n", client->name, status);
	kfree(client);
	return NULL;
}
EXPORT_SYMBOL_GPL(cpucomm_new_device);


/**
 * cpucomm_unregister_device - reverse effect of cpucomm_new_device()
 * @client: value returned from cpucomm_new_device()
 * Context: can sleep
 */
void cpucomm_unregister_device(struct cpucomm_client *client)
{
	device_unregister(&client->dev);
}
EXPORT_SYMBOL_GPL(cpucomm_unregister_device);


/* ------------------------------------------------------------------------- */

/* CPUCOMM bus adapters -- one roots each CPUCOMM or SMBUS segment */

static void cpucomm_adapter_dev_release(struct device *dev)
{
	struct cpucomm_adapter *adap = to_cpucomm_adapter(dev);
	complete(&adap->dev_released);
}

static struct attribute *cpucomm_adapter_attrs[] = {
	&dev_attr_name.attr,
	NULL
};

static struct attribute_group cpucomm_adapter_attr_group = {
	.attrs		= cpucomm_adapter_attrs,
};

static const struct attribute_group *cpucomm_adapter_attr_groups[] = {
	&cpucomm_adapter_attr_group,
	NULL
};

struct device_type cpucomm_adapter_type = {
	.groups		= cpucomm_adapter_attr_groups,
	.release	= cpucomm_adapter_dev_release,
};
EXPORT_SYMBOL_GPL(cpucomm_adapter_type);

static void cpucomm_scan_static_board_info(struct cpucomm_adapter *adapter)
{
	struct cpucomm_devinfo	*devinfo;

	down_read(&__cpucomm_board_lock);
	list_for_each_entry(devinfo, &__cpucomm_board_list, list) {
		if (!cpucomm_new_device(adapter,
						&devinfo->board_info))
			dev_err(&adapter->dev,
				"Can't create device %s\n",
				devinfo->board_info.name);
	}
	up_read(&__cpucomm_board_lock);
}

/**
 * cpucomm_register_adapter - declare cpucomm adapter, use dynamic bus number
 * @adapter: the adapter to add
 * Context: can sleep
 *
 * This routine is used to declare an CPUCOMM adapter when its bus number
 * doesn't matter.  Examples: for CPUCOMM adapters dynamically added by
 * USB links or PCI plugin cards.
 *
 * When this returns zero, a new bus number was allocated and stored
 * in adap->nr, and the specified adapter became available for clients.
 * Otherwise, a negative errno value is returned.
 */
int cpucomm_register_adapter(struct cpucomm_adapter *adap)
{
	int res = 0;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!cpucomm_bus_type.p))) {
		res = -EAGAIN;
		goto out_list;
	}

	/* Sanity checks */
	if (unlikely(adap->name[0] == '\0')) {
		pr_err("cpucomm-core: Attempt to register an adapter with "
		       "no name!\n");
		return -EINVAL;
	}
	if (unlikely(!adap->algo)) {
		pr_err("cpucomm-core: Attempt to register adapter '%s' with "
		       "no algo!\n", adap->name);
		return -EINVAL;
	}

	rt_mutex_init(&adap->bus_lock);

	/* Set default timeout to 1 second if not already set */
	if (adap->timeout == 0)
		adap->timeout = HZ;

	dev_set_name(&adap->dev, "%s", adap->name);
	adap->dev.bus = &cpucomm_bus_type;
	adap->dev.type = &cpucomm_adapter_type;
	res = device_register(&adap->dev);
	if (res)
		goto out_list;

	dev_dbg(&adap->dev, "adapter [%s] registered\n", adap->name);

	/* create pre-declared device nodes */
	cpucomm_scan_static_board_info(adap);

	return 0;

out_list:
	return res;
}
EXPORT_SYMBOL(cpucomm_register_adapter);

static int __cpucomm_unregister_client(struct device *dev, void *dummy)
{
	struct cpucomm_client *client = cpucomm_verify_client(dev);
	if (client && strcmp(client->name, "dummy"))
		cpucomm_unregister_device(client);
	return 0;
}

static int __cpucomm_unregister_dummy(struct device *dev, void *dummy)
{
	struct cpucomm_client *client = cpucomm_verify_client(dev);
	if (client)
		cpucomm_unregister_device(client);
	return 0;
}

/**
 * cpucomm_unregister_adapter - unregister CPUCOMM adapter
 * @adap: the adapter being unregistered
 * Context: can sleep
 *
 * This unregisters an CPUCOMM adapter which was previously registered
 * by @cpucomm_add_adapter or @cpucomm_add_numbered_adapter.
 */
int cpucomm_unregister_adapter(struct cpucomm_adapter *adap)
{
	int res = 0;

	/* Detach any active clients. This can't fail, thus we do not
	 * check the returned value. This is a two-pass process, because
	 * we can't remove the dummy devices during the first pass: they
	 * could have been instantiated by real devices wishing to clean
	 * them up properly, so we give them a chance to do that first. */
	res = device_for_each_child(&adap->dev, NULL, __cpucomm_unregister_client);
	res = device_for_each_child(&adap->dev, NULL, __cpucomm_unregister_dummy);

	/* device name is gone after device_unregister */
	dev_dbg(&adap->dev, "adapter [%s] unregistered\n", adap->name);

	/* clean up the sysfs representation */
	init_completion(&adap->dev_released);
	device_unregister(&adap->dev);

	/* wait for sysfs to drop all references */
	wait_for_completion(&adap->dev_released);

	/* Clear the device structure in case this adapter is ever going to be
	   added again */
	memset(&adap->dev, 0, sizeof(adap->dev));

	return 0;
}
EXPORT_SYMBOL(cpucomm_unregister_adapter);


/* ------------------------------------------------------------------------- */

int cpucomm_for_each_dev(void *data, int (*fn)(struct device *, void *))
{
	int res;

	mutex_lock(&core_lock);
	res = bus_for_each_dev(&cpucomm_bus_type, NULL, data, fn);
	mutex_unlock(&core_lock);

	return res;
}
EXPORT_SYMBOL_GPL(cpucomm_for_each_dev);

/*
 * An cpucomm_driver is used with one or more cpucomm_client (device) nodes to access
 * cpucomm slave chips, on a bus instance associated with some cpucomm_adapter.
 */

int cpucomm_register_driver(struct module *owner, struct cpucomm_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!cpucomm_bus_type.p)))
		return -EAGAIN;

	/* add the driver to the list of cpucomm drivers in the driver core */
	driver->driver.owner = owner;
	driver->driver.bus = &cpucomm_bus_type;

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
	res = driver_register(&driver->driver);
	if (res)
		return res;

	/* Drivers should switch to dev_pm_ops instead. */
	if (driver->suspend)
		pr_warn("cpucomm-core: driver [%s] using legacy suspend method\n",
			driver->driver.name);
	if (driver->resume)
		pr_warn("cpucomm-core: driver [%s] using legacy resume method\n",
			driver->driver.name);

	pr_debug("cpucomm-core: driver [%s] registered\n", driver->driver.name);

	return 0;
}

/**
 * cpucomm_unregister_driver - unregister CPUCOMM driver
 * @driver: the driver being unregistered
 * Context: can sleep
 */
void cpucomm_unregister_driver(struct cpucomm_driver *driver)
{
	driver_unregister(&driver->driver);
	pr_debug("cpucomm-core: driver [%s] unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL(cpucomm_unregister_driver);

/* ------------------------------------------------------------------------- */

static int __init cpucomm_init(void)
{
	int retval;

	retval = bus_register(&cpucomm_bus_type);
	if (retval)
		return retval;
	return 0;
}

static void __exit cpucomm_exit(void)
{
	bus_unregister(&cpucomm_bus_type);
}

/* We must initialize early, because some subsystems register cpucomm drivers
 * in subsys_initcall() code, but are linked (and initialized) before cpucomm.
 */
postcore_initcall(cpucomm_init);
module_exit(cpucomm_exit);

MODULE_AUTHOR("Chiket Lin <chiket_lin@a-i-t.com.tw>");
MODULE_DESCRIPTION("CpuComm-Bus main module");
MODULE_LICENSE("GPL");
