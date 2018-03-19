/*
 * cpucomm-cputest.c a specific driver for test cpucomm
 *
 * Author:	Chiket
 * Created:	Dec 23, 2014
 * Copyright:	A.I.T Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/cdev.h>


#include "cpucomm.h"
#include <mach/cpucomm/cpucomm-cputest.h>

struct cputest_client_data {
    CPU_COMM_ID         comm_id;    /* Cpucomm Entry ID */
    CPU_TEST_TYPE       test_type;  /* Test type */
    struct task_struct  *task;      /* Test task */
};

struct cputest_test_data {
    CPU_COMM_TYPE type;
    int (*func)(void *data);
};


struct test_stuff
{
	unsigned int count;
};

struct cputest_driver {
    struct module *owner;
    struct class *class;
    struct cdev cdev;
    const char	*name;
    int major;          /* major device number */
    int minor_start;    /* start of minor device number */    
    int count;          /* number of devices allocated */
    struct cpucomm_client *client[CPU_TEST_TYPE_MAX];
};

static char cpu_string[] = "B"; 

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Working thread proc
static int cputest_data_receive_func(void *data)
{
    struct cpucomm_client       *client = data;
    struct cputest_client_data  *client_data;
    struct test_stuff           stuff;
    MMP_ULONG                   previous_count = 0xFFFFFFFF;
    CPU_COMM_ERR                ret;

    client_data = cpucomm_get_clientdata(client);

    pr_warn( "cputest_data_receive_func(%d, %d)\r\n", client_data->comm_id, client_data->test_type );

    while(1) 
    {
        ret = cpucomm_data_receive(client, client_data->comm_id, &stuff, sizeof(struct cputest_client_data),1327, NULL);
        switch( ret )
        {
            case CPU_COMM_ERR_NONE:
                if( previous_count+1 != stuff.count )
                {
                    pr_warn("%s Rcv %d %u inc (%u)\r\n", cpu_string, client_data->comm_id, stuff.count, previous_count );
                }

                previous_count = stuff.count;

                if( stuff.count % 180811 == 0 ) // use prime number to avoid the string is shown at the same time 
                {
                    pr_warn("%s Rcv %d %u\r\n", cpu_string, client_data->comm_id, stuff.count);
                }
                break;

            default:
                pr_warn("%s Rcv %d %u e %d\r\n", cpu_string, client_data->comm_id, stuff.count, ret);
                break;
        }
    }

    return 0;
}

static int cputest_data_send_func(void *data)
{
    struct cpucomm_client       *client = data;
    struct cputest_client_data *client_data;
    struct test_stuff           stuff;
    CPU_COMM_ERR                ret;

    client_data = cpucomm_get_clientdata(client);
    stuff.count = 0;

    pr_warn( "cputest_data_send_func(%d, %d)\r\n", client_data->comm_id, client_data->test_type );
    

    while(1)
    {
        ret = cpucomm_data_send(client, client_data->comm_id, &stuff, sizeof(struct cputest_client_data),1009 );
        switch( ret )
        {
            case CPU_COMM_ERR_NONE:
                if( ++stuff.count % 151051 == 0 ) // use prime number to avoid the string is shown at the same time 
                {
                    pr_warn("%s Snd %d %u\r\n", cpu_string, client_data->comm_id, stuff.count );
                }
                break;

            default:
                pr_warn("%s Snd %d %u e %d\r\n", cpu_string, client_data->comm_id, stuff.count, ret );
                break;
        }
    }

    return 0;
}

static int cputest_sem_wait_func(void *data)
{
    return 0;
}

static int cputest_sem_post_func(void *data)
{
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// File operations
inline struct cpucomm_client *cputest_get_client(struct cputest_driver *driver, dev_t i_rdev)
{
    return driver->client[MINOR(i_rdev) - driver->minor_start];
}

static int cputest_open(struct inode *inode, struct file *file)
{
    struct cputest_driver *driver;
    struct cpucomm_client *client;

    pr_warn("%s(%x)\r\n", __FUNCTION__, inode->i_rdev);

    // client = to_cpucomm_client(inode->i_cdev);
    driver = container_of(inode->i_cdev, struct cputest_driver, cdev);
    client = cputest_get_client( driver, inode->i_rdev);
    if(!client) {
        pr_warn("unable to open a device with nulll client\n");
        return -ENODEV;
    }
        
    file->private_data = driver;

    return 0;
}

static int cputest_release(struct inode *inode, struct file *file)
{
    pr_warn("%s()\r\n", __FUNCTION__ );

    file->private_data = NULL;

    return 0;
}

static ssize_t cputest_read(struct file *file, char __user *buf, size_t count, loff_t *ptr)
{
    pr_warn("%s()\r\n", __FUNCTION__ );

    return 0;
}

static ssize_t cputest_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    pr_warn("%s()\r\n", __FUNCTION__ );
    return 0;
}

static long cputest_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    pr_warn("%s()\r\n", __FUNCTION__ );
    return 0;
}

struct file_operations cputest_fops = {
    .owner   = THIS_MODULE,
    .open           = cputest_open,
    .release        = cputest_release,
    .read           = cputest_read,
    .write          = cputest_write,
    .unlocked_ioctl = cputest_ioctl,
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
// cdev operations
static int cputest_register_driver(struct cputest_driver *driver)
{
	dev_t devt;
    int   ret;

	driver->class = class_create(driver->owner, driver->name);
	if (IS_ERR(driver->class))
		return PTR_ERR(driver->class);

    // request a cdev id
    ret = alloc_chrdev_region(&devt, 0, 0, driver->name);
    if( ret )
        return ret;

    // keep cdev info        
    driver->major = MAJOR(devt);
    driver->minor_start = MINOR(devt);

    // init cdev
	cdev_init(&driver->cdev, &cputest_fops);
	driver->cdev.owner = driver->owner;

    // add cdev to kernel
	ret = cdev_add(&driver->cdev, devt, driver->count);
	if(ret)
        unregister_chrdev_region(devt, driver->count);

    return ret;
}

static void cputest_unregister_driver(struct cputest_driver *driver)
{
	unregister_chrdev_region(MKDEV(driver->major, driver->minor_start), driver->count);
}

static int cputest_register_device(struct cputest_driver *driver, unsigned index, struct cpucomm_client *client)
{
	char name[64];
	dev_t devt = MKDEV(driver->major, driver->minor_start) + index;
    struct device * dev;

	if (index >= driver->count) {
		printk(KERN_ERR "Register invalid index %d\n", index);
		return -EINVAL;
	}

    sprintf( name, "%s%d", driver->name, index );

	dev = device_create(driver->class, &client->dev, devt, NULL, name);
	if (IS_ERR(dev)) {
        pr_warning( "cpu-test: Failed to register cdev device-%d\n", index);
		return PTR_ERR(dev);
	}

    // Assign the client to driver
    driver->client[index] = client;

    return 0;
}

static void cputest_unregister_device(struct cputest_driver *driver, unsigned index)
{
	device_destroy(driver->class, MKDEV(driver->major, driver->minor_start) + index);
    driver->client[index] = NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// driver operations
const static struct cputest_test_data test_data_array[] = {
    {CPU_COMM_TYPE_DATA, cputest_data_send_func},      // CPU_TEST_TYPE_DATA_SENDER
    {CPU_COMM_TYPE_DATA, cputest_data_receive_func},   // CPU_TEST_TYPE_DATA_RECEIVER
    {CPU_COMM_TYPE_SEM,  cputest_sem_post_func},       // CPU_TEST_TYPE_SEM_SENDER
    {CPU_COMM_TYPE_SEM,  cputest_sem_wait_func}        // CPU_TEST_TYPE_SEM_RECEIVER
};

static struct cputest_driver cputest_driver = {
    .owner          = THIS_MODULE,
    .name           = "cputest",
    .major          = 0,
    .minor_start    = 0,
    .count          = 4,
};

static int cputest_probe(struct cpucomm_client *client)
{
    struct cpucomm_cputest_dev_data *dev_data = client->dev.platform_data;
    struct cputest_client_data *client_data;
    const static struct cputest_test_data *test_data;
    CPU_TEST_TYPE test_type;
    int ret = 0;

    // Get test data
    test_type = cpucomm_get_device_id(client)->driver_data;
    test_data = &test_data_array[test_type];

    // Allocate data buffer
	client_data = kzalloc(sizeof(struct cputest_client_data), GFP_KERNEL);
	if (!client_data)
		return -ENOMEM;

    // assign client data
    client_data->comm_id   = dev_data->comm_id;
    client_data->test_type = test_type;

    // Assign client data
	cpucomm_set_clientdata(client, client_data);

    // Register CpuComm Entry
    cpucomm_register_entry(client, client_data->comm_id, test_data->type);

    // Register a cdev device
    ret = cputest_register_device(&cputest_driver, test_type, client);
	if(ret)
        goto error;

    // Create test thread
	client_data->task = kthread_run(test_data->func, (void*)client, "cputest-%d", client_data->test_type);
	if (IS_ERR(client_data->task)) {
        pr_warning( "cpu-test: Failed to run thread cputest-%d\n", client_data->test_type);
        cputest_unregister_device(&cputest_driver, test_type);
        ret = -EPERM;
        goto error;
	}

    return 0;
    
error:
    cpucomm_set_clientdata(client, NULL);
    kfree( client_data );
    
    return ret;
}

static int cputest_remove(struct cpucomm_client *client)
{
	struct cputest_client_data *client_data = cpucomm_get_clientdata(client);

    cputest_unregister_device(&cputest_driver, client_data->test_type);

    cpucomm_unregister_entry(client, client_data->comm_id);
    kfree( client_data );

    return 0;
}

static const struct cpucomm_device_id cputest_ids[] = {
	{ "cputest_data_send", CPU_TEST_TYPE_DATA_SENDER },
	{ "cputest_data_receive", CPU_TEST_TYPE_DATA_RECEIVER},
	{ "cputest_sem_post", CPU_TEST_TYPE_SEM_SENDER},
    { "cputest_sem_wait", CPU_TEST_TYPE_SEM_RECEIVER},
	{ }
};
MODULE_DEVICE_TABLE(cpucomm, cputest_ids);

static struct cpucomm_driver cpucomm_driver = {
	.driver = {
		.name	= "cpucomm_cputest",
		.owner	= THIS_MODULE,
	},
	.probe		= cputest_probe,
	.remove		= __devexit_p(cputest_remove),
	.id_table	= cputest_ids,
};

static int __init cputest_init(void)
{
    int   ret;

    // Register cputest driver to cdev
	ret = cputest_register_driver(&cputest_driver);
	if(ret)
        return ret;

    // Register a cpucomm driver to bus
    ret = cpucomm_add_driver(&cpucomm_driver);
    if(ret)
        goto error;

    return 0;

error:
    cputest_unregister_driver(&cputest_driver);
    return ret;
}

static void __exit cputest_exit(void)
{
    // Unregister cpucomm drvier from buffer
	cpucomm_unregister_driver(&cpucomm_driver);

    // Unregister cputest driver from cdev
    cputest_unregister_driver(&cputest_driver);
}

module_init(cputest_init);
module_exit(cputest_exit);

MODULE_AUTHOR("Chiket Lin");
