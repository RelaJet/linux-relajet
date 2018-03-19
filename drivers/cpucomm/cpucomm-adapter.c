/*
    cpucomm Support for AIT8428

    Copyright (C) 2014 Chiket

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
//#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>

#include <mach/cpucomm/cpucomm-lx.h>
#include "cpucomm_core.h"
#include "cpucomm-sysflag.h"

/*
 * Generic cpucomm master transfer entrypoint.
 */

static CPU_COMM_ERR _cpucomm_register_entry(struct cpucomm_adapter *adap, CPU_COMM_ID id, CPU_COMM_TYPE type)
{
    return CpuComm_RegisterEntry(id, type);
}

static CPU_COMM_ERR _cpucomm_unregister_entry(struct cpucomm_adapter *adap, CPU_COMM_ID id)
{
    return CpuComm_UnregisterEntry(id);
}

static CPU_COMM_ERR _cpucomm_sem_post(struct cpucomm_adapter *adap, CPU_COMM_ID id, unsigned int timeout)
{
    return CpuComm_SemPost(id, timeout);
}

static CPU_COMM_ERR _cpucomm_sem_wait(struct cpucomm_adapter *adap, CPU_COMM_ID id, unsigned int timeout)
{
    return CpuComm_SemWait(id, timeout);
}

static CPU_COMM_ERR _cpucomm_data_send(struct cpucomm_adapter *adap, CPU_COMM_ID id, void *data, unsigned int size, unsigned int timeout)
{
    return CpuComm_DataSend(id, data, size, timeout);
}

static CPU_COMM_ERR _cpucomm_data_receive(struct cpucomm_adapter *adap, CPU_COMM_ID id, void *data, unsigned int size, unsigned int timeout, CPUCOMM_RCV_PREPROC preproc)
{
    return CpuComm_DataReceive(id, data, size, timeout, preproc);
}

struct cpucomm_algorithm _cpucomm_algorithm = {
    .register_entry = _cpucomm_register_entry,
    .unregister_entry = _cpucomm_unregister_entry,
    .sem_post = _cpucomm_sem_post,
    .sem_wait = _cpucomm_sem_wait,
    .data_send = _cpucomm_data_send,
    .data_receive = _cpucomm_data_receive,
};

/*
 * Interrupt handler
 */
static irqreturn_t cpucomm_int_handler(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct cpucomm_adapter *adapter = platform_get_drvdata(pdev );
	AITPS_HINT pHINT = adapter->baseaddr_hint;

	
	if(pHINT->HINT_CTL & HINT_CPU2CPUA_INT)
	{
		pr_debug("CPUCOMM: IRQ B->A\n");
		CpuComm_SwapISR_CPUA();

		MMP_CPUCOMM_IRQ_CLEAR( _CPU_ID_A);
		return IRQ_HANDLED;

	}
#if 1
	if(pHINT->HINT_CTL & HINT_CPU2CPUB_INT)
	{
		pr_debug("CPUCOMM: IRQ A->B\n");		
		//CpuComm_SwapISR();
		//MMP_CPUCOMM_IRQ_CLEAR( _CPU_ID_B);		
	}
#endif 
	return IRQ_HANDLED;
}

/*
 * Main initialization routine.
 */
static int __devinit  cpucomm_adap_probe(struct platform_device *pdev)    //at91sam9260_twi_device
{
	struct cpucomm_adapter *adapter;
	struct resource *res;
    	int retval;

	
	// allocate buffer for adapter
	adapter = kzalloc(sizeof(struct cpucomm_adapter), GFP_KERNEL);
	if (adapter == NULL) {
	    dev_err(&pdev->dev, "can't allocate inteface!\n");
	    retval = -ENOMEM;
	    goto fail1;
	}
	
    // init adapter data
    snprintf(adapter->name, sizeof(adapter->name), "cpucomm_adap");
    adapter->algo = &_cpucomm_algorithm;
    adapter->algo_data = NULL;
    adapter->dev.parent = &pdev->dev;

    // add driver data to struct platform_device
    platform_set_drvdata(pdev, adapter);

    // Get interrupt controller base address
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
        retval = -ENXIO;
        goto fail_release_io;
    }
	
	if (!request_mem_region(res->start, resource_size(res), "cpucomm_adap_hint")) {
        retval = -EBUSY;
        goto fail_release_io;
    }

	adapter->baseaddr_hint = ioremap(res->start, resource_size(res));
		
	if( !adapter->baseaddr_hint ){
		retval = -ENOMEM;
		goto fail1;
	}


	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
        retval = -ENXIO;
        goto fail_release_io;
    }
	
	if (!request_mem_region(res->start, resource_size(res), "cpucomm_adap_share")) {
        retval = -EBUSY;
        goto fail_release_io;
    }

	if( !ioremap(res->start, resource_size(res)) ){
		retval = -ENOMEM;
		goto fail_release_io;
	}
    adapter->baseaddr_share = (CPU_COMM_SWAP_DATA*)res->start;
	if( !adapter->baseaddr_share ) {
        retval = -ENOMEM;
        goto fail_release_io;
    }

#ifdef CONFIG_AIT_MCRV2_DUAL_OS_ON_CPUA
	CpuComm_HwInit(_CPU_ID_A,adapter->baseaddr_hint);
#else
	// init Cpucomm HW
	CpuComm_HwInit(_CPU_ID_B);
#endif
	// request IRQ
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res) {
        retval = -ENXIO;
        goto fail_iounmap;
    }

	retval = request_irq(res->start, cpucomm_int_handler, 0, "cpucomm_irq", pdev ); // or IRQF_SHARED?
	if (retval)
	    goto fail_iounmap;

	// add  adapter to the platform device, and also register client device which has been added by cpucomm_register_board_info()
	retval = cpucomm_register_adapter(adapter);
	if (retval) {
	    dev_err(&pdev->dev, "Adapter %s registration failed\n",
	            adapter->name);
	    goto fail_freeirq;
	}

	dev_info(&pdev->dev, "cpucomm adapter driver established.\n");

	// temp place
	sysflag_init();

	return 0;
fail_freeirq:
	free_irq(res->start, pdev);
	
fail2:
    kfree(adapter);
fail_iounmap:
    platform_set_drvdata(pdev, NULL);
	iounmap(adapter->baseaddr_hint);
	iounmap(adapter->baseaddr_share);

fail_release_io:
	release_mem_region(res->start, resource_size(res));	
fail1:

    return retval;
}

static int __devexit  cpucomm_adap_remove(struct platform_device *pdev)
{
    struct cpucomm_adapter *adapter = platform_get_drvdata(pdev); // get adapter from struct platform_device
    int rc;

    // delete adapter
    rc = cpucomm_unregister_adapter(adapter);

    // set driver data of device as NULL
    platform_set_drvdata(pdev, NULL);

    kfree(adapter);

    return rc;
}

#ifdef CONFIG_PM

/* NOTE: could save a few mA by keeping clock off outside of at91_xfer... */

static int  cpucomm_adap_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int cpucomm_adap_resume(struct platform_device *pdev)
{
    return 0;
}

#else
#define cpucomm_adap_suspend       NULL
#define cpucomm_adap_resume        NULL
#endif

MODULE_ALIAS("platform:cpucomm_adap_sw");

static struct platform_driver  cpucomm_adap_driver = {
    .probe        = cpucomm_adap_probe,
    .remove        = __devexit_p(cpucomm_adap_remove),
    .suspend        = cpucomm_adap_suspend,
    .resume        = cpucomm_adap_resume,
    .driver        = {
        .name    = "cpucomm",
        .owner    = THIS_MODULE,
    },
};

static int __init  cpucomm_adap_init(void)
{
    printk( "cpucomm_adap_init()\n" );

    return platform_driver_register(&cpucomm_adap_driver );
}

static void __exit cpucomm_adap_exit(void)
{
    platform_driver_unregister(&cpucomm_adap_driver );
}

module_init(cpucomm_adap_init);
module_exit(cpucomm_adap_exit);

#if 0 // need check the implemenation of sysflag further
static int __init cpucomm_sys_notify(void)
{
    // register sysflag
    CpuComm_RegisterEntry(CPU_COMM_ID_SYSFLAG, CPU_COMM_TYPE_SEM);

    // Notify the cpucomm init is done
    CpuComm_SemPost(CPU_COMM_ID_SYSFLAG, 0);

    return 0;
}
pure_initcall(cpucomm_sys_notify);
#endif

MODULE_AUTHOR("Chiket Lin");
MODULE_DESCRIPTION("CPU-COMM adapter");
//MODULE_LICENSE("GPL");
