/*
    i2c Support for Atmel's AT91 Two-Wire Interface (TWI)

    Copyright (C) 2004 Rick Bronson
    Converted to 2.6 by Andrew Victor <andrew@sanpeople.com>

    Borrowed heavily from original work by:
    Copyright (C) 2000 Philip Edelbrock <phil@stimpy.netroedge.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/i2c/at24.h>
#include <mach/board.h>
#include <mach/cpu.h>

#include <mach/mmp_register.h>
//#include <mach/mmp_reg_vif.h>
#include <mach/mmp_reg_gbl.h>
#include <mach/mmp_reg_i2cm.h>
#include <mach/mmpf_i2cm.h>
//#include <mach/mmpf_vif.h>

#define TWI_CLOCK		100000		/* Hz. max 400 Kbits/sec */

static struct clk *i2cm_clk;

#if 0
static MMPF_I2CM_ATTRIBUTE m_PHI2CMAttribute = {
	.i2cmID		    = MMPF_I2CM_ID_0,
	.ubSlaveAddr 	= 0x1A,
	.ubRegLen		= 8,
	.ubDataLen		= 8,
	.ubDelayTime	= 0,
	.bDelayWaitEn	= MMP_FALSE,
	.bInputFilterEn	= MMP_TRUE,
	.b10BitModeEn	= MMP_FALSE,
	.bClkStretchEn	= MMP_FALSE,
	.ubSlaveAddr1	= 0,
	.ubDelayCycle	= 0,
	.ubPadNum		= 0,
    .ulI2cmSpeed    = 250000,
    .bOsProtectEn   = MMP_TRUE,
    .sw_clk_pin     = 0,
    .sw_data_pin    = 0,
    .bRfclModeEn    = MMP_FALSE,
    .bWfclModeEn    = MMP_FALSE
};
#endif
static int ait_phi2c_get_i2c_info(struct i2c_adapter *adap,u8 slave_addr,u8 *reg_len,u8 *data_len)
{
    
    struct i2c_board_info *board_info = (struct i2c_board_info *)i2c_get_board_info(adap,slave_addr) ;
    if(board_info) {
        struct ait_i2c_platform_data *pd = (struct ait_i2c_platform_data *)board_info->platform_data ;
        if(pd && pd->tag==AIT_I2C_INFO_TAG) {
            //printk(KERN_ERR"i2c.board->type : %s,addr :0x%x,reg_len : %d,data_len:%d\r\n",board_info->type,board_info->addr,pd->reg_len,pd->data_len);    
            *reg_len= pd->reg_len;
            *data_len = pd->data_len ;
            return 1 ;
        }
    }
    return 0 ;
}


/*
 * Initialize the TWI hardware registers.
 */
static void __devinit ait_phi2c_hwinit(MMPF_I2CM_ATTRIBUTE *i2c_attr)
{
	//MMPF_I2cm_DisableInterrupt(i2c_attr);
}

/*
 * Generic i2c master transfer entrypoint.
 *
 * Note: We do not use Atmel's feature of storing the "internal device address".
 * Instead the "internal device address" has to be written using a separate
 * i2c message.
 * http://lists.arm.linux.org.uk/pipermail/linux-arm-kernel/2004-September/024411.html
 */
#if 0
static int ait_phi2c_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int ret;

	dev_dbg(&adap->dev, "ait_phi2c_xfer: processing %d messages:\n", num);

	dev_dbg(&adap->dev, "# %sing %d byte%s %s 0x%02x\n",
			pmsg->flags & I2C_M_RD ? "read" : "writ",
			pmsg->len, pmsg->len > 1 ? "s" : "",
			pmsg->flags & I2C_M_RD ? "from" : "to",	pmsg->addr);

    m_PHI2CMAttribute.ubSlaveAddr = pmsg->addr;

    if (pmsg->len && pmsg->buf) {	/* sanity check */
        if (pmsg->flags & I2C_M_RD)
        {
            MMP_USHORT reg, data;
            reg = pmsg->buf[0];
            ret = MMPF_I2cm_ReadReg(&m_PHI2CMAttribute, reg, &data);
            pmsg->buf[1] = data;
        }
        else
        {
            ret = MMPF_I2cm_WriteReg(&m_PHI2CMAttribute, pmsg->buf[0], pmsg->buf[1]);

        }

        if(ret ==MMP_I2CM_ERR_SLAVE_NO_ACK )
            return -ENXIO;
        if (ret)
            return -ETIMEDOUT;

    }
    dev_dbg(&adap->dev, "transfer complete\n");

	return ret;
}
#else

typedef struct _ait_i2c_adapter
{
	struct i2c_adapter i2c_adp;
	MMPF_I2CM_ATTRIBUTE i2c_attr;
	struct ait_i2c_extension i2c_ext;
}ait_i2c_adapter;

int AIT_I2cm_Read_burst_safe(ait_i2c_adapter *adap, MMP_USHORT usReg, MMP_USHORT *usData, MMP_UBYTE usDataCnt)
{
	if(adap->i2c_ext.i2c_hw_lock)
	{
		MMP_ERR r;
		down(*adap->i2c_ext.i2c_hw_lock);
		//change to 1byte
		adap->i2c_attr.ubDataLen = 8;
		r = MMPF_I2cm_ReadBurstData(&adap->i2c_attr, usReg, usData, usDataCnt);
		up(*adap->i2c_ext.i2c_hw_lock);
		return r;
	}else{
		return MMPF_I2cm_ReadBurstData(&adap->i2c_attr, usReg, usData, usDataCnt);
	}
}

int AIT_I2cm_ReadReg_safe(ait_i2c_adapter *adap, MMP_USHORT usReg, MMP_USHORT *usData)
{
	if(adap->i2c_ext.i2c_hw_lock)
	{
		MMP_ERR r;
		down(*adap->i2c_ext.i2c_hw_lock);
		r = MMPF_I2cm_ReadReg(&adap->i2c_attr,usReg,usData);
		up(*adap->i2c_ext.i2c_hw_lock);
		return r;
	}else{
		return MMPF_I2cm_ReadReg(&adap->i2c_attr,usReg,usData);
	}
}

int AIT_I2cm_Write_burst_safe(ait_i2c_adapter *adap, MMP_USHORT usReg, MMP_USHORT *usData, MMP_UBYTE usDataCnt)
{

	if(adap->i2c_ext.i2c_hw_lock)
	{
		MMP_ERR r;
		down(*adap->i2c_ext.i2c_hw_lock);
		//change to 1byte
		adap->i2c_attr.ubDataLen = 8;
		r = MMPF_I2cm_WriteBurstData(&adap->i2c_attr, usReg, usData, usDataCnt);
		up(*adap->i2c_ext.i2c_hw_lock);
	}else{
		return MMPF_I2cm_WriteBurstData(&adap->i2c_attr, usReg, usData, usDataCnt);
	}
}

int	AIT_I2cm_WriteReg_safe(ait_i2c_adapter *adap, MMP_USHORT usReg, MMP_USHORT usData)
{
	if(adap->i2c_ext.i2c_hw_lock)
	{
		MMP_ERR r;
		down(*adap->i2c_ext.i2c_hw_lock);
		r = MMPF_I2cm_WriteReg(&adap->i2c_attr,usReg,usData);
		up(*adap->i2c_ext.i2c_hw_lock);
		return r;
	}else{
		return MMPF_I2cm_WriteReg(&adap->i2c_attr,usReg,usData);
	}
}

static int ait_phi2c_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	//TODO: the performance need to improve, current low level I2C driver can only transfer 1-2 bytes data at one time.
	int  ret=0,ret_i2c_info = 0 ;

	dev_dbg(&adap->dev, "ait_vsnv3_phi2c_xfer: processing %d messages:\n", num);
	{

		//MMPF_I2CM_ATTRIBUTE *i2c_attr = &(((ait_i2c_adapter*) dev_get_drvdata(&adap->dev))->i2c_attr);
		u8 slave_addr = pmsg->addr ;
		ait_i2c_adapter* ait_adap = (ait_i2c_adapter*) dev_get_drvdata(&adap->dev);
		MMPF_I2CM_ATTRIBUTE *i2c_attr = &ait_adap->i2c_attr;
		ret_i2c_info = ait_phi2c_get_i2c_info( adap, slave_addr,&i2c_attr->ubRegLen,&i2c_attr->ubDataLen);
		if(!ret_i2c_info) {		
		    i2c_attr->ubRegLen = 8;
		    i2c_attr->ubDataLen = 8;
		}
    	  		
		dev_dbg(&adap->dev, "%sing %d byte%s %s 0x%02x\n",
			pmsg->flags & I2C_M_RD ? "read" : "write",
			pmsg->len, pmsg->len > 1 ? "s" : "",
			pmsg->flags & I2C_M_RD ? "from" : "to",	pmsg->addr);
			i2c_attr->ubSlaveAddr = pmsg->addr;
		if(num>1)
		{
			int n=0;
			MMP_USHORT reg, data;

			if(pmsg[0].flags == 0)
			{
				if(pmsg[0].len==1)
				{
					reg = pmsg[0].buf[0];
				}
				else if(pmsg[0].len==2)
				{
					reg = pmsg[0].buf[0]*0x100 + pmsg[0].buf[1];
					if(!ret_i2c_info) {
					  i2c_attr->ubRegLen = 16 ;
					}
				}
				else
				{
					BUG_ON(1);
				}
				
				n=1;
			}else{
				printk("ait_vsnv3_phi2c_xfer unexpect case.\r\n");
				n=0;
				//return 0;
			}

			for(n;n<num;++n)
			{
				if (pmsg[n].flags & I2C_M_RD)
				{
					MMP_USHORT data;
					
					if(pmsg[n].len <= 2 )
					{
						if(!ret_i2c_info)
						{
							i2c_attr->ubDataLen = pmsg[n].len*8;
						}
					
						ret = AIT_I2cm_ReadReg_safe(ait_adap, reg, &data);

						if(pmsg[n].len == 1)
						{
							pmsg[n].buf[0] = data & 0xFF;
						}else if(pmsg[n].len == 2){
							*(MMP_USHORT*)pmsg[n].buf = data;
						}
					}else{
						int i = 0;
						MMP_USHORT data[128];

						ret = AIT_I2cm_Read_burst_safe(ait_adap, reg, data, pmsg[n].len);

						while(i < pmsg[n].len)
						{
							pmsg[n].buf[i] = data[i];
							i++;
						}
					}
				}else{
					if(!ret_i2c_info)
					{
						i2c_attr->ubDataLen = (pmsg[n].len-1)*8;
					}

					if(pmsg->len ==3)
					{
						ret = AIT_I2cm_WriteReg_safe(ait_adap, pmsg[n].buf[0], pmsg->buf[1] << 8 | pmsg->buf[2]);
					}
					else if(pmsg->len ==2)
					{
						ret = AIT_I2cm_WriteReg_safe(ait_adap, pmsg[n].buf[0], pmsg->buf[1]);
					}
					else
					{
						int i = 0;
						MMP_USHORT data[128];

						while(i < pmsg[n].len)
						{
							data[i] = pmsg[n].buf[i];
							i++;
						}

						ret = AIT_I2cm_Write_burst_safe(ait_adap, reg, data, pmsg[n].len);
					}
				}

				if(ret < 0)
				{
					if(ret ==MMP_I2CM_ERR_SLAVE_NO_ACK )
						return -ENXIO;
					if (ret)
						return -ETIMEDOUT;
				}
				else
					ret = n+1;
			}

		}else{
			if (pmsg->len && pmsg->buf) {	/* sanity check */
				if (pmsg->flags & I2C_M_RD)
				{
					MMP_USHORT reg, data;
					//reg = pmsg->buf[0];
					
					if(pmsg[0].len==1)
					{
						reg = pmsg[0].buf[0];
					}
					else if(pmsg[0].len==2)
					{
						reg = pmsg[0].buf[0]*0x100 + pmsg[0].buf[1];
						if(!ret_i2c_info) i2c_attr->ubRegLen = 2;
					}
					else
					{
						BUG_ON(1);
					}
					
					ret = AIT_I2cm_ReadReg_safe(ait_adap, reg, &data);
					pmsg->buf[1] = data;
				}
				else
				{
					//i2c_attr->ubDataLen = (pmsg->len-1)*8;
					if(pmsg->len ==3)
					{
						if(!ret_i2c_info)i2c_attr->ubDataLen = 16;
						ret = AIT_I2cm_WriteReg_safe(ait_adap, pmsg->buf[0], pmsg->buf[1] << 8 | pmsg->buf[2]);
					}
					else if(pmsg->len ==4)
					{
						if(!ret_i2c_info)i2c_attr->ubDataLen = 16;
						if(!ret_i2c_info)i2c_attr->ubRegLen = 16;
						ret = AIT_I2cm_WriteReg_safe(	ait_adap, pmsg->buf[0] << 8 | pmsg->buf[1], 
														pmsg->buf[2] << 8 | pmsg->buf[3]);
					}
					else if(pmsg->len ==2)
					{
						if(!ret_i2c_info)i2c_attr->ubDataLen = 8;
						ret = AIT_I2cm_WriteReg_safe(ait_adap, pmsg->buf[0], pmsg->buf[1]);
					}else{
                        int i = 0;
                        MMP_USHORT reg,data[128];

						if(i2c_attr->ubRegLen = 16)
						{
							reg = pmsg->buf[0] << 8 | pmsg->buf[1];
							while(i < pmsg->len-2)
							{
								data[i] = pmsg->buf[i+2];
								i++;
							}
                        	ret = AIT_I2cm_Write_burst_safe(ait_adap, reg, data, pmsg->len);
						}
						else
						{
							reg = pmsg->buf[0];
							while(i < pmsg->len-1)
							{
								data[i] = pmsg->buf[i+1];
								i++;
							}
							ret = AIT_I2cm_Write_burst_safe(ait_adap, reg, data, pmsg->len);
						}
					}
				}

				if(ret ==MMP_I2CM_ERR_SLAVE_NO_ACK )
					return -ENXIO;
				if (ret)
					return -ETIMEDOUT;
				ret = 1;
			}
		}
		dev_dbg(&adap->dev, "transfer complete ret = %d\n",ret);

	}
	return ret;
}
#endif
/*
 * Return list of supported functionality.
 */
static u32  ait_phi2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm ait_phi2c_algorithm = {
	.master_xfer	= ait_phi2c_xfer,
	.functionality	= ait_phi2c_func,
};

/*
 * Main initialization routine.
 */
static int __devinit  ait_phi2c_probe(struct platform_device *pdev)	//at91sam9260_twi_device
{
	struct i2c_adapter *adapter;
	ait_i2c_adapter *ait_adapter;
	MMPF_I2CM_ATTRIBUTE *i2c_attr;
	struct resource *res;
	int rc;
	struct ait_i2c_extension* i2c_ext;
	i2c_ext = (struct ait_i2c_extension*) pdev->dev.platform_data;

	if(i2c_ext->uI2cmID == MMPF_I2CM_ID_0)
	{
		i2cm_clk = clk_get(NULL, "i2c_clk");
	}else
		i2cm_clk = clk_get(NULL, "i2c_clk");

	if (IS_ERR(i2cm_clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		rc = -ENODEV;
		goto fail1;
	}

	ait_adapter = kzalloc(sizeof(ait_i2c_adapter), GFP_KERNEL);
	if (ait_adapter == NULL) {
		dev_err(&pdev->dev, "can't allocate inteface, kzalloc failed!\n");
		rc = -ENOMEM;
		goto fail2;
	}
	adapter = &(ait_adapter->i2c_adp);
	i2c_attr = &(ait_adapter->i2c_attr);
	//
  
	i2c_attr->i2cmID 		= i2c_ext->uI2cmID;			//MMPF_I2CM_ID_0 ~ MMPF_I2CM_ID_2 stand for HW I2CM
	i2c_attr->ubSlaveAddr 	= 0;
	i2c_attr->ubRegLen		= 8;				//Indicate register as the 8 bit mode or 16 bit mode.
	i2c_attr->ubDataLen		= 16;				//Indicate data as the 8 bit mode or 16 bit mode.
	i2c_attr->ubDelayTime	= 0;			//Used in SW I2CM (uI2cmID = MMPF_I2CM_ID_SW or MMPF_I2CM_ID_SW_SENSOR)
	   									//To Adjust the speed of software I2CM
	i2c_attr->bDelayWaitEn	= MMP_FALSE;
	i2c_attr->bInputFilterEn= MMP_FALSE;		//HW feature, to filter input noise
	i2c_attr->b10BitModeEn	= MMP_FALSE;			//HW I2CM supports 10 bit slave address, the bit8 and bit9 are in ubSlaveAddr1
	i2c_attr->bClkStretchEn	= MMP_FALSE; 		//HW support stretch clock
	i2c_attr->ubSlaveAddr1	= 0;
	i2c_attr->ubDelayCycle	= 0;  				//When bDelayWaitEn enable, set the delay cycle after each 8 bit transmission
	i2c_attr->ubPadNum		= i2c_ext->pad;      		//HW pad map, the relate pad definition, please refer global register spec.
	   									// ========= For Vision V2=========
										// If I2CM_ID = MMPF_I2CM_ID_0, ubPadNum = 0 stands for I2CM0_SCK = PHI2C_SCL
										//                              ubPadNum = 1 stands for I2CM0_SCK = PSNR_HSYNC
										// IF I2CM_ID = MMPF_I2CM_ID_1, only one pad(I2CM1_SCK = PSNR_SCK) is used
	   									// ========= For Vision V2=========
	//i2c_attr->ulI2cmSpeed 	= MMPF_I2CM_SPEED_HW_250K; //HW I2CM speec control
	i2c_attr->ulI2cmSpeed 	= MMPF_I2CM_SPEED_HW_100K; //HW I2CM speec control
	i2c_attr->sw_clk_pin 	= 0;  		//Used in SW I2CM (uI2cmID = MMPF_I2CM_ID_SW only), indicate the clock pin
	i2c_attr->sw_data_pin	= 0;
	i2c_attr->name 			= 0;
	i2c_attr->deviceID		= 0;

	ait_adapter->i2c_ext.uI2cmID = i2c_ext->uI2cmID;
	ait_adapter->i2c_ext.pad = i2c_ext->pad;
	//ait_adapter->i2c_ext.i2c_hw_spinlock = i2c_ext->i2c_hw_spinlock;
	ait_adapter->i2c_ext.i2c_hw_lock = i2c_ext->i2c_hw_lock;
	//

	//snprintf(adapter->name, sizeof(adapter->name), "VisionV3_PHI2C_Adapter");
	snprintf(adapter->name, sizeof(adapter->name), "VisionV3_I2C_%d_pad%d,nr : %d",i2c_ext->uI2cmID,i2c_ext->pad,pdev->id);
	printk("probe i2c bus %s \r\n",adapter->name);
	adapter->algo = &ait_phi2c_algorithm;
	adapter->class = I2C_CLASS_HWMON;
	adapter->dev.parent = &pdev->dev;
	/* adapter->id == 0 ... only one TWI controller for now */
	ait_phi2c_hwinit(i2c_attr);		/* initialize TWI controller */
	
	platform_set_drvdata(pdev, adapter);

	clk_enable(i2cm_clk);		/* enable peripheral clock */

	adapter->nr = pdev->id;

	rc = i2c_add_numbered_adapter(adapter);
	if (rc) {
		dev_err(&pdev->dev, "Adapter %s registration failed\n",
				adapter->name);
		goto fail3;
	}

	platform_set_drvdata(pdev, (void*)ait_adapter);
	dev_set_drvdata(&adapter->dev, (void*)ait_adapter);
	dev_info(&pdev->dev, "VSNV3 i2c bus driver.\n");
	return 0;

fail3:
	platform_set_drvdata(pdev, NULL);
	kfree(ait_adapter);
///	clk_disable(twi_clk);
fail2:
//	clk_put(twi_clk);
fail1:

	return rc;
}

static int __devexit  ait_phi2c_remove(struct platform_device *pdev)
{
	struct i2c_adapter *adapter = platform_get_drvdata(pdev);
	int rc;

	rc = i2c_del_adapter(adapter);
	platform_set_drvdata(pdev, NULL);

	//clk_disable(i2cm_clk);		/* disable peripheral clock */
	//clk_put(i2cm_clk);

	return rc;
}

//#ifdef CONFIG_PM

/* NOTE: could save a few mA by keeping clock off outside of at91_xfer... */

static int  ait_phi2c_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	//clk_disable(i2cm_clk);
	return 0;
}

static int ait_phi2c_resume(struct platform_device *pdev)
{
	return 0;//clk_enable(i2cm_clk);
}

//#else
//#define at91_i2c_suspend	NULL
//#define at91_i2c_resume		NULL
//#endif

MODULE_ALIAS("platform:VsionV3_i2c_sw");

static struct platform_driver  ait_vsnv3_phi2c_driver = {
	.probe		= ait_phi2c_probe,
	.remove		= __devexit_p(ait_phi2c_remove),
	.suspend		= ait_phi2c_suspend,
	.resume		= ait_phi2c_resume,
	.driver		= {
		//.name	= "VsionV3_phi2c",
		.name = "AIT_I2C",
		.owner	= THIS_MODULE,
	},
};

static int __init  ait_phi2c_init(void)
{
	return platform_driver_register(&ait_vsnv3_phi2c_driver );
}

static void __exit  ait_phi2c_exit(void)
{
	platform_driver_unregister(&ait_vsnv3_phi2c_driver );
}

module_init(ait_phi2c_init);
module_exit(ait_phi2c_exit);

MODULE_AUTHOR("Vincent Chen");
MODULE_DESCRIPTION("I2CM driver for AIT Vision V3");
MODULE_LICENSE("GPL");
