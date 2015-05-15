/* drivers/misc/scaler/scaler-i2c.c - scaler i2c handle
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/scaler-auto.h>

#define SCALER_I2C_RATE 200*1000

static int scaler_i2c_read_device(struct scaler_private_data *scaler, unsigned short reg,
				  int bytes, void *dest, int reg_size)
{
	struct i2c_client *client = scaler->control_data;
	struct i2c_adapter *i2c_adap = client->adapter;
	struct i2c_msg msgs[2];
	int i,res;
	
	if (!dest || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}
		
	client->addr = scaler->ops->slave_addr;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = (unsigned char *)&reg;
	if(reg_size == 2)		
	msgs[0].len = 2;
	else	
	msgs[0].len = 1;
	msgs[0].scl_rate = SCALER_I2C_RATE;
	
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = dest;
	msgs[1].len = bytes;
	msgs[1].scl_rate = SCALER_I2C_RATE; 

	res = i2c_transfer(i2c_adap, msgs, 2);
	
	DBG_SCALER("%s:reg=0x%x,len=%d,rxdata:",__func__, reg, bytes);
	for(i=0; i<bytes; i++)
		DBG_SCALER("0x%x,",((unsigned char *)dest)[i]);
	DBG_SCALER("\n");
	
	if (res == 2)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;
	
}

/* Currently we allocate the write buffer on the stack; this is OK for
 * small writes - if we need to do large writes this will need to be
 * revised.
 */
static int scaler_i2c_write_device(struct scaler_private_data *scaler, unsigned short reg,
				   int bytes, void *src, int reg_size)
{
	struct i2c_client *client = scaler->control_data;
	struct i2c_adapter *i2c_adap = client->adapter;
	struct i2c_msg msgs[1];
	int res;
	unsigned char buf[bytes + 2];
	int i = 0;
	
	if (!src || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	
	client->addr = scaler->ops->slave_addr;
	
	if(scaler->ops->reg_size == 2)
	{
		buf[0] = (reg & 0xff00) >> 8;
		buf[1] = (reg & 0x00ff) & 0xff;
		memcpy(&buf[2], src, bytes);
	}
	else
	{
		buf[0] = reg & 0xff;
		memcpy(&buf[1], src, bytes);
	}


	DBG_SCALER("%s:reg=0x%x,len=%d,txdata:",__func__, reg, bytes);
	for(i=0; i<bytes; i++)
		DBG_SCALER("0x%x,",buf[i]);
	DBG_SCALER("\n");


	if (!src || !i2c_adap) {
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return -EINVAL;
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = buf;
	if(scaler->ops->reg_size  == 2)		
	msgs[0].len = bytes+2;
	else	
	msgs[0].len = bytes+1;	
	msgs[0].scl_rate = SCALER_I2C_RATE;

	res = i2c_transfer(i2c_adap, msgs, 1);
	if (res == 1)
		return 0;
	else if(res == 0)
		return -EBUSY;
	else
		return res;
			
}

int scaler_bulk_read_normal(struct scaler_private_data *scaler,
		     int count, unsigned char *buf, int rate)
{
	int ret;
	unsigned short reg;
	struct i2c_client *client = scaler->control_data;
	client->addr = scaler->ops->slave_addr;
	
	mutex_lock(&scaler->io_lock);
	ret = i2c_master_normal_recv(client, buf, count, rate);
	if(ret == 1)
		ret = 0;
	mutex_unlock(&scaler->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(scaler_bulk_read_normal);


int scaler_bulk_write_normal(struct scaler_private_data *scaler, int count, unsigned char *buf, int rate)
{
	int ret;
	unsigned short reg;
	struct i2c_client *client = scaler->control_data;
	client->addr = scaler->ops->slave_addr;
	
	mutex_lock(&scaler->io_lock);
	ret = i2c_master_normal_send(client, buf, count, rate);
	if(ret == 1)
		ret = 0;
	mutex_unlock(&scaler->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(scaler_bulk_write_normal);



#ifdef CONFIG_HAS_EARLYSUSPEND
static void scaler_suspend(struct early_suspend *h)
{
	struct scaler_private_data *scaler = 
			container_of(h, struct scaler_private_data, early_suspend);
	
	return scaler_device_suspend(scaler);
}

static void scaler_resume(struct early_suspend *h)
{
	struct scaler_private_data *scaler = 
			container_of(h, struct scaler_private_data, early_suspend);

	return scaler_device_resume(scaler);
}
#endif

static int scaler_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct scaler_private_data *scaler;
	int ret,gpio,irq;
	int type = SCALER_BUS_TYPE_I2C;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->adapter->dev, "%s failed\n", __func__);
		return -ENODEV;
	}
	
	scaler = kzalloc(sizeof(struct scaler_private_data), GFP_KERNEL);
	if (scaler == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, scaler);
	
	//atomic_set(&scaler->flags.debug_flag, 1);
	
	scaler->irq = i2c->irq;
	scaler->dev = &i2c->dev;
	scaler->control_data = i2c;
	scaler->read_dev = scaler_i2c_read_device;
	scaler->write_dev = scaler_i2c_write_device;

	ret = scaler_device_init(scaler, type, scaler->irq);
	if(ret)
	{
		printk("%s:fail to regist touch, type is %d\n",__func__, type);
		return -1;
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	if((scaler->ops->suspend) && (scaler->ops->resume))
	{
		scaler->early_suspend.suspend = scaler_suspend;
		scaler->early_suspend.resume = scaler_resume;
		scaler->early_suspend.level = 0x02;
		register_early_suspend(&scaler->early_suspend);
	}
#endif
	
	atomic_set(&scaler->flags.debug_flag, 0);
	printk("%s initialize ok\n",__func__);

	return 0;
}

static int scaler_i2c_remove(struct i2c_client *i2c)
{
	struct scaler_private_data *scaler = i2c_get_clientdata(i2c);

	scaler_device_exit(scaler);

	return 0;
}

	
void scaler_i2c_shutdown (struct i2c_client *i2c)
{		
	struct scaler_private_data *scaler = i2c_get_clientdata(i2c);
	int result = 0;
	
	if(scaler->pdata->init_platform_hw)
	{
		result = scaler->pdata->init_platform_hw(scaler, SCALER_DISABLE);
		if(result)
		{	
			printk("%s:error: line=%d, result=%d\n",__func__, __LINE__, result);
			//return result;
		}
	}

	printk("%s\n",__func__);

}


static const struct i2c_device_id scaler_i2c_id[] = {
	{"auto_scaler_i2c", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, scaler_i2c_id);


static struct i2c_driver scaler_i2c_driver = {
	.driver = {
		.name = "auto_scaler_i2c",
		.owner = THIS_MODULE,
	},
	.probe = scaler_i2c_probe,
	.remove = scaler_i2c_remove,
	.shutdown = scaler_i2c_shutdown,
	.id_table = scaler_i2c_id,
};

static int __init scaler_i2c_init(void)
{
	int ret;

	printk("%s\n", __FUNCTION__);
	ret = i2c_add_driver(&scaler_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register scaler I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall_sync(scaler_i2c_init);

static void __exit scaler_i2c_exit(void)
{
	i2c_del_driver(&scaler_i2c_driver);
}
module_exit(scaler_i2c_exit);

