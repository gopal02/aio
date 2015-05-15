#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/input/mt.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/scaler-auto.h>
struct scaler_private_data *g_scaler;
static struct class *g_scaler_class;
static struct scaler_operate *g_scaler_ops[SCALER_ID_NUMS]; 


static ssize_t scaler_proc_write(struct file *file, const char __user *buffer,
			   size_t count, loff_t *data)
{
	char c;
	int rc;
	int num = 0;
    int param = 0;

	printk("%s:0:close debug; 1:open debug; 2:i2c to dp703; 3:i2c to rk3188; 4:i2s to dp703; 5:i2s to rk3188\n",__func__);

	if(!g_scaler)
	{
		printk("%s:scaler is null\n",__func__);
		return count;
	}
	
	rc = get_user(c, buffer);
	if (rc)
	{		
		atomic_set(&g_scaler->flags.debug_flag, 0);	
		return rc; 
	}
	
	num = c - '0';

	if(num == 0)
	atomic_set(&g_scaler->flags.debug_flag, 0);

	if(num == 1)
	atomic_set(&g_scaler->flags.debug_flag, 1);

#if 1
	if(g_scaler->pdata && g_scaler->pdata->switch_ctrl)
	{
		if(num == 2)
		g_scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_IN, SCALER_SWITCH_I2C, &param);	

		if(num == 3)
		g_scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_OUT, SCALER_SWITCH_I2C, &param);

		if(num == 4)
		g_scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_IN, SCALER_SWITCH_I2S, &param);

		if(num == 5)
		g_scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_OUT, SCALER_SWITCH_I2S, &param);
		
	}
	else
	{
		printk("%s:g_scaler->pdata is null\n",__func__);
	}
#endif
		
	return count; 
}

static const struct file_operations scaler_proc_fops = {
	.owner		= THIS_MODULE, 
	.write		= scaler_proc_write,
};



/**
 * scaler_reg_read: Read a single scaler register.
 *
 * @scaler: Device to read from.
 * @reg: Register to read.
 */
int scaler_reg_read(struct scaler_private_data *scaler, unsigned short reg)
{
	unsigned short val;
	int ret;

	mutex_lock(&scaler->io_lock);

	ret = scaler->read_dev(scaler, reg, scaler->ops->reg_size, &val, scaler->ops->reg_size);

	mutex_unlock(&scaler->io_lock);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(scaler_reg_read);

/**
 * scaler_bulk_read: Read multiple scaler registers
 *
 * @scaler: Device to read from
 * @reg: First register
 * @count: Number of registers
 * @buf: Buffer to fill.
 */
int scaler_bulk_read(struct scaler_private_data *scaler, unsigned short reg,
		     int count, unsigned char *buf)
{
	int ret;

	mutex_lock(&scaler->io_lock);

	ret = scaler->read_dev(scaler, reg, count, buf, scaler->ops->reg_size);

	mutex_unlock(&scaler->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(scaler_bulk_read);


/**
 * scaler_reg_write: Write a single scaler register.
 *
 * @scaler: Device to write to.
 * @reg: Register to write to.
 * @val: Value to write.
 */
int scaler_reg_write(struct scaler_private_data *scaler, unsigned short reg,
		     unsigned short val)
{
	int ret;

	mutex_lock(&scaler->io_lock);

	ret = scaler->write_dev(scaler, reg, scaler->ops->reg_size, &val, scaler->ops->reg_size);

	mutex_unlock(&scaler->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(scaler_reg_write);


int scaler_bulk_write(struct scaler_private_data *scaler, unsigned short reg,
		     int count, unsigned char *buf)
{
	int ret;

	mutex_lock(&scaler->io_lock);

	ret = scaler->write_dev(scaler, reg, count, buf, scaler->ops->reg_size);

	mutex_unlock(&scaler->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(scaler_bulk_write);



/**
 * scaler_set_biscaler: Set the value of a bitfield in a scaler register
 *
 * @scaler: Device to write to.
 * @reg: Register to write to.
 * @mask: Mask of biscaler to set.
 * @val: Value to set (unshifted)
 */
int scaler_set_bits(struct scaler_private_data *scaler, unsigned short reg,
		    unsigned short mask, unsigned short val)
{
	int ret;
	u16 r;

	mutex_lock(&scaler->io_lock);

	ret = scaler->read_dev(scaler, reg, scaler->ops->reg_size, &r, scaler->ops->reg_size);
	if (ret < 0)
		goto out;

	r &= ~mask;
	r |= val;

	ret = scaler->write_dev(scaler, reg, scaler->ops->reg_size, &r, scaler->ops->reg_size);

out:
	mutex_unlock(&scaler->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(scaler_set_bits);


static int scaler_get_id(struct scaler_operate *ops, struct scaler_private_data *scaler, int *value)
{	
	int result = 0;
	char temp[4] = {ops->id_reg & 0xff};
	int i = 0;
#if 0	
	DBG_SCALER("%s:start\n",__func__);
	if(ops->id_reg >= 0)
	{
		for(i=0; i<2; i++)
		{
			if(ops->reg_size == 2)
			{
				temp[0] = ops->id_reg >> 8;
				temp[1] = ops->id_reg & 0xff;
				result = scaler->read_dev(scaler, ops->id_reg, 2, temp, ops->reg_size);
				*value = (temp[0] << 8) | temp[1];
			}
			else
			{
				result = scaler->read_dev(scaler, ops->id_reg, 1, temp, ops->reg_size);
				*value = temp[0];
			}
			if(!result)
			break;

		}

		if(result)
			return result;
		
		if((ops->id_data != SCALER_UNKNOW_DATA)&&(ops->id_data != *value)) 
		{
			printk("%s:id=0x%x is not 0x%x\n",__func__,*value, ops->id_data);
			result = -1;
		}
			
		DBG_SCALER("%s:devid=0x%x\n",__func__,*value);
	}
#endif
	return result;
}



static int scaler_get_version(struct scaler_operate *ops, struct scaler_private_data *scaler)
{
	int result = 0;
	
	return result;
}


static int scaler_chip_init(struct scaler_private_data *scaler, int type)
{
	struct scaler_operate *ops = NULL;
	int result = 0;
	int i = 0;

	if((type <= SCALER_BUS_TYPE_INVALID) || (type >= SCALER_BUS_TYPE_NUMS))
	{
		printk("%s:type=%d is error\n",__func__,type);
		return -1;	
	}

	//init gpio and power on
	if(scaler->pdata->init_platform_hw)
	{
		result = scaler->pdata->init_platform_hw(scaler, SCALER_ENABLE);
		if(result)
		{	
			printk("%s:error: line=%d, result=%d\n",__func__, __LINE__, result);
			//return result;
		}
	}


	for(i=SCALER_ID_INVALID+1; i<SCALER_ID_NUMS; i++)
	{
		ops = g_scaler_ops[i];
		if(!ops)
		{
			printk("%s:error:%p\n",__func__,ops);
			result = -1;	
			continue;
		}
		
		if(ops->bus_type == type)
		{
		
			if(!ops->init || !ops->active)
			{
				printk("%s:error:%p,%p\n",__func__,ops->init,ops->active);
				result = -1;
				continue;
			}
				
			scaler->ops = ops;	//save ops

			result = scaler_get_id(ops, scaler, &scaler->devid);//get id
			if(result < 0)
			{	
				printk("%s:fail to read %s devid:0x%x\n",__func__, ops->name, scaler->devid);	
				continue;
			}
			
			result = scaler_get_version(ops, scaler);	//get version
			if(result < 0)
			{	
				printk("%s:fail to read %s version\n",__func__, ops->name);	
				continue;
			}

			//update firmware
			if(ops->firmware)
			{
				result = ops->firmware(scaler);
				if(result < 0)
				{
					printk("%s:fail to updata firmware ts\n",__func__);
					//return result;
				}
			}
			
			result = ops->init(scaler);
			if(result < 0)
			{
				printk("%s:fail to init scaler\n",__func__);	
				continue;
			}
		
			printk("%s:%s devid:0x%x\n",__func__, scaler->ops->name, scaler->devid);
		}

		break;
					
	}
	
	
	return result;

}



/*
 * This is a threaded IRQ handler so can access I2C/SPI.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C/SPI reads.  We're also assuming that
 * it's rare to get lots of interrupts firing simultaneously so try to
 * minimise I/O.
 */
static irqreturn_t scaler_threaded_interrupt(int irq, void *dev_id)
{
	struct scaler_private_data *scaler = (struct scaler_private_data *)dev_id;	
	
	mutex_lock(&scaler->scaler_lock);

	wake_lock(&scaler->irq_wake);
	if(atomic_read(&scaler->flags.suspend_flag) == 1)
	{
		atomic_set(&scaler->flags.suspend_flag,0);
		msleep(100);
		printk("%s:wait for 50ms\n",__FUNCTION__);
	}

	scaler->ops->irq_handle(scaler);	
	
	wake_unlock(&scaler->irq_wake);
	mutex_unlock(&scaler->scaler_lock);
	printk("%s:irq=%d, name=%s\n",__func__,irq, scaler->ops->name);
	
	return IRQ_HANDLED;
}

static int scaler_ap_wakeup_scaler(struct scaler_private_data *scaler, int wake)
{
	if(scaler->ops->ap_wake_scaler)
		scaler->ops->ap_wake_scaler(scaler, wake);	
	
}

static int scaler_ap_get_scaler_status(struct scaler_private_data *scaler)
{
	int wake = 0;
	
	if(scaler->ops->ap_wake_scaler)
		scaler->ops->scaler_status(scaler, &wake);	

	return wake;
}


static int scaler_ap_show_its_status(struct scaler_private_data *scaler, int wake)
{
	int result = 0;
	
	if(scaler->ops->ap_wake_scaler)
		scaler->ops->ap_status(scaler, wake);	

	return result;
}




static int scaler_irq_init(struct scaler_private_data *scaler)
{
	int result = 0;
	int irq = 0;
		
	//result = gpio_request(scaler->irq, scaler->i2c_id->name);
	//if (result)
	//{
	//	printk("%s:fail to request gpio :%d\n",__func__,scaler->irq);
	//}

	gpio_pull_updown(scaler->irq, PullEnable);
	irq = gpio_to_irq(scaler->irq);
	result = request_threaded_irq(irq, NULL, scaler_threaded_interrupt, scaler->ops->trig, scaler->ops->name, scaler);
	if (result) {
		printk(KERN_ERR "%s:fail to request irq = %d, ret = 0x%x\n",__func__, irq, result);	       
		goto error;	       
	}
	scaler->irq = irq;

	enable_irq_wake(scaler->irq);
	
	atomic_set(&scaler->flags.irq_enable_flag, 1);
	printk("%s:use irq=%d\n",__func__,irq);

error:		
	return result;
}




int scaler_device_init(struct scaler_private_data *scaler, int type, int irq)
{
	struct scaler_platform_data *pdata = scaler->dev->platform_data;
	int result = -1;
	struct proc_dir_entry *scaler_proc_entry;

	mutex_init(&scaler->io_lock);
	mutex_init(&scaler->scaler_lock);
	dev_set_drvdata(scaler->dev, scaler);
	
	scaler->pdata = pdata;
	result = scaler_chip_init(scaler, type);
	if(result < 0)
	{
		printk("%s:touch screen with bus type %d is not exist\n",__func__,type);
		goto out_free_memory;
	}

	atomic_set(&scaler->flags.connect_flag, SCALER_STATUS_INVALID);

	result = scaler_irq_init(scaler);
	if (result) {
		dev_err(scaler->dev,
			"fail to init scaler irq,ret=%d\n",result);
		goto out_free_memory;
	}

	
	wake_lock_init(&scaler->irq_wake, WAKE_LOCK_SUSPEND, "scaler_irq_wake");
	//wake_lock(&scaler->irq_wake);
	scaler_proc_entry = proc_create("driver/scaler_dbg", 0660, NULL, &scaler_proc_fops); 
	
	g_scaler = scaler;
	
	printk("%s:initialized ok,scaler name:%s,devid=%d\n\n",__func__,scaler->ops->name,scaler->devid);

	return result;
		
out_free_memory:	
	kfree(scaler);
	
	printk("%s:line=%d\n",__func__,__LINE__);
	return result;
}



void scaler_device_exit(struct scaler_private_data *scaler)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if((scaler->ops->suspend) && (scaler->ops->resume))
		unregister_early_suspend(&scaler->early_suspend);
#endif  
	kfree(scaler);	
}



int scaler_device_suspend(struct scaler_private_data *scaler)
{
	if(scaler->ops->suspend)
		scaler->ops->suspend(scaler);
	atomic_set(&scaler->flags.suspend_flag, 1);
	return 0;
}


int scaler_device_resume(struct scaler_private_data *scaler)
{
	if(scaler->ops->resume)
		scaler->ops->resume(scaler);
	
	atomic_set(&scaler->flags.suspend_flag, 0);
	
	return 0;
}


int scaler_get_hdmi_in_status(int scaler_id)
{
	int result = 0;

	if(!g_scaler)
	{
		printk("%s:g_scaler is null\n",__func__);
		return -1;
	}

	if((scaler_id <= SCALER_ID_INVALID) || (scaler_id >= SCALER_ID_NUMS))
	{
		printk("%s:scaler id is error,%d\n",__func__, scaler_id);
		return -1;
	}

	if(!g_scaler_ops[scaler_id])
	{
		printk("%s:g_scaler_ops[%d] is null\n",__func__, scaler_id);
		return -1;
	}

	result = atomic_read(&g_scaler->flags.connect_flag); 

	return result;
}
EXPORT_SYMBOL_GPL(scaler_get_hdmi_in_status);



int scaler_register_slave(struct scaler_private_data *scaler,
			struct scaler_platform_data *slave_pdata,
			struct scaler_operate *(*get_scaler_ops)(void))
{
	int result = 0;
	struct scaler_operate *ops = get_scaler_ops();
	if((ops->scaler_id >= SCALER_ID_NUMS) || (ops->scaler_id <= SCALER_ID_INVALID))
	{	
		printk("%s:%s id is error %d\n", __func__, ops->name, ops->scaler_id);
		return -1;	
	}
	g_scaler_ops[ops->scaler_id] = ops;
	printk("%s:%s,id=%d\n",__func__,g_scaler_ops[ops->scaler_id]->name, ops->scaler_id);
	return result;
}


int scaler_unregister_slave(struct scaler_private_data *scaler,
			struct scaler_platform_data *slave_pdata,
			struct scaler_operate *(*get_scaler_ops)(void))
{
	int result = 0;
	struct scaler_operate *ops = get_scaler_ops();
	if((ops->scaler_id >= SCALER_ID_NUMS) || (ops->scaler_id <= SCALER_ID_INVALID))
	{	
		printk("%s:%s id is error %d\n", __func__, ops->name, ops->scaler_id);
		return -1;	
	}
	printk("%s:%s,id=%d\n",__func__,g_scaler_ops[ops->scaler_id]->name, ops->scaler_id);
	g_scaler_ops[ops->scaler_id] = NULL;	
	return result;
}

