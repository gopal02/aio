/*
* bq24715 charger driver
*
* Copyright (C) 2013-2013 Liuwei <liuwei3@bitland.com.cn>
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License version 2 as published
* by the Free Software Foundation.
*
* Trademarks are the property of their respective owners.
*/
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/board.h>
#include <linux/interrupt.h>

#if 1
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

#define bq24715_error(fmt, ...)\
    printk(KERN_ERR "bq24715: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#define bq24715_info(fmt, ...) \
    printk(KERN_INFO "bq24715: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#define bq24715_debug(fmt, ...)                                               \
    do {                                                                            \
		if(data->debug)                                                         \
			printk(KERN_INFO "bq24715: %s: "fmt"\n", __func__, ##__VA_ARGS__);\
	}while(0)

//3 cell battery
/*
1. The power path management allows the system to be regulated at battery voltage but does not drop below system
minimum voltage (programmable). With this feature, the system keeps operating even when the battery is completely
discharged or removed. The power path management allows the battery to provide supplement current to the system
to keep the input supply from being overloaded.
2. Setting ChargeCurrent() below 64mA clears the register and terminates charging.
3. EC need write 0x15H – (charge voltage) first then write 0x14H – (charge current). After enable charge, IC will
regulate the charge voltage and current at 0x14H and 0x15H setting.
4. The total input current, from a wall cube or other DC source, is the sum of the system supply current and the
current required by the charger. When the input current exceeds the set input current limit, the bq24715/7
decreases the charge current to provide priority to system load current.
5. If a watchdog timeout occurs all register values keep unchanged but charge is suspended. Write ChargeVoltage() or
wirte ChargeCurrent() commands must be re-sent to reset watchdog timer and resume charging.
*/

#define SYSFS_DEBUG_CHARGER	1

#define ACIN_CHG_PIN		RK30_PIN0_PA6
#define BAT_PRS_PIN			RK30_PIN0_PD7
#define BID0_PIN			RK30_PIN0_PC4//low: batttery and charger present

#define REG_ChgOption		0x12
#define REG_ChgCurr			0x14
#define REG_MaxChgVol		0x15
#define REG_MinSysVol		0x3E
#define REG_InputCurr		0x3F
#define REG_ManufactureID	0xFE//always 0x0040
#define REG_DeviceID		0xFF//identify bq24715/7

#define InputCurr	0x08C0	//2.25A

#define OVER_CHARGED_ALARM		0x8000
#define TERMINATE_CHARGE_ALARM	0x4000
#define OVER_TEMP_ALARM			0x1000
#define INITIALIZED				0x0080
#define DISCHARGING				0x0040
#define FULLY_CHARGED			0x0020
#define FULLY_DISCHARGED		0x0010

struct bq24715_data{
	struct device *dev;
	struct i2c_client *client;
	struct power_supply ac;

	unsigned int interval;
	struct delayed_work chg_delay_work;
	int acin_irq;
	int batin_irq;

	int bat_vol;
	int bat_curr;

	int debug;
};

static int bq24715_read_word(struct i2c_client *client, int reg)
{
	int ret = 0;
	ret = i2c_smbus_read_word_data(client, reg);
	if(ret < 0)
	{
		dev_err(&client->dev, "bq24715_read_word read 0x%x error\n", reg);
		return ret;
	}
	//bq24715_debug("%s: reg[0x%02x] = 0x%02x\n", __func__, reg, ret);
	return ret;
}
static int bq24715_write_word(struct i2c_client *client, int reg, int val)
{
	int ret = 0;
	ret = i2c_smbus_write_word_data(client, reg, val);
	if(ret < 0)
	{
		dev_err(&client->dev, "bq24715_write_word write 0x%x error\n", reg);
		return ret;
	}
	//bq24715_debug("%s: set reg[0x%02x] = 0x%02x\n", __func__, reg, val);
	return ret;
}

//static union power_supply_propval val_current = { 0 };
//static union power_supply_propval val_voltage = { 0 };
static union power_supply_propval val_status = { 0 };

static int __get_battery_status(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	if(strcmp(psy->name, "smart_battery") == 0)
	{
		psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val_status);
	}
	return 0;
}

static int get_battery_status(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __get_battery_status);
	return val_status.intval;
}
/*
static int __get_battery_current(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	if(strcmp(psy->name, "smart_battery") == 0)
	{
		psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val_current);
	}
	return 0;
}

static int get_battery_current(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __get_battery_current);
	return val_current.intval;
}

static int __get_battery_voltage(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	if(strcmp(psy->name, "smart_battery") == 0)
	{
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val_voltage);
	}
	return 0;
}

int get_battery_voltage(void)
{
	class_for_each_device(power_supply_class, NULL, NULL, __get_battery_voltage);
	return val_voltage.intval;
}
*/
extern int get_smart_battery_chg_current();
extern int get_smart_battery_chg_voltage();

static void bq24715_charger_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct bq24715_data *data;
	int bat_curr;
	int bat_vol;
	int bat_status;
	int ret;

	delay_work = container_of(work, struct delayed_work, work);
	data = container_of(delay_work, struct bq24715_data, chg_delay_work);
//	gpio_pull_updown(BAT_PRS_PIN, GPIOPullUp);

	bq24715_debug("ACIN_CHG_PIN = %d, BAT_PRS_PIN = %d", gpio_get_value(ACIN_CHG_PIN), gpio_get_value(BAT_PRS_PIN));
	if(gpio_get_value(ACIN_CHG_PIN) & !gpio_get_value(BAT_PRS_PIN))
	{
		bat_status = get_battery_status();
		if((bat_status == POWER_SUPPLY_STATUS_FULL)/*TODO || (battery alarm)*/)
		{
			bq24715_debug("battery full or battery alarm, terminate charging! bat_status = 0x%x", bat_status);
			ret = bq24715_write_word(data->client, REG_MaxChgVol, 0);
			ret = bq24715_write_word(data->client, REG_ChgCurr, 0);
		}
		else
		{
			bat_vol = get_smart_battery_chg_voltage();
			bq24715_debug("set REG_MaxChgVol[0x%x] to %d", REG_MaxChgVol, bat_vol);
			ret = bq24715_write_word(data->client, REG_MaxChgVol, bat_vol);
			bat_curr = get_smart_battery_chg_current();
			bq24715_debug("set REG_ChgCurr[0x%x] to %d", REG_ChgCurr, bat_curr);
			ret = bq24715_write_word(data->client, REG_ChgCurr, bat_curr);
		}
	}
	else
	{
		bq24715_debug("no adapter or battery, terminate charging!");
		ret = bq24715_write_word(data->client, REG_MaxChgVol, 0);
		ret = bq24715_write_word(data->client, REG_ChgCurr, 0);
	}

	schedule_delayed_work(&data->chg_delay_work, data->interval);
	power_supply_changed(&data->ac);
}
/*
static irqreturn_t bq24715_acin_irq(int irq, void *dev_id)
{
	struct bq24715_data *data = dev_id;
	disable_irq_nosync(data->acin_irq);
	schedule_delayed_work(&data->chg_delay_work, 0);
	return IRQ_HANDLED;
}

static irqreturn_t bq24715_batin_irq(int irq, void *dev_id)
{
	struct bq24715_data *data = dev_id;
	disable_irq_nosync(data->batin_irq);
	schedule_delayed_work(&data->chg_delay_work, 0);
	return IRQ_HANDLED;
}
*/
static int bq24715_charger_init_reg(struct bq24715_data *data)
{
	int ret = 0;
	ret = bq24715_read_word(data->client, REG_DeviceID);
	bq24715_info("REG_DeviceID[0x%x] = 0x%x", REG_DeviceID, ret);

	ret = bq24715_write_word(data->client, REG_InputCurr, InputCurr);
	//ret = bq24715_write_word(data->client, REG_MinSysVol, ??);
	return ret;
}

static enum power_supply_property bq24715_charger_props[] ={
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq24715_charger_get_property(struct power_supply *psy,
							enum power_supply_property psp,
							union power_supply_propval *val)
{
	switch(psp)
	{
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = gpio_get_value(ACIN_CHG_PIN) ? 1 : 0;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

#if SYSFS_DEBUG_CHARGER
static ssize_t bq24715_charger_regs_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq24715_data *data = i2c_get_clientdata(client);
	int chgoption;
	int chgcurr;
	int maxchgvol;
	int minsysvol;
	int inputcurr;
	int manufactureid;
	int deviceid;

	chgoption = bq24715_read_word(client, REG_ChgOption);
	chgcurr = bq24715_read_word(client, REG_ChgCurr);
	maxchgvol = bq24715_read_word(client, REG_MaxChgVol);
	minsysvol = bq24715_read_word(client, REG_MinSysVol);
	inputcurr = bq24715_read_word(client, REG_InputCurr);
	manufactureid = bq24715_read_word(client, REG_ManufactureID);
	deviceid = bq24715_read_word(client, REG_DeviceID);

	return sprintf(buf, "REG_ChgOption[0x%02x] = 0x%04x\n"
						"REG_ChgCurr[0x%02x] = 0x%04x\n"
						"REG_MaxChgVol[0x%02x] = 0x%04x\n"
						"REG_MinSysVol[0x%02x] = 0x%04x\n"
						"REG_InputCurr[0x%02x] = 0x%04x\n"
						"REG_ManufactureID[0x%02x] = 0x%04x\n"
						"REG_DeviceID[0x%02x] = 0x%04x\n",
						REG_ChgOption,
						chgoption,
						REG_ChgCurr,
						chgcurr,
						REG_MaxChgVol,
						maxchgvol,
						REG_MinSysVol,
						minsysvol,
						REG_InputCurr,
						inputcurr,
						REG_ManufactureID,
						manufactureid,
						REG_DeviceID,
						deviceid);
}

static ssize_t bq24715_charger_debug_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq24715_data *data = i2c_get_clientdata(client);

	printk("%s: enter\n", __func__);
	return sprintf(buf, "%d\n", data->debug);
}

static ssize_t bq24715_charger_debug_store(struct device *dev,
							struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq24715_data *data = i2c_get_clientdata(client);

	printk("%s: enter\n", __func__);
	if(buf[0] == 'E')
	{
		data->debug = 1;
		printk("enable debug = %d\n", data->debug);
	}
	else if(buf[0] == 'D')
	{
		data->debug = 0;
		printk("disable debug = %d\n", data->debug);
	}

	return len;
}

static DEVICE_ATTR(regs, 0660, bq24715_charger_regs_show, NULL);
static DEVICE_ATTR(debug, 0660, bq24715_charger_debug_show, bq24715_charger_debug_store);

static struct attribute *bq24715_charger_attributes[] = {
	&dev_attr_regs.attr,
	&dev_attr_debug.attr,
	NULL
};

static struct attribute_group bq24715_charger_attribute_group = {
	.attrs = bq24715_charger_attributes
};
#endif

static int bq24715_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bq24715_data *data;
	int ret;

	bq24715_info("probe enter");
	if(gpio_get_value(BID0_PIN))
	{
		printk("%s :no charger and battery, end the probe\n", __func__);
		return -ENODEV;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if(!data)
	{
		dev_err(&client->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto alloc_mem_failed;
	}
	i2c_set_clientdata(client, data);

	data->dev		= &client->dev;
	data->client	= client;
	data->interval	= msecs_to_jiffies(1*1000);

	data->ac.name			= "adapter";
	data->ac.type			= POWER_SUPPLY_TYPE_MAINS;
	data->ac.properties		= bq24715_charger_props;
	data->ac.num_properties	= ARRAY_SIZE(bq24715_charger_props);
	data->ac.get_property	= bq24715_charger_get_property;

	bq24715_charger_init_reg(data);
	
	ret = gpio_request(BAT_PRS_PIN, "bat_prs");
	if(ret)
		printk("gpio_request BAT_PRS_PIN error\n");
	gpio_pull_updown(BAT_PRS_PIN, GPIOPullUp);
	gpio_direction_input(BAT_PRS_PIN);

	ret = power_supply_register(&client->dev, &data->ac);
	if(ret)
	{
		dev_err(&client->dev, "failed to register battery\n");
		goto power_supply_register_failed;
	}

	INIT_DELAYED_WORK(&data->chg_delay_work, bq24715_charger_work);
	schedule_delayed_work(&data->chg_delay_work, 0);

/*
	data->acin_irq = gpio_to_irq(ACIN_CHG_PIN);
	data->batin_irq = gpio_to_irq(BAT_PRS_PIN);
	ret = request_irq(data->acin_irq, bq24715_acin_irq, IRQF_TRIGGER_RISING, "bq24715_acin", data);
	if(ret)
	{
		dev_err(&client->dev, "failed to request acin irq\n");
		goto request_acin_irq_failed;
	}
	ret = request_irq(data->batin_irq, bq24715_batin_irq, IRQF_TRIGGER_FALLING, "bq24715_batin", data);
	if(ret)
	{
		dev_err(&client->dev, "failed to request batin irq\n");
		goto request_batin_irq_failed;
	}
*/
#if SYSFS_DEBUG_CHARGER
	ret = sysfs_create_group(&client->dev.kobj, &bq24715_charger_attribute_group);
	if(ret)
		pr_err("%s: sysfs_create_group returned err = %d. Abort.\n", __func__, ret);
#endif

	bq24715_info("probe OK!");

	return 0;
/*
request_batin_irq_failed:
	free_irq(data->batin_irq, data);
request_acin_irq_failed:
	free_irq(data->acin_irq, data);*/
power_supply_register_failed:
	kfree(data);
alloc_mem_failed:
	return ret;
}

static int bq24715_charger_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id bq24715_charger_id[] = {
	{"bq24715", 0},
	{},
};

static struct i2c_driver bq24715_charger_driver = {
	.driver		= {
		.name	= "bq24715",
		.owner	= THIS_MODULE,
	},
	.probe		= bq24715_charger_probe,
	.remove		= bq24715_charger_remove,
	.id_table	= bq24715_charger_id,
};

static int __init bq24715_charger_init(void)
{
	    return i2c_add_driver(&bq24715_charger_driver);
}
module_init(bq24715_charger_init);

static void __exit bq24715_charger_exit(void)
{
	    i2c_del_driver(&bq24715_charger_driver);
}
module_exit(bq24715_charger_exit);

MODULE_AUTHOR("Liuwei <liuwei3@bitland.com.cn>");
MODULE_DESCRIPTION("bq24715 charger driver");
MODULE_LICENSE("GPL");
