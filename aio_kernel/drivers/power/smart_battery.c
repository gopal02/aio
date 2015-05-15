/*
* Smart battery driver
*
* Copyright (C) 2013-2013 Liuwei <liuwei3@bitland.com.cn>
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License version 2 as published
* by the Free Software Foundation.
*
* Trademarks are the property of their respective owners.
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <asm/atomic.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <mach/board.h>

#if 1
#define DBG(x...) printk(KERN_INFO x)
#else
#define DBG(x...) do { } while (0)
#endif

#define smart_battery_error(fmt, ...)\
	printk(KERN_ERR "smart_battery: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#define smart_battery_info(fmt, ...) \
    printk(KERN_INFO "smart_battery: %s: "fmt"\n", __func__, ##__VA_ARGS__)

#define smart_battery_debug(fmt, ...)												\
	do {																			\
		if(bat_info->debug)															\
			printk(KERN_INFO "smart_battery: %s: "fmt"\n", __func__, ##__VA_ARGS__);\
	}while(0)

#define ACIN_CHG_PIN        RK30_PIN0_PA6
#define BAT_PRS_PIN         RK30_PIN0_PD7
#define BID0_PIN            RK30_PIN0_PC4//low: batttery and charger present

#define REG_RSOC				0x0d
#define REG_VOLTAGE				0x09//units: mV
#define REG_CURRENT				0x0a//units: mA
#define REG_CHARGING_CURRENT	0X14//units: mA
#define REG_CHARGING_VOLTAGE	0x15//units: mV
#define REG_STATUS				0x16
#define REG_TEMP				0x08//units 0.1°K
#define REG_FCC					0x10
#define REG_DC					0x18

#define INITIALIZED			0x0080
#define DISCHARGING			0x0040
#define FULLY_CHARGED		0x0020
#define FULLY_DISCHARGED	0x0010

struct smart_battery_info{
	struct device			*dev;
	struct i2c_client		*client;
	struct power_supply		bat;

	unsigned int interval;
	struct workqueue_struct *bat_workqueue;
	struct delayed_work bat_delayed_work;

	int bat_detc_pin;

	int status;
	int capacity;
	int voltage_now;
	int current_now;
	int temp;
	int voltage_chg;
	int current_chg;

	int reg_status;
	int reg_capacity;
	int reg_voltage_now;
	int reg_current_now;
	int reg_temp;
	int reg_voltage_chg;
	int reg_current_chg;

	int debug;
};

struct smart_battery_info *g_bat_info;

static int smart_battery_read(struct i2c_client *client, int reg)
{
	int ret = 0;

	ret = i2c_smbus_read_word_data(client, reg);
	if(ret < 0)
	{
		dev_err(&client->dev, "smart_battery_read i2c read 0x%x error\n", reg);
		return ret;
	}
	return ret;
}

static int smart_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct smart_battery_info *bat_info;

	bat_info = container_of(psy, struct smart_battery_info, bat);
	switch(psp){
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_info->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = gpio_get_value(bat_info->bat_detc_pin) ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat_info->capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat_info->voltage_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bat_info->current_now;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bat_info->temp;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property smart_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

int g_voltage;
int g_current;

int get_smart_battery_chg_current()
{
	return g_current;
}
EXPORT_SYMBOL(get_smart_battery_chg_current);

int get_smart_battery_chg_voltage()
{
	return g_voltage;
}
EXPORT_SYMBOL(get_smart_battery_chg_voltage);


static void smart_battery_update(struct smart_battery_info *bat_info)
{
	bat_info->reg_status = smart_battery_read(bat_info->client, REG_STATUS);
	if(!gpio_get_value(ACIN_CHG_PIN))//ac out, discharging
	{
		bat_info->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	else//ac in
	{
		if(bat_info->reg_status & DISCHARGING)//100% drop to 95%, show not charging even ac in
		{
			bat_info->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		else
		{
			if(bat_info->reg_status & FULLY_CHARGED)//full charged
				bat_info->status = POWER_SUPPLY_STATUS_FULL;
			else
				bat_info->status = POWER_SUPPLY_STATUS_CHARGING;
		}
	}

	bat_info->reg_capacity = smart_battery_read(bat_info->client, REG_RSOC);
	bat_info->capacity = bat_info->reg_capacity;

	bat_info->reg_voltage_now = smart_battery_read(bat_info->client, REG_VOLTAGE);
	bat_info->voltage_now = bat_info->reg_voltage_now * 1000;

	bat_info->reg_current_now = smart_battery_read(bat_info->client, REG_CURRENT);
	bat_info->current_now = bat_info->reg_current_now * 1000;

	bat_info->reg_temp = smart_battery_read(bat_info->client, REG_TEMP);
	bat_info->temp = bat_info->reg_temp - 2731;//units 0.1°C
	
	bat_info->reg_voltage_chg = smart_battery_read(bat_info->client, REG_CHARGING_VOLTAGE);
	bat_info->reg_current_chg = smart_battery_read(bat_info->client, REG_CHARGING_CURRENT);

	g_current = bat_info->reg_current_chg;
	g_voltage = bat_info->reg_voltage_chg;

	smart_battery_debug("status = %d", bat_info->status);
	smart_battery_debug("capacity = %d", bat_info->capacity);
	smart_battery_debug("voltage_now = %d(uV)", bat_info->voltage_now);
	smart_battery_debug("current_now = %d(uA)", bat_info->current_now);
	smart_battery_debug("temp = %d(0.1C)", bat_info->temp);
	smart_battery_debug("voltage_chg = %d(mV)", bat_info->reg_voltage_chg);
	printk("-----------------------------current_chg = %d(mA)", bat_info->reg_current_chg);

	return;
}

static void smart_battery_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct smart_battery_info *bat_info;

	delay_work = container_of(work, struct delayed_work, work);
	bat_info = container_of(delay_work, struct smart_battery_info, bat_delayed_work);

	printk("enter %s\n", __func__);
	if(!gpio_get_value(BAT_PRS_PIN))
	{
		smart_battery_update(bat_info);
		power_supply_changed(&bat_info->bat);
	}
	queue_delayed_work(bat_info->bat_workqueue, &bat_info->bat_delayed_work, bat_info->interval);
}

static ssize_t smart_battery_regs_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smart_battery_info *bat_info = i2c_get_clientdata(client);

	return sprintf(buf, "REG_STATUS[0x%02x] = 0x%04x\n"
						"REG_RSOC[0x%02x] = 0x%04x\n"
						"REG_CURRENT[0x%02x] = 0x%04x\n"
						"REG_VOLTAGE[0x%02x] = 0x%04x\n"
						"REG_CHARGING_CURRENT[0x%02x] = 0x%04x\n"
						"REG_CHARGING_VOLTAGE[0x%02x] = 0x%04x\n"
						"REG_TEMP[0x%02x] = 0x%04x\n",
						REG_STATUS,
						bat_info->reg_status,
						REG_RSOC,
						bat_info->reg_capacity,
						REG_CURRENT,
						bat_info->reg_current_now,
						REG_VOLTAGE,
						bat_info->reg_voltage_now,
						REG_CHARGING_CURRENT,
						bat_info->reg_current_chg,
						REG_CHARGING_VOLTAGE,
						bat_info->reg_voltage_chg,
						REG_TEMP,
						bat_info->reg_temp);
}

static ssize_t smart_battery_debug_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smart_battery_info *bat_info = i2c_get_clientdata(client);

	printk("%s: enter\n", __func__);
	return sprintf(buf, "%d\n", bat_info->debug);
}

static ssize_t smart_battery_debug_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smart_battery_info *bat_info = i2c_get_clientdata(client);

	printk("%s: enter\n", __func__);
	if(buf[0] == 'E')
	{
		bat_info->debug = 1;
		printk("enable debug = %d\n", bat_info->debug);
	}
	else if(buf[0] == 'D')
	{
		bat_info->debug = 0;
		printk("disable debug = %d\n", bat_info->debug);
	}

	return len;
}

static DEVICE_ATTR(debug, 0660, smart_battery_debug_show, smart_battery_debug_store);
static DEVICE_ATTR(regs, 0660, smart_battery_regs_show, NULL);

static struct attribute *smart_battery_attributes[] = {
	&dev_attr_debug.attr,
	&dev_attr_regs.attr,
	NULL
};

static struct attribute_group smart_battery_attribute_group = {
	.attrs = smart_battery_attributes
};

static int smart_battery_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct smart_battery_platform_data *pdata = client->dev.platform_data;
	struct smart_battery_info *bat_info;
	int ret;

	smart_battery_info("---------------------------------- probe enter");

	g_voltage = 0;
	g_current = 0;

	if(gpio_get_value(BID0_PIN))
	{
		smart_battery_error("no charger and battery, end the probe\n");
		return -ENODEV;
	}
	bat_info = kzalloc(sizeof(*bat_info), GFP_KERNEL);
	if(!bat_info)
	{
		dev_err(&client->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto alloc_mem_failed;
	}
	i2c_set_clientdata(client, bat_info);

	bat_info->dev				= &client->dev;
	bat_info->client			= client;
	bat_info->interval			= msecs_to_jiffies(1*1000);
	bat_info->bat_detc_pin		= BAT_PRS_PIN;

	bat_info->bat.name			= "smart_battery";
	bat_info->bat.type			= POWER_SUPPLY_TYPE_BATTERY;
	bat_info->bat.properties	= smart_battery_props;
	bat_info->bat.num_properties = ARRAY_SIZE(smart_battery_props);
	bat_info->bat.get_property	= smart_battery_get_property;

	ret = power_supply_register(&client->dev, &bat_info->bat);
	if(ret)
	{
		dev_err(&client->dev, "failed to register battery\n");
		goto power_supply_register_failed;
	}

	ret = sysfs_create_group(&client->dev.kobj, &smart_battery_attribute_group);
	if(ret)
	{
		dev_err(&client->dev, "%s: sysfs_create_group returned err = %d. Abort.\n", __func__, ret);
	}

	bat_info->bat_workqueue = create_singlethread_workqueue("smart_battery_workqueue");
	INIT_DELAYED_WORK(&bat_info->bat_delayed_work, smart_battery_work);
	queue_delayed_work(bat_info->bat_workqueue, &bat_info->bat_delayed_work, bat_info->interval);
	smart_battery_info("probe OK!");

	return 0;

power_supply_register_failed:
	kfree(bat_info);
alloc_mem_failed:
	return ret;
}

static int smart_battery_remove(struct i2c_client *client)
{
	struct smart_battery_info *bat_info= i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &smart_battery_attribute_group);
	cancel_delayed_work(&bat_info->bat_delayed_work);
	destroy_workqueue(bat_info->bat_workqueue);
	power_supply_unregister(&bat_info->bat);
	kfree(bat_info);

	return 0;
}

static const struct i2c_device_id smart_battery_id[] = {
	{"smart_battery", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smart_battery_id);

static struct i2c_driver smart_battery_driver = {
	.driver		={
		.name	= "smart_battery",
		.owner	= THIS_MODULE,
	},
	.probe		= smart_battery_probe,
	.remove		= smart_battery_remove,
	.id_table	= smart_battery_id,
};

static int __init smart_battery_init(void)
{
	return i2c_add_driver(&smart_battery_driver);
}
module_init(smart_battery_init);

static void __exit smart_battery_exit(void)
{
	i2c_del_driver(&smart_battery_driver);
}
module_exit(smart_battery_exit);

MODULE_AUTHOR("Liuwei <liuwei3@bitland.com.cn>");
MODULE_DESCRIPTION("Smart battery driver");
MODULE_LICENSE("GPL");
