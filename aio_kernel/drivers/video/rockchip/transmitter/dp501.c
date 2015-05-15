#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/dp501.h>
#include <linux/debugfs.h>

#define FIXED_PARAMETER		1

static int dp501_write_reg(struct i2c_client *client,char index,char reg,char val)
{
	int ret;
	if(index == 0)  //page 0
	{
		client->addr = (DP501_P0_ADDR >> 1);
	}
	else if(index == 1) //page1
	{
		client->addr = (DP501_P1_ADDR >> 1);
	}
	else if(index == 2) //page 2
	{
		client->addr = (DP501_P2_ADDR >> 1);
	}
	else if(index == 3)
	{
		client->addr = (DP501_P3_ADDR >> 1);
	}
	else
	{
		dev_err(&client->dev,"invalid page number\n");
		return -EINVAL;
	}
	ret = i2c_master_reg8_send(client, reg, &val, 1,DP501_SCL_RATE);
	if(ret < 0)
	{
		
		dev_err(&client->dev,"%s page%d:0x%x err\n",__func__,index,reg);
		ret = -EINVAL;
	}

	return ret;
	
}

static char dp501_read_reg(struct i2c_client *client,char index,char reg)
{
	int ret;
	char val;
	if(index == 0)  //page 0
	{
		client->addr = (DP501_P0_ADDR >> 1);
	}
	else if(index == 1) //page1
	{
		client->addr = (DP501_P1_ADDR>>1);
	}
	else if(index == 2) //page 2
	{
		client->addr = (DP501_P2_ADDR>>1);
	}
	else if(index == 3)
	{
		client->addr = (DP501_P3_ADDR>>1);
	}
	else
	{
		dev_err(&client->dev,"invalid page number\n");
		return -EINVAL;
	}

	
	ret = i2c_master_reg8_recv(client, reg, &val, 1, DP501_SCL_RATE);
	if(ret < 0)
	{
		dev_err(&client->dev,"%s page%d:0x%x err\n",__func__,index,reg);
		return  -EINVAL;
	}

	return val;
	
}
static int get_dp_chip_id(struct i2c_client *client)
{
	char c1,c2;
	int id;
	c1 = dp501_read_reg(client,2,CHIP_ID_L);
    	c2 = dp501_read_reg(client,2,CHIP_ID_H);
	id = c2;
	return (id<<8)|c1;
	return 0;
}

static int dp501_init(struct i2c_client *client, int resume)
{
	char val = 0;
	char val1 = 0;
	char bit2 = 0x04;
	int retry = 10;
	unsigned long later;
	int count;

	printk("resume = %d\n", resume);

	dp501_write_reg(client,2,0x24,0x02);
	dp501_write_reg(client,2,0x25,0x04);
	dp501_write_reg(client,2,0x26,0x10); //PIO setting
#if FIXED_PARAMETER
	dp501_write_reg(client,0,0x27,0x00);
#else
	dp501_write_reg(client,0,0x27,0x30); 
#endif
	dp501_write_reg(client,0,0x28,0x07); 
	dp501_write_reg(client,0,0x24,0xC4); 

	dp501_write_reg(client,0,0x5D,0x0A); 
	dp501_write_reg(client,0,0x5E,0x82);

	dp501_write_reg(client,0,0x72,0xA9);

	dp501_write_reg(client,0,0x8F,0x02);

#if FIXED_PARAMETER
        dp501_write_reg(client, 0, 0x10, 0xe0); // HTotal
        dp501_write_reg(client, 0, 0x11, 0x06); // HTotal -- 0x7da = 2010
        dp501_write_reg(client, 0, 0x12, 0x70); // HStart
        dp501_write_reg(client, 0, 0x13, 0x00); // HStart -- 0x3c = 60
        dp501_write_reg(client, 0, 0x14, 0x40); // HWidth
        dp501_write_reg(client, 0, 0x15, 0x06); // HWidth -- 0x780 = 1920
        dp501_write_reg(client, 0, 0x16, 0x9e); // VTotal
        dp501_write_reg(client, 0, 0x17, 0x03); // VTotal -- 0x46a = 1130
        dp501_write_reg(client, 0, 0x18, 0x17); // VStart
        dp501_write_reg(client, 0, 0x19, 0x00); // VStart -- 0x1e = 30
        dp501_write_reg(client, 0, 0x1a, 0x84); // VHeight
        dp501_write_reg(client, 0, 0x1b, 0x03); // VHeight -- 0x438 = 1080
        dp501_write_reg(client, 0, 0x1c, 0x20); // HspHsw
        dp501_write_reg(client, 0, 0x1d, 0x80); // HspHsw -- 0x801e = 30, horizontal sync polarity = 1, hsync pulse width = 30
        dp501_write_reg(client, 0, 0x1e, 0x05); // VspVsw
        dp501_write_reg(client, 0, 0x1f, 0x80); // VspVsw -- 0x800f = 15, vertical sync polarity = 1, vsync pulse width = 15

        dp501_write_reg(client, 0, 0x20, 0x20); // this is register's power-on-reset-value
        dp501_write_reg(client, 0, 0x26, 0x00);
#endif

	do{
		mdelay(50); //delay 50ms
		val = dp501_read_reg(client,0,0x8d);
		printk("HPD val = %2x\n", val);
		if(val & bit2){
			break;
		}
	}while(retry -- > 0); 

	if(!(val & bit2)){
		printk("HPD check failed\n");
		return 0;
	}
	retry = 10;
	printk("HPD check successed\n");
	do{
		dp501_write_reg(client,0,0x5F,0x0D); //trigger training
		later = jiffies + msecs_to_jiffies(2000);
		count = 0;
		while(1){
			count ++;
			val = dp501_read_reg(client, 0, 0x5f);
			if((val & 0x03) == 0x00) {
				// training complete
				printk("training complete, 0x5f read %d\n", count);
				break;
			}
			if(time_after(jiffies, later)) {
				// timeout
				printk("training in pregress, 0x5f read %d\n", count);
				break;
			}
			mdelay(100);
		}
		val = dp501_read_reg(client, 0, 0x63);
		if(val == 0x77) {
			// training success.
			printk("training sucess, retry=%d\n", retry);
			dp501_write_reg(client, 0, 0x74, 0x09);     // enable DP to output video
			if(resume)
			{
				return 0;
			}
			else
			{
				resume += 1;
				mdelay(10);
				continue;
			}
		}else{
			mdelay(10);
		}
	}while(retry--);

	printk("Training failed, retry=%d\n", retry);
	return 0;
	
	
#if 0
	dp501_write_reg(client,2,0x00,0x6C);
	dp501_write_reg(client,2,0x01,0x68);
	dp501_write_reg(client,2,0x02,0x28);
	dp501_write_reg(client,2,0x03,0x2A);
	dp501_write_reg(client,2,0x16,0x50);
	dp501_write_reg(client,2,0x24,0x22);
	dp501_write_reg(client,2,0x25,0x04);
	dp501_write_reg(client,2,0x26,0x10); //PIO setting
	
	dp501_write_reg(client,0,0x0a,0x0c); //block 74 & 76
	dp501_write_reg(client,0,0x20,0x00); 
	dp501_write_reg(client,0,0x27,0x30); //auto detect CRTC 
	dp501_write_reg(client,0,0x2f,0x82); //reset tpfifo at v blank 
	dp501_write_reg(client,0,0x24,0xc0); //DVO mapping ; crtc follow mode
	dp501_write_reg(client,0,0x28,0x07); //crtc follow mode
	dp501_write_reg(client,0,0x87,0x7f); //aux retry
	dp501_write_reg(client,0,0x88,0x1e); //aux retry
	dp501_write_reg(client,0,0xbb,0x06); //aux retry
	dp501_write_reg(client,0,0x72,0xa9); //DPCD readable
	dp501_write_reg(client,0,0x60,0x00); //Scramble on
	dp501_write_reg(client,0,0x8f,0x02); //debug select, read P0.0x8d[2] can check HPD


	//second, set up training
	dp501_write_reg(client,0,0x5d,0x06); //training link rate(2.7Gbps)
	dp501_write_reg(client,0,0x5e,0x84); //training lane count(4Lanes),
	dp501_write_reg(client,0,0x74,0x00); //idle pattern
	dp501_write_reg(client,0,0x5f,0x0d); //trigger training
	mdelay(100); //delay 100ms

	//then, check training result
	val = dp501_read_reg(client,0,0x63); 
	val1 = dp501_read_reg(client,0,0x64); //Each 4bits stand for one lane, 0x77/0x77 means training succeed with 4Lanes.
	dev_info(&client->dev,"training result:>>val:0x%x>>val1:0x%x\n",val,val1);
	
	return 0;
#endif
}



static int edp_reg_show(struct seq_file *s, void *v)
{
	int i = 0;
	char val;
	struct  dp501 *dp501= s->private;

	seq_printf(s,"page 0:\n");
	for(i=0;i< MAX_REG;i++)
	{
		val = dp501_read_reg(dp501->client,0,i);
		seq_printf(s,"0x%02x>>0x%02x\n",i,val);
	}

	seq_printf(s,"page 1:\n");
	for(i=0;i< MAX_REG;i++)
	{
		val = dp501_read_reg(dp501->client,1,i);
		seq_printf(s,"0x%02x>>0x%02x\n",i,val);
	}

	seq_printf(s,"page 2:\n");
	for(i=0;i< MAX_REG;i++)
	{
		val = dp501_read_reg(dp501->client,0,i);
		seq_printf(s,"0x%02x>>0x%02x\n",2,val);
	}

	seq_printf(s,"page 3:\n");
	for(i=0;i< MAX_REG;i++)
	{
		val = dp501_read_reg(dp501->client,3,i);
		seq_printf(s,"0x%02x>>0x%02x\n",i,val);
	}
	
	return 0;
}

static int edp_reg_open(struct inode *inode, struct file *file)
{
	struct dp501 *dp501 = inode->i_private;
	return single_open(file,edp_reg_show,dp501);
}

static const struct file_operations edp_reg_fops = {
	.owner		= THIS_MODULE,
	.open		= edp_reg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void dp501_early_suspend(struct early_suspend *h)
{
	struct dp501 *dp501 = container_of(h, struct dp501, early_suspend);
	if(dp501->pdata->dvdd33_en_pin != INVALID_GPIO)
		gpio_set_value(dp501->pdata->dvdd33_en_pin,!dp501->pdata->dvdd33_en_val);
	if(dp501->pdata->dvdd18_en_pin != INVALID_GPIO)
		gpio_set_value(dp501->pdata->dvdd18_en_pin,!dp501->pdata->dvdd18_en_val);
	
}

static void dp501_late_resume(struct early_suspend *h)
{
	struct dp501 *dp501 = container_of(h, struct dp501, early_suspend);
	if(dp501->pdata->dvdd33_en_pin != INVALID_GPIO)
		gpio_set_value(dp501->pdata->dvdd33_en_pin,dp501->pdata->dvdd33_en_val);
	if(dp501->pdata->dvdd18_en_pin != INVALID_GPIO)
		gpio_set_value(dp501->pdata->dvdd18_en_pin,dp501->pdata->dvdd18_en_val);
	if(dp501->pdata->edp_rst_pin != INVALID_GPIO) {
		gpio_set_value(dp501->pdata->edp_rst_pin,0);
		msleep(10);
		gpio_set_value(dp501->pdata->edp_rst_pin,1);
	}
	dp501->edp_init(dp501->client, 1);
}
#endif
static int dp501_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret;
	
	struct dp501 *dp501 = NULL;
	int chip_id;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
	}
	dp501 = kzalloc(sizeof(struct dp501), GFP_KERNEL);
	if (dp501 == NULL)
	{
		dev_err(&client->dev,"alloc for struct dp501 fail\n");
		ret = -ENOMEM;
	}

	dp501->client = client;
	dp501->pdata = client->dev.platform_data;
	i2c_set_clientdata(client,dp501);
	if(dp501->pdata->power_ctl)
		dp501->pdata->power_ctl();

	debugfs_create_file("edp-reg", S_IRUSR,NULL,dp501,&edp_reg_fops);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	dp501->early_suspend.suspend = dp501_early_suspend;
	dp501->early_suspend.resume = dp501_late_resume;
    	dp501->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	register_early_suspend(&dp501->early_suspend);
#endif

	chip_id = get_dp_chip_id(client);
	dp501->edp_init = dp501_init;
	dp501->edp_init(client, 0);


	printk("edp dp%x probe ok\n",chip_id);

	return ret;
}

static int __devexit dp501_i2c_remove(struct i2c_client *client)
{
	return 0;
}


static const struct i2c_device_id id_table[] = {
	{"dp501", 0 },
	{ }
};

static struct i2c_driver dp501_i2c_driver  = {
	.driver = {
		.name  = "dp501",
		.owner = THIS_MODULE,
	},
	.probe		= &dp501_i2c_probe,
	.remove     	= &dp501_i2c_remove,
	.id_table	= id_table,
};


static int __init dp501_module_init(void)
{
	return i2c_add_driver(&dp501_i2c_driver);
}

static void __exit dp501_module_exit(void)
{
	i2c_del_driver(&dp501_i2c_driver);
}

fs_initcall_sync(dp501_module_init);
module_exit(dp501_module_exit);

