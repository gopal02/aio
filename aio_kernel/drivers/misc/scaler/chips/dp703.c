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
#include <linux/input/mt.h>
#include <mach/gpio.h>
#include <mach/board.h> 
#include <linux/suspend.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif	 
#include <linux/scaler-auto.h>
	 
static struct i2c_client *this_client;	 
	 
#define DP703_ID_REG					0x00
#define DP703_DEVID					0x00

#define DP703_REG_STATUS				0x10
#define DP703_REG_STATUS_FEEDBACK			0X12

#define DP703_REG_SWITCH_TO_SCALER			0x13
#define DP703_REG_SWITCH_TO_SOC				0x15

//DP703_REG_STATUS
#define DP703_STATUS_HDMI_IN				0x04
#define DP703_STATUS_HDMI_OUT				0x05

//DP703_REG_STATUS_FEEDBACK
#define DP703_STATUS_FEEDBACK_SUCCESS			0x01
#define DP703_STATUS_FEEDBACK_INVALID			0x02

//DP703_REG_SWITCH_TO_SCALER
#define DP703_CMD_SWITCH_TO_SCALER			0x06
#define DP703_DATA_SWITCH_TO_SCALER_START		0x01
#define DP703_DATA_SWITCH_TO_SCALER_END			0x02

//DP703_REG_SWITCH_TO_SOC
#define DP703_CMD_SWITCH_TO_SOC				0x07
#define DP703_DATA_SWITCH_TO_SOC_START			0x01
#define DP703_DATA_SWITCH_TO_SOC_END			0x02

#define DP703_HDMI_FREQ_REG                     0x16

#define DP703_HDMI_FREQ_32000                   0x03
#define DP703_HDMI_FREQ_44100                   0x00
#define DP703_HDMI_FREQ_48000                   0x02
#define DP703_HDMI_FREQ_88200                   0x08
#define DP703_HDMI_FREQ_96000                   0x0a
#define DP703_HDMI_FREQ_176400                  0x0c
#define DP703_HDMI_FREQ_192000                  0x0e

enum dp703_hdmi_freq
{
    DP703_E_HDMI_FREQ_32000,
    DP703_E_HDMI_FREQ_44100,
    DP703_E_HDMI_FREQ_48000,
    DP703_E_HDMI_FREQ_88200,
    DP703_E_HDMI_FREQ_96000,
    DP703_E_HDMI_FREQ_176400,
    DP703_E_HDMI_FREQ_192000,
    DP703_E_HDMI_FREQ_BUTT,
};

extern suspend_state_t get_suspend_state(void);

#if defined(CONFIG_KEYS_RK29)
	extern void rk29_send_power_key(int state);
#else
{
	void rk29_send_power_key(int state)
	{

	}
}
#endif

#if defined(CONFIG_SCALER_DP703_FIRMWARE_SPI)
extern int spi_firmware_update_dp703(struct scaler_private_data *scaler);
#endif

#if defined(CONFIG_SCALER_DP703_FIRMWARE_I2C)
extern int i2c_firmware_update_dp703(struct scaler_private_data *scaler);
#endif


static int scaler_enable_power_key(struct scaler_private_data *scaler, int irq, int enable)
{
	if((enable) && (!atomic_read(&scaler->flags.irq_enable_flag)))
	{
		enable_irq(irq);
		atomic_set(&scaler->flags.irq_enable_flag, 1); 
	}
	else if((!enable) && (atomic_read(&scaler->flags.irq_enable_flag)))
	{
		disable_irq_nosync(irq);
		atomic_set(&scaler->flags.irq_enable_flag, 0); 
	}

	DBG_SCALER("%s:irq=%d,en=%d\n",__func__, irq, enable);
	return 0;
}

static int scaler_active(struct scaler_private_data *scaler, int enable)
{	
	int result = 0;
	if(enable)
	{
		//gpio_direction_output(scaler->pdata->power_pin, GPIO_LOW);

		//gpio_direction_output(scaler->pdata->reset_pin, GPIO_HIGH);
	}
	
	return result;
}


static int scaler_init(struct scaler_private_data *scaler)
{
	int result = 0;
	int i = 0;
	char temp[1] = {0};
	
	//scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_OUT);	
	
#if 0	
	//for test
	for(i=0x10; i<0x16; i++)
	{
		result = scaler->read_dev(scaler, i/*scaler->ops->id_reg*/, 1, temp, scaler->ops->reg_size);
		if(!result)
		printk("%s:reg[%d]=0x%x\n", __func__, i, temp[0]);
		else	
		printk("%s:data error,ret=%d\n", __func__, result);
	}
#endif	
	printk("%s:name=%s ok\n",__func__, scaler->ops->name);
	
	return result;
}


static int scaler_reset(struct scaler_private_data *scaler, int reset)
{
	int result = 0;
	if(reset)
	{	
		gpio_direction_output(scaler->pdata->reset_pin, GPIO_LOW);

	}
	else
	{	
		gpio_direction_output(scaler->pdata->reset_pin, GPIO_HIGH);
	}
	return result;
}


static int ap_wakeup_scaler(struct scaler_private_data *scaler, int wake)
{
	int result = 0;

	
	return result;
}


static int ap_get_scaler_status(struct scaler_private_data *scaler, int *status)
{
	int result = 0;
	int i = 0;
	int value1=0,value2=0;
	
	//gpio_request(scaler->pdata->scaler_status_pin,"scaler_status_pin");
	gpio_direction_input(scaler->pdata->scaler_status_pin);
	gpio_pull_updown(scaler->pdata->scaler_status_pin, PullDisable);
	for(i=0; i<3; i++)
	{
		value1 = gpio_get_value(scaler->pdata->scaler_status_pin);
		if(value1 < 0)
			continue;
		mdelay(1);
		value2 = gpio_get_value(scaler->pdata->scaler_status_pin);
		if(value2 < 0)
			continue;

		if(value1 == value2)
			break;
	}

	if(i>=3)
	return -1;
	else	
	return value1;
	
}

static int ap_show_its_status(struct scaler_private_data *scaler, int status)
{
	int result = 0;
	
	//gpio_request(scaler->pdata->ap_status_pin,"ap_status_pin");
	gpio_direction_output(scaler->pdata->ap_status_pin, status);
	
	return result;
}

static int scaler_get_hdmi_freq(struct scaler_private_data *scaler, int *freq)
{
    int result = 0;
    char hdmi_freq = 0;
    char detected_freq = 0;
       
    result = scaler->read_dev(scaler, DP703_HDMI_FREQ_REG, 1, &hdmi_freq, scaler->ops->reg_size);
    if (result)
    {
        printk("%s:line=%d, error, result = %d\n", __func__, __LINE__, result);
        return -1;
    }

    printk("%s:line=%d hdmi freq reg: 0x%x\n", __func__, __LINE__, hdmi_freq);
    detected_freq = hdmi_freq / 0x10;

    switch (detected_freq)
    {
        case DP703_HDMI_FREQ_32000:
            *freq = DP703_E_HDMI_FREQ_32000;
            break;
        case DP703_HDMI_FREQ_44100:
            *freq = DP703_E_HDMI_FREQ_44100;
            break;
        case DP703_HDMI_FREQ_48000:
            *freq = DP703_E_HDMI_FREQ_48000;
            break;
        case DP703_HDMI_FREQ_88200:
            *freq = DP703_E_HDMI_FREQ_88200;
            break;
        case DP703_HDMI_FREQ_96000:
            *freq = DP703_E_HDMI_FREQ_96000;
            break;
        case DP703_HDMI_FREQ_176400:
            *freq = DP703_E_HDMI_FREQ_176400;
            break;
         case DP703_HDMI_FREQ_192000:
            *freq = DP703_E_HDMI_FREQ_192000;
            break;
        default:
            printk("%s:line=%d dp703 detected hdmi freq error!!! freq = %d\n", 
                __func__, __LINE__, detected_freq);
            return -1;
            break;
    }

    return 0;
}

static int scaler_irq_handle(struct scaler_private_data *scaler)
{
	int result = 0;
	char status = 0;
	char cmd[2] = {0, 0};
    int freq = 0;
    int param = 0;

	result = scaler->read_dev(scaler, DP703_REG_STATUS, 1, &status, scaler->ops->reg_size);
	if(result)
	{	
		printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);		
		cmd[0] = DP703_STATUS_FEEDBACK_INVALID;	
		result = scaler->write_dev(scaler, DP703_REG_STATUS_FEEDBACK, 1, cmd, scaler->ops->reg_size);
		if(result)
		{	
			printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
			return result;
		}

		return -1;
	}
	else
	{		
		cmd[0] = DP703_STATUS_FEEDBACK_SUCCESS;
		result = scaler->write_dev(scaler, DP703_REG_STATUS_FEEDBACK, 1, cmd, scaler->ops->reg_size);
		if(result)
		{	
			printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
			return result;
		}
	}


	if(status == DP703_STATUS_HDMI_IN)
	{
		//atomic_set(&scaler->flags.connect_flag, SCALER_STATUS_HDMI_IN);	
				
		cmd[0] = DP703_CMD_SWITCH_TO_SCALER;	
		cmd[1] = DP703_DATA_SWITCH_TO_SCALER_START;
		result = scaler->write_dev(scaler, DP703_REG_SWITCH_TO_SCALER, 2, cmd, scaler->ops->reg_size);
		if(result)
		{	
			printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
			return result;
		}

		//send irq pulse to dp703	
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_LOW);
		msleep(2);
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_HIGH);

        result = scaler_get_hdmi_freq(scaler, &freq);
        if (-1 == result)
        {
            printk("%s:line=%d,get hdmi freq fail!\n",__func__,__LINE__);
			return result;
        }

		scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_IN, SCALER_SWITCH_I2S, &freq);			
		scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_IN, SCALER_SWITCH_I2C, &param);	

        atomic_set(&scaler->flags.connect_flag, SCALER_STATUS_HDMI_IN);	
		scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_IN, SCALER_SWITCH_KEY, &param);	

		cmd[0] = DP703_CMD_SWITCH_TO_SCALER;	
		cmd[1] = DP703_DATA_SWITCH_TO_SCALER_END;
		result = scaler->write_dev(scaler, DP703_REG_SWITCH_TO_SCALER, 2, cmd, scaler->ops->reg_size);
		if(result)
		{	
			printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
			return result;
		}

		//send irq pulse to dp703		
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_LOW);
		msleep(2);
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_HIGH);

		DBG_SCALER("%s:i2c,vol and codec i2s1 switch to dp703 ok\n",__func__);
		
		//if working then sleep
		if(get_suspend_state() == PM_SUSPEND_ON)
		{
			rk29_send_power_key(1);
			rk29_send_power_key(0);
		}
		scaler_enable_power_key(scaler, gpio_to_irq(RK30_PIN0_PA4), 0);//disable power key irq
		
		DBG_SCALER("%s:disable power key\n",__func__);
	}
	else if(status == DP703_STATUS_HDMI_OUT)
	{
		//atomic_set(&scaler->flags.connect_flag, SCALER_STATUS_HDMI_OUT);

		cmd[0] = DP703_CMD_SWITCH_TO_SOC;	
		cmd[1] = DP703_DATA_SWITCH_TO_SOC_START;
		result = scaler->write_dev(scaler, DP703_REG_SWITCH_TO_SOC, 2, cmd, scaler->ops->reg_size);
		if(result)
		{	
			printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
			return result;
		}

		//send irq pulse to dp703	
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_LOW);
		msleep(2);
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_HIGH);
		
		scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_OUT, SCALER_SWITCH_I2C, &param);	

        atomic_set(&scaler->flags.connect_flag, SCALER_STATUS_HDMI_OUT);
		scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_OUT, SCALER_SWITCH_I2S, &param);	
		scaler->pdata->switch_ctrl(SCALER_STATUS_HDMI_OUT, SCALER_SWITCH_KEY, &param);	

		cmd[0] = DP703_CMD_SWITCH_TO_SOC;	
		cmd[1] = DP703_DATA_SWITCH_TO_SOC_END;
		result = scaler->write_dev(scaler, DP703_REG_SWITCH_TO_SOC, 2, cmd, scaler->ops->reg_size);
		if(result)
		{	
			printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
			return result;
		}

		//send irq pulse to dp703	
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_LOW);
		msleep(2);
		gpio_direction_output(scaler->pdata->ap_wakeup_scaler, GPIO_HIGH);

		DBG_SCALER("%s:i2c,vol and codec i2s0 switch to soc ok\n",__func__);
		
		//if sleep then wake
		if(get_suspend_state() != PM_SUSPEND_ON)
		{
			rk29_send_power_key(1);
			rk29_send_power_key(0);
		}
		scaler_enable_power_key(scaler, gpio_to_irq(RK30_PIN0_PA4), 1);//enable power key irq
		
		DBG_SCALER("%s:enable power key\n",__func__);
	}

	printk("%s:name=%s,reg[%d]=0x%x, connect_flag=%d\n",__func__, scaler->ops->name, DP703_REG_STATUS, status, atomic_read(&scaler->flags.connect_flag));
	
	return result;
}


static int scaler_firmware(struct scaler_private_data *scaler)
{
	int result = 0;
	
#if defined(CONFIG_SCALER_DP703_FIRMWARE_SPI)
	result = spi_firmware_update_dp703(scaler);
#endif
#if defined(CONFIG_SCALER_DP703_FIRMWARE_I2C)
	result = i2c_firmware_update_dp703(scaler);
#endif

	return	result;
}


static int scaler_suspend(struct scaler_private_data *scaler)
{
	struct scaler_platform_data *pdata = scaler->pdata;

	if(scaler->ops->active)
		scaler->ops->active(scaler, 0);
	
	return 0;
}


static int scaler_resume(struct scaler_private_data *scaler)
{
	struct scaler_platform_data *pdata = scaler->pdata;
	
	if(scaler->ops->active)
		scaler->ops->active(scaler, 1);
	return 0;
}


struct scaler_operate scaler_dp703_ops = {
	.name				= "dp703",
	.scaler_id			= SCALER_ID_DP703,	//i2c id number
	.bus_type			= SCALER_BUS_TYPE_I2C,	
	.slave_addr			= 0x0D,
	.reg_size			= 1,
	.id_reg				= DP703_ID_REG,
	.id_data			= SCALER_UNKNOW_DATA,				
	.trig				= IRQF_TRIGGER_FALLING,	
	
	.active				= scaler_active,	
	.init				= scaler_init,
	.reset 				= scaler_reset,
	.irq_handle 			= scaler_irq_handle,
	.ap_wake_scaler 		= ap_wakeup_scaler,
	.scaler_status			= ap_get_scaler_status,
	.ap_status 			= ap_show_its_status,
	.firmware			= scaler_firmware,
	.suspend			= scaler_suspend,
	.resume				= scaler_resume,	
	.misc_name			= NULL,
	.misc_dev 			= NULL,
};

/****************operate according to scaler chip:end************/

//function name should not be changed
static struct scaler_operate *scaler_get_ops(void)
{
	return &scaler_dp703_ops;
}


static int __init scaler_dp703_init(void)
{
	struct scaler_operate *ops = scaler_get_ops();
	int result = 0;
	result = scaler_register_slave(NULL, NULL, scaler_get_ops);	
	//printk("%s\n",__func__);
	return result;
}

static void __exit scaler_dp703_exit(void)
{
	struct scaler_operate *ops = scaler_get_ops();
	scaler_unregister_slave(NULL, NULL, scaler_get_ops);
}


subsys_initcall(scaler_dp703_init);
module_exit(scaler_dp703_exit);

