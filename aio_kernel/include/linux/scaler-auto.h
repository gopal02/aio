#ifndef __SCALER_AUTO_H
#define __SCALER_AUTO_H
#include <linux/miscdevice.h>
#include <linux/wakelock.h>


#define SCALER_ENABLE		1
#define	SCALER_DISABLE		0
#define SCALER_UNKNOW_DATA	-1


struct scaler_private_data;


enum scaler_output_type
{	
	SCALER_OUT_INVALID = 0,
	SCALER_OUT_LVDS,
	SCALER_OUT_VGA,
	SCALER_OUT_HDMI,
	SCALER_OUT_DP,
	SCALER_OUT_NUMS,
};


enum scaler_input_type
{	
	SCALER_IN_INVALID = 0,
	SCALER_IN_VGA,
	SCALER_IN_DVI,
	SCALER_IN_RGB,
	SCALER_IN_HDMI,
	SCALER_IN_YPBPR,
	SCALER_IN_YCBCR,
	SCALER_IN_MYDP,
	SCALER_IN_NUMS,
};


enum scaler_bus_type{
	SCALER_BUS_TYPE_INVALID = 0,
	
	SCALER_BUS_TYPE_I2C,
	SCALER_BUS_TYPE_UART,
	SCALER_BUS_TYPE_SPI,
	
	SCALER_BUS_TYPE_NUMS,
};


enum scaler_id
{
	SCALER_ID_INVALID = 0,
	//i2c
	SCALER_ID_DP703,
	SCALER_ID_TV5735,
	SCALER_ID_TV5930,

	//uart
	//to do
	
	SCALER_ID_NUMS,
};

enum scaler_connect_status
{
	SCALER_STATUS_INVALID = 0,
		
	SCALER_STATUS_HDMI_IN,
	SCALER_STATUS_HDMI_OUT,
	
	SCALER_STATUS_NUMS,
};

enum scaler_swtich_type
{
	SCALER_SWITCH_INVALID = 0,
		
	SCALER_SWITCH_I2C,
	SCALER_SWITCH_I2S,
	SCALER_SWITCH_KEY,
	
	SCALER_SWITCH_NUMS,
};



struct scaler_chip_name{
	char name[32];
};


static const struct scaler_chip_name chip_name[] = 
{
	/*i2c interface*/	
	{"scaler_dp703", },
	{"scaler_tv5735",},
	{"scaler_tv5930",},
	/*uart interface*/

	{},
};


struct scaler_flag {
	atomic_t active_flag;
	atomic_t connect_flag;	
	atomic_t irq_enable_flag;
	atomic_t suspend_flag;
	atomic_t debug_flag;
};


struct scaler_platform_data {	
	//if null then dectect device auto
	enum scaler_id select_chip_id;
	enum scaler_bus_type bus_type;
	enum scaler_input_type input_format;	
	enum scaler_output_type output_format;
	
	int (*init_platform_hw)(struct scaler_private_data *scaler, int enable);
	int (*switch_ctrl)(enum scaler_connect_status status, enum scaler_swtich_type type, void *param);
	int power_pin;
	int reset_pin;
	int irq_pin;
	int ap_wakeup_scaler;
	int ap_status_pin;
	int scaler_status_pin;
};


struct scaler_operate {
	char *name;	//scaler name can be null
	int scaler_id;	//scaler id the value must be one of enum scaler_id
	int bus_type;	// scaler bus the value must be one of enum scaler_bus_type
	
	int slave_addr;
	int reg_size;
	int id_reg;
	int id_data;
	
	int trig;//if 1:used board gpio define else used scaler driver
	int irq;
	
	int (*active)(struct scaler_private_data *scaler, int enable);
	int (*init)(struct scaler_private_data *scaler);
	int (*reset)(struct scaler_private_data *scaler, int reset);
	int (*irq_handle)(struct scaler_private_data *scaler);
	int (*ap_wake_scaler)(struct scaler_private_data *scaler, int wake);
	int (*scaler_status)(struct scaler_private_data *scaler, int *status);
	int (*ap_status)(struct scaler_private_data *scaler, int status);
	int (*firmware)(struct scaler_private_data *scaler);
	int (*suspend)(struct scaler_private_data *scaler);
	int (*resume)(struct scaler_private_data *scaler);
	
	char *misc_name;
	struct miscdevice *misc_dev;
};


struct scaler_private_data {
	struct mutex io_lock;
	struct device *dev;
	int (*read_dev)(struct scaler_private_data *scaler, unsigned short reg,
			int bytes, void *dest, int reg_size);
	int (*write_dev)(struct scaler_private_data *scaler, unsigned short reg,
			 int bytes, void *src, int reg_size);
	void *control_data;
	int irq;
	//struct i2c_client *client;	
	struct input_dev *input_dev;
	struct work_struct work;
	struct delayed_work delaywork;	/*report second event*/	

	struct mutex data_mutex;
	struct mutex scaler_lock;

	
	struct wake_lock irq_wake;
		
	int devid;
	struct scaler_flag flags;
	struct i2c_device_id *i2c_id;
	struct scaler_platform_data *pdata;
	struct scaler_operate *ops; 
	struct file_operations fops;
	struct miscdevice miscdev;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct 	early_suspend early_suspend;
#endif
};


#if 1
#define DBG_SCALER(x...) if(atomic_read(&scaler->flags.debug_flag) == 1) printk(x)
#else
#define DBG_SCALER(x...)
#endif


extern int scaler_device_init(struct scaler_private_data *scaler, int type, int irq);
extern void scaler_device_exit(struct scaler_private_data *scaler);
extern int scaler_register_slave(struct scaler_private_data *scaler,
			struct scaler_platform_data *slave_pdata,
			struct scaler_operate *(*get_scaler_ops)(void));
extern int scaler_unregister_slave(struct scaler_private_data *scaler,
			struct scaler_platform_data *slave_pdata,
			struct scaler_operate *(*get_scaler_ops)(void));
extern int scaler_reg_read(struct scaler_private_data *scaler, unsigned short reg);
extern int scaler_reg_write(struct scaler_private_data *scaler, unsigned short reg,
		     unsigned short val);
extern int scaler_bulk_read(struct scaler_private_data *scaler, unsigned short reg,
		     int count, unsigned char *buf);
extern int scaler_bulk_read_normal(struct scaler_private_data *scaler, int count, unsigned char *buf, int rate);
extern int scaler_bulk_write(struct scaler_private_data *scaler, unsigned short reg,
		     int count, unsigned char *buf);
extern int scaler_bulk_write_normal(struct scaler_private_data *scaler, int count, unsigned char *buf, int rate);
extern int scaler_set_bits(struct scaler_private_data *scaler, unsigned short reg,
		    unsigned short mask, unsigned short val);
extern int scaler_device_suspend(struct scaler_private_data *scaler);

extern int scaler_device_resume(struct scaler_private_data *scaler);


#endif

