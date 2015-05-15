//##########################################################################
//#                                                                        #
//#        Copyright (C) 2013 by Beijing IRTOUCHSYSTEMS Co., Ltd           #
//#                                                                        #
//#        IRTOUCH Multi I2C Touchscreen                                   #    
//#                                                                        #
//#        Create by Yin Xingjie                                           #
//#                                                                        #
//#        Email: yinxj@unitop.cc                                          #
//#                                                                        #
//##########################################################################

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/uaccess.h> 
#include <linux/hrtimer.h>
#include <linux/poll.h>

#define DRIVER_VERSION "v14.2.0.0006 for android"
#define DRIVER_AUTHOR "Beijing IRTOUCH SYSTEMS Co.,Ltd"
#define DRIVER_DESC "IRTOUCH I2C touchscreen driver for multi point"
#define DRIVER_LICENSE "GPL"
#define DRIVER_NAME "irtouchi2c_mt"
#define TOUCH_DEVICE_NAME "irtouch"

#define SET_COMMAND 0x01
#define GET_COMMAND 0x02

#define SET_COMMAND_MODE 0x71
#define SET_WORK_MODE	0x93
#define	GET_CALIB_X	0x2D
#define	SET_CALIB_X	0x2E
#define	GET_CALIB_Y	0x2B
#define	SET_CALIB_Y	0x2C
#define	GET_DEVICE_CONFIG	0x30
#define	SET_DEVICE_CONFIG	0x31
#define GET_FIRMWARE_VER 0x32
#define GET_BIOS_VER	0x07
#define	GET_DEVICE_SN	0x34
#define	GET_DEVICE_BARCODE	0x17

#define COMMAND_RETRY_COUNT  5

#define MAJOR_DEVICE_NUMBER 47
#define MINOR_DEVICE_NUMBER 194
#define TOUCH_POINT_COUNT  2
#define USB_TRANSFER_LENGTH			2330

//Control code for firmware update
#define CTLCODE_GET_FIRMWARE_VERSION 0xD0
#define CTLCODE_GET_SERIAL_NUMBER 0xD1
#define CTLCODE_SND_MSG 0xD2
#define CTLCODE_RCV_MSG 0xD3
#define CTLCODE_GET_BIOS_VERSION 0xD4
#define CTLCODE_GET_DRIVER_VERSION 0xD5
#define CTLCODE_SET_UPDATE_STATUE 0xD6
#define CTLCODE_GET_RAW_DATA 0xD7

#define CTLCODE_GET_COORDINATE 0xc0
#define CTLCODE_SET_CALIB_PARA_X 0xc1
#define CTLCODE_SET_CALIB_PARA_Y 0xc2
#define CTLCODE_GET_CALIB_PARA_X 0xc3
#define CTLCODE_GET_CALIB_PARA_Y 0xc4
#define CTLCODE_START_CALIB 0xc5
#define CTLCODE_GET_CALIB_STATUS 0xc6
#define CTLCODE_DEVICE_RESET 0xc7
#define CTLCODE_GET_SCR_PARA 0xc8
#define CTLCODE_GET_PRODUCT_SN 0xc9
#define CTLCODE_GET_PRODUCT_ID 0xca
#define CTLCODE_SET_SCAN_MODE 0xcb
#define CTLCODE_GET_PRODUCT_BAR 0xcc
#define CTLCODE_SET_SERVICE_STATUE 0xcd

#define err(format, arg...)					\
	printk(KERN_ERR KBUILD_MODNAME ": " format "\n", ##arg)

#pragma pack(1)
struct irtouch_point
{
	unsigned char	PointId;
	unsigned char	status;
	short	x;
	short	y;
};

struct irtouch_pkt
{	
	union
	{
		struct
		{
			unsigned char startBit;
			unsigned char commandType;
			unsigned char command;
			unsigned char pkgLength;
			unsigned char deviceId;
			union
			{
				struct
				{
					struct
					{
						unsigned char  ID;
						short X;
						short Y;
						short Z;
						short Width;
						short Height;
						unsigned char  Status;
						unsigned char  Reserved;
					}points[0x03];
					struct
					{
						unsigned char PointCount:5;
						unsigned char PointIndex:3;
					}PacketStatus;
				}IRPacket;
				unsigned char CRC;
			}DataPacket;
		}Packets;
		unsigned char RawInput[USB_TRANSFER_LENGTH];
	};
};

struct points
{
	int	x;
	int	y;
};

struct calib_param{
	long	A00;
	long	A01;
	long	A10;
	unsigned char reserve[46];
};

struct	device_config{
	unsigned char deviceID;
	unsigned char monitorID;
	unsigned char reserved[8];
	unsigned char calibrateStatus;
	unsigned char reserved1[47];
};

struct scr_props
{
	char DefaultOrigin	: 4;
	char PhysicalOrigin	: 4;
    unsigned char s1;		//红外未使用
    int width;				//unit:mm/s
    int height;				//unit:mm/s
 	short lightNo_h;		//横轴灯数
	short lightNo_v;		//纵轴灯数
	unsigned char maxcount;	//支持最大点数
	unsigned char ScanMethod;	//扫描方式 如1对3、1对5等
	unsigned char ScanInterv;	//发射接收最大偏灯数
	union
	{
		unsigned char nCustomized;	//特定版本号
		struct
		{
			unsigned char flags	: 2;	//特定版
			unsigned char Size	: 3;	//屏尺寸类型
			unsigned char Style	: 3;	//直灯/偏灯
		}TypeStatus;
	};

    unsigned short dHLInvalidInterv;	//X方向左边框附近无效区域（mm）(0.00~127.99)
	unsigned short dHRInvalidInterv;	//X方向右边框附近无效区域（mm）(0.00~127.99)
	unsigned short dVTInvalidInterv;	//V方向上边框附近无效区域（mm）(0.00~127.99)
	unsigned short dVBInvalidInterv;	//V方向下边框附近无效区域（mm）(0.00~127.99)
    unsigned short dHLightInterv;	//H灯间距 (0.00~127.99)
	unsigned short dVLightInterv;	//V灯间距 (0.00~127.99)
	unsigned short dMargin;	//边框特殊处理区域大小	(0.00~127.99)
	unsigned short dOffset;	//外扩强度	(0.00~127.99)
	
	int reserve1;
    int reserve2;
    int reserve3;
    int reserve4;

    float reserve5;
    float reserve6;
};

struct product_descriptor	//RKA100W021S
{
	unsigned char 	ProductType;
	unsigned char 	ProductSeries;
	unsigned char 	ProductReplace[4];
	unsigned char 	ScreenType;
	unsigned char 	size[3];
	unsigned char 	standardType;
	unsigned char	Reserved[9];
};

struct AlgContext
{
	int	actualCounter;
	struct irtouch_point inPoint[TOUCH_POINT_COUNT];
};

struct scan_range
{
    unsigned char Status;
    unsigned short   Start;
    unsigned short   End;
};

struct irscan_mode
{
    unsigned char Enable_Switch;   //0:stop 1:direct 2:Oblique 3:direct and Oblique
    unsigned char Scan_Mode;   //0:all 1:trace
    unsigned char Being_Num;   //0:1on1 1:1on3 2:1on5
    unsigned char Offset; 
    unsigned char Track_Width;
    unsigned char DataType;
    struct scan_range HRange[2];
    struct scan_range VRange[2];
    unsigned char ScanOffset;  //0:nothing 1:ScanInterv
    unsigned char reserve;
};
#pragma pack()

struct irtouch_mt_data {
	struct input_dev	*input_dev;
	struct i2c_client	*i2c_client;
	struct hrtimer timer;
	struct work_struct  work;
	wait_queue_head_t queue_wait;
	struct mutex		irtouch_mutex;
	int read_byte;
	int read_cnt;
	int write_cnt;
	int mask_cnt;
	unsigned char inbuf[USB_TRANSFER_LENGTH];
	unsigned char bufflag[10];
	unsigned char		ctrl_addr;
	unsigned char		status_addr;
	unsigned char		data_addr;
	unsigned char		id_addr;
};

struct device_context{
	bool startCalib;
	u16  productid;
	int  datalen;
	struct points points;
	struct AlgContext algCtx;
	struct calib_param calibX;
	struct calib_param calibY;
	struct device_config devConfig;
	struct scr_props     ScreenProps;
	struct product_descriptor	ProductSN;
	char firmwareVer[10];
	char	BarCodeInfo[40];
	char biosVer[10];
	struct irtouch_mt_data *irtouch;
};

struct cdev irtouch_cdev;
struct device_context *irtouchdev_context = NULL;
static struct class *irser_class;
static struct file_operations irtouch_fops;
static bool update_flag = false;
static bool service_flag = false;
static struct workqueue_struct *irtouch_wq;


static int irtouch_mt_i2c_byte_write(struct i2c_client *client,
						unsigned char address,
						unsigned char *dest, int count)
{
	unsigned char txbuf[65];
	int ret = 0, i;

	txbuf[0]	= address;
	for(i=0; i<count; i++)
		txbuf[i+1]	= dest[i];
	
	ret	= i2c_master_send(client, txbuf, count+1);
	return ret;
}

static int irtouch_mt_i2c_byte_read(struct i2c_client *client,
				     unsigned char address,
					 unsigned char *dest, int count)
{
	int ret = 0;
	//int framesum = 0;
	//unsigned char length = 16;
	//int i, j, bufindex = 0;
	
	//framesum = count / 16;
	//if((count % 16) != 0)
	//	framesum++;

	//if(count < length)
	//	length = count;
	
	ret = i2c_master_send(client, &address, 1);
	if (ret < 0)
		return ret;
		
	ret = i2c_master_recv(client, dest, count);

/*	for(i=0; i<framesum; i++)
	{
		for(j=0; j<length; j++)
			printk("0x%02x ", dest[j+bufindex]);
		printk("\n");
		
		bufindex += 16;
		if((count - bufindex) < 16)
			length = count - bufindex;
	}*/

	return ret;
}

static void irtouch_translatePoint(struct irtouch_point pt, int *x, int *y)
{
	*x = ((irtouchdev_context->calibX.A01 * pt.y) / 10000) + ((irtouchdev_context->calibX.A10 * pt.x) / 10000) + irtouchdev_context->calibX.A00;
	*y = ((irtouchdev_context->calibY.A01 * pt.y) / 10000) + ((irtouchdev_context->calibY.A10 * pt.x) / 10000) + irtouchdev_context->calibY.A00;
}

static int irtouch_build_packet(
		unsigned char 	command_type, 
		unsigned char 	command, 
		int 		device_id, 
		unsigned char * data, 
		int 		length, 
		unsigned char * out_data
		)
{
	unsigned char 	tmp = 0x55;
	unsigned char 	package[64] = {0};
	int		out_length = 0;
	int i;

	package[0] = 0xaa;
	package[1] = command_type;
	package[2] = command;
	package[3] = (unsigned char)length + 1;
	package[4] = (unsigned char)device_id;

	memcpy(&package[5],data,length);

	for(i = 0; i < 5 + length; i++)
	{
		tmp = package[i] + tmp;
	}

	package[5 + length] = (unsigned char)tmp;

	out_length = 6 + length;

	memcpy(out_data, package, out_length);

	return out_length;
}

unsigned char irtouch_send_command(struct irtouch_mt_data *irtouch_data,
			unsigned char 	command_type,
			unsigned char * in_data,
			unsigned char * out_data,
			int 		length
		)
{
	char	buf[64];
	int		count = 0;
	int		ret = 0;
	unsigned char	result = 0x00;

	do{
		memset(buf,0,sizeof(buf));
		ret = irtouch_mt_i2c_byte_write(irtouch_data->i2c_client, 
					irtouch_data->ctrl_addr, 
					(char *)in_data,
					length);
		msleep(200);

		ret = irtouch_mt_i2c_byte_read(irtouch_data->i2c_client, 
					irtouch_data->ctrl_addr, 
					(char *)buf, 
					64);
		msleep(200);

		count++;
		result = (buf[1] >> 4) & 0x03;
	}while((0x01 != result) && (count < COMMAND_RETRY_COUNT));

	if(count >= COMMAND_RETRY_COUNT)
	{
		return false;
	}

	memcpy(out_data,buf,sizeof(buf));

	return true;
	
}

static void irtouch_set_scanmode(struct irtouch_mt_data *irtouch_data, struct irscan_mode *scanmode)
{
	int	ret = 0;
	unsigned char inBuffer[64] = { 0 };
	unsigned char inBufferLen = 0;

	inBufferLen = irtouch_build_packet(SET_COMMAND, 0x39, 0, (unsigned char *)scanmode, sizeof(struct irscan_mode), inBuffer);
	ret = irtouch_mt_i2c_byte_write(irtouch_data->i2c_client, 
					irtouch_data->ctrl_addr, 
					(char *)inBuffer,
					inBufferLen);
}

static void irtouch_get_device_param(struct irtouch_mt_data *irtouch_data)
{
	unsigned char	inData[58];
	unsigned char	outData[64];
	unsigned char	getData[64];
	int status = -1;
	int length = 0;

	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	//set touchscreen to command mode
	inData[0] = 0x01;

	length = irtouch_build_packet(SET_COMMAND,SET_COMMAND_MODE,0,inData,1,outData);
	status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Set command mode failed.",__func__);
	}
	
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	inData[0] = 0xF4;

	length = irtouch_build_packet(SET_COMMAND,SET_WORK_MODE,0,inData,1,outData);
	status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Set device to work status failed.",__func__);
	}

	//get device configuration
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_DEVICE_CONFIG,0,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Get device configuration failed.",__func__);
	}
	memcpy(&irtouchdev_context->ScreenProps,&getData[5],sizeof(struct scr_props));
	printk("IRTOUCH maxcount=%d!\n", irtouchdev_context->ScreenProps.maxcount);
	if(0 == irtouchdev_context->ScreenProps.maxcount)
		irtouchdev_context->ScreenProps.maxcount = TOUCH_POINT_COUNT;

	irtouchdev_context->datalen = 2 * (irtouchdev_context->ScreenProps.lightNo_h + irtouchdev_context->ScreenProps.lightNo_v);
	if(irtouchdev_context->datalen == 0)
		irtouchdev_context->datalen = USB_TRANSFER_LENGTH;
	printk("IRTOUCH datalen=%d\n", irtouchdev_context->datalen);
	
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_DEVICE_CONFIG,1,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Get device configuration failed.",__func__);
	}
	memcpy(&irtouchdev_context->devConfig,&getData[5],sizeof(struct device_config));

	//get calib x parameter
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_CALIB_X,0,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Get calib X param failed.",__func__);
	}
	memcpy(&irtouchdev_context->calibX,&getData[5],sizeof(struct calib_param));

	//get calib y parameter
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_CALIB_Y,0,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Get calib Y param failed.",__func__);
	}
	memcpy(&irtouchdev_context->calibY,&getData[5],sizeof(struct calib_param));

	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_DEVICE_SN,0,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Get device ProductSN failed.",__func__);
	}
	memcpy(&irtouchdev_context->ProductSN,&getData[5],sizeof(struct product_descriptor));
	
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_FIRMWARE_VER,0,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(status)
	{
		memset(irtouchdev_context->firmwareVer,0,sizeof(irtouchdev_context->firmwareVer));		
		if(getData[3] == 3)
		{
			sprintf(irtouchdev_context->firmwareVer, "%d.%d",getData[6],getData[5]);
		}
		else if(getData[3] == 5)
		{
			sprintf(irtouchdev_context->firmwareVer, "%d.%d.%04d",getData[6],getData[5],(getData[8]<<8|getData[7]));
		}
	}
	else
	{
		err("IRTOUCH:    %s  get firmware version failed.",__func__);
	}
	printk("IRTOUCH MCU version is: %s.\n", irtouchdev_context->firmwareVer);
	
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_DEVICE_BARCODE,0,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Get device BARCODE failed.",__func__);
	}
	memcpy(irtouchdev_context->BarCodeInfo,&getData[5],40);

	//get BIOS version
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(GET_COMMAND,GET_BIOS_VER,0,inData,0,outData);
	status = irtouch_send_command(irtouch_data,GET_COMMAND,outData,getData,length);
	if(status)
	{
		sprintf(irtouchdev_context->biosVer,"%d.%d.%02d%02d",getData[6],getData[5],getData[8],getData[7]);
	}
	else
	{
		err("IRTOUCH:    %s  get BIOS version failed.",__func__);
	}
	
	//set touchscreen to work mode
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	length = irtouch_build_packet(SET_COMMAND,SET_COMMAND_MODE,0,inData,1,outData);
	status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Set to work mode failed.",__func__);
	}
}

static void irtouch_set_calib(struct irtouch_mt_data *irtouch_data, bool resetFlg)
{
	unsigned char	inData[58];
	unsigned char	outData[64];
	unsigned char	getData[64];
	int status = -1;
	int length = 0;

	//set touchscreen to command mode
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));
	memset(getData,0,sizeof(getData));

	inData[0] = 0x01;

	length = irtouch_build_packet(SET_COMMAND,SET_COMMAND_MODE,0,inData,1,outData);
	status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Set to command mode failed.",__func__);
	}
	
	//Set device configuration
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));

	if(resetFlg)
	{
		irtouchdev_context->devConfig.calibrateStatus = 0;
		memcpy(&inData,&irtouchdev_context->devConfig,58);
	
		length = irtouch_build_packet(SET_COMMAND,SET_DEVICE_CONFIG,1,inData,58,outData);
		status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
		if(!status)
		{
			err("IRTOUCH:    %s  Set device configuration failed.",__func__);
		}
	}
	else
	{
		irtouchdev_context->devConfig.calibrateStatus = 1;
		memcpy(&inData,&irtouchdev_context->devConfig,58);
	
		length = irtouch_build_packet(SET_COMMAND,SET_DEVICE_CONFIG,1,inData,58,outData);
		status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
		if(!status)
		{
			err("IRTOUCH:    %s  Set device configuration failed.",__func__);
		}

		//Set calib X param
		memset(inData,0,sizeof(inData));
		memset(outData,0,sizeof(outData));

		memcpy(&inData,&irtouchdev_context->calibX,58);
		length = irtouch_build_packet(SET_COMMAND,SET_CALIB_X,0,inData,58,outData);
		status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
		if(!status)
		{
			err("IRTOUCH:    %s  Set calib X param failed.",__func__);
		}
		
		//Set calib Y param
		memset(inData,0,sizeof(inData));
		memset(outData,0,sizeof(outData));

		memcpy(&inData,&irtouchdev_context->calibY,58);
		length = irtouch_build_packet(SET_COMMAND,SET_CALIB_Y,0,inData,58,outData);
		status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
		if(!status)
		{
			err("IRTOUCH:    %s  Set calib Y param failed.",__func__);
		}
	}

	//set touchscreen to work mode
	memset(inData,0,sizeof(inData));
	memset(outData,0,sizeof(outData));

	length = irtouch_build_packet(SET_COMMAND,SET_COMMAND_MODE,0,inData,1,outData);	
	status = irtouch_send_command(irtouch_data,SET_COMMAND,outData,getData,length);
	if(!status)
	{
		err("IRTOUCH:    %s  Set to work mode failed.",__func__);
	}
}

static int irtouch_open(struct inode * inode, struct file * filp)
{
	struct irtouch_mt_data * irtouch;

	irtouch = irtouchdev_context->irtouch;
	if(!irtouch)
	{
		err("IRTOUCH:    %s touch is NULL!",__func__);
		return -1;
	}

	filp->private_data = irtouch;
	//printk("IRTOUCH:    %s success!",__func__);
	return 0;
}

static int irtouch_release(struct inode * inode, struct file * filp)
{
	return 0;
}

static int irtouch_read(struct file * filp, char * buffer, size_t count, loff_t * ppos)
{
	int ret = irtouchdev_context->datalen;
	struct irtouch_mt_data * irtouch;
	
	irtouch = filp->private_data;

	mutex_lock(&(irtouch->irtouch_mutex));
	
	if(irtouch->inbuf[0] == 0x05)
	{
		ret = -ETIMEDOUT;
		goto exit;
	}
	
	if(irtouch->bufflag[irtouch->read_cnt] == 1)
	{
		if(copy_to_user(buffer, irtouch->inbuf+(irtouch->read_cnt*233), irtouchdev_context->datalen))
		{
			err("IRTOUCH:  %s copy_to_user failed!",__func__);
			ret = -EFAULT;
		}
		irtouch->bufflag[irtouch->read_cnt] = 0;
		irtouch->read_cnt++;
		if(irtouch->read_cnt == 10)
			irtouch->read_cnt = 0;
	}
	else
		ret = -ETIMEDOUT;
exit:
	irtouch->read_byte = 0;
	mutex_unlock(&(irtouch->irtouch_mutex));
	
	return ret;	
}

static void irtouch_process_data(struct irtouch_mt_data *irtouch)
{	
	int i;
	unsigned int x = 0, y = 0;
	int count = 0;
	struct input_dev *dev = irtouch->input_dev;
	
	for(i=0;i<irtouchdev_context->ScreenProps.maxcount;i++)
	{
		if(irtouchdev_context->algCtx.inPoint[i].status == 0x01)
		{
			x = irtouchdev_context->algCtx.inPoint[i].x;
			y = irtouchdev_context->algCtx.inPoint[i].y;

			if(irtouchdev_context->startCalib)
			{
				irtouchdev_context->points.x = irtouchdev_context->algCtx.inPoint[0].x;
				irtouchdev_context->points.y = irtouchdev_context->algCtx.inPoint[0].y;
			}
				
			if(irtouchdev_context->devConfig.calibrateStatus && !irtouchdev_context->startCalib)
			{
				irtouch_translatePoint(irtouchdev_context->algCtx.inPoint[i],&x,&y);
			}
			
			//printk("status%d=%d, x=%d, y=%d!\n", i, irtouchdev_context->algCtx.inPoint[i].status, x, y);
			input_report_abs(dev,ABS_MT_TRACKING_ID,irtouchdev_context->algCtx.inPoint[i].PointId);
			input_report_abs(dev,ABS_MT_POSITION_X, x);
			input_report_abs(dev,ABS_MT_POSITION_Y, y);
			input_report_abs(dev,ABS_MT_PRESSURE,255);
			input_report_key(dev,BTN_TOUCH,1);
			input_mt_sync(dev);
			count++;
		}
	}

	/* SYN_MT_REPORT only if no contact */
	if (!count)
		input_mt_sync(dev);
	
	input_sync(dev);
}

static int irtouch_write(struct file * filp, const char * user_buffer, size_t count, loff_t * ppos)
{
	int ret = -1;
	struct irtouch_mt_data *irtouch;
	
	irtouch = filp->private_data;
	if(copy_from_user(irtouchdev_context->algCtx.inPoint, user_buffer, sizeof(struct irtouch_point)*irtouchdev_context->ScreenProps.maxcount))
	{
		err("IRTOUCH: copy_from_user failed!\n");
		ret = -EFAULT;
	}
	else
	{
		ret = count;
		irtouch_process_data(irtouch);
	}
	//printk("IRTOUCH: irtouch_write,ret=%d!\n", ret);
	return ret;	
}

static long irtouch_ioctl(struct file * filp, unsigned int ctl_code, unsigned long ctl_param)
{
	unsigned char buf[64];
	unsigned char value = 0;
	int ret = -1;
	struct irtouch_mt_data *irtouch;

	irtouch = filp->private_data;

	switch(ctl_code)
	{
		case CTLCODE_START_CALIB:				
			ret = copy_from_user(&value, (unsigned char *)ctl_param, sizeof(unsigned char));
			if(ret == 0)
			{
				if(value == 0x01)
				{
					irtouchdev_context->startCalib = true;
				}
				else
				{
					irtouchdev_context->startCalib = false;
				}
			}	
			
			break;

		case CTLCODE_GET_COORDINATE:
			ret = copy_to_user((struct points *)ctl_param, &irtouchdev_context->points, sizeof(struct points));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_COORDINATE>copy_to_user failed!",__func__);
			}
			break;

		case CTLCODE_GET_CALIB_STATUS:
			{
				int status = irtouchdev_context->devConfig.calibrateStatus;
				ret = copy_to_user((int *)ctl_param, &status, sizeof(int));
				if(ret != 0)
				{
					err("IRTOUCH:    %s <CTLCODE_GET_CALIB_STATUS>copy_to_user failed!",__func__);
				}
				break;
			}
	
		case CTLCODE_SET_CALIB_PARA_X:
			irtouchdev_context->devConfig.calibrateStatus = 1;
			ret = copy_from_user(&irtouchdev_context->calibX, (struct calib_param *)ctl_param, sizeof(struct calib_param));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_SET_CALIB_PARA_X>copy_to_user failed!",__func__);
			}
			break;

		case CTLCODE_SET_CALIB_PARA_Y:
			irtouchdev_context->devConfig.calibrateStatus = 1;
			ret = copy_from_user(&irtouchdev_context->calibY, (struct calib_param *)ctl_param, sizeof(struct calib_param));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_SET_CALIB_PARA_Y>copy_to_user failed!",__func__);
			}
			break;

		case CTLCODE_GET_CALIB_PARA_X:
			ret = copy_to_user((struct calib_param *)ctl_param, &irtouchdev_context->calibX, sizeof(struct calib_param));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_CALIB_PARA>copy_to_user failed!",__func__);
			}
			break;

		case CTLCODE_GET_CALIB_PARA_Y:
			ret = copy_to_user((struct calib_param *)ctl_param, &irtouchdev_context->calibY, sizeof(struct calib_param));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_CALIB_PARA>copy_to_user failed!",__func__);
			}
			break;

		case CTLCODE_GET_SCR_PARA:
			ret = copy_to_user((struct scr_props *)ctl_param, &irtouchdev_context->ScreenProps, sizeof(struct scr_props));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_SCR_PARA>copy_to_user failed!",__func__);
			}
			break;
			
		case CTLCODE_GET_PRODUCT_SN:
			ret = copy_to_user((struct product_descriptor *)ctl_param, &irtouchdev_context->ProductSN, sizeof(struct product_descriptor));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_PRODUCT_SN>copy_to_user failed!",__func__);
			}
			break;

		case CTLCODE_GET_PRODUCT_BAR:
			ret = copy_to_user((unsigned char *)ctl_param, irtouchdev_context->BarCodeInfo, 40);
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_PRODUCT_BAR>copy_to_user failed!",__func__);
			}
			break;

		case CTLCODE_SET_UPDATE_STATUE:
			ret = copy_from_user(&value, (unsigned char *)ctl_param, sizeof(unsigned char));
			if(ret == 0)
			{
				if(value == 0x01)
				{
					update_flag = true;
				}
				else
				{
					update_flag = false;
				}
			}	
			break;

		case CTLCODE_SET_SERVICE_STATUE:
			ret = copy_from_user(&value, (unsigned char *)ctl_param, sizeof(unsigned char));
			if(ret == 0)
			{
				if(value == 0x01)
				{
					service_flag = true;
				}
				else
				{
					service_flag = false;
				}
			}	
			break;
			
		case CTLCODE_SET_SCAN_MODE:
			{
				struct irscan_mode scanmode = {0};
				ret = copy_from_user(&scanmode, (unsigned char *)ctl_param, sizeof(struct irscan_mode));
				if(ret != 0)
				{
					err("IRTOUCH:    %s <CTLCODE_GET_CALIB_STATUS>copy_to_user failed!",__func__);
				}
				else
				{
					irtouch_set_scanmode(irtouch, &scanmode);
				}
				break;
			}
			
		case CTLCODE_GET_PRODUCT_ID:
			{
				int status = irtouchdev_context->productid;
				ret = copy_to_user((int *)ctl_param, &status, sizeof(int));
				if(ret != 0)
				{
					err("IRTOUCH:    %s <CTLCODE_GET_PRODUCT_ID>copy_to_user failed!",__func__);
				}
				break;
			}
		
		case CTLCODE_DEVICE_RESET:
			ret = copy_from_user(&value, (unsigned char *)ctl_param, sizeof(unsigned char));
			if(ret == 0)
			{
				if(value == 0x01)
				{
					irtouchdev_context->devConfig.calibrateStatus = 0;
					//irtouch_set_calib(irtouch,true);
				}
				else
				{
					//irtouch_set_calib(irtouch,false);
				}
			}
			break;

		case CTLCODE_GET_FIRMWARE_VERSION:
			ret = copy_to_user((char*)ctl_param, irtouchdev_context->firmwareVer, sizeof(irtouchdev_context->firmwareVer));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_FIRMWARE_VERSION>copy_to_user failed!",__func__);
			}
			break;	

		case CTLCODE_GET_SERIAL_NUMBER:
			ret = copy_to_user((char*)ctl_param, irtouchdev_context->BarCodeInfo, sizeof(irtouchdev_context->BarCodeInfo));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_PRODUCT_SN>copy_to_user failed!",__func__);
			}
			break;	

		case CTLCODE_GET_BIOS_VERSION:
			ret = copy_to_user((char*)ctl_param, irtouchdev_context->biosVer, sizeof(irtouchdev_context->biosVer));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_BIOS_VERSION>copy_to_user failed!",__func__);
			}
			break;	

		case CTLCODE_GET_DRIVER_VERSION:
			ret = copy_to_user((char*)ctl_param, DRIVER_VERSION, sizeof(DRIVER_VERSION));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_DRIVER_VERSION>copy_to_user failed!",__func__);
			}
			break;	

		case CTLCODE_GET_RAW_DATA:
			mutex_lock(&(irtouch->irtouch_mutex));
			ret = copy_to_user((char*)ctl_param, irtouch->inbuf+(irtouch->mask_cnt*233), irtouchdev_context->datalen);
			mutex_unlock(&(irtouch->irtouch_mutex));
			if(ret != 0)
			{
				err("IRTOUCH:    %s <CTLCODE_GET_RAW_DATA>copy_to_user failed!",__func__);
			}
			irtouch->mask_cnt++;
			if(irtouch->mask_cnt == 10)
				irtouch->mask_cnt = 0;
			break;
			
		case CTLCODE_SND_MSG:
			ret = copy_from_user((unsigned char *)buf, (unsigned char *)ctl_param, sizeof(buf));
			if(ret == 0)
			{
				ret = irtouch_mt_i2c_byte_write(irtouch->i2c_client, 
					irtouch->ctrl_addr, 
					buf,
					buf[3]+5);	
				//printk("send:buf0=0x%x,buf1=0x%x,buf2=0x%x,buf3=0x%x,buf4=0x%x!\n", buf[0],buf[1],buf[2],buf[3],buf[4]);
			}	
			break;

		case CTLCODE_RCV_MSG:
			ret = irtouch_mt_i2c_byte_read(irtouch->i2c_client, 
					irtouch->ctrl_addr, 
					buf, 
					64);
			if(ret > 0)
			{	
				ret = copy_to_user((unsigned char *)ctl_param, buf, buf[3]+5);
				//printk("rev:buf0=0x%x,buf1=0x%x,buf2=0x%x,buf3=0x%x,buf4=0x%x!\n", buf[0],buf[1],buf[2],buf[3],buf[4]);
			}
			break;
			
		default:
			break;
	}

	return 0;
}

static unsigned int irtouch_poll(struct file *filp, poll_table *wait)
{
	struct irtouch_mt_data *irtouch;

	irtouch = filp->private_data;
	
	poll_wait(filp, &irtouch->queue_wait, wait);

	if (irtouch->read_byte == 233)
		return POLLIN | POLLRDNORM;
	
	return 0;
}

static struct file_operations irtouch_fops = {
	.owner = THIS_MODULE,
	.open = irtouch_open,
	.read = irtouch_read,
	.poll = irtouch_poll,
	.write = irtouch_write,
	.unlocked_ioctl = irtouch_ioctl,
	.release = irtouch_release,
};

static bool irtouch_mkdev(void)
{
	int retval;

	//create device node
	dev_t devno = MKDEV(MAJOR_DEVICE_NUMBER,MINOR_DEVICE_NUMBER);

	retval = register_chrdev_region(devno,1,TOUCH_DEVICE_NAME);
	if(retval < 0)
	{
		err("IRTOUCH:    %s register chrdev error.",__func__);
		return false;
	}

	cdev_init(&irtouch_cdev,&irtouch_fops);
	irtouch_cdev.owner = THIS_MODULE;
	irtouch_cdev.ops = &irtouch_fops;
	retval = cdev_add(&irtouch_cdev,devno,1);

	if(retval)
	{
		err("IRTOUCH:    %s  Adding char_reg_setup_cdev error=%d",__func__,retval);
		return false;
	}

	irser_class = class_create(THIS_MODULE, TOUCH_DEVICE_NAME);
	if(IS_ERR(irser_class))
	{
		err("IRTOUCH:    %s class create failed.",__func__);
		return false;
	}
	
	device_create(irser_class,NULL,MKDEV(MAJOR_DEVICE_NUMBER,MINOR_DEVICE_NUMBER),NULL,TOUCH_DEVICE_NAME);
	
	printk("IRTOUCH:    %s success!\n",__func__);
	
	return true;
}

/*****************************************************************
输入参数说明：
int nSLightNumH：  水平灯数量
int nSLightNumV：  垂直灯数量
double offsetX：   x轴第一颗灯距原点的距离，单位毫米
double offsetY：   y轴第一颗灯距原点的距离，单位毫米
double distXLight：x轴灯间距，单位毫米
double distYLight：y轴灯间距，单位毫米
int marginX :		 水平方向的丝印
int marginY : 		垂直方向的丝印

int thresh：       单点触摸遮挡线数上限

buf默认输入读取顺序：
	屏幕左上角为原点，
	buf前nSLightNumV位表示沿垂直灯以原点为起点从上到下扫描，
	buf后nSLightNumH位表示沿水平灯以原点为起点从左到右扫描。
******************************************************************/
static void irtouch_touchdata(unsigned char *buf, int len, 
							  int nSLightNumH, int nSLightNumV, int offsetX, int offsetY, 
							  int distXLight, int distYLight, int marginX,int marginY,int thresh)
{
	int i;
	unsigned char data;
	int x = 0;
	int y = 0;
	int nx = 0;
	int ny = 0;
	long xS = 0;//offsetX;
	long yS = 0;//offsetY;
	short *tmpbuf = (short *)buf;

	int H_len,V_len;

	H_len=(nSLightNumH-1)*distXLight - 2*marginX;
	V_len=(nSLightNumV-1)*distYLight - 2*marginY;

	for(i=0;i<len;i++)
	{
		data = (tmpbuf[i] >> 8) & 0x01;
		if(data == 0) //根据掩码信息，主轴数据有效
		{
			if( i >= nSLightNumH )
			{
				ny++;
				y = i - nSLightNumH; 
				yS += y * distYLight;
			}
			else
			{
				nx++;
				x = i;
				xS += x * distXLight;
			}
		}
	}
	
	if( (nx> 0 && nx < thresh)&& (ny>0 &&  ny < thresh) )
	{
		irtouchdev_context->algCtx.inPoint[0].PointId = 0;
		irtouchdev_context->algCtx.inPoint[0].status = 1;
		irtouchdev_context->algCtx.inPoint[0].x =(xS/nx - marginX)*32767/H_len;
		irtouchdev_context->algCtx.inPoint[0].y =(yS/ny - marginY)*32767/V_len;

		if(irtouchdev_context->algCtx.inPoint[0].x<0 
			|| irtouchdev_context->algCtx.inPoint[0].x>32767
			|| irtouchdev_context->algCtx.inPoint[0].y<0
			|| irtouchdev_context->algCtx.inPoint[0].y>32767
			)
		{
			irtouchdev_context->algCtx.inPoint[0].status = 0;
		}
		
		irtouchdev_context->algCtx.inPoint[0].x =irtouchdev_context->algCtx.inPoint[0].x<0? 
			0:irtouchdev_context->algCtx.inPoint[0].x;
		irtouchdev_context->algCtx.inPoint[0].x =irtouchdev_context->algCtx.inPoint[0].x>32767? 
			32767:irtouchdev_context->algCtx.inPoint[0].x;

		irtouchdev_context->algCtx.inPoint[0].y =irtouchdev_context->algCtx.inPoint[0].y<0? 
			0:irtouchdev_context->algCtx.inPoint[0].y;
		irtouchdev_context->algCtx.inPoint[0].y =irtouchdev_context->algCtx.inPoint[0].y>32767? 
			32767:irtouchdev_context->algCtx.inPoint[0].y;		 
	}
	else
	{
		irtouchdev_context->algCtx.inPoint[0].PointId = 0;
		irtouchdev_context->algCtx.inPoint[0].status = 0;
		irtouchdev_context->algCtx.inPoint[0].x = 0;
		irtouchdev_context->algCtx.inPoint[0].y = 0;
	}
}

static void irtouch_datacheck(struct irtouch_mt_data *irtouch_data)
{
	int marginX,marginY;

	if(service_flag == false)
	{
		marginX =(70*635-436*100)/2;
		marginY =(44*635-238*100)/2;
		irtouch_touchdata(irtouch_data->inbuf+(irtouch_data->write_cnt*233), 116, 71, 45, 1295, 900, 635, 635, 
			marginX,marginY,10);
		irtouch_process_data(irtouch_data);
		irtouch_data->bufflag[irtouch_data->read_cnt] = 0;
		irtouch_data->read_cnt++;
		if(irtouch_data->read_cnt == 10)
			irtouch_data->read_cnt = 0;
	}

}

static void irtouch_work_func(struct work_struct *work)
{
	struct irtouch_mt_data *irtouch = container_of(work, struct irtouch_mt_data, work);
	unsigned char status = 0;
	
	mutex_lock(&(irtouch->irtouch_mutex));
	if(!update_flag)
	{	
		irtouch_mt_i2c_byte_read(irtouch->i2c_client,
				irtouch->status_addr,
				&status, 1);
		if((status == 0x10) && (irtouch->bufflag[irtouch->write_cnt] == 0)) 	
		{	
			irtouch->read_byte = irtouch_mt_i2c_byte_read(irtouch->i2c_client,
					irtouch->data_addr,
					irtouch->inbuf+(irtouch->write_cnt*233), 233);
			irtouch->bufflag[irtouch->write_cnt] = 1;
			irtouch_datacheck(irtouch);
			irtouch->write_cnt++;
			if(irtouch->write_cnt == 10)
				irtouch->write_cnt = 0;
			wake_up(&irtouch->queue_wait);
		}
	}
	mutex_unlock(&(irtouch->irtouch_mutex));
}

static enum hrtimer_restart irtouch_timer_func(struct hrtimer *timer)
{
	struct irtouch_mt_data *irtouch = container_of(timer, struct irtouch_mt_data, timer);
	
	queue_work(irtouch_wq, &irtouch->work);
	hrtimer_start(&irtouch->timer, ktime_set(0, 1000000), HRTIMER_MODE_REL);//5000000
	
	return HRTIMER_NORESTART;
}

static int __devinit irtouch_mt_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
	int retval = -ENOMEM;
	struct irtouch_mt_data *irtouch_data;
	char txbuf[4] = {0};
	
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c smbus byte data not supported\n");
		return -EIO;
	}

	if (!(irtouchdev_context = kmalloc(sizeof(struct device_context), GFP_KERNEL)))  
		return -ENOMEM;
	memset(irtouchdev_context,0,sizeof(struct device_context));

	irtouch_data = kzalloc(sizeof(struct irtouch_mt_data), GFP_KERNEL);
	if(!irtouch_data)
	{
		err("IRTOUCH:    %s Out of memory.",__func__);
		goto EXIT;
	}
	memset(irtouch_data, 0, sizeof(struct irtouch_mt_data));
	irtouchdev_context->irtouch = irtouch_data;

	irtouch_data->id_addr = 0x00;
	irtouch_data->status_addr = 0x01;
	irtouch_data->ctrl_addr = 0x02;
	irtouch_data->data_addr = 0x03;
	
	irtouch_data->input_dev = input_allocate_device();
	if (irtouch_data->input_dev == NULL) {
		dev_err(&client->dev, "%s:input device alloc failed\n",
						__func__);
		retval = -ENOMEM;
		goto EXIT;
	}

	irtouch_data->i2c_client = client;

	retval = irtouch_mt_i2c_byte_read(client, irtouch_data->id_addr, txbuf, 4);
	if(retval < 0)
	{
		dev_err(&client->dev, "i2c read id failed, ret=%d\n", retval);
		goto EXIT;
	}
	
	irtouchdev_context->productid = (txbuf[0] << 8) | txbuf[1];
	sprintf(irtouchdev_context->firmwareVer, "%d.%d",txbuf[2],txbuf[3]);
	
	INIT_WORK(&irtouch_data->work, irtouch_work_func);
	init_waitqueue_head(&irtouch_data->queue_wait);
	mutex_init(&(irtouch_data->irtouch_mutex));
	
	/* Store the instance data in the i2c_client */
	i2c_set_clientdata(client, irtouch_data);
	
	/*initialize the input device parameters */
	irtouch_data->input_dev->name	= DRIVER_NAME;
	irtouch_data->input_dev->phys	= "Irtouch multitouch";
	irtouch_data->input_dev->id.bustype = BUS_I2C;
	irtouch_data->input_dev->id.vendor = 0x6615;
	irtouch_data->input_dev->id.product = irtouchdev_context->productid;
	irtouch_data->input_dev->dev.parent = &client->dev;
	input_set_drvdata(irtouch_data->input_dev, irtouch_data);

	irtouch_data->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	set_bit(BTN_TOUCH, irtouch_data->input_dev->keybit);
	irtouch_data->input_dev->absbit[0] = BIT(ABS_MT_POSITION_X) | BIT(ABS_MT_POSITION_Y);
	set_bit(ABS_MT_PRESSURE, irtouch_data->input_dev->absbit);

	input_set_abs_params(irtouch_data->input_dev, ABS_MT_PRESSURE,0,255,0,0);
	input_set_abs_params(irtouch_data->input_dev, ABS_MT_POSITION_X, 0, 32767, 0, 0);
	input_set_abs_params(irtouch_data->input_dev, ABS_MT_POSITION_Y, 0, 32767, 0, 0);
	input_set_abs_params(irtouch_data->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);

	retval = input_register_device(irtouch_data->input_dev);
	if (retval) {
		dev_err(&client->dev, "%s:input register failed\n", __func__);
		goto EXIT;
	}

	//msleep(5000);

	//	irtouch_get_device_param(irtouch_data);
	irtouchdev_context->ScreenProps.maxcount = 2;
	irtouchdev_context->ScreenProps.lightNo_h = 71;
	irtouchdev_context->ScreenProps.lightNo_v = 45;
	irtouchdev_context->ScreenProps.DefaultOrigin = 1;
	irtouchdev_context->ScreenProps.PhysicalOrigin = 1;
	irtouchdev_context->ScreenProps.ScanMethod = 0;
	irtouchdev_context->ScreenProps.ScanInterv = 0;
	irtouchdev_context->ScreenProps.dHLInvalidInterv = 0;
	irtouchdev_context->ScreenProps.dHRInvalidInterv = 0;
	irtouchdev_context->ScreenProps.dVTInvalidInterv = 0;
	irtouchdev_context->ScreenProps.dHLightInterv = 0;
	irtouchdev_context->ScreenProps.dVLightInterv = 0;
	irtouchdev_context->ScreenProps.dMargin = 0;
	irtouchdev_context->ScreenProps.dOffset = 0;
	irtouchdev_context->ScreenProps.TypeStatus.Size = 0;
	irtouchdev_context->ScreenProps.TypeStatus.Style = 0;
	irtouchdev_context->ScreenProps.width = 436;
	irtouchdev_context->ScreenProps.height = 238;

	irtouchdev_context->devConfig.calibrateStatus = 0;

	irtouchdev_context->ProductSN.size[0] = 0x30;
	irtouchdev_context->ProductSN.size[1] = 0x31;
	irtouchdev_context->ProductSN.size[2] = 0x39;
	
	irtouchdev_context->datalen = 232;
	
	irtouch_mkdev();
	printk("IRTOUCH id=0x%x: %s success!\n", irtouchdev_context->productid, __func__);

	hrtimer_init(&irtouch_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	irtouch_data->timer.function = irtouch_timer_func;
	hrtimer_start(&irtouch_data->timer, ktime_set(10, 0), HRTIMER_MODE_REL);
	
	return 0;

EXIT:
	if(irtouchdev_context)
		kfree(irtouchdev_context);
	
	if(irtouch_data->input_dev)
		input_free_device(irtouch_data->input_dev);

	if(irtouch_data)
		kfree(irtouch_data);

	return -retval;

}
/**
 * irtouch_mt_remove() - Removes the i2c-client touchscreen driver
 * @client: i2c client structure pointer
 *
 * This function uses to remove the i2c-client
 * touchscreen driver and returns integer.
 */
static int __devexit irtouch_mt_remove(struct i2c_client *client)
{
	struct irtouch_mt_data *irtouch_data = i2c_get_clientdata(client);

	dev_t devno = MKDEV(MAJOR_DEVICE_NUMBER,MINOR_DEVICE_NUMBER);

	cdev_del(&irtouch_cdev);
	unregister_chrdev_region(devno,1);

	device_destroy(irser_class,devno);
	class_destroy(irser_class);
	
	input_unregister_device(irtouch_data->input_dev);
	hrtimer_cancel(&irtouch_data->timer);
	kfree(irtouch_data);
	kfree(irtouchdev_context);
	
	return 0;
}

static const struct i2c_device_id irtouch_mt_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, irtouch_mt_id);

static struct i2c_driver irtouch_mt_driver = {
	.driver = {
		.name	=	DRIVER_NAME,
		.owner	=	THIS_MODULE,
	},
	.probe		=	irtouch_mt_probe,
	.remove		=	__devexit_p(irtouch_mt_remove),
	.id_table	=	irtouch_mt_id,
};
/**
 * irtouch_mt_init() - Initialize the touchscreen driver
 *
 * This function uses to initializes the synaptics
 * touchscreen driver and returns integer.
 */
static int __init irtouch_mt_init(void)
{
	irtouch_wq = create_singlethread_workqueue("irtouch_wq");
	if (!irtouch_wq)
		return -ENOMEM;
	return i2c_add_driver(&irtouch_mt_driver);
}
/**
 * irtouch_mt_exit() - De-initialize the touchscreen driver
 *
 * This function uses to de-initialize the synaptics
 * touchscreen driver and returns none.
 */
static void __exit irtouch_mt_exit(void)
{
	i2c_del_driver(&irtouch_mt_driver);
	if (irtouch_wq)
		destroy_workqueue(irtouch_wq);
}


module_init(irtouch_mt_init);
module_exit(irtouch_mt_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);
MODULE_VERSION(DRIVER_VERSION);

