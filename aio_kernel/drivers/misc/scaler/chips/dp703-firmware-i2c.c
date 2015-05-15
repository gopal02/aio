#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/miscdevice.h>
#include <asm/dma.h>
#include <linux/preempt.h>
#include <linux/spi/spi.h>
#include <mach/board.h>
#include <linux/earlysuspend.h>
#include <linux/scaler-auto.h>
#include <mach/iomux.h>
#include "dp703-bin.h"

#if defined(CONFIG_SCALER_DP703_FIRMWARE_I2C)
#if 1
#define PAGE2_ADDRESS		(0x14 >> 1)
#define PAGE7_ADDRESS		(0x1E >> 1)
#else
#define PAGE2_ADDRESS		(0x94 >> 1)
#define PAGE7_ADDRESS		(0x9E >> 1)
#endif

static unsigned char binary_data_dp703_read[DP703_BIN0_SIZE];

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(3 * HZ)	/* M25P16 specs 40s max chip erase */



static int WriteReg(struct scaler_private_data *scaler, int address, int offset, char data)
{
	int result = 0;
	
	scaler->ops->slave_addr = address;
	result = scaler->write_dev(scaler, offset, 1, &data, scaler->ops->reg_size);
	if(result)
	{	
		printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
		return result;
	}

	return result;
}


static int ReadPage(struct scaler_private_data *scaler, int address, int offset, int bytes, char *buf)
{
	int result = 0;

	scaler->ops->slave_addr = address;
	result = scaler->read_dev(scaler, offset, bytes, buf, scaler->ops->reg_size);
	if(result)
	{	
		printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
		return result;
	}
	
	return result;
}


static unsigned char ReadReg(struct scaler_private_data *scaler, int address, int offset)
{
	int result = 0;
	unsigned char buf = 0;

	scaler->ops->slave_addr = address;
	result = scaler->read_dev(scaler, offset, 1, &buf, scaler->ops->reg_size);
	if(result)
	{	
		printk("%s:line=%d,error,result=%d\n",__func__,__LINE__,result);
		return -1;
	}
	
	return buf;
}


/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct scaler_private_data *scaler, int address, int offset, u8 value)
{
	unsigned long deadline;
	int val;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		val = ReadReg(scaler, address, offset);
		if (!(val & value))
			return 0;
		
		cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	printk("%s:addr=0x%x, reg=0x%x, val_get=0x%x, val_need=0x%x\n", __func__, address, offset, val, value);

	return 1;
}




int _i2c_firmware_update_dp703(struct scaler_private_data *scaler)
{
	int i=0,j=0,k=0,m=0;
	int fail = 0;
	int result = 0;
	int err_count = 0;
	
	//Step 1: Stop MPU and Reset SPI
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xbc, 0xc0);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xbc, 0x40); //stop MPU
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03); //set WP pin high
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x04); // Write-Disable
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x05);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x02); // set WP pin low
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//Step 2: Chip Erase
	//enable write status register
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x50); // Enable-Write-Status-Register
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x05);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x02);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//disable all protection
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x01); //Write-Status-Register
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x00); //Status Register
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x01);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x05);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x02);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//wait for SPI module ready
	result = wait_till_ready(scaler, PAGE2_ADDRESS, 0x9e, 0x0c);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
		
	for(i=0; i<3; i++)
	{
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x05); // 0x05 RDSR; command of flash
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00); // command length
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x01); // trigger read
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		
		result = wait_till_ready(scaler, PAGE2_ADDRESS, 0x93, 0x01);// wait for SPI command done
		if(!result)
		break;
	}

	if(i >= 3)
	return result;
	
	//enable DP701 mapping function
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xda, 0xaa);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xda, 0x55);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
		
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xda, 0x50);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xda, 0x41);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xda, 0x52);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xda, 0x44);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//write enable
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x06); // Write-Enable command
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x05);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x02);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//chip erase
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x60); //chip erase command
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x05);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//wait for SPI interface ready
	result = wait_till_ready(scaler, PAGE2_ADDRESS,0x9e, 0x0c);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//wait for SPI ROM until not busy
	for(i=0; i<3; i++)
	{
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x05); // 0x05 RDSR; Read-Status-Register
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00); // command length
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x01); // trigger read
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		result = wait_till_ready(scaler, PAGE2_ADDRESS, 0x93, 0x01);// wait for SPI command done
		if(!result)
		break;
	}

	if(i >= 3)
	return result;
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x02);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//Step 3: Load F/W to SPI ROM
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x82, 0x20);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	msleep(100);
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x82, 0x00);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	for(k=0;k<16;k++) //1Mbytes SPI ROM Size (16 banks)
	{
		for(j=0; j<256; j++)
		{
			result = WriteReg(scaler, PAGE2_ADDRESS, 0x8e, j);
			if(result)
			{
				printk("%s:line=%d, error\n",__func__, __LINE__);
				return result;

			}
			
			result = WriteReg(scaler, PAGE2_ADDRESS, 0x8f, k);
			if(result)
			{
				printk("%s:line=%d, error\n",__func__, __LINE__);
				return result;

			}
			
			for( i = 0; i <256; i++ )
			{
				result = WriteReg(scaler, PAGE7_ADDRESS, i, binary_data_dp703[(k<<16) + (j<<8) + i]); //
				if(result)
				{
					printk("%s:line=%d, error\n",__func__, __LINE__);
					return result;

				}
				// maybe should add some delay here
			}
		}
	}
	
	//Step 4: Verify SPI ROM
	fail=0;
	for(k=0;k<16;k++) //1Mbytes SPI ROM Size (16 banks)
	{
		for(j=0;j<256;j++) //read SPI ROM data to ReadHex
		{
			result = WriteReg(scaler, PAGE2_ADDRESS, 0x8e, j);
			if(result)
			{
				printk("%s:line=%d, error\n",__func__, __LINE__);
				return result;

			}
			
			result = WriteReg(scaler, PAGE2_ADDRESS, 0x8f, k);
			if(result)
			{
				printk("%s:line=%d, error\n",__func__, __LINE__);
				return result;

			}
			
			result = ReadPage(scaler, PAGE7_ADDRESS, 0, 256, &binary_data_dp703_read[(k<<16) + (j<<8)]);
			if(result)
			{
				printk("%s:line=%d, error\n",__func__, __LINE__);
				return result;

			}
		}

		err_count = 0;
		for(m=0; m<65536; m++)
		{ 
			if(binary_data_dp703_read[m + (k<<16)] != binary_data_dp703[m + (k<<16)])
			{
				err_count ++;	
				printk("%d:0x%x,0x%x ", m + (k<<16), binary_data_dp703[m + (k<<16)], binary_data_dp703_read[m + (k<<16)]);
				if((m%6) == 0)
				printk("\n");	
			}
		}

		if(err_count > 0)
		{
			printk("%s:err_count=%d\n",__func__, err_count);
			return -1;
		}
		
	}
	
	//Step 5: Enable Write Protection
	//write enable
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x50); // Enable-Write-Status-Register
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x05);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x02);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//enable write register protection
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x03);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x01); //write status register value
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x8C); //protect BPL/BP0/BP1
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x01);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x05);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xb0, 0x02);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
	
	//wait for SPI module ready
	result = wait_till_ready(scaler, PAGE2_ADDRESS, 0x9e, 0x0c);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}
		
	for(i=0; i<3; i++)
	{
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x90, 0x05); // 0x05 RDSR; command of flash
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x92, 0x00); // command length
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		
		result = WriteReg(scaler, PAGE2_ADDRESS, 0x93, 0x01); // trigger read
		if(result)
		{
			printk("%s:line=%d, error\n",__func__, __LINE__);
			return result;

		}
		result = wait_till_ready(scaler, PAGE2_ADDRESS, 0x93, 0x01);// wait for SPI command done
		if(!result)
		break;
	}	//disable DP701 mapping function
	
	if(i >= 3)
	return result;
	
	result = WriteReg(scaler, PAGE2_ADDRESS, 0xda, 0x00);
	if(result)
	{
		printk("%s:line=%d, error\n",__func__, __LINE__);
		return result;

	}

	return result;
}


int i2c_firmware_update_dp703(struct scaler_private_data *scaler)
{
	int i = 0;
	int result = 0;

	msleep(100);
	
	for(i=0; i<3; i++)
	{
		result = _i2c_firmware_update_dp703(scaler);
		if(!result)
			break;
		else
			msleep(1000);
	}

	if(i >= 3)
	printk("\n%s:fail to update dp703 firmware at last\n", __func__);
	else
	printk("\n%s:update dp703 firmware successfully\n", __func__);

	return result;
}


#endif

