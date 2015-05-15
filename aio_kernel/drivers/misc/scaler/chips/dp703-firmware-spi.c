/*drivers/misc/scaler/chips/dp703-firmware-spi.c -spi firmware driver
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

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
#include "../../../spi/rk29_spim.h"

/* Flash opcodes. */
#define	OPCODE_WREN		0x06	/* Write enable */
#define	OPCODE_RDSR		0x05	/* Read status register */
#define	OPCODE_WRSR		0x01	/* Write status register 1 byte */
#define	OPCODE_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	OPCODE_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	OPCODE_PP		0x02	/* Page program (up to 256 bytes) */
#define	OPCODE_BE_4K		0x20	/* Erase 4KiB block */
#define	OPCODE_BE_32K		0x52	/* Erase 32KiB block */
#define	OPCODE_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	OPCODE_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	OPCODE_RMID		0x90	/* Read MID*/
#define	OPCODE_RDID		0x9f	/* Read JEDEC ID */

/* Used for SST flashes only. */
#define	OPCODE_BP		0x02	/* Byte program */
#define	OPCODE_WRDI		0x04	/* Write disable */
#define	OPCODE_AAI_WP		0xad	/* Auto address increment word program */

/* Used for Macronix flashes only. */
#define	OPCODE_EN4B		0xb7	/* Enter 4-byte mode */
#define	OPCODE_EX4B		0xe9	/* Exit 4-byte mode */

/* Used for Spansion flashes only. */
#define	OPCODE_BRWR		0x17	/* Bank register write */

/* Status Register bits. */
#define	SR_WIP			1	/* Write in progress */
#define	SR_WEL			2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define	SR_BP0			4	/* Block protect 0 */
#define	SR_BP1			8	/* Block protect 1 */
#define	SR_BP2			0x10	/* Block protect 2 */
#define	SR_SRWD			0x80	/* SR write protect */

/* Define max times to check status register before we give up. */
#define	MAX_READY_WAIT_JIFFIES	(3 * HZ)	/* M25P16 specs 40s max chip erase */
#define	MAX_CMD_SIZE		5

#ifdef CONFIG_M25PXX_USE_FAST_READ
#define OPCODE_READ 	OPCODE_FAST_READ
#define FAST_READ_DUMMY_BYTE 1
#else
#define OPCODE_READ 	OPCODE_NORM_READ
#define FAST_READ_DUMMY_BYTE 0
#endif

#define JEDEC_MFR(_jedec_id)	((_jedec_id) >> 16)


#define MAX_SPI_BUS_NUM 2



struct spi_firmware_data {
	struct device	*dev;
	struct spi_device	*spi;	
	char *rx_buf;
	int rx_len; 
	char *tx_buf;
	int tx_len; 
};
static struct spi_firmware_data *g_spi_firmware_data[MAX_SPI_BUS_NUM];


static struct rk29xx_spi_chip spi_firmware_chip[] = {
{
	//.poll_mode = 1,
	.enable_dma = 0,
},
{
	//.poll_mode = 1,
	.enable_dma = 1,
},

};
	
static struct spi_board_info board_spi_firmware_devices[] = {	
#if defined(CONFIG_SPIM0_RK29)
	{
		.modalias  = "spi_firmware_bus0",
		.bus_num = 0,	//0 or 1
		.max_speed_hz  = 12*1000*1000,
		.chip_select   = 0,		
		.mode	= SPI_MODE_0,
		.controller_data = &spi_firmware_chip[0],
	},
#endif
#if 0
#if defined(CONFIG_SPIM1_RK29)
	{
		.modalias  = "spi_firmware_bus1",
		.bus_num = 1,	//0 or 1
		.max_speed_hz  = 12*1000*1000,
		.chip_select   = 0,		
		.mode	= SPI_MODE_0,
		.controller_data = &spi_firmware_chip[1],
	}
#endif
#endif
};

/*
 * Internal helper functions
 */

/*
 * Read the status register, returning its value in the location
 * Return the status register value.
 * Returns negative if error occurred.
 */
static int read_sr(struct spi_device *spi)
{
	ssize_t retval;
	u8 code = OPCODE_RDSR;
	u8 val;

	retval = spi_write_then_read(spi, &code, 1, &val, 1);

	if (retval < 0) {
		dev_err(&spi->dev, "error %d reading SR\n",
				(int) retval);
		return retval;
	}

	return val;
}

/*
 * Write status register 1 byte
 * Returns negative if error occurred.
 */
static int write_sr(struct spi_device *spi, u8 val)
{
	char command[2];
	
	command[0] = OPCODE_WRSR;
	command[1] = val;

	return spi_write(spi, command, 2);
}

/*
 * Set write enable latch with Write Enable command.
 * Returns negative if error occurred.
 */
static inline int write_enable(struct spi_device *spi)
{
	u8	code = OPCODE_WREN;
	int i = 0, result = 0;
	
	//enable
	for(i=0; i<5; i++)
	{
		result = spi_write_then_read(spi, &code, 1, NULL, 0);
		if(!result)
		break;
		else
		msleep(10);
	}

	if( i >= 5)
	{
		printk("%s:fail to enable spi flash,try %d times\n",__func__,i);
	}

	return result;
}

/*
 * Send write disble instruction to the chip.
 */
static inline int write_disable(struct spi_device *spi)
{
	u8	code = OPCODE_WRDI;

	return spi_write_then_read(spi, &code, 1, NULL, 0);
}

static int read_mid(struct spi_device *spi, u8 *buf)
{
	ssize_t retval;
	char command[4];
	
	command[0] = OPCODE_RMID;
	command[1] = 0x00;	
	command[2] = 0x00;	
	command[3] = 0x00;
	
	retval = spi_write_then_read(spi, command, 4, buf, 2);

	if (retval < 0) {
		dev_err(&spi->dev, "error %d reading MID\n",
				(int) retval);
		return retval;
	}

	return retval;
}


#define CFI_MFR_MACRONIX	0x00C2

/*
 * Enable/disable 4-byte addressing mode.
 */
static inline int set_4byte(struct spi_device *spi, u32 jedec_id, int enable)
{
	char command[2];
	
	switch (JEDEC_MFR(jedec_id)) {
	case CFI_MFR_MACRONIX:
		command[0] = enable ? OPCODE_EN4B : OPCODE_EX4B;
		return spi_write(spi, command, 1);
	default:
		/* Spansion style */
		command[0] = OPCODE_BRWR;
		command[1] = enable << 7;
		return spi_write(spi, command, 2);
	}
}

/*
 * Service routine to read status register until ready, or timeout occurs.
 * Returns non-zero if error.
 */
static int wait_till_ready(struct spi_device *spi)
{
	unsigned long deadline;
	int sr;

	deadline = jiffies + MAX_READY_WAIT_JIFFIES;

	do {
		if ((sr = read_sr(spi)) < 0)
			break;
		else if (!(sr & SR_WIP))
			return 0;

		cond_resched();

	} while (!time_after_eq(jiffies, deadline));

	printk("%s:sr=0x%x\n",__func__,sr);

	return 1;
}

/*
 * Erase the whole flash memory
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_chip(struct spi_device *spi)
{
	char command[1];
	int result = 0;

	/* Send write enable, then erase commands. */
	write_enable(spi);

	/* Set up command buffer. */
	command[0] = OPCODE_CHIP_ERASE;

	result = spi_write(spi, command, 1);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}


	/* Wait until finished previous write command. */
	if (wait_till_ready(spi))
		return 1;

	return 0;
}

static void m25p_addr2cmd(struct spi_device *spi, unsigned int addr, u8 *cmd)
{
	int addr_width = 3;
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (addr_width * 8 -  8);
	cmd[2] = addr >> (addr_width * 8 - 16);
	cmd[3] = addr >> (addr_width * 8 - 24);
	//cmd[4] = addr >> (addr_width * 8 - 32);
}

static int m25p_cmdsz(struct spi_device *spi)
{	
	int addr_width = 3;
	return 1 + addr_width;
}

/*
 * Erase one sector of flash memory at offset ``offset'' which is any
 * address within the sector which should be erased.
 *
 * Returns 0 if successful, non-zero otherwise.
 */
static int erase_sector(struct spi_device *spi, u32 offset, int erase_opcode)
{
	char command[1];
	int result = 0;
	
	/* Wait until finished previous write command. */
	if (wait_till_ready(spi))
		return 1;

	/* Send write enable, then erase commands. */
	write_enable(spi);

	/* Set up command buffer. */
	command[0] = erase_opcode;
	m25p_addr2cmd(spi, offset, command);

	result = spi_write(spi, command, m25p_cmdsz(spi));
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}
	return 0;
}


static int write_page(struct spi_device *spi, int addr, int bytes)
{
	int result = 0;
	char cmd[4] = {OPCODE_PP,1,1,1};
	char buf[4+256];
	
	write_enable(spi);
	
	//write page cmd
	buf[0] = OPCODE_PP;
	buf[1] = (u8)(addr >> 16);
	buf[2] = (u8)(addr >> 8);
	buf[3] = (u8)addr;

	memcpy(&buf[4], &binary_data_dp703[addr], bytes);

	result = spi_write(spi, buf, 4+bytes);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	if (wait_till_ready(spi))
		return 1;
	
	return result;
}

static int read_page(struct spi_device *spi, int addr, int bytes, char *buf)
{
	int result = 0;
	char cmd[4] = {OPCODE_NORM_READ,1,1,1};
	
	write_enable(spi);
	
	//read page cmd
	cmd[0] = OPCODE_NORM_READ;
	cmd[1] = (u8)(addr >> 16);
	cmd[2] = (u8)(addr >> 8);
	cmd[3] = (u8)addr;

	result = spi_write_then_read(spi, cmd, 4, buf, bytes);
	if(result)
	{
		printk("%s:line=%d,error\n",__func__,__LINE__);
		return result;
	}

	if (wait_till_ready(spi))
		return 1;

	return result;
}

int _spi_firmware_update_dp703(struct scaler_private_data *scaler)
{
	int result = 0;
	struct spi_device *spi = NULL;
	int i = 0, j = 0, k = 0;
	u8 buf[256];
	u8 *binary_data_dp703_read = NULL;
	u8 val = 0;
    u8 *bin_version = NULL;
    unsigned int chip_fireware_version = 0;
    unsigned int bin_file_version = 0;
	
#if defined(CONFIG_SPIM0_RK29)
	if(!g_spi_firmware_data[0])
	{
		printk("%s:g_spi_firmware_data is null\n",__func__);
		return -1;
	}
	
	spi = g_spi_firmware_data[0]->spi;
	
	if(!spi)
	{
		printk("%s:spi is null\n",__func__);
		return -1;
	}

	printk("%s:waiting for update dp703 firmware...\n",__func__);

	iomux_set(SPI0_TXD);	
	iomux_set(SPI0_RXD);	
	iomux_set(SPI0_CS0);
	iomux_set(SPI0_CLK);
	
	if(scaler->ops->reset)
		scaler->ops->reset(scaler, 1);

	binary_data_dp703_read = kzalloc(DP703_BIN0_SIZE, GFP_KERNEL);
	if (binary_data_dp703_read == NULL)
	return -ENOMEM;

	result = read_page(spi, 0x75000, 4, buf);
	if(result)
	{
		printk("%s:line=%d,fail to read version\n",__func__, __LINE__);
		//goto exit;
	}
	
	printk("%s:version:0x%x,0x%x,0x%x,0x%x\n",__func__,buf[0],buf[1],buf[2],buf[3]);

    bin_version = &binary_data_dp703[0x75000];
    printk("%s:bin version:0x%x,0x%x,0x%x,0x%x\n", __func__, 
        bin_version[0], bin_version[1],bin_version[2],bin_version[3]);

    chip_fireware_version = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
    bin_file_version = bin_version[0] << 24 | bin_version[1] << 16 | bin_version[2] << 8 | bin_version[3];
    if (chip_fireware_version >= bin_file_version)
    {
        printk("dp703 fireware version is last verion.\n");    
        goto last_version;
    }

    printk("dp703 fireware updata begin... \n");
	result = read_mid(spi, buf);
	if(result)
	{
		printk("%s:line=%d,fail to read mid\n",__func__, __LINE__);
		//goto exit;
	}

	printk("%s:mid:0x%x,0x%x\n",__func__,buf[0],buf[1]);

    write_sr(spi, 0);
    msleep(1); // 1ms
	val = read_sr(spi);
	printk("%s:line=%d,val=0x%x\n",__func__, __LINE__, val);

	result = write_enable(spi);
	if(result)
	{
		printk("%s:fail to enable spi flash,try %d times\n",__func__,i);
		goto exit;
	}
	
	//erase all chip
	result = erase_chip(spi);
	if(result)
	{
		printk("%s:line=%d,fail to erase flash chip\n",__func__, __LINE__);
		goto exit;
	}

	val = read_sr(spi);
	printk("%s:line=%d,val=0x%x\n",__func__, __LINE__, val);
	
	//enable
	result = write_enable(spi);
	if(result)
	{
		printk("%s:fail to enable spi flash,try %d times\n",__func__,i);
		goto exit;
	}

	val = read_sr(spi);
	printk("%s:line=%d,val=0x%x\n",__func__, __LINE__, val);

	for(i=0; i<24; i++)	//sector
	{
		for(j=0; j<256; j++)	//page
		{
			result = write_page(spi, ((i<<16) + (j<<8)), 256);
			if(result)	
			printk("%s:warning:write block %dth sector %dth page %dth error, val=0x%x\n",__func__, i/16, i%16, j,read_sr(spi));
		}
	}

	//enable
	result = write_enable(spi);
	if(result)
	{
		printk("%s:fail to enable spi flash,try %d times\n",__func__,i);
		goto exit;
	}

	val = read_sr(spi);
	printk("%s:line=%d,val=0x%x\n",__func__, __LINE__, val);
	

	//modify firmware
	for(i=0; i<24; i++)	//sector
	{
		for(j=0; j<256; j++)	//page
		{
			result = read_page(spi, ((i<<16) + (j<<8)), 256, &binary_data_dp703_read[((i<<16) + (j<<8))]);
			if(result)
			printk("%s:warning:read block %dth sector %dth page %dth error,sr=0x%x\n",__func__, i/16, i%16, j, read_sr(spi));

		}
	}


	for(i=0; i<DP703_BIN0_SIZE; i++)
	{
		if(binary_data_dp703_read[i] != binary_data_dp703[i])
		{
			printk("%d:0x%x,0x%x ",i, binary_data_dp703[i], binary_data_dp703_read[i]);
			if(i%6 == 0)
			printk("\n");
		}
	}

	//disable
	result = write_disable(spi);
	if(result)
	printk("%s:fail to write_disable\n",__func__);

exit:
			
    if(scaler->pdata->init_platform_hw)
        scaler->pdata->init_platform_hw(scaler, 0);

    msleep(500);
    if(scaler->pdata->init_platform_hw)
        scaler->pdata->init_platform_hw(scaler, 1);
    
	if(!result)
	printk("%s:update dp703 firmware successfully\n",__func__);
	else
	printk("%s:fail to update dp703 firmware\n",__func__);

last_version:

    //switch to gpio input
	gpio_request(RK30_PIN1_PA4, "gpio_spi0_rxd");
	gpio_pull_updown(RK30_PIN1_PA4, PullDisable);
	gpio_direction_input(RK30_PIN1_PA4);
	
	gpio_request(RK30_PIN1_PA5, "gpio_spi0_txd");
	gpio_pull_updown(RK30_PIN1_PA5, PullDisable);
	gpio_direction_input(RK30_PIN1_PA5);

	gpio_request(RK30_PIN1_PA6, "gpio_spi0_clk");
	gpio_pull_updown(RK30_PIN1_PA6, PullDisable);
	gpio_direction_input(RK30_PIN1_PA6);

	gpio_request(RK30_PIN1_PA7, "gpio_spi0_cs");
	gpio_pull_updown(RK30_PIN1_PA7, PullDisable);
	gpio_direction_input(RK30_PIN1_PA7);

    if(scaler->ops->reset)
		scaler->ops->reset(scaler, 0);

	msleep(100);
#endif

	return result;	
}

int spi_firmware_update_dp703(struct scaler_private_data *scaler)
{
	int i = 0, result = 0;
	for(i=0; i<3; i++)
	{
		result = _spi_firmware_update_dp703(scaler);
		if(!result)
		{
			break;
		}
		else
		msleep(1000);
	}

	if(i >= 3)
	printk("%s:fail to update firmware of dp703 at last,i=%d\n",__func__, i);

	return result;
}	


#if 0
static ssize_t spi_firmware_write(struct file *file, 
			const char __user *buf, size_t count, loff_t *offset)
{

	return count;
}


static const struct file_operations spi_firmware_fops = {
	.write = spi_firmware_write,
};

static struct miscdevice spi_firmware_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "spi_misc_firmware",
	.fops = &spi_firmware_fops,
};
#endif
static int __devinit spi_firmware_probe(struct spi_device *spi)
{	
	struct spi_firmware_data *spi_firmware_data;
	int ret;

	if(!spi)	
	return -ENOMEM;

	if((spi->master->bus_num >= MAX_SPI_BUS_NUM) || (spi->master->bus_num < 0))
	{
		printk("%s:error:bus_num=%d\n",__func__, spi->master->bus_num);	
		return -ENOMEM;
	}
	
	spi_firmware_data = (struct spi_firmware_data *)kzalloc(sizeof(struct spi_firmware_data), GFP_KERNEL);
	if(!spi_firmware_data){
		dev_err(&spi->dev, "ERR: no memory for spi_firmware_data\n");
		return -ENOMEM;
	}

	spi->bits_per_word = 8;
	
	spi_firmware_data->spi = spi;
	spi_firmware_data->dev = &spi->dev;
	ret = spi_setup(spi);
	if (ret < 0){
		dev_err(spi_firmware_data->dev, "ERR: fail to setup spi\n");
		return -1;
	}	

	g_spi_firmware_data[spi->master->bus_num] = spi_firmware_data;

	printk("%s:bus_num=%d,ok\n",__func__,spi->master->bus_num);

	return ret;

}

static const struct spi_device_id spi_firmware_id[] = {		
	{"spi_firmware_bus0", 0},
	//{"spi_firmware_bus1", 1},
	{},
};


static struct spi_driver spi_firmware_driver = {
	.driver = {
		.name		= "spi_firmware",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},
	.id_table = spi_firmware_id,

	.probe		= spi_firmware_probe,
};

static int __init spi_firmware_init(void)
{	
	printk("%s\n",__func__);
	spi_register_board_info(board_spi_firmware_devices, ARRAY_SIZE(board_spi_firmware_devices));
	//misc_register(&spi_firmware_misc);
	return spi_register_driver(&spi_firmware_driver);
}

subsys_initcall(spi_firmware_init);

static void __exit spi_firmware_exit(void)
{
	//misc_deregister(&spi_firmware_misc);
	spi_unregister_driver(&spi_firmware_driver);
}


module_exit(spi_firmware_exit);
