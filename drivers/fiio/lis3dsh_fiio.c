/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
*
* File Name	: lis3dsh_i2c.c
* Authors	: AMS - Motion Mems Division - Application Team - Application Team
*		     : Giuseppe Barba <giuseppe.barba@st.com>
*		     : Mario Tesi <mario.tesi@st.com>
*		     : Author is willing to be considered the contact and update
* Version	: V.1.0.14
* Date		: 2016/Apr/26
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include "lis3dsh.h"

#define FIIO_DEBUG_SENSOR
#ifdef FIIO_DEBUG_SENSOR
#define fiio_debug(x...)  printk(KERN_INFO "[lis3dsh] " x)
#else
#define fiio_debug(x...)
#endif


#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

static struct i2c_client *g_client;
struct lis3dsh_reg{
			 u8 reg;
			 u8 value;
};
static struct lis3dsh_reg lis3dsh_reg_defaults[] ={
	{0x20, 0x67},
	{0x23, 0xC4},
	{0x24, 0x00},
	{0x25, 0x10},
	{0x27, 0xFF},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x00},
	{0x13, 0x00},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x16, 0xFF},
	{0x17, 0x7F},
	//{0x16, 0x00},
	//{0x17, 0x00},
	{0x1B, 0x7D},
	{0x1C, 0x72},
	{0x1D, 0x4C},
	{0x1E, 0x26},
	{0x1F, 0x00},
	{0x2E, 0x00},
};

static struct lis3dsh_reg lis3dsh_reg_8step_100hz_2g[] ={
	{0x19, 0x00 },
	{0x1A, 0x00 },
	{0x50, 0x00 },
	{0x51, 0x00 },
	{0x52, 0x30 },
	{0x53, 0x00 },
	{0x54, 0x46 },
	{0x55, 0x00 },
	{0x56, 0x00 },
	{0x57, 0x08 },
	{0x59, 0x00 },
	{0x5A, 0x03 },
	{0x5B, 0x21 },
	{0x5C, 0x00 },
	{0x5D, 0x1B },
	{0x5E, 0x00 },
	{0x5F, 0x01 },
	{0x70, 0x00 },
	{0x71, 0x00 },
	{0x72, 0x30 },
	{0x73, 0x00 },
	{0x74, 0x46 },
	{0x75, 0x00 },
	{0x76, 0x00 },
	{0x77, 0x08 },
	{0x78, 0x00 },
	{0x79, 0x00 },
	{0x7A, 0x03 },
	{0x7B, 0x21 },
	{0x7C, 0x01 },
	{0x7D, 0x00 },
	{0x7E, 0x00 },
	{0x7F, 0x00 },
	//STx_1 (40h-4Fh)
	{0x40, 0x15 },
	{0x41, 0x02 },
	{0x42, 0x15 },
	{0x43, 0x02 },
	{0x44, 0x15 },
	{0x45, 0x02 },
	{0x46, 0x15 },
	{0x47, 0x02 },
	{0x48, 0x15 },
	{0x49, 0x02 },
	{0x4A, 0x15 },
	{0x4B, 0x02 },
	{0x4C, 0x15 },
	{0x4D, 0x02 },
	{0x4E, 0xFF },
	{0x4F, 0x11 },
	{0x60, 0xFF },
	{0x61, 0x15 },
	{0x62, 0x02 },
	{0x63, 0xBB },
	{0x64, 0xBB },
	{0x65, 0xBB },
	{0x66, 0xBB },
	{0x67, 0xBB },
	{0x68, 0xBB },
	{0x69, 0xBB },
	{0x6A, 0xBB },
	{0x6B, 0x15 },
	{0x6C, 0x02 },
	{0x6D, 0x22 },
	{0x6E, 0x57 },
	{0x6F, 0xAA },
	//CTRL_REG3
	//{0x23, 0xC4 },
	{0x23, 0xC4 },
	
	{0x21, 0x09 },
	{0x22, 0x09 },
};


static int lis3dsh_i2c_read_fiio(struct i2c_client *client, u8 reg)
{
	struct i2c_msg msg[2];
	u8 val;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &val;
	msg[1].len = sizeof(val);
	
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		printk("%s read 0x%02x error!\n",__func__,reg);
		return ret;
	}
		
	return val;
}

static int lis3dsh_i2c_write_fiio(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		printk("%s write 0x%02x error code=%d!\n",__func__,reg,ret);
		return ret;
	}
	else if (ret != 1) {	
		printk("%s write 0x%02x error code=%d!\n",__func__,reg,EIO);
		return -EIO;
	}
		
	return 0;
}

/* read value from register, change it with mask left shifted and write back */
static int lis3dsh_i2c_write_mask(struct i2c_client *client, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = lis3dsh_i2c_read_fiio(client, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return lis3dsh_i2c_write_fiio(client, reg, ret);
}

/* read value from register, apply mask and right shift it */
static int lis3dsh_i2c_read_mask(struct i2c_client *client, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = lis3dsh_i2c_read_fiio(client, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}


/* read value from register and return one specified bit */
static int lis3dsh_i2c_read_bit(struct i2c_client *client, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return lis3dsh_i2c_read_mask(client, reg, BIT(bit), bit);
}

/* change only one bit in register */
static int lis3dsh_i2c_write_bit(struct i2c_client *client, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return lis3dsh_i2c_write_mask(client, reg, val, BIT(bit), bit);
}

static int lis3dsh_i2c_read(u8 reg_addr,
			    int len, u8 *data)
{
	int err = 0;
	struct i2c_msg msg[2];
	int tries = 0;
	struct i2c_client *client = g_client;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].len = 1;
	msg[0].buf = &reg_addr;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data;

	do {
		err = i2c_transfer(client->adapter, msg, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	return err;
}

static int lis3dsh_i2c_write(u8 reg_addr,
			     int len, u8 *data)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	int tries = 0;
	struct i2c_client *client = g_client;

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	len++;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = len;
	msg.buf = send;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1)
		return -EIO;

	return 0;
}

////////////////////////////////////////////////////////////////////////////////

static irqreturn_t lis3dsh_isr1(int irq, void *dev)
{
	struct lis3dsh_status *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);

	return IRQ_HANDLED;
}

static irqreturn_t lis3dsh_isr2(int irq, void *dev)
{
	struct lis3dsh_status *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);

	return IRQ_HANDLED;
}

static void lis3dsh_irq1_work_func(struct work_struct *work)
{
	struct lis3dsh_status *acc;

	acc = container_of(work, struct lis3dsh_status, irq1_work);
	fiio_debug("%s: IRQ1 triggered\n", LIS3DSH_ACC_DEV_NAME);
	/* TODO  add interrupt service procedure.
		 ie:lis3dsh_get_int_source(acc); */
	enable_irq(acc->irq1);
}

static void lis3dsh_irq2_work_func(struct work_struct *work)
{
	struct lis3dsh_status *acc;

	acc = container_of(work, struct lis3dsh_status, irq2_work);
	fiio_debug("%s: IRQ2 triggered\n", LIS3DSH_ACC_DEV_NAME);
	/* TODO  add interrupt service procedure.
		 ie:lis3dsh_get_stat_source(acc); */
	/* ; */
	enable_irq(acc->irq2);
}

//////////////////////////////////////////////////////////////////////////////

void lis3dsh_init(void)
{
	int i;
	int err = 0;
	u8 tmp_data[1];
	fiio_debug("%s:Enter!!!\n",__func__);
	for(i=0; i<sizeof(lis3dsh_reg_defaults)/sizeof(lis3dsh_reg_defaults[0]); i++)
	{
		tmp_data[0] = lis3dsh_reg_defaults[i].value;
		err = lis3dsh_i2c_write(lis3dsh_reg_defaults[i].reg, 1,tmp_data);
		if (err != 0) {
			printk("%s lis3dsh_reg_defaults write reg error!\n",__func__);
		}
  	}
	
	for(i=0; i<sizeof(lis3dsh_reg_8step_100hz_2g)/sizeof(lis3dsh_reg_8step_100hz_2g[0]); i++)
	{
		tmp_data[0] = lis3dsh_reg_8step_100hz_2g[i].value;
		err = lis3dsh_i2c_write(lis3dsh_reg_8step_100hz_2g[i].reg, 1,tmp_data);
		if (err != 0) {
			printk("%s lis3dsh_reg_8step_100hz_2g write reg error!\n",__func__);
		}
  	}
	fiio_debug("%s:Exit!!!\n",__func__);

}
EXPORT_SYMBOL(lis3dsh_init);

int lis3dsh_get_step(void) {
	#if 0
	u8 lc_l[1],lc_h[1];
	int err = 0;
	int counter = 0;
	err = lis3dsh_i2c_read(0x16,1,lc_l);
	if (err != 0) {
		printk("%s read reg[0x16] error!\n",__func__);
		return -EIO;
	}
	err = lis3dsh_i2c_read(0x17,1,lc_h);
	if (err != 0) {
		printk("%s read reg[0x17] error!\n",__func__);
		return -EIO;
	}
	printk(">>>>>>>>>>>>>>>>>>>>lc_l=0x%02x lc_h=0x%02x\n",lc_l[0],lc_h[0]);
	counter = lc_h[0] << 8 | lc_l[0];
	#endif
	int err = 0;
	int counter = 0;
	int lc_l,lc_h;
	lc_l = lis3dsh_i2c_read_fiio(g_client,0x16);
	lc_h = lis3dsh_i2c_read_fiio(g_client,0x17);
	fiio_debug(">>>>>>>>>>>>>>>>>>>>lc_l=%d lc_h=%d\n",lc_l,lc_h);
	counter = (0x7F << 8 | 0xFF)-(lc_h << 8 | lc_l);
	if (counter < 0)
		counter = 0;
	return counter;
	
}

EXPORT_SYMBOL(lis3dsh_get_step);


#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>

static int lis3dsh_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	int counter = 0;
	fiio_debug("%s:Enter!!!\n",__func__);
	
	struct lis3dsh_status *stat;
	
	stat = kmalloc(sizeof(struct lis3dsh_status), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bustype = BUS_I2C;
	i2c_set_clientdata(client, stat);

	stat->irq1 = gpio_to_irq(LIS3DH_INT1);
	stat->irq2 = gpio_to_irq(LIS3DH_INT2);
	stat->irq1_work_queue = create_singlethread_workqueue("irq1_work_queue");
	if (stat->irq1_work_queue == NULL) {
		printk("failed to create work queue\n");
		return -ENOMEM;
	}
	INIT_WORK(&stat->irq1_work,  lis3dsh_irq1_work_func);

	stat->irq2_work_queue = create_singlethread_workqueue("irq2_work_queue");
	if (stat->irq2_work_queue == NULL) {
		printk("failed to create work queue\n");
		return -ENOMEM;
	}
	INIT_WORK(&stat->irq2_work,lis3dsh_irq2_work_func);
	
	//queue_delayed_work(stat->irq1_work_queue,  &stat->irq1_work, msecs_to_jiffies(1000));

	
	g_client = client;
	counter = lis3dsh_get_step();
	fiio_debug("%s>>>>>>>>>>>>>>>>>>>>counter=%d\n",__func__,counter);
	//lis3dsh_init();
	//counter = lis3dsh_get_step();
	//fiio_debug("%s>>>>>>>>>>>>>>>>>>>>counter=%d\n",__func__,counter);

	err = request_irq(stat->irq1, lis3dsh_isr1,
                 IRQF_TRIGGER_FALLING, "lis3dsh_isr1", stat);
    if (err < 0) {
            printk("request irq failed: %d\n", err);
           return -ENOMEM;
    }

	err = request_irq(stat->irq2, lis3dsh_isr2,
                 IRQF_TRIGGER_FALLING, "lis3dsh_isr2", stat);
    if (err < 0) {
            printk("request irq failed: %d\n", err);
            return -ENOMEM;
    }
	
	fiio_debug("%s:Exit!!!\n",__func__);
	return 0;

free_data:

	return err;
}

static int lis3dsh_i2c_remove(struct i2c_client *client)
{

	return 0;
}

#ifdef CONFIG_PM
static int lis3dsh_suspend(struct device *dev)
{
	
	return 0;
}

static int lis3dsh_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops lis3dsh_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis3dsh_suspend, lis3dsh_resume)
};

#define LIS3DSH_PM_OPS	(&lis3dsh_pm_ops)
#else /* CONFIG_PM */
#define LIS3DSH_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis3dsh_ids[] = {
	{ LIS3DSH_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lis3dsh_ids);

#ifdef CONFIG_OF
static const struct of_device_id lis3dsh_id_table[] = {
	{ .compatible = "st,lis3dsh", },
	{ },
};
MODULE_DEVICE_TABLE(of, lis3dsh_id_table);
#endif

static struct i2c_driver lis3dsh_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS3DSH_ACC_DEV_NAME,
#ifdef CONFIG_PM
		.pm = LIS3DSH_PM_OPS,
#endif
#ifdef CONFIG_OF
		.of_match_table = lis3dsh_id_table,
#endif
	},
	.remove = lis3dsh_i2c_remove,
	.probe = lis3dsh_i2c_probe,
	.id_table = lis3dsh_ids,
};

module_i2c_driver(lis3dsh_i2c_driver);

MODULE_DESCRIPTION("STMicroelectronics lis3dsh i2c driver");
MODULE_AUTHOR("pengweizhong@fiio.net");
MODULE_LICENSE("GPL v2");
