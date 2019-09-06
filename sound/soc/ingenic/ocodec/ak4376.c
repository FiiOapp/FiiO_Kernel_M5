/*
 * ak4376.c
 *
 * Supports AXP173 Regulator
 *
 * Copyright (C) 2009 Texas Instrument Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
//#include <linux/power/axp173.h>
#include <linux/interrupt.h> 
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <irq.h>
#include <soc/gpio.h>
#include <linux/ioport.h>
#include <linux/gpio.h>

#include <linux/spinlock.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/pmu.h>
#include <linux/skytc/eq/eq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/delay.h>
#include <sound/ak4376.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
//#include <linux/gpio_keys.h>
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif
#if 0
#define DBG_INFO(x...)	printk(KERN_INFO x)
#else
#define DBG_INFO(x...)
#endif


#define  uint8   unsigned char
#define  uint16  unsigned short
#define  int16   short
#define  uint32  unsigned int
#define  int32   int
#define  uint64  unsigned long

struct ak4376_reg{
			 int reg;
			 int value;
};
static struct ak4376_reg ak4376_reg_defaults[] ={
// AKM_Register AK4376 20
{ 0x00 ,0x01 },
{ 0x01 ,0x33 },
{ 0x02 ,0x01 },
{ 0x03 ,0x03 },
{ 0x04 ,0x00 },
{ 0x05 ,0x0A },
{ 0x06 ,0x00 },
{ 0x07 ,0x21 },
{ 0x0B ,0x80 },
{ 0x0C ,0x00 },
{ 0x0D ,0x0B },
{ 0x0E ,0x01 },
{ 0x0F ,0x00 },
{ 0x10 ,0x00 },
{ 0x11 ,0x00 },
{ 0x12 ,0x27 },
{ 0x13 ,0x01 },
{ 0x14 ,0x09 },
{ 0x15 ,0xE0 },
{ 0x24 ,0x00 },
};

static struct ak4376_reg ak4376_low_defaults[] ={
// AKM_Register AK4376 20
{ 0x00 ,0x01 },
{ 0x01 ,0x33 },
{ 0x02 ,0x11 },
{ 0x03 ,0x03 },
{ 0x04 ,0x00 },
{ 0x05 ,0x0A },
{ 0x06 ,0x00 },
{ 0x07 ,0x21 },
{ 0x0B ,0x80 },
{ 0x0C ,0x00 },
{ 0x0D ,0x0B },
{ 0x0E ,0x01 },
{ 0x0F ,0x00 },
{ 0x10 ,0x00 },
{ 0x11 ,0x00 },
{ 0x12 ,0x27 },
{ 0x13 ,0x01 },
{ 0x14 ,0x09 },
{ 0x15 ,0xE0 },
{ 0x24 ,0x40 },
};


static char driver_name[] = "ak4376";	
struct ak4376 {
	struct device *dev;
	//struct mutex io_lock;
    int	irq;	/* if 0, use polling */
	//spinlock_t              io_lock;	
	struct i2c_client *i2c;
	int num_regulators;
	struct regulator_dev **rdev;
};
//static DEFINE_MUTEX(axp173_i2c_mutex);
struct ak4376 *ak4376;

static int ak4376_i2c_gpio_init(void)
{
    int ret ;
    printk("======%s:line:%d=============\n",__func__,__LINE__);

    if(AK4376_SCL> 0){
	printk(KERN_INFO"try to request ak4376 scl\n");
        ret = gpio_request(AK4376_SCL, "ak4376 scl");
        if (ret) {
            printk(KERN_ERR "can's request ak4376 scl\n");
            return ret;
        }
    }
    if(AK4376_SDA> 0){
	printk(KERN_INFO"try to request ak4376 sda\n");
        ret = gpio_request(AK4376_SDA, "ak4376 sda");
        if (ret) {
            printk(KERN_ERR "can's request ak4376 sda\n");
            return ret;
        }
    }
    if(PO_EN> 0){
        printk(KERN_INFO"try to request po_en\n");
        ret = gpio_request(PO_EN, "po_en");
        if (ret) {
            printk(KERN_ERR "can's request po_en\n");
            return ret;
        }
	gpio_direction_output(PO_EN,0);
	//mdelay(1);
	//gpio_direction_output(PO_EN,1);
    }

}	

//
static void IIC_Start(void)
{
	ndelay(50);
	gpio_direction_output(AK4376_SDA,1);
  	ndelay(50);	
	gpio_direction_output(AK4376_SCL,1);
	ndelay(50);
 	gpio_direction_output(AK4376_SDA,0);
	ndelay(50);
	gpio_direction_output(AK4376_SCL,0);
  	ndelay(50);
}	  

static void IIC_Stop(void)
{
	ndelay(50);
        gpio_direction_output(AK4376_SCL,0);
        ndelay(50);
        gpio_direction_output(AK4376_SDA,0);
        ndelay(50);
	gpio_direction_output(AK4376_SCL,1);
        ndelay(50);
        gpio_direction_output(AK4376_SDA,1);
        ndelay(50);
}

static uint8 IIC_Wait_Ack(void)
{
	uint16 ucErrTime=0;
	
	gpio_direction_input(AK4376_SDA);      
	gpio_direction_output(AK4376_SCL,1);
	ndelay(100);
	while(gpio_get_value(AK4376_SDA))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			printk("ak4376 ack Outtime!\n");	
			IIC_Stop();
			return 1;
		}
	}
	gpio_direction_output(AK4376_SCL,0);
	return 0;  
} 


static void IIC_Ack(void)
{
	gpio_direction_output(AK4376_SCL,0);
        gpio_direction_output(AK4376_SDA,0);
	ndelay(20);
	gpio_direction_output(AK4376_SCL,1);
	ndelay(50);
	gpio_direction_output(AK4376_SCL,0);
}

static void IIC_NAck(void)
{
	gpio_direction_output(AK4376_SCL,0);
        gpio_direction_output(AK4376_SDA,1);
        ndelay(20);
        gpio_direction_output(AK4376_SCL,1);
        ndelay(50);
        gpio_direction_output(AK4376_SCL,0);

}

static void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t;	    
    gpio_direction_output(AK4376_SCL,0);
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>>7)
		gpio_direction_output(AK4376_SDA,1);
	else gpio_direction_output(AK4376_SDA,0);
        txd<<=1; 	  
		ndelay(50);  
		gpio_direction_output(AK4376_SCL,1);
		ndelay(50);
    		ndelay(50);			
		gpio_direction_output(AK4376_SCL,0);
		
    }	 
		ndelay(50);
		
} 	    

static uint8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	gpio_direction_input(AK4376_SDA);
    for(i=0;i<8;i++ )
	{
        gpio_direction_output(AK4376_SCL,0);
        ndelay(50);
	gpio_direction_output(AK4376_SCL,1);
        receive<<=1;
        if(gpio_get_value(AK4376_SDA))receive++;   
		ndelay(50); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();    
    return receive;
}

unsigned char AK4376_ReadOneByte(uint8 ReadAddr)
{				  
	uint8 temp=0;		  	    																 
    IIC_Start();  
		IIC_Send_Byte(0x20);  	   
	  IIC_Wait_Ack(); 
		IIC_Send_Byte(ReadAddr);
		IIC_Wait_Ack();	    
	  IIC_Start();  	 	   
		IIC_Send_Byte(0x21);
		IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(1);
		IIC_Wait_Ack();
    IIC_Stop();	
	
	return temp;
}
EXPORT_SYMBOL(AK4376_ReadOneByte);
void AK4376_WriteOneByte(uint8 WriteAddr,uint8 DataToWrite)
{				   	  	    																 
    IIC_Start();  
		IIC_Send_Byte(0x20);
		IIC_Wait_Ack();
		IIC_Send_Byte(WriteAddr);
		IIC_Wait_Ack();	   
    IIC_Send_Byte(DataToWrite);   
	  IIC_Wait_Ack();
    IIC_Stop();
	ndelay(100);	 
}
EXPORT_SYMBOL(AK4376_WriteOneByte);
/* read value from register */
int ak4376_i2c_read(struct ak4376 *bq, u8 reg)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
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

	//mutex_lock(&axp173_i2c_mutex);
	//spin_lock(&axp173_i2c_lock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	//ret = __i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	//mutex_unlock(&axp173_i2c_mutex);
	//spin_unlock(&axp173_i2c_lock);
	if (ret < 0)
		return ret;

	return val;
}
EXPORT_SYMBOL(ak4376_i2c_read);

/* write value to register */
int ak4376_i2c_write(struct ak4376 *bq, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	//mutex_lock(&axp173_i2c_mutex);
	//spin_lock(&axp173_i2c_lock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	//mutex_unlock(&axp173_i2c_mutex);
	//spin_unlock(&axp173_i2c_lock);
	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(ak4376_i2c_write);
/* read value from register, change it with mask left shifted and write back */
static int ak4376_i2c_write_mask(struct ak4376 *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = ak4376_i2c_read(bq, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return ak4376_i2c_write(bq, reg, ret);
}

/* read value from register, apply mask and right shift it */
static int ak4376_i2c_read_mask(struct ak4376 *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = ak4376_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}


/* read value from register and return one specified bit */
static int ak4376_i2c_read_bit(struct ak4376 *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return ak4376_i2c_read_mask(bq, reg, BIT(bit), bit);
}
//EXPORT_SYMBOL(axp173_i2c_read_bit);

/* change only one bit in register */
static int ak4376_i2c_write_bit(struct ak4376 *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return ak4376_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

void ak4376_cache_reg_init(void)
{

        int i; 
        for(i=0; i<20; i++)
        {
                ak4376_i2c_write(ak4376,ak4376_reg_defaults[i].reg, ak4376_reg_defaults[i].value);
                
  }

}
void ak4376_reg_init(void)
{

        int i;
        for(i=0; i<20; i++)
        {
		if(!ak4376_power_mode)
               AK4376_WriteOneByte(ak4376_reg_defaults[i].reg, ak4376_reg_defaults[i].value);
		else if(ak4376_power_mode)
		AK4376_WriteOneByte(ak4376_low_defaults[i].reg, ak4376_low_defaults[i].value);
  }

}


static int  ak4376_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{

	//struct axp173_platform_data *pdata = i2c->dev.platform_data;
	int ret;
	printk("enter ak4376_i2c_probe\n");
	//ak4376_i2c_gpio_init();
	//ak4376_reg_init();
	ak4376 = kzalloc(sizeof(struct ak4376),GFP_ATOMIC);//GFP_KERNEL);//
    if (ak4376 == NULL) {
        ret = -ENOMEM;
        goto err;
    }

	ak4376->i2c = i2c;
	ak4376->dev = &i2c->dev;
	i2c_set_clientdata(i2c, ak4376);
	
	ak4376_cache_reg_init();
	printk("ak4376 reg15=%d\n",ak4376_i2c_read(ak4376,0x15));
	ret=ak4376_i2c_read_bit(ak4376,0x01,5);
	if ((ret < 0) || (ret == 0xff)){
		printk(KERN_ERR"The device is not act:ak4376 \n");
		return 0;
	}
   	
	return 0;

err:
	return ret;	

}


static ssize_t test_eq_show(struct device *device, struct device_attribute *attr, char *buf)
{

	return 0;
}

static ssize_t test_eq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	return 0;
}
static ssize_t reg_show(struct device *device, struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
	return 0;
}			
static struct device_attribute ak4376_device_attrs[] = {
	__ATTR(reg, S_IRUGO | S_IWUSR, reg_show, reg_store),
		
	__ATTR(test_eq, S_IRUGO | S_IWUSR, test_eq_show, test_eq_store),
};

static int ak4376_device_attr_register(struct miscdevice  *ak4376_misc_opt)
{
	int error = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ak4376_device_attrs); i++) {
		error = device_create_file(ak4376_misc_opt->this_device, &ak4376_device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(ak4376_misc_opt->this_device, &ak4376_device_attrs[i]);
	}

	return 0;
}

static int ak4376_device_attr_unregister(struct miscdevice  *ak4376_misc_opt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ak4376_device_attrs); i++)
		device_remove_file(ak4376_misc_opt->this_device, &ak4376_device_attrs[i]);

	return 0;
}
static int ak4376_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int ak4376_release(struct inode *inode, struct file *file)
{
	return 0;
}

void fiio_ak4376_poweron() {
	printk("try to power on ak4376\n");
    gpio_direction_output(PO_EN, 1);
    mdelay(1);
    ak4376_reg_init();

	printk("set volumeR=0x%02x volumeL=0x%02x\n",0x19,0x19);
	AK4376_WriteOneByte(0x0C,0x19);
	AK4376_WriteOneByte(0x0B,0x19);

}
EXPORT_SYMBOL(fiio_ak4376_poweron);

void fiio_ak4376_poweroff() {
	printk("try to power down ak4376\n");
    gpio_direction_output(PO_EN, 0);
}
EXPORT_SYMBOL(fiio_ak4376_poweroff);
static void set_ak4376_high_256k_sample() {
	//02H LPMODE 0:high performance default 0 1:Low Power Mode
	if(!ak4376_power_mode) 
		AK4376_WriteOneByte(0x02,0x11);
	else
		AK4376_WriteOneByte(0x02,0x01);
	//24H 64fs
	AK4376_WriteOneByte(0x24,0x00);
}
static void set_ak4376_low_256k_sample() {
	//02H LPMODE
	AK4376_WriteOneByte(0x02,0x11);
	//24H >128fs
	AK4376_WriteOneByte(0x24,0x40);
}
void ak4376_sample_set(int sample)
{
	AK4376_WriteOneByte(0x03,0xFC);
	switch(sample){
	case 1://8000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x20);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0xEF);
        AK4376_WriteOneByte(0x14,0x1D);
	break;
	case 2://11025
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x21);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x9F);
        AK4376_WriteOneByte(0x14,0x13);
	break;
	case 3://12000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x0E);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x9F);
        AK4376_WriteOneByte(0x14,0x13);
	break;
	case 4://16000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x24);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x77);
        AK4376_WriteOneByte(0x14,0x0E);
	break;
	case 5://22050
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x25);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x4F);
        AK4376_WriteOneByte(0x14,0x09);
	break;
	case 6://24000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x26);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x4F);
        AK4376_WriteOneByte(0x14,0x09);	
	break;
	case 7://32000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x08);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x3B);
        AK4376_WriteOneByte(0x14,0x0E);
	break;
	case 8://44100
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x09);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x27);
        AK4376_WriteOneByte(0x14,0x09);
	break;
	case 9://48000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x0A);
        AK4376_WriteOneByte(0x10,0x00);
        AK4376_WriteOneByte(0x12,0x27);
        AK4376_WriteOneByte(0x14,0x09);
	break;
	case 10://64000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x0C);
        AK4376_WriteOneByte(0x10,0x01);
        AK4376_WriteOneByte(0x12,0x27);
        AK4376_WriteOneByte(0x14,0x04);
	break;
	case 11://88200
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x0D);
        AK4376_WriteOneByte(0x10,0x01);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 12://96000
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x0E);
        AK4376_WriteOneByte(0x10,0x01);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 13://176400
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x71);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 14: //192K
		set_ak4376_low_256k_sample();
		AK4376_WriteOneByte(0x05,0x72);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 15:
		//high performance mode 64fs
		//256K
		set_ak4376_high_256k_sample();
		//05H D6 D5 must set 0 1
		AK4376_WriteOneByte(0x05,0x34);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 16:
		//high performance mode 64fs
		//352800
		set_ak4376_high_256k_sample();
		//05H D6 D5 must set 0 1
		AK4376_WriteOneByte(0x05,0x35);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	case 17: //384K
		//high performance mode 64fs
		set_ak4376_high_256k_sample();
		//05H D6 D5 must set 0 1
		AK4376_WriteOneByte(0x05,0x36);
		AK4376_WriteOneByte(0x10,0x03);
		AK4376_WriteOneByte(0x12,0x27);
		AK4376_WriteOneByte(0x14,0x04);
	break;
	default:
	break;
	}
	AK4376_WriteOneByte(0x03,0x03);
}
static int ak4376_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//	 int err = 0;
    int ret = 0;
   // int ioarg = 0;
 //   int reg,bit,sign;
   // struct jz_gpio_func_def *g1 = &platform_lcd_array[0];
    
    printk("enter  ak4376_ioctl\n");
	    /* 检测命令的有效性 */
 //   if (_IOC_TYPE(cmd) != MEMDEV_IOC_MAGIC) 
   //  {printk(" ak4376  cmd error1\n");
        //return -EINVAL;
     //   }
   //  if (_IOC_NR(cmd) > MEMDEV_IOC_MAXNR) 
   //        {  printk(" ak4376  cmd error2\n");
        //return -EINVAL;
//	}
	 /* 根据命令，执行相应的操作 */
    switch(cmd) {

      /* 打印当前设备信息 */
    //  case MEMDEV_IOCPRINT:
      
      //  break;
      /* 获取参数 */
      case AK4376_POWER_ON:
      printk("try to power on ak4376\n");
      gpio_direction_output(PO_EN, 1);
      mdelay(1);
      ak4376_reg_init();
      if(work_sign)
	{
             AK4376_WriteOneByte(0x0C,volumeR);
	     AK4376_WriteOneByte(0x0B,volumeL);
	}
        break;
      
      /* 设置参数 */
      case AK4376_POWER_DOWN: 
      printk("try to power down ak4376\n");
      gpio_direction_output(PO_EN, 0);
        break;

        default:  
        return -EINVAL;
    }
    return ret;
}

static struct file_operations ak4376_opt_fops = {
	.owner	=	THIS_MODULE,
	.open	=	ak4376_open,
	.unlocked_ioctl		=	ak4376_ioctl,
	.release	= 	ak4376_release,
};

static struct miscdevice ak4376_misc_opt = {
	.minor	=	MISC_DYNAMIC_MINOR,
	.name	= 	(char *)driver_name,
	.fops	=	&ak4376_opt_fops,
};
static int ak4376_i2c_remove(struct i2c_client *i2c)
{
	struct ak4376 *ak4376 = i2c_get_clientdata(i2c);
	i2c_set_clientdata(i2c, NULL);
	kfree(ak4376);
	return 0;
}
static int ak4376_suspend(struct i2c_client *i2c)
{
	printk("PM:ak4376_suspend,try to close dac\n");
	gpio_direction_output(PO_EN, 0);
	return 0;
}

static int ak4376_resume(struct i2c_client *i2c)
{
	printk("PM:ak4376_resume,try to start dac\n");
        //gpio_direction_output(PO_EN, 1);
	//mdelay(1);
        //ak4376_cache_reg_init();
        return 0;
}

static const struct i2c_device_id ak4376_i2c_id[] = {
       { "ak4376a", 0 },
       { }
};

MODULE_DEVICE_TABLE(i2c, ak4376_i2c_id);

static struct i2c_driver ak4376_i2c_driver = {
	.driver = {
		.name = "ak4376a",
		.owner = THIS_MODULE,
	},
	.probe    = ak4376_i2c_probe,
	.remove   = ak4376_i2c_remove,
	.suspend    = ak4376_suspend,
        .resume     = ak4376_resume,
	.id_table = ak4376_i2c_id,
};
//module_i2c_driver(axp173_i2c_driver);

static int __init ak4376_module_init(void)
{
	int ret, retval;
	printk(KERN_INFO"enter %s,line:%d\n",__func__,__LINE__);
/*	if(PO_EN > 0){
        ret = gpio_request(PO_EN, "PO_EN");
        if (ret) {
            printk(KERN_ERR "can's request PO_EN\n");
           return ret;
        }
        gpio_direction_output(PO_EN, 1);
        //gpio_direction_output(PO_EN, 0);
        }
*/	ak4376_i2c_gpio_init();
	//ak4376_reg_init();
	
//	ret = i2c_add_driver(&ak4376_i2c_driver);
	if (ret != 0)
		printk(KERN_INFO"Failed to register AK4376 I2C driver: %d\n", ret);
		else {
		printk("Successfully added driver %s\n",
				ak4376_i2c_driver.driver.name);
				}
				
	 retval = misc_register(&ak4376_misc_opt);
	if (retval < 0)
	{
		printk(KERN_ERR"register misc device opt failed.\n");
		return retval;
	}
	ak4376_device_attr_register(&ak4376_misc_opt);

    return ret;
}
module_init(ak4376_module_init);

static void __exit ak4376_module_exit(void)
{
	gpio_free(PO_EN);
	//i2c_del_driver(&ak4376_i2c_driver);
	//ak4376_device_attr_unregister(&ak4376_misc_opt);
}
module_exit(ak4376_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhengpeizhu <zhengpz@fiio.cn>");
MODULE_DESCRIPTION("ak4376 DAC driver");
