#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/power/axp173.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <soc/gpio.h>
#define  uint8   unsigned char
#define  uint16  unsigned short
#define  int16   short
#define  uint32  unsigned int
#define  int32   int
#define  uint64  unsigned long

static struct kobject *debug_kobj;
//extern void pr_spi_devices(void);
static ssize_t
lcd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t lcd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int event = simple_strtoul(buf, NULL, 0);
	switch(event) {
		case 5:
			//pr_spi_devices();
			break;
		case 6:
			//fiio_gpio_uninstall();
			break;
		default:
			break;
	}
	return n;
}

static ssize_t
axp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 1;
}

//axp173_i2c_write_bit(axp173,0x12,1,3);
static ssize_t axp_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int reg = 0;
	int bit = 0;
	int value = 0;
	printk("axp_store write command:%s\n",buf);
	sscanf(buf, "%d %d %d", &reg,&bit,&value);
	printk("write axp192 reg=0x%02x bit=%d value=%d\n",reg,bit,value);
	axp173_i2c_write_bit(axp173,reg,value,bit);
	printk("bit=%d bit_value=%d value=%d\n",bit,axp173_i2c_read_bit(axp173,reg,bit),axp173_i2c_read(axp173,reg));
	return n;
}

static ssize_t read_axp192_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int reg = simple_strtoul(buf, NULL, 0);
	int value = 0;
	int i = 0;
	for(i=0;i<8;i++) {
		printk("bit=%d bit_value=%d\n",i,axp173_i2c_read_bit(axp173,reg,i));
	}
	printk("\nreg=0x%02x value=0x%02x\n",reg,axp173_i2c_read(axp173,reg));
	return n;
}
//I2C 1 GPIO TEST
//#define FIIO_I2C1_GPIO_TEST (1)

#ifdef FIIO_I2C1_GPIO_TEST
#define FIIO_I2C1_CLK     GPIO_PC(26)
#define FIIO_I2C1_DATA    GPIO_PC(27)
static void i2c1_gpio_init(void) {
    int ret = 0;
    ret = gpio_request(FIIO_I2C1_CLK, "i2c1 clk");
	if (ret) {
		printk(KERN_ERR "can's request i2c1 clk\n");
		return ret;
	}
     ret = gpio_request(FIIO_I2C1_DATA, "i2c1 sda");
	if (ret) {
		printk(KERN_ERR "can's request i2c1 sda\n");
		return ret;
	}
}

static void test_i2c1_clk() {
	int i =0;
	for(i = 0;i< 500;i++) {
		printk("%s %d i=%d\n",__func__,__LINE__,i);
		gpio_direction_output(FIIO_I2C1_CLK,1);
		udelay(100000);
		gpio_direction_output(FIIO_I2C1_CLK,0);
		udelay(100000);
	}
}

static void test_i2c1_sda() {
	int i =0;
	for(i = 0;i< 500;i++) {
		printk("%s %d i=%d\n",__func__,__LINE__,i);
		gpio_direction_output(FIIO_I2C1_DATA,1);
		udelay(100000);
		gpio_direction_output(FIIO_I2C1_DATA,0);
		udelay(100000);
	}
}
static ssize_t
i2c1_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 1;
}

//axp173_i2c_write_bit(axp173,0x12,1,3);
static ssize_t i2c1_gpio_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = 0;
	printk("i2c1_gpio_store write command:%s\n",buf);
	sscanf(buf, "%d",&value);
	printk("write value=%d\n",value);
	switch(value) {
		case 0:
			test_i2c1_clk();
		break;
		case 1:
			test_i2c1_sda();
		break;
	}
	return n;
}
//end
static DEVICE_ATTR(fiio_i2c1, S_IRUGO|S_IWUSR, i2c1_gpio_show, i2c1_gpio_store);
#endif

/////////////////////////////////////////

#ifdef CONFIG_SND_ASOC_FIIO_AK4376
static int ak4376_reg[] ={
// AKM_Register AK4376 20
	0x00 ,
	0x01 ,
 	0x02 ,
 	0x03 ,
 	0x04 ,
 	0x05 ,
 	0x06 ,
 	0x07 ,
 	0x0B ,
 	0x0C ,
 	0x0D ,
 	0x0E ,
 	0x0F ,
 	0x10 ,
 	0x11 ,
 	0x12 ,
 	0x13 ,
 	0x14 ,
 	0x15 ,
	0x24 
};
static ssize_t
ak4376_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i =0 ;
	return 1;
}
extern unsigned char AK4376_ReadOneByte(uint8 ReadAddr);
extern void AK4376_WriteOneByte(uint8 WriteAddr,uint8 DataToWrite);
extern void fiio_ak4376_poweron();
extern void fiio_ak4376_poweroff();
static ssize_t ak4376_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int event = simple_strtoul(buf, NULL, 0);
	int i =0 ;
	switch(event) {
		case 0:
			printk("\n");
			printk("2:use AK4376_ReadOneByte to get all reg value.\n");
			printk("3:use AK4376_WriteOneByte to write reg = 0x15, value=0x40.\n");
			printk("4:try to power on ak4376.\n");
			printk("5:try to power down ak4376.\n");
			printk("\n");
		break;
		case 2:
			
			printk("\n");
			for(i=0;i<sizeof(ak4376_reg)/sizeof(int);i++) {
				printk("reg=0x%02x value=0x%02x\n",ak4376_reg[i],AK4376_ReadOneByte(ak4376_reg[i]));
			}
			printk("\n");
			break;
		case 3:
			AK4376_WriteOneByte(0x15,0x40);
		break;
		case 4:
			fiio_ak4376_poweron();
		break;
		case 5:
		 	fiio_ak4376_poweroff();
		break;
		default:
			break;
	}
	return n;
}
#endif

#ifdef CONFIG_SND_ASOC_FIIO_AK4377
static int ak4377_reg[] ={
// AKM_Register AK4377 20
		0x00 ,
	0x01 ,
 	0x02 ,
 	0x03 ,
 	0x04 ,
 	0x05 ,
 	0x06 ,
 	0x07 ,
 	0x0B ,
 	0x0C ,
 	0x0D ,
 	0x0E ,
 	0x0F ,
 	0x10 ,
 	0x11 ,
 	0x12 ,
 	0x13 ,
 	0x14 ,
 	0x15 ,
	0x16 ,
 	0x17 ,
	0x18 ,
	0x19 ,
	0x1A ,
 	0x1B ,
 	0x1C ,
 	0x1D ,
 	0x1E ,
 	0x1F ,
 	0x20 ,
	0x24
};
static ssize_t
ak4377_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i =0 ;
	return 1;
}
extern unsigned char AK4377_ReadOneByte(uint8 ReadAddr);
extern void AK4377_WriteOneByte(uint8 WriteAddr,uint8 DataToWrite);
extern void fiio_ak4377_poweron();
extern void fiio_ak4377_poweroff();
extern void fiio_ak4377_dsd_mode();
extern void fiio_ak4377_pcm_mode();
extern void fiio_ak4377_powerup();
extern void fiio_ak4377_0db();
extern void fiio_ak4377_poweron_low_power();
extern void fiio_test_ak4377();
extern void AK4377_WriteOneByte_test(uint8 WriteAddr,uint8 DataToWrite);

static ssize_t ak4377_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int event = simple_strtoul(buf, NULL, 0);
	int i =0 ;
	switch(event) {
		case 0:
			printk("\n");
			printk("1:write and read back .\n");
			printk("2:use AK4377_ReadOneByte to get all reg value.\n");
			printk("3:use AK4377_WriteOneByte to write reg = 0x15, value=0x40.\n");
			printk("4:try to power on ak4377.\n");
			printk("5:try to power down ak4377.\n");
			printk("6:try to ak4377 to dsd mode.\n");
			printk("7:try to ak4377 to pcm mode.\n");
			printk("8:try to power up ak4377.\n");
			printk("9:try to set ak4377 0db output.\n");
			printk("10:power on ak4377 low power mode.\n");
			printk("11:set  ak4377 reg=0x0C value=0xAA.\n");
			printk("12:try to power on ak4377 only power up,don't init.\n");
			printk("\n");
		break;
		case 1:

		break;
		case 2:
			printk("\n");
			for(i=0;i<sizeof(ak4377_reg)/sizeof(int);i++) {
				printk("reg=0x%02x value=0x%02x\n",ak4377_reg[i],AK4377_ReadOneByte(ak4377_reg[i]));
				ndelay(10000);
			}
			printk("\n");
			break;
		case 3:
			AK4377_WriteOneByte(0x15,0x40);
		break;
		case 4:
			fiio_ak4377_poweron();
		break;
		case 5:
		 	fiio_ak4377_poweroff();
		break;
		case 6:
			fiio_ak4377_dsd_mode();
		break;
		case 7:
			fiio_ak4377_pcm_mode();
		break;
		case 8:
			fiio_ak4377_powerup();
		break;
		case 9:
			fiio_ak4377_0db();
			break;
		case 10:
			fiio_ak4377_poweron_low_power();
			break;
		case 11:
			fiio_test_ak4377();
			break;
		case 12:
			gpio_direction_output(PO_EN, 1);
			break;
		default:
			break;
	}
	return n;
}
static ssize_t
ak4377_rw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 1;
}
static ssize_t ak4377_rw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int reg = 0;
	int value = 0;
	printk("ak4377_rw_store write command:%s\n",buf);
	sscanf(buf, "%d %d", &reg,&value);
	if (value != 1000) {
		if (reg == 0x0C || reg == 0x0B) {
			AK4377_WriteOneByte_test(0x0B,value);
			AK4377_WriteOneByte_test(0x0C,value);
		}
		else {
			AK4377_WriteOneByte_test(reg,value);
		}
		
	}
	value = AK4377_ReadOneByte(reg);
	printk("read reg=%02xH value=%d\n",reg,value);
	return n;
}

//ak4377 dsd mode
extern void fiio_sys_set_fiio_dsd_mode(int dsd_type);
static ssize_t ak4377_dsd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int event = simple_strtoul(buf, NULL, 0);
	int i =0 ;
	switch(event) {
		case 0:
			printk("\n");
			printk("1:set ak4377 DSD64(2.8M) mode .\n");
			printk("\n");
		break;
		case 1:
			fiio_sys_set_fiio_dsd_mode(0);
		break;
		case 2:
			fiio_sys_set_fiio_dsd_mode(1);
			break;
		default:
			fiio_sys_set_fiio_dsd_mode(0);
			break;
	}
	return n;
}

//ak4377 digital volume adjust
static ssize_t ak4377_volume_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)

{
	int volume = simple_strtoul(buf, NULL, 0);
	if (volume > 0x3F) {
		volume = 0x3F;
	}
	AK4377_WriteOneByte(0x0C,volume);
	AK4377_WriteOneByte(0x0B,volume);
	return n;
}

void set_fiio_ak4377_work_mode(int mode);
static ssize_t ak4377_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int event = simple_strtoul(buf, NULL, 0);
	int i =0 ;
	switch(event) {
		case 0:
			printk("\n");
			printk("1:set ak4377 work high performance mode .\n");
			printk("2:set ak4377 work low power mode .\n");
			printk("\n");
		break;
		case 1:
			set_fiio_ak4377_work_mode(0);
		break;
		case 2:
			set_fiio_ak4377_work_mode(1);
			break;
		default:
			set_fiio_ak4377_work_mode(0);
			break;
	}
	return n;
}


#endif

//BT
#define FIIO_CSR8675
#ifdef FIIO_CSR8675

#ifndef CONFIG_FIIO_SOUND_BUILD_MODULE
#ifdef CONFIG_SND_ASOC_JZ_AIC_I2S_V13
extern void fiio_start_i2s();
extern void fiio_stop_i2s();
#endif
#endif
extern void save_port_b(void);
extern void set_i2s_do_to_gpio(void);
extern void set_i2s_do_tofunction(void);
extern void set_i2s_to_gpio_input(void);
extern void set_i2s_to_i2s_function(void);
extern void set_uart0_to_gpio(void);
extern void set_gpio_to_uart0(void);
#ifdef CONFIG_FIIO_IO_CTRL
extern void fiio_csr8675_reset();
#endif
extern void fiio_get_curent_io_state(void);

#define FIIO_I2S_DO     GPIO_PB(4)

int can_use_i2s_do = 0;
static int fiio_i2s_do_set_input() {

	int error = 0;
	if (can_use_i2s_do == 1) {
		gpio_free(FIIO_I2S_DO);
	}
	if (gpio_is_valid(FIIO_I2S_DO)) {
		error = gpio_request_one(FIIO_I2S_DO, GPIOF_IN, "fiio_i2s_do");
		if (error < 0) {
			printk("Failed to request GPIO %d, error %d\n",
				FIIO_I2S_DO, error);
			return error;
		}
		can_use_i2s_do = 1;
	}
	return 0;
}
static int fiio_i2s_do_set_input_high() {
#if 0
	int error = 0;
	if (can_use_i2s_do == 1) {
		gpio_free(FIIO_I2S_DO);
	}
	if (gpio_is_valid(FIIO_I2S_DO)) {
		error = gpio_request_one(FIIO_I2S_DO, GPIOF_INIT_HIGH, "fiio_i2s_do");
		if (error < 0) {
			printk("Failed to request GPIO %d, error %d\n",
				FIIO_I2S_DO, error);
			return error;
		}
		can_use_i2s_do = 1;
	}
#endif
return 0;
}
static void fiio_i2s_do_free() {
	gpio_free(FIIO_I2S_DO);
	can_use_i2s_do = 0;
}

static ssize_t
bt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i =0 ;
	return 1;
}

static ssize_t bt_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int event = simple_strtoul(buf, NULL, 0);
	int i =0 ;
	switch(event) {
		case 0:
			printk("\n");
			printk("1:reset bt(csr8675) low active.\n");
			printk("2:bt sink start i2s.\n");
			printk("3:bt sink stop i2s.\n");
			printk("4:set i2s port to input gpio.\n");
			printk("5:set i2s port to i2s function.\n");
			printk("6:set i2s port data out input.\n");
			printk("7:set i2s port data out to i2s function.\n");
			printk("8:set uart0 port to input gpio.\n");
			printk("9:set uart0 port to uart0 function.\n");
			printk("10:print current gpio port funtion.\n");
			printk("\n");
		break;
		case 1:
			#ifdef CONFIG_FIIO_IO_CTRL
				fiio_csr8675_reset();
			#endif
			break;
		case 2:
			#ifndef CONFIG_FIIO_SOUND_BUILD_MODULE
				#ifdef CONFIG_SND_ASOC_JZ_AIC_I2S_V13
					fiio_start_i2s();
					set_i2s_do_to_gpio();
				#endif
			#endif
				//printk("can't start i2s for sound card build module!\n");
			//#endif
			break;
		case 3:
			#ifndef CONFIG_FIIO_SOUND_BUILD_MODULE
				#ifdef CONFIG_SND_ASOC_JZ_AIC_I2S_V13
					fiio_stop_i2s();
					set_i2s_do_tofunction();
				#endif
			#else
				printk("can't stop i2s for sound card build module!\n");
			#endif
			break;
		case 4:
			set_i2s_to_gpio_input();
			break;
		case 5:
			set_i2s_to_i2s_function();
			break;
		case 6:
			//save_port_b();
			set_i2s_do_to_gpio();
			break;
		case 7:
			set_i2s_do_tofunction();
			break;
		case 8:
			set_uart0_to_gpio();
			break;
		case 9:
			set_gpio_to_uart0();
			break;
		case 10:
			fiio_get_curent_io_state();
			break;
		default:
			break;
	}
	return n;
}
#endif
/////////////////////////////////////////////
//////lcd
#ifdef CONFIG_FIIO_LCD_DEBUG
extern void fiio_send_lcd_cmd(unsigned long cmd);
extern void fiio_send_lcd_data(unsigned long data);
extern void fiio_set_lcd_rotate(int rotate);
static ssize_t lcd_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int cmd = simple_strtoul(buf, NULL, 0);
	fiio_send_lcd_cmd(cmd);
	return n;
}
static ssize_t lcd_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int data = simple_strtoul(buf, NULL, 0);
	fiio_send_lcd_data(data);
	return n;
}
static ssize_t lcd_rotate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int rotate = simple_strtoul(buf, NULL, 0);
	fiio_set_lcd_rotate(rotate);
	return n;
}
#endif
//////end lcd

//usb id
//GPIO_OTG_ID
#ifdef CONFIG_M5_BORAD_1_0629
#if 0
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
static void init_otg_id() {
	int ret = 0;
    ret = gpio_request(GPIO_OTG_ID, "GPIO_OTG_ID");
	if (ret) {
		printk(KERN_ERR "can's request GPIO_OTG_ID\n");
		return ret;
	}
	gpio_direction_output(GPIO_OTG_ID,1);
}

static ssize_t usb_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int id = simple_strtoul(buf, NULL, 0);
	gpio_direction_output(GPIO_OTG_ID,id);
	return n;
}
#endif
#endif

static DEVICE_ATTR(fiio_lcd, S_IRUGO|S_IWUSR, lcd_show, lcd_store);
static DEVICE_ATTR(fiio_axp192, S_IRUGO|S_IWUSR, axp_show, axp_store);
static DEVICE_ATTR(fiio_read_axp192, S_IWUSR, NULL, read_axp192_store);
#ifdef CONFIG_SND_ASOC_FIIO_AK4376
static DEVICE_ATTR(fiio_ak4376, S_IRUGO|S_IWUSR, ak4376_show, ak4376_store);
#endif

#ifdef CONFIG_SND_ASOC_FIIO_AK4377
static DEVICE_ATTR(fiio_ak4377, S_IRUGO|S_IWUSR, ak4377_show, ak4377_store);
static DEVICE_ATTR(fiio_ak4377_rw, S_IRUGO|S_IWUSR, ak4377_rw_show, ak4377_rw_store);
static DEVICE_ATTR(fiio_ak4377_dsd, S_IWUSR, NULL, ak4377_dsd_store);
static DEVICE_ATTR(fiio_ak4377_volume, S_IWUSR, NULL, ak4377_volume_store);
static DEVICE_ATTR(fiio_ak4377_mode, S_IWUSR, NULL, ak4377_mode_store);

#endif

#ifdef FIIO_CSR8675
static DEVICE_ATTR(fiio_bt, S_IRUGO|S_IWUSR, bt_show, bt_store);
#endif
#ifdef CONFIG_FIIO_LCD_DEBUG
static DEVICE_ATTR(fiio_lcd_cmd, S_IWUSR, NULL, lcd_cmd_store);
static DEVICE_ATTR(fiio_lcd_data, S_IWUSR, NULL, lcd_data_store);
static DEVICE_ATTR(fiio_lcd_rotate, S_IWUSR, NULL, lcd_rotate_store);
#endif

#ifdef CONFIG_M5_BORAD_1_0629
static DEVICE_ATTR(fiio_usb_id, S_IWUSR, NULL, usb_id_store);
#endif

static struct attribute *debug_attrs[] = {
	&dev_attr_fiio_lcd.attr,
	&dev_attr_fiio_axp192.attr,
	&dev_attr_fiio_read_axp192.attr,
#ifdef FIIO_I2C1_GPIO_TEST
	&dev_attr_fiio_i2c1.attr,
#endif
#ifdef CONFIG_SND_ASOC_FIIO_AK4376
	&dev_attr_fiio_ak4376.attr,
#endif
#ifdef CONFIG_SND_ASOC_FIIO_AK4377
	&dev_attr_fiio_ak4377.attr,
	&dev_attr_fiio_ak4377_rw.attr,
	&dev_attr_fiio_ak4377_dsd.attr,
	&dev_attr_fiio_ak4377_volume.attr,
	&dev_attr_fiio_ak4377_mode.attr,
#endif
#ifdef FIIO_CSR8675
	&dev_attr_fiio_bt.attr,
#endif
#ifdef CONFIG_FIIO_LCD_DEBUG
	&dev_attr_fiio_lcd_cmd.attr,
	&dev_attr_fiio_lcd_data.attr,
	&dev_attr_fiio_lcd_rotate.attr,
#endif
#ifdef CONFIG_M5_BORAD_1_0629
	&dev_attr_fiio_usb_id.attr,
#endif
	NULL,
};

///sys/kernel/fiio/debug/fiio_lcd
///sys/kernel/fiio/debug/fiio_axp192
static const char debug_group_name[] = "debug";
static struct attribute_group debug_attr_group = {
	.name	= debug_group_name,
	.attrs	= debug_attrs,
};


static int __init debug_init(void)
{
	int ret, ret2;
	printk("%s %d\n",__func__,__LINE__);
	int retval;
	debug_kobj = kobject_create_and_add("fiio", kernel_kobj);
    if (!debug_kobj)
        return -ENOMEM;
	retval = sysfs_create_group(debug_kobj, &debug_attr_group);
    if (retval)
        kobject_put(debug_kobj);
	#ifdef FIIO_I2C1_GPIO_TEST
		i2c1_gpio_init();
	#endif

	//
	#if 0
		#ifdef CONFIG_M5_BORAD_1_0629
		init_otg_id();
		#endif
	#endif
	return ret;
}

static void __exit debug_exit(void)
{
	printk("%s %d\n",__func__,__LINE__);
}

module_init(debug_init);
module_exit(debug_exit);

MODULE_DESCRIPTION("Generic Debug Interface.");
MODULE_AUTHOR("pengweizhong@fiio.net");
MODULE_LICENSE("GPL");
