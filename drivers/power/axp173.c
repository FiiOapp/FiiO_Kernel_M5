/*
 * axp173.c
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
#include <linux/interrupt.h> 
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <soc/gpio.h>
#include <linux/spinlock.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/pmu.h>
#include <linux/skytc/eq/eq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/power/axp173.h>
#include <linux/delay.h>

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
#define ADC_SAMPLE_RATE        0.1

#if 0
//debug
#define FIIO_DEBUG_AXP173
#ifdef FIIO_DEBUG_AXP173
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_axp173] " x)
#else
#define fiio_debug(x...)
#endif
#endif

static int fiio_m5_send_key75_flag = 0;
module_param(fiio_m5_send_key75_flag, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(fiio_m5_send_key75_flag, "FiiO Send Key Code 75 flag");

static int fiio_axp192_debug = 0;
module_param(fiio_axp192_debug, int, 0644);
#define fiio_debug(msg...)			\
	do {					\
		if (fiio_axp192_debug)		\
			printk("axp192: " msg);	\
	} while(0)
	
#define OCVREG6                6        //3.1328
#define OCVREG0                0        //3.2736
#define OCVREG1                1        //3.5000
#define OCVREG2                2        //3.5552
#define OCVREG3                3        //3.6256
#define OCVREG4                4        //3.6608
#define OCVREG5                5        //3.6960

/*AXP19 初始化开路电压*/
/*
    只针对AXP19，可以改变，注意和上表的剩余电量百分比一一对应
*/
#define OCVVOL0                3300
#define OCVVOL1                3450
#define OCVVOL2                3580
#define OCVVOL3                3680
#define OCVVOL4                3780
#define OCVVOL5                3880

#define  uint8   unsigned char
#define  uint16  unsigned short
#define  int16   short
#define  uint32  unsigned int
#define  int32   int
#define  uint64  unsigned long

typedef enum charger_type_t {
    CHARGER_BATTERY = 0,
    CHARGER_USB,
    CHARGER_AC
}charger_type_t;

struct emxx_battery_data {
    struct power_supply battery;
    struct power_supply usb;
    struct power_supply ac;


    unsigned int battery_present;
    uint16 voltage_level;
    uint16 battery_temp;
    uint8 headset_inser;
    unsigned char usb_charge_state;
    uint16 line_key;
    bool usb_detect;
    bool gadget_sign; 
    uint8 battery_full_sign;
    unsigned int battery_voltage;
    
    charger_type_t charger;
    int usb_state;

    struct workqueue_struct *monitor_wqueue;
    struct delayed_work monitor_work;
};

//AXP173 IRQ
struct workqueue_struct *irq_wqueue;
struct delayed_work irq_work;


static struct emxx_battery_data *battery_data;
unsigned int battery_vol;
static unsigned int battery_number=1050;
static unsigned int delay_number,usb_delay;
bool m3k_usb_det=0;
bool m3k_gadget_sign=0;
EXPORT_SYMBOL(m3k_usb_det);
EXPORT_SYMBOL(m3k_gadget_sign);


static enum power_supply_property emxx_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,    //电量
    POWER_SUPPLY_PROP_TEMP,            //温度
    POWER_SUPPLY_PROP_VOLTAGE_NOW,    //电压
    //POWER_SUPPLY_PROP_HEADSET_INSER,
};

static enum power_supply_property emxx_power_props[] = {
    POWER_SUPPLY_PROP_ONLINE, //外部供电查看是否存在AC或usb
};




static char driver_name[] = "axp173";	
struct axp173 {
	struct device *dev;
	//struct mutex io_lock;
    int	irq;	/* if 0, use polling */
	//spinlock_t              io_lock;	
	struct i2c_client *i2c;
	int num_regulators;
	struct regulator_dev **rdev;
};

static int resume_sign=0,resume_count=0;
struct axp173 *axp173;
static unsigned int headset_inser_sign;
unsigned int get_headset_insert_state(void) {
	return headset_inser_sign;
}
EXPORT_SYMBOL(get_headset_insert_state);

int axp173_i2c_read_bit(struct axp173 *bq, u8 reg, u8 bit);
int axp173_i2c_read(struct axp173 *bq, u8 reg);
int axp173_i2c_write_bit(struct axp173 *bq, u8 reg, bool val, u8 bit);
int axp173_i2c_write(struct axp173 *bq, u8 reg, u8 val);

static int emxx_battery_get_status(void)
{
    int ret;

    switch (battery_data->charger) {
    case CHARGER_BATTERY:
	//printk("no charger!\n");
        ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
        break;
    case CHARGER_USB:
    case CHARGER_AC:
	if(battery_data->battery_full_sign)
	{  ret = POWER_SUPPLY_STATUS_FULL;//如果电池充满了则显示full状态
           //printk("battery full!\n");
	   //axp173_i2c_write_bit(axp173,0x41,1,2);
	}
	 else{
            ret = POWER_SUPPLY_STATUS_CHARGING; //未满并且外部电源存在则显示充电状态，人为设置为CHARGER_AC
       	#ifdef CONFIG_FIIO_DEBUG_ADB_AXP
	    printk("battery charging!\n");
	#endif
		}
	 break;
    default:
        ret = POWER_SUPPLY_STATUS_UNKNOWN;
    }
    return ret;
}    	
	
static int emxx_battery_get_property(struct power_supply *psy,
                 enum power_supply_property psp,
                 union power_supply_propval *val)
{
    int ret = 0;

    switch (psp) {
		
    case POWER_SUPPLY_PROP_STATUS:        // 0 
        val->intval = emxx_battery_get_status();
        break;
        
    case POWER_SUPPLY_PROP_HEALTH:        // 1 
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:        // 2 
        val->intval = battery_data->battery_present;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:    // 4 
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
        
    case POWER_SUPPLY_PROP_CAPACITY:    // 26 
        val->intval = battery_data->voltage_level;
        break;    
        
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = battery_data->battery_temp;
        break;
        
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:    
        val->intval = battery_data->battery_voltage;
        break;
    //case POWER_SUPPLY_PROP_HEADSET_INSER:
       // val->intval = battery_data->headset_inser;
       //     break;
        default:
        ret = -EINVAL;
        break;
    }

    return ret;
}	
		
/* read value from register */
int axp173_i2c_read(struct axp173 *bq, u8 reg)
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
//EXPORT_SYMBOL(axp173_i2c_read);

/* write value to register */
int axp173_i2c_write(struct axp173 *bq, u8 reg, u8 val)
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

/* read value from register, change it with mask left shifted and write back */
static int axp173_i2c_write_mask(struct axp173 *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = axp173_i2c_read(bq, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return axp173_i2c_write(bq, reg, ret);
}

/* read value from register, apply mask and right shift it */
static int axp173_i2c_read_mask(struct axp173 *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = axp173_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}


/* read value from register and return one specified bit */
int axp173_i2c_read_bit(struct axp173 *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return axp173_i2c_read_mask(bq, reg, BIT(bit), bit);
}
//EXPORT_SYMBOL(axp173_i2c_read_bit);

/* change only one bit in register */
int axp173_i2c_write_bit(struct axp173 *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return axp173_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}


/*根据电压获取当前电量，此算法简单实用，根据不同电池需调节最大电压，
 * 最小电压以及每段的电压及对应的电量，可达到充电快慢的程度，实际中
 * 需要通过内阻，开路电压及充放电电流按一定算法获取后并经过库仑计校
 * 准得到电量，实际电量的计算是很复杂的，往往无法达到理想的检测效果*/
static uint8_t axp_restcap(int ocv)
{
    if(ocv >= OCVVOL5)
    {
        return OCVREG5;
    }
    else if(ocv < OCVVOL0)
    {
        return OCVREG6;
    }
    else if(ocv < OCVVOL1)
    {
        return OCVREG0;
    }
    else if(ocv < OCVVOL2)
    {
        return OCVREG1;
    }
    else if(ocv < OCVVOL3)
    {
        return OCVREG2;
    }
    else if(ocv < OCVVOL4)
    {
        return OCVREG3;
    }
    else if(ocv < OCVVOL5)
    {
        return OCVREG4;
    }
    else
    {
        return 0;
    }
}

#define LPO_DET_IO  (0x96)
#define LPO_DET_IO_BIT  (4)
//#define CHECK_HEADSET_USE_ADC 1
#ifdef CHECK_HEADSET_USE_ADC
static int check_headset_state(void) {

    int h_8 = axp173_i2c_read(axp173,0x6A)&0xff;
    int l_4 = axp173_i2c_read(axp173,0x6B)&0x0f;
    //step 0.5mv
    int vol = ((h_8&0xf0 >> 4)*256 + (h_8&0x0f)*16+l_4 )*5/10;
    //fiio_debug("%s vol=%d h_8=0x%02x l4=0x%02x\n",__func__,vol,h_8,l_4);
    if (vol > 100) {
        return 1;
    }
    else {
        return 0;
    }
    return 0;
}
#endif
static int g_first_boot_system_flag = 0;
static void emxx_battery_work(struct work_struct *work)
{
   	const int interval = HZ * ADC_SAMPLE_RATE;
	unsigned int usb_voltage;
    unsigned int old_voltage_level;
    unsigned int old_battery_voltage;
    unsigned int old_battery_charger;
    bool old_usb_det,old_gadget_sign;
    old_voltage_level = battery_data->voltage_level;
	old_battery_voltage = battery_data->battery_voltage;
  	old_battery_charger = battery_data->charger;
	old_usb_det = battery_data->usb_detect;
	old_gadget_sign = battery_data->gadget_sign;

   battery_data->usb_detect = m3k_usb_det;
   battery_data->gadget_sign = m3k_gadget_sign;
   battery_number --;
   if((battery_number>50)||(battery_number<=1))
	{
    battery_data->voltage_level=axp_restcap((axp173_i2c_read(axp173, 0x78)*16+axp173_i2c_read(axp173, 0x79))*11/10+17);
    battery_data->battery_voltage=(axp173_i2c_read(axp173, 0x78)*16+axp173_i2c_read(axp173, 0x79))*11/10 +17;
	}
	//headset inser detect
	if (battery_number<=1045) {
        #ifndef CHECK_HEADSET_USE_ADC
        if(axp173_i2c_read_bit(axp173,LPO_DET_IO,LPO_DET_IO_BIT) && (0 == g_first_boot_system_flag)) {
        #else
        if(check_headset_state() == 1 && (0 == g_first_boot_system_flag)) {
        #endif
			fiio_debug("\nheadset inser 000 set headset_state to 1!\n");
			battery_data->headset_inser = 1;
		}
	}
	if (battery_number <= 1040) {
		g_first_boot_system_flag = 1;
	}
	if(battery_number<=1000)
	{
        #ifndef CHECK_HEADSET_USE_ADC
        if(axp173_i2c_read_bit(axp173,LPO_DET_IO,LPO_DET_IO_BIT)&&(headset_inser_sign==0))
        #else
        if(check_headset_state() == 1&&(headset_inser_sign==0))
        #endif
            {
	    
            printk("headset inser!\n");
            input_event(headset,EV_KEY,KEY_SCROLLLOCK, 1);
                        input_sync(headset);
            battery_data->headset_inser = 1;
            headset_inser_sign=1;
            }
        #ifndef CHECK_HEADSET_USE_ADC
           else if(!axp173_i2c_read_bit(axp173,LPO_DET_IO,LPO_DET_IO_BIT)&&(headset_inser_sign==1))
        #else
            else if(check_headset_state() == 0&&(headset_inser_sign==1))
        #endif

	    {
            printk("headset remove!\n");
            input_event(headset,EV_KEY,KEY_SCROLLLOCK, 0);
                        input_sync(headset);
            battery_data->headset_inser = 0;
            headset_inser_sign=0;
            }
	}
	/*判断是否有在充电*/

	  if((axp173_i2c_read_bit(axp173,0x0,2))||(axp173_i2c_read_bit(axp173,0x0,5)))
	  {
		if(battery_data->charger==CHARGER_BATTERY){	
		if(m3k_usb_det){
		battery_data->charger=CHARGER_USB;
		power_supply_changed(&battery_data->usb);
		power_supply_changed(&battery_data->battery);
		axp173_i2c_write_bit(axp173,0x30,1,7);
		axp173_i2c_write_bit(axp173,0x30,0,1);
		fiio_debug("try to set charger 450mA\n");
		axp173_i2c_write(axp173,0x33,0xE4);
		battery_data->usb_charge_state=4;
		}
		else	{
		battery_data->charger=CHARGER_AC;
		power_supply_changed(&battery_data->ac);
		power_supply_changed(&battery_data->battery);
		axp173_i2c_write_bit(axp173,0x30,1,7);
                axp173_i2c_write_bit(axp173,0x30,0,1);
		//printk("try to set charger 780mA\n");
                //axp173_i2c_write(axp173,0x33,0xc6);
		fiio_debug("try to set charger 450mA\n");
                axp173_i2c_write(axp173,0x33,0xE4);
		battery_data->usb_charge_state=4;
			}
		}
	    }
	else if(!axp173_i2c_read_bit(axp173,0x0,2)){
		if(battery_data->charger!=CHARGER_BATTERY){
		battery_data->charger=CHARGER_BATTERY;
		power_supply_changed(&battery_data->ac);
		power_supply_changed(&battery_data->battery);
		m3k_usb_det=0;
		}	
	}
	//防止在没充电的情况下，电量回升
	if(battery_data->charger==CHARGER_BATTERY)
	{ 	
		if(battery_data->voltage_level<6){
			if(battery_data->voltage_level > old_voltage_level)	
				{
					battery_data->voltage_level = old_voltage_level;
				}

		}
	}
	
	if((battery_data->charger==CHARGER_AC)||(battery_data->charger==CHARGER_USB))
	{
	usb_delay++;
	if(usb_delay>=10)
		{
			usb_voltage=axp173_i2c_read(axp173, 0x5A)*16+axp173_i2c_read(axp173, 0x5B);
                        if((usb_voltage>2647)&&(battery_data->usb_charge_state<4))
			{
				battery_data->usb_charge_state++;
				fiio_debug("VBUS voltage=%d\n",usb_voltage);
				fiio_debug("try to  charger current:%d\n",battery_data->usb_charge_state);
				axp173_i2c_write_bit(axp173,0x30,1,7);
				axp173_i2c_write_bit(axp173,0x30,0,1);
				axp173_i2c_write(axp173,0x33,0xE0+battery_data->usb_charge_state);
				fiio_debug("try to read 0x33:%d\n",axp173_i2c_read(axp173,0x33));
			}
			else if((usb_voltage<2616)&&(battery_data->usb_charge_state>0))
			{
				battery_data->usb_charge_state--;
				fiio_debug("VBUS voltage=%d\n",usb_voltage);
				fiio_debug("try to  charger current:%d\n",battery_data->usb_charge_state);
				axp173_i2c_write_bit(axp173,0x30,1,7);
				axp173_i2c_write_bit(axp173,0x30,0,1);
				axp173_i2c_write(axp173,0x33,0xE0+battery_data->usb_charge_state); 
				fiio_debug("try to read 0x33:%d\n",axp173_i2c_read(axp173,0x33));              
            }
			usb_delay=0;
		}
		if(axp173_i2c_read_bit(axp173,0x45,2)&&(!battery_data->battery_full_sign))
		{
			//printk("start charger full\n");
			if(axp_restcap((axp173_i2c_read(axp173, 0x78)*16+axp173_i2c_read(axp173, 0x79))*11/10+17) == OCVREG5 ){
			battery_data->battery_full_sign=1;
			power_supply_changed(&battery_data->battery);	
			}
			//else {
			//printk("========3======\n");
			//power_supply_changed(&battery_data->battery);
			//}	
		}else if((axp173_i2c_read_bit(axp173,0x45,3))&&(battery_data->battery_full_sign))
		{	//printk("start chargering\n");
			battery_data->battery_full_sign=0;
			power_supply_changed(&battery_data->battery);
			
		}
	}
        //当电脑设备发生变化，数据上报*/
	if(old_usb_det!=battery_data->usb_detect)
	{
	 if(m3k_usb_det){
	 battery_data->charger=CHARGER_USB;
	 //axp173_i2c_write_bit(axp173,0x30,1,7);
            //    axp173_i2c_write_bit(axp173,0x30,0,1);
              //  printk("try to set charger 450mA\n");
                //axp173_i2c_write(axp173,0x33,0xc4);
	}	
	power_supply_changed(&battery_data->usb);
	}
	if(battery_data->charger==CHARGER_AC){
		if(battery_data->gadget_sign)
		delay_number++;
	    if(delay_number>=50)
	    {
			if(m3k_usb_det==0) {
				fiio_debug("can not found a PC\n");
				power_supply_changed(&battery_data->usb);
			}
			delay_number=0;
			battery_data->gadget_sign =0;
			m3k_gadget_sign=0;
	    }
	}


     //当电压，温度，电量任意一个发生改变时，数据上报*/
	if(battery_number>50){
       	if((battery_number ==1021)||(battery_number ==1011)||(battery_number ==1001) \
	   	||(battery_number ==691)||(battery_number ==281)||(battery_number ==171) \
	   	||(battery_number ==61))
		power_supply_changed(&battery_data->battery);
	}
	if(battery_number <=1)	{
	{
	 //if (old_voltage_level != battery_data->voltage_level || old_battery_voltage != battery_data->battery_voltage ) {
        if (old_voltage_level != battery_data->voltage_level)   
	 power_supply_changed(&battery_data->battery);
        }
        battery_number = 50;
    }
    

	if(resume_sign)
	{
		resume_count++;
		if(resume_count>=10) { //唤醒后发现有耳机插入重发一次
             #ifndef CHECK_HEADSET_USE_ADC
            if(axp173_i2c_read_bit(axp173,LPO_DET_IO,LPO_DET_IO_BIT) &&(headset_inser_sign==1))
            #else
                if(check_headset_state() == 1 &&(headset_inser_sign==1))
            #endif
            {
            	fiio_debug("headset inser 002!\n");
            	input_event(headset,EV_KEY,KEY_SCROLLLOCK, 1);
                input_sync(headset);
            }
			fiio_debug("resume event :key code 75\n");
        	input_event(headset,EV_KEY, KEY_KP4,1);
            input_event(headset,EV_KEY,KEY_KP4, 0);
            input_sync(headset);
			resume_sign=0;
			resume_count=0;
			fiio_m5_send_key75_flag = 1;
			//
		}
	}
	queue_delayed_work(battery_data->monitor_wqueue,&(battery_data->monitor_work), interval);
}


static int emxx_power_get_property(struct power_supply *psy,
            enum power_supply_property psp,
            union power_supply_propval *val)
{
    charger_type_t charger;

    charger = battery_data->charger;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:    /* 3 */
        if (psy->type == POWER_SUPPLY_TYPE_MAINS)
            val->intval = (charger ==  CHARGER_AC ? 1 : 0);
        else if (psy->type == POWER_SUPPLY_TYPE_USB) {
            val->intval = m3k_usb_det;
        } else
            val->intval = 0;
        break;

    default:
        return -EINVAL;
    }

    return 0;
}

static int emxx_battery_probe(struct platform_device *pdev)
{
    int ret;
    struct emxx_battery_data *data;

    printk("Battery probe...\n");
    data = kzalloc(sizeof(*data), GFP_KERNEL);
    if (data == NULL) {
        ret = -ENOMEM;
        goto err_data_alloc_failed;
    }

    /* Battey */
  /*填充struct power_supply battery*/
    data->battery.name = "battery";
    data->battery.type = POWER_SUPPLY_TYPE_BATTERY;
    data->battery.properties = emxx_battery_props; //添加相应字段到battery目录下，包括电压，电量及温度。
    data->battery.num_properties = ARRAY_SIZE(emxx_battery_props);
    data->battery.get_property = emxx_battery_get_property; //注册回调函数，以此动态获取属性

    /* USB */
    data->usb.name = "usb";
    data->usb.type = POWER_SUPPLY_TYPE_USB;
    data->usb.properties = emxx_power_props;
    data->usb.num_properties = ARRAY_SIZE(emxx_power_props);
    data->usb.get_property = emxx_power_get_property;

     /* AC */
    data->ac.name = "ac";
    data->ac.type = POWER_SUPPLY_TYPE_MAINS;
    data->ac.properties = emxx_power_props;
    data->ac.num_properties = ARRAY_SIZE(emxx_power_props);
    data->ac.get_property = emxx_power_get_property;

    battery_data = data;    
   /*初始化电池电压为3700mv，温度为30*/
    battery_vol = 3700;
    battery_data->battery_temp = 30;
    battery_data->voltage_level = 5;
    battery_data->usb_charge_state=4;
    battery_data->charger = CHARGER_AC;
    ret = power_supply_register(&pdev->dev, &data->battery);
    if (ret)
    {
        printk("failed to register battery:%d\n",ret);
        return ret;
	}
    ret = power_supply_register(&pdev->dev, &data->usb);
    if (ret)
    {
	printk("failed to register usb:%d\n",ret);
        return ret;
	}

    ret = power_supply_register(&pdev->dev, &data->ac);
    if (ret)
    {
	printk("failed to register ac:%d\n",ret);
        return ret;
	}
    INIT_DELAYED_WORK(&data->monitor_work, emxx_battery_work);
    data->monitor_wqueue =
        create_singlethread_workqueue("axp192");
    if (!data->monitor_wqueue) {
        ret = -ESRCH;
        goto err_workqueue_failed;
    }
	
    platform_set_drvdata(pdev, data);

    queue_delayed_work(data->monitor_wqueue, &data->monitor_work, HZ);
	
    return 0;

err_workqueue_failed:
    power_supply_unregister(&data->battery);
err_data_alloc_failed:

    return ret;
}

static int emxx_battery_remove(struct platform_device *pdev)
{
    struct emxx_battery_data *data = platform_get_drvdata(pdev);

    printk(KERN_INFO "Battery driver remove...n");

    power_supply_unregister(&data->battery);

    kfree(data);
    battery_data = NULL;
    return 0;
}
static struct delayed_work	update_work;
static struct workqueue_struct *update_workqueue = NULL;
static void update_core_work(struct work_struct *work) 
{
	fiio_debug("Enter:+++++++++++++++++++++++++update_core_work v1.225++++++++++++++++++++++++\n");
	axp173_i2c_write(axp173,0x23,21);
	fiio_debug("Exit:+++++++++++++++++++++++++update_core_work v1.225++++++++++++++++++++++++\n");
}
static int  axp173_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{

	//struct axp173_platform_data *pdata = i2c->dev.platform_data;
	int ret;
	unsigned char recvbuf;
	unsigned char sendbuf;
	fiio_debug("+++++++++++++++++++++++++Enter axp173_i2c_probe++++++++++++++++++++++++\n");
	
	axp173 = kzalloc(sizeof(struct axp173),GFP_ATOMIC);//GFP_KERNEL);//
    if (axp173 == NULL) {
        ret = -ENOMEM;
        goto err;
    }

	axp173->i2c = i2c;
	axp173->dev = &i2c->dev;
	i2c_set_clientdata(i2c, axp173);

	ret=axp173_i2c_read_bit(axp173,0x01,5);
	if ((ret < 0) || (ret == 0xff)){
		printk(KERN_ERR"The device is not act:axp173 \n");
		return 0;
	}
    printk(KERN_INFO"axp173 charge control reg 1 is %x\n",ret);

    //充电使能 4.2v 充电电流小于10，停止充电 450mA充电电流
    axp173_i2c_write(axp173,0x33,0xE4);
    printk("reg:0x33 = %d\n",axp173_i2c_read(axp173,0x33));
    printk("reg:0x34 = %d\n",axp173_i2c_read(axp173,0x34));
	
    printk(KERN_INFO"set VBUS-IPSOUT select!\n");
    axp173_i2c_write_bit(axp173,0x30,1,7);
    axp173_i2c_write_bit(axp173,0x30,0,1);

    printk(KERN_INFO"set LDO=3.3V LDO=3.3V");
    axp173_i2c_write(axp173,0x28,0xff);
    printk(KERN_INFO"open LDO2\n");
    axp173_i2c_write_bit(axp173,0x12,1,2);
    printk(KERN_INFO"open LDO3\n");
    axp173_i2c_write_bit(axp173,0x12,1,3);
    printk(KERN_INFO"open EXTEN\n");
	
    printk(KERN_INFO"GPIO4 output GPIO3 ADC\n");
    //axp173_i2c_write_bit(axp173,0x95,1,7);
    //axp173_i2c_write(axp173,0x95,0x87);
	
    printk(KERN_INFO"VBUS voltage ADC enable\n");
    axp173_i2c_write_bit(axp173,0x82,1,3);
    //power on time 2S
    axp173_i2c_write_bit(axp173,0x36,1,7);
    axp173_i2c_write_bit(axp173,0x36,0,6);

	  axp173_i2c_write_bit(axp173,0x36,1,1);
    axp173_i2c_write_bit(axp173,0x36,1,0);
	
    //printk(KERN_INFO"CHGLED control by 32Hbit[5:4]\n");
    //axp173_i2c_write_bit(axp173,0x32,1,3);
    printk(KERN_INFO"battery inser irq enable\n");
    printk(KERN_INFO"battery inset irq status=%d\n",axp173_i2c_read_bit(axp173,0x41,7));
    axp173_i2c_write_bit(axp173,0x41,0,7);
    printk(KERN_INFO"battery remove irq enable\n");
    printk(KERN_INFO"battery remove irq status=%d\n",axp173_i2c_read_bit(axp173,0x41,6));
    axp173_i2c_write_bit(axp173,0x41,0,6);
    printk(KERN_INFO"axp192 gpio0 & 3 adc input\n");
    axp173_i2c_write_bit(axp173,0x83,1,0);
    axp173_i2c_write_bit(axp173,0x83,1,3);
    axp173_i2c_write_bit(axp173,0x85,0,0);
    axp173_i2c_write_bit(axp173,0x85,1,3);

    //GPIO ADC IN
    axp173_i2c_write(axp173,0x90,0x04);
   // axp173_i2c_write(axp173,0x86,0x89);
   // axp173_i2c_write(axp173,0x87,0x0);

   	//M3K GPIO2 LPO_DET
    //axp173_i2c_write(axp173,0x93,0x01);
     #ifndef CHECK_HEADSET_USE_ADC
    //IO
    axp173_i2c_write(axp173,0x95,0x8E);
    #else
    //ADC
    axp173_i2c_write(axp173,0x95,0x8F);
    #endif
    axp173_i2c_write_bit(axp173,0x4A,1,2);
    
    //axp173_i2c_write_bit(axp173,0x80,1,2);
    //axp173_i2c_write_bit(axp173,0x80,1,3);
    //printk(KERN_INFO"DCDC2 PWM mode=%d\n",axp173_i2c_read_bit(axp173,0x80,2));
    //printk(KERN_INFO"DCDC3 PWM mode=%d\n",axp173_i2c_read_bit(axp173,0x80,3));
    recvbuf= axp173_i2c_read(axp173,0x80);
    recvbuf |=0x0C;
    sendbuf=recvbuf;
    axp173_i2c_write(axp173,0x80,sendbuf);
    printk(KERN_INFO"DCDC PWM mode=%x\n",axp173_i2c_read(axp173,0x80));
	
	//mdelay(1000);
	axp173_i2c_write(axp173,0x23,28);
	//�������2���Ӻ󣬽��˵�ѹ��1.225v
	INIT_DELAYED_WORK(&update_work, update_core_work);
	update_workqueue = create_workqueue("update_workqueue");
	queue_delayed_work(update_workqueue, &update_work, 6000); //60s
	fiio_debug("----------------Exit axp173_i2c_probe------------------\n");
	return 0;

err:
	return ret;	

}

static ssize_t headset_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
	return 0;
}

static ssize_t axp173_shutdown_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	printk("begin shut down the axp173");
	ret = axp173_i2c_write_bit(axp173,0x32,1,7);
	return count;
}


static ssize_t headset_state_show(struct device *device, struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", battery_data->headset_inser);
}	
////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t axp173_dc1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = axp173_i2c_read(axp173,0x80);
	int ret = simple_strtoul(buf, NULL, 0);
	if (ret == 1) {
		printk("DC-DC1 want to set PWM mode\n");
		value |= 0x08;
	}
	else {
		printk("DC-DC1 want to set PFM/PWM mode\n");
		value &= 0xF7;
	}
   	
   	axp173_i2c_write(axp173,0x80,value);
   	printk("DCDC1 work mode = %s\n",(axp173_i2c_read_bit(axp173,0x80,3)==1)?"PWM":"PFM/PWM");
	return count;
}
static ssize_t axp173_dc1_show(struct device *device, struct device_attribute *attr, char *buf)
{
	printk("DC-DC1 work mode = %s\n",(axp173_i2c_read_bit(axp173,0x80,3)==1)?"PWM":"PFM/PWM");
	return 1;
}	

static ssize_t axp173_dc2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = axp173_i2c_read(axp173,0x80);
	int ret = simple_strtoul(buf, NULL, 0);
   	if (ret == 1) {
		printk("DC-DC2 want to set PWM mode\n");
		value |= 0x04;
	}
	else {
		printk("DC-DC2 want to set PFM/PWM mode\n");
		value &= 0xFB;
	}
   	axp173_i2c_write(axp173,0x80,value);
   	printk("DC-DC2 work mode = %s\n",(axp173_i2c_read_bit(axp173,0x80,2)==1)?"PWM":"PFM/PWM");
	return count;
}
static ssize_t axp173_dc2_show(struct device *device, struct device_attribute *attr, char *buf)
{
	printk("DC-DC2 work mode = %s\n",(axp173_i2c_read_bit(axp173,0x80,2)==1)?"PWM":"PFM/PWM");
	return 1;
}

static ssize_t axp173_dc3_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = axp173_i2c_read(axp173,0x80);
	int ret = simple_strtoul(buf, NULL, 0);
   	if (ret == 1) {
		printk("DC-DC3 want to set PWM mode\n");
		value |= 0x04;
	}
	else {
		printk("DC-DC3 want to set PFM/PWM mode\n");
		value &= 0xFD;
	}
   	axp173_i2c_write(axp173,0x80,value);
   	printk("DC-DC3 work mode = %s\n",(axp173_i2c_read_bit(axp173,0x80,1)==1)?"PWM":"PFM/PWM");
	return count;
}
static ssize_t axp173_dc3_show(struct device *device, struct device_attribute *attr, char *buf)
{
	printk("DC-DC3 work mode = %s\n",(axp173_i2c_read_bit(axp173,0x80,1)==1)?"PWM":"PFM/PWM");
	return 1;
}
static ssize_t axp173_wreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value= 0, reg = 0;
	sscanf(buf, "%d %d", &reg,&value);
	printk("want to write reg=%02xH[%d] value=%02xH[%d]\n",reg,reg,value,value);
   	axp173_i2c_write(axp173,reg,value);
	value = axp173_i2c_read(axp173,reg);
   	printk("read from reg=%02xH[%d] value=%02xH[%d]\n",reg,reg,value,value);
	return count;
}

static ssize_t axp173_rreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value= 0, reg = 0;
	sscanf(buf, "%d", &reg);
	value = axp173_i2c_read(axp173,reg);
   	printk("read from reg=%02xH[%d] value=%02xH[%d]\n",reg,reg,value,value);
	return count;
}

static struct device_attribute axp173_device_attrs[] = {
	//__ATTR(reg, S_IRUGO | S_IWUSR, reg_show, reg_store),
	__ATTR(headset_state, S_IRUGO | S_IWUSR, headset_state_show, headset_state_store),
	__ATTR(axp173_shutdown, S_IWUSR, NULL, axp173_shutdown_store),
	__ATTR(axp173_dc1, S_IRUGO | S_IWUSR, axp173_dc1_show, axp173_dc1_store),
	__ATTR(axp173_dc2, S_IRUGO | S_IWUSR, axp173_dc2_show, axp173_dc2_store),
	__ATTR(axp173_dc3, S_IRUGO | S_IWUSR, axp173_dc3_show, axp173_dc3_store),
	__ATTR(axp173_wreg, S_IWUSR, NULL, axp173_wreg_store),
	__ATTR(axp173_rreg, S_IWUSR, NULL, axp173_rreg_store),
	//__ATTR(test_eq, S_IRUGO | S_IWUSR, test_eq_show, test_eq_store),
};

static int axp173_device_attr_register(struct miscdevice  *axp173_misc_opt)
{
	int error = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(axp173_device_attrs); i++) {
		error = device_create_file(axp173_misc_opt->this_device, &axp173_device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(axp173_misc_opt->this_device, &axp173_device_attrs[i]);
	}

	return 0;
}

static int axp173_device_attr_unregister(struct miscdevice  *axp173_misc_opt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(axp173_device_attrs); i++)
		device_remove_file(axp173_misc_opt->this_device, &axp173_device_attrs[i]);

	return 0;
}
static int axp173_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int axp173_release(struct inode *inode, struct file *file)
{
	return 0;
}


static int axp173_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int ret = 0;
	int ioarg = 0;
	int reg,bit,sign;
	printk("enter  axp173_ioctl\n");
	/* 检测命令的有效性 */
	if (_IOC_TYPE(cmd) != MEMDEV_IOC_MAGIC) 
	{
		printk(" axp173  cmd error1\n");
		//return -EINVAL;
	}
	if (_IOC_NR(cmd) > MEMDEV_IOC_MAXNR) 
	{  
		printk(" axp173  cmd error2\n");
		//return -EINVAL;
	}
	 /* 根据命令，执行相应的操作 */
    switch(cmd) {

      /* 打印当前设备信息 */
      case MEMDEV_IOCPRINT:
       fiio_debug("<--- axp192 volum--->\n\n");
      fiio_debug("reg:0x78=%d\n",axp173_i2c_read(axp173, 0x78));
      fiio_debug("reg:0x79=%d\n",axp173_i2c_read(axp173, 0x79));
      fiio_debug("<--- axp192 find battery=%d\n",axp173_i2c_read_bit(axp173,0x01,5));
      printk("<--- axp192 find chargeing=%d\n",axp173_i2c_read_bit(axp173,0x01,6));
      printk("<---- battery=%d%\n",axp_restcap(axp173_i2c_read(axp173, 0x78)*16+axp173_i2c_read(axp173, 0x79)));
        //  printk("<--- begin shut down do2--->\n\n");
          //  axp173_i2c_write_bit(axp173,0x10,1,2);
        break;
      /* 获取参数 */
      case MEMDEV_IOCGETDATA: 
      printk(" axp173  cmd MEMDEV_IOCGETDATA\n");
       // ioarg = 1101;
       // ret = __put_user(ioarg, (int *)arg);
       ret = __get_user(ioarg, (int *)arg);
        reg=ioarg/100;
       // sign=(ioarg-reg*100)/10;
        bit=ioarg%10;
        printk("get from user:reg=%d,sign=%d,bit=%d\n",reg,sign,bit);
        //printk("<--- In Kernel MEMDEV_IOCSETDATA ioarg = %d --->\n\n",ioarg);
        //axp173_i2c_write_bit(axp173,0x10,0,2);
       // axp173_i2c_write_bit(axp173,reg,sign,bit);
        printk("axp192,reg%x bit%d=%d\n",reg,bit,axp173_i2c_read_bit(axp173,reg,bit));

        break;
      
      /* 设置参数 */
      case MEMDEV_IOCSETDATA: 
       printk(" axp173  cmd MEMDEV_IOCSETDATA\n");
       ret = __get_user(ioarg, (int *)arg);
	reg=ioarg/100;
	sign=(ioarg-reg*100)/10;
	bit=ioarg%10;
	printk("get from user:reg=%d,sign=%d,bit=%d\n",reg,sign,bit);
        //printk("<--- In Kernel MEMDEV_IOCSETDATA ioarg = %d --->\n\n",ioarg);
        //axp173_i2c_write_bit(axp173,0x10,0,2);
        axp173_i2c_write_bit(axp173,reg,sign,bit);
	printk("axp192,reg%x bit%d=%d\n",reg,bit,axp173_i2c_read_bit(axp173,reg,bit));
        break;

      /* shut down axp173 */
      case AXP173_SHUTDOWN:
       {
            //  printk("<--- begin shut down --->\n\n");
             //axp173_i2c_write_bit(axp173,0x10,1,2);
             printk("begin shut down the axp173");
			ret = axp173_i2c_write_bit(axp173,0x32,1,7);
			if (ret != 0) {
				printk("Shut down fail ret=%d.\n",ret);
			}
          break;
	  }
      case BLED_OPEN:
	{
            printk("try to set charger 630mA\n");
            axp173_i2c_write(axp173,0x33,0xE4);
            printk("reg 0x33 = %d\n",axp173_i2c_read(axp173,0x33));
          break;
	}
      case BLED_CLOSE:
	{/*
	  printk("try to set charger 1000mA\n");
                axp173_i2c_write(axp173,0x33,0xc8);
        printk("reg 0x33 = %d\n",axp173_i2c_read(axp173,0x33));
	  */
		printk("try to close LDO2 LD3 TF3.3\n");
		axp173_i2c_write_bit(axp173,0x12,0,2);
        axp173_i2c_write_bit(axp173,0x12,0,3);
        axp173_i2c_write_bit(axp173,0x12,0,6);
	break;
	}
      default:  
        return -EINVAL;
    }
    return ret;
}

static void fiio_irq_work(struct work_struct *work) {
	//read irq state
	unsigned char irq_state = axp173_i2c_read(axp173,0x44);
	fiio_debug("\n###########AXP173 IRQ STATE[44H]=0x%02x\n",irq_state);
	irq_state = axp173_i2c_read(axp173,0x45);
	fiio_debug("\n###########AXP173 IRQ STATE[45H]=0x%02x\n",irq_state);
	irq_state = axp173_i2c_read(axp173,0x46);
	fiio_debug("\n###########AXP173 IRQ STATE[46H]=0x%02x\n",irq_state);
	irq_state = axp173_i2c_read(axp173,0x47);
	fiio_debug("\n###########AXP173 IRQ STATE[47H]=0x%02x\n",irq_state);
}


static irqreturn_t irq_handler(int irqno, void *dev_id) //中断处理函数
 {
     printk("\n###########axp192 irq\n");
     //battery_data->headset_inser=!(battery_data->headset_inser);
     /*struct axp173 *axp192 = dev_id;
        if(axp173_i2c_read_bit(axp192,0x4D,2))
        {
                printk("headset inser or remove!\n");
                axp173_i2c_write_bit(axp192,0x4D,1,2);
        }*/
     //power_supply_changed(&battery_data->battery);
     queue_delayed_work(irq_wqueue, &irq_work, HZ);
     return IRQ_HANDLED;
 }

static struct file_operations axp173_opt_fops = {
	.owner	=	THIS_MODULE,
	.open	=	axp173_open,
	.unlocked_ioctl		=	axp173_ioctl,
	.release	= 	axp173_release,
};

static struct miscdevice axp173_misc_opt = {
	.minor	=	MISC_DYNAMIC_MINOR,
	.name	= 	(char *)driver_name,
	.fops	=	&axp173_opt_fops,
};
static int axp173_i2c_remove(struct i2c_client *i2c)
{
	struct axp173 *axp173 = i2c_get_clientdata(i2c);
	i2c_set_clientdata(i2c, NULL);
	kfree(axp173);
	cancel_delayed_work_sync(&update_work);
	return 0;
}
static int g_irq_40h=0;
static int g_irq_41h=0;
static int g_irq_42h=0;
static int g_irq_43h=0;
static int g_irq_44h=0;
static int g_irq_4Ah=0;

static int axp173_suspend(struct i2c_client *i2c)
{
    //printk("PM:axp173_suspend,try to close LDO2 LDO3 TF3.3\n");
    //axp173_i2c_write_bit(axp173,0x12,0,2);
    //axp173_i2c_write_bit(axp173,0x12,0,3);
    //axp173_i2c_write_bit(axp173,0x12,0,6);
    //axp173_i2c_write_bit(axp173,0x80,0,2);
    fiio_debug("%s:Enter!!!\n",__func__);
    //axp173_i2c_write(axp173,0x90,0x01);
    //axp173_i2c_write(axp173,0x92,0x01);
    //axp173_i2c_write(axp173,0x93,0x01);
    //axp173_i2c_write(axp173,0x95,0x8A);
    g_irq_40h = axp173_i2c_read(axp173,0x40);
    g_irq_41h = axp173_i2c_read(axp173,0x41);
    g_irq_42h = axp173_i2c_read(axp173,0x42);
    g_irq_43h = axp173_i2c_read(axp173,0x43);
    g_irq_44h = axp173_i2c_read(axp173,0x44);
    g_irq_4Ah = axp173_i2c_read(axp173,0x4A);

    axp173_i2c_write(axp173,0x40,0x00);
    axp173_i2c_write(axp173,0x41,0x00);
    axp173_i2c_write(axp173,0x42,0x00);
    axp173_i2c_write(axp173,0x43,0x00);
    axp173_i2c_write(axp173,0x44,0x00);
    axp173_i2c_write(axp173,0x4A,0x00);
	fiio_m5_send_key75_flag = 0;
    fiio_debug("%s:Exit!!!\n",__func__);
    return 0;
}

static int axp173_resume(struct i2c_client *i2c)
{
    fiio_debug("%s:Enter!!!\n",__func__);
    //axp173_i2c_write_bit(axp173,0x80,1,2);
    //axp173_i2c_write_bit(axp173,0x12,1,2);
    //axp173_i2c_write_bit(axp173,0x12,1,3);

    resume_sign=1;
    //axp173_i2c_write_bit(axp173,0x12,1,6);
    axp173_i2c_write(axp173,0x40,g_irq_40h);
    axp173_i2c_write(axp173,0x41,g_irq_41h);
    axp173_i2c_write(axp173,0x42,g_irq_42h);
    axp173_i2c_write(axp173,0x43,g_irq_43h);
    axp173_i2c_write(axp173,0x44,g_irq_44h);
    axp173_i2c_write(axp173,0x4A,g_irq_4Ah);
    fiio_debug("%s:Exit!!!\n",__func__);
    return 0;
}


static const struct i2c_device_id axp173_i2c_id[] = {
       { "axp173", 0 },
       { }
};

MODULE_DEVICE_TABLE(i2c, axp173_i2c_id);

static struct i2c_driver axp173_i2c_driver = {
	.driver = {
		.name = "axp173",
		.owner = THIS_MODULE,
	},
	.probe    = axp173_i2c_probe,
	.remove   = axp173_i2c_remove,
	.suspend    = axp173_suspend,
        .resume     = axp173_resume,
	.id_table = axp173_i2c_id,
};
//module_i2c_driver(axp173_i2c_driver);

#ifdef CONFIG_PM
static int emxx_battery_suspend(struct platform_device *pdev,
                  pm_message_t state)
{
    return 0;
}

static int emxx_battery_resume(struct platform_device *pdev)
{
    struct emxx_battery_data *data = platform_get_drvdata(pdev);
	printk("Enter %s !!!\n",__func__);
    power_supply_changed(&data->battery);

    cancel_delayed_work(&data->monitor_work);
    queue_delayed_work(data->monitor_wqueue, &data->monitor_work, HZ);

	printk("Exit %s !!!\n",__func__);

    return 0;
}


#else
#define emxx_battery_suspend NULL
#define emxx_battery_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver emxx_battery_driver = {
    .probe        = emxx_battery_probe,
    .remove        = emxx_battery_remove,
    .suspend    = emxx_battery_suspend,
    .resume     = emxx_battery_resume,
    .driver = {
        .name = "emxx-battery",
        .owner	= THIS_MODULE,
    }
};

static int __init axp173_module_init(void)
{
	int ret, retval;
	printk(KERN_INFO"enter %s,line:%d\n",__func__,__LINE__);
	ret = i2c_add_driver(&axp173_i2c_driver);
	if (ret != 0)
		printk(KERN_INFO"Failed to register AXP173 I2C driver: %d\n", ret);
		else {
		printk("Successfully added driver %s\n",
				axp173_i2c_driver.driver.name);
				}
				
	 retval = misc_register(&axp173_misc_opt);
	if (retval < 0)
	{
		printk(KERN_ERR"register misc device opt failed.\n");
		return retval;
	}
	
	retval=platform_driver_register(&emxx_battery_driver);
	if (retval < 0)
	{
		printk(KERN_ERR"register emxx_battery_driver failed.\n");
		return retval;
	}
	else printk("register emxx_battery_driver ok!\n");
    //axp173_i2c_write_bit(axp173,0x10,1,2);
	axp173_device_attr_register(&axp173_misc_opt);

	printk("begin to request irq!\n");
	if(GPIO_AXP192_IRQ > 0){
        ret = gpio_request(GPIO_AXP192_IRQ, "GPIO_AXP192_IRQ");
        if (ret) {
            printk(KERN_ERR "can's request GPIO_AXP192_IRQ\n");
            return ret;
        }
    }

	ret = request_threaded_irq(gpio_to_irq(GPIO_AXP192_IRQ), irq_handler,NULL, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"axp192_irq", axp173);
	if(ret){
		printk("request axp192 irq failed!\n");
		return ret;
	}
	
	//add for irq
	INIT_DELAYED_WORK(&irq_work, fiio_irq_work);
    irq_wqueue = create_singlethread_workqueue("axp192_irq");
    if (!irq_wqueue) {
		printk("create_singlethread_workqueue axp192 irq failed!\n");
        ret = -ESRCH;
    }
	printk("end to request irq ret=%d!\n",ret);

    return ret;
}
//module_init(axp173_module_init);
//rootfs_initcall(axp173_module_init);
fs_initcall_sync(axp173_module_init);

static void __exit axp173_module_exit(void)
{
	i2c_del_driver(&axp173_i2c_driver);
	axp173_device_attr_unregister(&axp173_misc_opt);
	platform_driver_unregister(&emxx_battery_driver);
	free_irq(gpio_to_irq(GPIO_AXP192_IRQ), NULL);
	gpio_free(GPIO_AXP192_IRQ);
}
module_exit(axp173_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhengpeizhu <zhengpz@fiio.cn>");
MODULE_DESCRIPTION("axp173 PMIC driver");
