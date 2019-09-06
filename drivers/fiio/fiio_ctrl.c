#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/soundcard.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/mm.h>

#include <linux/regulator/machine.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>

#include "fiio_ctrl.h"

#define DRIVER_NAME "fiio_ctrl"
#if 0
//#define FIIO_DEBUG_CTRL
#ifdef FIIO_DEBUG_CTRL
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_ctrl] " x)
#else
#define fiio_debug(x...)
#endif
#endif
static int fiio_ctrl_debug = 0;
module_param(fiio_ctrl_debug, int, 0644);
#define fiio_debug(msg...)			\
	do {					\
		if (fiio_ctrl_debug)		\
			printk("fiioctrl: " msg);	\
	} while(0)

static void close_all_led() {
	gpio_direction_output(GPIO_M5_LED0,1);
	gpio_direction_output(GPIO_M5_LED1,1);
	gpio_direction_output(GPIO_M5_LED2,1);
}

extern int fiio_get_dsd_output_format(void);
extern void fiio_set_dsd_output_format(int format);
/*
 *设置LCD屏幕旋转角度
 *0:0度 1:90度 2:180度 3:360度
 *default:0 0度
 */
extern int fiio_get_lcd_rotate();
extern void fiio_set_lcd_rotate(int rotate);

/*
 *选择USB链接接口 x1000或是csr8675
 */
 #ifdef CONFIG_M5_BORAD_2_0910
 static void init_usb_select_gpio() {
 	int ret = 0;
	ret = gpio_request(GPIO_USB_EN, "usb port enable pin");
	if (ret) {
		printk(KERN_ERR "can's request usb port enable pin\n");
		return ret;
	}
	ret = gpio_request(GPIO_S1_EN, "usb port select pin");
	if (ret) {
		printk(KERN_ERR "can's request usb port select pin\n");
		return ret;
	}
 }
 static void uinstall_usb_select_gpio() {
	 gpio_free(GPIO_USB_EN);
	 gpio_free(GPIO_S1_EN);
 }
 //0:x1000 1:csr8675 2:no
static void select_usb_to_x1000_or_csr8675(int mode) {
	fiio_debug("File:%s %s mode=%d.\n",__FILE__,__func__,mode);
	switch(mode) {
		case 0:
			gpio_direction_output(GPIO_USB_EN,0);
			gpio_direction_output(GPIO_S1_EN,0);
			break;
		case 1:
			gpio_direction_output(GPIO_USB_EN,0);
			gpio_direction_output(GPIO_S1_EN,1);
			break;
		case 2:
			gpio_direction_output(GPIO_USB_EN,1);
			break;
	}
}

static int get_usb_path_mode() {
	int en_value,s_value,ret;
	en_value = gpio_get_value(GPIO_USB_EN);
	s_value = gpio_get_value(GPIO_S1_EN);
	if (0 == en_value) {
		if (0 == s_value) {
			ret = 0;
		}
		else {
			ret = 1;
		}
	}
	else {
		ret = 2;
	}
	return ret;
}
#endif

/*
 *spdif 输出控制IO
 */
extern void set_fiio_spdif_or_i2s_port_mode(int mode);
#ifdef CONFIG_M5_BORAD_2_0910

 static void init_spdif_output_gpio() {
 	int ret = 0;
	ret = gpio_request(GPIO_SPDIF_OUT, "spdif output pin");
	if (ret) {
		printk(KERN_ERR "can's request spdif output pin\n");
		return ret;
	}
	
 }
 static void set_spdif_out() {
 	fiio_debug("File:%s %s.\n",__FILE__,__func__);
	gpio_direction_output(GPIO_SPDIF_OUT,FIIO_M5_SPDIF_OUT);
 }
  static void set_spdif_off() {
  	fiio_debug("File:%s %s.\n",__FILE__,__func__);
	gpio_direction_output(GPIO_SPDIF_OUT,FIIO_M5_SPDIF_OFF);
 }
 static void uinstall_spdif_output_gpio() {
	 gpio_free(GPIO_SPDIF_OUT);
 }
 static void set_spdif_out_or_off(int flag) {
	if (1 == flag) {
		set_spdif_out();
        set_fiio_spdif_or_i2s_port_mode(FIIO_M5_AUDIO_PATH_SPDIF);
	}
	else {
		set_spdif_off();
        set_fiio_spdif_or_i2s_port_mode(FIIO_M5_AUDIO_PATH_I2S);
	}
 }
 //0:i2s 1:spdif
 static int get_spdif_out_or_off() {
	int value = gpio_get_value(GPIO_SPDIF_OUT);
	fiio_debug("File:%s %s value=%d.\n",__FILE__,__func__,value);
	if (FIIO_M5_SPDIF_OUT == value)
		return 1;
	else
		return 0;
 }
 #endif


static void bt_gpio_init(void) {
    int ret = 0;
    ret = gpio_request(BT_RST, "csr8675 reset gpio");
	if (ret) {
		printk(KERN_ERR "can's request csr8675 reset gpio\n");
		return ret;
	}
	#ifdef CONFIG_M5_BORAD_1_0629
    ret = gpio_request(BT_PW_EN, "csr8675 power on gpio");
	if (ret) {
		printk(KERN_ERR "can's request csr8675 power on gpio\n");
		return ret;
	}
	#endif

    ret = gpio_request(BT_VREG_ON, "csr8675 vreg on gpio");
	if (ret) {
		printk(KERN_ERR "can's request csr8675 vreg on gpio\n");
		return ret;
	}

    ret = gpio_request(BT_SWITCH_SINK_SOURCE, "csr8675 sink or source gpio");
    if (ret) {
        printk(KERN_ERR "can's request csr8675 sink or source gpio\n");
        return ret;
    }
    return 0;
}
/**
 * 1:source 
 * 0:sink
 */
static void switch_csr8675_to_sink_or_source(int sink_or_switch) {
    gpio_direction_output(BT_SWITCH_SINK_SOURCE,sink_or_switch);
}

static void init_csr8675_reset_and_vreg() {
	gpio_direction_output(BT_RST,1);
	gpio_direction_output(BT_VREG_ON,0);
}
static void fiio_csr8675_disable() {
	fiio_debug("%s:Enter!!!\n",__func__);

	gpio_direction_output(BT_RST,0);
	fiio_debug("%s:Exit!!!\n",__func__);
}

static void fiio_csr8675_no_disable() {

	fiio_debug("%s:Enter!!!\n",__func__);
	gpio_direction_output(BT_RST,1);
	fiio_debug("%s:Exit!!!\n",__func__);
}


void fiio_csr8675_reset() {
	fiio_debug("%s:Enter!!!\n",__func__);

	gpio_direction_output(BT_RST,1);
	udelay(500000); //500ms
    gpio_direction_output(BT_RST,0);
    udelay(500000); //500ms
    gpio_direction_output(BT_RST,1);
	fiio_debug("%s:Exit!!!\n",__func__);
}
EXPORT_SYMBOL(fiio_csr8675_reset);


static void fiio_csr8675_power_enable(int enable) {
	fiio_debug("%s:Enter!!!\n",__func__);
	#ifdef CONFIG_M5_BORAD_1_0629
    gpio_direction_output(BT_PW_EN,enable);
	#endif
	fiio_debug("%s:Exit!!!\n",__func__);
}

static void fiio_csr8675_vreg_on() {
	fiio_debug("%s:Enter!!!\n",__func__);
    gpio_direction_output(BT_VREG_ON,1);
    udelay(3000000); //2s
    gpio_direction_output(BT_VREG_ON,0);
	fiio_debug("%s:Exit!!!\n",__func__);
}

static void fiio_csr8675_vreg_off() {
	fiio_debug("%s:Enter!!!\n",__func__);
    gpio_direction_output(BT_VREG_ON,1);
    udelay(3500000); //3.5s
    gpio_direction_output(BT_VREG_ON,0);
	fiio_debug("%s:Exit!!!\n",__func__);
}


static void fiio_crs8675_gpio_init() {
	fiio_debug("%s:Enter!!!\n",__func__);
    gpio_direction_output(BT_RST,1);
    gpio_direction_output(BT_VREG_ON,0);
	#ifdef CONFIG_M5_BORAD_1_0629
    gpio_direction_output(BT_PW_EN,0);
	#endif
	fiio_debug("%s:Exit!!!\n",__func__);
}

/*
 *fiio_m5_player_mode
 *0:normal 
 *1:bt sink
 */
 extern void set_m5_player_mode_for_gpio_keys(int mode);
static int fiio_m5_player_mode = 0;
static void set_m5_player_mode(int mode) {
	fiio_m5_player_mode = mode;
	set_m5_player_mode_for_gpio_keys(mode);
}
int get_m5_player_mode(void) {
	return fiio_m5_player_mode;
}
EXPORT_SYMBOL(get_m5_player_mode);

/*
 *set bt to sink mode
 *step 1:set i2c0 port to input nopull mode
 *step 2:set I2S port to input nopull mode
 *step 3:power on csr8675
 *user application control from uart0
 *step 1:set bt sink
 *step 2:close lcd and suspend system
 *step 3:app recv open lcd command 
 *csr8675 application control
 *step 1:set sink mode
 *step 2:set i2s and i2s master mode
 *step 3:power on dac
 *step 4:init dac in high performance or low power mode
 */
 
extern void fiio_set_io_to_input_no_pull_state(void);
extern void fiio_set_io_to_normal_gpio_state(void);
extern void set_i2si2c0_to_gpio_input(void);
//#define PWZH_TEST_BTSINK
#ifdef PWZH_TEST_BTSINK
extern void fiio_ak4377_poweron();
extern void fiio_ak4377_poweron_low_power();
#endif
extern void fiio_set_gpio_to_input_no_pull_state(void);
static void fiio_set_bt_work_in_sink_mode() {
	fiio_debug("%s:Enter!!!\n",__func__);
	set_m5_player_mode(FIIO_M5_PLAYER_BT_SINK);
    switch_csr8675_to_sink_or_source(0);
    fiio_csr8675_reset();
	set_i2si2c0_to_gpio_input();
	fiio_set_io_to_input_no_pull_state();
	fiio_csr8675_power_enable(1);
	//must delay
	udelay(100000);//100ms
    fiio_csr8675_vreg_on();
	fiio_debug("%s:Exit!!!\n",__func__);
}
/**
*bt source
*/
static void fiio_set_bt_work_in_source_mode() {
    fiio_debug("%s:Enter!!!\n",__func__);
    switch_csr8675_to_sink_or_source(1);
    fiio_csr8675_reset();
    fiio_csr8675_power_enable(1);
    //must delay
    udelay(100000);//100ms
    fiio_csr8675_vreg_on();
    fiio_debug("%s:Exit!!!\n",__func__);
}

static void fiio_close_bt_source_mode() {
    fiio_debug("%s:Enter!!!\n",__func__);
    switch_csr8675_to_sink_or_source(0);
    fiio_csr8675_vreg_off();
    fiio_csr8675_power_enable(0);
	fiio_csr8675_disable();
    fiio_debug("%s:Exit!!!\n",__func__);
}

static void fiio_set_bt_work_in_sink_mode_lowpower() {
	set_i2si2c0_to_gpio_input();
	fiio_set_io_to_input_no_pull_state();
	fiio_csr8675_power_enable(1);
	set_m5_player_mode(FIIO_M5_PLAYER_BT_SINK);
}

/*
 *close sink mode
 *step 1:power down csr8675
 *step 2:set I2S port to i2s function
 *step 3:set i2c0 port to i2c function
 */
extern void set_i2si2c0_to_function(void);
extern void fiio_set_gpio_to_normal_function_state(void);
static void fiio_close_bt_sink_mode() {
	fiio_debug("%s:Enter!!!\n",__func__);
	fiio_csr8675_vreg_off();
    fiio_csr8675_power_enable(0);
	fiio_set_io_to_normal_gpio_state();
	set_i2si2c0_to_function();
    close_all_led();
	fiio_csr8675_disable();
	set_m5_player_mode(FIIO_M5_PLAYER_NORMAL);
	fiio_debug("%s:Exit!!!\n",__func__);
}

/*
 *config system wakeup source 
 *when system suspend in bt sink player mode don't wakeup system for short click play/pause button
 *so normal mode short click can wakeup
 */
static g_fiio_can_clear_normal_wakup = 0;
static g_fiio_can_clear_sink_wakup = 0;
static int bt_int_irq = 0;
static int global_irq = 0;
static void fiio_init_wakeup_source(void) {
	bt_int_irq = gpio_to_irq(BT_INT1);
	global_irq = gpio_to_irq(GPIO_ENDCALL_KEY);
} 
static void set_system_normal_wakp_source() {
	//
	if(gpio_is_valid(GPIO_ENDCALL_KEY)) {
		printk("%s enable_irq_wake GPIO_ENDCALL_KEY\n",__func__);
		enable_irq_wake(global_irq);
		g_fiio_can_clear_normal_wakup = 1;
	}
	if (1 == g_fiio_can_clear_sink_wakup) {
		if(gpio_is_valid(BT_INT1)) {
			printk("%s disable_irq_wake BT_INT1\n",__func__);
			disable_irq_wake(bt_int_irq);
		}
		g_fiio_can_clear_sink_wakup = 0;
	}
}

static void set_system_bt_wakp_source() {
	//
	if(gpio_is_valid(BT_INT1)) {
		printk("%s enable_irq_wake BT_INT1\n",__func__);
		enable_irq_wake(bt_int_irq);
		g_fiio_can_clear_sink_wakup = 1;
	}
	if (1 == g_fiio_can_clear_normal_wakup) {
		if(gpio_is_valid(GPIO_ENDCALL_KEY)) {
			printk("%s disable_irq_wake GPIO_ENDCALL_KEY\n",__func__);
			disable_irq_wake(global_irq);
			
		}
		g_fiio_can_clear_normal_wakup = 0;
	}
	
}

static void clear_system_wakeup_source() {
	if (1 == g_fiio_can_clear_normal_wakup) {
		if(gpio_is_valid(GPIO_ENDCALL_KEY)) {
			printk("%s disable_irq_wake GPIO_ENDCALL_KEY\n",__func__);
			disable_irq_wake(global_irq);
			
		}
		g_fiio_can_clear_normal_wakup = 0;
	}
	if (1 == g_fiio_can_clear_sink_wakup) {
		if(gpio_is_valid(BT_INT1)) {
			printk("%s disable_irq_wake BT_INT1\n",__func__);
			disable_irq_wake(bt_int_irq);
		}
		g_fiio_can_clear_sink_wakup = 0;
	}
}

void fiio_set_wakeup_source() {
#if 0
	if (FIIO_M5_PLAYER_NORMAL == get_m5_player_mode()) {
		set_system_normal_wakp_source();
	}
	else if (FIIO_M5_PLAYER_BT_SINK == get_m5_player_mode()) {
		set_system_bt_wakp_source();
	}
#endif
}
EXPORT_SYMBOL(fiio_set_wakeup_source);

void fiio_clear_wakeup_source() {
	//clear_system_wakeup_source();
}
EXPORT_SYMBOL(fiio_clear_wakeup_source);


static ssize_t bt_power_show(struct device *device, struct device_attribute *attr, char *buf)
{
	int value = 0;
	#ifdef CONFIG_M5_BORAD_1_0629
	value = gpio_get_value(BT_PW_EN);
	#endif
	return sprintf(buf,"%d\n",value);

}
static ssize_t bt_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int event_type = simple_strtoul(buf, NULL, 0);
    if(event_type == 1) {
        fiio_csr8675_power_enable(1);
    }
    else {
        fiio_csr8675_power_enable(0);
    }
	return count;

}	
static ssize_t bt_vregon_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int event_type = simple_strtoul(buf, NULL, 0);
    if(event_type == 1) {
        fiio_csr8675_vreg_on(1);
    }
	return count;
}	

static ssize_t bt_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
   
	int event_type = simple_strtoul(buf, NULL, 0);
    if(event_type == 1) {
        fiio_csr8675_reset(1);
    }
	return count;
}	

static ssize_t bt_disable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
   
	int event_type = simple_strtoul(buf, NULL, 0);
    if(event_type == 1) {
        fiio_csr8675_disable();
    }
	return count;
}	

static ssize_t bt_sink_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int event_type = simple_strtoul(buf, NULL, 0);
    if(event_type == 0) {
		printk("\n");
		printk("0:help!\n");
		printk("1:enter bt sink mode!\n");
		printk("2:exit bt sink mode!\n");
		printk("3:enter bt sink mode low power!\n");
		printk("4:set bt connect io in no pull input state!\n");
		printk("5:set bt connect io in normal state!\n");
		printk("\n");
    }
	else if(event_type == 1) {
		fiio_set_bt_work_in_sink_mode();
    }
	else if(event_type == 2) {
        fiio_close_bt_sink_mode();
    }
	else if(event_type == 3) {
		fiio_set_bt_work_in_sink_mode_lowpower();
    }
	else if(event_type == 4) {
		fiio_set_gpio_to_input_no_pull_state();
    }
	else if(event_type == 5) {
		fiio_set_gpio_to_normal_function_state();
    }
	return count;
}	


static ssize_t bt_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int event_type = simple_strtoul(buf, NULL, 0);
    if(event_type == 0) {
        printk("\n");
        printk("0:help!\n");
        printk("1:enter bt source mode!\n");
        printk("2:exit bt source mode!\n");
        printk("\n");
    }
    else if(event_type == 1) {
        fiio_set_bt_work_in_source_mode();
    }
    else if (event_type == 2) {
        fiio_close_bt_source_mode();
    }
    return count;
}

/*set cpu wakeup time*/
extern void fiio_set_wakeup_time(int time);
static ssize_t cpu_wakeup_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int time = simple_strtoul(buf, NULL, 0);
    fiio_set_wakeup_time(time);
	return count;
}	

//set lcd rotate


static ssize_t lcd_rotate_show(struct device *device, struct device_attribute *attr, char *buf)
{
	int rotate = fiio_get_lcd_rotate();
	switch(rotate) {
		case 0:
            fiio_debug("\nFIIO LCD ROTATE 0.\n");
			break;
		case 1:
            fiio_debug("\nFIIO LCD ROTATE 90.\n");
			break;
		case 2:
            fiio_debug("\nFIIO LCD ROTATE 180.\n");
			break;
		case 3:
            fiio_debug("\nFIIO LCD ROTATE 270.\n");
			break;
		default:
		break;
	}
	return sprintf(buf,"%d\n",rotate);

}

static ssize_t lcd_rotate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int rotate = simple_strtoul(buf, NULL, 0);
	fiio_set_lcd_rotate(rotate);
	return n;
}

static ssize_t spdif_show(struct device *device, struct device_attribute *attr, char *buf)
{
	int rotate = 0;
#ifdef CONFIG_M5_BORAD_2_0910
	rotate = get_spdif_out_or_off();
#endif
	switch(rotate) {
		case 1:
			printk("\nSPDIF ON MODE.\n");
			break;
		case 0:
			printk("\nSPDIF OFF MODE.\n");
			break;
		default:
		break;
	}
	return sprintf(buf,"%d\n",rotate);

}

static ssize_t spdif_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int flag = simple_strtoul(buf, NULL, 0);
	#ifdef CONFIG_M5_BORAD_2_0910
		switch(flag) {
			case 1:
				printk("\nSPDIF ON MODE.\n");
				break;
			case 0:
				printk("\nSPDIF OFF MODE.\n");
				break;
			default:
			break;
		}
		set_spdif_out_or_off(flag);
	#endif
	
	return n;
}


static ssize_t usb_show(struct device *device, struct device_attribute *attr, char *buf)
{
	int rotate = 0;
#ifdef CONFIG_M5_BORAD_2_0910
	rotate = get_usb_path_mode();
#endif
	switch(rotate) {
		case 0:
			printk("\nUSB TO X1000.\n");
			break;
		case 1:
			printk("\nUSB TO CSR8675.\n");
			break;
		case 2:
			printk("\nUSB NO Function.\n");
			break;
		default:
		break;
	}
	return sprintf(buf,"%d\n",rotate);

}

static ssize_t usb_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int flag = simple_strtoul(buf, NULL, 0);
	#ifdef CONFIG_M5_BORAD_2_0910
		switch(flag) {
			case 0:
				printk("\nUSB TO X1000.\n");
				break;
			case 1:
				printk("\nUSB TO CSR8675.\n");
				break;
			case 2:
				printk("\nUSB NO Function.\n");
				break;
			default:
			break;
		}
		select_usb_to_x1000_or_csr8675(flag);
	#endif
	
	return n;
}

/****************************************************************/
//设置USB OTG enable
//usb as host function set  GPIO_OTG_EN to 1
//usb device set GPIO_OTG_EN to 0
//set usb connect to x1000
static int m_m5_test_host_flag = 0;
void fiio_set_host_eys_test_mode(int enable) {
    m_m5_test_host_flag = enable;
}
int fiio_get_host_eys_test_mode(void) {
    return m_m5_test_host_flag;
}
EXPORT_SYMBOL(fiio_get_host_eys_test_mode);

#ifdef CONFIG_M5_BORAD_2_0910
void init_otg_enable_gpio(void) {
   int ret = 0;
   ret = gpio_request(GPIO_OTG_EN, "otg enable pin");
   if (ret) {
	   printk(KERN_ERR "can's request otg enable pin\n");
	   return ret;
  	}
 
   	ret = gpio_request(GPIO_OTG_ID, "GPIO_OTG_ID");
	if (ret) {
		printk(KERN_ERR "can's request GPIO_OTG_ID\n");
		return ret;
	}

	ret = gpio_request(GPIO_USB_POWER_ON, "GPIO_USB_POWER_ON");
	if (ret) {
		printk(KERN_ERR "can's request GPIO_USB_POWER_ON\n");
		return ret;
	}

	ret = gpio_request(GPIO_USB_CHANGE_EN, "GPIO_USB_CHANGE_EN");
	if (ret) {
		printk(KERN_ERR "can's request GPIO_USB_CHANGE_EN\n");
		return ret;
	}
	
	gpio_direction_output(GPIO_OTG_ID,1);
}
EXPORT_SYMBOL(init_otg_enable_gpio);

void fiio_usb_power_on(int enable) {
    if (1 == fiio_get_host_eys_test_mode()) {
        gpio_direction_output(GPIO_USB_POWER_ON,1);
    }
    else {
        gpio_direction_output(GPIO_USB_POWER_ON,enable);
    }

	if (1 == enable) {
		fiio_debug("%s usb power on !\n",__func__);
	}
	else {
		fiio_debug("%s usb power off!\n",__func__);
	}
}
EXPORT_SYMBOL(fiio_usb_power_on);

void fiio_check_poweron_io(void){
    int value = 0;
    gpio_direction_input(GPIO_USB_POWER_ON);
    value = gpio_get_value(GPIO_USB_POWER_ON);
    fiio_debug("%s GPIO_USB_POWER_ON gpio default value=%d\n",__func__,value);

    gpio_direction_output(GPIO_USB_POWER_ON,1);
    gpio_direction_input(GPIO_USB_POWER_ON);
    value = gpio_get_value(GPIO_USB_POWER_ON);
    fiio_debug("%s GPIO_USB_POWER_ON gpio set high value=%d\n",__func__,value);

    gpio_direction_output(GPIO_USB_POWER_ON,0);
    gpio_direction_input(GPIO_USB_POWER_ON);
    value = gpio_get_value(GPIO_USB_POWER_ON);
    fiio_debug("%s GPIO_USB_POWER_ON gpio set low value=%d\n",__func__,value);
}
EXPORT_SYMBOL(fiio_check_poweron_io);

static void uinstall_otg_enable_gpio() {
	gpio_free(GPIO_OTG_EN);
	gpio_free(GPIO_OTG_ID);
	gpio_free(GPIO_USB_POWER_ON);
	gpio_free(GPIO_USB_CHANGE_EN);
}

static ssize_t otg_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int flag = simple_strtoul(buf, NULL, 0);
	if (1 == flag) {
		gpio_direction_output(GPIO_OTG_EN,1);
	}
	else {
		gpio_direction_output(GPIO_OTG_EN,0);
	}
	return n;
}

void fiio_enable_or_disable_otg(int enable) {
	gpio_direction_output(GPIO_OTG_EN,enable);
}
EXPORT_SYMBOL(fiio_enable_or_disable_otg);


static ssize_t otg_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int id = simple_strtoul(buf, NULL, 0);
	gpio_direction_output(GPIO_OTG_ID,id);
	return n;
}

#define FIIO_USB_DEVICE 1
#define FIIO_USB_HOST 0


#define FIIO_USB_CHANGE 0
#define FIIO_USB_NO_CHANGE 1

//设置或是获取USB是host or device 模式
static int fiio_usb_is_host_flag = 1;
//1:device
//0:host
void fiio_set_usb_host_or_device(int flag) {
	gpio_direction_output(GPIO_OTG_ID,flag);
	if (FIIO_USB_DEVICE == flag) {
		gpio_direction_output(GPIO_OTG_EN,0);
		fiio_usb_is_host_flag = FIIO_USB_DEVICE;
		//change enable
		gpio_direction_output(GPIO_USB_CHANGE_EN,FIIO_USB_CHANGE);
	}
	else {
		gpio_direction_output(GPIO_OTG_EN,1);
		fiio_usb_is_host_flag = FIIO_USB_HOST;
		gpio_direction_output(GPIO_USB_CHANGE_EN,FIIO_USB_NO_CHANGE);
	}
	if (FIIO_USB_DEVICE == flag) {
		fiio_debug("%s usb in device mode!\n",__func__);
	}
	else {
		fiio_debug("%s usb in host mode!\n",__func__);
	}
}
EXPORT_SYMBOL(fiio_set_usb_host_or_device);

int get_fiio_usb_host_or_device(void) {
	return fiio_usb_is_host_flag;
}
EXPORT_SYMBOL(get_fiio_usb_host_or_device);

//1:connect 0:disconnect
static int fiio_usb_is_conn_or_disconn_flag = 0;
void fiio_set_usb_conn_or_disconn_flag(int flag) {
	fiio_usb_is_conn_or_disconn_flag = flag;
	
	if (1 == flag) {
		fiio_debug("%s usb connect!\n",__func__);
		fiio_usb_power_on(1);
	}
	else {
		fiio_debug("%s usb disconnect!\n",__func__);
		fiio_usb_power_on(0);
	}
}
EXPORT_SYMBOL(fiio_set_usb_conn_or_disconn_flag);

void fiio_update_usb_conn_or_disconn_flag(int flag) {
	fiio_usb_is_conn_or_disconn_flag = flag;
}
EXPORT_SYMBOL(fiio_update_usb_conn_or_disconn_flag);


int fiio_get_usb_conn_or_disconn_flag(void) {
	return fiio_usb_is_conn_or_disconn_flag;
}
EXPORT_SYMBOL(fiio_get_usb_conn_or_disconn_flag);


static ssize_t usb_power_on_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int enable = simple_strtoul(buf, NULL, 0);
	switch(enable) {
		case 0:
			printk("\n%s USB POWER OFF.\n",__func__);
			break;
		case 1:
			printk("\n%s USB POWER ON.\n",__func__);
			break;
		default:
		break;
	}
	fiio_usb_power_on(enable);
	
	return n;
}

//0:don't change 1:change
static void fiio_enable_hw_change(int enable) {
	if (FIIO_USB_DEVICE == get_fiio_usb_host_or_device()) {
		if (0 == enable) {
			gpio_direction_output(GPIO_USB_CHANGE_EN,FIIO_USB_NO_CHANGE);
			fiio_debug("%s enable changing!\n",__func__);
		}
		else {
			gpio_direction_output(GPIO_USB_CHANGE_EN,FIIO_USB_CHANGE);
			fiio_debug("%s disable changing!\n",__func__);
		}
	}
	else {
		//host
		if (0 == enable) {
			gpio_direction_output(GPIO_USB_CHANGE_EN,FIIO_USB_NO_CHANGE);
			fiio_debug("%s can't enable changing for current is host!\n",__func__);
		}
	}
	
}

static int fiio_get_hw_change_state(void) {
	int state = gpio_get_value(GPIO_USB_CHANGE_EN);
	if (1 == state)
		return 0;
	else
		return 1; 
}
#endif	

////////////////////////////begin 计步器接口/////////////////////////////////////
#ifdef CONFIG_FIIO_SENSORS_LIS3DSH
extern int lis3dsh_get_step(void);
extern void lis3dsh_init(void);
#endif
#ifdef CONFIG_FIIO_SENSORS_STK8323
extern int stk8323_get_step(void);
extern void stk8323_reset(void);
extern void stk8323_set_power_mode(int mode);
#endif

static ssize_t step_get_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int counter = 0;
	int cmd = simple_strtoul(buf, NULL, 0);
	switch(cmd) {
		case 0:
			printk("0:help.\n");
			printk("1:get pedometer counter.\n");
			printk("2:reset pedometer chip.\n");
			printk("3:set chip to suspend mode.\n");
			printk("4:set chip to low power mode.\n");
			printk("5:set chip to sleep timer mode.\n");
			break;
		case 1:
			#ifdef CONFIG_FIIO_SENSORS_LIS3DSH
			counter = lis3dsh_get_step();
			#endif
			#ifdef CONFIG_FIIO_SENSORS_STK8323
			counter = stk8323_get_step();
			#endif
			fiio_debug("%s>>>>>>>>>>>>>>>>>>>>counter=%d\n",__func__,counter);
			break;
		case 2:
			#ifdef CONFIG_FIIO_SENSORS_LIS3DSH
			lis3dsh_init();
			#endif
			#ifdef CONFIG_FIIO_SENSORS_STK8323
			stk8323_reset();
			#endif
			break;
		#ifdef CONFIG_FIIO_SENSORS_STK8323
		case 3:
			stk8323_set_power_mode(0);
			break;
		case 4:
			stk8323_set_power_mode(1);
			break;
		case 5:
			stk8323_set_power_mode(2);
			break;
		#endif
		case 6:
			break;
		default:
		break;
	}
	
	return n;
}

#define FIIO_STEP_INIT 0
#define FIIO_STEP_REINIT 1
#define FIIO_STEP_STOP 2
#define FIIO_STEP_RESTART 3

#ifdef CONFIG_FIIO_SENSORS_STK832X
extern void fiio_start_setup();
extern void fiio_stop_setup();
extern int fiio_get_setup_counter();
extern void fiio_reinit_setup_counter();
extern void fiio_restart_setup();
#endif

static void process_pedometer_set_event(int event) {
	switch(event) {
		case FIIO_STEP_INIT:
			#ifdef CONFIG_FIIO_SENSORS_LIS3DSH
				lis3dsh_init();
			#endif
			#ifdef CONFIG_FIIO_SENSORS_STK832X
				fiio_start_setup();
			#endif
			break;
		case FIIO_STEP_REINIT:
			#ifdef CONFIG_FIIO_SENSORS_STK832X
				fiio_stop_setup();
			#endif
			break;
		case FIIO_STEP_STOP:
			#ifdef CONFIG_FIIO_SENSORS_STK832X
				fiio_reinit_setup_counter();
			#endif
			break;
		case FIIO_STEP_RESTART:
			#ifdef CONFIG_FIIO_SENSORS_STK832X
			fiio_restart_setup();
			#endif
			break;
	}
}

static int process_pedometer_get_event() {
	int counter = 0;
	#ifdef CONFIG_FIIO_SENSORS_LIS3DSH
		counter = lis3dsh_get_step();
	#endif
	#ifdef CONFIG_FIIO_SENSORS_STK832X
		counter = fiio_get_setup_counter();
	#endif
	return counter;
}

////////////////////////////end 计步器接口////////////////////////////////////

///////////////////////////begin LED//////////////////////////////////////////////////
static void led_gpio_init() {
	int ret = 0;
   	ret = gpio_request(GPIO_M5_LED0, "led gpio 0");
   	if (ret) {
	   	printk(KERN_ERR "can's request led gpio 0 pin\n");
	   	return ret;
  	}
	ret = gpio_request(GPIO_M5_LED1, "led gpio 1");
   	if (ret) {
	   printk(KERN_ERR "can's request led gpio 1 pin\n");
	   return ret;
  	}
    ret = gpio_request(GPIO_M5_LED2, "led gpio 2");
   	if (ret) {
	   printk(KERN_ERR "can's request led gpio 2 pin\n");
	   return ret;
  	}
}
static void get_led_gpio_state() {
	int value = gpio_get_value(GPIO_M5_LED0);
	printk("LED0 gpio level = %d.\n",value);
	value = gpio_get_value(GPIO_M5_LED1);
	printk("LED1 gpio level = %d.\n",value);
	value = gpio_get_value(GPIO_M5_LED2);
	printk("LED2 gpio level = %d.\n",value);
}

static ssize_t led_show(struct device *device, struct device_attribute *attr, char *buf)
{
	get_led_gpio_state();
	return sprintf(buf,"%d\n",1);

}
static void fiio_set_led0(int level) {
	gpio_direction_output(GPIO_M5_LED0,level);
}
static void fiio_set_led1(int level) {
	gpio_direction_output(GPIO_M5_LED1,level);
}
static void fiio_set_led2(int level) {
	gpio_direction_output(GPIO_M5_LED2,level);
}

extern  void fiio_set_led_to_input_no_pull_state(void);

static ssize_t led_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int flag = simple_strtoul(buf, NULL, 0);
	switch(flag) {
		case 0:
			printk("\n0:help.\n");
			printk("\n1:LED0 HIGH.\n");
			printk("\n2:LED0 LOW.\n");
			printk("\n3:LED1 HIGH.\n");
			printk("\n4:LED1 LOW.\n");
			printk("\n5:LED2 HIGH.\n");
			printk("\n6:LED2 LOW.\n");
			break;
		case 1:
			fiio_set_led0(1);
			break;
		case 2:
			fiio_set_led0(0);
			break;
		case 3:
			fiio_set_led1(1);
			break;
		case 4:
			fiio_set_led1(0);
			break;
		case 5:
			fiio_set_led2(1);
			break;
		case 6:
			fiio_set_led2(0);
			break;
		case 7:
			fiio_set_led_to_input_no_pull_state();
			break;
		default:
		break;
	}
	
	return n;
}
//////////////////////////////end LED////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
static ssize_t m5_charge_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int flag = simple_strtoul(buf, NULL, 0);
	switch(flag) {
		case 0:
			printk("\nM5 charge.\n");
			fiio_enable_hw_change(0);
			break;
		case 1:
			printk("\nM5 can't charge.\n");
			fiio_enable_hw_change(1);
			break;
		default:
		break;
	}
	
	return n;
}
/////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
extern void fiio_init_gpio_for_shutdown(void);
extern void fiio_resume_gpio(void);


static ssize_t fiio_system_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int flag = simple_strtoul(buf, NULL, 0);
	switch(flag) {
		case 0:
			printk("\nM5 fiio_init_gpio_for_shutdown.\n");
			fiio_init_gpio_for_shutdown();
			break;
		case 1:
			printk("\nM5 fiio_resume_gpio.\n");
			fiio_resume_gpio();
			break;
		default:
		break;
	}
	
	return n;
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_USB_DWC2
extern void fiio_update_usb_status(void);
static ssize_t update_usb_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	fiio_update_usb_status();
	return n;
}
#endif
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

static ssize_t io_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int flag = simple_strtoul(buf, NULL, 0);
	switch(flag) {
		case 0:
			printk("\n1:set bt control io to input no pull state.\n");
			printk("    [key(PB28 PB31) LED(PB05 PB21 PB22) ----DAC(PA16) key(PA19)]\n");
			printk("2:set bt control io to normal gpio.\n");
			printk("3:set bt i2s and i2c to no pull input state!\n");
			printk("4:set bt i2s and i2c to device function state!\n");
			printk("5:set bt i2s, i2c and io to no pull input state!\n");
			printk("6:set bt i2s , i2c and io to device function state!\n");
			break;
		case 1:
			printk("\nset bt control io to input no pull state.\n");
			fiio_set_io_to_input_no_pull_state();
			break;
		case 2:
			printk("\nset bt control io to normal gpio.\n");
			fiio_set_io_to_normal_gpio_state();
			close_all_led();
			break;
		case 3:
			printk("3:set bt i2s and i2c to no pull input state!\n");
			set_i2si2c0_to_gpio_input();
			break;
		case 4:
			printk("4:set bt i2s and i2c to device function state!\n");
			set_i2si2c0_to_function();
			break;
		case 5:
			set_i2si2c0_to_gpio_input();
			fiio_set_io_to_input_no_pull_state();
			init_csr8675_reset_and_vreg();
			break;
		case 6:
			set_i2si2c0_to_function();
			fiio_set_io_to_normal_gpio_state();
			close_all_led();
			break;
		case 7:
			init_csr8675_reset_and_vreg();
			break;
		case 8:
			gpio_direction_output(BT_RST,1);
			break;
		case 9:
			gpio_direction_output(BT_VREG_ON,0);
			break;
		default:
		break;
	}
	return n;
}
/////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
extern void fiio_debug_lcd_vdvs(int flag,int value);
static ssize_t lcd_vdvs_add_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	
	fiio_debug_lcd_vdvs(1,value);
	return n;
}
static ssize_t lcd_vdvs_dec_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	
	fiio_debug_lcd_vdvs(0,value);
	return n;
}
/////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
extern void set_g_sqrxtune(int v);
extern void set_g_txhsxvtune(int v);
extern void set_g_txvreftune(int v);

extern void set_g_txrisetune(int v);
extern void set_g_txpreemphtine(int v);



static ssize_t eye_usb_sqrxtune_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	
	if (value == 1000) {
		printk("7-0:-20%~+15% hold:5%\n");
	}
	else {
		set_g_sqrxtune(value);
	}
	return n;
}

static ssize_t eye_usb_txhsxvtune_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	
	if (value == 1000) {
		printk("3:default 2:+15mv 1:-15mv 0:reserved\n");
	}
	else {
		set_g_txhsxvtune(value);
	}
	return n;
}

static ssize_t eye_usb_txvreftune_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	
	if (value == 1000) {
		printk("b1111 +12.5%\n  b1110 +11.25%\n b1101 +10%\n b1100 +8.75%\nb1011 +7.5%\nb1010 +6.255\nb1001 +5%\nb1000 +3.75%\nb0111 +2.5% \n b0110 +1.25%\n b0101 Default \n b0100 -1.25%\n b0011 -2.5%\n b0010 -3.75%\n b0001 -5%\n b0000 -6.25%\n");
	}
	else {
		set_g_txvreftune(value);
	}
	return n;
}


static ssize_t eye_usb_txrisetune_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	
	if (value == 1000) {
		printk("This bit adjusts the rise/fall times of the high-speed waveform 1: -8% 0: default\n");
	}
	else {
		set_g_txrisetune(value);
	}
	return n;
}
static ssize_t eye_usb_txpreemphtine_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	
	if (value == 1000) {
		printk("This bit controls HS transmitter Pre-emphasis enable. 1: enable 0: disable default 1\n");
	}
	else {
		set_g_txpreemphtine(value);
	}
	return n;
}

extern void fiio_test_host_eye_diagram(void);
extern void fiio_test_host_eye_diagram_enterHost(void);
extern void fiio_get_host_status(void);
static ssize_t host_eye_diagram_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	switch(value) {
		case 0:
			fiio_test_host_eye_diagram();
            fiio_set_host_eys_test_mode(1);
			break;
		case 1:
			fiio_test_host_eye_diagram_enterHost();
			break;
        case 2:
            fiio_get_host_status();
            break;
        case 3:
            fiio_set_host_eys_test_mode(0);
            break;
		default:
		break;
	}
	return n;
}		

/////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////
/*
* @param mode
* 0:high performance mode
* 1:ow power mode
*/
extern void set_fiio_ak4377_work_mode(int mode);
extern int get_fiio_ak4377_work_mode();
///////////////////////////////////////////////////////////////////

//////////////////////////debug ////////////////////////////////////////
extern void AK4377_WriteOneByte_test(unsigned char WriteAddr,unsigned char DataToWrite);
static ssize_t ak4377_volume_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	//0dB
	//0.5dB
	if (0 == value) {
		printk("default:0dB 0.5dB step add.\n");
	}
	AK4377_WriteOneByte_test(0x0B,0x39+value);
	AK4377_WriteOneByte_test(0x0C,0x39+value);
	return n;
}
static ssize_t ak4377_hpg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);
	if (0 == value) {
		printk("default:0dB 1dB step add.\n");
	}
	AK4377_WriteOneByte_test(0x0D,0x6B+value);
	return n;
}

extern void set_fiio_m5_serial_debug(int flag);
static ssize_t serial_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	int value = simple_strtoul(buf, NULL, 0);

	set_fiio_m5_serial_debug(value);
	return n;
}
/////////////////////////////////////////////////////////////////
static struct device_attribute fiio_ctrl_device_attrs[] = {
	__ATTR(bt_poweron, S_IRUGO | S_IWUSR, bt_power_show, bt_power_store),
    __ATTR(bt_vregon,  S_IWUSR, NULL, bt_vregon_store),
    __ATTR(bt_reset,  S_IWUSR, NULL, bt_reset_store),
    __ATTR(bt_disable,  S_IWUSR, NULL, bt_disable_store),
    __ATTR(cpu_wakeup_time,  S_IWUSR, NULL, cpu_wakeup_time_store),
    __ATTR(bt_sink,  S_IWUSR, NULL, bt_sink_store),
    __ATTR(bt_source,  S_IWUSR, NULL, bt_source_store),
    __ATTR(lcd_rotate, S_IRUGO | S_IWUSR, lcd_rotate_show, lcd_rotate_store),
    __ATTR(spdif, S_IRUGO | S_IWUSR, spdif_show, spdif_store),
    __ATTR(usb, S_IRUGO | S_IWUSR, usb_show, usb_store),
    #ifdef CONFIG_M5_BORAD_2_0910
    __ATTR(otg_enable,  S_IWUSR, NULL, otg_enable_store),
    __ATTR(otg_id,  S_IWUSR, NULL, otg_id_store),
    __ATTR(usb_power_on,  S_IWUSR, NULL, usb_power_on_store),
    #endif
	__ATTR(pedometer,  S_IWUSR, NULL, step_get_store),
	#ifdef CONFIG_M5_BORAD_2_0910
	__ATTR(led, S_IRUGO | S_IWUSR, led_show, led_store),
	#endif
	#ifdef CONFIG_M5_BORAD_2_0910
	__ATTR(charge,  S_IWUSR, NULL, m5_charge_store),
	#endif
	__ATTR(fiio_system,  S_IWUSR, NULL, fiio_system_store),
#ifdef CONFIG_USB_DWC2
	__ATTR(update_usb,  S_IWUSR, NULL, update_usb_store),
#endif
	__ATTR(io,  S_IWUSR, NULL, io_store),
	__ATTR(lcd_vdvs_add,  S_IWUSR, NULL, lcd_vdvs_add_store),
	__ATTR(lcd_vdvs_dec,  S_IWUSR, NULL, lcd_vdvs_dec_store),
	
	__ATTR(eye_usb_sqrxtune,  S_IWUSR, NULL, eye_usb_sqrxtune_store),
	__ATTR(eye_usb_txhsxvtune,  S_IWUSR, NULL, eye_usb_txhsxvtune_store),
	__ATTR(eye_usb_txvreftune,  S_IWUSR, NULL, eye_usb_txvreftune_store),
	__ATTR(eye_usb_txrisetune,  S_IWUSR, NULL, eye_usb_txrisetune_store),
	__ATTR(eye_usb_txpreemphtine,  S_IWUSR, NULL, eye_usb_txpreemphtine_store),
	__ATTR(host_eye_diagram,  S_IWUSR, NULL, host_eye_diagram_store),
	__ATTR(ak4377_volume,  S_IWUSR, NULL, ak4377_volume_store),
	__ATTR(ak4377_hpg,  S_IWUSR, NULL, ak4377_hpg_store),
	__ATTR(serial_debug,  S_IWUSR, NULL, serial_debug_store),
    NULL
};

static int fiio_ctrl_device_attr_register(struct miscdevice  *fiio_ctrl_misc_opt)
{
	int error = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(fiio_ctrl_device_attrs)-1; i++) {
		error = device_create_file(fiio_ctrl_misc_opt->this_device, &fiio_ctrl_device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(fiio_ctrl_misc_opt->this_device, &fiio_ctrl_device_attrs[i]);
	}

	return 0;
}

static int fiio_ctrl_device_attr_unregister(struct miscdevice  *fiio_ctrl_misc_opt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(fiio_ctrl_device_attrs); i++)
		device_remove_file(fiio_ctrl_misc_opt->this_device, &fiio_ctrl_device_attrs[i]);

	return 0;
}


//////////////////////////////

extern void AK4377_Mute(void);

extern void fiio_set_dac_volume(void);

///////////////////////////
static int fiio_ctrl_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int fiio_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

extern int get_fiio_m5_output_select_bt(void);
extern void set_fiio_m5_output_select_bt(int flag);

static int fiio_ctrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
    int ret = 0;
    int ioarg = 0;
#if 0
	 /* 检测命令的有效性 */
    if (_IOC_TYPE(cmd) != MEMDEV_IOC_MAGIC2) 
    {
        printk(" fiio_ctrl  cmd error1\n");
        //return -EINVAL;
    }
    if (_IOC_NR(cmd) > MEMDEV_IOC_MAXNR2) 
    {  
        printk(" fiio_ctrl  cmd error2\n");
        //return -EINVAL;
	}
#endif
    fiio_debug("%s cmd=0x%02x _IOC_TYPE(cmd)=%d _IOC_NR(cmd)=%d\n",__func__,cmd,_IOC_TYPE(cmd),_IOC_NR(cmd));
	 /* 根据命令，执行相应的操作 */
    switch(cmd) {
        case MEMDEV_IO_BT_SOURCE_MODE:
            //bt sink mode
            fiio_debug("%s:Enter MEMDEV_IO_BT_SOURCE_MODE!\n",__func__);
            fiio_set_bt_work_in_source_mode();
            fiio_debug("%s:Exit MEMDEV_IO_BT_SOURCE_MODE ioarg=%d!\n",__func__,ioarg);
            break;
        case MEMDEV_IO_EXIT_BT_SOURCE_MODE:
            fiio_debug("%s:Enter MEMDEV_IO_EXIT_BT_SOURCE_MODE!\n",__func__);
            fiio_close_bt_source_mode();
            fiio_debug("%s:Exit MEMDEV_IO_EXIT_BT_SOURCE_MODE ioarg=%d!\n",__func__,ioarg);
            break;
        /* 打印当前设备信息 */
        case MEMDEV_IO_BT_POWER_ON:

        	break;
        /* 获取参数 */
        case MEMDEV_IO_GET_DATA_CMD_TEST: 
            ioarg = 1101;
            ret = __put_user(ioarg, (int *)arg);
        	break;
      
        /* 设置参数 */
        case MEMDEV_IO_SET_DATA_CMD_TEST:
            ret = __get_user(ioarg, (int *)arg);
            printk("<--- In Kernel MEMDEV_IOCSETDATA ioarg = %d --->\n\n",ioarg);
        	break;
		case MEMDEV_IO_BT_SINK_MODE:
			//bt sink mode
            fiio_debug("%s:Enter MEMDEV_IO_BT_SINK_MODE!\n",__func__);
			fiio_set_bt_work_in_sink_mode();
            fiio_debug("%s:Exit MEMDEV_IO_BT_SINK_MODE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_EXIT_BT_SINK_MODE:
            fiio_debug("%s:Enter MEMDEV_IO_EXIT_BT_SINK_MODE!\n",__func__);
            fiio_close_bt_sink_mode();
            fiio_debug("%s:Exit MEMDEV_IO_EXIT_BT_SINK_MODE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_BT_SINK_MODE_LOW_POWER:
            fiio_debug("%s:Enter MEMDEV_IO_BT_SINK_MODE_LOW_POWER!\n",__func__);
			fiio_set_bt_work_in_sink_mode_lowpower();
            fiio_debug("%s:Exit MEMDEV_IO_BT_SINK_MODE_LOW_POWER ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_DATA_CMD_LCD_ROTATE:
            fiio_debug("%s:Enter MEMDEV_IO_SET_DATA_CMD_LCD_ROTATE!\n",__func__);
			ret = __get_user(ioarg, (int *)arg);
			fiio_set_lcd_rotate(ioarg);
            fiio_debug("%s:Exit MEMDEV_IO_SET_DATA_CMD_LCD_ROTATE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_DATA_CMD_LCD_ROTATE:
            fiio_debug("%s:Enter MEMDEV_IO_GET_DATA_CMD_LCD_ROTATE!\n",__func__);
			ioarg = fiio_get_lcd_rotate();
			ret = __put_user(ioarg, (int *)arg);
            fiio_debug("%s:Exit MEMDEV_IO_GET_DATA_CMD_LCD_ROTATE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_DATA_CMD_D2P_OR_DOP:
            fiio_debug("%s:Enter MEMDEV_IO_SET_DATA_CMD_D2P_OR_DOP!\n",__func__);
			ret = __get_user(ioarg, (int *)arg);
			fiio_set_dsd_output_format(ioarg);
            fiio_debug("%s:Exit MEMDEV_IO_SET_DATA_CMD_D2P_OR_DOP ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_DATA_CMD_D2P_OR_DOP:
            fiio_debug("%s:Enter MEMDEV_IO_GET_DATA_CMD_D2P_OR_DOP!\n",__func__);
			ioarg = fiio_get_dsd_output_format();
			ret = __put_user(ioarg, (int *)arg);
            fiio_debug("%s:Exit MEMDEV_IO_GET_DATA_CMD_D2P_OR_DOP ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_DATA_CMD_I2S_OR_SPDIF:
            fiio_debug("%s:Enter MEMDEV_IO_SET_DATA_CMD_I2S_OR_SPDIF!\n",__func__);
			ret = __get_user(ioarg, (int *)arg);
			#ifdef CONFIG_M5_BORAD_2_0910
				set_spdif_out_or_off(ioarg);
			#endif
            fiio_debug("%s:Exit MEMDEV_IO_SET_DATA_CMD_I2S_OR_SPDIF ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_DATA_CMD_I2S_OR_SPDIF:
            fiio_debug("%s:Enter MEMDEV_IO_GET_DATA_CMD_I2S_OR_SPDIF!\n",__func__);
			#ifdef CONFIG_M5_BORAD_2_0910
				ioarg = get_spdif_out_or_off();
			#endif
			ret = __put_user(ioarg, (int *)arg);
            fiio_debug("%s:Exit MEMDEV_IO_GET_DATA_CMD_I2S_OR_SPDIF ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_DATA_CMD_BT_SINK_OR_SOURCE:
            fiio_debug("%s:Enter MEMDEV_IO_SET_DATA_CMD_BT_SINK_OR_SOURCE!\n",__func__);
            fiio_debug("%s:Exit MEMDEV_IO_SET_DATA_CMD_BT_SINK_OR_SOURCE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_DATA_CMD_BT_SINK_OR_SOURCE:
            fiio_debug("%s:Enter MEMDEV_IO_GET_DATA_CMD_BT_SINK_OR_SOURCE!\n",__func__);
            fiio_debug("%s:Exit MEMDEV_IO_GET_DATA_CMD_BT_SINK_OR_SOURCE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_DATA_CMD_DAC_MODE:
            fiio_debug("%s:Enter MEMDEV_IO_SET_DATA_CMD_DAC_MODE!\n",__func__);
            ret = __get_user(ioarg, (int *)arg);

            set_fiio_ak4377_work_mode(ioarg);
            fiio_debug("%s:Exit MEMDEV_IO_SET_DATA_CMD_DAC_MODE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_DATA_CMD_DAC_MODE:
            fiio_debug("%s:Enter MEMDEV_IO_GET_DATA_CMD_DAC_MODE!\n",__func__);
            ioarg = get_fiio_ak4377_work_mode();

            ret = __put_user(ioarg, (int *)arg);
            fiio_debug("%s:Exit MEMDEV_IO_GET_DATA_CMD_DAC_MODE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_USB_PATH_MODE:
            fiio_debug("%s:Enter MEMDEV_IO_SET_USB_PATH_MODE!\n",__func__);
			ret = __get_user(ioarg, (int *)arg);

			#ifdef CONFIG_M5_BORAD_2_0910
				select_usb_to_x1000_or_csr8675(ioarg);
			#endif
            fiio_debug("%s:Exit MEMDEV_IO_SET_USB_PATH_MODE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_USB_PATH_MODE:
            fiio_debug("%s:Enter MEMDEV_IO_GET_USB_PATH_MODE!\n",__func__);
			#ifdef CONFIG_M5_BORAD_2_0910
			ioarg = get_usb_path_mode();
			#endif

			ret = __put_user(ioarg, (int *)arg);
			break;
            fiio_debug("%s:Exit MEMDEV_IO_GET_USB_PATH_MODE ioarg=%d!\n",__func__,ioarg);
		//hw chage
		case MEMDEV_IO_SET_HW_CHANGE:
            fiio_debug("%s:Enter MEMDEV_IO_SET_HW_CHANGE!\n",__func__);
			ret = __get_user(ioarg, (int *)arg);

			#ifdef CONFIG_M5_BORAD_2_0910
				fiio_enable_hw_change(ioarg);
			#endif
            fiio_debug("%s:Exit MEMDEV_IO_SET_HW_CHANGE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_HW_CHANGE:
            fiio_debug("%s:Enter MEMDEV_IO_GET_HW_CHANGE!\n",__func__);
			#ifdef CONFIG_M5_BORAD_2_0910
			ioarg = fiio_get_hw_change_state();
			#endif

			ret = __put_user(ioarg, (int *)arg);
            fiio_debug("%s:Exit MEMDEV_IO_GET_HW_CHANGE ioarg=%d!\n",__func__,ioarg);
			break;
		
		case MEMDEV_IO_SET_PEDOMETER:
            fiio_debug("%s:Enter MEMDEV_IO_SET_PEDOMETER!\n",__func__);
            ret = __get_user(ioarg, (int *)arg);
			//#ifdef CONFIG_FIIO_SENSORS_LIS3DSH
				process_pedometer_set_event(ioarg);
			//#endif
            fiio_debug("%s:Exit MEMDEV_IO_SET_PEDOMETER ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_PEDOMETER:
            fiio_debug("%s:Enter MEMDEV_IO_GET_PEDOMETER!\n",__func__);
			//#ifdef CONFIG_FIIO_SENSORS_LIS3DSH
			ioarg = process_pedometer_get_event();
			//#endif

			ret = __put_user(ioarg, (int *)arg);
            fiio_debug("%s:Exit MEMDEV_IO_GET_PEDOMETER ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_PLAY_BT_SOURCE:
			fiio_debug("%s:Enter MEMDEV_IO_SET_PLAY_BT_SOURCE!\n",__func__);
           ret = __get_user(ioarg, (int *)arg);
			set_fiio_m5_output_select_bt(ioarg);
           fiio_debug("%s:Exit MEMDEV_IO_SET_PLAY_BT_SOURCE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_GET_PLAY_BT_SOURCE:
			fiio_debug("%s:Enter MEMDEV_IO_GET_PLAY_BT_SOURCE!\n",__func__);
			ioarg = get_fiio_m5_output_select_bt();
			ret = __put_user(ioarg, (int *)arg);
           fiio_debug("%s:Exit MEMDEV_IO_GET_PLAY_BT_SOURCE ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_BT_RESET:
			fiio_debug("%s:Enter MEMDEV_IO_SET_BT_RESET!\n",__func__);
           ret = __get_user(ioarg, (int *)arg);
			if (1 == ioarg) {
				fiio_csr8675_reset();
			}
			else if (2 == ioarg) {
				fiio_csr8675_disable();
			}
           fiio_debug("%s:Exit MEMDEV_IO_SET_BT_RESET ioarg=%d!\n",__func__,ioarg);
			break;
		case MEMDEV_IO_SET_DAC_MUTE:
			fiio_debug("%s:Enter MEMDEV_IO_SET_DAC_MUTE!\n",__func__);
			 ret = __get_user(ioarg, (int *)arg);
			if (0 == ioarg) {
				//unmute
				fiio_set_dac_volume();
			}
			else if (1 == ioarg) {
				// 1:mute
				AK4377_Mute();
			}
			fiio_debug("%s:Exit MEMDEV_IO_SET_DAC_MUTE ioarg=%d!\n",__func__,ioarg);
			break;
		default:
			printk("%s %d NO This Command!\n",__func__,__LINE__);
			break;
    }
    return ret;
}

////////////////////////////test auto bt mode//////////////////////////////////////
#ifdef CONFIG_FIIO_AUTO_ENTER_BT_MODE
struct workqueue_struct *start_bt_wq;
struct delayed_work start_bt_work;
static void start_enter_bt_mode_work(struct work_struct *work)
{
	fiio_set_bt_work_in_sink_mode();
}

void bt_test_set_en_otg5v(void) {
	gpio_direction_output(GPIO_OTG_EN,1);
	gpio_direction_output(GPIO_USB_CHANGE_EN,1);
}
EXPORT_SYMBOL(bt_test_set_en_otg5v);
void bt_test_set_disable_otg5v(void) {
	gpio_direction_output(GPIO_OTG_EN,0);
}
EXPORT_SYMBOL(bt_test_set_disable_otg5v);

#endif


struct workqueue_struct *reset_bt_wq;
struct delayed_work reset_bt_work;
static void start_reset_bt_mode_work(struct work_struct *work)
{
	switch_csr8675_to_sink_or_source(1);
	
	fiio_csr8675_reset();
}
//////////////////////////////////////////////////////////////////////////////////

static struct file_operations fiio_ctrl_opt_fops = {
	.owner	=	THIS_MODULE,
	.open	=	fiio_ctrl_open,
	.unlocked_ioctl		=	fiio_ctrl_ioctl,
	.release	= 	fiio_ctrl_release,
};

static struct miscdevice fiio_ctrl_misc_opt = {
	.minor	=	MISC_DYNAMIC_MINOR,
	.name	= 	DRIVER_NAME,
	.fops	=	&fiio_ctrl_opt_fops,
};

/**
 * Module init
 */
static int __init init_fiio_ctrl(void)
{
    //power off
    int ret = 0;
#ifdef CONFIG_M5_BORAD_2_0910
	printk("##############################################################\n");
	printk("################FiiO M5 Kernel v-1.0.0.1######################\n");
	printk("##############################################################\n");
#endif
	fiio_init_wakeup_source();

    bt_gpio_init();
    fiio_crs8675_gpio_init();

	#ifdef CONFIG_M5_BORAD_2_0910
		//usb direction
 		init_usb_select_gpio();
		//spdif
		init_spdif_output_gpio();
		set_spdif_off();
		//otg int in dwc2 module
		//init_otg_enable_gpio();

		//led
		led_gpio_init();
 	#endif

	
	
	//默认关闭蓝牙
	//fiio_csr8675_disable();
#if 0
	//burst to reset bt
	fiio_csr8675_reset();
	//{add for burst bt firmware
	//source
	switch_csr8675_to_sink_or_source(1);
	
	fiio_csr8675_no_disable();
	//}
#endif
    ret = misc_register(&fiio_ctrl_misc_opt);
	if (ret < 0)
	{
		printk(KERN_ERR"register misc device opt failed.\n");
		return ret;
	}
	fiio_ctrl_device_attr_register(&fiio_ctrl_misc_opt);

	/////////////////////////////////////////////////////////////
	#ifdef CONFIG_FIIO_AUTO_ENTER_BT_MODE
	INIT_DELAYED_WORK(&start_bt_work,  start_enter_bt_mode_work);
	start_bt_wq = create_singlethread_workqueue("start_bt_wq");
	if (start_bt_wq == NULL) {
		printk(KERN_ERR"failed to create work queue\n");
		return -ENOMEM;
	}
	queue_delayed_work(start_bt_wq,  &start_bt_work, msecs_to_jiffies(3000));
	//connect csr8675 for update bt
	//select_usb_to_x1000_or_csr8675(1);
	#endif
	/////////////////////////////////////////////////////////

	INIT_DELAYED_WORK(&reset_bt_work,  start_reset_bt_mode_work);
	reset_bt_wq = create_singlethread_workqueue("reset_bt_wq");
	if (reset_bt_wq == NULL) {
		printk(KERN_ERR"failed to create work queue\n");
		return -ENOMEM;
	}
	queue_delayed_work(reset_bt_wq,  &reset_bt_work, msecs_to_jiffies(10));
	
	return 0;
}

/**
 * Module exit
 */
static void __exit cleanup_fiio_ctrl(void)
{

	fiio_ctrl_device_attr_unregister(&fiio_ctrl_misc_opt);
    misc_deregister(&fiio_ctrl_misc_opt);
	#ifdef CONFIG_M5_BORAD_1_0629
    gpio_free(BT_PW_EN);
	#endif
    gpio_free(BT_VREG_ON);
    gpio_free(BT_RST);
	#ifdef CONFIG_M5_BORAD_2_0910
	uinstall_usb_select_gpio();
	uinstall_spdif_output_gpio();
	uinstall_otg_enable_gpio();
	#endif
}
module_init(init_fiio_ctrl);
module_exit(cleanup_fiio_ctrl);

MODULE_AUTHOR("pengweizhong@fiio.net");
MODULE_DESCRIPTION("fiio ctrl driver");
MODULE_LICENSE("GPL");
