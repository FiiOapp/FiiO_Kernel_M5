/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <soc/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>

//#include <linux/power/axp173.h>
#define LONG_PRESS_COUNT			5 //100 * 10 = 1000ms
struct input_dev *headset;
//struct timer_list mytimer;
//struct axp173 *axp192 = axp173;
//static unsigned int long_press_count1;
//static bool key_longpressed;
//static unsigned int longpressed;
struct gpio_button_data {
	int long_press_count;
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_longpressed;
	unsigned int longpressed;
	unsigned int long_press_count1;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	const struct gpio_keys_platform_data *pdata;
	struct input_dev *input;
	struct mutex disable_lock;
	struct gpio_button_data data[0];
};
static void keys_long_press_timer(unsigned long _data);
/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(bdata->irq);
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
		if(bdata->button->code_long_press)
		__set_bit(bdata->button->code_long_press, bits);	
}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordingly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, true);
ATTR_SHOW_FN(switches, EV_SW, true);
ATTR_SHOW_FN(disabled_keys, EV_KEY, false);
ATTR_SHOW_FN(disabled_switches, EV_SW, false);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);
#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type =  EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;

	//if (type == EV_ABS) {
	/*	if (state)
			input_event(input, type, button->code, button->value);
	} else {
		input_event(input, type, button->code, !!state);
	}*/
	//unsigned long flags;
	//spin_lock_irqsave(&bdata->lock,flags);
		if(state)
		{
		//printk(KERN_WARNING" find a KEY  pressed !!! \n");
		bdata->key_longpressed=1; 
		mod_timer(&bdata->timer, jiffies + HZ/10);
		//code_longpressed=button->code_long_press;
		//add_timer(&bdata->timer);
		}
	else {
		//bdata->key_longpressed=0;
		
		//bdata->long_press_count1 =0;
		if(bdata->longpressed){
		//printk(KERN_WARNING"KEY %d is  longreleased !!! \n",button->code_long_press);
		input_event(input, type,button->code_long_press, 0);
                       // input_sync(input);
		bdata->key_longpressed=0;
		bdata->longpressed=0;
	//	add_timer(&mytimer);	
		}else if((!bdata->longpressed)&&(bdata->key_longpressed)) {
			//printk(KERN_WARNING"KEY %d is  shortpressed !!! \n",button->code);
			bdata->key_longpressed=0;
			input_event(input,type, button->code,1);
                        //input_sync(input);
			//printk(KERN_WARNING"KEY %d is  shortreleased !!! \n",button->code);
                        input_event(input,type,button->code, 0);
                        //input_sync(input);
		}
		//del_timer(&bdata->timer);
	
		//}
	input_sync(input);
	 bdata->key_longpressed=0;

         bdata->long_press_count1 =0;
	//spin_unlock_irqrestore(&bdata->lock,flags);
	}
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);

	gpio_keys_gpio_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}
static void keys_long_press_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct gpio_keys_button *button = bdata->button;
        struct input_dev *input = bdata->input;
	unsigned int type =  EV_KEY;
	//int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;
	//printk(KERN_WARNING"enter %s,long_press=%d \n",__func__,state);
	//mod_timer(&bdata->timer, jiffies + HZ/10);
	//unsigned long flags;
	//spin_lock_irqsave(&bdata->lock,flags);
	if(bdata->key_longpressed){
		//printk(KERN_WARNING"enter  longpressed2 !!! \n");
		//key_longpressed=1;
		bdata->long_press_count1++;
		mod_timer(&bdata->timer, jiffies + HZ/10);
		//printk(KERN_WARNING"long_press_count=%d !!! \n",long_press_count1);

               // mod_timer(&mytimer,
                 //               jiffies + HZ/10);
	if(button->code == KEY_POWER){
	if(bdata->long_press_count1 == 10)
                {
                      //printk(KERN_WARNING"power key long pressed \n");
                       // printk(KERN_WARNING"KEY %d  is  longpressed !!! \n",button->code_long_press);
                        bdata->longpressed=1;
                        input_event(input, type,button->code_long_press, 1);
                        input_sync(input);
                }
	}
	else if(bdata->long_press_count1 == 5)
		{
		//	printk(KERN_WARNING"KEY  is  longpressed !!! \n");
			//printk(KERN_WARNING"KEY %d  is  longpressed !!! \n",button->code_long_press);
			bdata->longpressed=1;
			input_event(input, type,button->code_long_press, 1);
                        input_sync(input);
		}
	
	}
	//spin_unlock_irqrestore(&bdata->lock,flags);
	/*else {
		if((key_longpressed==1)&&(long_press_count1 < 5)){
			long_press_count1 = 0;
			key_longpressed=0;
			printk(KERN_WARNING"KEY %d is  shortpressed !!! \n",button->code);
                        input_event(input,type, button->code, 1);
                        input_sync(input);
                        printk(KERN_WARNING"KEY %d is  shortreleased !!! \n",button->code);
                        input_event(input,type,button->code, 0);
                        input_sync(input);
			}
		else if((key_longpressed==1)&&(long_press_count1 > 5)){
			printk(KERN_WARNING"KEY %d is  longreleased !!! \n",(button->code)+4);
                input_event(input, type,(button->code)+4, 0);
                        input_sync(input);
			key_longpressed=0;
			long_press_count1=0;
			}
		}*/
}

static void gpio_keys_gpio_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	//struct gpio_keys_button *button = bdata->button;
        //struct input_dev *input = bdata->input;
        //unsigned int type =  EV_KEY;
	//printk(KERN_WARNING"enter gpio timer! \n");
	/*if(key_longpressed){
                //printk(KERN_WARNING"enter  longpressed2 !!! \n");
                long_press_count1++;
               // mod_timer(&bdata->timer, jiffies + HZ/10);
                //printk(KERN_WARNING"long_press_count=%d !!! \n",long_press_count1);

               // mod_timer(&mytimer,
                 //               jiffies + HZ/10);

        if(long_press_count1 == 5)
                {
                //      printk(KERN_WARNING"KEY  is  longpressed !!! \n");
                        printk(KERN_WARNING"KEY %d  is  longpressed !!! \n",(button->code)+4);
                        longpressed=1;
                        input_event(input, type,(button->code)+4, 1);
                        input_sync(input);
                }

        }
*/
	schedule_work(&bdata->work);
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);
	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
	//if(1)
		//mod_timer(&bdata->timer, jiffies + HZ/10);
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	unsigned long flags;
	//printk(KERN_WARNING"enter irq timer! \n");
	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, bdata->button->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;
	unsigned int type =  EV_KEY;
	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);
	//printk(KERN_WARNING"enter irq isr! \n");
	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);
		
	/*	 key_longpressed=1;
                //code_longpressed=button->code_long_press;
                add_timer(&bdata->timer);
                }
        else {
                key_longpressed=0;
                long_press_count1 =0;
                if(longpressed){
                printk(KERN_WARNING"KEY %d is  longreleased !!! \n",(button->code)-3);
                input_event(input, type, (button->code)-3, 0);
                        input_sync(input);
                longpressed=0;*/
		//input_event(input, EV_KEY, button->code, 1);
		//input_sync(input);

		if (!bdata->timer_debounce) {/*
			printk(KERN_WARNING"KEY %d is  shortpressed !!! \n",button->code);
                        input_event(input,type, button->code, 1);
                        input_sync(input);
                        printk(KERN_WARNING"KEY %d is  shortreleased !!! \n",button->code);
                        input_event(input,type,button->code, 0);
                        input_sync(input);
                }
                del_timer(&bdata->timer);
*/
			//input_event(input, EV_KEY, button->code, 0);
			//input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
		//bdata->key_longpressed=true;
	}
	//mod_timer(&bdata->timer, jiffies + HZ/10);
	if (bdata->timer_debounce)
		//mod_timer(&bdata->timer, jiffies + HZ/10);
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}
/*
static void headset_control(unsigned long data)

{
	struct axp173 *axp192 = axp173;
        printk("%s\n", (char *)data);
	//struct axp173 *axp192 = (struct axp173 *)data;
	if(axp173_i2c_read_bit(axp192,0x94,6))
		printk("headset remove!\n");
	//else if(!axp173_i2c_read_bit(axp173,0x94,6))
	//	printk("headset inser!\n");
        mod_timer(&mytimer, jiffies + HZ/2);

}
*/
void fiio_send_input_event(int event) {
	if (headset != NULL) {
		input_event(headset,EV_KEY,KEY_SCROLLLOCK, event);
        input_sync(headset);
	}
	else {
		printk("*********headset is NULL!\n");
	}
}
EXPORT_SYMBOL_GPL(fiio_send_input_event);


static int gpio_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_button_data *bdata,
				const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (gpio_is_valid(button->gpio)) {

		error = gpio_request_one(button->gpio, GPIOF_IN, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		if (button->debounce_interval) {
			error = gpio_set_debounce(button->gpio,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->timer_debounce =
						button->debounce_interval;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}
		bdata->irq = irq;

		INIT_WORK(&bdata->work, gpio_keys_gpio_work_func);
		if(button->code_long_press){
			setup_timer(&bdata->timer,
				keys_long_press_timer, (unsigned long)bdata);
		bdata->timer.expires = jiffies + HZ/10;
		//add_timer(&bdata->timer);
		}
	        //add_timer(&mytimer);

		else if (button->code)
		setup_timer(&bdata->timer,
			    gpio_keys_gpio_timer, (unsigned long)bdata);
		isr = gpio_keys_gpio_isr;
		if (button->gpio == BT_INT1) {
			irqflags = IRQF_TRIGGER_FALLING;
		}
		else {
			irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
		}
		

	} else {
		if (!button->irq) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->timer_debounce = button->debounce_interval;
		setup_timer(&bdata->timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}

	input_set_capability(input, EV_KEY, KEY_SCROLLLOCK);
        input_set_capability(input, EV_KEY, KEY_KP4);
	input_set_capability(input, EV_KEY, KEY_KP7);
	input_set_capability(input, EV_KEY, KEY_KP8);
	input_set_capability(input, EV_KEY, KEY_KP9);
	input_set_capability(input, EV_KEY, KEY_KPMINUS);
	//mytimer.expires = jiffies + HZ/10;
	//add_timer(&mytimer);
	if(button->code_long_press)
	input_set_capability(input, button->type ?: EV_KEY, button->code_long_press);
	input_set_capability(input, button->type ?: EV_KEY, button->code);

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		goto fail;
	}
	if(button->gpio_pullup){/* need gpio inter pull up*/
		error = jzgpio_ctrl_pull(button->gpio / 32,1,BIT(button->gpio % 32));
		if(error < 0)
			dev_err(dev, "Failed to set gpio pull ! GPIO %d, error %d\n",button->gpio,error);
	}

	return 0;

fail:
	if (gpio_is_valid(button->gpio))
		gpio_free(button->gpio);

	return error;
}

static void gpio_keys_report_state(struct gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		//add_timer(&bdata->timer);
		if (gpio_is_valid(bdata->button->gpio))
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}
	/* Report current state of buttons that are connected to GPIOs */
	gpio_keys_report_state(ddata);

	return 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node) {
		error = -ENODEV;
		goto err_out;
	}

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0) {
		error = -ENODEV;
		goto err_out;
	}

	pdata = kzalloc(sizeof(*pdata) + nbuttons * (sizeof *button),
			GFP_KERNEL);
	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	pdata->buttons = (struct gpio_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);

	i = 0;
	for_each_child_of_node(node, pp) {
		int gpio;
		enum of_gpio_flags flags;

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->nbuttons--;
			dev_warn(dev, "Found button without gpios\n");
			continue;
		}

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (gpio < 0) {
			error = gpio;
			if (error != -EPROBE_DEFER)
				dev_err(dev,
					"Failed to get gpio flags, error: %d\n",
					error);
			goto err_free_pdata;
		}

		button = &pdata->buttons[i++];

		button->gpio = gpio;
		button->active_low = flags & OF_GPIO_ACTIVE_LOW;

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->gpio);
			error = -EINVAL;
			goto err_free_pdata;
		}

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = !!of_get_property(pp, "gpio-key,wakeup", NULL);

		if (of_property_read_u32(pp, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0) {
		error = -EINVAL;
		goto err_free_pdata;
	}

	return pdata;

err_free_pdata:
	kfree(pdata);
err_out:
	return ERR_PTR(error);
}

static struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

#else

static inline struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->timer_debounce)
		{
		printk("remove timer!\n");
		del_timer_sync(&bdata->timer);
		}
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->button->gpio))
		gpio_free(bdata->button->gpio);
}

static int gpio_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	//struct axp173 *axp192 = axp173;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;
	}
/*	
	setup_timer(&mytimer, headset_control, (unsigned long)"headset control");

        mytimer.expires = jiffies + HZ/2;

        //add_timer(&mytimer);
*/
	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail3;
	}
	
	wakeup = 1;
	device_init_wakeup(&pdev->dev, wakeup);
	headset = input;
	return 0;

 fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
 fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);
	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(pdata);

	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->pdata->nbuttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);

	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(ddata->pdata);

	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
extern int get_m5_player_mode_for_gpio_keys(void);

static int can_disable_bt_irq_wake_flag = 0;
static int can_disable_global_irq_wake_flag = 0;

static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int i;
	//printk("##############%s enter!\n",__func__);
	if (device_may_wakeup(dev)) {
		printk("##############%s!\n",__func__);
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
			//BT_INT1
			//printk("##############%s bdata->button->gpio=%d BT_INT1=%d GPIO_ENDCALL_KEY=%d!\n",__func__,bdata->button->gpio,BT_INT1,GPIO_ENDCALL_KEY);
			if (bdata->button->gpio == BT_INT1) {
                if (FIIO_M5_PLAYER_BT_SINK == get_m5_player_mode_for_gpio_keys()) {
					//printk("##############%s enable_irq_wake BT_INT1!\n",__func__);
					enable_irq_wake(bdata->irq);
					can_disable_bt_irq_wake_flag = 1;
				}
			}
			//ENDCALL
			if (bdata->button->gpio == GPIO_ENDCALL_KEY) {
                if (FIIO_M5_PLAYER_NORMAL == get_m5_player_mode_for_gpio_keys()) {
					//printk("##############%s enable_irq_wake ACTIVE_LOW_ENDCALL!\n",__func__);
					enable_irq_wake(bdata->irq);
					can_disable_global_irq_wake_flag = 1;
				}
			}
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			gpio_keys_close(input);
		mutex_unlock(&input->mutex);
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	int error = 0;
	int i;
	//resume event
	//printk("resume event :key code 75\n");
        //input_event(input,EV_KEY, KEY_KP4,1);
          //              input_event(input,EV_KEY,KEY_KP4, 0);
            //            input_sync(input);

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->pdata->nbuttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				disable_irq_wake(bdata->irq);
			//BT_INT1
			if (bdata->button->gpio == BT_INT1) {
				if (1 == can_disable_bt_irq_wake_flag) {
					//printk("##############%s disable_irq_wake BT_INT1!\n",__func__);
					can_disable_bt_irq_wake_flag = 0;
					disable_irq_wake(bdata->irq);
				}
			}
			//ENDCALL
			if (bdata->button->gpio == GPIO_ENDCALL_KEY) {
				if (1 == can_disable_global_irq_wake_flag) {
					//printk("##############%s disable_irq_wake ACTIVE_LOW_ENDCALL!\n",__func__);
					can_disable_global_irq_wake_flag = 0;
					disable_irq_wake(bdata->irq);
				}
			}
		}
	} else {
		mutex_lock(&input->mutex);
		if (input->users)
			error = gpio_keys_open(input);
		mutex_unlock(&input->mutex);
	}

	if (error)
		return error;
	mutex_lock(&input->mutex);
	gpio_keys_report_state(ddata);
	mutex_unlock(&input->mutex);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = of_match_ptr(gpio_keys_of_match),
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
	
	//del_timer(&mytimer);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-keys");
