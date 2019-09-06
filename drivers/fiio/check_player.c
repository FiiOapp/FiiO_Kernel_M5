#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <soc/gpio.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
//debug
#define FIIO_DEBUG_CHECK_PLAYER
#ifdef FIIO_DEBUG_CHECK_PLAYER
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_uevent] " x)
#else
#define fiio_debug(x...)
#endif

//第一次开机后延时检测
#define DELAY_WORK_JIFFIES        (10)//ms
static struct delayed_work check_player_work;
static struct workqueue_struct *check_player_workqueue = NULL;
static int irp_player_state = 0,queue_player_state=0;
extern void fiio_remotectl_event_send_player(char* uevent_name,int uevent_type);
static irqreturn_t process_isr(int irq, void *dev_id)
{
	irp_player_state = gpio_get_value(BT_PLAY_DETET);
	fiio_debug("++++%s irp_player_state:%d\n",__func__,irp_player_state);
	queue_delayed_work(check_player_workqueue,&check_player_work, DELAY_WORK_JIFFIES);
	return IRQ_HANDLED;
}

static void check_player_func(struct work_struct *work)
{
	
	queue_player_state = gpio_get_value(BT_PLAY_DETET);
	fiio_debug("++++%s queue_player_state:%d irp_player_state:%d\n",__func__,queue_player_state,irp_player_state);
	if (irp_player_state == queue_player_state) 
	{
		if (0 == irp_player_state) 
		{
			fiio_debug("%s no insert\n",__func__);
			fiio_remotectl_event_send_player("FIIO_PLAYER_MODE",0);
		}
		else 
		{
			fiio_debug("%s insert player\n",__func__);
			fiio_remotectl_event_send_player("FIIO_PLAYER_MODE",1);
		}
	}
}

static int check_player_probe(struct platform_device *pdev) {
    u8 irq, error;
   
    if (gpio_is_valid(BT_PLAY_DETET)) {
        error = gpio_request_one(BT_PLAY_DETET, GPIOF_IN, "check_player");
		if (error < 0) {
			printk("Failed to request GPIO %d, error %d\n",
				BT_PLAY_DETET, error);
			return error;
		}

        irq = gpio_to_irq(BT_PLAY_DETET);
		if (irq < 0) {
			error = irq;
			printk("Unable to get irq number for GPIO %d, error %d\n",
				BT_PLAY_DETET, error);
			goto fail;
		}
    }

	//get io state
	irp_player_state = gpio_get_value(BT_PLAY_DETET);
	fiio_debug("++++%s irp_player_state:%d\n",__func__,irp_player_state);
	
	INIT_DELAYED_WORK(&check_player_work, check_player_func);
    fiio_debug("%s %d.\n",__func__,__LINE__);
    check_player_workqueue = create_singlethread_workqueue("check_player");
	fiio_debug("%s %d.\n",__func__,__LINE__);
	//check player after 10ms
	queue_delayed_work(check_player_workqueue,&check_player_work, DELAY_WORK_JIFFIES);

    error = request_any_context_irq(irq, process_isr,  (IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), pdev->name, NULL);
    if (error < 0) {
        printk("Unable to claim irq %d; error %d\n",irq, error);
        goto fail;
    }
	fiio_debug("%s %d error=%d.\n",__func__,__LINE__,error);
	return 0;
	fail:
		return error;
}

static int check_player_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
extern void fiio_set_wakeup_source();
extern void fiio_clear_wakeup_source();
static int check_palyer_suspend(struct platform_device *pdev,
                  pm_message_t state)
{
	fiio_set_wakeup_source();
    return 0;
}
static int check_palyer_resume(struct platform_device *pdev)
{
	fiio_clear_wakeup_source();
    return 0;
}
#else
#define check_palyer_suspend NULL
#define check_palyer_resume NULL
#endif /* CONFIG_PM */


static struct platform_driver check_palyer_driver = {
	.probe		= check_player_probe,
	.remove		= check_player_remove,
	.driver		= {
		.name	= "check_player",
		.owner	= THIS_MODULE,
	},
	#ifdef CONFIG_PM
		.suspend = check_palyer_suspend,
		.resume = check_palyer_resume,
	#endif
};


struct platform_device check_player_device = {
    .name           = "check_player",
    .id             = -1,
};

//dev/check_player
static int __init check_player_init(void)
{
	int ret;
    //平台设备
    ret = platform_device_register(&check_player_device);
    //平台设备驱动
    ret =  platform_driver_register(&check_palyer_driver);
	return ret;
}

static void __exit check_player_exit(void)
{
    platform_driver_unregister(&check_palyer_driver);
    platform_device_unregister(&check_player_device);
}


MODULE_AUTHOR("pengweizhong <pengweizhong@fiio.net>");
MODULE_DESCRIPTION("check m5 play mode driver");
MODULE_LICENSE("GPL");

module_init(check_player_init);
module_exit(check_player_exit);
