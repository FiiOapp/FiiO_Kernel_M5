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

#include "fiio_i2c_interface.h"
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
#include "stk8323_fiio.h"

#define FIIO_DEBUG_SENSOR
#ifdef FIIO_DEBUG_SENSOR
#define fiio_debug(x...)  printk(KERN_INFO "[stk8323] " x)
#else
#define fiio_debug(x...)
#endif

#define DEVICE_NAME "stk8323"

static struct i2c_client *g_client;

struct stk8323_data {
	const char *name;
	u16 bustype;
	struct device *dev;
	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
};

////////////////////////////////////////////////////////////////////////////////

static irqreturn_t stk8323_isr1(int irq, void *dev)
{
	struct stk8323_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);

	return IRQ_HANDLED;
}

static irqreturn_t stk8323_isr2(int irq, void *dev)
{
	struct stk8323_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);

	return IRQ_HANDLED;
}

static void stk8323_irq1_work_func(struct work_struct *work)
{
	struct stk8323_data *acc;

	acc = container_of(work, struct stk8323_data, irq1_work);
	fiio_debug("%s: IRQ1 triggered\n", DEVICE_NAME);
	fiio_debug("%s INTSTS1=0x%02x\n",__func__,fiio_i2c_read_byte(g_client,INTSTS1));
	fiio_debug("%s INTSTS2=0x%02x\n",__func__,fiio_i2c_read_byte(g_client,INTSTS2));
	enable_irq(acc->irq1);
}

static void stk8323_irq2_work_func(struct work_struct *work)
{
	struct stk8323_data *acc;

	acc = container_of(work, struct stk8323_data, irq2_work);

	fiio_debug("%s: IRQ2 triggered\n", DEVICE_NAME);

	enable_irq(acc->irq2);
}

void stk8323_set_power_mode(int mode) {
	switch(mode) {
		case 0://suspend
			fiio_i2c_write_bit(g_client,POWMODE,0,5);
			fiio_i2c_write_bit(g_client,POWMODE,0,6);
			fiio_i2c_write_bit(g_client,POWMODE,1,7);
			break;
		case 1://low power
			fiio_i2c_write_bit(g_client,POWMODE,0,5);
			fiio_i2c_write_bit(g_client,POWMODE,0,7);
			fiio_i2c_write_bit(g_client,POWMODE,1,6);
			break;
		case 2://sleep
			fiio_i2c_write_bit(g_client,POWMODE,0,6);
			fiio_i2c_write_bit(g_client,POWMODE,0,7);
			fiio_i2c_write_bit(g_client,POWMODE,1,5);
			break;
		
	}
}
EXPORT_SYMBOL(stk8323_set_power_mode);

static void stk8323_init(struct i2c_client *client) {
	fiio_debug("%s STEPCNT1=%d",__func__,fiio_i2c_read_byte(client,STEPCNT1));
	fiio_i2c_write_byte(client,STEPCNT1,0);
	
	fiio_i2c_write_bit(client,STEPCNT2,1,3);
	//STEPOUT1 (0x0D) and STEPOUT2 (0x2E) full STEPOVR2INT1(3)
	fiio_i2c_write_bit(client,INTMAP1,1,3); //STEPOVR2INT1
	fiio_i2c_write_bit(client,INTMAP1,1,1);//STEP2INT1
	fiio_i2c_write_bit(client,INTMAP1,1,0);//SIGMOT2INT1
	
	fiio_i2c_write_bit(client,INTMAP2,1,0);//DATA2INT1
	fiio_i2c_write_bit(client,INTMAP2,1,1);//FWM2INT1
	fiio_i2c_write_bit(client,INTMAP2,1,2);//FFULL2INT1

	//This register is used to define the INT1 and INT2 pins output type and active level
	fiio_i2c_write_bit(client,INTCFG1,1,0);//active high	
	//fiio_i2c_write_bit(client,INTCFG1,1,1);//Open-drain output type
	fiio_i2c_write_bit(client,INTCFG1,0,1);//Push-pull output type

	fiio_i2c_write_bit(client,INTEN1,1,0);//SLP_EN_X
	fiio_i2c_write_bit(client,INTEN1,1,1);//SLP_EN_Y
	fiio_i2c_write_bit(client,INTEN1,1,2);//SLP_EN_Z

	fiio_i2c_write_bit(client,INTEN2,1,4);//DATA_EN
	fiio_i2c_write_bit(client,INTEN2,1,5);//FFULL_EN
	fiio_i2c_write_bit(client,INTEN2,1,6);//FWM_EN

	fiio_debug("%s STEPCNT1=%d\n",__func__,fiio_i2c_read_byte(client,STEPCNT1));
	fiio_debug("%s STEPCNT2=%d\n",__func__,fiio_i2c_read_byte(client,STEPCNT2));
	fiio_debug("%s INTMAP1=0x%02x\n",__func__,fiio_i2c_read_byte(client,INTMAP1));
	fiio_debug("%s INTMAP2=0x%02x\n",__func__,fiio_i2c_read_byte(client,INTMAP2));
	fiio_debug("%s INTCFG1=0x%02x\n",__func__,fiio_i2c_read_byte(client,INTCFG1));
	fiio_debug("%s INTEN1=0x%02x\n",__func__,fiio_i2c_read_byte(client,INTEN1));
	fiio_debug("%s INTEN2=0x%02x\n",__func__,fiio_i2c_read_byte(client,INTEN2));
	
}
int stk8323_get_step(void) {
	int cnt_l,cnt_h,counter;
	cnt_l = fiio_i2c_read_byte(g_client,STEPOUT_L);
	cnt_h = fiio_i2c_read_byte(g_client,STEPOUT_H);
	counter = cnt_h*256+cnt_l;
	fiio_debug("%s cnt_l=%d cnt_h=%d counter=%d",__func__,cnt_l,cnt_h,counter);
	return counter;
}
EXPORT_SYMBOL(stk8323_get_step);

void stk8323_reset(void) {
	//reset STEP_COUNTER_OUT 0000H
	fiio_i2c_write_bit(g_client,STEPCNT2,1,2);
	
	fiio_i2c_write_byte(g_client,STEPCNT1,1);
	fiio_i2c_write_bit(g_client,STEPCNT2,1,3);
}
EXPORT_SYMBOL(stk8323_reset);

static int stk8323_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int err;
	int counter = 0;
	fiio_debug("%s:Enter!!!\n",__func__);
	
	struct stk8323_data *stat;
	
	stat = kmalloc(sizeof(struct stk8323_data), GFP_KERNEL);
	if (!stat)
		return -ENOMEM;

	stat->dev = &client->dev;
	stat->name = client->name;
	stat->bustype = BUS_I2C;
	i2c_set_clientdata(client, stat);

	stat->irq1 = gpio_to_irq(STK8323_INT1);
	stat->irq2 = gpio_to_irq(STK8323_INT2);
	stat->irq1_work_queue = create_singlethread_workqueue("irq1_work_queue");
	if (stat->irq1_work_queue == NULL) {
		printk("failed to create work queue\n");
		return -ENOMEM;
	}
	INIT_WORK(&stat->irq1_work,  stk8323_irq1_work_func);

	stat->irq2_work_queue = create_singlethread_workqueue("irq2_work_queue");
	if (stat->irq2_work_queue == NULL) {
		printk("failed to create work queue\n");
		return -ENOMEM;
	}
	INIT_WORK(&stat->irq2_work,stk8323_irq2_work_func);
	
	//queue_delayed_work(stat->irq1_work_queue,  &stat->irq1_work, msecs_to_jiffies(1000));

	
	g_client = client;
	

	err = request_irq(stat->irq1, stk8323_isr1,
                 IRQF_TRIGGER_RISING, "stk8323_isr1", stat);
    if (err < 0) {
            printk("request irq failed: %d\n", err);
           return -ENOMEM;
    }

	err = request_irq(stat->irq2, stk8323_isr2,
                 IRQF_TRIGGER_RISING, "stk8323_isr2", stat);
    if (err < 0) {
            printk("request irq failed: %d\n", err);
            return -ENOMEM;
    }

	stk8323_init(client);
	//get chip id
	fiio_debug("stk8323 chip ID=0x%02x\n",fiio_i2c_read_byte(client, CHIPID));
	fiio_debug("%s:Exit!!!\n",__func__);
	return 0;

free_data:

	return err;
}

static int stk8323_i2c_remove(struct i2c_client *client)
{

	return 0;
}

#ifdef CONFIG_PM
static int stk8323_suspend(struct device *dev)
{
	
	return 0;
}

static int stk8323_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops stk8323_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk8323_suspend, stk8323_resume)
};

#define STK8323_PM_OPS	(&stk8323_pm_ops)
#else /* CONFIG_PM */
#define STK8323_PM_OPS	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id stk8323_ids[] = {
	{ DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stk8323_ids);

#ifdef CONFIG_OF
static const struct of_device_id stk8323_id_table[] = {
	{ .compatible = "st,stk8323", },
	{ },
};
MODULE_DEVICE_TABLE(of, stk8323_id_table);
#endif

static struct i2c_driver stk8323_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
#ifdef CONFIG_PM
		.pm = STK8323_PM_OPS,
#endif
#ifdef CONFIG_OF
		.of_match_table = stk8323_id_table,
#endif
	},
	.remove = stk8323_i2c_remove,
	.probe = stk8323_i2c_probe,
	.id_table = stk8323_ids,
};

module_i2c_driver(stk8323_i2c_driver);

MODULE_DESCRIPTION("stk8323 i2c driver");
MODULE_AUTHOR("pengweizhong@fiio.net");
MODULE_LICENSE("GPL");


