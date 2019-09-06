/*
 * st1615 Touchscreen Controller Driver
 *
 * Copyright (C) 2010 Renesas Solutions Corp.
 *	Tony SIM <chinyeow.sim.xt@renesas.com>
 *
 * Using code from:
 *  - android.git.kernel.org: projects/kernel/common.git: synaptics_i2c_rmi.c
 *	Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/input/st1615.h>

//#define CONFIG_ST1615_UPGRADE_FIRMWARE

//#define FIIO_DEBUG_TP
#ifdef FIIO_DEBUG_TP
#define fiio_debug(x...)  printk(KERN_INFO "[st1615_ts] " x)
#else
#define fiio_debug(x...)
#endif


struct st1615_ts_finger {
	u16 x;
	u16 y;
	u8 t;
	bool is_valid;
};

struct st1615_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct st1615_ts_finger finger[CFG_MAX_TOUCH_POINTS];
	struct dev_pm_qos_request low_latency_req;
	struct st1615_platform_data *pdata;
	struct work_struct  work;
	struct workqueue_struct *workqueue;
	#ifdef CONFIG_ST1615_UPGRADE_FIRMWARE
		struct work_struct  upgrade_work;
		struct workqueue_struct *upgrade_workqueue;
	#endif
	int irq;
};
int st1615_i2c_Read(struct st1615_ts_data *ts, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;
	struct i2c_client *client = ts->client;
	
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int st1615_i2c_write(struct st1615_ts_data *ts,char *writebuf, int writelen) {
	int ret;
	struct i2c_client *client = ts->client;
	
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

static int st1615_ts_read_fw_version(struct st1615_ts_data *ts) {
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int error;
	u8 start_reg;
	u8 buf[4];
	int x,y;
	/* read touchscreen data from st1615 */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x0C;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	error = i2c_transfer(client->adapter, msg, 2);
	if (error < 0)
		return error;
	
	fiio_debug("%s tp version[0x%02x 0x%02x 0x%02x 0x%02x]\n",__func__,buf[0],buf[1],buf[2],buf[3]);
	return 0;
}

static int st1615_ts_read_xy_res(struct st1615_ts_data *ts) {
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int error;
	u8 start_reg;
	u8 buf[3];
	int x,y;
	/* read touchscreen data from st1615 */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x04;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	error = i2c_transfer(client->adapter, msg, 2);
	if (error < 0)
		return error;
	x = ((buf[0] & 0x0070) << 4) | buf[1];
	y = ((buf[0] & 0x0007) << 8) | buf[2];
	fiio_debug("%s x=%d y=%d\n",__func__,x,y);
	return 0;
}

static void st1615_tx_init_xy_res(struct st1615_ts_data *ts) {
	u8 buf[10];
	buf[0] = 0x04;
	buf[1] = 0;
	buf[2] = ts->pdata->x_max;
	buf[3] = ts->pdata->y_max;
	st1615_i2c_write(ts,buf,4);
	
}
static int st1615_ts_read_data(struct st1615_ts_data *ts)
{
	struct st1615_ts_finger *finger = ts->finger;
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int error;
	u8 start_reg;
	u8 buf[10];

	/* read touchscreen data from st1615 */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x10;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	error = i2c_transfer(client->adapter, msg, 2);
	if (error < 0)
		return error;

	/* get "valid" bits */
	finger[0].is_valid = buf[2] >> 7;
	#if 0
	finger[1].is_valid = buf[5] >> 7;
	#endif
	/* get xy coordinate */
	if (finger[0].is_valid) {
		finger[0].x = ((buf[2] & 0x0070) << 4) | buf[3];
		finger[0].y = ((buf[2] & 0x0007) << 8) | buf[4];
		finger[0].t = buf[8];
		//fiio_debug("%s :get buf[2]=0x%02x buf[3]=0x%02x buf[4]=0x%02x finger[0].x=%d finger[0].y=%d finger[0].t=%d\n"
		//	,__func__,buf[2],buf[3],buf[4],finger[0].x,finger[0].y,finger[0].t);
	}

#if 0
	if (finger[1].is_valid) {
		finger[1].x = ((buf[5] & 0x0070) << 4) | buf[6];
		finger[1].y = ((buf[5] & 0x0007) << 8) | buf[7];
		finger[1].t = buf[9];
	}
#endif
	return 0;
}

extern int fiio_get_lcd_rotate();
static void st1615_report_value(struct st1615_ts_data *ts) 
{
	struct st1615_ts_finger *finger = ts->finger;
	struct input_dev *input_dev = ts->input_dev;
	int i=0,count=0;
	u8 convert_x = 0;
	u8 convert_y = 0;
	u8 report_convert_x = 0;
	u8 report_convert_y = 0;
	int lcd_rotate_flag = 0;
	/* multi touch protocol */
	lcd_rotate_flag = fiio_get_lcd_rotate();
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		if (!finger[i].is_valid)
			continue;
		fiio_debug("%s :get x=%d y=%d\n",__func__,finger[i].x,finger[i].y);
		#if 0
		//第一版本的TP，没升级固件之前需要处理坐标
		convert_x = (finger[i].x*3)/4;
		//convert_y = (finger[i].y*3)/4;
		convert_y = (finger[i].y*240)/254;
		convert_y = 240-convert_y;
		//fiio_debug("%s :get convert_x=%d convert_y=%d lcd_rotate_flag=%d\n",__func__,convert_x,convert_y,lcd_rotate_flag);
		#else
		//升级固件后，坐标已经正确，（0，0）（240，240）
		convert_x = finger[i].x;
		convert_y = finger[i].y;
		#endif
		
		if (1 == lcd_rotate_flag) {
			//90
			report_convert_x = 240-convert_y;
			report_convert_y = convert_x;
			
		}
		else if (2 == lcd_rotate_flag) {
			report_convert_x = 240-convert_x;
			report_convert_y = 240-convert_y;
		}
		else if (3 == lcd_rotate_flag) {
			//270
			report_convert_x = convert_y;
			report_convert_y = 240-convert_x;
		}
		else {
			report_convert_x = convert_x;
			report_convert_y = convert_y;
		}
		fiio_debug("%s :repory x=%d y=%d lcd_rotate_flag=%d\n",__func__,report_convert_x,report_convert_y,lcd_rotate_flag);
		#ifdef CONFIG_FIIO_TP_BUILD_AS_TOUCH
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_report_abs(input_dev, ABS_X, report_convert_x);
			input_report_abs(input_dev, ABS_Y, report_convert_y);
			input_report_abs(input_dev, ABS_PRESSURE, 1);
			input_sync(input_dev);
		#else
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[i].t);
			input_report_abs(input_dev, ABS_MT_POSITION_X, report_convert_x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, report_convert_y);
			input_mt_sync(input_dev);
		#endif
		
		#if 0
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[i].t);
		input_report_abs(input_dev, ABS_MT_POSITION_X, finger[i].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[i].y);
		input_mt_sync(input_dev);
		fiio_debug("%s :repory x=%d y=%d\n",__func__,finger[i].x,finger[i].y);
		#endif
		count++;
	}

	/* SYN_MT_REPORT only if no contact */
	if (!count) {
		#ifdef CONFIG_FIIO_TP_BUILD_AS_TOUCH
			input_report_abs(input_dev, ABS_PRESSURE, 0);
			input_report_key(input_dev, BTN_TOUCH, 0);
		#endif
		input_mt_sync(input_dev);
		if (ts->low_latency_req.dev) {
			dev_pm_qos_remove_request(&ts->low_latency_req);
			ts->low_latency_req.dev = NULL;
		}
	} else if (!ts->low_latency_req.dev) {
		/* First contact, request 100 us latency. */
		dev_pm_qos_add_ancestor_request(&ts->client->dev,
						&ts->low_latency_req, 100);
	}

	/* SYN_REPORT */
	input_sync(input_dev);
}

static void st1615_work_handler(struct work_struct *work)
{
	struct st1615_ts_data *st1615_ts = container_of(work, struct st1615_ts_data, work);
	int ret = 0;
	ret = st1615_ts_read_data(st1615_ts);
	if (ret == 0)
		st1615_report_value(st1615_ts);
	enable_irq(st1615_ts->irq);
}

#ifdef CONFIG_ST1615_UPGRADE_FIRMWARE
extern int st1615_upgrade_fw(struct i2c_client *client);

static void st1615_upgrade_work_handler(struct work_struct *work)
{
	struct st1615_ts_data *st1615_ts = container_of(work, struct st1615_ts_data, upgrade_work);
	fiio_debug("%s:Enter !!!\n",__func__);  	
	st1615_upgrade_fw(st1615_ts->client);
	fiio_debug("%s:Exit !!!\n",__func__);  	
}

#endif


static irqreturn_t st1615_ts_irq_handler(int irq, void *dev_id)
{
	struct st1615_ts_data *ts = dev_id;

	disable_irq_nosync(ts->irq);

	if (!work_pending(&ts->work)) {
		queue_work(ts->workqueue, &ts->work);
	} else {
		enable_irq(ts->irq);
	}
	return IRQ_HANDLED;
}

static void st1615_ts_power(struct st1615_ts_data *ts, bool poweron)
{
	if (gpio_is_valid(ts->pdata->reset_gpio))
		gpio_direction_output(ts->pdata->reset_gpio, poweron);
}

static void st1615_reset(struct st1615_ts_data *ts) {
	if(gpio_is_valid(ts->pdata->reset_gpio)) {
		gpio_direction_output(ts->pdata->reset_gpio,0);
		udelay(10000); //100ms
		gpio_direction_output(ts->pdata->reset_gpio,1);
		udelay(50000);
		gpio_direction_output(ts->pdata->reset_gpio,0);
	}
}

#ifdef CONFIG_FIIO_TP_BUILD_AS_TOUCH
static void st1615_close(struct input_dev *dev)
{
	struct st1615_ts_data *ts = input_get_drvdata(dev);
	fiio_debug("%s :Enter!!!\n",__func__);
	//disable_irq(ts->irq);
	fiio_debug("%s :Exit!!!\n",__func__);
}

static int st1615_open(struct input_dev *dev)
{
	struct st1615_ts_data *ts = input_get_drvdata(dev);
	fiio_debug("%s :Enter!!!\n",__func__);
	//st1615_ts_power(ts,true);
	//enable_irq(ts->irq);
	fiio_debug("%s :Exit!!!\n",__func__);
	return 0;
}
#endif

static int st1615_ts_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct st1615_ts_data *ts;
	struct st1615_platform_data *pdata = (struct st1615_platform_data*)client->dev.platform_data;
	struct input_dev *input_dev;
	int error;
	fiio_debug("%s :Enter !!!\n",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -EIO;
	}
	
	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev)
		return -ENOMEM;

	ts->client = client;
	ts->input_dev = input_dev;
	ts->pdata = pdata;
	client->irq = ts->irq;
	//irq
	if (gpio_is_valid(ts->pdata->int_gpio)) {
		error = gpio_request(pdata->int_gpio,"st1615 gpio irq");
		if (error < 0) {
			dev_err(&client->dev, "%s:failed to set gpio irq.\n",__func__);
			goto exit_request_int_irq_fail;
		}
		ts->irq = gpio_to_irq(pdata->int_gpio);
	}
	
	if (gpio_is_valid(ts->pdata->reset_gpio)) {
		error = devm_gpio_request(&client->dev, ts->pdata->reset_gpio, NULL);
		if (error) {
			dev_err(&client->dev,
				"Unable to request GPIO pin %d.\n",
				ts->pdata->reset_gpio);
				return error;
		}
	}

	st1615_ts_power(ts, true);

	input_dev->name = "st1615-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	
#ifdef CONFIG_FIIO_TP_BUILD_AS_TOUCH
	fiio_debug("%s version 1.0.0.0\n",__func__);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	

	input_set_abs_params(input_dev, ABS_X, 0, ts->pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->pdata->y_max, 0, 0);

	input_set_abs_params(input_dev, ABS_PRESSURE, 0, CFG_MAX_TOUCH_POINTS, 0 , 0);
#endif


	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, ts->pdata->x_min, ts->pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, ts->pdata->y_min, ts->pdata->y_max, 0, 0);

	
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	
#ifdef CONFIG_FIIO_TP_BUILD_AS_TOUCH
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;
#endif

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&client->dev, "Unable to register %s input device\n",
			input_dev->name);
		return error;
	}
	
#ifdef CONFIG_FIIO_TP_BUILD_AS_TOUCH
	input_dev->open = st1615_open;
	input_dev->close = st1615_close;
	input_set_drvdata(input_dev, ts);
#endif
	
	INIT_WORK(&ts->work, st1615_work_handler);
	ts->workqueue = create_singlethread_workqueue("st1615_wq");
	
	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, 
					  st1615_ts_irq_handler,
					  IRQF_ONESHOT,
					  client->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}

	error = request_irq(ts->irq, 
				st1615_ts_irq_handler,
				ts->pdata->irqflags, 
				client->dev.driver->name,
				ts);
	if (error < 0) {
		dev_err(&client->dev, "%s: request irq failed\n",__func__);
		goto exit_irq_request_failed;
	}
	
	i2c_set_clientdata(client, ts);

	//中断不唤醒系统
	device_init_wakeup(&client->dev, 0);

	st1615_ts_read_xy_res(ts);
	//st1615_tx_init_xy_res(ts);
	//st1615_ts_read_xy_res(ts);
	st1615_ts_read_fw_version(ts);
	
#ifdef CONFIG_ST1615_UPGRADE_FIRMWARE
	INIT_WORK(&ts->upgrade_work, st1615_upgrade_work_handler);
	ts->upgrade_workqueue = create_singlethread_workqueue("st1615_upgrade");
	queue_work(ts->upgrade_workqueue, &ts->upgrade_work);
#endif
	
	fiio_debug("%s :Exit !!!\n",__func__);
	return 0;
	exit_irq_request_failed:
		input_free_device(input_dev);
	exit_request_int_irq_fail:
		i2c_set_clientdata(client, NULL);
		kfree(ts);
		return -1;
}

static int st1615_ts_remove(struct i2c_client *client)
{
	struct st1615_ts_data *ts = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);
	st1615_ts_power(ts, false);
	
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int st1615_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1615_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(client->irq);
	} else {
		disable_irq(client->irq);
		st1615_ts_power(ts, false);
	}
	return 0;
}

static int st1615_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1615_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(client->irq);
	} else {
		st1615_ts_power(ts, true);
		enable_irq(client->irq);
	}

	return 0;
}

#endif

static SIMPLE_DEV_PM_OPS(st1615_ts_pm_ops,
			 st1615_ts_suspend, st1615_ts_resume);

static const struct i2c_device_id st1615_ts_id[] = {
	{ ST1615_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, st1615_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id st1615_ts_dt_ids[] = {
	{ .compatible = "sitronix,st1615", },
	{ }
};
MODULE_DEVICE_TABLE(of, st1615_ts_dt_ids);
#endif

static struct i2c_driver st1615_ts_driver = {
	.probe		= st1615_ts_probe,
	.remove		= st1615_ts_remove,
	.id_table	= st1615_ts_id,
	.driver = {
		.name	= ST1615_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(st1615_ts_dt_ids),
		.pm	= &st1615_ts_pm_ops,
	},
};

module_i2c_driver(st1615_ts_driver);

MODULE_AUTHOR("pengweizhong@fiio.net");
MODULE_DESCRIPTION("SITRONIX ST1615 Touchscreen Controller Driver");
MODULE_LICENSE("GPL");

