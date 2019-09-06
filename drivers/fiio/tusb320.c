/********************************************************************************************/
/*FiiO M5                                                                                   */
/*TUSB320 Driver                                                                            */
/*2018-09-20                                                                                */
/********************************************************************************************/

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
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>

#define FIIO_CC_TEST

#define FIIO_DEBUG_TUSB320
#ifdef FIIO_DEBUG_TUSB320
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_tusb320] " x)
#else
#define fiio_debug(x...)
#endif

#define	TUSB320_DEV_NAME		"tusb320"

#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
                                      ((_x) & 0x04 ? 2 : 3)) :\
                       ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
                                      ((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
        ((_x) ? __CONST_FFS(_x) : 0)

#undef  BITS
#define BITS(_end, _start) \
        ((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_byte, _mask, _shift) \
        (((_byte) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_byte, _bit) \
        __BITS_GET(_byte, _bit, FFS(_bit))

#undef  __BITS_SET
#define __BITS_SET(_byte, _mask, _shift, _val) \
        (((_byte) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef  BITS_SET
#define BITS_SET(_byte, _bit, _val) \
        __BITS_SET(_byte, _bit, FFS(_bit), _val)

#undef  BITS_MATCH
#define BITS_MATCH(_byte, _bit) \
        (((_byte) & (_bit)) == (_bit))

#define TUBS320_CSR_REG_ID(nr)		(0x00 + (nr))
#define TUBS320_CSR_REG_ID_MAX		8

#define TUBS320_CSR_REG_08		0x08

#define TUBS320_DFP_POWER_MODE		BITS(7,6)
#define TUBS320_DFP_POWER_DEFAULT	0
#define TUBS320_DFP_POWER_MEDIUM	1
#define TUBS320_DFP_POWER_HIGH		2

#define TUBS320_UFP_POWER_MODE		BITS(5,4)
#define TUBS320_UFP_POWER_DEFAULT	0	/* 500/900mA */
#define TUBS320_UFP_POWER_MEDIUM	1	/* 1.5A */
#define TUBS320_UFP_POWER_ACC		2	/* Charge through accessory (500mA) */
#define TUBS320_UFP_POWER_HIGH		3	/* 3A */

#define TUBS320_ACC_CONN		BITS(3,1)
#define TUBS320_ACC_NOT_ATTACH		0
#define TUBS320_ACC_AUDIO_DFP		4
#define TUBS320_ACC_DEBUG_DFP		6

#define TUBS320_CSR_REG_09		0x09

#define TUBS320_ATTACH_STATE		BITS(7,6)
#define TUBS320_NOT_ATTACH		0
#define TUBS320_ATTACH_AS_UFP		1
#define TUBS320_ATTACH_AS_DFP		2
#define TUBS320_ATTACH_ACC		3

#define TUBS320_CABLE_DIR		BIT (5)

#define TUBS320_INT_STATE		BIT (4)
#define TUBS320_INT_INTR		0
#define TUBS320_INT_CLEAR		1

#define TUBS320_CSR_REG_0A		0x0A

#define TUBS320_MODE_SELECT		BITS(5,4)
#define TUBS320_MODE_DEFAULT		0
#define TUBS320_MODE_UFP		1
#define TUBS320_MODE_DFP		2
#define TUBS320_MODE_DRP		3

#define TUBS320_SOFT_RESET		BIT(3)
#define TUBS320_RESET_VALUE		1

#define TUBS320_ENB_ENABLE		0
#define TUBS320_ENB_DISABLE		1
#define TUBS320_ENB_INTERVAL		100

#define TUBS320_I2C_RESET		1
#define TUBS320_GPIO_I2C_RESET		2

struct tusb320_data {
    unsigned int_gpio;
};

struct tusb320_chip {
    struct i2c_client *client;
    struct tusb320_data *pdata;
    int irq_gpio;
    u8 mode;
    u8 state;
    u8 cable_direction;
    u8 ufp_power;
    u8 accessory_mode;
    struct wake_lock wlock;

	//test
	#ifdef FIIO_CC_TEST
	struct workqueue_struct *tusb320_wq;
	struct delayed_work otg_cc_work;
	#endif
};


static struct tusb320_chip *gchip  = NULL;

static int tusb320_write_masked_byte(struct i2c_client *client,
                                     u8 addr, u8 mask, u8 val) {
    int rc;

    if (!mask) {
        /* no actual access */
        rc = -EINVAL;
        goto out;
    }

    rc = i2c_smbus_read_byte_data(client, addr);
    if (!IS_ERR_VALUE(rc)) {
        rc = i2c_smbus_write_byte_data(client,
                                       addr, BITS_SET((u8)rc, mask, val));
    }

out:
    return rc;
}

static int tusb320_read_device_id(struct tusb320_chip *chip) {
    struct device *cdev = &chip->client->dev;
    u8 buffer[TUBS320_CSR_REG_ID_MAX + 1];
    int rc = 0, i = 0;
    for (i = TUBS320_CSR_REG_ID_MAX-1; i >= 0; i -= 1) {
        rc = i2c_smbus_read_byte_data(chip->client,TUBS320_CSR_REG_ID(i));
        if (IS_ERR_VALUE(rc)) {
            dev_err(cdev, "%s: failed to read REG_ID\n",__func__);
            return rc;
        }
        buffer[i] = rc & 0xFF;
        dev_info(cdev, "device id: buffer[%d]=%X", i,buffer[i]);
    }

    buffer[TUBS320_CSR_REG_ID_MAX+1] = '\0';
    dev_info(cdev, "device id: %s\n", buffer);
    return 0;
}

static int tusb320_select_mode(struct tusb320_chip *chip, u8 sel_mode) {
    struct device *cdev = &chip->client->dev;
    int rc = 0;

    if (sel_mode > TUBS320_MODE_DRP) {
        dev_err(cdev, "%s: sel_mode(%d) is unavailable\n",
                __func__, sel_mode);
        return -EINVAL;
    }

    /* mode selection is possibel only in standby/unattached mode */
    if (chip->state != TUBS320_NOT_ATTACH) {
        dev_err(cdev, "%s: unavailable in attached state (%d)\n",
                __func__, chip->state);
        return -EPERM;
    }

    if (chip->mode != sel_mode) {
        rc = tusb320_write_masked_byte(chip->client,
                                       TUBS320_CSR_REG_0A,
                                       TUBS320_MODE_SELECT,
                                       sel_mode);
        if (IS_ERR_VALUE(rc)) {
            dev_err(cdev, "failed to write MODE_SELECT\n");
            return rc;
        }

        chip->mode = sel_mode;
    }

    dev_info(cdev, "%s: mode (%d)\n", __func__, chip->mode);

    return rc;
}

static int tusb320_i2c_reset_device(struct tusb320_chip *chip) {
    struct device *cdev = &chip->client->dev;
    int rc = 0;

    rc = tusb320_write_masked_byte(chip->client,
                                   TUBS320_CSR_REG_0A,
                                   TUBS320_SOFT_RESET,
                                   TUBS320_RESET_VALUE);
    if(IS_ERR_VALUE(rc)) {
        dev_err(cdev, "%s: failed to write REG_0A\n",
                __func__);
        return rc;
    }

    chip->mode = TUBS320_MODE_DEFAULT;
    chip->state = TUBS320_NOT_ATTACH;
    chip->cable_direction = 0;
    chip->ufp_power = TUBS320_UFP_POWER_DEFAULT;
    chip->accessory_mode = TUBS320_ACC_NOT_ATTACH;

    //dev_info(cdev, "%s is done\n", __func__);

    return 0;
}

static int tusb320_gpio_reset_device(struct tusb320_chip *chip) {
    struct device *cdev = &chip->client->dev;

    //gpio_set_value(chip->pdata->enb_gpio, TUBS320_ENB_DISABLE);
    msleep(TUBS320_ENB_INTERVAL);
    //gpio_set_value(chip->pdata->enb_gpio, TUBS320_ENB_ENABLE);
    msleep(TUBS320_ENB_INTERVAL);

    chip->mode = TUBS320_MODE_DEFAULT;
    chip->state = TUBS320_NOT_ATTACH;
    chip->cable_direction = 0;
    chip->ufp_power = TUBS320_UFP_POWER_DEFAULT;
    chip->accessory_mode = TUBS320_ACC_NOT_ATTACH;

    dev_dbg(cdev, "%s is done\n", __func__);

    return 0;
}

static int tusb320_reset_device(struct tusb320_chip *chip, int mode) {
    struct device *cdev = &chip->client->dev;
    int rc = 0;

    //dev_info(cdev, "%s: mode(%d)\n", __func__, mode);

    switch (mode) {
    case TUBS320_I2C_RESET:
        rc = tusb320_i2c_reset_device(chip);
        if (!IS_ERR_VALUE(rc))
            break;
    case TUBS320_GPIO_I2C_RESET:
        tusb320_gpio_reset_device(chip);
        rc = tusb320_i2c_reset_device(chip);
        if (IS_ERR_VALUE(rc))
            dev_err(cdev, "%s: TUBS320_GPIO_I2C_RESET fails\n",
                    __func__);
        break;
    default:
        rc = -EINVAL;
        dev_err(cdev, "%s: Invaild mode\n", __func__);
        break;
    }

    return rc;
}

#define TUSB320_DEV_ATTR(field, format_string)				\
	static ssize_t							\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	struct i2c_client *client = to_i2c_client(dev);			\
	struct tusb320_chip *chip = i2c_get_clientdata(client);		\
	\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, chip->field);			\
}									\
static DEVICE_ATTR(field, S_IRUGO, field ## _show, NULL);

TUSB320_DEV_ATTR(state, "%u\n")
TUSB320_DEV_ATTR(accessory_mode, "%u\n")
TUSB320_DEV_ATTR(cable_direction, "%u\n")

ssize_t tusb320_show_mode(struct device *dev,
                          struct device_attribute *attr,
                          char *buf) {
    struct i2c_client *client = to_i2c_client(dev);
    struct tusb320_chip *chip = i2c_get_clientdata(client);

    return snprintf(buf, PAGE_SIZE, "%u\n", chip->mode);
}

static ssize_t tusb320_store_mode(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buff, size_t size) {
    struct i2c_client *client = to_i2c_client(dev);
    struct tusb320_chip *chip = i2c_get_clientdata(client);
    unsigned mode = 0;
    int rc = 0;

    if (sscanf(buff, "%d", &mode) == 1) {
        rc = tusb320_select_mode(chip, (u8)mode);
        if (IS_ERR_VALUE(rc))
            return rc;

        return size;
    }

    return -EINVAL;
}
DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, tusb320_show_mode, tusb320_store_mode);

static ssize_t tusb320_store_reset(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buff, size_t size) {
    struct i2c_client *client = to_i2c_client(dev);
    struct tusb320_chip *chip = i2c_get_clientdata(client);
    unsigned state = 0;
    int rc = 0;

    if (sscanf(buff, "%u", &state) == 1) {
        rc = tusb320_reset_device(chip, state);
        if (IS_ERR_VALUE(rc))
            return rc;

        return size;
    }

    return -EINVAL;
}
DEVICE_ATTR(reset, S_IWUSR, NULL, tusb320_store_reset);

ssize_t tusb320_show_chipid(struct device *dev,
                          struct device_attribute *attr,
                          char *buf) {
    struct i2c_client *client = to_i2c_client(dev);
    struct tusb320_chip *chip = i2c_get_clientdata(client);
   	int ret = tusb320_read_device_id(chip);
    if (IS_ERR_VALUE(ret)) {
        dev_err(dev, "failed to read device id\n");
    }

    return 0;
}


DEVICE_ATTR(chipid, S_IRUGO , tusb320_show_chipid,NULL );

static int tusb320_create_devices(struct device *cdev) {
    int ret = 0;

    ret = device_create_file(cdev, &dev_attr_state);
    if (ret < 0) {
        dev_err(cdev, "failed to create dev_attr_state\n");
        ret = -ENODEV;
        goto err1;
    }

    ret = device_create_file(cdev, &dev_attr_accessory_mode);
    if (ret < 0) {
        dev_err(cdev, "failed to create dev_attr_accessory_mode\n");
        ret = -ENODEV;
        goto err2;
    }

    ret = device_create_file(cdev, &dev_attr_cable_direction);
    if (ret < 0) {
        dev_err(cdev, "failed to create dev_attr_cable_direction\n");
        ret = -ENODEV;
        goto err3;
    }

    ret = device_create_file(cdev, &dev_attr_mode);
    if (ret < 0) {
        dev_err(cdev, "failed to create dev_attr_mode\n");
        ret = -ENODEV;
        goto err4;
    }

    ret = device_create_file(cdev, &dev_attr_reset);
    if (ret < 0) {
        dev_err(cdev, "failed to create dev_attr_reset\n");
        ret = -ENODEV;
        goto err5;
    }
	 ret = device_create_file(cdev, &dev_attr_chipid);
    if (ret < 0) {
        dev_err(cdev, "failed to create dev_attr_chipid\n");
        ret = -ENODEV;
        goto err6;
    }
    return ret;
err6:
    device_remove_file(cdev, &dev_attr_reset);
err5:
    device_remove_file(cdev, &dev_attr_mode);
err4:
    device_remove_file(cdev, &dev_attr_cable_direction);
err3:
    device_remove_file(cdev, &dev_attr_accessory_mode);
err2:
    device_remove_file(cdev, &dev_attr_state);
err1:
    return ret;
}

static void tusb320_destory_device(struct device *cdev) {
    device_remove_file(cdev, &dev_attr_state);
    device_remove_file(cdev, &dev_attr_accessory_mode);
    device_remove_file(cdev, &dev_attr_cable_direction);
    device_remove_file(cdev, &dev_attr_mode);
    device_create_file(cdev, &dev_attr_reset);
	device_create_file(cdev, &dev_attr_chipid);
}

static void tusb320_not_attach(struct tusb320_chip *chip) {
    struct device *cdev = &chip->client->dev;

    //dev_info(cdev, "%s: state (%d)\n", __func__, chip->state);

    switch (chip->state) {
    case TUBS320_ATTACH_AS_DFP:
        break;
    case TUBS320_ATTACH_AS_UFP:
        //power_supply_set_usb_otg(chip->usb_psy, false);
        break;
    case TUBS320_ATTACH_ACC:
        break;
    case TUBS320_NOT_ATTACH:
        break;
    default:
        dev_err(cdev, "%s: Invaild state\n", __func__);
        break;
    }
    tusb320_reset_device(chip, TUBS320_I2C_RESET);

    chip->state = TUBS320_NOT_ATTACH;
    chip->cable_direction = 0;
    chip->ufp_power = TUBS320_UFP_POWER_DEFAULT;
    chip->accessory_mode = TUBS320_ACC_NOT_ATTACH;
}

static void tusb320_attach_as_ufp(struct tusb320_chip *chip, u8 detail) {
    struct device *cdev = &chip->client->dev;

    dev_info(cdev, "%s: state (%d)\n", __func__, chip->state);

    if (chip->mode == TUBS320_MODE_UFP) {
        dev_err(cdev, "%s: mode is UFP\n", __func__);
        return;
    }

    switch (chip->state) {
    case TUBS320_NOT_ATTACH:
        chip->state = TUBS320_ATTACH_AS_UFP;
        //power_supply_set_usb_otg(chip->usb_psy, true);
        break;
    case TUBS320_ATTACH_AS_UFP:
        break;
    case TUBS320_ATTACH_AS_DFP:
    case TUBS320_ATTACH_ACC:
    default:
        tusb320_reset_device(chip, TUBS320_I2C_RESET);
        dev_err(cdev, "%s: Invaild state\n", __func__);
        break;
    }
}

static void tusb320_attach_as_dfp(struct tusb320_chip *chip, u8 detail) {
    struct device *cdev = &chip->client->dev;
    u8 ufp_pow = BITS_GET(detail, TUBS320_UFP_POWER_MODE);
    int limit = 0;

   // dev_info(cdev, "%s: state (%d)\n", __func__, chip->state);

    if (chip->mode == TUBS320_MODE_DFP) {
        dev_err(cdev, "%s: mode is DFP\n", __func__);
        return;
    }

    dev_dbg(cdev, "%s: ufp_power [before(%d) vs after(%d)]\n",
            __func__, chip->ufp_power, ufp_pow);

    limit = (ufp_pow == TUBS320_UFP_POWER_HIGH ? 3000 :
             (ufp_pow == TUBS320_UFP_POWER_MEDIUM ? 1500 : 0));

    switch (chip->state) {
    case TUBS320_NOT_ATTACH:
        chip->state = TUBS320_ATTACH_AS_DFP;
        chip->ufp_power = ufp_pow;
        //tusb320_set_current_max(chip->usb_psy, limit);
        break;
    case TUBS320_ATTACH_AS_DFP:
        if (chip->ufp_power != ufp_pow) {
            //tusb320_set_current_max(chip->usb_psy, limit);
        }
        break;
    case TUBS320_ATTACH_AS_UFP:
    case TUBS320_ATTACH_ACC:
    default:
        tusb320_reset_device(chip, TUBS320_I2C_RESET);
        dev_err(cdev, "%s: Invaild state\n", __func__);
        break;
    }
}

static void tusb320_attach_accessory_detail(struct tusb320_chip *chip,
        u8 acc_conn, u8 detail) {
    struct device *cdev = &chip->client->dev;

    if (chip->accessory_mode == TUBS320_ACC_NOT_ATTACH) {
        chip->accessory_mode = acc_conn;

        switch (acc_conn) {
        case TUBS320_ACC_AUDIO_DFP:
            break;
        case TUBS320_ACC_DEBUG_DFP:
            break;
        default:
            tusb320_reset_device(chip, TUBS320_I2C_RESET);
            dev_err(cdev, "%s: Invaild state\n", __func__);
            break;
        }
    }
}

static void tusb320_attach_accessory(struct tusb320_chip *chip, u8 detail) {
    struct device *cdev = &chip->client->dev;
    u8 acc_conn = BITS_GET(detail, TUBS320_ACC_CONN);

    dev_info(cdev, "%s: state (%d)\n", __func__, chip->state);
    dev_info(cdev, "%s: accessory_mode [before(%d) vs after(%d)]\n",
             __func__, chip->accessory_mode, acc_conn);

    switch (chip->state) {
    case TUBS320_NOT_ATTACH:
        chip->state = TUBS320_ATTACH_ACC;
        tusb320_attach_accessory_detail(chip, acc_conn, detail);
        break;
    case TUBS320_ATTACH_ACC:
        tusb320_attach_accessory_detail(chip, acc_conn, detail);
        break;
    case TUBS320_ATTACH_AS_UFP:
    case TUBS320_ATTACH_AS_DFP:
    default:
        tusb320_reset_device(chip, TUBS320_I2C_RESET);
        dev_err(cdev, "%s: Invaild state\n", __func__);
        break;
    }
}

int tusb320_work_handler(void) {
    struct tusb320_chip *chip = gchip;
    //container_of(work, struct tusb320_chip, dwork);
    struct device *cdev = &chip->client->dev;
    int ret;
    u8 reg09, reg08, state;

    wake_lock(&chip->wlock);

    /* Get status (reg8/reg9) */
    ret = i2c_smbus_read_byte_data(chip->client, TUBS320_CSR_REG_08);
    if (IS_ERR_VALUE(ret)) {
        dev_err(cdev, "%s: failed to read REG_08\n", __func__);
        goto work_unlock;
    }
    reg08 = ret & 0xFF;

    ret = i2c_smbus_read_byte_data(chip->client, TUBS320_CSR_REG_09);
    if (IS_ERR_VALUE(ret)) {
        dev_err(cdev, "%s: failed to read REG_09\n", __func__);
        goto work_unlock;
    }
    reg09 = ret & 0xFF;
	/*printk("reg08:%02x \n",i2c_smbus_read_byte_data(chip->client, TUBS320_CSR_REG_08));
	printk("reg09:%02x \n",i2c_smbus_read_byte_data(chip->client, TUBS320_CSR_REG_09));
	printk("reg0A:%02x \n",i2c_smbus_read_byte_data(chip->client, TUBS320_CSR_REG_0A));*/
    /* Clear Interrupt */
    ret = tusb320_write_masked_byte(chip->client,
                                    TUBS320_CSR_REG_09,
                                    TUBS320_INT_STATE,
                                    TUBS320_INT_CLEAR);
    if (IS_ERR_VALUE(ret)) {
        dev_err(cdev, "%s: failed to write REG_09\n", __func__);
        goto work_unlock;
    }

    state = BITS_GET(reg09, TUBS320_ATTACH_STATE);
    chip->cable_direction = BITS_GET(reg09, TUBS320_CABLE_DIR);

   // dev_info(cdev, "%s: [state %d],[direction:%d]\n",__func__, state, chip->cable_direction);

    switch (state) {
    case TUBS320_NOT_ATTACH:
        tusb320_not_attach(chip);
        break;
    case TUBS320_ATTACH_AS_UFP:
        tusb320_attach_as_ufp(chip, reg08);
        break;
    case TUBS320_ATTACH_AS_DFP:
        tusb320_attach_as_dfp(chip, reg08);
        break;
    case TUBS320_ATTACH_ACC:
        tusb320_attach_accessory(chip, reg08);
        break;
    default:
        tusb320_reset_device(chip, TUBS320_I2C_RESET);
        dev_err(cdev, "%s: Invalid state\n", __func__);
        break;
    }
work_unlock:
    wake_unlock(&chip->wlock);
    return state;
}
EXPORT_SYMBOL_GPL(tusb320_work_handler);


static irqreturn_t tusb320_interrupt(int irq, void *data) {
    struct tusb320_chip *chip = (struct tusb320_chip *)data;
    if (!chip) {
        pr_err("%s : called before init.\n", __func__);
        return IRQ_HANDLED;
    }

    dev_dbg(&chip->client->dev, "%s\n", __func__);

    //schedule_work(&chip->dwork);
	queue_delayed_work(chip->tusb320_wq,  &chip->otg_cc_work, msecs_to_jiffies(5));

    return IRQ_HANDLED;
}

int tusb320_init_gpio(struct tusb320_chip *chip) {
    struct device *cdev = &chip->client->dev;
    int rc = 0;

    if (gpio_is_valid(chip->pdata->int_gpio)) {
        rc = gpio_request_one(chip->pdata->int_gpio,GPIOF_DIR_IN, "tusb320_int_gpio");
        if (rc) {
            dev_err(cdev, "unable to request int_gpio %d\n",chip->pdata->int_gpio);
            goto err;
        }
    } else {
        dev_err(cdev, "int_gpio %d is not valid\n",chip->pdata->int_gpio);
        rc = -EINVAL;
        goto err;
    }
    return rc;

err:
    return rc;
}

static void tusb320_free_gpio(struct tusb320_chip *chip) {
    if (gpio_is_valid(chip->pdata->int_gpio))
        gpio_free(chip->pdata->int_gpio);
}

#ifdef FIIO_CC_TEST
extern void fiio_set_usb_host_or_device(int flag);
extern void fiio_update_usb_conn_or_disconn_flag(int flag);
#ifdef CONFIG_USB_DWC2
extern void fiio_update_usb_status(int type);
extern void fiio_diconnect_usb_and_reset_usb_controller(void);
#endif


extern int fiio_get_host_eys_test_mode(void);
static void tusb320_usb_cc_work(struct work_struct *work)
{
	int data = 0,otg_state = 0;
	struct tusb320_chip *chip =	container_of(work, struct tusb320_chip, otg_cc_work.work);
	//iio_read_channel_raw(bq->chan, &data);

    otg_state = tusb320_work_handler();
	printk(">>>>>>>>>>>>>>>>tusb320_usb_cc_work otg_state:%d enable otg\n",otg_state);
    if (0 == fiio_get_host_eys_test_mode()) {
        switch(otg_state) {
        case 0:
            //disable otg from last connect status
            #ifdef CONFIG_USB_DWC2
            fiio_diconnect_usb_and_reset_usb_controller();
            #endif
            fiio_update_usb_conn_or_disconn_flag(0);
            fiio_set_usb_host_or_device(1);
            //fiio_usb_power_on(0);
            break;
        case 1:
            //enable otg
            //fiio_usb_power_on(1);
            fiio_update_usb_conn_or_disconn_flag(1);
            fiio_set_usb_host_or_device(0);
    #ifdef CONFIG_USB_DWC2
            fiio_update_usb_status(0);
    #endif
            break;
        case 2:
            fiio_update_usb_conn_or_disconn_flag(1);
            fiio_set_usb_host_or_device(1);
    #ifdef CONFIG_USB_DWC2
            fiio_update_usb_status(2);
    #endif
            break;
        case 3:
            //disable otg from last connect status
            #ifdef CONFIG_USB_DWC2
            fiio_diconnect_usb_and_reset_usb_controller();
            #endif
            fiio_update_usb_conn_or_disconn_flag(0);
            fiio_set_usb_host_or_device(1);
            break;
        default:
            //disable otg from last connect status
            #ifdef CONFIG_USB_DWC2
            fiio_diconnect_usb_and_reset_usb_controller();
            #endif
            fiio_update_usb_conn_or_disconn_flag(0);
            fiio_set_usb_host_or_device(1);
            break;
        }
    }
    else {
        printk(">>>>>>>>>>>>>>>>tusb320_usb_cc_work fiio test host mode\n");
        switch(otg_state) {
        case 1:
            //enable otg
            //fiio_usb_power_on(1);
            fiio_update_usb_conn_or_disconn_flag(1);
            fiio_set_usb_host_or_device(0);
    #ifdef CONFIG_USB_DWC2
            fiio_update_usb_status(0);
    #endif
            break;
        }
    }

	
	//if(data > USB_TYPE_MIN_VALUE && data < USB_TYPE_MAX_VALUE){
	if(otg_state==1 || otg_state==3 ){
		//otg insert
		//if(bq25890_field_read(bq, F_OTG_CFG) != 1){
			//printk("bq25890_usb_cc_work data:%d enable otg\n",data);
			//bq25890_charger_dat_switch(bq, 1);
			//bq25890_field_write(bq, F_OTG_CFG, 1);
			//muic_notifier_attach_attached_dev(ATTACHED_DEV_OTG_MUIC);
		//}
	}else{
		//if(bq25890_field_read(bq, F_OTG_CFG) != 0){
			//printk("bq25890_usb_cc_work data:%d disable otg\n",data);
			//bq25890_field_write(bq, F_OTG_CFG, 0);
			//bq25890_charger_dat_switch(bq, 0);
			//muic_notifier_detach_attached_dev(ATTACHED_DEV_OTG_MUIC);
		//}
	}

	//queue_delayed_work(chip->charger_wq,  &chip->otg_cc_work, msecs_to_jiffies(1000));
}
#endif

static int tusb320_probe(struct i2c_client *client,
                         const struct i2c_device_id *id) {
    struct tusb320_chip *chip;
    struct device *cdev = &client->dev;
    int ret = 0, is_active;
	int err = 0;
	fiio_debug("%s:Enter!!!\n",__func__);
    if (!i2c_check_functionality(client->adapter,
                                 I2C_FUNC_SMBUS_BYTE_DATA |
                                 I2C_FUNC_SMBUS_WORD_DATA)) {
        dev_err(cdev, "smbus data not supported!\n");
        return -EIO;
    }

    chip = devm_kzalloc(cdev, sizeof(struct tusb320_chip), GFP_KERNEL);
    if (!chip) {
        dev_err(cdev, "can't alloc tusb320_chip\n");
        return -ENOMEM;
    }
    gchip = chip;
    chip->client = client;
    i2c_set_clientdata(client, chip);

    if (&client->dev.of_node) {
        struct tusb320_data *data = devm_kzalloc(cdev,
                                    sizeof(struct tusb320_data), GFP_KERNEL);

        if (!data) {
            dev_err(cdev, "can't alloc tusb320_data\n");
            ret = -ENOMEM;
            goto err1;
        }

        chip->pdata = data;
		chip->pdata->int_gpio = GPIO_CC_INT;

    } else {
        chip->pdata = client->dev.platform_data;
    }

    ret = tusb320_init_gpio(chip);
    if (ret) {
        dev_err(cdev, "failed to init gpio\n");
        goto err2;
    }

    chip->mode = TUBS320_MODE_DEFAULT;
    chip->state = TUBS320_NOT_ATTACH;
    chip->cable_direction = 0;
    chip->ufp_power = TUBS320_UFP_POWER_DEFAULT;
    chip->accessory_mode = TUBS320_ACC_NOT_ATTACH;

    //INIT_WORK(&chip->dwork, tusb320_work_handler);
    wake_lock_init(&chip->wlock, WAKE_LOCK_SUSPEND, "tusb320_wake");

    ret = tusb320_create_devices(cdev);
    if (IS_ERR_VALUE(ret)) {
        dev_err(cdev, "could not create devices\n");
        goto err3;
    }

    chip->irq_gpio = gpio_to_irq(chip->pdata->int_gpio);
    if (chip->irq_gpio < 0) {
        dev_err(cdev, "could not register int_gpio\n");
        ret = -ENXIO;
        goto err4;
    }
	#if 1
    ret = tusb320_read_device_id(chip);
    if (IS_ERR_VALUE(ret)) {
        dev_err(cdev, "failed to read device id\n");
    }
    tusb320_reset_device(chip, TUBS320_I2C_RESET);
    ret = tusb320_read_device_id(chip);
    if (IS_ERR_VALUE(ret)) {
        dev_err(cdev, "failed to read device id\n");
    }
	#endif
#if 1
    if (is_active) {
        dev_info(cdev, "presents interrupt initially\n");
        //schedule_work(&chip->dwork);
    } else {
        ret = tusb320_select_mode(chip, 3);
        if (IS_ERR_VALUE(ret))
            dev_err(cdev, "failed to select mode and work as default\n");
    }
#endif

	//add test
	#ifdef FIIO_CC_TEST
	chip->tusb320_wq = create_singlethread_workqueue("tusb320_wq");
	if (chip->tusb320_wq == NULL) {
		dev_err(cdev, "failed to create work queue\n");
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&chip->otg_cc_work,  tusb320_usb_cc_work);
	//开机1s后读取CC状态
	queue_delayed_work(chip->tusb320_wq,  &chip->otg_cc_work, msecs_to_jiffies(1000));


	err = request_irq(chip->irq_gpio, tusb320_interrupt,
                 IRQF_TRIGGER_FALLING, "tusb320_isr", chip);
    if (err < 0) {
            dev_err(cdev, "request irq failed: %d\n", err);
            goto err4;
    }
	
	device_init_wakeup(&client->dev, 1);
	fiio_debug("%s:Exit!!!\n",__func__);
	#endif
	
    return 0;

err4:
    tusb320_destory_device(cdev);
err3:
    wake_lock_destroy(&chip->wlock);
    tusb320_free_gpio(chip);
err2:
    if (&client->dev.of_node)
        devm_kfree(cdev, chip->pdata);
err1:
    i2c_set_clientdata(client, NULL);
    devm_kfree(cdev, chip);

    return ret;
}

static int tusb320_remove(struct i2c_client *client) {
    struct tusb320_chip *chip = i2c_get_clientdata(client);
    struct device *cdev = &client->dev;

    if (!chip) {
        pr_err("%s : chip is null\n", __func__);
        return -ENODEV;
    }

    if (chip->irq_gpio > 0)
        devm_free_irq(cdev, chip->irq_gpio, chip);

    tusb320_destory_device(cdev);
    wake_lock_destroy(&chip->wlock);
    tusb320_free_gpio(chip);

    if (&client->dev.of_node)
        devm_kfree(cdev, chip->pdata);

    i2c_set_clientdata(client, NULL);
    devm_kfree(cdev, chip);

    return 0;
}

#ifdef CONFIG_PM
extern void fiio_set_wakeup_source();
extern void fiio_clear_wakeup_source();

static int tusb320_suspend(struct device *dev) {

	struct i2c_client *client = to_i2c_client(dev);
	struct tusb320_chip *chip = i2c_get_clientdata(client);
	fiio_debug("%s:Enter!!!\n",__func__);
	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(chip->irq_gpio);
	} else {
		disable_irq(chip->irq_gpio);
	}

	//fiio_set_wakeup_source();
	fiio_debug("%s:Exit!!!\n",__func__);
    return 0;
}

static int tusb320_resume(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);
	struct tusb320_chip *chip = i2c_get_clientdata(client);
	fiio_debug("%s:Enter!!!\n",__func__);
	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(chip->irq_gpio);
	} else {
		enable_irq(chip->irq_gpio);
	}
	//fiio_clear_wakeup_source();
	//check it
	queue_delayed_work(chip->tusb320_wq,  &chip->otg_cc_work, msecs_to_jiffies(500));
	fiio_debug("%s:Exit!!!\n",__func__);
    return 0;
}

static const struct dev_pm_ops tusb320_dev_pm_ops = {
    .suspend = tusb320_suspend,
    .resume  = tusb320_resume,
};
#endif

static const struct i2c_device_id tusb320_id_table[] = {
    {"tusb320", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, tusb320_id_table);

#ifdef CONFIG_OF
static struct of_device_id tusb320_match_table[] = {
    { .compatible = "ti,tusb320",},
    { },
};
#else
#define tusb320_match_table NULL
#endif


static struct i2c_driver tusb320_i2c_driver = {
    .driver = {
        .name = "tusb320",
        .owner = THIS_MODULE,
        .of_match_table = tusb320_match_table,
#ifdef CONFIG_PM
        .pm = &tusb320_dev_pm_ops,
#endif
    },
    .probe = tusb320_probe,
    .remove = tusb320_remove,
    .id_table = tusb320_id_table,
};


static __init int tusb320_i2c_init(void) {
    return i2c_add_driver(&tusb320_i2c_driver);
}

static __exit void tusb320_i2c_exit(void) {
    i2c_del_driver(&tusb320_i2c_driver);
}
late_initcall(tusb320_i2c_init);
//module_init(tusb320_i2c_init);
module_exit(tusb320_i2c_exit);



MODULE_DESCRIPTION("TUSB320 driver");
MODULE_AUTHOR("pengweizhong <pengweizhong@fiio.net>");
MODULE_LICENSE("GPL");



