#include <linux/version.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

int fiio_i2c_read_byte(struct i2c_client *client, u8 reg)
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

int fiio_i2c_write_byte(struct i2c_client *client, u8 reg, u8 val)
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
int fiio_i2c_write_mask(struct i2c_client *client, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = fiio_i2c_read_byte(client, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return fiio_i2c_write_byte(client, reg, ret);
}

/* read value from register, apply mask and right shift it */
int fiio_i2c_read_mask(struct i2c_client *client, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = fiio_i2c_read_byte(client, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}


/* read value from register and return one specified bit */
int fiio_i2c_read_bit(struct i2c_client *client, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return fiio_i2c_read_mask(client, reg, BIT(bit), bit);
}

/* change only one bit in register */
int fiio_i2c_write_bit(struct i2c_client *client, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return fiio_i2c_write_mask(client, reg, val, BIT(bit), bit);
}

int fiio_i2c_read_bytes(struct i2c_client *client,u8 reg_addr,int len, u8 *data)
{
	int err = 0;
	struct i2c_msg msg[2];
	int tries = 0;
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

int fiio_i2c_write_bytes(struct i2c_client *client,u8 reg_addr,int len, u8 *data)
{
	int err = 0;
	u8 send[len + 1];
	struct i2c_msg msg;
	int tries = 0;

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

