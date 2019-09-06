#include <linux/i2c.h>
int fiio_i2c_read_byte(struct i2c_client *client, u8 reg);
int fiio_i2c_write_byte(struct i2c_client *client, u8 reg, u8 val);
int fiio_i2c_write_mask(struct i2c_client *client, u8 reg, u8 val,u8 mask, u8 shift);
int fiio_i2c_read_mask(struct i2c_client *client, u8 reg,u8 mask, u8 shift);
int fiio_i2c_read_bit(struct i2c_client *client, u8 reg, u8 bit);
int fiio_i2c_write_bit(struct i2c_client *client, u8 reg,bool val, u8 bit);
int fiio_i2c_read_bytes(struct i2c_client *client,u8 reg_addr,int len, u8 *data);
int fiio_i2c_write_bytes(struct i2c_client *client,u8 reg_addr,int len, u8 *data);





