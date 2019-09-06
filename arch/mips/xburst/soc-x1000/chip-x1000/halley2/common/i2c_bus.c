#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/interrupt.h>
#include "board_base.h"
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/jzsnd.h>

#ifdef CONFIG_EEPROM_AT24
#include <asm-generic/sizes.h>
#include <linux/i2c/at24.h>
#include <mach/platform.h>
#endif

#define SZ_16K	0x00004000

//extern struct platform_device jz_i2c2_device;

#if defined(CONFIG_TOUCHSCREEN_FT6X06)
#include <linux/input/ft6x06_ts.h>
static struct ft6x06_platform_data ft6x06_tsc_pdata = {
	.x_max          = 0,
	.y_max          = 0,
	.va_x_max	= 240,
	.va_y_max	= 240,
	.irqflags = IRQF_TRIGGER_FALLING|IRQF_DISABLED,
	.irq = GPIO_TP_INT,
	.reset = GPIO_TP_RESET,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_ST1615
	#include <linux/input/st1615.h>
	static struct st1615_platform_data st1615_ts_pdata = {
		.x_min          = 0,
		.y_min          = 0,
		.x_max	= 240-1,
		.y_max	= 240-1,
		.irqflags = IRQF_TRIGGER_FALLING|IRQF_DISABLED,
		.int_gpio = GPIO_TP_INT,
		.reset_gpio = GPIO_TP_RESET,
	};
#endif
#ifdef CONFIG_EEPROM_AT24
static struct at24_platform_data at24c16 = {
	.byte_len = SZ_16K / 8,
	.page_size = 16,

};
#endif
#ifdef CONFIG_WM8594_CODEC_V12
static struct snd_codec_data wm8594_codec_pdata = {
	.codec_sys_clk = 1200000,

};
#endif

#if (defined(CONFIG_SOFT_I2C0_GPIO_V12_JZ) || defined(CONFIG_I2C0_V12_JZ))
struct i2c_board_info jz_i2c0_devs[] __initdata = {
#if (defined(CONFIG_SND_ASOC_FIIO_AK4376))
	{
        I2C_BOARD_INFO("ak4376a",0x20>>1),
    },
#endif
#ifdef CONFIG_SND_ASOC_FIIO_AK4377
	{
        I2C_BOARD_INFO("ak4377",0x20>>1),
    },
#endif
/*

#ifdef  CONFIG_I2C_SI4713
{
	I2C_BOARD_INFO("si4713",0x22>>1),
	},
#endif
#ifdef CONFIG_SENSORS_BMA2X2
	{
		I2C_BOARD_INFO("bma2x2", 0x18),
		.irq = GPIO_GSENSOR_INTR,
	},
#endif*/
/*
#if defined(CONFIG_TOUCHSCREEN_FT6X06)
	{
		I2C_BOARD_INFO(FT6X06_NAME, 0x38),
		.platform_data = &ft6x06_tsc_pdata,
	},
#endif*/
};
int jz_i2c0_devs_size = ARRAY_SIZE(jz_i2c0_devs);

struct i2c_board_info jz_v4l2_camera_devs[] __initdata = {
#ifdef  CONFIG_SOC_CAMERA_OV7725
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("ov772x_fornt", 0x21),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_OV5640
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("ov5640-front", 0x3c),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_GC0308
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("gc0308", 0x21),
	},
#endif
#ifdef CONFIG_SOC_CAMERA_GC2155
	[FRONT_CAMERA_INDEX] = {
		I2C_BOARD_INFO("gc2155", 0x3c),
	},
#endif
};
int jz_v4l2_devs_size = ARRAY_SIZE(jz_v4l2_camera_devs);
#endif

#if (defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ))
struct i2c_board_info jz_i2c2_devs[] __initdata = {
#ifdef  CONFIG_REGULATOR_AXP173
	{
		I2C_BOARD_INFO("axp173",0x68>>1),//34H
	},
#endif

//begin add by pengweizhong for sensors mma8653
#ifdef  CONFIG_FIIO_SENSORS_MMA865X
	{
		I2C_BOARD_INFO("mma8653",0x1D),
	},
#endif
#ifdef  CONFIG_FIIO_SENSORS_LIS3DH
	{
		I2C_BOARD_INFO("lis3dh_acc",0x18),
	},
#endif

#ifdef  CONFIG_FIIO_SENSORS_LIS3DSH
	{
		I2C_BOARD_INFO("lis3dsh_acc",0x1D),
	},
#endif

#ifdef  CONFIG_FIIO_SENSORS_STK8323
	{
		I2C_BOARD_INFO("stk8323",0x1F),
	},
#endif

#ifdef  CONFIG_FIIO_SENSORS_STK832X
	{
		I2C_BOARD_INFO("stk832x",0x1F),
	},
#endif


//I2C read and write addr,see from ADDR pin
#ifdef  CONFIG_FIIO_CC_TUSB320
    #ifdef CONFIG_FIIO_CC_SGM7220
	{
        I2C_BOARD_INFO("tusb320",0x47),
	},
    #else
    {
        I2C_BOARD_INFO("tusb320",0x60),
    },
    #endif
#endif

//end

#ifdef CONFIG_EEPROM_AT24
	{
		I2C_BOARD_INFO("at24",0x57),
		.platform_data  = &at24c16,
	},
#endif
#ifdef CONFIG_WM8594_CODEC_V12
	{
		I2C_BOARD_INFO("wm8594", 0x1a),
		.platform_data  = &wm8594_codec_pdata,
	},
#endif
};
#endif

#if (defined(CONFIG_SOFT_I2C1_GPIO_V12_JZ) || defined(CONFIG_I2C1_V12_JZ))
struct i2c_board_info jz_i2c1_devs[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_FT6X06
        {
                I2C_BOARD_INFO(FT6X06_NAME, 0x38),
                .platform_data = &ft6x06_tsc_pdata,
        },
#endif

#ifdef CONFIG_TOUCHSCREEN_ST1615
		{
				I2C_BOARD_INFO(ST1615_NAME, 0x55),
				.platform_data = &st1615_ts_pdata,
		},
#endif

#ifdef CONFIG_EEPROM_AT24
	{
		I2C_BOARD_INFO("at24",0x57),
		.platform_data  = &at24c16,
	},
#endif
#ifdef CONFIG_WM8594_CODEC_V12
	{
		I2C_BOARD_INFO("wm8594", 0x1a),
		.platform_data  = &wm8594_codec_pdata,
	},
#endif
};
#endif


#if     defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ)
int jz_i2c2_devs_size = ARRAY_SIZE(jz_i2c2_devs);
#endif

#if     defined(CONFIG_SOFT_I2C1_GPIO_V12_JZ) || defined(CONFIG_I2C1_V12_JZ)
int jz_i2c1_devs_size = ARRAY_SIZE(jz_i2c1_devs);
#endif


#ifdef CONFIG_EEPROM_AT24

struct i2c_client *at24_client;

static int  at24_dev_init(void)
{
	struct i2c_adapter *i2c_adap;


#if defined(CONFIG_SOFT_I2C2_GPIO_V12_JZ) || defined(CONFIG_I2C2_V12_JZ)
	i2c_adap = i2c_get_adapter(2);
	at24_client = i2c_new_device(i2c_adap, jz_i2c2_devs);
#endif
#if defined(CONFIG_SOFT_I2C1_GPIO_V12_JZ) || defined(CONFIG_I2C1_V12_JZ)
	i2c_adap = i2c_get_adapter(1);
	at24_client = i2c_new_device(i2c_adap, jz_i2c1_devs);
#endif

#if defined(CONFIG_SOFT_I2C0_GPIO_V12_JZ) || defined(CONFIG_I2C0_V12_JZ)
	i2c_adap = i2c_get_adapter(0);
	at24_client = i2c_new_device(i2c_adap, jz_i2c0_devs);
#endif

	i2c_put_adapter(i2c_adap);

	return 0;
}


static void  at24_dev_exit(void)
{
	 i2c_unregister_device(at24_client);
}



module_init(at24_dev_init);

module_exit(at24_dev_exit);


MODULE_LICENSE("GPL");
#endif
#ifdef CONFIG_I2C_GPIO
#define DEF_GPIO_I2C(NO)                        \
    static struct i2c_gpio_platform_data i2c##NO##_gpio_data = {    \
        .sda_pin    = GPIO_I2C##NO##_SDA,           \
        .scl_pin    = GPIO_I2C##NO##_SCK,           \
     };                              \
    struct platform_device i2c##NO##_gpio_device = {        \
        .name   = "i2c-gpio",                   \
        .id = NO,                       \
        .dev    = { .platform_data = &i2c##NO##_gpio_data,},    \
    };
#ifdef CONFIG_SOFT_I2C1_GPIO_V12_JZ
DEF_GPIO_I2C(1);
#endif
#ifdef CONFIG_SOFT_I2C0_GPIO_V12_JZ
DEF_GPIO_I2C(0);
#endif
#endif /*CONFIG_I2C_GPIO*/
