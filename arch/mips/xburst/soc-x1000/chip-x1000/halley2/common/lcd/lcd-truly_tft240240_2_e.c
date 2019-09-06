/*
 * Copyright (c) 2017 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * JZ-M200 orion board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/regulator/consumer.h>
#include <mach/jzfb.h>
#include "../board_base.h"

/*ifdef is 18bit,6-6-6 ,ifndef default 5-6-6*/
//#define CONFIG_SLCD_TRULY_18BIT
#define CONFIG_TRULY_320X240_ROTATE_180
//#define CONFIG_TRULY_320X240_ROTATE_90
#ifdef CONFIG_FIIO_LCD_ROTATE_90
#define FIIO_LCD_ROTATE 0xA0
#define FIIO_BEGIN_X 80
#define FIIO_END_X 319
#define FIIO_BEGIN_Y 0
#define FIIO_END_Y 239
#endif

#ifdef	CONFIG_SLCD_TRULY_18BIT
static int slcd_inited = 1;
#else
static int slcd_inited = 0;
#endif

struct truly_tft240240_power{
	struct regulator *vlcdio;
	struct regulator *vlcdvcc;
	int inited;
};

static struct truly_tft240240_power lcd_power = {
	NULL,
    NULL,
    0
};

int truly_tft240240_power_init(struct lcd_device *ld)
{
    int ret ;
    printk("======truly_tft240240_power_init==============\n");
    if(GPIO_LCD_RST > 0){
        ret = gpio_request(GPIO_LCD_RST, "lcd rst");
        if (ret) {
            printk(KERN_ERR "can's request lcd rst\n");
            return ret;
        }
    }
    if(GPIO_LCD_CS > 0){
        ret = gpio_request(GPIO_LCD_CS, "lcd cs");
        if (ret) {
            printk(KERN_ERR "can's request lcd cs\n");
            return ret;
        }
    }
    if(GPIO_LCD_RD > 0){
        ret = gpio_request(GPIO_LCD_RD, "lcd rd");
        if (ret) {
            printk(KERN_ERR "can's request lcd rd\n");
            return ret;
        }
    }
    printk("set lcd_power.inited  =======1 \n");
    lcd_power.inited = 1;
    return 0;
}
int truly_tft240240_power_reset(struct lcd_device *ld)
{
	if (!lcd_power.inited)
		return -EFAULT;
	//printk("%s:Enter !!!!\n",__func__);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(20);
	gpio_direction_output(GPIO_LCD_RST, 0);
	mdelay(200);
	gpio_direction_output(GPIO_LCD_RST, 1);
	mdelay(200);
	//printk("%s:Exit !!!!\n",__func__);
	return 0;
}

int truly_tft240240_power_on(struct lcd_device *ld, int enable)
{
	if (!lcd_power.inited && truly_tft240240_power_init(ld))
		return -EFAULT;
	//printk("%s:Enter enable=%d!!!!\n",__func__,enable);
	if (enable) {
		gpio_direction_output(GPIO_LCD_CS, 1);
		gpio_direction_output(GPIO_LCD_RD, 1);

		truly_tft240240_power_reset(ld);

		gpio_direction_output(GPIO_LCD_CS, 0);

	} else {
		mdelay(5);
		gpio_direction_output(GPIO_LCD_CS, 0);
		gpio_direction_output(GPIO_LCD_RD, 0);
		gpio_direction_output(GPIO_LCD_RST, 0);
		slcd_inited = 0;
	}
	//printk("%s:Exit !!!!\n",__func__);
	return 0;
}

struct lcd_platform_data truly_tft240240_pdata = {
	.reset    = truly_tft240240_power_reset,
	.power_on = truly_tft240240_power_on,
};

/* LCD Panel Device */
struct platform_device truly_tft240240_device = {
	.name		= "truly_tft240240_slcd",
	.dev		= {
		.platform_data	= &truly_tft240240_pdata,
	},
};
//#define M5_LCD_VERSION_1_0
#define M5_LCD_VERSION_2_0

static struct smart_lcd_data_table truly_tft240240_data_table[] = {
      /* LCD init code */
    {SMART_CONFIG_CMD, 0x01},  //soft reset, 120 ms = 120 000 us
    {SMART_CONFIG_UDELAY, 120000},
    {SMART_CONFIG_CMD, 0x11},
    {SMART_CONFIG_UDELAY, 5000},	  /* sleep out 5 ms  */

    {SMART_CONFIG_CMD, 0x36},
#ifdef	CONFIG_TRULY_320X240_ROTATE_180
    /*{0x36, 0xc0, 2, 0}, //40*/
#ifdef CONFIG_FIIO_LCD_ROTATE_90
    {SMART_CONFIG_DATA, FIIO_LCD_ROTATE}, //40
#else
	{SMART_CONFIG_DATA, 0x40}, //40
#endif

#else
    {SMART_CONFIG_DATA, 0x00}, //40
#endif
/*
    {SMART_CONFIG_CMD, 0x2a},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xef},

    {SMART_CONFIG_CMD, 0x2b},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xef},

*/
    {SMART_CONFIG_CMD, 0x3a},
#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
    {SMART_CONFIG_DATA, 0x06}, //6-6-6
#else
    {SMART_CONFIG_DATA, 0x05}, //5-6-5
#endif
    //	{SMART_CONFIG_DATA, 0x55},

    {SMART_CONFIG_CMD, 0xb2},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x33},
/*
    {SMART_CONFIG_CMD, 0xb3},
    {SMART_CONFIG_DATA, 0x10},
    {SMART_CONFIG_DATA, 0x05},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xb4},
    {SMART_CONFIG_DATA, 0x0b},
*/
    {SMART_CONFIG_CMD, 0xb7},
    {SMART_CONFIG_DATA, 0x35},

    {SMART_CONFIG_CMD, 0xbb},
    {SMART_CONFIG_DATA, 0x1f}, //23

    {SMART_CONFIG_CMD, 0xbc},
    {SMART_CONFIG_DATA, 0xec},
    {SMART_CONFIG_CMD, 0xbd},
    {SMART_CONFIG_DATA, 0xfe},
/*
    {SMART_CONFIG_CMD, 0xc0},
    {SMART_CONFIG_DATA, 0x2c},
*/
    {SMART_CONFIG_CMD, 0xc2},
    {SMART_CONFIG_DATA, 0x01},

    {SMART_CONFIG_CMD, 0xc3},
    {SMART_CONFIG_DATA, 0x19}, //14

    {SMART_CONFIG_CMD, 0xc4},
    {SMART_CONFIG_DATA, 0x20},

    {SMART_CONFIG_CMD, 0xc6},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xd0},
    {SMART_CONFIG_DATA, 0xa4},
    {SMART_CONFIG_DATA, 0xa1},
    
    {SMART_CONFIG_CMD, 0xd6},
    {SMART_CONFIG_DATA, 0xa1},

    {SMART_CONFIG_CMD, 0xe0},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x09},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x32},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x49},
    {SMART_CONFIG_DATA, 0x19},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x2b},
    {SMART_CONFIG_DATA, 0x34},

    {SMART_CONFIG_CMD, 0xe1},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x09},
    {SMART_CONFIG_DATA, 0x11},
    {SMART_CONFIG_DATA, 0x37},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x49},
    {SMART_CONFIG_DATA, 0x19},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x2d},
    {SMART_CONFIG_DATA, 0x34},

   // {SMART_CONFIG_CMD, 0x35}, // TE on
   // {SMART_CONFIG_DATA, 0x00}, // TE mode: 0, mode1; 1, mode2
    //	{SMART_CONFIG_CMD, 0x34}, // TE off
    {SMART_CONFIG_CMD, 0x21},
    {SMART_CONFIG_CMD, 0x29}, //Display ON

    /* set window size*/
        //	{SMART_CONFIG_CMD, 0xcd},
#ifdef CONFIG_FIIO_LCD_ROTATE_90
    {SMART_CONFIG_CMD, 0x2a},
    {SMART_CONFIG_DATA, FIIO_BEGIN_X},
    {SMART_CONFIG_DATA, FIIO_BEGIN_X},
    {SMART_CONFIG_DATA, (FIIO_END_X>> 8) & 0xff},
    {SMART_CONFIG_DATA, FIIO_END_X & 0xff},

    {SMART_CONFIG_CMD, 0x2b},
    {SMART_CONFIG_DATA, FIIO_BEGIN_Y},
    {SMART_CONFIG_DATA, FIIO_BEGIN_Y},
    {SMART_CONFIG_DATA, (FIIO_END_Y>> 8) & 0xff},
    {SMART_CONFIG_DATA, FIIO_END_Y & 0xff},
#else
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (239>> 8) & 0xff},
	{SMART_CONFIG_DATA, 239 & 0xff},

	{SMART_CONFIG_CMD, 0x2b},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (319>> 8) & 0xff},
	{SMART_CONFIG_DATA, 319 & 0xff},
 #endif

//#endif
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
};


static struct smart_lcd_data_table truly_tft240240_off_data_table[] = {
      /* LCD init code */
    {SMART_CONFIG_CMD, 0x01},  //soft reset, 120 ms = 120 000 us
    {SMART_CONFIG_UDELAY, 120000},
    {SMART_CONFIG_CMD, 0x11},
    {SMART_CONFIG_UDELAY, 5000},	  /* sleep out 5 ms  */

    {SMART_CONFIG_CMD, 0x36},
#ifdef	CONFIG_TRULY_320X240_ROTATE_180
    /*{0x36, 0xc0, 2, 0}, //40*/
#ifdef CONFIG_FIIO_LCD_ROTATE_90
    {SMART_CONFIG_DATA, FIIO_LCD_ROTATE}, //40
#else
	{SMART_CONFIG_DATA, 0x40}, //40
#endif
#else
    {SMART_CONFIG_DATA, 0x00}, //40
#endif
/*
    {SMART_CONFIG_CMD, 0x2a},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xef},

    {SMART_CONFIG_CMD, 0x2b},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xef},

*/
    {SMART_CONFIG_CMD, 0x3a},
#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
    {SMART_CONFIG_DATA, 0x06}, //6-6-6
#else
    {SMART_CONFIG_DATA, 0x05}, //5-6-5
#endif
    //	{SMART_CONFIG_DATA, 0x55},

    {SMART_CONFIG_CMD, 0xb2},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x33},
/*
    {SMART_CONFIG_CMD, 0xb3},
    {SMART_CONFIG_DATA, 0x10},
    {SMART_CONFIG_DATA, 0x05},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xb4},
    {SMART_CONFIG_DATA, 0x0b},
*/
    {SMART_CONFIG_CMD, 0xb7},
    {SMART_CONFIG_DATA, 0x35},

    {SMART_CONFIG_CMD, 0xbb},
    {SMART_CONFIG_DATA, 0x1f}, //23

    {SMART_CONFIG_CMD, 0xbc},
    {SMART_CONFIG_DATA, 0xec},
    {SMART_CONFIG_CMD, 0xbd},
    {SMART_CONFIG_DATA, 0xfe},
/*
    {SMART_CONFIG_CMD, 0xc0},
    {SMART_CONFIG_DATA, 0x2c},
*/
    {SMART_CONFIG_CMD, 0xc2},
    {SMART_CONFIG_DATA, 0x01},

    {SMART_CONFIG_CMD, 0xc3},
    {SMART_CONFIG_DATA, 0x19}, //14

    {SMART_CONFIG_CMD, 0xc4},
    {SMART_CONFIG_DATA, 0x20},

    {SMART_CONFIG_CMD, 0xc6},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xd0},
    {SMART_CONFIG_DATA, 0xa4},
    {SMART_CONFIG_DATA, 0xa1},
    
    {SMART_CONFIG_CMD, 0xd6},
    {SMART_CONFIG_DATA, 0xa1},

    {SMART_CONFIG_CMD, 0xe0},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x09},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x32},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x49},
    {SMART_CONFIG_DATA, 0x19},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x2b},
    {SMART_CONFIG_DATA, 0x34},

    {SMART_CONFIG_CMD, 0xe1},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x09},
    {SMART_CONFIG_DATA, 0x11},
    {SMART_CONFIG_DATA, 0x37},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x49},
    {SMART_CONFIG_DATA, 0x19},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x2d},
    {SMART_CONFIG_DATA, 0x34},

   // {SMART_CONFIG_CMD, 0x35}, // TE on
   // {SMART_CONFIG_DATA, 0x00}, // TE mode: 0, mode1; 1, mode2
    //	{SMART_CONFIG_CMD, 0x34}, // TE off

#ifdef CONFIG_FIIO_LCD_ROTATE_90
		{SMART_CONFIG_CMD, 0x2a},
		{SMART_CONFIG_DATA, FIIO_BEGIN_X},
		{SMART_CONFIG_DATA, FIIO_BEGIN_X},
		{SMART_CONFIG_DATA, (FIIO_END_X>> 8) & 0xff},
		{SMART_CONFIG_DATA, FIIO_END_X & 0xff},
	
		{SMART_CONFIG_CMD, 0x2b},
		{SMART_CONFIG_DATA, FIIO_BEGIN_Y},
		{SMART_CONFIG_DATA, FIIO_BEGIN_Y},
		{SMART_CONFIG_DATA, (FIIO_END_Y>> 8) & 0xff},
		{SMART_CONFIG_DATA, FIIO_END_Y & 0xff},
#else
		{SMART_CONFIG_CMD, 0x2a},
		{SMART_CONFIG_DATA, 0},
		{SMART_CONFIG_DATA, 0},
		{SMART_CONFIG_DATA, (239>> 8) & 0xff},
		{SMART_CONFIG_DATA, 239 & 0xff},
	
		{SMART_CONFIG_CMD, 0x2b},
		{SMART_CONFIG_DATA, 0},
		{SMART_CONFIG_DATA, 0},
		{SMART_CONFIG_DATA, (319>> 8) & 0xff},
		{SMART_CONFIG_DATA, 319 & 0xff},
 #endif
 
    {SMART_CONFIG_CMD, 0x21},
    {SMART_CONFIG_CMD, 0x28}, //Display off
	{SMART_CONFIG_CMD, 0x10},
	{SMART_CONFIG_UDELAY, 120000},
    /* set window size*/
        //	{SMART_CONFIG_CMD, 0xcd},



//#endif
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
};

#if 1
static struct smart_lcd_data_table new_truly_tft240240_data_table[] = {
#ifdef M5_LCD_VERSION_1_0
        /* LCD init code */
    {SMART_CONFIG_CMD, 0x01},  //soft reset, 120 ms = 120 000 us
    {SMART_CONFIG_UDELAY, 120000},
    {SMART_CONFIG_CMD, 0x11},
    {SMART_CONFIG_UDELAY, 120000},	  /* sleep out 120 ms  */

    {SMART_CONFIG_CMD, 0x36},
#ifdef	CONFIG_TRULY_320X240_ROTATE_180
    /*{0x36, 0xc0, 2, 0}, //40*/
#ifdef CONFIG_FIIO_LCD_ROTATE_90
    {SMART_CONFIG_DATA, FIIO_LCD_ROTATE}, //40
#else
	{SMART_CONFIG_DATA, 0x40}, //40
#endif
#else
    {SMART_CONFIG_DATA, 0x00}, //40
#endif
/*
    {SMART_CONFIG_CMD, 0x2a},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xef},

    {SMART_CONFIG_CMD, 0x2b},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0xef},

*/
    {SMART_CONFIG_CMD, 0x3a},
#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
    {SMART_CONFIG_DATA, 0x06}, //6-6-6
#else
    {SMART_CONFIG_DATA, 0x05}, //5-6-5
#endif
    //	{SMART_CONFIG_DATA, 0x55},

    {SMART_CONFIG_CMD, 0xb2},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x33},
/*
    {SMART_CONFIG_CMD, 0xb3},
    {SMART_CONFIG_DATA, 0x10},
    {SMART_CONFIG_DATA, 0x05},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xb4},
    {SMART_CONFIG_DATA, 0x0b},
*/
    {SMART_CONFIG_CMD, 0xb7},
    {SMART_CONFIG_DATA, 0x35},

    {SMART_CONFIG_CMD, 0xbb},
    {SMART_CONFIG_DATA, 0x1f}, //23

    {SMART_CONFIG_CMD, 0xbc},
    {SMART_CONFIG_DATA, 0xec},
    {SMART_CONFIG_CMD, 0xbd},
    {SMART_CONFIG_DATA, 0xfe},
/*
    {SMART_CONFIG_CMD, 0xc0},
    {SMART_CONFIG_DATA, 0x2c},
*/
    {SMART_CONFIG_CMD, 0xc2},
    {SMART_CONFIG_DATA, 0x01},

    {SMART_CONFIG_CMD, 0xc3},
    {SMART_CONFIG_DATA, 0x19}, //14

    {SMART_CONFIG_CMD, 0xc4},
    {SMART_CONFIG_DATA, 0x20},

    {SMART_CONFIG_CMD, 0xc6},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xd0},
    {SMART_CONFIG_DATA, 0xa4},
    {SMART_CONFIG_DATA, 0xa1},
    
    {SMART_CONFIG_CMD, 0xd6},
    {SMART_CONFIG_DATA, 0xa1},

    {SMART_CONFIG_CMD, 0xe0},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x09},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x32},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x49},
    {SMART_CONFIG_DATA, 0x19},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x2b},
    {SMART_CONFIG_DATA, 0x34},

    {SMART_CONFIG_CMD, 0xe1},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0a},
    {SMART_CONFIG_DATA, 0x09},
    {SMART_CONFIG_DATA, 0x11},
    {SMART_CONFIG_DATA, 0x37},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x49},
    {SMART_CONFIG_DATA, 0x19},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x2d},
    {SMART_CONFIG_DATA, 0x34},

   // {SMART_CONFIG_CMD, 0x35}, // TE on
   // {SMART_CONFIG_DATA, 0x00}, // TE mode: 0, mode1; 1, mode2
    //	{SMART_CONFIG_CMD, 0x34}, // TE off
    {SMART_CONFIG_CMD, 0x21},
    {SMART_CONFIG_CMD, 0x29}, //Display ON

    /* set window size*/
        //	{SMART_CONFIG_CMD, 0xcd},

#ifdef CONFIG_FIIO_LCD_ROTATE_90
    {SMART_CONFIG_CMD, 0x2a},
    {SMART_CONFIG_DATA, FIIO_BEGIN_X},
    {SMART_CONFIG_DATA, FIIO_BEGIN_X},
    {SMART_CONFIG_DATA, (FIIO_END_X>> 8) & 0xff},
    {SMART_CONFIG_DATA, FIIO_END_X & 0xff},

    {SMART_CONFIG_CMD, 0x2b},
    {SMART_CONFIG_DATA, FIIO_BEGIN_Y},
    {SMART_CONFIG_DATA, FIIO_BEGIN_Y},
    {SMART_CONFIG_DATA, (FIIO_END_Y>> 8) & 0xff},
    {SMART_CONFIG_DATA, FIIO_END_Y & 0xff},
#else
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (239>> 8) & 0xff},
	{SMART_CONFIG_DATA, 239 & 0xff},

	{SMART_CONFIG_CMD, 0x2b},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (319>> 8) & 0xff},
	{SMART_CONFIG_DATA, 319 & 0xff},
 #endif
//#endif
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
#endif
///2.0
#ifdef M5_LCD_VERSION_2_0
        /* LCD init code */
    //{SMART_CONFIG_CMD, 0x01},  //soft reset, 120 ms = 120 000 us
    //{SMART_CONFIG_UDELAY, 120000},
    {SMART_CONFIG_CMD, 0x11},
    {SMART_CONFIG_UDELAY, 120000},	  /* sleep out 200 ms  */

    {SMART_CONFIG_CMD, 0x36},
	{SMART_CONFIG_DATA, 0xC0},

    {SMART_CONFIG_CMD, 0x3a},
    {SMART_CONFIG_DATA, 0x05}, //5-6-5

    {SMART_CONFIG_CMD, 0xb2},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x33},

    {SMART_CONFIG_CMD, 0xb7},
    {SMART_CONFIG_DATA, 0x35},

    {SMART_CONFIG_CMD, 0xbb},
    {SMART_CONFIG_DATA, 0x32}, //23

    {SMART_CONFIG_CMD, 0xc2},
    {SMART_CONFIG_DATA, 0x01},

    {SMART_CONFIG_CMD, 0xc3},
    {SMART_CONFIG_DATA, 0x19}, //14

    {SMART_CONFIG_CMD, 0xc4},
    {SMART_CONFIG_DATA, 0x20},

    {SMART_CONFIG_CMD, 0xc6},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xd0},
    {SMART_CONFIG_DATA, 0xa4},
    {SMART_CONFIG_DATA, 0xa1},
    
    {SMART_CONFIG_CMD, 0xd6},
    {SMART_CONFIG_DATA, 0xa1},

    {SMART_CONFIG_CMD, 0xe0},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x08},
    {SMART_CONFIG_DATA, 0x0D},
    {SMART_CONFIG_DATA, 0x0A},
    {SMART_CONFIG_DATA, 0x0A},
    {SMART_CONFIG_DATA, 0x0A},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x48},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x0D},
    {SMART_CONFIG_DATA, 0x0B},
    {SMART_CONFIG_DATA, 0x29},
    {SMART_CONFIG_DATA, 0x2E},

    {SMART_CONFIG_CMD, 0xe1},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x07},
    {SMART_CONFIG_DATA, 0x0D},
    {SMART_CONFIG_DATA, 0x0B},
    {SMART_CONFIG_DATA, 0x0A},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x32},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x48},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x0C},
    {SMART_CONFIG_DATA, 0x0B},
    {SMART_CONFIG_DATA, 0x29},
    {SMART_CONFIG_DATA, 0x2D},

    /* set window size*/
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (239>> 8) & 0xff},
	{SMART_CONFIG_DATA, 239 & 0xff},

	{SMART_CONFIG_CMD, 0x2b},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (319>> 8) & 0xff},
	{SMART_CONFIG_DATA, 319 & 0xff},
	
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    
    {SMART_CONFIG_CMD, 0x21},
    {SMART_CONFIG_UDELAY, 120000},
    {SMART_CONFIG_CMD, 0x29}, //Display ON

    #endif
	
};
#else
static struct smart_lcd_data_table new_truly_tft240240_data_table[] = {
        /* LCD init code */
    //{SMART_CONFIG_CMD, 0x01},  //soft reset, 120 ms = 120 000 us
    {SMART_CONFIG_UDELAY, 120000},
    {SMART_CONFIG_CMD, 0x11},
    {SMART_CONFIG_UDELAY, 120000},	  /* sleep out 5 ms  */

    {SMART_CONFIG_CMD, 0x36},
#ifdef	CONFIG_TRULY_320X240_ROTATE_180
    /*{0x36, 0xc0, 2, 0}, //40*/
#ifdef CONFIG_FIIO_LCD_ROTATE_90
    {SMART_CONFIG_DATA, FIIO_LCD_ROTATE}, //40
#else
	{SMART_CONFIG_DATA, 0x40}, //40
#endif
#else
    {SMART_CONFIG_DATA, 0x00}, //40
#endif
#ifdef CONFIG_FIIO_LCD_ROTATE_90
    {SMART_CONFIG_CMD, 0x2a},
    {SMART_CONFIG_DATA, FIIO_BEGIN_X},
    {SMART_CONFIG_DATA, FIIO_BEGIN_X},
    {SMART_CONFIG_DATA, (FIIO_END_X>> 8) & 0xff},
    {SMART_CONFIG_DATA, FIIO_END_X & 0xff},

    {SMART_CONFIG_CMD, 0x2b},
    {SMART_CONFIG_DATA, FIIO_BEGIN_Y},
    {SMART_CONFIG_DATA, FIIO_BEGIN_Y},
    {SMART_CONFIG_DATA, (FIIO_END_Y>> 8) & 0xff},
    {SMART_CONFIG_DATA, FIIO_END_Y & 0xff},
#else
	{SMART_CONFIG_CMD, 0x2a},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (239>> 8) & 0xff},
	{SMART_CONFIG_DATA, 239 & 0xff},

	{SMART_CONFIG_CMD, 0x2b},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, 0},
	{SMART_CONFIG_DATA, (319>> 8) & 0xff},
	{SMART_CONFIG_DATA, 319 & 0xff},
 #endif

    {SMART_CONFIG_CMD, 0x3a},
#if defined(CONFIG_SLCD_TRULY_18BIT)  //if 18bit/pixel unusual. try to use 16bit/pixel
    {SMART_CONFIG_DATA, 0x06}, //6-6-6
#else
    {SMART_CONFIG_DATA, 0x05}, //5-6-5
#endif
    //	{SMART_CONFIG_DATA, 0x55},

    {SMART_CONFIG_CMD, 0xb2},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x00},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x33},
    
    {SMART_CONFIG_CMD, 0xb7},
    {SMART_CONFIG_DATA, 0x35},

    {SMART_CONFIG_CMD, 0xbb},
    {SMART_CONFIG_DATA, 0x1f}, //23
#if 0
    {SMART_CONFIG_CMD, 0xbc},
    {SMART_CONFIG_DATA, 0xec},
    {SMART_CONFIG_CMD, 0xbd},
    {SMART_CONFIG_DATA, 0xfe},
#endif
	{SMART_CONFIG_CMD, 0xc0},
	{SMART_CONFIG_DATA, 0x2c},

    {SMART_CONFIG_CMD, 0xc2},
    {SMART_CONFIG_DATA, 0x01},

    {SMART_CONFIG_CMD, 0xc3},
    {SMART_CONFIG_DATA, 0x0A}, //14

    {SMART_CONFIG_CMD, 0xc4},
    {SMART_CONFIG_DATA, 0x20},

    {SMART_CONFIG_CMD, 0xc6},
    {SMART_CONFIG_DATA, 0x0f},

    {SMART_CONFIG_CMD, 0xd0},
    {SMART_CONFIG_DATA, 0xa4},
    {SMART_CONFIG_DATA, 0xa1},
    
    {SMART_CONFIG_CMD, 0xd6},
    {SMART_CONFIG_DATA, 0xa1},

    {SMART_CONFIG_CMD, 0xe0},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x08},
    {SMART_CONFIG_DATA, 0x11},
    {SMART_CONFIG_DATA, 0x08},
    {SMART_CONFIG_DATA, 0x0c},
    {SMART_CONFIG_DATA, 0x15},
    {SMART_CONFIG_DATA, 0x39},
    {SMART_CONFIG_DATA, 0x33},
    {SMART_CONFIG_DATA, 0x50},
    {SMART_CONFIG_DATA, 0x36},
    {SMART_CONFIG_DATA, 0x13},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x29},
    {SMART_CONFIG_DATA, 0x2d},
    

    {SMART_CONFIG_CMD, 0xe1},
    {SMART_CONFIG_DATA, 0xd0},
    {SMART_CONFIG_DATA, 0x08},
    {SMART_CONFIG_DATA, 0x10},
    {SMART_CONFIG_DATA, 0x08},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x06},
    {SMART_CONFIG_DATA, 0x39},
    {SMART_CONFIG_DATA, 0x44},
    {SMART_CONFIG_DATA, 0x51},
    {SMART_CONFIG_DATA, 0x0b},
    {SMART_CONFIG_DATA, 0x16},
    {SMART_CONFIG_DATA, 0x14},
    {SMART_CONFIG_DATA, 0x2f},
    {SMART_CONFIG_DATA, 0x31},

    {SMART_CONFIG_CMD, 0x29}, //Display ON

    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
    {SMART_CONFIG_CMD, 0x2C},
};
 #endif

unsigned long truly_cmd_buf[]= {
	0x2C2C2C2C,
};

struct fb_videomode jzfb0_videomode = {
	.name = "240x240",
	.refresh = 60,
	.xres = 240,
	.yres = 240,
	.pixclock = KHZ2PICOS(30000),
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};


struct jzfb_platform_data jzfb_pdata = {
	.num_modes = 1,
	.modes = &jzfb0_videomode,
	.lcd_type = LCD_TYPE_SLCD,
	.bpp    = 16,
	.width = 27,
	.height = 27,
	.pinmd  = 0,

	.smart_config.rsply_cmd_high       = 0,
	.smart_config.csply_active_high    = 0,
	.smart_config.newcfg_fmt_conv =  1,
	/* write graphic ram command, in word, for example 8-bit bus, write_gram_cmd=C3C2C1C0. */
	.smart_config.write_gram_cmd = truly_cmd_buf,
	.smart_config.length_cmd = ARRAY_SIZE(truly_cmd_buf),
	.smart_config.bus_width = 16,
	.smart_config.data_table_width = 8,
	.smart_config.length_data_table =  ARRAY_SIZE(truly_tft240240_data_table),
	.smart_config.data_table = truly_tft240240_data_table,
	.smart_config.off_data_table = truly_tft240240_off_data_table,
	.dither_enable = 0,
};
/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM

void fix_new_lcd(void)
{
	unsigned int value=0;
	unsigned int pull_value=0;

	pull_value = *((volatile unsigned int*)(0xb0010300+0x70)) & (1 << 3);
	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x78)) = 1 << 3;

	value = (*((volatile unsigned int *)(0xb0010300+0))) & (1 << 3);
	if(!value) {
		jzfb_pdata.smart_config.length_data_table =  ARRAY_SIZE(new_truly_tft240240_data_table);
		jzfb_pdata.smart_config.data_table = new_truly_tft240240_data_table;
	}

	if(pull_value)
		*((volatile unsigned int *)(0xb0010300+0x74)) = 1 << 3;

}

static int backlight_init(struct device *dev)
{
	//int ret;
#if 0
	ret = gpio_request(GPIO_LCD_PWM, "Backlight");
	if (ret) {
		printk(KERN_ERR "failed to request GPF for PWM-OUT1\n");
		return ret;
	}
#endif
	printk("------------------------------------------------");
/*	ret = gpio_request(GPIO_BL_PWR_EN, "BL PWR");
	if (ret) {
		printk(KERN_ERR "failed to reqeust BL PWR\n");
		return ret;
	}
	gpio_direction_output(GPIO_BL_PWR_EN, 1);*/
	return 0;
}

static int backlight_notify(struct device *dev, int brightness)
{
	//if (brightness)
		//gpio_direction_output(GPIO_BL_PWR_EN, 1);
	//else
		//gpio_direction_output(GPIO_BL_PWR_EN, 0);

	return brightness;
}

static void backlight_exit(struct device *dev)
{
	gpio_free(GPIO_LCD_PWM);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 120,
	.pwm_period_ns	= 33000,
	.init		= backlight_init,
	.exit		= backlight_exit,
	.notify		= backlight_notify,
};

struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &backlight_data,
	},
};
#endif
