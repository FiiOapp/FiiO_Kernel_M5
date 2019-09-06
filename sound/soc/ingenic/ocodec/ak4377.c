/*
 * ak4377.c
 *
 * Supports AXP173 Regulator
 *
 * Copyright (C) 2009 Texas Instrument Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
//#include <linux/power/axp173.h>
#include <linux/interrupt.h> 
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <irq.h>
#include <soc/gpio.h>
#include <linux/ioport.h>
#include <linux/gpio.h>

#include <linux/spinlock.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/pmu.h>
#include <linux/skytc/eq/eq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/delay.h>
#include <sound/ak4377.h>
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>
//#include <linux/gpio_keys.h>
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif
#if 0
#define DBG_INFO(x...)	printk(KERN_INFO x)
#else
#define DBG_INFO(x...)
#endif
//debug
#if 0
#define FIIO_DEBUG_AK4377
#ifdef FIIO_DEBUG_AK4377
#define fiio_debug(x...)  printk(KERN_INFO "[fiio_ak4377] " x)
#else
#define fiio_debug(x...)
#endif
#endif
static int fiio_ak4377_debug = 0;
module_param(fiio_ak4377_debug, int, 0644);
#define fiio_debug(msg...)			\
	do {					\
		if (fiio_ak4377_debug)		\
			printk("ak4377: " msg);	\
	} while(0)

//default 0 dB
unsigned char volumeL = 0x39;
unsigned char volumeR = 0x39;
int work_sign=0;
unsigned char dsd_volume = 0;
bool ak4377_power_mode =0;
EXPORT_SYMBOL(volumeL);
EXPORT_SYMBOL(volumeR);
EXPORT_SYMBOL(work_sign);
EXPORT_SYMBOL(dsd_volume);
EXPORT_SYMBOL(ak4377_power_mode);

#define  uint8   unsigned char
#define  uint16  unsigned short
#define  int16   short
#define  uint32  unsigned int
#define  int32   int
#define  uint64  unsigned long

//add by pengweizhong
#define FIIO_AK4377_USB_IO_I2C
//#define FIIO_AK4377_DELAY_TIME
void fiio_set_dac_volume();
int get_fiio_ak4377_work_mode();
extern int get_fiio_m5_play_sample(void);
//0:high performance 1:low power mode
#define FIIO_AK4377_HIGH_PERFORMANCE_MODE 0
#define FIIO_AK4377_LOW_POWER_MODE 1

static int g_fiio_ak4377_work_mode = 0;

static int g_fiio_ak4377_is_power_flag = 0;

int ak4377_i2c_read(struct ak4377 *bq, u8 reg);
int ak4377_i2c_write(struct ak4377 *bq, u8 reg, u8 val);
struct ak4377_reg{
			 int reg;
			 int value;
};


//default pcm mode
static struct ak4377_reg ak4377_reg_defaults[] ={
	// AKM_Register AK4377 20
	{ 0x00 ,0x01 },
	{ 0x01 ,0x73 },
	{ 0x02 ,0x01 },
	{ 0x03 ,0x03 },
	{ 0x04 ,0x00 },
	{ 0x05 ,0x09 },
	{ 0x06 ,0x00 },
	{ 0x07 ,0x21 },
	//{ 0x0B ,0x00 },
	//{ 0x0C ,0x00 },
	{ 0x0D ,0x6B },
	{ 0x0E ,0x01 },
	{ 0x0F ,0x00 },
	{ 0x10 ,0x03 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x17 ,0x00 },
	{ 0x18 ,0x00 },
	{ 0x19 ,0x05 },
	{ 0x1A ,0xFA },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
	{ 0x24 ,0x00 },
};

static struct ak4377_reg ak4377_low_defaults[] ={
	// AKM_Register AK4377 20
	{ 0x00 ,0x01 },
	{ 0x01 ,0x73 },
	{ 0x02 ,0x11 },
	{ 0x03 ,0x03 },
	{ 0x04 ,0x00 },
	{ 0x05 ,0x09 },
	{ 0x06 ,0x00 },
	{ 0x07 ,0x21 },
	//{ 0x0B ,0x00 },
	//{ 0x0C ,0x00 },
	{ 0x0D ,0x6B },
	{ 0x0E ,0x01 },
	{ 0x0F ,0x00 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x17 ,0x00 },
	{ 0x18 ,0x00 },
	{ 0x19 ,0x05 },
	{ 0x1A ,0xFA },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
	{ 0x24 ,0x40 },
};

//////////////////////////low power mode ////////////////////
//512fs
static struct ak4377_reg ak4377_reg_8_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x20 },
    { 0x10 ,0x00 },
    { 0x12 ,0xEF },
    { 0x13 ,0x01 },
    { 0x14 ,0x1D },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_11_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x21 },
    { 0x10 ,0x00 },
    { 0x11 ,0x00 },
    { 0x12 ,0x9F },
    { 0x13 ,0x01 },
    { 0x14 ,0x13 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_12_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x22 },
    { 0x10 ,0x00 },
    { 0x11 ,0x00 },
    { 0x12 ,0x9F },
    { 0x13 ,0x01 },
    { 0x14 ,0x13 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_16_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x24 },
    { 0x10 ,0x00 },
    { 0x11 ,0x00 },
    { 0x12 ,0x77 },
    { 0x13 ,0x01 },
    { 0x14 ,0x0E },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_22_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x25 },
    { 0x10 ,0x00 },
    { 0x11 ,0x00 },
    { 0x12 ,0x4F },
    { 0x13 ,0x01 },
    { 0x14 ,0x09 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_24_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x26 },
    { 0x10 ,0x00 },
    { 0x11 ,0x00 },
    { 0x12 ,0x4F },
    { 0x13 ,0x01 },
    { 0x14 ,0x09 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
//256fs
static struct ak4377_reg ak4377_reg_32_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x08 },
    { 0x10 ,0x00 },
    { 0x11 ,0x00 },
    { 0x12 ,0x3B },
    { 0x13 ,0x01 },
    { 0x14 ,0x0E },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_defaults_44_low[] ={
	// AKM_Register AK4377 20
	{ 0x02 ,0x11 },
	{ 0x05 ,0x09 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
	{ 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_defaults_48_low[] ={
    // AKM_Register AK4377 20
    { 0x02 ,0x11 },
    { 0x05 ,0x0A },
    { 0x10 ,0x00 },
    { 0x11 ,0x00 },
    { 0x12 ,0x27 },
    { 0x13 ,0x01 },
    { 0x14 ,0x09 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};

static struct ak4377_reg ak4377_reg_64_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x0C },
    { 0x10 ,0x01 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x04 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_88_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x0D },
    { 0x10 ,0x01 },
    { 0x11 ,0x00 },
    { 0x12 ,0x27 },
    { 0x13 ,0x01 },
    { 0x14 ,0x04 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_96_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x0E },
    { 0x10 ,0x01 },
    { 0x11 ,0x00 },
    { 0x12 ,0x27 },
    { 0x13 ,0x01 },
    { 0x14 ,0x04 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_128_low[] ={
    { 0x02 ,0x11 },
    { 0x05 ,0x10 },
    { 0x10 ,0x01 },
    { 0x11 ,0x00 },
    { 0x12 ,0x27 },
    { 0x13 ,0x01 },
    { 0x14 ,0x04 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
static struct ak4377_reg ak4377_reg_defaults_176_low[] ={
    // AKM_Register AK4377 20
    { 0x02 ,0x11 },
    { 0x05 ,0x71 },
    { 0x10 ,0x03 },
    { 0x12 ,0x27 },
    { 0x13 ,0x11 },
    { 0x14 ,0x04 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};

		
static struct ak4377_reg ak4377_reg_defaults_192_low[] ={
    // AKM_Register AK4377 20
    { 0x02 ,0x11 },
    { 0x05 ,0x72 },
    { 0x10 ,0x03 },
    { 0x12 ,0x27 },
    { 0x13 ,0x11 },
    { 0x14 ,0x04 },
    { 0x15 ,0xE0 },
    { 0x16 ,0x01 },
    { 0x1E ,0x02 },
    { 0x1F ,0x1D },
    { 0x20 ,0x1D },
    { 0x24 ,0x40 },
};
//////////////////////////////////////////////////////////
//512fs	
static struct ak4377_reg ak4377_reg_8[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x20 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0xEF },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x1D },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_11[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x21 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x9F },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x13 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_12[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x22 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x9F },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x13 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_16[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x24 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x77 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x0E },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_22[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x25 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x4F },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_24[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x26 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x4F },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
//256fs
static struct ak4377_reg ak4377_reg_32[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x08 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x3B },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x0E },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};

static struct ak4377_reg ak4377_reg_defaults_44[] ={
	// AKM_Register AK4377 20
	{ 0x02 ,0x01 },
	{ 0x05 ,0x09 },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
	{ 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_defaults_48[] ={
	// AKM_Register AK4377 20
    { 0x02 ,0x01 },
	{ 0x05 ,0x0A },
	{ 0x10 ,0x00 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};

static struct ak4377_reg ak4377_reg_64[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x0C },
	{ 0x10 ,0x01 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },	
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_88[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x0D },
	{ 0x10 ,0x01 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_96[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x0E },
	{ 0x10 ,0x01 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_128[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x10 },
	{ 0x10 ,0x01 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_defaults_176[] ={
	// AKM_Register AK4377 20
    { 0x02 ,0x01 },
	{ 0x05 ,0x11 },
	{ 0x10 ,0x03 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x11 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};

static struct ak4377_reg ak4377_reg_defaults_192[] ={
	// AKM_Register AK4377 20
    { 0x02 ,0x01 },
	{ 0x05 ,0x12 },
	{ 0x10 ,0x03 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x11 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
//64fs
static struct ak4377_reg ak4377_reg_256[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x34 },
	{ 0x10 ,0x07 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x3B },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x0E },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_352[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x35 },
	{ 0x10 ,0x07 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_defaults_384[] ={
	// AKM_Register AK4377 20
	// { 0x00 ,0x01 },
	// { 0x01 ,0x73 },
       { 0x02 ,0x01 },
	// { 0x03 ,0x03 },
	//{ 0x04 ,0x00 },
	{ 0x05 ,0x36 },
	// { 0x06 ,0x00 },
	// { 0x07 ,0x21 },
	// { 0x0B ,0x39 },
	// { 0x0C ,0x39 },
	// { 0x0D ,0x69 },
	// { 0x0E ,0x01 },
	// { 0x0F ,0x00 },
	{ 0x10 ,0x07 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x04 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	// { 0x16 ,0x01 },
	// { 0x17 ,0x00 },
	// { 0x18 ,0x00 },
	// { 0x19 ,0x05 },
	// { 0x1A ,0xFA },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
	// { 0x24 ,0x00 },
    { 0x24 ,0x00 },

};
static struct ak4377_reg ak4377_reg_512[] ={
	{ 0x02 ,0x01 },
	{ 0x05 ,0x38 },
	{ 0x10 ,0x07 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x14 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x01 },
	{ 0x1E ,0x02 },
	{ 0x1F ,0x1D },
	{ 0x20 ,0x1D },
    { 0x24 ,0x00 },
};


static struct ak4377_reg ak4377_reg_dsd_256_28224_1764[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x09 },
	{ 0x10 ,0x03 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x04 },
	{ 0x1E ,0x00 },
	{ 0x1F ,0x00 },
	{ 0x20 ,0x00 },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_dsd_256_3072_192[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x0A },
	{ 0x10 ,0x03 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x04 },
	{ 0x1E ,0x00 },
	{ 0x1F ,0x00 },
	{ 0x20 ,0x00 },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_dsd_256_56448_3528[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x09 },
	{ 0x10 ,0x07 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x05 },
	{ 0x1E ,0x00 },
	{ 0x1F ,0x00 },
	{ 0x20 ,0x00 },
    { 0x24 ,0x00 },
};
static struct ak4377_reg ak4377_reg_dsd_256_6144_384[] ={
    { 0x02 ,0x01 },
	{ 0x05 ,0x0A },
	{ 0x10 ,0x07 },
	{ 0x11 ,0x00 },
	{ 0x12 ,0x27 },
	{ 0x13 ,0x01 },
	{ 0x14 ,0x09 },
	{ 0x15 ,0xE0 },
	{ 0x16 ,0x05 },
	{ 0x1E ,0x00 },
	{ 0x1F ,0x00 },
	{ 0x20 ,0x00 },
    { 0x24 ,0x00 },
};

static char driver_name[] = "ak4377";
struct ak4377 {
	struct device *dev;
	//struct mutex io_lock;
    int	irq;	/* if 0, use polling */
	//spinlock_t              io_lock;	
	struct i2c_client *i2c;
	int num_regulators;
	struct regulator_dev **rdev;
};
//static DEFINE_MUTEX(axp173_i2c_mutex);
struct ak4377 *g_ak4377;

static int ak4377_i2c_gpio_init(void)
{
    int ret ;
    fiio_debug("======%s:line:%d=============\n",__func__,__LINE__);
#ifdef FIIO_AK4377_USB_IO_I2C
    if(AK4377_SCL> 0){
	fiio_debug(KERN_INFO"try to request ak4377 scl\n");
        ret = gpio_request(AK4377_SCL, "ak4377 scl");
        if (ret) {
            printk(KERN_ERR "can's request ak4377 scl\n");
            return ret;
        }
    }
    if(AK4377_SDA> 0){
	printk(KERN_INFO"try to request ak4377 sda\n");
        ret = gpio_request(AK4377_SDA, "ak4377 sda");
        if (ret) {
            printk(KERN_ERR "can's request ak4377 sda\n");
            return ret;
        }
    }
#endif
    if(PO_EN> 0){
        printk(KERN_INFO"try to request po_en\n");
        ret = gpio_request(PO_EN, "po_en");
        if (ret) {
            printk(KERN_ERR "can's request po_en\n");
            return ret;
        }
	gpio_direction_output(PO_EN,0);
	//mdelay(1);
	//gpio_direction_output(PO_EN,1);
    }

}	

//
static void IIC_Start(void)
{
	ndelay(50);
	gpio_direction_output(AK4377_SDA,1);
  	ndelay(50);	
	gpio_direction_output(AK4377_SCL,1);
	ndelay(50);
 	gpio_direction_output(AK4377_SDA,0);
	ndelay(50);
	gpio_direction_output(AK4377_SCL,0);
  	ndelay(50);
}	  

static void IIC_Stop(void)
{
	ndelay(50);
	gpio_direction_output(AK4377_SCL,0);
	ndelay(50);
	gpio_direction_output(AK4377_SDA,0);
	ndelay(50);
	gpio_direction_output(AK4377_SCL,1);
    ndelay(50);
    gpio_direction_output(AK4377_SDA,1);
    ndelay(50);
}

static uint8 IIC_Wait_Ack(void)
{
	uint16 ucErrTime=0;
	
	gpio_direction_input(AK4377_SDA);      
	gpio_direction_output(AK4377_SCL,1);
	ndelay(100);
	while(gpio_get_value(AK4377_SDA))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			printk("ak4377 ack Outtime!\n");	
			IIC_Stop();
			return 1;
		}
	}
	gpio_direction_output(AK4377_SCL,0);
	return 0;  
} 


static void IIC_Ack(void)
{
	gpio_direction_output(AK4377_SCL,0);
    gpio_direction_output(AK4377_SDA,0);
	ndelay(20);
	gpio_direction_output(AK4377_SCL,1);
	ndelay(50);
	gpio_direction_output(AK4377_SCL,0);
}

static void IIC_NAck(void)
{
	gpio_direction_output(AK4377_SCL,0);
	gpio_direction_output(AK4377_SDA,1);
	ndelay(20);
	gpio_direction_output(AK4377_SCL,1);
	ndelay(50);
	gpio_direction_output(AK4377_SCL,0);
}

static void IIC_Send_Byte(uint8 txd)
{                        
    uint8 t;	    
    gpio_direction_output(AK4377_SCL,0);
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>>7)
			gpio_direction_output(AK4377_SDA,1);
		else 
			gpio_direction_output(AK4377_SDA,0);
        txd<<=1; 	  
		ndelay(50);  
		gpio_direction_output(AK4377_SCL,1);
		ndelay(50);
    	ndelay(50);			
		gpio_direction_output(AK4377_SCL,0);
		
    }	 
		ndelay(50);
		
} 	    

static uint8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	gpio_direction_input(AK4377_SDA);
    for(i=0;i<8;i++ )
	{
        gpio_direction_output(AK4377_SCL,0);
        ndelay(50);
		gpio_direction_output(AK4377_SCL,1);
        receive<<=1;
        if(gpio_get_value(AK4377_SDA))receive++;   
		ndelay(50); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();    
    return receive;
}

unsigned char AK4377_ReadOneByte(uint8 ReadAddr)
{		
	uint8 temp=0;		
	#ifndef FIIO_AK4377_USB_IO_I2C
		temp = ak4377_i2c_read(ak4377,ReadAddr);
	#else	    	    																 
		IIC_Start();  
		IIC_Send_Byte(0x20);     
		IIC_Wait_Ack(); 
		IIC_Send_Byte(ReadAddr);
		IIC_Wait_Ack();	    
		IIC_Start();  	 	   
		IIC_Send_Byte(0x21);
		IIC_Wait_Ack();	 
		temp=IIC_Read_Byte(1);
		IIC_Wait_Ack();
		IIC_Stop();		
	#endif
	
	return temp;
}
EXPORT_SYMBOL(AK4377_ReadOneByte);
void AK4377_WriteOneByte_FIIO(uint8 WriteAddr,uint8 DataToWrite)
{			
	if (0 == g_fiio_ak4377_is_power_flag)
		return;
    if (WriteAddr == 0x0C || WriteAddr == 0x0B) {
		
       DataToWrite = 0x39;
    }
	fiio_debug("%s WriteAddr=%02xH Data=%02xH\n",__func__,WriteAddr,DataToWrite);   	  	    																 
    IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);
	IIC_Wait_Ack();	   
    IIC_Send_Byte(DataToWrite);   
	IIC_Wait_Ack();
    IIC_Stop();
	ndelay(100);	 
}
void AK4377_WriteOneByte(uint8 WriteAddr,uint8 DataToWrite)
{		
	if (0 == g_fiio_ak4377_is_power_flag)
		return;
    if (WriteAddr == 0x0C || WriteAddr == 0x0B) {
		DataToWrite = 0x39;
    }

	fiio_debug("%s WriteAddr=%02xH Data=%02xH\n",__func__,WriteAddr,DataToWrite);   	  	    																 
    IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);
	IIC_Wait_Ack();	   
    IIC_Send_Byte(DataToWrite);   
	IIC_Wait_Ack();
    IIC_Stop();
	ndelay(100);
}
EXPORT_SYMBOL(AK4377_WriteOneByte);

void AK4377_WriteOneByte_Volume(uint8 WriteAddr,uint8 DataToWrite)
{		
	if (0 == g_fiio_ak4377_is_power_flag) {
		fiio_debug("%s can't write ak4377 for not power up\n",__func__);  
		return;
	}
		

	fiio_debug("%s WriteAddr=%02xH Data=%02xH\n",__func__,WriteAddr,DataToWrite);   	  	    																 
	IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);
	IIC_Wait_Ack();	   
	IIC_Send_Byte(DataToWrite);   
	IIC_Wait_Ack();
	IIC_Stop();
	ndelay(100);
}
EXPORT_SYMBOL(AK4377_WriteOneByte_Volume);

//mute hpg
void m5_hpg_mute() {
	int hpg_value = AK4377_ReadOneByte(0x0D);
	//mute 1111 1000
	hpg_value &= 0xF0;
	AK4377_WriteOneByte(0x0D,hpg_value);
}
EXPORT_SYMBOL(m5_hpg_mute);

void m5_hpg_unmute() {
	AK4377_WriteOneByte(0x0D,0x6B);
}
EXPORT_SYMBOL(m5_hpg_unmute);



void AK4377_WriteOneByte_test(uint8 WriteAddr,uint8 DataToWrite)
{		
  
	if (0 == g_fiio_ak4377_is_power_flag) {
		fiio_debug("%s can't write ak4377 for not power up\n",__func__);   	
		return;
	}
		
	fiio_debug("%s WriteAddr=%02xH Data=%02xH\n",__func__,WriteAddr,DataToWrite);   	  	    																 
    IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);
	IIC_Wait_Ack();	   
    IIC_Send_Byte(DataToWrite);   
	IIC_Wait_Ack();
    IIC_Stop();
	ndelay(100);
}
EXPORT_SYMBOL(AK4377_WriteOneByte_test);

void m5_dac_volume() {
	AK4377_WriteOneByte_test(0x0B,0x39);
	AK4377_WriteOneByte_test(0x0C,0x39);
}
EXPORT_SYMBOL(m5_dac_volume);

void m5_dac_mute() {
	AK4377_WriteOneByte_test(0x0B,0x00);
	AK4377_WriteOneByte_test(0x0C,0x00);
}
EXPORT_SYMBOL(m5_dac_mute);

void AK4377_WriteOneByte_Init(uint8 WriteAddr,uint8 DataToWrite)
{		
  if (0 == g_fiio_ak4377_is_power_flag)
		return;
	fiio_debug("%s WriteAddr=%02xH Data=%02xH\n",__func__,WriteAddr,DataToWrite);   	  	    																 
    IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);
	IIC_Wait_Ack();	   
    IIC_Send_Byte(DataToWrite);   
	IIC_Wait_Ack();
    IIC_Stop();
	ndelay(100);
}
int fiio_get_dsd_output_format(void);
static void ak4377_work_func(struct work_struct *work)
{
	fiio_debug("ak4377_work_func set volume \n");
	if (1 == fiio_get_dsd_output_format()) {
		//fiio_set_dsd_volume(dsd_volume);
	}
	else {
		AK4377_WriteOneByte_Volume(0x0B,0x39);
		AK4377_WriteOneByte_Volume(0x0C,0x39);
		AK4377_WriteOneByte_Volume(0x0D,0x0B);
	}
	
}
static DECLARE_DELAYED_WORK(ak4377_delay_work, ak4377_work_func);
void AK4377_Mute(void)
{		
    if (0 == g_fiio_ak4377_is_power_flag)
		return;
	int ret=work_busy((struct work_struct*)&ak4377_delay_work);
	if(ret&WORK_BUSY_PENDING) {
		printk(KERN_WARNING "%s ak4377_delay_work work_func WORK_BUSY_PENDING cancel it\n",__func__);
		cancel_delayed_work_sync(&ak4377_delay_work);
	}
	if(ret& WORK_BUSY_RUNNING) {
		printk("%s ak4377_delay_work work_func WORK_BUSY_RUNNING wait it to finish\n",__func__);
		flush_workqueue(ak4377_delay_work.wq);
	}	
	fiio_debug("Enter:%s\n",__func__);   	  	    																 
    IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x0B);
	IIC_Wait_Ack();	   
   IIC_Send_Byte(0x00);   
	IIC_Wait_Ack();
   IIC_Stop();
	ndelay(100);

	IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x0C);
	IIC_Wait_Ack();	   
    IIC_Send_Byte(0x00);   
	IIC_Wait_Ack();
    IIC_Stop();
	ndelay(100);

	IIC_Start();  
	IIC_Send_Byte(0x20);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x0D);
	IIC_Wait_Ack();	   
    IIC_Send_Byte(0x00);   
	IIC_Wait_Ack();
    IIC_Stop();
	ndelay(100);
	fiio_debug("Exit:%s\n",__func__);   	
}
EXPORT_SYMBOL(AK4377_Mute);
/* read value from register */
int ak4377_i2c_read(struct ak4377 *bq, u8 reg)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
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

	//mutex_lock(&axp173_i2c_mutex);
	//spin_lock(&axp173_i2c_lock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	//ret = __i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	//mutex_unlock(&axp173_i2c_mutex);
	//spin_unlock(&axp173_i2c_lock);
	if (ret < 0)
		return ret;

	return val;
}
EXPORT_SYMBOL(ak4377_i2c_read);

/* write value to register */
int ak4377_i2c_write(struct ak4377 *bq, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = val;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = ARRAY_SIZE(data);

	//mutex_lock(&axp173_i2c_mutex);
	//spin_lock(&axp173_i2c_lock);
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	//mutex_unlock(&axp173_i2c_mutex);
	//spin_unlock(&axp173_i2c_lock);
	/* i2c_transfer returns number of messages transferred */
	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(ak4377_i2c_write);
/* read value from register, change it with mask left shifted and write back */
static int ak4377_i2c_write_mask(struct ak4377 *bq, u8 reg, u8 val,
				  u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = ak4377_i2c_read(bq, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val << shift;

	return ak4377_i2c_write(bq, reg, ret);
}

/* read value from register, apply mask and right shift it */
static int ak4377_i2c_read_mask(struct ak4377 *bq, u8 reg,
				 u8 mask, u8 shift)
{
	int ret;

	if (shift > 8)
		return -EINVAL;

	ret = ak4377_i2c_read(bq, reg);
	if (ret < 0)
		return ret;
	return (ret & mask) >> shift;
}


/* read value from register and return one specified bit */
static int ak4377_i2c_read_bit(struct ak4377 *bq, u8 reg, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return ak4377_i2c_read_mask(bq, reg, BIT(bit), bit);
}
//EXPORT_SYMBOL(axp173_i2c_read_bit);

/* change only one bit in register */
static int ak4377_i2c_write_bit(struct ak4377 *bq, u8 reg,
				 bool val, u8 bit)
{
	if (bit > 8)
		return -EINVAL;
	return ak4377_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

void ak4377_cache_reg_init(void)
{

	int i; 
	for(i=0; i<28; i++)
	{
		ak4377_i2c_write(g_ak4377,ak4377_reg_defaults[i].reg, ak4377_reg_defaults[i].value);
                
  	}

}
void ak4377_reg_init_8(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_8)/sizeof(ak4377_reg_8[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_8[i].reg, ak4377_reg_8[i].value);
  	}
}

void ak4377_reg_init_11(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_11)/sizeof(ak4377_reg_11[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_11[i].reg, ak4377_reg_11[i].value);
  	}
}

void ak4377_reg_init_12(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_12)/sizeof(ak4377_reg_12[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_12[i].reg, ak4377_reg_12[i].value);
  	}
}
void ak4377_reg_init_16(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_16)/sizeof(ak4377_reg_16[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_16[i].reg, ak4377_reg_16[i].value);
  	}
}

void ak4377_reg_init_22(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_22)/sizeof(ak4377_reg_22[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_22[i].reg, ak4377_reg_22[i].value);
  	}
}
void ak4377_reg_init_24(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_24)/sizeof(ak4377_reg_24[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_24[i].reg, ak4377_reg_24[i].value);
  	}
}
void ak4377_reg_init_32(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_32)/sizeof(ak4377_reg_32[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_32[i].reg, ak4377_reg_32[i].value);
  	}
}
void ak4377_reg_init_64(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_64)/sizeof(ak4377_reg_64[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_64[i].reg, ak4377_reg_64[i].value);
  	}
}
void ak4377_reg_init_88(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_88)/sizeof(ak4377_reg_88[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_88[i].reg, ak4377_reg_88[i].value);
  	}
}
void ak4377_reg_init_96(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_96)/sizeof(ak4377_reg_96[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_96[i].reg, ak4377_reg_96[i].value);
  	}
}
void ak4377_reg_init_128(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_128)/sizeof(ak4377_reg_128[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_128[i].reg, ak4377_reg_128[i].value);
  	}
}

void ak4377_reg_init_256(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_256)/sizeof(ak4377_reg_256[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_256[i].reg, ak4377_reg_256[i].value);
  	}
}

void ak4377_reg_init_352(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_352)/sizeof(ak4377_reg_352[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_352[i].reg, ak4377_reg_352[i].value);
  	}
}

void ak4377_reg_init_512(void)
{
	int i;
	for(i=0; i<sizeof(ak4377_reg_512)/sizeof(ak4377_reg_512[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_512[i].reg, ak4377_reg_512[i].value);
  	}
}

void ak4377_reg_init_384(void)
{
	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_defaults_384)/sizeof(ak4377_reg_defaults_384[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_384[i].reg, ak4377_reg_defaults_384[i].value);
  	}

}

void ak4377_reg_init_192(void)
{
	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_defaults_192)/sizeof(ak4377_reg_defaults_192[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_192[i].reg, ak4377_reg_defaults_192[i].value);
  	}

}
void ak4377_reg_init_176(void)
{
	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_defaults_176)/sizeof(ak4377_reg_defaults_176[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_176[i].reg, ak4377_reg_defaults_176[i].value);
  	}

}
void ak4377_reg_init_48(void)
{
	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_defaults_48)/sizeof(ak4377_reg_defaults_48[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_48[i].reg, ak4377_reg_defaults_48[i].value);
  	}

}
void ak4377_reg_init_44(void)
{
	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_defaults_44)/sizeof(ak4377_reg_defaults_44[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_44[i].reg, ak4377_reg_defaults_44[i].value);
  	}

}
//////////////////////////////////////low power mode/////////////////////////
void ak4377_reg_init_44_low(void) {
	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_defaults_44_low)/sizeof(ak4377_reg_defaults_44_low[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_44_low[i].reg, ak4377_reg_defaults_44_low[i].value);
  	}
}
void ak4377_reg_init_8_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_8_low)/sizeof(ak4377_reg_8_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_8_low[i].reg, ak4377_reg_8_low[i].value);
    }
}

void ak4377_reg_init_11_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_11_low)/sizeof(ak4377_reg_11_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_11_low[i].reg, ak4377_reg_11_low[i].value);
    }
}

void ak4377_reg_init_12_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_12_low)/sizeof(ak4377_reg_12_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_12_low[i].reg, ak4377_reg_12_low[i].value);
    }
}
void ak4377_reg_init_16_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_16_low)/sizeof(ak4377_reg_16_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_16_low[i].reg, ak4377_reg_16[i].value);
    }
}

void ak4377_reg_init_22_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_22_low)/sizeof(ak4377_reg_22_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_22_low[i].reg, ak4377_reg_22_low[i].value);
    }
}
void ak4377_reg_init_24_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_24_low)/sizeof(ak4377_reg_24_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_24_low[i].reg, ak4377_reg_24_low[i].value);
    }
}
void ak4377_reg_init_32_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_32_low)/sizeof(ak4377_reg_32_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_32_low[i].reg, ak4377_reg_32_low[i].value);
    }
}
void ak4377_reg_init_64_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_64_low)/sizeof(ak4377_reg_64_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_64_low[i].reg, ak4377_reg_64_low[i].value);
    }
}
void ak4377_reg_init_88_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_88_low)/sizeof(ak4377_reg_88_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_88_low[i].reg, ak4377_reg_88_low[i].value);
    }
}
void ak4377_reg_init_96_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_96_low)/sizeof(ak4377_reg_96_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_96_low[i].reg, ak4377_reg_96_low[i].value);
    }
}
void ak4377_reg_init_128_low(void)
{
    int i;
    for(i=0; i<sizeof(ak4377_reg_128_low)/sizeof(ak4377_reg_128_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_128_low[i].reg, ak4377_reg_128_low[i].value);
    }
}

void ak4377_reg_init_192_low(void)
{
    fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
    int i;
    for(i=0; i<sizeof(ak4377_reg_defaults_192_low)/sizeof(ak4377_reg_defaults_192_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_192_low[i].reg, ak4377_reg_defaults_192_low[i].value);
    }

}
void ak4377_reg_init_176_low(void)
{
    fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
    int i;
    for(i=0; i<sizeof(ak4377_reg_defaults_176_low)/sizeof(ak4377_reg_defaults_176_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_176_low[i].reg, ak4377_reg_defaults_176_low[i].value);
    }

}
void ak4377_reg_init_48_low(void)
{
    fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
    int i;
    for(i=0; i<sizeof(ak4377_reg_defaults_48_low)/sizeof(ak4377_reg_defaults_48_low[0]); i++)
    {
        AK4377_WriteOneByte_FIIO(ak4377_reg_defaults_48_low[i].reg, ak4377_reg_defaults_48_low[i].value);
    }

}
/////////////////////////////////////////////////////////////
void ak4377_reg_init_low_power(void)
{

	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,"low power");
	int i;
	for(i=0; i<sizeof(ak4377_low_defaults)/sizeof(ak4377_low_defaults[0]); i++)
	{
		if (ak4377_reg_defaults[i].reg == 0x03) {
			
		}
		else {
			if (ak4377_low_defaults[i].reg == 0x0B || ak4377_low_defaults[i].reg == 0x0C) {
				AK4377_WriteOneByte_Init(ak4377_low_defaults[i].reg, ak4377_low_defaults[i].value);
			}
			else {
				AK4377_WriteOneByte_FIIO(ak4377_low_defaults[i].reg, ak4377_low_defaults[i].value);
			}
		}
		
		
  	}

}


void ak4377_reg_init(void)
{

	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,(FIIO_AK4377_LOW_POWER_MODE == get_fiio_ak4377_work_mode())?"low power":"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_defaults)/sizeof(ak4377_reg_defaults[0]); i++)
	{
		if (ak4377_reg_defaults[i].reg == 0x03) {
			
		}
		else {
			if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
				if (ak4377_reg_defaults[i].reg == 0x0B || ak4377_reg_defaults[i].reg == 0x0C) {
						AK4377_WriteOneByte_Init(ak4377_reg_defaults[i].reg, ak4377_reg_defaults[i].value);
					}
					else {
						AK4377_WriteOneByte_FIIO(ak4377_reg_defaults[i].reg, ak4377_reg_defaults[i].value);
					}
			 }
			 else {
				if (ak4377_low_defaults[i].reg == 0x0B || ak4377_low_defaults[i].reg == 0x0C) {
					AK4377_WriteOneByte_Init(ak4377_low_defaults[i].reg, ak4377_low_defaults[i].value);
				}
				else {
					AK4377_WriteOneByte_FIIO(ak4377_low_defaults[i].reg, ak4377_low_defaults[i].value);
				}
			 }
		}
		 
  	}

}

//DSD64
void ak4377_init_dsd_28224(void)
{

	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_dsd_256_28224_1764)/sizeof(ak4377_reg_dsd_256_28224_1764[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_dsd_256_28224_1764[i].reg, ak4377_reg_dsd_256_28224_1764[i].value);
  	}

}

//DSD128
void ak4377_init_dsd_56448(void)
{

	fiio_debug("%s %d ak4377 mode:%s\n",__func__,__LINE__,"high performance");
	int i;
	for(i=0; i<sizeof(ak4377_reg_dsd_256_56448_3528)/sizeof(ak4377_reg_dsd_256_56448_3528[0]); i++)
	{
		AK4377_WriteOneByte_FIIO(ak4377_reg_dsd_256_56448_3528[i].reg, ak4377_reg_dsd_256_56448_3528[i].value);
  	}

}



static int  ak4377_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{

	//struct axp173_platform_data *pdata = i2c->dev.platform_data;
	int ret;
	printk("enter ak4377_i2c_probe\n");
	//ak4377_i2c_gpio_init();
	//ak4377_reg_init();
	g_ak4377 = kzalloc(sizeof(struct ak4377),GFP_ATOMIC);//GFP_KERNEL);//
    if (g_ak4377 == NULL) {
        ret = -ENOMEM;
        goto err;
    }

	g_ak4377->i2c = i2c;
	g_ak4377->dev = &i2c->dev;
	i2c_set_clientdata(i2c, g_ak4377);
	
	ak4377_cache_reg_init();
	printk("ak4377 reg15=%d\n",ak4377_i2c_read(g_ak4377,0x15));
	ret=ak4377_i2c_read_bit(g_ak4377,0x01,5);
	if ((ret < 0) || (ret == 0xff)){
		printk(KERN_ERR"The device is not act:ak4377 \n");
		return 0;
	}
   	
	return 0;

err:
	return ret;	

}


static ssize_t test_eq_show(struct device *device, struct device_attribute *attr, char *buf)
{

	return 0;
}

static ssize_t test_eq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	return count;
}
static ssize_t reg_show(struct device *device, struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int event = simple_strtoul(buf, NULL, 0);
	switch(event) {
		case 0:
			printk("0:help\n");
			printk("1:mute hpg\n");
			printk("2:umute hpg\n");
			printk("3:dac 0dB\n");
			printk("4:dac mute\n");
			break;
		case 1:
			m5_hpg_mute();
			break;
		case 2:
			m5_hpg_unmute();
			break;
		case 3:
			m5_dac_volume();
			break;
		case 4:
			m5_dac_mute();
			break;
	}
	
	return count;
}			
static struct device_attribute ak4377_device_attrs[] = {
	__ATTR(reg, S_IRUGO | S_IWUSR, reg_show, reg_store),
		
	__ATTR(test_eq, S_IRUGO | S_IWUSR, test_eq_show, test_eq_store),
};

static int ak4377_device_attr_register(struct miscdevice  *ak4377_misc_opt)
{
	int error = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ak4377_device_attrs); i++) {
		error = device_create_file(ak4377_misc_opt->this_device, &ak4377_device_attrs[i]);

		if (error)
			break;
	}

	if (error) {
		while (--i >= 0)
			device_remove_file(ak4377_misc_opt->this_device, &ak4377_device_attrs[i]);
	}

	return 0;
}

static int ak4377_device_attr_unregister(struct miscdevice  *ak4377_misc_opt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ak4377_device_attrs); i++)
		device_remove_file(ak4377_misc_opt->this_device, &ak4377_device_attrs[i]);

	return 0;
}
static int ak4377_open(struct inode *inode, struct file *file)
{
	return 0;
}
static int ak4377_release(struct inode *inode, struct file *file)
{
	return 0;
}
 
void fiio_test_ak4377() {
       gpio_direction_output(PO_EN, 1);
       mdelay(500);
       AK4377_WriteOneByte(0x0C,0xAA);
}
EXPORT_SYMBOL(fiio_test_ak4377);

void fiio_ak4377_0db() {
	AK4377_WriteOneByte(0x0C,0x39);
	AK4377_WriteOneByte(0x0B,0x39);
}
EXPORT_SYMBOL(fiio_ak4377_0db);

void fiio_ak4377_poweron_low_power() {
	fiio_debug("try to power on ak4377 in low power mode\n");
    gpio_direction_output(PO_EN, 1);
    mdelay(500);
    ak4377_reg_init_low_power();
	//AK4377_WriteOneByte(0x0C,0x19);
	//AK4377_WriteOneByte(0x0B,0x19);

}
EXPORT_SYMBOL(fiio_ak4377_poweron_low_power);


void fiio_ak4377_poweron() {
	fiio_debug("try to power on ak4377\n");
    gpio_direction_output(PO_EN, 1);
    mdelay(500);
    ak4377_reg_init();
	//AK4377_WriteOneByte(0x0C,0x19);
	//AK4377_WriteOneByte(0x0B,0x19);

}
EXPORT_SYMBOL(fiio_ak4377_poweron);

void fiio_ak4377_powerup() {
	fiio_debug("try to power up ak4377\n");
    gpio_direction_output(PO_EN, 1);
}
EXPORT_SYMBOL(fiio_ak4377_powerup);

void fiio_ak4377_poweroff() {
	fiio_debug("try to power down ak4377\n");
    gpio_direction_output(PO_EN, 0);
}
EXPORT_SYMBOL(fiio_ak4377_poweroff);

void fiio_ak4377_dsd_mode() {
	fiio_debug("try to start ak4377 DSD mode\n");
	AK4377_WriteOneByte(0x03,0xFC);
	AK4377_WriteOneByte(0x10,0x07);
    AK4377_WriteOneByte(0x15,0x60);
    AK4377_WriteOneByte(0x16,0x05);
 	AK4377_WriteOneByte(0x03,0x03);
}
EXPORT_SYMBOL(fiio_ak4377_dsd_mode);

void fiio_ak4377_pcm_mode() {
	fiio_debug("try to start ak4377 PCM mode\n");
	AK4377_WriteOneByte(0x03,0xFC);
    AK4377_WriteOneByte(0x10,0x00);
    AK4377_WriteOneByte(0x15,0x60);
    AK4377_WriteOneByte(0x16,0x01);
    AK4377_WriteOneByte(0x03,0x03);
}
EXPORT_SYMBOL(fiio_ak4377_pcm_mode);

void fiio_sys_set_fiio_dsd_mode(int dsd_type) {
	AK4377_WriteOneByte(0x03,0xFC);
	switch(dsd_type) {
		case 0:
			ak4377_init_dsd_28224();
			break;
		case 1:
			ak4377_init_dsd_56448();
			break;
		default:
			ak4377_init_dsd_28224();
			break;
	}
	AK4377_WriteOneByte(0x03,0x03);
}
EXPORT_SYMBOL(fiio_sys_set_fiio_dsd_mode);


void ak4377_sample_set(int sample,int power_flag);
void set_fiio_ak4377_work_mode(int mode) {
	g_fiio_ak4377_work_mode = mode;
	AK4377_WriteOneByte(0x03,0xFC);
    if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == mode) {
        //high performance
        ak4377_reg_init();
    }
    else {
        //low power
        ak4377_reg_init_low_power();
    }
    //根据当前频率设置DAC
    ak4377_sample_set(get_fiio_m5_play_sample(),0);
	//set volume
	//AK4377_WriteOneByte(0x0C,0x39);
    //AK4377_WriteOneByte(0x0B,0x39);
    fiio_set_dac_volume();
	AK4377_WriteOneByte(0x03,0x03);
}
EXPORT_SYMBOL(set_fiio_ak4377_work_mode);

int get_fiio_ak4377_work_mode() {
	return g_fiio_ak4377_work_mode;
}

static void set_ak4377_high_256k_sample() {
	//02H LPMODE 0:high performance default 0 1:Low Power Mode
	//if(!ak4377_power_mode) 
	//	AK4377_WriteOneByte(0x02,0x11);
	//else
		//AK4377_WriteOneByte(0x02,0x01);
	//24H 
	//AK4377_WriteOneByte(0x24,0x00);
}
static void set_ak4377_low_256k_sample() {
	#if 0
	//02H LPMODE
	AK4377_WriteOneByte(0x02,0x11);
	//24H >128fs
	AK4377_WriteOneByte(0x24,0x40);
	#else
	// if(!ak4377_power_mode) 
	// 	AK4377_WriteOneByte(0x02,0x11);
	// else
	// 	AK4377_WriteOneByte(0x02,0x01);
	//24H 64fs
	//AK4377_WriteOneByte(0x24,0x00);
	#endif
}

/*
 *设置DSD输出格式
 *0:D2P  1:DOP
 *default:0 D2P
 */
static int g_fiio_dsd_output_format = 0;
int fiio_get_dsd_output_format(void) {
    return g_fiio_dsd_output_format;
}
EXPORT_SYMBOL(fiio_get_dsd_output_format);

static int g_mute_ak4377_flag = 0;
static int g_last_ak4377_volume = 0;
static int g_volume_delay_time =  5;
void fiio_set_dac_volume();
void fiio_set_dsd_output_format(int format) {

    int sample = get_fiio_m5_play_sample();
	fiio_debug("fiio_set_dsd_output_format==========sample=%d cur format=%d last format=%d\n"
		,sample,format,g_fiio_dsd_output_format);
	//&& g_fiio_dsd_output_format==1
	if (format != g_fiio_dsd_output_format  && (sample==16||sample==13)) {
		g_fiio_dsd_output_format = format;
		g_mute_ak4377_flag = 1;
		#if 1
		AK4377_WriteOneByte(0x03,0xFC);
		//mute dop->d2p
		if (1 == g_mute_ak4377_flag) {
			AK4377_WriteOneByte_Volume(0x0B,0x00);
			AK4377_WriteOneByte_Volume(0x0C,0x00);
			AK4377_WriteOneByte_Volume(0x0D,0x00);
		}
		
		
	    //直接初始化DAC
	    if (1 == format) {
	        //DSD
	        if (16 == sample) {
	            ak4377_init_dsd_56448();
	        }
	        else {
	            ak4377_init_dsd_28224();
	        }

	    }
	    else {

	        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
	            //high performance
	            ak4377_reg_init();
	        }
	        else {
	            //low power
	            ak4377_reg_init_low_power();
	        }
	        //根据当前频率设置DAC	
	        ak4377_sample_set(sample,0);
	    }
		if (1 == g_mute_ak4377_flag) {
			//if (sample==16) {
				// 1s
			//	g_volume_delay_time = 10;
			//}
			AK4377_WriteOneByte_Volume(0x0B,0x01);
			AK4377_WriteOneByte_Volume(0x0C,0x01);
			AK4377_WriteOneByte_Volume(0x0D,0x01);
			fiio_set_dac_volume();
			g_mute_ak4377_flag = 0;
		}
		AK4377_WriteOneByte(0x03,0x03);
		#endif
	}
	else {
	
		g_fiio_dsd_output_format = format;
		fiio_set_dac_volume();
	}

	
}
EXPORT_SYMBOL(fiio_set_dsd_output_format);

void fiio_set_dsd_volume(unsigned char m_volume) {
	int m_val = 0;
	//printk("%s ------------m_volume=%d\n",__func__,m_volume);
	#if 0
	if (m_volume == 0) {
			//printk("%s ------00------m_volume=%d[0x%02x]\n",__func__,m_volume,m_volume);
			//mute
			AK4377_WriteOneByte_Volume(0x03,0x3F);
			AK4377_WriteOneByte_Volume(0x0B,0x00);
			AK4377_WriteOneByte_Volume(0x0C,0x00);
			AK4377_WriteOneByte_Volume(0x0D,0x00);
			AK4377_WriteOneByte_Volume(0x03,0x03);
	}
	else {
		// HPTM 2dB step -20~+2dB 
		// OVOLCN -28dB
		//-48dB~
		if (m_volume < 12) {// 1-11(-18dB...+2dB)(+2 +2 +2 +2 +2 +2 +2 +2 +2 +2 +2)
			// 1-11 -20 -18 -16 -14 -12 -10 -8 -6 -4 -2 0(BH)
			//printk("%s ----0-11--------m_volume=%d[0x%02x]\n",__func__,m_volume,m_volume);
			AK4377_WriteOneByte_Volume(0x0B,0x01);//-28dB
			AK4377_WriteOneByte_Volume(0x0C,0x01);
			AK4377_WriteOneByte_Volume(0x0D,m_volume);//MAX:0x0B
		}
		else {
			// 12-60
			// -28dB~+1dB
			// 0.5dB step
			// 2:-27.5dB(12)
			// 60(+1db) 59(+0.5dB) 58(0dB) 57(-0.5db) 56(-1dB)...36(-11dB) 16(-21dB) 15(-21.5dB) 14(-22dB) 13(-24) 12(-26)
			//60(+1db) 59(+0.5dB) 58(0dB) 57(-0.5db) 56(-1dB)...36(-11dB) 26(-16dB) 25(-16.5dB) 24(-17dB) 23(-17.5dB) 22(-18dB)
			// 21(-18.5) 20(-19) 19(-20) 18(-21) 17(-22) 16(-23) 15(-24) 14(-25) 13(-26) 12(-27)
			if (m_volume < 21) {
				// 12-20 
				m_val = 0x01 + (m_volume-11)*2;
				//printk("%s -----12-20-------m_volume=%d[0x%02x] m_val=%d[0x%02x]\n",__func__,m_volume,m_volume,m_val,m_val);
				AK4377_WriteOneByte_Volume(0x0B,m_val);
				AK4377_WriteOneByte_Volume(0x0C,m_val);
			}
			else {
				// 21-60
				m_val = 0x13 + (m_volume-20);
				//printk("%s ----21-60--------m_volume=%d[0x%02x] m_val=%d[0x%02x]\n",__func__,m_volume,m_volume,m_val,m_val);
				AK4377_WriteOneByte_Volume(0x0B,m_val);
				AK4377_WriteOneByte_Volume(0x0C,m_val);
			}
			AK4377_WriteOneByte_Volume(0x0D,0x0C);//+1dB
		}
	}
	#endif

	if (m_volume >= 42) {
		//60 59 58  57 56  55  54 53  52 51 50 49 48 47 46 45 44 43 42
		//0  1  2   3  4  5   6  7   8  9  10 11 12 13 14 15 16 17 18
		//39 38 37 36 35  34  33 32  31 30 2F 2E 2D 2C 2B 2A 29 28 27H
		m_val = 0x39-(60-m_volume);
		AK4377_WriteOneByte_Volume(0x0B,m_val); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0C,m_val); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0D,0x0D);//+2dB Headphone Amplifier Volume
		//printk("%s ------------m_volume=%d [0B0C]=%02x [0D]=%02x\n",__func__,m_volume,m_val,0x0D);
	}
	else if (m_volume >= 25) {
		//41 40 39 38 37 36 35 34 33 32 31 30 29 28 27 26 25
	   //25  23 21 1F 1D 1B 19 17 15 13 11 0F 0D 0B 09 07 05 
		m_val = 0x25-(41-m_volume)*2;
		AK4377_WriteOneByte_Volume(0x0B,m_val); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0C,m_val); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0D,0x0D);//+2dB Headphone Amplifier Volume
		//printk("%s ------------m_volume=%d [0B0C]=%02x [0D]=%02x\n",__func__,m_volume,m_val,0x0D);
	}
	else if (m_volume >= 14) {
		//24 23 22 21 20 19 18 17 16 15 14
		//0B 0A 09 08 07 06 05 04 03 02 01
		m_val = 0x0B-(24-m_volume);
		AK4377_WriteOneByte_Volume(0x0B,0x05); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0C,0x05); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0D,m_val);//+2dB Headphone Amplifier Volume
		//printk("%s ------------m_volume=%d [0B0C]=%02x [0D]=%02x\n",__func__,m_volume,0x05,m_val);
	}
	else if (m_volume == 13) {
		AK4377_WriteOneByte_Volume(0x0B,0x01); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0C,0x01); //Digital Volume
		AK4377_WriteOneByte_Volume(0x0D,0x01);//+2dB Headphone Amplifier Volume
		//printk("%s ------------m_volume=%d [0B0C]=%02x [0D]=%02x\n",__func__,m_volume,0x01,0x01);
	}
	else {
		AK4377_WriteOneByte_Volume(0x03,0x3F);
		AK4377_WriteOneByte_Volume(0x0B,0x00);
		AK4377_WriteOneByte_Volume(0x0C,0x00);
		AK4377_WriteOneByte_Volume(0x0D,0x00);
		AK4377_WriteOneByte_Volume(0x03,0x03);
		//printk("%s ------------m_volume=%d [0B0C]=%02x [0D]=%02x  MUTE\n",__func__,m_volume,0x00,0x00);
	}
	
}
EXPORT_SYMBOL(fiio_set_dsd_volume);


//#define AUDIO_LINK_SHUTDOWN_DELAY (0.5)

void fiio_set_dac_volume(void) {
	fiio_debug("*****************%s dsd_volume=%d dop_or_d2p=%d"
		,__func__,dsd_volume,fiio_get_dsd_output_format());
	//dsd
	if (1 == fiio_get_dsd_output_format()) {
		fiio_set_dsd_volume(dsd_volume);
	}	
	else {
		//0dp 0dp
		//AK4377_WriteOneByte_Volume(0x0B,0x39);
		//AK4377_WriteOneByte_Volume(0x0C,0x39);
		//AK4377_WriteOneByte_Volume(0x0D,0x0B);
		int ret = 0;
		int start_run = 1;
		ret=work_busy((struct work_struct*)&ak4377_delay_work);
		if(ret&WORK_BUSY_PENDING) {
			//printk(KERN_WARNING "ak4377_delay_work work_func WORK_BUSY_PENDING cancel it\n");
			cancel_delayed_work_sync(&ak4377_delay_work);
			start_run = 0;
		}
		if(ret&WORK_BUSY_RUNNING) {

			//printk("ak4377_delay_work work_func WORK_BUSY_RUNNING wait it to finish\n");
			flush_workqueue(ak4377_delay_work.wq);
			start_run = 0;
		}
		if (1 == start_run) {
			//printk(">>>>>>>>>fiio_set_dac_volume g_volume_delay_time=%d\n",g_volume_delay_time);
			schedule_delayed_work(&ak4377_delay_work,msecs_to_jiffies(g_volume_delay_time * 100));
		}
			
		g_volume_delay_time = 5;
	}
}
EXPORT_SYMBOL(fiio_set_dac_volume);

extern unsigned int get_headset_insert_state(void);
static int g_ak4377_init_flag = 0;
void ak4377_sample_set(int sample,int power_flag)
{
	//power up
	#if 1
	if (1==get_headset_insert_state() && 1==g_fiio_ak4377_is_power_flag) {
		//check power up
		//printk("begin=============================%s want to init ak4377!\n",__func__);
		if (0 == g_ak4377_init_flag) {
			g_ak4377_init_flag = 1;
			if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
	            //high performance
	            ak4377_reg_init();
	        }
	        else {
	            //low power
	            ak4377_reg_init_low_power();
	        }
		}
		//printk("end=============================%s want to init ak4377!\n",__func__);
	}
	#endif
	if (1 == power_flag)
		AK4377_WriteOneByte(0x03,0xFC);
	switch(sample){
	case 1://8000
		#if 0
		set_ak4377_low_256k_sample();
		AK4377_WriteOneByte(0x05,0x20);
        AK4377_WriteOneByte(0x10,0x00);
        AK4377_WriteOneByte(0x12,0xEF);
        AK4377_WriteOneByte(0x14,0x1D);
		#endif

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_8();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_8_low();
        }
	break;
	case 2://11025
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x21);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x9F);
        // AK4377_WriteOneByte(0x14,0x13);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_11();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_11_low();
        }
	break;
	case 3://12000
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x2E);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x9F);
        // AK4377_WriteOneByte(0x14,0x13);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_12();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_12_low();
        }
	break;
	case 4://16000
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x04);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x77);
        // AK4377_WriteOneByte(0x14,0x0E);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_16();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_16_low();
        }
	break;
	case 5://22050
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x05);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x4F);
        // AK4377_WriteOneByte(0x14,0x09);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_22();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_22_low();
        }
	break;
	case 6://24000
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x06);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x4F);
        // AK4377_WriteOneByte(0x14,0x09);	

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_24();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_24_low();
        }
	break;
	case 7://32000
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x08);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x3B);
        // AK4377_WriteOneByte(0x14,0x0E);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_32();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_32_low();
        }
	break;
	case 8://44100
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x09);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x27);
        // AK4377_WriteOneByte(0x14,0x09);
        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
			fiio_debug("ak4377 work in high performance mode!\n");
			ak4377_reg_init_44();
		}
		else {
			fiio_debug("ak4377 work in low power mode!\n");
			ak4377_reg_init_44_low();
		}
		
	break;
	case 9://48000
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x0A);
        // AK4377_WriteOneByte(0x10,0x00);
        // AK4377_WriteOneByte(0x12,0x27);
        // AK4377_WriteOneByte(0x14,0x09);
		// AK4377_WriteOneByte(0x03,0x23);
        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_48();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_48_low();
        }



	break;
	case 10://64000
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x0C);
        // AK4377_WriteOneByte(0x10,0x01);
        // AK4377_WriteOneByte(0x12,0x27);
        // AK4377_WriteOneByte(0x14,0x04);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_64();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_64_low();
        }
	break;
	case 11://88200
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x0D);
        // AK4377_WriteOneByte(0x10,0x01);
		// AK4377_WriteOneByte(0x12,0x27);
		// AK4377_WriteOneByte(0x14,0x04);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_88();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_88_low();
        }
	break;
	case 12://96000
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x0E);
        // AK4377_WriteOneByte(0x10,0x01);
		// AK4377_WriteOneByte(0x12,0x27);
		// AK4377_WriteOneByte(0x14,0x04);

        if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
            fiio_debug("ak4377 work in high performance mode!\n");
            ak4377_reg_init_96();
        }
        else {
            fiio_debug("ak4377 work in low power mode!\n");
            ak4377_reg_init_96_low();
        }
	break;
	case 13://176400 256fs
		// set_ak4377_high_256k_sample();
		// AK4377_WriteOneByte(0x05,0x11);
		// AK4377_WriteOneByte(0x10,0x03);
		// AK4377_WriteOneByte(0x12,0x27);
		// AK4377_WriteOneByte(0x14,0x04);
		#ifdef CONFIG_FIIO_IO_CTRL
        if (0 == fiio_get_dsd_output_format()) {
            ak4377_reg_init_176();
        }

		else
		#endif
			ak4377_init_dsd_28224();
	break;
	case 14: //192K 256fs

		// set_ak4377_high_256k_sample();
		// //AK4377_WriteOneByte(0x06,0xC0);
		// AK4377_WriteOneByte(0x05,0x12);
		// //DAC分频
		// AK4377_WriteOneByte(0x10,0x03);//PLD=3 BCLK=64fs BCLK/4=12.288MHz/4=3.072MHz=16fs
		// AK4377_WriteOneByte(0x12,0x27);//3.072*40=122.88MHz
		// //
		// AK4377_WriteOneByte(0x13,0x11); //2.5
		// AK4377_WriteOneByte(0x14,0x04);//DACMCLK=256fs  DACMCLK*2.5=122.88MHz


        ak4377_reg_init_192();
	
	break;
	case 15:
		//high performance mode 64fs
		//256K
		// set_ak4377_high_256k_sample();
		// //05H D6 D5 must set 0 1
		// AK4377_WriteOneByte(0x05,0x34);
		// AK4377_WriteOneByte(0x10,0x03);
		// AK4377_WriteOneByte(0x12,0x27);
		// AK4377_WriteOneByte(0x14,0x04);
		ak4377_reg_init_256();
	break;
	case 16:
		//high performance mode 64fs
		//352800
		// set_ak4377_high_256k_sample();
		// //05H D6 D5 must set 0 1
		// AK4377_WriteOneByte(0x05,0x35);
		// AK4377_WriteOneByte(0x10,0x03);
		// AK4377_WriteOneByte(0x12,0x27);
		// AK4377_WriteOneByte(0x14,0x04);
		#ifdef CONFIG_FIIO_IO_CTRL
		if (0 == fiio_get_dsd_output_format())
			ak4377_reg_init_352();
		else
		#endif
			ak4377_init_dsd_56448();
	break;
	case 17: //384K
		//high performance mode 64fs
		// set_ak4377_high_256k_sample();
		// //05H D6 D5 must set 0 1
		// AK4377_WriteOneByte(0x05,0x36);
		// AK4377_WriteOneByte(0x10,0x03);
		// AK4377_WriteOneByte(0x12,0x27);
		// AK4377_WriteOneByte(0x14,0x04);
		ak4377_reg_init_384();
	break;
	default:
	break;
	}
	// AK4377_WriteOneByte(0x00,0x01);
	// AK4377_WriteOneByte(0x01,0x73);
	// AK4377_WriteOneByte(0x02,0x01);
	if (1 == power_flag)
		AK4377_WriteOneByte(0x03,0x03);
}
EXPORT_SYMBOL(ak4377_sample_set);




static int ak4377_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
//	 int err = 0;
    int ret = 0;
   // int ioarg = 0;
 //   int reg,bit,sign;
   // struct jz_gpio_func_def *g1 = &platform_lcd_array[0];
    
    printk("enter  ak4377_ioctl\n");
	    /* 检测命令的有效性 */
 //   if (_IOC_TYPE(cmd) != MEMDEV_IOC_MAGIC) 
   //  {printk(" ak4377  cmd error1\n");
        //return -EINVAL;
     //   }
   //  if (_IOC_NR(cmd) > MEMDEV_IOC_MAXNR) 
   //        {  printk(" ak4377  cmd error2\n");
        //return -EINVAL;
//	}
	 /* 根据命令，执行相应的操作 */
    switch(cmd) {

      	/* 打印当前设备信息 */
    	//  case MEMDEV_IOCPRINT:
      
      	//  break;
      	/* 获取参数 */
      	case AK4376_POWER_ON:
			
			printk("try to power on ak4377 work_sign=%d\n",work_sign);
			
			gpio_direction_output(PO_EN, 1);
			g_fiio_ak4377_is_power_flag = 1;
			mdelay(5);
			if (1 == work_sign) {
				#if 0
				ak4377_reg_init();
				#else
				AK4377_WriteOneByte(0x03,0xFC);
				AK4377_Mute();
			    if (FIIO_AK4377_HIGH_PERFORMANCE_MODE == get_fiio_ak4377_work_mode()) {
		            //high performance
		            ak4377_reg_init();
		        }
		        else {
		            //low power
		            ak4377_reg_init_low_power();
		        }
				if (1 == fiio_get_dsd_output_format()) {
			        //DSD
			        if (16 == get_fiio_m5_play_sample()) {
			            ak4377_init_dsd_56448();
			        }
			        else {
			            ak4377_init_dsd_28224();
			        }
					
			    }
			    else {
			        ak4377_sample_set(get_fiio_m5_play_sample(),0);
			    }
				if(work_sign) {
					fiio_debug("power on ak4377 and set volume!");
					//AK4377_WriteOneByte(0x0C,0x39);
					//AK4377_WriteOneByte(0x0B,0x39);
					fiio_set_dac_volume();
				}
				else {
					fiio_debug("power on ak4377 ,but don't set volume!");
				}
				
				AK4377_WriteOneByte(0x03,0x03);
				
				#endif
				#if 0
				if(work_sign)
				{
					AK4377_WriteOneByte(0x0C,volumeR);
					AK4377_WriteOneByte(0x0B,volumeL);
				}
				#endif
			}
			
			
			break;
      
      	/* 设置参数 */
      	case AK4376_POWER_DOWN: 
			printk("try to power down ak4377\n");
			gpio_direction_output(PO_EN, 0);
			g_fiio_ak4377_is_power_flag = 0;
			g_ak4377_init_flag = 0;
			break;

      	case AK4377_DSD_MODE:
			printk("try to start ak4377 DSD mode\n");
			AK4377_WriteOneByte(0x03,0xFC);
			AK4377_WriteOneByte(0x10,0x07);
			AK4377_WriteOneByte(0x15,0x60);
			AK4377_WriteOneByte(0x16,0x05);
			AK4377_WriteOneByte(0x03,0x03);
			break;
		case AK4377_PCM_MODE:
			printk("try to start ak4377 pcm mode\n");
			AK4377_WriteOneByte(0x03,0xFC);
			AK4377_WriteOneByte(0x10,0x00);
			AK4377_WriteOneByte(0x15,0x60);
			AK4377_WriteOneByte(0x16,0x01);
			AK4377_WriteOneByte(0x03,0x03);
			break;

        default:  
        return -EINVAL;
    }
    return ret;
}

static struct file_operations ak4377_opt_fops = {
	.owner	=	THIS_MODULE,
	.open	=	ak4377_open,
	.unlocked_ioctl		=	ak4377_ioctl,
	.release	= 	ak4377_release,
};

static struct miscdevice ak4377_misc_opt = {
	.minor	=	MISC_DYNAMIC_MINOR,
	.name	= 	(char *)driver_name,
	.fops	=	&ak4377_opt_fops,
};
static int ak4377_i2c_remove(struct i2c_client *i2c)
{
	struct ak4377 *ak4377 = i2c_get_clientdata(i2c);
	i2c_set_clientdata(i2c, NULL);
	kfree(ak4377);
	return 0;
}
static int ak4377_suspend(struct i2c_client *i2c)
{
	printk("PM:ak4377_suspend,try to close dac\n");
	gpio_direction_output(PO_EN, 0);
	return 0;
}

static int ak4377_resume(struct i2c_client *i2c)
{
	printk("PM:ak4377_resume,try to start dac\n");
    //gpio_direction_output(PO_EN, 1);
	//mdelay(1);
    //ak4377_cache_reg_init();
    return 0;
}

static const struct i2c_device_id ak4377_i2c_id[] = {
       { "ak4377", 0 },
       { }
};

MODULE_DEVICE_TABLE(i2c, ak4377_i2c_id);

static struct i2c_driver ak4377_i2c_driver = {
	.driver = {
		.name = "ak4377",
		.owner = THIS_MODULE,
	},
	.probe    = ak4377_i2c_probe,
	.remove   = ak4377_i2c_remove,
	.suspend    = ak4377_suspend,
        .resume     = ak4377_resume,
	.id_table = ak4377_i2c_id,
};
//module_i2c_driver(axp173_i2c_driver);




static int __init ak4377_module_init(void)
{
	int ret, retval;
	printk(KERN_INFO"enter %s,line:%d\n",__func__,__LINE__);
/*	if(PO_EN > 0){
        ret = gpio_request(PO_EN, "PO_EN");
        if (ret) {
            printk(KERN_ERR "can's request PO_EN\n");
           return ret;
        }
        gpio_direction_output(PO_EN, 1);
        //gpio_direction_output(PO_EN, 0);
        }
*/	
	ak4377_i2c_gpio_init();
	//ak4377_reg_init();
	#ifndef FIIO_AK4377_USB_IO_I2C
		ret = i2c_add_driver(&ak4377_i2c_driver);
		if (ret != 0)
			printk(KERN_INFO"Failed to register AK4377 I2C driver: %d\n", ret);
		else {
			printk("Successfully added driver %s\n",
			ak4377_i2c_driver.driver.name);
		}
	#endif
	
				
	 retval = misc_register(&ak4377_misc_opt);
	if (retval < 0)
	{
		printk(KERN_ERR"register misc device opt failed.\n");
		return retval;
	}
	ak4377_device_attr_register(&ak4377_misc_opt);

    return ret;
}
module_init(ak4377_module_init);

static void __exit ak4377_module_exit(void)
{
	gpio_free(PO_EN);
	#ifndef FIIO_AK4377_USB_IO_I2C
	i2c_del_driver(&ak4377_i2c_driver);
	#endif
	ak4377_device_attr_unregister(&ak4377_misc_opt);
}
module_exit(ak4377_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("pengweizhong@fiio.net");
MODULE_DESCRIPTION("AK4377 DAC driver");
