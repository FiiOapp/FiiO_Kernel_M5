#ifndef __LINUX_ST1615_TS_H__
#define __LINUX_ST1615_TS_H__

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	1
#define MAX_AREA	0xff

#define ST1615_NAME 	"st1615_ts"

/*register address*/
#define FT6x06_REG_FW_VER		0xA6


/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct st1615_platform_data {
	unsigned int x_min;
	unsigned int y_min;
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int int_gpio;
	unsigned int reset_gpio;
};
#endif
