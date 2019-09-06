/*
 * stk832x.c - Linux driver for sensortek stk832x accelerometer
 * Copyright (C) 2017 Sensortek
 */

#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>
#ifdef CONFIG_OF
    #include <linux/of_gpio.h>
#endif
#include <../../arch/mips/xburst/soc-x1000/chip-x1000/halley2/halley2_v10/board.h>

#define FIIO_SUSPEND_STEP_RUNNING
//#define FIIO_DEBUG_SENSOR
#ifdef FIIO_DEBUG_SENSOR
#define fiio_debug(x...)  printk(KERN_INFO "[stk832x] " x)
#else
#define fiio_debug(x...)
#endif

//#include <linux/stk832x.h>

/****************** Global variables ******************/
/**
  * enalbe INTERRUPT_MODE:
  *         FIFO, ANYMOTION, DATA use the same INT1.
  * disable INTERRUPT_MODE:
  *         DATA read via polling, ANYMOTION use INT1, FIFO use INT2.
  */
//#define INTERRUPT_MODE

/* Turn on step counter */
#define STK_STEP_COUNTER

/* enable check code feature */
//#define STK_CHECK_CODE
#ifdef STK_CHECK_CODE
    #include <linux/cc_stk832x.h>
    /* Ignore the first STK_CHECKCODE_IGNORE+1 data for STK_CHECK_CODE feature */
    #define STK_CHECKCODE_IGNORE    3
    /* for stk832x_data.cc_status */
    #define STK_CCSTATUS_NORMAL     0x0
    #define STK_CCSTATUS_ZSIM       0x1
    #define STK_CCSTATUS_XYSIM      0x2
#endif /* STK_CHECK_CODE */

/*
 * enable low-pass mode.
 * Define STK_FIR to turn ON low pass mode.
 */
#define STK_FIR
#ifdef STK_FIR
    #define STK_FIR_LEN         2
    #define STK_FIR_LEN_MAX     32
#endif /* STK_FIR */

#ifdef STK_FIR
struct data_fir
{
    s16 xyz[STK_FIR_LEN_MAX][3];
    int sum[3];
    int idx;
    int count;
};
#endif /* STK_FIR */

/*
 * enable Zero-G simulation.
 * This feature only works when both of STK_FIR and STK_ZG are turn ON.
 */
//#define STK_ZG
#if (defined STK_FIR && defined STK_ZG)
    #define ZG_FACTOR   0
#endif /* defined STK_FIR && defined STK_ZG */

#define STK_ACC_DRIVER_VERSION "0.0.2"
#define STK832X_I2C_NAME "stk832x"
#define STK832X_MISC_NAME "stk832x"
#define IN_DEV_ACCEL_NAME "accelerometer"
#define IN_DEV_ANY_NAME "any motion"
#define STK832X_IRQ_INT1_LABEL "STK_ACCEL_INT1"
#define STK832X_IRQ_INT1_NAME "stk832x_int1"
#ifndef INTERRUPT_MODE /* no INTERRUPT_MODE, polling mode */
    #define STK832X_IRQ_INT2_LABEL "STK_ACCEL_INT2"
    #define STK832X_IRQ_INT2_NAME "stk832x_int2"
#endif /* no INTERRUPT_MODE */

/* calibration parameters */
#define STK_CALI_SAMPLE_NO          10
#define STK_CALI_VER0               0x18
#define STK_CALI_VER1               0x03
#define STK_CALI_END                '\0'
#define STK_CALI_FILE               "/data/misc/sensor/stkacccali.conf"
#define STK_CALI_FILE_SIZE          25
/* parameter for cali_status/atomic_t and cali file */
#define STK_K_SUCCESS_FILE          0x01
/* parameter for cali_status/atomic_t */
#define STK_K_FAIL_WRITE_OFST       0xF2
#define STK_K_FAIL_I2C              0xF8
#define STK_K_FAIL_W_FILE           0xFB
#define STK_K_FAIL_VERIFY_CALI      0xFD
#define STK_K_RUNNING               0xFE
#define STK_K_NO_CALI               0xFF

/* stk832x register */
#define STK832X_REG_CHIPID          0x00
#define STK832X_REG_XOUT1           0x02
#define STK832X_REG_XOUT2           0x03
#define STK832X_REG_YOUT1           0x04
#define STK832X_REG_YOUT2           0x05
#define STK832X_REG_ZOUT1           0x06
#define STK832X_REG_ZOUT2           0x07
#define STK832X_REG_INTSTS1         0x09
#define STK832X_REG_INTSTS2         0x0A
#define STK832X_REG_STEPOUT1        0x0D
#define STK832X_REG_STEPOUT2        0x0E
#define STK832X_REG_RANGESEL        0x0F
#define STK832X_REG_BWSEL           0x10
#define STK832X_REG_POWMODE         0x11
#define STK832X_REG_SWRST           0x14
#define STK832X_REG_INTEN1          0x16
#define STK832X_REG_INTEN2          0x17
#define STK832X_REG_INTMAP1         0x19
#define STK832X_REG_INTMAP2         0x1A
#define STK832X_REG_INTCFG1         0x20
#define STK832X_REG_INTCFG2         0x21
#define STK832X_REG_SLOPEDLY        0x27
#define STK832X_REG_SLOPETHD        0x28
#define STK832X_REG_SIGMOT1         0x29
#define STK832X_REG_SIGMOT2         0x2A
#define STK832X_REG_SIGMOT3         0x2B
#define STK832X_REG_STEPCNT1        0x2C
#define STK832X_REG_STEPCNT2        0x2D
/*
 * default:0x32 correspond to 195.3mg 
 * 1 LSB=3.9mg
 */
#define STK832X_REG_STEPTHD         0x2E
#define STK832X_REG_STEPDEB         0x2F
#define STK832X_REG_STEPMAXTW       0x31
#define STK832X_REG_INTFCFG         0x34
#define STK832X_REG_OFSTCOMP1       0x36
#define STK832X_REG_OFSTX           0x38
#define STK832X_REG_OFSTY           0x39
#define STK832X_REG_OFSTZ           0x3A
#define STK832X_REG_CFG1            0x3D
#define STK832X_REG_CFG2            0x3E
#define STK832X_REG_FIFOOUT         0x3F

/* STK832X_REG_CHIPID */
#define STK8323_ID                          0x23 /* include for STK8321 */
#define STK8325_ID                          0x25

/* STK832X_REG_INTSTS1 */
#define STK832X_INTSTS1_SIG_MOT_STS         0x1
#define STK832X_INTSTS1_ANY_MOT_STS         0x4

/* STK832X_REG_INTSTS2 */
#define STK832X_INTSTS2_FWM_STS_MASK        0x40

/* STK832X_REG_RANGESEL */
#define STK832X_RANGESEL_2G                 0x3
#define STK832X_RANGESEL_4G                 0x5
#define STK832X_RANGESEL_8G                 0x8
#define STK832X_RANGESEL_BW_MASK            0xF
#define STK832X_RANGESEL_DEF                STK832X_RANGESEL_2G

/* STK832X_REG_BWSEL */
#define STK832X_BWSEL_INIT_ODR              0x0A    /* ODR = BW x 2 = 62.5Hz */
/* ODR: 31.25, 62.5, 125 */
const static int STK832X_SAMPLE_TIME[] = {32000, 16000, 8000}; /* usec */
#define STK832X_SPTIME_BASE                 0x9     /* for 32000, ODR:31.25 */
#define STK832X_SPTIME_BOUND                0xB     /* for 8000, ODR:125 */

/* STK832X_REG_POWMODE */
#define STK832X_PWMD_SUSPEND                0x80
#define STK832X_PWMD_LOWPOWER               0x40
#define STK832X_PWMD_NORMAL                 0x00
#define STK832X_PWMD_SLP_MASK               0x3E

/* STK832X_REG_SWRST */
#define STK832X_SWRST_VAL                   0xB6

/* STK832X_REG_INTEN1 */
#define STK832X_INTEN1_SLP_EN_XYZ           0x07

/* STK832X_REG_INTEN2 */
#define STK832X_INTEN2_DATA_EN              0x10
#define STK832X_INTEN2_FWM_EN               0x40

/* STK832X_REG_INTMAP1 */
#define STK832X_INTMAP1_SIGMOT2INT1         0x01
#define STK832X_INTMAP1_ANYMOT2INT1         0x04

/* STK832X_REG_INTMAP2 */
#define STK832X_INTMAP2_DATA2INT1           0x01
#define STK832X_INTMAP2_FWM2INT1            0x02
#define STK832X_INTMAP2_FWM2INT2            0x40

/* STK832X_REG_INTCFG1 */
#define STK832X_INTCFG1_INT1_ACTIVE_H       0x01
#define STK832X_INTCFG1_INT1_OD_PUSHPULL    0x00
#define STK832X_INTCFG1_INT2_ACTIVE_H       0x04
#define STK832X_INTCFG1_INT2_OD_PUSHPULL    0x00

/* STK832X_REG_INTCFG2 */
#define STK832X_INTCFG2_NOLATCHED           0x00
#define STK832X_INTCFG2_LATCHED             0x0F
#define STK832X_INTCFG2_INT_RST             0x80

/* STK832X_REG_SLOPETHD */
#define STK832X_SLOPETHD_DEF                0x14

/* STK832X_REG_SIGMOT1 */
#define STK832X_SIGMOT1_SKIP_TIME_3SEC      0x96    /* default value */

/* STK832X_REG_SIGMOT2 */
#define STK832X_SIGMOT2_SIG_MOT_EN          0x02
#define STK832X_SIGMOT2_ANY_MOT_EN          0x04

/* STK832X_REG_SIGMOT3 */
#define STK832X_SIGMOT3_PROOF_TIME_1SEC     0x32    /* default value */

/* STK832X_REG_STEPCNT2 */
#define STK832X_STEPCNT2_RST_CNT            0x04
#define STK832X_STEPCNT2_STEP_CNT_EN        0x08

/* STK832X_REG_INTFCFG */
#define STK832X_INTFCFG_I2C_WDT_EN          0x04

/* STK832X_REG_OFSTCOMP1 */
#define STK832X_OFSTCOMP1_OFST_RST          0x80

/* STK832X_REG_CFG1 */
/* the maximum space for FIFO is 32*3 bytes */
#define STK832X_CFG1_XYZ_FRAME_MAX          32

/* STK832X_REG_CFG2 */
#define STK832X_CFG2_FIFO_MODE_BYPASS       0x0
#define STK832X_CFG2_FIFO_MODE_FIFO         0x1
#define STK832X_CFG2_FIFO_MODE_SHIFT        5
#define STK832X_CFG2_FIFO_DATA_SEL_XYZ      0x0
#define STK832X_CFG2_FIFO_DATA_SEL_X        0x1
#define STK832X_CFG2_FIFO_DATA_SEL_Y        0x2
#define STK832X_CFG2_FIFO_DATA_SEL_Z        0x3
#define STK832X_CFG2_FIFO_DATA_SEL_MASK     0x3

/* STK832X_REG_OFSTx */
#define STK832X_OFST_LSB                    128     /* 8 bits for +-1G */

struct stk832x_data
{
    /* platform related */
    struct i2c_client           *client;
    /* device tree */
    int                         direction;
    int                         interrupt_int1_pin;
    int                         interrupt_int2_pin;
    /* chip informateion */
    int                         pid;
    /* system operation */
    atomic_t                    enabled;            /* chip is enabled or not */
    atomic_t                    cali_status;        /* cali status */
    atomic_t                    recv;               /* recv data. DEVICE_ATTR(recv, ...) */
    struct mutex                reg_lock;           /* mutex lock for register R/W */
    u8                          power_mode;
    struct input_dev            *input_dev_accel;   /* accel data */
    struct input_dev            *input_dev_any;     /* any motion data */
    bool                        temp_enable;        /* record current power status. For Suspend/Resume used. */
    int                         sensitivity;        /* sensitivity, bit number per G */
    s16                         xyz[3];             /* The latest data of xyz */
#ifdef STK_STEP_COUNTER
    int                         steps;              /* The latest step counter value */
#endif /* STK_STEP_COUNTER */
#ifdef INTERRUPT_MODE
    int                         irq1;               /* for all data usage(DATA, FIFO, ANYMOTION) */
    struct workqueue_struct     *alldata_workqueue; /* all data workqueue for int1. (DATA, FIFO, ANYMOTION) */
    struct work_struct          alldata_work;       /* all data work for int1. (DATA, FIFO, ANYMOTION) */
#else /* no INTERRUPT_MODE */
    int                         irq1;               /* for any motion usage */
    struct delayed_work         any_delaywork;      /* any motion delay work for int1. */
    int                         irq2;               /* for fifo usage */
    struct delayed_work         fifo_delaywork;     /* fifo delay work for int2. */
    struct delayed_work         accel_delaywork;
    struct hrtimer              accel_timer;
    ktime_t                     poll_delay;
#endif /* INTERRUPT_MODE */
#ifdef STK_CHECK_CODE
    int                         cc_count;
    u8                          cc_status;          /* refer STK_CCSTATUS_x */
#endif /* STK_CHECK_CODE */
#ifdef STK_FIR
    struct data_fir             fir;
    /*
     * fir_len
     * 0: turn OFF FIR operation
     * 1 ~ STK_FIR_LEN_MAX: turn ON FIR operation
     */
    atomic_t                    fir_len;
#endif /* STK_FIR */
};

/*	direction settings	*/
static const int coordinate_trans[8][3][3] = {
	/* x_after, y_after, z_after */
	{{0,-1,0}, {1,0,0}, {0,0,1}},
	{{1,0,0}, {0,1,0}, {0,0,1}},
	{{0,1,0}, {-1,0,0}, {0,0,1}},
	{{-1,0,0}, {0,-1,0}, {0,0,1}},
	{{0,1,0}, {1,0,0}, {0,0,-1}},
	{{-1,0,0}, {0,1,0}, {0,0,-1}},
	{{0,-1,0}, {-1,0,0}, {0,0,-1}},
	{{1,0,0}, {0,-1,0}, {0,0,-1}},
};

struct stk832x_platform_data
{
    unsigned char   direction;
    int             interrupt_int1_pin;
#ifndef INTERRUPT_MODE /* polling mode */
    int             interrupt_int2_pin;
#endif /* no INTERRUPT_MODE */
};

static struct stk832x_platform_data stk832x_plat_data =
{
    .direction              = 1,
    .interrupt_int1_pin     = STK8323_INT1,
#ifndef INTERRUPT_MODE /* polling mode */
    .interrupt_int2_pin     = STK8323_INT2,
#endif /* no INTERRUPT_MODE */
};

/*
 * global 
 */
struct stk832x_data *g_stk;
/********************* Functions **********************/
#ifdef CONFIG_OF
/*
 * @brief: Parse data in device tree
 *
 * @param[in] dev: struct device *
 * @param[in/out] pdata: struct stk832x_platform_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk832x_parse_dt(struct device *dev,
                            struct stk832x_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    const int *p;
    uint32_t int_flags;
    p = of_get_property(np, "stk,direction", NULL);

    if (p)
        pdata->direction = be32_to_cpu(*p);

    pdata->interrupt_int1_pin = of_get_named_gpio_flags(np,
                                "stk832x,irq-gpio", 0, &int_flags);

    if (pdata->interrupt_int1_pin < 0)
    {
        dev_err(dev, "%s: Unable to read stk832x,irq-gpio\n", __func__);
        return pdata->interrupt_int1_pin;
    }

#ifndef INTERRUPT_MODE /* polling mode */
    pdata->interrupt_int2_pin = of_get_named_gpio_flags(np,
                                "stk832x,irq-gpio", 1, &int_flags);

    if (pdata->interrupt_int2_pin < 0)
    {
        dev_err(dev, "%s: Unable to read stk832x,irq-gpio\n", __func__);
        return pdata->interrupt_int2_pin;
    }

#endif /* no INTERRUPT_MODE */
    return 0; /* SUCCESS */
}
#else
static int stk832x_parse_dt(struct device *dev,
                            struct stk832x_platform_data *pdata)
{
    return -ENODEV;
}
#endif /* CONFIG_OF */

/*
 * stk832x register write
 * @brief: Register writing via I2C
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] reg: Register address
 * @param[in] val: Data, what you want to write.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk832x_reg_write(struct stk832x_data *stk, u8 reg, u8 val)
{
    int error = 0;
    mutex_lock(&stk->reg_lock);
    error = i2c_smbus_write_byte_data(stk->client, reg, val);
    mutex_unlock(&stk->reg_lock);

    if (error)
        dev_err(&stk->client->dev,
                "%s: failed to write reg:0x%x with val:0x%x\n",
                __func__, reg, val);

    return error;
}

/*
 * stk832x register read
 * @brief: Register reading via I2C
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] reg: Register address
 * @param[in] len: 0, for normal usage. Others, read length (FIFO used).
 * @param[out] val: Data, the register what you want to read.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk832x_reg_read(struct stk832x_data *stk, u8 reg, int len, u8 *val)
{
    int error = 0;
    struct i2c_msg msgs[2] = {
        {
            .addr = stk->client->addr,
            .flags = 0,
            .len = 1,
            .buf = &reg
        },
        {
            .addr = stk->client->addr,
            .flags = I2C_M_RD,
            .len = (0 >= len) ? 1 : len,
            .buf = val
        }
    };

    mutex_lock(&stk->reg_lock);
    error = i2c_transfer(stk->client->adapter, msgs, 2);
    mutex_unlock(&stk->reg_lock);

    if (2 == error)
        error = 0;
    else if (0 > error)
    {
        dev_err(&stk->client->dev, "transfer failed to read reg:0x%x with len:%d, error=%d\n", reg, len, error);
    }
    else
    {
        dev_err(&stk->client->dev, "size error in reading reg:0x%x with len:%d, error=%d\n", reg, len, error);
        error = -1;
    }

    return error;
}

/*
 * @brief: Get platform data
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int get_platform_data(struct stk832x_data *stk)
{
    int error = 0;
    struct stk832x_platform_data *stk_platdata;

    if (stk->client->dev.of_node)
    {
        dev_info(&stk->client->dev,
                 "%s: probe with device tree\n", __func__);
        stk_platdata = devm_kzalloc(&stk->client->dev,
                                    sizeof(struct stk832x_platform_data), GFP_KERNEL);

        if (!stk_platdata)
        {
            dev_err(&stk->client->dev,
                    "Failed to allocate memory\n");
            return -ENOMEM;
        }

        error = stk832x_parse_dt(&stk->client->dev, stk_platdata);

        if (error)
        {
            dev_err(&stk->client->dev,
                    "%s: stk832x_parse_dt ret=%d\n", __func__, error);
            return error;
        }
    }
    else
    {
        if (NULL != stk->client->dev.platform_data)
        {
            dev_info(&stk->client->dev,
                     "%s: probe with platform data\n", __func__);
            stk_platdata = stk->client->dev.platform_data;
        }
        else
        {
            dev_info(&stk->client->dev,
                     "%s: probe with private platform data\n", __func__);
            stk_platdata = &stk832x_plat_data;
        }
    }

    stk->interrupt_int1_pin = stk_platdata->interrupt_int1_pin;
#ifndef INTERRUPT_MODE /* polling mode */
    stk->interrupt_int2_pin = stk_platdata->interrupt_int2_pin;
#endif /* no INTERRUPT_MODE */
    stk->direction = stk_platdata->direction;
    return 0;
}

/*
 * @brief: Change power mode
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] pwd_md: power mode for STK832X_REG_POWMODE
 *              STK832X_PWMD_SUSPEND
 *              STK832X_PWMD_LOWPOWER
 *              STK832X_PWMD_NORMAL
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_change_power_mode(struct stk832x_data *stk, u8 pwd_md)
{
    if (pwd_md != stk->power_mode)
    {
        int error = 0;
        u8 val = 0;
        error = stk832x_reg_read(stk, STK832X_REG_POWMODE, 0, &val);

        if (error)
            return error;

        val &= STK832X_PWMD_SLP_MASK;
        error = stk832x_reg_write(stk, STK832X_REG_POWMODE, (val | pwd_md));

        if (error)
            return error;

        stk->power_mode = pwd_md;
    }
    else
        dev_info(&stk->client->dev,
                 "%s: Same as original power mode: 0x%X\n",
                 __func__, stk->power_mode);

    return 0;
}

/*
 * @brief: Get sensitivity. Set result to stk832x_data.sensitivity.
 *          sensitivity = number bit per G (LSB/g)
 *          Example: RANGESEL=8g, 12 bits for STK832x full resolution
 *          Ans: number bit per G = 2^12 / (8x2) = 256 (LSB/g)
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_get_sensitivity(struct stk832x_data *stk)
{
    u8 val = 0;
    stk->sensitivity = 0;

    if ( 0 == stk832x_reg_read(stk, STK832X_REG_RANGESEL, 0, &val))
    {
        val &= STK832X_RANGESEL_BW_MASK;

        switch (val)
        {
            case STK832X_RANGESEL_2G:
                stk->sensitivity = 1024;
                break;

            case STK832X_RANGESEL_4G:
                stk->sensitivity = 512;
                break;

            case STK832X_RANGESEL_8G:
                stk->sensitivity = 256;
                break;

            default:
                break;
        }
    }
}

/*
 * @brief: Set range
 *          1. Setting STK832X_REG_RANGESEL
 *          2. Calculate sensitivity and store to stk832x_data.sensitivity
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] range: range for STK832X_REG_RANGESEL
 *              STK832X_RANGESEL_2G
 *              STK832X_RANGESEL_4G
 *              STK832X_RANGESEL_8G
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_range_selection(struct stk832x_data *stk, u8 range)
{
    int result = 0;
    result = stk832x_reg_write(stk, STK832X_REG_RANGESEL, range);

    if (result)
        return result;

    stk_get_sensitivity(stk);
    return 0;
}

/*
 * @brief: report any motion result to /sys/class/input/inputX/capabilities/abs
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] flag: ANY MOTION status
 *              true: set 1 to /sys/class/input/inputX/capabilities/abs
 *              false: set 0 to /sys/class/input/inputX/capabilities/abs
 */
static void stk_report_ANYMOTION(struct stk832x_data *stk, bool flag)
{
	fiio_debug("%s:Enter!!!\n",__func__);
    if (!stk->input_dev_any)
    {
        dev_err(&stk->client->dev,
                "%s: No input device for ANY motion\n", __func__);
        return;
    }
    if (flag)
    {
        input_report_abs(stk->input_dev_any, ABS_MISC, 0x1);
		#ifdef FIIO_DEBUG_SENSOR
        dev_info(&stk->client->dev, "%s: trigger\n", __func__);
		#endif
		
    }
    else
    {
		input_report_abs(stk->input_dev_any, ABS_MISC, 0x0);
		#ifdef FIIO_DEBUG_SENSOR
        dev_info(&stk->client->dev, "%s: no ANY motion\n", __func__);
		#endif
    }
    input_sync(stk->input_dev_any);
	fiio_debug("%s:Exit!!!\n",__func__);
}

/*
 * @brief: Read any motion data, then report to userspace.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] check_STS: need to check ANY_MOT_STS or not
 *                  true/false
 */
static void stk_read_anymotion_data(struct stk832x_data *stk, bool check_STS)
{
    u8 data = 0;
	fiio_debug("%s:Enter!!!\n",__func__);
    if (check_STS)
    {
        if (stk832x_reg_read(stk, STK832X_REG_INTSTS1, 0, &data))
            return;

        if (STK832X_INTSTS1_ANY_MOT_STS & data)
        {
            /* TODO: report to android */
            stk_report_ANYMOTION(stk, true);
        }
        else
        {
            stk_report_ANYMOTION(stk, false);
        }
    }
    else
    {
        /* TODO: report to android */
        stk_report_ANYMOTION(stk, true);
    }
	fiio_debug("%s:Exit!!!\n",__func__);
}

/*
 * @brief: read FIFO data
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] check_STS: need to check FWM_STS or not
 *                  true/false
 */
static void stk_read_fifo_data(struct stk832x_data *stk, bool check_STS)
{
    u8 data = 0;

    if (check_STS)
    {
        if (stk832x_reg_read(stk, STK832X_REG_INTSTS2, 0, &data))
            return;

        if (STK832X_INTSTS2_FWM_STS_MASK & data)
        {
            /* This irq for FIFO */
            /* TODO: do something */
        }
    }
    else
    {
        /* This irq for FIFO */
        /* TODO: do something */
    }
}

/*
 * stk_set_enable
 * @brief: Turn ON/OFF the power state of stk832x.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] en: turn ON/OFF
 *              0 for suspend mode;
 *              1 for normal mode.
 */
static void stk_set_enable(struct stk832x_data *stk, char en)
{
    if (en == atomic_read(&stk->enabled))
        return;

    if (en)
    {
        /* ID46: Low-power -> Suspend -> Normal */
        if (stk_change_power_mode(stk, STK832X_PWMD_SUSPEND))
            return;

        if (stk_change_power_mode(stk, STK832X_PWMD_NORMAL))
            return;

#ifndef INTERRUPT_MODE /* polling mode */
        hrtimer_start(&stk->accel_timer, stk->poll_delay, HRTIMER_MODE_REL);
#endif /* no INTERRUPT_MODE */
#ifdef STK_CHECK_CODE
        stk->cc_count = 0;
        stk->cc_status = STK_CCSTATUS_NORMAL;
#endif /* STK_CHECK_CODE */
    }
    else
    {
        if (stk_change_power_mode(stk, STK832X_PWMD_SUSPEND))
            return;

#ifndef INTERRUPT_MODE /* polling mode */
        hrtimer_cancel(&stk->accel_timer);
#endif /* no INTERRUPT_MODE */
    }

    stk_report_ANYMOTION(stk, false);
    atomic_set(&stk->enabled, en);
}

/*
 * @brief: Get delay
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: delay in usec
 *          Please refer STK832X_SAMPLE_TIME[]
 */
static int stk_get_delay(struct stk832x_data *stk)
{
    u8 data = 0;
    int delay_us = 0;

    if (stk832x_reg_read(stk, STK832X_REG_BWSEL, 0, &data))
    {
        dev_err(&stk->client->dev, "%s: failed to read delay\n", __func__);
    }
    else if ((STK832X_SPTIME_BASE > data) || (STK832X_SPTIME_BOUND < data))
    {
        dev_err(&stk->client->dev, "%s: BW out of range, 0x%X\n",
                __func__, data);
    }
    else
    {
        delay_us = STK832X_SAMPLE_TIME[data - STK832X_SPTIME_BASE];
    }

    return delay_us;
}

/*
 * @brief: Set delay
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] delay_us: delay in usec
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_set_delay(struct stk832x_data *stk, int delay_us)
{
    int error = 0;
    bool enable = false;
    unsigned char sr_no;

    for (sr_no = 0; sr_no <= STK832X_SPTIME_BOUND - STK832X_SPTIME_BASE;
         sr_no++)
        if (delay_us >= STK832X_SAMPLE_TIME[sr_no])
            break;

    if (sr_no == STK832X_SPTIME_BOUND - STK832X_SPTIME_BASE + 1)
    {
        sr_no--;
        //delay_us = STK832X_SAMPLE_TIME[sr_no];
    }

    sr_no += STK832X_SPTIME_BASE;

    if (atomic_read(&stk->enabled))
    {
        stk_set_enable(stk, 0);
        enable = true;
    }

    error = stk832x_reg_write(stk, STK832X_REG_BWSEL, sr_no);

    if (error)
        dev_err(&stk->client->dev, "%s, failed to change ODR\n", __func__);

    if (enable)
    {
        stk_set_enable(stk, 1);
    }

#ifndef INTERRUPT_MODE /* polling mode */
    stk->poll_delay = ns_to_ktime(
                          STK832X_SAMPLE_TIME[sr_no - STK832X_SPTIME_BASE] * NSEC_PER_USEC);
#endif /* no INTERRUPT_MODE */
    return error;
}

#ifdef STK_CHECK_CODE
/*
 * @brief: check stiction or not
 *          If 3 times and continue stiction will change stk832x_data.cc_status
 *          to STK_CCSTATUS_ZSIM or STK_CCSTATUS_XYZIM.
 *          Others, keep stk832x_data.cc_status to STK_CCSTATUS_NORMAL.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] clean: clean internal flag of check_result or not.
 *                  true: clean check_result
 *                  false: don't clean check_result
 */
static void stk_check_data(struct stk832x_data *stk, bool clean)
{
    static s8 event_no = 0;
    static s8 check_result = 0;
    /* 12 bits per axis */
    const int max_value = 2047;
    const int min_value = -2048;

    if (18 <= event_no)
        return;

    if (max_value == stk->xyz[0] || min_value == stk->xyz[0]
        || max_value == stk->xyz[1] || min_value == stk->xyz[1]
        || max_value == stk->xyz[2] || min_value == stk->xyz[2])
    {
        dev_info(&stk->client->dev, "%s: acc:0x%X, 0x%X, 0x%X\n",
                 __func__, stk->xyz[0], stk->xyz[1], stk->xyz[2]);
        check_result++;
    }
    else
    {
        check_result = 0;
        goto exit;
    }

    if (clean)
    {
        if (3 <= check_result)
        {
            if (max_value != stk->xyz[0] && min_value != stk->xyz[0]
                && max_value != stk->xyz[1] && min_value != stk->xyz[1])
                stk->cc_status = STK_CCSTATUS_ZSIM;
            else
                stk->cc_status = STK_CCSTATUS_XYSIM;

            dev_info(&stk->client->dev, "%s: incorrect reading\n", __func__);
        }

        check_result = 0;
    }

exit:
    event_no++;
    return;
}

/*
 * @brief: check_code operation
 *          z = sqrt(x^2 + y^2)
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_check_code(struct stk832x_data *stk)
{
    u16 x, y;
    int sen, item;
    sen = stk->sensitivity;

    if (0 <= stk->xyz[0])
        x = stk->xyz[0];
    else
        x = -stk->xyz[0];

    if (0 <= stk->xyz[1])
        y = stk->xyz[1];
    else
        y = -stk->xyz[1];

    if ((x >= sen) || (y >= sen))
    {
        stk->xyz[2] = 0;
        return;
    }

    switch (sen)
    {
        case 1024:
            /* 2G */
            item = (x >> 1) * sen + (y >> 1);

            if (stkCheckCode_4g[item])
                stk->xyz[2] = (s16)stkCheckCode_4g[item];
            else
            {
                stk->xyz[2] = 0;
                dev_err(&stk->client->dev,
                        "%s: null point for stkCheckCode_4g[%d][%d]\n",
                        __func__, x, y);
            }

            break;

        case 512:
            /* 4G */
            item = x * sen + y;

            if (stkCheckCode_4g[item])
                stk->xyz[2] = (s16)stkCheckCode_4g[item];
            else
            {
                stk->xyz[2] = 0;
                dev_err(&stk->client->dev,
                        "%s: null point for stkCheckCode_4g[%d][%d]\n",
                        __func__, x, y);
            }

            break;

        case 256:
            /*8G */
            item = (x << 1) * sen + (y << 1);

            if (stkCheckCode_4g[item])
                stk->xyz[2] = (s16)stkCheckCode_4g[item];
            else
            {
                stk->xyz[2] = 0;
                dev_err(&stk->client->dev,
                        "%s: null point for stkCheckCode_4g[%d][%d]\n",
                        __func__, x, y);
            }

            break;

        default:
            dev_err(&stk->client->dev, "%s: failed. sen=%d, x=%d, y=%d\n",
                    __func__, sen, stk->xyz[0], stk->xyz[1]);
            stk->xyz[2] = 0;
            break;
    }
}
#endif /* STK_CHECK_CODE */

#ifdef STK_FIR
/*
 * @brief: low-pass filter operation
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_low_pass_fir(struct stk832x_data *stk)
{
    int firlength = atomic_read(&stk->fir_len);
#ifdef STK_ZG
    s16 avg;
    int jitter_boundary = stk->sensitivity / 128;
#if 0

    if (0 == jitter_boundary)
        jitter_boundary = 1;

#endif
#endif /* STK_ZG */

    if (0 == firlength)
    {
        /* stk832x_data.fir_len == 0: turn OFF FIR operation */
        return;
    }

    if (firlength > stk->fir.count)
    {
        stk->fir.xyz[stk->fir.idx][0] = stk->xyz[0];
        stk->fir.xyz[stk->fir.idx][1] = stk->xyz[1];
        stk->fir.xyz[stk->fir.idx][2] = stk->xyz[2];
        stk->fir.sum[0] += stk->xyz[0];
        stk->fir.sum[1] += stk->xyz[1];
        stk->fir.sum[2] += stk->xyz[2];
        stk->fir.count++;
        stk->fir.idx++;
    }
    else
    {
        if (firlength <= stk->fir.idx)
            stk->fir.idx = 0;

        stk->fir.sum[0] -= stk->fir.xyz[stk->fir.idx][0];
        stk->fir.sum[1] -= stk->fir.xyz[stk->fir.idx][1];
        stk->fir.sum[2] -= stk->fir.xyz[stk->fir.idx][2];
        stk->fir.xyz[stk->fir.idx][0] = stk->xyz[0];
        stk->fir.xyz[stk->fir.idx][1] = stk->xyz[1];
        stk->fir.xyz[stk->fir.idx][2] = stk->xyz[2];
        stk->fir.sum[0] += stk->xyz[0];
        stk->fir.sum[1] += stk->xyz[1];
        stk->fir.sum[2] += stk->xyz[2];
        stk->fir.idx++;
#ifdef STK_ZG
        avg = stk->fir.sum[0] / firlength;

        if (abs(avg) <= jitter_boundary)
            stk->xyz[0] = avg * ZG_FACTOR;
        else
            stk->xyz[0] = avg;

        avg = stk->fir.sum[1] / firlength;

        if (abs(avg) <= jitter_boundary)
            stk->xyz[1] = avg * ZG_FACTOR;
        else
            stk->xyz[1] = avg;

        avg = stk->fir.sum[2] / firlength;

        if (abs(avg) <= jitter_boundary)
            stk->xyz[2] = avg * ZG_FACTOR;
        else
            stk->xyz[2] = avg;

#else /* STK_ZG */
        stk->xyz[0] = stk->fir.sum[0] / firlength;
        stk->xyz[1] = stk->fir.sum[1] / firlength;
        stk->xyz[2] = stk->fir.sum[2] / firlength;
#endif /* STK_ZG */
    }
}
#endif /* STK_FIR */

/**
 * @brief: read accel raw data from register.
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_read_accel_rawdata(struct stk832x_data *stk)
{
    u8 dataL = 0;
    u8 dataH = 0;

    if (stk832x_reg_read(stk, STK832X_REG_XOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_XOUT2, 0, &dataH))
        return;

    stk->xyz[0] = dataH << 8 | dataL;
    stk->xyz[0] >>= 4;

    if (stk832x_reg_read(stk, STK832X_REG_YOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_YOUT2, 0, &dataH))
        return;

    stk->xyz[1] = dataH << 8 | dataL;
    stk->xyz[1] >>= 4;

    if (stk832x_reg_read(stk, STK832X_REG_ZOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_ZOUT2, 0, &dataH))
        return;

    stk->xyz[2] = dataH << 8 | dataL;
    stk->xyz[2] >>= 4;
}

/*
 * @brief: read accel low data from register.
 *          Store result to stk832x_data.xyz[].
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_read_accel_data(struct stk832x_data *stk)
{
    int ii = 0;
    s16 coor_trans[3] = {0};

    stk_read_accel_rawdata(stk);
#ifdef STK_CHECK_CODE

    if ((STK_CHECKCODE_IGNORE + 1) == stk->cc_count
        || (STK_CHECKCODE_IGNORE + 2) == stk->cc_count)
        stk_check_data(stk, false);
    else if ((STK_CHECKCODE_IGNORE + 3) == stk->cc_count)
        stk_check_data(stk, true);
    else if (STK_CCSTATUS_ZSIM == stk->cc_status)
        stk_check_code(stk);

    if ((STK_CHECKCODE_IGNORE + 6) > stk->cc_count)
        stk->cc_count++;

#endif /* STK_CHECK_CODE */

#ifdef FIIO_DEBUG_SENSOR 
    dev_info(&stk->client->dev, "%s: xyz before coordinate trans %d %d %d with direction:%d\n",
            __func__, stk->xyz[0], stk->xyz[1], stk->xyz[2], stk->direction);
#endif

    for (ii = 0; ii < 3; ii++)
    {
        coor_trans[0] += stk->xyz[ii] * coordinate_trans[stk->direction][0][ii];
        coor_trans[1] += stk->xyz[ii] * coordinate_trans[stk->direction][1][ii];
        coor_trans[2] += stk->xyz[ii] * coordinate_trans[stk->direction][2][ii];
    }
    stk->xyz[0] = coor_trans[0];
    stk->xyz[1] = coor_trans[1];
    stk->xyz[2] = coor_trans[2];
	
#ifdef FIIO_DEBUG_SENSOR
    dev_info(&stk->client->dev, "%s: xyz after coordinate trans %d %d %d\n",
            __func__, stk->xyz[0], stk->xyz[1], stk->xyz[2]);
#endif

#ifdef STK_FIR
    stk_low_pass_fir(stk);
#endif /* STK_FIR */
}

#ifdef STK_STEP_COUNTER
/**
 * @brief: read step counter value from register.
 *          Store result to stk832x_data.steps.
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_read_step_data(struct stk832x_data *stk)
{
    u8 dataL = 0;
    u8 dataH = 0;

    if (stk832x_reg_read(stk, STK832X_REG_STEPOUT1, 0, &dataL))
        return;

    if (stk832x_reg_read(stk, STK832X_REG_STEPOUT2, 0, &dataH))
        return;

    stk->steps = dataH << 8 | dataL;
}

/**
 * @brief: Turn ON/OFF step count.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] turn: true to turn ON step count; false to turn OFF.
 */
static void stk_turn_step_counter(struct stk832x_data *stk, bool turn)
{
    if (turn)
    {
        if (stk832x_reg_write(stk, STK832X_REG_STEPCNT2,
                              STK832X_STEPCNT2_RST_CNT | STK832X_STEPCNT2_STEP_CNT_EN))
            return;
    }
    else
    {
        if (stk832x_reg_write(stk, STK832X_REG_STEPCNT2, 0))
            return;
    }

    stk->steps = 0;
}
#endif /* STK_STEP_COUNTER */

/*
 * @brief: Write calibration config file to STK_CALI_FILE.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] w_buf: cali data what want to write to STK_CALI_FILE.
 * @param[in] buf_size: size of w_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_write_to_file(struct stk832x_data *stk,
                             char *w_buf, int8_t buf_size)
{
    struct file *cali_file;
    char r_buf[buf_size];
    mm_segment_t fs;
    ssize_t ret;
    int i;
    cali_file = filp_open(STK_CALI_FILE, O_CREAT | O_RDWR, 0666);

    if (IS_ERR(cali_file))
    {
        dev_err(&stk->client->dev,
                "%s: err=%ld, failed to open %s\n",
                __func__, PTR_ERR(cali_file), STK_CALI_FILE);
        return -ENOENT;
    }
    else
    {
        fs = get_fs();
        set_fs(get_ds());
        ret = cali_file->f_op->write(cali_file, w_buf, buf_size,
                                     &cali_file->f_pos);

        if (0 > ret)
        {
            dev_err(&stk->client->dev, "%s: write error, ret=%d\n",
                    __func__, (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }

        cali_file->f_pos = 0x0;
        ret = cali_file->f_op->read(cali_file, r_buf, buf_size,
                                    &cali_file->f_pos);

        if (0 > ret)
        {
            dev_err(&stk->client->dev, "%s: read error, ret=%d\n",
                    __func__, (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }

        set_fs(fs);

        for (i = 0; i < buf_size; i++)
        {
            if (r_buf[i] != w_buf[i])
            {
                dev_err(&stk->client->dev,
                        "%s: read back error! r_buf[%d]=0x%X, w_buf[%d]=0x%X\n",
                        __func__, i, r_buf[i], i, w_buf[i]);
                filp_close(cali_file, NULL);
                return -1;
            }
        }
    }

    filp_close(cali_file, NULL);
    return 0;
}

/*
 * @brief: Get calibration config file from STK_CALI_FILE.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] r_buf: cali data what want to read from STK_CALI_FILE.
 * @param[in] buf_size: size of r_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_get_from_file(struct stk832x_data *stk,
                             char *r_buf, int8_t buf_size)
{
    struct file *cali_file;
    mm_segment_t fs;
    ssize_t ret;
    cali_file = filp_open(STK_CALI_FILE, O_RDONLY, 0);

    if (IS_ERR(cali_file))
    {
        dev_err(&stk->client->dev,
                "%s: err=%ld, failed to open %s\n",
                __func__, PTR_ERR(cali_file), STK_CALI_FILE);
        return -ENOENT;
    }
    else
    {
        fs = get_fs();
        set_fs(get_ds());
        ret = cali_file->f_op->read(cali_file, r_buf, buf_size,
                                    &cali_file->f_pos);
        set_fs(fs);

        if (0 > ret)
        {
            dev_err(&stk->client->dev, "%s: read error, ret=%d\n",
                    __func__, (int)ret);
            filp_close(cali_file, NULL);
            return -EIO;
        }
    }

    filp_close(cali_file, NULL);
    return 0;
}

/*
 * @brief: Get calibration data and status.
 *          Set cali status to stk832x_data.cali_status.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] r_buf: cali data what want to read from STK_CALI_FILE.
 * @param[in] buf_size: size of r_buf.
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static void stk_get_cali(struct stk832x_data *stk)
{
    char stk_file[STK_CALI_FILE_SIZE];

    if (stk_get_from_file(stk, stk_file, STK_CALI_FILE_SIZE) == 0)
    {
        if (STK_CALI_VER0 == stk_file[0]
            && STK_CALI_VER1 == stk_file[1]
            && STK_CALI_END == stk_file[STK_CALI_FILE_SIZE - 1])
        {
            atomic_set(&stk->cali_status, (int)stk_file[8]);
            dev_info(&stk->client->dev, "%s: offset:%d,%d,%d, mode=0x%X\n",
                     __func__, stk_file[3], stk_file[5], stk_file[7],
                     stk_file[8]);
            dev_info(&stk->client->dev, "%s: variance=%u,%u,%u\n", __func__,
                     (stk_file[9] << 24 | stk_file[10] << 16 | stk_file[11] << 8 | stk_file[12]),
                     (stk_file[13] << 24 | stk_file[14] << 16 | stk_file[15] << 8 | stk_file[16]),
                     (stk_file[17] << 24 | stk_file[18] << 16 | stk_file[19] << 8 | stk_file[20]));
        }
        else
        {
            int i;
            dev_err(&stk->client->dev, "%s: wrong cali version number\n",
                    __func__);

            for (i = 0; i < STK_CALI_FILE_SIZE; i++)
                dev_info(&stk->client->dev, "%s:cali_file[%d]=0x%X\n",
                         __func__, i, stk_file[i]);
        }
    }
}

/*
 * @brief: Get sample_no of samples then calculate average
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] delay_ms: delay in msec
 * @param[in] sample_no: amount of sample
 * @param[out] acc_ave: XYZ average
 */
static void stk_calculate_average(struct stk832x_data *stk,
                                  unsigned int delay_ms, int sample_no, int acc_ave[3])
{
    int i;

    for (i = 0; i < sample_no; i++)
    {
        msleep(delay_ms);
        stk_read_accel_data(stk);
        acc_ave[0] += stk->xyz[0];
        acc_ave[1] += stk->xyz[1];
        acc_ave[2] += stk->xyz[2];
    }

    /*
     * Take ceiling operation.
     * ave = (ave + SAMPLE_NO/2) / SAMPLE_NO
     *     = ave/SAMPLE_NO + 1/2
     * Example: ave=7, SAMPLE_NO=10
     * Ans: ave = 7/10 + 1/2 = (int)(1.2) = 1
     */
    for (i = 0; i < 3; i++)
    {
        if ( 0 <= acc_ave[i])
            acc_ave[i] = (acc_ave[i] + sample_no / 2) / sample_no;
        else
            acc_ave[i] = (acc_ave[i] - sample_no / 2) / sample_no;
    }

    /*
     * For Z-axis
     * Pre-condition: Sensor be put on a flat plane, with +z face up.
     */
    if (0 < acc_ave[2])
        acc_ave[2] -= stk->sensitivity;
    else
        acc_ave[2] += stk->sensitivity;
}

/*
 * @brief: Align STK832X_REG_OFSTx sensitivity with STK832X_REG_RANGESEL
 *  Description:
 *  Example:
 *      RANGESEL=0x3 -> +-2G / 12bits for STK832x full resolution
 *              number bit per G = 2^12 / (2x2) = 1024 (LSB/g)
 *              (2x2) / 2^12 = 0.97 mG/bit
 *      OFSTx: There are 8 bits to describe OFSTx for +-1G
 *              number bit per G = 2^8 / (1x2) = 128 (LSB/g)
 *              (1x2) / 2^8 = 7.8125mG/bit
 *      Align: acc_OFST = acc * 128 / 1024
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in/out] acc: accel data
 *
 */
static void stk_align_offset_sensitivity(struct stk832x_data *stk, int acc[3])
{
    int axis;

    /*
     * Take ceiling operation.
     * ave = (ave + SAMPLE_NO/2) / SAMPLE_NO
     *     = ave/SAMPLE_NO + 1/2
     * Example: ave=7, SAMPLE_NO=10
     * Ans: ave = 7/10 + 1/2 = (int)(1.2) = 1
     */
    for (axis = 0; axis < 3; axis++)
    {
        if (acc[axis] > 0)
        {
            acc[axis] = (acc[axis] * STK832X_OFST_LSB + stk->sensitivity / 2)
                        / stk->sensitivity;
        }
        else
        {
            acc[axis] = (acc[axis] * STK832X_OFST_LSB - stk->sensitivity / 2)
                        / stk->sensitivity;
        }
    }
}

/*
 * @brief: Read all register (0x0 ~ 0x3F)
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] show_buffer: record all register value
 *
 * @return: buffer length or fail
 *          positive value: return buffer length
 *          -1: Fail
 */
static int stk_show_all_reg(struct stk832x_data *stk, char *show_buffer)
{
    bool enable = false;
    int reg;
    int len = 0;
    u8 data = 0;

    if (NULL == show_buffer)
        return -1;

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    for (reg = 0; reg <= 0x3F; reg++)
    {
        if (stk832x_reg_read(stk, reg, 0, &data))
        {
            len = -1;
            goto exit;
        }

        if (0 >= (PAGE_SIZE - len))
        {
            dev_err(&stk->client->dev,
                    "%s: print string out of PAGE_SIZE\n", __func__);
            goto exit;
        }

        len += scnprintf(show_buffer + len, PAGE_SIZE - len,
                         "[0x%2X]=0x%2X\n ", reg, data);
    }

    len += scnprintf(show_buffer + len, PAGE_SIZE - len, "\n");
exit:

    if (!enable)
        stk_set_enable(stk, 0);

    return len;
}

/*
 * @brief: Get offset
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] offset: offset value read from register
 *                  STK832X_REG_OFSTX,  STK832X_REG_OFSTY, STK832X_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_get_offset(struct stk832x_data *stk, u8 offset[3])
{
    int error = 0;
    bool enable = false;

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_read(stk, STK832X_REG_OFSTX, 0, &offset[0]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_read(stk, STK832X_REG_OFSTY, 0, &offset[1]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_read(stk, STK832X_REG_OFSTZ, 0, &offset[2]))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    return error;
}

/*
 * @brief: Set offset
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] offset: offset value write to register
 *                  STK832X_REG_OFSTX,  STK832X_REG_OFSTY, STK832X_REG_OFSTZ
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_set_offset(struct stk832x_data *stk, u8 offset[3])
{
    int error = 0;
    bool enable = false;

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_write(stk, STK832X_REG_OFSTX, offset[0]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_write(stk, STK832X_REG_OFSTY, offset[1]))
    {
        error = -1;
        goto exit;
    }

    if (stk832x_reg_write(stk, STK832X_REG_OFSTZ, offset[2]))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    return error;
}

/*
 * @brief: Verify offset.
 *          Read register of STK832X_REG_OFSTx, then check data are the same as
 *          what we wrote or not.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] offset: offset value to compare with the value in register
 *
 * @return: Success or fail
 *          0: Success
 *          STK_K_FAIL_I2C: I2C error
 *          STK_K_FAIL_WRITE_OFSET: offset value not the same as the value in
 *                                  register
 */
static int stk_verify_offset(struct stk832x_data *stk, u8 offset[3])
{
    int axis;
    u8 offset_from_reg[3] = {0, 0, 0};

    if (stk_get_offset(stk, offset_from_reg))
        return STK_K_FAIL_I2C;

    for (axis = 0; axis < 3; axis++)
    {
        if (offset_from_reg[axis] != offset[axis])
        {
            dev_err(&stk->client->dev,
                    "%s: set OFST failed! offset[%d]=%d, read from reg[%d]=%d\n",
                    __func__, axis, offset[axis], axis, offset_from_reg[axis]);
            atomic_set(&stk->cali_status, STK_K_FAIL_WRITE_OFST);
            return STK_K_FAIL_WRITE_OFST;
        }
    }

    return 0;
}

/*
 * @brief: Write calibration data to config file
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] offset: offset value
 * @param[in] status: status
 *                  STK_K_SUCCESS_FILE
 *
 * @return: Success or fail
 *          0: Success
 *          -1: Fail
 */
static int stk_write_cali_to_file(struct stk832x_data *stk,
                                  u8 offset[3], u8 status)
{
    char file_buf[STK_CALI_FILE_SIZE];
    memset(file_buf, 0, sizeof(file_buf));
    file_buf[0] = STK_CALI_VER0;
    file_buf[1] = STK_CALI_VER1;
    file_buf[3] = offset[0];
    file_buf[5] = offset[1];
    file_buf[7] = offset[2];
    file_buf[8] = status;
    file_buf[STK_CALI_FILE_SIZE - 2] = '\0';
    file_buf[STK_CALI_FILE_SIZE - 1] = STK_CALI_END;

    if (stk_write_to_file(stk, file_buf, STK_CALI_FILE_SIZE))
        return -1;

    return 0;
}

/*
 * @brief: Calibration action
 *          1. Calculate calibration data
 *          2. Write data to STK832X_REG_OFSTx
 *          3. Check calibration well-done with chip register
 *          4. Write calibration data to file
 *          Pre-condition: Sensor be put on a flat plane, with +z face up.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] delay_us: delay in usec
 *
 * @return: Success or fail
 *          0: Success
 *          STK_K_FAIL_I2C: I2C error
 *          STK_K_FAIL_WRITE_OFSET: offset value not the same as the value in
 *                                  register
 *          STK_K_FAIL_W_FILE: fail during writing cali to file
 */
static int stk_cali_do(struct stk832x_data *stk, int delay_us)
{
    int error = 0;
    int acc_ave[3] = {0, 0, 0};
    unsigned int delay_ms = delay_us / 1000;
    u8 offset[3] = {0, 0, 0};
    int acc_verify[3] = {0, 0, 0};
    const unsigned char verify_diff = stk->sensitivity / 10;
    int axis;
#ifdef STK_CHECK_CODE
    msleep(delay_ms * STK_CHECKCODE_IGNORE);
#endif /* STK_CHECK_CODE */
    stk_calculate_average(stk, delay_ms, STK_CALI_SAMPLE_NO, acc_ave);
    stk_align_offset_sensitivity(stk, acc_ave);

    for (axis = 0; axis < 3; axis++)
        offset[axis] = -acc_ave[axis];

    dev_info(&stk->client->dev, "%s: New offset for XYZ: %d, %d, %d\n",
             __func__, acc_ave[0], acc_ave[1], acc_ave[2]);
    error = stk_set_offset(stk, offset);

    if (error)
        return STK_K_FAIL_I2C;

    /* Read register, then check OFSTx are the same as we wrote or not */
    error = stk_verify_offset(stk, offset);

    if (error)
        return error;

    /* verify cali */
    stk_calculate_average(stk, delay_ms, 3, acc_verify);

    if (verify_diff < abs(acc_verify[0]) || verify_diff < abs(acc_verify[1])
        || verify_diff < abs(acc_verify[2]))
    {
        dev_err(&stk->client->dev, "%s: Check data x:%d, y:%d, z:%d. Check failed!\n",
                __func__, acc_verify[0], acc_verify[1], acc_verify[2]);
        return STK_K_FAIL_VERIFY_CALI;
    }

    /* write cali to file */
    error = stk_write_cali_to_file(stk, offset, STK_K_SUCCESS_FILE);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: failed to stk_write_cali_to_file, error=%d\n",
                __func__, error);
        return STK_K_FAIL_W_FILE;
    }

    atomic_set(&stk->cali_status, STK_K_SUCCESS_FILE);
    return 0;
}

/*
 * @brief: Set calibration
 *          1. Change delay to 8000msec
 *          2. Reset offset value by trigger OFST_RST
 *          3. Calibration action
 *          4. Change delay value back
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_set_cali(struct stk832x_data *stk)
{
    int error = 0;
    bool enable;
    int org_delay_us, real_delay_us;
    atomic_set(&stk->cali_status, STK_K_RUNNING);
    org_delay_us = stk_get_delay(stk);
    /* Use several samples (with ODR:125) for calibration data base */
    error = stk_set_delay(stk, 8000);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: failed to stk_set_delay, error=%d\n", __func__, error);
        atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
        return;
    }

    real_delay_us = stk_get_delay(stk);

    /* SW reset before getting calibration data base */
    if (atomic_read(&stk->enabled))
    {
        enable = true;
        stk_set_enable(stk, 0);
    }
    else
        enable = false;

    stk_set_enable(stk, 1);
    error = stk832x_reg_write(stk, STK832X_REG_OFSTCOMP1,
                              STK832X_OFSTCOMP1_OFST_RST);

    if (error)
    {
        atomic_set(&stk->cali_status, STK_K_FAIL_I2C);
        goto exit_for_OFST_RST;
    }

    /* Action for calibration */
    error = stk_cali_do(stk, real_delay_us);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: failed to stk_cali_do, error=%d\n",
                __func__, error);
        atomic_set(&stk->cali_status, error);
        goto exit_for_OFST_RST;
    }

    dev_info(&stk->client->dev, "%s: successful calibration\n", __func__);
exit_for_OFST_RST:

    if (!enable)
        stk_set_enable(stk, 0);

    stk_set_delay(stk, org_delay_us);
}

/*
 * @brief: Read FIFO data
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[out] fifo: FIFO data
 * @param[in] len: FIFO size what you want to read
 */
static void stk_fifo_reading(struct stk832x_data *stk, u8 fifo[], int len)
{
    /* Reject all register R/W to protect FIFO data reading */
    dev_info(&stk->client->dev, "%s: Start to read FIFO data\n", __func__);

    if (stk832x_reg_read(stk, STK832X_REG_FIFOOUT, len, fifo))
    {
        dev_err(&stk->client->dev, "%s: Break to read FIFO data\n", __func__);
    }

    dev_info(&stk->client->dev, "%s: Done for reading FIFO data\n", __func__);
}

/*
 * @brief: Change FIFO status
 *          If wm = 0, change FIFO to bypass mode.
 *          STK832X_CFG1_XYZ_FRAME_MAX >= wm, change FIFO to FIFO mode +
 *                                          STK832X_CFG2_FIFO_DATA_SEL_XYZ.
 *          Do nothing if STK832X_CFG1_XYZ_FRAME_MAX < wm.
 *
 * @param[in/out] stk: struct stk832x_data *
 * @param[in] wm: water mark
 *
 * @return: Success or fail
 *          0: Success
 *          Others: Fail
 */
static int stk_change_fifo_status(struct stk832x_data *stk, u8 wm)
{
    int error = 0;

    if (STK832X_CFG1_XYZ_FRAME_MAX < wm)
    {
        dev_err(&stk->client->dev,
                "%s: water mark out of range(%d).\n", __func__, wm);
        return -1;
    }

    if (wm)
    {
        /* FIFO settings: FIFO mode + XYZ per frame */
        error = stk832x_reg_write(stk, STK832X_REG_CFG2,
                                  (STK832X_CFG2_FIFO_MODE_FIFO << STK832X_CFG2_FIFO_MODE_SHIFT)
                                  | STK832X_CFG2_FIFO_DATA_SEL_XYZ);

        if (error)
            return error;
    }
    else
    {
        /* FIFO settings: bypass mode */
        error = stk832x_reg_write(stk, STK832X_REG_CFG2,
                                  STK832X_CFG2_FIFO_MODE_BYPASS << STK832X_CFG2_FIFO_MODE_SHIFT);

        if (error)
            return error;
    }

    error = stk832x_reg_write(stk, STK832X_REG_CFG1, wm);

    if (error)
        return error;

    return 0;
}

/*
 * @brief: Get power status
 *          Send 0 or 1 to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    char en;
    en = atomic_read(&stk->enabled);
    return scnprintf(buf, PAGE_SIZE, "%d\n", en);
}

/*
 * @brief: Set power status
 *          Get 0 or 1 from userspace, then set stk832x power status.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_enable_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    unsigned int data;
    int error;
    error = kstrtouint(buf, 10, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoul failed, error=%d\n",
                __func__, error);
        return error;
    }

    if ((1 == data) || (0 == data))
        stk_set_enable(stk, data);
    else
        dev_err(&stk->client->dev, "%s: invalid argument, en=%d\n",
                __func__, data);

    return count;
}

/*
 * @brief: Get accel data
 *          Send accel data to userspce.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_value_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    stk_read_accel_data(stk);
    return scnprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
                     stk->xyz[0], stk->xyz[1], stk->xyz[2]);
}

/*
 * @brief: Get delay value in usec
 *          Send delay in usec to userspce.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_delay_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)stk_get_delay(stk) * 1000);
}

/*
 * @brief: Set delay value in usec
 *          Get delay value in usec from userspace, then write to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_delay_store(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    long data;
    int error;
    error = kstrtoll(buf, 10, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoul failed, error=%d\n",
                __func__, error);
        return error;
    }

   stk_set_delay(stk, (int)(data / 1000));
    return count;
}

/*
 * @brief: Get calibration status
 *          Send calibration status to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_cali_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);

    if (STK_K_RUNNING != atomic_read(&stk->cali_status))
        stk_get_cali(stk);

    return scnprintf(buf, PAGE_SIZE, "0x%02X\n", atomic_read(&stk->cali_status));
}

/*
 * @brief: Trigger to calculate calibration data
 *          Get 1 from userspace, then start to calculate calibration data.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_cali_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);

    if (sysfs_streq(buf, "1"))
        stk_set_cali(stk);
    else
    {
        dev_err(&stk->client->dev, "%s: invalid value %d\n", __func__, *buf);
        return -EINVAL;
    }

    return count;
}

/*
 * @brief: Get offset value
 *          Send X/Y/Z offset value to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_offset_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    u8 offset[3] = {0, 0, 0};
    stk_get_offset(stk, offset);
    return scnprintf(buf, PAGE_SIZE, "0x%X 0x%X 0x%X\n",
                     offset[0], offset[1], offset[2]);
}

/*
 * @brief: Set offset value
 *          Get X/Y/Z offset value from userspace, then write to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_offset_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    char *token[10];
    u8 r_offset[3];
    int error, data, i;

    for (i = 0; i < 3; i++)
        token[i] = strsep((char **)&buf, " ");

    error = kstrtoint(token[0], 16, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    r_offset[0] = (u8)data;
    error = kstrtoint(token[1], 16, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    r_offset[1] = (u8)data;
    error = kstrtoint(token[2], 16, &data);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    r_offset[2] = (u8)data;
    dev_info(&stk->client->dev, "%s: offset=0x%X, 0x%X, 0x%X\n", __func__,
             r_offset[0], r_offset[1], r_offset[2]);
    stk_set_offset(stk, r_offset);
    return count;
}

/*
 * @brief: Register writting
 *          Get address and content from userspace, then write to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_send_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    char *token[10];
    int addr, cmd, error, i;
    bool enable = false;

    for (i = 0; i < 2; i++)
        token[i] = strsep((char **)&buf, " ");

    error = kstrtoint(token[0], 16, &addr);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    error = kstrtoint(token[1], 16, &cmd);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    dev_info(&stk->client->dev, "%s, write reg[0x%X]=0x%X\n",
             __func__, addr, cmd);

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_write(stk, (u8)addr, (u8)cmd))
    {
        error = -1;
        goto exit;
    }

exit:

    if (!enable)
        stk_set_enable(stk, 0);

    if (error)
        return -1;

    return count;
}

/*
 * @brief: Read stk832x_data.recv(from stk_recv_store), then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_recv_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "0x%X\n", atomic_read(&stk->recv));
}

/*
 * @brief: Get the read address from userspace, then store the result to
 *          stk832x_data.recv.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_recv_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    int addr, error;
    u8 data = 0;
    bool enable = false;
    error = kstrtoint(buf, 16, &addr);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (!atomic_read(&stk->enabled))
        stk_set_enable(stk, 1);
    else
        enable = true;

    if (stk832x_reg_read(stk, (u8)addr, 0, &data))
    {
        error = -1;
        goto exit;
    }

    atomic_set(&stk->recv, data);
    dev_info(&stk->client->dev, "%s: read reg[0x%X]=0x%X\n",
             __func__, addr, data);
exit:

    if (!enable)
        stk_set_enable(stk, 0);

    if (error)
        return -1;

    return count;
}

/*
 * @brief: Read all register value, then send result to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_allreg_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    int result;
    result = stk_show_all_reg(stk, buf);

    if (0 >  result)
        return result;

    return (ssize_t)result;
}

/*
 * @brief: Check PID, then send chip number to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_chipinfo_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);

    if (STK8323_ID == stk->pid)
        return scnprintf(buf, PAGE_SIZE, "stk8321/8323\n");
    else if (STK8325_ID == stk->pid)
        return scnprintf(buf, PAGE_SIZE, "stk8325\n");

    return scnprintf(buf, PAGE_SIZE, "unknown\n");
}

/*
 * @brief: Read FIFO data, then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_fifo_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    u8 fifo_wm = 0;
    u8 frame_unit = 0;
    int fifo_len, len = 0;

    if (stk832x_reg_read(stk, STK832X_REG_CFG1, 0, &fifo_wm))
        return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");

    if (0 == fifo_wm)
        return scnprintf(buf, PAGE_SIZE , "FIFO disabled\n");

    if (stk832x_reg_read(stk, STK832X_REG_CFG2, 0, &frame_unit))
        return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");

    frame_unit &= STK832X_CFG2_FIFO_DATA_SEL_MASK;

    if (0 == frame_unit)
        fifo_len = fifo_wm * 6; /* xyz * 2 bytes/axis */
    else
        fifo_len = fifo_wm * 2; /* single axis * 2 bytes/axis */

    {
        u8 *fifo = NULL;
        int i;
        /* vzalloc: allocate memory and set to zero. */
        fifo = vzalloc(sizeof(u8) * fifo_len);

        if (!fifo)
        {
            dev_err(&stk->client->dev, "%s: memory allocation error\n",
                    __func__);
            return scnprintf(buf, PAGE_SIZE , "fail to read FIFO\n");
        }

        stk_fifo_reading(stk, fifo, fifo_len);

        for (i = 0; i < fifo_wm; i++)
        {
            if (0 == frame_unit)
            {
                s16 x, y, z;
                x = fifo[i * 6 + 1] << 8 | fifo[i * 6];
                x >>= 4;
                y = fifo[i * 6 + 3] << 8 | fifo[i * 6 + 2];
                y >>= 4;
                z = fifo[i * 6 + 5] << 8 | fifo[i * 6 + 4];
                z >>= 4;
                len += scnprintf(buf + len, PAGE_SIZE - len,
                                 "%dth x:%d, y:%d, z:%d\n", i, x, y, z);
            }
            else
            {
                s16 xyz;
                xyz = fifo[i * 2 + 1] << 8 | fifo[i * 2];
                xyz >>= 4;
                len += scnprintf(buf + len, PAGE_SIZE - len,
                                 "%dth fifo:%d\n", i, xyz);
            }

            if ( 0 >= (PAGE_SIZE - len))
            {
                dev_err(&stk->client->dev,
                        "%s: print string out of PAGE_SIZE\n", __func__);
                break;
            }
        }

        vfree(fifo);
    }
    return len;
}

/*
 * @brief: Read water mark from userspace, then send to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_fifo_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    int wm, error;
    error = kstrtoint(buf, 10, &wm);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (stk_change_fifo_status(stk, (u8)wm))
    {
        return -1;
    }

    return count;
}

#ifdef STK_STEP_COUNTER
/*
 * @brief: Read step counter data, then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */

static ssize_t stk_step_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    /*
        bool enable = true;

        if (!atomic_read(&stk->enabled))
        {
            stk_set_enable(stk, 1);
            enable = false;
        }
    */
    stk_read_step_data(stk);
    /*
        if (!enable)
            stk_set_enable(stk, 0);
    */
    return scnprintf(buf, PAGE_SIZE, "%d\n", stk->steps);
}

/*
 * @brief: Read step counter setting from userspace, then send to register.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_step_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    int step, error;
    error = kstrtoint(buf, 10, &step);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (step)
        stk_turn_step_counter(stk, true);
    else
        stk_turn_step_counter(stk, false);

    return count;
}
#endif /* STK_STEP_COUNTER */

#ifdef STK_FIR
/*
 * @brief: Get FIR parameter, then send to userspace.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 *
 * @return: ssize_t
 */
static ssize_t stk_firlen_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    int len = atomic_read(&stk->fir_len);

    if (len)
    {
        dev_info(&stk->client->dev, "FIR count=%2d, idx=%2d\n",
                 stk->fir.count, stk->fir.idx);
        dev_info(&stk->client->dev, "sum = [\t%d \t%d \t%d]\n",
                 stk->fir.sum[0], stk->fir.sum[1], stk->fir.sum[2]);
        dev_info(&stk->client->dev, "avg = [\t%d \t%d \t%d]\n",
                 stk->fir.sum[0] / len, stk->fir.sum[1] / len, stk->fir.sum[2] / len);
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", len);
}

/*
 * @brief: Get FIR length from userspace, then write to stk832x_data.fir_len.
 *
 * @param[in] dev: struct device *
 * @param[in] attr: struct device_attribute *
 * @param[in/out] buf: char *
 * @param[in] count: size_t
 *
 * @return: ssize_t
 */
static ssize_t stk_firlen_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    int firlen, error;
    error = kstrtoint(buf, 10, &firlen);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    if (STK_FIR_LEN_MAX < firlen)
        dev_err(&stk->client->dev, "%s: maximum FIR length is %d\n",
                __func__, STK_FIR_LEN_MAX);
    else
    {
        memset(&stk->fir, 0, sizeof(struct data_fir));
        atomic_set(&stk->fir_len, firlen);
    }

    return count;
}
#endif /* STK_FIR */
////////////////////////////////////////////////////////////////////////////
#define STEPTHD_DEFAULT_VALUE 0x32
static ssize_t stk_stepthd_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
   
   	u8 data = 0;
   	if (stk832x_reg_read(stk, STK832X_REG_STEPTHD, 0, &data)) {
		data = 0;
	}
		
    return scnprintf(buf, PAGE_SIZE, "%d\n", data);
}

static ssize_t stk_stepthd_store(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t count)
{
    struct stk832x_data *stk = dev_get_drvdata(dev);
    int step, error;
    error = kstrtoint(buf, 10, &step);

    if (error)
    {
        dev_err(&stk->client->dev, "%s: kstrtoint failed, error=%d\n",
                __func__, error);
        return error;
    }

    stk832x_reg_write(stk,STK832X_REG_STEPTHD,step);

    return count;
}
/////////////////////////////////////////////////////////////////////////////////
static DEVICE_ATTR(enable, 0664, stk_enable_show, stk_enable_store);
static DEVICE_ATTR(value, 0444, stk_value_show, NULL);
static DEVICE_ATTR(delay, 0664, stk_delay_show, stk_delay_store);
static DEVICE_ATTR(cali, 0664, stk_cali_show, stk_cali_store);
static DEVICE_ATTR(offset, 0664, stk_offset_show, stk_offset_store);
static DEVICE_ATTR(send, 0220, NULL, stk_send_store);
static DEVICE_ATTR(recv, 0664, stk_recv_show, stk_recv_store);
static DEVICE_ATTR(allreg, 0444, stk_allreg_show, NULL);
static DEVICE_ATTR(chipinfo, 0444, stk_chipinfo_show, NULL);
static DEVICE_ATTR(fifo, 0664, stk_fifo_show, stk_fifo_store);
#ifdef STK_STEP_COUNTER
    static DEVICE_ATTR(stepcount, 0644, stk_step_show, stk_step_store);
#endif /* STK_STEP_COUNTER */
#ifdef STK_FIR
    static DEVICE_ATTR(firlen, 0664, stk_firlen_show, stk_firlen_store);
#endif /* STK_FIR */

static DEVICE_ATTR(stepthd, 0644, stk_stepthd_show, stk_stepthd_store);

static struct attribute *stk_attribute_accel[] =
{
    &dev_attr_enable.attr,
    &dev_attr_value.attr,
    &dev_attr_delay.attr,
    &dev_attr_cali.attr,
    &dev_attr_offset.attr,
    &dev_attr_send.attr,
    &dev_attr_recv.attr,
    &dev_attr_allreg.attr,
    &dev_attr_chipinfo.attr,
    &dev_attr_fifo.attr,
#ifdef STK_STEP_COUNTER
    &dev_attr_stepcount.attr,
#endif /* STK_STEP_COUNTER */
#ifdef STK_FIR
    &dev_attr_firlen.attr,
#endif /* STK_FIR */
	&dev_attr_stepthd.attr,
    NULL
};

static struct attribute_group stk_attribute_accel_group =
{
    .name = "driver",
    .attrs = stk_attribute_accel,
};

/*
 * define for app
 */
void fiio_start_setup() {
	stk_turn_step_counter(g_stk, true);
	stk_set_enable(g_stk, 1);
}
EXPORT_SYMBOL(fiio_start_setup);
void fiio_stop_setup() {
	stk_set_enable(g_stk, 0);
}
EXPORT_SYMBOL(fiio_stop_setup);

void fiio_restart_setup() {
	stk_set_enable(g_stk, 1);
}
EXPORT_SYMBOL(fiio_restart_setup);

int fiio_get_setup_counter() {
	stk_read_step_data(g_stk);
	return g_stk->steps;
}
EXPORT_SYMBOL(fiio_get_setup_counter);
void fiio_reinit_setup_counter() {
	stk_turn_step_counter(g_stk, true);
}
EXPORT_SYMBOL(fiio_reinit_setup_counter);

/*
 * @brief: Report accel data to /sys/class/input/inputX/capabilities/rel
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_report_accel_data(struct stk832x_data *stk)
{
#ifdef STK_CHECK_CODE

    if ((STK_CCSTATUS_XYSIM == stk->cc_status)
        || ((STK_CHECKCODE_IGNORE + 6) > stk->cc_count))
        return;

#endif /* STK_CHECK_CODE */

    if (!stk->input_dev_accel)
    {
        dev_err(&stk->client->dev,
                "%s: No input device for accel data\n", __func__);
        return;
    }

    input_report_rel(stk->input_dev_accel, REL_X, stk->xyz[0]);
    input_report_rel(stk->input_dev_accel, REL_Y, stk->xyz[1]);
    input_report_rel(stk->input_dev_accel, REL_Z, stk->xyz[2]);
    input_sync(stk->input_dev_accel);
}

#ifdef INTERRUPT_MODE
/*
 * @brief: Trigger INT_RST for latched STS
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_reset_latched_int(struct stk832x_data *stk)
{
    u8 data = 0;

    if (stk832x_reg_read(stk, STK832X_REG_INTCFG2, 0, &data))
        return;

    if (stk832x_reg_write(stk, STK832X_REG_INTCFG2,
                          (data | STK832X_INTCFG2_INT_RST)))
        return;
}

/*
 * @brief: Queue work list.
 *          1. Read accel data, then report to userspace.
 *          2. Read FIFO data.
 *          3. Read ANY MOTION data.
 *          4. Reset latch status.
 *          5. Enable IRQ.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_data_irq_work(struct work_struct *work)
{
    struct stk832x_data *stk =
        container_of(work, struct stk832x_data, alldata_work);
    stk_read_accel_data(stk);
    stk_report_accel_data(stk);
    stk_read_fifo_data(stk, true);
    stk_read_anymotion_data(stk, true);
    stk_reset_latched_int(stk);
    enable_irq(stk->irq1);
}

/*
 * @brief: IRQ handler. This function will be trigger after receiving IRQ.
 *          1. Disable IRQ without waiting.
 *          2. Send work to quque.
 *
 * @param[in] irq: irq number
 * @param[in] data: void *
 *
 * @return: IRQ_HANDLED
 */
static irqreturn_t stk_all_data_handler(int irq, void *data)
{
    struct stk832x_data *stk = data;
    disable_irq_nosync(irq);
    queue_work(stk->alldata_workqueue, &stk->alldata_work);
    return IRQ_HANDLED;
}

/*
 * @brief: IRQ setup
 *          1. Set GPIO as input direction.
 *          2. Allocate an interrupt resource, enable the interrupt, and IRQ
 *              handling.
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return:
 *          IRQC_IS_HARDIRQ or IRQC_IS_NESTED: Success
 *          Negative value: Fail
 */
static int stk_irq_alldata_setup(struct stk832x_data *stk)
{
    int irq = 0;
    gpio_direction_input(stk->interrupt_int1_pin);
    irq = gpio_to_irq(stk->interrupt_int1_pin);

    if (0 > irq)
    {
        dev_err(&stk->client->dev, "%s: gpio_to_irq(%d) failed\n",
                __func__, stk->interrupt_int1_pin);
        return -1;
    }

    stk->irq1 = irq;
    dev_info(&stk->client->dev, "%s: irq #=%d, interrupt pin=%d\n",
             __func__, irq, stk->interrupt_int1_pin);
    irq = request_any_context_irq(stk->irq1, stk_all_data_handler,
                                  IRQF_TRIGGER_RISING, STK832X_IRQ_INT1_NAME, stk);

    if (0 > irq)
    {
        dev_err(&stk->client->dev,
                "%s: request_any_context_irq(%d) failed for %d\n",
                __func__, stk->irq1, irq);
        return -1;
    }

    return irq;
}
#else /* no INTERRUPT_MODE */
/*
 * @brief: Queue delayed_work list.
 *          1. Read ANY MOTION data.
 *          2. Enable IRQ.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_sig_irq_delay_work(struct work_struct *work)
{
    struct stk832x_data *stk =
        container_of(work, struct stk832x_data, any_delaywork.work);
	fiio_debug("%s:Enter!!!\n",__func__);
    stk_read_anymotion_data(stk, false);
    enable_irq(stk->irq1);
	fiio_debug("%s:Exit!!!\n",__func__);
}

/*
 * @brief: IRQ handler. This function will be trigger after receiving IRQ.
 *          1. Disable IRQ without waiting.
 *          2. Send delayed_work to quque.
 *
 * @param[in] irq: irq number
 * @param[in] data: void *
 *
 * @return: IRQ_HANDLED
 */
static irqreturn_t stk_sig_handler(int irq, void *data)
{
    struct stk832x_data *stk = data;
	fiio_debug("%s:Enter!!!\n",__func__);
    disable_irq_nosync(irq);
    schedule_delayed_work(&stk->any_delaywork, 0);
	fiio_debug("%s:Exit!!!\n",__func__);
    return IRQ_HANDLED;
}

/*
 * @brief: IRQ setup
 *          1. Set GPIO as input direction.
 *          2. Allocate an interrupt resource, enable the interrupt, and IRQ
 *              handling.
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return:
 *          IRQC_IS_HARDIRQ or IRQC_IS_NESTED: Success
 *          Negative value: Fail
 */
static int stk_irq_sig_setup(struct stk832x_data *stk)
{
    int irq = 0;
    gpio_direction_input(stk->interrupt_int1_pin);
    irq = gpio_to_irq(stk->interrupt_int1_pin);

    if (0 > irq)
    {
        dev_err(&stk->client->dev, "%s: gpio_to_irq(%d) failed\n",
                __func__, stk->interrupt_int1_pin);
        return -1;
    }

    stk->irq1 = irq;
    dev_info(&stk->client->dev, "%s: irq #=%d, interrupt pin=%d\n",
             __func__, irq, stk->interrupt_int1_pin);
    irq = request_any_context_irq(stk->irq1, stk_sig_handler,
                                  IRQF_TRIGGER_FALLING, STK832X_IRQ_INT1_NAME, stk);

    if (0 > irq)
    {
        dev_err(&stk->client->dev,
                "%s: request_any_context_irq(%d) failed for %d\n",
                __func__, stk->irq1, irq);
        return -1;
    }

    return irq;
}

/*
 * @brief: Queue delayed_work list.
 *          1. Read FIFO data.
 *          2. Enable IRQ.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_fifo_irq_delay_work(struct work_struct *work)
{
    struct stk832x_data *stk =
        container_of(work, struct stk832x_data, fifo_delaywork.work);
    stk_read_fifo_data(stk, false);
    enable_irq(stk->irq2);
}

/*
 * @brief: IRQ handler. This function will be trigger after receiving IRQ.
 *          1. Disable IRQ without waiting.
 *          2. Send delayed_work to quque.
 *
 * @param[in] irq: irq number
 * @param[in] data: void *
 *
 * @return: IRQ_HANDLED
 */
static irqreturn_t stk_fifo_handler(int irq, void *data)
{
    struct stk832x_data *stk = data;
    disable_irq_nosync(irq);
    schedule_delayed_work(&stk->fifo_delaywork, 0);
    return IRQ_HANDLED;
}

/*
 * @brief: IRQ setup
 *          1. Set GPIO as input direction.
 *          2. Allocate an interrupt resource, enable the interrupt, and IRQ
 *              handling.
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return:
 *          IRQC_IS_HARDIRQ or IRQC_IS_NESTED: Success
 *          Negative value: Fail
 */
static int stk_irq_fifo_setup(struct stk832x_data *stk)
{
    int irq = 0;
    gpio_direction_input(stk->interrupt_int2_pin);
    irq = gpio_to_irq(stk->interrupt_int2_pin);

    if (0 > irq)
    {
        dev_err(&stk->client->dev, "%s: gpio_to_irq(%d) failed\n",
                __func__, stk->interrupt_int2_pin);
        return -1;
    }

    stk->irq2 = irq;
    dev_info(&stk->client->dev, "%s: irq #=%d, interrupt pin=%d\n",
             __func__, irq, stk->interrupt_int2_pin);
    irq = request_any_context_irq(stk->irq2, stk_fifo_handler,
                                  IRQF_TRIGGER_RISING, STK832X_IRQ_INT2_NAME, stk);

    if (0 > irq)
    {
        dev_err(&stk->client->dev,
                "%s: request_any_context_irq(%d) failed for %d\n",
                __func__, stk->irq2, irq);
        return -1;
    }

    return irq;
}

/*
 * @brief: Queue delayed_work list.
 *          1. Read accel data, then report to userspace.
 *          2. Read ANY MOTION data.
 *
 * @param[in] work: struct work_struct *
 */
static void stk_accel_delay_work(struct work_struct *work)
{
    struct stk832x_data *stk =
        container_of(work, struct stk832x_data, accel_delaywork.work);
	fiio_debug("%s:Enter!!!\n",__func__);
    stk_read_accel_data(stk);
    stk_report_accel_data(stk);
    stk_read_anymotion_data(stk, true);
	fiio_debug("%s:Exit!!!\n",__func__);
}

/*
 * @brief: This function will send delayed_work to queue.
 *          This function will be called regularly with period:
 *          stk832x_data.poll_delay.
 *
 * @param[in] timer: struct hrtimer *
 *
 * @return: HRTIMER_RESTART.
 */
static enum hrtimer_restart stk_accel_timer_func(struct hrtimer *timer)
{
    struct stk832x_data *stk =
        container_of(timer, struct stk832x_data, accel_timer);
	fiio_debug("%s:Enter!!!\n",__func__);
    schedule_delayed_work(&stk->accel_delaywork, 0);
    hrtimer_forward_now(&stk->accel_timer, stk->poll_delay);
	fiio_debug("%s:Exit!!!\n",__func__);
    return HRTIMER_RESTART;
}
#endif /* INTERRUPT_MODE */

/*
 * @brief: Initialize some data in stk832x_data.
 *
 * @param[in/out] stk: struct stk832x_data *
 */
static void stk_data_initialize(struct stk832x_data *stk)
{
    atomic_set(&stk->enabled, 0);
    atomic_set(&stk->cali_status, STK_K_NO_CALI);
    atomic_set(&stk->recv, 0);
    stk->power_mode = STK832X_PWMD_SUSPEND;
    stk->temp_enable = false;
#ifdef STK_FIR
    memset(&stk->fir, 0, sizeof(struct data_fir));
    atomic_set(&stk->fir_len, STK_FIR_LEN);
#endif /* STK_FIR */
    dev_info(&stk->client->dev, "%s: done\n", __func__);
}

/*
 * @brief: Read PID and write to stk832x_data.pid.
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_get_pid(struct stk832x_data *stk)
{
    int error = 0;
    u8 val = 0;
    error = stk832x_reg_read(stk, STK832X_REG_CHIPID, 0, &val);

    if (error)
        dev_err(&stk->client->dev,
                "%s: failed to read PID\n", __func__);
    else
        stk->pid = (int)val;

    return error;
}

/*
 * @brief: SW reset for stk832x
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_sw_reset(struct stk832x_data *stk)
{
    int error = 0;
    error = stk832x_reg_write(stk, STK832X_REG_SWRST, STK832X_SWRST_VAL);

    if (error)
        return error;

    usleep_range(1000, 2000);
    return 0;
}

/*
 * @brief: stk832x register initialize
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail.
 *          0: Success
 *          others: Fail
 */
static int stk_reg_init(struct stk832x_data *stk)
{
    int error = 0;
    /* SW reset */
    error = stk_sw_reset(stk);

    if (error)
        return error;

    ///* SUSPEND */
    //stk_set_enable(stk, 0);
    /* ID46: Low-power -> Suspend -> Normal */
    error = stk_change_power_mode(stk, STK832X_PWMD_SUSPEND);

    if (error)
        return error;

    error = stk_change_power_mode(stk, STK832X_PWMD_NORMAL);

    if (error)
        return error;

    atomic_set(&stk->enabled, 1);
    /* INT1, push-pull, active high. INT2, push-pull, activt high. */
    error = stk832x_reg_write(stk, STK832X_REG_INTCFG1,
                              STK832X_INTCFG1_INT1_ACTIVE_H | STK832X_INTCFG1_INT1_OD_PUSHPULL
                              | STK832X_INTCFG1_INT2_ACTIVE_H | STK832X_INTCFG1_INT2_OD_PUSHPULL);

    if (error)
        return error;

#ifdef INTERRUPT_MODE
    /* map any motion interrupt to int1 */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP1,
                              STK832X_INTMAP1_ANYMOT2INT1);

    if (error)
        return error;

    /* map new accel data and fifo interrupt to int1 */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP2,
                              STK832X_INTMAP2_DATA2INT1 | STK832X_INTMAP2_FWM2INT1);

    if (error)
        return error;

    /* enable new data interrupt for any motion */
    error = stk832x_reg_write(stk, STK832X_REG_INTEN1,
                              STK832X_INTEN1_SLP_EN_XYZ);

    if (error)
        return error;

    /* enable new data interrupt for both new accel data and fifo */
    error = stk832x_reg_write(stk, STK832X_REG_INTEN2,
                              STK832X_INTEN2_DATA_EN | STK832X_INTEN2_FWM_EN);

    if (error)
        return error;

    /*
     * latch int
     * In interrupt mode + significant mode, both of them share the same INT.
     * Set latched to make sure we can get ANY data(ANY_MOT_STS) before signal fall down.
     * Read ANY flow:
     * Get INT --> check INTSTS1.ANY_MOT_STS status -> INTCFG2.INT_RST(relese all latched INT)
     * Read FIFO flow:
     * Get INT --> check INTSTS2.FWM_STS status -> INTCFG2.INT_RST(relese all latched INT)
     * In latch mode, echo interrupt(SIT_MOT_STS/FWM_STS) will cause all INT(INT1/INT2)
     * rising up.
     */
    error = stk832x_reg_write(stk, STK832X_REG_INTCFG2, STK832X_INTCFG2_LATCHED);

    if (error)
        return error;

#else /* no INTERRUPT_MODE */
    /* map any motion interrupt to int1 */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP1,
                              STK832X_INTMAP1_ANYMOT2INT1);

    if (error)
        return error;

    /* map FIFO interrupt to int2 */
    error = stk832x_reg_write(stk, STK832X_REG_INTMAP2, STK832X_INTMAP2_FWM2INT2);

    if (error)
        return error;

    /* enable new data interrupt for any motion */
    error = stk832x_reg_write(stk, STK832X_REG_INTEN1, STK832X_INTEN1_SLP_EN_XYZ);

    if (error)
        return error;

    /* enable new data interrupt for FIFO only */
    error = stk832x_reg_write(stk, STK832X_REG_INTEN2, STK832X_INTEN2_FWM_EN);

    if (error)
        return error;

    /* non-latch int */
    error = stk832x_reg_write(stk, STK832X_REG_INTCFG2, STK832X_INTCFG2_NOLATCHED);

    if (error)
        return error;

#endif /* INTERRUPT_MODE */
    /* SLOPE DELAY */
    error = stk832x_reg_write(stk, STK832X_REG_SLOPEDLY, 0x00);

    if (error)
        return error;

    /* SLOPE THRESHOLD */
    error = stk832x_reg_write(stk, STK832X_REG_SLOPETHD, STK832X_SLOPETHD_DEF);

    if (error)
        return error;

    /* SIGMOT1 */
    error = stk832x_reg_write(stk, STK832X_REG_SIGMOT1,
                              STK832X_SIGMOT1_SKIP_TIME_3SEC);

    if (error)
        return error;

    /* SIGMOT2 */
    error = stk832x_reg_write(stk, STK832X_REG_SIGMOT2,
                              STK832X_SIGMOT2_ANY_MOT_EN);

    if (error)
        return error;

    /* SIGMOT3 */
    error = stk832x_reg_write(stk, STK832X_REG_SIGMOT3,
                              STK832X_SIGMOT3_PROOF_TIME_1SEC);

    if (error)
        return error;

    /* According to STK_DEF_DYNAMIC_RANGE */
    error = stk_range_selection(stk, STK832X_RANGESEL_DEF);

    if (error)
        return error;

    /* ODR */
    error = stk832x_reg_write(stk, STK832X_REG_BWSEL, STK832X_BWSEL_INIT_ODR);

    if (error)
        return error;

    stk_change_fifo_status(stk, 0);
    /* i2c watchdog enable */
    error = stk832x_reg_write(stk, STK832X_REG_INTFCFG,
                              STK832X_INTFCFG_I2C_WDT_EN);

    if (error)
        return error;
    //设置STK832X_REG_STEPDEB 5step
    error = stk832x_reg_write(stk, STK832X_REG_STEPDEB,4);
    if (error)
        return error;

    //STK832X_REG_STEPMAXTW 2s
    error = stk832x_reg_write(stk, STK832X_REG_STEPMAXTW,0xfa);
    if (error)
        return error;

    /* SUSPEND */
    //stk_set_enable(stk, 0);
    error = stk_change_power_mode(stk, STK832X_PWMD_SUSPEND);
    if (error)
        return error;


    atomic_set(&stk->enabled, 0);
    return 0;
}

/*
 * @brief: File system setup for accel and any motion
 *
 * @param[in/out] stk: struct stk832x_data *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk_input_setup(struct stk832x_data *stk)
{
    int error = 0;
    /* input device: setup for accel */
    stk->input_dev_accel = input_allocate_device();

    if (!stk->input_dev_accel)
    {
        dev_err(&stk->client->dev,
                "%s: input_allocate_device for accel failed\n", __func__);
        return -ENOMEM;
    }

    stk->input_dev_accel->name = IN_DEV_ACCEL_NAME;
    stk->input_dev_accel->id.bustype = BUS_I2C;
    input_set_capability(stk->input_dev_accel, EV_REL, REL_X);
    input_set_capability(stk->input_dev_accel, EV_REL, REL_Y);
    input_set_capability(stk->input_dev_accel, EV_REL, REL_Z);
    input_set_drvdata(stk->input_dev_accel, stk);
    error = input_register_device(stk->input_dev_accel);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: Unable to register input device: %s\n",
                __func__, stk->input_dev_accel->name);
        input_free_device(stk->input_dev_accel);
        return error;
    }

    /* input device: setup for any motion */
    stk->input_dev_any = input_allocate_device();

    if (!stk->input_dev_any)
    {
        dev_err(&stk->client->dev,
                "%s: input_allocate_device for ANY MOTION failed\n", __func__);
        input_free_device(stk->input_dev_accel);
        input_unregister_device(stk->input_dev_accel);
        return -ENOMEM;
    }

    stk->input_dev_any->name = IN_DEV_ANY_NAME;
    stk->input_dev_any->id.bustype = BUS_I2C;
    input_set_capability(stk->input_dev_any, EV_ABS, ABS_MISC);
    input_set_drvdata(stk->input_dev_any, stk);
	input_set_abs_params(stk->input_dev_any, ABS_MISC, 0, 1, 0, 0);
    error = input_register_device(stk->input_dev_any);

    if (error)
    {
        dev_err(&stk->client->dev,
                "%s: Unable to register input device: %s\n",
                __func__, stk->input_dev_any->name);
        input_free_device(stk->input_dev_any);
        input_free_device(stk->input_dev_accel);
        input_unregister_device(stk->input_dev_accel);
        return error;
    }

    return 0;
}

/*
 * @brief: Proble function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 * @param[in] id: struct i2c_device_id *
 *
 * @return: Success or fail
 *          0: Success
 *          others: Fail
 */
static int stk832x_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int error = 0;
    struct stk832x_data *stk;
	fiio_debug("%s:Enter driver version:%s!!!\n",__func__,STK_ACC_DRIVER_VERSION);
    dev_info(&client->dev, "%s: driver version:%s\n",
             __func__, STK_ACC_DRIVER_VERSION);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        error = i2c_get_functionality(client->adapter);
        dev_err(&client->dev,
                "%s: i2c_check_functionality error, functionality=0x%x\n",
                __func__, error);
        return -EIO;
    }

    /* kzalloc: allocate memory and set to zero. */
    stk = kzalloc(sizeof(struct stk832x_data), GFP_KERNEL);

    if (!stk)
    {
        dev_err(&client->dev, "%s: memory allocation error\n", __func__);
        return -ENOMEM;
    }

    stk->client = client;
    i2c_set_clientdata(client, stk);
    mutex_init(&stk->reg_lock);
	g_stk = stk;
	
    if (get_platform_data(stk))
        goto err_free_mem;

    if (stk_get_pid(stk))
        goto err_free_mem;

    dev_info(&client->dev, "%s: PID 0x%x\n", __func__, stk->pid);
#ifdef INTERRUPT_MODE

    if (gpio_request(stk->interrupt_int1_pin, STK832X_IRQ_INT1_LABEL))
    {
        dev_err(&client->dev, "%s: gpio_request failed\n", __func__);
        goto err_free_mem;
    }

    stk->alldata_workqueue = create_singlethread_workqueue("stk_int1_wq");

    if (stk->alldata_workqueue)
        INIT_WORK(&stk->alldata_work, stk_data_irq_work);
    else
    {
        dev_err(&stk->client->dev, "%s: create_singlethread_workqueue error\n",
                __func__);
        error = -EPERM;
        goto exit_int1_create_singlethread_workqueue;
    }

    error = stk_irq_alldata_setup(stk);

    if (0 > error)
        goto exit_irq_int1_setup_error;

#else /* no INTERRUPT_MODE */

    /* ANYMOTION */
    if (gpio_request(stk->interrupt_int1_pin, STK832X_IRQ_INT1_LABEL))
    {
        dev_err(&stk->client->dev, "%s: gpio_request failed\n", __func__);
        goto err_free_mem;
    }

    INIT_DELAYED_WORK(&stk->any_delaywork, stk_sig_irq_delay_work);
    error = stk_irq_sig_setup(stk);

    if (0 > error)
        goto exit_irq_int1_setup_error;

    /* FIFO */
    if (gpio_request(stk->interrupt_int2_pin, STK832X_IRQ_INT2_LABEL))
    {
        dev_err(&stk->client->dev, "%s: gpio_request failed\n", __func__);
        goto exit_int2_gpio_request;
    }

    INIT_DELAYED_WORK(&stk->fifo_delaywork, stk_fifo_irq_delay_work);
    error = stk_irq_fifo_setup(stk);

    if (0 > error)
        goto exit_irq_int2_setup_error;

    /* polling accel data */
    INIT_DELAYED_WORK(&stk->accel_delaywork, stk_accel_delay_work);
    hrtimer_init(&stk->accel_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    stk->poll_delay = ns_to_ktime(
                          STK832X_SAMPLE_TIME[STK832X_BWSEL_INIT_ODR - STK832X_SPTIME_BASE]
                          * NSEC_PER_USEC);
	fiio_debug("%s delay:%ld\n",__func__,STK832X_SAMPLE_TIME[STK832X_BWSEL_INIT_ODR - STK832X_SPTIME_BASE] * NSEC_PER_USEC);//16000000ns
    stk->accel_timer.function = stk_accel_timer_func;
	
#endif /* INTERRUPT_MODE */
    stk_data_initialize(stk);

    if (stk_reg_init(stk))
    {
        dev_err(&client->dev, "%s: stk832x initialization failed\n", __func__);
        goto exit_stk_init_error;
    }

    if (stk_input_setup(stk))
    {
        dev_err(&client->dev, "%s: failed\n", __func__);
        goto exit_stk_init_error;
    }

    /* sysfs: create file system */
    error = sysfs_create_group(&stk->input_dev_accel->dev.kobj,
                               &stk_attribute_accel_group);

    if (error)
    {
        dev_err(&client->dev, "%s: sysfs_create_group failed for accel\n",
                __func__);
        goto exit_sysfs_create_accelgroup_error;
    }

    dev_info(&client->dev, "%s: Successfully\n", __func__);
	fiio_debug("%s:Exit!!!\n",__func__);
    return 0;
exit_sysfs_create_accelgroup_error:
    input_unregister_device(stk->input_dev_any);
    input_unregister_device(stk->input_dev_accel);
exit_stk_init_error:
#ifdef INTERRUPT_MODE
    free_irq(stk->irq1, stk);
exit_irq_int1_setup_error:
    cancel_work_sync(&stk->alldata_work);
    destroy_workqueue(stk->alldata_workqueue);
exit_int1_create_singlethread_workqueue:
    gpio_free(stk->interrupt_int1_pin);
#else /* no INTERRUPT_MODE */
    hrtimer_try_to_cancel(&stk->accel_timer);
    cancel_delayed_work_sync(&stk->accel_delaywork);
    free_irq(stk->irq2, stk);
exit_irq_int2_setup_error:
    cancel_delayed_work_sync(&stk->fifo_delaywork);
    gpio_free(stk->interrupt_int2_pin);
exit_int2_gpio_request:
    free_irq(stk->irq1, stk);
exit_irq_int1_setup_error:
    cancel_delayed_work_sync(&stk->any_delaywork);
    gpio_free(stk->interrupt_int1_pin);
#endif /* INTERRUPT_MODE */
err_free_mem:
    mutex_destroy(&stk->reg_lock);
    kfree(stk);
    return error;
}

/*
 * @brief: Remove function for i2c_driver.
 *
 * @param[in] client: struct i2c_client *
 *
 * @return: 0
 */
static int stk832x_remove(struct i2c_client *client)
{
    struct stk832x_data *stk = i2c_get_clientdata(client);
    sysfs_remove_group(&stk->input_dev_accel->dev.kobj,
                       &stk_attribute_accel_group);
    input_unregister_device(stk->input_dev_any);
    input_unregister_device(stk->input_dev_accel);
#ifdef INTERRUPT_MODE
    free_irq(stk->irq1, stk);
    cancel_work_sync(&stk->alldata_work);
    destroy_workqueue(stk->alldata_workqueue);
    gpio_free(stk->interrupt_int1_pin);
#else /* no INTERRUPT_MODE */
    hrtimer_try_to_cancel(&stk->accel_timer);
    cancel_delayed_work_sync(&stk->accel_delaywork);
    free_irq(stk->irq2, stk);
    cancel_delayed_work_sync(&stk->fifo_delaywork);
    gpio_free(stk->interrupt_int2_pin);
    free_irq(stk->irq1, stk);
    cancel_delayed_work_sync(&stk->any_delaywork);
    gpio_free(stk->interrupt_int1_pin);
#endif /* INTERRUPT_MODE */
    mutex_destroy(&stk->reg_lock);
    kfree(stk);
    stk = NULL;
    return 0;
}

#ifdef CONFIG_PM_SLEEP
/*
 * @brief: Suspend function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk832x_suspend(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct stk832x_data *stk = i2c_get_clientdata(client);
#ifndef FIIO_SUSPEND_STEP_RUNNING
    if (atomic_read(&stk->enabled))
    {
        stk_set_enable(stk, 0);
        stk->temp_enable = true;
    }
    else
        stk->temp_enable = false;
#endif
    return 0;
}

/*
 * @brief: Resume function for dev_pm_ops.
 *
 * @param[in] dev: struct device *
 *
 * @return: 0
 */
static int stk832x_resume(struct device *dev)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct stk832x_data *stk = i2c_get_clientdata(client);
#ifndef FIIO_SUSPEND_STEP_RUNNING
    if (stk->temp_enable)
        stk_set_enable(stk, 1);

    stk->temp_enable = false;
#endif
    return 0;
}

static const struct dev_pm_ops stk832x_pm_ops =
{
    .suspend = stk832x_suspend,
    .resume = stk832x_resume,
};
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_ACPI
static const struct acpi_device_id stk832x_acpi_id[] =
{
    {"STK832X", 0},
    {}
};
MODULE_DEVICE_TABLE(acpi, stk832x_acpi_id);
#endif /* CONFIG_ACPI */

#ifdef CONFIG_OF
static struct of_device_id stk832x_match_table[] =
{
    { .compatible = "stk,stk832x", },
    {}
};
#endif /* CONFIG_OF */

static const struct i2c_device_id stk832x_i2c_id[] =
{
    {STK832X_I2C_NAME, 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, stk832x_i2c_id);

static struct i2c_driver stk832x_i2c_driver =
{
    .probe      = stk832x_probe,
    .remove     = stk832x_remove,
    .id_table   = stk832x_i2c_id,
    .class      = I2C_CLASS_HWMON,
    .driver = {
        .owner  = THIS_MODULE,
        .name   = STK832X_I2C_NAME,
#ifdef CONFIG_PM_SLEEP
        .pm     = &stk832x_pm_ops,
#endif
#ifdef CONFIG_ACPI
        .acpi_match_table = ACPI_PTR(stk832x_acpi_id),
#endif /* CONFIG_ACPI */
#ifdef CONFIG_OF
        .of_match_table = stk832x_match_table,
#endif /* CONFIG_OF */
    }
};

module_i2c_driver(stk832x_i2c_driver);

MODULE_AUTHOR("Sensortek");
MODULE_DESCRIPTION("stk832x 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(STK_ACC_DRIVER_VERSION);

