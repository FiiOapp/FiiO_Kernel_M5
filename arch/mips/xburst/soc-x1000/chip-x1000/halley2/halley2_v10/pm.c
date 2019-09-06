/*
 * Copyright (c) 2006-2010  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <gpio.h>
#if 0
//#define FIIO_M5_GPIO_INIT
// default gpio state is input pull;
__initdata int gpio_ss_table[][2] = {
	{32*0+0,	GSS_OUTPUT_LOW	},	/* SLCD_D0 */
	{32*0+1,	GSS_OUTPUT_LOW	},	/* SLCD_D1 */
	{32*0+2,	GSS_OUTPUT_LOW	},	/* SLCD_D2 */
	{32*0+3,	GSS_OUTPUT_LOW	},	/* SLCD_D3 */
	{32*0+4,	GSS_OUTPUT_LOW	},	/* SLCD_D4 */
	{32*0+5,	GSS_OUTPUT_LOW	},	/* SLCD_D5 */
	{32*0+6,	GSS_OUTPUT_LOW	},	/* SLCD_D6 */
	{32*0+7,	GSS_OUTPUT_LOW	},	/* SLCD_D7 */
	{32*0+8,	GSS_OUTPUT_LOW	},	/* SLCD_D8 */
	{32*0+9,	GSS_OUTPUT_LOW	},	/* SLCD_D9 */
	{32*0+10,	GSS_OUTPUT_LOW	},	/* SLCD_D10 */
	{32*0+11,	GSS_OUTPUT_LOW	},	/* SLCD_D11 */
	{32*0+12,	GSS_OUTPUT_LOW	},	/* SLCD_D12 */
	{32*0+13,	GSS_OUTPUT_LOW	},	/* SLCD_D13 */
	{32*0+14,	GSS_OUTPUT_LOW	},	/* SLCD_D14 */
	{32*0+15,	GSS_OUTPUT_LOW	},	/* SLCD_D15 */
	//begin modify by pengweizhong
	#ifdef FIIO_M5_GPIO_INIT
	{32*0+16,	GSS_OUTPUT_LOW	},	/* PO_EN */
	#else
	//N
	{32*0+16,	GSS_IGNORE	},
	//{32*0+16,	GSS_INPUT_NOPULL	},
	#endif
	//end
	{32*0+17,	GSS_INPUT_PULL	},	/* CIM_D2 */
	{32*0+18,	GSS_OUTPUT_LOW	},	/* CIM_D1 */
	#ifdef FIIO_M5_GPIO_INIT
	{32*0+19,	GSS_INPUT_PULL	},	/* VOL+ */
	#else
	{32*0+19,	GSS_INPUT_NOPULL	},	/* VOL+ */
	#endif
	{32*0+20,	GSS_OUTPUT_LOW	},	/* MSC0_D3 */
	{32*0+21,	GSS_OUTPUT_LOW	},	/* MSC0_D2 */
	{32*0+22,	GSS_OUTPUT_LOW	},	/* MSC0_D1 */
	{32*0+23,	GSS_OUTPUT_LOW	},	/* MSC0_D0 */
	{32*0+24,	GSS_OUTPUT_LOW	},	/* MSC0_CLK */
	{32*0+25,	GSS_OUTPUT_LOW	},	/* MSC0_CMD */
	{32*0+26,	GSS_INPUT_PULL	},	/* SFC_CLK */
	{32*0+27,	GSS_INPUT_PULL	},	/* SFC_CE */
	{32*0+28,	GSS_INPUT_PULL	},	/* SFC_DR */
	{32*0+29,	GSS_INPUT_PULL	},	/* SFC_DT */
	{32*0+30,	GSS_INPUT_PULL	},	/* SFC_WP */
	{32*0+31,	GSS_INPUT_PULL	},	/* SFC_HOL */

	//modify by pengweizhong
	#ifdef FIIO_M5_GPIO_INIT
	 //{32*1+0,	GSS_INPUT_PULL	},	/* I2S_MCLK */
	 //{32*1+1,	GSS_INPUT_PULL	},	/* I2S_BCLK */
	 //{32*1+2,	GSS_INPUT_PULL	},	/* I2S_LRCK */
	 //{32*1+3,	GSS_INPUT_PULL	},	/* I2S_DI */
	 //{32*1+4,	GSS_INPUT_PULL	},	/* I2S_DO */

	 {32*1+0,	GSS_INPUT_NOPULL	},	/* I2S_MCLK */
	 {32*1+1,	GSS_INPUT_NOPULL	},	/* I2S_BCLK */
	 {32*1+2,	GSS_INPUT_NOPULL	},	/* I2S_LRCK */
	 {32*1+3,	GSS_INPUT_NOPULL	},	/* I2S_DI */
	 {32*1+4,	GSS_INPUT_NOPULL	},	/* I2S_DO */
	#else
	//must set to GSS_IGNORE
	#if 0
	{32*1+0,	GSS_IGNORE	},	/* I2S_MCLK */
	{32*1+1,	GSS_IGNORE	},	/* I2S_BCLK */
	{32*1+2,	GSS_IGNORE	},	/* I2S_LRCK */
	{32*1+3,	GSS_IGNORE	},	/* I2S_DI */
	{32*1+4,	GSS_IGNORE	},	/* I2S_DO */
	#endif
    //normal suspend
    {32*1+0,	GSS_OUTPUT_LOW	},	/* I2S_MCLK */
    {32*1+1,	GSS_OUTPUT_LOW	},	/* I2S_BCLK */
    {32*1+2,	GSS_OUTPUT_LOW	},	/* I2S_LRCK */
    {32*1+3,	GSS_OUTPUT_LOW	},	/* I2S_DI */
    {32*1+4,	GSS_OUTPUT_LOW	},	/* I2S_DO */
	#endif
	//end
	//LED
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+5,	GSS_INPUT_PULL	},	/* LED0 */
	#else
    {32*1+5,	GSS_INPUT_NOPULL	},	/* LED0 */
	#endif
	
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+6,	GSS_INPUT_PULL	},	/* MSC0 DET */
	#else
	{32*1+6,	GSS_INPUT_NOPULL	},	/* MSC0 DET */
	#endif
	
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+7,	GSS_INPUT_PULL	},	/* INT_N */
	#else
	{32*1+7,	GSS_INPUT_NOPULL	},	/* INT_N */
	#endif

	#ifdef FIIO_M5_GPIO_INIT
	{32*1+8,	GSS_INPUT_PULL	},	/* USB POWER ON */
	#else
	{32*1+8,	GSS_INPUT_NOPULL	},	/* USB POWER ON */
	#endif
	
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+9,	GSS_INPUT_PULL	},	/* USB ID */
	#else
	{32*1+9,	GSS_INPUT_NOPULL	},	/*  USB ID  */
	#endif
	
	{32*1+10,	GSS_INPUT_PULL	},	/* MAC_TXEN */
	
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+11,	GSS_OUTPUT_LOW	},	/* USB DET */
	#else
	{32*1+11,	GSS_INPUT_NOPULL	},	/* USB DET */ 
	#endif
	
	{32*1+12,	GSS_OUTPUT_LOW	},	/* MAC_TXD0 */
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+13,	GSS_OUTPUT_LOW	},	/* OTG_EN */
	#else
	{32*1+13,	GSS_INPUT_NOPULL	},	/* OTG_EN */
	#endif
	{32*1+14,	GSS_OUTPUT_LOW	},	/* MAC_MDIO */
	{32*1+15,	GSS_OUTPUT_LOW	},	/* MAC_REF_CLK */
	{32*1+16,	GSS_OUTPUT_LOW	},	/* SLCD_RD */
	{32*1+17,	GSS_OUTPUT_LOW	},	/* SLCD_WR */
	{32*1+18,	GSS_OUTPUT_LOW	},	/* SLCD_CE */
	{32*1+19,	GSS_OUTPUT_LOW	},	/* SLCD_TE */
	{32*1+20,	GSS_OUTPUT_LOW	},	/* SLCD_DC */
	
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+21,	GSS_OUTPUT_LOW	},	/* LED2 */
	{32*1+22,	GSS_OUTPUT_LOW	},	/* LED1 */
	#else
	//减少4ms电流
    {32*1+21,	GSS_INPUT_NOPULL	},	/* LED2 */
    {32*1+22,	GSS_INPUT_NOPULL	},	/* LED1 */
	#endif
	
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+23,	GSS_INPUT_PULL	},	/* SMB0_SCK */
	{32*1+24,	GSS_INPUT_PULL	},	/* SMB0_SDA */
	#else
	{32*1+23,	GSS_INPUT_NOPULL	},	/* SMB0_SCK *///avoid csr8675 can't find ak4377
	{32*1+24,	GSS_INPUT_NOPULL	},	/* SMB0_SDA */
	#endif
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+25,	GSS_OUTPUT_LOW	},	/* DRVVBUS */
	#else
	{32*1+25,	GSS_INPUT_NOPULL	},	/* DRVVBUS */
	#endif
	
	{32*1+26,	GSS_OUTPUT_LOW	},	/* CLK32K */
	{32*1+27,	GSS_OUTPUT_LOW	},	/* EXCLK */
	{32*1+28,	GSS_INPUT_NOPULL},	/* BOOT_SEL0 */
	{32*1+29,	GSS_INPUT_NOPULL},	/* BOOT_SEL1 */
	{32*1+30,	GSS_INPUT_NOPULL},	/* BOOT_SEL2 */
	//begin modify by pengweizhong
	#ifdef FIIO_M5_GPIO_INIT
	{32*1+31,	GSS_INPUT_NOPULL	},	/* WAKEUP */
	#else
	//N
	{32*1+31,	GSS_INPUT_NOPULL	},
	#endif
	//end

	//modify by pengweizhong
	//{32*2+0,	GSS_OUTPUT_LOW      },	/* BT_INT1 */
	{32*2+0,	GSS_INPUT_NOPULL      },
	{32*2+1,	GSS_OUTPUT_LOW      },	/* MSC1_CMD */
	
	//modify by pengweizhong
	//{32*2+2,	GSS_OUTPUT_LOW      },	/* BT_INT2 */
	{32*2+2,	GSS_INPUT_NOPULL      },
	
	//begin modify by pengweizhong
	#ifdef FIIO_M5_GPIO_INIT
	{32*2+3,	GSS_OUTPUT_LOW      },	/* BT_RST */
	#else
	//{32*2+3,	GSS_IGNORE      },
	{32*2+3,	GSS_OUTPUT_HIGH      },
	#endif
	//end
	{32*2+4,	GSS_OUTPUT_LOW      },	/* MSC1_D2 */
	{32*2+5,	GSS_OUTPUT_LOW      },	/* MSC1_D3 */
	{32*2+6,	GSS_OUTPUT_LOW	},	/* PCM_CLK */
	{32*2+7,	GSS_OUTPUT_LOW	},	/* PCM_DO */
	{32*2+8,	GSS_OUTPUT_LOW	},	/* PCM_DI */
	{32*2+9,	GSS_OUTPUT_LOW	},	/* PCM_SYN */
	//begin modify by pengweizhong
	#ifdef FIIO_M5_GPIO_INIT
 	//{32*2+10,	GSS_OUTPUT_LOW	},	/* UART0_RXD */
	//{32*2+11,	GSS_OUTPUT_LOW	},	/* UART0_TXD */
	//{32*2+12,	GSS_OUTPUT_LOW	},	/* UART0_CTS_N */
	//{32*2+13,	GSS_OUTPUT_LOW	},	/* UART0_RTS_N */
	{32*2+10,	GSS_INPUT_NOPULL	},	/* UART0_RXD */
	{32*2+11,	GSS_INPUT_NOPULL	},	/* UART0_TXD */
	{32*2+12,	GSS_INPUT_NOPULL	},	/* UART0_CTS_N */
	{32*2+13,	GSS_INPUT_NOPULL	},	/* UART0_RTS_N */
	#else
	{32*2+10,	GSS_IGNORE	},	/* UART0_RXD */
	{32*2+11,	GSS_IGNORE	},	/* UART0_TXD */
	{32*2+12,	GSS_IGNORE	},	/* UART0_CTS_N */
	{32*2+13,	GSS_IGNORE	},	/* UART0_RTS_N */
	#endif
	//end
	{32*2+16,	GSS_OUTPUT_LOW      },	/* WL_WAKE_HOST */
	{32*2+17,	GSS_OUTPUT_LOW	},	/* WL_REG_EN */
	{32*2+18,	GSS_OUTPUT_LOW	},	/* BT_REG_EN */
	//modify by pegnweizhong
	#ifdef FIIO_M5_GPIO_INIT
	{32*2+19,	GSS_OUTPUT_LOW      },	/* HOST_WAKE_BT */
	#else
	{32*2+19,	GSS_IGNORE      },
	#endif
	//end
	{32*2+20,	GSS_OUTPUT_LOW      },	/* BT_WAKE_HOST */
	#ifdef FIIO_M5_GPIO_INIT
	{32*2+21,	GSS_OUTPUT_LOW  },	/* csr8675 connect  */
	{32*2+22,	GSS_OUTPUT_LOW  },	/* X1000 connect usb*/
	#else
	//N
	{32*2+21,	GSS_INPUT_NOPULL  },	/* csr8675 connect  */
	{32*2+22,	GSS_INPUT_NOPULL  },	/* X1000 connect usb*/
	#endif
	#ifdef FIIO_M5_GPIO_INIT
	{32*2+23,	GSS_OUTPUT_LOW	},	/* BT_VREG_ON */
	#else
	//{32*2+23,	GSS_IGNORE	},	/* BT_VREG_ON */
	{32*2+23,	GSS_OUTPUT_LOW	},	/* BT_VREG_ON */
	#endif
	
	
	{32*2+24,	GSS_OUTPUT_LOW	},	/* USB_DETE */
	{32*2+25,	GSS_OUTPUT_LOW	},	/* LCD_PWM */
	
	
	#ifdef FIIO_M5_GPIO_INIT
	{32*2+26,	GSS_INPUT_PULL	},	/* SMB1_SCk */ //???
	{32*2+27,	GSS_INPUT_PULL	},	/* SMB1_SDA */ //???
	#else
	{32*2+26,	GSS_INPUT_NOPULL	},	/* SMB1_SCk */ //???
	{32*2+27,	GSS_INPUT_NOPULL	},	/* SMB1_SDA */ //???
	#endif
	{32*2+28,	GSS_IGNORE	},	/* SMB1_SDA */
	{32*2+29,	GSS_IGNORE	},	/* SMB1_SDA */
	{32*2+30,	GSS_IGNORE	},	/* SMB1_SDA */
	{32*2+31,	GSS_IGNORE	},	/* uart2 */
#if 0
	//deefault
	{32*3+0,	GSS_INPUT_PULL	},	/* SSI0_CLK */
	{32*3+1,	GSS_INPUT_PULL	},	/* SSI0_CE0 */
	{32*3+2,	GSS_OUTPUT_LOW	},	/* SSI0_DT */
	//modify by pengweizhong
	#ifdef FIIO_M5_GPIO_INIT
	{32*3+3,	GSS_OUTPUT_LOW	},	/* SSI0_DR */
	#else
	{32*3+3,	GSS_IGNORE	},	/* SSI0_DR */
	#endif
	//end
	{32*3+4,	GSS_OUTPUT_LOW	},	/* UART2_TXD *///MMA8653 INT2
	{32*3+5,	GSS_OUTPUT_LOW	},	/* UART2_RXD *///MMA8653 INT1
#endif
#if 1
	//default
	//{32*3+0,	GSS_INPUT_PULL	},	/* SSI0_CLK */
	//{32*3+1,	GSS_INPUT_PULL	},	/* SSI0_CE0 *///9mA
	
	//{32*3+0,	GSS_OUTPUT_LOW	},	/* SSI0_CLK */
	//{32*3+1,	GSS_OUTPUT_LOW	},	/* SSI0_CE0 *///18mA

	//{32*3+0,	GSS_INPUT_NOPULL	},	/* SSI0_CLK */
	//{32*3+1,	GSS_INPUT_NOPULL	},	/* SSI0_CE0 *///9mA

#ifdef FIIO_M5_GPIO_INIT
	{32*3+0,	GSS_IGNORE	},	/* SMB2_SCK */
	{32*3+1,	GSS_IGNORE	},	/* SMB2_SDA */
#else
	//No power
	{32*3+0,	GSS_INPUT_NOPULL	},	/* SMB2_SCK */
	{32*3+1,	GSS_INPUT_NOPULL	},	/* SMB2_SDA */
#endif
	{32*3+2,	GSS_OUTPUT_LOW	},	/* SSI0_DT */
	//modify by pengweizhong
	//spdif
	#ifdef FIIO_M5_GPIO_INIT
	{32*3+3,	GSS_OUTPUT_LOW	},	/* SSI0_DR */
	#else
	//use GSS_IGNORE may have 1mA power,so how to do?
    //bt sink
    {32*3+3,	GSS_IGNORE	},	/* SPDIF/PO_SEL */
    //normal
    //{32*3+3,	GSS_INPUT_NOPULL	},	/* SPDIF/PO_SEL */
	#endif
	//end
	{32*3+4,	GSS_INPUT_NOPULL	},	/* UART2_TXD *///MMA8653 INT2
	{32*3+5,	GSS_INPUT_NOPULL	},	/* UART2_RXD *///MMA8653 INT1
#endif
	{GSS_TABLET_END,GSS_TABLET_END	}	/* GPIO Group Set End */
};
#endif
#if 0
//内核烧录到Ｍ３Ｋ的板子，休眠功耗0.680ｍＡ
__initdata int gpio_ss_table[][2] = {
    {32*0+0,	GSS_OUTPUT_LOW	},	/* SLCD_D0 */
    {32*0+1,	GSS_OUTPUT_LOW	},	/* SLCD_D1 */
    {32*0+2,	GSS_OUTPUT_LOW	},	/* SLCD_D2 */
    {32*0+3,	GSS_OUTPUT_LOW	},	/* SLCD_D3 */
    {32*0+4,	GSS_OUTPUT_LOW	},	/* SLCD_D4 */
    {32*0+5,	GSS_OUTPUT_LOW	},	/* SLCD_D5 */
    {32*0+6,	GSS_OUTPUT_LOW	},	/* SLCD_D6 */
    {32*0+7,	GSS_OUTPUT_LOW	},	/* SLCD_D7 */
    {32*0+8,	GSS_OUTPUT_LOW	},	/* SLCD_D8 */
    {32*0+9,	GSS_OUTPUT_LOW	},	/* SLCD_D9 */
    {32*0+10,	GSS_OUTPUT_LOW	},	/* SLCD_D10 */
    {32*0+11,	GSS_OUTPUT_LOW	},	/* SLCD_D11 */
    {32*0+12,	GSS_OUTPUT_LOW	},	/* SLCD_D12 */
    {32*0+13,	GSS_OUTPUT_LOW	},	/* SLCD_D13 */
    {32*0+14,	GSS_OUTPUT_LOW	},	/* SLCD_D14 */
    {32*0+15,	GSS_OUTPUT_LOW	},	/* SLCD_D15 */
    {32*0+16,	GSS_OUTPUT_LOW	},	/* CIM_D3 */
    {32*0+17,	GSS_INPUT_PULL	},	/* CIM_D2 */
    {32*0+18,	GSS_OUTPUT_LOW	},	/* CIM_D1 */
    {32*0+19,	GSS_INPUT_PULL	},	/* CIM_D0 */

    //SD
    #if 1
    {32*0+20,	GSS_OUTPUT_LOW	},	/* MSC0_D3 */
    {32*0+21,	GSS_OUTPUT_LOW	},	/* MSC0_D2 */
    {32*0+22,	GSS_OUTPUT_LOW	},	/* MSC0_D1 */
    {32*0+23,	GSS_OUTPUT_LOW	},	/* MSC0_D0 */
    {32*0+24,	GSS_OUTPUT_LOW	},	/* MSC0_CLK */
    {32*0+25,	GSS_OUTPUT_LOW	},	/* MSC0_CMD */
    #else
    {32*0+20,	GSS_INPUT_NOPULL	},	/* MSC0_D3 */
    {32*0+21,	GSS_INPUT_NOPULL	},	/* MSC0_D2 */
    {32*0+22,	GSS_INPUT_NOPULL	},	/* MSC0_D1 */
    {32*0+23,	GSS_INPUT_NOPULL	},	/* MSC0_D0 */
    {32*0+24,	GSS_INPUT_NOPULL	},	/* MSC0_CLK */
    {32*0+25,	GSS_INPUT_NOPULL	},	/* MSC0_CMD */
    #endif

    {32*0+26,	GSS_INPUT_PULL	},	/* SFC_CLK */
    {32*0+27,	GSS_INPUT_PULL	},	/* SFC_CE */
    {32*0+28,	GSS_INPUT_PULL	},	/* SFC_DR */
    {32*0+29,	GSS_INPUT_PULL	},	/* SFC_DT */
    {32*0+30,	GSS_INPUT_PULL	},	/* SFC_WP */
    {32*0+31,	GSS_INPUT_PULL	},	/* SFC_HOL */


    {32*1+0,	GSS_INPUT_PULL	},	/* I2S_MCLK */
    {32*1+1,	GSS_INPUT_PULL	},	/* I2S_BCLK */
    {32*1+2,	GSS_INPUT_PULL	},	/* I2S_LRCK */
    {32*1+3,	GSS_INPUT_PULL	},	/* I2S_DI */
    {32*1+4,	GSS_INPUT_PULL	},	/* I2S_DO */
    {32*1+5,	GSS_INPUT_PULL	},	/* DMIC_IN1 */

    //MSC0_DET
    //{32*1+6,	GSS_INPUT_PULL	},	/* MAC_PHY_CLK */
    {32*1+6,	GSS_INPUT_NOPULL},	/* MAC_PHY_CLK */

    //USB ID
    #if 0
    {32*1+7,	GSS_INPUT_PULL	},	/* MAC_CRS_DV */
    #else
    {32*1+7,	GSS_INPUT_NOPULL	},
    #endif

    {32*1+8,	GSS_INPUT_PULL	},	/* MAC_RXD1 */
    {32*1+9,	GSS_INPUT_PULL	},	/* MAC_RXD0 */
    {32*1+10,	GSS_INPUT_PULL	},	/* MAC_TXEN */

    //USB_DET
    #if 0
    {32*1+11,	GSS_OUTPUT_LOW	},	/* MAC_TXD1 */ //???
    #else
    {32*1+11,	GSS_INPUT_NOPULL},
    #endif

    {32*1+12,	GSS_OUTPUT_LOW	},	/* MAC_TXD0 */
    {32*1+13,	GSS_OUTPUT_LOW	},	/* MAC_MDC */
    {32*1+14,	GSS_OUTPUT_LOW	},	/* MAC_MDIO */
    {32*1+15,	GSS_OUTPUT_LOW	},	/* MAC_REF_CLK */
    {32*1+16,	GSS_OUTPUT_LOW	},	/* SLCD_RD */
    {32*1+17,	GSS_OUTPUT_LOW	},	/* SLCD_WR */
    {32*1+18,	GSS_OUTPUT_LOW	},	/* SLCD_CE */
    {32*1+19,	GSS_OUTPUT_LOW	},	/* SLCD_TE */
    {32*1+20,	GSS_OUTPUT_LOW	},	/* SLCD_DC */
    {32*1+21,	GSS_OUTPUT_LOW	},	/* DMIC_CLK */
    {32*1+22,	GSS_OUTPUT_LOW	},	/* DMIC_IN0 */
    {32*1+23,	GSS_INPUT_PULL	},	/* SMB0_SCK */
    {32*1+24,	GSS_INPUT_PULL	},	/* SMB0_SDA */
    {32*1+25,	GSS_OUTPUT_LOW	},	/* DRVVBUS */
    {32*1+26,	GSS_OUTPUT_LOW	},	/* CLK32K */
    {32*1+27,	GSS_OUTPUT_LOW	},	/* EXCLK */
    {32*1+28,	GSS_INPUT_NOPULL},	/* BOOT_SEL0 */
    {32*1+29,	GSS_INPUT_NOPULL},	/* BOOT_SEL1 */
    {32*1+30,	GSS_INPUT_NOPULL},	/* BOOT_SEL2 */
    {32*1+31,	GSS_INPUT_NOPULL	},	/* WAKEUP */


    {32*2+0,	GSS_OUTPUT_LOW      },	/* MSC1_CLK */
    {32*2+1,	GSS_OUTPUT_LOW      },	/* MSC1_CMD */
    {32*2+2,	GSS_OUTPUT_LOW      },	/* MSC1_D0 */
    {32*2+3,	GSS_OUTPUT_LOW      },	/* MSC1_D1 */
    {32*2+4,	GSS_OUTPUT_LOW      },	/* MSC1_D2 */
    {32*2+5,	GSS_OUTPUT_LOW      },	/* MSC1_D3 */
    {32*2+6,	GSS_OUTPUT_LOW	},	/* PCM_CLK */
    {32*2+7,	GSS_OUTPUT_LOW	},	/* PCM_DO */
    {32*2+8,	GSS_OUTPUT_LOW	},	/* PCM_DI */
    {32*2+9,	GSS_OUTPUT_LOW	},	/* PCM_SYN */
    {32*2+10,	GSS_OUTPUT_LOW	},	/* UART0_RXD */
    {32*2+11,	GSS_OUTPUT_LOW	},	/* UART0_TXD */
    {32*2+12,	GSS_OUTPUT_LOW	},	/* UART0_CTS_N */
    {32*2+13,	GSS_OUTPUT_LOW	},	/* UART0_RTS_N */
    {32*2+16,	GSS_OUTPUT_LOW      },	/* WL_WAKE_HOST */
    {32*2+17,	GSS_OUTPUT_LOW	},	/* WL_REG_EN */
    {32*2+18,	GSS_OUTPUT_LOW	},	/* BT_REG_EN */
    {32*2+19,	GSS_OUTPUT_LOW      },	/* HOST_WAKE_BT */
    {32*2+20,	GSS_OUTPUT_LOW      },	/* BT_WAKE_HOST */
    {32*2+21,	GSS_OUTPUT_LOW  },	/* OTG_ID */
    {32*2+22,	GSS_OUTPUT_LOW  },	/* USB_DETECT */
    {32*2+23,	GSS_OUTPUT_LOW	},	/* MAC_RST_N */
    {32*2+24,	GSS_OUTPUT_LOW	},	/* USB_DETE */
    {32*2+25,	GSS_OUTPUT_LOW	},	/* LCD_PWM */
    {32*2+26,	GSS_INPUT_PULL	},	/* SMB1_SCk */ //???
    {32*2+27,	GSS_INPUT_PULL	},	/* SMB1_SDA */ //???
    {32*2+28,	GSS_IGNORE	},	/* SMB1_SDA */
    {32*2+29,	GSS_IGNORE	},	/* SMB1_SDA */
    {32*2+30,	GSS_IGNORE	},	/* SMB1_SDA */
    {32*2+31,	GSS_IGNORE	},	/* uart2 */

    {32*3+0,	GSS_INPUT_PULL	},	/* SSI0_CLK */
    {32*3+1,	GSS_INPUT_PULL	},	/* SSI0_CE0 */
    {32*3+2,	GSS_OUTPUT_LOW	},	/* SSI0_DT */
    {32*3+3,	GSS_OUTPUT_LOW	},	/* SSI0_DR */
    {32*3+4,	GSS_OUTPUT_LOW	},	/* UART2_TXD */
    {32*3+5,	GSS_OUTPUT_LOW	},	/* UART2_RXD */
    {GSS_TABLET_END,GSS_TABLET_END	}	/* GPIO Group Set End */
};

#endif
#if 0
//直接是Ｍ3K　LED to low IO配置 OKOK
//以下的配置不贴csr8675，不连接USB开机，首次开机休眠功耗0.840mA  再次休眠功耗0.951mA
//连接USB开机，首次休眠功耗0.960mA,再次休眠功耗0.960mA,初步预测USB模块休眠时耗电0.1mA
__initdata int gpio_ss_table[][2] = {
    {32*0+0,	GSS_OUTPUT_LOW      },	/* SLCD_D0 */
    {32*0+1,	GSS_OUTPUT_LOW      },	/* SLCD_D1 */
    {32*0+2,	GSS_OUTPUT_LOW      },	/* SLCD_D2 */
    {32*0+3,	GSS_OUTPUT_LOW      },	/* SLCD_D3 */
    {32*0+4,	GSS_OUTPUT_LOW      },	/* SLCD_D4 */
    {32*0+5,	GSS_OUTPUT_LOW      },	/* SLCD_D5 */
    {32*0+6,	GSS_OUTPUT_LOW      },	/* SLCD_D6 */
    {32*0+7,	GSS_OUTPUT_LOW      },	/* SLCD_D7 */
    {32*0+8,	GSS_OUTPUT_LOW      },	/* SLCD_D8 */
    {32*0+9,	GSS_OUTPUT_LOW      },	/* SLCD_D9 */
    {32*0+10,	GSS_OUTPUT_LOW      },	/* SLCD_D10 */
    {32*0+11,	GSS_OUTPUT_LOW      },	/* SLCD_D11 */
    {32*0+12,	GSS_OUTPUT_LOW      },	/* SLCD_D12 */
    {32*0+13,	GSS_OUTPUT_LOW      },	/* SLCD_D13 */
    {32*0+14,	GSS_OUTPUT_LOW      },	/* SLCD_D14 */
    {32*0+15,	GSS_OUTPUT_LOW      },	/* SLCD_D15 */
    {32*0+16,	GSS_OUTPUT_LOW      },	/* PO_EN */
    {32*0+17,	GSS_INPUT_PULL      },	/* CIM_D2 */
    {32*0+18,	GSS_OUTPUT_LOW      },	/* CIM_D1 */
    {32*0+19,	GSS_INPUT_PULL    },	/* M5 VOL+  BT sink mode GSS_INPUT_NOPULL,GSS_INPUT_PULL(M3K)*/
    {32*0+20,	GSS_OUTPUT_LOW      },	/* MSC0_D3 */
    {32*0+21,	GSS_OUTPUT_LOW      },	/* MSC0_D2 */
    {32*0+22,	GSS_OUTPUT_LOW      },	/* MSC0_D1 */
    {32*0+23,	GSS_OUTPUT_LOW      },	/* MSC0_D0 */
    {32*0+24,	GSS_OUTPUT_LOW      },	/* MSC0_CLK */
    {32*0+25,	GSS_OUTPUT_LOW      },	/* MSC0_CMD */
    {32*0+26,	GSS_INPUT_PULL      },	/* SFC_CLK */
    {32*0+27,	GSS_INPUT_PULL      },	/* SFC_CE */
    {32*0+28,	GSS_INPUT_PULL      },	/* SFC_DR */
    {32*0+29,	GSS_INPUT_PULL      },	/* SFC_DT */
    {32*0+30,	GSS_INPUT_PULL      },	/* SFC_WP */
    {32*0+31,	GSS_INPUT_PULL      },	/* SFC_HOL */


    {32*1+0,	GSS_INPUT_PULL    },	/* I2S_MCLK  bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+1,	GSS_INPUT_PULL    },	/* I2S_BCLK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+2,	GSS_INPUT_PULL    },	/* I2S_LRCK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+3,	GSS_INPUT_PULL    },	/* I2S_DI bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+4,	GSS_INPUT_PULL    },	/* I2S_DO bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+5,	GSS_INPUT_PULL    },	/* M5 LED0 bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    //MSC0_DET
    {32*1+6,	GSS_INPUT_NOPULL    },	/* MAC_PHY_CLK */
    //USB ID
    {32*1+7,	GSS_INPUT_NOPULL	},  /* M5 CC INT_N */
    {32*1+8,	GSS_INPUT_PULL      },	/* M5 USB POWER ON AVDOTG25 2.5V GSS_INPUT_PULL(M3K)*/
    {32*1+9,	GSS_INPUT_PULL      },	/* M5 USB ID  */
    {32*1+10,	GSS_INPUT_PULL      },	/* MAC_TXEN */
    {32*1+11,	GSS_INPUT_NOPULL    },  /* M5 USB DET */
    {32*1+12,	GSS_OUTPUT_LOW      },	/* MAC_TXD0 */
    {32*1+13,	GSS_OUTPUT_LOW      },	/* M5 OTG_EN */
    {32*1+14,	GSS_OUTPUT_LOW      },	/* MAC_MDIO */
    {32*1+15,	GSS_OUTPUT_LOW      },	/* MAC_REF_CLK */
    {32*1+16,	GSS_OUTPUT_LOW      },	/* SLCD_RD */
    {32*1+17,	GSS_OUTPUT_LOW      },	/* SLCD_WR */
    {32*1+18,	GSS_OUTPUT_LOW      },	/* SLCD_CE */
    {32*1+19,	GSS_OUTPUT_LOW      },	/* SLCD_TE */
    {32*1+20,	GSS_OUTPUT_LOW      },	/* SLCD_DC */
    {32*1+21,	GSS_INPUT_PULL      },	/* M5 LED2(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+22,	GSS_INPUT_PULL      },	/* M5 LED1(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+23,	GSS_INPUT_PULL      },	/* M5 SMB0_SCK bt sink mode set GSS_INPUT_NOPULL(1.240~1.253mA) ,GSS_INPUT_PULL(M3K1.230~1.247mA)*/
    {32*1+24,	GSS_INPUT_PULL      },	/* M5 SMB0_SDA bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K)*/
    {32*1+25,	GSS_OUTPUT_LOW      },	/* DRVVBUS */
    {32*1+26,	GSS_OUTPUT_LOW      },	/* CLK32K */
    {32*1+27,	GSS_OUTPUT_LOW      },	/* EXCLK */
    {32*1+28,	GSS_INPUT_NOPULL    },	/* BOOT_SEL0  VOL-*/
    {32*1+29,	GSS_INPUT_NOPULL    },	/* BOOT_SEL1 */
    {32*1+30,	GSS_INPUT_NOPULL    },	/* BOOT_SEL2 */
    {32*1+31,	GSS_INPUT_NOPULL	},	/* WAKEUP */


    {32*2+0,	GSS_OUTPUT_LOW      },	/* M5 BT_INT1 */
    {32*2+1,	GSS_OUTPUT_LOW      },	/* MSC1_CMD */
    {32*2+2,	GSS_OUTPUT_LOW      },	/* M5 BT_INT2 */
    {32*2+3,	GSS_OUTPUT_LOW      },	/* M5 BT_RST bt sink mode set GSS_OUTPUT_HIGH if GSS_OUTPUT_LOW(M3K-0.25mA)*/
    {32*2+4,	GSS_OUTPUT_LOW      },	/* MSC1_D2 */
    {32*2+5,	GSS_OUTPUT_LOW      },	/* MSC1_D3 */
    {32*2+6,	GSS_OUTPUT_LOW      },	/* PCM_CLK */
    {32*2+7,	GSS_OUTPUT_LOW      },	/* PCM_DO */
    {32*2+8,	GSS_OUTPUT_LOW      },	/* PCM_DI */
    {32*2+9,	GSS_OUTPUT_LOW      },	/* PCM_SYN */
    {32*2+10,	GSS_OUTPUT_LOW    },	/* M5 UART0_RXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW(M3K)*/
    {32*2+11,	GSS_OUTPUT_LOW    },	/* M5 UART0_TXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+12,	GSS_OUTPUT_LOW    },	/* M5 UART0_CTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+13,	GSS_OUTPUT_LOW    },	/* M5 UART0_RTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+16,	GSS_OUTPUT_LOW      },	/* WL_WAKE_HOST */
    {32*2+17,	GSS_OUTPUT_LOW      },	/* WL_REG_EN */
    {32*2+18,	GSS_OUTPUT_LOW      },	/* BT_REG_EN */
    {32*2+19,	GSS_OUTPUT_LOW      },	/* HOST_WAKE_BT */
    {32*2+20,	GSS_OUTPUT_LOW      },	/* BT_WAKE_HOST */
    {32*2+21,	GSS_OUTPUT_LOW      },	/* M5 sr8675 connect  */
    {32*2+22,	GSS_OUTPUT_LOW      },	/* M5 X1000 connect usb */
    {32*2+23,	GSS_OUTPUT_LOW      },	/* M5 BT_VREG_ON */
    {32*2+24,	GSS_OUTPUT_LOW      },	/* null */
    {32*2+25,	GSS_OUTPUT_LOW      },	/* LCD_PWM */
    {32*2+26,	GSS_INPUT_PULL      },	/* SMB1_SCk */ //???
    {32*2+27,	GSS_INPUT_PULL      },	/* SMB1_SDA */ //???
    {32*2+28,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+29,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+30,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+31,	GSS_IGNORE          },	/* uart2    */

    {32*3+0,	GSS_INPUT_PULL      },	/* SSI0_CLK */
    {32*3+1,	GSS_INPUT_PULL      },	/* SSI0_CE0 */
    {32*3+2,	GSS_OUTPUT_LOW      },	/* SSI0_DT */
    {32*3+3,	GSS_OUTPUT_LOW      },	/* M5 SPDIF/PO_SEL bt sink mode set LOW OR HIGH */
    {32*3+4,	GSS_OUTPUT_LOW      },	/* UART2_TXD */
    {32*3+5,	GSS_OUTPUT_LOW      },	/* UART2_RXD */
    {GSS_TABLET_END,GSS_TABLET_END	}	/* GPIO Group Set End */
};
#endif



#if 0
//check with M3K use for M5 test
//The first board
//1.39mA
__initdata int gpio_ss_table[][2] = {
    {32*0+0,	GSS_OUTPUT_LOW      },	/* SLCD_D0 */
    {32*0+1,	GSS_OUTPUT_LOW      },	/* SLCD_D1 */
    {32*0+2,	GSS_OUTPUT_LOW      },	/* SLCD_D2 */
    {32*0+3,	GSS_OUTPUT_LOW      },	/* SLCD_D3 */
    {32*0+4,	GSS_OUTPUT_LOW      },	/* SLCD_D4 */
    {32*0+5,	GSS_OUTPUT_LOW      },	/* SLCD_D5 */
    {32*0+6,	GSS_OUTPUT_LOW      },	/* SLCD_D6 */
    {32*0+7,	GSS_OUTPUT_LOW      },	/* SLCD_D7 */
    {32*0+8,	GSS_OUTPUT_LOW      },	/* SLCD_D8 */
    {32*0+9,	GSS_OUTPUT_LOW      },	/* SLCD_D9 */
    {32*0+10,	GSS_OUTPUT_LOW      },	/* SLCD_D10 */
    {32*0+11,	GSS_OUTPUT_LOW      },	/* SLCD_D11 */
    {32*0+12,	GSS_OUTPUT_LOW      },	/* SLCD_D12 */
    {32*0+13,	GSS_OUTPUT_LOW      },	/* SLCD_D13 */
    {32*0+14,	GSS_OUTPUT_LOW      },	/* SLCD_D14 */
    {32*0+15,	GSS_OUTPUT_LOW      },	/* SLCD_D15 */
    {32*0+16,	GSS_OUTPUT_LOW      },	/* PO_EN */
    {32*0+17,	GSS_INPUT_PULL      },	/* CIM_D2 */
    {32*0+18,	GSS_OUTPUT_LOW      },	/* CIM_D1 */
    {32*0+19,	GSS_INPUT_PULL      },	/* M5 VOL+  BT sink mode GSS_INPUT_NOPULL,GSS_INPUT_PULL(M3K)*/
    {32*0+20,	GSS_OUTPUT_LOW      },	/* MSC0_D3 */
    {32*0+21,	GSS_OUTPUT_LOW      },	/* MSC0_D2 */
    {32*0+22,	GSS_OUTPUT_LOW      },	/* MSC0_D1 */
    {32*0+23,	GSS_OUTPUT_LOW      },	/* MSC0_D0 */
    {32*0+24,	GSS_OUTPUT_LOW      },	/* MSC0_CLK */
    {32*0+25,	GSS_OUTPUT_LOW      },	/* MSC0_CMD */
    {32*0+26,	GSS_INPUT_PULL      },	/* SFC_CLK */
    {32*0+27,	GSS_INPUT_PULL      },	/* SFC_CE */
    {32*0+28,	GSS_INPUT_PULL      },	/* SFC_DR */
    {32*0+29,	GSS_INPUT_PULL      },	/* SFC_DT */
    {32*0+30,	GSS_INPUT_PULL      },	/* SFC_WP */
    {32*0+31,	GSS_INPUT_PULL      },	/* SFC_HOL */


    {32*1+0,	GSS_INPUT_NOPULL    },	/* M5 I2S_MCLK  bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+1,	GSS_INPUT_NOPULL    },	/* M5 I2S_BCLK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+2,	GSS_INPUT_NOPULL    },	/* M5 I2S_LRCK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+3,	GSS_OUTPUT_LOW    },	/* I2S_DI GSS_INPUT_PULL(M3K) */
    {32*1+4,	GSS_INPUT_NOPULL    },	/* M5 I2S_DO bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+5,	GSS_INPUT_PULL    },	/* M5 LED0 bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    //MSC0_DET
    {32*1+6,	GSS_INPUT_NOPULL    },	/* MAC_PHY_CLK */
    //USB ID
    {32*1+7,	GSS_INPUT_NOPULL	},  /* M5 CC INT_N */
    {32*1+8,	GSS_INPUT_PULL      },	/* M5 USB POWER ON AVDOTG25 2.5V GSS_INPUT_PULL(M3K)*/
    {32*1+9,	GSS_INPUT_PULL      },	/* M5 USB ID  */
    {32*1+10,	GSS_INPUT_PULL      },	/* MAC_TXEN */
    {32*1+11,	GSS_OUTPUT_LOW      },  /* M5 USB DET */
    {32*1+12,	GSS_OUTPUT_LOW      },	/* MAC_TXD0 */
    {32*1+13,	GSS_OUTPUT_LOW      },	/* M5 OTG_EN */
    {32*1+14,	GSS_OUTPUT_LOW      },	/* MAC_MDIO */
    {32*1+15,	GSS_OUTPUT_LOW      },	/* MAC_REF_CLK */
    {32*1+16,	GSS_OUTPUT_LOW      },	/* SLCD_RD */
    {32*1+17,	GSS_OUTPUT_LOW      },	/* SLCD_WR */
    {32*1+18,	GSS_OUTPUT_LOW      },	/* SLCD_CE */
    {32*1+19,	GSS_OUTPUT_LOW      },	/* SLCD_TE */
    {32*1+20,	GSS_OUTPUT_LOW      },	/* SLCD_DC */
    {32*1+21,	GSS_INPUT_PULL      },	/* M5 LED2(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+22,	GSS_INPUT_PULL      },	/* M5 LED1(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+23,	GSS_INPUT_PULL      },	/* M5 SMB0_SCK bt sink mode set GSS_INPUT_NOPULL(0.1mA) ,GSS_INPUT_PULL(M3K1.230~1.247mA)*/
    {32*1+24,	GSS_INPUT_PULL      },	/* M5 SMB0_SDA bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K)*/
    {32*1+25,	GSS_OUTPUT_LOW      },	/* DRVVBUS */
    {32*1+26,	GSS_OUTPUT_LOW      },	/* CLK32K */
    {32*1+27,	GSS_OUTPUT_LOW      },	/* EXCLK */
    {32*1+28,	GSS_INPUT_NOPULL    },	/* BOOT_SEL0  VOL-*/
    {32*1+29,	GSS_INPUT_NOPULL    },	/* BOOT_SEL1 */
    {32*1+30,	GSS_INPUT_NOPULL    },	/* BOOT_SEL2 */
    {32*1+31,	GSS_INPUT_NOPULL	},	/* WAKEUP */


    {32*2+0,	GSS_OUTPUT_LOW      },	/* M5 BT_INT1 */
    {32*2+1,	GSS_OUTPUT_LOW      },	/* MSC1_CMD */
    {32*2+2,	GSS_OUTPUT_LOW      },	/* M5 BT_INT2 */
    {32*2+3,	GSS_INPUT_PULL      },	/* M5 BT_RST bt sink mode set GSS_OUTPUT_HIGH if GSS_OUTPUT_LOW(M3K-0.25mA)*/
    {32*2+4,	GSS_OUTPUT_LOW      },	/* MSC1_D2 */
    {32*2+5,	GSS_OUTPUT_LOW      },	/* MSC1_D3 */
    {32*2+6,	GSS_OUTPUT_LOW      },	/* PCM_CLK */
    {32*2+7,	GSS_OUTPUT_LOW      },	/* PCM_DO */
    {32*2+8,	GSS_OUTPUT_LOW      },	/* PCM_DI */
    {32*2+9,	GSS_OUTPUT_LOW      },	/* PCM_SYN */
    {32*2+10,	GSS_OUTPUT_LOW      },	/* M5 UART0_RXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW(M3K)*/
    {32*2+11,	GSS_OUTPUT_LOW      },	/* M5 UART0_TXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+12,	GSS_OUTPUT_LOW      },	/* M5 UART0_CTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+13,	GSS_OUTPUT_LOW      },	/* M5 UART0_RTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+16,	GSS_OUTPUT_LOW      },	/* WL_WAKE_HOST */
    {32*2+17,	GSS_OUTPUT_LOW      },	/* WL_REG_EN */
    {32*2+18,	GSS_OUTPUT_LOW      },	/* BT_REG_EN */
    {32*2+19,	GSS_OUTPUT_LOW      },	/* HOST_WAKE_BT */
    {32*2+20,	GSS_OUTPUT_LOW      },	/* BT_WAKE_HOST */

    {32*2+21,	GSS_OUTPUT_LOW      },	/* M5 csr8675 connect  */
    {32*2+22,	GSS_OUTPUT_LOW      },	/* M5 X1000 connect usb */

    {32*2+23,	GSS_OUTPUT_LOW      },	/* M5 BT_VREG_ON */
    {32*2+24,	GSS_OUTPUT_LOW      },	/* null */
    {32*2+25,	GSS_OUTPUT_LOW      },	/* LCD_PWM */
    {32*2+26,	GSS_INPUT_PULL      },	/* SMB1_SCk */ //???
    {32*2+27,	GSS_INPUT_PULL      },	/* SMB1_SDA */ //???
    {32*2+28,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+29,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+30,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+31,	GSS_IGNORE          },	/* uart2    */

    {32*3+0,	GSS_INPUT_PULL      },	/* SSI0_CLK */
    {32*3+1,	GSS_INPUT_PULL      },	/* SSI0_CE0 */
    {32*3+2,	GSS_OUTPUT_LOW      },	/* SSI0_DT */
    {32*3+3,	GSS_OUTPUT_LOW      },	/* M5 SPDIF/PO_SEL bt sink mode set LOW OR HIGH */
    {32*3+4,	GSS_OUTPUT_LOW      },	/* UART2_TXD */
    {32*3+5,	GSS_OUTPUT_LOW      },	/* UART2_RXD */
    {GSS_TABLET_END,GSS_TABLET_END	}	/* GPIO Group Set End */
};
#endif

#if 0
//check with M3K use for M5 test
//The second board
//1.095mA
//1.066mA
__initdata int gpio_ss_table[][2] = {
    {32*0+0,	GSS_OUTPUT_LOW      },	/* SLCD_D0 */
    {32*0+1,	GSS_OUTPUT_LOW      },	/* SLCD_D1 */
    {32*0+2,	GSS_OUTPUT_LOW      },	/* SLCD_D2 */
    {32*0+3,	GSS_OUTPUT_LOW      },	/* SLCD_D3 */
    {32*0+4,	GSS_OUTPUT_LOW      },	/* SLCD_D4 */
    {32*0+5,	GSS_OUTPUT_LOW      },	/* SLCD_D5 */
    {32*0+6,	GSS_OUTPUT_LOW      },	/* SLCD_D6 */
    {32*0+7,	GSS_OUTPUT_LOW      },	/* SLCD_D7 */
    {32*0+8,	GSS_OUTPUT_LOW      },	/* SLCD_D8 */
    {32*0+9,	GSS_OUTPUT_LOW      },	/* SLCD_D9 */
    {32*0+10,	GSS_OUTPUT_LOW      },	/* SLCD_D10 */
    {32*0+11,	GSS_OUTPUT_LOW      },	/* SLCD_D11 */
    {32*0+12,	GSS_OUTPUT_LOW      },	/* SLCD_D12 */
    {32*0+13,	GSS_OUTPUT_LOW      },	/* SLCD_D13 */
    {32*0+14,	GSS_OUTPUT_LOW      },	/* SLCD_D14 */
    {32*0+15,	GSS_OUTPUT_LOW      },	/* SLCD_D15 */
    {32*0+16,	GSS_OUTPUT_LOW      },	/* PO_EN */
    {32*0+17,	GSS_INPUT_PULL      },	/* CIM_D2 */
    {32*0+18,	GSS_OUTPUT_LOW      },	/* CIM_D1 */
    {32*0+19,	GSS_INPUT_PULL      },	/* M5 VOL+  BT sink mode GSS_INPUT_NOPULL,GSS_INPUT_PULL(M3K)*/
    {32*0+20,	GSS_OUTPUT_LOW      },	/* MSC0_D3 */
    {32*0+21,	GSS_OUTPUT_LOW      },	/* MSC0_D2 */
    {32*0+22,	GSS_OUTPUT_LOW      },	/* MSC0_D1 */
    {32*0+23,	GSS_OUTPUT_LOW      },	/* MSC0_D0 */
    {32*0+24,	GSS_OUTPUT_LOW      },	/* MSC0_CLK */
    {32*0+25,	GSS_OUTPUT_LOW      },	/* MSC0_CMD */
    {32*0+26,	GSS_INPUT_PULL      },	/* SFC_CLK */
    {32*0+27,	GSS_INPUT_PULL      },	/* SFC_CE */
    {32*0+28,	GSS_INPUT_PULL      },	/* SFC_DR */
    {32*0+29,	GSS_INPUT_PULL      },	/* SFC_DT */
    {32*0+30,	GSS_INPUT_PULL      },	/* SFC_WP */
    {32*0+31,	GSS_INPUT_PULL      },	/* SFC_HOL */


    {32*1+0,	GSS_INPUT_PULL    },	/* M5 I2S_MCLK  bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+1,	GSS_INPUT_PULL    },	/* M5 I2S_BCLK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+2,	GSS_INPUT_PULL    },	/* M5 I2S_LRCK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+3,	GSS_OUTPUT_LOW    },	/* I2S_DI GSS_INPUT_PULL(M3K) */
    {32*1+4,	GSS_INPUT_PULL    },	/* M5 I2S_DO bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+5,	GSS_INPUT_PULL    },	/* M5 LED0 bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    //MSC0_DET
    {32*1+6,	GSS_INPUT_NOPULL    },	/* MAC_PHY_CLK */
    //USB ID
    {32*1+7,	GSS_INPUT_NOPULL	},  /* M5 CC INT_N */
    {32*1+8,	GSS_INPUT_PULL      },	/* M5 USB POWER ON AVDOTG25 2.5V GSS_INPUT_PULL(M3K)*/
    {32*1+9,	GSS_INPUT_PULL      },	/* M5 USB ID  */
    {32*1+10,	GSS_INPUT_PULL      },	/* MAC_TXEN */
    {32*1+11,	GSS_OUTPUT_LOW      },  /* M5 USB DET */
    {32*1+12,	GSS_OUTPUT_LOW      },	/* MAC_TXD0 */
    {32*1+13,	GSS_OUTPUT_LOW      },	/* M5 OTG_EN */
    {32*1+14,	GSS_OUTPUT_LOW      },	/* MAC_MDIO */
    {32*1+15,	GSS_OUTPUT_LOW      },	/* MAC_REF_CLK */
    {32*1+16,	GSS_OUTPUT_LOW      },	/* SLCD_RD */
    {32*1+17,	GSS_OUTPUT_LOW      },	/* SLCD_WR */
    {32*1+18,	GSS_OUTPUT_LOW      },	/* SLCD_CE */
    {32*1+19,	GSS_OUTPUT_LOW      },	/* SLCD_TE */
    {32*1+20,	GSS_OUTPUT_LOW      },	/* SLCD_DC */
    {32*1+21,	GSS_INPUT_PULL      },	/* M5 LED2(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+22,	GSS_INPUT_PULL      },	/* M5 LED1(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+23,	GSS_INPUT_PULL      },	/* M5 SMB0_SCK bt sink mode set GSS_INPUT_NOPULL(0.1mA) ,GSS_INPUT_PULL(M3K1.230~1.247mA)*/
    {32*1+24,	GSS_INPUT_PULL      },	/* M5 SMB0_SDA bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K)*/
    {32*1+25,	GSS_OUTPUT_LOW      },	/* DRVVBUS */
    {32*1+26,	GSS_OUTPUT_LOW      },	/* CLK32K */
    {32*1+27,	GSS_OUTPUT_LOW      },	/* EXCLK */
    {32*1+28,	GSS_INPUT_NOPULL    },	/* BOOT_SEL0  VOL-*/
    {32*1+29,	GSS_INPUT_NOPULL    },	/* BOOT_SEL1 */
    {32*1+30,	GSS_INPUT_NOPULL    },	/* BOOT_SEL2 */
    {32*1+31,	GSS_INPUT_NOPULL	},	/* WAKEUP */


    {32*2+0,	GSS_OUTPUT_LOW      },	/* M5 BT_INT1 */
    {32*2+1,	GSS_OUTPUT_LOW      },	/* MSC1_CMD */
    {32*2+2,	GSS_OUTPUT_LOW      },	/* M5 BT_INT2 */
    {32*2+3,	GSS_INPUT_PULL      },	/* M5 BT_RST bt sink mode set GSS_OUTPUT_HIGH if GSS_OUTPUT_LOW(M3K-0.25mA)*/
    {32*2+4,	GSS_OUTPUT_LOW      },	/* MSC1_D2 */
    {32*2+5,	GSS_OUTPUT_LOW      },	/* MSC1_D3 */
    {32*2+6,	GSS_OUTPUT_LOW      },	/* PCM_CLK */
    {32*2+7,	GSS_OUTPUT_LOW      },	/* PCM_DO */
    {32*2+8,	GSS_OUTPUT_LOW      },	/* PCM_DI */
    {32*2+9,	GSS_OUTPUT_LOW      },	/* PCM_SYN */
    {32*2+10,	GSS_OUTPUT_LOW      },	/* M5 UART0_RXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW(M3K)*/
    {32*2+11,	GSS_OUTPUT_LOW      },	/* M5 UART0_TXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+12,	GSS_OUTPUT_LOW      },	/* M5 UART0_CTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+13,	GSS_OUTPUT_LOW      },	/* M5 UART0_RTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+16,	GSS_OUTPUT_LOW      },	/* WL_WAKE_HOST */
    {32*2+17,	GSS_OUTPUT_LOW      },	/* WL_REG_EN */
    {32*2+18,	GSS_OUTPUT_LOW      },	/* BT_REG_EN */
    {32*2+19,	GSS_OUTPUT_LOW      },	/* HOST_WAKE_BT */
    {32*2+20,	GSS_OUTPUT_LOW      },	/* BT_WAKE_HOST */

    {32*2+21,	GSS_OUTPUT_LOW      },	/* M5 csr8675 connect  */
    {32*2+22,	GSS_OUTPUT_LOW      },	/* M5 X1000 connect usb */

    {32*2+23,	GSS_OUTPUT_LOW      },	/* M5 BT_VREG_ON */
    {32*2+24,	GSS_OUTPUT_LOW      },	/* null */
    {32*2+25,	GSS_OUTPUT_LOW      },	/* LCD_PWM */
    {32*2+26,	GSS_INPUT_PULL      },	/* SMB1_SCk */ //???
    {32*2+27,	GSS_INPUT_PULL      },	/* SMB1_SDA */ //???
    {32*2+28,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+29,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+30,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+31,	GSS_IGNORE          },	/* uart2    */

    {32*3+0,	GSS_INPUT_PULL      },	/* SSI0_CLK */
    {32*3+1,	GSS_INPUT_PULL      },	/* SSI0_CE0 */
    {32*3+2,	GSS_OUTPUT_LOW      },	/* SSI0_DT */
    {32*3+3,	GSS_INPUT_NOPULL      },	/* M5 SPDIF/PO_SEL bt sink mode set LOW OR HIGH */
    {32*3+4,	GSS_OUTPUT_LOW      },	/* UART2_TXD */
    {32*3+5,	GSS_OUTPUT_LOW      },	/* UART2_RXD */
    {GSS_TABLET_END,GSS_TABLET_END	}	/* GPIO Group Set End */
};
#endif

#if 1
//1.055mA
__initdata int gpio_ss_table[][2] = {
    {32*0+0,	GSS_OUTPUT_LOW      },	/* SLCD_D0 */
    {32*0+1,	GSS_OUTPUT_LOW      },	/* SLCD_D1 */
    {32*0+2,	GSS_OUTPUT_LOW      },	/* SLCD_D2 */
    {32*0+3,	GSS_OUTPUT_LOW      },	/* SLCD_D3 */
    {32*0+4,	GSS_OUTPUT_LOW      },	/* SLCD_D4 */
    {32*0+5,	GSS_OUTPUT_LOW      },	/* SLCD_D5 */
    {32*0+6,	GSS_OUTPUT_LOW      },	/* SLCD_D6 */
    {32*0+7,	GSS_OUTPUT_LOW      },	/* SLCD_D7 */
    {32*0+8,	GSS_OUTPUT_LOW      },	/* SLCD_D8 */
    {32*0+9,	GSS_OUTPUT_LOW      },	/* SLCD_D9 */
    {32*0+10,	GSS_OUTPUT_LOW      },	/* SLCD_D10 */
    {32*0+11,	GSS_OUTPUT_LOW      },	/* SLCD_D11 */
    {32*0+12,	GSS_OUTPUT_LOW      },	/* SLCD_D12 */
    {32*0+13,	GSS_OUTPUT_LOW      },	/* SLCD_D13 */
    {32*0+14,	GSS_OUTPUT_LOW      },	/* SLCD_D14 */
    {32*0+15,	GSS_OUTPUT_LOW      },	/* SLCD_D15 */
    {32*0+16,	GSS_OUTPUT_LOW      },	/* PO_EN */
    {32*0+17,	GSS_INPUT_PULL      },	/* CIM_D2 */
    {32*0+18,	GSS_OUTPUT_LOW      },	/* CIM_D1 */
    {32*0+19,	GSS_INPUT_PULL      },	/* M5 VOL+  BT sink mode GSS_INPUT_NOPULL,GSS_INPUT_PULL(M3K)*/
    {32*0+20,	GSS_OUTPUT_LOW      },	/* MSC0_D3 */
    {32*0+21,	GSS_OUTPUT_LOW      },	/* MSC0_D2 */
    {32*0+22,	GSS_OUTPUT_LOW      },	/* MSC0_D1 */
    {32*0+23,	GSS_OUTPUT_LOW      },	/* MSC0_D0 */
    {32*0+24,	GSS_OUTPUT_LOW      },	/* MSC0_CLK */
    {32*0+25,	GSS_OUTPUT_LOW      },	/* MSC0_CMD */
    {32*0+26,	GSS_INPUT_PULL      },	/* SFC_CLK */
    {32*0+27,	GSS_INPUT_PULL      },	/* SFC_CE */
    {32*0+28,	GSS_INPUT_PULL      },	/* SFC_DR */
    {32*0+29,	GSS_INPUT_PULL      },	/* SFC_DT */
    {32*0+30,	GSS_INPUT_PULL      },	/* SFC_WP */
    {32*0+31,	GSS_INPUT_PULL      },	/* SFC_HOL */


    {32*1+0,	GSS_INPUT_PULL    },	/* M5 I2S_MCLK  bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+1,	GSS_INPUT_PULL    },	/* M5 I2S_BCLK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+2,	GSS_INPUT_PULL    },	/* M5 I2S_LRCK bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+3,	GSS_OUTPUT_LOW    },	/* I2S_DI GSS_INPUT_PULL(M3K) */
    {32*1+4,	GSS_INPUT_PULL    },	/* M5 I2S_DO bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    {32*1+5,	GSS_INPUT_PULL    },	/* M5 LED0 bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K) */
    //MSC0_DET
    {32*1+6,	GSS_INPUT_NOPULL    },	/* MAC_PHY_CLK */
    //USB ID
    {32*1+7,	GSS_INPUT_NOPULL	},  /* M5 CC INT_N */
    {32*1+8,	GSS_OUTPUT_LOW      },	/* M5 USB POWER ON AVDOTG25 2.5V GSS_INPUT_PULL(M3K)*/
    {32*1+9,	GSS_INPUT_PULL      },	/* M5 USB ID  */
    {32*1+10,	GSS_INPUT_PULL      },	/* M5 PMU_IRQ 3.3v/51Kohm */
    {32*1+11,	GSS_INPUT_NOPULL    },  /* M5 USB DET */
    {32*1+12,	GSS_OUTPUT_LOW      },	/* M5 TP_INT GSS_OUTPUT_LOW(M3K)*/
    {32*1+13,	GSS_OUTPUT_LOW      },	/* M5 OTG_EN */
    {32*1+14,	GSS_OUTPUT_LOW      },	/* MAC_MDIO */
    {32*1+15,	GSS_OUTPUT_LOW      },	/* MAC_REF_CLK */
    {32*1+16,	GSS_OUTPUT_LOW      },	/* SLCD_RD */
    {32*1+17,	GSS_OUTPUT_LOW      },	/* SLCD_WR */
    {32*1+18,	GSS_OUTPUT_LOW      },	/* SLCD_CE */
    {32*1+19,	GSS_OUTPUT_LOW      },	/* SLCD_TE */
    {32*1+20,	GSS_OUTPUT_LOW      },	/* SLCD_DC */
    {32*1+21,	GSS_INPUT_PULL      },	/* M5 LED2(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+22,	GSS_INPUT_PULL      },	/* M5 LED1(GSS_INPUT_NOPULL) GSS_OUTPUT_LOW(M3K) */
    {32*1+23,	GSS_INPUT_PULL      },	/* M5 SMB0_SCK bt sink mode set GSS_INPUT_NOPULL(0.1mA) ,GSS_INPUT_PULL(M3K1.230~1.247mA)*/
    {32*1+24,	GSS_INPUT_PULL      },	/* M5 SMB0_SDA bt sink mode set GSS_INPUT_NOPULL ,GSS_INPUT_PULL(M3K)*/
    {32*1+25,	GSS_OUTPUT_LOW      },	/* DRVVBUS */
    {32*1+26,	GSS_OUTPUT_LOW      },	/* CLK32K */
    {32*1+27,	GSS_OUTPUT_LOW      },	/* EXCLK */
    {32*1+28,	GSS_INPUT_NOPULL    },	/* BOOT_SEL0  VOL-*/
    {32*1+29,	GSS_INPUT_NOPULL    },	/* BOOT_SEL1 */
    {32*1+30,	GSS_INPUT_NOPULL    },	/* BOOT_SEL2 */
    {32*1+31,	GSS_INPUT_NOPULL	},	/* WAKEUP  3.3v/100kohm*/


    {32*2+0,	GSS_OUTPUT_LOW      },	/* M5 BT_INT1 */
    {32*2+1,	GSS_OUTPUT_LOW      },	/* MSC1_CMD */
    {32*2+2,	GSS_OUTPUT_LOW      },	/* M5 BT_INT2 */
    {32*2+3,	GSS_INPUT_PULL      },	/* M5 BT_RST bt sink mode set GSS_OUTPUT_HIGH if GSS_OUTPUT_LOW(M3K-0.25mA)*/
    {32*2+4,	GSS_OUTPUT_LOW      },	/* MSC1_D2 */
    {32*2+5,	GSS_OUTPUT_LOW      },	/* MSC1_D3 */
    {32*2+6,	GSS_OUTPUT_LOW      },	/* PCM_CLK */
    {32*2+7,	GSS_OUTPUT_LOW      },	/* PCM_DO */
    {32*2+8,	GSS_OUTPUT_LOW      },	/* PCM_DI */
    {32*2+9,	GSS_OUTPUT_LOW      },	/* PCM_SYN */
    {32*2+10,	GSS_OUTPUT_LOW      },	/* M5 UART0_RXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW(M3K)*/
    {32*2+11,	GSS_OUTPUT_LOW      },	/* M5 UART0_TXD bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+12,	GSS_OUTPUT_LOW      },	/* M5 UART0_CTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+13,	GSS_OUTPUT_LOW      },	/* M5 UART0_RTS_N bt sink mode may set GSS_INPUT_NOPULL,default GSS_OUTPUT_LOW */
    {32*2+16,	GSS_OUTPUT_LOW      },	/* WL_WAKE_HOST */
    {32*2+17,	GSS_OUTPUT_LOW      },	/* WL_REG_EN */
    {32*2+18,	GSS_OUTPUT_LOW      },	/* BT_REG_EN */
    {32*2+19,	GSS_OUTPUT_LOW      },	/* HOST_WAKE_BT */
    {32*2+20,	GSS_OUTPUT_LOW      },	/* BT_WAKE_HOST */

    {32*2+21,	GSS_OUTPUT_LOW      },	/* M5 csr8675 connect  */
    {32*2+22,	GSS_OUTPUT_LOW      },	/* M5 X1000 connect usb */

    {32*2+23,	GSS_OUTPUT_LOW      },	/* M5 BT_VREG_ON */
    {32*2+24,	GSS_OUTPUT_LOW      },	/* null */
    {32*2+25,	GSS_OUTPUT_LOW      },	/* LCD_PWM */
    {32*2+26,	GSS_OUTPUT_LOW      },	/* M5 TP SMB1_SCk  GSS_INPUT_PULL(M3K) */ //???
    {32*2+27,	GSS_OUTPUT_LOW      },	/* M5 TP SMB1_SDA GSS_INPUT_PULL(M3K) */ //???
    {32*2+28,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+29,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+30,	GSS_IGNORE          },	/* SMB1_SDA */
    {32*2+31,	GSS_IGNORE          },	/* uart2    */

    {32*3+0,	GSS_INPUT_PULL      },	/* SSI0_CLK */
    {32*3+1,	GSS_INPUT_PULL      },	/* SSI0_CE0 */
    {32*3+2,	GSS_OUTPUT_LOW      },	/* SSI0_DT */
    {32*3+3,	GSS_OUTPUT_LOW      },	/* M5 SPDIF/PO_SEL bt sink mode set LOW OR HIGH */
    {32*3+4,	GSS_INPUT_NOPULL      },	/* M5 STEP-X1000 INT1 UART2_TXD */
    {32*3+5,	GSS_INPUT_NOPULL      },	/* M5 STEP-X1000 INT2 UART2_RXD */
    {GSS_TABLET_END,GSS_TABLET_END	}	/* GPIO Group Set End */
};
#endif
