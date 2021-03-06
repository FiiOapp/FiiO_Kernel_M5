/* drivers/video/jz4780/regs.h
 *
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Register definition file for ingenic jz4780 Display Controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _REGS_LCDC_H_
#define _REGS_LCDC_H_

/* Register Map Of LCDC */
#define LCDC_CFG	(0x00)	/* Configure Register */
#define LCDC_CTRL	(0x30)	/* Control Register */
#define LCDC_STATE	(0x34)	/* Status Register */

#define LCDC_OSDC	(0x100)	/* OSD Configure Register */
#define LCDC_OSDCTRL	(0x104)	/* OSD Control Register */
#define LCDC_OSDS	(0x108)	/* OSD Status Register */
#define LCDC_BGC0	(0x10c)	/* Background 0 Color Register */
#define LCDC_BGC1	(0x2c4)	/* Background 1 Color Register */
//#define LCDC_BGC1     (0x24c) /* Background 1 Color Register */
#define LCDC_KEY0	(0x110)	/* Foreground Color Key Register 0 */
#define LCDC_KEY1	(0x114)	/* Foreground Color Key Register 1 */
#define LCDC_ALPHA	(0x118)	/* ALPHA Register */
#define LCDC_IPUR	(0x11c)	/* IPU Restart Register */
#define LCDC_RGBC	(0x90)	/* RGB Controll Register */

#define LCDC_VAT	(0x0c)	/* Virtual Area Setting Register */
#define LCDC_DAH	(0x10)	/* Display Area Horizontal Start/End Point */
#define LCDC_DAV	(0x14)	/* Display Area Vertical Start/End Point */

#define LCDC_XYP0	(0x120)	/* Foreground 0 XY Position Register */
#define LCDC_XYP1	(0x124)	/* Foreground 1 XY Position Register */
#define LCDC_SIZE0	(0x128)	/* Foreground 0 Size Register */
#define LCDC_SIZE1	(0x12c)	/* Foreground 1 Size Register */

#define LCDC_VSYNC	(0x04)	/* Vertical Synchronize Register */
#define LCDC_HSYNC	(0x08)	/* Horizontal Synchronize Register */
#define LCDC_PS		(0x18)	/* PS Signal Setting */
#define LCDC_CLS	(0x1c)	/* CLS Signal Setting */
#define LCDC_SPL	(0x20)	/* SPL Signal Setting */
#define LCDC_REV	(0x24)	/* REV Signal Setting */
#define LCDC_IID	(0x38)	/* Interrupt ID Register */
#define LCDC_DA0	(0x40)	/* Descriptor Address Register 0 */
#define LCDC_SA0	(0x44)	/* Source Address Register 0 */
#define LCDC_FID0	(0x48)	/* Frame ID Register 0 */
#define LCDC_CMD0	(0x4c)	/* DMA Command Register 0 */

#define LCDC_OFFS0	(0x60)	/* DMA Offsize Register 0 */
#define LCDC_PW0	(0x64)	/* DMA Page Width Register 0 */
#define LCDC_CNUM0	(0x68)	/*
				 * DMA Command Counter Register 0
				 * only used in smart LCD mode
				 */
#define LCDC_CPOS0	(0x68)	/* DMA Command and position Register 0 */
#define LCDC_DESSIZE0	(0x6c)	/* Foreground Size in Descriptor 0 Register */

#define LCDC_DA1	(0x50)	/* Descriptor Address Register 1 */
#define LCDC_SA1	(0x54)	/* Source Address Register 1 */
#define LCDC_FID1	(0x58)	/* Frame ID Register 1 */
#define LCDC_CMD1	(0x5c)	/* DMA Command Register 1 */
#define LCDC_OFFS1	(0x70)	/* DMA Offsize Register 1 */
#define LCDC_PW1	(0x74)	/* DMA Page Width Register 1 */
#define LCDC_CNUM1	(0x78)	/*
				 * DMA Command Counter Register 1
				 * only used in smart LCD mode
				 */
#define LCDC_CPOS1	(0x78)	/* DMA Command and position Register 1 */
#define LCDC_DESSIZE1	(0x7c)	/* Foreground Size in Descriptor 1 Register */

#define LCDC_PCFG	(0x2c0)	/* Priority level threshold configure Register */
#define LCDC_DUAL_CTRL	(0x2c8)	/* Dual LCDC Channel Control register */

#define LCDC_ENH_CFG	(0x400)	/* Image inhancement CFG Register */
#define LCDC_ENH_CSCCFG	(0x404)	/* Color space conversion CFG Register */
#define LCDC_ENH_LUMACFG	(0x408)	/* LUMA CFG Register */
#define LCDC_ENH_CHROCFG0	(0x40c)	/* CHROMA0 CFG Register */
#define LCDC_ENH_CHROCFG1	(0x410)	/* CHROMA1 CFG Register */
#define LCDC_ENH_DITHERCFG	(0x414)	/* DITHER CFG Register */
#define LCDC_ENH_STATUS	(0x418)	/* Enhance status Register */
#define LCDC_ENH_GAMMA	(0x800)	/* GAMMA CFG Register */
#define LCDC_ENH_VEE	(0x1000)	/* VEE CFG Register */

/* LCD Configure Register */
#define LCDC_CFG_LCDPIN_BIT	31	/* LCD pins selection */
#define LCDC_CFG_LCDPIN_MASK	(0x1 << LCDC_CFG_LCDPIN_BIT)
#define LCDC_CFG_LCDPIN_LCD	(0x0 << LCDC_CFG_LCDPIN_BIT)
#define LCDC_CFG_LCDPIN_SLCD	(0x1 << LCDC_CFG_LCDPIN_BIT)
#define LCDC_CFG_TVEPEH		(1 << 30)	/* TVE PAL enable extra halfline signal */
					  /* Keep this bit to 0 */
#define LCDC_CFG_NEWDES		(1 << 28)	/* use new descripter. old: 4words, new:8words */
#define LCDC_CFG_PALBP		(1 << 27)	/* bypass data format and alpha blending */
#define LCDC_CFG_TVEN		(1 << 26)	/* indicate the terminal is lcd or tv */
#define LCDC_CFG_RECOVER		(1 << 25)	/* Auto recover when output fifo underrun */
/* Dither function has been move to DITHER CFG Register*/
#define LCDC_CFG_PSM		(1 << 23)	/* PS signal mode */
#define LCDC_CFG_CLSM		(1 << 22)	/* CLS signal mode */
#define LCDC_CFG_SPLM		(1 << 21)	/* SPL signal mode */
#define LCDC_CFG_REVM		(1 << 20)	/* REV signal mode */
#define LCDC_CFG_HSYNM		(1 << 19)	/* HSYNC signal mode */
#define LCDC_CFG_PCLKM		(1 << 18)	/* PCLK signal mode */
#define LCDC_CFG_INVDAT		(1 << 17)	/* Inverse output data */
#define LCDC_CFG_SYNDIR_IN	(1 << 16)	/* VSYNC&HSYNC direction */
#define LCDC_CFG_PSP		(1 << 15)	/* PS pin reset state */
#define LCDC_CFG_CLSP		(1 << 14)	/* CLS pin reset state */
#define LCDC_CFG_SPLP		(1 << 13)	/* SPL pin reset state */
#define LCDC_CFG_REVP		(1 << 12)	/* REV pin reset state */
#define LCDC_CFG_HSP		(1 << 11)	/* HSYNC polarity:0-active high,1-active low */
#define LCDC_CFG_PCP		(1 << 10)	/* PCLK polarity:0-rising,1-falling */
#define LCDC_CFG_DEP		(1 << 9)	/* DE polarity:0-active high,1-active low */
#define LCDC_CFG_VSP		(1 << 8)	/* VSYNC polarity:0-rising,1-falling */
#define LCDC_CFG_MODE_TFT_18BIT 	(1 << 7)	/* 18bit TFT */
#define LCDC_CFG_MODE_TFT_16BIT 	(0 << 7)	/* 16bit TFT */
#define LCDC_CFG_MODE_TFT_24BIT 	(1 << 6)	/* 24bit TFT */

#define LCDC_CFG_MODE_BIT	0	/* Display Device Mode Select */
#define LCDC_CFG_MODE_MASK	(0x0f << LCDC_CFG_MODE_BIT)
#define LCDC_CFG_MODE_GENERIC_TFT	(0 << LCDC_CFG_MODE_BIT)	/* 16,18 bit TFT */
#define LCDC_CFG_MODE_SPECIAL_TFT_1	(1 << LCDC_CFG_MODE_BIT)
#define LCDC_CFG_MODE_SPECIAL_TFT_2	(2 << LCDC_CFG_MODE_BIT)
#define LCDC_CFG_MODE_SPECIAL_TFT_3	(3 << LCDC_CFG_MODE_BIT)
#define LCDC_CFG_MODE_NONINTER_CCIR656	(4 << LCDC_CFG_MODE_BIT)
#define LCDC_CFG_MODE_INTER_CCIR656	(6 << LCDC_CFG_MODE_BIT)
#define LCDC_CFG_MODE_SERIAL_TFT	(12 << LCDC_CFG_MODE_BIT)
#define LCDC_CFG_MODE_LCM  		(13 << LCDC_CFG_MODE_BIT)

/* LCD Control Register */
#define LCDC_CTRL_PINMD		(1 << 31)	/* This register set Pin distribution in 16-bit parallel mode
						   0: 16-bit data correspond with LCDC_D[15:0]
						   1: 16-bit data correspond with LCDC_D[17:10], LCDC_D[8:1] */
#define LCDC_CTRL_BST_BIT	28	/* Burst Length Selection */
#define LCDC_CTRL_BST_MASK	(0x7 << LCDC_CTRL_BST_BIT)
#define LCDC_CTRL_BST_4		(0 << LCDC_CTRL_BST_BIT)	/* 4-word */
#define LCDC_CTRL_BST_8		(1 << LCDC_CTRL_BST_BIT)	/* 8-word */
#define LCDC_CTRL_BST_16	(2 << LCDC_CTRL_BST_BIT)	/* 16-word */
#define LCDC_CTRL_BST_32	(3 << LCDC_CTRL_BST_BIT)	/* 32-word */
#define LCDC_CTRL_BST_64	(4 << LCDC_CTRL_BST_BIT)	/* 64-word */
#define LCDC_CTRL_RGB565		(0 << 27)	/* RGB565 mode(foreground 0 in OSD mode) */
#define LCDC_CTRL_RGB555		(1 << 27)	/* RGB555 mode(foreground 0 in OSD mode) */
#define LCDC_CTRL_OFUP		(1 << 26)	/* Output FIFO underrun protection enable */
#define LCDC_CTRL_PDD_BIT	16	/* Load Palette Delay Counter */
#define LCDC_CTRL_PDD_MASK	(0xff << LCDC_CTRL_PDD_BIT)
				/* Keep this bit to 0 */
#define LCDC_CTRL_DACTE		(1 << 14)	/* DAC loop back test */
#define LCDC_CTRL_EOFM		(1 << 13)	/* EOF interrupt mask */
#define LCDC_CTRL_SOFM		(1 << 12)	/* SOF interrupt mask */
#define LCDC_CTRL_OFUM		(1 << 11)	/* Output FIFO underrun interrupt mask */
#define LCDC_CTRL_IFUM0		(1 << 10)	/* Input FIFO 0 underrun interrupt mask */
#define LCDC_CTRL_IFUM1		(1 << 9)	/* Input FIFO 1 underrun interrupt mask */
#define LCDC_CTRL_LDDM		(1 << 8)	/* LCD disable done interrupt mask */
#define LCDC_CTRL_QDM		(1 << 7)	/* LCD quick disable done interrupt mask */
#define LCDC_CTRL_BEDN		(1 << 6)	/* Endian selection */
#define LCDC_CTRL_PEDN		(1 << 5)	/* Endian in byte:0-msb first, 1-lsb first */
#define LCDC_CTRL_DIS		(1 << 4)	/* Disable indicate bit */
#define LCDC_CTRL_ENA		(1 << 3)	/* LCDC enable bit */
#define LCDC_CTRL_BPP_BIT	0	/* Bits Per Pixel */
#define LCDC_CTRL_BPP_MASK	(0x07 << LCDC_CTRL_BPP_BIT)
#define LCDC_CTRL_BPP_1		(0 << LCDC_CTRL_BPP_BIT)	/* 1 bpp */
#define LCDC_CTRL_BPP_2		(1 << LCDC_CTRL_BPP_BIT)	/* 2 bpp */
#define LCDC_CTRL_BPP_4		(2 << LCDC_CTRL_BPP_BIT)	/* 4 bpp */
#define LCDC_CTRL_BPP_8		(3 << LCDC_CTRL_BPP_BIT)	/* 8 bpp */
#define LCDC_CTRL_BPP_16	(4 << LCDC_CTRL_BPP_BIT)	/* 15/16 bpp */
#define LCDC_CTRL_BPP_18_24	(5 << LCDC_CTRL_BPP_BIT)	/* 18/24/32 bpp */
#define LCDC_CTRL_BPP_CMPS_24	(6 << LCDC_CTRL_BPP_BIT)	/* 24 compress bpp */
#define LCDC_CTRL_BPP_30	(7 << LCDC_CTRL_BPP_BIT)	/* 30 bpp */

/* LCD Status Register */
#define LCDC_STATE_QD		(1 << 7)	/* Quick Disable Done */
#define LCDC_STATE_EOF		(1 << 5)	/* EOF Flag */
#define LCDC_STATE_SOF		(1 << 4)	/* SOF Flag */
#define LCDC_STATE_OFU		(1 << 3)	/* Output FIFO Underrun */
#define LCDC_STATE_IFU0		(1 << 2)	/* Input FIFO 0 Underrun */
#define LCDC_STATE_IFU1		(1 << 1)	/* Input FIFO 1 Underrun */
#define LCDC_STATE_LDD		(1 << 0)	/* LCD Disabled */

/* OSD Configure Register */
#define LCDC_OSDC_PREMULTI1		(1 << 23)	/*
							 * Premulti enable of foreground 1
							 * 0:data has been premultied and don't need premulti
							 * 1:data should be premultied by lcd
							 */
#define LCDC_OSDC_COEF_SLE1_BIT		21	/* Select coefficient for foreground 1 */
#define LCDC_OSDC_COEF_SLE1_MASK		(0x03 << LCDC_OSDC_COEF_SLE1_BIT)
#define LCDC_OSDC_COEF_SLE1_0		(0 << LCDC_OSDC_COEF_SLE1_BIT)	/* 00:0 */
#define LCDC_OSDC_COEF_SLE1_1		(1 << LCDC_OSDC_COEF_SLE1_BIT)	/* 01:1 */
#define LCDC_OSDC_COEF_SLE1_2		(2 << LCDC_OSDC_COEF_SLE1_BIT)	/* 10:alpha0 */
#define LCDC_OSDC_COEF_SLE1_3		(3 << LCDC_OSDC_COEF_SLE1_BIT)	/* 11:1-alpha0 */

#define LCDC_OSDC_PREMULTI0		(1 << 20)	/*
							 * Premulti enable of foreground 0
							 * 0:data has been premultied and don't need premulti
							 * 1:data should be premultied by lcd
							 */
#define LCDC_OSDC_COEF_SLE0_BIT		18	/* Select coefficient for foreground 0 */
#define LCDC_OSDC_COEF_SLE0_MASK		(0x03 << LCDC_OSDC_COEF_SLE0_BIT)
#define LCDC_OSDC_COEF_SLE0_0		(0 << LCDC_OSDC_COEF_SLE0_BIT)	/* 00:0 */
#define LCDC_OSDC_COEF_SLE0_1		(1 << LCDC_OSDC_COEF_SLE0_BIT)	/* 01:1 */
#define LCDC_OSDC_COEF_SLE0_2		(2 << LCDC_OSDC_COEF_SLE0_BIT)	/* 10:alpha1 */
#define LCDC_OSDC_COEF_SLE0_3		(3 << LCDC_OSDC_COEF_SLE0_BIT)	/* 11:1-alpha1 */
#define LCDC_OSDC_ALPHAMD1		(1 << 17)	/* Alpha blending mode for foreground 1 */

#define LCDC_OSDC_SOFM1		(1 << 15)	/* Start of frame interrupt mask for foreground 1 */
#define LCDC_OSDC_EOFM1		(1 << 14)	/* End of frame interrupt mask for foreground 1 */
#define LCDC_OSDC_SOFM0		(1 << 11)	/* Start of frame interrupt mask for foreground 0 */
#define LCDC_OSDC_EOFM0		(1 << 10)	/* End of frame interrupt mask for foreground 0 */
#define LCDC_OSDC_DENDM		(1 << 9)	/* Display end interrupt mask */
#define LCDC_OSDC_F1EN		(1 << 4)	/* enable foreground 1 */
#define LCDC_OSDC_F0EN		(1 << 3)	/* enable foreground 0 */
#define LCDC_OSDC_ALPHAEN	(1 << 2)	/* enable alpha blending */
#define LCDC_OSDC_ALPHAMD0	(1 << 1)	/* alpha blending mode */
#define LCDC_OSDC_OSDEN		(1 << 0)	/* OSD mode enable */

/* OSD Controll Register */
#define LCDC_OSDCTRL_IPU_CLKEN	(1 << 15)	/* IPU clock enable: 1: enable; 0:disable */
#define LCDC_OSDCTRL_RGB0_RGB565	(0 << 5)	/* foreground 0, 16bpp, 0-RGB565, 1-RGB555 */
#define LCDC_OSDCTRL_RGB0_RGB555	(1 << 5)	/* foreground 0, 16bpp, 0-RGB565, 1-RGB555 */
#define LCDC_OSDCTRL_RGB1_RGB565	(0 << 4)	/* foreground 1, 16bpp, 0-RGB565, 1-RGB555 */
#define LCDC_OSDCTRL_RGB1_RGB555	(1 << 4)	/* foreground 1, 16bpp, 0-RGB565, 1-RGB555 */

#define LCDC_OSDCTRL_BPP_BIT	0	/* Bits Per Pixel of OSD Channel 1 */
#define LCDC_OSDCTRL_BPP_MASK	(0x7<<LCDC_OSDCTRL_BPP_BIT)	/* Bits Per Pixel of OSD Channel 1's MASK */
#define LCDC_OSDCTRL_BPP_15_16	(4 << LCDC_OSDCTRL_BPP_BIT)	/* RGB 15,16 bit */
#define LCDC_OSDCTRL_BPP_18_24	(5 << LCDC_OSDCTRL_BPP_BIT)	/* RGB 18,24 bit */
#define LCDC_OSDCTRL_BPP_CMPS_24	(6 << LCDC_OSDCTRL_BPP_BIT)	/* RGB compress 24 bit */
#define LCDC_OSDCTRL_BPP_30		(7 << LCDC_OSDCTRL_BPP_BIT)	/* RGB 30 bit */

/* OSD State Register */
#define LCDC_OSDS_SOF1		(1 << 15)	/* Start of frame flag for foreground 1 */
#define LCDC_OSDS_EOF1		(1 << 14)	/* End of frame flag for foreground 1 */
#define LCDC_OSDS_SOF0		(1 << 11)	/* Start of frame flag for foreground 0 */
#define LCDC_OSDS_EOF0		(1 << 10)	/* End of frame flag for foreground 0 */
#define LCDC_OSDS_DEND		(1 << 8)	/* Display end */

/* Background 0 or Background 1 Color Register */
#define LCDC_BGC_RED_OFFSET	16	/* Red color offset */
#define LCDC_BGC_RED_MASK	(0xFF << LCDC_BGC_RED_OFFSET)
#define LCDC_BGC_GREEN_OFFSET    8	/* Green color offset */
#define LCDC_BGC_GREEN_MASK	(0xFF << LCDC_BGC_GREEN_OFFSET)
#define LCDC_BGC_BLUE_OFFSET	0	/* Blue color offset */
#define LCDC_BGC_BLUE_MASK	(0xFF << LCDC_BGC_BLUE_OFFSET)

/* Foreground 0 or Foreground 1 Color Key Register */
#define LCDC_KEY_KEYEN		(1 << 31)	/* enable color key */
#define LCDC_KEY_KEYMD		(1 << 30)	/* color key mode */
#define LCDC_KEY_RED_OFFSET	16	/* Red color offset */
#define LCDC_KEY_RED_MASK	(0xFF << LCDC_KEY_RED_OFFSET)
#define LCDC_KEY_GREEN_OFFSET	8	/* Green color offset */
#define LCDC_KEY_GREEN_MASK	(0xFF << LCDC_KEY_GREEN_OFFSET)
#define LCDC_KEY_BLUE_OFFSET	0	/* Blue color offset */
#define LCDC_KEY_BLUE_MASK	(0xFF << LCDC_KEY_BLUE_OFFSET)
#define LCDC_KEY_MASK		(LCDC_KEY_RED_MASK | LCDC_KEY_GREEN_MASK\
				 | LCDC_KEY_BLUE_MASK)

/* ALPHA Register */
#define LCDC_ALPHA1_OFFSET	8	/* ALPHA 1 offset */
#define LCDC_ALPHA1_MASK		(0xFF << LCDC_ALPHA1_OFFSET)
#define LCDC_ALPHA0_OFFSET	0	/* ALPHA 0 offset */
#define LCDC_ALPHA0_MASK		(0xFF << LCDC_ALPHA0_OFFSET)

/* IPU Restart Register */
#define LCDC_IPUR_IPUREN		(1 << 31)	/* IPU restart function enable */
#define LCDC_IPUR_IPURMASK	(0xFFFFFF)	/* IPU restart value mask */

/* RGB Control Register */
#define LCDC_RGBC_RGBDM		(1 << 15)	/* enable RGB Dummy data */
#define LCDC_RGBC_DMM		(1 << 14)	/* RGB Dummy mode */
#define LCDC_RGBC_422		(1 << 8)	/* Change 444 to 422 */
#define LCDC_RGBC_RGBFMT	(1 << 7)	/* RGB format enable */
#define LCDC_RGBC_ODDRGB_BIT	4	/* odd line serial RGB data arrangement */
#define LCDC_RGBC_ODDRGB_MASK	(0x7 << LCDC_RGBC_ODDRGB_BIT)
#define LCDC_RGBC_ODD_RGB	(0 << LCDC_RGBC_ODDRGB_BIT)	/* RGB */
#define LCDC_RGBC_ODD_RBG	(1 << LCDC_RGBC_ODDRGB_BIT)	/* RBG */
#define LCDC_RGBC_ODD_GRB	(2 << LCDC_RGBC_ODDRGB_BIT)	/* GRB */
#define LCDC_RGBC_ODD_GBR	(3 << LCDC_RGBC_ODDRGB_BIT)	/* GBR */
#define LCDC_RGBC_ODD_BRG	(4 << LCDC_RGBC_ODDRGB_BIT)	/* BRG */
#define LCDC_RGBC_ODD_BGR	(5 << LCDC_RGBC_ODDRGB_BIT)	/* BGR */

#define LCDC_RGBC_EVENRGB_BIT	0	/* even line serial RGB data arrangement */
#define LCDC_RGBC_EVENRGB_MASK	(0x7<<LCDC_RGBC_EVENRGB_BIT)
#define LCDC_RGBC_EVEN_RGB	0	/* RGB */
#define LCDC_RGBC_EVEN_RBG	1	/* RBG */
#define LCDC_RGBC_EVEN_GRB	2	/* GRB */
#define LCDC_RGBC_EVEN_GBR	3	/* GBR */
#define LCDC_RGBC_EVEN_BRG	4	/* BRG */
#define LCDC_RGBC_EVEN_BGR	5	/* BGR */

/* Vertical Synchronize Register */
#define LCDC_VSYNC_VPS_BIT	16	/* VSYNC pulse start in line clock, fixed to 0 */
#define LCDC_VSYNC_VPS_MASK	(0xfff << LCDC_VSYNC_VPS_BIT)
#define LCDC_VSYNC_VPE_BIT	0	/* VSYNC pulse end in line clock */
#define LCDC_VSYNC_VPE_MASK	(0xfff << LCDC_VSYNC_VPE_BIT)

/* Horizontal Synchronize Register */
#define LCDC_HSYNC_HPS_BIT	16	/* HSYNC pulse start position in dot clock */
#define LCDC_HSYNC_HPS_MASK	(0xfff << LCDC_HSYNC_HPS_BIT)
#define LCDC_HSYNC_HPE_BIT	0	/* HSYNC pulse end position in dot clock */
#define LCDC_HSYNC_HPE_MASK	(0xfff << LCDC_HSYNC_HPE_BIT)

/* Virtual Area Setting Register */
#define LCDC_VAT_HT_BIT		16	/* Horizontal Total size in dot clock */
#define LCDC_VAT_HT_MASK		(0xfff << LCDC_VAT_HT_BIT)
#define LCDC_VAT_VT_BIT		0	/* Vertical Total size in dot clock */
#define LCDC_VAT_VT_MASK		(0xfff << LCDC_VAT_VT_BIT)

/* Display Area Horizontal Start/End Point Register */
#define LCDC_DAH_HDS_BIT		16	/* Horizontal display area start in dot clock */
#define LCDC_DAH_HDS_MASK	(0xfff << LCDC_DAH_HDS_BIT)
#define LCDC_DAH_HDE_BIT		0	/* Horizontal display area end in dot clock */
#define LCDC_DAH_HDE_MASK	(0xfff << LCDC_DAH_HDE_BIT)

/* Display Area Vertical Start/End Point Register */
#define LCDC_DAV_VDS_BIT		16	/* Vertical display area start in line clock */
#define LCDC_DAV_VDS_MASK	(0xfff << LCDC_DAV_VDS_BIT)
#define LCDC_DAV_VDE_BIT		0	/* Vertical display area end in line clock */
#define LCDC_DAV_VDE_MASK	(0xfff << LCDC_DAV_VDE_BIT)

/* Foreground 0 or Foreground 1 XY Position Register */
#define LCDC_XYP_YPOS_BIT	16	/* Y position bit of foreground 0 or 1 */
#define LCDC_XYP_YPOS_MASK	(0xfff << LCDC_XYP_YPOS_BIT)
#define LCDC_XYP_XPOS_BIT	0	/* X position bit of foreground 0 or 1 */
#define LCDC_XYP_XPOS_MASK	(0xfff << LCDC_XYP_XPOS_BIT)

/* Foreground 0 or Foreground 1 Size Register */
#define LCDC_SIZE_HEIGHT_BIT	16	/* The height of foreground 0 or 1 */
#define LCDC_SIZE_HEIGHT_MASK	(0xfff << LCDC_SIZE_HEIGHT_BIT)	/* The height of foreground 0 or 1 */
#define LCDC_SIZE_WIDTH_BIT	0	/* The width of foreground 0 or 1 */
#define LCDC_SIZE_WIDTH_MASK	(0xfff << LCDC_SIZE_WIDTH_BIT)	/* The width of foreground 0 or 1 */

/* PS Signal Setting */
#define LCDC_PS_PSS_BIT		16	/* PS signal start position in dot clock */
#define LCDC_PS_PSS_MASK		(0xfff << LCDC_PS_PSS_BIT)
#define LCDC_PS_PSE_BIT		0	/* PS signal end position in dot clock */
#define LCDC_PS_PSE_MASK		(0xfff << LCDC_PS_PSE_BIT)

/* CLS Signal Setting */
#define LCDC_CLS_CLSS_BIT	16	/* CLS signal start position in dot clock */
#define LCDC_CLS_CLSS_MASK	(0xfff << LCDC_CLS_CLSS_BIT)
#define LCDC_CLS_CLSE_BIT	0	/* CLS signal end position in dot clock */
#define LCDC_CLS_CLSE_MASK	(0xfff << LCDC_CLS_CLSE_BIT)

/* SPL Signal Setting */
#define LCDC_SPL_SPLS_BIT	16	/* SPL signal start position in dot clock */
#define LCDC_SPL_SPLS_MASK	(0xfff << LCDC_SPL_SPLS_BIT)
#define LCDC_SPL_SPLE_BIT	0	/* SPL signal end position in dot clock */
#define LCDC_SPL_SPLE_MASK	(0xfff << LCDC_SPL_SPLE_BIT)

/* REV Signal Setting */
#define LCDC_REV_REVS_BIT	16	/* REV signal start position in dot clock */
#define LCDC_REV_REVS_MASK	(0xfff << LCDC_REV_REVS_BIT)

/* DMA Command 0 or 1 Register */
#define LCDC_CMD_SOFINT		(1 << 31)	/* Enable start of frame interrupt */
#define LCDC_CMD_EOFINT		(1 << 30)	/* Enable end of frame interrupt */
#define LCDC_CMD_CMD		(1 << 29)	/* indicate command in slcd mode */
#define LCDC_CMD_PAL		(1 << 28)	/* The descriptor contains a palette buffer */
#define LCDC_CMD_COMPEN		(1 << 27)	/*
						 * It indicate this frame is 16/24bpp compressed or not
						 * 0:not compressed
						 * 1:compressed
						 */
#define LCDC_CMD_FRM_EN		(1 << 26)	/* Indicate this frame is enable */
#define LCDC_CMD_FIELD_SEL	(1 << 25)	/* Field select for interlace
						 * 0:odd field or no interlace
						 * 1:even field
						 */
#define LCDC_CMD_16X16BLOCK	(1 << 24)	/* Fetch data by 16x16 block */
#define LCDC_CMD_LEN_BIT		0	/* The buffer length value (in word) */
#define LCDC_CMD_LEN_MASK	(0xffffff << LCDC_CMD_LEN_BIT)

/* DMA Offsize Register 0,1 */
#define LCDC_OFFS_BIT		0	/* OFFSIZE value for DMA 0,1(in word) */
#define LCDC_OFFS_OFFSIZE_MASK	(0xffffff << LCDC_OFFS_BIT)

/* DMA Page Width Register 0,1 */
#define LCDC_PW_BIT		0	/* Page width for DMA 0,1(in word) */
#define LCDC_PW_PAGEWIDTH_MASK	(0xffffff << LCDC_PW_BIT)

/* DMA Command Counter Register 0,1 */
#define LCDC_CNUM_BIT		 0	/* Commands' number in this frame transfer by DMA */
#define LCDC_CNUM_CNUM_MASK	(0xff << LCDC_CNUM_BIT)	/* Only use in Smart LCD mode */

/* DMA Command Counter Register */
#define LCDC_CPOS_ALPHAMD1	(1 << 31)	/* Alpha blending mode for foreground 0,1 */
#define LCDC_CPOS_RGB_RGB565	(0 << 30)	/* foreground 0 or 1, 16bpp, 0-RGB565, 1-RGB555 */
#define LCDC_CPOS_RGB_RGB555	(1 << 30)	/* foreground 0 or 1, 16bpp, 0-RGB565, 1-RGB555 */

#define LCDC_CPOS_BPP_BIT	27	/* Bits Per Pixel of OSD channel 1 (cannot use palette) */
#define LCDC_CPOS_BPP_MASK	(0x07 << LCDC_CPOS_BPP_BIT)
#define LCDC_CPOS_BPP_16	(4 << LCDC_CPOS_BPP_BIT)	/* 15/16 bpp */
#define LCDC_CPOS_BPP_18_24	(5 << LCDC_CPOS_BPP_BIT)	/* 18/24/32 bpp */
#define LCDC_CPOS_BPP_CMPS_24	(6 << LCDC_CPOS_BPP_BIT)	/* 24 compress bpp */
#define LCDC_CPOS_BPP_30	(7 << LCDC_CPOS_BPP_BIT)	/* 30 bpp */

#define LCDC_CPOS_PREMULTI	(1 << 26)	/* Premulti enable of foreground 0,1 */
#define LCDC_CPOS_COEF_SLE_BIT	24	/* Select coefficient for foreground 0,1 */
#define LCDC_CPOS_COEF_SLE_MASK	(0x3 << LCDC_CPOS_COEF_SLE_BIT)
#define LCDC_CPOS_COEF_SLE_0	(0 << LCDC_CPOS_COEF_SLE_BIT)	/* 00:0 */
#define LCDC_CPOS_COEF_SLE_1	(1 << LCDC_CPOS_COEF_SLE_BIT)	/* 01:1 */
#define LCDC_CPOS_COEF_SLE_2	(2 << LCDC_CPOS_COEF_SLE_BIT)	/* 10:alpha1 */
#define LCDC_CPOS_COEF_SLE_3	(3 << LCDC_CPOS_COEF_SLE_BIT)	/* 11:1-alpha1 */

#define LCDC_CPOS_YPOS_BIT	12	/* The Y position of top-left part for foreground 0,1 */
#define LCDC_CPOS_YPOS_MASK	(0xfff << LCDC_CPOS_YPOS_BIT)
#define LCDC_CPOS_XPOS_BIT	0	/* The Y position of top-left part for foreground 0,1 */
#define LCDC_CPOS_XPOS_MASK	(0xfff << LCDC_CPOS_XPOS_BIT)

/* Foreground 0,1 Size Register */
#define LCDC_DESSIZE_ALPHA_BIT	24	/*  The global alpha value of foreground 0,1 */
#define LCDC_DESSIZE_ALPHA_MASK	(0xff << LCDC_DESSIZE_ALPHA_BIT)
#define LCDC_DESSIZE_HEIGHT_BIT	12	/* height of foreground 1 */
#define LCDC_DESSIZE_HEIGHT_MASK	(0xfff << LCDC_DESSIZE_HEIGHT_BIT)
#define LCDC_DESSIZE_WIDTH_BIT	0	/* width of foreground 1 */
#define LCDC_DESSIZE_WIDTH_MASK	(0xfff << LCDC_DESSIZE_WIDTH_BIT)

/* Priority level threshold configure Register */
#define LCDC_PCFG_LCDC_PRI_MD	(1 << 31)

#define LCDC_PCFG_HP_BST_BIT	28
#define LCDC_PCFG_HP_BST_MASK	(0x7 << LCDC_PCFG_HP_BST_BIT)
#define LCDC_PCFG_HP_BST_4	(0 << LCDC_PCFG_HP_BST_BIT)	/* 000:4 word */
#define LCDC_PCFG_HP_BST_8	(1 << LCDC_PCFG_HP_BST_BIT)	/* 001:8 word */
#define LCDC_PCFG_HP_BST_16	(2 << LCDC_PCFG_HP_BST_BIT)	/* 010:16 word */
#define LCDC_PCFG_HP_BST_32	(3 << LCDC_PCFG_HP_BST_BIT)	/* 011:32 word */
#define LCDC_PCFG_HP_BST_C16	(5 << LCDC_PCFG_HP_BST_BIT)	/* 101:Continue 16 */
#define LCDC_PCFG_HP_BST_64	(4 << LCDC_PCFG_HP_BST_BIT)	/* 100:64 word */
#define LCDC_PCFG_HP_BST_DIS	(7 << LCDC_PCFG_HP_BST_BIT)	/* 111:disable */

#define LCDC_PCFG_PCFG2_BIT	18
#define LCDC_PCFG_PCFG2_MASK	(0x1ff << LCDC_PCFG_PCFG2_BIT)
#define LCDC_PCFG_PCFG1_BIT	9
#define LCDC_PCFG_PCFG1_MASK	(0x1ff << LCDC_PCFG_PCFG1_BIT)
#define LCDC_PCFG_PCFG0_BIT	0
#define LCDC_PCFG_PCFG0_MASK	(0x1ff << LCDC_PCFG_PCFG0_BIT)

/* Dual LCDC Channel Control register */
/*
 * Select which IPU is able to write back, this field is just
 * available in lcdc1. 0:ipu1; 1:ipu0
 */
#define LCDC_DUAL_CTRL_IPU_WR_SEL	(1 << 8)
/*
 * Select which controller output to the tft/slcd panel, this field is just
 * available in lcdc1. 0:lcdc1; 1:lcdc0
 */
#define LCDC_DUAL_CTRL_TFT_SEL		(1 << 6)
/*
 * 1: fix the priority of ipu0/1 in lcd internal arbiter;
 * 0: use priority of ipu0/1 generated by lcd in lcd internal arbiter
 */
#define LCDC_DUAL_CTRL_PRI_IPU_EN	(1 << 5)
#define LCDC_DUAL_CTRL_PRI_IPU_BIT	3
#define LCDC_DUAL_CTRL_PRI_IPU_MASK	(0x3 << LCDC_DUAL_CTRL_PRI_IPU_BIT)
/*
 * 1: fix the priority of lcd0/1 in lcd internal arbiter;
 * 0: use priority of lcd0/1 generated by lcd in lcd internal arbiter
 */
#define LCDC_DUAL_CTRL_PRI_LCD_EN	(1 << 2)
#define LCDC_DUAL_CTRL_PRI_LCD_BIT	0
#define LCDC_DUAL_CTRL_PRI_LCD_MASK	(0x3 << LCDC_DUAL_CTRL_PRI_LCD_BIT)

/* Image Enhancement CFG Register */
#define LCDC_ENH_CFG_DITHER_EN	(1 << 9)	/* Dither enable */
#define LCDC_ENH_CFG_YCC2RGB_EN	(1 << 8)	/* YCbCr to RGB enable */
#define LCDC_ENH_CFG_SATURATION_EN	(1 << 7)	/* Saturation enable */
#define LCDC_ENH_CFG_VEE_EN	(1 << 6)	/* Visibility enhance enable */
#define LCDC_ENH_CFG_HUE_EN	(1 << 5)	/* Hue enable */
#define LCDC_ENH_CFG_BRIGHTNESS_EN	(1 << 4)	/* Brightness enable */
#define LCDC_ENH_CFG_CONTRAST_EN	(1 << 3)	/* Contrast enable */
#define LCDC_ENH_CFG_RGB2YCC_EN	(1 << 2)	/* RGB to YCbCr enable */
#define LCDC_ENH_CFG_GAMMA_EN	(1 << 1)	/* Gamma control enable */
#define LCDC_ENH_CFG_ENH_EN	(1 << 0)	/* Enhancement enable */

/* Color Space Conversion CFG Register */
#define LCDC_ENH_CSCCFG_YCC2RGBMD_BIT	2	/* YCbCr to RGB */
#define LCDC_ENH_CSCCFG_YCC2RGBMD_MASK	(0x03 << LCDC_ENH_CSCCFG_YCC2RGBMD_BIT)
#define LCDC_ENH_CSCCFG_YCC2RGBMD_0	(0 << LCDC_ENH_CSCCFG_YCC2RGBMD_BIT)
#define LCDC_ENH_CSCCFG_YCC2RGBMD_1	(1 << LCDC_ENH_CSCCFG_YCC2RGBMD_BIT)
#define LCDC_ENH_CSCCFG_YCC2RGBMD_2	(2 << LCDC_ENH_CSCCFG_YCC2RGBMD_BIT)
#define LCDC_ENH_CSCCFG_YCC2RGBMD_3	(3 << LCDC_ENH_CSCCFG_YCC2RGBMD_BIT)
/*
 * 00:601WIDE; 01:601NARROW
 * 10:709WIDE; 11:709NARROW
 * WIDE:RGB range 16-235
 * NARROW:RGB range 0-255
*/
#define LCDC_ENH_CSCCFG_RGB2YCCMD_BIT	0	/* RGB to YCbCr */
#define LCDC_ENH_CSCCFG_RGB2YCCMD_MASK	(0x03 << LCDC_ENH_CSCCFG_RGB2YCCMD_BIT)
#define LCDC_ENH_CSCCFG_RGB2YCCMD_0	(0 << LCDC_ENH_CSCCFG_RGB2YCCMD_BIT)
#define LCDC_ENH_CSCCFG_RGB2YCCMD_1	(1 << LCDC_ENH_CSCCFG_RGB2YCCMD_BIT)
#define LCDC_ENH_CSCCFG_RGB2YCCMD_2	(2 << LCDC_ENH_CSCCFG_RGB2YCCMD_BIT)
#define LCDC_ENH_CSCCFG_RGB2YCCMD_3	(3 << LCDC_ENH_CSCCFG_RGB2YCCMD_BIT)

/* LUMA CFG Register */
#define LCDC_ENH_LUMACFG_BRIGHTNESS_BIT	16	/*
						 * Brightness value :0x0-0x7ff
						 * means 0.9999~-0.9999
						 */
#define LCDC_ENH_LUMACFG_BRIGHTNESS_MASK	(0x7ff << LCDC_ENH_LUMACFG_BRIGHTNESS_BIT)

#define LCDC_ENH_LUMACFG_CONTRAST_BIT	0	/*
						 * Contrast value :0x0-0x7ff
						 * means 0~1.9999
						 */
#define LCDC_ENH_LUMACFG_CONTRAST_MASK	(0x7ff << LCDC_ENH_LUMACFG_CONTRAST_BIT)

/* CHROMA0 CFG Register */
#define LCDC_ENH_CHROCFG0_HUE_SIN_BIT	16	/* Hue sin value :0xc00-400 means -1~1 */
#define LCDC_ENH_CHROCFG0_HUE_SIN_MASK	(0xfff << LCDC_ENH_CHROCFG0_HUE_SIN_BIT)
#define LCDC_ENH_CHROCFG0_HUE_COS_BIT	0	/* Hue sin value :0xc00-400 means -1~1 */
#define LCDC_ENH_CHROCFG0_HUE_COS_MASK	(0xfff << LCDC_ENH_CHROCFG0_HUE_COS_BIT)

/* CHROMA1 CFG Register */
#define LCDC_ENH_CHROCFG1_SATURATION_BIT	0	/* Saturation value :0x0-0x7ff means 0~1.9999 */
#define LCDC_ENH_CHROCFG1_SATURATION_MASK	(0x7ff << LCDC_ENH_CHROCFG1_SATURATION_BIT)

/* DITHER CFG Register */
/*
 * 00:8bit dither
 * 01:6bit dither
 * 10:5bit dither
 * 11:4bit dither
*/
#define LCDC_ENH_DITHERCFG_DITHERMD_RED_BIT	4
#define LCDC_ENH_DITHERCFG_DITHERMD_RED_MASK	(0x03 << LCDC_ENH_DITHERCFG_DITHERMD_RED_BIT)
#define LCDC_ENH_DITHERCFG_DITHERMD_GREEN_BIT	2
#define LCDC_ENH_DITHERCFG_DITHERMD_GREEN_MASK	(0x03 << LCDC_ENH_DITHERCFG_DITHERMD_GREEN_BIT)
#define LCDC_ENH_DITHERCFG_DITHERMD_BLUE_BIT	0
#define LCDC_ENH_DITHERCFG_DITHERMD_BLUE_MASK	(0x03 << LCDC_ENH_DITHERCFG_DITHERMD_BLUE_BIT)

/* Enhance Status Register */
#define LCDC_ENH_STATUS_DITHER_DIS	(1 << 9)	/* Dither disable done */
#define LCDC_ENH_STATUS_YCC2RGB_DIS	(1 << 8)	/* YCbCr to RGB disable done */
#define LCDC_ENH_STATUS_SATURATION_DIS	(1 << 7)	/* Saturation disable done */
#define LCDC_ENH_STATUS_VEE_DIS	(1 << 6)	/* Visibility enhance disable done */
#define LCDC_ENH_STATUS_HUE_DIS	(1 << 5)	/* Hue disable done */
#define LCDC_ENH_STATUS_BRIGHTNESS_DIS	(1 << 4)	/* Brightness disable done */
#define LCDC_ENH_STATUS_CONTRAST_DIS	(1 << 3)	/* Contrast disable done */
#define LCDC_ENH_STATUS_RGB2YCC_DIS	(1 << 2)	/* RGB to YCbCr disable done */
#define LCDC_ENH_STATUS_GAMMA_DIS	(1 << 1)	/* Gamma control disable done */

/* GAMMA CFG Register */
#define LCDC_ENH_GAMMA_GAMMA_DATA1_BIT	16	/* Gamma data 1,3,...,1023 */
#define LCDC_ENH_GAMMA_GAMMA_DATA1_MASK	(0x3ff << LCDC_ENH_GAMMA_GAMMA_DATA1_BIT)
#define LCDC_ENH_GAMMA_GAMMA_DATA0_BIT	0	/* Gamma data 0,2,...,1022 */
#define LCDC_ENH_GAMMA_GAMMA_DATA0_MASK	(0x3ff << LCDC_ENH_GAMMA_GAMMA_DATA0_BIT)
#define LCDC_ENH_GAMMA_LEN		(0x800)

/* VEE CFG Register */
#define LCDC_ENH_VEE_VEE_DATA1_BIT	16	/* Vee data 1,3,...,1023 */
#define LCDC_ENH_VEE_VEE_DATA1_MASK	(0x3ff << LCDC_ENH_VEE_VEE_DATA1_BIT)
#define LCDC_ENH_VEE_VEE_DATA0_BIT	0	/* Vee data 0,2,...,1022 */
#define LCDC_ENH_VEE_VEE_DATA0_MASK	(0x3ff << LCDC_ENH_VEE_VEE_DATA0_BIT)
#define LCDC_ENH_VEE_LEN		(0x800)

/* Register Map Of SLCD (Smart LCD Controller) */
#define SLCDC_CFG	(0xA0)	/* SLCD Configure Register */
#define SLCDC_CTRL	(0xA4)	/* SLCD Control Register */
#define SLCDC_STATE	(0xA8)	/* SLCD Status Register */
#define SLCDC_DATA	(0xAC)	/* SLCD Data Register */
#define SLCDC_CFG_NEW   (0xB8)
#define SLCDC_WTIME     (0xB0)
#define SLCDC_TAS       (0xB4)
#define SLCDC_SLOW_TIME (0xBC)

/* SLCD Configure Register */
#define SLCDC_CFG_DWIDTH_BIT	10
#define SLCDC_CFG_DWIDTH_MASK	(0x7 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_DWIDTH_18BIT	(0 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_DWIDTH_16BIT	(1 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_DWIDTH_8BIT_x3	(2 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_DWIDTH_8BIT_x2	(3 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_DWIDTH_8BIT_x1	(4 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_DWIDTH_24BIT	(5 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_DWIDTH_9BIT_x2	(7 << SLCDC_CFG_DWIDTH_BIT)
#define SLCDC_CFG_CWIDTH_BIT	(8)
#define SLCDC_CFG_CWIDTH_MASK	(0x3 << SLCDC_CFG_CWIDTH_BIT)
#define SLCDC_CFG_CWIDTH_16BIT	(0 << SLCDC_CFG_CWIDTH_BIT)
#define SLCDC_CFG_CWIDTH_8BIT	(1 << SLCDC_CFG_CWIDTH_BIT)
#define SLCDC_CFG_CWIDTH_18BIT	(2 << SLCDC_CFG_CWIDTH_BIT)
#define SLCDC_CFG_CWIDTH_24BIT	(3 << SLCDC_CFG_CWIDTH_BIT)
#define SLCDC_CFG_CS_ACTIVE_LOW	(0 << 4)
#define SLCDC_CFG_CS_ACTIVE_HIGH	(1 << 4)
#define SLCDC_CFG_RS_CMD_LOW	(0 << 3)
#define SLCDC_CFG_RS_CMD_HIGH	(1 << 3)
#define SLCDC_CFG_CLK_ACTIVE_FALLING	(0 << 1)
#define SLCDC_CFG_CLK_ACTIVE_RISING	(1 << 1)
#define SLCDC_CFG_TYPE_PARALLEL	(0 << 0)
#define SLCDC_CFG_TYPE_SERIAL	(1 << 0)

/* SLCD New Configure Register */
#define SLCDC_NEW_CFG_DWIDTH_BIT	13
#define SLCDC_NEW_CFG_DWIDTH_MASK	(0x7 << SLCDC_NEW_CFG_DWIDTH_BIT)
#define SLCDC_NEW_CFG_DWIDTH_8BIT	(0 << SLCDC_NEW_CFG_DWIDTH_BIT)
#define SLCDC_NEW_CFG_DWIDTH_9BIT	(1 << SLCDC_NEW_CFG_DWIDTH_BIT)
#define SLCDC_NEW_CFG_DWIDTH_16BIT	(2 << SLCDC_NEW_CFG_DWIDTH_BIT)
#define SLCDC_NEW_CFG_DWIDTH_18BIT	(3 << SLCDC_NEW_CFG_DWIDTH_BIT)
#define SLCDC_NEW_CFG_DWIDTH_24BIT	(4 << SLCDC_NEW_CFG_DWIDTH_BIT)
#define SLCDC_NEW_CFG_6800_MD		(1 << 11)
#define SLCDC_NEW_CFG_CMD_9BIT		(1 << 10)	/* only use in old slcd */
#define SLCDC_NEW_CFG_CMD_16BIT		(0 << 10)	/* only use in old slcd */
#define SLCDC_NEW_CFG_DTIME_BIT		8
#define SLCDC_NEW_CFG_DTIME_MASK	(0x3 << SLCDC_NEW_CFG_DTIME_BIT)
#define SLCDC_NEW_CFG_DTIME_ONCE	(0 << SLCDC_NEW_CFG_DTIME_BIT)
#define SLCDC_NEW_CFG_DTIME_TWICE	(1 << SLCDC_NEW_CFG_DTIME_BIT)
#define SLCDC_NEW_CFG_DTIME_THREE	(2 << SLCDC_NEW_CFG_DTIME_BIT)
#define SLCDC_NEW_CFG_CS_HIGH_IDLE	(0 << 5)
#define SLCDC_NEW_CFG_CS_LOW_IDLE	(1 << 5)
#define SLCDC_NEW_CFG_RS_CMD_LOW	(0 << 4)
#define SLCDC_NEW_CFG_RS_CMD_HIGH	(1 << 4)
#define SLCDC_NEW_CFG_CLK_ACTIVE_FALLING	(0 << 3)
#define SLCDC_NEW_CFG_CLK_ACTIVE_RISING	(1 << 3)
#define SLCDC_NEW_CFG_DTYPE_PARALLEL	(0 << 2)
#define SLCDC_NEW_CFG_DTYPE_SERIAL	(1 << 2)
#define SLCDC_NEW_CFG_CTYPE_PARALLEL	(0 << 1)
#define SLCDC_NEW_CFG_CTYPE_SERIAL	(1 << 1)
#define SLCDC_NEW_CFG_FMT_CONV_EN	(1 << 0)

/* SLCD Control Register */
#define SLCDC_CTRL_NOT_USE_TE   (1 << 8)
#define SLCDC_CTRL_DCSI_SEL	(1 << 7)
#define SLCDC_CTRL_MIPI_MODE	(1 << 6)
#define SLCDC_CTRL_NEW_MODE	(1 << 5)
#define SLCDC_CTRL_FAST_MODE	(1 << 4)
#define SLCDC_CTRL_GATE_MASK	(1 << 3)
#define SLCDC_CTRL_DMA_MODE	(1 << 2)
#define SLCDC_CTRL_DMA_START	(1 << 1)
#define SLCDC_CTRL_DMA_EN	(1 << 0)

/* SLCD Status Register */
#define SLCDC_STATE_BUSY		(1 << 0)

/* SLCD Data Register */
#define SLCDC_DATA_RS_DATA	(0 << 30)
#define SLCDC_DATA_RS_COMMAND	(1 << 30)

static inline unsigned long jzdrm_read(struct drm_device *dev, u32 reg)
{
	struct jzdrm_drm_private *priv = dev->dev_private;
	return ioread32(priv->mmio + reg);
}

static inline void jzdrm_write(struct drm_device *dev, u32 reg, u32 data)
{
	struct jzdrm_drm_private *priv = dev->dev_private;
	iowrite32(data, priv->mmio + reg);
}
#endif /* _REGS_LCDC_H_ */
