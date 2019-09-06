/* include/sound/ak4376.h
 *
 * Copyright (C) 2011 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __SOUND_ak4377_H
#define __SOUND_ak4377_H
#define  uint8   unsigned char
#define  uint16  unsigned short
#define  int16   short
#define  uint32  unsigned int
#define  int32   int
#define  uint64  unsigned long

extern struct ak4377 *ak4377;
extern int ak4377_i2c_read(struct ak4377 *bq, u8 reg);
extern int ak4377_i2c_write(struct ak4377 *bq, u8 reg, u8 val);
extern unsigned char AK4377_ReadOneByte(uint8 ReadAddr);
extern void AK4377_WriteOneByte(uint8 WriteAddr,uint8 DataToWrite);
extern unsigned char volumeL;
extern unsigned char volumeR;
extern int work_sign;
extern unsigned char dsd_volume;
extern void ak4377_cache_reg_init(void);
extern void ak4377_reg_init(void);
extern bool ak4377_power_mode;
//extern void set_ak4377_high_256k_sample();
//extern void set_ak4377_low_256k_sample();
extern void ak4377_sample_set(int sample,int power_flag);

#define AK4376_IOC_MAGIC  '4'
#define AK4376_POWER_ON  _IO(AK4376_IOC_MAGIC,0x24)
#define AK4376_POWER_DOWN  _IO(AK4376_IOC_MAGIC,0x25)
#define AK4377_DSD_MODE _IO(AK4376_IOC_MAGIC,0x26)
#define AK4377_PCM_MODE _IO(AK4376_IOC_MAGIC,0x27)
#endif

