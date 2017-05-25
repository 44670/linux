/*
 * FB driver for the st75256 OLED Controller
 *
 * Copyright (C) 2013 Noralf Tronnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "fbtft.h"

#define DRVNAME		"fb_st75256"
#define WIDTH		(240)
#define HEIGHT		(160)
#define TXBUFLEN (WIDTH*HEIGHT*2/8)

static void lcd_address(struct fbtft_par *par, int colStart, int colEnd, int pageStart, int pageEnd)
{
	write_reg(par, 0x15, colStart, colEnd); //Set Column Address
	write_reg(par, 0x75, pageStart, pageEnd); //Set Page Address
	write_reg(par, 0x30);
	write_reg(par, 0x5c);
}

/*
 * write_reg() caveat:
 *
 * This doesn't work because D/C has to be LOW for both values:
 * write_reg(par, val1, val2);
 *
 * Do it like this:
 * write_reg(par, val1);
 * write_reg(par, val2);
 */

/* Init sequence taken from the Adafruit st75256 Arduino library */
static int init_display(struct fbtft_par *par)
{
	par->fbtftops.reset(par);

	if (par->gamma.curves[0] == 0) {
		mutex_lock(&par->gamma.lock);
		if (par->info->var.yres == 64)
			par->gamma.curves[0] = 0xCF;
		else
			par->gamma.curves[0] = 0x8F;
		mutex_unlock(&par->gamma.lock);
	}

 write_reg(par, 0x30); //EXT=0
 write_reg(par, 0x94); //Sleep out
 write_reg(par, 0x31); //EXT=1
 write_reg(par, 0xD7, 0X9F); //Autoread disable
 //transfer_data_lcd(0X9F); //
 
 write_reg(par, 0x32, 0x00, 0x01, 0x01); //Analog SET
 //transfer_data_lcd(0x00); //OSC Frequency adjustment
 //transfer_data_lcd(0x01); //Frequency on booster capacitors->6KHz
 //transfer_data_lcd(0x01); //Bias=1/13

 write_reg(par, 0x20, 0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x10, 0x11, 0x13, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f); //灰度设置
 /*
 transfer_data_lcd(0x01);
 transfer_data_lcd(0x03);
 transfer_data_lcd(0x05);
 transfer_data_lcd(0x07);
 transfer_data_lcd(0x09);
 transfer_data_lcd(0x0b);
 transfer_data_lcd(0x0d);
 transfer_data_lcd(0x10);
 transfer_data_lcd(0x11);
 transfer_data_lcd(0x13);
 transfer_data_lcd(0x15);
 transfer_data_lcd(0x17);
 transfer_data_lcd(0x19);
 transfer_data_lcd(0x1b);
 transfer_data_lcd(0x1d);
 transfer_data_lcd(0x1f);*/

 write_reg(par, 0x30); //EXT1=0，EXT0=0,表示选择了“扩展指令表 1”
 write_reg(par, 0x75, 0, 39); //页地址设置
 //transfer_data_lcd(0X00); //起始页地址：YS=0X00
 //transfer_data_lcd(0X14); //结束页地址：YE=0x1F每 4 行为一页，第 0～3 行为第 0 页，第 124～127 行为第 31 页（31=0x1f）
 write_reg(par, 0x15, 16, 255); //列地址设置
 //transfer_data_lcd(0X00); //起始列地址：XS=0
 //transfer_data_lcd(0Xff); //结束列地址：XE=256（0xff）
 
 

 write_reg(par, 0xBC, 0x02); //行列扫描方向
 //transfer_data_lcd(0x02); //MX.MY=Normal

 write_reg(par, 0x0c); //数据格式选择,0x0C 是低位在前 D0-D7,0x08 是高位在前 D7-D0

 write_reg(par, 0xCA, 0x00, 0x9f, 0x20); //显示控制
 //transfer_data_lcd(0X00); //设置 CL 驱动频率：CLD=0
 //transfer_data_lcd(0X9F); //占空比：Duty=160
 //transfer_data_lcd(0X20); //N 行反显：Nline=off

 write_reg(par, 0xF0, 0x11); //显示模式
 //transfer_data_lcd(0X11); //如果设为 0x11：表示选择 4 灰度级模式，如果设为 0x10:表示选择黑白模式

 write_reg(par, 0x81, 0x1c, 0x04); 
 //transfer_data_lcd(0x1c); //微调对比度,可调范围 0x00～0x3f，共 64 级
 //transfer_data_lcd(0x04); //粗调对比度,可调范围 0x00～0x07，共 8 级
 write_reg(par, 0x20, 0x0b); //电源控制
 //transfer_data_lcd(0x0B); //D0=regulator ; D1=follower ; D3=booste, on:1 off:0
 write_reg(par, 0xAF); //打开显示 

	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	lcd_address(par, 16, 255, 0, 39);
}

static int blank(struct fbtft_par *par, bool on)
{

	
	return 0;
}

/* Gamma is used to control Contrast */
static int set_gamma(struct fbtft_par *par, unsigned long *curves)
{


	return 0;
}

static int write_vmem(struct fbtft_par *par, size_t offset, size_t len)
{
	u16 *vmem16 = (u16 *)par->info->screen_buffer;
	u8 *buf = par->txbuf.buf;
	int ret = 0;
	int row, col, i;
	int width = par->info->var.xres;
	int height = par->info->var.yres;
	int bytesPerRow = width * 2 / 8;
	int txBufSize = width * height * 2 / 8;

	set_addr_win(par, 0, 0, width, height);
	memset(buf, 0, txBufSize);
	for (row = 0; row < height; row += 4) {
		for (i = 0; i < 4; i++) {
			for (col = 0; col < width; col ++) {
				u16 pix = vmem16[(row + i) *
						width + col];
				u16 tmp = (pix & 0x1f) + ((pix & 0x7c0) >> 6) + ((pix & 0xF800) >> 11);
				tmp = tmp / 3;
				tmp = tmp >> 3;
				buf[row * bytesPerRow + col] |= ((u8) tmp) << ((i) * 2);
			}
		}
	}

	/* Write data */
	gpio_set_value(par->gpio.dc, 1);
	ret = par->fbtftops.write(par, par->txbuf.buf,
				  txBufSize);
	if (ret < 0)
		dev_err(par->info->device, "write failed and returned: %d\n",
			ret);

	return ret;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.txbuflen = TXBUFLEN,
	.gamma_num = 1,
	.gamma_len = 1,
	.gamma = "00",
	.fbtftops = {
		.write_vmem = write_vmem,
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.blank = blank,
		.set_gamma = set_gamma,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st75256", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st75256");
MODULE_ALIAS("platform:st75256");

MODULE_DESCRIPTION("st75256 LCD Driver");
MODULE_AUTHOR("None");
MODULE_LICENSE("GPL");
