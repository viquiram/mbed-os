/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_CLCD_API_H
#define MBED_CLCD_API_H

#include "device.h"

// External Variables from flyer.c
//extern const unsigned short flyerData[];
// External Variables from intro.c
extern const unsigned short introData[];

#if DEVICE_CLCD

#include "GLCD_SPI_MPS2.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct clcd_s clcd_t;

void clcd_init         		 	(clcd_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel);
void clcd_format       		 	(clcd_t *obj, int bits, int mode, int slave);
void clcd_frequency    		 	(clcd_t *obj, int hz);
int  clcd_master_write 		 	(clcd_t *obj, unsigned char reg, int value);
int  clcd_master_write_cmd 	(clcd_t *obj, unsigned char reg);
int  clcd_master_write_fw 	(clcd_t *obj, int value);
int  clcd_slave_receive			(clcd_t *obj);
int  clcd_slave_read   			(clcd_t *obj);
void clcd_slave_write  			(clcd_t *obj, int value);
int  clcd_busy         			(clcd_t *obj);
void clcd_write_start				(clcd_t *obj);
void clcd_write_stop				(clcd_t *obj);
void clcd_Fillscreen 	      (clcd_t *obj, unsigned short int colour);
void clcd_settextcolor   		(unsigned short color);
void clcd_setbackcolor   		(unsigned short color);
void clcd_drawchar       		(unsigned int x,  unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c);
void clcd_displaychar    		(unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c);
void clcd_displaystring  		(unsigned int ln, unsigned int col, unsigned char fi, char *s);
void clcd_clearline        	(unsigned int ln, unsigned char fi);
void clcd_bargraph       		(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val);
void clcd_bitmap         		(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned short *bitmap);
void clcd_bitmapsize 				(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short *bitmap);
void clcd_scrollvertical 		(unsigned int dy);
void clcd_boarder        		(void);
void clcd_box 					 		(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int color);
void clcd_putpixel					(unsigned int x, unsigned int y);
void clcd_putpixelcolor  		(unsigned int x, unsigned int y, unsigned short color);
void clcd_boxsize 					(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short color);	
void clcd_setwindowsize 		(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye);

#ifdef __cplusplus
}
#endif

#endif

#endif
