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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#include "clcd_api.h"
#include "spi_def.h"
#include "cmsis.h"
#include "pinmap.h"
#include "error.h"
#include "wait_api.h"
//#include "icons.h"
//#include "clcd_api_funcs.h"
//#include "clcd_api_funcs.c"
//#include "flyer.c"
#include "GLCD_SPI_MPS2.h"
//#include "GLCD_SPI_MPS2.c"



static const PinMap PinMap_CLCD_SCLK[] = {
//    {SCLK_SPI , SPI_0, 0},
    {CLCD_SCLK , SPI_1, 0},
    {NC   , NC   , 0}
};

static const PinMap PinMap_CLCD_MOSI[] = {
//    {MOSI_SPI, SPI_0, 0},
    {CLCD_MOSI, SPI_1, 0},
    {NC   , NC   , 0}
};

static const PinMap PinMap_CLCD_MISO[] = {
//    {MISO_SPI, SPI_0, 0},
    {CLCD_MISO, SPI_1, 0},
    {NC   , NC   , 0}
};

static const PinMap PinMap_CLCD_SSEL[] = {
//    {SSEL_SPI, SPI_0, 0},
    {CLCD_SSEL, SPI_1, 0},
    {NC   , NC   , 0}
};

static inline int ssp2_disable(clcd_t *obj);
static inline int ssp2_enable(clcd_t *obj);

void clcd_init(clcd_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel) {
    // determine the SPI to use
    SPIName clcd_mosi = (SPIName)pinmap_peripheral(mosi, PinMap_CLCD_MOSI);
    SPIName clcd_miso = (SPIName)pinmap_peripheral(miso, PinMap_CLCD_MISO);
    SPIName clcd_sclk = (SPIName)pinmap_peripheral(sclk, PinMap_CLCD_SCLK);
    SPIName clcd_ssel = (SPIName)pinmap_peripheral(ssel, PinMap_CLCD_SSEL);
    SPIName clcd_data = (SPIName)pinmap_merge(clcd_mosi, clcd_miso);
    SPIName clcd_cntl = (SPIName)pinmap_merge(clcd_sclk, clcd_ssel);
    obj->clcd = (MPS2_SSP_TypeDef*)pinmap_merge(clcd_data, clcd_cntl);
    if ((int)obj->clcd == NC) {
        error("SPI pinout mapping failed");
    }
    
    // enable power and clocking
//    switch ((int)obj->clcd) {
 //       case SPI_0: 
					
/*					obj->clcd->CR1   	= 0;
					obj->clcd->CR0   	= SSPCR0_SCR_DFLT | SSPCR0_FRF_MOT | SSPCR0_DSS_8;
					obj->clcd->CPSR  	= SSPCPSR_DFLT; 
					obj->clcd->IMSC  	= 0x8; 
					obj->clcd->DMACR 	= 0;
				  obj->clcd->CR1   	= SSPCR1_SSE;
					obj->clcd->ICR   	= 0x3;  
//        case SPI_1:
*/	//				apCLCD_init(obj);
		// CLCD screen register setup
    GLCD_Initialize();

    // Fill CLCD screen
    GLCD_Clear(Black);

    // Draw intro screen image
    //GLCD_Bitmap (0, 0, 320, 240, (unsigned short *)flyerData);

		
				
//    }
    
    // set default format and frequency
/*    if (ssel == NC) {
        clcd_format(obj, 8, 0, 0);  // 8 bits, mode 0, master
    } else {
        clcd_format(obj, 8, 0, 1);  // 8 bits, mode 0, slave
    }
    clcd_frequency(obj, 6600000);*/
    
    // enable the ssp2 channel
    ssp2_enable(obj);

    // pin out the clcd pins
    pinmap_pinout(mosi, PinMap_CLCD_MOSI);
    pinmap_pinout(miso, PinMap_CLCD_MISO);
    pinmap_pinout(sclk, PinMap_CLCD_SCLK);
    if (ssel != NC) {
        pinmap_pinout(ssel, PinMap_CLCD_SSEL);
    }
}

void clcd_format(clcd_t *obj, int bits, int mode, int slave) {
    ssp2_disable(obj);
    if (!(bits >= 4 && bits <= 16) || !(mode >= 0 && mode <= 3)) {
        error("SPI format error");
    }
    
    int polarity = (mode & 0x2) ? 1 : 0;
    int phase = (mode & 0x1) ? 1 : 0;
    
    // set it up
    int DSS = bits - 1;            // DSS (data select size)
    int SPO = (polarity) ? 1 : 0;  // SPO - clock out polarity
    int SPH = (phase) ? 1 : 0;     // SPH - clock out phase
    
    int FRF = 0;                   // FRF (frame format) = SPI
    uint32_t tmp = obj->clcd->CR0;
    tmp &= ~(0xFFFF);
    tmp |= DSS << 0
        | FRF << 4
        | SPO << 6
        | SPH << 7;
    obj->clcd->CR0 = tmp;
    
    tmp = obj->clcd->CR1;
    tmp &= ~(0xD);
    tmp |= 0 << 0                   // LBM - loop back mode - off
        | ((slave) ? 1 : 0) << 2   // MS - master slave mode, 1 = slave
        | 0 << 3;                  // SOD - slave output disable - na
    obj->clcd->CR1 = tmp;
    
    ssp2_enable(obj);
}

void clcd_frequency(clcd_t *obj, int hz) {
    ssp2_disable(obj);
    
//    // setup the clcd clock diveder to /1
//    switch ((int)obj->clcd) {
//        case SPI_0:
//            LPC_SC->PCLKSEL1 &= ~(3 << 10);
//            LPC_SC->PCLKSEL1 |=  (1 << 10);
//            break;
//        case SPI_1:
//            LPC_SC->PCLKSEL0 &= ~(3 << 20);
//            LPC_SC->PCLKSEL0 |=  (1 << 20);
//            break;
//    }
    
    uint32_t PCLK = SystemCoreClock;
    
    int prescaler;
    
    for (prescaler = 2; prescaler <= 254; prescaler += 2) {
        int prescale_hz = PCLK / prescaler;
        
        // calculate the divider
        int divider = floor(((float)prescale_hz / (float)hz) + 0.5f);
        
        // check we can support the divider
        if (divider < 256) {
            // prescaler
            obj->clcd->CPSR = prescaler;
            
            // divider
            obj->clcd->CR0 &= ~(0xFFFF << 8);
            obj->clcd->CR0 |= (divider - 1) << 8;
            ssp2_enable(obj);
            return;
        }
    }
    error("Couldn't setup requested SPI frequency");
}

static inline int ssp2_disable(clcd_t *obj) {
    return obj->clcd->CR1 &= ~(1 << 1);
}

static inline int ssp2_enable(clcd_t *obj) {
    return obj->clcd->CR1 |= (1 << 1);
}

static inline int ssp2_readable(clcd_t *obj) {
    return obj->clcd->SR & (1 << 2);
}

static inline int ssp2_writeable(clcd_t *obj) {
    return obj->clcd->SR & (1 << 1);
}

/*static inline void ssp2_write(clcd_t *obj, unsigned char reg, int value) {
					while (!ssp2_writeable(obj));
				  wr_reg(reg, value);
}

static inline void ssp2_write_cmd(clcd_t *obj, unsigned char reg) {
					while (!ssp2_writeable(obj));
				  wr_cmd(reg);
}

static inline void ssp2_write_fw(clcd_t *obj, int value) {
					while (!ssp2_writeable(obj));
				  wr_dat_only(value);
}*/

static inline int ssp2_read(clcd_t *obj) {
//    while (!ssp2_readable(obj));
    return obj->clcd->DR;
}

static inline int ssp2_busy(clcd_t *obj) {
    return (obj->clcd->SR & (1 << 4)) ? (1) : (0);
}

int clcd_master_write(clcd_t *obj, unsigned char reg, int value) {
		while (!ssp2_writeable(obj));
		GLCD_WrReg(reg, value);
    return ssp2_read(obj);
}

int clcd_master_write_cmd(clcd_t *obj, unsigned char reg) {
		while (!ssp2_writeable(obj));
		GLCD_WrCmd(reg);
    return ssp2_read(obj);
}

int clcd_master_write_fw(clcd_t *obj, int value) {
		while (!ssp2_writeable(obj));
		GLCD_Write(value);
		//clcd_fw(obj,value);
    return ssp2_read(obj);
}

int clcd_slave_receive(clcd_t *obj) {
    return (ssp2_readable(obj) && !ssp2_busy(obj)) ? (1) : (0);
}

int clcd_slave_read(clcd_t *obj) {
    return obj->clcd->DR;
}

void clcd_slave_write(clcd_t *obj, int value) {
    while (ssp2_writeable(obj) == 0) ;
    obj->clcd->DR = value;
}

int clcd_busy(clcd_t *obj) {
    return ssp2_busy(obj);
}

void clcd_write_start(clcd_t *obj){
	GLCD_Start();
}
void clcd_write_stop(clcd_t *obj){
	GLCD_End();
}

void clcd_Fillscreen(clcd_t *obj, unsigned short int colour) {
    //clcd_fillscreen(obj,colour);
		GLCD_Clear(colour);
}

void clcd_settextcolor(unsigned short color){
	GLCD_SetTextColor(color);
}
void clcd_setbackcolor(unsigned short color){
	GLCD_SetBackColor(color);
}
void clcd_drawchar(unsigned int x,  unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c){
	GLCD_DrawChar(x,y,cw,ch,c);

}
void clcd_displaychar(unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c){
	GLCD_DisplayChar(ln,col,fi,c);

}
void clcd_displaystring(unsigned int ln, unsigned int col, unsigned char fi, char *s){
	GLCD_DisplayString(ln,col,fi,s);

}
void clcd_clearline(unsigned int ln, unsigned char fi){
	GLCD_ClearLn(ln,fi);

}
void clcd_bargraph(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val){
	GLCD_Bargraph(x,y,w,h,val);

}
void clcd_bitmap(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned short *bitmap){
	GLCD_Bitmap(x,y,w,h,bitmap);

}
void clcd_scrollvertical(unsigned int dy){
	GLCD_ScrollVertical(dy);
}
void clcd_boarder(void){
	GLCD_Boarder();
}
void clcd_box(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int color){
	GLCD_Box(x,y,w,h,color);
}

void clcd_putpixel(unsigned int x, unsigned int y)
{
	GLCD_PutPixel(x,y);
}

void clcd_putpixelcolor(unsigned int x, unsigned int y, unsigned short color)
{
	GLCD_PutPixelColor(x,y,color);
}
/*******************************************************************************
* Draw box filled with color                                                   *
*   Parameter:      xs:        horizontal start position                       *
*                   xe:        horizontal end position                         *
*                   ys:        vertical start position                         *
*                   ye:        vertical end position                           *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void clcd_boxsize (unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short color) 
{
	GLCD_BoxSize(xs,xe,ys,ye,color);
}
void clcd_setwindowsize (unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye)
{
    GLCD_SetWindowSize (xs,xe,ys,ye);
}
void clcd_bitmapsize (unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short *bitmap)
{
	GLCD_BitmapSize(xs,xe,ys,ye,bitmap);
}
