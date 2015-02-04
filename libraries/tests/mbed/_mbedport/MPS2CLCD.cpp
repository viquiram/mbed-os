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
#include "mbed.h"
#include "MPS2CLCD.h"

#include "clcd_api.h"


MPS2CLCD::MPS2CLCD(PinName mosi, PinName miso, PinName sclk, PinName _unused) {
    clcd_init(&_clcd, mosi, miso, sclk, NC);
    _bits = 8;
 //   _mode = 0;
//    _hz = 1000000;
//    clcd_format(&_clcd, _bits, _mode, 0);
//    clcd_frequency(&_clcd, _hz);
}

void MPS2CLCD::CLCD_INIT(PinName mosi, PinName miso, PinName sclk, PinName _unused) {
    clcd_init(&_clcd, mosi, miso, sclk, NC);
    _bits = 8;
 //   _mode = 0;
//    _hz = 1000000;
//    clcd_format(&_clcd, _bits, _mode, 0);
//    clcd_frequency(&_clcd, _hz);
}

void MPS2CLCD::format(int bits, int mode) {
    _bits = bits;
    _mode = mode;
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

void MPS2CLCD::frequency(int hz) {
    _hz = hz;
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

MPS2CLCD* MPS2CLCD::_owner = NULL;

// ignore the fact there are multiple physical clcds, and always update if it wasnt us last
void MPS2CLCD::aquire() {
     if (_owner != this) {
       // clcd_format(&_clcd, _bits, _mode, 0);
       // clcd_frequency(&_clcd, _hz);
        _owner = this;
    }
}

int MPS2CLCD::CLCD_write(unsigned char reg, int value) {
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    return clcd_master_write(&_clcd, reg, value);
}

int MPS2CLCD::CLCD_write_cmd(unsigned char reg) {
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    return clcd_master_write_cmd(&_clcd, reg);
}

int MPS2CLCD::CLCD_write_fw(int value) {
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    return clcd_master_write_fw(&_clcd, value);
}

void MPS2CLCD::CLCD_master_write_start(){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_write_start(&_clcd);
}
void MPS2CLCD::CLCD_master_write_stop(){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_write_stop(&_clcd);
}

void MPS2CLCD::CLCD_fillscreen(unsigned short int colour){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_Fillscreen(&_clcd, colour);
}

void MPS2CLCD::CLCD_settextcolor(unsigned short color){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_settextcolor(color);
}
void MPS2CLCD::CLCD_setbackcolor(unsigned short color){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_setbackcolor(color);
}
void MPS2CLCD::CLCD_drawchar(unsigned int x,  unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_drawchar(x,y,cw,ch,c);

}
void MPS2CLCD::CLCD_displaychar(unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_displaychar(ln,col,fi,c);

}
void MPS2CLCD::CLCD_displaystring(unsigned int ln, unsigned int col, unsigned char fi, char *s){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_displaystring(ln,col,fi,s);

}
void MPS2CLCD::CLCD_clearline(unsigned int ln, unsigned char fi){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_clearline(ln,fi);

}
void MPS2CLCD::CLCD_bargraph(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_bargraph(x,y,w,h,val);

}
void MPS2CLCD::CLCD_bitmap(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned short *bitmap){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_bitmap(x,y,w,h,bitmap);

}

void MPS2CLCD::CLCD_bitmapsize(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short *bitmap)
{
  MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
  aquire();
	clcd_bitmapsize(xs,xe,ys,ye,bitmap);
	
}

void MPS2CLCD::CLCD_scrollvertical(unsigned int dy){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_scrollvertical(dy);
}
void MPS2CLCD::CLCD_boarder(void){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_boarder();
}
void MPS2CLCD::CLCD_box(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int color){
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_box(x,y,w,h,color);
}

void MPS2CLCD::CLCD_putpixel(unsigned int x, unsigned int y)
{
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_putpixel(x,y);
}

void MPS2CLCD::CLCD_putpixelcolor(unsigned int x, unsigned int y, unsigned short color)
{
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_putpixelcolor(x,y,color);
}
void MPS2CLCD::CLCD_boxsize (unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short color) 
{
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_boxsize(xs,xe,ys,ye,color);
}
void MPS2CLCD::CLCD_setwindowsize (unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye)
{
    MPS2CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    clcd_setwindowsize (xs,xe,ys,ye);
}

