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
#include "CLCD.h"

#if DEVICE_CLCD

namespace mbed {

CLCD::CLCD(PinName mosi, PinName miso, PinName sclk, PinName _unused) {
    clcd_init(&_clcd, mosi, miso, sclk, NC);
    _bits = 8;
 //   _mode = 0;
//    _hz = 1000000;
//    clcd_format(&_clcd, _bits, _mode, 0);
//    clcd_frequency(&_clcd, _hz);
}

void CLCD::CLCD_INIT(PinName mosi, PinName miso, PinName sclk, PinName _unused) {
    clcd_init(&_clcd, mosi, miso, sclk, NC);
    _bits = 8;
 //   _mode = 0;
//    _hz = 1000000;
//    clcd_format(&_clcd, _bits, _mode, 0);
//    clcd_frequency(&_clcd, _hz);
}

void CLCD::format(int bits, int mode) {
    _bits = bits;
    _mode = mode;
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

void CLCD::frequency(int hz) {
    _hz = hz;
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
}

CLCD* CLCD::_owner = NULL;

// ignore the fact there are multiple physical clcds, and always update if it wasnt us last
void CLCD::aquire() {
     if (_owner != this) {
       // clcd_format(&_clcd, _bits, _mode, 0);
       // clcd_frequency(&_clcd, _hz);
        _owner = this;
    }
}

int CLCD::CLCD_write(unsigned char reg, int value) {
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    return clcd_master_write(&_clcd, reg, value);
}

int CLCD::CLCD_write_cmd(unsigned char reg) {
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    return clcd_master_write_cmd(&_clcd, reg);
}

int CLCD::CLCD_write_fw(int value) {
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    return clcd_master_write_fw(&_clcd, value);
}

void CLCD::CLCD_master_write_start(){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_write_start(&_clcd);
}
void CLCD::CLCD_master_write_stop(){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_write_stop(&_clcd);
}

void CLCD::CLCD_fillscreen(unsigned short int colour){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_Fillscreen(&_clcd, colour);
}

void CLCD::CLCD_settextcolor(unsigned short color){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_settextcolor(color);
}
void CLCD::CLCD_setbackcolor(unsigned short color){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_setbackcolor(color);
}
void CLCD::CLCD_drawchar(unsigned int x,  unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_drawchar(x,y,cw,ch,c);

}
void CLCD::CLCD_displaychar(unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_displaychar(ln,col,fi,c);

}
void CLCD::CLCD_displaystring(unsigned int ln, unsigned int col, unsigned char fi, char *s){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_displaystring(ln,col,fi,s);

}
void CLCD::CLCD_clearline(unsigned int ln, unsigned char fi){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_clearline(ln,fi);

}
void CLCD::CLCD_bargraph(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_bargraph(x,y,w,h,val);

}
void CLCD::CLCD_bitmap(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned short *bitmap){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_bitmap(x,y,w,h,bitmap);

}

void CLCD::CLCD_bitmapsize(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short *bitmap)
{
  CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
  aquire();
	clcd_bitmapsize(xs,xe,ys,ye,bitmap);
	
}

void CLCD::CLCD_scrollvertical(unsigned int dy){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_scrollvertical(dy);
}
void CLCD::CLCD_boarder(void){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_boarder();
}
void CLCD::CLCD_box(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int color){
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_box(x,y,w,h,color);
}

void CLCD::CLCD_putpixel(unsigned int x, unsigned int y)
{
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_putpixel(x,y);
}

void CLCD::CLCD_putpixelcolor(unsigned int x, unsigned int y, unsigned short color)
{
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_putpixelcolor(x,y,color);
}
void CLCD::CLCD_boxsize (unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short color) 
{
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
	clcd_boxsize(xs,xe,ys,ye,color);
}
void CLCD::CLCD_setwindowsize (unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye)
{
    CLCD::_owner = NULL; // Not that elegant, but works. rmeyer
    aquire();
    clcd_setwindowsize (xs,xe,ys,ye);
}

} // namespace mbed

#endif
