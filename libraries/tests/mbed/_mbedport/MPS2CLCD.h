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
#ifndef MBED_CLCD_H
#define MBED_CLCD_H

#include "mbed.h"
#include "clcd_api.h"

/** A CLCD Master, used for communicating with CLCD slave devices
 *
 * The default format is set to 8-bits, mode 0, and a clock frequency of 1MHz
 *
 * Most CLCD devices will also require Chip Select and Reset signals. These
 * can be controlled using <DigitalOut> pins
 *
 * Example:
 * @code
 * // Send a byte to a CLCD slave, and record the response
 *
 * #include "mbed.h"
 *
 * CLCD device(p5, p6, p7); // mosi, miso, sclk
 *
 * int main() {
 *     int response = device.write(0xFF);
 * }
 * @endcode
 */
class MPS2CLCD {

public:

    /** Create a CLCD master connected to the specified pins
     *
     * Pin Options:
     *  (5, 6, 7) or (11, 12, 13)
     *
     *  mosi or miso can be specfied as NC if not used
     *
     *  @param mosi CLCD Master Out, Slave In pin
     *  @param miso CLCD Master In, Slave Out pin
     *  @param sclk CLCD Clock pin
     */
    MPS2CLCD(PinName mosi, PinName miso, PinName sclk, PinName _unused=NC);

    /** Configure the data transmission format
     *
     *  @param bits Number of bits per CLCD frame (4 - 16)
     *  @param mode Clock polarity and phase mode (0 - 3)
     *
     * @code
     * mode | POL PHA
     * -----+--------
     *   0  |  0   0
     *   1  |  0   1
     *   2  |  1   0
     *   3  |  1   1
     * @endcode
     */
    void format(int bits, int mode = 0);

    /** Set the clcd bus clock frequency
     *
     *  @param hz SCLK frequency in hz (default = 1MHz)
     */
    void frequency(int hz = 1000000);

    /** Write to the CLCD Slave and return the response
     *
     *  @param value Data to be sent to the CLCD slave
     *
     *  @returns
     *    Response from the CLCD slave
    */
		virtual void  CLCD_INIT(PinName mosi, PinName miso, PinName sclk, PinName _unused=NC);

    virtual int 	CLCD_write							(unsigned char reg, int value);
    virtual int 	CLCD_write_cmd					(unsigned char reg);
    virtual int 	CLCD_write_fw						(int value);
		virtual void 	CLCD_master_write_start	();
		virtual void 	CLCD_master_write_stop	();
		virtual void 	CLCD_fillscreen					(unsigned short int col);
		virtual void 	CLCD_settextcolor   		(unsigned short color);
		virtual void 	CLCD_setbackcolor   		(unsigned short color);
		virtual void 	CLCD_drawchar       		(unsigned int x,  unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c);
		virtual void 	CLCD_displaychar    		(unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c);
		virtual void 	CLCD_displaystring  		(unsigned int ln, unsigned int col, unsigned char fi, char *s);
		virtual void 	CLCD_clearline        	(unsigned int ln, unsigned char fi);
		virtual void 	CLCD_bargraph       		(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val);
		virtual void 	CLCD_bitmap         		(unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned short *bitmap);
		virtual void 	CLCD_bitmapsize					(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short *bitmap);
		virtual void 	CLCD_scrollvertical 		(unsigned int dy);
		virtual void 	CLCD_boarder        		(void);
		virtual void 	CLCD_box 					 		  (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int color);
		virtual void  CLCD_putpixel						(unsigned int x, unsigned int y);
		virtual	void  CLCD_putpixelcolor  		(unsigned int x, unsigned int y, unsigned short color);
		virtual void 	CLCD_boxsize 						(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye, unsigned short color);
		virtual void 	CLCD_setwindowsize 			(unsigned int xs, unsigned int xe, unsigned int ys, unsigned int ye);


protected:
    clcd_t _clcd;

    void aquire(void);
    static MPS2CLCD *_owner;
    int _bits;
    int _mode;
    int _hz;
};


#endif


