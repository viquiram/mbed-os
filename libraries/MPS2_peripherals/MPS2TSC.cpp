
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
#include "MPS2TSC.h"

#include "tsc_api.h"

MPS2TSC *MPS2TSC::_owner = NULL;

MPS2TSC::MPS2TSC(PinName sda, PinName scl) {
    // The init function also set the frequency to 100000
    tsc_init(&_tsc, sda, scl);
    _hz = 100000;

    // Used to avoid unnecessary frequency updates
    _owner = this;
}

void MPS2TSC::aquire() {
    if (_owner != this) {
        _owner = this;
    }
}

void MPS2TSC::TSC_INIT(PinName sda, PinName scl) {
    tsc_init(&_tsc, sda, scl);
}
// write - Master Transmitter Mode
void MPS2TSC::TSC_WRITE(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr) {
    aquire();

    TSC_write(reg_addr, data_byte, sadr);

}


// read - Master Reciever Mode
unsigned int MPS2TSC::TSC_READ(unsigned char reg_addr, unsigned char sadr, unsigned char bytes) {
		unsigned int rxdata;
    aquire();

		rxdata = TSC_read(reg_addr, sadr, bytes);
		
		return rxdata;
}

// read - Master Reciever Mode
unsigned int MPS2TSC::TSC_READ_X() {
		unsigned int rxdata;
    aquire();

		rxdata = TSC_read_x();
		
		return rxdata;
}
// read - Master Reciever Mode
unsigned int MPS2TSC::TSC_READ_Y() {
		unsigned int rydata;
    aquire();

		rydata = TSC_read_y();
		
		return rydata;
}

// TSC read pen down, X,Y
unsigned int MPS2TSC::TSC_READXY(unsigned int *PENX, unsigned int *PENY)
{
	unsigned int readxy;
	aquire(); 
	readxy = TSC_readxy(PENX,PENY);
	return readxy;
}
