
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
 
/*******************************************************************************
 * Copyright (c) 2011 - 2014 ARM LIMITED
 * 
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of ARM nor the names of its contributors may be used
 *   to endorse or promote products derived from this software without
 *   specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include "AACI.h"

#if DEVICE_AACI

namespace mbed {

AACI *AACI::_owner = NULL;

AACI::AACI(PinName sda, PinName scl) {
    // The init function also set the frequency to 100000
    audio_aaci_init(&_aaci);
    _hz = 100000;

    // Used to avoid unnecessary frequency updates
    _owner = this;
}

void AACI::aquire() {
    if (_owner != this) {
        _owner = this;
    }
}

unsigned char AACI::AACI_INIT() {
	aquire();
    return audio_aaci_init(&_aaci);
}
// write - Master Transmitter Mode
void AACI::AACI_WRITE_SINE(){ //unsigned char reg_addr, unsigned char data_byte, unsigned char sadr) {
    aquire();

    audio_aaci_out_sine(); //reg_addr, data_byte, sadr);

}


// read - Master Reciever Mode
unsigned int AACI::AACI_READ() {
		unsigned int rxdata;
    aquire();

		rxdata = audio_aaci_read();
		
		return rxdata;
}

// read - Master Reciever Mode
void AACI::AACI_WRITE(unsigned int data) {
    aquire();

		audio_aaci_write(data);
}

void AACI::AACI_VOLUME(unsigned int data) {
    aquire();

		audio_aaci_volume(data);
}

unsigned char AACI::AUDIO_I2C_READ (unsigned char reg_addr, unsigned char sadr)
{
	return audio_i2c_read(reg_addr,sadr);
}

void AACI::AUDIO_I2C_WRITE(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr)
{
	audio_i2c_write(reg_addr,data_byte,sadr);		
}




} // namespace mbed

#endif
