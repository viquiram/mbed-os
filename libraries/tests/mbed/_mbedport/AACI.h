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
#ifndef MBED_AACI_H
#define MBED_AACI_H

#include "platform.h"

#if DEVICE_AACI

#include "audio_aaci_api.h"

namespace mbed {

/** An AACI Master, used for communicating with AACI slave devices
 *
 * Example:
 * @code
 * // Read from AACI slave at address 0x62
 *
 * #include "mbed.h"
 *
 * AACI aaci(p28, p27);
 *
 * int main() {
 *     int address = 0x62;
 *     char data[2];
 *     aaci.read(address, data, 2);
 * }
 * @endcode
 */
class AACI {

public:

    /** Create an AACI Master interface, connected to the specified pins
     *
     *  @param sda AACI data line pin
     *  @param scl AACI clock line pin
     */
    AACI(PinName sda, PinName scl);

    /** Read from an AACI slave
     *
     * Performs a complete read transaction. 
     *
     *  @returns
     *   int value of the read
     */
		unsigned int AACI_READ();

    /** Write to AACI output
     *
     * Performs a complete write transaction.      *
     *  @param data Pointer to the byte-array data to send
     */
		void AACI_WRITE(unsigned int data);//unsigned char reg_addr, unsigned char data_byte, unsigned char sadr);
		
		/** setting the volume of the audio output, range is 1 to 127 
		* 1 being the highest (loudest) volume level and 127 being the lowest (quietest).
		*/
		void AACI_VOLUME(unsigned int data);//unsigned char reg_addr, unsigned char data_byte, unsigned char sadr);
		
		void AACI_WRITE_SINE(void);//unsigned char reg_addr, unsigned char data_byte, unsigned char sadr);

		unsigned char AACI_INIT(void); //PinName sda, PinName scl);
		
		unsigned char AUDIO_I2C_READ (unsigned char reg_addr, unsigned char sadr);

		void AUDIO_I2C_WRITE(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr);

		
		
		/*unsigned int AACI_READ_X ();
		
		unsigned int AACI_READ_Y ();*/


protected:
    void aquire();

    aaci_t _aaci;
    static AACI  *_owner;
    int         _hz;
};

} // namespace mbed

#endif

#endif

