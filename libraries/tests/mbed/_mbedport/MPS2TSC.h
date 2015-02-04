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
#ifndef MBED_TSC_H
#define MBED_TSC_H

#include "mbed.h"
#include "tsc_api.h"

/** An TSC Master, used for communicating with TSC slave devices
 *
 * Example:
 * @code
 * // Read from TSC slave at address 0x62
 *
 * #include "mbed.h"
 *
 * TSC tsc(p28, p27);
 *
 * int main() {
 *     int address = 0x62;
 *     char data[2];
 *     tsc.read(address, data, 2);
 * }
 * @endcode
 */
class MPS2TSC {

public:

    /** Create an TSC Master interface, connected to the specified pins
     *
     *  @param sda TSC data line pin
     *  @param scl TSC clock line pin
     */
    MPS2TSC(PinName sda, PinName scl);

    /** Read from an TSC slave
     *
     * Performs a complete read transaction. The bottom bit of
     * the address is forced to 1 to indicate a read.
     *
     *  @param address 8-bit TSC slave address [ addr | 1 ]
     *  @param data Pointer to the byte-array to read data in to
     *  @param length Number of bytes to read
     *  @param repeated Repeated start, true - don't send stop at end
     *
     *  @returns
     *       0 on success (ack),
     *   non-0 on failure (nack)
     */
		unsigned int TSC_READ (unsigned char reg_addr, unsigned char sadr, unsigned char bytes);

    /** Write to an TSC slave
     *
     * Performs a complete write transaction. The bottom bit of
     * the address is forced to 0 to indicate a write.
     *
     *  @param address 8-bit TSC slave address [ addr | 0 ]
     *  @param data Pointer to the byte-array data to send
     *  @param length Number of bytes to send
     *  @param repeated Repeated start, true - do not send stop at end
     *
     *  @returns
     *       0 on success (ack),
     *   non-0 on failure (nack)
     */
		void TSC_WRITE(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr);

		void TSC_INIT(PinName sda, PinName scl);
		
		unsigned int TSC_READ_X ();
		
		unsigned int TSC_READ_Y ();

		unsigned int TSC_READXY(unsigned int *PENX, unsigned int *PENY);


protected:
    void aquire();

    tsc_t _tsc;
    static MPS2TSC  *_owner;
    int         _hz;
};

#endif

