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
#ifndef MBED_MPS2_ETHERNET_H
#define MBED_MPS2_ETHERNET_H

#include "platform.h"

class MPS2_Ethernet {

public:

	 /** Send an outgoing ethernet packet.
     *
     *  After filling in the data in an ethernet packet it must be send.
     *  Send will provide a new packet to write to.
     *
     *  @returns
     *    0 if the sending was failed,
     *    or the size of the packet successfully sent.
     */
    int MPS2_transmission(unsigned char * pkt, unsigned int length);

    /** Recevies an arrived ethernet packet.
     *
     *  Receiving an ethernet packet will drop the last received ethernet packet
     *  and make a new ethernet packet ready to read.
     *  If no ethernet packet is arrived it will return 0.
     *
     *  @returns
     *    0 if no ethernet packet is arrived,
     *    or the size of the arrived packet.
     */
    int MPS2_reception(unsigned int *recvbuf, unsigned int *index);
    int MPS2_Ethernet_init();
    int MPS2_mac_address(char *mac);

	unsigned int MPS2_Ethernet_check_ready(void);
	virtual unsigned int MPS2_Ethernet_intf (void);

};


#endif

