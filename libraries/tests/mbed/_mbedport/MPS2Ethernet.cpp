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
#include "MPS2Ethernet.h"

#include "ethernet_api.h"

MPS2Ethernet::MPS2Ethernet() {
    ethernet_init();
}

MPS2Ethernet::~MPS2Ethernet() {
    ethernet_free();
}

int MPS2Ethernet::write(const char *data, int size) {
    return ethernet_write(data, size);
}

int MPS2Ethernet::send() {
    return ethernet_send();
}

int MPS2Ethernet::receive() {
    return ethernet_receive();
}

int MPS2Ethernet::read(char *data, int size) {
    return ethernet_read(data, size);
}

void MPS2Ethernet::address(char *mac) {
    return ethernet_address(mac);
}

int MPS2Ethernet::link() {
    return ethernet_link();
}

void MPS2Ethernet::set_link(Mode mode) {
    int speed = -1;
    int duplex = 0;

    switch(mode) {
        case AutoNegotiate : speed = -1; duplex = 0; break;
        case HalfDuplex10  : speed = 0;  duplex = 0; break;
        case FullDuplex10  : speed = 0;  duplex = 1; break;
        case HalfDuplex100 : speed = 1;  duplex = 0; break;
        case FullDuplex100 : speed = 1;  duplex = 1; break;
    }

    ethernet_set_link(speed, duplex);
}

int MPS2Ethernet::Ethernet_init() {
    return ethernet_init();
}

int MPS2Ethernet::transmission(unsigned char * pkt, unsigned int length) {
    return ethernet_transmission(pkt,length);
}

int MPS2Ethernet::reception(unsigned int *recvbuf, unsigned int *index) {
    return ethernet_reception((unsigned int *)recvbuf, index);
}

int MPS2Ethernet::mac_address(char *mac) {
    return ethernet_mac_address(mac);
}

unsigned int MPS2Ethernet::Ethernet_check_ready(void)
{
	return ethernet_check_ready();
}

unsigned int MPS2Ethernet :: Ethernet_intf (void)
{
	return ethernet_intf();
}

