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
#include <string.h>

#include "ethernet_api.h"
#include "cmsis.h"
#include "mbed_interface.h"
#include "toolchain.h"
#include "error.h"
#include "ETH_MPS2.h"
#include "wait_api.h"

#define TX_PKT_SIZE 256
#define RX_PKT_SIZE 300

//static unsigned char txpkt[TX_PKT_SIZE];

//static unsigned char rxpkt[RX_PKT_SIZE];
// Types
#undef FALSE
#undef TRUE
#define FALSE   0
#define TRUE    1


// For debugging the tests.
//#define DEBUG
#ifdef DEBUG
    #define apDebug(...)      printf(__VA_ARGS__)
#else
    #define apDebug(...)
#endif  // ifdef DEBUG

#ifdef SEMIHOST
#define printf_err(...)                         \
    do {                                        \
        printf(__VA_ARGS__);                    \
        if (RunAllTests) {                      \
            logfile = fopen(logfilepath, "a");  \
            fprintf(logfile, __VA_ARGS__);      \
            fclose(logfile);                    \
        }                                       \
    } while(0)
#else
#define printf_err(...)                         \
    do {                                        \
        printf(__VA_ARGS__);                    \
    } while(0)
#endif

int smsc9220_check_id(void)
{
    int error;
    unsigned int id;
    error = 0;

    id = smsc9220_read_id();

    // If bottom and top halves of the word are the same
    if(((id >> 16) & 0xFFFF) == (id & 0xFFFF)) {
        apDebug("Error: The SMSC9220 bus is in 16-bit mode. 32-bit mode was expected.\n");
        error = 1;
        return error;
    }
    switch(((id >> 16) & 0xFFFF)) {
        case 0x9220:
            apDebug("SMSC9220 is identified successfully.\n");
            break;

        default:
            apDebug("SMSC9220 id reads: %#08x, either an unknown chip (SMSC%x?), or error.\n",
                    ((id >> 16) & 0xFFFF), ((id >> 16) & 0xFFFF));
            error = 1;
            break;
    }

    return error;
}

int smsc9220_check_macaddress(void)
{
    int error;
    const unsigned int mac_valid_high = 0xC00A;
    const unsigned int mac_valid_low = 0x00F70200;
    unsigned int mac_low;
    unsigned int mac_high;

    error = 0;

    // Read current mac address.
    smsc9220_mac_regread(SMSC9220_MAC_ADDRH, &mac_high);
    smsc9220_mac_regread(SMSC9220_MAC_ADDRL, &mac_low);

    // Print mac address in 2-nibble (8-bit) parts.
    apDebug("MAC address read: %02X:%02X:%02X:%02X:%02X:%02X:\n",
            mac_low & 0xFF, (mac_low >> 8) & 0xFF,
            (mac_low >> 16) & 0xFF, (mac_low >> 24) & 0xFF,
            mac_high & 0xFF, (mac_high >> 8) & 0xFF);

    // If it's invalid, assign a temporary valid address:
    if(mac_high == 0xFFFF || mac_low == 0xFFFFFFFF ||
        mac_high == 0x00000000 || mac_low == 0x00000000 ||
        (mac_low & (1 << 15))) {
        apDebug("MAC Address is invalid. Assigning temporary address.\n");
    } else {
        apDebug("Assigning temporary MAC address.\n");
    }

    // Writing temporary address:
    smsc9220_mac_regwrite(SMSC9220_MAC_ADDRH, mac_valid_high);
    smsc9220_mac_regwrite(SMSC9220_MAC_ADDRL, mac_valid_low);

    // Verify write was correct:
    smsc9220_mac_regread(SMSC9220_MAC_ADDRH, &mac_high);
    smsc9220_mac_regread(SMSC9220_MAC_ADDRL, &mac_low);

    apDebug("MAC address after modification: %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_low & 0xFF, (mac_low >> 8) & 0xFF,
            (mac_low >> 16) & 0xFF, (mac_low >> 24) & 0xFF,
            mac_high & 0xFF, (mac_high >> 8) & 0xFF);

    if(mac_high != mac_valid_high || mac_low != mac_valid_low) {
        apDebug("Writing temporary mac address failed.\n");
        apDebug("MAC Address written was: mac_high: %#08x, mac_low: %#08x\n",
               mac_valid_high, mac_valid_low);
        error = TRUE;
        return error;
    }

    apDebug("\n");
    return error;
}

void smsc9220_print_mac_registers()
{
    unsigned int read;
    int i;

    i = 0;
    read = 0;

    for(i = 1; i <= 0xC; i++) {
        smsc9220_mac_regread(i, &read);
        apDebug("MAC Register %d: %#08x\n",i,read);
    }

    apDebug("\n");
    return;
}
static void smsc9220_print_phy_registers()
{
    unsigned short read;
    unsigned int i;

    i = 0;
    read = 0;
    for(i = 0; i <= 6; i++) {
        smsc9220_phy_regread(i, &read);
        apDebug("PHY Register %d: %#08x\n",i,read);
    }
    smsc9220_phy_regread(i = 17, &read);
    apDebug("Phy Register %d: %#08x\n", i, read);

    smsc9220_phy_regread(i = 18, &read);
    apDebug("Phy Register %d: %#08x\n", i, read);

    smsc9220_phy_regread(i = 27, &read);
    apDebug("Phy Register %d: %#08x\n", i, read);

    smsc9220_phy_regread(i = 29, &read);
    apDebug("Phy Register %d: %#08x\n", i, read);

    smsc9220_phy_regread(i = 30, &read);
    apDebug("Phy Register %d: %#08x\n", i, read);

    smsc9220_phy_regread(i = 31, &read);
    apDebug("Phy Register %d: %#08x\n", i, read);

    apDebug("\n");
    return;
}

/*----------------------------------------------------------------------------
  Ethernet Device initialize
 *----------------------------------------------------------------------------*/

int ethernet_transmission(unsigned char * pkt, unsigned int length)
{
	smsc9220_xmit_packet(pkt, length);
	return 0;
}

int ethernet_reception(unsigned int *recvbuf, unsigned int *index) 
{
	return smsc9220_recv_packet((unsigned int *)recvbuf, index);
}

int ethernet_mac_address(char *mac) 
{
    return smsc9220_check_macaddress();
}

unsigned int ethernet_check_ready(void)
{
	return smsc9220_check_ready();
}

unsigned int ethernet_intf()
{
  unsigned int txfifo_inf;
 
	txfifo_inf = SMSC9220->TX_FIFO_INF;
	
	return txfifo_inf;

}
