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
#include "i2c_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"
//#include "MPS2_RESOURCE_PACK.h"
//#include "mps2_i2c_api.h"
//#include "TSC_I2C_MPS2.c"
//#include "AAIC_I2C_MPS2.h"
//#include "AAIC_I2C_MPS2.c"
#include "wait_api.h"
//#include "fpga.c"
//#include "tsc_api.h"

// Types
#undef FALSE
#undef TRUE
#define FALSE   0
#define TRUE    1


static const PinMap PinMap_I2C_SDA[] = {
    {TSC_SDA, I2C_0, 0},
    {AUD_SDA, I2C_1, 0},
//    {SDA3, I2C_2, 0},
    {NC   , NC   , 0}
};

static const PinMap PinMap_I2C_SCL[] = {
    {TSC_SCL, I2C_0, 0},
    {AUD_SCL, I2C_1, 0},
 //   {SCL3, I2C_2, 0},
    {NC   , NC,    0}
};


void i2c_init(i2c_t *obj, PinName sda, PinName scl) {
	

    // determine the SPI to use
    I2CName i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
    I2CName i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);
    obj->i2c = (MPS2_I2C_TypeDef *)pinmap_merge(i2c_sda, i2c_scl);
    
    if ((int)obj->i2c == NC) {
        error("I2C pin mapping failed");
    }
    switch ((int)obj->i2c) {
        case I2C_0: mps2_i2c_ts_init(); break;
        case I2C_1: mps2_i2c_aaci_init(); break;    // Read and check the I2C chip ID and revision
    }
		
    pinmap_pinout(sda, PinMap_I2C_SDA);
    pinmap_pinout(scl, PinMap_I2C_SCL);
}

inline int i2c_start(i2c_t *obj) {
/*
    int status = 0;
        // Start bit
    switch ((int)obj->i2c) {
			case I2C_0: i2c_delay(TSC_TSU);
									MPS2_TS_I2C->CONTROLS = SDA | SCL;
									i2c_delay(TSC_TSU);
									MPS2_TS_I2C->CONTROLC = SDA;
									i2c_delay(TSC_TSU);
									break;
        case I2C_1: mps2_i2c_aaci_init(); break;    // Read and check the I2C chip ID and revision
		}

    return status;*/
    return 0;
}

inline int i2c_stop(i2c_t *obj) {
    //int timeout = 0;
    // Stop bit
/*		switch ((int)obj->i2c) {
			case I2C_0: i2c_delay(TSC_TSU);
									MPS2_TS_I2C->CONTROLC = SDA;
									i2c_delay(TSC_TSU);
									MPS2_TS_I2C->CONTROLS = SCL;
									i2c_delay(TSC_TSU);
									MPS2_TS_I2C->CONTROLS = SDA;
									i2c_delay(TSC_TSU);
									break;
        case I2C_1: mps2_i2c_aaci_init(); break;    // Read and check the I2C chip ID and revision
		}
*/
		return 0;
}



void i2c_frequency(i2c_t *obj, int hz) {
/*    // [TODO] set pclk to /4
    uint32_t PCLK = SystemCoreClock / 4;
    
    uint32_t pulse = PCLK / (hz * 2);
    
    // I2C Rate
    I2C_SCLL(obj, pulse);
    I2C_SCLH(obj, pulse);*/
}

// The I2C does a read or a write as a whole operation
// There are two types of error conditions it can encounter
//  1) it can not obtain the bus
//  2) it gets error responses at part of the transmission
//
// We tackle them as follows:
//  1) we retry until we get the bus. we could have a "timeout" if we can not get it
//      which basically turns it in to a 2)
//  2) on error, we use the standard error mechanisms to report/debug
//
// Therefore an I2C transaction should always complete. If it doesn't it is usually
// because something is setup wrong (e.g. wiring), and we don't need to programatically
// check for that

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop) {
/*	int loop,din,isr;
	
	// Read interrupt status
    	isr = TSC_I2C_read(0x0B, TSC_I2C_ADDR, 1);

   	// Test for FIFO_TH interrupt
    if (isr & 0x02)
    {
			// Empty the FIFO
			loop = TSC_I2C_read(0x4C, TSC_I2C_ADDR, 1);
			while (loop > 1)
			{
				din = TSC_I2C_read(0xD7, TSC_I2C_ADDR, 4);
				loop--;
			}

			// Clear the interrupt (must be immediately after FIFO empty)
			TSC_I2C_write(0x0B, isr, TSC_I2C_ADDR);

			// Read coordinates
			din = TSC_I2C_read(0xD7, TSC_I2C_ADDR, 4);
		}

*/
/*	int count, status;
    
    status = i2c_start(obj);
    
    if ((status != 0x10) && (status != 0x08)) {
        i2c_stop(obj);
        return I2C_ERROR_BUS_BUSY;
    }
    
    status = i2c_do_write(obj, (address | 0x01), 1);
    if (status != 0x40) {
        i2c_stop(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    
    // Read in all except last byte
    for (count = 0; count < (length - 1); count++) {
        int value = i2c_do_read(obj, 0);
        status = i2c_status(obj);
        if (status != 0x50) {
            i2c_stop(obj);
            return count;
        }
        data[count] = (char) value;
    }
    
    // read in last byte
    int value = i2c_do_read(obj, 1);
    status = i2c_status(obj);
    if (status != 0x58) {
        i2c_stop(obj);
        return length - 1;
    }
    
    data[count] = (char) value;
    
    // If not repeated start, send stop.
    if (stop) {
        i2c_stop(obj);
    }*/
    
    //return din;
	return 0;
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop) {
/*    int i, status;
    
    status = i2c_start(obj);
    
    if ((status != 0x10) && (status != 0x08)) {
        i2c_stop(obj);
        return I2C_ERROR_BUS_BUSY;
    }
    
    status = i2c_do_write(obj, (address & 0xFE), 1);
    if (status != 0x18) {
        i2c_stop(obj);
        return I2C_ERROR_NO_SLAVE;
    }
    
    for (i=0; i<length; i++) {
        status = i2c_do_write(obj, data[i], 0);
        if(status != 0x28) {
            i2c_stop(obj);
            return i;
        }
    }
    
    // clearing the serial interrupt here might cause an unintended rewrite of the last byte
    // see also issue report https://mbed.org/users/mbed_official/code/mbed/issues/1
    // i2c_clear_SI(obj);
    
    // If not repeated start, send stop.
    if (stop) {
        i2c_stop(obj);
    }
    
    return length;*/
		return 0;
}

void i2c_reset(i2c_t *obj) {
    i2c_stop(obj);
}

int i2c_byte_read(i2c_t *obj, int last) {
    return 0;
}

int i2c_byte_write(i2c_t *obj, int data) {
       
    return 0;
}

void i2c_slave_mode(i2c_t *obj, int enable_slave) {
/*    if (enable_slave != 0) {
        i2c_conclr(obj, 1, 1, 1, 0);
        i2c_conset(obj, 0, 0, 0, 1);
    } else {
        i2c_conclr(obj, 1, 1, 1, 1);
    }*/
}

int i2c_slave_receive(i2c_t *obj) {
/*    int status;
    int retval;
    
    status = i2c_status(obj);
    switch(status) {
        case 0x60: retval = 3; break;
        case 0x70: retval = 2; break;
        case 0xA8: retval = 1; break;
        default  : retval = 0; break;
    }
    
    return(retval);*/
	return 0;
}

int i2c_slave_read(i2c_t *obj, char *data, int length) {
/*    int count = 0;
    int status;
    
    do {
        i2c_clear_SI(obj);
        i2c_wait_SI(obj);
        status = i2c_status(obj);
        if((status == 0x80) || (status == 0x90)) {
            data[count] = I2C_DAT(obj) & 0xFF;
        }
        count++;
    } while (((status == 0x80) || (status == 0x90) ||
            (status == 0x060) || (status == 0x70)) && (count < length));
    
    if(status != 0xA0) {
        i2c_stop(obj);
    }
    
    i2c_clear_SI(obj);
    
    return count;*/
	return 0;
}

int i2c_slave_write(i2c_t *obj, const char *data, int length) {
/*    int count = 0;
    int status;
    
    if(length <= 0) {
        return(0);
    }
    
    do {
        status = i2c_do_write(obj, data[count], 0);
        count++;
    } while ((count < length) && (status == 0xB8));
    
    if ((status != 0xC0) && (status != 0xC8)) {
        i2c_stop(obj);
    }
    
    i2c_clear_SI(obj);
    
    return(count);*/
	return 0;
}

void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask) {
/*    uint32_t addr;
    
    if ((idx >= 0) && (idx <= 3)) {
        addr = ((uint32_t)obj->i2c) + I2C_addr_offset[0][idx];
        *((uint32_t *) addr) = address & 0xFF;
        addr = ((uint32_t)obj->i2c) + I2C_addr_offset[1][idx];
        *((uint32_t *) addr) = mask & 0xFE;
    }*/
}




