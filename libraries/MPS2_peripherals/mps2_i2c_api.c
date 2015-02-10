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
#include "mps2_i2c_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"
#include "TSC_I2C_MPS2.h"
//#include "TSC_I2C_MPS2.c"
#include "AAIC_I2C_MPS2.h"
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


int mps2_i2c_ts_init()
{
		unsigned int din, err;

	err = FALSE;

	// Clear the TSC I2C interface as there is no RESET control
	TSC_I2C_clear();

    // Read and check the I2C chip ID
    din = TSC_I2C_read(TSC_I2C_CRID, TSC_I2C_ADDR, 2);
    if (din != TSC_I2C_CRID)
    {
        //printf("ERROR: TSC ID:0x%04X\n", din);
        err = TRUE;
    }
    else
    {
        //printf("TSC ID:0x%04X\n", din);

		// Initialise the TSC I2C interface
		TSC_I2C_write(0x40, 0x03, TSC_I2C_ADDR);

		// Reset Touch-screen controller
		TSC_I2C_write(0x03, 0x03, TSC_I2C_ADDR);
		wait_us(10000);

		// Enable TSC and ADC
		TSC_I2C_write(0x04, 0x0C, TSC_I2C_ADDR);

		// Enable Touch detect, FIFO
		TSC_I2C_write(0x0A, 0x07, TSC_I2C_ADDR);

		// Set sample time , 12-bit mode
		TSC_I2C_write(0x20, 0x39, TSC_I2C_ADDR);
		wait_us(2000);

		// ADC frequency 3.25 MHz
		TSC_I2C_write(0x21, 0x01, TSC_I2C_ADDR);

		// Set TSC_CFG register
		TSC_I2C_write(0x41, 0xF5, TSC_I2C_ADDR);

		// Threshold for FIFO
		TSC_I2C_write(0x4A, 0x02, TSC_I2C_ADDR);

		// FIFO reset
		TSC_I2C_write(0x4B, 0x01, TSC_I2C_ADDR);
		wait_us(1000);
		TSC_I2C_write(0x4B, 0x00, TSC_I2C_ADDR);

		// Fraction Z
		TSC_I2C_write(0x56, 0x07, TSC_I2C_ADDR);

		// Drive 50 mA typical
		TSC_I2C_write(0x58, 0x01, TSC_I2C_ADDR);

		// Enable TSC
		TSC_I2C_write(0x40, 0x01, TSC_I2C_ADDR);

		// Clear interrupt status
		TSC_I2C_write(0x0B, 0xFF, TSC_I2C_ADDR);

		// Enable global interrupt
		TSC_I2C_write(0x09, 0x01, TSC_I2C_ADDR);

		// Clear status register
		TSC_I2C_write(0x0B, 0xFF, TSC_I2C_ADDR);
    }

    return err;

}
	
void mps2_i2c_aaci_init()
{
/*	unsigned char din;
  din = AAIC_I2C_read(AAIC_I2C_CRID, AAIC_I2C_ADDR);

  // Initialise the AACI I2C interface (see DS680F2 page 38)
  AAIC_I2C_write(0x00, 0x99, AAIC_I2C_ADDR);
  AAIC_I2C_write(0x3E, 0xBA, AAIC_I2C_ADDR);
  AAIC_I2C_write(0x47, 0x80, AAIC_I2C_ADDR);
  AAIC_I2C_write(0x32, 0xBB, AAIC_I2C_ADDR);
  AAIC_I2C_write(0x32, 0x3B, AAIC_I2C_ADDR);
  AAIC_I2C_write(0x00, 0x00, AAIC_I2C_ADDR);
  wait_ms(100);

  // Enable MCLK and set frequency (LRCK=48KHz, MCLK=12.288MHz, /256)
  AAIC_I2C_write(AAIC_I2C_CLKCRTL, 0xA0, AAIC_I2C_ADDR);

  // Power control 1 Codec powered up
  AAIC_I2C_write(AAIC_I2C_CRPWRC1, 0x00, AAIC_I2C_ADDR);
  // Power control 2 MIC powered up
  AAIC_I2C_write(AAIC_I2C_CRPWRC2, 0x00, AAIC_I2C_ADDR);
  // Power control 3 Headphone channel always on, Speaker channel always on
  AAIC_I2C_write(AAIC_I2C_CRPWRC3, 0xAA, AAIC_I2C_ADDR);

  // Input select AIN1A and AIN1B
  AAIC_I2C_write(AAIC_I2C_INPUTASEL, 0x00, AAIC_I2C_ADDR);
  AAIC_I2C_write(AAIC_I2C_INPUTBSEL, 0x00, AAIC_I2C_ADDR);

  // Audio setup complete
  wait_ms(10);*/


}
