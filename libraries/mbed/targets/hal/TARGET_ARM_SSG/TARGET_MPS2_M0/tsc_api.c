/*
 * Copyright:
 * ----------------------------------------------------------------
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 *   (C) COPYRIGHT 2013 ARM Limited
 *       ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 * ----------------------------------------------------------------
 * File:     apaaci_i2c.c
 * Release:  Version 2.0
 * ----------------------------------------------------------------
 */

/*
 * Code implementation file for the AACI (Advanced Audio CODEC I2C) interface.
 */

#include <stdio.h>
#include <stdlib.h>
#include "wait_api.h"
#include "tsc_api.h"
#include "TSC_I2C_MPS2.h"
//#include "icons.h"

// Screen size
#define LCD_WIDTH           320         // Screen Width (in pixels)
#define LCD_HEIGHT          240         // Screen Height (in pixels)


#define TSC_TSU            15           // Setup delay 600nS min

// Initialise TSC
int tsc_init(tsc_t *obj, PinName sda, PinName scl)
{
	unsigned int din,err; //

	err = FALSE;

	// Clear the TSC I2C interface as there is no RESET control
    TSC_I2C_clear();

    // Read and check the I2C chip ID
    din = TSC_I2C_read(TSC_I2C_CRID, TSC_I2C_ADDR, 2);
    if (din != TSC_I2C_CID)
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


// Write data stream and read one/two bytes
unsigned int TSC_read(unsigned char reg_addr, unsigned char sadr, unsigned char bytes)
{
	unsigned int rxdata;
    rxdata = TSC_I2C_read(reg_addr, sadr, bytes);

    return rxdata;
}
// Write data stream and read one/two bytes
unsigned int TSC_read_x()
{
	unsigned int rxdata;
	unsigned int din,isr/*,pendown*/,loop;
	unsigned int Y, XPXL;
  // Read interrupt status
  isr = TSC_read(0x0B, TSC_I2C_ADDR, 1);

/*  // Clear pen down if state changes
  if (isr & 0x01)
	{
		pendown = FALSE;
	}*/

  // Test for FIFO_TH interrupt
  if (isr & 0x02)
  {
		// Empty the FIFO
		loop = TSC_read(0x4C, TSC_I2C_ADDR, 1);
		while (loop > 1)
		{
			din = TSC_read(0xD7, TSC_I2C_ADDR, 4);
			loop--;
		}
	
		// Clear the interrupt (must be immediately after FIFO empty)
		TSC_write(0x0B, isr, TSC_I2C_ADDR);
	
		// Read coordinates
		din = TSC_read(0xD7, TSC_I2C_ADDR, 4);
		//X = (din >> 20) & 0x00000FFF;
		Y = (din >>  8) & 0x00000FFF;
	
		// Calculate the pixel position (X/Y are swapped)
		XPXL = 320 - ((Y * 10) / (TSC_MAXVAL / 320)) + TSC_XOFF;
		XPXL = (XPXL & 0x80000000) ? 0 : XPXL;
		XPXL = (XPXL >= 320 ) ? 320  - 1 : XPXL;
	}
  else
	{
		// Clear status register
		if (isr)
		{
  	 TSC_write(0x0B, isr, TSC_I2C_ADDR);
		}
	}
  
  
	if(XPXL > 320){ rxdata = 320; }
	else { rxdata = XPXL;}
	
  return rxdata;
}
// Write data stream and read one/two bytes
unsigned int TSC_read_y()
{
	unsigned int rydata;
	unsigned int din, isr,/* pendown,*/ loop;
	unsigned int X, YPXL;
	
  // Read interrupt status
  isr = TSC_read(0x0B, TSC_I2C_ADDR, 1);

/*  // Clear pen down if state changes
  if (isr & 0x01)
	{
		pendown = FALSE;
	}*/

  // Test for FIFO_TH interrupt
  if (isr & 0x02)
	{
		// Empty the FIFO
		loop = TSC_read(0x4C, TSC_I2C_ADDR, 1);
		while (loop > 1)
		{
			din = TSC_read(0xD7, TSC_I2C_ADDR, 4);
			loop--;
		}

		// Clear the interrupt (must be immediately after FIFO empty)
		TSC_write(0x0B, isr, TSC_I2C_ADDR);
	
		// Read coordinates
		din = TSC_read(0xD7, TSC_I2C_ADDR, 4);
		X = (din >> 20) & 0x00000FFF;
		//Y = (din >>  8) & 0x00000FFF;
	
		// Calculate the pixel position (X/Y are swapped)
		YPXL = ((X * 10) / (TSC_MAXVAL / 240)) - TSC_YOFF;
		YPXL = (YPXL & 0x80000000) ? 0 : YPXL;
		YPXL = (YPXL >= 240) ? 240 - 1 : YPXL;
	}
  else
	{
		// Clear status register
		if (isr)
		{
			TSC_write(0x0B, isr, TSC_I2C_ADDR);
		}
	}
	
	if(YPXL > 240){ rydata = 240; }
	else { rydata = YPXL; }

	return rydata;
}

// Write data stream and write one byte
void TSC_write(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr)
{
	TSC_I2C_write(reg_addr,data_byte,sadr);
}

void TSC_clear(void)
{
	TSC_I2C_clear();
}

// TSC read pen down, X,Y
unsigned int TSC_readxy(unsigned int *PENX, unsigned int *PENY)
{
    unsigned int loop, din, isr;
    unsigned int X, Y, XPXL, YPXL, pendown;
    //unsigned int Z;

	pendown  = FALSE;
	*PENX    = 0;
	*PENY    = 0;

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
		X = (din >> 20) & 0x00000FFF;
		Y = (din >>  8) & 0x00000FFF;
		//Z = (din >>  0) & 0x0000000F;

		// printf("X:0x%04X, Y:0x%04X, Z:0x%02X\n", X, Y, Z);

		// Calculate the pixel position (X/Y are swapped)
		XPXL = LCD_WIDTH - ((Y * 10) / (TSC_MAXVAL / LCD_WIDTH)) + TSC_XOFF;
		YPXL = ((X * 10) / (TSC_MAXVAL / LCD_HEIGHT)) - TSC_YOFF;
		XPXL = (XPXL & 0x80000000) ? 0 : XPXL;
		YPXL = (YPXL & 0x80000000) ? 0 : YPXL;
		XPXL = (XPXL >= LCD_WIDTH ) ? LCD_WIDTH  - 1 : XPXL;
		YPXL = (YPXL >= LCD_HEIGHT) ? LCD_HEIGHT - 1 : YPXL;

		// Move to new position if pen lifted
		if (!pendown)
		{
			*PENX = XPXL;
			*PENY = YPXL;
		}
	}

	// Read interrupt status
	din = TSC_I2C_read(0x40, TSC_I2C_ADDR, 1);

	// Read the raw pen down status
	if (din & 0x80)
		pendown = TRUE;

    return pendown;
}

// Move slider horizontal
