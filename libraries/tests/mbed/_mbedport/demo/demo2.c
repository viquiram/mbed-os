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
 * File:     demo2.c
 * Release:  Version 1.0
 * ----------------------------------------------------------------
 */

/*
 * Code implementation file for the Audio DEMO2.
 */

#include <stdio.h>
#include <stdlib.h>

#include "AAIC_I2S_MPS2.h"
#include "AAIC_I2C_MPS2.h"
#include "TSC_I2C_MPS2.h"
#include "GLCD_SPI_MPS2.h"

#include "demo2.h"
#include "icons.h"
#include "../main/common.h"
#include "../aptsc/aptsc.h"

#define AACI_TIMEOUT       1000                                    // Timeout for reading FIFOs (10mS)

/* The AACI has I2C and I2S interfaces from the FPGA
 * The IC2 interface is a simple GPIO interface and the AAIC_I2C_
 * software functions generate the correct I2C protocol.
 * The I2S is a simple FIFO buffer in the FPGA with a FIFO full
 * flag to indicate the FIFO status, the FIFO is shifted out
 * serially to the CODEC.
 */
static unsigned char apAACI_INIT(void)
{
    I2S_config config;

    // See power-up sequence (see DS680F2 page 37)

    // set resets
    i2s_codec_set_reset(MPS2_AAIC_I2S);
    i2s_fifo_set_reset(MPS2_AAIC_I2S);

    // Configure AACI I2S Interface
    config.tx_enable = 1;
    config.tx_int_enable = 1;
    config.tx_waterlevel = 2;
    config.rx_enable = 1;
    config.rx_int_enable = 1;
    config.rx_waterlevel = 2;
    
    i2s_config(MPS2_AAIC_I2S, &config);

    apSleep(10);
    
    // Release AACI nRESET
    i2s_codec_clear_reset(MPS2_AAIC_I2S);
    apSleep(100);

    /* AACI clocks MCLK = 12.288MHz, SCLK = 3.072MHz, LRCLK = 48KHz
     * LRCLK divide ratio [9:0], 3.072MHz (SCLK) / 48KHz / 2 (L+R) = 32
     */
    i2s_speed_config(MPS2_AAIC_I2S, 32);

    // Initialise the AACI I2C interface (see DS680F2 page 38)
    AAIC_I2C_write(0x00, 0x99, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x3E, 0xBA, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x47, 0x80, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x32, 0xBB, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x32, 0x3B, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x00, 0x00, AAIC_I2C_ADDR);
    apSleep(100);

    // Enable MCLK and set frequency (LRCK=48KHz, MCLK=12.288MHz, /256)
    AAIC_I2C_write(0x05, 0xA0, AAIC_I2C_ADDR);

    // Set PDN to 0
    AAIC_I2C_write(AAIC_I2C_CRPWRC1, 0x00, AAIC_I2C_ADDR);

    // Power control 1 Codec powered up
    AAIC_I2C_write(0x02, 0x00, AAIC_I2C_ADDR);
    // Power control 2 MIC powered up
    AAIC_I2C_write(0x03, 0x00, AAIC_I2C_ADDR);
    // Power control 3 Headphone channel always on, Speaker channel always on
    AAIC_I2C_write(0x04, 0xAA, AAIC_I2C_ADDR);

    // Input select AIN1A and AIN1B
    AAIC_I2C_write(0x08, 0x00, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x09, 0x00, AAIC_I2C_ADDR);

    // Master volume
    AAIC_I2C_write(0x20, 0xFF, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x21, 0xFF, AAIC_I2C_ADDR);

    // Audio setup complete
    apSleep(10);

    // Release I2S FIFO reset
    i2s_fifo_clear_reset(MPS2_AAIC_I2S);

    // Read the I2C chip ID and revision
    return AAIC_I2C_read(AAIC_I2C_CRID, AAIC_I2C_ADDR);
}


// Demo 2 Screen initialisation
void apDEMO2_init(void)
{
    unsigned int loop;
    
	// Set background to black
    GLCD_Clear (Black);

	// Draw EXIT button image
    GLCD_BitmapSize (268, 319, 185, 239, (unsigned short *)exitData);

    // Draw the slider horizontal
    GLCD_BitmapSize (60, 235, 178, 205, (unsigned short *)slideData);

    // Draw the slider vertical
    GLCD_BitmapSize (5, 32, 10, 186, (unsigned short *)slidevData);

	// Draw the wav window
	for (loop = 50; loop < 250; loop++)
	{
        GLCD_PutPixelColor(loop,  49, White);
        GLCD_PutPixelColor(loop, 150, White);
	}
	for (loop = 50; loop < 150; loop++)
	{
        GLCD_PutPixelColor(50, loop, White);
        GLCD_PutPixelColor(250, loop, White);
	}
}

// Demo 2 test
void apDEMO2_run(void)
{
	unsigned int loop, done, uwav, wavh, wavv, uvol, uspk, spkmd, timer, hop;
	unsigned int hpos, vpos, pend, penx, peny, nx, ny, col, x;
	int          y;
    unsigned char din;

	// AACI CODEC init
	din = apAACI_INIT();

    // Read and check the I2C chip ID and revision
    din = AAIC_I2C_read(AAIC_I2C_CRID, AAIC_I2C_ADDR);
    if ((din & 0xF8) != 0xE0)
    {
        apDebug("ERROR: AACI ID:0x%02X\n", din);
    }
    else
    {
        apDebug("AACI ID:0x%02X\n", din);
    }

    // Initialise screen for demo
    apDEMO2_init();
    
	// Set default sliders
	done  = FALSE;
	uwav  = TRUE;
	uvol  = TRUE;
	uspk  = TRUE;
	spkmd = TRUE;
	pend  = TRUE;
	wavv  = 64;
	wavh  = 64;
	hpos  = 64;
	vpos  = 64;
	penx  = 64 + 86;
    
	apTSC_setslide(hpos + 86, 180, 0x07E0, hpos + 86);
	apTSC_setslidev(7, vpos + 25, 0x07E0, vpos + 25);

	do
	{
		// Output sinewave until pen is released
		if (!pend && spkmd)
		{
			// Wav hop size
			hop = (42 + hpos) >> 4;

			// Clear TSC Interrupt
            NVIC_ClearPendingIRQ(TSC_IRQn);
			//*NVIC_CLRPEND0 |= (1 << NVIC_TSC);
			do
			{
				// Wait for TX FIFO not to be full then write left and right channels
				timer = AACI_TIMEOUT;
				while ((MPS2_AAIC_I2S->STATUS & I2S_STATUS_TXFull_Msk) && timer)
					timer--;

				// Left then right audio out
				MPS2_AAIC_I2S->TXBUF = sinewave[loop];

				// Next sine value
				loop = loop + hop;
				if (loop > 359)
					loop = loop - 359;

			} while (!NVIC_GetPendingIRQ(TSC_IRQn));
		}

		// Check TSC interrupt
		if (NVIC_GetPendingIRQ(TSC_IRQn))
		{
            // Clear interrupt
			NVIC_ClearPendingIRQ(TSC_IRQn);
            
			TSC_I2C_write(0x0B, 0x01, TSC_I2C_ADDR);
		}

		// Read touchscreen
		pend = apTSC_READXY(&penx, &peny);

		// Done
		if (pend && (penx > 238) && (peny > 200))
		    done = TRUE;

		// Select Speaker or Mic mode
		if (pend && (penx > 275) && (penx < 320) && (peny > 40) && (peny < 140))
		{
			if(peny < 90)
				spkmd = TRUE;
			else
				spkmd = FALSE;
			uspk = TRUE;
		}

		// Draw slider horizontal (spos = 1 to 127)
		if (pend && (penx > 76) && (penx < 224) && (peny > 153) && (peny < 230))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			apTSC_setslide(hpos + 86, 180, 0x07E0, nx);
			hpos = nx - 86;
			uwav = TRUE;
		}

		// Draw slider vertical (spos = 1 to 127)
		if (pend && (penx > 0) && (penx < 37) && (peny > 0) && (peny < 196))
		{
			ny = (peny < 150) ? peny : 150;
			ny = (ny > 24) ? ny : 24;
			apTSC_setslidev(7, vpos + 25, 0x07E0, ny + 2);
			vpos = ny - 23;
			uvol = TRUE;
			uwav = TRUE;
		}

	    // Update volume
		if (uvol)
		{
			loop = 0xFF - (vpos >> 1);
			AAIC_I2C_write(0x20, loop, AAIC_I2C_ADDR);
			AAIC_I2C_write(0x21, loop, AAIC_I2C_ADDR);
		}

	    // Draw wave
		if (uwav)
		{
			// Clear the old wav
			for (x = 51; x < 249; x++)
			{
				// Calculate X
				hop = (x * (wavh + 64)) / 32;
				while (hop >= 360)
					hop = hop - 360;

				// Calculate Y
				y = (sinewave[hop] / 0xFFFF) ^ 0xFFFF8000;
				y = y * (128 - wavv) * 200;

				// Draw wav
				GLCD_PutPixelColor(x,  100 + (y >> 24), Black);
			}

			// New wav h/v value
			wavh = hpos;
			wavv = vpos;

			// Draw the new wav
			for (x = 51; x < 249; x++)
			{
				// Calculate X
				hop = (x * (wavh + 64)) / 32;
				while (hop >= 360)
					hop = hop - 360;

				// Calculate Y
				y = (sinewave[hop] / 0xFFFF) ^ 0xFFFF8000;
				y = y * (128 - wavv) * 200;

				// Draw wav
				GLCD_PutPixelColor(x,  100 + (y >> 24), White);
			}

			if (spkmd)
			{
				// Draw the grids
				for (loop = 51; loop < 249; loop = loop + 5)
					GLCD_PutPixelColor(loop,  100, LightGrey);
				for (loop = 51; loop < 149; loop = loop + 5)
					GLCD_PutPixelColor(150, loop, LightGrey);
			}

			uwav = FALSE;
		}

	    // Draw speaker and mic
		if (uspk)
		{
			// Draw the speaker
			GLCD_SetWindowSize (280, 315, 50, 84);
			GLCD_Start();
			for(loop = 0; loop < (35*35); loop++)
			{
				col = (spkrData[loop] & 0xFFFF);
				if ((col == 0xC618) && spkmd)
					col = 0x07E0;
				GLCD_Write(col);
			}
			GLCD_End();

			// Draw the speaker off
			GLCD_SetWindowSize (280, 315, 100, 134);
			GLCD_Start();
			for(loop = 0; loop < (35*35); loop++)
			{
				col = (spkroffData[loop] & 0xFFFF);
				if ((col == 0xC618) && !spkmd)
					col = 0xF800;
				if ((col == 0x8410) && !spkmd)
					col = 0x07E0;
				GLCD_Write(col);
			}
			GLCD_End();

			uspk = FALSE;
		}

	} while (!done);

	// Change EXIT image
	GLCD_SetWindowSize (268, 319, 185, 239);
	GLCD_Start();
	for(loop = 0; loop < (52*54); loop++)
	{
		col = (exitData[loop] & 0xFFFF);
		if(col == 0xFFFF)
    		GLCD_Write(0x07E0);
		else
    		GLCD_Write(col);
	}
    GLCD_End();

	apSleep(1000);
}
