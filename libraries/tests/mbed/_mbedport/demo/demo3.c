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
 * File:     demo3.c
 * Release:  Version 1.0
 * ----------------------------------------------------------------
 */

/*
 * Code implementation file for the LED DEMO3.
 */

#include <stdio.h>
#include <stdlib.h>

#include "SMM_MPS2.h"                   // MPS2 common header
#include "GLCD_SPI_MPS2.h"

#include "demo3.h"
#include "icons.h"

#include "../main/common.h"
#include "../aptsc/aptsc.h"

// Demo 3 Screen initialisation
void apDEMO3_init(void)
{
    unsigned int x;
    
	// Set background to black
	GLCD_Clear(Black);

	// Draw EXIT button image
    GLCD_BitmapSize (268, 319, 185, 239, (unsigned short *)exitData);

    // Draw the slider
    GLCD_BitmapSize (60, 235, 105, 132, (unsigned short *)slideData);

	// Set the default LED images
	for (x = 0; x < 8; x++)
	{
        GLCD_BitmapSize (40 + (x * 30), 63 + (x * 30), 30, 77, (unsigned short *)ledoffData);
	}
}

// Demo 3 test
void apDEMO3_run(void)
{
	unsigned int loop, done, uled, usw, ledv, swv;
	unsigned int spos, pend, penx, peny, nx, col, x;

    apDEMO3_init();

// Set default sliders
	done = FALSE;
	uled = TRUE;
	usw  = TRUE;
	ledv = 0;
	swv  = 0;
	spos = 64;
	penx = 64 + 86;
	apTSC_setslide(spos + 86, 107, 0x007E0, penx);
	MPS2_SCC->LEDS = 0x08;

	do
	{
		// Read touchscreen
		pend = apTSC_READXY(&penx, &peny);

		// Done
		if (pend && (penx > 238) && (peny > 200))
		    done = TRUE;

		// Led cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 90) && (peny < 147))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			apTSC_setslide(spos + 86, 107, 0x07E0, nx);
			spos = nx - 86;
			uled = TRUE;
		}

	    // Draw the new leds
		if (uled)
		{
			// Clear the old LED image
            GLCD_BitmapSize (40 + ((7 - ledv) * 30), 63 + ((7 - ledv) * 30), 30, 77, (unsigned short *)ledoffData);

			// Calculate LED position
			if (spos > 112)
				ledv = 0;
			else if (spos > 96)
				ledv = 1;
			else if (spos > 80)
				ledv = 2;
			else if (spos > 64)
				ledv = 3;
			else if (spos > 48)
				ledv = 4;
			else if (spos > 32)
				ledv = 5;
			else if (spos > 16)
				ledv = 6;
			else
				ledv = 7;

			// Set the actual LED
			MPS2_SCC->LEDS = 0x01 << ledv;

			// Set the LED image
            GLCD_BitmapSize (40 + ((7 - ledv) * 30), 63 + ((7 - ledv) * 30), 30, 77, (unsigned short *)ledonData);

			uled = FALSE;
		}

	    // Update switches
		if (usw | (swv != (MPS2_SCC->SWITCHES & 0xFF)))
		{
	        // Read the switch value
	        swv = MPS2_SCC->SWITCHES & 0xFF;

	        // Draw the switch
            GLCD_BitmapSize (60, 235, 146, 235, (unsigned short *)switchData);

	        // Draw the switches
	        for (x = 0; x < 8; x++)
	        {
	        	if (swv & (0x80 >> x))
	        	{
					GLCD_BoxSize(68 + (x * 21), 77 + (x * 21), 189, 195, 0x001F);
	        	}
	        	else
	        	{
					GLCD_BoxSize(68 + (x * 21), 77 + (x * 21), 180, 186, 0xF800);
	        	}
	        }
	    	usw  = FALSE;
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
