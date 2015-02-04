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
 * File:     demo1.c
 * Release:  Version 1.0
 * ----------------------------------------------------------------
 */

/*
 * Code implementation file for the Color DEMO1.
 */

#include <stdio.h>
#include <stdlib.h>

#include "GLCD_SPI_MPS2.h"

#include "demo1.h"
#include "../icons.h"

//#include "../main/common.h"
//#include "../aptsc/aptsc.h"

// Demo 1 Screen initialisation
void apDEMO1_init(CLCD *obj)
{
	// Set background to black
	obj.CLCD_fillscreen(Black);

	// Draw EXIT button image
    GLCD_BitmapSize (268, 319, 185, 239, (unsigned short *)exitData);

    // Draw the three sliders
    GLCD_BitmapSize (60, 235, 110, 137, (unsigned short *)slideData);

    GLCD_BitmapSize (60, 235, 160, 187, (unsigned short *)slideData);

    GLCD_BitmapSize (60, 235, 210, 237, (unsigned short *)slideData);

    // Draw next icon
    GLCD_BitmapSize (275, 310, 119, 156, (unsigned short *)nextData);
}

// Demo 1 test
void apDEMO1_run(void)
{
	unsigned int loop, done, ucol, ncar, carno;
	unsigned int rpos, gpos, bpos, pend, penx, peny, nx, col, ncol;

    // Screen Initialisation
    apDEMO1_init();
    
	// Set default sliders
	done  = FALSE;
	ucol  = TRUE;
	ncar  = FALSE;
	carno = 0;
	rpos  = 64;
	gpos  = 64;
	bpos  = 64;
	penx  = 64 + 86;
	apTSC_setslide(rpos + 86, 112, 0xF800, penx);
	apTSC_setslide(gpos + 86, 162, 0x07E0, penx);
	apTSC_setslide(bpos + 86, 212, 0x001F, penx);

	do
	{
		// Read touchscreen
		pend = apTSC_READXY(&penx, &peny);

		// Done
		if (pend && (penx > 238) && (peny > 200))
		    done = TRUE;

		// Red cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 95) && (peny < 135))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			apTSC_setslide(rpos + 86, 112, 0xF800, nx);
			rpos = nx - 86;
			ucol = TRUE;
		}
		// Green cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 147) && (peny < 185))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			apTSC_setslide(gpos + 86, 162, 0x07E0, nx);
			gpos = nx - 86;
			ucol = TRUE;
		}
		// Blue cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 195) && (peny < 235))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			apTSC_setslide(bpos + 86, 212, 0x001F, nx);
			bpos = nx - 86;
			ucol = TRUE;
		}

		// New car
		if (pend && (penx > 275) && (penx < 310) && (peny > 119) && (peny < 156))
		{
			// Clear the old car
            GLCD_BoxSize(0, 319, 0, 110, 0x0000);

			// Next car
			carno = (carno == 2) ? 0 : carno + 1;
			ncar = TRUE;
		}

	    // Draw the new car colour
		if (ucol || ncar)
		{
			// Calculate car colour
			ncol = ((rpos << 9) & 0xF800) | ((gpos << 4) & 0x07E0) | ((bpos >> 2) & 0x001F);

			// Draw car
			if (carno == 0)
			{
				GLCD_SetWindowSize (28, 271, 5, 86);
				GLCD_Start();
				for(loop = 0; loop < (244*82); loop++)
				{
					col = (car1Data[loop] & 0xFFFF);
					if (col == 0xF800)
						col = ncol;
					GLCD_Write(col);
				}
				GLCD_End();
			}
			else if (carno == 1)
			{
				GLCD_SetWindowSize (36, 263, 0, 100);
				GLCD_Start();
				for(loop = 0; loop < (228*101); loop++)
				{
					col = (car2Data[loop] & 0xFFFF);
					if (col == 0xF800)
						col = ncol;
					GLCD_Write(col);
				}
				GLCD_End();
			}
			else
			{
				GLCD_SetWindowSize (26, 277, 0, 99);
				GLCD_Start();
				for(loop = 0; loop < (251*100); loop++)
				{
					col = (car3Data[loop] & 0xFFFF);
					if (col == 0xF800)
						col = ncol;
					GLCD_Write(col);
				}
				GLCD_End();
			}
			if (ncar)
				apSleep(1000);

			ucol = FALSE;
			ncar = FALSE;
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
