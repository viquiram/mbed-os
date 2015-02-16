/*
 * Copyright:
 * ----------------------------------------------------------------
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 *   (C) COPYRIGHT 2014 ARM Limited
 *       ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 * ----------------------------------------------------------------
 * File:     fpga.c
 * Release:  Version 1.0
 * ----------------------------------------------------------------
 */

/*
 * Code implementation file for the fpga functions.
 */

#include "SMM_MPS2.h"                   // MPS2 common header

// Function to delay n*ticks (25MHz = 40nS per tick)
// Used for I2C drivers
void i2c_delay(unsigned int tick);

/* Sleep function to delay n*mS
 * Uses FPGA counter.
 */
void Sleepms(unsigned int msec);

/* Sleep function to delay n*uS
 */
void Sleepus(unsigned int usec);
