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
 * File:     apaaci.c
 * Release:  Version 2.0
 * ----------------------------------------------------------------
 */

/*
 * Code implementation file for the AACI (Advanced Audio CODEC) interface.
 */

#include <stdio.h>
#include <stdlib.h>
#include "wait_api.h"
#include "audio_aaci_api.h"
#include "AAIC_I2S_MPS2.h"
#include "AAIC_I2C_MPS2.h"



// A 48 point, 16 bit sine table
static const int sinewave[] = { 0x80008000, 0x823B823B, 0x84778477, 0x86B286B2, 0x88ED88ED, 0x8B278B27, 0x8D608D60, 0x8F998F99, 0x91D091D0, 0x94059405,
	    0x96399639, 0x986C986C, 0x9A9C9A9C, 0x9CCA9CCA, 0x9EF69EF6, 0xA120A120, 0xA347A347, 0xA56BA56B, 0xA78DA78D, 0xA9ABA9AB,
	    0xABC6ABC6, 0xADDEADDE, 0xAFF2AFF2, 0xB202B202, 0xB40FB40F, 0xB617B617, 0xB81BB81B, 0xBA1BBA1B, 0xBC16BC16, 0xBE0DBE0D,
	    0xBFFFBFFF, 0xC1EBC1EB, 0xC3D3C3D3, 0xC5B5C5B5, 0xC792C792, 0xC969C969, 0xCB3BCB3B, 0xCD07CD07, 0xCECCCECC, 0xD08CD08C,
	    0xD245D245, 0xD3F8D3F8, 0xD5A4D5A4, 0xD74AD74A, 0xD8E9D8E9, 0xDA81DA81, 0xDC12DC12, 0xDD9BDD9B, 0xDF1EDF1E, 0xE099E099,
	    0xE20CE20C, 0xE378E378, 0xE4DCE4DC, 0xE638E638, 0xE78CE78C, 0xE8D8E8D8, 0xEA1CEA1C, 0xEB58EB58, 0xEC8BEC8B, 0xEDB6EDB6,
	    0xEED8EED8, 0xEFF2EFF2, 0xF103F103, 0xF20BF20B, 0xF30AF30A, 0xF400F400, 0xF4EDF4ED, 0xF5D1F5D1, 0xF6ACF6AC, 0xF77EF77E,
	    0xF846F846, 0xF905F905, 0xF9BAF9BA, 0xFA66FA66, 0xFB09FB09, 0xFBA2FBA2, 0xFC31FC31, 0xFCB6FCB6, 0xFD32FD32, 0xFDA4FDA4,
	    0xFE0CFE0C, 0xFE6BFE6B, 0xFEBFFEBF, 0xFF0AFF0A, 0xFF4BFF4B, 0xFF82FF82, 0xFFAFFFAF, 0xFFD2FFD2, 0xFFEAFFEA, 0xFFF9FFF9,
	    0xFFFEFFFE, 0xFFFAFFFA, 0xFFEBFFEB, 0xFFD2FFD2, 0xFFAFFFAF, 0xFF82FF82, 0xFF4BFF4B, 0xFF0AFF0A, 0xFEC0FEC0, 0xFE6BFE6B,
	    0xFE0DFE0D, 0xFDA5FDA5, 0xFD33FD33, 0xFCB7FCB7, 0xFC32FC32, 0xFBA2FBA2, 0xFB0AFB0A, 0xFA67FA67, 0xF9BBF9BB, 0xF906F906,
	    0xF847F847, 0xF77FF77F, 0xF6ADF6AD, 0xF5D2F5D2, 0xF4EEF4EE, 0xF401F401, 0xF30BF30B, 0xF20CF20C, 0xF104F104, 0xEFF3EFF3,
	    0xEEDAEEDA, 0xEDB7EDB7, 0xEC8DEC8D, 0xEB59EB59, 0xEA1EEA1E, 0xE8DAE8DA, 0xE78EE78E, 0xE63AE63A, 0xE4DEE4DE, 0xE37AE37A,
	    0xE20EE20E, 0xE09BE09B, 0xDF20DF20, 0xDD9DDD9D, 0xDC14DC14, 0xDA83DA83, 0xD8EBD8EB, 0xD74CD74C, 0xD5A7D5A7, 0xD3FAD3FA,
	    0xD248D248, 0xD08ED08E, 0xCECFCECF, 0xCD09CD09, 0xCB3DCB3D, 0xC96CC96C, 0xC795C795, 0xC5B8C5B8, 0xC3D5C3D5, 0xC1EEC1EE,
	    0xC001C001, 0xBE0FBE0F, 0xBC19BC19, 0xBA1EBA1E, 0xB81EB81E, 0xB61AB61A, 0xB411B411, 0xB205B205, 0xAFF5AFF5, 0xADE1ADE1,
	    0xABC9ABC9, 0xA9AEA9AE, 0xA790A790, 0xA56EA56E, 0xA34AA34A, 0xA123A123, 0x9EF99EF9, 0x9CCD9CCD, 0x9A9F9A9F, 0x986F986F,
	    0x963C963C, 0x94089408, 0x91D391D3, 0x8F9C8F9C, 0x8D648D64, 0x8B2A8B2A, 0x88F088F0, 0x86B586B5, 0x847A847A, 0x823E823E,
	    0x80038003, 0x7DC77DC7, 0x7B8B7B8B, 0x79507950, 0x77157715, 0x74DB74DB, 0x72A272A2, 0x70697069, 0x6E326E32, 0x6BFD6BFD,
	    0x69C969C9, 0x67966796, 0x65666566, 0x63386338, 0x610C610C, 0x5EE25EE2, 0x5CBB5CBB, 0x5A975A97, 0x58755875, 0x56575657,
	    0x543C543C, 0x52245224, 0x50105010, 0x4E004E00, 0x4BF34BF3, 0x49EB49EB, 0x47E747E7, 0x45E745E7, 0x43EB43EB, 0x41F541F5,
	    0x40034003, 0x3E163E16, 0x3C2F3C2F, 0x3A4C3A4C, 0x386F386F, 0x36983698, 0x34C634C6, 0x32FB32FB, 0x31353135, 0x2F752F75,
	    0x2DBC2DBC, 0x2C092C09, 0x2A5D2A5D, 0x28B728B7, 0x27182718, 0x25802580, 0x23F023F0, 0x22662266, 0x20E320E3, 0x1F681F68,
	    0x1DF51DF5, 0x1C891C89, 0x1B251B25, 0x19C919C9, 0x18751875, 0x17291729, 0x15E515E5, 0x14A914A9, 0x13761376, 0x124B124B,
	    0x11281128, 0x100F100F, 0x0EFE0EFE, 0x0DF60DF6, 0x0CF70CF7, 0x0C000C00, 0x0B130B13, 0x0A2F0A2F, 0x09540954, 0x08820882,
	    0x07BA07BA, 0x06FB06FB, 0x06460646, 0x059A059A, 0x04F704F7, 0x045E045E, 0x03CF03CF, 0x03490349, 0x02CD02CD, 0x025B025B,
	    0x01F301F3, 0x01950195, 0x01400140, 0x00F500F5, 0x00B400B4, 0x007E007E, 0x00510051, 0x002E002E, 0x00150015, 0x00060006,
	    0x00010001, 0x00050005, 0x00140014, 0x002D002D, 0x00500050, 0x007D007D, 0x00B400B4, 0x00F400F4, 0x013F013F, 0x01930193,
	    0x01F101F1, 0x025A025A, 0x02CC02CC, 0x03470347, 0x03CD03CD, 0x045C045C, 0x04F504F5, 0x05970597, 0x06430643, 0x06F806F8,
	    0x07B707B7, 0x087F087F, 0x09510951, 0x0A2B0A2B, 0x0B0F0B0F, 0x0BFC0BFC, 0x0CF30CF3, 0x0DF20DF2, 0x0EFA0EFA, 0x100A100A,
	    0x11241124, 0x12461246, 0x13711371, 0x14A414A4, 0x15E015E0, 0x17231723, 0x186F186F, 0x19C319C3, 0x1B201B20, 0x1C831C83,
	    0x1DEF1DEF, 0x1F621F62, 0x20DD20DD, 0x22602260, 0x23E923E9, 0x257A257A, 0x27122712, 0x28B128B1, 0x2A562A56, 0x2C022C02,
	    0x2DB52DB5, 0x2F6E2F6E, 0x312E312E, 0x32F332F3, 0x34BF34BF, 0x36913691, 0x38683868, 0x3A453A45, 0x3C273C27, 0x3E0E3E0E,
	    0x3FFB3FFB, 0x41ED41ED, 0x43E343E3, 0x45DF45DF, 0x47DE47DE, 0x49E249E2, 0x4BEB4BEB, 0x4DF74DF7, 0x50075007, 0x521C521C,
	    0x54335433, 0x564E564E, 0x586C586C, 0x5A8E5A8E, 0x5CB25CB2, 0x5ED95ED9, 0x61036103, 0x632F632F, 0x655D655D, 0x678D678D,
	    0x69C069C0, 0x6BF46BF4, 0x6E296E29, 0x70607060, 0x72987298, 0x74D274D2, 0x770C770C, 0x79477947, 0x7B827B82, 0x7DBE7DBE };

/* The AACI has I2C and I2S interfaces from the FPGA
 * The IC2 interface is a simple GPIO interface and the AAIC_I2C_
 * software functions generate the correct I2C protocol.
 * The I2S is a simple FIFO buffer in the FPGA with a FIFO full
 * flag to indicate the FIFO status, the FIFO is shifted out
 * serially to the CODEC.
 */
unsigned char audio_aaci_init(aaci_t *obj)
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

    wait_ms(10);
    
    // Release AACI nRESET
    i2s_codec_clear_reset(MPS2_AAIC_I2S);
    wait_ms(100);

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
    wait_ms(100);

    // Enable MCLK and set frequency (LRCK=48KHz, MCLK=12.288MHz, /256)
    AAIC_I2C_write(AAIC_I2C_CLKCRTL, 0xA0, AAIC_I2C_ADDR);

    // Set PDN to 0
    AAIC_I2C_write(AAIC_I2C_CRPWRC1, 0x00, AAIC_I2C_ADDR);

    // Power control 1 Codec powered up
    AAIC_I2C_write(AAIC_I2C_CRPWRC1, 0x00, AAIC_I2C_ADDR);
    // Power control 2 MIC powered up
    AAIC_I2C_write(AAIC_I2C_CRPWRC2, 0x00, AAIC_I2C_ADDR);
    // Power control 3 Headphone channel always on, Speaker channel always on
    AAIC_I2C_write(AAIC_I2C_CRPWRC3, 0xAA, AAIC_I2C_ADDR);

    // Input select AIN1A and AIN1B
    AAIC_I2C_write(AAIC_I2C_INPUTASEL, 0x00, AAIC_I2C_ADDR);
    AAIC_I2C_write(AAIC_I2C_INPUTBSEL, 0x00, AAIC_I2C_ADDR);

    // Master volume
    AAIC_I2C_write(0x20, 0xFF, AAIC_I2C_ADDR);
    AAIC_I2C_write(0x21, 0xFF, AAIC_I2C_ADDR);

    // Audio setup complete
    wait_ms(10);

    // Release I2S FIFO reset
    i2s_fifo_clear_reset(MPS2_AAIC_I2S);

    // Read the I2C chip ID and revision
    return AAIC_I2C_read(AAIC_I2C_CRID, AAIC_I2C_ADDR);
}

void audio_aaci_out_sine(void)
{
	int loop, time,  timer;
	MPS2_AAIC_I2S->TXBUF = 0x00000000;

  // Give the analogue circuit time to settle
  wait_ms(1000);

	// Generate a 500Hz tone and measure the loopback levels
  for (time = 0; time < TONETIME; time++)
  {
		for (loop = 0; loop < 360; loop++)
		{
			// Wait for TX FIFO not to be full then write left and right channels
      timer = AACI_TIMEOUT;
      while (i2s_tx_fifo_full(MPS2_AAIC_I2S) && timer)
				timer--;
      
      // Left and right audio out
      MPS2_AAIC_I2S->TXBUF = sinewave[loop];
    }
	}
}


int audio_aaci_read(void)
{
  int din; 
  din = MPS2_AAIC_I2S->RXBUF;
	return din;
}

void audio_aaci_write(int data)
{
	MPS2_AAIC_I2S->TXBUF = data;
}

void audio_aaci_volume(int volume)
{
	unsigned int loop, vpos;
	vpos = volume;
	loop = 0xFF - (vpos >> 1);
	AAIC_I2C_write(0x20, loop, AAIC_I2C_ADDR);
	AAIC_I2C_write(0x21, loop, AAIC_I2C_ADDR);
}

unsigned char audio_i2c_read (unsigned char reg_addr, unsigned char sadr)
{
	return AAIC_I2C_read(reg_addr,sadr);
}

void audio_i2c_write(unsigned char reg_addr, unsigned char data_byte, unsigned char sadr)
{
	AAIC_I2C_write(reg_addr,data_byte,sadr);		
}

