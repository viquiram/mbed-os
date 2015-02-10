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
#include <math.h>

#include "spi_def.h"
#include "cmsis.h"
#include "pinmap.h"
#include "error.h"
#include "wait_api.h"
#include "clcd_api_funcs.h"

/*
 * Code implementation file for the Color LCD interface.
 */

// Fast write of 2 bytes over the serial communication
void clcd_fw (clcd_t *obj, unsigned int byte)
{
    int waittimer;

    // Wait for TX
    waittimer = LSSPMAXTIME;
    while ((obj->clcd->SR & LSSPSR_BSY) && waittimer)
        waittimer--;

    // Send MSB
    obj->clcd->DR = byte >> 8;

    // Wait for TX
    waittimer = LSSPMAXTIME;
    while ((obj->clcd->SR & LSSPSR_BSY) && waittimer)
        waittimer--;

    // Send LSB
    obj->clcd->DR = byte & 0xFF;
}

// Transfer 1 byte over the serial communication
unsigned char clcd_rw (clcd_t *obj, unsigned char byte)
{
    int waittimer;

    // Send data
    obj->clcd->DR = byte & 0xFF;

    // Wait for TX
    waittimer = LSSPMAXTIME;
    while ((obj->clcd->SR & LSSPSR_BSY) && waittimer)
        waittimer--;

    // Return data
    return (obj->clcd->DR & 0xFF);
}

// Write a command the LCD controller
void clcd_wr_cmd (clcd_t *obj, unsigned char cmd)
{
	// nCS low
    MPS2_FPGAIO->MISC &= ~LSSPCS_nCS0;
    // Write : RS = 0, RW = 0
    clcd_rw(obj, LSPI_START | LSPI_WR | LSPI_INDEX);
    clcd_rw(obj, 0);
    // WR command
    clcd_rw(obj, cmd);
    // nCS high
    MPS2_FPGAIO->MISC |= LSSPCS_nCS0;
}

// Write data to the LCD controller
void clcd_wr_dat (clcd_t *obj, unsigned short dat)
{
	// nCS low
    MPS2_FPGAIO->MISC &= ~LSSPCS_nCS0;
    // Write : RS = 1, RW = 0
    clcd_rw(obj, LSPI_START | LSPI_WR | LSPI_DATA);
    // Write D8..D15
    clcd_rw(obj, (dat >> 8));
    // Write D0..D7
    clcd_rw(obj, (dat & 0xFF));
    // nCS high
    MPS2_FPGAIO->MISC |= LSSPCS_nCS0;
}

// Write a value to the to LCD register
void clcd_wr_reg (clcd_t *obj, unsigned char reg, unsigned short val)
{
    clcd_wr_cmd(obj,reg);
    clcd_wr_dat(obj,val);
}

// Start of data block transfer
void clcd_start (clcd_t *obj)
{
	// SRAM write
	clcd_wr_cmd(obj,0x22);
	// nCS low
	wait_us(1);
    MPS2_FPGAIO->MISC &= ~LSSPCS_nCS0;
	// Start data transfer: Write, RS = 1, RW = 0
    clcd_rw(obj, LSPI_START | LSPI_WR | LSPI_DATA);
}

// End of data block transfer
void clcd_end (clcd_t *obj)
{
    int waittimer;

    // Wait for last TX
    waittimer = LSSPMAXTIME;
    while ((obj->clcd->SR & LSSPSR_BSY) && waittimer)
        waittimer--;

	// nCS high
    MPS2_FPGAIO->MISC |= LSSPCS_nCS0;
    wait_us(1);
}

// Draw a pixel
void clcd_PutPixel (clcd_t *obj, unsigned int x, unsigned int y, unsigned int col)
{
    clcd_wr_reg(obj,0x02, x >>    8);            // Column address start MSB
	clcd_wr_reg(obj,0x03, x &  0xFF);            // Column address start LSB
	clcd_wr_reg(obj,0x04, x >>    8);            // Column address end MSB
	clcd_wr_reg(obj,0x05, x &  0xFF);            // Column address end LSB

	clcd_wr_reg(obj,0x06, y >>    8);            // Row address start MSB
	clcd_wr_reg(obj,0x07, y &  0xFF);            // Row address start LSB
	clcd_wr_reg(obj,0x08, y >>    8);            // Row address end MSB
	clcd_wr_reg(obj,0x09, y &  0xFF);            // Row address end LSB

	// SRAM write
	clcd_wr_cmd(obj,0x22);
	// Set the pixel colour
	clcd_wr_dat(obj,col);
}

// Fill screen
void clcd_fillscreen (clcd_t *obj, unsigned int col)
{
	unsigned int loop;

    // --- Set draw window region ---
    // Column address start
    clcd_wr_reg(obj,0x02, 0x00);
    clcd_wr_reg(obj,0x03, 0x00);
    // Column address end
    clcd_wr_reg(obj,0x04, (LCD_WIDTH - 1) >> 8);
    clcd_wr_reg(obj,0x05, (LCD_WIDTH - 1) & 0xFF);
    // Row address start
    clcd_wr_reg(obj,0x06, 0x00);
    clcd_wr_reg(obj,0x07, 0x00);
    // Row address end
    clcd_wr_reg(obj,0x08, (LCD_HEIGHT - 1) >> 8);
    clcd_wr_reg(obj,0x09, (LCD_HEIGHT - 1) & 0xFF);
	// SRAM write
	clcd_wr_cmd(obj,0x22);

    // Fill display
    clcd_start(obj);
    // Clear screen buffer
    for(loop = 0; loop < (320*240); loop++)
    	clcd_fw(obj, col);
    // end of data
    clcd_end(obj);
}

// Initialise LCD display
void apCLCD_init(clcd_t *obj)
{
    // --- CLCD SSP setup ---

    // Disable serial port operation
    obj->clcd->CR1 = 0;

    // Set serial clock rate, Format, data size
    obj->clcd->CR0 = LSSPCR0_SCR_DFLT | LSSPCR0_FRF_MOT | LSSPCR0_DSS_8;

    // Clock prescale register, set SSP clock to 6MHz (6.6MHz max)
    obj->clcd->CPSR = LSSPCPSR_DFLT;

    // Mask all FIFO/IRQ interrupts apart from the Tx FIFO empty interrupt
    obj->clcd->IMSC = 0x8;

    // Disable FIFO DMA
    obj->clcd->DMACR = 0;

    // Enable serial port operation
    obj->clcd->CR1 = LSSPCR1_SSE;

    // Clear existing IRQ's
    obj->clcd->ICR = 0x3;

    /* CLCD screen setup
     * Default CLCD screen interface state
     */
    MPS2_FPGAIO->MISC |=  (LSSPCS_nCS0 | LSSPCS_nCS2 | LCD_RESET);
	wait_us(100);
    MPS2_FPGAIO->MISC &= ~(LCD_RS | LCD_RD | LCD_BL);
	wait_us(100);

    // Reset CLCD screen
    MPS2_FPGAIO->MISC &= ~LCD_RESET;
	wait_us(10000);
    MPS2_FPGAIO->MISC |=  LCD_RESET;
	wait_us(100000);

    // Turn on backlight
    MPS2_FPGAIO->MISC |= LCD_BL;

    // --- CLCD screen setup ---

    // Driving ability settings
    clcd_wr_reg(obj,0xEA, 0x00);                 // Power control internal used (1)
    clcd_wr_reg(obj,0xEB, 0x20);                 // Power control internal used (2)
    clcd_wr_reg(obj,0xEC, 0x0C);                 // Source control internal used (1)
    clcd_wr_reg(obj,0xED, 0xC7);                 // Source control internal used (2)
    clcd_wr_reg(obj,0xE8, 0x38);                 // Source output period Normal mode
    clcd_wr_reg(obj,0xE9, 0x10);                 // Source output period Idle mode
    clcd_wr_reg(obj,0xF1, 0x01);                 // RGB 18-bit interface ;0x0110
    clcd_wr_reg(obj,0xF2, 0x10);

    // Adjust the Gamma Curve
    clcd_wr_reg(obj,0x40, 0x01);
    clcd_wr_reg(obj,0x41, 0x00);
    clcd_wr_reg(obj,0x42, 0x00);
    clcd_wr_reg(obj,0x43, 0x10);
    clcd_wr_reg(obj,0x44, 0x0E);
    clcd_wr_reg(obj,0x45, 0x24);
    clcd_wr_reg(obj,0x46, 0x04);
    clcd_wr_reg(obj,0x47, 0x50);
    clcd_wr_reg(obj,0x48, 0x02);
    clcd_wr_reg(obj,0x49, 0x13);
    clcd_wr_reg(obj,0x4A, 0x19);
    clcd_wr_reg(obj,0x4B, 0x19);
    clcd_wr_reg(obj,0x4C, 0x16);

    clcd_wr_reg(obj,0x50, 0x1B);
    clcd_wr_reg(obj,0x51, 0x31);
    clcd_wr_reg(obj,0x52, 0x2F);
    clcd_wr_reg(obj,0x53, 0x3F);
    clcd_wr_reg(obj,0x54, 0x3F);
    clcd_wr_reg(obj,0x55, 0x3E);
    clcd_wr_reg(obj,0x56, 0x2F);
    clcd_wr_reg(obj,0x57, 0x7B);
    clcd_wr_reg(obj,0x58, 0x09);
    clcd_wr_reg(obj,0x59, 0x06);
    clcd_wr_reg(obj,0x5A, 0x06);
    clcd_wr_reg(obj,0x5B, 0x0C);
    clcd_wr_reg(obj,0x5C, 0x1D);
    clcd_wr_reg(obj,0x5D, 0xCC);

    // Power voltage setting
    clcd_wr_reg(obj,0x1B, 0x1B);
    clcd_wr_reg(obj,0x1A, 0x01);
    clcd_wr_reg(obj,0x24, 0x2F);
    clcd_wr_reg(obj,0x25, 0x57);
    clcd_wr_reg(obj,0x23, 0x88);

    // Power on setting
    clcd_wr_reg(obj,0x18, 0x36);                 // Internal oscillator frequency adj
    clcd_wr_reg(obj,0x19, 0x01);                 // Enable internal oscillator
    clcd_wr_reg(obj,0x01, 0x00);                 // Normal mode, no scrool
    clcd_wr_reg(obj,0x1F, 0x88);                 // Power control 6 - DDVDH Off
    wait_us(20);
    clcd_wr_reg(obj,0x1F, 0x82);                 // Power control 6 - Step-up: 3 x VCI
    wait_us(5);
    clcd_wr_reg(obj,0x1F, 0x92);                 // Power control 6 - Step-up: On
    wait_us(5);
    clcd_wr_reg(obj,0x1F, 0xD2);                 // Power control 6 - VCOML active
    wait_us(5);

    // Color selection
    clcd_wr_reg(obj,0x17, 0x55);                 // RGB, System interface: 16 Bit/Pixel
    clcd_wr_reg(obj,0x00, 0x00);                 // Scrolling off, no standby

    // Interface config
    clcd_wr_reg(obj,0x2F, 0x11);                 // LCD Drive: 1-line inversion
    clcd_wr_reg(obj,0x31, 0x00);
    clcd_wr_reg(obj,0x32, 0x00);                 // DPL=0, HSPL=0, VSPL=0, EPL=0

    // Display on setting
    clcd_wr_reg(obj,0x28, 0x38);                 // PT(0,0) active, VGL/VGL
    wait_us(20);
    clcd_wr_reg(obj,0x28, 0x3C);                 // Display active, VGL/VGL

    // Display scrolling settings
    clcd_wr_reg(obj,0x0E, 0x00);                 // TFA MSB
    clcd_wr_reg(obj,0x0F, 0x00);                 // TFA LSB
    clcd_wr_reg(obj,0x10, 320 >> 8);             // VSA MSB
    clcd_wr_reg(obj,0x11, 320 &  0xFF);          // VSA LSB
    clcd_wr_reg(obj,0x12, 0x00);                 // BFA MSB
    clcd_wr_reg(obj,0x13, 0x00);                 // BFA LSB

    // Rotate/Landscape screen options
    //clcd_wr_reg(obj,0x16, 0xA8);                 // Landscape, 0
    clcd_wr_reg(obj,0x16, 0x68);               // Landscape,180
    //clcd_wr_reg(obj,0x16, 0x08);               // Portrait, 0
    //clcd_wr_reg(obj,0x16, 0xC8);               // Portrait, 180

    // --- Set draw window region ---
    // Column address start
    clcd_wr_reg(obj,0x02, 0x00);
    clcd_wr_reg(obj,0x03, 0x00);
    // Column address end
    clcd_wr_reg(obj,0x04, (LCD_WIDTH - 1) >> 8);
    clcd_wr_reg(obj,0x05, (LCD_WIDTH - 1) & 0xFF);
    // Row address start
    clcd_wr_reg(obj,0x06, 0x00);
    clcd_wr_reg(obj,0x07, 0x00);
    // Row address end
    clcd_wr_reg(obj,0x08, (LCD_HEIGHT - 1) >> 8);
    clcd_wr_reg(obj,0x09, (LCD_HEIGHT - 1) & 0xFF);
		// SRAM write
		clcd_wr_cmd(obj,0x22);
		
		//clcd_fillscreen(obj, Navy);
    // Set intro screen image box
    clcd_wr_reg(obj,0x02, ( 22 >> 8));//0 >> 8));
    clcd_wr_reg(obj,0x03, ( 22 & 0xFF));//0  & 0xFF));
    clcd_wr_reg(obj,0x04, (297 >> 8));  //319 >> 8));
    clcd_wr_reg(obj,0x05, (297 & 0xFF));//319 & 0xFF));
    clcd_wr_reg(obj,0x06, ( 71 >> 8));  //0 >> 8));
    clcd_wr_reg(obj,0x07, ( 71 & 0xFF));//0 & 0xFF));
    clcd_wr_reg(obj,0x08, (169 >> 8));  //239 >> 8));
    clcd_wr_reg(obj,0x09, (169 & 0xFF));//239 & 0xFF));
		clcd_wr_cmd(obj,0x22);

		clcd_start(obj);
    for(int loop = 0; loop < (275*98); loop++)
    	clcd_fw(obj,introData[loop]);
    clcd_end(obj);

}

/*// CLCD test
apError clcd_TEST(void)
{
    int failtest, loop;

    failtest = FALSE;

    // Some instructions
    printf("The CLCD test will fill the LCD display with\n");
    printf("a test pattern image.\n");
	Wait_For_Enter(FALSE);

    // CLCD screen register setup
    clcd_INIT();

    // Fill screen
    clcd_fillscreen(Black);

    // Fill display with test pattern bitmap (320*240*16bit RGB 5:6:5)
    clcd_start();
    // Clear screen buffer
    for(loop = 0; loop < (320*240); loop++)
    	clcd_fw(obj, flyerData[loop]);
    // End of data
    clcd_end();

    printf("\nDid the test pattern display correctly  (Y/N): ");
    if (!Get_OK())
        failtest = TRUE;

    if (failtest)
        return apERR_CHARLCD_START;
    else
    	return apERR_NONE;
}*/
