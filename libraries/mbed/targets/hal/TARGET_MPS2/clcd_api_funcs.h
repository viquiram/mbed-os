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
#ifndef MBED_CLCD_API_H
#define MBED_CLCD_API_H

#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 *
 *            Color LCD Support
 *            =================
 */

// Color LCD Controller Internal Register addresses
#define LSSPCS_BASE          (0x4002804C)  // LSSP chip select register
#define LSSP_BASE            (0x40021000)  // LSSP Prime Cell

#define LSSPCR0              ((volatile unsigned int *)(LSSP_BASE + 0x00))
#define LSSPCR1              ((volatile unsigned int *)(LSSP_BASE + 0x04))
#define LSSPDR               ((volatile unsigned int *)(LSSP_BASE + 0x08))
#define LSSPSR               ((volatile unsigned int *)(LSSP_BASE + 0x0C))
#define LSSPCPSR             ((volatile unsigned int *)(LSSP_BASE + 0x10))
#define LSSPIMSC             ((volatile unsigned int *)(LSSP_BASE + 0x14))
#define LSSPRIS              ((volatile unsigned int *)(LSSP_BASE + 0x18))
#define LSSPMIS              ((volatile unsigned int *)(LSSP_BASE + 0x1C))
#define LSSPICR              ((volatile unsigned int *)(LSSP_BASE + 0x20))
#define LSSPDMACR            ((volatile unsigned int *)(LSSP_BASE + 0x24))
#define LSSPCS               ((volatile unsigned int *)(LSSPCS_BASE))

// LSSPCR0 Control register 0
#define LSSPCR0_SCR_DFLT    0x0100      // Serial Clock Rate (divide), CLK/(CPSR*(1+SCR))
#define LSSPCR0_SPH         0x0080      // LSSPCLKOUT phase
#define LSSPCR0_SPO         0x0040      // LSSPCLKOUT polarity
#define LSSPCR0_FRF_MOT     0x0000      // Frame format, Motorola
#define LSSPCR0_DSS_8       0x0007      // Data packet size, 8bits
#define LSSPCR0_DSS_16      0x000F      // Data packet size, 16bits

// LSSPCR1 Control register 1
#define LSSPCR1_SOD         0x0008      // Slave Output mode Disable
#define LSSPCR1_MS          0x0004      // Master or Slave mode
#define LSSPCR1_SSE         0x0002      // Serial port enable
#define LSSPCR1_LBM         0x0001      // Loop Back Mode

// LSSPSR Status register
#define LSSPSR_BSY          0x0010      // Busy
#define LSSPSR_RFF          0x0008      // Receive  FIFO full
#define LSSPSR_RNE          0x0004      // Receive  FIFO not empty
#define LSSPSR_TNF          0x0002      // Transmit FIFO not full
#define LSSPSR_TFE          0x0001      // Transmit FIFO empty

// LSSPCPSR Clock prescale register
#define LSSPCPSR_DFLT       0x0002      // Clock prescale (use with SCR)

// SPICS register
#define LSSPCS_nCS0         0x0001      // nCS0      (CLCD_CS)
#define LSSPCS_nCS2         0x0004      // nCS2      (CLCD_T_CS)
#define LCD_RESET           0x0008      // RESET     (CLCD_RESET)
#define LCD_RS              0x0010      // RS        (CLCD_RS)
#define LCD_RD              0x0020      // RD        (CLCD_RD)
#define LCD_BL              0x0040      // Backlight (CLCD_BL_CTRL)

// SPI defaults
#define LSSPMAXTIME         10000       // Maximum time to wait for LSSP (10*10uS)
#define LSPI_START          (0x70)      // Start byte for SPI transfer
#define LSPI_RD             (0x01)      // WR bit 1 within start
#define LSPI_WR             (0x00)      // WR bit 0 within start
#define LSPI_DATA           (0x02)      // RS bit 1 within start byte
#define LSPI_INDEX          (0x00)      // RS bit 0 within start byte

// GLCD RGB color definitions
//#define Black               0x0000      //   0,   0,   0
//#define Navy                0x000F      //   0,   0, 128
//#define DarkGreen           0x03E0      //   0, 128,   0
//#define DarkCyan            0x03EF      //   0, 128, 128
//#define Maroon              0x7800      // 128,   0,   0
//#define Purple              0x780F      // 128,   0, 128
//#define Olive               0x7BE0      // 128, 128,   0
//#define LightGrey           0xC618      // 192, 192, 192
//#define DarkGrey            0x7BEF      // 128, 128, 128
//#define Blue                0x001F      //   0,   0, 255
//#define Green               0x07E0      //   0, 255,   0
//#define Cyan                0x07FF      //   0, 255, 255
//#define Red                 0xF800      // 255,   0,   0
//#define Magenta             0xF81F      // 255,   0, 255
//#define Yellow              0xFFE0      // 255, 255, 0
//#define White               0xFFFF      // 255, 255, 255

// Screen size
#define LCD_WIDTH           320         // Screen Width (in pixels)
#define LCD_HEIGHT          240         // Screen Height (in pixels)

typedef struct clcd_s clcd_t;

// Initialise LCD display
void apCLCD_init(clcd_t *obj);

// Fast write of 2 bytes over the serial communication
void clcd_fw (clcd_t *obj, unsigned int byte);

// Write a command the LCD controller
void clcd_wr_cmd (clcd_t *obj, unsigned char cmd);

// Write a value to the to LCD register
void clcd_wr_reg (clcd_t *obj, unsigned char reg, unsigned short val);

// Start of data block transfer
void clcd_start (clcd_t *obj);

// End of data block transfer
void clcd_end (clcd_t *obj);

// Draw a pixel
void clcd_putpixel (clcd_t *obj, unsigned int x, unsigned int y, unsigned int col);

// Fill screen
void clcd_fillscreen (clcd_t *obj, unsigned int col);

#ifdef __cplusplus
}
#endif

#endif
