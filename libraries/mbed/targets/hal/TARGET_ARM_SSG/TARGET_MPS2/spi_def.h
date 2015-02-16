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
 * File:     apspi.h
 * Release:  Version 2.0
 * ----------------------------------------------------------------
 *  
 *            SSP interface Support
 *            =====================
 */

#define SSPCS_BASE          (0x4002804C)  // SSP chip select register
#define SSP_BASE            (0x40020000)  // SSP Prime Cell

#define SSPCR0              ((volatile unsigned int *)(SSP_BASE + 0x00))
#define SSPCR1              ((volatile unsigned int *)(SSP_BASE + 0x04))
#define SSPDR               ((volatile unsigned int *)(SSP_BASE + 0x08))
#define SSPSR               ((volatile unsigned int *)(SSP_BASE + 0x0C))
#define SSPCPSR             ((volatile unsigned int *)(SSP_BASE + 0x10))
#define SSPIMSC             ((volatile unsigned int *)(SSP_BASE + 0x14))
#define SSPRIS              ((volatile unsigned int *)(SSP_BASE + 0x18))
#define SSPMIS              ((volatile unsigned int *)(SSP_BASE + 0x1C))
#define SSPICR              ((volatile unsigned int *)(SSP_BASE + 0x20))
#define SSPDMACR            ((volatile unsigned int *)(SSP_BASE + 0x24))
#define SSPCS               ((volatile unsigned int *)(SSPCS_BASE))

// SSPCR0 Control register 0
#define SSPCR0_SCR_DFLT     0x0300      // Serial Clock Rate (divide), default set at 3
#define SSPCR0_SPH          0x0080      // SSPCLKOUT phase
#define SSPCR0_SPO          0x0040      // SSPCLKOUT polarity
#define SSPCR0_FRF_MOT      0x0000      // Frame format, Motorola
#define SSPCR0_DSS_8        0x0007      // Data packet size, 8bits
#define SSPCR0_DSS_16       0x000F      // Data packet size, 16bits

// SSPCR1 Control register 1
#define SSPCR1_SOD          0x0008      // Slave Output mode Disable
#define SSPCR1_MS           0x0004      // Master or Slave mode
#define SSPCR1_SSE          0x0002      // Serial port enable
#define SSPCR1_LBM          0x0001      // Loop Back Mode

// SSPSR Status register
#define SSPSR_BSY           0x0010      // Busy
#define SSPSR_RFF           0x0008      // Receive  FIFO full
#define SSPSR_RNE           0x0004      // Receive  FIFO not empty
#define SSPSR_TNF           0x0002      // Transmit FIFO not full
#define SSPSR_TFE           0x0001      // Transmit FIFO empty

// SSPCPSR Clock prescale register
#define SSPCPSR_DFLT        0x0008      // Clock prescale (use with SCR), default set at 8

// SSPIMSC Interrupt mask set and clear register
#define SSPIMSC_TXIM        0x0008      // Transmit FIFO not Masked
#define SSPIMSC_RXIM        0x0004      // Receive  FIFO not Masked
#define SSPIMSC_RTIM        0x0002      // Receive timeout not Masked
#define SSPIMSC_RORIM       0x0001      // Receive overrun not Masked

// SSPRIS Raw interrupt status register
#define SSPRIS_TXRIS        0x0008      // Raw Transmit interrupt flag
#define SSPRIS_RXRIS        0x0004      // Raw Receive  interrupt flag
#define SSPRIS_RTRIS        0x0002      // Raw Timemout interrupt flag
#define SSPRIS_RORRIS       0x0001      // Raw Overrun  interrupt flag

// SSPMIS Masked interrupt status register
#define SSPMIS_TXMIS        0x0008      // Masked Transmit interrupt flag
#define SSPMIS_RXMIS        0x0004      // Masked Receive  interrupt flag
#define SSPMIS_RTMIS        0x0002      // Masked Timemout interrupt flag
#define SSPMIS_RORMIS       0x0001      // Masked Overrun  interrupt flag

// SSPICR Interrupt clear register
#define SSPICR_RTIC         0x0002      // Clears Timeout interrupt flag
#define SSPICR_RORIC        0x0001      // Clears Overrun interrupt flag

// SSPDMACR DMA control register
#define SSPDMACR_TXDMAE     0x0002      // Enable Transmit FIFO DMA
#define SSPDMACR_RXDMAE     0x0001      // Enable Receive  FIFO DMA

// SPICS register (0=Chip Select low)
#define SSPCS_nCS1          0x0002      // nCS1  (SPI_nSS)

// SPI defaults
#define SSPMAXTIME          1000        // Maximum time to wait for SSP (10*10uS)

// EEPROM instruction set
#define EEWRSR              0x0001      // Write status
#define EEWRITE             0x0002      // Write data
#define EEREAD              0x0003      // Read data
#define EEWDI               0x0004      // Write disable
#define EEWREN              0x0006      // Write enable
#define EERDSR              0x0005      // Read status

// EEPROM status register flags
#define EERDSR_WIP          0x0001      // Write in process
#define EERDSR_WEL          0x0002      // Write enable latch
#define EERDSR_BP0          0x0004      // Block protect 0
#define EERDSR_BP1          0x0008      // Block protect 1
#define EERDSR_WPEN         0x0080      // Write protect enable

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
