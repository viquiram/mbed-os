/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low Level Serial Routines
 * Note(s): possible defines select the used communication interface:
 *            __DBG_ITM  - ITM SWO interface
 *                       - COM? (UART0) interface  (default)
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "peripherallink.h"                     // Keil::Board Support:V2M-MPS2:Common
#include "serial.h"                     // Keil::Board Support:V2M-MPS2:Serial

#ifdef __DBG_ITM
volatile int32_t ITM_RxBuffer = ITM_RXBUFFER_EMPTY;  /*  CMSIS Debug Input    */
#endif


/*----------------------------------------------------------------------------
  Initialize UART pins, Baudrate
 *----------------------------------------------------------------------------*/
void Serial_Initialize (void) {
#ifndef __DBG_ITM

  SystemCoreClockUpdate();

  CMSDK_GPIO1->ALTFUNCSET |= ((1ul <<  0) |          /* configure GPIO pin    */
	                            (1ul <<  1) );

  CMSDK_UART0->BAUDDIV = SystemCoreClock / 115200;
  CMSDK_UART0->CTRL    = ((1ul <<  0) |              /* TX enable */
                          (1ul <<  1) );             /* RX enable */

#endif
}


/*----------------------------------------------------------------------------
  Uninitialize UART pins
 *----------------------------------------------------------------------------*/
void Serial_Uninitialize (void) {

}


/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int Serial_PutChar (int c) {

#if   defined __DBG_ITM
    ITM_SendChar(c);
#else
  while ( (CMSDK_UART0->STATE & (1ul << 0))); /* Wait if Transmit Holding register is full */
  CMSDK_UART0->DATA = c;
#endif
  return (c);
}


/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
int Serial_GetChar (void) {

#if   defined __DBG_ITM
  while (ITM_CheckChar() != 1) __NOP();
  return (ITM_ReceiveChar());
#else
  while (!(CMSDK_UART0->STATE & (1ul << 1))); /* Wait if Receive Holding register is empty */
  return (CMSDK_UART0->DATA);
#endif
}
