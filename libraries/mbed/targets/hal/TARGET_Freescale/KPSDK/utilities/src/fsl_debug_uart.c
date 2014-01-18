/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_debug_uart.h"
#include "fsl_uart_hal.h"
#include "fsl_clock_manager.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#if __ICCARM__
#include <yfuns.h>
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Runtime state information for the debug UART interface.*/
typedef struct DebugUartState {
    bool isInited;      /*!< Whether the debug UART has been initialized.*/
    uint32_t instance;  /*!< UART instance to use for debug messages.*/
} debug_uart_state_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Debug UART state information.*/
static debug_uart_state_t s_debugUart = { 0 };

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#pragma weak configure_uart_pin_mux
extern void configure_uart_pin_mux(uint32_t instance);

/*******************************************************************************
 * Code
 ******************************************************************************/

/* See fsl_debug_uart.h for documentation of this function.*/
void debug_uart_init(uint32_t uartInstance, uint32_t baudRate)
{
    /* Save instance number.*/
    s_debugUart.instance = uartInstance;
    
    /* Configure pinmux for the debug uart if the function is present.*/
    if (&configure_uart_pin_mux)
    {
        configure_uart_pin_mux(s_debugUart.instance);
    }
    
    /* Ungate UART clock.*/
    clock_manager_set_gate(kClockModuleUART, s_debugUart.instance, true);

    clock_names_t sourceClock = kBusClock;
    uint32_t freq;
    uint32_t divider = 1;
    
    /* UART0 and 1 on K series use the system clock as their source.*/
    if (uartInstance < 2)
    {
        sourceClock = kSystemClock;
    }

    clock_manager_get_frequency(sourceClock, &freq);

    freq /= divider;

    /* Configure the UART.*/
    uart_config_t config = {
            .uartSourceClockInHz = freq,
            .baudRate = baudRate,
            .parityMode = kUartParityDisabled,
            .stopBitCount = kUartOneStopBit,
            .bitCountPerChar = kUart8BitsPerChar,
            .rxDataInvert = false,
            .txDataInvert = false
        };
    
    uart_hal_init(s_debugUart.instance, &config);
    uart_hal_enable_transmitter(s_debugUart.instance);
    uart_hal_enable_receiver(s_debugUart.instance);
    
    s_debugUart.isInited = true;
}

#if __ICCARM__

#pragma weak __write
size_t __write(int handle, const unsigned char * buffer, size_t size)
{
    size_t nChars = 0;

    if (buffer == 0)
    {
        /* This means that we should flush internal buffers.  Since we*/
        /* don't we just return.  (Remember, "handle" == -1 means that all*/
        /* handles should be flushed.)*/
        return 0;
    }

    /* This function only writes to "standard out" and "standard err",*/
    /* for all other file handles it returns failure.*/
    if ((handle != _LLIO_STDOUT) && (handle != _LLIO_STDERR))
    {
        return _LLIO_ERROR;
    }
    
    /* Do nothing if the debug uart is not initialized.*/
    if (!s_debugUart.isInited)
    {
        return _LLIO_ERROR;
    }

    /* Send data.*/
    while (size--)
    {
        while (!uart_hal_is_transmit_data_register_empty(s_debugUart.instance))
        {
        }
        
        uart_hal_putchar(s_debugUart.instance, *buffer++);
        
        ++nChars;
    }

    return nChars;
}

#pragma weak __read
size_t __read(int handle, unsigned char * buffer, size_t size)
{
    int nChars = 0;

    /* This function only reads from "standard in", for all other file*/
    /* handles it returns failure.*/
    if (handle != _LLIO_STDIN)
    {
        return _LLIO_ERROR;
    }

    /* Do nothing if the debug uart is not initialized.*/
    if (!s_debugUart.isInited)
    {
        return _LLIO_ERROR;
    }

    for (; size > 0; --size)
    {
        if (!uart_hal_is_receive_data_register_full(s_debugUart.instance))
        {
            break;
        }
        
        uart_hal_getchar(s_debugUart.instance, buffer++);

        ++nChars;
    }

    return nChars;
}

#elif (defined(CW))

/*
** ===================================================================
**     Method      :  CsIO1___read_console (component ConsoleIO)
**
**     Description :
**         __read_console
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
int __read_console(__file_handle handle, unsigned char* buffer, size_t * count)
{
    size_t CharCnt = 0x00;

    (void)handle;                        /* Parameter is not used, suppress unused argument warning */

    /* Do nothing if the debug uart is not initialized.*/
    if (!s_debugUart.isInited)
    {
        return _LLIO_ERROR;
    }

    uart_hal_getchar(s_debugUart.instance, buffer);
    CharCnt = 1;                         /* Increase char counter */

    *count = CharCnt;
    return (__no_io_error);
}

/*
** ===================================================================
**     Method      :  CsIO1___write_console (component ConsoleIO)
**
**     Description :
**         __write_console
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
int __write_console(__file_handle handle, unsigned char* buffer, size_t* count)
{
    (void)handle;                        /* Parameter is not used, suppress unused argument warning */

    /* Do nothing if the debug uart is not initialized.*/
    if (!s_debugUart.isInited)
    {
        return _LLIO_ERROR;
    }

    size_t size = *count;
    size_t CharCnt = 0x00;
    while (size--)
    {
        while (!uart_hal_is_transmit_data_register_empty(s_debugUart.instance))
        {
        }
        
        uart_hal_putchar(s_debugUart.instance, *buffer++);
        
        ++CharCnt;
    }


    *count = CharCnt;
    return(__no_io_error);
}

/*
** ===================================================================
**     Method      :  CsIO1___close_console (component ConsoleIO)
**
**     Description :
**         __close_console
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
int __close_console(__file_handle handle)
{
    (void)handle;                        /* Parameter is not used, suppress unused argument warning */
    return(__no_io_error);
}

#elif (defined(__GNUC__))
int _write (int handle, char *buffer, int size)
{
    int nChars = 0;

    if (buffer == 0)
    {
        /* return -1 if error */
        return -1;
    }

    /* This function only writes to "standard out" and "standard err",*/
    /* for all other file handles it returns failure.*/
    if ((handle != 1) && (handle != 2))
    {
        return -1;
    }

    /* Do nothing if the debug uart is not initialized.*/
    if (!s_debugUart.isInited)
    {
        return -1;
    }

    /* Send data.*/
    while (size--)
    {
        while (!uart_hal_is_transmit_data_register_empty(s_debugUart.instance))
        {
        }

        uart_hal_putchar(s_debugUart.instance, *buffer++);

        ++nChars;
    }

    return nChars;
}

int _read(int handle, char *buffer, int size)
{
    int nChars = 0;

    /* This function only reads from "standard in", for all other file*/
    /* handles it returns failure.*/
    if (handle != 0)
    {
        return -1;
    }

    /* Do nothing if the debug uart is not initialized.*/
    if (!s_debugUart.isInited)
    {
        return -1;
    }

    for (; size > 0; --size)
    {
        uart_hal_getchar(s_debugUart.instance, buffer++);

        ++nChars;
    }

    return nChars;
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
