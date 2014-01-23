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
#if !defined(__FSL_UART_IRQ_H__)
#define __FSL_UART_IRQ_H__

//
#include "fsl_uart_features.h"
#include "fsl_uart_driver.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if !defined(UART_IRQ_PRIORITY)
    /*!
     * @brief The priority for UART driver interrupts.
     *
     * Can be overridden at the project level.
     */
    #define UART_IRQ_PRIORITY (2)
#endif

/*!
 * @brief Configuration of the UART IRQ driver state structure
 */
typedef struct UARTIrqConfig {
    uart_state_t  * uartStateIrq; /*!< Pointer to UART driver state information. */
} uart_irq_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Contains global IRQ configuration information for the UART driver. */
extern uart_irq_config_t g_uartIrqConfig[UART_INSTANCE_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Set the UART state information for the UART IRQ handler.
 *        The state structure values are defined in the driver and passed
 *        in through this function and are stored globally for use with the
 *        IRQ handler.
 */
static inline void uart_set_irq_state(uint32_t instance, void * uartDriverState)
{
    g_uartIrqConfig[instance].uartStateIrq = uartDriverState;
}

/* Weak extern for the UART driver's interrupt handler. */
#pragma weak uart_irq_handler
void uart_irq_handler(uart_state_t * uartStateIrq);

#endif /* __FSL_UART_IRQ_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

