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

#include "fsl_uart_irq.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*!
 * @brief Declare instantiations of the irq configuration. This is used in the irq handler
 * to keep track of the driver run-time state.
 */
uart_irq_config_t g_uartIrqConfig[UART_INSTANCE_COUNT];

/*!
 * @brief Table to save UART enum numbers defined in CMSIS files.
 *
 * This is used by the UART init function to enable or disable UART interrupts.
 * This table is indexed by the module instance number and returns UART IRQ numbers.
 */
#if defined (KL25Z4_SERIES)
IRQn_Type uart_irq_ids[UART_INSTANCE_COUNT] = {UART0_IRQn, UART1_IRQn, UART2_IRQn};
#elif defined (K64F12_SERIES) || defined (K70F12_SERIES) || defined (K22F51212_SERIES)
IRQn_Type uart_irq_ids[UART_INSTANCE_COUNT] =
{ UART0_RX_TX_IRQn,
  UART1_RX_TX_IRQn,
  UART2_RX_TX_IRQn,
  #if defined (K64F12_SERIES) || defined (K70F12_SERIES)
  UART3_RX_TX_IRQn,
  UART4_RX_TX_IRQn,
  UART5_RX_TX_IRQn
  #endif
};
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void uart_handle_irq(uint32_t instance);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Pass IRQ control to the driver.
 *
 * The address of the IRQ handler is checked to make sure it is non-zero before
 * it is called. If the IRQ handler's address is zero, it means that driver was
 * not present in the link (because the IRQ handlers are marked as weak). This would
 * actually be a program error, because it means the config for the IRQ
 * was set incorrectly.
 *
 * @param instance Module instance number
 */
static void uart_handle_irq(uint32_t instance)
{
    if (&uart_irq_handler)
    {
        /* UART IRQ handler defined in the driver */
        uart_irq_handler(g_uartIrqConfig[instance].uartStateIrq);
    }
}

#if defined (KL25Z4_SERIES)
/*!
 * @brief Implementation of UART0 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART0_IRQHandler(void)
{
    uart_handle_irq(0);
}

/*!
 * @brief Implementation of UART1 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART1_IRQHandler(void)
{
    uart_handle_irq(1);
}

/*!
 * @brief Implementation of UART2 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART2_IRQHandler(void)
{
    uart_handle_irq(2);
}

#elif defined (K64F12_SERIES) || defined (K70F12_SERIES) || defined (K22F51212_SERIES)
/*!
 * @brief Implementation of UART0 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART0_RX_TX_IRQHandler(void)
{
    uart_handle_irq(0);
}

/*!
 * @brief Implementation of UART1 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART1_RX_TX_IRQHandler(void)
{
    uart_handle_irq(1);
}

/*!
 * @brief Implementation of UART2 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART2_RX_TX_IRQHandler(void)
{
    uart_handle_irq(2);
}

/*!
 * @brief Implementation of UART3 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART3_RX_TX_IRQHandler(void)
{
    uart_handle_irq(3);
}

/*!
 * @brief Implementation of UART4 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART4_RX_TX_IRQHandler(void)
{
    uart_handle_irq(4);
}

/*!
 * @brief Implementation of UART5 handler named in startup code.
 *
 * Passes instance to generic UART IRQ handler.
 */
void UART5_RX_TX_IRQHandler(void)
{
    uart_handle_irq(5);
}
#endif
/*******************************************************************************
 * EOF
 ******************************************************************************/

