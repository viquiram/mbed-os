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

#include "fsl_soundcard.h"

 /*************************************************************************
 * Variables
 ************************************************************************/

#if USEDMA
extern void sai_tx_error_handle(uint32_t instance);
extern void sai_rx_error_handle(uint32_t instance);
#else
extern void sai_tx_interrupt_handle(uint32_t instance);
extern void sai_rx_interrupt_handle(uint32_t instance);
#endif

#if defined (K70F12_SERIES) || defined (K64F12_SERIES) || defined (K22F51212_SERIES)
IRQn_Type sai_irq_ids[HW_I2S_INSTANCE_COUNT][2] = 
{
    {I2S0_Tx_IRQn, I2S0_Rx_IRQn}
#if defined (K70F12_SERIES)
    , {I2S1_Tx_IRQn, I2S1_Rx_IRQn}
#endif
};
#endif

/*************************************************************************
 * Code
 ************************************************************************/
/* I2S IRQ handler with the same name in startup code */

#if defined (K70F12_SERIES) || defined (K64F12_SERIES) || defined (K22F51212_SERIES)

void I2S0_Tx_IRQHandler(void)
{
#if USEDMA
    sai_tx_error_handle(0U);
#else
    sai_tx_interrupt_handle(0U);
#endif
}

void I2S0_Rx_IRQHandler(void)
{
#if USEDMA
    sai_rx_error_handle(0U);
#else
    sai_rx_interrupt_handle(0U);
#endif

}

#if defined (K70F12_SERIES)
void I2S1_Tx_IRQHandler(void)
{
#if USEDMA
    sai_tx_error_handle(1U);
#else
    sai_tx_interrupt_handle(1U);
#endif

}

void I2S1_Rx_IRQHandler(void)
{
#if USEDMA
    sai_tx_error_handle(1U);
#else
    sai_tx_interrupt_handle(1U);
#endif

}
#endif /*defined K70F12_SERIES */

#endif

/*************************************************************************
 * EOF
 ************************************************************************/

