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
#if !defined(__FSL_INTERRUPT_MANAGER_H__)
#define __FSL_INTERRUPT_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_interrupt_features.h"
#include "device/fsl_device_registers.h"

/*! @addtogroup interrupt_manager*/
/*! @{*/

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*! @name interrupt_manager APIs*/
/*@{*/

/*!
 * @brief Install an interrupt handler routine for a given IRQ number. 
 *
 * This function will let application to register/replace the interrupt 
 * handler for specified IRQ number. IRQ number is different with Vector
 * number. IRQ 0 will start from Vector 16 address. Refer to reference
 * manual for details. Also refer to startup_MKxxxx.s file for each chip
 * family to find out the default interrut handler for each device. This
 * function will convert the IRQ number to vector number by adding 16 to
 * it. 
 *
 * @param irqNumber IRQ number
 * @param handler   Interrupt handler routine address pointer
 */
void interrupt_register_handler(IRQn_Type irqNumber, void (*handler)(void));

/*!
 * @brief Enable an interrupt for giving IRQ number. 
 *
 * This function will enable the individual interrupt for specified IRQ
 * number. It will call the system NVIC API to access the interrupt control
 * register. The input IRQ number will not include the core interrupt, just
 * the peripheral interrupt. That will be from 0 to maximum supported IRQ.
 *
 * @param irqNumber IRQ number
 */
static inline void interrupt_enable(IRQn_Type irqNumber)
{
    /* check IRQ number */
    assert(0 <= irqNumber);
    assert(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);

    /* call core API to enable the IRQ*/
    NVIC_EnableIRQ(irqNumber);
}

/*!
 * @brief Disnable an interrupt for giving IRQ number. 
 *
 * This function will enable the individual interrupt for specified IRQ
 * number. It will call the system NVIC API to access the interrupt control
 * register.
 *
 * @param irqNumber IRQ number
 */
static inline void interrupt_disable(IRQn_Type irqNumber)
{
    /* check IRQ number */
    assert(0 <= irqNumber);
    assert(irqNumber <= FSL_FEATURE_INTERRUPT_IRQ_MAX);

    /* call core API to disable the IRQ*/
    NVIC_DisableIRQ(irqNumber);
}

/*!
 * @brief Enable system interrupt.
 *
 * This function will enable the global interrupt by calling the core API
 *
 */
void interrupt_enable_global(void);

/*!
 * @brief Disnable system interrupt. 
 *
 * This function will disable the global interrupt by calling the core API
 *
 */
void interrupt_disable_global(void);

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* __FSL_INTERRUPT_MANAGER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

