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
#if !defined(__FSL_I2C_SHARED_IRQS_H__)
#define __FSL_I2C_SHARED_IRQS_H__

#include "fsl_device_registers.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if !defined(I2C_MASTER_IRQ_PRIORITY)
    /*! @brief The priority for I2C master driver interrupts.*/
    /*!*/
    /*! Can be overridden at the project level.*/
    #define I2C_MASTER_IRQ_PRIORITY (2)
#endif

#if !defined(I2C_SLAVE_IRQ_PRIORITY)
    /*! @brief The priority for I2C slave driver interrupts.*/
    /*!*/
    /*! Can be overridden at the project level.*/
    #define I2C_SLAVE_IRQ_PRIORITY (2)
#endif

/*! @brief Configuration of the I2C IRQs shared by master and slave drivers.*/
/*!*/
/*! */
typedef struct I2CSharedIrqConfig {
    bool isMaster;  /*!< Whether the IRQ is used by the master mode driver.*/
    void * state;   /*!< Pointer to state information.*/
} i2c_shared_irq_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Contains the global IRQ configuration information for the SPI drivers.*/
extern i2c_shared_irq_config_t g_i2cSharedIrqConfig[HW_I2C_INSTANCE_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! @brief Sets whether the master or slave driver IRQ handler are invoked.*/
static inline void i2c_set_shared_irq_is_master(uint32_t instance, bool isMaster)
{
    g_i2cSharedIrqConfig[instance].isMaster = isMaster;
}

/*! @brief Sets the state information for the shared IRQ handler.*/
static inline void i2c_set_shared_irq_state(uint32_t instance, void * state)
{
    g_i2cSharedIrqConfig[instance].state = state;
}

/* Weak external for the I2C master driver interrupt handler.*/
#pragma weak i2c_master_irq_handler
void i2c_master_irq_handler(void * state);

/* Weak external for the I2C slave driver interrupt handler.*/
#pragma weak i2c_slave_irq_handler
void i2c_slave_irq_handler(uint32_t instance);

#endif /* __FSL_I2C_SHARED_IRQS_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

