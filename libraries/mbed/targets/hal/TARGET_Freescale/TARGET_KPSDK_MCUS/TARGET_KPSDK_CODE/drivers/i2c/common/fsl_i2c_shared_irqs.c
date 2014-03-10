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

#include "fsl_i2c_shared_irqs.h"
#include <assert.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/

i2c_shared_irq_config_t g_i2cSharedIrqConfig[HW_I2C_INSTANCE_COUNT];

/*!
 * @brief Table to save I2C IRQ enum numbers defined in CMSIS files. 
 *
 * This is used by I2C master and slave init functions to enable or disable I2C interrupts. 
 * This table is indexed by the module instance number and returns I2C IRQ numbers.
 */
#if defined (K64F12_SERIES)
const IRQn_Type i2c_irq_ids[HW_I2C_INSTANCE_COUNT] = {I2C0_IRQn, I2C1_IRQn, I2C2_IRQn};
#else
const IRQn_Type i2c_irq_ids[HW_I2C_INSTANCE_COUNT] = {I2C0_IRQn, I2C1_IRQn};
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void i2c_handle_shared_irq(uint32_t instance);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Pass IRQ control to either the master or slave driver. 
 *
 * The address of the IRQ handlers are checked to make sure they are non-zero before
 * they are called. If the IRQ handler's address is zero, it means that driver was
 * not present in the link (because the IRQ handlers are marked as weak). This would
 * actually be a program error, because it means the master/slave config for the IRQ
 * was set incorrectly.
 *
 * @param instance   Instance number of the I2C module.
 */
static void i2c_handle_shared_irq(uint32_t instance)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);

    if (g_i2cSharedIrqConfig[instance].isMaster)
    {
        /* Master mode.*/
        if (&i2c_master_irq_handler)
        {
            i2c_master_irq_handler(g_i2cSharedIrqConfig[instance].state);
        }
    }
    else
    {
        /* Slave mode.*/
        if (&i2c_slave_irq_handler)
        {
            i2c_slave_irq_handler(instance);
        }
    }
}

/*!
 * @brief Implementation of I2C0 handler named in startup code.
 *
 * Passes instance to generic I2C IRQ handler.
 */
void I2C0_IRQHandler(void)
{
    i2c_handle_shared_irq(HW_I2C0);
}

/*!
 * @brief Implementation of I2C1 handler named in startup code.
 *
 * Passes instance to generic I2C IRQ handler.
 */
void I2C1_IRQHandler(void)
{
    i2c_handle_shared_irq(HW_I2C1);
}

#if defined (K64F12_SERIES)  
/*!
 * @brief Implementation of I2C2 handler named in startup code.
 *
 * Passes instance to generic I2C IRQ handler.
 */
void I2C2_IRQHandler(void)
{
    i2c_handle_shared_irq(HW_I2C2);
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/

