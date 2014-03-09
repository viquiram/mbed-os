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
#ifndef __FSL_I2C_SLAVE_H__
#define __FSL_I2C_SLAVE_H__

#include <stdint.h>
#include "fsl_i2c_hal.h"

/*! @addtogroup i2c_slave*/
/*! @{*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 *  brief Definition of application-implemented callback functions used by the I2C slave driver.
 */
typedef struct I2cSlaveUserConfig
{
    i2c_status_t (*data_source)(uint8_t * source_byte);   /*!< Callback to get byte to transmit.*/
    i2c_status_t (*data_sink)(uint8_t sink_byte);         /*!< Callback to put received byte.*/
    void (*on_error)(i2c_status_t error);                 /*!< Callback to report an I2C error.*/
    uint8_t slaveAddress;                                 /*!< 7-bit slave address.*/
} i2c_slave_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name I2C Slave*/
/*@{*/

/*!
 * @brief Initializes the I2C module. 
 *
 * Saves the application callback info, turns on the clock to the module,
 * enables the device, and enables interrupts. Sets the I2C to slave mode. 
 * IOMux is expected to be already handled in the init_hardware().
 *
 * @param instance   Instance number of the I2C module.
 * @param appInfo    Pointer of the application information.
 */
void i2c_slave_init(uint32_t instance, i2c_slave_user_config_t * appInfo);

/*!
 * @brief Shuts down the I2C slave driver.
 *
 * Clears the control register and turns off the clock to the module.
 *
 * @param instance  Instance number of the I2C module.
 */
void i2c_slave_shutdown(uint32_t instance);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_I2C_SLAVE_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

