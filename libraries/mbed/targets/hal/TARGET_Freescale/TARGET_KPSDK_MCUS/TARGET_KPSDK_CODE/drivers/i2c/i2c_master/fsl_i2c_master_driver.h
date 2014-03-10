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
#ifndef __FSL_I2C_MASTER_DRIVER_H__
#define __FSL_I2C_MASTER_DRIVER_H__

#include <stdlib.h>
#include <stdbool.h>
#include "fsl_i2c_hal.h"
#include "fsl_os_abstraction.h"

/*!
 * @addtogroup i2c_master
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Information necessary to communicate with an I2C slave device.*/
typedef struct I2CDeviceInfo 
{
    uint8_t address;        /*!< The slave's 7-bit address.*/
    uint32_t baudRate_kbps; /*!< The baud rate in kbps to use with this slave device.*/
} i2c_device_t;

/*! @brief Constants for the direction of an I2C transfer.*/
typedef enum I2CDirection 
{
    kI2CRead = 1,   /*!< Read from slave device.*/
    kI2CWrite = 0   /*!< Write to slave device.*/
} i2c_direction_t;

/*! @brief Optional flags to control a transfer.*/
enum _i2c_transfer_flags
{
    kI2CNoStart = 1 << 1,   /*!< Set this flag to prevent sending a START signal.*/
    kI2CNoStop = 1 << 2     /*!< Set this flag to prevent sending a STOP signal.*/
};

/*!
 * @brief Internal driver state information.
 *
 * @note The contents of this structure are internal to the driver and should not be
 *      modified by users. Also, contents of the structure are subject to change in
 *      future releases.
 */
typedef struct I2CMasterState {
    uint32_t instance;
    bool isTransferInProgress;
    uint32_t flags;
    i2c_direction_t direction;
    uint8_t * data;
    volatile size_t dataRemainingCount;
    volatile size_t bytesTransferredCount;
    volatile bool gotNak;
    uint32_t lastBaudRate_kbps;
    sync_object_t irqSync;
} i2c_master_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name I2C Master
 * @{
 */

/*!
 * @brief Initialize the I2C master mode driver.
 *
 * @param instance The I2C peripheral instance number.
 * @param master   The pointer to the I2C master driver state structure.
 */
void i2c_master_init(uint32_t instance, i2c_master_t * master);

/*!
 * @brief Shut down the driver.
 *
 * @param master   The pointer to the I2C master driver state structure.
 */
void i2c_master_shutdown(i2c_master_t * master);

/*!
 * @brief Configure the I2C bus to access a device.
 *
 * @param master   The pointer to the I2C master driver state structure.
 * @param device   The pointer to the I2C device information struct.
 */
i2c_status_t i2c_master_configure_bus(i2c_master_t * master, const i2c_device_t * device);

/*!
 * @brief Low-level I2C transfer function.
 *
 * @param master                   The pointer to the I2C master driver state structure.
 * @param flags                    The flags to control a transfer.
 * @param direction                The direction of an I2C transfer.
 * @param data                     The pointer to the data to be transfered.
 * @param dataLength               The length in bytes of the data to be transfered.
 * @param actualLengthTransferred  The length in bytes of the data thansfered.
 * @param timeous_ms               A timeout for the transfer in microseconds.
 */
i2c_status_t i2c_master_transfer_basic(i2c_master_t * master, 
                      uint32_t flags,
                      i2c_direction_t direction,
                      uint8_t * data,
                      size_t dataLength,
                      size_t * actualLengthTransferred,
                      uint32_t timeout_ms);

/*!
 * @brief Perform a blocking read or write transaction on the I2C bus.
 *
 * @param master                   The pointer to the I2C master driver state structure.
 * @param device                   The pointer to the I2C device information struct.
 * @param direction                The direction of an I2C transfer.
 * @param stopAfterTransfer        Send STOP signal after this transfer or not.
 * @param subaddress               The subaddress for a device if it has.
 * @param subaddressLength         The length of the subaddress.
 * @param data                     The pointer to the data to be transfered.
 * @param dataLength               The length in bytes of the data to be transfered.
 * @param actualLengthTransferred  The length in bytes of the data thansfered.
 * @param timeous_ms               A timeout for the transfer in microseconds.
 */
i2c_status_t i2c_master_transfer(i2c_master_t * master, 
                      const i2c_device_t * device,
                      i2c_direction_t direction,
                      bool stopAfterTransfer,
                      uint32_t subaddress,
                      size_t subaddressLength,
                      uint8_t * data,
                      size_t dataLength,
                      size_t * actualLengthTransferred,
                      uint32_t timeout_ms);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_I2C_MASTER_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

