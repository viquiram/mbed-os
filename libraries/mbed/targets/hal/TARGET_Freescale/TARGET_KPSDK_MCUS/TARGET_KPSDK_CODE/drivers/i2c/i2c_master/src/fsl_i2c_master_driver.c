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

#include <assert.h>
#include <string.h>
#include "fsl_i2c_master_driver.h"
#include "fsl_i2c_shared_irqs.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

enum _i2c_master_constants
{
    /*! The mask for a 7-bit I2C slave address.*/
    kI2CAddress7bitMask = 0x7F
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern IRQn_Type i2c_irq_ids[HW_I2C_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

 /*FUNCTION**********************************************************************
 *
 * Function Name : i2c_master_init
 * Description   : initializes the I2C master mode driver.
 * This function will initialize the I2C master mode driver, enable I2C clock,
 * and enable I2C interrupt.
 *
 *END**************************************************************************/
void i2c_master_init(uint32_t instance, i2c_master_t * master)
{
    assert(master);

    /* Init driver instance struct.*/
    memset(master, 0, sizeof(*master));
    master->instance = instance;
    sync_create(&master->irqSync, 0);

    /* Enable clock for I2C.*/
    clock_manager_set_gate(kClockModuleI2C, instance, true);

    /* Reset the peripheral.*/
    i2c_hal_reset(instance);

    /* Enable module.*/
    i2c_hal_enable(master->instance);

    /* Enable I2C interrupt.*/
    interrupt_enable(i2c_irq_ids[instance]);

    /* Configured shared IRQ.*/
    i2c_set_shared_irq_is_master(instance, true);
    i2c_set_shared_irq_state(instance, master);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_master_shutdown
 * Description   : shut down the I2C master mode driver.
 * This function will shut down the I2C master mode driver, disable I2C clock,
 * and disable I2C interrupt.
 *
 *END**************************************************************************/
void i2c_master_shutdown(i2c_master_t * master)
{
    assert(master);

    /* Disable module.*/
    i2c_hal_disable(master->instance);

    /* Disable clock for I2C.*/
    clock_manager_set_gate(kClockModuleI2C, master->instance, false);

    /* Disable the interrupt*/
    interrupt_disable(i2c_irq_ids[master->instance]);

    /* Cleared state pointer stored in shared IRQ handler.*/
    i2c_set_shared_irq_state(master->instance, NULL);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_master_configure_bus
 * Description   : configures the I2C bus to access a device.
 * This function will set baud rate.
 *
 *END**************************************************************************/
i2c_status_t i2c_master_configure_bus(i2c_master_t * master, const i2c_device_t * device)
{
    assert(master);
    assert(device);

    /* Set baud rate if different.*/
    if (device->baudRate_kbps != master->lastBaudRate_kbps)
    {
        /* Get the current bus clock.*/
        uint32_t busClock;
        clock_manager_error_code_t error = clock_manager_get_frequency(kBusClock, &busClock);
        if (error)
        {
            return kStatus_I2C_InvalidArgument;
        }

        i2c_hal_set_baud(master->instance, busClock, device->baudRate_kbps, NULL);
    }

    return kStatus_I2C_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_master_transfer
 * Description   : performs a blocking read or write transaction on the I2C bus.
 * This function will configure bus for the specified devices, send the device
 * address with r/w direction. If we have a subaddress, then that is always done
 * as a write transfer prior to transferring the actual data. At the last, perform
 * the main transfer.
 *
 *END**************************************************************************/
i2c_status_t i2c_master_transfer(i2c_master_t * master,
                      const i2c_device_t * device,
                      i2c_direction_t direction,
                      bool stopAfterTransfer,
                      uint32_t subaddress,
                      size_t subaddressLength,
                      uint8_t * data,
                      size_t dataLength,
                      size_t * actualLengthTransferred,
                      uint32_t timeout_ms)
{
    assert(master);

    /* Don't let another transfer start if one is already in progress.*/
    if (master->isTransferInProgress)
    {
        return kStatus_I2C_Busy;
    }

    /* Configure bus for the specified device.*/
    i2c_master_configure_bus(master, device);

    /* Send the device address with r/w direction. If we have a subaddress, then that is*/
    /* always done as a write transfer prior to transferring the actual data.*/
    uint8_t slaveAddress = (device->address & kI2CAddress7bitMask) << 1U;
    if (subaddressLength)
    {
        slaveAddress |= kI2CWrite;
    }
    else
    {
        slaveAddress |= direction;
    }

    /* Send slave address.*/
    size_t bytesTransferred;
    i2c_status_t error = i2c_master_transfer_basic(master, kI2CNoStop, kI2CWrite, &slaveAddress,
                                                   sizeof(slaveAddress), &bytesTransferred,
                                                   timeout_ms);
    if (error)
    {
        return error;
    }

    /* Send subaddress if one was provided.*/
    if (subaddressLength)
    {
        error = i2c_master_transfer_basic(master, kI2CNoStart | kI2CNoStop, kI2CWrite,
                                          (uint8_t *)&subaddress, subaddressLength,
                                          &bytesTransferred, timeout_ms);
        if (error)
        {
            return error;
        }

        /* If performing a read, we have to do a restart to switch from write to read.*/
        if (direction == kI2CRead)
        {
            slaveAddress |= kI2CRead;

            error = i2c_master_transfer_basic(master, kI2CNoStop, kI2CWrite, &slaveAddress,
                                              sizeof(slaveAddress), &bytesTransferred, timeout_ms);
            if (error)
            {
                return error;
            }
        }
    }

    /* Now perform the main transfer.*/
    error = i2c_master_transfer_basic(master, kI2CNoStart | (stopAfterTransfer ? 0u : kI2CNoStop),
                                      direction, data, dataLength,
                                      actualLengthTransferred, timeout_ms);

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : i2c_master_transfer_basic
 * Description   : perform a transfer.
 * This function will initiate (start) a transfer. This is not a public API as it
 * is called from other driver functions
 *
 *END**************************************************************************/
i2c_status_t i2c_master_transfer_basic(i2c_master_t * master,
                      uint32_t flags,
                      i2c_direction_t direction,
                      uint8_t * data,
                      size_t dataLength,
                      size_t * actualLengthTransferred,
                      uint32_t timeout_ms)
{
    assert(master);

    i2c_status_t error = kStatus_I2C_Success;
    fsl_rtos_status syncStatus;

    /* Don't let another transfer start if one is already in progress.*/
    if (master->isTransferInProgress)
    {
        return kStatus_I2C_Busy;
    }

    /* Save critical state information for the ISR.*/
    master->isTransferInProgress = true;
    master->flags = flags;
    master->direction = direction;
    master->data = data;
    master->dataRemainingCount = dataLength;
    master->bytesTransferredCount = 0;
    master->gotNak = false;

    /* Set to transmit mode.*/
    i2c_hal_set_direction(master->instance, direction == kI2CWrite ? kI2CTransmit : kI2CReceive);

    /* Send a START signal if requested.*/
    if (!(flags & kI2CNoStart))
    {
        i2c_hal_send_start(master->instance);
    }
    else
    {
        /* The NoStart flag was set, so make sure we are already a master.*/
        assert(i2c_hal_is_master(master->instance));
    }

    /* Set the flag to send a NAK after this first byte if there is only one byte to transfer.*/
    if (direction == kI2CRead)
    {
        if (master->dataRemainingCount == 1)
        {
            i2c_hal_send_nak(master->instance);
        }
        else
        {
            i2c_hal_send_ack(master->instance);
        }
    }

    /* Enable the I2C interupt.*/
    i2c_hal_clear_interrupt(master->instance);
    i2c_hal_enable_interrupt(master->instance);

    /* Read or write the first byte.*/
    if (direction == kI2CWrite)
    {
        i2c_hal_write(master->instance, data[0]);
        ++master->data;
    }
    else
    {
        /* Dummy read to initiate a read transfer.*/
        i2c_hal_read(master->instance);
    }

    /* Wait for the transfer to finish.*/
    do
    {
        syncStatus = sync_wait(&master->irqSync, timeout_ms);
    }while(syncStatus == kIdle);
    
    if (syncStatus != kSuccess)
    {
        /* Return an error.*/
        error = kStatus_I2C_Timeout;
    }

    /* Disable the I2C interrupt.*/
    i2c_hal_disable_interrupt(master->instance);

    /* Set the error to indicate if we got a NAK.*/
    if (master->gotNak)
    {
        error = kStatus_I2C_ReceivedNak;
    }

    /* Return actual number of bytes transferred to caller.*/
    if (actualLengthTransferred)
    {
        *actualLengthTransferred = master->bytesTransferredCount;
    }

    /* A transfer is no longer in progress.*/
    master->isTransferInProgress = false;

    return error;
}

/*!
 * @brief I2C master IRQ handler.
 * This handler uses the buffers stored in the i2c_master_t structs to transfer data.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void i2c_master_irq_handler(void * state)
{
    assert(state);

    i2c_master_t * master = (i2c_master_t *)state;
    uint32_t instance = master->instance;
    bool signalCompletion = false;

    /* Go ahead and clear the interrupt flag.*/
    i2c_hal_clear_interrupt(instance);

    /* Exit immediately if there is no transfer in progress.*/
    if (!master->isTransferInProgress)
    {
        return;
    }

    /* Check that the device is in master mode.*/
    if (!i2c_hal_is_master(instance))
    {
        return;
    }

    /* Update counters.*/
    --master->dataRemainingCount;
    ++master->bytesTransferredCount;

    /* Make sure the last byte was successful.*/
    if (master->dataRemainingCount == 0)
    {
        /* Check whether we got an ACK or NAK from the last byte we sent.*/
        if ((!i2c_hal_get_receive_ack(instance)) && (master->direction == kI2CWrite))
        {
            /* Record that we got a NAK.*/
            master->gotNak = true;
        }

        if (!((master->flags) & kI2CNoStop))
        {
            i2c_hal_send_stop(instance);
        }

        /* We're done after this ISR.*/
        signalCompletion = true;
    }

    /* Handle transmit.*/
    if (master->direction == kI2CWrite)
    {
        /* Was the last byte just transferred?*/
        if (master->dataRemainingCount > 0)
        {
            /* Check whether we got an ACK or NAK from the former byte we sent.*/
            if (!i2c_hal_get_receive_ack(instance))
            {
                /* Record that we got a NAK.*/
                master->gotNak = true;

                /* Got a NAK, so we're done with this transfer.*/
                signalCompletion = true;

                /* Always send a stop in this case.*/
                i2c_hal_send_stop(instance);
            }
            else
            {
                /* Transfer the next byte.*/
                assert(master->dataRemainingCount > 0);
                i2c_hal_write(instance, *(master->data));

                /* Update buffer info.*/
                ++master->data;
            }
        }
    }
    /* Handle receive.*/
    else
    {
        /* For the 2nd to last byte, we need to set NAK.*/
        if (master->dataRemainingCount == 1)
        {
            i2c_hal_send_nak(instance);
        }
        else
        {
            i2c_hal_send_ack(instance);
        }

        /* Read last received byte into buffer.*/
        *(master->data) = i2c_hal_read(instance);

        /* Update buffer info.*/
        ++master->data;
    }

    /* Signal the sync object.*/
    if (signalCompletion)
    {
        sync_signal_from_isr(&master->irqSync);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

