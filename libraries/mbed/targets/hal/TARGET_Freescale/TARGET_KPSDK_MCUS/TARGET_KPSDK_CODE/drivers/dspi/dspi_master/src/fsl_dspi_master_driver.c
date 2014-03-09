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

#include <string.h>
#include "fsl_dspi_master_driver.h"
#include "fsl_dspi_shared_irqs.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern IRQn_Type dspi_irq_ids[HW_SPI_INSTANCE_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static dspi_status_t dspi_master_start_transfer(dspi_master_state_t *dspiState,
                                           const dspi_device_t * restrict device);

static void dspi_master_complete_transfer(dspi_master_state_t * dspiState);
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_init
 * Description   : Initialize a DSPI instance for master mode operation.
 * This function will initialize the run-time state structure to keep track of the on-going
 * transfers, ungate the clock to the DSPI module, reset the DSPI module, initialize the module
 * to user defined settings and default settings, configure the IRQ state structure and enable
 * the module-level interrupt to the core, and enable the DSPI module.
 * The CTAR parameter is special in that it allows the user to have different SPI devices
 * connected to the same DSPI module instance in conjunction with different peripheral chip
 * selects. Each CTAR contains the bus attributes associated with that particular SPI device.
 * For simplicity and for most use cases where only one SPI device is connected per DSPI module
 * instance, it is recommended to use CTAR0.
 * The following is an example of how to set up the dspi_master_state_t and the
 * dspi_master_user_config_t parameters and how to call the dspi_master_init function by passing
 * in these parameters:
 *   dspi_master_state_t dspiMasterState; <- the user simply allocates memory for this struct
 *   uint32_t calculatedBaudRate;
 *   dspi_master_user_config_t userConfig; <- the user fills out members for this struct
 *   userConfig.isChipSelectContinuous = false;
 *   userConfig.isSckContinuous = false;
 *   userConfig.pcsPolarity = kDspiPcs_ActiveLow;
 *   userConfig.whichCtar = kDspiCtar0;
 *   userConfig.whichPcs = kDspiPcs0;
 *   dspi_master_init(masterInstance, &dspiMasterState, &userConfig, &calculatedBaudRate);
 *
 *END**************************************************************************/
dspi_status_t dspi_master_init(uint32_t instance,
                               dspi_master_state_t * dspiState,
                               const dspi_master_user_config_t * userConfig,
                               uint32_t * calculatedBaudRate)

{
    uint32_t dspiSourceClock;
    dspi_status_t errorCode = kStatus_DSPI_Success;

    /* DSPI config struct in hal, fill out it's members below*/
    dspi_master_config_t dspiConfig;

    /* Clear the run-time state struct for this instance.*/
    memset(dspiState, 0, sizeof(* dspiState));

    /* configure the run-time state struct with the instance number*/
    dspiState->instance = instance;

    /* Note, remember to first enable clocks to the DSPI module before making any register accesses
     * Enable clock for DSPI
     */
    clock_manager_set_gate(kClockModuleSPI, instance, true);
    /* Get module clock freq*/
    clock_manager_get_frequency(kBusClock, &dspiSourceClock);

    /* Reset the DSPI module */
    dspi_hal_reset(instance);

    /* Configure the run-time state struct with the DSPI source clock */
    dspiState->dspiSourceClock = dspiSourceClock;

    /* Configure the run-time state struct with the data command parameters*/
    dspiState->whichCtar = userConfig->whichCtar;  /* set the dspiState struct CTAR*/
    dspiState->whichPcs = userConfig->whichPcs;  /* set the dspiState strcut whichPcs*/
    dspiState->isChipSelectContinuous = userConfig->isChipSelectContinuous; /* continuous PCS*/

    /* Initialize the parameters of the hal DSPI config structure with desired data
     * members of the user config struct, then the hal init function will be called
     */
    dspiConfig.isEnabled = false;  /* disable the DSPI module while we're setting it up*/
    dspiConfig.whichCtar = userConfig->whichCtar;  /* select which CTAR to use*/
    dspiConfig.bitsPerSec = 0;  /* DSPI sck freq in Hz, set to 0 and config later*/
    dspiConfig.sourceClockInHz = dspiSourceClock;  /* module clock source*/
    dspiConfig.isSckContinuous = userConfig->isSckContinuous;
    dspiConfig.whichPcs = userConfig->whichPcs;  /* select the desired PCS signal to use*/
    dspiConfig.pcsPolarity = userConfig->pcsPolarity; /* select the desired PCS polarity*/
    dspiConfig.masterInSample = kDspiSckToSin_0Clock; /* set to default value */
    dspiConfig.isModifiedTimingFormatEnabled = false; /* disable modified timing format, deafault */
    dspiConfig.isTxFifoDisabled = false;  /* enable tx fifo*/
    dspiConfig.isRxFifoDisabled = false;  /* enable rx fifo*/

    /* data format field config
     * the user will configure these settings in the device config struct
     * so for now, initialize them to various default settings
     */
    dspiConfig.dataConfig.bitsPerFrame = 16;
    dspiConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    dspiConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    dspiConfig.dataConfig.direction = kDspiMsbFirst;

    dspiState->bitsPerFrame = dspiConfig.dataConfig.bitsPerFrame; /* update dspiState bits/frame */

    /* Do nothing for calculatedBaudRate. If the user wants to know the calculatedBaudRate
     * then they can call the configure bus function separately.
     */
    errorCode = dspi_hal_master_init(instance, &dspiConfig, calculatedBaudRate);
    if (errorCode!= kStatus_DSPI_Success)
    {
        return errorCode;  /* return immediately if there's a problem with the init*/
    }

    /* Initialize the configurable delays: PCS-to-SCK, after SCK delay, and delay after transfer
     * as follows:
     */
    dspi_delay_settings_config_t delayConfig;
    delayConfig.pcsToSck = 1;            /*!< PCS to SCK delay (CSSCK): initialize the scalar
                                          *   value to '1' to provide the master with a little
                                          *   more data-in read setup time.
                                          */
    delayConfig.pcsToSckPre = 0;         /*!< PCS to SCK delay prescalar (PCSSCK) */
    delayConfig.afterSckPre = 0;         /*!< After SCK delay prescalar (PASC)*/
    delayConfig.afterSck = 0;            /*!< After SCK delay scalar (ASC)*/
    delayConfig.afterTransferPre = 0;    /*!< Delay after transfer prescalar (PDT)*/
    delayConfig.afterTransfer = 0;
    dspi_hal_configure_delays(instance, dspiConfig.whichCtar, &delayConfig);

    /* Init the interrupt sync object.*/
    sync_create(&dspiState->irqSync, 0);

    /* Configure IRQ state structure, so irq handler can point to the correct state structure*/
    dspi_set_shared_irq_state(instance, dspiState, true);

    /* enable the interrupt*/
    interrupt_enable(dspi_irq_ids[instance]);

    /* DSPI system enable*/
    dspi_hal_enable(instance);

    /* Start the transfer process in the hardware */
    dspi_hal_start_transfer(instance);

    return errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_shutdown
 * Description   : Shutdown a DSPI instance.
 * This function resets the DSPI peripheral, gates its clock, and disables the interrupt to
 * the core.
 *
 *END**************************************************************************/
void dspi_master_shutdown(dspi_master_state_t * dspiState)
{
    uint32_t instance = dspiState->instance;

    /* First stop transfers */
    dspi_hal_stop_transfer(instance);

    /* Restore the module to defaults then power it down. This also disables the DSPI module.*/
    dspi_hal_reset(instance);

    /* disable the interrupt*/
    interrupt_disable(dspi_irq_ids[instance]);

    /* Gate the clock for DSPI.*/
    clock_manager_set_gate(kClockModuleSPI, instance, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_shutdown
 * Description   : Configure the DSPI modified transfer format in master mode.
 * This function allows the user to enable or disable (default setting) the modified transfer
 * format. The modified transfer format is supported to allow for high-speed communication with
 * peripherals that require longer setup times. The module can sample the incoming data
 * later than halfway through the cycle to give the peripheral more setup time. The data-in
 * sample point can also be configure in this function. Note, the data-in sample point setting
 * is valid only when the CPHA bit in the CTAR is cleared (when the dspi_clock_phase_t is
 * set to kDspiClockPhase_FirstEdge in the dspi_data_format_config_t).
 *
 *END**************************************************************************/
void dspi_master_configure_modified_transfer_format(dspi_master_state_t * dspiState,
                                                    bool enableOrDisable,
                                                    dspi_master_sample_point_t samplePnt)
{
    uint32_t instance = dspiState->instance;

    /* Enable or disable the modified transfer format */
    dspi_hal_configure_modified_timing_format(instance, enableOrDisable);

    /* Configure the master data-in sample point */
    dspi_hal_set_datain_samplepoint(instance, samplePnt);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_configure_bus
 * Description   : Configures the DSPI port physical parameters to access a device on the bus.
 * The term "device" is used to indicate the SPI device for which the DSPI master is communicating.
 * The user has two options to configure the device parameters: either pass in the
 * pointer to the device configuration structure to the desired transfer function (see
 * dspi_master_transfer or dspi_master_transfer_async) or pass it in to the
 * dspi_master_configure_bus function.  The user can pass in a device structure to the transfer
 * function which will contain the parameters for the bus (the transfer function will then call
 * this function). However, the user has the option to call this function directly especially if
 * they wish to obtain the calculated baud rate, at which point they may pass in NULL for the device
 * struct in the transfer function (assuming they have called this configure bus function
 * first). The following is an example of how to set up the dspi_device_t structure and how to call
 * the dspi_master_configure_bus function by passing in these parameters:
 *   dspi_device_t spiDevice;
 *   spiDevice.dataBusConfig.bitsPerFrame = 16;
 *   spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
 *   spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
 *   spiDevice.dataBusConfig.direction = kDspiMsbFirst;
 *   spiDevice.bitsPerSec = 50000;
 *   dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate);
 *
 *END**************************************************************************/
dspi_status_t dspi_master_configure_bus(dspi_master_state_t * dspiState,
                                        const dspi_device_t * device,
                                        uint32_t * calculatedBaudRate)
{
    assert(device);
    uint32_t instance = dspiState->instance;
    dspi_status_t errorCode = kStatus_DSPI_Success;

    /* Configure the bus to access the provided device.*/
    *calculatedBaudRate = dspi_hal_set_baud(instance, dspiState->whichCtar, device->bitsPerSec,
                      dspiState->dspiSourceClock);
    errorCode = dspi_hal_configure_data_format(instance, dspiState->whichCtar,
                                               &device->dataBusConfig);
    dspiState->bitsPerFrame = device->dataBusConfig.bitsPerFrame; /* update dspiState bits/frame */

    return errorCode;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_transfer
 * Description   : Perform a blocking SPI master mode transfer.
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function will not return until the transfer is complete.
 *
 *END**************************************************************************/
dspi_status_t dspi_master_transfer(dspi_master_state_t * dspiState,
                              const dspi_device_t * restrict device,
                              const uint8_t * sendBuffer,
                              uint8_t * receiveBuffer,
                              size_t transferByteCount,
                              uint32_t timeout)
{
    /* fill in members of the run-time state struct*/
    dspiState->isTransferAsync = false;
    dspiState->sendBuffer = (const uint8_t *)sendBuffer;
    dspiState->receiveBuffer = (uint8_t *)receiveBuffer;
    dspiState->remainingSendByteCount = transferByteCount;
    dspiState->remainingReceiveByteCount = transferByteCount;

    /* start the transfer process*/
    if (dspi_master_start_transfer(dspiState, device) == kStatus_DSPI_Busy)
    {
        return kStatus_DSPI_Busy;
    }

    /* As this is a synchronous transfer, wait until the transfer is complete.*/
    dspi_status_t error = kStatus_DSPI_Success;
    fsl_rtos_status syncStatus;

    do
    {
        syncStatus = sync_wait(&dspiState->irqSync, timeout);
    }while(syncStatus == kIdle);

    if (syncStatus != kSuccess)
    {
        error = kStatus_DSPI_Timeout;
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_transfer_async
 * Description   : Perform a non-blocking SPI master mode transfer.
 * This function will return immediately. It is the user's responsiblity to check back to
 * ascertain if the transfer is complete (using the dspi_master_get_transfer_status function). This
 * function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus.
 *
 *END**************************************************************************/
dspi_status_t dspi_master_transfer_async(dspi_master_state_t * dspiState,
                                    const dspi_device_t * restrict device,
                                    const uint8_t * sendBuffer,
                                    uint8_t * receiveBuffer,
                                    size_t transferByteCount)
{
    /* fill in members of the run-time state struct*/
    dspiState->isTransferAsync = true;
    dspiState->sendBuffer = sendBuffer;
    dspiState->receiveBuffer = (uint8_t *)receiveBuffer;
    dspiState->remainingSendByteCount = transferByteCount;
    dspiState->remainingReceiveByteCount = transferByteCount;

    /* start the transfer process*/
    if (dspi_master_start_transfer(dspiState, device) == kStatus_DSPI_Busy)
    {
        return kStatus_DSPI_Busy;
    }

    /* else, return immediately as this is an async transfer*/
    return kStatus_DSPI_Success;
}


/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_get_transfer_status
 * Description   : Returns whether the previous transfer has finished yet.
 * When performing an async transfer, the user can call this function to ascertain the state of the
 * current transfer: in progress (or busy) or complete (success). In addition, if the transfer
 * is still in progress, the user can obtain the number of words that have been currently
 * transferred.
 *
 *END**************************************************************************/
dspi_status_t dspi_master_get_transfer_status(dspi_master_state_t * dspiState,
                                         uint32_t * framesTransferred)
{
    uint32_t instance = dspiState->instance;

    /* Fill in the bytes transferred.*/
    if (framesTransferred)
    {
        *framesTransferred = dspi_hal_get_transfer_count(instance);
    }

    return (dspiState->isTransferInProgress ? kStatus_DSPI_Busy : kStatus_DSPI_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_master_abort_transfer
 * Description   : Terminates an asynchronous transfer early.
 * During an async transfer, the user has the option to terminate the transfer early if the transfer
 * is still in progress.
 *
 *END**************************************************************************/
dspi_status_t dspi_master_abort_transfer(dspi_master_state_t * dspiState)
{
    /* Check if a transfer is running.*/
    if (!dspiState->isTransferInProgress)
    {
        return kStatus_DSPI_NoTransferInProgress;
    }

    /* Stop the running transfer.*/
    dspi_master_complete_transfer(dspiState);

    return kStatus_DSPI_Success;
}

/*!
 * @brief Initiate (start) a transfer. This is not a public API as it is called from other
 *  driver functions
 */
static dspi_status_t dspi_master_start_transfer(dspi_master_state_t * dspiState,
                                           const dspi_device_t * restrict device)
{
    uint32_t instance = dspiState->instance;
    uint32_t calculatedBaudRate;
    dspi_command_config_t command;  /* create an instance of the data command struct*/
    uint16_t wordToSend = 0;
    /* Declare variables for storing volatile data later in the code */
    uint32_t remainingReceiveByteCount, remainingSendByteCount;

    /* Check the transfer byte count. If bits/frame > 8, meaning 2 bytes, then
     * the transfer byte count must not be an odd count, if so, drop the last odd byte.
     * Perform this operation here to ensure we get the latest bits/frame setting.
     */
    if (dspiState->bitsPerFrame > 8)
    {
        dspiState->remainingSendByteCount &= ~1UL;
        dspiState->remainingReceiveByteCount &= ~1UL;
    }

    /* If the transfer count is zero, then return immediately.*/
    if (dspiState->remainingSendByteCount == 0)
    {
        /* Signal the synchronous completion object if the transfer wasn't async.
         * Otherwise, when we return the the sync function we'll get stuck in the sync wait loop.
         */
        if (!dspiState->isTransferAsync)
        {
            sync_signal(&dspiState->irqSync);
        }

        return kStatus_DSPI_Success;
    }

    /* Check that we're not busy.*/
    if (dspiState->isTransferInProgress)
    {
        return kStatus_DSPI_Busy;
    }

    /* Configure bus for this device. If NULL is passed, we assume the caller has
     * preconfigured the bus using dspi_master_configure_bus().
     * Do nothing for calculatedBaudRate. If the user wants to know the calculatedBaudRate
     * then they can call this function separately.
     */
    if (device)
    {
        dspi_master_configure_bus(dspiState, device, &calculatedBaudRate);
        dspiState->bitsPerFrame = device->dataBusConfig.bitsPerFrame;/*update dspiState bits/frame*/
    }

    /* Save information about the transfer for use by the ISR.*/
    dspiState->isTransferInProgress = true;

    /* Enable the DSPI module for the duration of this transfer.*/
    dspi_hal_enable(instance);

    /* flush the fifos*/
    dspi_hal_flush_fifos(instance, true, true);

    /* Before sending the data, we first need to initialize the data command struct
     * Configure the data command attributes for the desired PCS, CTAR, and continuous PCS
     * which are derived from the run-time state struct
     */
    command.whichPcs = dspiState->whichPcs;
    command.whichCtar = dspiState->whichCtar;
    command.isChipSelectContinuous = dspiState->isChipSelectContinuous;

    /* If the data is the first word to be sent, then enable "clear the transfer count"
     * and set the firstWord flag. After the first word is sent, clear both of these.
     * Also, clear the end of queue member and set when it is determined that the last data
     * word is to be sent
     */
    command.clearTransferCount = 1;
    command.isEndOfQueue = 0;
    uint32_t firstWordFlag = 1;

    /* If bits/frame is greater than one byte */
    if (dspiState->bitsPerFrame > 8)
    {
        /* Fill the fifo until it is full or until the send word count is 0 or until the difference
         * between the remainingReceiveByteCount and remainingSendByteCount equals the FIFO depth.
         * The reason for checking the difference is to ensure we only send as much as the
         * RX FIFO can receive.
         * For this case wher bitsPerFrame > 8, each entry in the FIFO contains 2 bytes of the
         * send data, hence the difference between the remainingReceiveByteCount and
         * remainingSendByteCount must be divided by 2 to convert this difference into a
         * 16-bit (2 byte) value.
         */
        /* Store the DSPI state struct volatile member variables into temporary
         * non-volatile variables to allow for MISRA compliant calculations
         */
        remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
        remainingSendByteCount = dspiState->remainingSendByteCount;

        while((dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest) == 1) &&
              ((remainingReceiveByteCount - remainingSendByteCount)/2 <
                FSL_FEATURE_SPI_FIFO_SIZEn(instance)))
        {
            /* On the last word to be sent, set the end of queue flag in the data command struct
             * and ensure that the CONT bit in the PUSHR is also cleared even if it was cleared to
             * begin with. If CONT is set it means continuous chip select operation and to ensure
             * the chip select is de-asserted, this bit must be cleared on the last data word.
             */
            if ((dspiState->remainingSendByteCount - 2) == 0)
            {
                command.isEndOfQueue = 1;
                command.isChipSelectContinuous = 0;
            }

            /* If this is the first word to be transmitted, clear firstWord flag after transmit */
            if (firstWordFlag == 1)
            {
                /* If a send buffer was provided, the word comes from there. Otherwise we just send
                 * a zero (initialized above).
                 */
                if (dspiState->sendBuffer)
                {
                    wordToSend = *(dspiState->sendBuffer);
                    ++dspiState->sendBuffer; /* increment to next data byte */
                    wordToSend |= (unsigned)(*(dspiState->sendBuffer)) << 8U;
                    ++dspiState->sendBuffer; /* increment to next data byte */
                }

                dspi_hal_write_data_master_mode(instance, &command, wordToSend);
                dspiState->remainingSendByteCount -= 2; /* decrement remainingSendByteCount by 2 */

                firstWordFlag = 0; /* clear firstWordFlag */
                command.clearTransferCount = 0; /* Disable clear transfer count for remaining data*/
            }
            else
            {
                /* If a send buffer was provided, the word comes from there. Otherwise we just send
                 * a zero (initialized above).
                 */
                if (dspiState->sendBuffer)
                {
                    wordToSend = *(dspiState->sendBuffer);
                    ++dspiState->sendBuffer; /* increment to next data byte */
                    wordToSend |= (unsigned)(*(dspiState->sendBuffer)) << 8U;
                    ++dspiState->sendBuffer; /* increment to next data byte */
                }
                dspi_hal_write_data_master_mode(instance, &command, wordToSend);
                dspiState->remainingSendByteCount -= 2; /* decrement remainingSendByteCount by 2 */
            }

            /* try to clear TFFF by writing a one to it; it will not clear if TX FIFO not full*/
            dspi_hal_clear_status_flag(instance, kDspiTxFifoFillRequest);

            /* Store the DSPI state struct volatile member variables into temporary
             * non-volatile variables to allow for MISRA compliant calculations
             */
            remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
            remainingSendByteCount = dspiState->remainingSendByteCount;

            /* exit loop if send count is zero */
            if (dspiState->remainingSendByteCount == 0)
            {
                break;
            }
        } /* End of TX FIFO fill while loop */
    }
    /* Optimized for bits/frame less than or equal to one byte. */
    else
    {
        /* Fill the fifo until it is full or until the send word count is 0 or until the difference
         * between the remainingReceiveByteCount and remainingSendByteCount equals the FIFO depth.
         * The reason for checking the difference is to ensure we only send as much as the
         * RX FIFO can receive.
         */
        /* Store the DSPI state struct volatile member variables into temporary
         * non-volatile variables to allow for MISRA compliant calculations
         */
        remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
        remainingSendByteCount = dspiState->remainingSendByteCount;

        while((dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest) == 1) &&
              ((remainingReceiveByteCount - remainingSendByteCount) <
                FSL_FEATURE_SPI_FIFO_SIZEn(instance)))
        {
            /* On the last word to be sent, set the end of queue flag in the data command struct
             * and ensure that the CONT bit in the PUSHR is also cleared even if it was cleared to
             * begin with. If CONT is set it means continuous chip select operation and to ensure
             * the chip select is de-asserted, this bit must be cleared on the last data word.
             */
            if ((dspiState->remainingSendByteCount - 1) == 0)
            {
                command.isEndOfQueue = 1;
                command.isChipSelectContinuous = 0;
            }

            /* If this is the first word to be transmitted, clear firstWord flag after transmit*/
            if (firstWordFlag == 1)
            {
                /* If a send buffer was provided, the word comes from there. Otherwise we just send
                 * a zero (initialized above).
                 */
                if (dspiState->sendBuffer)
                {
                    wordToSend = *(dspiState->sendBuffer);
                    ++dspiState->sendBuffer; /* increment to next data byte */
                }
                dspi_hal_write_data_master_mode(instance, &command, wordToSend);
                --dspiState->remainingSendByteCount; /* decrement remainingSendByteCount */

                firstWordFlag = 0; /* clear firstWordFlag */
                command.clearTransferCount = 0; /* Disable clear transfer count for remaining data*/
            }
            else
            {
                /* If a send buffer was provided, the word comes from there. Otherwise we just send
                 * a zero (initialized above).
                 */
                if (dspiState->sendBuffer)
                {
                    wordToSend = *(dspiState->sendBuffer);
                    ++dspiState->sendBuffer; /* increment to next data word*/
                }
                dspi_hal_write_data_master_mode(instance, &command, wordToSend);
                --dspiState->remainingSendByteCount; /* decrement remainingSendByteCount*/
            }

            /* try to clear TFFF by writing a one to it; it will not clear if TX FIFO not full*/
            dspi_hal_clear_status_flag(instance, kDspiTxFifoFillRequest);

            /* Store the DSPI state struct volatile member variables into temporary
             * non-volatile variables to allow for MISRA compliant calculations
             */
            remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
            remainingSendByteCount = dspiState->remainingSendByteCount;

            /* exit loop if send count is zero */
            if (dspiState->remainingSendByteCount == 0)
            {
                break;
            }
        } /* End of TX FIFO fill while loop */
    }

    /* RX FIFO Drain request: RFDF_RE to enable RFDF interrupt
     * Since SPI is a synchronous interface, we only need to enable the RX interrupt.
     * The IRQ handler will get the status of RX and TX interrupt flags.
     */
    dspi_hal_configure_interrupt(instance, kDspiRxFifoDrainRequest, true);

    return kStatus_DSPI_Success;
}

/*!
 * @brief Finish up a transfer.
 * Cleans up after a transfer is complete. Interrupts are disabled, and the DSPI module
 * is disabled. This is not a public API as it is called from other driver functions.
 */
static void dspi_master_complete_transfer(dspi_master_state_t * dspiState)
{
    uint32_t instance = dspiState->instance;

    /* The transfer is complete.*/
    dspiState->isTransferInProgress = false;

    /* Disable interrupts.*/
    /* RX FIFO Drain request: RFDF_RE to disable RFDF interrupt */
    dspi_hal_configure_interrupt(instance, kDspiRxFifoDrainRequest, false);

    /* Transfer is complete, so disable the module. It gets re-enabled during the
     * start transfer function
     */
    dspi_hal_disable(instance);
}

/*!
 * @brief Interrupt handler for DSPI master mode.
 * This handler uses the buffers stored in the dspi_master_state_t structs to transfer data.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void dspi_master_irq_handler(void * state)
{
    /* instantiate local variable of type dspi_master_state_t and equate it to the
     * passed in pointer to state
     */
    dspi_master_state_t * dspiState = (dspi_master_state_t *)state;

    /* create an instance of the data command struct*/
    dspi_command_config_t command;

    uint32_t instance = dspiState->instance;

    /* Declare variables for storing volatile data later in the code */
    uint32_t remainingReceiveByteCount, remainingSendByteCount;

    /* Check read buffer.*/
    uint16_t wordReceived; /* Maximum supported data bit length in master mode is 16-bits */

    /* If bits/frame is greater than one byte */
    if (dspiState->bitsPerFrame > 8)
    {
        while (dspi_hal_get_status_flag(instance, kDspiRxFifoDrainRequest))
        {
            wordReceived = dspi_hal_read_data(instance);
            /* clear the rx fifo drain request, needed for non-DMA applications as this flag
             * will remain set even if the rx fifo is empty. By manually clearing this flag, it
             * either remain clear if no more data is in the fifo, or it will set if there is
             * more data in the fifo.
             */
            dspi_hal_clear_status_flag(instance, kDspiRxFifoDrainRequest);

            if (dspiState->receiveBuffer)
            {
                *dspiState->receiveBuffer = wordReceived; /* Write first data byte */
                ++dspiState->receiveBuffer; /* increment to next data byte */
                *dspiState->receiveBuffer = wordReceived >> 8; /* Write second data byte */
                ++dspiState->receiveBuffer; /* increment to next data byte */
            }
            dspiState->remainingReceiveByteCount -= 2;

            if (dspiState->remainingReceiveByteCount == 0)
            {
                break;
            }
        } /* End of RX FIFO drain while loop */
    }
    /* Optimized for bits/frame less than or equal to one byte. */
    else
    {
        while (dspi_hal_get_status_flag(instance, kDspiRxFifoDrainRequest))
        {
            wordReceived = dspi_hal_read_data(instance);
            /* clear the rx fifo drain request, needed for non-DMA applications as this flag
             * will remain set even if the rx fifo is empty. By manually clearing this flag, it
             * either remain clear if no more data is in the fifo, or it will set if there is
             * more data in the fifo.
             */
            dspi_hal_clear_status_flag(instance, kDspiRxFifoDrainRequest);

            if (dspiState->receiveBuffer)
            {
                *dspiState->receiveBuffer = wordReceived;
                ++dspiState->receiveBuffer;
            }
            --dspiState->remainingReceiveByteCount;

            if (dspiState->remainingReceiveByteCount == 0)
            {
                break;
            }
        } /* End of RX FIFO drain while loop */
    }

    /* Check write buffer. We always have to send a word in order to keep the transfer
     * moving. So if the caller didn't provide a send buffer, we just send a zero.
     */
    uint16_t wordToSend = 0;
    if (dspiState->remainingSendByteCount)
    {
        /* Before sending the data, we first need to initialize the data command struct
         * Configure the data command attributes for the desired PCS, CTAR, and continuous PCS
         * which are derived from the run-time state struct
         */
        command.whichPcs = dspiState->whichPcs;
        command.whichCtar = dspiState->whichCtar;
        command.isChipSelectContinuous = dspiState->isChipSelectContinuous;
        /* Disable the clear transfer count; clear end of queue member and set when it is
         * determined that the last data word is to be sent
         */
        command.clearTransferCount = 0;
        command.isEndOfQueue = 0;

        /* If bits/frame is greater than one byte */
        if (dspiState->bitsPerFrame > 8)
        {
            /* Fill the fifo until it is full or until the send word count is 0 or until the
             * difference between the remainingReceiveByteCount and remainingSendByteCount equals
             * the FIFO depth.
             * The reason for checking the difference is to ensure we only send as much as the
             * RX FIFO can receive.
             * For this case wher bitsPerFrame > 8, each entry in the FIFO contains 2 bytes of the
             * send data, hence the difference between the remainingReceiveByteCount and
             * remainingSendByteCount must be divided by 2 to convert this difference into a
             * 16-bit (2 byte) value.
             */
            /* Store the DSPI state struct volatile member variables into temporary
             * non-volatile variables to allow for MISRA compliant calculations
             */
            remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
            remainingSendByteCount = dspiState->remainingSendByteCount;

          while((dspiState->remainingSendByteCount != 0) &&
                  (dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest)==1) &&
                  ((remainingReceiveByteCount - remainingSendByteCount)/2 <
                FSL_FEATURE_SPI_FIFO_SIZEn(instance)))
          {
                /* On the last word to be sent, set the end of queue flag in the data command struct
                 * and ensure that the CONT bit in the PUSHR is also cleared even if it was cleared
                 * to begin with. If CONT is set it means continuous chip select operation and to
                 * ensure the chip select is de-asserted, this bit must be cleared on the last data
                 * word.
                 */
                if ((dspiState->remainingSendByteCount - 2) == 0)
                {
                    command.isEndOfQueue = 1;
                    command.isChipSelectContinuous = 0;
                }

                /* If a send buffer was provided, the word comes from there. Otherwise we just send
                 * a zero (initialized above).
                 */
                if (dspiState->sendBuffer)
                {
                    wordToSend = *(dspiState->sendBuffer);
                    ++dspiState->sendBuffer; /* increment to next data byte */
                    wordToSend |= (unsigned)(*(dspiState->sendBuffer)) << 8U;
                    ++dspiState->sendBuffer; /* increment to next data byte */
                }
                dspi_hal_write_data_master_mode(instance, &command, wordToSend);
                dspiState->remainingSendByteCount -= 2; /* decrement remainingSendByteCount by 2*/

                /* try to clear TFFF by writing a one to it; it will not clear if TX FIFO not full*/
                dspi_hal_clear_status_flag(instance, kDspiTxFifoFillRequest);

                /* Store the DSPI state struct volatile member variables into temporary
                 * non-volatile variables to allow for MISRA compliant calculations
                 */
                remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
                remainingSendByteCount = dspiState->remainingSendByteCount;

            } /* End of TX FIFO fill while loop */
        }
        /* Optimized for bits/frame less than or equal to one byte. */
        else
        {
            /* Fill the fifo until it is full or until the send word count is 0 or until the
             * difference between the remainingReceiveByteCount and remainingSendByteCount equals
             * the FIFO depth.
             * The reason for checking the difference is to ensure we only send as much as the
             * RX FIFO can receive.
             */
            /* Store the DSPI state struct volatile member variables into temporary
             * non-volatile variables to allow for MISRA compliant calculations
             */
            remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
            remainingSendByteCount = dspiState->remainingSendByteCount;

            while((dspiState->remainingSendByteCount != 0) &&
                  (dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest)==1) &&
                  ((remainingReceiveByteCount - remainingSendByteCount) <
                FSL_FEATURE_SPI_FIFO_SIZEn(instance)))
            {
                /* On the last word to be sent, set the end of queue flag in the data command struct
                 * and ensure that the CONT bit in the PUSHR is also cleared even if it was cleared
                 * to begin with. If CONT is set it means continuous chip select operation and to
                 * ensure the chip select is de-asserted, this bit must be cleared on the last data
                 * word.
                 */
                if ((dspiState->remainingSendByteCount - 1) == 0)
                {
                    command.isEndOfQueue = 1;
                    command.isChipSelectContinuous = 0;
                }

                /* If a send buffer was provided, the word comes from there. Otherwise we just send
                 * a zero (initialized above).
                 */
                if (dspiState->sendBuffer)
                {
                    wordToSend = *(dspiState->sendBuffer);
                    ++dspiState->sendBuffer; /* increment to next data word*/
                }
                dspi_hal_write_data_master_mode(instance, &command, wordToSend);
                --dspiState->remainingSendByteCount; /* decrement remainingSendByteCount*/

                /* try to clear TFFF by writing a one to it; it will not clear if TX FIFO not full*/
                dspi_hal_clear_status_flag(instance, kDspiTxFifoFillRequest);

                /* Store the DSPI state struct volatile member variables into temporary
                 * non-volatile variables to allow for MISRA compliant calculations
                 */
                remainingReceiveByteCount = dspiState->remainingReceiveByteCount;
                remainingSendByteCount = dspiState->remainingSendByteCount;

            } /* End of TX FIFO fill while loop */
        }
    }

    /* Check if we're done with this transfer.*/
    if ((dspiState->remainingSendByteCount == 0) && (dspiState->remainingReceiveByteCount == 0))
    {
        /* Complete the transfer. This disables the interrupts, so we don't wind up in
         * the ISR again.
         */
        dspi_master_complete_transfer(dspiState);

        /* Signal the synchronous completion object if the transfer wasn't async.*/
        if (!dspiState->isTransferAsync)
        {
            sync_signal(&dspiState->irqSync);
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

