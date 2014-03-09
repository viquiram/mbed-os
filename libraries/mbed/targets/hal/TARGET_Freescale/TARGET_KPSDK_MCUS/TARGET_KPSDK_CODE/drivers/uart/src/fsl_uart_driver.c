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
#include "fsl_uart_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_uart_irq.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern IRQn_Type uart_irq_ids[UART_INSTANCE_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static uart_status_t uart_start_send_data(uart_state_t * uartState, const uint8_t * sendBuffer,
                                          uint32_t txByteCount);
static void uart_complete_send_data(uart_state_t * uartState);
static uart_status_t uart_start_receive_data(uart_state_t * uartState, uint8_t * rxBuffer,
                                             uint32_t requestedByteCount);
static void uart_complete_receive_data(uart_state_t * uartState);
/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_init
 * Description   : This function initializes a UART instance for operation.
 * This function will initialize the run-time state structure to keep track of the on-going
 * transfers, ungate the clock to the UART module, initialize the module
 * to user defined settings and default settings, configure the IRQ state structure and enable
 * the module-level interrupt to the core, and enable the UART module transmitter and receiver.
 * The following is an example of how to set up the uart_state_t and the
 * uart_user_config_t parameters and how to call the uart_init function by passing
 * in these parameters:
 *    uart_user_config_t uartConfig;
 *    uartConfig.baudRate = 9600;
 *    uartConfig.bitCountPerChar = kUart8BitsPerChar;
 *    uartConfig.parityMode = kUartParityDisabled;
 *    uartConfig.stopBitCount = kUartOneStopBit;
 *    uart_state_t uartState;
 *    uart_init(uartInstance, &uartState, &uartConfig);
 *
 *END**************************************************************************/
uart_status_t uart_init(uint32_t uartInstance, uart_state_t * uartState,
                        const uart_user_config_t * uartUserConfig)
{
    uint32_t uartSourceClock;
    uart_status_t errorCode = kStatus_UART_Success;

    uart_config_t uartConfig;  /* UART config in the HAL */

    if (uartInstance >= UART_INSTANCE_COUNT)
    {
        return kStatus_UART_InvalidInstanceNumber;
    }

    /* Ungate UART module clock */
    clock_manager_set_gate(kClockModuleUART, uartInstance, true);

    /* Clear the state struct for this instance. */
    memset(uartState, 0, sizeof(*uartState));

    uartState->instance = uartInstance;

    /* Init the interrupt sync object. */
    sync_create(&uartState->txIrqSync, 0);
    sync_create(&uartState->rxIrqSync, 0);

    /* Initialize the parameters of the UART config structure with desired data */
    uartConfig.baudRate = uartUserConfig->baudRate;
    uartConfig.bitCountPerChar = uartUserConfig->bitCountPerChar;
    uartConfig.parityMode = uartUserConfig->parityMode;
    uartConfig.rxDataInvert = 0;
    uartConfig.stopBitCount = uartUserConfig->stopBitCount;
    uartConfig.txDataInvert = 0;

    /* UART clock source is either system clock or bus clock depending on the instance */
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    /* For KL25Z4 sub-family, UART0 has two optional clock sources */
    if (uartInstance == 0)
    {
        clock_manager_get_frequency_by_source(kClockUart0Src, &uartSourceClock);
    }
#else
    if ((uartInstance == 0)||(uartInstance == 1))
    {
        clock_manager_get_frequency(kSystemClock, &uartSourceClock);
    }
#endif
    else
    {
        clock_manager_get_frequency(kBusClock, &uartSourceClock);
    }

    uartConfig.uartSourceClockInHz = uartSourceClock;

#if FSL_FEATURE_UART_HAS_FIFO
    uint8_t fifoSize;
    /* Obtain raw TX FIFO size bit setting */
    fifoSize = uart_hal_get_tx_fifo_size(uartInstance);
    /* Now calculate the number of data words per given FIFO size */
    uartState->txFifoEntryCount = (fifoSize == 0 ? 1 : 0x1 << (fifoSize + 1));

    /* Configure the TX FIFO watermark to be 1/2 of the total entry or 0 if entry count = 1
     * A watermark setting of 0 for TX FIFO entry count of 1 means that TDRE will only interrupt
     * when the TX buffer (the one entry in the TX FIFO) is empty. Otherwise, if we set the
     * watermark to 1, the TDRE will always be set regardless if the TX buffer was empty or not
     * as the spec says TDRE will set when the FIFO is at or below the configured watermark.
     */
    if (uartState->txFifoEntryCount > 1)
    {
        uart_hal_set_tx_fifo_watermark(uartInstance, uartState->txFifoEntryCount/2);
    }
    else
    {
        uart_hal_set_tx_fifo_watermark(uartInstance, 0);
    }

    /* Configure the RX FIFO watermark to be 1 */
    /* Note about RX FIFO support: There is only one RX data full interrupt that is
     * associated with the RX FIFO Watermark.  The watermark cannot be dynamically changed.
     * This means if the remainingReceiveByteCount is less than the programmed watermark
     * the interrupt will never occur. If we try to change the watermark,
     * this will involve shutting down the receiver first - which is not a desirable operation
     * when the UART is actively receving data.  Hence, the best solution is to set the RX
     * FIFO watermark to 1.
     */
    uart_hal_set_rx_fifo_watermark(uartInstance, 1);

    /* Enable and flush the fifos prior to enabling the TX/RX */
    uart_hal_enable_tx_fifo(uartInstance);
    uart_hal_enable_rx_fifo(uartInstance);
    uart_hal_flush_tx_fifo(uartInstance);
    uart_hal_flush_rx_fifo(uartInstance);
#else
    /* For modules that do not support a FIFO, they have a data buffer that essentially
     * acts likes a one-entry FIFO, thus to make the code cleaner, we'll
     * equate txFifoEntryCount to 1.  Also note that TDRE flag will set only when the tx
     * buffer is empty.
     */
    uartState->txFifoEntryCount = 1;
#endif

    /* Initialize the UART instance */
    errorCode = uart_hal_init(uartInstance, &uartConfig);
    if (errorCode != kStatus_UART_Success)
    {
        return errorCode;
    }

    /* Configure IRQ state structure, so IRQ handler can point to the correct state structure. */
    uart_set_irq_state(uartInstance, uartState);

    /* Enable the interrupt */
    interrupt_enable(uart_irq_ids[uartInstance]);

    return kStatus_UART_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_shutdown
 * Description   : This function shuts down the UART by disabling interrupts and the
 *                 transmitter/receiver.
 * This function disables the UART interrupts, disables the transmitter and receiver, and
 * flushes the FIFOs (for modules that support FIFOs).
 *
 *END**************************************************************************/
void uart_shutdown(uart_state_t * uartState)
{
    /* Get uartInstance from UART state struct */
    uint32_t uartInstance = uartState->instance;

    /* In case there is still data in the TX FIFO or shift register that is being transmitted
     * wait till transmit is complete
     */
#if FSL_FEATURE_UART_HAS_FIFO
    /* Wait until there all of the data has been drained from the TX FIFO */
    while(uart_hal_get_tx_dataword_count_in_fifo(uartInstance) != 0) { }
#endif
    /* Wait until the data is completely shifted out of shift register */
    while(!(uart_hal_is_transmission_complete(uartInstance))) { }

    /* Disable the interrupt */
    interrupt_disable(uart_irq_ids[uartInstance]);

    /* Disable TX and RX */
    uart_hal_disable_transmitter(uartInstance);
    uart_hal_disable_receiver(uartInstance);

#if FSL_FEATURE_UART_HAS_FIFO
    /* Disable the FIFOs; should be done after disabling the TX/RX */
    uart_hal_disable_tx_fifo(uartInstance);
    uart_hal_disable_rx_fifo(uartInstance);
    uart_hal_flush_tx_fifo(uartInstance);
    uart_hal_flush_rx_fifo(uartInstance);
#endif

    /* Cleared state pointer stored in IRQ handler.*/
    uart_set_irq_state(uartInstance, NULL);

    /* Gate UART module clock */
    clock_manager_set_gate(kClockModuleUART, uartInstance, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_send_data
 * Description   : This function sends (transmits) data out through the UART module using a
 *                 blocking method.
 * A blocking (also known as synchronous) function means that the function does not return until
 * the transmit is complete. This blocking function is used to send data through the UART port.
 *
 *END**************************************************************************/
uart_status_t uart_send_data(uart_state_t * uartState, const uint8_t * sendBuffer,
                             uint32_t txByteCount, uint32_t timeout)
{
    assert(sendBuffer);

    uartState->isTransmitAsync = false;

    /* Start the transmission process */
    if (uart_start_send_data(uartState, sendBuffer, txByteCount) == kStatus_UART_TxBusy)
    {
        return kStatus_UART_TxBusy;
    }

    /* Wait until the transmit is complete. */
    uart_status_t error = kStatus_UART_Success;
    fsl_rtos_status syncStatus;

    do
    {
        syncStatus = sync_wait(&uartState->txIrqSync, timeout);
    }while(syncStatus == kIdle);

    if (syncStatus != kSuccess)
    {
        error = kStatus_UART_Timeout;
    }

#if FSL_FEATURE_UART_HAS_FIFO
    /* Wait until the TX FIFO is empty before returning. However, do not wait until the TX
     * is complete (when all data is shifted from the TX shift register). The reason is, this may
     * prevent us from exiting in time to queue up more data into the TX FIFO or TX Buffer for
     * another transmission.
     */
    while(uart_hal_get_tx_dataword_count_in_fifo(uartState->instance) != 0) { }
#endif

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_send_data_async
 * Description   : This function sends (transmits) data through the UART module using a
 *                 non-blocking method.
 * A non-blocking (also known as synchronous) function means that the function returns
 * immediately after initiating the transmit function. The application has to get the
 * transmit status to see when the transmit is complete. In other words, after calling non-blocking
 * (asynchronous) send function, the application must get the transmit status to check if transmit
 * is completed or not.
 * The asynchronous method of transmitting and receiving allows the UART to perform a full duplex
 * operation (simultaneously transmit and receive).
 *
 *END**************************************************************************/
uart_status_t uart_send_data_async(uart_state_t * uartState, const uint8_t * sendBuffer,
                                   uint32_t txByteCount)
{
    assert(sendBuffer);

    uartState->isTransmitAsync = true;

    /* Start the transmission process*/
    if (uart_start_send_data(uartState, sendBuffer, txByteCount) == kStatus_UART_TxBusy)
    {
        return kStatus_UART_TxBusy;
    }

    return kStatus_UART_Success;
}

/*!
 * @brief Initiate (start) a transmit by beginning the process of sending data and enabling
 *        the interrupt. This is not a public API as it is called from other driver functions.
 */
 static uart_status_t uart_start_send_data(uart_state_t * uartState, const uint8_t * sendBuffer,
                                           uint32_t txByteCount)
{
    uint32_t uartInstance = uartState->instance;

    /* Check that we're not busy already transmitting data from a previous function call. */
    if (uartState->isTransmitInProgress)
    {
        return kStatus_UART_TxBusy;
    }

    /* Initialize the module driver state struct  */
    uartState->sendBuffer = sendBuffer;
    uartState->remainingSendByteCount = txByteCount;
    uartState->transmittedByteCount = 0;

    /* Start the transmission by writing the first char. */
    uartState->isTransmitInProgress = true;

    /* Fill the TX FIFO or TX data buffer. In the event that there still might be data in the
     * TX FIFO, first ascertain the nubmer of empty spaces and then fill those up.
     */
    uint8_t emptyEntryCountInFifo;
#if FSL_FEATURE_UART_HAS_FIFO
    emptyEntryCountInFifo = uartState->txFifoEntryCount -
                            uart_hal_get_tx_dataword_count_in_fifo(uartInstance);
#else
    /* For modules that don't have a FIFO, there is no FIFO data count register */
    emptyEntryCountInFifo = uartState->txFifoEntryCount;
    /* Make sure the transmit data register is empty and ready for data */
    while(!uart_hal_is_transmit_data_register_empty(uartInstance)) { }
#endif

    /* Fill up FIFO, if only a 1-entry FIFO, then just fill the data buffer */
    while(emptyEntryCountInFifo--)
    {
        uart_hal_putchar(uartInstance, *(uartState->sendBuffer)); /* put data into FIFO */
        ++uartState->sendBuffer; /* Increment the sendBuffer pointer */
        --uartState->remainingSendByteCount;  /* Decrement the byte count */
        ++uartState->transmittedByteCount;  /* Increment the transmitted byte count */
        /* If there are no more bytes in the buffer to send, then complete transmit. No need
         * to spend time enabling the interrupt and going to the ISR.
         */
        if (uartState->remainingSendByteCount == 0)
        {
            uart_complete_send_data(uartState);
            /* Signal the synchronous completion object if the transmit wasn't async.*/
            if (!uartState->isTransmitAsync)
            {
                sync_signal(&uartState->txIrqSync);
            }
            return kStatus_UART_Success;
        }
    }

    /* Enable the transmitter data register empty interrupt. The TDRE flag will set whenever
     * the TX buffer is emptied into the TX shift register (for non-FIFO IPs) or when the
     * data in the TX FIFO is at or below the programmed watermark (for FIFO-supported IPs).
     */
    uart_hal_enable_tx_data_register_empty_interrupt(uartInstance);

    return kStatus_UART_Success;
}

/*!
 * @brief Finish up a transmit by completing the process of sending data and disabling the
 *        interrupt. This is not a public API as it is called from other driver functions.
 */
static void uart_complete_send_data(uart_state_t * uartState)
{
    /* Get the current instance number from the UART state struct */
    uint32_t uartInstance = uartState->instance;

    /* Disable the transmitter data register empty interrupt */
    uart_hal_disable_tx_data_register_empty_interrupt(uartInstance);

    /* Update the information of the module driver state */
    uartState->isTransmitInProgress = false;  /* transmission is complete*/
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_get_transmit_status
 * Description   : This function returns whether the previous UART transmit has finished.
 * When performing an async transmit, the user can call this function to ascertain the state of the
 * current transmission: in progress (or busy) or complete (success). In addition, if the
 * transmission is still in progress, the user can obtain the number of words that have been
 * currently transferred.
 *
 *END**************************************************************************/
uart_status_t uart_get_transmit_status(uart_state_t * uartState, uint32_t * bytesTransmitted)
{
    /* Fill in the bytes transferred. This may return that all bytes were transmitted,
     * however, for IPs with FIFO support, there still may be data in the TX FIFO still
     * in the process of being transmitted.
     */
    if (bytesTransmitted)
    {
        *bytesTransmitted = uartState->transmittedByteCount;
    }

    /*
     * Return kStatus_UART_TxBusy or kStatus_UART_Success depending on whether or not
     * the UART has a FIFO. If it does have a FIFO, we'll need to wait until the FIFO is
     * completely drained before indicating success in addition to isTransmitInProgress = 0.
     * If there is no FIFO, then we need to only worry about isTransmitInProgress
     */
#if FSL_FEATURE_UART_HAS_FIFO
    if ((uartState->isTransmitInProgress == 0) &&
        (uart_hal_get_tx_dataword_count_in_fifo(uartState->instance) == 0))
    {
        return kStatus_UART_Success; /* No more data to send, FIFO is empty */
    }
    else
    {
        return kStatus_UART_TxBusy; /* Either more data to send, or FIFO has data */
    }
#else
    return (uartState->isTransmitInProgress ? kStatus_UART_TxBusy : kStatus_UART_Success);
#endif
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_abort_sending_data
 * Description   : This function terminates an asynchronous UART transmission early.
 * During an async UART tranmission, the user has the option to terminate the transmission early
 * if the transmission is still in progress.
 *
 *END**************************************************************************/
uart_status_t uart_abort_sending_data(uart_state_t * uartState)
{

    /* Check if a transfer is running. */
    if (!uartState->isTransmitInProgress)
    {
        return kStatus_UART_NoTransmitInProgress;
    }

    /* Stop the running transfer. */
    uart_complete_send_data(uartState);

    return kStatus_UART_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_receive_data
 * Description   : This function gets (receives) data from the UART module using a blocking method.
 * A blocking (also known as synchronous) function means that the function does not return until
 * the receive is complete. This blocking function is used to send data through the UART port.
 *
 *END**************************************************************************/
uart_status_t uart_receive_data(uart_state_t * uartState, uint8_t * rxBuffer,
                           uint32_t requestedByteCount, uint32_t timeout)
{
    assert(rxBuffer);

    uartState->isReceiveAsync = false;

    if (uart_start_receive_data(uartState, rxBuffer, requestedByteCount) == kStatus_UART_RxBusy)
    {
        return kStatus_UART_RxBusy;
    }

    /* Wait until all the data is received or for timeout.*/
    uart_status_t error = kStatus_UART_Success;
    fsl_rtos_status syncStatus;

    do
    {
        syncStatus = sync_wait(&uartState->rxIrqSync, timeout);
    }while(syncStatus == kIdle);

    if (syncStatus != kSuccess)
    {
        error = kStatus_UART_Timeout;
    }

    return error;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_receive_data_async
 * Description   : This function gets (receives) data from the UART module using a non-blocking
 *                 method.
 * A non-blocking (also known as synchronous) function means that the function returns
 * immediately after initiating the receive function. The application has to get the
 * receive status to see when the receive is complete. In other words, after calling non-blocking
 * (asynchronous) get function, the application must get the receive status to check if receive
 * is completed or not.
 * The asynchronous method of transmitting and receiving allows the UART to perform a full duplex
 * operation (simultaneously transmit and receive).
 *
 *END**************************************************************************/
uart_status_t uart_receive_data_async(uart_state_t * uartState, uint8_t * rxBuffer,
                                 uint32_t requestedByteCount)
{
    assert(rxBuffer);

    uartState->isReceiveAsync = true;

    if (uart_start_receive_data(uartState, rxBuffer, requestedByteCount) == kStatus_UART_RxBusy)
    {
        return kStatus_UART_RxBusy;
    }

    return kStatus_UART_Success;
}


/*!
 * @brief Initiate (start) a receive by beginning the process of receiving data and enabling
 *        the interrupt. This is not a public API as it is called from other driver functions.
 */
static uart_status_t uart_start_receive_data(uart_state_t * uartState, uint8_t * rxBuffer,
                                 uint32_t requestedByteCount)
{
    /* Check that we're not busy already receiving data from a previous function call. */
    if (uartState->isReceiveInProgress)
    {
        return kStatus_UART_RxBusy;
    }

    /* Get the current instance number from the uart state struct */
    uint32_t uartInstance = uartState->instance;

    /* Initialize the module driver state struct to indicate transfer in progress
     * and with the buffer and byte count data
     */
    uartState->isReceiveInProgress = true;
    uartState->receiveBuffer = (uint8_t *)rxBuffer;
    uartState->remainingReceiveByteCount = requestedByteCount;
    uartState->receivedByteCount = 0;

    /* enable the receive data full interrupt */
    uart_hal_enable_rx_data_register_full_interrupt(uartInstance);

    return kStatus_UART_Success;
}

/*!
 * @brief Finish up a receive by completing the process of receiving data and disabling the
 *        interrupt. This is not a public API as it is called from other driver functions.
 */
static void uart_complete_receive_data(uart_state_t * uartState)
{
    /* Get the current instance number from the uart state struct */
    uint32_t uartInstance = uartState->instance;

    /* Disable receive data full interrupt */
    uart_hal_disable_rx_data_register_full_interrupt(uartInstance);

    /* Update the information of the module driver state */
    uartState->isReceiveInProgress = false;  /* receive is complete */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_get_receive_status
 * Description   : This function returns whether the previous UART receive is complete.
 * When performing an async receive, the user can call this function to ascertain the state of the
 * current receive progress: in progress (or busy) or complete (success). In addition, if the
 * receive is still in progress, the user can obtain the number of words that have been
 * currently received.
 *
 *END**************************************************************************/
uart_status_t uart_get_receive_status(uart_state_t * uartState, uint32_t * bytesReceived)
{
    /* Fill in the bytes transferred. */
    if (bytesReceived)
    {
        *bytesReceived = uartState->receivedByteCount;
    }

    return (uartState->isReceiveInProgress ? kStatus_UART_RxBusy : kStatus_UART_Success);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : uart_abort_receiving_data
 * Description   : This function shuts down the UART by disabling interrupts and the
 *                 transmitter/receiver.
 * This function disables the UART interrupts, disables the transmitter and receiver, and
 * flushes the FIFOs (for modules that support FIFOs).
 *
 *END**************************************************************************/
uart_status_t uart_abort_receiving_data(uart_state_t * uartState)
{
    /* Check if a transfer is running. */
    if (!uartState->isReceiveInProgress)
    {
        return kStatus_UART_NoReceiveInProgress;
    }

    /* Stop the running transfer. */
    uart_complete_receive_data(uartState);

    return kStatus_UART_Success;
}

/*!
 * @brief Interrupt handler for UART.
 * This handler uses the buffers stored in the uart_state_t structs to transfer data.
 * This is not a public API as it is called whenever an interrupt occurs.
 */
void uart_irq_handler(uart_state_t * uartStateIrq)
{
    /* Instantiate local variable of type uart_state_t and equate it to the
     * pointer to uartStateIrq
     */
    uart_state_t * uartState = uartStateIrq;

    /* Get uartInstance from uart state struct */
    uint32_t uartInstance = uartState->instance;

    /* Exit the ISR if no transfer is happening for this instance. */
    if ((!uartState->isTransmitInProgress)&&(!uartState->isReceiveInProgress))
    {
        return;
    }

    /* Check to see if the interrupt is due to receive data full
     * first see if the interrupt is enabled
     */
    if(uart_hal_is_receive_data_full_interrupt_enabled(uartInstance))
    {
        if(uart_hal_is_receive_data_register_full(uartInstance))
        {
#if FSL_FEATURE_UART_HAS_FIFO
            /* Read from RX FIFO while the RX count indicates there's data in the FIFO.
             * Even though the watermark is set to 1, it might be possible to have more than one
             * byte in the FIFO, so lets make sure to drain it.
             */
            while(uart_hal_get_rx_dataword_count_in_fifo(uartInstance))
            {
                /* Check to see if this was the last byte received */
                if ((uartState->remainingReceiveByteCount == 0))
                {
                    break;
                }
#endif
            /* Get data and put in receive buffer */
            uart_hal_getchar(uartInstance, uartState->receiveBuffer);
            ++uartState->receiveBuffer;  /* Increment the receiveBuffer pointer */
            --uartState->remainingReceiveByteCount;  /* Decrement the byte count  */
            ++uartState->receivedByteCount;  /* increment the received byte count */
#if FSL_FEATURE_UART_HAS_FIFO
            }
#endif
            /* Check to see if this was the last byte received */
            if ((uartState->remainingReceiveByteCount == 0))
            {
                /* Complete the transfer. This disables the interrupts, so we don't wind up in*/
                /* the ISR again. */
                uart_complete_receive_data(uartState);

                /* Signal the synchronous completion object if the receive wasn't async.*/
                if (!uartState->isReceiveAsync)
                {
                    sync_signal(&uartState->rxIrqSync);
                }
            }
        }
    }

    /* Check to see if the interrupt is due to transmit data register empty
     * first see if the interrupt is enabled.
     */
    if (uart_hal_is_tx_data_register_empty_interrupt_enabled(uartInstance))
    {
        if(uart_hal_is_transmit_data_register_empty(uartInstance))
        {
            /* Check to see if there are any more bytes to send */
            if (uartState->remainingSendByteCount)
            {
                uint8_t emptyEntryCountInFifo;
#if FSL_FEATURE_UART_HAS_FIFO
                emptyEntryCountInFifo = uartState->txFifoEntryCount -
                                        uart_hal_get_tx_dataword_count_in_fifo(uartInstance);
#else
                /* For modules that don't have a FIFO, there is no FIFO data count register */
                emptyEntryCountInFifo = uartState->txFifoEntryCount;
#endif

                /* Fill up FIFO, if only a 1-entry FIFO, then just fill the data buffer */
                while(emptyEntryCountInFifo--)
                {
                    uart_hal_putchar(uartInstance, *(uartState->sendBuffer));/* Transmit the data */
                    ++uartState->sendBuffer; /* Increment the sendBuffer pointer */
                    --uartState->remainingSendByteCount;  /* Decrement the byte count */
                    ++uartState->transmittedByteCount;  /* increment the transmitted byte count */

                    /* If there are no more bytes in the buffer to send, complete the transmit
                     * process and break out of the while loop. We should not re-enter the ISR again
                     * as all of the data has been put into the FIFO (or the TX data buffer).
                     */
                    if (!uartState->remainingSendByteCount)
                    {
                        uart_complete_send_data(uartState);
                        /* Signal the synchronous completion object if the transmit wasn't async.*/
                        if (!uartState->isTransmitAsync)
                        {
                            sync_signal(&uartState->txIrqSync);
                        }
                        break;
                    }
                }
            }
            /* In case the ISR is entered again, but there is no more data to send, go ahead and
             * complete the transmit process.
             */
            else
            {
                /* We're done with this transfer.
                * Complete the transfer. This disables the interrupts, so we don't wind up in
                * the ISR again.
                */
                uart_complete_send_data(uartState);

                /* Signal the synchronous completion object if the transmit wasn't async.*/
                if (!uartState->isTransmitAsync)
                {
                    sync_signal(&uartState->txIrqSync);
                }
            }
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

