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

#ifndef __FSL_UART_DRIVER_H__
#define __FSL_UART_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_os_abstraction.h"
#include "fsl_uart_hal.h"

/*!
 * @addtogroup uart_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Runtime state of the UART driver.
 *
 * This struct holds data that are used by the UART peripheral driver to
 * communicate between the transfer function and the interrupt handler. The
 * interrupt handler also uses this information to keep track of its progress.
 * The user is only responsible to pass in the memory for this run-time state structure where the
 * UART driver will take care of filling out the members.
 */
typedef struct UartState {
    uint32_t instance;                  /*!< UART module instance number. */
    bool isTransmitInProgress;          /*!< True if there is an active transmit. */
    bool isTransmitAsync;               /*!< Whether the transmit is asynchronous or not. */
    const uint8_t * sendBuffer;         /*!< The buffer of data being sent.*/
    size_t remainingSendByteCount;      /*!< The remaining number of bytes to be transmitted. */
    size_t transmittedByteCount;        /*!< Number of bytes transmitted so far. */
    bool isReceiveInProgress;           /*!< True if there is an active receive. */
    bool isReceiveAsync;                /*!< Whether the receive is asynchronous or not. */
    uint8_t * receiveBuffer;            /*!< The buffer of received data. */
    size_t remainingReceiveByteCount;   /*!< The remaining number of bytes to be received. */
    size_t receivedByteCount;           /*!< Number of bytes received so far. */
    sync_object_t txIrqSync;            /*!< Used to wait for ISR to complete its TX business. */
    sync_object_t rxIrqSync;            /*!< Used to wait for ISR to complete its RX business. */
    uint8_t txFifoEntryCount;           /*!< Number of data word entries in TX FIFO. */
} uart_state_t;

/*!
 * @brief User configuration structure for UART driver.
 *
 * Use an instance of this struct with uart_init(). This allows you to configure the
 * most common settings of the UART peripheral with a single function call. Settings include:
 * UART baud rate; UART parity mode: disabled (default), or even or odd; the number of stop bits;
 * the number of bits per data word.
 */
typedef struct UartUserConfig {
    uint32_t baudRate;            /*!< UART baud rate*/
    uart_parity_mode_t parityMode;     /*!< parity mode, disabled (default), even, odd */
    uart_stop_bit_count_t stopBitCount; /*!< number of stop bits, 1 stop bit (default)
                                             or 2 stop bits */
    uart_bit_count_per_char_t bitCountPerChar; /*!< number of bits, 8-bit (default) or 9-bit in
                                                    a word (up to 10-bits in some UART instances) */
} uart_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name UART Driver
 * @{
 */

/*!
 * @brief This function initializes a UART instance for operation.
 *
 * This function will initialize the run-time state structure to keep track of the on-going
 * transfers, ungate the clock to the UART module, initialize the module
 * to user defined settings and default settings, configure the IRQ state structure and enable
 * the module-level interrupt to the core, and enable the UART module transmitter and receiver.
 * The following is an example of how to set up the uart_state_t and the
 * uart_user_config_t parameters and how to call the uart_init function by passing
 * in these parameters:
   @code
    uart_user_config_t uartConfig;
    uartConfig.baudRate = 9600;
    uartConfig.bitCountPerChar = kUart8BitsPerChar;
    uartConfig.parityMode = kUartParityDisabled;
    uartConfig.stopBitCount = kUartOneStopBit;
    uart_state_t uartState;
    uart_init(uartInstance, &uartConfig, &uartState);
    @endcode
 *
 * @param uartInstance The UART module instance number.
 * @param uartUserConfig The user configuration structure of type uart_user_config_t. The user
 *  is responsbile to fill out the members of this structure and to pass the pointer of this struct
 *  into this function.
 * @param uartState A pointer to the UART driver state structure memory. The user is only
 *  responsible to pass in the memory for this run-time state structure where the UART driver
 *  will take care of filling out the members. This run-time state structure keeps track of the
 *  current transfer in progress.
 * @return An error code or kStatus_UART_Success.
 */
uart_status_t uart_init(uint32_t uartInstance, const uart_user_config_t * uartUserConfig,
                   uart_state_t * uartState);

/*!
 * @brief This function sends (transmits) data out through the UART module using a blocking method.
 *
 * A blocking (also known as synchronous) function means that the function does not return until
 * the transmit is complete. This blocking function is used to send data through the UART port.
 *
 * @param uartInstance The UART module instance number.
 * @param sendBuffer A pointer to the source buffer containing 8-bit data chars to send.
 * @param txByteCount The number of bytes to send.
 * @param timeout A timeout value for RTOS abstraction sync control in milli-seconds (ms).
 * @return An error code or kStatus_UART_Success.
 */
uart_status_t uart_send_data(uart_state_t * uartState, uint8_t * sendBuffer, uint32_t txByteCount,
                        uint32_t timeout);

/*!
 * @brief This function sends (transmits) data through the UART module using a non-blocking method.
 *
 * A non-blocking (also known as synchronous) function means that the function returns
 * immediately after initiating the transmit function. The application has to get the
 * transmit status to see when the transmit is complete. In other words, after calling non-blocking
 * (asynchronous) send function, the application must get the transmit status to check if transmit
 * is completed or not.
 * The asynchronous method of transmitting and receiving allows the UART to perform a full duplex
 * operation (simultaneously transmit and receive).
 *
 * @param uartState A pointer to the UART driver state structure.
 * @param sendBuffer A pointer to the source buffer containing 8-bit data chars to send.
 * @param txByteCount The number of bytes to send.
 * @return An error code or kStatus_UART_Success.
 */
uart_status_t uart_send_data_async(uart_state_t * uartState, uint8_t * sendBuffer,
                              uint32_t txByteCount);

/*!
 * @brief This function returns whether the previous UART transmit has finished.
 *
 * When performing an async transmit, the user can call this function to ascertain the state of the
 * current transmission: in progress (or busy) or complete (success). In addition, if the
 * transmission is still in progress, the user can obtain the number of words that have been
 * currently transferred.
 *
 * @param uartState A pointer to the UART driver state structure.
 * @param bytesTransmitted A pointer to a value that is filled in with the number of bytes that
 *                         are sent in the active transfer.
 *
 * @retval kStatus_UART_Success The transmit has completed successfully.
 * @retval kStatus_UART_TxBusy The transmit is still in progress. @a bytesTransmitted is
 *     filled with the number of bytes which are transmitted up to that point.
 */
uart_status_t uart_get_transmit_status(uart_state_t * uartState, uint32_t * bytesTransmitted);

/*!
 * @brief This function returns whether the previous UART receive is complete.
 *
 * When performing an async receive, the user can call this function to ascertain the state of the
 * current receive progress: in progress (or busy) or complete (success). In addition, if the
 * receive is still in progress, the user can obtain the number of words that have been
 * currently received.
 *
 * @param uartState A pointer to the UART driver state structure.
 * @param bytesReceived A pointer to a value that is filled in with the number of bytes which
 *                      are received in the active transfer.
 *
 * @retval kStatus_UART_Success The receive has completed successfully.
 * @retval kStatus_UART_RxBusy The receive is still in progress. @a bytesReceived is
 *     filled with the number of bytes which are received up to that point.
 */
uart_status_t uart_get_receive_status(uart_state_t * uartState, uint32_t * bytesReceived);

/*!
 * @brief This function shuts down the UART by disabling interrupts and the transmitter/receiver.
 *
 * This function disables the UART interrupts, disables the transmitter and receiver, and
 * flushes the FIFOs (for modules that support FIFOs).
 *
 * @param uartState A pointer to the UART driver state structure.
 */
void uart_shutdown(uart_state_t * uartState);

/*!
 * @brief This function gets (receives) data from the UART module using a blocking method.
 *
 * A blocking (also known as synchronous) function means that the function does not return until
 * the receive is complete. This blocking function is used to send data through the UART port.
 *
 * @param uartState A pointer to the UART driver state structure.
 * @param rxBuffer A pointer to the buffer containing 8-bit read data chars received.
 * @param requestedByteCount The number of bytes to receive.
 * @param timeout A timeout value for RTOS abstraction sync control in milli-seconds (ms).
 * @return An error code or kStatus_UART_Success.
 */
uart_status_t uart_receive_data(uart_state_t * uartState, uint8_t * rxBuffer,
                           uint32_t requestedByteCount, uint32_t timeout);

/*!
 * @brief This function gets (receives) data from the UART module using a non-blocking method.
 *
 * A non-blocking (also known as synchronous) function means that the function returns
 * immediately after initiating the receive function. The application has to get the
 * receive status to see when the receive is complete. In other words, after calling non-blocking
 * (asynchronous) get function, the application must get the receive status to check if receive
 * is completed or not.
 * The asynchronous method of transmitting and receiving allows the UART to perform a full duplex
 * operation (simultaneously transmit and receive).
 *
 * @param uartState A pointer to the UART driver state structure.
 * @param rxBuffer  A pointer to the buffer containing 8-bit read data chars received.
 * @param requestedByteCount The number of bytes to receive.
 * @return An error code or kStatus_UART_Success.
 */
uart_status_t uart_receive_data_async(uart_state_t * uartState, uint8_t * rxBuffer,
                                 uint32_t requestedByteCount);

/*!
 * @brief This function terminates an asynchronous UART transmission early.
 *
 * During an async UART tranmission, the user has the option to terminate the transmission early
 * if the transmission is still in progress.
 *
 * @param uartState A pointer to the UART driver state structure.
 *
 * @retval kStatus_UART_Success The transmit was succesful.
 * @retval kStatus_UART_NoTransmitInProgress No transmission is currently in progress.
 */
uart_status_t uart_abort_sending_data(uart_state_t * uartState);

/*!
 * @brief This function terminates an asynchronous UART receive early.
 *
 * During an async UART receive, the user has the option to terminate the receive early
 * if the receive is still in progress.
 *
 * @param uartState A pointer to the UART driver state structure.
 *
 * @retval kStatus_UART_Success The receive was succesful.
 * @retval kStatus_UART_NoTransmitInProgress No receive is currently in progress.
 */
uart_status_t uart_abort_receiving_data(uart_state_t * uartState);

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_UART_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

