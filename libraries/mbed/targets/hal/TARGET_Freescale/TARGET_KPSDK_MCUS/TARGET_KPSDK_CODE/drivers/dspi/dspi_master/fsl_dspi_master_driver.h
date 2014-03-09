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
#if !defined(__FSL_DSPI_MASTER_DRIVER_H__)
#define __FSL_DSPI_MASTER_DRIVER_H__

#include "fsl_dspi_hal.h"
#include "fsl_os_abstraction.h"

/*!
 * @addtogroup dspi_master_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Data structure containing information about a device on the SPI bus.
 *
 * The user must fill out these members to set up the DSPI master  to
 * properly communicate with the SPI device.
 */
typedef struct DSPIDevice {
    uint32_t bitsPerSec;                 /*!< @brief Baud rate in bits per second.*/
    dspi_data_format_config_t dataBusConfig;  /* data format config struct*/
} dspi_device_t;

/*!
 * @brief Runtime state structure for the DSPI master driver.
 *
 * This struct holds data that is used by the DSPI master peripheral driver to
 * communicate between the transfer function and the interrupt handler. The
 * interrupt handler also uses this information to keep track of its progress.
 * The user must pass  the memory for this run-time state structure and the
 * DSPI master driver will fill out the members.
 */
typedef struct DSPIMasterState {
    uint32_t instance;                      /*!< DSPI module instance number*/
    dspi_ctar_selection_t whichCtar; /*!< Desired Clock and Transfer Attributes Register (CTAR)*/
    uint32_t bitsPerFrame;         /*!< Desired number of bits per frame */
    dspi_which_pcs_config_t whichPcs; /*!< Desired Peripheral Chip Select (pcs) */
    bool isChipSelectContinuous;  /*!< Option to enable the continuous assertion of chip select
                                       between transfers*/
    uint32_t dspiSourceClock;              /*!< Module source clock*/
    volatile bool isTransferInProgress;             /*!< True if there is an active transfer.*/
    bool isTransferAsync;                  /*!< Whether the transfer is asynchronous (needed in
                                                IRQ).*/
    const uint8_t * restrict sendBuffer;  /*!< The buffer from which transmitted bytes are taken.*/
    uint8_t * restrict receiveBuffer;     /*!< The buffer into which received bytes are placed.*/
    volatile size_t remainingSendByteCount;         /*!< Number of bytes remaining to send.*/
    volatile size_t remainingReceiveByteCount;      /*!< Number of bytes remaining to receive.*/
    sync_object_t irqSync;                 /*!< Used to wait for ISR to complete its business.*/
} dspi_master_state_t;

/*!
 * @brief The user configuration structure for the DSPI master driver.
 *
 * Use an instance of this struct with the dspi_master_init(). This allows the user to configure the
 * most common settings of the DSPI peripheral with a single function call.
 */
typedef struct DSPIMasterUserConfig {
    dspi_ctar_selection_t whichCtar; /*!< Desired Clock and Transfer Attributes Register(CTAR)*/
    bool isSckContinuous;                  /*!< Disable or Enable continuous SCK operation*/
    bool isChipSelectContinuous;  /*!< Option to enable the continuous assertion of chip select
                                       between transfers */
    dspi_which_pcs_config_t whichPcs;        /*!< Desired Peripheral Chip Select (pcs) */
    dspi_pcs_polarity_config_t pcsPolarity;  /*!< Peripheral Chip Select (pcs) polarity setting.*/
} dspi_master_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Init and shutdown
 * @{
 */

/*!
 * @brief Initialize a DSPI instance for master mode operation.
 *
 * This function initializes the run-time state structure to track the ongoing
 * transfers, ungates the clock to the DSPI module, resets the DSPI module, initializes the module
 * to user defined settings and default settings, configures the IRQ state structure, enables
 * the module-level interrupt to the core, and enables the DSPI module.
 * The CTAR parameter is special in that it allows the user to have different SPI devices
 * connected to the same DSPI module instance in addition to different peripheral chip
 * selects. Each CTAR contains the bus attributes associated with that particular SPI device.
 * For most use cases where only one SPI device is connected per DSPI module
 * instance, use CTAR0.
 * This is an example to set up the dspi_master_state_t and the
 * dspi_master_user_config_t parameters and to call the dspi_master_init function by passing
 * in these parameters:
   @code
    dspi_master_state_t dspiMasterState; <- the user simply allocates memory for this struct
    uint32_t calculatedBaudRate;
    dspi_master_user_config_t userConfig; <- the user fills out members for this struct
    userConfig.isChipSelectContinuous = false;
    userConfig.isSckContinuous = false;
    userConfig.pcsPolarity = kDspiPcs_ActiveLow;
    userConfig.whichCtar = kDspiCtar0;
    userConfig.whichPcs = kDspiPcs0;
    dspi_master_init(masterInstance, &dspiMasterState, &userConfig, &calculatedBaudRate);
   @endcode
 *
 * @param instance The instance number of the DSPI peripheral.
 * @param dspiState The pointer to the DSPI master driver state structure. The user
 *  must pass the memory for this run-time state structure and the DSPI master driver
 *  will  fill out the members. This run-time state structure keeps track of the
 *  transfer in progress.
 * @param userConfig The dspi_master_user_config_t user configuration structure. The user
 *  must fill out the members of this structure and  pass the pointer of this struct
 *  into the function.
 * @param calculatedBaudRate The calculated baud rate passed back to the user to determine
 *  if the calculated baud rate is close enough to meet the needs. The value returned may be 0
 *  if the user decides to set the baud rate to 0 in the user configuration struct. The user can
 *  later configure the baud rate via the dspi_master_configure_bus function, and hence may not
 *  need to configure the baud rate in the dspi_master_init function.
 *
 * @return An error code or kStatus_DSPI_Success.
 */
dspi_status_t dspi_master_init(uint32_t instance,
                               dspi_master_state_t * dspiState,
                               const dspi_master_user_config_t * userConfig,
                               uint32_t * calculatedBaudRate);

/*!
 * @brief Shuts down a DSPI instance.
 *
 * This function resets the DSPI peripheral, gates its clock, and disables the interrupt to
 * the core.
 *
 * @param dspiState The pointer to the DSPI master driver state structure.
 */
void dspi_master_shutdown(dspi_master_state_t * dspiState);


/*!
 * @brief Configures the DSPI modified transfer format in master mode.
 *
 * This function allows the user to enable or disable (default setting) the modified transfer
 * format. The modified transfer format is supported to allow  high-speed communication with
 * peripherals that require longer setup times. The module can sample the incoming data
 * later than halfway through the cycle to give the peripheral more setup time. The data-in
 * sample point can also be configured in this function. Note that the data-in sample point setting
 * is valid only when the CPHA bit in the CTAR is cleared (when the dspi_clock_phase_t is
 * set to kDspiClockPhase_FirstEdge in the dspi_data_format_config_t).
 *
 * @param dspiState The pointer to the DSPI master driver state structure.
 * @param enableOrDisable Enables (true) or disables(false) the modified transfer (timing) format.
 * @param samplePnt Selects when the data in (SIN) is sampled, of type dspi_master_sample_point_t.
 *                  This value selects either 0, 1, or 2 system clocks between the SCK edge
 *                  and the SIN (data in) sample.
 */
void dspi_master_configure_modified_transfer_format(dspi_master_state_t * dspiState,
                                                    bool enableOrDisable,
                                                    dspi_master_sample_point_t samplePnt);

/*@}*/

/*!
 * @name Bus configuration
 * @{
 */

/*!
 * @brief Configures the DSPI port physical parameters to access a device on the bus.
 *
 * The term "device" is used to indicate the SPI device for which the DSPI master is communicating.
 * The user has two options to configure the device parameters: either pass in the
 * pointer to the device configuration structure to the desired transfer function (see
 * dspi_master_transfer or dspi_master_transfer_async) or pass it in to the
 * dspi_master_configure_bus function.  The user can pass in a device structure to the transfer
 * function which contains the parameters for the bus (the transfer function will then call
 * this function). However, the user has the option to call this function directly especially
 * to get the calculated baud rate, at which point they may pass in NULL for the device
 * struct in the transfer function (assuming they have called this configure bus function
 * first). This is an example to set up the dspi_device_t structure to call
 * the dspi_master_configure_bus function by passing in these parameters:
   @code
    dspi_device_t spiDevice;
    spiDevice.dataBusConfig.bitsPerFrame = 16;
    spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
    spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    spiDevice.dataBusConfig.direction = kDspiMsbFirst;
    spiDevice.bitsPerSec = 50000;
    dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate);
   @endcode
 *
 * @param dspiState The pointer to the DSPI master driver state structure.
 * @param device Pointer to the device information struct. This struct contains the settings
 *  for the SPI bus configuration.  The device parameters are the desired baud rate (in
 *  bits-per-sec), and the data format field which consists of bits-per-frame, clock polarity and
 *  phase, and data shift direction.
 * @param calculatedBaudRate The calculated baud rate passed back to the user to determine
 *  if the calculated baud rate is close enough to meet the needs. The baud rate never exceeds
 *  the desired baud rate.
 * @return An error code or kStatus_DSPI_Success.
 */
dspi_status_t dspi_master_configure_bus(dspi_master_state_t * dspiState,
                                        const dspi_device_t * device,
                                        uint32_t * calculatedBaudRate);

/*@}*/

/*!
 * @name Blocking transfers
 * @{
 */

/*!
 * @brief Performs a blocking SPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus. The function does not return until the transfer is complete.
 *
 * @param dspiState The pointer to the DSPI master driver state structure.
 * @param device Pointer to the device information struct. This struct contains the settings
 *  for the SPI bus configuration in this transfer. You may pass NULL for this
 *  parameter, in which case the current bus configuration is used unmodified. The device can be
 *  configured separately by calling the dspi_master_configure_bus function.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter and  bytes with a value of 0 (zero) will be sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 * @param timeout A timeout for the transfer in microseconds. If the transfer takes longer than
 *  this amount of time, the transfer is aborted and a #kStatus_SPI_Timeout error
 *  returned.
 *
 * @retval kStatus_DSPI_Success The transfer was successful.
 * @retval kStatus_DSPI_Busy Cannot perform another transfer because a transfer is already in progress.
 * @retval kStatus_DSPI_Timeout The transfer timed out and was aborted.
 */
dspi_status_t dspi_master_transfer(dspi_master_state_t * dspiState,
                              const dspi_device_t * restrict device,
                              const uint8_t * sendBuffer,
                              uint8_t * receiveBuffer,
                              size_t transferByteCount,
                              uint32_t timeout);
/*@}*/

/*!
 * @name Non-blocking transfers
 * @{
 */

/*!
 * @brief Performs a non-blocking SPI master mode transfer.
 *
 * This function  returns immediately. The user must check back to
 * check whether the transfer is complete (using the dspi_master_get_transfer_status function). This
 * function simultaneously sends and receives data on the SPI bus, as SPI is naturally
 * a full-duplex bus.
 *
 * @param dspiState The pointer to the DSPI master driver state structure.
 * @param device Pointer to the device information struct. This struct contains the settings
 *  for the SPI bus configuration in this transfer. You may pass NULL for this
 *  parameter, in which case the current bus configuration is used unmodified. The device can be
 *  configured separately by calling the dspi_master_configure_bus function.
 * @param sendBuffer The pointer to the data buffer of the data to send. You may pass NULL for this
 *  parameter, in which case bytes with a value of 0 (zero) are sent.
 * @param receiveBuffer Pointer to the buffer where the received bytes are stored. If you pass NULL
 *  for this parameter, the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 *
 * @retval kStatus_DSPI_Success The transfer was successful.
 * @retval kStatus_DSPI_Busy Cannot perform another transfer because a transfer is already in progress.
 */
dspi_status_t dspi_master_transfer_async(dspi_master_state_t * dspiState,
                                    const dspi_device_t * restrict device,
                                    const uint8_t * sendBuffer,
                                    uint8_t * receiveBuffer,
                                    size_t transferByteCount);


/*!
 * @brief Returns whether the previous transfer is finished.
 *
 * When performing an a-sync transfer, the user can call this function to ascertain the state of the
 * current transfer: in progress (or busy) or complete (success). In addition, if the transfer
 * is still in progress, the user can get the number of words that have been
 * transferred up to now.
 *
 * @param dspiState The pointer to the DSPI master driver state structure
 * @param framesTransferred Pointer to value that is filled in with the number of frames that
 *  have been sent in the active transfer. A frame is defined as the number of bits per frame.
 *
 * @retval kStatus_DSPI_Success The transfer has completed successfully.
 * @retval kStatus_DSPI_Busy The transfer is still in progress. @a wordsTransferred will be filled
 *  with the number of words that have been transferred so far.
 */
dspi_status_t dspi_master_get_transfer_status(dspi_master_state_t * dspiState,
                                         uint32_t * framesTransferred);

/*!
 * @brief Terminates an asynchronous transfer early.
 *
 * During an a-sync transfer, the user has the option to terminate the transfer early if the transfer
 * is still in progress.
 *
 * @param dspiState The pointer to the DSPI master driver state structure.
 *
 * @retval kStatus_DSPI_Success The transfer was successful.
 * @retval kStatus_DSPI_NoTransferInProgress No transfer is currently in progress.
 */
dspi_status_t dspi_master_abort_transfer(dspi_master_state_t * dspiState);

/* @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_DSPI_MASTER_DRIVER_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

