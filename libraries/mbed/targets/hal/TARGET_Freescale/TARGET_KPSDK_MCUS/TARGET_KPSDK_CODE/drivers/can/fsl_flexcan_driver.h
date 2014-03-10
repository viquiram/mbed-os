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
#ifndef __FSL_FLEXCAN_DRIVER_H__
#define __FSL_FLEXCAN_DRIVER_H__

#include "fsl_flexcan_hal.h"

/*!
 * @addtogroup flexcan_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* @brief Number of interrupts.*/
#define FSL_CAN_INTERRUPT_COUNT (4)

/*! @brief FlexCAN bitrates supported.*/
typedef enum _flexcan_bitrate {
    kFlexCanBitrate_125k  = 125000,  /*!< 125 kHz*/
    kFlexCanBitrate_250k  = 250000,  /*!< 250 kHz*/
    kFlexCanBitrate_500k  = 500000,  /*!< 500 kHz*/
    kFlexCanBitrate_750k  = 750000,  /*!< 750 kHz*/
    kFlexCanBitrate_1M    = 1000000, /*!< 1 MHz*/
} flexcan_bitrate_t;

/*! @brief FlexCAN bit rate and the related timing segments structure*/
typedef struct flexcan_bitrate_table {
    flexcan_bitrate_t bit_rate;    /*!< bit rate*/
    uint32_t propseg;              /*!< Propagation segment*/
    uint32_t pseg1;                /*!< Phase segment 1*/
    uint32_t pseg2;                /*!< Phase segment 2*/
    uint32_t pre_divider;          /*!< Clock pre divider*/
    uint32_t rjw;                  /*!< Re-sync jump width*/
} flexcan_bitrate_table_t;

/*! @brief FlexCAN data info from user*/
typedef struct FLEXCANDataInfo {
    flexcan_mb_id_type_t msg_id_type;            /*!< Type of message ID (standard or extended)*/
    uint32_t data_length;                        /*!< Length of Data in Bytes*/
} flexcan_data_info_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Bit rate
 * @{
 */

/*!
 * @brief Sets FlexCAN bit rate.
 *
 * @param   instance    A FlexCAN instance number
 * @param   bit rate     Selects a FlexCAN bit rate in the bit_rate_table.
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_set_bitrate(uint8_t instance, flexcan_bitrate_t bitrate);

/*!
 * @brief Gets FlexCAN bit rate.
 *
 * @param   instance    A FlexCAN instance number
 * @param   bit rate     A pointer to a variable for returning the FlexCAN bit rate
 *                      in the bit_rate_table.
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_get_bitrate(uint8_t instance, flexcan_bitrate_t *bitrate);

/*@}*/

/*!
 * @name Global mask
 * @{
 */

/*!
 * @brief Sets the RX masking type.
 *
 * @param   instance     A FlexCAN instance number
 * @param   type         The FlexCAN RX mask type
 */
void flexcan_set_mask_type(uint8_t instance, flexcan_rx_mask_type_t type);

/*!
 * @brief Sets the FlexCAN RX FIFO global standard or extended mask.
 *
 * @param   instance    A FlexCAN instance number
 * @param   id_type     Standard ID or extended ID
 * @param   mask        Mask value
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_set_rx_fifo_global_mask(
    uint8_t instance,
    flexcan_mb_id_type_t id_type,
    uint32_t mask);

/*!
 * @brief Sets the FlexCAN RX MB global standard or extended mask.
 *
 * @param   instance    A FlexCAN instance number
 * @param   id_type     Standard ID or extended ID
 * @param   mask        Mask value
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_set_rx_mb_global_mask(
    uint8_t instance,
    flexcan_mb_id_type_t id_type,
    uint32_t mask);

/*!
 * @brief Sets the FlexCAN RX individual standard or extended mask.
 *
 * @param   instance                   A FlexCAN instance number
 * @param   id_type                    A standard ID or an extended ID
 * @param   mb_idx                     Index of the message buffer
 * @param   mask                       Mask value
 * @return  0 if successful; non-zero failed.
 */
flexcan_status_t flexcan_set_rx_individual_mask(
    uint8_t instance,
    const flexcan_user_config_t * data,
    flexcan_mb_id_type_t id_type,
    uint32_t mb_idx,
    uint32_t mask);

/*@}*/

/*!
 * @name Init and Shutdown
 * @{
 */

/*!
 * @brief Initializes the FlexCAN peripheral.
 *
 * This function initializes 
 * @param   instance                   A FlexCAN instance number
 * @param   data                       The FlexCAN platform data
 * @param   enable_err_interrupts      1 if enabled, 0 if not
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_init(
   uint8_t instance,
   const flexcan_user_config_t *data,
   bool enable_err_interrupts);

/*!
 * @brief Shut down a FlexCAN instance.
 *
 * @param   instance    A FlexCAN instance number
 * @return  0 if successful; non-zero failed
 */
uint32_t flexcan_shutdown(uint8_t instance);

/*@}*/

/*!
 * @name Send configuration
 * @{
 */

/*!
 * @brief FlexCAN transmit message buffer field configuration.
 *
 * @param   instance                   A FlexCAN instance number
 * @param   data                       The FlexCAN platform data
 * @param   mb_idx                     Index of the message buffer
 * @param   tx_info                    Data info
 * @param   msg_id                     ID of the message to transmit
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_tx_mb_config(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *tx_info,
    uint32_t msg_id);

/*!
 * @brief Sends FlexCAN messages.
 *
 * @param   instance                   A FlexCAN instance number
 * @param   data                       The FlexCAN platform data
 * @param   mb_idx                     Index of the message buffer
 * @param   tx_info                    Data info
 * @param   msg_id                     ID of the message to transmit
 * @param   num_bytes                  The number of bytes in mb_data
 * @param   mb_data                    Bytes of the FlexCAN message
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_send(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *tx_info,
    uint32_t msg_id,
    uint32_t num_bytes,
    uint8_t *mb_data);

/*@}*/

/*!
 * @name Receive configuration
 * @{
 */

/*!
 * @brief FlexCAN receive message buffer field configuration
 *
 * @param   instance                   A FlexCAN instance number
 * @param   data                       The FlexCAN platform data
 * @param   mb_idx                     Index of the message buffer
 * @param   rx_info                    Data info
 * @param   msg_id                     ID of the message to transmit
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_rx_mb_config(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *rx_info,
    uint32_t msg_id);

/*!
 * @brief FlexCAN RX FIFO field configuration
 *
 * @param   instance           A FlexCAN instance number
 * @param   data               The FlexCAN platform data
 * @param   id_format          The format of the Rx FIFO ID Filter Table Elements
 * @param   id_filter_table    The ID filter table elements which contain RTR bit, IDE bit,
 *                             and RX message ID
 * @return  0 if successful; non-zero failed.
 */
flexcan_status_t flexcan_rx_fifo_config(
    uint8_t instance,
    const flexcan_user_config_t *data,
    flexcan_rx_fifo_id_element_format_t id_format,
    flexcan_id_table_t *id_filter_table);

/*!
 * @brief FlexCAN is waiting to receive data.
 *
 * @param   instance                   A FlexCAN instance number
 * @param   data                       The FlexCAN platform data
 * @param   mb_idx                     Index of the message buffer
 * @param   msg_id                     ID of the message to transmit
 * @param   receiveDataCount           The number of data to be received
 * @param   rx_mb                      The FlexCAN receive message buffer data.
 * @param   rx_fifo                    The FlexCAN receive FIFO data.
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_start_receive(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    uint32_t receiveDataCount,
    bool *is_rx_mb_data,
    bool *is_rx_fifo_data,
    flexcan_mb_t *rx_mb,
    flexcan_mb_t *rx_fifo);

/*!
 * @brief FlexCAN is preparing to receive data.
 *
 * @param   instance                   A FlexCAN instance number
 * @param   data                       The FlexCAN platform data
 * @param   mb_idx                     Index of the message buffer
 * @param   cs                         CODE/status values (RX)
 * @param   msg_id                     ID of the message to transmit
 * @param   id_format                  The format of the Rx FIFO ID Filter Table Elements
 * @param   id_filter_table            The ID filter table elements which contain RTR bit, IDE bit,
 *                                     and RX message ID
 * @param   receiveDataCount           The number of data to be received
 * @param   rx_mb                      The FlexCAN receive message buffer data
 * @param   rx_fifo                    The FlexCAN receive FIFO data
 * @return  0 if successful; non-zero failed
 */
flexcan_status_t flexcan_receive(
    uint8_t instance,
    const flexcan_user_config_t *data,
    uint32_t mb_idx,
    flexcan_data_info_t *rx_info,
    uint32_t msg_id,
    flexcan_rx_fifo_id_element_format_t id_format,
    flexcan_id_table_t *id_filter_table,
    uint32_t receiveDataCount,
    flexcan_mb_t *rx_mb,
    flexcan_mb_t *rx_fifo);

/*@}*/

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif /* __FSL_FLEXCAN_DRIVER_H__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

