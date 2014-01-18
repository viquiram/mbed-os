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

#ifndef __FSL_ENET_HAL_H__
#define __FSL_ENET_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_device_registers.h"
#include "fsl_enet_features.h"
#include <assert.h>

/*!
 * @addtogroup enet_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Define system endian type*/
#define SYSTEM_LITTLE_ENDIAN      (1)

/*! @brief Define macro to do the endianness swap*/
#define BSWAP_16(x)	(uint16_t)((uint16_t)(((uint16_t)(x) & (uint16_t)0xFF00) >> 0x8) | (uint16_t)(((uint16_t)(x) & (uint16_t)0xFF) << 0x8))
#define BSWAP_32(x) (uint32_t)((((uint32_t)(x) & 0x00FFU) << 24) | (((uint32_t)(x) & 0x00FF00U) << 8) | (((uint32_t)(x) & 0xFF0000U) >> 8) | (((uint32_t)(x) & 0xFF000000U) >> 24))
#if SYSTEM_LITTLE_ENDIAN && FSL_FEATURE_ENET_DMA_BIG_ENDIAN_ONLY 
#define HTONS(n)                      BSWAP_16(n)
#define HTONL(n)                      BSWAP_32(n)
#define NTOHS(n)                      BSWAP_16(n)
#define NTOHL(n)                      BSWAP_32(n)
#else
#define HTONS(n)                       (n)
#define HTONL(n)                       (n)
#define NTOHS(n)                       (n)
#define NTOHL(n)                       (n)
#endif

/*! @brief Define Status return codes.*/
typedef enum _enet_status
{
    kStatus_ENET_Success = 0,
    kStatus_ENET_InvalidInput,       /*!< Invalid enet input parameter */
    kStatus_ENET_MemoryAllocateFail, /*!< Memory allocate failure*/
    kStatus_ENET_GetClockFreqFail,   /*!< Get clock frequency failure*/
    kStatus_ENET_Initialized,        /*!< ENET device already initialized*/
    kStatus_ENET_Layer2QueueNull,    /*!< NULL L2 ptp buffer queue pointer*/
    kStatus_ENET_Layer2OverLarge,    /*!< Layer2 packet length over large*/
    kStatus_ENET_Layer2BufferFull,   /*!< Layer2 packet buffer full*/
    kStatus_ENET_PtpringBufferFull,  /*!< Ptp ring buffer full*/
    kStatus_ENET_PtpringBufferEmpty, /*!< Ptp ring buffer empty*/
    kStatus_ENET_Miiuninitialized,   /*!< MII uninitialized*/
    kStatus_ENET_RxbdInvalid,        /*!< Receive buffer descriptor invalid*/
    kStatus_ENET_RxbdEmpty,          /*!< Receive buffer descriptor empty*/
    kStatus_ENET_RxbdTrunc,          /*!< Receive buffer descriptor truncate*/
    kStatus_ENET_RxbdError,          /*!< Receive buffer descriptor error*/
    kStatus_ENET_RxBdFull,           /*!< Receive buffer descriptor full*/
    kStatus_ENET_SmallBdSize,        /*!< Small receive buffer size*/
    kStatus_ENET_LargeBufferFull,    /*!< Receive large buffer full*/
    kStatus_ENET_TxbdFull,           /*!< Transmit buffer descriptor full*/
    kStatus_ENET_TxbdNull,           /*!< Transmit buffer descriptor Null*/
    kStatus_ENET_TxBufferNull,       /*!< Transmit data buffer Null*/
    kStatus_ENET_NoRxBufferLeft,       /*!< No more receive buffer left*/
    kStatus_ENET_UnknownCommand,     /*!< Invalid ENET ptp ioctl command*/
    kStatus_ENET_TimeOut,            /*!< ENET Timeout*/
    kStatus_ENET_MulticastPointerNull, /*!< Null multicast group pointer*/
    kStatus_ENET_AlreadyAddedMulticast /*!< Have Already added to multicast group*/
}enet_status_t;


#if FSL_FEATURE_ENET_DMA_BIG_ENDIAN_ONLY && SYSTEM_LITTLE_ENDIAN
/*! @brief Define control and status regions of receive buffer descriptor*/
typedef enum _enet_rx_bd_control_status
{
    kEnetRxBdBroadCast = 0x8000,       /*!< Broadcast */
    kEnetRxBdMultiCast = 0x4000,       /*!< Multicast*/
    kEnetRxBdLengthViolation = 0x2000, /*!< Receive length violation*/
    kEnetRxBdNoOctet = 0x1000,         /*!< Receive non-octet aligned frame*/
    kEnetRxBdCrc = 0x0400,             /*!< Receive crc error*/
    kEnetRxBdOverRun = 0x0200,         /*!< Receive FIFO overrun*/
    kEnetRxBdTrunc = 0x0100,           /*!< Frame is truncated */
    kEnetRxBdEmpty = 0x0080,           /*!< Empty bit*/
    kEnetRxBdRxSoftOwner1 = 0x0040,    /*!< Receive software owner*/
    kEnetRxBdWrap = 0x0020,            /*!< Update buffer descriptor*/
    kEnetRxBdRxSoftOwner2 = 0x0010,    /*!< receive software owner*/
    kEnetRxBdLast = 0x0008,            /*!< last bd in the frame*/
    kEnetRxBdMiss = 0x0001             /*!< receive for promiscuous mode*/
} enet_rx_bd_control_status_t;

/*! @brief Define control extended regions of receive buffer descriptor*/
typedef enum _enet_rx_bd_control_extend
{  
    kEnetRxBdUnicast = 0x0001,              /*!< Unicast frame*/
    kEnetRxBdCollision = 0x0002,            /*!< Bd collision*/
    kEnetRxBdPhyErr = 0x0004,               /*!< Phy error*/
    kEnetRxBdMacErr = 0x0080,               /*!< Mac error*/
    kEnetRxBdIpv4 = 0x0100,                 /*!< Ipv4 frame*/
    kEnetRxBdIpv6 = 0x0200,                 /*!< Ipv6 frame*/
    kEnetRxBdVlan = 0x0400,                 /*!< Vlan*/
    kEnetRxBdProtocolChecksumErr = 0x1000,  /*!< protocol checksum error*/
    kEnetRxBdIpHeaderChecksumErr = 0x2000,  /*!< ip header checksum error*/
    kEnetRxBdIntrrupt = 0x8000              /*!< bd interrupt*/
} enet_rx_bd_control_extend_t;

/*! @brief Define control status region of transmit buffer descriptor*/
typedef enum _enet_tx_bd_control_status
{
    kEnetTxBdReady = 0x0080,         /*!<  Ready bit*/
    kEnetTxBdTxSoftOwner1 = 0x0040,  /*!<  Transmit software owner*/
    kEnetTxBdWrap = 0x0020,          /*!<  Wrap buffer descriptor*/
    kEnetTxBdTxSoftOwner2 = 0x0010,  /*!<  Transmit software owner*/
    kEnetTxBdLast = 0x0008,          /*!<  Last bd in the frame*/
    kEnetTxBdTransmitCrc = 0x0004    /*!<  Receive for transmit CRC*/
} enet_tx_bd_control_status_t;

/*! @brief Define control extended region of transmit buffer descriptor*/
typedef enum _enet_tx_bd_control_extend
{
    kEnetTxBdTxErr = 0x0080,                 /*!<  Transmit error*/
    kEnetTxBdTxUnderFlowErr = 0x0020,        /*!<  Underflow error*/
    kEnetTxBdExcessCollisionErr = 0x0010,    /*!<  Excess collision error*/
    kEnetTxBdTxFrameErr = 0x0008,            /*!<  Frame error*/
    kEnetTxBdLatecollisionErr = 0x0004,      /*!<  Late collision error*/
    kEnetTxBdOverFlowErr = 0x0002,           /*!<  Overflow error*/
    kEnetTxTimestampErr = 0x0001             /*!<  Timestamp error*/
} enet_tx_bd_control_extend_t;

/*! @brief Define control extended2 region of transmit buffer descriptor*/
typedef enum _enet_tx_bd_control_extend2
{
    kEnetTxBdTxInterrupt = 0x0040, /*!< Transmit interrupt*/
    kEnetTxBdTimeStamp = 0x0020    /*!< Transmit timesstamp flag */
}enet_tx_bd_control_extend2_t;
#else
/*! @brief Define control and status region of receive buffer descriptor*/
typedef enum _enet_rx_bd_control_status
{
    kEnetRxBdEmpty = 0x8000,           /*!< Empty bit*/
    kEnetRxBdRxSoftOwner1 = 0x4000,    /*!< Receive software owner*/
    kEnetRxBdWrap = 0x2000,            /*!< Update buffer descriptor*/
    kEnetRxBdRxSoftOwner2 = 0x1000,    /*!< Receive software owner*/
    kEnetRxBdLast = 0x0800,            /*!< Last bd in the frame*/
    kEnetRxBdMiss = 0x0100,            /*!< Receive for promiscuous mode*/
    kEnetRxBdBroadCast = 0x0080,       /*!< Broadcast */
    kEnetRxBdMultiCast = 0x0040,       /*!< Multicast*/
    kEnetRxBdLengthViolation = 0x0020, /*!< Receive length violation*/
    kEnetRxBdNoOctet = 0x0010,         /*!< Receive non-octet aligned frame*/
    kEnetRxBdCrc = 0x0004,             /*!< Receive crc error*/
    kEnetRxBdOverRun = 0x0002,         /*!< Receive FIFO overrun*/
    kEnetRxBdTrunc = 0x0001            /*!< Frame is truncated    */
} enet_rx_bd_control_status_t;

/*! @brief Define control extended region of receive buffer descriptor*/
typedef enum _enet_rx_bd_control_extend
{  
    kEnetRxBdIpv4 = 0x0001,                 /*!< Ipv4 frame*/
    kEnetRxBdIpv6 = 0x0002,                 /*!< Ipv6 frame*/
    kEnetRxBdVlan = 0x0004,                 /*!< Vlan*/
    kEnetRxBdProtocolChecksumErr = 0x0010,  /*!< Protocol checksum error*/
    kEnetRxBdIpHeaderChecksumErr = 0x0020,  /*!< Ip header checksum error*/
    kEnetRxBdIntrrupt = 0x0080,             /*!< Bd interrupt*/
    kEnetRxBdUnicast = 0x0100,              /*!< Unicast frame*/
    kEnetRxBdCollision = 0x0200,            /*!< Bd collision*/
    kEnetRxBdPhyErr = 0x0400,               /*!< Phy error*/
    kEnetRxBdMacErr = 0x8000                /*!< Mac error */
} enet_rx_bd_control_extend_t;

/*! @brief Define control status of transmit buffer descriptor*/
typedef enum _enet_tx_bd_control_status
{
    kEnetTxBdReady = 0x8000,         /*!<  Ready bit*/
    kEnetTxBdTxSoftOwner1 = 0x4000,  /*!<  Transmit software owner*/
    kEnetTxBdWrap = 0x2000,          /*!<  Wrap buffer descriptor*/
    kEnetTxBdTxSoftOwner2 = 0x1000,  /*!<  Transmit software owner*/
    kEnetTxBdLast = 0x0800,          /*!<  Last bd in the frame*/
    kEnetTxBdTransmitCrc = 0x0400    /*!<  Receive for transmit CRC   */
} enet_tx_bd_control_status_t;

/*! @brief Define control extended of transmit buffer descriptor*/
typedef enum _enet_tx_bd_control_extend
{
    kEnetTxBdTxErr = 0x8000,                /*!<  Transmit error*/
    kEnetTxBdTxUnderFlowErr = 0x2000,       /*!<  Underflow error*/
    kEnetTxBdExcessCollisionErr = 0x1000,   /*!<  Excess collision error*/
    kEnetTxBdTxFrameErr = 0x0800,           /*!<  Frame error*/
    kEnetTxBdLatecollisionErr = 0x0400,     /*!<  Late collision error*/
    kEnetTxBdOverFlowErr = 0x0200,          /*!<  Overflow error*/
    kEnetTxTimestampErr = 0x0100            /*!<  Timestamp error*/
} enet_tx_bd_control_extend_t;

/*! @brief Define control extended2 of transmit buffer descriptor*/
typedef enum _enet_tx_bd_control_extend2
{
    kEnetTxBdTxInterrupt = 0x4000, /*!< Transmit interrupt*/
    kEnetTxBdTimeStamp = 0x2000    /*!< Transmit timesstamp flag */
}enet_tx_bd_control_extend2_t;
#endif

#if (!FSL_FEATURE_ENET_DMA_BIG_ENDIAN_ONLY) && SYSTEM_LITTLE_ENDIAN
/*! @brief Define the buffer descriptor structure for little-Endian system and endianness configurable IP*/
typedef struct enet_bd_struct
{
    uint16_t  length;           /*!< Buffer descriptor data length*/
    uint16_t  control;          /*!< Buffer descriptor control*/
    uint8_t   *buffer;          /*!< Data buffer pointer*/
    uint16_t  controlExtend0;   /*!< Extend buffer descriptor control0*/
    uint16_t  controlExtend1;   /*!< Extend buffer descriptor control1*/
    uint16_t  payloadCheckSum;  /*!< Internal payload checksum*/
    uint8_t   headerLength;     /*!< Header length*/
    uint8_t   protocalTyte;     /*!< Protocal type*/
    uint16_t  reserved0;
    uint16_t  controlExtend2;   /*!< Extend buffer descriptor control2*/
    uint32_t  timestamp;        /*!< Timestamp */
    uint16_t  reserved1;
    uint16_t  reserved2;
    uint16_t  reserved3;
    uint16_t  reserved4;
}enet_bd_struct_t;

#else
/*! @brief Define the buffer descriptors structure for Big-Endian system*/
typedef struct enet_bd_struct
{
    uint16_t  control;          /*!< Buffer descriptor control   */
    uint16_t   length;          /*!< Buffer descriptor data length*/
    uint8_t   *buffer;          /*!< Data buffer pointer*/
    uint16_t  controlExtend1;   /*!< Extend buffer descriptor control1*/
    uint16_t  controlExtend0;   /*!< Extend buffer descriptor control0*/
    uint8_t   headerLength;     /*!< Header length*/
    uint8_t   protocalTyte;     /*!< Protocal type*/
    uint16_t  payloadCheckSum;  /*!< Internal payload checksum*/
    uint16_t  controlExtend2;   /*!< Extend buffer descriptor control2*/
    uint16_t  reserved0;  
    uint32_t  timestamp;        /*!< Timestamp pointer*/
    uint16_t  reserved1;
    uint16_t  reserved2;
    uint16_t  reserved3;
    uint16_t  reserved4;
}enet_bd_struct_t;
#endif

/*! @brief Define macro to different enet constant value*/
typedef enum _enet_constant_parameter
{
    kEnetMacAddrLen = 6,       /*!< enet mac address length*/
    kEnetHashValMask = 0x1f,   /*!< enet hash value mask*/
    kEnetRxBdCtlJudge1 = 0x0080,/*!< enet receive buffer descriptor control judge value1*/
    kEnetRxBdCtlJudge2 = 0x8000 /*!< enet receive buffer descriptor control judge value2*/
}enet_constant_parameter_t;

/*! @brief Define six-byte mac address type*/
typedef uint8_t enetMacAddr[kEnetMacAddrLen];

/*! @brief Define rmii or mii mode for data interface between MAC and PHY*/
typedef enum _enet_config_rmii
{
    kEnetCfgMii = 0,   /*!< Mii mode for data interface*/
    kEnetCfgRmii = 1   /*!< RMii mode for data interface*/
} enet_config_rmii_t;

/*! @brief Define 10Mbps or 100Mbps speed mode for data transfer */
typedef enum _enet_config_speed
{
    kEnetCfgSpeed100M = 0,  /*!< Speed 100M mode*/
    kEnetCfgSpeed10M = 1    /*!< Speed 10M mode*/
} enet_config_speed_t;

/*! @brief Define half or full duplex mode for data transfer*/
typedef enum _enet_config_duplex
{
    kEnetCfgHalfDuplex = 0, /*!< Half duplex mode*/
    kEnetCfgFullDuplex = 1  /*!< Full duplex mode*/
} enet_config_duplex_t;

/*! @brief Define Write /read operation for MII*/
typedef enum _enet_mii_operation
{
    kEnetWriteNoCompliant = 0, /*!< Write frame operation, but not MII compliant.*/
    kEnetWriteValidFrame = 1,  /*!< Write frame operation for a valid MII management frame*/
    kEnetReadValidFrame = 2,   /*!< Read frame operation for a valid MII management frame.*/
    kEnetReadNoCompliant = 3   /*!< Read frame operation, but not MII compliant*/
}enet_mii_operation_t;

/*! @brief Define initialize , enable or disable operation for special adddress filter */
typedef enum _enet_special_address_filter
{
    kEnetSpecialAddressInit= 0,     /*!< Initialize special address filter */
    kEnetSpecialAddressEnable = 1,  /*!< Enbale special address filter*/
    kEnetSpecialAddressDisable = 2  /*!< Disable special address filter*/
} enet_special_address_filter_t;

/*! @brief Define capture or compare mode for 1588 timer channels */
typedef enum _enet_timer_channel_mode
{
    kEnetChannelDisable = 0,         /*!< Disable timer channel*/
    kEnetChannelRisingCapture = 1,   /*!< Input capture on rising edge*/
    kEnetChannelFallingCapture = 2,  /*!< Input capture on falling edge*/
    kEnetChannelBothCapture = 3,     /*!< Input capture on both edges*/
    kEnetChannelSoftCompare = 4,     /*!< Output compare software only*/
    kEnetChannelToggleCompare = 5,   /*!< Toggle output on compare*/
    kEnetChannelClearCompare = 6,    /*!< Clear output on compare*/
    kEnetChannelSetCompare = 7,      /*!< Set output on compare*/
    kEnetChannelClearCompareSetOverflow = 8, /*!< Clear output on compare, set output on overflow*/
    kEnetChannelSetCompareClearOverflow = 9 /*!< Set output on compare,clear output on overflow*/
} enet_timer_channel_mode_t;

/*! @brief Define configuration structure for 1588 ptp timer*/
typedef struct enet_config_ptp_timer
{
    bool isSlaveEnabled;        /*!< Master or slave ptp timer*/
    uint32_t clockIncease;      /*!< Timer increase value each clock period*/
    uint32_t period;            /*!< Timer period for generate interrupt event  */
} enet_config_ptp_timer_t;

/*! @brief Define RXFRAME/RXBYTE/TXFRAME/TXBYTE/MII/TSTIMER/TSAVAIL interrupt source for enet*/
typedef enum _enet_interrupt_request
{
    kEnetBabrInterrupt = 0x40000000,   /*!< BABR interrupt source*/
    kEnetBabtInterrupt = 0x20000000,   /*!< BABT interrupt source*/
    kEnetGraInterrupt = 0x10000000,    /*!< GRA interrupt source*/
    kEnetTxFrameInterrupt = 0x8000000, /*!< TXFRAME interrupt source */
    kEnetTxByteInterrupt = 0x4000000,  /*!< TXBYTE interrupt source*/
    kEnetRxFrameInterrupt = 0x2000000, /*!< RXFRAME interrupt source */
    kEnetRxByteInterrupt = 0x1000000,  /*!< RXBYTE interrupt source */
    kEnetMiiInterrupt = 0x0800000,     /*!< MII interrupt source*/
    kEnetEBERInterrupt = 0x0400000,    /*!< EBERR interrupt source*/
    kEnetLcInterrupt = 0x0200000,      /*!< LC interrupt source*/
    kEnetRlInterrupt = 0x0100000,      /*!< RL interrupt source*/
    kEnetUnInterrupt = 0x0080000,      /*!< UN interrupt source*/
    kEnetPlrInterrupt = 0x0040000,     /*!< PLR interrupt source*/
    kEnetWakeupInterrupt = 0x0020000,  /*!< WAKEUP interrupt source*/
    kEnetTsAvailInterrupt = 0x0010000, /*!< TS AVAIL interrupt source*/
    kEnetTsTimerInterrupt = 0x0008000, /*!< TS WRAP interrupt source*/
    kEnetAllInterrupt = 0xFFFFFFFF     /*!< All interrupt*/
} enet_interrupt_request_t;

/*! @brief Define transmit accelerator configuration*/
typedef struct enet_config_tx_accelerator
{
    bool  isIpCheckEnabled;         /*!< Insert ip header checksum */
    bool  isProtocolCheckEnabled;   /*!< Insert protocol checksum*/
    bool  isShift16Enabled;         /*!< Tx fifo shift-16*/
} enet_config_tx_accelerator_t;

/*! @brief Define receive accelerator configuration*/
typedef struct enet_config_rx_accelerator
{
    bool isIpcheckEnabled;        /*!< Discard with wrong ip header checksum */
    bool isProtocolCheckEnabled;  /*!< Discard with wrong protocol checksum*/
    bool isMacCheckEnabled;       /*!< Discard with mac layer errors*/
    bool isPadRemoveEnabled;      /*!< Padding removal for short ip frames*/
    bool isShift16Enabled;        /*!< Rx fifo shift-16*/
} enet_config_rx_accelerator_t;

/*! @brief Define transmit fifo configuration*/
typedef struct enet_config_tx_fifo
{
    bool isStoreForwardEnabled;   /*!< Transmit fifo store and forward */
    uint8_t txFifoWrite;          /*!< Transmit fifo write */
    uint8_t txEmpty;              /*!< Transmit fifo section empty threshold*/
    uint8_t txAlmostEmpty;        /*!< Transmit fifo section almost empty threshold*/
    uint8_t txAlmostFull;         /*!< Transmit fifo section almost full threshold*/
} enet_config_tx_fifo_t;

/*! @brief Define receive fifo configuration*/
typedef struct enet_config_rx_fifo
{
    uint8_t rxFull;           /*!< Receive fifo section full threshold*/
    uint8_t rxAlmostFull;     /*!< Receive fifo section almost full threshold*/
    uint8_t rxEmpty;          /*!< Receive fifo section empty threshold*/
    uint8_t rxAlmostEmpty;    /*!< Receive fifo section almost empty threshold*/
} enet_config_rx_fifo_t;

/*******************************************************************************
 * API                              
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Reset enet module.
 *
 * @param instance The enet instance number.
 */
static inline void enet_hal_reset_ethernet(uint32_t instance)
{
   assert(instance < HW_ENET_INSTANCE_COUNT);
   
   BW_ENET_ECR_RESET(instance,1);
}

/*!
 * @brief Get enet status to check if the reset has completed.
 *
 * @param instance The enet instance number.
 * @return Current status of the reset operation.
 *         - true if enet reset completed.
 *         - false if enet reset has not completed.
 */
static inline bool enet_hal_is_reset_completed(uint32_t instance)
{
   assert(instance < HW_ENET_INSTANCE_COUNT);
   
   return (BR_ENET_ECR_RESET(instance) == 0);
}

/*!
 * @brief Set mac address.
 *
 * This interface is used to set the six-byte mac address of the enet interface.
 *
 * @param instance The enet instance number.
 * @param hwAddr The mac address pointer store for six bytes mac address.
 */
void enet_hal_set_mac_address(uint32_t instance, enetMacAddr hwAddr);

/*!
 * @brief Set the hardware addressing filtering to multicast group address.
 *
 * This interface is used to add the enet device to a multicast group address,
 * After joining the group , the mac will receive all frames with the group mac address.
 *
 * @param instance The enet instance number.
 * @param crcValue The crc value of the special address.
 * @param mode The operation for init/enable/disable the specified hardware address.
 */
void enet_hal_set_group_hashtable(uint32_t instance, uint32_t crcValue, enet_special_address_filter_t mode);

/*!
 * @brief Set the hardware addressing filtering to individual address.
 *
 * This interface is used to add a individual address to the hardware address
 * filter. Then the mac will receive all frames with this individual address as Destination address.
 *
 * @param instance The enet instance number.
 * @param crcValue The crc value of the special address.
 * @param mode The operation for init/enable/disable the specified hardware address.
 */
void enet_hal_set_individual_hashtable(uint32_t instance, uint32_t crcValue, enet_special_address_filter_t mode);

/*!
 * @brief Set the maximum receive buffer size and the maximum frame size.
 * 
 * @param instance The enet instance number.
 * @param maxBufferSize The maximum receive buffer size and it should not be smaller than 256.
 *        it should be evenly divisible by 16 and the maximum receive size should not be larger than 0x3ff0.
 * @param maxFrameSize The maximum receive Frame size, the reset value is 1518 or 1522 if the vlan tags are 
 *        supported. the Length is measured starting at DA and includes the CRC.
 */
static inline void enet_hal_set_rx_max_size(uint32_t instance, uint32_t maxBufferSize, uint32_t maxFrameSize)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    /* max buffer size must larger than 256 to minimize bus usage*/
    assert(maxBufferSize >= 256); 
    assert(maxFrameSize <= (BM_ENET_RCR_MAX_FL >> BP_ENET_RCR_MAX_FL));
	
    BW_ENET_RCR_MAX_FL(instance, maxFrameSize);
    HW_ENET_MRBR_SET(instance, (maxBufferSize & BM_ENET_MRBR_R_BUF_SIZE));
}

/*!
 * @brief Configure enet transmit FIFO.
 *
 * @param instance The enet instance number.
 * @param thresholdCfg The FIFO threshold configuration.
 */
void enet_hal_config_tx_fifo(uint32_t instance, enet_config_tx_fifo_t *thresholdCfg);

/*!
 * @brief Configure enet receive FIFO.
 *
 * @param instance The enet instance number.
 * @param thresholdCfg The FIFO threshold configuration.
 */
void enet_hal_config_rx_fifo(uint32_t instance, enet_config_rx_fifo_t *thresholdCfg);

/*!
 * @brief Initialize the start address for enet buffer descriptors.
 *
 * This interface is used to provide the beginning of the receive 
 * and transmit buffer descriptor queue in the external memory. The
 * input two address must be evenly divisible by 16.
 *
 * @param instance The enet instance number.
 * @param rxBdAddr The start address of receive buffer descriptors.
 * @param txBdAddr The start address of transmit buffer descriptors.
 */
void enet_hal_init_bd_address(uint32_t instance, uint32_t rxBdAddr, uint32_t txBdAddr);

/*!
 * @brief Initialize receive buffer descriptors.
 *
 * To make sure the uDMA will do the right data transfer after you active
 * with wrap flag and all the buffer descriptors should initalized with empty bit.
 * 
 * @param rxBds The current receive buffer descriptor.
 * @param buffer The data buffer on buffer descriptor.
 * @param isLastBd The flag to indicate the last receive buffer descriptor.
 */
void enet_hal_init_rxbds(void *rxBds, uint8_t *buffer, bool isLastBd);

/*!
 * @brief Update receive buffer descriptors.
 *
 * This interface mainly clear the status region and update the received
 * buffer descriptor to make sure this bd will be correctly used again.
 *
 * @param rxBds The current receive buffer descriptor.
 * @param data The data buffer address.
 * @param isbufferUpdate The data buffer update flag. When you want to update 
 *        the data buffer of the buffer descriptor please make sure this flag
 *        is set.
 */
void enet_hal_update_rxbds(void *rxBds, uint8_t *data, bool isbufferUpdate);

/*!
 * @brief Initialize transmit buffer descriptors.
 *
 * To make sure the uDMA will do the right data transfer after you active
 * with wrap flag. 
 * 
 * @param txBds The current transmit buffer descriptor.
 * @param isLastBd The last transmit buffer descriptor flag.
 */
void enet_hal_init_txbds(void *txBds, bool isLastBd);

/*!
 * @brief Update transmit buffer descriptors.
 *
 * This interface mainly clear the status region and update the transmit
 * buffer descriptor to make sure this bd will be correctly used again.
 * you should set isTxtsCfged when the transmit timestamp feature is required. 
 *
 * @param txBds The current transmit buffer descriptor.
 * @param buffer The data buffer on buffer descriptor.
 * @param length The data length on buffer descriptor.
 * @param isTxtsCfged The timestamp configure flag. The timestamp will be
 *        added to the transmit buffer descriptor when this flag is set.
 */
void enet_hal_update_txbds(void *txBds,uint8_t *buffer, uint16_t length, bool isTxtsCfged);

/*!
 * @brief Clear the context in transmit buffer descriptors.
 *
 * Clear the data,length, control and status region of the transmit buffer descriptor.
 *
 * @param curBd The current buffer descriptor.
 */
static inline void enet_hal_clear_txbds(void *CurBd)
{
    assert(CurBd);

    ((enet_bd_struct_t *)CurBd)->length = 0;                /* Set data length*/
    ((enet_bd_struct_t *)CurBd)->buffer = (uint8_t *)(NULL);/* Set data buffer*/
    ((enet_bd_struct_t *)CurBd)->control &= (kEnetTxBdWrap);/* Set control */
}

/*!
 * @brief Get control and status region of receive buffer descriptors.
 *
 * This interface can get the whole control and status region of the 
 * receive buffer descriptor. the enet_rx_bd_control_status_t enum type 
 * definition should be used if you want to get each status bit of
 * the control and status region.
 *
 * @param curBd The current receive buffer descriptor.
 * @return The control and status data on buffer descriptors.
 */
uint16_t enet_hal_get_rxbd_control(void *curBd);

/*!
 * @brief Get control and status region of transmit buffer descriptors.
 *
 * This interface can get the whole control and status region of the 
 * transmit buffer descriptor. the enet_tx_bd_control_status_t enum type 
 * definition should be used if you want to get each status bit of
 * the control and status region.
 *
 * @param curBd The current transmit buffer descriptor.
 * @return The extened control region of transmit buffer descriptor.
 */
uint16_t enet_hal_get_txbd_control(void *curBd);

/*!
 * @brief Get extended control region of receive buffer descriptors.
 *
 * This interface can get the whole control and status region of the 
 * receive buffer descriptor. the enet_rx_bd_control_extend_t enum type 
 * definition should be used if you want to get each status bit of
 * the control and status region.
 *
 * @param curBd The current receive buffer descriptor.
 * @param controlRegion The different control region.
 * @return The extend control region data of receive buffer desctiptor.
 *         - true when the control region is set 
 *         - false when the control region is not set.
 */
bool enet_hal_get_rxbd_control_extend(void *curBd,enet_rx_bd_control_extend_t controlRegion);
/*!
 * @brief Get extended control region of transmit buffer descriptors.
 *
 * This interface can get the whole control and status region of the 
 * transmit buffer descriptor. the enet_tx_bd_control_extend_t enum type 
 * definition should be used if you want to get each status bit of
 * the control and status region.
 *
 * @param curBd The current transmit buffer descriptor.
 * @return The extended control data.
 */
uint16_t enet_hal_get_txbd_control_extend(void *curBd);

/*!
 * @brief Get the data length of buffer descriptors.
 *
 * @param curBd The current buffer descriptor.
 * @return The data length of the buffer descriptor.
 */
static inline uint16_t enet_hal_get_bd_length(void *curBd)
{
    assert(curBd);
    return NTOHS(((enet_bd_struct_t *)curBd)->length);
}

/*!
 * @brief Get the buffer address of buffer descriptors.
 *
 * @param curBd The current buffer descriptor.
 * @return The buffer address of the buffer descriptor.
 */
static inline uint8_t* enet_hal_get_bd_buffer(void *curBd)
{
    assert(curBd);
    return  (uint8_t *)NTOHL(((uint32_t)(((enet_bd_struct_t *)curBd)->buffer)));
}	

/*!
 * @brief Get the timestamp of buffer descriptors.
 *
 * @param curBd The current buffer descriptor.
 * @return The time stamp of the frame in the buffer descriptor.
 *         Notice the timestamp of a frame is only set in the last  
 *         buffer descriptor of the frame. 
 */
static inline uint32_t enet_hal_get_bd_timestamp(void *curBd)
{
    assert(curBd);

    return NTOHL((uint32_t)(((enet_bd_struct_t *)curBd)->timestamp ));
}	

/*!
 * @brief Active receive buffer descriptor.
 *
 * This is used to active the receive buffer descriptor. The active buffer descriptor
 * should be done after the enet module is enabled otherwise the active operation will fail.
 *
 * @param instance The enet instance number.
 */
 static inline void enet_hal_active_rxbd(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_RDAR_RDAR(instance,1);
}

/*!
 * @brief Active transmit buffer descriptor.
 *
 * The active buffer descriptor should be done after the enet module is
 * enabled otherwise the active operation will fail.
 * 
 * @param instance The enet instance number.
 */
static inline void enet_hal_active_txbd(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_TDAR_TDAR(instance,1);
}

/*!
 * @brief Config the (R)MII of enet.
 *
 * @param instance The enet instance number.
 * @param mode The rmii or mii mode.
 * @param speed The speed of rmii.
 * @param duplex The full or half duplex mode.
 * @param isRxOnTxDisabled The Receive on transmit disable flag.
 * @param isLoopEnabled The loop enable flag.
 */
void enet_hal_config_rmii(uint32_t instance, enet_config_rmii_t mode, enet_config_speed_t speed, enet_config_duplex_t duplex, bool isRxOnTxDisabled,  bool isLoopEnabled);

/*!
 * @brief Config the mii of enet.
 *
 * This is used to set the mii interface between mac and phy, The miiSpeed is in 
 * fact a value controls the frequency of MDC relative to the internal module clock(InterClockSrc).
 * A value of zero in this parameter turns off MDC and leaves it in low voltage state.
 * Any non-zero value results in the MDC frequency MDC = InterClockSrc/((miiSpeed + 1)*2).
 * The desired MII(MDC) clock is 2.5MHZ(maximum), miiSpeed = InterClockSrc/(2*2.5MHZ), plus 1 to round up.
 *
 * @param instance The enet instance number.
 * @param miiSpeed The mii speed,This value is ranged from 0~0x3F.
 * @param isPreambleDisabled The preamble diabled flag.
 */
static inline void enet_hal_config_mii(uint32_t instance, uint32_t miiSpeed, bool isPreambleDisabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
	
    BW_ENET_MSCR_MII_SPEED(instance,miiSpeed);                /* mii speed set*/
    BW_ENET_MSCR_DIS_PRE(instance,isPreambleDisabled);       /* preamble is disabled*/
}

/*!
 * @brief Get the mii configuration status.
 *
 * This interface is usually be called to check the mii interface before 
 * the mac do write or read phy registers.
 *
 * @param instance The enet instance number.
 * @return The mii configuration status.
 *         - true if the mii has been configured. 
 *         - false if the mii has not been configured.
 */
static inline bool enet_hal_is_mii_enabled(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    return (HW_ENET_MSCR_RD(instance) & 0x7E)!= 0;	
}

/*!
 * @brief Read data from phy. 
 *
 * @param instance The enet instance number.
 * @return The data read from Phy.
 */
static inline uint32_t enet_hal_get_mii_data(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    return (uint32_t)BR_ENET_MMFR_DATA(instance);
}

/*!
 * @brief Set mii command.
 *
 * @param instance The enet instance number.
 * @param phyAddr The PHY address.
 * @param phyReg The PHY register.
 * @param operation The read or write operation.
 * @param data The data written to PHY.
 */
void enet_hal_set_mii_command(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, enet_mii_operation_t operation, uint32_t data);

/*!
 * @brief Enable/Disable enet module.
 *
 * @param instance The enet instance number.
 * @param isEnhanced The enhanced 1588 feature switch.
 * @param isEnabled The enet module enable switch.
 */
void enet_hal_config_ethernet(uint32_t instance, bool isEnhanced, bool isEnabled);

/*!
 * @brief Enable/Disable enet interrupt.
 *
 * @param instance The enet instance number.
 * @param source The interrupt sources. enet_interrupt_request_t enum types
 *        is recommended to be used as the interrupt sources.
 * @param isEnabled The interrupt enable switch.
 */
void enet_hal_config_interrupt(uint32_t instance, uint32_t source, bool isEnabled);

/*!
 * @brief Clear enet interrupt events. 
 *
 * @param instance The enet instance number.
 * @param source The interrupt source to be cleared. enet_interrupt_request_t 
 *        enum types is recommended to be used as the interrupt sources.
 */
static inline void enet_hal_clear_interrupt(uint32_t instance, uint32_t source)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
 
    HW_ENET_EIR_SET(instance,source);    
}

/*!
 * @brief Get enet interrupt status.
 *
 * @param instance The enet instance number.
 * @param source The interrupt sources. enet_interrupt_request_t 
 *        enum types is recommended to be used as the interrupt sources.
 * @return The event status of the interrupt source.
 *         - true if the interrupt event happened. 
 *         - false if the interrupt event has not happened.
 */
static inline bool enet_hal_get_interrupt_status(uint32_t instance, uint32_t source)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    return ((HW_ENET_EIR_RD(instance) & source) != 0);  
}

/*
 * @brief Enable/disble enet promiscuous mode.
 *
 * @param instance The enet instance number.
 * @param isEnabled The enable switch.
 */
static inline void enet_hal_config_promiscuous(uint32_t instance, bool isEnabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_RCR_PROM(instance,isEnabled);	
}

/*!
 * @brief Enable/disble clear MIB counter. 
 *
 * @param instance The enet instance number.
 * @param isEnabled The enable switch.
 */
static inline void enet_hal_clear_mib(uint32_t instance, bool isEnabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_MIBC_MIB_CLEAR(instance,isEnabled);

}

/*!
 * @brief Set Enable/disble MIB block. 
 *
 * @param instance The enet instance number.
 * @param isEnabled The enable flag.
 */
static inline void enet_hal_config_mib(uint32_t instance, bool isEnabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_MIBC_MIB_DIS(instance,!isEnabled);

}

/*!
 * @brief Get MIB idle status. 
 *
 * @param instance The enet instance number.
 * @return true if in mib idle and mib is not updating else false.
 */
static inline bool enet_hal_get_mib_status(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);
    
    return BR_ENET_MIBC_MIB_IDLE(instance);
}

/*!
 * @brief Set transmit accelerator.
 *
 * @param instance The enet instance number.
 * @param txCfgPtr The transmit accelerator configuration .
 */
void enet_hal_config_tx_accelerator(uint32_t instance, enet_config_tx_accelerator_t *txCfgPtr);

/*!
 * @brief Set receive accelerator. 
 *
 * @param instance The enet instance number.
 * @param rxCfgPtr The receive accelerator configuration .
 */
void enet_hal_config_rx_accelerator(uint32_t instance, enet_config_rx_accelerator_t *rxCfgPtr);

/*!
 * @brief Initialize 1588 timer.
 *
 * This interface mainly initialize 1588 context structure. 
 * do intialize 1588 parameters according to users configuration structure.
 *
 * @param instance The enet instance number.
 * @param ptpCfg The 1588 timer configuration.
 */
void enet_hal_init_ptp_timer(uint32_t instance, enet_config_ptp_timer_t *ptpCfgPtr);

/*!
 * @brief Start or stop the 1588 timer.
 *
 * This is the acutal interface for ptp timer intialize. Set the right timer source
 * the timer increase value and the timer wrap period.
 *
 * @param instance The enet instance number.
 * @param isEnabled The 1588 timer start/stop switch.
 */
static inline void enet_hal_start_ptp_timer(uint32_t instance, uint32_t isEnabled)
{
    assert(instance < HW_ENET_INSTANCE_COUNT); 

    BW_ENET_ATCR_EN(instance,isEnabled);                          
}

/*!
 * @brief Restart the 1588 timer.
 *
 * Restart ptp timer will clear all ptp timer counters to zero.
 *
 * @param instance The enet instance number.
 */
static inline void enet_hal_restart_ptp_timer(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT); 

    BW_ENET_ATCR_RESTART(instance,1);                          
}

/*!
 * @brief Adjust the 1588 timer.
 *
 * Adjust 1588 timer accodring to configured correction increase and correcion period.
 *
 * @param instance The enet instance number.
 * @param inceaseCorrection The increase correction for 1588 timer.
 * @param periodCorrection The period correction for 1588 timer.
 */
static inline void enet_hal_adjust_ptp_timer(uint32_t instance, uint32_t increaseCorrection, uint32_t periodCorrection)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_ATINC_INC_CORR(instance,(increaseCorrection & (ENET_ATINC_INC_CORR_MASK>>ENET_ATINC_INC_CORR_SHIFT)));      /* set correction for ptp timer increase*/
    BW_ENET_ATCOR_COR(instance,periodCorrection);             /* set correction for ptp timer period	*/
}

/*!
 * @brief Initialize the 1588 timer channel.
 *
 * @param instance The enet instance number.
 * @Param channel The 1588 timer channel number. 
 * @param mode The compare or capture mode for 1588 timer channel.
 */
static inline void enet_hal_init_timer_channel(uint32_t instance, uint32_t channel, enet_timer_channel_mode_t mode)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_TCSRn_TMODE(instance,channel,mode);
    BW_ENET_TCSRn_TIE(instance,channel, 1);    
}

/*!
 * @brief Set the compare value for 1588 timer channel.
 *
 * @param instance The enet instance number.
 * @Param channel The 1588 timer channel number. 
 * @param compareValue The compare value for 1588 timer channel.
 */
static inline void enet_hal_set_timer_channel_compare(uint32_t instance, uint32_t channel, uint32_t compareValue)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    HW_ENET_TCCRn_SET(instance,channel, compareValue);   
}

/*!
 * @brief Get 1588 timer channel status.
 *
 * @param instance The enet instance number.
 * @param channel The 1588 timer channel number. 
 * @return is the compare happened.
 */
static inline bool enet_hal_get_timer_channel_status(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    return BR_ENET_TCSRn_TF(instance,channel);  
}

/*!
 * @brief Clear 1588 timer channel flag.
 *
 * @param instance The enet instance number.
 * @param channel The 1588 timer channel number. 
 */
static inline void enet_hal_clear_timer_channel_flag(uint32_t instance, uint32_t channel)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_TCSRn_TF(instance,channel, 1);                /* clear interrupt flag*/
    HW_ENET_TGSR_SET(instance,(1U << channel));            /* clear channel flag*/
}

/*!
 * @brief Set capture command to 1588 timer.
 *
 * This is used before reading the current time register.
 *
 * @param instance The enet instance number.
 */
static inline void enet_hal_set_timer_capture(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    BW_ENET_ATCR_CAPTURE(instance,1); 
}

/*!
 * @brief Set 1588 timer.
 *
 * @param instance The enet instance number.
 * @param nanSecond The nanosecond set to 1588 timer.
 */
static inline void enet_hal_set_current_time(uint32_t instance, uint32_t nanSecond)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    HW_ENET_ATVR_SET(instance,nanSecond);
}

/*!
 * @brief Get time from 1588 timer.
 *
 * @param instance The enet instance number.
 * @return the current time from 1588 timer.
 */
static inline uint32_t enet_hal_get_current_time(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    return HW_ENET_ATVR_RD(instance);   
}

/*!
 * @brief Get the transmit timestamp.
 *
 * @param instance The enet instance number.
 * @return The timestamp of the last transmitted frame.
 */
static inline uint32_t enet_hal_get_tx_timestamp(uint32_t instance)
{
    assert(instance < HW_ENET_INSTANCE_COUNT);

    return HW_ENET_ATSTMP_RD(instance);
}

/*!
 * @brief Get the receive timestamp.
 *
 * @param instance The enet instance number.
 * @return The tiemstamp of the receive frame.
 */
static inline uint32_t enet_hal_get_rx_timestamp(void *curRxBd)
{
    assert(curRxBd);
	
    return NTOHL((uint32_t)(((enet_bd_struct_t *)curRxBd)->timestamp));    /* get the timestamp on buffer descriptor*/
}

/*!
 * @brief Get transmit buffer descriptor timestamp flag.
 *
 * @param curBd The enet transmit buffer descriptor.
 * @return true if timestamp region is set else false.
 */
bool enet_hal_get_txbd_timestamp_flag(void *curBd);

/*!
 * @brief Get the buffer descritor timestamp.
 *
 * @param null
 * @return The the size of buffer descriptor.
 */
static inline uint32_t enet_hal_get_bd_size(void)
{
    return sizeof(enet_bd_struct_t);
}

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/
#endif /*!< __FSL_ENET_HAL_H__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

