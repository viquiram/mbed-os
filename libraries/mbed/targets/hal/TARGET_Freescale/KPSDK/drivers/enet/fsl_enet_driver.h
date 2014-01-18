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
#ifndef __FSL_ENET_DRIVER_H__
#define __FSL_ENET_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_enet_hal.h"
#include "fsl_os_abstraction.h"
/*! 
 * @addtogroup enet_driver
 * @{
 */

/*******************************************************************************
 * Definitions

 ******************************************************************************/
/*! @brief Define the statistic enable macro*/
#define ENET_ENABLE_DETAIL_STATS     0

/*! @brief Define alignment operation*/
#define ENET_ALIGN(x,align)        ((unsigned int)((x) + ((align)-1)) & (unsigned int)(~(unsigned int)((align)- 1)))

#if FSL_FEATURE_ENET_SUPPORT_PTP
/*! @brief Define ptp ioctl macro*/
typedef enum _enet_ptp_ioctl
{
    kEnetPtpGetRxTimestamp = 0,    /*!< enet ptp get receive timestamp*/
    kEnetPtpGetTxTimestamp,        /*!< enet ptp get transmit timestamp*/
    kEnetPtpGetCurrentTime,        /*!< enet ptp get current time*/
    kEnetPtpSetCurrentTime,        /*!< enet ptp set current time*/
    kEnetPtpFlushTimestamp,        /*!< enet ptp flush timestamp*/
    kEnetPtpCorrectTime,           /*!< enet ptp time Correction*/
    kEnetPtpSendEthernetPtpV2,     /*!< enet ptpv2 send ethernet frame*/
    kEnetPtpReceiveEthernetPtpV2   /*!< enet ptpv2 receive with ethernet frame*/
}

/*! @brief Define ptpv2 message buffer number*/
typedef enum _enet_ptp_l2packet_buffer_number
{
    kEnetPtpL2bufferNumber = 10  /*!< ptp layer2 frame buffer number*/
}enet_ptp_l2packet_buffer_number_t;

/*! @brief Define ENET's ptp message related constant*/
typedef enum _enet_ptp_event_type
{
     kEnetPtpSourcePortIdLen = 10,  /*!< Ptp message sequence id length*/
     kEnetPtpEventMsgType = 3,      /*!< Ptp event message type*/
     kEnetPtpEventPort = 319,       /*!< Ptp event port number*/
     kEnetPtpGnrlPort = 320         /*!< Ptp general port number*/
}enet_ptp_event_type_t;

/*! @brief Define all enet ptp content offest in IPV4 ptp UDP/IP multicast message*/
typedef enum _enet_ipv4_ptp_content_offset
{
    kEnetPtpIpVersionOffset = 0xe,   /*!< Ipv4 ptp message ip version offset*/
    kEnetPtpUdpProtocolOffset = 0x17,/*!< Ipv4 ptp message udp protocol offset*/
    kEnetPtpUdpPortOffset = 0x24,    /*!< Ipv4 ptp message udp port offset*/
    kEnetPtpUdpMsgTypeOffset = 0x2a, /*!< Ipv4 ptp message udp message type offset*/
    kEnetPtpUdpVersionoffset = 0x2b, /*!< Ipv4 ptp message udp version offset*/
    kEnetPtpUdpClockIdOffset = 0x3e, /*!< Ipv4 ptp message udp clock id offset*/
    kEnetPtpUdpSequenIdOffset = 0x48,/*!< Ipv4 ptp message udp sequence id offset*/
    kEnetPtpUdpCtlOffset = 0x4a      /*!< Ipv4 ptp message udp control offset*/
}enet_ipv4_ptp_content_offset_t;

/*! @brief Define all enet ptp content offest in IPV6 ptp UDP/IP multicast message*/
typedef enum _enet_ipv6_ptp_content_offset
{
    kEnetPtpIpv6UdpProtocolOffset = 0x14,  /*!< Ipv6 ptp message udp protocol offset*/
    kEnetPtpIpv6UdpPortOffset = 0x38,      /*!< Ipv6 ptp message udp port offset*/
    kEnetPtpIpv6UdpMsgTypeOffset = 0x3e,   /*!< Ipv6 ptp message udp message type offset*/
    kEnetPtpIpv6UdpVersionOffset = 0x3f,   /*!< Ipv6 ptp message udp version offset*/
    kEnetPtpIpv6UdpClockIdOffset = 0x52,   /*!< Ipv6 ptp message udp clock id offset*/
    kEnetPtpIpv6UdpSequenceIdOffset = 0x5c,/*!< Ipv6 ptp message udp sequence id offset*/
    kEnetPtpIpv6UdpCtlOffset = 0x5e        /*!< Ipv6 ptp message udp control offset*/
}enet_ipv6_ptp_content_offset_t;

/*! @brief Define all enet ptp content offest in ptp Layer2 ETHERENT message*/
typedef enum _enet_ethernet_ptp_content_offset
{
    kEnetPtpEtherPktTypeOffset = 0x0c,   /*!< Ptpv2 message etherent packet type offset*/
    kEnetPtpEtherMsgTypeOffset = 0x0e,   /*!< Ptpv2 message etherent message type offset*/
    kEnetPtpEtherVersionOffset = 0x0f,   /*!< Ptpv2 message etherent version type offest*/
    kEnetPtpEtherClockIdOffset = 0x22,   /*!< Ptpv2 message ethernet clock id offset*/
    kEnetPtpEtherSequenceIdOffset = 0x2c,/*!< Ptpv2 message ethernet sequence id offset*/
    kEnetPtpEtherCtlOffset = 0x2e        /*!< Ptpv2 message ethernet control offset*/
}enet_ethernet_ptp_content_offset_t;

/*! @brief Define the 1588 timer parameters*/
typedef enum enet_ptp_timer_wrap_period
{
    kEnetPtpAtperVaule = 1000000000, /*!< Ptp timer wrap aroud of one second */
    kEnetBaseIncreaseUnit = 2        /*!< Ptp timer adjust clock increase vlaue 2*/
}enet_ptp_timer_wrap_period_t;
#endif

/*! @brief Define interrupt source index for interrupt vector change table*/
typedef enum _enet_interrupt_number
{
    kEnetTstimerInt = 0, /*!< Timestamp interreupt*/
    kEnetTsAvailInt, /*!< Ts-avail interreupt*/
    kEnetWakeUpInt,  /*!< Wakeup interreupt*/
    kEnetPlrInt,     /*!< Plr interreupt*/
    kEnetUnInt,      /*!< Un interreupt*/
    kEnetRlInt,      /*!< Rl interreupt*/
    kEnetLcInt,      /*!< Lc interreupt*/
    kEnetEberrInt,   /*!< Eberr interreupt*/
    kEnetMiiInt,     /*!< Mii interreupt*/
    kEnetRxbInt ,    /*!< Receive byte interreupt*/
    kEnetRxfInt,    /*!< Receive frame interreupt*/
    kEnetTxbInt,    /*!< Transmit byte interreupt*/
    kEnetTxfInt,    /*!< Transmit frame interreupt*/
    kEnetGraInt,    /*!< Gra interreupt*/
    kEnetBabtInt,   /*!< Babt interreupt*/
    kEnetBabrInt,   /*!< Babr interrupt*/
    kEnetIntNum     /*!< Interreupt number*/
}enet_interrupt_number_t;

/*! @brief Define enet main constant*/
typedef enum _enet_frame_max
{
    kEnetMaxTimeout = 0x10000,    /*!< Max timeout*/
    kEnetMaxFrameSize = 1518,     /*!< Max frame size*/
    kEnetMaxFrameVlanSize = 1522, /*!< Max vlan frame size*/
    kEnetMaxFrameDateSize = 1500, /*!< Max frame data size*/
    kEnetMaxFrameBdNumbers = 7,   /*!< Max buffer descriptor numbers of a frame*/
    kEnetFrameFcsLen = 4,         /*!< Fcs length*/
}enet_frame_max_t;

/*! @brief Define the crc data for hash value calculation*/
typedef enum _enet_crc_parameter
{
    kEnetCrcData = 0xFFFFFFFFU,  /*!< Crc-32 max data */
    kEnetCrcOffset = 8,          /*!< Crc-32 offset2*/
    kEnetCrcMask1 = 0x3F         /*!< Crc-32 mask*/
}enet_crc_parameter_t;

/*! @brief Define enet protocol type and main parameters*/
typedef enum _enet_protocol_type
{
    kEnetProtocolIeee8023 = 0x88F7,  /*!< Packet type etherent ieee802.3*/
    kEnetProtocolIpv4 = 0x0800,      /*!< Packet type ipv4*/
    kEnetProtocolIpv6 = 0x86dd,      /*!< Packet type ipv6*/
    kEnetProtocol8021QVlan = 0x8100, /*!< Packet type vlan*/
    kEnetPacketUdpVersion = 0x11,    /*!< Udp protocol type*/
    kEnetPacketIpv4Version = 0x4,    /*!< Packet ip version ipv4*/
    kEnetPacketIpv6Version = 0x6     /*!< Packet ip version IPV6*/
}enet_protocol_type_t;

/*! @brief Define multicast group structure for ENET device */
typedef struct enet_multicast_group
{
    enetMacAddr groupAdddr;        /*!< Multicast group address*/
    uint32_t hash;                 /*!< Hash value of the multicast group address*/
    struct enet_multicast_group *next; /*!< Pointer of next group structure*/
    struct enet_multicast_group *prv;  /*!< Pointer of the previou structure*/
} enet_multicast_group_t;

/*! @brief Define basic configuration structure for ENET device*/
typedef struct enet_mac_config
{
    uint16_t rxBufferSize;  /*!< Recieve buffer size*/
    uint16_t rxBufferNumber;/*!< Receive buffer number,should be larger than rx bd number when the buffer size is
    not the maximum frame size. should be at least equal to rx bd number when buffer size equal to maximum size*/
	uint16_t rxLargeBufferNumber; /*!< Receive large buffer number only needs when the bd size is smaller than the maximum frame length*/
    uint16_t rxBufferAlignment;  /*!< Receive buffer alignment*/
    uint16_t txLargeBufferNumber;/*!< Transmit large bufer number*/
    uint16_t txBufferAlignment;  /*!< Transmit buffer alignment*/
    uint16_t rxBdNumber;    /*!< Receive buffer descriptor number*/
    uint16_t txBdNumber;    /*!< Transmit buffer descriptor number*/
    uint16_t bdAlignment;   /*!< Buffer descriptor alignment*/
    enetMacAddr macAddr;    /*!< Mac hardware address*/
    enet_config_rmii_t rmiiCfgMode;/*!< Rmii configure mode*/
    enet_config_speed_t speed;     /*!< Speed configuration*/
    enet_config_duplex_t duplex;   /*!< Duplex configuration*/
    bool isLoopEnabled;   /*!< Switcher to enable mac loopback mode*/
    bool isPromiscEnabled;/*!< Switcher to enable promiscuous*/
    bool isTxAccelEnabled;/*!< Switcher to enable transmit accelerator*/
    bool isRxAccelEnabled;/*!< Switcher to enable receive accelerator*/
    bool isStoreAndFwEnabled;      /*!< Switcher to enable store and forward*/
    enet_config_rx_accelerator_t rxAcceler; /*!< Receive accelerator configure*/
    enet_config_tx_accelerator_t txAcceler; /*!< Transmit accelerator configure*/
    bool isVlanEnabled;    /*!< Switcher to enable vlan frame*/
    bool isPhyAutoDiscover;/*!< Swithcer to use phy auto discover*/
    uint32_t miiClock;     /*!< Mii speed*/
#if FSL_FEATURE_ENET_SUPPORT_PTP
    uint16_t ptpRingBufferNumber; /*!< Ptp ring buffer number*/
    bool isSlaveModeEnabled;      /*!< Ptp timer configuration*/
#endif
}enet_mac_config_t;

/*! @brief Define basic configuration for PHY*/
typedef struct enet_phy_config
{
    uint8_t phyAddr;    /*!< Phy address*/
    bool isLoopEnabled; /*!< Switcher to enable phy loop mode*/
}enet_phy_config_t;

#if FSL_FEATURE_ENET_SUPPORT_PTP
/*! @brief Define enet mac ptp timestamp struture*/
typedef struct enet_mac_ptp_time
{
    uint64_t second;     /*!< Second*/
    uint32_t nanosecond; /*!< Nanosecond*/
}enet_mac_ptp_time_t;

/*! @brief Define enet ptp timer drift struture*/
typedef struct enet_ptp_drift
{
    int32_t drift;    /*!< Drift for ptp timer to do adjust*/
}enet_ptp_drift_t;

/*! @brief Define enet mac ptp time parameter*/
typedef struct enet_ptp_master_time_data
{
    uint8_t MasterPtpInstance;/*!< Ptp master timer instance*/
    uint64_t second;          /*!< Ptp master timer second */
}enet_ptp_master_time_data_t;

/*! @brief Define structure for enet ptp message data and timestamp data*/
typedef struct enet_mac_ptp_ts_data
{
    uint8_t version;              /*!< Ptp version*/
    uint8_t sourcePortId[kEnetPtpSourcePortIdLen];/*!< Ptp source port ID*/
    uint16_t sequenceId;          /*!< Ptp sequence ID*/
    uint8_t messageType;          /*!< Ptp message type*/
    enet_mac_ptp_time_t timeStamp;/*!< Ptp timestamp*/
}enet_mac_ptp_ts_data_t;

/*! @brief Define enet ptp ring buffer structure for ptp message timestamp store*/
typedef struct enet_mac_ptp_ts_ring
{
    uint32_t front; /*!< The first index of the ring*/
    uint32_t end;   /*!< The end index of the ring*/
    uint32_t size;  /*!< The size of the ring*/
    enet_mac_ptp_ts_data_t *ptpTsDataPtr;/*!< Ptp message data structure*/
}enet_mac_ptp_ts_ring_t;

/*! @brief Define enet packet for ptp version2 message using layer2 etherent frame*/
typedef struct enet_ptp_l2packet
{
    uint8_t packet[kEnetMaxFrameDateSize]; /*!< Buffer for ptpv2 message*/
    uint16_t length;                       /*!< Ptp message length*/
}enet_ptp_l2packet_t;

/*! @brief Define enet ptpv2 packet queue using layer2 etherent frame*/
typedef struct enet_ptp_l2queue
{
    enet_ptp_l2packet_t l2Packet[kEnetPtpL2bufferNumber]; /*!< Ptp layer2 packet*/
    uint16_t writeIdex;          /*!< Queue write index*/
    uint16_t readIdx;            /*!< Queue read index*/
}enet_ptp_l2queue_t;

/*! @brief Define enet ptp layer2 ethernet frame structure*/
typedef struct enet_ptp_l2_etherent
{
    uint8_t *ptpMsg;     /*!< Ptp message*/
    uint16_t length;     /*!< Length of ptp message*/
    enetMacAddr hwAddr;  /*!< Destination hardware address*/
}enet_ptp_l2_ethernet_t;

/*! @brief Define enet ptp buffer structure for all 1588 data*/
typedef struct enet_private_ptp_buffer
{
    enet_mac_ptp_ts_ring_t rxTimeStamp;/*!< Data structure for receive message*/
    enet_mac_ptp_ts_ring_t txTimeStamp;/*!< Data structure for transmit timestamp*/
    enet_ptp_l2queue_t *l2QueuePtr;    /*!< Data structure for layer2 etherent queue*/
    uint64_t masterSecond;             /*!< Ptp time second when it's master time*/
}enet_private_ptp_buffer_t;
#endif

/*! @brief Define enet header struct */
typedef struct enet_etherent_header
{
    enetMacAddr destAddr;  /*!< Destination address */
    enetMacAddr sourceAddr;/*!< Source address*/
    uint16_t type;         /*!< Protocol type*/
}enet_etherent_header_t;

/*! @brief Define enet vlan frame header structure */
typedef struct enet_8021vlan_header
{
    enetMacAddr destAddr;  /*!< Destination address */
    enetMacAddr sourceAddr;/*!< Source address*/
    uint16_t tpidtag;      /*!< Enet 8021tag header tag region*/
    uint16_t othertag;     /*!< Enet 8021tag header type region*/
    uint16_t type;         /*!< Protocol type*/
}enet_8021vlan_header_t;

/*! @brief Define enet mac context structure for buffer address,buffer descriptor address etc*/
typedef struct enet_mac_context
{
    uint8_t *rxBufferPtr;   /*!< Receive buffer pointer*/
	uint8_t *rxLargeBufferPtr; /*!< Receive large buffer descriptor*/
    uint8_t *txBufferPtr;   /*!< Transmit buffer pointer*/
    uint8_t *rxBdBasePtr;   /*!< Receive buffer descriptor base address pointer*/
    uint8_t *rxBdCurPtr;    /*!< Current receive buffer descriptor pointer*/
	uint8_t *rxBdDirtyPtr;  /*!< Receive dirty buffer descriptor*/
    uint8_t *txBdBasePtr;   /*!< Transmit buffer descriptor base address pointer*/
    uint8_t *txBdCurPtr;    /*!< Current transmit buffer descriptor pointer*/
    uint8_t *txBdDirtyPtr;  /*!< Last cleaned transmit buffer descriptor pointer*/
    bool  isTxFull;         /*!< Transmit buffer descriptor full*/
	bool  isRxFull;         /*!< Reveive buffer descriptor full*/
    uint32_t bufferdescSize;         /*!< Enet buffer descriptor size*/
	uint16_t rxBufferSizeAligned;      /*!< Receive buffer alignment size*/
#if FSL_FEATURE_ENET_SUPPORT_PTP
    enet_private_ptp_buffer_t *privatePtpPtr;/*!< Ptp private buffer pointer*/
#endif
}enet_mac_context_t;

/*! @brief Define ENET packets statistic structure*/
typedef struct enet_mac_stats
{
    uint32_t statsRxTotal;   /*!< Total number of receive packets*/
    uint32_t statsRxMissed;  /*!< Total number of receive packets*/
    uint32_t statsRxDiscard; /*!< Receive discarded with error */
    uint32_t statsRxError;   /*!< Receive discarded with error packets*/
    uint32_t statsTxTotal;   /*!< Total number of transmit packets*/
    uint32_t statsTxMissed;  /*!< Transmit missed*/
    uint32_t statsTxDiscard; /*!< Transmit discarded with error */
    uint32_t statsTxError;   /*!< Transmit error*/
    uint32_t statsRxAlign;   /*!< Receive non-octet alignment*/
    uint32_t statsRxFcs;     /*!< Receive crc error*/
    uint32_t statsRxTruncate;/*!< Receive truncate*/
    uint32_t statsRxLengthGreater;  /*!< Receive length greater than rcr[MAX_FL] */
    uint32_t statsRxCollision;      /*!< Receive collision*/
    uint32_t statsRxOverRun;        /*!< Receive over run*/
    uint32_t statsTxOverFlow;       /*!< Transmit overflow*/
    uint32_t statsTxLateCollision;  /*!< Transmit late collision*/
    uint32_t statsTxExcessCollision;/*!< Transmit excess collision*/
    uint32_t statsTxUnderFlow;      /*!< Transmit under flow*/
    uint32_t statsTxLarge;          /*!< Transmit large packet*/
    uint32_t statsTxSmall;          /*!< Transmit small pacaket*/
}enet_stats_t;

/*! @brief Define enet mac packet buffer structure*/
typedef struct enet_mac_packet_buffer
{
    uint8_t *data;
    uint16_t length;
}enet_mac_packet_buffer_t;

/*! @brief Define enet device data structure for ENET*/
typedef struct enet_dev_if
{
    struct enet_dev_if *next; /*!< Next device structure address*/
    void *netIfPtr;           /*!< Store the connected  upper layyer instructure*/
    enet_multicast_group_t *multiGroupPtr; /*!< Multicast group chain*/
    uint32_t deviceNumber;    /*!< Device number*/
    bool isInitialized;       /*!< Device initialized*/
    uint16_t maxFrameSize;  /*!< Mac maximum frame size*/
    enet_mac_config_t *macCfgPtr;/*!< Mac configuration structure*/
    enet_phy_config_t *phyCfgPtr;/*!< Phy configuration structure*/
    const struct enet_mac_api *macApiPtr;   /*!< Mac application interface structure*/
    void *phyApiPtr;             /*!< Phy application interface structure*/
    enet_mac_context_t *macContextPtr; /*!< Mac context pointer*/
#if ENET_ENABLE_DETAIL_STATS
    enet_stats_t stats;                /*!< Packets statistic*/
#endif
    event_object_t enetReceiveSync;     /*!< Receive sync signal*/
    lock_object_t enetContextSync;     /*!< Sync signal*/
}enet_dev_if_t;

/*! @brief Define basic application for ENET device*/
typedef struct enet_mac_api
{
    uint32_t (* enet_mac_init)(enet_dev_if_t * enetIfPtr);/*!< Mac initialize interface*/
    uint32_t (* enet_mac_close)(enet_dev_if_t * enetIfPtr);/*!< Mac close interface*/
    uint32_t (* enet_mac_send)(enet_dev_if_t * enetIfPtr, uint8_t *packet, uint32_t size);/*!< Mac send packets*/
    uint32_t (* enet_mac_receive)(enet_dev_if_t * enetIfPtr, enet_mac_packet_buffer_t *packBuffer);/*!< Mac receive interface*/
    uint32_t (* enet_mii_read)(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, uint32_t *dataPtr);/*!< Mii read phy*/
    uint32_t (* enet_mii_write)(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, uint32_t data);/*!< Mii write phy*/
    uint32_t (* enet_add_multicast_group)(uint32_t instance, enet_multicast_group_t *multiGroupPtr, uint8_t *groupAddr);/*!< Add multicast group*/
    uint32_t (* enet_leave_multicast_group)(uint32_t instance, enet_multicast_group_t *multiGroupPtr, uint8_t *groupAddr);/*!< Leave multicast group*/
#if FSL_FEATURE_ENET_SUPPORT_PTP
    uint32_t (* enet_ptp_ioctl)(enet_dev_if_t * enetIfPtr, uint32_t commandId, void *inOutPtr);/*!< Ptp ioctl*/
#endif
}enet_mac_api_t;

/*******************************************************************
* Global variables
 
***********************************************************************/
extern const enet_mac_api_t g_enetMacApi;

/*******************************************************************************
 * API 
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! 
  * @name ENET Driver
  * @{
  */


#if FSL_FEATURE_ENET_SUPPORT_PTP
/*!
 * @brief Initialize the ENET ptp buffer ring with basic configuration.
 *
 * @param ptpTsRingPtr The pointer to basic ptp configuration structure.
 * @return The execution status.
 */
uint32_t enet_ptp_ring_init(enet_mac_ptp_ts_ring_t *ptpTsRingPtr);

/*!
 * @brief Initialize the ENET ptp context structure with basic configuration.
 *
 * @param enetIfPtr The pointer to enet basic configuration structure.
 * @return The execution status.
 */
uint32_t enet_ptp_init(enet_dev_if_t *enetIfPtr);
  
/*!
 * @brief Initialize the ENET ptp timer with basic configuration.
 *
 * After ptp start, the 1588 timer starts run. If you want the 1588 timer
 * the slave one please don't forget to enable the isSlaveEnabled flag.
 *
 * @param instance The enet instance number.
 * @param ptpCfgPtr The pointer to basic ptp timer configuration structure.
 * @return The execution status.
 */
uint32_t enet_ptp_start(uint32_t instance, bool isSlaveEnabled);

/*!
 * @brief Parse the ENET packet. 
 *
 * Parse the enet message and Check if it is a ptp message. If it is a ptp message
 * the message will be stored at ptp information structure. Message parsing 
 * decide if timesstamp processing will be done after that.
 *
 * @param packet The enet packet.
 * @param ptpTsPtr The pointer to ptp data structure.
 * @param isPtpMsg The ptp message flag.
 * @return The execution status.
 */
uint32_t enet_ptp_parse(uint8_t *packet, enet_mac_ptp_ts_data_t *ptpTsPtr, bool *isPtpMsg);

/*!
 * @brief Get current value of ENET ptp time.
 *
 * @param ptpTimerPtr The ptp timer structure.
 * @return The execution status.
 */
uint32_t enet_ptp_get_time(enet_mac_ptp_time_t *ptpTimerPtr);

/*!
 * @brief Set current value of ENET ptp time.
 *
 * @param ptpTimerPtr The ptp timer structure.
 * @return The execution status.
 */
uint32_t enet_ptp_set_time(enet_mac_ptp_time_t *ptpTimerPtr);

/*!
 * @brief Adjust enet ptp time.
 *
 * @param instance The enet instance number.
 * @param drift The ptp timer drift value.
 * @return The execution status.
 */
uint32_t enet_ptp_correction_time(uint32_t instance, int32_t drift);


/*!
 * @brief Store transmit timestamp.
 *
 * @param ptpBuffer The ptp buffer pointer.
 * @param bdPtr The current transmit buffer descriptor.
 * @return The execution status.
 */	
uint32_t enet_ptp_store_tx_timestamp(enet_private_ptp_buffer_t *ptpBuffer,void *bdPtr);

/*!
 * @brief Store receive timestamp.
 *
 * @param ptpBuffer The ptp buffer pointer.
 * @param packet The current receive packet.
 * @param bdPtr The current receive buffer descriptor.
 * @return The execution status.
 */
uint32_t enet_ptp_store_rx_timestamp(enet_private_ptp_buffer_t *ptpBuffer, uint8_t *packet, void *bdPtr);

/*!
 * @brief Initialize buffer queue for ptp layer2 etherent packets.
 *
 * @param ptpBuffer The ptp buffer pointer.
 * @return The execution status.
 */
uint32_t enet_ptp_l2queue_init(enet_private_ptp_buffer_t *ptpBuffer);

/*!
 * @brief Add the ptp layer2 etherent packet to the ptp ethernet paket queue.
 *
 * @param enetIfPtr The enet context structure pointer.
 * @param packet The packet buffer pointer.
 * @param length The packet length. 
 * @return The execution status.
 */
uint32_t enet_ptp_service_l2packet(enet_dev_if_t * enetIfPtr, uint8_t *packet, uint16_t length);

/*!
 * @brief Send the ptp layer2 etherent packet to net.
 *
 * @param enetIfPtr The enet context structure.
 * @param paramPtr The buffer from upper layer. 
 * @return The execution status.
 */
uint32_t enet_ptp_send_l2packet(enet_dev_if_t * enetIfPtr, void *paramPtr);

/*!
 * @brief Receive the ptp layer2 etherent packet from the net.
 *
 * @param enetIfPtr The enet context structure.
 * @param paramPtr The buffer receive from net and will send to upper layer. 
 * @return The execution status.
 */
uint32_t enet_ptp_receive_l2packet(enet_dev_if_t * enetIfPtr,void *paramPtr);

/*!
 * @brief The function provides the handler for 1588 stack to do ptp ioctl.
 *
 * @param enetIfPtr The enet context structure.
 * @param commandId The command id.
 * @param inOutPtr The data buffer. 
 * @return The execution status.
 */
uint32_t enet_ptp_ioctl(enet_dev_if_t * enetIfPtr, uint32_t commandId, void *inOutPtr);

/*!
 * @brief Stop enet ptp timer.
 *
 * @param instance The enet instance number.
 * @return The execution status.
 */
 uint32_t enet_ptp_stop(uint32_t instance);

/*!
 * @brief Check if the ptp ring buffer is full.
 *
 * @param ptpTsRingPtr The enet ptp timestamp ring.
 * @return True if the ptp ring buffer is full else false.
 */
bool enet_ptp_ring_is_full(enet_mac_ptp_ts_ring_t *ptpTsRingPtr);

/*!
 * @brief Update the latest ring buffers.
 *
 * Add the ptp message data to the ptp ring buffers and increase the 
 * ptp ring buffer index.
 *
 * @param ptpTsRingPtr The enet ptp timestamp ring.
 * @param data The ptp data buffer.
 * @return The execution status.
 */
uint32_t enet_ptp_ring_update(enet_mac_ptp_ts_ring_t *ptpTsRingPtr, enet_mac_ptp_ts_data_t *data);

/*!
 * @brief Search the element in ring buffers with the message ID and Clock Id.
 *
 * @param ptpTsRingPtr The enet ptp timestamp ring.
 * @param data The ptp data buffer.
 * @return The execution status.
 */
uint32_t enet_ptp_ring_search(enet_mac_ptp_ts_ring_t *ptpTsRingPtr, enet_mac_ptp_ts_data_t *data);

/*!
 * @brief Calculate enet ptp ring buffer index.
 *
 * @param size The ring size.
 * @param curIdx The current ring index.
 * @param offset The offset index.
 * @return The execution status.
 */
static inline uint32_t enet_ptp_ring_index(uint32_t size, uint32_t curIdx, uint32_t offset)
{
    return ((curIdx + offset) % size);
}

/*!
 * @brief Free all ring buffers.
 *
 * @param enetContextPtr The enet mac context buffer.
 * @return The execution status.
 */
uint32_t enet_ptp_deinit(enet_mac_context_t *enetContextPtr);

/*!
 * @brief The enet ptp time interrupt handler.
 *
 * @param enetIfPtr The enet context structure pointer.
 */
void enet_mac_ts_isr(void *enetIfPtr);
#endif
/*!
 * @brief(R)MII Read function.
 *
 * @param instance The enet instance number.
 * @param phyAddr The phy address.
 * @param phyReg The phy register.
 * @param dataPtr The data read from MII.
 * @return The execution status.
 */
uint32_t enet_mii_read(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, uint32_t *dataPtr);

/*!
 * @brief(R)MII Read function.
 *
 * @param instance The enet instance number.
 * @param phyAddr The phy address.
 * @param phyReg The phy register.
 * @param data The data write to MII.
 * @return The execution status.
 */
uint32_t enet_mii_write(uint32_t instance, uint32_t phyAddr, uint32_t phyReg, uint32_t data);

/*!
 * @brief Initialize enet buffer descriptors.
 *
 * @param enetIfPtr The enet context structure.
 * @return The execution status.
 */
uint32_t enet_mac_bd_init(enet_dev_if_t * enetIfPtr);

 /*!
 * @brief Initialize enet mac rmii/mii interface.
 *
 * @param enetIfPtr The enet context structure.
 * @return The execution status.
 */
uint32_t enet_mac_mii_init(enet_dev_if_t * enetIfPtr);

 /*!
 * @brief Initialize enet mac fifo and accelerator with the basic configuration.
 *
 * @param enetIfPtr The enet context structure.
 * @return The execution status.
 */
uint32_t enet_mac_fifo_accelerator_init(enet_dev_if_t * enetIfPtr);

/*!
 * @brief Close enet device.
 *
 * @param enetIfPtr The enet context structure.
 * @return The execution status.
 */
uint32_t enet_mac_close(enet_dev_if_t * enetIfPtr);

/*!
 * @brief Updata receive buffer descriptor.
 *
 * This is used to update the used receive buffer descriptor ring to
 * make sure the used bds will be correctly used again. It will clean 
 * the status region and set the control region of the used receive buffer 
 * descriptor. If the isBufferUpdate flag is set the data buffer in the
 * buffer descriptor will be updated.
 *
 * @param enetIfPtr The enet context structure.
 * @param isBufferUpdate The data buffer update flag.
 * @return The execution status.
 */
uint32_t enet_mac_update_rxbd(enet_dev_if_t * enetIfPtr, bool isBufferUpdate);
/*!
 * @brief Process enet receive frame error statistic.
 *
 * This interface is used to get the error statistic of the received frame.
 * beacuse the error information is in the last bd of a frame, this interface
 * should be called when processing the last bd of a frame.
 *
 * @param enetIfPtr The enet context structure.
 * @param data The current control and status data of buffer descriptor.
 * @return The frame error status.
 *         - true if the frame is error. 
 *         - false if the frame is without any error.
 */
bool enet_mac_rx_error_stats(enet_dev_if_t * enetIfPtr, uint32_t data);

/*!
 * @brief Process enet transmit frame statistic.
 *
 * This interface is used to get the error statistic of the transmit frame.
 * beacuse the error information is in the last bd of a frame, this interface
 * should be called when processing the last bd of a frame.
 *
 * @param enetIfPtr The enet context structure.
 * @param curBd The current buffer descriptor.
 */
void enet_mac_tx_error_stats(enet_dev_if_t * enetIfPtr,void *curBd);

/*!
 * @brief Enet transmit buffer descriptor cleanup.
 *
 * First, store transmit frame error statistic and ptp timestamp of transmitted packets. 
 * Second, cleanup the used transmit buffer descriptors.
 * If the ptp 1588 feature is open, this interface will do capture 1588 timestamp. 
 * It is called by transmit interrupt handler.
 *
 * @param enetIfPtr The enet context structure.
 * @return The execution status.
 */
uint32_t enet_mac_tx_cleanup(enet_dev_if_t * enetIfPtr);

/*!
 * @brief Receive enet packets.
 *
 * @param enetIfPtr The enet context structure.
 * @param packBuffer The received data buffer.
 * @return The execution status.
 */
uint32_t enet_mac_receive(enet_dev_if_t * enetIfPtr, enet_mac_packet_buffer_t *packBuffer);

/*!
 * @brief Transmit enet packets.
 *
 * @param enetIfPtr The enet context structure.
 * @param packet The frame to be transmitted.
 * @param size The frame size.
 * @return The execution status.
 */
uint32_t enet_mac_send(enet_dev_if_t * enetIfPtr, uint8_t *packet, uint32_t size);

/*!
 * @brief The enet receive interrupt handler.
 *
 * @param enetIfPtr The enet context structure pointer.
 */
void enet_mac_rx_isr(void *enetIfPtr);

/*!
 * @brief The enet transmit interrupt handler.
 *
 * @param enetIfPtr The enet context structure pointer.
 */
void enet_mac_tx_isr(void *enetIfPtr);

/*!
 * @brief Calculate crc hash value.
 *
 * @param address The enet mac hardware address.
 * @param crcVlaue The calculated crc value of the mac address.
 */
void enet_mac_calculate_crc32(enetMacAddr address, uint32_t *crcValue);

/*!
 * @brief Add enet device to multicast group.
 *
 * @param instance The enet instance number.
 * @param multiGroupPtr The enet multicast group structure.
 * @param address The enet mac hardware address.
 * @return The execution status.
 */
uint32_t enet_mac_add_multicast_group(uint32_t instance, enet_multicast_group_t *multiGroupPtr, enetMacAddr address);

/*!
 * @brief Move enet device from a multicast group.
 *
 * @param instance The enet instance number.
 * @param multiGroupPtr The enet multicast group structure.
 * @param address The enet mac hardware address.
 * @return The execution status.
 */
uint32_t enet_mac_leave_multicast_group(uint32_t instance, enet_multicast_group_t *multiGroupPtr, enetMacAddr address);

/*!
 * @brief Initialize the enet with the basic configuration.
 *
 * @param enetIfPtr The pointer to basic configuration structure.
 * @return The execution status.
 */
uint32_t enet_mac_init(enet_dev_if_t * enetIfPtr);

/*!
 * @brief Enqueue a data buffer to buffer queue.
 *
 * @param queue The buffer queue.
 * @param buffer The buffer to add to the buffer queue
 */
void enet_mac_enqueue_buffer( void **queue, void *buffer);

/*!
 * @brief Dequeue a buffer from buffer queue.
 *
 * @param queue The buffer queue.
 * @return The dequeued data buffer.
 */
void *enet_mac_dequeue_buffer( void **queue);

/* @} */

#if defined(__cplusplus)
extern }
#endif

/*! @}*/

#endif /* __FSL_ENET_DRIVER_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/

