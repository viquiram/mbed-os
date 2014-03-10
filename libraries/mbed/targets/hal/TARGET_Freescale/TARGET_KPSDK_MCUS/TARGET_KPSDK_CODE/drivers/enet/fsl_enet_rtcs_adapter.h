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

#ifndef __FSL_ENET_RTCS_ADAPTOR_H__
#define __FSL_ENET_RTCS_ADAPTOR_H__


/*! 
 * @addtogroup enet_rtcs_adaptor
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Define Task parameter*/
extern unsigned long  _RTCSTASK_priority;
#define ENET_RECEIVE_TASK_PRIO     (_RTCSTASK_priority + 2)
#define ENET_TASK_STACK_SIZE       (800)
#define ENET_PCB_NUM               (16) 

/*! @brief Define parameter for configuration*/
#define ENET_RX_RING_LEN           (8)
#define ENET_TX_RING_LEN           (4)
#define ENET_RX_LARGE_BUFFER_NUM   (0)        
#define ENET_RX_BUFFER_ALIGNMENT     (16)  
#define ENET_TX_BUFFER_ALIGNMENT     (16)
#define ENET_BD_ALIGNMENT            (16)
#define ENET_MII_CLOCK               (2500000L)
#if FSL_FEATURE_ENET_SUPPORT_PTP
#define ENET_PTP_RING_BUFFER_NUM     (30)
#endif

/*! @brief Define Error codes */
#define DRIVER_ERROR_BASE           (0xA000)
#define ENET_ERROR_BASE             (DRIVER_ERROR_BASE | 0x0400)
#define ENET_OK                     (0)
#define ENET_ERROR                  (ENET_ERROR_BASE | 0xff)  /* general ENET error */
#define ENETERR_INVALID_DEVICE      (ENET_ERROR_BASE | 0x00)   /* Device number out of range  */
#define ENETERR_INIT_DEVICE         (ENET_ERROR_BASE | 0x01)   /* Device already initialized  */
#define ENETERR_ALLOC_CFG           (ENET_ERROR_BASE | 0x02)   /* Alloc state failed          */
#define ENETERR_ALLOC_PCB           (ENET_ERROR_BASE | 0x03)   /* Alloc PCBs failed           */
#define ENETERR_ALLOC_BD            (ENET_ERROR_BASE | 0x04)   /* Alloc BDs failed            */
#define ENETERR_INSTALL_ISR         (ENET_ERROR_BASE | 0x05)   /* Install ISR failed          */
#define ENETERR_FREE_PCB            (ENET_ERROR_BASE | 0x06)   /* PCBs in use                 */
#define ENETERR_ALLOC_ECB           (ENET_ERROR_BASE | 0x07)   /* Alloc ECB failed            */
#define ENETERR_OPEN_PROT           (ENET_ERROR_BASE | 0x08)   /* Protocol not open           */
#define ENETERR_CLOSE_PROT          (ENET_ERROR_BASE | 0x09)   /* Protocol already open       */
#define ENETERR_SEND_SHORT          (ENET_ERROR_BASE | 0x0A)   /* Packet too short            */
#define ENETERR_SEND_LONG           (ENET_ERROR_BASE | 0x0B)   /* Packet too long             */
#define ENETERR_JOIN_MULTICAST      (ENET_ERROR_BASE | 0x0C)   /* Not a multicast address     */
#define ENETERR_ALLOC_MCB           (ENET_ERROR_BASE | 0x0D)   /* Alloc MCB failed            */
#define ENETERR_LEAVE_GROUP         (ENET_ERROR_BASE | 0x0E)   /* Not a joined group          */
#define ENETERR_SEND_FULL           (ENET_ERROR_BASE | 0x0F)   /* Transmit ring full          */
#define ENETERR_IP_TABLE_FULL       (ENET_ERROR_BASE | 0x10)   /* IP Table full of IP pairs   */
#define ENETERR_ALLOC               (ENET_ERROR_BASE | 0x11)   /* Generic alloc failed        */
#define ENETERR_INIT_FAILED         (ENET_ERROR_BASE | 0x12)   /* Device failed to initialize */
#define ENETERR_DEVICE_TIMEOUT      (ENET_ERROR_BASE | 0x13)   /* Device read/write timeout   */
#define ENETERR_ALLOC_BUFFERS       (ENET_ERROR_BASE | 0x14)   /* Buffer alloc failed         */
#define ENETERR_ALLOC_MAC_CONTEXT   (ENET_ERROR_BASE | 0x15)   /* Buffer alloc failed         */
#define ENETERR_NO_TX_BUFFER        (ENET_ERROR_BASE | 0x16)   /* TX Buffer alloc failed      */
#define ENETERR_INVALID_INIT_PARAM  (ENET_ERROR_BASE | 0x17)   /* Invalid init. parameter     */
#define ENETERR_DEVICE_IN_USE       (ENET_ERROR_BASE | 0x18)   /* Shutdown failed, dev. in use*/
#define ENETERR_INITIALIZED_DEVICE  (ENET_ERROR_BASE | 0x19)   /* Device already initialized  */
#define ENETERR_INPROGRESS          (ENET_ERROR_BASE | 0x1A)   /* In Wifi Device Setting of ESSID in progress*/
#define ENETERR_1588_LWEVENT        (ENET_ERROR_BASE | 0x1B)   /* 1588driver lwevent creation failed */
#define ENETERR_INVALID_MODE        (ENET_ERROR_BASE | 0x1C)   /* Invalid mode for this ethernet driver */
#define ENETERR_INVALID_OPTION      (ENET_ERROR_BASE | 0x1D)   /* Invalid option for this ethernet driver */
#define ENETERR_SEND_LARGE          (ENET_ERROR_BASE | 0x1E)   /* Send packet large*/
#define ENETERR_INITIALIZED_MULTICAST (ENET_ERROR_BASE | 0x1F) /* Multicast group already added*/
#define ENETERR_NULL_MULTICAST        (ENET_ERROR_BASE | 0x20) /* Multicast group NULL*/
#define ENETERR_INVALID_MULTICAST     (ENET_ERROR_BASE | 0x21) /* Invalid Multicast group*/
#define ENETERR_MIN                 (ENETERR_INVALID_DEVICE)
#define ENETERR_MAX                 (ENETERR_INVALID_OPTION)

/*! @brief Define ENET protocol parameter*/ 
#define ENETPROT_IP               0x0800
#define ENETPROT_ARP              0x0806
#define ENETPROT_8021Q            0x8100
#define ENETPROT_IP6              0x86DD
#define ENETPROT_ETHERNET         0x88F7
#define ENET_OPT_8023             0x0001
#define ENET_OPT_8021QTAG         0x0002
#define ENET_SETOPT_8021QPRIO(p)  (ENET_OPT_8021QTAG | (((uint_32)(p) & 0x7) << 2))
#define ENET_GETOPT_8021QPRIO(f)  ((((unsigned int)f) >> 2) & 0x7) 

/*! @brief Define ENET option macro*/
#define ENET_OPTION_HW_TX_IP_CHECKSUM       0x00001000
#define ENET_OPTION_HW_TX_PROTOCOL_CHECKSUM 0x00002000
#define ENET_OPTION_HW_RX_IP_CHECKSUM       0x00004000
#define ENET_OPTION_HW_RX_PROTOCOL_CHECKSUM 0x00008000
#define ENET_OPTION_HW_RX_MAC_ERR           0x00010000

/*! @brief Define for ENET default MAC*/
#define ENET_DEFAULT_MAC_ADD                { 0x00, 0x00, 0x5E, 0, 0, 0 }
#define PCB_MINIMUM_SIZE                    (sizeof(PCB2))
#define PCB_free(pcb_ptr)                   ((pcb_ptr)->FREE(pcb_ptr))

/*! @brief Define macro for byte-swap*/
#define htonl(p,x) (((uint_8_ptr)(p))[0] = ((x) >> 24) & 0xFF, \
                    ((uint_8_ptr)(p))[1] = ((x) >> 16) & 0xFF, \
                    ((uint_8_ptr)(p))[2] = ((x) >>  8) & 0xFF, \
                    ((uint_8_ptr)(p))[3] =  (x)        & 0xFF, \
                    (x))

#define htons(p,x) (((uint_8_ptr)(p))[0] = ((x) >>  8) & 0xFF, \
                    ((uint_8_ptr)(p))[1] =  (x)        & 0xFF, \
                    (x))

#define htonc(p,x) (((uint_8_ptr)(p))[0] = (x) & 0xFF, \
                    (x))

#define ntohl(p)   (\
                    (((uint_32)(((uint_8_ptr)(p))[0])) << 24) | \
                    (((uint_32)(((uint_8_ptr)(p))[1])) << 16) | \
                    (((uint_32)(((uint_8_ptr)(p))[2])) << 8) | \
                    ( (uint_32)(((uint_8_ptr)(p))[3])) \
                   )

#define ntohs(p)   (\
                    (((uint_16)(((uint_8_ptr)(p))[0])) << 8) | \
                    ( (uint_16)(((uint_8_ptr)(p))[1])) \
                   )

#define ntohc(p)     ((uint_8)(((uint_8_ptr)(p))[0]))
#define htone(p,x)   ((p)[0] = (x)[0], \
                         (p)[1] = (x)[1], \
                         (p)[2] = (x)[2], \
                         (p)[3] = (x)[3], \
                         (p)[4] = (x)[4], \
                         (p)[5] = (x)[5]  \
                      )

#define ntohe(p,x)   ((x)[0] = (p)[0] & 0xFF, \
                      (x)[1] = (p)[1] & 0xFF, \
                      (x)[2] = (p)[2] & 0xFF, \
                      (x)[3] = (p)[3] & 0xFF, \
                      (x)[4] = (p)[4] & 0xFF, \
                      (x)[5] = (p)[5] & 0xFF  \
                      )

/*! @brief Define add to queue*/
#define QUEUEADD(head,tail,pcb)      \
   if ((head) == NULL) {         \
      (head) = (pcb);            \
   } else {                      \
      (tail)->PRIVATE = (pcb);   \
   }                             \
   (tail) = (pcb);               \
   (pcb)->PRIVATE = NULL
   
/*! @brief Define get from queue*/
#define QUEUEGET(head,tail,pcb)      \
   (pcb) = (head);               \
   if (head) {                   \
      (head) = (head)->PRIVATE;  \
      if ((head) == NULL) {      \
         (tail) = NULL;          \
      }                          \
   }  

/*! @brief Define for ENET six-byte MAC type*/
typedef unsigned char   _enet_address[6];

/*! @brief Define the structure for ipcfg*/
typedef void * _enet_handle;
struct pcb;
typedef void (*  PCB_FREE_FPTR)(struct pcb *);

/*! @brief Define Structure for Ethernet packet header*/
typedef struct enet_header 
{
    _enet_address    DEST;     /*!< destination MAC address*/
    _enet_address    SOURCE;   /*!< source MAC address*/
    unsigned char    TYPE[2];  /*!< protocol type*/
} ENET_HEADER, * ENET_HEADER_PTR;

/*! @brief Define Structure that contains fragment of PCB*/
typedef struct pcb_fragment 
{
    uint32_t           LENGTH;       /*!< Packet fragment length*/
    unsigned char     *FRAGMENT;     /*!< brief Pointer to fragment*/
} PCB_FRAGMENT, * PCB_FRAGMENT_PTR;

/*! @brief Define PCB structure for RTCS adaptor.*/
typedef struct pcb 
{
    PCB_FREE_FPTR     FREE;   /*!< Function that frees PCB*/
    void   *PRIVATE;          /*!< Private PCB information*/
    PCB_FRAGMENT  FRAG[1];    /*!< Pointer to PCB fragment*/
} PCB, * PCB_PTR;

/*! @brief Define PCB structure contains two fragments*/
typedef struct pcb2
{
    PCB_FREE_FPTR FREE;      /*!< Function that frees PCB*/
    void  *PRIVATE;          /*!< Private PCB information*/
    PCB_FRAGMENT FRAG[2];    /*!< Pointers to two PCB fragments*/
} PCB2,  *PCB2_PTR;

/*! @brief Define PCB structure contains two fragments*/
typedef struct pcb_queue  
{
    PCB *pcbHead;     /*!< PCB buffer head*/      
    PCB *pcbTail;     /*!< PCB buffer tail*/ 
}pcb_queue;

/*! @brief Define ECB structure contains protocol type and it's related service function*/
typedef struct enet_ecb_struct
{
    uint16_t  TYPE;
    void (* SERVICE)(PCB_PTR, void *);
    void  *PRIVATE;
    struct enet_ecb_struct *NEXT;
}ENET_ECB_STRUCT,* ENET_ECB_STRUCT_PTR;

/*! @brief Define 8022 header*/
typedef struct enet_8022_header
{
    uint8_t dsap[1];           /*!< DSAP region*/
    uint8_t ssap[1];           /*!< SSAP region*/
    uint8_t command[1];        /*!< Command region*/
    uint8_t oui[3];            /*!< OUI region*/
    uint16_t type;             /*!< type region*/
}enet_8022_header_t, *enet_8022_header_ptr;

/*! @brief Define common status structure*/
typedef struct enet_commom_stats_struct {
    uint32_t     ST_RX_TOTAL;         /*!< Total number of received packets*/
    uint32_t     ST_RX_MISSED;        /*!<  Number of missed packets*/
    uint32_t     ST_RX_DISCARDED;     /*!< Discarded unrecognized protocol*/
    uint32_t     ST_RX_ERRORS;        /*!< Discarded error during reception*/
    uint32_t     ST_TX_TOTAL;         /*!< Total number of transmitted packets*/
    uint32_t     ST_TX_MISSED;        /*!< Discarded transmit ring full*/
    uint32_t     ST_TX_DISCARDED;     /*!< Discarded bad packet*/
    uint32_t     ST_TX_ERRORS;        /*!< Error during transmission*/
} ENET_COMMON_STATS_STRUCT, * ENET_COMMON_STATS_STRUCT_PTR;

typedef struct enet_stats {
    ENET_COMMON_STATS_STRUCT   COMMON; /*!< Common status structure*/
    uint32_t     ST_RX_ALIGN;          /*!< Frame Alignment error*/
    uint32_t     ST_RX_FCS;            /*!< CRC error  */
    uint32_t     ST_RX_RUNT;           /*!< Runt packet received */
    uint32_t     ST_RX_GIANT;          /*!< Giant packet received*/
    uint32_t     ST_RX_LATECOLL;       /*!< Late collision */
    uint32_t     ST_RX_OVERRUN;        /*!< DMA overrun*/
    uint32_t     ST_TX_SQE;            /*!< Heartbeat lost*/
    uint32_t     ST_TX_DEFERRED;       /*!< Transmission deferred*/
    uint32_t     ST_TX_LATECOLL;       /*!< Late collision*/
    uint32_t     ST_TX_EXCESSCOLL;     /*!< Excessive collisions*/
    uint32_t     ST_TX_CARRIER;        /*!< Carrier sense lost*/
    uint32_t     ST_TX_UNDERRUN;       /*!< DMA underrun*/
   /* Following stats are collected by the Ethernet driver  */
    uint32_t     ST_RX_COPY_SMALL;     /*!< Driver had to copy packet */
    uint32_t     ST_RX_COPY_LARGE;     /*!< Driver had to copy packet */
    uint32_t     ST_TX_COPY_SMALL;     /*!< Driver had to copy packet */
    uint32_t     ST_TX_COPY_LARGE;     /*!< Driver had to copy packet */
    uint32_t     RX_FRAGS_EXCEEDED;
    uint32_t     RX_PCBS_EXHAUSTED;
    uint32_t     RX_LARGE_BUFFERS_EXHAUSTED;
    uint32_t     TX_ALIGNED;
    uint32_t     TX_ALL_ALIGNED;
#if BSPCFG_ENABLE_ENET_HISTOGRAM
    uint32_t     RX_HISTOGRAM[ENET_HISTOGRAM_ENTRIES];  
    uint32_t     TX_HISTOGRAM[ENET_HISTOGRAM_ENTRIES];  
#endif
  
} ENET_STATS, * ENET_STATS_PTR;

/*******************************************************************************
 * API 
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! 
  * @name ENET RTCS ADAPTOR
  * @{
  */

 /*!
 * @brief Initialize the ENET device.
 *
 * @param device The ENET device number.
 * @param address The hardware address.
 * @param flag The flag for upper layer.
 * @param handle The address pointer for ENET device structure.
 * @return The execution status.
 */
uint32_t ENET_initialize(uint32_t device, _enet_address address,uint32_t flag, _enet_handle *handle);

/*!
 * @brief Open the ENET device.
 *
 * @param handle The address pointer for ENET device structure.
 * @param type The ENET protocol type.
 * @param service The service function for type.
 * @param private The private data for ENET device.
 * @return The execution status.
 */
uint32_t ENET_open(_enet_handle handle, uint16_t type, void (* service)(PCB_PTR, void *), void *private);

/*!
 * @brief Shutdown the ENET device.
 *
 * @param handle The address pointer for ENET device structure.
 * @return The execution status.
 */
uint32_t ENET_shutdown(_enet_handle handle);
#if !ENET_RECEIVE_ALL_INTERRUPT
/*!
 * @brief ENET frame receive.
 *
 * @param enetIfPtr The address pointer for ENET device structure.
 */
static void ENET_receive(void *param);
#endif
/*!
 * @brief ENET frame transmit.
 *
 * @param handle The address pointer for ENET device structure.
 * @param packet The ENET packet buffer.
 * @param type The ENET protocol type.
 * @param dest The destination hardware address.
 * @param flag The flag for upper layer.
 * @return The execution status.
 */
uint32_t ENET_send(_enet_handle handle, PCB_PTR packet, uint32_t type, _enet_address dest, uint32_t flags)	;

/*!
 * @brief ENET get address with initialized device.
 *
 * @param handle The address pointer for ENET device structure.
 * @param address The destination hardware address.
 * @return The execution status.
 */
uint32_t ENET_get_address(_enet_handle handle, _enet_address address);

/*!
 * @brief ENET get address with uninitialized device.
 *
 * @param handle The address pointer for ENET device structure.
 * @param value The value to change the last three bytes of hardware.
 * @param address The destination hardware address.
 * @return True if the execution status is success else false.
 */
uint32_t ENET_get_mac_address(uint32_t device, uint32_t value, _enet_address address);
/*!
 * @brief ENET join a multicast group address.
 *
 * @param handle The address pointer for ENET device structure.
 * @param type The ENET protocol type.
 * @param address The destination hardware address.
 * @return The execution status.
 */
uint32_t ENET_join(_enet_handle handle, uint16_t type, _enet_address address);

/*!
 * @brief ENET leave a multicast group address.
 *
 * @param handle The address pointer for ENET device structure.
 * @param type The ENET protocol type.
 * @param address The destination hardware address.
 * @return The execution status.
 */
uint32_t ENET_leave(_enet_handle handle, uint16_t type, _enet_address address);
#if BSPCFG_ENABLE_ENET_STATS
/*!
 * @brief ENET get packet statistic.
 *
 * @param handle The address pointer for ENET device structure.
 * @return The statistic.
 */
ENET_STATS_PTR ENET_get_stats(_enet_handle handle);
#endif
/*!
 * @brief ENET get link status.
 *
 * @param handle The address pointer for ENET device structure.
 * @return The link status.
 */
bool ENET_link_status(_enet_handle handle);

/*!
 * @brief ENET get link speed.
 *
 * @param handle The address pointer for ENET device structure.
 * @return The link speed.
 */
uint32_t ENET_get_speed(_enet_handle handle);

/*!
 * @brief ENET get MTU.
 *
 * @param handle The address pointer for ENET device structure.
 * @return The link MTU
 */
uint32_t ENET_get_MTU(_enet_handle handle);

/*!
 * @brief Get ENET PHY registers.
 *
 * @param handle The address pointer for ENET device structure.
 * @param numRegs The number of registers.
 * @param regPtr The buffer for data read from PHY registers.
 * @return True if all numRegs registers are read succeed else false.
 */
bool ENET_phy_registers(_enet_handle handle, uint32_t numRegs, uint32_t *regPtr);

/*!
 * @brief Get ENET options.
 *
 * @param handle The address pointer for ENET device structure.
 * @return ENET options.
 */
uint32_t ENET_get_options(_enet_handle handle);

/*!
 * @brief Unregisters a protocol type on an Ethernet channel.
 *
 * @param handle The address pointer for ENET device structure.
 * @return ENET options.
 */
uint32_t ENET_close(_enet_handle handle, uint16_t type);

/*!
 * @brief ENET mediactl .
 *
 * @param handle The address pointer for ENET device structure.
 * @param The command Id.
 * @param The buffer for input or output parameters.
 * @return ENET options.
 */
uint32_t ENET_mediactl(_enet_handle handle, uint32_t commandId, void *inOutParam);

/*!
 * @brief Get the next ENET device handle address.
 *
 * @param handle The address pointer for ENET device structure.
 * @return The address of next ENET device handle.
 */
_enet_handle ENET_get_next_device_handle(_enet_handle handle);

/*!
 * @brief ENET free .
 *
 * @param packet The buffer address.
 */
void ENET_free(PCB_PTR packet);

/*!
 * @brief ENET error description.
 *
 * @param error The ENET error code.
 * @return The error string.
 */
const char * ENET_strerror(uint32_t  error);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_ENET_RTCS_ADAPTOR_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/




