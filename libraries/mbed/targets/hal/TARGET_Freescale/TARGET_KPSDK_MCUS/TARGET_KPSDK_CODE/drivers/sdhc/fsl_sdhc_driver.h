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

#ifndef __FSL_SDHC_H__
#define __FSL_SDHC_H__

#include "fsl_sdhc_hal.h"
#include "fsl_os_abstraction.h"

/*! @addtogroup sdhc_pd_data_types */
/*! @{ */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum _sdhc_card_type
{
    kCardTypeUnknown = 1,           /*!< Unknown card type */
    kCardTypeSDSC,                  /*!< SDSC card type */
    kCardTypeSDHC,                  /*!< SDHC card type */
    kCardTypeSDXC,                  /*!< SDXC card type */
    kCardTypeMMC,                   /*!< MMC card type */
} sdhc_card_type_t;

typedef enum _sdhc_status
{
    kStatus_SDHC_NoError = 0,               /*!< SDHC no error */
    kStatus_SDHC_WaitTimeoutError,          /*!< SDHC wait timeout error */
    kStatus_SDHC_IoError,                   /*!< SDHC general IO error */
    kStatus_SDHC_CmdIoError,                /*!< SDHC CMD I/O error */
    kStatus_SDHC_DataIoError,               /*!< SDHC data I/O error */
    kStatus_SDHC_InvalidParameter,          /*!< SDHC invalid parameter */
    kStatus_SDHC_RequestFailed,             /*!< SDHC request failed */
    kStatus_SDHC_SwitchFailed,              /*!< SDHC switch failed */
    kStatus_SDHC_NotSupportYet,             /*!< SDHC not support */
    kStatus_SDHC_TimeoutError,              /*!< SDHC timeout error*/
    kStatus_SDHC_CardNotSupport,            /*!< SDHC card does not support */
    kStatus_SDHC_CmdError,                  /*!< SDHC CMD error */
    kStatus_SDHC_DataError,                 /*!< SDHC data error */
    kStatus_SDHC_Failed,                    /*!< SDHC general failed */
    kStatus_SDHC_NoMedium,                  /*!< SDHC no medium error */
} sdhc_status_t;

typedef enum _sdhc_power_mode
{
    kSdhcPowerModeRunning = 0,      /*!< SDHC is running */
    kSdhcPowerModeSuspended,        /*!< SDHC is suspended */
    kSdhcPowerModeStopped,          /*!< SDHC is stopped */
} sdhc_power_mode_t;

/*!
 * @brief SDHC Card Structure
 * 
 * Defines the card structure including the necessary fields to identify and
 * describe the card.
 */
typedef struct SdhcCard
{
    struct SdhcHostDevice * host;                   /*!< Associated host */
    uint32_t version;                               /*!< Card version */
    uint32_t rca;                                   /*!< RCA */
    sdhc_card_type_t cardType;                      /*!< Card type */
    uint32_t flags;                                 /*!< Flags */
    uint32_t caps;                                  /*!< Capability */
#define SDMMC_CARD_CAPS_HIGHSPEED    (1 << 0)           /*!< SD card high speed support bit */
#define SDMMC_CARD_CAPS_4BIT_MODE    (1 << 1)           /*!< SD card high speed support bit */
    uint32_t busMode;                               /*!< Data width */
    uint32_t rawCid[4];                             /*!< Raw CID */
    uint32_t rawCsd[4];                             /*!< Raw CSD */
    uint32_t rawScr[2];                             /*!< Raw SCR */
    uint8_t* rawExtCsd;                             /*!< Raw EXT_CSD  */
    uint32_t capacity;                              /*!< Card total size */
} sdhc_card_t;

/*!
 * @brief SDHC Initialization Configuration Structure
 * 
 * Defines the configuration data structure to initialize the SDHC.
 */
typedef struct SdhcUserConfig
{
    uint32_t clock;                                 /*!< Clock rate */
    uint32_t busWidth;                              /*!< Data bus width */
    void (*card_detect_callback)(void *param);      /*!< Card detect callback function */
} sdhc_user_config_t;

/*!
 * @brief SDHC Configure Structure
 * 
 * SDHC Configuration Data Structure
 */
typedef struct SdhcHostConfig
{
    uint32_t clock;                                 /*!< Clock rate */
    sdhc_power_mode_t powerMode;                    /*!< Power supply mode */
    uint32_t busWidth;                              /*!< Data bus width */
} sdhc_host_config_t;

/*!
 * @brief SDHC Host Device Structure
 * 
 * Defines the Host device structure which includes both the static and the runtime SDHC information.
 */
typedef struct SdhcHostDevice
{
    uint8_t instance;                               /*!< Host instance index */
    uint32_t specVer;                               /*!< Host specification version */
    uint32_t vendorVer;                             /*!< Host vendor version */
    sdhc_hal_endian_t endian;                       /*!< Endian mode the host's working at */
    IRQn_Type irq;                                  /*!< IRQ number */
    uint32_t flags;                                 /*!< Host flags */
    uint32_t busWidth;                              /*!< Current busWidth */
    uint32_t caps;                                  /*!< Host capability */
    uint32_t ocr;
    uint32_t ocrSupported;                          /*!< Supported OCR */
    uint32_t clock;                                 /*!< Current clock frequency */
    sdhc_power_mode_t powerMode;                    /*!< Current power mode */
    uint32_t maxClock;                              /*!< Maximum clock supported */
    uint32_t maxBlockSize;                          /*!< Maximum block size supported */
    struct SdhcHostConfig config;                   /*!< Host configuration */
    struct SdhcRequest * currentReq;                /*!< Associated request */
    struct SdhcCommand * currentCmd;                /*!< Associated command  */
    struct SdhcData * currentData;                  /*!< Associated data */
    struct SdhcCard * card;                         /*!< Associated card */
    sync_object_t host_lock;                        /*!< Sync object */
} sdhc_host_t;

/*!
 * @brief SDHC Data Structure
 * 
 * Defines the SDHC data structure including the block size/count and flags.
 */
typedef struct SdhcData
{
    struct SdhcRequest *req;                        /*!< Associated request */
    struct SdhcCommand *cmd;                        /*!< Associated command */
    uint32_t blockSize;                             /*!< Block size */
    uint32_t blockCount;                            /*!< Block count */
    uint32_t flags;                                 /*!< Data flags */
#define SDMMC_DATA_READ         (1 << 0)            /*!< flag: read data */
#define SDMMC_DATA_WRITE        (1 << 1)            /*!< flag: write data */

#define SDMMC_RSP_PRESENT       (1 << 1)            /*!< flag: response presented */
#define SDMMC_RSP_136BITS       (1 << 2)            /*!< flag: response with 136 bits length */
#define SDMMC_RSP_CRC           (1 << 3)            /*!< flag: response checking CRC */
#define SDMMC_RSP_BUSY          (1 << 4)            /*!< flag: response with busy */
#define SDMMC_RSP_CHK_IDX       (1 << 5)            /*!< flag: response with checking command index*/

#define SDMMC_CMD_MASK          (3 << 6)            /*!< flag: command mask */
#define SDMMC_CMD_AC            (0 << 6)            /*!< flag: addressed command */
#define SDMMC_CMD_ADTC          (1 << 6)            /*!< flag: addressed data transfer command */
#define SDMMC_CMD_BC            (2 << 6)            /*!< flag: broadcast command */
#define SDMMC_CMD_BCR           (3 << 6)            /*!< flag: broadcast command with response */

#define SDMMC_RSP_NONE          (0U)
#define SDMMC_RSP_R1            (SDMMC_RSP_PRESENT|SDMMC_RSP_CRC|SDMMC_RSP_CHK_IDX)                 /*!< Response 1 */
#define SDMMC_RSP_R1B           (SDMMC_RSP_PRESENT|SDMMC_RSP_CRC|SDMMC_RSP_CHK_IDX|SDMMC_RSP_BUSY)  /*!< Response 1 with busy */
#define SDMMC_RSP_R2            (SDMMC_RSP_PRESENT|SDMMC_RSP_136BITS|SDMMC_RSP_CRC)                 /*!< Response 2 */
#define SDMMC_RSP_R3            (SDMMC_RSP_PRESENT)                                                 /*!< Response 3 */
#define SDMMC_RSP_R4            (SDMMC_RSP_PRESENT)                                                 /*!< Response 4 */
#define SDMMC_RSP_R5            (SDMMC_RSP_PRESENT|SDMMC_RSP_CRC|SDMMC_RSP_CHK_IDX)                 /*!< Response 5 */
#define SDMMC_RSP_R6            (SDMMC_RSP_PRESENT|SDMMC_RSP_CRC|SDMMC_RSP_CHK_IDX)                 /*!< Response 6 */
#define SDMMC_RSP_R7            (SDMMC_RSP_PRESENT|SDMMC_RSP_CRC|SDMMC_RSP_CHK_IDX)                 /*!< Response 7 */
    uint32_t error;                                 /*!< Data error code */
    uint32_t bytesTransferred;                      /*!< Transferred buffer */
    uint32_t length;                                /*!< Data length */
    uint32_t *buffer;                               /*!< Data buffer */
} sdhc_data_t;

/*!
 * @brief SDHC Command Structure
 * 
 * Defines the SDHC command structure  including the command index, argument, flags, and response.
 */
typedef struct SdhcCommand
{
    struct SdhcRequest *req;                        /*!< Associated request */
    struct SdhcData *data;                          /*!< Data associated with request */
    uint32_t index;                                 /*!< Command index */
    uint32_t argument;                              /*!< Command argument */
    uint32_t flags;                                 /*!< Command flags */
    uint32_t response[4];                           /*!< Response for this command */
    uint32_t error;                                 /*!< Command error code */
} sdhc_command_t;

/*!
 * @brief SDHC Request Done Callback
 */
typedef void (*request_done)(struct SdhcHostDevice *host, struct SdhcRequest *req);

/*!
 * @brief SDHC Request Structure
 * 
 * Defines the SDHC request structure. In  most   cases, it  includes the related
 * command and data and  callback upon its completion.
 */
typedef struct SdhcRequest
{
    struct SdhcCommand *cmd;                        /*!< Command associated with the request */
    struct SdhcData *data;                          /*!< Data associated with request */
    sync_object_t done;                             /*!< Sync object */
    request_done onDone;                            /*!< Callback on request done */
} sdhc_request_t;
/*! @} */

/*! @addtogroup sdhc_pd */
/*! @{ */

/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif
/*! @name SDHC PD FUNCTION */
/*@{ */

/*!
 * @brief Initializes the Host controller by a specific instance index.
 *
 * This function initializes the SDHC module according to the given
 * initialization configuration structure including the clock frequency,
 * bus width, and card detect callback.
 *
 * @param instance the specific instance index
 * @param host the memory address allocated for the host handle
 * @param config initialization configuration data
 * @return kStatus_SDHC_NoError if success
 */
sdhc_status_t sdhc_init(uint8_t instance, sdhc_host_t * host, const sdhc_user_config_t *config);

/*!
 * @brief Destroy host controller
 *
 * @param host the pointer to the host controller about to be destroyed
 */
void sdhc_shutdown(sdhc_host_t *host);

/*!
 * @brief check whether the card is present on specified host controller.
 *
 * This function checks if there's a card inserted to the SDHC.
 *
 * @param host the pointer to the host controller
 * @param card the pointer to retrieve card information
 * @return kStatus_SDHC_NoError on success
 */
sdhc_status_t sdhc_check_card(sdhc_host_t *host, sdhc_card_t *card);

/*!
 * @brief Checks the read only for the attached card.
 *
 * @param host the pointer to the host controller
 * @return kStatus_SDHC_NoError on success
 */
sdhc_status_t sdhc_check_ro(sdhc_host_t *host);

/*!
 * @brief Configures the specified host controller.
 *
 * With this function, a user can modify the specific SDHC configuration.
 *
 *
 * @param host the pointer to the host controller
 * @param config the pointer to the configuration information
 * @return kStatus_SDHC_NoError on success
 */
sdhc_status_t sdhc_config_host(sdhc_host_t *host, sdhc_host_config_t *config);

/*!
 * @brief Issues the request on a specific Host controller and returns on completion.
 *
 * This function  issues the request to the card on a specific SDHC.
 * The command  is sent and is blocked as long as
 * the response/data is sending back from the card.
 *
 * @param host the pointer to the host controller
 * @param req the pointer to the request
 * @return kStatus_SDHC_NoError on success
 */
sdhc_status_t sdhc_issue_request(sdhc_host_t *host, sdhc_request_t *req);

/*@} */
#if defined(__cplusplus)
}
#endif
/*! @} */
#endif /* __FSL_SDHC_H__ */

/*************************************************************************************************
 * EOF
 ************************************************************************************************/

