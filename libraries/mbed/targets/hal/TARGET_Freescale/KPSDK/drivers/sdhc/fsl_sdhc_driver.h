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
    kStatus_SDHC_CmdIoError,                /*!< SDHC cmd IO error */
    kStatus_SDHC_DataIoError,               /*!< SDHC data IO error */
    kStatus_SDHC_InvalidParameter,          /*!< SDHC invalid parameter */
    kStatus_SDHC_RequestFailed,             /*!< SDHC request failed */
    kStatus_SDHC_SwitchFailed,              /*!< SDHC switch failed */
    kStatus_SDHC_NotSupportYet,             /*!< SDHC not support */
    kStatus_SDHC_TimeoutError,              /*!< SDHC timeout error*/
    kStatus_SDHC_CardNotSupport,            /*!< SDHC card does not support */
    kStatus_SDHC_CmdError,                  /*!< SDHC cmd error */
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

typedef struct SdhcHostDevice sdhc_host_t;
typedef struct SdhcData sdhc_data_t;
typedef struct SdhcCommand sdhc_command_t;
typedef struct SdhcRequest sdhc_request_t;
typedef struct SdhcHostConfig sdhc_host_config_t;
typedef struct SdhcCard sdhc_card_t;
typedef struct SdhcHostInitConfig sdhc_init_config_t;
/*!
 * @brief SDHC Request Done Callback
 */
typedef void (*request_done)(sdhc_host_t *host, sdhc_request_t *req);

/*!
 * @brief SDHC Card Structure
 * 
 * Define structure of a card, including necessary fields to identify and
 * describe the card.
 */
struct SdhcCard
{
    sdhc_host_t * host;                             /*!< associated host */
    uint32_t version;                               /*!< card version */
    uint32_t rca;                                   /*!< RCA */
    sdhc_card_type_t cardType;                      /*!< card type */
    uint32_t flags;                                 /*!< flags */
    uint32_t caps;                                  /*!< capability */
#define SDMMC_CARD_CAPS_HIGHSPEED    (1 << 0)           /*!< SD card high speed support bit */
#define SDMMC_CARD_CAPS_4BIT_MODE    (1 << 1)           /*!< SD card high speed support bit */
    uint32_t busMode;                               /*!< data width */
    uint32_t rawCid[4];                             /*!< raw CID */
    uint32_t rawCsd[4];                             /*!< raw CSD */
    uint32_t rawScr[2];                             /*!< raw SCR */
    uint8_t* rawExtCsd;                             /*!< raw EXT_CSD  */
    uint32_t capacity;                              /*!< card total size */
};

/*!
 * @brief SDHC Initialization Configure Structure
 * 
 * Define structure of the necessary configuration data to initialize SDHC.
 */
struct SdhcHostInitConfig
{
    uint32_t clock;                                 /*!< clock rate */
    uint32_t busWidth;                              /*!< data bus width */
    void (*card_detect_callback)(void *param);      /*!< card detect callback function */
};

/*!
 * @brief SDHC Configure Structure
 * 
 * Define structure of configuration data to set to SDHC on the fly.
 */
struct SdhcHostConfig
{
    uint32_t clock;                                 /*!< clock rate */
    sdhc_power_mode_t powerMode;                    /*!< power supply mode */
    uint32_t busWidth;                              /*!< data bus width */
};

/*!
 * @brief SDHC Host Device Structure
 * 
 * Define structure of host device includes both static and runtime informations of SDHC.
 */
struct SdhcHostDevice
{
    uint8_t instance;                               /*!< host instance index */
    uint32_t specVer;                               /*!< host specification version */
    uint32_t vendorVer;                             /*!< host vendor version */
    uint32_t endian;                                /*!< endian mode the host's working at */
    IRQn_Type irq;                                  /*!< irq number */
    uint32_t flags;                                 /*!< host flags */
    uint32_t busWidth;                              /*!< current busWidth */
    uint32_t caps;                                  /*!< host capability */
    uint32_t ocr;
    uint32_t ocrSupported;                          /*!< supported OCR */
    uint32_t clock;                                 /*!< current clock frequency */
    sdhc_power_mode_t powerMode;                    /*!< current power mode */
    uint32_t maxClock;                              /*!< max clock supported */
    uint32_t maxBlockSize;                          /*!< max block size supported */
    sdhc_host_config_t config;                      /*!< host configuration */
    sdhc_request_t * currentReq;                    /*!< associated request */
    sdhc_command_t * currentCmd;                    /*!< associated command  */
    sdhc_data_t * currentData;                      /*!< associated data */
    sdhc_card_t * card;                             /*!< associated card */
    sync_object_t host_lock;                        /*!< sync object */
};

/*!
 * @brief SDHC Data Structure
 * 
 * Define structure of data for SDHC, including the block size/count and flags.
 */
struct SdhcData
{
    sdhc_request_t *req;                            /*!< associated request */
    sdhc_command_t *cmd;                            /*!< associated command */
    uint32_t blockSize;                             /*!< block size */
    uint32_t blockCount;                            /*!< block count */
    uint32_t flags;                                 /*!< data flags */
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
    uint32_t error;                                 /*!< data error code */
    uint32_t bytesTransferred;                      /*!< transferred buffer */
    uint32_t length;                                /*!< data length */
    uint32_t *buffer;                               /*!< data buffer */
};

/*!
 * @brief SDHC Command Structure
 * 
 * Define structure of command for SDHC, including the command index, argument, flags and response.
 */
struct SdhcCommand
{
    sdhc_request_t *req;                            /*!< associated request */
    sdhc_data_t *data;                              /*!< data associated with request */
    uint32_t index;                                 /*!< command index */
    uint32_t argument;                              /*!< command argument */
    uint32_t flags;                                 /*!< command flags */
    uint32_t response[4];                           /*!< response for this command */
    uint32_t error;                                 /*!< command error code */
};

/*!
 * @brief SDHC Request Structure
 * 
 * Define structure of Request for SDHC, for most of the cases, it will include the related
 * command and data, and the callback on its completion.
 */
struct SdhcRequest
{
    sdhc_command_t *cmd;                            /*!< command associated with the request */
    sdhc_data_t *data;                              /*!< data associated with request */
    sync_object_t done;                             /*!< sync object */
    request_done onDone;                            /*!< callback on request done */
};
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
 * @brief initialize host controller by specific instance index
 *
 * This function initialize the SDHC module according to the given
 * initialization configure structure including clock frequency,
 * bus width and card detect callback.
 *
 * @param instance the specific instance index
 * @param host the memory address allocated for the host handle
 * @param config initialization configuration data
 * @return kStatus_SDHC_NoError if success
 */
sdhc_status_t sdhc_init(uint8_t instance, sdhc_host_t * host, sdhc_init_config_t *config);

/*!
 * @brief destory host controller
 *
 * @param host the pointer to the host controller about to be destoried
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
 * @brief Check read only for the attached card 
 *
 * @param host the pointer to the host controller
 * @return kStatus_SDHC_NoError on success
 */
sdhc_status_t sdhc_check_ro(sdhc_host_t *host);

/*!
 * @brief configure the specified host controller.
 *
 * With this function, user can modify the configuration of the
 * specific SDHC.
 *
 * @param host the pointer to the host controller
 * @param config the pointer to configration information
 * @return kStatus_SDHC_NoError on success
 */
sdhc_status_t sdhc_config_host(sdhc_host_t *host, sdhc_host_config_t *config);

/*!
 * @brief issue request on specific host controller and return on completion
 *
 * This function will issue the request to the card on the specific SDHC.
 * In which, the command will be sent and it will be blocked as long as
 * the response/data sending back from the card.
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

