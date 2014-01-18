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
#ifndef __FSL_SDHC_HAL_H__
#define __FSL_SDHC_HAL_H__

/*! @addtogroup sdhc_hal */
/*! @{ */

/* PRSSTA */
#define SDHC_HAL_DAT0_LEVEL             (BM_SDHC_PRSSTAT_DLSL & (1 << 24))

/* XFERTYP */
#define SDHC_HAL_MAX_BLOCK_COUNT        (BS_SDHC_BLKATTR_BLKCNT - 1)
#define SDHC_HAL_ENABLE_DMA             BM_SDHC_XFERTYP_DMAEN
#define SDHC_HAL_DISABLE_DMA            (0U)

#define SDHC_HAL_CMD_TYPE_SUSPEND       (BF_SDHC_XFERTYP_CMDTYP(1))
#define SDHC_HAL_CMD_TYPE_RESUME        (BF_SDHC_XFERTYP_CMDTYP(2))
#define SDHC_HAL_CMD_TYPE_ABORT         (BF_SDHC_XFERTYP_CMDTYP(3))

#define SDHC_HAL_ENABLE_BLOCK_COUNT     BM_SDHC_XFERTYP_BCEN
#define SDHC_HAL_DISABLE_BLOCK_COUNT    (0U)

#define SDHC_HAL_ENABLE_AUTO_CMD12      BM_SDHC_XFERTYP_AC12EN
#define SDHC_HAL_DISABLE_AUTO_CMD12     (0U)

#define SDHC_HAL_ENABLE_DATA_READ       BM_SDHC_XFERTYP_DTDSEL
#define SDHC_HAL_DISABLE_AUTO_CMD12     (0U)

#define SDHC_HAL_MULTIPLE_BLOCK         BM_SDHC_XFERTYP_MSBSEL
#define SDHC_HAL_SINGLE_BLOCK           (0U)

#define SDHC_HAL_NO_RESPONE             (0U)
#define SDHC_HAL_RESP_LEN_136           ((0x1 << BP_SDHC_XFERTYP_RSPTYP) & BM_SDHC_XFERTYP_RSPTYP)
#define SDHC_HAL_RESP_LEN_48            ((0x2 << BP_SDHC_XFERTYP_RSPTYP) & BM_SDHC_XFERTYP_RSPTYP)
#define SDHC_HAL_RESP_LEN_48_BC         ((0x3 << BP_SDHC_XFERTYP_RSPTYP) & BM_SDHC_XFERTYP_RSPTYP)

#define SDHC_HAL_ENABLE_CRC_CHECK       BM_SDHC_XFERTYP_CCCEN
#define SDHC_HAL_DISABLE_CRC_CHECK      (0U)

#define SDHC_HAL_ENABLE_INDEX_CHECK     BM_SDHC_XFERTYP_CICEN
#define SDHC_HAL_DISABLE_INDEX_CHECK    (0U)

#define SDHC_HAL_DATA_PRESENT           BM_SDHC_XFERTYP_DPSEL
#define SDHC_HAL_NO_DATA_PRESENT        (0U)

/* SYSCTL */
#define SDHC_HAL_MAX_DVS                (16U)
#define SDHC_HAL_INITIAL_DVS            (1U)            /* initial value of divisor to calculate clock rate */
#define SDHC_HAL_INITIAL_CLKFS          (2U)            /* initial value of clock selector to calculate clock rate */
#define SDHC_HAL_NEXT_DVS(x)            (x += 1)
#define SDHC_HAL_PREV_DVS(x)            (x -= 1)
#define SDHC_HAL_MAX_CLKFS              (256U)
#define SDHC_HAL_NEXT_CLKFS(x)          (x <<= 1)
#define SDHC_HAL_PREV_CLKFS(x)          (x >>= 1)

/* IRQSTAT */
#define SDHC_HAL_CMD_COMPLETE_INT       BM_SDHC_IRQSTAT_CC
#define SDHC_HAL_TRANS_COMPLETE_INT     BM_SDHC_IRQSTAT_TC
#define SDHC_HAL_BLOCK_GAP_EVENT_INT    BM_SDHC_IRQSTAT_BGE
#define SDHC_HAL_DMA_INT                BM_SDHC_IRQSTAT_DMAE
#define SDHC_HAL_BUF_WRITE_READY_INT    BM_SDHC_IRQSTAT_BWR
#define SDHC_HAL_BUF_READ_READY_INT     BM_SDHC_IRQSTAT_BRR
#define SDHC_HAL_CARD_INSERTION_INT     BM_SDHC_IRQSTAT_CINS
#define SDHC_HAL_CARD_REMOVAL_INT       BM_SDHC_IRQSTAT_CRM
#define SDHC_HAL_CARD_INT               BM_SDHC_IRQSTAT_CINT
#define SDHC_HAL_CMD_TIMEOUT_ERR_INT    BM_SDHC_IRQSTAT_CTOE
#define SDHC_HAL_CMD_CRC_ERR_INT        BM_SDHC_IRQSTAT_CCE
#define SDHC_HAL_CMD_END_BIT_ERR_INT    BM_SDHC_IRQSTAT_CEBE
#define SDHC_HAL_CMD_INDEX_ERR_INT      BM_SDHC_IRQSTAT_CIE
#define SDHC_HAL_DATA_TIMEOUT_ERR_INT   BM_SDHC_IRQSTAT_DTOE
#define SDHC_HAL_DATA_CRC_ERR_INT       BM_SDHC_IRQSTAT_CCE
#define SDHC_HAL_DATA_END_BIT_ERR_INT   BM_SDHC_IRQSTAT_DEBE
#define SDHC_HAL_AUTO_CMD12_ERR_INT     BM_SDHC_IRQSTAT_AC12E
#define SDHC_HAL_DMA_ERR_INT            BM_SDHC_IRQSTAT_DMAE

/* AC12ERR */
#define SDHC_HAL_ACMD12_NOT_EXEC_ERR    BM_SDHC_AC12ERR_AC12NE
#define SDHC_HAL_ACMD12_TIMEOUT_ERR     BM_SDHC_AC12ERR_AC12TOE
#define SDHC_HAL_ACMD12_END_BIT_ERR     BM_SDHC_AC12ERR_AC12EBE
#define SDHC_HAL_ACMD12_CRC_ERR         BM_SDHC_AC12ERR_AC12CE
#define SDHC_HAL_ACMD12_INDEX_ERR       BM_SDHC_AC12ERR_AC12IE
#define SDHC_HAL_ACMD12_NOT_ISSUE_ERR   BM_SDHC_AC12ERR_CNIBAC12E

/* HTCAPBLT */
#define SDHC_HAL_SUPPORT_ADMA           BM_SDHC_HTCAPBLT_ADMAS
#define SDHC_HAL_SUPPORT_HIGHSPEED     BM_SDHC_HTCAPBLT_HSS
#define SDHC_HAL_SUPPORT_DMA            BM_SDHC_HTCAPBLT_DMAS
#define SDHC_HAL_SUPPORT_SUSPEND_RESUME BM_SDHC_HTCAPBLT_SRS
#define SDHC_HAL_SUPPORT_3_3_V          BM_SDHC_HTCAPBLT_VS33
#define SDHC_HAL_SUPPORT_3_0_V          BM_SDHC_HTCAPBLT_VS30
#define SDHC_HAL_SUPPORT_1_8_V          BM_SDHC_HTCAPBLT_VS18

/* FEVT */
#define SDHC_HAL_ACMD12_NOT_EXEC_ERR_EVENT  BM_SDHC_FEVT_AC12NE
#define SDHC_HAL_ACMD12_TIMEOUT_ERR_EVENT   BM_SDHC_FEVT_AC12TOE
#define SDHC_HAL_ACMD12_CRC_ERR_EVENT       BM_SDHC_FEVT_AC12CE
#define SDHC_HAL_ACMD12_END_BIT_ERR_EVENT   BM_SDHC_FEVT_AC12EBE
#define SDHC_HAL_ACMD12_INDEX_ERR_EVENT     BM_SDHC_FEVT_AC12IE
#define SDHC_HAL_ACMD12_NOT_ISSUE_ERR_EVENT BM_SDHC_FEVT_CNIBAC12E
#define SDHC_HAL_CMD_TIMEOUT_ERR_EVENT      BM_SDHC_FEVT_CTOE
#define SDHC_HAL_CMD_CRC_ERR_EVENT          BM_SDHC_FEVT_CCE
#define SDHC_HAL_CMD_END_BIT_ERR_EVENT      BM_SDHC_FEVT_CEBE
#define SDHC_HAL_CMD_INDEX_ERR_EVENT        BM_SDHC_FEVT_CIE
#define SDHC_HAL_DATA_TIMEOUT_ERR_EVENT     BM_SDHC_FEVT_DTOE
#define SDHC_HAL_DATA_CRC_ERR_EVENT         BM_SDHC_FEVT_DCE
#define SDHC_HAL_DATA_END_BIT_ERR_EVENT     BM_SDHC_FEVT_DEBE
#define SDHC_HAL_ACMD12_ERR_EVENT           BM_SDHC_FEVT_AC12E
#define SDHC_HAL_CARD_INT_EVENT             BM_SDHC_FEVT_CINT
#define SDHC_HAL_DMA_ERROR_EVENT            BM_SDHC_FEVT_DMAE

/* MMCBOOT */
#define SDHC_HAL_MMCBOOT_NORMAL_BOOT        (0U)
#define SDHC_HAL_MMCBOOT_ALTER_BOOT         (1U)

/* PROCTL */
#define SDHC_HAL_LED_OFF                    (0U)
#define SDHC_HAL_LED_ON                     (1U)

#define SDHC_HAL_1_BIT_MODE                 (0U)
#define SDHC_HAL_4_BIT_MODE                 (1U)
#define SDHC_HAL_8_BIT_MODE                 (2U)

#define SDHC_HAL_BIG_ENDIAN_MODE            (0U)
#define SDHC_HAL_HALF_WORD_BIG_ENDIAN_MODE  (1U)
#define SDHC_HAL_LITTLE_ENDIAN_MODE         (2U)

#define SDHC_HAL_NO_DMA_OR_SDMA_MODE        (0U)
#define SDHC_HAL_ADMA1_MODE                 (1U)
#define SDHC_HAL_ADMA2_MODE                 (2U)

#define SDHC_HAL_RST_TYPE_ALL               BM_SDHC_SYSCTL_RSTA
#define SDHC_HAL_RST_TYPE_CMD               BM_SDHC_SYSCTL_RSTC
#define SDHC_HAL_RST_TYPE_DATA              BM_SDHC_SYSCTL_RSTD

#define SDHC_HAL_MAX_BLKLEN_512B            (0U)
#define SDHC_HAL_MAX_BLKLEN_1024B           (1U)
#define SDHC_HAL_MAX_BLKLEN_2048B           (2U)
#define SDHC_HAL_MAX_BLKLEN_4096B           (3U)

/*************************************************************************************************
 * API
 ************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*! @name SDHC HAL FUNCTION */
/*@{ */

/*!
 * @brief Check if the given instance is valid
 *
 * @param instance sdhc instance id
 * @return true if valid
 */
static inline bool sdhc_hal_is_valid_instance(uint8_t instance)
{
    return (instance < HW_SDHC_INSTANCE_COUNT);
}

/*!
 * @brief Configure the dma address
 *
 * @param instance sdhc instance id
 * @param address the dma address
 */
static inline void sdhc_hal_set_dma_addr(uint8_t instance, uint32_t address)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    HW_SDHC_DSADDR_WR(BF_SDHC_DSADDR_DSADDR(address));
}

/*!
 * @brief Get the dma address
 *
 * @param instance sdhc instance id
 * @return the dma address
 */
static inline uint32_t sdhc_hal_get_dma_addr(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return HW_SDHC_DSADDR_RD;
}

/*!
 * @brief Get block size configured
 *
 * @param instance sdhc instance id
 * @return the block size already configured
 */
static inline uint32_t sdhc_hal_get_blksz(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_BLKATTR_BLKSIZE;
}

/*!
 * @brief Set block size
 *
 * @param instance sdhc instance id
 * @param blockSize the block size
 */
static inline void sdhc_hal_set_blksz(uint8_t instance, uint32_t blockSize)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_BLKATTR_BLKSIZE(blockSize);
}

/*!
 * @brief Set block count
 *
 * @param instance sdhc instance id
 * @param blockCount the block count
 */
static inline void sdhc_hal_set_blkcnt(uint8_t instance, uint32_t blockCount)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_BLKATTR_BLKCNT(blockCount);
}

/*!
 * @brief Get block count configured
 *
 * @param instance sdhc instance id
 * @return the block count already configured
 */
static inline uint32_t sdhc_hal_get_blkcnt(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_BLKATTR_BLKCNT;
}

/*!
 * @brief Configure command argument
 *
 * @param instance sdhc instance id
 * @param arg the command argument
 */
static inline void sdhc_hal_set_cmd_arg(uint8_t instance, uint32_t arg)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_CMDARG_CMDARG(arg);
}

/*!
 * @brief Send command
 *
 * @param instance sdhc instance id
 * @param index command index
 * @param flags transfer type flags
 */
static inline void sdhc_hal_send_cmd(uint8_t instance, uint32_t index, uint32_t flags)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    HW_SDHC_XFERTYP_WR(((index << BP_SDHC_XFERTYP_CMDINX) & BM_SDHC_XFERTYP_CMDINX)
            | (flags & ( BM_SDHC_XFERTYP_DMAEN | BM_SDHC_XFERTYP_MSBSEL | BM_SDHC_XFERTYP_DPSEL
                | BM_SDHC_XFERTYP_CMDTYP | BM_SDHC_XFERTYP_BCEN | BM_SDHC_XFERTYP_CICEN
                | BM_SDHC_XFERTYP_CCCEN | BM_SDHC_XFERTYP_RSPTYP | BM_SDHC_XFERTYP_DTDSEL
                | BM_SDHC_XFERTYP_AC12EN)));
}

/*!
 * @brief Get command response
 *
 * @param instance sdhc instance id
 * @param resp an array of response, 4 bytes
 */
static inline void sdhc_hal_get_resp(uint8_t instance, uint32_t * resp)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    resp[0] = BR_SDHC_CMDRSP0_CMDRSP0;
    resp[1] = BR_SDHC_CMDRSP1_CMDRSP1;
    resp[2] = BR_SDHC_CMDRSP2_CMDRSP2;
    resp[3] = BR_SDHC_CMDRSP3_CMDRSP3;
}

/*!
 * @brief Fill the data port
 *
 * @param instance sdhc instance id
 * @param data the data about to be sent
 */
static inline void sdhc_hal_set_data(uint8_t instance, uint32_t data)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    HW_SDHC_DATPORT_WR(data);
}

/*!
 * @brief Retrieve the data from data port
 *
 * @param instance sdhc instance id
 * @return data the data read
 */
static inline uint32_t sdhc_hal_get_data(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_DATPORT_DATCONT;
}

/*!
 * @brief Check if command inhibit bit is set or not
 *
 * @param instance sdhc instance id
 * @return 1 if command inhibit, 0 if not.
 */
static inline uint32_t sdhc_hal_is_cmd_inhibit(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_CIHB;
}

/*!
 * @brief Check if data inhibit bit is set or not
 *
 * @param instance sdhc instance id
 * @return 1 if data inhibit, 0 if not.
 */
static inline uint32_t sdhc_hal_is_data_inhibit(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_CDIHB;
}

/*!
 * @brief Check if data line is active
 *
 * @param instance sdhc instance id
 * @return 1 if it's active, 0 if not.
 */
static inline uint32_t sdhc_hal_is_data_line_active(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_DLA;
}

/*!
 * @brief Check if SD clock is stable or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's stable, 0 if not.
 */
static inline uint32_t sdhc_hal_is_sd_clk_stable(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_SDSTB;
}

/*!
 * @brief Check if IPG clock is off or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's off, 0 if not.
 */
static inline uint32_t sdhc_hal_is_ipg_clk_off(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_IPGOFF;
}

/*!
 * @brief Check if system clock is off or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's off, 0 if not.
 */
static inline uint32_t sdhc_hal_is_sys_clk_off(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_HCKOFF;
}

/*!
 * @brief Check if peripheral clock is off or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's off, 0 if not.
 */
static inline uint32_t sdhc_hal_is_per_clk_off(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_PEROFF;
}

/*!
 * @brief Check if SD clock is off or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's off, 0 if not.
 */
static inline uint32_t sdhc_hal_is_sd_clk_off(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_SDOFF;
}

/*!
 * @brief Check if write transfer is active or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's active, 0 if not.
 */
static inline uint32_t sdhc_hal_is_write_trans_active(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_WTA;
}

/*!
 * @brief Check if read transfer is active or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's off, 0 if not.
 */
static inline uint32_t sdhc_hal_is_read_trans_active(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_RTA;
}

/*!
 * @brief Check if buffer write is enabled or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's isEnabledd, 0 if not.
 */
static inline uint32_t sdhc_hal_is_buf_write_enabled(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_BWEN;
}

/*!
 * @brief Check if buffer read is enabled or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's isEnabledd, 0 if not.
 */
static inline uint32_t sdhc_hal_is_buf_read_enabled(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_BREN;
}

/*!
 * @brief Check if card is inserted or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's inserted, 0 if not.
 */
static inline uint32_t sdhc_hal_is_card_inserted(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_CINS;
}

/*!
 * @brief Check if command line signal is high or not
 *
 * @param instance sdhc instance id
 * @return 1 if it's high, 0 if not.
 */
static inline uint32_t sdhc_hal_is_cmd_line_level_high(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_CLSL;
}

/*!
 * @brief Get data line signal level or not
 *
 * @param instance sdhc instance id
 * @return [7:0] data line signal level
 */
static inline uint32_t sdhc_hal_get_data_line_level(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PRSSTAT_DLSL;
}

/*!
 * @brief Set LED state
 *
 * @param instance sdhc instance id
 * @param state the LED state
 */
static inline void sdhc_hal_set_led_state(uint8_t instance, uint32_t state)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_LCTL(state);
}

/*!
 * @brief Set data transfer width
 *
 * @param instance sdhc instance id
 * @param dtw data transfer width
 */
static inline void sdhc_hal_set_data_trans_width(uint8_t instance, uint32_t dtw)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_DTW(dtw);
}

/*!
 * @brief Check if DAT3 is taken as card detect pin
 *
 * @param instance sdhc instance id
 */
static inline bool sdhc_hal_is_d3cd_enabled(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PROCTL_D3CD;
}

/*!
 * @brief Enable DAT3 as card detect pin
 *
 * @param instance sdhc instance id
 * @param isEnabled isEnabled the feature
 */
static inline void sdhc_hal_enable_d3cd(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_D3CD(isEnabled ? 1 : 0);
}

/*!
 * @brief Configure endian mode
 *
 * @param instance sdhc instance id
 * @param endianMode endian mode
 */
static inline void sdhc_hal_set_endian(uint8_t instance, uint32_t endianMode)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_EMODE(endianMode);
}

/*!
* @brief Get card detect test level
*
* @param instance sdhc instance id
* @return card detect test level
*/
static inline uint32_t sdhc_hal_get_cd_test_level(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_PROCTL_CDTL;
}

/*!
* @brief Enable card detect test
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_cd_test(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_CDSS(isEnabled ? 1 : 0);
}

/*!
* @brief Set DMA mode
*
* @param instance sdhc instance id
* @param dmaMode the DMA mode
*/
static inline void sdhc_hal_set_dma_mode(uint8_t instance, uint32_t dmaMode)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_DMAS(dmaMode);
}

/*!
* @brief Enable stop at block gap
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_stop_at_blkgap(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_SABGREQ(isEnabled ? 1 : 0);
}

/*!
* @brief Restart a transaction which has stopped at block gap
*
* @param instance sdhc instance id
*/
static inline void sdhc_hal_continue_req(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_CREQ(1);
}

/*!
* @brief Enable read wait control for SDIO cards.
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_read_wait_ctrl(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_RWCTL(isEnabled ? 1 : 0);
}

/*!
* @brief Enable stop at block gap requests
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_intr_stop_at_blk_gap(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_IABG(isEnabled ? 1 : 0);
}

/*!
* @brief Enable wakeup event on card interrupt
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_wakeup_on_card_intr(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_WECINT(isEnabled ? 1 : 0);
}

/*!
* @brief Enable wakeup event on card insertion
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_wakeup_on_card_ins(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_WECINS(isEnabled ? 1 : 0);
}

/*!
* @brief Enable wakeup event on card removal
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_wakeup_on_card_rm(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_PROCTL_WECRM(isEnabled ? 1 : 0);
}

/*!
* @brief Enable IPG clock, then no automatic clock gating off
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_ipg_clk(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_IPGEN(isEnabled ? 1 : 0);
}

/*!
* @brief Enable system clock, then no automatic clock gating off
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_sys_clk(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_HCKEN(isEnabled ? 1 : 0);
}

/*!
* @brief Enable peripheral clock, then no automatic clock gating off
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_per_clk(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_PEREN(isEnabled ? 1 : 0);
}

/*!
* @brief Enable SD clock. It should be disabled before changing SD clock
* frequency.
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_sd_clk(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_SDCLKEN(isEnabled ? 1 : 0);
}

/*!
* @brief Set SD clock frequency divisor
*
* @param instance sdhc instance id
* @param divisor the divisor
*/
static inline void sdhc_hal_set_clk_div(uint8_t instance, uint32_t divisor)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_DVS(divisor);
}

/*!
* @brief Set SD clock frequency select
*
* @param instance sdhc instance id
* @param freq the frequency selector
*/
static inline void sdhc_hal_set_clk_freq(uint8_t instance, uint32_t freq)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_SDCLKFS(freq);
}

/*!
* @brief Set data timeout counter value
*
* @param instance sdhc instance id
* @param timeout
*/
static inline void sdhc_hal_set_data_timeout(uint8_t instance, uint32_t timeout)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_DTOCV(timeout);
}

/*!
* @brief Perform kinds of SDHC reset
*
* @param instance sdhc instance id
* @param type the type of reset
*/
static inline void sdhc_hal_reset(uint8_t instance, uint32_t type)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    HW_SDHC_SYSCTL_SET(type & (BM_SDHC_SYSCTL_RSTA | BM_SDHC_SYSCTL_RSTC | BM_SDHC_SYSCTL_RSTD));
}

/*!
* @brief Check if the given SDHC reset is finished
*
* @param instance sdhc instance id
* @param type the type of reset
* @return if the given reset is done
*/
static inline uint32_t sdhc_hal_is_reset_done(uint8_t instance, uint32_t type)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return !(HW_SDHC_SYSCTL_RD
            & (type & (BM_SDHC_SYSCTL_RSTA | BM_SDHC_SYSCTL_RSTC | BM_SDHC_SYSCTL_RSTD)));
}

/*!
* @brief Send 80 SD clock cycles to card
*
* @param instance sdhc instance id
*/
static inline void sdhc_hal_init_card(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_SYSCTL_INITA(1);
}

/*!
* @brief Check if sending 80 SD clock cycles to card is finished
*
* @param instance sdhc instance id
* @return if sending 80 SD clock cycles is finished
*/
static inline uint32_t sdhc_hal_is_init_card_done(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return !(BR_SDHC_SYSCTL_INITA);
}

/*!
* @brief Get current interrupt status
*
* @param instance sdhc instance id
* @return current interrupt flags
*/
static inline uint32_t sdhc_hal_get_intr_flags(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return HW_SDHC_IRQSTAT_RD;
}

/*!
* @brief Clear specified interrupts' status
*
* @param instance sdhc instance id
* @param mask to specify interrupts' flags to be cleared
*/
static inline void sdhc_hal_clear_intr_flags(uint8_t instance, uint32_t mask)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    HW_SDHC_IRQSTAT_WR(mask);
}

/*!
* @brief Get currently enabled interrupts' singal
*
* @param instance sdhc instance id
* @return currently enabled interrupts' singal
*/
static inline uint32_t sdhc_hal_get_intr_signal(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return HW_SDHC_IRQSIGEN_RD;
}

/*!
* @brief Get currently enabled interrupts' state
*
* @param instance sdhc instance id
* @return currently enabled interrupts' state
*/
static inline uint32_t sdhc_hal_get_intr_state(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return HW_SDHC_IRQSTATEN_RD;
}

/*!
* @brief Enable specified interrupts
*
* @param instance sdhc instance id
* @param isEnabled enable or disable
* @param mask to specify interrupts to be isEnabledd
*/
static inline void sdhc_hal_enable_intr_signal(uint8_t instance, bool isEnabled, uint32_t mask)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    if (isEnabled)
    {
        HW_SDHC_IRQSIGEN_SET(mask);
    }
    else
    {
        HW_SDHC_IRQSIGEN_CLR(mask);
    }
}

/*!
* @brief Enable specified interrupts' state 
*
* @param instance sdhc instance id
* @param isEnabled enable or disable
* @param mask to specify interrupts' state to be enabled
*/
static inline void sdhc_hal_enable_intr_state(uint8_t instance, bool isEnabled, uint32_t mask)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    if (isEnabled)
    {
        HW_SDHC_IRQSTATEN_SET(mask);
    }
    else
    {
        HW_SDHC_IRQSTATEN_CLR(mask);
    }
}

/*!
* @brief Get auto cmd12 error
*
* @param instance sdhc instance id
* @return auto cmd12 error status
*/
static inline uint32_t sdhc_hal_get_ac12_error(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return HW_SDHC_AC12ERR_RD;
}

/*!
* @brief Get the max block length supported
*
* @param instance sdhc instance id
* @return the max block length support
*/
static inline uint32_t sdhc_hal_get_max_blklen(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HTCAPBLT_MBL;
}

/*!
* @brief Check if ADMA is supported
*
* @param instance sdhc instance id
* @return if ADMA is supported
*/
static inline uint32_t sdhc_hal_host_can_do_adma(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HTCAPBLT_ADMAS;
}

/*!
* @brief Check if high speed is supported
*
* @param instance sdhc instance id
* @return if high speed is supported
*/
static inline uint32_t sdhc_hal_host_can_do_highspeed(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HTCAPBLT_HSS;
}

/*!
* @brief Check if DMA is supported
*
* @param instance sdhc instance id
* @return if high speed is supported
*/
static inline uint32_t sdhc_hal_host_can_do_dma(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HTCAPBLT_DMAS;
}

/*!
* @brief Check if suspend resume is supported
*
* @param instance sdhc instance id
* @return if suspend and resume is supported
*/
static inline uint32_t sdhc_hal_host_can_do_suspend_resume(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HTCAPBLT_SRS;
}

/*!
* @brief Check if voltage 3.3 is supported
*
* @param instance sdhc instance id
* @return if voltage 3.3 is supported
*/
static inline uint32_t sdhc_hal_host_supports_v330(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HTCAPBLT_VS33;
}

/*!
* @brief Check if voltage 3.0 is supported
*
* @param instance sdhc instance id
* @return if voltage 3.0 is supported
*/
static inline uint32_t sdhc_hal_host_supports_v300(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
#if defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FN1M0VMJ15)
    return BR_SDHC_HTCAPBLT_VS30;
#else
    return 0;
#endif
}

/*!
* @brief Check if voltage 1.8 is supported
*
* @param instance sdhc instance id
* @return if voltage 1.8 is supported
*/
static inline uint32_t sdhc_hal_host_supports_v180(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
#if defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FN1M0VMJ15)
    return BR_SDHC_HTCAPBLT_VS18;
#else
    return 0;
#endif
}

/*!
* @brief Set watermark for writing
*
* @param instance sdhc instance id
* @param watermark for writing
*/
static inline void sdhc_hal_set_write_watermark(uint8_t instance, uint32_t watermark)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_WML_WRWML(watermark);
}

/*!
* @brief Set watermark for reading
*
* @param instance sdhc instance id
* @param watermark for reading
*/
static inline void sdhc_hal_set_read_watermark(uint8_t instance, uint32_t watermark)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_WML_RDWML(watermark);
}

/*!
* @brief Set force events according to the given mask
*
* @param instance sdhc instance id
* @param mask to specify the force events' flags to be set
*/
static inline void sdhc_hal_set_force_event_flags(uint8_t instance, uint32_t mask)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    HW_SDHC_FEVT_WR(mask);
}

/*!
* @brief Check if adma error is length mismatch
*
* @param instance sdhc instance id
* @return if adma error is length mismatch
*/
static inline uint32_t sdhc_hal_is_adma_len_mismatch_err(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_ADMAES_ADMALME;
}

/*!
* @brief Get back the state of adma error
*
* @param instance sdhc instance id
* @return error state
*/
static inline uint32_t sdhc_hal_get_adma_error_stat(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_ADMAES_ADMAES;
}

/*!
* @brief Check if adma error is descriptor error
*
* @param instance sdhc instance id
* @return if adma error is descriptor error
*/
static inline uint32_t sdhc_hal_is_adma_desc_err(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_ADMAES_ADMADCE;
}

/*!
* @brief Set adma address
*
* @param instance sdhc instance id
* @param address for adma transfer
*/
static inline void sdhc_hal_set_adma_addr(uint8_t instance, uint32_t address)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    HW_SDHC_ADSADDR_WR(BF_SDHC_ADSADDR_ADSADDR(address));
}

/*!
* @brief Enable external DMA request
*
* @param instance sdhc instance id
* @param isEnabled or not
*/
static inline void sdhc_hal_enable_ext_dma_req(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_VENDOR_EXTDMAEN(isEnabled ? 1 : 0);
}

/*!
* @brief Enable exact block number for SDIO CMD53
*
* @param instance sdhc instance id
* @param isEnabled or not
*/
static inline void sdhc_hal_enable_exact_blk_num(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_VENDOR_EXBLKNU(isEnabled ? 1 : 0);
}

/*!
* @brief Set timeout value for boot ACK
*
* @param instance sdhc instance id
* @param timeout
*/
static inline void sdhc_hal_set_boot_ack_timeout(uint8_t instance, uint32_t timeout)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_MMCBOOT_DTOCVACK(timeout);
}

/*!
* @brief Enable boot ACK
*
* @param instance sdhc instance id
* @param isEnabled
*/
static inline void sdhc_hal_enable_boot_ack(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_MMCBOOT_BOOTACK(isEnabled ? 1 : 0);
}

/*!
* @brief Configure boot mode
*
* @param instance sdhc instance id
* @param mode the boot mode
*/
static inline void sdhc_hal_set_boot_mode(uint8_t instance, uint32_t mode)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_MMCBOOT_BOOTMODE(mode);
}

/*!
* @brief Enable fast boot
*
* @param instance sdhc instance id
* @param isEnabled or not
*/
static inline void sdhc_hal_enable_fastboot(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_MMCBOOT_BOOTEN(isEnabled ? 1 : 0);
}

/*!
* @brief Enable automatic stop at block gap
*
* @param instance sdhc instance id
* @param isEnabled or not
*/
static inline void sdhc_hal_enable_auto_stop_at_blkgap(uint8_t instance, bool isEnabled)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_MMCBOOT_AUTOSABGEN(isEnabled ? 1 : 0);
}

/*!
* @brief Configure the block count for boot
*
* @param instance sdhc instance id
* @param blockCount the block count for boot
*/
static inline void sdhc_hal_set_boot_blkcnt(uint8_t instance, uint32_t blockCount)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    BW_SDHC_MMCBOOT_BOOTBLKCNT(blockCount);
}

/*!
* @brief Get specification version
*
* @param instance sdhc instance id
* @return specification version
*/
static inline uint32_t sdhc_hal_get_spec_ver(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HOSTVER_SVN;
}

/*!
* @brief Get vendor version
*
* @param instance sdhc instance id
* @return vendor version
*/
static inline uint32_t sdhc_hal_get_vendor_ver(uint8_t instance)
{
    assert(instance < HW_SDHC_INSTANCE_COUNT);
    return BR_SDHC_HOSTVER_VVN;
}

/*@} */

#if defined(__cplusplus)
}
#endif
/*! @} */

#endif

/*************************************************************************************************
 * EOF
 ************************************************************************************************/

