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
#include <stdlib.h>
#include <string.h>
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "fsl_os_abstraction.h"
#include "fsl_sdhc_hal.h"
#include "fsl_sdhc_driver.h"
#include "fsl_sdhc_features.h"
#include "sdhc.h"
#include "sdmmc.h"

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_init_card
 * Description: Initialize card by sending 80 clocks to card
 *
 *END*********************************************************************/
static void sdhc_init_card(sdhc_host_t *host)
{
    assert(host);

    sdhc_hal_init_card(host->instance);
    while(sdhc_hal_is_init_card_done(host->instance)) {}
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_select_clk
 * Description: Select clock source for specific host controller
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_select_clk(sdhc_host_t *host, uint32_t clkSource)
{
    assert(host);

    /* TODO: select clock source interface in clock manager */
    if (clock_manager_get_frequency_by_source((clock_source_names_t)clkSource, &host->maxClock) != kClockManagerSuccess)
    {
        return kStatus_SDHC_Failed;
    }
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_enable_clk
 * Description: Enable clock for specific host controller
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_enable_clk(sdhc_host_t *host, bool isEnabled)
{
    assert(host);

    if (clock_manager_set_gate(kClockModuleESDHC, host->instance, isEnabled) != kClockManagerSuccess)
    {
        return kStatus_SDHC_Failed;
    }
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_is_card_present
 * Description: Check whether card is inserted
 *
 *END*********************************************************************/
static bool sdhc_is_card_present(sdhc_host_t *host)
{
    assert(host);
    uint32_t isInserted;
    uint32_t irqStat = sdhc_hal_get_intr_flags(host->instance);
    if (irqStat & SDHC_INT_CARD_REMOVE) {
        isInserted = 0;
    } else {
        isInserted = 1;
    }
    sdhc_hal_clear_intr_flags(host->instance, SDHC_INT_CARD_REMOVE | SDHC_INT_CARD_INSERT);
    return isInserted;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_wait_for_request_done
 * Description: wait the completion of the request for the
 *      specific host controller
 *
 *END*********************************************************************/
static void sdhc_wait_for_request_done(sdhc_host_t *host, sdhc_request_t *req)
{
    if ((!host) || (!req))
    {
        return;
    }
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_set_clock
 * Description: configure clock of host controller, it will set the most
 *      close clock frequency to the given clock
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_set_clock(sdhc_host_t *host, uint32_t clock)
{
    assert(host);
    uint32_t divisor = SDHC_HAL_INITIAL_DVS, freq = SDHC_HAL_INITIAL_CLKFS;

    sdhc_hal_enable_ipg_clk(host->instance, false);
    sdhc_hal_enable_sys_clk(host->instance, false);
    sdhc_hal_enable_per_clk(host->instance, false);
    sdhc_hal_enable_sd_clk(host->instance, false);

    if (clock > 0)
    {
        while((host->maxClock / freq / SDHC_HAL_MAX_DVS > clock) && (freq < SDHC_HAL_MAX_CLKFS))
        {
            SDHC_HAL_NEXT_CLKFS(freq);
        }
        while((host->maxClock / freq / divisor > clock) && (divisor < SDHC_HAL_MAX_DVS))
        {
            SDHC_HAL_NEXT_DVS(divisor);
        }

        clock = host->maxClock / freq / divisor;
        SDHC_HAL_PREV_CLKFS(freq);
        SDHC_HAL_PREV_DVS(divisor);

        sdhc_hal_set_clk_div(host->instance, divisor);
        sdhc_hal_set_clk_freq(host->instance, freq);
        sdhc_hal_set_data_timeout(host->instance, 0xE);

        sdhc_hal_enable_ipg_clk(host->instance, true);
        sdhc_hal_enable_sys_clk(host->instance, true);
        sdhc_hal_enable_per_clk(host->instance, true);

        while(!sdhc_hal_is_sd_clk_stable(host->instance)) {}
        sdhc_hal_enable_sd_clk(host->instance, true);
    }

    host->clock = clock;
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_set_bus_width
 * Description: Configure bus width of host controller
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_set_bus_width(sdhc_host_t *host, uint32_t busWidth)
{
    assert(host);
    sdhc_hal_dtw_t dtw = kSDHC_HAL_DTW_1Bit;

    switch(busWidth)
    {
        case SD_BUS_WIDTH_1BIT:
            dtw = kSDHC_HAL_DTW_1Bit;
            break;
        case SD_BUS_WIDTH_4BIT:
            dtw = kSDHC_HAL_DTW_4Bit;
            break;
        default:
            return kStatus_SDHC_InvalidParameter;
    }
    sdhc_hal_set_data_trans_width(host->instance, dtw);
    host->busWidth = busWidth;
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_set_power_mode
 * Description: Configure power mode of host controller
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_set_power_mode(sdhc_host_t *host, sdhc_power_mode_t powerMode)
{
    if ((!host) || ((powerMode != kSdhcPowerModeRunning)
                && (powerMode != kSdhcPowerModeSuspended)
                && (powerMode != kSdhcPowerModeStopped)))
    {
        return kStatus_SDHC_InvalidParameter;
    }
    host->powerMode = powerMode;
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_reset
 * Description: Reset host controller accord to the given mask
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_reset(sdhc_host_t *host, uint32_t mask)
{
    assert(host);
    if (!(mask & (SDHC_RESET_ALL | SDHC_RESET_DATA | SDHC_RESET_CMD)))
    {
        return kStatus_SDHC_InvalidParameter;
    }

    if (mask & SDHC_RESET_ALL)
    {
        host->clock = 0;
        sdhc_hal_reset(host->instance, SDHC_HAL_RST_TYPE_ALL);
        while(!sdhc_hal_is_reset_done(host->instance, SDHC_HAL_RST_TYPE_ALL)) {}
    }
    else if (mask == (SDHC_RESET_DATA | SDHC_RESET_CMD))
    {
        sdhc_hal_reset(host->instance, (SDHC_HAL_RST_TYPE_DATA | SDHC_HAL_RST_TYPE_CMD));
        while(!sdhc_hal_is_reset_done(host->instance, (SDHC_HAL_RST_TYPE_DATA | SDHC_HAL_RST_TYPE_CMD))) {}
    }
    else if (mask == SDHC_RESET_CMD)
    {
        sdhc_hal_reset(host->instance, SDHC_HAL_RST_TYPE_CMD);
        while(!sdhc_hal_is_reset_done(host->instance, SDHC_HAL_RST_TYPE_CMD)) {}
    }
    else if (mask == SDHC_RESET_DATA)
    {
        sdhc_hal_reset(host->instance, SDHC_HAL_RST_TYPE_DATA);
        while(!sdhc_hal_is_reset_done(host->instance, SDHC_HAL_RST_TYPE_DATA)) {}
    }
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_wait_intr
 * Description: Wait for specific interrupts
 *
 *END*********************************************************************/
static uint32_t sdhc_wait_intr(sdhc_host_t *host, uint32_t mask)
{
    assert(host);
    uint32_t irq = 0;
    do
    {
        irq = (sdhc_hal_get_intr_flags(host->instance) & mask);
    }
    while (!irq);

    return irq;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_set_max_blksz
 * Description: Set max block size supported for the host controller
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_set_max_blksz(sdhc_host_t *host)
{
    uint32_t mbl = sdhc_hal_get_max_blklen(host->instance);

    if (!host)
    {
        return kStatus_SDHC_InvalidParameter;
    }

    switch (mbl)
    {
        case SDHC_HAL_MAX_BLKLEN_512B:
            host->maxBlockSize = 512;
            break;
        case SDHC_HAL_MAX_BLKLEN_1024B:
            host->maxBlockSize = 1024;
            break;
        case SDHC_HAL_MAX_BLKLEN_2048B:
            host->maxBlockSize = 2048;
            break;
        case SDHC_HAL_MAX_BLKLEN_4096B:
            host->maxBlockSize = 4096;
            break;
        default:
            return kStatus_SDHC_Failed;
    }

    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_get_host_version
 * Description: Set host version for the host controller
 *
 *END*********************************************************************/
static void sdhc_get_host_version(sdhc_host_t *host)
{
    if (!host)
    {
        return;
    }

    host->specVer = sdhc_hal_get_spec_ver(host->instance);
    host->vendorVer = sdhc_hal_get_vendor_ver(host->instance);
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_init
 * Description: Initialize host controller by specific instance index.
 *
 *END*********************************************************************/
sdhc_status_t sdhc_init(uint8_t instance, sdhc_host_t * host, sdhc_init_config_t *config)
{
    uint32_t irqEnabled;

    if ((!sdhc_hal_is_valid_instance(instance)) || (!config) || (!host))
    {
        return kStatus_SDHC_InvalidParameter;
    }

    memset(host, 0, sizeof(sdhc_host_t));

    host->instance = instance;

    sdhc_enable_clk(host, false);

    if (sdhc_select_clk(host, kClockPllfllSel))     /* Use core clock as source */
    {
        return kStatus_SDHC_Failed;
    }
    sdhc_enable_clk(host, true);
    sdhc_reset(host, SDHC_RESET_ALL);
    sdhc_hal_enable_ext_dma_req(host->instance, false);

    sdhc_get_host_version(host);
    if (kStatus_SDHC_NoError != sdhc_set_max_blksz(host))
    {
        return kStatus_SDHC_Failed;
    }

    host->ocrSupported = SD_OCR_VDD_29_30 | SD_OCR_VDD_32_33 | SD_OCR_VDD_33_34;

    /* enable irqs */
    sdhc_hal_enable_intr_state(host->instance, false, SDHC_INT_ALL_MASK);
    sdhc_hal_enable_intr_signal(host->instance, false, SDHC_INT_ALL_MASK);
    irqEnabled = SDHC_INT_E_CMD_INDEX | SDHC_INT_E_CMD_CRC | SDHC_INT_E_CMD_END_BIT | SDHC_INT_E_CMD_TIMEOUT
                | SDHC_INT_E_DATA_CRC | SDHC_INT_E_DATA_TIMEOUT | SDHC_INT_E_DATA_END_BIT | SDHC_INT_RBUF_READY
                | SDHC_INT_WBUF_READY | SDHC_INT_CMD_DONE | SDHC_INT_TRANSFER_DONE | SDHC_INT_CARD_INSERT | SDHC_INT_CARD_REMOVE;
    sdhc_hal_enable_intr_state(host->instance, true, irqEnabled);
#if ! defined(SDHC_CARD_DETECT_IRQ_ENABLED)
    irqEnabled &= ~(SDHC_INT_CARD_INSERT | SDHC_INT_CARD_REMOVE);
#endif
    sdhc_hal_enable_intr_signal(host->instance, true, irqEnabled);

#if defined(BIG_ENDIAN)
    host->endian = kSDHC_HAL_ENDIAN_Big;
#else
    host->endian = kSDHC_HAL_ENDIAN_Little;
#endif
    sdhc_hal_set_endian(host->instance, host->endian);

    sdhc_hal_set_write_watermark(host->instance, 1);
    sdhc_hal_set_read_watermark(host->instance, 1);

    sync_create(&host->host_lock, 1);

    sdhc_set_power_mode(host, kSdhcPowerModeRunning);
    sdhc_set_bus_width(host, config->busWidth);
    sdhc_set_clock(host, config->clock);

    sdhc_hal_enable_d3cd(host->instance, true);
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_shutdown
 * Description: Deinitialize host controller
 *
 *END*********************************************************************/
void sdhc_shutdown(sdhc_host_t *host)
{
    assert(host);
    sdhc_enable_clk(host, false);
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_check_card
 * Description: check whether the card is present on specified host
 *      controller.
 *
 *END*********************************************************************/
sdhc_status_t sdhc_check_card(sdhc_host_t *host, sdhc_card_t *card)
{
    if ((!host) || (!card))
    {
        return kStatus_SDHC_InvalidParameter;
    }

    sdhc_enable_clk(host, true);
    sdhc_init_card(host);

    if (!sdhc_is_card_present(host))
    {
        sdhc_enable_clk(host, false);
        return kStatus_SDHC_NoMedium;
    }

    host->card = card;
    card->cardType = kCardTypeUnknown;
    card->host = host;
    sdhc_enable_clk(host, false);
    return kStatus_SDHC_NoError;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_send_command
 * Description: Send command to card
 *
 *END*********************************************************************/
static sdhc_status_t sdhc_send_command(sdhc_host_t *host, sdhc_command_t *cmd)
{
    sdhc_status_t ret = kStatus_SDHC_NoError;
    uint32_t flags = 0;
    if ((!host) || (!cmd))
    {
        return kStatus_SDHC_InvalidParameter;
    }

    while(sdhc_hal_is_cmd_inhibit(host->instance)) {}

    if (cmd->data)
    {
        while(sdhc_hal_is_data_inhibit(host->instance)) {}
    }

    while(sdhc_hal_is_data_line_active(host->instance)) {}

    host->currentCmd = cmd;

    if (cmd->index == kStopTransmission)
    {
        flags |= SDHC_HAL_CMD_TYPE_ABORT;
    }

    if (!(cmd->flags & SDMMC_RSP_PRESENT))
    {
        flags |= SDHC_HAL_NO_RESPONE;
    }
    else if (cmd->flags & SDMMC_RSP_136BITS)
    {
        flags |= SDHC_HAL_RESP_LEN_136;
    }
    else if (cmd->flags & SDMMC_RSP_BUSY)
    {
        flags |= SDHC_HAL_RESP_LEN_48_BC;
    }
    else
    {
        flags |= SDHC_HAL_RESP_LEN_48;
    }

    if (cmd->flags & SDMMC_RSP_CRC)
    {
        flags |= SDHC_HAL_ENABLE_CRC_CHECK;
    }
    if (cmd->flags & SDMMC_RSP_CHK_IDX)
    {
        flags |= SDHC_HAL_ENABLE_INDEX_CHECK;
    }

    if (cmd->data)
    {
        flags |= SDHC_HAL_DATA_PRESENT;

        if (cmd->data->flags & SDMMC_DATA_READ)
        {
            flags |= SDHC_HAL_ENABLE_DATA_READ;
        }

        if (cmd->data->blockCount > 1)
        {
            flags |= SDHC_HAL_MULTIPLE_BLOCK;
            flags |= SDHC_HAL_ENABLE_BLOCK_COUNT;
        }
        else
        {
            flags |= SDHC_HAL_SINGLE_BLOCK;
        }
        if (cmd->data->blockCount == ((uint32_t) -1))
        {
            sdhc_hal_set_blksz(host->instance, cmd->data->blockSize);
            sdhc_hal_set_blkcnt(host->instance, SDHC_HAL_MAX_BLOCK_COUNT);
            flags &= ~SDHC_HAL_ENABLE_BLOCK_COUNT;
        }
        else
        {
            sdhc_hal_set_blksz(host->instance, cmd->data->blockSize);
            sdhc_hal_set_blkcnt(host->instance, cmd->data->blockCount);
        }
    }
    else
    {
        sdhc_hal_set_blksz(host->instance, 0);
        sdhc_hal_set_blkcnt(host->instance, 0);
    }

    sdhc_hal_set_cmd_arg(host->instance, cmd->argument);
    sdhc_hal_set_dma_addr(host->instance, 0);
    sdhc_hal_send_cmd(host->instance, cmd->index, flags);

    return ret;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_issue_request
  * Description: Isuue request on specific host controller and return
 *      on completion.
 *
 *END*********************************************************************/
sdhc_status_t sdhc_issue_request(sdhc_host_t *host, sdhc_request_t *req)
{
    sdhc_status_t ret = kStatus_SDHC_NoError;
    fsl_rtos_status syncStatus;
    uint32_t mask = 0, opMask = 0, irqFlags = 0, timeout, i, j;
    if ((!host) || (!req))
    {
        return kStatus_SDHC_InvalidParameter;
    }

    do
    {
        syncStatus = sync_wait(&host->host_lock, kSyncWaitForever);
    }while(syncStatus == kIdle);
    
    sdhc_enable_clk(host, true);

    if (!sdhc_is_card_present(host))
    {
        host->currentReq = 0;
        host->currentCmd = 0;
        host->currentData = 0;
        sdhc_enable_clk(host, false);
        sync_signal(&host->host_lock);
        return kStatus_SDHC_NoMedium;
    }

    if (host->currentReq)
    {
        sdhc_wait_for_request_done(host, host->currentReq);
    }

    host->currentReq = req;

    if (kStatus_SDHC_NoError != sdhc_send_command(host, req->cmd))
    {
        host->currentReq = 0;
        host->currentCmd = 0;
        host->currentData = 0;
        sdhc_enable_clk(host, false);
        sync_signal(&host->host_lock);
        return kStatus_SDHC_Failed;
    }
    mask = SDHC_HAL_CMD_COMPLETE_INT | SDHC_HAL_CMD_CRC_ERR_INT | SDHC_HAL_CMD_INDEX_ERR_INT
        | SDHC_HAL_CMD_END_BIT_ERR_INT | SDHC_HAL_CMD_TIMEOUT_ERR_INT;
    irqFlags = sdhc_wait_intr(host, mask);

    if (irqFlags != SDHC_HAL_CMD_COMPLETE_INT)
    {
        sdhc_hal_clear_intr_flags(host->instance, mask);
        host->currentReq = 0;
        host->currentCmd = 0;
        host->currentData = 0;
        sdhc_enable_clk(host, false);
        sync_signal(&host->host_lock);
        return kStatus_SDHC_Failed;
    }

    sdhc_hal_clear_intr_flags(host->instance, SDHC_HAL_CMD_COMPLETE_INT);
    if (!sdhc_is_card_present(host))
    {
        host->currentReq = 0;
        host->currentCmd = 0;
        host->currentData = 0;
        sdhc_enable_clk(host, false);
        sync_signal(&host->host_lock);
        return kStatus_SDHC_Failed;
    }
    if (req->cmd->flags & SDMMC_RSP_BUSY)
    {
        timeout = 1000;
        while (timeout > 0)
        {
            if (sdhc_hal_get_data_line_level(host->instance) & SDHC_HAL_DAT0_LEVEL)
            {
                break;
            }
            timeout--;
        }
        if (!timeout)
        {
            host->currentReq = 0;
            host->currentCmd = 0;
            host->currentData = 0;
            sdhc_enable_clk(host, false);
            sync_signal(&host->host_lock);
            return kStatus_SDHC_Failed;
        }
    }

    if (host->currentCmd->flags & SDMMC_RSP_PRESENT)
    {
        sdhc_hal_get_resp(host->instance, host->currentCmd->response);
        if (!(host->currentCmd->flags & SDMMC_RSP_136BITS))
        {
            host->currentCmd->response[1] = 0;
            host->currentCmd->response[2] = 0;
            host->currentCmd->response[3] = 0;
        }
    }

    if (req->cmd->data)
    {
        mask = SDHC_HAL_TRANS_COMPLETE_INT | SDHC_HAL_DATA_TIMEOUT_ERR_INT | SDHC_HAL_DATA_CRC_ERR_INT
            | SDHC_HAL_DATA_END_BIT_ERR_INT;
        if ((req->cmd->data->flags & SDMMC_DATA_READ))
        {
            opMask |= SDHC_HAL_BUF_READ_READY_INT;
        }
        else
        {
            opMask |= SDHC_HAL_BUF_WRITE_READY_INT;
        }
        for (i = 0; i < req->cmd->data->blockCount; i++)
        {
            for (j = 0; j < req->cmd->data->blockSize >> 2; j++)
            {
                irqFlags = sdhc_wait_intr(host, mask | opMask);
                if (irqFlags & (SDHC_HAL_DATA_TIMEOUT_ERR_INT | SDHC_HAL_DATA_CRC_ERR_INT | SDHC_HAL_DATA_END_BIT_ERR_INT))
                {
                    sdhc_hal_clear_intr_flags(host->instance, mask);
                    host->currentReq = 0;
                    host->currentCmd = 0;
                    host->currentData = 0;
                    sdhc_enable_clk(host, false);
                    sync_signal(&host->host_lock);
                    return kStatus_SDHC_Failed;
                }
                if (irqFlags & opMask)
                {
                    if ((req->cmd->data->flags & SDMMC_DATA_READ))
                    {
                        req->cmd->data->buffer[(i * (req->cmd->data->blockSize >> 2)) + j] = sdhc_hal_get_data(host->instance);
                    }
                    else
                    {
                        sdhc_hal_set_data(host->instance, req->cmd->data->buffer[(i * (req->cmd->data->blockSize >> 2)) + j]);
                    }
                    sdhc_hal_clear_intr_flags(host->instance, opMask);
                }
                req->cmd->data->bytesTransferred += 4;
            }
        }
        while(!(sdhc_wait_intr(host, mask) & SDHC_HAL_TRANS_COMPLETE_INT)) {}
        sdhc_hal_clear_intr_flags(host->instance, mask);
    }

    host->currentReq = 0;
    host->currentCmd = 0;
    host->currentData = 0;
    sdhc_enable_clk(host, false);
    sync_signal(&host->host_lock);
    return ret;
}

/*FUNCTION****************************************************************
 *
 * Function Name: sdhc_config_host
 * Description: Configure the specified host controller.
 *
 *END*********************************************************************/
sdhc_status_t sdhc_config_host(sdhc_host_t *host, sdhc_host_config_t *config)
{
    sdhc_status_t ret = kStatus_SDHC_NoError;
    fsl_rtos_status syncStatus;

    if ((!host) || (!config))
    {
        return kStatus_SDHC_InvalidParameter;
    }

    do
    {
        syncStatus = sync_wait(&host->host_lock, kSyncWaitForever);
    }while(syncStatus == kIdle);

    sdhc_enable_clk(host, true);

    if (host->clock != config->clock)
    {
        ret = sdhc_set_clock(host, config->clock);
    }

    if (host->busWidth != config->busWidth)
    {
        ret = sdhc_set_bus_width(host, config->busWidth);
    }

    if (host->powerMode != config->powerMode)
    {
        ret = sdhc_set_power_mode(host, config->powerMode);
    }

    sdhc_enable_clk(host, true);
    sync_signal(&host->host_lock);
    return ret;
}

/*************************************************************************************************
 * EOF
 ************************************************************************************************/

