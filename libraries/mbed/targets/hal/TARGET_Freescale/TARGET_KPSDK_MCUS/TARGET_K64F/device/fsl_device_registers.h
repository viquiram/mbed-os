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
#ifndef __FSL_DEVICE_REGISTERS_H__
#define __FSL_DEVICE_REGISTERS_H__

/*
 * Include the cpu specific register header files.
 *
 * The CPU macro should be declared in the project or makefile.
 */

    #define K64F12_SERIES
    /* Extension register headers. (These will eventually be merged into the CMSIS-style header.)*/
    #include "MK64F12_adc.h"
    #include "MK64F12_aips.h"
    #include "MK64F12_axbs.h"
    #include "MK64F12_can.h"
    #include "MK64F12_cau.h"
    #include "MK64F12_cmp.h"
    #include "MK64F12_cmt.h"
    #include "MK64F12_crc.h"
    #include "MK64F12_dac.h"
    #include "MK64F12_dma.h"
    #include "MK64F12_dmamux.h"
    #include "MK64F12_enet.h"
    #include "MK64F12_ewm.h"
    #include "MK64F12_fb.h"
    #include "MK64F12_fmc.h"
    #include "MK64F12_ftfe.h"
    #include "MK64F12_ftm.h"
    #include "MK64F12_gpio.h"
    #include "MK64F12_i2c.h"
    #include "MK64F12_i2s.h"
    #include "MK64F12_llwu.h"
    #include "MK64F12_lptmr.h"
    #include "MK64F12_mcg.h"
    #include "MK64F12_mcm.h"
    #include "MK64F12_mpu.h"
    #include "MK64F12_nv.h"
    #include "MK64F12_osc.h"
    #include "MK64F12_pdb.h"
    #include "MK64F12_pit.h"
    #include "MK64F12_pmc.h"
    #include "MK64F12_port.h"
    #include "MK64F12_rcm.h"
    #include "MK64F12_rfsys.h"
    #include "MK64F12_rfvbat.h"
    #include "MK64F12_rng.h"
    #include "MK64F12_rtc.h"
    #include "MK64F12_sdhc.h"
    #include "MK64F12_sim.h"
    #include "MK64F12_smc.h"
    #include "MK64F12_spi.h"
    #include "MK64F12_uart.h"
    #include "MK64F12_usb.h"
    #include "MK64F12_usbdcd.h"
    #include "MK64F12_vref.h"
    #include "MK64F12_wdog.h"

    /* CMSIS-style register definitions*/
    #include "MK64F12.h"


#endif /* __FSL_DEVICE_REGISTERS_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
