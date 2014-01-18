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
 
#include "fsl_edma_driver.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if defined (K64F12_SERIES)
/*!
 * @brief Table to save eDMA IRQ enum numbers defined in CMSIS files. 
 *
 * This table is indexed by channel number which could return eDMA IRQ numbers.
 */
IRQn_Type edma_irq_ids[HW_DMA_INSTANCE_COUNT][FSL_FEATURE_DMA_MODULE_CHANNEL] =
{
    {DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn, DMA4_IRQn, DMA5_IRQn,
     DMA6_IRQn, DMA7_IRQn, DMA8_IRQn, DMA9_IRQn, DMA10_IRQn, DMA11_IRQn,
     DMA12_IRQn, DMA13_IRQn, DMA14_IRQn, DMA15_IRQn}
};
/*!
 * @brief Table to save eDMA ERROR IRQ enum numbers defined in CMSIS files. 
 *
 * This table is indexed by module number which would return eDMA error IRQ numbers.
 */
IRQn_Type edma_error_irq_ids[HW_DMA_INSTANCE_COUNT] =
{
    {DMA_Error_IRQn}
};
#elif defined (K70F12_SERIES)
/*!
 * @brief Table to save eDMA IRQ enum numbers defined in CMSIS files. 
 *
 * This table is indexed by channel number which could return eDMA IRQ numbers.
 */
IRQn_Type edma_irq_ids[HW_DMA_INSTANCE_COUNT][FSL_FEATURE_DMA_MODULE_CHANNEL] =
{
    {DMA0_DMA16_IRQn, DMA1_DMA17_IRQn, DMA2_DMA18_IRQn, DMA3_DMA19_IRQn,
     DMA4_DMA20_IRQn, DMA5_DMA21_IRQn, DMA6_DMA22_IRQn, DMA7_DMA23_IRQn,
     DMA8_DMA24_IRQn, DMA9_DMA25_IRQn, DMA10_DMA26_IRQn, DMA11_DMA27_IRQn,
     DMA12_DMA28_IRQn, DMA13_DMA29_IRQn, DMA14_DMA30_IRQn, DMA15_DMA31_IRQn}
};
/*!
 * @brief Table to save eDMA ERROR IRQ enum numbers defined in CMSIS files. 
 *
 * This table is indexed by module number which would return eDMA error IRQ numbers.
 */
IRQn_Type edma_error_irq_ids[HW_DMA_INSTANCE_COUNT] =
{
    {DMA_Error_IRQn}
};
#elif defined (K22F51212_SERIES)
/*!
 * @brief Table to save eDMA IRQ enum numbers defined in CMSIS files. 
 *
 * This table is indexed by channel number which could return eDMA IRQ numbers.
 * Channel n share the same irq number with channel (n + 16).
 */
IRQn_Type edma_irq_ids[HW_DMA_INSTANCE_COUNT][FSL_FEATURE_DMA_MODULE_CHANNEL] =
{
    {DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn, DMA4_IRQn, DMA5_IRQn,
     DMA6_IRQn, DMA7_IRQn, DMA8_IRQn, DMA9_IRQn, DMA10_IRQn, DMA11_IRQn,
     DMA12_IRQn, DMA13_IRQn, DMA14_IRQn, DMA15_IRQn}
};
/*!
 * @brief Table to save eDMA ERROR IRQ enum numbers defined in CMSIS files. 
 *
 * This table is indexed by module number which would return eDMA error IRQ numbers.
 */
IRQn_Type edma_error_irq_ids[HW_DMA_INSTANCE_COUNT] =
{
    {DMA_Error_IRQn}
};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*! @brief Edma Handler*/
#if defined (K64F12_SERIES) || defined (K22F51212_SERIES)
/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA0_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(0);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA1_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(1);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA2_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(2);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA3_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(3);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA4_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(4);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA5_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(5);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA6_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(6);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA7_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(7);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA8_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(8);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA9_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(9);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA10_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(10);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA11_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(11);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA12_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(12);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA13_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(13);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA14_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(14);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA15_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(15);
}

/*! @brief EDMA ERROR IRQ handler with the same name in startup code*/
void DMA_Error_IRQHandler(void)
{
	DMA_ERR_IRQHandler(0);
}
#elif defined (K70F12_SERIES)
void DMA0_DMA16_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(0);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA1_DMA17_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(1);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA2_DMA18_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(2);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA3_DMA19_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(3);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA4_DMA20_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(4);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA5_DMA21_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(5);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA6_DMA22_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(6);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA7_DMA23_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(7);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA8_DMA24_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(8);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA9_DMA25_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(9);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA10_DMA26_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(10);
}


/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA11_DMA27_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(11);
}


/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA12_DMA28_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(12);
}


/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA13_DMA29_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(13);
}


/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA14_DMA30_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(14);
}

/*! @brief EDMA IRQ handler with the same name in startup code*/
void DMA15_DMA31_IRQHandler(void)
{
    EDMA_IRQ_HANDLER(15);
}
/*! @brief EDMA ERROR IRQ handler with the same name in startup code*/
void DMA_Error_IRQHandler(void)
{
	DMA_ERR_IRQHandler(0);
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/

