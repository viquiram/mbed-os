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

#include "fsl_mcg_hal.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : get_mcgffclk
 * Description   : Internal function to find the fll reference clock
 * This is an internal function to get the fll reference clock. The returned
 * value will be used for other APIs to calculate teh fll and other clock value.
 * 
 *END**************************************************************************/
uint32_t get_mcgffclk(void)
{
    uint32_t mcgffclk;
    uint8_t  divider;
    
    if ((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u) 
    {
        /* External reference clock is selected */
#if FSL_FEATURE_MCG_USE_OSCSEL          /* case 1: use oscsel for ffclk      */

        int32_t oscsel = (MCG->C7 & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT;
        if (oscsel == 0x0u) 
        {
#if FSL_FEATURE_MCG_HAS_OSC1          
            /* System oscillator 0 drives MCG clock */
            mcgffclk = CPU_XTAL0_CLK_HZ;
#else
            /* System oscillator 0 drives MCG clock */
            mcgffclk = CPU_XTAL_CLK_HZ;
#endif            
        } 
        else if (oscsel == 0x01u)
        { 
            /* RTC 32 kHz oscillator drives MCG clock */
            mcgffclk = CPU_XTAL32k_CLK_HZ;
        }
#if FSL_FEATURE_MCG_HAS_IRC_48M         /* case 1.1: if IRC 48M exists*/
        else if (oscsel == 0x02u)
        { 
            /* IRC 48Mhz oscillator drives MCG clock */
            mcgffclk = CPU_INT_IRC_CLK_HZ;
        }
        else
        {
            mcgffclk = 0;
        }
#endif

#else                                   /* case 2: use default osc0*/

        /* System oscillator 0 drives MCG clock */
        mcgffclk = CPU_XTAL_CLK_HZ;

#endif        
        
        divider = (uint8_t)(1u << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
        
        /* Calculate the divided FLL reference clock*/
        mcgffclk = (mcgffclk / divider);
        
        if ((MCG->C2 & MCG_C2_RANGE0_MASK) != 0x0u)
        {
            /* If high range is enabled, additional 32 divider is active*/
            mcgffclk = (mcgffclk >> kMcgConstant5);
        }
    } 
    else 
    { 
        /* The slow internal reference clock is selected */
        mcgffclk = CPU_INT_SLOW_CLK_HZ;
    } 
    return mcgffclk;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : clock_hal_get_fllclk
 * Description   : Get the current mcg fll clock
 * This function will return the mcgfllclk value in frequency(hz) based on 
 * current mcg configurations and settings. Fll should be properly configured
 * in order to get the valid value.
 * 
 *END**************************************************************************/
uint32_t clock_hal_get_fllclk(void)
{
    uint32_t mcgfllclk;

    mcgfllclk = get_mcgffclk();
    
    /* Select correct multiplier to calculate the MCG output clock  */
    switch (MCG->C4 & (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) 
    {
        case 0x0u:
            mcgfllclk *= kMcgConstant640;
            break;
        case kMcgConstantHex20:
            mcgfllclk *= kMcgConstant1280;
            break;
        case kMcgConstantHex40:
            mcgfllclk *= kMcgConstant1920;
            break;
        case kMcgConstantHex60:
            mcgfllclk *= kMcgConstant2560;
            break;
        case kMcgConstantHex80:
            mcgfllclk *= kMcgConstant732;
            break;
        case kMcgConstantHexA0:
            mcgfllclk *= kMcgConstant1464;
            break;
        case kMcgConstantHexC0:
            mcgfllclk *= kMcgConstant2197;
            break;
        case kMcgConstantHexE0:
            mcgfllclk *= kMcgConstant2929;
            break;
        default:
            break;
    }
    return mcgfllclk;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : clock_hal_get_pll0clk
 * Description   : Get the current mcg pll/pll0 clock
 * This function will return the mcgpllclk/mcgpll0 value in frequency(hz) based
 * on current mcg configurations and settings. PLL/PLL0 should be properly 
 * configured in order to get the valid value.
 * 
 *END**************************************************************************/
uint32_t clock_hal_get_pll0clk(void)
{
    uint32_t mcgpll0clk;
    uint8_t  divider;

    /* PLL(0) output is selected*/
#if FSL_FEATURE_MCG_USE_PLLREFSEL /* case 1 use pllrefsel to select pll*/

    if ((MCG->C5 & MCG_C5_PLLREFSEL0_MASK) != 0x0u)
    {
        /* OSC1 clock source used as an external reference clock */
        mcgpll0clk = CPU_XTAL1_CLK_HZ;
    }
    else
    { 
        /* OSC0 clock source used as an external reference clock*/
        mcgpll0clk = CPU_XTAL0_CLK_HZ;
    }
#else                   /* case 3 use default osc0*/
    /* OSC0 clock source used as an external reference clock*/
    mcgpll0clk = CPU_XTAL_CLK_HZ;
#endif
    
    divider = (kMcgConstant1 + (MCG->C5 & MCG_C5_PRDIV0_MASK));
    
    /* Calculate the PLL reference clock*/
    mcgpll0clk /= divider;
    divider = ((MCG->C6 & MCG_C6_VDIV0_MASK) + FSL_FEATURE_MCG_PLL_VDIV_BASE);
    
    /* Calculate the MCG output clock*/
    mcgpll0clk = (mcgpll0clk * divider); 

#if FSL_FEATURE_MCG_HAS_PLL_EXTRA_DIV      
    mcgpll0clk = (mcgpll0clk >> kMcgConstant1);  /* divided by 2*/
#endif
    
    return mcgpll0clk;
}

#if FSL_FEATURE_MCG_HAS_PLL1
/*FUNCTION**********************************************************************
 *
 * Function Name : clock_hal_get_pll1clk
 * Description   : Get the current mcg pll1 clock
 * This function will return the mcgpll1clk value in frequency(hz) based
 * on current mcg configurations and settings. PLL1 should be properly configured
 * in order to get the valid value.
 * 
 *END**************************************************************************/
uint32_t clock_hal_get_pll1clk(void)
{
    uint32_t mcgpll1clk;
    uint8_t  divider;
    
    if ((MCG->C11 & MCG_C11_PLLREFSEL1_MASK) != 0x0u)
    {
        /* OSC1 clock source used as an external reference clock*/
        mcgpll1clk = CPU_XTAL1_CLK_HZ;
    }
    else
    { 
        /* OSC0 clock source used as an external reference clock*/
        mcgpll1clk = CPU_XTAL0_CLK_HZ;
    } 
    
    divider = (kMcgConstant1 + (MCG->C11 & MCG_C11_PRDIV1_MASK));
    
    /* Calculate the PLL reference clock*/
    mcgpll1clk /= divider;
    divider = ((MCG->C12 & MCG_C12_VDIV1_MASK) + FSL_FEATURE_MCG_PLL_VDIV_BASE);
    
    /* Calculate the MCG output clock*/
    mcgpll1clk = ((mcgpll1clk * divider) >> kMcgConstant1); /* divided by 2*/
    return mcgpll1clk;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : clock_hal_get_irclk
 * Description   : Get the current mcg ir clock
 * This function will return the mcgirclk value in frequency(hz) based
 * on current mcg configurations and settings. It will not check if the 
 * mcgirclk is enabled or not, just calculate and return the value.
 * 
 *END**************************************************************************/
uint32_t clock_hal_get_irclk(void)
{
    int32_t mcgirclk;
    if ((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u)
    {
        /* Slow internal reference clock selected*/
        mcgirclk = CPU_INT_SLOW_CLK_HZ;
    }
    else
    {
        mcgirclk = CPU_INT_FAST_CLK_HZ/(1<<((MCG->SC & MCG_SC_FCRDIV_MASK)>>MCG_SC_FCRDIV_SHIFT));
    }
    return mcgirclk;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : clock_hal_get_outclk
 * Description   : Get the current mcg out clock
 * This function will return the mcgoutclk value in frequency(hz) based on 
 * current mcg configurations and settings. The configuration should be 
 * properly done in order to get the valid value.
 * 
 *END**************************************************************************/
uint32_t clock_hal_get_outclk(void)
{
    /* Variable to store output clock frequency of the MCG module*/
    uint32_t mcgoutclk = 0;

    if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x0u)
    {
        /* Output of FLL or PLL is selected*/
        if ((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u)
        {
            /* FLL is selected*/
            mcgoutclk = clock_hal_get_fllclk();
        }
        else
        { 
            /* PLL is selected*/
#if FSL_FEATURE_MCG_HAS_PLL1          
            if ((MCG->C11 & MCG_C11_PLLCS_MASK) != 0x0u)
            {
                /* PLL1 output is selected*/
                mcgoutclk = clock_hal_get_pll1clk();
            }
            else
            {
                mcgoutclk = clock_hal_get_pll0clk();
            }
#else
            mcgoutclk = clock_hal_get_pll0clk();
#endif            
        }
    }
    else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x40u)
    {
        /* Internal reference clock is selected*/
        mcgoutclk = clock_hal_get_irclk();
    } 
    else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u)
    {
        /* External reference clock is selected*/

#if FSL_FEATURE_MCG_USE_OSCSEL              /* case 1: use oscsel for outclock      */

        uint32_t oscsel = (MCG->C7 & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT;
        if (oscsel == 0x0u)
        {
#if FSL_FEATURE_MCG_HAS_OSC1          
            /* System oscillator drives MCG clock*/
            mcgoutclk = CPU_XTAL0_CLK_HZ;
#else
            /* System oscillator drives MCG clock*/
            mcgoutclk = CPU_XTAL_CLK_HZ;
#endif            
        }
        else if (oscsel == 0x01u)
        {
            /* RTC 32 kHz oscillator drives MCG clock*/
            mcgoutclk = CPU_XTAL32k_CLK_HZ;
        }
#if FSL_FEATURE_MCG_HAS_IRC_48M             /* case 1.1: IRC 48M exists*/
        else if (oscsel == 0x02u)
        {
            /* IRC 48Mhz oscillator drives MCG clock*/
            mcgoutclk = CPU_INT_IRC_CLK_HZ;
        }
        else
        {
            mcgoutclk = 0;
        }
#endif

#else                                       /* case 2: use default osc0*/
        /* System oscillator drives MCG clock*/
        mcgoutclk = CPU_XTAL_CLK_HZ;
#endif
    }
    else
    {
        /* Reserved value*/
        return mcgoutclk;
    } 
    return mcgoutclk;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

