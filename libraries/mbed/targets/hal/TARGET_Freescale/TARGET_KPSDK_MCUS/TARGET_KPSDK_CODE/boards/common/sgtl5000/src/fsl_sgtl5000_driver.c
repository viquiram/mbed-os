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
#include "fsl_sgtl5000_driver.h"

/*******************************************************************************
 *Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : sgtl_init
 * Description   : Initialize the sgtl5000 board.
 * This function has configured the sgtl5000 board. 
 *END**************************************************************************/
sgtl_status_t sgtl_init(sgtl_handler_t *handler, void *codec_config)
{
    sgtl_i2c_init(handler);
    /*
    * Power Supply Configuration
    * NOTE: This next 2 Write calls is needed ONLY if VDDD is internally driven by the chip.
    * Configure VDDD level to 1.2V (bits 3:0)
    */
    sgtl_write_reg(handler,CHIP_LINREG_CTRL, 0x0008);
    /* Power up internal linear regulator (Set bit 9) */
    sgtl_write_reg(handler,CHIP_ANA_POWER, 0x7260);
    /*
    * NOTE: This next Write call is needed ONLY if VDDD is externally driven
    * Turn off startup power supplies to save power (Clear bit 12 and 13)
    */
    sgtl_write_reg(handler,CHIP_ANA_POWER, 0x4260);
    /*	
    * NOTE: The next 2 Write calls is needed only if both VDDA and
    * VDDIO power supplies are less than 3.1V.
    * Enable the internal oscillator for the charge pump (Set bit 11)
    */
    sgtl_write_reg(handler,CHIP_CLK_TOP_CTRL, 0x0800);
    /* Enable charge pump (Set bit 11) */
    sgtl_write_reg(handler,CHIP_ANA_POWER, 0x4A60);
    /*	
    * NOTE: The next 2 modify calls is only needed if both VDDA and
    * VDDIO are greater than 3.1V
    * Configure the charge pump to use the VDDIO rail (set bit 5 and bit 6)
    */
    sgtl_write_reg(handler,CHIP_LINREG_CTRL, 0x006C);
    /*
    * Reference Voltage and Bias Current Configuration
    * NOTE: The value written in the next 2 Write calls is dependent on the VDDA voltage value.
    * Set ground, ADC, DAC reference voltage (bits 8:4). The value should
    * be set to VDDA/2. This example assumes VDDA = 1.8V. VDDA/2 = 0.9V.
    * The bias current should be set to 50% of the nominal value (bits 3:1)
    */
    sgtl_write_reg(handler,CHIP_REF_CTRL, 0x004E);
    /*	
    * Set LINEOUT reference voltage to VDDIO/2 (1.65V) (bits 5:0) and bias current
    * (bits 11:8) to the recommended value of 0.36mA for 10kOhm load with 1nF capacitance.
    */ 
    sgtl_write_reg(handler,CHIP_LINE_OUT_CTRL, 0x0322);
    /*
    *Other Analog Block Configurations
    * Configure slow ramp up rate to minimize pop (bit 0)
    */
    sgtl_write_reg(handler,CHIP_REF_CTRL, 0x004F);

    /*
    * Enable short detect mode for headphone left/right and center channel and set short detect 
    *current trip level to 75mA
    */
    sgtl_write_reg(handler,CHIP_SHORT_CTRL, 0x1106);
    /* Enable Zero-cross detect if needed for HP_OUT (bit 5) and ADC (bit 1) */
    sgtl_write_reg(handler,CHIP_ANA_CTRL, 0x0133);
    /*
    * Power up Inputs/Outputs/Digital Blocks
    * Power up LINEOUT, HP, ADC, DAC.
    */
    sgtl_write_reg(handler,CHIP_ANA_POWER, 0x6AFF);
    /*
    * Power up desired digital blocks.
    * I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4), DAC (bit 5), ADC (bit 6) are powered on
    */
    sgtl_write_reg(handler,CHIP_DIG_POWER, 0x0073);
    /*
    * Set LINEOUT Volume Level
    * Set the LINEOUT volume level based on voltage reference (VAG) values using this formula.
    * Value = (int)(40*log(VAG_VAL/LO_VAGCNTRL) + 15).
    * Assuming VAG_VAL and LO_VAGCNTRL is set to 0.9V and 1.65V respectively, the
    * left LO volume (bits 12:8) and right LO volume (bits 4:0) value should be set to 5.
    */
    sgtl_write_reg(handler,CHIP_LINE_OUT_VOL, 0x0505);    
    /* Configure SYS_FS clock to 48kHz, MCLK_FREQ to 256*Fs. */
    sgtl_modify_reg(handler,CHIP_CLK_CTRL, 0xFFC8, 0x0008); 
    /*
    * Configure the I2S clocks in slave mode.
    * I2S LRCLK is same as the system sample clock.
    * Data length = 16 bits.
    */
    sgtl_modify_reg(handler,CHIP_I2S_CTRL, 0xFFFF, 0x01B0);
    /* I2S_IN -> DAC -> HP_OUT, Route I2S_IN to DAC */
    sgtl_modify_reg(handler,CHIP_SSS_CTRL, 0xFFDF, 0x0010);   
    /* Select DAC as the input to HP_OUT */
    sgtl_modify_reg(handler,CHIP_ANA_CTRL, 0xFFBF, 0x0000);
    /* LINE_IN -> ADC -> I2S_OUT. Set ADC input to LINE_IN. */
    sgtl_modify_reg(handler,CHIP_ANA_CTRL, 0xFFFF, 0x0004); 
    /* Route ADC to I2S_OUT */
    sgtl_modify_reg(handler,CHIP_SSS_CTRL, 0xFFFC, 0x0000); 
    /*
    * Input Volume Control
    * Configure ADC left and right analog volume to desired default.
    * Example shows volume of 0dB.
    */
    sgtl_write_reg(handler,CHIP_ANA_ADC_CTRL, 0x0000);
    /* Configure MIC gain if needed. Example shows gain of 20dB. */
    sgtl_modify_reg(handler,CHIP_MIC_CTRL, 0xFFFD, 0x0001); 
    /*	
    * Volume and Mute Control
    * Configure HP_OUT left and right volume to minimum, unmute.
    * HP_OUT and ramp the volume up to desired volume.
    */
    sgtl_write_reg(handler,CHIP_ANA_HP_CTRL, 0x7F7F);
    /* Code assumes that left and right volumes are set to same value. */
    sgtl_write_reg(handler,CHIP_ANA_HP_CTRL, 0x0000);
    sgtl_modify_reg(handler,CHIP_ANA_CTRL, 0xFFEF, 0x0000); 
    /* LINEOUT and DAC volume control. */
    sgtl_modify_reg(handler,CHIP_ANA_CTRL, 0xFEFF, 0x0000); 
    /* Configure DAC left and right digital volume. Example shows volume of 0dB. */
    sgtl_write_reg(handler,CHIP_DAC_VOL, 0x7C7C);
    sgtl_modify_reg(handler,CHIP_ADCDAC_CTRL, 0xFFFB, 0x0000); 
    sgtl_modify_reg(handler,CHIP_ADCDAC_CTRL, 0xFFF7, 0x0000); 
    /* Unmute ADC. */
    sgtl_modify_reg(handler,CHIP_ANA_CTRL, 0xFFFE ,0x0000); 
    return kStatus_SGTL_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sgtl_i2c_init
 * Description   : Initialize the I2C transfer.
 * The sgtl5000 codec is controlled by I2C, using I2C transfer can access the sgtl register. 
 *END**************************************************************************/
sgtl_status_t sgtl_i2c_init(sgtl_handler_t *handler)
{
    i2c_device_t *device = &handler->device;
    i2c_master_t *master = &handler->master;
    /* The master structure initialize */
    master->instance = 0;	
    i2c_master_init(master->instance, master);
    /* Configure the device info of I2C */
    device->baudRate_kbps = 100;
    device->address = SGTL5000_I2C_ADDR;
    i2c_master_configure_bus(master, device);
    return kStatus_SGTL_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sgtl_deinit
 * Description   : Deinit the sgtl5000 board.
 * This function would free the i2c source applied. 
 *END**************************************************************************/
sgtl_status_t sgtl_deinit(sgtl_handler_t *handler)
{
    i2c_master_shutdown(&handler->master);
    return kStatus_SGTL_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sgtl_configure_data_format
 * Description   : Configure the audio data format of the sgtl5000.
 * This function would configure the sample rate and the data length.
 *END**************************************************************************/
sgtl_status_t sgtl_configure_data_format
(
sgtl_handler_t *handler, 
uint32_t mclk, 
uint32_t sample_rate, 
uint8_t bits
)
{
    uint16_t val ; 
    uint16_t bit_val;
    uint8_t retval;
    uint16_t mul_clk = mclk/sample_rate;
    retval = sgtl_read_reg(handler, CHIP_I2S_CTRL, &bit_val);
    retval = sgtl_read_reg(handler, CHIP_CLK_CTRL, &val);

    if(retval != kStatus_SGTL_Success)
    {
        return kStatus_SGTL_Fail;
    }
   /* Configure the mul_clk. Sgtl-5000 only support 256, 384 and 512 oversample rate */
   if((mul_clk/128 - 2) > 2)
   {
       return kStatus_SGTL_Fail;
   }
   else
   {
       val = mul_clk/128 -2;
   }
    switch(sample_rate)
    {
        case 8000:
            val |= 0x0020;
            break;     
        case 11025:
            val |= 0x0024;
            break;
        case 12000:
            val = 0x0028;
            break;    
        case 16000:
            val |= 0x0010;
            break;  
        case 22050:
            val |= 0x0014;
            break;  
        case 24000:
            val |= 0x0018;
            break;   
        case 32000:
            val |= 0x0000;
            break;           
        case 44100:
            val |= 0x0004;
            break;         
        case 48000:
            val |= 0x0008;
            break;            
        case 96000:
            val |= 0x000C;
            break;           
        default:
            return kStatus_SGTL_Fail;
    }
    sgtl_write_reg(handler,CHIP_CLK_CTRL , val);

    /* data bits configure,sgtl supports 16bit, 20bit 24bit, 32bit */
    sgtl_read_reg(handler, CHIP_I2S_CTRL, &bit_val);
    bit_val &= 0xFF4F;
    switch(bits)
    {
        case 16:
            bit_val |= (3 << 4);
            break;
        case 20:
            bit_val |= (2 << 4);
            break;
        case 24:
            bit_val |= (1 << 4);
            break;
        case 32:
            bit_val |= (0 << 4) ;
            break;
        default:
            return kStatus_SGTL_Fail;
    }
    sgtl_write_reg(handler, CHIP_I2S_CTRL,bit_val);

    return kStatus_SGTL_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sgtl_write_reg
 * Description   : Write the specified register of sgtl5000.
 * The writing process is through I2C.
 *END**************************************************************************/
sgtl_status_t sgtl_write_reg(sgtl_handler_t *handler, uint16_t reg, uint16_t val)
{
    i2c_device_t *device = &(handler->device);
    i2c_master_t *master = &(handler->master);   
    uint8_t buff[4];
    uint32_t transferred = 0;
    /* The register address */
    buff[0] = (reg & 0xFF00U) >> 8U;
    buff[1] = reg & 0xFF;  
    /* Data */
    buff[2] = (val & 0xFF00U) >> 8U;
    buff[3] = val & 0xFF;
    
    i2c_master_transfer(master, device , kI2CWrite, 0, 0, buff, 4, &transferred, kSyncWaitForever);
    return kStatus_SGTL_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sgtl_read_reg
 * Description   : Read the specified register value of sgtl5000.
 * The reading process is through I2C.
 *END**************************************************************************/
sgtl_status_t sgtl_read_reg(sgtl_handler_t *handler, uint16_t reg, uint16_t *val)
{
    i2c_device_t *device = &(handler->device);
    i2c_master_t *master = &(handler->master);  
    uint8_t buff[2];
    uint8_t data[2];
    uint32_t transferred;
    uint8_t retval = 0;
    buff[0] = (reg & 0xFF00U) >> 8U;
    buff[1] = reg & 0xFF;
    retval = i2c_master_transfer(master, device , kI2CWrite, 0, 0, buff, 2, &transferred, kSyncWaitForever);
    if(retval != kStatus_SGTL_Success)
    {
        return kStatus_SGTL_Fail;
    }
    retval = i2c_master_transfer(master, device , kI2CRead, 0, 0, data, 2, &transferred, kSyncWaitForever);
    *val = (uint16_t)(((uint32_t)data[0] << 8U) | (uint32_t)data[1]); 
    return kStatus_SGTL_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : sgtl_modify_reg
 * Description   : Modify the specified register value of sgtl5000.
 * The modify process is through I2C.
 *END**************************************************************************/
sgtl_status_t sgtl_modify_reg(sgtl_handler_t *handler, uint16_t reg, uint16_t mask, uint16_t val)
{
    uint8_t retval = 0;	
    uint16_t reg_val;	
    retval = sgtl_read_reg(handler, reg, &reg_val);
    if(retval != kStatus_SGTL_Success)
    {
        return kStatus_SGTL_Fail;
    }
    reg_val &= mask;
    reg_val |= val;	
    retval = sgtl_write_reg(handler, reg, reg_val);
    if(retval != kStatus_SGTL_Success)
    {
        return kStatus_SGTL_Fail;
    }	
    return kStatus_SGTL_Success;
}

/*******************************************************************************
 *EOF
 ******************************************************************************/

