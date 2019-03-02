/** \file max30102.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30102.cpp
* Description: This module is an embedded controller driver for the MAX30102
*
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include "MAX30102.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

extern UART_HandleTypeDef huart1;

bool maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{
  char ach_i2c_data[2];
  ach_i2c_data[0]=uch_addr;
  ach_i2c_data[1]=uch_data;
  
  if(HAL_I2C_Mem_Write(&hi2c3, I2C_WRITE_ADDR, ach_i2c_data[0], I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ach_i2c_data[1], 1, 10) == HAL_OK)
  {
	//   HAL_UART_Transmit(&huart1,(uint8_t *)"Mem_Write success!", 19, 10);
	  return true;
  }
  else
	  return false;
}

bool maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{
  // char ch_i2c_data[2];
  // ch_i2c_data[0]=uch_addr;

  if(HAL_I2C_Mem_Read(&hi2c3, I2C_READ_ADDR, uch_addr, I2C_MEMADD_SIZE_8BIT, puch_data, 1, 10)==0)
  {
    //HAL_UART_Transmit(&huart1,puch_data, 1, 10);
    return true;
  }
  else
    return false;
}

bool maxim_max30102_init()
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)) // INTR setting
    return false;
  if(!maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x02)) // Enable DIE_TEMP_RDY: Internal Temperature Ready Flag. The interrupt is cleared by reading either the Interrupt Status 2 register (0x01) or the TFRAC register (0x20).
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
    return false;
  if(!maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return false;
  if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return false;
  if(!maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return false;
  
  if(!maxim_max30102_write_reg(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
    return false;
  if(!maxim_max30102_write_reg(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
    return false;
  if(!maxim_max30102_write_reg(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
    return false;
  if(!maxim_max30102_write_reg(REG_TEMP_CONFIG,0x01))   // Enable TEMP
    return false;

  return true;  
}

bool maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
  uint32_t un_temp;
  unsigned char uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  char ach_i2c_data[6];
  
  //read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
  ach_i2c_data[0]=REG_FIFO_DATA;
  if(HAL_I2C_Mem_Read(&hi2c3, I2C_READ_ADDR, REG_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, ach_i2c_data, 6, 10)!= HAL_OK)
  {
    return false;
  }
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  
  
  return true;
}

bool maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(!maxim_max30102_write_reg(REG_MODE_CONFIG,0x40))
        return false;
    else
        return true;    
}

float maxim_max30102_read_temperature()
{
    uint8_t temp_inter, temp_fra;
    float temp_value;
    maxim_max30102_read_reg(REG_TEMP_INTR,&temp_inter);
    maxim_max30102_read_reg(REG_TEMP_FRAC,&temp_fra);
    
    temp_value = temp_inter + temp_fra*0.0625;
    return  temp_value;
}
