/**
  ******************************************************************************
  * @file    stm32f3_discovery_magnitometer.c
  * @author  DIfeID (MCD Application Team)
  * @version V0.0.1
  * @date    9-April-2016
  * @brief   This file provides a set of functions needed to manage the MAGNITOMETER
  *          MEMS available on STM32F3-Discovery Kit.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery_magnitometer.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */ 

/** @addtogroup STM32F3_DISCOVERY_MAGNITOMETER
  * @{
  */


/** @defgroup STM32F3_DISCOVERY_MAGNITO_Private_TypesDefinitions STM32F3_DISCOVERY_MAGNITO_Private_TypesDefinitions
  * @{
  */
  
/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_MAGNITO_Private_Defines STM32F3_DISCOVERY_MAGNITO_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_MAGNITO_Private_Macros STM32F3_DISCOVERY_MAGNITO_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup STM32F3_DISCOVERY_MAGNITO_Private_Variables STM32F3_DISCOVERY_MAGNITO_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_MAGNITO_Private_FunctionPrototypes STM32F3_DISCOVERY_MAGNITO_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_MAGNITO_Private_Functions STM32F3_DISCOVERY_MAGNITO_Private_Functions
  * @{
  */

/**
  * @brief  Set MAGNITOMETER Initialization.
  * @retval MAGNITO_OK if no problem during initialization
  */
uint8_t BSP_MAGNITO_Init(void)
{  
  uint8_t ret = MAGNITO_ERROR;
  LACCELERO_InitTypeDef LSM303DLHC_InitStructure;
 
  if(Lsm303dlhcDrv.ReadID() == I_AM_LMS303DLHC)
  {
  /* MEMS configuration ------------------------------------------------------*/
    /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
    LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
    LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_220_HZ;
    LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_1_9_GA;
    LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
    LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

    ret = MAGNITO_OK;
  }  
  else
  {
    ret = MAGNITO_ERROR;
  }
  return ret;
}

/**
  * @brief  Get XYZ
  * @param pDataXYZ Pointeur on 3 angular
  *                 pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
* @retval None
*/
void BSP_MAGNITO_GetXYZ(float* pDataXYZ)
{
  uint8_t buffer[6] = {0};
  int16_t pnRawData[3] = {0};
  uint8_t CTRLB = 0;
  uint8_t i = 0;
  float Magn_Sensitivity_XY = 1.0f, Magn_Sensitivity_Z = 1.0f;

  CTRLB = LSM303DLHC_MagGetDataStatus();
  buffer[0] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M);
  buffer[1] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M);
  buffer[2] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M);
  buffer[3] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M);
  buffer[4] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M);
  buffer[5] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M);

  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
    case LSM303DLHC_FS_1_3_GA:
      Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
      Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
      break;
    case LSM303DLHC_FS_1_9_GA:
      Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
      Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
      break;
    case LSM303DLHC_FS_2_5_GA:
      Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
      Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
      break;
    case LSM303DLHC_FS_4_0_GA:
      Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
      Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
      break;
    case LSM303DLHC_FS_4_7_GA:
      Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
      Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
      break;
    case LSM303DLHC_FS_5_6_GA:
      Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
      Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
      break;
    case LSM303DLHC_FS_8_1_GA:
      Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
      Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
      break;
  }

  for(i=0; i<3; i++)
  {
     pnRawData[i]=(int16_t)(((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
  }

  pDataXYZ[0] = (float)(pnRawData[0] / Magn_Sensitivity_XY);
  pDataXYZ[1] = (float)(pnRawData[1] / Magn_Sensitivity_XY);
  pDataXYZ[2] = (float)(pnRawData[2] / Magn_Sensitivity_Z);
}


/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */ 
  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
