/**
  ******************************************************************************
  * @file    stm32f3_discovery_magnitometer.h
  * @author  DIfeID (MCD Application Team)
  * @version V0.0.1
  * @date    9-April-2016
  * @brief   This file contains definitions for stm32f3_discovery_magnitometer.c
  *          firmware driver.
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
  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3_DISCOVERY_MAGNITO_H
#define __STM32F3_DISCOVERY_MAGNITO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery.h"
/* Include component driver */
#include "lsm303dlhc.h"
   
/** @addtogroup BSP
  * @{
  */
  
/** @addtogroup STM32F3_DISCOVERY
  * @{
  */ 

/** @addtogroup STM32F3_DISCOVERY_MAGNITOMETER
  * @{
  */
  
/** @defgroup STM32F3_DISCOVERY_MAGNITO_Exported_Types STM32F3_DISCOVERY_MAGNITO_Exported_Type
  * @{
  */

/**
  * @}
  */
  
/** @defgroup STM32F3_DISCOVERY_MAGNITO_Exported_Constants STM32F3_DISCOVERY_MAGNITO_Exported_Constants
  * @{
  */
typedef enum 
{
  MAGNITO_OK = 0,
  MAGNITO_ERROR = 1
} 
MAGNITO_StatusTypeDef;

/**
  * @}
  */
  
/** @defgroup STM32F3_DISCOVERY_MAGNITO_Exported_Functions STM32F3_DISCOVERY_MAGNITO_Exported_Functions
  * @{
  */
/* Acc functions */  
uint8_t   BSP_MAGNITO_Init(void);
void      BSP_MAGNITO_GetXYZ(float *pDataXYZ);

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
#ifdef __cplusplus
}
#endif

#endif /* __STM32F3_DISCOVERY_MAGNITO_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
