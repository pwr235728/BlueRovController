/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_RS_Pin LL_GPIO_PIN_0
#define LCD_RS_GPIO_Port GPIOF
#define LCD_E_Pin LL_GPIO_PIN_1
#define LCD_E_GPIO_Port GPIOF
#define P1_Pin LL_GPIO_PIN_0
#define P1_GPIO_Port GPIOA
#define P2_Pin LL_GPIO_PIN_1
#define P2_GPIO_Port GPIOA
#define P3_Pin LL_GPIO_PIN_3
#define P3_GPIO_Port GPIOA
#define P4_Pin LL_GPIO_PIN_4
#define P4_GPIO_Port GPIOA
#define P5_Pin LL_GPIO_PIN_5
#define P5_GPIO_Port GPIOA
#define JOY_X_Pin LL_GPIO_PIN_7
#define JOY_X_GPIO_Port GPIOA
#define JOY_Y_Pin LL_GPIO_PIN_0
#define JOY_Y_GPIO_Port GPIOB
#define P6_Pin LL_GPIO_PIN_1
#define P6_GPIO_Port GPIOB
#define LCD_D4_Pin LL_GPIO_PIN_8
#define LCD_D4_GPIO_Port GPIOA
#define ZW_Pin LL_GPIO_PIN_9
#define ZW_GPIO_Port GPIOA
#define RS_DIR_Pin LL_GPIO_PIN_10
#define RS_DIR_GPIO_Port GPIOA
#define LCD_D5_Pin LL_GPIO_PIN_11
#define LCD_D5_GPIO_Port GPIOA
#define JOY_EN_Pin LL_GPIO_PIN_12
#define JOY_EN_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LD3_Pin LL_GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define LCD_D7_Pin LL_GPIO_PIN_4
#define LCD_D7_GPIO_Port GPIOB
#define LCD_D6_Pin LL_GPIO_PIN_5
#define LCD_D6_GPIO_Port GPIOB
#define RS_TX_Pin LL_GPIO_PIN_6
#define RS_TX_GPIO_Port GPIOB
#define RS_RX_Pin LL_GPIO_PIN_7
#define RS_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
