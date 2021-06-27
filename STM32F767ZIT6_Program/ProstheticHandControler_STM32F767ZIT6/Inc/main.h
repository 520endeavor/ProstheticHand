/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define STM32_Bluetooth_Pairing_Info_Pin GPIO_PIN_6
#define STM32_Bluetooth_Pairing_Info_GPIO_Port GPIOE
#define STM32_Bluetooth_AT_CS_Pin GPIO_PIN_13
#define STM32_Bluetooth_AT_CS_GPIO_Port GPIOC
#define STM32_Bluetooth_Pairing_LED_Pin GPIO_PIN_8
#define STM32_Bluetooth_Pairing_LED_GPIO_Port GPIOF
#define STM32_SPI1_SCK_LTC1867_Pin GPIO_PIN_5
#define STM32_SPI1_SCK_LTC1867_GPIO_Port GPIOA
#define STM32_SPI1_MISO_LTC1867_Pin GPIO_PIN_6
#define STM32_SPI1_MISO_LTC1867_GPIO_Port GPIOA
#define STM32_SPI1_MOSI_LTC1867_Pin GPIO_PIN_7
#define STM32_SPI1_MOSI_LTC1867_GPIO_Port GPIOA
#define STM32_SPI1_PSC0_LTC1867_Pin GPIO_PIN_4
#define STM32_SPI1_PSC0_LTC1867_GPIO_Port GPIOC
#define STM32_USART3_TX_AS69T20_Pin GPIO_PIN_10
#define STM32_USART3_TX_AS69T20_GPIO_Port GPIOB
#define STM32_USART3_RX_AS69T20_Pin GPIO_PIN_11
#define STM32_USART3_RX_AS69T20_GPIO_Port GPIOB
#define STM32_USART5_RX_DSP_Pin GPIO_PIN_12
#define STM32_USART5_RX_DSP_GPIO_Port GPIOB
#define STM32_USART5_TX_DSP_Pin GPIO_PIN_13
#define STM32_USART5_TX_DSP_GPIO_Port GPIOB
#define STM32_CY7C024_INTL_Pin GPIO_PIN_2
#define STM32_CY7C024_INTL_GPIO_Port GPIOG
#define STM32_USART1_TX_Bluetooth_Pin GPIO_PIN_9
#define STM32_USART1_TX_Bluetooth_GPIO_Port GPIOA
#define STM32_USART1_RX_Bluetooth_Pin GPIO_PIN_10
#define STM32_USART1_RX_Bluetooth_GPIO_Port GPIOA
#define STM32_CAN1_RX_Pin GPIO_PIN_11
#define STM32_CAN1_RX_GPIO_Port GPIOA
#define STM32_CAN1_TX_Pin GPIO_PIN_12
#define STM32_CAN1_TX_GPIO_Port GPIOA
#define STM32_TIM3_CH1_G_Pin GPIO_PIN_4
#define STM32_TIM3_CH1_G_GPIO_Port GPIOB
#define STM32_TIM3_CH2_R_Pin GPIO_PIN_5
#define STM32_TIM3_CH2_R_GPIO_Port GPIOB
#define STM32_TIM4_CH1_B_Pin GPIO_PIN_6
#define STM32_TIM4_CH1_B_GPIO_Port GPIOB
#define STM32_I2C1_SCL_FRAM16_Pin GPIO_PIN_8
#define STM32_I2C1_SCL_FRAM16_GPIO_Port GPIOB
#define STM32_I2C1_SDA_FRAM16_Pin GPIO_PIN_9
#define STM32_I2C1_SDA_FRAM16_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
