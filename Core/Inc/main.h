/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f3xx_hal.h"

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
#define HEART_BEAT_Pin GPIO_PIN_2
#define HEART_BEAT_GPIO_Port GPIOE
#define POS_REFL_Pin GPIO_PIN_3
#define POS_REFL_GPIO_Port GPIOE
#define POS_REFR_Pin GPIO_PIN_4
#define POS_REFR_GPIO_Port GPIOE
#define _1G_Laser_EN_Pin GPIO_PIN_7
#define _1G_Laser_EN_GPIO_Port GPIOE
#define _2G_Laser_EN_Pin GPIO_PIN_8
#define _2G_Laser_EN_GPIO_Port GPIOE
#define _1G_PWM_Pin GPIO_PIN_9
#define _1G_PWM_GPIO_Port GPIOE
#define _1R_Laser_EN_Pin GPIO_PIN_10
#define _1R_Laser_EN_GPIO_Port GPIOE
#define _2G_PWM_Pin GPIO_PIN_11
#define _2G_PWM_GPIO_Port GPIOE
#define _2R_Laser_EN_Pin GPIO_PIN_12
#define _2R_Laser_EN_GPIO_Port GPIOE
#define _1R_PWM_Pin GPIO_PIN_13
#define _1R_PWM_GPIO_Port GPIOE
#define _2R_PWM_Pin GPIO_PIN_14
#define _2R_PWM_GPIO_Port GPIOE
#define USART3_5130_RX_Pin GPIO_PIN_15
#define USART3_5130_RX_GPIO_Port GPIOE
#define USART3_5130_TX_Pin GPIO_PIN_10
#define USART3_5130_TX_GPIO_Port GPIOB
#define _5130_CMD_EN_Pin GPIO_PIN_11
#define _5130_CMD_EN_GPIO_Port GPIOB
#define _SPI2_BLE_CSN_Pin GPIO_PIN_12
#define _SPI2_BLE_CSN_GPIO_Port GPIOB
#define _SPI2_BLE_SCK_Pin GPIO_PIN_13
#define _SPI2_BLE_SCK_GPIO_Port GPIOB
#define _SPI2_BLE_MISO_Pin GPIO_PIN_14
#define _SPI2_BLE_MISO_GPIO_Port GPIOB
#define _SPI2_BLE_MOSI_Pin GPIO_PIN_15
#define _SPI2_BLE_MOSI_GPIO_Port GPIOB
#define _SPI2_BLE_CE_Pin GPIO_PIN_8
#define _SPI2_BLE_CE_GPIO_Port GPIOD
#define _SPI2_BLE_IRQ_Pin GPIO_PIN_9
#define _SPI2_BLE_IRQ_GPIO_Port GPIOD
#define _BRAKE_EN_Pin GPIO_PIN_10
#define _BRAKE_EN_GPIO_Port GPIOD
#define ROM_I2C2_SCL_Pin GPIO_PIN_9
#define ROM_I2C2_SCL_GPIO_Port GPIOA
#define ROM_I2C2_SDA_Pin GPIO_PIN_10
#define ROM_I2C2_SDA_GPIO_Port GPIOA
#define _WATCH_DOG_Pin GPIO_PIN_15
#define _WATCH_DOG_GPIO_Port GPIOA
#define __UART5_USB_TX_Pin GPIO_PIN_12
#define __UART5_USB_TX_GPIO_Port GPIOC
#define _UART5_USB_RX_Pin GPIO_PIN_2
#define _UART5_USB_RX_GPIO_Port GPIOD
#define _USART2_TRAN_EN_Pin GPIO_PIN_4
#define _USART2_TRAN_EN_GPIO_Port GPIOD
#define _USART2_TRANS_TX_Pin GPIO_PIN_5
#define _USART2_TRANS_TX_GPIO_Port GPIOD
#define _UASRT2_TRANS_RX_Pin GPIO_PIN_6
#define _UASRT2_TRANS_RX_GPIO_Port GPIOD
#define _USART1_5160_EN_Pin GPIO_PIN_9
#define _USART1_5160_EN_GPIO_Port GPIOB
#define _URART1_5160_TX_Pin GPIO_PIN_0
#define _URART1_5160_TX_GPIO_Port GPIOE
#define _USART1_5160_RX_Pin GPIO_PIN_1
#define _USART1_5160_RX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
