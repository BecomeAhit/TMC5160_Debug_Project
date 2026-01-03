/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, HEART_BEAT_Pin|_1G_Laser_EN_Pin|_2G_Laser_EN_Pin|_1R_Laser_EN_Pin
                          |_2R_Laser_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, _5130_CMD_EN_Pin|_SPI2_BLE_CSN_Pin|_USART1_5160_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(_SPI2_BLE_CE_GPIO_Port, _SPI2_BLE_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, _BRAKE_EN_Pin|_USART2_TRAN_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : HEART_BEAT_Pin _1G_Laser_EN_Pin _2G_Laser_EN_Pin _1R_Laser_EN_Pin
                           _2R_Laser_EN_Pin */
  GPIO_InitStruct.Pin = HEART_BEAT_Pin|_1G_Laser_EN_Pin|_2G_Laser_EN_Pin|_1R_Laser_EN_Pin
                          |_2R_Laser_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : POS_REFL_Pin POS_REFR_Pin */
  GPIO_InitStruct.Pin = POS_REFL_Pin|POS_REFR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : _5130_CMD_EN_Pin _SPI2_BLE_CSN_Pin _USART1_5160_EN_Pin */
  GPIO_InitStruct.Pin = _5130_CMD_EN_Pin|_SPI2_BLE_CSN_Pin|_USART1_5160_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : _SPI2_BLE_CE_Pin */
  GPIO_InitStruct.Pin = _SPI2_BLE_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(_SPI2_BLE_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : _SPI2_BLE_IRQ_Pin */
  GPIO_InitStruct.Pin = _SPI2_BLE_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(_SPI2_BLE_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : _BRAKE_EN_Pin _USART2_TRAN_EN_Pin */
  GPIO_InitStruct.Pin = _BRAKE_EN_Pin|_USART2_TRAN_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
