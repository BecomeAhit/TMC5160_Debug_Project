/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"
#include "tmc5160.h"

/* USER CODE BEGIN 0 */

/* ---------------- UART5: PC ͨ---------------- */

static uint8_t  uart5_rx_byte = 0;											//byte buffer

uint8_t Receive_data[7]; 			// received data from USB-UART5

extern uint8_t buffer;
extern uint8_t UART_BUF;
extern uint8_t UART_INT;
extern uint8_t UART_FLAG;


/* ---------------- USART1 7-byte frame ---------------- */

//volatile uint8_t buffer = 0;

extern uint8_t Read_Data[8];

static uint8_t usart1_rx_byte = 0;


#define TMC_FRAME_LEN 8

static uint8_t usart1_rx_byte;
static volatile uint8_t usart1_rx_idx = 0;
static volatile uint8_t usart1_frame_ready = 0;
static uint8_t usart1_frame[TMC_FRAME_LEN];


/* ---------------- HAL_UART_RxCpltCallback ---------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    
	/* --------------- UART5 PC Communication ---------------- */
	if (huart->Instance == UART5)
    {
        /* 相当于 USART_ReceiveData(USARTx) 读到的字节 */
        uint8_t byte = uart5_rx_byte;

        if (UART_BUF == 6)   /* Received 7 byte (index 0..6) */
        {
            UART_BUF = 0;
        }
        else
        {
            Receive_data[UART_BUF] = byte;

            if (Receive_data[0] == 0xAE)  /* START_BYTE is AE */
            {
                UART_BUF++;
            }
            else
            {
                UART_BUF = 0;
            }
        }

        if (Receive_data[1] == 0xFF)
        {
            UART_INT = 0xAA;
        }

        /* === 关键：重新开启下一字节接收中断 === */
        (void)HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
    }
		
	/* ---------------- USART1 ---------------- */
	if (huart->Instance == USART1)
    {
        usart1_frame[usart1_rx_idx++] = usart1_rx_byte;

        if (usart1_rx_idx >= TMC_FRAME_LEN)
        {
            usart1_rx_idx = 0;
            usart1_frame_ready = 1;
        }

        (void)HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1);
        return;
    }
}


/* USER CODE END 0 */

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
	
	// enable UART5 interrupt
	HAL_UART_Receive_IT(&huart5, &uart5_rx_byte, 1);
	
  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

	// enable USART1 interrupt
	HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1);
	
  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = __UART5_USB_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
    HAL_GPIO_Init(__UART5_USB_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = _UART5_USB_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
    HAL_GPIO_Init(_UART5_USB_RX_GPIO_Port, &GPIO_InitStruct);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PE0     ------> USART1_TX
    PE1     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = _URART1_5160_TX_Pin|_USART1_5160_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = _USART2_TRANS_TX_Pin|_UASRT2_TRANS_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PE15     ------> USART3_RX
    PB10     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = USART3_5130_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(USART3_5130_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USART3_5130_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(USART3_5130_TX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(__UART5_USB_TX_GPIO_Port, __UART5_USB_TX_Pin);

    HAL_GPIO_DeInit(_UART5_USB_RX_GPIO_Port, _UART5_USB_RX_Pin);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PE0     ------> USART1_TX
    PE1     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOE, _URART1_5160_TX_Pin|_USART1_5160_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, _USART2_TRANS_TX_Pin|_UASRT2_TRANS_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PE15     ------> USART3_RX
    PB10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(USART3_5130_RX_GPIO_Port, USART3_5130_RX_Pin);

    HAL_GPIO_DeInit(USART3_5130_TX_GPIO_Port, USART3_5130_TX_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void UART5_SendByte(uint8_t b)
{
    HAL_UART_Transmit(&huart5, &b, 1, 100);
}



void UART5_Send_Array(uint8_t *send_array, uint8_t num)
{

    for ( uint8_t i = 0; i < num; i++)
    {
        /* 等待发送缓冲区空 */
        while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TXE) == RESET);

        /* 写数据寄存器 */
        huart5.Instance->TDR = send_array[i];

        /* 等待发送完成 */
        while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) == RESET);
    }
}


void USART1_Send_Array(uint8_t *send_array, uint8_t num)
{
    TMC5160_TxEnable();                      

		for (uint8_t i = 0; i < num; i++)
    {
        // TXE
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
        
        //
        huart1.Instance->TDR = send_array[i];
    }

				// 
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
		
		TMC5160_RxEnable();                    

}

uint8_t UART_CAL_CRC(uint8_t* datagram, uint8_t datagramLength)
{
	int i,j;
	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	uint8_t currentByte;
	
	*crc = 0;
	for (i=0; i<(datagramLength-1); i++) 					// Execute for all bytes of a message
	{ 
		currentByte = datagram[i]; 									// Retrieve a byte to be sent from Array
			for (j=0; j<8; j++)
			{
					if ((*crc >> 7) ^ (currentByte&0x01)) 		// update CRC based result of XOR operation
					{
						*crc = (*crc << 1) ^ 0x07;
					}
					else
					{
						*crc = (*crc << 1);
					}
					currentByte = currentByte >> 1;
				} // for CRC bit
		 } // for message byte
	return(*crc);
}

void USART1_Send_CMD_AutoCRC(uint8_t Motor_Addr,
                             uint8_t reg_addr,
                             uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    // 8 字节示例：1(SYNC)+1(ADDR)+1(REG)+4(DATA)+1(CRC) = 8
    uint8_t datagram[8];

    datagram[0] = 0x05;                // Sync byte 
    datagram[1] = Motor_Addr;          // Slave address
    datagram[2] = (uint8_t)(reg_addr | 0x80); // 写寄存器通常需要置位写标志

    datagram[3] = data1;
    datagram[4] = data2;
    datagram[5] = data3;
    datagram[6] = data4;

    datagram[7] = 0x00;                // 预留 CRC 位
    datagram[7] = UART_CAL_CRC(datagram, (uint8_t)sizeof(datagram));

    // ===== 发送部分 =====
    // 例如 HAL 阻塞发送：
    TMC5160_TxEnable();  
		HAL_UART_Transmit(&huart1, datagram, sizeof(datagram), 100);
	  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {;}
    TMC5160_RxEnable();

}


/* USER CODE END 1 */
