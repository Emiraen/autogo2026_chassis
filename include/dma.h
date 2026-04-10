/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
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
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* 保留给用户自定义 DMA 相关宏，当前实现使用 serial_protocol.h 中的帧尺寸定义 */
/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */
void DMA_UART1_Start(void);        /* 初始化 USART1 DMA RX + IDLE 检测 */
void DMA_UART1_RxIdleHandler(void);/* 在 USART1 空闲中断中调用，触发数据搬运 */
void DMA_UART1_TxInit(void);       
HAL_StatusTypeDef DMA_UART1_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

void DMA_UART2_Start(void);        /* 初始化 USART2 DMA RX + IDLE 检测 */
void DMA_UART2_RxIdleHandler(void);/* 在 USART2 空闲中断中调用，触发数据搬运 */
void DMA_UART2_TxInit(void);       /* 初始化 USART2 TX FIFO + DMA */

HAL_StatusTypeDef DMA_UART2_TxEnqueue(const uint8_t *data, uint16_t len);

HAL_StatusTypeDef DMA_UART2_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

