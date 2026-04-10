/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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
#include "dma.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>
#include <string.h>
#include "usart.h"
#include "serial_protocol.h"
#include "stm32f1xx_hal.h"
#include <stm32f1xx_hal_rcc.h>
#define UART1_RX_DMA_BUF_SIZE   (256U)                   /**< USART1 IMU 串口 DMA 环形缓冲大小 */
#define UART2_RX_DMA_BUF_SIZE   (MAX_RX_DMA_FRAME_SIZE)  /**< USART2 协议解析路径最大帧长 */
#define UART2_TX_DMA_BUF_SIZE   (PROTOCOL_MAX_FRAME_BYTES) /**< USART2 发送 FIFO 最大帧长 */
#define UART2_TX_TIMEOUT_MS     (5U)                     /**< USART2 发送 FIFO 等待超时时间，单位 ms */
#define UART1_TX_DMA_BUF_SIZE   (128U)                   /**< USART1 发送 FIFO 最大帧长 */
/* USART1 发送环形缓冲相关的全局变量 */
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
static uint8_t uart1_dma_tx_buf[UART1_TX_DMA_BUF_SIZE]; 
static volatile uint16_t uart1_tx_head = 0U;  /* 写指针 */
static volatile uint16_t uart1_tx_tail = 0U;  /* 读指针 */
static volatile uint16_t uart1_tx_dma_len = 0U; /* 当前 DMA 发送字节数 */
static volatile bool uart1_tx_busy = false;   /* 是否存在正在进行的 DMA 传输 */
static uint8_t uart1_dma_rx_buf[UART1_RX_DMA_BUF_SIZE]; /* USART1 (IMU) DMA 环形缓冲 */
static uint16_t uart1_last_pos = 0U;

static uint8_t uart2_dma_rx_buf[UART2_RX_DMA_BUF_SIZE]; /* USART2 (PC) DMA 环形缓冲 */
static uint16_t uart2_last_pos = 0U;

static uint8_t uart2_dma_tx_buf[UART2_TX_DMA_BUF_SIZE]; /* USART2 (PC) DMA 发送环形缓冲 */
static volatile uint16_t uart2_tx_head = 0U;  /* 写指针 */
static volatile uint16_t uart2_tx_tail = 0U;  /* 读指针 */
static volatile uint16_t uart2_tx_dma_len = 0U; /* 当前 DMA 发送字节数 */
static volatile bool uart2_tx_busy = false;   /* 是否存在正在进行的 DMA 传输 */

__weak void imu_uart_feed_stream(const uint8_t *buf, uint16_t len);
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */
/**
 * @brief 计算 DMA 写指针（累计写入的字节数）
 */
/**
 * @brief 计算 DMA 写指针（累计写入的字节数）。
 * @param[in] hdma      DMA 句柄
 * @param[in] buf_size  关联环形缓冲区字节数
 * @return 当前写指针位置（0~buf_size-1 的等效位置）
 * @note  通过剩余计数器反推出已写入的数据量，适用于循环模式。
 */
static inline uint16_t dma_uart_get_write_pos(DMA_HandleTypeDef *hdma, uint16_t buf_size)
{
  return (uint16_t)(buf_size - __HAL_DMA_GET_COUNTER(hdma));
}

/**
 * @brief 处理 USART1 DMA 缓冲中的一段新数据。
 * @param[in] start 起始索引
 * @param[in] count 字节数
 * @note  实际数据消费委托给 imu_uart_feed_stream（弱符号，可在 JY901B.c 中实现）。
 */
static void uart1_process_window(uint16_t start, uint16_t count)
{
  if (count == 0U) { return; }
  imu_uart_feed_stream(&uart1_dma_rx_buf[start], count);
}

/**
 * @brief 处理 USART2 DMA 缓冲中的一段新数据。
 * @param[in] start 起始索引
 * @param[in] count 字节数
 * @note  直接将字节流交给协议状态机 protocol_feed_stream 进行拼帧与调度。
 */
static void uart2_process_window(uint16_t start, uint16_t count)
{
  if (count == 0U) { return; }
  protocol_feed_stream(&uart2_dma_rx_buf[start], count);
}

/**
 * @brief 在关中断上下文下计算 USART2 发送 FIFO 剩余空间。
 * @return 当前可写入的字节数
 * @note  调用方需自行保证在关中断状态下调用，避免多核/中断竞争。
 */
static uint16_t uart2_tx_free_locked(void)
{
  const uint16_t head = uart2_tx_head;
  const uint16_t tail = uart2_tx_tail;
  if (head >= tail)
  {
    return (uint16_t)(UART2_TX_DMA_BUF_SIZE - (head - tail) - 1U);
  }
  return (uint16_t)(tail - head - 1U);
}

/**
 * @brief 在 FIFO 中存在待发送数据且 DMA 空闲时，尝试启动一次 DMA 发送。
 * @note  关中断保护 head/tail、busy 等共享状态，防止与中断回调竞争。
 */
static void uart2_tx_try_start(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (uart2_tx_busy || uart2_tx_head == uart2_tx_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  const uint16_t tail = uart2_tx_tail;
  const uint16_t head = uart2_tx_head;
  uint16_t size = (head > tail) ? (uint16_t)(head - tail)
                                : (uint16_t)(UART2_TX_DMA_BUF_SIZE - tail);
  uart2_tx_busy = true;
  uart2_tx_dma_len = size;
  uint8_t *start = &uart2_dma_tx_buf[tail];
  if (primask == 0U)
  {
    __enable_irq();
  }
  (void)HAL_UART_Transmit_DMA(&huart2, start, size);
}

/**
 * @brief USART1 空闲中断内部实现：推送新到字节窗口给 IMU 解析函数。
 * @note  由 DMA_UART1_RxIdleHandler 间接调用，运行在中断上下文。
 */
static void uart1_idle_isr(void)
{
  const uint16_t pos = dma_uart_get_write_pos(huart1.hdmarx, UART1_RX_DMA_BUF_SIZE);
  if (pos == uart1_last_pos) { return; }

  if (pos > uart1_last_pos)
  {
    uart1_process_window(uart1_last_pos, (uint16_t)(pos - uart1_last_pos));
  }
  else
  {
    uart1_process_window(uart1_last_pos, (uint16_t)(UART1_RX_DMA_BUF_SIZE - uart1_last_pos));
    uart1_process_window(0U, pos);
  }

  uart1_last_pos = pos;
}

/**
 * @brief USART2 空闲中断内部实现：推送新到字节窗口给协议状态机。
 * @note  由 DMA_UART2_RxIdleHandler 间接调用，运行在中断上下文。
 */
static void uart2_idle_isr(void)
{
  const uint16_t pos = dma_uart_get_write_pos(huart2.hdmarx, UART2_RX_DMA_BUF_SIZE);
  if (pos == uart2_last_pos) { return; }

  if (pos > uart2_last_pos)
  {
    uart2_process_window(uart2_last_pos, (uint16_t)(pos - uart2_last_pos));
  }
  else
  {
    uart2_process_window(uart2_last_pos, (uint16_t)(UART2_RX_DMA_BUF_SIZE - uart2_last_pos));
    uart2_process_window(0U, pos);
  }

  uart2_last_pos = pos;
}
/**
 * @brief 在关中断上下文下计算 USART1 发送 FIFO 剩余空间。
 */
static uint16_t uart1_tx_free_locked(void)
{
  const uint16_t head = uart1_tx_head;
  const uint16_t tail = uart1_tx_tail;
  if (head >= tail)
  {
    return (uint16_t)(UART1_TX_DMA_BUF_SIZE - (head - tail) - 1U);
  }
  return (uint16_t)(tail - head - 1U);
}

/**
 * @brief 尝试启动一次 USART1 DMA 发送
 */
static void uart1_tx_try_start(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (uart1_tx_busy || uart1_tx_head == uart1_tx_tail)
  {
    if (primask == 0U) __enable_irq();
    return;
  }

  const uint16_t tail = uart1_tx_tail;
  const uint16_t head = uart1_tx_head;
  uint16_t size = (head > tail) ? (uint16_t)(head - tail)
                                : (uint16_t)(UART1_TX_DMA_BUF_SIZE - tail);
  uart1_tx_busy = true;
  uart1_tx_dma_len = size;
  uint8_t *start = &uart1_dma_tx_buf[tail];
  if (primask == 0U) __enable_irq();
  
  (void)HAL_UART_Transmit_DMA(&huart1, start, size);
}

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/* USER CODE BEGIN 2 */
/**
 * @brief 初始化 USART1 (IMU) DMA RX + 空闲中断
 * @note  需在 `MX_USART1_UART_Init` 完成后调用，采用环形缓冲 + IDLE 中断模式。
 */
void DMA_UART1_Start(void)
{
  uart1_last_pos = 0U;
  HAL_UART_Receive_DMA(&huart1, uart1_dma_rx_buf, UART1_RX_DMA_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  DMA_UART1_TxInit();
}

void DMA_UART1_TxInit(void)
{
  uart1_tx_head = 0U;
  uart1_tx_tail = 0U;
  uart1_tx_dma_len = 0U;
  uart1_tx_busy = false;
  memset(uart1_dma_tx_buf, 0, sizeof(uart1_dma_tx_buf));
}
static HAL_StatusTypeDef uart1_tx_write_fifo(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0U) return HAL_OK;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint16_t free_space = uart1_tx_free_locked();
  if (free_space < len)
  {
    if (primask == 0U) __enable_irq();
    return HAL_BUSY;
  }
  uint16_t head = uart1_tx_head;
  const uint16_t capacity = UART1_TX_DMA_BUF_SIZE;
  const uint16_t first = (uint16_t)MIN(len, (uint16_t)(capacity - head));
  memcpy(&uart1_dma_tx_buf[head], data, first);
  const uint16_t remain = (uint16_t)(len - first);
  if (remain > 0U)
  {
    memcpy(&uart1_dma_tx_buf[0], data + first, remain);
    head = remain;
  }
  else
  {
    head = (uint16_t)(head + first);
  }
  uart1_tx_head = (uint16_t)(head % capacity);
  if (primask == 0U) __enable_irq();
  
  return HAL_OK;
}
HAL_StatusTypeDef DMA_UART1_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    // 如果在中断上下文中，使用简单的硬循环防止死锁
    if (__get_IPSR() != 0) 
    {
        uint32_t retry = timeout_ms * (SystemCoreClock / 10000); 
        while (retry > 0)
        {
            if (uart1_tx_write_fifo(data, len) == HAL_OK)
            {
                uart1_tx_try_start();
                return HAL_OK;
            }
            retry--;
            __NOP();
        }
        return HAL_TIMEOUT;
    }
    
    // 如果在主循环中，正常使用 HAL_GetTick()
    const uint32_t start = HAL_GetTick();
    while (true)
    {
        if (uart1_tx_write_fifo(data, len) == HAL_OK)
        {
            uart1_tx_try_start();
            return HAL_OK;
        }

        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        __NOP();
    }
}




/**
 * @brief 初始化 USART2 (PC) DMA RX + 空闲中断
 * @note  需在 `MX_USART2_UART_Init` 完成后调用，解析路径为 DMA/IDLE → protocol_feed_stream。
 */
void DMA_UART2_Start(void)
{
  uart2_last_pos = 0U;
  HAL_UART_Receive_DMA(&huart2, uart2_dma_rx_buf, UART2_RX_DMA_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  DMA_UART2_TxInit();
}

/**
 * @brief 初始化 USART2 发送 FIFO 及相关状态。
 * @note  仅重置软件缓冲，不涉及硬件寄存器。
 */
void DMA_UART2_TxInit(void)
{
  uart2_tx_head = 0U;
  uart2_tx_tail = 0U;
  uart2_tx_dma_len = 0U;
  uart2_tx_busy = false;
  memset(uart2_dma_tx_buf, 0, sizeof(uart2_dma_tx_buf));
}

static HAL_StatusTypeDef uart2_tx_write_fifo(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0U)
  {
    return HAL_OK;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint16_t free_space = uart2_tx_free_locked();
  if (free_space < len)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return HAL_BUSY;
  }

  uint16_t head = uart2_tx_head;
  const uint16_t capacity = UART2_TX_DMA_BUF_SIZE;
  const uint16_t first = (uint16_t)MIN(len, (uint16_t)(capacity - head));
  memcpy(&uart2_dma_tx_buf[head], data, first);
  const uint16_t remain = (uint16_t)(len - first);
  if (remain > 0U)
  {
    memcpy(&uart2_dma_tx_buf[0], data + first, remain);
    head = remain;
  }
  else
  {
    head = (uint16_t)(head + first);
  }
  uart2_tx_head = (uint16_t)(head % capacity);
  if (primask == 0U)
  {
    __enable_irq();
  }
  return HAL_OK;
}

/**
 * @brief 将一帧数据写入 USART2 软件 FIFO，并在有空间时启动 DMA 发送。
 * @param[in] data        待发送数据指针
 * @param[in] len         数据长度
 * @param[in] timeout_ms  在 FIFO 空间不足时的等待超时（ms）
 * @return HAL_OK      写入成功并已启动发送
 * @return HAL_TIMEOUT 在超时时间内始终无足够空间
 * @note  运行在主循环上下文，内部通过轮询 + __NOP() 方式等待，避免长时间关中断。
 */
HAL_StatusTypeDef DMA_UART2_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    // 如果在中断上下文中，使用简单的硬循环防止死锁
    if (__get_IPSR() != 0) 
    {
        uint32_t retry = timeout_ms * (SystemCoreClock / 10000); 
        while (retry > 0)
        {
            if (uart2_tx_write_fifo(data, len) == HAL_OK)
            {
                uart2_tx_try_start();
                return HAL_OK;
            }
            retry--;
            __NOP();
        }
        return HAL_TIMEOUT;
    }
    
    // 如果在主循环中，正常使用 HAL_GetTick()
    const uint32_t start = HAL_GetTick();
    while (true)
    {
        if (uart2_tx_write_fifo(data, len) == HAL_OK)
        {
            uart2_tx_try_start();
            return HAL_OK;
        }
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        __NOP();
    }
}

/**
 * @brief USART2 空闲中断入口
 * @return 无
 * @note  场景：在 `USART2_IRQHandler` 中检测到 IDLE 后调用
 */
void DMA_UART2_RxIdleHandler(void)
{
  /* USART2 空闲中断回调此函数，处理“当前批次”未消化的数据 */
  uart2_idle_isr();
}

/**
 * @brief USART1 空闲中断入口
 * @note  场景：在 `USART1_IRQHandler` 中检测到 IDLE 后调用
 */
void DMA_UART1_RxIdleHandler(void)
{
  uart1_idle_isr();
}

/**
 * @brief HAL 弱函数重载：DMA 半传输回调
 * @param huart 当前 UART 句柄
 * @return 无
 * @note  当前实现仅使用 IDLE 检测，未在半传输中搬运数据。
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
  /* 当前实现仅使用 IDLE 检测；若无需半传输中断可在 CubeMX 关闭对应 NVIC，以减少开销 */
}

/**
 * @brief HAL 弱函数重载：DMA 全传输回调
 * @param huart 当前 UART 句柄
 * @return 无
 * @note  当前实现仅使用 IDLE 检测，DMA 全传输中断仅保留缺省行为。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
  /* 当前实现仅使用 IDLE 检测；若无需全传输中断可在 CubeMX 关闭对应 NVIC */
}

/**
 * @brief HAL 回调：DMA 发送完成
 * @note  对 USART2：推进软件 FIFO 读指针并尝试发送下一段；对其他串口保留弱钩子。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART2)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uart2_tx_tail = (uint16_t)((uart2_tx_tail + uart2_tx_dma_len) % UART2_TX_DMA_BUF_SIZE);
    uart2_tx_dma_len = 0U;
    if (uart2_tx_head == uart2_tx_tail)
    {
      uart2_tx_busy = false;
      if (primask == 0U)
      {
        __enable_irq();
      }
      return;
    }
    if (primask == 0U)
    {
      __enable_irq();
    }
    uart2_tx_busy = false;
    uart2_tx_try_start();
  }
  else if (huart->Instance == USART1)
  {
    /* 解决 TODO: 释放 USART1 发送忙标志，并自动发送下一段 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uart1_tx_tail = (uint16_t)((uart1_tx_tail + uart1_tx_dma_len) % UART1_TX_DMA_BUF_SIZE);
    uart1_tx_dma_len = 0U;
    if (uart1_tx_head == uart1_tx_tail)
    {
      uart1_tx_busy = false;
      if (primask == 0U) __enable_irq();
      return;
    }
    if (primask == 0U) __enable_irq();
    uart1_tx_busy = false;
    uart1_tx_try_start();
  }
}

__weak void imu_uart_feed_stream(const uint8_t *buf, uint16_t len)
{
  (void)buf;
  (void)len;
  /* 默认不处理 IMU 串口数据，用户可在其他模块中实现该函数 */
}

HAL_StatusTypeDef DMA_UART2_TxEnqueue(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0U)
  {
    return HAL_OK;
  }

  // 仅尝试写 FIFO，不等待
  if (uart2_tx_write_fifo(data, len) != HAL_OK)
  {
    return HAL_BUSY;
  }

  // 立即 kick DMA（非阻塞）
  uart2_tx_try_start();
  return HAL_OK;
}

/* USER CODE END 2 */

