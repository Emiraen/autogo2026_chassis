//
// Created by dw on 2025/11/15.
//

#ifndef LOWER_JY901B_H
#define LOWER_JY901B_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define RXBUFFERSIZE  255     // 最大接收字节数
#define TXBUFFERSIZE  255     // 最大接收字节数
#define JY901_FRAME_LEN  (11U)

#ifndef JY901B_ENABLE_UART_FEED
/**
 * @brief 0 表示禁用 UART IMU 数据流，仅保留 I2C 路径（默认配置）。
 * @note  需要串口冗余时可在编译期覆盖为 1。
 */
#define JY901B_ENABLE_UART_FEED 0
#endif

#ifndef JY901_I2C_BLOCK_BYTES
#define JY901_I2C_BLOCK_BYTES   (32U)
#endif

typedef struct
{
    float ax;
    float ay;
    float az;
    float wx;
    float wy;
    float wz;
    float roll;
    float pitch;
    float yaw;
} JY901B_Data;

extern JY901B_Data g_jy901_data;
extern uint8_t g_jy901_raw[JY901_I2C_BLOCK_BYTES];

int checkSum(char RxBuffer[]);

/* DMA+IDLE 入口：由 dma.c 的弱符号调用 */
void imu_uart_feed_stream(const uint8_t *buf, uint16_t len);

/* 解析单帧（11 字节，0x55 + type + 8 字节数据 + sum） */
void JY901B_FeedFrame(const uint8_t frame[JY901_FRAME_LEN]);

/**
 * @brief 轮询式 I2C 读 IMU 并刷新全局姿态数据。
 * @param[in] hi2c  I2C 句柄（通常为 hi2c2）
 * @return true  读到合法数据并完成刷新；false 发生 I2C 错误
 */
bool JY901B_PollI2C(I2C_HandleTypeDef *hi2c);

/**
 * @brief 拷贝最近一次 IMU 数据快照。
 * @param[out] out  目标缓冲
 * @return true  已有有效数据；false 尚未采集到
 */
bool JY901B_GetSnapshot(JY901B_Data *out);

/**
 * @brief 返回已经成功更新的样本计数（200 Hz 任务累加）。
 */
uint32_t JY901B_GetSampleCounter(void);

/**
 * @brief 返回 I2C 读取失败计数，便于健康监控。
 */
uint32_t JY901B_GetErrorCounter(void);


#ifndef JY901_I2C_ADDR
#define JY901_I2C_ADDR          (0x50U)
#endif
#ifndef JY901_REG_START
#define JY901_REG_START         (0x30U)
#endif
#ifndef JY901_DATA_OFFSET_BYTES
#define JY901_DATA_OFFSET_BYTES (8U)
#endif
#ifndef JY901_I2C_TIMEOUT_MS
#define JY901_I2C_TIMEOUT_MS    (5U)
#endif
#endif //LOWER_JY901B_H