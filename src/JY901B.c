//
// Created by dw on 2025/11/15.
//
#include "JY901B.h"
#include <string.h>

JY901B_Data g_jy901_data = {0};
uint8_t g_jy901_raw[JY901_I2C_BLOCK_BYTES] = {0};    /**< 最近一次 I2C 读取的原始数据（0x30~0x4F，共 32B） */

#if JY901B_ENABLE_UART_FEED
static uint8_t s_frame[JY901_FRAME_LEN];
static uint8_t s_index = 0U;
#endif
static uint32_t s_sample_counter = 0U;
static uint32_t s_error_counter  = 0U;

// #define JY901_I2C_ADDR          (0x50U)   /**< HWT906/JY901 I2C 器件地址 */
// #define JY901_REG_START         (0x30U)   /**< 连续读取起始寄存器（从时间字段开始） */
// #define JY901_I2C_BLOCK_BYTES   (32U)     /**< 一次读取 0x30~0x4F 共 32 字节 */
// #define JY901_DATA_OFFSET_BYTES (8U)      /**< 前 8 字节为时间戳，IMU 数据从 AX 开始 */
// #define JY901_I2C_TIMEOUT_MS    (5U)      /**< HAL I2C 读超时，单位 ms */

/**
 * @brief 内部校验 JY901 串口帧的校验和。
 * @param[in] frame 11 字节原始帧缓存
 * @return 1 表示校验通过，-1 表示校验失败
 * @note  此函数仅在本模块内部使用，串口与 I2C 路径复用同一数据结构 g_jy901_data。
 */
static int checkSum_internal(const uint8_t frame[JY901_FRAME_LEN])
{
    uint16_t sum = 0U;
    for (uint8_t i = 0U; i < 10U; ++i)
    {
        sum += frame[i];
    }
    return (frame[10] == (uint8_t)sum) ? 1 : -1;
}

/**
 * @brief 对外暴露的校验和函数（兼容旧接口）。
 * @param[in] RxBuffer 输入帧缓存（长度至少 11 字节）
 * @return 1 表示校验通过，-1 表示校验失败
 */
int checkSum(char RxBuffer[])
{
    return checkSum_internal((const uint8_t *)RxBuffer);
}

/**
 * @brief 解析一帧 JY901 串口数据并更新全局姿态。
 * @param[in] frame 单帧原始数据（0x55 + type + 8 字节 payload + sum）
 * @note  本函数假定帧边界已对齐，仅做类型选择与数值转换。
 */
#if JY901B_ENABLE_UART_FEED
void JY901B_FeedFrame(const uint8_t frame[JY901_FRAME_LEN])
{
    if (frame[0] != 0x55U || checkSum_internal(frame) <= 0)
    {
        return;
    }

    const uint8_t type = frame[1];
    switch (type)
    {
    case 0x51: /* 加速度 */
        g_jy901_data.ax = (float)((int16_t)((frame[3] << 8) | frame[2])) / 32768.0f * 16.0f;
        g_jy901_data.ay = (float)((int16_t)((frame[5] << 8) | frame[4])) / 32768.0f * 16.0f;
        g_jy901_data.az = (float)((int16_t)((frame[7] << 8) | frame[6])) / 32768.0f * 16.0f;
        break;

    case 0x52: /* 角速度 */
        g_jy901_data.wx = (float)((int16_t)((frame[3] << 8) | frame[2])) / 32768.0f * 2000.0f;
        g_jy901_data.wy = (float)((int16_t)((frame[5] << 8) | frame[4])) / 32768.0f * 2000.0f;
        g_jy901_data.wz = (float)((int16_t)((frame[7] << 8) | frame[6])) / 32768.0f * 2000.0f;
        break;

    case 0x53: /* 欧拉角 */
        g_jy901_data.roll  = (float)((int16_t)((frame[3] << 8) | frame[2])) / 32768.0f * 180.0f;
        g_jy901_data.pitch = (float)((int16_t)((frame[5] << 8) | frame[4])) / 32768.0f * 180.0f;
        g_jy901_data.yaw   = (float)((int16_t)((frame[7] << 8) | frame[6])) / 32768.0f * 180.0f;
        break;

    default:
        /* 其他帧类型可按需扩展 */
        break;
    }
}
#else
void JY901B_FeedFrame(const uint8_t frame[JY901_FRAME_LEN])
{
    (void)frame;
}
#endif /* JY901B_ENABLE_UART_FEED */

/**
 * @brief UART + DMA IDLE 入口：从字节流拼接 JY901 帧。
 * @param[in] buf 新到达的连续字节段
 * @param[in] len 新字节数量
 * @note  典型调用路径：USART1 DMA + IDLE 中断中调用，需保证常数时间、无阻塞。
 */
#if JY901B_ENABLE_UART_FEED
void imu_uart_feed_stream(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0U; i < len; ++i)
    {
        const uint8_t byte = buf[i];

        if (s_index == 0U)
        {
            if (byte != 0x55U)
            {
                continue;
            }
            s_frame[s_index++] = byte;
            continue;
        }

        s_frame[s_index++] = byte;
        if (s_index >= JY901_FRAME_LEN)
        {
            JY901B_FeedFrame(s_frame);
            s_index = 0U;
            memset(s_frame, 0, sizeof(s_frame));
        }
    }
}
#else
void imu_uart_feed_stream(const uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
}
#endif /* JY901B_ENABLE_UART_FEED */

/**
 * @brief 将连续 2 字节（小端）转换为有符号 16 位。
 * @param[in] src 指向低字节在前的 2 字节缓冲
 * @return 转换后的 int16 数值
 */
static int16_t jy901_le_to_int16(const uint8_t *src)
{
    return (int16_t)(((uint16_t)src[1] << 8U) | src[0]);
}

/**
 * @brief 轮询式 I2C 读取 JY901 寄存器窗口并更新姿态数据。
 * @param[in] hi2c  I2C 句柄（通常为 &hi2c2）
 * @return true  读取成功且解析完成；false I2C 出错或句柄非法
 * @note  调度频率建议 200Hz 左右，运行于 App_Loop 任务上下文。
 */
bool JY901B_PollI2C(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return false;
    }

    /* 1. 通过 I2C 批量读取寄存器 0x30~0x4F，共 32 字节 */
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c,
                                                      (uint16_t)(JY901_I2C_ADDR << 1U),
                                                      JY901_REG_START,
                                                      I2C_MEMADD_SIZE_8BIT,
                                                      g_jy901_raw,
                                                      JY901_I2C_BLOCK_BYTES,
                                                      JY901_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        s_error_counter++;
        return false;
    }

    /* 2. 解析 AX 开始的 24 字节数据，采用与串口帧相同的比例换算 */
    const uint8_t *payload = &g_jy901_raw[JY901_DATA_OFFSET_BYTES];

    g_jy901_data.ax = (float)jy901_le_to_int16(&payload[0]) / 32768.0f * 16.0f;
    g_jy901_data.ay = (float)jy901_le_to_int16(&payload[2]) / 32768.0f * 16.0f;
    g_jy901_data.az = (float)jy901_le_to_int16(&payload[4]) / 32768.0f * 16.0f;

    g_jy901_data.wx = (float)jy901_le_to_int16(&payload[6]) / 32768.0f * 2000.0f;
    g_jy901_data.wy = (float)jy901_le_to_int16(&payload[8]) / 32768.0f * 2000.0f;
    g_jy901_data.wz = (float)jy901_le_to_int16(&payload[10]) / 32768.0f * 2000.0f;

    /* 跳过磁数据 6 字节（12~17），继续解析欧拉角 */
    g_jy901_data.roll  = (float)jy901_le_to_int16(&payload[18]) / 32768.0f * 180.0f;
    g_jy901_data.pitch = (float)jy901_le_to_int16(&payload[20]) / 32768.0f * 180.0f;
    g_jy901_data.yaw   = (float)jy901_le_to_int16(&payload[22]) / 32768.0f * 180.0f;

    s_sample_counter++;
    return true;
}

/**
 * @brief 拷贝最近一次 IMU 姿态快照。
 * @param[out] out 输出结构体指针，用于接收 ax/ay/az 等 9 轴数据
 * @return true  已有至少一帧有效样本；false 尚未完成任何采样
 */
bool JY901B_GetSnapshot(JY901B_Data *out)
{
    if (out == NULL)
    {
        return false;
    }

    *out = g_jy901_data;
    return (s_sample_counter > 0U);
}

/**
 * @brief 获取 I2C 成功采样计数。
 * @return 从上电以来成功刷新的样本数量
 * @note  可用于 Telemetry 任务判断是否有新样本。
 */
uint32_t JY901B_GetSampleCounter(void)
{
    return s_sample_counter;
}

/**
 * @brief 获取 I2C 读取错误计数。
 * @return HAL I2C 返回非 HAL_OK 的次数
 * @note  建议在健康监控或 OLED 页面中显示，辅助排查线缆/上拉问题。
 */
uint32_t JY901B_GetErrorCounter(void)
{
    return s_error_counter;
}
