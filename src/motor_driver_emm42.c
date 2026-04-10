/**
 * @file motor_driver_emm42.c
 * @brief Emm42_V5.0 闭环步进驱动串口控制最小骨架实现。
 *
 * 仅实现基础初始化与固定 0x6B 校验下的 Enable/StopNow 命令，
 * 便于早期联调串口连线与基本控制。其它函数可按需在后续扩展。
 */

#include "motor_driver_emm42.h"
#include <stdint.h>
#include "stm32f1xx.h"
#include "dma.h"
/**
 * @brief 内部发送一帧简单命令（固定 0x6B 校验，不等待驱动器回复）。
 *
 * 1. 按“地址 + 功能码 + 数据... + 校验”格式打包一帧命令；
 * 2. 当前仅支持固定 0x6B 校验模式（EMM42_CHECKSUM_FIXED_0x6B）；
 * 3. 通过 HAL_UART_Transmit 阻塞方式发送到指定串口。
 *
 * @param[in] handle   Emm42 句柄
 * @param[in] func     功能码（如 0xF3/0xFE 等）
 * @param[in] payload  指令数据指针，可为 NULL（长度为 0 时忽略）
 * @param[in] length   指令数据长度（不含地址、功能码与校验）
 * @return EMM42_OK 表示发送到串口成功；其他返回值表示参数或硬件错误
 *
 * @note 本函数为阻塞发送，适合在低频配置/调试路径使用，不宜在高频控制循环中调用。
 */
static Emm42_Status Emm42_SendSimpleCommand(const Emm42_Handle *handle,
                                            uint8_t func,
                                            const uint8_t *payload,
                                            uint8_t length)
{
    if (handle == NULL || handle->huart == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    if (handle->checksum_mode != EMM42_CHECKSUM_FIXED_0x6B)
    {
        /* 当前骨架仅支持固定 0x6B 校验模式 */
        return EMM42_ERR_PARAM;
    }

    /* 地址 + 功能码 + 数据... + 校验字节 */
    uint8_t buf[16U] = {0U};
    const uint8_t max_payload = (uint8_t)(sizeof(buf) - 3U); /* 预留地址/功能码/校验 */
    if (length > max_payload)
    {
        return EMM42_ERR_PARAM;
    }

    buf[0] = handle->id_addr;
    buf[1] = func;
    for (uint8_t i = 0U; i < length; ++i)
    {
        buf[2U + i] = (payload != NULL) ? payload[i] : 0U;
    }
    buf[2U + length] = 0x6BU; /* 固定校验字节 */

    const uint16_t total_len = (uint16_t)(3U + length);
     if (handle->huart->Instance == USART1)
{
     if (DMA_UART1_TxSend(buf, total_len, 2U) != HAL_OK)
     {
         return EMM42_ERR_BUSY;
     }
 }
 else
 {
     /* 兼容其他普通串口的阻塞发送 */
     if (HAL_UART_Transmit(handle->huart, buf, total_len, 10U) != HAL_OK)
     {
         return EMM42_ERR_HW;
     }
 }

    return EMM42_OK;
}

Emm42_Status Emm42_Init(Emm42_Handle *handle,
                        UART_HandleTypeDef *huart,
                        uint8_t id_addr)
{
    /**
     * @brief Emm42 句柄初始化实现。
     *
     * 1. 检查句柄与 UART 指针合法性；
     * 2. 拒绝使用地址 0（广播地址预留），要求 1–255；
     * 3. 记录 UART 句柄与驱动地址，并默认使用固定 0x6B 校验模式。
     */
    if (handle == NULL || huart == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    if (id_addr == 0U)
    {
        /* 0 作为广播地址保留，句柄中要求使用 1~255 的单机地址 */
        return EMM42_ERR_PARAM;
    }

    handle->huart = huart;
    handle->id_addr = id_addr;
    handle->checksum_mode = EMM42_CHECKSUM_FIXED_0x6B;

    return EMM42_OK;
}

Emm42_Status Emm42_SetChecksumMode(Emm42_Handle *handle,
                                   Emm42_ChecksumMode mode)
{
    /**
     * @brief 设置 Emm42 校验模式实现。
     *
     * @note 当前骨架实现仅在 EMM42_CHECKSUM_FIXED_0x6B 模式下具备发送能力，
     *       其他模式预留用于后续扩展。
     */
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    handle->checksum_mode = mode;
    return EMM42_OK;
}

Emm42_Status Emm42_Enable(Emm42_Handle *handle,
                          bool enable,
                          bool sync)
{
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    /* 功能码 F3, 子功能 AB, 后续为使能状态 + 多机同步标志 */
    uint8_t payload[3U];
    payload[0] = 0xABU;
    payload[1] = enable ? 0x01U : 0x00U;
    payload[2] = sync ? 0x01U : 0x00U;

    return Emm42_SendSimpleCommand(handle, 0xF3U, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_SetSpeed(Emm42_Handle *handle,
                            bool ccw,
                            uint16_t rpm,
                            uint8_t accel,
                            bool sync)
{
    /**
     * @brief 速度模式控制占位实现。
     *
     * @note 按说明书 F6 命令格式实现
     * 方向(1B) + 速度(2B, 大端) + 加速度(1B) + 同步标志(1B)
     */
     if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }
    uint8_t payload[5U];
    
    // 1. 方向 (0x01: CCW, 0x00: CW)
    payload[0] = ccw ? 0x01U : 0x00U;
    
    // 2. 速度 RPM (大端模式传输，高字节在前)
    payload[1] = (uint8_t)(rpm >> 8U);   // 提取高 8 位
    payload[2] = (uint8_t)(rpm & 0xFFU); // 提取低 8 位
    
    // 3. 加速度档位 (0=直接启动不加减速，1~255 为曲线加减速)
    payload[3] = accel;
    
    // 4. 多机同步标志 (0x01: 启用, 0x00: 不启用)
    payload[4] = sync ? 0x01U : 0x00U;
    // 将 0xF6 命令及打包好的 5 字节 payload 发送到底层骨架
    return Emm42_SendSimpleCommand(handle, 0xF6U, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_MoveRelative(Emm42_Handle *handle,
                                bool ccw,
                                uint16_t rpm,
                                uint8_t accel,
                               uint32_t pulses,
                               bool use_absolute,
                               bool sync)
{
    /**
     * @brief 位置模式控制占位实现。
     *
     * @note 方向(1B) + 速度(2B大端) + 加速度(1B) + 脉冲数(4B大端) + 绝对/相对标志(1B) + 同步标志(1B)
     */
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }
    uint8_t payload[10U];
    // 1. 方向 (0x01: CCW, 0x00: CW)
    payload[0] = ccw ? 0x01U : 0x00U;
    // 2. 速度 RPM (大端模式传输)
    payload[1] = (uint8_t)(rpm >> 8U);
    payload[2] = (uint8_t)(rpm & 0xFFU);
    // 3. 加速度档位
    payload[3] = accel;
    // 4. 脉冲数 (4 字节大端模式传输，高字节在前)
    payload[4] = (uint8_t)(pulses >> 24U);
    payload[5] = (uint8_t)((pulses >> 16U) & 0xFFU);
    payload[6] = (uint8_t)((pulses >> 8U) & 0xFFU);
    payload[7] = (uint8_t)(pulses & 0xFFU);
    // 5. 相对/绝对模式标志 (0x00: 相对, 0x01: 绝对)
    payload[8] = use_absolute ? 0x01U : 0x00U;
    // 6. 多机同步标志 (0x01: 启用, 0x00: 不启用)
    payload[9] = sync ? 0x01U : 0x00U;
    // 将 0xFD 命令及打包好的 10 字节 payload 发送到底层骨架
    return Emm42_SendSimpleCommand(handle, 0xFDU, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_StopNow(Emm42_Handle *handle,
                           bool sync)
{
    if (handle == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    /* 功能码 FE, 子功能 98, 后续仅多机同步标志 */
    uint8_t payload[2U];
    payload[0] = 0x98U;
    payload[1] = sync ? 0x01U : 0x00U;

    return Emm42_SendSimpleCommand(handle, 0xFEU, payload, (uint8_t)sizeof(payload));
}

Emm42_Status Emm42_SyncStart(Emm42_Handle *handle)
{
    if (handle == NULL || handle->huart == NULL)
    {
        return EMM42_ERR_PARAM;
    }

    // 广播地址必须是 0x00，功能码 0xFF，数据 0x66，固定校验 0x6B
    uint8_t buf[4] = {0x00, 0xFF, 0x66, 0x6B};
    
    if (handle->huart->Instance == USART1)
    {
        if (DMA_UART1_TxSend(buf, 4, 2U) != HAL_OK) return EMM42_ERR_BUSY;
    }
    else
    {
        if (HAL_UART_Transmit(handle->huart, buf, 4, 10U) != HAL_OK) return EMM42_ERR_HW;
    }
    return EMM42_OK;
}
